# -*- coding: utf-8 -*-
"""
眼在手上 (eye-in-hand) 求解器: 相机装在法兰上, 标定板固定于世界。

约束模型 (对每个样本 i 成立):
    T_base_flange(i) · T_flange_camera · T_camera_board(i) = T_base_board
其中 T_flange_camera (常量 X) 与 T_base_board (常量 Z) 为未知量。

求解流程:
  1. 用 OpenCV 的 AX=XB 家族 (Tsai / Park / Andreff-Daniel / Horaud / Daniilidis)
     解出 X。五个方法各有各的退化域, 全部跑一遍再按残差择优, 而不是硬编码选一个。
  2. 把 X 代回约束式, 对全部样本求 Z 的均值, 得到标定板在基座系下的位姿。
     Z 是眼在手上独有的有用输出: 它把"相机看到的工件"锚定到机器人基座系,
     眼在手外里对应角色的是 chessboard_offset (那里是板装在法兰上)。
  3. 以闭式解为初值, 在像素域对 (X, Z) 做 Levenberg-Marquardt 精修 —— AX=XB 最小化的
     是刚体方程的代数残差, 不等价于角点重投影误差, 直接用像素目标还能压掉一截误差。
  4. 用真·重投影误差 (像素) 择优并报告, 而不是用平移残差冒充重投影误差。

退化告警: AX=XB 的可观测性完全来自法兰旋转轴方向的变化。若样本近似绕同一轴转动,
X 沿该轴的分量不可观测, 求解器仍会返回一个数值上"收敛"的错误解, 因此必须检查
axis_coverage 而不是只看残差。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

from .geometry import (
    average_rotation, invert_transform, make_transform, project_points,
    rotation_angle_deg, rotation_axis_coverage,
)
from .samples import CalibSample

# 尝试顺序, 最终按残差择优。
AX_XB_METHODS: Tuple[str, ...] = ("tsai", "park", "andreff", "horaud", "daniilidis")

_MIN_AXIS_COVERAGE = 0.30

# 像素域精修至少需要这么多带角点的样本, 否则 12 个自由度的优化会自己骗自己。
_MIN_REFINE_SAMPLES = 6

_OPENCV_METHOD_CONSTANTS: Dict[str, int] = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


@dataclass
class EyeInHandSolution:
    T_flange_camera: np.ndarray
    T_base_board: np.ndarray
    reprojection_error_px: Optional[float]
    translation_error_mm: float
    rotation_error_deg: float
    method: str
    axis_coverage: float
    degenerate: bool
    method_report: List[Dict[str, object]] = field(default_factory=list)
    refined: bool = False
    # 逐样本角点重投影误差 (px), 键是样本在传入列表里的下标; 无角点/无内参时为空。
    # 给上层的残差裁剪用, 与上面的 reprojection_error_px 同口径。
    per_sample_reprojection_px: Dict[int, float] = field(default_factory=dict)


def _board_pose_in_base(samples: Sequence[CalibSample],
                        T_flange_camera: np.ndarray) -> List[np.ndarray]:
    """每个样本独立推出的标定板基座位姿 U_i · X · V_i, 理论上应彼此相等。"""
    return [s.T_base_flange @ T_flange_camera @ s.T_camera_board for s in samples]


def _average_transform(T_list: Sequence[np.ndarray]) -> np.ndarray:
    """平移取均值, 旋转取均值后 SVD 正交化回 SO(3)。"""
    R = average_rotation([T[:3, :3] for T in T_list])
    t_avg = np.mean([T[:3, 3] for T in T_list], axis=0)
    return make_transform(R, t_avg)


def per_sample_reprojection_px(samples: Sequence[CalibSample], T_flange_camera: np.ndarray,
                              T_base_board: np.ndarray, K: Optional[np.ndarray],
                              D: Optional[Sequence[float]]) -> Dict[int, float]:
    """
    逐样本角点重投影误差 (px), 键为样本在传入列表中的下标。

    拿最终解 (X, Z) 反推每个样本应当看到的板角点, 投影回像素与实测比较。没角点的
    样本直接缺席结果。_residuals 与上层的残差裁剪共用这一处, 保证“上报的重投影
    误差”与“用来剔样本的误差”是同一个口径。
    """
    out: Dict[int, float] = {}
    if K is None:
        return out
    for i, s in enumerate(samples):
        if s.obj_pts is None or s.corners_px is None:
            continue
        # U_i · X · V_i = Z  =>  V_i = inv(U_i · X) · Z
        T_camera_board_pred = invert_transform(s.T_base_flange @ T_flange_camera) @ T_base_board
        projected = project_points(s.obj_pts, T_camera_board_pred, K, D)
        out[i] = float(np.mean(np.linalg.norm(
            projected - s.corners_px.reshape(-1, 2), axis=1)))
    return out


def _residuals(samples: Sequence[CalibSample], T_flange_camera: np.ndarray,
               K: Optional[np.ndarray] = None,
               D: Optional[Sequence[float]] = None,
               T_base_board: Optional[np.ndarray] = None) -> Tuple[float, float, Optional[float]]:
    """
    返回 (平移残差 mm, 旋转残差 deg, 重投影残差 px 或 None)。

    T_base_board 缺省时取各样本反推板位姿的均值 (衡量样本间自洽度); 传入具体 Z 时,
    重投影误差按该解计算 —— 精修后的 X 与它的板位姿必须同口径上报。
    """
    T_board_list = _board_pose_in_base(samples, T_flange_camera)
    if T_base_board is None:
        T_base_board = _average_transform(T_board_list)

    t_errs = [float(np.linalg.norm(T[:3, 3] - T_base_board[:3, 3])) for T in T_board_list]
    r_errs = [rotation_angle_deg(T[:3, :3], T_base_board[:3, :3]) for T in T_board_list]

    errs = per_sample_reprojection_px(samples, T_flange_camera, T_base_board, K, D)
    reproj = float(np.mean(list(errs.values()))) if errs else None

    return float(np.mean(t_errs)), float(np.mean(r_errs)), reproj


def _pack(T: np.ndarray) -> np.ndarray:
    """4x4 齐次矩阵 -> [旋转向量(3, rad), 平移(3, mm)]，优化器的 6 自由度参数化。"""
    return np.concatenate([Rot.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]])


def _unpack(p: np.ndarray) -> np.ndarray:
    """_pack 的逆。"""
    return make_transform(Rot.from_rotvec(p[:3]).as_matrix(), p[3:6])


def _pixel_residual(p: np.ndarray, samples: Sequence[CalibSample],
                    K: np.ndarray, D: Optional[Sequence[float]]) -> np.ndarray:
    """残差向量: 每个样本把约定的板位姿 inv(U·X)·Z 投影回像素, 与实测角点求差。"""
    X, Z = _unpack(p[:6]), _unpack(p[6:12])
    chunks = []
    for s in samples:
        T_camera_board = invert_transform(s.T_base_flange @ X) @ Z
        chunks.append((project_points(s.obj_pts, T_camera_board, K, D)
                       - s.corners_px).ravel())
    return np.concatenate(chunks)


def _refine_reprojection(samples: Sequence[CalibSample], X: np.ndarray, Z: np.ndarray,
                         K: Optional[np.ndarray], D: Optional[Sequence[float]]
                         ) -> Optional[Tuple[np.ndarray, np.ndarray, float, float, float]]:
    """
    以闭式解为初值, 在像素域对 (X, Z) 联合做 Levenberg-Marquardt 最小二乘精修。

    AX=XB 解的是刚体方程的代数残差, 并不等价于角点的像素误差; 而且 X 的姿态靠法兰
    旋转差分约定, 未标定机械臂 0.5~1° 的绝对姿态误差会被“相机到标定板”的杆臂
    (常 300~600mm) 放大成毫米级平移残差。直接以重投影像素为目标重解, 是把误差
    压下去最省事也最对的一步。

    返回 (X, Z, 平移残差 mm, 旋转残差 deg, 重投影残差 px); 无内参/角点不足/优化失败时返回 None。
    平移/旋转残差是“各样本反推板位姿绕均值的离散度”, 按定义以均值为参考; 重投影残差
    则对准优化器自己解出的 Z —— 否则上报的像素误差与写进结果文件的矩阵不是同一个解
    (实测差 1.7px)。
    """
    if K is None:
        return None
    usable = [s for s in samples if s.obj_pts is not None and s.corners_px is not None]
    if len(usable) < _MIN_REFINE_SAMPLES:
        return None

    p0 = np.concatenate([_pack(X), _pack(Z)])
    try:
        # 旋转以 rad、平移以 mm 参数化, 量纲差两个数量级, 必须给 x_scale 否则数值条件差
        res = least_squares(_pixel_residual, p0, args=(usable, K, D),
                            x_scale=[0.01] * 6 + [1.0] * 6, max_nfev=200)
    except Exception:
        return None
    if not np.isfinite(res.x).all():
        return None

    X_new = _unpack(res.x[:6])
    Z_new = _unpack(res.x[6:12])
    t_err, r_err, _ = _residuals(samples, X_new, K, D)
    _, _, reproj = _residuals(samples, X_new, K, D, T_base_board=Z_new)
    if reproj is None or not np.isfinite([t_err, r_err, reproj]).all():
        return None
    return X_new, Z_new, t_err, r_err, reproj


def solve(samples: Sequence[CalibSample],
          K: Optional[np.ndarray] = None,
          D: Optional[Sequence[float]] = None) -> Optional[EyeInHandSolution]:
    """跑遍 AX=XB 各方法, 按重投影残差 (无角点时退化为平移残差) 择优并精修。"""
    if len(samples) < 3:
        return None

    R_g2b = [np.ascontiguousarray(s.T_base_flange[:3, :3]) for s in samples]
    t_g2b = [np.ascontiguousarray(s.T_base_flange[:3, 3:4]) for s in samples]
    R_t2c = [np.ascontiguousarray(s.T_camera_board[:3, :3]) for s in samples]
    t_t2c = [np.ascontiguousarray(s.T_camera_board[:3, 3:4]) for s in samples]

    candidates: List[Tuple[float, str, np.ndarray, float, float, Optional[float]]] = []
    report: List[Dict[str, object]] = []

    for name in AX_XB_METHODS:
        entry: Dict[str, object] = {"method": name}
        try:
            R_c2g, t_c2g = cv2.calibrateHandEye(
                R_g2b, t_g2b, R_t2c, t_t2c, _OPENCV_METHOD_CONSTANTS[name])
        except Exception as exc:
            entry["status"] = f"failed: {type(exc).__name__}"
            report.append(entry)
            continue

        X = make_transform(R_c2g, np.asarray(t_c2g, dtype=np.float64).reshape(3))
        if not np.isfinite(X).all():
            entry["status"] = "non-finite"
            report.append(entry)
            continue

        t_err, r_err, reproj = _residuals(samples, X, K, D)
        entry.update({
            "status": "ok",
            "translation_error_mm": round(t_err, 4),
            "rotation_error_deg": round(r_err, 4),
        })
        if reproj is not None:
            entry["reprojection_error_px"] = round(reproj, 4)
        report.append(entry)

        score = reproj if reproj is not None else t_err
        candidates.append((score, name, X, t_err, r_err, reproj))

    if not candidates:
        return None

    candidates.sort(key=lambda c: c[0])
    score, name, X, t_err, r_err, reproj = candidates[0]

    T_base_board = _average_transform(_board_pose_in_base(samples, X))

    refined = _refine_reprojection(samples, X, T_base_board, K, D)
    is_refined = bool(refined and (reproj is None or refined[4] <= reproj))
    if is_refined:
        X, T_base_board, t_err, r_err, reproj = refined

    coverage = rotation_axis_coverage([s.rotation for s in samples])

    return EyeInHandSolution(
        T_flange_camera=X,
        T_base_board=T_base_board,
        reprojection_error_px=reproj,
        translation_error_mm=t_err,
        rotation_error_deg=r_err,
        method=name,
        axis_coverage=coverage,
        degenerate=bool(coverage < _MIN_AXIS_COVERAGE),
        method_report=report,
        refined=is_refined,
        per_sample_reprojection_px=per_sample_reprojection_px(samples, X, T_base_board, K, D),
    )
