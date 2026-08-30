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
  3. 用真·重投影误差 (像素) 择优并报告, 而不是用平移残差冒充重投影误差。

退化告警: AX=XB 的可观测性完全来自法兰旋转轴方向的变化。若样本近似绕同一轴转动,
X 沿该轴的分量不可观测, 求解器仍会返回一个数值上"收敛"的错误解, 因此必须检查
axis_coverage 而不是只看残差。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .geometry import (
    invert_transform, make_transform, project_points, rotation_angle_deg,
    rotation_axis_coverage,
)
from .samples import CalibSample

try:
    import cv2 as _cv2
except ImportError:  # pragma: no cover - cv2 是本项目的硬依赖
    _cv2 = None

# 尝试顺序, 最终按残差择优。
AX_XB_METHODS: Tuple[str, ...] = ("tsai", "park", "andreff", "horaud", "daniilidis")

_MIN_AXIS_COVERAGE = 0.30


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


def _opencv_method_constants() -> Dict[str, int]:
    if _cv2 is None:
        return {}
    return {
        "tsai": _cv2.CALIB_HAND_EYE_TSAI,
        "park": _cv2.CALIB_HAND_EYE_PARK,
        "andreff": _cv2.CALIB_HAND_EYE_ANDREFF,
        "horaud": _cv2.CALIB_HAND_EYE_HORAUD,
        "daniilidis": _cv2.CALIB_HAND_EYE_DANIILIDIS,
    }


def _board_pose_in_base(samples: Sequence[CalibSample],
                        T_flange_camera: np.ndarray) -> List[np.ndarray]:
    """每个样本独立推出的标定板基座位姿 U_i · X · V_i, 理论上应彼此相等。"""
    return [s.T_base_flange @ T_flange_camera @ s.T_camera_board for s in samples]


def _average_transform(T_list: Sequence[np.ndarray]) -> np.ndarray:
    """平移取均值, 旋转取均值后 SVD 正交化回 SO(3)。"""
    R_avg = np.mean([T[:3, :3] for T in T_list], axis=0)
    U, _, Vt = np.linalg.svd(R_avg)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    t_avg = np.mean([T[:3, 3] for T in T_list], axis=0)
    return make_transform(R, t_avg)


def _residuals(samples: Sequence[CalibSample], T_flange_camera: np.ndarray,
               K: Optional[np.ndarray] = None,
               D: Optional[Sequence[float]] = None) -> Tuple[float, float, Optional[float]]:
    """返回 (平移残差 mm, 旋转残差 deg, 重投影残差 px 或 None)。"""
    T_board_list = _board_pose_in_base(samples, T_flange_camera)
    T_base_board = _average_transform(T_board_list)

    t_errs = [float(np.linalg.norm(T[:3, 3] - T_base_board[:3, 3])) for T in T_board_list]
    r_errs = [rotation_angle_deg(T[:3, :3], T_base_board[:3, :3]) for T in T_board_list]

    reproj = None
    if K is not None:
        usable = [s for s in samples if s.obj_pts is not None and s.corners_px is not None]
        if usable:
            errs = []
            for s in usable:
                # U_i · X · V_i = Z  =>  V_i = inv(U_i · X) · Z
                T_camera_board_pred = invert_transform(s.T_base_flange @ T_flange_camera) @ T_base_board
                projected = project_points(s.obj_pts, T_camera_board_pred, K, D)
                errs.append(float(np.mean(np.linalg.norm(
                    projected - s.corners_px.reshape(-1, 2), axis=1))))
            reproj = float(np.mean(errs)) if errs else None

    return float(np.mean(t_errs)), float(np.mean(r_errs)), reproj


def solve(samples: Sequence[CalibSample],
          K: Optional[np.ndarray] = None,
          D: Optional[Sequence[float]] = None) -> Optional[EyeInHandSolution]:
    """跑遍 AX=XB 各方法, 按重投影残差 (无角点时退化为平移残差) 择优。"""
    if _cv2 is None:
        raise RuntimeError("cv2 is required for eye-in-hand calibration")
    if len(samples) < 3:
        return None

    R_g2b = [np.ascontiguousarray(s.T_base_flange[:3, :3]) for s in samples]
    t_g2b = [np.ascontiguousarray(s.T_base_flange[:3, 3:4]) for s in samples]
    R_t2c = [np.ascontiguousarray(s.T_camera_board[:3, :3]) for s in samples]
    t_t2c = [np.ascontiguousarray(s.T_camera_board[:3, 3:4]) for s in samples]

    constants = _opencv_method_constants()
    candidates: List[Tuple[float, str, np.ndarray, float, float, Optional[float]]] = []
    report: List[Dict[str, object]] = []

    for name in AX_XB_METHODS:
        entry: Dict[str, object] = {"method": name}
        try:
            R_c2g, t_c2g = _cv2.calibrateHandEye(
                R_g2b, t_g2b, R_t2c, t_t2c, constants[name])
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
    )
