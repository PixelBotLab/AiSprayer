# -*- coding: utf-8 -*-
"""
手眼标定求解内核 (两种安装共用)。

对外提供统一求解入口 `solve_hand_eye(mount, samples, ...)`, 由 mount 决定算法模型:
- 'eye-to-hand': 眼在手外 (标定板在手端, 相机固定)
- 'eye-in-hand': 眼在手上 (相机在手端, 标定板固定)

同时提供机器人位姿与 SE(3) 刚体变换工具函数 (如 pose_to_matrix, matrix_to_pose)。
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from .eye_in_hand import EyeInHandSolution
from .eye_in_hand import solve as solve_eye_in_hand
from .eye_to_hand import EyeToHandSolution
from .eye_to_hand import solve as solve_eye_to_hand
from .geometry import (
    DOBOT_EULER_SEQ, EYE_IN_HAND, EYE_TO_HAND, INTERNAL_ANGLE_UNIT, MIN_SAMPLES,
    MOUNTS, POSE_FRAMES, POSE_FRAME_CONTROLLER_V1, POSE_FRAME_JOINT_OFFSET_V2,
    RECOMMENDED_SAMPLES, UNIT_DEG, UNIT_RAD, average_rotation,
    chessboard_object_points, declare_pose_frame, euler_deg_from_rotation, infer_angle_unit,
    invert_transform,
    make_transform, matrix_to_pose, normalize_pose, pixel_ray_to_base, pose_frame_mismatch,
    pose_to_matrix, project_points, resolve_camera_extrinsic, resolve_result_mount,
    resolve_result_pose_frame,
    rotation_angle_deg, rotation_axis_coverage, rotation_closest_to_with_z_axis,
    rotation_from_pose,
    tool_euler_deg_from_z_axis, tool_frame_from_z_axis,
)
from .samples import (CalibSample, evaluate_data_quality, prune_outliers,
                      readout_rotation_consistency)

__all__ = [
    "CalibSample", "DOBOT_EULER_SEQ", "EYE_IN_HAND", "EYE_TO_HAND",
    "EyeInHandSolution", "EyeToHandSolution", "INTERNAL_ANGLE_UNIT",
    "MIN_SAMPLES", "MOUNTS", "POSE_FRAMES", "POSE_FRAME_CONTROLLER_V1",
    "POSE_FRAME_JOINT_OFFSET_V2", "RECOMMENDED_SAMPLES", "UNIT_DEG", "UNIT_RAD",
    "average_rotation", "assess_confidence", "chessboard_object_points",
    "declare_pose_frame", "euler_deg_from_rotation",
    "evaluate_data_quality", "extrinsic_uncertainty", "infer_angle_unit",
    "invert_transform", "make_transform", "matrix_to_pose", "minimum_samples",
    "normalize_pose", "pixel_ray_to_base", "pose_frame_mismatch", "pose_to_matrix",
    "project_points",
    "prune_outliers",
    "readout_rotation_consistency", "recommended_samples",
    "resolve_camera_extrinsic",
    "resolve_result_mount", "resolve_result_pose_frame",
    "rotation_angle_deg", "rotation_axis_coverage",
    "rotation_closest_to_with_z_axis",
    "rotation_from_pose", "solve_hand_eye", "solution_extrinsic",
    "tool_euler_deg_from_z_axis", "tool_frame_from_z_axis",
]


def solve_hand_eye(mount: str, samples: Sequence[CalibSample],
                   K: Optional[np.ndarray] = None,
                   D: Optional[Sequence[float]] = None):
    """按安装方式求解手眼外参, 失败返回 None。"""
    if mount == EYE_TO_HAND:
        return solve_eye_to_hand(samples, K=K, D=D)
    if mount == EYE_IN_HAND:
        return solve_eye_in_hand(samples, K=K, D=D)
    raise ValueError(f"unknown hand-eye mount '{mount}', expected one of {MOUNTS}")


def minimum_samples(mount: str) -> int:
    """该安装方式可求解的绝对下界样本数。"""
    return MIN_SAMPLES.get(mount, 3)


def recommended_samples(mount: str) -> int:
    """该安装方式达到稳定精度的建议样本数。"""
    return RECOMMENDED_SAMPLES.get(mount, 12)


# 两种装法各自的"相机相对安装基座"待标定外参键名 (写入结果文件与读取共用一处)。
_EXTRINSIC_KEYS: Dict[str, str] = {
    EYE_TO_HAND: "T_base_camera",
    EYE_IN_HAND: "T_flange_camera",
}

# 留一法的成本 = 重解次数 x 单次求解耗时; 眼在手外每次还要跑欧拉顺规网格搜索,
# 样本上百时会把接口卡到分钟级。超过该上限就按下标等间隔抽样剔除。
_MAX_JACKKNIFE_FITS = 20


def solution_extrinsic(mount: str, solution: Any) -> np.ndarray:
    """取解里的相机外参矩阵: 眼在手外是 T_base_camera, 眼在手上是 T_flange_camera。"""
    return getattr(solution, _EXTRINSIC_KEYS[mount])


def extrinsic_uncertainty(mount: str,
                          samples: Sequence[CalibSample],
                          K: Optional[np.ndarray] = None,
                          D: Optional[Sequence[float]] = None) -> Optional[Dict[str, Any]]:
    """
    留一法 (leave-one-out jackknife) 估计外参能被这批数据定到多准。

    重投影误差只说明"样本之间有多自洽", 说明不了外参本身的不确定度: 眼在上的
    T_flange_camera 靠法兰旋转差分求解, 未标定机械臂 0.5~1° 的绝对姿态误差会被
    相机到标定板的杆臂放大, 表现为"残差不大但 X 整体可以浮动好几毫米"。这里每次
    剔除一个样本重解, 用 n 次解的离散度当标准误, 直接告诉操作员这份标定能不能用。

    两个口径必须固定, 否则数字没有可比性:
    1. **走像素域精修路径** (传入 K/D): 闭式 AX=XB 路径对"手坐标系原点放在法兰还是放在
       tool 尖"并不协变 —— 实测同一批数据两种口径的旋转不变散布差 4~13 倍, 拿它当
       绝对门槛会随示教器 tool 设置漂移; 而精修路径实测两口径逐位一致。
       无角点/无内参时退化到闭式路径, 并在 `refined=False` 里标出来。
    2. **平移用旋转不变量** (三分量合成范数, 不是 max(分量)): 分量会随法兰坐标系绕常旋转
       重排, 拿 max(分量) 比阈值等于把阈值绑在 tool 取向上。逐轴值仍随 `axes_mm` 返回供展示。

    旋转不确定度用各次解到均值旋转的 geodesic 夹角 (度), 避开欧拉角 ±180° 回绕。
    """
    n = len(samples)
    if n < minimum_samples(mount) + 2:
        return None

    # 样本多时等间隔抽样剔除 (确定性), 单次解的离散度仍按全样本数 n 换算成标准误。
    stride = -(-n // _MAX_JACKKNIFE_FITS)
    fits = []
    for i in range(0, n, stride):
        subset = [s for j, s in enumerate(samples) if j != i]
        sol = solve_hand_eye(mount, subset, K=K, D=D)
        if sol is None:
            continue
        T = np.asarray(solution_extrinsic(mount, sol), dtype=np.float64)
        fits.append((T[:3, 3], T[:3, :3]))
    m = len(fits)
    if m < minimum_samples(mount):
        return None

    t_all = np.array([f[0] for f in fits], dtype=np.float64)
    t_mean = t_all.mean(axis=0)
    # 经典留一法标准误: se² = ((n-1)²/n) · s², s² 为各次留一解的样本方差 (无偏, 除 m-1)。
    scale = (n - 1.0) ** 2 / n / max(m - 1, 1)
    se_axes = np.sqrt(scale * np.sum((t_all - t_mean) ** 2, axis=0))
    se_t = float(np.sqrt(np.sum(se_axes ** 2)))

    r_mean = average_rotation([f[1] for f in fits])
    angles = np.array([rotation_angle_deg(r_mean, f[1]) for f in fits])
    se_r = float(np.sqrt(scale * np.sum((angles - angles.mean()) ** 2)))

    return {
        "translation_mm": round(se_t, 3),
        "axes_mm": [round(float(v), 3) for v in se_axes],
        "rotation_deg": round(se_r, 4),
        "fits": m,
        "refined": bool(K is not None),
    }


# 一份外参能不能上机使用的门槛。量级依据: 本系统相机 fx≈611px, 标定板作业距离
# 300~900mm, 故 1px ≈ 0.5~1.5mm; 喷涂轨迹要求外参误差在 1~2mm 量级。
#
# 下面的实测锚点全部用**像素域精修 + 旋转不变范数**的新口径重测 (见
# extrinsic_uncertainty 注释), 三份真实会话的数值:
#   E2H calib_20260812_230221 (已投用, 27 帧 / 2.83px): ±2.84 mm / ±0.132 deg,
#       转角一致性 0.285 median / 0.804 p90 / 1.412 max over 322 pairs -> CAUTION
#   EIH calib_20261002_125721 (29 帧 / 12.12px):        ±10.80 mm / ±0.572 deg,
#       一致性 0.431 median / 2.153 p90 -> NOT_USABLE
#   EIH calib_20261002_102239 (22 帧 / 7.35px):         ±284 mm / ±80 deg
#       (留一重解跳到另一个分支 = 这批数据定不住外参), 一致性 0.558 median / 1.737 p90
# 旧口径 (闭式解 + max 分量) 在同样数据上给的是 ±3.7mm / ±0.54° 与 ±14.7mm / ±9.4°
# (取决于 tool 怎么配), 所以阈值不能沿用历史值。平移上限取 5mm: 卡在 3mm 会把当前
# 实际投用的 E2H (2.84mm) 判死, 而 1px 在 600mm 深度上已经是 1mm 量级。
_WARN_REPROJ_PX = 1.5
_USABLE_REPROJ_PX = 4.0
_WARN_STD_MM = 1.5
_USABLE_STD_MM = 5.0
_WARN_STD_DEG = 0.2
_USABLE_STD_DEG = 1.0
# 臂读数与视觉观测的相对转角失配 (度): 失配是位姿链路本身的矛盾, 不是采样布局问题,
# 无论怎么重采都解不出可信外参, 所以它可以单独判不可用。基准是已投用的 E2H:
# 0.285° median / 0.804° p90 / 1.412° max。
_WARN_CONSISTENCY_MED_DEG = 0.3
_BLOCK_CONSISTENCY_MED_DEG = 1.0
_BLOCK_CONSISTENCY_P90_DEG = 1.5


def assess_confidence(mount: str, quality: Dict[str, Any],
                      uncertainty: Optional[Dict[str, Any]],
                      reprojection_error_px: Optional[float],
                      consistency: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    把误差数字折算成“这份标定能不能用”的结论与英文告警清单 (界面只负责显示, 判定只在这一处)。

    重投影误差只说明样本之间多自洽, 不等于外参准; 所以必须同时看留一法不确定度、旋转轴
    覆盖度与臂读数一致性。grade: OK / CAUTION / NOT_USABLE。
    """
    warnings: List[str] = []
    blocking: List[str] = []

    if quality.get("degenerate"):
        blocking.append(
            f"Rotation axes are nearly parallel (coverage "
            f"{float(quality.get('axis_coverage', 0.0)):.2f} < 0.30): the extrinsic is "
            f"not observable. Re-sample with the flange rotated about clearly different axes."
        )
    elif float(quality.get("rotation_span_deg", 0.0)) < 30.0:
        warnings.append(
            f"Rotation span is only {float(quality.get('rotation_span_deg', 0.0)):.1f} deg. "
            f"Move the board through larger angles for a better conditioned solve."
        )

    if reprojection_error_px is None:
        warnings.append("No chessboard corner pixels available, reprojection error is unknown.")
    elif float(reprojection_error_px) > _USABLE_REPROJ_PX:
        blocking.append(
            f"Reprojection error {float(reprojection_error_px):.2f} px exceeds the "
            f"{_USABLE_REPROJ_PX:.1f} px limit."
        )
    elif float(reprojection_error_px) > _WARN_REPROJ_PX:
        warnings.append(
            f"Reprojection error {float(reprojection_error_px):.2f} px is above the "
            f"{_WARN_REPROJ_PX:.1f} px target."
        )

    if uncertainty is None:
        warnings.append(
            f"Too few samples to estimate the extrinsic uncertainty for '{mount}' "
            f"(leave-one-out needs more than {minimum_samples(mount) + 2})."
        )
    else:
        std_t = float(uncertainty.get("translation_mm", 0.0))
        std_r = float(uncertainty.get("rotation_deg", 0.0))
        if not uncertainty.get("refined", True):
            warnings.append(
                "Extrinsic uncertainty was estimated without corner pixels (closed-form path); "
                "the value depends on which frame the hand pose is expressed in."
            )
        over_t = std_t > _USABLE_STD_MM
        over_r = std_r > _USABLE_STD_DEG
        if over_t:
            blocking.append(
                f"Extrinsic uncertainty is +/-{std_t:.1f} mm, above the "
                f"+/-{_USABLE_STD_MM:.1f} mm limit. The camera offset is not repeatable "
                f"with this data."
            )
        if over_r:
            blocking.append(
                f"Extrinsic orientation uncertainty is +/-{std_r:.2f} deg, above the "
                f"+/-{_USABLE_STD_DEG:.1f} deg limit."
            )
        # 已经触发阻断的就不再多说一条“偏高”: 同一个数字只说一次。
        if not (over_t or over_r) and (std_t > _WARN_STD_MM or std_r > _WARN_STD_DEG):
            warnings.append(
                f"Extrinsic uncertainty +/-{std_t:.1f} mm, +/-{std_r:.2f} deg "
                f"(leave-one-out estimate)."
            )

    if consistency is not None:
        med = float(consistency.get("median_deg", 0.0))
        p90 = float(consistency.get("p90_deg", 0.0))
        worst = consistency.get("worst_pair", "")
        detail = (f"worst {worst}: the arm reports {float(consistency.get('worst_arm_deg', 0.0)):.2f} deg "
                  f"while the camera sees {float(consistency.get('worst_vision_deg', 0.0)):.2f} deg")
        if med > _BLOCK_CONSISTENCY_MED_DEG or p90 > _BLOCK_CONSISTENCY_P90_DEG:
            blocking.append(
                f"Robot pose readout contradicts the camera: relative rotations disagree by "
                f"+/-{med:.2f} deg (median) / +/-{p90:.2f} deg (p90) across "
                f"{consistency.get('pairs', 0)} pairs, {detail}. The pose chain itself is "
                f"untrustworthy, so no extrinsic can be calibrated from this data. Check the "
                f"camera bracket, the board fixation and the arm's kinematic calibration."
            )
        elif med > _WARN_CONSISTENCY_MED_DEG:
            warnings.append(
                f"Arm readout vs camera disagree by +/-{med:.2f} deg (median), {detail}. "
                f"Expect a limited accuracy floor from the pose chain."
            )

    if blocking:
        grade, usable = "NOT_USABLE", False
    elif warnings:
        grade, usable = "CAUTION", True
    else:
        grade, usable = "OK", True

    return {"grade": grade, "usable": usable, "warnings": warnings, "blocking": blocking}
