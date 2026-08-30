# -*- coding: utf-8 -*-
"""
手眼标定的坐标系约定与 SE(3) 基础运算。

两种安装的数学模型（本项目统一用 T_<a>_<b> 表示 "b 系中的点表达成 a 系" 的变换）：

  eye-to-hand (眼在手外): 相机固定于基座，标定板刚性装在法兰上
      标定板在基座系下的位姿随法兰运动: P_board_base(i) = P_base_flange(i) + R_base_flange(i) · t_off
      未知量 = T_base_camera (常量) + t_off (板相对法兰的 TCP 偏移)

  eye-in-hand (眼在手上): 相机固定于法兰，标定板固定于世界
      T_base_flange(i) · T_flange_camera · T_camera_board(i) = T_base_board
      未知量 = T_flange_camera (常量, 即 AX=XB 的 X) + T_base_board

所有平移单位一律为毫米 (mm)，与标定板 square_size_mm / solvePnP 输出一致；
换算到米由下游 core.config 负责。
"""
from __future__ import annotations

from typing import Any, Dict, List, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as Rot

EYE_TO_HAND = "eye-to-hand"
EYE_IN_HAND = "eye-in-hand"
MOUNTS: tuple[str, str] = (EYE_TO_HAND, EYE_IN_HAND)

# Dobot 控制器回报的 [x, y, z, rx, ry, rz] 中姿态部分满足
#   R = Rz(rz) · Ry(ry) · Rx(rx)      (定系下的矩阵乘序；等价于 scipy 内禀序列 'xyz')
# 注意别把"内禀 xyz"读成 Rx·Ry·Rz —— 内禀是绕**动**轴依次转 x→y→z，写成定系矩阵乘积时
# 顺序反过来。该对应关系已用 CR5Kinematics.forward_controller 逐轴随机姿态实测，与
# Rotation.from_matrix(T_ctrl).as_euler('xyz', degrees=True) 完全一致 (<1e-15)。
# 老版求解器靠 12 顺规 x 8 符号网格搜索"猜"出这个约定, 现在直接确定化,
# 网格搜索仅作为异常控制器固件的兜底保留在 eye_to_hand 里。
DOBOT_EULER_SEQ = "xyz"

UNIT_DEG = "deg"
UNIT_RAD = "rad"

# 本包内部一律以"度"为姿态单位。注意仓库其它层把机器人位姿存成弧度
# (dobot_driver.get_current_pose 把 Dobot 的度 math.radians 了一下), 所以
# 写入 session 时必须显式声明来源单位, 由 normalize_pose 负责换算。
# 旧版靠 evaluate_data_diversity 里 "max(|rpy|) < 7.0 即视为弧度" 的幅值启发式
# 猜测单位, 姿态恰好接近 0 时会误判成度, 这里改为显式记录。
INTERNAL_ANGLE_UNIT = UNIT_DEG

# 各安装方式的最少样本数。AX=XB 理论下界是 3, 但眼在手上对旋转多样性极其敏感,
# 低于 5 个样本时旋转退化到无唯一解, 因此硬性提高门槛。
MIN_SAMPLES: Dict[str, int] = {EYE_TO_HAND: 3, EYE_IN_HAND: 5}
RECOMMENDED_SAMPLES: Dict[str, int] = {EYE_TO_HAND: 12, EYE_IN_HAND: 20}

_POSE_KEYS = ("x", "y", "z")
_ROT_KEYS = (("rx", "a"), ("ry", "b"), ("rz", "c"))

PoseLike = Any  # dict | Sequence[float] | np.ndarray


def infer_angle_unit(samples: Sequence[dict]) -> str:
    """
    为未记录 pose_angle_unit 的历史 session 猜测姿态单位。

    仅在读取旧数据时作为兜底: 欧拉角绝对值全部落在 (-2π, 2π) 内时按弧度处理,
    否则按度处理。新数据一律以 session yaml 里的 pose_angle_unit 为准。
    """
    values = []
    for s in samples:
        pose = s.get("robot_pose") or s.get("pose") or {}
        for pair in _ROT_KEYS:
            for key in pair:
                if key in pose and pose[key] is not None:
                    values.append(abs(float(pose[key])))
                    break
    if not values:
        return UNIT_DEG
    return UNIT_RAD if max(values) <= 2.0 * np.pi else UNIT_DEG


def normalize_pose(pose: PoseLike, angle_unit: str = UNIT_DEG) -> np.ndarray:
    """把任意位姿表示归一成 [x, y, z, rx, ry, rz]，平移 mm、姿态一律为度。

    兼容三世代数据: 新版 rx/ry/rz、旧版 a/b/c、以及裸 6 元列表。
    """
    if isinstance(pose, dict):
        xyz = [float(pose.get(k, 0.0)) for k in _POSE_KEYS]
        rpy = []
        for pair in _ROT_KEYS:
            for key in pair:
                if key in pose and pose[key] is not None:
                    rpy.append(float(pose[key]))
                    break
            else:
                rpy.append(0.0)
        arr = np.array(xyz + rpy, dtype=np.float64)
    else:
        arr = np.asarray(pose, dtype=np.float64).reshape(-1)
        if arr.size < 6:
            raise ValueError(f"pose needs at least 6 values [x,y,z,rx,ry,rz], got {arr.size}")
        arr = arr[:6].copy()

    if angle_unit == UNIT_RAD:
        arr[3:] = np.degrees(arr[3:])
    return arr


def rotation_from_pose(pose: PoseLike, angle_unit: str = UNIT_DEG) -> np.ndarray:
    """取位姿的 3x3 旋转 (Dobot 内禀 'xyz')。"""
    rpy = normalize_pose(pose, angle_unit)[3:]
    return Rot.from_euler(DOBOT_EULER_SEQ, rpy, degrees=True).as_matrix()


def pose_to_matrix(pose: PoseLike, angle_unit: str = UNIT_DEG) -> np.ndarray:
    """Dobot 位姿 -> 4x4 齐次矩阵 (mm, 度)。"""
    v = normalize_pose(pose, angle_unit)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rot.from_euler(DOBOT_EULER_SEQ, v[3:], degrees=True).as_matrix()
    T[:3, 3] = v[:3]
    return T


def matrix_to_pose(T: np.ndarray, angle_unit: str = UNIT_DEG) -> List[float]:
    """4x4 齐次矩阵 -> Dobot 位姿 [x, y, z, rx, ry, rz] (mm, 度或弧度)。"""
    T = np.asarray(T, dtype=np.float64)
    rpy = Rot.from_matrix(T[:3, :3]).as_euler(DOBOT_EULER_SEQ, degrees=True)
    if angle_unit == UNIT_RAD:
        rpy = np.radians(rpy)
    return [float(x) for x in T[:3, 3]] + [float(x) for x in rpy]


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """刚性逆变换, 避免调用 np.linalg.inv 带来的数值噪声。"""
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    Rt = T[:3, :3].T
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rt
    out[:3, 3] = -Rt @ T[:3, 3]
    return out


def project_points(obj_pts: np.ndarray, T_camera_obj: np.ndarray,
                   K: np.ndarray, D: Sequence[float] | None = None) -> np.ndarray:
    """把目标系下的三维点投影到像素, 用于真正的重投影误差。

    未标定畸变时 (D 为空或长度不足) 退化为针孔投影, 与 cv2.projectPoints 无畸变分支一致。
    """
    import cv2

    T = np.asarray(T_camera_obj, dtype=np.float64).reshape(4, 4)
    pts = np.asarray(obj_pts, dtype=np.float64).reshape(-1, 3)
    in_cam = T[:3, :3] @ pts.T + T[:3, 3:4]
    dist = np.asarray(D, dtype=np.float64).reshape(-1) if D is not None else np.zeros(5)
    if dist.size < 4:
        dist = np.pad(dist, (0, 5 - dist.size))
    out, _ = cv2.projectPoints(
        in_cam.T.astype(np.float64), np.zeros(3), np.zeros(3),
        np.asarray(K, dtype=np.float64).reshape(3, 3), dist,
    )
    return out.reshape(-1, 2)


def rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """两个旋转矩阵之间的角位移 (度)。"""
    R_diff = np.asarray(R_a, dtype=np.float64).T @ np.asarray(R_b, dtype=np.float64)
    trace_val = (np.trace(R_diff) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(trace_val, -1.0, 1.0))))


def average_rotation(R_list: Sequence[np.ndarray]) -> np.ndarray:
    """旋转矩阵的均值并重投影回 SO(3) (SVD 正交化, 去漂移)。"""
    avg = np.mean(np.asarray(R_list, dtype=np.float64), axis=0)
    U, _, Vt = np.linalg.svd(avg)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def chessboard_object_points(pattern_size: Sequence[int], square_size_mm: float) -> np.ndarray:
    """按 OpenCV 惯例生成棋盘格三维模型点 (Z=0 平面, 单位 mm)。"""
    cols, rows = int(pattern_size[0]), int(pattern_size[1])
    objp = np.zeros((rows * cols, 3), dtype=np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return objp * float(square_size_mm)


def sample_board_points(T_camera_board: np.ndarray, objp: np.ndarray) -> np.ndarray:
    """标定板角点从板系变换到相机系, 即 solvePnP 观测到的三维位置。"""
    T = np.asarray(T_camera_board, dtype=np.float64).reshape(4, 4)
    return (T[:3, :3] @ np.asarray(objp).T + T[:3, 3:4]).T


def rotation_axis_coverage(R_list: Sequence[np.ndarray]) -> float:
    """
    旋转轴方向覆盖度, 用来诊断 AX=XB 是否退化。

    AX=XB 的可观测性来自旋转轴方向的变化: 若所有样本绕同一轴旋转, X 沿该轴的分量
    完全不可观测, 求解器照样会返回一个"收敛"的错误解。这里取每个样本相对首样本的
    旋转轴, 构造它们两两之间夹角的倒数均值: 数值越接近 0 表示轴越分散, 越可靠。
    返回值是覆盖度 (0~1, 越大越好)。
    """
    axes = []
    R0 = np.asarray(R_list[0], dtype=np.float64)
    for R in R_list[1:]:
        rotvec = Rot.from_matrix(R0.T @ np.asarray(R, dtype=np.float64)).as_rotvec()
        norm = float(np.linalg.norm(rotvec))
        if norm < np.radians(5.0):
            continue
        axes.append(rotvec / norm)

    if len(axes) < 2:
        return 0.0

    sin_angles = []
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            sin_angles.append(float(np.linalg.norm(np.cross(axes[i], axes[j]))))
    if not sin_angles:
        return 0.0
    return float(np.mean(sin_angles))
