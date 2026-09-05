# -*- coding: utf-8 -*-
"""
眼在手外 (eye-to-hand) 求解器。

模型: 相机静止于基座, 标定板刚性装在法兰上。
  P_board_base(i) = P_base_flange(i) + R_base_flange(i) · t_off
  P_board_base(i) = R_base_camera · P_board_camera(i) + t_base_camera
两式联立, 交替求解:
  1. 固定 t_off, 用 Kabsch/SVD 把 {P_board_base} 刚体配准到 {P_board_camera} 得 R_base_camera;
  2. 固定 R_base_camera, 用最小二乘解 t_base_camera 与 t_off。

保留 12 顺规 x 8 符号的网格搜索作为控制器欧拉角惯例兜底: 首选序列命中且残差
足够小时会提前退出, 避免 RK3588 上 96 次全量求解的开销。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from .geometry import (
    DOBOT_EULER_SEQ, average_rotation, make_transform, project_points,
    rotation_angle_deg,
)
from .samples import CalibSample

_EULER_ORDERS = ("xyz", "ZYX", 'XYZ', 'zyx', "YXZ", "YZX", "ZXY", "XZY",
                 'xzy', 'yxz', 'yzx', 'zxy')
_SIGN_VECTORS = ((1, 1, 1), (1, 1, -1), (1, -1, 1), (-1, 1, 1),
                 (1, -1, -1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1))

# 提前退出阈值: 平均平移残差已小于该值 (mm) 说明当前顺规就是正确约定。
_EARLY_EXIT_MM = 1.0
# 不符合物理常识的 TCP 偏移上限, 超过判定为发散或局部极小。
_MAX_T_OFF_MM = 500.0


@dataclass
class EyeToHandSolution:
    T_base_camera: np.ndarray
    board_offset_flange_mm: np.ndarray
    board_rotation_flange: np.ndarray
    translation_error_mm: float
    rotation_error_deg: float
    reprojection_error_px: Optional[float]
    euler_order: str
    sign_vector: Tuple[int, int, int]
    per_sample_errors_mm: List[float]


def _rotations(samples: Sequence[CalibSample], order: str, signs) -> Optional[List[np.ndarray]]:
    """按候选顺规与符号, 从控制器原始欧拉角读数构造法兰旋转矩阵。"""
    out = []
    for s in samples:
        angles = s.flange_euler * np.array(signs, dtype=np.float64)
        try:
            out.append(Rot.from_euler(order, angles, degrees=True).as_matrix())
        except Exception:
            return None
    return out


def _solve_once(samples: Sequence[CalibSample], R_flange: List[np.ndarray],
                iterations: int = 15) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, List[float]]]:
    """给定一组法兰旋转, 交替优化求 (R_base_camera, t_base_camera, t_off, 逐样本残差)。"""
    P_robot = np.array([s.flange_xyz for s in samples], dtype=np.float64)
    P_cam = np.array([s.board_in_camera for s in samples], dtype=np.float64)

    t_off = np.zeros(3)
    R_base_cam = np.eye(3)
    t_base_cam = np.zeros(3)

    for _ in range(iterations):
        P_board_base = P_robot + np.array([R @ t_off for R in R_flange])

        cA = np.mean(P_board_base, axis=0)
        cB = np.mean(P_cam, axis=0)
        H = (P_board_base - cA).T @ (P_cam - cB)
        U, _, Vt = np.linalg.svd(H)
        R_cam_base = Vt.T @ U.T
        if np.linalg.det(R_cam_base) < 0:
            Vt[2, :] *= -1
            R_cam_base = Vt.T @ U.T
        R_base_cam = R_cam_base.T

        A_rows, B_rows = [], []
        for i in range(len(samples)):
            A_rows.append(np.hstack([np.eye(3), -R_flange[i]]))
            B_rows.append(P_robot[i] - R_base_cam @ P_cam[i])
        res, _, _, _ = np.linalg.lstsq(np.vstack(A_rows), np.hstack(B_rows), rcond=None)
        t_base_cam = res[:3]
        t_off = res[3:]

    if np.linalg.norm(t_off) > _MAX_T_OFF_MM or not np.isfinite(t_off).all():
        return None

    errs = []
    for i in range(len(samples)):
        p_board_base = P_robot[i] + R_flange[i] @ t_off
        p_cam_pred = R_cam_base @ (p_board_base - t_base_cam)
        errs.append(float(np.linalg.norm(p_cam_pred - P_cam[i])))

    R_base_cam = np.asarray(R_base_cam, dtype=np.float64)
    t_base_cam = np.asarray(t_base_cam, dtype=np.float64)
    return make_transform(R_base_cam, t_base_cam), t_base_cam, t_off, errs


def solve(samples: Sequence[CalibSample],
          K: Optional[np.ndarray] = None,
          D: Optional[Sequence[float]] = None) -> Optional[EyeToHandSolution]:
    """网格搜索最优欧拉角顺规/符号, 求解眼在手外外参。"""
    if len(samples) < 3:
        return None

    best = None
    for order in _EULER_ORDERS:
        for signs in _SIGN_VECTORS:
            R_flange = _rotations(samples, order, signs)
            if R_flange is None:
                continue
            solved = _solve_once(samples, R_flange)
            if solved is None:
                continue
            T_base_cam, t_base_cam, t_off, errs = solved
            mean_err = float(np.mean(errs))
            if best is None or mean_err < best[0]:
                best = (mean_err, order, signs, T_base_cam, t_base_cam, t_off, errs, R_flange)
            if mean_err < _EARLY_EXIT_MM and order == DOBOT_EULER_SEQ and signs == (1, 1, 1):
                break
        if best and best[0] < _EARLY_EXIT_MM:
            break

    if best is None:
        return None

    mean_err, order, signs, T_base_cam, t_base_cam, t_off, errs, R_bt_list = best

    R_flange_board, rotation_error = _mean_board_rotation_residual(samples, T_base_cam, R_bt_list)

    reproj_px = None
    if K is not None:
        reproj_px = _reprojection_error_px(samples, T_base_cam, t_off, R_bt_list, R_flange_board, K, D)

    return EyeToHandSolution(
        T_base_camera=T_base_cam,
        board_offset_flange_mm=np.asarray(t_off, dtype=np.float64),
        board_rotation_flange=R_flange_board,
        translation_error_mm=mean_err,
        rotation_error_deg=rotation_error,
        reprojection_error_px=reproj_px,
        euler_order=order,
        sign_vector=tuple(int(x) for x in signs),
        per_sample_errors_mm=errs,
    )


def _mean_board_rotation_residual(
    samples: Sequence[CalibSample], T_base_camera: np.ndarray,
    R_bt_list: List[np.ndarray],
) -> Tuple[np.ndarray, float]:
    """
    法兰到标定板的相对姿态 T_fb = T_flange_base · T_base_camera · T_camera_board
    应当与样本无关: 其均值即板在法兰系的安装旋转 (含面外倾角), 各样本到均值的
    角位移均值即旋转标定精度 (度)。
    """
    T_fb_list = []
    for s, R_bt in zip(samples, R_bt_list):
        T_bt = s.T_base_flange.copy()
        T_bt[:3, :3] = R_bt
        T_fb_list.append((np.linalg.inv(T_bt) @ T_base_camera @ s.T_camera_board)[:3, :3])

    R_fb = average_rotation(T_fb_list)
    return np.asarray(R_fb, dtype=np.float64), float(np.mean(
        [rotation_angle_deg(R, R_fb) for R in T_fb_list]))


def _reprojection_error_px(samples: Sequence[CalibSample], T_base_camera: np.ndarray,
                           t_off: np.ndarray, R_bt_list: List[np.ndarray],
                           R_flange_board: np.ndarray, K: np.ndarray,
                           D: Optional[Sequence[float]]) -> Optional[float]:
    """
    真·重投影误差: 由标定结果反推板角点在相机系的三维位置, 投影回像素后与实测角点比较。
    """
    usable = [(s, R_bt) for s, R_bt in zip(samples, R_bt_list)
              if s.obj_pts is not None and s.corners_px is not None]
    if not usable:
        return None

    errs = []
    for s, R_bt in usable:
        T_board_base = make_transform(R_bt @ R_flange_board, s.flange_xyz + R_bt @ t_off)
        T_camera_board = np.linalg.inv(T_base_camera) @ T_board_base
        projected = project_points(s.obj_pts, T_camera_board, K, D)
        errs.append(float(np.mean(np.linalg.norm(
            projected - s.corners_px.reshape(-1, 2), axis=1))))
    return float(np.mean(errs)) if errs else None
