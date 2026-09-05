# -*- coding: utf-8 -*-
"""
标定样本模型与数据质量评估 (两种安装共用)。

两种安装共用同一套清洗与质量评估, 因为它们的运动学约束是同一条: 相机观测到的
标定板位移, 必须与机器人法兰位移在刚体运动学的允许范围内一致。
  eye-to-hand: 标定板随法兰运动, 静止相机观测其位移;
  eye-in-hand: 标定板固定, 相机随法兰运动, 板在相机系下的观测反向平移。
差别只在旋转轴覆盖度的重要性: AX=XB 的可观测性让眼在手上的旋转多样性权重更高。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from .geometry import DOBOT_EULER_SEQ, rotation_angle_deg, rotation_axis_coverage


@dataclass
class CalibSample:
    """一次采样的完整记录: 机器人法兰位姿 + 相机看到的标定板位姿 + 原始观测。"""

    sample_id: int
    T_base_flange: np.ndarray
    T_camera_board: np.ndarray
    pose_dobot: Optional[np.ndarray] = None
    corners_px: Optional[np.ndarray] = None
    image_file: str = ""
    joints_deg: Optional[Sequence[float]] = None
    pose_diag_deg: Optional[float] = None
    obj_pts: Optional[np.ndarray] = field(default=None, repr=False)

    @property
    def flange_xyz(self) -> np.ndarray:
        return self.T_base_flange[:3, 3]

    @property
    def flange_euler(self) -> np.ndarray:
        """控制器原始欧拉角读数 (度)。网格搜索顺规时需要它, 矩阵已经丢失了参数化信息。"""
        if self.pose_dobot is None:
            return Rot.from_matrix(self.rotation).as_euler(DOBOT_EULER_SEQ, degrees=True)
        return np.asarray(self.pose_dobot, dtype=np.float64)[3:6]

    @property
    def board_in_camera(self) -> np.ndarray:
        """标定板原点 (板系零点) 在相机系下的坐标, 即 solvePnP 的 tvec。"""
        return self.T_camera_board[:3, 3]

    @property
    def rotation(self) -> np.ndarray:
        return self.T_base_flange[:3, :3]


def clean_samples(samples: Sequence[CalibSample],
                  threshold: float = 0.05,
                  motion_envelope_mm: float = 500.0,
                  min_reliable_dist_mm: float = 10.0,
                  log_callback=None) -> List[CalibSample]:
    """
    剔除相机观测位移与机器人运动学不一致的异常样本 (两种安装共用)。

    判据不再直接用 d_c / d_r ≈ 1: 那只对"纯平移"成立。刚体上偏离法兰原点的点,
    在法兰转动 θ 时额外移动最多 2·sin(θ/2)·|t|, 其中 |t| 是该点到法兰原点的距离
    (眼在手外是板相对法兰的 TCP 偏移, 眼在手上是板相对法兰的作用距离)。老实现因此
    把旋转丰富的样本一律误判为异常, 只能用 0.80~1.20 的经验宽容带打补丁。

    这里用三角不等式给出随转角自动张开的一致区间:
        |d_r - d_rot| · (1-threshold)  <=  d_c  <=  (d_r + d_rot) · (1+threshold)
    既能容忍真实的大角度样本, 又能抓住角点检测错乱这类粗差。
    """
    if not samples:
        return []

    def log(msg: str) -> None:
        if log_callback:
            log_callback(msg)

    base = samples[0]
    kept = [base]
    log(f"  [KEEP] Sample {base.sample_id}: Reference Base")

    for s in samples[1:]:
        d_r = float(np.linalg.norm(s.flange_xyz - base.flange_xyz))
        d_c = float(np.linalg.norm(s.board_in_camera - base.board_in_camera))

        if d_r < min_reliable_dist_mm:
            kept.append(s)
            continue

        theta = np.radians(rotation_angle_deg(base.rotation, s.rotation))
        d_rot = 2.0 * abs(np.sin(theta / 2.0)) * motion_envelope_mm

        lo = max(0.0, d_r - d_rot) * (1.0 - threshold)
        hi = (d_r + d_rot) * (1.0 + threshold)

        if lo <= d_c <= hi:
            kept.append(s)
            log(f"  [KEEP] Sample {s.sample_id}: observed {d_c:.1f}mm vs kinematic "
                f"{d_r:.1f}mm (+/-{d_rot:.1f}mm rotation term)")
        else:
            log(f"  [DROP] Sample {s.sample_id}: observed {d_c:.1f}mm outside kinematic "
                f"band [{lo:.1f}, {hi:.1f}]mm")
    return kept


def evaluate_data_quality(samples: Sequence[CalibSample], mount: str) -> dict:
    """
    评估样本的空间与姿态多样性, 输出 0~100 评分与退化告警。

    eye-to-hand 靠平移与旋转共同激励; eye-in-hand 额外受 AX=XB 可观测性约束,
    旋转轴方向覆盖度不足时解不唯一, 因此对旋转分量权重更高并单独暴露 axis_coverage。
    """
    if not samples:
        return {"score": 0.0, "axis_coverage": 0.0, "rotation_span_deg": 0.0,
                "translation_span_mm": 0.0, "degenerate": True}

    pos = np.array([s.flange_xyz for s in samples], dtype=np.float64)
    rot = [s.rotation for s in samples]

    eulers = np.array([Rot.from_matrix(R).as_euler("xyz", degrees=True) for R in rot])

    ptp_xyz = np.ptp(pos, axis=0)
    ptp_abc = np.ptp(eulers, axis=0)
    translation_span = float(np.mean(ptp_xyz))
    rotation_span = float(np.mean(ptp_abc))

    p_score = min(1.0, translation_span / 300.0)
    r_score = min(1.0, rotation_span / 30.0)
    weights = (0.4, 0.6) if mount == "eye-to-hand" else (0.25, 0.75)
    score = (p_score * weights[0] + r_score * weights[1]) * 100.0

    coverage = rotation_axis_coverage(rot)
    degenerate = coverage < 0.30
    return {
        "score": float(score),
        "axis_coverage": float(coverage),
        "rotation_span_deg": rotation_span,
        "translation_span_mm": translation_span,
        "degenerate": bool(degenerate),
    }
