# -*- coding: utf-8 -*-
"""
标定样本模型与数据质量评估 (两种安装共用)。

两种安装共用同一套质量评估: 样本的空间/姿态多样性决定解的可观测性, 而刚体约束
U·X·V = Z 本身就是免费的一致性校验 —— 相机观测与机械臂位姿读数只要互相矛盾,
再怎么优化都解不出可信外参。
  eye-to-hand: 标定板随法兰运动, 静止相机观测其位移;
  eye-in-hand: 标定板固定, 相机随法兰运动, 板在相机系下的观测反向平移。
差别只在旋转轴覆盖度的重要性: AX=XB 的可观测性让眼在手上的旋转多样性权重更高。

粗差裁剪必须在**首轮拟合之后**做 (prune_outliers): 拿拟合前的运动学包络做三角
不等式时, 区间要按“法兰到板的作用距离”张开 (眼在上 300~600mm), 对真实采样偏差
完全不敏感, 实测一批 12px 的数据一帧都剔不掉。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from .geometry import DOBOT_EULER_SEQ, rotation_angle_deg, rotation_axis_coverage

# 两帧之间至少要转这么多度, 这一对才拿来判定一致性: 相对转角接近 0 时, 旋转轴方向
# 本身是数值病态的, 零点几度的差值没有物理意义。
_MIN_PAIR_ROTATION_DEG = 5.0


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


def readout_rotation_consistency(samples: Sequence[CalibSample]) -> Optional[dict]:
    """
    机械臂笛卡尔姿态读数与相机观测是否互相矛盾 (两种装法共用, 不依赖任何待标定外参)。

    原理: 刚体约束 U_i·X·V_i = Z (X 为外参常量, Z 为该装法下的常量位姿) 展开即
        V_i⁻¹·V_j = Z⁻¹·(U_i⁻¹·U_j)·Z
    右边是左边的共轭, 而**共轭不改变旋转角** —— 所以任意两帧之间, “臂说自己转了多少度”
    与“相机看到板转了多少度”必须严格相等。该等式与 X、Z、tool 偏置、基座/用户坐标系
    约定全部无关, 是唯一不需要先标定就能做的判决实验。

    差值明显大于零说明位姿链路本身不可信 (法兰姿态读数不准 / 相机在支架上微动 / 标定板
    被碰过), 这批数据怎么解都解不出好外参, 应当直接拒收而不是送去重解。

    返回 None 表示样本不足以成对比较; `per_sample_deg` 按样本号给出每个样本参与过的
    最大失配, 供界面逐行标红。
    """
    usable = [s for s in samples
              if s.T_base_flange is not None and s.T_camera_board is not None]
    if len(usable) < 2:
        return None

    diffs: List[float] = []
    per_sample: Dict[int, float] = {}
    worst = (0.0, usable[0].sample_id, usable[1].sample_id, 0.0, 0.0)
    for i in range(len(usable) - 1):
        for j in range(i + 1, len(usable)):
            a, b = usable[i], usable[j]
            arm = rotation_angle_deg(a.T_base_flange[:3, :3], b.T_base_flange[:3, :3])
            vis = rotation_angle_deg(a.T_camera_board[:3, :3], b.T_camera_board[:3, :3])
            if max(arm, vis) < _MIN_PAIR_ROTATION_DEG:
                continue
            d = abs(arm - vis)
            diffs.append(d)
            for sid in (a.sample_id, b.sample_id):
                per_sample[sid] = max(per_sample.get(sid, 0.0), d)
            if d > worst[0]:
                worst = (d, a.sample_id, b.sample_id, arm, vis)
    if not diffs:
        return None

    arr = np.sort(np.array(diffs, dtype=np.float64))
    return {
        "median_deg": round(float(np.median(arr)), 3),
        "p90_deg": round(float(arr[min(int(0.9 * len(arr)), len(arr) - 1)]), 3),
        "max_deg": round(float(arr[-1]), 3),
        "pairs": int(len(arr)),
        "worst_pair": f"#{worst[1]}<->#{worst[2]}",
        "worst_arm_deg": round(float(worst[3]), 3),
        "worst_vision_deg": round(float(worst[4]), 3),
        "per_sample_deg": {int(k): round(float(v), 2) for k, v in per_sample.items()},
    }


def prune_outliers(samples: Sequence[CalibSample],
                   errors: Sequence[Optional[float]],
                   max_error: float,
                   unit: str,
                   min_keep: int,
                   max_drop_ratio: float = 0.25,
                   log_callback=None) -> Tuple[List[CalibSample], List[int]]:
    """
    按“每个样本自己的拟合残差”裁剪粗差, 返回 (保留样本, 被剔样本号)。

    用的已经是拟合出的解对每个样本的残差, 判据比任何先验包络都直接: 拿运动学包络做
    三角不等式时, 区间必须按“法兰到板的作用距离”张开 (眼在上 300~600mm), 对真实采样
    偏差完全不敏感 —— 实测一批 12px 的数据一帧都剔不掉。因此必须在首轮拟合之后做。

    只重解一次, 不做迭代剪枝 —— 按残差反复剔除会把可观测性一起剪掉 (旋转轴覆盖度急剧下降)。

    只能剪少数粗差, 不能拿它整改整批坏数据:
    - 残差为 None 的样本 (无角点可比较) 不判不剪: 没有证据就不能当粗差处理;
    - 没人超阈、或保留数低于 min_keep 时不剪;
    - 超阈样本超过 max_drop_ratio 比例时也不剪 —— 那是整批系统性不自洽 (位姿链路或采样
      布局问题), 剪到凑巧能拟合是造假结果而不是修好结果, 应当让判决说 NOT_USABLE。
      实测一批 29 帧平均 12px 的数据按 8px 上限会剪掉 21 帧, 剩下 8 帧旋转轴退化,
      重解反而发到 88px / 米级不确定度。
    """
    if len(errors) != len(samples):
        raise ValueError(f"errors ({len(errors)}) must align with samples ({len(samples)})")

    def log(msg: str) -> None:
        if log_callback:
            log_callback(msg)

    keep_idx = [i for i, e in enumerate(errors) if e is None or float(e) <= max_error]
    dropped_n = len(samples) - len(keep_idx)
    if dropped_n == 0:
        return list(samples), []
    if len(keep_idx) < min_keep:
        log(f"  [SKIP] {dropped_n} sample(s) exceed the {max_error:.2f}{unit} limit, but only "
            f"{len(keep_idx)} would remain (below the {min_keep} needed to solve); keeping all")
        return list(samples), []
    if dropped_n > max_drop_ratio * len(samples):
        log(f"  [SKIP] {dropped_n}/{len(samples)} samples exceed the {max_error:.2f}{unit} "
            f"limit: the batch is systematically inconsistent, pruning would remove the "
            f"evidence instead of the blunder; keeping all and reporting the verdict")
        return list(samples), []

    kept: List[CalibSample] = []
    dropped: List[int] = []
    for i, s in enumerate(samples):
        if i in keep_idx:
            kept.append(s)
        else:
            dropped.append(s.sample_id)
            log(f"  [DROP] Sample {s.sample_id}: own residual {float(errors[i]):.2f}{unit} "
                f"exceeds the {max_error:.2f}{unit} limit")
    return kept, dropped


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
