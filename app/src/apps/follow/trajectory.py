# -*- coding: utf-8 -*-
"""
JointTrajectorySmoother：把稀疏的 IK keyframe（poll_hz ≈ 20 Hz）抹成稠密的发射流（33 Hz）。

为什么要有它：
  * 页面仿真臂消费的 `joints_deg` 如果只在"有新解算"时才变，臂就是 20 Hz 阶梯跳；
    静止时跟随噪声还会让每个台阶都带一点抖动。
  * 解算失败/坏帧被拦下时，目标会整段停住 —— 发射层不该把"停住"也变成一次跳变。

**为什么是"追踪式"而不是"分段式"（踩过的坑，务必保留）：**
  第一版照搬 waypoints 的分段余弦缓动：每个新 keyframe 重起一段，段时长由限速决定，
  段内余弦缓动。这在 waypoints（少量、稀疏的路点）是对的，在 follow 是错的：
  keyframe 以 ~20 Hz 稠密到达，每段只有 ~50 ms，余弦缓动在**每一段的两个端点角速度
  都是 0** —— 臂的运动变成 20 Hz 的"加速-减速-加速-减速"，速度脉动肉眼可见，
  实测比不做平滑的旧版还要顿。稠密目标流的正确形态是速度连续：输出位带着速度
  持续追踪最新目标，目标不动才阻尼刹停（钉位）。

当前设计（逐关节一阶追踪 + 限速钳制）：
  * 每个 tick：目标速度 v_cmd = clamp(k·(q_target − q_out), ±max_vel)；
    实际速度向 v_cmd 一阶收敛（时间常数 tau = 1/k）后按速度积分。
  * k（带宽）取大值：目标以 20 Hz 变化时跟得住，不产生可见的相位拖尾；
    目标停住时指数刹停，到 1e-4 rad 内直接钉位（静止发射流 = 常数，无抖动）。
  * 中途来新目标：不重规划、不清速度 —— 输出位带着当前速度直接转向，
    速度曲线连续，没有段边界的"顿"。
  * 限速是硬钳制：任何时刻 |dq_out/dt| ≤ max_vel。

单位：内部全弧度；跨界才换度 —— 与 mirror / pose_io 的规矩一致。
线程约定：一个生产者（轮询线程）调 push_target，一个消费者（发射线程）调 step ——
两者不同线程，但各自的调用序列内部是串行的；类内不再加锁（调用方各自在自己的锁里换数据）。
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np


class JointTrajectorySmoother:
    def __init__(self, max_joint_vel_rad_s: float = math.radians(90.0),
                 gain: float = 20.0, pin_tol_rad: float = 1e-4) -> None:
        # 限速是安全侧参数：夹到正数，0/负数会让速度钳制失去意义。
        self.max_vel = max(float(max_joint_vel_rad_s), 1e-3)
        # k = 1/tau：误差→目标速度的带宽。太小会拖尾（目标 20 Hz 变化时臂滞后可见，
        # 实测 k=12（τ≈83ms）时用户已觉得"滞后很多"），太大则速度在 ±max_vel 之间
        # 硬切换（接近 bang-bang，又回到顿挫）。20 对应 50 ms 时间常数：33 Hz 下约 2 拍
        # 收敛，滞后压到半拍以内，同时速度仍连续。
        self.k = max(float(gain), 0.1)
        self.pin_tol = max(float(pin_tol_rad), 1e-6)
        self.q_out: Optional[np.ndarray] = None        # 最近一次发射出去的关节角（rad）
        self.q_target: Optional[np.ndarray] = None     # 最新 keyframe
        self.v: Optional[np.ndarray] = None            # 当前发射速度（rad/s），跨目标保持
        self._pinned = False                           # 已钉位（目标静止且已刹停）

    # ------------------------------------------------------------ 生产者侧
    def push_target(self, q_target: np.ndarray) -> bool:
        """
        收到一个新的 keyframe。返回 True = 目标被接受。
        第一次调用同时把输出位定下来 —— 此前没有"现在在哪儿"，追踪无从谈起。
        目标真的变了就解除钉位：带着当前速度（静止时为 0）开始追踪。
        """
        q = np.asarray(q_target, dtype=np.float64)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            return False
        if self.q_out is None:
            self.q_out = q.copy()
            self.v = np.zeros(6)
        changed = self.q_target is None or not np.allclose(q, self.q_target, atol=1e-9)
        self.q_target = q.copy()
        if changed:
            self._pinned = False
        return True

    def reset(self) -> None:
        """停止跟随时调用：下一次启动是全新的一段，旧的速度与目标不属于新基线。"""
        self.q_out = None
        self.q_target = None
        self.v = None
        self._pinned = False

    # ------------------------------------------------------------ 消费者侧
    def step(self, dt_s: float) -> Optional[np.ndarray]:
        """
        前进 dt 秒，返回该发出去的关节角（rad）。没有目标/没有输出位时返回 None：
        发射层拿到 None 就不发新 joints（页面保持上一显示），而不是发一个 0 向量。
        """
        if self.q_out is None:
            return None
        if self.q_target is None:
            return self.q_out.copy()
        if self._pinned:
            return self.q_out.copy()          # 钉位：静止发射流 = 常数

        dt = max(float(dt_s), 0.0)
        err = self.q_target - self.q_out
        # 目标速度：比例追踪 + 硬限速。方向突变时 v_cmd 连续变化（经过 0），不跳变。
        v_cmd = np.clip(self.k * err, -self.max_vel, self.max_vel)
        # 实际速度向目标速度一阶收敛：这就是速度连续性的来源 —— 段与目标之间没有断崖。
        alpha = 1.0 - math.exp(-self.k * dt)
        self.v = self.v + (v_cmd - self.v) * alpha
        # 钳制兜底：一阶收敛本身不保证逐拍不越限（v 从反向大值收敛时会过冲穿越），
        # 限速是安全契约，这里再兜一道。
        self.v = np.clip(self.v, -self.max_vel, self.max_vel)
        self.q_out = self.q_out + self.v * dt

        # 刹停判据：贴近目标且速度已经收小 ⇒ 精确落位并钉住。
        # 不钉住的话指数尾巴会把静止发射流变成一串越来越小的抖动值。
        if float(np.max(np.abs(self.q_target - self.q_out))) < self.pin_tol and \
                float(np.max(np.abs(self.v))) < self.pin_tol * self.k:
            self.q_out = self.q_target.copy()
            self.v = np.zeros(6)
            self._pinned = True
        return self.q_out.copy()

    @property
    def moving(self) -> bool:
        """正在追踪（未钉位）。用于日志统计"平滑器在干活的时长"。"""
        return self.q_target is not None and not self._pinned

    # ------------------------------------------------------------ 兼容旧接口
    @property
    def seg_dur(self) -> float:
        """旧分段实现的遗物：测试与外部若问"段时长"，追踪式下无段的概念，恒 0。"""
        return 0.0
