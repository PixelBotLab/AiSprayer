# -*- coding: utf-8 -*-
"""
相机增量 → 机械臂关节角 的那段数学。**纯函数**：不碰网络、不碰设备、不读配置，
所以它能在 test_follow_mirror.py 里被完整钉住，而不需要相机或机械臂在场。

口径（本模块是唯一一处声明，其它文件不再各自解释一遍）：

* `T_<a>_<b>` 表示"b 系的点表达成 a 系"的 4x4 齐次矩阵。
* follow 快照里的 `delta_r` / `delta_t_m` 是**相机相对示教位**的增量，落在**示教时的相机轴**里，
  平移单位米（C++ 内部 SI，跨界才换单位 —— 见 follow/include/follow/pose_io.hpp）。
* **本模块里所有 4x4 一律是米**。这一点是被踩出来的：`core.motion.kinematics` 的
  `forward/inverse/controller_matrix_to_urdf/get_best_ik` 全都吃米制矩阵（仓库里
  `T_gun` 的既有用法就是这样，打印 mm 时才 ×1000），而 `forward_controller` 返回的是 **mm 位姿
  向量**（Dobot 报文口径）。两者差 1000，混用的表现是"矩阵看着完全正常、IK 却一个解都没有"。
  所以：矩阵走米，只有给页面的位姿向量走 mm。
* 姿态口径：Dobot `[rx,ry,rz]` 展开成 `R = Rz(rz)·Ry(ry)·Rx(rx)`，与
  `CR5Kinematics.forward_controller` 的解析分解同一套；实现复用
  `core.handeye.pose_to_matrix`，**不在这里再写一遍欧拉**
  —— 两份"各自看着对"的欧拉代码是这条链路最贵的错误。

为什么增量要**左乘**（共轭到基座系）而不是右乘：
    用户要的是"位移和旋转的增量保持一致"。相机沿基座 X 走了 50 mm，臂就该沿基座 X 走 50 mm。
    一个在 c 系里写作 (R, t) 的运动，换到基座描述是 (R_cb·R·R_cbᵀ, R_cb·t) —— 这就是
    `delta_to_base` 干的事，然后**左乘**到基线位姿上：`T_target = Δ_base · T_baseline`。
    右乘（`T_baseline · Δ`）表达的是"沿臂自己当前工具轴动"，那是另一种物理运动：同一个相机
    平移在 home 朝下和朝前时会把臂甩向两个不同方向，与"增量一致"直接矛盾。
    注：`Δ` 只用到 `T_base_camera` 的**旋转**部分，相机装在哪儿（平移）不影响增量映射。
"""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from core.handeye import pose_to_matrix

MM_TO_M = 0.001           # 只用于把 forward_controller 的 mm 位姿向量搬进米制矩阵


def rotation_camera_to_base(T_base_camera: Sequence[Sequence[float]]) -> np.ndarray:
    """
    从 `T_base_camera` 取旋转块（相机轴 → 基座轴）。

    只取旋转：增量映射与相机装在哪无关，所以也顺带绕开了"标定文件平移是 mm、
    core.config 的 resolver 又除成 m"这个单位分叉 —— 别把它的平移喂进来再乘回去。
    正交性必须查：拿一个被截断/被改坏的矩阵去共轭，得到的是一个自洽但全错的运动。
    """
    T = np.asarray(T_base_camera, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"T_base_camera needs a 4x4 matrix, got shape {T.shape}")
    R = T[:3, :3].copy()
    err = float(np.abs(R.T @ R - np.eye(3)).max())
    if err > 1e-6 or float(np.linalg.det(R)) <= 0.0:
        raise ValueError(f"T_base_camera rotation is not orthonormal (err={err:.3e})")
    return R


def rotation_camera_to_base_fallback(rpy_deg: Sequence[float]) -> np.ndarray:
    """
    没有标定结果时的退路：用配置里写死的一组 [rx,ry,rz]（基座系下相机轴的朝向）当 R_cb。

    这是一条**降级**路径，不是等价的替代：它只对手工对齐过的装法近似成立，装歪一点
    平移方向就会偏。调用方必须把"用的是哪一个"报进状态里，别让它悄悄生效。
    """
    rpy = np.asarray(rpy_deg, dtype=np.float64).reshape(-1)
    if rpy.size < 3 or not np.all(np.isfinite(rpy[:3])):
        raise ValueError("camera_to_base_fallback euler_deg needs 3 finite values [rx,ry,rz]")
    # pose_to_matrix 只管 R = Rz·Ry·Rx 这一块，平移给 0。
    return pose_to_matrix([0.0, 0.0, 0.0, float(rpy[0]), float(rpy[1]), float(rpy[2])])[:3, :3]


def delta_to_base(R_cb: np.ndarray, delta_r: Sequence[Sequence[float]],
                  delta_t_m: Sequence[float]) -> np.ndarray:
    """
    相机系增量 → 基座系（控制器帧）的 4x4 运动量，平移**米**。

    `delta_r` 允许是 9 元行主序或 3x3；`delta_t_m` 是米。两者都来自 follow 快照，
    所以这一层根本不出现毫米 —— 少一次乘除就少一次单位口径出错的机会。
    """
    R = np.asarray(delta_r, dtype=np.float64)
    if R.shape == (9,):
        R = R.reshape(3, 3)
    if R.shape != (3, 3):
        raise ValueError(f"delta_r needs 3x3 or 9 values, got shape {np.asarray(delta_r).shape}")
    t = np.asarray(delta_t_m, dtype=np.float64).reshape(-1)
    if t.size != 3:
        raise ValueError(f"delta_t_m needs 3 values, got {t.size}")
    if not (np.all(np.isfinite(R)) and np.all(np.isfinite(t))):
        raise ValueError("camera delta contains non-finite values")

    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = R_cb @ R @ R_cb.T
    out[:3, 3] = R_cb @ t
    return out


def tcp_pose_ctrl_from_joints(kin, joints_rad: Sequence[float]) -> np.ndarray:
    """
    当前关节角 → 控制器帧 TCP 位姿 (4x4, 米)。

    走 `forward_controller`（和 follow 输出用的同一套欧拉口径），再交给 pose_to_matrix，
    所以这里不存在第二个姿态约定；translation 必须除成米 —— `controller_matrix_to_urdf`
    和 `inverse` 都在米制上工作（见模块 docstring）。
    """
    xyz_mm, rpy_deg = kin.forward_controller(list(joints_rad))
    T = pose_to_matrix([xyz_mm[0], xyz_mm[1], xyz_mm[2],
                        rpy_deg[0], rpy_deg[1], rpy_deg[2]])
    T[:3, 3] *= MM_TO_M
    return T


def pose_ctrl_from_target(kin, joints_rad: Sequence[float]) -> list[float]:
    """
    关节角 → 控制器报文位姿 [x,y,z,rx,ry,rz]（**mm/度**），给页面显示"臂要去哪儿"。

    直接返回 `forward_controller` 的结果，不绕矩阵：那是 Dobot 口径的唯一权威出处，
    在这里乘 1000 就等于再造一份单位约定。
    """
    xyz_mm, rpy_deg = kin.forward_controller([float(v) for v in joints_rad])
    return [float(v) for v in xyz_mm] + [float(v) for v in rpy_deg]


def joints_to_target(
        kin, R_cb: np.ndarray, delta_r: Sequence[Sequence[float]], delta_t_m: Sequence[float],
        baseline_rad: Sequence[float],
        nearest_to: Optional[Sequence[float]] = None) -> tuple[Optional[np.ndarray], np.ndarray, str]:
    """
    一步完整合成：返回 (目标关节角 rad 或 None, 目标 TCP 位姿 4x4（控制器帧，米）, 失败原因)。

    `baseline_rad` 是"示教那一刻"臂的关节角（启动 = home，调零 = 当时位姿），**不是**实时位姿：
    目标永远从基线 + 累计增量算出来。用实时位姿当基准会形成正反馈 —— 臂跟着自己上一次的
    误差继续走，一点点噪声就能把它推走。
    `nearest_to` 是取最近 IK 分支的参照（上一次的目标），它只影响"用哪种姿势到达"，
    不影响到达哪里。
    """
    baseline_rad = np.asarray(baseline_rad, dtype=np.float64).reshape(-1)
    if baseline_rad.size != 6:
        return None, np.eye(4), f"基线关节角需要 6 个值，收到 {baseline_rad.size} 个"
    if not np.all(np.isfinite(baseline_rad)):
        return None, np.eye(4), "基线关节角含非有限值"

    T_base = tcp_pose_ctrl_from_joints(kin, baseline_rad)
    try:
        delta = delta_to_base(R_cb, delta_r, delta_t_m)
    except ValueError as e:
        return None, T_base, str(e)

    T_target_ctrl = delta @ T_base
    # URDF 帧：kin.inverse/get_best_ik 都吃这个帧，用 controller_matrix_to_urdf 跨过去，
    # 全程不在欧拉上做任何乘法 —— 欧拉上做乘法一定会错。两个帧同为米制，故此处无单位换算。
    T_target_urdf = kin.controller_matrix_to_urdf(T_target_ctrl)

    ref = np.asarray(nearest_to if nearest_to is not None else baseline_rad, dtype=np.float64)
    best = kin.get_best_ik(T_target_urdf, ref)
    if best is None:
        # 保持上一目标由调用方负责（见 follow_service）：这里不夹位、不缩增量，
        # 否则"增量一致"这个契约就被一次静默截断破坏了。
        return None, T_target_ctrl, "ik_failed"
    if not kin.is_joint_valid(best):
        return None, T_target_ctrl, "ik_out_of_limits"
    best = np.asarray(best, dtype=np.float64).reshape(-1)
    return best, T_target_ctrl, ""
