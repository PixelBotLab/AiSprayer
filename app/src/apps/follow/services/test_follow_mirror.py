# -*- coding: utf-8 -*-
"""
`mirror.py` + `follow_service.py` 的无相机、无臂回归。

跑法：
    cd app/src && python3 -m apps.follow.services.test_follow_mirror

钉住的是一条"错了会静默自洽"的链路，所以断言全部用**独立来源**算期望值，不用被测函数自己
校自己：

* 欧拉口径：手写 `Rz(rz)·Ry(ry)·Rx(rx)` 与 `pose_to_matrix` 对账（这份代码里唯一一处会因
  为一条写歪的注释而被改错的地方）。
* 增量映射：`R_cb = RotZ(90°)` 时相机 X 平移必须落在基座 Y 上 —— 轴映射错一位，页面看起来
  仍然是"在跟"，只是方向不对，肉眼最难发现。
* 保持上一目标：IK 失败后 `_target_q` 必须**逐位相等**，不是"接近"。
* 三条真机护栏：`mode: real` 必须拒绝；相机服务连不上必须置起 `camera_service_down`；
  调零在没有当前位姿时必须失败（不能偷偷用 home 当基线把臂瞬移走）。
"""
from __future__ import annotations

import math
import os
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from apps.follow.mirror import (  # noqa: E402
    delta_to_base, joints_to_target, pose_ctrl_from_target, rotation_camera_to_base,
    rotation_camera_to_base_fallback, tcp_pose_ctrl_from_joints,
)
from apps.follow.services import follow_service as fs_mod  # noqa: E402
from apps.follow.services.pose_stream import PoseStream  # noqa: E402
from apps.follow.trajectory import JointTrajectorySmoother  # noqa: E402
from core.handeye import matrix_to_pose, pose_to_matrix  # noqa: E402
from core.motion.kinematics import CR5Kinematics  # noqa: E402

HOME_DEG = [0.0, 0.0, -90.0, -90.0, -90.0, 0.0]
IK_POS_TOL_M = 5e-5       # == 0.05 mm；矩阵全程米制
IK_ROT_TOL = 1e-3
IK_ANG_TOL_DEG = 0.5


def _rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, math.cos(a), -math.sin(a)],
                     [0.0, math.sin(a), math.cos(a)]])


def _rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)],
                     [0.0, 1.0, 0.0],
                     [-math.sin(a), 0.0, math.cos(a)]])


def _rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0],
                     [math.sin(a), math.cos(a), 0.0],
                     [0.0, 0.0, 1.0]])


def _hand_euler(rx: float, ry: float, rz: float) -> np.ndarray:
    """独立实现的控制口径姿态：R = Rz(rz)·Ry(ry)·Rx(rx)。"""
    return _rot_z(rz) @ _rot_y(ry) @ _rot_x(rx)


class TestEulerConvention(unittest.TestCase):
    """pose_to_matrix 必须展开成 Rz·Ry·Rx —— 整条映射只在控制器帧里做乘法，口径一错全错。"""

    def test_pose_to_matrix_matches_hand_rolled_composition(self):
        for rpy in ([0, 0, 0], [30, -20, 45], [-90, 90, 0], [10, 170, -60], [45, 45, 45]):
            T = pose_to_matrix([11.0, -22.0, 33.0, rpy[0], rpy[1], rpy[2]])
            with self.subTest(rpy=rpy):
                self.assertTrue(np.allclose(T[:3, :3], _hand_euler(*rpy), atol=1e-12))
                self.assertTrue(np.allclose(T[:3, 3], [11.0, -22.0, 33.0], atol=1e-12))

    def test_round_trip_through_matrix(self):
        for pose in ([164.4, -140.0, 962.5, -62.6, 9.5, -92.6],
                     [0.0, 0.0, 800.0, 0.0, 30.0, 0.0]):
            with self.subTest(pose=pose):
                back = matrix_to_pose(pose_to_matrix(pose))
                self.assertTrue(np.allclose(back, pose, atol=1e-9), f"{back} != {pose}")

    def test_gimbal_pose_returns_equivalent_branch_not_the_same_numbers(self):
        """
        ry 越过 ±90 时 `matrix_to_pose` 会给出**另一支等价**的欧拉（rx±180、ry'=180-ry、rz±180），
        这是表示不唯一，不是 bug。结论对下游有约束力：**任何地方都不许拿欧拉三元组判等**，
        要判等就比矩阵（这条用例钉的就是这一点）。
        """
        pose = [180.159, -0.293, 90.653, 90.066, 90.035, 0.077]      # Dobot 实测奇异附近样本
        back = matrix_to_pose(pose_to_matrix(pose))
        self.assertTrue(np.allclose(pose_to_matrix(back)[:3, :3],
                                    pose_to_matrix(pose)[:3, :3], atol=1e-12), f"{back} != {pose}")
        self.assertTrue(np.allclose(back[:3], pose[:3], atol=1e-12))

    def test_forward_controller_pose_is_reconstructable(self):
        """forward_controller 的返回值经 pose_to_matrix 再 FK 一次，必须回到同一块姿态。"""
        kin = CR5Kinematics()
        q = np.radians(np.asarray(HOME_DEG))
        xyz, rpy = kin.forward_controller(list(q))
        T = pose_to_matrix([*xyz, *rpy])
        self.assertTrue(np.allclose(T[:3, :3], _hand_euler(*rpy), atol=1e-9))


class TestDeltaToBase(unittest.TestCase):
    def test_zero_delta_is_identity_motion(self):
        D = delta_to_base(_rot_z(90.0), np.eye(3), [0.0, 0.0, 0.0])
        self.assertTrue(np.allclose(D, np.eye(4), atol=1e-12))

    def test_translation_conjugates_to_base_axes(self):
        """相机 X 走 10 mm，R_cb=RotZ(90°) ⇒ 基座 Y 走 10 mm；姿态增量仍为单位。"""
        D = delta_to_base(_rot_z(90.0), np.eye(3), [0.010, 0.0, 0.0])
        self.assertTrue(np.allclose(D[:3, 3], [0.0, 0.010, 0.0], atol=1e-12), D[:3, 3])
        self.assertTrue(np.allclose(D[:3, :3], np.eye(3), atol=1e-12))

    def test_rotation_delta_conjugates(self):
        """R_cam = RotY(20°) ⇒ R_base = R_cb·R_cam·R_cbᵀ。R_cb=RotZ(90°) 把相机 Y 轴映到基座 -X，
        所以期望值是 RotX(-20°) —— 这个负号是共轭本身的语义（轴被搬到了反方向），不是笔误。"""
        R_cb = _rot_z(90.0)
        R_cam = _rot_y(20.0)
        D = delta_to_base(R_cb, R_cam, [0.0, 0.0, 0.0])
        self.assertTrue(np.allclose(D[:3, :3], R_cb @ R_cam @ R_cb.T, atol=1e-12))
        self.assertTrue(np.allclose(R_cb @ R_cam @ R_cb.T, _rot_x(-20.0), atol=1e-9))

    def test_accepts_flat_delta_r_from_json(self):
        """C++ 快照的 delta_r 是 9 元行主序（JSON 里就是一维数组），必须等价于 3x3。"""
        flat = [float(v) for v in _rot_y(20.0).reshape(-1)]
        A = delta_to_base(np.eye(3), flat, [0.0, 0.0, 0.01])
        B = delta_to_base(np.eye(3), _rot_y(20.0), [0.0, 0.0, 0.01])
        self.assertTrue(np.allclose(A, B, atol=1e-12))

    def test_rejects_garbage(self):
        for bad in ((np.eye(2), [0, 0, 0]), (np.eye(3), [0, 0]), (np.eye(3), [0, np.nan, 0])):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                delta_to_base(np.eye(3), bad[0], bad[1])


class TestRotationFromCalibration(unittest.TestCase):
    def test_takes_only_rotation_block(self):
        T = np.eye(4)
        T[:3, :3] = _rot_z(90.0)
        T[:3, 3] = [-23.735, 0.0, 220.0]          # mm 也一样：平移被无视，单位分叉碰不到这里
        self.assertTrue(np.allclose(rotation_camera_to_base(T), _rot_z(90.0), atol=1e-12))

    def test_rejects_non_orthonormal(self):
        stretched = np.eye(4)
        stretched[:3, :3] = 2.0 * _rot_z(30.0)     # 被 scale 污染：正交性检查必须拦住
        with self.assertRaises(ValueError):
            rotation_camera_to_base(stretched)
        mirrored = np.eye(4)
        R = _rot_z(30.0).copy()
        R[:, 0] *= -1.0                            # det<0：镜像不是旋转
        mirrored[:3, :3] = R
        with self.assertRaises(ValueError):
            rotation_camera_to_base(mirrored)
        with self.assertRaises(ValueError):
            rotation_camera_to_base(np.eye(3))

    def test_fallback_euler_matches_calibration_shape(self):
        self.assertTrue(np.allclose(rotation_camera_to_base_fallback([0.0, 0.0, 180.0]),
                                    _rot_z(180.0), atol=1e-12))
        with self.assertRaises(ValueError):
            rotation_camera_to_base_fallback([0.0, 0.0])


class TestJointsToTarget(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = CR5Kinematics()
        cls.home = np.radians(np.asarray(HOME_DEG))

    def test_zero_delta_keeps_baseline(self):
        best, T_target, reason = joints_to_target(
            self.kin, np.eye(3), np.eye(3), [0.0, 0.0, 0.0], self.home)
        self.assertEqual(reason, "")
        self.assertTrue(np.allclose(T_target, tcp_pose_ctrl_from_joints(self.kin, self.home),
                                    atol=1e-9))
        self.assertTrue(np.allclose(np.degrees(best), np.degrees(self.home), atol=IK_ANG_TOL_DEG),
                        f"零增量却动了: {np.degrees(best)} vs {HOME_DEG}")

    def test_solved_joints_reproduce_target_pose(self):
        """全链路自洽：IK 出来的关节角再 FK，必须落在 delta 指定的那个 TCP 位姿上。"""
        R_cb = _rot_z(90.0)
        for delta_t in ([0.030, 0.0, 0.0], [0.0, -0.020, 0.015]):
            for tag, delta_r in (("rot:0", np.eye(3)), ("rot:ry8", _rot_y(8.0))):
                with self.subTest(delta_t=delta_t, delta_r=tag):
                    best, T_target, reason = joints_to_target(
                        self.kin, R_cb, delta_r, delta_t, self.home)
                    self.assertEqual(reason, "")
                    self.assertIsNotNone(best)
                    T_back = tcp_pose_ctrl_from_joints(self.kin, best)
                    self.assertTrue(np.allclose(T_back[:3, 3], T_target[:3, 3],
                                                atol=IK_POS_TOL_M))
                    self.assertLess(float(np.abs(T_back[:3, :3] - T_target[:3, :3]).max()),
                                    IK_ROT_TOL)

    def test_tcp_matrix_is_metric(self):
        """
        单位护栏：`tcp_pose_ctrl_from_joints` 必须和 URDF 帧的 `forward()` 同量级（差在
        base/tool 变换上，不是 1000 倍）。这条专防"矩阵用 mm 喂 inverse"——表现是矩阵看着
        完全正常、IK 一个解都没有，是最难从现象反推的那类错误。
        """
        q = self.home
        T_ctrl = tcp_pose_ctrl_from_joints(self.kin, q)
        T_urdf = self.kin.controller_matrix_to_urdf(T_ctrl)
        T_fwd = self.kin.forward(list(q))
        self.assertLess(float(np.linalg.norm(T_urdf[:3, 3] - T_fwd[:3, 3])), 1e-3)
        self.assertLess(float(np.abs(T_urdf[:3, :3] - T_fwd[:3, :3]).max()), IK_ROT_TOL)
        self.assertGreater(float(np.linalg.norm(T_ctrl[:3, 3])), 0.1)   # 米，不是 462 mm
        self.assertLess(float(np.linalg.norm(T_ctrl[:3, 3])), 3.0)

    def test_baseline_not_replaced_by_live_pose(self):
        """同一增量、不同 nearest_to：目标位姿必须完全一样（nearest 只选分支，不改目的地）。"""
        _, T1, _ = joints_to_target(self.kin, np.eye(3), _rot_y(5.0), [0.02, 0.0, 0.0],
                                    self.home, nearest_to=self.home)
        _, T2, _ = joints_to_target(self.kin, np.eye(3), _rot_y(5.0), [0.02, 0.0, 0.0],
                                    self.home, nearest_to=self.home + 0.3)
        self.assertTrue(np.allclose(T1, T2, atol=1e-12))

    def test_rejects_bad_baseline(self):
        self.assertIn("基线关节角需要 6 个值",
                      joints_to_target(self.kin, np.eye(3), np.eye(3), [0, 0, 0],
                                       [0.0, 0.0, 0.0])[2])
        self.assertIn("非有限",
                      joints_to_target(self.kin, np.eye(3), np.eye(3), [0, 0, 0],
                                       [0.0, np.nan, 0.0, 0.0, 0.0, 0.0])[2])

    def test_unreachable_target_reports_failure_not_clamped_pose(self):
        """50 m 平移：必须返回 None + ik_failed，而不是把位姿夹到限位上假装成功。"""
        best, _T, reason = joints_to_target(self.kin, np.eye(3), np.eye(3),
                                            [50.0, 50.0, 50.0], self.home)
        self.assertIsNone(best)
        self.assertIn("ik", reason)

    def test_pose_ctrl_from_target_shape(self):
        pose = pose_ctrl_from_target(self.kin, self.home)
        self.assertEqual(len(pose), 6)
        self.assertTrue(np.all(np.isfinite(pose)))


class TestTrajectorySmoother(unittest.TestCase):
    """33 Hz 发射链的平滑器：限速、速度连续、掉头、钉位 —— 每条都对应一种页面上看得见的抖动。

    实现是追踪式（一阶收敛 + 限速钳制），不是分段式：分段余弦缓动在 20 Hz 稠密
    keyframe 下每个段边界速度归零，臂会变成 20 Hz 的"加-减-加-减"脉动，实测比不平滑还顿。
    这里的用例就是钉住这个教训的。
    """

    def _sm(self, vel_deg=90.0):
        return JointTrajectorySmoother(math.radians(vel_deg))

    def test_no_output_before_first_keyframe(self):
        sm = self._sm()
        self.assertIsNone(sm.step(0.03))      # 没有"现在在哪儿"时不许发零向量冒充输出
        self.assertFalse(sm.push_target(np.zeros(5)))            # 维数不对拒收
        self.assertFalse(sm.push_target(np.full(6, np.nan)))     # 非有限拒收
        self.assertIsNone(sm.step(0.03))

    def test_rate_limit_holds_on_dense_stream(self):
        """这是 33 Hz 契约的全部意义：相邻发射点的关节角变化率不许超限。
        追踪式里速度逐拍被钳在 ±max_vel，所以这里用硬上限判，不给余量。"""
        sm = self._sm(vel_deg=90.0)
        sm.push_target(np.zeros(6))
        target = np.zeros(6); target[1] = math.radians(20.0)
        sm.push_target(target)
        dt = 1.0 / 33.0
        prev = sm.step(dt)
        vmax = math.radians(90.0)
        while sm.moving:
            cur = sm.step(dt)
            step_rate = float(np.max(np.abs(cur - prev))) / dt
            self.assertLessEqual(step_rate, vmax * 1.001,
                                 f"相邻发射点超速：{math.degrees(step_rate):.1f}°/s")
            prev = cur
        # 刹停后精确落位，之后钉住不动（静止时发射流 = 常数）。
        self.assertTrue(np.allclose(prev, target, atol=1e-12))
        again = sm.step(dt)
        self.assertTrue(np.array_equal(again, prev))

    def test_dense_keyframes_do_not_pulse_velocity(self):
        """回归用例（旧分段余弦版的死刑判决）：目标以 20 Hz 匀速运动时，33 Hz 发射流的
        逐拍速度在追稳后必须一直"在动"，不许在每个 keyframe 边界塌回零附近。
        旧实现在这条用例下：每 50 ms 速度归零一次，脉动幅度 ≈ 全部速度。"""
        sm = self._sm(vel_deg=90.0)
        dt = 1.0 / 33.0
        target_vel = math.radians(10.0)                        # 目标沿关节0匀速 10°/s（限速之内）
        q = np.zeros(6)
        sm.push_target(q.copy())
        speeds = []
        prev = None
        t = 0.0
        kf_next = 0.0
        for _ in range(660):                                   # 20 s：足够走过收敛段
            if t >= kf_next:                                   # 20 Hz keyframe：目标继续往前走
                q = q.copy(); q[0] += target_vel * 0.05
                sm.push_target(q)
                kf_next += 0.05
            cur = sm.step(dt)
            if prev is not None:
                speeds.append(abs(cur[0] - prev[0]) / dt)
            prev = cur
            t += dt
        steady = speeds[100:]                                  # 丢掉前 3 s 的起步段
        self.assertGreater(min(steady), target_vel * 0.35,
                           f"追稳后速度仍塌到 {math.degrees(min(steady)):.2f}°/s —— 脉动复活")
        # 也不许冲过头太多：拖尾上限 ≈ v_target/k + 一拍积分，给到 1° 封顶。
        lag = q[0] - prev[0]
        self.assertGreaterEqual(lag, -1e-3)
        self.assertLess(lag, math.radians(1.0))

    def test_retarget_reverses_from_current_output_without_jump(self):
        """中途掉头：带着当前速度直接转向 —— 输出位连续（不跳变），方向可以反。
        旧接口查 q_from/重规划；追踪式里没有段，判据换成输出序列的连续性。"""
        sm = self._sm(vel_deg=90.0)
        sm.push_target(np.zeros(6))
        fwd = np.zeros(6); fwd[2] = math.radians(30.0)
        sm.push_target(fwd)
        mid = None
        for _ in range(5):                                     # 走几步，让它带上正向速度
            mid = sm.step(0.05)
        self.assertGreater(mid[2], 0.0)
        sm.push_target(np.zeros(6))                            # 掉头
        prev = mid
        crossed = False
        for _ in range(2000):
            cur = sm.step(0.03)
            self.assertLessEqual(float(np.max(np.abs(cur - prev))),
                                 math.radians(90.0) * 0.03 * 1.001)   # 每一步都限速，无跳变
            if not crossed and cur[2] < 0.0:
                crossed = True                                 # 允许过冲，不许回头绕大圈
            if not sm.moving:
                break
            prev = cur
        self.assertTrue(np.allclose(sm.step(0.03), np.zeros(6), atol=1e-9))

    def test_reset_starts_fresh(self):
        sm = self._sm()
        sm.push_target(np.zeros(6))
        tgt = np.zeros(6); tgt[3] = 0.5
        sm.push_target(tgt)
        sm.step(0.05)
        sm.reset()
        self.assertIsNone(sm.step(0.03))        # 旧基线下的速度与目标不属于下一次启动
        self.assertFalse(sm.moving)


def _snap(frames: int, delta_t, delta_r=None) -> dict:
    """一条"C++ 解出了一帧"的最小快照。"""
    return {"enabled": True, "has_pose": True, "status": "ok", "frames": frames,
            "delta_r": (np.eye(3) if delta_r is None else delta_r).reshape(3, 3).tolist(),
            "delta_t_m": list(delta_t)}


def _stub_service() -> fs_mod.FollowService:
    """
    一个**永不出网、永不起线程**的 FollowService：轴映射钉成单位阵（期望值可手算）、基线
    = home、轮询线程与推送订阅两个入口全部堵死。

    两个用例类共用这一份，是因为"忘了堵 `_ensure_stream`"这种缺陷只在一个类里出现时，
    失败要等到相机服务恰好在跑才现形 —— 那时服务端会往单例里灌真帧，和测试自己的假快照
    抢同一个 `_target_q`，表现出来就是断言莫名其妙地不稳。
    """
    svc = fs_mod.FollowService()
    svc._R_cb = np.eye(3)
    svc._baseline_q = np.radians(np.asarray(HOME_DEG))
    svc._ensure_poller = lambda: None      # 测试里不起轮询线程
    svc._ensure_stream = lambda: None      # 也不起订阅线程
    svc._stop_stream = lambda: None
    return svc


class TestFollowServiceGuards(unittest.TestCase):
    """服务层的四条不变量：拒绝真实臂、保持上一目标、停止清空、后端失联可被看见。"""

    def setUp(self):
        self.svc = _stub_service()

    def test_real_mode_is_refused_not_faked(self):
        self.svc._arm["mode"] = "real"
        ok, msg = self.svc.start(HOME_DEG)
        self.assertFalse(ok)
        self.assertIn("P5", msg)
        self.assertFalse(self.svc._active)

    def test_missing_axis_mapping_is_refused(self):
        self.svc._R_cb = None
        self.svc._R_cb_source = "标定缺失"
        ok, msg = self.svc.start(HOME_DEG)
        self.assertFalse(ok)
        self.assertIn("标定缺失", msg)          # 用哪个来源必须跟着失败一起报出去

    def test_holds_last_target_on_ik_failure(self):
        self.svc._active = True
        self.svc._target_q = self.svc._baseline_q.copy()
        self.svc._fetch_snapshot = lambda: _snap(1, [0.020, 0.0, 0.0])
        self.svc._poll_once()
        solved = self.svc._target_q.copy()
        self.assertFalse(self.svc._ik_failed)
        self.assertGreater(float(np.linalg.norm(solved - self.svc._baseline_q)), 1e-4)

        self.svc._fetch_snapshot = lambda: _snap(2, [50.0, 50.0, 50.0])
        self.svc._poll_once()
        self.assertTrue(self.svc._ik_failed)
        self.assertIn("ik", self.svc._last_error)
        self.assertTrue(np.array_equal(self.svc._target_q, solved), "失败帧改动了目标关节角")

    def test_duplicate_frame_is_not_resolved_twice(self):
        self.svc._active = True
        self.svc._target_q = self.svc._baseline_q.copy()
        snap = _snap(7, [0.020, 0.0, 0.0])
        calls = {"n": 0}

        def _solve(*_a, **_k):
            calls["n"] += 1
            return None, np.eye(4), "boom"

        self.svc._fetch_snapshot = lambda: dict(snap)
        orig = self.svc._solve
        self.svc._solve = _solve
        try:
            self.svc._poll_once()
            self.svc._poll_once()               # frames 没前进 ⇒ 同一批帧只解一次
        finally:
            self.svc._solve = orig
        self.assertEqual(calls["n"], 1)

    def test_not_ok_status_does_not_move_arm(self):
        self.svc._active = True
        self.svc._target_q = self.svc._baseline_q.copy()
        held = self.svc._target_q.copy()
        snap = _snap(3, [0.5, 0.5, 0.5])
        snap["status"] = "lost"                 # 丢目标：C++ 报 last-trustworthy，但 ok=False
        snap["has_pose"] = False
        self.svc._fetch_snapshot = lambda: snap
        self.svc._poll_once()
        self.assertTrue(np.array_equal(self.svc._target_q, held))

    def test_stop_clears_arm_state(self):
        self.svc._active = True
        self.svc._target_q = self.svc._baseline_q.copy()
        self.svc._cpp = lambda kind, payload=None, timeout=1.0: (True, "", {"enabled": False})
        ok, _msg = self.svc.stop()
        self.assertTrue(ok)
        self.assertFalse(self.svc._active)
        self.assertIsNone(self.svc._target_q)
        self.assertIsNone(self.svc._baseline_q)

    def test_stop_reports_measured_profile(self):
        # 报"实测回到了哪一档"：快照里有尺寸时必须是那两个数，而不是配置键名。
        self.svc._active = True
        self.svc._cpp = lambda kind, payload=None, timeout=1.0: (
            True, "", {"enabled": False, "capture_width": 1280, "capture_height": 800})
        ok, msg = self.svc.stop()
        self.assertTrue(ok)
        self.assertIn("1280x800", msg)

    def test_zero_without_known_pose_refuses(self):
        self.svc._active = True
        self.svc._target_q = None
        ok, msg = self.svc.zero(None)
        self.assertFalse(ok)
        self.assertIn("调零", msg)

    def test_upstream_down_flag_tracks_connectivity(self):
        self.assertFalse(self.svc.camera_service_down)
        ok, msg, _data = self.svc._cpp("status", timeout=0.2)
        if not ok and not self.svc.camera_service_down:
            # 相机服务在跑着：换个必然连不上的端口再验一次降级判定
            orig = fs_mod.CPP_BASE_URL
            fs_mod.CPP_BASE_URL = "http://127.0.0.1:1"
            try:
                self.assertFalse(self.svc._cpp("status", timeout=0.2)[0])
                self.assertTrue(self.svc.camera_service_down)
            finally:
                fs_mod.CPP_BASE_URL = orig
        self.svc._upstream_down = False          # 别把状态留给后面的用例

    def test_state_payload_carries_arm_fields(self):
        self.svc._active = True
        self.svc._target_q = self.svc._baseline_q.copy()
        self.svc._snapshot = {"status": "ok", "frames": 1}
        st = self.svc.status()
        for key in ("joints_deg", "target_pose", "arm_baseline_deg", "arm_target_deg",
                    "ik_failed", "r_cb_source", "camera_service_reachable",
                    "emit_hz", "max_joint_vel_deg_s"):
            self.assertIn(key, st)
        self.assertEqual(len(st["joints_deg"]), 6)
        self.assertEqual(len(st["target_pose"]), 6)
        self.assertGreaterEqual(st["emit_hz"], 5)

    def test_keyframe_flows_into_33hz_emit_chain(self):
        """轮询解出的目标必须进发射链：抽干队列后发射一步，广播的 joints 在动。"""
        self.svc._active = True
        self.svc._baseline_q = np.radians(np.asarray(HOME_DEG))
        self.svc._smoother.reset()
        self.svc._smoother.push_target(self.svc._baseline_q)
        self.svc._target_q = self.svc._baseline_q.copy()
        self.svc._fetch_snapshot = lambda: _snap(1, [0.030, 0.0, 0.0])
        got = []
        self.svc.register_ws_callback(lambda m: got.append(m))
        self.svc._poll_once()                    # 解算 → keyframe 进队列
        self.assertEqual(self.svc._kf_queue.qsize(), 1)
        self.svc._emit_tick(1.0 / 33.0)          # 发射线程的一个 tick：抽队列 + 平滑器 + 广播
        self.assertEqual(self.svc._kf_queue.qsize(), 0)
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0]["type"], "follow_state")
        self.assertEqual(len(got[0]["data"]["joints_deg"]), 6)


class TestDataPlane(unittest.TestCase):
    """
    数据面选路（推送 ⇄ 兜底轮询）。钉的是四条会安静出错的性质：
    推送新鲜时轮询一次都不出网；推送变凉/没起来/被关掉时轮询立刻接管且说清原因；
    两条路共用同一个 `_ingest`（否则同一帧会解出两个目标）；停订阅时不许持锁（真线上是死锁）。
    """

    def setUp(self):
        self.svc = _stub_service()
        self.svc._active = True
        self.svc._target_q = self.svc._baseline_q.copy()
        self.svc._push_enabled = True
        # 用**真的** PoseStream 但不 start()：构造它不开 socket，而 `last_event_age()` /
        # `stats()` 就走的真是实现。手写替身一旦和真类字段漂移，测试只剩自证。
        self.stream = PoseStream("http://127.0.0.1:1", self.svc._on_push_snapshot)
        self.svc._push = self.stream
        self.fetched: list = []

        def _fetch() -> dict:
            self.fetched.append(1)
            return _snap(len(self.fetched) + 1, [0.020, 0.0, 0.0])

        self.svc._fetch_snapshot = _fetch

    def _age(self, seconds) -> None:
        """把订阅端的"最近一帧"打到 seconds 秒前；None = 一帧都没收到过。"""
        self.stream._last_event_mono = 0.0 if seconds is None else time.monotonic() - seconds

    def _stale(self) -> float:
        return self.svc._push_stale_s + 1.0

    def test_fresh_push_keeps_poller_off_the_network(self):
        self._age(0.0)
        self.svc._poll_once()
        self.assertEqual(self.fetched, [], "推送活着还去拉：和推送抢同一份 C++ 快照")
        dp = self.svc.status()["data_plane"]
        self.assertEqual(dp["mode"], "push")
        self.assertNotIn("reason", dp)                # 正常态不该有"为什么退化了"

    def test_stale_push_hands_data_plane_back_to_poll(self):
        self._age(self._stale())
        self.svc._poll_once()
        self.assertEqual(len(self.fetched), 1)
        dp = self.svc.status()["data_plane"]
        self.assertEqual(dp["mode"], "poll")
        self.assertIn("退回轮询", dp["reason"])

    def test_no_event_yet_still_polls(self):
        """启动后的第一个 poll 周期里推送还不该被当成"活着"，否则闭环开局是瞎的。"""
        self._age(None)
        self.svc._poll_once()
        self.assertEqual(len(self.fetched), 1)
        self.assertIn("尚无一帧", self.svc.status()["data_plane"]["reason"])

    def test_push_switch_off_is_pure_poll(self):
        self.svc._push_enabled = False
        self._age(0.0)                                 # 即便流上是新鲜的
        self.svc._poll_once()
        self.assertEqual(len(self.fetched), 1)
        dp = self.svc.status()["data_plane"]
        self.assertEqual(dp["mode"], "poll")
        # 原因必须是"开关未开"，不是下游症状"线程未运行" —— 后者会把人支使去查根本没起的线程。
        self.assertIn("开关未开", dp["reason"])

    def test_reason_is_present_even_before_subscribe(self):
        """没点"启动"时 `push` 子对象不存在，但 reason 键必须在：页面不该按调用历史分支渲染。"""
        self.svc._push = None
        dp = self.svc.status()["data_plane"]
        self.assertEqual(dp["mode"], "poll")
        self.assertIn("订阅未启动", dp["reason"])

    def test_both_paths_funnel_into_one_ingest(self):
        seen: list = []
        self.svc._ingest = lambda snap: seen.append(snap)
        self._age(0.0)
        pushed = _snap(11, [0.010, 0.0, 0.0])
        self.svc._on_push_snapshot(pushed)             # 推送这条路
        self.svc._poll_once()                          # 推送新鲜 ⇒ 轮询不该再喂一次
        self.assertEqual(seen, [pushed])
        self._age(self._stale())
        polled = _snap(12, [0.010, 0.0, 0.0])
        self.svc._fetch_snapshot = lambda: polled
        self.svc._poll_once()                          # 兜底这条路
        self.assertEqual(seen, [pushed, polled], "两条路喂给的不是同一个入口或不是同一份载荷")

    def test_pushed_frame_actually_moves_arm(self):
        """推送帧必须走完整解算链：只测"回调被调了"不够，方向错了页面看起来仍在跟。"""
        self._age(0.0)
        self.svc._on_push_snapshot(_snap(1, [0.020, 0.0, 0.0]))
        self.assertFalse(self.svc._ik_failed)
        # R_cb = 单位阵 ⇒ 相机 X 平移原样落在基座 X 上：J1 应当离开基线
        self.assertGreater(float(np.linalg.norm(self.svc._target_q - self.svc._baseline_q)), 1e-4)
        self.assertEqual(self.svc._kf_queue.qsize(), 1, "解出的目标没进 33 Hz 发射链")

    def test_push_snapshot_dropped_when_not_following(self):
        """没在跟时来的帧不许写进缓存：否则"停止"之后页面还在显示一套没人用的位姿。"""
        self.svc._active = False
        self.svc._snapshot = {}
        self.svc._on_push_snapshot(_snap(5, [0.5, 0.5, 0.5]))
        self.assertEqual(self.svc._snapshot, {})

    def test_stop_drops_the_stream_reference(self):
        """停止后不许留着那条已死的流：否则状态会继续报"数据面=push"（打点还是新鲜的）。
        这里把 setUp 的替身撤掉走真实 `_stop_stream`，只把 `PoseStream.stop` 换成记录器
        —— join 线程不是本用例要测的东西，"引用被丢掉"才是。"""
        self.svc.__dict__.pop("_stop_stream")
        stopped: list = []
        self.stream.stop = lambda: stopped.append(1)
        self.svc._cpp = lambda kind, payload=None, timeout=1.0: (True, "", {"enabled": False})
        ok, _msg = self.svc.stop()
        self.assertTrue(ok)
        self.assertEqual(stopped, [1], "没有真的停订阅线程：socket 会一直挂着")
        self.assertIsNone(self.svc._push)
        self.assertIn("订阅未启动", self.svc.status()["data_plane"]["reason"])

    def test_stop_stream_runs_outside_service_lock(self):
        """
        真线上唯一的死锁成因：`_stop_stream` 要 join 读者线程，而那个线程的回调正等 `_lock`
        —— 持锁来停就是互等到超时，而且只在"恰好有一帧正在投递"时复现。
        判据用 `_is_owned()`（同线程、当场）：换"另一个线程去抢锁"的探针要么得非阻塞抢
        （主线程随后又会拿锁 ⇒ 偶发假失败），要么给超时重试（bug 场景里最终也抢得到 ⇒ 白测）。
        """
        owned: list = []
        self.svc._stop_stream = lambda: owned.append(self.svc._lock._is_owned())
        self.svc._cpp = lambda kind, payload=None, timeout=1.0: (True, "", {"enabled": False})
        ok, _msg = self.svc.stop()
        self.assertTrue(ok)
        self.assertEqual(owned, [False], "stop() 持着 _lock 调了 _stop_stream：真线上会死锁")


if __name__ == "__main__":
    unittest.main(verbosity=2)
