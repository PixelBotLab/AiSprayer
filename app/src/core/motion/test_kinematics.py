# -*- coding: utf-8 -*-
"""libmotion_c 绑定冒烟：Home FK、控制器 IK 复现、最近分支。

    cd app/src && python3 -m core.motion.test_kinematics
"""
from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from core.motion.kinematics import (  # noqa: E402
    MAX_JOINT_OFFSET_DEG, CR5Kinematics, validate_joint_offsets,
)

HOME_DEG = [0.0, 0.0, -90.0, -90.0, -90.0, 0.0]
POS_TOL_MM = 0.05
IK_ANG_TOL_DEG = 0.5


class TestMotionKinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = CR5Kinematics()

    def test_home_fk_controller(self):
        q = [math.radians(v) for v in HOME_DEG]
        xyz, rpy = self.kin.forward_controller(q)
        self.assertEqual(len(xyz), 3)
        self.assertEqual(len(rpy), 3)
        self.assertTrue(all(math.isfinite(v) for v in xyz + rpy))

    def test_controller_ik_recovers_home(self):
        q = np.radians(HOME_DEG)
        xyz, rpy = self.kin.forward_controller(q)
        sols = self.kin.inverse_controller(xyz, rpy)
        self.assertGreater(len(sols), 0)
        recovered = any(
            np.allclose(np.degrees(sol), HOME_DEG, atol=IK_ANG_TOL_DEG) for sol in sols
        )
        self.assertTrue(recovered, f"home not in { [np.round(np.degrees(s), 3) for s in sols] }")

    def test_best_ik_stays_on_home_branch(self):
        q = np.radians(HOME_DEG)
        T = self.kin.forward(q)
        best = self.kin.get_best_ik(T, q)
        self.assertIsNotNone(best)
        self.assertTrue(np.allclose(np.degrees(best), HOME_DEG, atol=IK_ANG_TOL_DEG))

    def test_fk_ik_roundtrip_sample(self):
        q = np.radians([45.0, -30.0, 60.0, 10.0, 45.0, -20.0])
        xyz, rpy = self.kin.forward_controller(q)
        sols = self.kin.inverse_controller(xyz, rpy)
        self.assertGreater(len(sols), 0)
        xyz2, _ = self.kin.forward_controller(sols[0])
        self.assertTrue(np.allclose(xyz, xyz2, atol=POS_TOL_MM))


class TestJointOffsetCompensation(unittest.TestCase):
    """关节零位偏移 Δq 的口径契约: FK 入口加、IK 出口减, 入出参永远是上报值。"""

    # 本臂真实识别量的量级 (二手 CR5, 跨会话稳到 0.34° 以内); j6=0 是规范自由度
    DQ_DEG = [-0.706, -0.953, -1.488, -0.061, -1.958, 0.0]
    Q_REPORTED_DEG = [10.0, -20.0, 45.0, 5.0, 30.0, -15.0]

    def setUp(self):
        self.plain = CR5Kinematics()
        self.corr = CR5Kinematics(joint_offsets_deg=self.DQ_DEG)
        self.q_rep = np.radians(self.Q_REPORTED_DEG)

    def test_zero_offsets_reproduce_legacy_behaviour(self):
        """全零偏移必须与不传偏移逐位相同 —— 未启用补偿时不得改变任何现有行为。"""
        plain = CR5Kinematics(joint_offsets_deg=[0.0] * 6)
        T_a, T_b = self.plain.forward(self.q_rep), plain.forward(self.q_rep)
        self.assertTrue(np.array_equal(T_a, T_b))
        self.assertEqual(self.plain.forward_controller(self.q_rep),
                         plain.forward_controller(self.q_rep))
        self.assertFalse(plain.has_joint_offsets)

    def test_fk_equals_forward_of_corrected_joints(self):
        """定义性性质: FK_corrected(q_上报) 必须逐位等于 FK_plain(q_上报 + Δq)。"""
        shifted = self.q_rep + np.radians(self.DQ_DEG)
        self.assertTrue(np.allclose(self.corr.forward(self.q_rep),
                                    self.plain.forward(shifted), atol=1e-12))
        self.assertTrue(np.allclose(self.corr.forward_controller(self.q_rep)[0],
                                    self.plain.forward_controller(shifted)[0], atol=1e-9))

    def test_correction_moves_the_pose_by_the_measured_lever_arm(self):
        """偏移非零时位姿必须真的变 (防止“加了参数但没生效”这种假修复)。

        用 forward_controller 量 (平移 mm); forward 的矩阵是 URDF 帧 (平移 m), 量纲不同。
        """
        a = np.asarray(self.corr.forward_controller(self.q_rep)[0])
        b = np.asarray(self.plain.forward_controller(self.q_rep)[0])
        self.assertGreater(np.linalg.norm(a - b), 5.0,
                           "本臂 Δq 在杆臂上的平移投影实测约 17mm")
        self.assertGreater(np.linalg.norm(np.asarray(
            self.corr.forward_controller(self.q_rep)[1])
            - np.asarray(self.plain.forward_controller(self.q_rep)[1])), 0.5)

    def test_ik_is_inverse_of_fk_under_offsets(self):
        """IK 出口减偏移: 反解结果直接是“可下发的上报关节角”, 与 FK 闭环。"""
        best = self.corr.get_best_ik(self.corr.forward(self.q_rep), self.q_rep)
        self.assertIsNotNone(best)
        self.assertTrue(np.allclose(np.degrees(best), self.Q_REPORTED_DEG, atol=IK_ANG_TOL_DEG),
                        f"闭环失败: {np.degrees(best)}")
        T_back = self.corr.forward(best)
        self.assertTrue(np.allclose(T_back[:3, 3],
                                    self.corr.forward(self.q_rep)[:3, 3], atol=1e-6),
                        "URDF 帧平移单位是 m, 1e-6 m = 0.001 mm")

    def test_inverse_controller_returns_reported_joints(self):
        """控制器口径反解同样吃/吐上报值: 候选解里必须包含原上报关节角。"""
        xyz, rpy = self.corr.forward_controller(self.q_rep)
        sols = self.corr.inverse_controller(xyz, rpy)
        self.assertGreater(len(sols), 0)
        hit = any(np.allclose(np.degrees(s), self.Q_REPORTED_DEG, atol=IK_ANG_TOL_DEG)
                  for s in sols)
        self.assertTrue(hit, f"上报口径的解丢失: {[np.round(np.degrees(s), 3) for s in sols]}")

    def test_flange_pose_helper_matches_forward_controller(self):
        """法兰位姿便捷方法 = forward_controller 的拼接 (度入度出, 平移 mm)。"""
        pose = self.corr.flange_pose_controller_deg(self.Q_REPORTED_DEG)
        xyz, rpy = self.corr.forward_controller(self.q_rep)
        self.assertTrue(np.allclose(pose, list(xyz) + list(rpy), atol=1e-9))

    def test_validate_rejects_illegal_offsets(self):
        """防御校验: 轴数错 / 非有限 / 超物理上限都必须快速失败。"""
        for bad in ([0.1, 0.2, 0.3], [0.0] * 5, [float("nan")] + [0.0] * 5,
                    [MAX_JOINT_OFFSET_DEG + 0.1] + [0.0] * 5):
            with self.assertRaises(ValueError):
                validate_joint_offsets(bad)
        self.assertTrue(np.array_equal(validate_joint_offsets(None), np.zeros(6)))
        self.assertTrue(np.array_equal(validate_joint_offsets([]), np.zeros(6)))
        self.assertEqual(validate_joint_offsets([1.0] * 6).tolist(), [1.0] * 6)
        with self.assertRaises(ValueError):
            CR5Kinematics(joint_offsets_deg=[9.0, 0, 0, 0, 0, 0])

    def test_matches_joint_offsets_for_cache_invalidation(self):
        """调用方缓存实例, 靠它判定配置是否改过 (非法输入视为不匹配, 不抛)。"""
        self.assertTrue(self.corr.matches_joint_offsets(self.DQ_DEG))
        self.assertFalse(self.corr.matches_joint_offsets([0.0] * 6))
        self.assertFalse(self.corr.matches_joint_offsets([self.DQ_DEG[0] + 0.01]
                                                        + self.DQ_DEG[1:]))
        self.assertFalse(self.corr.matches_joint_offsets([1e9] * 6))
        self.assertTrue(self.plain.matches_joint_offsets(None))
        self.assertTrue(self.corr.has_joint_offsets)


if __name__ == "__main__":
    unittest.main()
