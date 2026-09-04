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

from core.motion.kinematics import CR5Kinematics  # noqa: E402

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


if __name__ == "__main__":
    unittest.main()
