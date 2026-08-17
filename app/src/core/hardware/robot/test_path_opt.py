"""稀疏喷涂 Waypoint Viterbi 优化器单测。"""

import math
import os
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../")))

from app.src.core.hardware.robot.cr5_kinematics import CR5Kinematics
from app.src.core.hardware.robot.verification.path_opt import (
    SprayWaypointOptimizer,
    _geodesic_deg,
    _project_to_anchor_envelope,
    _waypoint_to_T,
)


def _sample_pose(solver: CR5Kinematics):
    """用一组非奇异关节做 FK，得到控制器系枪尖位姿（mm / deg）。"""
    q = np.deg2rad([45.0, -30.0, 60.0, 0.0, 45.0, 10.0])
    xyz, rpy = solver.forward_controller(q)
    wp = {
        "x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2]),
        "rx": float(rpy[0]), "ry": float(rpy[1]), "rz": float(rpy[2]),
    }
    return q, wp


class TestSprayWaypointOptimizer(unittest.TestCase):
    def setUp(self):
        self.solver = CR5Kinematics(backend="auto")
        # 单测关掉密采样门和自旋搜索，缩短 DP；生产默认会开 verifier + 全自旋网格
        self.opt = SprayWaypointOptimizer(
            solver=self.solver,
            verifier=None,
            dense_verify=False,
            tol_x_deg=(-5.0, 5.0, 5.0),
            tol_y_deg=(-5.0, 5.0, 5.0),
            tol_z_deg=(0.0, 0.0, 30.0),
            beam_width=32,
            num_movel_checks=4,
        )
        self.q0, self.wp0 = _sample_pose(self.solver)
        self.wp1 = dict(self.wp0)
        self.wp1["x"] = self.wp0["x"] + 100.0

    def test_anchor_projection_clips_and_keeps_inside(self):
        """越界相对 Rx 应裁到 ±5°；已在盒内的姿态保持不变。"""
        anchor = R_scipy.from_euler("xyz", [0.0, 0.0, 0.0], degrees=True)
        cand = R_scipy.from_euler("xyz", [20.0, 0.0, 0.0], degrees=True)
        proj = _project_to_anchor_envelope(cand, anchor, (5.0, 5.0, 180.0))
        e = (anchor.inv() * proj).as_euler("xyz", degrees=True)
        self.assertLess(abs(e[0]), 5.0 + 1e-6)
        inside = R_scipy.from_euler("xyz", [2.0, 1.0, 10.0], degrees=True)
        kept = _project_to_anchor_envelope(inside, anchor, (5.0, 5.0, 180.0))
        self.assertLess(_geodesic_deg(inside, kept), 1e-6)

    def test_easy_movel_stays_safe_and_uses_tilt_only_if_needed(self):
        """短段尽量保持名义姿态；较长 MoveL 必要时才用尽 5° 倾角，且关节连续、非奇异。"""
        short = dict(self.wp0)
        short["x"] = self.wp0["x"] + 20.0
        poses, qs_deg, transforms = self.opt.optimize(
            [self.wp0, short],
            init_q=self.q0,
            anchor_tolerances_deg=(15.0, 15.0, 180.0),
        )
        self.assertEqual(len(poses), 2)
        rot0 = R_scipy.from_matrix(_waypoint_to_T(self.wp0)[:3, :3])
        geo0 = _geodesic_deg(rot0, R_scipy.from_matrix(transforms[0][:3, :3]))
        geo1 = _geodesic_deg(rot0, R_scipy.from_matrix(transforms[1][:3, :3]))
        self.assertLess(geo0, 1.0)
        self.assertLess(max(geo0, geo1), 5.0 + 1e-6)

        long_wp = dict(self.wp0)
        long_wp["x"] = self.wp0["x"] + 100.0
        _, qs_long, Ts_long = self.opt.optimize(
            [self.wp0, long_wp],
            init_q=self.q0,
            anchor_tolerances_deg=(15.0, 15.0, 180.0),
        )
        dq = np.deg2rad(qs_long[1]) - np.deg2rad(qs_long[0])
        dq = np.mod(dq + math.pi, 2.0 * math.pi) - math.pi
        self.assertLess(np.max(np.abs(np.degrees(dq))), 45.0)
        for T, q_deg in zip(Ts_long, qs_long):
            q = np.deg2rad(q_deg)
            self.assertTrue(self.solver.is_joint_valid(q))
            T_urdf = self.solver.controller_matrix_to_urdf(T)
            self.assertFalse(self.solver.check_singularity_risk(q, T=T_urdf)["is_singular"])

    def test_path_item_writes_tcp_pose_base(self):
        """生产路径格式应写回 tcp_pose_base，位置间距保持 100 mm。"""
        item = {
            "path_id": 7,
            "name": "seg",
            "points": [
                {"tcp_pose_base": dict(self.wp0)},
                {"tcp_pose_base": dict(self.wp1)},
            ],
        }
        out, _ = self.opt.optimize_path_item(item, init_q=self.q0)
        self.assertEqual(len(out["points"]), 2)
        self.assertIn("tcp_pose_base", out["points"][0])
        self.assertIn("spray_opt_joints_deg", out)
        x0 = out["points"][0]["tcp_pose_base"]["x"]
        x1 = out["points"][1]["tcp_pose_base"]["x"]
        self.assertAlmostEqual(x1 - x0, 100.0, places=1)

    def test_single_waypoint(self):
        """单点路径只选姿，不跑段间 DP。"""
        poses, qs, Ts = self.opt.optimize([self.wp0], init_q=self.q0)
        self.assertEqual(len(poses), 1)
        self.assertEqual(len(Ts), 1)
        self.assertTrue(self.solver.is_joint_valid(np.deg2rad(qs[0])))


if __name__ == "__main__":
    unittest.main()
