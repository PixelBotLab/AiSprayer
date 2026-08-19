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
    _R_from_euler_xyz_deg,
    _branch_key,
    _euler_xyz_deg_from_R,
    _geodesic_deg,
    _project_R_to_anchor,
    _project_to_anchor_envelope,
    _quat_from_R,
    _quat_key_arr,
    _waypoint_to_T,
    _wrap_pi,
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
        # 整段行程可以超过 45°；45° 只约束相邻抽检点，这里只排除翻腕级跳变
        self.assertLess(np.max(np.abs(np.degrees(dq))), 170.0)
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

    def test_dense_verify_report_carries_path_identity(self):
        """optimize() 密校验报告的 path_id / name 应透传自 path_item。"""
        captured: dict = {}

        class _FakeVerifier:
            def verify_single_path(self, path_item, init_q=None):
                captured.update(path_item)
                return {
                    "path_id": path_item["path_id"],
                    "name": path_item["name"],
                    "status": "OK",
                    "issues": [],
                    "trajectory_q": [],
                    "trajectory_tcp": [],
                }

        opt = SprayWaypointOptimizer(
            solver=self.solver,
            verifier=_FakeVerifier(),
            dense_verify=True,
            tol_x_deg=(-5.0, 5.0, 5.0),
            tol_y_deg=(-5.0, 5.0, 5.0),
            tol_z_deg=(0.0, 0.0, 30.0),
            beam_width=32,
            num_movel_checks=4,
        )
        item = {
            "path_id": 42,
            "name": "review-seg",
            "points": [
                {"tcp_pose_base": dict(self.wp0)},
                {"tcp_pose_base": dict(self.wp1)},
            ],
        }
        opt.optimize(
            item["points"],
            init_q=self.q0,
            path_id=item["path_id"],
            path_name=item["name"],
        )
        self.assertEqual(captured.get("path_id"), 42)
        self.assertEqual(captured.get("name"), "review-seg")

    def test_single_waypoint(self):
        """单点路径只选姿，不跑段间 DP。"""
        poses, qs, Ts = self.opt.optimize([self.wp0], init_q=self.q0)
        self.assertEqual(len(poses), 1)
        self.assertEqual(len(Ts), 1)
        self.assertTrue(self.solver.is_joint_valid(np.deg2rad(qs[0])))

    def test_rotation_helpers_match_scipy(self):
        """网格用的矩阵/四元数/欧拉辅助函数应与 SciPy xyz 内旋一致。"""
        samples = (
            [0.0, 0.0, 0.0],
            [5.0, -3.0, 10.0],
            [-5.0, 5.0, 180.0],
            [2.0, 1.0, -170.0],
            [10.0, 89.9, 25.0],
            [10.0, 89.95, 25.0],
            [10.0, 90.0, 25.0],
            [-30.0, -90.0, 40.0],
            [5.0, -89.999, -70.0],
        )
        for ang in samples:
            rs = R_scipy.from_euler("xyz", ang, degrees=True)
            Rm = _R_from_euler_xyz_deg(ang[0], ang[1], ang[2])
            np.testing.assert_allclose(Rm, rs.as_matrix(), atol=1e-12)
            e = _euler_xyz_deg_from_R(Rm)
            recon = R_scipy.from_matrix(_R_from_euler_xyz_deg(float(e[0]), float(e[1]), float(e[2])))
            geo_err = float(np.degrees((recon.inv() * rs).magnitude()))
            self.assertLess(geo_err, 0.15, msg=f"gimbal recon failed for {ang}, err={geo_err:.3f}°")
            e = (e + 180.0) % 360.0 - 180.0
            e_ref = (rs.as_euler("xyz", degrees=True) + 180.0) % 360.0 - 180.0
            if abs(ang[1]) < 89.5:
                np.testing.assert_allclose(e, e_ref, atol=1e-9)
            q = _quat_from_R(Rm)
            q_ref = np.asarray(rs.as_quat(), dtype=np.float64)
            if q_ref[3] < 0.0:
                q_ref = -q_ref
            np.testing.assert_allclose(q, q_ref, atol=1e-12)
            self.assertEqual(_quat_key_arr(q), _quat_key_arr(q_ref))

    def test_project_R_to_anchor_near_gimbal(self):
        """相对旋转 ry≈±90° 时，投影后矩阵重建误差应 < 0.15°（复核文档 3.1）。"""
        R_anc = R_scipy.from_euler("xyz", [30.0, 45.0, 10.0], degrees=True).as_matrix()
        for ry_rel in (89.95, 90.0, -90.0):
            R_rel = R_scipy.from_euler("xyz", [5.0, ry_rel, 0.0], degrees=True).as_matrix()
            R_cand = R_anc @ R_rel
            R_proj = _project_R_to_anchor(R_cand, R_anc, (15.0, 15.0, 180.0))
            R_rel_out = R_anc.T @ R_proj
            recon = R_scipy.from_matrix(_R_from_euler_xyz_deg(*_euler_xyz_deg_from_R(R_rel_out)))
            err = float(np.degrees((recon.inv() * R_scipy.from_matrix(R_rel_out)).magnitude()))
            self.assertLess(err, 0.15, msg=f"matrix recon err={err:.3f}° at ry_rel={ry_rel}")

    def test_branch_key_is_stable_and_not_list_index(self):
        """同一姿态的多组解析解应按肩/肘/腕签名分桶，键值在 0..7。"""
        T = _waypoint_to_T(self.wp0)
        sols = self.solver.inverse_controller_matrix(T)
        keys = [_branch_key(s) for s in sols]
        self.assertTrue(all(0 <= k <= 7 for k in keys))
        self.assertGreaterEqual(len(set(keys)), min(2, len(keys)))

    def test_movel_prefilter_allows_large_total_joint_travel(self):
        """整段 Δq>45° 只要相邻抽检连续、终点同支，边就应可行。"""
        far = dict(self.wp0)
        far["x"] = self.wp0["x"] + 180.0
        T0 = _waypoint_to_T(self.wp0)
        T1 = _waypoint_to_T(far)
        q1 = self.solver.get_best_ik_controller(T1, self.q0)
        if q1 is None:
            self.skipTest("180 mm 平移不可达，无法验证整段行程预筛")
        total = float(np.max(np.abs(np.degrees(_wrap_pi(q1 - self.q0)))))
        node0 = {
            "T": T0,
            "quat": np.asarray(R_scipy.from_matrix(T0[:3, :3]).as_quat(), dtype=np.float64),
            "q_branch": np.array(self.q0, dtype=np.float64),
        }
        node1 = {
            "T": T1,
            "quat": np.asarray(R_scipy.from_matrix(T1[:3, :3]).as_quat(), dtype=np.float64),
            "q_branch": np.array(q1, dtype=np.float64),
        }
        ok, _, q_end = self.opt._check_movel_segment(node0, node1, self.q0)
        travel_lim = min(170.0, max(120.0, 0.9 * 180.0))
        if 45.0 < total <= travel_lim:
            self.assertTrue(ok, f"整段 Δq={total:.1f}° 被误杀（上限 {travel_lim:.0f}°）")
        if ok:
            self.assertIsNotNone(q_end)
            self.assertLess(
                float(np.max(np.abs(np.degrees(_wrap_pi(q_end - q1))))),
                5.0 + 1e-6,
            )


if __name__ == "__main__":
    unittest.main()