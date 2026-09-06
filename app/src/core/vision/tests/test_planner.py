"""WaypointPlanner 3D 喷涂航点与轨迹规划单元测试（纯 CPU、无外部模型权重依赖）。"""

from __future__ import annotations

import unittest

import cv2
import numpy as np
import trimesh

from core.vision.planner import (
    WaypointPlanner,
    WaypointPlannerError,
    split_jeans_mask,
)


def _identity_k(w=200, h=200, f=200.0) -> np.ndarray:
    return np.array([[f, 0.0, w / 2.0], [0.0, f, h / 2.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _front_camera_T() -> np.ndarray:
    """相机位于 base (1.2, 0, 0)，沿 -X 方向朝向原点。"""
    r = np.array([
        [0.0, 0.0, -1.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ], dtype=np.float64)
    t = np.array([1.2, 0.0, 0.0], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = r
    T[:3, 3] = t
    return T


def _two_leg_boxes() -> trimesh.Trimesh:
    """在 YZ 平面站立的两只细长盒子（模拟左右两腿）。"""
    left = trimesh.creation.box(extents=(0.02, 0.08, 0.25))
    left.apply_translation([0.85, 0.10, 0.0])
    right = trimesh.creation.box(extents=(0.02, 0.08, 0.25))
    right.apply_translation([0.85, -0.10, 0.0])
    return trimesh.util.concatenate([left, right])


def _synthetic_pants_mask(h=200, w=200, frayed_left=False) -> np.ndarray:
    """构造人字形裤子掩码：腰部 y:40..70, 左腿 x:40..85, 右腿 x:115..160, 裆部凹陷至 y=100。"""
    mask = np.zeros((h, w), dtype=bool)
    # 腰部
    mask[40:70, 40:160] = True
    # 左腿
    mask[70:180, 40:85] = True
    # 右腿
    mask[70:180, 115:160] = True

    if frayed_left:
        # 在左腿膝盖处切断 2 个像素 (y=120..122) 模拟猫须磨损或褶皱阴影
        mask[120:122, 40:85] = False

    return mask


class TestWaypointPlanner(unittest.TestCase):
    def test_missing_kt_raises(self):
        planner = WaypointPlanner(dedup_radius_mm=20.0)
        mesh = trimesh.creation.box(extents=(0.1, 0.1, 0.1))
        with self.assertRaises(WaypointPlannerError):
            planner.plan(mesh, {"masks": [{"polygons": [[[0, 0], [10, 0], [10, 10]]]}]})

    def test_empty_masks_raises(self):
        planner = WaypointPlanner(
            camera_intrinsics=_identity_k(),
            T_camera_to_base=np.eye(4),
            image_size=(200, 200),
            dedup_radius_mm=20.0,
        )
        mesh = trimesh.creation.box(extents=(0.1, 0.1, 0.1))
        with self.assertRaises(WaypointPlannerError):
            planner.plan(mesh, {"masks": []})

    def test_plan_single_path_and_does_not_split_mesh(self):
        mesh = _two_leg_boxes()
        n_faces = int(len(mesh.faces))
        k = _identity_k(320, 240, 280.0)
        t = _front_camera_T()

        planner = WaypointPlanner(
            spray_dist_mm=150.0,
            row_spacing_mm=25.0,
            point_spacing_mm=40.0,
            image_size=(320, 240),
            camera_intrinsics=k,
            T_camera_to_base=t,
            dedup_radius_mm=20.0,
            align_outer_edge=True,
        )

        uv, z_ok = planner._project_vertices(
            np.asarray(mesh.vertices, dtype=np.float64), k, t
        )
        good = uv[z_ok]
        self.assertGreater(good.shape[0], 10)
        xs = np.clip(good[:, 0], 2, 317)
        ys = np.clip(good[:, 1], 2, 237)
        poly = [
            [int(xs.min()), int(ys.min())],
            [int(xs.max()), int(ys.min())],
            [int(xs.max()), int(ys.max())],
            [int(xs.min()), int(ys.max())],
        ]
        masks = {"masks": [{"polygons": [poly]}]}
        out = planner.plan(mesh, masks)

        # 验证原 mesh 拓扑不被修改
        self.assertEqual(int(len(mesh.faces)), n_faces)
        self.assertEqual(out["type"], "auto")
        self.assertEqual(len(out["paths"]), 1)
        pts = out["paths"][0]["points"]
        self.assertGreater(len(pts), 4)
        self.assertIn("tcp_pose_base", pts[0])
        self.assertEqual(
            set(pts[0]["tcp_pose_base"]), {"x", "y", "z", "rx", "ry", "rz"}
        )

    def test_crotch_split(self):
        """验证 2D 裤裆凸缺陷识别与左右腿切分。"""
        mask = _synthetic_pants_mask()
        legs = split_jeans_mask(mask, depth_threshold_ratio=0.1, overlap_px=0.0)
        self.assertEqual(len(legs), 2)
        # 左腿主要在 x:40..85，右腿主要在 x:115..160
        self.assertTrue(legs[0][100, 60])
        self.assertFalse(legs[0][100, 140])
        self.assertTrue(legs[1][100, 140])
        self.assertFalse(legs[1][100, 60])

    def test_crotch_split_keeps_frayed_leg(self):
        """回归测试 §6.2 Bug 2：带有褶皱微小断口的裤腿不会被粗暴丢弃。"""
        # 左腿在 y=120..122 处断开为上下两段
        mask_frayed = _synthetic_pants_mask(frayed_left=True)
        legs = split_jeans_mask(
            mask_frayed, depth_threshold_ratio=0.1, overlap_px=0.0
        )
        self.assertEqual(len(legs), 2)
        mask_left = legs[0]
        # 大腿部分 (y=90) 与小腿部分 (y=150) 必须同时被保留！
        self.assertTrue(mask_left[90, 60], "大腿部分应被保留")
        self.assertTrue(mask_left[150, 60], "断开的小腿部分不得被丢弃")

    def test_split_vx_zero(self):
        """回归测试 v_vec 偏角 clip 之后的二次 v_norm 早退保护。

        当两个凸缺陷端点 x 相同时（例如上下堆叠腿或极窄竖带），
        max_vy=0.25*|v_vec[0]| 会将 v_vec[1] 也 clip 到 0，导致 v_norm=0 与
        signed_distance 除零。本用例锁定：
        1) 当前上游 span 过滤已经排除该分支，函数应安全回到单掩码；
        2) 即使未来放宽 span 过滤，新加的二次 v_norm 早退保护也不会写入 NaN/Inf。
        """
        # 场景 1：上下堆叠双腿，凸缺陷 span 仅在 Y 方向 (start[0]==end[0])
        mask_vert = np.zeros((200, 200), dtype=bool)
        mask_vert[40:80, 60:140] = True     # 上股
        mask_vert[120:180, 60:140] = True   # 下股
        mask_vert[80:120, 95:105] = True    # 中间细腰 (制造垂直方向缺陷)
        legs = split_jeans_mask(
            mask_vert, depth_threshold_ratio=0.1, overlap_px=0.0
        )
        self.assertGreaterEqual(len(legs), 1)
        for m in legs:
            self.assertEqual(m.shape, mask_vert.shape)
            self.assertEqual(m.dtype, bool)
            # 布尔掩码中不得出现任何非布尔位（防止 signed_distance 归零写入 NaN）
            self.assertTrue(np.isin(m, [True, False]).all())

        # 场景 2：极窄竖带 + 顶部横带，尝试将分界线推近垂直
        mask_thin = np.zeros((200, 200), dtype=bool)
        mask_thin[40:180, 95:105] = True
        mask_thin[40:60, 30:170] = True
        legs = split_jeans_mask(
            mask_thin, depth_threshold_ratio=0.1, overlap_px=0.0
        )
        self.assertGreaterEqual(len(legs), 1)
        for m in legs:
            self.assertEqual(m.shape, mask_thin.shape)
            self.assertEqual(m.dtype, bool)
            self.assertTrue(np.isin(m, [True, False]).all())

    def test_tcp_pose_continuity_and_no_singularity(self):
        """回归测试 §6.1 Bug 1：连续旋转法向量轨迹不产生 90° 姿态奇异跳变。"""
        planner = WaypointPlanner()
        k = _identity_k(200, 200)
        t = np.eye(4, dtype=np.float64)

        # 构造一条在 X-Z 平面内旋转法向量的连续样本 (从仰角 -45° 旋转到 +45°)
        # 期间经过 normal = [0, 0, 1]，即 z_tool 与全局 Z 轴完全重合的敏感奇异区域
        angles = np.linspace(-np.pi / 4, np.pi / 4, 25)
        samples = []
        for i, a in enumerate(angles):
            nx = float(np.sin(a))
            nz = float(np.cos(a))
            samples.append(
                {
                    "point": [0.5, float(i) * 0.01, 0.5],
                    "normal": [nx, 0.0, nz],
                    "is_jump": False,
                }
            )

        waypoints = planner._samples_to_waypoints(samples, k, t)
        self.assertEqual(len(waypoints), len(samples))

        # 检查所有相邻点之间的 Euler 角变化率，严禁出现 90° 或 180° 的突变跳跃
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]["tcp_pose_base"]
            p2 = waypoints[i + 1]["tcp_pose_base"]

            def angle_diff(a1, a2):
                diff = abs(a1 - a2) % 360.0
                return min(diff, 360.0 - diff)

            delta_rx = angle_diff(p1["rx"], p2["rx"])
            delta_ry = angle_diff(p1["ry"], p2["ry"])
            delta_rz = angle_diff(p1["rz"], p2["rz"])

            # 正常连续变化步长约在 3~5° 之间，若发生奇异翻转会导致 >= 80° 的突变
            self.assertLess(
                delta_rx,
                30.0,
                f"rx sudden snap at point {i}->{i+1}: {p1['rx']} vs {p2['rx']}",
            )
            self.assertLess(
                delta_ry,
                30.0,
                f"ry sudden snap at point {i}->{i+1}: {p1['ry']} vs {p2['ry']}",
            )
            self.assertLess(
                delta_rz,
                30.0,
                f"rz sudden snap at point {i}->{i+1}: {p1['rz']} vs {p2['rz']}",
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
