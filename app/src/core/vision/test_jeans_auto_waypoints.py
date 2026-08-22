"""JeansAutoWaypoints 单元测试。"""

import os
import sys
import unittest

import numpy as np
import trimesh

_HERE = os.path.dirname(os.path.abspath(__file__))
_APP_SRC = os.path.abspath(os.path.join(_HERE, "../.."))
if _APP_SRC not in sys.path:
    sys.path.insert(0, _APP_SRC)

from core.vision.jeans_auto_waypoints import JeansAutoWaypoints, JeansAutoWaypointsError


def _identity_k(w=200, h=200, f=200.0):
    return np.array([[f, 0.0, w / 2.0], [0.0, f, h / 2.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _front_camera_T():
    """Camera at +X looking toward -X, so base +X is depth-ish after RotZ(180)-like map.

    Simpler: camera frame = a pose at (1.2, 0, 0) looking at origin along -X.
    p_cam = R.T @ (p_base - t), with camera +Z along -X_base.
    """
    # Camera origin in base, +Z_cam = -X_base, +Y_cam = -Y_base, +X_cam = +Z_base
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


def _two_leg_boxes():
    """Two thin boxes standing in YZ, separated in Y (left / right legs)."""
    left = trimesh.creation.box(extents=(0.02, 0.08, 0.25))
    left.apply_translation([0.85, 0.10, 0.0])
    right = trimesh.creation.box(extents=(0.02, 0.08, 0.25))
    right.apply_translation([0.85, -0.10, 0.0])
    return trimesh.util.concatenate([left, right])


class TestJeansAutoWaypoints(unittest.TestCase):
    def test_missing_kt_raises(self):
        planner = JeansAutoWaypoints(dedup_radius_mm=20.0)
        mesh = trimesh.creation.box(extents=(0.1, 0.1, 0.1))
        with self.assertRaises(JeansAutoWaypointsError):
            planner.plan(mesh, {"masks": [{"polygons": [[[0, 0], [10, 0], [10, 10]]]}]})

    def test_empty_masks_raises(self):
        planner = JeansAutoWaypoints(
            camera_intrinsics=_identity_k(),
            T_camera_to_base=np.eye(4),
            image_size=(200, 200),
            dedup_radius_mm=20.0,
        )
        mesh = trimesh.creation.box(extents=(0.1, 0.1, 0.1))
        with self.assertRaises(JeansAutoWaypointsError):
            planner.plan(mesh, {"masks": []})

    def test_plan_single_path_and_does_not_split_mesh(self):
        mesh = _two_leg_boxes()
        n_faces = int(len(mesh.faces))
        k = _identity_k(320, 240, 280.0)
        t = _front_camera_T()
        # Project a few vertices to build a covering polygon
        planner = JeansAutoWaypoints(
            spray_dist_mm=150.0,
            row_spacing_mm=25.0,
            point_spacing_mm=40.0,
            image_size=(320, 240),
            camera_intrinsics=k,
            T_camera_to_base=t,
            dedup_radius_mm=20.0,
            align_outer_edge=True,
        )
        uv, z_ok = planner._project_vertices(np.asarray(mesh.vertices, dtype=np.float64))
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
        self.assertEqual(int(len(mesh.faces)), n_faces)
        self.assertEqual(out["type"], "auto")
        self.assertEqual(len(out["paths"]), 1)
        pts = out["paths"][0]["points"]
        self.assertGreater(len(pts), 4)
        self.assertIn("tcp_pose_base", pts[0])
        self.assertEqual(set(pts[0]["tcp_pose_base"]), {"x", "y", "z", "rx", "ry", "rz"})


if __name__ == "__main__":
    unittest.main()
