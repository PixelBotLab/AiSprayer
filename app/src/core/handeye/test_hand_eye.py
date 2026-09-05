# -*- coding: utf-8 -*-
"""
手眼标定内核的合成数据回归测试。

真实机器人不可用时, 这是唯一能同时验证"坐标系约定 / 单位换算 / 两种安装数学模型 /
重投影误差口径"的手段: 用已知真值正向生成观测, 再让求解器反解, 比对还原误差。

运行:
    cd app/src && ../.venv/bin/python -m unittest core.handeye.test_hand_eye
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from core.handeye import (  # noqa: E402
    EYE_IN_HAND, EYE_TO_HAND, CalibSample, chessboard_object_points, clean_samples,
    evaluate_data_quality, invert_transform, make_transform, pose_to_matrix,
    solve_hand_eye,
)

K = np.array([[611.68, 0.0, 643.43],
              [0.0, 611.69, 405.15],
              [0.0, 0.0, 1.0]], dtype=np.float64)
PATTERN = (8, 11)
SQUARE_MM = 15.0

_OBJP = chessboard_object_points(PATTERN, SQUARE_MM)


def _random_flange_poses(n: int, seed: int = 0) -> list[np.ndarray]:
    """生成 n 个法兰位姿: 位置跨度 ~400mm, 姿态绕互不相同的轴转动 30~70 度。"""
    rng = np.random.default_rng(seed)
    poses = []
    base = Rot.from_euler("xyz", [148.0, 0.0, 98.0], degrees=True).as_matrix()
    for i in range(n):
        offset = rng.uniform(-180.0, 180.0, 3) + np.array([i * 18.0, (i % 3) * 40.0, -i * 12.0])
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = np.radians(rng.uniform(30.0, 70.0))
        R = base @ Rot.from_rotvec(axis * angle).as_matrix()
        T = make_transform(R, np.array([400.0, -90.0, 200.0]) + offset)
        poses.append(T)
    return poses


def _make_samples(T_flange_list, T_camera_board_list, ids=None):
    samples = []
    for i, (T_fl, T_cb) in enumerate(zip(T_flange_list, T_camera_board_list)):
        corners = _project(T_cb)
        samples.append(CalibSample(
            sample_id=(ids[i] if ids is not None else i + 1),
            T_base_flange=T_fl,
            T_camera_board=T_cb,
            pose_dobot=np.concatenate([T_fl[:3, 3],
                                        Rot.from_matrix(T_fl[:3, :3]).as_euler("xyz", degrees=True)]),
            corners_px=corners,
            obj_pts=_OBJP,
            image_file=f"image_{i + 1:03d}.png",
        ))
    return samples


def _project(T_camera_board: np.ndarray) -> np.ndarray:
    pts = T_camera_board[:3, :3] @ _OBJP.T + T_camera_board[:3, 3:4]
    dist = np.zeros(5)
    out, _ = cv2.projectPoints(pts.T.astype(np.float64), np.zeros(3), np.zeros(3), K, dist)
    return out.reshape(-1, 2)


class EyeToHandTest(unittest.TestCase):
    """眼在手外: 相机固定, 标定板装在法兰上。"""

    T_BASE_CAMERA = make_transform(
        Rot.from_euler("xyz", [-89.0, -0.3, -87.8], degrees=True).as_matrix(),
        np.array([135.2, 25.9, 24.5]),
    )
    T_FLANGE_BOARD = make_transform(
        Rot.from_euler("xyz", [0.0, 12.0, -5.0], degrees=True).as_matrix(),
        np.array([-73.9, -10.7, 70.2]),
    )

    def _build(self, n=12, seed=3):
        T_flange = _random_flange_poses(n, seed)
        T_board_base = [T @ self.T_FLANGE_BOARD for T in T_flange]
        T_cam_board = [invert_transform(self.T_BASE_CAMERA) @ T for T in T_board_base]
        return T_flange, T_cam_board

    def test_recovers_camera_extrinsics(self):
        T_flange, T_cam_board = self._build()
        samples = _make_samples(T_flange, T_cam_board)

        sol = solve_hand_eye(EYE_TO_HAND, samples, K=K, D=None)
        self.assertIsNotNone(sol, "solver should return a solution")
        t_err = np.linalg.norm(sol.T_base_camera[:3, 3] - self.T_BASE_CAMERA[:3, 3])
        self.assertLess(t_err, 0.5, f"camera translation off by {t_err:.3f} mm")
        self.assertLess(sol.translation_error_mm, 0.1)
        self.assertLess(sol.rotation_error_deg, 0.1)
        self.assertEqual(sol.euler_order, "xyz")
        self.assertEqual(sol.sign_vector, (1, 1, 1))

    def test_recovers_board_offset(self):
        T_flange, T_cam_board = self._build()
        samples = _make_samples(T_flange, T_cam_board)

        sol = solve_hand_eye(EYE_TO_HAND, samples, K=K, D=None)
        self.assertIsNotNone(sol)
        off_err = np.linalg.norm(sol.board_offset_flange_mm - self.T_FLANGE_BOARD[:3, 3])
        self.assertLess(off_err, 0.5, f"board offset off by {off_err:.3f} mm")

    def test_reports_reprojection_error(self):
        T_flange, T_cam_board = self._build()
        samples = _make_samples(T_flange, T_cam_board)
        sol = solve_hand_eye(EYE_TO_HAND, samples, K=K, D=None)
        self.assertIsNotNone(sol.reprojection_error_px,
                             "true reprojection error should be reported when K is given")
        self.assertLess(sol.reprojection_error_px, 0.5)

    def test_noise_degrades_gracefully(self):
        T_flange, T_cam_board = self._build()
        clean = _make_samples(T_flange, T_cam_board)
        clean_sol = solve_hand_eye(EYE_TO_HAND, clean, K=K)

        rng = np.random.default_rng(11)
        noisy = []
        for s in clean:
            jittered = s.T_camera_board.copy()
            jittered[:3, 3] += rng.normal(0.0, 1.5, 3)
            noisy.append(CalibSample(
                sample_id=s.sample_id, T_base_flange=s.T_base_flange,
                T_camera_board=jittered, pose_dobot=s.pose_dobot,
                corners_px=s.corners_px, obj_pts=s.obj_pts, image_file=s.image_file))
        noisy_sol = solve_hand_eye(EYE_TO_HAND, noisy, K=K)

        self.assertGreater(noisy_sol.translation_error_mm, clean_sol.translation_error_mm)


class EyeInHandTest(unittest.TestCase):
    """眼在手上: 相机装在法兰上, 标定板固定于世界。"""

    T_FLANGE_CAMERA = make_transform(
        Rot.from_euler("xyz", [90.0, 0.0, -90.0], degrees=True).as_matrix(),
        np.array([45.0, -12.0, 68.0]),
    )
    T_BASE_BOARD = make_transform(
        Rot.from_euler("xyz", [0.0, 0.0, 15.0], degrees=True).as_matrix(),
        np.array([550.0, 10.0, -50.0]),
    )

    def _build(self, n=12, seed=5):
        T_flange = _random_flange_poses(n, seed)
        T_cam_board = [invert_transform(T @ self.T_FLANGE_CAMERA) @ self.T_BASE_BOARD
                       for T in T_flange]
        return T_flange, T_cam_board

    def test_recovers_hand_eye_transform(self):
        T_flange, T_cam_board = self._build()
        samples = _make_samples(T_flange, T_cam_board)

        sol = solve_hand_eye(EYE_IN_HAND, samples, K=K, D=None)
        self.assertIsNotNone(sol, "eye-in-hand solver returned None")

        t_err = np.linalg.norm(sol.T_flange_camera[:3, 3] - self.T_FLANGE_CAMERA[:3, 3])
        r_err = np.degrees(np.arccos(np.clip(
            (np.trace(sol.T_flange_camera[:3, :3].T @ self.T_FLANGE_CAMERA[:3, :3]) - 1.0) / 2.0,
            -1.0, 1.0)))
        self.assertLess(t_err, 1.0, f"camera-on-flange translation off by {t_err:.3f} mm")
        self.assertLess(r_err, 0.5, f"camera-on-flange rotation off by {r_err:.3f} deg")

        board_t = np.linalg.norm(sol.T_base_board[:3, 3] - self.T_BASE_BOARD[:3, 3])
        self.assertLess(board_t, 1.0, f"board pose translation off by {board_t:.3f} mm")

    def test_projection_chain_closes(self):
        """由 T_flange_camera 与 T_base_board 反推每帧板在相机系的位姿, 应与真值观测一致。"""
        T_flange, T_cam_board = self._build()
        samples = _make_samples(T_flange, T_cam_board)
        sol = solve_hand_eye(EYE_IN_HAND, samples, K=K, D=None)

        worst_mm = 0.0
        for T_fl, truth in zip(T_flange, T_cam_board):
            T_base_cam_i = T_fl @ sol.T_flange_camera
            predicted = invert_transform(T_base_cam_i) @ sol.T_base_board
            worst_mm = max(worst_mm, float(np.linalg.norm(
                predicted[:3, 3] - truth[:3, 3])))
        self.assertLess(worst_mm, 2.0,
                        f"closed-loop board pose drifts up to {worst_mm:.3f} mm")
        self.assertIsNotNone(sol.reprojection_error_px,
                             "true reprojection error should be reported when K is given")
        self.assertLess(sol.reprojection_error_px, 0.5,
                        f"noiseless samples reproject at {sol.reprojection_error_px:.3f} px")

    def test_detects_rotation_degeneracy(self):
        """所有样本绕同一轴旋转时 AX=XB 退化, 必须被 axis_coverage 抓到。"""
        rng = np.random.default_rng(7)
        axis = np.array([0.0, 0.0, 1.0])
        T_flange = []
        for i in range(10):
            R = Rot.from_rotvec(axis * np.radians(rng.uniform(0.0, 120.0))).as_matrix()
            T_flange.append(make_transform(R, np.array([400.0 + i * 20, -90.0, 200.0])))
        T_cam_board = [invert_transform(T @ self.T_FLANGE_CAMERA) @ self.T_BASE_BOARD
                       for T in T_flange]
        samples = _make_samples(T_flange, T_cam_board)

        quality = evaluate_data_quality(samples, EYE_IN_HAND)
        self.assertLess(quality["axis_coverage"], 0.30,
                        "single-axis samples must be flagged as degenerate")
        self.assertTrue(quality["degenerate"])


class SharedQualityTest(unittest.TestCase):
    def test_cleaner_keeps_rotated_and_drops_blunders(self):
        """旋转丰富的样本必须留下 (老的比例判据会误删), 粗差必须在窄包络下被剔除。"""
        T_flange, T_cam_board = EyeToHandTest()._build(n=6, seed=1)
        samples = _make_samples(T_flange, T_cam_board)

        kept = clean_samples(samples, threshold=0.05, motion_envelope_mm=500.0)
        self.assertEqual(len(kept), 6, "large-angle samples must not be discarded")

        bad = list(samples)
        tampered = bad[2].T_camera_board.copy()
        tampered[:3, 3] += np.array([300.0, 0.0, 0.0])
        bad[2] = CalibSample(sample_id=bad[2].sample_id, T_base_flange=bad[2].T_base_flange,
                            T_camera_board=tampered, pose_dobot=bad[2].pose_dobot,
                            corners_px=bad[2].corners_px, obj_pts=bad[2].obj_pts,
                            image_file=bad[2].image_file)
        tightened = clean_samples(bad, threshold=0.05, motion_envelope_mm=90.0)
        self.assertNotIn(3, [s.sample_id for s in tightened],
                         "a 300mm corner blunder must be dropped under the bootstrapped envelope")

    def test_pose_to_matrix_matches_dobot_convention(self):
        """位姿->矩阵->位姿 自洽, 且与 CR5 FK 的 'xyz' 内禀序列一致。"""
        pose = [400.0, -90.0, 200.0, 148.0, 0.0, 98.0]
        T = pose_to_matrix(pose)
        expected = Rot.from_euler("xyz", pose[3:], degrees=True).as_matrix()
        self.assertTrue(np.allclose(T[:3, :3], expected))
        self.assertTrue(np.allclose(T[:3, 3], pose[:3]))

    def test_radian_input_is_normalized(self):
        from core.handeye import UNIT_RAD
        pose_rad = [400.0, -90.0, 200.0, np.radians(148.0), 0.0, np.radians(98.0)]
        T_rad = pose_to_matrix(pose_rad, angle_unit=UNIT_RAD)
        T_deg = pose_to_matrix([400.0, -90.0, 200.0, 148.0, 0.0, 98.0])
        self.assertTrue(np.allclose(T_rad, T_deg, atol=1e-9))


if __name__ == "__main__":
    unittest.main(verbosity=2)
