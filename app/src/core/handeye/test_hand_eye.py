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
    EYE_IN_HAND, EYE_TO_HAND, POSE_FRAME_CONTROLLER_V1, POSE_FRAME_JOINT_OFFSET_V2,
    CalibSample, chessboard_object_points, declare_pose_frame, euler_deg_from_rotation,
    evaluate_data_quality, invert_transform, make_transform, minimum_samples,
    pixel_ray_to_base, pose_frame_mismatch, pose_to_matrix, prune_outliers,
    readout_rotation_consistency,
    resolve_camera_extrinsic, resolve_result_mount, resolve_result_pose_frame,
    rotation_angle_deg, rotation_closest_to_with_z_axis,
    rotation_from_pose, solve_hand_eye, tool_euler_deg_from_z_axis, tool_frame_from_z_axis,
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
    def test_readout_consistency_needs_no_extrinsic(self):
        """共轭不改旋转角: 自洽数据的臂读数与视觉观测必须一致, 与装法/外参无关。"""
        T_flange, T_cam_board = EyeToHandTest()._build(n=8, seed=1)
        samples = _make_samples(T_flange, T_cam_board)

        cons = readout_rotation_consistency(samples)
        self.assertIsNotNone(cons, "8 rotated samples must give comparable pairs")
        self.assertLess(cons["max_deg"], 0.05,
                        f"self-consistent readouts disagree by {cons['max_deg']:.3f} deg")

        # 篡改一帧法兰姿态读数 (模拟位姿链路矛盾), 不重新拟合也要能抓出来
        broken = list(samples)
        broken[3] = CalibSample(**{**broken[3].__dict__,
                                  "T_base_flange": make_transform(
                                      Rot.from_euler("z", 20.0, degrees=True).as_matrix()
                                      @ samples[3].T_base_flange[:3, :3],
                                      samples[3].T_base_flange[:3, 3])})
        cons_broken = readout_rotation_consistency(broken)
        self.assertGreater(cons_broken["max_deg"], 10.0,
                           "a 20 deg pose readout blunder must show up as an arm-vs-camera gap")
        self.assertIn(samples[3].sample_id, cons_broken["per_sample_deg"],
                      "the offending sample must be attributable for row-level flagging")

    def test_prune_drops_samples_by_own_residual(self):
        """粗差样本必须由首轮拟合对自身的重投影残差抓出来, 裁剪后重解回到真值。"""
        T_flange, T_cam_board = EyeToHandTest()._build(n=8, seed=1)
        samples = _make_samples(T_flange, T_cam_board)

        # 第 4 帧当成角点误检: 位姿与像素都来自一个偏离 120mm 的假板
        tampered = T_cam_board[3].copy()
        tampered[:3, 3] += np.array([120.0, 0.0, 0.0])
        samples[3].T_camera_board = tampered
        samples[3].corners_px = _project(tampered)

        sol = solve_hand_eye(EYE_TO_HAND, samples, K=K, D=None)
        self.assertIsNotNone(sol)
        errors = [sol.per_sample_reprojection_px.get(i) for i in range(len(samples))]
        self.assertEqual(len(errors), len(samples),
                         "every sample must carry its own residual for pruning")
        worst = int(np.nanargmax([e if e is not None else -1.0 for e in errors]))
        self.assertEqual(worst, 3, "the tampered frame must be the worst own-residual")

        kept, dropped = prune_outliers(samples, errors,
                                       max_error=float(errors[3]) / 2.0,
                                       unit="px",
                                       min_keep=minimum_samples(EYE_TO_HAND))
        self.assertEqual(dropped, [samples[3].sample_id])
        self.assertEqual(len(kept), 7)

        clean = solve_hand_eye(EYE_TO_HAND, kept, K=K, D=None)
        t_err = np.linalg.norm(clean.T_base_camera[:3, 3] - EyeToHandTest.T_BASE_CAMERA[:3, 3])
        self.assertLess(t_err, 1.0, f"pruned solve drifts {t_err:.3f} mm from the truth")

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


class PoseFrameConventionTest(unittest.TestCase):
    """法兰位姿口径的读出与拦截 —— 与装法正交的第二个维度, 错用是静默的。"""

    DQ = [-0.706, -0.953, -1.488, -0.061, -1.958, 0.0]

    def test_history_without_field_is_controller_v1(self):
        """历史文件 (没写字段 / 写在 metadata 里的 v1) 必须读作 controller_v1。

        绝不能“缺字段 = 跟当前运行时口径”, 否则改一次配置就会让所有旧结果自动变口径。
        """
        for data in ({}, {"metadata": {}}, {"metadata": {"pose_frame_convention": "controller_v1"}},
                     {"pose_frame_convention": "nonsense"}):
            got = resolve_result_pose_frame(data)
            self.assertEqual(got["frame"], POSE_FRAME_CONTROLLER_V1, data)
            self.assertEqual(got["joint_offsets_deg"], [0.0] * 6)

    def test_declared_v2_reads_offsets_from_either_level(self):
        self.assertEqual(
            resolve_result_pose_frame({"pose_frame_convention": POSE_FRAME_JOINT_OFFSET_V2,
                                       "joint_offsets_deg": self.DQ})["joint_offsets_deg"],
            self.DQ)
        self.assertEqual(
            resolve_result_pose_frame({"metadata": {"pose_frame_convention": POSE_FRAME_JOINT_OFFSET_V2,
                                                    "joint_offsets_deg": self.DQ}})["frame"],
            POSE_FRAME_JOINT_OFFSET_V2)

    def test_v2_without_usable_offsets_is_not_silently_zero(self):
        """声明 v2 但偏移缺失/形状错 -> None, 调用方必须当不匹配处理。"""
        for bad in ({}, {"joint_offsets_deg": [1.0, 2.0]}, {"joint_offsets_deg": ["x"] * 6}):
            data = {"pose_frame_convention": POSE_FRAME_JOINT_OFFSET_V2, **bad}
            self.assertIsNone(resolve_result_pose_frame(data)["joint_offsets_deg"], data)

    def test_mismatch_gate(self):
        v1_file = {"metadata": {}}
        v2_file = {"metadata": {"pose_frame_convention": POSE_FRAME_JOINT_OFFSET_V2,
                                "joint_offsets_deg": self.DQ}}
        zero = [0.0] * 6

        # 没结果 / 未配偏移: 一律放行 (不能拦住改造前就一直正常的路径)
        self.assertIsNone(pose_frame_mismatch({}, POSE_FRAME_CONTROLLER_V1, zero))
        self.assertIsNone(pose_frame_mismatch(None, POSE_FRAME_JOINT_OFFSET_V2, self.DQ))
        self.assertIsNone(pose_frame_mismatch(v1_file, POSE_FRAME_CONTROLLER_V1, zero))
        self.assertIsNone(pose_frame_mismatch(v2_file, POSE_FRAME_JOINT_OFFSET_V2, self.DQ))

        # 旧结果 + 开了补偿: 拦, 且原因里告知怎么恢复
        hit = pose_frame_mismatch(v1_file, POSE_FRAME_JOINT_OFFSET_V2, self.DQ)
        self.assertIsNotNone(hit)
        self.assertIn("re-run the calibration", hit)

        # 新结果 + 改了/清了偏移: 拦
        self.assertIsNotNone(pose_frame_mismatch(v2_file, POSE_FRAME_CONTROLLER_V1, zero))
        drift = pose_frame_mismatch(v2_file, POSE_FRAME_JOINT_OFFSET_V2,
                                    [self.DQ[0] + 0.1] + self.DQ[1:])
        self.assertIsNotNone(drift)
        self.assertIn("joint offsets", drift)

        # 声明 v2 但无偏移值: 拦
        broken = pose_frame_mismatch({"pose_frame_convention": POSE_FRAME_JOINT_OFFSET_V2},
                                     POSE_FRAME_JOINT_OFFSET_V2, self.DQ)
        self.assertIsNotNone(broken)
        self.assertIn("no usable", broken)

    def test_unknown_runtime_frame_falls_back_to_v1(self):
        """运行时口径传了认不出的值, 按 controller_v1 处理 (宁保守不可谎报)。"""
        self.assertIsNone(pose_frame_mismatch({"metadata": {}}, "garbage", [0.0] * 6))
        self.assertIsNotNone(pose_frame_mismatch(
            {"pose_frame_convention": POSE_FRAME_JOINT_OFFSET_V2,
             "joint_offsets_deg": self.DQ}, "garbage", [0.0] * 6))

    def test_declare_frame_requires_all_live_fk(self):
        """口径声明: 只有全部样本都是现场 FK 才能计 v2, 混口径一律降级。"""
        # (现场 FK, 旧缓存, TCP 读数, 样本总数)
        self.assertEqual(declare_pose_frame(POSE_FRAME_CONTROLLER_V1, 5, 0, 0, 5),
                         POSE_FRAME_CONTROLLER_V1)
        self.assertEqual(declare_pose_frame(POSE_FRAME_JOINT_OFFSET_V2, 5, 0, 0, 5),
                         POSE_FRAME_JOINT_OFFSET_V2)
        for mixed in [(4, 1, 0, 5), (4, 0, 1, 5), (5, 0, 0, 6), (0, 0, 0, 0)]:
            self.assertEqual(declare_pose_frame(POSE_FRAME_JOINT_OFFSET_V2, *mixed),
                             POSE_FRAME_CONTROLLER_V1, mixed)


class CameraExtrinsicResolutionTest(unittest.TestCase):
    """
    两装法共用的外参解析入口: 一份结果 (+ 可选法兰位姿) -> T_base_camera。

    这是全项目唯一一处 mm -> m 的换算, 也是唯一一处“装法决定能不能降级”的判定:
    眼在手外返基座系常量 (口径旧了只提醒), 眼在手上必须靠拍摄时刻的法兰位姿复合
    (缺位姿或口径不同源都硬拦, 因为降级出来的是一个错位的落点而不是缺数据)。
    """

    DQ = [-0.706, -0.953, -1.488, -0.061, -1.958, 0.0]
    T_BASE_CAMERA = [[0.0, 0.0, 1.0, 120.0],
                     [-1.0, 0.0, 0.0, 0.0],
                     [0.0, -1.0, 0.0, 800.0],
                     [0.0, 0.0, 0.0, 1.0]]
    # 相机相对法兰的安装量 (mm): 旋转恰好是 Rz(90), 方便手算复合后的期望值
    T_FLANGE_CAMERA = [[0.0, -1.0, 0.0, 0.0],
                       [1.0, 0.0, 0.0, -35.0],
                       [0.0, 0.0, 1.0, 120.0],
                       [0.0, 0.0, 0.0, 1.0]]

    def _e2h(self, key="T_base_camera", frame=None):
        data = {key: self.T_BASE_CAMERA, "metadata": {"hand_eye_mount": EYE_TO_HAND}}
        if frame:
            data["metadata"]["pose_frame_convention"] = frame
        return data

    def _eih(self, with_x=True, frame=POSE_FRAME_JOINT_OFFSET_V2):
        meta = {"hand_eye_mount": EYE_IN_HAND}
        if frame:
            meta["pose_frame_convention"] = frame
            meta["joint_offsets_deg"] = list(self.DQ)
        data = {"metadata": meta}
        if with_x:
            data["T_flange_camera"] = self.T_FLANGE_CAMERA
        return data

    def test_eye_to_hand_returns_the_base_frame_constant_in_meters(self):
        """E2H 的相机相对基座不动: 直接取常量, 平移 mm -> m 只在这一处发生。"""
        T, mount, note = resolve_camera_extrinsic(
            self._e2h(), runtime_frame=POSE_FRAME_CONTROLLER_V1,
            runtime_offsets_deg=[0.0] * 6)
        self.assertEqual(mount, EYE_TO_HAND)
        self.assertIsNone(note)
        self.assertAlmostEqual(T[0][3], 0.12, places=9)
        self.assertAlmostEqual(T[2][3], 0.80, places=9)

    def test_eye_to_hand_ignores_the_flange_pose(self):
        """传了法兰位姿也不能改变 E2H 的结果: 那个位姿对成像没有意义。"""
        T, _m, _n = resolve_camera_extrinsic(self._e2h())
        T2, _m2, _n2 = resolve_camera_extrinsic(self._e2h(), [100.0] * 6)
        self.assertEqual(T, T2)

    def test_legacy_key_spelling_is_still_read(self):
        """历史上两种写法都存过 (T_camera_to_base 里装的其实就是基座系常量)。"""
        T, _mount, note = resolve_camera_extrinsic(self._e2h(key="T_camera_to_base"))
        self.assertIsNone(note)
        self.assertAlmostEqual(T[2][3], 0.80, places=9)

    def test_eye_to_hand_without_any_constant_explains_why(self):
        """没基座系常量的结果对本链路无意义: 给英文原因让界面能直接告知重标。"""
        T, mount, note = resolve_camera_extrinsic({"metadata": {"hand_eye_mount": EYE_TO_HAND}})
        self.assertIsNone(T)
        self.assertEqual(mount, EYE_TO_HAND)
        self.assertIn("no T_base_camera", note)

    def test_empty_result_defaults_to_eye_to_hand(self):
        """空结果不能被认为“可能就是眼在手上”: 默认按此前唯一支持的装法处理。"""
        T, mount, note = resolve_camera_extrinsic(None)
        self.assertEqual(mount, EYE_TO_HAND)
        self.assertIsNone(T)
        self.assertIsNotNone(note)

    def test_eye_in_hand_composes_with_identity_flange_orientation(self):
        """法兰姿态恒等时平移就是相加: 复合顺序写反或 mm/m 混用都过不了这一行。"""
        T, mount, note = resolve_camera_extrinsic(
            self._eih(), [100.0, 200.0, 300.0, 0.0, 0.0, 0.0],
            runtime_frame=POSE_FRAME_JOINT_OFFSET_V2, runtime_offsets_deg=self.DQ)
        self.assertEqual(mount, EYE_IN_HAND)
        self.assertIsNone(note)
        for got, want in zip((T[0][3], T[1][3], T[2][3]), (0.100, 0.165, 0.420)):
            self.assertAlmostEqual(got, want, places=9)

    def test_eye_in_hand_uses_the_dobot_intrinsic_xyz_convention(self):
        """带姿态的法兰位姿: 期望值逐元素手算 (R = [Rz(90)·Rx(90)] · Rz(90)), 顺规或复合顺序
        一错就整片错开, 而平移 (220, 200, 265) mm 同样只能从那个 R 来。"""
        T, _mount, note = resolve_camera_extrinsic(
            self._eih(), [100.0, 200.0, 300.0, 90.0, 0.0, 90.0])
        self.assertIsNone(note)
        self.assertTrue(np.allclose(np.array(T)[:3, :3],
                                     [[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]],
                                     atol=1e-9), T)
        for got, want in zip((T[0][3], T[1][3], T[2][3]), (0.220, 0.200, 0.265)):
            self.assertAlmostEqual(got, want, places=9)

    def test_eye_in_hand_needs_the_capture_flange_pose(self):
        """眼在手上时“相机此刻在哪”不是常量: 缺位姿就没得降级, 只能报错。"""
        T, _mount, note = resolve_camera_extrinsic(self._eih())
        self.assertIsNone(T)
        self.assertIn("flange pose at capture time is required", note)

    def test_eye_in_hand_pose_frame_mismatch_is_a_hard_block(self):
        """结果是控制器口径而运行时带 Δq: 错一整个 Δq 在杆臂上的投影, 必须拦。"""
        T, _mount, note = resolve_camera_extrinsic(
            self._eih(frame=POSE_FRAME_CONTROLLER_V1), [100.0, 200.0, 300.0, 0.0, 0.0, 0.0],
            runtime_frame=POSE_FRAME_JOINT_OFFSET_V2, runtime_offsets_deg=self.DQ)
        self.assertIsNone(T)
        self.assertIn(POSE_FRAME_CONTROLLER_V1, note)

    def test_eye_to_hand_pose_frame_mismatch_is_only_a_note(self):
        """E2H 不消费法兰位姿, 口径旧了只影响标定时被臂误差吸收的那一小部分: 照给常量。"""
        T, _mount, note = resolve_camera_extrinsic(
            self._e2h(frame=POSE_FRAME_CONTROLLER_V1),
            runtime_frame=POSE_FRAME_JOINT_OFFSET_V2, runtime_offsets_deg=self.DQ)
        self.assertIsNotNone(T)
        self.assertIsNotNone(note)
        self.assertAlmostEqual(T[2][3], 0.80, places=9)

    def test_eye_in_hand_result_without_x_explains_why(self):
        """标称 EIH 却带不出安装外参 X: 文件坏了, 不能当成基座系常量用。"""
        T, _mount, note = resolve_camera_extrinsic(self._eih(with_x=False), [100.0] * 6)
        self.assertIsNone(T)
        self.assertIn("T_flange_camera is missing", note)

    def test_mount_of_both_spellings_is_read_the_same_way(self):
        """入口里读到的装法必须与全局消费方共用同一判定 (否则一处认 E2H 另一处认 EIH)。"""
        self.assertEqual(resolve_result_mount(self._eih(), default=EYE_TO_HAND), EYE_IN_HAND)
        self.assertEqual(resolve_result_mount(self._e2h(), default=EYE_IN_HAND), EYE_TO_HAND)
        data = self._eih()
        data.pop("metadata")
        self.assertEqual(resolve_result_mount(data, default=EYE_TO_HAND), EYE_TO_HAND)


class PixelRayGeometryTest(unittest.TestCase):
    """
    像素 -> 基座系观测射线 与 工具姿态构造 的口径回归。

    实时视频“指尖指向”与交互页航点共用这两个入口, 它们错的代价是静默错位 (不报异常),
    所以单位 (m vs mm)、边界、去畸变、自旋退化这几件事必须钉住。
    """

    IMAGE_SIZE = (1280, 800)                     # (width, height) 像素, 与 K 的主点量级一致
    T_IDENTITY = [[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0]]

    def _ray(self, u, v, T=None, D=None, size=IMAGE_SIZE):
        return pixel_ray_to_base(u, v, K, T if T is not None else self.T_IDENTITY, D=D, image_size=size)

    def test_center_pixel_is_the_optical_axis(self):
        """主点像素 = 相机 +Z; 恒等外参下它就是基座 +Z, 射线原点落在光心。"""
        origin, direction = self._ray(K[0, 2], K[1, 2])
        np.testing.assert_allclose(origin, [0.0, 0.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(direction, [0.0, 0.0, 1.0], atol=1e-9)

    def test_ray_origin_is_millimetres_while_extrinsic_is_metres(self):
        """T_base_camera 平移是**米** (resolve_camera_extrinsic 的原生口径), 射线原点必须是**毫米**。"""
        T = [row[:] for row in self.T_IDENTITY]
        T[2][3] = 0.8                                          # 800 mm = 0.8 m
        origin, _direction = self._ray(K[0, 2], K[1, 2], T=T)
        np.testing.assert_allclose(origin, [0.0, 0.0, 800.0], atol=1e-9)

    def test_off_center_pixel_matches_manual_back_projection(self):
        """去畸变缺省时与手算反投影逐分量一致 (方向归一)。"""
        u, v = K[0, 2] + 200.0, K[1, 2] - 100.0
        manual = np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
        manual /= np.linalg.norm(manual)
        _origin, direction = self._ray(u, v)
        np.testing.assert_allclose(direction, manual, atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.norm(direction)), 1.0, places=12)

    def test_extrinsic_rotation_carries_the_ray_into_the_base_frame(self):
        """外参旋转是唯一的坐标系出口: Ry(90) 把相机 +Z 映射到基座 +X (两装法走同一条链)。"""
        T = make_transform(Rot.from_euler('y', 90.0, degrees=True).as_matrix(), [0.1, 0.2, 0.3])
        origin, direction = self._ray(K[0, 2], K[1, 2], T=T)
        np.testing.assert_allclose(origin, [100.0, 200.0, 300.0], atol=1e-9)
        np.testing.assert_allclose(direction, [1.0, 0.0, 0.0], atol=1e-9)

    def test_pixel_outside_the_image_fails_fast(self):
        """越界像素不能静默给个方向: 射线会指到图像外, 机械臂就去撞不存在的东西。"""
        with self.assertRaises(ValueError) as ctx:
            self._ray(self.IMAGE_SIZE[0], 100.0)
        self.assertIn("outside the image bounds", str(ctx.exception))

    def test_distortion_is_removed_before_back_projection(self):
        """给了有效畸变系数就必须先 cv2.undistortPoints: 角上差 1px 在 600mm 深度是几毫米。"""
        corner = (K[0, 2] + 500.0, K[1, 2] + 300.0)
        D = [-0.032, 0.02, 0.0, 0.0, 0.0]
        _o, dir_plain = self._ray(*corner)
        _o, dir_fixed = self._ray(*corner, D=D)
        self.assertGreater(float(np.linalg.norm(dir_plain - dir_fixed)), 1e-4)
        # 主点上的畸变是对称的, 去畸变不该把轴上像素挪走
        _o, dir_center = self._ray(K[0, 2], K[1, 2], D=D)
        np.testing.assert_allclose(dir_center, [0.0, 0.0, 1.0], atol=1e-9)

    def test_bad_matrix_shapes_are_rejected(self):
        """K 不是 3x3 / T 不是 4x4 都当场报错, 不能靠 numpy 广播“算出”一个看似合理的数。"""
        with self.assertRaises(ValueError):
            pixel_ray_to_base(1.0, 1.0, [[1.0, 0.0], [0.0, 1.0]], self.T_IDENTITY)
        with self.assertRaises(ValueError):
            pixel_ray_to_base(1.0, 1.0, K, [[1.0] * 4] * 3)
        with self.assertRaises(ValueError):
            pixel_ray_to_base(1.0, 1.0, K, [[0.0] * 4] * 4)

    def test_tool_frame_is_an_exact_right_handed_basis(self):
        """工具 +Z 就是指尖/喷枪指向: 构造出的 3x3 必须严格正交、行列式 +1、Z 列就是请求的方向。"""
        for z in ([0.0, 0.0, -1.0], [1.0, 1.0, 1.0], [0.3, -0.9, 0.1]):
            R = tool_frame_from_z_axis(z)
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
            np.testing.assert_allclose(R[:, 2], np.array(z) / np.linalg.norm(z), atol=1e-12)

    def test_spin_reference_parallel_to_tool_axis_does_not_degenerate(self):
        """工具 Z 与默认参考轴 (基座 +Z) 平行时叉积退化, 必须改参考轴而不是给出非正交阵。"""
        for z in ([0.0, 0.0, 1.0], [0.0, 0.0, -1.0]):
            R = tool_frame_from_z_axis(z)
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)

    def test_chained_spin_reference_keeps_consecutive_frames_close(self):
        """连续航点传上一点的 x_tool 时, 新姿态的 x 轴必须接着它走 (否则整条路径 Rz 跳 180°)。"""
        R0 = tool_frame_from_z_axis([0.0, 0.6, -0.8])
        R1 = tool_frame_from_z_axis([0.1, 0.6, -0.8], x_ref=R0[:, 0])
        self.assertGreater(abs(float(np.dot(R1[:, 0], R0[:, 0]))), 0.9)
        with self.assertRaises(ValueError):
            tool_frame_from_z_axis([0.0, 0.0, 1.0], x_ref=[1.0, 0.0])
        with self.assertRaises(ValueError):
            tool_frame_from_z_axis([0.0, 0.0, 0.0])

    def test_euler_round_trip_recovers_the_pointing_direction(self):
        """指向 -> Dobot 欧拉 (度, 'xyz' 内禀) -> 旋转, 第三列必须回到原方向 (与机械臂封包同一约定)。

        [0, 1, 0] 是故意选的极点: 此时 pitch=±90° 会触发 gimbal lock (库会打一条 UserWarning
        并把第三个角置 0), 但"工具 +Z 指向"仍然等价 —— 正好钉住我们只依赖指向、自旋可自由选择这一事实。
        """
        for z in ([0.0, 0.0, -1.0], [0.4, -0.5, 0.77], [0.0, 1.0, 0.0]):
            euler_deg = tool_euler_deg_from_z_axis(z)
            self.assertEqual(len(euler_deg), 3)
            self.assertTrue(all(abs(v) <= 180.0 for v in euler_deg))
            np.testing.assert_allclose(rotation_from_pose([0.0, 0.0, 0.0] + euler_deg)[:, 2],
                                       np.array(z) / np.linalg.norm(z), atol=1e-9)

    def test_euler_deg_from_rotation_matches_pose_conventions(self):
        """矩阵 -> 欧拉 -> 矩阵 闭环, 且与 pose_to_matrix 吃同一个顺规 (全项目只有一处定义)。"""
        R = Rot.from_euler('xyz', [20.0, -35.0, 100.0], degrees=True).as_matrix()
        pose_deg = [0.0, 0.0, 0.0] + euler_deg_from_rotation(R)
        np.testing.assert_allclose(rotation_from_pose(pose_deg), R, atol=1e-9)
        np.testing.assert_allclose(pose_to_matrix(pose_deg)[:3, :3], R, atol=1e-9)


class GimbalAimRotationTest(unittest.TestCase):
    """
    云台式指向的姿态构造: 只把工具 +Z 摆到指定方向, 除此之外一律不动。

    实时视频"光束指向" (apps/robot/services/live_aim_service) 靠它把姿态改动压到最小
    —— 摆过去的那一下是必需的, 额外的拧腕只会把腕关节推近限位/奇异。
    """

    def _refs(self):
        return [Rot.from_euler('xyz', e, degrees=True).as_matrix() for e in
                ([0.0, 0.0, 0.0], [37.0, -12.0, 155.0], [-90.0, 45.0, 0.0])]

    def test_the_axis_lands_exactly_on_the_requested_direction(self):
        """不管参考姿态怎么转, 结果必须仍是严格右手正交阵且 +Z = 请求方向。"""
        for R_ref in self._refs():
            for z in ([0.0, 0.0, -1.0], [1.0, 1.0, 1.0], [0.3, -0.9, 0.1], [0.0, 1.0, 0.0]):
                R = rotation_closest_to_with_z_axis(z, R_ref)
                np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
                self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
                np.testing.assert_allclose(R[:, 2], np.array(z) / np.linalg.norm(z), atol=1e-12)

    def test_only_the_swing_is_applied_never_an_extra_spin(self):
        """转过的角度恰好 = 两个 +Z 轴的夹角: 这是可达的最小转动, 多一度都是无妄之灾。"""
        for R_ref in self._refs():
            z = np.array([0.4, -0.5, 0.77])
            z = z / np.linalg.norm(z)
            R = rotation_closest_to_with_z_axis(z, R_ref)
            wanted = float(np.degrees(np.arccos(np.clip(float(np.dot(R_ref[:, 2], z)), -1.0, 1.0))))
            self.assertAlmostEqual(rotation_angle_deg(R_ref, R), wanted, places=9)

    def test_an_already_aligned_pose_is_handed_back_untouched(self):
        """已经指着该指的地方就不许再动: 否则现场会看到一次没意义的腕部翻转。"""
        for R_ref in self._refs():
            R = rotation_closest_to_with_z_axis(R_ref[:, 2], R_ref)
            np.testing.assert_allclose(R, R_ref, atol=1e-12)

    def test_dead_ahead_target_swings_by_180_without_degenerating(self):
        """目标方向与当前 +Z 完全相反 (转轴不唯一): 必须给一个正交右手阵且真的转 180 度。"""
        for R_ref in self._refs():
            R = rotation_closest_to_with_z_axis(-R_ref[:, 2], R_ref)
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
            np.testing.assert_allclose(R[:, 2], -R_ref[:, 2], atol=1e-12)
            self.assertAlmostEqual(rotation_angle_deg(R_ref, R), 180.0, places=6)

    def test_twist_spins_about_the_beam_without_moving_the_axis(self):
        """绕光束轴自旋 (后备拧腕) 只换时钟: 指向不变, 转角就是请求的 twist。"""
        z = np.array([0.2, -0.6, -0.77])
        R_ref = self._refs()[1]
        R0 = rotation_closest_to_with_z_axis(z, R_ref)
        for twist in (45.0, -45.0, 90.0, -90.0):
            R = rotation_closest_to_with_z_axis(z, R_ref, twist_deg=twist)
            np.testing.assert_allclose(R[:, 2], R0[:, 2], atol=1e-12)
            self.assertAlmostEqual(rotation_angle_deg(R0, R), abs(twist), places=9)

    def test_bad_inputs_fail_fast_in_english(self):
        """零向量 / 非 3x3 / 含 nan 的参考旋转都不能逸成“算出一个看似合理的姿态”。"""
        R_ref = np.eye(3)
        with self.assertRaises(ValueError):
            rotation_closest_to_with_z_axis([0.0, 0.0, 0.0], R_ref)
        with self.assertRaises(ValueError):
            rotation_closest_to_with_z_axis([1.0, 0.0, 0.0], np.ones((2, 2)))
        with self.assertRaises(ValueError):
            rotation_closest_to_with_z_axis([1.0, 0.0, float("nan")], R_ref)
        with self.assertRaises(ValueError):
            rotation_closest_to_with_z_axis([1.0, 0.0, 0.0], np.full((3, 3), float("nan")))


if __name__ == "__main__":
    unittest.main(verbosity=2)
