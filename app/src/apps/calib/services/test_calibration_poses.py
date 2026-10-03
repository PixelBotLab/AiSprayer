# -*- coding: utf-8 -*-
"""标定服务的法兰位姿来源优先级与 Δq 口径贯通。

手眼约束的 A 矩阵只能有一个来源: 法兰位姿。本文件守住三件事 ——
1. 有 joints 就现场 FK, 绝不信任采集时落盘的 flange_pose 缓存 (偏移一改缓存即脏);
2. 配置的关节零位偏移必须一路贯通到 FK, 且改了配置后缓存的内核实例必须重建;
3. 偏移非法时安全降级 (返回 None 走兜底), 不许把异常抛穿到采集线程。

第 4 条是环上测试的通用约束: 本机配置的 Δq 已启用 (非零), 凡"未校正口径"的假设必须
由测试自己钉住, 绝不能吃环境值 —— 否则换一台机器/改一次配置就假失败。

运行:
    cd app/src && ../.venv/bin/python -m apps.calib.services.test_calibration_poses
"""
from __future__ import annotations

import os
import sys
import unittest
from contextlib import contextmanager
from unittest.mock import patch

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from apps.calib.services.calibration_service import (  # noqa: E402
    CalibrationService, _pose_dict,
)
from core.config import SprayerConfig  # noqa: E402
from core.handeye import UNIT_DEG  # noqa: E402
from core.motion.kinematics import CR5Kinematics, flange_pose_from_joints  # noqa: E402

# 一个远离奇异、便于复现的上报关节角 (度)
JOINTS_DEG = [10.0, -20.0, 45.0, 5.0, 30.0, -15.0]
# 本臂 (二手 CR5) 实测的零位偏移量级, j6 是规范自由度故钉 0
DQ_DEG = [-0.706, -0.953, -1.488, -0.061, -1.958, 0.0]
# 故意与任何 FK 结果都不相干的位姿: 用来冒充"旧口径的缓存 / TCP 读数"
STALE_POSE = [123.4, 56.7, 89.0, 0.0, 0.0, 0.0]
# 控制器口径 (补偿关闭)
ZERO_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


@contextmanager
def runtime_offsets(deg):
    """把运行时口径钉成给定偏移 (deg); 测试不依赖本机配置的真实值。"""
    with patch.object(SprayerConfig, "robot_joint_offsets_deg",
                      property(lambda self: list(deg))):
        yield


class FlangePoseSourceTest(unittest.TestCase):
    def setUp(self):
        self.svc = CalibrationService()

    @staticmethod
    def _sample(**extra):
        s = {"id": 1, "robot_pose": _pose_dict(STALE_POSE)}
        s.update(extra)
        return s

    def test_live_fk_wins_over_stale_cache(self):
        """有 joints 就必须现算: 落盘的 flange_pose 只是那次 FK 的缓存, 口径一改即脏。"""
        s = self._sample(joints=JOINTS_DEG, flange_pose=_pose_dict(STALE_POSE))
        T, pose6, src = self.svc._sample_flange_pose(s, UNIT_DEG)
        self.assertEqual(src, "derived")
        expected = CR5Kinematics(
            joint_offsets_deg=self.svc.sprayer.robot_joint_offsets_deg
        ).flange_pose_controller_deg(JOINTS_DEG)
        self.assertTrue(np.allclose(pose6, expected, atol=1e-6), f"{pose6} != {expected}")
        self.assertFalse(np.allclose(T[:3, 3], STALE_POSE[:3]), "缓存被当成了真值")

    def test_recorded_flange_pose_used_without_joints(self):
        """老会话没 joints 时才吃落盘的 flange_pose (恒为 mm/deg 的法兰读数)。"""
        cached = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        _, pose6, src = self.svc._sample_flange_pose(
            self._sample(flange_pose=_pose_dict(cached)), UNIT_DEG)
        self.assertEqual(src, "flange")
        self.assertTrue(np.allclose(pose6, cached))

    def test_tcp_readout_is_last_resort(self):
        """joints 与 flange_pose 都没有才退回 robot_pose (当前 tool 的 TCP), 来源必须标明。"""
        _, pose6, src = self.svc._sample_flange_pose(self._sample(), UNIT_DEG)
        self.assertEqual(src, "tcp")
        self.assertTrue(np.allclose(pose6, STALE_POSE))

    def test_missing_pose_returns_none(self):
        T, pose6, src = self.svc._sample_flange_pose({"id": 9}, UNIT_DEG)
        self.assertEqual((T, pose6, src), (None, None, "tcp"))

    def test_offsets_take_effect_and_invalidate_cached_kernel(self):
        """配置偏移必须贯通到 FK, 且缓存的内核实例必须随配置重建。"""
        with runtime_offsets(ZERO_DEG):
            pose_off = flange_pose_from_joints(JOINTS_DEG)
        with runtime_offsets(DQ_DEG):
            pose_on = flange_pose_from_joints(JOINTS_DEG)
        self.assertIsNotNone(pose_off)
        self.assertIsNotNone(pose_on)
        shifted = np.linalg.norm(np.asarray(pose_on[:3]) - np.asarray(pose_off[:3]))
        self.assertGreater(shifted, 5.0, "Δq 在杆臂上的平移投影实测约 17mm, 这里没生效")
        ref = CR5Kinematics(joint_offsets_deg=DQ_DEG).flange_pose_controller_deg(JOINTS_DEG)
        self.assertTrue(np.allclose(pose_on, ref, atol=1e-9))

    def test_illegal_offsets_degrade_safely(self):
        """偏移越界时快速失败于内核, 对外只表现为"法兰位姿不可用", 不抛穿采集线程。"""
        with runtime_offsets([99.0] * 6):
            self.assertIsNone(flange_pose_from_joints(JOINTS_DEG))
        # 缓存未被非法实例污染: 恢复合法配置后仍能正常反推
        with runtime_offsets(ZERO_DEG):
            self.assertIsNotNone(flange_pose_from_joints(JOINTS_DEG))


class ResultPoseFrameDeclarationTest(unittest.TestCase):
    """结果文件必须自报口径, 且口径不符的历史结果不能被静默消费。"""

    def setUp(self):
        self.svc = CalibrationService()

    def test_history_result_is_accepted_while_uncorrected(self):
        """未启用偏移时, 无口径字段的老结果 (读作 controller_v1) 必须无警告。"""
        legacy = {"metadata": {"hand_eye_mount": "eye-to-hand"}}
        with runtime_offsets(ZERO_DEG):
            self.assertIsNone(self.svc.sprayer.pose_frame_mismatch(legacy))

    def test_stale_result_is_flagged_once_offsets_enabled(self):
        """启用偏移后, 未校正的老结果必须给出可执行的英文原因 (前端直接展示)。"""
        legacy = {"metadata": {"hand_eye_mount": "eye-to-hand"}}
        with runtime_offsets(DQ_DEG):
            reason = self.svc.sprayer.pose_frame_mismatch(legacy)
        self.assertIsNotNone(reason)
        self.assertIn("re-run the calibration", reason)

    def test_summary_reports_frame_and_warning(self):
        """发布预检读的就是这份摘要: 口径与不匹配原因都必须带出来。"""
        import tempfile
        import yaml
        with runtime_offsets(ZERO_DEG):
            with tempfile.TemporaryDirectory() as d:
                path = os.path.join(d, "calibration_result.yaml")
                with open(path, "w", encoding="utf-8") as f:
                    yaml.safe_dump({"metadata": {"hand_eye_mount": "eye-to-hand"}}, f)
                summary = self.svc._result_summary(path)
        self.assertTrue(summary["exists"])
        self.assertEqual(summary["pose_frame"], "controller_v1")
        self.assertIsNone(summary["pose_frame_warning"])


class RuntimeExtrinsicGateTest(unittest.TestCase):
    """口径闸的作用域: 眼在手要实时复合法兰位姿 → 硬拦; 眼在手外给的是常量 → 只告警。"""

    # 任意一个法兰位姿读数 [x, y, z, rx, ry, rz] (mm, deg)
    FLANGE = [700.0, 100.0, -200.0, 179.0, 3.0, -8.0]

    def setUp(self):
        self.cfg = SprayerConfig()
        self._saved = self.cfg.calib_data
        self._warned = getattr(self.cfg, "_warned_pose_frame", False)

    def tearDown(self):
        self.cfg.calib_data = self._saved
        self.cfg._warned_pose_frame = self._warned

    @staticmethod
    def _e2h_result():
        return {"T_base_camera": [[1.0, 0.0, 0.0, 800.0], [0.0, 1.0, 0.0, 0.0],
                                  [0.0, 0.0, 1.0, -300.0], [0.0, 0.0, 0.0, 1.0]],
                "metadata": {"hand_eye_mount": "eye-to-hand"}}

    @staticmethod
    def _eih_result(frame=None, offsets=None):
        meta = {"hand_eye_mount": "eye-in-hand"}
        if frame:
            meta["pose_frame_convention"] = frame
            meta["joint_offsets_deg"] = offsets
        return {"T_flange_camera": [[1.0, 0.0, 0.0, 20.0], [0.0, 1.0, 0.0, 40.0],
                                    [0.0, 0.0, 1.0, 140.0], [0.0, 0.0, 0.0, 1.0]],
                "metadata": meta}

    def test_eye_to_hand_runtime_survives_a_frame_mismatch(self):
        """E2H 运行期不消费法兰位姿: 口径不符也必须照常给常量 (只告警), 不能喷歪在用的产线。"""
        self.cfg.calib_data = self._e2h_result()
        with runtime_offsets(DQ_DEG):
            T = self.cfg.T_camera_to_base_at(self.FLANGE)
        self.assertIsNotNone(T)
        self.assertEqual(T, self.cfg.T_camera_to_base)
        self.assertAlmostEqual(T[0][3], 0.8, places=9)   # mm -> m

    def test_eye_in_hand_refuses_a_stale_frame_and_accepts_a_matching_one(self):
        """EIH 必须硬拦旧口径结果 (错一整个 Δq), 口径对上后又能正常复合。"""
        self.cfg.calib_data = self._eih_result()
        with runtime_offsets(DQ_DEG):
            self.assertIsNone(self.cfg.T_camera_to_base_at(self.FLANGE))
        with runtime_offsets(ZERO_DEG):
            self.assertIsNotNone(self.cfg.T_camera_to_base_at(self.FLANGE))

    def test_eye_in_hand_refuses_when_offsets_changed_after_solve(self):
        """结果声明了 v2 但配置里的偏移被改过: 同样拦 (不匹配就是不可用)。"""
        self.cfg.calib_data = self._eih_result(frame="joint_offset_v2", offsets=DQ_DEG)
        drifted = [DQ_DEG[0] + 0.5] + DQ_DEG[1:]
        with runtime_offsets(drifted):
            self.assertIsNone(self.cfg.T_camera_to_base_at(self.FLANGE))
        with runtime_offsets(DQ_DEG):
            self.assertIsNotNone(self.cfg.T_camera_to_base_at(self.FLANGE))


if __name__ == "__main__":
    unittest.main(verbosity=2)
