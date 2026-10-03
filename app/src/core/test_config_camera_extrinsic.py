# -*- coding: utf-8 -*-
"""运行时配置层的外参出口: 两装法共存时"谁读哪一份"的归属。

判定本体都在 core.handeye.resolve_camera_extrinsic (内核自己有一组用例), 本文件只守
配置层这一圈的**归属** —— 也就是"切换装法"与"两装法同时用"能不能成立的那几件事:
1. 全局生效结果 (spraying.calib_path) 是眼在手上时, T_camera_to_base 必须给 None,
   绝不能回一个"看似合理"的常量 (眼在手上时相机位姿根本不是常量);
2. T_camera_to_base_at(法兰位姿) 与交互式重建走同一条解析链, 复合结果一致;
3. follow 侧只读 follow.runtime.calib_path 那一份, 与全局结果互不牵连 —— 这正是
   "跟随用 E2H、交互用 EIH" 能同时生效的前提;
4. follow.runtime.calib_path 未配时退回 spraying.calib_path (保持历史行为)。

SprayerConfig 是进程内单例, calib_* 是普通实例属性: 用例改完必须还原, 否则污染同
批次其它用例 (与 test_reconstruction_calibration_choice 同一套约束)。

运行:
    cd app/src && ../.venv/bin/python -m unittest core.test_config_camera_extrinsic
"""
from __future__ import annotations

import os
import sys
import tempfile
import unittest
from contextlib import contextmanager
from unittest.mock import patch

import numpy as np
import yaml

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from core.config import PROJECT_ROOT, SprayerConfig  # noqa: E402

# 相机相对法兰的安装量 (平移 mm): 旋转取 Rz(90), 复合后的期望值可手算
T_FLANGE_CAMERA = [[0.0, -1.0, 0.0, 0.0],
                   [1.0, 0.0, 0.0, -35.0],
                   [0.0, 0.0, 1.0, 120.0],
                   [0.0, 0.0, 0.0, 1.0]]
# 基座系常量外参 (平移 mm), 只出现在眼在手外的结果里
T_BASE_CAMERA = [[0.0, 0.0, 1.0, 120.0],
                 [-1.0, 0.0, 0.0, 0.0],
                 [0.0, -1.0, 0.0, 800.0],
                 [0.0, 0.0, 0.0, 1.0]]
# 采集时刻的法兰位姿 [x, y, z, rx, ry, rz] (mm, deg), 恒等姿态 -> 平移直接相加
FLANGE_POSE_MM_DEG = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
ZERO_DEG = [0.0] * 6


def _eye_to_hand() -> dict:
    return {"T_base_camera": T_BASE_CAMERA,
            "metadata": {"hand_eye_mount": "eye-to-hand"}}


def _eye_in_hand() -> dict:
    return {"T_flange_camera": T_FLANGE_CAMERA,
            "metadata": {"hand_eye_mount": "eye-in-hand"}}


@contextmanager
def pinned_cfg(**attrs):
    """把单例上的实例属性临时改成用例要的样子, 退出即原样还原。"""
    cfg = SprayerConfig()
    saved = {key: (key in cfg.__dict__, cfg.__dict__.get(key)) for key in attrs}
    for key, val in attrs.items():
        setattr(cfg, key, val)
    try:
        yield cfg
    finally:
        for key, (had, val) in saved.items():
            if had:
                setattr(cfg, key, val)
            else:
                cfg.__dict__.pop(key, None)


@contextmanager
def runtime_offsets(deg):
    """钉住运行时口径 (关节零位偏移 deg); 本机真实值不参与任何断言。"""
    with patch.object(SprayerConfig, "robot_joint_offsets_deg",
                      property(lambda self: list(deg))):
        yield


class ActiveResultTest(unittest.TestCase):
    """全局生效结果: 装法决定 T_camera_to_base / T_camera_to_base_at 的可用性。"""

    def test_eye_in_hand_gives_no_constant(self):
        """眼在手上时返回一个常量就是错的落点: 必须 None + 指向 T_camera_to_base_at。"""
        with pinned_cfg(calib_data=_eye_in_hand()), runtime_offsets(ZERO_DEG):
            self.assertIsNone(SprayerConfig().T_camera_to_base)

    def test_eye_to_hand_returns_the_constant_in_meters(self):
        with pinned_cfg(calib_data=_eye_to_hand()), runtime_offsets(ZERO_DEG):
            T = SprayerConfig().T_camera_to_base
        self.assertAlmostEqual(T[0][3], 0.12, places=9)
        self.assertAlmostEqual(T[2][3], 0.80, places=9)

    def test_at_composes_the_mount_extrinsic_on_the_flange_pose(self):
        """给定法兰位姿时复合 (平移 mm -> m): 与交互式重建解出的是同一个 T。"""
        with pinned_cfg(calib_data=_eye_in_hand()), runtime_offsets(ZERO_DEG):
            T = SprayerConfig().T_camera_to_base_at(FLANGE_POSE_MM_DEG)
        self.assertAlmostEqual(T[0][3], 0.100, places=9)
        self.assertAlmostEqual(T[1][3], 0.165, places=9)
        self.assertAlmostEqual(T[2][3], 0.420, places=9)

    def test_at_without_pose_returns_none(self):
        """没给法兰位姿就没法知道相机在哪: None (而不是拿 0 位姿硬算一个)。"""
        with pinned_cfg(calib_data=_eye_in_hand()), runtime_offsets(ZERO_DEG):
            self.assertIsNone(SprayerConfig().T_camera_to_base_at(None))

    def test_no_calibration_at_all_yields_nothing(self):
        with pinned_cfg(calib_data={}), runtime_offsets(ZERO_DEG):
            self.assertIsNone(SprayerConfig().T_camera_to_base)
            self.assertIsNone(SprayerConfig().T_camera_to_base_at(FLANGE_POSE_MM_DEG))


class FollowExtrinsicTest(unittest.TestCase):
    """follow 侧外参独立: 装法切换与"两装法同时用"都不能把跟随的轴映射带崩。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.e2h_path = self._write("follow_e2h.yaml", _eye_to_hand())
        self.eih_path = self._write("follow_eih.yaml", _eye_in_hand())

    def _write(self, name: str, data: dict) -> str:
        path = os.path.join(self._tmp.name, name)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f)
        return path

    def _use_follow_path(self, path: str):
        """把 follow_calib_path 钉成给定文件: follow.runtime.* 怎么解由下面两个用例单独管。"""
        return patch.object(SprayerConfig, "follow_calib_path",
                            property(lambda self, p=path: p))

    def test_follow_reads_its_own_result_when_global_is_eye_in_hand(self):
        """共存的那一格: 交互用 EIH, 跟随仍拿到自己的 E2H 常量, 互不牵连。"""
        with pinned_cfg(calib_data=_eye_in_hand(), calib_path=self.eih_path), \
                self._use_follow_path(self.e2h_path), runtime_offsets(ZERO_DEG):
            T = SprayerConfig().follow_camera_to_base
        self.assertAlmostEqual(T[0][3], 0.12, places=9)

    def test_follow_degrades_to_none_when_its_result_is_eye_in_hand(self):
        """眼在手上没有常量可用: 返 None 让调用方明确降级, 而不是悄悄给个错的轴映射。"""
        with pinned_cfg(calib_data=_eye_to_hand(), calib_path=self.e2h_path), \
                self._use_follow_path(self.eih_path), runtime_offsets(ZERO_DEG):
            self.assertIsNone(SprayerConfig().follow_camera_to_base)

    def test_follow_path_falls_back_to_the_active_result(self):
        """未单独配 follow.runtime.calib_path 时退回 spraying.calib_path (历史行为)。"""
        with pinned_cfg(calib_path=self.e2h_path, config_data={}):
            self.assertEqual(SprayerConfig().follow_calib_path, self.e2h_path)

    def test_follow_path_prefers_its_own_entry(self):
        """配了就用配的, 且相对路径按仓库根解析。"""
        with pinned_cfg(calib_path=self.e2h_path,
                        config_data={"follow": {"runtime": {"calib_path": "configs/x.yaml"}}}):
            self.assertEqual(SprayerConfig().follow_calib_path,
                             os.path.join(PROJECT_ROOT, "configs/x.yaml"))

    def test_missing_follow_file_yields_none_instead_of_an_error(self):
        """文件不存在时 _load_yaml 给空字典: 外参解不出就是 None (调用方负责降级)。"""
        gone = os.path.join(self._tmp.name, "not_written.yaml")
        # config_data 也钉空: 否则 follow.runtime.calib_path 会从本机真实配置里拿到那一份
        with pinned_cfg(calib_data={}, calib_path=gone, config_data={}), \
                runtime_offsets(ZERO_DEG):
            self.assertIsNone(SprayerConfig().follow_camera_to_base)

    def test_follow_result_is_read_even_when_global_result_is_empty(self):
        """生效结果还没发布 (calib_data 空) 时, 跟随照样能用自己那一份。"""
        with pinned_cfg(calib_data={}, calib_path=None), \
                self._use_follow_path(self.e2h_path), runtime_offsets(ZERO_DEG):
            T = SprayerConfig().follow_camera_to_base
        self.assertTrue(np.allclose(np.array(T)[:3, :3],
                                     np.array(T_BASE_CAMERA)[:3, :3]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
