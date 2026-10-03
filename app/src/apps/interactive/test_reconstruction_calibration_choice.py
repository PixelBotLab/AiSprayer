# -*- coding: utf-8 -*-
"""交互式重建取外参的两级链路守卫: 先解已发布生效结果, 不可用才扫 data/calib 兜底。

两级各有各的闸, 合起来要守住六件事 ——
1. 生效结果 (spraying.calib_path) 优先, 两种装法都能被它解析: 眼在手外直接取基座系常量,
   眼在手上按该 scan 采集时刻记录的法兰位姿复合; 解不出来当场报错, 绝不静默降级;
2. 兜底那条链 (扫 session 文件) 绕过 SprayerConfig 的加载守卫, 所以自己把装法与口径各判一遍:
   只吃眼在手外, 眼在手跳过且在 desc 里点名;
3. 兜底优先取口径一致的最新一份, 不一致的那份要留下痕迹;
4. 一份都不一致时沿用最新的, 但把降级写进 desc (E2H 运行期不消费法兰位姿, 只提示不阻断);
5. 采集时落的 hand_eye 溯源块是眼在手上唯一的像素→基座依据, 没块/装法不符/口径不符/缺位姿
   四道拒用闸都要给出现场能自己修的英文原因;
6. 平移 mm -> m 的换算只在 core.handeye.resolve_camera_extrinsic 一处发生。

运行:
    cd app/src && ../.venv/bin/python -m apps.interactive.test_reconstruction_calibration_choice
"""
from __future__ import annotations

import os
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

import apps.interactive.reconstruction_service as rs_mod  # noqa: E402  (patch 本模块里的 FK 入口)
from apps.interactive.reconstruction_service import (  # noqa: E402
    HAND_EYE_BLOCK, HandEyeCalibrationError, InteractiveReconstructionService,
)
from apps.robot.services.robot_service import robot_service  # noqa: E402
from core.config import SprayerConfig  # noqa: E402

# 本臂 (二手 CR5) 实测的零位偏移量级; j6 是规范自由度故钉 0
DQ_DEG = [-0.706, -0.953, -1.488, -0.061, -1.958, 0.0]
V1 = "controller_v1"
V2 = "joint_offset_v2"


def _result(mount="eye-to-hand", frame=None, offsets=None, tz_mm=300.0):
    """最小可用的一份标定结果: 装法 + 口径 (缺字段即历史 v1) + 基座系外参。"""
    meta = {"hand_eye_mount": mount, "reprojection_error_mm": 1.0}
    if frame:
        meta["pose_frame_convention"] = frame
        meta["joint_offsets_deg"] = offsets
    return {"T_base_camera": [[1.0, 0.0, 0.0, 800.0], [0.0, 1.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0, tz_mm], [0.0, 0.0, 0.0, 1.0]],
            "metadata": meta}


# 相机相对法兰的安装量 (平移 mm): 刻意非零且与轴不平行 —— 复合顺序写错必然验不过
T_FLANGE_CAMERA = [[0.0, -1.0, 0.0, 0.0],
                   [1.0, 0.0, 0.0, -35.0],
                   [0.0, 0.0, 1.0, 120.0],
                   [0.0, 0.0, 0.0, 1.0]]
# 采集那一刻的法兰位姿 [x, y, z, rx, ry, rz] (mm, deg): 取恒等姿态, 期望值可手算
FLANGE_AT_CAPTURE = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
JOINTS_DEG = [10.0, -20.0, 45.0, 5.0, 30.0, -15.0]
ZERO_DEG = [0.0] * 6


def _eih_result(frame=V2, offsets=DQ_DEG, with_x=True):
    """眼在手上结果: 只带法兰系安装外参 X, 没有基座系常量 (运行时得靠法兰位姿复合)。"""
    meta = {"hand_eye_mount": "eye-in-hand", "reprojection_error_mm": 0.833}
    if frame:
        meta["pose_frame_convention"] = frame
        meta["joint_offsets_deg"] = offsets
    data = {"metadata": meta}
    if with_x:
        data["T_flange_camera"] = T_FLANGE_CAMERA
    return data


def _capture_block(**over):
    """采集时应当落盘的 hand_eye 块; 逐用例只改要触发的那道闸。"""
    block = {"mount": "eye-in-hand", "pose_frame_convention": V2,
             "flange_pose_mm_deg": list(FLANGE_AT_CAPTURE),
             "flange_joints_deg": list(JOINTS_DEG),
             "calibration_source": "configs/calib/published.yaml"}
    block.update(over)
    return block


class ConfigPinningTestCase(unittest.TestCase):
    """SprayerConfig 是进程内单例, calib_path/calib_data 是普通实例属性: 改了必须还原。"""

    def setUp(self):
        self.cfg = SprayerConfig()

    def _pin_cfg(self, **attrs):
        for key, val in attrs.items():
            had = key in self.cfg.__dict__
            saved = self.cfg.__dict__.get(key)
            setattr(self.cfg, key, val)
            self.addCleanup(self._restore_cfg, key, had, saved)

    @staticmethod
    def _restore_cfg(key, had, saved):
        cfg = SprayerConfig()
        if had:
            setattr(cfg, key, saved)
        else:
            cfg.__dict__.pop(key, None)


class LatestCalibrationSelectionTest(ConfigPinningTestCase):
    """兜底那条链: 没有可用生效结果时, 按文件自挑 session 的装法与口径守卫。"""

    def setUp(self):
        super().setUp()
        self.svc = InteractiveReconstructionService()
        self._tmp = tempfile.TemporaryDirectory()
        self.svc.calib_dir = self._tmp.name
        # 本类只测兜底链: 把生效结果钉空, 否则 step 0 会直接吃本机真实那一份 (装法还可能
        # 当场报错), 用例就随环境漂了。
        self._pin_cfg(calib_path=None, calib_data={})
        # 已发布全局槽位与本链路的"最新 session"判定无关, 固定为 None 保持断言干净
        self._pub = patch.object(InteractiveReconstructionService,
                                 "_published_session_name", return_value=None)
        self._pub.start()
        self.addCleanup(self._pub.stop)
        self.addCleanup(self._tmp.cleanup)

    def _seed(self, sessions: dict):
        """sessions: {session 名: 结果字典}, 逐个落成 calibration_result.yaml。"""
        for name, data in sessions.items():
            d = os.path.join(self._tmp.name, name)
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, "calibration_result.yaml"), "w",
                      encoding="utf-8") as f:
                yaml.safe_dump(data, f)

    def _load(self, offsets_deg):
        with patch.object(SprayerConfig, "robot_joint_offsets_deg",
                          property(lambda self: offsets_deg)):
            return self.svc.get_latest_calibration()

    def test_legacy_result_is_used_without_caveat_when_offsets_disabled(self):
        """零回归: 未启用偏移 (全零) 时, 无口径字段的历史结果原样选用, desc 不带任何注记。"""
        self._seed({"20260901_000000": _result()})
        T, intr_k, desc = self._load([0.0] * 6)
        self.assertEqual(desc, "20260901_000000 (Reprojection Error: 1.000 mm)")
        self.assertIsNone(intr_k)
        # 文件里是 mm, 下游统一 m: 换算只发生在这一处
        self.assertAlmostEqual(T[0, 3], 0.8, places=9)
        self.assertAlmostEqual(T[2, 3], 0.3, places=9)

    def test_prefers_latest_session_matching_the_runtime_frame(self):
        """启用偏移后: 更新的 v1 那份必须让位给口径一致的次新结果, 且降级可见。"""
        self._seed({
            "20260903_000000": _result(),                    # 最新, 但是旧口径
            "20260902_000000": _result(frame=V2, offsets=DQ_DEG),
            "20260901_000000": _result(frame=V2, offsets=DQ_DEG),
        })
        _, _, desc = self._load(DQ_DEG)
        self.assertTrue(desc.startswith("20260902_000000"), desc)
        self.assertIn("skipped 1 session(s) with a stale pose frame: 20260903_000000", desc)

    def test_offsets_drifted_after_solve_are_treated_as_stale(self):
        """声明 v2 还要逐值比对偏移量: 口径名相同但配置被改过, 一样算不一致。"""
        drifted = [DQ_DEG[0] + 0.5] + DQ_DEG[1:]
        self._seed({"20260902_000000": _result(frame=V2, offsets=DQ_DEG),    # 按旧偏移解的
                    "20260901_000000": _result(frame=V2, offsets=drifted)})  # 与当前配置逐值一致
        _, _, desc = self._load(drifted)
        self.assertTrue(desc.startswith("20260901_000000"), desc)
        self.assertIn("skipped 1 session(s) with a stale pose frame: 20260902_000000", desc)

    def test_falls_back_to_newest_with_warning_when_nothing_matches(self):
        """全都不一致时仍要用起来 (现场可能要一份将就), 但必须把不匹配写进 desc。"""
        self._seed({"20260902_000000": _result(), "20260901_000000": _result()})
        T, _, desc = self._load(DQ_DEG)
        self.assertTrue(desc.startswith("20260902_000000"), desc)
        self.assertIn("WARNING: no session matches the configured pose frame", desc)
        self.assertIn(V2, desc)
        self.assertIn("re-run the calibration", desc)
        self.assertIsNotNone(T)

    def test_eye_in_hand_results_are_skipped_and_reported(self):
        """EIH 没有恒定的 T_base_camera, 混进按文件自挑的链路会被当基座系外参用错。"""
        self._seed({"20260902_000000": _result(mount="eye-in-hand"),
                    "20260901_000000": _result()})
        _, _, desc = self._load([0.0] * 6)
        self.assertTrue(desc.startswith("20260901_000000"), desc)
        self.assertIn("skipped 1 eye-in-hand session(s): 20260902_000000", desc)

    def test_v2_declaration_without_offsets_is_not_trusted(self):
        """声明 v2 却带不出可用偏移: 无法证明口径, 按不一致处理。"""
        self._seed({"20260902_000000": _result(frame=V2, offsets=[]),
                    "20260901_000000": _result(frame=V2, offsets=DQ_DEG)})
        _, _, desc = self._load(DQ_DEG)
        self.assertTrue(desc.startswith("20260901_000000"), desc)
        self.assertIn("skipped 1 session(s) with a stale pose frame: 20260902_000000", desc)

    def test_session_without_base_extrinsic_is_skipped(self):
        """没有基座系外参的结果文件对本链路无意义, 直接跳过而不是抛穿请求。"""
        bare = {"metadata": {"hand_eye_mount": "eye-to-hand"}}
        self._seed({"20260902_000000": bare, "20260901_000000": _result()})
        _, _, desc = self._load([0.0] * 6)
        self.assertTrue(desc.startswith("20260901_000000"), desc)

    def test_broken_yaml_does_not_abort_the_scan(self):
        """坏文件写不出合法 yaml: 记日志跳过, 继续找下一份可用的。"""
        bad_dir = os.path.join(self._tmp.name, "20260902_000000")
        os.makedirs(bad_dir, exist_ok=True)
        with open(os.path.join(bad_dir, "calibration_result.yaml"), "w",
                  encoding="utf-8") as f:
            f.write("T_base_camera: [this is: not, a matrix\n")
        self._seed({"20260901_000000": _result()})
        _, _, desc = self._load([0.0] * 6)
        self.assertTrue(desc.startswith("20260901_000000"), desc)

    def test_no_result_at_all_falls_back_to_identity(self):
        """既没有生效结果也扫不到 session: 退回单位阵 (desc 里写明未标定), 不能抛穿请求。"""
        empty = tempfile.TemporaryDirectory()
        self.addCleanup(empty.cleanup)
        self.svc.calib_dir = empty.name
        with patch.object(SprayerConfig, "robot_joint_offsets_deg",
                          property(lambda self: [0.0] * 6)):
            T, _, desc = self.svc.get_latest_calibration()
        self.assertEqual(desc, "Identity (Uncalibrated)")
        self.assertTrue(np.allclose(T, np.eye(4)))


class ActiveCalibrationResolutionTest(ConfigPinningTestCase):
    """生效结果优先那条链: 两种装法都从 spraying.calib_path 那一份解析, 解不出就当场失败。"""

    def setUp(self):
        super().setUp()
        self.svc = InteractiveReconstructionService()
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.svc.calib_dir = os.path.join(self._tmp.name, "sessions")
        os.makedirs(self.svc.calib_dir, exist_ok=True)
        self._pin_cfg(calib_path=os.path.join(self._tmp.name, "published.yaml"),
                      calib_data={})

    def _seed_session(self, name, data):
        """往兜底目录里落一份 session 结果: 用来证明生效结果不会被它抢位。"""
        d = os.path.join(self.svc.calib_dir, name)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "calibration_result.yaml"), "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f)

    def _scan(self, hand_eye=None):
        """造模板目录并落 scan.params.yaml; hand_eye=None = 压根没写这个块 (旧格式采集)。"""
        d = os.path.join(self._tmp.name, "template")
        os.makedirs(d, exist_ok=True)
        pdata = {"camera_params": {"intrinsic_matrix": [[600.0, 0.0, 640.0],
                                                       [0.0, 600.0, 400.0],
                                                       [0.0, 0.0, 1.0]]}}
        if hand_eye is not None:
            pdata[HAND_EYE_BLOCK] = hand_eye
        with open(os.path.join(d, "scan.params.yaml"), "w", encoding="utf-8") as f:
            yaml.safe_dump(pdata, f)
        return d

    def _load(self, data, offsets_deg, template_path):
        """把 data 钉成生效结果并在给定运行时口径下解析一次。"""
        self._pin_cfg(calib_data=data)
        with patch.object(SprayerConfig, "robot_joint_offsets_deg",
                          property(lambda self: list(offsets_deg))):
            return self.svc.get_latest_calibration(template_path)

    def test_published_eye_to_hand_wins_over_session_scan(self):
        """改一行配置就即刻生效: 生效结果可用时绝不再扫 session, 否则“点的”与“走的”两套外参。"""
        self._seed_session("20260901_000000", _result(tz_mm=999.0))
        T, intr_k, desc = self._load(_result(tz_mm=300.0), ZERO_DEG, self._scan())
        self.assertAlmostEqual(T[2, 3], 0.3, places=9)
        self.assertIn("published.yaml", desc)
        self.assertNotIn("20260901_000000", desc)
        # scan.params.yaml 里的内参不属于本方法的输出 (重建侧自己读), 保持 None
        self.assertIsNone(intr_k)

    def test_eye_in_hand_composes_with_capture_flange_pose(self):
        """EIH 解析 = T_base_flange(采集时) · T_flange_camera: 平移与旋转都从那张图来。"""
        T, _, desc = self._load(_eih_result(), DQ_DEG, self._scan(_capture_block()))
        # 法兰姿态为恒等 -> 平移直接相加; 复合顺序反了或 mm->m 少除一次都过不了这一行
        self.assertAlmostEqual(T[0, 3], 0.100, places=9)
        self.assertAlmostEqual(T[1, 3], 0.165, places=9)
        self.assertAlmostEqual(T[2, 3], 0.420, places=9)
        self.assertTrue(np.allclose(T[:3, :3], np.array(T_FLANGE_CAMERA)[:3, :3]))
        self.assertIn("eye-in-hand", desc)
        self.assertIn("flange pose from capture", desc)

    def test_eye_in_hand_without_hand_eye_block_is_rejected(self):
        """没溯源块 = 不知道相机当时在哪: 宁可报错, 也不能拿旧外参或单位阵静默降级。"""
        with self.assertRaises(HandEyeCalibrationError) as cm:
            self._load(_eih_result(), DQ_DEG, self._scan())
        self.assertIn(f"no '{HAND_EYE_BLOCK}' block", str(cm.exception))

    def test_eye_in_hand_rejects_scan_from_the_other_mount(self):
        """相机重新装过但沿用了旧图: 采集装法与生效结果不一致就是拿错杆臂。"""
        with self.assertRaises(HandEyeCalibrationError) as cm:
            self._load(_eih_result(), DQ_DEG,
                       self._scan(_capture_block(mount="eye-to-hand")))
        self.assertIn("remounting", str(cm.exception))

    def test_eye_in_hand_rejects_scan_captured_under_the_other_pose_frame(self):
        """采完图后又改了 joint_offsets_deg: 法兰位姿与标定不同源, 错一整个 Δq 在杆臂上的投影。"""
        with self.assertRaises(HandEyeCalibrationError) as cm:
            self._load(_eih_result(), DQ_DEG,
                       self._scan(_capture_block(pose_frame_convention=V1)))
        self.assertIn("joint_offsets_deg changed after capture", str(cm.exception))

    def test_eye_in_hand_rejects_block_without_flange_pose(self):
        """有块但没位姿 (臂未连接时采的废图): 同样不能本身就能“算得出数”就放过去。"""
        with self.assertRaises(HandEyeCalibrationError) as cm:
            self._load(_eih_result(), DQ_DEG,
                       self._scan(_capture_block(flange_pose_mm_deg=None)))
        self.assertIn("no flange_pose_mm_deg", str(cm.exception))

    def test_eye_in_hand_requires_the_template_directory(self):
        """没传模板目录 = 没地方取拍摄时的法兰位姿: 用法错误必须说清。"""
        with self.assertRaises(HandEyeCalibrationError) as cm:
            self._load(_eih_result(), DQ_DEG, None)
        self.assertIn("Pass the template directory", str(cm.exception))

    def test_eye_in_hand_result_without_x_is_rejected(self):
        """标称 EIH 却带不出 T_flange_camera: 文件坏了, 不能当成 E2H 的常量用。"""
        with self.assertRaises(HandEyeCalibrationError) as cm:
            self._load(_eih_result(with_x=False), DQ_DEG, self._scan(_capture_block()))
        self.assertIn("T_flange_camera is missing", str(cm.exception))

    def test_eye_in_hand_refuses_to_compose_across_pose_frames(self):
        """结果是控制器口径而运行时带 Δq: 硬拦, 不能只提醒。"""
        with self.assertRaises(HandEyeCalibrationError) as cm:
            self._load(_eih_result(frame=V1, offsets=None), DQ_DEG,
                       self._scan(_capture_block()))
        self.assertIn(V1, str(cm.exception))

    def test_eye_to_hand_with_stale_pose_frame_is_used_with_warning(self):
        """E2H 运行期不消费法兰位姿: 口径旧了只影响标定时被臂误差吸收的那一小部分, 提示不阻断。"""
        self._seed_session("20260901_000000", _result(frame=V2, offsets=DQ_DEG))
        T, _, desc = self._load(_result(frame=None), DQ_DEG, self._scan())
        self.assertAlmostEqual(T[0, 3], 0.8, places=9)
        self.assertIn("WARNING", desc)
        self.assertIn(V1, desc)

    def test_no_published_result_falls_through_to_session_scan(self):
        """生效结果缺失时兜底链仍然工作: 两种装法共存不能把历史行为弄断。"""
        self._seed_session("20260901_000000", _result(tz_mm=300.0))
        T, _, desc = self._load({}, ZERO_DEG, self._scan())
        self.assertAlmostEqual(T[2, 3], 0.3, places=9)
        self.assertTrue(desc.startswith("20260901_000000"), desc)


class CaptureProvenanceTest(ConfigPinningTestCase):
    """采集时落的 hand_eye 溯源块: 眼在手上必须当场记下法兰位姿, 拿不到就拒绝拍这张图。"""

    def setUp(self):
        super().setUp()
        self.svc = InteractiveReconstructionService()
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self._pin_cfg(calib_path=os.path.join("configs", "calib", "published.yaml"),
                      calib_data={})

    def test_eye_to_hand_capture_records_mount_without_touching_robot(self):
        """E2H 的相机相对基座不动: 法兰位姿对成像没意义, 不该为了溯源去读臂。"""
        self._pin_cfg(calib_data=_result())
        with patch.object(robot_service, "get_current_joint",
                          side_effect=AssertionError("must not read the robot")), \
                patch.object(SprayerConfig, "robot_joint_offsets_deg",
                             property(lambda self: ZERO_DEG)):
            block = self.svc.capture_hand_eye_provenance()
        self.assertEqual(block["mount"], "eye-to-hand")
        self.assertEqual(block["pose_frame_convention"], V1)
        self.assertIsNone(block["flange_pose_mm_deg"])
        self.assertIsNone(block["flange_joints_deg"])
        self.assertIn("published.yaml", block["calibration_source"])

    def test_eye_in_hand_capture_records_flange_pose(self):
        """EIH 的 3D 尺度只在拍摄那一刻存在: 当场 FK(q+Δq) 反推法兰位姿落盘。"""
        self._pin_cfg(calib_data=_eih_result())
        with patch.object(SprayerConfig, "robot_joint_offsets_deg",
                          property(lambda self: DQ_DEG)), \
                patch.object(robot_service, "get_current_joint",
                             return_value=(list(JOINTS_DEG), "")), \
                patch.object(rs_mod, "flange_pose_from_joints",
                             return_value=list(FLANGE_AT_CAPTURE)) as fk:
            block = self.svc.capture_hand_eye_provenance()
        fk.assert_called_once_with(list(JOINTS_DEG))
        self.assertEqual(block["mount"], "eye-in-hand")
        self.assertEqual(block["pose_frame_convention"], V2)
        self.assertEqual(block["flange_pose_mm_deg"], FLANGE_AT_CAPTURE)
        self.assertEqual(block["flange_joints_deg"], JOINTS_DEG)

    def test_eye_in_hand_capture_refuses_without_robot(self):
        """臂没连接就不拍废图: 当场拒绝并给出现场能自己修的英文理由。"""
        self._pin_cfg(calib_data=_eih_result())
        with patch.object(SprayerConfig, "robot_joint_offsets_deg",
                          property(lambda self: DQ_DEG)), \
                patch.object(robot_service, "get_current_joint",
                             return_value=(None, "Robot is not connected")):
            with self.assertRaises(HandEyeCalibrationError) as cm:
                self.svc.capture_hand_eye_provenance()
        msg = str(cm.exception)
        self.assertIn("Robot is not connected", msg)
        self.assertIn("Connect the robot and capture again", msg)

    def test_eye_in_hand_capture_refuses_when_fk_unavailable(self):
        """关节读得出但 FK 不可用 (偏移非法): 同样拒绝, 不能留下一张无尺度的图。"""
        self._pin_cfg(calib_data=_eih_result())
        with patch.object(SprayerConfig, "robot_joint_offsets_deg",
                          property(lambda self: DQ_DEG)), \
                patch.object(robot_service, "get_current_joint",
                             return_value=(list(JOINTS_DEG), "")), \
                patch.object(rs_mod, "flange_pose_from_joints", return_value=None):
            with self.assertRaises(HandEyeCalibrationError) as cm:
                self.svc.capture_hand_eye_provenance()
        self.assertIn("forward kinematics unavailable", str(cm.exception))

    def test_round_trip_of_a_recorded_block_resolves_the_extrinsic(self):
        """写与读必须闭环: 采到的块直接喂回本链路能复合出基座系外参。"""
        self._pin_cfg(calib_data=_eih_result())
        with patch.object(SprayerConfig, "robot_joint_offsets_deg",
                          property(lambda self: DQ_DEG)), \
                patch.object(robot_service, "get_current_joint",
                             return_value=(list(JOINTS_DEG), "")), \
                patch.object(rs_mod, "flange_pose_from_joints",
                             return_value=list(FLANGE_AT_CAPTURE)):
            block = self.svc.capture_hand_eye_provenance()
        template = os.path.join(self._tmp.name, "template")
        os.makedirs(template, exist_ok=True)
        with open(os.path.join(template, "scan.params.yaml"), "w",
                  encoding="utf-8") as f:
            yaml.safe_dump({HAND_EYE_BLOCK: block}, f)
        with patch.object(SprayerConfig, "robot_joint_offsets_deg",
                          property(lambda self: DQ_DEG)):
            T, _, desc = self.svc.get_latest_calibration(template)
        self.assertAlmostEqual(T[2, 3], 0.420, places=9)
        self.assertIn("flange pose from capture", desc)


if __name__ == "__main__":
    unittest.main(verbosity=2)
