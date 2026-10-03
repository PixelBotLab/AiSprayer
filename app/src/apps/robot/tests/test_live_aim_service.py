# -*- coding: utf-8 -*-
"""
实时视频"光束指向"服务的回归测试 (真机不可用时的唯一验证手段)。

需求口径 (钉死在这里, 免得哪天又被改回"光束只要经过某个猜出来的目标点"):
工具轴线这条光束必须与**观测射线共线** —— 像云台激光笔那样把光斑打在被点的视线上。
单个像素只有方向没有深度, 只要求"光束经过射线上某一点"会留下一个**视差角**: 眼在手上时
光心与 TCP 实测差 150mm, 在 600mm 猜测深度上是 atan(150/600)=14°, 打到 1m 外的工件就是
20~30cm 脱靶 (真机就是这个现象)。共线之后深度猜错也不影响打中, 于是 aim_distance_mm
退化成纯界面标注。

守住的六类会静默出事的地方:
1. 共线性 —— 每个候选的工具 +Z 都必须等于视线方向 (错了不报错, 只是光斑跑到别处);
2. 深度无关 —— 换 depth 不许动机械臂落点 (视差角的直接回归);
3. 可达球口径 —— 半径必须是"法兰臂展 + 实测工具长度", 少算一截工具会把 on-ray 族整族误杀;
   指尖真的摆不上视线时退回 swing (指尖不动、光束斜着穿过被点那个点), 而不是"与视线平行"
   —— 后者整条光束偏开视线, 在任何深度都打不中, 就是真机上"点了不动也不中"的来源;
4. 双装法取数 —— 眼在手上必须吃**当前法兰位姿**, 眼在手外必须是常量外参, 两者不能混;
5. 工业安全顺序 —— 互锁、工具号先于规划生效、立即指令关喷 (fail-close)、
   到位成功后才按配置亮激光, 任何中途失败都不许把 DO 留在开;
6. 人工点选十字中心校核 —— mark_cross_center 把目标点与被点十字各用自己采集时刻的法兰
   反投影回基座系对齐, 拉开“共模标定误差”; 校核失败只能降级成一句实话, 绝不能把一次已
   成功的指向反过来说成失败。

robot_service / sprayer_config / camera_service 全部用 mock 顶掉: 本文件不碰硬件,
也不污染进程内的配置单例。

运行:
    cd app/src && ../.venv/bin/python -m unittest apps.robot.tests.test_live_aim_service
"""
from __future__ import annotations

import math
import os
import sys
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))

from core.handeye import (  # noqa: E402
    EYE_IN_HAND, EYE_TO_HAND, UNIT_RAD, make_transform, matrix_to_pose, normalize_pose,
    rotation_from_pose, tool_frame_from_z_axis,
)

import apps.robot.services.live_aim_service as live_aim_module  # noqa: E402
from apps.robot.services.live_aim_service import LiveAimError, LiveAimService  # noqa: E402

K = np.array([[611.68, 0.0, 643.43],
              [0.0, 611.69, 405.15],
              [0.0, 0.0, 1.0]], dtype=np.float64)
INTRINSICS = {
    "width": 1280,
    "height": 800,
    "intrinsic_matrix": K.tolist(),
    "distortion_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],   # 全零 -> 走纯反投影, 期望值可手算
}
# 相机->基座 (平移**米**): 恒等旋转 + 光心在基座 (100, 0, 500) mm -> 中心像素的射线就是 +Z
T_BASE_CAMERA = [[1.0, 0.0, 0.0, 0.1],
                 [0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0, 0.5],
                 [0.0, 0.0, 0.0, 1.0]]
CENTER_U, CENTER_V = 643.43, 405.15          # 主点像素 = 光轴方向
RAY_ORIGIN_MM = [100.0, 0.0, 500.0]
RAY_DIR = [0.0, 0.0, 1.0]                    # 中心像素的视线方向 (基座系)
DEPTH_MM = 600.0                             # aim_distance_mm: 纯深度标注 (界面回显用)
TARGET_MM = [100.0, 0.0, 1100.0]             # 光心 + 600mm x 视线方向
STANDOFF_MM = 150.0                          # spray_dist_mm: 枪口离镜头的最小安全/工艺距离
TIP_NOW_MM = [0.0, 0.0, 0.0]                 # 假臂默认 TCP (基座原点), 姿态单位阵
FLANGE_NOW_MM = [0.0, 0.0, -153.0]           # 默认 FK 法兰 -> 实测工具长度 153mm (与真机同量级)


def _fake_robot_service(tip_pose: list | None = None) -> MagicMock:
    """一台"空闲、已连接、什么都答应"的假机器: 各用例只覆盖自己关心的那个返回。"""
    svc = MagicMock()
    svc.is_connected.return_value = True
    svc.is_moving.return_value = False
    svc.get_current_joint.return_value = ([0.0] * 6, "")
    pose = TIP_NOW_MM + [0.0, 0.0, 0.0] if tip_pose is None else tip_pose   # mm + rad
    svc.get_current_pose.return_value = (pose, "")
    svc.get_speed.return_value = (100.0, 20.0, 20.0, 25.0)   # speed_l, acc_l, speed_j, acc_j
    svc.max_joint_speed_deg_s = [180.0] * 6
    # 默认“臂早就静止了”: 眼在手上的画面-位姿同源闸门 (settle) 不影响其他几何用例,
    # 闸门自己的行为由下面 LiveAimImagePairingTest 用独立场景钉。
    svc.seconds_since_motion.return_value = 999.0
    svc.check_reachability.return_value = (True, [], "")
    svc.set_tool.return_value = (True, "")
    svc.set_do.return_value = (True, "")
    svc.move_to_pose_j.return_value = (True, "")
    svc.tool_num = 1
    svc.spray_do_index = 3
    return svc


def _fake_config(reach_mm: float = 5000.0) -> MagicMock:
    """
    假配置。臂展默认给 5000mm: 本文件的射线基线是"光心 (100,0,500) 沿 +Z 看", 目标点在基座
    1104mm 处 —— 若给真实的 900mm, 整条基线会被可达球筛掉而干扰共线性断言。
    可达球那套几何 (工具长度扩半径 / 落空退回 swing) 由下面两个专用类用贴近实机的尺寸钉。
    """
    cfg = MagicMock()
    cfg.hand_eye_mount = EYE_TO_HAND
    cfg.aim_distance_mm = DEPTH_MM
    cfg.spray_distance_mm = STANDOFF_MM
    cfg.robot_max_reach_mm = reach_mm
    cfg.aim_speed_percent = 50.0
    cfg.aim_do_on_arrive = True
    cfg.aim_frame_settle_s = 2.0
    cfg.laser_tilt_tool_deg = [0.0, 0.0]   # 离线默认不补偿: 光束==工具+Z, 保持旧断言口径
    cfg.camera_extrinsic_at.return_value = (T_BASE_CAMERA, EYE_TO_HAND, None)
    return cfg


def _healthy_video() -> dict:
    """取流健康: 15fps 交付档位下画面就是当前现场 (闸门只拒绝断流与重启)。"""
    return {"online": True, "streaming": True, "color_fps": 15.0}




def _extrinsic_for(origin_mm, dir_base) -> list:
    """造一个相机->基座外参 (4x4, 平移 m): 光心落在 origin_mm, 相机 +Z (主点视线) 指向 dir_base。

    只靠这两个量就能把"视线与可达球的位置关系"摆成任何想要的形状, 不需要真相机。
    """
    d = np.asarray(dir_base, dtype=np.float64)
    d = d / np.linalg.norm(d)
    helper = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x_axis = np.cross(helper, d)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(d, x_axis)
    R = np.column_stack([x_axis, y_axis, d])        # 第三列 = 相机 +Z 在基座系的朝向
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(origin_mm, dtype=np.float64) / 1000.0   # 外参平移量纲是 m
    return T.tolist()


def _tool_axis(pose_rad: list) -> np.ndarray:
    """候选/反馈姿态的工具 +Z (光束方向), 与欧拉角的表示歧义无关。"""
    return Rot.from_euler("xyz", pose_rad[3:]).as_matrix()[:, 2]


def _spot_error_mm(pose_rad: list, point_mm) -> tuple[float, float]:
    """(光斑垂距 mm, 沿光束距离 mm): 光束没打中那个点时这两个数就是证据。"""
    tip = np.asarray(pose_rad[:3], dtype=np.float64)
    axis = _tool_axis(pose_rad)
    to_point = np.asarray(point_mm, dtype=np.float64) - tip
    along = float(np.dot(to_point, axis))
    return float(np.linalg.norm(to_point - along * axis)), along


class _Patched(unittest.TestCase):
    """公共夹具: 假臂 + 假配置 + 假内参 + 假 FK, 用例只改自己关心的那一处。

    flange_pose 是唯一的 FK 输出旋钮: 它必须与假 TCP 保持"差 153mm"的关系,
    否则 _tool_length_mm 实测出来的工具长度就是一个无意义的斜距。
    """

    reach_mm = 5000.0

    def setUp(self):
        self.service = LiveAimService()
        self.robot = _fake_robot_service()
        self.cfg = _fake_config(reach_mm=self.reach_mm)
        self.flange_pose = FLANGE_NOW_MM
        for p in (
            patch.object(live_aim_module, "robot_service", self.robot),
            patch.object(live_aim_module, "sprayer_config", self.cfg),
            patch.object(live_aim_module, "flange_pose_from_joints",
                         side_effect=lambda _joints: self.flange_pose),
            patch("apps.camera.services.camera_service.camera_service.get_intrinsics_dict",
                  return_value=INTRINSICS),
            patch("apps.camera.services.camera_service.camera_service.get_status",
                  return_value=_healthy_video()),
            # 深度默认读不到: 目标点退回配置靶距 (600mm), 几何用例的期望值才可手算。
            # 要吃实测深度的用例 (兜底族定心 / 光斑自检) 自己再 patch get_depth_frame。
            patch("apps.camera.services.camera_service.camera_service.get_depth_frame",
                  return_value=None),
        ):
            p.start()
            self.addCleanup(p.stop)

    def _use_ray(self, origin_mm, dir_base) -> None:
        self.cfg.camera_extrinsic_at.return_value = (
            _extrinsic_for(origin_mm, dir_base), EYE_TO_HAND, None)

    def _use_tcp(self, tip_mm, rpy_rad=(0.0, 0.0, 0.0)) -> None:
        """把假臂摆到指定 TCP, 并让 FK 法兰保持在其后 153mm (工具长度与真机同量级)。"""
        self.robot.get_current_pose.return_value = (list(tip_mm) + list(rpy_rad), "")
        self.flange_pose = [tip_mm[0], tip_mm[1], tip_mm[2] - 153.0, 0.0, 0.0, 0.0]


class LiveAimCollinearityTest(_Patched):
    """规划阶段的核心不变量: 光束与观测视线共线, 且与深度标注无关。"""

    def test_center_pixel_stands_the_nozzle_on_the_sight_line_at_the_standoff(self):
        plan = self.service.plan(CENTER_U, CENTER_V)
        np.testing.assert_allclose(plan["ray_origin_mm"], RAY_ORIGIN_MM, atol=1e-6)
        np.testing.assert_allclose(plan["ray_dir_base"], RAY_DIR, atol=1e-9)
        np.testing.assert_allclose(plan["target_point_mm"], TARGET_MM, atol=1e-6)
        chosen = plan["chosen"]
        self.assertEqual(chosen["strategy"], "on-ray")
        # 意图站位 = 离当前指尖最近的那个射线点, 但不早于"枪口离镜头一个靶距"的安全下界
        self.assertAlmostEqual(chosen["along_ray_mm"], STANDOFF_MM, delta=0.05)
        np.testing.assert_allclose(chosen["tip_mm"], [100.0, 0.0, 500.0 + STANDOFF_MM], atol=0.05)
        self.assertAlmostEqual(chosen["beam_offset_mm"], 0.0, places=1)     # 共线 -> 视线垂距 0
        np.testing.assert_allclose(rotation_from_pose(chosen["pose_mm_deg"], UNIT_RAD)[:, 2],
                                   RAY_DIR, atol=1e-6)
        spot, depth = _spot_error_mm(chosen["pose_rad"], plan["target_point_mm"])
        self.assertLess(spot, 0.01)
        self.assertGreater(depth, 0.0)

    def test_every_candidate_puts_the_beam_through_the_clicked_point(self):
        """
        两族的光束都必须**穿过被点的那个空间点** (差别只在深度敏感度), 且自报几何量与姿态一致。

        这一条是"点了却打不中"的正面回归: beam_offset_mm 是目标点到光束线的垂距, 按构造必须是 0。
        """
        for u, v in ((CENTER_U, CENTER_V), (CENTER_U + 200.0, CENTER_V), (200.0, 700.0)):
            plan = self.service.plan(u, v)
            target = np.asarray(plan["target_point_mm"], dtype=np.float64)
            ray_dir = np.asarray(plan["ray_dir_base"], dtype=np.float64)
            self.assertTrue(plan["candidates"])
            for cand in plan["candidates"]:
                self.assertIn(cand["strategy"], ("on-ray", "swing", "re-aim"))
                tip = np.asarray(cand["tip_mm"], dtype=np.float64)
                axis = np.asarray(cand["axis_base"], dtype=np.float64)
                to_target = target - tip
                forward = float(np.dot(to_target, axis))
                self.assertGreater(forward, 0.0)                       # 目标点在枪口前方, 不是背后
                self.assertLess(float(np.linalg.norm(to_target - forward * axis)), 0.05)
                self.assertAlmostEqual(cand["beam_offset_mm"], 0.0, delta=0.05)
                self.assertEqual(cand["depth_locked"], cand["strategy"] in ("swing", "re-aim"))
                if cand["strategy"] == "on-ray":
                    np.testing.assert_allclose(axis, ray_dir, atol=1e-6)   # 共线族: 光束就是视线
                    along = float(np.dot(tip - np.asarray(plan["ray_origin_mm"], dtype=np.float64),
                                         ray_dir))
                    self.assertAlmostEqual(cand["along_ray_mm"], along, delta=0.05)
            # axis_base 是回显值 (五舍五入到 5 位小数), 与未取整的姿态轴只能同量级对齐
            np.testing.assert_allclose(_tool_axis(plan["chosen"]["pose_rad"]),
                                       np.asarray(plan["chosen"]["axis_base"], dtype=np.float64),
                                       atol=2e-5)

    def test_the_depth_label_does_not_move_the_arm(self):
        """
        视差角回归的本体: 深度标注只是界面上的一个数字。

        旧实现把"光束经过深度 600mm 处那个点"当约束, 换 depth 就换指向 -> 现场表现为
        "点越远偏得越多"。共线之后同一像素无论标 600 还是 1500mm, 指尖与姿态都不许变。
        """
        near = self.service.plan(CENTER_U, CENTER_V, distance_mm=600.0)
        far = self.service.plan(CENTER_U, CENTER_V, distance_mm=1500.0)
        self.assertEqual(near["chosen"]["tip_mm"], far["chosen"]["tip_mm"])
        self.assertEqual(near["chosen"]["pose_mm_deg"], far["chosen"]["pose_mm_deg"])
        np.testing.assert_allclose(far["target_point_mm"], [100.0, 0.0, 500.0 + 1500.0], atol=1e-6)

    def test_swing_family_keeps_the_tip_and_aims_at_the_clicked_point(self):
        """
        兜底族: 指尖原地不动 (视线离它 100mm), 但光束要**斜着穿过目标点**, 不是平行偏开。

        旧实现在这里放的是"与视线平行", 于是光束离被点的点整整差 100mm 且在任何深度都差着 ——
        真机上"点了好几次都还是偏"就是这么来的。
        """
        plan = self.service.plan(CENTER_U, CENTER_V)
        swings = [c for c in plan["candidates"] if c["strategy"] == "swing"]
        self.assertEqual(len(swings), 1)
        self.assertEqual(swings[0]["tip_mm"], TIP_NOW_MM)                           # 指尖零位移
        self.assertAlmostEqual(swings[0]["along_ray_mm"], -500.0, delta=0.05)       # 指尖在光心 z=500 身后
        self.assertTrue(swings[0]["depth_locked"])
        self.assertAlmostEqual(swings[0]["beam_offset_mm"], 0.0, delta=0.05)
        axis = np.asarray(swings[0]["axis_base"], dtype=np.float64)
        self.assertLess(float(np.dot(axis, np.asarray(plan["ray_dir_base"], dtype=np.float64))), 0.999)
        self.assertNotEqual(plan["chosen"]["strategy"], "swing")                    # 共线族优先

    def test_the_fallback_aims_at_the_measured_surface_depth(self):
        """
        兜底族只能瞄"某一个深度上的点", 所以那个深度必须**真测**: 用猜的 600mm 去打 1.8m 的墙,
        光束方向会拧偏好几度 (偏差随距离放大); 深度读不到时才退回配置靶距并如实标注来源。
        """
        self.cfg.robot_max_reach_mm = 900.0
        self._use_ray([0.0, 2000.0, 0.0], [1.0, 0.0, 0.0])     # 视线在可达球外 -> on-ray 整族落空
        wall = np.full((INTRINSICS["height"], INTRINSICS["width"]), 1800, dtype=np.uint16)
        with patch("apps.camera.services.camera_service.camera_service.get_depth_frame",
                   return_value=wall):
            plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual(plan["depth_source"], "measured")
        self.assertEqual(plan["display_depth_mm"], 1800.0)
        self.assertEqual(plan["chosen"]["strategy"], "swing")
        np.testing.assert_allclose(plan["target_point_mm"], [1800.0, 2000.0, 0.0], atol=1e-6)
        axis = np.asarray(plan["chosen"]["axis_base"], dtype=np.float64)
        np.testing.assert_allclose(axis, np.asarray([1800.0, 2000.0, 0.0]) /
                                   np.linalg.norm([1800.0, 2000.0, 0.0]), atol=1e-4)

        guessed = self.service.plan(CENTER_U, CENTER_V)        # 深度读不到 -> 退回配置靶距
        self.assertEqual(guessed["depth_source"], "config")
        self.assertEqual(guessed["display_depth_mm"], DEPTH_MM)
        guessed_axis = np.asarray(guessed["chosen"]["axis_base"], dtype=np.float64)
        skew_deg = math.degrees(math.acos(float(np.dot(axis, guessed_axis))))
        self.assertGreater(skew_deg, 5.0)                      # 猜错深度就是这么大方向误差

    def test_a_measured_depth_prefers_the_wrist_only_solution(self):
        """
        深度是实测的 -> 首选纯转腕 (二维云台语义): 指向只是方位问题, 与臂展无关, 指尖零位移。

        这一条钉住用户口径: 以前的实现在这里会去挪整条手臂把枪口摆到视线上, 于是"够不着"就变成
        了指向失败的原因 —— 而云台激光笔根本没有臂展这回事。
        """
        wall = np.full((INTRINSICS["height"], INTRINSICS["width"]), 1800, dtype=np.uint16)
        with patch("apps.camera.services.camera_service.camera_service.get_depth_frame",
                   return_value=wall):
            plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual(plan["depth_source"], "measured")
        self.assertEqual(plan["chosen"]["strategy"], "swing")
        self.assertEqual(plan["chosen"]["tip_mm"], TIP_NOW_MM)          # 指尖一动不动
        self.assertTrue(plan["chosen"]["depth_locked"])
        # 共线族仍然留在候选批里: 腕部被拒解时要能自动降级, 不能"够不着就失败"
        self.assertIn("on-ray", {c["strategy"] for c in plan["candidates"]})
        # 光束仍然穿过被点的那个空间点
        tip = np.asarray(plan["chosen"]["tip_mm"], dtype=np.float64)
        axis = np.asarray(plan["chosen"]["axis_base"], dtype=np.float64)
        to_t = np.asarray(plan["target_point_mm"], dtype=np.float64) - tip
        self.assertLess(float(np.linalg.norm(to_t - float(np.dot(to_t, axis)) * axis)), 0.05)

    def test_without_a_depth_reading_the_depth_independent_solution_wins(self):
        """
        深度读不到 -> 首选共线: 光束就是视线本身, 猜错深度也照样打中。

        这时若让纯转腕打头阵, 它只能瞄那个**猜测**深度上的点 (1.8m 的墙配 600mm 的猜测,
        落点偏 20cm 量级), 所以优先级必须翻回来。
        """
        plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual(plan["depth_source"], "config")
        self.assertEqual(plan["display_depth_mm"], DEPTH_MM)
        self.assertEqual(plan["chosen"]["strategy"], "on-ray")
        self.assertFalse(plan["chosen"]["depth_locked"])

    def test_an_already_aimed_wrist_is_not_twisted_for_nothing(self):
        """swing-only 的定义: 已经指着视线方向就不该再多拧一下腕 (现场会看到无意义的翻转)。"""
        plan = self.service.plan(CENTER_U, CENTER_V)     # 默认姿态单位阵, 工具 +Z 已 = 基座 +Z = 视线
        np.testing.assert_allclose(plan["chosen"]["pose_mm_deg"][3:], [0.0, 0.0, 0.0], atol=1e-6)

    def test_nozzle_behind_the_lens_is_excluded_by_the_self_collision_guard(self):
        """视线在球内的这一段几乎全在镜头背后 (枪口只能站在离镜头 3mm 处): 必须拒提并说清原因。"""
        self._use_ray([0.0, 0.0, 870.0], [0.0, 0.0, 1.0])    # 光心已在球边界上, 视线朝外
        self.cfg.robot_max_reach_mm = 900.0
        self.flange_pose = None                              # 测不出工具长度 -> 球按法兰尺寸
        plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertIsNone(plan["ray_window_mm"])
        self.assertTrue(any("leaves no room for a nozzle" in s for s in plan["skipped"]),
                        plan["skipped"])
        self.assertEqual({c["strategy"] for c in plan["candidates"]}, {"swing"})

    def test_missing_tcp_readout_still_plans_the_on_ray_family(self):
        """读不到当前 TCP 时只少了 swing 族与自旋参考, 不能让功能整体失效。"""
        self.robot.get_current_pose.return_value = (None, "Failed to read pose from driver")
        plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual({c["strategy"] for c in plan["candidates"]}, {"on-ray"})
        self.assertEqual(self.robot.check_reachability.call_count, 1)

    def test_the_wrist_reserve_covers_every_family_not_just_the_first_entries(self):
        """
        整批被拒后的腕部时钟重试必须**覆盖每一族的代表**, 不能只取列表头几个。

        真机现场 (14:34:21): 头 4 个候选全是 on-ray, 旧实现只重试头 3 个, 于是唯一那个与臂展
        无关的 swing 解从来没被重试过 —— 5 个候选 + 12 个后备全被拒, 整次点击什么都没发生,
        用户看到的就是"机械臂没有移动"。
        """
        batches: list = []

        def _gate(poses):
            batches.append(list(poses))
            if len(batches) == 1:
                return True, list(range(len(poses))), "refused all of them on purpose"
            return True, [], ""

        self.robot.check_reachability.side_effect = _gate
        plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual(len(batches), 2)                        # 确实走了腕部时钟重试
        swing = next(c for c in plan["candidates"] if c["strategy"] == "swing")
        want = np.asarray(swing["axis_base"], dtype=np.float64)
        # 绕光束轴自旋不改变光束方向, 所以重试批里只要有一条轴向等于 swing 的轴向就算覆盖到了
        axes = [_tool_axis(p.to_list()) for p in batches[1]]
        self.assertTrue(any(float(np.dot(a, want)) > 0.999 for a in axes),
                        f"the wrist-clock retry never covered the swing family: {axes}")

    def test_distance_outside_the_physical_guard_band_is_rejected(self):
        for bad in (10.0, 99999.0, float("nan")):
            with self.assertRaises(LiveAimError):
                self.service.plan(CENTER_U, CENTER_V, distance_mm=bad)

    def test_pixel_outside_the_image_is_rejected_before_any_motion_planning(self):
        with self.assertRaises(LiveAimError) as ctx:
            self.service.plan(1920.0, 100.0)
        self.assertIn("outside the image bounds", str(ctx.exception))

    def test_missing_intrinsics_says_the_camera_is_offline(self):
        with patch("apps.camera.services.camera_service.camera_service.get_intrinsics_dict",
                   return_value={}):
            with self.assertRaises(LiveAimError) as ctx:
                self.service.plan(CENTER_U, CENTER_V)
        self.assertIn("Camera intrinsics are unavailable", str(ctx.exception))

    # ---- IK 闸门与自旋后备 ----

    def test_controller_is_asked_once_in_the_happy_path(self):
        """候选一次性批量送控制器 (每点一次 IK 都要一次往返), 成功时不该有第二轮。"""
        plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual(self.robot.check_reachability.call_count, 1)
        sent = self.robot.check_reachability.call_args[0][0]
        self.assertEqual(len(sent), len(plan["candidates"]))
        self.assertEqual(plan["unreachable"], 0)

    def test_wrist_clock_reserve_runs_only_after_a_total_refusal(self):
        """
        整批被拒时才补一轮"绕光束轴拧腕": 指向不变, 只是把 J6 从限位/奇异上挪开。

        钉住两件事: 后备轮之前一定先问完第一轮 (否则现场白等一倍时间); 补出来的候选
        工具 +Z 仍然等于视线方向 (twist 只绕光束轴转, 不许改变指向)。
        """
        state = {"calls": 0}

        def _gate(poses):
            state["calls"] += 1
            if state["calls"] == 1:
                return (False, list(range(len(poses))), "near singular")
            return (True, [1], "near singular")      # 只拒第二轮第二个 (= tip0 的 twist -45)

        self.robot.check_reachability.side_effect = _gate
        plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual(state["calls"], 2)
        first = [c for c in plan["candidates"] if c["twist_deg"] == 0.0]
        reserve = [c for c in plan["candidates"] if c["twist_deg"] != 0.0]
        self.assertEqual(len(reserve),
                         live_aim_module._RESERVE_TIP_LIMIT * len(live_aim_module._TWIST_RESERVE_DEG))
        self.assertEqual(plan["unreachable"], len(first) + 1)      # 两轮被拒的总数
        self.assertAlmostEqual(plan["chosen"]["twist_deg"], live_aim_module._TWIST_RESERVE_DEG[0], places=6)
        np.testing.assert_allclose(_tool_axis(plan["chosen"]["pose_rad"]),
                                   plan["ray_dir_base"], atol=1e-6)

    def test_every_pose_refused_says_the_sight_line_is_fine_but_the_arm_is_too_far(self):
        self.robot.check_reachability.side_effect = lambda poses: (
            False, list(range(len(poses))), "controller rejected all")
        with self.assertRaises(LiveAimError) as ctx:
            self.service.plan(CENTER_U, CENTER_V)
        message = str(ctx.exception)
        self.assertIn("refused every pose whose tool axis follows this sight line", message)
        self.assertIn("controller rejected all", message)      # 控制器原文必须传出来
        self.assertIn("jog the arm toward", message)
        self.assertEqual(self.robot.check_reachability.call_count, 2)   # 第一轮 + 自旋后备

    # ---- 双装法取数 ----

    def test_eye_to_hand_keeps_the_extrinsic_constant(self):
        """眼在手外: 外参与臂在哪无关 (但仍会读关节做 FK, 那是为了实测工具长度)。"""
        self.service.plan(CENTER_U, CENTER_V)
        self.cfg.camera_extrinsic_at.assert_called_once_with(None)

    def test_eye_in_hand_composes_the_extrinsic_from_the_current_flange_pose(self):
        self.cfg.hand_eye_mount = EYE_IN_HAND
        self.cfg.camera_extrinsic_at.return_value = (T_BASE_CAMERA, EYE_IN_HAND, None)
        flange = [367.6, 22.2, 246.2, 119.2, -0.9, 72.0]      # mm + deg
        self.flange_pose = flange
        plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual(self.robot.check_reachability.call_count, 1)   # 只一轮 IK, 法兰只用来复合与测工具长
        self.cfg.camera_extrinsic_at.assert_called_once_with(flange)
        self.assertEqual(plan["mount"], EYE_IN_HAND)

    def test_eye_in_hand_without_forward_kinematics_is_blocked_not_silently_misframed(self):
        self.cfg.hand_eye_mount = EYE_IN_HAND
        self.flange_pose = None
        with self.assertRaises(LiveAimError) as ctx:
            self.service.plan(CENTER_U, CENTER_V)
        self.assertIn("Forward kinematics", str(ctx.exception))

    def test_unusable_extrinsic_surfaces_the_kernel_reason(self):
        """口径不同源 / 缺 T_flange_camera 这类硬拦, 英文原因必须原样传到用户眼前。"""
        self.cfg.camera_extrinsic_at.return_value = (None, EYE_IN_HAND, "pose frame mismatch: joint_offset_v2")
        with self.assertRaises(LiveAimError) as ctx:
            self.service.plan(CENTER_U, CENTER_V)
        self.assertIn("pose frame mismatch", str(ctx.exception))


class LiveAimReachBallTest(_Patched):
    """
    可达球半径 = 法兰臂展 + 实测工具长度, 以及视线在球外时的 parallel 兜底。

    场景直接取真机 Home 那一刻 (日志 11:41:04): 光心 (540.0, 184.8, 623.6) mm, 视线
    (0.9809, 0.1899, 0.0418), 30004 的 TCP (666.2, 181.0, 534.4) mm。CR5 臂展 900mm +
    实测工具 153mm -> 球 1021.4mm, 视线在球内可站到光心前 232.4mm (靶距 150mm 放得下);
    少算那一截工具时球只有 873.0mm, 可用段被砍到 38.9mm, 连自撞护栏 (50mm) 都容不下
    -> on-ray 族整族被误杀, 只能退回兜底族。真机"移动一次后第二次点选够不着、回 Home 又可以"
    就是这段误杀造成的, 目测的"偏 10cm"就是兜底族那段垂距。

    兜底族本身也换过口径: 旧实现是"与视线平行"(整条光束偏开 98.3mm, 在任何深度都打不中,
    这就是"点了好几次都还是偏"), 现在是 swing —— 指尖不动、光束斜着穿过被点那个点。
    """

    reach_mm = 900.0
    HOME_ORIGIN_MM = [540.0, 184.8, 623.6]
    HOME_DIR = [0.9809, 0.1899, 0.0418]
    HOME_TCP_MM = [666.2, 181.0, 534.4]

    def setUp(self):
        super().setUp()
        self._use_ray(self.HOME_ORIGIN_MM, self.HOME_DIR)
        self._use_tcp(self.HOME_TCP_MM)

    def _ball_radius(self) -> float:
        return self.service.plan(CENTER_U, CENTER_V)["tcp_reach_ball_mm"]

    def test_measured_tool_length_widens_the_reach_ball(self):
        """工具长度 = |TCP 读数 - FK 法兰| = 153mm, 必须叠加进球半径 (不读配置、跟着 tool 号走)。"""
        margin = live_aim_module._REACH_MARGIN_RATIO
        self.assertAlmostEqual(self._ball_radius(), (self.reach_mm + 153.0) * margin, delta=0.2)
        self.flange_pose = None                     # 测不出工具长度 -> 保守退回法兰尺寸
        self.assertAlmostEqual(self._ball_radius(), self.reach_mm * margin, delta=0.2)

    def test_the_sight_line_window_follows_the_ball_and_the_lens_guard(self):
        plan = self.service.plan(CENTER_U, CENTER_V)
        lo, hi = plan["ray_window_mm"]
        self.assertAlmostEqual(lo, live_aim_module._MIN_TIP_AHEAD_MM, places=3)
        # 窗口上界就是球边界: 射线上的点离基座不超过 tcp_reach_ball_mm
        self.assertLessEqual(np.linalg.norm(np.asarray(self.HOME_ORIGIN_MM) + hi * np.asarray(self.HOME_DIR)),
                             plan["tcp_reach_ball_mm"] + 1.0)
        self.assertGreater(hi, STANDOFF_MM)                    # 够装下"离镜头一个靶距"的站位
        for cand in plan["candidates"]:
            if cand["strategy"] == "on-ray":
                self.assertGreaterEqual(cand["base_distance_mm"], 0.0)
                self.assertLessEqual(cand["base_distance_mm"], plan["tcp_reach_ball_mm"] + 1.0)

    def test_on_ray_survives_at_home_thanks_to_the_tool_allowance(self):
        """
        真机现场回归: 同一视线、同一臂展, 只有"算不算工具长度"的差别。

        旧口径 (900mm 法兰球) 把可用段砍到 38.9mm, 连 150mm 靶距都放不下 -> on-ray 被误判不可达,
        只能退回带 98.3mm 视线垂距的兜底族, 现场表现为"点越远越偏"。
        """
        with_tool = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual(with_tool["chosen"]["strategy"], "on-ray")
        self.assertAlmostEqual(with_tool["chosen"]["along_ray_mm"], STANDOFF_MM, delta=0.05)
        self.assertAlmostEqual(with_tool["chosen"]["beam_offset_mm"], 0.0, places=1)
        self.flange_pose = None
        without = self.service.plan(CENTER_U, CENTER_V)
        self.assertIsNone(without["ray_window_mm"])                   # 球内已容不下靶距站位
        self.assertEqual(without["chosen"]["strategy"], "swing")
        np.testing.assert_allclose(without["chosen"]["tip_mm"], self.HOME_TCP_MM, atol=0.05)
        # 兜底族照样必须打中: 光束从当前枪口斜穿目标点, 垂距归零 (旧口径这里恒差 98.3mm)
        self.assertAlmostEqual(without["chosen"]["beam_offset_mm"], 0.0, delta=0.05)
        self.assertTrue(without["chosen"]["depth_locked"])
        self.assertTrue(any("leaves no room for a nozzle" in s for s in without["skipped"]),
                        without["skipped"])

    def test_sight_line_outside_the_ball_falls_back_to_swinging_onto_the_point(self):
        """
        视线完全不进可达球 (离基座 2000mm): 不许报"够不着"完事, 也不许只把光束拧到与视线平行
        (那样整条光束偏开 2022mm, 任何深度都打不中), 而要原地斜着瞄向被点的那个点。
        """
        self._use_ray([0.0, 2000.0, 0.0], [1.0, 0.0, 0.0])
        self._use_tcp([400.0, 0.0, 300.0])
        plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertIsNone(plan["ray_window_mm"])
        self.assertTrue(any("stays 2000 mm from the robot base" in s for s in plan["skipped"]),
                        plan["skipped"])
        chosen = plan["chosen"]
        self.assertEqual(chosen["strategy"], "swing")
        np.testing.assert_allclose(chosen["tip_mm"], [400.0, 0.0, 300.0], atol=0.05)   # 指尖零位移
        self.assertAlmostEqual(chosen["beam_offset_mm"], 0.0, delta=0.05)               # 但光束打中
        # 目标点 = 光心 + 配置靶距 600mm 沿 +X; 光束必须从枪口直指它
        want = np.asarray(plan["target_point_mm"], dtype=np.float64) - np.asarray([400.0, 0.0, 300.0])
        # axis_base 回显时五舍五入到 5 位小数, 目标点回显到 0.01mm: 只能同量级对齐
        np.testing.assert_allclose(_tool_axis(chosen["pose_rad"]), want / np.linalg.norm(want), atol=1e-4)

    def test_reach_radius_comes_from_config(self):
        """换臂型只改配置: 臂展变大, 视线窗口必须变长 (不能把 900 写死在代码里)。"""
        _, hi_900 = self.service.plan(CENTER_U, CENTER_V)["ray_window_mm"]
        self.cfg.robot_max_reach_mm = 1200.0
        _, hi_1200 = self.service.plan(CENTER_U, CENTER_V)["ray_window_mm"]
        self.assertGreater(hi_1200, hi_900)

    def test_controller_still_has_the_final_say_inside_the_ball(self):
        """球内不等于可达 (关节限位/奇异): 控制器拒完所有候选仍要报错, 不能默默放行。"""
        self.robot.check_reachability.side_effect = lambda poses: (
            False, list(range(len(poses))), "near singular")
        with self.assertRaises(LiveAimError) as ctx:
            self.service.plan(CENTER_U, CENTER_V)
        self.assertIn("near singular", str(ctx.exception))


class LiveAimImagePairingTest(_Patched):
    """
    眼在手上的硬前提: 点击的那一帧画面必须对应**此刻**的法兰位姿。

    真机现场吃过这个亏 (日志 12:36:25-31 连续 waitForFrameset timeout / 软重启 / Capture FPS 0.26,
    而 12:36:32 的指向已按当前法兰复合了视线)。两条守护各自独立, 且都是“拒指”而不是“默默指歪”;
    眼在手外时相机不动, 视线与臂姿态无关, 不能因为这个闸门把它拦掉 (否则白白锁死一个可用功能)。
    """

    def _use_eih(self) -> None:
        self.cfg.hand_eye_mount = EYE_IN_HAND
        self.cfg.camera_extrinsic_at.return_value = (T_BASE_CAMERA, EYE_IN_HAND, None)

    def test_eye_in_hand_refuses_a_click_while_the_image_has_not_caught_up(self):
        self._use_eih()
        self.robot.seconds_since_motion.return_value = 0.3     # 臂 0.3s 前才停, 显示延迟还没覆盖
        with self.assertRaises(LiveAimError) as ctx:
            self.service.plan(CENTER_U, CENTER_V)
        message = str(ctx.exception)
        self.assertIn("stopped only 0.3 s ago", message)
        self.assertIn("Wait until the image settles", message)     # 告诉用户下一步做什么
        self.robot.check_reachability.assert_not_called()           # 拒在几何之前

    def test_eye_in_hand_refuses_a_stalled_or_restarting_stream(self):
        """相机断流时画面停在某个未知时刻: 宁可拒指, 也不能拿旧画面算新视线。"""
        self._use_eih()
        with patch("apps.camera.services.camera_service.camera_service.get_status",
                   return_value={"online": True, "streaming": True, "color_fps": 0.26}):
            with self.assertRaises(LiveAimError) as ctx:
                self.service.plan(CENTER_U, CENTER_V)
        self.assertIn("stalled or restarting", str(ctx.exception))

    def test_eye_to_hand_is_exempt_because_the_camera_does_not_move(self):
        """外参是常量, 画面新旧与射线无关: 同样的异常现场不能拦住眼在手外的指向。"""
        self.robot.seconds_since_motion.return_value = 0.0
        with patch("apps.camera.services.camera_service.camera_service.get_status",
                   return_value={"online": False, "streaming": False, "color_fps": 0.0}):
            plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual(plan["chosen"]["strategy"], "on-ray")

    def test_the_settle_floor_can_be_turned_off_by_config(self):
        """显示链路很短的现场可以关掉静止时长这一项 (取 0), 但断流守护仍然生效。"""
        self._use_eih()
        self.cfg.aim_frame_settle_s = 0.0
        self.robot.seconds_since_motion.return_value = 0.05
        self.assertEqual(self.service.plan(CENTER_U, CENTER_V)["mount"], EYE_IN_HAND)


class LiveAimMotionTest(_Patched):
    """动作阶段: 互锁、工具号先于规划、故障安全关喷、指向速度、到位亮激光与读回光斑偏差。"""

    def setUp(self):
        super().setUp()
        self.order = []
        for method in ("set_tool", "set_do", "move_to_pose_j"):
            self._stub_hardware_step(method, (True, ""))

    def _stub_hardware_step(self, method: str, ret: tuple) -> None:
        """
        把某个硬件动作接成"留痕 + 返回指定结果"。

        顺序断言 (工具号 -> 关喷 -> 动 -> 读回 -> 亮激光) 靠这份留痕, 所以失败分支也必须走同一个
        桩 —— 改用 return_value 会丢留痕, 就看不出到底跑到哪一步停了。
        """
        token = {"set_tool": "set_tool", "set_do": "set_do", "move_to_pose_j": "move"}[method]

        def _step(*_args, **_kwargs):
            self.order.append(token)
            return ret

        getattr(self.robot, method).side_effect = _step

    def test_a_disconnected_robot_is_refused_before_any_geometry_work(self):
        self.robot.is_connected.return_value = False
        with self.assertRaises(LiveAimError) as ctx:
            self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertIn("not connected", str(ctx.exception))
        self.robot.check_reachability.assert_not_called()

    def test_moving_arm_locks_the_action(self):
        self.robot.is_moving.return_value = True
        with self.assertRaises(LiveAimError) as ctx:
            self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertIn("already moving", str(ctx.exception))
        self.robot.move_to_pose_j.assert_not_called()

    def test_interlock_is_rechecked_after_planning_and_before_motion(self):
        """规划期 (含 IK 往返) 里臂可能已经被别的动作开走: 下发前必须以控制器反馈再判一次。"""
        self.robot.is_moving.side_effect = [False, True]
        with self.assertRaises(LiveAimError) as ctx:
            self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertIn("already moving", str(ctx.exception))
        self.robot.move_to_pose_j.assert_not_called()

    def test_tool_is_active_before_planning_and_spraying_is_cut_with_an_immediate_do(self):
        """
        顺序口径: 工具号必须**先于规划**生效 —— plan() 要读当前 TCP 位置、实测工具长度并让控制器
        按同一个 tool 逆解, 坐标系不同源的话"原地摆头"就不是原地, IK 也不是执行期那个 IK。
        """
        self.robot.get_current_pose.side_effect = lambda: ((self.order.append("read_tcp"),
                                                             TIP_NOW_MM + [0.0, 0.0, 0.0])[1], "")
        self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertEqual(self.order, ["set_tool", "read_tcp", "set_do", "move", "read_tcp", "set_do"])
        self.robot.set_tool.assert_called_once_with(1)
        first_do = self.robot.set_do.call_args_list[0]
        self.assertEqual(list(first_do[0]), [3, 0])           # 喷涂 DO 编号, 置 0
        self.assertTrue(first_do[1]["immediate"])             # 立即指令 (DOExecute), 不走队列

    def test_move_is_aborted_when_the_spray_do_cannot_be_turned_off(self):
        """fail-close: 断不了料就绝不动 —— 机械臂带着出料去运动比停在原地危险。"""
        self._stub_hardware_step("set_do", (False, "DO write failed"))
        with self.assertRaises(LiveAimError) as ctx:
            self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertIn("could not be turned off", str(ctx.exception))
        self.robot.move_to_pose_j.assert_not_called()
        self.assertEqual(self.order, ["set_tool", "set_do"])

    def test_move_failure_is_reported_in_english_and_leaves_the_do_off(self):
        self._stub_hardware_step("move_to_pose_j", (False, "move_j returned error code: 32"))
        with self.assertRaises(LiveAimError) as ctx:
            self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertIn("Robot refused the aiming move", str(ctx.exception))
        self.assertEqual(self.order, ["set_tool", "set_do", "move"])     # 没走到亮激光

    def test_the_laser_is_switched_on_only_after_a_successful_arrival(self):
        """
        到位亮激光 (spraying.aim_do_on_arrive): 排在运动成功且读回反馈之后。

        这是整条链路上唯一的开阀点 —— 放在最后是因为它之前的任何异常都必须天然停在"已关喷"
        的安全态; 关掉这一项 (真喷枪) 时, 动作后 DO 必须仍然只被写过 0。
        """
        report = self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertEqual(self.order, ["set_tool", "set_do", "move", "set_do"])
        ons = [c for c in self.robot.set_do.call_args_list if list(c[0])[1] == 1]
        self.assertEqual(len(ons), 1)
        self.assertEqual(list(ons[0][0]), [3, 1])
        self.assertTrue(ons[0][1]["immediate"])
        self.assertEqual(report["spray_do_state"], "on")
        self.assertEqual(report["spray_do_index"], 3)

    def test_auto_do_can_be_disabled_for_a_real_spray_gun(self):
        self.cfg.aim_do_on_arrive = False
        report = self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertEqual(self.order, ["set_tool", "set_do", "move"])
        self.assertTrue(report["spray_do_state"].startswith("off"))

    def test_a_failed_switch_on_is_reported_but_does_not_fail_the_action(self):
        """臂已经到位了, 激光没亮起来是显示问题不是安全问题: 降级为 on-failed 并继续返回结果。"""
        calls = {"n": 0}

        def _do(*_args, **_kwargs):
            calls["n"] += 1
            return (True, "") if calls["n"] == 1 else (False, "DO write failed")

        self.robot.set_do.side_effect = _do
        report = self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertEqual(report["status"], "moved")
        self.assertTrue(report["spray_do_state"].startswith("on-failed"))

    def test_speed_uses_the_aim_percent_of_the_joint_limit(self):
        """
        指向是单点空走, 不吃面板上为喷涂走线调的慢速: 配置写 50 就是控制器的 SpeedJ=50%。

        move_to_pose_j 收到的是 deg/s, 服务层再按同一个上限折回百分比, 所以这里断言的
        45.0 == 50% x 90 deg/s 就是"配置百分比"这一口径没有被算歪的证据。
        """
        self.cfg.aim_speed_percent = 25.0
        self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertAlmostEqual(self.robot.move_to_pose_j.call_args[1]["speed"], 45.0, places=6)
        self.assertEqual(self.robot.move_to_pose_j.call_args[1]["acc"], 25.0)     # acc 仍读面板 acc_j

    def test_explicit_speed_and_a_single_axis_limit_are_honoured(self):
        self.service.aim_at_pixel(CENTER_U, CENTER_V, speed=12.0, acc=8.0)
        self.assertEqual(self.robot.move_to_pose_j.call_args[1]["speed"], 12.0)
        self.assertEqual(self.robot.move_to_pose_j.call_args[1]["acc"], 8.0)
        with self.assertRaises(LiveAimError):
            self.service.aim_at_pixel(CENTER_U, CENTER_V, speed=0.0)
        self.cfg.aim_speed_percent = 900.0          # 越界配置被钉回 100%
        self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertAlmostEqual(self.robot.move_to_pose_j.call_args[1]["speed"], 180.0, places=6)

    def test_pose_is_delivered_in_the_driver_convention_of_millimetres_and_radians(self):
        self.service.aim_at_pixel(CENTER_U, CENTER_V)
        pose = self.robot.move_to_pose_j.call_args[0][0]
        self.assertEqual(len(pose), 6)
        # 中心像素 -> 视线 (100,0,500)+t*+Z 上靶距处, 姿态角在 rad 值域内
        np.testing.assert_allclose(pose[:3], [100.0, 0.0, 650.0], atol=0.05)
        self.assertTrue(all(abs(v) <= 2.0 * math.pi for v in pose[3:]))

    def test_readback_measures_the_spot_against_the_clicked_point(self):
        """按规划姿态回填反馈 -> 光斑正好落在被点的点上: 垂距 0 / 角误差 0。"""
        plan = self.service.plan(CENTER_U, CENTER_V)
        self.robot.get_current_pose.return_value = (plan["chosen"]["pose_rad"], "")
        report = self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertEqual(report["status"], "moved")
        self.assertAlmostEqual(report["spot_error_mm"], 0.0, places=1)
        self.assertAlmostEqual(report["beam_angle_deg"], 0.0, places=1)
        self.assertAlmostEqual(report["nozzle_in_front_of_lens_mm"], STANDOFF_MM, delta=0.2)
        self.assertEqual(report["target_point_mm"], plan["target_point_mm"])
        self.assertEqual(report["strategy"], "on-ray")

    def test_a_beam_that_misses_shows_up_as_an_angular_and_a_spot_error(self):
        """
        反馈里工具轴偏了 1 度 (指尖不动): 角误差必须如实是 1 度, 光斑垂距随距离放大。

        这一条同时钉住"误差用视线/目标点量而不是用规划值自证"的口径 —— 旧实现拿规划目标点
        当约束再拿它当指标, 指歪了也会报 miss 0。
        """
        plan = self.service.plan(CENTER_U, CENTER_V)
        drifted = list(plan["chosen"]["pose_rad"])
        tilt = Rot.from_rotvec([0.0, math.radians(1.0), 0.0]).as_matrix()
        drifted[3:] = [float(x) for x in Rot.from_matrix(
            tilt @ Rot.from_euler("xyz", drifted[3:]).as_matrix()).as_euler("xyz")]
        self.robot.get_current_pose.return_value = (drifted, "")
        report = self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertAlmostEqual(report["beam_angle_deg"], 1.0, delta=0.05)
        # 光斑离被点 (在枪口前 450mm) 约 450*tan(1deg) = 7.9mm
        self.assertAlmostEqual(report["spot_error_mm"], 7.9, delta=0.6)

    def test_missing_readback_does_not_fail_the_action_but_is_reported(self):
        self.robot.get_current_pose.return_value = (None, "Failed to read pose from driver")
        report = self.service.aim_at_pixel(CENTER_U, CENTER_V)
        self.assertEqual(report["status"], "moved")
        self.assertIsNone(report["beam_angle_deg"])
        self.assertIn("read pose", report["readback_error"])




class LiveAimMarkCrossTest(_Patched):
    """
    人工点选十字中心做真值校核: 把被点目标点与人工点下的十字中心各自用**那一刻的相机位姿**
    反投影回基座系再比 —— 完全绕开脆弱的差分检测, 由人眼定中心。这里用固定内参 + 眼在手外
    (常量外参, 两次点击相机同一点) 让期望值可手算。

    钉死三件事:
    1. 完美情形 (点回同一个像素、TCP 正落在视线上) 三个真值全为 0 —— 不编误差;
    2. 侧向偏移: 把十字往 u 方向点偏 -> 视线夹角、侧向脉靶、光束轴偏差三者同步非零且可算;
    3. 没先指向 / 没深度 时诚实降级 (报错或只报角度), 绝不拿 None 拼一个假 mm。
    """

    def _snapshot(self, tip_on_ray):
        """把一次"指向已到位"的真值参照直接摄进服务 (避开跑整条 aim): 目标 = 中心像素 @ 600mm。"""
        self._use_tcp(tip_on_ray)
        self.service._last_aim = {
            "pixel": [CENTER_U, CENTER_V],
            "ray_origin_mm": list(RAY_ORIGIN_MM),      # (100,0,500)
            "ray_dir_base": list(RAY_DIR),             # (0,0,1)
            "target_point_mm": list(TARGET_MM),        # (100,0,1100)
            "display_depth_mm": DEPTH_MM,
            "depth_source": "measured",
            "mount": EYE_TO_HAND,
            "strategy": "on-ray",
            "planned_beam_axis_base": list(RAY_DIR),
            "arrived_pose_mm_deg": None,
            "beam_on": True,
            "ts": 0.0,
        }

    @staticmethod
    def _depth_at(value_mm):
        return np.full((INTRINSICS["height"], INTRINSICS["width"]), int(value_mm), dtype=np.uint16)

    def test_marking_the_same_pixel_with_nozzle_on_ray_reads_zero_everywhere(self):
        """点回目标同一像素、且 TCP 正站在视线上: 脉靶、视线夹角、光束轴偏差必须全为 0。"""
        self._snapshot(tip_on_ray=[100.0, 0.0, 500.0])   # 相机光心处 (视线上的点), 轴 +Z
        with patch("apps.camera.services.camera_service.camera_service.get_depth_frame",
                   return_value=self._depth_at(600)):
            r = self.service.mark_cross_center(CENTER_U, CENTER_V)
        self.assertEqual(r["status"], "verified")
        self.assertAlmostEqual(r["sight_angle_deg"], 0.0, places=2)
        self.assertAlmostEqual(r["miss_vs_target_mm"], 0.0, places=1)
        self.assertAlmostEqual(r["beam_axis_gap_mm"], 0.0, places=1)
        self.assertAlmostEqual(r["beam_axis_angle_deg"], 0.0, places=2)

    def test_marking_the_cross_off_centre_reports_the_true_lateral_miss(self):
        """把十字中点往 +u 偏 60px: 侧向脉靶≈60mm@光心那层折算, 三个量互相印证且可手算。"""
        self._snapshot(tip_on_ray=[100.0, 0.0, 500.0])
        du = 60.0
        with patch("apps.camera.services.camera_service.camera_service.get_depth_frame",
                   return_value=self._depth_at(600)):
            r = self.service.mark_cross_center(CENTER_U + du, CENTER_V)
        # x/z = du/fx; 视线偏差角 = atan(du/fx); 落点侧向 = 600 * du/fx (同深度), 光束轴同值
        du_over_fx = du / 611.68
        expect_angle = round(math.degrees(math.atan(du_over_fx)), 2)
        self.assertAlmostEqual(r["sight_angle_deg"], expect_angle, places=1)
        # 沿目标视线 (纯 +Z) 的深度分量应接近 0 (同一深度), 侧向分量占全部
        self.assertLess(abs(r["miss_depth_mm"]), 1.0)
        self.assertAlmostEqual(r["miss_lateral_mm"], 600.0 * du_over_fx, delta=1.5)
        self.assertAlmostEqual(r["beam_axis_gap_mm"], 600.0 * du_over_fx, delta=1.5)
        self.assertAlmostEqual(r["beam_axis_angle_deg"], expect_angle, places=1)

    def test_marking_without_depth_only_gives_the_angle(self):
        """深度读不到时: 只能给两条视线夹角, 三维量全 None 并明说, 绝不编一个 mm。"""
        self._snapshot(tip_on_ray=[100.0, 0.0, 500.0])
        with patch("apps.camera.services.camera_service.camera_service.get_depth_frame",
                   return_value=None):
            r = self.service.mark_cross_center(CENTER_U + 60.0, CENTER_V)
        self.assertIsNotNone(r["sight_angle_deg"])
        self.assertIsNone(r["cross_point_mm"])
        self.assertIsNone(r["miss_vs_target_mm"])
        self.assertIn("no usable depth", r["note"])

    def test_marking_before_any_aim_is_refused(self):
        """还没指向过就点十字: 无从参照, 必须明确报错而不是算出一个无意义的数。"""
        self.service._last_aim = None
        with self.assertRaises(LiveAimError) as ctx:
            self.service.mark_cross_center(CENTER_U, CENTER_V)
        self.assertIn("Nothing to verify yet", str(ctx.exception))

    def test_ui_notice_is_mirrored_into_the_verify_log(self):
        """界面弹出的每条通知必须落进专用校核日志 (用户不必复制粘贴)。"""
        with patch.object(self.service._verify_log, "log") as log:
            self.service.log_ui_notice("ok", "cross center at pixel (643, 405), 0.4 deg off")
        log.assert_called_once()
        self.assertIn("cross center at pixel", log.call_args.args[1])

    def test_empty_notice_is_not_logged(self):
        """空通知不写日志 (避免前端刷新打出满屏空行)。"""
        with patch.object(self.service._verify_log, "log") as log:
            self.service.log_ui_notice("info", "   ")
        log.assert_not_called()


class LiveAimReAimFamilyTest(_Patched):
    """
    re-aim 族: 把当前 TCP **绕基座原点**重定位来补腕部锥。

    旋转轴都过基座原点 => |TCP| 不变 => 指尖仍落在同一可达球壳上 (绝不会被拖去臂展边缘),
    但换了指尖方位; 在新位置重新瞄向目标点, 接住 swing (原地转腕) 单一锥面拧不过去的方向。
    """

    def setUp(self):
        super().setUp()
        # 必须用非原点 TCP: TIP_NOW 若在基座原点, 绕原点旋转仍是原点 -> re-aim 与 swing 去重归零。
        self._use_tcp([300.0, 0.0, 600.0])

    def test_radius_is_preserved_by_base_rotation(self):
        """纯几何: 绕基座 +Z/+Y 旋转保持 |v| 不变 (这正是 re-aim 不被拖去臂展边缘的根本), 零角=恒等。"""
        tip = np.array([300.0, 0.0, 600.0])
        r0 = float(np.linalg.norm(tip))
        for az in (0.0, 30.0, -30.0, 60.0, -60.0):
            for pi in (0.0, 30.0, -30.0):
                v = live_aim_module._rotate_about_base(tip, az, pi)
                self.assertAlmostEqual(float(np.linalg.norm(v)), r0, places=6)
        np.testing.assert_allclose(live_aim_module._rotate_about_base(tip, 0.0, 0.0), tip, atol=1e-9)

    def test_reaim_family_appears_and_keeps_beam_through_the_point(self):
        """TCP 离开原点后 re-aim 族必须出现; 每个候选半径与 swing 一致 (同可达球壳), 且光束穿过被点。"""
        plan = self.service.plan(CENTER_U, CENTER_V)
        reaims = [c for c in plan["candidates"] if c["strategy"] == "re-aim"]
        self.assertTrue(reaims, "expected re-aim candidates once the tip is off the base origin")
        target = np.asarray(plan["target_point_mm"], dtype=np.float64)
        swing = next(c for c in plan["candidates"] if c["strategy"] == "swing")
        r_swing = float(np.linalg.norm(swing["tip_mm"]))
        for c in reaims:
            self.assertTrue(c["depth_locked"])                              # 与 swing 同为深度锁定族
            self.assertAlmostEqual(c["base_distance_mm"], r_swing, delta=0.5)  # 半径不变
            tip = np.asarray(c["tip_mm"], dtype=np.float64)
            axis = np.asarray(c["axis_base"], dtype=np.float64)
            to_t = target - tip
            fwd = float(np.dot(to_t, axis))
            self.assertGreater(fwd, 0.0)                                    # 目标在枪口前方
            self.assertLess(float(np.linalg.norm(to_t - fwd * axis)), 0.05)  # 光束穿过被点
            self.assertAlmostEqual(c["beam_offset_mm"], 0.0, delta=0.05)

    def test_reaim_rescues_a_click_that_swing_cannot_reach(self):
        """实测深度下排序 swing 在前; 控制器拒了 swing 却放行 re-aim -> 必须自动降级到 re-aim。"""
        wall = np.full((INTRINSICS["height"], INTRINSICS["width"]), 1800, dtype=np.uint16)
        with patch("apps.camera.services.camera_service.camera_service.get_depth_frame",
                   return_value=wall):
            # sort 后 index0=swing; 只拒它, 其余放行 -> chosen 落到第一个 re-aim
            self.robot.check_reachability.side_effect = lambda _poses: (False, [0], "swing pose refused")
            plan = self.service.plan(CENTER_U, CENTER_V)
        self.assertEqual(plan["depth_source"], "measured")
        self.assertEqual(plan["chosen"]["strategy"], "re-aim")


class LiveAimLaserTiltTest(unittest.TestCase):
    """
    激光光轴相对工具 +Z 的安装角偏补偿: 只钉几何不变量 (不跑整条 aim)。

    补偿的正确定义是: 规划把一个"令工具+Z==视线"的姿态 R0 改为 R0@R_undo, 于是**真实光轴**
    (R_cmd@b_tool) 而非 +Z 落在视线上; 且 R_undo 必须把 b_tool 旋回 +Z (否则残差不会收敛到 0)。
    """

    def _frames(self, tilt):
        live_aim_module._LASER_TILT_CACHE.clear()
        with patch.object(live_aim_module, "sprayer_config") as cfg:
            cfg.laser_tilt_tool_deg = list(tilt)
            return live_aim_module._laser_tilt_frames()

    def test_undo_rotation_maps_beam_back_to_z(self):
        """任意角偏: b_tool 必须单位化, R_undo@b_tool 必须精确==+Z (残差口径自洽的前提)。"""
        e3 = np.array([0.0, 0.0, 1.0])
        for tilt in ([0.0, 0.0], [-2.55, 1.20], [4.0, -3.0], [0.0, 2.5], [-7.0, -7.0]):
            b, r_undo = self._frames(tilt)
            self.assertAlmostEqual(float(np.linalg.norm(b)), 1.0, places=9)
            np.testing.assert_allclose(r_undo @ b, e3, atol=1e-9)
            np.testing.assert_allclose(r_undo @ r_undo.T, np.eye(3), atol=1e-9)  # 正当旋转

    def test_zero_tilt_degenerates_to_identity(self):
        """补偿关闭 ([0,0]) 时 b_tool==+Z 且 R_undo==单位阵, 完全等价旧行为。"""
        b, r_undo = self._frames([0.0, 0.0])
        np.testing.assert_allclose(b, [0.0, 0.0, 1.0], atol=1e-12)
        np.testing.assert_allclose(r_undo, np.eye(3), atol=1e-12)

    def test_commanded_pose_lands_real_beam_on_sight_axis(self):
        """指令姿态 R=R0@R_undo 后, 真实光轴 R@b_tool 必须仍精确==视线 axis (对任意姿态)。"""
        b, r_undo = self._frames([-2.55, 1.20])
        e3 = np.array([0.0, 0.0, 1.0])
        for axis in ([1, 0, 0], [0, 1, 0], [0.3, -0.6, 0.75], [0, 0, 1], [-0.5, 0.5, -0.7]):
            a = np.asarray(axis, dtype=np.float64)
            a = a / np.linalg.norm(a)
            # R0: 把 +Z 对齐到视线 a (等价于 tool_frame_from_z_axis 的不变量 R0@e3=a)
            ax = np.cross(e3, a)
            if float(np.linalg.norm(ax)) < 1e-12:
                R0 = np.eye(3) if np.dot(e3, a) > 0 else live_aim_module._rodrigues(np.array([1.0, 0, 0]), math.pi)
            else:
                R0 = live_aim_module._rodrigues(ax, math.acos(float(np.clip(np.dot(e3, a), -1.0, 1.0))))
            real_beam = (R0 @ r_undo) @ b
            np.testing.assert_allclose(real_beam, R0 @ e3, atol=1e-9)   # == axis


if __name__ == "__main__":
    unittest.main(verbosity=2)
