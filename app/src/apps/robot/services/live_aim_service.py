# -*- coding: utf-8 -*-
"""
实时视频 "光束指向" (live aim) 服务。

在实时视频上点一个像素, 机械臂就把**工具轴线当成一支激光笔**, 让这条光束穿过被点的那个
空间点 —— 像二维云台那样只管调指向, 指尖站在哪儿、腕部怎么绕过去都不设额外约束。

首选族是**共线** (on-ray): 光束方向就等于视线方向, 指尖沿这条线站在哪儿都行 ——

    R·e_z = û   且   指尖 T 落在射线 O + t·û 上 (t 取可达、在镜头前、且不越过目标点的那一段)

为什么优先共线: 单个像素反投影只能给出一条**射线**, 深度未知。若只要求光束经过"按猜测深度取的
那一点 P", 光束方向就是 normalize(P - T), 与视线差一个**视差角**: 眼在手上时光心与 TCP 实测相差
150mm 上下, 在 600mm 的猜测深度上就是 atan(150/600) = 14°。共线则与深度无关, 猜错也照样打中。

另一族是 swing: 指尖原地不动, 只转腕把工具轴瞄向被点的那个空间点 P = O + depth·û, 即
R·e_z = normalize(P - T)。**指向本身是纯方位问题, 与臂展无关** —— 二维云台激光笔就是这么工作的
(它只出光不出料, 所以没有"枪口要摆到工件前一个靶距"这个位置要求)。它的落点依赖深度, 所以那个
深度必须真测 (读被点像素的对齐深度), 读不到才退回配置靶距, 并把 depth_locked=True 交出去。

谁优先由"手上有没有真深度"决定: 实测到深度就 swing 优先 (±20mm 的深度噪声折算到 1.8m 外的落点
只有 ~1mm, 而指尖零位移、动作最小); 深度读不到就 on-ray 优先, 因为那时 swing 只能瞄猜测深度上
的点, 1.8m 的墙配 600mm 的猜测会偏 20cm 量级 (真机实测过)。臂展只在 on-ray 这一族才是约束:
它要把枪口也挪到工件前一个工艺靶距去, 那是**位置**要求, 来自喷涂工艺而不是"打中"。

指尖真实在哪儿、腕部怎么绕过去都不设额外约束: 方向只吃掉 2 个自由度, 6 轴臂还剩 4 个可挑,
所以"绝大部分像素都能指到"是几何事实, 不是调参结果。

三层职责 (本文件只在服务层):
- 几何与单位换算交给 core.handeye 纯函数内核 (像素射线、姿态构造、位姿归一);
- 硬件通信、状态轮询、IK 闸门、DO 开关交给 robot_service; 画面交给 camera_service;
- 本层只做编排: 互锁 -> 内参 -> 外参 -> 目标点 (被点像素 + 实测深度) -> 生成两族候选并过一次
  IK -> 关喷 -> MovJ -> 读回光束偏离规划方向的量 (落点真值校核由人工点选十字中心完成, 见 mark_cross_center)。

单位口径: 内部一律 mm + rad (RobotPose), 姿态角对用户/日志展示为 deg, 封包换算在驱动层。
"""
import logging
import os
import sys
import time
from typing import Any, Dict, List, Optional, Sequence, Set

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from core.config import sprayer_config
from core.handeye import (
    UNIT_RAD, EYE_IN_HAND, make_transform, matrix_to_pose, normalize_pose, pixel_ray_to_base,
    rotation_closest_to_with_z_axis, rotation_from_pose, tool_frame_from_z_axis,
)
from core.hardware.robot.base_driver import RobotPose
from core.motion.kinematics import flange_pose_from_joints

from apps.robot.services.robot_service import robot_service

logger = logging.getLogger(__name__)

# 允许的像素深度范围 (mm): 对 on-ray 族它只是个标注 (光束就是视线, 深度猜错照样打中);
# 对兜底的 swing 族它就是落点定义 (只能瞄准某一个深度上的点), 所以先读实测深度。
_DISTANCE_RANGE_MM = (60.0, 2500.0)

# 指尖沿射线离光心的安全距离 (mm): 眼在手上时相机与 TCP 固连, 把枪口摆到镜头前面这么近
# 就是让喷枪/激光头去怼镜头与本体的自撞区, 一律排除; 工艺上更希望至少有一个靶距。
_MIN_TIP_AHEAD_MM = 50.0

# 可达球安全裕量: 贴满伸展处控制器几乎必判奇异/超限, 预筛时先把球收缩一圈。
# 这只是“少提几批注定被拒的 IK 请求”的预筛, 不是放行依据 —— 放行与否仍由控制器逆解说了算。
_REACH_MARGIN_RATIO = 0.97

# 候选族 (一次性批量送控制器逆解, 按列表顺序取第一个过关的; 两族的光束都穿过目标点):
#   swing      指尖原地不动, 只转腕把工具轴瞄向被点的那个空间点 -> 纯方位问题, 与臂展无关,
#              动作最小; 代价是落点**锁定在目标点那个深度上**, 所以深度必须真测。
#   on-ray     指尖也摆到观测射线上 (离镜头一个工艺靶距) -> 光束与视线严格重合, 深度无关,
#              且喷嘴离工件的距离符合喷涂工艺; 代价是要移动整条手臂, 而且视线离基座太远时
#              (眼在手上很常见: 相机本身就在基座 1m 外) 控制器会判够不着/近奇异而拒解。
#   re-aim     swing 拧不过去 (被点方向落在"绕小臂轴的腕部锥"外, 腕限位/近奇异被拒) 时的补锥兜底:
#              把指尖绕基座原点转一个方位(J1)+俯仰(J2/J3) —— 旋转轴过原点 => |TCP| 不变 => 指尖仍
#              留在同一可达球壳内 (绝不像 on-ray 那样被拖到臂展边缘), 却换了指尖方位, 于是等效于
#              "先把小臂轴整体转向被点方位、锥重新对准目标, 再让腕子精瞄"。与 swing 同为深度锁定族。
# 谁排前面由 depth_source 决定: 实测到深度就 swing 优先, 读不到就 on-ray 优先 (那时 swing
# 只能瞄猜测深度上的点, 1.8m 的墙配 600mm 的猜测会偏 20cm 量级)。
# 旧实现在兜底位置放的是"与视线平行": 整条光束偏开视线几十到上百 mm, 在任何深度都打不中,
# 而指尖零位移 —— 真机上"点了好几次都还是偏、机械臂看起来根本没动"就是它。
# _RAY_SLIDE_RATIOS 是相对可行窗长度的探点比例, 0 = 意图站位; 越界探点钉回窗口后自然去重。
_RAY_SLIDE_RATIOS = (0.0, 0.12, -0.12, 0.30, -0.30)

# re-aim 族的重定位扫掠步长 (基座系, 单位 deg): 0 那组与 swing 的"原地"候选完全重合, 由
# _add_tip 的"同指尖+同指向"去重自动吃掉, 所以这里只关心非零的补锥偏移。
_REAIM_AZIMUTH_DEG = (0.0, 30.0, -30.0, 60.0, -60.0)   # 绕基座 +Z 转 = 让 J1 把小臂转向被点方位
_REAIM_PITCH_DEG = (0.0, 30.0, -30.0)                   # 绕基座 +Y 转 = 让 J2/J3 抬/压小臂重对准锥

# IK 后备自旋 (度): 整批候选都被拒时, 拿优先级最高的几个指尖绕光束轴再拧一次腕重问一轮 ——
# 光束指向完全不变, 只是把 J6 从限位/奇异上挪开。只在失败路径上发, 成功时仍只有一批往返。
_TWIST_RESERVE_DEG = (45.0, -45.0, 90.0, -90.0)
_RESERVE_TIP_LIMIT = 3

# 眼在手上时允许的最低取流帧率 (fps): 交付档位是 15fps, 低于这个值说明画面已经停滞或在
# 软重启 (真机日志里 waitForFrameset timeout / Capture FPS: 0.26 就是这么印出来的)。此时用户
# 看到的画面不知道是哪个时刻的, 像素与按当前法兰复合的视线就无法保证同源, 直接拒指。
_MIN_VIDEO_FPS = 5.0


# 光斑那一点的表面距离有效窗口 (mm): 超出深度相机量程或低于最小量程的读数一律当无效值
# (天空/零值/空洞填充), 拿它算出的"三维点"只会变成一个假的错位量。
_MIN_SPOT_DEPTH_MM = 200.0
_MAX_SPOT_DEPTH_MM = 8000.0



def _valid_depth_mm(value: float) -> Optional[float]:
    """
    深度读数的有效性判定 (mm): 只有落在深度相机量程窗口内的读数才算数, 其余一律 None。

    单一口径, 两处共用 (被点像素的目标深度 / 光斑那一点的表面深度): 低于下限多半是最小量程
    内的空洞被填成了别的值, 高于上限是天空/零值那一类无效读数 —— 拿它们算出来的"三维点"
    只会变成一个假的错位量或一个假的落点。
    """
    return value if _MIN_SPOT_DEPTH_MM <= value <= _MAX_SPOT_DEPTH_MM else None


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    """绕单位轴 axis 转 angle (rad) 的 3x3 旋转矩阵 (Rodrigues 公式)。"""
    k = np.asarray(axis, dtype=np.float64)
    k = k / max(float(np.linalg.norm(k)), 1e-12)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + float(np.sin(angle)) * K + float(1.0 - np.cos(angle)) * (K @ K)


# 按配置值缓存 (laser_tilt_tool_deg 是运行期常量, 热改需重启; 避免每次指向都重建旋转)。
_LASER_TILT_CACHE: Dict[tuple, tuple] = {}


def _laser_tilt_frames() -> tuple:
    """
    由 spraying.laser_tilt_tool_deg [tx, ty] (deg) 张成补偿用的两个量:

    - b_tool: 真实激光光轴在**工具系**的单位方向 (把 +Z 朝 +X 偏 tx、朝 +Y 偏 ty 后的轴)。
      用正切张成再归一化, 与 mark_cross 记录的 tilt_tool_x_deg=atan2(x_offset, fwd) 同口径, 可直接对消。
    - R_undo: 把 b_tool 转回 +Z 的工具系最小旋转 (R_undo @ b_tool == e3)。

    补偿 [0,0] 时 b_tool=(0,0,1)、R_undo=单位阵, 所有下游退化回旧的"光束==工具+Z"行为。
    """
    tx, ty = sprayer_config.laser_tilt_tool_deg
    key = (round(float(tx), 6), round(float(ty), 6))
    cached = _LASER_TILT_CACHE.get(key)
    if cached is not None:
        return cached
    b = np.array([np.tan(np.radians(tx)), np.tan(np.radians(ty)), 1.0], dtype=np.float64)
    b /= float(np.linalg.norm(b))
    e3 = np.array([0.0, 0.0, 1.0])
    c = max(-1.0, min(1.0, float(b[2])))          # dot(b, e3), b 已归一
    if c > 1.0 - 1e-12:
        r_undo = np.eye(3)                         # 无偏角: 单位阵
    else:
        r_undo = _rodrigues(np.cross(b, e3), float(np.arccos(c)))   # 把 b 旋到 e3 的最小旋转
    _LASER_TILT_CACHE[key] = (b, r_undo)
    return b, r_undo


def _rotate_about_base(v: Sequence[float], azim_deg: float, pitch_deg: float) -> np.ndarray:
    """
    把基座系矢量 v 先绕基座 +Z 转 azim_deg, 再绕基座 +Y 转 pitch_deg (两个旋转轴都过基座原点)。

    绕**过原点**的轴旋转保持 |v| 不变 —— 用它挪 TCP 就等于把指尖留在同一可达球壳上: 换了方位
    (J1) 又换了俯仰 (J2/J3), 却不会像沿视线外推那样被拖去臂展边缘 (那正是 on-ray 常年被拒的原因)。
    角度单位 deg; 返回基座系 mm 矢量 (与输入同量纲)。
    """
    v = np.asarray(v, dtype=np.float64)
    az, pi = np.radians(float(azim_deg)), np.radians(float(pitch_deg))
    cz, sz = np.cos(az), np.sin(az)
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    cp, sp = np.cos(pi), np.sin(pi)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return ry @ (rz @ v)





class LiveAimError(ValueError):
    """用户可直接看见的失败原因 (英文); API 层原样转成 HTTP 400 detail, 不做二次包装。"""




_VERIFY_LOG_NAME = "live_aim.verify"


def _get_verify_logger() -> logging.Logger:
    """
    取 (必要时先配好) 一个把每个"指向真值测量结果"单独落盘的 logger。

    为什么要单独一份: 用户明确要求"界面弹出的信息 / 自检那句 skip / 人工点十字中心量到的
    偏移" 全部自动存成日志, 不要他复制粘贴。除了照常进 backend.log (propagate=True), 还额外
    挂一个专用文件 app/logs/aim_verify.log, 我每次只读这一个文件就能拿到全部现场测量结果。
    幂等: 已挂过 handler 就不重复挂 (开发热重载会反复 import 本模块, 否则会日志重复行)。
    """
    vlog = logging.getLogger(_VERIFY_LOG_NAME)
    if vlog.handlers:
        return vlog
    try:
        from logging.handlers import RotatingFileHandler
        log_dir = os.path.join(PROJECT_ROOT, "logs")
        os.makedirs(log_dir, exist_ok=True)
        handler = RotatingFileHandler(os.path.join(log_dir, "aim_verify.log"),
                                      maxBytes=2_000_000, backupCount=5, encoding="utf-8")
        handler.setFormatter(logging.Formatter("%(asctime)s %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
        vlog.addHandler(handler)
        vlog.setLevel(logging.INFO)
        vlog.propagate = True   # 同时进 root (backend.log), 两处都能查
    except Exception as e:      # 落盘配不起来也绝不影响指向主流程, 只靠 backend.log
        logging.getLogger(__name__).warning("live aim: the dedicated verify log could not be opened: %s", e)
    return vlog


class LiveAimService:
    """单点光束指向的动作编排: 纯几何计算与真机运动严格分开, 前者可在离线环境验证。"""

    def __init__(self) -> None:
        # 上一次"指向到位"的现场快照 (供人工点选十字中心做真值校核):
        #   眼在手上相机随臂走, 点目标点与被点十字中心时相机在两个不同法兰位姿, 必须各用
        #   自己那一刻的法兰反投影回基座系才能对齐 —— 这里存的就是"目标点那一侧"的全部依据。
        #   None = 还没指向过, 人工校核无从参照。
        self._last_aim: Optional[Dict[str, Any]] = None
        # 专用校核日志: 把"界面弹出/自检/人工点选量到的每一个测量结果"落到一个我直接能读的文件,
        # 用户不必复制粘贴。 propagate 仍为 True, 故同一条也会进 backend.log, 两处都能查。
        self._verify_log = _get_verify_logger()

    # ------------------------------------------------------------------ 内部件

    def _camera_intrinsics(self) -> tuple[np.ndarray, Optional[np.ndarray], Optional[tuple[int, int]]]:
        """
        取相机硬件内参: (K 3x3, D 畸变系数或 None, (width, height) 像素或 None)。

        实时页面点的是视频流上的像素, 而流分辨率与内参分辨率可能不同 (前端按归一化
        坐标换算), 所以这里必须把内参自己的 width/height 一起交出去当像素边界依据。
        """
        from apps.camera.services.camera_service import camera_service  # 延迟导入: 相机服务不依赖本模块
        info = camera_service.get_intrinsics_dict()
        K = np.asarray(info.get("intrinsic_matrix") or [], dtype=np.float64)
        if K.size != 9:
            raise LiveAimError(
                "Camera intrinsics are unavailable (the camera service is offline or not calibrated); "
                "a pixel cannot be turned into a ray without them.")
        K = K.reshape(3, 3)
        D_raw = info.get("distortion_coeffs") or []
        D = np.asarray(D_raw, dtype=np.float64).reshape(-1) if len(D_raw) else None
        width, height = info.get("width"), info.get("height")
        size = (int(width), int(height)) if width and height else None
        return K, D, size

    def _joint_flange_pose(self) -> Optional[Sequence[float]]:
        """当前关节反馈 -> **校正后法兰**位姿 (mm, deg); 取不到返回 None (调用方降级, 不报错)。"""
        joints_deg, _ = robot_service.get_current_joint()
        return flange_pose_from_joints(joints_deg) if joints_deg is not None else None

    def _extrinsic_now(self) -> tuple[Any, str, Optional[str]]:
        """
        取**此刻**的相机->基座外参 (4x4, 平移 m)、装法与英文提醒; 不可用则抛 LiveAimError。

        眼在手上: 相机随臂走, 必须用当前关节反馈反推的**法兰**位姿复合 (core.motion.
        kinematics.flange_pose_from_joints), 不能用 30004 的 tool_vector_actual —— 后者是
        当前 tool 的 TCP, 与法兰差一个常量工具变换, 混用会让外参名不副实。
        眼在手外: 常量外参, 与臂在哪无关。
        """
        mount_cfg = sprayer_config.hand_eye_mount
        flange = None
        if mount_cfg == EYE_IN_HAND:
            flange = self._joint_flange_pose()
            if flange is None:
                raise LiveAimError(
                    "Forward kinematics from joint feedback is unavailable, so the camera pose cannot be "
                    "composed for eye-in-hand aiming. Check hardware.robot.robot_urdf and joint_offsets_deg.")

        T, mount, note = sprayer_config.camera_extrinsic_at(flange)
        if T is None:
            raise LiveAimError(
                f"Hand-eye calibration is not usable for live aiming ({mount}): {note}")
        return T, mount, note

    def _tool_length_mm(self, tip_mm: Optional[np.ndarray]) -> float:
        """
        实测当前工具的长度 (mm) = |30004 的 TCP 读数 - FK 出的法兰位置|。

        为什么非要现算而不是读配置: 可达球的半径按**法兰**臂展给 (hardware.robot.max_reach_mm,
        CR5 = 900mm), 而候选指尖是 TK 标定后的 TCP —— 差的就是这一截工具。眼在手上实测约 153mm,
        少加这一截会把视线在球内的可用段从 ~268mm 砍到 ~39mm, on-ray 族整族被误判"够不着",
        只能退到带视差的兜底族 (真机那 20~30cm 的脱靶就是这么来的)。工具长度是平移量的模,
        与法兰姿态无关, 所以两个装法都能这样实测, 且天然跟着示教器上换的 tool 号走。

        :return: 工具长度 (mm); 任一读数缺失时 0.0 (退回按法兰臂展筛, 保守不误放行)
        """
        if tip_mm is None:
            return 0.0
        flange = self._joint_flange_pose()
        if flange is None:
            logger.warning("live aim: flange FK unavailable, reach ball keeps its flange-only radius")
            return 0.0
        return float(np.linalg.norm(np.asarray(tip_mm, dtype=np.float64) -
                                    np.asarray(flange[:3], dtype=np.float64)))

    @staticmethod
    def _validated_distance(distance_mm: Any) -> float:
        """像素深度标注入参防御 (mm): 必须是有限正数且落在物理护栏内。"""
        try:
            val = float(distance_mm)
        except (TypeError, ValueError):
            raise LiveAimError(f"distance_mm must be a number in millimetres, got {distance_mm!r}")
        if not np.isfinite(val) or not (_DISTANCE_RANGE_MM[0] <= val <= _DISTANCE_RANGE_MM[1]):
            raise LiveAimError(
                f"distance_mm {val} is outside the supported range "
                f"{_DISTANCE_RANGE_MM[0]}-{_DISTANCE_RANGE_MM[1]} mm.")
        return val

    @staticmethod
    def _reach_ball_span(point_mm: np.ndarray, dir_unit: np.ndarray, reach_mm: float,
                         ) -> Optional[tuple[float, float]]:
        """
        直线 `point + t*dir` 落在“可达球”(球心 = 基座, 半径 = reach_mm) 内的 t 区间 (mm)。

        t 处到基座的距离平方 = t^2 + 2(P·D)t + |P|^2, 令其 = R^2 解一元二次方程:
            t± = -(P·D) ± sqrt((P·D)^2 - |P|^2 + R^2)
        根号内 < 0 表示这条直线根本不进可达球 -> 返回 None; 调用方不能就此判死 (原地平行摆头
        仍然能把光束打到同一条视线的方向上), 但要把“差多少毫米”说清楚。

        :return: (t_minus, t_plus) 且 t_minus <= t_plus; 直线与球不相交时 None
        """
        p = np.asarray(point_mm, dtype=np.float64)
        d = np.asarray(dir_unit, dtype=np.float64)
        b = float(np.dot(p, d))                  # P·D
        c = float(np.dot(p, p))                  # |P|^2
        disc = b * b - c + reach_mm * reach_mm
        if disc < 0.0:
            return None
        half = float(np.sqrt(disc))
        return -b - half, -b + half

    @staticmethod
    def _closest_approach_mm(point_mm: np.ndarray, dir_unit: np.ndarray) -> float:
        """直线到基座的最近距离 (mm) —— 报“这条视线永远够不着”时给的就是它。"""
        p = np.asarray(point_mm, dtype=np.float64)
        d = np.asarray(dir_unit, dtype=np.float64)
        b = float(np.dot(p, d))
        return float(np.sqrt(max(float(np.dot(p, p)) - b * b, 0.0)))

    @staticmethod
    def _assert_image_pose_pairing() -> None:
        """
        眼在手上的硬前提: 用户点的那一帧画面, 必须对应**此刻**的法兰位姿。

        为什么需要它: 视线是按请求那一刻的 FK 法兰位姿复合的 (EIH: 相机随臂走), 而画面比现场
        落后整条显示链路 (采集→编码→推流→播放器缓冲), 相机断流/软重启时更久。两者不同源时,
        算出来的射线与实际成像的射线差的就是那段时间里臂的旋转量 (实测一次腕部变化就把光线原点
        挪 120mm、方向差 ~7°, 1m 外就是十几到几十厘米), 而本地指标完全看不出来 —— 光束与射线
        共用同一份外参, 是共模量, 自证永远 0.0。

        两条各自独立的必要条件 (缺一不可):
        1. 臂静止时长覆盖显示延迟: 以控制器反馈记账 (robot_service.seconds_since_motion);
        2. 取流本身是活的: 帧率掉底 = 画面停在某个未知时刻。
        眼在手外时相机不动, 视线与臂姿态无关, 调用方按装法跳过这一检查。
        """
        settle_s = sprayer_config.aim_frame_settle_s
        idle_s = robot_service.seconds_since_motion()
        if settle_s > 0.0 and idle_s is not None and idle_s < settle_s:
            raise LiveAimError(
                f"The arm stopped only {idle_s:.1f} s ago, but the live image lags the robot by "
                f">= {settle_s:.1f} s (encode + stream + player buffer). In eye-in-hand aiming the sight "
                f"ray is composed from the arm pose of the frame you clicked, so a click on a stale image "
                f"aims at a direction the camera never saw. Wait until the image settles, then aim again.")

        from apps.camera.services.camera_service import camera_service  # 延迟导入: 与本模块内核同一口径
        status = camera_service.get_status()
        fps = float(status.get("color_fps", 0.0) or 0.0)
        if not status.get("online") or not status.get("streaming") or fps < _MIN_VIDEO_FPS:
            raise LiveAimError(
                f"Live video is stalled or restarting (online={bool(status.get('online'))}, "
                f"streaming={bool(status.get('streaming'))}, capture fps={fps:.1f}), so the image you "
                f"clicked has an unknown age and cannot be paired with the current camera pose. "
                f"Wait for the stream to recover, then aim again.")

    # ------------------------------------------------------------------ 计算

    def plan(self, u_px: float, v_px: float, distance_mm: Optional[float] = None) -> Dict[str, Any]:
        """
        像素 -> 观测射线 (O, û) -> 生成“工具轴线与视线共线”的候选姿态, 并过一次控制器 IK
        闸门。**不动机械臂**。

        两族的光束都**穿过被点的那个空间点**, 差别只在深度敏感度:
        - on-ray: 指尖落在射线上, 光束即视线本身, 深度猜错也照样打中; 意图取“离当前指尖最近的
          那个射线点” (动作最小), 且不得比镜头前一个工艺靶距更近 (防枪头怼镜头), 也不得越过
          可达球边界;
        - swing: 指尖摆不上射线时 (臂展边界) 的兜底 —— 指尖原地不动, 把工具轴直接瞄向目标点。
          它只在目标点那一个深度上打中, 所以深度取**实测值** (读不到才退回配置靶距), 并把
          depth_locked=True 交出去让界面明说“这一枪是锁定深度的”。

        :param u_px, v_px: 像素坐标, 相机内参分辨率坐标系
        :param distance_mm: 显式指定目标点深度 (沿射线 mm); 缺省时先读被点像素的实测深度,
                            读不到再用 spraying.aim_distance_mm
        :return: {"ray_origin_mm", "ray_dir_base", "target_point_mm", "depth_source", "ray_window_mm",
                  "chosen": {...}, "candidates": [...], "unreachable": int, "skipped": [...]}
        :raises LiveAimError: 入参非法 / 内外参不可用 / 一个候选都构造不出来 / 全被控制器拒解
        """
        K, D, size = self._camera_intrinsics()
        T, mount, note = self._extrinsic_now()
        if mount == EYE_IN_HAND:
            # 只有眼在手上需要“画面与位姿同源”: 相机随臂走, 旧画面 = 旧法兰 = 另一条视线。
            self._assert_image_pose_pairing()

        try:
            origin_mm, dir_base = pixel_ray_to_base(u_px, v_px, K, T, D=D, image_size=size)
        except ValueError as e:
            raise LiveAimError(str(e))
        origin_mm = np.asarray(origin_mm, dtype=np.float64)
        dir_base = np.asarray(dir_base, dtype=np.float64)

        # 目标点 = 被点像素 + 那一点的实际表面深度。on-ray 族与深度无关 (光束就是视线), 但兜底的
        # swing 族只能瞄“某一个深度上的点”: 用猜的 600mm 去打 1.8m 的墙, 方向偏差会随距离放大三倍,
        # 所以深度必须真测。实测值只受深度相机量程约束, 不套 _DISTANCE_RANGE_MM (那是给人工输入
        # 的护栏), 否则一面 3m 外的墙会让整个指向请求直接报错。
        if distance_mm is not None:
            display_depth, depth_source = self._validated_distance(distance_mm), "caller"
        else:
            measured = self._clicked_depth_mm(u_px, v_px, size)
            if measured is None:
                display_depth, depth_source = sprayer_config.aim_distance_mm, "config"
            else:
                display_depth, depth_source = measured, "measured"
        target_mm = origin_mm + display_depth * dir_base

        # 当前 TCP: on-ray 族用它挑“动作最小”的站位, swing 族直接拿它当指尖, 同时当自旋参考。
        pose_now, err_pose = robot_service.get_current_pose()
        if pose_now is None:
            logger.warning(f"live aim: current TCP readout failed ({err_pose}); "
                           f"the swing family and the wrist-clock reserve are skipped, "
                           f"and the reach ball stays flange-sized")
            tip_now = None
            rot_now = None
        else:
            tip_now = normalize_pose(pose_now, UNIT_RAD)[:3]
            rot_now = rotation_from_pose(pose_now, UNIT_RAD)

        # 可达球按 TCP 尺寸给: 法兰臂展 + 实测工具长度, 再留一圈奇异/限位裕量。
        tcp_reach_mm = (sprayer_config.robot_max_reach_mm + self._tool_length_mm(tip_now)) * _REACH_MARGIN_RATIO

        tips: List[Dict[str, Any]] = []      # 顺序 = 优先级
        skipped: List[str] = []

        def _add_tip(strategy: str, tip: np.ndarray, aim_at: Optional[np.ndarray] = None) -> None:
            """
            登记一个候选 (指尖 + 光束指向), 并把“光束离被点目标点差多少”一次算清。

            aim_at=None -> 光束就沿视线 (on-ray); 给了点 -> 从指尖直接瞄那个点 (swing)。
            beam_offset_mm 是目标点到光束线的垂距, 两族按构造都应该是 0 —— 留着当构造自检,
            哪天有人把某族的指向算歪了, 它会立刻变成非零而不是静默脱靶。
            """
            tip = np.asarray(tip, dtype=np.float64)
            if aim_at is None:
                axis = dir_base
            else:
                to_aim = np.asarray(aim_at, dtype=np.float64) - tip
                axis = to_aim / max(float(np.linalg.norm(to_aim)), 1e-9)
            for seen in tips:                       # 同一个指尖 + 同一个指向只问控制器一次
                if (float(np.linalg.norm(np.asarray(seen["tip_mm"]) - tip)) < 1.0
                        and float(np.dot(np.asarray(seen["axis_base"]), axis)) > 0.99999):
                    return
            along = float(np.dot(tip - origin_mm, dir_base))    # 指尖在光心前多少 mm (可负)
            to_target = target_mm - tip
            forward = float(np.dot(to_target, axis))            # 目标点沿光束的前向距离 (负 = 枪口背后)
            offset = float(np.linalg.norm(to_target - forward * axis))
            tips.append({
                "strategy": strategy,
                "tip_mm": [round(float(v), 2) for v in tip],
                "axis_base": [round(float(v), 5) for v in axis],
                "along_ray_mm": round(along, 1),
                "beam_offset_mm": round(offset, 1),
                "depth_locked": aim_at is not None,
                "base_distance_mm": round(float(np.linalg.norm(tip)), 1),
            })

        # --- 族 1 on-ray: 把指尖摆到视线与可达球的交段上, 光束即视线本身 (严格打中, 与深度无关)。
        ray_window = None
        span = self._reach_ball_span(origin_mm, dir_base, tcp_reach_mm)
        if span is None:
            skipped.append(f"on-ray: this sight line stays "
                           f"{self._closest_approach_mm(origin_mm, dir_base):.0f} mm from the robot base at its "
                           f"closest, outside the {tcp_reach_mm:.0f} mm TCP reach ball")
        else:
            lo = max(span[0], _MIN_TIP_AHEAD_MM)                 # 镜头前 _MIN_TIP_AHEAD_MM 内是自撞区
            # 上界除了可达球, 还必须卡在**目标点之前**: 站到被点那个点后面去, 光束就朝着背离工件
            # 的方向出去 (那条线上仍然含有目标点, 但它在枪口背后), 那不是打中而是打反。
            hi = min(span[1], _DISTANCE_RANGE_MM[1], display_depth)
            if hi < lo:
                skipped.append(f"on-ray: the reachable part of the sight line [{span[0]:.0f}, {span[1]:.0f}] mm "
                               f"leaves no room for a nozzle that must sit in front of the lens "
                               f"({_MIN_TIP_AHEAD_MM:.0f} mm guard) and still behind the target point "
                               f"({display_depth:.0f} mm deep)")
            else:
                ray_window = (lo, hi)
                # 期望枪口至少离光心一个工艺靶距 (EIH 防枪头贴镜头); 整段可达区更近时就取窗口上界。
                floor = min(max(sprayer_config.spray_distance_mm, lo), hi)
                if tip_now is not None:
                    proj_t = float(np.dot(tip_now - origin_mm, dir_base))   # 视线离当前指尖最近的点
                    intent = min(max(proj_t, floor), hi)
                else:
                    intent = floor
                for ratio in _RAY_SLIDE_RATIOS:
                    t = min(max(intent + ratio * (hi - lo), lo), hi)
                    _add_tip("on-ray", origin_mm + t * dir_base)

        # --- 族 2 swing: 指尖原地不动 (它本来就在可达球内), 把光束直接瞄向被点的那个空间点。
        # 只转腕不移动 -> 与臂展无关, 这正是“二维云台激光笔”的语义 (云台只出光不出料, 所以它
        # 从来没有"枪口要摆到工件前一个靶距"这个位置要求, 也就没有够不着的问题)。
        if tip_now is not None:
            _add_tip("swing", tip_now, aim_at=target_mm)

        # --- 族 2.5 re-aim: swing 被控制器拒解 (腕拧不过去 / 近奇异) 时的补锥兜底 —— 见上方族注释。
        # 把当前 TCP 绕基座原点转方位+俯仰 (半径不变, 仍在可达球内), 在新指尖位置重新把光束瞄向
        # 被点目标点。它把"绕小臂轴的那个腕部锥"整体转向目标, 接住 swing 单一锥面盖不住的方向。
        if tip_now is not None:
            for azim_deg in _REAIM_AZIMUTH_DEG:
                for pitch_deg in _REAIM_PITCH_DEG:
                    _add_tip("re-aim", _rotate_about_base(tip_now, azim_deg, pitch_deg), aim_at=target_mm)

        # 族顺序 = 选择口径 (取第一个通过控制器 IK 的), 由"手上有没有真深度"决定:
        #   有实测深度 -> 先 swing (纯转腕): 指尖零位移、动作最小、与臂展无关; 落点锁在实测深度上,
        #     而 ±20mm 的深度噪声折算到 1.8m 外的落点只有 ~1mm, 完全够用。
        #   深度读不到 -> 先 on-ray (共线): 光束就是视线本身, 深度无关。此时若改用纯转腕, 就只能
        #     瞄那个**猜测**深度上的点 —— 1.8m 的墙配 600mm 的猜测, 落点会偏 20cm 量级 (真机实测过)。
        # 两族都留在同一批候选里: 腕限位/奇异被控制器拒解时自动降级到另一族, 不会"够不着就失败"。
        # sort 是稳定的, 所以同族内部仍按 _RAY_SLIDE_RATIOS 的探点顺序。
        # 族优先级 (取第一个通过控制器 IK 的):
        #   有实测深度 -> swing(原地转腕, 动作最小) > re-aim(原地拧不过去才挪位补锥, 仍锁实测深度)
        #                > on-ray(要整臂大挪移且常年贴臂展边缘被拒)。
        #   深度读不到 -> on-ray(深度无关, 唯一能真打中) > swing > re-aim (后两族只能瞄猜测深度点)。
        # sort 稳定, 同族内部保持 _RAY_SLIDE_RATIOS / 扫掠步长的插入顺序。
        _fam_rank = ({"swing": 0, "re-aim": 1, "on-ray": 2} if depth_source == "measured"
                     else {"on-ray": 0, "swing": 1, "re-aim": 2})
        tips.sort(key=lambda e: _fam_rank.get(e["strategy"], 3))

        if not tips:
            raise LiveAimError(
                f"No fingertip can put the tool axis through this clicked point: "
                f"{[round(float(v), 1) for v in target_mm]} mm at depth "
                f"{display_depth:.0f} mm along the ray. " + " | ".join(skipped))

        # 姿态构造: 工具 +Z = 该候选自己的光束方向 (on-ray 是视线, swing 是“枪口 -> 目标点”),
        # 自旋取“离当前姿态最近”的那个 (转角最小 -> 腕部超限/奇异被拒概率最低);
        # 反馈读数不可用时退回内核规范构造。
        def _pose_of(tip: Sequence[float], axis: Sequence[float], twist_deg: float) -> RobotPose:
            axis_v = np.asarray(axis, dtype=np.float64)
            if rot_now is None:
                R = tool_frame_from_z_axis(axis_v)
            else:
                R = rotation_closest_to_with_z_axis(axis_v, rot_now, twist_deg=twist_deg)
            # 预倾指令姿态 (右乘工具系 R_undo): 让**真实激光光轴** (工具系 b_tool) 而非工具 +Z
            # 落在被点视线上 —— R @ R_undo @ b_tool = R @ e3 = axis_v。补偿关闭时 R_undo=I, 行为不变。
            R = R @ _laser_tilt_frames()[1]
            return RobotPose.from_list(matrix_to_pose(make_transform(R, np.asarray(tip)), UNIT_RAD))

        candidates: List[Dict[str, Any]] = [
            dict(entry, twist_deg=0.0, pose=_pose_of(entry["tip_mm"], entry["axis_base"], 0.0))
            for entry in tips]
        logger.info(
            f"live aim ray: pixel=({float(u_px):.1f}, {float(v_px):.1f}) of {size}, mount={mount}, "
            f"ray_origin_base_mm={[round(float(v), 1) for v in origin_mm]}, "
            f"ray_dir_base={[round(float(v), 4) for v in dir_base]}, "
            f"tool_length={tcp_reach_mm / _REACH_MARGIN_RATIO - sprayer_config.robot_max_reach_mm:.0f} mm "
            f"-> tcp_reach_ball={tcp_reach_mm:.0f} mm, "
            f"on_ray_window_mm={[round(ray_window[0], 1), round(ray_window[1], 1)] if ray_window else 'none'}, "
            f"depth={display_depth:.0f} mm ({depth_source}) -> target_base_mm={[round(float(v), 1) for v in target_mm]}, "
            f"tips={[(c['strategy'], c['along_ray_mm'], c['beam_offset_mm'], c['base_distance_mm']) for c in candidates]} mm, "
            f"skipped={skipped or 'none'}")

        reachable, failed, msg = self._ik_gate(candidates)
        if not reachable and rot_now is not None:
            # 指向没问题却被整批拒解 -> 只剩腕部时钟这一个自由度可用: 绕光束轴拧一拧再问一轮。
            # 后备必须覆盖**每一族的代表**, 不能只取列表头几个: 头几个可能全是同一族 (真机上就是
            # 4 个 on-ray 排在前面), 于是唯一那个与臂展无关的 swing 解从来没被重试过 —— 那次点击
            # 5 个候选 + 12 个后备全被拒, 整次动作什么都没发生。
            first_of_family: List[int] = []
            seen_families: Set[str] = set()
            for i, entry in enumerate(candidates):
                if entry["strategy"] not in seen_families:
                    seen_families.add(entry["strategy"])
                    first_of_family.append(i)
            order = first_of_family + [i for i in range(len(candidates)) if i not in first_of_family]
            reserve: List[Dict[str, Any]] = []
            for entry in [candidates[i] for i in order[:_RESERVE_TIP_LIMIT]]:
                for twist in _TWIST_RESERVE_DEG:
                    reserve.append(dict(entry, twist_deg=twist,
                                        pose=_pose_of(entry["tip_mm"], entry["axis_base"], twist)))
            logger.warning(
                f"live aim: the controller refused all {len(candidates)} aiming candidates "
                f"(message: {msg or 'none'}); retrying {len(reserve)} wrist-clock variants "
                f"twist={_TWIST_RESERVE_DEG} deg at the same beam directions")
            offset = len(candidates)
            candidates.extend(reserve)
            more, more_failed, msg2 = self._ik_gate(reserve)
            reachable = {offset + i for i in more}
            failed += more_failed
            msg = msg2 or msg

        if not reachable:
            logger.warning(
                f"live aim has no reachable pose: ray_dir_base={[round(float(v), 4) for v in dir_base]}, "
                f"target_base_mm={[round(float(v), 1) for v in target_mm]} (depth {depth_source}), "
                f"tips={[(c['strategy'], c['tip_mm'], c['base_distance_mm']) for c in candidates]}, "
                f"controller_message={msg or 'none'}")
            detail = f" ({msg})" if msg else ""
            raise LiveAimError(
                f"The controller refused every pose whose tool axis follows this sight line "
                f"(checked {len(candidates)} fingertip/wrist combinations, including in-place swing and a "
                f"base-sweep re-aim of the forearm{detail}). The arm is folded too far "
                f"away from the line of sight to put its nozzle on it — jog the arm toward the selected point "
                f"and aim again.")

        chosen_idx = min(reachable)
        chosen = candidates[chosen_idx]
        logger.info(
            f"live aim chosen: {chosen['strategy']} fingertip "
            f"{[round(float(v), 1) for v in chosen['tip_mm']]} mm, {chosen['along_ray_mm']:.0f} mm in front of "
            f"the lens, beam {chosen['beam_offset_mm']:.1f} mm off the clicked point, "
            f"depth_locked={chosen['depth_locked']}, twist {chosen['twist_deg']:.0f} deg, "
            f"refused={failed}/{len(candidates)}")
        return {
            "pixel": [round(float(u_px), 1), round(float(v_px), 1)],
            "ray_origin_mm": [round(float(v), 2) for v in origin_mm],
            "ray_dir_base": [round(float(v), 5) for v in dir_base],
            "display_depth_mm": round(display_depth, 1),
            "depth_source": depth_source,
            "target_point_mm": [round(float(v), 2) for v in target_mm],
            "tcp_reach_ball_mm": round(tcp_reach_mm, 1),
            "ray_window_mm": [round(ray_window[0], 1), round(ray_window[1], 1)] if ray_window else None,
            "mount": mount,
            "notice": note,
            "skipped": skipped,
            "candidates": [
                {k: c[k] for k in ("strategy", "tip_mm", "along_ray_mm", "beam_offset_mm",
                                   "base_distance_mm", "twist_deg", "depth_locked", "axis_base")}
                | {"reachable": i in reachable}
                for i, c in enumerate(candidates)
            ],
            "unreachable": failed,
            "chosen": {
                "strategy": chosen["strategy"],
                "twist_deg": round(chosen["twist_deg"], 1),
                "tip_mm": chosen["tip_mm"],
                "along_ray_mm": chosen["along_ray_mm"],
                "beam_offset_mm": chosen["beam_offset_mm"],
                "depth_locked": chosen["depth_locked"],
                "axis_base": chosen["axis_base"],
                "pose_mm_deg": [round(x, 2) for x in normalize_pose(chosen["pose"].to_list(), UNIT_RAD)],
                # 驱动约定的位姿列表 [x,y,z,a,b,c] (mm + rad); 只放原生可序列化类型, 让 plan()
                # 的返值可以直接当成 JSON 给上层 (包括只看不动的预览) 用, 不用解包 RobotPose。
                "pose_rad": chosen["pose"].to_list(),
            },
        }

    @staticmethod
    def _ik_gate(candidates: List[Dict[str, Any]]) -> tuple[Set[int], int, str]:
        """
        把一批姿态一次性送控制器逆解 (每点一次网络往返), 返回 (本批可达下标集合, 被拒数, 原文)。

        IK 闸门按当前生效工具号逆解, 与真实执行同一坐标系; 未连接/不支持时自动全放行。
        候选里存在被拒的是常态 (被拒的只是那个姿态, 不是这条视线), 所以取“第一个可达的”。
        """
        _, failed, msg = robot_service.check_reachability([c["pose"] for c in candidates])
        rejected = set(failed)
        return {i for i in range(len(candidates)) if i not in rejected}, len(rejected), msg

    # ------------------------------------------------------------------ 动作

    def aim_at_pixel(self, u_px: float, v_px: float, distance_mm: Optional[float] = None,
                     speed: Optional[float] = None, acc: Optional[float] = None) -> Dict[str, Any]:
        """
        让工具轴线这条光束落到实时视频里被点那条视线上: 规划 -> 关喷 -> MovJ -> 读回 -> 到位开激光。

        安全顺序是硬性约束:
        1. 互锁 —— 臂在动 (running_status != 0) 一律拒绝, 不并发提交指令;
        2. 工具号先生效 —— TCP 读数、IK 闸门、真实执行必须同源一个工具坐标系;
        3. 规划 + 控制器 IK 闸门 —— 只查不动;
        4. 立即指令关喷涂 DO (fail-close) —— 关不掉就不动, 任何异常都不能带着出料去运动;
        5. MovJ 关节插补单点直达 (空走不描轨迹, 不必保 CP 连续性);
        6. 读回反馈位姿, 量化光束离视线多远 (以反馈为准);
        7. 最后才按配置把 DO 打开 (spraying.aim_do_on_arrive): 排在所有可能抛异常的步骤之后,
           这样任何中途失败都天然停在“已关喷”的安全状态上。当前端接的是激光笔, 这一步就是
           “到位亮激光”用来肉眼确认光斑; 换成真喷枪必须把它关掉, 否则原地堆漆。

        :param speed: MovJ 速度 (deg/s); 缺省用 spraying.aim_speed_percent 折算 (指向是空走, 不必跟着喷涂速度)
        :param acc: 加速度 (%); 缺省读控制面板当前关节加速度
        """
        # 先把请求落日志: 任何一步失败 (互锁/几何/IK) 都能从日志反推“点的到底是哪个像素、带了哪些参数”。
        logger.info(
            f"live aim request: pixel=({float(u_px):.1f}, {float(v_px):.1f}) px, "
            f"distance_mm={distance_mm if distance_mm is not None else 'auto (measured depth, else config)'}, "
            f"speed={speed if speed is not None else f'{sprayer_config.aim_speed_percent}% (config)'}, "
            f"acc={acc if acc is not None else 'panel current'}")
        if not robot_service.is_connected():
            raise LiveAimError("Robot is not connected: live aiming is unavailable.")
        self._assert_idle("before planning")

        # 工具号先下发再规划: plan() 要读当前 TCP 位置、实测工具长度并让控制器按同一 tool 逆解,
        # 坐标系不同源的话算出来的“原地摆头”就不是原地, IK 也不是执行期那个 IK。
        tool_num = robot_service.tool_num
        ok_tool, err_tool = robot_service.set_tool(tool_num)
        if not ok_tool:
            raise LiveAimError(f"Failed to activate tool coordinate {tool_num} before aiming: {err_tool}")

        plan = self.plan(u_px, v_px, distance_mm)

        eff_speed = float(speed) if speed is not None else self._aim_speed_deg_s()
        eff_acc = float(acc) if acc is not None else robot_service.get_speed()[3]
        if eff_speed <= 0 or eff_acc <= 0:
            raise LiveAimError(f"Speed and acceleration must be greater than 0 (got {eff_speed}, {eff_acc}).")

        # 故障安全: 任何动作之前先断料 (立即指令 DOExecute, 毫秒级, 绕过队列)
        ok_do, err_do = robot_service.set_do(robot_service.spray_do_index, 0, immediate=True)
        if not ok_do:
            raise LiveAimError(
                f"Safety interlock: spraying DO{robot_service.spray_do_index} could not be turned off "
                f"({err_do}); the move was aborted.")

        self._assert_idle("before motion")

        ok_mv, err_mv = robot_service.move_to_pose_j(plan["chosen"]["pose_rad"], speed=eff_speed, acc=eff_acc,
                                                    tool_num=tool_num)
        if not ok_mv:
            raise LiveAimError(f"Robot refused the aiming move (MovJ): {err_mv}")

        # 到位后只读一次控制器反馈: 偏差量化与光斑自检必须同源这一次读数, 否则两套口径对不上。
        pose_now, err_pose = robot_service.get_current_pose()
        report = self._measure(plan, pose_now, err_pose)   # 读不到反馈时各偏差项是 None, 不能让日志拼格式炸掉
        chosen = plan["chosen"]


        # 到位亮激光 (fail-close 之后的唯一开阀点, 见 docstring 第 7 步)
        beam_on = False
        do_on = ""
        if sprayer_config.aim_do_on_arrive:
            ok_on, err_on = robot_service.set_do(robot_service.spray_do_index, 1, immediate=True)
            beam_on = bool(ok_on)
            do_on = "on" if ok_on else f"on-failed({err_on})"
            if not ok_on:
                logger.warning(f"live aim: the arm arrived but DO{robot_service.spray_do_index} "
                               f"could not be switched on: {err_on}")
        else:
            do_on = "off (spraying.aim_do_on_arrive is disabled)"


        # 存下这一枪的**真值参照**: 人工点选十字中心做校核时要用"目标点那一侧"的全部依据
        # (被点像素、基座系目标点、那条观测射线的原点/方向/深度、规划光束方向)。
        self._last_aim = {
            "pixel": plan["pixel"],
            "ray_origin_mm": plan["ray_origin_mm"],
            "ray_dir_base": plan["ray_dir_base"],
            "target_point_mm": plan["target_point_mm"],
            "display_depth_mm": plan["display_depth_mm"],
            "depth_source": plan["depth_source"],
            "mount": plan["mount"],
            "strategy": chosen["strategy"],
            "planned_beam_axis_base": chosen["axis_base"],
            "arrived_pose_mm_deg": report.get("actual_pose_mm_deg"),
            "beam_on": beam_on,
            "ts": time.time(),
        }
        # 每一次"指向到位"都顺手写进专用校核日志: 界面弹出的信息与自检那句 skip 全落盘,
        # 用户不必复制粘贴 (见 _get_verify_logger)。
        self._verify_log.info(
            f"AIM-AT-PIXEL pixel={plan['pixel']} -> target base {plan['target_point_mm']} mm "
            f"(depth {plan['display_depth_mm']} mm, {plan['depth_source']}) via {chosen['strategy']}, "
            f"beam {report['beam_angle_deg']} deg off the planned beam, spot {report['spot_error_mm']} mm "
            f"off the clicked point, laser DO{robot_service.spray_do_index} -> {do_on}")

        logger.info(
            f"live aim done: tool axis put through the clicked point at base "
            f"{[round(float(v), 1) for v in plan['target_point_mm']]} mm "
            f"(depth {plan['display_depth_mm']} mm, {plan['depth_source']}) via {chosen['strategy']} "
            f"(fingertip {chosen['tip_mm']}, {chosen['along_ray_mm']:.0f} mm in front of the lens, "
            f"twist {chosen['twist_deg']:.0f} deg), spot {report['spot_error_mm']} mm off the clicked point "
            f"({report['beam_angle_deg']} deg off the planned beam), DO{robot_service.spray_do_index} -> {do_on}")
        return {
            "status": "moved",
            "mount": plan["mount"],
            "notice": plan["notice"],
            "pixel": plan["pixel"],
            "ray_origin_mm": plan["ray_origin_mm"],
            "ray_dir_base": plan["ray_dir_base"],
            "display_depth_mm": plan["display_depth_mm"],
            "target_point_mm": plan["target_point_mm"],
            "strategy": chosen["strategy"],
            "depth_locked": chosen["depth_locked"],
            "depth_source": plan["depth_source"],
            "twist_deg": chosen["twist_deg"],
            "tip_mm": chosen["tip_mm"],
            "along_ray_mm": chosen["along_ray_mm"],
            "planned_offset_mm": chosen["beam_offset_mm"],
            "target_pose_mm_deg": chosen["pose_mm_deg"],
            "candidates": plan["candidates"],
            "unreachable": plan["unreachable"],
            "speed_deg_s": round(eff_speed, 1),
            "acc_percent": round(eff_acc, 1),
            "tool_num": tool_num,
            "spray_do_index": robot_service.spray_do_index,
            "spray_do_state": do_on,
            **report,
        }

    @staticmethod
    def _aim_speed_deg_s() -> float:
        """
        指向动作的关节速度 (deg/s): 由 spraying.aim_speed_percent 按驱动上限折算。

        move_to_pose_j 会把 deg/s 再折算回 SpeedJ 百分比, 这里按同一个上限反算,
        所以配置里写的百分比就是控制器收到的 SpeedJ 百分比 (单一换算口径, 不重复定义)。
        """
        pct = min(max(float(sprayer_config.aim_speed_percent), 1.0), 100.0)
        limits = robot_service.max_joint_speed_deg_s
        max_jnt = limits[0] if limits else 180.0
        return pct / 100.0 * float(max_jnt)

    @staticmethod
    def _assert_idle(stage: str) -> None:
        """状态互锁: 以控制器反馈 (running_status) 为唯一真理, 运动中拒绝下发新动作。"""
        if robot_service.is_moving():
            raise LiveAimError(
                f"Robot is already moving ({stage}): live aiming is locked while the arm runs.")

    @staticmethod
    def _clicked_depth_mm(u_px: float, v_px: float, size: Optional[tuple]) -> Optional[float]:
        """
        读被点像素那一点的实际表面距离 (mm): 与彩色**对齐**的 uint16 深度图。

        为什么指向要吃它: 兜底的 swing 族是“从枪口瞄那个点”, 点定在哪儿就决定了光束方向;
        拿猜的靶距会把方向误差按 (真实深度 / 猜测深度) 放大。on-ray 族与深度无关, 读了也不影响。
        深度服务不在线、没开对齐、该像素是空洞或超出量程时返回 None, 由调用方退回配置靶距 ——
        宁可标注“用的是猜的深度”, 也不拿一个 0 当距离。
        """
        try:
            from apps.camera.services.camera_service import camera_service  # 延迟导入: 与本模块内核同口径
            depth = camera_service.get_depth_frame()
        except Exception as e:
            logger.warning(f"live aim: the clicked pixel's depth is unavailable ({e}); "
                           f"falling back to the configured aiming distance")
            return None
        if depth is None or depth.ndim != 2:
            return None
        h, w = int(depth.shape[0]), int(depth.shape[1])
        # 深度网格与内参档位可能不同: 索引按各自分辨率换算
        col = int(round(float(u_px) * (w / size[0]))) if size else int(round(float(u_px)))
        row = int(round(float(v_px) * (h / size[1]))) if size else int(round(float(v_px)))
        if not (0 <= row < h and 0 <= col < w):
            return None
        return _valid_depth_mm(float(depth[row, col]))

    @staticmethod
    def _measure(plan: Dict[str, Any], pose_now: Optional[Sequence[float]],
                 readback_error: str = "") -> Dict[str, Any]:
        """
        根据**已读回**的控制器反馈位姿, 量化“光束与视线差多少” (mm / deg)。

        三个量各有分工, 缺一个看不住一个出错渠道:
        - spot_error_mm: 被点目标点 (射线 + 实测深度) 到**实际光束线**的垂距 = 肉眼看到的脱靶量;
          它同时吃了位置与指向两种误差, 且随距离放大 (1 度角在 950mm 处是 17mm);
        - beam_angle_deg: 实际工具 +Z 与**规划光束方向**的夹角 = 纯跟踪角误差, 与远近无关。
          基准必须是规划方向而不是视线: swing 族的光束本来就不与视线同向 (它从枪口斜着瞄目标点),
          拿视线当基准会把一个走到位的动作报成十几度的失败;
        - nozzle_in_front_of_lens_mm: 枪口在光心前多少 mm (负值 = 跑到相机背后去了)。

        注意口径上限: 基准射线与被点像素来自同一份手眼外参, 所以**标定本身的角误差是共模量,
        本地永远量不出来** —— 本项只能证明“机械臂确实走到了我们算的那条线上”。若肉眼仍见
        光斑偏离, 剩下的只可能是外参/关节零位那条链路, 而那恰好要靠人工点选十字中心
        (mark_cross_center) 拿相机直接量出来。
        两族的 spot_error 理论上都只剩控制器重复定位残差 (光束按构造穿过目标点); swing 族的
        落点还额外依赖深度读数准不准, 那部分同样由人工点选十字中心拿相机直接量。

        :param pose_now: 到位后那一次 get_current_pose 的位姿 (mm + rad); None = 反馈读不到
        :param readback_error: 读不到时的驱动原文, 原样回显给用户
        """
        origin = np.asarray(plan["ray_origin_mm"], dtype=np.float64)
        ray_dir = np.asarray(plan["ray_dir_base"], dtype=np.float64)
        target = np.asarray(plan["target_point_mm"], dtype=np.float64)
        if pose_now is None:
            logger.warning(f"live aim: pose readback failed after the move: {readback_error}")
            return {"actual_pose_mm_deg": None, "spot_error_mm": None, "beam_angle_deg": None,
                    "nozzle_in_front_of_lens_mm": None, "readback_error": readback_error}

        planned_axis = np.asarray(plan["chosen"]["axis_base"], dtype=np.float64)
        actual = normalize_pose(pose_now, UNIT_RAD)          # mm + deg (内核统一口径)
        # 模型光束轴 = 工具 +Z 经安装角偏后的真实光轴 (R·b_tool); 补偿关闭时 b_tool=+Z, 与旧口径一致。
        R_now = rotation_from_pose(pose_now, UNIT_RAD)
        axis = R_now @ _laser_tilt_frames()[0]               # 真实光轴在基座系的方向
        to_tip = actual[:3] - origin
        along = float(np.dot(to_tip, ray_dir))               # 枪口沿视线的前向位置 (负 = 相机背后)
        to_target = target - actual[:3]
        depth = float(np.dot(to_target, axis))               # 目标点沿光束的前向距离 (负 = 枪口背后)
        spot_error = float(np.linalg.norm(to_target - depth * axis))    # 光斑离被点有多远
        angle = float(np.degrees(np.arccos(max(-1.0, min(1.0, float(np.dot(axis, planned_axis)))))))
        return {
            "actual_pose_mm_deg": [round(float(x), 2) for x in actual],
            "spot_error_mm": round(spot_error, 1),
            "beam_angle_deg": round(angle, 2),
            "nozzle_in_front_of_lens_mm": round(along, 1),
        }


    # ------------------------------------------------- 人工点选十字中心真值校核

    def log_ui_notice(self, level: str, message: str) -> None:
        """
        把界面当时弹给用户的一条提示/通知原文写进专用校核日志。

        存在的理由 (用户明确要求): "cross center at pixel… / X deg off… / skipped (N blobs…)"
        这些都只弹在界面上, 以前要用户手动复制粘贴给我。前端每次 setNotice 都顺手 POST 过来,
        我直接读 app/logs/aim_verify.log 就够了。消息本身就是界面上的英文, 不做加面包装。
        """
        text = (message or "").strip()
        if not text:
            return
        lvl = {"error": logging.WARNING, "warn": logging.WARNING}.get((level or "info").lower(),
                                                                        logging.INFO)
        self._verify_log.log(lvl, f"UI-NOTICE [{level}] {text}")

    def mark_cross_center(self, u_px: float, v_px: float) -> Dict[str, Any]:
        """
        人工点选"肉眼看到的十字中心", 绕过脆弱的 CV 检测, 直接量这一枪激光真实落在哪里、离目标点差多少。

        为什么这样最准 (用户提出): 现场墙上激光碎成散斑/被多处反光淹没, 自动差分要么 skip 要么
        把碎块当中心; 而**人眼是最强的检测器**。难点在相机装在末端会随臂走: 点目标点与点十字
        中心发生在**两个不同的法兰位姿**, 画面自然不同 —— 解法就是把两次点击各自用它那一刻的
        相机位姿 (EIH: 此刻法兰复合出的 相机->基座) 反投影回**同一个基座系**再比。

        于是交回两个正交的真值 (哪个大就标定哪个):
        - beam_axis_angle_deg / beam_axis_gap_mm: 真实十字三维点到"过到位 TCP、沿工具 +Z"这条
          **模型光束线**的垂距及它对应的角 —— 不为 0 就说明激光光轴 != 工具 +Z (安装角偏),
          这是本地任何读数都看不到的、唯一没被验过的假设 (planning 一直硬编码光束=工具 +Z);
        - miss_vs_target_mm: 真实十字三维点与被点目标点 (基座系) 的空间距离 = 肉眼看到的总脱靶量;
          沿目标视线的分量 (miss_depth_mm) 与垂直分量 (miss_lateral_mm) 分开展出, 各自住一条出错渠道。
        两个量都需要深度把"一条射线"定成"一个三维点": 读不到时退化为只报两条视线夹角
        (sight_angle_deg, 与距离无关) 并明说没有三维量, 绝不编一个数。

        :param u_px, v_px: 用户点下的十字中心像素 (相机内参分辨率坐标系)
        :raises LiveAimError: 未连接 / 还没指向过 (无从参照) / 臂在动或画面没跟上 (点到的不是此刻视野)
        """
        if not robot_service.is_connected():
            raise LiveAimError("Robot is not connected: cross-centre verification is unavailable.")
        if self._last_aim is None:
            raise LiveAimError(
                "Nothing to verify yet: aim at a point first, wait for the arm to arrive and the laser "
                "to switch on, then click the centre of the cross you see.")
        self._assert_idle("before marking the cross centre")

        aim = self._last_aim
        if aim["mount"] == EYE_IN_HAND:
            # 十字中心视线是按"请求此刻"的法兰复合的; 画面没跟上到位姿态 = 点的不是这片视野。
            self._assert_image_pose_pairing()

        K, D, size = self._camera_intrinsics()
        # 此刻 (到位后) 相机->基座外参: _extrinsic_now 读当前关节反推的法兰, 正是看十字那一帧的相机。
        T_now, _mount, _note = self._extrinsic_now()
        try:
            origin2, dir2 = pixel_ray_to_base(u_px, v_px, K, T_now, D=D, image_size=size)
        except ValueError as e:
            raise LiveAimError(str(e))
        origin2 = np.asarray(origin2, dtype=np.float64)
        dir2 = np.asarray(dir2, dtype=np.float64)
        origin1 = np.asarray(aim["ray_origin_mm"], dtype=np.float64)
        dir1 = np.asarray(aim["ray_dir_base"], dtype=np.float64)
        target = np.asarray(aim["target_point_mm"], dtype=np.float64)

        # 两条视线夹角: 目标点那一侧 与 十字中心那一侧, 都是基座系单位方向; 这个量与距离无关。
        sight_angle_deg = round(float(np.degrees(np.arccos(
            max(-1.0, min(1.0, float(np.dot(dir1, dir2))))))), 2)

        result: Dict[str, Any] = {
            "marked_pixel": [round(float(u_px), 1), round(float(v_px), 1)],
            "target_pixel": aim["pixel"],
            "target_point_mm": [round(float(v), 1) for v in target],
            "strategy": aim["strategy"],
            "depth_source": aim["depth_source"],
            "sight_angle_deg": sight_angle_deg,
            "laser_on_when_aimed": bool(aim["beam_on"]),
            "cross_point_mm": None,
            "cross_depth_mm": None,
            "miss_vs_target_mm": None,
            "miss_lateral_mm": None,
            "miss_depth_mm": None,
            "beam_axis_gap_mm": None,
            "beam_axis_angle_deg": None,
            # 工具系下的光束倾角分量 (deg): 倾角大小只能告诉你歪了多少度, 但要把常量补偿
            # 写进外参还得知道歪的方向; 下面两个分量在**固定安装角偏**下应当跨任意臂姿态几乎不变。
            "tilt_tool_x_deg": None,   # 光束偏离 +Z 朝工具 +X 方向的角分量
            "tilt_tool_y_deg": None,   # 光束偏离 +Z 朝工具 +Y 方向的角分量
            "tilt_azimuth_tool_deg": None,  # 上述两个分量合成的工具系方位角 (固定量)
        }

        depth_mm = self._clicked_depth_mm(float(u_px), float(v_px), size)
        if depth_mm is not None:
            # 深度图存的是**沿光轴 Z** 的距离而不是斜距, 所以射线参数要除以射线与光轴的 z 分量。
            cam_z = np.asarray(T_now, dtype=np.float64)[:3, 2]
            cross_point = origin2 + (depth_mm / float(np.dot(dir2, cam_z))) * dir2
            result["cross_point_mm"] = [round(float(v), 1) for v in cross_point]
            result["cross_depth_mm"] = round(depth_mm, 1)

            miss_vec = cross_point - target
            result["miss_vs_target_mm"] = round(float(np.linalg.norm(miss_vec)), 1)
            along_target = float(np.dot(miss_vec, dir1))              # 沿目标视线 = 深度差那一路
            result["miss_depth_mm"] = round(along_target, 1)
            lateral = miss_vec - along_target * dir1
            result["miss_lateral_mm"] = round(float(np.linalg.norm(lateral)), 1)

            # 模型光束线 (过到位 TCP、沿**真实光轴** = 工具 +Z 经安装角偏补偿) 与真实十字点的垂距。
            pose_now, err_pose = robot_service.get_current_pose()
            if pose_now is not None:
                b_tool, r_undo = _laser_tilt_frames()
                R = rotation_from_pose(pose_now, UNIT_RAD)            # 3x3, 列 = 工具各轴在基座系的方向
                tip = np.asarray(normalize_pose(pose_now, UNIT_RAD)[:3], dtype=np.float64)
                axis = R @ b_tool                                     # 真实光轴在基座系的方向 (补偿后)
                to_pt = cross_point - tip
                fwd = float(np.dot(to_pt, axis))                     # 沿光束的前向距离 (负=十字在枪后)
                perp = to_pt - fwd * axis
                gap = float(np.linalg.norm(perp))
                result["beam_axis_gap_mm"] = round(gap, 1)
                if fwd > 0.0:
                    result["beam_axis_angle_deg"] = round(float(np.degrees(np.arctan2(gap, fwd))), 2)
                    # 把基座系偏移表达到**光轴系** (r_undo·R^T·to_pt, 其 Z 轴=真实光轴): 报的是残差
                    # 倾角 —— 补偿正确时这三项应收敛到 ~0; 仍稳定非零说明常量取偏了或还有随姿态的误差。
                    tilt_tool = r_undo @ (R.T @ to_pt)
                    result["tilt_tool_x_deg"] = round(float(np.degrees(np.arctan2(tilt_tool[0], fwd))), 2)
                    result["tilt_tool_y_deg"] = round(float(np.degrees(np.arctan2(tilt_tool[1], fwd))), 2)
                    result["tilt_azimuth_tool_deg"] = round(
                        float(np.degrees(np.arctan2(tilt_tool[1], tilt_tool[0]))), 1)
            else:
                result["readback_error"] = err_pose
        else:
            result["note"] = ("no usable depth at the marked pixel, so only the sight-line angle could be "
                              "measured (the mm terms need a 3D point)")

        self._verify_log.info(f"MARK-CROSS {self._format_verify(result)}")
        return {"status": "verified", **result}

    @staticmethod
    def _format_verify(r: Dict[str, Any]) -> str:
        """把一次人工校核量到的全部数字压缩成一句英文日志 (None 项写成 'n/a', 不打印 'None')。"""
        txt = lambda key, unit="": ("n/a" if r.get(key) is None
                                     else f"{r[key]}{unit}")
        return (f"pixel {r['marked_pixel']} (target pixel {r['target_pixel']}): "
                f"cross point base {r['cross_point_mm']} mm at surface {txt('cross_depth_mm', ' mm')}, "
                f"sight line {r['sight_angle_deg']} deg off the clicked one, "
                f"miss {txt('miss_vs_target_mm', ' mm')} off the target "
                f"(lateral {txt('miss_lateral_mm', ' mm')}, depth {txt('miss_depth_mm', ' mm')}), "
                f"beam axis {txt('beam_axis_gap_mm', ' mm')} off the modelled beam "
                f"(= {txt('beam_axis_angle_deg', ' deg')} tilt of the real laser vs tool +Z, "
                f"tool-frame tilt {txt('tilt_tool_x_deg')}x/{txt('tilt_tool_y_deg')}y deg "
                f"@ azimuth {txt('tilt_azimuth_tool_deg', ' deg')}), "
                f"laser_on={r['laser_on_when_aimed']}, depth_source={r['depth_source']}")


live_aim_service = LiveAimService()
