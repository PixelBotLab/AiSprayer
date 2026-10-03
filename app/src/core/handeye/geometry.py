# -*- coding: utf-8 -*-
"""
手眼标定的坐标系约定与 SE(3) 基础运算。

两种安装的数学模型（本项目统一用 T_<a>_<b> 表示 "b 系中的点表达成 a 系" 的变换）：

  eye-to-hand (眼在手外): 相机固定于基座，标定板刚性装在法兰上
      标定板在基座系下的位姿随法兰运动: P_board_base(i) = P_base_flange(i) + R_base_flange(i) · t_off
      未知量 = T_base_camera (常量) + t_off (板相对法兰的 TCP 偏移)

  eye-in-hand (眼在手上): 相机固定于法兰，标定板固定于世界
      T_base_flange(i) · T_flange_camera · T_camera_board(i) = T_base_board
      未知量 = T_flange_camera (常量, 即 AX=XB 的 X) + T_base_board

所有平移单位一律为毫米 (mm)，与标定板 square_size_mm / solvePnP 输出一致；
唯一的例外是 `resolve_camera_extrinsic` —— 它是把外参交付给运行时的出口，米制换算只在
那一处发生 (下游的点云 / 规划 / URDF 统一用米)，以避免同一个换算在多处各写一遍而漂移。
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

EYE_TO_HAND = "eye-to-hand"
EYE_IN_HAND = "eye-in-hand"
MOUNTS: tuple[str, str] = (EYE_TO_HAND, EYE_IN_HAND)

# 法兰位姿口径 (pose frame convention) —— 与装法正交的第二个维度。
#   controller_v1    : 手眼求解吃的是 FK(q_encoder) / TCP 读数, 即控制器自己的位姿 (历史默认)
#   joint_offset_v2  : 吃的是 FK(q_encoder + Δq), 关节零位偏移已补偿
# 同一个物理装法下这两种口径解出来的外参**不可互换**: 实测两份 X 差 14.44mm / 1.59°。
# 口径字段是后加的, 历史文件一律读作 controller_v1。
POSE_FRAME_CONTROLLER_V1 = "controller_v1"
POSE_FRAME_JOINT_OFFSET_V2 = "joint_offset_v2"
POSE_FRAMES: tuple[str, str] = (POSE_FRAME_CONTROLLER_V1, POSE_FRAME_JOINT_OFFSET_V2)

# Dobot 控制器回报的 [x, y, z, rx, ry, rz] 中姿态部分满足
#   R = Rz(rz) · Ry(ry) · Rx(rx)      (定系下的矩阵乘序；等价于 scipy 内禀序列 'xyz')
# 注意别把"内禀 xyz"读成 Rx·Ry·Rz —— 内禀是绕**动**轴依次转 x→y→z，写成定系矩阵乘积时
# 顺序反过来。该对应关系已用 CR5Kinematics.forward_controller 逐轴随机姿态实测，与
# Rotation.from_matrix(T_ctrl).as_euler('xyz', degrees=True) 完全一致 (<1e-15)。
# 老版求解器靠 12 顺规 x 8 符号网格搜索"猜"出这个约定, 现在直接确定化,
# 网格搜索仅作为异常控制器固件的兜底保留在 eye_to_hand 里。
DOBOT_EULER_SEQ = "xyz"

UNIT_DEG = "deg"
UNIT_RAD = "rad"

# 本包内部一律以"度"为姿态单位。注意仓库其它层把机器人位姿存成弧度
# (dobot_driver.get_current_pose 把 Dobot 的度 math.radians 了一下), 所以
# 写入 session 时必须显式声明来源单位, 由 normalize_pose 负责换算。
# 旧版靠 evaluate_data_diversity 里 "max(|rpy|) < 7.0 即视为弧度" 的幅值启发式
# 猜测单位, 姿态恰好接近 0 时会误判成度, 这里改为显式记录。
INTERNAL_ANGLE_UNIT = UNIT_DEG

# 各安装方式的最少样本数。AX=XB 理论下界是 3, 但眼在手上对旋转多样性极其敏感,
# 低于 5 个样本时旋转退化到无唯一解, 因此硬性提高门槛。
MIN_SAMPLES: Dict[str, int] = {EYE_TO_HAND: 3, EYE_IN_HAND: 5}
RECOMMENDED_SAMPLES: Dict[str, int] = {EYE_TO_HAND: 12, EYE_IN_HAND: 20}

_POSE_KEYS = ("x", "y", "z")
_ROT_KEYS = (("rx", "a"), ("ry", "b"), ("rz", "c"))

PoseLike = Any  # dict | Sequence[float] | np.ndarray


def resolve_result_mount(data: Dict[str, Any], default: str = EYE_TO_HAND) -> str:
    """
    从一份标定文件 (calibration_result.yaml / calibration_info.yaml) 读出相机安装方式。

    装法字段改过三代 (无 → calibration_mode → hand_eye_mount, 且早期写在 metadata
    里), 老 session 与老的全局结果文件都必须仍能读出正确装法; 写了但不是已知值的,
    按眼在手外处理 —— 那是本项目此前唯一支持的装法, 认不得的装法当错误值处理比当
    真值用安全。

    全局消费方 (config / 交互式重建 / 标定服务) 共用这一份判定, 避免同一文件在不同
    入口被认成不同装法。

    :param data: 标定 yaml 解析出的字典
    :param default: 文件里压根没写装法时的回落值 (调用方可传配置默认)
    """
    meta = data.get("metadata") or {}
    mount = (data.get("hand_eye_mount") or data.get("calibration_mode")
             or meta.get("hand_eye_mount") or meta.get("calibration_mode"))
    if not mount:
        return default
    return EYE_IN_HAND if mount == EYE_IN_HAND else EYE_TO_HAND


def resolve_result_pose_frame(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    读出一份标定文件的法兰位姿口径: {"frame": str, "joint_offsets_deg": Optional[List[float]]}。

    为什么缺字段必须解释为 controller_v1 而不是"当前运行时口径": 那些结果是在
    "FK 不加偏移"的约定下解出来的; 若把缺字段认作当前口径, 改一次配置就会让一切
    历史标定文件"自动跟着变口径", 那正是静默错位的源头 (与装法判定同一个教训)。

    :param data: calibration_result.yaml / calibration_info.yaml 解析出的字典
    :return: frame 总是 POSE_FRAMES 之一; joint_offsets_deg 在声明 v2 但偏移不可用时为 None
             (调用方必须把它当不匹配处理, 不能当零偏移处理)
    """
    data = data or {}
    meta = data.get("metadata") or {}
    frame = data.get("pose_frame_convention") or meta.get("pose_frame_convention")
    if frame != POSE_FRAME_JOINT_OFFSET_V2:
        return {"frame": POSE_FRAME_CONTROLLER_V1, "joint_offsets_deg": [0.0] * 6}

    raw = data.get("joint_offsets_deg") or meta.get("joint_offsets_deg")
    try:
        offs = [float(v) for v in (raw or [])]
    except (TypeError, ValueError):
        offs = []
    return {"frame": POSE_FRAME_JOINT_OFFSET_V2,
            "joint_offsets_deg": offs if len(offs) == 6 else None}


POSE_FRAME_OFFSET_TOL_DEG = 1e-6   # 结果里声明的偏移与当前配置偏移的等价容差 (deg)


def pose_frame_mismatch(result_data: Optional[Dict[str, Any]],
                        runtime_frame: str,
                        runtime_offsets_deg: Sequence[float]) -> Optional[str]:
    """
    对比一份标定结果的法兰位姿口径与当前运行时口径; 匹配返回 None, 不匹配返回英文原因。

    为什么必须拦: 同一装法下两种口径解出来的外参**不可互换** (实测两份 X 差
    14.44mm / 1.59°), 而且错用是**静默**的 —— 重投影误差与置信度判决链吃的是同一批
    带口径的位姿, 它们自己看不出自相矛盾, 只有上机喷歪才会暴露。

    判定封在这里而不是配置层: 加载守卫、发布预检与界面待确认标记读的是同一个结论。

    :param result_data: 标定结果 yaml 解析出的字典 (空字典 / None 视为无结果, 不拦)
    :param runtime_frame: 当前运行时的口径 (POSE_FRAMES 之一; 未知值按 controller_v1)
    :param runtime_offsets_deg: 当前配置的关节零位偏移 (deg, 6 轴)
    :return: None = 可用; 否则为可直接展示给用户的英文原因
    """
    if not result_data:
        return None
    got = resolve_result_pose_frame(result_data)
    want = runtime_frame if runtime_frame in POSE_FRAMES else POSE_FRAME_CONTROLLER_V1
    if got["frame"] == want:
        if want == POSE_FRAME_CONTROLLER_V1:
            return None
        offs = got["joint_offsets_deg"]
        if offs is None:
            return ("Calibration declares joint-offset poses but carries no usable "
                    "joint_offsets_deg values; re-run the calibration")
        cur = [float(v) for v in (runtime_offsets_deg or [])]
        if len(cur) != 6 or any(abs(a - b) > POSE_FRAME_OFFSET_TOL_DEG for a, b in zip(offs, cur)):
            return (f"Calibration was solved with joint offsets {offs} deg but the "
                    f"current configuration uses {cur} deg; re-run the calibration "
                    f"or restore hardware.robot.joint_offsets_deg")
        return None
    if want == POSE_FRAME_JOINT_OFFSET_V2:
        return ("Calibration poses are uncorrected (controller_v1) while "
                "hardware.robot.joint_offsets_deg is non-zero (joint_offset_v2); "
                "re-run the calibration so its flange poses include the offsets")
    return (f"Calibration pose frame '{got['frame']}' does not match the current "
            f"runtime frame '{want}'; re-run the calibration")


def declare_pose_frame(runtime_frame: str, n_flange_derived: int,
                       n_flange_recorded: int, n_tcp_fallback: int,
                       n_observations: int) -> str:
    """
    本轮标定结果应当声明的法兰位姿口径 (与 pose_frame_mismatch 同一套规则的另一半)。

    只有当带 Δq 的 FK 确实喂进了手眼约束 (运行时偏移非零, 且**全部**样本都是现场由
    关节反推) 才能声明 joint_offset_v2。旧会话落盘的 flange_pose 缓存与 TCP 读数都是
    控制器口径, 混进一个就是混口径结果 —— 宁可降级为 controller_v1 让消费方拒用, 也不能
    谎报; 谎报的后果是静默错位 (实测 14.44mm / 1.59°), 比拒用难查得多。

    :param runtime_frame: 当前运行时口径 (SprayerConfig.pose_frame_convention)
    :param n_flange_derived: 现场 FK(q+Δq) 反推的样本数
    :param n_flange_recorded: 用落盘 flange_pose 缓存的样本数 (旧口径)
    :param n_tcp_fallback: 退回控制器笛卡尔读数的样本数
    :param n_observations: 真正进入求解的样本总数
    """
    if runtime_frame != POSE_FRAME_JOINT_OFFSET_V2:
        return POSE_FRAME_CONTROLLER_V1
    all_live_fk = (n_flange_derived > 0
                   and n_flange_derived == n_observations
                   and n_flange_recorded == 0 and n_tcp_fallback == 0)
    return POSE_FRAME_JOINT_OFFSET_V2 if all_live_fk else POSE_FRAME_CONTROLLER_V1


def resolve_camera_extrinsic(result_data: Optional[Dict[str, Any]],
                             flange_pose_mm_deg: Optional[Sequence[float]] = None,
                             *,
                             runtime_frame: Optional[str] = None,
                             runtime_offsets_deg: Optional[Sequence[float]] = None,
                             ) -> tuple[Optional[List[List[float]]], str, Optional[str]]:
    """
    一份标定结果 (+ 可选的拍摄时刻法兰位姿) -> 相机在基座系的位姿; 两种装法共用的唯一入口。

    - 眼在手外: 结果里就是基座系常量, 与机械臂此刻在哪无关, 直接取 (传入的法兰位姿被忽略)。
    - 眼在手上: T_base_camera = T_base_flange(法兰位姿) · T_flange_camera, 每拍摄一张图都不同,
      所以必须给出 flange_pose_mm_deg; 给不出就是“不知道相机此刻在哪”, 没有降级可用 —— 拿旧的
      或拿当前的凑都会把落点错开一个杆臂量, 比直接失败危险得多。

    口径 (controller_v1 / joint_offset_v2) 与装法正交, 但只眼在手上才是硬拦: 传进来的
    法兰位姿必须与标定时同源, 否则错的是一整个 Δq 在杆臂上的投影 (实测 14.44mm / 1.59°);
    眼在手外运行期不消费法兰位姿, 口径旧了只影响标定时被臂误差吸收掉的那一小部分 (实测
    ±2.84mm 量级), 因此照常给常量而把不匹配当作提醒返回。

    平移单位: 标定文件里是 mm, 本函数输出统一折算为 m (全项目唯一一处换算)。

    :param result_data: 标定 yaml 解析出的字典 (空/None 视为无结果)
    :param flange_pose_mm_deg: 拍摄时刻的**法兰**位姿 [x, y, z, rx, ry, rz] (mm, deg, Dobot 'xyz'
        内禀序列); 不能传 30004 的 tool_vector_actual (那是当前 tool 的 TCP, 与法兰差一个常量)
    :param runtime_frame: 当前运行时口径; 与 runtime_offsets_deg 必须成对传入, 传了才做口径判定
    :param runtime_offsets_deg: 当前配置的关节零位偏移 (deg, 6 轴)
    :return: (T_base_camera 4x4 列表或 None, mount, 英文原因或 None)
             T 为 None 时原因总能解释为什么; T 可用但带提醒时原因同样非 None (仅眼在手外)。
    """
    data = result_data or {}
    mount = resolve_result_mount(data)
    note = (pose_frame_mismatch(data, runtime_frame, runtime_offsets_deg or [])
            if runtime_frame is not None else None)

    if mount == EYE_IN_HAND:
        pose = list(flange_pose_mm_deg or [])
        if len(pose) < 6:
            return None, mount, (
                "Active calibration is eye-in-hand: the camera pose in the base frame is not "
                "a constant, so the flange pose at capture time is required to resolve it")
        if note:
            return None, mount, note
        T_flange_camera = data.get("T_flange_camera")
        if not T_flange_camera:
            return None, mount, "Calibration result is eye-in-hand but T_flange_camera is missing"
        T = pose_to_matrix(pose) @ np.array(T_flange_camera, dtype=np.float64)
        T[:3, 3] /= 1000.0
        return T.tolist(), mount, None

    key = ('T_base_camera' if 'T_base_camera' in data
           else ('T_camera_to_base' if 'T_camera_to_base' in data else None))
    if key is None:
        return None, mount, (
            "Eye-to-hand calibration result carries no T_base_camera extrinsic; "
            "re-run the calibration")
    T = np.array(data[key], dtype=np.float64)
    T[:3, 3] /= 1000.0
    return T.tolist(), mount, note


def infer_angle_unit(samples: Sequence[dict]) -> str:
    """
    为未记录 pose_angle_unit 的历史 session 猜测姿态单位。

    仅在读取旧数据时作为兜底: 欧拉角绝对值全部落在 (-2π, 2π) 内时按弧度处理,
    否则按度处理。新数据一律以 session yaml 里的 pose_angle_unit 为准。
    """
    values = []
    for s in samples:
        pose = s.get("robot_pose") or s.get("pose") or {}
        for pair in _ROT_KEYS:
            for key in pair:
                if key in pose and pose[key] is not None:
                    values.append(abs(float(pose[key])))
                    break
    if not values:
        return UNIT_DEG
    return UNIT_RAD if max(values) <= 2.0 * np.pi else UNIT_DEG


def normalize_pose(pose: PoseLike, angle_unit: str = UNIT_DEG) -> np.ndarray:
    """把任意位姿表示归一成 [x, y, z, rx, ry, rz]，平移 mm、姿态一律为度。

    兼容三世代数据: 新版 rx/ry/rz、旧版 a/b/c、以及裸 6 元列表。
    """
    if isinstance(pose, dict):
        xyz = [float(pose.get(k, 0.0)) for k in _POSE_KEYS]
        rpy = []
        for pair in _ROT_KEYS:
            for key in pair:
                if key in pose and pose[key] is not None:
                    rpy.append(float(pose[key]))
                    break
            else:
                rpy.append(0.0)
        arr = np.array(xyz + rpy, dtype=np.float64)
    else:
        arr = np.asarray(pose, dtype=np.float64).reshape(-1)
        if arr.size < 6:
            raise ValueError(f"pose needs at least 6 values [x,y,z,rx,ry,rz], got {arr.size}")
        arr = arr[:6].copy()

    if angle_unit == UNIT_RAD:
        arr[3:] = np.degrees(arr[3:])
    return arr


def rotation_from_pose(pose: PoseLike, angle_unit: str = UNIT_DEG) -> np.ndarray:
    """取位姿的 3x3 旋转 (Dobot 内禀 'xyz')。"""
    rpy = normalize_pose(pose, angle_unit)[3:]
    return Rot.from_euler(DOBOT_EULER_SEQ, rpy, degrees=True).as_matrix()


def pose_to_matrix(pose: PoseLike, angle_unit: str = UNIT_DEG) -> np.ndarray:
    """Dobot 位姿 -> 4x4 齐次矩阵 (mm, 度)。"""
    v = normalize_pose(pose, angle_unit)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rot.from_euler(DOBOT_EULER_SEQ, v[3:], degrees=True).as_matrix()
    T[:3, 3] = v[:3]
    return T


def matrix_to_pose(T: np.ndarray, angle_unit: str = UNIT_DEG) -> List[float]:
    """4x4 齐次矩阵 -> Dobot 位姿 [x, y, z, rx, ry, rz] (mm, 度或弧度)。"""
    T = np.asarray(T, dtype=np.float64)
    rpy = Rot.from_matrix(T[:3, :3]).as_euler(DOBOT_EULER_SEQ, degrees=True)
    if angle_unit == UNIT_RAD:
        rpy = np.radians(rpy)
    return [float(x) for x in T[:3, 3]] + [float(x) for x in rpy]


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """刚性逆变换, 避免调用 np.linalg.inv 带来的数值噪声。"""
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    Rt = T[:3, :3].T
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rt
    out[:3, 3] = -Rt @ T[:3, 3]
    return out


def project_points(obj_pts: np.ndarray, T_camera_obj: np.ndarray,
                   K: np.ndarray, D: Sequence[float] | None = None) -> np.ndarray:
    """把目标系下的三维点投影到像素, 用于真正的重投影误差。

    未标定畸变时 (D 为空或长度不足) 退化为针孔投影, 与 cv2.projectPoints 无畸变分支一致。
    """
    T = np.asarray(T_camera_obj, dtype=np.float64).reshape(4, 4)
    pts = np.asarray(obj_pts, dtype=np.float64).reshape(-1, 3)
    in_cam = T[:3, :3] @ pts.T + T[:3, 3:4]
    dist = np.asarray(D, dtype=np.float64).reshape(-1) if D is not None else np.zeros(5)
    if dist.size < 4:
        dist = np.pad(dist, (0, 5 - dist.size))
    out, _ = cv2.projectPoints(
        in_cam.T.astype(np.float64), np.zeros(3), np.zeros(3),
        np.asarray(K, dtype=np.float64).reshape(3, 3), dist,
    )
    return out.reshape(-1, 2)


def pixel_ray_to_base(u_px: float, v_px: float, K: Sequence[Sequence[float]],
                      T_base_camera, D: Optional[Sequence[float]] = None,
                      image_size: Optional[Sequence[int]] = None,
                      ) -> tuple[np.ndarray, np.ndarray]:
    """
    像素 (u, v) -> 相机光心出发的一条**基座系观测射线** (原点 mm, 单位方向)。

    这是“只知道方向、不知道深度”那一类问题的唯一正确表达: 单个像素反投影出来永远是一条
    射线, 深度必须由别的东西给出 (深度图 / 工件已知尺寸 / 沿射线滑到的那个可达姿态)。
    实时视频上点一下就要求机械臂指向该点, 吃的就是这条射线。

    单位口径: `T_base_camera` 的平移是**米** —— 即 `resolve_camera_extrinsic` 交付给运行时
    的原生单位 (全项目唯一一处 mm->m 就发生在那儿); 射线原点在这里乘回 1000 变成 mm,
    因为机械臂位姿与沿射线的距离在本项目一律 mm。方向是单位向量, 与单位无关。

    畸变: 给了有效畸变系数就先 `cv2.undistortPoints` 去畸变再反投影。本相机实测
    (k1=-0.032, k2=0.035, k3=-0.012, 1280x800): 角上像素被抬回 6.1px / 3.7px, 在 600mm
    深度上约合 6mm —— 对“指尖指准”是可见误差, 所以不去畸变是不够的。

    :param u_px, v_px: 像素坐标, 必须落在 [0, width-1] x [0, height-1] 内
    :param K: 3x3 内参矩阵 (像素单位)
    :param T_base_camera: 4x4 相机->基座变换 (平移 m)
    :param D: 畸变系数 (可空/短向量按零补齐)
    :param image_size: (width, height) 像素; 不传时只能按主点粗估边界 (中心对称假设)
    :return: (origin_mm[3], dir_unit[3]); 入参非法时抛 ValueError (英文)
    """
    Km = np.asarray(K, dtype=np.float64)
    T = np.asarray(T_base_camera, dtype=np.float64)
    if Km.shape != (3, 3) or T.shape != (4, 4):
        raise ValueError(f"pixel_ray_to_base needs K 3x3 and T_base_camera 4x4, got {Km.shape} and {T.shape}")
    # 带病的外参 (坏 YAML 读出 null/字符串、法兰位姿含 nan) 必须当场拦住: 它们会一路算成
    # nan 姿态, 而封包后的 nan 在控制器上表现为一个没人预期的动作, 比报错危险得多。
    if not (np.all(np.isfinite(Km)) and np.all(np.isfinite(T))):
        raise ValueError("Camera intrinsics and the camera->base transform must contain only finite numbers.")
    fx, fy = float(Km[0, 0]), float(Km[1, 1])
    cx, cy = float(Km[0, 2]), float(Km[1, 2])
    if not (fx > 0.0 and fy > 0.0):
        raise ValueError(f"Invalid camera intrinsics: fx/fy must be positive, got fx={fx}, fy={fy}")
    u = float(u_px)
    v = float(v_px)
    if not (np.isfinite(u) and np.isfinite(v)):
        raise ValueError(f"Pixel coordinates must be finite numbers, got ({u}, {v})")
    if image_size is not None and len(image_size) >= 2:
        width, height = float(image_size[0]), float(image_size[1])
        bound_note = f"{int(width)}x{int(height)} (from the intrinsics)"
    else:
        # 主点近似图像中心: 只在没有分辨率可考时当粗界用
        width, height = float(cx * 2.0), float(cy * 2.0)
        bound_note = f"~{width:.0f}x{height:.0f} (inferred from the principal point)"
    if not (0.0 <= u <= width - 1.0 and 0.0 <= v <= height - 1.0):
        raise ValueError(
            f"Pixel ({u:.0f}, {v:.0f}) is outside the image bounds {bound_note}")

    x_cam, y_cam, z_cam = (u - cx) / fx, (v - cy) / fy, 1.0
    dist = np.asarray(D, dtype=np.float64).reshape(-1) if D is not None else np.zeros(5)
    if dist.size and not np.all(np.isfinite(dist)):
        raise ValueError("Camera distortion coefficients must be finite numbers.")
    if dist.size < 4:
        dist = np.pad(dist, (0, 5 - dist.size))
    if np.any(np.abs(dist[:5]) > 1e-9):
        # cv2.undistortPoints 的 K/D 是位置参数 (绑定的形参名是 cameraMatrix/distCoeffs,
        # 当关键字传进去会直接报 Overload resolution failed), P=K 让输出回到像素域。
        ideal = cv2.undistortPoints(np.float64([[[u, v]]]), Km, dist[:5], P=Km)
        u_id, v_id = ideal.reshape(-1)[0], ideal.reshape(-1)[1]
        x_cam, y_cam = (u_id - cx) / fx, (v_id - cy) / fy

    dir_cam = np.array([x_cam, y_cam, z_cam], dtype=np.float64)
    dir_cam /= float(np.linalg.norm(dir_cam))
    origin_mm = T[:3, 3] * 1000.0
    dir_base = T[:3, :3] @ dir_cam
    dir_norm = float(np.linalg.norm(dir_base))
    if dir_norm < 1e-12:
        raise ValueError(
            "The camera->base rotation collapsed the observation ray to a zero vector; "
            "T_base_camera is not a valid rigid transform.")
    dir_base /= dir_norm
    return origin_mm, dir_base


def tool_frame_from_z_axis(z_axis: Sequence[float],
                           x_ref: Optional[Sequence[float]] = None) -> np.ndarray:
    """
    给定工具 +Z 轴在基座系的方向, 构造 3x3 工具姿态矩阵 (列 = x_tool, y_tool, z_tool)。

    工具 +Z 就是喷枪/指尖的指向 (与喷涂航点同一约定: 法向从表面指向相机, 工具 Z 取负法向,
    即指向工件内部), 所以“指尖当激光笔指向某点”与“航点垂直于表面”共用这一个构造器。

    绕 Z 的自旋由 x_ref 决定: 缺省用基座系 +Z 作参考, 与参考轴近平行 (|cos|>0.92) 时换 +X,
    否则叉积退化出一个不正交的姿态。连续构造时应把上一点的 x_tool (本函数返回矩阵的第一列)
    传进来, 否则逐点的 Rz 会在整条路径上跳 180°。

    :param z_axis: 工具 +Z 在基座系的方向向量 (不需事先归一)
    :param x_ref: 可选的自旋参考轴 (通常是上一点的 x_tool)
    :return: 3x3 旋转矩阵; 入参非法时抛 ValueError (英文)
    """
    z_tool = np.asarray(z_axis, dtype=np.float64).reshape(-1)
    if z_tool.size != 3 or not np.all(np.isfinite(z_tool)):
        raise ValueError(f"Tool Z axis needs 3 finite values, got {z_axis!r}")
    z_norm = float(np.linalg.norm(z_tool))
    if z_norm < 1e-9:
        raise ValueError("Tool Z axis is a zero vector; cannot build an orientation from it")
    z_tool = z_tool / z_norm

    if x_ref is None:
        ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        if abs(float(np.dot(z_tool, ref))) > 0.92:
            ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        ref = np.asarray(x_ref, dtype=np.float64).reshape(-1)
        if ref.size != 3:
            raise ValueError(f"x_ref needs 3 values, got {x_ref!r}")

    y_tool = np.cross(z_tool, ref)
    y_norm = float(np.linalg.norm(y_tool))
    if y_norm < 1e-9:
        y_tool = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    else:
        y_tool = y_tool / y_norm
    x_tool = np.cross(y_tool, z_tool)
    x_norm = float(np.linalg.norm(x_tool))
    if x_norm < 1e-9:
        x_tool = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        x_tool = x_tool / x_norm

    return np.column_stack((x_tool, y_tool, z_tool))


def euler_deg_from_rotation(R: np.ndarray) -> List[float]:
    """3x3 旋转矩阵 -> Dobot 姿态角 [rx, ry, rz] (度, 内禀 'xyz')。顺规只在 DOBOT_EULER_SEQ 那一处定义。"""
    euler = Rot.from_matrix(np.asarray(R, dtype=np.float64).reshape(3, 3)).as_euler(
        DOBOT_EULER_SEQ, degrees=True)
    return [float(x) for x in euler]


def tool_euler_deg_from_z_axis(z_axis: Sequence[float],
                               x_ref: Optional[Sequence[float]] = None) -> List[float]:
    """工具 +Z 指向 -> Dobot 姿态角 [rx, ry, rz] (度, 内禀 'xyz'); 见 tool_frame_from_z_axis。"""
    return euler_deg_from_rotation(tool_frame_from_z_axis(z_axis, x_ref=x_ref))


def rotation_closest_to_with_z_axis(z_axis: Sequence[float], R_ref: np.ndarray,
                                    twist_deg: float = 0.0) -> np.ndarray:
    """
    把一个旋转的 +Z 轴摆到 `z_axis`, 且让结果姿态离 `R_ref` **最近** (swing-only 最短弧)。

    与 `tool_frame_from_z_axis` 的分工: 后者用一根固定参考轴把绕 Z 的自旋一次定死 (适合
    沿轨迹逐点构造, 靠传上一点的 x_tool 保持连续); 本函数**保留 `R_ref` 自带的自旋**, 只做
    "把光轴摆过去" 这一件事。云台式指向 (只要求工具轴线经过目标点, 不约束指尖在哪) 要的正是
    它: 从当前姿态出发转过的角度最小 -> 腕部超限与奇异被控制器拒解的概率最低。

    算法: 转轴 n = z_now x z_target (同时垂直于两个轴 = 只摆头不拧腕的旋转方向), 转角
    theta = acos(z_now · z_target) (最短弧, 0..pi), 结果 R = Rot(n, theta) @ R_ref。
    反平行 (theta = pi) 时转轴在数学上不唯一, 借参考系的 X 轴叉 z_target 现取一个正交轴。

    :param z_axis: 期望的工具 +Z 在基座系的方向 (不需事先归一)
    :param R_ref: 参考 3x3 旋转 (通常是当前 TCP 姿态)
    :param twist_deg: 摆好之后再绕工具 +Z (光束轴) 自旋的角度 (度) —— 指向不变, 只换腕部时钟,
                      用于绕开腕关节限位/奇异
    :return: 3x3 旋转矩阵, 其第三列 = 归一后的 z_axis
    :raises ValueError: 入参非法 (英文)
    """
    ref = np.asarray(R_ref, dtype=np.float64)
    if ref.shape != (3, 3) or not np.all(np.isfinite(ref)):
        raise ValueError(f"R_ref needs a finite 3x3 rotation, got {R_ref!r}")
    z_target = tool_frame_from_z_axis(z_axis)[:, 2]          # 归一与合法性由构造器统一把关
    z_now = ref[:, 2]
    axis = np.cross(z_now, z_target)
    if float(np.linalg.norm(axis)) < 1e-9:
        if float(np.dot(z_now, z_target)) > 0.0:
            R = ref.copy()                                   # 已经指着同一个方向, 无需旋转
        else:
            spin = np.cross(ref[:, 0], z_target)
            if float(np.linalg.norm(spin)) < 1e-9:
                spin = np.cross(ref[:, 1], z_target)
            R = Rot.from_rotvec(float(np.pi) * spin / np.linalg.norm(spin)).as_matrix() @ ref
    else:
        theta = float(np.arccos(max(-1.0, min(1.0, float(np.dot(z_now, z_target))))))
        R = Rot.from_rotvec(theta * axis / float(np.linalg.norm(axis))).as_matrix() @ ref
    if twist_deg:
        R = Rot.from_rotvec(float(np.radians(twist_deg)) * z_target).as_matrix() @ R
    return R


def rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """两个旋转矩阵之间的角位移 (度)。"""
    R_diff = np.asarray(R_a, dtype=np.float64).T @ np.asarray(R_b, dtype=np.float64)
    trace_val = (np.trace(R_diff) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(trace_val, -1.0, 1.0))))


def average_rotation(R_list: Sequence[np.ndarray]) -> np.ndarray:
    """旋转矩阵的均值并重投影回 SO(3) (SVD 正交化, 去漂移)。"""
    avg = np.mean(np.asarray(R_list, dtype=np.float64), axis=0)
    U, _, Vt = np.linalg.svd(avg)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def chessboard_object_points(pattern_size: Sequence[int], square_size_mm: float) -> np.ndarray:
    """按 OpenCV 惯例生成棋盘格三维模型点 (Z=0 平面, 单位 mm)。"""
    cols, rows = int(pattern_size[0]), int(pattern_size[1])
    objp = np.zeros((rows * cols, 3), dtype=np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return objp * float(square_size_mm)


def rotation_axis_coverage(R_list: Sequence[np.ndarray]) -> float:
    """
    旋转轴方向覆盖度, 用来诊断 AX=XB 是否退化。

    AX=XB 的可观测性来自旋转轴方向的变化: 若所有样本绕同一轴旋转, X 沿该轴的分量
    完全不可观测, 求解器照样会返回一个"收敛"的错误解。这里取每个样本相对首样本的
    旋转轴, 构造它们两两之间夹角的倒数均值: 数值越接近 0 表示轴越分散, 越可靠。
    返回值是覆盖度 (0~1, 越大越好)。
    """
    axes = []
    R0 = np.asarray(R_list[0], dtype=np.float64)
    for R in R_list[1:]:
        rotvec = Rot.from_matrix(R0.T @ np.asarray(R, dtype=np.float64)).as_rotvec()
        norm = float(np.linalg.norm(rotvec))
        if norm < np.radians(5.0):
            continue
        axes.append(rotvec / norm)

    if len(axes) < 2:
        return 0.0

    sin_angles = []
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            sin_angles.append(float(np.linalg.norm(np.cross(axes[i], axes[j]))))
    if not sin_angles:
        return 0.0
    return float(np.mean(sin_angles))
