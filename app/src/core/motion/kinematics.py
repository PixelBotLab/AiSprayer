"""CR5 FK/IK 的唯一 Python 入口：ctypes → libmotion_c.so。

follow / 交互页 / 其它 Python 调用方都走这里，不再加载 cr5_kinematics_cpp。
公共方法与旧 CR5Kinematics 对齐（forward_controller / get_best_ik / …）。
"""

from __future__ import annotations

import ctypes
import logging
import math
import os
import threading
from typing import Optional, Sequence

import numpy as np

logger = logging.getLogger(__name__)

PI = math.pi

_LIB = None

# 关节零位偏移的物理合理上限 (deg)。真实的编码器零位偏差量级在度以内 (本项目实测
# 最大 1.96°); 超出这个范围的输入只可能是配错、单位错或手抖多打了一个零, 必须快速
# 失败而不是带病运行 —— 偏移会进 FK 与 IK 的每一条位姿, 错了没有任何下游能发现。
MAX_JOINT_OFFSET_DEG = 5.0


def validate_joint_offsets(values: Optional[Sequence[float]]) -> np.ndarray:
    """
    校验关节零位偏移 (deg), 返回 6 维 float64 数组; None/全空 代表不补偿 (控制器口径)。

    偏移绑定具体机器 (换机/大修/撞过就要重解), 所以它是配置项而不是代码常量;
    任何形状错、非有限值、超上限的输入都直接报错, 错误信息里点出配置键便于定位。
    """
    if values is None or (hasattr(values, "__len__") and len(values) == 0):
        return np.zeros(6, dtype=np.float64)
    arr = np.asarray(list(values), dtype=np.float64).ravel()
    if arr.size != 6:
        raise ValueError(f"hardware.robot.joint_offsets_deg needs 6 values, got {arr.size}")
    if not np.all(np.isfinite(arr)):
        raise ValueError("hardware.robot.joint_offsets_deg contains non-finite values")
    if np.any(np.abs(arr) > MAX_JOINT_OFFSET_DEG):
        raise ValueError(
            f"hardware.robot.joint_offsets_deg exceeds the physical limit "
            f"±{MAX_JOINT_OFFSET_DEG} deg: {arr.tolist()} — check the unit (deg, not rad) "
            f"and re-identify the offsets instead of widening this bound")
    return arr


def find_motion_c_lib() -> Optional[str]:
    override = os.environ.get("MOTION_C_LIB")
    if override and os.path.isfile(override):
        return override
    here = os.path.dirname(os.path.abspath(__file__))
    install_lib = os.path.abspath(os.path.join(here, "../../../../lib"))
    for folder in (os.path.join(here, "bin"), install_lib, os.path.join(here, "build"), here):
        for name in ("libmotion_c.so", "libmotion_c.dylib", "motion_c.dll"):
            path = os.path.join(folder, name)
            if os.path.isfile(path):
                return path
    return None


def load_motion_c_lib():
    global _LIB
    if _LIB is not None:
        return _LIB
    path = find_motion_c_lib()
    if path is None:
        raise RuntimeError(
            "libmotion_c.so 不存在。请先运行 app/src/core/motion/scripts/build.sh"
        )
    lib = ctypes.CDLL(path)
    dbl_p = ctypes.POINTER(ctypes.c_double)
    lib.c_forward.argtypes = [dbl_p, dbl_p]
    lib.c_forward.restype = None
    lib.c_inverse.argtypes = [dbl_p, dbl_p]
    lib.c_inverse.restype = ctypes.c_int
    lib.c_forward_controller.argtypes = [dbl_p, dbl_p, dbl_p]
    lib.c_forward_controller.restype = None
    lib.c_inverse_controller.argtypes = [dbl_p, dbl_p, dbl_p]
    lib.c_inverse_controller.restype = ctypes.c_int
    lib.c_get_best_ik.argtypes = [dbl_p, dbl_p, dbl_p, dbl_p, dbl_p, dbl_p]
    lib.c_get_best_ik.restype = ctypes.c_int
    _LIB = lib
    return lib


def _fill(buf, values) -> None:
    src = np.ascontiguousarray(values, dtype=np.float64).ravel()
    if src.size != len(buf):
        raise ValueError(f"expected {len(buf)} values, got {src.size}")
    ctypes.memmove(buf, src.ctypes.data, src.nbytes)


class CR5Kinematics:
    """Dobot CR5 运动学。数值全部来自 libmotion_c.so。

    ctypes 缓冲是 per-instance 的，多线程共用同一实例时必须外加锁
    （follow_service 已有 `_kin_lock`）。

    关节零位偏移 (Δq) 的约定 —— 本类的**入参一律是编码器上报值, 出参一律是可直接下发的
    上报值**, 校正封在内部:
      FK(q_reported) = lib.forward(q_reported + Δq)      -> 真实位姿
      IK(T_true)     = lib.inverse(T_true) - Δq          -> 可下发的关节目标
    这样正反两个方向永远自洽 (FK∘IK = 恒等), 调用方无需知道 Δq 存在, 也就不可能在
    一处补偿、另一处忘补。不传偏移时 Δq=0, 行为与改造前逐位相同 (控制器口径)。
    """

    def __init__(
        self,
        joint_min: Sequence[float] | None = None,
        joint_max: Sequence[float] | None = None,
        joint_offsets_deg: Sequence[float] | None = None,
    ):
        self.joint_offsets_deg = validate_joint_offsets(joint_offsets_deg)
        self.joint_offsets_rad = np.radians(self.joint_offsets_deg)
        self.joint_min = np.array(
            joint_min
            if joint_min is not None
            else [-2.0 * PI, -PI, -2.86159, -PI, -PI, -2.0 * PI],
            dtype=np.float64,
        )
        self.joint_max = np.array(
            joint_max
            if joint_max is not None
            else [2.0 * PI, PI, 2.86159, PI, PI, 2.0 * PI],
            dtype=np.float64,
        )
        self._lib = load_motion_c_lib()
        self._q_buf = (ctypes.c_double * 6)()
        self._T_buf = (ctypes.c_double * 16)()
        self._q_sols_buf = (ctypes.c_double * 48)()
        self._xyz_buf = (ctypes.c_double * 3)()
        self._rpy_buf = (ctypes.c_double * 3)()
        self._q_curr_buf = (ctypes.c_double * 6)()
        self._q_out_buf = (ctypes.c_double * 6)()
        self._jmin_buf = (ctypes.c_double * 6)()
        self._jmax_buf = (ctypes.c_double * 6)()
        self._w_buf = (ctypes.c_double * 6)()

    @property
    def has_joint_offsets(self) -> bool:
        """本实例是否带非零关节零位偏移 (即是否运行在校正口径)。"""
        return bool(np.any(np.abs(self.joint_offsets_deg) > 1e-9))

    def matches_joint_offsets(self, values: Optional[Sequence[float]]) -> bool:
        """
        判断一组偏移 (deg) 与本实例构造时用的是否同一组 —— 调用方缓存了实例, 配置改了必须重建。

        偏移不一致时 FK/IK 会差 Δq 在杆臂上的投影 (实测 17mm / 3°), 是静默错位, 只能重建。
        共用本方法而不是各处重写比较, 避免容差在两处漂。
        """
        try:
            arr = validate_joint_offsets(values)
        except ValueError:
            return False
        return bool(np.array_equal(arr, self.joint_offsets_deg))

    def _sols(self, n: int) -> list[np.ndarray]:
        if n <= 0:
            return []
        raw = np.ctypeslib.as_array(self._q_sols_buf).reshape(8, 6)[:n]
        return [raw[i].copy() for i in range(n)]

    def forward(self, q_reported: Sequence[float]) -> np.ndarray:
        """上报关节角 (rad) -> 真实位姿矩阵 (**URDF 帧, 平移 m**); 带 Δq 的实例输出已补偿。"""
        _fill(self._q_buf, np.asarray(q_reported, dtype=np.float64).ravel() + self.joint_offsets_rad)
        self._lib.c_forward(self._q_buf, self._T_buf)
        return np.array(self._T_buf, dtype=np.float64).reshape(4, 4)

    def inverse(self, T: np.ndarray, q6_des: float = 0.0) -> list[np.ndarray]:
        """真实位姿矩阵 (**URDF 帧, 平移 m**) -> 可直接下发的上报关节角 (rad); 与 forward 互逆。"""
        del q6_des
        _fill(self._T_buf, np.asarray(T, dtype=np.float64).reshape(-1))
        n = int(self._lib.c_inverse(self._T_buf, self._q_sols_buf))
        return [sol - self.joint_offsets_rad for sol in self._sols(n)]

    def forward_controller(self, q_reported: Sequence[float]) -> tuple[list[float], list[float]]:
        """上报关节角 (rad) -> 控制器口径法兰位姿 (mm, deg); 带 Δq 的实例输出已补偿。"""
        _fill(self._q_buf, np.asarray(q_reported, dtype=np.float64).ravel() + self.joint_offsets_rad)
        self._lib.c_forward_controller(self._q_buf, self._xyz_buf, self._rpy_buf)
        return [self._xyz_buf[0], self._xyz_buf[1], self._xyz_buf[2]], [
            self._rpy_buf[0],
            self._rpy_buf[1],
            self._rpy_buf[2],
        ]

    def flange_pose_controller_deg(self, joints_deg: Sequence[float]) -> list[float]:
        """
        关节反馈 (度) -> **法兰**位姿 [x, y, z, rx, ry, rz] (mm, deg), 控制器的坐标约定。

        与 30004 的 tool_vector_actual 是同一套坐标约定, 差别仅在于后者叠加了示教器
        当前选中的 tool 偏置 (TCP 不在法兰面上时差一个常量刚体变换)。手眼标定与运行时
        复合需要的是法兰本身, 用它可让外参不随 tool 号切换而失效。

        实测口径: 两个标定会话共 51 帧, 本方法与 tool_vector_actual 之比恒等于同一个
        常量变换 (逐帧偏离 max 0.2um / 0.0000deg), 说明两套运动学模型完全一致。

        带上 joint_offsets_deg 后该结论不再成立: 本方法返回 FK(q_encoder + Δq) 即**校正后**
        法兰位姿, 与控制器自己的笛卡尔读数差 Δq 在杆臂上的投影 (本项目实测约 17mm / 3°)。
        反过来说: 关节空间是唯一可校正的通道, 笛卡尔读数上的误差在笛卡尔域不可逆。
        """
        q = np.radians(np.asarray(list(joints_deg)[:6], dtype=np.float64))
        xyz, rpy = self.forward_controller(q.tolist())
        return list(xyz) + list(rpy)

    def inverse_controller(self, xyz_mm: Sequence[float], rpy_deg: Sequence[float]) -> list[np.ndarray]:
        """控制器**约定**下的校正位姿 (mm, deg) -> 可下发的上报关节角 (rad); 与 forward_controller 互逆。

        注意吃的是带 Δq 的位姿 (与 forward_controller 的输出口径一致), 不是 30004 端口的原始
        笛卡尔读数 —— 后者是控制器自己算的, 不含我们的零位补偿。
        """
        _fill(self._xyz_buf, xyz_mm)
        _fill(self._rpy_buf, rpy_deg)
        n = int(self._lib.c_inverse_controller(self._xyz_buf, self._rpy_buf, self._q_sols_buf))
        return [sol - self.joint_offsets_rad for sol in self._sols(n)]

    def controller_matrix_to_urdf(self, T_ctrl: np.ndarray) -> np.ndarray:
        R = np.asarray(T_ctrl, dtype=np.float64)[:3, :3]
        p = np.asarray(T_ctrl, dtype=np.float64)[:3, 3]
        T = np.eye(4, dtype=np.float64)
        T[0, 0] = -R[0, 2]
        T[0, 1] = R[0, 0]
        T[0, 2] = R[0, 1]
        T[0, 3] = -p[0]
        T[1, 0] = -R[1, 2]
        T[1, 1] = R[1, 0]
        T[1, 2] = R[1, 1]
        T[1, 3] = -p[1]
        T[2, 0] = R[2, 2]
        T[2, 1] = -R[2, 0]
        T[2, 2] = -R[2, 1]
        T[2, 3] = p[2]
        return T

    def is_joint_valid(self, q: Sequence[float], tolerance: float = 1e-4) -> bool:
        q_arr = np.asarray(q, dtype=np.float64)
        return bool(
            np.all(q_arr >= self.joint_min - tolerance) and np.all(q_arr <= self.joint_max + tolerance)
        )

    def get_best_ik(
        self,
        T: np.ndarray,
        current_joints: Sequence[float],
        weights: Sequence[float] | None = None,
    ) -> np.ndarray | None:
        """位姿 (**URDF 帧, 平移 m**) -> 离当前构型最近分支的上报关节角 (rad)。入出参都在上报口径。

        C 内核搜的是模型/真实关节, 所以 current 必须先加上 Δq 再送进去, 输出再减回来;
        否则在有偏移的实例上会错一圈 (偏移量级的分支误选)。
        """
        curr = np.asarray(current_joints, dtype=np.float64) + self.joint_offsets_rad
        w = np.asarray(weights if weights is not None else [1.0] * 6, dtype=np.float64)
        _fill(self._T_buf, np.asarray(T, dtype=np.float64).reshape(-1))
        _fill(self._q_curr_buf, curr)
        _fill(self._jmin_buf, self.joint_min)
        _fill(self._jmax_buf, self.joint_max)
        _fill(self._w_buf, w)
        ok = int(
            self._lib.c_get_best_ik(
                self._T_buf,
                self._q_curr_buf,
                self._jmin_buf,
                self._jmax_buf,
                self._w_buf,
                self._q_out_buf,
            )
        )
        if not ok:
            return None
        return np.array(self._q_out_buf, dtype=np.float64) - self.joint_offsets_rad


def kinematics_from_config(use_joint_offsets: bool = True) -> CR5Kinematics:
    """
    按运行时配置构造运动学实例 (全项目唯一的 CR5Kinematics 装配入口)。

    关节零位偏移 Δq 只在这一个地方从配置读出并注入: follow / 交互页 / 标定服务共用同一条
    口径, 不可能出现"一处补偿、另一处忘补"。偏移改动后必须重建实例 (调用方各自缓存)。

    :param use_joint_offsets: False 时强制走控制器口径 (Δq=0)。给那些**本来就要和控制器
        原始读数对齐**的调用方用 (如 POI 锚点 —— 其比较对象是 30004 的笛卡尔回报), 必须
        在调用处注释清楚为何不补偿, 否则一律用默认值。
    :return: CR5Kinematics 实例
    """
    from core.config import SprayerConfig
    cfg = SprayerConfig()
    return CR5Kinematics(joint_offsets_deg=(cfg.robot_joint_offsets_deg
                                           if use_joint_offsets else None))


# 现场反推法兰位姿用的内核实例: 它绑定 Δq, 配置一改就必须重建 (否则整批样本吃旧口径的
# FK); ctypes 缓冲是 per-instance 的, 线程间复用必须外加锁。放在模块级是为了让标定与
# 交互采集走同一个实例、同一套失效规则, 不再各抄一份缓存判断。
_fk_kernel: Optional[CR5Kinematics] = None
_fk_lock = threading.Lock()


def flange_pose_from_joints(joints_deg: Optional[Sequence[float]]) -> Optional[list[float]]:
    """
    关节反馈 (度) -> **校正后**法兰位姿 [x, y, z, rx, ry, rz] (mm, deg); 不可用时 None。

    这是手眼标定与运行时复合相机位姿共用的唯一法兰来源。运动学内核只算**法兰**, 而 30004
    的 tool_vector_actual 报的是示教器当前 tool 的 TCP: 两者差一个常量 tool 变换 (本项目
    实测 252.7mm / 175.1°)。拿 TCP 读数做手眼求解或运行时复合, 该常量会被外参 X 吸收成
    “相对工具尖”的安装量: 残差与重投影误差一分不变 (旧指标看不出来), 但发布的
    T_flange_camera 名不副实, 换 tool 号静默失效。用 FK(joints) 则与 tool/user 号无关,
    也让工作距离等派生量回到真实量级。

    关节零位偏移 Δq 在这里生效 (FK(q+Δq)): 它是唯一可校正的通道 —— 笛卡尔读数上的误差在
    笛卡尔域不可逆 (实测常量折算会把残差从 9.2px 推到 11.4px)。

    非法偏移 (超 ±5° 等) 在这里被内核拒收, 对外只表现为“法兰位姿不可用”, 且绝不把病实例
    缓存下来 —— 下一个合法配置必须还能正常反推。
    """
    if joints_deg is None or len(list(joints_deg)) < 6:
        return None
    global _fk_kernel
    with _fk_lock:
        try:
            from core.config import SprayerConfig
            offsets = SprayerConfig().robot_joint_offsets_deg
            if _fk_kernel is None or not _fk_kernel.matches_joint_offsets(offsets):
                _fk_kernel = kinematics_from_config()
            return [float(v) for v in _fk_kernel.flange_pose_controller_deg(joints_deg)]
        except Exception as e:
            _fk_kernel = None      # 构造失败原因不明, 丢掉引用重建, 不吃可能带病的缓存
            logger.warning(f"Forward kinematics from joint feedback unavailable: {e}")
            return None
