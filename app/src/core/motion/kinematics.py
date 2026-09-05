"""CR5 FK/IK 的唯一 Python 入口：ctypes → libmotion_c.so。

follow / 交互页 / 其它 Python 调用方都走这里，不再加载 cr5_kinematics_cpp。
公共方法与旧 CR5Kinematics 对齐（forward_controller / get_best_ik / …）。
"""

from __future__ import annotations

import ctypes
import math
import os
from typing import Optional, Sequence

import numpy as np

PI = math.pi

_LIB = None

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
    """

    def __init__(
        self,
        joint_min: Sequence[float] | None = None,
        joint_max: Sequence[float] | None = None,
    ):
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

    def _sols(self, n: int) -> list[np.ndarray]:
        if n <= 0:
            return []
        raw = np.ctypeslib.as_array(self._q_sols_buf).reshape(8, 6)[:n]
        return [raw[i].copy() for i in range(n)]

    def forward(self, q_urdf: Sequence[float]) -> np.ndarray:
        _fill(self._q_buf, q_urdf)
        self._lib.c_forward(self._q_buf, self._T_buf)
        return np.array(self._T_buf, dtype=np.float64).reshape(4, 4)

    def inverse(self, T: np.ndarray, q6_des: float = 0.0) -> list[np.ndarray]:
        del q6_des
        _fill(self._T_buf, np.asarray(T, dtype=np.float64).reshape(-1))
        n = int(self._lib.c_inverse(self._T_buf, self._q_sols_buf))
        return self._sols(n)

    def forward_controller(self, q_urdf: Sequence[float]) -> tuple[list[float], list[float]]:
        _fill(self._q_buf, q_urdf)
        self._lib.c_forward_controller(self._q_buf, self._xyz_buf, self._rpy_buf)
        return [self._xyz_buf[0], self._xyz_buf[1], self._xyz_buf[2]], [
            self._rpy_buf[0],
            self._rpy_buf[1],
            self._rpy_buf[2],
        ]

    def inverse_controller(self, xyz_mm: Sequence[float], rpy_deg: Sequence[float]) -> list[np.ndarray]:
        _fill(self._xyz_buf, xyz_mm)
        _fill(self._rpy_buf, rpy_deg)
        n = int(self._lib.c_inverse_controller(self._xyz_buf, self._rpy_buf, self._q_sols_buf))
        return self._sols(n)

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
        curr = np.asarray(current_joints, dtype=np.float64)
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
        return np.array(self._q_out_buf, dtype=np.float64)
