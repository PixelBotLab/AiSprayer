"""
C++ Wrapped CR5 / UR-style Analytical Forward and Inverse Kinematics Core.
"""

import math
import numpy as np
import ctypes
import os

_lib_path = os.path.join(os.path.dirname(__file__), "libur_kin.so")
if not os.path.exists(_lib_path):
    raise RuntimeError(f"Missing shared library: {_lib_path}")

_lib = ctypes.CDLL(_lib_path)

_lib.c_ur_forward.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
_lib.c_ur_forward.restype = None

_lib.c_ur_inverse.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_double]
_lib.c_ur_inverse.restype = ctypes.c_int

def forward(q: list[float] | np.ndarray) -> np.ndarray:
    q_arr = (ctypes.c_double * 6)(*q)
    T_arr = (ctypes.c_double * 16)()
    _lib.c_ur_forward(q_arr, T_arr)
    return np.array(T_arr, dtype=np.float64).reshape((4, 4))

def inverse(T: np.ndarray, q6_des: float = 0.0) -> list[list[float]]:
    T_arr = (ctypes.c_double * 16)(*T.flatten())
    q_sols_arr = (ctypes.c_double * 48)()
    num_sols = _lib.c_ur_inverse(T_arr, q_sols_arr, q6_des)
    
    sols = []
    for i in range(num_sols):
        sols.append([q_sols_arr[i*6 + j] for j in range(6)])
    return sols
