"""
CR5 Kinematics Solver and Dobot Controller Adapter.

Analytical FK/IK for Dobot CR5. Numerical backends:

- `CR5PythonKinematics` — `cr5_ur_kin.py`
- `CR5CppKinematics` — ctypes → libur_kin (`c_forward` / `c_inverse` / …)

`CR5Kinematics` picks one of those classes and keeps shared logic (branch
selection, ±2π expand, singularity) on top.

Coordinate frames
-----------------
- URDF / standard APIs (`forward` / `inverse`): Base → Link6, joint angles in URDF
  convention. Internally DH q2/q4 are offset by ±π/2 relative to URDF.
- Controller APIs (`forward_controller` / `inverse_controller`): Dobot teach-pendant
  frame. Base is RotZ(180°); tool axes are permuted (X_t=-Y_u, Y_t=-Z_u, Z_t=X_u).
  Position is mm, Euler ZYX in degrees (rx, ry, rz).

IK branches
-----------
A reachable pose has up to 8 closed-form solutions = 2(shoulder/q1) × 2(wrist/q5)
× 2(elbow/q3). Trajectory code must pick the nearest branch (`get_best_ik`), not sols[0].
"""

from __future__ import annotations

import ctypes
import math
import os
from abc import ABC, abstractmethod

import numpy as np

from . import cr5_ur_kin

PI = math.pi

# Wrist/elbow: |sin(q)| below sin(3°) ≈ collinear axes. Shoulder: two q1 solutions
# closer than 2×3° means the wrist is on the |d4| cylinder about J1.
_SING_DEG = 3.0
_SING_SIN = math.sin(math.radians(_SING_DEG))
_SHOULDER_HALF_RAD = math.radians(_SING_DEG)

_CPP_LIB = None

# Controller ↔ URDF: RotZ(180) on base, then tool axis permutation.
_T_BASE = np.array([
    [-1.0,  0.0, 0.0, 0.0],
    [ 0.0, -1.0, 0.0, 0.0],
    [ 0.0,  0.0, 1.0, 0.0],
    [ 0.0,  0.0, 0.0, 1.0],
], dtype=np.float64)
_T_TOOL = np.array([
    [ 0.0,  0.0,  1.0, 0.0],
    [-1.0,  0.0,  0.0, 0.0],
    [ 0.0, -1.0,  0.0, 0.0],
    [ 0.0,  0.0,  0.0, 1.0],
], dtype=np.float64)


def find_cr5_cpp_lib() -> str | None:
    """Locate libur_kin next to the C++ sources or in the CMake build dir."""
    base = os.path.join(os.path.dirname(__file__), "cr5_kinematics_cpp")
    names = ("libur_kin.dylib", "libur_kin.so", "ur_kin.dll")
    for folder in (os.path.join(base, "build"), base):
        for name in names:
            path = os.path.join(folder, name)
            if os.path.exists(path):
                return path
    return None


def load_cr5_cpp_lib():
    """Load libur_kin once and bind the six high-level C ABI functions."""
    global _CPP_LIB
    if _CPP_LIB is not None:
        return _CPP_LIB

    path = find_cr5_cpp_lib()
    if path is None:
        raise RuntimeError(
            "libur_kin not found. Build it with: "
            "cmake -S app/src/core/hardware/robot/cr5_kinematics_cpp -B "
            "app/src/core/hardware/robot/cr5_kinematics_cpp/build "
            "-DCMAKE_BUILD_TYPE=Release && cmake --build "
            "app/src/core/hardware/robot/cr5_kinematics_cpp/build"
        )

    lib = ctypes.CDLL(path)
    dbl_p = ctypes.POINTER(ctypes.c_double)
    lib.c_forward.argtypes = [dbl_p, dbl_p]
    lib.c_forward.restype = None
    lib.c_inverse.argtypes = [dbl_p, dbl_p]
    lib.c_inverse.restype = ctypes.c_int
    lib.c_compute_fk.argtypes = [dbl_p, dbl_p, dbl_p]
    lib.c_compute_fk.restype = None
    lib.c_compute_ik.argtypes = [dbl_p, dbl_p, dbl_p]
    lib.c_compute_ik.restype = ctypes.c_int
    lib.c_forward_controller.argtypes = [dbl_p, dbl_p, dbl_p]
    lib.c_forward_controller.restype = None
    lib.c_inverse_controller.argtypes = [dbl_p, dbl_p, dbl_p]
    lib.c_inverse_controller.restype = ctypes.c_int
    _CPP_LIB = lib
    return lib


def _resolve_backend(backend: str) -> str:
    b = (backend or "python").strip().lower()
    if b not in ("python", "py", "cpp", "c++", "auto"):
        raise ValueError(f"Unknown kinematics backend '{backend}', expected python|cpp|auto")
    if b in ("python", "py"):
        return "python"
    if b in ("cpp", "c++"):
        return "cpp"
    return "cpp" if find_cr5_cpp_lib() else "python"


def _urdf_to_dh(q_urdf: list[float] | np.ndarray) -> np.ndarray:
    q_dh = np.array(q_urdf, dtype=np.float64)
    q_dh[1] -= PI / 2.0
    q_dh[3] -= PI / 2.0
    return q_dh


def _dh_sols_to_urdf(raw_sols) -> list[np.ndarray]:
    if not raw_sols:
        return []
    q_u = np.array(raw_sols, dtype=np.float64)
    q_u[:, 1] += PI / 2.0
    q_u[:, 3] += PI / 2.0
    q_u = np.mod(q_u + PI, 2.0 * PI) - PI
    return [q_u[i] for i in range(q_u.shape[0])]


class CR5KinematicsBackend(ABC):
    """Numerical FK/IK only. Branch selection lives on CR5Kinematics."""

    name: str

    @abstractmethod
    def forward(self, q_urdf: list[float] | np.ndarray) -> np.ndarray: ...

    @abstractmethod
    def inverse(self, T: np.ndarray, q6_des: float = 0.0) -> list[np.ndarray]: ...

    @abstractmethod
    def compute_fk(self, q_urdf: list[float] | np.ndarray) -> tuple[np.ndarray, np.ndarray]: ...

    @abstractmethod
    def compute_ik(self, translation: np.ndarray, rotation: np.ndarray) -> list[np.ndarray]: ...

    @abstractmethod
    def forward_controller(
        self, q_urdf: list[float] | np.ndarray
    ) -> tuple[list[float], list[float]]: ...

    @abstractmethod
    def inverse_controller(self, xyz_mm: list[float], rpy_deg: list[float]) -> list[np.ndarray]: ...

    def forward_all(self, q_urdf: list[float] | np.ndarray) -> list[np.ndarray]:
        """Link frames T1–T6. Not in the C ABI; Python stub (identities) until implemented."""
        return cr5_ur_kin.forward_all(_urdf_to_dh(q_urdf))


class CR5PythonKinematics(CR5KinematicsBackend):
    """URDF/controller FK/IK via cr5_ur_kin.py (DH offsets applied here)."""

    name = "python"

    def __init__(self):
        self.T_base = _T_BASE
        self.T_tool = _T_TOOL
        self._T_work = np.eye(4, dtype=np.float64)

    def forward(self, q_urdf: list[float] | np.ndarray) -> np.ndarray:
        return cr5_ur_kin.forward(_urdf_to_dh(q_urdf))

    def inverse(self, T: np.ndarray, q6_des: float = 0.0) -> list[np.ndarray]:
        return _dh_sols_to_urdf(cr5_ur_kin.inverse(T, q6_des=q6_des))

    def compute_fk(self, q_urdf: list[float] | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        T = self.forward(q_urdf)
        return T[:3, 3], T[:3, :3]

    def compute_ik(self, translation: np.ndarray, rotation: np.ndarray) -> list[np.ndarray]:
        T = self._T_work
        T[:3, :3] = rotation
        T[:3, 3] = translation
        return self.inverse(T)

    def forward_controller(self, q_urdf: list[float] | np.ndarray) -> tuple[list[float], list[float]]:
        T_ctrl = self.T_base @ self.forward(q_urdf) @ self.T_tool
        xyz_mm = [float(T_ctrl[0, 3] * 1000.0),
                  float(T_ctrl[1, 3] * 1000.0),
                  float(T_ctrl[2, 3] * 1000.0)]
        R_ctrl = T_ctrl[:3, :3]
        # ZYX: R = Rz(rz) Ry(ry) Rx(rx). Matches C++ atan2 decomposition.
        beta = math.atan2(-R_ctrl[2, 0], math.sqrt(R_ctrl[0, 0] ** 2 + R_ctrl[1, 0] ** 2))
        if abs(math.cos(beta)) > 1e-6:
            alpha = math.atan2(R_ctrl[1, 0] / math.cos(beta), R_ctrl[0, 0] / math.cos(beta))
            gamma = math.atan2(R_ctrl[2, 1] / math.cos(beta), R_ctrl[2, 2] / math.cos(beta))
        else:
            alpha = 0.0
            gamma = math.atan2(R_ctrl[0, 1], R_ctrl[1, 1])
        return xyz_mm, [float(math.degrees(gamma)),
                        float(math.degrees(beta)),
                        float(math.degrees(alpha))]

    def inverse_controller(self, xyz_mm: list[float], rpy_deg: list[float]) -> list[np.ndarray]:
        rx = math.radians(rpy_deg[0])
        ry = math.radians(rpy_deg[1])
        rz = math.radians(rpy_deg[2])
        sx, cx = math.sin(rx), math.cos(rx)
        sy, cy = math.sin(ry), math.cos(ry)
        sz, cz = math.sin(rz), math.cos(rz)
        # R_ctrl = Rz Ry Rx, then T_urdf = T_base_inv * T_ctrl * T_tool_inv (index form).
        r00 = cz * cy
        r01 = cz * sy * sx - sz * cx
        r02 = cz * sy * cx + sz * sx
        r10 = sz * cy
        r11 = sz * sy * sx + cz * cx
        r12 = sz * sy * cx - cz * sx
        r20 = -sy
        r21 = cy * sx
        r22 = cy * cx
        x, y, z = xyz_mm[0] / 1000.0, xyz_mm[1] / 1000.0, xyz_mm[2] / 1000.0
        T = self._T_work
        T[0, 0] = -r02; T[0, 1] = r00; T[0, 2] = r01; T[0, 3] = -x
        T[1, 0] = -r12; T[1, 1] = r10; T[1, 2] = r11; T[1, 3] = -y
        T[2, 0] = r22; T[2, 1] = -r20; T[2, 2] = -r21; T[2, 3] = z
        return self.inverse(T)


class CR5CppKinematics(CR5KinematicsBackend):
    """URDF/controller FK/IK via ctypes → libur_kin (c_forward / c_inverse / …)."""

    name = "cpp"

    def __init__(self):
        self._lib = load_cr5_cpp_lib()
        # Per-instance ctypes buffers (not thread-safe; one solver per thread).
        self._q_buf = (ctypes.c_double * 6)()
        self._T_buf = (ctypes.c_double * 16)()
        self._q_sols_buf = (ctypes.c_double * 48)()
        self._xyz_buf = (ctypes.c_double * 3)()
        self._rpy_buf = (ctypes.c_double * 3)()
        self._ee_t_buf = (ctypes.c_double * 3)()
        self._ee_r_buf = (ctypes.c_double * 9)()
        # c_inverse has no q6_des; non-zero hint uses the Python analytical path.
        self._python = CR5PythonKinematics()

    @staticmethod
    def _fill_buf(buf, values) -> None:
        src = np.ascontiguousarray(values, dtype=np.float64).ravel()
        if src.size != len(buf):
            raise ValueError(f"expected {len(buf)} values, got {src.size}")
        ctypes.memmove(buf, src.ctypes.data, src.nbytes)

    def _sols_from_buf(self, n: int) -> list[np.ndarray]:
        if n <= 0:
            return []
        raw = np.ctypeslib.as_array(self._q_sols_buf).reshape(8, 6)[:n]
        return [raw[i].copy() for i in range(n)]

    def forward(self, q_urdf: list[float] | np.ndarray) -> np.ndarray:
        self._fill_buf(self._q_buf, q_urdf)
        self._lib.c_forward(self._q_buf, self._T_buf)
        return np.array(self._T_buf, dtype=np.float64).reshape(4, 4)

    def inverse(self, T: np.ndarray, q6_des: float = 0.0) -> list[np.ndarray]:
        if q6_des != 0.0:
            return self._python.inverse(T, q6_des)
        self._fill_buf(self._T_buf, np.asarray(T, dtype=np.float64).reshape(-1))
        n = int(self._lib.c_inverse(self._T_buf, self._q_sols_buf))
        return self._sols_from_buf(n)

    def compute_fk(self, q_urdf: list[float] | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self._fill_buf(self._q_buf, q_urdf)
        self._lib.c_compute_fk(self._q_buf, self._ee_t_buf, self._ee_r_buf)
        trans = np.array(self._ee_t_buf, dtype=np.float64)
        rot = np.array(self._ee_r_buf, dtype=np.float64).reshape(3, 3)
        return trans, rot

    def compute_ik(self, translation: np.ndarray, rotation: np.ndarray) -> list[np.ndarray]:
        self._fill_buf(self._ee_t_buf, translation)
        self._fill_buf(self._ee_r_buf, rotation)
        n = int(self._lib.c_compute_ik(self._ee_t_buf, self._ee_r_buf, self._q_sols_buf))
        return self._sols_from_buf(n)

    def forward_controller(self, q_urdf: list[float] | np.ndarray) -> tuple[list[float], list[float]]:
        self._fill_buf(self._q_buf, q_urdf)
        self._lib.c_forward_controller(self._q_buf, self._xyz_buf, self._rpy_buf)
        return [self._xyz_buf[0], self._xyz_buf[1], self._xyz_buf[2]], \
               [self._rpy_buf[0], self._rpy_buf[1], self._rpy_buf[2]]

    def inverse_controller(self, xyz_mm: list[float], rpy_deg: list[float]) -> list[np.ndarray]:
        self._fill_buf(self._xyz_buf, xyz_mm)
        self._fill_buf(self._rpy_buf, rpy_deg)
        n = int(self._lib.c_inverse_controller(self._xyz_buf, self._rpy_buf, self._q_sols_buf))
        return self._sols_from_buf(n)

    def forward_all(self, q_urdf: list[float] | np.ndarray) -> list[np.ndarray]:
        return self._python.forward_all(q_urdf)


class CR5Kinematics:
    """
    Kinematics solver for Dobot CR5.

    Instantiates `CR5PythonKinematics` or `CR5CppKinematics` from `backend`, then
    layers joint-limit expand / nearest-branch IK / singularity checks on top.

    :param joint_min/joint_max: 6-vector limits in radians (URDF).
    :param backend: "python", "cpp", or "auto" (prefer C++ lib when built).
    """

    def __init__(
        self,
        joint_min: list[float] = None,
        joint_max: list[float] = None,
        backend: str = "python",
    ):
        # J1–J5: [-π, π], J3 slightly tighter from URDF, J6: [-2π, 2π] (multi-turn).
        self.joint_min = np.array(joint_min if joint_min is not None else [
            -PI, -PI, -2.86159, -PI, -PI, -2.0 * PI
        ], dtype=np.float64)
        self.joint_max = np.array(joint_max if joint_max is not None else [
             PI,  PI,  2.86159,  PI,  PI,  2.0 * PI
        ], dtype=np.float64)

        self.T_base = _T_BASE
        self.T_base_inv = _T_BASE
        self.T_tool = _T_TOOL
        self.T_tool_inv = np.linalg.inv(_T_TOOL)

        self.backend = _resolve_backend(backend)
        if self.backend == "cpp":
            self._impl: CR5KinematicsBackend = CR5CppKinematics()
        else:
            self._impl = CR5PythonKinematics()

    # =========================================================================
    # 1. URDF STANDARD INTERFACES (delegated to python / cpp backend)
    # =========================================================================

    def forward(self, q_urdf: list[float] | np.ndarray) -> np.ndarray:
        """URDF-frame FK. Both backends apply DH q2/q4 ±π/2 internally."""
        return self._impl.forward(q_urdf)

    def forward_all(self, q_urdf: list[float] | np.ndarray) -> list[np.ndarray]:
        """Link frames T1–T6. Python stub only; jacobian still unused."""
        return self._impl.forward_all(q_urdf)

    def inverse(self, T: np.ndarray, q6_des: float = 0.0) -> list[np.ndarray]:
        """Analytical IK in URDF frame: up to 8 solutions, each normalized to [-π, π]."""
        return self._impl.inverse(T, q6_des)

    def compute_fk(self, q_urdf: list[float] | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """IKFast-style FK: (translation[3], rotation 3×3), same pose as `forward`."""
        return self._impl.compute_fk(q_urdf)

    def compute_ik(self, translation: np.ndarray, rotation: np.ndarray) -> list[np.ndarray]:
        """IKFast-style IK from translation + 3×3 rotation (URDF), same solutions as `inverse`."""
        return self._impl.compute_ik(translation, rotation)

    # =========================================================================
    # 2. DOBOT CONTROLLER INTERFACES
    # =========================================================================

    def forward_controller(self, q_urdf: list[float] | np.ndarray) -> tuple[list[float], list[float]]:
        """FK in Dobot controller frame. Returns (xyz_mm, rpy_deg) Euler ZYX (rx, ry, rz)."""
        return self._impl.forward_controller(q_urdf)

    def inverse_controller_matrix(self, T_ctrl: np.ndarray) -> list[np.ndarray]:
        """IK from a controller-frame 4×4. Strips base/tool, then URDF `inverse`."""
        return self.inverse(self.controller_matrix_to_urdf(T_ctrl))

    def inverse_controller(self, xyz_mm: list[float], rpy_deg: list[float]) -> list[np.ndarray]:
        """IK from Dobot pose (mm + ZYX deg)."""
        return self._impl.inverse_controller(xyz_mm, rpy_deg)

    def controller_matrix_to_urdf(self, T_ctrl: np.ndarray) -> np.ndarray:
        """
        T_urdf = T_base_inv * T_ctrl * T_tool_inv, written as a permutation:
        rows 0/1 of translation are negated (RotZ 180°); rotation columns are
        remapped by T_tool_inv = [[0,-1,0],[0,0,-1],[1,0,0]].
        """
        R = np.asarray(T_ctrl, dtype=np.float64)[:3, :3]
        p = np.asarray(T_ctrl, dtype=np.float64)[:3, 3]
        T = np.eye(4, dtype=np.float64)
        T[0, 0] = -R[0, 2]; T[0, 1] = R[0, 0]; T[0, 2] = R[0, 1]; T[0, 3] = -p[0]
        T[1, 0] = -R[1, 2]; T[1, 1] = R[1, 0]; T[1, 2] = R[1, 1]; T[1, 3] = -p[1]
        T[2, 0] = R[2, 2]; T[2, 1] = -R[2, 0]; T[2, 2] = -R[2, 1]; T[2, 3] = p[2]
        return T

    # =========================================================================
    # 3. BRANCH SELECTION (8 IK solutions + ±2π aliases)
    # =========================================================================

    def is_joint_valid(self, q: list[float] | np.ndarray, tolerance: float = 1e-4) -> bool:
        """True if all 6 joints lie inside [joint_min, joint_max] (soft tolerance)."""
        q_arr = np.asarray(q, dtype=np.float64)
        return bool(np.all(q_arr >= self.joint_min - tolerance) and
                    np.all(q_arr <= self.joint_max + tolerance))

    def expand_solutions(self, base_sols: list[np.ndarray]) -> list[np.ndarray]:
        """
        For each of the ≤8 analytical sols in [-π, π], try k∈{-1,0,1} turns per
        joint (q + 2πk) and keep those inside limits. Needed for J6 (±2π) so
        tracking can stay on the robot's current winding instead of wrapping to -10°.
        """
        expanded = []

        def _backtrack(sol: np.ndarray, joint_idx: int):
            if joint_idx == 6:
                if self.is_joint_valid(sol):
                    expanded.append(sol.copy())
                return
            orig = sol[joint_idx]
            for k in (-1, 0, 1):
                cand = orig + k * 2.0 * PI
                if (cand >= self.joint_min[joint_idx] - 1e-4 and
                        cand <= self.joint_max[joint_idx] + 1e-4):
                    sol[joint_idx] = cand
                    _backtrack(sol, joint_idx + 1)
            sol[joint_idx] = orig

        for base_sol in base_sols:
            _backtrack(np.array(base_sol, dtype=np.float64, copy=True), 0)
        return expanded

    def solve_ik(self, T: np.ndarray, expand: bool = True) -> list[np.ndarray]:
        """Analytical IK plus optional ±2π expansion; drops out-of-limit sols."""
        base_sols = self.inverse(T)
        if not expand:
            return [s for s in base_sols if self.is_joint_valid(s)]
        return self.expand_solutions(base_sols)

    def get_best_ik(
            self,
            T: np.ndarray,
            current_joints: list[float] | np.ndarray,
            weights: list[float] = None) -> np.ndarray | None:
        """
        Pick the IK branch closest to `current_joints` to avoid shoulder/elbow/wrist flips.

        Distance is the weighted L2 of the *unwrapped* joint error
        d = wrap(sol - curr) ∈ [-π, π]. The returned vector is curr+d when that
        stays in limits (same winding as the live robot); otherwise the in-limit
        expanded representative is returned.
        """
        valid_sols = self.solve_ik(T, expand=False)
        if not valid_sols:
            return None

        curr = np.asarray(current_joints, dtype=np.float64)
        w = np.asarray(weights if weights is not None else [1.0] * 6, dtype=np.float64)
        best_sol = None
        min_dist = float("inf")
        for sol in valid_sols:
            d = np.mod(sol - curr + PI, 2.0 * PI) - PI
            unwrapped = curr + d
            if self.is_joint_valid(unwrapped):
                dist = float(np.sum(w * (d ** 2)))
                if dist < min_dist:
                    min_dist = dist
                    best_sol = unwrapped
            elif self.is_joint_valid(sol):
                dist = float(np.sum(w * (d ** 2)))
                if dist < min_dist:
                    min_dist = dist
                    best_sol = sol
        return best_sol

    def get_best_ik_controller(
            self,
            T_ctrl: np.ndarray,
            current_joints: list[float] | np.ndarray,
            weights: list[float] = None) -> np.ndarray | None:
        """Same as `get_best_ik` but T_ctrl is Dobot controller 4×4 (meters)."""
        return self.get_best_ik(self.controller_matrix_to_urdf(T_ctrl), current_joints, weights)

    # =========================================================================
    # 4. SINGULARITY DIAGNOSIS
    # =========================================================================

    def jacobian(self, q_urdf: list[float] | np.ndarray) -> np.ndarray:
        """
        Geometric Jacobian in Base. Depends on `forward_all`; the Python core still
        returns identity frames, so this (and manipulability) is not used by the verifier.
        """
        T_chain = self.forward_all(q_urdf)
        o_0 = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        z_0 = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        origins = [o_0] + [T[:3, 3] for T in T_chain]
        z_axes = [z_0] + [T[:3, 2] for T in T_chain]
        o_e = origins[-1]
        J = np.zeros((6, 6), dtype=np.float64)
        for i in range(6):
            J[:3, i] = np.cross(z_axes[i], o_e - origins[i])
            J[3:, i] = z_axes[i]
        return J

    def manipulability(self, q_urdf: list[float] | np.ndarray) -> float:
        """Yoshikawa w = sqrt(det(J J^T)). Invalid until forward_all is real; unused."""
        J = self.jacobian(q_urdf)
        det = np.linalg.det(J @ J.T)
        return float(math.sqrt(max(0.0, det)))

    def _shoulder_q1_half_separation_rad(self, T: np.ndarray) -> float:
        """
        Half-angle between the two analytical q1 solutions from the same discriminant
        as `cr5_ur_kin.inverse` (A, B, R=A²+B², q1 from acos(d4/sqrt(R))).

        0  → shoulder singularity (branches coincide, or wrist inside the |d4| cylinder).
        π/2 → the two shoulders are 180° apart (well conditioned).
        """
        T_flat = np.asarray(T, dtype=np.float64).reshape(-1)
        t02, t03 = -T_flat[0], -T_flat[3]
        t12, t13 = -T_flat[4], -T_flat[7]
        a = cr5_ur_kin.D6 * t12 - t13
        b = cr5_ur_kin.D6 * t02 - t03
        r = a * a + b * b
        d4 = abs(cr5_ur_kin.D4)
        if r <= 1e-16:
            return 0.0
        ratio = d4 / math.sqrt(r)
        if ratio >= 1.0:
            return 0.0
        if ratio <= -1.0:
            return PI
        return math.acos(ratio)

    def check_singularity_risk(self, q_urdf: list[float] | np.ndarray, T: np.ndarray = None) -> dict:
        """
        Wrist:  sin(q5)≈0 → J4 ∥ J6 (flip-wrist pair collapses). Works for q5≈0°/±180°/360°.
        Elbow:  sin(q3)≈0 → a2, a3 collinear (flip-elbow pair collapses). Practical CR5
                case is q3≈0° (stretch); ±180° is outside J3 limits (~±164°).
        Shoulder: two q1 solutions closer than 6° → wrist near the J1-axis cylinder of
                radius |d4|; Cartesian motion then requires a large Δq1 (arm swing).
        """
        q = np.asarray(q_urdf, dtype=np.float64)
        if T is None:
            T = self.forward(q)

        is_wrist = abs(math.sin(float(q[4]))) < _SING_SIN
        is_elbow = abs(math.sin(float(q[2]))) < _SING_SIN
        q1_half = self._shoulder_q1_half_separation_rad(T)
        is_shoulder = q1_half < _SHOULDER_HALF_RAD

        return {
            "wrist_singularity": is_wrist,
            "elbow_singularity": is_elbow,
            "shoulder_singularity": is_shoulder,
            "wrist_angle_deg": float(math.degrees(q[4])),
            "elbow_angle_deg": float(math.degrees(q[2])),
            "shoulder_q1_separation_deg": float(math.degrees(2.0 * q1_half)),
            "is_singular": bool(is_wrist or is_elbow or is_shoulder),
        }
