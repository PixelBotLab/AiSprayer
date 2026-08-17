"""
Unit tests and benchmark for CR5 Kinematics Solver.

Compares the four public APIs (plus controller adapters) in Python vs C++
under identical inputs, iteration counts, and closed-loop tolerances.

Keep protocol constants in sync with cr5_kinematics_cpp/test_cr5_kinematics.cpp.
"""

import unittest
import math
import time
import ctypes
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../")))

from app.src.core.hardware.robot.cr5_kinematics import (
    CR5CppKinematics,
    CR5Kinematics,
    CR5PythonKinematics,
    find_cr5_cpp_lib,
)

# ---- shared protocol (must match test_cr5_kinematics.cpp) ----
NUM_WARMUP = 200
NUM_ITERATIONS = 100000
SAMPLE_Q_DEG = [45.0, -30.0, 60.0, 0.0, 45.0, 10.0]
FUNC_CONFIGS_DEG = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [45.0, -30.0, 60.0, 10.0, 45.0, -20.0],
    [-90.0, 45.0, -45.0, 90.0, -90.0, 180.0],
    [120.0, -60.0, 30.0, -45.0, 60.0, 90.0],
    [180.159, -0.293, 90.653, 90.066, 90.035, 0.077],
    [0.028, 0.009, -90.029, 90.066, 90.035, 0.077],
]
DOBOT_CASES = [
    ("Case 1", [180.159, -0.293, 90.653, 90.066, 90.035, 0.077]),
    ("Case 2", [0.028, 0.009, -90.029, 90.066, 90.035, 0.077]),
]
IK_TOL_RAD = math.radians(0.5)
POS_TOL_MM = 0.05
ROT_TOL = 1e-3


def _deg_to_rad(deg):
    return [math.radians(a) for a in deg]


def _ang_diff(a, b):
    d = abs(a - b)
    while d > math.pi:
        d = abs(d - 2.0 * math.pi)
    return d


def _joints_match(a, b, tol=IK_TOL_RAD):
    return all(_ang_diff(a[j], b[j]) <= tol for j in range(6))


def _find_matching_sol(sols, q_target, tol=IK_TOL_RAD):
    return any(_joints_match(sol, q_target, tol) for sol in sols)


def _load_cpp_lib():
    """Load libur_kin for in-process C++ vs Python numerical comparison."""
    base = os.path.join(os.path.dirname(__file__), "cr5_kinematics_cpp")
    candidates = [
        os.path.join(base, "build", "libur_kin.dylib"),
        os.path.join(base, "build", "libur_kin.so"),
        os.path.join(base, "libur_kin.dylib"),
        os.path.join(base, "libur_kin.so"),
    ]
    path = next((p for p in candidates if os.path.exists(p)), None)
    if path is None:
        return None
    lib = ctypes.CDLL(path)
    lib.c_forward.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
    lib.c_forward.restype = None
    lib.c_inverse.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
    lib.c_inverse.restype = ctypes.c_int
    lib.c_compute_fk.argtypes = [
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.c_compute_fk.restype = None
    lib.c_compute_ik.argtypes = [
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.c_compute_ik.restype = ctypes.c_int
    lib.c_forward_controller.argtypes = [
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.c_forward_controller.restype = None
    lib.c_inverse_controller.argtypes = [
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.c_inverse_controller.restype = ctypes.c_int
    return lib


def _c_arr(values, n):
    return (ctypes.c_double * n)(*values)


class _CppHotPath:
    """ctypes session with preallocated buffers, matching C++ stress reuse."""

    def __init__(self, lib, q, T, trans, rot, xyz, rpy):
        self.lib = lib
        self.q = (ctypes.c_double * 6)(*q)
        self.T = (ctypes.c_double * 16)(*np.asarray(T, dtype=np.float64).reshape(-1))
        self.q_sols = (ctypes.c_double * 48)()
        self.ee_t = (ctypes.c_double * 3)(*np.asarray(trans, dtype=np.float64).reshape(-1))
        self.ee_r = (ctypes.c_double * 9)(*np.asarray(rot, dtype=np.float64).reshape(-1))
        self.xyz = (ctypes.c_double * 3)(*xyz)
        self.rpy = (ctypes.c_double * 3)(*rpy)
        self.T_out = (ctypes.c_double * 16)()
        self.ee_t_out = (ctypes.c_double * 3)()
        self.ee_r_out = (ctypes.c_double * 9)()
        self.xyz_out = (ctypes.c_double * 3)()
        self.rpy_out = (ctypes.c_double * 3)()

    def forward(self):
        self.lib.c_forward(self.q, self.T_out)
        return 1

    def inverse(self):
        return int(self.lib.c_inverse(self.T, self.q_sols))

    def compute_fk(self):
        self.lib.c_compute_fk(self.q, self.ee_t_out, self.ee_r_out)
        return 1

    def compute_ik(self):
        return int(self.lib.c_compute_ik(self.ee_t, self.ee_r, self.q_sols))

    def forward_controller(self):
        self.lib.c_forward_controller(self.q, self.xyz_out, self.rpy_out)
        return 1

    def inverse_controller(self):
        return int(self.lib.c_inverse_controller(self.xyz, self.rpy, self.q_sols))


def _cpp_forward(lib, q):
    T = (ctypes.c_double * 16)()
    lib.c_forward(_c_arr(q, 6), T)
    return np.array(T, dtype=np.float64).reshape(4, 4)


def _cpp_inverse(lib, T):
    q_sols = (ctypes.c_double * 48)()
    n = lib.c_inverse(_c_arr(T.flatten(), 16), q_sols)
    return [np.array([q_sols[i * 6 + j] for j in range(6)]) for i in range(n)]


def _cpp_compute_fk(lib, q):
    ee_t = (ctypes.c_double * 3)()
    ee_r = (ctypes.c_double * 9)()
    lib.c_compute_fk(_c_arr(q, 6), ee_t, ee_r)
    return np.array(ee_t, dtype=np.float64), np.array(ee_r, dtype=np.float64).reshape(3, 3)


def _cpp_compute_ik(lib, trans, rot):
    q_sols = (ctypes.c_double * 48)()
    n = lib.c_compute_ik(_c_arr(trans, 3), _c_arr(rot.flatten(), 9), q_sols)
    return [np.array([q_sols[i * 6 + j] for j in range(6)]) for i in range(n)]


def _cpp_forward_controller(lib, q):
    xyz = (ctypes.c_double * 3)()
    rpy = (ctypes.c_double * 3)()
    lib.c_forward_controller(_c_arr(q, 6), xyz, rpy)
    return list(xyz), list(rpy)


def _cpp_inverse_controller(lib, xyz, rpy):
    q_sols = (ctypes.c_double * 48)()
    n = lib.c_inverse_controller(_c_arr(xyz, 3), _c_arr(rpy, 3), q_sols)
    return [np.array([q_sols[i * 6 + j] for j in range(6)]) for i in range(n)]


def _match_sol_sets(a_sols, b_sols, tol=1e-6):
    if len(a_sols) != len(b_sols):
        return False
    used = [False] * len(b_sols)
    for a in a_sols:
        found = False
        for i, b in enumerate(b_sols):
            if not used[i] and _joints_match(a, b, tol):
                used[i] = True
                found = True
                break
        if not found:
            return False
    return True


def _bench(name, fn):
    sink = 0
    for _ in range(NUM_WARMUP):
        sink += fn()
    t0 = time.perf_counter()
    total = 0
    for _ in range(NUM_ITERATIONS):
        total += fn()
    elapsed = time.perf_counter() - t0
    sink += total
    avg_us = (elapsed * 1e6) / NUM_ITERATIONS
    freq = NUM_ITERATIONS / elapsed
    sols_per = (total // NUM_ITERATIONS) if total else 0
    return {
        "name": name,
        "avg_us": avg_us,
        "freq": freq,
        "sols_per": sols_per,
        "sink": sink,
    }


def _fmt_joints_deg(q_rad):
    return " ".join(f"{math.degrees(a):.3f}" for a in q_rad)


def _print_controller_case(tag, name, q_target_rad, xyz_mm, rpy_deg, sols, fk_fn):
    print(f"\n----- {name} [{tag}] -----")
    print(f"{name} Joint Angles (deg): {[round(math.degrees(a), 3) for a in q_target_rad]}")
    print(f"{name} TCP Position (mm): XYZ = {[round(x, 3) for x in xyz_mm]}")
    print(f"{name} TCP Orientation (deg): RPY = {[round(x, 3) for x in rpy_deg]}")
    print(f"Found {len(sols)} IK solutions from controller interface.")
    for i, sol in enumerate(sols):
        mark = " <-- MATCHES TARGET" if _joints_match(sol, q_target_rad) else ""
        print(f"  Solution {i} (deg): {_fmt_joints_deg(sol)}{mark}")
    print(f"--- Verifying FK for all {len(sols)} IK solutions ---")
    for i, sol in enumerate(sols):
        sol_xyz, sol_rpy = fk_fn(sol)
        print(
            f"  Sol {i} FK -> XYZ (mm): "
            f"{sol_xyz[0]:10.3f} {sol_xyz[1]:10.3f} {sol_xyz[2]:10.3f}"
            f" | Euler ZYX (rx, ry, rz): "
            f"{sol_rpy[0]:10.3f} {sol_rpy[1]:10.3f} {sol_rpy[2]:10.3f}"
        )


class TestCR5Kinematics(unittest.TestCase):
    def setUp(self):
        self.solver = CR5Kinematics()

    def test_single_case_dobot_controller(self):
        """
        Dobot CR5 measured poses: Case 1 and Case 2.
        Prints controller pose + all IK solutions for both pure Python and ctypes→libur_kin.
        """
        lib = _load_cpp_lib()
        if lib is None:
            self.skipTest("libur_kin not found; build cr5_kinematics_cpp first")

        print("\n========== DOBOT CONTROLLER CASES (Python vs ctypes→libur_kin) ==========")
        for name, q_target_deg in DOBOT_CASES:
            q_target_rad = _deg_to_rad(q_target_deg)

            xyz_py, rpy_py = self.solver.forward_controller(q_target_rad)
            sols_py = self.solver.inverse_controller(xyz_py, rpy_py)
            _print_controller_case(
                "pure Python", name, q_target_rad, xyz_py, rpy_py, sols_py,
                self.solver.forward_controller,
            )

            xyz_cpp, rpy_cpp = _cpp_forward_controller(lib, q_target_rad)
            sols_cpp = _cpp_inverse_controller(lib, xyz_cpp, rpy_cpp)
            _print_controller_case(
                "ctypes→libur_kin", name, q_target_rad, xyz_cpp, rpy_cpp, sols_cpp,
                lambda sol, _lib=lib: _cpp_forward_controller(_lib, sol),
            )

            self.assertGreater(len(sols_py), 0, f"{name} Python IK empty")
            self.assertGreater(len(sols_cpp), 0, f"{name} C++ IK empty")
            self.assertTrue(_find_matching_sol(sols_py, q_target_rad), f"{name} Python missed target q")
            self.assertTrue(_find_matching_sol(sols_cpp, q_target_rad), f"{name} C++ missed target q")
            self.assertLess(
                np.linalg.norm(np.array(xyz_py) - np.array(xyz_cpp)),
                1e-4,
                f"{name} XYZ mismatch Python {xyz_py} vs C++ {xyz_cpp}",
            )
            for a, b in zip(rpy_py, rpy_cpp):
                d = abs(a - b)
                while d > 180.0:
                    d = abs(d - 360.0)
                self.assertLess(d, 1e-4, f"{name} RPY mismatch Python {rpy_py} vs C++ {rpy_cpp}")
            self.assertTrue(
                _match_sol_sets(sols_py, sols_cpp, tol=1e-6),
                f"{name} IK solution sets differ: py={len(sols_py)} cpp={len(sols_cpp)}",
            )
            for sols, tag in ((sols_py, "Python"), (sols_cpp, "C++")):
                for i, sol in enumerate(sols):
                    sol_xyz, _ = (
                        self.solver.forward_controller(sol) if tag == "Python"
                        else _cpp_forward_controller(lib, sol)
                    )
                    src_xyz = xyz_py if tag == "Python" else xyz_cpp
                    pos_err = math.sqrt(sum((a - b) ** 2 for a, b in zip(src_xyz, sol_xyz)))
                    self.assertLess(pos_err, POS_TOL_MM, f"{name} {tag} sol {i} pos error {pos_err} mm")
            print(f"{name} COMPARE OK: Python and C++ lib agree on pose and {len(sols_py)} IK solutions.\n")

    def test_functional_four_interfaces(self):
        """Closed-loop functional checks for forward / inverse / compute_fk / compute_ik (+ controller)."""
        for idx, q_deg in enumerate(FUNC_CONFIGS_DEG):
            q = _deg_to_rad(q_deg)
            with self.subTest(config=idx, q_deg=q_deg):
                # 1) forward
                T = self.solver.forward(q)
                self.assertAlmostEqual(T[3, 3], 1.0, places=12)
                np.testing.assert_allclose(T[3, :3], 0.0, atol=1e-12)

                # 2) inverse
                sols = self.solver.inverse(T)
                self.assertGreater(len(sols), 0)
                self.assertTrue(_find_matching_sol(sols, q), f"inverse did not recover q for config {idx}")
                for sol in sols:
                    T_sol = self.solver.forward(sol)
                    pos_err = np.linalg.norm(T_sol[:3, 3] - T[:3, 3]) * 1000.0
                    rot_err = np.linalg.norm(T_sol[:3, :3] - T[:3, :3])
                    self.assertLess(pos_err, POS_TOL_MM)
                    self.assertLess(rot_err, ROT_TOL)

                # 3) compute_fk matches forward unpack
                trans, rot = self.solver.compute_fk(q)
                np.testing.assert_allclose(trans, T[:3, 3], atol=1e-12)
                np.testing.assert_allclose(rot, T[:3, :3], atol=1e-12)

                # 4) compute_ik matches inverse
                cik = self.solver.compute_ik(trans, rot)
                self.assertEqual(len(cik), len(sols))
                self.assertTrue(_find_matching_sol(cik, q), f"compute_ik did not recover q for config {idx}")
                self.assertTrue(_match_sol_sets(sols, cik, tol=1e-9))

                # controller adapters
                xyz, rpy = self.solver.forward_controller(q)
                csols = self.solver.inverse_controller(xyz, rpy)
                self.assertGreater(len(csols), 0)
                self.assertTrue(
                    _find_matching_sol(csols, q),
                    f"inverse_controller did not recover q for config {idx}",
                )
                for sol in csols:
                    xyz2, _ = self.solver.forward_controller(sol)
                    pos_err = math.sqrt(sum((a - b) ** 2 for a, b in zip(xyz, xyz2)))
                    self.assertLess(pos_err, POS_TOL_MM)

    def test_cpp_python_numerical_agreement(self):
        """Same inputs: Python APIs vs C++ libur_kin (ctypes)."""
        lib = _load_cpp_lib()
        if lib is None:
            self.skipTest("libur_kin not found; build cr5_kinematics_cpp first")

        print("\n========== C++ vs Python NUMERICAL AGREEMENT ==========")
        for idx, q_deg in enumerate(FUNC_CONFIGS_DEG):
            q = _deg_to_rad(q_deg)
            with self.subTest(config=idx):
                T_py = self.solver.forward(q)
                T_cpp = _cpp_forward(lib, q)
                t_err = np.linalg.norm(T_py[:3, 3] - T_cpp[:3, 3]) * 1000.0
                r_err = np.linalg.norm(T_py[:3, :3] - T_cpp[:3, :3])
                self.assertLess(t_err, 1e-6, f"forward trans mismatch config {idx}: {t_err} mm")
                self.assertLess(r_err, 1e-9, f"forward rot mismatch config {idx}: {r_err}")

                sols_py = self.solver.inverse(T_py)
                sols_cpp = _cpp_inverse(lib, T_cpp)
                self.assertTrue(
                    _match_sol_sets(sols_py, sols_cpp, tol=1e-6),
                    f"inverse sols mismatch config {idx}: py={len(sols_py)} cpp={len(sols_cpp)}",
                )

                trans_py, rot_py = self.solver.compute_fk(q)
                trans_cpp, rot_cpp = _cpp_compute_fk(lib, q)
                np.testing.assert_allclose(trans_py, trans_cpp, atol=1e-9)
                np.testing.assert_allclose(rot_py, rot_cpp, atol=1e-9)

                cik_py = self.solver.compute_ik(trans_py, rot_py)
                cik_cpp = _cpp_compute_ik(lib, trans_cpp, rot_cpp)
                self.assertTrue(_match_sol_sets(cik_py, cik_cpp, tol=1e-6))

                xyz_py, rpy_py = self.solver.forward_controller(q)
                xyz_cpp, rpy_cpp = _cpp_forward_controller(lib, q)
                self.assertLess(np.linalg.norm(np.array(xyz_py) - np.array(xyz_cpp)), 1e-4)
                # Euler can wrap ±180; compare via wrapped deg
                for a, b in zip(rpy_py, rpy_cpp):
                    d = abs(a - b)
                    while d > 180.0:
                        d = abs(d - 360.0)
                    self.assertLess(d, 1e-4, f"rpy mismatch {rpy_py} vs {rpy_cpp}")

                ctrl_py = self.solver.inverse_controller(xyz_py, rpy_py)
                ctrl_cpp = _cpp_inverse_controller(lib, xyz_cpp, rpy_cpp)
                self.assertTrue(
                    _match_sol_sets(ctrl_py, ctrl_cpp, tol=1e-6),
                    f"inverse_controller sols mismatch config {idx}",
                )
            print(f"  config {idx}: PASS  (forward/inverse/ComputeFk/ComputeIk/controller)")
        print("All configs numerically agree between C++ and Python.")

    def test_best_ik_smooth_selection(self):
        q_current = np.array([
            math.radians(30), math.radians(-20), math.radians(45),
            math.radians(10), math.radians(60), math.radians(0),
        ])
        T_target = self.solver.forward(q_current)
        best_sol = self.solver.get_best_ik(T_target, q_current)
        self.assertIsNotNone(best_sol)
        self.assertLess(np.linalg.norm(best_sol - q_current), 1e-4)

    def test_best_ik_unwraps_onto_current_winding(self):
        q_current = np.array([
            math.radians(30), math.radians(-20), math.radians(45),
            math.radians(10), math.radians(60), math.radians(350.0),
        ])
        T_target = self.solver.forward(q_current)
        best_sol = self.solver.get_best_ik(T_target, q_current)
        self.assertIsNotNone(best_sol)
        self.assertLess(abs(best_sol[5] - q_current[5]), 1e-3)
        self.assertTrue(self.solver.is_joint_valid(best_sol))

    def test_check_singularity_risk(self):
        wrist_q = np.array([0.0, 0.0, math.radians(90.0), 0.0, 0.0, 0.0])
        wrist = self.solver.check_singularity_risk(wrist_q)
        self.assertTrue(wrist["wrist_singularity"])
        self.assertTrue(wrist["is_singular"])

        wrist_wrapped = wrist_q.copy()
        wrist_wrapped[4] = 2.0 * math.pi
        self.assertTrue(self.solver.check_singularity_risk(wrist_wrapped)["wrist_singularity"])

        elbow_q = np.array([0.0, 0.0, 0.0, 0.0, math.radians(45.0), 0.0])
        elbow = self.solver.check_singularity_risk(elbow_q)
        self.assertTrue(elbow["elbow_singularity"])

        normal_q = np.array(_deg_to_rad([180.159, -0.293, 90.653, 90.066, 90.035, 0.077]))
        normal = self.solver.check_singularity_risk(normal_q)
        self.assertFalse(normal["wrist_singularity"])
        self.assertFalse(normal["elbow_singularity"])
        self.assertFalse(normal["is_singular"])

    def test_cpp_backend_matches_python(self):
        """CR5Kinematics(backend='cpp') must match backend='python' on all public APIs."""
        if find_cr5_cpp_lib() is None:
            self.skipTest("libur_kin not found; build cr5_kinematics_cpp first")

        py = CR5Kinematics(backend="python")
        cpp = CR5Kinematics(backend="cpp")
        self.assertIsInstance(py._impl, CR5PythonKinematics)
        self.assertIsInstance(cpp._impl, CR5CppKinematics)
        self.assertEqual(cpp.backend, "cpp")

        for idx, q_deg in enumerate(FUNC_CONFIGS_DEG):
            q = _deg_to_rad(q_deg)
            with self.subTest(config=idx, q_deg=q_deg):
                T_py = py.forward(q)
                T_cpp = cpp.forward(q)
                self.assertLess(np.linalg.norm(T_py[:3, 3] - T_cpp[:3, 3]) * 1000.0, 1e-6)
                self.assertLess(np.linalg.norm(T_py[:3, :3] - T_cpp[:3, :3]), 1e-9)

                self.assertTrue(_match_sol_sets(py.inverse(T_py), cpp.inverse(T_cpp), tol=1e-6))

                trans_py, rot_py = py.compute_fk(q)
                trans_cpp, rot_cpp = cpp.compute_fk(q)
                np.testing.assert_allclose(trans_py, trans_cpp, atol=1e-9)
                np.testing.assert_allclose(rot_py, rot_cpp, atol=1e-9)
                self.assertTrue(
                    _match_sol_sets(py.compute_ik(trans_py, rot_py),
                                    cpp.compute_ik(trans_cpp, rot_cpp), tol=1e-6)
                )

                xyz_py, rpy_py = py.forward_controller(q)
                xyz_cpp, rpy_cpp = cpp.forward_controller(q)
                self.assertLess(np.linalg.norm(np.array(xyz_py) - np.array(xyz_cpp)), 1e-4)
                for a, b in zip(rpy_py, rpy_cpp):
                    d = abs(a - b)
                    while d > 180.0:
                        d = abs(d - 360.0)
                    self.assertLess(d, 1e-4)

                self.assertTrue(
                    _match_sol_sets(
                        py.inverse_controller(xyz_py, rpy_py),
                        cpp.inverse_controller(xyz_cpp, rpy_cpp),
                        tol=1e-6,
                    )
                )

                best_py = py.get_best_ik(T_py, q)
                best_cpp = cpp.get_best_ik(T_cpp, q)
                self.assertIsNotNone(best_py)
                self.assertIsNotNone(best_cpp)
                self.assertTrue(_joints_match(best_py, best_cpp, tol=1e-6))

    def test_backend_auto_and_invalid(self):
        auto = CR5Kinematics(backend="auto")
        self.assertIn(auto.backend, ("python", "cpp"))
        if find_cr5_cpp_lib() is not None:
            self.assertEqual(auto.backend, "cpp")
            self.assertIsInstance(auto._impl, CR5CppKinematics)
        else:
            self.assertEqual(auto.backend, "python")
            self.assertIsInstance(auto._impl, CR5PythonKinematics)
        with self.assertRaises(ValueError):
            CR5Kinematics(backend="fortran")

    def test_stress_four_interfaces(self):
        """Stress: pure Python vs ctypes→libur_kin, same pose / warmup / iterations."""
        lib = _load_cpp_lib()
        if lib is None:
            self.skipTest("libur_kin not found; build cr5_kinematics_cpp first")

        q = _deg_to_rad(SAMPLE_Q_DEG)
        T = self.solver.forward(q)
        trans, rot = self.solver.compute_fk(q)
        xyz, rpy = self.solver.forward_controller(q)
        cpp = _CppHotPath(lib, q, T, trans, rot, xyz, rpy)

        py_rows = [
            _bench("1. forward", lambda: (self.solver.forward(q), 1)[1]),
            _bench("2. inverse", lambda: len(self.solver.inverse(T))),
            _bench("3. compute_fk", lambda: (self.solver.compute_fk(q), 1)[1]),
            _bench("4. compute_ik", lambda: len(self.solver.compute_ik(trans, rot))),
            _bench("5. forward_controller", lambda: (self.solver.forward_controller(q), 1)[1]),
            _bench("6. inverse_controller", lambda: len(self.solver.inverse_controller(xyz, rpy))),
        ]
        cpp_rows = [
            _bench("1. forward", cpp.forward),
            _bench("2. inverse", cpp.inverse),
            _bench("3. compute_fk", cpp.compute_fk),
            _bench("4. compute_ik", cpp.compute_ik),
            _bench("5. forward_controller", cpp.forward_controller),
            _bench("6. inverse_controller", cpp.inverse_controller),
        ]

        print("\n========== STRESS TEST (pure Python vs ctypes→libur_kin) ==========")
        print(f"warmup={NUM_WARMUP}  iterations={NUM_ITERATIONS}  "
              f"sample_q_deg={SAMPLE_Q_DEG}")
        print(f"{'Interface':<24}{'Python us':>12}{'ctypes us':>12}"
              f"{'speedup':>10}{'py Hz':>12}{'ctypes Hz':>12}{'sols':>8}")
        print("-" * 90)
        for py_r, cpp_r in zip(py_rows, cpp_rows):
            speedup = py_r["avg_us"] / cpp_r["avg_us"] if cpp_r["avg_us"] > 0 else float("inf")
            print(f"{py_r['name']:<24}{py_r['avg_us']:12.2f}{cpp_r['avg_us']:12.2f}"
                  f"{speedup:9.1f}x{int(py_r['freq']):12d}{int(cpp_r['freq']):12d}"
                  f"{cpp_r['sols_per']:8d}")
            self.assertEqual(py_r["sols_per"], cpp_r["sols_per"], py_r["name"])

        self.assertGreater(py_rows[1]["sols_per"], 0)
        self.assertEqual(py_rows[1]["sols_per"], py_rows[3]["sols_per"])


if __name__ == "__main__":
    unittest.main()
