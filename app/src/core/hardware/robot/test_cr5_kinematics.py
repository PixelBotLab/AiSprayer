"""
Unit tests and benchmark for CR5 Kinematics Solver.
Ported from test_cr5_kinematics.cpp and kinematics_benchmark_test.cpp.
"""

import unittest
import math
import time
import numpy as np
import sys
import os

# Add workspace root to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../")))

from app.src.core.hardware.robot.cr5_kinematics import CR5Kinematics
from app.src.core.hardware.robot import cr5_ur_kin

class TestCR5Kinematics(unittest.TestCase):
    def setUp(self):
        self.solver = CR5Kinematics()

    def test_single_case_dobot_controller(self):
        """
        Tests against real Dobot CR5 measured pose from test_cr5_kinematics.cpp.
        Outputs Case 1 and Case 2 poses as requested.
        """
        cases = [
            ("Case 1", [180.159, -0.293, 90.653, 90.066, 90.035, 0.077]),
            ("Case 2", [0.028, 0.009, -90.029, 90.066, 90.035, 0.077])
        ]
        
        print("\n--- Dobot Controller FK Outputs ---")
        for name, q_target_deg in cases:
            q_target_rad = [math.radians(a) for a in q_target_deg]
            xyz_mm, rpy_deg = self.solver.forward_controller(q_target_rad)
            
            print(f"{name} Joint Angles (deg): {[round(x, 3) for x in q_target_deg]}")
            print(f"{name} TCP Position (mm): XYZ = {[round(x, 2) for x in xyz_mm]}")
            print(f"{name} TCP Orientation (deg): RPY = {[round(x, 2) for x in rpy_deg]}\n")
            
            # Test inverse controller
            sols = self.solver.inverse_controller(xyz_mm, rpy_deg)
            self.assertGreater(len(sols), 0, f"Should find at least 1 IK solution for {name}")

            found_match = False
            for sol in sols:
                sol_deg = [math.degrees(a) for a in sol]
                diffs = []
                for j in range(6):
                    d = abs(sol_deg[j] - q_target_deg[j])
                    while d > 180.0:
                        d = abs(d - 360.0)
                    diffs.append(d)
                if max(diffs) < 0.5:
                    found_match = True
                    break
            
            self.assertTrue(found_match, f"Should find exact matching target configuration among IK solutions for {name}")

    def test_fk_ik_closed_loop_consistency(self):
        """
        Tests closed loop FK -> IK -> FK position and rotation accuracy across multiple random joint configurations.
        """
        test_configs = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [math.radians(45), math.radians(-30), math.radians(60), math.radians(10), math.radians(45), math.radians(-20)],
            [math.radians(-90), math.radians(45), math.radians(-45), math.radians(90), math.radians(-90), math.radians(180)],
            [math.radians(120), math.radians(-60), math.radians(30), math.radians(-45), math.radians(60), math.radians(90)]
        ]

        for q in test_configs:
            T_fk = self.solver.forward(q)
            sols = self.solver.inverse(T_fk)
            self.assertGreater(len(sols), 0, f"Failed to find IK solutions for config {q}")

            # Verify that every returned solution maps back to the same FK pose
            for sol in sols:
                T_sol = self.solver.forward(sol)
                pos_err = np.linalg.norm(T_sol[:3, 3] - T_fk[:3, 3]) * 1000.0  # in mm
                rot_err = np.linalg.norm(T_sol[:3, :3] - T_fk[:3, :3])
                self.assertLess(pos_err, 0.05, f"Position error {pos_err}mm exceeds tolerance")
                self.assertLess(rot_err, 1e-3, f"Rotation error exceeds tolerance")

    def test_best_ik_smooth_selection(self):
        """
        Tests that get_best_ik picks the closest solution to current configuration without branch jumps.
        """
        q_current = np.array([math.radians(30), math.radians(-20), math.radians(45), 
                              math.radians(10), math.radians(60), math.radians(0)])
        T_target = self.solver.forward(q_current)

        best_sol = self.solver.get_best_ik(T_target, q_current)
        self.assertIsNotNone(best_sol)
        diff = np.linalg.norm(best_sol - q_current)
        self.assertLess(diff, 1e-4, "get_best_ik should recover the identical current joint configuration")

    def test_benchmark_performance(self):
        """
        Benchmark: Measures execution time for 10,000 continuous IK evaluations.
        """
        q_sample = [math.radians(45.0), math.radians(-30.0), math.radians(60.0),
                    math.radians(0.0), math.radians(45.0), math.radians(10.0)]
        T_bench = self.solver.forward(q_sample)

        num_iterations = 10000
        start = time.perf_counter()
        for _ in range(num_iterations):
            _ = self.solver.inverse(T_bench)
        elapsed = time.perf_counter() - start

        avg_us = (elapsed * 1e6) / num_iterations
        freq = num_iterations / elapsed

        print(f"\n[CR5 Kinematics Python Benchmark]")
        print(f"Iterations:     {num_iterations}")
        print(f"Total Time:     {elapsed * 1000.0:.2f} ms")
        print(f"Average Time:   {avg_us:.2f} us / solve")
        print(f"Throughput:     {int(freq)} Hz")

        self.assertLess(avg_us, 100.0, "Single IK evaluation in Python should be < 100 microseconds")

if __name__ == '__main__':
    unittest.main()
