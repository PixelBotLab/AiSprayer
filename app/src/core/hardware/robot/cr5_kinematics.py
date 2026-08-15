"""
CR5 Kinematics Solver and Dobot Controller Adapter.
Faithfully ported from cr5_kinematics.cpp and cr5_kinematics_solver.cpp.
Provides high-level kinematic interfaces, Dobot controller frame conversions,
multi-turn angle expansion, near-neighbor smooth branch selection, and Jacobian calculations.
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

from . import cr5_ur_kin

PI = math.pi

class CR5Kinematics:
    """
    Kinematics solver for Dobot CR5 collaborative robot.
    """
    def __init__(self, 
                 joint_min: list[float] = None, 
                 joint_max: list[float] = None):
        # Default CR5 joint limits in radians (from URDF)
        # J1~J5: [-pi, pi], J6: [-2*pi, 2*pi]
        self.joint_min = np.array(joint_min if joint_min is not None else [
            -PI, -PI, -2.86159, -PI, -PI, -2.0 * PI
        ], dtype=np.float64)
        
        self.joint_max = np.array(joint_max if joint_max is not None else [
             PI,  PI,  2.86159,  PI,  PI,  2.0 * PI
        ], dtype=np.float64)

        # Base and Tool conversion matrices to map URDF <-> Dobot Controller Frame
        # T_base = RotZ(180 deg)
        self.T_base = np.array([
            [-1.0,  0.0, 0.0, 0.0],
            [ 0.0, -1.0, 0.0, 0.0],
            [ 0.0,  0.0, 1.0, 0.0],
            [ 0.0,  0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.T_base_inv = self.T_base  # RotZ(180) is self-inverse

        # T_tool: X_t = -Y_u, Y_t = -Z_u, Z_t = X_u
        self.T_tool = np.array([
            [ 0.0, -1.0,  0.0, 0.0],
            [ 0.0,  0.0, -1.0, 0.0],
            [ 1.0,  0.0,  0.0, 0.0],
            [ 0.0,  0.0,  0.0, 1.0]
        ], dtype=np.float64)
        
        # T_tool_inv = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1]
        self.T_tool_inv = np.linalg.inv(self.T_tool)

    # =========================================================================
    # 1. URDF STANDARD INTERFACES (Base -> Link6)
    # =========================================================================

    def forward(self, q_urdf: list[float] | np.ndarray) -> np.ndarray:
        """
        Computes forward kinematics in URDF coordinate frame.
        :param q_urdf: 6 joint angles in radians (URDF base)
        :return: 4x4 Homogeneous transformation matrix (Base -> Link6)
        """
        q_dh = np.array(q_urdf, dtype=np.float64)
        q_dh[1] -= PI / 2.0
        q_dh[3] -= PI / 2.0
        return cr5_ur_kin.forward(q_dh)

    def forward_all(self, q_urdf: list[float] | np.ndarray) -> list[np.ndarray]:
        """
        Computes all link frames T1~T6 in URDF base coordinate frame.
        """
        q_dh = np.array(q_urdf, dtype=np.float64)
        q_dh[1] -= PI / 2.0
        q_dh[3] -= PI / 2.0
        return cr5_ur_kin.forward_all(q_dh)

    def inverse(self, T: np.ndarray, q6_des: float = 0.0) -> list[np.ndarray]:
        """
        Computes analytical inverse kinematics in URDF coordinate frame.
        Returns up to 8 normalized solutions in [-pi, pi].
        :param T: 4x4 Homogeneous transformation matrix (Base -> Link6)
        :param q6_des: desired q6 for singular wrist (default 0.0)
        :return: list of np.ndarray of shape (6,)
        """
        raw_sols = cr5_ur_kin.inverse(T, q6_des=q6_des)
        urdf_sols = []
        for sol in raw_sols:
            q_u = np.array(sol, dtype=np.float64)
            q_u[1] += PI / 2.0
            q_u[3] += PI / 2.0
            
            # Normalize to [-pi, pi]
            for j in range(6):
                while q_u[j] > PI:
                    q_u[j] -= 2.0 * PI
                while q_u[j] < -PI:
                    q_u[j] += 2.0 * PI
            urdf_sols.append(q_u)
        return urdf_sols

    def compute_fk(self, q_urdf: list[float] | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Returns (translation_xyz, rotation_matrix_3x3).
        """
        T = self.forward(q_urdf)
        return T[:3, 3], T[:3, :3]

    def compute_ik(self, translation: np.ndarray, rotation: np.ndarray) -> list[np.ndarray]:
        """
        Computes IK given translation [x, y, z] and rotation [3x3].
        """
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = rotation
        T[:3, 3] = translation
        return self.inverse(T)

    # =========================================================================
    # 2. DOBOT CONTROLLER INTERFACES (Mapped to Real Robot TCP Frame)
    # =========================================================================

    def forward_controller(self, q_urdf: list[float] | np.ndarray) -> tuple[list[float], list[float]]:
        """
        Computes forward kinematics in Dobot controller coordinate system.
        :param q_urdf: 6 joint angles in radians
        :return: (xyz_mm, rpy_deg) where xyz in mm, rpy in ZYX Euler degrees
        """
        T_urdf = self.forward(q_urdf)
        T_ctrl = self.T_base @ T_urdf @ self.T_tool

        xyz_mm = [float(T_ctrl[0, 3] * 1000.0), 
                  float(T_ctrl[1, 3] * 1000.0), 
                  float(T_ctrl[2, 3] * 1000.0)]
        
        R_ctrl = T_ctrl[:3, :3]
        
        # Euler ZYX matching Dobot Controller (rx=roll, ry=pitch, rz=yaw)
        # R = Rz(rz) * Ry(ry) * Rx(rx)
        rpy_rad = R_scipy.from_matrix(R_ctrl).as_euler('xyz', degrees=False)
        # Or explicit formula matching cr5_kinematics.cpp
        # Let's match the exact C++ Euler decomposition:
        beta = math.atan2(-R_ctrl[2, 0], math.sqrt(R_ctrl[0, 0]**2 + R_ctrl[1, 0]**2))
        if abs(math.cos(beta)) > 1e-6:
            alpha = math.atan2(R_ctrl[1, 0] / math.cos(beta), R_ctrl[0, 0] / math.cos(beta))
            gamma = math.atan2(R_ctrl[2, 1] / math.cos(beta), R_ctrl[2, 2] / math.cos(beta))
        else:
            alpha = 0.0
            gamma = math.atan2(R_ctrl[0, 1], R_ctrl[1, 1])

        rx_deg = float(math.degrees(gamma))
        ry_deg = float(math.degrees(beta))
        rz_deg = float(math.degrees(alpha))

        return xyz_mm, [rx_deg, ry_deg, rz_deg]

    def inverse_controller(self, xyz_mm: list[float], rpy_deg: list[float]) -> list[np.ndarray]:
        """
        Computes inverse kinematics from Dobot controller pose.
        :param xyz_mm: [x, y, z] in mm
        :param rpy_deg: [rx, ry, rz] in degrees (Euler ZYX)
        :return: list of valid joint solutions in radians
        """
        rx = math.radians(rpy_deg[0])
        ry = math.radians(rpy_deg[1])
        rz = math.radians(rpy_deg[2])

        R_z = np.array([
            [math.cos(rz), -math.sin(rz), 0],
            [math.sin(rz),  math.cos(rz), 0],
            [0,             0,            1]
        ], dtype=np.float64)

        R_y = np.array([
            [ math.cos(ry), 0, math.sin(ry)],
            [ 0,            1, 0           ],
            [-math.sin(ry), 0, math.cos(ry)]
        ], dtype=np.float64)

        R_x = np.array([
            [1, 0,            0           ],
            [0, math.cos(rx), -math.sin(rx)],
            [0, math.sin(rx),  math.cos(rx)]
        ], dtype=np.float64)

        R_ctrl = R_z @ R_y @ R_x
        
        T_ctrl = np.eye(4, dtype=np.float64)
        T_ctrl[:3, :3] = R_ctrl
        T_ctrl[0, 3] = xyz_mm[0] / 1000.0
        T_ctrl[1, 3] = xyz_mm[1] / 1000.0
        T_ctrl[2, 3] = xyz_mm[2] / 1000.0

        # T_urdf = T_base_inv * T_ctrl * T_tool_inv
        T_urdf = self.T_base_inv @ T_ctrl @ self.T_tool_inv

        return self.inverse(T_urdf)

    # =========================================================================
    # 3. ADVANCED SELECTION & EXPANSION
    # =========================================================================

    def is_joint_valid(self, q: list[float] | np.ndarray, tolerance: float = 1e-4) -> bool:
        """
        Checks if joint angles are within soft limits.
        """
        q_arr = np.array(q)
        return bool(np.all(q_arr >= self.joint_min - tolerance) and 
                    np.all(q_arr <= self.joint_max + tolerance))

    def expand_solutions(self, base_sols: list[np.ndarray]) -> list[np.ndarray]:
        """
        Expands base solutions by searching +-2pi aliases within joint limits.
        Ensures all valid physical configurations in multi-turn joints are found.
        """
        expanded = []

        def _backtrack(sol: np.ndarray, joint_idx: int):
            if joint_idx == 6:
                if self.is_joint_valid(sol):
                    expanded.append(sol.copy())
                return
            
            orig = sol[joint_idx]
            for k in [-1, 0, 1]:
                cand = orig + k * 2.0 * PI
                if (cand >= self.joint_min[joint_idx] - 1e-4 and 
                    cand <= self.joint_max[joint_idx] + 1e-4):
                    sol[joint_idx] = cand
                    _backtrack(sol, joint_idx + 1)
            sol[joint_idx] = orig

        for base_sol in base_sols:
            _backtrack(base_sol.copy(), 0)

        return expanded

    def solve_ik(self, T: np.ndarray, expand: bool = True) -> list[np.ndarray]:
        """
        Full IK solver: computes 8 base analytical solutions and optionally expands +-2pi aliases.
        Filters out out-of-limit solutions.
        """
        base_sols = self.inverse(T)
        if not expand:
            return [s for s in base_sols if self.is_joint_valid(s)]
        return self.expand_solutions(base_sols)

    def get_best_ik(self, 
                    T: np.ndarray, 
                    current_joints: list[float] | np.ndarray, 
                    weights: list[float] = None) -> np.ndarray | None:
        """
        Finds the optimal IK solution closest to current_joints (minimizing weighted distance).
        Prevents axis flips and sudden branch mutations.
        :param T: 4x4 target transformation matrix (Base -> Link6)
        :param current_joints: 6 joint angles of previous/current pose (radians)
        :param weights: optional weight vector of length 6 (e.g. higher weight for base joints)
        :return: optimal np.ndarray (6,) or None if no valid solution
        """
        valid_sols = self.solve_ik(T, expand=True)
        if not valid_sols:
            return None

        curr = np.array(current_joints, dtype=np.float64)
        w = np.array(weights if weights is not None else [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float64)

        best_sol = None
        min_dist = float('inf')

        for sol in valid_sols:
            diff = sol - curr
            dist = float(np.sum(w * (diff ** 2)))
            if dist < min_dist:
                min_dist = dist
                best_sol = sol

        return best_sol

    # =========================================================================
    # 4. KINEMATICS DYNAMICS & SINGULARITY EVALUATION
    # =========================================================================

    def jacobian(self, q_urdf: list[float] | np.ndarray) -> np.ndarray:
        """
        Computes 6x6 Geometric Jacobian matrix in Base coordinate frame.
        J = [J_v; J_w], where v_e = J_v * q_dot, w_e = J_w * q_dot.
        """
        T_chain = self.forward_all(q_urdf)
        # Link origins
        o_0 = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        z_0 = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        
        origins = [o_0] + [T[:3, 3] for T in T_chain]
        z_axes  = [z_0] + [T[:3, 2] for T in T_chain]

        o_e = origins[-1]  # End-effector origin
        J = np.zeros((6, 6), dtype=np.float64)

        for i in range(6):
            z_i = z_axes[i]
            o_i = origins[i]
            p_diff = o_e - o_i
            J[:3, i] = np.cross(z_i, p_diff)
            J[3:, i] = z_i

        return J

    def manipulability(self, q_urdf: list[float] | np.ndarray) -> float:
        """
        Computes Yoshikawa Manipulability Measure w = sqrt(det(J * J^T)).
        When w -> 0, robot is close to a kinematic singularity.
        """
        J = self.jacobian(q_urdf)
        JJT = J @ J.T
        det = np.linalg.det(JJT)
        return float(math.sqrt(max(0.0, det)))

    def check_singularity_risk(self, q_urdf: list[float] | np.ndarray) -> dict:
        """
        Diagnoses specific singularity risks for CR5:
        1. Wrist Singularity: |q5| < 3.0 deg
        2. Elbow Singularity: |q3| < 3.0 deg or |q3 +- 180 deg| < 3.0 deg
        3. Overall Manipulability index w
        """
        q = np.array(q_urdf)
        q5_deg = abs(math.degrees(q[4]))
        q3_deg = abs(math.degrees(q[2]))
        w = self.manipulability(q_urdf)

        is_wrist_sing = (q5_deg < 3.0) or (abs(q5_deg - 180.0) < 3.0)
        is_elbow_sing = (q3_deg < 3.0) or (abs(q3_deg - 180.0) < 3.0)

        return {
            "wrist_singularity": is_wrist_sing,
            "elbow_singularity": is_elbow_sing,
            "wrist_angle_deg": float(q5_deg),
            "elbow_angle_deg": float(q3_deg),
            "manipulability_w": float(w),
            "is_singular": is_wrist_sing or is_elbow_sing or (w < 1e-4)
        }
