"""
CR5 Offline Chain Path Verifier & Tolerance-based Auto-Fix Optimizer Facade.
Coordinates modular verification, interpolation, and optimization subsystems:
- RobotConfig: URDF joint limits, velocities, tool link TCP offsets
- PathInterpolator: Cartesian MoveL linear & Slerp interpolation
- KinematicChainVerifier: Continuous IK branch tracking, singularity diagnosis, and trajectory recording
- AxialSpinOptimizer: Free axial tool rotation (OPT) auto-fix
- PoiConstraintOptimizer: Bounded tolerance envelope spray-cone constraint optimization (POI)
"""

import logging
from typing import Optional, List, Dict, Any

from .cr5_kinematics import CR5Kinematics
from .verification import (
    get_configured_robot_config,
    load_limits_from_urdf,
    load_tcp_from_urdf,
    RobotConfig,
    pose_dict_to_matrix,
    matrix_to_pose_dict,
    PathInterpolator,
    KinematicChainVerifier,
    AxialSpinOptimizer,
    PoiConstraintOptimizer,
)

logger = logging.getLogger(__name__)


class CR5PathVerifier:
    """
    Unified Facade coordinator for CR5 path kinematics verification and multi-mode trajectory optimization.
    """
    @classmethod
    def load_limits_from_urdf(cls, urdf_path: str = None) -> dict:
        """Parses joint limit angles (deg) and max velocity (deg/s) from URDF XML."""
        return load_limits_from_urdf(urdf_path)

    @classmethod
    def load_tcp_from_urdf(cls, urdf_path: str = None, target_tcp_name: str = None) -> dict:
        """Parses TCP offset (XYZ mm, RPY deg) from URDF attached to Link6."""
        return load_tcp_from_urdf(urdf_path, target_tcp_name)

    def __init__(
        self,
        urdf_path: str = None,
        step_size_mm: float = 1.5,
        linear_velocity_mm_s: float = 120.0,
        max_joint_vel_deg_s: list[float] = None
    ):
        self.step_size_mm = step_size_mm
        self.linear_velocity_mm_s = linear_velocity_mm_s

        # 1. Configuration & Tool TCP Subsystem
        self.config = RobotConfig(
            urdf_path=urdf_path,
            max_joint_vel_deg_s=max_joint_vel_deg_s
        )
        self.urdf_info = self.config.urdf_info
        self.urdf_tcp = self.config.urdf_tcp
        self.max_joint_vel_deg_s = self.config.max_joint_vel_deg_s

        # 2. Kinematics Solver
        self.solver = CR5Kinematics()

        # 3. Cartesian Trajectory Interpolator Subsystem
        self.interpolator = PathInterpolator(
            robot_config=self.config,
            step_size_mm=self.step_size_mm,
            linear_velocity_mm_s=self.linear_velocity_mm_s
        )

        # 4. Kinematic Chain Verifier Subsystem
        self.verifier = KinematicChainVerifier(
            robot_config=self.config,
            interpolator=self.interpolator,
            kinematics_solver=self.solver
        )

        # 5. Axial Tool Rotation Auto-Fix Optimizer Subsystem (OPT)
        self.axial_optimizer = AxialSpinOptimizer(
            robot_config=self.config,
            interpolator=self.interpolator,
            verifier=self.verifier
        )

        # 6. POI Pose Constraint & Tolerance Envelope Optimizer Subsystem (POI)
        self.poi_optimizer = PoiConstraintOptimizer(
            robot_config=self.config,
            interpolator=self.interpolator,
            verifier=self.verifier
        )

    # ─── TCP Offset Configuration ───────────────────────────────────────────
    def set_tcp_offset(self, xyz_mm: list[float], rpy_deg: list[float]):
        """Sets the active Tool TCP offset relative to flange."""
        self.config.set_tcp_offset(xyz_mm, rpy_deg)
        self.urdf_tcp = self.config.urdf_tcp

    # ─── Matrix & Interpolation Helpers ─────────────────────────────────────
    def pose_dict_to_matrix(self, pose_dict: dict):
        return pose_dict_to_matrix(pose_dict)

    def matrix_to_pose_dict(self, T):
        return matrix_to_pose_dict(T)

    def interpolate_path_dense(self, waypoints: list[dict], speed_override_mm_s: float = None):
        return self.interpolator.interpolate_path_dense(waypoints, speed_override_mm_s)

    def align_waypoints_smoothly(self, waypoints: list[dict]):
        return self.axial_optimizer.align_waypoints_smoothly(waypoints)

    # ─── Verification APIs ──────────────────────────────────────────────────
    def verify_single_path(self, path_item: dict, init_q: list[float] = None) -> dict:
        """Verifies kinematic feasibility of a single path."""
        return self.verifier.verify_single_path(path_item, init_q=init_q)

    def verify_all_paths(self, paths_data: dict) -> dict:
        """Verifies all paths in paths_data dictionary."""
        return self.verifier.verify_all_paths(paths_data)

    # ─── Axial Optimization APIs (OPT) ──────────────────────────────────────
    def optimize_single_path(self, path_item: dict, init_q: list[float] = None):
        """Optimizes a single path using axial tool rotation (Rz) auto-fix."""
        return self.axial_optimizer.optimize_single_path(path_item, init_q=init_q)

    def optimize_all_paths(self, paths_data: dict):
        """Optimizes all paths using axial tool rotation (Rz) auto-fix."""
        return self.axial_optimizer.optimize_all_paths(paths_data)

    # ─── POI Constraint Optimization APIs (POI) ─────────────────────────────
    def optimize_poi_single_path(
        self,
        path_item: dict,
        ref_rpy_deg: list[float] = None,
        tolerance_rpy_deg: list[float] = None,
        tol_rpy_deg: list[float] = None,
        init_q: list[float] = None
    ):
        """Optimizes a single path within a bounded 3D tolerance envelope around an anchor pose."""
        return self.poi_optimizer.optimize_poi_single_path(
            path_item,
            ref_rpy_deg=ref_rpy_deg,
            tolerance_rpy_deg=tolerance_rpy_deg,
            tol_rpy_deg=tol_rpy_deg,
            init_q=init_q
        )

    def optimize_poi_all_paths(
        self,
        paths_data: dict,
        ref_rpy_deg: list[float] = None,
        tolerance_rpy_deg: list[float] = None,
        tol_rpy_deg: list[float] = None
    ):
        """Optimizes all paths within a bounded 3D tolerance envelope around an anchor pose."""
        return self.poi_optimizer.optimize_poi_all_paths(
            paths_data,
            ref_rpy_deg=ref_rpy_deg,
            tolerance_rpy_deg=tolerance_rpy_deg,
            tol_rpy_deg=tol_rpy_deg
        )
