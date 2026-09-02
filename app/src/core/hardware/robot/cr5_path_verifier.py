"""
CR5 Offline Chain Path Verifier & Tolerance-based Auto-Fix Optimizer Facade.
Coordinates modular verification, interpolation, and optimization subsystems:
- RobotConfig: URDF joint limits, velocities, tool link TCP offsets
- PathInterpolator: Cartesian MoveL linear & Slerp interpolation
- KinematicChainVerifier: Continuous IK branch tracking, singularity diagnosis, and trajectory recording
- AxialSpinOptimizer: Free axial tool rotation (OPT) auto-fix
- PoiConstraintOptimizer: Bounded tolerance envelope spray-cone constraint optimization (POI)
- SprayWaypointOptimizer: Sparse Viterbi attitude search for controller-native MoveL
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
    SprayWaypointOptimizer,
)

logger = logging.getLogger(__name__)


class CR5PathVerifier:
    """
    Unified facade for CR5 path kinematics verification and trajectory optimization.

    :param kinematics_backend: passed to CR5Kinematics — "python", "cpp", or "auto"
        (prefer libur_kin for dense MoveL IK when the shared library is built).
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
        max_joint_vel_deg_s: list[float] = None,
        kinematics_backend: str = "auto",
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

        # 2. Kinematics Solver ("auto" prefers libur_kin when the C++ lib is built)
        self.solver = CR5Kinematics(
            joint_min=self.config.joint_min_rad.tolist(),
            joint_max=self.config.joint_max_rad.tolist(),
            backend=kinematics_backend,
        )

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

        # 7. Sparse Viterbi MoveL optimizer (path_opt)
        self.spray_optimizer = SprayWaypointOptimizer(
            solver=self.solver,
            verifier=self.verifier,
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
        """Optimizes a single path within a bounded 3D tolerance envelope using PoiConstraintOptimizer (Viterbi DP)."""
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
        tol_rpy_deg: list[float] = None,
        init_q: list[float] = None,
        anchor_source: str = "home",
    ):
        """Optimizes all paths within a bounded 3D tolerance envelope using PoiConstraintOptimizer (Viterbi DP).

        :param anchor_source: 'config'/'home' → 全局单一锚点；'raw' → 逐点名义法向锚点
        """
        return self.poi_optimizer.optimize_poi_all_paths(
            paths_data,
            ref_rpy_deg=ref_rpy_deg,
            tolerance_rpy_deg=tolerance_rpy_deg,
            tol_rpy_deg=tol_rpy_deg,
            init_q=init_q,
            anchor_source=anchor_source
        )

    # ─── Sparse Viterbi MoveL Optimizer ─────────────────────────────────────
    def optimize_spray_path(
        self,
        path_item: dict,
        init_q: list[float] = None,
        ref_rpy_deg: list[float] = None,
        tolerance_rpy_deg: list[float] = None,
    ):
        """
        稀疏 Waypoint 姿态全局优化（工具系网格 + 锚点包络 + MoveL 抽检 + Viterbi）。
        输出仍是稀疏 tcp_pose_base，供控制器原生 MoveL，不生成密关节轨。

        :param path_item: 单条路径 {"points": [{"tcp_pose_base": {x,y,z,rx,ry,rz}}, ...]}
        :param init_q: 起始关节，弧度
        :param ref_rpy_deg: 锚点参考姿态 [rx, ry, rz]（度）
        :param tolerance_rpy_deg: 锚点硬包络 [tol_rx, tol_ry, tol_rz]（度）
        :return: (optimized_path_item, was_modified)
        """
        return self.spray_optimizer.optimize_path_item(
            path_item,
            init_q=init_q,
            ref_rpy_deg=ref_rpy_deg,
            tolerance_rpy_deg=tolerance_rpy_deg,
        )

    def optimize_spray_all_paths(
        self,
        paths_data: dict,
        init_q: list[float] = None,
        ref_rpy_deg: list[float] = None,
        tolerance_rpy_deg: list[float] = None,
    ):
        """
        对 paths_data 中每条路径做稀疏 Viterbi 选姿，路径之间用上一终点关节衔接，
        最后用密采样校验器出总报告。
        """
        return self.spray_optimizer.optimize_all_paths(
            paths_data,
            init_q=init_q,
            ref_rpy_deg=ref_rpy_deg,
            tolerance_rpy_deg=tolerance_rpy_deg,
        )
