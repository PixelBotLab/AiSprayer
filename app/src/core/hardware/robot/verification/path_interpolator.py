"""
Cartesian MoveL Trajectory Interpolator.
Handles linear Cartesian position interpolation and Slerp spherical quaternion orientation interpolation
between discrete waypoints, decoupling gun poses to flange poses via T_tcp_inv.
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy, Slerp
from .robot_config import RobotConfig


def pose_dict_to_matrix(pose_dict: dict) -> np.ndarray:
    """
    Converts a pose dictionary {"x": mm, "y": mm, "z": mm, "rx": deg, "ry": deg, "rz": deg}
    to a 4x4 homogeneous transformation matrix T (meters).
    """
    x = float(pose_dict["x"]) / 1000.0
    y = float(pose_dict["y"]) / 1000.0
    z = float(pose_dict["z"]) / 1000.0
    rx = float(pose_dict.get("rx", 0.0))
    ry = float(pose_dict.get("ry", 0.0))
    rz = float(pose_dict.get("rz", 0.0))

    r_mat = R_scipy.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = r_mat
    T[:3, 3] = [x, y, z]
    return T


def matrix_to_pose_dict(T: np.ndarray) -> dict:
    """
    Converts a 4x4 homogeneous transformation matrix T (meters)
    back to a pose dictionary {"x": mm, "y": mm, "z": mm, "rx": deg, "ry": deg, "rz": deg}.
    """
    x = float(T[0, 3]) * 1000.0
    y = float(T[1, 3]) * 1000.0
    z = float(T[2, 3]) * 1000.0
    rpy = R_scipy.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    return {
        "x": round(x, 2),
        "y": round(y, 2),
        "z": round(z, 2),
        "rx": round(float(rpy[0]), 2),
        "ry": round(float(rpy[1]), 2),
        "rz": round(float(rpy[2]), 2),
    }


class PathInterpolator:
    """
    Interpolates continuous Cartesian trajectories between discrete waypoints.
    """
    def __init__(
        self,
        robot_config: RobotConfig,
        step_size_mm: float = 1.5,
        linear_velocity_mm_s: float = 120.0
    ):
        self.config = robot_config
        self.step_size_mm = step_size_mm
        self.linear_velocity_mm_s = linear_velocity_mm_s

    def interpolate_path_dense(
        self,
        waypoints: list[dict],
        speed_override_mm_s: float = None
    ) -> list[tuple[np.ndarray, np.ndarray, float, int]]:
        """
        Subdivides the waypoint list into dense MoveL Cartesian steps.
        Returns: list of (T_gun, T_flange, dt_sec, segment_index)
        """
        if len(waypoints) < 2:
            dense_points = []
            for i, wp in enumerate(waypoints):
                pose = wp.get("tcp_pose_base", wp)
                T_gun = pose_dict_to_matrix(pose)
                T_flange = T_gun @ self.config.T_tcp_inv
                dense_points.append((T_gun, T_flange, 0.05, 0))
            return dense_points

        effective_speed = speed_override_mm_s if speed_override_mm_s else self.linear_velocity_mm_s
        step_size_m = self.step_size_mm / 1000.0
        dense_points = []

        for seg_idx in range(len(waypoints) - 1):
            wp_start = waypoints[seg_idx]
            wp_end = waypoints[seg_idx + 1]

            p_start = wp_start.get("tcp_pose_base", wp_start)
            p_end = wp_end.get("tcp_pose_base", wp_end)

            T_start = pose_dict_to_matrix(p_start)
            T_end = pose_dict_to_matrix(p_end)

            pos_start = T_start[:3, 3]
            pos_end = T_end[:3, 3]
            dist_m = float(np.linalg.norm(pos_end - pos_start))

            num_steps = max(1, int(math.ceil(dist_m / step_size_m)))
            seg_duration = max(0.001, dist_m / (effective_speed / 1000.0))
            dt = seg_duration / float(num_steps)

            times = [0.0, 1.0]
            rotations = R_scipy.from_matrix([T_start[:3, :3], T_end[:3, :3]])
            slerp = Slerp(times, rotations)

            is_jump = bool(wp_end.get("is_jump", False) or wp_end.get("spraying") == "off")
            for step in range(num_steps):
                t = step / float(num_steps)
                interp_pos = (1.0 - t) * pos_start + t * pos_end
                interp_rot = slerp([t]).as_matrix()[0]

                T_gun_interp = np.eye(4, dtype=np.float64)
                T_gun_interp[:3, :3] = interp_rot
                T_gun_interp[:3, 3] = interp_pos

                T_flange_interp = T_gun_interp @ self.config.T_tcp_inv
                dense_points.append((T_gun_interp, T_flange_interp, dt, seg_idx, is_jump))

        # Add the final endpoint
        wp_final = waypoints[-1]
        p_final = wp_final.get("tcp_pose_base", wp_final)
        T_gun_final = pose_dict_to_matrix(p_final)
        T_flange_final = T_gun_final @ self.config.T_tcp_inv
        dense_points.append((T_gun_final, T_flange_final, 0.05, len(waypoints) - 2, False))

        return dense_points
