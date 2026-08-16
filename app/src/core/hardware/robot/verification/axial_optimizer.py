"""
Axial Tool Rotation Auto-Fix Optimizer (OPT).
Applies spray symmetry free axial spin around tool Z-axis to eliminate kinematic branch jumps,
wrist/elbow singularities, and joint overspeeds.
"""

import copy
import logging
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

from .robot_config import RobotConfig
from .path_interpolator import PathInterpolator, pose_dict_to_matrix, matrix_to_pose_dict
from .kinematic_chain_verifier import KinematicChainVerifier

logger = logging.getLogger(__name__)


class AxialSpinOptimizer:
    """
    Optimizes manual path waypoint orientations using tool symmetry axial spin (Rz).
    """
    def __init__(
        self,
        robot_config: RobotConfig,
        interpolator: PathInterpolator,
        verifier: KinematicChainVerifier
    ):
        self.config = robot_config
        self.interpolator = interpolator
        self.verifier = verifier

    def align_waypoints_smoothly(self, waypoints: list[dict]) -> list[dict]:
        """
        Ensures smooth transition of Euler angles between consecutive waypoints
        to prevent 180° Euler ambiguity flips.
        """
        if len(waypoints) < 2:
            return waypoints

        aligned = [dict(waypoints[0])]
        for i in range(1, len(waypoints)):
            wp = dict(waypoints[i])
            prev_wp = aligned[-1]

            prev_pose = prev_wp.get("tcp_pose_base", prev_wp)
            curr_pose = wp.get("tcp_pose_base", wp)

            prev_rpy = np.array([prev_pose["rx"], prev_pose["ry"], prev_pose["rz"]])
            curr_rpy = np.array([curr_pose["rx"], curr_pose["ry"], curr_pose["rz"]])

            diff = (curr_rpy - prev_rpy + 180.0) % 360.0 - 180.0
            smooth_rpy = prev_rpy + diff

            new_pose = dict(curr_pose)
            new_pose["rx"] = round(float(smooth_rpy[0]), 2)
            new_pose["ry"] = round(float(smooth_rpy[1]), 2)
            new_pose["rz"] = round(float(smooth_rpy[2]), 2)

            if "tcp_pose_base" in wp:
                wp["tcp_pose_base"] = new_pose
            else:
                wp.update(new_pose)

            aligned.append(wp)

        return aligned

    def optimize_single_path(self, path_item: dict, init_q: list[float] = None) -> tuple[dict, bool]:
        """
        Optimizes waypoint orientations for a single path using axial rotation search:
        - Evaluates discrete candidates around gun Z-axis: [0°, ±30°, ±60°, ±90°, ±120°, ±150°, 180°]
        - Finds continuous branch configurations with zero discontinuities and minimal joint motion
        Returns: (optimized_path_item, was_modified)
        """
        orig_waypoints = path_item.get("points", [])
        if len(orig_waypoints) < 1:
            return path_item, False

        # First align orientations smoothly
        smoothed_waypoints = self.align_waypoints_smoothly(orig_waypoints)

        # Baseline verification
        initial_rep = self.verifier.verify_single_path(
            {"path_id": path_item.get("path_id", 1), "points": smoothed_waypoints},
            init_q=init_q
        )
        if initial_rep["status"] == "PASS":
            logger.info(f"✨ [AxialSpinOptimizer] Path {path_item.get('path_id')} already PASSES baseline verification. No orientation modification needed.")
            opt_path = copy.deepcopy(path_item)
            opt_path["points"] = smoothed_waypoints
            return opt_path, False

        logger.info(f"⚙️ [AxialSpinOptimizer] Path {path_item.get('path_id')} has status={initial_rep['status']} (Issues={len(initial_rep['issues'])}). Starting axial spin search...")

        # Search candidates: finer 15° resolution
        axial_angles_deg = [0.0]
        for a in range(15, 181, 15):
            axial_angles_deg.append(float(a))
            axial_angles_deg.append(float(-a))

        opt_waypoints = []
        curr_q = init_q
        path_modified = False

        for wp_idx, wp in enumerate(smoothed_waypoints):
            orig_pose = wp.get("tcp_pose_base", wp)
            T_orig_gun = pose_dict_to_matrix(orig_pose)

            best_T_gun = None
            best_q = None
            best_score = float('inf')

            for rz_deg in axial_angles_deg:
                # Apply axial rotation around gun local Z axis
                R_local_spin = R_scipy.from_euler('z', rz_deg, degrees=True).as_matrix()
                T_cand_gun = np.copy(T_orig_gun)
                T_cand_gun[:3, :3] = T_orig_gun[:3, :3] @ R_local_spin

                # Always use the controller matrix (T_cand_gun) for inverse kinematics
                ik_sols = self.verifier.solver.inverse_controller_matrix(T_cand_gun)

                if not ik_sols:
                    continue

                for sol in ik_sols:
                    if curr_q is not None:
                        d = sol - np.array(curr_q)
                        d = (d + np.pi) % (2 * np.pi) - np.pi
                        dq_norm = np.linalg.norm(d)
                        max_dq_deg = np.max(np.abs(np.degrees(d)))
                    else:
                        # Seed selection: prefer natural non-singular workspace branch close to Home
                        home_q = np.array([np.pi, 0.0, np.pi/2.0, np.pi/2.0, np.pi/2.0, 0.0])
                        d = sol - home_q
                        d = (d + np.pi) % (2 * np.pi) - np.pi
                        dq_norm = np.linalg.norm(d)
                        max_dq_deg = 0.0

                    # Singularity penalties
                    sin_theta3 = abs(np.sin(sol[2]))
                    sin_theta5 = abs(np.sin(sol[4]))

                    singularity_penalty = 0.0
                    if sin_theta3 < 0.08:
                        singularity_penalty += (0.08 - sin_theta3) * 50.0
                    if sin_theta5 < 0.08:
                        singularity_penalty += (0.08 - sin_theta5) * 50.0

                    # Prefer smaller axial changes
                    spin_penalty = abs(rz_deg) * 0.005

                    score = dq_norm + singularity_penalty + spin_penalty
                    if max_dq_deg > 60.0:
                        score += 100.0

                    if score < best_score:
                        best_score = score
                        best_T_gun = T_cand_gun
                        best_q = sol

            if best_T_gun is not None:
                new_wp = copy.deepcopy(wp)
                new_pose = matrix_to_pose_dict(best_T_gun)
                if "tcp_pose_base" in new_wp:
                    new_wp["tcp_pose_base"] = new_pose
                else:
                    new_wp.update(new_pose)

                opt_waypoints.append(new_wp)
                curr_q = best_q
                path_modified = True
            else:
                logger.warning(f"⚠️ [AxialSpinOptimizer] Waypoint #{wp_idx+1} could not find feasible orientation. Retaining original.")
                opt_waypoints.append(wp)

        # Smooth Euler angles on the resulting optimized waypoints
        opt_waypoints = self.align_waypoints_smoothly(opt_waypoints)

        opt_path = copy.deepcopy(path_item)
        opt_path["points"] = opt_waypoints
        return opt_path, path_modified

    def optimize_all_paths(self, paths_data: dict) -> tuple[dict, dict]:
        """
        Optimizes all manual paths in paths_data and returns:
        (optimized_paths_data, verification_report_of_optimized_paths)
        """
        paths = paths_data.get("paths", [])
        opt_paths = []
        last_q = None

        for path in paths:
            opt_path, _ = self.optimize_single_path(path, init_q=last_q)
            opt_paths.append(opt_path)

            rep = self.verifier.verify_single_path(opt_path, init_q=last_q)
            if rep.get("trajectory_q"):
                last_q = rep["trajectory_q"][-1]

        opt_data = copy.deepcopy(paths_data)
        opt_data["paths"] = opt_paths
        opt_data["type"] = "opt"

        opt_report = self.verifier.verify_all_paths(opt_data)
        opt_report["state_type"] = "opt"
        opt_report["optimized_paths_available"] = True
        return opt_data, opt_report
