"""
POI Pose Constraint Optimizer (POI).
Optimizes waypoint orientations relative to an anchor reference pose within a bounded 3D tolerance envelope
[±ΔRx, ±ΔRy, ±ΔRz] with adaptive feedrate scaling to satisfy physical spray cone consistency.
"""

import copy
import logging
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

from .robot_config import RobotConfig
from .path_interpolator import PathInterpolator, pose_dict_to_matrix, matrix_to_pose_dict
from .kinematic_chain_verifier import KinematicChainVerifier

logger = logging.getLogger(__name__)


class PoiConstraintOptimizer:
    """
    Optimizes manual path waypoint orientations using anchor reference pose and tolerance envelope constraints.
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

    def optimize_poi_single_path(
        self,
        path_item: dict,
        ref_rpy_deg: list[float] = None,
        tolerance_rpy_deg: list[float] = None,
        tol_rpy_deg: list[float] = None,
        init_q: list[float] = None
    ) -> tuple[dict, bool]:
        """
        Optimizes a single path within a bounded 3D tolerance envelope around a reference anchor pose:
        - ref_rpy_deg: [ref_rx, ref_ry, ref_rz] anchor reference orientation in robot base frame
        - tolerance_rpy_deg / tol_rpy_deg: [tol_rx, tol_ry, tol_rz] permissible variation envelope in degrees
        - Evaluates candidates within the tolerance envelope to eliminate singularities and minimize joint speed spikes
        - Performs adaptive feedrate scaling if joint overspeed persists
        Returns: (poi_optimized_path, was_modified)
        """
        orig_waypoints = path_item.get("points", [])
        if len(orig_waypoints) < 1:
            return path_item, False

        effective_tol = tolerance_rpy_deg or tol_rpy_deg or [3.0, 15.0, 180.0]
        tol = effective_tol if len(effective_tol) == 3 else [3.0, 15.0, 180.0]
        tol_rx, tol_ry, tol_rz = float(tol[0]), float(tol[1]), float(tol[2])

        # Anchor reference matrix
        if ref_rpy_deg and len(ref_rpy_deg) == 3:
            R_ref = R_scipy.from_euler('xyz', ref_rpy_deg, degrees=True).as_matrix()
        else:
            # Fallback to first waypoint's orientation as anchor reference
            first_pose = orig_waypoints[0].get("tcp_pose_base", orig_waypoints[0])
            first_rpy = [first_pose.get("rx", 0.0), first_pose.get("ry", 0.0), first_pose.get("rz", 0.0)]
            R_ref = R_scipy.from_euler('xyz', first_rpy, degrees=True).as_matrix()

        # Build candidate search grid within tolerance envelope
        rx_steps = np.linspace(-tol_rx, tol_rx, num=max(1, int(tol_rx * 2 + 1))) if tol_rx > 0 else [0.0]
        ry_steps = np.linspace(-tol_ry, tol_ry, num=max(1, int(tol_ry / 3 + 1))) if tol_ry > 0 else [0.0]

        if tol_rz >= 170.0:
            rz_steps = np.linspace(-180.0, 180.0, num=25)  # 15° steps
        elif tol_rz > 0:
            rz_steps = np.linspace(-tol_rz, tol_rz, num=max(1, int(tol_rz / 10 + 1)))
        else:
            rz_steps = [0.0]

        logger.info(f"🎯 [PoiConstraintOptimizer] Path {path_item.get('path_id')} starting POI search with envelope: Tol_Rx=±{tol_rx}°, Tol_Ry=±{tol_ry}°, Tol_Rz=±{tol_rz}° (Grid size={len(rx_steps)*len(ry_steps)*len(rz_steps)})...")

        opt_waypoints = []
        curr_q = init_q
        path_modified = False

        for wp_idx, wp in enumerate(orig_waypoints):
            orig_pose = wp.get("tcp_pose_base", wp)
            T_orig_gun = pose_dict_to_matrix(orig_pose)
            pos_m = T_orig_gun[:3, 3]

            best_T_gun = None
            best_q = None
            best_score = float('inf')
            best_normal_angle_deg = None
            surface_normal = np.array(wp.get("surface_normal_base", [0.0, 0.0, 1.0]), dtype=np.float64)
            n_norm = np.linalg.norm(surface_normal)
            if n_norm > 1e-6:
                surface_normal = surface_normal / n_norm
            else:
                surface_normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            desired_tool_z = -surface_normal
            for drx in rx_steps:
                for dry in ry_steps:
                    for drz in rz_steps:
                        R_delta = R_scipy.from_euler('xyz', [drx, dry, drz], degrees=True).as_matrix()
                        R_cand = R_ref @ R_delta

                        T_cand_gun = np.eye(4, dtype=np.float64)
                        T_cand_gun[:3, :3] = R_cand
                        T_cand_gun[:3, 3] = pos_m

                        T_cand_flange = T_cand_gun @ self.config.T_tcp_inv
                        ik_sols = self.verifier.solver.inverse(T_cand_flange)

                        if not ik_sols:
                            continue

                        for sol in ik_sols:
                            if curr_q is not None:
                                d = sol - np.array(curr_q)
                                d = (d + np.pi) % (2 * np.pi) - np.pi
                                dq_norm = np.linalg.norm(d)
                                max_dq_deg = np.max(np.abs(np.degrees(d)))
                            else:
                                dq_norm = np.linalg.norm(sol)
                                max_dq_deg = 0.0

                            # Singularity penalties
                            sin_theta3 = abs(np.sin(sol[2]))
                            sin_theta5 = abs(np.sin(sol[4]))

                            singularity_penalty = 0.0
                            if sin_theta3 < 0.08:
                                singularity_penalty += (0.08 - sin_theta3) * 60.0
                            if sin_theta5 < 0.08:
                                singularity_penalty += (0.08 - sin_theta5) * 60.0

                            # Prefer the absolute-anchor candidate whose tool Z axis is closest to the local surface normal direction.
                            tool_z = R_cand[:, 2]
                            cos_align = float(np.clip(np.dot(tool_z, desired_tool_z), -1.0, 1.0))
                            normal_angle_deg = float(np.degrees(np.arccos(cos_align)))
                            normal_alignment_penalty = normal_angle_deg * 0.08

                            # Proximity penalty to anchor reference
                            delta_penalty = (abs(drx) * 0.1 + abs(dry) * 0.05 + abs(drz) * 0.002)

                            score = dq_norm + singularity_penalty + delta_penalty + normal_alignment_penalty
                            if max_dq_deg > 50.0:
                                score += 150.0

                            if score < best_score:
                                best_score = score
                                best_T_gun = T_cand_gun
                                best_q = sol
                                best_normal_angle_deg = normal_angle_deg

            if best_T_gun is not None:
                new_wp = copy.deepcopy(wp)
                new_pose = matrix_to_pose_dict(best_T_gun)
                if "tcp_pose_base" in new_wp:
                    new_wp["tcp_pose_base"] = new_pose
                else:
                    new_wp.update(new_pose)

                new_wp["poi_alignment"] = {
                    "surface_normal_angle_deg": round(float(best_normal_angle_deg or 0.0), 2),
                    "score": round(float(best_score), 4),
                    "model": "absolute_anchor_tolerance_closest_to_surface_normal",
                }

                opt_waypoints.append(new_wp)
                curr_q = best_q
                path_modified = True
            else:
                logger.warning(f"⚠️ [PoiConstraintOptimizer] Waypoint #{wp_idx+1} POI constraint unresolvable in envelope. Retaining original.")
                opt_waypoints.append(wp)

        # Adaptive feedrate check: if overspeed issues remain, automatically lower recommended safe velocity
        poi_path = copy.deepcopy(path_item)
        poi_path["points"] = opt_waypoints

        rep = self.verifier.verify_single_path(poi_path, init_q=init_q)
        if rep.get("recommended_safe_speed_mm_s") and rep["recommended_safe_speed_mm_s"] < self.interpolator.linear_velocity_mm_s:
            poi_path["recommended_speed_mm_s"] = rep["recommended_safe_speed_mm_s"]
            logger.info(f"⚡ [PoiConstraintOptimizer] Path {path_item.get('path_id')} tuned feedrate: {self.interpolator.linear_velocity_mm_s} -> {rep['recommended_safe_speed_mm_s']} mm/s")

        return poi_path, path_modified

    def optimize_poi_all_paths(
        self,
        paths_data: dict,
        ref_rpy_deg: list[float] = None,
        tolerance_rpy_deg: list[float] = None,
        tol_rpy_deg: list[float] = None
    ) -> tuple[dict, dict]:
        """
        Optimizes all manual paths in paths_data using POI constraint envelope search.
        Returns: (poi_paths_data, poi_verification_report)
        """
        effective_tol = tolerance_rpy_deg or tol_rpy_deg or [3.0, 15.0, 180.0]
        if len(effective_tol) != 3:
            effective_tol = [3.0, 15.0, 180.0]
        effective_tol = [float(v) for v in effective_tol]
        paths = paths_data.get("paths", [])
        poi_paths = []
        last_q = None

        for path in paths:
            opt_path, _ = self.optimize_poi_single_path(
                path,
                ref_rpy_deg=ref_rpy_deg,
                tolerance_rpy_deg=effective_tol,
                init_q=last_q
            )
            poi_paths.append(opt_path)

            rep = self.verifier.verify_single_path(opt_path, init_q=last_q)
            if rep.get("trajectory_q"):
                last_q = rep["trajectory_q"][-1]

        poi_data = copy.deepcopy(paths_data)
        poi_data["paths"] = poi_paths
        poi_data["type"] = "poi"
        poi_data["poi_config"] = {
            "mode": "absolute_anchor_tolerance",
            "ref_rpy_deg": ref_rpy_deg,
            "tolerance_rpy_deg": effective_tol,
            "euler_order": "xyz",
            "units": "deg",
        }

        poi_report = self.verifier.verify_all_paths(poi_data)
        poi_report["state_type"] = "poi"
        poi_report["optimized_paths_available"] = True
        poi_report["poi_config"] = poi_data["poi_config"]
        return poi_data, poi_report
