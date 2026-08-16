"""
Kinematic Chain Path Verifier for Dobot CR5.
Performs offline Cartesian continuous inverse kinematics tracking, velocity profiling,
singularity diagnosis (elbow/wrist), unreachable point detection, and records dense simulation trajectories.
"""

import math
import logging
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

from ..cr5_kinematics import CR5Kinematics
from .robot_config import RobotConfig
from .path_interpolator import PathInterpolator

logger = logging.getLogger(__name__)
PI = math.pi


class KinematicChainVerifier:
    """
    Simulates continuous MoveL Cartesian execution along robot kinematic chain.
    """
    def __init__(
        self,
        robot_config: RobotConfig,
        interpolator: PathInterpolator,
        kinematics_solver: CR5Kinematics = None
    ):
        self.config = robot_config
        self.interpolator = interpolator
        self.solver = kinematics_solver or CR5Kinematics()

    def verify_single_path(self, path_item: dict, init_q: list[float] = None) -> dict:
        """
        Verifies kinematic feasibility of a single path item:
        - Interpolates dense MoveL Cartesian steps
        - Solves continuous inverse kinematics
        - Checks joint angle limits and joint velocities against URDF limits
        - Diagnoses elbow / wrist singularities and unreachable steps
        - Records trajectory_q and trajectory_tcp for 60 FPS simulation playback
        """
        path_id = path_item.get("path_id", 1)
        name = path_item.get("name", f"Path {path_id}")
        waypoints = path_item.get("points", [])

        if not waypoints:
            return {
                "path_id": path_id,
                "name": name,
                "status": "FAILED",
                "total_interpolated": 0,
                "issues": [{"severity": "ERROR", "type": "EMPTY_PATH", "detail": "Path contains 0 waypoints"}],
                "max_joint_velocities_deg_s": [0.0] * 6,
                "peak_joint_speeds_deg_s": [0.0] * 6,
                "trajectory_q": [],
                "trajectory_tcp": []
            }

        dense_points = self.interpolator.interpolate_path_dense(waypoints)
        total_steps = len(dense_points)
        issues = []
        joint_velocities_deg_s = []
        trajectory_q = []

        # 1. Solve initial waypoint seed configuration
        # Pass T_gun (T_ctrl) directly to inverse_controller_matrix to avoid double-application of base/tool transforms
        first_T_gun = dense_points[0][0]
        ik_sols_first = self.solver.inverse_controller_matrix(first_T_gun)

        if not ik_sols_first:
            loc_xyz = [round(float(x), 2) for x in dense_points[0][0][:3, 3] * 1000.0]
            logger.error(f"❌ [KinematicChainVerifier] Path {path_id} Start Point Unreachable: XYZ={loc_xyz}")
            return {
                "path_id": path_id,
                "name": name,
                "status": "FAILED",
                "total_interpolated": total_steps,
                "issues": [{
                    "severity": "ERROR",
                    "type": "UNREACHABLE_START",
                    "detail": "Start waypoint has no valid IK solutions (out of robot workspace).",
                    "location_xyz": loc_xyz,
                    "step_index": 0,
                    "segment_index": 0
                }],
                "max_joint_velocities_deg_s": [0.0] * 6,
                "peak_joint_speeds_deg_s": [0.0] * 6,
                "trajectory_q": [],
                "trajectory_tcp": []
            }

        # Choose seed configuration closest to init_q (or standard home posture) with angular difference unwrapping
        default_seed_q = np.array([PI, 0.0, PI / 2.0, PI / 2.0, PI / 2.0, 0.0], dtype=np.float64)
        q_ref = np.array(init_q, dtype=np.float64) if init_q is not None else default_seed_q
        
        branch_diffs = []
        for sol in ik_sols_first:
            d = (sol - q_ref + PI) % (2 * PI) - PI
            branch_diffs.append(np.linalg.norm(d))
        
        best_idx = int(np.argmin(branch_diffs))
        curr_q = ik_sols_first[best_idx]

        trajectory_q.append(curr_q.tolist())

        # 2. Continuous IK chain tracking across all steps
        for step_idx in range(1, total_steps):
            T_gun, T_flange, dt, seg_idx = dense_points[step_idx]

            # Always use the controller matrix (T_gun) for inverse kinematics
            ik_sols = self.solver.inverse_controller_matrix(T_gun)
            if not ik_sols:
                loc_xyz = [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]
                logger.error(f"❌ [KinematicChainVerifier] Path {path_id} [UNREACHABLE]: Step {step_idx}/{total_steps} (Seg {seg_idx}) at XYZ={loc_xyz}")
                issues.append({
                    "severity": "ERROR",
                    "type": "UNREACHABLE_STEP",
                    "detail": f"Step {step_idx} (segment {seg_idx}) has no analytical IK solutions.",
                    "step_index": step_idx,
                    "segment_index": seg_idx,
                    "location_xyz": loc_xyz
                })
                break

            # Find nearest IK branch solution to minimize joint movement
            diffs = []
            for sol in ik_sols:
                d = sol - curr_q
                # Unwrap angular differences to [-pi, pi]
                d = (d + PI) % (2 * PI) - PI
                diffs.append(np.linalg.norm(d))

            best_sol_idx = int(np.argmin(diffs))
            raw_next_q = ik_sols[best_sol_idx]

            # Continuous branch unwrapping
            dq = (raw_next_q - curr_q + PI) % (2 * PI) - PI
            next_q = curr_q + dq

            # Check kinematic branch discontinuity (> 45 deg in single step)
            if np.max(np.abs(np.degrees(dq))) > 45.0:
                loc_xyz = [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]
                logger.warning(f"⚠️ [KinematicChainVerifier] Path {path_id} [DISCONTINUITY]: Step {step_idx} jump={np.max(np.abs(np.degrees(dq))):.1f}° at XYZ={loc_xyz}")
                issues.append({
                    "severity": "ERROR",
                    "type": "KINEMATIC_DISCONTINUITY",
                    "segment_index": seg_idx,
                    "step_index": step_idx,
                    "detail": f"Branch jump detected (max Δq={np.max(np.abs(np.degrees(dq))):.1f}°). Near singularity or joint limit.",
                    "location_xyz": loc_xyz
                })

            # Check singularity proximity (Elbow / Wrist)
            theta3 = next_q[2]
            theta5 = next_q[4]
            is_near_singularity = False

            if abs(math.sin(theta3)) < 0.05:
                is_near_singularity = True
                loc_xyz = [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]
                issues.append({
                    "type": "ELBOW_SINGULARITY",
                    "severity": "WARNING",
                    "segment_index": seg_idx,
                    "step_index": step_idx,
                    "detail": f"Near elbow singularity: Joint 3 angle={math.degrees(theta3):.1f}° (near 0°/180° collinear stretch).",
                    "location_xyz": loc_xyz
                })

            if abs(math.sin(theta5)) < 0.05:
                is_near_singularity = True
                loc_xyz = [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]
                issues.append({
                    "type": "WRIST_SINGULARITY",
                    "severity": "WARNING",
                    "segment_index": seg_idx,
                    "step_index": step_idx,
                    "detail": f"Near wrist singularity: Joint 5 angle={math.degrees(theta5):.1f}° (Joints 4 and 6 collinear alignment).",
                    "location_xyz": loc_xyz
                })

            # Compute joint angular velocities
            vel_rad_s = np.abs(dq) / dt
            vel_deg_s = np.degrees(vel_rad_s)
            joint_velocities_deg_s.append(vel_deg_s.tolist())

            over_limits = vel_deg_s > self.config.max_joint_vel_deg_s
            # If not near singularity, report pure kinematic joint overspeed
            if np.any(over_limits) and not is_near_singularity:
                bad_joints = [f"J{j+1}:{vel_deg_s[j]:.1f}°/s" for j in range(6) if over_limits[j]]
                loc_xyz = [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]
                logger.warning(f"⚠️ [KinematicChainVerifier] Path {path_id} [OVERSPEED]: Step {step_idx} (Seg {seg_idx}) -> {', '.join(bad_joints)} (XYZ={loc_xyz})")
                issues.append({
                    "type": "JOINT_OVERSPEED",
                    "severity": "WARNING",
                    "segment_index": seg_idx,
                    "step_index": step_idx,
                    "detail": f"Joint overspeed detected: {', '.join(bad_joints)} (max: {self.config.max_joint_vel_deg_s.tolist()})",
                    "location_xyz": loc_xyz
                })

            curr_q = next_q
            trajectory_q.append(curr_q.tolist())

        # Build trajectory TCP poses for simulation playback
        trajectory_tcp = []
        for step_idx in range(len(trajectory_q)):
            T_g = dense_points[step_idx][0]
            pos_xyz = [round(float(v) * 1000.0, 2) for v in T_g[:3, 3]]
            rpy = [round(float(v), 2) for v in R_scipy.from_matrix(T_g[:3, :3]).as_euler('xyz', degrees=True)]
            trajectory_tcp.append(pos_xyz + rpy)

        # Determine overall status and recommended safe velocity
        has_errors = any(issue["severity"] == "ERROR" for issue in issues)
        has_warnings = any(issue["severity"] == "WARNING" for issue in issues)

        status = "FAILED" if has_errors else ("WARNING" if has_warnings else "PASS")

        if joint_velocities_deg_s:
            peak_vels = np.max(np.array(joint_velocities_deg_s), axis=0).tolist()
            vel_ratios = np.array(peak_vels) / self.config.max_joint_vel_deg_s
            max_ratio = float(np.max(vel_ratios))
            if max_ratio > 1.0:
                rec_speed = round(self.interpolator.linear_velocity_mm_s / max_ratio * 0.9, 1)
            else:
                rec_speed = self.interpolator.linear_velocity_mm_s
        else:
            peak_vels = [0.0] * 6
            rec_speed = self.interpolator.linear_velocity_mm_s

        return {
            "path_id": path_id,
            "name": name,
            "status": status,
            "total_interpolated": total_steps,
            "speed_mm_s": self.interpolator.linear_velocity_mm_s,
            "step_size_mm": self.interpolator.step_size_mm,
            "recommended_safe_speed_mm_s": rec_speed,
            "max_joint_velocities_deg_s": self.config.max_joint_vel_deg_s.tolist(),
            "max_joint_velocity_deg_s": self.config.max_joint_vel_deg_s.tolist(),
            "peak_joint_speeds_deg_s": [round(v, 1) for v in peak_vels],
            "issues": issues,
            "trajectory_q": trajectory_q,
            "trajectory_tcp": trajectory_tcp
        }

    def verify_all_paths(self, paths_data: dict) -> dict:
        """
        Verifies all paths in paths_data dictionary.
        """
        paths = paths_data.get("paths", [])
        reports = []
        overall_status = "PASS"
        total_issues = 0
        total_steps = 0
        total_waypoints = 0

        singularity_cnt = 0
        overspeed_cnt = 0
        unreachable_cnt = 0

        last_q = None

        for path in paths:
            rep = self.verify_single_path(path, init_q=last_q)
            reports.append(rep)
            if rep["status"] == "FAILED":
                overall_status = "FAILED"
            elif rep["status"] == "WARNING" and overall_status != "FAILED":
                overall_status = "WARNING"

            total_issues += len(rep["issues"])
            total_steps += rep["total_interpolated"]
            total_waypoints += len(path.get("points", []))

            for iss in rep["issues"]:
                t = iss.get("type", "")
                if "SINGULARITY" in t:
                    singularity_cnt += 1
                elif "OVERSPEED" in t:
                    overspeed_cnt += 1
                elif "UNREACHABLE" in t or "DISCONTINUITY" in t:
                    unreachable_cnt += 1

            if rep.get("trajectory_q"):
                last_q = rep["trajectory_q"][-1]

        return {
            "summary": {
                "status": overall_status,
                "total_paths": len(paths),
                "total_waypoints": total_waypoints,
                "total_steps": total_steps,
                "total_issues": total_issues,
                "singularity_count": singularity_cnt,
                "overspeed_count": overspeed_cnt,
                "unreachable_count": unreachable_cnt
            },
            "nominal_speed_mm_s": self.interpolator.linear_velocity_mm_s,
            "slerp_step_mm": self.interpolator.step_size_mm,
            "max_joint_velocities_deg_s": self.config.max_joint_vel_deg_s.tolist(),
            "urdf_tcp": self.config.urdf_tcp,
            "path_reports": reports
        }
