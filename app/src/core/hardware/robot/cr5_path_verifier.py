"""
CR5 Offline Chain Path Verifier & Tolerance-based Auto-Fix Optimizer.
Simulates dense Cartesian MoveL interpolation, decouples TCP offset (Flange = Gun * T_tcp_inv),
tracks continuous IK solutions, diagnoses velocity spikes / singularities,
and optimizes waypoint orientations using spray-gun axial rotation tolerance.
"""

import os
import math
import logging
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R_scipy, Slerp

from .cr5_kinematics import CR5Kinematics

logger = logging.getLogger(__name__)
PI = math.pi

class CR5PathVerifier:
    @classmethod
    def load_limits_from_urdf(cls, urdf_path: str = None) -> dict:
        """
        Parses joint limit angles (lower, upper in deg) and max joint velocity (deg/s) from URDF XML.
        """
        if urdf_path is None:
            candidates = [
                os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../urdf/cr5_robot.urdf")),
                os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../urdf/cr5_robot_with_gun.urdf")),
                os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../configs/m530_r6.urdf.xml")),
            ]
            for c in candidates:
                if os.path.exists(c):
                    urdf_path = c
                    break

        joint_limits_deg = {}
        joint_max_vel_deg_s = [180.0] * 6

        if urdf_path and os.path.exists(urdf_path):
            try:
                import xml.etree.ElementTree as ET
                tree = ET.parse(urdf_path)
                root = tree.getroot()
                for joint in root.findall('joint'):
                    name = joint.get('name', '')
                    j_idx = None
                    for idx in range(1, 7):
                        if name.lower() in [f"joint{idx}", f"joint_{idx}", f"j{idx}"]:
                            j_idx = idx - 1
                            break
                    if j_idx is not None:
                        limit = joint.find('limit')
                        if limit is not None:
                            lower = float(limit.get('lower', -math.pi))
                            upper = float(limit.get('upper', math.pi))
                            vel = float(limit.get('velocity', math.pi))
                            joint_limits_deg[f"joint{j_idx+1}"] = {
                                "lower_deg": round(math.degrees(lower), 1),
                                "upper_deg": round(math.degrees(upper), 1),
                                "velocity_deg_s": round(math.degrees(vel), 1)
                            }
                            joint_max_vel_deg_s[j_idx] = round(math.degrees(vel), 1)
            except Exception as e:
                logger.warning(f"Could not parse joint limits from URDF {urdf_path}: {e}")

        return {
            "joint_limits": joint_limits_deg,
            "max_joint_vel_deg_s": joint_max_vel_deg_s,
            "urdf_source": os.path.basename(urdf_path) if urdf_path else None
        }

    def __init__(self,
                 step_size_mm: float = 1.5,
                 linear_velocity_mm_s: float = 120.0,
                 max_joint_vel_deg_s: list[float] = None,
                 t_tcp_flange: np.ndarray = None,
                 urdf_path: str = None):
        """
        :param step_size_mm: Cartesian dense interpolation resolution in mm
        :param linear_velocity_mm_s: nominal MoveL execution speed
        :param max_joint_vel_deg_s: maximum allowable joint speed per axis (deg/s)
        :param t_tcp_flange: 4x4 matrix from Link6 Flange to Gun Tip TCP
        :param urdf_path: optional path to robot URDF
        """
        self.step_size_mm = step_size_mm
        self.linear_velocity_mm_s = linear_velocity_mm_s
        self.solver = CR5Kinematics()

        # Load limits dynamically from URDF
        self.urdf_info = self.load_limits_from_urdf(urdf_path)
        urdf_max_vels = self.urdf_info["max_joint_vel_deg_s"]

        self.max_joint_vel_deg_s = np.array(
            max_joint_vel_deg_s if max_joint_vel_deg_s is not None else urdf_max_vels,
            dtype=np.float64
        )


        # Default TCP transform: from configs/m530_r6.urdf.xml
        # origin xyz="0.05 0 0" rpy="0 1.57079632679 0"
        if t_tcp_flange is not None:
            self.T_tcp_flange = np.array(t_tcp_flange, dtype=np.float64)
        else:
            # 50mm along X, +90 deg around Y
            r_mat = R_scipy.from_euler('xyz', [0.0, 90.0, 0.0], degrees=True).as_matrix()
            self.T_tcp_flange = np.eye(4, dtype=np.float64)
            self.T_tcp_flange[:3, :3] = r_mat
            self.T_tcp_flange[0, 3] = 0.05  # 50mm in meters
            
        self.T_tcp_flange_inv = np.linalg.inv(self.T_tcp_flange)

    def set_tcp_offset(self, xyz_mm: list[float], rpy_deg: list[float]):
        """
        Configures the TCP transform (Flange -> Gun Tip).
        """
        r_mat = R_scipy.from_euler('xyz', rpy_deg, degrees=True).as_matrix()
        self.T_tcp_flange = np.eye(4, dtype=np.float64)
        self.T_tcp_flange[:3, :3] = r_mat
        self.T_tcp_flange[0, 3] = xyz_mm[0] / 1000.0
        self.T_tcp_flange[1, 3] = xyz_mm[1] / 1000.0
        self.T_tcp_flange[2, 3] = xyz_mm[2] / 1000.0
        self.T_tcp_flange_inv = np.linalg.inv(self.T_tcp_flange)

    def pose_dict_to_matrix(self, pose_dict: dict) -> np.ndarray:
        """
        Converts tcp_pose_base dict {x, y, z, rx, ry, rz} (mm, deg) to 4x4 matrix (meters).
        Note: rx, ry, rz in yaml are Euler 'xyz' in degrees in Robot Base Frame.
        """
        x_m = float(pose_dict["x"]) / 1000.0
        y_m = float(pose_dict["y"]) / 1000.0
        z_m = float(pose_dict["z"]) / 1000.0
        rx = float(pose_dict.get("rx", 0.0))
        ry = float(pose_dict.get("ry", 0.0))
        rz = float(pose_dict.get("rz", 0.0))

        R_mat = R_scipy.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R_mat
        T[0, 3] = x_m
        T[1, 3] = y_m
        T[2, 3] = z_m
        return T

    def matrix_to_pose_dict(self, T: np.ndarray) -> dict:
        """
        Converts 4x4 matrix (meters) to tcp_pose_base dict {x, y, z, rx, ry, rz} (mm, deg).
        """
        x_mm = float(T[0, 3] * 1000.0)
        y_mm = float(T[1, 3] * 1000.0)
        z_mm = float(T[2, 3] * 1000.0)
        rpy_deg = R_scipy.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
        return {
            "x": round(x_mm, 2),
            "y": round(y_mm, 2),
            "z": round(z_mm, 2),
            "rx": round(float(rpy_deg[0]), 2),
            "ry": round(float(rpy_deg[1]), 2),
            "rz": round(float(rpy_deg[2]), 2)
        }

    # =========================================================================
    # 1. DENSE CARTESIAN INTERPOLATION (LERP + SLERP)
    # =========================================================================

    def interpolate_path_dense(self, waypoints: list[dict], speed_override_mm_s: float = None) -> list[tuple[np.ndarray, np.ndarray, float, int]]:
        """
        Performs high-density Cartesian interpolation between consecutive waypoints.
        :return: list of (T_gun, T_flange, dt_seconds, segment_index)
        """
        if len(waypoints) < 2:
            return []

        effective_speed = float(speed_override_mm_s or self.linear_velocity_mm_s)
        interpolated = []
        for i in range(len(waypoints) - 1):
            wp_a = waypoints[i]
            wp_b = waypoints[i + 1]

            T_a = self.pose_dict_to_matrix(wp_a["tcp_pose_base"])
            T_b = self.pose_dict_to_matrix(wp_b["tcp_pose_base"])

            pos_a = T_a[:3, 3]
            pos_b = T_b[:3, 3]
            rot_a = R_scipy.from_matrix(T_a[:3, :3])
            rot_b = R_scipy.from_matrix(T_b[:3, :3])

            dist_m = np.linalg.norm(pos_b - pos_a)
            dist_mm = dist_m * 1000.0
            num_steps = max(2, int(math.ceil(dist_mm / self.step_size_mm)))

            key_times = [0.0, 1.0]
            key_rots = R_scipy.from_matrix([T_a[:3, :3], T_b[:3, :3]])
            slerp_solver = Slerp(key_times, key_rots)

            step_dt = (dist_m / effective_speed * 1000.0) / max(1, num_steps - 1) if num_steps > 1 else 0.01

            for s in range(num_steps):
                if i > 0 and s == 0:
                    continue  # Skip duplicate start point of subsequent segments
                alpha = s / float(num_steps - 1) if num_steps > 1 else 0.0
                
                # Lerp Position
                pos_interp = (1.0 - alpha) * pos_a + alpha * pos_b
                
                # Slerp Orientation
                rot_interp = slerp_solver([alpha])[0].as_matrix()

                T_gun = np.eye(4, dtype=np.float64)
                T_gun[:3, :3] = rot_interp
                T_gun[:3, 3] = pos_interp

                # Decouple Tool Offset: T_flange = T_gun * T_tcp_inv
                T_flange = T_gun @ self.T_tcp_flange_inv

                interpolated.append((T_gun, T_flange, max(0.005, step_dt), i))

        return interpolated

    # =========================================================================
    # 2. CONTINUOUS KINEMATIC CHAIN TRACKING & DIAGNOSTICS
    # =========================================================================

    def verify_single_path(self, path_item: dict, init_q: list[float] = None) -> dict:
        """
        Performs full chain verification on a single path.
        """
        waypoints = path_item.get("points", [])
        path_id = path_item.get("path_id", 1)
        path_name = path_item.get("name", f"Path_{path_id}")
        path_speed = float(path_item.get("speed_mm_s") or self.linear_velocity_mm_s)

        if len(waypoints) < 2:
            logger.info(f"ℹ️ [CR5 Verifier] Path {path_id} ('{path_name}') has < 2 points, skipped.")
            return {
                "path_id": path_id,
                "name": path_name,
                "status": "PASS",
                "total_points": len(waypoints),
                "speed_mm_s": path_speed,
                "issues": [],
                "max_joint_velocity_deg_s": [0.0]*6,
                "trajectory_q": []
            }

        dense_points = self.interpolate_path_dense(waypoints, speed_override_mm_s=path_speed)
        logger.info(f"🔍 [CR5 Verifier] Verifying Path {path_id} ('{path_name}'): {len(waypoints)} waypoints -> {len(dense_points)} dense interpolated points (Δs={self.step_size_mm}mm, v={path_speed}mm/s)...")

        issues = []
        trajectory_q = []
        joint_velocities_deg_s = []

        # Find starting joint configuration
        first_T_flange = dense_points[0][1]
        if init_q is not None:
            curr_q = self.solver.get_best_ik(first_T_flange, init_q)
            logger.debug(f"  └─ Initial joint selected from previous path end: {[round(math.degrees(a), 1) for a in curr_q] if curr_q is not None else 'None'}")
        else:
            default_ref = np.array([0.0, -PI/4, PI/2, -PI/4, PI/2, 0.0])
            curr_q = self.solver.get_best_ik(first_T_flange, default_ref)
            logger.debug(f"  └─ Initial joint selected from default reference: {[round(math.degrees(a), 1) for a in curr_q] if curr_q is not None else 'None'}")

        if curr_q is None:
            loc_xyz = [round(float(x), 2) for x in dense_points[0][0][:3, 3] * 1000.0]
            logger.error(f"❌ [CR5 Verifier] Path {path_id} [UNREACHABLE]: Initial waypoint out of workspace at XYZ={loc_xyz}")
            issues.append({
                "type": "UNREACHABLE",
                "severity": "ERROR",
                "waypoint_index": 0,
                "step_index": 0,
                "detail": "Initial waypoint is out of robot reachable workspace",
                "location_xyz": loc_xyz
            })
            return {
                "path_id": path_id,
                "name": path_name,
                "status": "ERROR",
                "total_interpolated": len(dense_points),
                "issues": issues,
                "max_joint_velocity_deg_s": [0.0]*6,
                "trajectory_q": []
            }

        trajectory_q.append(curr_q.tolist())
        max_vels = np.zeros(6, dtype=np.float64)

        for step_idx in range(1, len(dense_points)):
            T_gun, T_flange, dt, seg_idx = dense_points[step_idx]
            next_q = self.solver.get_best_ik(T_flange, curr_q)

            if next_q is None:
                loc_xyz = [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]
                logger.error(f"❌ [CR5 Verifier] Path {path_id} [UNREACHABLE_STEP]: IK lost at step {step_idx}/{len(dense_points)} (Seg {seg_idx}) at XYZ={loc_xyz}")
                issues.append({
                    "type": "UNREACHABLE_STEP",
                    "severity": "ERROR",
                    "segment_index": seg_idx,
                    "step_index": step_idx,
                    "detail": f"IK solution lost at dense step {step_idx} (Cartesian unreachable)",
                    "location_xyz": loc_xyz
                })
                break

            # 1. Check Kinematic Discontinuity (Branch Jump across Singularity / Workspace Boundary)
            delta_q_deg = np.abs(np.degrees(next_q - curr_q))
            if np.any(delta_q_deg > 25.0):
                loc_xyz = [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]
                jump_info = [f"J{j+1}:{delta_q_deg[j]:.1f}°" for j in range(6) if delta_q_deg[j] > 25.0]
                logger.error(f"❌ [CR5 Verifier] Path {path_id} [KINEMATIC_DISCONTINUITY]: Robot arm reached reach/singularity boundary at step {step_idx}/{len(dense_points)} (Seg {seg_idx}) at XYZ={loc_xyz}. Posture jumps: {', '.join(jump_info)}")
                issues.append({
                    "type": "KINEMATIC_DISCONTINUITY",
                    "severity": "ERROR",
                    "segment_index": seg_idx,
                    "step_index": step_idx,
                    "detail": f"Arm reach boundary/singularity reached at step {step_idx}. Posture discontinuity ({', '.join(jump_info)}). Arm cannot continue smoothly along MoveL.",
                    "location_xyz": loc_xyz
                })
                break

            # 2. Check Singularity Risk
            sing_info = self.solver.check_singularity_risk(next_q)
            is_near_singularity = False
            if sing_info["wrist_singularity"]:
                is_near_singularity = True
                loc_xyz = [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]
                logger.warning(f"⚠️ [CR5 Verifier] Path {path_id} [WRIST_SINGULARITY]: Step {step_idx} J5={sing_info['wrist_angle_deg']:.1f}° near 0° at XYZ={loc_xyz}")
                issues.append({
                    "type": "WRIST_SINGULARITY",
                    "severity": "WARNING",
                    "segment_index": seg_idx,
                    "step_index": step_idx,
                    "detail": f"Wrist J5 angle near 0° ({sing_info['wrist_angle_deg']:.1f}°)",
                    "location_xyz": loc_xyz
                })
            elif sing_info["elbow_singularity"]:
                is_near_singularity = True
                loc_xyz = [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]
                logger.warning(f"⚠️ [CR5 Verifier] Path {path_id} [ELBOW_SINGULARITY]: Step {step_idx} J3={sing_info['elbow_angle_deg']:.1f}° near singular at XYZ={loc_xyz}")
                issues.append({
                    "type": "ELBOW_SINGULARITY",
                    "severity": "WARNING",
                    "segment_index": seg_idx,
                    "step_index": step_idx,
                    "detail": f"Elbow J3 angle near singular ({sing_info['elbow_angle_deg']:.1f}°)",
                    "location_xyz": loc_xyz
                })

            # 3. Check Joint Speed
            vel_deg_s = delta_q_deg / dt
            max_vels = np.maximum(max_vels, vel_deg_s)
            joint_velocities_deg_s.append(vel_deg_s.tolist())

            over_limits = vel_deg_s > self.max_joint_vel_deg_s
            # If not near singularity, report pure kinematic joint overspeed
            if np.any(over_limits) and not is_near_singularity:
                bad_joints = [f"J{j+1}:{vel_deg_s[j]:.1f}°/s" for j in range(6) if over_limits[j]]
                loc_xyz = [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]
                logger.warning(f"⚠️ [CR5 Verifier] Path {path_id} [OVERSPEED]: Step {step_idx} (Seg {seg_idx}) -> {', '.join(bad_joints)} (XYZ={loc_xyz})")
                issues.append({
                    "type": "JOINT_OVERSPEED",
                    "severity": "WARNING",
                    "segment_index": seg_idx,
                    "step_index": step_idx,
                    "detail": f"Joint overspeed detected: {', '.join(bad_joints)} (max: {self.max_joint_vel_deg_s.tolist()})",
                    "location_xyz": loc_xyz
                })
                issues.append({
                    "type": "ELBOW_SINGULARITY",
                    "severity": "WARNING",
                    "segment_index": seg_idx,
                    "step_index": step_idx,
                    "detail": f"Elbow J3 angle near singular ({sing_info['elbow_angle_deg']:.1f}°)",
                    "location_xyz": loc_xyz
                })

            curr_q = next_q
            trajectory_q.append(curr_q.tolist())

        # Determine overall status and recommended safe velocity
        has_errors = any(issue["severity"] == "ERROR" for issue in issues)
        has_warnings = any(issue["severity"] == "WARNING" for issue in issues)
        status = "ERROR" if has_errors else ("WARNING" if has_warnings else "PASS")

        # Calculate recommended max linear speed to bring all joints within URDF limits
        safe_speed = float(path_speed)
        max_v = float(np.max(max_vels)) if len(max_vels) > 0 else 0.0
        urdf_limit = float(np.min(self.max_joint_vel_deg_s)) if len(self.max_joint_vel_deg_s) > 0 else 179.9
        if max_v > urdf_limit and max_v > 1e-3:
            scale = (urdf_limit - 5.0) / max_v
            safe_speed = float(math.floor(path_speed * scale))

        logger.info(f"📋 [CR5 Verifier] Path {path_id} Result: Status={status}, Max Velocity={[round(float(v), 1) for v in max_vels]}°/s, Safe Speed={safe_speed}mm/s, Issues={len(issues)}")

        return {
            "path_id": path_id,
            "name": path_name,
            "status": status,
            "speed_mm_s": round(path_speed, 1),
            "total_interpolated": len(dense_points),
            "max_joint_velocity_deg_s": [round(float(v), 1) for v in max_vels],
            "recommended_safe_speed_mm_s": safe_speed,
            "issues": issues,
            "trajectory_q": trajectory_q
        }

    def verify_all_paths(self, paths_data: dict) -> dict:
        """
        Verifies all paths in a paths data dictionary.
        """
        import time
        start_time = time.time()
        paths = paths_data.get("paths", [])
        logger.info(f"🚀 [CR5 Verifier] Starting multi-path verification: {len(paths)} path(s) loaded.")

        results = []
        overall_status = "PASS"
        total_issues = 0

        curr_end_q = None
        for p in paths:
            res = self.verify_single_path(p, init_q=curr_end_q)
            results.append(res)
            if res["trajectory_q"]:
                curr_end_q = np.array(res["trajectory_q"][-1])
            if res["status"] == "ERROR":
                overall_status = "ERROR"
            elif res["status"] == "WARNING" and overall_status != "ERROR":
                overall_status = "WARNING"
            total_issues += len(res["issues"])

        elapsed_ms = (time.time() - start_time) * 1000.0
        logger.info(f"🏁 [CR5 Verifier] All paths verified in {elapsed_ms:.1f}ms: Overall Status={overall_status}, Total Issues={total_issues}")

        return {
            "success": True,
            "summary": {
                "status": overall_status,
                "total_paths": len(paths),
                "total_issues": total_issues,
                "can_optimize": total_issues > 0,
                "elapsed_ms": round(elapsed_ms, 1)
            },
            "path_reports": results
        }

    # =========================================================================
    # 3. SPRAY TOLERANCE-BASED AUTO-FIX OPTIMIZER
    # =========================================================================

    def align_waypoints_smoothly(self, waypoints: list[dict]) -> list[dict]:
        """
        Smooths orientation transitions between consecutive waypoints along the spray axis (Z-axis).
        Eliminates 180-degree tool roll flips caused by coordinate frame reference flips.
        """
        if len(waypoints) < 2:
            return waypoints

        aligned_waypoints = [dict(waypoints[0])]
        prev_R = self.pose_dict_to_matrix(aligned_waypoints[0]["tcp_pose_base"])[:3, :3]

        for i in range(1, len(waypoints)):
            curr_wp = dict(waypoints[i])
            curr_wp["tcp_pose_base"] = dict(waypoints[i]["tcp_pose_base"])
            
            T_curr = self.pose_dict_to_matrix(curr_wp["tcp_pose_base"])
            R_curr = T_curr[:3, :3]

            best_psi_deg = 0.0
            max_trace = -float('inf')

            for psi in np.linspace(-180.0, 180.0, 73, endpoint=False):
                R_spin = R_scipy.from_euler('z', psi, degrees=True).as_matrix()
                R_cand = R_curr @ R_spin
                tr = float(np.trace(prev_R.T @ R_cand))
                if tr > max_trace:
                    max_trace = tr
                    best_psi_deg = psi

            R_spin_best = R_scipy.from_euler('z', best_psi_deg, degrees=True).as_matrix()
            T_opt = T_curr.copy()
            T_opt[:3, :3] = R_curr @ R_spin_best
            
            curr_wp["tcp_pose_base"] = self.matrix_to_pose_dict(T_opt)
            aligned_waypoints.append(curr_wp)
            prev_R = T_opt[:3, :3]

        return aligned_waypoints

    def optimize_single_path(self, path_item: dict, init_q: list[float] = None) -> tuple[dict, bool]:
        """
        Optimizes waypoint orientations using spray-axial rotation tolerance (psi in [-pi, pi]).
        Attempts to eliminate singularities and overspeeds while preserving spatial positions.
        """
        path_id = path_item.get("path_id", 1)
        waypoints = [dict(wp) for wp in path_item.get("points", [])]
        if len(waypoints) < 2:
            return path_item, False

        initial_res = self.verify_single_path({"points": waypoints, "path_id": path_id}, init_q=init_q)
        if initial_res["status"] == "PASS":
            logger.info(f"✨ [CR5 Optimizer] Path {path_id} is already PASS, no optimization needed.")
            return path_item, False

        logger.info(f"✨ [CR5 Optimizer] Starting tolerance auto-fix on Path {path_id} (Initial status={initial_res['status']}, issues={len(initial_res['issues'])})...")

        # 1. First step: perform continuous smooth alignment along path
        aligned_wps = self.align_waypoints_smoothly(waypoints)
        res_aligned = self.verify_single_path({"points": aligned_wps, "path_id": path_id}, init_q=init_q)
        logger.info(f"  ├─ Step 1 (Axial Smooth Alignment): Status={res_aligned['status']}, Issues={len(res_aligned['issues'])}")
        
        if res_aligned["status"] == "PASS":
            opt_path = dict(path_item)
            opt_path["points"] = aligned_wps
            logger.info(f"  └─ Step 1 successfully resolved all issues for Path {path_id}!")
            return opt_path, True

        # 2. Second step: global + local spin fine-tuning if aligned alone isn't sufficient
        best_waypoints = aligned_wps
        best_spin_chosen = 0.0
        best_score = (len([i for i in res_aligned["issues"] if i["severity"] == "ERROR"]) * 1000.0 +
                      len([i for i in res_aligned["issues"] if i["severity"] == "WARNING"]) * 100.0 +
                      float(np.sum(res_aligned["max_joint_velocity_deg_s"])))

        spin_candidates_deg = np.linspace(-180.0, 180.0, 25, endpoint=False)
        is_fixed = False

        for base_spin in spin_candidates_deg:
            cand_waypoints = []
            for wp in aligned_wps:
                cand_wp = {k: v for k, v in wp.items()}
                cand_wp["tcp_pose_base"] = dict(wp["tcp_pose_base"])
                
                T_orig = self.pose_dict_to_matrix(cand_wp["tcp_pose_base"])
                R_spin = R_scipy.from_euler('z', base_spin, degrees=True).as_matrix()
                T_cand = T_orig.copy()
                T_cand[:3, :3] = T_orig[:3, :3] @ R_spin
                
                cand_wp["tcp_pose_base"] = self.matrix_to_pose_dict(T_cand)
                cand_waypoints.append(cand_wp)

            res = self.verify_single_path({"points": cand_waypoints, "path_id": path_id}, init_q=init_q)
            score = (len([i for i in res["issues"] if i["severity"] == "ERROR"]) * 1000.0 +
                     len([i for i in res["issues"] if i["severity"] == "WARNING"]) * 100.0 +
                     float(np.sum(res["max_joint_velocity_deg_s"])))

            if score < best_score:
                best_score = score
                best_waypoints = cand_waypoints
                best_spin_chosen = base_spin
                if res["status"] == "PASS":
                    is_fixed = True
                    break

        logger.info(f"  └─ Step 2 (Grid Spin Search): Selected spin={best_spin_chosen:.1f}° -> Best Score={best_score:.1f}, Fixed={is_fixed}")
        optimized_path = dict(path_item)
        optimized_path["points"] = best_waypoints

        # 3. Third step: Adaptive Feedrate Auto-Tuning for high-curvature segments
        # If axial spin alone still has joint overspeed, auto-tune MoveL speed iteratively to guarantee 100% PASS
        if not is_fixed:
            curr_test_speed = float(self.linear_velocity_mm_s)
            for iter_idx in range(6):
                v_checker = CR5PathVerifier(
                    step_size_mm=self.step_size_mm,
                    linear_velocity_mm_s=curr_test_speed,
                    max_joint_vel_deg_s=self.max_joint_vel_deg_s
                )
                v_checker.T_tcp_flange = self.T_tcp_flange
                v_checker.T_tcp_flange_inv = self.T_tcp_flange_inv
                res_eval = v_checker.verify_single_path({"points": best_waypoints, "path_id": path_id}, init_q=init_q)
                
                has_errors = any(i["severity"] == "ERROR" for i in res_eval.get("issues", []))
                if has_errors:
                    break
                    
                optimized_path["speed_mm_s"] = curr_test_speed
                if res_eval["status"] == "PASS":
                    optimized_path["speed_auto_tuned"] = True
                    is_fixed = True
                    logger.info(f"  └─ Step 3 (Adaptive Feedrate): Auto-tuned MoveL speed to {curr_test_speed} mm/s -> Reached 100% PASS!")
                    break
                    
                safe_spd = float(res_eval.get("recommended_safe_speed_mm_s", 0.0))
                if safe_spd >= curr_test_speed or safe_spd < 5.0:
                    curr_test_speed = round(max(10.0, curr_test_speed * 0.65), 1)
                else:
                    curr_test_speed = round(safe_spd, 1)

        return optimized_path, (is_fixed or best_score < 1000.0)

    def optimize_all_paths(self, paths_data: dict) -> tuple[dict, dict]:
        """
        Optimizes all paths in the dataset and returns (optimized_data, verification_summary).
        """
        import time
        start_time = time.time()
        paths = paths_data.get("paths", [])
        logger.info(f"🎯 [CR5 Optimizer] Optimizing all {len(paths)} paths...")

        opt_paths = []
        any_changed = False
        curr_end_q = None

        for p in paths:
            opt_p, changed = self.optimize_single_path(p, init_q=curr_end_q)
            opt_paths.append(opt_p)
            if changed:
                any_changed = True

            res_p = self.verify_single_path(opt_p, init_q=curr_end_q)
            if res_p.get("trajectory_q"):
                curr_end_q = np.array(res_p["trajectory_q"][-1])

        opt_paths_data = dict(paths_data)
        opt_paths_data["paths"] = opt_paths

        verification_report = self.verify_all_paths(opt_paths_data)
        
        # Tag each path in opt_paths with execution status and safety flags
        report_by_id = {pr["path_id"]: pr for pr in verification_report.get("path_reports", [])}
        for opt_p in opt_paths:
            pid = opt_p.get("path_id")
            p_rep = report_by_id.get(pid)
            if p_rep:
                opt_p["status"] = p_rep["status"]
                opt_p["is_executable"] = (p_rep["status"] in ("PASS", "WARNING"))
                if p_rep["status"] == "ERROR":
                    err_issues = [i["detail"] for i in p_rep.get("issues", []) if i.get("severity") == "ERROR"]
                    opt_p["unreachable_reason"] = "; ".join(err_issues) if err_issues else "Inverse kinematics out of workspace"
                else:
                    opt_p.pop("unreachable_reason", None)

        elapsed_ms = (time.time() - start_time) * 1000.0
        logger.info(f"🎉 [CR5 Optimizer] All paths optimization finished in {elapsed_ms:.1f}ms: Overall Status={verification_report['summary']['status']}")
        return opt_paths_data, verification_report

