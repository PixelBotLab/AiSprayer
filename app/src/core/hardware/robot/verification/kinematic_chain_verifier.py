"""
Kinematic Chain Path Verifier for Dobot CR5.

Simulates a dense MoveL Cartesian path as a continuous IK chain:

1. Interpolate waypoints to ~step_size_mm Cartesian samples.
2. At each sample, pick the IK branch nearest to the previous q
   (`get_best_ik_controller`: expand ±2π then unwrap onto the live winding).
   Using sols[0] would flip shoulder/elbow/wrist and produce ~180° jumps.
3. Diagnose joint limits, branch jumps (>45°), and singularities.
   Shoulder/elbow/wrist warnings are rising-edge only (enter once, not every 1.5 mm).
4. Joint overspeed near a singularity is ERROR (Cartesian speed maps to huge Δq);
   overspeed far from singularity is WARNING (slow the path).
5. Record trajectory_q / trajectory_tcp for playback.
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
BRANCH_JUMP_DEG = 45.0


class KinematicChainVerifier:
    """
    Simulates continuous MoveL Cartesian execution along the robot kinematic chain.

    Pass a pre-built `kinematics_solver` (with python/cpp backend) from CR5PathVerifier.
    If omitted, a solver is created with URDF limits and backend="auto".
    """
    def __init__(
        self,
        robot_config: RobotConfig,
        interpolator: PathInterpolator,
        kinematics_solver: CR5Kinematics = None,
        kinematics_backend: str = "auto",
    ):
        self.config = robot_config
        self.interpolator = interpolator
        if kinematics_solver is not None:
            self.solver = kinematics_solver
        else:
            self.solver = CR5Kinematics(
                joint_min=getattr(robot_config, "joint_min_rad", None),
                joint_max=getattr(robot_config, "joint_max_rad", None),
                backend=kinematics_backend,
            )

    def _loc_xyz(self, T_gun: np.ndarray) -> list[float]:
        return [round(float(x), 2) for x in T_gun[:3, 3] * 1000.0]

    def _append_issue(self, issues: list, **kwargs):
        issues.append(kwargs)

    def _diagnose_step(
        self,
        q: np.ndarray,
        T_gun: np.ndarray,
        step_idx: int,
        seg_idx: int,
        issues: list,
        prev_flags: dict,
        emit_even_if_continuing: bool = False,
    ) -> dict:
        """
        Per-step diagnosis. Joint-limit ERROR every time it is out of range.
        Singularity WARNINGs fire on rising edge (or always on the first point)
        so a long near-singular segment does not spam one issue per millimetre.
        """
        loc = self._loc_xyz(T_gun)
        T_urdf = self.solver.controller_matrix_to_urdf(T_gun)
        risk = self.solver.check_singularity_risk(q, T=T_urdf)

        if not self.solver.is_joint_valid(q):
            q_deg = [round(math.degrees(a), 2) for a in q]
            logger.error(
                f"❌ [KinematicChainVerifier] JOINT_LIMIT step={step_idx} q_deg={q_deg} XYZ={loc}"
            )
            self._append_issue(
                issues,
                type="JOINT_LIMIT",
                severity="ERROR",
                segment_index=seg_idx,
                step_index=step_idx,
                detail=f"Joint angles out of URDF/CR5 limits: {q_deg} deg",
                location_xyz=loc,
            )

        specs = (
            ("shoulder_singularity", "SHOULDER_SINGULARITY",
             f"Near shoulder singularity: two q1 branches separated by "
             f"{risk['shoulder_q1_separation_deg']:.1f}° (wrist near J1-axis cylinder)."),
            ("elbow_singularity", "ELBOW_SINGULARITY",
             f"Near elbow singularity: Joint 3 angle={risk['elbow_angle_deg']:.1f}° "
             f"(a2/a3 collinear, sin(q3)~0)."),
            ("wrist_singularity", "WRIST_SINGULARITY",
             f"Near wrist singularity: Joint 5 angle={risk['wrist_angle_deg']:.1f}° "
             f"(J4/J6 collinear, sin(q5)~0)."),
        )
        for flag, itype, detail in specs:
            active = bool(risk[flag])
            entered = active and (emit_even_if_continuing or not prev_flags.get(flag, False))
            if entered:
                logger.warning(
                    f"⚠️ [KinematicChainVerifier] {itype} step={step_idx} XYZ={loc} {detail}"
                )
                self._append_issue(
                    issues,
                    type=itype,
                    severity="WARNING",
                    segment_index=seg_idx,
                    step_index=step_idx,
                    detail=detail,
                    location_xyz=loc,
                )
            prev_flags[flag] = active

        return risk

    def verify_single_path(self, path_item: dict, init_q: list[float] = None) -> dict:
        """
        Verifies kinematic feasibility of a single path item:
        - Interpolates dense MoveL Cartesian steps
        - Tracks continuous IK with get_best_ik (expand + unwrap onto current q)
        - Checks joint limits, joint velocities, shoulder/elbow/wrist singularities
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
        prev_flags = {
            "shoulder_singularity": False,
            "elbow_singularity": False,
            "wrist_singularity": False,
        }

        first_T_gun = dense_points[0][0]
        # Seed must be a reachable posture close to the real start; default is a typical
        # CR5 "ready" pose. When chaining paths, pass the previous path's last q as init_q
        # so the first step does not jump to a different IK branch.
        default_seed_q = np.array([PI, 0.0, PI / 2.0, PI / 2.0, PI / 2.0, 0.0], dtype=np.float64)
        q_ref = np.array(init_q, dtype=np.float64) if init_q is not None else default_seed_q

        curr_q = self.solver.get_best_ik_controller(first_T_gun, q_ref)
        if curr_q is None:
            loc_xyz = self._loc_xyz(first_T_gun)
            logger.error(f"❌ [KinematicChainVerifier] Path {path_id} Start Point Unreachable: XYZ={loc_xyz}")
            return {
                "path_id": path_id,
                "name": name,
                "status": "FAILED",
                "total_interpolated": total_steps,
                "issues": [{
                    "severity": "ERROR",
                    "type": "UNREACHABLE_START",
                    "detail": "Start waypoint has no valid IK solutions within joint limits.",
                    "location_xyz": loc_xyz,
                    "step_index": 0,
                    "segment_index": 0
                }],
                "max_joint_velocities_deg_s": [0.0] * 6,
                "peak_joint_speeds_deg_s": [0.0] * 6,
                "trajectory_q": [],
                "trajectory_tcp": []
            }

        self._diagnose_step(
            curr_q, first_T_gun, 0, 0, issues, prev_flags, emit_even_if_continuing=True
        )
        trajectory_q.append(curr_q.tolist())

        for step_idx in range(1, total_steps):
            T_gun, _, dt, seg_idx = dense_points[step_idx]
            loc_xyz = self._loc_xyz(T_gun)

            # Nearest of the 8 IK branches, unwrapped onto curr_q so J6 stays on
            # the same ±2π winding instead of snapping to the [-π, π] representative.
            next_q = self.solver.get_best_ik_controller(T_gun, curr_q)
            if next_q is None:
                logger.error(
                    f"❌ [KinematicChainVerifier] Path {path_id} [UNREACHABLE]: "
                    f"Step {step_idx}/{total_steps} (Seg {seg_idx}) at XYZ={loc_xyz}"
                )
                self._append_issue(
                    issues,
                    severity="ERROR",
                    type="UNREACHABLE_STEP",
                    detail=f"Step {step_idx} (segment {seg_idx}) has no valid IK within joint limits.",
                    step_index=step_idx,
                    segment_index=seg_idx,
                    location_xyz=loc_xyz,
                )
                break

            dq = next_q - curr_q
            # 45° in one Cartesian millimetre is not a continuous MoveL — usually a
            # different IK branch (shoulder/elbow/wrist flip) or a joint-limit bounce.
            if np.max(np.abs(np.degrees(dq))) > BRANCH_JUMP_DEG:
                logger.warning(
                    f"⚠️ [KinematicChainVerifier] Path {path_id} [DISCONTINUITY]: "
                    f"Step {step_idx} jump={np.max(np.abs(np.degrees(dq))):.1f}° at XYZ={loc_xyz}"
                )
                self._append_issue(
                    issues,
                    severity="ERROR",
                    type="KINEMATIC_DISCONTINUITY",
                    segment_index=seg_idx,
                    step_index=step_idx,
                    detail=(
                        f"Branch jump detected (max Δq={np.max(np.abs(np.degrees(dq))):.1f}°). "
                        f"Near singularity or joint limit."
                    ),
                    location_xyz=loc_xyz,
                )

            risk = self._diagnose_step(next_q, T_gun, step_idx, seg_idx, issues, prev_flags)
            is_near_singularity = bool(risk["is_singular"])

            # ω = Δq/dt from the interpolator's Cartesian dt. Near a singularity
            # that Δq explodes; treat as ERROR so the path is not marked "just slow it".
            if dt > 1e-9:
                vel_rad_s = np.abs(dq) / dt
                vel_deg_s = np.degrees(vel_rad_s)
                joint_velocities_deg_s.append(vel_deg_s.tolist())
                over_limits = vel_deg_s > self.config.max_joint_vel_deg_s
                if np.any(over_limits):
                    bad_joints = [
                        f"J{j + 1}:{vel_deg_s[j]:.1f}°/s"
                        for j in range(6) if over_limits[j]
                    ]
                    severity = "ERROR" if is_near_singularity else "WARNING"
                    logger.warning(
                        f"⚠️ [KinematicChainVerifier] Path {path_id} [OVERSPEED/{severity}]: "
                        f"Step {step_idx} (Seg {seg_idx}) -> {', '.join(bad_joints)} (XYZ={loc_xyz})"
                    )
                    self._append_issue(
                        issues,
                        type="JOINT_OVERSPEED",
                        severity=severity,
                        segment_index=seg_idx,
                        step_index=step_idx,
                        detail=(
                            f"Joint overspeed detected: {', '.join(bad_joints)} "
                            f"(max: {self.config.max_joint_vel_deg_s.tolist()})"
                            + ("; near singularity" if is_near_singularity else "")
                        ),
                        location_xyz=loc_xyz,
                    )

            curr_q = next_q
            trajectory_q.append(curr_q.tolist())

        trajectory_tcp = []
        for step_idx in range(len(trajectory_q)):
            T_g = dense_points[step_idx][0]
            pos_xyz = [round(float(v) * 1000.0, 2) for v in T_g[:3, 3]]
            rpy = [round(float(v), 2) for v in R_scipy.from_matrix(T_g[:3, :3]).as_euler('xyz', degrees=True)]
            trajectory_tcp.append(pos_xyz + rpy)

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
                elif "UNREACHABLE" in t or "DISCONTINUITY" in t or t == "JOINT_LIMIT":
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
