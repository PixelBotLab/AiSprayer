import os
import sys
import time
import json
import logging
import yaml
from typing import Optional, List, Dict, Any

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from core.hardware.robot.cr5_path_verifier import CR5PathVerifier

logger = logging.getLogger(__name__)

class PathVerificationService:
    """
    Dedicated service for robot kinematic chain verification, singularity/overspeed analysis,
    and spray tolerance axial rotation auto-fix optimization.
    Can be used by interactive manual paths, automated trajectory generators, or spray execution workflows.
    """
    def __init__(self):
        self.template_group_dir = os.path.abspath(os.path.join(PROJECT_ROOT, "data", "template_group"))

    def create_verifier(self, options: dict = None) -> CR5PathVerifier:
        """
        Factory helper to instantiate a CR5PathVerifier configured with custom parameters.
        """
        opts = options or {}
        step_size_mm = float(opts.get("step_size_mm", 1.5))
        
        default_linear_vel = 120.0
        try:
            from services.robot_service import robot_service
            spd_l, _, _, _ = robot_service.get_speed()
            if spd_l and spd_l > 0:
                default_linear_vel = float(spd_l)
        except Exception:
            pass

        linear_velocity_mm_s = float(opts.get("linear_velocity_mm_s", default_linear_vel))
        max_joint_vel = opts.get("max_joint_vel_deg_s", None)
        
        verifier = CR5PathVerifier(
            step_size_mm=step_size_mm,
            linear_velocity_mm_s=linear_velocity_mm_s,
            max_joint_vel_deg_s=max_joint_vel
        )

        tcp_xyz = opts.get("tcp_offset_xyz_mm", None)
        tcp_rpy = opts.get("tcp_offset_rpy_deg", None)
        if tcp_xyz is not None and tcp_rpy is not None:
            verifier.set_tcp_offset(tcp_xyz, tcp_rpy)
            
        return verifier


    def get_urdf_tcp(self) -> dict:
        """Returns the current active URDF tool TCP offset."""
        return CR5PathVerifier.load_tcp_from_urdf()

    def verify_raw_path_data(self, paths_data: dict, options: dict = None) -> dict:
        """
        Directly verifies arbitrary path dictionary data in memory without template file dependency.
        """
        verifier = self.create_verifier(options)
        report = verifier.verify_all_paths(paths_data)
        report["urdf_info"] = verifier.urdf_info
        report["urdf_tcp"] = verifier.urdf_tcp
        report["nominal_speed_mm_s"] = verifier.linear_velocity_mm_s
        return report

    def optimize_raw_path_data(self, paths_data: dict, options: dict = None) -> tuple[dict, dict]:
        """
        Directly optimizes arbitrary path dictionary data in memory without template file dependency.
        Returns: (opt_data, opt_report)
        """
        verifier = self.create_verifier(options)
        opt_data, opt_report = verifier.optimize_all_paths(paths_data)
        opt_report["urdf_info"] = verifier.urdf_info
        opt_report["urdf_tcp"] = verifier.urdf_tcp
        opt_report["nominal_speed_mm_s"] = verifier.linear_velocity_mm_s
        return opt_data, opt_report

    def verify_template_paths(self, template_name: str, state_type: str = "raw", options: dict = None) -> dict:
        """
        Runs offline kinematic chain verification on a template's path YAML (raw, opt, or poi).
        """
        template_dir = os.path.join(self.template_group_dir, template_name)
        
        candidates = [
            f"scan.{state_type}.path.yaml",
            f"{state_type}.path.yaml",
            f"scan.manual_{state_type}_paths.yaml",
        ]
        if state_type == "raw":
            candidates.append("scan.manual_paths.yaml")

        paths_file = None
        for cand in candidates:
            cand_path = os.path.join(template_dir, cand)
            if os.path.exists(cand_path):
                paths_file = cand_path
                break

        if not paths_file:
            logger.error(f"❌ [Verification Service] No path file found for '{state_type}' in {template_dir}")
            raise FileNotFoundError(f"Paths file 'scan.{state_type}.path.yaml' not found in {template_dir}")

        logger.info(f"📂 [Verification Service] Loading '{paths_file}' for template '{template_name}' ({state_type})...")
        with open(paths_file, 'r', encoding='utf-8') as f:
            paths_data = yaml.safe_load(f) or {}

        verifier = self.create_verifier(options)
        report = verifier.verify_all_paths(paths_data)
        report["source_file"] = os.path.basename(paths_file)
        report["state_type"] = state_type
        report["urdf_info"] = verifier.urdf_info
        report["urdf_tcp"] = verifier.urdf_tcp
        report["nominal_speed_mm_s"] = verifier.linear_velocity_mm_s

        # Persist report to disk JSON: scan.raw.report.json / scan.opt.report.json / scan.poi.report.json
        try:
            report_filename = f"scan.{state_type}.report.json"
            report_path = os.path.join(template_dir, report_filename)
            with open(report_path, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            logger.info(f"💾 [Verification Service] Saved diagnostic report to: {report_path}")
        except Exception as e:
            logger.warning(f"⚠️ [Verification Service] Could not save report file: {e}")

        logger.info(f"✅ [Verification Service] Verification finished for '{template_name}' ({state_type}): Status={report['summary']['status']}, Issues={report['summary']['total_issues']}")
        return report

    def get_saved_report(self, template_name: str, state_type: str = "raw") -> Optional[dict]:
        """
        Retrieves previously saved diagnostic report from disk without re-computing IK.
        """
        template_dir = os.path.join(self.template_group_dir, template_name)
        candidates = [
            f"scan.{state_type}.report.json",
            f"{state_type}.report.json",
            f"scan.manual_{state_type}_paths.report.json",
        ]
        if state_type == "raw":
            candidates.append("scan.manual_paths.report.json")

        for cand in candidates:
            report_path = os.path.join(template_dir, cand)
            if os.path.exists(report_path):
                try:
                    with open(report_path, 'r', encoding='utf-8') as f:
                        return json.load(f)
                except Exception as e:
                    logger.warning(f"Failed to read report {report_path}: {e}")
        return None

    def optimize_template_paths(
        self,
        template_name: str,
        mode: str = "opt",
        poi_config: dict = None,
        options: dict = None
    ) -> dict:
        """
        Optimizes manual paths for a template:
        - mode='opt': Free axial spin auto-fix -> saves to scan.opt.path.yaml & scan.opt.report.json
        - mode='poi': POI Pose Constraint auto-fix -> saves to scan.poi.path.yaml & scan.poi.report.json
        """
        template_dir = os.path.join(self.template_group_dir, template_name)
        
        # Source is raw paths
        raw_candidates = ["scan.raw.path.yaml", "raw.path.yaml", "scan.manual_paths.yaml"]
        raw_paths_file = None
        for cand in raw_candidates:
            cand_path = os.path.join(template_dir, cand)
            if os.path.exists(cand_path):
                raw_paths_file = cand_path
                break

        if not raw_paths_file:
            logger.error(f"❌ [Verification Service] Optimization failed: 'scan.raw.path.yaml' not found in {template_dir}")
            raise FileNotFoundError(f"Original paths file 'scan.raw.path.yaml' not found in {template_dir}")

        logger.info(f"⚡ [Verification Service] Starting '{mode}' optimization for template '{template_name}' from '{raw_paths_file}'...")
        with open(raw_paths_file, 'r', encoding='utf-8') as f:
            paths_data = yaml.safe_load(f) or {}

        verifier = self.create_verifier(options)

        if mode == "poi":
            p_cfg = poi_config or {}
            ref_rpy = p_cfg.get("ref_rpy_deg", None)
            tol_rpy = p_cfg.get("tolerance_rpy_deg", [3.0, 15.0, 180.0])
            opt_data, opt_report = verifier.optimize_poi_all_paths(paths_data, ref_rpy_deg=ref_rpy, tol_rpy_deg=tol_rpy)
            target_yaml = "scan.poi.path.yaml"
            target_report = "scan.poi.report.json"
            opt_data["type"] = "poi"
        else:
            opt_data, opt_report = verifier.optimize_all_paths(paths_data)
            target_yaml = "scan.opt.path.yaml"
            target_report = "scan.opt.report.json"
            opt_data["type"] = "opt"

        opt_file = os.path.join(template_dir, target_yaml)
        opt_data["template"] = template_name
        opt_data["updated_at"] = int(time.time())
        opt_data["coordinate_frame"] = "base_link"

        with open(opt_file, 'w', encoding='utf-8') as f:
            yaml.dump(opt_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

        logger.info(f"💾 [Verification Service] Saved {mode} paths to: {opt_file}")
        opt_report["source_file"] = target_yaml
        opt_report["saved_file"] = target_yaml
        opt_report["state_type"] = mode
        opt_report["optimized_paths"] = opt_data.get("paths", [])
        opt_report["urdf_info"] = verifier.urdf_info
        opt_report["urdf_tcp"] = verifier.urdf_tcp
        opt_report["nominal_speed_mm_s"] = verifier.linear_velocity_mm_s

        # Persist report to disk
        try:
            report_path = os.path.join(template_dir, target_report)
            with open(report_path, 'w', encoding='utf-8') as f:
                json.dump(opt_report, f, indent=2, ensure_ascii=False)
            logger.info(f"💾 [Verification Service] Saved {mode} diagnostic report to: {report_path}")
        except Exception as e:
            logger.warning(f"⚠️ [Verification Service] Could not save {target_report}: {e}")

        # Also refresh raw report for accurate side-by-side comparison
        try:
            raw_rep = self.verify_template_paths(template_name, state_type="raw", options=options)
            opt_report["raw_report"] = raw_rep
        except Exception as e:
            logger.warning(f"Could not synchronize raw report during optimization: {e}")

        logger.info(f"🎉 [Verification Service] Optimization ({mode}) completed for '{template_name}': Status={opt_report['summary']['status']}")
        return opt_report


path_verification_service = PathVerificationService()

