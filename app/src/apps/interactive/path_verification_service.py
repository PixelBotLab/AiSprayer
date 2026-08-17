import os
import sys
import time
import json
import logging
import math
import yaml
from typing import Optional

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from core.hardware.robot.cr5_path_verifier import CR5PathVerifier

logger = logging.getLogger(__name__)

VALID_PATH_STATES = {"raw", "opt", "poi"}
DEFAULT_POI_TOLERANCE_RPY_DEG = [10.0, 10.0, 180.0]


def _ensure_state_type(state_type: str) -> str:
    state = (state_type or "").strip().lower()
    if state not in VALID_PATH_STATES:
        raise ValueError(f"Invalid path state '{state_type}'. Expected one of: raw, opt, poi")
    return state


def _scan_path_filename(state_type: str) -> str:
    return f"scan.{_ensure_state_type(state_type)}.path.yaml"


def _scan_report_filename(state_type: str) -> str:
    return f"scan.{_ensure_state_type(state_type)}.report.json"


def _validate_float_triplet(values, field_name: str, default=None, *, min_value: float | None = None, max_value: float | None = None) -> list[float]:
    if values is None:
        if default is None:
            raise ValueError(f"{field_name} is required")
        values = default
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError(f"{field_name} must be a 3-element list")
    result = []
    for idx, value in enumerate(values):
        v = float(value)
        if not math.isfinite(v):
            raise ValueError(f"{field_name}[{idx}] must be finite")
        if min_value is not None and v < min_value:
            raise ValueError(f"{field_name}[{idx}] must be >= {min_value}")
        if max_value is not None and v > max_value:
            raise ValueError(f"{field_name}[{idx}] must be <= {max_value}")
        result.append(v)
    return result

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

    def _path_file(self, template_name: str, state_type: str) -> str:
        state = _ensure_state_type(state_type)
        return os.path.join(self.template_group_dir, template_name, _scan_path_filename(state))

    def _report_file(self, template_name: str, state_type: str) -> str:
        state = _ensure_state_type(state_type)
        return os.path.join(self.template_group_dir, template_name, _scan_report_filename(state))

    def _decorate_report(self, report: dict, verifier: CR5PathVerifier, state_type: str, source_file: str) -> dict:
        report["source_file"] = source_file
        report["state_type"] = state_type
        report["urdf_info"] = verifier.urdf_info
        report["urdf_tcp"] = verifier.urdf_tcp
        report["nominal_speed_mm_s"] = verifier.linear_velocity_mm_s
        return report

    def verify_template_paths(self, template_name: str, state_type: str = "raw", options: dict = None) -> dict:
        """
        Runs offline kinematic chain verification on scan.raw/opt/poi.path.yaml only.
        """
        state = _ensure_state_type(state_type)
        paths_file = self._path_file(template_name, state)

        if not os.path.exists(paths_file):
            logger.error(f"❌ [Verification Service] Required path file not found: {paths_file}")
            raise FileNotFoundError(f"Paths file '{_scan_path_filename(state)}' not found")

        logger.info(f"📂 [Verification Service] Loading '{paths_file}' for template '{template_name}' ({state})...")
        with open(paths_file, 'r', encoding='utf-8') as f:
            paths_data = yaml.safe_load(f) or {}

        verifier = self.create_verifier(options)
        report = verifier.verify_all_paths(paths_data)
        self._decorate_report(report, verifier, state, os.path.basename(paths_file))

        report_path = self._report_file(template_name, state)
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        logger.info(f"💾 [Verification Service] Saved diagnostic report to: {report_path}")

        logger.info(f"✅ [Verification Service] Verification finished for '{template_name}' ({state}): Status={report['summary']['status']}, Issues={report['summary']['total_issues']}")
        return report

    def get_saved_report(self, template_name: str, state_type: str = "raw") -> Optional[dict]:
        """
        Retrieves saved scan.raw/opt/poi.report.json from disk without re-computing IK.
        """
        state = _ensure_state_type(state_type)
        report_path = self._report_file(template_name, state)
        if not os.path.exists(report_path):
            return None
        try:
            with open(report_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            logger.warning(f"Failed to read report {report_path}: {e}")
            return None

    def _normalize_poi_config(self, poi_config: dict | None, anchor_source: str | None = None) -> dict:
        cfg = poi_config or {}
        raw_ref = cfg.get("ref_rpy_deg")
        source = cfg.get("anchor_source") or anchor_source or ("manual" if raw_ref is not None else "home")
        if source not in {"home", "live", "manual", "raw"}:
            raise ValueError("poi_config.anchor_source must be one of: home, live, manual, raw")

        if raw_ref is not None:
            ref_rpy = _validate_float_triplet(raw_ref, "poi_config.ref_rpy_deg")
        elif source == "home":
            ref_rpy = [90.0, 0.0, 90.0]
        else:
            ref_rpy = None

        tol_rpy = _validate_float_triplet(
            cfg.get("tolerance_rpy_deg"),
            "poi_config.tolerance_rpy_deg",
            default=[10.0, 10.0, 180.0],
            min_value=0.0,
            max_value=180.0,
        )

        return {
            "mode": "absolute_anchor_tolerance",
            "anchor_source": source,
            "ref_rpy_deg": ref_rpy,
            "tolerance_rpy_deg": tol_rpy,
            "euler_order": "XYZ",
            "units": "deg",
        }

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
        - mode='poi': Absolute Anchor + tolerance POI -> saves to scan.poi.path.yaml & scan.poi.report.json
        """
        state = _ensure_state_type(mode)
        if state == "raw":
            raise ValueError("Optimization mode must be 'opt' or 'poi'")

        template_dir = os.path.join(self.template_group_dir, template_name)
        raw_paths_file = self._path_file(template_name, "raw")
        if not os.path.exists(raw_paths_file):
            logger.error(f"❌ [Verification Service] Optimization failed: '{_scan_path_filename('raw')}' not found")
            raise FileNotFoundError(f"Original paths file '{_scan_path_filename('raw')}' not found")

        logger.info(f"⚡ [Verification Service] Starting '{state}' optimization for template '{template_name}' from '{raw_paths_file}'...")
        with open(raw_paths_file, 'r', encoding='utf-8') as f:
            paths_data = yaml.safe_load(f) or {}

        verifier = self.create_verifier(options)
        effective_poi_config = None

        if state == "poi":
            effective_poi_config = self._normalize_poi_config(poi_config)
            opt_data, _ = verifier.optimize_poi_all_paths(
                paths_data,
                ref_rpy_deg=effective_poi_config["ref_rpy_deg"],
                tolerance_rpy_deg=effective_poi_config["tolerance_rpy_deg"],
            )
            opt_data["poi_config"] = effective_poi_config
        else:
            opt_data, _ = verifier.optimize_all_paths(paths_data)

        target_yaml = _scan_path_filename(state)
        target_report = _scan_report_filename(state)
        opt_file = os.path.join(template_dir, target_yaml)
        opt_data["template"] = template_name
        opt_data["type"] = state
        opt_data["state_type"] = state
        opt_data["updated_at"] = int(time.time())
        opt_data["coordinate_frame"] = "base_link"
        opt_data["source_file"] = _scan_path_filename("raw")
        opt_data["saved_file"] = target_yaml
        opt_data["execution_speed_mm_s"] = verifier.linear_velocity_mm_s

        if state == "poi":
            for path in opt_data.get("paths", []):
                rec_speed = path.get("recommended_speed_mm_s")
                if rec_speed is not None:
                    opt_data["execution_speed_mm_s"] = min(float(opt_data["execution_speed_mm_s"]), float(rec_speed))

        with open(opt_file, 'w', encoding='utf-8') as f:
            yaml.dump(opt_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
        logger.info(f"💾 [Verification Service] Saved {state} paths to: {opt_file}")

        opt_report = verifier.verify_all_paths(opt_data)
        self._decorate_report(opt_report, verifier, state, target_yaml)
        opt_report["saved_file"] = target_yaml
        opt_report["report_file"] = target_report
        opt_report["optimized_paths"] = opt_data.get("paths", [])
        opt_report["execution_speed_mm_s"] = opt_data.get("execution_speed_mm_s", verifier.linear_velocity_mm_s)
        opt_report["optimized_paths_available"] = True
        if effective_poi_config:
            opt_report["poi_config"] = effective_poi_config
            if opt_report["execution_speed_mm_s"] < verifier.linear_velocity_mm_s:
                opt_report["adaptive_feedrate_applied"] = True

        report_path = self._report_file(template_name, state)
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(opt_report, f, indent=2, ensure_ascii=False)
        logger.info(f"💾 [Verification Service] Saved {state} diagnostic report to: {report_path}")

        try:
            raw_rep = self.verify_template_paths(template_name, state_type="raw", options=options)
            opt_report["raw_report"] = raw_rep
        except Exception as e:
            logger.warning(f"Could not synchronize raw report during optimization: {e}")

        logger.info(f"🎉 [Verification Service] Optimization ({state}) completed for '{template_name}': Status={opt_report['summary']['status']}")
        return opt_report


path_verification_service = PathVerificationService()

