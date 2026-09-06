import os
import logging
from typing import Optional
import yaml

from core.config import load_tcp_from_urdf, sprayer_config
from core.motion.cli_client import MotionCliError, optimize_path, verify_path
from core.utils.fast_yaml import fast_yaml_load

logger = logging.getLogger(__name__)

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))

VALID_PATH_STATES = {"raw", "auto", "poi", "auto_poi"}
PATH_YAML_NAMES = {
    "raw": "scan.manual.path.yaml",
    "auto": "scan.auto.path.yaml",
    "poi": "scan.manual.poi.path.yaml",
    "auto_poi": "scan.auto.poi.path.yaml",
}
PATH_YAML_LEGACY = {
    "raw": ("scan.raw.path.yaml",),
    "poi": ("scan.poi.path.yaml",),
}


def get_default_poi_tolerance_rpy_deg() -> list[float]:
    """Reads the live POI anchor tolerance envelope from aisprayer_config.yaml."""
    return list(sprayer_config.poi_tolerance_rpy_deg or [10.0, 10.0, 180.0])


def _ensure_state_type(state_type: str) -> str:
    state = (state_type or "").strip().lower()
    if state not in VALID_PATH_STATES:
        raise ValueError(f"Invalid path state '{state_type}'. Expected one of: raw, auto, poi, auto_poi")
    return state


def _scan_path_filename(state_type: str) -> str:
    return PATH_YAML_NAMES[_ensure_state_type(state_type)]


def _resolve_named_file(template_dir: str, canonical: str, legacy: tuple[str, ...] = ()) -> str:
    canonical_path = os.path.join(template_dir, canonical)
    if os.path.exists(canonical_path):
        return canonical_path
    for name in legacy:
        legacy_path = os.path.join(template_dir, name)
        if os.path.exists(legacy_path):
            return legacy_path
    return canonical_path


def _cleanup_stale_reports(template_dir: str):
    if not os.path.isdir(template_dir):
        return
    for item in os.listdir(template_dir):
        if item.endswith(".report.json") or item.endswith(".report.yaml"):
            try:
                os.remove(os.path.join(template_dir, item))
                logger.info("🧹 Cleaned legacy standalone report file: %s", item)
            except OSError as e:
                logger.warning("Could not remove legacy report %s: %s", item, e)


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = fast_yaml_load(f)
    return data or {}


def _speed_step(options: dict | None) -> tuple[float | None, float | None]:
    """只返回调用方显式给出的速度/步长；未给则让 motion_cli 读 --config。"""
    opts = options or {}
    speed = opts.get("linear_velocity_mm_s")
    step = opts.get("step_size_mm")
    return (
        float(speed) if speed is not None else None,
        float(step) if step is not None else None,
    )


class PathVerificationService:
    """Kinematic verify / POI optimize via motion_cli. Reports live in the unified .path.yaml."""

    def __init__(self):
        self.template_group_dir = os.path.abspath(os.path.join(PROJECT_ROOT, "data", "template_group"))

    def get_urdf_tcp(self) -> dict:
        return load_tcp_from_urdf()

    def _path_file(self, template_name: str, state_type: str, *, existing: bool = True) -> str:
        state = _ensure_state_type(state_type)
        template_dir = os.path.join(self.template_group_dir, template_name)
        canonical = os.path.join(template_dir, _scan_path_filename(state))
        if not existing:
            return canonical
        return _resolve_named_file(template_dir, _scan_path_filename(state), PATH_YAML_LEGACY.get(state, ()))

    def verify_template_paths(self, template_name: str, state_type: str = "raw", options: dict = None) -> dict:
        state = _ensure_state_type(state_type)
        template_dir = os.path.join(self.template_group_dir, template_name)
        paths_file = self._path_file(template_name, state)
        if not os.path.exists(paths_file):
            logger.error("❌ [Verification Service] Required path file not found: %s", paths_file)
            raise FileNotFoundError(f"Paths file '{_scan_path_filename(state)}' not found")

        canonical_path = os.path.join(template_dir, _scan_path_filename(state))
        speed, step = _speed_step(options)
        logger.info(
            "📂 [Verification Service] motion_cli verify '%s' (%s) speed=%s step=%s (None=配置文件)",
            paths_file, state, speed, step,
        )
        try:
            verify_path(paths_file, canonical_path, speed_mm_s=speed, step_mm=step)
        except MotionCliError as e:
            raise RuntimeError(str(e)) from e

        data = _load_yaml(canonical_path)
        cleaned_report = data.get("verification") or {}
        _cleanup_stale_reports(template_dir)
        logger.info(
            "✅ [Verification Service] Verification finished for '%s' (%s): Status=%s",
            template_name, state, cleaned_report.get("status", "UNKNOWN"),
        )
        return cleaned_report

    def get_saved_report(self, template_name: str, state_type: str = "raw") -> Optional[dict]:
        paths_file = self._path_file(template_name, state_type)
        if not os.path.exists(paths_file):
            return None
        try:
            return _load_yaml(paths_file).get("verification")
        except Exception as e:
            logger.warning("Failed to read verification from %s: %s", paths_file, e)
            return None

    def optimize_template_paths(
        self,
        template_name: str,
        source: str = "raw",
        poi_config: dict = None,
        options: dict = None,
    ) -> dict:
        source = (source or "raw").strip().lower()
        if source not in {"raw", "auto"}:
            source = "raw"

        out_state = "poi" if source == "raw" else "auto_poi"
        template_dir = os.path.join(self.template_group_dir, template_name)
        source_paths_file = self._path_file(template_name, source)
        if not os.path.exists(source_paths_file):
            logger.error("❌ [Verification Service] Optimization failed: '%s' not found", _scan_path_filename(source))
            raise FileNotFoundError(f"Original paths file '{_scan_path_filename(source)}' not found")

        anchor_source = sprayer_config.poi_anchor_source
        ref_rpy = sprayer_config.poi_ref_rpy_deg if anchor_source == "config" else None
        tol_rpy = get_default_poi_tolerance_rpy_deg()
        if poi_config and isinstance(poi_config, dict):
            if poi_config.get("anchor_source"):
                anchor_source = str(poi_config["anchor_source"]).strip().lower()
                ref_rpy = sprayer_config.poi_ref_rpy_deg if anchor_source == "config" else None
            if poi_config.get("tolerance_rpy_deg"):
                tol_rpy = list(poi_config["tolerance_rpy_deg"])
            if poi_config.get("ref_rpy_deg"):
                ref_rpy = list(poi_config["ref_rpy_deg"])
                anchor_source = "config"

        if anchor_source == "config" and not ref_rpy:
            logger.warning(
                "⚠️ [Verification Service] poi_anchor_source='config' 但 spraying.poi_ref_rpy_deg 未配置，退回 Home 正解锚点"
            )
            anchor_source = "home"

        speed, step = _speed_step(options)
        target_yaml = _scan_path_filename(out_state)
        opt_file = os.path.join(template_dir, target_yaml)
        logger.info(
            "🚀 [Verification Service] motion_cli optimize %s → %s (anchor=%s ref_rpy=%s tol=%s speed=%s step=%s, None=配置文件)",
            source_paths_file, opt_file, anchor_source, ref_rpy, tol_rpy, speed, step,
        )
        try:
            optimize_path(
                source_paths_file,
                opt_file,
                state_type=out_state,
                anchor_source=anchor_source,
                ref_rpy_deg=ref_rpy,
                tolerance_rpy_deg=tol_rpy,
                speed_mm_s=speed,
                step_mm=step,
            )
        except MotionCliError as e:
            logger.error(
                f"❌ [Verification Service] Optimization failed for '{template_name}' (anchor={anchor_source}): {e}. "
                "Robot execution is blocked."
            )
            return self.record_optimization_failure(template_name, source=source, error=str(e))
        except Exception as e:
            logger.error(
                f"❌ [Verification Service] Unexpected optimization error for '{template_name}': {e}. "
                "Robot execution is blocked."
            )
            return self.record_optimization_failure(template_name, source=source, error=str(e))

        data = _load_yaml(opt_file)
        cleaned_report = data.get("verification") or {}
        cleaned_report["optimized_paths"] = data.get("paths") or []
        cleaned_report["saved_file"] = target_yaml
        # 容差阶梯择优：motion_cli 可能采纳了比请求更紧的一档包络（大容差不保证解更好，
        # 见 app/src/core/motion/docs/optimizer_monotonicity_improvement_proposal.md §6.5）。
        # 这里把“实际生效的那一档”回传并记日志，避免用户误以为放宽没生效。
        poi_cfg = data.get("poi_config") or {}
        adopted_tol = poi_cfg.get("adopted_tolerance_rpy_deg")
        cleaned_report["requested_tolerance_rpy_deg"] = list(tol_rpy)
        if adopted_tol:
            cleaned_report["adopted_tolerance_rpy_deg"] = list(adopted_tol)
            cleaned_report["tolerance_ladder_applied"] = bool(poi_cfg.get("tolerance_ladder_applied"))
            if [float(v) for v in adopted_tol] != [float(v) for v in tol_rpy]:
                logger.info(
                    "🪜 [Verification Service] 容差阶梯择优: 请求包络 %s → 采纳 %s（更紧的一档解质量更优）",
                    list(tol_rpy), list(adopted_tol),
                )
        if "issues" not in cleaned_report or not cleaned_report["issues"]:
            all_issues = []
            for pr in cleaned_report.get("path_reports", []):
                all_issues.extend(pr.get("issues", []))
            cleaned_report["issues"] = all_issues
        _cleanup_stale_reports(template_dir)

        is_pass = cleaned_report.get("status") == "PASS" or cleaned_report.get("summary", {}).get("status") == "PASS"
        issues = cleaned_report.get("issues", [])
        if not is_pass:
            logger.error(
                f"❌ [Verification Service] Optimization completed for '{template_name}', but verification status is FAILED "
                f"({len(issues)} issue(s) detected). Robot execution is blocked."
            )
            for iss in issues[:5]:
                logger.error(
                    f"   ❌ [{iss.get('severity', 'ERROR')}] {iss.get('type', 'UNKNOWN')}: {iss.get('detail', '')}"
                )
        else:
            logger.info(
                f"🎉 [Verification Service] Optimization ({out_state}) completed successfully for '{template_name}': Status=PASS"
            )

        return cleaned_report

    def record_optimization_failure(self, template_name: str, source: str = "raw", error: str = "") -> dict:
        """
        在优化完全失败（如无可行解或 CLI 异常）时，将目标 .path.yaml 文件落盘标记为 FAILED，
        并返回统一校验报告，杜绝前端残留旧的 PASS 状态并严防机械臂误执行。
        """
        out_state = "poi" if source == "raw" else "auto_poi"
        template_dir = os.path.join(self.template_group_dir, template_name)
        target_yaml = _scan_path_filename(out_state)
        opt_file = os.path.join(template_dir, target_yaml)
        source_paths_file = self._path_file(template_name, source)

        paths = []
        if os.path.exists(source_paths_file):
            try:
                paths = _load_yaml(source_paths_file).get("paths", [])
            except Exception:
                paths = []

        failed_report = {
            "status": "FAILED",
            "saved_file": target_yaml,
            "optimized_paths": paths,
            "summary": {
                "status": "FAILED",
                "total_issues": 1,
                "total_paths": len(paths),
                "total_waypoints": sum(len(p.get("points", [])) for p in paths),
                "total_steps": 0,
            },
            "issues": [
                {
                    "type": "OPTIMIZATION_FAILED",
                    "severity": "ERROR",
                    "detail": f"Path optimization rejected: {error}",
                }
            ],
            "path_reports": [],
        }

        doc = {
            "type": out_state,
            "state_type": out_state,
            "source_file": os.path.basename(source_paths_file),
            "paths": paths,
            "verification": failed_report,
        }
        try:
            with open(opt_file, "w", encoding="utf-8") as f:
                yaml.dump(doc, f, allow_unicode=True, sort_keys=False)
        except Exception as write_err:
            logger.error(f"Failed to write failure record to {opt_file}: {write_err}")

        return failed_report


path_verification_service = PathVerificationService()
