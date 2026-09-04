import os
import sys
import time
import logging
import math
import yaml
from typing import Optional

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from core.hardware.robot.cr5_path_verifier import CR5PathVerifier
from core.config import sprayer_config
from core.utils.fast_yaml import fast_yaml_load, fast_yaml_dump

logger = logging.getLogger(__name__)

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
    """Reads the live POI anchor tolerance envelope from aisprayer_config.yaml (spraying.poi_tolerance_rpy_deg)."""
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


def _clean_waypoints_data(points: list) -> list:
    """Strips redundant or zeroed-out fields from waypoints to keep YAML compact."""
    cleaned = []
    for p in points:
        wp = {
            "index": int(p.get("index", len(cleaned) + 1)),
            "pixel": [int(p["pixel"][0]), int(p["pixel"][1])] if "pixel" in p else [0, 0],
            "surface_point_base_mm": [round(float(v), 2) for v in p.get("surface_point_base_mm", [0, 0, 0])],
            "surface_normal_base": [round(float(v), 4) for v in p.get("surface_normal_base", [0, 0, 1])],
            "standoff_distance_mm": round(float(p.get("standoff_distance_mm", 150.0)), 1),
            "tcp_pose_base": {
                "x": round(float(p.get("tcp_pose_base", {}).get("x", 0.0)), 2),
                "y": round(float(p.get("tcp_pose_base", {}).get("y", 0.0)), 2),
                "z": round(float(p.get("tcp_pose_base", {}).get("z", 0.0)), 2),
                "rx": round(float(p.get("tcp_pose_base", {}).get("rx", 0.0)), 2),
                "ry": round(float(p.get("tcp_pose_base", {}).get("ry", 0.0)), 2),
                "rz": round(float(p.get("tcp_pose_base", {}).get("rz", 0.0)), 2),
            },
        }
        if "normal_2d_proj" in p and p["normal_2d_proj"]:
            wp["normal_2d_proj"] = [round(float(v), 1) for v in p["normal_2d_proj"]]
        if "spraying" in p:
            wp["spraying"] = p["spraying"]
        elif p.get("is_jump", False):
            wp["spraying"] = "off"
        else:
            wp["spraying"] = "on"
        if "is_jump" in p:
            wp["is_jump"] = bool(p["is_jump"])
        cleaned.append(wp)
    return cleaned


def _clean_report_data(report: dict) -> dict:
    """Compacts verification report by rounding trajectory floats and removing duplicate blobs."""
    if not report:
        return {}

    cleaned_path_reports = []
    for pr in report.get("path_reports", []):
        tq = pr.get("trajectory_q", [])
        tt = pr.get("trajectory_tcp", [])

        # Round trajectory floats for compact YAML storage
        compact_q = [[round(float(q), 4) for q in step] for step in tq]
        compact_tcp = [[round(float(val), 2) for val in step] for step in tt]

        cleaned_pr = {
            "path_id": pr.get("path_id", 1),
            "name": pr.get("name", "Path 1"),
            "status": pr.get("status", "PASS"),
            "total_interpolated": pr.get("total_interpolated", len(tq)),
            "speed_mm_s": round(float(pr.get("speed_mm_s", 150.0)), 1),
            "recommended_safe_speed_mm_s": round(float(pr.get("recommended_safe_speed_mm_s", pr.get("speed_mm_s", 150.0))), 1),
            "peak_joint_speeds_deg_s": [round(float(v), 1) for v in pr.get("peak_joint_speeds_deg_s", [])],
            "issues": pr.get("issues", []),
            "trajectory_q": compact_q,
            "trajectory_tcp": compact_tcp,
        }
        cleaned_path_reports.append(cleaned_pr)

    return {
        "status": report.get("summary", {}).get("status", report.get("status", "PASS")),
        "summary": report.get("summary", {}),
        "nominal_speed_mm_s": round(float(report.get("nominal_speed_mm_s", 150.0)), 1),
        "slerp_step_mm": round(float(report.get("slerp_step_mm", 1.5)), 2),
        "max_joint_velocities_deg_s": report.get("max_joint_velocities_deg_s", []),
        "urdf_tcp": report.get("urdf_tcp", {}),
        "path_reports": cleaned_path_reports,
    }


def _cleanup_stale_reports(template_dir: str):
    """Removes obsolete separate .report.json files since they are now unified inside .path.yaml."""
    for item in os.listdir(template_dir):
        if item.endswith(".report.json") or item.endswith(".report.yaml"):
            try:
                os.remove(os.path.join(template_dir, item))
                logger.info(f"🧹 Cleaned legacy standalone report file: {item}")
            except Exception as e:
                logger.warning(f"Could not remove legacy report {item}: {e}")


import multiprocessing as mp
from logging.handlers import QueueHandler
import queue


def _boost_process_priority():
    """Cross-platform process prioritization and CPU core affinity booster."""
    if sys.platform == "darwin":
        try:
            import ctypes
            libsystem = ctypes.CDLL("/usr/lib/libSystem.dylib")
            libsystem.pthread_set_qos_class_self_np(0x21, 0)
        except Exception:
            pass
        return

    if sys.platform.startswith("linux"):
        try:
            os.setpriority(os.PRIO_PROCESS, 0, -10)
        except Exception:
            try:
                os.nice(-5)
            except Exception:
                pass

        if hasattr(os, "sched_setaffinity") and hasattr(os, "sched_getaffinity"):
            try:
                available_cpus = list(os.sched_getaffinity(0))
                if len(available_cpus) > 1:
                    cpu_freqs = {}
                    for cpu_id in available_cpus:
                        freq_path = f"/sys/devices/system/cpu/cpu{cpu_id}/cpufreq/cpuinfo_max_freq"
                        if os.path.exists(freq_path):
                            try:
                                with open(freq_path, "r") as f:
                                    cpu_freqs[cpu_id] = int(f.read().strip())
                            except Exception:
                                pass

                    if cpu_freqs:
                        max_f = max(cpu_freqs.values())
                        big_cores = {cid for cid, f in cpu_freqs.items() if f == max_f}
                        if big_cores and len(big_cores) < len(available_cpus):
                            os.sched_setaffinity(0, big_cores)
                    elif len(available_cpus) == 8 and set(range(8)).issubset(set(available_cpus)):
                        os.sched_setaffinity(0, {4, 5, 6, 7})
            except Exception:
                pass


def _poi_optimization_worker(
    log_queue: mp.Queue,
    res_conn,
    paths_data: dict,
    ref_rpy_deg: list[float],
    tolerance_rpy_deg: list[float],
    options: dict = None,
    anchor_source: str = "home",
):
    """Subprocess worker: runs POI optimization in an isolated process."""
    try:
        _boost_process_priority()

        root_logger = logging.getLogger()
        root_logger.setLevel(logging.INFO)
        root_logger.handlers = [QueueHandler(log_queue)]

        opts = options or {}
        step_size_mm = float(opts.get("step_size_mm", sprayer_config.slerp_step_mm))
        linear_velocity_mm_s = float(opts.get("linear_velocity_mm_s", sprayer_config.spraying_velocity))
        max_joint_vel = opts.get("max_joint_vel_deg_s", sprayer_config.max_joint_speed_deg_s)

        verifier = CR5PathVerifier(
            step_size_mm=step_size_mm,
            linear_velocity_mm_s=linear_velocity_mm_s,
            max_joint_vel_deg_s=max_joint_vel,
        )
        tcp_xyz = opts.get("tcp_offset_xyz_mm", None)
        tcp_rpy = opts.get("tcp_offset_rpy_deg", None)
        if tcp_xyz is not None and tcp_rpy is not None:
            verifier.set_tcp_offset(tcp_xyz, tcp_rpy)

        opt_data, opt_report = verifier.optimize_poi_all_paths(
            paths_data,
            ref_rpy_deg=ref_rpy_deg,
            tolerance_rpy_deg=tolerance_rpy_deg,
            anchor_source=anchor_source,
        )
        res_conn.send({"success": True, "opt_data": opt_data, "opt_report": opt_report})
    except Exception as e:
        import traceback
        res_conn.send({"success": False, "error": str(e), "traceback": traceback.format_exc()})
    finally:
        log_queue.put(None)


import threading


def _drain_log_queue(log_queue: mp.Queue, proc: mp.Process):
    while True:
        try:
            record = log_queue.get(timeout=0.1)
            if record is None:
                break
            dest_logger = logging.getLogger(record.name)
            dest_logger.handle(record)
        except queue.Empty:
            if not proc.is_alive():
                while True:
                    try:
                        record = log_queue.get_nowait()
                        if record is None:
                            break
                        logging.getLogger(record.name).handle(record)
                    except queue.Empty:
                        break
                break


def run_poi_optimization_subprocess(
    paths_data: dict,
    ref_rpy_deg: list[float],
    tolerance_rpy_deg: list[float],
    options: dict = None,
    anchor_source: str = "home",
) -> tuple[dict, dict]:
    """Spawns an isolated worker process to execute POI optimization."""
    ctx = mp.get_context("spawn")
    log_queue = ctx.Queue()
    parent_conn, child_conn = ctx.Pipe(duplex=False)

    proc = ctx.Process(
        target=_poi_optimization_worker,
        args=(log_queue, child_conn, paths_data, ref_rpy_deg, tolerance_rpy_deg, options, anchor_source),
    )
    proc.start()
    # 必须关掉父进程手里的写端: 否则子进程万一崩溃, 管道仍有写入者,
    # 下面的 recv() 收不到 EOF 会永久阻塞, 把路径优化接口和线程池的一个线程一起挂死
    child_conn.close()

    log_thread = threading.Thread(target=_drain_log_queue, args=(log_queue, proc), daemon=True)
    log_thread.start()

    res = None
    try:
        res = parent_conn.recv()
    except EOFError:
        pass

    log_thread.join(timeout=3.0)
    proc.join(timeout=3.0)
    if proc.is_alive():
        proc.terminate()
        proc.join()

    if res is None:
        raise RuntimeError(f"Optimization worker process terminated unexpectedly (exit code {proc.exitcode})")

    if not res.get("success"):
        tb = res.get("traceback", "")
        err = res.get("error", "Unknown error")
        logger.error(f"Worker optimization failed: {err}\n{tb}")
        raise RuntimeError(f"Optimization worker error: {err}")

    return res["opt_data"], res["opt_report"]


class PathVerificationService:
    """
    Dedicated service for robot kinematic chain verification and POI pose optimization.
    Unifies verification, simulation trajectory generation, and path persistence into single .path.yaml files.
    """
    def __init__(self):
        self.template_group_dir = os.path.abspath(os.path.join(PROJECT_ROOT, "data", "template_group"))

    def create_verifier(self, options: dict = None) -> CR5PathVerifier:
        """Factory helper: creates CR5PathVerifier reading configuration from aisprayer_config.yaml."""
        opts = options or {}
        step_size_mm = float(opts.get("step_size_mm", sprayer_config.slerp_step_mm))
        linear_velocity_mm_s = float(opts.get("linear_velocity_mm_s", sprayer_config.spraying_velocity))
        max_joint_vel = opts.get("max_joint_vel_deg_s", sprayer_config.max_joint_speed_deg_s)

        verifier = CR5PathVerifier(
            step_size_mm=step_size_mm,
            linear_velocity_mm_s=linear_velocity_mm_s,
            max_joint_vel_deg_s=max_joint_vel,
        )

        tcp_xyz = opts.get("tcp_offset_xyz_mm", None)
        tcp_rpy = opts.get("tcp_offset_rpy_deg", None)
        if tcp_xyz is not None and tcp_rpy is not None:
            verifier.set_tcp_offset(tcp_xyz, tcp_rpy)

        return verifier

    def get_urdf_tcp(self) -> dict:
        """Returns the active URDF tool TCP offset."""
        return CR5PathVerifier.load_tcp_from_urdf()

    def _path_file(self, template_name: str, state_type: str, *, existing: bool = True) -> str:
        state = _ensure_state_type(state_type)
        template_dir = os.path.join(self.template_group_dir, template_name)
        canonical = os.path.join(template_dir, _scan_path_filename(state))
        if not existing:
            return canonical
        return _resolve_named_file(template_dir, _scan_path_filename(state), PATH_YAML_LEGACY.get(state, ()))

    def verify_template_paths(self, template_name: str, state_type: str = "raw", options: dict = None) -> dict:
        """
        Runs offline kinematic chain verification on a template path yaml.
        Generates dense MoveL simulation trajectory and merges the verification report
        directly into the single unified .path.yaml file (de-duplicated).
        """
        state = _ensure_state_type(state_type)
        template_dir = os.path.join(self.template_group_dir, template_name)
        paths_file = self._path_file(template_name, state)

        if not os.path.exists(paths_file):
            logger.error(f"❌ [Verification Service] Required path file not found: {paths_file}")
            raise FileNotFoundError(f"Paths file '{_scan_path_filename(state)}' not found")

        logger.info(f"📂 [Verification Service] Loading '{paths_file}' for template '{template_name}' ({state})...")
        with open(paths_file, 'r', encoding='utf-8') as f:
            paths_data = fast_yaml_load(f)

        verifier = self.create_verifier(options)
        raw_report = verifier.verify_all_paths(paths_data)
        cleaned_report = _clean_report_data(raw_report)

        # De-duplicate: Clean waypoints and remove per-path duplicate blobs
        cleaned_paths = []
        for p in paths_data.get("paths", []):
            pts = _clean_waypoints_data(p.get("points", []))
            cleaned_p = {
                "path_id": p.get("path_id", len(cleaned_paths) + 1),
                "name": p.get("name", f"Path {len(cleaned_paths) + 1}"),
                "points": pts,
            }
            if "dense_surface_points_base_mm" in p:
                cleaned_p["dense_surface_points_base_mm"] = p["dense_surface_points_base_mm"]
            cleaned_paths.append(cleaned_p)

        unified_paths_data = {
            "standoff_distance_mm": paths_data.get("standoff_distance_mm", 150.0),
            "template": template_name,
            "type": state,
            "state_type": state,
            "updated_at": int(time.time()),
            "coordinate_frame": "base_link",
            "execution_speed_mm_s": verifier.linear_velocity_mm_s,
            "verification": cleaned_report,
            "paths": cleaned_paths,
        }

        # Save unified YAML
        canonical_path = os.path.join(template_dir, _scan_path_filename(state))
        with open(canonical_path, 'w', encoding='utf-8') as f:
            fast_yaml_dump(unified_paths_data, f)
        logger.info(f"💾 [Verification Service] Merged verification report & trajectory into unified file: {canonical_path}")

        # Clean up legacy .report.json files
        _cleanup_stale_reports(template_dir)

        logger.info(f"✅ [Verification Service] Verification finished for '{template_name}' ({state}): Status={cleaned_report['status']}")
        return cleaned_report

    def get_saved_report(self, template_name: str, state_type: str = "raw") -> Optional[dict]:
        """Retrieves saved verification report from unified .path.yaml."""
        state = _ensure_state_type(state_type)
        paths_file = self._path_file(template_name, state)
        if not os.path.exists(paths_file):
            return None
        try:
            with open(paths_file, 'r', encoding='utf-8') as f:
                data = fast_yaml_load(f)
            verif = data.get("verification")
            if verif:
                return verif
        except Exception as e:
            logger.warning(f"Failed to read verification from {paths_file}: {e}")
        return None

    def optimize_template_paths(
        self,
        template_name: str,
        mode: str = "poi",
        source: str = "raw",
        poi_config: dict = None,
        options: dict = None
    ) -> dict:
        """
        POI-optimizes trajectory from source ('raw' or 'auto') using aisprayer_config.yaml parameters.
        Saves optimized waypoints and merged verification/simulation report into unified .path.yaml.
        """
        source = (source or "raw").strip().lower()
        if source not in {"raw", "auto"}:
            source = "raw"

        out_state = "poi" if source == "raw" else "auto_poi"
        template_dir = os.path.join(self.template_group_dir, template_name)
        source_paths_file = self._path_file(template_name, source)
        if not os.path.exists(source_paths_file):
            logger.error(f"❌ [Verification Service] Optimization failed: '{_scan_path_filename(source)}' not found")
            raise FileNotFoundError(f"Original paths file '{_scan_path_filename(source)}' not found")

        logger.info(f"⚡ [Verification Service] Starting '{out_state}' optimization for '{template_name}' from '{source_paths_file}'...")
        with open(source_paths_file, 'r', encoding='utf-8') as f:
            paths_data = fast_yaml_load(f)

        verifier = self.create_verifier(options)

        # POI anchor & tolerance directly from config; a caller-supplied poi_config wins field by field
        # anchor_source: 'config' = spraying.poi_ref_rpy_deg, 'home' = Home 关节正解, 'raw' = 逐点名义法向
        anchor_source = sprayer_config.poi_anchor_source
        ref_rpy = sprayer_config.poi_ref_rpy_deg if anchor_source == "config" else None
        tol_rpy = get_default_poi_tolerance_rpy_deg()
        if poi_config and isinstance(poi_config, dict):
            if poi_config.get("anchor_source"):
                anchor_source = str(poi_config["anchor_source"]).strip().lower()
                ref_rpy = sprayer_config.poi_ref_rpy_deg if anchor_source == "config" else None
            if poi_config.get("tolerance_rpy_deg"):
                tol_rpy = list(poi_config["tolerance_rpy_deg"])
            # 调用方显式给出参考姿态时（live 已在接口层解析成具体姿态），以它为包络中心
            if poi_config.get("ref_rpy_deg"):
                ref_rpy = list(poi_config["ref_rpy_deg"])
                anchor_source = "config"

        if anchor_source == "config" and not ref_rpy:
            logger.warning("⚠️ [Verification Service] poi_anchor_source='config' 但 spraying.poi_ref_rpy_deg 未配置，退回 Home 正解锚点")

        logger.info(f"🚀 [Verification Service] Running POI optimization (anchor_source: {anchor_source}, ref_rpy: {ref_rpy}, tolerance: {tol_rpy})...")
        opt_data, opt_report = run_poi_optimization_subprocess(
            paths_data,
            ref_rpy_deg=ref_rpy,
            tolerance_rpy_deg=tol_rpy,
            options=options,
            anchor_source=anchor_source,
        )

        cleaned_report = _clean_report_data(opt_report)

        # Record the POI constraints actually used by the optimizer (anchor derived inside worker when ref_rpy is None)
        effective_poi_config = opt_data.get("poi_config") or {
            "mode": "per_waypoint_nominal_envelope" if anchor_source == "raw" else "absolute_anchor_tolerance",
            "anchor_source": anchor_source,
            "ref_rpy_deg": ref_rpy,
            "tolerance_rpy_deg": tol_rpy,
            "euler_order": "xyz",
            "units": "deg",
        }

        # De-duplicate: Clean waypoints and strip duplicate trajectory arrays inside each path
        cleaned_paths = []
        for p in opt_data.get("paths", []):
            pts = _clean_waypoints_data(p.get("points", []))
            cleaned_p = {
                "path_id": p.get("path_id", len(cleaned_paths) + 1),
                "name": p.get("name", f"Path {len(cleaned_paths) + 1}"),
                "points": pts,
            }
            if "dense_surface_points_base_mm" in p:
                cleaned_p["dense_surface_points_base_mm"] = p["dense_surface_points_base_mm"]
            cleaned_paths.append(cleaned_p)

        target_yaml = _scan_path_filename(out_state)
        opt_file = os.path.join(template_dir, target_yaml)

        unified_data = {
            "standoff_distance_mm": paths_data.get("standoff_distance_mm", 150.0),
            "template": template_name,
            "type": out_state,
            "state_type": out_state,
            "source_file": _scan_path_filename(source),
            "updated_at": int(time.time()),
            "coordinate_frame": "base_link",
            "execution_speed_mm_s": verifier.linear_velocity_mm_s,
            "poi_config": effective_poi_config,
            "verification": cleaned_report,
            "paths": cleaned_paths,
        }

        with open(opt_file, 'w', encoding='utf-8') as f:
            fast_yaml_dump(unified_data, f)
        logger.info(f"💾 [Verification Service] Saved unified optimized {out_state} file to: {opt_file}")

        # Clean up legacy .report.json files
        _cleanup_stale_reports(template_dir)

        cleaned_report["optimized_paths"] = cleaned_paths
        cleaned_report["saved_file"] = target_yaml
        logger.info(f"🎉 [Verification Service] Optimization ({out_state}) completed for '{template_name}': Status={cleaned_report['status']}")
        return cleaned_report


path_verification_service = PathVerificationService()
