#!/usr/bin/env python3
"""用重构前的 Python 优化链路再生成一份 auto_poi yaml（不改动模板目录里的原文件）。"""

import os
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../.."))
sys.path[:0] = [ROOT, os.path.join(ROOT, "app", "src")]

import yaml  # noqa: E402

from apps.interactive.path_verification_service import (  # noqa: E402
    _clean_report_data,
    _clean_waypoints_data,
)
from core.hardware.robot.cr5_path_verifier import CR5PathVerifier  # noqa: E402


def main() -> int:
    src = os.path.join(ROOT, "data/template_group/2026-09-03_225937/scan.auto.path.yaml")
    out = os.path.join(ROOT, "app/src/core/motion/build/out/scan.auto.py.poi.path.yaml")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    with open(src, encoding="utf-8") as f:
        paths_data = yaml.safe_load(f)

    t0 = time.perf_counter()
    verifier = CR5PathVerifier(step_size_mm=1.5, linear_velocity_mm_s=120.0)
    opt_data, opt_report = verifier.optimize_poi_all_paths(
        paths_data,
        ref_rpy_deg=[90.0, 0.0, 90.0],
        tolerance_rpy_deg=[10.0, 10.0, 30.0],
        anchor_source="config",
    )
    cleaned_report = _clean_report_data(opt_report)
    cleaned_paths = []
    for p in opt_data.get("paths", []):
        item = {
            "path_id": p.get("path_id", len(cleaned_paths) + 1),
            "name": p.get("name", f"Path {len(cleaned_paths) + 1}"),
            "points": _clean_waypoints_data(p.get("points", [])),
        }
        if "dense_surface_points_base_mm" in p:
            item["dense_surface_points_base_mm"] = p["dense_surface_points_base_mm"]
        cleaned_paths.append(item)

    unified = {
        "standoff_distance_mm": paths_data.get("standoff_distance_mm", 150.0),
        "template": "2026-09-03_225937",
        "type": "auto_poi",
        "state_type": "auto_poi",
        "source_file": "scan.auto.path.yaml",
        "updated_at": int(time.time()),
        "coordinate_frame": "base_link",
        "execution_speed_mm_s": verifier.linear_velocity_mm_s,
        "poi_config": opt_data.get("poi_config")
        or {
            "mode": "absolute_anchor_tolerance",
            "anchor_source": "config",
            "ref_rpy_deg": [90.0, 0.0, 90.0],
            "tolerance_rpy_deg": [10.0, 10.0, 30.0],
            "euler_order": "xyz",
            "units": "deg",
        },
        "verification": cleaned_report,
        "paths": cleaned_paths,
    }
    with open(out, "w", encoding="utf-8") as f:
        yaml.safe_dump(unified, f, sort_keys=False, allow_unicode=True)
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    backend = verifier.solver.backend
    print(f"py_out={out}")
    print(f"py_backend={backend}")
    print(f"py_wall_ms={elapsed_ms:.3f}")
    print(f"py_status={cleaned_report.get('status')}")
    print(f"py_steps={cleaned_report.get('summary', {}).get('total_steps')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
