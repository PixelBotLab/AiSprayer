#!/usr/bin/env python3
"""Compare vision_cli against the existing Python recon / auto-path on a template."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

APP_SRC = Path(__file__).resolve().parents[3]
REPO = APP_SRC.parent.parent
if str(APP_SRC) not in sys.path:
    sys.path.insert(0, str(APP_SRC))

from core.config import SprayerConfig  # noqa: E402
from core.vision.jeans_auto_waypoints import JeansAutoWaypoints  # noqa: E402
from core.vision.reconstruction import PoissonReconstructor, depth_to_point_cloud, k_matrix_to_intrinsics  # noqa: E402


def _latest_calib(repo: Path) -> Path:
    calib_dir = repo / "data" / "calib"
    if calib_dir.is_dir():
        sessions = sorted([p for p in calib_dir.iterdir() if p.is_dir()], reverse=True)
        for sess in sessions:
            cand = sess / "calibration_result.yaml"
            if cand.is_file():
                return cand
    fallback = repo / "configs" / "calib" / "calibration_result.yaml"
    if fallback.is_file():
        return fallback
    raise FileNotFoundError("no calibration_result.yaml found")


def _load_T_and_K(calib_path: Path, template_dir: Path):
    with open(calib_path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    t_mat = data.get("T_base_camera") or data.get("T_camera_to_base")
    if not t_mat:
        raise RuntimeError("calib missing T")
    T = np.array(t_mat, dtype=np.float64)
    T[0, 3] /= 1000.0
    T[1, 3] /= 1000.0
    T[2, 3] /= 1000.0
    k = None
    params = template_dir / "scan.params.yaml"
    if params.is_file():
        with open(params, encoding="utf-8") as f:
            pdata = yaml.safe_load(f) or {}
        k_list = (pdata.get("camera_params") or {}).get("intrinsic_matrix")
        if k_list:
            k = np.array(k_list, dtype=np.float64)
    if k is None:
        cam = data.get("camera_params") or {}
        if "intrinsic_matrix" in cam:
            k = np.array(cam["intrinsic_matrix"], dtype=np.float64)
    if k is None:
        raise RuntimeError("K missing")
    return T, k


def _process_params():
    cfg = SprayerConfig()
    spray = float(cfg.spray_distance_mm)
    row = float(cfg.row_spacing_mm)
    point = float(cfg.point_spacing_mm)
    return spray, row, point, 0.5 * row


def _run_python_auto(template_dir: Path, T, k, out_yaml: Path, spray, row, point, dedup):
    import trimesh

    mesh = trimesh.load(str(template_dir / "scan.mesh.ply"), force="mesh", process=False)
    with open(template_dir / "scan.masks.yaml", encoding="utf-8") as f:
        masks = yaml.safe_load(f) or {}
    cam = {}
    with open(template_dir / "scan.params.yaml", encoding="utf-8") as f:
        cam = (yaml.safe_load(f) or {}).get("camera_params") or {}
    image_size = (int(cam.get("width") or 1280), int(cam.get("height") or 800))
    t0 = time.perf_counter()
    planned = JeansAutoWaypoints(
        spray_dist_mm=spray,
        row_spacing_mm=row,
        point_spacing_mm=point,
        image_size=image_size,
        camera_intrinsics=k,
        T_camera_to_base=T,
        dedup_radius_mm=dedup,
        mesh_unit="m",
        align_outer_edge=True,
    ).plan(mesh, masks)
    elapsed = (time.perf_counter() - t0) * 1000.0
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    with open(out_yaml, "w", encoding="utf-8") as f:
        yaml.safe_dump(planned, f, allow_unicode=True, sort_keys=False)
    n = sum(len(p.get("points") or []) for p in planned.get("paths") or [])
    return planned, n, elapsed


def _run_cli(cli: Path, args: list[str]) -> dict:
    proc = subprocess.run([str(cli), *args], capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(
            f"vision_cli failed ({proc.returncode})\nstdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    line = [ln for ln in proc.stdout.strip().splitlines() if ln.strip()][-1]
    return json.loads(line)


def _geodesic_deg(rpy_a, rpy_b):
    from scipy.spatial.transform import Rotation as R

    Ra = R.from_euler("xyz", rpy_a, degrees=True)
    Rb = R.from_euler("xyz", rpy_b, degrees=True)
    return float(np.degrees((Ra.inv() * Rb).magnitude()))


def _overlay_template(src: Path, mesh_ply: Path, dest: Path) -> Path:
    dest.mkdir(parents=True, exist_ok=True)
    for name in ("scan.masks.yaml", "scan.params.yaml", "scan.depth.png"):
        s = src / name
        if not s.exists():
            continue
        t = dest / name
        if t.exists() or t.is_symlink():
            t.unlink()
        t.symlink_to(s.resolve())
    target = dest / "scan.mesh.ply"
    if target.exists() or target.is_symlink():
        target.unlink()
    target.symlink_to(Path(mesh_ply).resolve())
    return dest


def _auto_points(doc):
    return (doc.get("paths") or [{}])[0].get("points") or []


def _compare_auto_nn(a_doc, b_doc, pos_mm=1.0, ang_deg=2.0):
    """Nearest-neighbor match when two meshes produce different waypoint counts."""
    from scipy.spatial import cKDTree

    a = _auto_points(a_doc)
    b = _auto_points(b_doc)
    report = {
        "a_count": len(a),
        "b_count": len(b),
        "matched": 0,
        "unmatched_a": len(a),
        "max_surface_mm": None,
        "p95_surface_mm": None,
        "max_tcp_mm": None,
        "max_rpy_deg": None,
        "zero_normals": 0,
        "pass": False,
    }
    if not a or not b:
        return report
    pb = np.array([p["surface_point_base_mm"] for p in b], dtype=np.float64)
    tree = cKDTree(pb)
    pa = np.array([p["surface_point_base_mm"] for p in a], dtype=np.float64)
    dist, idx = tree.query(pa, k=1)
    surf, tcp, ang = [], [], []
    for i, j in enumerate(idx):
        surf.append(float(dist[i]))
        ta = np.array([a[i]["tcp_pose_base"][k] for k in ("x", "y", "z")], dtype=np.float64)
        tb = np.array([b[j]["tcp_pose_base"][k] for k in ("x", "y", "z")], dtype=np.float64)
        tcp.append(float(np.linalg.norm(ta - tb)))
        ang.append(_geodesic_deg(
            [a[i]["tcp_pose_base"][k] for k in ("rx", "ry", "rz")],
            [b[j]["tcp_pose_base"][k] for k in ("rx", "ry", "rz")],
        ))
        nb = np.array(b[j].get("surface_normal_base") or [0, 0, 0], dtype=np.float64)
        if np.linalg.norm(nb) < 1e-6:
            report["zero_normals"] += 1
    report["matched"] = int(len(a))
    report["unmatched_a"] = 0
    report["max_surface_mm"] = max(surf)
    report["p95_surface_mm"] = float(np.percentile(surf, 95))
    report["max_tcp_mm"] = max(tcp)
    report["p95_tcp_mm"] = float(np.percentile(tcp, 95))
    report["max_rpy_deg"] = max(ang)
    report["p95_rpy_deg"] = float(np.percentile(ang, 95))
    report["pass"] = (
        report["p95_surface_mm"] <= pos_mm
        and report["p95_tcp_mm"] <= pos_mm
        and report["p95_rpy_deg"] <= ang_deg
        and report["zero_normals"] == 0
        and abs(len(a) - len(b)) <= max(2, int(0.05 * max(len(a), len(b))))
    )
    return report


def _run_auto_pair(cli: Path, template: Path, calib: Path, T, k, out_dir: Path,
                   spray, row, point, dedup, pos_mm, ang_deg, tag: str):
    py_yaml = out_dir / f"python_{tag}.path.yaml"
    cpp_yaml = out_dir / f"cpp_{tag}.path.yaml"
    py_doc, py_n, py_ms = _run_python_auto(template, T, k, py_yaml, spray, row, point, dedup)
    print(f"python auto-path [{tag}]: {py_n} points in {py_ms:.1f} ms")
    cpp_json = _run_cli(cli, [
        "auto-path",
        "--template-dir", str(template),
        "--calib", str(calib),
        "--spray-dist", str(spray),
        "--row-spacing", str(row),
        "--point-spacing", str(point),
        "--dedup-radius", str(dedup),
        "--output", str(cpp_yaml),
    ])
    print(f"cpp auto-path [{tag}]: {cpp_json}")
    with open(cpp_yaml, encoding="utf-8") as f:
        cpp_doc = yaml.safe_load(f) or {}
    rep = _compare_auto(py_doc, cpp_doc, pos_mm, ang_deg)
    print(f"auto-path compare [{tag}]:", json.dumps(rep, indent=2))
    return py_doc, cpp_doc, rep


def _compare_auto(py_doc, cpp_doc, pos_mm=1.0, ang_deg=2.0):
    py = (py_doc.get("paths") or [{}])[0].get("points") or []
    cpp = (cpp_doc.get("paths") or [{}])[0].get("points") or []
    report = {
        "python_count": len(py),
        "cpp_count": len(cpp),
        "count_match": len(py) == len(cpp),
        "max_surface_mm": None,
        "p95_surface_mm": None,
        "max_tcp_mm": None,
        "max_rpy_deg": None,
        "zero_normals": 0,
        "pass": False,
    }
    if not py or not cpp:
        return report
    n = min(len(py), len(cpp))
    surf, tcp, ang = [], [], []
    for i in range(n):
        a, b = py[i], cpp[i]
        pa = np.array(a["surface_point_base_mm"], dtype=np.float64)
        pb = np.array(b["surface_point_base_mm"], dtype=np.float64)
        surf.append(float(np.linalg.norm(pa - pb)))
        ta = np.array([a["tcp_pose_base"][k] for k in ("x", "y", "z")], dtype=np.float64)
        tb = np.array([b["tcp_pose_base"][k] for k in ("x", "y", "z")], dtype=np.float64)
        tcp.append(float(np.linalg.norm(ta - tb)))
        ang.append(_geodesic_deg(
            [a["tcp_pose_base"][k] for k in ("rx", "ry", "rz")],
            [b["tcp_pose_base"][k] for k in ("rx", "ry", "rz")],
        ))
        nb = np.array(b.get("surface_normal_base") or [0, 0, 0], dtype=np.float64)
        if np.linalg.norm(nb) < 1e-6:
            report["zero_normals"] += 1
    report["max_surface_mm"] = max(surf)
    report["p95_surface_mm"] = float(np.percentile(surf, 95))
    report["max_tcp_mm"] = max(tcp)
    report["max_rpy_deg"] = max(ang)
    report["pass"] = (
        report["count_match"]
        and report["max_surface_mm"] <= pos_mm
        and report["max_tcp_mm"] <= pos_mm
        and report["max_rpy_deg"] <= ang_deg
        and report["zero_normals"] == 0
    )
    return report


def _python_point_cloud(template_dir: Path, T, k):
    depth = cv2.imread(str(template_dir / "scan.depth.png"), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise RuntimeError("cannot read scan.depth.png")
    h, w = depth.shape[:2]
    from apps.interactive.reconstruction_service import reconstruction_service

    mask = reconstruction_service.rasterize_masks(str(template_dir / "scan.masks.yaml"), h, w)
    recon = PoissonReconstructor(
        T_camera_to_base=T,
        intrinsics_k=k,
        segmenter=None,
        z_min=100,
        z_max=3000,
        mask_erode_px=1,
        flying_pixel_max_grad=50.0,
        poisson_depth=8,
        density_threshold=0.15,
        voxel_size=0.003,
        normal_radius=0.03,
        smooth_iterations=20,
    )
    valid = (depth > recon.z_min) & (depth < recon.z_max)
    holes = mask & (~valid)
    if np.any(holes):
        filled = cv2.inpaint(depth.astype(np.float32), holes.astype(np.uint8) * 255, 5, cv2.INPAINT_NS)
        depth = depth.astype(np.float32)
        depth[holes] = filled[holes]
        valid = (depth > recon.z_min) & (depth < recon.z_max)
    else:
        depth = depth.astype(np.float32)
    eroded = recon._erode_mask(mask, recon.mask_erode_px)
    flying = recon._flying_pixel_mask(depth, recon.flying_pixel_max_grad)
    combined = eroded & valid & flying
    raw = depth_to_point_cloud(depth, k_matrix_to_intrinsics(k))
    pts = raw[combined] / 1000.0
    ones = np.ones((pts.shape[0], 1))
    base = (T @ np.hstack((pts, ones)).T).T[:, :3]
    return base


def _hausdorff_p95(mesh_vertices, cloud, sample=4000):
    from scipy.spatial import cKDTree

    tree = cKDTree(cloud)
    v = np.asarray(mesh_vertices, dtype=np.float64)
    if len(v) > sample:
        rng = np.random.default_rng(0)
        v = v[rng.choice(len(v), sample, replace=False)]
    d, _ = tree.query(v, k=1)
    return float(np.max(d) * 1000.0), float(np.percentile(d, 95) * 1000.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--template-dir", required=True)
    parser.add_argument("--calib", default="")
    parser.add_argument("--vision-cli", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--skip-recon", action="store_true")
    parser.add_argument("--mesh", default="", help="optional ply override for extra auto-path check")
    parser.add_argument("--pos-mm", type=float, default=1.0)
    parser.add_argument("--ang-deg", type=float, default=2.0)
    args = parser.parse_args()

    template = Path(args.template_dir).resolve()
    repo = REPO
    calib = Path(args.calib).resolve() if args.calib else _latest_calib(repo)
    cli = Path(args.vision_cli).resolve()
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    T, k = _load_T_and_K(calib, template)
    spray, row, point, dedup = _process_params()
    print(f"calib={calib}")
    print(f"params spray={spray} row={row} point={point} dedup={dedup}")

    py_old, cpp_old, auto_rep = _run_auto_pair(
        cli, template, calib, T, k, out_dir, spray, row, point, dedup,
        args.pos_mm, args.ang_deg, "old_mesh",
    )

    recon_rep = {"skipped": True}
    if not args.skip_recon:
        recon_out = out_dir / "recon"
        recon_out.mkdir(exist_ok=True)
        t0 = time.perf_counter()
        recon_json = _run_cli(cli, [
            "recon",
            "--template-dir", str(template),
            "--calib", str(calib),
            "--output-dir", str(recon_out),
        ])
        recon_ms = (time.perf_counter() - t0) * 1000.0
        print(f"cpp recon: {recon_json} wall_ms={recon_ms:.1f}")
        import trimesh

        new_mesh = trimesh.load(str(recon_out / "scan.mesh.ply"), force="mesh", process=False)
        old_mesh = trimesh.load(str(template / "scan.mesh.ply"), force="mesh", process=False)
        cloud = _python_point_cloud(template, T, k)
        h_new, p95_new = _hausdorff_p95(new_mesh.vertices, cloud)
        h_old, p95_old = _hausdorff_p95(old_mesh.vertices, cloud)
        cloud_to_new, cloud_p95_new = _hausdorff_p95(cloud, new_mesh.vertices, sample=8000)
        cloud_to_old, cloud_p95_old = _hausdorff_p95(cloud, old_mesh.vertices, sample=8000)

        def _bbox(v):
            v = np.asarray(v, dtype=np.float64)
            return {
                "min_m": v.min(axis=0).tolist(),
                "max_m": v.max(axis=0).tolist(),
                "extent_mm": ((v.max(axis=0) - v.min(axis=0)) * 1000.0).tolist(),
            }

        def _vertex_components(mesh):
            faces = np.asarray(mesh.faces, dtype=np.int64)
            n = int(len(mesh.vertices))
            if n == 0 or len(faces) == 0:
                return 0
            parent = np.arange(n, dtype=np.int64)

            def find(x):
                while parent[x] != x:
                    parent[x] = parent[parent[x]]
                    x = parent[x]
                return x

            def union(a, b):
                ra, rb = find(a), find(b)
                if ra != rb:
                    parent[rb] = ra

            for f in faces:
                union(int(f[0]), int(f[1]))
                union(int(f[1]), int(f[2]))
            used = np.zeros(n, dtype=bool)
            used[faces.reshape(-1)] = True
            return len({int(find(i)) for i in range(n) if used[i]})

        cpp_cc = _vertex_components(new_mesh)
        py_cc = _vertex_components(old_mesh)
        recon_rep = {
            "skipped": False,
            "cpp": recon_json,
            "cpp_to_cloud_max_mm": h_new,
            "cpp_to_cloud_p95_mm": p95_new,
            "cloud_to_cpp_max_mm": cloud_to_new,
            "cloud_to_cpp_p95_mm": cloud_p95_new,
            "python_ply_to_cloud_max_mm": h_old,
            "python_ply_to_cloud_p95_mm": p95_old,
            "cloud_to_python_p95_mm": cloud_p95_old,
            "cpp_faces": int(len(new_mesh.faces)),
            "cpp_vertices": int(len(new_mesh.vertices)),
            "python_faces": int(len(old_mesh.faces)),
            "python_vertices": int(len(old_mesh.vertices)),
            "cpp_components": int(cpp_cc),
            "python_components": int(py_cc),
            "cpp_bbox": _bbox(new_mesh.vertices),
            "python_bbox": _bbox(old_mesh.vertices),
            "pass": (
                p95_new < 15.0
                and cloud_p95_new < 25.0
                and int(len(new_mesh.faces)) > 1000
                and int(cpp_cc) >= 1
            ),
        }
        print("recon compare:", json.dumps(recon_rep, indent=2))

    new_mesh_ply = Path(args.mesh).resolve() if args.mesh else None
    if new_mesh_ply is None and not recon_rep.get("skipped"):
        cand = out_dir / "recon" / "scan.mesh.ply"
        if cand.is_file():
            new_mesh_ply = cand
    auto_new = {"skipped": True}
    auto_cross = {"skipped": True}
    if new_mesh_ply is not None and new_mesh_ply.is_file():
        overlay = _overlay_template(template, new_mesh_ply, out_dir / "template_new_mesh")
        py_new, cpp_new, auto_new = _run_auto_pair(
            cli, overlay, calib, T, k, out_dir, spray, row, point, dedup,
            args.pos_mm, args.ang_deg, "new_mesh",
        )
        auto_new["mesh"] = str(new_mesh_ply)
        # Same planner, different mesh: nearest-neighbor, looser spray-relevant band.
        auto_cross = _compare_auto_nn(py_old, cpp_new, pos_mm=15.0, ang_deg=10.0)
        auto_cross["note"] = "old-mesh Python vs new-mesh C++ (nearest surface point)"
        print("auto-path cross old→new:", json.dumps(auto_cross, indent=2))

    summary = {
        "auto_path_old_mesh": auto_rep,
        "auto_path": auto_rep,
        "auto_path_new_mesh": auto_new,
        "auto_path_old_vs_new_mesh": auto_cross,
        "recon": recon_rep,
    }
    with open(out_dir / "compare_report.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    ok = bool(auto_rep.get("pass")) and (recon_rep.get("skipped") or recon_rep.get("pass"))
    if not auto_new.get("skipped") and not auto_new.get("pass"):
        ok = False
    if not ok:
        print("COMPARE FAIL")
        return 1
    print("COMPARE PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
