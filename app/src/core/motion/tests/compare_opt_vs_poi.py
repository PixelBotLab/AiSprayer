#!/usr/bin/env python3
"""对比 C++ 优化结果 与 重构前 scan.auto.poi.path.yaml。"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml


def pose(wp: dict) -> tuple[list[float], list[float]]:
    p = wp.get("tcp_pose_base", wp)
    xyz = [float(p["x"]), float(p["y"]), float(p["z"])]
    rpy = [float(p.get("rx", 0)), float(p.get("ry", 0)), float(p.get("rz", 0))]
    return xyz, rpy


def rot_xyz(rx: float, ry: float, rz: float):
    ax, ay, az = map(math.radians, (rx, ry, rz))
    cx, sx = math.cos(ax), math.sin(ax)
    cy, sy = math.cos(ay), math.sin(ay)
    cz, sz = math.cos(az), math.sin(az)
    # R = Rz @ Ry @ Rx
    return (
        (cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
        (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
        (-sy, cy * sx, cy * cx),
    )


def geodesic_deg(a, b) -> float:
    tr = a[0][0] * b[0][0] + a[0][1] * b[0][1] + a[0][2] * b[0][2]
    tr += a[1][0] * b[1][0] + a[1][1] * b[1][1] + a[1][2] * b[1][2]
    tr += a[2][0] * b[2][0] + a[2][1] * b[2][1] + a[2][2] * b[2][2]
    c = max(-1.0, min(1.0, 0.5 * (tr - 1.0)))
    return math.degrees(math.acos(c))


def load_points(path: Path) -> list[dict]:
    with path.open() as f:
        doc = yaml.safe_load(f)
    paths = doc.get("paths") or [{"points": doc.get("points", [])}]
    return list(paths[0].get("points") or [])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", required=True, help="未优化 scan.auto.path.yaml")
    ap.add_argument("--poi", required=True, help="重构前 scan.auto.poi.path.yaml")
    ap.add_argument("--cpp", required=True, help="C++ optimize 输出 yaml")
    args = ap.parse_args()

    auto = load_points(Path(args.auto))
    poi = load_points(Path(args.poi))
    cpp = load_points(Path(args.cpp))
    n = min(len(auto), len(poi), len(cpp))
    if len({len(auto), len(poi), len(cpp)}) != 1:
        print(f"waypoint 数量不一致: auto={len(auto)} poi={len(poi)} cpp={len(cpp)}，按 {n} 点对比")

    pos_err = []
    geo_poi = []
    geo_auto = []
    rpy_d = []
    for i in range(n):
        a_xyz, a_rpy = pose(auto[i])
        p_xyz, p_rpy = pose(poi[i])
        c_xyz, c_rpy = pose(cpp[i])
        dpos = max(abs(c_xyz[k] - p_xyz[k]) for k in range(3))
        dpos_auto = max(abs(c_xyz[k] - a_xyz[k]) for k in range(3))
        pos_err.append(dpos)
        Ra, Rp, Rc = rot_xyz(*a_rpy), rot_xyz(*p_rpy), rot_xyz(*c_rpy)
        geo_poi.append(geodesic_deg(Rc, Rp))
        geo_auto.append(geodesic_deg(Rc, Ra))
        rpy_d.append([abs(((c_rpy[k] - p_rpy[k] + 180) % 360) - 180) for k in range(3)])

    def stats(xs: list[float]) -> str:
        return f"max={max(xs):.4f} mean={sum(xs)/len(xs):.4f} median={sorted(xs)[len(xs)//2]:.4f}"

    print("=== C++ opt vs 重构前 POI ===")
    print(f"点数 {n}")
    print(f"位置误差 mm (相对 POI): {stats(pos_err)}")
    print(f"姿态测地角 ° (相对 POI): {stats(geo_poi)}")
    print(f"姿态测地角 ° (相对 auto 名义): {stats(geo_auto)}")
    rx, ry, rz = zip(*rpy_d)
    print(f"|Δrx|° vs POI: {stats(list(rx))}")
    print(f"|Δry|° vs POI: {stats(list(ry))}")
    print(f"|Δrz|° vs POI: {stats(list(rz))}")

    close = sum(1 for g in geo_poi if g < 1e-3)
    near = sum(1 for g in geo_poi if g < 1.0)
    print(f"与 POI 几乎相同 (<0.001°): {close}/{n}")
    print(f"与 POI 接近 (<1°): {near}/{n}")

    # 最大差异的几个点
    order = sorted(range(n), key=lambda i: geo_poi[i], reverse=True)[:8]
    print("差异最大的航点 (index, geo_vs_poi°, Δrpy vs poi, cpp_rpy, poi_rpy):")
    for i in order:
        _, p_rpy = pose(poi[i])
        _, c_rpy = pose(cpp[i])
        print(
            f"  wp={auto[i].get('index', i+1):3d}  geo={geo_poi[i]:7.3f}  "
            f"Δrpy=({rpy_d[i][0]:6.2f},{rpy_d[i][1]:6.2f},{rpy_d[i][2]:6.2f})  "
            f"cpp=({c_rpy[0]:7.2f},{c_rpy[1]:7.2f},{c_rpy[2]:7.2f})  "
            f"poi=({p_rpy[0]:7.2f},{p_rpy[1]:7.2f},{p_rpy[2]:7.2f})"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
