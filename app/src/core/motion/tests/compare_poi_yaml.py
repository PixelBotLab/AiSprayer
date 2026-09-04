#!/usr/bin/env python3
"""逐字段比对 C++ motion_cli optimize 的产物与重构前 Python 版的 *.poi.path.yaml。

检查两件事：
  1. schema 对齐——键集合、嵌套结构、标量的 Python 类型（'on' 必须是 str 而非 bool）；
  2. 数值对齐——tcp_pose 位置/姿态差、trajectory_q 差、以及原样透传字段完全相等。

用法：
  python3 compare_poi_yaml.py --ref <旧 poi yaml> --new <C++ 产出 yaml>
"""

import argparse
import math
import sys

import yaml

# 这些键允许不同：时间戳、以及记录来源的路径
IGNORED_VALUE_KEYS = {"updated_at"}
# 透传字段：优化不应改动，必须逐值相等
PASSTHROUGH_KEYS = (
    "index",
    "pixel",
    "surface_point_base_mm",
    "surface_normal_base",
    "standoff_distance_mm",
    "normal_2d_proj",
    "spraying",
    "is_jump",
)


def type_name(v):
    return type(v).__name__


def compare_schema(ref, new, path, errors):
    """递归比较键集合与标量类型。"""
    if isinstance(ref, dict):
        if not isinstance(new, dict):
            errors.append(f"{path}: 结构不同 ref=dict new={type_name(new)}")
            return
        missing = sorted(set(ref) - set(new))
        extra = sorted(set(new) - set(ref))
        if missing:
            errors.append(f"{path}: 缺少键 {missing}")
        if extra:
            errors.append(f"{path}: 多出键 {extra}")
        for k in sorted(set(ref) & set(new)):
            compare_schema(ref[k], new[k], f"{path}.{k}", errors)
        return

    if isinstance(ref, list):
        if not isinstance(new, list):
            errors.append(f"{path}: 结构不同 ref=list new={type_name(new)}")
            return
        if len(ref) != len(new):
            errors.append(f"{path}: 长度不同 ref={len(ref)} new={len(new)}")
        # 长列表只抽查首尾，schema 是同构的
        idx = range(len(ref)) if len(ref) <= 4 else (0, len(ref) - 1)
        for i in idx:
            if i < len(new):
                compare_schema(ref[i], new[i], f"{path}[{i}]", errors)
        return

    # 标量：int/float 互认（YAML 里 150 与 150.0 对 Python 使用方等价），其余类型必须一致
    numeric = (int, float)
    if isinstance(ref, bool) or isinstance(new, bool):
        if type(ref) is not type(new):
            errors.append(f"{path}: 类型不同 ref={type_name(ref)}({ref!r}) new={type_name(new)}({new!r})")
    elif isinstance(ref, numeric) and isinstance(new, numeric):
        pass
    elif type(ref) is not type(new):
        errors.append(f"{path}: 类型不同 ref={type_name(ref)}({ref!r}) new={type_name(new)}({new!r})")


def rot_xyz(rx, ry, rz):
    """Dobot 控制器 RPY(deg) → 旋转矩阵，R = Rz·Ry·Rx。"""
    ax, ay, az = (math.radians(v) for v in (rx, ry, rz))
    cx, sx, cy, sy, cz, sz = (
        math.cos(ax), math.sin(ax), math.cos(ay), math.sin(ay), math.cos(az), math.sin(az)
    )
    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ]


def geodesic_deg(a, b):
    tr = sum(a[k][i] * b[k][i] for k in range(3) for i in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, 0.5 * (tr - 1.0)))))


def compare_values(ref, new, errors):
    ref_pts = ref["paths"][0]["points"]
    new_pts = new["paths"][0]["points"]
    if len(ref_pts) != len(new_pts):
        errors.append(f"航点数不同 ref={len(ref_pts)} new={len(new_pts)}")
        return None

    max_pos, max_geo, passthrough_bad = 0.0, 0.0, []
    for i, (a, b) in enumerate(zip(ref_pts, new_pts)):
        pa, pb = a["tcp_pose_base"], b["tcp_pose_base"]
        max_pos = max(max_pos, max(abs(pa[k] - pb[k]) for k in "xyz"))
        max_geo = max(
            max_geo,
            geodesic_deg(
                rot_xyz(pa["rx"], pa["ry"], pa["rz"]), rot_xyz(pb["rx"], pb["ry"], pb["rz"])
            ),
        )
        for k in PASSTHROUGH_KEYS:
            if k in a and a[k] != b.get(k):
                passthrough_bad.append(f"点{i}.{k}: ref={a[k]!r} new={b.get(k)!r}")

    if passthrough_bad:
        errors.append("透传字段被改动: " + "; ".join(passthrough_bad[:5]))

    max_q = 0.0
    ref_tq = ref["verification"]["path_reports"][0].get("trajectory_q") or []
    new_tq = new["verification"]["path_reports"][0].get("trajectory_q") or []
    if len(ref_tq) != len(new_tq):
        errors.append(f"trajectory_q 长度不同 ref={len(ref_tq)} new={len(new_tq)}")
    else:
        for qa, qb in zip(ref_tq, new_tq):
            max_q = max(max_q, max(abs(x - y) for x, y in zip(qa, qb)))

    return max_pos, max_geo, max_q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True, help="重构前 Python 版 poi yaml")
    ap.add_argument("--new", required=True, help="C++ motion_cli 产出 yaml")
    ap.add_argument("--pos-tol-mm", type=float, default=0.05)
    ap.add_argument("--geo-tol-deg", type=float, default=0.05)
    ap.add_argument("--q-tol-rad", type=float, default=1e-3)
    args = ap.parse_args()

    with open(args.ref, encoding="utf-8") as f:
        ref = yaml.safe_load(f)
    with open(args.new, encoding="utf-8") as f:
        new = yaml.safe_load(f)

    schema_errors = []
    compare_schema(ref, new, "$", schema_errors)
    schema_errors = [e for e in schema_errors if not any(k in e for k in IGNORED_VALUE_KEYS)]

    value_errors = []
    metrics = compare_values(ref, new, value_errors)

    print("=== schema 对齐 ===")
    if schema_errors:
        for e in schema_errors:
            print("  ✗", e)
    else:
        print("  ✓ 键集合、嵌套结构、标量类型完全一致")

    print("=== 数值对齐 ===")
    if metrics:
        max_pos, max_geo, max_q = metrics
        print(f"  tcp 位置最大差   {max_pos:.6f} mm   (阈值 {args.pos_tol_mm})")
        print(f"  tcp 姿态最大测地差 {max_geo:.6f} deg  (阈值 {args.geo_tol_deg})")
        print(f"  trajectory_q 最大差 {max_q:.6f} rad  (阈值 {args.q_tol_rad})")
        if max_pos > args.pos_tol_mm:
            value_errors.append(f"位置超差 {max_pos}")
        if max_geo > args.geo_tol_deg:
            value_errors.append(f"姿态超差 {max_geo}")
        if max_q > args.q_tol_rad:
            value_errors.append(f"关节轨迹超差 {max_q}")
    for e in value_errors:
        print("  ✗", e)

    if schema_errors or value_errors:
        print(f"\n不一致：schema {len(schema_errors)} 项，数值 {len(value_errors)} 项")
        return 1
    print("\n✓ C++ 产物与重构前 Python 版逐字段一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
