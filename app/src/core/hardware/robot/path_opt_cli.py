#!/usr/bin/env python3
"""
Spray Waypoint Optimizer CLI (基于 Viterbi 动态规划的喷涂位姿全局连续性优化器命令行工具)

功能：
1. 从指定的 YAML 路径文件（例如 scan.raw.path.yaml）中加载 waypoints 轨迹。
2. 通过 Dobot Home 关节角 [0, 0, -90, -90, -90, 0] 调用正运动学 (FK) 自动解算锚点参考位姿。
3. 调用 SprayWaypointOptimizer 执行基于 8 大解析支多样性剪枝与自适应回退的全局连续性位姿优化。
4. 输出三张高可读性表格：
   - 📊 优化前 (原始 Raw) 航点姿态、相对锚点偏角、相对指向夹角与关节角度
   - 📊 优化后 (优化 Opt) 航点姿态、相对锚点偏角、相对指向夹角与关节角度
   - 📊 优化前后关键指标对比总表 (3D 枪尖指向偏角、相对锚点变化、关节最大偏量)
5. 调用 KinematicChainVerifier 进行全轨迹 1.5mm 密插值校验并输出健康度报告。
6. 支持将优化后的位姿与关节角写回/另存为 YAML 文件。

使用示例：
    # 默认读取内置模板 YAML 测试
    python path_opt_cli.py

    # 指定自定义 YAML 文件并另存优化后结果
    python path_opt_cli.py -f data/template_group/2026-08-14_154353/scan.raw.path.yaml -o scan.opt.path.yaml

    # 自定义容差搜索步长、锚点包络与密插值校验参数
    python path_opt_cli.py -f scan.raw.path.yaml --tol-x -5,5,2 --tol-y -5,5,2 --tol-z -180,180,10 --anchor-tol 20,20,180 --verify-step 1.5 --verify-speed 120
"""

import argparse
import copy
import logging
import math
import os
import sys
import time
from typing import Any, Optional

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R_scipy

# 自动定位项目根目录并导入模块
# 与 app/src/main.py 一致：仓库根用于 app.src.*；app/src 用于 core.*
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "../../../../../"))
APP_SRC = os.path.join(PROJECT_ROOT, "app", "src")
SRC_DIR = os.path.join(PROJECT_ROOT, "src")
for _p in (PROJECT_ROOT, APP_SRC, SRC_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from app.src.core.hardware.robot.cr5_kinematics import CR5Kinematics
from app.src.core.hardware.robot.cr5_path_verifier import CR5PathVerifier
from app.src.core.hardware.robot.verification.path_interpolator import pose_dict_to_matrix
from app.src.core.hardware.robot.verification.path_opt import SprayWaypointOptimizer


def parse_tuple_floats(s: str) -> tuple[float, ...]:
    """解析以逗号分隔的浮点数字符串为元组，如 '-5,5,2' -> (-5.0, 5.0, 2.0)"""
    return tuple(float(x.strip()) for x in s.split(","))


def load_waypoints_from_yaml(yaml_path: str, path_id: Optional[int] = None) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    """
    从 YAML 文件中读取 waypoints。

    :return: (raw_yaml_data, points_list, selected_path_dict)
    """
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"YAML 路径文件不存在: {yaml_path}")

    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError(f"YAML 顶层格式必须为 Mapping 字典，当前为: {type(data)}")

    paths = data.get("paths", [])
    if paths and isinstance(paths, list):
        if path_id is not None:
            matched = [p for p in paths if p.get("path_id") == path_id]
            if not matched:
                raise ValueError(f"未在 YAML 的 paths 中找到 path_id={path_id} 的轨迹。")
            selected_path = matched[0]
        else:
            selected_path = paths[0]
        points = selected_path.get("points", [])
    elif "points" in data and isinstance(data["points"], list):
        selected_path = data
        points = data["points"]
    else:
        raise ValueError("YAML 文件中未找到有效的 'paths' 或 'points' 列表。")

    if not points:
        raise ValueError(f"提取出的 points 列表为空！")

    return data, points, selected_path


def run_cli():
    parser = argparse.ArgumentParser(
        description="Spray Waypoint Optimizer CLI (喷涂位姿全局连续性优化命令行工具)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    default_yaml = os.path.join(PROJECT_ROOT, "data/template_group/2026-08-14_154353/scan.raw.path.yaml")

    parser.add_argument("-f", "--yaml", type=str, default=default_yaml, help="输入 YAML 路径文件路径")
    parser.add_argument("-p", "--path-id", type=int, default=None, help="YAML 中待优化的 path_id (默认取第 1 条轨迹)")
    parser.add_argument("-o", "--output", type=str, default=None, help="优化后 YAML 保存路径 (可选)")
    parser.add_argument("--tol-x", type=str, default="-5,5,2", help="绕工具 X 轴倾角搜索 (min,max,step)°")
    parser.add_argument("--tol-y", type=str, default="-5,5,2", help="绕工具 Y 轴倾角搜索 (min,max,step)°")
    parser.add_argument("--tol-z", type=str, default="-180,180,10", help="绕工具 Z 轴自旋搜索 (min,max,step)°")
    parser.add_argument("--anchor-tol", type=str, default="10,10,180", help="相对锚点硬容差包络 (tol_rx,tol_ry,tol_rz)°")
    parser.add_argument("--home-joints", type=str, default="0,0,-90,-90,-90,0", help="Home 关节角 (用于 FK 确定锚点)°")
    parser.add_argument("--beam-width", type=int, default=32, help="DP Beam 宽度 (保留最优节点数)")
    parser.add_argument("--max-candidates-per-branch", type=int, default=16, help="每个解析支保留的最优姿态候选数")
    parser.add_argument("--movel-checks", type=str, default="10,100", help="MoveL 抽检点数范围 (min,max)")
    parser.add_argument("--movel-spacing", type=float, default=5.0, help="DP 边评估 MoveL 抽检估算间距 (mm)")
    parser.add_argument(
        "--verify-step",
        type=float,
        default=1.5,
        help="KinematicChainVerifier 密插值步长 (mm)，用于 dense_verify 与校验报告",
    )
    parser.add_argument(
        "--verify-speed",
        type=float,
        default=120.0,
        help="KinematicChainVerifier 假定笛卡尔线速度 (mm/s)，用于关节超速估算",
    )

    args = parser.parse_args()

    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    print("=" * 135)
    print("🚀 [PathOpt CLI] 启动 SprayWaypointOptimizer 喷涂路径位姿全局连续性优化器")
    print("=" * 135)

    # 1. 解析参数
    tol_x = parse_tuple_floats(args.tol_x)
    tol_y = parse_tuple_floats(args.tol_y)
    tol_z = parse_tuple_floats(args.tol_z)
    anchor_tol = parse_tuple_floats(args.anchor_tol)
    home_deg = parse_tuple_floats(args.home_joints)
    movel_checks = parse_tuple_floats(args.movel_checks)

    # 2. 读取 YAML 数据
    raw_yaml_data, raw_points, path_dict = load_waypoints_from_yaml(args.yaml, args.path_id)
    print(f"\n📂 1. 输入数据来源: {args.yaml}")
    print(f"   轨迹名称: {path_dict.get('name', 'N/A')} (ID: {path_dict.get('path_id', 'N/A')}), 包含航点数: {len(raw_points)}")

    # 3. 初始化运动学求解器与验证器
    verify_step_mm = max(1e-3, float(args.verify_step))
    verify_speed_mm_s = max(1e-3, float(args.verify_speed))
    pv = CR5PathVerifier(
        kinematics_backend="auto",
        step_size_mm=verify_step_mm,
        linear_velocity_mm_s=verify_speed_mm_s,
    )
    solver: CR5Kinematics = pv.solver
    verifier = pv.verifier

    print(f"\n⚡ 2. 运动学引擎状态:")
    print(f"   当前解算后端: {solver.backend.upper()}")
    print(f"   C++ 加速动态库 (libur_kin): {'已加载 (高吞吐模式 ~2.3 MHz)' if solver.backend == 'cpp' else '未加载 (Python 模式)'}")
    print(f"   密插值校验: 步长 {verify_step_mm} mm, 假定线速度 {verify_speed_mm_s} mm/s")

    # 4. 从 Home 关节角正解计算锚点参考位姿
    t_fk_start = time.time()
    home_rad = [math.radians(a) for a in home_deg]
    anchor_xyz, anchor_rpy = solver.forward_controller(home_rad)
    t_fk_ms = (time.time() - t_fk_start) * 1000.0

    print(f"\n📍 3. 锚点配置 (Anchor Pose via Forward Kinematics):")
    print(f"   Home 关节角度: {list(home_deg)} deg")
    print(f"   解算锚点位置: XYZ = [{anchor_xyz[0]:.2f}, {anchor_xyz[1]:.2f}, {anchor_xyz[2]:.2f}] mm")
    print(f"   解算锚点姿态: RPY = [{anchor_rpy[0]:.2f}, {anchor_rpy[1]:.2f}, {anchor_rpy[2]:.2f}] deg (Euler 'xyz')")
    print(f"   FK 耗时: {t_fk_ms:.3f} ms")

    # 5. 构建优化器
    optimizer = SprayWaypointOptimizer(
        solver=solver,
        verifier=verifier,
        dense_verify=True,
        tol_x_deg=tol_x,
        tol_y_deg=tol_y,
        tol_z_deg=tol_z,
        beam_width=args.beam_width,
        max_candidates_per_branch=args.max_candidates_per_branch,
        num_movel_checks=int(movel_checks[0]),
        max_movel_checks=int(movel_checks[1]),
        movel_check_spacing_mm=args.movel_spacing,
    )

    print(f"\n⚙️  4. 优化参数设置:")
    print(f"   搜索网格: Tol_X={optimizer.tol_x_deg}, Tol_Y={optimizer.tol_y_deg}, Tol_Z={optimizer.tol_z_deg}")
    print(f"   锚点硬包络: (Tol_Rx=±{anchor_tol[0]}°, Tol_Ry=±{anchor_tol[1]}°, Tol_Rz=±{anchor_tol[2]}°)")
    print(f"   Beam Width: {optimizer.beam_width}, 8支单桶容量: {optimizer.max_candidates_per_branch}")
    print(f"   DP MoveL 抽检: [{optimizer.num_movel_checks}, {optimizer.max_movel_checks}] 点 (间距 {optimizer.movel_check_spacing_mm} mm)")
    print(f"   Verifier 密插值: 步长 {verify_step_mm} mm, 线速度 {verify_speed_mm_s} mm/s")

    # 6. 执行优化
    print(f"\n🔄 5. 正在执行 Viterbi DP 全局连续性优化 (Waypoints 数量: {len(raw_points)})...")
    path_item = {
        "path_id": path_dict.get("path_id", 1),
        "name": path_dict.get("name", "Path"),
        "points": copy.deepcopy(raw_points),
    }

    t_start = time.time()
    opt_path_item, was_modified = optimizer.optimize_path_item(
        path_item,
        init_q=home_rad,
        ref_rpy_deg=anchor_rpy,
        tolerance_rpy_deg=list(anchor_tol),
    )
    elapsed_ms = (time.time() - t_start) * 1000.0
    print(f"   优化完成! 耗时: {elapsed_ms:.2f} ms | 姿态已修改: {was_modified}")

    # 7. 计算原始路径的连续运动学逆解
    r_anchor = R_scipy.from_euler("xyz", anchor_rpy, degrees=True)
    z_anchor = r_anchor.as_matrix()[:, 2]

    curr_q_raw = home_rad
    raw_joints_list = []
    for wp in raw_points:
        raw_T = pose_dict_to_matrix(wp.get("tcp_pose_base", wp))
        q_sol = solver.get_best_ik_controller(raw_T, curr_q_raw)
        if q_sol is not None:
            curr_q_raw = q_sol
            raw_joints_list.append(np.degrees(q_sol))
        else:
            raw_joints_list.append(None)

    opt_points = opt_path_item.get("points", [])
    opt_joints_list = opt_path_item.get("spray_opt_joints_deg", [])

    def _char_width(c: str) -> int:
        code = ord(c)
        if (0x4E00 <= code <= 0x9FFF or 0x3400 <= code <= 0x4DBF or
            0x3000 <= code <= 0x303F or 0xFF01 <= code <= 0xFF60 or
            0x2500 <= code <= 0x257F or 0x2580 <= code <= 0x259F):
            return 2
        return 1

    def _disp_len(s: str) -> int:
        return sum(_char_width(c) for c in str(s))

    def _pad(s: str, width: int, align: str = "left") -> str:
        s = str(s)
        cur_len = _disp_len(s)
        pad_len = max(0, width - cur_len)
        if align == "right":
            return " " * pad_len + s
        elif align == "center":
            l = pad_len // 2
            r = pad_len - l
            return " " * l + s + " " * r
        return s + " " * pad_len

    def _format_table(headers: list[str], widths: list[int], rows: list[list[str]], title: str = "") -> str:
        lines = []
        tot_w = sum(widths) + 3 * (len(widths) - 1) + 4
        lines.append("=" * tot_w)
        if title:
            lines.append(f"📊 {title}")
            lines.append("-" * tot_w)
        hdr_str = "| " + " | ".join(_pad(h, w, "center" if i == 0 else "left") for i, (h, w) in enumerate(zip(headers, widths))) + " |"
        sep_str = "+-" + "-+-".join("-" * w for w in widths) + "-+"
        lines.append(hdr_str)
        lines.append(sep_str)
        for r in rows:
            r_str = "| " + " | ".join(_pad(v, w, "center" if i == 0 else "left") for i, (v, w) in enumerate(zip(r, widths))) + " |"
            lines.append(r_str)
        lines.append("=" * tot_w)
        return "\n".join(lines)

    # 8. 打印三张详细对比表格
    h1 = ["序号", "位置 (X,Y,Z) mm", "原始姿态 RPY°", "相对锚点偏角", "指向偏角", "原始关节 J1~J6 (deg)"]
    w1 = [6, 21, 21, 17, 10, 42]
    r1 = []
    for i in range(len(raw_points)):
        raw_p = raw_points[i].get("tcp_pose_base", raw_points[i])
        r_raw = R_scipy.from_euler("xyz", [raw_p["rx"], raw_p["ry"], raw_p["rz"]], degrees=True)
        z_raw = r_raw.as_matrix()[:, 2]

        rel_raw = (r_anchor.inv() * r_raw).as_euler("xyz", degrees=True)
        rel_raw = (rel_raw + 180.0) % 360.0 - 180.0
        rel_raw_str = f"[{rel_raw[0]:+4.0f}, {rel_raw[1]:+4.0f}, {rel_raw[2]:+5.0f}]"

        pt_raw = float(np.degrees(np.arccos(np.clip(np.dot(z_raw, z_anchor), -1.0, 1.0))))
        q_raw = raw_joints_list[i]
        if q_raw is None:
            q_raw_str = "IK 无解"
        else:
            q_raw_str = f"[{q_raw[0]:5.1f}, {q_raw[1]:5.1f}, {q_raw[2]:5.1f}, {q_raw[3]:5.1f}, {q_raw[4]:5.1f}, {q_raw[5]:5.1f}]"
        pos_str = f"{raw_p['x']:5.1f}, {raw_p['y']:5.1f}, {raw_p['z']:5.1f}"
        raw_rpy_str = f"{raw_p['rx']:6.2f}, {raw_p['ry']:6.2f}, {raw_p['rz']:6.2f}"

        r1.append([f"#{i+1}", pos_str, raw_rpy_str, rel_raw_str, f"{pt_raw:6.2f}°", q_raw_str])

    print("\n" + _format_table(h1, w1, r1, "6.1 优化前 (原始 Raw) 航点姿态、相对锚点偏角与关节角度"))

    h2 = ["序号", "位置 (X,Y,Z) mm", "优化后姿态 RPY°", "相对锚点偏角", "指向偏角", "优化关节 J1~J6 (deg)"]
    w2 = [6, 21, 21, 17, 10, 42]
    r2 = []
    for i in range(len(raw_points)):
        raw_p = raw_points[i].get("tcp_pose_base", raw_points[i])
        opt_p = opt_points[i].get("tcp_pose_base", opt_points[i])
        r_opt = R_scipy.from_euler("xyz", [opt_p["rx"], opt_p["ry"], opt_p["rz"]], degrees=True)
        z_opt = r_opt.as_matrix()[:, 2]

        rel_opt = (r_anchor.inv() * r_opt).as_euler("xyz", degrees=True)
        rel_opt = (rel_opt + 180.0) % 360.0 - 180.0
        rel_opt_str = f"[{rel_opt[0]:+4.0f}, {rel_opt[1]:+4.0f}, {rel_opt[2]:+5.0f}]"

        pt_opt = float(np.degrees(np.arccos(np.clip(np.dot(z_opt, z_anchor), -1.0, 1.0))))
        q_opt = opt_joints_list[i] if i < len(opt_joints_list) else [0.0] * 6
        q_opt_str = f"[{q_opt[0]:5.1f}, {q_opt[1]:5.1f}, {q_opt[2]:5.1f}, {q_opt[3]:5.1f}, {q_opt[4]:5.1f}, {q_opt[5]:5.1f}]"
        pos_str = f"{raw_p['x']:5.1f}, {raw_p['y']:5.1f}, {raw_p['z']:5.1f}"
        opt_rpy_str = f"{opt_p['rx']:6.2f}, {opt_p['ry']:6.2f}, {opt_p['rz']:6.2f}"

        r2.append([f"#{i+1}", pos_str, opt_rpy_str, rel_opt_str, f"{pt_opt:6.2f}°", q_opt_str])

    print("\n" + _format_table(h2, w2, r2, "6.2 优化后 (优化 Opt) 航点姿态、相对锚点偏角与关节角度"))

    h3 = ["序号", "原始姿态 RPY°", "优化后姿态 RPY°", "3D指向偏量", "相对锚点偏角变化 (Raw -> Opt)", "关节偏量 Δq_max"]
    w3 = [6, 21, 21, 12, 33, 16]
    r3 = []
    for i in range(len(raw_points)):
        raw_p = raw_points[i].get("tcp_pose_base", raw_points[i])
        opt_p = opt_points[i].get("tcp_pose_base", opt_points[i])
        r_raw = R_scipy.from_euler("xyz", [raw_p["rx"], raw_p["ry"], raw_p["rz"]], degrees=True)
        r_opt = R_scipy.from_euler("xyz", [opt_p["rx"], opt_p["ry"], opt_p["rz"]], degrees=True)
        z_raw = r_raw.as_matrix()[:, 2]
        z_opt = r_opt.as_matrix()[:, 2]

        pointing_diff = float(np.degrees(np.arccos(np.clip(np.dot(z_raw, z_opt), -1.0, 1.0))))

        rel_raw = (r_anchor.inv() * r_raw).as_euler("xyz", degrees=True)
        rel_raw = (rel_raw + 180.0) % 360.0 - 180.0
        rel_opt = (r_anchor.inv() * r_opt).as_euler("xyz", degrees=True)
        rel_opt = (rel_opt + 180.0) % 360.0 - 180.0

        rel_change = f"[{rel_raw[0]:+4.0f},{rel_raw[1]:+4.0f},{rel_raw[2]:+4.0f}] -> [{rel_opt[0]:+4.0f},{rel_opt[1]:+4.0f},{rel_opt[2]:+4.0f}]"

        q_raw = raw_joints_list[i]
        q_opt = np.array(opt_joints_list[i]) if i < len(opt_joints_list) else None
        if q_raw is None or q_opt is None:
            dq_max_str = "N/A"
        else:
            dq_max = float(np.max(np.abs((np.array(q_opt) - np.array(q_raw) + 180.0) % 360.0 - 180.0)))
            dq_max_str = f"{dq_max:5.2f}°"

        raw_rpy_str = f"{raw_p['rx']:5.1f}, {raw_p['ry']:5.1f}, {raw_p['rz']:5.1f}"
        opt_rpy_str = f"{opt_p['rx']:5.1f}, {opt_p['ry']:5.1f}, {opt_p['rz']:5.1f}"

        r3.append([f"#{i+1}", raw_rpy_str, opt_rpy_str, f"{pointing_diff:6.2f}°", rel_change, dq_max_str])

    print("\n" + _format_table(h3, w3, r3, "6.3 优化前后综合指标对比总表 (Raw vs Opt)"))

    print("\n💡 注解说明:")
    print(
        f"   1. [相对锚点偏角]: 表示当前姿态相对 Home 锚点 "
        f"[{anchor_rpy[0]:.0f}, {anchor_rpy[1]:.0f}, {anchor_rpy[2]:.0f}] 的旋转量，"
        f"严格受控在容差包络 (Rx:±{anchor_tol[0]:.0f}°, Ry:±{anchor_tol[1]:.0f}°, Rz:±{anchor_tol[2]:.0f}°) 之内。"
    )
    print("   2. [相对锚点指向角]: 表示喷枪中心法向与 Home 锚点喷枪法向在 3D 空间中的夹角。")
    print("   3. [枪尖指向偏量(3D)]: 表示优化前后喷枪法向的实际偏转角度（排除了 Euler 欧拉角在 Ry≈-90° 时的万向节死锁双重表示现象）。")

    # 9. 全路径运动学密插值校验：复用 optimize_path_item 已写入的报告，避免再跑一遍 1.5 mm
    print("\n🔍 7. 全路径运动学链校验结果 (Kinematic Chain Verification):")
    rep = opt_path_item.get("verification_report")
    if not rep:
        rep = verifier.verify_single_path(opt_path_item, init_q=home_rad)
    status = rep.get("status", "UNKNOWN")
    issues = rep.get("issues", [])
    total_steps = rep.get("total_interpolated", 0)
    max_speeds = rep.get("peak_joint_speeds_deg_s", [0.0] * 6)

    rep_step = rep.get("step_size_mm", verify_step_mm)
    rep_speed = rep.get("speed_mm_s", verify_speed_mm_s)
    print(
        f"   校验状态: {status} (步长 {rep_step} mm, 假定 {rep_speed} mm/s, "
        f"总插值步数: {total_steps}, 发现问题数: {len(issues)})"
    )
    print(f"   各轴峰值速度: {[round(s, 1) for s in max_speeds]} deg/s")
    if rep.get("recommended_safe_speed_mm_s") is not None:
        print(f"   推荐安全线速度: {rep['recommended_safe_speed_mm_s']} mm/s")

    if issues:
        print("   ⚠️ 校验问题详情:")
        for issue in issues[:10]:
            print(f"      - {issue}")
    else:
        print("   🎉 校验完美通过 (0 奇异, 0 超速, 0 不可达, 关节连续平滑)!")

    # 10. 保存输出 YAML 文件
    if args.output:
        out_data = copy.deepcopy(raw_yaml_data)
        if "paths" in out_data and isinstance(out_data["paths"], list):
            for idx, p in enumerate(out_data["paths"]):
                if p.get("path_id") == opt_path_item.get("path_id") or len(out_data["paths"]) == 1:
                    out_data["paths"][idx] = opt_path_item
                    break
        else:
            out_data = opt_path_item

        out_dir = os.path.dirname(os.path.abspath(args.output))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as f:
            yaml.dump(out_data, f, allow_unicode=True, sort_keys=False)
        print(f"\n💾 8. 优化后轨迹已成功保存至: {args.output}")

    print("=" * 135)


if __name__ == "__main__":
    run_cli()