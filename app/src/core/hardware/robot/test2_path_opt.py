#!/usr/bin/env python3
"""
测试 path_opt.py (SprayWaypointOptimizer) 稀疏喷涂位姿优化器。

功能：
1. 内置 data/template_group/2026-08-14_154353/scan.raw.path.yaml 中的 11 个 Waypoints 数据，无需动态读取文件。
2. 通过 Dobot Home 关节角 [0, 0, -90, -90, -90, 0] 调用正运动学 (FK) 自动解算锚点参考位姿。
3. 调用 SprayWaypointOptimizer 执行基于 Viterbi DP 的全局连续性位姿优化。
4. 打印前后对比表格（位置、名义姿态、优化姿态、姿态偏量、优化后关节角度）。
5. 调用 KinematicChainVerifier 进行全轨迹密采样校验，输出校验结果。
"""

import os
import sys
import math
import time
import copy
import numpy as np

# 自动定位项目根目录并导入模块
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "../../../../../"))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from app.src.core.hardware.robot.cr5_kinematics import CR5Kinematics
from app.src.core.hardware.robot.cr5_path_verifier import CR5PathVerifier
from app.src.core.hardware.robot.verification.path_opt import SprayWaypointOptimizer
from app.src.core.hardware.robot.verification.path_interpolator import pose_dict_to_matrix

# 从 data/template_group/2026-08-14_154353/scan.raw.path.yaml 抽取的原始 Waypoints (Path 1)
RAW_WAYPOINTS = [
    {
        "index": 1,
        "tcp_pose_base": {"x": 680.75, "y": 198.33, "z": 329.02, "rx": 0.0, "ry": -88.34, "rz": -176.06},
        "surface_normal_base": [-0.9972, -0.0688, -0.029]
    },
    {
        "index": 2,
        "tcp_pose_base": {"x": 689.91, "y": 183.30, "z": 273.06, "rx": 0.5, "ry": -83.35, "rz": -174.55},
        "surface_normal_base": [-0.9879, -0.1030, -0.1158]
    },
    {
        "index": 3,
        "tcp_pose_base": {"x": 702.81, "y": 173.10, "z": 226.17, "rx": 1.64, "ry": -79.11, "rz": -173.50},
        "surface_normal_base": [-0.9720, -0.1395, -0.1889]
    },
    {
        "index": 4,
        "tcp_pose_base": {"x": 707.73, "y": 195.39, "z": 188.57, "rx": -22.18, "ry": -82.46, "rz": -166.43},
        "surface_normal_base": [-0.9809, 0.1517, -0.1215]
    },
    {
        "index": 5,
        "tcp_pose_base": {"x": 731.49, "y": 236.96, "z": 170.64, "rx": -102.69, "ry": -84.46, "rz": -109.27},
        "surface_normal_base": [-0.8487, 0.5284, 0.0212]
    },
    {
        "index": 6,
        "tcp_pose_base": {"x": 713.40, "y": 172.63, "z": 109.13, "rx": -97.36, "ry": -84.46, "rz": -94.91},
        "surface_normal_base": [-0.9772, 0.2118, 0.0124]
    },
    {
        "index": 7,
        "tcp_pose_base": {"x": 684.79, "y": 46.10, "z": 93.57, "rx": -109.38, "ry": -84.17, "rz": -67.95},
        "surface_normal_base": [-0.9983, -0.0480, 0.0337]
    },
    {
        "index": 8,
        "tcp_pose_base": {"x": 684.01, "y": -33.85, "z": 93.51, "rx": -116.72, "ry": -83.88, "rz": -61.62},
        "surface_normal_base": [-0.9984, -0.0313, 0.0479]
    },
    {
        "index": 9,
        "tcp_pose_base": {"x": 717.02, "y": -127.18, "z": 87.87, "rx": -51.84, "ry": -83.19, "rz": -129.03},
        "surface_normal_base": [-0.9971, 0.0186, -0.0733]
    },
    {
        "index": 10,
        "tcp_pose_base": {"x": 726.73, "y": -135.33, "z": 103.74, "rx": -23.68, "ry": -75.83, "rz": -161.80},
        "surface_normal_base": [-0.9690, 0.1042, -0.2242]
    },
    {
        "index": 11,
        "tcp_pose_base": {"x": 724.81, "y": -157.86, "z": 163.08, "rx": -19.96, "ry": -71.84, "rz": -167.83},
        "surface_normal_base": [-0.9450, 0.1454, -0.2929]
    }
]


def run_path_opt_test():
    print("=" * 115)
    print("🚀 [PathOpt Test] 启动 path_opt.py (SprayWaypointOptimizer) 独立优化与验证测试")
    print("=" * 115)

    # 1. 初始化运动学求解器与验证器
    pv = CR5PathVerifier()
    solver: CR5Kinematics = pv.solver
    verifier = pv.verifier

    # 2. 从 Dobot Home 关节角 [0, 0, -90, -90, -90, 0] 通过正解 (FK) 计算锚点参考位姿
    home_deg = [0.0, 0.0, -90.0, -90.0, -90.0, 0.0]
    home_rad = [math.radians(a) for a in home_deg]
    anchor_xyz, anchor_rpy = solver.forward_controller(home_rad)

    print("\n📍 1. 锚点配置 (Anchor Pose via Forward Kinematics):")
    print(f"   Home 关节角度: {home_deg} deg")
    print(f"   解算锚点位置: XYZ = [{anchor_xyz[0]:.2f}, {anchor_xyz[1]:.2f}, {anchor_xyz[2]:.2f}] mm")
    print(f"   解算锚点姿态: RPY = [{anchor_rpy[0]:.2f}, {anchor_rpy[1]:.2f}, {anchor_rpy[2]:.2f}] deg (Euler 'xyz')")

    # 3. 创建 SprayWaypointOptimizer
    # 工艺容差：绕工具 X/Y 允许 ±5° 微倾，绕工具 Z 允许 ±180° 自旋
    anchor_tol_deg = (10.0, 15.0, 180.0)
    optimizer = SprayWaypointOptimizer(
        solver=solver,
        verifier=verifier,
        dense_verify=True,
        tol_x_deg=(-5.0, 5.0, 5.0),
        tol_y_deg=(-5.0, 5.0, 5.0),
        tol_z_deg=(-180.0, 180.0, 30.0),
        beam_width=32,
        num_movel_checks=4,
        max_movel_checks=8
    )

    print(f"\n⚙️  2. 优化参数设置:")
    print(f"   搜索网格: Tol_X={optimizer.tol_x_deg}, Tol_Y={optimizer.tol_y_deg}, Tol_Z={optimizer.tol_z_deg}")
    print(f"   锚点硬包络: (Tol_Rx=±{anchor_tol_deg[0]}°, Tol_Ry=±{anchor_tol_deg[1]}°, Tol_Rz=±{anchor_tol_deg[2]}°)")
    print(f"   Beam Width: {optimizer.beam_width}, MoveL 抽检点数: [{optimizer.num_movel_checks}, {optimizer.max_movel_checks}]")

    # 4. 执行优化
    print(f"\n🔄 3. 正在执行 Viterbi DP 全局连续性优化 (Waypoints 数量: {len(RAW_WAYPOINTS)})...")
    path_item = {
        "path_id": 1,
        "name": "Path 1",
        "points": copy.deepcopy(RAW_WAYPOINTS)
    }

    t_start = time.time()
    opt_path_item, was_modified = optimizer.optimize_path_item(
        path_item,
        init_q=home_rad,
        ref_rpy_deg=anchor_rpy,
        tolerance_rpy_deg=list(anchor_tol_deg)
    )
    elapsed_ms = (time.time() - t_start) * 1000.0
    print(f"   优化完成! 耗时: {elapsed_ms:.2f} ms | 姿态已修改: {was_modified}")

    # 5. 输出对比表格
    print("\n" + "=" * 115)
    print("📊 4. 优化前后 Waypoints 对比表格")
    print("=" * 115)
    header = (
        f"{'序号':<4} | "
        f"{'位置 (X, Y, Z) mm':<24} | "
        f"{'原始姿态 (Rx, Ry, Rz)°':<25} | "
        f"{'优化后姿态 (Rx, Ry, Rz)°':<25} | "
        f"{'姿态偏量 Δ(Rx,Ry,Rz)°':<22} | "
        f"{'优化后关节角 (J1~J6)°'}"
    )
    print(header)
    print("-" * 115)

    opt_points = opt_path_item.get("points", [])
    opt_joints_list = opt_path_item.get("spray_opt_joints_deg", [])

    for i in range(len(RAW_WAYPOINTS)):
        raw_p = RAW_WAYPOINTS[i]["tcp_pose_base"]
        opt_p = opt_points[i]["tcp_pose_base"]
        q_deg = opt_joints_list[i] if i < len(opt_joints_list) else [0.0] * 6

        pos_str = f"{raw_p['x']:6.1f}, {raw_p['y']:6.1f}, {raw_p['z']:6.1f}"
        raw_rpy_str = f"{raw_p['rx']:6.2f}, {raw_p['ry']:6.2f}, {raw_p['rz']:6.2f}"
        opt_rpy_str = f"{opt_p['rx']:6.2f}, {opt_p['ry']:6.2f}, {opt_p['rz']:6.2f}"

        drx = opt_p['rx'] - raw_p['rx']
        dry = opt_p['ry'] - raw_p['ry']
        drz = (opt_p['rz'] - raw_p['rz'] + 180.0) % 360.0 - 180.0
        delta_str = f"{drx:+5.1f}, {dry:+5.1f}, {drz:+6.1f}"

        q_str = f"[{q_deg[0]:6.1f}, {q_deg[1]:5.1f}, {q_deg[2]:5.1f}, {q_deg[3]:5.1f}, {q_deg[4]:5.1f}, {q_deg[5]:5.1f}]"

        print(f"#{i+1:<3} | {pos_str:<24} | {raw_rpy_str:<25} | {opt_rpy_str:<25} | {delta_str:<22} | {q_str}")

    print("-" * 115)

    # 6. 全路径运动学校验 (Dense MoveL Kinematic Verification)
    print("\n🔍 5. 全路径运动学链校验结果 (Kinematic Chain Verification):")
    rep = verifier.verify_single_path(opt_path_item, init_q=home_rad)
    status = rep.get("status", "UNKNOWN")
    issues = rep.get("issues", [])
    total_steps = rep.get("total_interpolated", 0)
    max_speeds = rep.get("peak_joint_speeds_deg_s", [0.0] * 6)

    print(f"   校验状态: {status} (总插值步数: {total_steps}, 发现问题数: {len(issues)})")
    print(f"   各轴峰值速度: {[round(s, 1) for s in max_speeds]} deg/s")

    if issues:
        print("   ⚠️ 校验问题详情:")
        for idx, iss in enumerate(issues, 1):
            print(f"      [{idx}] 类型: {iss.get('type')}, 级别: {iss.get('severity')}, 详情: {iss.get('detail')}")
    else:
        print("   🎉 校验完美通过 (0 奇异, 0 超速, 0 不可达, 关节连续平滑)!")

    print("=" * 115)


if __name__ == "__main__":
    run_path_opt_test()
