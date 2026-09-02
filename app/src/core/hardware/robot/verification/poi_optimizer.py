"""
POI Pose Constraint Optimizer (POI).
Powered by SprayWaypointOptimizer (Viterbi DP global MoveL continuous optimization)
with bounded tolerance envelope [±ΔRx, ±ΔRy, ±ΔRz] relative to an anchor reference pose.
Outputs clean, beautifully aligned comparison diagnostics matching path_opt_cli.
"""

import copy
import logging
import math
import time
from typing import Optional
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

from .robot_config import RobotConfig, get_configured_optimization_config
from .path_interpolator import PathInterpolator, pose_dict_to_matrix, matrix_to_pose_dict
from .kinematic_chain_verifier import KinematicChainVerifier
from .path_opt import SprayWaypointOptimizer

logger = logging.getLogger(__name__)


def _char_width(c: str) -> int:
    """Calculates visual display width for characters (handles East Asian fullwidth / CJK)."""
    code = ord(c)
    if (0x4E00 <= code <= 0x9FFF or 0x3400 <= code <= 0x4DBF or
        0x3000 <= code <= 0x303F or 0xFF01 <= code <= 0xFF60 or
        0x2500 <= code <= 0x257F or 0x2580 <= code <= 0x259F):
        return 2
    return 1


def _disp_len(s: str) -> int:
    """Returns visual string length in monospaced terminal columns."""
    return sum(_char_width(c) for c in str(s))


def _pad(s: str, width: int, align: str = "left") -> str:
    """Pads string to visual column width considering wide CJK characters."""
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
    """Renders a pixel-perfect aligned ASCII table."""
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


def _format_comparison_report(
    solver,
    verifier,
    optimizer: SprayWaypointOptimizer,
    raw_path_item: dict,
    opt_path_item: dict,
    anchor_rpy: list[float],
    anchor_tol: list[float],
    home_rad: list[float],
    elapsed_ms: float,
    was_modified: bool,
    rep: Optional[dict] = None,
) -> str:
    """Formats full before/after comparison tables and diagnostics identical to path_opt_cli."""
    raw_points = raw_path_item.get("points", [])
    opt_points = opt_path_item.get("points", [])
    opt_joints_list = opt_path_item.get("spray_opt_joints_deg", [])

    lines = []
    lines.append("\n" + "=" * 136)
    lines.append("⚡ 2. 运动学引擎状态:")
    backend_str = solver.backend.upper() if hasattr(solver, "backend") else "PYTHON"
    cpp_status = "已加载 (高吞吐模式 ~2.3 MHz)" if getattr(solver, "backend", "") == "cpp" else "未加载 (Python 模式)"
    lines.append(f"   当前解算后端: {backend_str}")
    lines.append(f"   C++ 加速动态库 (libur_kin): {cpp_status}")

    home_deg = [round(math.degrees(a), 2) for a in home_rad]
    anchor_xyz, _ = solver.forward_controller(home_rad)

    lines.append(f"\n📍 3. 锚点配置 (Anchor Pose via Forward Kinematics):")
    lines.append(f"   Home 关节角度: {home_deg} deg")
    lines.append(f"   解算锚点位置: XYZ = [{anchor_xyz[0]:.2f}, {anchor_xyz[1]:.2f}, {anchor_xyz[2]:.2f}] mm")
    if anchor_rpy is not None:
        lines.append(f"   解算锚点姿态: RPY = [{anchor_rpy[0]:.2f}, {anchor_rpy[1]:.2f}, {anchor_rpy[2]:.2f}] deg (Euler 'xyz')")
    else:
        lines.append(f"   解算锚点姿态: 逐点自适应名义法向姿态 (Adaptive Per-waypoint Surface Normal Envelope)")

    lines.append(f"\n⚙️  4. 优化参数设置:")
    lines.append(f"   搜索网格: Tol_X={optimizer.tol_x_deg}, Tol_Y={optimizer.tol_y_deg}, Tol_Z={optimizer.tol_z_deg}")
    lines.append(f"   锚点硬包络: (Tol_Rx=±{anchor_tol[0]:.1f}°, Tol_Ry=±{anchor_tol[1]:.1f}°, Tol_Rz=±{anchor_tol[2]:.1f}°)")
    lines.append(
        f"   Beam Width: {optimizer.beam_width}, 8支单桶容量: {optimizer.max_candidates_per_branch}, "
        f"MoveL 抽检: [{optimizer.num_movel_checks}, {optimizer.max_movel_checks}] 点 (间距 {optimizer.movel_check_spacing_mm} mm)"
    )

    lines.append(f"\n🔄 5. 正在执行 Viterbi DP 全局连续性优化 (Waypoints 数量: {len(raw_points)})...")
    lines.append(f"   优化完成! 耗时: {elapsed_ms:.2f} ms | 姿态已修改: {was_modified}")

    # anchor_rpy 为 None = 逐点锚点(raw 模式): 每个航点以自身名义法向姿态作包络中心
    per_point_anchor = anchor_rpy is None
    r_anchor_fix = None if per_point_anchor else R_scipy.from_euler("xyz", anchor_rpy, degrees=True)
    z_anchor_fix = None if per_point_anchor else r_anchor_fix.as_matrix()[:, 2]

    curr_q_raw = home_rad
    raw_joints_list = []
    for wp in raw_points:
        raw_T = pose_dict_to_matrix(wp.get("tcp_pose_base", wp))
        q_sol = solver.get_best_ik_controller(raw_T, curr_q_raw)
        if q_sol is not None:
            curr_q_raw = q_sol
            raw_joints_list.append(np.degrees(q_sol))
        else:
            raw_joints_list.append(np.zeros(6))

    # Table 6.1: Raw
    h1 = ["序号", "位置 (X,Y,Z) mm", "原始姿态 RPY°", "相对锚点偏角", "指向偏角", "原始关节 J1~J6 (deg)"]
    w1 = [6, 21, 21, 17, 10, 42]
    r1 = []
    for i in range(len(raw_points)):
        raw_p = raw_points[i].get("tcp_pose_base", raw_points[i])
        r_raw = R_scipy.from_euler("xyz", [raw_p["rx"], raw_p["ry"], raw_p["rz"]], degrees=True)
        z_raw = r_raw.as_matrix()[:, 2]
        r_anchor = r_raw if per_point_anchor else r_anchor_fix
        z_anchor = z_raw if per_point_anchor else z_anchor_fix

        rel_raw = (r_anchor.inv() * r_raw).as_euler("xyz", degrees=True)
        rel_raw = (rel_raw + 180.0) % 360.0 - 180.0
        rel_raw_str = f"[{rel_raw[0]:+4.0f}, {rel_raw[1]:+4.0f}, {rel_raw[2]:+5.0f}]"

        pt_raw = float(np.degrees(np.arccos(np.clip(np.dot(z_raw, z_anchor), -1.0, 1.0))))
        q_raw = raw_joints_list[i]
        q_raw_str = f"[{q_raw[0]:5.1f}, {q_raw[1]:5.1f}, {q_raw[2]:5.1f}, {q_raw[3]:5.1f}, {q_raw[4]:5.1f}, {q_raw[5]:5.1f}]"
        pos_str = f"{raw_p['x']:5.1f}, {raw_p['y']:5.1f}, {raw_p['z']:5.1f}"
        raw_rpy_str = f"{raw_p['rx']:6.2f}, {raw_p['ry']:6.2f}, {raw_p['rz']:6.2f}"

        r1.append([f"#{i+1}", pos_str, raw_rpy_str, rel_raw_str, f"{pt_raw:6.2f}°", q_raw_str])

    lines.append("\n" + _format_table(h1, w1, r1, "6.1 优化前 (原始 Raw) 航点姿态、相对锚点偏角与关节角度"))

    # Table 6.2: Opt
    h2 = ["序号", "位置 (X,Y,Z) mm", "优化后姿态 RPY°", "相对锚点偏角", "指向偏角", "优化关节 J1~J6 (deg)"]
    w2 = [6, 21, 21, 17, 10, 42]
    r2 = []
    for i in range(len(raw_points)):
        raw_p = raw_points[i].get("tcp_pose_base", raw_points[i])
        opt_p = opt_points[i].get("tcp_pose_base", opt_points[i])
        r_opt = R_scipy.from_euler("xyz", [opt_p["rx"], opt_p["ry"], opt_p["rz"]], degrees=True)
        z_opt = r_opt.as_matrix()[:, 2]
        if per_point_anchor:
            r_raw = R_scipy.from_euler("xyz", [raw_p["rx"], raw_p["ry"], raw_p["rz"]], degrees=True)
            r_anchor, z_anchor = r_raw, r_raw.as_matrix()[:, 2]
        else:
            r_anchor, z_anchor = r_anchor_fix, z_anchor_fix

        rel_opt = (r_anchor.inv() * r_opt).as_euler("xyz", degrees=True)
        rel_opt = (rel_opt + 180.0) % 360.0 - 180.0
        rel_opt_str = f"[{rel_opt[0]:+4.0f}, {rel_opt[1]:+4.0f}, {rel_opt[2]:+5.0f}]"

        pt_opt = float(np.degrees(np.arccos(np.clip(np.dot(z_opt, z_anchor), -1.0, 1.0))))
        q_opt = opt_joints_list[i] if i < len(opt_joints_list) else [0.0] * 6
        q_opt_str = f"[{q_opt[0]:5.1f}, {q_opt[1]:5.1f}, {q_opt[2]:5.1f}, {q_opt[3]:5.1f}, {q_opt[4]:5.1f}, {q_opt[5]:5.1f}]"
        pos_str = f"{raw_p['x']:5.1f}, {raw_p['y']:5.1f}, {raw_p['z']:5.1f}"
        opt_rpy_str = f"{opt_p['rx']:6.2f}, {opt_p['ry']:6.2f}, {opt_p['rz']:6.2f}"

        r2.append([f"#{i+1}", pos_str, opt_rpy_str, rel_opt_str, f"{pt_opt:6.2f}°", q_opt_str])

    lines.append("\n" + _format_table(h2, w2, r2, "6.2 优化后 (优化 Opt) 航点姿态、相对锚点偏角与关节角度"))

    # Table 6.3: Difference
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
        r_anchor = r_raw if per_point_anchor else r_anchor_fix

        pointing_diff = float(np.degrees(np.arccos(np.clip(np.dot(z_raw, z_opt), -1.0, 1.0))))

        rel_raw = (r_anchor.inv() * r_raw).as_euler("xyz", degrees=True)
        rel_raw = (rel_raw + 180.0) % 360.0 - 180.0
        rel_opt = (r_anchor.inv() * r_opt).as_euler("xyz", degrees=True)
        rel_opt = (rel_opt + 180.0) % 360.0 - 180.0

        rel_change = f"[{rel_raw[0]:+4.0f},{rel_raw[1]:+4.0f},{rel_raw[2]:+4.0f}] -> [{rel_opt[0]:+4.0f},{rel_opt[1]:+4.0f},{rel_opt[2]:+4.0f}]"

        q_raw = np.array(raw_joints_list[i])
        q_opt = np.array(opt_joints_list[i])
        dq_max = float(np.max(np.abs((q_opt - q_raw + 180.0) % 360.0 - 180.0)))

        raw_rpy_str = f"{raw_p['rx']:5.1f}, {raw_p['ry']:5.1f}, {raw_p['rz']:5.1f}"
        opt_rpy_str = f"{opt_p['rx']:5.1f}, {opt_p['ry']:5.1f}, {opt_p['rz']:5.1f}"

        r3.append([f"#{i+1}", raw_rpy_str, opt_rpy_str, f"{pointing_diff:6.2f}°", rel_change, f"{dq_max:5.2f}°"])

    lines.append("\n" + _format_table(h3, w3, r3, "6.3 优化前后综合指标对比总表 (Raw vs Opt)"))

    lines.append("\n💡 注解说明:")
    anchor_desc = (
        "逐点名义法向锚点 (Adaptive Per-waypoint Normal)" if per_point_anchor
        else f"锚点 [{anchor_rpy[0]:.0f}, {anchor_rpy[1]:.0f}, {anchor_rpy[2]:.0f}]"
    )
    lines.append(f"   1. [相对锚点偏角]: 表示当前姿态相对{anchor_desc}的旋转量，严格受控在容差包络 (Rx:±{anchor_tol[0]:.1f}°, Ry:±{anchor_tol[1]:.1f}°, Rz:±{anchor_tol[2]:.1f}°) 之内。")
    lines.append(f"   2. [相对锚点指向角]: 表示喷枪中心法向与{anchor_desc}喷枪法向在 3D 空间中的夹角。")
    lines.append("   3. [枪尖指向偏量(3D)]: 表示优化前后喷枪法向的实际偏转角度（排除了 Euler 欧拉角在 Ry≈-90° 时的万向节死锁双重表示现象）。")

    if rep:
        status = rep.get("status", "UNKNOWN")
        issues = rep.get("issues", [])
        total_steps = rep.get("total_interpolated", len(rep.get("trajectory_q", [])))
        max_speeds = rep.get("peak_joint_speeds_deg_s", [0.0] * 6)

        lines.append("\n🔍 7. 全路径运动学链校验结果 (Kinematic Chain Verification):")
        lines.append(f"   校验状态: {status} (总插值步数: {total_steps}, 发现问题数: {len(issues)})")
        lines.append(f"   各轴峰值速度: {[round(s, 1) for s in max_speeds]} deg/s")

        if issues:
            lines.append("   ⚠️ 校验问题详情:")
            for issue in issues[:10]:
                lines.append(f"      - {issue}")
        else:
            lines.append("   🎉 校验完美通过 (0 奇异, 0 超速, 0 不可达, 关节连续平滑)!")
    lines.append("=" * 136 + "\n")
    return "\n".join(lines)


class PoiConstraintOptimizer:
    """
    Optimizes manual path waypoint orientations using anchor reference pose and tolerance envelope constraints
    via global Viterbi DP MoveL continuous optimization (SprayWaypointOptimizer).
    """
    def __init__(
        self,
        robot_config: RobotConfig,
        interpolator: PathInterpolator,
        verifier: KinematicChainVerifier
    ):
        self.config = robot_config
        self.interpolator = interpolator
        self.verifier = verifier
        self.opt_cfg = get_configured_optimization_config()
        self.spray_optimizer = SprayWaypointOptimizer(
            solver=self.verifier.solver,
            verifier=self.verifier,
            dense_verify=True,
            tol_x_deg=self.opt_cfg.get("grid_tol_x_deg", (-5.0, 5.0, 2.0)),
            tol_y_deg=self.opt_cfg.get("grid_tol_y_deg", (-5.0, 5.0, 2.0)),
            tol_z_deg=self.opt_cfg.get("grid_tol_z_deg", (-180.0, 180.0, 10.0)),
            beam_width=32,
            max_candidates_per_branch=16,
            num_movel_checks=10,
            max_movel_checks=100,
            movel_check_spacing_mm=5.0,
        )

    def optimize_poi_single_path(
        self,
        path_item: dict,
        ref_rpy_deg: list[float] = None,
        tolerance_rpy_deg: list[float] = None,
        tol_rpy_deg: list[float] = None,
        init_q: list[float] = None
    ) -> tuple[dict, bool]:
        """
        Optimizes a single path within a bounded 3D tolerance envelope around an anchor reference pose.
        Returns: (poi_optimized_path, was_modified)
        """
        effective_tol = tolerance_rpy_deg or tol_rpy_deg or [10.0, 10.0, 180.0]
        if len(effective_tol) != 3:
            effective_tol = [10.0, 10.0, 180.0]
        effective_tol = [float(v) for v in effective_tol]

        home_rad = init_q if init_q is not None else [0.0, 0.0, -math.pi / 2.0, -math.pi / 2.0, -math.pi / 2.0, 0.0]
        
        if ref_rpy_deg is not None and len(ref_rpy_deg) == 3:
            effective_anchor_rpy = [float(v) for v in ref_rpy_deg]
        else:
            effective_anchor_rpy = None

        t_start = time.time()
        opt_path, was_modified = self.spray_optimizer.optimize_path_item(
            path_item,
            init_q=home_rad,
            ref_rpy_deg=effective_anchor_rpy,
            tolerance_rpy_deg=effective_tol,
        )
        elapsed_ms = (time.time() - t_start) * 1000.0

        rep = opt_path.get("verification_report")
        if rep is None and self.verifier is not None:
            rep = self.verifier.verify_single_path(opt_path, init_q=home_rad)

        # 打印并记录对齐的结构化表格日志
        report_text = _format_comparison_report(
            solver=self.verifier.solver,
            verifier=self.verifier,
            optimizer=self.spray_optimizer,
            raw_path_item=path_item,
            opt_path_item=opt_path,
            anchor_rpy=effective_anchor_rpy,
            anchor_tol=effective_tol,
            home_rad=home_rad,
            elapsed_ms=elapsed_ms,
            was_modified=was_modified,
            rep=rep,
        )
        print(report_text)
        logger.info(report_text)

        return opt_path, was_modified

    def optimize_poi_all_paths(
        self,
        paths_data: dict,
        ref_rpy_deg: list[float] = None,
        tolerance_rpy_deg: list[float] = None,
        tol_rpy_deg: list[float] = None,
        init_q: list[float] = None,
        anchor_source: str = "home",
    ) -> tuple[dict, dict]:
        """
        Optimizes all manual paths in paths_data using POI constraint envelope search (Viterbi DP).
        Returns: (poi_paths_data, poi_verification_report)

        :param ref_rpy_deg: 包络中心(锚点)参考姿态 [rx, ry, rz]（度）；None 时按 anchor_source 推导
        :param anchor_source: 'config'/'home' → 整条路径共用一个锚点（优先用 ref_rpy_deg，缺省时用 Home 正解）；
                             'raw' → 逐点以航点自身名义（法向）姿态为锚点，即每个点在自己法向 ±容差内选姿态
        """
        effective_tol = tolerance_rpy_deg or tol_rpy_deg or [10.0, 10.0, 180.0]
        if len(effective_tol) != 3:
            effective_tol = [10.0, 10.0, 180.0]
        effective_tol = [float(v) for v in effective_tol]

        src = (anchor_source or "home").strip().lower()
        if src not in {"config", "home", "raw"}:
            src = "home"

        home_rad = init_q if init_q is not None else [0.0, 0.0, -math.pi / 2.0, -math.pi / 2.0, -math.pi / 2.0, 0.0]
        if src == "raw":
            # 逐点名义法向锚点：ref_rpy 置 None 交给 path_opt.optimize 逐点展开
            effective_anchor_rpy = None
        elif ref_rpy_deg is not None and len(ref_rpy_deg) == 3:
            effective_anchor_rpy = [float(v) for v in ref_rpy_deg]
        else:
            # 未给出参考姿态时退回 Home 正解，并把来源如实记成 home
            _, effective_anchor_rpy = self.verifier.solver.forward_controller(home_rad)
            effective_anchor_rpy = [round(float(v), 2) for v in effective_anchor_rpy]
            src = "home"

        paths = paths_data.get("paths", [])
        opt_paths = []
        last_q = home_rad

        for idx, path in enumerate(paths):
            opt_path, was_mod = self.optimize_poi_single_path(
                path,
                ref_rpy_deg=effective_anchor_rpy,
                tolerance_rpy_deg=effective_tol,
                init_q=last_q,
            )
            opt_paths.append(opt_path)
            joints = opt_path.get("spray_opt_joints_deg")
            if joints:
                last_q = [math.radians(a) for a in joints[-1]]

        opt_data = copy.deepcopy(paths_data)
        opt_data["paths"] = opt_paths
        opt_data["type"] = "poi"
        opt_data["poi_config"] = {
            "mode": "per_waypoint_nominal_envelope" if effective_anchor_rpy is None else "absolute_anchor_tolerance",
            "anchor_source": src,
            "ref_rpy_deg": effective_anchor_rpy,
            "tolerance_rpy_deg": effective_tol,
            "euler_order": "xyz",
            "units": "deg",
        }

        reports = []
        overall_status = "PASS"
        total_issues = 0
        total_steps = 0
        total_waypoints = 0
        singularity_cnt = 0
        overspeed_cnt = 0
        unreachable_cnt = 0

        for i, opt_p in enumerate(opt_paths):
            p_rep = opt_p.get("verification_report")
            if not p_rep and self.verifier is not None:
                p_rep = self.verifier.verify_single_path(opt_p)
            if p_rep:
                reports.append(p_rep)
                if p_rep.get("status") == "FAILED":
                    overall_status = "FAILED"
                elif p_rep.get("status") == "WARNING" and overall_status != "FAILED":
                    overall_status = "WARNING"
                total_issues += len(p_rep.get("issues", []))
                total_steps += p_rep.get("total_interpolated", 0)
                total_waypoints += len(opt_p.get("points", []))
                for iss in p_rep.get("issues", []):
                    t = iss.get("type", "")
                    if "SINGULARITY" in t:
                        singularity_cnt += 1
                    elif "OVERSPEED" in t:
                        overspeed_cnt += 1
                    elif "UNREACHABLE" in t or "DISCONTINUITY" in t or t == "JOINT_LIMIT":
                        unreachable_cnt += 1

                opt_p["trajectory_q"] = p_rep.get("trajectory_q", [])
                opt_p["trajectory_tcp"] = p_rep.get("trajectory_tcp", [])
                opt_p["total_interpolated"] = p_rep.get("total_interpolated", 0)

        report = {
            "summary": {
                "status": overall_status,
                "total_paths": len(opt_paths),
                "total_waypoints": total_waypoints,
                "total_steps": total_steps,
                "total_issues": total_issues,
                "singularity_count": singularity_cnt,
                "overspeed_count": overspeed_cnt,
                "unreachable_count": unreachable_cnt,
            },
            "nominal_speed_mm_s": self.interpolator.linear_velocity_mm_s if self.interpolator else 120.0,
            "slerp_step_mm": self.interpolator.step_size_mm if self.interpolator else 1.5,
            "max_joint_velocities_deg_s": self.config.max_joint_vel_deg_s.tolist() if self.config else [180.0] * 6,
            "urdf_tcp": self.config.urdf_tcp if self.config else {},
            "path_reports": reports,
            "poi_config": opt_data["poi_config"],
            "state_type": "poi",
            "optimized_paths_available": True,
        }

        return opt_data, report
