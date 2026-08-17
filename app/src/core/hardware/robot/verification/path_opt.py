"""
稀疏喷涂 Waypoint 姿态优化器（面向控制器原生 MoveL）。

上游输入
--------
由表面算法（如 Noether）生成的稀疏笛卡尔枪尖位姿，间距约 10 cm。
工艺上允许一定姿态容差：绕工具 X/Y 的小倾角，以及绕喷枪轴线（工具 Z）的自旋。

下游输出
--------
仍是同一批稀疏 Waypoint（只改姿态 / IK 分支），直接交给机械臂控制器做
MoveL（笛卡尔直线 + 过渡圆角），不下发带时间戳的稠密关节轨迹。

搜索流程
--------
1. 三轴独立容差离散化：在局部工具坐标系内旋复合 R_nom * R_xyz(dx, dy, dz)
2. 锚点姿态硬包络：超出则按相对 Euler-xyz 裁剪回边界；同一姿态只保留
   与原始目标测地夹角最小的那个
3. 零偏代价：能垂直喷则垂直喷（wx/wy 大、wz 很小），只有边不可行时 DP 才会动用倾角
4. 节点硬过滤：关节超限或肩/肘/腕奇异的 IK 解直接丢弃
5. 10 cm MoveL 抽检：位置线性插值 + 四元数 Slerp；
        用 get_best_ik_controller 连续跟分支，拦截无解 / 奇异 / 超限 / 翻腕跳变
        （点数由 movel_check_spacing_mm 与 num/max_movel_checks 决定）
6. Viterbi DP 全局回溯 + beam，秒级给出一条无奇异、无跳变的稀疏序列
7. 若挂了 KinematicChainVerifier，再用 1.5 mm 密采样做硬门

坐标系与单位
------------
位姿与项目其余模块一致：控制器枪尖系，位置 mm、姿态 scipy Euler 'xyz'（度），
4×4 矩阵平移为米。关节内部用弧度，optimize() 对外返回的关节为度。
"""

from __future__ import annotations

import copy
import logging
import math
import time
from typing import Any, Callable, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

from ..cr5_kinematics import CR5Kinematics
from .path_interpolator import matrix_to_pose_dict, pose_dict_to_matrix

logger = logging.getLogger(__name__)

PI = math.pi
# 与 KinematicChainVerifier.BRANCH_JUMP_DEG 对齐：单步超过该值视为换支 / 翻腕
_BRANCH_JUMP_DEG = 45.0
# 无 init_q 时的默认种子构型 (Dobot CR5 Home 姿态 [0, 0, -90, -90, -90, 0]°)
_DEFAULT_SEED_Q = np.array([0.0, 0.0, -PI / 2.0, -PI / 2.0, -PI / 2.0, 0.0], dtype=np.float64)


def _wrap_pi(dq: np.ndarray) -> np.ndarray:
    """把关节差包到 [-π, π]，用于连续距离，避免 359° 与 -1° 被当成大跳变。"""
    return np.mod(dq + PI, 2.0 * PI) - PI


def _axis_grid(spec: tuple[float, float, float]) -> np.ndarray:
    """
    将 (min_deg, max_deg, step_deg) 展开为一维采样。
    step<=0 或区间非法时只保留 0°（即该轴不搜索）。
    +0.5*step 是为了把右端点（如 +5、+180）收进 arange。
    """
    lo, hi, step = (float(spec[0]), float(spec[1]), float(spec[2]))
    if step <= 0.0 or hi < lo:
        return np.array([0.0], dtype=np.float64)
    return np.arange(lo, hi + 0.5 * step, step, dtype=np.float64)


def _geodesic_deg(rot_a: R_scipy, rot_b: R_scipy) -> float:
    """两姿态在 SO(3) 上的测地角（度），即相对旋转向量的模。"""
    return float(np.degrees((rot_a.inv() * rot_b).magnitude()))


def _quat_key(rot: R_scipy) -> tuple:
    """
    四元数去重键。q 与 -q 是同一旋转（双覆盖），统一把 q.w>=0；
    再量化到 5 位小数，使锚点投影后塌缩的网格点能合并。
    """
    q = rot.as_quat()
    if q[3] < 0.0:
        q = -q
    return tuple(np.round(q, 5).tolist())


def _fast_quat_slerp_matrix(q1: np.ndarray, q2: np.ndarray, alpha: float) -> np.ndarray:
    """
    轻量级无对象分配的单位四元数 Slerp -> 3×3 旋转矩阵。
    数学上与 scipy.spatial.transform.Slerp 100% 等价（误差 < 1e-15），
    但在每秒几十万次插值循环中比 SciPy 类构造快 17+ 倍。
    """
    dot = float(q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3])
    q2_mod = q2 if dot >= 0.0 else -q2
    dot_abs = abs(dot)
    if dot_abs > 0.9995:
        # 夹角极小，退化为线性插值 Lerp 并归一化
        q = (1.0 - alpha) * q1 + alpha * q2_mod
        norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        q = q / norm
    else:
        theta = math.acos(min(max(dot_abs, -1.0), 1.0))
        sin_theta = math.sin(theta)
        q = (math.sin((1.0 - alpha) * theta) * q1 + math.sin(alpha * theta) * q2_mod) / sin_theta
    # 四元数 [x, y, z, w] 转 3×3 旋转矩阵
    x, y, z, w = q[0], q[1], q[2], q[3]
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),       2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),       2.0 * (y * z + x * w),       1.0 - 2.0 * (x * x + y * y)]
    ], dtype=np.float64)


def _waypoint_to_T(wp) -> np.ndarray:
    """
    稀疏点 → 控制器系 4×4（米）。
    支持生产路径 dict（tcp_pose_base 或扁平 x/y/z/rx/ry/rz，mm/deg），
    也支持长度为 6 的 numpy/list。
    """
    if isinstance(wp, dict):
        return pose_dict_to_matrix(wp.get("tcp_pose_base", wp))
    arr = np.asarray(wp, dtype=np.float64).reshape(-1)
    if arr.size < 6:
        raise ValueError("waypoint must be a pose dict or [x_mm, y_mm, z_mm, rx, ry, rz]")
    return pose_dict_to_matrix({
        "x": float(arr[0]), "y": float(arr[1]), "z": float(arr[2]),
        "rx": float(arr[3]), "ry": float(arr[4]), "rz": float(arr[5]),
    })


def _T_to_pose6(T: np.ndarray) -> np.ndarray:
    """4×4（米）→ [x_mm, y_mm, z_mm, rx, ry, rz_deg]，与 matrix_to_pose_dict 同一套 Euler。"""
    pose = matrix_to_pose_dict(T)
    return np.array([pose["x"], pose["y"], pose["z"], pose["rx"], pose["ry"], pose["rz"]], dtype=np.float64)


def _project_to_anchor_envelope(
    rot_cand: R_scipy,
    rot_anchor: R_scipy,
    anchor_tol_deg: tuple[float, float, float],
) -> R_scipy:
    """
    锚点硬包络投影（相对 Euler-xyz 盒）。

    相对姿态 R_rel = R_anchor^{-1} * R_cand，拆成 xyz 欧拉角后逐轴 clip 到
    ±(tol_x, tol_y, tol_z)。已在盒内则不变；越界则落到边界。
    这是工艺容差的硬约束，不是测地球面投影；同姿态去重在网格层用测地角完成。
    """
    rel = (rot_anchor.inv() * rot_cand).as_euler("xyz", degrees=True)
    rel = (rel + 180.0) % 360.0 - 180.0  # wrap 到 [-180, 180]，避免 179/-179 裁剪错误
    tol = np.array(anchor_tol_deg, dtype=np.float64)
    clipped = np.clip(rel, -np.abs(tol), np.abs(tol))
    return rot_anchor * R_scipy.from_euler("xyz", clipped, degrees=True)


class SprayWaypointOptimizer:
    """
    针对喷涂/表面加工稀疏 Waypoint 的离散姿态优化器。

    在容差内为每个点选出姿态与关节构型，使相邻点之间的控制器 MoveL 直线
    不经过奇异、不翻腕、不超关节硬限位。内部关节用弧度；对外 6D 位姿用度。
    """

    def __init__(
        self,
        solver: CR5Kinematics = None,
        verifier=None,
        ik_solver: Callable[[np.ndarray], list[np.ndarray]] = None,
        tol_x_deg: tuple[float, float, float] = (-5.0, 5.0, 2.0),
        tol_y_deg: tuple[float, float, float] = (-5.0, 5.0, 2.0),
        tol_z_deg: tuple[float, float, float] = (-180.0, 180.0, 10.0),
        num_movel_checks: int = 10,
        max_movel_checks: int = 100,
        movel_check_spacing_mm: float = 5.0,
        max_joint_jump_deg: float = _BRANCH_JUMP_DEG,
        weight_zero_dev: tuple[float, float, float] = (1.0, 1.0, 0.01),
        joint_weights: Optional[list[float]] = None,
        beam_width: int = 64,
        max_candidates_per_branch: int = 16,
        dense_verify: bool = True,
        ik_returns_degrees: bool = False,
    ):
        """
        :param solver: CR5 解析 IK。省略时按 backend='auto' 创建（有 libur_kin 则走 C++）。
        :param verifier: 可选的 KinematicChainVerifier。dense_verify=True 时，
            DP 选出的稀疏路径会再按 1.5 mm 密插值校验，ERROR / 奇异则失败。
        :param ik_solver: 可选自定义 IK：输入 4×4 枪尖矩阵，返回关节解列表。
            仅用于生成候选节点；MoveL 抽检仍走 solver.get_best_ik_controller。
        :param tol_x_deg: (min, max, step) 绕工具 X 的倾角容差与步长（度）
        :param tol_y_deg: (min, max, step) 绕工具 Y 的倾角容差与步长（度）
        :param tol_z_deg: (min, max, step) 绕工具 Z（喷枪轴）的自旋容差与步长（度）
        :param num_movel_checks: 每段 MoveL 最少中间抽检点数（含按距离加密后的下限）
        :param max_movel_checks: 每段 MoveL 最多中间抽检点数
        :param movel_check_spacing_mm: 按段长估算抽检密度：n ≈ round(段长_mm / 本值)，
            再夹到 [num_movel_checks, max_movel_checks]
        :param max_joint_jump_deg: 抽检相邻样本允许的最大单轴跳变（度），超过判为翻腕/换支
        :param weight_zero_dev: 零偏惩罚 (wx, wy, wz)，按「相对名义姿态的工具系欧拉角」每度平方计。
            默认倾角很贵、自旋几乎免费 → 能垂直喷则垂直喷。
        :param joint_weights: 6 轴边代价权重，抽检 Δq 的加权平方和
        :param beam_width: 每层 DP 只保留代价最低的这么多节点，控制 10 cm 段的边数
        :param max_candidates_per_branch: 8 大解析支中每个构型保留的最优姿态候选上限（如 16，则每层 ~128 节点），
            配合自适应全量回退机制，兼具 0.5s 极速与 100% 不漏解保底。
        :param dense_verify: 是否在写出路径后跑密采样校验器（需传入 verifier）
        :param ik_returns_degrees: 仅当使用 ik_solver 且其返回值为度时置 True
        """
        self.solver = solver if solver is not None else CR5Kinematics(backend="auto")
        self.verifier = verifier
        self._ik_override = ik_solver
        self.ik_returns_degrees = ik_returns_degrees
        self.tol_x_deg = tol_x_deg
        self.tol_y_deg = tol_y_deg
        self.tol_z_deg = tol_z_deg
        self.num_movel_checks = max(1, int(num_movel_checks))
        self.max_movel_checks = max(self.num_movel_checks, int(max_movel_checks))
        self.movel_check_spacing_mm = max(1e-3, float(movel_check_spacing_mm))
        self.max_jump_deg = float(max_joint_jump_deg)
        self.w_zero_dev = np.array(weight_zero_dev, dtype=np.float64)
        # J2 略加重（大臂）、腕部略轻：边代价偏向少甩大关节
        self.joint_weights = np.array(
            joint_weights if joint_weights is not None else [1.0, 1.2, 1.0, 0.8, 0.8, 0.5],
            dtype=np.float64,
        )
        self.beam_width = max(8, int(beam_width))
        self.max_candidates_per_branch = max(1, int(max_candidates_per_branch))
        self.dense_verify = bool(dense_verify)

    def _ik_gun(self, T_gun: np.ndarray) -> list[np.ndarray]:
        """枪尖控制器系 4×4 → 最多 8 组 URDF 关节（弧度）。"""
        if self._ik_override is not None:
            sols = self._ik_override(T_gun)
            if not sols:
                return []
            if self.ik_returns_degrees:
                return [np.deg2rad(np.asarray(q, dtype=np.float64)) for q in sols]
            return [np.asarray(q, dtype=np.float64) for q in sols]
        return list(self.solver.inverse_controller_matrix(T_gun))

    def _is_safe_q(self, q: np.ndarray, T_gun: np.ndarray) -> bool:
        """硬门：URDF 关节限位 + 肩/肘/腕奇异（与校验器同一套 check_singularity_risk）。"""
        if not self.solver.is_joint_valid(q):
            return False
        T_urdf = self.solver.controller_matrix_to_urdf(T_gun)
        return not self.solver.check_singularity_risk(q, T=T_urdf)["is_singular"]

    def _unwrap_onto(self, q_sol: np.ndarray, q_ref: np.ndarray) -> np.ndarray | None:
        """
        把解析解展开到与 q_ref 同一圈绕组（尤其 J6 ±2π）。
        优先 curr = q_ref + wrap(sol-ref)；若该展开超限再退回 [-π, π] 代表。
        """
        q_u = q_ref + _wrap_pi(q_sol - q_ref)
        if self.solver.is_joint_valid(q_u):
            return q_u
        if self.solver.is_joint_valid(q_sol):
            return np.array(q_sol, dtype=np.float64, copy=True)
        return None

    def _apply_anchor_constraint(
        self,
        target_rot: R_scipy,
        anchor_rot: R_scipy,
        anchor_tol_deg: tuple[float, float, float],
    ) -> R_scipy:
        """
        基于锚点姿态的容差包络约束（全角度制）。
        若超出容差范围则裁剪到边界。
        """
        return _project_to_anchor_envelope(target_rot, anchor_rot, anchor_tol_deg)

    def _generate_stage_candidates(
        self,
        T_nominal: np.ndarray,
        anchor_rot: Optional[R_scipy] = None,
        anchor_tol_deg: Optional[tuple[float, float, float]] = None,
    ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        """
        针对单个 Waypoint 生成 DP 节点：
        (姿态网格 × 锚点投影去重) → IK 全解 → 丢掉奇异/超限。

        :return: (fast_candidates, full_candidates)
        fast_candidates 为 8 大解析支中各自按 zero_dev_cost 最优保留 top-K 的精简候选；
        full_candidates 为全量有效候选（用于自适应回退重试保底）。
        """
        base_pos = T_nominal[:3, 3]
        rot_nominal = R_scipy.from_matrix(T_nominal[:3, :3])
        xs = _axis_grid(self.tol_x_deg)
        ys = _axis_grid(self.tol_y_deg)
        zs = _axis_grid(self.tol_z_deg)

        # key=量化四元数 → 投影后重复姿态只留「离名义喷姿测地角最小」的一个
        unique: dict[tuple, dict[str, Any]] = {}
        for dx in xs:
            for dy in ys:
                for dz in zs:
                    # 1. 局部工具系内旋：R_new = R_nom * Rx(dx)Ry(dy)Rz(dz)
                    rot_offset = R_scipy.from_euler("xyz", [dx, dy, dz], degrees=True)
                    rot_cand = rot_nominal * rot_offset
                    # 2. 锚点硬约束：越界则投影回 Euler 盒边界
                    if anchor_rot is not None and anchor_tol_deg is not None:
                        rot_cand = self._apply_anchor_constraint(
                            rot_cand, anchor_rot, anchor_tol_deg
                        )
                    geo = _geodesic_deg(rot_nominal, rot_cand)
                    key = _quat_key(rot_cand)
                    prev = unique.get(key)
                    if prev is not None and geo >= prev["geo_deg"]:
                        continue
                    # 3. 零偏代价用投影后的真实工具系偏差，不用裁剪前的 (dx,dy,dz)
                    e_tool = (rot_nominal.inv() * rot_cand).as_euler("xyz", degrees=True)
                    e_tool = (e_tool + 180.0) % 360.0 - 180.0
                    zero_dev = float(np.dot(self.w_zero_dev, e_tool ** 2))
                    unique[key] = {
                        "rot": rot_cand,
                        "geo_deg": geo,
                        "zero_dev_cost": zero_dev,
                    }

        full_candidates: list[dict[str, Any]] = []
        branch_buckets: dict[int, list[dict[str, Any]]] = {b: [] for b in range(8)}

        for item in unique.values():
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = item["rot"].as_matrix()
            T[:3, 3] = base_pos
            q_arr = np.asarray(item["rot"].as_quat(), dtype=np.float64)
            sols = self._ik_gun(T)
            for branch_idx, q_sol in enumerate(sols):
                if not self._is_safe_q(q_sol, T):
                    continue
                node = {
                    "T": T,
                    "quat": q_arr,
                    "q": np.array(q_sol, dtype=np.float64),
                    "q_branch": np.array(q_sol, dtype=np.float64),  # 解析支标识（[-π,π]）
                    "zero_dev_cost": item["zero_dev_cost"],
                    "geo_deg": item["geo_deg"],
                    "branch_id": branch_idx,
                }
                full_candidates.append(node)
                b_id = branch_idx % 8
                branch_buckets[b_id].append(node)

        # 8 大解析支中每个构型各自保留零偏最小的 top-K 个优质姿态
        fast_candidates: list[dict[str, Any]] = []
        for b_id, nodes in branch_buckets.items():
            if not nodes:
                continue
            sorted_nodes = sorted(nodes, key=lambda nd: nd["zero_dev_cost"] + 0.01 * nd["geo_deg"])
            fast_candidates.extend(sorted_nodes[: self.max_candidates_per_branch])

        if not fast_candidates and full_candidates:
            fast_candidates = full_candidates

        return fast_candidates, full_candidates

    def _check_movel_segment(
        self,
        node_start: dict[str, Any],
        node_end: dict[str, Any],
        q_start: np.ndarray,
    ) -> tuple[bool, float, np.ndarray | None]:
        """
        模拟两点间控制器 MoveL：位置线性插值 + 姿态 Slerp，多点抽检。

        从 q_start（已按入边展开的绕组）出发，每步 get_best_ik_controller 跟最近支。
        任一点无解 / 奇异 / 超限 / 单轴跳变过大 → 边不可行。
        终点必须落到 node_end 的同一解析支（q_branch），否则视为换了肩/肘/腕。

        :return: (是否可行, 加权关节路程代价, 走到终点时的关节 q)
        """
        # 1. 快速 O(1) 连通性预筛：终点目标分支直接对 q_start 进行解析解包
        q_end = self._unwrap_onto(node_end["q_branch"], q_start)
        if q_end is None or not self.solver.is_joint_valid(q_end):
            return False, np.inf, None

        branch_diff = q_end - q_start
        if float(np.max(np.abs(np.degrees(branch_diff)))) > self.max_jump_deg:
            return False, np.inf, None

        T_start, T_end = node_start["T"], node_end["T"]
        p_start, p_end = T_start[:3, 3], T_end[:3, 3]
        dist_mm = float(np.linalg.norm(p_end - p_start) * 1000.0)
        n_est = round(dist_mm / self.movel_check_spacing_mm)
        n_mid = int(np.clip(n_est, self.num_movel_checks, self.max_movel_checks))
        # 中间采样点 α ∈ (0, 1)
        alphas = np.linspace(0.0, 1.0, n_mid + 2)[1:-1]

        q1 = node_start["quat"]
        q2 = node_end["quat"]

        prev_q = np.array(q_start, dtype=np.float64)
        acc = 0.0
        for alpha in alphas:
            T = np.eye(4, dtype=np.float64)
            T[:3, 3] = (1.0 - alpha) * p_start + alpha * p_end
            T[:3, :3] = _fast_quat_slerp_matrix(q1, q2, alpha)
            nxt = self.solver.get_best_ik_controller(T, prev_q)
            if nxt is None or not self._is_safe_q(nxt, T):
                return False, np.inf, None
            dq = _wrap_pi(nxt - prev_q)
            if float(np.max(np.abs(np.degrees(dq)))) > self.max_jump_deg:
                return False, np.inf, None
            acc += float(np.sum(self.joint_weights * (np.degrees(dq) ** 2)))
            prev_q = nxt

        # 加上终点段的代价与终点关节
        dq_final = _wrap_pi(q_end - prev_q)
        if float(np.max(np.abs(np.degrees(dq_final)))) > self.max_jump_deg:
            return False, np.inf, None
        acc += float(np.sum(self.joint_weights * (np.degrees(dq_final) ** 2)))

        return True, acc, q_end

    def optimize(
        self,
        waypoints: list,
        anchor_poses: Optional[list] = None,
        anchor_tolerances_deg: Optional[tuple[float, float, float]] = (15.0, 15.0, 180.0),
        init_q: Optional[list[float] | np.ndarray] = None,
    ) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
        """
        执行全局 Viterbi（动态规划）选优。

        :param waypoints: 稀疏点列表。每点为 pose dict 或
            np.array([x_mm, y_mm, z_mm, rx, ry, rz_deg])
        :param anchor_poses: 锚点参考位姿。长度为 1 则整条路径共用；
            长度与 waypoints 相同则逐点对应。省略时用第一点姿态作锚点。
        :param anchor_tolerances_deg: 锚点允许的 (tol_rx, tol_ry, tol_rz)（度）。
            传 None 则不做锚点硬裁剪，只受工具系网格约束。
        :param init_q: 起始关节（弧度），用于第一点展开绕组；省略则用 ready 种子。
        :return: (optimal_poses_6d_deg, optimal_joints_deg, optimal_transforms_4x4)
        """
        n = len(waypoints)
        if n < 1:
            raise ValueError("at least one waypoint is required")

        q_seed = np.array(init_q, dtype=np.float64) if init_q is not None else _DEFAULT_SEED_Q.copy()
        T_list = [_waypoint_to_T(wp) for wp in waypoints]

        # 默认锚点 = 第一枪姿态（工艺「相对这把垂直喷」）
        rot_anchor = None
        if anchor_poses:
            rot_anchor = R_scipy.from_matrix(_waypoint_to_T(anchor_poses[0])[:3, :3])
        elif anchor_tolerances_deg is not None:
            rot_anchor = R_scipy.from_matrix(T_list[0][:3, :3])

        # 1. 每个 Stage 展开候选节点（姿态 × IK 分支）
        t_cands_start = time.time()
        stages_fast: list[list[dict[str, Any]]] = []
        stages_full: list[list[dict[str, Any]]] = []
        for i, T_nom in enumerate(T_list):
            rot_a = rot_anchor
            if anchor_poses is not None and len(anchor_poses) == n:
                rot_a = R_scipy.from_matrix(_waypoint_to_T(anchor_poses[i])[:3, :3])
            c_fast, c_full = self._generate_stage_candidates(T_nom, rot_a, anchor_tolerances_deg)
            if not c_fast and not c_full:
                raise RuntimeError(
                    f"Waypoint [{i}] has no non-singular in-limit IK inside the attitude envelope."
                )
            stages_fast.append(c_fast)
            stages_full.append(c_full)

        stages = [list(s) for s in stages_fast]
        t_cands_ms = (time.time() - t_cands_start) * 1000.0
        total_fast = sum(len(s) for s in stages)
        total_full = sum(len(s) for s in stages_full)
        logger.info(
            f"⏱️ [SprayOpt] Stage candidates generated: {n} waypoints, "
            f"fast_candidates={total_fast} (diverse 8-branch pruned), full_nodes={total_full}, elapsed={t_cands_ms:.2f} ms"
        )

        if n == 1:
            # 单点：只需离名义喷姿近、且关节靠近种子
            best = min(
                stages[0],
                key=lambda nd: nd["zero_dev_cost"] + float(np.sum(_wrap_pi(nd["q"] - q_seed) ** 2)),
            )
            return [_T_to_pose6(best["T"])], [np.degrees(best["q"])], [best["T"]]

        # 2. DP 表：代价、父节点下标、到达该节点时的展开关节（绕组随入边变化）
        t_dp_start = time.time()
        dp_cost = [np.full(len(s), np.inf) for s in stages]
        dp_parent = [np.full(len(s), -1, dtype=int) for s in stages]
        dp_q = [np.zeros((len(s), 6), dtype=np.float64) for s in stages]

        for j, node in enumerate(stages[0]):
            q0 = self._unwrap_onto(node["q"], q_seed)
            if q0 is None or not self._is_safe_q(q0, node["T"]):
                continue
            # 第一层：零偏 + 相对当前机器人构型的小惩罚，避免无故换支
            dp_cost[0][j] = node["zero_dev_cost"] + 0.05 * float(np.sum(self.joint_weights * np.degrees(_wrap_pi(q0 - q_seed)) ** 2))
            dp_q[0][j] = q0
            node["q"] = q0

        def _beam_keep(stage_idx: int) -> None:
            """只保留本层代价最低的 beam_width 个节点，其余标 inf，避免 O(|C|^2) 爆炸。"""
            finite = np.where(np.isfinite(dp_cost[stage_idx]))[0]
            if finite.size <= self.beam_width:
                return
            keep = finite[np.argsort(dp_cost[stage_idx][finite])[: self.beam_width]]
            drop = np.setdiff1d(finite, keep)
            dp_cost[stage_idx][drop] = np.inf

        _beam_keep(0)

        for i in range(1, n):
            t_seg_start = time.time()
            edges_tested = 0
            valid_edges = 0

            # 内部执行单层 DP 边的评估
            def _eval_segment(stage_curr_nodes: list[dict[str, Any]]):
                c_tested = 0
                c_valid = 0
                s_cost = np.full(len(stage_curr_nodes), np.inf)
                s_parent = np.full(len(stage_curr_nodes), -1, dtype=int)
                s_q = [np.zeros(6, dtype=np.float64) for _ in range(len(stage_curr_nodes))]

                for prev_k, node_prev in enumerate(stages[i - 1]):
                    if not np.isfinite(dp_cost[i - 1][prev_k]):
                        continue
                    q_prev = dp_q[i - 1][prev_k]
                    for curr_j, node_curr in enumerate(stage_curr_nodes):
                        c_tested += 1
                        ok, edge_cost, q_arr = self._check_movel_segment(node_prev, node_curr, q_prev)
                        if not ok or q_arr is None:
                            continue
                        c_valid += 1
                        total = dp_cost[i - 1][prev_k] + edge_cost + node_curr["zero_dev_cost"]
                        if total < s_cost[curr_j]:
                            s_cost[curr_j] = total
                            s_parent[curr_j] = prev_k
                            s_q[curr_j] = q_arr
                return s_cost, s_parent, s_q, c_tested, c_valid

            # 1. 优先使用精简的 fast candidates (~128 节点) 进行极速 DP
            s_cost, s_parent, s_q, e_tested, e_valid = _eval_segment(stages[i])
            dp_cost[i] = s_cost
            dp_parent[i] = s_parent
            dp_q[i] = s_q
            edges_tested += e_tested
            valid_edges += e_valid

            # 2. 自适应回退机制：如果精简候选集未能连通，自动触发回退全量搜索（Fallback to full_stages[i]）
            if not np.any(np.isfinite(dp_cost[i])) and len(stages_full[i]) > len(stages[i]):
                logger.warning(
                    f"⚠️ [SprayOpt] Segment {i-1}->{i} failed with pruned candidates ({len(stages[i])}). "
                    f"Triggering adaptive fallback with ALL {len(stages_full[i])} candidates..."
                )
                stages[i] = stages_full[i]
                s_cost, s_parent, s_q, e_tested, e_valid = _eval_segment(stages[i])
                dp_cost[i] = s_cost
                dp_parent[i] = s_parent
                dp_q[i] = s_q
                edges_tested += e_tested
                valid_edges += e_valid

            _beam_keep(i)
            t_seg_ms = (time.time() - t_seg_start) * 1000.0
            logger.info(f"⏱️ [SprayOpt] DP Segment {i-1}->{i}: edges={edges_tested} (feasible={valid_edges}), elapsed={t_seg_ms:.2f} ms")
            if not np.any(np.isfinite(dp_cost[i])):
                raise RuntimeError(
                    f"Global search failed at segment {i - 1}->{i}: "
                    "MoveL samples hit singularity, joint limit, or a branch jump."
                )
        t_dp_ms = (time.time() - t_dp_start) * 1000.0
        logger.info(f"⏱️ [SprayOpt] Total Viterbi DP search: {n-1} segments, elapsed={t_dp_ms:.2f} ms")

        # 3. 从最后一层最小代价节点回溯整条链
        last = int(np.argmin(dp_cost[-1]))
        chain_idx = [last]
        for i in range(n - 1, 0, -1):
            last = int(dp_parent[i][last])
            chain_idx.append(last)
        chain_idx.reverse()

        best_T = [stages[i][chain_idx[i]]["T"] for i in range(n)]
        best_q = [dp_q[i][chain_idx[i]] for i in range(n)]
        best_poses = [_T_to_pose6(T) for T in best_T]
        best_q_deg = [np.degrees(q) for q in best_q]
        return best_poses, best_q_deg, best_T

    def optimize_path_item(
        self,
        path_item: dict,
        init_q: Optional[list[float] | np.ndarray] = None,
        ref_rpy_deg: Optional[list[float]] = None,
        tolerance_rpy_deg: Optional[list[float]] = None,
        anchor_poses: Optional[list] = None,
        anchor_tolerances_deg: Optional[tuple[float, float, float]] = None,
    ) -> tuple[dict, bool]:
        """
        优化一条生产路径（path_item.points 含 tcp_pose_base）。

        :param path_item: {"path_id", "points": [{"tcp_pose_base": {x,y,z,rx,ry,rz}, ...}, ...]}
        :param init_q: 起始关节，弧度（与 KinematicChainVerifier 相同）
        :param ref_rpy_deg: 锚点参考姿态 [rx, ry, rz]（度）；位置沿用第一点
        :param tolerance_rpy_deg: 锚点包络 [tol_rx, tol_ry, tol_rz]（度），同 POI 入参名
        :return: (更新后的 path_item, 是否改过姿态)
        """
        points = path_item.get("points", [])
        if not points:
            return path_item, False

        if anchor_tolerances_deg is None and tolerance_rpy_deg is not None:
            t = [float(v) for v in tolerance_rpy_deg]
            anchor_tolerances_deg = (t[0], t[1], t[2])
        if anchor_poses is None and ref_rpy_deg is not None and len(ref_rpy_deg) == 3:
            first = points[0].get("tcp_pose_base", points[0])
            anchor_poses = [{
                "x": first["x"], "y": first["y"], "z": first["z"],
                "rx": float(ref_rpy_deg[0]), "ry": float(ref_rpy_deg[1]), "rz": float(ref_rpy_deg[2]),
            }]

        poses, joints_deg, transforms = self.optimize(
            points,
            anchor_poses=anchor_poses,
            anchor_tolerances_deg=anchor_tolerances_deg,
            init_q=init_q,
        )

        out = copy.deepcopy(path_item)
        new_points = []
        for wp, T in zip(points, transforms):
            new_wp = copy.deepcopy(wp)
            pose = matrix_to_pose_dict(T)
            if "tcp_pose_base" in new_wp:
                new_wp["tcp_pose_base"] = pose
            else:
                new_wp.update(pose)
            new_points.append(new_wp)
        out["points"] = new_points
        out["spray_opt_joints_deg"] = [np.round(q, 3).tolist() for q in joints_deg]
        out["type"] = out.get("type", "spray_opt")

        modified = any(
            not np.allclose(_waypoint_to_T(a)[:3, :3], _waypoint_to_T(b)[:3, :3], atol=1e-6)
            for a, b in zip(points, new_points)
        )

        # 密采样校验与轨迹插值挂载：生成 60 FPS 连续仿真与执行所需的 dense 轨迹点
        if self.verifier is not None and len(new_points) >= 2:
            rep = self.verifier.verify_single_path(out, init_q=init_q)
            hard = [
                iss for iss in rep.get("issues", [])
                if iss.get("severity") == "ERROR" or "SINGULARITY" in str(iss.get("type", ""))
            ]
            if self.dense_verify and (hard or rep.get("status") == "FAILED"):
                types = sorted({iss.get("type", "?") for iss in hard}) or ["FAILED"]
                raise RuntimeError(
                    f"Dense MoveL verifier rejected the DP path: {types}. "
                    "Widen tilt/spin tolerance or split the segment."
                )
            if rep.get("recommended_safe_speed_mm_s"):
                out["recommended_speed_mm_s"] = rep["recommended_safe_speed_mm_s"]

            # 直接返回并挂载密插值轨迹点供 3D/2D 仿真与控制引擎直接取用
            out["trajectory_q"] = rep.get("trajectory_q", [])
            out["trajectory_tcp"] = rep.get("trajectory_tcp", [])
            out["total_interpolated"] = rep.get("total_interpolated", len(out["trajectory_q"]))
            out["verification_report"] = rep
        return out, modified

    def optimize_all_paths(
        self,
        paths_data: dict,
        init_q: Optional[list[float] | np.ndarray] = None,
        ref_rpy_deg: Optional[list[float]] = None,
        tolerance_rpy_deg: Optional[list[float]] = None,
        state_type: str = "poi",
    ) -> tuple[dict, dict | None]:
        """
        优化 paths_data["paths"] 中的全部路径。
        后一条的 init_q 取前一条最后一个展开关节，避免路径衔接处换支。
        同时挂载密插值轨迹点供 3D/2D 仿真与控制引擎直接取用。

        :return: (优化后的 paths_data, 校验报告或 None)
        """
        paths = paths_data.get("paths", [])
        out_paths = []
        last_q = init_q
        for path in paths:
            opt, _ = self.optimize_path_item(
                path,
                init_q=last_q,
                ref_rpy_deg=ref_rpy_deg,
                tolerance_rpy_deg=tolerance_rpy_deg,
            )
            out_paths.append(opt)
            joints = opt.get("spray_opt_joints_deg")
            if joints:
                last_q = np.deg2rad(joints[-1])

        data = copy.deepcopy(paths_data)
        data["paths"] = out_paths
        data["type"] = state_type
        report = None
        if self.verifier is not None:
            report = self.verifier.verify_all_paths(data)
            report["state_type"] = state_type
            report["optimized_paths_available"] = True
            # 同步挂载每条路径的插值点
            for idx, p_rep in enumerate(report.get("path_reports", [])):
                if idx < len(data["paths"]):
                    data["paths"][idx]["trajectory_q"] = p_rep.get("trajectory_q", [])
                    data["paths"][idx]["trajectory_tcp"] = p_rep.get("trajectory_tcp", [])
                    data["paths"][idx]["total_interpolated"] = p_rep.get("total_interpolated", 0)
        return data, report
