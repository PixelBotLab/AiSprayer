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
   （R_xyz 与 scipy Euler 'xyz' 相同，即 Rz @ Ry @ Rx）
2. 锚点姿态硬包络：超出则按相对 Euler-xyz 裁剪回边界；同一姿态只保留
   与原始目标测地夹角最小的那个
3. 零偏代价：能垂直喷则垂直喷（wx/wy 大、wz 很小），只有边不可行时 DP 才会动用倾角
4. 节点硬过滤：关节超限或肩/肘/腕奇异的 IK 解直接丢弃
5. 相邻 waypoint 之间的 MoveL 抽检：O(1) 预筛（肘腕家族 + 按实际段长缩放的整段行程）
        后按间距采样；45° 只约束相邻抽检点。上游稀疏点典型间距约 10 cm，不是固定段长
6. Viterbi DP 全局回溯 + beam；本层断连时回退该层全量候选
7. dense_verify=True 且挂了 KinematicChainVerifier 时，optimize() 用 1.5 mm 密采样做硬门

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

from ..cr5_kinematics import CR5Kinematics, _SHOULDER_HALF_RAD, _SING_SIN
from ..cr5_ur_kin import D4, D6
from .path_interpolator import matrix_to_pose_dict, pose_dict_to_matrix

logger = logging.getLogger(__name__)

PI = math.pi
# 与 KinematicChainVerifier.BRANCH_JUMP_DEG 对齐：相邻抽检点超过该值视为换支 / 翻腕
_BRANCH_JUMP_DEG = 45.0
# 跟支到达终点后，与节点解析解的最大允许偏差（同支数值误差，远小于 180° 换支）
_BRANCH_MATCH_DEG = 5.0
# 整段起终点允许的单轴行程默认上限（度）。按段长放大，且远宽于相邻 45°，
# 避免把平滑走出 50–90° 的合法边误杀，同时把明显换支的边挡在 IK 循环外。
_SEGMENT_TRAVEL_DEG = 120.0
_SEGMENT_TRAVEL_CAP_DEG = 170.0
_SEGMENT_TRAVEL_DEG_PER_MM = 0.9
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
    """两姿态在 SO(3) 上的测地角（度）。保留供单测与外部兼容。"""
    return float(np.degrees((rot_a.inv() * rot_b).magnitude()))


def _geodesic_R_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    """两 3×3 旋转矩阵的测地角（度）。"""
    c = 0.5 * (float(np.trace(Ra.T @ Rb)) - 1.0)
    return float(np.degrees(math.acos(min(1.0, max(-1.0, c)))))


def _R_from_euler_xyz_deg(rx: float, ry: float, rz: float) -> np.ndarray:
    """与 scipy Rotation.from_euler('xyz') 一致：R = Rz @ Ry @ Rx。"""
    ax, ay, az = math.radians(rx), math.radians(ry), math.radians(rz)
    cx, sx = math.cos(ax), math.sin(ax)
    cy, sy = math.cos(ay), math.sin(ay)
    cz, sz = math.cos(az), math.sin(az)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return Rz @ Ry @ Rx


def _euler_xyz_deg_from_R(Rm: np.ndarray) -> np.ndarray:
    """3×3 → scipy as_euler('xyz')（度）。对应 R = Rz @ Ry @ Rx。"""
    sy = min(1.0, max(-1.0, -float(Rm[2, 0])))
    y = math.asin(sy)
    if abs(sy) < 0.999999:
        x = math.atan2(float(Rm[2, 1]), float(Rm[2, 2]))
        z = math.atan2(float(Rm[1, 0]), float(Rm[0, 0]))
    else:
        x = math.atan2(math.copysign(1.0, sy) * float(Rm[0, 1]), float(Rm[1, 1]))
        z = 0.0
    return np.array([math.degrees(x), math.degrees(y), math.degrees(z)], dtype=np.float64)


def _euler_xyz_deg_from_R_batch(Rm: np.ndarray) -> np.ndarray:
    """批量 3×3 → xyz 欧拉角（度），与 _euler_xyz_deg_from_R 同语义。"""
    sy = np.clip(-Rm[:, 2, 0], -1.0, 1.0)
    y = np.arcsin(sy)
    gimbal = np.abs(sy) >= 0.999999
    x = np.empty(Rm.shape[0], dtype=np.float64)
    z = np.empty(Rm.shape[0], dtype=np.float64)
    ok = ~gimbal
    x[ok] = np.arctan2(Rm[ok, 2, 1], Rm[ok, 2, 2])
    z[ok] = np.arctan2(Rm[ok, 1, 0], Rm[ok, 0, 0])
    if np.any(gimbal):
        sign = np.sign(sy[gimbal])
        sign[sign == 0.0] = 1.0
        x[gimbal] = np.arctan2(sign * Rm[gimbal, 0, 1], Rm[gimbal, 1, 1])
        z[gimbal] = 0.0
    return np.degrees(np.stack([x, y, z], axis=1))


def _quat_from_R(Rm: np.ndarray) -> np.ndarray:
    """3×3 → 四元数 [x, y, z, w]，w>=0，对应 scipy as_quat 双覆盖约定。"""
    m00, m01, m02 = float(Rm[0, 0]), float(Rm[0, 1]), float(Rm[0, 2])
    m10, m11, m12 = float(Rm[1, 0]), float(Rm[1, 1]), float(Rm[1, 2])
    m20, m21, m22 = float(Rm[2, 0]), float(Rm[2, 1]), float(Rm[2, 2])
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    return np.array([x, y, z, w], dtype=np.float64)


def _quat_key_arr(q: np.ndarray) -> tuple:
    """四元数去重键。q 与 -q 同一旋转，统一 w>=0 后量化到 5 位小数。"""
    if q[3] < 0.0:
        q = -q
    return tuple(np.round(q, 5).tolist())


def _axis_rot_mats(angles_deg: np.ndarray, axis: int) -> np.ndarray:
    """预计算一轴旋转矩阵堆叠，shape (N, 3, 3)。axis: 0=X, 1=Y, 2=Z。"""
    n = int(angles_deg.size)
    mats = np.zeros((n, 3, 3), dtype=np.float64)
    for i, d in enumerate(angles_deg):
        a = math.radians(float(d))
        c, s = math.cos(a), math.sin(a)
        if axis == 0:
            mats[i, 0, 0] = 1.0
            mats[i, 1, 1] = c
            mats[i, 1, 2] = -s
            mats[i, 2, 1] = s
            mats[i, 2, 2] = c
        elif axis == 1:
            mats[i, 0, 0] = c
            mats[i, 0, 2] = s
            mats[i, 1, 1] = 1.0
            mats[i, 2, 0] = -s
            mats[i, 2, 2] = c
        else:
            mats[i, 0, 0] = c
            mats[i, 0, 1] = -s
            mats[i, 1, 0] = s
            mats[i, 1, 1] = c
            mats[i, 2, 2] = 1.0
    return mats


def _project_R_to_anchor(
    R_cand: np.ndarray,
    R_anchor: np.ndarray,
    anchor_tol_deg: tuple[float, float, float],
) -> np.ndarray:
    """相对 Euler-xyz 盒投影，矩阵版（与 _project_to_anchor_envelope 同语义）。"""
    rel = _euler_xyz_deg_from_R(R_anchor.T @ R_cand)
    rel = (rel + 180.0) % 360.0 - 180.0
    tol = np.abs(np.asarray(anchor_tol_deg, dtype=np.float64))
    clipped = np.clip(rel, -tol, tol)
    if np.allclose(clipped, rel, atol=1e-12):
        return R_cand
    return R_anchor @ _R_from_euler_xyz_deg(float(clipped[0]), float(clipped[1]), float(clipped[2]))


def _project_R_to_anchor_batch(
    R_cands: np.ndarray,
    R_anchor: np.ndarray,
    anchor_tol_deg: tuple[float, float, float],
) -> np.ndarray:
    """_project_R_to_anchor 的批量版。R_cands shape (N, 3, 3)。"""
    rel = _euler_xyz_deg_from_R_batch(np.matmul(R_anchor.T, R_cands))
    rel = (rel + 180.0) % 360.0 - 180.0
    tol = np.abs(np.asarray(anchor_tol_deg, dtype=np.float64))
    clipped = np.clip(rel, -tol, tol)
    need = np.any(np.abs(clipped - rel) > 1e-12, axis=1)
    if not np.any(need):
        return R_cands
    out = np.array(R_cands, dtype=np.float64, copy=True)
    R_off = R_scipy.from_euler("xyz", clipped[need], degrees=True).as_matrix()
    out[need] = np.matmul(R_anchor, R_off)
    return out


def _branch_key(q: np.ndarray) -> int:
    """
    与 inverse() 返回列表下标无关的 8 路构型键：肩 × 肘 × 腕。

    肘/腕解析对是 arccos 与 2π-arccos，折到 [0, 2π) 后落在 [0, π] 或 (π, 2π]。
    肩用 wrap(q1) 的符号作半平面划分（两肩不一定分居 [0,π]/[π,2π]）。
    """
    qw = _wrap_pi(np.asarray(q, dtype=np.float64).reshape(-1)[:6])
    q02 = np.mod(qw, 2.0 * PI)
    shoulder = 0 if qw[0] >= 0.0 else 1
    elbow = 0 if q02[2] <= PI else 1
    wrist = 0 if q02[4] <= PI else 1
    return (shoulder << 2) | (elbow << 1) | wrist


def _ew_family(q: np.ndarray) -> int:
    """肘/腕 2bit 家族。不含肩：连续段上 q1 过 0° 不应被当成换支。"""
    return _branch_key(q) & 3


def _ctrl_to_urdf_into(T_ctrl: np.ndarray, out: np.ndarray) -> np.ndarray:
    """controller_matrix_to_urdf 的就地版本，避免边循环里反复分配 4×4。"""
    R = T_ctrl[:3, :3]
    p = T_ctrl[:3, 3]
    out[0, 0] = -R[0, 2]
    out[0, 1] = R[0, 0]
    out[0, 2] = R[0, 1]
    out[0, 3] = -p[0]
    out[1, 0] = -R[1, 2]
    out[1, 1] = R[1, 0]
    out[1, 2] = R[1, 1]
    out[1, 3] = -p[1]
    out[2, 0] = R[2, 2]
    out[2, 1] = -R[2, 0]
    out[2, 2] = -R[2, 1]
    out[2, 3] = p[2]
    out[3, 0] = 0.0
    out[3, 1] = 0.0
    out[3, 2] = 0.0
    out[3, 3] = 1.0
    return out


def _ctrl_to_urdf_batch(T_ctrl: np.ndarray) -> np.ndarray:
    """controller_matrix_to_urdf 的批量版本，T_ctrl shape (N, 4, 4)。"""
    R = T_ctrl[:, :3, :3]
    p = T_ctrl[:, :3, 3]
    out = np.zeros_like(T_ctrl)
    out[:, 0, 0] = -R[:, 0, 2]
    out[:, 0, 1] = R[:, 0, 0]
    out[:, 0, 2] = R[:, 0, 1]
    out[:, 0, 3] = -p[:, 0]
    out[:, 1, 0] = -R[:, 1, 2]
    out[:, 1, 1] = R[:, 1, 0]
    out[:, 1, 2] = R[:, 1, 1]
    out[:, 1, 3] = -p[:, 1]
    out[:, 2, 0] = R[:, 2, 2]
    out[:, 2, 1] = -R[:, 2, 0]
    out[:, 2, 2] = -R[:, 2, 1]
    out[:, 2, 3] = p[:, 2]
    out[:, 3, 3] = 1.0
    return out


def _precompute_offset_rots(xs: np.ndarray, ys: np.ndarray, zs: np.ndarray) -> np.ndarray:
    """预计算全部 R_off = Rz @ Ry @ Rx，shape (Nz*Ny*Nx, 3, 3)。"""
    Rx = _axis_rot_mats(xs, 0)
    Ry = _axis_rot_mats(ys, 1)
    Rz = _axis_rot_mats(zs, 2)
    rzy = Rz[:, None, None, :, :] @ Ry[None, :, None, :, :]
    return (rzy @ Rx[None, None, :, :, :]).reshape(-1, 3, 3)


def _shoulder_ok_batch(T_urdf: np.ndarray) -> np.ndarray:
    """与 CR5Kinematics._shoulder_q1_half_separation_rad 同一套索引。"""
    t02 = -T_urdf[:, 0, 0]
    t03 = -T_urdf[:, 0, 3]
    t12 = -T_urdf[:, 1, 0]
    t13 = -T_urdf[:, 1, 3]
    a = D6 * t12 - t13
    b = D6 * t02 - t03
    r = a * a + b * b
    half = np.zeros(T_urdf.shape[0], dtype=np.float64)
    valid = r > 1e-16
    ratio = np.empty_like(r)
    ratio[valid] = abs(D4) / np.sqrt(r[valid])
    below = valid & (ratio <= -1.0)
    mid = valid & (ratio > -1.0) & (ratio < 1.0)
    half[below] = PI
    half[mid] = np.arccos(ratio[mid])
    return half >= _SHOULDER_HALF_RAD


def _make_dp_node(
    T: np.ndarray,
    quat: np.ndarray,
    q: np.ndarray,
    zero_dev: float,
    geo_deg: float,
    branch_id: int,
) -> dict[str, Any]:
    return {
        "T": T,
        "quat": quat,
        "q": q,
        "q_branch": q,
        "zero_dev_cost": zero_dev,
        "geo_deg": geo_deg,
        "branch_id": branch_id,
        "ew_family": branch_id & 3,
    }


def _fast_quat_slerp_into(q1: np.ndarray, q2: np.ndarray, alpha: float, out_R: np.ndarray) -> None:
    """单位四元数 Slerp 写入已有 3×3，不分配新矩阵。"""
    dot = float(q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3])
    q2_mod = q2 if dot >= 0.0 else -q2
    dot_abs = abs(dot)
    if dot_abs > 0.9995:
        q = (1.0 - alpha) * q1 + alpha * q2_mod
        norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        q = q / norm
    else:
        theta = math.acos(min(max(dot_abs, -1.0), 1.0))
        sin_theta = math.sin(theta)
        q = (math.sin((1.0 - alpha) * theta) * q1 + math.sin(alpha * theta) * q2_mod) / sin_theta
    x, y, z, w = q[0], q[1], q[2], q[3]
    out_R[0, 0] = 1.0 - 2.0 * (y * y + z * z)
    out_R[0, 1] = 2.0 * (x * y - z * w)
    out_R[0, 2] = 2.0 * (x * z + y * w)
    out_R[1, 0] = 2.0 * (x * y + z * w)
    out_R[1, 1] = 1.0 - 2.0 * (x * x + z * z)
    out_R[1, 2] = 2.0 * (y * z - x * w)
    out_R[2, 0] = 2.0 * (x * z - y * w)
    out_R[2, 1] = 2.0 * (y * z + x * w)
    out_R[2, 2] = 1.0 - 2.0 * (x * x + y * y)


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
    锚点硬包络投影（相对 Euler-xyz 盒）。保留供单测与外部兼容。

    相对姿态 R_rel = R_anchor^{-1} * R_cand，拆成 xyz 欧拉角后逐轴 clip 到
    ±(tol_x, tol_y, tol_z)。已在盒内则不变；越界则落到边界。
    这是工艺容差的硬约束，不是测地球面投影；同姿态去重在网格层用测地角完成。
    """
    R = _project_R_to_anchor(rot_cand.as_matrix(), rot_anchor.as_matrix(), anchor_tol_deg)
    return R_scipy.from_matrix(R)


class SprayWaypointOptimizer:
    """
    针对喷涂/表面加工稀疏 Waypoint 的离散姿态优化器。

    在容差内为每个点选出姿态与关节构型，使相邻点之间的控制器 MoveL 直线
    不经过奇异、不翻腕、不超关节硬限位。内部关节用弧度；对外 6D 位姿用度。

    线程安全：实例复用 _T_work / _T_urdf_work 与 _last_dense_report，且底层
    CR5Kinematics（尤其 C++ 后端）亦不可跨线程共享；请每线程独立实例。
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
        beam_width: int = 32,
        max_candidates_per_branch: int = 16,
        dense_verify: bool = True,
        ik_returns_degrees: bool = False,
        max_segment_travel_deg: float = _SEGMENT_TRAVEL_DEG,
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
        :param max_joint_jump_deg: 抽检相邻样本允许的最大单轴跳变（度），超过判为翻腕/换支。
            只约束相邻抽检点，不约束整段起终点行程。
        :param max_segment_travel_deg: 整段起终点单轴行程的 O(1) 预筛下限（度）。
            实际阈值 = min(170, max(本值, 0.9°/mm × 段长))，用于挡住明显换支，
            不替代相邻 45° 跳变检查。
        :param weight_zero_dev: 零偏惩罚 (wx, wy, wz)，按「相对名义姿态的工具系欧拉角」每度平方计。
            默认倾角很贵、自旋几乎免费 → 能垂直喷则垂直喷。
        :param joint_weights: 6 轴边代价权重，抽检 Δq 的加权平方和
        :param beam_width: 每层 DP 只保留代价最低的这么多节点，控制 10 cm 段的边数
        :param max_candidates_per_branch: 按肩/肘/腕签名分的 8 桶中，每桶保留的最优姿态上限
            （如 16，则每层最多约 128 节点）。本层断连时回退该层全量候选。
        :param dense_verify: 是否在 optimize() 选出路径后跑密采样校验器（需传入 verifier）
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
        self.max_segment_travel_deg = float(max_segment_travel_deg)
        self._max_jump_rad = math.radians(self.max_jump_deg)
        self._match_rad = math.radians(_BRANCH_MATCH_DEG)
        self._deg2_from_rad2 = (180.0 / PI) ** 2
        self.w_zero_dev = np.array(weight_zero_dev, dtype=np.float64)
        # J2 略加重（大臂）、腕部略轻：边代价偏向少甩大关节
        self.joint_weights = np.array(
            joint_weights if joint_weights is not None else [1.0, 1.2, 1.0, 0.8, 0.8, 0.5],
            dtype=np.float64,
        )
        self.beam_width = max(8, int(beam_width))
        self.max_candidates_per_branch = max(1, int(max_candidates_per_branch))
        self.dense_verify = bool(dense_verify)
        self._T_work = np.eye(4, dtype=np.float64)
        self._T_urdf_work = np.eye(4, dtype=np.float64)
        self._last_dense_report: dict[str, Any] | None = None
        self._R_off = _precompute_offset_rots(
            _axis_grid(self.tol_x_deg),
            _axis_grid(self.tol_y_deg),
            _axis_grid(self.tol_z_deg),
        )
        self._alphas_cache: dict[int, np.ndarray] = {}
        self.T_tcp_inv = getattr(self.verifier.config, 'T_tcp_inv', None) if self.verifier and hasattr(self.verifier, 'config') else None

    def _ik_gun(self, T_gun: np.ndarray) -> list[np.ndarray]:
        """枪尖控制器系 4×4 → 最多 8 组 URDF 关节（弧度）。"""
        if self._ik_override is not None:
            sols = self._ik_override(T_gun)
            if not sols:
                return []
            if self.ik_returns_degrees:
                return [np.deg2rad(np.asarray(q, dtype=np.float64)) for q in sols]
            return [np.asarray(q, dtype=np.float64) for q in sols]
        if self.T_tcp_inv is not None:
            T_gun = T_gun @ self.T_tcp_inv
        T_urdf = self.solver.controller_matrix_to_urdf(T_gun)
        return list(self.solver.inverse(T_urdf))

    def _is_safe_q(
        self,
        q: np.ndarray,
        T_gun: np.ndarray,
        T_urdf: Optional[np.ndarray] = None,
    ) -> bool:
        """硬门：URDF 关节限位 + 肩/肘/腕奇异（与校验器同一套 check_singularity_risk）。"""
        if not self.solver.is_joint_valid(q):
            return False
        qv = np.asarray(q, dtype=np.float64).reshape(-1)
        if abs(math.sin(float(qv[4]))) < _SING_SIN or abs(math.sin(float(qv[2]))) < _SING_SIN:
            return False
        if T_urdf is None:
            if self.T_tcp_inv is not None:
                T_gun = T_gun @ self.T_tcp_inv
            T_urdf = self.solver.controller_matrix_to_urdf(T_gun)
        return self.solver.shoulder_q1_half_separation_rad(T_urdf) >= _SHOULDER_HALF_RAD

    def _track_ik_step(self, T_ctrl: np.ndarray, prev_q: np.ndarray) -> np.ndarray | None:
        """一次 URDF 转换 + 跟最近支 + 限位/奇异。失败返回 None。"""
        if self.T_tcp_inv is not None:
            T_ctrl = T_ctrl @ self.T_tcp_inv
        T_urdf = _ctrl_to_urdf_into(T_ctrl, self._T_urdf_work)
        nxt = self.solver.get_best_ik(T_urdf, prev_q)
        if nxt is None or not self.solver.is_joint_valid(nxt):
            return None
        if abs(math.sin(float(nxt[4]))) < _SING_SIN or abs(math.sin(float(nxt[2]))) < _SING_SIN:
            return None
        if self.solver.shoulder_q1_half_separation_rad(T_urdf) < _SHOULDER_HALF_RAD:
            return None
        return nxt

    def _dense_verify_or_raise(
        self,
        waypoints: list,
        transforms: list[np.ndarray],
        init_q: np.ndarray,
        path_id: int,
        path_name: str,
    ) -> dict[str, Any]:
        """对 DP 选出的稀疏位姿做 1.5 mm 密采样硬门。"""
        path_item = {
            "path_id": path_id,
            "name": path_name,
            "points": [
                {**wp, "tcp_pose_base": matrix_to_pose_dict(T)}
                for wp, T in zip(waypoints, transforms)
            ],
        }
        report = self.verifier.verify_single_path(path_item, init_q=init_q)
        hard = [
            iss for iss in report.get("issues", [])
            if iss.get("severity") == "ERROR" or "SINGULARITY" in str(iss.get("type", ""))
        ]
        if hard or report.get("status") == "FAILED":
            types = sorted({iss.get("type", "?") for iss in hard}) or ["FAILED"]
            raise RuntimeError(
                f"Dense MoveL verifier rejected the DP path: {types}. "
                "Widen tilt/spin tolerance or split the segment."
            )
        return report

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

    def _materialize_pack(self, pack: dict[str, Any]) -> list[dict[str, Any]]:
        """把紧凑候选包展开为 DP 节点 dict（仅自适应回退时调用）。"""
        nodes: list[dict[str, Any]] = []
        T_all = pack["T"]
        quat_all = pack["quat"]
        geo = pack["geo"]
        zero = pack["zero"]
        qs = pack["q"]
        pose_idx = pack["pose_idx"]
        branch_id = pack["branch_id"]
        for m in range(qs.shape[0]):
            pi = int(pose_idx[m])
            nodes.append(_make_dp_node(
                T_all[pi], quat_all[pi], qs[m],
                float(zero[pi]), float(geo[pi]), int(branch_id[m]),
            ))
        return nodes

    def _generate_stage_candidates(
        self,
        T_nominal: np.ndarray,
        anchor_rot: Optional[R_scipy] = None,
        anchor_tol_deg: Optional[tuple[float, float, float]] = None,
    ) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        """
        针对单个 Waypoint 生成 DP 节点：
        (姿态网格 × 锚点投影去重) → IK 全解 → 丢掉奇异/超限。

        :return: (fast_candidates, full_pack)
        fast_candidates 按肩/肘/腕签名分 8 桶，各留 zero_dev 最优的 top-K；
        full_pack 为全量有效候选的紧凑数组（本层断连时再物化）。
        """
        base_pos = T_nominal[:3, 3]
        R_nom = np.array(T_nominal[:3, :3], dtype=np.float64, copy=True)
        R_cands = np.matmul(R_nom, self._R_off)
        if anchor_rot is not None and anchor_tol_deg is not None:
            R_anc = np.array(anchor_rot.as_matrix(), dtype=np.float64, copy=True)
            R_cands = _project_R_to_anchor_batch(R_cands, R_anc, anchor_tol_deg)

        r_all = R_scipy.from_matrix(R_cands)
        geo_all = np.degrees((R_scipy.from_matrix(R_nom).inv() * r_all).magnitude())
        quats = np.asarray(r_all.as_quat(), dtype=np.float64)
        quats = np.where(quats[:, 3:4] < 0.0, -quats, quats)
        e_tool = _euler_xyz_deg_from_R_batch(np.matmul(R_nom.T, R_cands))
        e_tool = (e_tool + 180.0) % 360.0 - 180.0
        zero_all = e_tool ** 2 @ self.w_zero_dev

        best: dict[tuple, int] = {}
        keys = np.round(quats, 5)
        for i in range(R_cands.shape[0]):
            key = (float(keys[i, 0]), float(keys[i, 1]), float(keys[i, 2]), float(keys[i, 3]))
            prev = best.get(key)
            if prev is not None and geo_all[i] >= geo_all[prev]:
                continue
            best[key] = i
        keep_idx = np.fromiter(best.values(), dtype=np.int32, count=len(best))
        R_keep = R_cands[keep_idx]
        quat_keep = quats[keep_idx]
        geo_keep = geo_all[keep_idx]
        zero_keep = zero_all[keep_idx]

        empty_pack = {
            "T": np.zeros((0, 4, 4), dtype=np.float64),
            "quat": np.zeros((0, 4), dtype=np.float64),
            "geo": np.zeros(0, dtype=np.float64),
            "zero": np.zeros(0, dtype=np.float64),
            "q": np.zeros((0, 6), dtype=np.float64),
            "pose_idx": np.zeros(0, dtype=np.int32),
            "branch_id": np.zeros(0, dtype=np.int32),
        }
        P = int(keep_idx.size)
        if P == 0:
            return [], empty_pack

        T_ctrl = np.zeros((P, 4, 4), dtype=np.float64)
        T_ctrl[:, 3, 3] = 1.0
        T_ctrl[:, :3, :3] = R_keep
        T_ctrl[:, :3, 3] = base_pos
        if self.T_tcp_inv is not None:
            T_flange_ctrl = T_ctrl @ self.T_tcp_inv
            T_urdf = _ctrl_to_urdf_batch(T_flange_ctrl)
        else:
            T_urdf = _ctrl_to_urdf_batch(T_ctrl)

        if self._ik_override is not None:
            q_sols = np.zeros((P, 8, 6), dtype=np.float64)
            n_sols = np.zeros(P, dtype=np.int32)
            for i in range(P):
                sols = self._ik_gun(T_ctrl[i])
                n_sols[i] = min(8, len(sols))
                for k, sol in enumerate(sols[:8]):
                    q_sols[i, k] = sol
        else:
            q_sols, n_sols = self.solver.inverse_batch(T_urdf)

        shoulder_ok = _shoulder_ok_batch(T_urdf)
        jmin = self.solver.joint_min - 1e-4
        jmax = self.solver.joint_max + 1e-4
        pose_ok = shoulder_ok & (n_sols > 0)
        if not np.any(pose_ok):
            return [], empty_pack

        remap = np.full(P, -1, dtype=np.int32)
        kept_pose_idx = np.where(pose_ok)[0]
        remap[kept_pose_idx] = np.arange(kept_pose_idx.size, dtype=np.int32)

        idx_p = np.broadcast_to(np.arange(P)[:, None], q_sols.shape[:2])
        idx_k = np.broadcast_to(np.arange(8)[None, :], q_sols.shape[:2])
        sol_ok = (idx_k < n_sols[:, None]) & pose_ok[:, None]
        sol_ok &= np.all(q_sols >= jmin, axis=2) & np.all(q_sols <= jmax, axis=2)
        sol_ok &= (np.abs(np.sin(q_sols[:, :, 4])) >= _SING_SIN) & (
            np.abs(np.sin(q_sols[:, :, 2])) >= _SING_SIN
        )
        if not np.any(sol_ok):
            return [], empty_pack

        q_flat = q_sols[sol_ok]
        pose_idx = remap[idx_p[sol_ok]]
        qw = _wrap_pi(q_flat)
        q02 = np.mod(qw, 2.0 * PI)
        branch_id = (
            ((qw[:, 0] < 0.0).astype(np.int32) << 2)
            | ((q02[:, 2] > PI).astype(np.int32) << 1)
            | (q02[:, 4] > PI).astype(np.int32)
        )

        pack = {
            "T": T_ctrl[kept_pose_idx],
            "quat": quat_keep[kept_pose_idx],
            "geo": geo_keep[kept_pose_idx],
            "zero": zero_keep[kept_pose_idx],
            "q": q_flat,
            "pose_idx": pose_idx.astype(np.int32, copy=False),
            "branch_id": branch_id,
        }

        scores = pack["zero"][pack["pose_idx"]] + 0.01 * pack["geo"][pack["pose_idx"]]
        fast_candidates: list[dict[str, Any]] = []
        for b_id in range(8):
            idx = np.where(pack["branch_id"] == b_id)[0]
            if idx.size == 0:
                continue
            order = idx[np.argsort(scores[idx], kind="stable")]
            for m in order[: self.max_candidates_per_branch]:
                pi = int(pack["pose_idx"][m])
                fast_candidates.append(_make_dp_node(
                    pack["T"][pi], pack["quat"][pi], pack["q"][m],
                    float(pack["zero"][pi]), float(pack["geo"][pi]), int(pack["branch_id"][m]),
                ))

        if not fast_candidates:
            fast_candidates = self._materialize_pack(pack)
        return fast_candidates, pack

    def _walk_alphas(
        self,
        p_start: np.ndarray,
        p_end: np.ndarray,
        q1: np.ndarray,
        q2: np.ndarray,
        q_start: np.ndarray,
        alphas: np.ndarray,
        node_end: dict[str, Any],
        check_end_branch: bool,
    ) -> tuple[bool, float, np.ndarray | None]:
        """按 α 序列跟支。check_end_branch 时要求终点落到 node_end 解析支。"""
        cpp = self.solver.walk_movel_controller(
            p_start, p_end, q1, q2, q_start, alphas,
            node_end["q_branch"], check_end_branch,
            self._max_jump_rad, self._match_rad,
            self.joint_weights, self._deg2_from_rad2,
        )
        if cpp is not None:
            return cpp

        T = self._T_work
        prev_q = q_start
        acc = 0.0
        nxt = prev_q
        jump = self._max_jump_rad
        w = self.joint_weights
        deg2 = self._deg2_from_rad2
        for alpha in alphas:
            a = float(alpha)
            om = 1.0 - a
            T[0, 3] = om * p_start[0] + a * p_end[0]
            T[1, 3] = om * p_start[1] + a * p_end[1]
            T[2, 3] = om * p_start[2] + a * p_end[2]
            _fast_quat_slerp_into(q1, q2, a, T[:3, :3])
            nxt = self._track_ik_step(T, prev_q)
            if nxt is None:
                return False, np.inf, None
            dq = _wrap_pi(nxt - prev_q)
            if float(np.max(np.abs(dq))) > jump:
                return False, np.inf, None
            acc += float(np.sum(w * (dq * dq))) * deg2
            prev_q = nxt
        if check_end_branch:
            q_target = self._unwrap_onto(node_end["q_branch"], nxt)
            if q_target is None:
                return False, np.inf, None
            if float(np.max(np.abs(_wrap_pi(nxt - q_target)))) > self._match_rad:
                return False, np.inf, None
        return True, acc, nxt

    def _check_movel_segment(
        self,
        node_start: dict[str, Any],
        node_end: dict[str, Any],
        q_start: np.ndarray,
        is_jump: bool = False,
    ) -> tuple[bool, float, np.ndarray | None]:
        """
        模拟两点间控制器 MoveL：位置线性插值 + 姿态 Slerp，多点抽检（含终点）。
        如果是列间转移跳跃 (is_jump=True)，允许关节空间平滑过渡 (MoveJ/重定支)。

        从 q_start（已按入边展开的绕组）出发，每步跟最近支。
        任一点无解 / 奇异 / 超限 / 相邻单轴跳变过大 → 边不可行。
        终点必须落到 node_end 的同一解析支（与 q_branch 偏差 < 5°），否则视为换了肩/肘/腕。

        O(1) 预筛：绕组可展开、肘/腕家族相同、整段行程不超过按段长缩放的上限
        （默认 ≥120°，不是相邻 45°）。多数换支边在此被挡掉，避免走进完整 IK。

        :return: (是否可行, 加权关节路程代价, 走到终点时的关节 q)
        """
        if is_jump:
            q_end_hint = self._unwrap_onto(node_end["q_branch"], q_start)
            if q_end_hint is None:
                q_end_hint = node_end["q_branch"]
            if not self._is_safe_q(q_end_hint, node_end["T"]):
                return False, np.inf, None
            edge_cost = float(np.sum(self.joint_weights * np.degrees(_wrap_pi(q_end_hint - q_start)) ** 2))
            return True, edge_cost, q_end_hint

        q_end_hint = self._unwrap_onto(node_end["q_branch"], q_start)
        if q_end_hint is None:
            return False, np.inf, None

        ew_s = node_start.get("ew_family")
        if ew_s is None:
            ew_s = _ew_family(q_start)
        ew_e = node_end.get("ew_family")
        if ew_e is None:
            ew_e = _ew_family(node_end["q_branch"])
        if ew_s != ew_e:
            return False, np.inf, None

        T_start, T_end = node_start["T"], node_end["T"]
        if self.T_tcp_inv is not None:
            T_s_flange = T_start @ self.T_tcp_inv
            T_e_flange = T_end @ self.T_tcp_inv
            p_start, p_end = T_s_flange[:3, 3], T_e_flange[:3, 3]
            q1 = _quat_from_R(T_s_flange[:3, :3])
            q2 = _quat_from_R(T_e_flange[:3, :3])
        else:
            p_start, p_end = T_start[:3, 3], T_end[:3, 3]
            q1 = node_start["quat"]
            q2 = node_end["quat"]

        dist_mm = float(np.linalg.norm(p_end - p_start) * 1000.0)
        travel = float(np.max(np.abs(np.degrees(_wrap_pi(q_end_hint - q_start)))))
        travel_lim = min(
            _SEGMENT_TRAVEL_CAP_DEG,
            max(self.max_segment_travel_deg, _SEGMENT_TRAVEL_DEG_PER_MM * dist_mm),
        )
        if travel > travel_lim:
            return False, np.inf, None

        n_est = round(dist_mm / self.movel_check_spacing_mm)
        n_mid = int(np.clip(n_est, self.num_movel_checks, self.max_movel_checks))
        alphas_full = self._alphas_cache.get(n_mid)
        if alphas_full is None:
            alphas_full = np.linspace(0.0, 1.0, n_mid + 2, dtype=np.float64)[1:]
            self._alphas_cache[n_mid] = alphas_full

        return self._walk_alphas(
            p_start, p_end, q1, q2, q_start, alphas_full, node_end, check_end_branch=True,
        )

    def optimize(
        self,
        waypoints: list,
        anchor_poses: Optional[list] = None,
        anchor_tolerances_deg: Optional[tuple[float, float, float]] = (15.0, 15.0, 180.0),
        init_q: Optional[list[float] | np.ndarray] = None,
        path_id: Optional[int] = None,
        path_name: Optional[str] = None,
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
        :param path_id: 密采样校验报告 path_id；optimize_path_item 传入真实值。
        :param path_name: 密采样校验报告 name；optimize_path_item 传入真实值。
        :return: (optimal_poses_6d_deg, optimal_joints_deg, optimal_transforms_4x4)
        """
        n = len(waypoints)
        if n < 1:
            raise ValueError("at least one waypoint is required")

        self._last_dense_report = None
        q_seed = np.array(init_q, dtype=np.float64) if init_q is not None else _DEFAULT_SEED_Q.copy()
        T_list = [_waypoint_to_T(wp) for wp in waypoints]

        # 默认锚点：若未指定全局参考位姿，则按逐点名义法向姿态作为各自的锚点中心
        rot_anchor = None
        if anchor_poses and len(anchor_poses) == 1:
            rot_anchor = R_scipy.from_matrix(_waypoint_to_T(anchor_poses[0])[:3, :3])

        # 1. 每个 Stage 展开候选节点（姿态 × IK 分支）
        t_cands_start = time.time()
        stages_fast: list[list[dict[str, Any]]] = []
        stages_full: list[list[dict[str, Any]]] = []
        for i, T_nom in enumerate(T_list):
            if anchor_poses is not None and len(anchor_poses) == n:
                rot_a = R_scipy.from_matrix(_waypoint_to_T(anchor_poses[i])[:3, :3])
            elif rot_anchor is not None:
                rot_a = rot_anchor
            elif anchor_tolerances_deg is not None:
                rot_a = R_scipy.from_matrix(T_nom[:3, :3])
            else:
                rot_a = None
            c_fast, c_pack = self._generate_stage_candidates(T_nom, rot_a, anchor_tolerances_deg)
            if not c_fast and int(c_pack["q"].shape[0]) == 0:
                raise RuntimeError(
                    f"Waypoint [{i}] has no non-singular in-limit IK inside the attitude envelope."
                )
            stages_fast.append(c_fast)
            stages_full.append(c_pack)

        stages = [list(s) for s in stages_fast]
        t_cands_ms = (time.time() - t_cands_start) * 1000.0
        total_fast = sum(len(s) for s in stages)
        total_full = sum(int(p["q"].shape[0]) for p in stages_full)
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

            is_jump = bool(waypoints[i].get("is_jump", False) or waypoints[i].get("spraying") == "off")

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
                        ok, edge_cost, q_arr = self._check_movel_segment(node_prev, node_curr, q_prev, is_jump=is_jump)
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
            full_n = int(stages_full[i]["q"].shape[0])
            if not np.any(np.isfinite(dp_cost[i])) and full_n > len(stages[i]):
                logger.warning(
                    f"⚠️ [SprayOpt] Segment {i-1}->{i} failed with pruned candidates ({len(stages[i])}). "
                    f"Triggering adaptive fallback with ALL {full_n} candidates..."
                )
                stages[i] = self._materialize_pack(stages_full[i])
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
        if self.verifier is not None and self.dense_verify and n >= 2:
            self._last_dense_report = self._dense_verify_or_raise(
                waypoints,
                best_T,
                q_seed,
                path_id=int(path_id if path_id is not None else 0),
                path_name=str(path_name if path_name is not None else "spray_opt"),
            )
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
            path_id=path_item.get("path_id"),
            path_name=path_item.get("name"),
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

        # 密采样校验与轨迹插值挂载：dense_verify 时 optimize() 已验过，这里复用报告
        if self.verifier is not None and len(new_points) >= 2:
            rep = self._last_dense_report
            if rep is None:
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