# -*- coding: utf-8 -*-
import numpy as np
from scipy.spatial.transform import Rotation as R_tool

def _get_pose_rot(pose):
    """
    Safely extract rotation angles from pose dict, supporting both new format (rx, ry, rz)
    and legacy format (a, b, c).
    """
    rx = pose.get("rx", pose.get("a", 0.0))
    ry = pose.get("ry", pose.get("b", 0.0))
    rz = pose.get("rz", pose.get("c", 0.0))
    return float(rx), float(ry), float(rz)

def clean_calibration_data(all_samples, threshold=0.05, log_callback=None):
    """
    清洗标定采样数据。
    原理：
      在手眼标定采样时，如果机械臂末端（或基座）发生了平移位移 d_r，那么对应的相机观测到的标定板在相机系下的位移 d_c 也应当与之接近。
      比例 ratio = d_c / d_r 理论上应该在 1.0 附近。
      如果比值偏离 1.0 太远，说明可能发生了棋盘格角点检测错误、机械臂读取延迟或深度噪点等异常，应当予以剔除。
      如果比值偏离但同时检测到较大的姿态旋转（orientation change），则允许进入较宽的宽容区间以保留旋转丰富的样本。
    """
    if not all_samples:
        return []
    base = all_samples[0]
    samples = [base]
    if log_callback:
        log_callback(f"  [KEEP] Sample {base['id']}: Reference Base")
    for i in range(1, len(all_samples)):
        s = all_samples[i]
        # 获取第0个参考点与当前点的机器人位置
        p_base = np.array([base["pose"]["x"], base["pose"]["y"], base["pose"]["z"]])
        p_curr = np.array([s["pose"]["x"], s["pose"]["y"], s["pose"]["z"]])
        
        # 计算机器人的实际物理位移和相机的观测位移
        dist_r = np.linalg.norm(p_curr - p_base)
        dist_c = np.linalg.norm(s["t_cb"] - base["t_cb"])
        
        # 如果机器人位移非常小（小于10mm），为了避免微小分母导致的比例异常，直接予以保留
        if dist_r < 10.0:
            samples.append(s)
            continue
            
        ratio = dist_c / dist_r
        # 判断位移比率是否在设定的阈值区间内
        if (1.0 - threshold) < ratio < (1.0 + threshold):
            samples.append(s)
            if log_callback:
                log_callback(f"  [KEEP] Sample {s['id']}: Displacement ratio = {ratio:.3f}")
        else:
            # 检查姿态是否发生了显著旋转（欧拉角差异评估）
            r_base = np.array(_get_pose_rot(base["pose"]))
            r_curr = np.array(_get_pose_rot(s["pose"]))
            rot_diff = np.linalg.norm(r_curr - r_base)
            
            if rot_diff > 0.05:  # 旋转差异大于 ~3 度
                # 如果检测到了显著的姿态改变，适当放宽比率范围（0.80 - 1.20）以防止误伤宝贵的角度变化样本
                if 0.80 < ratio < 1.20:
                    samples.append(s)
                    if log_callback:
                        log_callback(f"  [KEEP*] Sample {s['id']}: Displacement ratio = {ratio:.3f} (orientation change detected)")
                else:
                    if log_callback:
                        log_callback(f"  [DROP] Sample {s['id']}: Displacement ratio = {ratio:.3f} (excessive deviation)")
            else:
                if log_callback:
                    log_callback(f"  [DROP] Sample {s['id']}: Displacement ratio = {ratio:.3f}")
    return samples

def evaluate_data_diversity(samples):
    """
    评估采样点在空间位置和旋转姿态上的多样性得分。
    原理：
      手眼标定要求样本点在 X, Y, Z 三个轴向上有足够的跨度，且 A, B, C 旋转角度有足够的变化。
      本函数计算坐标的峰峰值 (Peak-to-Peak, ptp)，并映射为 0~100 的多样性评分，用于提示用户是否需要补充更多角度和位置的样本。
    """
    if not samples:
        return {"score": 0.0}
    pos_x = [s["pose"]["x"] for s in samples]
    pos_y = [s["pose"]["y"] for s in samples]
    pos_z = [s["pose"]["z"] for s in samples]
    rot_a = [s["pose"].get("rx", s["pose"].get("a", 0.0)) for s in samples]
    rot_b = [s["pose"].get("ry", s["pose"].get("b", 0.0)) for s in samples]
    rot_c = [s["pose"].get("rz", s["pose"].get("c", 0.0)) for s in samples]

    # 判断旋转欧拉角输入是弧度还是角度
    is_radians = np.max(np.abs([rot_a, rot_b, rot_c])) < 7.0
    if is_radians:
        rot_a = np.degrees(rot_a)
        rot_b = np.degrees(rot_b)
        rot_c = np.degrees(rot_c)

    # 计算平移与旋转在三个轴向上的跨度（峰峰值）
    ptp_xyz = [np.ptp(pos_x), np.ptp(pos_y), np.ptp(pos_z)]
    ptp_abc = [np.ptp(rot_a), np.ptp(rot_b), np.ptp(rot_c)]

    # 设定位移平均跨度达到 300mm 为满分，旋转平均跨度达到 30度 为满分
    p_score = min(1.0, np.mean(ptp_xyz) / 300.0)
    r_score = min(1.0, np.mean(ptp_abc) / 30.0)

    # 综合权重：平移占 40%，旋转占 60%（因为手眼标定对姿态旋转的多样性更为敏感）
    score = (p_score * 0.4 + r_score * 0.6) * 100.0
    return {"score": score}

def optimize_extrinsics_solve(samples):
    """
    标定外参交替优化求解器。
    原理与步骤：
      1. 机械臂厂商的欧拉角顺规（如 ZYX, XYZ 等）以及极性正负号（Sign vector）可能与软件默认不一致。
         本求解器会对 12 种常见顺规和 8 种正负符号排列进行全网格搜索，以最小化重投影残差。
      2. 标定数学模型：
         对 Eye-to-Hand（眼在手外）模型，关系式为:
            P_board_base = P_robot + R_bt * t_off
         其中：
            - P_board_base 是标定板坐标在机器人基座坐标系下的位置（恒定值）。
            - P_robot 是当前机械臂末端位置。
            - R_bt 是机械臂末端的姿态旋转矩阵。
            - t_off 是标定板中心相对于机器人末端的 TCP 偏移。
         同时，通过相机外参 T_base_camera (由 R_base_cam 和 t_base_cam 组成)：
            P_board_base = R_base_cam @ P_cam + t_base_cam
         联立以上公式，通过交替迭代优化（Alternating Optimization）求解未知量 R_base_cam, t_base_cam 和 t_off：
            - 固定 t_off，通过点云对齐（SVD/Kabsch 算法）求解相机系与基座系之间的旋转 R_base_cam (或者说是 R_cam_base)。
            - 固定旋转矩阵，通过最小二乘法求解平移向量 t_base_cam 与标定板 TCP 偏移 t_off。
      3. 迭代至收敛后，评估重投影残差，挑选重投影残差最小的顺规组合作为最终外参。
    """
    best_err = float('inf')
    best_res = None
    
    # 穷举测试 12 种欧拉角旋转顺规
    test_orders = ['ZYX', 'XYZ', 'YXZ', 'YZX', 'ZXY', 'XZY', 'zyx', 'xyz', 'yxz', 'yzx', 'zxy', 'xzy']
    # 穷举测试 8 种轴方向符号映射组合
    signs = [(1,1,1), (1,1,-1), (1,-1,1), (-1,1,1), (1,-1,-1), (-1,1,-1), (-1,-1,1), (-1,-1,-1)]

    for order in test_orders:
        for s_vec in signs:
            t_off = np.zeros(3)
            R_base_cam = np.eye(3)
            t_base_cam = np.zeros(3)
            
            P_robot = np.array([[s["pose"]["x"], s["pose"]["y"], s["pose"]["z"]] for s in samples])
            P_cam = np.array([s["t_cb"] for s in samples])
            
            try:
                # 转换机器人欧拉角至旋转矩阵列表
                R_bt_list = []
                for s in samples:
                    rx, ry, rz = _get_pose_rot(s["pose"])
                    euler_angles = [rx * s_vec[0], ry * s_vec[1], rz * s_vec[2]]
                    R_bt_list.append(R_tool.from_euler(order, euler_angles).as_matrix())
            except:
                continue

            try:
                # 交替最优化迭代（循环15次通常足够收敛）
                for _ in range(15):
                    # 1. 估算标定板在机器人基座坐标系下的估算位置
                    P_board_base = P_robot + np.array([R @ t_off for R in R_bt_list])
                    cA = np.mean(P_board_base, axis=0)
                    cB = np.mean(P_cam, axis=0)
                    
                    # 2. Kabsch 算法 (基于奇异值分解 SVD 的三维点云配准)
                    H = (P_board_base - cA).T @ (P_cam - cB)
                    U, S, Vt = np.linalg.svd(H)
                    R_cam_base = Vt.T @ U.T
                    # 保证旋转矩阵的正交性与行列式为 +1（防止产生镜像反射矩阵）
                    if np.linalg.det(R_cam_base) < 0:
                        Vt[2, :] *= -1
                        R_cam_base = Vt.T @ U.T
                    R_base_cam = R_cam_base.T
                    
                    # 3. 构造超定线性方程组 Ax = B 求解平移向量 (t_base_cam 和 t_off)
                    A_mat = []
                    B_mat = []
                    for i in range(len(samples)):
                        # 方程：t_base_cam - R_bt_list[i] * t_off = P_robot[i] - R_base_cam * P_cam[i]
                        A_mat.append(np.hstack([np.eye(3), -R_bt_list[i]]))
                        B_mat.append(P_robot[i] - R_base_cam @ P_cam[i])
                    
                    res_lstsq, _, _, _ = np.linalg.lstsq(np.vstack(A_mat), np.hstack(B_mat), rcond=None)
                    t_base_cam = res_lstsq[:3]
                    t_off = res_lstsq[3:]
                
                # 过滤不符合物理常识的大尺度偏移量（如 TCP 偏移超过500mm，判断为解算发散或局域解）
                if np.linalg.norm(t_off) > 500.0:
                    continue

                # 4. 计算该顺规/极性组合下的平均重投影平移误差
                t_errs = []
                for i in range(len(samples)):
                    p_board_base = P_robot[i] + R_bt_list[i] @ t_off
                    p_cam_pred = R_cam_base @ (p_board_base - t_base_cam)
                    t_errs.append(np.linalg.norm(p_cam_pred - P_cam[i]))
                
                mean_err = np.mean(t_errs)
                # 保留误差最小的最优排列参数
                if mean_err < best_err:
                    best_err = mean_err
                    best_res = (R_base_cam, t_base_cam, t_off, mean_err, order, s_vec)
            except:
                continue
    return best_res

def calculate_rotation_error(samples, best_res):
    """
    计算多次采样之间的平均旋转残差。
    原理：
      对于正确的标定结果，任意两个样本点算出的“末端到标定板板面（Tool-to-Target）”的相对姿态 T_tt 应该保持不变。
      公式: T_tt = T_tool_base * T_base_camera * T_camera_board
      我们计算所有样本相对姿态的平均矩阵，然后计算每个样本到平均姿态的角度偏离偏差，其均值即为旋转标定精度（度）。
    """
    R_bc, t_bc, t_off, err, order, s_vec = best_res
    r_errs = []
    T_tt_list = []
    
    # 1. 计算每个采样点下的末端到标定板的齐次变换矩阵
    for s in samples:
        p = s["pose"]
        rx, ry, rz = _get_pose_rot(p)
        euler_angles = [rx * s_vec[0], ry * s_vec[1], rz * s_vec[2]]
        R_bt = R_tool.from_euler(order, euler_angles).as_matrix()
        
        T_bt = np.eye(4)
        T_bt[:3, :3] = R_bt
        T_bt[:3, 3] = [p['x'], p['y'], p['z']]
        
        T_cb = np.eye(4)
        T_cb[:3, :3] = s["R_cb"]
        T_cb[:3, 3] = s["t_cb"]
        
        T_bc_mat = np.eye(4)
        T_bc_mat[:3, :3] = R_bc
        T_bc_mat[:3, 3] = t_bc
        
        # T_tt = T_tb @ T_bc @ T_cb = T_bt^-1 @ T_bc @ T_cb
        T_tt = np.linalg.inv(T_bt) @ T_bc_mat @ T_cb
        T_tt_list.append(T_tt)
    
    # 2. 求旋转矩阵的算术平均并重投影回 SO(3) 空间（通过 SVD 正交化消除漂移）
    avg_T_tt = np.mean(T_tt_list, axis=0)
    U, _, Vt = np.linalg.svd(avg_T_tt[:3, :3])
    avg_T_tt[:3, :3] = U @ Vt
    
    # 3. 计算每个样板与平均姿态的角度差异 (角位移 residue)
    for T in T_tt_list:
        r_diff = T[:3, :3].T @ avg_T_tt[:3, :3]
        # 通过旋转矩阵的迹(trace)还原旋转角：trace = 1 + 2*cos(theta)
        trace_val = (np.trace(r_diff) - 1.0) / 2.0
        angle_val = np.arccos(np.clip(trace_val, -1.0, 1.0))
        r_errs.append(np.degrees(angle_val))
        
    return np.mean(r_errs)
