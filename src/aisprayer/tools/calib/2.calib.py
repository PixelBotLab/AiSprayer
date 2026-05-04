#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, cv2, yaml, numpy as np, argparse, glob, sys
from math import cos, sin, acos, degrees
from datetime import datetime
from scipy.spatial.transform import Rotation as R_tool

# 1. 路径锚定策略 (无论从哪个目录执行，都能准确找到项目根目录)
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.hardware.camera.factory import get_camera
from aisprayer.utils.config_helper import load_config as load_global_config, get_abs_path

def parse_args():
    parser = argparse.ArgumentParser(description="标定计算工具 (生产版)")
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs/aisprayer_config.yaml"), help="配置文件路径")
    return parser.parse_args()

def create_rotation_matrix(a, b, c, order, s=(1,1,1)):
    return R_tool.from_euler(order, [a*s[0], b*s[1], c*s[2]]).as_matrix()

def load_calib_info(data_dir):
    """加载采集目录下的标定元数据"""
    info_path = os.path.join(data_dir, "calibration_info.yaml")
    if not os.path.exists(info_path):
        raise FileNotFoundError(f"找不到配置: {info_path}")
    with open(info_path, 'r') as f: 
        return yaml.safe_load(f)

def extract_corners_and_pnp(data_dir, config, camera_model):
    """提取角点并解算 PnP，返回所有样本"""
    info = config
    # 校验型号一致性 (与全局配置对比)
    data_cam = info.get("camera_params", {}).get("camera_model")
    if data_cam != camera_model:
        print(f"\n[WARNING] 采集数据相机型号 ({data_cam}) 与当前配置 ({camera_model}) 不一致，将使用采集数据中的参数进行计算。")
    
    # 直接使用采集数据中的内参进行 PnP
    K = np.array(info["camera_params"]["intrinsic_matrix"])
    D = np.array(info["camera_params"]["distortion_coeffs"])
    
    print(f"[*] 正在处理标定数据: {data_dir}")
    
    # 4. 执行标定计算
    # K 和 D 已经在函数开头从 info 中加载
    pattern_size = tuple(config["board_params"]["pattern_size_inner"])
    sq_size = config["board_params"]["square_size_mm"]

    all_samples = []
    print(f"[*] 正在提取角点...")
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * sq_size
    
    first_img_size = None
    for s in config["samples"]:
        img_path = os.path.join(data_dir, s["image_file"])
        img = cv2.imread(img_path)
        if img is None: continue
        
        # 记录图像分辨率（如果配置中没有的话）
        if first_img_size is None:
            h, w = img.shape[:2]
            first_img_size = (w, h)
            if "width" not in config["camera_params"]:
                config["camera_params"]["width"] = w
                config["camera_params"]["height"] = h
                print(f"[*] 自动检测到标定分辨率: {w}x{h}")

        ret, corners = cv2.findChessboardCorners(img, pattern_size, None)
        if ret:
            # 亚像素级精细化
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)
            
            _, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
            R_cb, _ = cv2.Rodrigues(rvec)
            all_samples.append({"id": s["id"], "R_cb": R_cb, "t_cb": tvec.flatten(), "pose": s["robot_pose"]})
            
    return all_samples

def clean_data(all_samples):
    """根据位移比例清洗数据，剔除畸变或失焦坏点"""
    if not all_samples: return []
    base = all_samples[0]
    samples = [base]
    print(f"[*] 正在清洗数据 (基于物理位移比例分析)...")
    print(f"  [KEEP] Sample {base['id']}: 基准点")
    for i in range(1, len(all_samples)):
        s = all_samples[i]
        p_base = np.array([base["pose"]["x"], base["pose"]["y"], base["pose"]["z"]])
        p_curr = np.array([s["pose"]["x"], s["pose"]["y"], s["pose"]["z"]])
        dist_r = np.linalg.norm(p_curr - p_base)
        dist_c = np.linalg.norm(s["t_cb"] - base["t_cb"])
        ratio = dist_c / dist_r if dist_r > 5 else 1.0
        
        if 0.95 < ratio < 1.05:
            samples.append(s)
            print(f"  [KEEP] Sample {s['id']}: 位移比例 = {ratio:.3f}")
        else:
            print(f"  [DROP] Sample {s['id']}: 位移比例 = {ratio:.3f} (偏差过大剔除)")
            
    print(f"[*] 最终参与计算的优质样本数: {len(samples)} / {len(all_samples)}")
    return samples

def optimize_extrinsics(samples):
    """全排列搜索与交替优化，求解外参和偏置"""
    best_err = float('inf')
    best_res = None
    
    test_orders = ['ZYX', 'XYZ', 'YXZ', 'YZX', 'ZXY', 'XZY', 'zyx', 'xyz', 'yxz', 'yzx', 'zxy', 'xzy']
    signs = [(1,1,1), (1,1,-1), (1,-1,1), (-1,1,1), (1,-1,-1), (-1,1,-1), (-1,-1,1), (-1,-1,-1)]

    print(f"[*] 正在执行全排列联合交替优化 (包含标定板偏置与旋转搜索)...")
    for order in test_orders:
        for s_vec in signs:
            t_off = np.zeros(3)
            R_base_cam = np.eye(3)
            t_base_cam = np.zeros(3)
            
            P_robot = np.array([[s["pose"]["x"], s["pose"]["y"], s["pose"]["z"]] for s in samples])
            P_cam = np.array([s["t_cb"] for s in samples])
            R_bt_list = [create_rotation_matrix(s["pose"]['a'], s["pose"]['b'], s["pose"]['c'], order, s_vec) for s in samples]
            
            try:
                for _ in range(10): # 交替迭代10次
                    P_board_base = P_robot + np.array([R @ t_off for R in R_bt_list])
                    
                    cA, cB = np.mean(P_board_base, axis=0), np.mean(P_cam, axis=0)
                    H = (P_board_base - cA).T @ (P_cam - cB)
                    U, S, Vt = np.linalg.svd(H)
                    R_cam_base = Vt.T @ U.T
                    if np.linalg.det(R_cam_base) < 0:
                        Vt[2,:] *= -1
                        R_cam_base = Vt.T @ U.T
                    R_base_cam = R_cam_base.T
                    
                    A_mat, B_mat = [], []
                    for i in range(len(samples)):
                        A_mat.append(np.hstack([np.eye(3), -R_bt_list[i]]))
                        B_mat.append(P_robot[i] - R_base_cam @ P_cam[i])
                    
                    res, _, _, _ = np.linalg.lstsq(np.vstack(A_mat), np.hstack(B_mat), rcond=None)
                    t_base_cam, t_off = res[:3], res[3:]
                
                t_errs = []
                for i in range(len(samples)):
                    p_board_base = P_robot[i] + R_bt_list[i] @ t_off
                    p_cam_pred = R_cam_base @ (p_board_base - t_base_cam)
                    t_errs.append(np.linalg.norm(p_cam_pred - P_cam[i]))
                
                m_err = np.mean(t_errs)
                if m_err < best_err:
                    best_err = m_err
                    best_res = (R_base_cam, t_base_cam, t_off, m_err, order, s_vec)
            except: continue
            
    return best_res

def calculate_rotation_error(samples, best_res):
    """计算各样本间的平均旋转残差"""
    R_bc, t_bc, t_off, err, order, s_vec = best_res
    r_errs = []
    T_tt_list = []
    for s in samples:
        p = s["pose"]
        R_bt = create_rotation_matrix(p['a'], p['b'], p['c'], order, s_vec)
        T_bt = np.eye(4); T_bt[:3,:3], T_bt[:3,3] = R_bt, [p['x'], p['y'], p['z']]
        T_cb = np.eye(4); T_cb[:3,:3], T_cb[:3,3] = s["R_cb"], s["t_cb"]
        T_bc_mat = np.eye(4); T_bc_mat[:3,:3], T_bc_mat[:3,3] = R_bc, t_bc
        T_tt = np.linalg.inv(T_bt) @ T_bc_mat @ T_cb
        T_tt_list.append(T_tt)
    
    avg_T_tt = np.mean(T_tt_list, axis=0)
    U, _, Vt = np.linalg.svd(avg_T_tt[:3,:3]); avg_T_tt[:3,:3] = U @ Vt
    for T in T_tt_list:
        r_diff = T[:3,:3].T @ avg_T_tt[:3,:3]
        r_errs.append(degrees(acos(np.clip((np.trace(r_diff)-1)/2, -1.0, 1.0))))
        
    return np.mean(r_errs)

def save_and_print_results(data_dir, best_res, r_err_mean, camera_params, board_params):
    """打印详细结果并保存全量配置至 yaml 文件"""
    R_bc, t_bc, t_off, err, order, s_vec = best_res
    T_bc = np.eye(4); T_bc[:3,:3], T_bc[:3,3] = R_bc, t_bc
    
    # 结果保存路径
    output_res_path = os.path.join(PROJECT_ROOT, "configs/calib/calibration_result.yaml")
    os.makedirs(os.path.dirname(output_res_path), exist_ok=True)
    
    print("\n" + "="*50)
    print(f"标定成功！配置: {order}, Signs: {s_vec}")
    print(f"平均平移误差 (Residual): {err:.3f} mm")
    print(f"平均旋转误差 (Angular): {r_err_mean:.3f} °")
    print(f"标定板偏移 (Offset): {t_off} mm")
    
    # 构造全量结果字典 (确保 camera_model 存在)
    if "camera_model" not in camera_params or camera_params["camera_model"] is None:
        # 从当前环境尝试获取，或者标记为 unknown
        camera_params["camera_model"] = "orbbec" 

    output_data = {
        "T_base_camera": T_bc.tolist(), 
        "t_offset": t_off.tolist(), 
        "error_mm": float(err), 
        "error_deg": float(r_err_mean),
        "camera_params": camera_params,
        "board_params": board_params
    }

    with open(output_res_path, 'w') as f:
        yaml.dump(output_data, f, default_flow_style=False)
    print(f"\n[+] 标定结果已保存至: {output_res_path}")

def main():
    args = parse_args()
    
    # 1. 加载全局配置
    full_cfg = load_global_config(args.config, PROJECT_ROOT)
    c_cfg = full_cfg.get("calib", {})
    camera_model = full_cfg.get("hardware", {}).get("camera", {}).get("model", "orbbec")
    
    # 2. 自动定位最新的采集目录
    data_root = get_abs_path(c_cfg.get("capture", {}).get("output_dir", "data/calib"), PROJECT_ROOT)
    dirs = sorted(glob.glob(os.path.join(data_root, "calib_*")))
    if not dirs:
        print(f"[-] 找不到标定数据目录: {data_root}")
        return
    
    latest_dir = dirs[-1]
    print(f"[*] 自动锁定最新采集目录: {latest_dir}")

    # 3. 加载该目录下的标定元数据
    try:
        info = load_calib_info(latest_dir)
    except Exception as e:
        print(f"[-] 加载标定信息失败: {e}")
        return
    
    # 3. 提取角点与 PnP (直接离线处理采集数据)
    all_samples = extract_corners_and_pnp(latest_dir, info, camera_model)
    if not all_samples:
        print("[-] 未能提取任何样本的角点，标定失败")
        return

    # 2. 数据清洗
    samples = clean_data(all_samples)
    if len(samples) < 3:
        print("[-] 有效样本数量不足，无法进行标定")
        return

    # 3. 联合交替优化解算
    best_res = optimize_extrinsics(samples)
    if not best_res:
        print("[-] 标定求解失败")
        return

    # 4. 计算旋转误差
    r_err_mean = calculate_rotation_error(samples, best_res)

    # 7. 打印并保存结果 (信任并透传采集数据中的相机参数)
    save_and_print_results(
        latest_dir, best_res, r_err_mean, 
        info["camera_params"], info["board_params"]
    )

if __name__ == "__main__": 
    main()
