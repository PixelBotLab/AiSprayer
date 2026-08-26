#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, cv2, yaml, numpy as np, argparse, glob, sys, time
from math import cos, sin, acos, degrees
from datetime import datetime
from scipy.spatial.transform import Rotation as R_tool

# 1. 路径锚定策略 (无论从哪个目录执行，都能准确找到项目根目录)
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.append(os.path.join(PROJECT_ROOT, "src"))
sys.path.append(os.path.join(PROJECT_ROOT, "app/src"))

from aisprayer.utils.config_helper import load_config as load_global_config, get_abs_path

def parse_args():
    parser = argparse.ArgumentParser(description="标定计算工具 (生产版)")
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs/aisprayer_config.yaml"), help="配置文件路径")
    parser.add_argument("--data", help="指定待处理的采集目录路径 (如 data/calib/calib_20260505)。若不指定则自动使用最新目录。")
    return parser.parse_args()

def _get_pose_rot(pose):
    """Safely extract rotation angles from pose dict, supporting rx/ry/rz and a/b/c."""
    rx = pose.get("rx", pose.get("a", 0.0))
    ry = pose.get("ry", pose.get("b", 0.0))
    rz = pose.get("rz", pose.get("c", 0.0))
    return float(rx), float(ry), float(rz)

def create_rotation_matrix(a, b, c, order, s=(1,1,1)):
    return R_tool.from_euler(order, [a*s[0], b*s[1], c*s[2]]).as_matrix()

def load_calib_info(data_dir, camera_model=None):
    """加载采集目录下的标定元数据并校验一致性"""
    info_path = os.path.join(data_dir, "calibration_info.yaml")
    if not os.path.exists(info_path):
        raise FileNotFoundError(f"找不到配置: {info_path}")
    
    with open(info_path, 'r') as f: 
        info = yaml.safe_load(f)
    
    # Verify camera model consistency
    if camera_model:
        data_cam = info.get("camera_params", {}).get("camera_model")
        if data_cam != camera_model:
            print(f"\n[WARNING] Dataset camera model ({data_cam}) differs from config ({camera_model}), using dataset parameters.")
            
    return info

def extract_corners_and_pnp(data_dir, config):
    """Extract chessboard corners and compute PnP for all samples"""
    info = config
    K = np.array(info["camera_params"]["intrinsic_matrix"])
    D = np.array(info["camera_params"]["distortion_coeffs"])
    
    print(f"[*] Processing calibration dataset: {data_dir}")
    
    # 2. Board parameters
    pattern_size = tuple(config["board_params"]["pattern_size_inner"])
    sq_size = config["board_params"]["square_size_mm"]

    print(f"[*] Extracting chessboard corners...")
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * sq_size
    
    all_samples = []
    fail_count = 0
    first_img_size = None
    for s in config["samples"]:
        pose = s.get("robot_pose", {})
        if any(abs(v) > 2500 for v in [pose.get('x',0), pose.get('y',0), pose.get('z',0)]):
            print(f"  [!] Ignored invalid pose sample: #{s['id']}")
            continue

        img_file = s.get("image_file", "unknown")
        img_path = os.path.join(data_dir, img_file)
        img = cv2.imread(img_path)
        if img is None:
            print(f"  [!] Failed to read image: {img_file}")
            fail_count += 1
            continue
        
        # 记录图像分辨率（如果配置中没有的话）
        if first_img_size is None:
            h, w = img.shape[:2]
            first_img_size = (w, h)
            if "width" not in config["camera_params"]:
                config["camera_params"]["width"] = w
                config["camera_params"]["height"] = h
                print(f"[*] Detected calibration resolution: {w}x{h}")

        ret, corners = cv2.findChessboardCorners(img, pattern_size, None)
        if ret:
            # Subpixel refinement
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)
            
            _, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
            R_cb, _ = cv2.Rodrigues(rvec)
            all_samples.append({"id": s["id"], "R_cb": R_cb, "t_cb": tvec.flatten(), "pose": s["robot_pose"]})
        else:
            print(f"  [!] Corner extraction failed: {img_file}")
            fail_count += 1
            
    print(f"[*] Corner extraction complete: {len(all_samples)} succeeded, {fail_count} failed")
    return all_samples

def clean_data(all_samples, threshold=0.05):
    """Filter outlier samples based on displacement ratio"""
    if not all_samples: return []
    base = all_samples[0]
    samples = [base]
    print(f"[*] Cleaning samples (Threshold: {threshold*100:.1f}%)...")
    print(f"  [KEEP] Sample {base['id']}: Reference base point")
    for i in range(1, len(all_samples)):
        s = all_samples[i]
        p_base = np.array([base["pose"]["x"], base["pose"]["y"], base["pose"]["z"]])
        p_curr = np.array([s["pose"]["x"], s["pose"]["y"], s["pose"]["z"]])
        dist_r = np.linalg.norm(p_curr - p_base)
        dist_c = np.linalg.norm(s["t_cb"] - base["t_cb"])
        
        # If robot barely moved (<10mm), keep sample
        if dist_r < 10:
            samples.append(s)
            continue
            
        ratio = dist_c / dist_r
        if (1.0 - threshold) < ratio < (1.0 + threshold):
            samples.append(s)
            print(f"  [KEEP] Sample {s['id']}: Displacement ratio = {ratio:.3f}")
        else:
            # Check for significant orientation change (>0.05 rad)
            r_base = np.array(_get_pose_rot(base["pose"]))
            r_curr = np.array(_get_pose_rot(s["pose"]))
            rot_diff = np.linalg.norm(r_curr - r_base)
            
            if rot_diff > 0.05: # > 3 deg rotation
                if 0.80 < ratio < 1.20:
                    samples.append(s)
                    print(f"  [KEEP*] Sample {s['id']}: Displacement ratio = {ratio:.3f} (orientation change detected)")
                else:
                    print(f"  [DROP] Sample {s['id']}: Displacement ratio = {ratio:.3f} (excessive deviation with rotation)")
            else:
                print(f"  [DROP] Sample {s['id']}: Displacement ratio = {ratio:.3f} (deviation without rotation)")
            
    print(f"[*] High-quality samples kept for calculation: {len(samples)} / {len(all_samples)}")
    return samples

def evaluate_diversity(samples):
    """评估样本在空间和姿态上的多样性"""
    if not samples: return {}
    
    pos_x = [s["pose"]["x"] for s in samples]
    pos_y = [s["pose"]["y"] for s in samples]
    pos_z = [s["pose"]["z"] for s in samples]
    rot_a = [s["pose"].get("rx", s["pose"].get("a", 0.0)) for s in samples]
    rot_b = [s["pose"].get("ry", s["pose"].get("b", 0.0)) for s in samples]
    rot_c = [s["pose"].get("rz", s["pose"].get("c", 0.0)) for s in samples]
    
    # 检查是否为弧度 (如果最大值很小，极有可能是弧度)
    is_radians = np.max(np.abs([rot_a, rot_b, rot_c])) < 7.0
    
    if is_radians:
        rot_a = np.degrees(rot_a)
        rot_b = np.degrees(rot_b)
        rot_c = np.degrees(rot_c)
    
    report = {
        "position_range_mm": {
            "x": float(np.ptp(pos_x)), "y": float(np.ptp(pos_y)), "z": float(np.ptp(pos_z))
        },
        "rotation_range_deg": {
            "a": float(np.ptp(rot_a)), "b": float(np.ptp(rot_b)), "c": float(np.ptp(rot_c)),
            "rx": float(np.ptp(rot_a)), "ry": float(np.ptp(rot_b)), "rz": float(np.ptp(rot_c))
        },
        "score": 0.0
    }
    
    # 计算一个简单的多样性得分 (0-100)
    # 平移理想跨度 > 300mm, 旋转理想跨度 > 30 deg
    p_score = min(1.0, np.mean([report["position_range_mm"][k] for k in 'xyz']) / 300.0)
    r_score = min(1.0, np.mean([report["rotation_range_deg"][k] for k in 'abc']) / 30.0)
    
    report["p_score"] = float(p_score * 100)
    report["r_score"] = float(r_score * 100)
    report["score"] = float((p_score * 0.4 + r_score * 0.6) * 100)
    
    return report

def optimize_extrinsics(samples):
    """全排列搜索与交替优化，求解外参和偏置"""
    best_err = float('inf')
    best_res = None
    
    test_orders = ['ZYX', 'XYZ', 'YXZ', 'YZX', 'ZXY', 'XZY', 'zyx', 'xyz', 'yxz', 'yzx', 'zxy', 'xzy']
    signs = [(1,1,1), (1,1,-1), (1,-1,1), (-1,1,1), (-1,1,-1), (-1,1,-1), (-1,-1,1), (-1,-1,-1)]

    print(f"[*] Running exhaustive grid optimization (Extrinsics & Hand-eye Offset)...")
    for order in test_orders:
        for s_vec in signs:
            t_off = np.zeros(3)
            R_base_cam = np.eye(3)
            t_base_cam = np.zeros(3)
            
            P_robot = np.array([[s["pose"]["x"], s["pose"]["y"], s["pose"]["z"]] for s in samples])
            P_cam = np.array([s["t_cb"] for s in samples])
            R_bt_list = [
                create_rotation_matrix(
                    s["pose"].get("rx", s["pose"].get("a", 0.0)),
                    s["pose"].get("ry", s["pose"].get("b", 0.0)),
                    s["pose"].get("rz", s["pose"].get("c", 0.0)),
                    order, s_vec
                ) for s in samples
            ]
            
            try:
                for _ in range(15): # 增加迭代次数
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
                
                # 物理合理性约束：标定板偏移通常不应超过 0.5 米 (针对工业机器人的常规手柄长度)
                if np.linalg.norm(t_off) > 500:
                    print(f"  [DROP] Sample {s['id']}: 标定板偏移过大，不予计算")
                    continue

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
        rx, ry, rz = _get_pose_rot(p)
        R_bt = create_rotation_matrix(rx, ry, rz, order, s_vec)
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

def save_and_print_results(data_dir, best_res, r_err_mean, camera_params, board_params, output_path, sample_stats, diversity):
    """Print detailed results and save full configuration to YAML file in English"""
    R_bc, t_bc, t_off, err, order, s_vec = best_res
    T_bc = np.eye(4); T_bc[:3,:3], T_bc[:3,3] = R_bc, t_bc
    
    # Results path
    output_res_path = get_abs_path(output_path, PROJECT_ROOT)
    os.makedirs(os.path.dirname(output_res_path), exist_ok=True)
    
    # Readable pose (XYZ + RPY degrees)
    xyz = T_bc[:3, 3].tolist()
    rpy = R_tool.from_matrix(T_bc[:3, :3]).as_euler('xyz', degrees=True).tolist()

    print("\n" + "="*60)
    print(f"  CALIBRATION SUCCESSFUL! Config: Euler={order}, Signs={s_vec}")
    print(f"  - Reprojection Error (Residual): {err:.4f} mm")
    print(f"  - Mean Angular Error: {r_err_mean:.4f} deg")
    print(f"  - Chessboard Offset (Offset): [{t_off[0]:.3f}, {t_off[1]:.3f}, {t_off[2]:.3f}] mm")
    print(f"  - Camera Pose (Base): X={xyz[0]:.2f}, Y={xyz[1]:.2f}, Z={xyz[2]:.2f} mm | Roll={rpy[0]:.2f}°, Pitch={rpy[1]:.2f}°, Yaw={rpy[2]:.2f}°")
    print("="*60)

    # Full output data dict
    output_data = {
        "metadata": {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "source_data_dir": data_dir,
            "calibration_mode": "eye-to-hand",
            "reprojection_error_mm": float(err),
            "rotation_error_deg": float(r_err_mean),
            "samples_total": sample_stats[0],
            "samples_used": sample_stats[1],
            "data_diversity": diversity,
            "optimization_config": {
                "axis_order": order,
                "sign_vector": s_vec.tolist() if hasattr(s_vec, 'tolist') else list(s_vec)
            },
            "optimized_at": time.time()
        },
        "camera_pose_base": {
            "x": xyz[0], "y": xyz[1], "z": xyz[2],
            "roll_deg": rpy[0], "pitch_deg": rpy[1], "yaw_deg": rpy[2]
        },
        "T_base_camera": T_bc.tolist(), 
        "camera_params": camera_params,
        "board_params": board_params,
        "chessboard_offset": t_off.tolist(),
    }

    with open(output_res_path, 'w') as f:
        yaml.dump(output_data, f, default_flow_style=False)
    print(f"[+] Calibration results saved to: {output_res_path}\n")

def main():
    args = parse_args()
    
    # 1. 加载全局配置
    full_cfg = load_global_config(args.config, PROJECT_ROOT)
    c_cfg = full_cfg.get("calib", {})
    camera_model = full_cfg.get("hardware", {}).get("camera", {}).get("model", "orbbec")
    
    # 2. Resolve dataset directory
    if args.data:
        target_dir = get_abs_path(args.data, PROJECT_ROOT)
        if not os.path.exists(target_dir):
            print(f"[-] Specified directory does not exist: {target_dir}")
            return
        print(f"[*] Using specified dataset directory: {target_dir}")
    else:
        data_root = get_abs_path(c_cfg.get("capture", {}).get("output_dir", "data/calib"), PROJECT_ROOT)
        dirs = sorted(glob.glob(os.path.join(data_root, "calib_*")))
        if not dirs:
            print(f"[-] No calibration datasets found in: {data_root}")
            return
        target_dir = dirs[-1]
        print(f"[*] Auto-selected latest dataset directory: {target_dir}")
    
    # 3. Load metadata
    try:
        info = load_calib_info(target_dir, camera_model)
    except Exception as e:
        print(f"[-] Failed to load calibration info: {e}")
        return
    
    # 4. Extract corners and solve PnP offline
    all_samples = extract_corners_and_pnp(target_dir, info)
    if not all_samples:
        print("[-] Failed to extract corners from any samples. Calibration aborted.")
        return

    # 5. Data cleaning (based on physical displacement ratio)
    clean_thr = c_cfg.get("cleaning_threshold", 0.05)
    samples = clean_data(all_samples, threshold=clean_thr)
    if len(samples) < 3:
        print("[-] Insufficient valid samples for calibration after filtering.")
        return

    # 6. Evaluate data diversity
    diversity = evaluate_diversity(samples)
    print(f"[*] Data Diversity Score: {diversity['score']:.1f} / 100")
    if diversity['score'] < 60:
        if diversity['p_score'] < 50:
            print("    [!] Warning: Spatial translation span is small. Consider moving the robot across a larger area.")
        if diversity['r_score'] < 50:
            print("    [!] Warning: Orientation rotation span is small. Consider varying TCP tilt/roll angles.")

    # 7. Alternating grid optimization
    best_res = optimize_extrinsics(samples)
    if not best_res:
        print("[-] Calibration solver failed to find a valid solution.")
        return

    # 8. Compute rotation error
    r_err_mean = calculate_rotation_error(samples, best_res)

    # 9. Print and save results
    calib_out = c_cfg.get("result_path", "configs/calib/calibration_result.yaml")
    save_and_print_results(
        target_dir, best_res, r_err_mean, 
        info["camera_params"], info["board_params"],
        calib_out,
        (len(all_samples), len(samples)),
        diversity
    )

if __name__ == "__main__": 
    main()
