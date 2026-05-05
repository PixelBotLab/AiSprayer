import os, cv2, yaml, numpy as np, time, sys
import argparse
import glob

# 1. 路径锚定策略
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.hardware.camera.factory import get_camera
from scipy.spatial.transform import Rotation as R_tool
from aisprayer.utils.config_helper import load_config, get_abs_path
from aisprayer.utils.hardware_helper import verify_hardware_consistency

def get_latest_calib_file(config_dict):
    c_cfg = config_dict.get("calib", {})
    p_cfg = config_dict.get("vision", {}).get("planner", {})
    
    # 1. 优先尝试配置中的正式路径 (只关注 calib 自身的配置或默认路径)
    raw_path = c_cfg.get("result_path", "configs/calib/calibration_result.yaml")
    official_path = get_abs_path(raw_path, PROJECT_ROOT)
    if official_path and os.path.exists(official_path):
        return official_path
        
    # 2. 如果没有正式文件，回退到数据目录找最新的
    data_dir = get_abs_path(c_cfg.get("capture", {}).get("output_dir", "data/calib"), PROJECT_ROOT)
    dirs = glob.glob(os.path.join(data_dir, "calib_*"))
    if not dirs: return None
    
    latest_dir = max(dirs, key=os.path.getctime)
    calib_file = os.path.join(latest_dir, "calibration_result.yaml")
    return calib_file if os.path.exists(calib_file) else None

def main():
    parser = argparse.ArgumentParser(description="标定验证工具 (生产版)")
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs/aisprayer_config.yaml"), help="配置文件路径")
    args = parser.parse_args()

    # 1. 加载配置
    full_cfg = load_config(args.config, PROJECT_ROOT)
    v_cfg = full_cfg.get("vision", {})
    p_cfg = v_cfg.get("planner", {})
    calib_path = get_abs_path(p_cfg.get("calib_path", "configs/calibration_result.yaml"), PROJECT_ROOT)
    
    if not os.path.exists(calib_path):
        print(f"[-] 找不到标定文件: {calib_path}")
        return
    print(f"[*] 正在验证标定文件: {calib_path}")

    # 2. 读取标定结果元数据
    with open(calib_path, 'r') as f: 
        res = yaml.safe_load(f)
    
    cam_params = res.get("camera_params", {})
    calib_cam = cam_params.get("camera_model", "orbbec")
    calib_w = cam_params.get("width", 640)
    calib_h = cam_params.get("height", 480)
    K = np.array(cam_params.get("intrinsic_matrix", []))
    D = np.array(cam_params.get("distortion_coeffs", []))

    # 3. 硬件型号与内参一致性校验
    h_cfg = full_cfg.get("hardware", {})
    cfg_model = h_cfg.get("camera", {}).get("model", "orbbec")
    
    if calib_cam != cfg_model:
        print(f"\n[CRITICAL] 硬件型号不匹配！\n  标定记录: {calib_cam}\n  当前配置: {cfg_model}")
        return

    # 4. 初始化并启动相机 (仅此一次)
    print(f"[*] 正在初始化并验证硬件: {calib_cam} ({calib_w}x{calib_h})")
    cam = get_camera(calib_cam, width=calib_w, height=calib_h)
    
    try:
        cam.start()
        # 显式校验：如果相机对象没有正确获取到参数，说明启动并未真正成功
        if not hasattr(cam, "pipeline") or cam.pipeline is None:
             print("[-] 相机启动异常，跳过硬件比对。")
             cam.stop(); return

        # 使用通用函数进行硬件一致性比对 (直接传入 cam 对象)
        ok, msg = verify_hardware_consistency(live=cam, calib=cam_params)
        if not ok:
            print(f"\n[CRITICAL] 硬件校验失败！\n  原因: {msg}")
            cam.stop(); return
        print(f"[+] 硬件一致性校验通过。")
    except Exception as e:
        print(f"[-] 相机启动或校验失败: {e}"); return

    T_bc = np.array(res["T_base_camera"])
    R_bc = T_bc[:3, :3]
    t_bc = T_bc[:3, 3]

    print("\n" + "="*80)
    print(" 纯文字版标定验证工具 (实时输出 XYZ + ABC) ")
    print("="*80)
    print("正在检测标定板...")

    pattern_size = (8, 11)
    sq_size = 15
    objp = np.zeros((88, 3), np.float32); objp[:, :2] = np.mgrid[0:8, 0:11].T.reshape(-1, 2) * sq_size

    try:
        while True:
            color_frame, _ = cam.get_frame()
            if color_frame is None:
                time.sleep(0.1); continue
            
            gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            
            if found:
                # 1. 解算位姿
                _, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
                R_cam_board, _ = cv2.Rodrigues(rvec)
                P_cam = tvec.flatten()
                
                # 2. 转换到基座系
                P_base = R_bc @ P_cam + t_bc
                R_base_board = R_bc @ R_cam_board
                
                # 3. 尝试不同的欧拉角顺序 (弧度)
                # 顺序 1: XYZ (Extrinsic)
                abc_xyz = R_tool.from_matrix(R_base_board).as_euler('XYZ', degrees=False)
                # 顺序 2: ZYX (Extrinsic, 很多工业机器人使用)
                abc_zyx = R_tool.from_matrix(R_base_board).as_euler('ZYX', degrees=False)
                
                # 刷新输出
                print(f"\r[POS] X:{P_base[0]:6.1f} Y:{P_base[1]:6.1f} Z:{P_base[2]:6.1f} | ", end="")
                print(f"[XYZ顺] A:{abc_xyz[0]:6.3f} B:{abc_xyz[1]:6.3f} C:{abc_xyz[2]:6.3f} | ", end="")
                print(f"[ZYX顺] A:{abc_zyx[2]:6.3f} B:{abc_zyx[1]:6.3f} C:{abc_zyx[0]:6.3f}", end="", flush=True)
            else:
                print(f"\r[未检测到标定板] 等待中...                                                                            ", end="", flush=True)
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n\n程序已退出。")
    finally:
        cam.stop()

if __name__ == "__main__": main()
