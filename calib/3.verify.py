import os, cv2, yaml, numpy as np, time, sys
import argparse
import glob

# 确保能找到项目根目录下的模块
sys.path.append(os.getcwd())
from camera.orbbec_driver import OrbbecDriver
from scipy.spatial.transform import Rotation as R_tool

def get_latest_calib_file(data_dir="calib/data"):
    dirs = glob.glob(os.path.join(data_dir, "calib_*"))
    if not dirs:
        return None
    latest_dir = max(dirs, key=os.path.getctime)
    calib_file = os.path.join(latest_dir, "calibration_result.yaml")
    return calib_file if os.path.exists(calib_file) else None

def main():
    parser = argparse.ArgumentParser(description="标定验证工具")
    parser.add_argument("--calib", default=None, help="标定结果文件路径 (默认自动查找最新的)")
    args = parser.parse_args()

    # 1. 确定标定文件路径
    res_path = args.calib
    if not res_path:
        res_path = get_latest_calib_file()
        if not res_path:
            print("[-] 找不到任何标定文件！请手动指定。")
            return
        print(f"[*] 自动找到最新标定文件: {res_path}")

    if not os.path.exists(res_path):
        print(f"[-] 标定文件不存在: {res_path}")
        return
    
    with open(res_path, 'r') as f: res = yaml.safe_load(f)
    T_bc = np.array(res["T_base_camera"])
    R_bc = T_bc[:3, :3]
    t_bc = T_bc[:3, 3]
    
    # 2. 从标定文件中直接加载相机参数
    cam_params = res.get("camera_params", {})
    if not cam_params:
        print("[-] 标定文件中未找到 camera_params 字段！")
        return
        
    K = np.array(cam_params.get("intrinsic_matrix", []))
    D = np.array(cam_params.get("distortion_coeffs", []))
    calib_w = cam_params.get("width", 640)
    calib_h = cam_params.get("height", 480)
    
    # 3. 初始化 Orbbec 相机驱动
    cam = OrbbecDriver(width=calib_w, height=calib_h)
    try:
        cam.start()
    except Exception as e:
        print(f"[-] 相机启动失败: {e}"); return

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
