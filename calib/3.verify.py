import os, cv2, yaml, numpy as np, time, sys

# 确保能找到项目根目录下的模块
sys.path.append(os.getcwd())
from camera.orbbec_driver import OrbbecDriver
from scipy.spatial.transform import Rotation as R_tool

def main():
    # 1. 加载最新的标定结果
    res_path = "calib/data/calib_20260501/calibration_result.yaml"
    if not os.path.exists(res_path):
        print(f"[-] 找不到标定文件: {res_path}"); return
    
    with open(res_path, 'r') as f: res = yaml.safe_load(f)
    T_bc = np.array(res["T_base_camera"])
    R_bc = T_bc[:3, :3]
    t_bc = T_bc[:3, 3]
    
    # 2. 加载相机参数
    info_path = "calib/data/calib_20260501/calibration_info.yaml"
    with open(info_path, 'r') as f: info = yaml.safe_load(f)
    K = np.array(info["camera_params"]["intrinsic_matrix"])
    D = np.array(info["camera_params"]["distortion_coeffs"])
    
    # 3. 初始化 Orbbec 相机驱动
    cam = OrbbecDriver()
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
