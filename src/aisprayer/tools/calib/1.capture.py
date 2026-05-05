import os
import sys
import argparse
import cv2
import numpy as np
import yaml
import time
from datetime import datetime

# 1. 路径锚定策略
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.hardware.robot.inexbot_driver import InexbotDriver, RobotPose, MODE_TEACH, COORD_MCS
from aisprayer.core.hardware.camera.factory import get_camera
from aisprayer.utils.config_helper import load_config, get_abs_path
from aisprayer.utils.hardware_helper import verify_hardware_consistency

def parse_args():
    parser = argparse.ArgumentParser(description="标定数据采集工具 (生产版)")
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs/aisprayer_config.yaml"), help="配置文件路径")
    return parser.parse_args()

def load_or_init_info(yaml_path, cam, cam_model, board_cfg):
    """
    加载已有的或初始化新的采集信息。
    
    逻辑说明:
    - rows/cols 是棋盘格的方块数，但 OpenCV 寻找的是内角点，所以 pattern_size = (cols-1, rows-1)
    """
    rows, cols = board_cfg.get("rows", 12), board_cfg.get("cols", 9)
    size = board_cfg.get("square_size_mm", 15.0)
    pattern_size = (cols - 1, rows - 1)

    if os.path.exists(yaml_path):
        # 情况 A: 续接之前的采集任务
        print(f"[*] 检测到已存在的采集记录: {yaml_path}，正在加载并校验...")
        with open(yaml_path, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f)
        
        # 校验当前连接的相机是否与记录中的一致，防止数据混淆
        ok, msg = verify_hardware_consistency(live=cam, scan=info.get("camera_params", {}))
        if not ok:
            raise ValueError(f"\n[CRITICAL] 硬件不一致！无法继续之前的采集任务。\n原因: {msg}")
            
        print(f"[+] 硬件校验通过，已加载 {len(info.get('samples', []))} 组现有样本。")
    else:
        # 情况 B: 创建全新的采集记录
        K, D = cam.get_intrinsics()
        info = {
            "version": "1.0",
            "camera_params": {
                "camera_model": cam_model,
                "width": cam.width,
                "height": cam.height,
                "intrinsic_matrix": K.tolist() if K is not None else [],
                "distortion_coeffs": D.tolist() if D is not None else []
            },
            "board_params": {
                "rows": rows, "cols": cols, "square_size_mm": size,
                "pattern_size_inner": [pattern_size[0], pattern_size[1]]
            },
            "samples": []
        }
        print(f"[*] 开始新采集任务。相机: {cam_model}, 标定板: {rows}x{cols}({size}mm)")
    return info, pattern_size

def do_capture(info, color, pose, save_dir, yaml_path):
    """执行单次采集逻辑：保存图片并更新 YAML 记录"""
    count = len(info["samples"]) + 1
    img_name = f"image_{count:03d}.png"
    img_path = os.path.join(save_dir, img_name)
    
    # 1. 保存原始彩色图
    cv2.imwrite(img_path, color)
    
    # 2. 记录当前机械臂位姿 (X, Y, Z, A, B, C)
    sample = {
        "id": count,
        "image_file": img_name,
        "robot_pose": {
            "x": round(pose.x, 3), "y": round(pose.y, 3), "z": round(pose.z, 3),
            "a": round(pose.a, 4), "b": round(pose.b, 4), "c": round(pose.c, 4)
        }
    }
    info["samples"].append(sample)
    
    # 3. 实时同步保存 YAML，防止意外断电导致数据丢失
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.dump(info, f, default_flow_style=False)
    
    print(f"[OK] 已保存第 {count} 组样本: {img_name}")

def main():
    args = parse_args()
    
    # --- 1. 加载全局配置 ---
    full_cfg = load_config(args.config, PROJECT_ROOT)
    h_cfg = full_cfg.get("hardware", {})
    c_cfg = full_cfg.get("calib", {})
    b_cfg = c_cfg.get("board", {})
    
    camera_model = h_cfg.get("camera", {}).get("model", "orbbec")
    robot_ip = h_cfg.get("robot", {}).get("ip", "192.168.2.14")
    robot_port = h_cfg.get("robot", {}).get("port", 6001)
    
    # 标定采集数据的存储根目录
    output_root = get_abs_path(c_cfg.get("capture", {}).get("output_dir", "data/calib"), PROJECT_ROOT)

    # --- 2. 初始化硬件对象 ---
    cam = get_camera(camera_model)
    robot = InexbotDriver(ip=robot_ip, port=robot_port)

    # --- 3. 硬件启动与握手 ---
    print(f"\n[*] 标定板配置: {b_cfg.get('rows')} 行 x {b_cfg.get('cols')} 列 (格大小: {b_cfg.get('square_size_mm')} mm)")
        
    try:
        # 1. 连接相机
        print(f"[*] 正在尝试连接硬件 (相机: {camera_model})...")
        try:
            cam.start()
        except RuntimeError as e:
            print(f"\n[!] 相机启动失败: {e}")
            print("[!] 请检查相机是否连接或驱动是否正常。")
            return

        # 2. 连接机器人
        print(f"[*] 正在尝试连接硬件 (机器人 IP: {robot_ip})...")
        if not robot.connect():
            print(f"\n[!] 机器人连接失败！请确认 IP {robot_ip} 是否通畅。")
            return
        
        # 标定必须在教学模式 (MODE_TEACH) 下手动移动机械臂
        robot.set_mode(MODE_TEACH)
        robot.set_coord(COORD_MCS)

        print("[*] 正在执行 Home 回零，确保坐标系状态一致...")
        res_home = robot.go_home()
        if res_home != 0:
            print(f"[-] 回零运动失败！错误码: {res_home}")
            return
        robot.wait_motion_done()
        print("[+] 硬件自检通过，机械臂已就位。")

        # --- 4. 目录准备 ---
        # 自动以当前日期命名目录，例如 data/calib/calib_20240505
        timestamp = datetime.now().strftime("%Y%m%d")
        save_dir = os.path.join(output_root, f"calib_{timestamp}")
        os.makedirs(save_dir, exist_ok=True)
        yaml_path = os.path.join(save_dir, "calibration_info.yaml")
        
        # 加载或初始化标定信息 (从配置同步 board 规格)
        try:
            info, pattern_size = load_or_init_info(yaml_path, cam, camera_model, b_cfg)
        except ValueError as e:
            print(e)
            return

        # --- 5. 采集主循环 ---
        win_name = "Calibration Capture Tool"
        cv2.namedWindow(win_name)
        
        print("\n" + "="*50)
        print(" [SPACE]: 触发采集")
        print(" [Q]: 退出并保存")
        print("="*50)

        while True:
            color, depth = cam.get_frame()
            if color is None: continue
            
            display = color.copy()
            # 绘制角点预览
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if ret:
                cv2.drawChessboardCorners(display, pattern_size, corners, ret)

            cv2.imshow(win_name, display)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' '):
                pose = robot.get_current_pose()
                if pose:
                    do_capture(info, color, pose, save_dir, yaml_path)
                else:
                    print("[-] 获取位姿失败，无法采集")
            elif key == ord('q'):
                break
                
    finally:
        cam.stop()
        robot.disconnect()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
