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
    parser.add_argument("--rows", type=int, default=12, help="标定板行数")
    parser.add_argument("--cols", type=int, default=9, help="标定板列数")
    parser.add_argument("--size", type=float, default=20.0, help="标定板方格大小(mm)")
    return parser.parse_args()

def load_or_init_info(yaml_path, cam, cam_model, args, pattern_size):
    """加载已有的或初始化新的采集信息"""
    if os.path.exists(yaml_path):
        print(f"[*] 检测到已存在的采集记录: {yaml_path}，正在加载并校验...")
        with open(yaml_path, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f)
        
        # 使用通用函数进行硬件一致性比对 (直接传入 cam 对象)
        ok, msg = verify_hardware_consistency(live=cam, scan=info.get("camera_params", {}))
        if not ok:
            raise ValueError(f"\n[CRITICAL] 硬件不一致！无法继续之前的采集任务。\n原因: {msg}")
            
        print(f"[+] 硬件校验通过，已加载 {len(info['samples'])} 组现有样本。")
    else:
        # 初始化全新的记录
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
                "rows": args.rows, "cols": args.cols, "square_size_mm": args.size,
                "pattern_size_inner": [pattern_size[0], pattern_size[1]]
            },
            "samples": []
        }
        print(f"[*] 开始新采集任务。相机: {cam_model}, 分辨率: {cam.width}x{cam.height}")
    return info

def do_capture(info, color, pose, save_dir, yaml_path):
    """执行单次采集逻辑：保存图片并更新 YAML"""
    count = len(info["samples"]) + 1
    img_name = f"image_{count:03d}.png"
    img_path = os.path.join(save_dir, img_name)
    
    cv2.imwrite(img_path, color)
    
    sample = {
        "id": count,
        "image_file": img_name,
        "robot_pose": {
            "x": round(pose.x, 3), "y": round(pose.y, 3), "z": round(pose.z, 3),
            "a": round(pose.a, 4), "b": round(pose.b, 4), "c": round(pose.c, 4)
        }
    }
    info["samples"].append(sample)
    
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.dump(info, f, default_flow_style=False)
    
    print(f"[OK] 已保存第 {count} 组样本: {img_name}")

def main():
    args = parse_args()
    
    # 1. 加载配置
    full_cfg = load_config(args.config, PROJECT_ROOT)
    h_cfg = full_cfg.get("hardware", {})
    c_cfg = full_cfg.get("calib", {})
    b_cfg = c_cfg.get("board", {})
    
    camera_model = h_cfg.get("camera", {}).get("model", "orbbec")
    robot_ip = h_cfg.get("robot", {}).get("ip", "192.168.2.14")
    robot_port = h_cfg.get("robot", {}).get("port", 6001)
    
    pattern_size = (args.cols - 1, args.rows - 1)
    output_root = get_abs_path(c_cfg.get("capture", {}).get("output_dir", "data/calib"), PROJECT_ROOT)

    # 2. 初始化硬件 (不进行 IO)
    cam = get_camera(camera_model)
    robot = InexbotDriver(ip=robot_ip, port=robot_port)

    try:
        # 3. 硬件启动与握手
        cam.start()
        if not robot.connect():
            print(f"[-] 机器人连接失败！IP: {robot_ip}")
            return
        
        robot.set_mode(MODE_TEACH)
        robot.set_coord(COORD_MCS)

        print("[*] 正在执行 Home 回零...")
        res_home = robot.go_home()
        if res_home != 0:
            print(f"[-] 回零运动失败！错误码: {res_home}")
            return
        robot.wait_motion_done()
        print("[+] 硬件自检通过，机械臂已就位。")

        # 4. 只有在硬件就绪后，才正式创建采集目录
        timestamp = datetime.now().strftime("%Y%m%d")
        save_dir = os.path.join(output_root, f"calib_{timestamp}")
        os.makedirs(save_dir, exist_ok=True)
        yaml_path = os.path.join(save_dir, "calibration_info.yaml")
        
        try:
            info = load_or_init_info(yaml_path, cam, camera_model, args, pattern_size)
        except ValueError as e:
            print(e)
            return

        # 5. 采集循环
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
