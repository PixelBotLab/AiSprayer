#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
眼在手外 (Eye-to-Hand) 标定数据采集程序 - 增强版

功能：
1. 实时预览并自动识别标定板角点，支持 2 倍放大显示。
2. 启动时自动加载已有采集数据，支持在同一目录下继续采集。
3. 记录相机内参、标定板参数及机械臂位姿到 YAML。
"""

import os
import sys
import cv2
import yaml
import argparse
import numpy as np
from datetime import datetime

# 确保导入路径正确
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(ROOT_DIR)

from robot.inexbot_driver import InexbotDriver, COORD_MCS, MODE_TEACH
from camera.orbbec_driver import OrbbecDriver

def parse_args():
    parser = argparse.ArgumentParser(description="标定数据采集工具")
    parser.add_argument("--ip", type=str, default="192.168.2.14", help="机器人 IP")
    parser.add_argument("--rows", type=int, default=12, help="标定板行数 (节点数)")
    parser.add_argument("--cols", type=int, default=9, help="标定板列数 (节点数)")
    parser.add_argument("--size", type=float, default=15.0, help="棋盘格每个格子的大小 (mm)")
    parser.add_argument("--out", type=str, default="data", help="输出根目录")
    return parser.parse_args()

def load_or_init_info(yaml_path, cam, args, pattern_size):
    """加载已有 YAML 或初始化新的配置信息"""
    if os.path.exists(yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f)
        print(f"[*] 加载到已有数据，当前已采集 {len(info['samples'])} 组样本。")
    else:
        # 获取相机内参
        K, D = cam.get_intrinsics()
        info = {
            "camera_params": {
                "intrinsic_matrix": K.tolist() if K is not None else [],
                "distortion_coeffs": D.tolist() if D is not None else []
            },
            "board_params": {
                "rows": args.rows, "cols": args.cols, "square_size_mm": args.size,
                "pattern_size_inner": [pattern_size[0], pattern_size[1]]
            },
            "samples": []
        }
        print("[*] 未发现历史数据，开始新的采集任务。")
    return info

def do_capture(info, color, pose, save_dir, yaml_path):
    """执行单次采集逻辑：保存图片并更新 YAML"""
    # 自动计算下一个 ID
    count = len(info["samples"]) + 1
    img_name = f"image_{count:03d}.png"
    img_path = os.path.join(save_dir, img_name)
    
    # 保存图片 (保存原始图以维持精度)
    cv2.imwrite(img_path, color)
    
    # 构造样本数据
    sample = {
        "id": count,
        "image_file": img_name,
        "robot_pose": {
            "x": round(pose.x, 3), "y": round(pose.y, 3), "z": round(pose.z, 3),
            "a": round(pose.a, 4), "b": round(pose.b, 4), "c": round(pose.c, 4)
        }
    }
    info["samples"].append(sample)
    
    # 写入 YAML
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.dump(info, f, default_flow_style=False)
    
    print(f"[OK] 已保存第 {count} 组样本: {img_name}")
    return count

def main():
    args = parse_args()
    pattern_size = (args.cols - 1, args.rows - 1)

    # 1. 确定保存目录
    timestamp = datetime.now().strftime("%Y%m%d")
    save_dir = os.path.join(args.out, f"calib_{timestamp}")
    os.makedirs(save_dir, exist_ok=True)
    yaml_path = os.path.join(save_dir, "calibration_info.yaml")
    
    # 2. 初始化硬件
    cam = OrbbecDriver(width=1280, height=720)
    robot = InexbotDriver(ip=args.ip)

    try:
        cam.start()
        if not robot.connect():
            print("[-] 机器人连接失败！")
            return
        
        robot.set_mode(MODE_TEACH)
        robot.set_coord(COORD_MCS)

        print("go home ... ")
        robot.go_home()
        print("go home done")
        
        # 3. 加载数据配置
        info = load_or_init_info(yaml_path, cam, args, pattern_size)

        print(f"[*] 输出目录: {save_dir}")
        print("\n[操作说明] 按 [空格] 采集，按 [Q] 退出\n")

        while True:
            color, _ = cam.get_frame()
            if color is None: continue

            # 标定板检测预览
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            
            display_img = color.copy()
            if ret:
                cv2.drawChessboardCorners(display_img, pattern_size, corners, ret)
                status_color, status_text = (0, 255, 0), "READY"
            else:
                status_color, status_text = (0, 0, 255), "SEARCHING"

            # 叠加状态信息
            cur_count = len(info["samples"])
            cv2.putText(display_img, f"Total Samples: {cur_count} | Status: {status_text}", 
                        (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
            
            # 放大 1 倍显示 (2560x1440)
            zoom_img = cv2.resize(display_img, (display_img.shape[1]*2, display_img.shape[0]*2))
            cv2.imshow("Calibration Capture", zoom_img)
            
            key = cv2.waitKey(1) & 0xFF
            
            # 按空格采集
            if key == ord(' ') and ret:
                pose = robot.get_current_pose()
                if pose:
                    do_capture(info, color, pose, save_dir, yaml_path)
                else:
                    print("[Error] 无法获取机器人位姿")
            
            elif key in [ord('q'), 27]:
                break

    except Exception as e:
        print(f"\n[Fatal Error] {e}")
    finally:
        cam.stop()
        robot.disconnect()
        cv2.destroyAllWindows()
        print(f"\n采集结束。最终样本总数: {len(info.get('samples', []))}")

if __name__ == "__main__":
    main()
