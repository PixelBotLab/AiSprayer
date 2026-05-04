import os
import sys
import argparse
import cv2
import time
import yaml
import numpy as np

# 1. 路径锚定策略
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
SRC_PATH = os.path.join(PROJECT_ROOT, "src")
if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)

from aisprayer.core.vision.recorder import ScanRecorder
from aisprayer.core.hardware.camera.factory import get_camera
from aisprayer.utils.config_helper import load_config, get_abs_path

def main():
    parser = argparse.ArgumentParser(description="AiSprayer 数据采集工具 (生产版)")
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs/aisprayer_config.yaml"), help="配置文件路径")
    parser.add_argument("--input", help="指定裤子 ID (不指定则自动锁定最新裤子)")
    parser.add_argument("--new", action="store_true", help="开启一个全新的裤子采集任务")
    parser.add_argument("--angle", choices=["0", "90", "180", "270"], default="0", help="指定当前视角 (默认 0)")
    args = parser.parse_args()

    # 1. 加载配置与输出路径
    full_cfg = load_config(args.config, PROJECT_ROOT)
    v_cfg = full_cfg.get("vision", {})
    output_root = get_abs_path(v_cfg.get("output_root", "data/runs"), PROJECT_ROOT)
    
    # 2. 确定 Garment ID (智能管理)
    garment_id = args.input
    
    def get_next_id(root):
        """扫描目录获取下一个自增 ID"""
        if not os.path.exists(root): return "trouser_001"
        existing_ids = []
        for d in os.listdir(root):
            if d.startswith("trouser_") and d[8:].isdigit():
                existing_ids.append(int(d[8:]))
        if not existing_ids: return "trouser_001"
        return f"trouser_{max(existing_ids) + 1:03d}"

    if args.new:
        garment_id = get_next_id(output_root)
        print(f"[*] 开启全新采集任务: {garment_id}")
    elif not garment_id:
        # 自动寻找最近一次操作的目录
        if os.path.exists(output_root):
            dirs = [d for d in os.listdir(output_root) if os.path.isdir(os.path.join(output_root, d)) and d.startswith("trouser_")]
            if dirs:
                garment_id = sorted(dirs, key=lambda d: os.path.getmtime(os.path.join(output_root, d)))[-1]
                print(f"[*] 自动锁定最新裤子: {garment_id}")
        
        if not garment_id:
            garment_id = "trouser_001"
            print(f"[*] 未发现历史数据，初始化: {garment_id}")

    # 3. 初始化硬件
    h_cfg = full_cfg.get("hardware", {})
    camera_model = h_cfg.get("camera", {}).get("model", "orbbec")
    print(f"[*] 正在连接相机: {camera_model}")
    cam = get_camera(camera_model)
    try:
        cam.start()
    except Exception as e:
        print(f"[-] 相机启动失败: {e}")
        return

    # 4. 初始化记录器
    K, D = cam.get_intrinsics()
    cam_info = {
        "camera_model": camera_model,
        "width": cam.width, "height": cam.height,
        "intrinsic_matrix": K.tolist() if K is not None else [],
        "distortion_coeffs": D.tolist() if D is not None else []
    }
    recorder = ScanRecorder(output_root=output_root)
    
    win_name = f"Scan Capture - {garment_id} [{args.angle}deg]"
    cv2.namedWindow(win_name)

    print("\n" + "="*50)
    print(f" [采集目标]: {garment_id}")
    print(f" [当前视角]: {args.angle} 度")
    print(" [指令]: [SPACE] 拍照 | [Q] 退出")
    print("="*50)

    try:
        while True:
            color, depth = cam.get_frame()
            if color is None: continue

            # 简单的实时预览
            display = color.copy()
            cv2.putText(display, f"ID: {args.garment} | Angle: {args.angle}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow(win_name, display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                print(f"[*] 正在保存数据 (相机: {camera_model})...")
                # 调用 recorder 保存数据，传入标准化的 cam_info
                saved_path = recorder.save_scan(color, depth, cam_info, 
                                               garment_id=args.garment, 
                                               angle=args.angle)
                print(f"[+] 数据已成功落盘: {saved_path}")
            elif key == ord('q'):
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
