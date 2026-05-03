import cv2
import numpy as np
import open3d as o3d
import argparse
import os
import time
import sys
import yaml

# 确保能找到项目根目录下的模块
sys.path.append(os.getcwd())
from camera.factory import get_camera

def depth_to_pcd(depth, intrinsics):
    """将深度图转换为点云数据"""
    fx, fy, cx, cy = intrinsics
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    z = depth.astype(np.float32)
    
    # 过滤无效深度
    mask = (z > 100) & (z < 3000)
    
    z = z[mask]
    u = u[mask]
    v = v[mask]
    
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    
    points = np.column_stack((x, y, z))
    return points

def save_scan(color, depth, intrinsics, output_dir, custom_name=None):
    """保存扫描数据 (每个采集保存到独立文件夹)"""
    if custom_name:
        scan_dir = os.path.join(output_dir, custom_name)
    else:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        scan_dir = os.path.join(output_dir, f"scan_{timestamp}")
        
    os.makedirs(scan_dir, exist_ok=True)
    
    # 转换为点云
    points = depth_to_pcd(depth, intrinsics)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 文件路径
    pcd_path = os.path.join(scan_dir, "scan.pcd")
    img_path = os.path.join(scan_dir, "scan.jpg")
    depth_path = os.path.join(scan_dir, "scan.depth.npy")
    
    o3d.io.write_point_cloud(pcd_path, pcd)
    cv2.imwrite(img_path, color)
    np.save(depth_path, depth)
    
    print(f"[+] 保存成功: {scan_dir}")
    print(f"    - 点数: {len(points)}")

def letterbox(img, target_size=(640, 480), color=(0, 0, 0)):
    """保持比例缩放并填充 (Letterbox)"""
    h, w = img.shape[:2]
    tw, th = target_size
    scale = min(tw / w, th / h)
    nw, nh = int(w * scale), int(h * scale)
    
    # 缩放图像
    resized_img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    
    # 创建画布并填充
    top = (th - nh) // 2
    bottom = th - nh - top
    left = (tw - nw) // 2
    right = tw - nw - left
    
    new_img = cv2.copyMakeBorder(resized_img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return new_img

def main():
    parser = argparse.ArgumentParser(description="点云采集工具 (支持实机采集与模拟生成)")
    parser.add_argument("--frames", type=int, default=1, help="融合的帧数 (默认 1)")
    parser.add_argument("--calib", default="calib/data/calib_20260501/calibration_result.yaml", help="标定文件路径")
    parser.add_argument("--output_dir", default="vision/data", help="数据保存目录")
    parser.add_argument("--mock", nargs="+", help="模拟模式：指定一个或多个输入图片路径，将跳过相机直接生成点云数据")
    parser.add_argument("--camera", choices=["orbbec", "realsense"], default="orbbec", help="选择相机类型 (默认 orbbec)")
    args = parser.parse_args()

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    # 1. 从标定文件读取分辨率和内参
    width, height = 1280, 720
    intrinsics = [900, 900, 640, 360]
    if os.path.exists(args.calib):
        with open(args.calib, 'r') as f:
            res = yaml.safe_load(f)
            width = res.get("camera_params", {}).get("width", 1280)
            height = res.get("camera_params", {}).get("height", 720)
            K = np.array(res["camera_params"]["intrinsic_matrix"])
            intrinsics = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
        print(f"[+] 已从标定文件加载参数: {width}x{height}, fx={intrinsics[0]}")
    else:
        print(f"[!] 警告: 找不到标定文件 {args.calib}，使用默认参数")

    # 2. 模拟模式逻辑
    if args.mock:
        print(f"[*] 正在运行批量模拟模式，共 {len(args.mock)} 个文件...")
        for img_path in args.mock:
            if not os.path.exists(img_path):
                print(f"[-] 警告: 找不到模拟图片 {img_path}，跳过")
                continue
            
            file_name = os.path.splitext(os.path.basename(img_path))[0]
            print(f"[*] 处理文件: {img_path} -> 目录: {file_name}")
            
            mock_img = cv2.imread(img_path)
            if mock_img is None:
                print(f"[-] 错误: 图片 {img_path} 解码失败，跳过")
                continue
                
            # 使用 Letterbox 缩放，保持比例不变形
            mock_img = letterbox(mock_img, (width, height))
            
            # 模拟 1200mm 的平面深度图
            mock_depth = np.full((height, width), 1200, dtype=np.uint16)
            
            save_scan(mock_img, mock_depth, intrinsics, args.output_dir, custom_name=file_name)
            
        print("[+] 所有模拟数据生成完成。")
        return

    # 3. 正常相机模式
    print(f"[*] 正在初始化 {args.camera} 相机...")
    try:
        cam = get_camera(args.camera, width=width, height=height)
        cam.start()
        print(f"[+] 相机已就绪。")
    except Exception as e:
        print(f"[-] 相机启动失败: {e}")
        return

    win_name = "Point Cloud Capture (Space to Save, Q to Quit)"
    cv2.namedWindow(win_name)

    print("\n" + "="*50)
    print(" [采集指令]:")
    print("  [Space]: 采集并保存一帧 (或多帧融合) 点云")
    print("  [Q]: 退出程序")
    print("="*50)

    try:
        while True:
            color, depth = cam.get_frame()
            if color is None: continue

            display = color.copy()
            cv2.putText(display, f"FUSION_FRAMES: {args.frames}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow(win_name, display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                print(f"[*] 正在采集 {args.frames} 帧并融合...")
                
                accum_depth = np.zeros_like(depth, dtype=np.float32)
                valid_counts = np.zeros_like(depth, dtype=np.int32)
                captured_color = color.copy()
                
                for i in range(args.frames):
                    _, d = cam.get_frame()
                    if d is not None:
                        mask = d > 0
                        accum_depth[mask] += d[mask]
                        valid_counts[mask] += 1
                    time.sleep(0.01)
                
                final_depth = np.zeros_like(depth, dtype=np.uint16)
                mask = valid_counts > 0
                final_depth[mask] = (accum_depth[mask] / valid_counts[mask]).astype(np.uint16)

                save_scan(captured_color, final_depth, intrinsics, args.output_dir)

            elif key == ord('q'):
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
