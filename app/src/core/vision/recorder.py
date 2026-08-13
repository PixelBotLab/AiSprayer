import cv2
import numpy as np
import os
import time
import yaml
import open3d as o3d
from core.vision.point_cloud_processor import depth_to_pcd

class ScanRecorder:
    """
    负责采集视觉数据并以标准格式保存到磁盘。
    标准格式包含：彩色图 (JPG)、深度图 (NPY)、点云 (PCD) 以及相机参数 (YAML)。
    """
    def __init__(self, output_root="data/runs"):
        """
        初始化录制器
        :param output_root: 存储根目录，建议使用绝对路径
        """
        self.output_root = output_root
        if not os.path.exists(self.output_root):
            os.makedirs(self.output_root)

    def save_scan(self, color, depth, camera_params, garment_id=None, angle="0"):
        """
        保存单次扫描数据。
        :param color: BGR 图像
        :param depth: uint16 深度图 (mm)
        :param camera_params: 相机内参字典
        :param garment_id: 裤子唯一 ID (如 garment_001)
        :param angle: 角度 (0, 90, 180, 270)
        """
        if garment_id:
            # 模式 A: 按裤子 ID 和角度组织 (生产模式)
            scan_dir = os.path.join(self.output_root, garment_id, str(angle))
        else:
            # 模式 B: 仅按时间戳组织 (调试模式)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            scan_dir = os.path.join(self.output_root, f"scan_{timestamp}")
            
        os.makedirs(scan_dir, exist_ok=True)
        
        # 1. 提取内参并保存点云
        cam_model = camera_params.get("camera_model", "unknown")
        K_list = camera_params.get("intrinsic_matrix", [])
        
        # 提取 [fx, fy, cx, cy] 用于点云转换
        if len(K_list) >= 3:
            intr = [K_list[0][0], K_list[1][1], K_list[0][2], K_list[1][2]]
        else:
            # 回退默认值
            intr = [900.0, 900.0, 640.0, 360.0]
            
        points = depth_to_pcd(depth, intr)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(os.path.join(scan_dir, "scan.pcd"), pcd)
        
        # 2. 保存原始彩色图和深度图
        cv2.imwrite(os.path.join(scan_dir, "scan.jpg"), color)
        np.save(os.path.join(scan_dir, "scan.depth.npy"), depth)
        
        # 3. 保存元数据 (包含完整的硬件身份)
        meta = {
            "version": "1.2",
            "garment_id": garment_id or "debug",
            "angle": angle,
            "timestamp": time.time(),
            "camera_params": camera_params, # 包含型号、分辨率、内参、畸变
        }
        
        with open(os.path.join(scan_dir, "scan.params.yaml"), 'w') as f:
            yaml.dump(meta, f, default_flow_style=False)
            
        return scan_dir

def letterbox(img, target_size=(640, 480), color=(0, 0, 0)):
    """保持比例缩放并填充 (Letterbox)，常用于模型输入或数据对齐"""
    h, w = img.shape[:2]
    tw, th = target_size
    scale = min(tw / w, th / h)
    nw, nh = int(w * scale), int(h * scale)
    
    resized_img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    
    top = (th - nh) // 2
    bottom = th - nh - top
    left = (tw - nw) // 2
    right = tw - nw - left
    
    new_img = cv2.copyMakeBorder(resized_img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return new_img, scale, (left, top)
