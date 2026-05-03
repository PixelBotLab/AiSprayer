import numpy as np
import cv2
import os
import sys
import yaml
import glob
from scipy.spatial.transform import Rotation as R_tool

# 确保能找到项目根目录下的模块
sys.path.append(os.getcwd())
from vision.segmentation import TrousersSegmenter

class AiSprayPlanner:
    """
    工业级喷涂路径规划器
    基于 14 个裤子关键点和 3D 点云，生成符合机器人基座坐标系的“之”字形轨迹。
    参考了 calib/4.aligner.py 的坐标转换逻辑。
    """
    def __init__(self, spray_width=80, overlap=0.2, spray_dist=150, v_step_mm=20.0, calib_path=None):
        """
        :param spray_width: 喷头有效幅宽 (mm)
        :param overlap: 路径重叠率 (0~1)
        :param spray_dist: 喷涂时保持的法向距离 (mm)
        :param v_step_mm: 纵向轨迹点采样间隔 (mm)
        :param calib_path: 标定文件路径，用于获取 T_base_camera
        """
        self.step_x = spray_width * (1 - overlap) 
        self.spray_dist_mm = spray_dist
        self.v_step_mm = v_step_mm
        self.points_3d_cam = {} # 存储 14 个关键点的相机坐标 (1-based)
        self.T_base_camera = np.eye(4)
        self.camera_intrinsics = None # [fx, fy, cx, cy]
        
        if calib_path and os.path.exists(calib_path):
            self.load_calibration(calib_path)
            
        self.polygon_pts = None # 用于判定是否在裤子区域内的 2D 多边形
        self.segmenter = TrousersSegmenter() # 初始化 2D 轮廓分割器
        
        # 加载新的 YOLO 分割模型 (wissight.pt)
        self.yolo_segmenter = None
        yolo_model_path = os.path.join(os.getcwd(), "models/wissight.pt")
        if os.path.exists(yolo_model_path):
            from vision.yolo_segmentation import YoloTrousersSegmenter
            self.yolo_segmenter = YoloTrousersSegmenter(model_path=yolo_model_path)
            print("[+] Planner: 已加载 YOLO 高精度分割模型")

    def load_calibration(self, calib_path):
        """加载手眼标定结果及相机内参"""
        with open(calib_path, 'r') as f:
            res = yaml.safe_load(f)
        
        # 加载外参
        self.T_base_camera = np.array(res["T_base_camera"])
        
        # 加载内参
        K = np.array(res["camera_params"]["intrinsic_matrix"])
        self.camera_intrinsics = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
        
        print(f"[+] Planner: 已加载外参 T_base_camera")
        print(f"[+] Planner: 已加载内参 fx={self.camera_intrinsics[0]:.2f}, fy={self.camera_intrinsics[1]:.2f}")

    def set_landmarks_3d(self, landmarks_2d, depth_map, camera_intrinsics, image=None, detect_res=None):
        """
        将 2D 像素关键点投影到 3D 相机坐标系。
        camera_intrinsics: [fx, fy, cx, cy]
        image: 原始图像，用于 2D 轮廓细化
        detect_res: 包含 bbox 和 keypoints 分数的原始检测结果
        """
        fx, fy, cx, cy = camera_intrinsics
        self.depth_map = depth_map # 存储深度图供规划时使用
        
        # 1. 默认：使用关键点组成的 14 点多边形
        poly_indices = [0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 2, 1]
        self.polygon_pts = np.array([landmarks_2d[i] for i in poly_indices], dtype=np.int32)

        # 2. 姿态模糊检测与自动降级 (Silhouette 模式)
        if image is not None and detect_res is not None:
            full_kpts = detect_res['keypoints']
            bbox = detect_res['boxes']
            
            if self.segmenter.is_pose_ambiguous(full_kpts, bbox):
                print("[!] Planner: 检测到姿态模糊 (可能为侧面或遮挡)，切换至高精度轮廓模式...")
                
                # 优先使用 YOLO 分割模型
                silhouette_poly = None
                if self.yolo_segmenter is not None:
                    silhouette_poly = self.yolo_segmenter.get_silhouette_polygon(image)
                    if silhouette_poly is not None:
                        print(f"[+] Planner: 已使用 YOLO 提取物理轮廓 (点数: {len(silhouette_poly)})")
                
                # 如果没有 YOLO 或提取失败，使用传统的 GrabCut
                if silhouette_poly is None:
                    silhouette_poly = self.segmenter.get_silhouette_polygon(image, bbox, full_kpts)
                    if silhouette_poly is not None:
                        print(f"[+] Planner: 已使用 GrabCut 提取物理轮廓 (点数: {len(silhouette_poly)})")
                
                if silhouette_poly is not None:
                    self.polygon_pts = silhouette_poly
                    return # 轮廓模式下不再处理 3D 关键点语义

        # 3. 投影关键点到 3D
        for i, (u, v) in enumerate(landmarks_2d):
            v_int, u_int = int(v), int(u)
            # 简单中值滤波获取深度
            roi = depth_map[max(0, v_int-2):v_int+3, max(0, u_int-2):u_int+3]
            valid_depths = roi[roi > 0]
            z = float(np.median(valid_depths)) if len(valid_depths) > 0 else 0
            
            if z <= 0: continue
                
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            self.points_3d_cam[i + 1] = np.array([x, y, z])

    def generate_path(self, point_cloud_processor):
        """
        生成机器人基座坐标系下的轨迹序列。
        彻底解决边缘点问题：先在 2D 像素平面判定多边形内点，然后再映射到 3D 查询法向。
        """
        if self.polygon_pts is None or self.depth_map is None:
            print("[-] Planner: 缺少关键点或深度图数据")
            return []

        # 获取 2D 多边形的 bounding box 并外扩 10 像素以确保覆盖边缘
        margin = 10
        u_min = np.min(self.polygon_pts[:, 0]) - margin
        u_max = np.max(self.polygon_pts[:, 0]) + margin
        v_min = np.min(self.polygon_pts[:, 1]) - margin
        v_max = np.max(self.polygon_pts[:, 1]) + margin
        
        # 估算像素步长
        valid_depths = self.depth_map[(self.depth_map > 100) & (self.depth_map < 3000)]
        if len(valid_depths) == 0:
            print("[-] Planner: 深度图无效")
            return []
        median_z = float(np.median(valid_depths))
        
        fx, fy, cx, cy = self.camera_intrinsics
        step_u_target = int(self.step_x * fx / median_z)
        step_v = int(self.v_step_mm * fy / median_z) 
        
        if step_u_target < 1: step_u_target = 10
        if step_v < 1: step_v = 5

        # --- 优化：确保第一条和最后一条线对齐边界 ---
        poly_w = u_max - u_min
        n_cols = max(2, int(np.ceil(poly_w / step_u_target)) + 1)
        u_samples = np.linspace(u_min, u_max, n_cols).astype(np.int32)
        
        full_trajectory = []
        is_downward = True

        # 之字形扫描 (基于生成的边界对齐采样点)
        for u_curr in u_samples:
            v_samples = list(range(int(v_min), int(v_max), step_v))
            
            valid_v_info = []
            for v_curr in v_samples:
                # 1. 优先判定：点是否在多边形内
                dist = cv2.pointPolygonTest(self.polygon_pts, (float(u_curr), float(v_curr)), True)
                
                # 允许边缘外扩少量像素，以防覆盖不全
                is_inside = dist >= -15
                
                if dist < -50:
                    continue # 偏离太远的点直接跳过
                
                # 2. 获取该像素的深度
                z = float(self.depth_map[v_curr, u_curr])
                if z <= 100 or z >= 3000:
                    # 深度无效时尝试使用周围深度的中值
                    roi = self.depth_map[max(0, v_curr-2):v_curr+3, max(0, u_curr-2):u_curr+3]
                    roi_valid = roi[(roi > 100) & (roi < 3000)]
                    if len(roi_valid) > 0:
                        z = float(np.median(roi_valid))
                    else:
                        continue # 仍然无效则抛弃
                
                # 3. 映射为相机系 3D 坐标
                x_cam = (u_curr - cx) * z / fx
                y_cam = (v_curr - cy) * z / fy
                pt_cam = np.array([x_cam, y_cam, z])
                
                # 4. 在点云中寻找最近点来估算法向
                [k, idx, _] = point_cloud_processor.kdtree.search_knn_vector_3d(pt_cam, 5)
                if k < 1:
                    continue
                
                # 平均最近邻的法向量
                normals = np.asarray(point_cloud_processor.pcd.normals)[idx]
                normal_cam = np.mean(normals, axis=0)
                norm_len = np.linalg.norm(normal_cam)
                if norm_len < 1e-6:
                    continue
                normal_cam /= norm_len
                
                valid_v_info.append({
                    "u": u_curr,
                    "v": v_curr,
                    "pt_cam": pt_cam,
                    "normal_cam": normal_cam,
                    "is_inside": is_inside
                })
            
            if not valid_v_info:
                continue
                
            # 找到当前列中，第一个和最后一个落在有效区域内的索引，进行截断
            inside_indices = [i for i, info in enumerate(valid_v_info) if info["is_inside"]]
            if not inside_indices:
                continue
                
            first_idx = inside_indices[0]
            last_idx = inside_indices[-1]
            trimmed_info = valid_v_info[first_idx : last_idx + 1]
            
            if not is_downward:
                trimmed_info = trimmed_info[::-1]

            line_points = []
            for j, info in enumerate(trimmed_info):
                pt_cam = info["pt_cam"]
                normal_cam = info["normal_cam"]
                is_inside = info["is_inside"]

                # TCP 位置：沿法向后退喷涂距离
                tcp_pos_cam = pt_cam + normal_cam * self.spray_dist_mm
                
                # 基座坐标
                p_base = self.T_base_camera[:3, :3] @ tcp_pos_cam + self.T_base_camera[:3, 3]
                
                # 姿态
                R_cam = self._calculate_rotation_matrix_cam(normal_cam)
                R_base = self.T_base_camera[:3, :3] @ R_cam
                abc_base = R_tool.from_matrix(R_base).as_euler('XYZ', degrees=False)

                line_points.append({
                    "pos": p_base,
                    "abc": abc_base,
                    "spray_on": is_inside,
                    "speed_factor": 1.0 if is_inside else 3.0,
                    "uv": (info["u"], info["v"]),   # 原始扫描像素坐标，供可视化使用
                    "new_line": (j == 0)             # 标记每列的第一个点，用于断开连线
                })
            
            full_trajectory.extend(line_points)
            is_downward = not is_downward
            
        return full_trajectory

    def _calculate_rotation_matrix_cam(self, normal_cam):
        """
        在相机坐标系下计算旋转矩阵。
        使喷枪末端 Z 轴对齐 -normal_cam。
        """
        z_axis = -normal_cam / (np.linalg.norm(normal_cam) + 1e-6)
        # 这里的 X 参考轴可以根据具体安装姿态调整，通常取 [1, 0, 0]
        x_ref = np.array([1.0, 0.0, 0.0])
        y_axis = np.cross(z_axis, x_ref)
        if np.linalg.norm(y_axis) < 1e-3:
            x_ref = np.array([0.0, 1.0, 0.0])
            y_axis = np.cross(z_axis, x_ref)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        
        return np.column_stack((x_axis, y_axis, z_axis))

    def save_path(self, trajectory, file_path="vision/output_path.yaml"):
        """将生成的轨迹保存为 YAML 文件"""
        import yaml
        data = []
        for p in trajectory:
            data.append({
                "pos": p["pos"].tolist(),
                "abc": p["abc"].tolist(),
                "spray_on": bool(p["spray_on"]),
                "speed_factor": float(p["speed_factor"])
            })
        
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        with open(file_path, 'w') as f:
            yaml.dump(data, f)
        print(f"[+] Planner: 轨迹已保存至 {file_path}")

    def visualize_plan(self, image, trajectory, camera_intrinsics, landmarks_2d=None):
        """将 3D 轨迹点投影回 2D 图像进行模拟显示，增加连线、方向箭头以及关键点参考"""
        fx, fy, cx, cy = camera_intrinsics
        vis_img = image.copy()
        
        # 1. 首先画出物理多边形参考边界 (黄色，加粗)
        if self.polygon_pts is not None:
            cv2.polylines(vis_img, [self.polygon_pts.astype(np.int32)], True, (0, 255, 255), 2)
            cv2.putText(vis_img, "PLANNING POLYGON", (int(self.polygon_pts[0,0]), int(self.polygon_pts[0,1]-10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
        # 2. 画出 14 个原始关键点 (红色点 + 编号)
        if landmarks_2d is not None:
            for i, (x, y) in enumerate(landmarks_2d):
                cv2.circle(vis_img, (int(x), int(y)), 5, (0, 0, 255), -1)
                cv2.putText(vis_img, str(i + 1), (int(x), int(y) - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

        # 3. 画出轨迹
        # 直接使用生成路径时记录的原始 2D 像素坐标，避免反投影 TCP 导致偏移
        last_uv = None         # 上一个点，用于列内连线
        last_col_end_uv = None # 上一列的最后一个点，用于绘制列间箭头
        
        for i, p in enumerate(trajectory):
            curr_uv = p.get("uv")
            if curr_uv is None:
                continue
            
            is_new_line = p.get("new_line", False)
            
            if is_new_line:
                # 画列间过渡箭头 (黄色，从上一列末尾指向当前列开头)
                if last_col_end_uv is not None:
                    cv2.arrowedLine(vis_img, last_col_end_uv, curr_uv,
                                    (0, 200, 255), 1, tipLength=0.15)
                last_uv = None  # 列内连线重置，不画跨列灰线
            
            color = (0, 255, 0) if p["spray_on"] else (0, 0, 255)
            
            # 列内连线和方向箭头
            if last_uv is not None and not is_new_line:
                # 深灰色列内连线
                cv2.line(vis_img, last_uv, curr_uv, (90, 90, 90), 1)
                if i % 5 == 0:
                    mid_pt = ((last_uv[0] + curr_uv[0]) // 2, (last_uv[1] + curr_uv[1]) // 2)
                    cv2.arrowedLine(vis_img, last_uv, mid_pt, (180, 180, 180), 1, tipLength=0.4)

            cv2.circle(vis_img, curr_uv, 3, color, -1)
            last_uv = curr_uv
            # 记录每列最后一个点（会被下一个 new_line 覆盖前更新）
            if not is_new_line or last_col_end_uv is None:
                last_col_end_uv = curr_uv
            
        return vis_img

if __name__ == "__main__":
    import sys
    import glob
    # 确保能找到项目根目录下的模块
    sys.path.append(os.getcwd())
    
    from vision.point_cloud_processor import PointCloudProcessor
    from vision.keypoints import TrousersKeypoints
    
    print("\n" + "="*50)
    print(" [Planner 实战验证模式] ")
    print("="*50)

    import argparse
    parser = argparse.ArgumentParser(description="裤子喷涂路径规划器验证工具")
    parser.add_argument("--input_dir", help="指定要处理的数据目录 (如 vision/data/0)")
    parser.add_argument("--width_mm", type=float, default=80.0, help="喷头有效幅宽 (mm), 默认 80")
    parser.add_argument("--overlap", type=float, default=0.2, help="路径重叠率 (0~1), 默认 0.2")
    parser.add_argument("--dist_mm", type=float, default=150.0, help="喷涂法向距离 (mm), 默认 150")
    parser.add_argument("--v_step_mm", type=float, default=20.0, help="纵向采样间隔 (mm), 默认 20")
    args = parser.parse_args()

    # 1. 初始化路径规划器
    calib_file = "calib/data/calib_20260501/calibration_result.yaml"
    planner = AiSprayPlanner(
        spray_width=args.width_mm, 
        overlap=args.overlap, 
        spray_dist=args.dist_mm, 
        v_step_mm=args.v_step_mm,
        calib_path=calib_file
    )
    
    print(f"[*] 参数配置: 幅宽={args.width_mm}mm, 重叠={args.overlap}, 距离={args.dist_mm}mm, 纵向步长={args.v_step_mm}mm")
    
    # 2. 确定采集子目录
    data_dir = "vision/data"
    latest_dir = None

    if args.input_dir:
        latest_dir = args.input_dir
    else:
        # 自动寻找最新的采集子目录 (支持 scan_* 和 自定义命名的目录)
        all_dirs = [os.path.join(data_dir, d) for d in os.listdir(data_dir) 
                    if os.path.isdir(os.path.join(data_dir, d))]
        if all_dirs:
            # 按修改时间排序，取最新的
            all_dirs.sort(key=os.path.getmtime)
            latest_dir = all_dirs[-1]
    
    if latest_dir is None or not os.path.exists(latest_dir):
        print(f"[-] 错误: 未找到有效的数据目录。请运行 capture_pcd.py 生成数据，或使用 --input_dir 指定。")
    else:
        print(f"[*] 目标数据目录: {latest_dir}")

        img_file = os.path.join(latest_dir, "scan.jpg")
        pcd_file = os.path.join(latest_dir, "scan.pcd")
        depth_file = os.path.join(latest_dir, "scan.depth.npy")

        if not (os.path.exists(img_file) and os.path.exists(pcd_file) and os.path.exists(depth_file)):
            print(f"[-] 错误: 目录中文件不完整 (需包含 scan.jpg, scan.pcd, scan.depth.npy): {latest_dir}")
        else:
            # 3. 关键点识别
            print("[*] 正在识别裤子关键点...")
            detector = TrousersKeypoints(model_path="models/model.safetensors")
            detect_res = detector.predict(img_file, threshold=0.8)
            
            if detect_res is None:
                print("[-] 识别失败，未检测到裤子。")
            else:
                # 提取识别到的 14 个关键点像素坐标
                kpts_2d = detect_res['keypoints'][:, :2]
                
                # 4. 加载深度图并映射 3D 关键点
                depth_map = np.load(depth_file)
                img = cv2.imread(img_file)
                planner.set_landmarks_3d(kpts_2d, depth_map, planner.camera_intrinsics, 
                                        image=img, detect_res=detect_res)
                
                # 5. 加载点云处理器
                pc_processor = PointCloudProcessor()
                pc_processor.load_pcd(pcd_file)
                
                # 6. 执行生成
                print("[*] 正在规划轨迹...")
                path = planner.generate_path(pc_processor)
                print(f"[+] 规划完成，共 {len(path)} 个轨迹点")
                
                if path:
                    # 7. 保存结果到同一个目录下
                    out_yaml = os.path.join(latest_dir, "path.yaml")
                    planner.save_path(path, out_yaml)
                    
                    # 8. 可视化关键点识别结果
                    out_kpts = os.path.join(latest_dir, "kpts.png")
                    detector.visualize(img_file, detect_res, out_kpts, polygon=planner.polygon_pts)
                    
                    # 9. 可视化规划轨迹
                    img = cv2.imread(img_file)
                    vis_result = planner.visualize_plan(img, path, planner.camera_intrinsics, kpts_2d)
                    out_vis = os.path.join(latest_dir, "plan.png")
                    cv2.imwrite(out_vis, vis_result)
                    
                    print(f"[+] 结果已全部保存至目录: {latest_dir}")
                    print(f"    - 轨迹: path.yaml")
                    print(f"    - 关键点: kpts.png")
                    print(f"    - 可视化: plan.png")