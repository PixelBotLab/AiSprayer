import numpy as np
import cv2
import os
import yaml
import glob
from aisprayer.core.vision.segmentation import TrousersSegmenter, YoloTrousersSegmenter
from aisprayer.core.vision.point_cloud_processor import PointCloudProcessor
from aisprayer.core.vision.strategies import ZigZagStrategy, BestPathStrategy

class AiSprayPlanner:
    """
    工业级喷涂路径规划器
    负责加载采集好的 Scan 数据，调用识别算法与规划策略，输出最终轨迹。
    """
    def __init__(self, calib_path=None, config=None):
        self.config = config or {
            "spray_width": 80,
            "overlap": 0.2,
            "spray_dist": 150,
            "v_step_mm": 20.0
        }
        self.T_base_camera = np.eye(4)
        self.camera_intrinsics = None # [fx, fy, cx, cy]
        
        # --- 路径锚定策略 ---
        # 核心：无论在哪里执行，都通过 __file__ 找到项目根目录 (AiSprayer/)
        # planner.py 位于 src/aisprayer/core/vision/，向上 4 级到达根目录
        self.project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../"))
        
        print(f"[*] Planner: 正在初始化规划器 (项目根目录: {self.project_root})")
        
        if calib_path:
            # 如果传入的是相对路径，则相对于项目根目录解析
            if not os.path.isabs(calib_path):
                calib_path = os.path.join(self.project_root, calib_path)
            self.load_calibration(calib_path)
        else:
            print(f"[!] Planner: 未加载标定文件，将使用单位阵")
            
        # 2. 加载分割模型
        self.segmenter = TrousersSegmenter()
        self.yolo_segmenter = None
        
        # 优先从配置字典读取模型路径
        model_rel_path = config.get("model_path", "models/wissight.pt")
        yolo_model_path = os.path.join(self.project_root, model_rel_path)
        
        if os.path.exists(yolo_model_path):
            print(f"[+] Planner: 加载 YOLO 模型 -> {yolo_model_path}")
            self.yolo_segmenter = YoloTrousersSegmenter(model_path=yolo_model_path)
        else:
            print(f"[!] Planner: 未发现模型文件 {yolo_model_path}")

    def load_calibration(self, calib_path):
        """加载手眼标定结果"""
        with open(calib_path, 'r') as f:
            res = yaml.safe_load(f)
        self.T_base_camera = np.array(res["T_base_camera"])
        K = np.array(res["camera_params"]["intrinsic_matrix"])
        self.camera_intrinsics = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
        print(f"[+] Planner: 已加载标定参数")

    def plan_garment(self, scan_dir, garment_id="unknown", angle="0"):
        """对指定的裤子视角进行轨迹规划"""
        # 1. 查找数据文件
        color = cv2.imread(os.path.join(scan_dir, "scan.jpg"))
        depth = np.load(os.path.join(scan_dir, "scan.depth.npy"))
        with open(os.path.join(scan_dir, "scan.params.yaml"), 'r') as f:
            params = yaml.safe_load(f)
        
        # 使用 Scan 自带的内参 (如果可用)
        intr = params.get("camera_params", {}).get("intrinsic_matrix", [])
        if len(intr) > 0:
            K = np.array(intr)
            if K.shape == (3, 3):
                current_intr = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
            else:
                current_intr = intr
        else:
            current_intr = self.camera_intrinsics

        # 2. 识别与轮廓提取
        polygon_pts = None
        if self.yolo_segmenter:
            polygon_pts = self.yolo_segmenter.get_silhouette_polygon(color)
        
        if polygon_pts is None:
            print("[!] Planner: YOLO 识别失败，使用默认全图区域 (示例)")
            h, w = color.shape[:2]
            polygon_pts = np.array([[w//4, h//4], [3*w//4, h//4], [3*w//4, 3*h//4], [w//4, 3*h//4]], dtype=np.int32)

        # 3. 初始化点云处理器
        pcd_proc = PointCloudProcessor()
        pcd_proc.load_pcd(os.path.join(scan_dir, "scan.pcd"))

        # 4. 执行规划策略
        strategy = ZigZagStrategy()
        trajectory = strategy.plan(
            pcd_proc, depth, polygon_pts, current_intr, self.T_base_camera, self.config
        )

        # 5. 保存结果到 scan_dir (恢复为 YAML 格式，更具可读性)
        self.save_path(trajectory, scan_dir, garment_id=garment_id, angle=angle)
        self.visualize_plan(color, trajectory, current_intr, polygon_pts, 
                            os.path.join(scan_dir, "plan.png"), 
                            garment_id=garment_id, angle=angle)
        
        return trajectory

    def save_path(self, trajectory, save_dir, garment_id="unknown", angle="0"):
        """将轨迹保存为人类可读的 YAML 格式"""
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, "path.yaml")
        
        # 计算物理参数
        w = self.config.get("spray_width_mm", 100.0)
        ov = self.config.get("overlap_rate", 0.2)
        col_dist = w * (1 - ov)
        v_step = self.config.get("v_step_mm", 20.0)

        data = {
            "metadata": {
                "garment_id": garment_id,
                "angle": angle,
                "spray_width_mm": float(w),
                "overlap_rate": float(ov),
                "inter_column_dist_mm": float(col_dist),
                "intra_column_step_mm": float(v_step),
                "point_count": len(trajectory)
            },
            "points": []
        }
        
        for p in trajectory:
            data["points"].append({
                "pos": p["pos"].tolist(),
                "abc": p["abc"].tolist(),
                "spray_on": bool(p["spray_on"]),
                "speed_factor": float(p["speed_factor"])
            })
        
        with open(save_path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        print(f"[+] Planner: 轨迹已导出至 {save_path}")

    def visualize_plan(self, image, trajectory, intrinsics, polygon_pts, output_path, garment_id="unknown", angle="0"):
        """可视化规划结果：增加物理参数水印"""
        vis_img = image.copy()
        
        # 1. 绘制参数信息水印 (左上角)
        w = self.config.get("spray_width_mm", 100.0)
        ov = self.config.get("overlap_rate", 0.2)
        col_dist = w * (1 - ov)
        v_step = self.config.get("v_step_mm", 20.0)
        
        watermark = [
            f"Garment ID: {garment_id}",
            f"Angle: {angle}",
            f"Inter-Col Dist: {col_dist:.1f} mm",
            f"Intra-Col Step: {v_step:.1f} mm",
            f"Spray Width: {w:.1f} mm",
            f"Points: {len(trajectory)}"
        ]
        for i, text in enumerate(watermark):
            cv2.putText(vis_img, text, (20, 30 + i*20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # 2. 绘制识别区域边界 (绿色框 + 黄色轮廓)
        if polygon_pts is not None and len(polygon_pts) > 0:
            x_min, y_min = np.min(polygon_pts, axis=0)
            x_max, y_max = np.max(polygon_pts, axis=0)
            cv2.rectangle(vis_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (255, 0, 0), 2)
            cv2.putText(vis_img, "Trouser Detection", (int(x_min), int(y_min)-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            
            cv2.polylines(vis_img, [polygon_pts.astype(np.int32)], True, (0, 255, 255), 1)
            # 绘制多边形顶点
            for pt in polygon_pts:
                cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 3, (0, 165, 255), -1)
        
        # 3. 绘制轨迹
        last_uv = None         # 上一个点，用于列内连线
        last_col_end_uv = None # 上一列的最后一个点，用于绘制列间衔接箭头
        
        def draw_fixed_arrow(img, p1, p2, color, thickness=1, tip_len=10):
            """绘制固定尺寸箭头的辅助函数"""
            cv2.line(img, p1, p2, color, thickness)
            angle = np.arctan2(p1[1] - p2[1], p1[0] - p2[0])
            # 绘制箭头两侧
            tp1 = (int(p2[0] + tip_len * np.cos(angle + 0.523)), int(p2[1] + tip_len * np.sin(angle + 0.523)))
            tp2 = (int(p2[0] + tip_len * np.cos(angle - 0.523)), int(p2[1] + tip_len * np.sin(angle - 0.523)))
            cv2.line(img, p2, tp1, color, thickness)
            cv2.line(img, p2, tp2, color, thickness)

        for i, p in enumerate(trajectory):
            curr_uv = tuple(map(int, p["uv"]))
            is_new_line = p.get("new_line", False)
            
            if is_new_line:
                # A. 绘制跨列过渡箭头 (改为深灰色，细线，颜色与纵列内一致)
                if last_col_end_uv is not None:
                    # 使用浅灰色 tip 保证可见度
                    draw_fixed_arrow(vis_img, last_col_end_uv, curr_uv, (80, 80, 80), 1, tip_len=12)
                    draw_fixed_arrow(vis_img, last_col_end_uv, curr_uv, (220, 220, 220), 1, tip_len=12)
                last_uv = None 
            
            # 状态颜色：亮绿 (0,255,0) 或 红色 (0,0,255)
            dot_color = (0, 255, 0) if p["spray_on"] else (0, 0, 255)
            
            # B. 列内连线和方向引导
            if last_uv is not None and not is_new_line:
                cv2.line(vis_img, last_uv, curr_uv, (80, 80, 80), 1)
                
                # 每隔 8 个点绘制一个固定尺寸的浅灰色箭头
                if i % 8 == 0:
                    draw_fixed_arrow(vis_img, last_uv, curr_uv, (220, 220, 220), 1, tip_len=8)

            # C. 绘制执行点 (半径为 4，增强可见性)
            cv2.circle(vis_img, curr_uv, 4, dot_color, -1)
            
            # D. 绘制法线 (红色箭头，从外向里，全量绘制)
            if "pt_cam" in p and "normal_cam" in p:
                pt_cam = p["pt_cam"]
                norm_cam = p["normal_cam"]
                fx, fy, cx, cy = intrinsics
                
                # 计算空气中的起点 (向法向外延 45mm)
                pt_air = pt_cam + norm_cam * 45
                if pt_air[2] > 1e-3:
                    u_air = int((pt_air[0] * fx / pt_air[2]) + cx)
                    v_air = int((pt_air[1] * fy / pt_air[2]) + cy)
                    # 绘制红色箭头 (tipLength 调大)
                    cv2.arrowedLine(vis_img, (u_air, v_air), curr_uv, (0, 0, 255), 1, tipLength=0.4)

            last_uv = curr_uv
            if not is_new_line or last_col_end_uv is None:
                last_col_end_uv = curr_uv
        
        # 3. 标注起始点和终点
        if len(trajectory) > 0:
            start_uv = tuple(map(int, trajectory[0]["uv"]))
            end_uv = tuple(map(int, trajectory[-1]["uv"]))
            
            # 起始点：蓝色实心圆 + 白色 "S"
            cv2.circle(vis_img, start_uv, 8, (255, 120, 0), -1)
            cv2.putText(vis_img, "S", (start_uv[0]-4, start_uv[1]+5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # 终点：紫红色实心圆 + 白色 "E"
            cv2.circle(vis_img, end_uv, 8, (255, 0, 255), -1)
            cv2.putText(vis_img, "E", (end_uv[0]-4, end_uv[1]+5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
        cv2.imwrite(output_path, vis_img)
        print(f"[+] Planner: 预览图已保存至 {output_path}")
