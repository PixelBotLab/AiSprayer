import json
import os
import cv2
import numpy as np
import open3d as o3d
import trimesh
import yaml

# 当前文件位于 src/aisprayer/core/vision/ 下，向上 4 层到达项目根目录 (AiSprayer/)
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
# 默认数据输出目录
DEFAULT_DATA_DIR = os.path.join(PROJECT_ROOT, "data")

def resolve_project_path(path):
    """把相对路径解析为相对项目根目录 (AiSprayer/) 的绝对路径。"""
    if os.path.isabs(path):
        return path
    return os.path.join(PROJECT_ROOT, path)

def k_matrix_to_intrinsics(k):
    """把相机驱动 get_intrinsics() 返回的 3x3 内参矩阵 K 转换为 [fx, fy, cx, cy]"""
    return [k[0, 0], k[1, 1], k[0, 2], k[1, 2]]

def overlay_mask_on_image(image, mask, color=(0, 255, 0), alpha=0.25):
    """把布尔掩码以半透明色块 + 轮廓线的方式叠加到彩色图像上，便于人工观察分割效果。"""
    overlay = image.copy()
    overlay[mask] = color
    blended = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

    # 额外画出掩码轮廓线，增强边界可读性
    mask_uint8 = mask.astype(np.uint8) * 255
    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(blended, contours, -1, color, 2)

    return blended

def depth_to_point_cloud(depth, intrinsics):
    """将深度图转换为与原图对齐的点云网格 [H, W, 3] (相机坐标系，单位 mm)"""
    fx, fy, cx, cy = intrinsics
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    z = depth.astype(np.float32)

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    return np.dstack((x, y, z))


class PoissonReconstructor:
    """
    视觉处理流水线 (泊松重建版)：
    相机采集 (camera) -> YOLO 分割裤子掩码 -> 深度图转点云 -> 
    手眼标定对齐 -> Open3D 泊松曲面重建 (Poisson) -> 导出网格
    """

    def __init__(self, T_camera_to_base, intrinsics_k, segmenter, 
                 z_min: float = 100, z_max: float = 3000,
                 mask_erode_px: int = 1, flying_pixel_max_grad: float = 50.0,
                 mask_alpha: float = 0.25, poisson_depth: int = 9, density_threshold: float = 0.05,
                 voxel_size: float = 0.003, normal_radius: float = 0.03, smooth_iterations: int = 20,
                 **kwargs):
        """
        :param T_camera_to_base: 4x4 手眼标定矩阵 (numpy 数组或二维列表)
        :param intrinsics_k: 3x3 相机内参矩阵
        :param segmenter: 外部传入的分割模型对象
        :param z_min: 有效深度下限 (mm)
        :param z_max: 有效深度上限 (mm)
        :param mask_erode_px: 分割掩码向内腐蚀的像素数，用于排除轮廓边缘的飞点，0 表示不腐蚀
        :param flying_pixel_max_grad: 深度梯度飞点检测阈值 (mm/像素)，超过视为深度突变边缘，0 表示不启用
        :param mask_alpha: 掩码叠加到彩色图时的不透明度 (0~1)
        :param poisson_depth: 泊松重建八叉树深度，越大越精细
        :param density_threshold: 泊松重建密度阈值百分比，用于剔除低密度区域（开洞边界）
        """
        self.T_camera_to_base = np.array(T_camera_to_base)
        self.intrinsics_k = np.array(intrinsics_k) if intrinsics_k is not None else None
        self.segmenter = segmenter
        self.z_min = z_min
        self.z_max = z_max
        self.mask_erode_px = mask_erode_px
        self.flying_pixel_max_grad = flying_pixel_max_grad
        self.mask_alpha = mask_alpha
        self.poisson_depth = poisson_depth
        self.density_threshold = density_threshold
        self.voxel_size = voxel_size
        self.normal_radius = normal_radius
        self.smooth_iterations = smooth_iterations

    @staticmethod
    def _erode_mask(mask_2d, erode_px):
        if erode_px <= 0:
            return mask_2d
        kernel = np.ones((erode_px * 2 + 1, erode_px * 2 + 1), np.uint8)
        return cv2.erode(mask_2d.astype(np.uint8), kernel, iterations=1).astype(bool)

    @staticmethod
    def _flying_pixel_mask(depth_image, max_grad):
        depth = depth_image.astype(np.float32)
        grad_x = cv2.Sobel(depth, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(grad_x ** 2 + grad_y ** 2)
        edge_mask = grad_mag > max_grad
        edge_mask = cv2.dilate(edge_mask.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=1).astype(bool)
        return ~edge_mask

    def from_file(self, color_image_path, depth_image_path, split_parts: int = 1, split_overlap_px: int = 12):
        print("-" * 50)
        print("🔍 [PoissonReconstructor] 泊松重建流水线启动...")
        print("-" * 50)

        color_image = cv2.imread(color_image_path)
        if color_image is None:
            raise RuntimeError(f"无法读取彩色图: {color_image_path}")
            
        try:
            depth_image = np.load(depth_image_path)
        except Exception as e:
            raise RuntimeError(f"无法读取深度图 {depth_image_path}: {e}")

        intrinsics = k_matrix_to_intrinsics(self.intrinsics_k)
        
        if self.segmenter is None:
            raise ValueError("未传入有效的 segmenter 对象")
        yolo_mask_2d = self.segmenter.get_mask(color_image)
        if yolo_mask_2d is None:
            raise RuntimeError("YOLO 分割未识别到有效区域")

        valid_depth_mask = (depth_image > self.z_min) & (depth_image < self.z_max)
        
        # =========================================================
        # 传统计算机视觉：深度图自动补洞 (Inpainting)
        # =========================================================
        # 找到 YOLO 掩码内部的硬件深度破洞 (值为0或越界)
        holes_mask = yolo_mask_2d & (~valid_depth_mask)
        if np.any(holes_mask):
            print(f"[*] 正在使用 OpenCV 图像修复算法填补深度图破洞 (数量: {np.sum(holes_mask)} 像素)...")
            depth_f32 = depth_image.astype(np.float32)
            inpaint_mask = holes_mask.astype(np.uint8) * 255
            # 利用真实的边界流体插值向内修补，保持完美的圆柱体 3D 形状！
            filled_depth = cv2.inpaint(depth_f32, inpaint_mask, inpaintRadius=5, flags=cv2.INPAINT_NS)
            
            # 将修复好的结果直接写回原图的破洞区域
            depth_image[holes_mask] = filled_depth[holes_mask]
            
            # 补洞后，重新计算有效的深度掩码
            valid_depth_mask = (depth_image > self.z_min) & (depth_image < self.z_max)

        # 适度腐蚀掩码，去除轮廓边缘不稳定数据
        eroded_yolo_mask = self._erode_mask(yolo_mask_2d, self.mask_erode_px)
        
        # 深度梯度飞点检测，切除物理相机在边缘产生的杂质毛刺
        flying_pixel_valid = self._flying_pixel_mask(depth_image, self.flying_pixel_max_grad) \
            if self.flying_pixel_max_grad > 0 else np.ones_like(valid_depth_mask, dtype=bool)
            
        combined_mask = eroded_yolo_mask & valid_depth_mask & flying_pixel_valid

        # 现场由深度图和内参还原生成 2.5D 点云网格
        raw_point_cloud = depth_to_point_cloud(depth_image, intrinsics)

        # 把 YOLO 分割掩码叠加到彩色图上，以便返回给调用方保存
        mask_overlay_image = overlay_mask_on_image(color_image, yolo_mask_2d, alpha=self.mask_alpha)

        if split_parts == 1:
            # 整体重建，保持原逻辑，返回单个 mesh
            mesh = self.reconstruct_mesh(raw_point_cloud, combined_mask)
            return mesh, mask_overlay_image
        else:
            # 分腿拆分掩码，防止在接缝处连接不自然
            from aisprayer.core.vision.image2d.jeans_segmentation import split_jeans_mask
            masks = split_jeans_mask(combined_mask, overlap_px=split_overlap_px)
            meshes = []
            for mask in masks:
                meshes.append(self.reconstruct_mesh(raw_point_cloud, mask))
            return meshes, mask_overlay_image

    def reconstruct_mesh(self, raw_point_cloud, yolo_mask_2d):
        """
        raw_point_cloud: 点云 [H, W, 3] (相机坐标系)
        yolo_mask_2d: YOLO 分割掩码对应的点云索引布尔矩阵
        """
        # =========================================================
        # 1. 空间坐标系对齐 (提取掩码点云并单位换算)
        # =========================================================
        if raw_point_cloud.ndim == 3:
            jeans_pts_cam = raw_point_cloud[yolo_mask_2d]
        else:
            raise ValueError("预期 [H, W, 3] 维度的点云。")

        if jeans_pts_cam.shape[0] < 50:
            raise RuntimeError("掩码内有效点数过少，无法重建曲面。")

        jeans_pts_cam = jeans_pts_cam / 1000.0  # mm -> m

        # =========================================================
        # 2. Open3D 点云前处理与法线估计
        # =========================================================
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(jeans_pts_cam)

        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)  # 默认3mm 间距下采样
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=self.normal_radius, max_nn=30)
        )
        pcd.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0]))

        # =========================================================
        # 3. 泊松表面重建
        # =========================================================
        print(f"🚀 正在通过 Poisson 泊松重建生成网格 (depth={self.poisson_depth})...")
        o3d_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=self.poisson_depth
        )

        # 剔除低密度顶点以打开边界 (泊松默认生成封闭曲面)
        densities = np.asarray(densities)
        density_val = np.quantile(densities, self.density_threshold)
        vertices_to_remove = densities < density_val
        
        # 【关键重叠修复】：严苛的距离裁剪
        # 泊松重建会在边界处生成圆滑的“裙边”或者背面闭合曲面，
        # 导致原本在 2D 掩码切分开的两条裤腿在 3D 空间再次膨胀，并在裆部发生物理重叠。
        # 这里强制计算网格所有顶点到原始点云 (pcd) 的绝对物理距离，
        # 将偏离超过 2 倍体素大小（如 6mm）的“虚假外壳”和“膨胀裙边”彻底裁剪掉！
        #mesh_pcd = o3d.geometry.PointCloud()
        #mesh_pcd.points = o3d_mesh.vertices
        #dists = np.asarray(mesh_pcd.compute_point_cloud_distance(pcd))
        #distance_threshold = self.voxel_size * 2.0
        #vertices_to_remove = vertices_to_remove | (dists > distance_threshold)
        
        o3d_mesh.remove_vertices_by_mask(vertices_to_remove)

        # 可选的后处理平滑 (Taubin 平滑不会显著导致收缩)
        if self.smooth_iterations > 0:
            print(f"🧼 正在应用 Taubin 表面平滑 ({self.smooth_iterations} 次迭代)...")
            o3d_mesh = o3d_mesh.filter_smooth_taubin(number_of_iterations=self.smooth_iterations)

        # =========================================================
        # 4. 坐标系转换与后处理
        # =========================================================
        vertices_cam = np.asarray(o3d_mesh.vertices)
        faces = np.asarray(o3d_mesh.triangles)

        if vertices_cam.shape[0] == 0:
            raise RuntimeError("泊松重建未能生成任何顶点，请检查点云。")

        # 转换到基座坐标系
        ones = np.ones((vertices_cam.shape[0], 1))
        vertices_homo = np.hstack((vertices_cam, ones))
        vertices_base = (self.T_camera_to_base @ vertices_homo.T).T[:, 0:3]

        jeans_trimesh = trimesh.Trimesh(vertices=vertices_base, faces=faces)

        # 清除小碎片，只保留最大的连通分量
        components = jeans_trimesh.split(only_watertight=False)
        if len(components) > 1:
            jeans_trimesh = max(components, key=lambda m: len(m.vertices))

        # =========================================================
        # 5. 返回结果
        # =========================================================
        print(f"📊 网格水密性检查 (是否完全封闭): {jeans_trimesh.is_watertight}")
        return jeans_trimesh

    @staticmethod
    def calculate_cut_direction(mesh_or_path):
        """基于 2.5D PCA 自动计算物体在基座 YZ 平面内的偏角"""
        if isinstance(mesh_or_path, str):
            mesh = trimesh.load(mesh_or_path)
        else:
            mesh = mesh_or_path

        vertices = np.asarray(mesh.vertices)
        if len(vertices) == 0:
            return [0.0, 0.0, 1.0]

        coords = vertices[:, [1, 2]]
        mean = coords.mean(axis=0)
        centered = coords - mean
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)

        v0 = eigvecs[:, 0]
        v1 = eigvecs[:, 1]
        main_dir = v0 if abs(v0[0]) > abs(v1[0]) else v1

        if main_dir[0] < 0:
            main_dir = -main_dir

        cut_dir = [0.0, -main_dir[1], main_dir[0]]
        return cut_dir


def poisson_reconstruct_for_surface_walk(raw_cloud_data, yolo_mask_2d, config):
    """全局入口封装"""
    if isinstance(config, dict):
        calib_path = config.get("system", {}).get("calibration_matrix_path", "./config/hand_eye_calibration.json")
        calib_path = resolve_project_path(calib_path)
    else:
        calib_path = config.calibration_matrix_path

    with open(calib_path, 'r') as f:
        calib_data = json.load(f)
    T_cam_to_base = calib_data['T_camera_to_base']

    reconstructor = PoissonReconstructor(
        T_camera_to_base=T_cam_to_base,
        intrinsics_k=None,
        segmenter=None
    )
    return reconstructor.reconstruct_mesh(raw_cloud_data, yolo_mask_2d)


# =========================================================
# 单元测试入口
# =========================================================
if __name__ == "__main__":
    import argparse
    import sys
    
    CORE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if CORE_DIR not in sys.path:
        sys.path.insert(0, CORE_DIR)
        
    from aisprayer.core.vision.image2d.segmenter import SegmenterFactory
    from config import SprayerConfig

    parser = argparse.ArgumentParser(description="视觉处理流水线 (泊松版) - 离线测试")
    parser.add_argument("--scan_dir", default=None, help="扫描数据目录")
    parser.add_argument("--config", default="configs/aisprayer_config.yaml", help="配置文件路径")
    parser.add_argument("--conf", type=float, default=0.5, help="YOLO 置信度")
    parser.add_argument("--show", action="store_true", help="是否在输出后显示重建效果")
    parser.add_argument("--segmenter", type=str, default="yolo_trousers", choices=["yolo_trousers", "sam3.1"],
                        help="选择使用的分割器引擎 (YOLO 或 SAM3.1)")
    parser.add_argument("--split_parts", type=int, default=1, choices=[1, 2], help="重建成1个整体还是2个(分腿)")
    parser.add_argument("--split_overlap_px", type=int, default=12, help="分腿时的重叠像素")
    args = parser.parse_args()

    sprayer_config = SprayerConfig(args.config)
    scan_dir = args.scan_dir
    if not scan_dir:
        output_root = sprayer_config._resolve_path(sprayer_config.output_root)
        subdirs = [os.path.join(output_root, d) for d in os.listdir(output_root) if os.path.isdir(os.path.join(output_root, d))]
        try:
            scan_dir = max(subdirs, key=lambda x: int(os.path.basename(x)))
        except ValueError:
            scan_dir = max(subdirs, key=os.path.getmtime)
        print(f"[*] 使用最新扫描目录: {scan_dir}")
    
    if args.segmenter == "yolo_trousers":
        segmenter = SegmenterFactory.create("yolo_trousers", model_path=sprayer_config.model_path, conf=args.conf)
    elif args.segmenter == "sam3.1":
        # 用户指定的 SAM3.1 分割器
        segmenter = SegmenterFactory.create("sam3.1", model_path="models/sam3.1.pt", conf=args.conf)
    else:
        segmenter = SegmenterFactory.create(args.segmenter, conf=args.conf)

    color_path = os.path.join(scan_dir, "scan.jpg")
    depth_path = os.path.join(scan_dir, "scan.depth.npy")

    intrinsics_k = None
    try:
        with open(os.path.join(scan_dir, "scan.params.yaml"), "r") as f:
            params = yaml.safe_load(f)
            k_list = params.get("camera_params", {}).get("intrinsic_matrix")
            if k_list:
                intrinsics_k = np.array(k_list)
    except Exception:
        pass

    reconstructor = PoissonReconstructor(
        T_camera_to_base=sprayer_config.T_camera_to_base,
        intrinsics_k=intrinsics_k,
        segmenter=segmenter,
    )

    result = reconstructor.from_file(
        color_image_path=color_path,
        depth_image_path=depth_path,
        split_parts=args.split_parts,
        split_overlap_px=args.split_overlap_px
    )
    
    # 根据 split_parts 的值，返回值可能是 (mesh, image) 或者是 (meshes_list, image)
    if args.split_parts == 1:
        meshes = [result[0]]
        mask_overlay_image = result[1]
    else:
        meshes = result[0]
        mask_overlay_image = result[1]

    output_dir = os.path.join(scan_dir, "output")
    os.makedirs(output_dir, exist_ok=True)
    
    # 外部保存掩码图像
    mask_overlay_path = os.path.join(output_dir, "captured_color_mask.jpg")
    cv2.imwrite(mask_overlay_path, mask_overlay_image)
    print(f"🎭 掩码标记图已导出为: {mask_overlay_path}")

    # 外部保存 3D 网格
    output_paths = []
    if len(meshes) == 1:
        output_obj_path = os.path.join(output_dir, "reconstructed.obj")
        meshes[0].export(output_obj_path)
        output_paths.append(output_obj_path)
    else:
        for i, mesh in enumerate(meshes):
            output_obj_path = os.path.join(output_dir, f"{i + 1}.obj")
            mesh.export(output_obj_path)
            output_paths.append(output_obj_path)
    print(f"✓ 测试完成，网格已导出为: {', '.join(output_paths)}")

    if args.show:
        print("[*] 正在准备可视化重建网格...")
        o3d_meshes = [o3d.io.read_triangle_mesh(p) for p in output_paths]
        mesh = o3d_meshes[0]
        for m in o3d_meshes[1:]:
            mesh += m
        mesh.compute_vertex_normals()
        
        color_img = cv2.imread(color_path)
        t_cam_to_base = np.array(sprayer_config.T_camera_to_base)
        
        if color_img is not None and intrinsics_k is not None and t_cam_to_base is not None:
            t_base_to_cam = np.linalg.inv(t_cam_to_base)
            vertices = np.asarray(mesh.vertices)
            fx, fy, cx, cy = intrinsics_k[0, 0], intrinsics_k[1, 1], intrinsics_k[0, 2], intrinsics_k[1, 2]
            
            ones = np.ones((vertices.shape[0], 1))
            verts_homo = np.hstack((vertices, ones))
            verts_cam = (t_base_to_cam @ verts_homo.T).T[:, :3]
            
            x, y, z = verts_cam[:, 0], verts_cam[:, 1], verts_cam[:, 2]
            valid = z > 1e-6
            u = np.zeros_like(x)
            v = np.zeros_like(y)
            u[valid] = fx * x[valid] / z[valid] + cx
            v[valid] = fy * y[valid] / z[valid] + cy
            
            h, w = color_img.shape[:2]
            ui = np.clip(np.round(u).astype(int), 0, w - 1)
            vi = np.clip(np.round(v).astype(int), 0, h - 1)
            valid &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
            
            bgr = color_img[vi, ui].astype(np.float32) / 255.0
            colors = bgr[:, ::-1].copy()
            colors[~valid] = np.array([0.6, 0.6, 0.6])
            
            mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
        else:
            mesh.paint_uniform_color([0.75, 0.75, 0.82])
            
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        
        # 创建地面网格 (2米 x 2米, 10cm一格)
        def create_grid(size=2.0, n=20, color=[0.6, 0.6, 0.6]):
            points = []
            lines = []
            half = size / 2.0
            step = size / n
            for i in range(n + 1):
                pos = -half + i * step
                # 平行于X轴的线
                points.append([-half, pos, 0.0])
                points.append([half, pos, 0.0])
                lines.append([len(points)-2, len(points)-1])
                # 平行于Y轴的线
                points.append([pos, -half, 0.0])
                points.append([pos, half, 0.0])
                lines.append([len(points)-2, len(points)-1])
                
            grid = o3d.geometry.LineSet()
            grid.points = o3d.utility.Vector3dVector(points)
            grid.lines = o3d.utility.Vector2iVector(lines)
            grid.colors = o3d.utility.Vector3dVector([color for _ in range(len(lines))])
            return grid
            
        ground_grid = create_grid()
        
        print("[*] 打开交互式 3D 渲染窗口 (Open3D GUI)...")
        o3d.visualization.draw_geometries(
            [mesh, coord_frame, ground_grid],
            window_name="AiSprayer 泊松重建 3D 预览",
            width=1280,
            height=800,
        )
