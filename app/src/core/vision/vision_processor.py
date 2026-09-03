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
    """把相对路径解析为相对项目根目录 (AiSprayer/) 的绝对路径。
    
    这样无论脚本从哪个工作目录被执行，相对路径都能被正确找到，不用依赖运行时的 cwd。
    """
    if os.path.isabs(path):
        return path
    return os.path.join(PROJECT_ROOT, path)


def k_matrix_to_intrinsics(k):
    """把相机驱动 get_intrinsics() 返回的 3x3 内参矩阵 K 转换为 [fx, fy, cx, cy]"""
    return [k[0, 0], k[1, 1], k[0, 2], k[1, 2]]


def overlay_mask_on_image(image, mask, color=(0, 255, 0), alpha=0.25):
    """把布尔掩码以半透明色块 + 轮廓线的方式叠加到彩色图像上，便于人工观察分割效果。

    :param image: HxWx3 BGR 彩色图
    :param mask: HxW 布尔掩码，需与 image 分辨率一致
    :param color: 叠加颜色 (BGR)，默认绿色
    :param alpha: 色块叠加不透明度 (0~1)，值越小掩码区域颜色越淡、底图越清晰可见；
        默认调低到 0.25，避免完全遮住掩码区域内的原图内容 (比如裤子本身的纹理)
    :return: 叠加后的新图像 (不修改传入的原图)
    """
    overlay = image.copy()
    overlay[mask] = color
    blended = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

    # 额外画出掩码轮廓线，增强边界可读性
    mask_uint8 = mask.astype(np.uint8) * 255
    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(blended, contours, -1, color, 2)

    return blended


def depth_to_point_cloud(depth, intrinsics):
    """将深度图转换为与原图对齐的点云网格 [H, W, 3] (相机坐标系，单位 mm)

    保留原始 (H, W) 形状而不做有效性过滤，这样可以直接用 YOLO 输出的
    HxW 布尔掩码 (yolo_mask_2d) 索引。

    :param depth: HxW uint16/float 深度图 (mm)
    :param intrinsics: [fx, fy, cx, cy]
    """
    fx, fy, cx, cy = intrinsics
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    z = depth.astype(np.float32)

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    return np.dstack((x, y, z))


class VisionProcessor:
    """
    牛仔裤视觉处理流水线：

    相机采集 (camera) -> YOLO 分割裤子掩码 ->
    深度图转点云 -> 手眼标定对齐 -> Open3D 滚球法 (BPA) 网格重建 ->
    Trimesh 拉普拉斯平滑 -> 导出网格 (供 SurfaceWalk 提取自由边缘)
    """

    def __init__(self, T_camera_to_base, intrinsics_k, segmenter, 
                 smoothing_iterations: int = 15, z_min: float = 100, z_max: float = 3000,
                 mask_erode_px: int = 1, flying_pixel_max_grad: float = 50.0,
                 split_overlap_px: int = 12, mask_alpha: float = 0.25):
        """
        :param T_camera_to_base: 4x4 手眼标定矩阵 (numpy 数组或二维列表)
        :param intrinsics_k: 3x3 相机内参矩阵
        :param segmenter: 外部传入的分割模型对象
        :param smoothing_iterations: Taubin 平滑的迭代次数，默认 15
        :param z_min: 有效深度下限 (mm)
        :param z_max: 有效深度上限 (mm)
        :param mask_erode_px: 分割掩码向内腐蚀的像素数，用于排除轮廓边缘的飞点，0 表示不腐蚀
        :param flying_pixel_max_grad: 深度梯度飞点检测阈值 (mm/像素)，超过视为深度突变边缘，0 表示不启用
        :param split_overlap_px: 裤裆分腿时两侧掩码在切线处的重叠宽度（像素）
        :param mask_alpha: 掩码叠加到彩色图时的不透明度 (0~1)
        """
        self.T_camera_to_base = np.array(T_camera_to_base)
        self.intrinsics_k = np.array(intrinsics_k) if intrinsics_k is not None else None
        self.segmenter = segmenter
        self.smoothing_iterations = smoothing_iterations
        self.z_min = z_min
        self.z_max = z_max
        self.mask_erode_px = mask_erode_px
        self.flying_pixel_max_grad = flying_pixel_max_grad
        self.split_overlap_px = split_overlap_px
        self.mask_alpha = mask_alpha


    @staticmethod
    def _erode_mask(mask_2d, erode_px):
        """把分割掩码向内腐蚀几个像素。

        深度相机 (尤其结构光/主动双目) 在物体轮廓边缘上极易产生"飞点"(flying pixel)：
        像素同时"骑"在物体和背景两个不同深度上，测出一个夹在两者之间、物理上并不
        存在的错误深度值。这些飞点几乎全部集中在分割掩码边界附近的一圈像素里，
        直接把掩码向内缩几个像素，从源头上把这一圈易出错区域排除掉，
        比事后再用统计离群点剔除更精准（离群点剔除是按整体分布判断，对贴着
        真实表面分布的飞点很不敏感）。
        """
        if erode_px <= 0:
            return mask_2d
        kernel = np.ones((erode_px * 2 + 1, erode_px * 2 + 1), np.uint8)
        return cv2.erode(mask_2d.astype(np.uint8), kernel, iterations=1).astype(bool)

    @staticmethod
    def _remove_non_manifold_faces(mesh):
        """剔除所有"非流形边"所在的三角面：既包括被 3 个及以上三角面共享的边
        (无向边计数 > 2)，也包括看似正常 (无向计数为 2) 但两个面对该边的绕向
        完全相同 (有向边出现了重复) 的情况。

        后一种情况是踩坑排查出来的一个隐蔽陷阱：滚球算法拼接不同区域的面片时
        不保证全局绕向一致，导致同一条有向边 (v0,v1) 被两个面同方向各贡献一次，
        而不是一去一回 (v0,v1) + (v1,v0)。trimesh 的 Laplacian/Taubin 平滑在
        构造邻接权重矩阵时用的是布尔型稀疏矩阵，合并这种"方向相同的重复有向边"
        时会静默地把重复次数坍缩成 1 次，导致该顶点的归一化权重之和 < 1——由于
        网格顶点坐标是机器人基座系下的绝对坐标 (离原点 0.3~0.7m)，这个权重缺口
        会被放大成几十到几百毫米的顶点跳变，也就是看到的"拉丝毛刺"，而且是平滑
        次数越多、暴露越多，并非滚球重建或分割掩码的问题。

        :return: (清理后的新 Trimesh 对象, 被剔除的三角面数量)
        """
        faces = mesh.faces
        n_faces = len(faces)

        # 无向边计数 > 2：真正意义上的非流形边 (3+ 个面共享同一条边)
        undirected = np.vstack([
            np.sort(faces[:, [0, 1]], axis=1),
            np.sort(faces[:, [1, 2]], axis=1),
            np.sort(faces[:, [2, 0]], axis=1),
        ])
        face_of_edge = np.tile(np.arange(n_faces), 3)
        _, inv_u, cnt_u = np.unique(undirected, axis=0, return_inverse=True, return_counts=True)
        bad_undirected = np.isin(inv_u, np.where(cnt_u > 2)[0])

        # 有向边重复：绕向相同的重复边 (触发 trimesh 布尔稀疏矩阵坍缩 bug 的元凶)
        directed = np.vstack([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
        _, inv_d, cnt_d = np.unique(directed, axis=0, return_inverse=True, return_counts=True)
        bad_directed = np.isin(inv_d, np.where(cnt_d > 1)[0])

        bad_mask = bad_undirected | bad_directed
        if not bad_mask.any():
            return mesh, 0

        bad_faces = np.unique(face_of_edge[bad_mask])
        keep_mask = np.ones(n_faces, dtype=bool)
        keep_mask[bad_faces] = False

        # process=False：这里只是单纯删面，不需要 trimesh 顺手再做一次顶点合并
        # (merge_vertices)。顶点合并本身没问题，但它会悄悄改变部分面引用的顶点
        # id，等于在我们刚清理干净的拓扑上又重新组合出一批新面——经实测这可能
        # 把原本没问题的边重新变成重复/非流形边，等于清理了个寂寞。真要合并
        # 顶点应该在整个清理流程最前面显式做一次，而不是在每次局部删面时都
        # 隐式触发。
        cleaned = trimesh.Trimesh(vertices=mesh.vertices, faces=faces[keep_mask], process=False)
        cleaned.remove_unreferenced_vertices()
        return cleaned, len(bad_faces)

    @staticmethod
    def _fill_small_holes(mesh, max_hole_edges=10):
        """只把"小洞"补上，真正的大洞 (裤子腰部/裤脚的真实开口边界) 原样保留。

        前面几步 (非流形边修复、拉丝毛刺过滤) 为了拓扑干净，代价是会把局部一小
        撮三角面整体挖掉，副作用是网格中间会多出一些"镂空"小洞——这些洞是清理
        产生的副作用，不是裤子本身真实的开口，应该补上，否则会比之前的拉丝
        毛刺更显眼 (一片片破洞)。

        但不能直接用 trimesh.repair.fill_holes() 无脑把所有边界洞都填平：
        SurfaceWalk 规划依赖网格保留真实的自由边缘 (腰部/裤脚开口) 来做向内
        等高线收缩，如果把这些真实大洞也焊死，网格就变成水密的了，SurfaceWalk
        直接没法用。这里参考 trimesh.repair.fill_holes 的实现，自己按"洞口边数"
        做一次过滤：只填补边数 <= max_hole_edges 的小洞 (清理产生的局部小坑
        基本上就三五条边)，边数明显更多的洞视为真实轮廓边界，原样跳过不动。

        :return: (处理后的 mesh, 补上的小洞数量)
        """
        import networkx as nx
        from trimesh.geometry import faces_to_edges, triangulate_quads
        from trimesh.grouping import group_rows, hashable_rows

        if mesh.is_watertight:
            return mesh, 0

        boundary_groups = group_rows(mesh.edges_sorted, require_count=1)
        if len(boundary_groups) < 3:
            return mesh, 0

        boundary = mesh.edges[boundary_groups]
        all_holes = nx.cycle_basis(nx.from_edgelist(boundary))
        small_holes = [h for h in all_holes if len(h) <= max_hole_edges]
        if not small_holes:
            return mesh, 0

        new_faces = triangulate_quads(small_holes)
        if len(new_faces) == 0:
            return mesh, 0

        # 新面片的绕向要跟原网格保持一致：如果新面的某条有向边跟洞口边界的
        # 有向边完全同向 (而不是正常应有的"首尾相反")，说明这个面绕反了，翻过来。
        new_edges = faces_to_edges(new_faces)
        hashable_new = hashable_rows(new_edges)
        hashable_old = hashable_rows(boundary)
        needs_reverse = np.isin(hashable_new, hashable_old).reshape((-1, 3)).any(axis=1)
        new_faces[needs_reverse] = np.fliplr(new_faces[needs_reverse])

        # --- 过滤新面片，避免引入任何非流形边 (无向边数量 >= 3 或有向边重复) ---
        from collections import defaultdict
        edge_counts = defaultdict(int)
        directed_counts = defaultdict(int)

        # 统计原网格中的所有边
        mesh_edges_sorted = np.sort(mesh.edges, axis=1)
        for edge in mesh_edges_sorted:
            edge_counts[tuple(edge)] += 1
        for edge in mesh.edges:
            directed_counts[tuple(edge)] += 1

        accepted_faces = []
        for face in new_faces:
            f_edges = [
                tuple(sorted([face[0], face[1]])),
                tuple(sorted([face[1], face[2]])),
                tuple(sorted([face[2], face[0]]))
            ]
            f_directed = [
                (face[0], face[1]),
                (face[1], face[2]),
                (face[2], face[0])
            ]
            # 如果任何一条边在加入当前面后，无向共享面数 >= 3，或有向同向数 >= 2，则跳过
            if any(edge_counts[e] >= 2 for e in f_edges):
                continue
            if any(directed_counts[de] >= 1 for de in f_directed):
                continue

            # 接受该面，并更新边计数
            for e in f_edges:
                edge_counts[e] += 1
            for de in f_directed:
                directed_counts[de] += 1
            accepted_faces.append(face)

        if len(accepted_faces) == 0:
            return mesh, 0

        new_faces = np.array(accepted_faces)
        # -------------------------------------------------------------

        mesh.extend_faces(new_faces)
        return mesh, len(small_holes)

    @classmethod
    def _repair_non_manifold_until_stable(cls, mesh, max_iterations=10):
        """反复调用 _remove_non_manifold_faces 直到没有非流形边为止再返回。

        单趟清理有时不够彻底：删掉一批坏面后，剩下网格的顶点/面引用关系变化，
        可能会在原本正常的位置暴露出"次生"非流形边 (比如某条边原来被 3 个面
        共享，删掉其中 1 个坏面后看起来正常了，但同一批删除操作在别处又让
        另一条边意外变成了非流形)。多跑几轮直到收敛 (不再有新的非流形面被
        发现) 才能真正保证喂给 Taubin 平滑的网格是"干净"的。
        """
        total_removed = 0
        for _ in range(max_iterations):
            mesh, n_removed = cls._remove_non_manifold_faces(mesh)
            total_removed += n_removed
            if n_removed == 0:
                break
        return mesh, total_removed

    @staticmethod
    def _flying_pixel_mask(depth_image, max_grad):
        """基于深度图梯度幅值检测"飞点"：只要某像素跟邻域的深度差过大 (深度突变)，
        就认为该像素及其近邻处在深度不连续边界上，很可能是飞点，标记为无效。

        跟 _erode_mask 是互补关系：_erode_mask 只处理分割掩码的外轮廓，
        而衣物内部的褶皱/自遮挡边界也会产生同样的深度突变飞点，
        这里用梯度检测能覆盖到分割掩码内部的这类边界，而不仅仅是外轮廓。
        """
        depth = depth_image.astype(np.float32)
        grad_x = cv2.Sobel(depth, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(grad_x ** 2 + grad_y ** 2)
        edge_mask = grad_mag > max_grad
        # 飞点通常紧贴在深度突变边缘的一两个像素范围内，膨胀一圈把这些近邻也一并排除
        edge_mask = cv2.dilate(edge_mask.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=1).astype(bool)
        return ~edge_mask

    def process_scan_data(self, color_image_path, depth_image_path, output_dir):
        """处理给定的一帧彩色图、深度图，生成裤腿区域点云，并执行滚球重建 + 拉普拉斯平滑

        :param color_image_path: 彩色图文件路径 (比如 scan.jpg)
        :param depth_image_path: 深度图文件路径 (比如 scan.depth.npy，存储 uint16 或 float)
        :param output_dir: 采集到的彩色图与重建网格的保存目录
        :return: 平滑网格导出路径 (多条裤腿的路径以逗号分隔)
        """
        print("-" * 50)
        print("🔍 [VisionProcessor] 当前视觉处理参数配置:")
        print(f"  - 输入彩色图: {color_image_path}")
        print(f"  - 输入深度图: {depth_image_path}")
        print(f"  - 输出根目录: {output_dir}")
        print(f"  - 相机内参 (K):\n{np.array2string(self.intrinsics_k, prefix='      ') if self.intrinsics_k is not None else '    [未提供，使用默认默认值]'}")
        print(f"  - 手眼标定矩阵 (T_camera_to_base):\n{np.array2string(self.T_camera_to_base, prefix='      ')}")
        print(f"  - 深度有效范围: {self.z_min}mm ~ {self.z_max}mm")
        print(f"  - 掩码边缘腐蚀: {self.mask_erode_px} 像素")
        print(f"  - 飞点检测梯度: {self.flying_pixel_max_grad} mm/px")
        print(f"  - 裤腿拆分重叠: {self.split_overlap_px} 像素")
        print(f"  - Taubin 平滑: {self.smoothing_iterations} 次迭代")
        print("-" * 50)

        color_image = cv2.imread(color_image_path)
        if color_image is None:
            raise RuntimeError(f"无法读取彩色图: {color_image_path}")
            
        try:
            depth_image = np.load(depth_image_path)
        except Exception as e:
            raise RuntimeError(f"无法读取深度图 {depth_image_path}: {e}")

        intrinsics = k_matrix_to_intrinsics(self.intrinsics_k)
        
        # 现场由深度图和内参还原生成 2.5D 点云网格
        raw_point_cloud = depth_to_point_cloud(depth_image, intrinsics)

        if self.segmenter is None:
            raise ValueError("构造 VisionProcessor 时必须传入有效的 segmenter 对象，以便推理掩码")
        yolo_mask_2d = self.segmenter.get_mask(color_image)
        if yolo_mask_2d is None:
            raise RuntimeError("YOLO 分割未识别到裤子区域")

        # 深度量程有效性掩码与 YOLO 分割掩码取交集，过滤背景/噪点
        valid_depth_mask = (depth_image > self.z_min) & (depth_image < self.z_max)

        # 分割掩码向内腐蚀几个像素，排除轮廓边缘的"飞点" (flying pixel)；
        # 再叠加深度梯度飞点检测，进一步排除衣物内部褶皱/自遮挡处的深度突变飞点。
        # 这两类飞点如果混进点云，滚球算法会把它们错误地"架桥"连到主表面上，
        # 形成一条条细长的拉丝毛刺——这是目前看到的毛刺主因，比噪点更难靠
        # 统计离群点剔除清理干净，必须在生成点云之前就从源头排除。
        eroded_yolo_mask = self._erode_mask(yolo_mask_2d, self.mask_erode_px)
        flying_pixel_valid = self._flying_pixel_mask(depth_image, self.flying_pixel_max_grad) \
            if self.flying_pixel_max_grad > 0 else np.ones_like(valid_depth_mask, dtype=bool)
        combined_mask = eroded_yolo_mask & valid_depth_mask & flying_pixel_valid
        print(f"🧹 边缘飞点过滤：掩码腐蚀 {self.mask_erode_px}px + 深度梯度阈值 {self.flying_pixel_max_grad}mm/px，"
              f"有效点数 {yolo_mask_2d.sum()} -> {combined_mask.sum()}")

        os.makedirs(output_dir, exist_ok=True)

        # 把 YOLO 分割掩码叠加到彩色图上另存一份，方便直接用看图工具检查分割是否准确
        mask_overlay_image = overlay_mask_on_image(color_image, yolo_mask_2d, alpha=self.mask_alpha)
        mask_overlay_path = os.path.join(output_dir, "captured_color_mask.jpg")
        cv2.imwrite(mask_overlay_path, mask_overlay_image)
        print(f"🎭 掩码标记图已保存 (alpha={self.mask_alpha}): {mask_overlay_path}")

        # 两条裤腿应分别重建、分别规划：每条腿有各自的最长主轴，合并成整条裤子后只做
        # 一次 PCA 会丢失这种局部方向信息。分割掩码在切线处保留重叠带，避免两个独立
        # BPA 网格都在边界收缩而形成中缝空白。
        from core.vision.image2d.jeans_segmentation import split_jeans_mask
        masks = split_jeans_mask(combined_mask, overlap_px=self.split_overlap_px)
        output_paths = []

        if len(masks) == 1:
            out_path = self.filter_and_smooth_jeans(
                raw_point_cloud, masks[0], output_dir=output_dir, output_name="1.obj"
            )
            output_paths.append(out_path)
        else:
            for i, mask in enumerate(masks):
                out_path = self.filter_and_smooth_jeans(
                    raw_point_cloud, mask, output_dir=output_dir, output_name=f"{i + 1}.obj"
                )
                output_paths.append(out_path)

        return ",".join(output_paths)

    def filter_and_smooth_jeans(self, raw_point_cloud, yolo_mask_2d,
                                 output_dir=DEFAULT_DATA_DIR, output_name="jeans_smoothed.obj"):
        """
        raw_point_cloud: 奥比中光 336L 原始采集到的点云 [N, 3] 或 [H, W, 3] (相机坐标系)
        yolo_mask_2d: YOLOv8-Segment 提取出的裤腿像素掩码对应的点云索引布尔矩阵
        output_dir: 重建网格的保存目录，默认 backend/data/
        output_name: 重建网格的文件名
        """
        # =========================================================
        # 1. 空间坐标系对齐 (手眼标定矩阵应用)
        # =========================================================
        T_cam_to_base = self.T_camera_to_base

        # 利用 YOLO 掩码裁剪背景，仅保留牛仔裤区域的 3D 点
        if raw_point_cloud.ndim == 3:
            jeans_pts_cam = raw_point_cloud[yolo_mask_2d]
        else:
            raise ValueError(f"预期 [H, W, 3] 维度的点云以便与 2D 掩码对齐，但收到形状: {raw_point_cloud.shape}。若是 [N, 3] 格式，请先恢复到 2.5D 结构。")

        if jeans_pts_cam.shape[0] < 50:
            raise RuntimeError(
                f"掩码内有效点数过少 ({jeans_pts_cam.shape[0]} 个)，无法重建曲面。"
                "请检查 YOLO 分割掩码与深度图是否对齐、深度采集范围 (z_min/z_max) 是否覆盖了实际拍摄距离。"
            )

        # depth_to_point_cloud 输出的相机系点云单位是毫米 (mm)，
        # 而手眼标定矩阵 T_camera_to_base 以及下面 Open3D 的 voxel_size/法线搜索半径
        # 都是按"米"设计的，这里必须先换算单位，否则法线估计的搜索半径相对点间距会小
        # 几个数量级，导致邻域搜不到点 -> 法线归一化时 0/0 产生 NaN -> 滚球重建整张网格顶点全炸。
        jeans_pts_cam = jeans_pts_cam / 1000.0  # mm -> m

        # =========================================================
        # 2. Open3D 点云前处理与高精法线估计
        # =========================================================
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(jeans_pts_cam)

        # 体素下采样，保持点云密度均匀，防止某些区域太密导致重构产生多余碎面
        pcd = pcd.voxel_down_sample(voxel_size=0.003)  # 3mm 间距

        # 统计学离群点剔除：深度相机在深色/反光布料区域容易产生零星噪声点，
        # 这些离群点混在正常点之间会让滚球算法在离群点周围"搭桥"，生成又细又长的
        # "拉丝毛刺" 三角面片。剔除掉这些噪声点能从源头上大幅减少毛刺的产生。
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # 估计表面法线，这是滚球算法的核心输入
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30)
        )
        # 强行让法线统一朝向相机原点，完全避免标定系下硬编码相机位置导致的法线反转与缺片
        pcd.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0]))

        # =========================================================
        # 3. 泊松曲面重建 (Poisson Surface Reconstruction)
        # =========================================================
        # 用泊松重建替代滚球算法 (BPA)。
        # 核心优势：泊松重建以上一步估计好的点云法向量作为直接输入，求解一个指示函数
        # (indicator function) 使得重建曲面的法向场与输入法向场最吻合。
        # 这样得到的网格法向量在物理上是真正垂直于表面的，不依赖网格拓扑推算。
        # BPA 会产生拓扑不规则、法向量不准确的问题，泊松重建从根源上解决这个问题。
        print("🚀 正在通过泊松重建生成平滑网格...")
        o3d_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd,
            depth=9,        # 八叉树深度，控制重建精细度 (8~10 适合本场景)
            width=0,
            scale=1.1,
            linear_fit=False
        )

        # 泊松重建是闭合曲面算法，会在裤腰/裤脚等开口处自动补出一个"外壳"。
        # 通过顶点密度过滤（低密度顶点 = 该位置没有原始点云数据支撑，是人工外壳）
        # 裁剪掉这层外壳，恢复裤子真实的开口边界。
        densities_np = np.asarray(densities)
        density_threshold = np.quantile(densities_np, 0.05)  # 去掉最低 5% 密度的顶点
        vertices_to_remove = densities_np < density_threshold
        
        # 【关键重叠修复】：严苛的距离裁剪
        # 泊松重建会在分腿边界处生成圆滑的“裙边”或者背面闭合曲面，
        # 导致原本切分开的两条裤腿在 3D 空间再次向外膨胀，在中间发生物理重叠。
        # 我们计算生成网格的所有顶点到原始输入点云 (pcd) 的绝对物理距离，
        # 将偏离点云超过 6mm 的“虚假外壳”和“膨胀裙边”彻底裁剪掉！
        mesh_pcd = o3d.geometry.PointCloud()
        mesh_pcd.points = o3d_mesh.vertices
        dists = np.asarray(mesh_pcd.compute_point_cloud_distance(pcd))
        distance_threshold = 0.006  # 6mm
        vertices_to_remove = vertices_to_remove | (dists > distance_threshold)
        
        o3d_mesh.remove_vertices_by_mask(vertices_to_remove)
        print(f"🧹 泊松边界与重叠裁剪：去除低密度及膨胀外壳顶点 {vertices_to_remove.sum()}/{len(densities_np)} "
              f"(密度阈值={density_threshold:.4f}, 距离阈值={distance_threshold*1000:.1f}mm)")

        # =========================================================
        # 4. Trimesh 接入与拉普拉斯"烫平"布料褶皱
        # =========================================================
        # 将 Open3D 对象转为数据结构更适合拓扑操作的 Trimesh 对象
        vertices_cam = np.asarray(o3d_mesh.vertices)
        faces = np.asarray(o3d_mesh.triangles)

        if vertices_cam.shape[0] == 0:
            raise RuntimeError("泊松重建未能重建出任何顶点，请检查点云密度/法线质量是否正常。")

        # 将重建后的网格顶点从相机坐标系变换到机器人基座坐标系下
        ones = np.ones((vertices_cam.shape[0], 1))
        vertices_homo = np.hstack((vertices_cam, ones))
        vertices_base = (T_cam_to_base @ vertices_homo.T).T[:, 0:3]

        if not np.isfinite(vertices_base).all():
            n_bad = (~np.isfinite(vertices_base).all(axis=1)).sum()
            raise RuntimeError(
                f"泊松重建转换后有 {n_bad}/{vertices_base.shape[0]} 个顶点坐标是 NaN/Inf，"
                "请检查标定矩阵 T_camera_to_base 是否包含异常值。"
            )

        jeans_trimesh = trimesh.Trimesh(vertices=vertices_base, faces=faces)

        # 泊松重建输出的是干净流形网格，一般不会有非流形边，但仍做一次兜底清理。
        jeans_trimesh, n_nonmanifold = self._repair_non_manifold_until_stable(jeans_trimesh)
        if n_nonmanifold:
            print(f"🧹 非流形拓扑修复：剔除了 {n_nonmanifold} 个非流形边/重复面所在的三角面")

        # 滚球算法在点云噪声/密度不均的区域，经常会额外生成一些跟主体裤子网格
        # 不连通的小碎片 (就是看起来"拉丝毛刺"的那些东西，本质是没有正确并入主表面
        # 的独立小三角面组)。这里按连通分量拆分，只保留顶点数最多的主体部分，
        # 直接把这些孤立碎片扔掉，比事后再去平滑更彻底。
        components = jeans_trimesh.split(only_watertight=False)
        if len(components) > 1:
            n_before = len(jeans_trimesh.vertices)
            # 过滤掉低于 200 个顶点的微小噪点碎片，保留所有较大的有效连通区域，防止丢掉大块裤子网格
            large_components = [c for c in components if len(c.vertices) > 200]
            if len(large_components) > 0:
                jeans_trimesh = trimesh.util.concatenate(large_components)
            else:
                jeans_trimesh = max(components, key=lambda m: len(m.vertices))
            print(f"🧹 网格清理：从 {len(components)} 个连通分量中过滤微小碎片，保留主要分量 "
                  f"(顶点数 {n_before} -> {len(jeans_trimesh.vertices)})")

        # split() / 前面几步的 trimesh.Trimesh() 重建默认都会顺手做一次隐式顶点合并
        # (process=True)，这有可能在原本已经清理干净的拓扑上重新组合出新的非流形边
        # (实测发生过)。喂给 Taubin 平滑前必须再兜底修一遍，宁可多跑几次也不能让
        # 任何非流形边混进去，否则平滑结果又会炸出拉丝毛刺。
        jeans_trimesh, n_nonmanifold_final = self._repair_non_manifold_until_stable(jeans_trimesh)
        if n_nonmanifold_final:
            print(f"🧹 非流形拓扑修复 (平滑前兜底)：剔除了 {n_nonmanifold_final} 个非流形边/重复面所在的三角面")

        # 上面几轮清理为了拓扑干净会局部整片挖掉三角面，副作用是留下一些细碎小洞；
        # 这里只补小洞，裤子腰部/裤脚的真实开口 (大洞) 原样保留，供 SurfaceWalk 使用。
        jeans_trimesh, n_holes_filled = self._fill_small_holes(jeans_trimesh, max_hole_edges=50)
        if n_holes_filled:
            print(f"🧹 小洞修补：补上了 {n_holes_filled} 个清理过程中产生的局部小洞 "
                  f"(边数 <= 50，裤子真实的腰部/裤脚大洞不受影响)")
            # 补洞新增了面，这里再确认一遍，由于前面做了严格的非流形过滤，这里通常不会再剔除面
            jeans_trimesh, n_nonmanifold_after_fill = self._repair_non_manifold_until_stable(jeans_trimesh)
            if n_nonmanifold_after_fill:
                print(f"🧹 非流形拓扑修复 (补洞后兜底)：剔除了 {n_nonmanifold_after_fill} 个三角面")

        # 读取高层配置的烫平迭代次数
        smooth_iters = self.smoothing_iterations  # 15次

        print(f"🧼 正在应用 Laplacian 滤波器，执行 {smooth_iters} 次褶皱烫平...")
        # 换回标准的 Laplacian 平滑，虽然会造成一定的体积收缩，
        # 但能非常强力地抹平高频噪点，产生用户期望的“绝对平滑”表面。
        # 配合之前的非流形边修复，应该不会再产生严重的顶点发散发毛刺现象。
        smoothed_jeans_mesh = trimesh.smoothing.filter_laplacian(
            jeans_trimesh,
            lamb=0.5,
            iterations=smooth_iters
        )

        # =========================================================
        # 5. 最终验证与文件保存
        # =========================================================
        # 检查网格是否为完全封闭。对于 SurfaceWalk，我们绝对要求返回 False (有开口边界)
        is_watertight = smoothed_jeans_mesh.is_watertight
        print(f"📊 网格水密性检查 (是否完全封闭): {is_watertight}")

        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, output_name)
        smoothed_jeans_mesh.export(output_path)
        print(f"✓ 阶段二圆满完成！网格已导出为: {output_path}，已就绪供 SurfaceWalk 提取自由边缘。")

        return output_path

    @staticmethod
    def calculate_cut_direction(mesh_or_path):
        """基于 2.5D PCA 自动计算牛仔裤在基座 YZ 平面内的偏角，并返回最适合的切片走线方向 -d 向量。

        :param mesh_or_path: trimesh.Trimesh 对象或网格文件路径 (.obj)
        :return: [0.0, cut_y, cut_z] 方向向量
        """
        if isinstance(mesh_or_path, str):
            mesh = trimesh.load(mesh_or_path)
        else:
            mesh = mesh_or_path

        vertices = np.asarray(mesh.vertices)
        if len(vertices) == 0:
            return [0.0, 0.0, 1.0]

        # 提取 Base 坐标系下的 Y 和 Z 坐标（因为 X 是相机到表面的深度方向，YZ 是裤子铺展平面）
        coords = vertices[:, [1, 2]]
        mean = coords.mean(axis=0)
        centered = coords - mean
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)

        # 选出与基座 Y 轴 (Index 0，即垂直长度轴) 夹角更小的特征向量作为裤腿的延伸主轴
        v0 = eigvecs[:, 0]
        v1 = eigvecs[:, 1]
        if abs(v0[0]) > abs(v1[0]):
            main_dir = v0
        else:
            main_dir = v1

        # 保证 Y 分量为正 (向上延伸)
        if main_dir[0] < 0:
            main_dir = -main_dir

        # 走线方向与纵向主轴垂直，实现纵向（竖直）长扫喷涂，横向跨步
        cut_dir = [0.0, -main_dir[1], main_dir[0]]
        return cut_dir


def process_jeans_cloud_for_surface_walk(raw_cloud_data, yolo_mask_2d, config):
    """阶段二的全局入口封装，供 trajectory_planner.py 调用。"""
    if isinstance(config, dict):
        calib_path = config.get("system", {}).get("calibration_matrix_path", "./config/hand_eye_calibration.json")
        calib_path = resolve_project_path(calib_path)
        smoothing = int(config.get("process_parameters", {}).get("smoothing_iterations", 15))
    else:
        # 假定为 AppConfig 对象
        calib_path = config.calibration_matrix_path
        smoothing = config.smoothing_iterations

    # 解析标定矩阵以便直接传入
    with open(calib_path, 'r') as f:
        calib_data = json.load(f)
    T_cam_to_base = calib_data['T_camera_to_base']

    # 这里仅用于阶段二的 filter_and_smooth_jeans，无需重建点云和执行分割
    processor = VisionProcessor(
        T_camera_to_base=T_cam_to_base,
        intrinsics_k=None,
        segmenter=None,
        smoothing_iterations=smoothing
    )
    return processor.filter_and_smooth_jeans(raw_cloud_data, yolo_mask_2d)


# =========================================================
# 单元测试入口 (基于 ScanRecorder 采集的离线数据测试)
# =========================================================
if __name__ == "__main__":
    import argparse
    import sys
    
    # 动态把外层的 src/aisprayer/core 目录加入 Python 搜索路径，否则会报找不到 config
    CORE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if CORE_DIR not in sys.path:
        sys.path.insert(0, CORE_DIR)
        
    from core.vision.image2d.segmenter import SegmenterFactory
    from config import SprayerConfig

    parser = argparse.ArgumentParser(description="牛仔裤视觉处理流水线 - 离线数据测试")
    parser.add_argument("--scan_dir", default=None, help="ScanRecorder 生成的扫描数据目录 (不指定则自动读取 data/runs 下最新的)")
    parser.add_argument("--config", default="configs/aisprayer_config.yaml", help="主配置文件路径")
    parser.add_argument("--conf", type=float, default=0.5, help="YOLO 分割置信度阈值")
    args = parser.parse_args()

    # 使用 SprayerConfig 解析统一配置与标定文件
    sprayer_config = SprayerConfig(args.config)

    scan_dir = args.scan_dir
    if not scan_dir:
        output_root = sprayer_config._resolve_path(sprayer_config.output_root)
        if not os.path.exists(output_root):
            raise ValueError(f"自动获取扫描目录失败：数据根目录不存在 {output_root}")
        
        subdirs = [os.path.join(output_root, d) for d in os.listdir(output_root) if os.path.isdir(os.path.join(output_root, d))]
        if not subdirs:
            raise ValueError(f"自动获取扫描目录失败：{output_root} 下没有任何数据")
            
        try:
            # 优先尝试按纯数字名称排序 (0, 1, 2...) 获取最大编号的最新测试流
            scan_dir = max(subdirs, key=lambda x: int(os.path.basename(x)))
        except ValueError:
            # 否则退化为按照操作系统文件修改时间获取最新
            scan_dir = max(subdirs, key=os.path.getmtime)
        print(f"[*] 未指定 --scan_dir，自动探测并使用最新扫描目录: {scan_dir}")
    
    if sprayer_config.T_camera_to_base is None:
        raise ValueError("从配置中未能成功读取到 T_camera_to_base 手眼标定矩阵")
    if not sprayer_config.model_path:
        raise ValueError("从配置中未能读取到 YOLO 模型路径 (spraying.model_path)")

    segmenter = SegmenterFactory.create("yolo_trousers", model_path=sprayer_config.model_path, conf=args.conf)

    print(f"[*] 正在准备扫描数据 {scan_dir} ...")
    color_path = os.path.join(scan_dir, "scan.jpg")
    depth_path = os.path.join(scan_dir, "scan.depth.npy")

    intrinsics_k = None
    try:
        with open(os.path.join(scan_dir, "scan.params.yaml"), "r") as f:
            params = yaml.safe_load(f)
            k_list = params.get("camera_params", {}).get("intrinsic_matrix")
            if k_list:
                intrinsics_k = np.array(k_list)
    except Exception as e:
        print(f"[-] 无法读取相机内参: {e}")

    processor = VisionProcessor(
        T_camera_to_base=sprayer_config.T_camera_to_base,
        intrinsics_k=intrinsics_k,
        segmenter=segmenter,
    )

    print("[*] 正在执行点云重建...")
    output_path = processor.process_scan_data(
        color_image_path=color_path,
        depth_image_path=depth_path,
        output_dir=os.path.join(scan_dir, "output")
    )
    print(f"✓ 测试完成，网格已导出为: {output_path}")
