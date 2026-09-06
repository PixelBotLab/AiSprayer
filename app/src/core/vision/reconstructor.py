"""3D 表面重建引擎 (Poisson Surface Reconstruction)。

提供从 (深度图 + 2D 掩码 + 相机内参 + 手眼标定) 到 (完整平滑 3D Trimesh 网格) 的一站式高内聚处理。
定位为纯内存、无副作用的计算核心：不读写磁盘，不管理子进程。
"""

from __future__ import annotations

import logging
import os
from typing import Optional, Tuple

import cv2
import numpy as np
import open3d as o3d
import trimesh

from core.vision.types import depth_to_point_cloud, k_matrix_to_intrinsics

logger = logging.getLogger(__name__)


class SurfaceReconstructor:
    """泊松曲面重建器 (SurfaceReconstructor)。

    封装深度修复、毛刺过滤、点云转换、Open3D 泊松曲面重建与基座坐标对齐流程。
    """

    def __init__(
        self,
        z_min: float = 100.0,
        z_max: float = 3000.0,
        mask_erode_px: int = 1,
        flying_pixel_max_grad: float = 50.0,
        poisson_depth: int = 8,
        density_threshold: float = 0.15,
        voxel_size: float = 0.003,
        normal_radius: float = 0.03,
        smooth_iterations: int = 20,
        n_threads: int = 4,
    ):
        """
        :param z_min: 有效深度下限 (mm)
        :param z_max: 有效深度上限 (mm)
        :param mask_erode_px: 掩码向内腐蚀像素数，用于切除轮廓边缘飞点 (0 表示不腐蚀)
        :param flying_pixel_max_grad: 深度梯度飞点检测阈值 (mm/像素)，0 表示不启用
        :param poisson_depth: 泊松重建八叉树深度，越大越精细 (常用 8 或 9)
        :param density_threshold: 泊松重建密度裁剪分位数 (0~1)，用于修剪开洞边界
        :param voxel_size: 体素降采样间距 (单位: 米，默认 0.003m 即 3mm)
        :param normal_radius: 法向估计搜索半径 (单位: 米，默认 0.03m 即 30mm)
        :param smooth_iterations: Taubin 平滑迭代次数 (不导致体积收缩)
        :param n_threads: 泊松算法并行线程数
        """
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.mask_erode_px = int(mask_erode_px)
        self.flying_pixel_max_grad = float(flying_pixel_max_grad)
        self.poisson_depth = int(poisson_depth)
        self.density_threshold = float(density_threshold)
        self.voxel_size = float(voxel_size)
        self.normal_radius = float(normal_radius)
        self.smooth_iterations = int(smooth_iterations)
        self.n_threads = int(n_threads)

    @staticmethod
    def _erode_mask(mask_2d: np.ndarray, erode_px: int) -> np.ndarray:
        """对 2D 掩码向内腐蚀指定像素数。"""
        if erode_px <= 0:
            return mask_2d
        kernel = np.ones((erode_px * 2 + 1, erode_px * 2 + 1), np.uint8)
        return cv2.erode(mask_2d.astype(np.uint8), kernel, iterations=1).astype(bool)

    @staticmethod
    def _flying_pixel_mask(depth_image: np.ndarray, max_grad: float) -> np.ndarray:
        """基于 Sobel 梯度的飞点杂质毛刺过滤掩码（True 表示有效）。"""
        if max_grad <= 0:
            return np.ones(depth_image.shape[:2], dtype=bool)
        depth = depth_image.astype(np.float32)
        grad_x = cv2.Sobel(depth, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)
        edge_mask = grad_mag > max_grad
        edge_mask = cv2.dilate(
            edge_mask.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=1
        ).astype(bool)
        return ~edge_mask

    def preprocess_depth(
        self,
        depth_image: np.ndarray,
        mask_2d: np.ndarray,
        inpaint_holes: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """对深度图进行掩码内破洞修补、腐蚀与飞点过滤。

        :param depth_image: HxW 原始深度图 (uint16 或 float，单位 mm)
        :param mask_2d: HxW 目标二值掩码 (bool)
        :param inpaint_holes: 是否使用 OpenCV Navier-Stokes 修复掩码内部缺失的深度破洞
        :return: (processed_depth, combined_valid_mask)
        """
        depth = depth_image.copy()
        valid_depth_mask = (depth > self.z_min) & (depth < self.z_max)

        # 1. 深度破洞修补 (Inpainting)
        if inpaint_holes:
            holes_mask = mask_2d & (~valid_depth_mask)
            if np.any(holes_mask):
                hole_count = int(np.count_nonzero(holes_mask))
                logger.info(
                    "Inpainting %d depth holes inside mask using OpenCV Navier-Stokes...",
                    hole_count,
                )
                depth_f32 = depth.astype(np.float32)
                inpaint_mask = holes_mask.astype(np.uint8) * 255
                filled_depth = cv2.inpaint(
                    depth_f32, inpaint_mask, inpaintRadius=5, flags=cv2.INPAINT_NS
                )
                # 消除修复产生的 NaN / 负值溢出 (Bug 3 修复)
                filled_depth = np.nan_to_num(filled_depth, nan=0.0)
                filled_depth = np.clip(filled_depth, 0.0, self.z_max)
                depth[holes_mask] = filled_depth[holes_mask]
                valid_depth_mask = (depth > self.z_min) & (depth < self.z_max)

        # 2. 掩码适度腐蚀与飞点检测
        eroded_mask = self._erode_mask(mask_2d, self.mask_erode_px)
        flying_pixel_valid = self._flying_pixel_mask(depth, self.flying_pixel_max_grad)

        combined_mask = eroded_mask & valid_depth_mask & flying_pixel_valid
        return depth, combined_mask

    def reconstruct_mesh(
        self,
        raw_point_cloud: np.ndarray,
        valid_mask: np.ndarray,
        T_camera_to_base: Optional[np.ndarray] = None,
    ) -> trimesh.Trimesh:
        """从 2.5D 点云矩阵中提取有效点，执行 Open3D 泊松重建并对齐到基座坐标系。

        :param raw_point_cloud: [H, W, 3] 相机坐标系下的点云 (单位 mm)
        :param valid_mask: [H, W] 布尔过滤掩码
        :param T_camera_to_base: 4x4 手眼标定矩阵 (如为 None 则使用初始化时的矩阵，仍无则保持相机系)
        :return: trimesh.Trimesh 表面网格 (单位: 米)
        """
        if raw_point_cloud.ndim != 3:
            raise ValueError(
                f"raw_point_cloud must be [H, W, 3], got shape {raw_point_cloud.shape}"
            )

        jeans_pts_cam = raw_point_cloud[valid_mask]
        if jeans_pts_cam.shape[0] < 50:
            raise RuntimeError(
                f"Too few valid points inside mask ({jeans_pts_cam.shape[0]} < 50), cannot reconstruct."
            )

        # 单位换算：mm -> m
        jeans_pts_cam = (jeans_pts_cam / 1000.0).astype(np.float64)

        # 1. 点云滤波与法向估计
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(jeans_pts_cam)
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius, max_nn=30
            )
        )
        pcd.orient_normals_towards_camera_location(
            camera_location=np.array([0.0, 0.0, 0.0])
        )

        # 2. 泊松曲面重建
        n_threads = (
            self.n_threads if self.n_threads > 0 else min(4, os.cpu_count() or 4)
        )
        logger.info(
            "Executing Poisson surface reconstruction (depth=%d, threads=%d)...",
            self.poisson_depth,
            n_threads,
        )
        o3d_mesh, densities = (
            o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                pcd, depth=self.poisson_depth, n_threads=n_threads
            )
        )

        # 3. 剔除低密度顶点以打开边界
        densities = np.asarray(densities)
        if len(densities) > 0 and self.density_threshold > 0.0:
            density_val = np.quantile(densities, self.density_threshold)
            vertices_to_remove = densities < density_val
            o3d_mesh.remove_vertices_by_mask(vertices_to_remove)

        # 4. Taubin 表面平滑
        if self.smooth_iterations > 0:
            o3d_mesh = o3d_mesh.filter_smooth_taubin(
                number_of_iterations=self.smooth_iterations
            )

        vertices_cam = np.asarray(o3d_mesh.vertices)
        faces = np.asarray(o3d_mesh.triangles)
        if vertices_cam.shape[0] == 0:
            raise RuntimeError("Poisson reconstruction produced 0 vertices.")

        # 5. 坐标系转换 (相机系 -> 机器人基座系)。T_camera_to_base=None 时保持相机系输出。
        if T_camera_to_base is not None:
            t_mat = np.asarray(T_camera_to_base, dtype=np.float64)
            ones = np.ones((vertices_cam.shape[0], 1), dtype=np.float64)
            vertices_homo = np.hstack((vertices_cam, ones))
            vertices_final = (t_mat @ vertices_homo.T).T[:, :3]
        else:
            vertices_final = vertices_cam

        mesh = trimesh.Trimesh(
            vertices=vertices_final, faces=faces, process=False
        )

        # 6. 保留最大连通分量 (剔除孤立飞片碎片)
        try:
            edges = mesh.face_adjacency
            if len(edges) > 0:
                from trimesh.graph import connected_components

                comps = connected_components(edges, min_len=1)
                if len(comps) > 1:
                    largest_faces = max(comps, key=len)
                    mesh = mesh.submesh([largest_faces], append=True, repair=False)
        except Exception as e:
            logger.warning("Filtering connected components skipped: %s", e)

        logger.info(
            "Mesh reconstruction completed: %d vertices, %d faces",
            len(mesh.vertices),
            len(mesh.faces),
        )
        return mesh

    def reconstruct(
        self,
        depth_image: np.ndarray,
        mask_2d: np.ndarray,
        intrinsics_k: np.ndarray,
        T_camera_to_base: Optional[np.ndarray] = None,
        inpaint_holes: bool = True,
    ) -> trimesh.Trimesh:
        """高内聚端到端重建入口。

        执行：深度图预处理 -> 2.5D 点云提取 -> 泊松表面重建 -> 基座对齐。

        :param depth_image: HxW 原始深度图 (uint16 或 float, mm)
        :param mask_2d: HxW 目标 2D 布尔掩码
        :param intrinsics_k: 3x3 相机内参矩阵
        :param T_camera_to_base: 4x4 手眼标定矩阵；None 时输出保持相机系
        :param inpaint_holes: 是否修复深度图破洞
        :return: trimesh.Trimesh 网格对象
        """
        intrinsics = k_matrix_to_intrinsics(intrinsics_k)
        depth_proc, valid_mask = self.preprocess_depth(
            depth_image, mask_2d, inpaint_holes=inpaint_holes
        )
        raw_point_cloud = depth_to_point_cloud(depth_proc, intrinsics)
        return self.reconstruct_mesh(
            raw_point_cloud, valid_mask, T_camera_to_base=T_camera_to_base
        )
