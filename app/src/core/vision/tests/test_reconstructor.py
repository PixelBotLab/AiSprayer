"""SurfaceReconstructor 表面重建引擎单元测试（纯 CPU、无外部模型权重依赖）。"""

from __future__ import annotations

import unittest

import numpy as np
import trimesh

from core.vision.reconstructor import SurfaceReconstructor
from core.vision.types import depth_to_point_cloud, k_matrix_to_intrinsics


class TestSurfaceReconstructor(unittest.TestCase):
    def setUp(self):
        self.reconstructor = SurfaceReconstructor(
            z_min=100.0,
            z_max=2000.0,
            mask_erode_px=1,
            flying_pixel_max_grad=50.0,
            poisson_depth=6,  # 单测使用较浅深度加速
            density_threshold=0.1,
            voxel_size=0.005,
            smooth_iterations=5,
            n_threads=2,
        )

    def test_depth_inpainting(self):
        """回归测试 §6.3 Bug 3：验证掩码内深度 0 破洞被正确填补，无 NaN / 无极值溢出。"""
        h, w = 100, 100
        # 构造基底深度 800mm 的平面
        depth = np.full((h, w), 800.0, dtype=np.float32)
        mask = np.zeros((h, w), dtype=bool)
        mask[20:80, 20:80] = True

        # 在掩码中心挖一个 20x20 的深度破洞 (0.0mm，属于无效深度)
        hole_region = slice(40, 60), slice(40, 60)
        depth[hole_region] = 0.0

        depth_proc, valid_mask = self.reconstructor.preprocess_depth(
            depth, mask, inpaint_holes=True
        )

        # 1. 验证无 NaN 或 Inf
        self.assertFalse(np.isnan(depth_proc).any())
        self.assertFalse(np.isinf(depth_proc).any())

        # 2. 验证破洞区域被修补回有效深度范围内 (接近 800mm)
        repaired_hole = depth_proc[hole_region]
        self.assertTrue((repaired_hole >= 700.0).all())
        self.assertTrue((repaired_hole <= 900.0).all())

        # 3. 验证破洞区域在 valid_mask 中恢复为 True
        self.assertTrue(valid_mask[45:55, 45:55].all())

    def test_flying_pixel_filtering(self):
        """验证深度断崖边缘的梯度飞点被正确识别并剔除。"""
        h, w = 60, 60
        depth = np.full((h, w), 500.0, dtype=np.float32)
        # 制造纵向阶梯跳变：从 500mm 跳变到 1500mm
        depth[:, 30:] = 1500.0

        mask = np.ones((h, w), dtype=bool)
        _depth_proc, valid_mask = self.reconstructor.preprocess_depth(
            depth, mask, inpaint_holes=False
        )

        # 边界附近 (x=29~31) 应该被梯度飞点检测过滤掉
        self.assertFalse(valid_mask[:, 30].any())

    def test_mask_erode(self):
        """验证掩码向内腐蚀收缩。"""
        mask = np.zeros((50, 50), dtype=bool)
        mask[10:40, 10:40] = True
        eroded = SurfaceReconstructor._erode_mask(mask, erode_px=2)
        # 边界向内缩了 2 个像素：从 (30x30=900) 缩小到 (26x26=676)
        self.assertEqual(int(np.count_nonzero(eroded)), 26 * 26)

    def test_reconstruct_synthetic_cylinder(self):
        """合成一个半径 200mm 的圆柱面点云，验证端到端泊松曲面重建与基座坐标变换。"""
        w, h = 80, 80
        fx, fy, cx, cy = 100.0, 100.0, 40.0, 40.0
        k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

        # 构造平滑凸圆柱深度图
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))
        # 距中心水平偏离
        dx = (u_grid - cx) / fx
        # z = 500 - 30 * dx^2 (拱形圆柱面)
        depth = 600.0 - 150.0 * (dx**2)
        depth = depth.astype(np.float32)

        mask = np.zeros((h, w), dtype=bool)
        mask[15:65, 15:65] = True

        # 平移并旋转的 4x4 手眼标定矩阵 (将相机系平移到 base_link: X+1.0m, Z+0.5m)
        T_camera_to_base = np.eye(4, dtype=np.float64)
        T_camera_to_base[0, 3] = 1.0
        T_camera_to_base[2, 3] = 0.5

        mesh = self.reconstructor.reconstruct(
            depth_image=depth,
            mask_2d=mask,
            intrinsics_k=k,
            T_camera_to_base=T_camera_to_base,
            inpaint_holes=False,
        )

        self.assertIsInstance(mesh, trimesh.Trimesh)
        self.assertGreater(len(mesh.vertices), 20)
        self.assertGreater(len(mesh.faces), 10)

        # 验证顶点已变换到 base 坐标系 (基座系 X 均值应接近相机系 X + 1.0m)
        verts = np.asarray(mesh.vertices)
        mean_x = float(np.mean(verts[:, 0]))
        self.assertAlmostEqual(mean_x, 1.0, delta=0.2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
