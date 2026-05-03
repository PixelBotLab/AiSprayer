import open3d as o3d
import numpy as np
import os

class PointCloudProcessor:
    """
    负责处理 3D 点云数据，提供局部表面的位置和法向量查询功能。
    """
    def __init__(self, search_radius=25):
        """
        :param search_radius: 搜索半径 (mm)，用于估计局部法向量和邻域搜索
        """
        self.pcd = None
        self.kdtree = None
        self.search_radius = search_radius

    def load_pcd(self, pcd_data):
        """
        加载点云数据。
        :param pcd_data: 可以是文件路径 (str)、o3d.geometry.PointCloud 对象，或者是 (N, 3) 的 numpy 数组
        """
        if isinstance(pcd_data, str):
            if os.path.exists(pcd_data):
                self.pcd = o3d.io.read_point_cloud(pcd_data)
            else:
                print(f"[-] PointCloudProcessor: 找不到文件 {pcd_data}")
                return
        elif isinstance(pcd_data, o3d.geometry.PointCloud):
            self.pcd = pcd_data
        elif isinstance(pcd_data, np.ndarray):
            # 过滤掉深度为 0 或异常的点
            valid_mask = (pcd_data[:, 2] > 100) & (pcd_data[:, 2] < 3000)
            self.pcd = o3d.geometry.PointCloud()
            self.pcd.points = o3d.utility.Vector3dVector(pcd_data[valid_mask])
        else:
            print("[-] PointCloudProcessor: 不支持的数据类型")
            return

        # 1. 体素降采样以平衡精度和计算速度
        self.pcd = self.pcd.voxel_down_sample(voxel_size=8.0)

        # 2. 估计法向量 (对于姿态计算至关重要)
        self.pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=self.search_radius, max_nn=30)
        )
        
        # 3. 统一法向量方向：朝向相机 (假设相机在原点 [0,0,0])
        self.pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
            
        # 4. 构建 KDTree 以便快速检索
        self.kdtree = o3d.geometry.KDTreeFlann(self.pcd)
        print(f"[+] PointCloudProcessor: 点云加载完成，当前点数: {len(self.pcd.points)}")