import numpy as np
import trimesh
from scipy.spatial import cKDTree

class SurfaceConformalSampler:
    """
    3D 曲面随形之字形 (Conformal Zigzag) 采点器
    功能：计算网格的纵向主轴，通过横向切片并计算表面弧长，
    实现“顺着边缘轮廓”一组一组完美平行偏移的喷涂轨迹。
    """
    
    def __init__(self):
        pass

    def sample(self, mesh: trimesh.Trimesh, row_spacing_mm: float, point_spacing_mm: float):
        """
        对 3D 网格进行随形之字形采点。
        
        :param mesh: trimesh 对象
        :param row_spacing_mm: 行间间距 (两条平行轨迹之间的横向偏移距离)
        :param point_spacing_mm: 点之间的间距 (同一轨迹上的纵向切片间距)
        :return: 包含所有采点信息 (3D坐标、法向量) 的有序列表
        """
        # 将 mm 转换为 m (因为网格的单位是米)
        row_spacing = row_spacing_mm / 1000.0
        point_spacing = point_spacing_mm / 1000.0
        
        vertices = np.asarray(mesh.vertices)
        if len(vertices) < 10:
            return []

        # 预计算法向量的 KDTree
        vertex_normals = np.asarray(mesh.vertex_normals)
        tree = cKDTree(vertices)

        # 1. 自动计算网格的 3D 主轴方向
        mean = vertices.mean(axis=0)
        centered = vertices - mean
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)

        # 纵向主轴
        main_axis_3d = eigvecs[:, 2]
        if main_axis_3d[1] < 0:
            main_axis_3d = -main_axis_3d

        # 横向轴 (用于确定左右排序)
        transverse_axis = np.cross([1.0, 0.0, 0.0], main_axis_3d)
        norm_t = np.linalg.norm(transverse_axis)
        if norm_t < 1e-5:
            transverse_axis = np.array([0.0, 0.0, 1.0])
        else:
            transverse_axis /= norm_t
            
        # 2. 我们通过垂直于纵向主轴的平面进行横向切片
        plane_normal = main_axis_3d
        
        projections = np.dot(vertices, plane_normal)
        min_proj = np.min(projections)
        max_proj = np.max(projections)

        # 从上到下的横向切片序列
        slice_projs = np.arange(min_proj, max_proj, point_spacing)
        
        print(f"[Debug] min_proj={min_proj:.4f}, max_proj={max_proj:.4f}, num_slices={len(slice_projs)}")
        
        points_grid = []
        normals_grid = []

        for proj in slice_projs:
            plane_origin = plane_normal * proj
            
            # 获取平面与网格交线段集合 (横截面)
            lines = trimesh.intersections.mesh_plane(mesh, plane_normal=plane_normal, plane_origin=plane_origin)
            
            if len(lines) == 0:
                points_grid.append([])
                normals_grid.append([])
                continue

            # 展开所有交点并去重
            pts = lines.reshape(-1, 3)
            pts = np.unique(np.round(pts, decimals=4), axis=0)
            if len(pts) < 2:
                points_grid.append([])
                normals_grid.append([])
                continue
                
            # 按横向投影坐标对横截面上的点云进行由左到右排序
            t_projs = np.dot(pts, transverse_axis)
            sorted_indices = np.argsort(t_projs)
            sorted_pts = pts[sorted_indices]
            
            # 计算沿这段横向曲线的真实 3D 表面累积弧长
            diffs = np.diff(sorted_pts, axis=0)
            dists = np.linalg.norm(diffs, axis=1)
            cum_dists = np.insert(np.cumsum(dists), 0, 0.0)
            total_dist = cum_dists[-1]
            
            # 在横向弧线上，每隔 row_spacing 采一个点。
            # 为了避免刚好压在边缘上导致喷漆喷空，向内缩进半个 row_spacing
            sample_dists = np.arange(row_spacing / 2.0, total_dist, row_spacing)
            if len(sample_dists) == 0:
                points_grid.append([])
                normals_grid.append([])
                continue
                
            sampled_points = np.zeros((len(sample_dists), 3))
            for i in range(3):
                sampled_points[:, i] = np.interp(sample_dists, cum_dists, sorted_pts[:, i])
                
            # 查询法向量
            _, idx = tree.query(sampled_points)
            sampled_normals = vertex_normals[idx]
            
            points_grid.append(sampled_points)
            normals_grid.append(sampled_normals)
            
        # 3. 纵向重组 (将每一层的同一列点连起来)
        max_paths = max([len(row) for row in points_grid]) if points_grid else 0
        
        zigzag_points = []
        direction_forward = True
        
        for col_idx in range(max_paths):
            col_points = []
            col_normals = []
            for row_idx in range(len(points_grid)):
                # 如果当前切片层存在这一列的采样点，则加入轨迹
                if col_idx < len(points_grid[row_idx]):
                    col_points.append(points_grid[row_idx][col_idx])
                    col_normals.append(normals_grid[row_idx][col_idx])
                    
            if len(col_points) < 2:
                continue
                
            # 交替方向实现 Zigzag
            if not direction_forward:
                col_points = col_points[::-1]
                col_normals = col_normals[::-1]
                
            for p, n in zip(col_points, col_normals):
                zigzag_points.append({
                    "point": p.tolist(),
                    "normal": n.tolist()
                })
                
            direction_forward = not direction_forward
            
        return zigzag_points
