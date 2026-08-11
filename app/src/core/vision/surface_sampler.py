import numpy as np
import trimesh
import cv2

class SurfaceZigzagSampler:
    """
    3D 曲面之字形 (Zigzag) 采点器
    功能：直接在 3D 重建网格上通过平行面切片 (Slicing) 提取绝对平直的曲面轨迹，
    并沿着截线以真实物理单位 (mm) 等距采样，同时获取每个采样点处的表面法向量。
    """
    def __init__(self):
        pass

    def sample(self, mesh, row_spacing_mm, point_spacing_mm):
        """
        :param mesh: trimesh.Trimesh 对象 (顶点单位应为米 m，机器人的 base 坐标系)
        :param row_spacing_mm: 行距 (毫米)
        :param point_spacing_mm: 沿曲线的采样点距 (毫米)
        :return: List of dicts, e.g., [{"point": [x, y, z], "normal": [nx, ny, nz]}, ...]
        """
        row_spacing = row_spacing_mm / 1000.0
        point_spacing = point_spacing_mm / 1000.0

        vertices = np.asarray(mesh.vertices)
        if len(vertices) == 0:
            return []

        # 1. 自动计算 3D 网格主轴，确定切片平面的法向量
        # 使用真实的 3D PCA 寻找裤腿在空间中的延伸主轴
        mean = vertices.mean(axis=0)
        centered = vertices - mean
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        
        # 特征值按升序排列，最后一个对应的特征向量即为方差最大方向（纵向延伸主轴）
        main_axis_3d = eigvecs[:, 2]
        
        # 统一让纵向主轴朝向 Y 轴正半轴方向 (如果主轴恰好朝下，就翻转)
        if main_axis_3d[1] < 0:
            main_axis_3d = -main_axis_3d

        # 为了实现沿裤腿方向 (纵向) 喷涂，切片平面必须平行于裤腿纵向，且平行于深度(X轴)。
        # 因此，切片平面的法向量应该是裤腿的“横向” (Transverse Axis)。
        transverse_axis = np.cross([1.0, 0.0, 0.0], main_axis_3d)
        norm_t = np.linalg.norm(transverse_axis)
        if norm_t < 1e-5:
            transverse_axis = np.array([0.0, 0.0, 1.0])
        else:
            transverse_axis /= norm_t
            
        plane_normal = transverse_axis

        # 2. 找到切片的起止范围
        projections = np.dot(vertices, plane_normal)
        min_proj = np.min(projections)
        max_proj = np.max(projections)

        slice_projs = np.arange(min_proj + row_spacing / 2, max_proj, row_spacing)

        print(f"[Debug] min_proj={min_proj}, max_proj={max_proj}, len(slice_projs)={len(slice_projs)}")
        
        all_points = []
        direction_forward = True

        for proj in slice_projs:
            plane_origin = plane_normal * proj
            
            # 3. 获取平面与网格交线段集合 (m, 2, 3)
            lines = trimesh.intersections.mesh_plane(mesh, plane_normal=plane_normal, plane_origin=plane_origin)
            
            if len(lines) == 0:
                continue

            # 展开所有交点并去重
            pts = lines.reshape(-1, 3)
            pts = np.unique(np.round(pts, decimals=4), axis=0)
            if len(pts) < 2:
                continue
                
            # 4. 寻找沿裤腿纵向的排序轴 (用于对切出来的纵向离散点排序，连接断口)
            # 纵向轴就是我们之前求出来的 main_axis_3d
            
            # 按纵向投影坐标对点云排序，强制连接断裂的线段
            t_projs = np.dot(pts, main_axis_3d)
            sorted_indices = np.argsort(t_projs)
            sorted_pts = pts[sorted_indices]
            
            # 5. 沿 3D 曲线按照 point_spacing 进行等距离散化采样
            diffs = np.diff(sorted_pts, axis=0)
            dists = np.linalg.norm(diffs, axis=1)
            cum_dists = np.insert(np.cumsum(dists), 0, 0.0)
            total_dist = cum_dists[-1]
            
            sample_dists = np.arange(0, total_dist, point_spacing)
            if len(sample_dists) == 0:
                continue
                
            sampled_points = np.zeros((len(sample_dists), 3))
            for i in range(3):
                sampled_points[:, i] = np.interp(sample_dists, cum_dists, sorted_pts[:, i])
                
            # 交替采样方向，形成 之字形 (Zigzag)
            if not direction_forward:
                sampled_points = sampled_points[::-1]
                
            # 5. 获取采样点处的法向量 (最近邻顶点法向，比面片法向更平滑，且无需 rtree 依赖)
            from scipy.spatial import cKDTree
            tree = cKDTree(vertices)
            _, vertex_ids = tree.query(sampled_points)
            normals = mesh.vertex_normals[vertex_ids]
            
            # 追加到结果列表
            for pt, n in zip(sampled_points, normals):
                all_points.append({
                    "point": pt,
                    "normal": n
                })
                
            direction_forward = not direction_forward

        return all_points
