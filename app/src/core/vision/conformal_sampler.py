import numpy as np
import trimesh
from scipy.spatial import cKDTree

class SurfaceConformalSampler:
    """
    边缘对齐直线喷涂 (Edge-Aligned Straight Zigzag) 采点器
    功能：自动寻找裤腿的直外侧边，将外侧边作为喷涂主轴进行直面切片。
    实现“第一条线完美贴合并平行于裤边，后续全部等距平行”的纯直线喷涂轨迹。
    """
    
    def __init__(self):
        pass

    def sample(self, mesh: trimesh.Trimesh, row_spacing_mm: float, point_spacing_mm: float):
        """
        对 3D 网格进行边缘对齐之字形采点。
        
        :param mesh: trimesh 对象
        :param row_spacing_mm: 行间间距 (两条平行轨迹之间的横向偏移距离)
        :param point_spacing_mm: 点之间的间距 (同一轨迹上的纵向采点间距)
        :return: 包含所有采点信息 (3D坐标、法向量) 的有序列表
        """
        row_spacing = row_spacing_mm / 1000.0
        point_spacing = point_spacing_mm / 1000.0
        
        vertices = np.asarray(mesh.vertices)
        if len(vertices) < 10:
            return []

        # 预计算法向量的 KDTree
        vertex_normals = np.asarray(mesh.vertex_normals)
        tree = cKDTree(vertices)

        # 1. 计算粗略的整体 PCA 主轴
        mean = vertices.mean(axis=0)
        centered = vertices - mean
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)

        rough_main = eigvecs[:, 2]
        if rough_main[1] < 0:
            rough_main = -rough_main

        # 投影到粗略坐标系
        v_long = np.dot(vertices, rough_main)
        
        # 相机深度是 X，我们用 [1, 0, 0] (X轴) 与 rough_main 叉乘得到横向轴
        rough_trans = np.cross([1.0, 0.0, 0.0], rough_main)
        norm_t = np.linalg.norm(rough_trans)
        if norm_t > 1e-5:
            rough_trans /= norm_t
        else:
            rough_trans = np.array([0.0, 1.0, 0.0])
            
        v_trans = np.dot(vertices, rough_trans)

        # 2. 提取左右边缘并拟合直线
        # 将长度方向分成 20 个区间
        min_l, max_l = v_long.min(), v_long.max()
        bins = np.linspace(min_l, max_l, 21)
        
        left_edge_2d = []
        right_edge_2d = []
        
        for i in range(20):
            mask = (v_long >= bins[i]) & (v_long <= bins[i+1])
            if not np.any(mask):
                continue
            bin_trans = v_trans[mask]
            bin_long = v_long[mask]
            
            idx_left = np.argmin(bin_trans)
            idx_right = np.argmax(bin_trans)
            left_edge_2d.append([bin_trans[idx_left], bin_long[idx_left]])
            right_edge_2d.append([bin_trans[idx_right], bin_long[idx_right]])
            
        left_edge_2d = np.array(left_edge_2d)
        right_edge_2d = np.array(right_edge_2d)
        
        # 直线拟合及误差评估函数
        def fit_edge(edge_pts):
            if len(edge_pts) < 2:
                return rough_main, float('inf')
            mean_pt = edge_pts.mean(axis=0)
            centered_pts = edge_pts - mean_pt
            u, s, vh = np.linalg.svd(centered_pts)
            d_trans, d_long = vh[0]
            if d_long < 0:
                d_trans, d_long = -d_trans, -d_long
                
            # 第 2 个奇异值代表了偏离直线的程度（弯曲度）
            error = s[1] if len(s) > 1 else 0.0
            axis_3d = d_trans * rough_trans + d_long * rough_main
            axis_3d /= np.linalg.norm(axis_3d)
            return axis_3d, error

        left_axis, left_err = fit_edge(left_edge_2d)
        right_axis, right_err = fit_edge(right_edge_2d)
        
        # 选择误差较小 (更直) 的边缘作为基准轴，这通常是裤腿的外侧缝线 (Outer Seam)
        if left_err < right_err:
            edge_axis_3d = left_axis
            is_left_edge_better = True
            print(f"[Debug] 选用【左侧边缘】作为直线主轴, err={left_err:.2f} (右侧err={right_err:.2f})")
        else:
            edge_axis_3d = right_axis
            is_left_edge_better = False
            print(f"[Debug] 选用【右侧边缘】作为直线主轴, err={right_err:.2f} (左侧err={left_err:.2f})")

        # 3. 构建真正的直面切片平面
        # 切片平面需要平行于 edge_axis_3d，且平行于 X 轴 (深度)
        # 所以它的法向就是这两个向量的叉积
        plane_normal = np.cross([1.0, 0.0, 0.0], edge_axis_3d)
        plane_normal /= np.linalg.norm(plane_normal)
        
        # 统一法向朝向，确保它指向横向 (Y > 0)
        if plane_normal[1] < 0:
            plane_normal = -plane_normal
            
        projections = np.dot(vertices, plane_normal)
        min_proj = np.min(projections)
        max_proj = np.max(projections)
        
        # 如果是左边缘作为基准，我们从极左侧 (min_proj) 开始，往右等距切片。
        # 如果是右边缘作为基准，我们从极右侧 (max_proj) 开始，往左等距切片。
        # 保证第一条切片线距离外边缘刚好半个 row_spacing。
        if is_left_edge_better:
            slice_projs = np.arange(min_proj + row_spacing/2.0, max_proj, row_spacing)
        else:
            slice_projs = np.arange(max_proj - row_spacing/2.0, min_proj, -row_spacing)
            
        print(f"[Debug] 切片范围: {slice_projs[0]:.4f} -> {slice_projs[-1]:.4f}, 共计 {len(slice_projs)} 条轨迹线")
            
        zigzag_points = []
        direction_forward = True
        
        for proj in slice_projs:
            plane_origin = plane_normal * proj
            
            # 使用平面去横切网格，得到横截线
            lines = trimesh.intersections.mesh_plane(mesh, plane_normal=plane_normal, plane_origin=plane_origin)
            
            if len(lines) == 0:
                continue
                
            pts = lines.reshape(-1, 3)
            pts = np.unique(np.round(pts, decimals=4), axis=0)
            if len(pts) < 2:
                continue
                
            # 沿着真正的直线主轴 (edge_axis_3d) 对交线点进行排序
            l_projs = np.dot(pts, edge_axis_3d)
            sorted_indices = np.argsort(l_projs)
            sorted_pts = pts[sorted_indices]
            
            # 计算沿这条 3D 轨迹线的表面累积弧长
            diffs = np.diff(sorted_pts, axis=0)
            dists = np.linalg.norm(diffs, axis=1)
            cum_dists = np.insert(np.cumsum(dists), 0, 0.0)
            total_dist = cum_dists[-1]
            
            # 每隔 point_spacing 采一个点
            sample_dists = np.arange(0, total_dist, point_spacing)
            if len(sample_dists) == 0:
                continue
                
            sampled_points = np.zeros((len(sample_dists), 3))
            for i in range(3):
                sampled_points[:, i] = np.interp(sample_dists, cum_dists, sorted_pts[:, i])
                
            # 之字形交替掉头
            if not direction_forward:
                sampled_points = sampled_points[::-1]
                
            # 查询法向量
            _, idx = tree.query(sampled_points)
            sampled_normals = vertex_normals[idx]
            
            for i_pt, (p, n) in enumerate(zip(sampled_points, sampled_normals)):
                is_jump = (len(zigzag_points) > 0 and i_pt == 0)
                zigzag_points.append({
                    "point": p.tolist(),
                    "normal": n.tolist(),
                    "is_jump": is_jump
                })
                
            direction_forward = not direction_forward
            
        return zigzag_points
