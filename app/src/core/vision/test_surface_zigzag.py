import os
import sys
import argparse
import numpy as np
import yaml
import open3d as o3d
import cv2

# 将项目根目录加入 sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
SRC_DIR = os.path.join(PROJECT_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from aisprayer.core.vision.reconstruction import PoissonReconstructor
from aisprayer.core.vision.image2d.segmenter import SegmenterFactory
from aisprayer.core.config import SprayerConfig
from aisprayer.core.vision.surface_sampler import SurfaceZigzagSampler
from aisprayer.core.vision.conformal_sampler import SurfaceConformalSampler
from aisprayer.core.vision.normal_smoother import PathNormalSmoother


def create_sphere_mesh(center, radius=0.003, color=[0.0, 1.0, 0.0]):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=10)
    sphere.paint_uniform_color(color)
    sphere.translate(center)
    return sphere

def create_cylinder_mesh(p1, p2, radius=0.001, color=[1.0, 0.0, 0.0]):
    p1 = np.array(p1)
    p2 = np.array(p2)
    vec = p2 - p1
    vec_len = np.linalg.norm(vec)
    if vec_len < 1e-6:
        return o3d.geometry.TriangleMesh()
    cyl = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=vec_len, resolution=10)
    cyl.paint_uniform_color(color)
    z_axis = np.array([0.0, 0.0, 1.0])
    dir_vec = vec / vec_len
    axis = np.cross(z_axis, dir_vec)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-6:
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(np.array([np.pi, 0, 0])) if np.dot(z_axis, dir_vec) < 0 else np.eye(3)
    else:
        angle = np.arccos(np.clip(np.dot(z_axis, dir_vec), -1.0, 1.0))
        R = o3d.geometry.get_rotation_matrix_from_axis_angle((axis / axis_norm) * angle)
    cyl.rotate(R, center=(0, 0, 0))
    cyl.translate((p1 + p2) / 2.0)
    return cyl

def create_arrow_mesh(origin, vector, color=[0.0, 0.5, 1.0]):
    vec_len = np.linalg.norm(vector)
    if vec_len < 1e-6:
        return o3d.geometry.TriangleMesh()
    cone_h = vec_len * 0.3
    cyl_h = vec_len - cone_h
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.0015, cone_radius=0.0035,
        cylinder_height=cyl_h, cone_height=cone_h, resolution=15)
    arrow.paint_uniform_color(color)
    z_axis = np.array([0.0, 0.0, 1.0])
    dir_vec = vector / vec_len
    axis = np.cross(z_axis, dir_vec)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-6:
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(np.array([np.pi, 0, 0])) if np.dot(z_axis, dir_vec) < 0 else np.eye(3)
    else:
        angle = np.arccos(np.clip(np.dot(z_axis, dir_vec), -1.0, 1.0))
        R = o3d.geometry.get_rotation_matrix_from_axis_angle((axis / axis_norm) * angle)
    arrow.rotate(R, center=(0, 0, 0))
    arrow.translate(origin)
    return arrow

def visualize_3d(meshes, paths_per_mesh):
    """
    使用 Open3D 渲染多块网格及其表面的之字形采点路点。
    """
    print("[*] 正在打开 Open3D 交互式可视化窗口...")
    geometries = []

    # 添加坐标系参考
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    geometries.append(coord_frame)

    for i, trimesh_obj in enumerate(meshes):
        # 1. 转换 trimesh 为 open3d mesh
        o3d_mesh = o3d.geometry.TriangleMesh()
        o3d_mesh.vertices = o3d.utility.Vector3dVector(trimesh_obj.vertices)
        o3d_mesh.triangles = o3d.utility.Vector3iVector(trimesh_obj.faces)
        o3d_mesh.compute_vertex_normals()
        o3d_mesh.paint_uniform_color([0.7, 0.7, 0.8])  # 浅蓝色
        geometries.append(o3d_mesh)

        # 构建 Open3D 的 RaycastingScene 用于后续的表面投影吸附
        scene = o3d.t.geometry.RaycastingScene()
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(o3d_mesh)
        scene.add_triangles(mesh_t)

        # 2. 绘制 3D 采点轨迹、点位与法向量
        path = paths_per_mesh[i]
        if len(path) > 1:
            points = [p["point"] for p in path]
            normals = [p["normal"] for p in path]
            
            # A. 绘制粗点位 (球体, 半径3mm)
            spheres_mesh = o3d.geometry.TriangleMesh()
            for pt in points:
                spheres_mesh += create_sphere_mesh(pt, radius=0.003, color=[0.0, 1.0, 0.0])
            geometries.append(spheres_mesh)
            
            # B. 绘制轨迹连线 (顺着曲面走，圆柱体, 半径1.0mm)
            cylinders_mesh = o3d.geometry.TriangleMesh()
            
            for j in range(len(points) - 1):
                p1 = np.array(points[j])
                p2 = np.array(points[j+1])
                
                # 判断当前连线是不是"换行跳跃" (transverse jump)
                is_jump = path[j+1].get("is_jump", False)
                line_color = [1.0, 1.0, 0.0] if is_jump else [1.0, 0.0, 0.0]  # 跳跃黄，正常红
                
                # 线性插值 15 个中间点，并投影到网格表面，实现"顺着表面画"的效果
                num_steps = 15
                query_points = np.linspace(p1, p2, num_steps).astype(np.float32)
                
                # 使用 Open3D 的 RaycastingScene 查找最近表面点 (无需 rtree 依赖)
                query_tensor = o3d.core.Tensor(query_points, dtype=o3d.core.Dtype.Float32)
                closest_result = scene.compute_closest_points(query_tensor)
                closest_points = closest_result['points'].numpy()
                
                for k in range(len(closest_points) - 1):
                    cylinders_mesh += create_cylinder_mesh(closest_points[k], closest_points[k+1], radius=0.001, color=line_color)
                    
            geometries.append(cylinders_mesh)
            
            # C. 绘制真实的 3D 箭头表示法向 (长度 5cm)
            arrows_mesh = o3d.geometry.TriangleMesh()
            for pt, n in zip(points, normals):
                pt = np.array(pt)
                n = np.array(n)
                n_norm = np.linalg.norm(n)
                if n_norm > 1e-6:
                    n_scaled = (n / n_norm) * 0.05  # 5cm
                    arrows_mesh += create_arrow_mesh(pt, n_scaled, color=[0.0, 0.5, 1.0])
            geometries.append(arrows_mesh)
            
    o3d.visualization.draw_geometries(geometries, window_name="3D Surface Zigzag Sampler", width=1280, height=800)


def main():
    parser = argparse.ArgumentParser(description="Test 3D Surface Zigzag Sampler")
    default_scan = os.path.join(PROJECT_ROOT, "data/runs/0")
    parser.add_argument("--scan_dir", type=str, default=default_scan, help="扫描数据目录")
    parser.add_argument("--config", default="configs/aisprayer_config.yaml", help="配置文件路径")
    parser.add_argument("--row_spacing", type=float, default=60.0, help="之字形横向行距 (mm)")
    parser.add_argument("--point_spacing", type=float, default=100.0, help="路径上点距 (mm)")
    parser.add_argument("--segmenter", type=str, default="yolo_trousers", help="使用的分割模型")
    
    args = parser.parse_args()

    sprayer_config = SprayerConfig(args.config)
    scan_dir = args.scan_dir

    color_path = os.path.join(scan_dir, "scan.jpg")
    depth_path = os.path.join(scan_dir, "scan.depth.npy")
    params_path = os.path.join(scan_dir, "scan.params.yaml")

    if not os.path.exists(color_path) or not os.path.exists(depth_path):
        print(f"[-] 找不到图像或深度数据: {scan_dir}")
        return

    # 加载相机内参
    intrinsics_k = None
    try:
        with open(params_path, "r") as f:
            params = yaml.safe_load(f)
            k_list = params.get("camera_params", {}).get("intrinsic_matrix")
            if k_list:
                intrinsics_k = np.array(k_list)
    except Exception as e:
        print(f"[-] 无法读取相机内参，可能导致重建失败: {e}")

    # 1. 实例化分割器与重建器
    print(f"[*] 正在初始化 {args.segmenter} ...")
    if args.segmenter == "sam3.1":
        segmenter = SegmenterFactory.create("sam3.1", model_path="models/sam3.1_multiplex/sam3.1_multiplex.pt")
    else:
        segmenter = SegmenterFactory.create(args.segmenter, model_path=sprayer_config.model_path)

    reconstructor = PoissonReconstructor(
        T_camera_to_base=sprayer_config.T_camera_to_base,
        intrinsics_k=intrinsics_k,
        segmenter=segmenter,
        poisson_depth=8,
        density_threshold=0.15
    )

    # 2. 从数据执行泊松重建，直接拿到内存里的 trimesh.Trimesh 对象 (分为左右两条腿)
    print("[*] 正在执行泊松曲面重建...")
    result = reconstructor.from_file(
        color_image_path=color_path,
        depth_image_path=depth_path,
        split_parts=2,
        split_overlap_px=0
    )
    meshes, _ = result  # [mesh_left, mesh_right]

    if not isinstance(meshes, list):
        meshes = [meshes]

    print(f"[+] 成功重建 {len(meshes)} 部分网格。")

    # 3. 运行 3D 曲面之字形采点
    print(f"[*] 开始 3D 曲面之字形采点 (行距={args.row_spacing}mm, 点距={args.point_spacing}mm)...")
    sampler = SurfaceConformalSampler()
    
    paths_per_mesh = []
    total_points_count = 0
    for i, mesh in enumerate(meshes):
        points = sampler.sample(mesh, row_spacing_mm=args.row_spacing, point_spacing_mm=args.point_spacing)
        
        # 使用独立类进行法向平滑，防止机器人执行时喷枪高频抖动
        smoother = PathNormalSmoother(window_size=5)
        points = smoother.smooth(points)
        
        paths_per_mesh.append(points)
        total_points_count += len(points)
        print(f"  - 第 {i+1} 部分网格：生成了 {len(points)} 个真实 3D 喷涂路点")

    print(f"[+] 采点完毕，共计 {total_points_count} 个 3D 点。")

    # 4. 3D 可视化
    visualize_3d(meshes, paths_per_mesh)

if __name__ == "__main__":
    main()
