#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import yaml
import numpy as np
import argparse
import open3d as o3d
import cv2

# 0. 修复 Open3D 在 Wayland/远程环境下的显示问题
os.environ['XDG_SESSION_TYPE'] = 'x11'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

# 1. 路径锚定策略
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.utils.config_helper import load_config, get_abs_path

def resolve_garment_id(raw_id):
    """智能转换 ID: 001 -> trouser_001"""
    if os.path.exists(raw_id): return raw_id
    if raw_id.isdigit() and not raw_id.startswith("trouser_"):
        return f"trouser_{raw_id}"
    return raw_id

def create_thick_line_mesh(points, radius=0.8, color=[0.0, 1.0, 1.0]):
    """
    使用圆柱体 (Cylinder) 集合模拟具有物理粗细的轨迹连线。
    
    参数:
        points: list of dict, 包含 x, y, z 坐标的点序列
        radius: float, 线条半径 (mm)
        color: list, RGB 颜色
    """
    line_meshes = o3d.geometry.TriangleMesh()
    for i in range(len(points) - 1):
        p1 = np.array([points[i]['x'], points[i]['y'], points[i]['z']])
        p2 = np.array([points[i+1]['x'], points[i+1]['y'], points[i+1]['z']])
        
        vec = p2 - p1
        dist = np.linalg.norm(vec)
        if dist < 1e-6: continue
        
        # 1. 创建基础圆柱体，默认高度为 dist，垂直于 Z 轴
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=dist)
        
        # 2. 计算从默认 Z 轴旋转到目标向量 v 的旋转矩阵
        z_axis = [0, 0, 1]
        v = vec / dist
        axis = np.cross(z_axis, v)
        angle = np.arccos(np.clip(np.dot(z_axis, v), -1.0, 1.0))
        
        if np.linalg.norm(axis) > 1e-6:
            # 使用轴角公式获取旋转矩阵
            R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis/np.linalg.norm(axis) * angle)
            cylinder.rotate(R, center=[0, 0, 0])
        
        # 3. 将圆柱体中心移动到线段的中点
        cylinder.translate((p1 + p2) / 2)
        cylinder.paint_uniform_color(color)
        line_meshes += cylinder
    return line_meshes

def load_visual_elements(s_dir, T_base_camera=None, cam_space=False):
    """
    从指定的视角目录加载所有可视化元素。
    
    此函数具备“双路径”验证逻辑：
    - 路径 A (橙金色球): 直接展示 plan.yaml 中的 pos，受标定外参影响，用于验证最终输出。
    - 路径 B (亮绿色球): 通过 uv_ratio 从深度图反向投影，不受标定影响，用于验证规划算法本身的正确性。
    
    参数:
        s_dir: 视角目录 (包含 scan.jpg, scan.depth.npy 等)
        T_base_camera: 4x4 标定矩阵 (Camera -> Base)
        cam_space: 是否切换到相机空间进行诊断
    """
    elements = []
    
    # --- 1. 文件准备与配置加载 ---
    jpg_path, depth_path = os.path.join(s_dir, "scan.jpg"), os.path.join(s_dir, "scan.depth.npy")
    params_path, plan_path = os.path.join(s_dir, "scan.params.yaml"), os.path.join(s_dir, "plan.yaml")

    res_data = None
    if os.path.exists(plan_path):
        with open(plan_path, 'r') as f: 
            res_data = yaml.safe_load(f)

    # --- 2. 3D 彩色点云重建 ---
    pcd = None
    if os.path.exists(jpg_path) and os.path.exists(depth_path) and os.path.exists(params_path):
        try:
            # 加载原始数据
            color_img, depth_img = cv2.imread(jpg_path), np.load(depth_path)
            with open(params_path, 'r') as f: 
                params = yaml.safe_load(f)
            
            # 提取内参
            K_list = params.get("camera_params", {}).get("intrinsic_matrix", [])
            fx, fy, cx, cy = [K_list[0][0], K_list[1][1], K_list[0][2], K_list[1][2]] if len(K_list)>=3 else [900., 900., 640., 360.]
            h, w = depth_img.shape
            z = depth_img.astype(np.float32)
            
            # 基础过滤：剔除无效深度值
            mask = (z > 100) & (z < 3000)
            
            # 多边形裁切：仅显示规划范围内的点云，减少视觉干扰
            poly_ratio = res_data.get("metadata", {}).get("polygon_ratio") if res_data else None
            if poly_ratio:
                poly_pts = (np.array(poly_ratio) * [w, h]).astype(np.int32)
                poly_mask = np.zeros((h, w), dtype=np.uint8)
                cv2.fillPoly(poly_mask, [poly_pts], 255)
                mask &= (poly_mask > 0)

            # 执行反投影：像素 (u, v, z) -> 相机空间 (x, y, z)
            z_final = z[mask]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.column_stack(((np.mgrid[0:h, 0:w][1][mask] - cx) * z_final / fx, 
                                                                    (np.mgrid[0:h, 0:w][0][mask] - cy) * z_final / fy, 
                                                                    z_final)))
            pcd.colors = o3d.utility.Vector3dVector(color_img[mask][:, [2, 1, 0]] / 255.0)
        except Exception as e: 
            print(f" [!] 重建失败: {e}")

    # 如果有标定矩阵且非诊断模式，将点云转到基座系
    if pcd:
        if not cam_space and T_base_camera is not None: 
            pcd.transform(T_base_camera)
        elements.append(pcd)
    
    # --- 3. 规划轨迹处理 (核心逻辑) ---
    if res_data:
        cols_data = res_data.get("columns", [])
        if cols_data and 'depth_img' in locals():
            # 准备变换矩阵用于坐标对齐
            R_bc = T_base_camera[:3, :3] if T_base_camera is not None else np.eye(3)
            t_bc = T_base_camera[:3, 3] if T_base_camera is not None else np.zeros(3)
            # 基座到相机的逆变换
            R_cb, t_cb = R_bc.T, -R_bc.T @ t_bc

            traj_points_uv = []
            flat_points = [] # 记录所有有效点用于寻找起点/终点

            for col in cols_data:
                last_p_uv = None
                for pt_idx, p in enumerate(col):
                    # --- A. 展示 Robot Pos (橙金色球) ---
                    # 这是 plan.yaml 里保存的物理坐标，体现了标定的结果
                    pos_base = np.array(p.get("pos"))
                    display_pos_base = R_cb @ pos_base + t_cb if cam_space else pos_base
                    
                    sphere_base = o3d.geometry.TriangleMesh.create_sphere(radius=2.5)
                    sphere_base.translate(display_pos_base)
                    sphere_base.paint_uniform_color([1.0, 0.4, 0.0]) # 橙金色
                    elements.append(sphere_base)

                    # --- B. 展示 UV 投影 (亮绿色球) ---
                    # 这是根据像素坐标直接投影的“真值”，不受标定误差干扰
                    uv = p.get("uv_ratio")
                    if uv:
                        u, v = int(uv[0]*w), int(uv[1]*h)
                        zv = depth_img[np.clip(v,0,h-1), np.clip(u,0,w-1)]
                        if zv > 0:
                            # 1. 计算相机空间坐标
                            p_cam = np.array([(u-cx)*zv/fx, (v-cy)*zv/fy, zv])
                            # 2. 根据当前模式决定显示位置
                            dp_uv = p_cam if cam_space else (R_bc @ p_cam + t_bc)
                            
                            sphere_uv = o3d.geometry.TriangleMesh.create_sphere(radius=3.5)
                            sphere_uv.translate(dp_uv)
                            sphere_uv.paint_uniform_color([0.0, 1.0, 0.2]) # 亮绿色
                            elements.append(sphere_uv)
                            
                            # 记录连线点
                            traj_points_uv.append({'x':dp_uv[0], 'y':dp_uv[1], 'z':dp_uv[2]})
                            flat_points.append(dp_uv)

                            # --- C. 喷涂法向验证 (红色箭头) ---
                            # 箭头起点：在空中；终点：指向裤子表面
                            n_raw = p.get("normal_cam")
                            if n_raw:
                                nv = np.array(n_raw) if cam_space else (R_bc @ np.array(n_raw))
                                nv /= np.linalg.norm(nv)
                                arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.8, cone_radius=1.8, cylinder_height=20, cone_height=8)
                                
                                # 让箭头朝向表面 (-nv)
                                target, z_ax = -nv, [0, 0, 1]
                                axis = np.cross(z_ax, target)
                                if np.linalg.norm(axis) > 1e-6:
                                    R_a = o3d.geometry.get_rotation_matrix_from_axis_angle(axis/np.linalg.norm(axis)*np.arccos(np.dot(z_ax, target)))
                                    arrow.rotate(R_a, center=[0,0,0])
                                
                                # 将箭头尾部定位在偏离表面 28mm 的位置
                                arrow.translate(dp_uv + nv * 28.0)
                                arrow.paint_uniform_color([1, 0, 0])
                                elements.append(arrow)

                            # --- D. 运动流向验证 (亮白色箭头) ---
                            # 在轨迹上每隔 3 个点显示一个箭头，表示前进方向
                            if last_p_uv is not None and pt_idx % 3 == 0:
                                mv = dp_uv - last_p_uv
                                md = np.linalg.norm(mv)
                                if md > 5:
                                    m_dir = mv/md
                                    am = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.6, cone_radius=1.5, cylinder_height=10, cone_height=6)
                                    ax_m = np.cross(z_ax, m_dir)
                                    if np.linalg.norm(ax_m) > 1e-6:
                                        am.rotate(o3d.geometry.get_rotation_matrix_from_axis_angle(ax_m/np.linalg.norm(ax_m)*np.arccos(np.clip(np.dot(z_ax, m_dir),-1,1))), center=[0,0,0])
                                    am.translate(last_p_uv + m_dir*5)
                                    am.paint_uniform_color([1, 1, 1])
                                    elements.append(am)
                            last_p_uv = dp_uv

            # E. 起点终点
            if flat_points:
                for pt, clr in [(flat_points[0], [0,0.4,1]), (flat_points[-1], [1,0,1])]:
                    s = o3d.geometry.TriangleMesh.create_sphere(radius=8)
                    s.translate(pt); s.paint_uniform_color(clr); elements.append(s)

            # F. 加粗轨迹线 (青色)
            if traj_points_uv:
                elements.append(create_thick_line_mesh(traj_points_uv, radius=1.0, color=[0, 1, 1]))

    return elements

    return elements

def main():
    parser = argparse.ArgumentParser(description="AiSprayer Open3D 可视化工具")
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs/aisprayer_config.yaml"), help="配置文件路径")
    parser.add_argument("--input", nargs='+', help="指定任务: [ID] (全视角) 或 [ID 角度] (单视角)")
    parser.add_argument("--cam-space", action="store_true",
                        help="诊断模式: 点云保持相机系，规划点逆变换至相机系，用于验证标定正确性")
    args = parser.parse_args()
    cam_space = args.cam_space
    if cam_space:
        print("[!] 诊断模式已启用 (--cam-space): 所有元素将显示在相机坐标系下")

    full_cfg = load_config(args.config, PROJECT_ROOT)
    v_cfg = full_cfg.get("vision", {})
    p_cfg = v_cfg.get("planner", {})
    output_root = get_abs_path(v_cfg.get("output_root", "data/runs"), PROJECT_ROOT)
    
    # 加载标定外参 (用于点云变换)
    T_base_camera = np.eye(4)
    calib_path = get_abs_path(p_cfg.get("calib_path"), PROJECT_ROOT)
    if os.path.exists(calib_path):
        with open(calib_path, 'r') as f:
            c_res = yaml.safe_load(f)
            T_base_camera = np.array(c_res.get("T_base_camera", np.eye(4)))
            print(f"[*] 已加载标定外参，点云将自动对齐至机器人基座坐标系。")
    else:
        print(f"[!] 警告: 找不到标定文件 {calib_path}，点云可能显示不正确。")
    
    # 2. 任务发现逻辑 (同步 planner_tool.py)
    tasks = {} 
    if not args.input:
        subdirs = [d for d in os.listdir(output_root) if os.path.isdir(os.path.join(output_root, d))]
        if not subdirs:
            print("[-] 错误: data/runs 下没有发现任何采集数据"); return
        target_id = max(subdirs, key=lambda d: os.path.getmtime(os.path.join(output_root, d)))
        g_dir = os.path.join(output_root, target_id)
        tasks[target_id] = [os.path.join(g_dir, d) for d in ["0", "90", "180", "270"] if os.path.exists(os.path.join(g_dir, d))]
        print(f"[*] 自动锁定最新任务: {target_id}")
    else:
        raw_id = args.input[0]
        target_id = resolve_garment_id(raw_id)
        g_dir = os.path.join(output_root, target_id)
        if not os.path.exists(g_dir):
            print(f"[-] 错误: 找不到目录 {g_dir}"); return
        if len(args.input) == 1:
            tasks[target_id] = [os.path.join(g_dir, d) for d in ["0", "90", "180", "270"] if os.path.exists(os.path.join(g_dir, d))]
        else:
            angle = args.input[1]
            angle_dir = os.path.join(g_dir, angle)
            if not os.path.exists(angle_dir):
                print(f"[-] 错误: 视角目录不存在 {angle_dir}"); return
            tasks[target_id] = [angle_dir]

    # 3. 汇总并可视化
    all_geometries = []
    # 添加一个坐标轴参考
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=200.0, origin=[0, 0, 0])
    all_geometries.append(axes)

    for g_id, ang_dirs in tasks.items():
        for s_dir in ang_dirs:
            geoms = load_visual_elements(s_dir, T_base_camera, cam_space=cam_space)
            all_geometries.extend(geoms)

    if not all_geometries or len(all_geometries) <= 1:
        print("[-] 没有找到可显示的点云或轨迹数据。")
        return

    print(f"[*] 正在启动 Open3D 渲染窗口...")
    o3d.visualization.draw_geometries(all_geometries, 
                                    window_name=f"AiSprayer Viewer - {target_id}",
                                    width=1280, height=720,
                                    mesh_show_back_face=True)

if __name__ == "__main__":
    main()
