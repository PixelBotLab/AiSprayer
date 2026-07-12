#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
可视化工具：将 VisionProcessor 生成的 3D 牛仔裤网格可视化。
支持直接读取已有扫描目录，或者通过相机现场拍摄、生成网格后可视化。
配置和使用方式与 vision_processor.py 一致。
"""
import argparse
import os
import sys
import glob

import cv2
import numpy as np
import open3d as o3d
import yaml

# 将 src 目录加入 Python 搜索路径，以便能识别 aisprayer 包
SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from aisprayer.core.config import SprayerConfig
from aisprayer.core.vision.segmenter import SegmenterFactory
from aisprayer.core.vision.vision_processor import VisionProcessor
from aisprayer.core.vision.recorder import ScanRecorder

def main():
    parser = argparse.ArgumentParser(description="AiSprayer 视觉 3D 结果可视化")
    parser.add_argument("--config", default="configs/aisprayer_config.yaml", help="主配置文件路径")
    parser.add_argument("--scan_dir", default=None, help="指定已有的数据目录，不指定则自动取最新")
    parser.add_argument("--camera", action="store_true", help="开启相机采集模式（自动调用相机拍摄、录制、处理并可视化）")
    parser.add_argument("--conf", type=float, default=0.5, help="YOLO 分割置信度阈值")
    parser.add_argument("--mode", default="traj", choices=["traj", "robot"], help="可视化模式: traj(叠加路径和网格), robot(使用PyBullet播放机器人运动)")
    args = parser.parse_args()

    sprayer_config = SprayerConfig(args.config)

    if sprayer_config.T_camera_to_base is None:
        raise ValueError("未能从配置文件中读取到手眼标定矩阵 (T_camera_to_base)")

    # 1. 如果是相机模式，执行录制和处理流水线
    if args.camera:
        print("[*] 模式: 现场相机拍摄并重建可视化")
        if not sprayer_config.model_path:
            raise ValueError("未能从配置文件中读取到 YOLO 模型路径")
            
        segmenter = SegmenterFactory.create("yolo_trousers", model_path=sprayer_config.model_path, conf=args.conf)
        
        from aisprayer.core.hardware.camera.factory import get_camera
        cam = get_camera(sprayer_config.camera_model)
        try:
            print(f"[*] 正在启动 {sprayer_config.camera_model} 相机...")
            cam.start()
            import time
            time.sleep(1.5)  # 给予相机充分的预热和自动曝光调节时间
            color, depth = cam.get_frame()
            if color is None or depth is None:
                raise RuntimeError("无法从相机获取图像数据，请检查相机连接状态")
            print("[*] ✓ 图像采集成功")
        finally:
            cam.stop()
            print("[*] 相机已关闭")

        # 将相机数据录制保存为标准目录结构
        recorder = ScanRecorder(output_root=sprayer_config.output_root)
        k_list = sprayer_config.calib_data.get("camera_params", {}).get("intrinsic_matrix")
        camera_params = {
            "intrinsic_matrix": k_list,
            "camera_model": sprayer_config.camera_model
        }
        scan_dir = recorder.save_scan(color, depth, camera_params)
        print(f"[*] ✓ 数据已保存录制到: {scan_dir}")
        
        # 初始化处理器并执行点云重建
        intrinsics_k = np.array(k_list) if k_list else None
        processor = VisionProcessor(
            T_camera_to_base=sprayer_config.T_camera_to_base,
            intrinsics_k=intrinsics_k,
            segmenter=segmenter,
        )
        color_path = os.path.join(scan_dir, "scan.jpg")
        depth_path = os.path.join(scan_dir, "scan.depth.npy")
        output_dir = os.path.join(scan_dir, "output")
        print("[*] 正在执行点云网格重建流水线...")
        processor.process_scan_data(color_image_path=color_path, depth_image_path=depth_path, output_dir=output_dir)
        
    else:
        # 2. 如果是离线目录读取模式
        scan_dir = args.scan_dir
        if not scan_dir:
            output_root = sprayer_config.output_root
            if not os.path.exists(output_root):
                raise ValueError(f"数据根目录不存在: {output_root}")
            subdirs = [os.path.join(output_root, d) for d in os.listdir(output_root) if os.path.isdir(os.path.join(output_root, d))]
            if not subdirs:
                raise ValueError(f"在 {output_root} 下未找到任何子数据目录")
            try:
                scan_dir = max(subdirs, key=lambda x: int(os.path.basename(x)))
            except ValueError:
                scan_dir = max(subdirs, key=os.path.getmtime)
            print(f"[*] 未指定 --scan_dir，自动获取最新扫描目录: {scan_dir}")
        else:
            print(f"[*] 模式: 加载指定的离线数据目录: {scan_dir}")

    # =========================================================
    # PyBullet 机器人仿真模式
    # =========================================================
    if args.mode == "robot":
        print("[*] 启动 PyBullet 机器人运动仿真模式...")
        traj_json = os.path.join(scan_dir, "output", "trajectory.json")
        urdf_path = sprayer_config.urdf_path
        if not urdf_path:
            raise ValueError("[!] 未能在配置文件中找到 robot_urdf 设置。")
            
        render_robot_trajectory(
            urdf_path=urdf_path,
            trajectory_path=traj_json,
            tcp_name="spray_nozzle_link",
            fps=30.0,
            base_height=0.0
        )
        return

    # =========================================================
    # 可视化阶段：读取输出目录并在 Open3D 窗口中加载网格与真实颜色
    # =========================================================
    output_dir = os.path.join(scan_dir, "output")
    color_img_path = os.path.join(scan_dir, "scan.jpg")
    
    obj_files = glob.glob(os.path.join(output_dir, "*.obj"))
    if not obj_files:
        print(f"[!] 在 {output_dir} 下未找到任何 .obj 文件，请检查处理流水线是否成功执行。")
        return
        
    print(f"[*] 准备可视化 {len(obj_files)} 个生成的 3D 网格文件...")
    
    # 尝试加载带内参的彩色照片用于材质渲染反投影
    color_img = None
    k_matrix = None
    if os.path.exists(color_img_path):
        color_img = cv2.imread(color_img_path)
        try:
            with open(os.path.join(scan_dir, "scan.params.yaml"), "r") as f:
                meta = yaml.safe_load(f)
                k_list = meta.get("camera_params", {}).get("intrinsic_matrix")
                if k_list:
                    k_matrix = np.array(k_list)
        except Exception as e:
            print(f"[!] 无法读取相机内参以进行颜色反投影: {e}")
            k_matrix = None

    # 用来进行颜色反投影所需的变换矩阵
    t_cam_to_base = np.array(sprayer_config.T_camera_to_base)
    t_base_to_cam = np.linalg.inv(t_cam_to_base)

    geometries = []
    
    # 渲染每一个 3D 裤腿网格
    for obj_path in obj_files:
        mesh = o3d.io.read_triangle_mesh(obj_path)
        if mesh.is_empty():
            print(f"[!] 警告: 网格加载为空: {obj_path}")
            continue
        mesh.compute_vertex_normals()
        
        # 将彩色图真实像素映射到 3D 顶点上
        if color_img is not None and k_matrix is not None:
            vertices = np.asarray(mesh.vertices)
            fx, fy, cx, cy = k_matrix[0, 0], k_matrix[1, 1], k_matrix[0, 2], k_matrix[1, 2]
            
            # 从基座坐标系反变回相机坐标系
            ones = np.ones((vertices.shape[0], 1))
            verts_homo = np.hstack((vertices, ones))
            verts_cam = (t_base_to_cam @ verts_homo.T).T[:, :3]
            
            # 透视投影算像素坐标
            x, y, z = verts_cam[:, 0], verts_cam[:, 1], verts_cam[:, 2]
            valid = z > 1e-6  # 在相机前方的点
            u = np.zeros_like(x)
            v = np.zeros_like(y)
            u[valid] = fx * x[valid] / z[valid] + cx
            v[valid] = fy * y[valid] / z[valid] + cy
            
            h, w = color_img.shape[:2]
            ui = np.clip(np.round(u).astype(int), 0, w - 1)
            vi = np.clip(np.round(v).astype(int), 0, h - 1)
            valid &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
            
            bgr = color_img[vi, ui].astype(np.float32) / 255.0
            colors = bgr[:, ::-1].copy()  # BGR 转换到 RGB
            
            # 对于看不见的背面或者视角外的部分，用灰色兜底
            colors[~valid] = np.array([0.6, 0.6, 0.6])
            
            mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
        else:
            # 降级方案：纯色渲染
            mesh.paint_uniform_color([0.75, 0.75, 0.82])
            
        geometries.append(mesh)

    if not geometries:
        raise RuntimeError("未能提取到任何用于渲染的网格。")

    # 增加基座原点坐标系 (0.1米，红=X，绿=Y，蓝=Z)
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    geometries.append(coord_frame)

    # =========================================================
    # 追加显示路径和 TCP 轨迹（内置渲染逻辑）
    # =========================================================
    path_json = os.path.join(output_dir, "path_surface.json")
    traj_json = os.path.join(output_dir, "trajectory.json")
    
    import json
    from scipy.spatial.transform import Rotation as R
    
    def load_path(json_path):
        with open(json_path, "r") as f:
            data = json.load(f)
        
        # 兼容新的包含 mesh_info 的字典结构
        if isinstance(data, dict) and "surface_points" in data:
            points_data = data["surface_points"]
            mesh_info = data.get("mesh_info", [])
            for info in mesh_info:
                print(f"[PCA Info] mesh: {info.get('mesh_source')}, PCA angle: {info.get('pca_angle_deg'):.2f} deg")
        else:
            points_data = data
            
        positions = np.array([[p["x"], p["y"], p["z"]] for p in points_data])
        quats = np.array([[p["qx"], p["qy"], p["qz"], p["qw"]] for p in points_data])
        n = len(positions)
        is_freespace = np.zeros(max(0, n - 1), dtype=bool)
        for i in range(n - 1):
            if points_data[i + 1].get("segment_start", False):
                is_freespace[i] = True
        return positions, quats, is_freespace

    def load_tcp_trajectory(json_path, urdf_path, tcp_name):
        """加载旧版 TCP 位姿轨迹，或通过 URDF 对关节轨迹执行正运动学。"""
        with open(json_path, "r") as f:
            data = json.load(f)
        if not data:
            return np.empty((0, 3)), np.empty((0, 4)), np.empty(0, dtype=bool)

        if all(key in data[0] for key in ("x", "y", "z", "qx", "qy", "qz", "qw")):
            positions = np.array([[p["x"], p["y"], p["z"]] for p in data])
            quats = np.array([[p["qx"], p["qy"], p["qz"], p["qw"]] for p in data])
        elif "joint_positions" not in data[0]:
            raise ValueError("轨迹点既不包含 TCP 位姿，也不包含 joint_positions")
        if not urdf_path or not os.path.exists(urdf_path):
            raise ValueError(f"无法找到用于 TCP 正运动学的 URDF: {urdf_path}")

        import pybullet as p

        client_id = p.connect(p.DIRECT)
        try:
            robot_id = p.loadURDF(urdf_path, useFixedBase=True, physicsClientId=client_id)
            active_joints = [
                index
                for index in range(p.getNumJoints(robot_id, physicsClientId=client_id))
                if p.getJointInfo(robot_id, index, physicsClientId=client_id)[2] == p.JOINT_REVOLUTE
            ]
            tcp_link_index = next(
                (
                    index
                    for index in range(p.getNumJoints(robot_id, physicsClientId=client_id))
                    if p.getJointInfo(robot_id, index, physicsClientId=client_id)[12].decode("utf-8") == tcp_name
                ),
                None,
            )
            if tcp_link_index is None:
                raise ValueError(f"URDF 中未找到 TCP 链接: {tcp_name}")

            positions, quats = [], []
            for point in data:
                joints = point["joint_positions"]
                if len(joints) != len(active_joints):
                    raise ValueError(
                        f"关节数不匹配：轨迹为 {len(joints)}，URDF 活动关节为 {len(active_joints)}"
                    )
                unit = point.get("angle_unit", "deg")
                if unit not in ("deg", "rad"):
                    raise ValueError(f"不支持的关节角单位: {unit}")
                scale = np.pi / 180.0 if unit == "deg" else 1.0
                for joint_index, value in zip(active_joints, joints):
                    p.resetJointState(robot_id, joint_index, value * scale, physicsClientId=client_id)
                tcp_state = p.getLinkState(
                    robot_id, tcp_link_index, computeForwardKinematics=True, physicsClientId=client_id
                )
                positions.append(tcp_state[4])
                quats.append(tcp_state[5])
        finally:
            p.disconnect(client_id)

        n = len(positions)
        is_freespace = np.zeros(max(0, n - 1), dtype=bool)
        for i in range(n - 1):
            p1 = data[i]
            p2 = data[i + 1]
            if p2.get("segment_start", False):
                is_freespace[i] = True
            elif p1.get("motion_type") == "FREESPACE" or p2.get("motion_type") == "FREESPACE":
                is_freespace[i] = True

        return (
            np.asarray(positions),
            np.asarray(quats),
            is_freespace,
        )
        
    def create_cylinder_between_points(A, B, radius=0.0015, color=[0, 0, 1]):
        direction = (B - A).astype(np.float64)
        height = np.linalg.norm(direction)
        if height < 1e-6:
            return None
        direction /= height
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height, resolution=8)
        cylinder.paint_uniform_color(color)
        cylinder.translate([0, 0, height / 2.0])
        z_axis = np.array([0.0, 0.0, 1.0])
        rotation_axis = np.cross(z_axis, direction)
        norm_rotation_axis = np.linalg.norm(rotation_axis)
        if norm_rotation_axis > 1e-6:
            rotation_axis /= norm_rotation_axis
            angle = np.arccos(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
            rot = R.from_rotvec(rotation_axis * angle).as_matrix()
        else:
            rot = -np.eye(3) if np.dot(z_axis, direction) < 0 else np.eye(3)
        cylinder.rotate(rot, center=[0, 0, 0])
        cylinder.translate(A)
        return cylinder

    def build_path_tube(positions, is_freespace, radius=0.0015, start_color=(0.0, 0.0, 1.0), end_color=(1.0, 0.0, 0.0)):
        n = positions.shape[0]
        t = np.linspace(0.0, 1.0, n)
        start_color = np.asarray(start_color)
        end_color = np.asarray(end_color)
        point_colors = (1.0 - t[:, None]) * start_color + t[:, None] * end_color
        segment_colors = (point_colors[:-1] + point_colors[1:]) / 2.0
        path_mesh = o3d.geometry.TriangleMesh()
        for i in range(n - 1):
            if is_freespace[i]:
                continue
            cyl = create_cylinder_between_points(positions[i], positions[i + 1], radius=radius, color=segment_colors[i])
            if cyl is not None:
                path_mesh += cyl
        return path_mesh

    def build_freespace_connections(positions, is_freespace, radius=0.0005, color=(0.8, 0.8, 0.0), dash_len=0.005, gap_len=0.005):
        freespace_mesh = o3d.geometry.TriangleMesh()
        n = positions.shape[0]
        
        current_group = []
        for i in range(n - 1):
            if is_freespace[i]:
                current_group.append(i)
            
            # If group ends (next is not freespace, or it's the last segment)
            if (not is_freespace[i] or i == n - 2) and len(current_group) > 0:
                pts = [positions[current_group[0]]]
                for idx in current_group:
                    pts.append(positions[idx + 1])
                pts = np.array(pts)
                
                dists = np.linalg.norm(pts[1:] - pts[:-1], axis=1)
                cum_dists = np.insert(np.cumsum(dists), 0, 0.0)
                total_dist = cum_dists[-1]
                
                if total_dist > 1e-6:
                    # 沿着曲线均匀绘制虚线
                    current_dist = 0.0
                    while current_dist < total_dist:
                        start_dist = current_dist
                        end_dist = min(current_dist + dash_len, total_dist)
                        
                        start_pt = np.array([np.interp(start_dist, cum_dists, pts[:, 0]), 
                                             np.interp(start_dist, cum_dists, pts[:, 1]), 
                                             np.interp(start_dist, cum_dists, pts[:, 2])])
                        end_pt = np.array([np.interp(end_dist, cum_dists, pts[:, 0]), 
                                           np.interp(end_dist, cum_dists, pts[:, 1]), 
                                           np.interp(end_dist, cum_dists, pts[:, 2])])
                        
                        cyl = create_cylinder_between_points(start_pt, end_pt, radius=radius, color=color)
                        if cyl is not None:
                            freespace_mesh += cyl
                            
                        current_dist += dash_len + gap_len
                        
                    # 在整段 FREESPACE 曲线的末尾添加一个箭头
                    direction = pts[-1] - pts[-2]
                    last_dist = np.linalg.norm(direction)
                    if last_dist > 1e-6:
                        direction /= last_dist
                        arrow_len = min(0.015, total_dist / 2.0)
                        if arrow_len > 0.001:
                            arrow = o3d.geometry.TriangleMesh.create_arrow(
                                cylinder_radius=radius*1.5, cone_radius=radius*3.5,
                                cylinder_height=arrow_len*0.5, cone_height=arrow_len*0.5, resolution=8
                            )
                            arrow.paint_uniform_color(color)
                            z_axis = np.array([0.0, 0.0, 1.0])
                            rotation_axis = np.cross(z_axis, direction)
                            norm_rotation_axis = np.linalg.norm(rotation_axis)
                            if norm_rotation_axis > 1e-6:
                                rotation_axis /= norm_rotation_axis
                                angle = np.arccos(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
                                rot = R.from_rotvec(rotation_axis * angle).as_matrix()
                            else:
                                rot = -np.eye(3) if np.dot(z_axis, direction) < 0 else np.eye(3)
                            arrow.rotate(rot, center=[0, 0, 0])
                            arrow.translate(pts[-1] - direction * arrow_len)
                            freespace_mesh += arrow
                            
                current_group = []
                    
        return freespace_mesh

    def create_text_mesh(text, position, scale=0.015, color=[1.0, 1.0, 1.0]):
        segments = [
            [(0, 2), (1, 2)], # 0: top
            [(1, 2), (1, 1)], # 1: top right
            [(1, 1), (1, 0)], # 2: bot right
            [(0, 0), (1, 0)], # 3: bot
            [(0, 0), (0, 1)], # 4: bot left
            [(0, 1), (0, 2)], # 5: top left
            [(0, 1), (1, 1)], # 6: middle
        ]
        digits = {
            '0': [0, 1, 2, 3, 4, 5],
            '1': [1, 2],
            '2': [0, 1, 6, 4, 3],
            '3': [0, 1, 6, 2, 3],
            '4': [5, 6, 1, 2],
            '5': [0, 5, 6, 2, 3],
            '6': [0, 5, 4, 3, 2, 6],
            '7': [0, 1, 2],
            '8': [0, 1, 2, 3, 4, 5, 6],
            '9': [0, 1, 2, 3, 5, 6],
        }
        
        mesh = o3d.geometry.TriangleMesh()
        offset_x = 0
        for char in text:
            if char in digits:
                for seg_idx in digits[char]:
                    p1, p2 = segments[seg_idx]
                    A = position + np.array([offset_x + p1[0]*scale, p1[1]*scale, 0])
                    B = position + np.array([offset_x + p2[0]*scale, p2[1]*scale, 0])
                    cyl = create_cylinder_between_points(A, B, radius=scale*0.08, color=color)
                    if cyl:
                        mesh += cyl
            offset_x += scale * 1.5
        return mesh

    def build_stroke_numbers(positions, is_freespace, scale=0.008, color=(1.0, 0.0, 0.0)):
        mesh = o3d.geometry.TriangleMesh()
        n = positions.shape[0]
        stroke_idx = 1
        
        for i in range(n - 1):
            if not is_freespace[i]:
                is_start = False
                if i == 0:
                    is_start = True
                elif is_freespace[i - 1]:
                    is_start = True
                
                if is_start:
                    pos = positions[i] + np.array([0.005, 0.005, 0.005])
                    num_mesh = create_text_mesh(str(stroke_idx), pos, scale=scale, color=color)
                    mesh += num_mesh
                    stroke_idx += 1
                    
        return mesh

    def build_waypoint_cloud(positions, color=(0.05, 0.05, 0.05)):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(positions)
        pcd.paint_uniform_color(color)
        return pcd

    def build_waypoint_arrows(positions, quats, stride, size=0.025, color=(1.0, 0.55, 0.0)):
        arrows = o3d.geometry.TriangleMesh()
        idx = np.arange(0, len(positions), max(stride, 1))
        for i in idx:
            pos = positions[i]
            quat = quats[i]
            arrow = o3d.geometry.TriangleMesh.create_arrow(
                cylinder_radius=0.0006, cone_radius=0.0016, cylinder_height=size * 0.6, cone_height=size * 0.4, resolution=8
            )
            arrow.paint_uniform_color(color)
            T = np.eye(4)
            T[:3, :3] = R.from_quat(quat).as_matrix()
            T[:3, 3] = pos
            arrow.transform(T)
            arrows += arrow
        return arrows
        
    if os.path.exists(path_json):
        print(f"[*] 找到表面路径文件，正在渲染: {path_json}")
        try:
            positions, quats, p_freespace = load_path(path_json)
            if len(positions) > 0:
                path_tube = build_path_tube(positions, p_freespace, radius=0.0015)
                waypoint_pcd = build_waypoint_cloud(positions)
                
                start_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
                start_marker.translate(positions[0])
                start_marker.paint_uniform_color([0.0, 1.0, 0.0])
                
                end_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
                end_marker.translate(positions[-1])
                end_marker.paint_uniform_color([1.0, 0.0, 1.0])
                
                arrows = build_waypoint_arrows(positions, quats, stride=5, size=0.03)
                stroke_numbers = build_stroke_numbers(positions, p_freespace, scale=0.008, color=(1.0, 0.0, 0.0))
                path_freespace = build_freespace_connections(positions, p_freespace, radius=0.0008, color=(0.8, 0.8, 0.0))
                
                geometries.extend([path_tube, waypoint_pcd, start_marker, end_marker, arrows, stroke_numbers, path_freespace])
        except Exception as e:
            print(f"[!] 渲染表面路径失败: {e}")
        
    if os.path.exists(traj_json):
        print(f"[*] 找到 TCP 轨迹文件，正在渲染: {traj_json}")
        try:
            t_pos, t_quats, t_freespace = load_tcp_trajectory(
                traj_json, sprayer_config.urdf_path, "spray_nozzle_link"
            )
            if len(t_pos) > 0:
                traj_tube = build_path_tube(
                    t_pos, t_freespace, radius=0.0011, start_color=(0.0, 0.85, 0.85), end_color=(0.0, 0.35, 0.15)
                )
                traj_freespace = build_freespace_connections(t_pos, t_freespace, radius=0.0008, color=(1.0, 0.0, 0.0))
                traj_pcd = build_waypoint_cloud(t_pos, color=(0.0, 0.75, 0.75))
                traj_arrows = build_waypoint_arrows(t_pos, t_quats, stride=5, size=0.03, color=(0.0, 1.0, 1.0))
                
                traj_start_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
                traj_start_marker.translate(t_pos[0])
                traj_start_marker.paint_uniform_color([0.0, 1.0, 0.0])
                
                traj_end_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
                traj_end_marker.translate(t_pos[-1])
                traj_end_marker.paint_uniform_color([1.0, 0.0, 1.0])
                
                traj_stroke_numbers = build_stroke_numbers(t_pos, t_freespace, scale=0.008, color=(1.0, 0.0, 0.0))
                
                geometries.extend([traj_tube, traj_freespace, traj_pcd, traj_arrows, traj_start_marker, traj_end_marker, traj_stroke_numbers])
        except Exception as e:
            print(f"[!] 渲染 TCP 轨迹失败: {e}")

    print("\n[*] 打开交互式 3D 渲染窗口 (Open3D GUI)...")
    o3d.visualization.draw_geometries(
        geometries,
        window_name="AiSprayer 视觉网格及喷涂轨迹 3D 预览",
        width=1280,
        height=900,
    )

def render_robot_trajectory(urdf_path, trajectory_path, tcp_name="spray_nozzle_link", fps=30.0, base_height=0.0):
    import pybullet as p
    import pybullet_data
    import time
    import math
    import json
    
    if not os.path.exists(urdf_path):
        print(f"[!] URDF 文件未找到: {urdf_path}")
        return
    if not os.path.exists(trajectory_path):
        print(f"[!] 轨迹 JSON 文件未找到: {trajectory_path}")
        return

    with open(trajectory_path, "r") as f:
        traj_data = json.load(f)

    if not traj_data or "joint_positions" not in traj_data[0]:
        print(f"[!] 轨迹 JSON 不包含 joint_positions: {trajectory_path}")
        return

    print(f"[*] 加载了 {len(traj_data)} 个轨迹点。")
    
    physicsClient = p.connect(p.GUI)
    
    # 调整默认视角的距离、偏航角、俯仰角和目标中心点，避免机器人重叠遮挡
    p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=60, cameraPitch=-35, cameraTargetPosition=[0.8, 0, 0.5])
    # 隐藏侧边的控制面板，让视窗更干净
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    planeId = p.loadURDF("plane.urdf")
    base_position = [0.0, 0.0, base_height]
    robotId = p.loadURDF(urdf_path, basePosition=base_position, useFixedBase=1)
    
    num_joints = p.getNumJoints(robotId)
    revolute_joint_indices = []
    link_indices = {}
    for i in range(num_joints):
        info = p.getJointInfo(robotId, i)
        link_name = info[12].decode("utf-8")
        link_indices[link_name] = i
        if info[2] == p.JOINT_REVOLUTE:
            revolute_joint_indices.append(i)
            print(f"[*] 找到活动关节: {info[1].decode('utf-8')} (索引 {i})")
            
    if not revolute_joint_indices:
        print("[!] URDF 中没有找到活动关节 (Revolute)。")
        p.disconnect()
        return

    previous_position = None
    arrow_stride = 10
    for index, point in enumerate(traj_data):
        if "x" not in point:
            continue
        position = [point["x"] + base_position[0], point["y"] + base_position[1], point["z"] + base_position[2]]
        if previous_position is not None and not point.get("segment_start", False):
            p.addUserDebugLine(previous_position, position, [0.0, 0.9, 0.9], lineWidth=2.0)
        
        if arrow_stride > 0 and index % arrow_stride == 0:
            rotation = p.getMatrixFromQuaternion([point["qx"], point["qy"], point["qz"], point["qw"]])
            z_axis = [rotation[2], rotation[5], rotation[8]]
            arrow_end = [position[i] + 0.04 * z_axis[i] for i in range(3)]
            p.addUserDebugLine(position, arrow_end, [0.0, 1.0, 1.0], lineWidth=2.5)
        previous_position = position

    tcp_link_index = link_indices.get(tcp_name)
    if tcp_link_index is None:
        print(f"[!] URDF 中未找到指定的 TCP 链接 ({tcp_name})，将不绘制实际 TCP 跟随轨迹。")

    print(f"[*] 开始播放动画循环。关闭 PyBullet 窗口可退出。")
    print("\n=======================================================")
    print(" 💡 调整视角小贴士 (PyBullet):")
    print(" ▶ 旋转视角: 按住 Ctrl 键 + 鼠标左键拖动 (Mac 上为 Cmd/Ctrl + 拖动)")
    print(" ▶ 平移视角: 按住 Ctrl 键 + 鼠标中键拖动")
    print(" ▶ 缩放视角: 鼠标滚轮")
    print("=======================================================\n")
    time.sleep(1)
    
    try:
        while True:
            previous_tcp_position = None
            for pt in traj_data:
                positions = pt["joint_positions"]
                for i, joint_idx in enumerate(revolute_joint_indices):
                    if i < len(positions):
                        val = positions[i]
                        val = math.radians(val)
                        p.resetJointState(robotId, joint_idx, val)
                
                p.stepSimulation()
                if tcp_link_index is not None:
                    tcp_position = p.getLinkState(robotId, tcp_link_index, computeForwardKinematics=True)[4]
                    if previous_tcp_position is not None:
                        p.addUserDebugLine(previous_tcp_position, tcp_position, [1.0, 0.8, 0.0], lineWidth=2.0)
                    previous_tcp_position = tcp_position
                time.sleep(1.0 / fps)
            print("[*] 动画循环重新开始...")
            time.sleep(1)
    except p.error:
        print("[*] 窗口被关闭。")
    except KeyboardInterrupt:
        print("[*] 用户中止播放。")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()
