from abc import ABC, abstractmethod
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R_tool

class PathStrategy(ABC):
    """路径规划算法基类"""
    @abstractmethod
    def plan(self, pcd_processor, depth_map, polygon_pts, intrinsics, T_base_camera, config):
        pass

    def _calculate_rotation_matrix_cam(self, normal_cam):
        """通用：在相机坐标系下根据法向计算旋转矩阵"""
        z_axis = -normal_cam / (np.linalg.norm(normal_cam) + 1e-6)
        x_ref = np.array([1.0, 0.0, 0.0])
        y_axis = np.cross(z_axis, x_ref)
        if np.linalg.norm(y_axis) < 1e-3:
            x_ref = np.array([0.0, 1.0, 0.0])
            y_axis = np.cross(z_axis, x_ref)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

class ZigZagStrategy(PathStrategy):
    """经典之字形规划算法"""
    def plan(self, pcd_processor, depth_map, polygon_pts, intrinsics, T_base_camera, config):
        fx, fy, cx, cy = intrinsics
        spray_width = config.get("spray_width_mm", 80.0)
        overlap = config.get("overlap_rate", 0.2)
        spray_dist_mm = config.get("spray_dist_mm", 150.0)
        v_step_mm = config.get("v_step_mm", 20.0)

        # 确定边界
        u_min, v_min = np.min(polygon_pts, axis=0)
        u_max, v_max = np.max(polygon_pts, axis=0)
        
        step_x = spray_width * (1 - overlap)
        n_cols = int(max((u_max - u_min) / (step_x * fx / 1000.0), 2)) # 估算像素步长
        u_samples = np.linspace(u_min, u_max, n_cols).astype(np.int32)
        
        step_v = int(v_step_mm * fy / 1000.0) # 物理步长转像素步长
        
        full_trajectory = []
        is_downward = True

        for u_curr in u_samples:
            v_samples = list(range(int(v_min), int(v_max), step_v))
            valid_v_info = []
            for v_curr in v_samples:
                # 判定点是否在有效区域内
                dist = cv2.pointPolygonTest(polygon_pts, (float(u_curr), float(v_curr)), True)
                is_inside = dist >= -15 # 允许小范围外扩
                if dist < -50: continue
                
                # 获取深度
                z = float(depth_map[v_curr, u_curr])
                if z <= 100 or z >= 3000:
                    roi = depth_map[max(0, v_curr-2):v_curr+3, max(0, u_curr-2):u_curr+3]
                    roi_valid = roi[(roi > 100) & (roi < 3000)]
                    if len(roi_valid) > 0: z = float(np.median(roi_valid))
                    else: continue
                
                # 相机系 3D
                x_cam = (u_curr - cx) * z / fx
                y_cam = (v_curr - cy) * z / fy
                pt_cam = np.array([x_cam, y_cam, z])
                
                # 法向估算
                [k, idx, _] = pcd_processor.kdtree.search_knn_vector_3d(pt_cam, 5)
                if k < 1: continue
                normal_cam = np.mean(np.asarray(pcd_processor.pcd.normals)[idx], axis=0)
                norm_len = np.linalg.norm(normal_cam)
                if norm_len < 1e-6: continue
                normal_cam /= norm_len
                
                valid_v_info.append({
                    "u": u_curr, "v": v_curr, "pt_cam": pt_cam,
                    "normal_cam": normal_cam, "is_inside": is_inside
                })
            
            if not valid_v_info: continue
            inside_idxs = [i for i, info in enumerate(valid_v_info) if info["is_inside"]]
            if not inside_idxs: continue
            
            trimmed_info = valid_v_info[inside_idxs[0] : inside_idxs[-1] + 1]
            if not is_downward: trimmed_info = trimmed_info[::-1]

            for j, info in enumerate(trimmed_info):
                tcp_pos_cam = info["pt_cam"] + info["normal_cam"] * spray_dist_mm
                p_base = T_base_camera[:3, :3] @ tcp_pos_cam + T_base_camera[:3, 3]
                
                R_cam = self._calculate_rotation_matrix_cam(info["normal_cam"])
                R_base = T_base_camera[:3, :3] @ R_cam
                abc_base = R_tool.from_matrix(R_base).as_euler('XYZ', degrees=False)

                full_trajectory.append({
                    "pos": p_base, "abc": abc_base, "spray_on": info["is_inside"],
                    "speed_factor": 1.0 if info["is_inside"] else 3.0,
                    "uv": (info["u"], info["v"]),
                    "pt_cam": info["pt_cam"],
                    "normal_cam": info["normal_cam"],
                    "new_line": (j == 0)
                })
            is_downward = not is_downward
            
        return full_trajectory

class BestPathStrategy(PathStrategy):
    """未来扩展：最优路径算法"""
    def plan(self, pcd_processor, depth_map, polygon_pts, intrinsics, T_base_camera, config):
        print("[!] Warning: BestPathStrategy is not yet implemented. Falling back to ZigZag.")
        return ZigZagStrategy().plan(pcd_processor, depth_map, polygon_pts, intrinsics, T_base_camera, config)
