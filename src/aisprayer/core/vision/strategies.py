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
    """经典之字形规划算法 —— 只在多边形内部生成喷涂点"""

    def plan(self, pcd_processor, depth_map, polygon_pts, intrinsics, T_base_camera, config):
        fx, fy, cx, cy = intrinsics
        h, w = depth_map.shape
        spray_width   = config.get("spray_width_mm", 80.0)
        overlap       = config.get("overlap_rate", 0.2)
        spray_dist_mm = config.get("spray_dist_mm", 150.0)
        v_step_mm     = config.get("v_step_mm", 20.0)

        # 确定扫描范围
        u_min, v_min = np.min(polygon_pts, axis=0)
        u_max, v_max = np.max(polygon_pts, axis=0)

        step_x = spray_width * (1 - overlap)
        n_cols = int(max((u_max - u_min) / (step_x * fx / 1000.0), 2))
        u_samples = np.linspace(u_min, u_max, n_cols).astype(np.int32)
        step_v    = max(1, int(v_step_mm * fy / 1000.0))

        full_trajectory = []
        is_downward = True

        for u_curr in u_samples:
            v_samples = list(range(int(v_min), int(v_max), step_v))
            col_info = []

            for v_curr in v_samples:
                # 只处理严格在多边形内的点（无外扩，无红色点）
                if cv2.pointPolygonTest(polygon_pts,
                                        (float(u_curr), float(v_curr)), False) < 0:
                    continue

                # 获取深度（先读原点，失败则取邻域中值）
                z = float(depth_map[v_curr, u_curr])
                if z <= 100 or z >= 3000:
                    roi = depth_map[max(0, v_curr-2):v_curr+3,
                                    max(0, u_curr-2):u_curr+3]
                    roi_valid = roi[(roi > 100) & (roi < 3000)]
                    if len(roi_valid) > 0:
                        z = float(np.median(roi_valid))
                    else:
                        continue

                # 相机系 3D 坐标
                pt_cam = np.array([(u_curr - cx) * z / fx,
                                   (v_curr - cy) * z / fy,
                                   z])

                # 法向估算（KNN=5）
                k, idx, _ = pcd_processor.kdtree.search_knn_vector_3d(pt_cam, 5)
                if k < 1:
                    continue
                normal_cam = np.mean(np.asarray(pcd_processor.pcd.normals)[idx], axis=0)
                norm_len = np.linalg.norm(normal_cam)
                if norm_len < 1e-6:
                    continue
                normal_cam /= norm_len

                col_info.append({
                    "u": u_curr, "v": v_curr,
                    "pt_cam": pt_cam,
                    "normal_cam": normal_cam
                })

            if not col_info:
                continue

            # 蛇形方向
            if not is_downward:
                col_info = col_info[::-1]

            # --- 列内法向平滑（Moving Average，窗口=5）---
            if len(col_info) >= 3:
                window_size = 5
                raw_normals = np.array([p["normal_cam"] for p in col_info])
                smoothed    = raw_normals.copy()
                for i in range(len(raw_normals)):
                    s = max(0, i - window_size // 2)
                    e = min(len(raw_normals), i + window_size // 2 + 1)
                    avg_n = np.mean(raw_normals[s:e], axis=0)
                    smoothed[i] = avg_n / (np.linalg.norm(avg_n) + 1e-6)
                for i, p in enumerate(col_info):
                    p["normal_cam"] = smoothed[i]

            # --- 转换为机器人基座坐标系并生成轨迹 ---
            limits = config.get("workspace_limits", {})
            x_limit = limits.get("x", [-np.inf, np.inf])
            y_limit = limits.get("y", [-np.inf, np.inf])
            z_limit = limits.get("z", [-np.inf, np.inf])
            
            filtered_count = 0
            for j, p in enumerate(col_info):
                tcp_pos_cam = p["pt_cam"] + p["normal_cam"] * spray_dist_mm
                p_base      = T_base_camera[:3, :3] @ tcp_pos_cam + T_base_camera[:3, 3]

                # 工作空间安全判定
                if not (x_limit[0] <= p_base[0] <= x_limit[1] and
                        y_limit[0] <= p_base[1] <= y_limit[1] and
                        z_limit[0] <= p_base[2] <= z_limit[1]):
                    if filtered_count == 0:
                        print(f"    [Limit] First filtered point in column: X={p_base[0]:.1f}, Y={p_base[1]:.1f}, Z={p_base[2]:.1f}")
                    filtered_count += 1
                    continue

                R_cam   = self._calculate_rotation_matrix_cam(p["normal_cam"])
                R_base  = T_base_camera[:3, :3] @ R_cam
                abc_base = R_tool.from_matrix(R_base).as_euler('XYZ', degrees=False)

                full_trajectory.append({
                    "pos":        p_base,
                    "abc":        abc_base,
                    "spray_on":   True,     # 所有点均在多边形内，全部喷涂
                    "speed_factor": 1.0,
                    "uv":         (p["u"], p["v"]),
                    "pt_cam":     p["pt_cam"],
                    "normal_cam": p["normal_cam"],
                    "new_line":   (j == 0)
                })
            
            if filtered_count > 0:
                print(f"[!] Warning: {filtered_count} points filtered out by workspace limits in current column.")

            is_downward = not is_downward

        return full_trajectory


class BestPathStrategy(PathStrategy):
    """未来扩展：最优路径算法（暂不可用，回退到 ZigZag）"""
    def plan(self, pcd_processor, depth_map, polygon_pts, intrinsics, T_base_camera, config):
        print("[!] Warning: BestPathStrategy is not yet implemented. Falling back to ZigZag.")
        return ZigZagStrategy().plan(
            pcd_processor, depth_map, polygon_pts, intrinsics, T_base_camera, config
        )
