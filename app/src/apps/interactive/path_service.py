import os
import sys
import time
import logging
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R_tool

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from apps.interactive.reconstruction_service import reconstruction_service

logger = logging.getLogger(__name__)

class ManualPathService:
    def __init__(self):
        self.template_group_dir = os.path.abspath(os.path.join(PROJECT_ROOT, "data", "template_group"))
        self._depth_cache = {}  # { template_name: (mtime, depth_map) }
        self._calib_cache = None  # (last_check_time, T_cam_to_base, k_matrix, calib_desc)

    def _get_depth_map(self, template_name: str) -> np.ndarray:
        template_dir = os.path.join(self.template_group_dir, template_name)
        depth_npy = os.path.join(template_dir, "scan.depth.npy")
        depth_png = os.path.join(template_dir, "scan.depth.png")

        path_to_load = depth_npy if os.path.exists(depth_npy) else (depth_png if os.path.exists(depth_png) else None)
        if not path_to_load:
            raise FileNotFoundError(f"Depth file (scan.depth.npy / scan.depth.png) not found in: {template_dir}")

        mtime = os.path.getmtime(path_to_load)
        cached = self._depth_cache.get(template_name)
        if cached and cached[0] == mtime:
            return cached[1]

        if path_to_load.endswith('.npy'):
            depth_map = np.load(path_to_load)
        else:
            depth_map = cv2.imread(path_to_load, cv2.IMREAD_UNCHANGED)

        if depth_map is None:
            raise ValueError(f"Failed to load depth image from template: {template_name}")

        self._depth_cache[template_name] = (mtime, depth_map)
        return depth_map

    def _get_calibration(self):
        now = time.time()
        if self._calib_cache and (now - self._calib_cache[0] < 5.0):
            return self._calib_cache[1], self._calib_cache[2], self._calib_cache[3]

        T, k, desc = reconstruction_service.get_latest_calibration()
        self._calib_cache = (now, T, k, desc)
        return T, k, desc

    def sample_point_pose(
        self, 
        template_name: str, 
        u: int, 
        v: int, 
        standoff_dist_mm: float = 150.0
    ) -> dict:
        """
        Samples a 2D image pixel coordinate (u, v) and calculates:
        1. 3D physical surface coordinate in base frame (mm)
        2. Surface normal vector in base frame and camera frame
        3. Standoff-offset 3D TCP position in base frame (mm)
        4. 6D TCP Pose [X, Y, Z, Rx, Ry, Rz] in base frame
        5. 2D normal projection vector for UI rendering
        """
        depth_map = self._get_depth_map(template_name)
        h, w = depth_map.shape[:2]
        u = max(0, min(w - 1, int(u)))
        v = max(0, min(h - 1, int(v)))

        # 1. Load Hand-Eye calibration and intrinsics (cached)
        T_cam_to_base_m, k_matrix, calib_desc = self._get_calibration()
        
        if k_matrix is not None:
            fx = float(k_matrix[0, 0])
            fy = float(k_matrix[1, 1])
            cx = float(k_matrix[0, 2])
            cy = float(k_matrix[1, 2])
        else:
            fx, fy, cx, cy = 900.0, 900.0, 640.0, 400.0

        # T_cam_to_base in mm
        T_base_camera = T_cam_to_base_m.copy()
        T_base_camera[0:3, 3] *= 1000.0  # Convert meters to mm
        R_base_camera = T_base_camera[0:3, 0:3]

        # 2. Extract local depth around (u, v) with a robust window & outlier rejection
        win_size = 21
        half_w = win_size // 2
        u_min, u_max = max(0, u - half_w), min(w, u + half_w + 1)
        v_min, v_max = max(0, v - half_w), min(h, v + half_w + 1)
        
        roi_depth = depth_map[v_min:v_max, u_min:u_max].astype(np.float32)
        center_raw_z = float(depth_map[v, u])
        
        if 100 < center_raw_z < 3000:
            valid_mask = (roi_depth > 100) & (roi_depth < 3000) & (np.abs(roi_depth - center_raw_z) < 40.0)
            center_z = center_raw_z
        else:
            valid_mask = (roi_depth > 100) & (roi_depth < 3000)
            center_z = float(np.median(roi_depth[valid_mask])) if np.any(valid_mask) else 800.0

        if not np.any(valid_mask) or np.count_nonzero(valid_mask) < 6:
            normal_cam = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        else:
            grid_v, grid_u = np.mgrid[v_min:v_max, u_min:u_max]
            grid_u = grid_u[valid_mask]
            grid_v = grid_v[valid_mask]
            grid_z = roi_depth[valid_mask]

            grid_x = (grid_u - cx) * grid_z / fx
            grid_y = (grid_v - cy) * grid_z / fy
            pts_3d = np.column_stack((grid_x, grid_y, grid_z))

            # Plane fitting via SVD on centered points
            pts_centered = pts_3d - pts_3d.mean(axis=0)
            _, _, vh = np.linalg.svd(pts_centered)
            normal_cam = vh[2, :]  # Last singular vector is surface normal
            
            # Normal must point towards camera (N_z < 0)
            if normal_cam[2] > 0:
                normal_cam = -normal_cam
            norm_len = np.linalg.norm(normal_cam)
            if norm_len > 1e-6:
                normal_cam /= norm_len
            else:
                normal_cam = np.array([0.0, 0.0, -1.0], dtype=np.float32)

        # 3. 3D surface point in camera coordinates (mm)
        p_surf_cam = np.array([
            (u - cx) * center_z / fx,
            (v - cy) * center_z / fy,
            center_z
        ], dtype=np.float32)

        # 4. Transform Surface Point and Normal to Robot Base Frame (mm)
        p_surf_base = (T_base_camera @ np.array([p_surf_cam[0], p_surf_cam[1], p_surf_cam[2], 1.0]))[0:3]
        normal_base = (R_base_camera @ normal_cam)
        norm_base_len = np.linalg.norm(normal_base)
        if norm_base_len > 1e-6:
            normal_base /= norm_base_len
        else:
            normal_base = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        # 5. Compute Standoff-Offset TCP Target Position (mm)
        # TCP is positioned at distance d along surface normal: P_tcp = P_surf + d * N_base
        p_tcp_base = p_surf_base + float(standoff_dist_mm) * normal_base

        # 6. Compute Tool 6D Orientation Euler Angles (deg)
        # Spray approach direction is -normal_base (towards surface)
        z_tool = -normal_base / (np.linalg.norm(normal_base) + 1e-6)
        x_ref = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        if abs(np.dot(z_tool, x_ref)) > 0.92:
            x_ref = np.array([1.0, 0.0, 0.0], dtype=np.float32)
            
        y_tool = np.cross(z_tool, x_ref)
        y_tool /= (np.linalg.norm(y_tool) + 1e-6)
        x_tool = np.cross(y_tool, z_tool)
        x_tool /= (np.linalg.norm(x_tool) + 1e-6)

        r_mat = np.column_stack((x_tool, y_tool, z_tool))
        euler_deg = R_tool.from_matrix(r_mat).as_euler('xyz', degrees=True)

        # 7. Compute Physically Accurate 2D Normal vector projection on image
        # In camera frame, TCP point is P_tcp_cam = P_surf_cam + standoff * normal_cam
        p_tcp_cam = p_surf_cam + float(standoff_dist_mm) * normal_cam
        if p_tcp_cam[2] > 50.0:
            proj_u_tcp = (fx * p_tcp_cam[0] / p_tcp_cam[2]) + cx
            proj_v_tcp = (fy * p_tcp_cam[1] / p_tcp_cam[2]) + cy
            raw_dx = proj_u_tcp - u
            raw_dy = proj_v_tcp - v
        else:
            raw_dx = normal_cam[0] * 35.0
            raw_dy = normal_cam[1] * 35.0

        raw_mag = float(np.sqrt(raw_dx**2 + raw_dy**2))
        arrow_len = 36.0
        if raw_mag > 1.2:
            proj_dx = (raw_dx / raw_mag) * arrow_len
            proj_dy = (raw_dy / raw_mag) * arrow_len
        else:
            # Flat facing camera: point cleanly upward along -Y with slight tilt
            proj_dx = 0.0
            proj_dy = -arrow_len

        return {
            "pixel": [int(u), int(v)],
            "surface_point_cam_mm": [round(float(x), 2) for x in p_surf_cam],
            "surface_point_base_mm": [round(float(x), 2) for x in p_surf_base],
            "surface_normal_base": [round(float(x), 4) for x in normal_base],
            "surface_normal_cam": [round(float(x), 4) for x in normal_cam],
            "standoff_distance_mm": round(float(standoff_dist_mm), 1),
            "tcp_pose_base": {
                "x": round(float(p_tcp_base[0]), 2),
                "y": round(float(p_tcp_base[1]), 2),
                "z": round(float(p_tcp_base[2]), 2),
                "rx": round(float(euler_deg[0]), 2),
                "ry": round(float(euler_deg[1]), 2),
                "rz": round(float(euler_deg[2]), 2),
            },
            "normal_2d_proj": [round(float(proj_dx), 1), round(float(proj_dy), 1)],
            "calib_source": calib_desc
        }

    def load_manual_paths(self, template_name: str) -> dict:
        """Loads scan.manual_paths.yaml for a given template."""
        template_dir = os.path.join(self.template_group_dir, template_name)
        paths_file = os.path.join(template_dir, "scan.manual_paths.yaml")
        if not os.path.exists(paths_file):
            return {
                "template": template_name,
                "type": "manual",
                "paths": [],
                "standoff_distance_mm": 150.0
            }
        try:
            with open(paths_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            return data
        except Exception as e:
            logger.error(f"Failed to load manual paths from {paths_file}: {e}")
            return {
                "template": template_name,
                "type": "manual",
                "paths": [],
                "standoff_distance_mm": 150.0
            }

    def save_manual_paths(self, template_name: str, paths_data: dict) -> bool:
        """Saves manual paths to scan.manual_paths.yaml with dense surface tracing."""
        template_dir = os.path.join(self.template_group_dir, template_name)
        if not os.path.exists(template_dir):
            os.makedirs(template_dir, exist_ok=True)
            
        paths_file = os.path.join(template_dir, "scan.manual_paths.yaml")
        paths_data["template"] = template_name
        paths_data["type"] = "manual"
        paths_data["updated_at"] = int(time.time())
        paths_data["coordinate_frame"] = "base_link"

        # Compute dense 3D surface points along each path segment using depth map
        try:
            depth_map = self._get_depth_map(template_name)
            T_cam_to_base_m, k_matrix, _ = self._get_calibration()
            if k_matrix is not None:
                fx, fy = float(k_matrix[0, 0]), float(k_matrix[1, 1])
                cx, cy = float(k_matrix[0, 2]), float(k_matrix[1, 2])
            else:
                fx, fy, cx, cy = 900.0, 900.0, 640.0, 400.0

            T_base_cam = T_cam_to_base_m.copy()
            T_base_cam[0:3, 3] *= 1000.0
            h, w = depth_map.shape[:2]

            for path in paths_data.get("paths", []):
                pts = path.get("points", [])
                dense_pts = []
                for i in range(len(pts)):
                    p1 = pts[i]
                    p1_base = p1["surface_point_base_mm"]
                    dense_pts.append([round(float(x), 2) for x in p1_base])
                    if i < len(pts) - 1:
                        p2 = pts[i + 1]
                        u1, v1 = p1["pixel"]
                        u2, v2 = p2["pixel"]
                        steps = 20
                        for k in range(1, steps):
                            t = k / float(steps)
                            uk = int(round((1 - t) * u1 + t * u2))
                            vk = int(round((1 - t) * v1 + t * v2))
                            uk = max(0, min(w - 1, uk))
                            vk = max(0, min(h - 1, vk))
                            zk = float(depth_map[vk, uk])
                            if 100 < zk < 3000:
                                xk = (uk - cx) * zk / fx
                                yk = (vk - cy) * zk / fy
                                pk_cam = np.array([xk, yk, zk, 1.0])
                                pk_base = (T_base_cam @ pk_cam)[0:3]
                                dense_pts.append([round(float(x), 2) for x in pk_base])
                            else:
                                # Linear fallback if depth missing
                                p2_base = p2["surface_point_base_mm"]
                                pk_base = (1 - t) * np.array(p1_base) + t * np.array(p2_base)
                                dense_pts.append([round(float(x), 2) for x in pk_base])
                path["dense_surface_points_base_mm"] = dense_pts
        except Exception as e:
            logger.warning(f"Could not compute dense surface points: {e}")

        try:
            with open(paths_file, 'w', encoding='utf-8') as f:
                yaml.dump(paths_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
            logger.info(f"Successfully saved manual TCP paths to: {paths_file}")
            return True
        except PermissionError as e:
            logger.warning(f"Permission denied writing manual paths to {paths_file}: {e}")
            raise PermissionError(f"Permission denied: cannot write to '{paths_file}'. Directory may be owned by root or read-only.") from e
        except Exception as e:
            logger.warning(f"Failed to save manual paths to {paths_file}: {e}")
            raise IOError(f"File error saving manual paths: {e}") from e

manual_path_service = ManualPathService()
