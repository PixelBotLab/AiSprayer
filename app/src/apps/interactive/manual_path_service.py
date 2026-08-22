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
from core.utils.fast_yaml import fast_yaml_load, fast_yaml_dump
from core.config import sprayer_config

logger = logging.getLogger(__name__)

class ManualPathService:
    """
    Dedicated service for interactive manual TCP path generation, 2D image sampling,
    surface normal extraction, standoff offset calculations, and YAML persistence.
    """
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

        # 2. Extract local normal matching verify_tab.py compute_local_normal (cross-pattern step=5)
        def get_robust_depth_py(depth, ui, vi, max_r=3):
            if ui < 0 or ui >= w or vi < 0 or vi >= h:
                return 0.0
            z0 = float(depth[vi, ui])
            if 100 < z0 < 3000:
                return z0
            for r in range(1, max_r + 1):
                valid = []
                for du in range(-r, r + 1):
                    for dv in range(-r, r + 1):
                        if abs(du) == r or abs(dv) == r:
                            nui, nvi = ui + du, vi + dv
                            if 0 <= nui < w and 0 <= nvi < h:
                                val = float(depth[nvi, nui])
                                if 100 < val < 3000:
                                    valid.append(val)
                if valid:
                    return float(np.median(valid))
            return 0.0

        center_z = get_robust_depth_py(depth_map, u, v, max_r=5)
        if center_z <= 0:
            center_z = 800.0

        step = 5
        zL = get_robust_depth_py(depth_map, u - step, v, max_r=3)
        zR = get_robust_depth_py(depth_map, u + step, v, max_r=3)
        zU = get_robust_depth_py(depth_map, u, v - step, max_r=3)
        zD = get_robust_depth_py(depth_map, u, v + step, max_r=3)

        if zL > 0 and zR > 0 and zU > 0 and zD > 0:
            pL = np.array([(u - step - cx) * zL / fx, (v - cy) * zL / fy, zL])
            pR = np.array([(u + step - cx) * zR / fx, (v - cy) * zR / fy, zR])
            pU = np.array([(u - cx) * zU / fx, (v - step - cy) * zU / fy, zU])
            pD = np.array([(u - cx) * zD / fx, (v + step - cy) * zD / fy, zD])
            v1 = pR - pL
            v2 = pD - pU
            normal_cam = np.cross(v1, v2)
            n_len = np.linalg.norm(normal_cam)
            if n_len > 1e-6:
                normal_cam /= n_len
                if normal_cam[2] > 0:
                    normal_cam = -normal_cam
            else:
                normal_cam = np.array([0.0, 0.0, -1.0], dtype=np.float32)
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
        p_tcp_base = p_surf_base + float(standoff_dist_mm) * normal_base

        # 6. Compute Tool 6D Orientation Euler Angles (deg)
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

        # 7. Compute Physically Accurate 2D Normal vector projection on image (matches verify_tab.py)
        # In camera frame, TCP point is P_tcp_cam = P_surf_cam + standoff * normal_cam
        p_tcp_cam = p_surf_cam + float(standoff_dist_mm) * normal_cam
        if p_tcp_cam[2] > 50.0:
            proj_u_tcp = (fx * p_tcp_cam[0] / p_tcp_cam[2]) + cx
            proj_v_tcp = (fy * p_tcp_cam[1] / p_tcp_cam[2]) + cy
            proj_dx = float(proj_u_tcp - u)
            proj_dy = float(proj_v_tcp - v)
        else:
            proj_dx = 0.0
            proj_dy = 0.0

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

    @staticmethod
    def path_filename(state_type: str) -> str:
        names = {
            "raw": "scan.manual.path.yaml",
            "auto": "scan.auto.path.yaml",
            "poi": "scan.manual.poi.path.yaml",
            "auto_poi": "scan.auto.poi.path.yaml",
        }
        if state_type not in names:
            raise ValueError(f"Invalid path state '{state_type}'. Expected one of: {', '.join(names)}")
        return names[state_type]

    @staticmethod
    def legacy_path_filenames(state_type: str) -> tuple[str, ...]:
        return {
            "raw": ("scan.raw.path.yaml",),
            "poi": ("scan.poi.path.yaml",),
        }.get(state_type, ())

    def _ensure_state_type(self, state_type: str) -> str:
        state = (state_type or "").strip().lower()
        self.path_filename(state)
        return state

    def _resolve_path_file(self, template_dir: str, state_type: str) -> str:
        canonical = os.path.join(template_dir, self.path_filename(state_type))
        if os.path.exists(canonical):
            return canonical
        for name in self.legacy_path_filenames(state_type):
            legacy = os.path.join(template_dir, name)
            if os.path.exists(legacy):
                return legacy
        return canonical

    def load_manual_paths(self, template_name: str, state_type: str = "raw") -> dict:
        """Loads scan.{manual,auto,manual.poi,auto.poi}.path.yaml for a given template."""
        state_type = self._ensure_state_type(state_type)

        template_dir = os.path.join(self.template_group_dir, template_name)
        paths_file = self._resolve_path_file(template_dir, state_type)
        if not os.path.exists(paths_file):
            return {
                "template": template_name,
                "type": state_type,
                "state_type": state_type,
                "paths": [],
                "standoff_distance_mm": sprayer_config.spray_distance_mm
            }
        try:
            with open(paths_file, 'r', encoding='utf-8') as f:
                data = fast_yaml_load(f)
            data["loaded_from"] = os.path.basename(paths_file)
            data["state_type"] = state_type
            return data
        except Exception as e:
            logger.error(f"Failed to load manual paths from {paths_file}: {e}")
            return {
                "template": template_name,
                "type": state_type,
                "state_type": state_type,
                "paths": [],
                "standoff_distance_mm": sprayer_config.spray_distance_mm
            }

    def smooth_path_waypoints(self, points: list[dict]) -> list[dict]:
        """
        Smooths surface normal vectors across waypoints to remove depth sensor noise spikes
        and builds tangent-consistent tool frames to eliminate 180° flip discontinuities.
        """
        if len(points) < 2:
            return points

        n = len(points)
        normals = np.array([p["surface_normal_base"] for p in points], dtype=np.float64)
        
        # 1. 1D Gaussian / Laplacian moving average on normal vectors
        smoothed_normals = np.zeros_like(normals)
        for i in range(n):
            if i == 0:
                smoothed_normals[i] = 0.7 * normals[0] + 0.3 * normals[min(1, n - 1)]
            elif i == n - 1:
                smoothed_normals[i] = 0.7 * normals[-1] + 0.3 * normals[max(0, n - 2)]
            else:
                smoothed_normals[i] = 0.25 * normals[i - 1] + 0.5 * normals[i] + 0.25 * normals[i + 1]
            
            n_len = np.linalg.norm(smoothed_normals[i])
            if n_len > 1e-6:
                smoothed_normals[i] /= n_len
            else:
                smoothed_normals[i] = normals[i]

        # 2. Tangent-Consistent Frame Construction (prevents 180° Euler angle flips)
        new_points = []
        prev_x_tool = None

        for i, p in enumerate(points):
            wp = dict(p)
            norm_base = smoothed_normals[i]
            wp["surface_normal_base"] = [round(float(x), 4) for x in norm_base]

            # Tool Z points opposite to surface normal into the target
            z_tool = -norm_base / (np.linalg.norm(norm_base) + 1e-6)

            if prev_x_tool is None:
                x_ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                if abs(np.dot(z_tool, x_ref)) > 0.92:
                    x_ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            else:
                x_ref = prev_x_tool

            y_tool = np.cross(z_tool, x_ref)
            y_len = np.linalg.norm(y_tool)
            if y_len > 1e-6:
                y_tool /= y_len
            else:
                y_tool = np.array([0.0, 1.0, 0.0], dtype=np.float64)

            x_tool = np.cross(y_tool, z_tool)
            x_len = np.linalg.norm(x_tool)
            if x_len > 1e-6:
                x_tool /= x_len
            else:
                x_tool = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            prev_x_tool = x_tool

            r_mat = np.column_stack((x_tool, y_tool, z_tool))
            euler_deg = R_tool.from_matrix(r_mat).as_euler('xyz', degrees=True)

            p_surf_base = np.array(wp["surface_point_base_mm"], dtype=np.float64)
            standoff = float(wp.get("standoff_distance_mm", 150.0))
            p_tcp_base = p_surf_base + standoff * norm_base

            wp["tcp_pose_base"] = {
                "x": round(float(p_tcp_base[0]), 2),
                "y": round(float(p_tcp_base[1]), 2),
                "z": round(float(p_tcp_base[2]), 2),
                "rx": round(float(euler_deg[0]), 2),
                "ry": round(float(euler_deg[1]), 2),
                "rz": round(float(euler_deg[2]), 2),
            }
            new_points.append(wp)

        return new_points

    def save_manual_paths(self, template_name: str, paths_data: dict, state_type: str = "raw") -> bool:
        """Saves paths to scan.{manual,auto,manual.poi,auto.poi}.path.yaml. Raw/auto save cleans that family's stale POI."""
        state_type = self._ensure_state_type(state_type)
        template_dir = os.path.join(self.template_group_dir, template_name)
        if not os.path.exists(template_dir):
            os.makedirs(template_dir, exist_ok=True)
            
        file_name = self.path_filename(state_type)
        paths_file = os.path.join(template_dir, file_name)
        paths_data["template"] = template_name
        paths_data["type"] = state_type
        paths_data["updated_at"] = int(time.time())
        paths_data["coordinate_frame"] = "base_link"

        # Apply normal smoothing, tangent frame consistency, and clean waypoints
        cleaned_paths = []
        for path in paths_data.get("paths", []):
            smoothed = self.smooth_path_waypoints(path.get("points", []))
            cleaned_pts = []
            for p in smoothed:
                wp = {
                    "index": int(p.get("index", len(cleaned_pts) + 1)),
                    "pixel": [int(p["pixel"][0]), int(p["pixel"][1])] if "pixel" in p else [0, 0],
                    "surface_point_base_mm": [round(float(v), 2) for v in p.get("surface_point_base_mm", [0, 0, 0])],
                    "surface_normal_base": [round(float(v), 4) for v in p.get("surface_normal_base", [0, 0, 1])],
                    "standoff_distance_mm": round(float(p.get("standoff_distance_mm", 150.0)), 1),
                    "tcp_pose_base": {
                        "x": round(float(p.get("tcp_pose_base", {}).get("x", 0.0)), 2),
                        "y": round(float(p.get("tcp_pose_base", {}).get("y", 0.0)), 2),
                        "z": round(float(p.get("tcp_pose_base", {}).get("z", 0.0)), 2),
                        "rx": round(float(p.get("tcp_pose_base", {}).get("rx", 0.0)), 2),
                        "ry": round(float(p.get("tcp_pose_base", {}).get("ry", 0.0)), 2),
                        "rz": round(float(p.get("tcp_pose_base", {}).get("rz", 0.0)), 2),
                    },
                }
                if "normal_2d_proj" in p and p["normal_2d_proj"]:
                    wp["normal_2d_proj"] = [round(float(v), 1) for v in p["normal_2d_proj"]]
                cleaned_pts.append(wp)
            path["points"] = cleaned_pts
            cleaned_paths.append(path)
        paths_data["paths"] = cleaned_paths

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

        unified_paths_data = {
            "standoff_distance_mm": paths_data.get("standoff_distance_mm", sprayer_config.spray_distance_mm),
            "template": template_name,
            "type": state_type,
            "state_type": state_type,
            "updated_at": int(time.time()),
            "coordinate_frame": "base_link",
        }
        if "execution_speed_mm_s" in paths_data:
            unified_paths_data["execution_speed_mm_s"] = paths_data["execution_speed_mm_s"]
        if "poi_config" in paths_data:
            unified_paths_data["poi_config"] = paths_data["poi_config"]
        if "verification" in paths_data:
            unified_paths_data["verification"] = paths_data["verification"]
        unified_paths_data["paths"] = cleaned_paths

        try:
            with open(paths_file, 'w', encoding='utf-8') as f:
                fast_yaml_dump(unified_paths_data, f)
            logger.info(f"Successfully saved manual TCP paths to: {paths_file}")

            if state_type == "raw":
                stale_files = [
                    "scan.raw.path.yaml",
                    "scan.manual.poi.path.yaml", "scan.poi.path.yaml",
                    "scan.manual.report.json", "scan.raw.report.json",
                    "scan.manual.poi.report.json", "scan.poi.report.json",
                    "scan.opt.path.yaml", "scan.opt.report.json",
                ]
            elif state_type == "auto":
                stale_files = [
                    "scan.auto.poi.path.yaml",
                    "scan.auto.report.json", "scan.auto.poi.report.json",
                ]
            elif state_type == "poi":
                stale_files = [
                    "scan.poi.path.yaml",
                    "scan.poi.report.json",
                ]
            else:
                stale_files = []
            for stale_file in stale_files:
                stale_path = os.path.join(template_dir, stale_file)
                if os.path.exists(stale_path):
                    try:
                        os.remove(stale_path)
                        logger.info(f"🧹 [ManualPathService] Cleaned stale file: {stale_file}")
                    except Exception as ex:
                        logger.warning(f"Could not remove stale file {stale_file}: {ex}")

            return True
        except PermissionError as e:
            logger.warning(f"Permission denied writing manual paths to {paths_file}: {e}")
            raise PermissionError(f"Permission denied: cannot write to '{paths_file}'. Directory may be owned by root or read-only.") from e
        except Exception as e:
            logger.warning(f"Failed to save manual paths to {paths_file}: {e}")
            raise IOError(f"File error saving manual paths: {e}") from e


manual_path_service = ManualPathService()
