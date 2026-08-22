"""
交互模板自动航点：读 mesh / masks / 标定，调用 JeansAutoWaypoints，写入 scan.auto.path.yaml。
"""

from __future__ import annotations

import logging
import os
from typing import Any, Optional

import numpy as np
import trimesh
import yaml

from apps.interactive.manual_path_service import manual_path_service
from apps.interactive.reconstruction_service import reconstruction_service
from apps.interactive.sam_service import sam_service
from core.config import SprayerConfig
from core.vision.jeans_auto_waypoints import JeansAutoWaypoints, JeansAutoWaypointsError

logger = logging.getLogger(__name__)

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
TEMPLATE_GROUP_DIR = os.path.join(PROJECT_ROOT, "data", "template_group")


class AutoPathServiceError(ValueError):
    """自动航点生成失败（缺文件、缺标定、规划器拒绝等）。"""


class AutoPathService:
    def __init__(self):
        self.template_group_dir = TEMPLATE_GROUP_DIR

    def generate_auto_paths(
        self,
        template_name: str,
        standoff_dist_mm: Optional[float] = None,
        row_spacing_mm: Optional[float] = None,
        point_spacing_mm: Optional[float] = None,
    ) -> dict[str, Any]:
        template_dir = os.path.join(self.template_group_dir, template_name)
        if not os.path.isdir(template_dir):
            raise AutoPathServiceError(f"template not found: {template_name}")

        mesh = self._load_mesh(template_dir)
        masks_data = self._load_masks(template_dir)
        camera_k, image_size = self._load_intrinsics(template_dir)
        T_camera_to_base, calib_k, calib_desc = self._load_hand_eye()
        if camera_k is None and calib_k is not None:
            camera_k = calib_k
            logger.info("Using camera K from calibration result (%s)", calib_desc)
        if camera_k is None:
            raise AutoPathServiceError("camera K is missing (scan.params.yaml and calibration)")
        if T_camera_to_base is None:
            raise AutoPathServiceError("T_camera_to_base is missing; refuse to plan without hand-eye calibration")

        spray_dist, row_mm, point_mm, dedup_mm = self._resolve_process_params(
            standoff_dist_mm, row_spacing_mm, point_spacing_mm
        )

        planner = JeansAutoWaypoints(
            spray_dist_mm=spray_dist,
            row_spacing_mm=row_mm,
            point_spacing_mm=point_mm,
            image_size=image_size,
            camera_intrinsics=camera_k,
            T_camera_to_base=T_camera_to_base,
            # dedup_radius_mm：左右航点过近时丢掉右腿点，外层按 0.5×行距传入
            dedup_radius_mm=dedup_mm,
            # normal_smooth_window / mesh_unit：法向滑动窗口；重建 mesh 为米
            mesh_unit="m",
            align_outer_edge=True,
        )
        try:
            planned = planner.plan(mesh, masks_data)
        except JeansAutoWaypointsError as e:
            raise AutoPathServiceError(str(e)) from e

        paths = planned.get("paths") or []
        n_points = sum(len(p.get("points") or []) for p in paths)
        if n_points < 1:
            raise AutoPathServiceError("planner returned an empty path")

        payload = {
            "paths": paths,
            "standoff_distance_mm": float(planned.get("standoff_distance_mm", spray_dist)),
        }
        manual_path_service.save_manual_paths(template_name, payload, state_type="auto")
        saved = manual_path_service.load_manual_paths(template_name, state_type="auto")
        paths = saved.get("paths") or paths

        logger.info(
            "Auto path for '%s': %d path(s), %d points, spray=%.1f mm, row=%.1f mm, "
            "point=%.1f mm, dedup=%.1f mm, calib=%s",
            template_name, len(paths), n_points, spray_dist, row_mm, point_mm, dedup_mm, calib_desc,
        )
        return {
            "message": "Auto waypoints generated and saved to scan.auto.path.yaml",
            "template": template_name,
            "path_count": len(paths),
            "point_count": n_points,
            "standoff_distance_mm": float(payload["standoff_distance_mm"]),
            "row_spacing_mm": row_mm,
            "point_spacing_mm": point_mm,
            "dedup_radius_mm": dedup_mm,
            "calibration_source": calib_desc,
            "paths": paths,
        }

    @staticmethod
    def _resolve_process_params(
        standoff_dist_mm: Optional[float],
        row_spacing_mm: Optional[float],
        point_spacing_mm: Optional[float],
    ) -> tuple[float, float, float, float]:
        cfg = SprayerConfig()
        planner_cfg = (cfg.config_data or {}).get("vision", {}).get("planner", {}) or {}
        spray_dist = float(standoff_dist_mm) if standoff_dist_mm is not None else float(cfg.spray_distance) * 1000.0
        if row_spacing_mm is not None:
            row_mm = float(row_spacing_mm)
        else:
            width_mm = float(cfg.spray_width) * 1000.0
            overlap = float(planner_cfg.get("overlap_rate", 0.0) or 0.0)
            row_mm = width_mm * (1.0 - overlap)
        point_mm = float(point_spacing_mm) if point_spacing_mm is not None else 100.0
        if spray_dist <= 0 or row_mm <= 0 or point_mm <= 0:
            raise AutoPathServiceError("spray / row / point spacing must be positive")
        dedup_mm = 0.5 * row_mm
        return spray_dist, row_mm, point_mm, dedup_mm

    @staticmethod
    def _load_mesh(template_dir: str) -> trimesh.Trimesh:
        ply_path = os.path.join(template_dir, "scan.mesh.ply")
        stl_path = os.path.join(template_dir, "scan.mesh.stl")
        mesh_path = ply_path if os.path.exists(ply_path) else (stl_path if os.path.exists(stl_path) else None)
        if mesh_path is None:
            raise AutoPathServiceError("scan.mesh.ply / scan.mesh.stl not found; reconstruct the surface first")
        mesh = trimesh.load(mesh_path, force="mesh", process=False)
        if not isinstance(mesh, trimesh.Trimesh):
            raise AutoPathServiceError(f"failed to load a triangle mesh from {os.path.basename(mesh_path)}")
        if len(mesh.vertices) < 10 or len(mesh.faces) < 1:
            raise AutoPathServiceError("mesh is empty or too small")
        return mesh

    @staticmethod
    def _load_masks(template_dir: str) -> dict:
        data = sam_service.get_template_masks(template_dir)
        if not data or not data.get("masks"):
            raise AutoPathServiceError("scan.masks.yaml is missing or empty; segment the garment first")
        return data

    @staticmethod
    def _load_intrinsics(template_dir: str) -> tuple[Optional[np.ndarray], tuple[int, int]]:
        params_path = os.path.join(template_dir, "scan.params.yaml")
        image_size = (1280, 800)
        k = None
        if os.path.exists(params_path):
            try:
                with open(params_path, "r", encoding="utf-8") as f:
                    pdata = yaml.safe_load(f) or {}
                cam = pdata.get("camera_params") or {}
                k_list = cam.get("intrinsic_matrix")
                if k_list:
                    k = np.asarray(k_list, dtype=np.float64)
                w, h = cam.get("width"), cam.get("height")
                if w and h:
                    image_size = (int(w), int(h))
            except Exception as e:
                logger.warning("Could not read scan.params.yaml: %s", e)
        return k, image_size

    @staticmethod
    def _load_hand_eye() -> tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
        T, calib_k, desc = reconstruction_service.get_latest_calibration()
        if desc.startswith("Identity") or T is None:
            return None, calib_k, desc
        T = np.asarray(T, dtype=np.float64)
        if T.shape != (4, 4):
            return None, calib_k, desc
        k = None if calib_k is None else np.asarray(calib_k, dtype=np.float64)
        return T, k, desc


auto_path_service = AutoPathService()
