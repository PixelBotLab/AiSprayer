"""
牛仔裤完整 mesh 自动航点：2D 掩码分腿打标签，各腿 PCA + 外侧缝之字，重叠去重。
输入/输出均为内存对象，文件读写由外层处理。
"""

from __future__ import annotations

import logging
from typing import Any, Optional

import cv2
import numpy as np
import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R_tool

from core.vision.image2d.jeans_segmentation import split_jeans_mask
from core.vision.normal_smoother import PathNormalSmoother

logger = logging.getLogger(__name__)

LABEL_NONE = 0
LABEL_LEFT = 1
LABEL_RIGHT = 2
LABEL_OVERLAP = 3


class JeansAutoWaypointsError(ValueError):
    """规划失败（缺标定、空输入、无有效航点等）。"""


class JeansAutoWaypoints:
    """
    在完整单 mesh 上生成与 scan.manual.path.yaml 同构的自动航点（一条路径）。
    """

    def __init__(
        self,
        spray_dist_mm: float = 150.0,
        row_spacing_mm: float = 60.0,
        point_spacing_mm: float = 100.0,
        image_size: tuple[int, int] = (1280, 800),
        camera_intrinsics: Optional[np.ndarray] = None,
        T_camera_to_base: Optional[np.ndarray] = None,
        depth_threshold_ratio: float = 0.1,  # 裤裆凸缺陷深度 / 掩码高度，低于此当单腿
        dedup_radius_mm: float = 30.0,  # 裆部重叠去重半径（mm），由外层传入，不按行距推算
        normal_smooth_window: int = 5,  # PathNormalSmoother 滑动窗口（路径序点数）
        mesh_unit: str = "m",  # 输入 mesh 顶点单位，仅内部换算，不改原对象
        align_outer_edge: bool = True,
    ):
        self.spray_dist_mm = float(spray_dist_mm)
        self.row_spacing_mm = float(row_spacing_mm)
        self.point_spacing_mm = float(point_spacing_mm)
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self.camera_intrinsics = None if camera_intrinsics is None else np.asarray(camera_intrinsics, dtype=np.float64)
        self.T_camera_to_base = None if T_camera_to_base is None else np.asarray(T_camera_to_base, dtype=np.float64)
        self.depth_threshold_ratio = float(depth_threshold_ratio)
        self.dedup_radius_mm = float(dedup_radius_mm)
        self.normal_smooth_window = int(normal_smooth_window)
        self.mesh_unit = str(mesh_unit).strip().lower()
        self.align_outer_edge = bool(align_outer_edge)
        if self.mesh_unit not in ("m", "mm"):
            raise JeansAutoWaypointsError(f"mesh_unit must be 'm' or 'mm', got {mesh_unit!r}")

    def plan(self, mesh: trimesh.Trimesh, masks_data: dict) -> dict:
        """
        :param mesh: 完整表面网格（不修改其 faces）
        :param masks_data: 已解析的 scan.masks.yaml
        :return: scan.auto.path.yaml 结构的 dict
        :raises JeansAutoWaypointsError: 缺 K/T、空输入、采不到点
        """
        self._require_camera()
        if mesh is None or len(getattr(mesh, "vertices", [])) < 10 or len(getattr(mesh, "faces", [])) < 1:
            raise JeansAutoWaypointsError("mesh is empty or too small")

        n_faces0 = int(len(mesh.faces))
        verts_m = self._vertices_meters(mesh)
        combined = self._rasterize_masks(masks_data)
        # 只借 2D 切裆打标签，不切 mesh；overlap_px=0，补缝带是旧「分腿重建」才要的。
        leg_masks = split_jeans_mask(
            combined,
            depth_threshold_ratio=self.depth_threshold_ratio,
            overlap_px=0.0,
        )
        uv, z_ok = self._project_vertices(verts_m)
        labels = self._label_vertices(uv, z_ok, leg_masks)

        full_tree = cKDTree(verts_m)
        vertex_normals = np.asarray(mesh.vertex_normals, dtype=np.float64)
        if vertex_normals.shape[0] != verts_m.shape[0]:
            raise JeansAutoWaypointsError("mesh.vertex_normals size mismatch")

        if len(leg_masks) == 1:
            face_keep = self._faces_for_labels(mesh, labels, {LABEL_LEFT, LABEL_RIGHT, LABEL_OVERLAP})
            samples = self._sample_leg_faces(mesh, verts_m, face_keep, full_tree, vertex_normals, leg_id=0)
            if not samples:
                raise JeansAutoWaypointsError("no waypoints sampled on the single-leg region")
        else:
            left_faces = self._faces_for_labels(mesh, labels, {LABEL_LEFT, LABEL_OVERLAP})
            right_faces = self._faces_for_labels(mesh, labels, {LABEL_RIGHT, LABEL_OVERLAP})
            left_pts = self._sample_leg_faces(mesh, verts_m, left_faces, full_tree, vertex_normals, leg_id=0)
            right_pts = self._sample_leg_faces(mesh, verts_m, right_faces, full_tree, vertex_normals, leg_id=1)
            if not left_pts and not right_pts:
                raise JeansAutoWaypointsError("no waypoints sampled on either leg")
            samples = self._concat_and_dedup(left_pts, right_pts)

        if int(len(mesh.faces)) != n_faces0:
            raise JeansAutoWaypointsError("mesh faces were mutated; this is a bug")

        smoother = PathNormalSmoother(window_size=self.normal_smooth_window)
        samples = smoother.smooth(samples)
        points = self._samples_to_waypoints(samples)
        if not points:
            raise JeansAutoWaypointsError("waypoint conversion produced an empty path")

        return {
            "paths": [{
                "path_id": 1,
                "name": "Auto Path",
                "points": points,
            }],
            "standoff_distance_mm": float(self.spray_dist_mm),
            "type": "auto",
            "coordinate_frame": "base_link",
        }

    def _require_camera(self) -> None:
        if self.camera_intrinsics is None or self.T_camera_to_base is None:
            raise JeansAutoWaypointsError(
                "camera_intrinsics and T_camera_to_base are required; refusing to plan without K/T"
            )
        k = np.asarray(self.camera_intrinsics, dtype=np.float64)
        t = np.asarray(self.T_camera_to_base, dtype=np.float64)
        if k.shape != (3, 3):
            raise JeansAutoWaypointsError(f"camera_intrinsics must be 3x3, got {k.shape}")
        if t.shape != (4, 4):
            raise JeansAutoWaypointsError(f"T_camera_to_base must be 4x4, got {t.shape}")

    def _vertices_meters(self, mesh: trimesh.Trimesh) -> np.ndarray:
        verts = np.asarray(mesh.vertices, dtype=np.float64)
        if self.mesh_unit == "mm":
            return verts / 1000.0
        return verts.copy()

    def _rasterize_masks(self, masks_data: dict) -> np.ndarray:
        if not isinstance(masks_data, dict):
            raise JeansAutoWaypointsError("masks_data must be a parsed dict")
        items = masks_data.get("masks", [])
        if not items:
            raise JeansAutoWaypointsError("no masks defined in masks_data")
        width, height = self.image_size
        canvas = np.zeros((height, width), dtype=np.uint8)
        n_poly = 0
        for item in items:
            for poly in item.get("polygons", []) or []:
                if poly is None or len(poly) < 3:
                    continue
                pts = np.asarray(poly, dtype=np.int32)
                if pts.ndim != 2 or pts.shape[1] < 2:
                    continue
                cv2.fillPoly(canvas, [pts[:, :2]], 255)
                n_poly += 1
        if n_poly == 0 or int(np.count_nonzero(canvas)) < 50:
            raise JeansAutoWaypointsError("mask area is empty or too small")
        return canvas > 0

    def _project_vertices(self, verts_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        k = np.asarray(self.camera_intrinsics, dtype=np.float64)
        t_cb = np.asarray(self.T_camera_to_base, dtype=np.float64)
        t_bc = np.linalg.inv(t_cb)
        r = t_bc[:3, :3]
        t = t_bc[:3, 3]
        p_cam = (r @ verts_m.T).T + t
        z = p_cam[:, 2]
        z_ok = z > 1e-6
        uv = np.full((verts_m.shape[0], 2), -1.0, dtype=np.float64)
        fx, fy = float(k[0, 0]), float(k[1, 1])
        cx, cy = float(k[0, 2]), float(k[1, 2])
        if np.any(z_ok):
            uv[z_ok, 0] = fx * p_cam[z_ok, 0] / z[z_ok] + cx
            uv[z_ok, 1] = fy * p_cam[z_ok, 1] / z[z_ok] + cy
        return uv, z_ok

    def _label_vertices(
        self,
        uv: np.ndarray,
        z_ok: np.ndarray,
        leg_masks: list[np.ndarray],
    ) -> np.ndarray:
        width, height = self.image_size
        labels = np.full(uv.shape[0], LABEL_NONE, dtype=np.int32)
        u = np.rint(uv[:, 0]).astype(np.int32)
        v = np.rint(uv[:, 1]).astype(np.int32)
        in_img = z_ok & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        if not np.any(in_img):
            raise JeansAutoWaypointsError("no mesh vertices project into the image; check K/T and mesh frame")

        if len(leg_masks) == 1:
            m0 = leg_masks[0]
            hit = np.zeros(uv.shape[0], dtype=bool)
            hit[in_img] = m0[v[in_img], u[in_img]]
            labels[hit] = LABEL_LEFT
            return labels

        left, right = leg_masks[0], leg_masks[1]
        in_l = np.zeros(uv.shape[0], dtype=bool)
        in_r = np.zeros(uv.shape[0], dtype=bool)
        in_l[in_img] = left[v[in_img], u[in_img]]
        in_r[in_img] = right[v[in_img], u[in_img]]
        labels[in_l & ~in_r] = LABEL_LEFT
        labels[in_r & ~in_l] = LABEL_RIGHT
        labels[in_l & in_r] = LABEL_OVERLAP
        if not np.any(labels != LABEL_NONE):
            raise JeansAutoWaypointsError("projected vertices miss both leg masks")
        return labels

    @staticmethod
    def _faces_for_labels(mesh: trimesh.Trimesh, labels: np.ndarray, accept: set[int]) -> np.ndarray:
        faces = np.asarray(mesh.faces, dtype=np.int64)
        accept_mask = np.zeros(labels.shape[0], dtype=bool)
        for lab in accept:
            accept_mask |= labels == lab
        return np.any(accept_mask[faces], axis=1)

    def _sample_leg_faces(
        self,
        mesh: trimesh.Trimesh,
        verts_m: np.ndarray,
        face_keep: np.ndarray,
        full_tree: cKDTree,
        vertex_normals: np.ndarray,
        leg_id: int,
    ) -> list[dict[str, Any]]:
        if not np.any(face_keep):
            logger.warning("leg %s has no faces after labeling", leg_id)
            return []
        faces = np.asarray(mesh.faces)[face_keep]
        leg_mesh = trimesh.Trimesh(vertices=verts_m, faces=faces, process=False)
        used = np.unique(faces.reshape(-1))
        pca_verts = verts_m[used]
        if pca_verts.shape[0] < 10:
            logger.warning("leg %s has too few vertices (%d)", leg_id, pca_verts.shape[0])
            return []
        samples = self._zigzag_sample(leg_mesh, pca_verts, full_tree, vertex_normals)
        for p in samples:
            p["leg_id"] = leg_id
        return samples

    def _pca_main_axis(self, vertices: np.ndarray) -> np.ndarray:
        centered = vertices - vertices.mean(axis=0)
        cov = np.cov(centered.T)
        _w, vecs = np.linalg.eigh(cov)
        main = np.asarray(vecs[:, 2], dtype=np.float64)
        if main[1] < 0.0:
            main = -main
        n = np.linalg.norm(main)
        if n < 1e-9:
            return np.array([0.0, 1.0, 0.0], dtype=np.float64)
        return main / n

    def _transverse_axis(self, main: np.ndarray) -> np.ndarray:
        trans = np.cross(np.array([1.0, 0.0, 0.0], dtype=np.float64), main)
        n = np.linalg.norm(trans)
        if n < 1e-5:
            return np.array([0.0, 1.0, 0.0], dtype=np.float64)
        return trans / n

    def _fit_outer_edge_axis(self, vertices: np.ndarray) -> tuple[np.ndarray, bool]:
        """conformal_sampler 外侧缝拟合，返回 (edge_axis, is_left_edge_better)。"""
        rough_main = self._pca_main_axis(vertices)
        rough_trans = self._transverse_axis(rough_main)
        v_long = vertices @ rough_main
        v_trans = vertices @ rough_trans
        min_l, max_l = float(v_long.min()), float(v_long.max())
        bins = np.linspace(min_l, max_l, 21)
        left_edge, right_edge = [], []
        for i in range(20):
            mask = (v_long >= bins[i]) & (v_long <= bins[i + 1])
            if not np.any(mask):
                continue
            bt, bl = v_trans[mask], v_long[mask]
            left_edge.append([bt[int(np.argmin(bt))], bl[int(np.argmin(bt))]])
            right_edge.append([bt[int(np.argmax(bt))], bl[int(np.argmax(bt))]])
        left_edge = np.asarray(left_edge, dtype=np.float64)
        right_edge = np.asarray(right_edge, dtype=np.float64)

        def fit_edge(edge_pts: np.ndarray) -> tuple[np.ndarray, float]:
            if edge_pts.shape[0] < 2:
                return rough_main, float("inf")
            centered = edge_pts - edge_pts.mean(axis=0)
            _u, s, vh = np.linalg.svd(centered, full_matrices=False)
            d_trans, d_long = float(vh[0, 0]), float(vh[0, 1])
            if d_long < 0.0:
                d_trans, d_long = -d_trans, -d_long
            err = float(s[1]) if s.size > 1 else 0.0
            axis = d_trans * rough_trans + d_long * rough_main
            n = np.linalg.norm(axis)
            if n < 1e-9:
                return rough_main, err
            return axis / n, err

        left_axis, left_err = fit_edge(left_edge)
        right_axis, right_err = fit_edge(right_edge)
        if left_err <= right_err:
            logger.info("leg outer-edge: left seam err=%.4f (right=%.4f)", left_err, right_err)
            return left_axis, True
        logger.info("leg outer-edge: right seam err=%.4f (left=%.4f)", right_err, left_err)
        return right_axis, False

    def _zigzag_sample(
        self,
        slice_mesh: trimesh.Trimesh,
        pca_verts: np.ndarray,
        full_tree: cKDTree,
        vertex_normals: np.ndarray,
    ) -> list[dict[str, Any]]:
        row_s = self.row_spacing_mm / 1000.0
        pt_s = self.point_spacing_mm / 1000.0
        if self.align_outer_edge:
            edge_axis, is_left = self._fit_outer_edge_axis(pca_verts)
        else:
            edge_axis, is_left = self._pca_main_axis(pca_verts), True

        plane_normal = np.cross(np.array([1.0, 0.0, 0.0], dtype=np.float64), edge_axis)
        n = np.linalg.norm(plane_normal)
        if n < 1e-8:
            plane_normal = self._transverse_axis(edge_axis)
        else:
            plane_normal = plane_normal / n
        if plane_normal[1] < 0.0:
            plane_normal = -plane_normal

        projections = pca_verts @ plane_normal
        min_proj, max_proj = float(projections.min()), float(projections.max())
        if max_proj - min_proj < row_s * 0.5:
            logger.warning("leg span %.1f mm is smaller than half row spacing", (max_proj - min_proj) * 1000.0)
        if is_left:
            slice_projs = np.arange(min_proj + row_s / 2.0, max_proj + 1e-12, row_s)
        else:
            slice_projs = np.arange(max_proj - row_s / 2.0, min_proj - 1e-12, -row_s)

        zigzag: list[dict[str, Any]] = []
        direction_forward = True
        for proj in slice_projs:
            lines = trimesh.intersections.mesh_plane(
                slice_mesh,
                plane_normal=plane_normal,
                plane_origin=plane_normal * float(proj),
            )
            if lines is None or len(lines) == 0:
                continue
            pts = np.unique(np.round(np.asarray(lines, dtype=np.float64).reshape(-1, 3), decimals=4), axis=0)
            if pts.shape[0] < 2:
                continue
            order = np.argsort(pts @ edge_axis)
            sorted_pts = pts[order]
            diffs = np.diff(sorted_pts, axis=0)
            dists = np.linalg.norm(diffs, axis=1)
            cum = np.insert(np.cumsum(dists), 0, 0.0)
            total = float(cum[-1])
            if total < pt_s * 0.25:
                continue
            sample_d = np.arange(0.0, total, pt_s)
            if sample_d.size == 0:
                continue
            sampled = np.zeros((sample_d.size, 3), dtype=np.float64)
            for i in range(3):
                sampled[:, i] = np.interp(sample_d, cum, sorted_pts[:, i])
            if not direction_forward:
                sampled = sampled[::-1]
            _, idx = full_tree.query(sampled)
            normals = vertex_normals[idx]
            for i_pt, (p, nrm) in enumerate(zip(sampled, normals)):
                nlen = float(np.linalg.norm(nrm))
                if nlen < 1e-9:
                    nrm = np.array([1.0, 0.0, 0.0], dtype=np.float64)
                else:
                    nrm = nrm / nlen
                zigzag.append({
                    "point": p.astype(np.float64).tolist(),
                    "normal": nrm.astype(np.float64).tolist(),
                    "is_jump": bool(len(zigzag) > 0 and i_pt == 0),
                })
            direction_forward = not direction_forward
        return zigzag

    def _concat_and_dedup(
        self,
        left_pts: list[dict[str, Any]],
        right_pts: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        if not right_pts:
            return list(left_pts)
        if not left_pts:
            return list(right_pts)

        radius_m = max(self.dedup_radius_mm, 0.0) / 1000.0
        keep_right = np.ones(len(right_pts), dtype=bool)
        if radius_m > 0.0:
            left_xyz = np.asarray([p["point"] for p in left_pts], dtype=np.float64)
            right_xyz = np.asarray([p["point"] for p in right_pts], dtype=np.float64)
            d_lr, _ = cKDTree(left_xyz).query(right_xyz, k=1)
            drop = d_lr <= radius_m
            keep_right[drop] = False
            logger.info(
                "leg-seam dedup: dropped %d / %d right-leg points (radius=%.1f mm)",
                int(np.count_nonzero(drop)),
                len(right_pts),
                self.dedup_radius_mm,
            )

        kept_right = [p for p, k in zip(right_pts, keep_right) if k]
        if kept_right:
            first = dict(kept_right[0])
            first["is_jump"] = True
            kept_right[0] = first
        return list(left_pts) + kept_right

    def _cam_from_base(self) -> tuple[np.ndarray, np.ndarray]:
        t_bc = np.linalg.inv(np.asarray(self.T_camera_to_base, dtype=np.float64))
        return t_bc[:3, :3], t_bc[:3, 3]

    def _samples_to_waypoints(self, samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
        k = np.asarray(self.camera_intrinsics, dtype=np.float64)
        fx, fy = float(k[0, 0]), float(k[1, 1])
        cx, cy = float(k[0, 2]), float(k[1, 2])
        r_bc, t_bc = self._cam_from_base()
        points: list[dict[str, Any]] = []
        prev_x_tool: Optional[np.ndarray] = None

        for i, s in enumerate(samples):
            p_m = np.asarray(s["point"], dtype=np.float64)
            n_base = np.asarray(s["normal"], dtype=np.float64)
            nlen = float(np.linalg.norm(n_base))
            if nlen < 1e-9:
                n_base = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            else:
                n_base = n_base / nlen

            p_surf_mm = p_m * 1000.0
            p_tcp_mm = p_surf_mm + self.spray_dist_mm * n_base
            z_tool = -n_base
            if prev_x_tool is None:
                x_ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                if abs(float(np.dot(z_tool, x_ref))) > 0.92:
                    x_ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            else:
                x_ref = prev_x_tool
            y_tool = np.cross(z_tool, x_ref)
            ylen = float(np.linalg.norm(y_tool))
            if ylen < 1e-9:
                y_tool = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            else:
                y_tool = y_tool / ylen
            x_tool = np.cross(y_tool, z_tool)
            xlen = float(np.linalg.norm(x_tool))
            if xlen < 1e-9:
                x_tool = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            else:
                x_tool = x_tool / xlen
            prev_x_tool = x_tool
            euler = R_tool.from_matrix(np.column_stack((x_tool, y_tool, z_tool))).as_euler("xyz", degrees=True)

            p_cam_m = r_bc @ p_m + t_bc
            n_cam = r_bc @ n_base
            ncam_len = float(np.linalg.norm(n_cam))
            if ncam_len > 1e-9:
                n_cam = n_cam / ncam_len
            if p_cam_m[2] > 1e-6:
                u = fx * p_cam_m[0] / p_cam_m[2] + cx
                v = fy * p_cam_m[1] / p_cam_m[2] + cy
                p_tcp_cam = p_cam_m + (self.spray_dist_mm / 1000.0) * n_cam
                if p_tcp_cam[2] > 1e-6:
                    u_tcp = fx * p_tcp_cam[0] / p_tcp_cam[2] + cx
                    v_tcp = fy * p_tcp_cam[1] / p_tcp_cam[2] + cy
                    proj = [round(float(u_tcp - u), 1), round(float(v_tcp - v), 1)]
                else:
                    proj = [0.0, 0.0]
            else:
                u, v = 0.0, 0.0
                proj = [0.0, 0.0]

            points.append({
                "index": i + 1,
                "pixel": [int(round(u)), int(round(v))],
                "surface_point_cam_mm": [round(float(x) * 1000.0, 2) for x in p_cam_m],
                "surface_point_base_mm": [round(float(x), 2) for x in p_surf_mm],
                "surface_normal_base": [round(float(x), 4) for x in n_base],
                "surface_normal_cam": [round(float(x), 4) for x in n_cam],
                "standoff_distance_mm": round(float(self.spray_dist_mm), 1),
                "tcp_pose_base": {
                    "x": round(float(p_tcp_mm[0]), 2),
                    "y": round(float(p_tcp_mm[1]), 2),
                    "z": round(float(p_tcp_mm[2]), 2),
                    "rx": round(float(euler[0]), 2),
                    "ry": round(float(euler[1]), 2),
                    "rz": round(float(euler[2]), 2),
                },
                "normal_2d_proj": proj,
                "is_jump": bool(s.get("is_jump", False)),
                "leg_id": int(s.get("leg_id", 0)),
            })
        return points
