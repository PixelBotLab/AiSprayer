import os
import sys
import time
import queue
import logging
import threading
import traceback
import multiprocessing as mp
from logging.handlers import QueueHandler

import cv2
import numpy as np
import yaml
import trimesh

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from core.vision.reconstruction import (
    PoissonReconstructor, 
    k_matrix_to_intrinsics, 
    depth_to_point_cloud
)
from core.config import SprayerConfig

logger = logging.getLogger(__name__)

# open3d 0.19 (aarch64) 内置 PoissonRecon 的等值面提取是竞态的: 同一份输入连跑会得到不同面数
# (71222/71223/71224), 偶发打印 "Failed to close loop" 并进一步升级为挂死或段错误,
# 会把整个后端进程(含相机/机械臂/SAM 服务)一起带走。
# 因此把重建整体放进 spawn 子进程执行: 崩溃只死子进程, 父进程按退出码识别后自动重试。
RECON_MAX_ATTEMPTS = 3
# 单次超时: 正常一次约 4s(含子进程启动+import open3d 约 6s), 60s = 10 倍余量,
# 也容得下将来把 poisson_depth 从 8 提到 9 (约 4~8 倍耗时)。
# 超时即判定为挂死并杀掉重试; 前端是裸 fetch 无超时, 最坏 3×60s 仍在浏览器容忍范围内。
RECON_SUBPROCESS_TIMEOUT_S = 60.0


def _reconstruction_worker(log_queue: mp.Queue, res_conn, template_path: str, template_name: str):
    """Subprocess worker: runs the whole Poisson reconstruction in an isolated process."""
    try:
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.INFO)
        root_logger.handlers = [QueueHandler(log_queue)]

        result = InteractiveReconstructionService()._reconstruct_surface_impl(template_path, template_name)
        res_conn.send({"success": True, "result": result})
    except Exception as e:
        res_conn.send({
            "success": False,
            "error": str(e),
            "exc_type": type(e).__name__,
            "traceback": traceback.format_exc(),
        })
    finally:
        log_queue.put(None)


def _drain_log_queue(log_queue: mp.Queue, proc: mp.Process):
    """Forwards the worker's log records into this process's handlers (same as the POI optimizer worker)."""
    while True:
        try:
            record = log_queue.get(timeout=0.1)
            if record is None:
                break
            logging.getLogger(record.name).handle(record)
        except queue.Empty:
            if not proc.is_alive():
                while True:
                    try:
                        record = log_queue.get_nowait()
                        if record is None:
                            break
                        logging.getLogger(record.name).handle(record)
                    except queue.Empty:
                        break
                break


def _reraise_worker_error(res: dict):
    """Re-raises the worker's exception with its original type so api.py keeps mapping HTTP status codes."""
    err = res.get("error", "Unknown error")
    tb = res.get("traceback", "")
    exc_type = res.get("exc_type") or ""
    logger.error(f"Reconstruction worker failed ({exc_type}): {err}\n{tb}")
    if exc_type == "FileNotFoundError":
        raise FileNotFoundError(err)
    if exc_type == "ValueError":
        raise ValueError(err)
    raise RuntimeError(f"Reconstruction worker error: {err}")


def run_reconstruction_subprocess(
    template_path: str,
    template_name: str,
    max_attempts: int = RECON_MAX_ATTEMPTS,
    timeout_s: float = RECON_SUBPROCESS_TIMEOUT_S,
) -> dict:
    """
    Spawns an isolated worker process for Poisson reconstruction, retrying on crash/hang.

    竞态是偶发的, 重跑一次通常就过; 确定性错误(缺文件/掩码为空)则原样抛出, 不浪费重试。
    """
    last_reason = "unknown"
    for attempt in range(1, max_attempts + 1):
        ctx = mp.get_context("spawn")
        log_queue = ctx.Queue()
        parent_conn, child_conn = ctx.Pipe(duplex=False)

        proc = ctx.Process(
            target=_reconstruction_worker,
            args=(log_queue, child_conn, template_path, template_name),
        )
        t0 = time.time()
        proc.start()
        # 必须关掉父进程手里的写端: 否则子进程崩溃时管道仍有写入者,
        # parent_conn.poll() 永远等不到 EOF, 只能白白挂满整个超时才发现
        child_conn.close()
        log_thread = threading.Thread(target=_drain_log_queue, args=(log_queue, proc), daemon=True)
        log_thread.start()

        res = None
        timed_out = False
        try:
            if parent_conn.poll(timeout_s):
                try:
                    res = parent_conn.recv()
                except (EOFError, OSError):
                    pass            # 管道已关: 子进程死了但没留下结果
            else:
                timed_out = True    # 等满超时 = Poisson 挂死
        except (EOFError, OSError, ValueError):
            timed_out = True

        if timed_out and proc.is_alive():
            proc.terminate()        # 已判定挂死, 立刻杀, 不再白等下面的 join

        log_thread.join(timeout=3.0)
        proc.join(timeout=5.0)
        if proc.is_alive():
            proc.kill()             # SIGTERM 杀不掉(卡在 C++ 里)时兜底
            proc.join()
        parent_conn.close()

        elapsed = time.time() - t0

        if res is not None:
            if not res.get("success"):
                _reraise_worker_error(res)
            return res["result"]

        # 没拿到结果 = 被信号打死(段错误)或超时挂死, 两者都是竞态的表现, 重试
        last_reason = (f"hung and was killed after {elapsed:.0f}s" if timed_out
                       else f"died without returning a result (exit code {proc.exitcode})")
        logger.error(
            f"⚠️ [Reconstruction] 第 {attempt}/{max_attempts} 次尝试{last_reason}"
            f"{'' if attempt >= max_attempts else ', 自动重试...'}"
        )

    raise RuntimeError(
        f"Surface reconstruction failed {max_attempts} times for template '{template_name}' "
        f"(last attempt: {last_reason}). open3d Poisson 在多线程下偶发崩溃/挂死, 请重试。"
    )


class InteractiveReconstructionService:
    def __init__(self):
        self.calib_dir = os.path.abspath(os.path.join(PROJECT_ROOT, "data", "calib"))

    def get_latest_calibration(self) -> tuple[np.ndarray, np.ndarray | None, str]:
        """
        Locates the latest hand-eye calibration result from data/calib/
        Returns:
            (T_camera_to_base_4x4_in_meters, intrinsics_k_3x3, source_description)
        """
        # 1. Search data/calib for latest calibration_result.yaml
        if os.path.exists(self.calib_dir):
            sessions = sorted(
                [d for d in os.listdir(self.calib_dir) if os.path.isdir(os.path.join(self.calib_dir, d))],
                reverse=True
            )
            for sess in sessions:
                res_path = os.path.join(self.calib_dir, sess, "calibration_result.yaml")
                if os.path.exists(res_path):
                    try:
                        with open(res_path, 'r', encoding='utf-8') as f:
                            data = yaml.safe_load(f) or {}
                        
                        t_mat = data.get("T_base_camera") or data.get("T_camera_to_base")
                        if t_mat:
                            T = np.array(t_mat, dtype=np.float64)
                            # Convert translation from mm to meters
                            T[0, 3] /= 1000.0
                            T[1, 3] /= 1000.0
                            T[2, 3] /= 1000.0
                            
                            intr_k = None
                            cam_params = data.get("camera_params", {})
                            if "intrinsic_matrix" in cam_params:
                                intr_k = np.array(cam_params["intrinsic_matrix"], dtype=np.float64)
                                
                            err = data.get("metadata", {}).get("reprojection_error_mm", 0.0)
                            desc = f"{sess} (Reprojection Error: {err:.3f} mm)"
                            logger.info(f"Loaded latest calibration from: {res_path} ({desc})")
                            return T, intr_k, desc
                    except Exception as e:
                        logger.warning(f"Error reading calibration file {res_path}: {e}")

        # 2. Fallback to SprayerConfig global config
        try:
            cfg = SprayerConfig()
            if cfg.T_camera_to_base is not None:
                T = np.array(cfg.T_camera_to_base, dtype=np.float64)
                desc = f"Global config ({cfg.calib_path})"
                logger.info(f"Loaded calibration from global config: {desc}")
                return T, None, desc
        except Exception as e:
            logger.warning(f"Error reading global SprayerConfig calibration: {e}")

        # 3. Fallback to Identity
        logger.warning("No calibration result found. Falling back to Identity matrix.")
        return np.eye(4, dtype=np.float64), None, "Identity (Uncalibrated)"

    def rasterize_masks(self, masks_yaml_path: str, height: int, width: int) -> np.ndarray:
        """
        Loads all polygons from scan.masks.yaml and rasterizes them into a single 2D boolean mask
        """
        if not os.path.exists(masks_yaml_path):
            raise FileNotFoundError(f"Masks file not found: {masks_yaml_path}")

        class SafeLoaderWithTuples(yaml.SafeLoader):
            pass
        def tuple_constructor(loader, node):
            return list(loader.construct_sequence(node))
        SafeLoaderWithTuples.add_constructor('tag:yaml.org,2002:python/tuple', tuple_constructor)
        SafeLoaderWithTuples.add_constructor('!tuple', tuple_constructor)

        with open(masks_yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.load(f, Loader=SafeLoaderWithTuples) or {}

        mask_items = data.get("masks", [])
        if not mask_items:
            raise ValueError("No masks defined in scan.masks.yaml")

        combined_mask = np.zeros((height, width), dtype=np.uint8)
        polygon_count = 0

        for m in mask_items:
            polygons = m.get("polygons", [])
            for poly in polygons:
                if len(poly) >= 3:
                    pts = np.array(poly, dtype=np.int32)
                    cv2.fillPoly(combined_mask, [pts], 255)
                    polygon_count += 1

        active_pixels = int(np.count_nonzero(combined_mask))
        logger.info(f"Rasterized {len(mask_items)} mask objects ({polygon_count} polygons, {active_pixels} active pixels) for {width}x{height}")
        
        if active_pixels < 50:
            raise ValueError("Mask area is too small or empty (less than 50 pixels).")

        return combined_mask > 0

    def reconstruct_surface(self, template_path: str, template_name: str) -> dict:
        """
        Public entry: runs Poisson surface reconstruction inside an isolated subprocess.

        真实工作在 _reconstruct_surface_impl 里, 由 run_reconstruction_subprocess 派生的子进程执行,
        以免 open3d Poisson 偶发的段错误/挂死拖垮整个后端。
        """
        return run_reconstruction_subprocess(template_path, template_name)

    def _reconstruct_surface_impl(self, template_path: str, template_name: str) -> dict:
        """
        Executes Poisson surface reconstruction using depth data, masks, and calibration.
        Runs inside the spawned worker process — do not call it directly from the API layer.
        """
        logger.info(f"==================================================")
        logger.info(f"🚀 Starting Surface Reconstruction for template: '{template_name}'")
        logger.info(f"==================================================")

        # 1. Check required files
        depth_png_path = os.path.join(template_path, "scan.depth.png")
        depth_npy_path = os.path.join(template_path, "scan.depth.npy")
        masks_path = os.path.join(template_path, "scan.masks.yaml")
        params_path = os.path.join(template_path, "scan.params.yaml")
        color_jpg_path = os.path.join(template_path, "scan.color.jpg")
        color_legacy_path = os.path.join(template_path, "scan.jpg")
        color_path = color_jpg_path if os.path.exists(color_jpg_path) else (color_legacy_path if os.path.exists(color_legacy_path) else None)

        depth_path = depth_png_path if os.path.exists(depth_png_path) else (depth_npy_path if os.path.exists(depth_npy_path) else None)

        if not depth_path:
            logger.error(f"Reconstruction failed: 'scan.depth.png' not found in {template_path}")
            raise FileNotFoundError("Depth data 'scan.depth.png' not found. Please capture data first.")

        if not os.path.exists(masks_path):
            logger.error(f"Reconstruction failed: 'scan.masks.yaml' not found in {template_path}")
            raise FileNotFoundError("Segmentation data 'scan.masks.yaml' not found. Please segment and save masks first.")

        # 2. Load Depth Image (16-bit)
        try:
            if depth_path.endswith('.npy'):
                depth_image = np.load(depth_path)
            else:
                depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            if depth_image is None:
                raise ValueError(f"cv2.imread returned None for {depth_path}")
            h, w = depth_image.shape
            logger.info(f"Loaded depth map: {w}x{h} (min={depth_image.min()}mm, max={depth_image.max()}mm)")
        except Exception as e:
            logger.error(f"Failed to load depth map: {e}")
            raise RuntimeError(f"Invalid depth data: {e}")

        # 3. Load or determine Camera Intrinsics
        intrinsics_k = None
        if os.path.exists(params_path):
            try:
                with open(params_path, 'r', encoding='utf-8') as f:
                    pdata = yaml.safe_load(f) or {}
                k_list = pdata.get("camera_params", {}).get("intrinsic_matrix")
                if k_list:
                    intrinsics_k = np.array(k_list, dtype=np.float64)
                    logger.info("Loaded camera intrinsics from scan.params.yaml")
            except Exception as e:
                logger.warning(f"Could not read intrinsics from scan.params.yaml: {e}")

        # 4. Load Hand-Eye Calibration
        T_camera_to_base, calib_k, calib_desc = self.get_latest_calibration()
        if intrinsics_k is None and calib_k is not None:
            intrinsics_k = calib_k
            logger.info("Using camera intrinsics from calibration result")

        if intrinsics_k is None:
            # Standard Orbbec default fallback
            intrinsics_k = np.array([
                [611.68, 0.0, float(w) / 2.0],
                [0.0, 611.69, float(h) / 2.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)
            logger.warning(f"Using default camera intrinsics K for resolution {w}x{h}")

        # 5. Rasterize Masks from scan.masks.yaml
        unified_mask_2d = self.rasterize_masks(masks_path, height=h, width=w)

        # 6. Initialize Reconstructor
        reconstructor = PoissonReconstructor(
            T_camera_to_base=T_camera_to_base,
            intrinsics_k=intrinsics_k,
            segmenter=None,
            z_min=100.0,
            z_max=3000.0,
            mask_erode_px=1,
            flying_pixel_max_grad=50.0,
            poisson_depth=8,
            density_threshold=0.15,
            voxel_size=0.003,
            normal_radius=0.03,
            smooth_iterations=20
        )

        intrinsics = k_matrix_to_intrinsics(intrinsics_k)
        valid_depth_mask = (depth_image > reconstructor.z_min) & (depth_image < reconstructor.z_max)

        # Inpaint holes inside mask
        holes_mask = unified_mask_2d & (~valid_depth_mask)
        if np.any(holes_mask):
            hole_count = int(np.sum(holes_mask))
            logger.info(f"Inpainting {hole_count} depth holes inside mask using OpenCV Navier-Stokes...")
            depth_f32 = depth_image.astype(np.float32)
            inpaint_mask = holes_mask.astype(np.uint8) * 255
            filled_depth = cv2.inpaint(depth_f32, inpaint_mask, inpaintRadius=5, flags=cv2.INPAINT_NS)
            depth_image[holes_mask] = filled_depth[holes_mask]
            valid_depth_mask = (depth_image > reconstructor.z_min) & (depth_image < reconstructor.z_max)

        # Erode mask to avoid boundary flying points
        eroded_mask = reconstructor._erode_mask(unified_mask_2d, reconstructor.mask_erode_px)
        flying_pixel_valid = reconstructor._flying_pixel_mask(depth_image, reconstructor.flying_pixel_max_grad) \
            if reconstructor.flying_pixel_max_grad > 0 else np.ones_like(valid_depth_mask, dtype=bool)

        combined_mask = eroded_mask & valid_depth_mask & flying_pixel_valid
        
        # 7. Convert Depth to 2.5D Camera Point Cloud
        raw_point_cloud = depth_to_point_cloud(depth_image, intrinsics)

        # 8. Perform 3D Poisson Reconstruction
        logger.info("Executing Poisson surface reconstruction and base coordinate alignment...")
        mesh: trimesh.Trimesh = reconstructor.reconstruct_mesh(raw_point_cloud, combined_mask)

        # 9. Save output mesh files
        ply_path = os.path.join(template_path, "scan.mesh.ply")
        stl_path = os.path.join(template_path, "scan.mesh.stl")

        mesh.export(ply_path)
        logger.info(f"Saved reconstructed PLY mesh: {ply_path} ({len(mesh.vertices)} vertices, {len(mesh.faces)} faces)")

        mesh.export(stl_path)
        logger.info(f"Saved reconstructed STL mesh: {stl_path}")

        logger.info(f"✅ Surface reconstruction successfully finished for template '{template_name}'.")

        return {
            "status": "success",
            "template": template_name,
            "calibration_source": calib_desc,
            "vertices": len(mesh.vertices),
            "faces": len(mesh.faces),
            "is_watertight": bool(mesh.is_watertight),
            "files": ["scan.mesh.ply", "scan.mesh.stl"]
        }

reconstruction_service = InteractiveReconstructionService()
