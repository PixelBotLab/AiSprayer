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


def _persistent_worker_entry(work_conn, log_queue: mp.Queue):
    """
    Subprocess worker: runs in a persistent isolated process.
    Pre-imports open3d, trimesh, and PoissonReconstructor once during startup.
    Handles multiple reconstruction requests sequentially over the work_conn pipe.
    """
    try:
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.INFO)
        root_logger.handlers = [QueueHandler(log_queue)]

        # Override single-thread limits inherited from main server process
        # so Open3D & Poisson solvers run multi-threaded
        for _name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
            os.environ[_name] = "4"

        # Pre-import heavy dependencies to eliminate cold-start lag
        import open3d as o3d  # noqa: F401
        import trimesh  # noqa: F401
        from core.vision.reconstruction import PoissonReconstructor  # noqa: F401

        svc = InteractiveReconstructionService()

        while True:
            try:
                msg = work_conn.recv()
            except (EOFError, KeyboardInterrupt):
                break

            if not msg or msg.get("action") == "stop":
                break

            if msg.get("action") == "reconstruct":
                template_path = msg.get("template_path")
                template_name = msg.get("template_name")
                try:
                    result = svc._reconstruct_surface_impl(template_path, template_name)
                    work_conn.send({"success": True, "result": result})
                except Exception as e:
                    work_conn.send({
                        "success": False,
                        "error": str(e),
                        "exc_type": type(e).__name__,
                        "traceback": traceback.format_exc(),
                    })
    except Exception as e:
        try:
            work_conn.send({
                "success": False,
                "error": f"Worker crashed: {e}",
                "exc_type": type(e).__name__,
                "traceback": traceback.format_exc(),
            })
        except Exception:
            pass
    finally:
        try:
            log_queue.put(None)
        except Exception:
            pass


def _drain_persistent_log_queue(log_queue: mp.Queue, stop_event: threading.Event):
    """Forwards log records from the persistent worker queue into the parent logger."""
    while not stop_event.is_set():
        try:
            record = log_queue.get(timeout=0.1)
            if record is None:
                break
            logging.getLogger(record.name).handle(record)
        except queue.Empty:
            continue

    # Drain any remaining logs
    while True:
        try:
            record = log_queue.get_nowait()
            if record is None:
                break
            logging.getLogger(record.name).handle(record)
        except queue.Empty:
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


class ReconstructionWorkerManager:
    """
    Manages a persistent, pre-warmed worker process for Poisson surface reconstruction.

    Protects the main FastAPI server from Open3D Poisson segfaults / race-conditions on ARM64
    while eliminating the ~6.2s cold import penalty on every reconstruction request.
    """
    def __init__(self, timeout_s: float = RECON_SUBPROCESS_TIMEOUT_S, max_attempts: int = RECON_MAX_ATTEMPTS):
        self.timeout_s = timeout_s
        self.max_attempts = max_attempts
        self._lock = threading.Lock()
        self._proc: mp.Process | None = None
        self._parent_conn = None
        self._log_queue: mp.Queue | None = None
        self._log_thread: threading.Thread | None = None
        self._stop_event: threading.Event | None = None

    def _start_worker(self):
        """Starts a new persistent worker process."""
        self._terminate_worker()
        ctx = mp.get_context("spawn")
        self._log_queue = ctx.Queue()
        self._parent_conn, child_conn = ctx.Pipe(duplex=True)
        self._stop_event = threading.Event()

        self._proc = ctx.Process(
            target=_persistent_worker_entry,
            args=(child_conn, self._log_queue),
            daemon=True
        )
        self._proc.start()
        child_conn.close()

        self._log_thread = threading.Thread(
            target=_drain_persistent_log_queue,
            args=(self._log_queue, self._stop_event),
            daemon=True
        )
        self._log_thread.start()
        logger.info(f"🚀 [ReconstructionWorker] Persistent worker process launched (PID: {self._proc.pid}).")

    def _ensure_worker(self):
        """Ensures the worker process is running."""
        if self._proc is None or not self._proc.is_alive():
            self._start_worker()

    def warmup(self):
        """Pre-warms the worker in the background on startup."""
        with self._lock:
            self._ensure_worker()

    def _terminate_worker(self):
        """Terminates the current worker process and cleans up resources."""
        if self._stop_event:
            self._stop_event.set()

        if self._parent_conn:
            try:
                self._parent_conn.send({"action": "stop"})
            except Exception:
                pass

        if self._proc and self._proc.is_alive():
            self._proc.join(timeout=1.5)
            if self._proc.is_alive():
                self._proc.terminate()
                self._proc.join(timeout=1.0)
            if self._proc.is_alive():
                self._proc.kill()
                self._proc.join()

        if self._parent_conn:
            try:
                self._parent_conn.close()
            except Exception:
                pass
            self._parent_conn = None

        if self._log_thread and self._log_thread.is_alive():
            self._log_thread.join(timeout=2.0)
            self._log_thread = None

        self._proc = None
        self._log_queue = None
        self._stop_event = None

    def shutdown(self):
        """Clean shutdown of worker process."""
        with self._lock:
            logger.info("🛑 [ReconstructionWorker] Shutting down persistent worker process...")
            self._terminate_worker()

    def execute(self, template_path: str, template_name: str) -> dict:
        """
        Executes reconstruction through the persistent warm worker.
        If the worker hangs or segfaults (Open3D RK3588 bug), restarts and retries automatically.
        """
        with self._lock:
            last_reason = "unknown"
            for attempt in range(1, self.max_attempts + 1):
                self._ensure_worker()
                t0 = time.time()

                try:
                    self._parent_conn.send({
                        "action": "reconstruct",
                        "template_path": template_path,
                        "template_name": template_name,
                    })
                except (BrokenPipeError, OSError, EOFError) as e:
                    logger.warning(f"⚠️ [ReconstructionWorker] Pipe broken on send: {e}. Restarting worker...")
                    self._terminate_worker()
                    continue

                res = None
                timed_out = False
                try:
                    if self._parent_conn.poll(self.timeout_s):
                        try:
                            res = self._parent_conn.recv()
                        except (EOFError, OSError):
                            pass
                    else:
                        timed_out = True
                except (EOFError, OSError, ValueError):
                    timed_out = True

                elapsed = time.time() - t0

                if timed_out or res is None:
                    exitcode = getattr(self._proc, "exitcode", None)
                    last_reason = (f"hung and timed out after {elapsed:.0f}s" if timed_out
                                   else f"process died/segfaulted without result (exit code {exitcode})")
                    logger.error(
                        f"⚠️ [Reconstruction] Attempt {attempt}/{self.max_attempts} {last_reason}"
                        f"{'' if attempt >= self.max_attempts else ', restarting worker and retrying...'}"
                    )
                    self._terminate_worker()
                    continue

                if not res.get("success"):
                    _reraise_worker_error(res)
                return res["result"]

            raise RuntimeError(
                f"Surface reconstruction failed {self.max_attempts} times for template '{template_name}' "
                f"(last attempt: {last_reason})."
            )


_worker_manager = ReconstructionWorkerManager()


def run_reconstruction_subprocess(
    template_path: str,
    template_name: str,
    max_attempts: int = RECON_MAX_ATTEMPTS,
    timeout_s: float = RECON_SUBPROCESS_TIMEOUT_S,
) -> dict:
    """Legacy helper: delegates to the persistent worker manager."""
    return _worker_manager.execute(template_path, template_name)


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

    def warmup(self):
        """Pre-warms the worker in the background on service startup."""
        _worker_manager.warmup()

    def shutdown(self):
        """Terminates the persistent worker process on service shutdown."""
        _worker_manager.shutdown()

    def reconstruct_surface(self, template_path: str, template_name: str) -> dict:
        """
        Public entry: runs Poisson surface reconstruction inside an isolated, persistent warm worker process.
        """
        return _worker_manager.execute(template_path, template_name)

    def _reconstruct_surface_impl(self, template_path: str, template_name: str) -> dict:
        """
        Executes Poisson surface reconstruction using depth data, masks, and calibration.
        Runs inside the worker process — do not call it directly from the API layer.
        """
        t_start = time.perf_counter()
        logger.info("==================================================")
        logger.info(f"🚀 Starting Surface Reconstruction for template: '{template_name}'")
        logger.info("==================================================")

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
        t_mask = time.perf_counter()
        t_load_and_mask_s = round(t_mask - t_start, 3)
        active_pixel_count = int(np.count_nonzero(unified_mask_2d))
        logger.info(f"⏱️ [Reconstruct Step 1] Depth, calib & masks loaded in {t_load_and_mask_s:.2f}s ({active_pixel_count} mask pixels)")

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
            smooth_iterations=20,
            n_threads=4
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
        t_pcd = time.perf_counter()
        t_inpaint_and_pcd_s = round(t_pcd - t_mask, 3)
        active_points = int(np.count_nonzero(combined_mask))
        logger.info(f"⏱️ [Reconstruct Step 2] Inpaint & PCD extraction in {t_inpaint_and_pcd_s:.2f}s ({active_points} valid points)")

        # 8. Perform 3D Poisson Reconstruction
        logger.info("Executing Poisson surface reconstruction and base coordinate alignment...")
        mesh: trimesh.Trimesh = reconstructor.reconstruct_mesh(raw_point_cloud, combined_mask)
        t_mesh = time.perf_counter()
        t_poisson_mesh_s = round(t_mesh - t_pcd, 3)
        logger.info(f"⏱️ [Reconstruct Step 3] Poisson mesh & Taubin smoothing in {t_poisson_mesh_s:.2f}s ({len(mesh.vertices)} vertices, {len(mesh.faces)} faces)")

        # 9. Save output mesh files
        ply_path = os.path.join(template_path, "scan.mesh.ply")
        stl_path = os.path.join(template_path, "scan.mesh.stl")

        mesh.export(ply_path)
        logger.info(f"Saved reconstructed PLY mesh: {ply_path} ({len(mesh.vertices)} vertices, {len(mesh.faces)} faces)")

        mesh.export(stl_path)
        logger.info(f"Saved reconstructed STL mesh: {stl_path}")
        t_export = time.perf_counter()
        t_export_s = round(t_export - t_mesh, 3)
        t_total_compute_s = round(t_export - t_start, 3)

        logger.info(f"⏱️ [Reconstruct Step 4] Mesh files exported in {t_export_s:.2f}s")
        logger.info(f"✅ Surface reconstruction successfully finished in {t_total_compute_s:.2f}s for template '{template_name}'.")

        return {
            "status": "success",
            "template": template_name,
            "calibration_source": calib_desc,
            "vertices": len(mesh.vertices),
            "faces": len(mesh.faces),
            "is_watertight": bool(mesh.is_watertight),
            "files": ["scan.mesh.ply", "scan.mesh.stl"],
            "elapsed_seconds": t_total_compute_s,
            "timings": {
                "load_and_mask_s": t_load_and_mask_s,
                "inpaint_and_pcd_s": t_inpaint_and_pcd_s,
                "poisson_mesh_s": t_poisson_mesh_s,
                "export_files_s": t_export_s,
                "total_compute_s": t_total_compute_s,
            }
        }

reconstruction_service = InteractiveReconstructionService()
