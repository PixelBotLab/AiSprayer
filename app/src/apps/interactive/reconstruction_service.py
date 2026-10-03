import os
import sys
import time
import queue
import logging
import threading
import traceback
import multiprocessing as mp
from logging.handlers import QueueHandler
from typing import Optional

import cv2
import numpy as np
import yaml
import trimesh

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from core.vision import (
    SurfaceReconstructor,
    k_matrix_to_intrinsics,
    depth_to_point_cloud,
)
from core.config import SprayerConfig
from core.handeye import (
    EYE_IN_HAND, pose_frame_mismatch, resolve_camera_extrinsic, resolve_result_mount,
)
from core.motion.kinematics import flange_pose_from_joints

logger = logging.getLogger(__name__)

# scan.params.yaml 里记录“这张图拍摄时相机靠什么定位”的块名。眼在手上时它是像素→基座
# 映射的唯一依据, 写与读都只走本模块 (capture_hand_eye_provenance / read_scan_hand_eye)。
HAND_EYE_BLOCK = "hand_eye"


class HandEyeCalibrationError(ValueError):
    """
    手眼外参当场解不出来 (生效结果是眼在手上但缺法兰位姿 / 采集时的装法或口径对不上)。

    故意做成 ValueError 的子类: 重建在子进程里跑, 异常名与文本是唯一能带回父进程的部分,
    _reraise_worker_error 按名重建后 api 仍会把英文原因当成 400 报给界面。
    """

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
    Pre-imports open3d, trimesh, and SurfaceReconstructor once during startup.
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
        from core.vision import SurfaceReconstructor  # noqa: F401

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
    if exc_type == HandEyeCalibrationError.__name__:
        # 自定义异常跳回父进程时只剩类型名字符串: 不显式按名重建就会落到 RuntimeError, 界面拿到 500,
        # 而这三条拒用理由都是现场能自己修的 (接臂 / 重新采集 / 改回 E2H), 必须是 400。
        raise HandEyeCalibrationError(err)
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


class InteractiveReconstructionService:
    def __init__(self):
        self.calib_dir = os.path.abspath(os.path.join(PROJECT_ROOT, "data", "calib"))

    def _published_session_name(self) -> str | None:
        """全局生效 (已发布) 的那一份结果来于哪个 session; 读不到返回 None。"""
        try:
            meta = (SprayerConfig().calib_data or {}).get("metadata", {}) or {}
        except Exception:
            return None
        src = meta.get("source_data_dir")
        return os.path.basename(str(src)) if src else None

    @staticmethod
    def _intrinsics_k(data: dict):
        """标定结果里的相机内参 K (3x3) 或 None; 两种装法的结果文件写法相同。"""
        cam = data.get("camera_params") or {}
        k_list = cam.get("intrinsic_matrix")
        return np.array(k_list, dtype=np.float64) if k_list else None

    @classmethod
    def _e2h_payload(cls, data: dict):
        """
        从一份 E2H 结果里取 (T_camera_to_base_4x4_米制, 内参 K_3x3 或 None, 误差_mm)。

        没有基座系常量时返回 None (调用方跳过这个 session)。mm -> m 的换算不在这里做,
        而是交给 core.handeye.resolve_camera_extrinsic —— 与生效结果、眼在手上复合走同一条
        解析链, 避免同一个换算两处各写一遍而漂移。
        """
        T, _mount, _note = resolve_camera_extrinsic(data)
        if T is None:
            return None
        err = (data.get("metadata", {}) or {}).get("reprojection_error_mm", 0.0)
        return np.array(T, dtype=np.float64), cls._intrinsics_k(data), float(err)

    @staticmethod
    def read_scan_hand_eye(template_path: Optional[str]) -> dict:
        """读 scan.params.yaml 里的 hand_eye 块; 模板、文件或块缺失都返回 {} (由调用方处置)。"""
        if not template_path:
            return {}
        path = os.path.join(template_path, "scan.params.yaml")
        try:
            with open(path, 'r', encoding='utf-8') as f:
                pdata = yaml.safe_load(f) or {}
        except Exception as e:
            logger.warning(f"Could not read hand_eye from {path}: {e}")
            return {}
        block = pdata.get(HAND_EYE_BLOCK) or {}
        return block if isinstance(block, dict) else {}

    def capture_hand_eye_provenance(self) -> dict:
        """
        采集一张 scan 时写下“这张图的相机在哪由什么确定”的溯源信息 (由调用方写进 scan.params.yaml)。

        眼在手上: 必须当场由关节反馈 FK 出法兰位姿 —— 那是这张图唯一的 3D 尺度来源, 事后补
        不出来 (臂一动相机就动), 拿不到就直接失败: 静默拍一张“以后解不出外参”的图比拒绝贵得多。
        眼在手外: 相机相对基座不动, 法兰位姿对成像没有意义, 但仍把装法与口径落盘, 供现场
        核对“这张图是在哪种装法、哪个口径下采的”。

        法兰位姿走的是与标定完全同源的那个 FK (FK(q+Δq), 校正口径), 否则采集与求解不同源,
        复合时会错一整个 Δq 在杆臂上的投影 (实测 14.44mm / 1.59°)。
        """
        cfg = SprayerConfig()
        mount = cfg.hand_eye_mount
        joints = None
        flange_pose = None
        if mount == EYE_IN_HAND:
            # 局部 import: 本模块还要在 spawn 子进程里跑, 不应把机械臂服务拖进重建 worker
            from apps.robot.services.robot_service import robot_service
            joints, reason = robot_service.get_current_joint()
            flange_pose = flange_pose_from_joints(joints)
            if flange_pose is None:
                raise HandEyeCalibrationError(
                    "Active calibration is eye-in-hand but the flange pose could not be "
                    f"recorded at capture ({reason or 'forward kinematics unavailable'}). "
                    "Connect the robot and capture again, or point spraying.calib_path at an "
                    "eye-to-hand result")
        rel_source = ""
        if cfg.calib_path:
            rel_source = os.path.relpath(cfg.calib_path, PROJECT_ROOT)
        return {
            "mount": mount,
            "pose_frame_convention": cfg.pose_frame_convention,
            "flange_pose_mm_deg": ([round(float(v), 4) for v in flange_pose]
                                   if flange_pose else None),
            "flange_joints_deg": ([round(float(v), 4) for v in list(joints)[:6]]
                                  if joints else None),
            "calibration_source": rel_source,
        }

    def _require_scan_flange_pose(self, template_path: Optional[str], cfg, mount: str) -> list:
        """
        眼在手上时从这张 scan 的 hand_eye 块里取采集时刻的法兰位姿; 任何对不上都当场拒绝。

        三道闸缺一不可:
        1. 没块 = 图是在支持眼在手之前 (或臂未连接) 采的 —— 没有“相机当时在哪”这个信息;
        2. 采集时记的装法与生效结果不同 = 相机已经重新装过但沿用了旧图;
        3. 采集时的口径与当前运行时口径不同 = 采完图后又改了 hardware.robot.joint_offsets_deg。
        这三种情况都能“算得出数”, 但算出来的是错的落点 —— 只失败, 不回退。
        """
        if not template_path:
            raise HandEyeCalibrationError(
                "Active calibration is eye-in-hand: the camera pose is not a constant, so the "
                "scan's capture-time flange pose is required. Pass the template directory.")
        block = self.read_scan_hand_eye(template_path)
        params_rel = os.path.relpath(os.path.join(template_path, "scan.params.yaml"),
                                     PROJECT_ROOT)
        cap_mount = block.get("mount")
        if not cap_mount:
            raise HandEyeCalibrationError(
                f"{params_rel} records no '{HAND_EYE_BLOCK}' block, so this scan has no flange "
                "pose to resolve the eye-in-hand camera against. Capture it again with the "
                "robot connected (the arm-mounted camera must not move during capture)")
        if cap_mount != mount:
            raise HandEyeCalibrationError(
                f"This scan was captured with the camera mounted '{cap_mount}' while the active "
                f"calibration is '{mount}'; re-capture after remounting and re-calibrating")
        cap_frame = block.get("pose_frame_convention")
        if cap_frame != cfg.pose_frame_convention:
            raise HandEyeCalibrationError(
                f"This scan's flange pose was recorded in pose frame '{cap_frame}' but the "
                f"runtime frame is '{cfg.pose_frame_convention}' "
                "(hardware.robot.joint_offsets_deg changed after capture); re-capture the scan")
        pose = block.get("flange_pose_mm_deg")
        if not pose or len(list(pose)) < 6:
            raise HandEyeCalibrationError(
                f"{params_rel} has an eye-in-hand '{HAND_EYE_BLOCK}' block but no "
                "flange_pose_mm_deg; capture the scan again with the robot connected")
        return [float(v) for v in list(pose)[:6]]

    def _resolve_active_calibration(self, cfg, template_path: Optional[str]):
        """
        把 spraying.calib_path 指向的**已发布**结果解析成本次要用的外参; 不可用时返回 None。

        返回 None 只发生在“这份结果对本链路根本没用”时 (无数据 / 眼在手外但带不出常量),
        那时才允许退回扫 data/calib 的历史兜底。眼在手上解不出法兰位姿是报错, 不是 None。
        """
        data = cfg.calib_data or {}
        if not data:
            return None
        mount = resolve_result_mount(data, default=cfg.calib_mount)
        flange_pose = (self._require_scan_flange_pose(template_path, cfg, mount)
                       if mount == EYE_IN_HAND else None)
        T, resolved_mount, note = resolve_camera_extrinsic(
            data, flange_pose,
            runtime_frame=cfg.pose_frame_convention,
            runtime_offsets_deg=cfg.robot_joint_offsets_deg)
        if T is None:
            if mount == EYE_IN_HAND:
                raise HandEyeCalibrationError(note)
            return None
        rel = (os.path.relpath(cfg.calib_path, PROJECT_ROOT) if cfg.calib_path
               else "global config")
        desc = f"{rel} ({resolved_mount}"
        if flange_pose is not None:
            desc += (f", flange pose from capture at "
                     f"[{', '.join(f'{v:.1f}' for v in flange_pose[:3])} mm]")
        if note:
            desc += f" [WARNING: {note}]"
        return np.array(T, dtype=np.float64), self._intrinsics_k(data), desc

    def get_latest_calibration(self, template_path: Optional[str] = None
                               ) -> tuple[np.ndarray, np.ndarray | None, str]:
        """
        取当前生效的手眼外参, 解析成 (T_base_camera 4x4 米制, 内参 K_3x3 或 None, 英文来源描述)。

        两种装法都支持, 装法不靠猜 —— 由结果文件自己的 metadata.hand_eye_mount 决定 (判定与
        SprayerConfig.hand_eye_mount 共用 resolve_result_mount), 所以切换只要在配置里改
        spraying.calib_path; 眼在手上时本方法按该 scan 采集时刻的法兰位姿复合外参。

        取哪一份: 先解 spraying.calib_path 指向的**已发布**结果 (机械臂运行时用的就是它,
        交互页必须与之一致, 否则“点的那一下”与“走的那条”吃两套外参); 只有发布结果对本链路
        完全不可用时, 才退回扫 data/calib/ 取最新的眼在手外 session (历史兜底)。

        口径 (controller_v1 / joint_offset_v2) 与装法正交, 同样得对齐: 兜底那条链是按文件
        自己挑结果的, 不经过 SprayerConfig 的加载守卫, 所以在此比一次 —— 优先取口径一致的
        最新一份; 一份都不一致时沿用最新的, 但把不符写进 desc 让界面看得见 (E2H 运行期不消费
        法兰位姿, 错的只是标定时被臂误差吸收掉的那一小部分, 所以只提示不阻断)。

        :param template_path: 本次要映射的那张 scan 所在模板目录 (眼在手上时必需)
        :raises HandEyeCalibrationError: 生效结果是眼在手上, 但这次解析不出相机位姿
                 (图没记录法兰位姿 / 采集装法或口径与生效结果不符) —— 绝不静默降级。
        """
        cfg = SprayerConfig()

        # 0. 已发布的生效结果 (两种装法通用); 它不可用时才继续往下扫 session
        active = self._resolve_active_calibration(cfg, template_path)
        if active is not None:
            logger.info(f"Loaded calibration from the active result: {active[2]}")
            return active

        runtime_frame = cfg.pose_frame_convention
        published_sess = self._published_session_name()
        skipped_eye_in_hand: list[str] = []
        stale_frame: list[str] = []
        chosen = None             # 口径一致的最新一份 (优先)
        fallback = None           # 口径不一致但可用的最新一份

        # 1. Search data/calib for latest calibration_result.yaml (仅当没有可用发布结果)
        if os.path.exists(self.calib_dir):
            sessions = sorted(
                [d for d in os.listdir(self.calib_dir) if os.path.isdir(os.path.join(self.calib_dir, d))],
                reverse=True
            )
            for sess in sessions:
                res_path = os.path.join(self.calib_dir, sess, "calibration_result.yaml")
                if not os.path.exists(res_path):
                    continue
                try:
                    with open(res_path, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f) or {}

                    # 显式识别装法: 不能只靠"没有 T_base_camera 这个键"隐式过滤,
                    # 否则哪天给 EIH 结果补写一个兼容键, 就会被当成基座系外参静默用错。
                    if resolve_result_mount(data) == EYE_IN_HAND:
                        skipped_eye_in_hand.append(sess)
                        logger.info(
                            f"Skipping eye-in-hand calibration session '{sess}' "
                            f"(fixed-scene mapping needs a constant T_base_camera)")
                        continue

                    payload = self._e2h_payload(data)
                    if payload is None:
                        continue

                    # 口径守卫: 本链路绕开了配置层的加载拦截, 必须自己比一次
                    reason = pose_frame_mismatch(data, runtime_frame,
                                                 cfg.robot_joint_offsets_deg)
                    if reason:
                        stale_frame.append(sess)
                        logger.warning(
                            f"Calibration session '{sess}' has a stale pose frame: {reason}")
                        if fallback is None:
                            fallback = (sess, *payload)
                        continue

                    chosen = (sess, *payload)
                    break
                except Exception as e:
                    logger.warning(f"Error reading calibration file {res_path}: {e}")

        cand = chosen if chosen is not None else fallback
        if cand is not None:
            sess, T, intr_k, err = cand
            desc = f"{sess} (Reprojection Error: {err:.3f} mm)"
            if chosen is None:
                # 没有一份对得上口径: 用最新的, 但必须让界面看得见这个降级
                desc += (f" [WARNING: no session matches the configured pose frame "
                         f"'{runtime_frame}'; re-run the calibration: "
                         f"{', '.join(stale_frame)}]")
            if stale_frame and chosen is not None:
                desc += (f" [skipped {len(stale_frame)} session(s) with a stale pose "
                         f"frame: {', '.join(stale_frame)}]")
            if skipped_eye_in_hand:
                desc += (f" [skipped {len(skipped_eye_in_hand)} eye-in-hand "
                         f"session(s): {', '.join(skipped_eye_in_hand)}]")
            # 本流水线按"最新 session"取, 与已发布的全局槽位不是同一份时
            # 必须能在界面上看出来, 否则现场会按错的标定调轨迹
            if published_sess and published_sess != sess:
                desc += f" [published global calibration: {published_sess}]"
            logger.info(f"Loaded calibration from session '{sess}' ({desc})")
            return T, intr_k, desc

        # 2. Fallback to Identity
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
        T_camera_to_base, calib_k, calib_desc = self.get_latest_calibration(template_path)
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

        # 6. Initialize Surface Reconstructor & Preprocess
        reconstructor = SurfaceReconstructor(
            z_min=100.0,
            z_max=3000.0,
            mask_erode_px=1,
            flying_pixel_max_grad=50.0,
            poisson_depth=8,
            density_threshold=0.15,
            voxel_size=0.003,
            normal_radius=0.03,
            smooth_iterations=20,
            n_threads=4,
        )

        intrinsics = k_matrix_to_intrinsics(intrinsics_k)
        t_prep_start = time.perf_counter()
        depth_proc, combined_mask = reconstructor.preprocess_depth(
            depth_image, unified_mask_2d, inpaint_holes=True
        )

        # 7. Convert Depth to 2.5D Camera Point Cloud
        raw_point_cloud = depth_to_point_cloud(depth_proc, intrinsics)
        t_pcd = time.perf_counter()
        t_inpaint_and_pcd_s = round(t_pcd - t_prep_start, 3)
        active_points = int(np.count_nonzero(combined_mask))
        logger.info(f"⏱️ [Reconstruct Step 2] Inpaint & PCD extraction in {t_inpaint_and_pcd_s:.2f}s ({active_points} valid points)")

        # 8. Perform 3D Poisson Reconstruction
        logger.info("Executing Poisson surface reconstruction and base coordinate alignment...")
        mesh: trimesh.Trimesh = reconstructor.reconstruct_mesh(
            raw_point_cloud, combined_mask, T_camera_to_base=T_camera_to_base
        )
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
