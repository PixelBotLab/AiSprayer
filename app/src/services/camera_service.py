import cv2
import threading
import logging
import time
from typing import Optional, Generator, Callable, List
import numpy as np

from core.hardware.camera.factory import get_camera

logger = logging.getLogger(__name__)

HW_FRAME_RATE = 15.0
CAMERA_TIMEOUT_MS = 1000
STATS_INTERVAL = 10.0
CORNER_DETECTION_HZ = HW_FRAME_RATE


class CameraService:
    def __init__(self):
        self._cam = None
        self._latest_color_frame: Optional[np.ndarray] = None
        self._latest_depth_frame: Optional[np.ndarray] = None
        self._is_streaming = False
        self._frame_thread = None
        self._corner_thread = None
        self._stop_event = threading.Event()
        
        self._draw_calibration_corners = False
        self._cached_corners = None
        self._cached_corners_time: float = 0.0
        self._last_frame_time: float = 0.0
        self._camera_type: str = "orbbec"
        self._last_online_state: bool = False
        
        self._fps_frame_count = 0
        self._fps_detect_count = 0
        self._fps_found_count = 0
        self._detect_times = []
        self._draw_times = []
        self._fps_last_log_time = time.time()
        self._status_callbacks: List[Callable] = []
        logger.info("CameraService initialized.")

    def register_status_callback(self, cb: Callable):
        if cb not in self._status_callbacks:
            self._status_callbacks.append(cb)

    def unregister_status_callback(self, cb: Callable):
        if cb in self._status_callbacks:
            self._status_callbacks.remove(cb)

    def _notify_status(self):
        status = self.get_status()
        for cb in list(self._status_callbacks):
            try:
                cb(status)
            except Exception:
                pass

    def start_stream(self, camera_type: str = "orbbec") -> bool:
        self._camera_type = camera_type
        if self._is_streaming:
            self.stop_stream()
        if ((self._frame_thread and self._frame_thread.is_alive()) or
                (self._corner_thread and self._corner_thread.is_alive())):
            logger.error("Cannot start camera while previous camera threads are still alive")
            return False
            
        try:
            logger.info(f"Initializing {camera_type} camera...")
            self._cam = get_camera(camera_type, fps=int(HW_FRAME_RATE), timeout_ms=CAMERA_TIMEOUT_MS)
            self._cam.start()
            logger.info("Camera started successfully.")
        except Exception as e:
            logger.warning(f"Initial camera start for {camera_type} failed ({e}), auto-reconnect will retry in background.")
            self._cam = None
            self._last_online_state = False
            
        self._stop_event.clear()
        self._is_streaming = True
        self._frame_thread = threading.Thread(target=self._update_frame, daemon=True)
        self._frame_thread.start()
        self._corner_thread = threading.Thread(target=self._corner_detector_loop, daemon=True)
        self._corner_thread.start()
        self._notify_status()
        return True

    def is_streaming(self) -> bool:
        return self._is_streaming

    def get_status(self) -> dict:
        now = time.time()
        is_online = (
            self._cam is not None 
            and self._is_streaming 
            and (self._latest_color_frame is not None)
            and getattr(self._cam, "_running", False)
            and (now - self._last_frame_time < 3.0 if self._last_frame_time > 0 else False)
        )
        return {
            "online": is_online,
            "streaming": self._is_streaming,
            "has_frame": self._latest_color_frame is not None,
            "last_frame_age_s": round(now - self._last_frame_time, 2) if self._last_frame_time > 0 else None,
            "camera_type": self._camera_type
        }

    def stop_stream(self):
        self._is_streaming = False
        self._stop_event.set()
        camera = self._cam
        if camera:
            try:
                camera.stop()
            except Exception:
                logger.exception("Failed to stop camera driver")
        if self._frame_thread and self._frame_thread is not threading.current_thread():
            self._frame_thread.join(timeout=5.0)
        if self._corner_thread and self._corner_thread is not threading.current_thread():
            self._corner_thread.join(timeout=5.0)
        frame_thread_alive = self._frame_thread is not None and self._frame_thread.is_alive()
        corner_thread_alive = self._corner_thread is not None and self._corner_thread.is_alive()
        if frame_thread_alive or corner_thread_alive:
            logger.error("Camera threads did not stop cleanly; refusing restart")
        else:
            self._frame_thread = None
            self._corner_thread = None
        self._cam = None
        self._latest_color_frame = None
        self._latest_depth_frame = None
        self._cached_corners = None
        self._last_frame_time = 0.0
        self._last_online_state = False
        self._notify_status()

    def _update_frame(self):
        """Dedicated high-speed reader loop with automatic reconnection."""
        while self._is_streaming and not self._stop_event.is_set():
            # If camera is missing or not running, attempt auto-reconnect
            if self._cam is None or not getattr(self._cam, "_running", True):
                if self._last_online_state:
                    self._last_online_state = False
                    self._notify_status()

                # Safely clean up dead camera instance
                if self._cam is not None:
                    try:
                        self._cam.stop()
                    except Exception:
                        pass
                    self._cam = None

                logger.info(f"Camera offline. Attempting to reconnect ({self._camera_type})...")
                if self._stop_event.wait(1.5):
                    break

                try:
                    new_cam = get_camera(self._camera_type, fps=int(HW_FRAME_RATE), timeout_ms=CAMERA_TIMEOUT_MS)
                    new_cam.start()
                    self._cam = new_cam
                    logger.info(f"Camera ({self._camera_type}) successfully reconnected!")
                except Exception as e:
                    logger.debug(f"Camera reconnect attempt failed: {e}")
                    continue

            camera = self._cam
            if camera is None:
                continue

            try:
                # In calibration mode, only 2D color images are needed for chessboard detection,
                # so disable depth alignment to save CPU and minimize latency.
                need_depth = not self._draw_calibration_corners
                color, depth = camera.get_frame(enable_depth=need_depth, timeout_ms=CAMERA_TIMEOUT_MS)
                if color is not None:
                    frame_time = time.time()
                    self._latest_color_frame = color
                    self._latest_depth_frame = depth
                    self._last_frame_time = frame_time
                    self._fps_frame_count += 1
                    
                    self._log_fps_stats(time.time())
                        
                    if not self._last_online_state:
                        self._last_online_state = True
                        self._notify_status()
                else:
                    if self._last_online_state and time.time() - self._last_frame_time >= 3.0:
                        self._last_online_state = False
                        self._notify_status()
                    if self._stop_event.wait(0.01):
                        break
            except Exception as e:
                logger.warning(f"Failed to grab frame: {e}")
                if self._last_online_state:
                    self._last_online_state = False
                    self._notify_status()
                if self._stop_event.wait(0.25):
                    break

    def _log_fps_stats(self, now: float):
        """Logs FPS and latency statistics periodically."""
        elapsed = now - self._fps_last_log_time
        if elapsed < STATS_INTERVAL:
            return

        detect_times = self._detect_times
        draw_times = self._draw_times
        self._detect_times = []
        self._draw_times = []

        if detect_times:
            avg_det = sum(detect_times) / len(detect_times)
            max_det = max(detect_times)
            min_det = min(detect_times)
            det_stats = f" | Detect(ms): avg={avg_det:.1f} min={min_det:.1f} max={max_det:.1f}"
        else:
            det_stats = ""

        if draw_times:
            avg_draw = sum(draw_times) / len(draw_times)
            max_draw = max(draw_times)
            min_draw = min(draw_times)
            draw_stats = f" | Draw(ms): avg={avg_draw:.1f} min={min_draw:.1f} max={max_draw:.1f}"
        else:
            draw_stats = ""

        logger.info(
            f"[FPS Monitor] Camera Read: {self._fps_frame_count/elapsed:.1f} fps | "
            f"Corner Detect: {self._fps_detect_count/elapsed:.1f} fps | "
            f"Found: {self._fps_found_count/elapsed:.1f} fps"
            f"{det_stats}{draw_stats}"
        )
        self._fps_frame_count = 0
        self._fps_detect_count = 0
        self._fps_found_count = 0
        self._fps_last_log_time = now

    def _corner_detector_loop(self):
        """Asynchronously detects chessboard corners in background without blocking the video stream."""
        cv2.setNumThreads(2)
        #cv2.ocl.setUseOpenCL(False)
        
        last_detection_interval = 1.0 / CORNER_DETECTION_HZ
        from core.config import config
        cols = config.calib_board_cols
        rows = config.calib_board_rows
        pattern_size = (cols - 1, rows - 1)
        
        last_detection_time = 0.0
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        
        while self._is_streaming and not self._stop_event.is_set():
            if not self._draw_calibration_corners:
                self._cached_corners = None
                time.sleep(0.1)
                continue

            now = time.time()
            if now - last_detection_time < last_detection_interval:
                time.sleep(0.005)
                continue

            frame = self.get_latest_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            last_detection_time = time.time()
            corners = None
            try:
                t_start_det = time.time()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                ret, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
                t_end_det = time.time()
                self._detect_times.append((t_end_det - t_start_det) * 1000.0)
                self._fps_detect_count += 1
                
                if ret and corners is not None:
                    self._fps_found_count += 1
                    self._cached_corners = corners
                    self._cached_corners_time = time.time()
                else:
                    if time.time() - self._cached_corners_time > 0.5:
                        self._cached_corners = None
            except Exception as e:
                logger.debug(f"Corner detection error: {e}")
                self._cached_corners = None

    def get_latest_frame(self) -> Optional[np.ndarray]:
        return self._latest_color_frame
        
    def get_latest_depth(self) -> Optional[np.ndarray]:
        return self._latest_depth_frame

    def get_intrinsics(self):
        if self._cam:
            return self._cam.get_intrinsics()
        return None, None

    def set_calibration_mode(self, enabled: bool):
        self._draw_calibration_corners = enabled
        logger.info(f"Set calibration mode: {enabled}")
        if not enabled:
            self._cached_corners = None

    def generate_mjpeg_stream(self) -> Generator[bytes, None, None]:
        """Streams real-time MJPEG with overlaid calibration corners using lightweight CPU encoding."""
        from core.config import config
        cols = config.calib_board_cols
        rows = config.calib_board_rows
        pattern_size = (cols - 1, rows - 1)
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        last_encoded_frame_time = 0
        last_yield_time = 0
        stream_interval = 1.0 / HW_FRAME_RATE

        while self._is_streaming:
            current_frame_time = self._last_frame_time
            if current_frame_time <= last_encoded_frame_time:
                time.sleep(0.005)
                continue
                
            frame = self.get_latest_frame()
            if frame is None:
                time.sleep(0.005)
                continue
                
            last_encoded_frame_time = current_frame_time

            # If calibration mode is on and corners are found, overlay corners (< 1ms)
            if self._draw_calibration_corners and self._cached_corners is not None and (time.time() - self._cached_corners_time < 0.6):
                display_frame = frame.copy()
                t_start_draw = time.time()
                cv2.drawChessboardCorners(display_frame, pattern_size, self._cached_corners, True)
                t_end_draw = time.time()
                self._draw_times.append((t_end_draw - t_start_draw) * 1000.0)
            else:
                display_frame = frame

            ret_enc, buffer = cv2.imencode('.jpg', display_frame, encode_param)
            if ret_enc:
                frame_bytes = buffer.tobytes()
                try:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    last_yield_time = time.time()
                except GeneratorExit:
                    break
                except Exception:
                    break

camera_service = CameraService()
