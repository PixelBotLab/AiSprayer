import cv2
import threading
import logging
import time
import os
import sys
from typing import Optional, Generator, Callable, List
import numpy as np

from core.hardware.camera.factory import get_camera

logger = logging.getLogger(__name__)

class CameraService:
    def __init__(self):
        self._cam = None
        self._latest_color_frame: Optional[np.ndarray] = None
        self._latest_depth_frame: Optional[np.ndarray] = None
        self._is_streaming = False
        self._frame_thread = None
        self._corner_thread = None
        self._draw_calibration_corners = False
        self._cached_corners = None
        self._cached_corners_time: float = 0.0
        self._last_frame_time: float = 0.0
        self._camera_type: str = "orbbec"
        self._last_online_state: bool = False
        self._status_callbacks: List[Callable] = []

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
            
        try:
            logger.info(f"Initializing {camera_type} camera...")
            self._cam = get_camera(camera_type)
            self._cam.start()
        except Exception as e:
            logger.error(f"Failed to start camera {camera_type}: {e}")
            self._cam = None
            self._last_online_state = False
            self._notify_status()
            return False
            
        logger.info("Camera started successfully.")
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
        if self._frame_thread:
            self._frame_thread.join(timeout=1.0)
            self._frame_thread = None
        if self._corner_thread:
            self._corner_thread.join(timeout=1.0)
            self._corner_thread = None
        if self._cam:
            self._cam.stop()
            self._cam = None
        self._latest_color_frame = None
        self._latest_depth_frame = None
        self._cached_corners = None
        self._last_frame_time = 0.0
        self._last_online_state = False
        self._notify_status()

    def _update_frame(self):
        while self._is_streaming and self._cam:
            try:
                color, depth = self._cam.get_frame()
                if color is not None:
                    self._latest_color_frame = color
                    self._latest_depth_frame = depth
                    self._last_frame_time = time.time()
                    if not self._last_online_state:
                        self._last_online_state = True
                        self._notify_status()
                else:
                    if self._last_online_state and (time.time() - self._last_frame_time >= 3.0):
                        self._last_online_state = False
                        self._notify_status()
                    time.sleep(0.005)
            except Exception as e:
                logger.warning(f"Failed to grab frame: {e}")
                if self._last_online_state:
                    self._last_online_state = False
                    self._notify_status()
                time.sleep(0.01)

    def _corner_detector_loop(self):
        """Asynchronously and efficiently detects chessboard corners without blocking the video stream."""
        import cv2
        from services.setting_service import SettingService
        
        # Limit OpenCV to 1 thread in background to avoid starving the CPU
        cv2.setNumThreads(1)
        
        while self._is_streaming:
            if not self._draw_calibration_corners:
                self._cached_corners = None
                time.sleep(0.15)
                continue
                
            frame = self.get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue
                
            try:
                settings = SettingService()
                cols = int(settings.get_value("calib_board_cols", 9))
                rows = int(settings.get_value("calib_board_rows", 12))
                pattern_size = (cols - 1, rows - 1)

                h, w = frame.shape[:2]
                # Downscale to ~480px width for fast and lightweight corner search on ARM64
                target_w = 480
                scale = max(1.0, float(w) / float(target_w))
                small_w = int(w / scale)
                small_h = int(h / scale)
                
                small = cv2.resize(frame, (small_w, small_h), interpolation=cv2.INTER_LINEAR)
                gray_small = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
                
                flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
                ret, corners_small = cv2.findChessboardCorners(gray_small, pattern_size, flags=flags)
                
                if ret and corners_small is not None:
                    # Scale corners back to original full resolution
                    corners_full = corners_small * float(scale)
                    self._cached_corners = corners_full
                    self._cached_corners_time = time.time()
                else:
                    if time.time() - self._cached_corners_time > 0.5:
                        self._cached_corners = None
            except Exception as e:
                logger.debug(f"Corner detection error: {e}")
                self._cached_corners = None
                
            time.sleep(0.08) # ~12 Hz max detection rate for smooth corner tracking

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
        if not enabled:
            self._cached_corners = None

    def generate_mjpeg_stream(self) -> Generator[bytes, None, None]:
        from services.setting_service import SettingService
        settings = SettingService()
        cols = int(settings.get_value("calib_board_cols", 9))
        rows = int(settings.get_value("calib_board_rows", 12))
        pattern_size = (cols - 1, rows - 1)

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

        while self._is_streaming:
            frame = self.get_latest_frame()
            if frame is not None:
                # If calibration mode is active and corners are fresh, overlay them
                if self._draw_calibration_corners and self._cached_corners is not None:
                    if time.time() - self._cached_corners_time < 0.6:
                        display_frame = frame.copy()
                        cv2.drawChessboardCorners(display_frame, pattern_size, self._cached_corners, True)
                    else:
                        display_frame = frame
                else:
                    display_frame = frame

                ret_enc, buffer = cv2.imencode('.jpg', display_frame, encode_param)
                if ret_enc:
                    frame_bytes = buffer.tobytes()
                    try:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    except GeneratorExit:
                        break
                    except Exception:
                        break
            
            time.sleep(0.033) # 30 FPS stream

camera_service = CameraService()
