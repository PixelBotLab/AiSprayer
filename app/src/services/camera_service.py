import cv2
import threading
import logging
import time
import os
import sys
from typing import Optional, Generator
import numpy as np

from aisprayer.core.hardware.camera.factory import get_camera

logger = logging.getLogger(__name__)

class CameraService:
    def __init__(self):
        self._cam = None
        self._latest_color_frame: Optional[np.ndarray] = None
        self._latest_depth_frame: Optional[np.ndarray] = None
        self._is_streaming = False
        self._thread = None
        self._draw_calibration_corners = False


    def start_stream(self, camera_type: str = "orbbec") -> bool:
        if self._is_streaming:
            self.stop_stream()
            
        try:
            logger.info(f"Initializing {camera_type} camera...")
            self._cam = get_camera(camera_type)
            self._cam.start()
        except Exception as e:
            logger.error(f"Failed to start camera {camera_type}: {e}")
            self._cam = None
            return False
            
        logger.info("Camera started successfully.")
        self._is_streaming = True
        self._thread = threading.Thread(target=self._update_frame, daemon=True)
        self._thread.start()
        return True

    def is_streaming(self) -> bool:
        return self._is_streaming

    def stop_stream(self):
        self._is_streaming = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cam:
            self._cam.stop()
            self._cam = None
        self._latest_color_frame = None
        self._latest_depth_frame = None

    def _update_frame(self):
        while self._is_streaming and self._cam:
            try:
                color, depth = self._cam.get_frame()
                if color is not None:
                    self._latest_color_frame = color
                    self._latest_depth_frame = depth
                else:
                    time.sleep(0.01)
            except Exception as e:
                logger.warning(f"Failed to grab frame: {e}")
                time.sleep(0.01)
                
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

    def generate_mjpeg_stream(self) -> Generator[bytes, None, None]:
        from services.setting_service import SettingService
        settings = SettingService()
        cols = int(settings.get_value("calib_board_cols", 9))
        rows = int(settings.get_value("calib_board_rows", 12))
        pattern_size = (cols - 1, rows - 1)

        frame_counter = 0
        last_corners = None
        last_detect_time = 0

        while self._is_streaming:
            frame = self.get_latest_frame()
            if frame is not None:
                display_frame = frame.copy()
                current_time = time.time()
                
                # Only run heavy detection if calibration mode is explicitly enabled
                if self._draw_calibration_corners:
                    # Run heavy detection only every 5 frames
                    if frame_counter % 5 == 0:
                        gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)
                        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
                        if ret:
                            last_corners = corners
                            last_detect_time = current_time
                        else:
                            last_corners = None
                    
                    # If we have recently detected corners (within the last 0.5s), draw them
                    if last_corners is not None and (current_time - last_detect_time) < 0.5:
                        cv2.drawChessboardCorners(display_frame, pattern_size, last_corners, True)
                else:
                    # Clear visual cache when disabled
                    last_corners = None

                ret_enc, buffer = cv2.imencode('.jpg', display_frame)
                if ret_enc:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            frame_counter += 1
            time.sleep(0.03) # ~30 fps cap

camera_service = CameraService()
