import sys
import os
import logging
import json
import shutil
import time
from datetime import datetime
from typing import List, Tuple, Dict, Any
import yaml
import numpy as np
import cv2

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from apps.calib.services.calib_solver import (
    clean_calibration_data, evaluate_data_diversity, 
    optimize_extrinsics_solve, calculate_rotation_error
)
from scipy.spatial.transform import Rotation as R_tool

from services.camera_service import camera_service

logger = logging.getLogger(__name__)

class CalibrationService:
    def __init__(self):
        self.calib_dir = os.path.abspath(os.path.join(PROJECT_ROOT, "..", "data", "calib"))
        os.makedirs(self.calib_dir, exist_ok=True)
        
        # In-memory store for live progress tracking
        self.progress_states = {}

        
        # Load config to get default board and camera params
        self.config_path = os.path.join(PROJECT_ROOT, "..", "configs", "aisprayer_config.yaml")
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f) or {}
        except Exception as e:
            logger.warning(f"Failed to load global config: {e}")
            self.config = {}

    def get_sessions(self) -> List[str]:
        if not os.path.exists(self.calib_dir):
            return []
        sessions = [d for d in os.listdir(self.calib_dir) if os.path.isdir(os.path.join(self.calib_dir, d))]
        return sorted(sessions, reverse=True)

    def create_session(self) -> str:
        session_id = datetime.now().strftime("calib_%Y%m%d_%H%M%S")
        session_path = os.path.join(self.calib_dir, session_id)
        os.makedirs(session_path, exist_ok=True)

        # Get config values
        b_cfg = self.config.get("calib", {}).get("board", {})
        h_cfg = self.config.get("hardware", {})
        
        # Get real camera intrinsic via camera_service if connected
        K, D = camera_service.get_intrinsics()
        cam_width = 1280
        cam_height = 800
        intrinsic_matrix = [[611.68, 0.0, 643.42], [0.0, 611.69, 405.15], [0.0, 0.0, 1.0]]
        distortion_coeffs = [-0.032, 0.034, 0.0003, 0.0003, -0.011]
        
        if K is not None and D is not None:
            intrinsic_matrix = K.tolist()
            distortion_coeffs = D.flatten().tolist()
            if camera_service._cam:
                cam_width = getattr(camera_service._cam, "width", 1280)
                cam_height = getattr(camera_service._cam, "height", 800)
        else:
            logger.warning("Camera not connected. Using default intrinsic values for calibration session.")
            
        board_rows = b_cfg.get("rows", 12)
        board_cols = b_cfg.get("cols", 9)
        square_size = b_cfg.get("square_size_mm", 15.0)
        pattern_size = [board_cols - 1, board_rows - 1]

        cap_info = {
            "version": "1.0",
            "calibration_mode": "eye-to-hand",
            "camera_params": {
                "camera_model": h_cfg.get("camera", {}).get("model", "orbbec"),
                "width": cam_width,
                "height": cam_height,
                "intrinsic_matrix": intrinsic_matrix,
                "distortion_coeffs": distortion_coeffs
            },
            "board_params": {
                "rows": board_rows,
                "cols": board_cols,
                "square_size_mm": square_size,
                "pattern_size_inner": pattern_size
            },
            "samples": []
        }

        yaml_file = os.path.join(session_path, "calibration_info.yaml")
        with open(yaml_file, 'w', encoding='utf-8') as f:
            yaml.dump(cap_info, f, default_flow_style=False)

        return session_id

    def delete_session(self, session_id: str) -> bool:
        session_path = os.path.join(self.calib_dir, session_id)
        if os.path.exists(session_path):
            shutil.rmtree(session_path)
            return True
        return False

    def get_session_data(self, session_id: str) -> Dict[str, Any]:
        session_path = os.path.join(self.calib_dir, session_id)
        yaml_file = os.path.join(session_path, "calibration_info.yaml")
        
        data = {"samples": [], "result": None, "mode": "eye-to-hand"}

        if os.path.exists(yaml_file):
            with open(yaml_file, "r", encoding='utf-8') as f:
                yaml_data = yaml.safe_load(f) or {}
                data["mode"] = yaml_data.get("calibration_mode", "eye-to-hand")
                
                old_samples = yaml_data.get("samples", [])
                formatted_samples = []
                for s in old_samples:
                    pose = s.get("robot_pose", {})
                    pose_list = [pose.get("x", 0), pose.get("y", 0), pose.get("z", 0),
                                 pose.get("a", 0), pose.get("b", 0), pose.get("c", 0)]
                    formatted_samples.append({
                        "id": s.get("id", 0),
                        "filename": s.get("image_file", ""),
                        "pose": pose_list
                    })
                data["samples"] = formatted_samples

        if os.path.exists(os.path.join(session_path, "calibration_result.yaml")):
            with open(os.path.join(session_path, "calibration_result.yaml"), "r", encoding='utf-8') as f:
                yaml_res = yaml.safe_load(f) or {}
                data["result"] = yaml_res

        return data

    def add_sample(self, session_id: str, image: np.ndarray, robot_pose: List[float]) -> int:
        session_path = os.path.join(self.calib_dir, session_id)
        if not os.path.exists(session_path):
            raise Exception(f"Session {session_id} does not exist.")

        yaml_file = os.path.join(session_path, "calibration_info.yaml")

        if not os.path.exists(yaml_file):
            raise Exception(f"Session {session_id} is missing calibration_info.yaml")

        with open(yaml_file, "r", encoding='utf-8') as f:
            cap_info = yaml.safe_load(f)

        samples = cap_info.get("samples", [])
        # We 1-index for UI display consistency in older app, or just len + 1
        count = len(samples) + 1
        img_name = f"image_{count:03d}.png"
        img_path = os.path.join(session_path, img_name)

        # Save image
        cv2.imwrite(img_path, image)

        # Update metadata
        sample = {
            "id": count,
            "image_file": img_name,
            "robot_pose": {
                "x": round(robot_pose[0], 3), "y": round(robot_pose[1], 3), "z": round(robot_pose[2], 3),
                "a": round(robot_pose[3], 5), "b": round(robot_pose[4], 5), "c": round(robot_pose[5], 5)
            }
        }
        cap_info["samples"].append(sample)

        with open(yaml_file, "w", encoding='utf-8') as f:
            yaml.dump(cap_info, f, default_flow_style=False)

        return len(cap_info["samples"])

    def run_calibration(self, session_id: str, progress_callback=None) -> Dict[str, Any]:
        session_path = os.path.join(self.calib_dir, session_id)
        yaml_file = os.path.join(session_path, "calibration_info.yaml")
        
        def safe_callback(idx, total, filename, status):
            self.progress_states[session_id] = {
                "current": idx,
                "total": total,
                "filename": filename,
                "status": status
            }
            if progress_callback:
                try:
                    progress_callback(idx, total, filename, status)
                except Exception as e:
                    logger.error(f"Callback error: {e}")

        if not os.path.exists(yaml_file):
            safe_callback(0, 1, "", "error")
            return {"success": False, "error": "Missing calibration_info.yaml"}

        with open(yaml_file, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f)

        total_samples = len(info.get("samples", []))
        if total_samples < 3:
            safe_callback(0, total_samples, "", "error")
            return {"success": False, "error": "Not enough samples. Need at least 3."}

        logger.info(f"Running calibration for session {session_id} with {total_samples} samples.")
        safe_callback(0, total_samples, "", "started")

        K = np.array(info["camera_params"]["intrinsic_matrix"])
        D = np.array(info["camera_params"]["distortion_coeffs"])
        pattern_size = tuple(info["board_params"]["pattern_size_inner"])
        sq_size = info["board_params"]["square_size_mm"]

        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * sq_size

        all_samples = []

        for idx, s in enumerate(info["samples"]):
            # Update progress
            safe_callback(idx + 1, total_samples, s["image_file"], "processing")
            
            pose = s["robot_pose"]
            if any(abs(v) > 2500 for v in [pose['x'], pose['y'], pose['z']]):
                logger.warning(f"Skipped sample {s['id']} (invalid position)")
                continue

            img_path = os.path.join(session_path, s["image_file"])
            img = cv2.imread(img_path)
            if img is None:
                logger.warning(f"Skipped sample {s['id']} (image load failed)")
                continue

            ret, corners = cv2.findChessboardCorners(img, pattern_size, None)
            if ret:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
                _, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
                R_cb, _ = cv2.Rodrigues(rvec)
                all_samples.append({
                    "id": s["id"], 
                    "R_cb": R_cb, 
                    "t_cb": tvec.flatten(), 
                    "pose": pose
                })
            else:
                logger.warning(f"Failed to extract corners from sample {s['id']}")

        safe_callback(total_samples, total_samples, "", "optimizing")
        
        if len(all_samples) < 3:
            safe_callback(total_samples, total_samples, "", "error")
            return {"success": False, "error": "Insufficient valid chessboard corners found."}

        clean_thr = self.config.get("calib", {}).get("cleaning_threshold", 0.05)
        samples = clean_calibration_data(
            all_samples, 
            threshold=clean_thr,
            log_callback=lambda msg: logger.info(msg)
        )

        if len(samples) < 3:
            safe_callback(total_samples, total_samples, "", "error")
            return {"success": False, "error": "Calibration failed: not enough clean samples after filtering."}

        diversity = evaluate_data_diversity(samples)
        logger.info(f"Diversity score: {diversity['score']:.1f} / 100")

        best_res = optimize_extrinsics_solve(samples)
        if not best_res:
            safe_callback(total_samples, total_samples, "", "error")
            return {"success": False, "error": "Optimization solver failed."}

        R_bc, t_bc, t_off, err, order, s_vec = best_res
        r_err_mean = calculate_rotation_error(samples, best_res)

        T_bc = np.eye(4)
        T_bc[:3, :3] = R_bc
        T_bc[:3, 3] = t_bc
        xyz = t_bc.tolist()
        rpy = R_tool.from_matrix(R_bc).as_euler('xyz', degrees=True).tolist()

        output_res = {
            "metadata": {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "source_data_dir": session_path,
                "calibration_mode": info.get("calibration_mode", "eye-to-hand"),
                "reprojection_error_mm": float(err),
                "rotation_error_deg": float(r_err_mean),
                "samples_total": len(all_samples),
                "samples_used": len(samples),
                "optimization_config": {
                    "axis_order": order,
                    "sign_vector": [int(x) for x in s_vec]
                }
            },
            "camera_pose_base": {
                "x": xyz[0], "y": xyz[1], "z": xyz[2],
                "roll_deg": rpy[0], "pitch_deg": rpy[1], "yaw_deg": rpy[2]
            },
            "T_base_camera": T_bc.tolist(),
            "camera_params": info["camera_params"],
            "board_params": info["board_params"],
            "chessboard_offset": t_off.tolist()
        }
        
        result_yaml = os.path.join(session_path, "calibration_result.yaml")
        with open(result_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(output_res, f, default_flow_style=False)

        safe_callback(total_samples, total_samples, "", "completed")

        return {
            "success": True,
            **output_res
        }
        
    def stream_progress(self, session_id: str):
        """Generator for Server-Sent Events (SSE) that yields progress updates."""
        while True:
            state = self.progress_states.get(session_id)
            if not state:
                yield f"data: {json.dumps({'status': 'waiting'})}\n\n"
            else:
                yield f"data: {json.dumps(state)}\n\n"
                
                if state.get("status") in ["completed", "error"]:
                    # Clean up and exit
                    if session_id in self.progress_states:
                        del self.progress_states[session_id]
                    break
            time.sleep(0.1)

    def get_image_with_corners(self, session_id: str, filename: str) -> bytes:
        """Loads an image, attempts to find and draw chessboard corners, and returns JPEG bytes."""
        session_path = os.path.join(self.calib_dir, session_id)
        img_path = os.path.join(session_path, filename)
        yaml_file = os.path.join(session_path, "calibration_info.yaml")

        if not os.path.exists(img_path):
            raise FileNotFoundError(f"Image not found: {img_path}")

        img = cv2.imread(img_path)
        if img is None:
            raise ValueError(f"Failed to decode image: {img_path}")

        # Default fallback pattern size
        pattern_size = (8, 11)
        
        # Try to read actual pattern size from yaml
        if os.path.exists(yaml_file):
            try:
                with open(yaml_file, 'r', encoding='utf-8') as f:
                    info = yaml.safe_load(f)
                    if info and "board_params" in info:
                        bp = info["board_params"]
                        if "pattern_size_inner" in bp:
                            pattern_size = tuple(bp["pattern_size_inner"])
                        elif "cols" in bp and "rows" in bp:
                            pattern_size = (bp["cols"] - 1, bp["rows"] - 1)
            except Exception as e:
                logger.warning(f"Failed to read pattern size for corners drawing: {e}")

        # Find and draw corners
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        
        if ret:
            # We don't necessarily need subpixel refinement just for visualization, 
            # but it looks cleaner if we do it. However, skipping it saves time.
            cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            cv2.putText(img, f"CORNERS FOUND: {pattern_size[0]}x{pattern_size[1]}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(img, "CORNERS NOT FOUND", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)

        success, buffer = cv2.imencode('.jpg', img)
        if not success:
            raise ValueError("Failed to encode image to JPEG")
            
        return buffer.tobytes()

calibration_service = CalibrationService()
