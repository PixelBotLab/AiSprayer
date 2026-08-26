import sys
import os
import logging
import json
import shutil
import time
from datetime import datetime
from typing import List, Tuple, Dict, Any, Optional
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

from apps.camera.services.camera_service import camera_service

logger = logging.getLogger(__name__)

def evaluate_image_quality(img: np.ndarray, pattern_size: Tuple[int, int]) -> Tuple[bool, Optional[np.ndarray], Dict[str, Any]]:
    """
    Evaluate image quality and detect chessboard corners.
    Returns: (corners_found, corners, quality_metrics)
    Quality metrics include:
      - sharpness (Laplacian variance for focus/blur detection)
      - brightness (Mean grayscale intensity 0-255)
      - contrast (Grayscale standard deviation)
      - overexposed_pct (Percentage of saturated/highlight pixels >= 250)
      - underexposed_pct (Percentage of dark pixels <= 5)
      - quality_rating ("EXCELLENT" | "GOOD" | "FAIR" | "POOR" | "FAIL (No Corners)")
    """
    if img is None or img.size == 0:
        return False, None, {"quality_rating": "FAIL (Empty Image)", "corners_count": 0, "sharpness": 0.0, "brightness": 0.0, "contrast": 0.0, "overexposed_pct": 0.0, "underexposed_pct": 0.0}

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    
    # 1. Chessboard corner detection
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    if ret:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    
    # 2. Image quality metrics
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    brightness = float(np.mean(gray))
    contrast = float(np.std(gray))
    overexposed_pct = float(np.sum(gray >= 250) / gray.size * 100.0)
    underexposed_pct = float(np.sum(gray <= 5) / gray.size * 100.0)
    
    num_corners = len(corners) if ret and corners is not None else 0
    
    if not ret:
        rating = "FAIL (No Corners)"
    elif sharpness < 40.0 or brightness < 35.0 or brightness > 225.0:
        rating = "POOR"
    elif sharpness < 100.0 or brightness < 60.0 or brightness > 195.0 or contrast < 30.0:
        rating = "FAIR"
    elif sharpness > 200.0 and 70.0 <= brightness <= 165.0 and contrast >= 40.0:
        rating = "EXCELLENT"
    else:
        rating = "GOOD"
        
    metrics = {
        "corners_found": bool(ret),
        "corners_count": int(num_corners),
        "sharpness": round(sharpness, 1),
        "brightness": round(brightness, 1),
        "contrast": round(contrast, 1),
        "overexposed_pct": round(overexposed_pct, 2),
        "underexposed_pct": round(underexposed_pct, 2),
        "quality_rating": rating
    }
    return ret, corners, metrics

class CalibrationService:
    def __init__(self):
        self.calib_dir = os.path.abspath(os.path.join(PROJECT_ROOT, "..", "data", "calib"))
        os.makedirs(self.calib_dir, exist_ok=True)
        
        # In-memory store for live progress tracking and corner image caching
        self.progress_states = {}
        self._corner_images_cache = {}

        
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
        
        # Get real camera intrinsic & resolution via camera_service if connected
        intr_dict = camera_service.get_intrinsics_dict()
        cam_width = h_cfg.get("camera", {}).get("resolution", {}).get("width", 1280)
        cam_height = h_cfg.get("camera", {}).get("resolution", {}).get("height", 800)
        intrinsic_matrix = [[611.68, 0.0, float(cam_width) / 2.0], [0.0, 611.69, float(cam_height) / 2.0], [0.0, 0.0, 1.0]]
        distortion_coeffs = [-0.032, 0.034, 0.0003, 0.0003, -0.011]
        
        if intr_dict and intr_dict.get("intrinsic_matrix"):
            intrinsic_matrix = intr_dict["intrinsic_matrix"]
            distortion_coeffs = intr_dict.get("distortion_coeffs", [])
            cam_width = intr_dict.get("width", cam_width)
            cam_height = intr_dict.get("height", cam_height)
        else:
            logger.warning("Camera offline. Using config fallback values for calibration session.")
            
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
            self._corner_images_cache = {
                k: v for k, v in self._corner_images_cache.items() if not k.startswith(f"{session_id}/")
            }
            return True
        return False

    def get_session_data(self, session_id: str) -> Dict[str, Any]:
        session_path = os.path.join(self.calib_dir, session_id)
        yaml_file = os.path.join(session_path, "calibration_info.yaml")
        res_file = os.path.join(session_path, "calibration_result.yaml")

        if not os.path.exists(yaml_file):
            raise Exception(f"Session {session_id} does not exist.")

        with open(yaml_file, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f) or {}

        mode = info.get("calibration_mode", "eye-to-hand")
        raw_samples = info.get("samples", [])
        formatted_samples = []
        for s in raw_samples:
            pose = s.get("robot_pose", {})
            rx = pose.get("rx", pose.get("a", 0.0))
            ry = pose.get("ry", pose.get("b", 0.0))
            rz = pose.get("rz", pose.get("c", 0.0))
            pose_list = [
                pose.get("x", 0.0),
                pose.get("y", 0.0),
                pose.get("z", 0.0),
                rx,
                ry,
                rz
            ]
            formatted_samples.append({
                "id": s.get("id", len(formatted_samples) + 1),
                "filename": s.get("image_file", s.get("filename", "")),
                "image_file": s.get("image_file", s.get("filename", "")),
                "pose": pose_list,
                "robot_pose": pose
            })

        result_data = None
        if os.path.exists(res_file):
            with open(res_file, 'r', encoding='utf-8') as f:
                result_data = yaml.safe_load(f)

        return {
            "session_id": session_id,
            "mode": mode,
            "samples": formatted_samples,
            "result": result_data
        }

    def add_sample(self, session_id: str, robot_pose: List[float]) -> int:
        session_path = os.path.join(self.calib_dir, session_id)
        if not os.path.exists(session_path):
            raise Exception(f"Session {session_id} does not exist.")
            
        yaml_file = os.path.join(session_path, "calibration_info.yaml")
        if not os.path.exists(yaml_file):
            raise Exception(f"Session {session_id} is missing calibration_info.yaml")

        with open(yaml_file, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f)

        samples = info.get("samples", [])
        sample_id = len(samples) + 1
        img_filename = f"image_{sample_id:03d}.png"

        # 触发 C++ 底层异步无锁直接写盘保存样本图片 (Zero-Copy)
        save_res = camera_service.save_frame(
            save_dir=session_path,
            color_filename=img_filename,
            save_color=True,
            save_depth=False,
            save_info_yaml=False,
            color_format="png"
        )
        if not save_res:
            raise Exception("Failed to trigger camera hardware frame persistence.")

        # Pose format: [x, y, z, rx, ry, rz]
        sample_entry = {
            "id": sample_id,
            "image_file": img_filename,
            "robot_pose": {
                "x": float(robot_pose[0]),
                "y": float(robot_pose[1]),
                "z": float(robot_pose[2]),
                "rx": float(robot_pose[3]),
                "ry": float(robot_pose[4]),
                "rz": float(robot_pose[5])
            },
            "timestamp": datetime.now().isoformat()
        }

        samples.append(sample_entry)
        info["samples"] = samples

        with open(yaml_file, 'w', encoding='utf-8') as f:
            yaml.dump(info, f, default_flow_style=False)

        return len(samples)

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

        # Avoid thread over-subscription on RK3588
        cv2.setNumThreads(2)

        for idx, s in enumerate(info["samples"]):
            # Update progress
            safe_callback(idx + 1, total_samples, s["image_file"], "processing")
            
            pose = s["robot_pose"]
            if any(abs(v) > 2500 for v in [pose['x'], pose['y'], pose['z']]):
                logger.warning(f"Skipped sample {s['id']} (invalid position)")
                time.sleep(0.06)
                continue

            img_path = os.path.join(session_path, s["image_file"])
            img = cv2.imread(img_path)
            if img is None:
                logger.warning(f"Skipped sample {s['id']} (image load failed)")
                time.sleep(0.06)
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
                # Draw corners and cache the rendered image for instant frontend retrieval
                cv2.drawChessboardCorners(img, pattern_size, corners, ret)
                cv2.putText(img, f"SAMPLE {idx+1}/{total_samples}: FOUND", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                logger.warning(f"Failed to extract corners from sample {s['id']}")
                cv2.putText(img, f"SAMPLE {idx+1}/{total_samples}: NOT FOUND", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)

            success, buffer = cv2.imencode('.jpg', img)
            if success:
                self._corner_images_cache[f"{session_id}/{s['image_file']}"] = buffer.tobytes()

            # Yield CPU so SSE stream and frontend can smoothly render the current frame with detected corners
            time.sleep(0.2)

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

        # Print and log complete calibration results in English
        banner_lines = [
            "=" * 70,
            f"  HAND-EYE CALIBRATION RESULT: {session_id}",
            "=" * 70,
            f"  - Status: SUCCESS",
            f"  - Samples: {len(samples)} used / {len(all_samples)} total",
            f"  - Reprojection Error (Residual): {float(err):.4f} mm",
            f"  - Mean Rotation Error (Angular): {float(r_err_mean):.4f} deg",
            f"  - Optimization Config: Euler Order = {order}, Signs = {s_vec}",
            f"  - Chessboard Offset (mm): [{t_off[0]:.3f}, {t_off[1]:.3f}, {t_off[2]:.3f}]",
            "-" * 70,
            f"  - Camera Pose in Robot Base Frame:",
            f"      X: {xyz[0]:.3f} mm,  Y: {xyz[1]:.3f} mm,  Z: {xyz[2]:.3f} mm",
            f"      Roll: {rpy[0]:.3f} deg,  Pitch: {rpy[1]:.3f} deg,  Yaw: {rpy[2]:.3f} deg",
            "-" * 70,
            f"  - Transformation Matrix T_base_camera (4x4):",
        ]
        for row in T_bc:
            banner_lines.append(f"      [ {row[0]:9.6f}, {row[1]:9.6f}, {row[2]:9.6f}, {row[3]:10.4f} ]")
        banner_lines.append(f"  - Result Saved To: {result_yaml}")
        banner_lines.append("=" * 70)
        
        banner_text = "\n".join(banner_lines)
        print(banner_text, flush=True)
        logger.info("\n" + banner_text)

        safe_callback(total_samples, total_samples, "", "completed")

        return {
            "success": True,
            **output_res
        }

    def resample_and_calibrate(self, session_id: str, progress_callback=None) -> Dict[str, Any]:
        """
        Automatic resample and calibration workflow:
        1. Read waypoint poses from calibration_info.yaml in the specified session;
        2. Drive the robot sequentially to each waypoint pose;
        3. Pause for 2s after reaching each pose for mechanical stabilization;
        4. Capture a new frame and evaluate chessboard corners and image quality metrics;
        5. Pause for 1s, then proceed to the next waypoint pose;
        6. Solve calibration extrinsics automatically after all samples are collected.
        """
        from services.robot_service import robot_service

        session_path = os.path.join(self.calib_dir, session_id)
        yaml_file = os.path.join(session_path, "calibration_info.yaml")

        def safe_callback(idx, total, filename, status, message=""):
            self.progress_states[session_id] = {
                "current": idx,
                "total": total,
                "filename": filename,
                "status": status,
                "message": message
            }
            if progress_callback:
                try:
                    progress_callback(idx, total, filename, status, message)
                except Exception as e:
                    logger.error(f"Callback error: {e}")

        if not os.path.exists(yaml_file):
            safe_callback(0, 1, "", "error", "Missing calibration_info.yaml")
            return {"success": False, "error": "Missing calibration_info.yaml"}

        with open(yaml_file, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f) or {}

        samples = info.get("samples", [])
        total_samples = len(samples)
        if total_samples < 3:
            safe_callback(0, total_samples, "", "error", "Session has fewer than 3 poses to resample.")
            return {"success": False, "error": "Not enough sample poses. Need at least 3."}

        if not robot_service.is_connected:
            safe_callback(0, total_samples, "", "error", "Robot is not connected.")
            return {"success": False, "error": "Robot is not connected. Please connect robot first."}

        logger.info(f"Starting automatic resample & calibration for session '{session_id}' with {total_samples} waypoints.")
        safe_callback(0, total_samples, "", "resampling_started", f"Starting automatic resampling ({total_samples} waypoints)...")

        pattern_size = tuple(info.get("board_params", {}).get("pattern_size_inner", [8, 11]))
        speed_l, acc_l, speed_j, acc_j = robot_service.get_speed()

        for idx, s in enumerate(samples):
            sample_id = s.get("id", idx + 1)
            img_filename = s.get("image_file", f"image_{sample_id:03d}.png")
            pose_dict = s.get("robot_pose", {})

            x = float(pose_dict.get("x", 0.0))
            y = float(pose_dict.get("y", 0.0))
            z = float(pose_dict.get("z", 0.0))
            rx = float(pose_dict.get("rx", pose_dict.get("a", 0.0)))
            ry = float(pose_dict.get("ry", pose_dict.get("b", 0.0)))
            rz = float(pose_dict.get("rz", pose_dict.get("c", 0.0)))
            target_pose = [x, y, z, rx, ry, rz]

            # 1. Drive robot to target waypoint pose
            safe_callback(idx + 1, total_samples, img_filename, "moving", f"Moving to waypoint {idx+1}/{total_samples}...")
            logger.info(f"[*] [Resample #{sample_id} ({idx+1}/{total_samples})] Moving robot to: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, Rx={rx:.4f}, Ry={ry:.4f}, Rz={rz:.4f}")

            move_ok, move_err = robot_service.move_to_pose_j(target_pose, speed=speed_j, acc=acc_j)
            if not move_ok:
                err_msg = f"Robot motion to sample #{sample_id} failed: {move_err}"
                logger.error(err_msg)
                safe_callback(idx + 1, total_samples, img_filename, "error", err_msg)
                return {"success": False, "error": err_msg}

            # 2. Settle robot for 2.0s after reaching waypoint
            safe_callback(idx + 1, total_samples, img_filename, "settling", f"Waypoint {idx+1}/{total_samples} reached. Settling 2.0s...")
            time.sleep(2.0)

            # 3. Capture and persist new camera frame (overwriting sample image)
            safe_callback(idx + 1, total_samples, img_filename, "capturing", f"Capturing sample {idx+1}/{total_samples}...")
            save_res = camera_service.save_frame(
                save_dir=session_path,
                color_filename=img_filename,
                save_color=True,
                save_depth=False,
                save_info_yaml=False,
                color_format="png"
            )
            time.sleep(0.15)

            # 4. Verify chessboard corners and evaluate image quality metrics, log quality report
            img_path = os.path.join(session_path, img_filename)
            img = cv2.imread(img_path)
            quality_str = ""
            if img is not None:
                corners_found, corners, quality = evaluate_image_quality(img, pattern_size)
                quality_str = f"Corners: {'FOUND' if corners_found else 'NONE'} | Sharpness: {quality['sharpness']} | Rating: {quality['quality_rating']}"
                
                # Log detailed image quality report
                logger.info(
                    f"[+] [Sample #{sample_id} Quality Report] "
                    f"File: {img_filename} | "
                    f"Corners: {'FOUND (' + str(quality['corners_count']) + ')' if corners_found else 'NOT FOUND'} | "
                    f"Rating: {quality['quality_rating']} | "
                    f"Sharpness: {quality['sharpness']} | "
                    f"Brightness: {quality['brightness']}/255 | "
                    f"Contrast: {quality['contrast']} | "
                    f"Overexposed: {quality['overexposed_pct']}% | "
                    f"Underexposed: {quality['underexposed_pct']}%"
                )

                # Render corners and quality badge to in-memory preview cache
                preview_img = img.copy()
                if corners_found:
                    cv2.drawChessboardCorners(preview_img, pattern_size, corners, corners_found)
                    status_text = f"SAMPLE {idx+1}/{total_samples}: FOUND | Sharpness: {quality['sharpness']:.1f} ({quality['quality_rating']})"
                    cv2.putText(preview_img, status_text, (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
                else:
                    status_text = f"SAMPLE {idx+1}/{total_samples}: NO CORNERS | Sharpness: {quality['sharpness']:.1f} ({quality['quality_rating']})"
                    cv2.putText(preview_img, status_text, (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)

                success, buffer = cv2.imencode('.jpg', preview_img)
                if success:
                    self._corner_images_cache[f"{session_id}/{img_filename}"] = buffer.tobytes()

                s["timestamp"] = datetime.now().isoformat()
            else:
                logger.warning(f"Failed to load image for quality check: {img_path}")

            # 5. Pause for 1.0s, then proceed to the next waypoint pose
            safe_callback(idx + 1, total_samples, img_filename, "waiting", f"Sample #{sample_id} captured ({quality_str}). Pausing 1.0s...")
            time.sleep(1.0)

        # Sync back updated metadata to YAML
        with open(yaml_file, 'w', encoding='utf-8') as f:
            yaml.dump(info, f, default_flow_style=False)

        logger.info(f"[*] Resampling completed for all {total_samples} waypoints. Starting calibration solver...")
        safe_callback(total_samples, total_samples, "", "optimizing", "All samples recaptured. Computing calibration...")

        # 6. Automatically solve calibration extrinsics after completing all waypoints
        return self.run_calibration(session_id, progress_callback=progress_callback)
        
    def stream_progress(self, session_id: str):
        """Generator for Server-Sent Events (SSE) that yields progress updates."""
        last_sent = None
        while True:
            state = self.progress_states.get(session_id)
            if not state:
                yield f"data: {json.dumps({'status': 'waiting'})}\n\n"
            else:
                curr = (state.get("current"), state.get("status"), state.get("filename"), state.get("message"))
                if curr != last_sent:
                    last_sent = curr
                    yield f"data: {json.dumps(state)}\n\n"
                
                if state.get("status") in ["completed", "error"]:
                    # Clean up and exit
                    if session_id in self.progress_states:
                        del self.progress_states[session_id]
                    break
            time.sleep(0.03)

    def get_image_with_corners(self, session_id: str, filename: str) -> bytes:
        """Loads an image, attempts to find and draw chessboard corners, and returns JPEG bytes."""
        cache_key = f"{session_id}/{filename}"
        if hasattr(self, "_corner_images_cache") and cache_key in self._corner_images_cache:
            return self._corner_images_cache[cache_key]

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
