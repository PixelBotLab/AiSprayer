import sys
import os
import logging
import json
import shutil
import time
from datetime import datetime
from typing import List, Sequence, Tuple, Dict, Any, Optional
import yaml
import numpy as np
import cv2

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from core.handeye import (
    DOBOT_EULER_SEQ, EYE_IN_HAND, EYE_TO_HAND, INTERNAL_ANGLE_UNIT, MOUNTS,
    UNIT_DEG, UNIT_RAD, CalibSample, chessboard_object_points, clean_samples,
    evaluate_data_quality, infer_angle_unit, invert_transform, make_transform,
    matrix_to_pose, minimum_samples, normalize_pose, pose_to_matrix,
    recommended_samples, solve_hand_eye,
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


def resolve_mount(info: Dict[str, Any]) -> str:
    """读取 session 的安装方式; 历史数据用 calibration_mode 键兜底, 未知值按眼在手外处理。"""
    mount = info.get("hand_eye_mount") or info.get("calibration_mode") or EYE_TO_HAND
    return mount if mount in MOUNTS else EYE_TO_HAND


def resolve_pose_unit(info: Dict[str, Any], samples: Sequence[dict]) -> str:
    """
    session  yaml 里姿态的单位。

    新 session 一律在写入时归一成度并显式记录 pose_angle_unit; 老数据没记, 只能
    按幅值猜 —— 这是历史包袱, 不是新代码的路径。
    """
    unit = info.get("pose_angle_unit")
    if unit in (UNIT_DEG, UNIT_RAD):
        return unit
    return infer_angle_unit(samples)


def _board_reach_mm(mount: str, samples: Sequence[CalibSample], solution: Any) -> float:
    """
    法兰原点到标定板原点的距离 (mm): 清洗区间里旋转项应当按这个尺度张开。

    眼在手外直接就是解出来的 TCP 偏移; 眼在手上传回的是相机外参, 需要再复合一次
    才得到板在法兰系下的位置。
    """
    if mount == EYE_TO_HAND:
        return float(np.linalg.norm(solution.board_offset_flange_mm))
    T_flange_cam = invert_transform(solution.T_flange_camera)
    dists = [float(np.linalg.norm((T_flange_cam @ s.T_camera_board)[:3, 3])) for s in samples]
    return float(np.median(dists)) if dists else 0.0


def _solution_score(solution: Any) -> float:
    """两个解择优用的标量: 优先像素重投影误差, 没有角点时退回平移残差。"""
    px = getattr(solution, "reprojection_error_px", None)
    return float(px) if px is not None else float(solution.translation_error_mm)


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

    def create_session(self, mount: Optional[str] = None) -> str:
        mount = mount or self.config.get("calib", {}).get("mount", EYE_TO_HAND)
        if mount not in MOUNTS:
            raise ValueError(f"unknown hand-eye mount '{mount}', expected one of {list(MOUNTS)}")

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
            "version": "2.0",
            "hand_eye_mount": mount,
            "pose_angle_unit": INTERNAL_ANGLE_UNIT,
            "min_samples": minimum_samples(mount),
            "recommended_samples": recommended_samples(mount),
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

        mount = resolve_mount(info)
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
                "robot_pose": pose,
                "joints": s.get("joints"),
            })

        result_data = None
        if os.path.exists(res_file):
            with open(res_file, 'r', encoding='utf-8') as f:
                result_data = yaml.safe_load(f)

        return {
            "session_id": session_id,
            "mount": mount,
            "pose_angle_unit": resolve_pose_unit(info, raw_samples),
            "min_samples": int(info.get("min_samples") or minimum_samples(mount)),
            "recommended_samples": int(info.get("recommended_samples") or recommended_samples(mount)),
            "samples": formatted_samples,
            "result": result_data
        }

    def _stamp_pose_unit(self, info: Dict[str, Any]) -> None:
        """
        把 session 内已有样本统一到度并写死 pose_angle_unit。

        老数据没记录单位 (robot_service 回报的是弧度), 若直接往里追加一条度样本,
        同一个文件就会混用两种单位, 而求解器只认一个声明值。
        """
        current = resolve_pose_unit(info, info.get("samples", []))
        if current != INTERNAL_ANGLE_UNIT:
            for s in info.get("samples", []):
                pose = s.get("robot_pose")
                if not pose:
                    continue
                for key in ("rx", "ry", "rz", "a", "b", "c"):
                    if key in pose and pose[key] is not None:
                        pose[key] = float(np.degrees(float(pose[key])))
        info["pose_angle_unit"] = INTERNAL_ANGLE_UNIT

    def add_sample(self, session_id: str, robot_pose: List[float],
                   angle_unit: str = UNIT_RAD,
                   joints_deg: Optional[Sequence[float]] = None) -> int:
        """
        采集一个样本: 保存当前帧并记录机器人法兰位姿。

        angle_unit 必须与 robot_pose 姿态部分的实际单位一致 —— robot_service.
        get_current_pose 返回的是弧度 (dobot_driver 里做过 math.radians), 因此默认
        UNIT_RAD。会话内一律存度, 避免求解器再猜单位。
        """
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

        # Pose format: [x, y, z, rx, ry, rz] -> 会话内统一为 mm / deg
        pose_deg = normalize_pose(robot_pose, angle_unit)
        self._stamp_pose_unit(info)

        sample_entry = {
            "id": sample_id,
            "image_file": img_filename,
            "robot_pose": {
                "x": float(pose_deg[0]),
                "y": float(pose_deg[1]),
                "z": float(pose_deg[2]),
                "rx": float(pose_deg[3]),
                "ry": float(pose_deg[4]),
                "rz": float(pose_deg[5])
            },
            "timestamp": datetime.now().isoformat()
        }
        if joints_deg is not None and len(joints_deg) >= 6:
            sample_entry["joints"] = [float(v) for v in list(joints_deg)[:6]]

        samples.append(sample_entry)
        info["samples"] = samples

        with open(yaml_file, 'w', encoding='utf-8') as f:
            yaml.dump(info, f, default_flow_style=False)

        return len(samples)

    def capture_sample(self, session_id: str) -> int:
        """从机器人读当前位姿/关节并采集样本, 单位换算只发生在这一处。"""
        from apps.robot.services.robot_service import robot_service

        pose, err = robot_service.get_current_pose()
        if pose is None:
            raise RuntimeError(err or "Robot pose unavailable")
        joints, _ = robot_service.get_current_joint()
        return self.add_sample(session_id, pose, angle_unit=UNIT_RAD, joints_deg=joints)

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
            err_msg = f"Missing calibration_info.yaml for session '{session_id}' at: {yaml_file}"
            logger.error(f"[-] [Calib ERROR] {err_msg}")
            safe_callback(0, 1, "", "error")
            return {"success": False, "error": err_msg}

        with open(yaml_file, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f)

        mount = resolve_mount(info)
        pose_unit = resolve_pose_unit(info, info.get("samples", []))
        min_samples = minimum_samples(mount)

        total_samples = len(info.get("samples", []))
        if total_samples < min_samples:
            err_msg = (f"Session '{session_id}' has {total_samples} samples, but "
                       f"'{mount}' needs at least {min_samples}.")
            logger.error(f"[-] [Calib ERROR] {err_msg}")
            safe_callback(0, total_samples, "", "error")
            return {"success": False, "error": err_msg}

        logger.info(f"Running {mount} calibration for session {session_id} "
                    f"with {total_samples} samples (poses in {pose_unit}).")
        safe_callback(0, total_samples, "", "started")

        K = np.array(info["camera_params"]["intrinsic_matrix"])
        D = np.array(info["camera_params"]["distortion_coeffs"])
        pattern_size = tuple(info["board_params"]["pattern_size_inner"])
        objp = chessboard_object_points(pattern_size, info["board_params"]["square_size_mm"])

        observations: List[CalibSample] = []

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
                observations.append(CalibSample(
                    sample_id=s["id"],
                    T_base_flange=pose_to_matrix(pose, pose_unit),
                    T_camera_board=make_transform(R_cb, tvec.flatten()),
                    pose_dobot=normalize_pose(pose, pose_unit),
                    corners_px=np.asarray(corners, dtype=np.float64).reshape(-1, 2).copy(),
                    image_file=s["image_file"],
                    joints_deg=s.get("joints"),
                    obj_pts=objp,
                ))
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
        
        if len(observations) < min_samples:
            safe_callback(total_samples, total_samples, "", "error")
            return {"success": False, "error": f"Insufficient valid chessboard corners found "
                                               f"(got {len(observations)}, need {min_samples})."}

        clean_thr = self.config.get("calib", {}).get("cleaning_threshold", 0.05)
        samples = clean_samples(observations, threshold=clean_thr,
                                log_callback=lambda msg: logger.info(msg))
        if len(samples) < min_samples:
            safe_callback(total_samples, total_samples, "", "error")
            return {"success": False, "error": "Calibration failed: not enough clean samples after filtering."}

        quality = evaluate_data_quality(samples, mount)
        logger.info(f"[{mount}] data quality: score={quality['score']:.1f}/100, "
                    f"translation_span={quality['translation_span_mm']:.1f}mm, "
                    f"rotation_span={quality['rotation_span_deg']:.1f}deg, "
                    f"axis_coverage={quality['axis_coverage']:.2f}")
        if quality["degenerate"]:
            logger.warning(
                f"[!] [{mount}] rotation axes are nearly parallel (axis_coverage="
                f"{quality['axis_coverage']:.2f} < 0.30). The hand-eye solution is poorly "
                f"observable; re-sample with the flange rotated about clearly different axes."
            )

        solution = solve_hand_eye(mount, samples, K=K, D=D)
        if solution is None:
            safe_callback(total_samples, total_samples, "", "error")
            return {"success": False, "error": "Optimization solver failed."}

        # 二次清洗: 首轮用宽松运动包络, 解出法兰到标定板的真实作用距离后收紧区间重解一次。
        reach_mm = _board_reach_mm(mount, samples, solution)
        if reach_mm > 0:
            tight = clean_samples(observations, threshold=clean_thr,
                                  motion_envelope_mm=max(reach_mm * 1.5, 60.0),
                                  log_callback=lambda msg: logger.info(f"[refine] {msg}"))
            if minimum_samples(mount) <= len(tight) < len(samples):
                refined = solve_hand_eye(mount, tight, K=K, D=D)
                if refined is not None and _solution_score(refined) <= _solution_score(solution):
                    samples, solution = tight, refined

        is_eto = mount == EYE_TO_HAND
        T_camera_mount = solution.T_base_camera if is_eto else solution.T_flange_camera
        mount_name = "base" if is_eto else "flange"
        pose_in_mount = matrix_to_pose(T_camera_mount)
        reproj_px = getattr(solution, "reprojection_error_px", None)

        output_res: Dict[str, Any] = {
            "metadata": {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "source_data_dir": session_path,
                "hand_eye_mount": mount,
                "pose_angle_unit": INTERNAL_ANGLE_UNIT,
                "reprojection_error_px": None if reproj_px is None else float(reproj_px),
                "translation_error_mm": float(solution.translation_error_mm),
                "rotation_error_deg": float(solution.rotation_error_deg),
                "samples_total": len(observations),
                "samples_used": len(samples),
                "data_quality": {
                    "score": round(quality["score"], 1),
                    "axis_coverage": round(quality["axis_coverage"], 3),
                    "rotation_span_deg": round(quality["rotation_span_deg"], 2),
                    "translation_span_mm": round(quality["translation_span_mm"], 1),
                    "degenerate": bool(quality["degenerate"]),
                },
                "solver": (
                    {"euler_order": solution.euler_order,
                     "sign_vector": [int(x) for x in solution.sign_vector]}
                    if is_eto else
                    {"ax_xb_method": solution.method,
                     "method_report": solution.method_report}
                ),
            },
            f"camera_pose_{mount_name}": {
                "x": pose_in_mount[0], "y": pose_in_mount[1], "z": pose_in_mount[2],
                "roll_deg": pose_in_mount[3], "pitch_deg": pose_in_mount[4],
                "yaw_deg": pose_in_mount[5],
            },
            f"T_{mount_name}_camera": T_camera_mount.tolist(),
            "camera_params": info["camera_params"],
            "board_params": info["board_params"],
        }

        if is_eto:
            board_rot = R_tool.from_matrix(solution.board_rotation_flange).as_euler(
                DOBOT_EULER_SEQ, degrees=True).tolist()
            output_res["chessboard_offset"] = solution.board_offset_flange_mm.tolist()
            output_res["chessboard_rotation_deg"] = board_rot
            # 历史键名, 值其实是运动学平移残差 (mm) 而非像素误差; reconstruction_service
            # 与前端仍在读它, 像素域误差另见 reprojection_error_px。
            output_res["metadata"]["reprojection_error_mm"] = float(solution.translation_error_mm)
        else:
            board_pose = matrix_to_pose(solution.T_base_board)
            output_res["T_base_board"] = solution.T_base_board.tolist()
            output_res["board_pose_base"] = {
                "x": board_pose[0], "y": board_pose[1], "z": board_pose[2],
                "roll_deg": board_pose[3], "pitch_deg": board_pose[4],
                "yaw_deg": board_pose[5],
            }
        
        result_yaml = os.path.join(session_path, "calibration_result.yaml")
        with open(result_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(output_res, f, default_flow_style=False)

        # Print and log complete calibration results in English
        reach_note = f" (board reach {reach_mm:.1f} mm)" if reach_mm > 0 else ""
        banner_lines = [
            "=" * 70,
            f"  HAND-EYE CALIBRATION RESULT: {session_id}",
            "=" * 70,
            f"  - Mount: {mount}",
            f"  - Status: SUCCESS",
            f"  - Samples: {len(samples)} used / {len(observations)} valid / {total_samples} captured",
            f"  - Reprojection Error: "
            + ("N/A (no corner pixels)" if reproj_px is None else f"{float(reproj_px):.4f} px"),
            f"  - Kinematic Residual: {float(solution.translation_error_mm):.4f} mm, "
            f"{float(solution.rotation_error_deg):.4f} deg{reach_note}",
            f"  - Data Quality: {quality['score']:.1f}/100 "
            f"(axis coverage {quality['axis_coverage']:.2f})"
            + ("  [DEGENERATE]" if quality["degenerate"] else ""),
            "-" * 70,
            f"  - Camera Pose in Robot {mount_name.capitalize()} Frame:",
            f"      X: {pose_in_mount[0]:.3f} mm,  Y: {pose_in_mount[1]:.3f} mm,  Z: {pose_in_mount[2]:.3f} mm",
            f"      Roll: {pose_in_mount[3]:.3f} deg,  Pitch: {pose_in_mount[4]:.3f} deg,  Yaw: {pose_in_mount[5]:.3f} deg",
            "-" * 70,
            f"  - Transformation Matrix T_{mount_name}_camera (4x4):",
        ]
        for row in T_camera_mount:
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
        from apps.robot.services.robot_service import robot_service

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
            err_msg = f"Missing calibration_info.yaml for session '{session_id}' at: {yaml_file}"
            logger.error(f"[-] [Resample ERROR] {err_msg}")
            safe_callback(0, 1, "", "error", err_msg)
            return {"success": False, "error": err_msg}

        with open(yaml_file, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f) or {}

        mount = resolve_mount(info)
        min_samples = minimum_samples(mount)
        samples = info.get("samples", [])
        total_samples = len(samples)
        if total_samples < min_samples:
            err_msg = (f"Session '{session_id}' has {total_samples} samples, but "
                       f"'{mount}' needs at least {min_samples}.")
            logger.error(f"[-] [Resample ERROR] {err_msg}")
            safe_callback(0, total_samples, "", "error", err_msg)
            return {"success": False, "error": err_msg}

        if not robot_service.is_connected():
            err_msg = "Robot is not connected. Please connect robot before starting automatic resampling."
            logger.error(f"[-] [Resample ERROR] {err_msg}")
            safe_callback(0, total_samples, "", "error", err_msg)
            return {"success": False, "error": err_msg}

        logger.info(f"Starting automatic resample & calibration for session '{session_id}' with {total_samples} waypoints.")
        safe_callback(0, total_samples, "", "resampling_started", f"Starting automatic resampling ({total_samples} waypoints)...")

        pattern_size = tuple(info.get("board_params", {}).get("pattern_size_inner", [8, 11]))
        speed_l, acc_l, speed_j, acc_j = robot_service.get_speed()

        pose_unit = resolve_pose_unit(info, samples)

        for idx, s in enumerate(samples):
            sample_id = s.get("id", idx + 1)
            img_filename = s.get("image_file", f"image_{sample_id:03d}.png")
            target_pose = normalize_pose(s.get("robot_pose", {}), pose_unit)
            joints = s.get("joints")

            # 1. Drive robot to target waypoint pose
            safe_callback(idx + 1, total_samples, img_filename, "moving", f"Moving to waypoint {idx+1}/{total_samples}...")
            logger.info(
                f"[*] [Resample #{sample_id} ({idx+1}/{total_samples})] Moving robot to: "
                f"X={target_pose[0]:.1f}, Y={target_pose[1]:.1f}, Z={target_pose[2]:.1f}, "
                f"Rx={target_pose[3]:.3f}, Ry={target_pose[4]:.3f}, Rz={target_pose[5]:.3f} (deg)"
            )

            if joints is not None and len(joints) >= 6:
                # 关节回放: 复现采集该图时的确切构型, 避免逆解在奇異点附近失败
                move_ok, move_err = robot_service.move_to_joint(joints, speed=speed_j, acc=acc_j)
            else:
                # robot_service.move_to_pose_j 接受弧度 (driver 内部再转回控制器度)
                move_pose = [target_pose[0], target_pose[1], target_pose[2]] + \
                    [float(v) for v in np.radians(target_pose[3:])]
                move_ok, move_err = robot_service.move_to_pose_j(move_pose, speed=speed_j, acc=acc_j)
            if not move_ok:
                err_msg = f"Robot motion to sample #{sample_id} failed: {move_err}"
                logger.error(err_msg)
                safe_callback(idx + 1, total_samples, img_filename, "error", err_msg)
                return {"success": False, "error": err_msg}

            # 2. Settle robot for 2.0s after reaching waypoint
            safe_callback(idx + 1, total_samples, img_filename, "settling", f"Waypoint {idx+1}/{total_samples} reached. Settling 2.0s...")
            time.sleep(2.0)

            # Query actual feedback pose & joints from robot encoders, store in the session's unit
            live_pose, _ = robot_service.get_current_pose()
            live_joints, _ = robot_service.get_current_joint()
            if live_pose and len(live_pose) >= 6:
                feedback = normalize_pose(live_pose, UNIT_RAD)
                s["robot_pose"] = {
                    "x": round(float(feedback[0]), 3),
                    "y": round(float(feedback[1]), 3),
                    "z": round(float(feedback[2]), 3),
                    "rx": round(float(feedback[3]), 5),
                    "ry": round(float(feedback[4]), 5),
                    "rz": round(float(feedback[5]), 5)
                }
            if live_joints and len(live_joints) >= 6:
                s["joints"] = [round(float(v), 4) for v in list(live_joints)[:6]]

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

                # Explicit error / warning logs on failure or degradation
                if not corners_found:
                    logger.error(
                        f"[-] [Sample #{sample_id} ERROR] Chessboard corners NOT detected in '{img_filename}'! "
                        f"(Sharpness: {quality['sharpness']}, Brightness: {quality['brightness']}). "
                        f"This sample will be excluded during calibration optimization."
                    )
                elif quality['quality_rating'] == 'POOR':
                    logger.warning(
                        f"[!] [Sample #{sample_id} WARNING] Poor image quality detected in '{img_filename}' "
                        f"(Sharpness: {quality['sharpness']}, Brightness: {quality['brightness']}, Overexposed: {quality['overexposed_pct']}%). "
                        f"Please inspect target illumination and check for glare or motion blur."
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
                logger.error(f"[-] [Sample #{sample_id} ERROR] Failed to load captured image file from disk: {img_path}")

            # 5. Pause for 1.0s, then proceed to the next waypoint pose
            safe_callback(idx + 1, total_samples, img_filename, "waiting", f"Sample #{sample_id} captured ({quality_str}). Pausing 1.0s...")
            time.sleep(1.0)

        # Sync back updated metadata and feedback poses to YAML
        info["pose_angle_unit"] = INTERNAL_ANGLE_UNIT
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
