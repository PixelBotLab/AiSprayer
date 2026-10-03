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
    DOBOT_EULER_SEQ, EYE_TO_HAND, INTERNAL_ANGLE_UNIT, MOUNTS,
    UNIT_DEG, UNIT_RAD, CalibSample, assess_confidence, chessboard_object_points,
    declare_pose_frame, evaluate_data_quality, extrinsic_uncertainty,
    infer_angle_unit, invert_transform, make_transform, matrix_to_pose,
    minimum_samples, normalize_pose, pose_to_matrix, POSE_FRAME_CONTROLLER_V1,
    prune_outliers, readout_rotation_consistency, recommended_samples,
    resolve_result_mount, resolve_result_pose_frame, solve_hand_eye,
)
from scipy.spatial.transform import Rotation as R_tool

from apps.camera.services.camera_service import camera_service
from core.config import SprayerConfig
from core.motion.kinematics import flange_pose_from_joints

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


def _pose_dict(pose6: Sequence[float]) -> Dict[str, float]:
    """[x, y, z, rx, ry, rz] -> 会话 yaml 的位姿字典 (平移 mm / 姿态度)。"""
    return {
        "x": round(float(pose6[0]), 3),
        "y": round(float(pose6[1]), 3),
        "z": round(float(pose6[2]), 3),
        "rx": round(float(pose6[3]), 5),
        "ry": round(float(pose6[4]), 5),
        "rz": round(float(pose6[5]), 5),
    }


def _readout_config_conflict(samples: Sequence[dict], tool_index: Optional[int],
                             user_index: Optional[int]) -> Optional[str]:
    """
    会话内已记录的 tool/user 号与当前控制器读数不符时, 记录警告日志, 不阻断采样。

    由于系统采集时恒记录关节角 (joints), 标定求解始终优先由正运动学反推纯机械法兰位姿
    (与控制器当前的 tool_index 偏置解耦), 故 tool 号切换不影响法兰位姿计算, 不再阻断采样。
    """
    for s in samples:
        for key, current in (("tool_index", tool_index), ("user_index", user_index)):
            recorded = s.get(key)
            if recorded is None or current is None:
                continue
            if int(recorded) != int(current):
                logger.warning(
                    f"Robot {key} changed from {int(recorded)} to {int(current)} during session "
                    f"(at sample {s.get('id')}). Flange pose is derived from joint kinematics, "
                    f"proceeding without blocking."
                )
                return None
    return None



def _board_reach_mm(mount: str, samples: Sequence[CalibSample], solution: Any) -> float:
    """
    法兰原点到标定板原点的距离 (mm): 告诉操作员姿态误差会被多长的杆臂放大。

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
        # 仓根 (configs/ 与 data/ 都挂在这里), 展示相对路径时用作基准
        self.repo_root = os.path.abspath(os.path.join(PROJECT_ROOT, ".."))
        
        # In-memory store for live progress tracking and corner image caching
        self.progress_states = {}
        self._corner_images_cache = {}

    @property
    def sprayer(self) -> SprayerConfig:
        """
        运行时配置单例 (三级级联: SQLite 覆盖 > YAML > 代码默认)。

        每次现取 —— 本服务不再自己抄一份 YAML 快照, 否则在 Settings 里改了
        calib.mount / calib.board.* 也要重启后端才生效。
        """
        return SprayerConfig()

    def get_sessions(self) -> List[str]:
        if not os.path.exists(self.calib_dir):
            return []
        sessions = [d for d in os.listdir(self.calib_dir) if os.path.isdir(os.path.join(self.calib_dir, d))]
        return sorted(sessions, reverse=True)

    def _active_result_path(self) -> str:
        """
        全局生效的标定结果文件路径 (当前为 configs/calib/calibration_result.yaml)。

        直接取 SprayerConfig.calib_path —— 运行时读的就是那一个, 发布目标必须是同一个,
        否则会出现"发布成功但机械臂仍用旧外参"。配置未写时给一个明确基线路径。
        """
        path = self.sprayer.calib_path
        return path or os.path.join(self.repo_root, "configs", "calib", "calibration_result.yaml")

    def _result_summary(self, path: str) -> Dict[str, Any]:
        """
        一份标定结果文件的摘要 (装法 / 求解时间 / 误差 / 来源 session)。

        只给界面展示与"发布会顶掉谁"的确认信息, 不返回完整矩阵。
        """
        rel = os.path.relpath(path, self.repo_root)
        if not os.path.exists(path):
            return {"exists": False, "path": rel}
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            logger.warning(f"Failed to read calibration result {path}: {e}")
            return {"exists": False, "path": rel, "error": "Unreadable result file"}
        meta = data.get("metadata", {}) or {}
        source_dir = meta.get("source_data_dir")
        return {
            "exists": True,
            "path": rel,
            "mount": resolve_result_mount(data),
            # 法兰位姿口径 + 与当前运行时口径不符的原因 (无问题为 None): 前端据此展示待确认标记并拦发布
            "pose_frame": resolve_result_pose_frame(data)["frame"],
            "pose_frame_warning": self.sprayer.pose_frame_mismatch(data),
            # 运行时自己的口径: 发布前要把“待发布 session 的口径”与之比一次, 而 session 结果
            # 不走 calib_data 那条加载链路, 所以随摘要一并递出。
            "runtime_pose_frame": self.sprayer.pose_frame_convention,
            "timestamp": meta.get("timestamp"),
            "reprojection_error_px": meta.get("reprojection_error_px"),
            "reprojection_error_mm": meta.get("reprojection_error_mm"),
            "source_session": os.path.basename(str(source_dir)) if source_dir else None,
        }

    def get_active_result(self) -> Dict[str, Any]:
        """当前全局生效的标定结果概要 (文件不存在时 exists=False)。"""
        return self._result_summary(self._active_result_path())

    def publish_result(self, session_id: str) -> Dict[str, Any]:
        """
        把一个 session 的求解结果发布为全局生效标定。

        session 目录原样保留, 只覆盖那一份人工维护的全局槽位; 覆盖前把旧结果滚动备份到
        同目录 calibration_result.prev.yaml —— 现场只需要上一份可退, 留多个时间戳备份反而
        分不清哪个是当前回退点。发布后 reload 运行时配置, follow / 规划无需重启后端。
        """
        src = os.path.join(self.calib_dir, session_id, "calibration_result.yaml")
        if not os.path.exists(src):
            raise FileNotFoundError(
                f"Session '{session_id}' has no calibration result, run the solver first")

        dst = self._active_result_path()
        replaced = self._result_summary(dst)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if replaced.get("exists"):
            shutil.copy2(dst, os.path.join(os.path.dirname(dst), "calibration_result.prev.yaml"))
        shutil.copy2(src, dst)

        try:
            SprayerConfig().reload()
        except Exception as e:
            logger.warning(f"Calibration published but runtime config reload failed: {e}")
        logger.info(f"Published calibration session '{session_id}' to {dst} "
                    f"(replaced mount={replaced.get('mount')}, timestamp={replaced.get('timestamp')})")

        return {"published": self._result_summary(dst), "replaced": replaced}

    def create_session(self, mount: Optional[str] = None) -> str:
        mount = mount or self.sprayer.calib_mount
        if mount not in MOUNTS:
            raise ValueError(f"unknown hand-eye mount '{mount}', expected one of {list(MOUNTS)}")

        session_id = datetime.now().strftime("calib_%Y%m%d_%H%M%S")
        session_path = os.path.join(self.calib_dir, session_id)
        os.makedirs(session_path, exist_ok=True)

        # Get config values (h_cfg 仅用于相机离线时的分辨率兜底, 不是受管配置项)
        h_cfg = (self.sprayer.config_data or {}).get("hardware", {})
        
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
            
        board_rows = self.sprayer.calib_board_rows
        board_cols = self.sprayer.calib_board_cols
        square_size = self.sprayer.calib_board_square_size_mm
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

    def set_session_mount(self, session_id: str, mount: str) -> Dict[str, Any]:
        """
        重新绑定 session 的相机安装方式 (改写 calibration_info.yaml)。

        样本只是"法兰位姿 + 图像", 本身与装法无关, 换装法不需要重采; 但必须重解,
        因为已落盘的 calibration_result.yaml 是按旧装法求的。EIH/E2H 的样本数门槛
        也不同 (5 / 3), 所以创建时快照下来的 min/recommended 两个字段要一起重盖。
        """
        if mount not in MOUNTS:
            raise ValueError(f"unknown hand-eye mount '{mount}', expected one of {list(MOUNTS)}")

        yaml_file = os.path.join(self.calib_dir, session_id, "calibration_info.yaml")
        if not os.path.exists(yaml_file):
            raise FileNotFoundError(f"Session '{session_id}' does not exist")

        with open(yaml_file, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f) or {}

        previous = resolve_result_mount(info)
        if previous != mount:
            # 旧版只写 calibration_mode 的文件走这个分支时一并归一到新键名
            info.pop("calibration_mode", None)
            info["hand_eye_mount"] = mount
            info["min_samples"] = minimum_samples(mount)
            info["recommended_samples"] = recommended_samples(mount)
            with open(yaml_file, 'w', encoding='utf-8') as f:
                yaml.dump(info, f, default_flow_style=False)
            logger.info(f"Session '{session_id}' re-bound from {previous} to {mount}, "
                        f"{len(info.get('samples', []))} samples kept")

        return {"session_id": session_id, "mount": mount, "previous": previous,
                "changed": previous != mount}

    def get_session_data(self, session_id: str) -> Dict[str, Any]:
        session_path = os.path.join(self.calib_dir, session_id)
        yaml_file = os.path.join(session_path, "calibration_info.yaml")
        res_file = os.path.join(session_path, "calibration_result.yaml")

        if not os.path.exists(yaml_file):
            raise Exception(f"Session {session_id} does not exist.")

        with open(yaml_file, 'r', encoding='utf-8') as f:
            info = yaml.safe_load(f) or {}

        mount = resolve_result_mount(info)
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
                # 手眼求解真正用的法兰位姿 (FK of joints); 旧会话没记则 None
                "flange_pose": s.get("flange_pose"),
                "tool_index": s.get("tool_index"),
                "user_index": s.get("user_index"),
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
                   joints_deg: Optional[Sequence[float]] = None,
                   tool_index: Optional[int] = None,
                   user_index: Optional[int] = None) -> int:
        """
        采集一个样本: 保存当前帧并记录机器人法兰位姿与控制器坐标配置。

        angle_unit 必须与 robot_pose 姿态部分的实际单位一致 —— robot_service.
        get_current_pose 返回的是弧度 (dobot_driver 里做过 math.radians), 因此默认
        UNIT_RAD。会话内一律存度, 避免求解器再猜单位。

        flange_pose 由 FK(joints) 反推, 与示教器的 tool 号无关; robot_pose 仍是控制器
        原样读数 (当前 tool 的 TCP), 只作为没有关节反馈时的兜底。
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
        conflict = _readout_config_conflict(samples, tool_index, user_index)
        if conflict:
            raise RuntimeError(conflict)

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
            "robot_pose": _pose_dict(pose_deg),
            "timestamp": datetime.now().isoformat()
        }
        if joints_deg is not None and len(joints_deg) >= 6:
            sample_entry["joints"] = [float(v) for v in list(joints_deg)[:6]]
            flange = flange_pose_from_joints(joints_deg)
            if flange is not None:
                sample_entry["flange_pose"] = _pose_dict(flange)
        if tool_index is not None:
            sample_entry["tool_index"] = int(tool_index)
        if user_index is not None:
            sample_entry["user_index"] = int(user_index)

        samples.append(sample_entry)
        info["samples"] = samples

        with open(yaml_file, 'w', encoding='utf-8') as f:
            yaml.dump(info, f, default_flow_style=False)

        return len(samples)

    def capture_sample(self, session_id: str) -> int:
        """从机器人读当前位姿/关节并采集样本, 单位换算只发生在这一处。

        工业互锁: 样本的成立前提是“位姿读数”与“拍到的那一帧”是同一个时刻。
        机械臂还在走 (running_status=1) 时采到的图与位姿不同步, 会直接污染 AX=XB
        约束; 停下后法兰仍有结构振动, 因此再等一段 settle 延时, 并在等待后以控制
        器反馈复核一次 (避免等待期间又被点动)。状态一律以控制器为唯一准源。

        关节反馈是硬前提: 手眼需要的是**法兰**位姿, 只有关节编码器能给出与 tool 号
        无关的法兰位置; 笛卡尔读数混着当前 tool 偏置, 不能直接与外参同源。
        """
        from apps.robot.services.robot_service import robot_service

        if not robot_service.is_connected():
            raise RuntimeError("Robot is not connected. Connect the robot before capturing a sample.")
        if robot_service.is_moving():
            raise RuntimeError("Robot is moving. Wait until it stops, then capture the sample again.")

        settle_ms = self.sprayer.calib_capture_settle_ms
        if settle_ms > 0:
            time.sleep(settle_ms / 1000.0)
            if robot_service.is_moving():
                raise RuntimeError("Robot started moving during the settle delay. Capture aborted.")

        pose, err = robot_service.get_current_pose()
        if pose is None:
            raise RuntimeError(err or "Robot pose unavailable")
        joints, jerr = robot_service.get_current_joint()
        if joints is None or len(list(joints)) < 6:
            raise RuntimeError(jerr or "Robot joint feedback unavailable, cannot derive the flange pose.")
        diag = robot_service.get_feedback_diagnostics()
        return self.add_sample(session_id, pose, angle_unit=UNIT_RAD, joints_deg=joints,
                               tool_index=diag.get("tool_index"),
                               user_index=diag.get("user_index"))

    def _sample_flange_pose(self, s: Dict[str, Any],
                            pose_unit: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
        """
        取一个样本的法兰位姿 (4x4, 平移 mm) 与配套的欧拉读数 (deg), 返回 (T, pose, 来源)。

        优先级:
        1. 有 joints 就现场 FK 一次反推法兰 —— 它是唯一的原始信息, 且永远按**当前口径**
           重算; 采集时落盘的 flange_pose 只是那次 FK 的缓存, 一旦关节零位偏移改过,
           缓存就是旧口径的脏数据, 拿它重解会静默用错位姿 (实测差 17mm / 3°), 故降为
           无 joints 时的兼容兜底;
        2. 没 joints 的旧会话才用落盘的 flange_pose (恒为 mm/deg, 由 FK(joints) 得来);
        3. 两者都没才退回 robot_pose 原样读数 (当前 tool 的 TCP), 来源标 'tcp' 供上报。

        平移与旋转必须同源: 眼在手外的顺规网格搜索直接消费欧拉读数, 若平移按法兰解释
        而旋转按工具尖解释, 解出来的外参是错的。
        """
        derived = flange_pose_from_joints(s.get("joints"))
        if derived is not None:
            pose6 = normalize_pose(derived, UNIT_DEG)
            return pose_to_matrix(derived, UNIT_DEG), pose6, "derived"
        flange = s.get("flange_pose")
        if flange:
            pose6 = normalize_pose(flange, UNIT_DEG)
            return pose_to_matrix(flange, UNIT_DEG), pose6, "flange"
        pose = s.get("robot_pose")
        if pose:
            return pose_to_matrix(pose, pose_unit), normalize_pose(pose, pose_unit), "tcp"
        return None, None, "tcp"

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

        mount = resolve_result_mount(info)
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
        # 手眼约束里的 A 矩阵到底从哪里来: 法兰 (FK) / 旧会话补算 / TCP 读数兜底
        n_flange_pose = 0
        n_flange_derived = 0
        n_tcp_fallback = 0

        # Avoid thread over-subscription on RK3588
        cv2.setNumThreads(2)

        for idx, s in enumerate(info["samples"]):
            # Update progress
            safe_callback(idx + 1, total_samples, s["image_file"], "processing")

            T_flange, pose_dobot, pose_src = self._sample_flange_pose(s, pose_unit)
            if T_flange is None or any(abs(v) > 2500 for v in T_flange[:3, 3]):
                logger.warning(f"Skipped sample {s['id']} (invalid or missing robot pose)")
                time.sleep(0.06)
                continue
            if pose_src == "flange":
                n_flange_pose += 1
            elif pose_src == "derived":
                n_flange_derived += 1
            else:
                n_tcp_fallback += 1

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
                    T_base_flange=T_flange,
                    T_camera_board=make_transform(R_cb, tvec.flatten()),
                    pose_dobot=pose_dobot,
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

        quality = evaluate_data_quality(observations, mount)
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

        # 首轮拟合先用全部样本, 再按“每个样本自己的重投影残差”裁粗差, 只重解一次。
        # 旧的运动包络判据已删: 它对眼在上必须按 300~600mm 的作用距离张开区间, 实测一批
        # 12px 的数据一帧也剔不掉, 反而会把真正可用的大角度样本误删。
        solution = solve_hand_eye(mount, observations, K=K, D=D)
        if solution is None:
            safe_callback(total_samples, total_samples, "", "error")
            return {"success": False, "error": "Optimization solver failed."}

        pruned_ids: List[int] = []
        own_px = [solution.per_sample_reprojection_px.get(i) for i in range(len(observations))]
        samples, pruned_ids = prune_outliers(
            observations, own_px,
            max_error=self.sprayer.calib_pruning_max_px, unit="px", min_keep=min_samples,
            log_callback=lambda msg: logger.info(f"[prune] {msg}"))
        if pruned_ids:
            pruned = solve_hand_eye(mount, samples, K=K, D=D)
            # 裁完必须真的更好才采纳: 样本变少会让条件变差, 宁可交全样本的首轮解。
            if pruned is None or _solution_score(pruned) > _solution_score(solution):
                logger.warning(f"[prune] re-solve after dropping {pruned_ids} is not better "
                               f"(was {None if pruned is None else round(_solution_score(pruned), 4)}), "
                               f"keeping the first-pass fit on all samples")
                samples, pruned_ids = list(observations), []
            else:
                solution = pruned
                # 质量分必须跟着重算: 上报的 data_quality 要描述真正用掉的那批样本。
                quality = evaluate_data_quality(samples, mount)

        # 法兰到标定板的作用距离: 机械臂绝对姿态误差就是被这段杆臂放大成平移残差的。
        reach_mm = _board_reach_mm(mount, samples, solution)

        # 留一法不确定度: 重投影误差只说明样本之间多自洽, 说明不了外参本身能定到多准。
        # 必须传 K/D 走像素域精修路径 (闭式 AX=XB 路径对“手坐标系原点放在法兰还是 tool 尖”不协变)。
        # 代价是按上限重解若干次, 因此只在最终解出来后做这一次。
        uncertainty = extrinsic_uncertainty(mount, samples, K=K, D=D)

        # 免标定的判决实验: 臂说转了多少度 vs 相机看到板转了多少度 (共轭不改旋转角)。
        consistency = readout_rotation_consistency(samples)

        is_eto = mount == EYE_TO_HAND
        T_camera_mount = solution.T_base_camera if is_eto else solution.T_flange_camera
        mount_name = "base" if is_eto else "flange"
        pose_in_mount = matrix_to_pose(T_camera_mount)
        reproj_px = getattr(solution, "reprojection_error_px", None)
        confidence = assess_confidence(mount, quality, uncertainty, reproj_px, consistency)

        # 会话内所有样本共享同一 tool/user 配置 (采集时已互锁), 取首条作为代表上报
        first_sample = info["samples"][0] if info["samples"] else {}
        pose_frame = ("flange" if n_tcp_fallback == 0 else
                      "tcp" if n_flange_pose + n_flange_derived == 0 else "flange+tcp")

        # 法兰位姿口径声明 (与 hand_eye_mount 正交的第二个维度): 判定本体在
        # core.handeye.declare_pose_frame —— 只有全部样本都是现场带 Δq 的 FK 才计 v2,
        # 混进旧缓存或 TCP 读数就降级为 controller_v1, 宁可拒用不可谎报。
        offsets_deg = self.sprayer.robot_joint_offsets_deg
        runtime_frame = self.sprayer.pose_frame_convention
        declared_frame = declare_pose_frame(
            runtime_frame, n_flange_derived, n_flange_pose, n_tcp_fallback,
            len(observations))
        if declared_frame != runtime_frame:
            logger.warning(
                "Joint offsets configured but this session's poses are not all live FK; "
                f"declaring {POSE_FRAME_CONTROLLER_V1} for later consumption")

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
                # 手眼约束里 A 矩阵的参考系: flange = FK(joints), 与示教器 tool 号无关;
                # tcp = 直接用 tool_vector_actual 读数 (没关节反馈时的兜底, 外参会被 tool
                # 偏置吸收, 换 tool 号即失效); flange+tcp = 同一批里两种源头混用。
                "pose_reference": {
                    "frame": pose_frame,
                    "flange_recorded": n_flange_pose,
                    "flange_from_joints": n_flange_derived,
                    "tcp_readout_fallback": n_tcp_fallback,
                },
                "robot_configuration": {
                    "tool_index": first_sample.get("tool_index"),
                    "user_index": first_sample.get("user_index"),
                },
                # 口径声明: 加载时由 SprayerConfig.pose_frame_mismatch 硬拦不匹配的结果。
                # 同一装法下两种口径解出的外参不可互换 (实测两份 X 差 14.44mm / 1.59°), 且错用
                # 是静默的 —— 重投影误差与判决链吃的是同一批带口径的位姿, 自己看不出自相矛盾。
                "pose_frame_convention": declared_frame,
                "joint_offsets_deg": [float(v) for v in offsets_deg],
                # 首轮拟合后按自身残差裁掉的样本号 (空列表 = 没剔过)
                "pruned_sample_ids": list(pruned_ids),
                # 臂读数与视觉观测的相对转角失配 (含 per_sample_deg: 前端按样本号逐行标红)
                "readout_consistency": consistency or {},
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
                     "pixel_refined": bool(getattr(solution, "refined", False)),
                     "method_report": solution.method_report}
                ),
                # 这份外参能不能上机使用的结论: grade OK / CAUTION / NOT_USABLE。
                # 不确定度是留一法标准误 (不是样本噪声), 代表外参本身的可能浮动量级。
                "confidence": {
                    "grade": confidence["grade"],
                    "usable": confidence["usable"],
                    "warnings": confidence["warnings"],
                    "blocking": confidence["blocking"],
                    "extrinsic_std_translation_mm": (
                        uncertainty["translation_mm"] if uncertainty else None),
                    "extrinsic_std_axes_mm": (
                        uncertainty["axes_mm"] if uncertainty else None),
                    "extrinsic_std_rotation_deg": (
                        uncertainty["rotation_deg"] if uncertainty else None),
                    "jackknife_fits": uncertainty["fits"] if uncertainty else 0,
                    "jackknife_refined": bool(uncertainty and uncertainty["refined"]),
                },
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
            f"  - Samples: {len(samples)} used / {len(observations)} valid / {total_samples} captured"
            + (f"  [pruned {', '.join('#%d' % i for i in pruned_ids)} by own residual]"
               if pruned_ids else ""),
            f"  - Pose Reference: {pose_frame} "
            f"(recorded {n_flange_pose}, from joints {n_flange_derived}, "
            f"tcp readout {n_tcp_fallback}; tool "
            f"{first_sample.get('tool_index', 'N/A')}, user "
            f"{first_sample.get('user_index', 'N/A')})",
            f"  - Pose Frame Convention: {declared_frame} "
            f"(joint offsets {offsets_deg})"
            + ("" if declared_frame == self.sprayer.pose_frame_convention else
               "  [configured frame differs]"),
            f"  - Reprojection Error: "
            + ("N/A (no corner pixels)" if reproj_px is None else f"{float(reproj_px):.4f} px"),
            f"  - Kinematic Residual: {float(solution.translation_error_mm):.4f} mm, "
            f"{float(solution.rotation_error_deg):.4f} deg{reach_note}",
            f"  - Data Quality: {quality['score']:.1f}/100 "
            f"(axis coverage {quality['axis_coverage']:.2f})"
            + ("  [DEGENERATE]" if quality["degenerate"] else ""),
            "  - Readout Consistency (arm vs camera, relative rotation): "
            + ("N/A (no comparable pair)" if consistency is None else
               f"+/-{consistency['median_deg']:.3f} deg median, "
               f"+/-{consistency['p90_deg']:.3f} deg p90 over {consistency['pairs']} pairs "
               f"(worst {consistency['worst_pair']}: arm {consistency['worst_arm_deg']:.2f} deg "
               f"vs camera {consistency['worst_vision_deg']:.2f} deg)"),
            "  - Extrinsic Uncertainty (leave-one-out): "
            + ("N/A (too few samples)" if uncertainty is None else
               f"+/-{uncertainty['translation_mm']:.2f} mm / "
               f"+/-{uncertainty['rotation_deg']:.3f} deg over {uncertainty['fits']} re-fits"
               + ("" if uncertainty["refined"] else "  [CLOSED-FORM: frame-dependent]")),
            f"  - Verdict: {confidence['grade']}",
        ]
        for line in confidence["blocking"] + confidence["warnings"]:
            banner_lines.append(f"      ! {line}")
        banner_lines += [
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

        mount = resolve_result_mount(info)
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

        # 互锁: 自动重采会自己下发运动, 与在飞的点动/轨迹并发会让指令乱序, 起步前先以控制器反馈拦住
        if robot_service.is_moving():
            err_msg = "Robot is moving. Wait until it stops before starting automatic resampling."
            logger.error(f"[-] [Resample ERROR] {err_msg}")
            safe_callback(0, total_samples, "", "error", err_msg)
            return {"success": False, "error": err_msg}

        # 互锁: 重采会覆写旧样本, 当前 tool/user 配置与本会话已记录的不一致时直接拒跑
        live_diag = robot_service.get_feedback_diagnostics()
        conflict = _readout_config_conflict(samples, live_diag.get("tool_index"),
                                           live_diag.get("user_index"))
        if conflict:
            logger.error(f"[-] [Resample ERROR] {conflict}")
            safe_callback(0, total_samples, "", "error", conflict)
            return {"success": False, "error": conflict}

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

            # 2. Settle robot for 5.0s after reaching waypoint
            # Increased from 2.0s to 5.0s to allow inertial vibrations and bracket elastic deformation to fully decay.
            # Eye-in-hand calibration is extremely sensitive to residual motion: even 0.1mm translation or 0.5° rotation
            # error during capture will cause readout_consistency mismatch and lead to calibration failure.
            safe_callback(idx + 1, total_samples, img_filename, "settling", f"Waypoint {idx+1}/{total_samples} reached. Settling 5.0s...")
            time.sleep(5.0)

            # Query actual feedback pose & joints from robot encoders, store in the session's unit
            live_pose, _ = robot_service.get_current_pose()
            live_joints, _ = robot_service.get_current_joint()
            if live_pose and len(live_pose) >= 6:
                s["robot_pose"] = _pose_dict(normalize_pose(live_pose, UNIT_RAD))
            if live_joints and len(live_joints) >= 6:
                s["joints"] = [round(float(v), 4) for v in list(live_joints)[:6]]
                flange = flange_pose_from_joints(s["joints"])
                if flange is not None:
                    s["flange_pose"] = _pose_dict(flange)
            if live_diag.get("tool_index") is not None:
                s["tool_index"] = int(live_diag["tool_index"])
            if live_diag.get("user_index") is not None:
                s["user_index"] = int(live_diag["user_index"])

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
