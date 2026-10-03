import math
import os
import xml.etree.ElementTree as ET
import yaml
import logging
from typing import Any, Dict, List, Optional, Sequence, Tuple

logger = logging.getLogger(__name__)

# 项目根目录 (SprayAnything/)
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

# 旧版扁平配置键名与标准命名空间键名映射表 (保证平滑向后兼容)
_LEGACY_KEY_ALIASES: Dict[str, List[str]] = {
    "robot.ip": ["robot_ip"],
    "robot.port": ["robot_port"],
    "robot.home_position": ["robot_home_position"],
    "robot.fold_position": ["robot_fold_position"],
    "calib.board.cols": ["calib_board_cols"],
    "calib.board.rows": ["calib_board_rows"],
    "robot_ip": ["robot.ip"],
    "robot_port": ["robot.port"],
    "robot_home_position": ["robot.home_position"],
    "robot_fold_position": ["robot.fold_position"],
    "calib_board_cols": ["calib.board.cols"],
    "calib_board_rows": ["calib.board.rows"],
}

# 统一系统受管配置注册表 (Declarative System Configuration Registry)
CONFIG_REGISTRY: List[Dict[str, Any]] = [
    # ─── 1. 机械臂控制与工具端 (Robot Hardware & Tooling) ────────────────────
    {
        "key": "robot.ip",
        "category": "robot",
        "label": "Robot Controller IP",
        "type": "string",
        "yaml_path": "hardware.robot.ip",
        "default": "192.168.5.1",
        "description": "Network IPv4 address of the Dobot robot controller.",
        "legacy_key": "robot_ip",
    },
    {
        "key": "robot.port",
        "category": "robot",
        "label": "Robot Control Port",
        "type": "number",
        "yaml_path": "hardware.robot.port",
        "default": 29999,
        "min": 1,
        "max": 65535,
        "description": "TCP control port (Dobot default 29999).",
        "legacy_key": "robot_port",
    },
    {
        "key": "robot.global_speed_factor",
        "category": "robot",
        "label": "Global Speed Factor (%)",
        "type": "number",
        "yaml_path": "hardware.robot.global_speed_factor",
        "default": 50,
        "min": 1,
        "max": 100,
        "step": 5,
        "description": "Global velocity scaling factor applied on startup and motion (1-100%).",
    },
    {
        "key": "robot.spray_do_index",
        "category": "robot",
        "label": "Spraying DO Port Index",
        "type": "number",
        "yaml_path": "hardware.robot.spray_do_index",
        "default": 1,
        "min": 1,
        "max": 16,
        "step": 1,
        "description": "Digital output terminal index for spray gun trigger (1-16).",
    },
    {
        "key": "robot.robot_tcp_id",
        "category": "robot",
        "label": "Robot TCP Tool ID",
        "type": "select",
        "yaml_path": "hardware.robot.robot_tcp_id",
        "default": 1,
        "options": [
            {"value": 0, "label": "0 - Flange / Default Tool"},
            {"value": 1, "label": "1 - Gripper Tip (gripper_tip_link)"},
            {"value": 2, "label": "2 - Laser/Spray Head (laser_head_link)"},
        ],
        "description": "Controller tool coordinate frame index (0: Flange, 1: Gripper, 2: Laser/Spray).",
    },
    {
        "key": "robot.robot_tcp",
        "category": "robot",
        "label": "Robot TCP Node Name",
        "type": "select",
        "yaml_path": "hardware.robot.robot_tcp",
        "default": "gripper_tip_link",
        "options": [
            {"value": "laser_head_link", "label": "laser_head_link"},
            {"value": "gripper_tip_link", "label": "gripper_tip_link"},
        ],
        "description": "URDF TCP frame link name corresponding to active end-effector.",
    },
    {
        "key": "robot.robot_urdf",
        "category": "robot",
        "label": "Robot URDF Model Path",
        "type": "string",
        "yaml_path": "hardware.robot.robot_urdf",
        "default": "app/urdf/cr5_robot_with_my_tools.urdf",
        "description": "Relative or absolute path to robot kinematic URDF model.",
    },
    {
        "key": "robot.home_position",
        "category": "robot",
        "label": "Robot Home Position (deg)",
        "type": "vector6",
        "yaml_path": "hardware.robot.home_position",
        "default": [0.0, 0.0, -90.0, -90.0, -90.0, 0.0],
        "description": "Robot homing / reset joint angles in degrees [J1, J2, J3, J4, J5, J6].",
        "legacy_key": "robot_home_position",
    },
    {
        "key": "robot.fold_position",
        "category": "robot",
        "label": "Robot Fold Position (deg)",
        "type": "vector6",
        "yaml_path": "hardware.robot.fold_position",
        "default": [0.0, 0.0, -156.0, 0.0, -170.0, 0.0],
        "description": "Robot folded / storage joint angles in degrees [J1, J2, J3, J4, J5, J6].",
        "legacy_key": "robot_fold_position",
    },

    # ─── 2. 标定参数与标定板 (Calibration Target & Mount) ────────────────────
    {
        "key": "calib.mount",
        "category": "calib",
        "label": "Hand-Eye Mount Mode",
        "type": "select",
        "yaml_path": "calib.mount",
        "default": "eye-to-hand",
        "options": [
            {"value": "eye-to-hand", "label": "Eye-to-Hand (Camera Fixed, Board on Flange)"},
            {"value": "eye-in-hand", "label": "Eye-in-Hand (Camera on Flange, Board Fixed)"},
        ],
        "description": "Default kinematic mount structure for new calibration sessions.",
    },
    {
        "key": "calib.board.rows",
        "category": "calib",
        "label": "Checkerboard Rows",
        "type": "number",
        "yaml_path": "calib.board.rows",
        "default": 12,
        "min": 3,
        "max": 50,
        "step": 1,
        "description": "Number of checkerboard grid rows (external corner count).",
        "legacy_key": "calib_board_rows",
    },
    {
        "key": "calib.board.cols",
        "category": "calib",
        "label": "Checkerboard Columns",
        "type": "number",
        "yaml_path": "calib.board.cols",
        "default": 9,
        "min": 3,
        "max": 50,
        "step": 1,
        "description": "Number of checkerboard grid columns (external corner count).",
        "legacy_key": "calib_board_cols",
    },
    {
        "key": "calib.board.square_size_mm",
        "category": "calib",
        "label": "Checkerboard Square Size (mm)",
        "type": "number",
        "yaml_path": "calib.board.square_size_mm",
        "default": 15.0,
        "min": 1.0,
        "max": 200.0,
        "step": 0.5,
        "description": "Physical size of each grid square in millimeters.",
    },
    {
        "key": "calib.pruning.max_px",
        "category": "calib",
        "label": "Per-Sample Residual Limit (px)",
        "type": "number",
        "yaml_path": "calib.pruning.max_px",
        "default": 8.0,
        "min": 1.0,
        "max": 50.0,
        "step": 0.5,
        "description": "Drop samples whose own reprojection residual against the first-pass fit exceeds this limit, then re-solve once.",
    },
    {
        "key": "calib.capture.settle_ms",
        "category": "calib",
        "label": "Sample Settle Delay (ms)",
        "type": "number",
        "yaml_path": "calib.capture.settle_ms",
        "default": 250,
        "min": 0,
        "max": 2000,
        "step": 50,
        "description": "Wait after the controller reports Idle before grabbing a calibration frame, so residual flange vibration does not desync pose and image.",
    },

    # ─── 3. 喷涂与规划工艺参数 (Spraying & Planning Process) ─────────────────
    {
        "key": "spraying.spray_dist_mm",
        "category": "spraying",
        "label": "Spray Standoff Distance (mm)",
        "type": "number",
        "yaml_path": "spraying.spray_dist_mm",
        "default": 150.0,
        "min": 20.0,
        "max": 1000.0,
        "step": 5.0,
        "description": "Target nozzle-to-workpiece normal standoff distance in millimeters.",
    },
    {
        "key": "spraying.spray_width_mm",
        "category": "spraying",
        "label": "Spray Fan Width (mm)",
        "type": "number",
        "yaml_path": "spraying.spray_width_mm",
        "default": 50.0,
        "min": 5.0,
        "max": 500.0,
        "step": 1.0,
        "description": "Effective fan pattern width of the spray nozzle in millimeters.",
    },
    {
        "key": "spraying.overlap_rate",
        "category": "spraying",
        "label": "Pass Overlap Ratio",
        "type": "number",
        "yaml_path": "spraying.overlap_rate",
        "default": 0.2,
        "min": 0.0,
        "max": 0.8,
        "step": 0.05,
        "description": "Overlap fraction between adjacent spray passes (0.0 to 0.8).",
    },
    {
        "key": "spraying.point_spacing_mm",
        "category": "spraying",
        "label": "Waypoint Spacing (mm)",
        "type": "number",
        "yaml_path": "spraying.point_spacing_mm",
        "default": 100.0,
        "min": 5.0,
        "max": 500.0,
        "step": 5.0,
        "description": "Discretization distance between consecutive trajectory waypoints along each pass.",
    },
    {
        "key": "spraying.velocity",
        "category": "spraying",
        "label": "Spraying Velocity (mm/s)",
        "type": "number",
        "yaml_path": "spraying.velocity",
        "default": 150.0,
        "min": 10.0,
        "max": 2000.0,
        "step": 10.0,
        "description": "Nominal linear execution speed of the TCP during spraying passes.",
    },
    {
        "key": "spraying.poi_anchor_source",
        "category": "spraying",
        "label": "POI Anchor Orientation Source",
        "type": "select",
        "yaml_path": "spraying.poi_anchor_source",
        "default": "config",
        "options": [
            {"value": "config", "label": "Config (Fixed Reference Euler RPY)"},
            {"value": "home", "label": "Home (Robot Home Pose TCP)"},
            {"value": "raw", "label": "Raw (Per-point Surface Normal)"},
        ],
        "description": "Center orientation baseline for POI tolerance envelope constraints.",
    },
    {
        "key": "spraying.poi_ref_rpy_deg",
        "category": "spraying",
        "label": "POI Reference RPY (deg)",
        "type": "vector3",
        "yaml_path": "spraying.poi_ref_rpy_deg",
        "default": [90.0, 0.0, 90.0],
        "description": "Reference Euler RPY [Rx, Ry, Rz] in degrees when anchor source is 'config'.",
    },
    {
        "key": "spraying.poi_tolerance_rpy_deg",
        "category": "spraying",
        "label": "POI Tolerance Envelope [Rx, Ry, Rz] (deg)",
        "type": "vector3",
        "yaml_path": "spraying.poi_tolerance_rpy_deg",
        "default": [30.0, 30.0, 180.0],
        "description": "Permissible angular deviation envelope [Rx, Ry, Rz] around anchor orientation.",
    },
    {
        "key": "spraying.tol_ladder",
        "category": "spraying",
        "label": "Tolerance Ladder Optimization Guard",
        "type": "boolean",
        "yaml_path": "spraying.tol_ladder",
        "default": True,
        "description": "Enforce monotonic ladder optimization passes to prevent high-tolerance J6 velocity spikes.",
    },
    {
        "key": "spraying.tol_ladder_stop_peak_ratio",
        "category": "spraying",
        "label": "Ladder Early Stop Peak Ratio",
        "type": "number",
        "yaml_path": "spraying.tol_ladder_stop_peak_ratio",
        "default": 0.3,
        "min": 0.05,
        "max": 1.0,
        "step": 0.05,
        "description": "Stop ladder tightening when joint velocity peak ratio drops below this threshold.",
    },
    {
        "key": "spraying.singularity_speed_scaling",
        "category": "spraying",
        "label": "Wrist Singularity Auto Speed Scaling",
        "type": "boolean",
        "yaml_path": "spraying.singularity_speed_scaling",
        "default": False,
        "description": "Master switch for wrist-singularity (|J5|->0) auto slowdown. True: optimizer/verifier scale dt by manipulability AND the executor really lowers TCP speed near singularity using the per-waypoint profile; False: speed unchanged (both model & hardware).",
    },
    {
        "key": "spraying.spray_on_delay_ms",
        "category": "spraying",
        "label": "Spray Gun ON Delay (ms)",
        "type": "number",
        "yaml_path": "spraying.spray_on_delay_ms",
        "default": 0,
        "min": 0,
        "max": 5000,
        "step": 10,
        "description": "Dwell (ms) after issuing the spray-ON DO before continuing motion, to cover the gun's physical open / pressure-build lag. 0 = no delay (default).",
    },
    {
        "key": "spraying.spray_off_delay_ms",
        "category": "spraying",
        "label": "Spray Gun OFF Delay (ms)",
        "type": "number",
        "yaml_path": "spraying.spray_off_delay_ms",
        "default": 0,
        "min": 0,
        "max": 5000,
        "step": 10,
        "description": "Dwell (ms) after issuing the spray-OFF DO before continuing motion, to cover the gun's physical close / cut-off lag. 0 = no delay (default).",
    },

    # ─── 4. 视觉识别与交互式分割 (Vision & Interactive SAM) ─────────────────
    {
        "key": "interactive.detector.enabled",
        "category": "interactive",
        "label": "Enable Pre-Detection",
        "type": "boolean",
        "yaml_path": "interactive.detector.enabled",
        "default": True,
        "description": "Run Wissight object detector to initialize bounding box prompts before segmentation.",
    },
    {
        "key": "interactive.detector.sam_refine",
        "category": "interactive",
        "label": "Refine Mask with MobileSAM",
        "type": "boolean",
        "yaml_path": "interactive.detector.sam_refine",
        "default": False,
        "description": "Feed detected bounding box to MobileSAM for high-resolution contour refinement.",
    },
    {
        "key": "interactive.detector.classes",
        "category": "interactive",
        "label": "Detection Target Classes",
        "type": "tags",
        "yaml_path": "interactive.detector.classes",
        "default": ["trousers"],
        "description": "Target class names to filter from detector results (empty = allow all).",
    },
    {
        "key": "interactive.detector.conf",
        "category": "interactive",
        "label": "Detection Confidence Threshold",
        "type": "number",
        "yaml_path": "interactive.detector.conf",
        "default": 0.25,
        "min": 0.01,
        "max": 1.0,
        "step": 0.05,
        "description": "Minimum confidence score for Wissight garment detection.",
    },
    {
        "key": "interactive.detector.iou",
        "category": "interactive",
        "label": "NMS IoU Threshold",
        "type": "number",
        "yaml_path": "interactive.detector.iou",
        "default": 0.7,
        "min": 0.1,
        "max": 1.0,
        "step": 0.05,
        "description": "Non-Maximum Suppression (NMS) intersection-over-union threshold.",
    },
    {
        "key": "interactive.detector.max_boxes",
        "category": "interactive",
        "label": "Max Candidate Boxes",
        "type": "number",
        "yaml_path": "interactive.detector.max_boxes",
        "default": 5,
        "min": 1,
        "max": 50,
        "step": 1,
        "description": "Maximum number of candidate detection bounding boxes returned.",
    },
    {
        "key": "interactive.detector.backend",
        "category": "interactive",
        "label": "Detector Inference Backend",
        "type": "select",
        "yaml_path": "interactive.detector.backend",
        "default": "auto",
        "options": [
            {"value": "auto", "label": "Auto Detect (RKNN > CUDA > ONNX > PyTorch)"},
            {"value": "rknn", "label": "RKNN (Rockchip NPU)"},
            {"value": "onnx", "label": "ONNX Runtime"},
            {"value": "pt", "label": "PyTorch"},
        ],
        "description": "Inference acceleration backend for Wissight detector.",
    },
    {
        "key": "interactive.sam.backend",
        "category": "interactive",
        "label": "MobileSAM Inference Backend",
        "type": "select",
        "yaml_path": "interactive.sam.backend",
        "default": "auto",
        "options": [
            {"value": "auto", "label": "Auto Detect (RKNN > CUDA > ONNX > PyTorch)"},
            {"value": "rknn", "label": "RKNN (Rockchip NPU)"},
            {"value": "onnx", "label": "ONNX Runtime"},
            {"value": "pt", "label": "PyTorch"},
        ],
        "description": "Inference acceleration backend for MobileSAM interactive segmentation.",
    },
]


class SprayerConfig:
    """
    统一读取和解析 AiSprayer 系统配置 (aisprayer_config.yaml) 及相关引用的配置文件。
    单例模式：全局共享同一实例，支持三级梯级读取：
      [1. SQLite 数据库覆盖] -> [2. YAML 配置文件基线] -> [3. 代码默认值]
    """
    _instance = None

    def __new__(cls, config_path="configs/aisprayer_config.yaml", force_reload=False):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, config_path="configs/aisprayer_config.yaml", force_reload=False):
        if getattr(self, "_initialized", False) and not force_reload:
            return
        self.config_path = self._resolve_path(config_path)
        self._db_overrides: Dict[str, Any] = {}
        self.reload()
        self._initialized = True

    def reload(self):
        """重新从磁盘加载 YAML 配置文件与关联标定文件，并同步刷新数据库覆盖项"""
        self.config_data = self._load_yaml(self.config_path)
        
        # 自动加载关联的标定文件 (calibration_result.yaml)
        calib_rel_path = (
            self.config_data.get("spraying", {}).get("calib_path")
            or self.config_data.get("calib", {}).get("result_path")
        )
        self.calib_path = self._resolve_path(calib_rel_path) if calib_rel_path else None
        self.calib_data = self._load_yaml(self.calib_path) if self.calib_path else {}
        self.reload_db_overrides()

    def reload_db_overrides(self):
        """重新从 SQLite 数据库加载动态配置覆盖至内存缓存中 (避免高频轮询 DB I/O)"""
        try:
            from services.setting_service import SettingService
            self._db_overrides = SettingService().get_all_settings()
        except Exception as e:
            logger.warning(f"Failed to reload setting overrides from DB: {e}")
            self._db_overrides = {}

    def _get_yaml_nested(self, path: Optional[str], default: Any = None) -> Any:
        """根据点分路径从 self.config_data 中获取配置值，例如 'hardware.robot.ip'"""
        if not path or not self.config_data:
            return default
        curr = self.config_data
        for part in path.split("."):
            if isinstance(curr, dict) and part in curr:
                curr = curr[part]
            else:
                return default
        return curr

    def get_cascading(self, db_key: str, yaml_path: Optional[str] = None, default: Any = None) -> Any:
        """
        三级梯级读取机制：
        1. 优先从内存中的 SQLite 动态配置 (_db_overrides) 获取；
        2. 若不存在，检查兼容历史别名 (legacy aliases)；
        3. 若不存在，从 YAML 配置文件 (aisprayer_config.yaml) 中获取；
        4. 最终回退至代码默认值 (default)。
        """
        if db_key in self._db_overrides:
            return self._db_overrides[db_key]
        for alias in _LEGACY_KEY_ALIASES.get(db_key, []):
            if alias in self._db_overrides:
                return self._db_overrides[alias]
        if yaml_path:
            yaml_val = self._get_yaml_nested(yaml_path)
            if yaml_val is not None:
                return yaml_val
        return default

    def get_config_metadata(self) -> List[Dict[str, Any]]:
        """返回所有受管配置项的完整元数据定义与当前生效值、默认值及覆盖状态"""
        metadata_list = []
        for reg in CONFIG_REGISTRY:
            key = reg["key"]
            yaml_path = reg.get("yaml_path")
            default_val = reg.get("default")
            legacy_key = reg.get("legacy_key")
            
            yaml_val = self._get_yaml_nested(yaml_path, default_val) if yaml_path else default_val
            is_overridden = key in self._db_overrides or (legacy_key is not None and legacy_key in self._db_overrides)
            effective_val = self.get_cascading(key, yaml_path, default_val)
            
            item = {
                "key": key,
                "category": reg["category"],
                "label": reg["label"],
                "type": reg["type"],
                "value": effective_val,
                "yaml_default": yaml_val,
                "is_overridden": bool(is_overridden),
                "description": reg.get("description", ""),
            }
            if "min" in reg:
                item["min"] = reg["min"]
            if "max" in reg:
                item["max"] = reg["max"]
            if "step" in reg:
                item["step"] = reg["step"]
            if "options" in reg:
                item["options"] = reg["options"]
            if legacy_key:
                item["legacy_key"] = legacy_key
            metadata_list.append(item)
        return metadata_list

    def _resolve_path(self, path):
        if not path:
            return None
        if os.path.isabs(path):
            return path
        return os.path.join(PROJECT_ROOT, path)

    def _load_yaml(self, path):
        if not path or not os.path.exists(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            logger.warning(f"Failed to load yaml config from {path}: {e}")
            return {}

    @property
    def hand_eye_mount(self):
        """
        当前标定结果对应的相机安装方式: 'eye-to-hand' 或 'eye-in-hand'。

        历史结果文件没写这个字段 (或写的是旧的 calibration_mode), 一律按眼在手外
        处理 —— 那是本项目此前唯一支持的装法。判定口径共用 core.handeye 的
        resolve_result_mount, 与交互式重建选文件时读的是同一套规则。
        """
        if not self.calib_data:
            return self.calib_mount
        from core.handeye import resolve_result_mount
        return resolve_result_mount(self.calib_data, default=self.calib_mount)

    @property
    def robot_joint_offsets_deg(self) -> List[float]:
        """
        关节编码器零位偏移 Δq (度, 6 轴, 默认全零 = 不补偿)。

        它是**这台机器**的物理属性 (本项目跨会话实测稳到 0.34° 以内), 换机 / 大修 /
        碰撞后必须重解。合法性 (6 个值、有限、± 5° 以内) 由 CR5Kinematics 统一校验,
        这里只负责读出与默认值 —— 避免同一个校验在两处漂移。

        只从 YAML 读、不进 SQLite 覆盖: 机器物理参数不属于运行期可调项, 出现在 Settings
        里反而会被误改 (改了并不会自动重标, 只会让口径不匹配)。
        """
        val = self._get_yaml_nested("hardware.robot.joint_offsets_deg", [0.0] * 6)
        return [float(v) for v in val]

    @property
    def pose_frame_convention(self) -> str:
        """当前运行时的法兰位姿口径: 偏移全零 = controller_v1, 否则 joint_offset_v2。"""
        from core.handeye import POSE_FRAME_CONTROLLER_V1, POSE_FRAME_JOINT_OFFSET_V2
        offs = self.robot_joint_offsets_deg
        return (POSE_FRAME_CONTROLLER_V1 if all(abs(v) <= 1e-9 for v in offs)
                else POSE_FRAME_JOINT_OFFSET_V2)

    def pose_frame_mismatch(self, data: Optional[Dict[str, Any]] = None) -> Optional[str]:
        """
        对比一份标定结果的法兰位姿口径与当前运行时口径; 匹配返回 None, 不匹配返回英文原因。

        判定本体在 core.handeye.pose_frame_mismatch (与发布预检、界面待确认标记同一口径),
        本方法只负责把运行时的口径与偏移递过去。
        """
        from core.handeye import pose_frame_mismatch
        return pose_frame_mismatch(
            self.calib_data if data is None else data,
            self.pose_frame_convention,
            self.robot_joint_offsets_deg)

    @property
    def T_flange_camera(self):
        """眼在手上标定的相机安装外参 (4x4 列表, 平移 mm); 眼在手外时为 None。"""
        if not self.calib_data:
            return None
        return self.calib_data.get("T_flange_camera")

    def _camera_extrinsic(self, flange_pose_mm_deg: Optional[Sequence[Any]] = None):
        """
        把生效结果交给内核解析: (T_base_camera 米制或 None, mount, 英文原因/提醒)。

        递过去的就是“当前运行时口径 + 当前偏移”这一对事实, 口径判定本体不在本层重写。
        """
        from core.handeye import resolve_camera_extrinsic
        return resolve_camera_extrinsic(
            self.calib_data, flange_pose_mm_deg,
            runtime_frame=self.pose_frame_convention,
            runtime_offsets_deg=self.robot_joint_offsets_deg)

    def camera_extrinsic_at(self, base_flange_pose: Optional[Sequence[Any]] = None):
        """
        与 T_camera_to_base_at 同一条解析链, 但把 (T_base_camera, mount, 英文原因) 一起交回来。

        存在的理由: 实时视频上的“指尖指向”这类动作必须把**为什么用不了外参**说给用户听
        (相机此刻在哪 / 与标定口径不同源), 只返 None 就只能给出一句笼统的失败原因。
        日志口径与 T_camera_to_base_at 完全一致 (本方法就是它的底层实现), 返值多一个 mount 与原因。

        :return: (T_base_camera 4x4 列表或 None, mount, 英文原因或 None)
        """
        from core.handeye import EYE_IN_HAND
        pose = list(base_flange_pose or [])
        T, mount, note = self._camera_extrinsic(pose if len(pose) >= 6 else None)
        if T is None:
            # 没给法兰位姿是用法问题 (告警即可); 口径不同源是硬拦 (错一整个 Δq 在杆臂上的
            # 投影), 必须报 error —— 两者都会返回 None, 但后者意味着当场就不能用。
            if mount == EYE_IN_HAND and len(pose) >= 6:
                logger.error(f"Refusing eye-in-hand extrinsics: {note}")
            else:
                logger.warning(f"Camera extrinsics unavailable: {note}")
        return T, mount, note

    def T_camera_to_base_at(self, base_flange_pose):
        """
        指定法兰位姿下相机到基座的变换 (4x4 列表, 平移 m); 不可用时为 None。

        眼在手外: 与法兰无关, 直接返回标定的常量外参。
        眼在手上: T_base_camera = T_base_flange(pose) · T_flange_camera, 每次拍摄都不同。

        :param base_flange_pose: [x, y, z, rx, ry, rz], 平移 mm, 姿态度 (Dobot 'xyz' 内禀序列)

        入参必须是**法兰**位姿, 不能直接传 30004 的 tool_vector_actual: 后者是当前 tool
        的 TCP 位姿, 与法兰差一个常量 tool 变换 (本项目实测 252.7mm / 175.1°)。标定的
        T_flange_camera 是以法兰为参考解出的, 两者必须同源; 用关节反馈反推的法兰位姿
        (core.motion.kinematics.flange_pose_from_joints) 不受示教器 tool 号影响。

        判定本体 (含口径硬拦与 mm->m 换算) 在 core.handeye.resolve_camera_extrinsic,
        与交互式重建走的是同一条解析链, 不会出现在“配置层一套、交互层一套”。
        需要知道“为什么不可用”的调用方请用 camera_extrinsic_at。
        """
        return self.camera_extrinsic_at(base_flange_pose)[0]

    @property
    def T_camera_to_base(self):
        """手眼标定矩阵 (4x4 列表，平移部分被自动转换为米)。眼在手上时为 None, 改用 T_camera_to_base_at。"""
        if not self.calib_data:
            return None
        from core.handeye import EYE_IN_HAND
        T, mount, note = self._camera_extrinsic()
        if T is None:
            # 眼在手上时相机在基座系的位姿不是常量, 返回一个错误的常量比返回 None 危险得多
            if mount == EYE_IN_HAND and not getattr(self, "_warned_eye_in_hand", False):
                self._warned_eye_in_hand = True
                logger.warning(
                    "Active calibration is eye-in-hand: T_camera_to_base is not constant, "
                    "use T_camera_to_base_at(base_flange_pose)"
                )
            return None
        if note and not getattr(self, "_warned_pose_frame", False):
            # 口径提示 (不阻断): 眼在手外运行期不消费法兰位姿, 返的就是一个基座系常量,
            # 口径不一致只影响标定时被臂误差吸收掉的那一小部分 (实测 ±2.84mm 量级)。
            self._warned_pose_frame = True
            logger.warning(f"Stale pose frame in the active calibration result: {note}")
        return T

    @property
    def follow_calib_path(self) -> Optional[str]:
        """
        follow 链路 (C++ 跟随节点 + Python 轴映射) 用的标定结果路径。

        与 spraying.calib_path 分开配才能两种装法共存: 跟随时相机必须相对基座不动 (E2H),
        而交互页现在也支持眼在手上。共用一个键的话, 把全局结果切到 EIH 会连带把跟随静默
        降级为配置常量近似。未配则退回 spraying.calib_path (保持历史行为)。
        """
        path = self._get_yaml_nested("follow.runtime.calib_path")
        return self._resolve_path(path) if path else self.calib_path

    @property
    def follow_camera_to_base(self):
        """
        follow 侧的相机→基座常量外参 (4x4 列表, 平移 m); 没有常量可用时为 None。

        只读 follow_calib_path 那一份, 不读全局生效结果 —— 否则“切换装法”会顺带把跟随
        的轴映射弄没。眼在手上时本来就没有常量可用, 返 None 让调用方明确降级。
        """
        from core.handeye import resolve_camera_extrinsic
        T, _mount, note = resolve_camera_extrinsic(
            self._load_yaml(self.follow_calib_path),
            runtime_frame=self.pose_frame_convention,
            runtime_offsets_deg=self.robot_joint_offsets_deg)
        if T is None:
            logger.warning(f"follow extrinsics unusable: {note}")
        elif note and not getattr(self, "_warned_follow_pose_frame", False):
            # 常量仍可用, 但口径对不上: 只记一次, 跟 follow_camera_to_base 一样不阻断产线
            self._warned_follow_pose_frame = True
            logger.warning(f"Stale pose frame in the follow calibration: {note}")
        return T

    @property
    def model_path(self):
        """YOLO 分割模型路径 (自动解析为绝对路径)。"""
        path = self.config_data.get("spraying", {}).get("model_path")
        return self._resolve_path(path)

    @property
    def output_root(self):
        """生产运行数据存储根目录 (例如 data/runs) (自动解析为绝对路径)。"""
        path = self.config_data.get("spraying", {}).get("output_root", "data/runs")
        return self._resolve_path(path)

    @property
    def spray_width_mm(self) -> float:
        """喷涂幅宽 (mm, 默认 50.0)"""
        return float(self.get_cascading("spraying.spray_width_mm", "spraying.spray_width_mm", 50.0))

    @property
    def spray_width(self) -> float:
        """喷涂幅宽 (返回单位: 米)"""
        return self.spray_width_mm / 1000.0

    @property
    def spray_distance_mm(self) -> float:
        """默认喷涂靶距 / TCP standoff 距离 (mm, 默认 150.0)"""
        val = self.get_cascading("spraying.spray_dist_mm", "spraying.spray_dist_mm", None)
        if val is None:
            val = self._get_yaml_nested("spraying.spray_distance_mm", 150.0)
        return float(val)

    @property
    def spray_distance(self) -> float:
        """喷涂距离 (返回单位: 米)"""
        return self.spray_distance_mm / 1000.0

    @property
    def standoff_distance_mm(self) -> float:
        """TCP standoff 距离别名 (mm)"""
        return self.spray_distance_mm

    @property
    def aim_distance_mm(self) -> float:
        """
        实时视频“光束指向”里被点像素的**深度标注** (mm, 默认 600.0) —— 只用于界面回显。

        量纲与物理意义与 spray_dist_mm 不同, 不能混用: spray_dist_mm 是**枪口到工件表面**
        的法向 standoff (航点偏移量, 实时指向也拿它当“枪口至少离镜头多远”的安全下界);
        本值是**相机光心沿观测射线到被点那一点**的距离 —— 单个像素反投影只能给出一条射线
        (深度未知), 实时页面上又没拍深度图, 所以要给一个默认深度才能把像素标成基座系里的点。
        本项目标定板作业距离 300~900mm, 600mm 是中位数量级。

        注意: 指向本身**不吃这个值**。服务要求工具轴线与观测射线共线, 深度猜错也照样打中,
        改这里只改变界面上那个“目标点坐标”的数字, 不改变机械臂落点。
        """
        return float(self.get_cascading("spraying.aim_distance_mm", "spraying.aim_distance_mm", 600.0))

    @property
    def aim_speed_percent(self) -> float:
        """
        实时指向这一趟 MovJ 的速度百分比 (1~100, 默认 50.0)。

        为什么不跟控制面板的关节速度: 面板里那个值是为**喷涂走线**调的慢速 (默认 20 deg/s,
        在 CR5 的 180 deg/s 上限下只折算成 ~11%), 而指向是单点空走、不描轨迹, 用喷涂速度
        会显得拖沓 (真机反馈“到位很慢”)。接口显式传 speed (deg/s) 时仍以传入值为准。
        """
        return float(self.get_cascading("spraying.aim_speed_percent", "spraying.aim_speed_percent", 50.0))

    @property
    def aim_do_on_arrive(self) -> bool:
        """
        指向到位后是否自动打开喷涂 DO (默认 True)。

        物理意义: 当前该端口接的是**激光笔**, 到位亮激光用来肉眼确认光斑。
        换成真实喷枪必须设为 false —— 单点到位即开阀会在原地堆漆/滴漆。
        无论这一项怎么设, 动作前与任何异常路径都仍然强制关断 (fail-close),
        自动开阀只排在运动成功且读回反馈之后。
        """
        return bool(self.get_cascading("spraying.aim_do_on_arrive", "spraying.aim_do_on_arrive", True))


    @property
    def aim_frame_settle_s(self) -> float:
        """
        眼在手上实时指向前，臂必须已经静止的时长 (秒, 默认 2.0; 0 = 关闭这一项校验)。

        物理意义: 视频画面比现场落后**整条显示链路** (采集→OpenH264 编码→ZLM 推流→播放器
        缓冲, 本链路实测百毫秒到秒级, 相机重启/断流时更久), 而指向的视线是按**请求那一刻**的
        法兰位姿复合的。两者不同源时, 用户点的是旧像素、后端算的是新射线, 整条射线差的就是那
        段时间里臂的旋转量 (实测十几度 = 1m 外几十厘米), 而且本地指标量不出来 (光束与射线各自
        自洽)。所以要求静止时长至少覆盖显示延迟; 取 2.0s 为保守值, 链路更短可调小。
        眼在手外时相机不动, 视线与臂姿态无关, 这一项不适用 (服务层按装法跳过)。
        """
        val = self.get_cascading("spraying.aim_frame_settle_s", "spraying.aim_frame_settle_s", 2.0)
        return max(float(val), 0.0)

    @property
    def laser_tilt_tool_deg(self) -> List[float]:
        """
        激光光轴相对**工具 +Z** 的固定安装角偏 (工具系 [tx, ty], 单位 deg, 默认 [0, 0] = 不补偿)。

        物理意义: 规划一直硬编码"光束 == 工具 +Z", 但激光头与工具是刚性装配, 实际出光轴与 +Z
        之间会有一个固定的小角度安装偏差。本项目 5 组人工 Mark-cross 在不同臂姿态下量到该偏差
        在**工具系里几乎恒定** (tx≈-2.55°, ty≈+1.20°, 方位~155°, 大小~2.8°), 正是刚性安装角偏
        的特征 —— 与关节零位那种"随姿态变"的误差不同, 故可以用一个常量旋转吸收进外参。

        消费方式 (服务层三处同口径):
        - 规划 _pose_of: 把指令姿态预乘一个工具系"逆倾"旋转, 让**真实光轴**(而非 +Z) 落在被点视线上;
        - _measure / mark_cross 校核: 模型光束轴改用倾斜后的 R·b_tool, 于是补偿对了残差就收敛到 ~0。
        [0, 0] 时所有旋转退化为单位阵, 完全等价于旧的"光束==+Z"行为 (故障安全默认, 只从 YAML 读)。
        这是**这台机器 + 这个激光工装**的物理属性, 换机/大修/碰过支架后必须用 Mark-cross 重标。
        """
        val = self._get_yaml_nested("spraying.laser_tilt_tool_deg", [0.0, 0.0]) or [0.0, 0.0]
        try:
            tx, ty = float(val[0]), float(val[1])   # 只取前两个分量: tx 朝 +X 偏, ty 朝 +Y 偏
        except (TypeError, ValueError, IndexError) as e:
            raise ValueError(f"spraying.laser_tilt_tool_deg must be [tx, ty] in deg, got {val!r}: {e}")
        # 防御: 安装角偏是"小量", 超限说明配错 (填成了欧拉全姿态 / 度弧混淆), 快速失败而非带病运动。
        for name, v in (("tx", tx), ("ty", ty)):
            if not (-15.0 <= v <= 15.0):
                raise ValueError(f"spraying.laser_tilt_tool_deg.{name}={v} deg out of the plausible "
                                 f"laser-mount range [-15, 15]; check units (deg, not rad)")
        return [tx, ty]

    @property
    def overlap_rate(self) -> float:
        """喷幅重叠率 (0~1.0, 默认 0.2)"""
        return float(self.get_cascading("spraying.overlap_rate", "spraying.overlap_rate", 0.2))

    @property
    def row_spacing_mm(self) -> float:
        """自动规划行间距 (mm, 默认根据 spray_width_mm * (1 - overlap_rate) 计算)"""
        spraying_cfg = self.config_data.get("spraying", {})
        if "row_spacing_mm" in spraying_cfg and spraying_cfg["row_spacing_mm"]:
            return float(spraying_cfg["row_spacing_mm"])
        return self.spray_width_mm * (1.0 - self.overlap_rate)

    @property
    def point_spacing_mm(self) -> float:
        """自动规划沿行点间距 (mm, 默认 100.0)"""
        val = self.get_cascading("spraying.point_spacing_mm", "spraying.point_spacing_mm", None)
        if val is not None:
            return float(val)
        v_step = self._get_yaml_nested("spraying.v_step_mm")
        if v_step is not None:
            return float(v_step)
        return 100.0

    @property
    def spraying_velocity(self) -> float:
        """喷涂移动速度 (mm/s, 默认 150.0)"""
        return float(self.get_cascading("spraying.velocity", "spraying.velocity", 150.0))

    @property
    def slerp_step_mm(self) -> float:
        """轨迹验证与仿真插值步长 (mm, 默认 2.0)"""
        val = self.get_cascading("spraying.slerp_step_mm", "spraying.slerp_step_mm", 2.0)
        return float(val)

    @property
    def urdf_path(self) -> str:
        """机器人 URDF 模型文件路径 (绝对路径)"""
        path = self.get_cascading("robot.robot_urdf", "hardware.robot.robot_urdf", "app/urdf/cr5_robot_with_my_tools.urdf")
        return self._resolve_path(path)

    @property
    def robot_urdf(self) -> str:
        """机器人 URDF 模型文件路径别名 (绝对路径)"""
        return self.urdf_path

    @property
    def robot_ip(self) -> str:
        """机器人控制器 IP 地址"""
        return str(self.get_cascading("robot.ip", "hardware.robot.ip", "192.168.5.1"))

    @property
    def robot_port(self) -> int:
        """机器人控制端口 (Dobot 默认 29999)"""
        return int(self.get_cascading("robot.port", "hardware.robot.port", 29999))

    @property
    def robot_tcp_id(self) -> int:
        """
        机械臂末端工具坐标系 ID (0: 默认法兰/工具0, 1: gripper_tip_link, 2: laser_head_link)。
        """
        val = self.get_cascading("robot.robot_tcp_id", "hardware.robot.robot_tcp_id", None)
        if val is not None:
            try:
                return int(val)
            except (ValueError, TypeError):
                pass
        # 若未显式配置 robot_tcp_id，则根据 robot_tcp 名称自动推导
        tcp_name = str(self.robot_tcp).lower()
        if any(k in tcp_name for k in ["grip", "finger", "tip"]):
            return 1
        elif any(k in tcp_name for k in ["laser", "nozzle", "spray", "gun"]):
            return 2
        return 0

    @property
    def robot_tcp(self) -> str:
        """机器人末端工具 TCP 节点名称 (例如 laser_head_link, gripper_tip_link)"""
        val = self.get_cascading("robot.robot_tcp", "hardware.robot.robot_tcp", None)
        if val:
            return str(val).strip()
        # 若未显式指定名称，则根据 robot_tcp_id 映射
        tcp_id = self.robot_tcp_id
        if tcp_id == 1:
            return "gripper_tip_link"
        elif tcp_id == 2:
            return "laser_head_link"
        return "gripper_tip_link"

    @property
    def spray_do_index(self) -> int:
        """
        机械臂喷涂开关数字输出端口 (DO) 编号 (1-based, 取值范围 1-16, 默认 1)。
        """
        val = self.get_cascading("robot.spray_do_index", "hardware.robot.spray_do_index", 1)
        try:
            index = int(val)
            if 1 <= index <= 16:
                return index
        except (ValueError, TypeError):
            pass
        return 1

    @property
    def home_position(self) -> List[float]:
        """
        机械臂原点/复位关节角 (单位: 度, [J1, J2, J3, J4, J5, J6])。
        优先从数据库读取，其次从 aisprayer_config.yaml 中读取，最后保底默认值。
        """
        val = self.get_cascading("robot.home_position", "hardware.robot.home_position", [0.0, 0.0, -90.0, -90.0, -90.0, 0.0])
        if isinstance(val, (list, tuple)) and len(val) == 6:
            try:
                return [float(x) for x in val]
            except (ValueError, TypeError):
                pass
        return [0.0, 0.0, -90.0, -90.0, -90.0, 0.0]

    @property
    def fold_position(self) -> List[float]:
        """
        机械臂折叠/收纳关节角 (单位: 度, [J1, J2, J3, J4, J5, J6])。
        优先从数据库读取，其次从 aisprayer_config.yaml 中读取，最后保底默认值。
        """
        val = self.get_cascading("robot.fold_position", "hardware.robot.fold_position", [0.0, 0.0, -156.0, 0.0, -170.0, 0.0])
        if isinstance(val, (list, tuple)) and len(val) == 6:
            try:
                return [float(x) for x in val]
            except (ValueError, TypeError):
                pass
        return [0.0, 0.0, -156.0, 0.0, -170.0, 0.0]

    @property
    def calib_mount(self) -> str:
        """新建标定会话默认相机安装方式: 'eye-to-hand' 或 'eye-in-hand'"""
        val = str(self.get_cascading("calib.mount", "calib.mount", "eye-to-hand")).strip().lower()
        return "eye-in-hand" if val == "eye-in-hand" else "eye-to-hand"

    @property
    def calib_board_cols(self) -> int:
        """标定板列数 (节点数)"""
        return int(self.get_cascading("calib.board.cols", "calib.board.cols", 9))

    @property
    def calib_board_rows(self) -> int:
        """标定板行数 (节点数)"""
        return int(self.get_cascading("calib.board.rows", "calib.board.rows", 12))

    @property
    def calib_board_square_size_mm(self) -> float:
        """标定板方格物理尺寸 (mm)"""
        return float(self.get_cascading("calib.board.square_size_mm", "calib.board.square_size_mm", 15.0))

    @property
    def calib_pruning_max_px(self) -> float:
        """
        单样本重投影残差上限 (像素, 1~50), 超出则该样本被剔除后重解一次。

        判据用的是首轮拟合对每个样本自己的残差, 比任何先验运动包络都直接。
        非法输入 (非数字/越界) 一律收敛到边界, 不带病运行。
        """
        val = self.get_cascading("calib.pruning.max_px", "calib.pruning.max_px", 8.0)
        try:
            return min(max(float(val), 1.0), 50.0)
        except (ValueError, TypeError):
            return 8.0

    @property
    def calib_capture_settle_ms(self) -> int:
        """
        手眼采样前的沉降等待 (毫秒, 0~2000)。

        控制器报 Idle 只代表插补结束, 法兰仍有残余振动; 立即拍照会让“位姿读数”与
        “图像里的标定板”不是同一时刻, 直接污染 AX=XB 约束。0 = 不等待。
        非法输入 (非数字/负数/超过 2s) 一律收敛到边界, 不带病运行。
        """
        val = self.get_cascading("calib.capture.settle_ms", "calib.capture.settle_ms", 250)
        try:
            return min(max(int(val), 0), 2000)
        except (ValueError, TypeError):
            return 250

    @property
    def camera_model(self) -> str:
        return str(self._get_yaml_nested("hardware.camera.model", "orbbec"))

    @property
    def global_speed_factor(self) -> int:
        """示教/远程/运行全局速度百分比 (1-100%, 默认 50)"""
        val = self.get_cascading("robot.global_speed_factor", "hardware.robot.global_speed_factor", None)
        if val is None:
            val = self._get_yaml_nested("hardware.robot.global_speed_percent", 50)
        return int(val)

    @property
    def global_speed_percent(self) -> int:
        """全局速度百分比（兼容别名）"""
        return self.global_speed_factor

    @property
    def max_tcp_speed_mm_s(self) -> float:
        """机器人最大末端 TCP 线速度 (mm/s, 默认 2000.0)"""
        val = self._get_yaml_nested("hardware.robot.max_tcp_speed_mm_s", 2000.0)
        return float(val)

    @property
    def max_joint_speed_deg_s(self) -> List[float]:
        """机器人最大关节速度 (度/s, 6轴列表, 默认 [180, 180, 180, 180, 180, 180])"""
        speeds = self._get_yaml_nested("hardware.robot.max_joint_speed_deg_s", [180.0, 180.0, 180.0, 180.0, 180.0, 180.0])
        return [float(x) for x in speeds]

    @property
    def robot_max_reach_mm(self) -> float:
        """
        机械臂臂展 (基座到**法兰**的最大直线距离, mm, 默认 900.0 = CR5 臂展)。

        物理意义: 以基座为球心的**可达球**半径。它只用来把候选位置做几何筛除 (避开
        那些根本够不着的点), 真正的可达/奇异判定仍然以控制器逆解为准 —— 球内不等
        于可达 (关节限位、奇异区、本体干涉都不在球模型里)。
        口径注意: 这一项是**法兰**半径, 不是带工具的 TCP 半径。实时指向候选的是 TK 标定后的
        TCP, 所以它会把实测的工具长度 (|30004 TCP 读数 - FK 法兰|) 叠加到球半径上; 直接用本值
        筛 TCP 会少算一整截工具 (本项目实测 153mm), 把可用工作区错杀成很小一段。
        换臂型 (如 M1 系 700mm / CR3 系 600mm) 必须改这一项, 否则筛除结果会偏保守或偏激进。
        """
        return float(self._get_yaml_nested("hardware.robot.max_reach_mm", 900.0))

    @property
    def poi_tolerance_rpy_deg(self) -> List[float]:
        """POI 锚点姿态容差包络 [Rx, Ry, Rz] (度)"""
        val = self.get_cascading("spraying.poi_tolerance_rpy_deg", "spraying.poi_tolerance_rpy_deg", None)
        if val is None:
            val = self._get_yaml_nested("optimization.poi_tolerance_rpy_deg", [30.0, 30.0, 180.0])
        return [float(v) for v in val]

    @property
    def poi_anchor_source(self) -> str:
        """POI 锚点(容差包络中心)来源: 'config' | 'home' | 'raw'"""
        val = self.get_cascading("spraying.poi_anchor_source", "spraying.poi_anchor_source", None)
        if val is None:
            val = self._get_yaml_nested("optimization.poi_anchor_source", "config")
        src = str(val).strip().lower()
        return src if src in {"config", "home", "raw"} else "config"

    @property
    def poi_ref_rpy_deg(self) -> Optional[List[float]]:
        """POI 锚点参考姿态 [Rx, Ry, Rz] (度, Euler 'xyz'); 未配置则返回 None"""
        val = self.get_cascading("spraying.poi_ref_rpy_deg", "spraying.poi_ref_rpy_deg", None)
        if val is None:
            val = self._get_yaml_nested("optimization.poi_ref_rpy_deg", None)
        if not val or len(val) != 3:
            return None
        return [float(v) for v in val]

    @property
    def tol_ladder(self) -> bool:
        """容差阶梯择优 (Monotonicity Guard) 开关"""
        return bool(self.get_cascading("spraying.tol_ladder", "spraying.tol_ladder", True))

    @property
    def tol_ladder_stop_peak_ratio(self) -> float:
        """容差阶梯早停阈值比例"""
        return float(self.get_cascading("spraying.tol_ladder_stop_peak_ratio", "spraying.tol_ladder_stop_peak_ratio", 0.3))

    @property
    def singularity_speed_scaling(self) -> bool:
        """
        腕部奇异(|J5|->0)自适应降速总开关 (与 motion_cli 侧 spraying.singularity_speed_scaling 同源)。
        True: 优化器/校验器按可操作度缩放 dt, 且执行侧按 poi yaml 的逐航点剖面真实压低 TCP 线速度;
        False: 模型与真机均不改变速度 (零回归, 默认关, 待真机 dry-run 验证后再开)。
        """
        return bool(self.get_cascading("spraying.singularity_speed_scaling", "spraying.singularity_speed_scaling", False))

    @property
    def spray_on_delay_ms(self) -> int:
        """喷枪开启响应延迟 (ms): 下发开喷 DO 后驻留此时长再继续下发本段 MoveL; 0 = 不延迟 (默认)。"""
        return int(self.get_cascading("spraying.spray_on_delay_ms", "spraying.spray_on_delay_ms", 0) or 0)

    @property
    def spray_off_delay_ms(self) -> int:
        """喷枪关闭响应延迟 (ms): 下发关喷 DO 后驻留此时长再继续下发本段 MoveL; 0 = 不延迟 (默认)。"""
        return int(self.get_cascading("spraying.spray_off_delay_ms", "spraying.spray_off_delay_ms", 0) or 0)

    @property
    def grid_tol_x_deg(self) -> Tuple[float, float, float]:
        """轨迹优化器 X 轴搜索网格 (min, max, step) (度)"""
        val = self._get_yaml_nested("spraying.grid_tol_x_deg") or self._get_yaml_nested("optimization.grid_tol_x_deg", [-5.0, 5.0, 2.0])
        return tuple(float(v) for v in val)

    @property
    def grid_tol_y_deg(self) -> Tuple[float, float, float]:
        """轨迹优化器 Y 轴搜索网格 (min, max, step) (度)"""
        val = self._get_yaml_nested("spraying.grid_tol_y_deg") or self._get_yaml_nested("optimization.grid_tol_y_deg", [-5.0, 5.0, 2.0])
        return tuple(float(v) for v in val)

    @property
    def grid_tol_z_deg(self) -> Tuple[float, float, float]:
        """轨迹优化器 Z 轴搜索网格 (min, max, step) (度)"""
        val = self._get_yaml_nested("spraying.grid_tol_z_deg") or self._get_yaml_nested("optimization.grid_tol_z_deg", [-30.0, 30.0, 5.0])
        return tuple(float(v) for v in val)

    # ─── 交互式分割与自动检测 (interactive.*) ────────────────────────────────
    @property
    def sam_backend(self) -> str:
        """MobileSAM 推理后端：auto | rknn | onnx | pt。"""
        return str(self.get_cascading("interactive.sam.backend", "interactive.sam.backend", "auto") or "auto").strip()

    @property
    def detector_enabled(self) -> bool:
        """进入交互分割时是否先跑目标检测出框（false = 维持纯手动点选行为）。"""
        return bool(self.get_cascading("interactive.detector.enabled", "interactive.detector.enabled", True))

    @property
    def detector_backend(self) -> str:
        """Wissight 推理后端：auto | rknn | onnx | pt。"""
        return str(self.get_cascading("interactive.detector.backend", "interactive.detector.backend", "auto") or "auto").strip()

    @property
    def detector_classes(self) -> List[str]:
        """允许当成 SAM prompt 的类别名；空列表 = 不按类别过滤。"""
        classes = self.get_cascading("interactive.detector.classes", "interactive.detector.classes", ["trousers"])
        return [str(c).strip() for c in classes] if classes else []

    @property
    def detector_conf(self) -> float:
        """检测置信度阈值。"""
        return float(self.get_cascading("interactive.detector.conf", "interactive.detector.conf", 0.25))

    @property
    def detector_iou(self) -> float:
        """NMS IoU 阈值。"""
        return float(self.get_cascading("interactive.detector.iou", "interactive.detector.iou", 0.7))

    @property
    def detector_max_boxes(self) -> int:
        """接口最多回传几个候选框。"""
        return int(self.get_cascading("interactive.detector.max_boxes", "interactive.detector.max_boxes", 5))

    @property
    def detector_sam_refine(self) -> bool:
        """检到目标后是否再用 MobileSAM 精修。"""
        return bool(self.get_cascading("interactive.detector.sam_refine", "interactive.detector.sam_refine", False))


# ─── 全局单例对象 (模块导入时完成初始化与加载) ──────────────────────────────────
config = SprayerConfig()
sprayer_config = config


def get_config() -> SprayerConfig:
    """获取全局配置单例对象"""
    return config


def get_configured_robot_config(config_path: str = None) -> tuple[str, str]:
    """统一从全局配置获取 (urdf_abs_path, tcp_target_link)。"""
    cfg = config if config_path is None else SprayerConfig(config_path=config_path)
    return cfg.robot_urdf, cfg.robot_tcp


def load_tcp_from_urdf(urdf_path: str = None, target_tcp_name: str = None) -> dict:
    """从 URDF 解析挂在 Link6/法兰上的工具 TCP（毫米 / 度），给 Web 回显用。"""
    if urdf_path is None or target_tcp_name is None:
        cfg_urdf, cfg_tcp = get_configured_robot_config()
        if urdf_path is None:
            urdf_path = cfg_urdf
        if target_tcp_name is None:
            target_tcp_name = cfg_tcp

    tcp_info = {
        "has_tool": False,
        "tool_name": "flange",
        "xyz_mm": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "urdf_source": os.path.basename(urdf_path) if urdf_path else None,
    }
    if not urdf_path or not os.path.exists(urdf_path):
        return tcp_info

    try:
        root = ET.parse(urdf_path).getroot()
        best_score = -1
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or parent.get("link") not in ["Link6", "link6", "flange"]:
                continue
            origin = joint.find("origin")
            if origin is None:
                continue
            child_name = child.get("link", "") if child is not None else ""
            xyz_m = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy_rad = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            xyz_mm = [round(v * 1000.0, 2) for v in xyz_m]
            rpy_deg = [round(math.degrees(v), 2) for v in rpy_rad]
            score = 0
            if target_tcp_name and (
                child_name.lower() == target_tcp_name.lower()
                or target_tcp_name.lower() in child_name.lower()
            ):
                score = 1000
            elif any(k in child_name.lower() for k in ["laser", "nozzle", "tcp"]):
                score = 100
            elif "tip" in child_name.lower():
                score = 80
            elif "gun" in child_name.lower():
                score = 50
            elif "tool" in child_name.lower():
                score = 30
            if score > best_score:
                best_score = score
                tcp_info = {
                    "has_tool": True,
                    "tool_name": child_name,
                    "xyz_mm": xyz_mm,
                    "rpy_deg": rpy_deg,
                    "urdf_source": os.path.basename(urdf_path),
                }
    except Exception as e:
        logger.warning("Could not parse TCP from URDF %s: %s", urdf_path, e)
    return tcp_info

