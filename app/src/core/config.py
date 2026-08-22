import os
import yaml
import logging

logger = logging.getLogger(__name__)

# 项目根目录 (SprayAnything/)
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))


class SprayerConfig:
    """
    统一读取和解析 AiSprayer 系统配置 (aisprayer_config.yaml) 及相关引用的配置文件。
    单例模式：全局共享同一实例，初始化时一次性加载所有配置数据。
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
        self.reload()
        self._initialized = True

    def reload(self):
        """重新从磁盘加载 YAML 配置文件与关联标定文件"""
        self.config_data = self._load_yaml(self.config_path)
        
        # 自动加载关联的标定文件 (calibration_result.yaml)
        calib_rel_path = self.config_data.get("vision", {}).get("planner", {}).get("calib_path")
        self.calib_path = self._resolve_path(calib_rel_path) if calib_rel_path else None
        self.calib_data = self._load_yaml(self.calib_path) if self.calib_path else {}

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
    def T_camera_to_base(self):
        """手眼标定矩阵 (4x4 列表，平移部分被自动转换为米)"""
        if self.calib_data:
            key = 'T_base_camera' if 'T_base_camera' in self.calib_data else ('T_camera_to_base' if 'T_camera_to_base' in self.calib_data else None)
            if key:
                import copy
                T = copy.deepcopy(self.calib_data[key])
                # 标定文件中的平移部分是以毫米为单位保存的 (例如 847.1)
                # 但后续 3D 处理流水线 (点云/URDF/规划) 均使用米 (m)
                # 故在此处统一将平移部分缩放为米
                T[0][3] /= 1000.0
                T[1][3] /= 1000.0
                T[2][3] /= 1000.0
                return T
        return None

    @property
    def model_path(self):
        """YOLO 分割模型路径 (自动解析为绝对路径)。"""
        path = self.config_data.get("vision", {}).get("planner", {}).get("model_path")
        return self._resolve_path(path)

    @property
    def output_root(self):
        """生产运行数据存储根目录 (例如 data/runs) (自动解析为绝对路径)。"""
        path = self.config_data.get("vision", {}).get("output_root", "data/runs")
        return self._resolve_path(path)

    @property
    def spray_width(self):
        """喷涂幅宽 (返回单位: 米)"""
        mm = self.config_data.get("vision", {}).get("planner", {}).get("spray_width_mm", 100.0)
        return float(mm) / 1000.0

    @property
    def spray_distance(self):
        """喷涂距离 (返回单位: 米)"""
        return self.spray_distance_mm / 1000.0

    @property
    def urdf_path(self):
        """机器人 URDF 模型文件路径 (绝对路径)"""
        path = self.config_data.get("hardware", {}).get("robot", {}).get("robot_urdf", "app/urdf/cr5_robot.urdf")
        return self._resolve_path(path)

    @property
    def robot_urdf(self):
        """机器人 URDF 模型文件路径别名 (绝对路径)"""
        return self.urdf_path

    @property
    def robot_ip(self):
        return self.config_data.get("hardware", {}).get("robot", {}).get("ip")

    @property
    def robot_port(self):
        return self.config_data.get("hardware", {}).get("robot", {}).get("port", 6001)

    @property
    def robot_tcp(self):
        """机器人末端工具 TCP 节点名称 (例如 spray_nozzle_link)"""
        return self.config_data.get("hardware", {}).get("robot", {}).get("robot_tcp", "spray_nozzle_link")

    @property
    def camera_model(self):
        return self.config_data.get("hardware", {}).get("camera", {}).get("model", "orbbec")

    @property
    def global_speed_factor(self) -> int:
        """示教/远程/运行全局速度百分比 (startup 时 set_speed, 默认 50)"""
        robot_cfg = self.config_data.get("hardware", {}).get("robot", {})
        return int(robot_cfg.get("global_speed_factor", robot_cfg.get("global_speed_percent", 50)))

    @property
    def global_speed_percent(self) -> int:
        """全局速度百分比（兼容别名）"""
        return self.global_speed_factor

    @property
    def max_tcp_speed_mm_s(self) -> float:
        """机器人最大末端 TCP 线速度 (mm/s, 默认 2000.0)"""
        robot_cfg = self.config_data.get("hardware", {}).get("robot", {})
        return float(robot_cfg.get("max_tcp_speed_mm_s", 2000.0))

    @property
    def max_joint_speed_deg_s(self) -> list[float]:
        """机器人最大关节速度 (度/s, 6轴列表, 默认 [180, 180, 180, 180, 180, 180])"""
        robot_cfg = self.config_data.get("hardware", {}).get("robot", {})
        speeds = robot_cfg.get("max_joint_speed_deg_s", [180.0, 180.0, 180.0, 180.0, 180.0, 180.0])
        return [float(x) for x in speeds]

    @property
    def spraying_velocity(self) -> float:
        """喷涂移动速度 (mm/s, 默认 150.0)"""
        return float(self.config_data.get("spraying", {}).get("velocity", 150.0))

    @property
    def slerp_step_mm(self) -> float:
        """轨迹验证与仿真插值步长 (mm, 默认 1.5)"""
        return float(self.config_data.get("spraying", {}).get("slerp_step_mm", 1.5))

    @property
    def spray_distance_mm(self) -> float:
        """默认喷涂靶距 / TCP standoff 距离 (mm, 默认 150.0)"""
        spraying_cfg = self.config_data.get("spraying", {})
        planner_cfg = self.config_data.get("vision", {}).get("planner", {})
        mm = spraying_cfg.get("standoff_dist_mm") or planner_cfg.get("spray_dist_mm") or 150.0
        return float(mm)

    @property
    def row_spacing_mm(self) -> float:
        """自动规划行间距 (mm, 默认根据 spray_width_mm * (1 - overlap_rate) 计算)"""
        spraying_cfg = self.config_data.get("spraying", {})
        if "row_spacing_mm" in spraying_cfg and spraying_cfg["row_spacing_mm"]:
            return float(spraying_cfg["row_spacing_mm"])
        planner_cfg = self.config_data.get("vision", {}).get("planner", {})
        if "row_spacing_mm" in planner_cfg and planner_cfg["row_spacing_mm"]:
            return float(planner_cfg["row_spacing_mm"])
        width_mm = float(planner_cfg.get("spray_width_mm", 100.0))
        overlap = float(planner_cfg.get("overlap_rate", 0.2))
        return width_mm * (1.0 - overlap)

    @property
    def point_spacing_mm(self) -> float:
        """自动规划沿行点间距 (mm, 默认从 spraying.point_spacing_mm 或 vision.planner.v_step_mm 读取)"""
        spraying_cfg = self.config_data.get("spraying", {})
        if "point_spacing_mm" in spraying_cfg and spraying_cfg["point_spacing_mm"]:
            return float(spraying_cfg["point_spacing_mm"])
        planner_cfg = self.config_data.get("vision", {}).get("planner", {})
        if "point_spacing_mm" in planner_cfg and planner_cfg["point_spacing_mm"]:
            return float(planner_cfg["point_spacing_mm"])
        if "v_step_mm" in planner_cfg and planner_cfg["v_step_mm"]:
            return float(planner_cfg["v_step_mm"])
        return 20.0

    @property
    def standoff_distance_mm(self) -> float:
        """TCP standoff 距离别名 (mm)"""
        return self.spray_distance_mm

    @property
    def poi_tolerance_rpy_deg(self) -> list[float]:
        """POI 锚点姿态容差包络 [Rx, Ry, Rz] (度)"""
        spraying_cfg = self.config_data.get("spraying", {})
        opt_cfg = self.config_data.get("optimization", {})
        tol = spraying_cfg.get("poi_tolerance_rpy_deg") or opt_cfg.get("poi_tolerance_rpy_deg") or [10.0, 10.0, 180.0]
        return [float(v) for v in tol]

    @property
    def grid_tol_x_deg(self) -> tuple[float, float, float]:
        """轨迹优化器 X 轴搜索网格 (min, max, step) (度)"""
        spraying_cfg = self.config_data.get("spraying", {})
        opt_cfg = self.config_data.get("optimization", {})
        val = spraying_cfg.get("grid_tol_x_deg") or opt_cfg.get("grid_tol_x_deg") or [-5.0, 5.0, 2.0]
        return tuple(float(v) for v in val)

    @property
    def grid_tol_y_deg(self) -> tuple[float, float, float]:
        """轨迹优化器 Y 轴搜索网格 (min, max, step) (度)"""
        spraying_cfg = self.config_data.get("spraying", {})
        opt_cfg = self.config_data.get("optimization", {})
        val = spraying_cfg.get("grid_tol_y_deg") or opt_cfg.get("grid_tol_y_deg") or [-5.0, 5.0, 2.0]
        return tuple(float(v) for v in val)

    @property
    def grid_tol_z_deg(self) -> tuple[float, float, float]:
        """轨迹优化器 Z 轴搜索网格 (min, max, step) (度)"""
        spraying_cfg = self.config_data.get("spraying", {})
        opt_cfg = self.config_data.get("optimization", {})
        val = spraying_cfg.get("grid_tol_z_deg") or opt_cfg.get("grid_tol_z_deg") or [-180.0, 180.0, 10.0]
        return tuple(float(v) for v in val)

    def get_optimization_config(self) -> dict:
        """获取轨迹优化与 POI 容差字典"""
        return {
            "poi_tolerance_rpy_deg": self.poi_tolerance_rpy_deg,
            "grid_tol_x_deg": self.grid_tol_x_deg,
            "grid_tol_y_deg": self.grid_tol_y_deg,
            "grid_tol_z_deg": self.grid_tol_z_deg,
        }


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


def get_configured_optimization_config(config_path: str = None) -> dict:
    """统一从全局配置获取轨迹优化与 POI 容差配置字典。"""
    cfg = config if config_path is None else SprayerConfig(config_path=config_path)
    return cfg.get_optimization_config()

