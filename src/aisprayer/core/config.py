import os
import yaml

# 项目根目录 (SprayAnything/)
# 当前文件位于 src/aisprayer/core/config.py，所以向上三层
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

class SprayerConfig:
    """
    统一读取和解析 AiSprayer 系统配置 (aisprayer_config.yaml) 及相关引用的配置文件。
    """
    def __init__(self, config_path="configs/aisprayer_config.yaml"):
        self.config_path = self._resolve_path(config_path)
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
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

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
    def urdf_path(self):
        """机器人 URDF 模型文件路径"""
        path = self.config_data.get("hardware", {}).get("robot", {}).get("robot_urdf")
        return self._resolve_path(path)

    @property
    def robot_ip(self):
        return self.config_data.get("hardware", {}).get("robot", {}).get("ip")

    @property
    def robot_port(self):
        return self.config_data.get("hardware", {}).get("robot", {}).get("port", 6001)

    @property
    def camera_model(self):
        return self.config_data.get("hardware", {}).get("camera", {}).get("model", "orbbec")
