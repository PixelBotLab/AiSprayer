import os
import yaml

def load_config(config_path=None, project_root=None):
    """
    加载统一配置文件并自动解析路径。
    :param config_path: 配置文件路径，如果为 None 则使用默认路径
    :param project_root: 项目根目录，用于解析相对路径
    :return: 配置字典
    """
    if project_root is None:
        # 假设本文件位于 src/aisprayer/utils/，向上 3 级到达根目录
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))

    if config_path is None:
        config_path = os.path.join(project_root, "configs/aisprayer_config.yaml")
    
    if not os.path.exists(config_path):
        print(f"[!] Warning: 配置文件不存在: {config_path}, 将使用空配置")
        return {}

    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    return config

def get_abs_path(rel_path, project_root):
    """辅助函数：将相对路径转为绝对路径"""
    if rel_path is None: return None
    if os.path.isabs(rel_path): return rel_path
    return os.path.abspath(os.path.join(project_root, rel_path))
