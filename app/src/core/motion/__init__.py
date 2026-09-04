"""C++ motion 模块：路径验证 CLI + 实时 FK/IK（libmotion_c）。"""

from .cli_client import MotionCliError, optimize_path, run_motion_cli, verify_path
from .kinematics import CR5Kinematics

__all__ = [
    "CR5Kinematics",
    "MotionCliError",
    "optimize_path",
    "run_motion_cli",
    "verify_path",
]
