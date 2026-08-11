import logging
from typing import Optional

from .base_driver import BaseRobotDriver

logger = logging.getLogger(__name__)

def get_robot(robot_type: str, ip: str, port: str, **kwargs) -> Optional[BaseRobotDriver]:
    """
    根据给定的 robot_type 返回对应的机械臂驱动实例。
    :param robot_type: 机器人类型，如 "inexbot", "dobot"
    :param ip: 机器人 IP
    :param port: 机器人端口 (部分驱动会有默认端口)
    :param kwargs: 其他初始化参数，如 toolnum, mode 等
    :return: BaseRobotDriver 实例或 None
    """
    robot_type = robot_type.lower().strip()
    if robot_type == "inexbot":
        from .inexbot_driver import InexbotDriver
        return InexbotDriver(ip=ip, port=port, **kwargs)
    elif robot_type == "dobot":
        from .dobot_driver import DobotDriver
        # dobot typically uses port 29999 for dashboard and 30003 for move,
        # but we can try to parse the port or use defaults.
        dashboard_port = int(port) if str(port).isdigit() else 29999
        return DobotDriver(ip=ip, dashboard_port=dashboard_port, **kwargs)
    else:
        logger.error(f"Unknown robot type: {robot_type}")
        return None
