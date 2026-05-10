import argparse
import logging
import os
import sys

# 1. 路径锚定策略 (与 capture_tool 保持一致)
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
SRC_PATH = os.path.join(PROJECT_ROOT, "src")
if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)

from aisprayer.core.controller.robot_trajectory_controller import RobotTrajectoryController
from aisprayer.utils.config_helper import load_config

logger = logging.getLogger(__name__)

def find_latest_plan():
    """
    自动寻找最新的规划文件: data/runs/<latest_trouser>/<latest_view>/plan.yaml
    """
    runs_dir = os.path.join(PROJECT_ROOT, "data/runs")
    if not os.path.exists(runs_dir):
        return None
    
    # 1. 找最新的裤子目录
    trousers = sorted([d for d in os.listdir(runs_dir) if os.path.isdir(os.path.join(runs_dir, d))])
    if not trousers: return None
    latest_trouser = os.path.join(runs_dir, trousers[-1])
    
    # 2. 找最新的视角目录 (数字命名的目录)
    views = sorted([d for d in os.listdir(latest_trouser) if d.isdigit() and os.path.isdir(os.path.join(latest_trouser, d))], key=int)
    if not views: return None
    latest_view = os.path.join(latest_trouser, views[-1])
    
    plan_path = os.path.join(latest_view, "plan.yaml")
    return plan_path if os.path.exists(plan_path) else None

def main():
    parser = argparse.ArgumentParser(description="AiSprayer 机器人轨迹运行工具")
    parser.add_argument("--config", type=str, default=os.path.join(PROJECT_ROOT, "configs/aisprayer_config.yaml"), help="配置文件路径")
    parser.add_argument("--plan", type=str, help="手动指定 plan.yaml 的路径 (如果不指定则自动寻找最新)")
    
    args = parser.parse_args()

    # 1. 配置日志: 使用自定义格式实现 I/E 等级名和短模块名
    for level, short in {logging.INFO: "I", logging.ERROR: "E", logging.WARNING: "W", logging.DEBUG: "D"}.items():
        logging.addLevelName(level, short)

    class CustomFormatter(logging.Formatter):
        def format(self, record):
            # 简化模块名: 只取最后一部分
            if "." in record.name:
                record.name = record.name.split(".")[-1]
            return super().format(record)

    handler = logging.StreamHandler()
    handler.setFormatter(CustomFormatter(
        fmt='%(asctime)s [%(levelname)s] [%(name)s:%(funcName)s] %(message)s',
        datefmt='%H:%M:%S'
    ))
    
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.addHandler(handler)

    # 2. 加载配置
    cfg = load_config(args.config, PROJECT_ROOT)
    robot_cfg = cfg.get("hardware", {}).get("robot", {})
    spray_cfg = cfg.get("spraying", {})

    ip = robot_cfg.get("ip", "192.168.2.14")
    port = robot_cfg.get("port", "6001")
    
    ready_dist = spray_cfg.get("ready_dist_mm", 300.0)
    velocity = spray_cfg.get("velocity", 150.0)
    pl = spray_cfg.get("pl", 0)
    tool_num = spray_cfg.get("tool_num", 0)

    # 3. 确定规划文件
    plan_path = args.plan if args.plan else find_latest_plan()
    if not plan_path:
        logger.error("未找到有效的规划文件 (plan.yaml)，请检查 data/runs 目录或手动指定 --plan")
        return

    logger.info(f"[*] 准备执行轨迹。规划文件: {plan_path}")
    logger.info(f"[*] 运行参数: IP={ip}, Tool={tool_num}, Speed={velocity}, PL={pl}, ReadyDist={ready_dist}")

    # 4. 执行控制
    ctrl = RobotTrajectoryController(
        ip=ip, 
        port=port, 
        ready_dist_mm=ready_dist, 
        tool_num=tool_num
    )

    try:
        # 启动机器人并归位
        if ctrl.startup():
            # 移动到避让位准备作业
            ctrl.go_avoidance()
            
            # 执行轨迹任务
            ctrl.execute(plan_path, velocity=velocity, pl=pl)
            
            # 任务结束回到避让位
            ctrl.go_avoidance()
            
    except KeyboardInterrupt:
        logger.info("用户中断操作")
    finally:
        ctrl.shutdown()

if __name__ == "__main__":
    main()
