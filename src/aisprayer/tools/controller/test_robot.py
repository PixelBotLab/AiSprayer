"""
InexbotDriver2 集成测试脚本
针对 aisprayer.core.hardware.robot.inexbot_driver2.py 中的 InexbotDriver 进行全功能覆盖测试。

注意：此脚本会控制机器人进行实际运动，请确保：
1. 机器人周围无障碍物。
2. 已经连接好机器人，且 IP 地址正确。
3. 处于安全距离。
"""

import time
import math
import logging
import sys
import os

# 1. 路径锚定策略
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.hardware.robot.inexbot_driver2 import InexbotDriver, RobotPose

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("TestRobot")

def test_inexbot_driver(ip="192.168.2.14"):
    logger.info(f"开始测试 InexbotDriver, 目标 IP: {ip}")
    
    # 1. 初始化驱动
    robot = InexbotDriver(ip=ip)
    
    try:
        # 2. 启动并上电 (覆盖 startup, print_system_info)
        logger.info(">>> 步骤 1: 启动与上电测试 (startup)")
        if not robot.startup():
            logger.error("机器人启动失败")
            return
        logger.info("机器人启动成功")

        # 3. 状态查询测试 (覆盖 get_servo_state, get_running_state, get_current_pose)
        logger.info(">>> 步骤 2: 状态查询测试")
        servo_state = robot.get_servo_state()
        running_state = robot.get_running_state()
        current_pose = robot.get_current_pose()
        
        logger.info(f"伺服状态: {servo_state}")
        logger.info(f"运行状态: {running_state}")
        logger.info(f"当前位姿: {current_pose}")
        
        assert servo_state != -1, "获取伺服状态失败"
        assert running_state != -1, "获取运行状态失败"
        assert current_pose is not None, "获取当前位姿失败"

        # 4. 可达性测试 (覆盖 is_reachable)
        logger.info(">>> 步骤 3: 可达性测试 (is_reachable)")
        test_pose = RobotPose(x=current_pose.x + 10, y=current_pose.y, z=current_pose.z, 
                              a=current_pose.a, b=current_pose.b, c=current_pose.c)
        reachable = robot.is_reachable(test_pose)
        logger.info(f"位姿 {test_pose} 可达性: {reachable}")

        # 5. 点对点运动测试 (覆盖 move_j, move_l, go_home)
        logger.info(">>> 步骤 4: 点对点运动测试 (move_j, move_l, go_home)")
        
        # 稍微偏移一下
        target_1 = RobotPose(x=current_pose.x, y=current_pose.y + 20, z=current_pose.z, 
                             a=current_pose.a, b=current_pose.b, c=current_pose.c)
        if robot.is_reachable(target_1):
            logger.info(f"执行 move_j 到 {target_1}")
            robot.move_j(target_1, velocity=20)
            
            logger.info(f"执行 move_l 回到初始位置")
            robot.move_l(current_pose, velocity=50)
        else:
            logger.warning("目标位姿 1 不可达，跳过运动测试")

        logger.info("执行 go_home")
        robot.go_home()
        home_pose = robot.get_current_pose()
        logger.info(f"回到 Home 后的位姿: {home_pose}")

        # 6. 队列运动测试 (覆盖 queue_start, queue_push_l, queue_push_j, queue_send, queue_get_remaining, queue_suspend, queue_resume, queue_stop)
        logger.info(">>> 步骤 5: 队列运动测试")
        
        logger.info("启动队列模式")
        robot.queue_start()
        
        # 构造几个点位形成矩形或三角形运动
        p1 = RobotPose(x=home_pose.x + 20, y=home_pose.y, z=home_pose.z, a=home_pose.a, b=home_pose.b, c=home_pose.c)
        p2 = RobotPose(x=home_pose.x + 20, y=home_pose.y + 20, z=home_pose.z, a=home_pose.a, b=home_pose.b, c=home_pose.c)
        p3 = RobotPose(x=home_pose.x, y=home_pose.y + 20, z=home_pose.z, a=home_pose.a, b=home_pose.b, c=home_pose.c)
        
        if all(robot.is_reachable(p) for p in [p1, p2, p3]):
            logger.info("添加队列指令 (move_j -> move_l -> move_l -> move_l)")
            robot.queue_push_j(p1, velocity=20)
            robot.queue_push_l(p2, velocity=100)
            robot.queue_push_l(p3, velocity=100)
            robot.queue_push_l(home_pose, velocity=100)
            
            rem = robot.queue_get_remaining()
            logger.info(f"控制器端待执行队列长度 (发送前): {rem}")
            
            logger.info("下发并执行队列 (wait=False)")
            robot.queue_send(wait=False)
            
            time.sleep(0.5)
            rem = robot.queue_get_remaining()
            logger.info(f"运动中控制器端剩余指令: {rem}")
            
            logger.info("测试暂停 (queue_suspend)")
            robot.queue_suspend()
            time.sleep(1)
            
            logger.info("测试恢复 (queue_resume)")
            robot.queue_resume()
            
            logger.info("测试停止并清空 (queue_stop)")
            robot.queue_stop()
            
            logger.info("重新执行完整队列并等待完成")
            robot.queue_start()
            robot.queue_push_j(p1, velocity=20)
            robot.queue_push_l(p2, velocity=100, pl=1) # 带平滑
            robot.queue_push_l(p3, velocity=100, pl=1)
            robot.queue_push_l(home_pose, velocity=100)
            robot.queue_send(wait=True)
            logger.info("队列运动执行完毕")
        else:
            logger.warning("队列测试中的部分点位不可达，跳过")

    except Exception as e:
        logger.exception(f"测试过程中出现异常: {e}")
    finally:
        # 7. 关闭驱动 (覆盖 shutdown)
        logger.info(">>> 步骤 6: 关闭驱动 (shutdown)")
        robot.shutdown()
        logger.info("测试结束")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="InexbotDriver2 全功能测试")
    parser.add_argument("--ip", type=str, default="192.168.2.14", help="机器人 IP")
    args = parser.parse_args()
    
    test_inexbot_driver(args.ip)
