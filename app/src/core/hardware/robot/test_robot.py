#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import argparse
import logging
import math
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))

from core.hardware.robot.factory import get_robot
from core.hardware.robot.base_driver import RobotPose


#  折叠姿态[0,0,-156,0,0,0]
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

def test_robot(robot_type: str, ip: str, port: str):
    logger.info(f"=== Starting tests for robot type: {robot_type} ===")
    
    # 1. Instantiate the robot
    robot = get_robot(robot_type, ip, port)
    if not robot:
        logger.error("Failed to instantiate robot driver.")
        return

    # 2. startup()
    logger.info("Testing: startup()")
    success = robot.startup(timeout=10.0)
    logger.info(f"Result: {success}")
    if not success:
        logger.error("Startup failed, aborting tests.")
        return
    time.sleep(2)

    # 3. get_running_state()
    logger.info("Testing: get_running_state()")
    state = robot.get_running_state()
    logger.info(f"Result: {state}")
    time.sleep(2)

    # 4. is_robot_idle()
    logger.info("Testing: is_robot_idle()")
    idle = robot.is_robot_idle()
    logger.info(f"Result: {idle}")
    time.sleep(2)

    # 5. set_global_speed()
    logger.info("Testing: set_global_speed()")
    res = robot.set_global_speed(30)
    logger.info(f"Result: {res}")
    if not res:
        logger.error("set_global_speed failed, aborting tests.")
        return
    time.sleep(2)

    # 6. set_tool_number()
    logger.info("Testing: set_tool_number()")
    res = robot.set_tool_number(0)
    logger.info(f"Result: {res}")
    if not res:
        logger.error("set_tool_number failed, aborting tests.")
        return
    time.sleep(2)

    # 7. go_home() (Go to home before moving)
    logger.info("Testing: go_home()")
    res = robot.go_home(wait=True)
    logger.info(f"Result: {res}")
    if res != 0:
        logger.error(f"go_home failed with code {res}, aborting tests.")
        return
    time.sleep(2)

    # 8. get_current_pose()
    logger.info("Testing: get_current_pose()")
    current_pose = robot.get_current_pose()
    logger.info(f"Result: {current_pose}")
    if current_pose is None:
        logger.error("get_current_pose failed, aborting tests.")
        return
    time.sleep(2)

    # Create a small offset pose for movement tests
    target_pose = RobotPose(
        x=current_pose.x, 
        y=current_pose.y, 
        z=current_pose.z + 20.0, # Move 20mm up safely
        a=current_pose.a, 
        b=current_pose.b, 
        c=current_pose.c
    )

    # 9. is_reachable()
    logger.info("Testing: is_reachable()")
    reachable = robot.is_reachable(target_pose, "MOVJ")
    logger.info(f"Result: {reachable}")
    if not reachable:
        logger.error("Target pose is not reachable, aborting movement tests.")
        return
    time.sleep(2)

    # 10. move_j()
    logger.info("Testing: move_j()")
    res = robot.move_j(target_pose, wait=True)
    logger.info(f"Result: {res}")
    if res != 0:
        logger.error(f"move_j failed with code {res}, aborting tests.")
        return
    time.sleep(2)

    # 11. move_l()
    # Move back to original pose
    logger.info("Testing: move_l()")
    res = robot.move_l(current_pose, wait=True)
    logger.info(f"Result: {res}")
    if res != 0:
        logger.error(f"move_l failed with code {res}, aborting tests.")
        return
    time.sleep(2)

    # 12. move_joint()
    logger.info("Testing: move_joint()")
    if robot_type.lower() == "dobot":
        test_joints = [0, 0, -90, 90, 90, 0]
    else:
        test_joints = [0, 0, 0, 0, 0, 0]
    res = robot.move_joint(test_joints, wait=True)
    logger.info(f"Result: {res}")
    if res != 0:
        logger.error(f"move_joint failed with code {res}, aborting tests.")
        return
    time.sleep(2)

    # 13. shutdown()
    logger.info("Testing: shutdown()")
    robot.shutdown()
    logger.info("Result: completed")
    
    logger.info("=== Tests completed successfully! ===")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test robot driver interfaces.")
    parser.add_argument("--robot", type=str, default="dobot", help="Robot type to test: 'dobot' or 'inexbot' (default: dobot)")
    parser.add_argument("--ip", type=str, default="192.168.5.1", help="Robot IP address")
    parser.add_argument("--port", type=str, default="29999", help="Robot port")
    args = parser.parse_args()

    # If the user asks for inexbot but leaves IP as default for dobot, switch IP to inexbot's default
    if args.robot.lower() == "inexbot" and args.ip == "192.168.5.1":
        args.ip = "192.168.2.14"
        args.port = "6001"

    test_robot(args.robot, args.ip, args.port)
