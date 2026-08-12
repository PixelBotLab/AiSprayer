import sys
import os
import time
import threading
import logging
import math
from typing import Optional, List, Callable

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from core.hardware.robot.factory import get_robot
from core.hardware.robot.base_driver import BaseRobotDriver

logger = logging.getLogger(__name__)

def _fmt(arr: Optional[List[float]]) -> str:
    if arr is None:
        return "None"
    return "[" + ", ".join(f"{x:.2f}" for x in arr) + "]"

class RobotService:
    def __init__(self):
        self._driver: Optional[BaseRobotDriver] = None
        self._is_connected = False
        self._polling_thread = None
        self._stop_polling = False
        self._ws_callbacks: List[Callable] = []
        self._global_speed = 20.0
        self._global_acc = 20.0

    def connect(self, robot_type: str, ip: str, port: str, **kwargs) -> tuple[bool, str]:
        if self._is_connected:
            self.disconnect()
            
        try:
            logger.info(f"Connecting to robot {robot_type} at {ip}:{port}...")
            self._driver = get_robot(robot_type, ip, port, **kwargs)
            if self._driver and self._driver.startup():
                self._is_connected = True
                logger.info("Robot connected successfully.")
                self.start_status_polling()
                return True, ""
            else:
                msg = f"Failed to initialize or start robot driver for {robot_type} at {ip}:{port}."
                logger.error(msg)
                self._is_connected = False
                self._driver = None
                return False, msg
        except Exception as e:
            msg = f"Failed to connect robot {robot_type} at {ip}:{port}: {e}"
            logger.error(msg)
            self._is_connected = False
            self._driver = None
            return False, msg

    def disconnect(self) -> tuple[bool, str]:
        logger.info("Disconnecting robot...")
        try:
            self.stop_status_polling()
            if self._driver and self._is_connected:
                self._driver.shutdown()
            self._is_connected = False
            self._driver = None
            return True, ""
        except Exception as e:
            msg = f"Error disconnecting robot: {e}"
            logger.error(msg)
            self._is_connected = False
            self._driver = None
            return False, msg

    def is_connected(self) -> bool:
        return self._is_connected

    def get_speed(self) -> tuple[float, float, float, float]:
        if self._driver and self._is_connected and hasattr(self._driver, 'get_global_speed'):
            spd_l, acc_l, spd_j, acc_j = self._driver.get_global_speed()
            logger.info(f"get_speed: spd_l:{spd_l}, acc_l:{acc_l}, spd_j:{spd_j}, acc_j:{acc_j}")
            # update internal cache to match real robot state
            self._global_speed = spd_l
            self._global_acc = acc_l
            return spd_l, acc_l, spd_j, acc_j
        return self._global_speed, self._global_acc, self._global_speed, self._global_acc

    def set_speed(self, speed_l: float, acc_l: float, speed_j: float, acc_j: float) -> tuple[bool, str]:
        self._global_speed = speed_l
        self._global_acc = acc_l
        # Pass to driver if connected
        if self._driver and self._is_connected:
            if hasattr(self._driver, 'dashboard') and self._driver.dashboard:
                try:
                    logger.info(f"set_speed: speed_l:{speed_l}, acc_l:{acc_l}, speed_j:{speed_j}, acc_j:{acc_j}")
                    self._driver.dashboard.SpeedL(int(speed_l))
                    self._driver.dashboard.AccL(int(acc_l))
                    self._driver.dashboard.SpeedJ(int(speed_j))
                    self._driver.dashboard.AccJ(int(acc_j))
                except Exception as e:
                    msg = f"Failed to set speed on robot: {e}"
                    logger.error(msg)
                    return False, msg
        return True, ""

    def get_current_pose(self) -> tuple[Optional[List[float]], str]:
        if not self._driver or not self._is_connected:
            return None, "Robot is not connected"
        try:
            pose_obj = self._driver.get_current_pose()
            if pose_obj:
                return pose_obj.to_list(), ""
            return None, "Failed to read pose from driver"
        except Exception as e:
            msg = f"Error getting pose: {e}"
            logger.error(msg)
            return None, msg

    def get_current_joint(self) -> tuple[Optional[List[float]], str]:
        if not self._driver or not self._is_connected:
            return None, "Robot is not connected"
        try:
            joints = self._driver.get_current_joint()
            if joints:
                return joints, ""
            return None, "Failed to read joints from driver"
        except Exception as e:
            msg = f"Error getting joint: {e}"
            logger.error(msg)
            return None, msg

    def move_to_pose(self, pose: List[float], speed: float = 50.0, acc: float = 50.0) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            msg = "move_to_pose: Robot is not connected"
            logger.error(msg)
            return False, msg

        pose_val, _ = self.get_current_pose()
        logger.info(f"move_to_pose: from pose:{_fmt(pose_val)}, to pose:{_fmt(pose)}, speed:{speed:.2f}, acc:{acc:.2f}")
        try:
            self._driver.move_l(pose, velocity=speed, acc=acc)
            new_pose_val, _ = self.get_current_pose()
            logger.info(f"move_to_pose: Success moving to pose, actual pose:{_fmt(new_pose_val)}")
            return True, ""
        except Exception as e:
            msg = f"move_to_pose: Error moving to pose: {e}"
            logger.error(msg)
            return False, msg

    def move_to_joint(self, joints: List[float], speed: float = 50.0, acc: float = 50.0) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            msg = "move_to_joint: Robot is not connected"
            logger.error(msg)
            return False, msg

        joint_val, _ = self.get_current_joint()
        logger.info(f"move_to_joint: from joints:{_fmt(joint_val)}, to joints:{_fmt(joints)}, speed:{speed:.2f}, acc:{acc:.2f}")
        try:
            self._driver.move_joint(joints, velocity=speed, acc=acc)
            new_joint_val, _ = self.get_current_joint()
            logger.info(f"move_to_joint: Success moving to joint, actual joints:{_fmt(new_joint_val)}")
            return True, ""
        except Exception as e:
            msg = f"move_to_joint: Error moving to joint: {e}"
            logger.error(msg)
            return False, msg

    def jog_step(self, axis: str, direction: int, step_size: float = 1.0, speed: float = 20.0, acc: float = 20.0) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            msg = "jog_step: Robot is not connected"
            logger.error(msg)
            return False, msg

        cartesian_map = {'X': 0, 'Y': 1, 'Z': 2, 'Rx': 3, 'Ry': 4, 'Rz': 5}
        joint_map = {'J1': 0, 'J2': 1, 'J3': 2, 'J4': 3, 'J5': 4, 'J6': 5}

        logger.info(f"jog_step: axis:{axis}, direction:{direction}, step_size:{step_size:.2f}, speed:{speed:.2f}, acc:{acc:.2f}")

        if axis in cartesian_map:
            pose, err_msg = self.get_current_pose()
            if not pose or len(pose) < 6:
                msg = f"jog_step: Pose is not valid ({err_msg})"
                logger.error(msg)
                return False, msg
            
            pose_list = list(pose)
            idx = cartesian_map[axis]
            # Convert degree step to radian if rotational
            step = step_size
            if idx >= 3:
                import math
                step = math.radians(step_size)
            
            pose_list[idx] += direction * step
            return self.move_to_pose(pose_list, speed=speed, acc=acc)

        elif axis in joint_map:
            joints, err_msg = self.get_current_joint()
            if not joints or len(joints) < 6:
                msg = f"jog_step: Joints is not valid ({err_msg})"
                logger.error(msg)
                return False, msg
            idx = joint_map[axis]
            joints[idx] += direction * step_size
            return self.move_to_joint(joints, speed=speed, acc=acc)
            
        return False, f"jog_step: Invalid axis {axis}"

    def go_zero(self, speed: float = 20.0, acc: float = 20.0) -> tuple[bool, str]:
        logger.info(f"go_zero: speed:{speed}, acc:{acc}")
        if not self._driver or not self._is_connected:
            msg = "go_zero: Robot is not connected"
            logger.error(msg)
            return False, msg
            
        try:
            return self.move_to_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed=speed, acc=acc)
        except Exception as e:
            msg = f"Error going zero: {e}"
            logger.error(msg)
            return False, msg

    def go_home(self, speed: float = 20.0, acc: float = 20.0) -> tuple[bool, str]:
        logger.info(f"go_home: speed:{speed}, acc:{acc}")
        if not self._driver or not self._is_connected:
            msg = "go_home: Robot is not connected"
            logger.error(msg)
            return False, msg
            
        try:
            res = self._driver.go_home(wait=True, velocity=speed)
            if res == 0:
                return True, ""
            return False, f"go_home returned error code: {res}"
        except Exception as e:
            msg = f"Error going home: {e}"
            logger.error(msg)
            return False, msg

    def register_ws_callback(self, callback: Callable):
        if callback not in self._ws_callbacks:
            self._ws_callbacks.append(callback)

    def unregister_ws_callback(self, callback: Callable):
        if callback in self._ws_callbacks:
            self._ws_callbacks.remove(callback)

    def start_status_polling(self, interval: float = 0.02):
        if self._polling_thread and self._polling_thread.is_alive():
            logger.info("Status polling thread is already running.")
            return
        logger.info("Starting status polling...")
        self._stop_polling = False
        self._polling_thread = threading.Thread(target=self._poll_loop, args=(interval,), daemon=True)
        self._polling_thread.start()
        logger.info("Status polling started.")

    def stop_status_polling(self):
        logger.info("Stopping status polling...")
        self._stop_polling = True
        if self._polling_thread:
            self._polling_thread.join(timeout=1.0)
            self._polling_thread = None
        logger.info("Status polling stopped.")

    def _poll_loop(self, interval: float):
        last_status = None
        while not self._stop_polling and self._is_connected:
            pose, _ = self.get_current_pose()
            joints, _ = self.get_current_joint()
            status = self._driver.get_running_state() if self._driver else 0
            
            if status != last_status:
                logger.info(f"Robot status changed: {status}")
                last_status = status
                
            if pose and joints:
                for cb in self._ws_callbacks:
                    try:
                        cb({"type": "robot_state", "data": {"pose": pose, "joint": joints, "status": status}})
                    except:
                        pass
            time.sleep(interval)


    def pause(self) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            return False, "Robot is not connected"
        success = self._driver.pause()
        return success, "" if success else "Failed to pause robot"

    def resume(self) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            return False, "Robot is not connected"
        success = self._driver.resume()
        return success, "" if success else "Failed to resume robot"

    def estop(self) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            return False, "Robot is not connected"
        success = self._driver.estop()
        return success, "" if success else "Failed to estop robot"

robot_service = RobotService()
