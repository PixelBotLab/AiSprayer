# -*- coding: utf-8 -*-
import logging
import math
import os
import sys
import threading
import time
from typing import Any, Callable, List, Optional, Union

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from core.config import SprayerConfig
from core.hardware.robot.base_driver import BaseRobotDriver, PoseLike, RobotPose
from core.hardware.robot.factory import get_robot

logger = logging.getLogger(__name__)


def _fmt(arr: Optional[List[float]]) -> str:
    if arr is None:
        return "None"
    return "[" + ", ".join(f"{x:.2f}" for x in arr) + "]"


class RobotService:
    def __init__(self, config: Optional[SprayerConfig] = None):
        self._config = config or SprayerConfig()
        self._driver: Optional[BaseRobotDriver] = None
        self._is_connected = False
        self._polling_thread = None
        self._stop_polling = False
        self._ws_callbacks: List[Callable] = []

        # 从统一配置文件读取速度与限位参数
        self._global_speed_factor: int = self._config.global_speed_factor
        self._max_tcp_speed_mm_s: float = self._config.max_tcp_speed_mm_s
        self._max_joint_speed_deg_s: List[float] = self._config.max_joint_speed_deg_s

        # 初始具体速度值 (mm/s, %, deg/s, %)
        eff_max_tcp = self._max_tcp_speed_mm_s * (self._global_speed_factor / 100.0)
        eff_max_jnt = (self._max_joint_speed_deg_s[0] if self._max_joint_speed_deg_s else 180.0) * (self._global_speed_factor / 100.0)
        self._speed_l: float = min(eff_max_tcp, 100.0)
        self._acc_l: float = 20.0
        self._speed_j: float = min(eff_max_jnt, 20.0)
        self._acc_j: float = 20.0

    @property
    def config(self) -> SprayerConfig:
        return self._config

    @property
    def is_connected_prop(self) -> bool:
        return self._is_connected

    @property
    def global_speed_factor(self) -> int:
        return self._global_speed_factor

    def set_global_speed_factor(self, factor: int) -> tuple[bool, str]:
        if not (1 <= factor <= 100):
            return False, "Global speed factor must be between 1 and 100"

        self._global_speed_factor = factor

        # Apply to connected robot if available
        if self._is_connected and self._driver and hasattr(self._driver, "set_global_speed"):
            try:
                self._driver.set_global_speed(factor)
                logger.info(f"Dynamically updated robot set_global_speed to {factor}")
            except Exception as e:
                logger.warning(f"Failed to dynamically set global speed: {e}")

        # Re-calculate effective max speeds based on the new global_speed_factor
        eff_max_tcp = self._max_tcp_speed_mm_s * (self._global_speed_factor / 100.0)
        eff_max_jnt = (self._max_joint_speed_deg_s[0] if self._max_joint_speed_deg_s else 180.0) * (self._global_speed_factor / 100.0)
        if self._speed_l > eff_max_tcp:
            self._speed_l = eff_max_tcp
        if self._speed_j > eff_max_jnt:
            self._speed_j = eff_max_jnt

        return True, "Success"

    @property
    def max_tcp_speed_mm_s(self) -> float:
        return self._max_tcp_speed_mm_s

    @property
    def max_joint_speed_deg_s(self) -> List[float]:
        return self._max_joint_speed_deg_s

    def reload_config(self, config_path: str = "configs/aisprayer_config.yaml"):
        """重新加载配置文件并更新速度与限位参数"""
        self._config = SprayerConfig(config_path)
        self._global_speed_factor = self._config.global_speed_factor
        self._max_tcp_speed_mm_s = self._config.max_tcp_speed_mm_s
        self._max_joint_speed_deg_s = self._config.max_joint_speed_deg_s
        eff_max_tcp = self._max_tcp_speed_mm_s * (self._global_speed_factor / 100.0)
        eff_max_jnt = (self._max_joint_speed_deg_s[0] if self._max_joint_speed_deg_s else 180.0) * (self._global_speed_factor / 100.0)
        self._speed_l = min(eff_max_tcp, max(1.0, self._speed_l))
        self._speed_j = min(eff_max_jnt, max(1.0, self._speed_j))
        logger.info(
            f"RobotService config reloaded: global_speed_factor={self._global_speed_factor}, "
            f"max_tcp_speed_mm_s={self._max_tcp_speed_mm_s}, max_joint_speed_deg_s={self._max_joint_speed_deg_s}"
        )

    def connect(self, robot_type: str, ip: str, port: str, **kwargs) -> tuple[bool, str]:
        if self._is_connected:
            self.disconnect()

        try:
            logger.info(f"Connecting to robot {robot_type} at {ip}:{port}...")
            self._driver = get_robot(robot_type, ip, port, **kwargs)
            if self._driver and self._driver.startup():
                self._is_connected = True
                time.sleep(1)

                self.start_status_polling()

                # Enforce global_speed_factor speed and configured tool on real robot upon connection
                try:
                    tool_id = self._config.robot_tcp_id
                    if hasattr(self._driver, "set_tool_number"):
                        self._driver.set_tool_number(tool_id)
                        logger.info(f"robot {robot_type} initialized tool to {tool_id} (target_tcp={self._config.robot_tcp})")
                    if hasattr(self._driver, "set_global_speed"):
                        self._driver.set_global_speed(self._global_speed_factor)
                        logger.info(f"robot {robot_type} set_global_speed to {self._global_speed_factor}")
                except Exception as e:
                    logger.warning(f"Failed to set initial speed/tool after connect: {e}")

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
        return self._speed_l, self._acc_l, self._speed_j, self._acc_j

    def set_speed(self, speed_l: float, acc_l: float, speed_j: float, acc_j: float) -> tuple[bool, str]:
        if speed_l <= 0 or acc_l <= 0 or speed_j <= 0 or acc_j <= 0:
            return False, "Speed and acceleration must be greater than 0"
        self._speed_l = float(speed_l)
        self._acc_l = float(acc_l)
        self._speed_j = float(speed_j)
        self._acc_j = float(acc_j)
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

    def set_tool(self, tool_num: int) -> tuple[bool, str]:
        """
        设置机械臂当前工具坐标系编号 (例如 0=默认法兰, 1=gripper_tip_link, 2=laser_head_link)。
        """
        if not self._driver or not self._is_connected:
            msg = "set_tool: Robot is not connected"
            logger.error(msg)
            return False, msg
        try:
            if hasattr(self._driver, "set_tool_number"):
                ok = self._driver.set_tool_number(tool_num)
                if ok:
                    logger.info(f"set_tool: Successfully set robot tool to {tool_num}")
                    return True, ""
                return False, f"Failed to set robot tool to {tool_num}"
            return True, ""
        except Exception as e:
            msg = f"set_tool: Error setting robot tool to {tool_num}: {e}"
            logger.error(msg)
            return False, msg

    def set_tool_number(self, tool_num: int) -> tuple[bool, str]:
        return self.set_tool(tool_num)

    @property
    def do_index(self) -> int:
        """从配置中获取喷涂/触发 DO 编号 (默认 1)"""
        if self._config:
            return getattr(self._config, "robot_do_index", 1)
        return 1

    def set_do(self, index: Optional[int] = None, status: int = 1) -> tuple[bool, str]:
        """
        设置机械臂数字输出端口 (DO) 状态。
        :param index: DO 端口编号 (1-based, 例如 1 代表 DO1; 若为 None 则从配置中读取)
        :param status: 1 代表高电平(开)，0 代表低电平(关)
        """
        eff_index = index if index is not None else self.do_index
        if not self._driver or not self._is_connected:
            msg = "set_do: Robot is not connected"
            logger.error(msg)
            return False, msg
        try:
            if hasattr(self._driver, "set_do"):
                ok = self._driver.set_do(eff_index, status)
                if ok:
                    logger.info(f"set_do: Successfully set DO({eff_index}, {status})")
                    return True, ""
                return False, f"Failed to set DO({eff_index}, {status})"
            return False, "Driver does not support set_do"
        except Exception as e:
            msg = f"set_do: Error setting DO({eff_index}, {status}): {e}"
            logger.error(msg)
            return False, msg

    def get_do(self, index: Optional[int] = None) -> tuple[Optional[int], str]:
        """获取机械臂数字输出端口 (DO) 状态 (若未传入 index 则从配置中读取)"""
        eff_index = index if index is not None else self.do_index
        if not self._driver or not self._is_connected:
            return None, "Robot is not connected"
        try:
            if hasattr(self._driver, "get_do"):
                val = self._driver.get_do(eff_index)
                return val, ""
            return None, "Driver does not support get_do"
        except Exception as e:
            return None, str(e)



    def move_to_pose(self, pose: List[float], speed: float = 10.0, acc: float = 10.0, tool_num: Optional[int] = None) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            msg = "move_to_pose: Robot is not connected"
            logger.error(msg)
            return False, msg

        eff_tool = tool_num if tool_num is not None else self._config.robot_tcp_id
        pose_val, _ = self.get_current_pose()
        logger.info(f"move_to_pose: from pose:{_fmt(pose_val)}, to pose:{_fmt(pose)}, speed:{speed:.2f}, acc:{acc:.2f}, tool:{eff_tool}")
        try:
            self._driver.move_l(pose, velocity_mm=speed, acc=acc, tool_num=eff_tool)
            new_pose_val, _ = self.get_current_pose()
            logger.info(f"move_to_pose: Success moving to pose, actual pose:{_fmt(new_pose_val)}")
            return True, ""
        except Exception as e:
            msg = f"move_to_pose: Error moving to pose: {e}"
            logger.error(msg)
            return False, msg

    def move_to_pose_j(self, pose: List[float], speed: float = 10.0, acc: float = 10.0, tool_num: Optional[int] = None) -> tuple[bool, str]:
        """使用 MovJ 关节插补运动到指定的直角坐标笛卡尔位姿。"""
        if not self._driver or not self._is_connected:
            msg = "move_to_pose_j: Robot is not connected"
            logger.error(msg)
            return False, msg

        eff_tool = tool_num if tool_num is not None else self._config.robot_tcp_id
        # SpeedJ 接收 0-100 的百分比，需要把 deg/s 转换
        max_jnt = self.max_joint_speed_deg_s[0] if self.max_joint_speed_deg_s else 180.0
        ratio_j = max(1, min(100, int((speed / max_jnt) * 100)))

        pose_val, _ = self.get_current_pose()
        logger.info(f"move_to_pose_j (MovJ): from pose:{_fmt(pose_val)}, to pose:{_fmt(pose)}, speed:{speed:.2f}, acc:{acc:.2f}, tool:{eff_tool}")
        try:
            res = self._driver.move_j(pose, velocity=ratio_j, acc=acc, tool_num=eff_tool)
            if res == 0:
                new_pose_val, _ = self.get_current_pose()
                logger.info(f"move_to_pose_j: Success moving to pose via MovJ, actual pose:{_fmt(new_pose_val)}")
                return True, ""
            return False, f"move_j returned error code: {res}"
        except Exception as e:
            msg = f"move_to_pose_j: Error moving to pose via MovJ: {e}"
            logger.error(msg)
            return False, msg

    def move_l_queue(
        self,
        poses: List[Any],
        speed: Optional[float] = None,
        acc: Optional[float] = None,
        cp_ratio: int = 98,
        wait: bool = True,
        tool_num: Optional[int] = None
    ) -> tuple[bool, str]:
        """
        通过驱动层 move_l_queue 批量执行笛卡尔连续轨迹路点 (Waypoints)。
        :param poses: 路点列表，支持 RobotPose, dict (含 x, y, z, rx, ry, rz), 或 list/tuple
        :param speed: 笛卡尔线速度 (mm/s)，为空则使用当前设定值
        :param acc: 加速度百分比 (1~100)
        :param cp_ratio: 平滑过渡比例 (1~100)
        :param wait: 是否等待整条队列执行完毕
        :param tool_num: 工具坐标系编号 (为空则默认使用配置的 robot_tcp_id)
        """
        if not self._driver or not self._is_connected:
            msg = "move_l_queue: Robot is not connected"
            logger.error(msg)
            return False, msg

        if not poses:
            return True, ""

        eff_tool = tool_num if tool_num is not None else self._config.robot_tcp_id
        speed_val = float(speed) if speed is not None else float(self._speed_l)
        acc_val = float(acc) if acc is not None else float(self._acc_l)

        converted_poses: List[RobotPose] = []
        for p in poses:
            if isinstance(p, RobotPose):
                converted_poses.append(p)
            elif isinstance(p, dict):
                x = float(p.get("x", 0.0))
                y = float(p.get("y", 0.0))
                z = float(p.get("z", 0.0))
                rx = float(p.get("rx", p.get("a", 0.0)))
                ry = float(p.get("ry", p.get("b", 0.0)))
                rz = float(p.get("rz", p.get("c", 0.0)))
                if not p.get("is_radians", False):
                    rx = math.radians(rx)
                    ry = math.radians(ry)
                    rz = math.radians(rz)
                converted_poses.append(RobotPose(x, y, z, rx, ry, rz))
            elif isinstance(p, (list, tuple)) and len(p) >= 6:
                converted_poses.append(RobotPose.from_list(list(p)))

        logger.info(f"move_l_queue: executing {len(converted_poses)} waypoints, speed:{speed_val} mm/s, acc:{acc_val}%, cp_ratio:{cp_ratio}, tool:{eff_tool}")
        try:
            res = self._driver.move_l_queue(
                converted_poses,
                velocity_mm=speed_val,
                acc=acc_val,
                wait=wait,
                cp_ratio=cp_ratio,
                tool_num=eff_tool
            )
            if res == 0:
                logger.info(f"move_l_queue: Successfully executed {len(converted_poses)} waypoints (tool={eff_tool}).")
                return True, ""
            return False, f"move_l_queue returned error code: {res}"
        except Exception as e:
            msg = f"move_l_queue error: {e}"
            logger.error(msg)
            return False, msg

    def move_to_joint(self, joints: List[float], speed: float = 10.0, acc: float = 10.0) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            msg = "move_to_joint: Robot is not connected"
            logger.error(msg)
            return False, msg

        # SpeedJ 接收 0-100 的百分比，需要把 deg/s 转换
        max_jnt = self.max_joint_speed_deg_s[0] if self.max_joint_speed_deg_s else 180.0
        ratio_j = max(1, min(100, int((speed / max_jnt) * 100)))

        joint_val, _ = self.get_current_joint()
        logger.info(f"move_to_joint: from joints:{_fmt(joint_val)}, to joints:{_fmt(joints)}, speed:{speed:.2f}, acc:{acc:.2f}")
        try:
            self._driver.move_joint(joints, velocity=ratio_j, acc=acc)
            new_joint_val, _ = self.get_current_joint()
            logger.info(f"move_to_joint: Success moving to joint, actual joints:{_fmt(new_joint_val)}")
            return True, ""
        except Exception as e:
            msg = f"move_to_joint: Error moving to joint: {e}"
            logger.error(msg)
            return False, msg

    def jog_step(self, axis: str, direction: int, step_size: float = 1.0, speed: float = 10.0, acc: float = 10.0) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            msg = "jog_step: Robot is not connected"
            logger.error(msg)
            return False, msg

        cartesian_map = {"X": 0, "Y": 1, "Z": 2, "Rx": 3, "Ry": 4, "Rz": 5}
        joint_map = {"J1": 0, "J2": 1, "J3": 2, "J4": 3, "J5": 4, "J6": 5}

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

    def jog_continuous(self, axis: str, direction: int) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            msg = "jog_continuous: Robot is not connected"
            logger.error(msg)
            return False, msg

        cartesian_axes = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        joint_axes = ["J1", "J2", "J3", "J4", "J5", "J6"]
        dir_str = "+" if direction > 0 else "-"

        if direction == 0:
            logger.info(f"jog_continuous: Stop jogging: {axis}{dir_str}")
            if axis in cartesian_axes:
                self._driver.move_jog_cartesian("")
                return True, ""
            if axis in joint_axes:
                self._driver.move_jog_joint("")
                return True, ""
            return False, f"jog_continuous: Invalid axis {axis}"

        if axis in cartesian_axes:
            axis_id = axis + dir_str
            logger.info(f"jog_continuous: Start cartesian jog {axis_id} (Tool CoordType 0)")
            success = self._driver.move_jog_cartesian(axis_id, coord_type=0)
        elif axis in joint_axes:
            axis_id = axis + dir_str
            logger.info(f"jog_continuous: Start joint jog {axis_id}")
            success = self._driver.move_jog_joint(axis_id)
        else:
            return False, f"jog_continuous: Invalid axis {axis}"

        if not success:
            msg = f"jog_continuous: Failed to execute MoveJog({axis_id})"
            logger.error(msg)
            return False, msg

        return True, ""

    def go_zero(self, speed: float = 10.0, acc: float = 10.0) -> tuple[bool, str]:
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

    def go_fold(self, speed: float = 10.0, acc: float = 10.0) -> tuple[bool, str]:
        logger.info(f"go_fold: speed:{speed}, acc:{acc}")
        if not self._driver or not self._is_connected:
            msg = "go_fold: Robot is not connected"
            logger.error(msg)
            return False, msg

        try:
            return self.move_to_joint([0.0, 0.0, -156.0, 0.0, -170.0, 0.0], speed=speed, acc=acc)
        except Exception as e:
            msg = f"Error going fold: {e}"
            logger.error(msg)
            return False, msg

    def go_home(self, speed: float = 10.0, acc: float = 10.0) -> tuple[bool, str]:
        logger.info(f"go_home: speed:{speed}, acc:{acc}")
        if not self._driver or not self._is_connected:
            msg = "go_home: Robot is not connected"
            logger.error(msg)
            return False, msg

        try:
            res = self._driver.go_home(wait=True, velocity=speed, acc=acc)
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

    def broadcast_exec_progress(self, current_waypoint: int, total_waypoints: int, path_idx: int, total_paths: int):
        """向所有 WebSocket 客户端广播轨迹执行进度事件。"""
        payload = {
            "type": "exec_progress",
            "data": {
                "current_waypoint": current_waypoint,
                "total_waypoints": total_waypoints,
                "path_idx": path_idx,
                "total_paths": total_paths,
                "progress": current_waypoint / max(total_waypoints, 1),
            }
        }
        for cb in self._ws_callbacks:
            try:
                cb(payload)
            except Exception:
                pass

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

    def get_feedback_diagnostics(self) -> dict:
        """获取机械臂驱动层的实时动力学与诊断数据"""
        if self._driver and hasattr(self._driver, "get_feedback_diagnostics"):
            try:
                return self._driver.get_feedback_diagnostics()
            except Exception as e:
                logger.warning(f"Error getting feedback diagnostics: {e}")
        return {
            "tcp_speed_actual": [0.0] * 6,
            "qd_actual": [0.0] * 6,
            "load": 0.0,
            "error_status": 0,
            "tool_vector_actual": [0.0] * 6,
            "hand_type": [0, 0, 0, 0],
            "tool_index": 0,
            "run_queued_cmd": 0,
            "velocity_ratio": 0,
            "xyz_velocity_ratio": 0,
            "r_velocity_ratio": 0,
            "digital_outputs": [0] * 16,
            "digital_output_bits": 0,
        }

    def _poll_loop(self, interval: float):
        last_status = None
        last_error_status = 0
        cached_error_details = []
        while not self._stop_polling and self._is_connected:
            pose, _ = self.get_current_pose()
            joints, _ = self.get_current_joint()
            status = self._driver.get_running_state() if self._driver else 0
            diagnostics = self.get_feedback_diagnostics()

            if status != last_status:
                logger.info(f"Robot status changed: {status}")
                last_status = status

            if pose and joints:
                current_error = diagnostics.get("error_status", 0)
                if current_error != 0 and last_error_status == 0:
                    cached_error_details = self._driver.get_error_details() if self._driver else []
                elif current_error == 0:
                    cached_error_details = []
                last_error_status = current_error

                msg_payload = {
                    "pose": pose,
                    "joint": joints,
                    "status": status,
                    "tcp_speed_actual": diagnostics.get("tcp_speed_actual", [0.0] * 6),
                    "tcp_speed_mm_s": diagnostics.get("tcp_speed_mm_s", 0.0),
                    "qd_actual": diagnostics.get("qd_actual", [0.0] * 6),
                    "load": diagnostics.get("load", 0.0),
                    "error_status": current_error,
                    "error_details": cached_error_details,
                    "tool_vector_actual": diagnostics.get("tool_vector_actual", pose),
                    "hand_type": diagnostics.get("hand_type", [0, 0, 0, 0]),
                    "tool_index": diagnostics.get("tool_index", 0),
                    "run_queued_cmd": diagnostics.get("run_queued_cmd", 0),
                    "velocity_ratio": diagnostics.get("velocity_ratio", 0),
                    "xyz_velocity_ratio": diagnostics.get("xyz_velocity_ratio", 0),
                    "r_velocity_ratio": diagnostics.get("r_velocity_ratio", 0),
                    "digital_outputs": diagnostics.get("digital_outputs", [0] * 16),
                    "digital_output_bits": diagnostics.get("digital_output_bits", 0),
                }
                for cb in self._ws_callbacks:
                    try:
                        cb({"type": "robot_state", "data": msg_payload})
                    except Exception:
                        pass
            time.sleep(interval)

    def pause(self) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            return False, "Robot is not connected"
        logger.info("pausing robot...")
        success = self._driver.pause()
        logger.info(f"robot paused. Success: {success}")
        return success, "" if success else "Failed to pause robot"

    def resume(self) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            return False, "Robot is not connected"
        logger.info("resuming robot...")
        success = self._driver.resume()
        logger.info(f"robot resumed. Success: {success}")
        return success, "" if success else "Failed to resume robot"

    def clear_error(self) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            return False, "Robot is not connected"
        logger.info("clearing error...")
        try:
            if hasattr(self._driver, "clear_error"):
                self._driver.clear_error()
                return True, ""
            elif hasattr(self._driver, "dashboard"):
                r = self._driver.dashboard.ClearError()
                logger.info(f"ClearError: {r}")
                # Dobot V3 protocol requires calling EnableRobot again to reopen the motion queue after an alarm is cleared.
                time.sleep(0.5)
                r2 = self._driver.dashboard.EnableRobot()
                logger.info(f"EnableRobot after clear: {r2}")
                return True, r
            return True, ""
        except Exception as e:
            return False, str(e)

    def estop(self) -> tuple[bool, str]:
        if not self._driver or not self._is_connected:
            return False, "Robot is not connected"
        logger.info("estoping robot...")
        success = self._driver.estop()
        return success, "" if success else "Failed to estop robot"


robot_service = RobotService()
