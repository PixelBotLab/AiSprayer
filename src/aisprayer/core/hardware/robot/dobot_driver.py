import logging
import time
from typing import Optional, List
import math

from .base_driver import BaseRobotDriver, RobotPose, PoseLike, _to_list
from .dobot_api import DobotApiDashboard, DobotApiMove

logger = logging.getLogger(__name__)

class DobotDriver(BaseRobotDriver):
    def __init__(self, ip: str = "192.168.5.1", dashboard_port: int = 29999, move_port: int = 30003, toolnum: int = 0):
        self.ip = ip
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.tool_num = toolnum
        self.dashboard: Optional[DobotApiDashboard] = None
        self.move: Optional[DobotApiMove] = None
        self._connected = False
        self._global_speed = 50

    def startup(self, timeout: float = 10.0) -> bool:
        try:
            self.dashboard = DobotApiDashboard(self.ip, self.dashboard_port)
            self.move = DobotApiMove(self.ip, self.move_port)
            self._connected = True
            
            # Enable robot
            self.dashboard.ClearError()
            time.sleep(0.5)
            self.dashboard.EnableRobot()
            
            # Set global speed and tool
            self.set_global_speed(self._global_speed)
            if self.tool_num > 0:
                self.set_tool_number(self.tool_num)
                
            return True
        except Exception as e:
            logger.error(f"Failed to start Dobot: {e}")
            self._connected = False
            return False

    def shutdown(self) -> None:
        if self._connected and self.dashboard:
            try:
                self.dashboard.DisableRobot()
                self.dashboard.close()
            except Exception:
                pass
        if self._connected and self.move:
            try:
                self.move.close()
            except Exception:
                pass
        self._connected = False

    def get_running_state(self) -> int:
        if not self._connected or not self.dashboard:
            return 0
        try:
            # RobotMode() returns string like "RobotMode(),5" where 5 is running, 7 is idle, etc.
            # We will map it roughly to 0=stop, 1=pause, 2=running
            res = self.dashboard.RobotMode()
            if res:
                parts = res.split(',')
                if len(parts) >= 2:
                    mode_str = parts[1].strip('{} \t\n\r')
                    mode = int(mode_str)
                    # Dobot RobotMode (rough map):
                    # 5: Running, 7: Normal idle, 9: error/alarm
                    if mode == 5:
                        return 2
                    return 0
        except Exception as e:
            logger.warning(f"Error getting Dobot state: {e}")
        return 0

    def is_robot_idle(self) -> bool:
        return self.get_running_state() == 0

    def get_current_pose(self) -> Optional[RobotPose]:
        if not self._connected or not self.dashboard:
            return None
        try:
            # GetPose() returns "GetPose(),x,y,z,rx,ry,rz;"
            res = self.dashboard.GetPose()
            if res:
                if res.startswith("Error"):
                    return None
                start_idx = res.find('{')
                end_idx = res.find('}')
                if start_idx != -1 and end_idx != -1:
                    vals_str = res[start_idx+1:end_idx].split(',')
                else:
                    # sometimes separated by comma directly after GetPose(), 
                    # format: ErrorID, {X, Y, Z, Rx, Ry, Rz}
                    parts = res.split('{')
                    if len(parts) > 1:
                        vals_str = parts[1].split('}')[0].split(',')
                    else:
                        vals_str = []
                        
                if len(vals_str) >= 6:
                    x, y, z, rx, ry, rz = [float(v) for v in vals_str[:6]]
                    # Dobot uses degrees for Rx, Ry, Rz, but RobotPose expects radians
                    return RobotPose(x, y, z, math.radians(rx), math.radians(ry), math.radians(rz))
        except Exception as e:
            logger.error(f"Error parsing Dobot pose: {e}")
        return None

    def is_reachable(self, pose: PoseLike, movetype: str = "MOVJ") -> bool:
        if not self._connected or not self.dashboard:
            return False
        lst = _to_list(pose)
        # Convert radians back to degrees for checking
        rx_deg = math.degrees(lst[3])
        ry_deg = math.degrees(lst[4])
        rz_deg = math.degrees(lst[5])
        try:
            # InverseSolution(x,y,z,rx,ry,rz,user,tool)
            res = self.dashboard.InverseSolution(lst[0], lst[1], lst[2], rx_deg, ry_deg, rz_deg, 0, self.tool_num)
            # If successful, returns joint angles, else error
            if res and "Error" not in res and "0,{" in res:
                return True
        except Exception:
            pass
        return False

    def _wait_motion_done(self, timeout: float = 600.0) -> bool:
        # Wait up to 1 second for the robot to register the motion and leave Idle state
        start_wait = time.time()
        while time.time() - start_wait < 1.0:
            if not self.is_robot_idle():
                break
            time.sleep(0.05)
            
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_robot_idle():
                return True
            time.sleep(0.05)
        return False

    def move_j(self, pose: PoseLike, velocity: float = 40.0, acc: float = 80.0, dec: float = 80.0, tool_num: int = 0, wait: bool = True) -> int:
        if not self._connected or not self.move:
            return -2
        lst = _to_list(pose)
        x, y, z = lst[0], lst[1], lst[2]
        rx_deg, ry_deg, rz_deg = math.degrees(lst[3]), math.degrees(lst[4]), math.degrees(lst[5])
        
        if self.dashboard:
            self.dashboard.SpeedJ(int(velocity))
            self.dashboard.AccJ(int(acc))
            
        # We can dynamically set speed if needed, but DobotMove API just sends the command.
        # Alternatively we can use SpeedJ() from dashboard
        self.move.MovJ(x, y, z, rx_deg, ry_deg, rz_deg)
        
        if wait:
            self._wait_motion_done()
        return 0

    def move_joint(self, joints: List[float], velocity: float = 40.0, acc: float = 80.0, dec: float = 80.0, tool_num: int = 0, wait: bool = True) -> int:
        if not self._connected or not self.move:
            return -2
        if len(joints) < 6:
            joints.extend([0.0]*(6-len(joints)))
            
        if self.dashboard:
            self.dashboard.SpeedJ(int(velocity))
            self.dashboard.AccJ(int(acc))
        
        self.move.JointMovJ(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5])
        
        if wait:
            self._wait_motion_done()
        return 0

    def move_l(self, pose: PoseLike, velocity: float = 100.0, acc: float = 80.0, dec: float = 80.0, tool_num: int = 0, wait: bool = True) -> int:
        if not self._connected or not self.move:
            return -2
        lst = _to_list(pose)
        x, y, z = lst[0], lst[1], lst[2]
        rx_deg, ry_deg, rz_deg = math.degrees(lst[3]), math.degrees(lst[4]), math.degrees(lst[5])
        
        if self.dashboard:
            self.dashboard.SpeedL(int(velocity))
            self.dashboard.AccL(int(acc))
            
        self.move.MovL(x, y, z, rx_deg, ry_deg, rz_deg)
        
        if wait:
            self._wait_motion_done()
        return 0

    def move_j_queue(
        self, 
        poses: List[PoseLike], 
        velocity: float = 40.0, 
        acc: float = 80.0, 
        dec: float = 80.0,
        tool_num: int = 0,
        wait: bool = True,
        cp_ratio: int = 50
    ) -> int:
        if not self._connected or not self.move:
            return -2
            
        if self.dashboard:
            self.dashboard.CP(cp_ratio)
            self.dashboard.SpeedJ(int(velocity))
            self.dashboard.AccJ(int(acc))
            
        for pose in poses:
            lst = _to_list(pose)
            x, y, z = lst[0], lst[1], lst[2]
            rx_deg, ry_deg, rz_deg = math.degrees(lst[3]), math.degrees(lst[4]), math.degrees(lst[5])
            self.move.MovJ(x, y, z, rx_deg, ry_deg, rz_deg)
            
        if wait:
            self._wait_motion_done()
            
        return 0

    def move_l_queue(
        self, 
        poses: List[PoseLike], 
        velocity: float = 100.0, 
        acc: float = 80.0, 
        dec: float = 80.0,
        tool_num: int = 0,
        wait: bool = True,
        cp_ratio: int = 50
    ) -> int:
        if not self._connected or not self.move:
            return -2
            
        if self.dashboard:
            self.dashboard.CP(cp_ratio)
            self.dashboard.SpeedL(int(velocity))
            self.dashboard.AccL(int(acc))
            
        for pose in poses:
            lst = _to_list(pose)
            x, y, z = lst[0], lst[1], lst[2]
            rx_deg, ry_deg, rz_deg = math.degrees(lst[3]), math.degrees(lst[4]), math.degrees(lst[5])
            self.move.MovL(x, y, z, rx_deg, ry_deg, rz_deg)
            
        if wait:
            self._wait_motion_done()
            
        return 0

    def set_tool_number(self, tool_num: int) -> bool:
        if not self._connected or not self.dashboard:
            return False
        res = self.dashboard.Tool(tool_num)
        if res and "0" in res:
            self.tool_num = tool_num
            return True
        return False

    def set_global_speed(self, speed: int) -> bool:
        if not self._connected or not self.dashboard:
            return False
        # Set both joint and linear speed ratio
        self.dashboard.SpeedFactor(int(speed))
        self._global_speed = int(speed)
        return True

    def go_home(self, wait: bool = True, velocity: Optional[float] = None) -> int:
        if velocity is not None:
            self.set_global_speed(int(velocity))
        # Default home position for Dobot
        return self.move_joint([0, 0, -90, 90, 90, 0], wait=wait)
