import logging
import time
import threading
import math
from typing import Optional, List

from .base_driver import BaseRobotDriver, RobotPose, PoseLike, _to_list
from .dobot_api import DobotApiDashboard, DobotApiMove, DobotApiFeedBack

logger = logging.getLogger(__name__)

class DobotDriver(BaseRobotDriver):
    def __init__(self, ip: str = "192.168.5.1", dashboard_port: int = 29999, move_port: int = 30003, toolnum: int = 0):
        self.ip = ip
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.tool_num = toolnum
        self.dashboard: Optional[DobotApiDashboard] = None
        self.move: Optional[DobotApiMove] = None
        self.feedback: Optional[DobotApiFeedBack] = None
        self._connected = False
        self._global_speed = 50
        # 30004 real-time data cache
        self._cached_joint: Optional[List[float]] = None   # degrees
        self._cached_pose: Optional[List[float]] = None    # mm, degrees
        self._cached_speed_l: float = 20.0
        self._cached_acc_l: float = 20.0
        self._cached_speed_j: float = 20.0
        self._cached_acc_j: float = 20.0
        self._cached_running_status: int = 0
        self._feedback_thread: Optional[threading.Thread] = None
        self._stop_feedback = False

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

            # Start 30004 real-time feedback thread
            self._start_feedback_thread()
                
            return True
        except Exception as e:
            logger.error(f"Failed to start Dobot: {e}")
            self._connected = False
            return False

    def shutdown(self) -> None:
        self._stop_feedback_thread()
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

    def _start_feedback_thread(self):
        """Connect to port 30004 and start real-time data reading thread."""
        try:
            self.feedback = DobotApiFeedBack(self.ip, 30004)
            self._stop_feedback = False
            self._feedback_thread = threading.Thread(
                target=self._feedback_loop, daemon=True, name="dobot-feedback-30004"
            )
            self._feedback_thread.start()
            logger.info("Dobot 30004 real-time feedback thread started")
        except Exception as e:
            logger.warning(f"Could not connect to 30004 feedback port: {e}. Falling back to polling.")
            self.feedback = None

    def _stop_feedback_thread(self):
        self._stop_feedback = True
        if self._feedback_thread:
            self._feedback_thread.join(timeout=1.0)
            self._feedback_thread = None
        if self.feedback:
            try:
                self.feedback.close()
            except Exception:
                pass
            self.feedback = None

    def _feedback_loop(self):
        """Continuously read from 30004 at ~200Hz and cache joint/pose data."""
        while not self._stop_feedback and self._connected:
            try:
                data = self.feedback.feedBackData()
                if data is not None and len(data) > 0:
                    d = data[0]
                    # q_actual: joint angles in degrees
                    q = d['q_actual'].tolist()
                    self._cached_joint = q
                    # tool_vector_actual: [X(mm), Y(mm), Z(mm), Rx(deg), Ry(deg), Rz(deg)]
                    tcp = d['tool_vector_actual'].tolist()
                    self._cached_pose = tcp
                    
                    if 'xyz_velocity_ratio' in d.dtype.names:
                        self._cached_speed_l = float(d['xyz_velocity_ratio'].item())
                    if 'xyz_acceleration_ratio' in d.dtype.names:
                        self._cached_acc_l = float(d['xyz_acceleration_ratio'].item())
                    if 'r_velocity_ratio' in d.dtype.names:
                        self._cached_speed_j = float(d['r_velocity_ratio'].item())
                    if 'r_acceleration_ratio' in d.dtype.names:
                        self._cached_acc_j = float(d['r_acceleration_ratio'].item())
                    if 'running_status' in d.dtype.names:
                        self._cached_running_status = int(d['running_status'].item())
            except Exception as e:
                if not self._stop_feedback:
                    logger.debug(f"Feedback read error: {e}")
                break

    def get_running_state(self) -> int:
        if not self._connected:
            return 0
        try:
            mode = self._cached_running_status
            # logger.info(f"DEBUG: _cached_running_status = {mode}")
            # 5: Running, 7: Normal idle (Common for Dobot)
            # Actually, let's just return the raw mode for now so the frontend can see it
            return mode
        except Exception as e:
            logger.warning(f"Error getting Dobot state: {e}")
        return 0

    def is_robot_idle(self) -> bool:
        return self.get_running_state() == 0

    def get_current_pose(self) -> Optional[RobotPose]:
        if not self._connected:
            return None
        # Fast path: use real-time 30004 cache
        if self._cached_pose is not None:
            try:
                x, y, z, rx, ry, rz = self._cached_pose[:6]
                # tool_vector_actual: mm, degrees → RobotPose expects mm, radians
                return RobotPose(x, y, z, math.radians(rx), math.radians(ry), math.radians(rz))
            except Exception:
                pass
        # Fallback: poll via dashboard
        try:
            res = self.dashboard.GetPose()
            if res:
                if res.startswith("Error"):
                    return None
                start_idx = res.find('{')
                end_idx = res.find('}')
                if start_idx != -1 and end_idx != -1:
                    vals_str = res[start_idx+1:end_idx].split(',')
                else:
                    parts = res.split('{')
                    vals_str = parts[1].split('}')[0].split(',') if len(parts) > 1 else []
                if len(vals_str) >= 6:
                    x, y, z, rx, ry, rz = [float(v) for v in vals_str[:6]]
                    return RobotPose(x, y, z, math.radians(rx), math.radians(ry), math.radians(rz))
        except Exception as e:
            logger.error(f"Error parsing Dobot pose: {e}")
        return None

    def get_current_joint(self) -> Optional[List[float]]:
        if not self._connected:
            return None
        # Fast path: use real-time 30004 cache (degrees)
        if self._cached_joint is not None:
            return list(self._cached_joint)
        # Fallback: poll via dashboard
        try:
            res = self.dashboard.GetAngle()
            if res:
                if res.startswith("Error"):
                    return None
                start_idx = res.find('{')
                end_idx = res.find('}')
                if start_idx != -1 and end_idx != -1:
                    vals_str = res[start_idx+1:end_idx].split(',')
                else:
                    parts = res.split('{')
                    vals_str = parts[1].split('}')[0].split(',') if len(parts) > 1 else []
                if len(vals_str) >= 6:
                    return [float(v) for v in vals_str[:6]]
        except Exception as e:
            logger.error(f"Error parsing Dobot joint angles: {e}")
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

    def get_global_speed(self) -> tuple[float, float, float, float]:
        if not self._connected:
            return 20.0, 20.0, 20.0, 20.0
        return self._cached_speed_l, self._cached_acc_l, self._cached_speed_j, self._cached_acc_j

    def go_home(self, wait: bool = True, velocity: Optional[float] = None) -> int:
        if velocity is not None:
            self.set_global_speed(int(velocity))
        # Default home position for Dobot
        return self.move_joint([0, 0, -90, -90, -90, 0], wait=wait)

    def pause(self) -> bool:
        if not self._connected:
            return False
        try:
            self.dashboard.pause()
            return True
        except Exception as e:
            logger.error(f"Error pausing robot: {e}")
            return False

    def resume(self) -> bool:
        if not self._connected:
            return False
        try:
            self.dashboard.Continue()
            return True
        except Exception as e:
            logger.error(f"Error resuming robot: {e}")
            return False

    def estop(self) -> bool:
        if not self._connected:
            return False
        try:
            self.dashboard.EmergencyStop()
            return True
        except Exception as e:
            logger.error(f"Error emergency stopping robot: {e}")
            return False
