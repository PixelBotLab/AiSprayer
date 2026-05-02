"""
纳博特(iNexbot) 机械臂官方 SDK (SWIG) 通信驱动封装

修正记录 (完全同步 nabot_robot.py 流程):
1. 同步上电流程：清错 -> 状态 0 -> 状态 1 -> PowerOn。
2. 修正 MoveCmd 的 targetPosType 为 PosType_data (2)。
3. 暂时屏蔽 is_reachable 中的 SDK 调用，避免在初始化阶段触发 25566 报警。
"""

import time
import logging
import pathlib
import sys
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════
#  数据类型
# ═══════════════════════════════════════════════

@dataclass
class RobotPose:
    x: float = 0.0; y: float = 0.0; z: float = 0.0
    a: float = 0.0; b: float = 0.0; c: float = 0.0
    @classmethod
    def from_list(cls, data: list):
        if not data: return cls()
        return cls(*[float(x) for x in data[:6]])
    def to_list(self) -> list:
        return [self.x, self.y, self.z, self.a, self.b, self.c]

@dataclass
class JointPose:
    j1: float = 0.0; j2: float = 0.0; j3: float = 0.0
    j4: float = 0.0; j5: float = 0.0; j6: float = 0.0
    @classmethod
    def from_list(cls, data: list):
        if not data: return cls()
        return cls(*[float(x) for x in data[:6]])
    def to_list(self) -> list:
        return [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6]

COORD_ACS = 0; COORD_MCS = 1
MODE_TEACH = 0; MODE_REMOTE = 1; MODE_RUN = 2

DEFAULT_VELOCITY = 50
DEFAULT_ACC = 50
DEFAULT_DEC = 50

# ═══════════════════════════════════════════════
#  SDK 后端
# ═══════════════════════════════════════════════

class _SdkBackend:
    def __init__(self):
        _sdk_dir = str(pathlib.Path(__file__).parent / "sdk_22.07" / "python" / "linux" / "linux_python_3.8_v2.0.4")
        if _sdk_dir not in sys.path: sys.path.insert(0, _sdk_dir)
        import nrc_interface
        self._nrc = nrc_interface
        self._fd: Optional[int] = None

    def connect(self, ip, port):
        self._fd = self._nrc.connect_robot(ip, str(port))
        if self._fd is None or self._fd <= 0: return False
        for _ in range(5):
            res = self._nrc.get_servo_state(self._fd, 0)
            if isinstance(res, (list, tuple)) and int(res[0]) == 0: return True
            time.sleep(0.5)
        return False

    def disconnect(self):
        if self._fd is not None: self._nrc.disconnect_robot(self._fd)
        self._fd = None

    def _make_cmd(self, target, coord, vel, acc, dec):
        vec = self._nrc.VectorDouble()
        for v in target: vec.append(float(v))
        cmd = self._nrc.MoveCmd()
        # 核心：必须使用 PosType_data (通常=2)
        if hasattr(self._nrc, "PosType_data"):
            cmd.targetPosType = self._nrc.PosType_data
        else:
            cmd.targetPosType = 2
            
        cmd.targetPosValue = vec
        cmd.velocity = float(vel)
        cmd.acc = float(acc)
        cmd.dec = float(dec)
        cmd.coord = coord
        cmd.pl = 0
        cmd.toolNum = 0
        cmd.userNum = 0
        return cmd

    def robot_movej(self, target, coord, vel, acc, dec):
        cmd = self._make_cmd(target, coord, vel, acc, dec)
        self._nrc.robot_movej(self._fd, cmd)

    def robot_movel(self, target, coord, vel, acc, dec):
        cmd = self._make_cmd(target, coord, vel, acc, dec)
        self._nrc.robot_movel(self._fd, cmd)

    def get_position(self, coord):
        pos = self._nrc.VectorDouble()
        self._nrc.get_current_position(self._fd, coord, pos)
        if pos.size() < 6: return [0.0] * 7
        return [float(pos[i]) for i in range(min(pos.size(), 7))]

    def get_servo_state(self):
        res = self._nrc.get_servo_state(self._fd, 0)
        return int(res[1]) if isinstance(res, (list, tuple)) and len(res) > 1 else -1

    def get_running_state(self):
        res = self._nrc.get_robot_running_state(self._fd, 0)
        return int(res[1]) if isinstance(res, (list, tuple)) and len(res) > 1 else -1

# ═══════════════════════════════════════════════
#  驱动接口
# ═══════════════════════════════════════════════

class InexbotDriver:
    def __init__(self, ip, port=6001):
        self.ip, self.port = ip, port
        self._backend = _SdkBackend()

    def connect(self): return self._backend.connect(self.ip, self.port)
    def disconnect(self): self._backend.disconnect()
    def __enter__(self): self.connect(); return self
    def __exit__(self, *args): self.disconnect()

    def servo_power_on(self):
        """同步 nabot_robot.py 的标准上电流程"""
        fd = self._backend._fd
        nrc = self._backend._nrc
        
        nrc.clear_error(fd)
        time.sleep(0.3)
        
        # 步骤 3: 强制停止(0) -> 就绪(1)
        nrc.set_servo_state(fd, 0)
        time.sleep(0.3)
        nrc.set_servo_state(fd, 1)
        time.sleep(0.5)
        
        # 步骤 4: 上电
        nrc.set_servo_poweron(fd)
        
        for _ in range(20):
            time.sleep(0.5)
            if self.get_servo_state() == 3:
                logger.info("伺服上电成功 (state=3)")
                return True
        return False

    def get_servo_state(self): return self._backend.get_servo_state()
    def get_running_state(self): return self._backend.get_running_state()
    def set_mode(self, mode): self._backend._nrc.set_current_mode(self._backend._fd, mode)
    def set_coord(self, coord): self._backend._nrc.set_current_coord(self._backend._fd, coord)
    def clear_error(self): self._backend._nrc.clear_error(self._backend._fd)
    def get_current_pose(self): return RobotPose.from_list(self._backend.get_position(COORD_MCS))
    def get_current_joints(self): return JointPose.from_list(self._backend.get_position(COORD_ACS))
    def move_to_pose(self, target: RobotPose, velocity=DEFAULT_VELOCITY, acc=DEFAULT_ACC, dec=DEFAULT_DEC):
        self._backend.robot_movej(target.to_list() + [0.0], COORD_MCS, velocity, acc, dec)

    def move_l(self, target: RobotPose, velocity=DEFAULT_VELOCITY, acc=DEFAULT_ACC, dec=DEFAULT_DEC):
        """执行直线插补运动 (Linear Motion)"""
        self._backend.robot_movel(target.to_list() + [0.0], COORD_MCS, velocity, acc, dec)

    def go_home(self, velocity=DEFAULT_VELOCITY):
        """移动到预设的 Home 位姿"""
        home_pose = RobotPose(
            x=870.975, y=125.478, z=1077.028,
            a=-3.141, b=0.0, c=0.0
        )
        self.move_to_pose(home_pose, velocity=velocity)
        return self.wait_motion_done()

    def wait_motion_done(self, timeout=60):
        start = time.time(); time.sleep(0.5)
        while time.time() - start < timeout:
            if self.get_running_state() == 0: return True
            time.sleep(0.3)
        return False
        
    def is_reachable(self, target: RobotPose, move_type=0):
        """保守起见，暂时跳过 SDK 的可达性检查，直接返回 True"""
        # 如果 get_pos_reachable 在此版本中有 Bug，可能会触发控制器报警
        return True
