import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List, Union

@dataclass
class RobotPose:
    """
    机器人直角坐标位姿（弧度制）
    默认坐标系为直角坐标系，坐标轴顺序为(x, y, z, a, b, c)
    平移单位: mm, 姿态角单位: rad。
    """
    x: float = 0.0; y: float = 0.0; z: float = 0.0
    a: float = 0.0; b: float = 0.0; c: float = 0.0

    @classmethod
    def from_list(cls, data: list) -> "RobotPose":
        """从长度 >=6 的列表创建位姿RobotPose, 不足部分填 0。"""
        if not data: return cls()
        return cls(*[float(x) for x in data[:6]])

    def to_list(self) -> list:
        """转换为 [x,y,z,a,b,c] 列表。"""
        return [self.x, self.y, self.z, self.a, self.b, self.c]
    
    def __repr__(self):
        """返回可读性较好的字符串表示。"""
        return (f"RobotPose(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
                f"a={self.a:.3f}, b={self.b:.3f}, c={self.c:.3f})")
    
    def __eq__(self, other):
        """重载==，允许直接比较两个 RobotPose 对象是否相等。"""
        if not isinstance(other, RobotPose):
            return NotImplemented
        return (math.isclose(self.x, other.x) and math.isclose(self.y, other.y) and 
                math.isclose(self.z, other.z) and math.isclose(self.a, other.a) and 
                math.isclose(self.b, other.b) and math.isclose(self.c, other.c))

PoseLike = Union[RobotPose, List[float]]

def _to_list(pose: PoseLike) -> list:
    """将 PoseLike 转换为 list 形式。"""
    if isinstance(pose, RobotPose):
        return pose.to_list()
    return [float(x) for x in pose]

class BaseRobotDriver(ABC):
    """
    机械臂驱动抽象基类，定义统一的对外控制接口。
    """
    
    @abstractmethod
    def startup(self, timeout: float = 10.0) -> bool:
        """
        启动机器人连接并使能伺服
        :param timeout: 超时时间(秒)
        :return: 是否启动成功
        """
        pass
        
    @abstractmethod
    def shutdown(self) -> None:
        """
        关闭机器人连接并下电/断开控制器
        """
        pass
        
    @abstractmethod
    def get_running_state(self) -> int:
        """
        获取机器人运行状态
        :return: 0=停止；1=暂停；2=运行中
        """
        pass
        
    @abstractmethod
    def is_robot_idle(self) -> bool:
        """
        运动程序是否空闲
        """
        pass

    @abstractmethod
    def get_current_pose(self) -> Optional[RobotPose]:
        """
        获取当前直角坐标位姿
        """
        pass

    @abstractmethod
    def is_reachable(self, pose: PoseLike, movetype: str = "MOVJ") -> bool:
        """
        判断目标位姿是否可达
        """
        pass

    @abstractmethod
    def move_j(
        self, 
        pose: PoseLike, 
        velocity: float = 50.0, 
        acc: float = 80.0, 
        dec: float = 80.0,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        关节运动(MOVJ)，目标位姿以直角坐标(笛卡尔)描述，由控制器内部做逆解。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 速度
        :param acc: 加速度
        :param dec: 减速度
        :param tool_num: 工具号
        :param wait: 是否等待运动完成
        :return: 返回码
        """
        pass

    @abstractmethod
    def move_joint(
        self, 
        joints: List[float], 
        velocity: float = 50.0, 
        acc: float = 80.0, 
        dec: float = 80.0,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        关节坐标系运动(Joint Movement)。目标位置由各关节角度确定。
        :param joints: 目标关节角列表(通常长度为6)
        :param velocity: 速度
        :param acc: 加速度
        :param dec: 减速度
        :param tool_num: 工具号
        :param wait: 是否等待运动完成
        :return: 返回码
        """
        pass

    @abstractmethod
    def move_l(
        self, 
        pose: PoseLike, 
        velocity: float = 50.0, 
        acc: float = 80.0, 
        dec: float = 80.0,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        直线运动(MOVL)，末端在笛卡尔空间走直线运动。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 速度
        :param acc: 加速度
        :param dec: 减速度
        :param tool_num: 工具号
        :param wait: 是否等待运动完成
        :return: 返回码
        """
        pass

    @abstractmethod
    def set_tool_number(self, tool_num: int) -> bool:
        """设置当前激活的工具编号"""
        pass

    @abstractmethod
    def set_global_speed(self, speed: int) -> bool:
        """设置全局速度百分比"""
        pass

    @abstractmethod
    def go_home(self, wait: bool = True, velocity: Optional[float] = None) -> int:
        """返回原点运动"""
        pass
