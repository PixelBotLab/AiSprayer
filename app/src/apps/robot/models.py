# -*- coding: utf-8 -*-
from typing import Optional
from pydantic import BaseModel



class ConnectRobotReq(BaseModel):
    robot_type: str = "dobot"


class JogReq(BaseModel):
    axis: str
    direction: int
    step: float = 1.0
    speed_l: float = 10.0
    acc_l: float = 10.0
    speed_j: float = 10.0
    acc_j: float = 10.0


class JogContinuousReq(BaseModel):
    axis: str
    direction: int


class HomeReq(BaseModel):
    speed: float = 10.0
    acc: float = 10.0


class SpeedReq(BaseModel):
    speed_l: float
    acc_l: float
    speed_j: float
    acc_j: float


class GlobalSpeedReq(BaseModel):
    factor: int


class SetDoReq(BaseModel):
    index: Optional[int] = None      # DO 端子编号 (1-16), 若未传入则从配置读取喷涂 DO
    status: int = 1                  # 1: 开, 0: 关
    immediate: bool = True           # True 为立即指令(手动开关需立即生效), False 为队列指令


class GripperMoveReq(BaseModel):
    stroke_mm: float = 0.0                      # 目标开度 (mm), 默认闭合 0.0mm
    force_percent: Optional[int] = None         # 夹持力比例 1 ~ 100 (%), None 则使用硬件规格默认值
    speed: Optional[int] = None                 # 速度比例 1 ~ 100 (%), None 则使用硬件规格默认值
    wait_complete: bool = False                 # 是否等待动作完成


class AimAtPixelReq(BaseModel):
    """实时视频“光束指向”入参: 让工具轴线 (光束) 与被点的像素共线, 深度猜错也打中同一视线。"""
    u_px: float                                  # 像素列坐标 (相机内参分辨率坐标系)
    v_px: float                                  # 像素行坐标 (相机内参分辨率坐标系)
    distance_mm: Optional[float] = None          # 被点像素的深度标注 (沿射线 mm), 仅用于界面回显; None 读 spraying.aim_distance_mm
    speed: Optional[float] = None                # MovJ 速度 (deg/s); None 用 spraying.aim_speed_percent 折算的百分比
    acc: Optional[float] = None                  # 加速度 (%); None 读控制面板当前关节加速度


class MarkCrossReq(BaseModel):
    """人工点选"肉眼看到的十字中心" (绕过 CV 检测做真值校核): 像素在内参分辨率坐标系。"""
    u_px: float                                  # 十字中心像素列坐标 (相机内参分辨率坐标系)
    v_px: float                                  # 十字中心像素行坐标 (相机内参分辨率坐标系)


class UiLogReq(BaseModel):
    """前端把界面上弹出的提示/通知回显到专用校核日志, 免得用户手动复制粘贴给助手。"""
    level: str = "info"                          # info / ok / error: 只影响日志级别, 不影响行为
    message: str                                 # 界面当时显示给用户的英文原文


class GripperActionReq(BaseModel):
    force_percent: Optional[int] = None         # 夹持力比例 1 ~ 100 (%)
    speed: Optional[int] = None                 # 速度比例 1 ~ 100 (%)
    wait_complete: bool = False                 # 是否等待动作完成
