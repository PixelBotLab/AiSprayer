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
