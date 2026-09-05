# -*- coding: utf-8 -*-
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
