# -*- coding: utf-8 -*-
"""命令下沉接口 (依赖倒置): core 只依赖 TeleopCommandSink 抽象协议。

- 上层 teleop.py 通过本协议下发"高层遥操作命令"(点动/开关喷/急停/回位/速率), 不 import
  任何 robot_service / FastAPI / 传输层, 保持 core 纯净与可单测;
- 具体实现由各运行环境注入: 本模块提供 ConsoleCommandSink (干跑, 只打印英文动作),
  app 层另建 RobotServiceSink 适配到 RobotService 既有色接口 (jog_continuous / set_do /
  estop / ...); 单测用 FakeCommandSink 记录命令序列。

协议隔离: 手柄连接方式(2.4G/USB/蓝牙)与本 sink 无关 —— sink 只认逻辑命令, 换协议不影响它。
"""

from __future__ import annotations

import time
from typing import List, Protocol, Tuple, runtime_checkable


@runtime_checkable
class TeleopCommandSink(Protocol):
    """遥操作命令下沉协议。axis ∈ {X,Y,Z,Rx,Ry,Rz, J1..J6}; direction ∈ {-1,0,+1}。"""

    def connect(self) -> bool:
        """连接机器人并使能伺服 (幂等; 对应 START 开机)。返回是否成功。"""
        ...

    def disconnect(self) -> None:
        """断开机器人/去使能 (对应 START 关机)。"""
        ...

    def jog_start(self, axis: str, direction: int) -> None:
        """启动某单轴点动 (direction 仅符号, 底层无比例速度)。"""
        ...

    def jog_stop(self, axis: str) -> None:
        """停止某轴点动 (等价 direction=0)。任何时候允许下发停止。"""
        ...

    def stop_all_jogs(self) -> None:
        """停止当前所有点动 (dead-man 释放 / 模式切换 / 安全停止用)。"""
        ...

    def set_spray(self, on: bool) -> None:
        """手动开/关喷涂 DO (实现必须走立即指令 DOExecute)。"""
        ...

    def set_speed_tier(self, percent: int) -> None:
        """设置全局速率档 (1..100)。"""
        ...

    def pause(self) -> None: ...
    def resume(self) -> None: ...
    def clear_error(self) -> None: ...
    def estop(self) -> None:
        """急停 (实现内部必须先立即关喷 DO 再停机)。"""
        ...
    def go_home(self) -> None: ...
    def go_fold(self) -> None: ...

    def gripper_open(self) -> None:
        """完全张开夹爪 (一键到底; 具体实现按硬件 specs 限幅)。"""
        ...

    def gripper_close(self) -> None:
        """完全闭合/夹持夹爪 (一键到底)。"""
        ...


class ConsoleCommandSink:
    """干跑 sink: 把每条命令打印成英文动作行, 不触碰任何硬件。用于本模块测试脚本。"""

    def __init__(self) -> None:
        self.log: List[Tuple[float, str]] = []

    def _emit(self, action: str) -> None:
        line = f"[sink] {action}"
        self.log.append((time.time(), action))
        print(line)

    def jog_start(self, axis: str, direction: int) -> None:
        self._emit(f"jog {axis} {'+' if direction > 0 else '-'}")

    def jog_stop(self, axis: str) -> None:
        self._emit(f"jog stop {axis}")

    def stop_all_jogs(self) -> None:
        self._emit("jog stop ALL")

    def set_spray(self, on: bool) -> None:
        self._emit("Spray ON" if on else "Spray OFF")

    def set_speed_tier(self, percent: int) -> None:
        self._emit(f"Speed tier: {percent}%")

    def pause(self) -> None:
        self._emit("Motion paused")

    def resume(self) -> None:
        self._emit("Motion resumed")

    def clear_error(self) -> None:
        self._emit("Alarm cleared")

    def estop(self) -> None:
        self._emit("EMERGENCY STOP (spray OFF + halt)")

    def go_home(self) -> None:
        self._emit("Returning home...")

    def go_fold(self) -> None:
        self._emit("Folding arm...")

    def gripper_open(self) -> None:
        self._emit("Gripper OPEN")

    def gripper_close(self) -> None:
        self._emit("Gripper CLOSE")

    def connect(self) -> bool:
        self._emit("Robot connected (servo enabled)")
        return True

    def disconnect(self) -> None:
        self._emit("Robot disconnected")


class FakeCommandSink:
    """记录型 sink (供纯逻辑单测断言命令序列)。"""

    def __init__(self) -> None:
        self.calls: List[Tuple[str, tuple]] = []

    def _rec(self, name: str, *args) -> None:
        self.calls.append((name, args))

    def jog_start(self, axis, direction): self._rec("jog_start", axis, direction)
    def jog_stop(self, axis): self._rec("jog_stop", axis)
    def stop_all_jogs(self): self._rec("stop_all_jogs")
    def set_spray(self, on): self._rec("set_spray", on)
    def set_speed_tier(self, percent): self._rec("set_speed_tier", percent)
    def pause(self): self._rec("pause")
    def resume(self): self._rec("resume")
    def clear_error(self): self._rec("clear_error")
    def estop(self): self._rec("estop")
    def go_home(self): self._rec("go_home")
    def go_fold(self): self._rec("go_fold")
    def gripper_open(self): self._rec("gripper_open")
    def gripper_close(self): self._rec("gripper_close")
    def connect(self): self._rec("connect"); return True
    def disconnect(self): self._rec("disconnect")
