# -*- coding: utf-8 -*-
"""遥操作状态模型 (纯数据 + 无副作用): 状态机枚举、会话快照、LED 灯语纯函数。

- 这里只描述"当前处于什么状态/模式", 不做按键解析也不下发命令 (那在 teleop.py);
- led_pattern() 为纯函数, 给定时刻与状态返回 5 颗玩家灯的亮灭 (0/1), 便于离屏反馈与单测;
- 灯语仅作只读反馈, 安全逻辑绝不依赖它 (反馈失效仍保持 Fail-Close)。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional


class TeleopState(str, Enum):
    """主安全状态机: DISCONNECTED -> IDLE -> ENABLED <-> MOVING, 任意态可入 E-STOP。

    机器人连接本身即"使能闸门" (START 键 connect/disconnect); ENABLED/IDLE 仅区分 RT 是否握下。
    """

    DISCONNECTED = "DISCONNECTED"  # 机器人链路未连接 (START 可连接上电)
    IDLE = "IDLE"                  # 已连接待命, 未握 RT
    ENABLED = "ENABLED"            # 已连接且 RT 握下 (armed), 等摇杆输入
    MOVING = "MOVING"              # RT 按住且有输入, 正在点动
    E_STOP = "E-STOP"              # 急停/故障/看门狗, 需 X 清障 (链路保留)


class MotionMode(str, Enum):
    TRANSLATION = "TRANSLATION"    # 平移: 主导摇杆轴 -> X/Y/Z/Rz
    ROTATION = "ROTATION"          # 姿态: Rx/Ry/Rz (+ 扳机 Z 微调)
    JOINT = "JOINT"                # 关节: 按住即动 (J1..J6), 右摇杆给 ±


# BACK 单击循环顺序
MODE_ORDER = (MotionMode.TRANSLATION, MotionMode.ROTATION, MotionMode.JOINT)

# 5 颗玩家灯逻辑索引 -> sysfs 灯名后缀 (4 绿 player-1..4 + 1 蓝 player-5)
LED_NAMES = ("player-1", "player-2", "player-3", "player-4", "player-5")

# 模式 -> 常亮绿灯数 (一眼可辨, 连接期间一直成立)
_MODE_GREEN_COUNT = {
    MotionMode.TRANSLATION: 1,
    MotionMode.ROTATION: 2,
    MotionMode.JOINT: 3,
}


@dataclass
class SessionStatus:
    """一次遥操作会话的可读快照 (可后续广播到 HUD; 现为纯记录)。"""

    state: TeleopState = TeleopState.DISCONNECTED
    mode: MotionMode = MotionMode.TRANSLATION
    active_axis: Optional[str] = None    # 当前点动的机器人轴 (如 'X' / 'J3')
    speed_pct: int = 25
    armed: bool = False                  # RT dead-man 是否按住
    reduced: bool = False                # LT 低速是否生效
    spray_on: bool = False
    gripper_open: bool = False           # 夹爪是否处于张开位

    def hud_line(self) -> str:
        """英文单行摘要 (界面/日志一律英文)。"""
        return (
            f"MODE={self.mode.value} STATE={self.state.value} "
            f"ACTIVE={self.active_axis or '-'} SPEED={self.speed_pct}% "
            f"{'[RT ARMED]' if self.armed else ''}"
            f"{'[SLOW]' if self.reduced else ''}{'[SPRAY]' if self.spray_on else ''}"
            f"{'[GRIP OPEN]' if self.gripper_open else ''}"
        )


def _blink(now: float, hz: float) -> int:
    """按给定频率产生方波 0/1 (慢闪 ~1Hz / 快闪 ~3Hz / 故障双闪)。"""
    return int(now * hz * 2.0) % 2


def led_pattern(
    now: float,
    connected: bool,
    state: TeleopState,
    mode: MotionMode,
    reduced: bool = False,
    fault: bool = False,
    gripper_open: bool = False,
) -> Dict[str, int]:
    """返回 5 颗玩家灯在时刻 now 的亮灭 (1 亮 / 0 灭)。见 docs §6。

    优先级: 故障(蓝双闪,绿全灭) > 未连接(全灭) > 模式常显(绿数) + 状态(这几颗闪法)
    + 低速(player-4 额外常亮) + 夹爪张开(player-5 蓝常亮, 仅非故障时)。
    """
    pattern = {name: 0 for name in LED_NAMES}
    if fault:
        pattern["player-5"] = _blink(now, 3.0)   # 蓝: 急停/故障/看门狗 双闪
        return pattern
    if not connected:
        return pattern

    # 状态决定"模式绿"的亮法: IDLE 常亮 / ENABLED 慢闪 / MOVING 快闪
    if state == TeleopState.MOVING:
        factor = _blink(now, 3.0)
    elif state == TeleopState.ENABLED:
        factor = _blink(now, 1.0)
    else:
        factor = 1

    count = _MODE_GREEN_COUNT.get(mode, 1)
    for i in range(1, 4):                         # player-1..3 编码模式
        pattern[f"player-{i}"] = factor if i <= count else 0
    # player-4 = 低速标记 (LT 按住时额外常亮, 不随模式闪烁)
    pattern["player-4"] = 1 if reduced else 0
    # player-5 = 夹爪张开标记 (蓝常亮; 与故障蓝双闪互斥, 上面已优先返回)
    pattern["player-5"] = 1 if gripper_open else 0
    return pattern
