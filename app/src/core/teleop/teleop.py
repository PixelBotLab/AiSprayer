# -*- coding: utf-8 -*-
"""遥操作编排层: TeleopController —— 逻辑输入 -> 状态机 -> 高层命令。

本层只认"逻辑控件名"(A/RB/DP_*/LS_X...), 完全不感知内核码与连接协议 (协议隔离在
device+mapping); 命令一律经 TeleopCommandSink 抽象下发 (依赖倒置, 不 import 服务层)。
关键安全实践 (对齐 AGENTS.md / Skill):
  - Dead-man: 未按住 RT(右扳机) 一切摇杆输入无效; RT 松开立即停当前点动;
  - 单轴点动: 任一时刻只从候选里取"主导轴"下发一条 MoveJog(带符号), 回中即停;
  - 互锁: 启动新点动前要求机器人处于 Idle (get_running_state()==0), Moving 拒绝并发;
  - Fail-Close: E-STOP / 断连 / 异常 -> 先立即关喷 DO, 再停机;
  - 内部只发高层命令, 量纲/封包由服务层与驱动层负责 (mm/rad -> deg)。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from .device import BaseTeleopInput, InputState
from .sink import TeleopCommandSink
from .state import (
    MODE_ORDER,
    MotionMode,
    SessionStatus,
    TeleopState,
    led_pattern,
)

# ---- 控制绑定 (逻辑名 -> 机器人轴/关节); 与协议无关, 换连接方式不受影响 ----
JOINT_SELECT: Dict[str, str] = {         # Joint 模式: 按住即动, 点按哪个键 = 哪个关节
    "DP_UP": "J1", "DP_DOWN": "J2", "DP_LEFT": "J3", "DP_RIGHT": "J4",
    "L3": "J5", "LB": "J6",
}
TRANSLATION_AXIS: Dict[str, str] = {     # 平移: 逻辑摇杆轴 -> 机器人笛卡尔轴 (上/右=正)
    "LS_Y": "X", "LS_X": "Y", "RS_Y": "Z", "RS_X": "Rz",
}
ROTATION_AXIS: Dict[str, str] = {        # 姿态: Rx/Ry/Rz
    "LS_X": "Rx", "LS_Y": "Ry", "RS_X": "Rz",
}
JOINT_DRIVE_AXIS = "RS_Y"                # Joint 模式用右摇杆上下给当前关节 ±


@dataclass
class TeleopConfig:
    deadzone: float = 0.12
    speed_tiers_pct: List[int] = field(default_factory=lambda: [10, 25, 50, 100])
    initial_tier_index: int = 1          # 默认 25%
    loop_hz: float = 50.0


class TeleopController:
    """把一个输入设备、一个命令 sink、一个玩家灯反馈装配成一台遥操作控制器。"""

    def __init__(
        self,
        device: BaseTeleopInput,
        sink: TeleopCommandSink,
        feedback=None,
        config: Optional[TeleopConfig] = None,
        is_connected: Optional[Callable[[], bool]] = None,
        is_robot_idle: Optional[Callable[[], bool]] = None,
        on_status: Optional[Callable[[SessionStatus], None]] = None,
    ) -> None:
        self._dev = device
        self._sink = sink
        self._led = feedback
        self._cfg = config or TeleopConfig()

        # 注入式探测: is_robot_idle 以控制器反馈为准; is_connected 为可选外部真实链路
        # (真机传入 svc.is_connected 以捕获被动掉线)。机器人连接改由 START 键主动 connect/disconnect。
        self._is_robot_idle = is_robot_idle or (lambda: True)
        self._ext_is_connected = is_connected
        self._on_status = on_status

        # ---- 会话状态 ----
        self.state = TeleopState.DISCONNECTED
        self.mode = MotionMode.TRANSLATION
        self._connected = False                            # 机器人链路: START 键 connect/disconnect 切换
        self.spray_on = False
        self._gripper_open = False                      # 夹爪张开/闭合 (R3 一键切换)
        self._paused = False
        self._tier_index = max(0, min(len(self._cfg.speed_tiers_pct) - 1,
                                      self._cfg.initial_tier_index))
        self._applied_tier: Optional[int] = None
        self._reduced = False                            # LT 低速是否生效 (供灯语/快照)
        self._active: Optional[Tuple[str, int]] = None   # 当前点动 (轴, 符号)
        self._joint_hold: Optional[str] = None           # Joint 按住选中的关节 (来自最后按下的键)

        self._prev_buttons: set[str] = set()
        self._press_order: List[str] = []                # Joint 选择优先级: 最近按下者优先
        self._running = False

    # ---- 生命周期 ---------------------------------------------------------
    def start(self) -> None:
        """打开手柄设备; 机器人链路初始为断开 (由 START 键连接/上电)。"""
        self._dev.open()
        if self._ext_is_connected is not None:
            self._connected = bool(self._ext_is_connected())
        self.state = TeleopState.IDLE if self._connected else TeleopState.DISCONNECTED
        self._running = True

    def stop(self) -> None:
        self._running = False
        self._safe_stop(spray_off=True)
        if self._led is not None:
            self._led.all_off()
        try:
            self._dev.close()
        except Exception:
            pass

    def run(self, max_seconds: Optional[float] = None) -> None:
        """阻塞主循环 (供测试脚本调用); 每 tick 读设备并处理, 看门狗保证 Fail-Close。"""
        dt = 1.0 / self._cfg.loop_hz
        t0 = time.monotonic()
        try:
            while self._running:
                now = time.monotonic()
                if max_seconds is not None and (now - t0) >= max_seconds:
                    break
                if not self._dev.alive():
                    self._enter_fault("input device lost (watchdog)")
                    break
                self._handle(self._dev.read(), now)
                time.sleep(dt)
        except KeyboardInterrupt:  # 退出前 Fail-Close
            pass
        finally:
            self.stop()

    # ---- 每拍处理 (纯逻辑, 可直接喂 InputState 做单测) --------------------
    def _handle(self, snap: InputState, now: float) -> None:
        if self._ext_is_connected is not None:            # 被动掉线以外部真实链路为准
            self._connected = bool(self._ext_is_connected())
        buttons = snap.buttons
        pressed_edges = buttons - self._prev_buttons
        released_edges = self._prev_buttons - buttons
        self._update_press_order(buttons, pressed_edges)

        # 系统键始终解析 (急停态下也要能按 X 清障); 运动/速率在非急停态才处理。
        self._handle_system_edges(pressed_edges)
        if self.state != TeleopState.E_STOP:
            self._handle_speed_edges(pressed_edges)
            self._update_speed(snap)
            self._update_motion(snap)

        self._prev_buttons = set(buttons)
        self._render_led(now)
        if self._on_status is not None:
            self._on_status(self.session_status())

    def _update_press_order(self, buttons: set[str], pressed_edges: set[str]) -> None:
        """维护关节选择键的"最近按下优先"序列 (按住即动时取末位)。"""
        self._press_order = [b for b in self._press_order if b in buttons]  # 丢弃已松开
        for b in pressed_edges:
            if b in JOINT_SELECT and b not in self._press_order:
                self._press_order.append(b)

    # ---- 系统与模式键 (按下边沿) ------------------------------------------
    def _handle_system_edges(self, edges: set[str]) -> None:
        if "Y" in edges:
            self._enter_estop()
            return
        if "X" in edges:
            self._sink.clear_error()
            self.state = TeleopState.IDLE if self._connected else TeleopState.DISCONNECTED
            return
        if "START" in edges:
            self._toggle_connection()
        if "BACK" in edges:
            self._cycle_mode()
        if "A" in edges:
            self._toggle_spray()
        if "B" in edges:
            self._toggle_pause()
        if "R3" in edges:
            self._toggle_gripper()
        if "CAPTURE" in edges:
            self._guarded_big_move(self._sink.go_home)
        if "HOME" in edges:
            self._guarded_big_move(self._sink.go_fold)

    def _toggle_connection(self) -> None:
        """START 键直接开关机械臂: 连接+上伺服 ↔ 断开 (复用服务层 connect/disconnect 流程)。

        - 未连接 -> sink.connect() 连接并使能伺服 (EnableRobot), 成功入 IDLE;
        - 已连接 -> 先安全停机并关喷, 再 sink.disconnect() 断链 (DisableRobot);
        - E-STOP 态忽略, 须先 X 清障。
        """
        if self.state == TeleopState.E_STOP:
            return
        if self._connected:
            self._safe_stop(spray_off=True)
            self._sink.disconnect()
            self._connected = False
            self.state = TeleopState.DISCONNECTED
        elif self._sink.connect():
            self._connected = True
            self.state = TeleopState.IDLE

    def _cycle_mode(self) -> None:
        idx = MODE_ORDER.index(self.mode)
        self.mode = MODE_ORDER[(idx + 1) % len(MODE_ORDER)]
        self._joint_hold = None
        self._stop_active()   # 切模式必停当前点动, 避免轴义漂移误动

    def _toggle_spray(self) -> None:
        if self.state in (TeleopState.DISCONNECTED, TeleopState.E_STOP):
            return
        self.spray_on = not self.spray_on
        self._sink.set_spray(self.spray_on)

    def _toggle_pause(self) -> None:
        self._paused = not self._paused
        self._sink.pause() if self._paused else self._sink.resume()

    def _toggle_gripper(self) -> None:
        """R3 一键切换夹爪全开/全闭 (夹爪与喷涂同为末端动作, 急停/未连接时不响应)。"""
        if self.state in (TeleopState.DISCONNECTED, TeleopState.E_STOP):
            return
        self._gripper_open = not self._gripper_open
        self._sink.gripper_open() if self._gripper_open else self._sink.gripper_close()

    def _guarded_big_move(self, fn: Callable[[], None]) -> None:
        """Home/Fold 需已连接且机器人 Idle (互锁); 否则拒绝。"""
        if self._connected and self._is_robot_idle():
            self._stop_active()
            fn()

    # ---- 速率 (RB=+, LB=-, 仅 Trans/Rot; Joint 模式肩键让位给关节选择) ----
    def _handle_speed_edges(self, edges: set[str]) -> None:
        if self.mode == MotionMode.JOINT:      # Joint 模式肩键让位给关节按住选择
            return
        if "RB" in edges:
            self._tier_index = min(len(self._cfg.speed_tiers_pct) - 1, self._tier_index + 1)
        if "LB" in edges:
            self._tier_index = max(0, self._tier_index - 1)

    def _update_speed(self, snap: InputState) -> None:
        reduced = snap.pressed("LT")
        desired = self._cfg.speed_tiers_pct[0] if reduced else self._cfg.speed_tiers_pct[self._tier_index]
        if desired != self._applied_tier:
            self._sink.set_speed_tier(desired)
            self._applied_tier = desired
        self._reduced = reduced

    # ---- 运动仲裁 (主导单轴 / 按住即动) -----------------------------------
    def _update_motion(self, snap: InputState) -> None:
        armed = snap.pressed("RT") and self._connected
        target = self._resolve_target(snap) if armed else None

        if target is None:
            if self._active is not None:
                self._stop_active()
            if not self._connected:
                self.state = TeleopState.DISCONNECTED
            else:
                self.state = TeleopState.ENABLED if snap.pressed("RT") else TeleopState.IDLE
            return

        axis, direction = target
        # 启动新点动前互锁: 机器人必须 Idle (允许停止, 不允许在 Running 时起新的)
        if not self._is_robot_idle():
            self.state = TeleopState.ENABLED
            return
        if self._active != (axis, direction):
            if self._active is not None:
                self._sink.jog_stop(self._active[0])
            self._sink.jog_start(axis, direction)
            self._active = (axis, direction)
        self.state = TeleopState.MOVING

    def _resolve_target(self, snap: InputState) -> Optional[Tuple[str, int]]:
        if self.mode == MotionMode.TRANSLATION:
            return self._dominant(snap, TRANSLATION_AXIS)
        if self.mode == MotionMode.ROTATION:
            # RT 已作 dead-man, LT 作低速超控, 不再占用二者做 Z 微调; Z 在 Translation 用右摇杆。
            return self._dominant(snap, ROTATION_AXIS)
        # JOINT: 按住选择 + 右摇杆上下给 ±
        return self._joint_target(snap)

    def _dominant(self, snap: InputState, axis_map: Dict[str, str]) -> Optional[Tuple[str, int]]:
        """从候选逻辑轴里取死后区绝对值最大的主导单轴, 返回 (机器人轴, 符号)。"""
        best_axis, best_mag, best_sign = None, self._cfg.deadzone, 0
        for logical, robot_axis in axis_map.items():
            v = snap.axis(logical)
            if abs(v) > best_mag:
                best_axis, best_mag, best_sign = robot_axis, abs(v), (1 if v > 0 else -1)
        return (best_axis, best_sign) if best_axis else None

    def _joint_target(self, snap: InputState) -> Optional[Tuple[str, int]]:
        joint = self._held_joint(snap)
        if joint is None:
            return None
        v = snap.axis(JOINT_DRIVE_AXIS)
        if abs(v) <= self._cfg.deadzone:
            return None
        return (joint, 1 if v > 0 else -1)

    def _held_joint(self, snap: InputState) -> Optional[str]:
        """按住即动: 取当前按下的关节键里"最近按下"者 (press_order 末位)。"""
        for logical in reversed(self._press_order):
            joint = JOINT_SELECT.get(logical)
            if joint and snap.pressed(logical):
                return joint
        return None

    # ---- 停止 / 故障 -------------------------------------------------------
    def _stop_active(self) -> None:
        if self._active is not None:
            self._sink.jog_stop(self._active[0])
            self._active = None
        if self.state == TeleopState.MOVING:
            self.state = TeleopState.IDLE if self._connected else TeleopState.DISCONNECTED

    def _enter_estop(self) -> None:
        self.state = TeleopState.E_STOP
        self._safe_stop(spray_off=True)
        self._sink.estop()   # 约定: estop 实现内部先立即关喷 DO 再停机

    def _enter_fault(self, reason: str) -> None:
        self.state = TeleopState.E_STOP
        self._safe_stop(spray_off=True)
        if self._led is not None:
            self._led.render(led_pattern(time.monotonic(), connected=False,
                                         state=self.state, mode=self.mode, fault=True))
        print(f"[teleop] FAULT: {reason}")

    def _safe_stop(self, spray_off: bool) -> None:
        if spray_off:
            self.spray_on = False
            self._sink.set_spray(False)   # 故障安全: 首要关喷
        self._stop_active()
        self._sink.stop_all_jogs()

    # ---- 灯语 / 快照 -------------------------------------------------------
    def _render_led(self, now: float) -> None:
        if self._led is None:
            return
        fault = self.state == TeleopState.E_STOP
        connected = self._dev.alive() and self._connected
        self._led.render(led_pattern(
            now, connected=connected, state=self.state, mode=self.mode,
            reduced=self._reduced, fault=fault, gripper_open=self._gripper_open,
        ))

    def session_status(self) -> SessionStatus:
        return SessionStatus(
            state=self.state,
            mode=self.mode,
            active_axis=self._active[0] if self._active else None,
            speed_pct=self._applied_tier
            if self._applied_tier is not None else self._cfg.speed_tiers_pct[self._tier_index],
            armed=self.state == TeleopState.MOVING,
            reduced=self._reduced,
            spray_on=self.spray_on,
            gripper_open=self._gripper_open,
        )
