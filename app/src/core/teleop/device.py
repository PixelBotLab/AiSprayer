# -*- coding: utf-8 -*-
"""设备层: 读 HID 原始事件 -> 归一化的逻辑输入快照 (InputState)。

职责边界 (驱动层, 不做业务判断):
  - 只负责打开一个手柄 evdev 节点、按 `ProtocolProfile` 把内核码翻译成"逻辑名",
    把摇杆归一化到 [-1.0, 1.0] 并应用符号(上/右=正), 维护按键/HAT 的按下集合;
  - 死区、方向仲裁、安全互锁一律不在这里 (属编排层 teleop.py);
  - evdev 仅在 LinuxGamepadInput 内部延迟导入, 保证 mapping/state/sink 在无 evdev
    的开发机上也能 import (便于纯逻辑单测)。

协议隔离: 换连接方式只换 profile(见 mapping.get_profile), 本文件逻辑不变。
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Dict, Optional, Set

from .mapping import ProtocolProfile, verify_capabilities

_AXIS_FULL_SCALE = 32767.0  # Switch Pro / 常见摇杆 EV_ABS 满量程 (±32767)


@dataclass
class InputState:
    """某一时刻的逻辑输入快照 (深拷贝, 供无锁消费)。"""

    buttons: Set[str] = field(default_factory=set)          # 当前按下的逻辑键名 (含 DP_*)
    axes: Dict[str, float] = field(default_factory=dict)     # 逻辑轴 -> [-1,1] 已应用符号
    seq: int = 0                                             # 单调递增, 便于判新

    def pressed(self, name: str) -> bool:
        return name in self.buttons

    def axis(self, name: str) -> float:
        return self.axes.get(name, 0.0)

    def snapshot(self) -> "InputState":
        return InputState(buttons=set(self.buttons), axes=dict(self.axes), seq=self.seq)


class BaseTeleopInput:
    """遥操作输入设备抽象 (开闭原则: 新增 pygame/inputs 后端只需实现本接口)。"""

    name: str = ""

    def open(self) -> None:  # pragma: no cover - 抽象
        raise NotImplementedError

    def read(self) -> InputState:  # pragma: no cover - 抽象
        raise NotImplementedError

    def alive(self) -> bool:  # 供看门狗判活 (读线程/设备是否仍在工作)
        return True

    def close(self) -> None:  # pragma: no cover - 抽象
        pass


class TeleopDeviceError(RuntimeError):
    """设备缺失 / 能力指纹不符 / 权限不足等启动期错误。"""


class LinuxGamepadInput(BaseTeleopInput):
    """Linux evdev 后端 (生产目标 RK3588)。内部线程读循环, read() 取最新快照。"""

    def __init__(self, profile: ProtocolProfile, path: Optional[str] = None,
                 name_substr: Optional[str] = None) -> None:
        self._profile = profile
        self._path = path
        self._name_substr = name_substr
        self._dev = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._state = InputState()
        self._hat: Dict[int, int] = {}          # HAT 轴码 -> 当前离散值 (-1/0/1)
        self.name = ""
        self.connected = False

    # ---- 生命周期 ---------------------------------------------------------
    def open(self) -> None:
        try:
            import evdev  # 延迟导入: 仅真机需要
            from evdev import InputDevice
        except Exception as e:  # pragma: no cover - 环境缺依赖
            raise TeleopDeviceError(f"evdev not available: {e}") from e

        dev = self._select_device(evdev, InputDevice)
        if dev is None:
            raise TeleopDeviceError(
                "No matching gamepad evdev node. Check receiver/mode, or pass --path."
            )
        self._dev = dev
        self.name = dev.name

        # 能力指纹校验: 设备实际 BTN/ABS 必须覆盖 profile 要求, 否则拒绝 (防错映射驱动机械臂)
        caps = dev.capabilities(absinfo=False)  # absinfo=False: EV_ABS 返回纯 code 而非 (code, AbsInfo) 元组
        btn_codes = {c for c in caps.get(evdev.ecodes.EV_KEY, []) if 300 <= c < 400}
        abs_codes = {c for c in caps.get(evdev.ecodes.EV_ABS, []) if c not in self._profile.hat_axis_codes}
        ok, why = verify_capabilities(self._profile, btn_codes, abs_codes)
        if not ok:
            dev.close()
            self._dev = None
            raise TeleopDeviceError(why)

        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="teleop-evdev", daemon=True)
        self._thread.start()
        self.connected = True

    def _select_device(self, evdev, InputDevice):
        from evdev import ecodes
        for p in evdev.list_devices():
            try:
                d = InputDevice(p)
            except (PermissionError, OSError):
                continue
            btns = d.capabilities(absinfo=False).get(ecodes.EV_KEY, [])
            looks_like_pad = any(300 <= b < 400 for b in btns)
            if not looks_like_pad:
                d.close()
                continue
            if self._path and p != self._path:
                d.close()
                continue
            if self._name_substr and self._name_substr.lower() not in d.name.lower():
                d.close()
                continue
            return d
        return None

    # ---- 后台读循环 -------------------------------------------------------
    def _run(self) -> None:  # pragma: no cover - 需真机事件流
        from evdev import ecodes
        try:
            for ev in self._dev.read_loop():
                if self._stop.is_set():
                    break
                if ev.type == ecodes.EV_KEY and 300 <= ev.code < 400:
                    self._on_button(ev.code, ev.value)
                elif ev.type == ecodes.EV_ABS:
                    self._on_abs(ev.code, ev.value)
        except OSError:
            # 设备拔插/休眠掉线: 标记失联, 交由上层看门狗触发 Fail-Close 停止
            self.connected = False

    def _on_button(self, code: int, value: int) -> None:
        logical = self._profile.btn_code_to_logical.get(code)
        if logical is None:
            return
        with self._lock:
            if value:  # 1=按下, 2=按住重复(部分驱动), 0=松开
                self._state.buttons.add(logical)
            else:
                self._state.buttons.discard(logical)
            self._state.seq += 1

    def _on_abs(self, code: int, value: int) -> None:
        if code in self._profile.hat_axis_codes:
            with self._lock:
                self._hat[code] = value
                self._refresh_hat()
                self._state.seq += 1
            return
        logical = self._profile.axis_code_to_logical.get(code)
        if logical is None:
            return
        sign = self._profile.axis_sign.get(logical, 1)
        norm = max(-1.0, min(1.0, (value / _AXIS_FULL_SCALE) * sign))
        with self._lock:
            self._state.axes[logical] = norm
            self._state.seq += 1

    def _refresh_hat(self) -> None:
        """按当前 HAT 两轴离散值重算 DP_* 按下集合 (对角时同时点亮两个方向)。"""
        for (axis_code, axis_val), logical in self._profile.hat_to_logical.items():
            if axis_val == 0:
                continue
            cur = self._hat.get(axis_code, 0)
            if cur == axis_val:
                self._state.buttons.add(logical)
            else:
                self._state.buttons.discard(logical)

    # ---- 消费接口 ---------------------------------------------------------
    def read(self) -> InputState:
        with self._lock:
            return self._state.snapshot()

    def alive(self) -> bool:
        return self.connected and (self._thread is None or self._thread.is_alive())

    def close(self) -> None:
        self._stop.set()
        self.connected = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            if self._dev is not None:
                self._dev.close()
        except Exception:
            pass
        self._dev = None
