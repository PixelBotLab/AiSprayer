# -*- coding: utf-8 -*-
"""离屏反馈: 通过 sysfs 玩家灯渲染状态 (Switch/8BitDo 手柄)。

- 内核 hid_nintendo 把 5 颗玩家灯暴露为 /sys/class/leds/0003:*:{green|blue}:player-N/
  (4 绿 player-1..4 + 1 蓝 player-5), max_brightness=1 (仅亮/灭); 经 udev `RUN+=chmod`
  免 sudo 可写。闪烁不靠内核 timer, 而由上层每周期喂 led_pattern() 结果实现 (见 state.py);
- 灯仅作只读反馈: 找不到节点 / 无写权限时安全降级为空操作, 绝不影响安全逻辑;
- 手柄会因休眠频繁重连, 实例号每次变 (如 .0009 -> .000C), 故按 *:player-N 通配动态解析。
"""

from __future__ import annotations

import glob
import os
from typing import Dict, Optional

from .state import LED_NAMES


class PlayerLedFeedback:
    """把逻辑灯名 (player-1..5) 映射到 sysfs brightness 节点并写入。"""

    def __init__(self, base_dir: str = "/sys/class/leds", enabled: bool = True) -> None:
        self._base = base_dir
        self._enabled = enabled
        self._paths: Dict[str, Optional[str]] = {name: None for name in LED_NAMES}
        self._last: Dict[str, int] = {name: -1 for name in LED_NAMES}
        if self._enabled:
            self.refresh()

    @property
    def available_count(self) -> int:
        """成功解析到的可控灯数 (0 表示这台环境没有灯, 全部空操作)。"""
        return sum(1 for p in self._paths.values() if p)

    def refresh(self) -> None:
        """(重)扫描玩家灯节点; 重连后实例号变化时调用。"""
        if not self._enabled:
            return
        for name in LED_NAMES:
            matches = sorted(glob.glob(os.path.join(self._base, f"*:{name}")))
            self._paths[name] = os.path.join(matches[0], "brightness") if matches else None
            self._last[name] = -1  # 强制下次重写

    def render(self, pattern: Dict[str, int]) -> None:
        """按 {灯名: 0/1} 写 brightness; 缺失/失败静默忽略。"""
        if not self._enabled:
            return
        for name, value in pattern.items():
            path = self._paths.get(name)
            if path is None or self._last.get(name) == value:
                continue  # 无节点或值未变 -> 免写
            try:
                with open(path, "w") as f:
                    f.write("1" if value else "0")
                self._last[name] = value
            except OSError:
                # 权限不足或掉线: 只读反馈失效不影响控制; 记下路径待重连 refresh
                self._paths[name] = self._paths.get(name)

    def all_off(self) -> None:
        """退出/断连时熄灭所有灯。"""
        for name in LED_NAMES:
            path = self._paths.get(name)
            if path is None:
                continue
            try:
                with open(path, "w") as f:
                    f.write("0")
                self._last[name] = 0
            except OSError:
                pass
