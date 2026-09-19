# -*- coding: utf-8 -*-
"""协议映射层 (纯数据 + 归一化): 手柄原始内核码 <-> teleop 逻辑控件名。

协议隔离的落点 (换 USB 直连 / 蓝牙只改本层, 上层零改动):
  - 本模块把"某一连接协议档实测得到的码表"(单一真实源 = map_linux_24g.py)
    封装成一个 `ProtocolProfile`: 码->逻辑名、摇杆符号(使 上/右=正)、十字键方向、
    以及能力指纹(该协议档必须存在的 BTN/ABS 码集合)。
  - `device.py` 仅按 profile 把原始事件翻译成"逻辑名", `state/teleop` 层永远只认
    逻辑名(A/B/X/Y/LB/RB/DP_*/LS_X...), 不感知任何内核码与厂商协议差异。
  - 切换连接方式: 用 tests/manual_map_evdev.py 重新 --save 生成一份数据模块, 在
    `get_profile()` 里按连接方式返回对应 profile 即可。加载时 `verify_capabilities`
    会比对真实设备能力与指纹, 不一致(如误从 2.4G 档拔掉换 USB 键位错动)即拒绝启动,
    防止"错映射驱动机械臂"。

注意: Switch/8BitDo 档的内核 BTN_SOUTH/EAST/NORTH/WEST 与 Xbox 丝印对角互换
(A↔B、X↔Y), 该修正已在采集阶段(map_linux_24g.py)按物理真值落好, 本层直接信任逻辑名。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, FrozenSet, Iterable, Tuple

from .map_linux_24g import EVDEV_AXIS_MAP, EVDEV_BTN_MAP, EVDEV_HAT_MAP

# teleop 逻辑控件全集 —— 上层可引用的稳定名字; 换协议只改码表, 不改这些名字。
LOGICAL_BUTTONS: Tuple[str, ...] = (
    "A", "B", "X", "Y",
    "LB", "RB", "LT", "RT",
    "BACK", "START", "HOME", "CAPTURE",
    "L3", "R3",
    "DP_UP", "DP_DOWN", "DP_LEFT", "DP_RIGHT",
)
LOGICAL_AXES: Tuple[str, ...] = ("LS_X", "LS_Y", "RS_X", "RS_Y")

# 十字键以 HAT(ABS_HAT0X=16 / ABS_HAT0Y=17)出现, 其内核码不算"摇杆模拟轴"。
_HAT_AXIS_CODES: FrozenSet[int] = frozenset(c for (c, _v) in EVDEV_HAT_MAP.values())


@dataclass(frozen=True)
class ProtocolProfile:
    """一个连接协议档的完整映射与指纹 (不可变)。"""

    name: str
    btn_code_to_logical: Dict[int, str]
    axis_code_to_logical: Dict[int, str]           # 仅摇杆模拟轴 (排除 HAT / 数字扳机)
    hat_axis_codes: FrozenSet[int]                   # 属于十字键的 ABS 码 (16/17)
    hat_to_logical: Dict[Tuple[int, int], str]       # (axis_code, value) -> DP_*
    axis_sign: Dict[str, int]                        # 逻辑轴 -> 符号, 使"上/右 = 正"
    required_btn_codes: FrozenSet[int] = field(default_factory=frozenset)
    required_axis_codes: FrozenSet[int] = field(default_factory=frozenset)


def _invert_by_value(mapping: Dict[str, Tuple[str, int]]) -> Dict[int, str]:
    """{逻辑名: (类型, 码)} -> {码: 逻辑名}; 输入 map 的来源已隐含类型, 这里只翻键值。"""
    return {code: logical for logical, (_kind, code) in mapping.items()}


def build_switch_pro_profile() -> ProtocolProfile:
    """2.4G NS 接收器 / Nintendo Switch Pro 协议 (evdev + hid_nintendo) 的 profile。"""
    btn = _invert_by_value(EVDEV_BTN_MAP)                       # 数字键 (含 LT/RT: Switch 下为数字)
    axis = {c: lg for c, lg in _invert_by_value(EVDEV_AXIS_MAP).items()
            if c not in _HAT_AXIS_CODES}                        # 只保留 4 支摇杆模拟轴
    hat = {(code, val): logical for logical, (code, val) in EVDEV_HAT_MAP.items()}
    # 内核 ABS_Y / ABS_RY: 上推为负值 -> 取反使"上=正"; X 轴右推本就为正, 不动。
    sign = {"LS_X": 1, "LS_Y": -1, "RS_X": 1, "RS_Y": -1}
    return ProtocolProfile(
        name="linux_evdev_switch_pro_24g",
        btn_code_to_logical=btn,
        axis_code_to_logical=axis,
        hat_axis_codes=_HAT_AXIS_CODES,
        hat_to_logical=hat,
        axis_sign=sign,
        required_btn_codes=frozenset(btn.keys()),
        required_axis_codes=frozenset(axis.keys()),
    )


# 连接协议档注册表: 未来新增 USB(xpad)/蓝牙档时在此登记, 上层用名字取用。
_PROFILES = {
    "switch_pro": build_switch_pro_profile,
    "24g": build_switch_pro_profile,
    "ns": build_switch_pro_profile,
}


def get_profile(name: str = "switch_pro") -> ProtocolProfile:
    """按连接协议档名取得 profile; 未知名字直接失败 (fail-fast, 不静默回退错映射)。"""
    key = (name or "").strip().lower()
    factory = _PROFILES.get(key)
    if factory is None:
        raise ValueError(
            f"Unknown teleop protocol profile {name!r}; available: {sorted(set(_PROFILES))}"
        )
    return factory()


def verify_capabilities(
    profile: ProtocolProfile,
    device_btn_codes: Iterable[int],
    device_axis_codes: Iterable[int],
) -> Tuple[bool, str]:
    """设备真实能力指纹 vs profile 要求。缺键/缺轴即判不安全, 返回 (ok, 英文原因)。

    换连接模式(2.4G Switch <-> USB xpad)会让 BTN/ABS 码集发生结构性变化, 以此拦截,
    避免用错表点动机械臂。仅校验 profile 声明要用到的码是否存在 (允许多余码)。
    """
    have_btn = set(device_btn_codes)
    have_axis = set(device_axis_codes)
    miss_btn = sorted(profile.required_btn_codes - have_btn)
    miss_axis = sorted(profile.required_axis_codes - have_axis)
    if miss_btn or miss_axis:
        parts = []
        if miss_btn:
            parts.append(f"missing BTN codes {miss_btn}")
        if miss_axis:
            parts.append(f"missing ABS axis codes {miss_axis}")
        return False, (
            f"Device capability mismatch for profile '{profile.name}': "
            + "; ".join(parts)
            + ". Re-run manual_map_evdev.py for the current connection mode."
        )
    return True, ""
