# -*- coding: utf-8 -*-
"""
手柄按键映射自动采集工具 (Linux evdev 后端, standalone)。

用途: 在真机(RK3588/Linux)上,针对**具体连接模式**(2.4G 接收器 / USB 直连 / 某
种手柄档位),实测并生成一张 "物理键 → 逻辑名 → 内核 BTN_*/ABS_* 码" 的对照表,
直接作为 teleop `mapping.py`(evdev 后端)的单一真实源。

为什么用 evdev 而不是 pygame/SDL:
  - 内核 HID 驱动(如 Switch Pro 的 `hid_nintendo`)已经在事件里带了**语义名**
    (BTN_SOUTH / BTN_TL2 / ABS_HAT0X ...),逻辑归属几乎零歧义;
  - 规避 SDL 在不同 OS/模式/手柄下重排 button index、把 d-pad 在 button↔hat 之间
    来回搬的问题(这正是 USB 直连与 2.4G 下 `--map` 标注对不上的根因);
  - 生产部署本就规划走 evdev(见 docs/teleop_design.md §2、M5),此表口径与线上一致。

流程:
  1) 自动探测(或 --path/--name 指定)一个手柄 evdev 节点,打印其能力总表;
  2) 进入实时采集: 把每个物理键/摇杆/十字键/扳机各操作一遍,终端逐条打印原始事件,
     并记录 "哪些码被触发过";
  3) Ctrl+C 结束后输出: 覆盖对照表、未触发(疑似死键)项、轴/HAT 行程,
     以及一段可直接粘贴进 mapping.py 的 Python 字典。

用法:
    cd app
    # 依赖: python -m pip install evdev
    PYTHONPATH=src .venv/bin/python src/core/teleop/tests/manual_map_evdev.py --list
    PYTHONPATH=src .venv/bin/python src/core/teleop/tests/manual_map_evdev.py            # 自动选第一个手柄, 采集
    PYTHONPATH=src .venv/bin/python src/core/teleop/tests/manual_map_evdev.py --wait      # 等手柄上线(切模式/唤醒)再采集
    PYTHONPATH=src .venv/bin/python src/core/teleop/tests/manual_map_evdev.py --path /dev/input/event9
    PYTHONPATH=src .venv/bin/python src/core/teleop/tests/manual_map_evdev.py --save map_linux_24g.py

权限: 读 /dev/input/* 通常需在 input 组; 若 PermissionDenied, 执行 `sudo usermod -aG input $USER` 后重登。

命名以 `manual_` 开头, pytest 不会收集它。
"""

from __future__ import annotations

import argparse
import sys
import time

# ---- ANSI 轻量着色(终端不支持也不报错) ----
_DIM = "\033[2m"
_RED = "\033[31m"
_GREEN = "\033[32m"
_YELLOW = "\033[33m"
_CYAN = "\033[36m"
_BOLD = "\033[1m"
_RESET = "\033[0m"

# 内核 BTN_* 语义名 → teleop 逻辑名 (§4 键位体系的中间层)。
# 覆盖常见手柄(标准 Linux gamepad / hid_nintendo Switch Pro / xpad Xbox)的命名别名。
BTN_NAME_TO_LOGICAL = {
    "BTN_A": "A", "BTN_SOUTH": "A",
    "BTN_B": "B", "BTN_EAST": "B",
    "BTN_X": "X", "BTN_WEST": "X",
    "BTN_Y": "Y", "BTN_NORTH": "Y",
    "BTN_TL": "LB", "BTN_TR": "RB",          # 肩键(数字)
    "BTN_TL2": "LT", "BTN_TR2": "RT",        # 扳机; Switch 模式下为数字, xpad 模式下可能是轴
    "BTN_SELECT": "BACK", "BTN_MINUS": "BACK",
    "BTN_START": "START", "BTN_PLUS": "START",
    "BTN_MODE": "HOME", "BTN_GUIDE": "HOME",
    "BTN_THUMBL": "L3", "BTN_THUMBR": "R3",
}
# 内核 ABS_* 语义名 → teleop 逻辑名(摇杆/模拟扳机)。
ABS_NAME_TO_LOGICAL = {
    "ABS_X": "LS_X", "ABS_Y": "LS_Y",        # 左摇杆
    "ABS_RX": "RS_X", "ABS_RY": "RS_Y",     # 右摇杆(现代主流布局: Switch Pro / xpad)
    "ABS_Z": "LT", "ABS_RZ": "RT",          # 模拟扳机(xpad 等以 Z/RZ 上报 LT/RT)
    "ABS_GAS": "RT", "ABS_BRAKE": "LT",
    "ABS_HAT0X": "DP_X", "ABS_HAT0Y": "DP_Y",  # 十字键(以 HAT 形式出现的轴对)
}
# 摇杆轴(用于漂移/行程统计, 区别于 HAT 的 ±1 离散值)
_STICK_AXES = {"LS_X", "LS_Y", "RS_X", "RS_Y"}


def _require_evdev():
    try:
        import evdev  # noqa: F401
        from evdev import InputDevice, ecodes  # noqa: F401
        return evdev
    except Exception as e:  # pragma: no cover - dev-time helper
        sys.stderr.write(
            f"{_YELLOW}未检测到 evdev:{_RESET}{e}\n"
            f"  安装:{_BOLD}python -m pip install evdev{_RESET}\n"
        )
        raise SystemExit(2)


def _btn_name(code: int) -> str:
    from evdev import ecodes
    names = ecodes.BTN.get(code)
    if isinstance(names, (list, tuple)):
        return names[0] if names else str(code)
    return str(names) if names else str(code)


def _abs_name(code: int) -> str:
    from evdev import ecodes
    return str(ecodes.ABS.get(code, code))


# 按设备族覆盖"内核码 → 逻辑名"。实测(2.4G Switch 档): 8BitDo 等以 Xbox 丝印(A/B/X/Y)
# 却走 Nintendo Switch 协议的手柄, 内核 BTN_SOUTH/EAST/NORTH/WEST 命名与丝印字母
# 对角相反(A↔B、X↔Y 成对互换), 且 Capture(□)键上报为 BTN_Z(309)。按物理实测覆盖
# generic 内核名猜测; 若为原版 Nintendo 壳(字母布局不同)请重新实测核对。
_BTN_CODE_OVERRIDES: dict[int, str] = {}
_SWITCH_FAMILY_CODES = {304: "B", 305: "A", 307: "X", 308: "Y", 309: "CAPTURE"}


def _apply_device_overrides(dev_name: str) -> None:
    global _BTN_CODE_OVERRIDES
    _BTN_CODE_OVERRIDES = {}
    low = dev_name.lower()
    if "switch pro" in low or "8bitdo" in low or "nintendo" in low:
        _BTN_CODE_OVERRIDES = dict(_SWITCH_FAMILY_CODES)


def _logical_for_btn(code: int) -> str:
    from evdev import ecodes
    if code in _BTN_CODE_OVERRIDES:
        return _BTN_CODE_OVERRIDES[code]
    names = ecodes.BTN.get(code, [])
    names = names if isinstance(names, (list, tuple)) else [names]
    for n in names:
        if n in BTN_NAME_TO_LOGICAL:
            return BTN_NAME_TO_LOGICAL[n]
    return "?"


def _logical_for_abs(code: int) -> str:
    from evdev import ecodes
    n = _abs_name(code)
    return ABS_NAME_TO_LOGICAL.get(n, "?")


def _iter_gamepad_paths(evdev):
    """枚举所有像手柄的 evdev 节点(携带 BTN_JOYSTICK~BTN_MISC 区间的按键)。"""
    from evdev import ecodes, InputDevice
    out = []
    for path in evdev.list_devices():
        try:
            dev = InputDevice(path)
        except (PermissionError, OSError):
            continue
        btns = dev.capabilities().get(ecodes.EV_KEY, [])
        if any(300 <= b < 400 for b in btns):
            out.append((path, dev.name, dev.phys))
        dev.close()
    return out


def _open_device(evdev, path=None, name_sub=None):
    """按 --path / --name 或自动挑选打开一个手柄, 返回 InputDevice; 找不到返回 None。"""
    from evdev import InputDevice
    cands = _iter_gamepad_paths(evdev)
    if not cands:
        return None
    if path:
        for p, nm, ph in cands:
            if p == path:
                return InputDevice(p)
        # path 显式给定但不像手柄, 仍尝试直接打开(容错)
        try:
            return InputDevice(path)
        except Exception:
            return None
    if name_sub:
        for p, nm, ph in cands:
            if name_sub.lower() in nm.lower():
                return InputDevice(p)
        return None
    # 自动: 优先名字里含 Controller/Pad/Gamepad 的
    cands.sort(key=lambda c: 0 if any(
        k in c[1].lower() for k in ("controller", "gamepad", "pad")) else 1)
    return InputDevice(cands[0][0])


def list_devices(evdev) -> None:
    cands = _iter_gamepad_paths(evdev)
    if not cands:
        print(f"{_YELLOW}未发现手柄 evdev 节点。{_RESET}检查接收器/手柄是否唤醒(按任意键)、是否已切到目标模式。")
        return
    print(f"{_BOLD}检测到手柄类输入设备 {len(cands)} 个:{_RESET}")
    for p, nm, ph in cands:
        print(f"  {_CYAN}{p}{_RESET}  {nm}   {_DIM}phys={ph}{_RESET}")


def _dump_caps(dev) -> dict:
    """打印设备能力总表(内核码 → 语义名 → 逻辑名), 返回 {btn_codes, abs_codes}。"""
    from evdev import ecodes
    # absinfo=False: 让 EV_ABS 返回纯 code 整数, 而非 (code, AbsInfo) 元组(默认元组会让下面格式化崩溃)
    caps = dev.capabilities(absinfo=False)
    btns = sorted(set(caps.get(ecodes.EV_KEY, [])))
    btns = [b for b in btns if 300 <= b < 400]
    axes = sorted({a[0] if isinstance(a, tuple) else a for a in caps.get(ecodes.EV_ABS, [])})
    print(f"{_BOLD}设备:{_CYAN}{dev.name}{_RESET}  {_DIM}{dev.path}{_RESET}")
    print(f"{_BOLD}按键能力({len(btns)}):{_RESET}")
    for c in btns:
        print(f"    BTN {c:>3}  {_btn_name(c):<16} → {_logical_for_btn(c)}")
    print(f"{_BOLD}轴能力({len(axes)}):{_RESET}")
    for c in axes:
        print(f"    ABS {c:>3}  {_abs_name(c):<12} → {_logical_for_abs(c)}")
    return {"btn_codes": btns, "abs_codes": axes}


def _print_checklist():
    print(f"{_BOLD}实时采集:{_RESET} 请把每个控件各操作一遍(看上方能力表逐个对照):")
    print("    主键 A B X Y | 肩键 LB RB | 扳机 LT RT | 功能 BACK/START/HOME | 摇杆按下 L3 R3")
    print("    左/右摇杆各方向推到底画圈 | 十字键 上 下 左 右")
    print(f"    {_DIM}每条原始事件会实时打印; 全部操作完按 Ctrl+C 生成对照表与 mapping 字典。{_RESET}\n")


def capture(dev) -> int:
    """实时采集事件直到 Ctrl+C, 记录触发过的码, 最后出表。"""
    from evdev import ecodes
    _apply_device_overrides(dev.name)
    caps = _dump_caps(dev)
    print()
    _print_checklist()

    pressed: set[int] = set()
    axis_seen: set[int] = set()
    axis_min: dict[int, int] = {}
    axis_max: dict[int, int] = {}
    hat_states: set[tuple[int, int]] = set()
    hat_val = {ecodes.ABS_HAT0X: 0, ecodes.ABS_HAT0Y: 0}

    try:
        for ev in dev.read_loop():
            if ev.type == ecodes.EV_KEY and 300 <= ev.code < 400:
                if ev.value >= 1:  # 1=按下, 2=按住重复
                    pressed.add(ev.code)
                    print(f"{_GREEN}▶ BTN {_BOLD}{ev.code:>3}{_RESET}{_GREEN} PRESS   {_btn_name(ev.code):<16} → {_logical_for_btn(ev.code)}{_RESET}")
            elif ev.type == ecodes.EV_ABS:
                axis_seen.add(ev.code)
                axis_min[ev.code] = min(axis_min.get(ev.code, ev.value), ev.value)
                axis_max[ev.code] = max(axis_max.get(ev.code, ev.value), ev.value)
                lg = _logical_for_abs(ev.code)
                if ev.code in (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
                    hat_val[ev.code] = ev.value
                    # 任一 HAT 轴变化都合成打印(修: 左右仅改 HAT0X, 旧逻辑只认 Y 会漏报)
                    hx, hy = hat_val[ecodes.ABS_HAT0X], hat_val[ecodes.ABS_HAT0Y]
                    hat_states.add((hx, hy))
                    print(f"{_YELLOW}▶ HAT  ({hx:+d},{hy:+d})  {_hat_label(hx, hy)}{_RESET}")
                else:
                    print(f"{_CYAN}▶ AXIS {ev.code:>3} value={ev.value:+6d}  {_abs_name(ev.code):<10} → {lg}{_RESET}")
    except (KeyboardInterrupt, OSError):
        print("\n")
    return _report(dev, caps, pressed, axis_seen, axis_min, axis_max, hat_states)


def _hat_label(hx: int, hy: int) -> str:
    m = {
        (0, -1): "DP_UP", (0, 1): "DP_DOWN", (-1, 0): "DP_LEFT", (1, 0): "DP_RIGHT",
        (1, -1): "DP_UP+RIGHT", (-1, -1): "DP_UP+LEFT", (1, 1): "DP_DOWN+RIGHT",
        (-1, 1): "DP_DOWN+LEFT", (0, 0): "DP_CENTER",
    }
    return m.get((hx, hy), "DP_?")


def _report(dev, caps, pressed, axis_seen, axis_min, axis_max, hat_states) -> int:
    from evdev import ecodes
    print(f"{_BOLD}===== 对照表:{dev.name} ====={_RESET}")

    print(f"\n{_BOLD}按键覆盖(✓=已触发, ·=未触发/死键候选):{_RESET}")
    missing = []
    for c in caps["btn_codes"]:
        ok = c in pressed
        if not ok:
            missing.append(c)
        mark = f"{_GREEN}✓{_RESET}" if ok else f"{_RED}·{_RESET}"
        print(f"    {mark} BTN {c:>3}  {_btn_name(c):<16} → {_logical_for_btn(c)}")

    print(f"\n{_BOLD}轴覆盖 / 行程:{_RESET}")
    dead_axes = []
    for c in caps["abs_codes"]:
        lg = _logical_for_abs(c)
        if c not in axis_seen:
            dead_axes.append(c)
            print(f"    {_RED}·{_RESET} ABS {c:>3}  {_abs_name(c):<10} → {lg:<8} {_RED}未移动/死轴{_RESET}")
            continue
        lo, hi = axis_min[c], axis_max[c]
        if lg in _STICK_AXES:
            span = (lo / 32767.0, hi / 32767.0)
            state = _GREEN + "OK" + _RESET if (span[0] < -0.85 and span[1] > 0.85) else _YELLOW + "行程不全?" + _RESET
            print(f"    {_GREEN}✓{_RESET} ABS {c:>3}  {_abs_name(c):<10} → {lg:<8} [{span[0]:+.2f} → {span[1]:+.2f}]  {state}")
        elif c in (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
            print(f"    {_GREEN}✓{_RESET} ABS {c:>3}  {_abs_name(c):<10} → {lg:<8} (十字键 HAT, 见下方方向)")
        else:
            print(f"    {_GREEN}✓{_RESET} ABS {c:>3}  {_abs_name(c):<10} → {lg:<8} [{lo} → {hi}]")

    if hat_states:
        dirs = sorted(d for d in hat_states if d != (0, 0))
        print(f"\n{_BOLD}十字键 HAT 触发方向:{_RESET} " + ", ".join(_hat_label(*d) for d in dirs))

    # 结论
    print(f"\n{_BOLD}===== 结论 ====={_RESET}")
    issues = []
    if missing:
        issues.append(f"未触发 BTN={missing}")
    if dead_axes:
        issues.append(f"未触发 ABS={dead_axes}")
    if not pressed and not axis_seen:
        print(f"    {_YELLOW}未收到任何输入 —— 节点选错了? 用 --list / --path 指定正确手柄。{_RESET}")
        return 1
    if issues:
        print(f"    {_YELLOW}⚠ 待确认(可能只是没按到): {'; '.join(issues)}{_RESET}")
    else:
        print(f"    {_GREEN}{_BOLD}✅ 所有已声明的按键/轴均有响应。{_RESET}")

    _emit_mapping(caps, hat_states)
    return 0


# 最近一次 _emit_mapping 生成的映射文本, 供 --save 落盘。
_LAST_MAPPING_BLOCK = ""


def _emit_mapping(caps, hat_states) -> str:
    """导出可直接粘贴进 mapping.py(evdev 后端)的逻辑名 → 原始码 字典。"""
    global _LAST_MAPPING_BLOCK
    print(f"\n{_BOLD}===== 粘贴进 mapping.py 的映射(单一真实源, 本平台/模式实测) ====={_RESET}")
    lines = ["# Linux evdev backend mapping — auto-generated by manual_map_evdev.py",
             "# {逻辑名: (类型, 内核码)}  类型: 'btn'=EV_KEY, 'abs'=EV_ABS",
             "EVDEV_BTN_MAP = {"]
    for c in caps["btn_codes"]:
        lg = _logical_for_btn(c)
        if lg == "?":
            continue
        lines.append(f"    {lg!r}: ('btn', {c}),  # {_btn_name(c)}")
    lines.append("}")
    lines.append("EVDEV_AXIS_MAP = {")
    for c in caps["abs_codes"]:
        lg = _logical_for_abs(c)
        if lg in ("?", "DP_X", "DP_Y"):
            continue
        lines.append(f"    {lg!r}: ('abs', {c}),  # {_abs_name(c)}")
    lines.append("}")
    lines.append("# 十字键以 HAT(ABS_HAT0X=16 / ABS_HAT0Y=17)离散值出现; Y 轴 -1=上/ +1=下(屏幕坐标), 按下判定用方向符号:")
    lines.append("EVDEV_HAT_MAP = {'DP_UP': (17, -1), 'DP_DOWN': (17, 1), "
                 "'DP_LEFT': (16, -1), 'DP_RIGHT': (16, 1)}  # (axis_code, value)")
    block = "\n".join(lines)
    _LAST_MAPPING_BLOCK = block
    print(_DIM + block + _RESET)
    return block


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="手柄按键映射自动采集(Linux evdev)")
    parser.add_argument("--list", action="store_true", help="仅列出检测到的手柄 evdev 节点后退出")
    parser.add_argument("--path", help="指定 evdev 节点(如 /dev/input/event9)")
    parser.add_argument("--name", help="按设备名子串匹配手柄")
    parser.add_argument("--wait", action="store_true", help="等待手柄上线(切模式/唤醒/插拔后自动继续)")
    parser.add_argument("--save", metavar="PATH", help="采集结束后把生成的映射字典写入指定 .py 文件")
    args = parser.parse_args(argv)

    evdev = _require_evdev()
    if args.list:
        list_devices(evdev)
        return 0

    dev = _open_device(evdev, args.path, args.name)
    if dev is None and args.wait:
        print(f"{_DIM}等待手柄上线…(按任意键唤醒/切换模式), Ctrl+C 退出{_RESET}")
        try:
            while dev is None:
                time.sleep(1.0)
                dev = _open_device(evdev, args.path, args.name)
        except KeyboardInterrupt:
            return 0
    if dev is None:
        print(f"{_YELLOW}未发现手柄。{_RESET}先 --list 看节点, 或加 --wait, 或检查模式/唤醒。")
        return 1

    try:
        rc = capture(dev)
        if args.save and _LAST_MAPPING_BLOCK:
            with open(args.save, "w", encoding="utf-8") as f:
                f.write(_LAST_MAPPING_BLOCK + "\n")
            print(f"{_GREEN}已写入映射:{_RESET}{args.save}")
        return rc
    finally:
        try:
            dev.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
