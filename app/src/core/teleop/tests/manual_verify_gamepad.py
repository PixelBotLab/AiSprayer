# -*- coding: utf-8 -*-
"""
手柄硬件体检工具 / Manual gamepad hardware health-check (standalone).

用途：在真机上验证一只手柄（尤其是**二手手柄**）的每个按键 / 摇杆 / 扳机 /
十字键是否工作正常——检测【死键】【摇杆漂移】【轴行程不全/扳机不线性】等常见故障。

本脚本是**独立工具**，不依赖尚未实现的 core.teleop 运行时模块；仅用 pygame(SDL2)
读取 HID。命名以 `manual_` 开头，pytest 不会收集它。

三阶段流程（verify 默认模式）：
  阶段1  漂移基线：请松开所有摇杆与扳机静置，采样各轴“回中”读数，超阈值判为疑似漂移。
  阶段2  逐项操作：按提示逐个按键/拨杆/推摇杆画圈，实时打印并统计“哪些被触发过”。
  阶段3  健康报告：汇总未触发的按键(死键)、未移动/单侧的轴、各轴行程 min/max、漂移结果，给出结论。

平台与后端：
  - macOS：用 pygame（pip 自带 SDL2）。8BitDo 需切到 macOS 可原生读取的模式
    （Switch/Pro 或有线直连）；2.4G 接收器在 macOS 呈 Xbox360 协议、无驱动读不到。
    iOS/MFi 档只走系统 GCController，SDL 读不到。
  - Linux / RK3588：同样可用 pygame；生产部署改用 evdev 后端（M5，见设计文档）。

用法：
    cd app
    # 依赖：python -m pip install "pygame-ce"   （或 pip install pygame）
    PYTHONPATH=src python src/core/teleop/tests/manual_verify_gamepad.py --list
    PYTHONPATH=src python src/core/teleop/tests/manual_verify_gamepad.py            # 默认第 0 个手柄，跑三阶段体检
    PYTHONPATH=src python src/core/teleop/tests/manual_verify_gamepad.py --index 1 --deadzone 0.15
    PYTHONPATH=src python src/core/teleop/tests/manual_verify_gamepad.py --watch    # 监视切模式/插拔瞬间出现的新手柄
    PYTHONPATH=src python src/core/teleop/tests/manual_verify_gamepad.py --map      # 交互自检时把原始 index 标注成逻辑按键名

退出：按 Ctrl+C（阶段2 随时可退出并给出已完成部分的健康报告）。
"""

from __future__ import annotations

import argparse
import sys
import time

# ---- ANSI 轻量着色（终端不支持也不报错）----
_DIM = "\033[2m"
_RED = "\033[31m"
_GREEN = "\033[32m"
_YELLOW = "\033[33m"
_CYAN = "\033[36m"
_BOLD = "\033[1m"
_RESET = "\033[0m"

# 静置时超过该 |轴值| 判定摇杆疑似漂移（二手手柄最常见故障）
DRIFT_THRESHOLD = 0.10
# 前几个轴通常对应左右摇杆 X/Y（不同手柄请以实测为准）
STICK_AXIS_COUNT = 4


def _require_pygame():
    try:
        import pygame  # noqa: F401
        return pygame
    except Exception as e:  # pragma: no cover - dev-time helper
        sys.stderr.write(
            f"{_YELLOW}未检测到 pygame：{_RESET}{e}\n"
            f"  安装：{_BOLD}python -m pip install 'pygame-ce'{_RESET}  （或 pip install pygame）\n"
        )
        raise SystemExit(2)


def _bar(value: float, width: int = 21) -> str:
    """把 [-1,1] 的轴值画成居中的字符条。"""
    value = max(-1.0, min(1.0, value))
    pos = int(round((value + 1.0) / 2.0 * (width - 1)))
    cells = ["│"] * width
    cells[pos] = "█"
    mid = width // 2
    cells[mid] = "┼" if pos != mid else "╪"
    return "".join(cells)


# ---- §4.5 逻辑按键名映射（实测：8BitDo 猎户座2 / Ultimate 2，NS·Switch 模式，macOS/SDL2）----
# 注：不同手柄/模式/后端索引各不同；本设备 button 7/8 未用，背键 L4/R4/PL/PR 与 L3/R3 在 Switch 协议不上报。
_BTN_LOGICAL = {
    0: "A", 1: "B", 2: "X", 3: "Y",
    4: "BACK(−)", 5: "HOME", 6: "START(+)",
    9: "LB(L)", 10: "RB(R)",
    11: "DP_UP", 12: "DP_DOWN", 13: "DP_LEFT", 14: "DP_RIGHT",
    15: "CAPTURE",
}
_AXIS_LOGICAL = {0: "LS_X", 1: "LS_Y", 2: "RS_X", 3: "RS_Y", 4: "LT(ZL)", 5: "RT(ZR)"}


def _btn_logical(i: int) -> str:
    return _BTN_LOGICAL.get(i, "?")


def _ax_logical(i: int) -> str:
    return _AXIS_LOGICAL.get(i, "?")


def _hat_logical(hx: int, hy: int) -> str:
    m = {
        (0, 1): "DP_UP", (0, -1): "DP_DOWN", (-1, 0): "DP_LEFT", (1, 0): "DP_RIGHT",
        (1, 1): "DP_UP+RIGHT", (-1, 1): "DP_UP+LEFT", (1, -1): "DP_DOWN+RIGHT",
        (-1, -1): "DP_DOWN+LEFT", (0, 0): "DP_CENTER",
    }
    return m.get((hx, hy), "DP_?")


def _open_joystick(pygame, index: int):
    """初始化 joystick 子系统并返回指定设备句柄；找不到返回 None。"""
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count == 0:
        print(f"{_YELLOW}未发现手柄。{_RESET}请检查：模式是否 macOS 可原生读取(Switch/Pro 或有线) / 是否已按任意键唤醒。")
        return None
    if index >= count:
        print(f"{_YELLOW}index {index} 超出范围{_RESET}（共 {count} 个设备，见 --list）。")
        return None
    js = pygame.joystick.Joystick(index)
    try:
        js.init()
    except Exception:
        pass
    return js


def list_devices(pygame):
    pygame.joystick.init()
    n = pygame.joystick.get_count()
    if n == 0:
        print(f"{_YELLOW}未发现手柄。{_RESET}请检查：2.4G 接收器在 macOS 呈 Xbox360 协议(读不到)，请切 Switch/Pro 或有线直连；插好后按任意键唤醒。")
        return
    print(f"{_BOLD}检测到 {n} 个输入设备：{_RESET}")
    for i in range(n):
        js = pygame.joystick.Joystick(i)
        try:
            js.init()
        except Exception:
            pass
        print(f"  [{i}] {_CYAN}{js.get_name()}{_RESET}  "
              f"axes={js.get_numaxes()} buttons={js.get_numbuttons()} "
              f"hats={js.get_numhats()} balls={js.get_numballs()}")


def _drift_baseline(js, na: int) -> list[tuple[int, float]]:
    """阶段1：静置采样各轴回中读数，返回疑似漂移的 (轴index, 静置峰值) 列表。"""
    print(f"{_BOLD}[阶段1/3] 漂移基线检测：请松开所有摇杆与扳机，让手柄静置…{_RESET}")
    for c in ("3", "2", "1"):
        sys.stdout.write(f"\r  采样倒计时 {c} 秒（保持不动）…   ")
        sys.stdout.flush()
        time.sleep(1)
    rest_peak = [0.0] * na
    for _ in range(40):  # ~2s 采样窗口
        for a in range(na):
            rest_peak[a] = max(rest_peak[a], abs(js.get_axis(a)))
        time.sleep(0.05)
    print(f"\r  静置各轴 |值| 峰值：{'  '.join(f'A{a}={rest_peak[a]:.2f}' for a in range(na))}          \n")
    drifts: list[tuple[int, float]] = []
    for a in range(na):
        if a < STICK_AXIS_COUNT and rest_peak[a] > DRIFT_THRESHOLD:
            drifts.append((a, rest_peak[a]))
            print(f"    {_RED}{_BOLD}⚠ A{a} ({_ax_logical(a)}) 静置峰值 {rest_peak[a]:.2f} > {DRIFT_THRESHOLD} → 疑似摇杆漂移{_RESET}")
        elif rest_peak[a] > DRIFT_THRESHOLD:
            print(f"    {_DIM}· A{a} ({_ax_logical(a)}) 静置 {rest_peak[a]:.2f}（若为扳机轴，未归中通常正常，请核对）{_RESET}")
    if not drifts:
        print(f"    {_GREEN}摇杆回中良好，暂未见明显漂移。{_RESET}\n")
    else:
        print()
    return drifts


def _print_checklist():
    print(f"{_BOLD}[阶段2/3] 逐项操作 —— 二手手柄验货清单（请全部做到，报告才会全绿）：{_RESET}")
    checklist = [
        "1) 主按键逐个按一遍：A  B  X  Y",
        "2) 肩键：LB  RB   （如有扳机：LT  RT 缓慢按到底再松开，看数值 0→满）",
        "3) 功能键：BACK(-)  START(+)  Home/Guide",
        "4) 摇杆按下：左摇杆(LS) 右摇杆(RS) 各垂直按下去",
        "5) 摇杆：左右摇杆各方向【推到底画圈】，检测全程与回中",
        "6) 十字键：上 下 左 右（若 hats=0，会以按钮或轴形式出现）",
        "7) 背键/宏键（若有）：P1  P2",
    ]
    for line in checklist:
        print(f"    {line}")
    print(f"    {_DIM}操作时每次按下/推动都会实时打印；完成后进阶段3报告。Ctrl+C 可随时结束并出报告。{_RESET}\n")


def verify(pygame, index: int, deadzone: float, show_map: bool = False) -> int:
    pygame.init()
    js = _open_joystick(pygame, index)
    if js is None:
        pygame.quit()
        return 1

    name = js.get_name()
    nb, na, nh = js.get_numbuttons(), js.get_numaxes(), js.get_numhats()
    print(f"{_BOLD}正在体检：{_CYAN}{name}{_RESET}  buttons={nb} axes={na} hats={nh}  deadzone={deadzone}")
    if show_map:
        print(f"{_DIM}映射参考(实测: 猎户座2 NS/Switch 模式·macOS SDL)：A/B/X/Y=0/1/2/3  −/Home/+=4/5/6  "
              f"LB/RB=9/10  十字上/下/左/右=11/12/13/14  Capture=15；轴 A0/A1=左摇杆 A2/A3=右摇杆 A4/A5=LT/RT{_RESET}\n")

    drifts = _drift_baseline(js, na)
    _print_checklist()

    pressed_buttons: set[int] = set()
    moved_axes: set[int] = set()
    used_hats: set[int] = set()
    axis_raw = [0.0] * na
    axis_min = [1.0] * na
    axis_max = [-1.0] * na
    prev_axis_state = [0] * na

    # 直接轮询硬件状态（不依赖 SDL 事件）：可规避 macOS 上 Switch Pro 等手柄不发事件的问题
    prev_btn = [0] * nb
    prev_hat = [(0, 0)] * nh
    print(f"{_DIM}(硬件直读轮询模式；若按你手上的手柄屏幕毫无变化，说明它不是 pygame 读到的设备——用 --list 核对设备名){_RESET}\n")
    last_redraw = 0.0
    try:
        pygame.event.pump()  # 保持 SDL 事件队列/热插拔更新
        while True:
            # 按钮上升/下降沿
            for i in range(nb):
                st = 1 if js.get_button(i) else 0
                if st != prev_btn[i]:
                    prev_btn[i] = st
                    if st:
                        pressed_buttons.add(i)
                        lg = f"  → {_btn_logical(i)}" if show_map else ""
                        print(f"{_GREEN}▶ BUTTON {_BOLD}{i:>2}{_RESET}{_GREEN} ↓ PRESS{lg}{_RESET}")
                    else:
                        print(f"{_DIM}   BUTTON {i:>2} ↑ release{_RESET}")
            # 轴阈值跨越 + 行程记录
            for a in range(na):
                v = js.get_axis(a)
                axis_raw[a] = v
                axis_min[a] = min(axis_min[a], v)
                axis_max[a] = max(axis_max[a], v)
                if abs(v) > deadzone:
                    moved_axes.add(a)
                zone = 0 if abs(v) <= deadzone else (1 if v > 0 else -1)
                if zone != prev_axis_state[a]:
                    prev_axis_state[a] = zone
                    label = "·" if zone == 0 else ("+" if zone > 0 else "-")
                    lg = f" → {_ax_logical(a)}" if show_map else ""
                    print(f"{_CYAN}▶ AXIS   {_BOLD}{a:>2}{_RESET}{_CYAN} {label} value={v:+.2f}  {_bar(v)}{lg}{_RESET}")
            # HAT 十字键
            for h in range(nh):
                hv = js.get_hat(h)
                if hv != prev_hat[h]:
                    prev_hat[h] = hv
                    used_hats.add(h)
                    lg = f"  -> {_hat_logical(hv[0], hv[1])}" if show_map else ""
                    print(f"{_YELLOW}▶ HAT    {_BOLD}{h}{_RESET}{_YELLOW} ({hv[0]:+d},{hv[1]:+d}){lg}{_RESET}")

            now = time.time()
            if now - last_redraw >= 0.4:
                last_redraw = now
                snap = "  ".join(f"A{a}={axis_raw[a]:+.2f}" for a in range(na))
                sys.stdout.write(f"\r{_DIM}[live] 键 {len(pressed_buttons)}/{nb}  轴 {len(moved_axes)}/{na}  |  {snap}{_RESET}          ")
                sys.stdout.flush()
            pygame.event.pump()
            time.sleep(0.02)  # ~50Hz 轮询
    except KeyboardInterrupt:
        print("\n")

    return _report(js, name, nb, na, nh, pressed_buttons, moved_axes, used_hats, axis_min, axis_max, drifts)


def _report(js, name, nb, na, nh, pressed_buttons, moved_axes, used_hats, axis_min, axis_max, drifts) -> int:
    """阶段3：健康报告。"""
    pygame_quit_safe()
    print(f"{_BOLD}[阶段3/3] ===== 健康报告：{name} ====={_RESET}")

    # 1) 按键覆盖（死键排查）
    print(f"\n{_BOLD}按键覆盖（逐个按，全绿说明无死键）：{_RESET}")
    _coverage_grid(range(nb), pressed_buttons)
    missing_btn = [b for b in range(nb) if b not in pressed_buttons]

    # 2) 轴行程 / 扳机
    print(f"\n{_BOLD}轴行程（推到底/扳机按到底，看 min→max 是否覆盖全程）：{_RESET}")
    dead_axes = []
    for a in range(na):
        if axis_max[a] < axis_min[a]:  # 从未更新过
            lo, hi = 0.0, 0.0
            state = f"{_RED}未移动/可能死轴{_RESET}"
            dead_axes.append(a)
        else:
            lo, hi = axis_min[a], axis_max[a]
            if a not in moved_axes:
                state = f"{_YELLOW}仅小幅/未过死区{_RESET}"
            elif lo > -0.85 or hi < 0.85:
                state = f"{_YELLOW}行程不全（未推到边界？）{_RESET}"
            else:
                state = f"{_GREEN}OK{_RESET}"
        print(f"    A{a} {_ax_logical(a):>11}: [{lo:+.2f} → {hi:+.2f}]  {state}")

    # 3) 十字键 HAT
    if nh:
        print(f"\n{_BOLD}十字键 HAT：{_RESET} 触发过的 hat 编号 {sorted(used_hats) or _note_none()}（{len(used_hats)}/{nh}）")
        if len(used_hats) < nh:
            print(f"    {_YELLOW}未触发过全部 HAT，请各方向按一遍（部分手柄十字键走按键而非 HAT）{_RESET}")
    else:
        print(f"\n{_DIM}此设备 hats=0：十字键会以按钮或轴形式出现，请用上方按键/轴覆盖来确认。{_RESET}")

    # 4) 漂移
    print(f"\n{_BOLD}漂移检测：{_RESET}")
    if drifts:
        print(f"    {_RED}{_BOLD}发现疑似漂移轴：{[f'A{a}={v:.2f}' for a, v in drifts]}{_RESET}  ← 二手重点警惕项")
    else:
        print(f"    {_GREEN}静置时摇杆回中良好，未见明显漂移。{_RESET}")

    # 结论
    print(f"\n{_BOLD}===== 结论 ====={_RESET}")
    issues = []
    if missing_btn:
        issues.append(f"未触发按键 index={missing_btn}")
    if dead_axes:
        issues.append(f"未移动/死轴 index={dead_axes}")
    if drifts:
        issues.append("疑似摇杆漂移")
    if pressed_buttons and not missing_btn and not dead_axes and not drifts:
        print(f"    {_GREEN}{_BOLD}✅ 所有按键/轴均有响应、无死键、无明显漂移 —— 手柄硬件状态良好。{_RESET}")
        return 0
    if not pressed_buttons and not moved_axes and not used_hats:
        print(f"    {_YELLOW}未收到任何输入。若这不是你要测的手柄 / 模式不对，请用 --list 或 --watch 确认后重试。{_RESET}")
        return 1
    print(f"    {_YELLOW}⚠ 存在待确认项：{'; '.join(issues)}{_RESET}")
    print(f"    {_DIM}（未触发项可能只是没按到；请对照清单把每个键/轴都操作一遍再判定）{_RESET}")
    return 0


def _coverage_grid(ids, hit, per_row: int = 10) -> None:
    ids = list(ids)
    for start in range(0, len(ids), per_row):
        cells = []
        for i in ids[start:start + per_row]:
            mark = f"{_GREEN}✓{i:<2}{_RESET}" if i in hit else f"{_DIM}·{i:<2}{_RESET}"
            cells.append(mark)
        print("    " + " ".join(cells))


def pygame_quit_safe() -> None:
    try:
        import pygame
        pygame.quit()
    except Exception:
        pass


def _note_none() -> str:
    return f"{_DIM}(none){_RESET}"


def watch_devices(pygame, interval: float = 1.5) -> None:
    """持续重扫 SDL 设备表：切模式/插拔的瞬间一旦出现新 gamepad，立即打印其 axes/buttons/hats。"""
    def scan():
        pygame.joystick.quit()
        pygame.joystick.init()
        out = {}
        for i in range(pygame.joystick.get_count()):
            js = pygame.joystick.Joystick(i)
            try:
                js.init()
            except Exception:
                pass
            out[i] = js.get_name()
        return out

    pygame.init()
    prev = scan()
    if prev:
        print(f"{_DIM}当前在位：{prev}{_RESET}")
    else:
        print(f"{_DIM}当前无手柄；等待 macOS 出现 gamepad（试着插拔/切到手柄的 Switch·Pro 或有线模式）…{_RESET}")
    print(f"{_BOLD}监视中（每 {interval}s 重扫一次），出现新手柄会自动打印。Ctrl+C 退出。{_RESET}")
    try:
        while True:
            time.sleep(interval)
            cur = scan()
            for i, name in cur.items():
                if i not in prev:
                    js = pygame.joystick.Joystick(i)
                    try:
                        js.init()
                    except Exception:
                        pass
                    print(f"{_GREEN}{_BOLD}★ 新手柄 [{i}] {_CYAN}{name}{_RESET}{_GREEN}  "
                          f"axes={js.get_numaxes()} buttons={js.get_numbuttons()} hats={js.get_numhats()}{_RESET}")
            for i in prev:
                if i not in cur:
                    print(f"{_YELLOW}✕ 手柄移除 [{i}] {prev[i]}{_RESET}")
            prev = cur
    except KeyboardInterrupt:
        print()
    finally:
        pygame.quit()


def dump_buttons(pygame, index: int) -> int:
    """持续打印每个按钮的原始电平(0/1)与轴值：用来判定某些键(如 Home/−)到底是被系统吞了还是映射在意外 index。"""
    pygame.init()
    js = _open_joystick(pygame, index)
    if js is None:
        pygame.quit()
        return 1
    nb, na = js.get_numbuttons(), js.get_numaxes()
    print(f"{_BOLD}原始按键电平（{js.get_name()}）：{_RESET}buttons={nb} axes={na}")
    print(f"{_DIM}按住某个键，看对应 index 是否变 █；若按 Home/− 时**没有任何一位变化**，说明被 macOS 系统拦截了（非死键）。Ctrl+C 退出。{_RESET}\n")
    ever_high = set()
    try:
        while True:
            btns = [1 if js.get_button(i) else 0 for i in range(nb)]
            ever_high |= {i for i, b in enumerate(btns) if b}
            axes = [js.get_axis(a) for a in range(na)]
            line = " ".join(f"{i}:{'█' if b else '·'}" for i, b in enumerate(btns))
            ax = "  ".join(f"A{a}={axes[a]:+.2f}" for a in range(na))
            sys.stdout.write(f"\rBTN {line}    {ax}    见过= {sorted(ever_high)}          ")
            sys.stdout.flush()
            pygame.event.pump()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        pygame.quit()
    never = [i for i in range(nb) if i not in ever_high]
    print(f"\n{_DIM}全程未出现高电平的按钮 index：{never or _note_none()}{_RESET}")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="8BitDo 手柄硬件体检（macOS/Linux，pygame 后端）")
    parser.add_argument("--list", action="store_true", help="仅列出检测到的手柄后退出")
    parser.add_argument("--index", type=int, default=0, help="使用第几个设备（默认 0）")
    parser.add_argument("--deadzone", type=float, default=0.12, help="摇杆死区阈值（默认 0.12）")
    parser.add_argument("--watch", action="store_true", help="持续监视：一旦 macOS 出现新手柄(切模式/插拔)自动打印其信息")
    parser.add_argument("--map", action="store_true", dest="show_map", help="交互自检时把原始 index 标注为 §4 逻辑按键名(标准 SDL2/Xbox 布局参考)")
    parser.add_argument("--buttons", action="store_true", help="持续打印每个按钮原始电平(0/1)与轴值，定位被系统拦截/意外 index 的键")
    args = parser.parse_args(argv)

    pygame = _require_pygame()
    if args.list:
        pygame.init()
        list_devices(pygame)
        pygame.quit()
        return 0
    if args.watch:
        watch_devices(pygame)
        return 0
    if args.buttons:
        return dump_buttons(pygame, args.index)
    return verify(pygame, args.index, args.deadzone, show_map=args.show_map)


if __name__ == "__main__":
    raise SystemExit(main())
