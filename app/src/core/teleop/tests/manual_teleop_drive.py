# -*- coding: utf-8 -*-
"""
遥操作控制器手动驱动脚本 (本模块自跑, 暂不接 Web)。

用途:
  在真机 (RK3588/Linux, 装 evdev) 上, 用一个手柄实时驱动 TeleopController,
  验证"模式切换 / 按住即动 / dead-man / 故障安全 / 玩家灯灯语"整套逻辑。

两种运行模式 (命令出口不同, 上层逻辑完全一致 —— 依赖倒置):
  1) 干跑 (默认): 命令打到 ConsoleCommandSink, 只打印英文动作, **不碰任何硬件**;
     没手柄 / 没机器人也能跑, 便于开发与演示。
  2) --robot: 惰性 import 服务层, 命令经本脚本内的 RobotServiceSink 适配器落到
     RobotService 既有色接口 (jog_continuous / set_do / estop / go_home / go_fold /
     set_global_speed_factor)。适配器写在本脚本里, 保证 core 层不依赖服务层。

协议隔离 (关键):
  本脚本只通过 `--profile` 选择"连接协议档"(默认 switch_pro = 2.4G NS 接收器)。
  以后换成 USB 直连 / 蓝牙, 只需用 manual_map_evdev.py --save 重新生成码表并在
  mapping.get_profile() 登记新档, **本脚本与 core 上层一行都不用改**。

用法:
    cd app
    # 依赖: python -m pip install evdev (仅真机读手柄需要)
    PYTHONPATH=src .venv/bin/python src/core/teleop/tests/manual_teleop_drive.py --list-profiles
    PYTHONPATH=src .venv/bin/python src/core/teleop/tests/manual_teleop_drive.py            # 干跑
    PYTHONPATH=src .venv/bin/python src/core/teleop/tests/manual_teleop_drive.py --path /dev/input/event9
    PYTHONPATH=src .venv/bin/python src/core/teleop/tests/manual_teleop_drive.py --robot     # 真正驱动机械臂

命名以 `manual_` 开头, pytest 不会收集它。
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import threading

# 让 `core.teleop.*` 可 import (src 目录 = 本文件上溯三级)
_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from core.teleop.device import LinuxGamepadInput, TeleopDeviceError  # noqa: E402
from core.teleop.feedback import PlayerLedFeedback                   # noqa: E402
from core.teleop.mapping import get_profile                          # noqa: E402
from core.teleop.sink import ConsoleCommandSink                      # noqa: E402
from core.teleop.teleop import TeleopController, TeleopConfig        # noqa: E402

# 键位速查 (与 docs/teleop_design.md §4/§6 一致; 全英文)
CHEATSHEET = """
Teleop control map (2.4G Switch Pro profile). All motion gated by RT dead-man.
  BACK         cycle mode: Translation -> Rotation -> Joint   (green LEDs = mode)
  START        connect / disconnect robot (servo power)   RT = arm motion (dead-man)
  A            toggle spray                     B   pause / resume motion
  R3           toggle gripper OPEN/CLOSE (steady blue LED = open)
  X            clear alarm -> IDLE              Y   EMERGENCY STOP (spray off + halt)
  RB / LB      speed tier + / -  (Trans/Rot)    LT = reduced speed (extra green LED)
  CAPTURE      Go HOME    (needs enabled + idle) HOME = Go FOLD (needs enabled + idle)
  Translation: LS_Y=X  LS_X=Y  RS_Y=Z  RS_X=Rz   (push + RT)
  Rotation:    LS_X=Rx LS_Y=Ry RS_X=Rz   (push + RT)
  Joint:       HOLD a key = that joint (DP:J1..J4, L3:J5, LB:J6); RS_Y gives +/- ; RT arm
  LEDs: #green=mode, blink=state (steady IDLE / slow ENABLED / fast MOVING), blue=FAULT
"""


class RobotServiceSink:
    """把遥操作高层命令适配到 RobotService 既有色接口 (仅 --robot 时按需构造)。

    适配器刻意留在测试脚本内, 而非 core, 以维持依赖方向: core -> 抽象协议, 服务层 -> 实现。
    """

    def __init__(self, svc, robot_type: str, ip: str, port: str) -> None:
        self._svc = svc
        self._robot_type = robot_type
        self._ip = ip
        self._port = port

    def connect(self) -> bool:
        # 服务层 connect 内含 EnableRobot (上电使能伺服); 幂等。由 START 键触发。
        # 注意: dobot_driver.startup() 会丢弃 EnableRobot 回复并直接 return True, 所以 connect()==True
        # 不等于伺服已使能。真正的使能结果看上面驱动的 "EnableRobot: ..." INFO 行。
        print(f"[teleop] START -> RobotService.connect({self._robot_type}, {self._ip}:{self._port})"
              f" (watch the 'EnableRobot: ...' INFO line above)")
        ok, msg = self._svc.connect(self._robot_type, self._ip, self._port)
        print(f"[teleop] START <- connect() ok={ok} msg={msg!r}"
              f" is_connected={self._svc.is_connected()} running_state={self._svc.get_running_state()}")
        if not ok:
            print(f"[teleop] connect failed: {msg}")
        return ok

    def disconnect(self) -> None:
        # 服务层 disconnect 内含 DisableRobot (去使能)。由 START 键触发 (core 已先安全停机关喷)。
        print("[teleop] START -> RobotService.disconnect() (DisableRobot)")
        self._svc.disconnect()
        print(f"[teleop] START <- disconnected; is_connected={self._svc.is_connected()}")

    def jog_start(self, axis: str, direction: int) -> None:
        self._svc.jog_continuous(axis, 1 if direction > 0 else -1)

    def jog_stop(self, axis: str) -> None:
        self._svc.jog_continuous(axis, 0)

    def stop_all_jogs(self) -> None:
        # direction=0 会停止该运动学组 (笛卡尔 / 关节), 两组各停一次覆盖全部点动。
        self._svc.jog_continuous("X", 0)
        self._svc.jog_continuous("J1", 0)

    def set_spray(self, on: bool) -> None:
        # 手动开关喷涂走立即指令 (DOExecute), 毫秒级断料。
        self._svc.set_do(index=self._svc.spray_do_index, status=1 if on else 0, immediate=True)

    def set_speed_tier(self, percent: int) -> None:
        self._svc.set_global_speed_factor(int(percent))

    def pause(self) -> None:
        self._svc.pause()

    def resume(self) -> None:
        self._svc.resume()

    def clear_error(self) -> None:
        self._svc.clear_error()

    def estop(self) -> None:
        # RobotService.estop 内部已"先立即关喷 DO 再急停" (Fail-Close)。
        self._svc.estop()

    def go_home(self) -> None:
        self._svc.go_home()

    def go_fold(self) -> None:
        self._svc.go_fold()

    def gripper_open(self) -> None:
        self._svc.open_gripper()

    def gripper_close(self) -> None:
        self._svc.clamp_gripper()


def _make_robot_sink(robot_type: str, ip: str, port: str) -> "RobotServiceSink":
    """惰性 import 服务层构造 sink (不预连接; 连接由 START 键→sink.connect() 触发)。"""
    from apps.robot.services.robot_service import RobotService  # 延迟: 干跑无需服务层

    return RobotServiceSink(RobotService(), robot_type, ip, port)


def main(argv=None) -> int:
    # 打开 INFO 日志, 才能看到驱动的 ClearError/EnableRobot 真实回复 (诊断"软件已连但伺服未使能"的关键)。
    logging.basicConfig(level=logging.INFO,
                        format="[%(levelname)s][%(name)s] %(message)s", force=True)
    ap = argparse.ArgumentParser(description="Manual teleop drive (dry-run by default).")
    ap.add_argument("--profile", default="switch_pro",
                    help="connection protocol profile (default: switch_pro = 2.4G)")
    ap.add_argument("--path", default=None, help="evdev node (e.g. /dev/input/event9)")
    ap.add_argument("--name", default=None, help="only match device whose name contains this")
    ap.add_argument("--seconds", type=float, default=None, help="auto-stop after N seconds")
    ap.add_argument("--no-led", action="store_true", help="disable player-LED feedback")
    ap.add_argument("--robot", action="store_true",
                    help="drive the real arm via RobotService (default is dry-run console)")
    ap.add_argument("--robot-type", default="dobot")
    ap.add_argument("--ip", default="192.168.5.1")
    ap.add_argument("--port", default="29999")
    ap.add_argument("--list-profiles", action="store_true", help="list protocol profiles and exit")
    args = ap.parse_args(argv)

    if args.list_profiles:
        from core.teleop import mapping as _m
        print("Available teleop profiles:", sorted(getattr(_m, "_PROFILES", {})))
        return 0

    profile = get_profile(args.profile)   # 未知档直接 fail-fast, 不静默回退错映射
    print(CHEATSHEET)

    device = LinuxGamepadInput(profile=profile, path=args.path, name_substr=args.name)
    feedback = PlayerLedFeedback(enabled=not args.no_led)

    if args.robot:
        sink = _make_robot_sink(args.robot_type, args.ip, str(args.port))
        svc = sink._svc
        is_connected = svc.is_connected
        is_robot_idle = lambda: svc.get_running_state() == 0  # noqa: E731
        print("[teleop] LIVE mode: press START on the pad to CONNECT+power the arm; START again to DISCONNECT.")
    else:
        sink = ConsoleCommandSink()
        is_connected = None       # 干跑: 无外部链路, START 只切换控制器内部连接标志
        is_robot_idle = None

    def show_status(st):
        # 仅状态行变化时打印, 避免刷屏 (每拍都回调, 这里靠 hud_line 文本去重)
        line = st.hud_line()
        if line != getattr(show_status, "_last", None):
            show_status._last = line
            print("[status]", line)

    ctrl = TeleopController(
        device=device, sink=sink, feedback=feedback, config=TeleopConfig(),
        is_connected=is_connected, is_robot_idle=is_robot_idle, on_status=show_status,
    )

    try:
        ctrl.start()
    except TeleopDeviceError as e:
        print(f"[teleop] device open failed: {e}")
        print("[teleop] Tip: check receiver/mode, add --path, or run dry-run without --robot.")
        return 1

    print(f"[teleop] device '{device.name}' ready. Press START to connect/power the arm. Ctrl+C to stop (Fail-Close: spray off + halt).")

    # Ctrl+C 加固: 第一次请求安全停 (让 run() 正常返回走 finally 的 Fail-Close), 但用一个
    # 看门狗线程兑底: 防止 stop()/connect 阻塞在已断的网络调用上时进程卡死无法退出; 再按一次立即硬退。
    def _force_exit():
        print("\n[teleop] force exit (skip blocking cleanup).")
        os._exit(130)

    _sig = {"n": 0}

    def _on_sigint(signum, frame):  # noqa: ARG001
        _sig["n"] += 1
        if _sig["n"] == 1:
            print("\n[teleop] Ctrl+C: safe stop requested (spray OFF + halt). "
                  "Press again to force-exit immediately.")
            ctrl._running = False                        # 让 run() 退出并执行 finally 的 Fail-Close
            threading.Timer(8.0, _force_exit).start()   # 网络被卡住也保证 8s 内退出
        else:
            _force_exit()

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        ctrl.run(max_seconds=args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.stop()
        if args.robot:
            try:
                if sink._svc.is_connected():
                    sink._svc.disconnect()
            except Exception:
                pass
        print("[teleop] stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
