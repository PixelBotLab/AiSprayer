"""
InexbotRobot 集成测试文件

状态语义（与官方 demo 一致）：
  - get_servo_state==3：伺服已上电使能（待命，非“正在运动”）
  - get_robot_running_state==0：运动程序段已结束（move/队列 wait 用此判定）

测试覆盖：
  - startup / shutdown
  - get_servo_state / get_running_state / get_current_pos
  - is_reachable
  - move_j / move_l / go_home
  - queue_start / queue_push_l / queue_push_j / queue_send
  - queue_suspend / queue_resume / queue_stop / queue_get_remaining

运行前确认：
  1. 机器人控制器已开机并连接到网络
  2. 修改 ROBOT_IP 为实际 IP
  3. 修改各测试点位为当前机器人可达范围内的安全位置
  4. 确保运动范围内无障碍物

分阶段运行（见 ProMD/Plan_per.md）：
  python test_robot.py --phase 2   # 仅状态查询
  python test_robot.py --phase 3   # 可达性（不动臂）
  python test_robot.py --phase 4   # 单段运动
  python test_robot.py --phase 5   # 队列运动
"""

import argparse
import math
import time
import sys
import os
import logging
from typing import List, Optional, Union

# 1. 路径锚定策略
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.hardware.robot.inexbot_driver import InexbotDriver, RobotPose
from aisprayer.core.hardware.robot.inexbot_driver import (
    SERVO_STATE_STOP, SERVO_STATE_READY, SERVO_STATE_ALARM, SERVO_STATE_RUNNING,
    RUNNING_STATE_STOP, RUNNING_STATE_PAUSE, RUNNING_STATE_RUNNING
)
# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname).1s] %(module)s:%(funcName)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("TestRobot")
# ──────────────────────────────────────────────────────────────────
#  配置区：根据实际情况修改
# ──────────────────────────────────────────────────────────────────

ROBOT_IP   = "192.168.2.14"
ROBOT_PORT = "6001"

# 示教模式联调（默认 --mode 0）；全局速度% 与 MOVL 线速度 mm/s 共同影响实际快慢
GLOBAL_SPEED_PERCENT = 80
QUEUE_MOVEL_SPEED    = 500   # MOVL mm/s，上限 1000；与全局 80% 配合（改速度前实机验证值）
QUEUE_MOVEL_ACC      = 80
QUEUE_MOVEL_DEC      = 80
QUEUE_PL_WORK        = 4
QUEUE_PL_HOME        = 2     # Home 略平滑，避免 pl=0 长时间精确爬行
QUEUE_LARGE_SEGMENTS = 40    # >31 测分批，末条为 HOME_POSE

# 测试用点位（直角坐标，姿态角单位：rad）
# 请根据实际机器人工作空间调整，确保安全！
POS_A      = [400.0,    0.0,  1077.0,  -3.141,  0.0,    0.0]   # 测试点 A
POS_B      = [400.0,  100.0,  1000.0,  -3.141,  0.0,    0.0]   # 测试点 B
POS_C      = [350.0,  100.0,  950.0,  -3.141,  0.0,    0.0]   # 测试点 C
POS_UNREACHABLE = [9999.0, 9999.0, 9999.0, -3.141, 0.0, 0.0]     # 明显不可达点

# 控制器 Home（与 go_home / startup 后位姿一致，2026-05-25 实机读取）
HOME_POSE  = [871.024, 125.479, 1076.740, -3.1416, 0.0, 0.0]

# 队列作业点（末条必须为 HOME_POSE，队列跑完后臂在 Home）
QUEUE_WORK_POINTS = [
    [400.0,   0.0, 950.0, -3.141,  0.0, 0.0],
    [420.0,  40.0, 960.0, -3.141,  0.0, 0.0],
    [440.0,  80.0, 970.0, -3.141,  0.0, 0.0],
    [420.0, 120.0, 980.0, -3.141,  0.0, 0.0],
    [400.0,  80.0, 990.0, -3.141,  0.0, 0.0],
]
QUEUE_POINTS = QUEUE_WORK_POINTS + [HOME_POSE]


# ──────────────────────────────────────────────────────────────────
#  测试工具
# ──────────────────────────────────────────────────────────────────

_pass = 0
_fail = 0
_skip = 0
_aborted = False

# 各阶段编号 -> (名称, 测试函数名)
PHASES = {
    1: ("startup / shutdown", "test_startup_shutdown"),
    2: ("状态查询", "test_state_query"),
    3: ("is_reachable", "test_is_reachable"),
    4: ("单段运动", "test_motion"),
    5: ("队列运动", "test_queue"),
}

# ── 阶梯式回退：间隔参数（先统计，再逐步改短）──
GAP_AFTER_SUB_ALL = 0.35     # Phase5 all：每个子项结束后的间隔（激进回退：直接到 0.35）
GAP_AFTER_SUB_SINGLE = 0.35  # Phase5 单个子项：结束后的间隔
GAP_AFTER_ALL_EXTRA = 0.35   # Phase5 all 全部子项跑完后的额外稳定等待（阶梯缩短）
LARGE_PRE_HOME_WAIT = 0.5    # large 子项开始前 go_home 后等待（阶梯缩短）


def _now() -> float:
    return time.time()


def _fmt_sec(sec: float) -> str:
    return f"{sec:.3f}s"


class _Timer:
    def __init__(self, name: str):
        self.name = name
        self.t0 = _now()

    def done(self) -> float:
        return _now() - self.t0


def _log_step_timing(label: str, fn_sec: float, finally_sec: float, sleep_sec: float) -> None:
    logger.info(
        "[timing] %s fn=%s finally=%s sleep=%s total=%s",
        label,
        _fmt_sec(fn_sec),
        _fmt_sec(finally_sec),
        _fmt_sec(sleep_sec),
        _fmt_sec(fn_sec + finally_sec + sleep_sec),
    )


def _wait_queue_running_started(robot: InexbotDriver, timeout: float = 20.0) -> bool:
    """queue_send(wait=False) 后等到 running==2，避免未起步就 suspend 导致 resume 后假死。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if robot.get_running_state() == RUNNING_STATE_RUNNING:
            return True
        time.sleep(0.05)
    logger.warning("[test] 队列未在 %.1fs 内进入 RUNNING(2)", timeout)
    return False


def _wait_async_move_done(robot: InexbotDriver, start_timeout: float = 15.0) -> bool:
    """
    move_* wait=False 下发后：先等到 running==2（本段已开始），再 _wait_motion_done。
    若仍停留在上一段的 STOP(0)，直接调 _wait_motion_done 会误判「已结束」。
    """
    time.sleep(0.05)
    t0 = time.time()
    while time.time() - t0 < start_timeout:
        if robot.get_running_state() == RUNNING_STATE_RUNNING:
            return robot._wait_motion_done()
        time.sleep(0.025)
    logger.warning(
        "[test] async move: 未在 %.1fs 内观察到 RUNNING(2)，仍尝试等待停止", start_timeout
    )
    return robot._wait_motion_done()


def _pose_list(pos: Optional[Union[RobotPose, list]]) -> Optional[list]:
    """将 RobotPose 或 list 统一为 6 维列表，用于打印与断言。"""
    if pos is None:
        return None
    if isinstance(pos, RobotPose):
        return pos.to_list()
    return list(pos)[:6]


def check(name: str, condition: bool, detail: str = "") -> None:
    global _pass, _fail
    status = "PASS" if condition else "FAIL"
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{status}] {name}{suffix}")
    if condition:
        _pass += 1
    else:
        _fail += 1


def skip(name: str, reason: str = "") -> None:
    global _skip
    suffix = f"  ({reason})" if reason else ""
    print(f"  [SKIP] {name}{suffix}")
    _skip += 1


def section(title: str) -> None:
    print(f"\n{'─' * 55}")
    print(f"  {title}")
    print(f"{'─' * 55}")


def summary() -> None:
    total = _pass + _fail + _skip
    print(f"\n{'═' * 55}")
    print(f"  测试结果：{_pass} 通过，{_fail} 失败，{_skip} 跳过，共 {total} 项")
    if _aborted:
        print("  （因异常/中断，部分阶段未执行）")
    print(f"{'═' * 55}\n")


# ──────────────────────────────────────────────────────────────────
#  各功能测试函数
# ──────────────────────────────────────────────────────────────────

def test_startup_shutdown(robot: InexbotDriver) -> bool:
    section("1. startup / shutdown")

    ok = robot.startup(timeout=25.0)
    check("startup 返回 True", ok)
    if not ok:
        print("  !! startup 失败，后续测试跳过")
        return False

    servo = robot.get_servo_state()
    check("startup 后伺服已使能(3，非运动完成)", servo == SERVO_STATE_RUNNING,
          f"实际={servo}（0停 1就绪 2报警 3已上电）")
    check("is_servo_enabled() 与伺服状态一致", robot.is_servo_enabled() == (servo == SERVO_STATE_RUNNING))
    return True


def test_state_query(robot: InexbotDriver) -> None:
    section("2. 状态查询")

    servo = robot.get_servo_state()
    check("get_servo_state 返回合法值 [0-3]", 0 <= servo <= 3,
          f"实际={servo}")

    running = robot.get_running_state()
    check("get_running_state 返回合法值 [0-2]", 0 <= running <= 2,
          f"实际={running}（0停止 1暂停 2运行中）")
    if running == RUNNING_STATE_STOP:
        check("静止时 is_robot_idle()", robot.is_robot_idle())

    pos = robot.get_current_pose()
    pl = _pose_list(pos)
    check("get_current_pose 返回 RobotPose", pos is not None and isinstance(pos, RobotPose),
          f"type={type(pos).__name__}")
    if pl:
        print(f"  当前位置: {[round(v, 4) for v in pl]}")


def test_is_reachable(robot: InexbotDriver) -> None:
    section("3. is_reachable 可达性判断")

    r1 = robot.is_reachable(POS_A, "MOVL")
    check("POS_A MOVL 可达", r1 is True)

    r2 = robot.is_reachable(POS_A, "MOVJ")
    check("POS_A MOVJ 可达", r2 is True)

    r3 = robot.is_reachable(POS_UNREACHABLE, "MOVL")
    check("超范围点 MOVL 不可达", r3 is False)


def test_move_j(robot: InexbotDriver) -> None:
    section("4. move_j 关节运动")

    ret = robot.move_j(POS_A, velocity=50)
    check("move_j 到 POS_A 成功（wait=True）", ret == 0, f"ret={ret}")

    state = robot.get_running_state()
    check("move_j 完成后运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")

    pl = _pose_list(robot.get_current_pose())
    if pl:
        print(f"  到达位置: {[round(v, 4) for v in pl]}")


def test_move_l(robot: InexbotDriver) -> None:
    section("5. move_l 直线运动")

    ret = robot.move_l(POS_B, velocity=250)
    check("move_l 到 POS_B 成功（wait=True）", ret == 0, f"ret={ret}")

    state = robot.get_running_state()
    check("move_l 完成后运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")

    ret2 = robot.move_l(POS_A, velocity=250, wait=False)
    check("move_l wait=False 下发成功", ret2 == 0, f"ret={ret2}")
    ok_wait = _wait_async_move_done(robot)
    check("异步 move_l 等待结束成功", ok_wait, "wait_motion_done 超时或失败")
    # 完成后再读 running，并做一次短稳定避免边沿读到 2
    time.sleep(0.1)
    state2 = robot.get_running_state()
    check("等待完成后运行状态为 STOP(0)", state2 == RUNNING_STATE_STOP,
          f"实际={state2}")


def test_go_home(robot: InexbotDriver) -> None:
    section("6. go_home 回零点")

    ret = robot.go_home()
    check("go_home 成功（wait=True）", ret == 0, f"ret={ret}")

    state = robot.get_running_state()
    check("go_home 完成后运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")


def _queue_push_work_path(robot: InexbotDriver, points: List[list]) -> None:
    """按统一速度/加速度入队；末点为 Home 时用 QUEUE_PL_HOME。"""
    for i, p in enumerate(points):
        pl = QUEUE_PL_HOME if i == len(points) - 1 else QUEUE_PL_WORK
        robot.queue_push_l(
            p,
            velocity=QUEUE_MOVEL_SPEED,
            acc=QUEUE_MOVEL_ACC,
            dec=QUEUE_MOVEL_DEC,
            pl=pl,
        )


def _wait_left_first_queue_point(
    robot: InexbotDriver, first_pose: list, min_travel_mm: float = 30.0, timeout: float = 15.0
) -> None:
    """等臂离开队列首点再暂停（替代固定 sleep 3s）。"""
    fx, fy, fz = float(first_pose[0]), float(first_pose[1]), float(first_pose[2])
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos = _pose_list(robot.get_current_pose())
        if pos:
            d = math.sqrt((pos[0] - fx) ** 2 + (pos[1] - fy) ** 2 + (pos[2] - fz) ** 2)
            if d >= min_travel_mm:
                return
        time.sleep(0.1)
    logger.warning("[test] wait leave first point timeout, continue anyway")


def test_motion(robot: InexbotDriver) -> None:
    """阶段 4：单段运动（move_j → move_l → go_home）"""
    test_move_j(robot)
    test_move_l(robot)
    test_go_home(robot)


def test_queue_basic(robot: InexbotDriver) -> None:
    section("7. 队列运动 — 基本流程（push_l + send）")

    ret_open = robot.queue_start()
    check("queue_start 成功", ret_open == 0, f"ret={ret_open}")

    for i, p in enumerate(QUEUE_POINTS):
        pl = QUEUE_PL_HOME if i == len(QUEUE_POINTS) - 1 else QUEUE_PL_WORK
        ret = robot.queue_push_l(
            p, velocity=QUEUE_MOVEL_SPEED, acc=QUEUE_MOVEL_ACC, dec=QUEUE_MOVEL_DEC, pl=pl
        )
        check(f"queue_push_l 点{i+1} 插入成功", ret == 0, f"ret={ret}")

    check("queue_size 计数正确", robot._queue_size == len(QUEUE_POINTS),
          f"实际={robot._queue_size}")

    ret_send = robot.queue_send(wait=True)
    check("queue_send 发送并等待完成", ret_send == 0, f"ret={ret_send}")

    state = robot.get_running_state()
    check("队列执行完成后运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")

    remaining = robot.queue_get_remaining()
    check("queue_get_remaining 执行完后为 0", remaining == 0,
          f"实际={remaining}")


def test_queue_push_j(robot: InexbotDriver) -> None:
    section("8. 队列运动 — queue_push_j（MOVJ）")

    robot.queue_start()
    ret1 = robot.queue_push_j(POS_A, velocity=30, pl=2)
    ret2 = robot.queue_push_j(POS_B, velocity=30, pl=2)
    ret3 = robot.queue_push_j(HOME_POSE, velocity=30, pl=0)
    check("queue_push_j x3（含回 Home）插入成功", ret1 == 0 and ret2 == 0 and ret3 == 0,
          f"ret1={ret1}, ret2={ret2}, ret3={ret3}")

    ret_send = robot.queue_send(wait=True)
    check("queue_push_j 发送并等待完成", ret_send == 0, f"ret={ret_send}")


def test_queue_suspend_resume(robot: InexbotDriver) -> None:
    section("9. 队列运动 — suspend / resume")

    robot.queue_start()
    _queue_push_work_path(robot, QUEUE_POINTS)
    robot.queue_send(wait=False)
    if not _wait_queue_running_started(robot):
        robot.queue_stop()
        raise RuntimeError("suspend 测试：队列未进入 RUNNING，已 queue_stop")
    _wait_left_first_queue_point(robot, QUEUE_POINTS[0])

    ret_susp = robot.queue_suspend()
    check("queue_suspend 成功", ret_susp == 0, f"ret={ret_susp}")
    time.sleep(0.5)

    try:
        ret_res = robot.queue_resume()
        check("queue_resume 成功", ret_res == 0, f"ret={ret_res}")
        ok_done = robot._wait_queue_done(timeout=600.0, assume_motion_already_started=True)
        if not ok_done:
            robot.queue_stop()
            raise RuntimeError("resume 后 wait_queue_done 失败（可能位姿停滞已 queue_stop）")
        state = robot.get_running_state()
        check("resume 后执行完成，运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
              f"实际={state}")
    except Exception:
        print("  !! suspend/resume 异常，尝试 queue_stop 解除暂停")
        robot.queue_stop()
        raise


def test_queue_stop(robot: InexbotDriver) -> None:
    section("10. 队列运动 — queue_stop 中途停止")

    robot.queue_start()
    _queue_push_work_path(robot, QUEUE_POINTS)
    robot.queue_send(wait=False)
    time.sleep(0.15)

    ret_stop = robot.queue_stop()
    check("queue_stop 成功", ret_stop == 0, f"ret={ret_stop}")
    time.sleep(0.5)

    state = robot.get_running_state()
    check("queue_stop 后运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")
    check("queue_stop 后 _queue_size 已清零",
          robot._queue_size == 0, f"实际={robot._queue_size}")


def test_queue_large(robot: InexbotDriver) -> None:
    section("11. 队列运动 — 超过 31 条自动分批")

    # 全量 all 中 large 位于 stop/suspend 之后，控制器状态机偶发残留导致 large 假跑。
    # 先回 Home 并等待运行状态稳定，再启动分批队列，能显著降低「running=2 但不动」与伺服不一致报警概率。
    try:
        robot.go_home()
        time.sleep(LARGE_PRE_HOME_WAIT)
    except Exception as e:
        logger.warning("[test_queue_large] go_home before large failed: %s", e)

    large_points = []
    for i in range(QUEUE_LARGE_SEGMENTS):
        large_points.append(POS_A if i % 2 == 0 else POS_B)
    large_points.append(HOME_POSE)
    n_pts = len(large_points)

    robot.queue_start()
    _queue_push_work_path(robot, large_points)

    check(
        f"{n_pts}条点位入队（{QUEUE_LARGE_SEGMENTS}交替+Home）计数正确",
        robot._queue_size == n_pts,
        f"实际={robot._queue_size}",
    )

    ret = robot.queue_send(wait=True)
    check(f"{n_pts}条队列分批发送并完成", ret == 0, f"ret={ret}")

    state = robot.get_running_state()
    check("分批执行完成后运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")


def test_queue(robot: InexbotDriver, sub: str = "all") -> None:
    """阶段 5：队列运动。sub: basic | push_j | suspend | stop | large | all"""
    steps = {
        "basic": [test_queue_basic],
        "push_j": [test_queue_push_j],
        "suspend": [test_queue_suspend_resume],
        "stop": [test_queue_stop],
        "large": [test_queue_large],
        "all": [
            test_queue_basic,
            test_queue_push_j,
            test_queue_suspend_resume,
            test_queue_stop,
            test_queue_large,
        ],
    }
    step_list = steps.get(sub, steps["all"])
    # all 连跑时子项间多留时间，减轻控制器轴状态与队列状态机抖动
    gap_after_sub = GAP_AFTER_SUB_ALL if sub == "all" else GAP_AFTER_SUB_SINGLE

    for fn in step_list:
        t_fn = _Timer(fn.__name__)
        try:
            fn(robot)
        finally:
            fn_sec = t_fn.done()
            t_fin = _Timer(fn.__name__ + ".finally")
            if robot._queue_mode_active or robot._queue_size > 0:
                robot.queue_stop()
            if robot.get_servo_state() == SERVO_STATE_ALARM:
                if not robot.recover_servo():
                    logger.warning("[test] servo ALARM, recovery failed")
            finally_sec = t_fin.done()
            time.sleep(gap_after_sub)
            _log_step_timing(fn.__name__, fn_sec, finally_sec, gap_after_sub)

    if sub == "all" and len(step_list) > 1:
        logger.info("[test_queue] all 子项已跑完，额外 queue_stop + 稳定等待")
        try:
            robot.queue_stop()
        except Exception:
            pass
        time.sleep(GAP_AFTER_ALL_EXTRA)


def test_shutdown(robot: InexbotDriver) -> None:
    section("12. shutdown")

    robot.shutdown()
    check("shutdown 后 fd 重置为 -1", robot.fd == -1,
          f"实际={robot.fd}")


def _run_phases(robot: InexbotDriver, phases: List[int], args: argparse.Namespace) -> None:
    """按阶段编号顺序执行测试。"""
    global _aborted

    queue_sub = getattr(args, "queue_sub", "all") if args else "all"

    phase_runners = {
        2: lambda: test_state_query(robot),
        3: lambda: test_is_reachable(robot),
        4: lambda: test_motion(robot),
        5: lambda: test_queue(robot, queue_sub),
    }

    for i, p in enumerate(phases):
        if p == 1:
            continue  # startup 在 main 中单独处理
        if p not in phase_runners:
            skip(f"阶段 {p}", "未知阶段编号")
            continue
        name, _ = PHASES[p]
        try:
            phase_runners[p]()
        except Exception as e:
            _aborted = True
            print(f"\n  !! 阶段 {p} ({name}) 异常：{e}")
            for rest in phases[i + 1:]:
                if rest in PHASES:
                    skip(f"阶段 {rest} {PHASES[rest][0]}", "前序阶段异常中断")
            raise


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inexbot driver2 集成测试")
    parser.add_argument(
        "--phase",
        type=int,
        choices=[1, 2, 3, 4, 5],
        default=None,
        help="仅运行指定阶段（2=状态 3=可达性 4=运动 5=队列）；默认运行全部",
    )
    parser.add_argument(
        "--queue-sub",
        type=str,
        default="basic",
        choices=["basic", "push_j", "suspend", "stop", "large", "all"],
        help="阶段5子项：建议先 basic，通过后再 all",
    )
    parser.add_argument(
        "--mode",
        type=int,
        default=0,
        choices=[0, 1, 2],
        help="SDK 运行模式：0示教 1远程 2运行（官方建议远程联调队列）",
    )
    return parser.parse_args()


# ──────────────────────────────────────────────────────────────────
#  主入口
# ──────────────────────────────────────────────────────────────────

def main() -> None:
    global _aborted
    args = _parse_args()

    if args.phase is not None:
        phases_to_run = [args.phase]
    else:
        phases_to_run = [2, 3, 4, 5]

    print(f"\n{'═' * 55}")
    print(f"  InexbotRobot 集成测试 (inexbot_driver)")
    _MODE_NAMES = {0: "示教", 1: "远程", 2: "运行"}
    print(f"  目标控制器：{ROBOT_IP}:{ROBOT_PORT}")
    print(f"  SDK 模式：{args.mode} ({_MODE_NAMES.get(args.mode, '?')})")
    print(f"  全局速度：{GLOBAL_SPEED_PERCENT}%  队列 MOVL：{QUEUE_MOVEL_SPEED} mm/s")
    if args.phase:
        extra = f", queue-sub={args.queue_sub}" if args.phase == 5 else ""
        print(f"  仅运行阶段：{args.phase} ({PHASES.get(args.phase, ('', ''))[0]}{extra})")
    print(f"{'═' * 55}")

    robot = InexbotDriver(
        ROBOT_IP, ROBOT_PORT, mode=args.mode, global_speed=GLOBAL_SPEED_PERCENT
    )

    # 避免紧接上一次 shutdown 后立即连接导致上电失败
    time.sleep(1.5)

    if not test_startup_shutdown(robot):
        robot.shutdown()
        summary()
        sys.exit(1)

    # 记录未执行阶段为 SKIP
    all_post_startup = [2, 3, 4, 5]
    for p in all_post_startup:
        if p not in phases_to_run:
            skip(f"阶段 {p} {PHASES[p][0]}", "未在 --phase 范围内")

    exit_code = 0
    try:
        _run_phases(robot, phases_to_run, args)
    except KeyboardInterrupt:
        _aborted = True
        print("\n\n  !! 用户中断，执行 queue_stop 后下电")
        if robot.fd >= 0:
            try:
                robot.queue_stop()
            except Exception:
                pass
        exit_code = 1
    except Exception:
        _aborted = True
        exit_code = 1
    finally:
        # 全量/队列测试后控制器可能仍占队列或轴状态未稳；先关队列再下电，减少「伺服不一致」
        # 与示教器侧「作业文件启动失败」连锁报警。
        if robot.fd >= 0:
            try:
                if 5 in phases_to_run or getattr(robot, "_queue_mode_active", False) or getattr(robot, "_queue_size", 0) > 0:
                    robot.queue_stop()
                    time.sleep(0.8)
                if robot.get_servo_state() == SERVO_STATE_ALARM:
                    robot.recover_servo()
                    time.sleep(0.3)
            except Exception:
                pass
        test_shutdown(robot)

    summary()
    if _fail > 0 or _aborted:
        exit_code = 1
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
