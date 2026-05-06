"""
InexbotRobot 集成测试文件

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
"""

import math
import time
import sys
import os
import logging

# 1. 路径锚定策略
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.hardware.robot.inexbot_driver2 import InexbotDriver, RobotPose
from aisprayer.core.hardware.robot.inexbot_driver2 import (
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

# 测试用点位（直角坐标，姿态角单位：rad）
# 请根据实际机器人工作空间调整，确保安全！
POS_A      = [400.0,    0.0,  1077.0,  -3.141,  0.0,    0.0]   # 测试点 A
POS_B      = [400.0,  100.0,  1000.0,  -3.141,  0.0,    0.0]   # 测试点 B
POS_C      = [350.0,  100.0,  950.0,  -3.141,  0.0,    0.0]   # 测试点 C
POS_UNREACHABLE = [9999.0, 9999.0, 9999.0, -3.141, 0.0, 0.0]     # 明显不可达点

# 队列测试点位列表（5 条）
QUEUE_POINTS = [
    [400.0,   0.0, 950.0, -3.141,  0.0, 0.0],
    [420.0,  40.0, 960.0, -3.141,  0.0, 0.0],
    [440.0,  80.0, 970.0, -3.141,  0.0, 0.0],
    [420.0, 120.0, 980.0, -3.141,  0.0, 0.0],
    [400.0,  80.0, 990.0, -3.141,  0.0, 0.0],
]


# ──────────────────────────────────────────────────────────────────
#  测试工具
# ──────────────────────────────────────────────────────────────────

_pass = 0
_fail = 0


def check(name: str, condition: bool, detail: str = "") -> None:
    global _pass, _fail
    status = "PASS" if condition else "FAIL"
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{status}] {name}{suffix}")
    if condition:
        _pass += 1
    else:
        _fail += 1


def section(title: str) -> None:
    print(f"\n{'─' * 55}")
    print(f"  {title}")
    print(f"{'─' * 55}")


def summary() -> None:
    total = _pass + _fail
    print(f"\n{'═' * 55}")
    print(f"  测试结果：{_pass}/{total} 通过，{_fail} 失败")
    print(f"{'═' * 55}\n")


# ──────────────────────────────────────────────────────────────────
#  各功能测试函数
# ──────────────────────────────────────────────────────────────────

def test_startup_shutdown(robot: InexbotDriver) -> bool:
    section("1. startup / shutdown")

    ok = robot.startup(timeout=15.0)
    check("startup 返回 True", ok)
    if not ok:
        print("  !! startup 失败，后续测试跳过")
        return False

    servo = robot.get_servo_state()
    check("startup 后伺服状态为 RUNNING(3)", servo == SERVO_STATE_RUNNING,
          f"实际={servo}")
    return True


def test_state_query(robot: InexbotDriver) -> None:
    section("2. 状态查询")

    servo = robot.get_servo_state()
    check("get_servo_state 返回合法值 [0-3]", 0 <= servo <= 3,
          f"实际={servo}")

    running = robot.get_running_state()
    check("get_running_state 返回合法值 [0-2]", 0 <= running <= 2,
          f"实际={running}")

    pos = robot.get_current_pose()
    check("get_current_pose 返回非空列表", isinstance(pos, list) and len(pos) >= 6,
          f"长度={len(pos)}")
    print(f"  当前位置: {[round(v, 4) for v in pos[:6]]}")


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

    ret = robot.move_j(POS_A, velocity=30)
    check("move_j 到 POS_A 成功（wait=True）", ret == 0, f"ret={ret}")

    state = robot.get_running_state()
    check("move_j 完成后运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")

    pos = robot.get_current_pose()
    print(f"  到达位置: {[round(v, 4) for v in pos[:6]]}")


def test_move_l(robot: InexbotDriver) -> None:
    section("5. move_l 直线运动")

    ret = robot.move_l(POS_B, velocity=100)
    check("move_l 到 POS_B 成功（wait=True）", ret == 0, f"ret={ret}")

    state = robot.get_running_state()
    check("move_l 完成后运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")

    # 不等待（wait=False），手动等待
    ret2 = robot.move_l(POS_A, velocity=100, wait=False)
    check("move_l wait=False 下发成功", ret2 == 0, f"ret={ret2}")
    time.sleep(0.2)
    robot._wait_motion_done()
    state2 = robot.get_running_state()
    check("手动等待后运行状态为 STOP(0)", state2 == RUNNING_STATE_STOP,
          f"实际={state2}")

def test_go_home(robot: InexbotDriver) -> None:
    section("6. go_home 回零点")

    ret = robot.go_home()
    check("go_home 成功（wait=True）", ret == 0, f"ret={ret}")

    state = robot.get_running_state()
    check("go_home 完成后运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")


def test_queue_basic(robot: InexbotDriver) -> None:
    section("7. 队列运动 — 基本流程（push_l + send）")

    ret_open = robot.queue_start()
    check("queue_start 成功", ret_open == 0, f"ret={ret_open}")

    for i, p in enumerate(QUEUE_POINTS):
        pl = 0 if i == len(QUEUE_POINTS) - 1 else 3
        ret = robot.queue_push_l(p, velocity=150, pl=pl)
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
    ret2 = robot.queue_push_j(POS_B, velocity=30, pl=0)
    check("queue_push_j x2 插入成功", ret1 == 0 and ret2 == 0,
          f"ret1={ret1}, ret2={ret2}")

    ret_send = robot.queue_send(wait=True)
    check("queue_push_j 发送并等待完成", ret_send == 0, f"ret={ret_send}")


def test_queue_suspend_resume(robot: InexbotDriver) -> None:
    section("9. 队列运动 — suspend / resume")

    robot.queue_start()
    for p in QUEUE_POINTS:
        robot.queue_push_l(p, velocity=80, pl=3)
    # 不等待，发送后立刻暂停
    robot.queue_send(wait=False)
    time.sleep(0.3)

    ret_susp = robot.queue_suspend()
    check("queue_suspend 成功", ret_susp == 0, f"ret={ret_susp}")
    time.sleep(0.5)

    ret_res = robot.queue_resume()
    check("queue_resume 成功", ret_res == 0, f"ret={ret_res}")

    # 等待最终完成
    robot._wait_queue_done()
    state = robot.get_running_state()
    check("resume 后执行完成，运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")


def test_queue_stop(robot: InexbotDriver) -> None:
    section("10. 队列运动 — queue_stop 中途停止")

    robot.queue_start()
    for p in QUEUE_POINTS:
        robot.queue_push_l(p, velocity=80, pl=3)
    robot.queue_send(wait=False)
    time.sleep(0.3)

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

    # 生成 40 条点位（在两个点之间来回）
    large_points = []
    for i in range(40):
        if i % 2 == 0:
            large_points.append(POS_A)
        else:
            large_points.append(POS_B)

    robot.queue_start()
    for i, p in enumerate(large_points):
        pl = 0 if i == len(large_points) - 1 else 2
        robot.queue_push_l(p, velocity=200, pl=pl)

    check("40条点位入队计数正确", robot._queue_size == 40,
          f"实际={robot._queue_size}")

    ret = robot.queue_send(wait=True)
    check("40条队列分批发送并完成", ret == 0, f"ret={ret}")

    state = robot.get_running_state()
    check("分批执行完成后运行状态为 STOP(0)", state == RUNNING_STATE_STOP,
          f"实际={state}")


def test_shutdown(robot: InexbotDriver) -> None:
    section("12. shutdown")

    robot.shutdown()
    check("shutdown 后 fd 重置为 -1", robot.fd == -1,
          f"实际={robot.fd}")

# ──────────────────────────────────────────────────────────────────
#  主入口
# ──────────────────────────────────────────────────────────────────

def main() -> None:
    print(f"\n{'═' * 55}")
    print(f"  InexbotRobot 集成测试")
    print(f"  目标控制器：{ROBOT_IP}:{ROBOT_PORT}")
    print(f"{'═' * 55}")

    robot = InexbotDriver(ROBOT_IP, ROBOT_PORT)

    # startup 失败则直接退出
    if not test_startup_shutdown(robot):
        summary()
        sys.exit(1)

    try:
        test_state_query(robot)
        test_is_reachable(robot)
        test_move_j(robot)
        test_move_l(robot)
        test_go_home(robot)
        test_queue_basic(robot)
        test_queue_push_j(robot)
        test_queue_suspend_resume(robot)
        test_queue_stop(robot)
        test_queue_large(robot)
    except KeyboardInterrupt:
        print("\n\n  !! 用户中断，执行紧急停止并下电")
        robot.queue_stop()
    except Exception as e:
        print(f"\n  !! 测试异常：{e}")
    finally:
        test_shutdown(robot)

    summary()
    sys.exit(0 if _fail == 0 else 1)


if __name__ == "__main__":
    main()