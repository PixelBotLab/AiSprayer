# 八位堂遥操作模块 —— 实现计划、步骤与单元测试

> 配套设计文档：[teleop_design.md](./teleop_design.md)
> 目标模块：`app/src/core/teleop/`
> 测试运行环境：`pytest`，`PYTHONPATH=app/src`（与 `apps/robot/tests`、`core/vision/tests` 同一约定）

本文件给出**可落地的实现顺序、每个里程碑的文件清单与验收标准**，并附**完整、可直接落盘的单元测试代码**。测试采用 TDD 契约风格：先固定 `core.teleop` 各模块的公开接口与行为，实现里程碑完成后这些用例即转绿。

---

## 1. 公开 API 契约（实现与测试共同遵循）

### 1.1 `mapping.py`（纯数据，单一真实源）
```python
LOGICAL_AXES = ("LS_X", "LS_Y", "RS_X", "RS_Y", "LT", "RT")   # LS/RS: [-1,1]; LT/RT: [0,1]
LOGICAL_BUTTONS = ("A", "B", "X", "Y", "LB", "RB", "BACK", "START",
                   "L3", "R3", "DP_UP", "DP_DOWN", "DP_LEFT", "DP_RIGHT", "P1", "P2")
MOTION_MODES = ("translation", "rotation", "joint")
FRAMES = ("base", "tool")
CARTESIAN_AXES = ("X", "Y", "Z", "Rx", "Ry", "Rz")
JOINT_AXES = ("J1", "J2", "J3", "J4", "J5", "J6")

# 逻辑摇杆轴 -> (机器人轴 token, 符号)。"SEL" 在运行时解析为当前选中关节 Jn。
AXIS_MAP = {
    "translation": {"LS_X": ("X", +1), "LS_Y": ("Y", +1), "RS_Y": ("Z", +1), "RS_X": ("Rz", +1)},
    "rotation":    {"LS_X": ("Rx", +1), "LS_Y": ("Ry", +1), "RS_X": ("Rz", +1)},
    "joint":       {"LS_X": ("SEL", +1), "RS_X": ("SEL", +2)},  # +2 表示粗调(速率翻倍)
}
```

### 1.2 `device.py`（驱动层：只读 HID、归一化，不含业务）
```python
@dataclass(frozen=True)
class InputState:
    axes: Dict[str, float]      # 见 LOGICAL_AXES，值已归一化
    buttons: Dict[str, bool]    # 见 LOGICAL_BUTTONS
    timestamp: float            # 单调时钟(秒)
    def axis(self, name: str) -> float: ...        # 缺省 0.0
    def pressed(self, name: str) -> bool: ...      # 缺省 False

class BaseTeleopInput(ABC):
    def open(self) -> bool: ...
    def read(self) -> Optional[InputState]: ...   # 非阻塞，无新事件返回 None
    def close(self) -> None: ...
# 具体后端: LinuxGamepadInput(evdev) / PygameGamepadInput(dev) —— 延后到 M1，且不在纯逻辑单测覆盖范围
```

### 1.3 `sink.py`（依赖倒置接口，core 只依赖它）
```python
class TeleopCommandSink(Protocol):
    def is_robot_connected(self) -> bool: ...
    def get_running_state(self) -> int: ...                 # 0=Idle,1=Moving,2=Paused
    def jog(self, axis: str, direction: int, speed_frac: float) -> None: ...  # dir∈{-1,0,1}, speed_frac∈[0,1]
    def stop_all_jog(self) -> None: ...
    def set_spray(self, on: bool) -> None: ...              # 立即指令关/开喷 DO
    def pause(self) -> None: ...
    def resume(self) -> None: ...
    def clear_error(self) -> None: ...
    def emergency_stop(self) -> None: ...                   # 内部含立即关喷
    def go_home(self) -> None: ...
    def go_fold(self) -> None: ...
    def set_speed_factor(self, pct: int) -> None: ...
    def set_frame(self, frame: str) -> None: ...
    def get_gripper_stroke_mm(self) -> float: ...
    def gripper_open(self) -> None: ...
    def gripper_clamp(self, force_percent: int) -> None: ...
    def gripper_move_stroke(self, stroke_mm: float, force_percent: int) -> None: ...
    def broadcast(self, action: str) -> None: ...           # 面向 UI 的英文状态
```
App 层 `RobotCommandSink` 把这些方法逐一映射到既有 `robot_service`（`jog`→`jog_continuous`/`set_global_speed_factor`，`set_spray`→`set_do(...,immediate=True)` 等）。

### 1.4 `teleop.py`（编排层：状态机 + 键位翻译，不含 HTTP/WS）
```python
class TeleopState(Enum):
    DISCONNECTED; IDLE; ENABLED; MOVING; E_STOP

@dataclass(frozen=True)
class TeleopParams:
    deadzone: float = 0.12
    speed_tiers_pct: Tuple[int, ...] = (10, 25, 50, 100)
    reduced_speed_pct: int = 5
    trigger_active_threshold: float = 0.5
    stroke_step_mm: float = 2.0
    force_step_pct: int = 5
    back_tap_max_s: float = 0.4
    watchdog_interval_s: float = 0.1
    watchdog_miss: int = 3
    gripper_min_stroke_mm: float = 0.0
    gripper_max_stroke_mm: float = 50.0
    gripper_low_force_pct: int = 20
    gripper_default_force_pct: int = 50

class TeleopController:
    def __init__(self, sink: TeleopCommandSink, params: TeleopParams | None = None): ...
    # 只读观测属性
    state: TeleopState; mode: str; frame: str
    armed: bool; reduced_speed: bool; spray_on: bool
    speed_tier_pct: int; selected_joint: int; gripper_force_pct: int
    # 主入口
    def update(self, gp: InputState, now: float | None = None) -> None: ...
    def check_watchdog(self, now: float) -> None: ...
```

### 1.5 关键行为规则（测试将逐条断言）
1. `sink.is_robot_connected()==False` → `DISCONNECTED`，丢弃一切运动/夹爪命令。
2. 已连接且机器人 Idle → `IDLE`；`START` 上升沿才 `ENABLED`；机器人非 Idle 时 `START` 被拒并 `broadcast("Cannot enable: robot not idle")`。
3. 仅 `RB` 按住(armed) 且摇杆越过死区才下发 `jog(axis, dir, speed_frac)`，进入 `MOVING`；`RB` 释放或摇杆回中 → `stop_all_jog()` 回 `ENABLED`。
4. `speed_frac = (eff_pct/100) * |axis|`，`LB` 按住时 `eff_pct = reduced_speed_pct`，否则取当前档；`|axis| < deadzone` 不下发。
5. `BACK` 单击(短按且未被占用)=循环模式；`BACK` 按住=夹爪/末端修饰层（此时扳机/十字键归夹爪，`back_consumed=True`，释放不再循环模式）。
6. `A`=切换喷涂（立即指令），`B`=暂停/恢复（依 `get_running_state`），`X`=清障，`Y`=急停。
7. **急停/看门狗序列固定顺序**：`set_spray(False)` → `stop_all_jog()` → `emergency_stop()`，状态→`E_STOP`；**全程绝不调用任何 `gripper_*`（Fail-Hold，防止掉件）**。
8. `E_STOP` 下运动被屏蔽；仅 `X`(clear_error) 后 `START` 重新握手才可离开。
9. 夹爪命令仅 `IDLE/ENABLED/MOVING` 允许，`DISCONNECTED/E_STOP` 拒绝；夹爪不依赖 `RB`。
10. `P2`=张开、`P1`=夹紧(用 `gripper_force_pct`)、`LB+P1`=低力夹紧；无背键回退 `BACK+RT`=张开、`BACK+LT`=夹紧；`BACK+DP_UP/DOWN`=行程 ±`stroke_step_mm`（限幅 `min~max`）；`BACK+DP_LEFT/RIGHT`=力度 ∓/± `force_step_pct`（限幅 1~100）。
11. `Joint` 模式下无 `BACK` 时 `DP_LEFT/RIGHT` 选关节 J1..J6。
12. 机器人 `get_running_state()!=0` 时禁止**启动**新点动与 Home/Fold（仅允许停止/急停）。

---

## 2. 实现里程碑与步骤

| 里程碑 | 交付文件 | 关键实现点 | 验收（对应测试） |
|---|---|---|---|
| **M0 骨架** | `teleop.py`/`sink.py`/`mapping.py`/`state.py`/`device.py` 空实现 + `tests/` 落盘 | 定义 §1 全部签名与枚举 | 契约评审通过；测试 collection 就绪（实现前红） |
| **M1 数据层** | `mapping.py`, `device.py`(`InputState`) | 纯数据表 + `axis()/pressed()` + 死区/归一化工具 | `test_mapping.py` |
| **M2 状态机** | `state.py` + `TeleopController` 的使能/armed/急停/看门狗骨架 | §1.5 规则 1/2/3/7/8 | `test_teleop_state.py`, `test_teleop_safety.py` |
| **M3 运动编排** | `TeleopController` 三模式轴映射、比例速度、降速、坐标系/关节选择 | 规则 3/4/5(模式)/11/12 | `test_teleop_motion.py` |
| **M4 夹爪编排** | `TeleopController` 夹爪层（背键/回退/步进/力度/Fail-Hold 复核） | 规则 5(修饰)/7/9/10 | `test_teleop_gripper.py` |
| **M5 HID 后端 + 装配** | `device.py`(`LinuxGamepadInput`) + `apps/teleop/`(`RobotCommandSink`、`TeleopService`、生命周期、轮询线程、WS 广播) + `config.py`/yaml 增 `hardware.teleop` | 真机联调、看门狗定时驱动 | 真机低速点动 + 急停/夹爪保持回归（手工，非单测） |

> 纯逻辑单测（M1–M4）用 `RecordingSink` + 直接构造 `InputState`，**不依赖 HID/真机/事件循环**，可在开发机与 CI 秒级运行；`device.py` 的真实 evdev/pygame 后端属 M5 集成层，单测不覆盖，交由真机验收。

**每个里程碑统一落地步骤：**
1. 先放开本里程碑对应的测试文件（§3 中的代码原样落盘到 `app/src/core/teleop/tests/`）。
2. 实现对应模块，跑到 `pytest` 该测试文件全绿。
3. 自检 Skill 红线：无死代码/无重复分支、`mm/rad` 内部量纲、面向用户 `broadcast` 文案全英文、异常路径先关喷 DO。
4. `pytest src/core/teleop -q` 与既有 `src/apps/robot/tests` 一并回归，确认零副作用。

---

## 3. 完整单元测试代码

### 3.1 `app/src/core/teleop/tests/conftest.py`
```python
# -*- coding: utf-8 -*-
"""Shared test doubles for teleop pure-logic tests (no HID / no real robot)."""
import os
import sys

import pytest

# Make `core.*` importable regardless of invocation cwd.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..") + "/src"))

from core.teleop.device import InputState
from core.teleop.mapping import LOGICAL_AXES, LOGICAL_BUTTONS
from core.teleop.teleop import TeleopController, TeleopParams


def _make_gp(axes=None, buttons=None, ts=0.0):
    a = {"LS_X": 0.0, "LS_Y": 0.0, "RS_X": 0.0, "RS_Y": 0.0, "LT": 0.0, "RT": 0.0}
    a.update(axes or {})
    b = {k: False for k in LOGICAL_BUTTONS}
    b.update(buttons or {})
    return InputState(axes=a, buttons=b, timestamp=float(ts))


class RecordingSink:
    """Deterministic TeleopCommandSink double: records every call with args."""

    def __init__(self, connected=True, running_state=0, gripper_stroke=0.0):
        self.calls = []                       # list[(name, args_tuple)]
        self._connected = connected
        self._running_state = running_state
        self._gripper_stroke = gripper_stroke

    def _rec(self, name, *args):
        self.calls.append((name, args))

    # ---- queries ----
    def is_robot_connected(self):
        return self._connected

    def get_running_state(self):
        return self._running_state

    def get_gripper_stroke_mm(self):
        return self._gripper_stroke

    # ---- commands ----
    def jog(self, axis, direction, speed_frac):
        self._rec("jog", axis, direction, speed_frac)

    def stop_all_jog(self):
        self._rec("stop_all_jog")

    def set_spray(self, on):
        self._rec("set_spray", on)

    def pause(self):
        self._rec("pause")

    def resume(self):
        self._rec("resume")

    def clear_error(self):
        self._rec("clear_error")

    def emergency_stop(self):
        self._rec("emergency_stop")

    def go_home(self):
        self._rec("go_home")

    def go_fold(self):
        self._rec("go_fold")

    def set_speed_factor(self, pct):
        self._rec("set_speed_factor", pct)

    def set_frame(self, frame):
        self._rec("set_frame", frame)

    def gripper_open(self):
        self._rec("gripper_open")

    def gripper_clamp(self, force_percent):
        self._rec("gripper_clamp", force_percent)

    def gripper_move_stroke(self, stroke_mm, force_percent):
        self._rec("gripper_move_stroke", stroke_mm, force_percent)

    def broadcast(self, action):
        self._rec("broadcast", action)

    # ---- assertions helpers ----
    def names(self):
        return [c[0] for c in self.calls]

    def find(self, name):
        return [c[1] for c in self.calls if c[0] == name]

    def count(self, name):
        return sum(1 for c in self.calls if c[0] == name)

    def last(self, name):
        hits = self.find(name)
        return hits[-1] if hits else None


@pytest.fixture
def sink():
    return RecordingSink()


@pytest.fixture
def gp():
    return _make_gp


@pytest.fixture
def make_controller(sink):
    def _make(**overrides):
        return TeleopController(sink, TeleopParams(**overrides))
    return _make


@pytest.fixture
def enabled(make_controller, gp, sink):
    """Return a controller already transitioned to ENABLED."""
    c = make_controller()
    c.update(gp(ts=1.0))                       # IDLE (connected + idle)
    c.update(gp(buttons={"START": True}, ts=2.0))  # ENABLED (START rising edge)
    assert sink.count("emergency_stop") == 0
    return c
```

### 3.2 `app/src/core/teleop/tests/test_mapping.py`
```python
# -*- coding: utf-8 -*-
"""M1: mapping tables + InputState normalization (pure data)."""
from core.teleop.mapping import (
    AXIS_MAP, LOGICAL_AXES, LOGICAL_BUTTONS, MOTION_MODES, CARTESIAN_AXES, JOINT_AXES,
)


def test_axis_map_covers_all_modes():
    assert set(AXIS_MAP.keys()) == set(MOTION_MODES)
    for mode in MOTION_MODES:
        for logical in AXIS_MAP[mode]:
            assert logical in LOGICAL_AXES, f"{logical} not a declared axis"
            token, sign = AXIS_MAP[mode][logical]
            assert sign in (-2, -1, 1, 2)
            assert token in CARTESIAN_AXES or token in JOINT_AXES or token == "SEL"


def test_translation_maps_six_dof_reachable():
    tokens = {tok for tok, _ in AXIS_MAP["translation"].values()}
    # 平移层至少覆盖 X / Y / Z 三个平移轴
    assert {"X", "Y", "Z"} <= tokens


def test_rotation_and_joint_layers_exist():
    assert {t for t, _ in AXIS_MAP["rotation"].values()} >= {"Rx", "Ry"}
    assert AXIS_MAP["joint"]["LS_X"][0] == "SEL"


def test_input_state_defaults_and_lookup(gp):
    st = gp()
    assert st.axis("LS_X") == 0.0
    assert st.pressed("A") is False
    st2 = gp(axes={"LS_X": 0.7}, buttons={"A": True}, ts=3.5)
    assert st2.axis("LS_X") == 0.7
    assert st2.pressed("A") is True
    assert st2.timestamp == 3.5
```

### 3.3 `app/src/core/teleop/tests/test_teleop_state.py`
```python
# -*- coding: utf-8 -*-
"""M2: state machine transitions (rules 1/2/3)."""
from core.teleop.teleop import TeleopState


def test_starts_disconnected_until_robot_online(gp, make_controller, sink):
    sink._connected = False
    c = make_controller()
    c.update(gp(ts=1.0))
    assert c.state == TeleopState.DISCONNECTED

    sink._connected = True
    c.update(gp(ts=2.0))
    assert c.state == TeleopState.IDLE


def test_enable_requires_start_edge_and_robot_idle(gp, make_controller, sink):
    c = make_controller()
    c.update(gp(ts=1.0))
    assert c.state == TeleopState.IDLE
    # Robot moving: START must be rejected, stays IDLE, broadcasts English notice.
    sink._running_state = 1
    c.update(gp(buttons={"START": True}, ts=2.0))
    assert c.state == TeleopState.IDLE
    assert any("not idle" in a[0].lower() for a in sink.find("broadcast"))

    # Robot idle: START enables.
    sink._running_state = 0
    c.update(gp(ts=2.5))                       # release START to get a clean edge
    c.update(gp(buttons={"START": True}, ts=3.0))
    assert c.state == TeleopState.ENABLED


def test_jog_requires_deadman_rb(gp, enabled, gp_unused=None):
    pass  # placeholder removed at implementation time


def test_motion_needs_rb_and_deadzone(gp, sink, make_controller):
    c = make_controller()
    c.update(gp(ts=1.0))
    c.update(gp(buttons={"START": True}, ts=2.0))
    # No RB: pushing stick must not move.
    c.update(gp(axes={"LS_X": 0.9}, ts=3.0))
    assert c.state == TeleopState.ENABLED
    assert sink.count("jog") == 0

    # RB held but within deadzone: still no motion.
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.05}, ts=3.5))
    assert sink.count("jog") == 0

    # RB held beyond deadzone: MOVING + one jog.
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.9}, ts=4.0))
    assert c.state == TeleopState.MOVING
    assert sink.count("jog") == 1


def test_release_rb_stops_motion(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"RB": True}, axes={"LS_Y": 0.8}, ts=3.0))
    assert c.state == TeleopState.MOVING
    c.update(gp(buttons={}, ts=3.5))          # RB released
    assert c.state == TeleopState.ENABLED
    assert sink.count("stop_all_jog") >= 1
```
> 说明：`test_jog_requires_deadman_rb` 仅为占位示意，实现时删除；核心覆盖在 `test_motion_needs_rb_and_deadzone`。

### 3.4 `app/src/core/teleop/tests/test_teleop_motion.py`
```python
# -*- coding: utf-8 -*-
"""M3: three-mode axis mapping, proportional/reduced speed, frame & joint select (rules 3/4/5/11/12)."""
from core.teleop.teleop import TeleopState


def _to_rotation(c, gp):
    c.update(gp(ts=10.0))
    c.update(gp(buttons={"BACK": True}, ts=10.05))   # short tap
    c.update(gp(ts=10.1))


def _to_joint(c, gp):
    _to_rotation(c, gp)
    c.update(gp(buttons={"BACK": True}, ts=11.0))
    c.update(gp(ts=11.05))


def test_translation_ls_x_maps_to_x_axis(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.8}, ts=3.0))
    axis, direction, frac = sink.last("jog")
    assert axis == "X" and direction == 1
    assert 0.30 < frac <= 0.55                    # tier 50% * 0.8 ~= 0.4


def test_proportional_speed_monotonic_with_deflection(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.3}, ts=3.0))
    slow = sink.last("jog")[2]
    c.update(gp(ts=3.2))                          # recenter
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.95}, ts=3.4))
    fast = sink.last("jog")[2]
    assert fast > slow


def test_negative_direction_sign(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"RB": True}, axes={"LS_X": -0.7}, ts=3.0))
    axis, direction, _ = sink.last("jog")
    assert axis == "X" and direction == -1


def test_reduced_speed_cap_with_lb(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"RB": True, "LB": True}, axes={"LS_X": 0.9}, ts=3.0))
    assert c.reduced_speed is True
    frac = sink.last("jog")[2]
    assert frac <= 0.10                           # reduced 5% * 0.9 ~= 0.045


def test_rotation_mode_maps_orientation_axis(gp, sink, make_controller):
    c = make_controller()
    c.update(gp(ts=1.0))
    c.update(gp(buttons={"START": True}, ts=2.0))
    _to_rotation(c, gp)
    assert c.mode == "rotation"
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.8}, ts=12.0))
    axis, direction, _ = sink.last("jog")
    assert axis == "Rx" and direction == 1


def test_joint_mode_select_and_jog(gp, sink, make_controller):
    c = make_controller()
    c.update(gp(ts=1.0))
    c.update(gp(buttons={"START": True}, ts=2.0))
    _to_joint(c, gp)
    assert c.mode == "joint"
    assert c.selected_joint == 1
    c.update(gp(buttons={"DP_RIGHT": True}, ts=12.0))     # select J2
    assert c.selected_joint == 2
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.8}, ts=12.5))
    axis, direction, _ = sink.last("jog")
    assert axis == "J2" and direction == 1


def test_mode_cycle_wraps_back_to_translation(gp, sink, make_controller):
    c = make_controller()
    c.update(gp(ts=1.0))
    c.update(gp(buttons={"START": True}, ts=2.0))
    _to_joint(c, gp)                              # rotation -> joint
    # one more tap -> translation
    c.update(gp(buttons={"BACK": True}, ts=13.0))
    c.update(gp(ts=13.05))
    assert c.mode == "translation"


def test_running_arm_interlocks_new_jog(enabled, gp, sink):
    c = enabled
    sink._running_state = 1                       # external program took over
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.8}, ts=5.0))
    assert sink.count("jog") == 0                 # new motion blocked; stops still allowed
```

### 3.5 `app/src/core/teleop/tests/test_teleop_safety.py`
```python
# -*- coding: utf-8 -*-
"""M2/M4: spray, e-stop fail-close order, watchdog, gripper fail-Hold, interlocks (rules 6/7/8/12)."""
from core.teleop.teleop import TeleopState


def test_spray_toggle_uses_immediate_do(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"A": True}, ts=3.0))
    assert sink.last("set_spray") == (True,)
    assert c.spray_on is True
    c.update(gp(ts=3.2))
    c.update(gp(buttons={"A": True}, ts=3.4))
    assert sink.last("set_spray") == (False,)
    assert c.spray_on is False


def test_pause_resume_depend_on_running_state(enabled, gp, sink):
    c = enabled
    sink._running_state = 1
    c.update(gp(buttons={"B": True}, ts=3.0))
    assert sink.count("pause") == 1
    sink._running_state = 2
    c.update(gp(ts=3.1))
    c.update(gp(buttons={"B": True}, ts=3.2))
    assert sink.count("resume") == 1


def test_emergency_stop_sequence_order_and_no_gripper(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.8}, ts=3.0))   # MOVING
    c.update(gp(buttons={"Y": True}, ts=3.5))                        # E-STOP
    seq = sink.names()
    # 断料(spray OFF) 必须在停 jog 与 emergency_stop 之前
    assert seq.index(("set_spray")) <= seq.index("stop_all_jog") <= seq.index("emergency_stop")
    assert sink.find("set_spray")[-1] == (False,)
    assert c.state == TeleopState.E_STOP
    # Fail-Hold: 急停全程不得触碰夹爪
    assert sink.count("gripper_open") == 0
    assert sink.count("gripper_clamp") == 0
    assert sink.count("gripper_move_stroke") == 0


def test_e_stop_blocks_motion_until_clear_and_reenable(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"Y": True}, ts=3.0))
    assert c.state == TeleopState.E_STOP
    sink.calls.clear()
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.8}, ts=4.0))   # ignored
    assert sink.count("jog") == 0
    c.update(gp(buttons={"X": True}, ts=5.0))                        # clear_error
    assert sink.count("clear_error") == 1
    c.update(gp(ts=5.1))
    c.update(gp(buttons={"START": True}, ts=5.2))                    # re-handshake
    assert c.state != TeleopState.E_STOP


def test_watchdog_timeout_triggers_safe_stop_no_gripper(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"RB": True}, axes={"LS_X": 0.8}, ts=3.0))
    sink.calls.clear()
    # 事件长时间未更新 -> 看门狗判失控
    c.check_watchdog(now=3.0 + 0.1 * 10)
    assert sink.count("set_spray") >= 1 and sink.count("stop_all_jog") >= 1
    assert c.state == TeleopState.E_STOP
    assert sink.count("gripper_open") == 0 and sink.count("gripper_clamp") == 0


def test_home_fold_require_idle(enabled, gp, sink):
    c = enabled
    # 非 Idle 时 RB+L3 不应触发回原点
    sink._running_state = 1
    c.update(gp(buttons={"RB": True, "L3": True}, ts=4.0))
    assert sink.count("go_home") == 0
    sink._running_state = 0
    c.update(gp(ts=4.1))
    c.update(gp(buttons={"RB": True, "L3": True}, ts=4.2))
    assert sink.count("go_home") == 1


def test_disconnected_drops_everything(gp, make_controller, sink):
    sink._connected = False
    c = make_controller()
    c.update(gp(buttons={"RB": True, "A": True, "P2": True}, axes={"LS_X": 0.9}, ts=1.0))
    assert c.state == TeleopState.DISCONNECTED
    assert sink.count("jog") == 0 and sink.count("set_spray") == 0 and sink.count("gripper_open") == 0
```

### 3.6 `app/src/core/teleop/tests/test_teleop_gripper.py`
```python
# -*- coding: utf-8 -*-
"""M4: gripper mapping incl. rear paddles, BACK-hold fallback, stroke/force stepping (rules 9/10)."""
from core.teleop.teleop import TeleopState


def _hold_back(c, gp, ts, **kw):
    axes = {"BACK_AXES": 0} and kw.pop("axes", None)
    c.update(gp(buttons={"BACK": True}, axes=axes, ts=ts))


def test_rear_paddle_open_and_clamp(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"P2": True}, ts=3.0))
    assert sink.count("gripper_open") == 1
    c.update(gp(ts=3.1))
    c.update(gp(buttons={"P1": True}, ts=3.2))
    assert sink.count("gripper_clamp") == 1
    assert sink.last("gripper_clamp")[0] == c.gripper_force_pct


def test_low_force_clamp_with_lb(enabled, gp, sink):
    c = enabled
    c.update(gp(buttons={"LB": True, "P1": True}, ts=3.0))
    force = sink.last("gripper_clamp")[0]
    assert force < c.gripper_force_pct or force == 20   # 低力档


def test_back_hold_fallback_triggers_without_paddles(gp, sink, make_controller):
    c = make_controller()
    c.update(gp(ts=1.0))
    c.update(gp(buttons={"START": True}, ts=2.0))
    c.update(gp(buttons={"BACK": True}, axes={"RT": 1.0}, ts=3.0))   # BACK+RT -> open
    assert sink.count("gripper_open") == 1
    c.update(gp(buttons={"BACK": True}, ts=3.1))
    c.update(gp(buttons={"BACK": True}, axes={"LT": 1.0}, ts=3.2))   # BACK+LT -> clamp
    assert sink.count("gripper_clamp") == 1
    # BACK 已被夹爪占用，释放不得循环运动模式
    mode_before = c.mode
    c.update(gp(ts=3.3))
    assert c.mode == mode_before


def test_stroke_step_up_and_clamped_at_max(enabled, gp, sink):
    c = enabled
    sink._gripper_stroke = 49.0
    c.update(gp(buttons={"BACK": True, "DP_UP": True}, ts=4.0))       # +2mm -> clamp to 50
    stroke = sink.last("gripper_move_stroke")[0]
    assert stroke <= 50.0 and stroke >= 49.0


def test_stroke_step_down(enabled, gp, sink):
    c = enabled
    sink._gripper_stroke = 20.0
    c.update(gp(buttons={"BACK": True, "DP_DOWN": True}, ts=4.0))     # -2mm -> 18
    assert sink.last("gripper_move_stroke")[0] == 18.0


def test_force_step_up_and_bounds(enabled, gp, sink):
    c = enabled
    before = c.gripper_force_pct
    c.update(gp(buttons={"BACK": True, "DP_RIGHT": True}, ts=4.0))    # +5%
    assert c.gripper_force_pct == before + 5
    # push to ceiling
    for i in range(30):
        c.update(gp(ts=5.0 + i * 0.1))
        c.update(gp(buttons={"BACK": True, "DP_RIGHT": True}, ts=5.05 + i * 0.1))
    assert c.gripper_force_pct <= 100


def test_gripper_rejected_in_estop_and_disconnected(gp, make_controller, sink):
    # E_STOP 拒绝夹爪（保持 Fail-Hold 语义一致）
    c = make_controller()
    c.update(gp(ts=1.0))
    c.update(gp(buttons={"START": True}, ts=2.0))
    c.update(gp(buttons={"Y": True}, ts=3.0))
    assert c.state == TeleopState.E_STOP
    sink.calls.clear()
    c.update(gp(buttons={"P2": True}, ts=4.0))
    assert sink.count("gripper_open") == 0
```

---

## 4. 运行测试

```bash
cd app
# M1 之前：契约测试会因模块未实现而 collection 失败（预期，属 TDD 红灯）
PYTHONPATH=src python -m pytest src/core/teleop/tests -q

# 单里程碑增量验证
PYTHONPATH=src python -m pytest src/core/teleop/tests/test_mapping.py -v
PYTHONPATH=src python -m pytest src/core/teleop/tests/test_teleop_safety.py -v

# 全量回归（确保未影响既有机器人服务测试）
PYTHONPATH=src python -m pytest src/core/teleop src/apps/robot/tests -q
```

---

## 5. 完成定义（Definition of Done）

- [ ] `mapping/state/teleop/sink/device(InputState)` 五模块实现，`TeleopController.update/check_watchdog` 覆盖 §1.5 全部 12 条规则。
- [ ] `tests/` 全部用例转绿；新增分支均有对应断言（使能门控、dead-man、比例/降速、三模式、急停顺序、看门狗、夹爪 Fail-Hold、运行互锁、断开丢弃）。
- [ ] 急停/看门狗/断连三条路径均验证"先关喷 DO、绝不张开夹爪"。
- [ ] 所有 `broadcast(...)` 文案为规范英文；无死代码/无重复判定；内部量纲 `mm`/`rad`、封包 `deg`。
- [ ] `pytest src/core/teleop src/apps/robot/tests` 全绿，未破坏既有用例。
- [ ] M5 真机验收：低速点动、模式/坐标系/关节切换、喷涂开关、夹爪开合与行程/力度步进、E-STOP 与掉线看门狗回归通过。
```
