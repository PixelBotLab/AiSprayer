# 八位堂（8BitDo）遥控手柄机械臂遥操作模块设计方案

> 模块路径：`app/src/core/teleop/`（按职责命名，主输入设备为八位堂手柄）
> 状态：**设计阶段（Design / Not Implemented）**
> 适用范围：使用标准 HID 游戏手柄（8BitDo Pro 2 / Ultimate / SN30 Pro 系列，X-Input / D-Input 模式）对 CR5 机械臂进行手动示教与点动遥操作。

---

## 0. 设计目标与非目标

### 0.1 目标
1. 让现场操作员脱离 Web UI，用手柄完成**低速点动、坐标系/模式切换、喷涂开关、急停**等高频手动动作，缩短示教回路。
2. 严格复用现有三层架构（驱动层 `BaseRobotDriver` → 服务层 `RobotService` → 应用/路由层），**不新增平行的机械臂通信通道**。
3. 键位映射遵循工业机器人示教器（Teach Pendant）与遥操作行业规范，做到"上手即安全、松手即停止"。
4. 全流程满足**故障安全（Fail-Close）**：任何异常、断连、超时，首要动作是关喷涂 DO（立即指令）并停止点动。

### 0.2 非目标
- 不做连续轨迹喷涂（自动航点执行仍由服务端 `move_l_segments` 完成，手柄只负责手动点动与状态编排）。
- 不在 core 层直接依赖 FastAPI / WebSocket，也不感知工件模板、相机等业务概念。
- 不在本期实现力/位混合、视觉伺服等高级功能，仅预留映射扩展位。

---

## 1. 遵循的行业规范与标准（Design Baseline）

| 标准 | 主题 | 本模块落地方式 |
|---|---|---|
| **USB HID Usage Tables**（Generic Desktop Page `0x04`, Usage `Gamepad 0x05`） | 手柄轴/按键语义 | 设备层按 HID axis/button 归一化读取，屏蔽具体厂商报文差异 |
| **Linux `evdev` / input-joystick（`ABS_*` / `BTN_*`）** 与 **SDL2 Game Controller** 标准映射 | 跨手柄抽象 | 以"逻辑按键名"（见 §4）为中间层，底层可接 `evdev` 或 `pygame`/`inputs`，上层不感知 |
| **ISO 10218-1 / -2:2011**（工业机器人安全；示教器功能） | 示教速度、使能装置、停机 | 三层速率分档 + 使能（dead-man）按钮 + 松手即停 |
| **ISO/TS 15066:2016**（协作机器人；功率与力限制、手引导） | 手动引导安全 | 低速档限速、连续动作需持续按住使能键 |
| **IEC 60947-5-6**（三段式使能装置）/ **IEC 61317**（机器人急停功能） | 使能与急停语义 | 使能键按下=armed；松开=解除并停止；急停键=最高优先级立即断料 |
| **ISO 13849-1 (PL) / IEC 62061** | 安全相关控制系统 | 急停/关喷走立即指令，独立于运动队列，不经业务逻辑判断 |
| **IEC 62682（报警管理）/ ISA-101（HMI 配色）** | 操作员状态反馈 | 手柄 LED 用绿/琥珀/红表征 ENABLED / 限速armed / E-STOP（见 §6） |
| 项目内 `AGENTS.md` + Skill `code-quality-and-robotics-practices` | 架构/量纲/英文界面 | 内部 `mm`+`rad`，封包 `deg`；所有面向用户的广播/报错文案 100% 英文 |

> 关键取舍：手柄**没有**工业示教器自带的三段式硬件使能开关（释放/按下/超程三态）。本设计用"按住式使能键（dead-man）+ 松手即停"逼近 IEC 60947-5-6 的**一态释放即停**语义，并在文档显式声明该**降级**：本模块仅用于**低速示教点动**，不得替代安全回路中的硬件急停/使能装置。

---

## 2. 模块架构与分层（High Cohesion / Low Coupling）

```
┌────────────────────────────────────────────────────────────┐
│ apps/teleop (App 层，后续实现，本设计仅约定接口)              │
│  - TeleopService: 生命周期 / 与前端 WS 状态广播 / 装配         │
│  - RobotCommandSink: 把 teleop 命令翻译成 robot_service 调用  │
└───────────────▲───────────────────────────────┬────────────┘
                │ 注入 sink (DIP)                │ 读状态
┌───────────────┴───────────────────────────────▼────────────┐
│ core/teleop (Core 层，本模块)                                │
│  device.py    BaseTeleopInput + 事件轮询线程（HID 协议层）   │
│  mapping.py   逻辑按键名 + 键位映射表（可配置，纯数据）        │
│  state.py     遥操作状态机（DISCONNECTED→IDLE→ENABLED→MOVING）│
│  teleop.py    TeleopController：按键→命令编排（不含 HTTP/WS） │
│  sink.py      TeleopCommandSink 抽象协议（依赖倒置接口）      │
└───────────────────────────────▲────────────────────────────┘
                                │ 复用（高扇入）
                    apps/robot/services/robot_service.py
                    core/hardware/robot/base_driver.py
```

- **`device.py`（驱动层职责）**：只做 HID 报文读取与归一化（把原始轴值映射到 `[-1.0, 1.0]`，按键去抖），不做业务判断。满足开闭原则：新增 `inputs`/`pygame`/`evdev` 后端只需实现 `BaseTeleopInput`。
  - **命名约定（按职责命名，不绑定硬件）**：抽象层一律用 `Teleop`/`Input` 前缀（`BaseTeleopInput` 抽象基类、`InputState` 输入快照、`TeleopCommandSink`、`TeleopController`）；只有**具体手柄驱动**（`LinuxGamepadInput` / `PygameGamepadInput`）保留 `Gamepad`；`HID Usage Gamepad 0x05` 等为 USB 规范固定术语，保持原样。
- **`teleop.py` + `state.py`（编排职责）**：运行状态机与键位→命令翻译，通过 `TeleopCommandSink` 协议下发命令，**不直接 import `robot_service`**（依赖倒置），由 App 层注入具体实现。
- **`sink.py`（抽象接口）**：core 只依赖此协议；App 层的 `RobotCommandSink` 负责调用 `RobotService.jog_continuous / set_do / estop / set_global_speed_factor` 等既有色接口，实现**零重复通信逻辑**。

---

## 3. 从启动到控制的完整生命周期（Startup → Control Flow）

```
[1] 接入手柄 → [2] 设备枚举/绑定 → [3] 连接机械臂(robot_service) → [4] 手柄进入 IDLE
   → [5] 按 Start 使能(需机械臂 Idle) → [6] ENABLED：按住 RB 使能键 + 拨杆点动
   → [7] 松手/松开使能/异常 → 立即停止点动，回 IDLE/SAFE
   → [8] 任意时刻按 Y 急停 → E-STOP：立即关喷 DO + 机械臂 estop
```

### 3.1 启动时序（分阶段）

| 阶段 | 动作 | 前置校验 | 失败处理 |
|---|---|---|---|
| S1 设备上线 | `BaseTeleopInput.open()` 轮询 `/dev/input`（或 SDL）匹配 8BitDo VID/PID | 找到设备节点 | 报 `Teleop input device not found`，LED 熄灭，模块不启动 |
| S2 输入校准 | 读全轴静止基线，设定死区 `deadzone`（默认 0.12）与量程归一化 | 各轴抖动 < 阈值 | 报 `Axis drift`，要求重放手柄 |
| S3 机械臂就绪 | 复用 `robot_service.is_connected()` 与 `get_running_state()` | 已连接且 `status==0(Idle)` | 未连接→禁止使能，LED 琥珀慢闪 |
| S4 进入 IDLE | 装载键位映射（`configs/` 覆盖或默认表） | 配置校验通过 | 回退默认映射并告警 |
| S5 使能握手 | 操作员按 **Start(+)** | 见 §5 状态机 ENABLE 前置条件 | 条件不满足→拒绝，LED 双闪提示 |
| S6 点动控制 | 按住 **RB**(dead-man) + 拨杆 | 持续 armed、状态 Idle/允许点动 | 松 RB / 抖动越限→停 |
| S7 关断/退出 | 断开、进程退出、看门狗超时 | — | **Fail-Close**：`set_do(spray,0,immediate=True)` + jog stop |

> **看门狗（Heartbeat Watchdog）**：`teleop.py` 以固定周期（默认 100 ms）向 sink 发送心跳；连续 N 个周期（默认 3）收不到手柄事件（断连/休眠），视为失控，自动执行 §7 的安全停止。

---

## 4. 键位映射总表（从启动到控制）

### 4.1 逻辑按键名（与物理报文解耦，作为配置键）
| 逻辑名 | 物理含义（8BitDo 标准布局） |
|---|---|
| `LS_X` / `LS_Y` | 左摇杆 水平/垂直（模拟轴，`-1.0..1.0`） |
| `RS_X` / `RS_Y` | 右摇杆 水平/垂直（模拟轴） |
| `LT` / `RT` | 左/右扳机（单向模拟轴 `0..1`） |
| `LB` / `RB` | 左/右肩键（数字键） |
| `DP_UP/DOWN/LEFT/RIGHT` | 十字键 四向 |
| `A` `B` `X` `Y` | 面键 |
| `BACK` | Select / `−` |
| `START` | Start / `+` |
| `L3` / `R3` | 左/右摇杆按下 |

> 逻辑名到**具体原始索引**随平台/后端/手柄模式而变；§4.5 记录了参考手柄（8BitDo 猎户座2 NS 模式）的实测对照表，`mapping.py` 以此为初始单一真实源并按平台覆盖。

### 4.2 系统与模式键（数字键，边沿触发）

| 按键 | 功能 | 触发 | 服务层映射 | 英文提示（广播/日志） |
|---|---|---|---|---|
| **START (+)** | 使能伺服 / 进入 ENABLED | 按下(边沿) | 校验后本地置 ENABLED | `Teleop enabled` |
| **BACK (−)** | 单击=循环运动模式 Trans→Rot→Joint；**按住=夹爪/末端修饰键**（见 §4.4） | 单击(边沿) / 按住(修饰) | 本地状态 | `Motion mode: Translation/Rotation/Joint` |
| **RB**（按住） | **使能/Dead-man**：仅此键按住时摇杆才动 | 按住=armed / 松开=stop | 松开→`jog_continuous(axis,0)` 停 | `Dead-man released, motion stopped` |
| **LB**（按住） | **降速超控**：强制切到示教低速档 | 按住有效 | 临时速率因子 | `Reduced speed engaged` |
| **A** | 喷涂 DO 手动开/关切换（立即指令） | 按下(边沿) | `set_do(spray, toggle, immediate=True)` | `Spray ON` / `Spray OFF` |
| **B** | 暂停/恢复（有队列轨迹时） | 按下(边沿) | `pause()` / `resume()` | `Motion paused` / `Motion resumed` |
| **X** | 清报警/错误 | 按下(边沿) | `clear_error()` | `Alarm cleared` |
| **Y** | **急停 E-STOP**（最高优先级） | 按下(边沿) | `estop()`（内部含立即关喷） | `EMERGENCY STOP` |
| **DP_UP / DP_DOWN** | 速率档位 +/− （10/25/50/100%） | 按下(边沿) | `set_global_speed_factor(...)` | `Speed tier: N` |
| **DP_LEFT / DP_RIGHT** | 坐标系循环 Base↔Tool↔User / Joint 模式选关节 J1..J6 | 按下(边沿) | 本地帧选择 + `set_tool_number` | `Frame: Base/Tool` / `Joint: Jn` |
| **L3**（按住 RB+L3） | 回 Home 原点（需 Idle+armed，LED 双闪确认） | 组合按住 | `go_home()` | `Returning home...` |
| **R3**（按住 RB+R3） | 回 Fold 收纳位（同上互锁） | 组合按住 | `go_fold()` | `Folding arm...` |

> ⚠ 参考手柄（猎户座2 NS 模式）**未测到 `L3/R3`**（见 §4.5），故 `RB+L3/R3`（Home/Fold）在该手柄不可用；需要时改用已确认存在的组合或经 8BitDo 软件 remap。

> **急停优先原则**：`Y` 与看门狗/异常路径**绕过一切业务判断**，直接调用服务层立即指令关喷并 `estop()`，符合 IEC 61317 / Skill「故障安全」。

### 4.3 摇杆/扳机运动映射（模拟轴，按住比例点动）

摇杆采用**比例速度控制**：`cmd_speed = speed_tier × scale(|axis_after_deadzone|)`，越推越快，回中即停；配合连续点动接口 `jog_continuous(axis, direction)`（内部 `move_jog_cartesian`/`move_jog_joint`），松手发 `direction=0` 停止。多轴按**最大绝对值优先**避免相互干扰。

**模式一：Translation（平移，默认）**
| 控件 | 轴 | 方向 | 坐标系 |
|---|---|---|---|
| `LS_X` ↔ | **X** ± | 右+/左− | Base / Tool |
| `LS_Y` ↕ | **Y** ± | 上+/下− | Base / Tool |
| `RS_Y` ↕ | **Z** ± | 上+/下− | Base / Tool |
| `RS_X` ↔ | **Rz** ±（偏航） | 右+/左− | Tool |

**模式二：Rotation（姿态，BACK 切到 Rot）**
| 控件 | 轴 | 方向 |
|---|---|---|
| `LS_X` ↔ | **Rx** ±（翻滚） | 右+/左− |
| `LS_Y` ↕ | **Ry** ±（俯仰） | 上+/下− |
| `RS_X` ↔ | **Rz** ±（偏航） | 右+/左− |
| `RT` / `LT` | **Z** 精细 ±（上/下微调） | RT=Z+, LT=Z− |

**模式三：Joint（关节，BACK 切到 Joint）**
| 控件 | 轴 | 说明 |
|---|---|---|
| `DP_LEFT/RIGHT` | 选关节 | 循环 J1..J6，LED 短闪次数提示序号 |
| `LS_X` ↔ | 选中关节 ± | 比例点动，调用 `jog_continuous("Jn", dir)` |
| `RS_X` ↔ | 选中关节 ±（粗调） | 与 LS 同轴，速率翻倍，便于大范围移动 |

> 每次运动命令下发前，`state.py` 必须读取 `robot_service.get_running_state()`：仅当 `status==0(Idle)` 才允许**启动**新的点动，运行中(`status==1`)拒绝并发提交（互锁，见 Skill「状态判定统一标准」）。停止命令(`direction=0`)任何时候都允许下发。

### 4.4 夹爪（Junduo EPG50，`min_stroke`=闭合 ~ `max_stroke`=全张）键位映射

夹爪为**绝对位置指令**（服务层无连续点动 API），因此采用“点按到位 + 按住步进微调”。夹爪动作与机械臂点动相互独立，但同样受状态机门控（仅 `IDLE/ENABLED/MOVING` 允许，`E-STOP/DISCONNECTED` 拒绝）。

**主映射（带背键的手柄：Pro 2 / Ultimate / SN30 Pro）**
| 控件 | 功能 | 服务层映射 | 英文提示 |
|---|---|---|---|
| `P1`（背键1，点按） | 夹紧/闭合到 `min_stroke` | `clamp_gripper()` | `Gripper: clamp` |
| `P2`（背键2，点按） | 完全张开到 `max_stroke` | `open_gripper()` | `Gripper: open` |
| `LB`+`P1`（按住 LB 再夹紧） | 低力夹紧（软质工件/衣物） | `clamp_gripper(force_percent=低力档)` | `Gripper: low-force clamp` |

> ⚠ **实测可用性**：参考手柄 8BitDo **猎户座2（Ultimate 2）在 NS/Switch 模式下背键 `P1/P2` 不上报**（Switch 协议按键集固定、无背键槽位，见 §4.5）。因此**该手柄须使用下方通用回退**；建议把“通用回退”作为**跨手柄默认主用**，背键仅在确认部署模式暴露时作为可选快捷方式。

**通用回退（无背键手柄 / 默认主用）**：按住 `BACK`（此时 BACK 作为末端执行器修饰键）→ `LT`=夹紧、`RT`=张开；`BACK` 单击仍循环运动模式（§4.2）。

**行程/力度微调（按住 `BACK` 期间）**
| 控件 | 功能 | 服务层映射 | 英文提示 |
|---|---|---|---|
| `BACK`+`DP_UP/DP_DOWN` | 行程 ±2 mm 步进（限幅 `min~max`），≤2 Hz 节流连发 | `move_gripper(stroke_mm=当前±2)` | `Gripper stroke: N mm` |
| `BACK`+`DP_LEFT/RIGHT` | 夹紧力 −/+ 5% 档（作用于下次 clamp） | 本地 `force_percent` 状态 | `Gripper force: N%` |
| `BACK`+`RS_Y` ↕ | 行程连续比例扫动 | `move_gripper`（节流） | `Gripper stroke: N mm` |

> 微调前从 `get_gripper_state()` 读当前行程作为增量基准；所有 stroke/force 均按 `JunduoGripper.get_specs()` 限幅，越界 Fail-fast。夹爪指令同样需 `robot_service.is_connected()`，否则服务层直接拒绝并回英文错误。

### 4.5 实测按键索引（参考手柄：8BitDo 猎户座2 / Ultimate 2，NS·Switch 模式，macOS/SDL2 后端）

用 `manual_verify_gamepad.py --buttons` 在 macOS 实测得到的 **物理键 → 逻辑名 → SDL 原始索引** 对照，作为 `mapping.py` 的初始单一真实源（其它平台/模式各自实测覆盖，见 §9）。

| 物理按键 | 逻辑名 | SDL 原始索引 | 类型 |
|---|---|---|---|
| A / B / X / Y | `A` `B` `X` `Y` | button 0 / 1 / 2 / 3 | 数字键 |
| `−` Create | `BACK` | button 4 | 数字键 |
| Home (F) | `HOME` | button 5 | 数字键（系统键，慎用） |
| `+` Pause | `START` | button 6 | 数字键 |
| 左摇杆 X / Y | `LS_X` / `LS_Y` | axis 0 / axis 1 | 模拟轴 |
| 右摇杆 X / Y | `RS_X` / `RS_Y` | axis 2 / axis 3 | 模拟轴 |
| 左扳机 ZL | `LT` | axis 4 | 模拟轴 |
| 右扳机 ZR | `RT` | axis 5 | 模拟轴 |
| 左肩 L | `LB` | button 9 | 数字键 |
| 右肩 R | `RB` | button 10 | 数字键 |
| 十字 上/下/左/右 | `DP_UP/DOWN/LEFT/RIGHT` | button 11 / 12 / 13 / 14 | 数字键（hats=0，走按键） |
| 方块 Capture | `CAPTURE` | button 15 | 数字键（系统键，慎用） |

**实测确认的“未使用 / 不可用”**
- `button 7 / 8`：本模式无任何映射（此前“7、8 没反应”符合预期，**非死键**）。
- **背键 `L4 / R4 / PL / PR`（夹爪里的 `P1/P2`）**：Switch/Pro 协议按键集固定、**无背键槽位 → 不上报**；需 8BitDo 软件 remap 或换 DirectInput 模式才可用 → §4.4 夹爪以“肩+扳机组合”为默认主用。
- **摇杆按下 `L3 / R3`**：NS 模式未测到，`RB+L3/R3`（Home/Fold）在该手柄不可用。
- `LT/RT` 为**轴**（非按键），停止判定按轴回中处理。

> 该手柄在 macOS/NS 模式**可稳定使用**的键：`A/B/X/Y`、`BACK(−)`、`START(+)`、`LB/RB`、`LT/RT`(轴)、`十字四向`；`HOME/Capture` 可能被系统拦截，**背键 / L3/R3 不可用**。落地 `mapping.py` 时对参考手柄优先使用“可稳定使用”集合。

---

## 5. 遥操作状态机（State Machine）

```
        open() 成功            robot Idle & START            RB 按住 & 有摇杆输入
 DISCONNECTED ──▶ IDLE ─────────────────────▶ ENABLED ─────────────────────▶ MOVING
      ▲   ▲        │  ▲                          │  │  ▲                        │
      │   │        │  │  RB 松开 / 摇杆回中        │  │  │                        │
      │   │        │  └──────────────────────────┘  │  └──── 松手 / 看门狗 ───────┘
      │   │        │                                 │
      │   └────────┴───── 断连 / 校验失败 ◀───────────┤
      │                                              │ 任意状态
      └──────────── reset ──────────◀── E-STOP ◀─────┘（Y / 异常 / 看门狗超时）
                                            │
                              estop()+立即关喷 DO + 停所有 jog；夹爪保持当前位 (Fail-Hold, 不张开)；需 X 清障 + START 重新握手
```

| 状态 | 允许动作 | 进入条件 | 退出/停止条件 |
|---|---|---|---|
| `DISCONNECTED` | 无 | 初始/断连 | 设备+机器人均就绪 → IDLE |
| `IDLE` | 模式/坐标系/速率切换（非运动） | 设备在线且机器人 Idle | START 握手成功 → ENABLED |
| `ENABLED` | 点动待命（RB 未按时不动） | 已使能伺服 | RB+摇杆 → MOVING |
| `MOVING` | 连续点动 | armed 且有轴输入 | 松 RB / 回中 / 看门狗 → ENABLED/IDLE |
| `E-STOP` | 仅清障(X)+重启握手(START) | Y / 异常 / 超时 | 清障+确认 → IDLE |

---

## 6. LED / 操作员反馈约定（对齐 ISA-101 / IEC 62682）

| 状态 | LED 表现（8BitDo 可编程灯） | 含义 |
|---|---|---|
| `DISCONNECTED` | 熄灭 | 手柄/机器人未就绪 |
| `IDLE` | 绿色常亮 | 已连接，待使能 |
| `ENABLED`（RB armed） | 绿色快闪 | 使能待命 |
| `MOVING` | 琥珀色常亮 | 正在点动 |
| `Reduced speed (LB)` | 琥珀色慢闪 | 示教低速档生效 |
| `E-STOP` / 故障 | 红色双闪 | 急停生效，需清障 |
| 看门狗超时预警 | 红/绿交替 | 输入丢失，即将自动停止 |

若目标手柄型号不支持可编程 LED，退化为**机载蜂鸣/振动**（`LT/RT` 短脉冲）做等价提示；核心安全逻辑不依赖反馈通道（反馈失效仍保持 Fail-Close）。

---

## 7. 安全与互锁设计（Fail-Safe）

1. **Dead-man 强制**：无 RB 按住 → 任何摇杆输入被丢弃；RB 松开瞬间对所有活动轴下发 `jog_continuous(axis, 0)`。
2. **故障关喷优先**：E-STOP / 看门狗超时 / 设备断连 / 进程异常 / 未捕获异常，一律 `set_do(spray_do_index, 0, immediate=True)`，随后 `estop()`。顺序：先断料再停机。
3. **状态互锁**：机器人 `status!=0`（Running/Paused）时禁止启动新点动、禁止 Home/Fold；仅允许停止与急停。
4. **速率上限**：`speed_tier`、`max_tcp_speed_mm_s`、`max_joint_speed_deg_s` 三重限幅，LB 按住再乘降速系数（默认 ≤5% 满速）；非法值快速失败并保留上一有效档。
5. **边界校验（防御式编程）**：DO 编号 `1<=index<=16`、速率因子 `1..100`、轴值裁剪到 `[-1,1]`、死区外才响应；一切入参非法 → Fail-fast + 英文错误。
6. **指令缓冲自愈**：进入 Home/Fold 等大动作前，沿用服务层既有的 `_discard_pending()` 清残包机制（由 `robot_service` 保证，手柄层不重复实现）。
7. **量纲一致**：core 内部 `mm`/`rad`；`jog` 封包时经驱动层显式转 `deg`，手柄层不直接拼欧拉角数组，统一走 `RobotPose`/服务层色接口。
8. **夹爪故障保持（Fail-Hold）**：E-STOP / 断连 / 看门狗触发时**绝不张开夹爪**（防止掉落已夹持工件），仅停机械臂 + 关喷 DO，夹爪保持当前行程；夹爪动作仅在 `ENABLED` 后的受控状态允许，且下发前校验 `robot+gripper` 已连接（复用服务层互锁）。

---

## 8. 配置设计（对齐 `SprayerConfig` 声明式 schema）

在 `configs/aisprayer_config.yaml` 新增 `hardware.teleop` 段，并在 `core/config.py` 的 `_CONFIG_SCHEMA` 增加条目（`category: "teleop"`）：

```yaml
hardware:
  teleop:
    enabled: true
    backend: evdev            # 目标板 RK3588=evdev；macOS 开发机=pygame（BaseTeleopInput 实现选择，见 §9）
    vendor_id: 0x045e         # 2.4G 接收器经内核 xpad 呈 Xbox360 HID（RK3588 上 evdev 实际看到的 VID/PID）
    product_id: 0x028e
    device_name: "8BitDo"     # 兜底匹配：VID/PID 命中失败时按设备名子串匹配（跨协议/跨模式最稳，见 §9 指纹）
    deadzone: 0.12            # 摇杆死区 (0..0.5)
    watchdog_interval_ms: 100 # 心跳周期
    watchdog_miss: 3          # 连续丢失次数触发安全停止
    speed_tiers_pct: [10, 25, 50, 100]   # D-Pad 上/下循环切换的档位
    reduced_speed_pct: 5                  # LB 按住时的示教低速
    mapping: default          # 键位映射方案名，支持自定义表覆盖 §4
```

对应新增读取属性（示例，遵循现有 `get_cascading` / `_get_yaml_nested` 模式）：
`teleop_enabled`, `teleop_backend`, `teleop_deadzone`, `teleop_speed_tiers_pct` 等。键位映射表以 `mapping.py` 中的**纯数据字典**为单一真实源，配置可覆盖，禁止在业务代码散落硬编码按键判断（DRY）。

---

## 9. 依赖与运行环境

核心口径：状态机 / 键位映射 / 夹爪编排为纯 Python，macOS 与 RK3588 **同一份代码**；唯一随平台分叉的是 `device.py` 的 HID 读取后端，它被 `BaseTeleopInput` 接口隔离，两端都实现同一个 `read() -> InputState` 契约。

| 维度 | macOS（开发 / 验证） | RK3588 arm64（生产目标） |
|---|---|---|
| HID 后端 | `PygameGamepadInput`（SDL2，pip 自带） | `LinuxGamepadInput`（`evdev` `/dev/input`） |
| 8BitDo 呈现 | 2.4G 接收器 = Xbox360(`0x045E:0x028E`)，macOS **无驱动读不到**；需切 Switch/Pro 或有线 | 2.4G 接收器由内核 **xpad** 原生支持，插即出 `/dev/input` |
| 归一化输入 | SDL `button/axis` index | evdev `ABS_*` / `BTN_*` |
| 权限 | 首次可能需授予输入/蓝牙权限 | 服务用户加入 `input` 组 / udev 规则；容器放开头像设备节点 |
| 单测 | fake sink + 手工喂 `InputState`（不碰 HID，两端一致） | 同左 |

**实测设备指纹（8BitDo Ultimate + UM 2 Receiver，macOS M4 上抓得）**
- USB 层：`8BitDo UM 2 Receiver` = `idVendor 0x2DEC / idProduct 0x30EB`
- HID 层（2.4G，经 Xbox360 协议）：`0x045E / 0x028E`（← **RK3588 上 evdev 将看到的 VID/PID**）
- macOS 直连 / 切 Switch 模式：`0x057E / 0x2009`（Pro Controller，SDL 可原生读取，用于本机验证）
- 因不同模式 VID/PID 会变，`vendor_id/product_id` 命中失败时按 `device_name` 子串匹配兜底最稳。
- **按键可用性取决于模式/协议**：Switch 与 Xbox360 的按键集固定、**不含背拨片**（猎户座2 NS 模式实测 4 个背键不上报，见 §4.5）；需全按键矩阵时用 DirectInput/通用 HID 模式，或在部署模式实测后据实映射。

- **依赖**：`app/requirements.txt` 增加可选后端（macOS `pygame`；RK3588 `evdev`），Docker 运行镜像按 `linux/arm64` 带 `evdev` 并放开设备节点。**遵循最小依赖**，不引入 PyTorch 等重组件。
- **热插拔**：`/dev/input` 变化需重扫（`BaseTeleopInput.open()` 轮询匹配）；不连真机时用 `test_robot.py` 同款假驱动（fake sink）验证命令序列。

---

## 10. 落地计划（后续实现，本文件仅设计）

| 里程碑 | 交付 | 验收 |
|---|---|---|
| M1 设备层 | `device.py`(`BaseTeleopInput`+`evdev`) + `mapping.py` | 单测：给定原始事件流→产出正确逻辑按键/轴归一值 |
| M2 状态机 | `state.py` + `sink.py`（协议） | 单测：非法使能/松手/看门狗路径均产出正确命令序列（fake sink） |
| M3 编排 | `teleop.py` 全键位映射（§4） | 联调：假驱动回放 Home/Fold/E-STOP/Spray 时序 |
| M4 装配 | `apps/teleop/`：`RobotCommandSink` 接 `robot_service` + 生命周期 + WS 广播 | 真机低速点动 + 急停/关喷验证 |
| M5 硬化 | 配置化映射、死区标定、LED/振动反馈、权限/Docker | 现场示教回归 + 安全用例全通过 |

---

## 11. 键位速查卡（Cheat Sheet，可打印贴操作员台）

```
[系统/模式]
  START = Enable Servo        BACK tap=Mode / hold=Gripper modifier
  RB(hold) = Dead-man Enable  LB(hold) = Reduced Speed
  A = Spray ON/OFF            B = Pause/Resume
  X = Clear Alarm             Y = E-STOP
  D-Up/Dn = Speed +/-         D-Left/Rt = Frame / Select Joint
  RB+L3 = Go Home             RB+R3 = Go Fold

[Translation]  LS_X=X  LS_Y=Y  RS_Y=Z  RS_X=Rz
[Rotation]     LS_X=Rx LS_Y=Ry RS_X=Rz RT/LT=Z fine
[Joint]        D-L/R select Jn, LS_X=Jn jog, RS_X=Jn fast

[Gripper]      P1=Clamp  P2=Open  LB+P1=Low-force clamp
               (no paddles: hold BACK + LT=Clamp / RT=Open)
               hold BACK + D-Up/Dn = stroke +/-2mm, BACK + D-L/Rt = force -/+5%

Safety: release RB => arm stop | watchdog lost => spray OFF + stop (gripper HOLDS)
```
