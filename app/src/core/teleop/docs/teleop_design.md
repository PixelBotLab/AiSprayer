# 八位堂（8BitDo）遥控手柄机械臂遥操作模块设计方案

> 模块路径：`app/src/core/teleop/`（按职责命名，主输入设备为八位堂手柄）
> 状态：**设计阶段（Design / Not Implemented）**
> 适用范围：使用 HID 游戏手柄（部署机型：8BitDo 猎户座2/Ultimate 2 NS 版，经 2.4G 接收器走 **Nintendo Switch Pro 协议**；亦兼容 X-Input/D-Input 后端）对 CR5 机械臂进行手动示教与点动遥操作。

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
   → [5] 按 Start 连接+上伺服(需机械臂 Idle) → [6] ENABLED：按住 RT 使能键 + 拨杆点动
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
| S6 点动控制 | 按住 **RT**(dead-man) + 拨杆 | 持续 armed、状态 Idle/允许点动 | 松 RT / 抖动越限→停 |
| S7 关断/退出 | 断开、进程退出、看门狗超时 | — | **Fail-Close**：`set_do(spray,0,immediate=True)` + jog stop |

> **看门狗（Heartbeat Watchdog）**：`teleop.py` 以固定周期（默认 100 ms）向 sink 发送心跳；连续 N 个周期（默认 3）收不到手柄事件（断连/休眠），视为失控，自动执行 §7 的安全停止。

---

## 4. 键位映射总表（从启动到控制）

> 部署实测设备：**8BitDo 猎户座2（Ultimate 2）NS 版**，经 **2.4G NS 接收器**走 **Nintendo Switch Pro 协议**（内核 `hid_nintendo`，`057E:2009`）。以下按键/轴/十字键/LED 均在本机 `evdev` + `/sys/class/leds` 实测确认；单一真实源见 §4.5 与 `map_linux_24g.py`。

### 4.0 底层硬约束：单轴点动（先读）
点动链路 `RobotService.jog_continuous(axis, direction)` → `move_jog_cartesian/move_jog_joint` → Dobot `MoveJog(axis_id)`，其语义为：
- **一个周期只能点动一个轴**（`axis_id` 单值，joint 与 cartesian 皆然）；
- `direction` 仅为**正/负符号**（`+`/`-`），**不携带模拟比例速度**，`direction=0` 停止；
- 因此摇杆**无法**比例调速，速度只来自**全局速率档**（`set_global_speed_factor`）与 `LT` 降速。
- 遥操作层每周期从多路输入中取**主导单轴**（死后区绝对值最大者）下发一条 `MoveJog`；回中或松 `RT` → `direction=0` 停止。

> 这是对早期"摇杆比例速度"设想的更正：真机为**符号式单轴点动**。

### 4.1 逻辑按键名（与物理报文解耦，作为配置键）
| 逻辑名 | 物理含义（本机 NS 2.4G 实测） |
|---|---|
| `LS_X` / `LS_Y` | 左摇杆 水平/垂直（模拟轴；`LS_Y` 上推为负，需取反） |
| `RS_X` / `RS_Y` | 右摇杆 水平/垂直（模拟轴；`RS_Y` 上推为负，需取反） |
| `LT` / `RT` | 左/右扳机（**Switch 协议下为数字键**，非模拟轴） |
| `LB` / `RB` | 左/右肩键（数字键） |
| `DP_UP/DOWN/LEFT/RIGHT` | 十字键四向（走 **HAT** `ABS_HAT0X/Y`，离散 -1/0/1） |
| `A` `B` `X` `Y` | 面键（**Switch 协议内核名与丝印对角互换**，见 §4.5） |
| `BACK` / `START` | `−` / `+` |
| `HOME` | 机身 Home 键（`BTN_MODE`）：作 **Go Fold 收纳位**（需实测确认未被系统拦截） |
| `CAPTURE` | 方形 Capture 键：作 **Go Home 回原点** |
| `L3` / `R3` | 左/右摇杆按下（**实测可用**） |

> 逻辑名到原始内核码见 §4.5；`mapping.py` 以 `map_linux_24g.py` 为单一真实源，加载时做**设备能力指纹校验**（轴/键/HAT/LED 缺失即拒绝启动并告警）。

### 4.2 系统与模式键（数字键，边沿触发）

安全/系统键**在所有运动模式下含义固定**，不随模式漂移：

| 按键 | 功能 | 触发 | 服务层映射 | 英文提示（广播/日志） |
|---|---|---|---|---|
| **START** | **开关机械臂**：连接+上伺服 ↔ 断开（复用服务层 `connect()`/`disconnect()`，即 EnableRobot/DisableRobot） | 按下(边沿) | `connect()` / 先关喷+停点动后 `disconnect()` | `Robot connected (servo enabled)` / `Robot disconnected` |
| **BACK（单击）** | 循环运动模式 Trans→Rot→Joint | 单击(边沿) | 本地状态；模式经 §6 **绿灯常亮颗数(1/2/3)** 显示 | `Motion mode: Translation/Rotation/Joint` |
| **RT（按住）** | **Dead-man**：仅此键按住时摇杆才动作 | 按住=armed/松开=stop | 松开→`jog_continuous(axis,0)` 停 | `Dead-man released, motion stopped` |
| **LT（按住）** | **降速超控**：切示教低速档 | 按住有效 | 临时速率因子 | `Reduced speed engaged` |
| **A** | 喷涂 DO 手动开/关（立即指令） | 按下(边沿) | `set_do(spray, toggle, immediate=True)` | `Spray ON` / `Spray OFF` |
| **B** | 暂停/恢复（有队列轨迹时） | 按下(边沿) | `pause()` / `resume()` | `Motion paused` / `Motion resumed` |
| **R3** | 夹爪 全开/全闭 一键切换（§4.4） | 按下(边沿) | `open_gripper()` / `clamp_gripper()` | `Gripper OPEN` / `Gripper CLOSE` |
| **X** | 清报警/错误 | 按下(边沿) | `clear_error()` | `Alarm cleared` |
| **Y** | **急停 E-STOP**（最高优先级） | 按下(边沿) | `estop()`（内部先立即关喷） | `EMERGENCY STOP` |
| **CAPTURE** | 回 Home 原点（需 IDLE+armed 互锁） | 按下(边沿) | `go_home()` | `Returning home...` |
| **HOME** | 回 Fold 收纳位（同上互锁；若 `BTN_MODE` 被系统拦截则改用组合兜底） | 按下(边沿) | `go_fold()` | `Folding arm...` |
| **RB / LB** | 速率档 +/−（10/25/50/100%）**——仅 Trans/Rot 模式**（Joint 模式肩键让位给关节选择，见 §4.3） | 按下(边沿) | `set_global_speed_factor(...)` | `Speed tier: N` |
| **DP_LEFT / DP_RIGHT** | 坐标系循环 Base↔Tool↔User（**仅 Trans/Rot**） | 按下(边沿) | 本地帧 + `set_tool_number` | `Frame: Base/Tool/User` |

> **面键 A/B/X/Y 按丝印命名**（§4.5 已修正 Switch 协议的内核名互换）：`A`=喷涂、`B`=暂停/恢复、`X`=清障、`Y`=急停（`Y` 在顶部，符合"上=急停"直觉）。`HOME`(316) 若实测触发桌面环境快捷键，则把 Fold 兜底改挂 `CAPTURE 双击` 或 `BACK+START` 组合，功能不丢。

> **急停优先原则**：`Y` 与看门狗/异常路径**绕过一切业务判断**，直接调用服务层立即指令关喷并 `estop()`，符合 IEC 61317 / Skill「故障安全」。

### 4.3 摇杆/十字键运动映射（单轴点动 + 模式）

因 §4.0 硬约束，任一时刻**只驱动一个轴**。摇杆先过死区，再在候选轴里取**绝对值最大的主导轴**，以其符号下发 `MoveJog(axis, ±)`；回中/松 RT 立即停。速率档决定点动速度，`LT` 按住临时降速。

**模式一：Translation（平移，BACK 单击进入，默认）**
| 控件 | → 轴 | 符号约定（实测取反后） | 坐标系 |
|---|---|---|---|
| `LS_Y` ↕（左摇杆 **上下**） | **X** ± | 上推=+X（前进），下拉=−X | Base/Tool |
| `LS_X` ↔（左摇杆 **左右**） | **Y** ± | 右推=+Y，左推=−Y | Base/Tool |
| `RS_Y` ↕（右摇杆 **上下**） | **Z** ± | 上推=+Z（抬升），下推=−Z | Base/Tool |
| `RS_X` ↔（右摇杆 左右） | **Rz** ±（偏航） | 右=+ | Tool |
> 从 `LS_X/LS_Y/RS_X/RS_Y` 四候选中取**主导单轴**（死后区绝对值最大）下发一条 `MoveJog`；`DP_UP/DOWN`=速率、`DP_LEFT/RIGHT`=坐标系（§4.2）。

**模式二：Rotation（姿态，BACK 切到 Rot）**
| 控件 | → 轴 | 符号 |
|---|---|---|
| `LS_X` ↔ | **Rx** ±（翻滚） | 右=+ |
| `LS_Y` ↕ | **Ry** ±（俯仰） | 上=+（取反） |
| `RS_X` ↔ | **Rz** ±（偏航） | 右=+ |

> **注**：RT 已升为全局 dead-man、LT 为降速超控，Rotation 模式不再单独占用二者做 Z 微调；需要升降时切回 Translation 用右摇杆 `RS_Y`。

**模式三：Joint（关节，BACK 切到 Joint）——按住即动（示教器惯例，无需“选择记忆”）**
| 控件（左手 **按住不放**） | → 关节 | 说明 |
|---|---|---|
| `DP_UP / DP_DOWN / DP_LEFT / DP_RIGHT` | **J1 / J2 / J3 / J4** | 按住哪个 = 动哪个；松手立即停 |
| `L3`（左摇杆按下） / `LB` | **J5 / J6** | 腕部两键（J6 用左肩 `LB`；与 Trans/Rot 的速度±按模式分时复用） |
| `RS_Y` ↕（右摇杆 上下） | 当前按住关节的 ± | 上=+/下=−（内核取反后）；回中=停 |
| `RT`（右扳机按住） | Dead-man 使能 | 不按住则任何关节键无效 |
| `LT`（按住） | 示教低速 | 临时覆盖为最低速率档 |
> **设计动机（行业不成文标准）**：工业示教器没有“先锁定某关节、再靠屏记住”的做法——你**正按住的那个键就是当前关节**，天然不会搞错，完全不依赖 HUD/屏。底层仍是 §4.0 单轴点动（一次只按住一个关节键 = 单轴）。
> 按住瞬间 `state.py` 仍在 `player-1` 短闪 n 下作**二次确认**（可选）；§6 绿灯常亮颗数继续表示模式。`R3` 用作夹爪开/关切换（见 §4.4）。
> 速率沿用 §4.2 设定档（Trans/Rot 下用 RB/LB 调）；Joint 模式内靠 `LT` 降速。

> 每次启动新点动前，`state.py` 必须读 `robot_service.get_running_state()`：仅 `status==0(Idle)` 才允许**启动**，`status==1(Moving)` 拒绝并发（互锁，Skill「状态判定统一标准」）。停止(`direction=0`)任何时候允许。

### 4.4 夹爪（Junduo EPG50）键位映射

夹爪当前实现为 **R3 一键切换全开/全闭**（绝对到位指令，非比例）：`teleop.py._toggle_gripper()` 维护 `gripper_open` 布尔，每按一次 R3 在"张开↔闭合"间翻转，分别经 `gripper_open()`/`gripper_close()` 下发服务层 `open_gripper()`（到 `max_stroke`）/ `clamp_gripper()`（到 `min_stroke`）；服务层按 `JunduoGripper.get_specs()` 自动限幅。

| 控件 | 功能 | sink 方法 → 服务层 | 灯语 | 英文提示 |
|---|---|---|---|---|
| `R3`（右摇杆按下，单击边沿） | 切换 夹爪 全开/全闭 | `gripper_open()`→`open_gripper()` / `gripper_close()`→`clamp_gripper()` | `player-5` 蓝**常亮**=已张开（灭=已闭合） | `Gripper OPEN` / `Gripper CLOSE` |

> **为何用 R3**：`R3` 是本键位体系里唯一在**所有模式都空闲**的逻辑键（`L3/LB` 在 Joint 模式已作 J5/J6；背键 `P1/P2` 在 Switch/2.4G 协议下不上报，见 §4.5），挂 R3 不与他功能冲突。
> **门控**：仅 `IDLE/ENABLED/MOVING` 响应，`E-STOP/DISCONNECTED` 拒绝（夹爪与喷涂同为末端动作；急停时**不主动改动夹持**，避免误丢已夹工件）。
> **待扩展（暂不实现）**：低力夹紧、行程/力度步进微调（服务层 `move_gripper(stroke_mm, force_percent)` 已具备能力）——如需再另择空闲组合或引入"夹爪子模式"；本协议档无模拟扳机，比例连续扫动不可行。

### 4.5 实测键位对照（部署真实源：Linux · evdev · 2.4G NS/Switch 协议）

本机 `manual_map_evdev.py` 实测、`map_linux_24g.py` 固化的**逻辑名→内核码**表（**单一真实源**）：

| 物理键/控件 | 逻辑名 | 内核码 (EV_KEY/ABS/HAT) | 类型/备注 |
|---|---|---|---|
| A / B / X / Y | `A`/`B`/`X`/`Y` | 305 / 304 / 307 / 308 | 数字键；**Switch 协议内核名与丝印对角互换**（`BTN_A(305)=物理A`, `BTN_B(304)=物理B`, `BTN_NORTH(307)=物理X`, `BTN_WEST(308)=物理Y`） |
| 方形 Capture | `CAPTURE` | 309 `BTN_Z` | =Go Home |
| L / R 肩键 | `LB` / `RB` | 310 / 311 | Joint J6 / 速度−·速度+（Trans/Rot） |
| ZL / ZR 扳机 | `LT` / `RT` | 312 / 313 **数字** | LT=降速超控 / RT=dead-man；Switch 协议量化为键，**非模拟轴** |
| − / + | `BACK` / `START` | 314 / 315 | 模式循环 / 使能 |
| Home | `HOME` | 316 `BTN_MODE` | =Go Fold（需实测确认未被系统拦截） |
| 摇杆按下 | `L3` / `R3` | 317 / 318 | **实测可用**（L3=Joint J5；R3=夹爪切换） |
| 左摇杆 水平/垂直 | `LS_X` / `LS_Y` | ABS_X(0) / ABS_Y(1) | ±32767；右=+, **上=−（需取反）** |
| 右摇杆 水平/垂直 | `RS_X` / `RS_Y` | ABS_RX(3) / ABS_RY(4) | 右=+, **上=−（需取反）** |
| 十字 上/下/左/右 | `DP_UP/DOWN/LEFT/RIGHT` | HAT (17,-1)/(17,1)/(16,-1)/(16,1) | `ABS_HAT0Y=17`,`ABS_HAT0X=16`；Y **−1=上** |
| 背键 L4/R4/PL/PR | — | **不上报** | Switch 协议无背键槽位 → §4.4 用 BACK+扳机回退 |

**实测要点（务必写入 `mapping.py` 覆盖层与指纹校验）**
- **A/B↔X/Y 对角互换**：Switch 协议内核 `BTN_*` 语义与 Xbox 丝印不一致，`manual_map_evdev.py` 已按设备名（switch pro/8bitdo/nintendo）覆盖为物理真值，`mapping.py` 沿用逻辑名 `A/B/X/Y`（丝印）。
- **摇杆方向**：`LS_X(0)+=右`、`LS_Y(1)−=上`、`RS_X(3)+=右`、`RS_Y(4)−=上`；Y 轴内核为"上=负"，遥操作层统一**取反**成"上=+"。
- **十字键走 HAT**（非 button），左右变化只动 `ABS_HAT0X`；判定用离散方向符号，勿当连续轴。
- **LT/RT 为数字键**（Switch 报告 ZL/ZR 各 1 bit）；要模拟扳机需切 PC/X-Input 协议（本方案**不切**，锁 NS 档）。
- **玩家 LED 可控**：`/sys/class/leds/*:green:player-1..4` + `blue:player-5`，`max_brightness=1`（仅亮/灭），经 udev `RUN+=chmod` 免 sudo 写；**8BitDo 星键 RGB 固件自管、主机无设色通道，不可用于模式显示**（见 §6）。
- 与 macOS/SDL(pygame) 后端的差异：SDL 会重排 index 且把 HAT 转 button，**两端各自实测覆盖**，不可共用一张表。

---

## 5. 遥操作状态机（State Machine）

```
     START(connect+EnableRobot)          RB 按住           RB 按住 & 有摇杆输入
 DISCONNECTED ──────────────▶ IDLE ───────────▶ ENABLED ────────────▶ MOVING
      ▲  ▲                     │  ▲               │  │  ▲                  │
      │  │   START(disconnect) │  │  松 RB         │  │  │                  │
      │  └───────────────────┘  └─────────────┘  │  └─ 松手/回中/看门狗 ─┘
      │  ◀── 断连 / 校验失败 / 被动掉线 ────────────┤
      │                                            │ 任意状态
      └──────────── reset ──────────◀── E-STOP ◀─────┘（Y / 异常 / 看门狗超时）
                                            │
                 estop()+立即关喷 DO + 停所有 jog（链路保留、不自动断伺服以防落臂）；夹爪保持当前位 (Fail-Hold)；需 X 清障
```

| 状态 | 允许动作 | 进入条件 | 退出/停止条件 |
|---|---|---|---|
| `DISCONNECTED` | 仅 START 连接 | 初始 / START 断连 / 被动掉线 | START→`connect()` 成功 → IDLE |
| `IDLE` | 已连接待命（模式/夹爪/速率可调，RB 未握） | START 连接成功 | RB 握下 → ENABLED；START 再按 → `disconnect()` 回 DISCONNECTED |
| `ENABLED` | armed 待命（RB 按住但无摇杆输入） | 已连接 + RB | 摇杆输入 → MOVING；松 RB → IDLE |
| `MOVING` | 连续点动 | armed 且有轴输入 | 松 RB / 回中 / 看门狗 → IDLE/ENABLED |
| `E-STOP` | 仅清障(X)；START 仍可断连 | Y / 异常 / 超时 | 清障 → IDLE（链路保留） |

---

## 6. LED / 操作员反馈约定（player 灯 · 对齐 ISA-101 / IEC 62682）

**通道**：内核 `hid_nintendo` 暴露的 4 颗绿 `player-1..4` + 1 颗蓝 `player-5`（`/sys/class/leds`，仅亮/灭）。**闪烁由应用线程按自定节奏写 `brightness` 实现**（不依赖内核 `timer`）。蓝色 `player-5`：**双闪=急停/故障**（故障优先）；**常亮=夹爪已张开**（与故障双闪互斥）。

> **设计原则**：5 颗二值灯无法同时常显"模式+状态+关节+速率"。故分工——**绿灯亮几颗=当前模式**（随时可瞥见），**这几颗的闪动节奏=使能/运动状态**，**第 4 颗=低速标记**，**蓝=故障双闪/夹爪张开常亮**；**Joint 模式采用“按住即动”（§4.3），当前关节=正按住的键、自明，无需占用灯位显示关节**。

**A. 模式（绿灯 `player-1..3` 常亮颗数——一眼可辨，连接期间一直成立）**
| 模式 | 常亮绿 |
|---|---|
| Translation | 1（player-1） |
| Rotation | 2（player-1,2） |
| Joint | 3（player-1,2,3） |

**B. 状态（在"模式绿"之上叠加的亮法；蓝=故障）**
| 状态 | 模式绿表现 | player-4 | player-5(蓝) |
|---|---|---|---|
| `DISCONNECTED` | 全灭（不显模式） | 灭 | 灭 |
| `IDLE`（未使能） | 常亮 | 灭 | 灭 |
| `ENABLED`（RB armed） | 慢闪 ~1Hz | 灭 | 灭 |
| `MOVING`（点动中） | 快闪 ~3Hz | 灭 | 灭 |
| `Reduced speed`（LB 按住） | 维持当前节奏 | **额外常亮=低速灯** | 灭 |
| 夹爪张开（R3，§4.4） | 维持 | 维持 | **蓝色常亮**（灭=已闭合） |
| `E-STOP`/故障/看门狗 | 全灭 | 灭 | **蓝色双闪**（优先于夹爪常亮） |

**C. 事件 ack 短闪（在 player-1 上闪 N 下，N=序号；仅切换瞬间）**
| 事件 | 闪 N |
|---|---|
| 选中关节 J1..J6 | 1..6 |
| 速率档 1..4（10/25/50/100%） | 1..4 |

> **双保险**：player 灯给"离屏手感"确认；**HUD 徽章是权威真实源**（`MODE / ACTIVE(Jn|axis) / SPEED% / [RB ARMED]`），每次切模式、选关节、降速经 WebSocket 广播刷新。灯语只是冗余提示，反馈失效不影响 Fail-Close 安全逻辑。
> 若现场需要**醒目大彩灯**：用手臂空闲 **DO 驱动外部三色灯塔**（teleop `set_do` 切色），而非手柄固件星灯。

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
    vendor_id: 0x057e         # NS 2.4G 接收器经内核 hid_nintendo 呈 Switch Pro（057E:2009，本机 evdev 实测）
    product_id: 0x2009
    device_name: "Controller" # 兜底：VID/PID 命中失败时按设备名子串匹配（含 "Pro Controller"/"8BitDo" 时启用 A/B↔X/Y 覆盖，最稳）
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
[系统/模式]  (所有运动模式下固定)
  START = Enable Servo        BACK tap = Mode cycle (Trans>Rot>Joint)
  BACK hold = Gripper modifier
  RB(hold) = Dead-man Enable  LB(hold) = Reduced (teach-slow) Speed
  A = Spray ON/OFF   B = Pause/Resume   X = Clear Alarm   Y = E-STOP
  CAPTURE = Go Home        HOME = Go Fold
  D-Up/Dn = Speed +/- (Trans/Rot only)   D-Left/Rt = Frame (Trans/Rot only)

[TRANSLATION] dominant-axis, sign-only jog (no analog speed)
  LS_Y=X(+Up)  LS_X=Y(+R)  RS_Y=Z(+Up)  RS_X=Rz(+R)
[ROTATION]    LS_X=Rx  LS_Y=Ry  RS_X=Rz  RT/LT=Z fine(+/-)
[JOINT]       hold-to-jog (industry teach-pendant convention):
  HOLD J1..J4 = D-Up/Dn/L/R   HOLD J5 = L3   HOLD J6 = LT
  RS_Y(up/down) = held joint +/-   RB(hold)=enable   release = stop

[Gripper]  hold BACK:  LT=Clamp  RT=Open  (D-Up/Dn stroke +/-2mm, D-L/Rt force -/+5%)
           (no back paddles in NS mode)

LED: green COUNT = mode (Trans 1 / Rot 2 / Joint 3), always visible
     blink = state (solid idle / slow enabled / fast moving); player-4 on = LB slow
     blue(player-5) double-blink = E-STOP/fault/watchdog
     player-1 blinks N times = ack (joint# / speed tier#)
Safety: release RB => jog stop | fault/watchdog => spray OFF + stop (gripper HOLDS)
```
