# Dobot API 代码实现分析报告

基于文档 `app/docs/dobot_tcp_ip_v3_protocol.md`（控制柜 V3，六轴）对 `app/src/core/hardware/robot/dobot_api.py` 的逐条核对结果。

- 第一部分：对已有结论的复核（标注 **成立** / **需修正** / **存疑**）。
- 第二部分：本次新增发现的问题。
- 第三部分：核对通过、无需改动的部分（避免后续误改）。

> **修复状态**：本报告列出的问题已在 `dobot_api.py` 中全部处理，因此文中引用的行号
> 对应的是**修复前**的版本，仅用于定位问题来源，不再与当前代码一致。两处按“不改行为”
> 处理的例外：
>
> - 一、二.2 的 RunScript 引号（协议自身存在歧义），改为由 `RunScript(name, quoted=False)`
>   参数交给调用方决定，默认行为不变；
> - 接口-3 的非协议指令一律保留（避免破坏既有调用），只在 docstring 里标注
>   “【非 V3 协议指令】”并指向对应的协议指令；只有 `Jump` 这个假实现被删除。

---

# 第一部分：已有结论复核

## 一、 类方法重复定义（被相互覆盖）

结论：**重复定义的事实成立，但严重性描述需修正。**

用 AST 扫描确认，`DobotApiDashboard` 中确实有 4 个方法被重复定义（后者覆盖前者）：
`DOExecute`(298/496)、`ToolDO`(307/500)、`ToolDOExecute`(316/504)、`SetArmOrientation`(399/508)。

需要修正的是后果判断：

1. **`DOExecute` / `ToolDO` / `ToolDOExecute`：只是冗余，没有功能影响。** 覆盖版本同样是两个 `{:d}` 参数，拼出的指令字符串与被覆盖的版本完全一致。真实损失只有 docstring 和参数名语义（`index,status` 变成无意义的 `offset1,offset2`）。
2. **`SetArmOrientation`：这一条才是真正的破坏性覆盖。** 协议要求 `SetArmOrientation(LorR,UorD,ForN,Config6)` 四个必选参数，覆盖后只剩一个参数，按协议正确方式调用会直接 `TypeError`，退而按单参数调用则会被控制器回 `-20000`（参数数量错误）。结果是**手系功能完全不可用**。

## 二、 字符串拼接与指令格式 Bug

1. **`ToolDI` 复制粘贴遗留错误**：**成立**。第 636-638 行发送的是 `DI({:d})`，调用 `ToolDI(1)` 实际读的是控制柜 DI1 而不是末端 DI1，会静默返回错误端口的状态，属于"不报错但结果错"的危险类型。
2. **`RunScript` 参数缺失双引号**：**存疑，不建议直接按此修改。** 协议中 `RunScript` 的示例确实写作 `RunScript("demo")`，但同一份协议里其它所有 string 类型参数的示例都不带引号，例如 `ModbusCreate(127.0.0.1,60000,1,1)`、`HandleTrajPoints(recv_string)`、`GetPalletPose(pallet1,5)`、`StartTrace(recv_string)`。协议正文只规定"参数以英文逗号相隔"，未定义字符串引号规则。因此这一条应实测确认后再改，盲目加引号有可能反而把引号当作工程名的一部分。
3. **`DOGroup` 尾部多余逗号**：**成立**，但这是 `DOGroup` 三个问题里最轻的一个，详见第二部分【严重-1】。
4. **`InverseSolution` 和 `GetInRegs` 漏加逗号及崩溃隐患**：**成立**。补充：漏逗号只是表象，这两处的参数格式本身也不符合协议，详见第二部分【格式-4】【格式-5】。

## 三、 变长参数 (`*dynParams`) 使用设计缺陷

1. **解包崩溃问题**：**成立**。`RelMovJTool` / `RelMovLTool`（946-977 行）确实强制要求调用者传入一个嵌套元组，直接传三个数字会 `TypeError: 'int' object is not subscriptable`。
2. **内置 `str(tuple)` 产生的空格问题**：**成立但描述不完整，实际更严重。** 协议中 `MovLIO`/`MovJIO` 的并行 IO 参数原型是 `{Mode,Distance,Index,Status}`，**要求大括号**；而 `str((0,50,1,0))` 产出的是 `(0, 50, 1, 0)`——括号类型错了，不只是多了空格。

## 四、 其他逻辑漏洞

1. **反馈端口受限（与协议不符）**：**成立**。第 148 行只放行 29999/30003/30004，协议明确 3.5.2 及以上支持 30004/30005/30006，用 30005/30006 会被自己抛的异常拦掉。
2. **硬编码的 IP 和端口日志**：**成立，但位置需修正——只有 `MovJIO` 有这一行（801 行），`MovLIO` 没有。** 另外补充两点：`MovJIO` 属于 `DobotApiMove`（30003 端口），日志里写死 29999 是双重错误；且这行日志位于 dynParams 拼接**之前**，打印出来的指令是残缺的（没有 IO 组、没有右括号），比不打印更有误导性。
3. **`EnableRobot` 参数判断漏洞**：**成立**。协议规定可携带参数数量只能是 0/1/4，而 `load != 0` 的判断会让 `EnableRobot(0.0, 0, 0, 30.5)` 退化成 `EnableRobot()`，偏心参数被静默丢弃。补充：反向的 `EnableRobot(1.5, 0, 0, 0)`（显式声明零偏心）也拿不到 4 参数形式，用户无法精确控制下发的参数个数。

## 五、 网络通信与 TCP 粘包处理漏洞

1. **反馈端口解析逻辑严重错误**：三个子项（覆盖而非追加、`> 1440` 跳出条件把恰好收满 1440 的正常情况判为失败、`temp[0:1440]` 无包头对齐校验）**全部成立**。补充三个更前置的问题，见第二部分【严重-4】【严重-5】【严重-8】。
2. **异常被静默吞噬**：**成立**。`finally` 中带 `return` 会连 `BaseException` 一起吞掉，`send_data` 同理。补充返回类型不一致问题，见【健壮性-1】。
3. **指令端口的半包读取**：**成立**。协议规定应答以 `;` 结尾，代码写死单次 `recv(1024)`，没有按结束符循环读取。
4. **幽灵数据残留**：**成立**，而且比文档描述的更容易触发——见【严重-3】，1.0 秒超时对多条协议指令来说是必然超时，不是"偶发网络波动"。

---

# 第二部分：新增发现的问题

## 严重（会导致功能失效或线程卡死）

### 严重-1：`DOGroup` 根本没有把指令发出去

```python
def DOGroup(self, *dynParams):          # 640-646 行
    string = "DOGroup("
    for params in dynParams:
        string = string + str(params) + ","
    string = string + ")"
    print(string)                        # 只打印
    return self.wait_reply()             # 直接去读应答
```

这个方法从头到尾没有调用 `send_data` 或 `sendRecvMsg`，指令只被 `print` 到了控制台。后果有三层：

1. DO 端口不会有任何动作，但函数会返回一个看起来正常的字符串，调用方无法察觉。
2. 它绕过了 `__globalLock`，还会**偷走上一条指令的应答**，从此之后所有 `sendRecvMsg` 的"请求—应答"配对全部错位。
3. 若此时缓冲区为空，它会在 1 秒超时后返回空串。

这是全文件中危害最大的一处，而且是唯一一个"指令完全没有下发"的方法。

### 严重-2：`RelMovLTool` 用了错误的可选参数名

协议原型：`RelMovLTool(...,Tool,SpeedL=R, AccL=R,User=Index)`（直线运动用 `SpeedL/AccL`）。

代码第 974 行拼的是 `SpeedJ=/AccJ=`：

```python
string = string + ", SpeedJ={:d}, AccJ={:d}, User={:d}".format(...)   # 应为 SpeedL/AccL
```

直线相对运动的速度/加速度设置不生效或被控制器判为参数错误。这行代码显然是从 `RelMovJTool` 直接复制的。

### 严重-3：1.0 秒接收超时对阻塞型指令必然超时

`__init__` 中连接成功后把超时收紧到 1.0 秒（154 行），而协议里有多条指令的应答远超 1 秒：

- `Sync()`：阻塞直到队列**最后一条**指令执行完，一次长轨迹可能几十秒；
- `EnableRobot()`：协议说明上电到使能完成约需 10 秒；
- `HandleTrajPoints(traceName)`：轨迹预处理时间随文件大小变化；
- `MoveJog`、`StartTrace`/`StartPath` 等。

这些指令**每次调用都会超时**，返回空串，然后迟到的应答留在接收缓冲区里，把后续所有指令的应答顺序全部推后一位。原文档第五节第 4 条把它描述成偶发问题，实际上它是必然发生的。

### 严重-4：`feedBackData` 里的 `setblocking(True)` 清掉了超时，反馈线程可能永久卡死

```python
self.socket_dobot.setblocking(True)   # 1057 行
```

`setblocking(True)` 等价于 `settimeout(None)`，把 `__init__` 设的 1.0 秒超时抹掉了。一旦控制器停止发送或网线断开，`recv` 会**无限期阻塞**。配合 `dobot_driver.py` 里 `_stop_feedback_thread()` 的 `join(timeout=1.0)`，结果是：线程永远退不出，join 静默失败，线程泄漏；反复启停就会累积一堆卡死的线程和未关闭的 socket。

### 严重-5：`feedBackData` 开头主动丢弃刚收到的完整数据

```python
temp = self.socket_dobot.recv(144000)      # 1060 行
if len(temp) > 1440:
    temp = self.socket_dobot.recv(144000)  # 1062 行：把上面收到的全丢了，再收一次
```

30004 端口每 8ms 推一包，Python 侧调用间隔通常大于 8ms，所以一次 `recv` 拿到多包（`len > 1440`）是**常态**而非异常。这段逻辑在常态下白白丢弃一批数据再重收一次，既翻倍了延迟又浪费带宽；更要命的是它和后面 `while` 里"`> 1440` 才 break"的判断自相矛盾（前面把 `>1440` 当成要丢弃的坏数据，后面又把 `>1440` 当成收好了的标志）。

### 严重-6：`ServoJ` / `ServoP` 等待了一个协议规定不存在的应答

协议对这两条指令的“返回”一栏明确写的是**无**，也就是控制器不会回包。而代码走的是
`sendRecvMsg`，发完就去等应答，于是每次调用都会白等到超时。

协议同时建议这两条指令以 33Hz（约 30ms 间隔）循环调用来实现动态跟随。原实现每次调用
都要卡满 1 秒超时，动态跟随的调用频率根本无法满足，寸动功能实际不可用。修复方式是新增
只发不收的 `sendCmd`，并由 `ServoJ`/`ServoP` 使用。

### 严重-7：`send_data` 用了 `send` 而不是 `sendall`

```python
self.socket_dobot.send(str.encode(string, 'utf-8'))
```

`socket.send` 只保证“尽力发送”，返回实际写出的字节数，在发送缓冲区紧张时可能只写出
一部分。代码既不检查返回值也不补发，指令就会被截断成半条（例如
`MovJ(-500,100,20`），控制器解析后返回参数错误。指令字符串通常很短，这个问题不易复现，
但一旦出现极难排查。应使用 `sendall`。

### 严重-8：完全没有利用协议提供的两个自校验字段

协议在实时反馈表里给了两个专门用于校验的字段：

- `MessageSize`（字节 0~1）：本包总长，应为 1440；
- `TestValue`（字节 48~55）：内存结构测试标准值，固定为 `0x0123456789ABCDEF`。

`TestValue` 存在的唯一目的就是让上位机确认"包头对齐正确、结构体定义与控制器一致"。代码解析完从不校验这两个字段，于是原文档指出的"错位后几百个字段全部错乱"这一情况**没有任何检测手段**——错乱的关节角会被当成真实数据缓存下来，直接喂给运动逻辑。

## 协议格式不符

### 格式-1：`Circle3` 缺少大括号分组

协议原型：`Circle3({X1,Y1,Z1,Rx1,Ry1,Rz1},{X2,Y2,Z2,Rx2,Ry2,Rz2},count,...)`，两个点位必须各用大括号包起来。

代码（829-830 行）把 12 个 double 平铺下发，只有 `count` 跟在后面。这与协议定义的参数结构不一致。

### 格式-2：`SetCoils` 的写入值格式错误 + 残留调试打印

```python
string = "SetCoils({:d},{:d},{:d}".format(...) + "," + repr(offset4) + ")"   # 626-630 行
print(str(offset4))
```

协议要求 `SetCoils(index,addr,count,valTab)`，示例 `SetCoils(0,1000,3,{1,0,1})` 使用**大括号**。`repr([1,0,1])` 产出的是 `[1, 0, 1]`（方括号 + 空格）。另外末尾有一行与业务无关的 `print`。

### 格式-3：`SetPayload` 指令本身不在协议中，且实现有拼接 Bug

V3 协议的负载指令是 `PayLoad(weight,inertia)`（亦可写作 `LoadSet`），协议指令表中**没有 `SetPayload`**，调用它很可能得到 `-10000`（命令不存在）。

同时它的实现也是错的（512-518 行）：第一个动态参数前没有逗号，末尾又多一个逗号，`SetPayload(1.5, 0.4)` 会拼出 `SetPayload(1.5000000.4,)`——数值直接粘连。

### 格式-4：`InverseSolution` 的可选参数结构不符协议

协议可选参数是 `isJointNear`(int) 和 `JointNear`(string，形如 `{0,0,-90,0,90,0}`)，示例：

```
InverseSolution(473.0,-141.0,469.0,-180.0,0.0,-90.0,0,0,1,{0,0,-90,0,90,0})
```

代码用 `repr(params)` 拼接，只能得到 Python 的元组/列表字面量，既没有 `{}` 也没有把 `isJointNear` 和 `JointNear` 分开，配合原文档已指出的漏逗号，这个可选参数路径基本不可用（无参调用是正常的，`dobot_driver.py` 里的 `is_reachable` 恰好只用无参形式，所以暂未暴露）。

### 格式-5：`GetInRegs` 的可选参数取错了内容

除原文档指出的 `params[0]` 崩溃/漏逗号外，协议里这个可选参数是 `valType`（`U16`/`U32`/`F32`/`F64` 之一），代码取 `params[0]` 意味着即使传入字符串 `"U16"`，拼出来的也只有首字符 `U`。

### 格式-6：运动指令的可选参数必须由调用者自己写成 `Key=Value`，且与同类方法风格不一致

`MovJ`、`MovL`、`JointMovJ`、`RelMovJ`、`RelMovL`、`RelMovJUser`、`RelMovLUser`、`RelJointMovJ`、`Arc`、`Circle3`、`MoveJog` 一律是 `string + "," + str(params)` 直拼。协议规定这些可选参数只能以 `SpeedJ=R`、`AccL=R`、`User=index`、`CoordType=typeValue` 形式携带，因此调用者必须自己传字符串 `"SpeedJ=50"`；传数字 `50` 会拼成位置参数，控制器返回 `-20000`（参数数量错误）。

而同一文件里的 `RelMovJTool`/`RelMovLTool` 又是自动补 `Key=` 前缀的（要求传元组）。同一个类的同族方法有两套互不兼容的调用约定，是最容易踩的坑。

### 格式-7：多处在参数间插入了空格

- `StartPath`：`f"StartPath({trace_name}, {const}, {cart})"`（910 行）
- `RelMovJTool`/`RelMovLTool`/`RelMovJUser`/`RelMovLUser`：格式串写作 `"...{:f}, {:d}"`（946、970、995、1017 行）

协议规定"每一个参数之间以英文逗号 `,` 相隔"，未提及允许空白。虽然控制器多半能容错，但与协议示例不一致，建议统一去掉。

## 接口缺失 / 多余

### 接口-1：协议已定义但代码未实现的指令

`DIGroup`、`AI`、`ToolAI`、`SetUser`、`CalcUser`、`SetTool`、`CalcTool`、`PalletCreate`、`GetPalletPose`。

其中 `DIGroup`（批量读 DI）和 `AI`/`ToolAI`（读模拟量）对喷涂场景（气压/流量反馈、多路到位信号）是常用能力。

### 接口-2：协议中是可选参数的，代码全写成了必填

| 方法 | 协议 | 代码 |
|---|---|---|
| `GetPose` | 支持 `User=index,Tool=index` 可选参数 | 完全不支持传参 |
| `HandleTrajPoints` | 允许不带参数调用以轮询预处理结果 | `{:s}` 强制要求参数，无法轮询 |
| `ModbusCreate` | `isRTU` 可选（默认 TCP） | 必填 |
| `GetHoldRegs` / `GetInRegs` | `valType` 可选（默认 U16） | `GetHoldRegs` 的 `type` 必填，且传 `None` 会因 `{:s}` 直接抛异常 |
| `SetTerminal485` | 仅 `baudRate` 常用，协议示例即 `SetTerminal485(115200)` | 四个参数全必填 |

### 接口-3：代码里有一批 V3 六轴协议中不存在的指令

`Arch`、`LimZ`（四轴/Magician 系列遗留）、`SetObstacleAvoid`、`SetTerminalKeys`、`ServoJS`、`StartFCTrace`、`SetPayload`。

这些指令在本协议文档中查不到定义，可能属于其它控制器版本或工艺包。保留它们没有标注版本要求，会让使用者误以为可用，实际调用大概率返回 `-10000`。

### 接口-4：`Jump` 是空实现

```python
def Jump(self):     # 730-731 行
    print("待定")
```

不发指令、不返回值（返回 `None`）。调用方若按其它方法的约定去 `res.find(...)` 会直接 `AttributeError`。协议的 V3 六轴指令表中也确实没有 `Jump`，建议直接删除而不是留一个假实现。

### 接口-5：docstring 中的取值范围与协议不符

| 位置 | docstring | 协议 |
|---|---|---|
| `DO` / `DOExecute` | index 1~24 | `[1,16]` 或 `[100,1000]`（后者需扩展 IO 模块） |
| `GetHoldRegs` / `SetHoldRegs` | count 1~16，addr 3095~4095 | count `[1,4]`，index `[0,4]` |
| `MoveJog` | coord_type "1: 用户坐标系，2: 工具坐标系，默认 1" | `CoordType`：0=用户，1=关节（默认），2=工具 |

`MoveJog` 那条尤其危险：按 docstring 传 `CoordType=1` 想用用户坐标系，实际得到的是关节点动，**运动方向完全不同**。

### 接口-6：`ServoJ` 无条件下发可选参数

协议注明 `t`/`lookahead_time`/`gain` 三个可选参数**仅控制器 3.5.5 及以上支持**。代码（846 行）无条件把三者都拼上，在低版本控制器上会得到参数数量错误。另外 `{:f}` 会拼出 `lookahead_time=50.000000`、`gain=500.000000`，虽为 float 型合法，但与协议示例 `lookahead_time=50` 的写法不同。

## 健壮性 / 工程问题

### 健壮性-1：`wait_reply` 返回类型不一致，且不识别断线

```python
data = ""                      # str
data = self.socket_dobot.recv(1024)   # bytes
...
if len(data) == 0:
    data_str = data            # 这里可能是 str "" 也可能是 bytes b""
else:
    data_str = str(data, encoding="utf-8")   # str
```

对端关闭连接时 `recv` 返回 `b''`，此时函数返回的是 **bytes**；超时（异常被吞）时返回的是 **str**。上层做 `res.startswith("Error")` 时前者会 `TypeError`（`dobot_driver.py` 里靠 `if res:` 的真值判断侥幸绕过了）。

更重要的是：`recv` 返回空 bytes 在 TCP 语义里明确表示"对端已关闭连接"，代码没有据此判定断线，也没有任何重连机制，只是继续正常返回空串。

### 健壮性-2：大量 `{:d}` 强制整型，传浮点直接抛异常

`SpeedFactor`、`AccJ`、`AccL`、`SpeedJ`、`SpeedL`、`CP`、`User`、`Tool`、`DO`、`DI`、`wait`、`LimZ`、`SetCollisionLevel` 等均用 `{:d}`。上层做过比例换算（如 `velocity * 0.8`）后传入 `50.0` 会直接 `ValueError: Unknown format code 'd' for object of type 'float'`。`dobot_driver.py` 里到处写 `int(velocity)`、`int(acc)` 兜底，说明这个坑已经被踩过了。

### 健壮性-3：调试 `print` 残留，绕过日志机制直写 stdout

- `print(string)`：`MovJ`(697)、`MovL`(715)、`DOGroup`(645)、`SetCoils`(629)
- `print(type(params), params)`：`InverseSolution`(529)、`GetInRegs`(616)、`RelMovJTool`(949)、`RelMovLTool`(973)

类里明明已经有 `log()` + `verbose` 开关机制，这些 print 完全绕过它。`MovJ`/`MovL` 是高频指令（轨迹回放时每秒几十次），无条件打印会刷屏并拖慢执行。

### 健壮性-4：顶层导入 tkinter，让驱动强依赖 GUI 环境

```python
from tkinter import Text, END    # 第 3 行
```

仅为一个可选的 GUI 日志框（`self.text_log`）而在模块顶层硬依赖 tkinter。在无 X11 的工业机、Docker 容器、精简版 Python（未装 python3-tk）上，**import 阶段就会失败，整个机器人模块不可用**。应改为惰性导入或直接用鸭子类型（任何有 `insert` 方法的对象）。

### 健壮性-5：连接失败时丢失真实错误原因

```python
except socket.error:
    print(socket.error)                                    # 打印的是异常"类"，不是实例
    raise Exception(f"Unable to set socket connection use port {self.port} !", socket.error)
```

没有写 `as e`，`print` 和 `raise` 里传的都是 `socket.error` 这个类本身，输出永远是 `<class 'OSError'>`。到底是连接超时、连接被拒绝、还是路由不可达，全部丢失——而这三种情况在现场的排查方向完全不同。另外连接失败时已创建的 socket 没有 `close()`。

### 健壮性-6：端口非法时的异常文案会误导排查

```python
raise Exception(f"Connect to dashboard server need use port {self.port} !")
```

字面意思是"连接 dashboard 需要用端口 xxx"，而实际语义是"端口 xxx 不在支持列表内"。现场看到这条日志很容易反向理解。

### 健壮性-7：完全没有解析协议应答，也不判断 ErrorID

协议应答格式是明确的：`ErrorID,{value,...},消息名称(...);`，且给出了完整错误码表（`-1` 执行失败、`-10000` 命令不存在、`-20000` 参数数量错误、`-3000x` 第 x 个参数类型错误、`-4000x` 第 x 个参数范围错误）。

当前所有方法都直接返回原始字符串，既不解析也不校验 `ErrorID`。于是上层只能靠字符串包含关系去猜，例如 `dobot_driver.py` 里的 `if res and "0" in res` 和 `"0,{" in res`——但 `-10000`、`-20000`、`-30001` 里**都含有字符 "0"**，这种判断在失败时同样会返回 True。建议在 `DobotApi` 层加一个统一的应答解析（拆出 ErrorID / 返回值列表 / 回显指令），并提供可选的"非 0 即抛异常"模式。

### 健壮性-8：`MyType` 中所有 char 字段用了有符号 int8

字节布局本身是对的（见第三部分），但 `np.byte` 是 **int8（有符号）**，协议里这些字段是 `char` 且按无符号语义使用：

- `CRRobotType` 的取值包含 **160**（CR10V2YD），int8 读出来是 **-96**；
- `BrakeStatus`、`DigitalInputs`/`DigitalOutputs` 这类按位字段，最高位为 1 时会变成负数，做位运算容易出错；
- `VelocityRatio` 等 0~100 的比例字段目前安全，但同样应统一为 `np.uint8`。

### 健壮性-9：`len` 字段吞掉了协议的保留位

```python
('len', np.int64)    # 占据字节 0~7
```

协议里字节 0~1 是 `MessageSize`（unsigned short），字节 2~7 是 3 个保留 short。用一个 int64 覆盖 8 个字节，目前因为保留位恒为 0 才碰巧能读出 1440，语义上不可靠。正确做法是 `('MessageSize', np.uint16), ('reserve1', np.uint16, (3,))`。

### 健壮性-10：`user`/`tool` 与 `user[6]`/`tool[6]` 字段命名混乱

前者是 char 型的坐标系**索引**（字节 1012/1013），后者是 double[6] 的坐标系**数值**（字节 1200~1295），语义完全不同却只差一个 `[6]`；而且字段名里带方括号，访问时只能写 `d['user[6]']`，非常别扭。建议改为 `user_index`/`tool_index` 与 `user_frame`/`tool_frame`。

### 健壮性-11：`DobotApiFeedBack` 继承了发送接口

30004/30005/30006 是纯只读反馈端口，但 `DobotApiFeedBack` 继承了 `send_data`/`sendRecvMsg`，误调用会往反馈端口写数据。另外 `feedBackData` 里 `data = bytes()` 是无用变量，还残留多段注释掉的计时代码。

### 健壮性-12：`alarmAlarmJsonFile` 是死代码

模块顶层定义了它来读取 `files/alarm_controller.json` 和 `files/alarm_servo.json`（文件确实存在），但 `GetErrorID` 的返回值完全没有和它关联。协议明确说明 `GetErrorID` 返回的 id 需要查这两个文件才能得到人类可读的报警描述——这块能力目前是断开的。

### 健壮性-13：`socket_dobot = 0` 用整数当哨兵值

`self.socket_dobot` 初始化为整数 `0`，连接成功后变成 socket 对象，`close()` 里用 `if self.socket_dobot != 0` 判断。类型混用不利于静态检查，建议用 `Optional[socket.socket] = None`。另外 `__del__` 中调用 `close()`，在解释器退出阶段属性可能已被回收而抛异常。

### 健壮性-14：`EnableRobot` 缩进与全文件不一致

218-226 行使用 2 空格缩进，文件其余部分是 4 空格。虽然 Python 语法允许，但会触发 lint 告警，也说明这段是后期手工插入的。

---

## 延伸风险（源于本文件，但暴露在调用侧）

`dobot_driver.py` 用 30004 反馈里的 `running_status` 判断运动是否完成（`is_robot_idle()` 判 `== 0`）。协议对该字节的说明只是"机器人运行状态"，而判断队列指令是否执行完毕，协议推荐的方式是 `Sync()` 阻塞或用 `RobotMode()` 判 7（RUNNING）/5（ENABLE 空闲）。由于本文件的 `Sync()` 必然踩到【严重-3】的 1 秒超时，上层被迫绕道用 `running_status`——修 `wait_reply` 超时策略时需要连带评估这条链路。

---

# 第三部分：核对通过、无需改动的部分

为避免后续误改，记录已逐项核对确认**正确**的部分：

1. **`MyType` 的字节布局与协议实时反馈表完全吻合。** 实测 `itemsize == 1440`，且逐字段偏移对齐：三个保留 double[3] 覆盖 120~191，17 组 double[6] 覆盖 192~1007，`hand_type` 起于 1008，`reserve2(82)` 覆盖 1038~1119，`m_actual` 起于 1120，`reserve3(24)` 起于 1416 正好收尾于 1440。字段**顺序和大小不要动**，需要改的只有前述的符号性（int8→uint8）和 `len` 字段拆分。
2. **`Arc` 使用平铺的 12 个 double 是正确的**（协议原型 `Arc(X1,...,Rz1,X2,...,Rz2,...)` 确实不带大括号），不要照 `Circle3` 的样子给它加大括号。
3. **指令的端口归属正确**：`TCPSpeed`/`TCPSpeedEnd`/`wait`/`Pause`/`Continue` 放在 `DobotApiDashboard`（29999），`Sync` 放在 `DobotApiMove`（30003），均与协议一致。
4. **`pause()` / `continue()` 用小写不是问题**：协议明确"TCP/IP 远程控制指令不区分大小写"。
5. **反馈数据按小端解析是正确的**：协议规定小端存储，numpy 在 x86 上默认小端（dtype 显示为 `<f8`）与之一致。
