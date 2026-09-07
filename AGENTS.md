# AiSprayer 项目开发与工业机器人控制强制规范 (Mandatory Project Rules)

本文件是当前项目的全局最高优先级常驻规则（Always-On Rule），AI 助手在进行任何任务时均会无条件自动加载并严格遵循。

## 核心准则：强制激活与遵循 Skill
在本项目中凡是涉及**代码编写、功能修改、Bug修复、审查重构、或者机械臂控制与轨迹喷涂开发**，必须自动激活并严格遵循以下 Skill：
* **Skill 名称**：[`code-quality-and-robotics-practices`](file:///.agents/skills/code-quality-and-robotics-practices/SKILL.md)
* **Skill 路径**：`.agents/skills/code-quality-and-robotics-practices/SKILL.md`

---

## 必须在每次执行中严格执行的四大铁律

### 1. 死代码与重复代码归零 (Zero Dead Code & DRY)
- 在任何修改或审查中，一旦发现未使用的变量、废弃函数、无用别名或重复判定逻辑，必须主动清除或抽象抽取为公共纯函数；
- 数据结构统一转换口径（如位姿统一使用 `RobotPose`），禁止平行拼凑实现。

### 2. 高内聚、低耦合与高扇入架构 (Architecture Standards)
- **驱动层 (`BaseRobotDriver`)**：只做硬件协议通信与轮询，严禁侵入业务逻辑；
- **服务层 (`RobotService`)**：单例管理生命周期、WebSocket 状态广播与线程安全编排；
- **API 层 (`api.py`)**：负责 HTTP 参数防御与校验分发；
- 核心基础支撑逻辑保持高扇入（复用度高），业务逻辑保持低扇出。

### 3. 机械臂控制工业级实践 (Robotics Best Practices)
- **喷涂与运动同步**：
  - 连续轨迹喷涂切换必须使用**队列指令 (`DO`)**，与运动插补在段间无缝衔接，保证 CP 连续平滑，段间绝不打断；
  - 手动开关、急停（E-Stop）、异常保护**必须使用立即指令 (`DOExecute`)**，毫秒级快速断料。
- **状态判定与防误触互锁 (Interlock Protection)**：
  - 机械臂运动状态以控制器反馈为唯一准理源（`running_status`：`0=Idle`, `1=Moving`）；
  - 只要机械臂处于运动中（`status === 1`），UI 界面所有执行、回零、点动按钮必须置灰锁定，API 层防御拦截。
- **坐标系与量纲**：内部计算一律采用毫米 `mm` 与弧度 `rad`；通信封包显式转换为 `deg`。
- **故障安全 (Fail-Safe)**：发生任何异常或中断，**首要动作必须强制关闭喷涂 DO (置 0)**。

### 4. 保持代码简单直观与保留关键注释 (KISS & Preserve Comments)
- 遵循 KISS 原则，不进行过度抽象；
- 严禁删除现有代码中的关键领域注释，必须保留量纲、物理意义、协议格式与算法原理说明。

### 5. 全界面与交互显示必须全英文 (Strict English-Only UI Interface)
- **100% 英文界面**：前端界面的所有元素、按钮、标签、卡片标题、Tooltip 悬浮提示、Badge 徽章、右键菜单项、HUD 动作胶囊 Pill 文本、Notice 通知弹窗；
- **用户可见的后端提示全部英文**：后端向前端 WebSocket 广播推送的所有执行状态与动作描述（如 `broadcast_exec_status(action=...)`）、以及抛给前端的 HTTP 异常 `detail` 报错信息，**必须全部使用英文，严禁在界面和给用户的提示中出现中文**。
