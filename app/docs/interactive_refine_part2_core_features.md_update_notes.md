# 交互操作界面优化方案（核心功能篇：需求 5～7）统一问题分析与验收记录

版本：2.0  
状态：代码核对完成，问题待修复  
基准设计：`app/docs/interactive_refine_part2_core_features.md`  
核对范围：`app/src`、`app/frontend/src/components/operations/interactive`  
重要约束：所有新生成和读取的运行文件名必须以 `scan.` 开头；不兼容、不迁移、不回退旧版本文件。

> 本文档已将文件体系、RAW/OPT/POI 优化、前端状态同步、Waypoint 对比、2D/3D 显示和真实机械臂安全要求合并为一条完整链路。后续修复必须同时满足后端数据、前端显示和执行安全要求，不能只修某一层的表面现象。

---

## 1. 统一结论

当前核心功能并非全部缺失，而是已经存在多个局部实现，但各层使用的数据快照和状态来源不统一，造成以下用户现象：

- Waypoint 对比中 RAW、OPT、POI 数值看起来都和 OPT 一样，或对比对象不确定。
- 优化完成后 2D 图有时不立即刷新，状态切换后可能仍显示上一状态。
- 2D 表面路径和红色法线箭头几乎不变化，无法证明 TCP 姿态优化是否生效。
- 3D 路径状态可能只变颜色、不换实际路径数据。
- 报告、离散 Waypoint、2D 表面路径和 dense 仿真轨迹来自不同数据层级。
- POI 当前算法可能将所有点的姿态压到同一个绝对 Anchor 附近，是否符合喷涂工艺尚未闭环确认。
- 在真实机械臂执行前，当前系统缺少足够强的路径版本一致性和安全放行门。

统一判断：这是一个“路径状态与数据快照一致性问题”，同时叠加“优化数学模型需要工艺确认”和“仿真展示语义不完整”。必须整体修复和验收。

---

## 2. 当前已实现与未实现边界

| 领域 | 当前已确认实现 | 当前缺口或风险 |
|---|---|---|
| 文件命名 | 新流程主要写入 `scan.raw.path.yaml`、`scan.opt.path.yaml`、`scan.poi.path.yaml` 及对应报告 | 读取仍包含无前缀和 `scan.manual_*` 回退候选，违反零兼容要求 |
| RAW 验证 | `PathVerificationService` 可验证 raw 路径并生成报告 | 报告与画布路径未建立版本/快照绑定 |
| OPT 优化 | `AxialSpinOptimizer` 支持工具局部 Z 轴离散旋转搜索 | 局部候选搜索后 Euler 平滑没有明确再次完整验证，修改结果缺少逐点差值 |
| POI 优化 | `PoiConstraintOptimizer` 支持 Anchor、容差网格、IK 候选评分和奇异性惩罚 | 当前 `R_ref @ R_delta` 是绝对 Anchor 模型，是否符合曲面喷涂姿态要求未确认；实际配置记录也可能不一致 |
| Anchor Pose | 已有 Home / Live API 与前端入口 | Live 不可用时存在回退 Home 的风险，可能造成错误锚定 |
| 三态状态 | 前端已有 RAW / OPT / POI 状态和诊断矩阵 | 状态切换与异步优化结果存在旧闭包问题，未形成单一状态源 |
| Waypoint 对比 | 已有 Raw/Opt/POI 姿态列 | 主要按数组下标对齐，位置列混用 Raw，缺少稳定 key 和差值校验 |
| 2D 显示 | 已有路径线、表面点、法线投影、仿真光标和进度着色 | 使用原始 `pixel` / `normal_2d_proj`，不会反映优化后的 TCP 姿态；表面路径与 TCP 路径语义混淆 |
| 3D 显示 | 已有 TCP 路径、表面轨迹、工具坐标轴和关节更新接口 | 路径加载 effect 未完整依赖 `pathState`，快速切换可能显示旧路径 |
| 仿真 | 已有 `requestAnimationFrame`、dense trajectory、投影和播放控件 | 多路径进度使用全局比例，结束后循环回起点，3D/2D/路径快照一致性未验证 |
| 真实执行 | 当前核对范围内没有足够的完整安全放行链路 | 在所有 P0/P1 问题关闭前不得下发真实机械臂 |

---

## 3. 统一数据模型和状态链路要求

RAW、OPT、POI 必须被视为三个独立且不可互相覆盖的路径快照：

```text
scan.raw.path.yaml  -> scan.raw.report.json  -> RAW snapshot
scan.opt.path.yaml  -> scan.opt.report.json  -> OPT snapshot
scan.poi.path.yaml  -> scan.poi.report.json  -> POI snapshot
```

每个 snapshot 必须同时绑定：

- `state_type`：`raw`、`opt` 或 `poi`。
- `source_file` 和 `report_file`。
- 模板名称和更新时间。
- 路径数量、`path_id`、waypoint index。
- 每个 waypoint 的 surface data、TCP pose、pixel 和法线数据。
- 优化配置、Anchor 来源、容差、TCP 配置、基坐标系和速度配置。
- 报告生成时使用的路径内容版本或内容摘要。

前端必须从同一份 snapshot 更新以下内容：

1. `rawPaths` / `optPaths` / `poiPaths`。
2. 当前 `manualPaths` 和 `activeState`。
3. 对比表数据。
4. 2D overlay 数据。
5. 3D viewer 路径数据。
6. 对应状态的 verification report。
7. 仿真数据和当前状态。

任何一项来自旧 snapshot，都必须阻止显示为“已完成”。

---

## 4. 统一问题清单、修复方式和验收证据

### P0-1：旧文件名兼容逻辑必须全部删除

涉及：

- `app/src/apps/interactive/path_verification_service.py`
- `app/src/apps/interactive/manual_path_service.py`
- 前端可能使用旧文件名判断状态的逻辑

当前问题：

服务仍尝试读取 `{state}.path.yaml`、`scan.manual_{state}_paths.yaml`、`scan.manual_paths.yaml`、无前缀报告以及旧 report 名称；保存 raw 时也清理旧文件。这会让运行时行为同时存在两套文件协议。

统一修复方式：

1. 路径白名单固定为：
   - `scan.raw.path.yaml`
   - `scan.opt.path.yaml`
   - `scan.poi.path.yaml`
2. 报告白名单固定为：
   - `scan.raw.report.json`
   - `scan.opt.report.json`
   - `scan.poi.report.json`
3. 删除所有旧候选、旧回退、旧清理、旧映射和旧错误提示。
4. `state_type` / `mode` 只允许 `raw`、`opt`、`poi`。
5. 新增公共命名函数或常量，调用方不得自行拼接其他文件名。
6. 已存在的旧文件不读取、不转换、不删除、不兼容。

验收证据：

- 运行时代码搜索不到旧文件名分支。
- 新模板只会生成上述六个路径/报告文件。
- API 对非法状态直接返回错误。
- 删除旧文件不会影响新流程，因为新流程不依赖旧文件。

### P0-2：优化结果必须通过单一快照进入前端

涉及：`InteractiveOp.tsx`。

当前问题：

优化完成后先执行 `setOptPaths(paths)` 或 `setPoiPaths(paths)`，再立即调用 `handleSelectActiveState()`。由于 React 状态更新异步，状态切换函数可能仍读取旧闭包中的 `optPaths` / `poiPaths`，导致报告已经更新但画布仍显示旧路径。

统一修复方式：

1. 优化成功后使用接口返回的 `paths` 直接构建目标 snapshot。
2. 同一个事件中同步更新目标状态路径、`manualPaths`、`activeState`、report 和 `onPathStateChange`。
3. 状态切换函数接收显式路径快照，不能从可能过期的闭包推导目标数据。
4. 每次路径切换清理旧的 hover、highlight、仿真和选中 waypoint 状态。
5. 快速切换时使用 request id 或取消机制，旧请求不能覆盖最后一次状态。
6. 优化成功提示只能在路径、报告、2D 和 3D 数据全部完成切换后显示。

验收证据：

- 优化接口返回后无需刷新页面，2D、Waypoint 表和 3D 都显示新结果。
- 连续点击 RAW → OPT → POI → RAW，最终四处都显示 RAW。
- report 的 `state_type`、路径文件名和当前 `activeState` 三者一致。

### P0-3：Waypoint 对比必须用稳定标识对齐，不能用数组下标

涉及：`DiagnosticsDashboard.tsx`。

当前问题：

表格由当前状态数组渲染，再用同一数组下标读取 raw、opt、poi。实现默认三种路径数量、顺序和 waypoint 数量永远一致，但代码没有校验。位置列还使用 Raw XYZ，姿态列使用三种状态，造成一行数据的来源混杂。

统一修复方式：

1. 使用 `(path_id, waypoint index)` 作为唯一对齐 key。
2. Raw、Opt、POI 的 XYZ 和 Rx/Ry/Rz 分别来自各自 snapshot。
3. 任一状态缺点时显示缺失，不允许使用其他状态的数据 fallback。
4. 对齐失败时显示 WARNING，并禁止标记为可执行。
5. 显示至少以下差值：
   - `Opt - Raw`：位置和姿态差。
   - `POI - Raw`：位置和姿态差。
   - 可选 `POI - Opt`：位置和姿态差。
6. 对象进入三态状态仓库时做深拷贝，禁止本地编辑通过引用改变其他状态。

验收证据：

- 人工交换 OPT 路径顺序后，表格仍按 `path_id` 正确对应。
- 删除某个 POI waypoint 后，POI 单元格显示缺失而不是显示 Raw/Opt。
- 任意姿态变化都能在对应状态列和差值列中看到。
- 表格中每个数值可追溯到具体文件、path id 和 waypoint index。

### P0-4：2D Surface Path、TCP Path、Tool Orientation 必须分离

涉及：`PathSvgOverlay.tsx`、`ManualPathService`、优化器。

当前问题：

当前 2D 使用：

- 路径线：`point.pixel`。
- 表面点：`point.pixel`。
- 红色箭头：`point.normal_2d_proj`。
- 仿真光标：dense `trajectory_tcp` 投影。

OPT/POI 主要修改 `tcp_pose_base`，不重新生成原始 surface pixel 和法线投影。因此表面线和红箭头不变是当前实现的结果，但界面没有清楚区分这几类数据，用户无法判断姿态优化是否生效。

统一修复方式：

1. 明确命名并分别渲染：
   - Surface Path：工件表面采样路径。
   - TCP Path：各状态 TCP 位置的相机投影。
   - Surface Normal：表面法线投影，默认不随姿态优化变化。
   - Tool Axis：优化后的 TCP 工具轴投影。
2. RAW、OPT、POI 分别根据各自 `tcp_pose_base` 计算 TCP/Tool Axis 投影。
3. 相机后方或视野外的点隐藏，不使用旧 pixel 兜底，不画跨越画布的错误连线。
4. 2D 图例和颜色必须说明红色箭头是 Surface Normal，不得让用户将其理解为优化后的枪体姿态。
5. Waypoint tooltip 显示当前状态 TCP pose、Surface Normal 和姿态差值。

验收证据：

- 仅改变 Rz 时 Surface Normal 红箭头保持不变，同时 Tool Axis 或姿态数值可见变化。
- 改变 Rx/Ry 时，TCP/Tool Axis 投影按状态变化。
- Raw/Opt/POI 的 surface path 可以重合，但 TCP path 和工具姿态能区分。
- 视野外 TCP 不在画面边缘产生错误线段。

### P0-5：3D 路径状态必须跟随 `pathState` 重新加载

涉及：`Robot3DViewer.tsx`。

当前问题：

3D 路径加载 effect 没有完整依赖 `pathState`，且仍混用二值 `useOptPaths`。RAW → POI 时可能只改变 badge 或颜色，实际加载的数据仍是上一状态。

统一修复方式：

1. 统一使用 `pathState` 作为路径状态来源。
2. 每次 RAW/OPT/POI 切换时清理旧 group 并加载对应 `scan.xxx.path.yaml`。
3. 3D 路径、TCP 点、法线线、工具坐标轴颜色都由同一个状态主题计算。
4. 请求返回时校验 request id 和 state，旧请求不能覆盖新状态。
5. 不再使用 `useOptPaths` 表达三态状态。

验收证据：

- 三态连续切换后，3D 路径点坐标、姿态轴和颜色均对应当前状态。
- 当前显示的 3D 路径文件名与 2D 当前 snapshot 一致。
- 快速切换过程中不会出现旧状态晚返回覆盖新状态。

### P1-1：POI 已明确为“绝对姿态 + 容差”的 Anchor 约束模型

涉及：`poi_optimizer.py`、`cr5_path_verifier.py`。

工艺定义：

POI 在本项目中不是“每个 waypoint 严格跟随工件局部法线”的相对姿态约束，而是“机器人基坐标系下的绝对 Anchor 姿态 + Rx/Ry/Rz 容差窗口”的姿态约束。也就是说，每个 waypoint 的候选姿态应围绕同一个参考姿态 `R_ref` 搜索：

```python
R_cand = R_ref @ R_delta
```

其中 `R_ref` 来自 Home 姿态、机器人实时姿态或用户显式输入；`R_delta` 被限制在设定的容差窗口内，例如默认 `±Rx=3°`、`±Ry=15°`、`±Rz=180°`。

采用绝对姿态约束的原因：

1. 深度相机的点云和法线估计会受到噪点、反光、遮挡、边缘缺失和深度孔洞影响；若每个 waypoint 都严格跟随局部法线，姿态会产生抖动。
2. 工件表面曲面复杂时，局部法线变化可能过快，直接转化为喷枪姿态会带来欧拉角跳变、关节速度尖峰和喷涂不连续。
3. 机械臂受关节限位、奇异区、腕部姿态和 IK 分支限制，并不是所有由表面法线推导出的姿态都有解。
4. 实际喷涂更需要稳定、可执行、可复现的喷枪姿态窗口；允许在绝对 Anchor 附近小范围浮动，比逐点追随噪声法线更安全。
5. `Rz` 可保留较大自由度，用于利用喷枪轴向对称性规避奇异点和关节超速；`Rx/Ry` 则用于限制喷枪姿态不要偏离工艺设定过大。

当前问题：

当前代码方向与上述“绝对姿态 + 容差”定义基本一致，但仍需要补齐以下可信度和安全验证：

- 原始 waypoint 的表面法线不再作为最终姿态的唯一基准，因此 UI 必须明确区分 Surface Normal 和 POI Tool Pose。
- 虽然姿态以绝对 Anchor 为主，仍必须监控喷枪轴线与表面法线的夹角，避免姿态稳定但喷涂角度过差。
- 容差内搜索不能只在离散 waypoint 层面局部评分，最终仍要在 dense MoveL 轨迹上验证连续性、关节速度和奇异性。
- Anchor 姿态、容差、TCP 偏置、坐标系和 Euler 顺序必须被写入最终路径快照，保证真实机械臂执行时可追溯。

统一修复方式：

1. 保留 POI 的绝对 Anchor 数学模型：`R_cand = R_ref @ R_delta`。
2. 在配置和 UI 中明确命名为 Absolute Anchor Pose Constraint，避免误解为 Surface-Normal-Following。
3. 每个候选保存：Anchor pose、candidate pose、相对 Anchor 误差、与原始 Raw 姿态差、与表面法线夹角、standoff、IK 分支和评分。
4. 优化目标必须同时考虑：容差约束、IK 可解、关节连续性、远离奇异区、速度限制、与表面法线的允许夹角。
5. 最终路径必须重新执行完整 MoveL 插值、连续 IK、关节限位、速度、奇异性和碰撞检查。
6. Anchor 的来源必须明确记录为 `home`、`live` 或 `manual`；`live` 不可用时不得静默回退为 `home`。
7. Anchor 的 Euler 顺序、单位、TCP 偏置和基坐标系必须与真实控制器逐项确认。

验收证据：

- 文档、UI 和报告均明确 POI 是 Absolute Anchor + Tolerance，不是逐点法线跟随。
- 每个 POI waypoint 都有可审计的 Anchor 误差、Raw 姿态差、法线夹角、standoff 和 IK 分支。
- 重新验证后的报告没有不可达、分支跳变、超速或奇异性问题。
- 2D/3D UI 同时展示 Surface Normal 与 POI Tool Axis，避免把红色法线箭头误认为优化姿态。
- 单点和单段低速实测姿态与离线报告一致。

### P1-2：OPT 最终姿态和 POI 最终姿态都必须重新完整验证

涉及：`axial_optimizer.py`、`poi_optimizer.py`、`kinematic_chain_verifier.py`。

当前问题：

OPT 在局部候选搜索后执行 Euler 平滑，POI 也逐 waypoint 评分；最终修改后的路径需要以最终 YAML 再次验证。否则报告可能针对搜索中间结果，而不是最终下发路径。

此外，OPT baseline 已经 PASS 时可能只做平滑，OPT 与 RAW 相同是合法结果，不能被误认为优化失败；`path_modified` 也不能仅因找到候选就无条件为 true。

统一修复方式：

1. 任何最终姿态平滑、速度调整或路径写盘后，都重新读取最终 `scan.opt.path.yaml` / `scan.poi.path.yaml` 再生成报告。
2. 用旋转矩阵或四元数角距离判断是否真的改变姿态。
3. 报告写入每个 waypoint 的修改状态：unchanged、modified、unresolved。
4. 报告记录 Raw/Opt/POI 的关节差、姿态差、峰值速度和奇异性结果。
5. 报告中的 `source_file` 必须指向实际验证的最终文件。

验收证据：

- 报告删除后重新验证，结果与优化接口返回的报告一致。
- 修改后的 YAML 内容与报告输入内容一致。
- baseline PASS 时明确显示 unchanged，而不是伪造 modified。
- 最终报告状态不是 PASS 时，前端和执行入口都不能放行。

### P1-3：配置、速度和 Anchor 来源必须写入最终路径快照

当前问题：

POI 默认容差实际使用 `[3.0, 15.0, 180.0]` 时，`poi_config` 可能记录传入的 `None`；推荐安全速度目前主要写入报告或 `recommended_speed_mm_s`，不一定成为仿真和执行真正使用的速度；Live Anchor 不可用时可能回退 Home。

统一修复方式：

1. 对 ref pose、tolerance、速度、TCP 偏置、坐标系做有限数值、长度和范围校验。
2. 将实际生效的配置写入最终路径 YAML 和 report JSON。
3. `source=live` 且实时姿态不可用时直接报错，不允许静默返回 Home。
4. 区分 `nominal_speed_mm_s`、`recommended_safe_speed_mm_s` 和 `execution_speed_mm_s`。
5. 仿真与真实执行必须读取同一个最终 `execution_speed_mm_s`。
6. 如果执行入口还不能应用安全速度，报告必须为 WARNING，不能显示可执行 PASS。

验收证据：

- 报告、路径 YAML、仿真控制器和执行器显示同一个生效配置。
- Live 断开时 Robot/Capture 操作明确失败。
- 超速测试中执行速度不会高于验证后的安全速度。

### P1-4：仿真必须使用统一时间轴并在终点停止

涉及：`InteractiveOp.tsx`、`PathSvgOverlay.tsx`、3D viewer 连接层。

当前问题：

仿真已有 `requestAnimationFrame` 和 dense trajectory，但多路径进度使用全局比例；路径段高亮按当前离散点数量推导；结束时 `stepIndex` 回到 0。这样会导致 2D 已走段与当前 TCP 光标不一致，也无法准确表达末点状态。

统一修复方式：

1. 统一仿真状态包含 state、snapshot id、speed、global step、path index、local path progress、joints、TCP pose 和 projected pixel。
2. 2D 和 3D 使用同一帧的 dense step。
3. 记录每条 path 的 dense step 起止范围，使用 local progress 渲染该路径。
4. 默认到最后一步停止并保持终点，循环播放必须显式开启。
5. 真实执行逻辑不能复用循环仿真逻辑。
6. 仿真启动前校验 report、路径 snapshot 和当前 state 完全匹配。

验收证据：

- 多路径仿真时当前路径局部进度、2D 已走段和 TCP 光标一致。
- 仿真到 100% 后保持末点，不跳回起点。
- 2D 与 3D 在同一帧的 TCP pose 和 joint state 一致。

### P2-1：文件 More 菜单和真实执行入口必须遵守同一安全门

当前问题：

文件列表已有 Sim、Diag 和右键菜单，但缺少完整的 Simulate All、Simulate Single Path、速度设置和真实执行状态检查。

统一修复方式：

1. Simulate Single Path 必须传递具体 `path_id`。
2. Simulate All 使用当前 snapshot 的所有路径，不能重新从其他状态读取。
3. 速度由统一仿真状态管理，不能由文件行单独维护。
4. Execute on Robot Arm 必须检查：当前 snapshot、最终报告、机器人连接、急停、限位、TCP、坐标系、速度和人工确认。
5. 任何 WARNING、FAILED、ERROR、缺失报告或快照不一致都禁用真实执行。

验收证据：

- 文件列表操作、诊断面板、2D 仿真和 3D 仿真使用同一 state/path id。
- 不满足安全条件时 Execute 按钮不可用。
- 真实执行前显示最终文件名、报告状态、速度、TCP 和路径摘要。

---

## 5. 统一实施顺序

### 阶段 A：冻结数据协议

1. 固定六个 `scan.` 路径/报告文件名，删除所有旧兼容分支。
2. 固定 RAW/OPT/POI 的 snapshot 结构、状态字段、路径 key 和配置字段。
3. 禁止无 `scan.` 前缀的运行时读写。

### 阶段 B：修复后端优化和报告可信度

1. 明确 POI 绝对/相对姿态数学定义。
2. 对最终 OPT/POI 文件重新完整验证。
3. 写入实际 Anchor、容差、TCP、速度和坐标系配置。
4. 增加姿态差、法线夹角、standoff、IK 分支和速度结果。

### 阶段 C：修复前端三态一致性

1. 使用单一 snapshot 更新三态路径、报告、当前状态、2D 和 3D。
2. 修复优化后的旧闭包问题和异步请求覆盖问题。
3. 用 `(path_id, waypoint index)` 对齐对比表并显示差值。
4. 清理状态切换时的旧 hover、highlight、simulation 和 selection。

### 阶段 D：修复 2D/3D 语义和仿真

1. 分离 Surface Path、TCP Path、Surface Normal 和 Tool Axis。
2. 让 3D viewer 严格跟随 `pathState`。
3. 使用统一 dense 时间轴和 local path progress。
4. 仿真默认到终点停止。

### 阶段 E：真实机械臂安全验证

1. 离线报告和最终文件一致性验证。
2. 单点、单段、空载、低速执行。
3. 验证急停、限位、断链和速度限制。
4. 人工确认后才能放行整条路径。

---

## 6. 统一验收清单

### 6.1 文件和状态一致性

- [ ] 运行时只生成和读取以 `scan.` 开头的文件。
- [ ] 源码中无旧文件名回退、迁移、双向映射和旧清理逻辑。
- [ ] RAW / OPT / POI 各自拥有独立路径和报告快照。
- [ ] `state_type`、路径文件名、报告文件名和 UI 当前状态一致。
- [ ] 快速切换不会被旧异步响应覆盖。

### 6.2 优化正确性

- [ ] POI 的绝对/相对姿态定义已经由工艺要求确认。
- [ ] Raw、Opt、POI 的 TCP pose、姿态差和关节差可逐点追溯。
- [ ] OPT 最终 Euler 平滑后重新完成完整验证。
- [ ] POI 最终路径重新完成 MoveL、连续 IK、速度、限位、奇异性和碰撞检查。
- [ ] 报告使用的是最终写盘文件，而不是中间内存结果。
- [ ] 实际 Anchor、容差、TCP、坐标系和执行速度写入最终 snapshot。
- [ ] Live Anchor 不可用时不会回退 Home。
- [ ] 推荐速度和真实仿真/执行速度一致，否则状态必须是 WARNING。

### 6.3 Waypoint 和 2D/3D 显示

- [ ] Waypoint 按 `(path_id, waypoint index)` 对齐，不按数组下标猜测。
- [ ] Raw、Opt、POI 的位置和姿态分别来自各自 snapshot。
- [ ] 缺失 waypoint 显示缺失，不使用其他状态 fallback。
- [ ] `Opt - Raw`、`POI - Raw` 姿态差可以直接查看。
- [ ] Surface Path、TCP Path、Surface Normal、Tool Axis 在 UI 中有明确区分。
- [ ] Rz 改变时表面法线可保持不变，但工具轴/姿态数值能够变化。
- [ ] Rx/Ry 改变时 TCP/Tool Axis 投影能够按状态变化。
- [ ] RAW/OPT/POI 3D 路径、点、法线线和工具轴都对应当前状态。
- [ ] 视野外或相机后方点不会产生错误投影线。

### 6.4 仿真和执行

- [ ] 2D 与 3D 使用同一 dense step 和同一 TCP pose。
- [ ] 多路径使用 path-local progress，不把全局进度直接用于当前路径。
- [ ] 仿真默认到终点停止，不自动回到起点。
- [ ] Simulate Single Path 能准确绑定 `path_id`。
- [ ] Execute on Robot Arm 经过连接、急停、限位、报告、TCP、坐标系、速度和人工确认检查。
- [ ] 任一 WARNING、FAILED、ERROR、缺失报告或快照不一致都会阻止真实执行。

---

## 7. 真实机械臂放行门

在所有 P0 和 P1 问题关闭、验收证据完成并通过人工复核前，OPT/POI 文件只能用于离线分析和仿真，不得下发真实机械臂。

最终放行必须同时满足：

1. 最终执行文件明确为 `scan.opt.path.yaml` 或 `scan.poi.path.yaml`。
2. 对应报告明确为 `scan.opt.report.json` 或 `scan.poi.report.json`，且由同一最终文件生成。
3. 报告状态为 PASS，没有不可达、分支跳变、关节超速、奇异性、碰撞或坐标系错误。
4. TCP 偏置、基坐标系、Euler 顺序、角度单位和速度单位已经实机单点确认。
5. 喷枪姿态与表面法线、standoff 和工艺要求一致。
6. 已完成空载低速单点、单段和整条路径的分阶段验证。
7. 急停、限位、通信断开和异常回退策略已经验证。
8. 最终执行前由人工确认文件、报告、状态、速度和路径摘要。

---

文档更新时间：2026-02-14  
更新者：AI 辅助代码核对  
统一结论：当前问题不是单独的 UI 刷新故障。根因是 RAW/OPT/POI 的路径快照、优化结果、Waypoint 对齐、2D/3D overlay 和报告轨迹没有使用统一数据契约；POI 工艺模型已明确为“绝对 Anchor 姿态 + 容差窗口”，这是为了降低深度相机法线噪声、复杂曲面法线突变和机械臂 IK/关节限制带来的不稳定风险。后续需要围绕该绝对姿态模型补齐配置记录、法线夹角监控、dense 轨迹验证和真实机械臂安全放行门。在统一数据链路、优化复核和安全放行门完成前，不建议将 OPT/POI 路径下发真实机械臂。
