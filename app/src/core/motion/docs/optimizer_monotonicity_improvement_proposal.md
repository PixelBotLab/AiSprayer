# 轨迹优化器单调性与高质量解空间算法改进方案

**文档归属**：`app/src/core/motion/docs/optimizer_monotonicity_improvement_proposal.md`  
**关联工件**：`data/template_group/2026-09-06_200125`  
**核心议题**：消除大容差包络下解质量变劣的反直觉现象，恢复超集单调性（Monotonicity Guarantee），进一步降低峰值关节速度与运动冲击。  
**评审补充**：§5–§9 为 2026-09-06 对照 `app/src/core/motion` 现有实现逐条核对后的评审与修订建议；§1–§4 原文保留，其中被推翻或需修正的表述见 §9 勘误清单。

---

## 1. 现象与深层根因分析

### 1.1 用户提出的核心命题
> **理论命题**：若容差包络 $\Omega_A \subseteq \Omega_B$（例如 $[10^\circ, 10^\circ, 50^\circ] \subset [30^\circ, 30^\circ, 180^\circ]$），优化算法的目标是在可行域中寻找最优解，则大容差包络下的解质量必然不劣于小容差包络（$J^*_B \le J^*_A$）。

### 1.2 为什么现有算法违反了该理论命题？

通过代码与数据排查，定位到当前系统的核心缺陷：

#### 缺陷 1：网格生成与容差截断脱节（Boundary Clamping Paradox，致命问题）
在 [optimizer.cpp:169-172](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L169-L172)：
```cpp
for (size_t i = 0; i < N; ++i) {
  R_cands[i] = R_nom * R_off[i];
  if (R_anchor) R_cands[i] = ProjectToAnchor(R_cands[i], *R_anchor, tol);
}
```
1. 偏移量 $R_{\text{off}}$ 仅依据 `grid_tol_z_deg: [-30, 30, 5]` 在名义法向周围生成；
2. 工件名义姿态与锚点 `[90, 0, 90]` 存在约 $+82.2^\circ$ 的初始自旋偏差；
3. 于是生成的候选自旋角原始分布为：$\text{rel\_z} \in [52.2^\circ, 112.2^\circ]$；
4. **当容差设为 50° 时**：
   - 所有在 $[52.2^\circ, 112.2^\circ]$ 的点都被 `ProjectToAnchor` 强行**截断在边界点 $\mathbf{50.0^\circ}$**；
   - 算法实际上得到了一组全部对齐在 $50.0^\circ$ 的姿态；
5. **当容差设为 180° 时**：
   - 因为 $180^\circ > 112.2^\circ$，**没有发生截断**；
   - 算法评估的点全部是 $[52.2^\circ, 57.2^\circ, \dots, 112.2^\circ]$；
   - **$\mathbf{50.0^\circ}$ 根本不在 180° 的候选集合里！**
6. **结论**：算法底层的候选集在 180° 时**并未包含** 50° 时的候选，实际搜索空间发生了“脱节和错位”。

#### 缺陷 2：分支内局部打分偏见（Heuristic Truncation Bias）
在 [optimizer.cpp:43-46](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L43-L46)：
- `PoseScore` 仅依据“单点逆解离机械臂零位的远近”评分；（**表述有误**，见 §9 勘误 E2/E3：实际是「候选姿态相对该航点名义法向姿态的偏离」，与 IK 解无关；分支配额实际为 16 不是 4）
- 每个分支只保留前 4 个候选（`max_candidates_per_branch = 4`）；
- 大容差引入了更多大偏移但碰巧零位距离小的姿态，把真正能平滑连接前后航点的姿态挤出了候选池。

#### 缺陷 3：$L_2$ 总能量与 $L_\infty$ 峰值速度的目标失配
- DP 优化的是 $\sum (\Delta q)^2$（积分能耗）；
- 运动学校验和用户感知的是 $\max |\dot{q}|$（最大瞬时冲击）。

---

## 2. 算法改进方案设计

针对上述根本缺陷，提出三阶递进的算法升级架构：

```
[用户设定容差 tol]
       │
       ▼
【改进 1：超集完备采样生成器】 (Superset-Preserving Candidate Generator)
  ├── 严格保证: Cand(tol_large) ⊇ Cand(tol_small)
  ├── 包含: 锚点姿态、名义姿态、容差网格点、包络边界极值点
       │
       ▼
【改进 2：多样性保全评分与分箱】 (Diversity-Preserving Branch Filtering)
  ├── 不再按单点零位绝对值一刀切
  ├── 沿姿态自旋轴分箱均匀抽样 (Stratified Sampling)
       │
       ▼
【改进 3：DP 束搜索 (全局拓扑定型)】
  ├── 确定无奇异、无分支跳变的关节构型拓扑
       │
       ▼
【改进 4：连续二次规划平滑 (QP Post-Smoothing)】
  ├── 在锁定构型下，对全路径姿态在容差盒内做连续梯度微调
  ├── 目标函数引入 Minimax 惩罚: 压降最大峰值速度
       │
       ▼
[输出: 兼具全局单调性与超低峰值速度的完美轨迹]
```

---

## 3. 具体模块技术细节

### 3.1 改进 1：超集完备候选生成器 (Superset Candidate Generator)
修改 `optimizer.cpp` 中的姿态采样逻辑：
1. **取消孤立的 `grid_tol` 静态配置**，采样网格的上下界直接取自当前生效的 `poi_tolerance_rpy_deg`：
   $$\text{grid\_z} = [-\text{tol}_z, +\text{tol}_z, \Delta z]$$
2. **基准点融合采样**：
   - 候选集 $C$ 显式由三部分联合构成：
     $$C = C_{\text{nominal\_perturb}} \cup C_{\text{anchor\_envelope}} \cup \{R_{\text{anchor}}, R_{\text{nominal}}\}$$
   - 无论容差如何放大，小容差覆盖的网格点始终是当前采样的严格子集；
   - 这样在数学上严格满足：
     $$\text{tol}_1 \le \text{tol}_2 \implies \text{Cand}(\text{tol}_1) \subseteq \text{Cand}(\text{tol}_2)$$

### 3.2 改进 2：分层分箱采样（Stratified Pose Filtering）
替代 `optimizer.cpp:258` 中简单的单打分截断：
- 将自旋角按 $[-\text{tol}_z, +\text{tol}_z]$ 等分为 $K$ 个区间；
- 每个区间各取 1 个在当前逆解分支中合法且连续性最好的姿态；
- 避免同质化姿态占据全部候选配额，确保手腕旋转方向覆盖均匀。

### 3.3 改进 3：连续后平滑优化器 (Continuous Trajectory Polishing)
在 DP 给出一条合格（PASS）的离散逆解路径 $q_1, \dots, q_M$ 之后：
- 建立沿轨迹的有约束优化问题：
  $$\min_{\{q_k\}} \sum_{k=1}^{M-1} \|q_{k+1} - q_k\|_W^2 + \lambda \sum_{k=1}^{M-2} \|q_{k+2} - 2q_{k+1} + q_k\|_A^2$$
  $$\text{s.t.} \quad |\text{Euler}(\text{FK}(q_k) \cdot R_{\text{anc}}^T)| \le \text{tol}$$
- 这一步可以在连续空间中把任何离散跳跃抹平，将峰值关节速度压低到物理极限。

---

## 4. 实施阶段与测试验证

| 阶段 | 内容 | 预期交付物 |
| :--- | :--- | :--- |
| **Phase 1 (已完成)** | 理论推导与根因定位 | 本文档与方案设计 |
| **Phase 2 (测试验证)** | 编写离线 Python 测试脚本验证超集采样原型 | `scratch/test_superset_optimizer.py`，验证大容差结果严格 $\le$ 小容差 |
| **Phase 3 (核心工程)** | 待用户确认后重构 `optimizer.cpp` / `segment.cpp` | C++ 模块升级与回归校验 |

> ⚠️ 本节阶段划分已被 §7 重排替代（原因见 §5.3：改进 1 单独落地有回退风险，必须先补速度约束与一致性）。

---

## 5. 对照现有实现的核对与评审（2026-09-06）

### 5.1 事实核对表

| 文档论断 | 核对结论 | 代码 / 数据依据 |
| :--- | :--- | :--- |
| 缺陷 1：采样网格与包络截断脱节 | **成立**，且可用既有实验数据复现 | [optimizer.cpp:169-172](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L169-L172) 生成 + [:102-115](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L102-L115) 逐分量钳位；`grid_tol_z_deg=[-30,30,5]` 以名义姿态为相位中心 → 候选自旋恒为 $\text{rel\_nom}_z \pm 30$。$\text{rel\_nom}_z=82.2^\circ$ 时，$\text{tol}_z=60$ 的解（自旋恰为 60.0）在 $\text{tol}_z=85$ 的候选集中确实不存在（格点为 57.2 / 62.2）。对应实测劣化：`orientation_optimization_analysis.md` §4.1 表，45°→46.1°/s、60°→44.1°/s、**85°→81.6°/s** |
| 缺陷 2：`PoseScore` 按“逆解离机械臂零位远近”打分 | **不成立（误读）** | [optimizer.cpp:184-188](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L184-L188)：`zero[i] = ‖Euler(R_nomᵀ R_cand)‖² · weight_zero_dev`，是**姿态相对该航点名义法向的偏离**，与机器人零位无关（变量名 `zero_dev` 有误导性）；且 `PoseScore` 只按 `pose_idx` 取值 → **同一姿态的 8 个 IK 分支分数完全相同** |
| 缺陷 2：`max_candidates_per_branch = 4` | **数值过期** | 实际默认 16（[report.hpp:67](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/include/motion/report.hpp#L62-L77)、`cli_main.cpp:75`）；Python 服务层不转发该参数 |
| 缺陷 3：$L_2$ 目标与 $L_\infty$ 峰值失配 | **方向对，诊断不完整** | $L_\infty$ 其实已有两处硬约束：[segment.cpp:199-206](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/segment.cpp#L199-L206) 每采样步 $\max\|\Delta q\|\le$ 45°、[optimizer.cpp:305-307](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L305-L307) 航点间 120~170°。但按 `movel_spacing_mm=5` / 150 mm/s 折算，45°/步 = **1350°/s**，比物理限速 179.91°/s 宽 7.5 倍 → 它们是**分支连续性**约束，不是速度约束。准确表述应为：“当前优化器里根本不存在任何速度层面的约束” |
| 改进 1 “取消 grid_tol、上下界取自 `poi_tolerance_rpy_deg`” | **只对 Z 轴可行，X/Y 不可照搬** | 见 §6.1：X/Y 的 ±5° 是工艺约束（喷嘴对中表面法向），扩到包络的 ±30° 会让候选姿态从 468/航点 涨到 ~70k/航点（×150），候选生成 0.11 s→~17 s，并引爆 [:498-512](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L498-L512) 的全量回退 |

### 5.2 方案遗漏的根因（优先级不低于缺陷 1）

- **R4｜DP 内校验与终校分辨率不一致**：`Walk` 沿 MoveL 按 `movel_spacing_mm=5.0` 采样（[optimizer.cpp:309-311](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L309-L311)，`n_mid` 夹在 [10,100]），而终校 `ChainVerifier` 按 `step_mm=1.5~2.0` 密集采样。DP 判“可行”的边，在 2.5~3.3× 更细的采样下可能恰好撞上奇异邻域分支翻转 → 直接表现为 FAILED + 万级°/s。这是“优化器认为自己成功、校验器判 FAIL”的结构性来源。
- **R5｜jump（列间过渡）边完全没有约束**：[optimizer.cpp:275-289](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L275-L289) 对 `is_jump ‖ !spraying` 的边提前 return：既不做 `ew_family` 锁定、不做 `Walk`、也没有任何 $\Delta q$ 上限，只有 $\Sigma w\Delta q^2$ 代价。而校验器对 jump 步仍按 **MoveL + 喷涂线速度** 计算 `dt` 并判 OVERSPEED（[verifier.cpp:160-201](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/verifier.cpp#L160-L201)，注意 `KINEMATIC_DISCONTINUITY` 排除了 jump，`JOINT_OVERSPEED` 没有）。本工件 83 航点中 11 个 `is_jump: true`；一个 290° 的腕部翻转换算成 $290^\circ / 6.7\,\text{ms} \approx 4.3\times10^4\,^\circ/\text{s}$ —— **报告的极端峰值相当比例是过渡段建模伪影，不是喷涂段真实质量**。不剔除这个噪声源，任何“峰值单调”实验结论都不可信。
- **R6｜自旋在代价函数里近乎免费**：`weight_zero_dev = (1, 1, 0.01)`（Z 即自旋被降权 100×）叠加 `joint_weights = (1, 1.2, 1, 0.8, 0.8, 0.5)`（J4/J6 最便宜），而六个关节的物理限速完全相同（179.91°/s）。一旦容差放开自旋，DP 必然用 J4/J6 去换 $\Sigma\Delta q^2$，峰值就出在 J4/J6（Case 5c: J4/J6=81.6°/s；Case 6c: J6=142.9°/s）。→ 这才是“大容差 → 峰值变差”的主动机；**钳位退化反而一直在替我们掩盖它**。
- **R7｜欧拉逐分量钳位的万向节风险**：`ProjectToAnchor` 裁剪的是 $R_{anc}^\top R_{cand}$ 的 xyz 欧拉分量。只有当该相对旋转的 $|r_y|\ll 90^\circ$ 时，“裁 $r_z$”才等价于“限绕枪轴自旋”。本工件恰好 $\text{rel\_nom}\approx(0,0,82.2)$ 所以成立；换锚点/换工件（例如名义 rpy 含 $r_y\approx -81.6$）时裁剪语义会退化，自旋与俯仰互相串扰。建议改为 **自旋 + 锥角（axis-cone）分解**：先解出绕枪轴的自旋角与轴向倾角，分别限幅。
- **R8｜格点相位不含 0**：`ExpandAxisGrid({-5,5,2})` 生成 `{-5,-3,-1,1,3,5}`（[conventions.hpp:151-158](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/include/motion/conventions.hpp#L143-L158)）——**名义姿态本身永远不是候选**，最小倾角偏置 1°。Z 轴因 ±30/step5 恰好含 0 而侥幸无碍。这正是 §6.1 强调“格点必须锚定在 0 相位”的现证。

### 5.3 总体判断

1. **缺陷 1 是真问题，改进 1 的方向正确**，但它只能保证 **DP 目标 $J$** 的单调性：命题 $J^*_B\le J^*_A$ 里的 $J$ 是 $\Sigma\Delta q^2+\Sigma$姿态偏置，**不含峰值角速度**。因此只落地改进 1 无法保证现场关心的“解质量”单调。
2. **更关键的风险：改进 1 单独实施很可能造成回退。** 现在 tol=45/60 之所以漂亮（44~46°/s），很大程度上是钳位把所有候选压成**同一个自旋值**的退化结果（DP 没在选，是没得选）；一旦按包络原生展开自旋，R6（自旋免费）+ R5（jump 无界）会立刻把峰值顶上去。→ **实施顺序必须倒过来**（§7）。
3. 改进 4（QP 后平滑）在缺少速度约束的情况下是“用重型非凸优化去补一个本可以用硬约束堵住的洞”，建议降级为可选末项，先做 §6.4 的 $\tau$-二分。
4. 方案缺一条**真正能给硬保证、且零算法风险**的手段：精英保留 / best-of（§6.5）。离散的候选集嵌套只能做到 $\varepsilon$-嵌套，而“拿已知的最好解兜底”是精确的。

---

## 6. 逐项修订建议

### 6.1 改进 1（保留，改写为“包络原生自旋格”）

只把 **Z（自旋）** 的采样绑定到包络，X/Y 保持名义系的工艺小格：

- 固定步长 $\Delta z$（沿用 `grid_tol_z_deg.step = 5°`，**不随容差变**），格点锚定在**锚点帧 0 相位**：$\mathcal{K}(\text{tol}) = \{k\in\mathbb{Z}: |k\Delta z| \le \text{tol}_z\}$；
- 每航点算一次 $\text{rel\_nom} = \text{EulerWrap}(R_{anc}^\top R_{nom})$；
- 自旋候选（转回名义工具帧施加）：$\sigma_k = k\Delta z - \text{rel\_nom}_z$；
- 候选姿态：$R_{cand} = R_{nom}\,R_y(\delta_y)R_x(\delta_x)R_z(\sigma_k)$，其中 $\delta_x,\delta_y$ 仍取 ±5° 小格；
- 追加**保真点** $\sigma^* = \text{clamp}(\text{rel\_nom}_z, \pm\text{tol}_z) - \text{rel\_nom}_z$（容差 ≥ $|\text{rel\_nom}_z|$ 时即名义姿态本身，替代现在的“钳位到边界”）。

**性质**：格点部分对 tol **严格嵌套** $\Rightarrow$ $\text{Cand}(\text{tol}_1)\subseteq\text{Cand}(\text{tol}_2)$ 精确成立；只有 $\sigma^*$ 是 tol 相关的额外点，其最坏影响是把上一轮最优自旋替换成 $\Delta z$ 内的相邻格点，$\varepsilon=2.5^\circ$，在 $w_z=0.01$ 下对 $J$ 的影响 $\le 0.06$ 量级，远小于一段边代价（数百）→ **工程上可视为严格，但文档不要宣称数学严格**（真正的硬保证靠 §6.5）。

**规模与预算**：tol_z=180 → z 点数 73，位姿/航点 $6\times6\times73 = 2628$（现 468 的 5.6×），候选生成 0.11 s→~0.6 s，`packs` 常驻内存 ~50 MB，可接受。必须同时加两道保险：① 位姿总数硬上限（超限则自动放大 $\Delta z$ 并打印告警）；② 把 [:498-512](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L498-L512) 的 `Materialize` 全量回退改为**有界扩容**（如 ×4），否则大网格下层失败会导致每层边数爆炸。若按原方案把 X/Y 也放大到 ±30：$31\times31\times73\approx70\text{k}$ 位姿/航点、$5.7\times10^6$ 位姿、`full_nodes` ~$4.5\times10^7$（仅 `q_sols` 缓冲即 ~2 GB）→ **禁止**。

### 6.2 改进 2（目标对，落点错）

截断发生在 8 个 IK 分支桶里，而 `PoseScore` 与 IK 解无关（§5.1）→ 8 个桶装的是**同一批最贴近名义的 16 个姿态**的重复，分箱配额浪费在分支维度上，真正缺的是**自旋维的姿态多样性**。修订：

- 若采纳 §6.1，自旋格点本身已经均匀分箱，**改进 2 大部分自动达成**，只需把截断从“分支桶内 top-K”改为“**pose 层去冗后再按分支展开**”；
- 截断分数加入连续性先验：$\text{score} = \text{pose\_dev} + \lambda_{ctx}\,d_{geo}(R_{cand}, R_{prev\_layer\_best})$，使预选不再只看单点保真（当前 [:43-46](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L43-L46) 完全没有任何邻接信息）；
- 必须把 `max_candidates_per_branch`(16) 与 `beam_width`(32) 当作**受控变量**：现每层 113 候选 → beam 只留 32，**每层都在剪枝（存活率 28%）**，这是与候选集嵌套性独立的第二个单调性破坏源（种群相关的 top-K 剪枝会随候选总数改变幸存集合）。做 T2 级实验时必须同时把它们调到 $\infty$。

### 6.3 改进 3（图中“DP 束搜索·全局拓扑定型”）

**现有实现已完成，不是新增工作量**：`ew_family` 跨层锁定（[:293](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L291-L293)）+ `BeamKeep`。§2 图列 4 项而 §3 只有 3 小节，编号错位（§3.3 实为图中改进 4），需重排。

### 6.4 改进 4（QP 后平滑）→ 降级为可选，先做 $\tau$-二分

QP 的四条现实障碍：

1. **非凸**：FK 为三角函数约束，还需显式奇异度约束（$\sigma_{min}(J)\ge\varepsilon$，不光滑），本质是 SQP/IPOPT 问题；
2. **依赖成本**：`motion_core` 目前只链 Eigen / yaml-cpp / tinyxml2（[CMakeLists.txt](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/CMakeLists.txt)），并强制 `-fno-fast-math` 保证黄金回归位级一致；引入 NLP 求解器的 RK3588/x86 交叉构建与镜像体积代价高；
3. **推平 DP 的已有保证**：平滑改变了中间位姿，DP 逐边验过的 MoveL 连续/无跳变/奇异结论全部作废，必须整段重校 + 失败回滚；
4. **收益不确定**：Case 5a/b 已到 44~46°/s（极限的 25%），边际收益小于 R4/R5/R6 三项一致性修正。

**替代方案（推荐先做）：峰值二分 $\tau$**。把 `MoveLQuery::max_jump_rad` 从固定 45° 改为参数 $\tau$，按 $\tau$ 二分：“存在 PASS 链则继续减，否则加”。一次 DP 仅 1.4 s，5~6 次 ~8 s 即可拿到**最小可达峰值**；因为它是可行性约束而非目标项，**天然满足容差单调**（包络越大 $\tau^*$ 只会越小），一并解决了“$L_2$ 不管 $L_\infty$”的问题。$\tau$ 换算必须按 $dt$ 走（$\tau_j = \kappa \cdot$ `max_vel_deg_s[j]` $\cdot$ spacing/speed），并在文档里注明与终校采样（1.5~2 mm）不一致时依赖 R4 先对齐。

若仍要连续平滑，**先在 Python 侧做位姿层原型**（`scipy` 已在 [requirements.txt](file:///home/zhanlu/robots/AiSprayer/app/requirements.txt)）：航点位置不动、只优化每航点自旋 $\sigma_k$（变量数 83），用 [kinematics.py](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/kinematics.py) 的 ctypes FK/IK 求 $q(\sigma)$，目标 $\min \max_j|\Delta q_j|$ 的平滑代理。几秒可出结果，验证收益后再决定要不要进 C++。

### 6.5 新增改进 5：精英保留 / best-of（唯一能给硬单调保证且零算法风险）

**它能回答的问题：“既然 $[30,30,180]\supset[10,10,50]$，为何大容差找不到不劣于小容差的解？”**

理论上 $J^*_B\le J^*_A$ 成立，但要落到**实际返回的结果**上需三个条件同时成立，当前三条全破：

| 条件 | 现状 | 归因 |
| :--- | :--- | :--- |
| (a) 大容差的**候选集**包含小容差的最优解 | ❌ $\text{tol}=[10,10,50]$ 时候选被钳位到自旋恒 $50.0^\circ$；$\text{tol}=[30,30,180]$ 时候选自旋为 $[52.2,112.2]$ — **50.0 根本不在集内** | 缺陷 1（§6.1 修） |
| (b) 搜索过程与候选数量无关 | ❌ 每层 113 候选、`beam_width=32` 每层都在剪（存活率 28%）；top-K 剪枝天然“种群相关”，候选越多幸存集越不同 | §6.2 末条（只能靠穷举实验隔离，无法靠采样修好） |
| (c) $J$ 就是人追求的“质量” | ❌ $J=\Sigma\Delta q^2+$姿态偏置，**不含峰值角速度**；即使 (a)(b) 全修好、DP 做到穷举最优，峰值仍可能变大（5b: 44.1 vs 5c: 81.6 就是这个形状） | 缺陷 3 + R6（§6.4/§6.6 修） |

**为什么只有 best-of 能给硬保证**：包络只是“允许选哪些姿态”的约束，不改变“这条轨迹能不能跑”的判定——$[10,10,50]$ 的解姿态必然仍在 $[30,30,180]$ 包络内，密集校验结果完全相同、照样 PASS。即**小容差的合法解集是大容差合法解集的子集，且评价函数（校验器）与容差无关**。因此“取两轮择优”不是近似，是严格的：大容差结果 $\equiv\min(\text{小容差解},\ \text{大容差新解})$，永不回退。而改进 1 的离散格点本质上只能做到 $\varepsilon$-嵌套（边界点 $\pm\text{tol}$ 不在固定格点上），给不了这种精确性。

**实施要点**：

- **服务层**（[path_verification_service.optimize_template_paths](file:///home/zhanlu/robots/AiSprayer/app/src/apps/interactive/path_verification_service.py)）：按字典序 (status → 峰值 → 指向偏量 → $J$) 只接受不劣于上一轮 PASS 结果；用户放大容差时自动持“上次最优”为兜底；
- **C++ 侧（更精确）**：`OptimizeResult` 增加 `objective` 字段，并支持“注入上一轮解姿态作为额外候选”（每层只多 1 pose / ≤8 node）→ $J$ 严格不劣；
- 因为单次优化仅 1.4 s，最便宜的实现是 **容差阶梯 + 择优**：`tol_z ∈ {30,45,60,90,180}` 依次跑并取最优，单调性由枚举保证，无需改优化器本体。**这已经直接满足上表 (a)(b)(c) 全部三条**（枚举代替搜索包含关系），所以它是 P1 而非 P2；建议先拿它验证改进 1 的价值，再决定是否重构采样器。
- 必须事先固定择优标尺（建议字典序：`校验 status` → `峰值角速度` → `最大指向偏量` → `J`），否则“不比…差”无定义；且需区分“没找到更好的”与“搜索挂了”，所以依赖 §7 P0 埋点把 $J$/包络占用/beam 存活率打出来；回显时要标明最终采纳解用的是哪一档包络，避免用户误读为“放宽无效”。

### 6.6 新增改进 6：一致性修正（低成本高收益，先于改进 1）

1. `movel_spacing_mm` 与终校 `step_mm` 对齐（5 → 1.5~2，或直接取 `step_mm`）；代价：每层边耗时近似线性增长，需配合 beam 控量；
2. jump 边加 $\Delta q$ 上限（或让校验器对 `is_jump` 改用 MoveJ 时间模型 $dt = \max_j \Delta q_j / \dot q^{max}_j$），消除 4 万°/s 级伪影；
3. 重标 `weight_zero_dev` / `joint_weights`：六轴限速相同，当前权重使自旋成为免费自由度（R6）；建议按“各轴可用速度预算”归一；
4. `ExpandAxisGrid` 改为以 0 为相位锚点的对称格点（R8），至少保证 $\delta_x=\delta_y=0$（名义姿态）始终在候选集内。

---

## 7. 重排后的实施路线（替代 §4）

| 阶段 | 内容 | 预期收益 / 风险 | 验收标准 |
| :--- | :--- | :--- | :--- |
| **P0 诊断埋点** | `OptimizeResult` 输出 $J$；报表输出每航点 $\text{rel}$ 欧拉占用、beam 存活率、per-branch 截断丢弃数 | 无行为改变；**没有它后续所有实验不可证伪** | 同一输入重跑位级一致 |
| **P1 一致性与守卫** | §6.6 全部 + §6.5 best-of | 峰值伪影消失、失败率下降；不动采样器 | tol ladder 全 PASS，峰值不高于现基线 |
| **P2 采样修正** | §6.1 包络原生自旋格 + §6.2 截断改造 | 真正恢复 $J$ 单调；**有回退风险，必须在 P1 之后** | T1 嵌套单测 + T2 $J$ 单调 |
| **P3 峰值最小化** | §6.4 $\tau$-二分（替代 QP） | 直接优化用户关心的指标 | 峰值较 P1 基线再降 $\ge$ 20% |
| **P4 连续平滑** | 先 Python/scipy 原型，再评估是否进 C++ | 抹平离散跳跃 | 仅当 P3 仍有不可消除的局部峰时才启动 |

---

## 8. 测试与验收方案修订

原文 Phase 2 “验证大容差结果严格 $\le$ 小容差”不能作为单一断言：峰值不是 $J$，不先修 R5 会测到建模噪声。拆为三层：

- **T1（纯函数，无机器人模型）**：对改进 1 的新采样器直接断言集合嵌套：$\forall\,\text{tol}_1\le\text{tol}_2$，格点部分 $\text{Cand}(\text{tol}_1)\subseteq\text{Cand}(\text{tol}_2)$（四元数键去重后，容差 1e-9）；同时用反例固定住旧行为（$\text{tol}_z=60$ 的最优自旋不在 $\text{tol}_z=85$ 集内）。
- **T2（搜索层）**：取同一路径前 8~12 航点，`beam_width`/`max_candidates_per_branch` 设为 $\infty$（穷举），断言 $J(\text{tol})$ 单调不增。这是验证“命题成立”的唯一干净场景。
- **T3（端到端）**：模板 `2026-09-06_200125`，锚点 `[90,0,90]` 固定，扫 tol_z ∈ {30,45,60,85,90,180}，记录 status / 峰值 / 指向偏量 / $J$。基线沿用 `orientation_optimization_analysis.md` §3（45→46.1、60→44.1、85→81.6）。**验收：单调性只在 $J$ 与 status 上严格要求；峰值作为 P3 的目标单独要求**。

工程落地注意：

1. **实验入口**：Python 服务层不转发 `--grid-*` / `--beam-width` / `--max-candidates-per-branch`（只有 yaml 里的 `grid_tol_*` 生效），实验必须直连 `motion_cli`（`app/src/core/motion/bin/motion_cli`）。`scratch/test_poi_anchor.py` 仍在传已删除的 `mode=` 参数，不能复用。
2. **黄金回归成本**：`tests/test_golden_optimize.cpp` 以 0.05° 位姿一致对比已存盘 poi（固定 `tol=[10,10,30]`），改进 1/2 必然打破它。需同步重生黄金件，并把断言改为“性质断言 + 松容差”，否则测试只锁住了旧缺陷。
3. **回归资产位置**：`test_superset_*` 应落在 `app/src/core/motion/tests/`（`scratch/` 不是长期资产）。
4. **跨平台**：x86 与 RK3588 均需重跑 ladder。即使 `-fno-fast-math`，候选顺序变化仍会经 `stable_sort` 平局传导（[optimizer.cpp:176-178](file:///home/zhanlu/robots/AiSprayer/app/src/core/motion/src/optimizer.cpp#L176-L178) 已明确该风险），需保证新采样器输出顺序确定性。

---

## 9. 配置语义冲突与勘误

**配置现状**（[configs/aisprayer_config.yaml:88-97](file:///home/zhanlu/robots/AiSprayer/configs/aisprayer_config.yaml#L88-L97)）：`poi_tolerance_rpy_deg: [30,30,180]` 与 `grid_tol_x/y: ±5` / `grid_tol_z: ±30` 并存——注释自写的“网格范围应不小于包络”已被违反：Rx/Ry 的 ±30 在 ±5 网格下永不生效（惰性约束），Rz 的 ±180 实际只能达到 $\text{rel\_nom}_z\pm30$。建议二选一：按 §6.1 让 Rz 采样随包络展开；或把两个概念在配置/UI 层面正式改名（“工艺包络” vs “搜索网格”），并标注“Rz 包络 > 网格半径 + rel_nom 时无效”。

**勘误清单**

| 编号 | 原文位置 | 修正 |
| :--- | :--- | :--- |
| E1 | §1.2 缺陷 1 步骤 3 | $[52.2^\circ,112.2^\circ]$ 仅在 $\text{rel\_nom}_z=82.2^\circ$（本工件 + 锚点 [90,0,90]）时成立，宜写成一般式 $\text{rel\_nom}_z \pm 30^\circ$ |
| E2 | §1.2 缺陷 2 | “单点逆解离**机械臂零位**的远近” → “候选姿态离**该航点名义法向姿态**的远近”；且不依赖 IK 解，同姿态各分支同分 |
| E3 | §1.2 缺陷 2 | `max_candidates_per_branch = 4` → 16（默认值，CLI `--max-candidates-per-branch` 可改） |
| E4 | §2 与 §3 | 图列 4 项、正文 3 小节，§3.3 对应图中“改进 4”；图中“改进 3”已有实现，应标为现状而非待做 |
| E5 | §1.2 缺陷 3 | “$L_2$ 与 $L_\infty$ 目标失配” → “优化器内不存在速度层约束（现有 45°/步 ≈ 1350°/s，属分支连续性约束）”，并补 R5（jump 边无约束）与 R6（自旋免费） |
| E6 | §4 Phase 1 | 标“已完成”但无 §8 的 T1/T2 证据；建议改标“根因分析完成（代码核对见 §5）” |
| E7 | §3 标题 | “修改 `optimizer.cpp` 中的姿态采样逻辑”应同时列出受牵连项：`ProjectToAnchor` 语义（R7）、`PoseScore`（E2）、`Materialize` 回退上限（§6.1）、黄金件重生（§8.2） |
