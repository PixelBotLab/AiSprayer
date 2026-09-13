# 3D 机械臂仿真喷雾粒子发射与工件表面碰撞着色设计方案 (3D Spray Particle Simulation & Surface Staining Design)

**文档状态**：详细设计完成 · 已完成设计评审（评审结论见 §0，正文已按结论就地修正）  
**文档路径**：`app/docs/3d_spray_simulation_design.md`  
**涉及模块**：
- 前端 3D 视口：`app/frontend/src/components/Robot3DViewer.tsx`
- 3D 控制面板：`app/frontend/src/components/RobotZone.tsx`
- 交互与仿真状态机：`app/frontend/src/components/operations/InteractiveOp.tsx`
- 生产配置文件：`configs/aisprayer_config.yaml`（`spraying` & `hardware.robot`）

---

## 0. 设计评审结论 (Design Review Findings)

> 本节对照现有前端实现（`Robot3DViewer.tsx`、`InteractiveOp.tsx`、`RobotZone.tsx`）复核本设计后的结论。正文相关小节已就地修正，实现前请先读本节。

### 0.1 定位修正：可视化预览 ≠ 膜厚定量验收
本设计**适合作为「喷涂覆盖 / 重叠 / 漏喷」的实时可视化预览工具**，选型（顶点色、视觉与染色解耦、世界→局部坐标转换、mm→m 口径）均与现有实现兼容。但 §1.2 / §3.3 / §5.2 中「膜厚工艺验证 / Compliance 定量指标」的定位**当前实现条件支撑不了定量结论**，须降级为「标定后方可作数」，原因见 §0.2-R3 与 §3.3 说明：离线回放走的是固定名义步速（与 `velocity` 无关），在线真机走的是低频 WebSocket 采样，二者都不是真实物理停留时间。

### 0.2 必读修正清单（R = Required，实现前必须处理）
- **R1 顶点色渲染前置（阻断级）**：当前工件材质 `MeshStandardMaterial`（`Robot3DViewer.tsx`）**未开启 `vertexColors: true`** 且 `color: #ffffff` 会覆盖，只往 `attributes.color` 写数据不会显示。必须切材质开关，详见 §4.3.1。
- **R2 仿真触发字段与真实状态不符（阻断级）**：本设计原用的 `simulationState.currentStepSpraying` / `stage === 'spraying'` **在实际 `SimulationState` 中不存在**；`simSteps` 每项当前只有 `{ q_deg, tcp, pixel, pathIdx }`，**没有 spraying 标志**。触发逻辑已按真实字段更正，并列出需先落地的数据打通，详见 §4.4。
- **R3 离线膜厚非物理标定（准确性）**：回放是固定 ~60 步/秒名义速率 + 段间 10 子步插值，`Avg Film`/`Compliance` 在离线模式仅为定性，需按真实 MoveL 时间映射 dt 后才定量，详见 §3.3。
- **R4 在线真机为空间失准示意（准确性）**：通道 1 靠低频轮询 `digital_outputs` 与位姿驱动，喷涂是空间触发、视觉是时间采样，必失同步；在线仅作实时示意，工艺判断以离线预演为准，详见 §4.4.1。
- **R5 Zero-GC 自洽性**：`applySprayStep` 原 `matrixWorld.clone().invert()` 每帧分配，违反本设计 §6.1，已改为预分配 scratch 矩阵。
- **R6 性能乐观（风险项）**：全顶点 O(N) 遍历 + 每帧全量 color buffer 重传是 RK3588 Mali 的主要瓶颈，30Hz 节流不改渐近；需 update-range / 顶点分桶，详见 §6.1。
- **R7 覆盖率分母偏差**：`coveredVertices / totalVertices` 含背面/不可喷面且顶点密度不均，应改面积加权，详见 §5.2。
- **R8 已知物理局限**：圆形圆锥近似（真实为扁平扇形光斑）、`cosAlpha` 仅挡朝向不挡自遮挡，薄壁反面仍有过染风险，详见 §6.3。
- **R9 配置落位**：纯前端可视化参数不应污染驱动真机的 `configs/aisprayer_config.yaml`，详见 §8。

---

## 1. 背景与业务目标 (Background & Objectives)

### 1.1 现状与痛点
在当前的 AiSprayer 生产系统中：
1. **3D 视口展示现状**：`Robot3DViewer` 已能够实时渲染 Dobot CR5 机械臂 URDF、工件表面扫描重建网格（`scan.mesh.ply` / `scan.mesh.stl`）以及 3D TCP 空间轨迹线。
2. **工艺反馈缺失**：
   - 现有的仿真和真机轨迹追踪仅能看到一条蓝色的 TCP 移动轨迹线（`tcp_trace_line`），操作人员无法直观判断喷枪打开（`DO === 1`）时，漆雾是否真正喷射到了工件上；
   - 无法在执行前预测**喷幅重叠率（Overlap）是否合理**，是否存在**漏喷（Under-spray / Uncovered area）**或**过度重叠流挂（Over-spray / Excessive accumulation）**；
   - 缺少数字孪生工业级交付物中的“涂装仿真与膜厚工艺验证”能力。

### 1.2 核心目标
1. **真实物理粒子喷雾表现 (Visual Particle Mist)**：
   - 当喷涂电磁阀开启（实机 DO 触发或离线仿真播放到 `spraying: 'on'` 航段）时，从喷枪 TCP 末端沿喷射法向连续发射具有锥形/扇形发散、渐变衰减的雾化漆雾粒子。
2. **工件表面实时碰撞着色 (Dynamic Surface Staining)**：
   - 漆雾射流与扫描重建的工件网格（`reconstructed_surface_mesh`）相交时，网格表面瞬间被着色，呈现具有高斯边缘衰减的逼真喷斑。
3. **漆膜厚度累加与工艺热力图 (Film Thickness Accumulation & Heatmap)**：
   - 支持多道轨迹往复扫描的膜厚叠加计算；
   - 提供**逼真涂层（Realistic Coating）**与**膜厚热力图（Thickness Heatmap）**两种视图切换，实时计算并显示工件**表面覆盖率（Coverage %）**与**平均膜厚（Avg Thickness）**。
4. **极致性能与边缘端兼容 (60 FPS on RK3588 & Low-End WebGL)**：
   - 采用“视效与物理着色解耦”架构，零 CPU 逐粒子碰撞检测，零垃圾回收（Zero-GC）内存抖动，保证在 RK3588 嵌入式平台及各类浏览器中稳定维持 60 FPS。
5. **符合全英文 UI 铁律**：所有前端新增交互控件、指标标签、HUD 胶囊完全采用专业英文。

---

## 2. 核心架构与解耦原则 (Architecture & Decoupling Principle)

### 2.1 视效与物理算法解耦（Visual & Physical Decoupling）
如果对成千上万颗独立粒子在 JavaScript 主线程中逐帧做三角网格射线碰撞检测（`Raycaster.intersectObject`），在几万至几十万面片的工件网格上会导致严重的 CPU 算力雪崩和帧率断崖。

因此，本设计严格采用**工业图形仿真标准的双层解耦架构**：

```
                    ┌────────────────────────────────────────────────────────┐
                    │      喷涂触发源 (Trigger Source)                        │
                    │  - 真机执行: robotState.digital_outputs[DO_INDEX-1] == 1  │
                    │  - 离线仿真: simulationState.stage == 'spraying'        │
                    └──────────────────────────┬─────────────────────────────┘
                                               │
                                               ▼
                    ┌────────────────────────────────────────────────────────┐
                    │          TCP 姿态提取与工艺参数 (TCP State & Config)     │
                    │  - 喷嘴世界坐标 P_tcp, 喷射轴向向量 N_tcp                │
                    │  - 靶距 spray_dist_mm, 喷幅 spray_width_mm             │
                    └──────────────────────────┬─────────────────────────────┘
                                               │
                      ┌────────────────────────┴────────────────────────┐
                      ▼                                                 ▼
      ┌───────────────────────────────┐               ┌──────────────────────────────────┐
      │  视觉层: GPU 雾化粒子系统      │               │  表面层: 锥体投影与顶点颜色引擎  │
      │  (SprayParticleEmitter)       │               │  (SurfaceStainingEngine)         │
      ├───────────────────────────────┤               ├──────────────────────────────────┤
      │ • THREE.Points / InstancedMesh│               │ • 顶点颜色属性 attributes.color  │
      │ • 200~400 个复用粒子对象池    │               │ • 空间距离粗筛 (Bounding Sphere)  │
      │ • 沿轴向喷射 + 径向高斯散射   │               │ • 圆锥投影角度与高斯衰减计算     │
      │ • 纯视觉特效，不参与碰撞计算  │               │ • 膜厚浮点缓冲 thicknessBuffer   │
      │ • 极其轻量，GPU 直接渲染      │               │ • needsUpdate 增量标记           │
      └───────────────────────────────┘               └──────────────────────────────────┘
                      │                                                 │
                      └────────────────────────┬────────────────────────┘
                                               ▼
                              ┌──────────────────────────────────┐
                              │  3D 视口最终合成 (Robot3DViewer) │
                              │  - 逼真流光喷雾                 │
                              │  - 工件表面喷斑实时渐变          │
                              │  - 工艺 HUD: 覆盖率/膜厚统计     │
                              └──────────────────────────────────┘
```

### 2.2 为什么优先选用“顶点颜色（Vertex Colors）”而非“UV 贴图”？
1. **天然契合当前工件数据**：
   - 现阶段工件是通过 3D 扫描重构生成的密集体素网格（`scan.mesh.ply` / `scan.mesh.stl`）；
   - 这类网格通常**没有展平的 UV 坐标贴图**，如果采用动态 2D 贴图印章（CanvasTexture），必须在运行时对复杂任意拓扑网格进行高开销的自动 UV 展开（UV Unwrapping），极易产生接缝畸变和性能消耗；
   - 扫描网格的顶点密度极高（单工件通常包含 10,000 ~ 150,000 个顶点），顶点间距通常在 0.5mm ~ 2mm 之间，**直接利用网格顶点的颜色插值（Gouraud / Phong Shading）即可获得极其平滑细腻的喷涂色斑效果**。
2. **极佳的写入与内存性能**：
   - 直接向已在显存中的 `geometry.attributes.color`（Float32Array）写入 RGB 增量；
   - 不需要管理额外的离屏 FBO、纹理内存和显存回读，完全契合嵌入式 RK3588 Mali GPU 资源敏感型设备。

---

## 3. 喷涂物理与几何数学模型 (Physical & Mathematical Formulation)

### 3.1 空间坐标系定义与 TCP 喷雾方向
1. **基座坐标系（Base Frame）**：所有世界几何计算的基础坐标系。工件网格顶点 $P_v \in \mathbb{R}^3$ 固定在 Base 系。
2. **工具坐标系（Tool TCP Frame）**：
   - 设当前配置的喷涂末端工具节点为 `gripper_tip_link` 或 `laser_head_link`；
   - 从该节点的 `matrixWorld` 中解算出喷嘴出口中心点世界坐标 $\mathbf{P}_{tcp}$；
   - 喷射主轴向量 $\mathbf{N}_{spray}$（通常为工具坐标系的 $+Z$ 轴，归一化后转换到 Base 系）：
     $$\mathbf{N}_{spray} = \text{normalize}\left(\mathbf{R}_{tcp} \cdot \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}\right)$$

### 3.2 喷雾几何投影圆锥模型 (Spray Frustum Model)
从喷嘴 $\mathbf{P}_{tcp}$ 沿方向 $\mathbf{N}_{spray}$ 建立发散立体圆锥：
- **名义靶距 (Standoff Distance)**：$H = \text{spray\_dist\_mm}$（配置项，通常 150mm）；
- **靶面名义喷幅宽 (Spray Width)**：$W = \text{spray\_width\_mm}$（配置项，通常 60mm）；
- **喷射扩散半角 (Cone Half-Angle)**：
  $$\theta_{half} = \arctan\left(\frac{W / 2}{H}\right)$$
- **有效喷射距离区间**：$[d_{min}, d_{max}]$，其中 $d_{min} \approx 20\text{mm}$，截断最大距离 $d_{max} \approx H \times 1.6$。

### 3.3 工件表面沉积速率与高斯衰减公式
对于工件表面任意顶点 $V_i = (x_i, y_i, z_i)$：
1. **相对向量计算**：
   $$\mathbf{D}_i = V_i - \mathbf{P}_{tcp}$$
   轴向距离（沿喷射主轴的投影距离）：
   $$d_{axial} = \mathbf{D}_i \cdot \mathbf{N}_{spray}$$
   若 $d_{axial} < d_{min}$ 或 $d_{axial} > d_{max}$，则判定在该帧有效射程外，直接跳过。

2. **径向偏离距离计算**：
   顶点在垂直于喷射主轴的横截面上的投影距离：
   $$r_{radial} = \left\| \mathbf{D}_i - d_{axial} \cdot \mathbf{N}_{spray} \right\|$$
   在当前轴向距离处的实际截面光斑半径：
   $$R(d_{axial}) = d_{axial} \cdot \tan(\theta_{half})$$
   若 $r_{radial} > R(d_{axial})$，则位于喷雾锥体外，跳过。

3. **高斯沉积分布与入射角余弦修正**：
   真实喷枪的喷幅中心涂料最浓，边缘按高斯分布扩散，且喷射表面倾角越大，沉积效率越低：
   - **高斯扩散系数**：设标准差 $\sigma = \frac{R(d_{axial})}{2.5}$，径向衰减权重为：
     $$w_{radial} = \exp\left(-\frac{r_{radial}^2}{2\sigma^2}\right)$$
   - **距离衰减系数**（遵循平方反比或线性平滑截断）：
     $$w_{dist} = 1.0 - \left(\frac{d_{axial} - d_{min}}{d_{max} - d_{min}}\right)^2$$
   - **表面法向夹角修正**（若网格具备顶点法向 $\mathbf{N}_{v,i}$）：
     $$\cos\alpha = \max\left(0, -\mathbf{N}_{spray} \cdot \mathbf{N}_{v,i}\right)$$
     如果夹角大于 $90^\circ$（背对喷嘴），则 $\cos\alpha = 0$，防止穿透染色。

4. **单帧膜厚增量**：
   $$\Delta T_i = Q_{rate} \cdot w_{radial} \cdot w_{dist} \cdot \cos\alpha \cdot \Delta t$$
   其中 $Q_{rate}$ 为出漆流量系数（标称值约 $15\,\mu\text{m/s}$），$\Delta t$ 为当前帧时间步长（秒）。

> **[评审 R3] $\Delta t$ 必须是真实物理停留时间，膜厚才有定量意义。** 当前离线回放循环按固定名义步速推进（约 60 步/秒 + 段间 10 子步，见 `InteractiveOp.tsx`），与配置的 `velocity`、`slerp_step_mm` 无关。要让 `Avg Film`/`Compliance` 成为可信工艺量，须把每个仿真步的 $\Delta t$ 按该段真实 MoveL 时间（≈ 点距 / 速度）映射；否则离线指标仅作**定性可视化**，HUD 须标注 `Indicative`。在线真机模式见 §4.4.1 的时间采样失准说明。

### 3.4 顶点颜色映射 (Color Mapping Modes)
系统内部为网格每个顶点维护一个独立的单精度浮点膜厚数组：`thicknessArray[i] += ΔT_i`。

#### 模式 A：Realistic Coating Mode（逼真涂装模式）
将底漆颜色（如初始银白色/金属灰 `#d1d5db`）向面漆颜色（如经典工业亮蓝 `#2563eb` 或艳红 `#dc2626`）按漆膜遮盖率过渡：
- 遮盖饱和厚度阈值设为 $T_{sat} = 50\,\mu\text{m}$；
- 遮盖率因子：$k_{cover} = 1.0 - \exp(-2.5 \cdot \frac{T_i}{T_{sat}})$；
- 最终颜色：
  $$\mathbf{C}_i = (1 - k_{cover}) \cdot \mathbf{C}_{substrate} + k_{cover} \cdot \mathbf{C}_{paint}$$

#### 模式 B：Thickness Heatmap Mode（膜厚工艺热力图模式）
根据标准工业喷涂厚度公差（例如标准 $40 \sim 60\,\mu\text{m}$）生成彩虹/多级色谱：
- $T_i = 0\,\mu\text{m}$：工件基底灰（未喷涂 / 漏喷）
- $0 < T_i < 30\,\mu\text{m}$：深蓝 $\rightarrow$ 青绿（过薄 Under-sprayed）
- $30 \le T_i \le 60\,\mu\text{m}$：翠绿 $\rightarrow$ 黄绿（合格标准 Standard Thickness）
- $60 < T_i \le 80\,\mu\text{m}$：橙黄（轻度过厚 Slight Over-spray）
- $T_i > 80\,\mu\text{m}$：鲜红 $\rightarrow$ 绛紫（严重超标 / 流挂风险 Sagging Danger）

---

## 4. 详细模块设计与类结构 (Detailed Software Design)

```
app/frontend/src/components/
├── Robot3DViewer.tsx              # 主 3D 视口容器
└── spray/
    ├── SprayParticleEmitter.tsx   # 喷枪末端粒子雾化视效组件
    ├── SurfaceStainingEngine.ts   # 表面网格顶点投影与染色核心算法
    ├── SpraySimulationOverlay.tsx # 3D 视口浮动英文 HUD 与控制药丸
    └── sprayTypes.ts              # 喷涂仿真配置与数据结构定义
```

### 4.1 数据结构定义 (`sprayTypes.ts`)

```typescript
export type SprayVisualMode = 'realistic' | 'heatmap';

export interface SpraySimConfig {
  enabled: boolean;             // 总开关: 是否启用仿真喷漆
  visualMode: SprayVisualMode;  // 视图模式: 逼真油漆 vs 膜厚热力图
  paintColor: string;           // 逼真模式下的油漆颜色 (Hex, 如 '#2563eb')
  targetDistanceMm: number;     // 标称喷涂距离 (spray_dist_mm, 默认 150)
  sprayWidthMm: number;         // 标称喷幅宽度 (spray_width_mm, 默认 60)
  flowRateMicronsPerSec: number;// 出漆速率 (微米/秒, 默认 15.0)
  targetMinThickness: number;   // 达标最小厚度 (微米, 默认 35)
  targetMaxThickness: number;   // 达标最大厚度 (微米, 默认 65)
}

export interface SprayCoverageStats {
  totalVertices: number;
  coveredVertices: number;
  coveragePercentage: number;   // 覆盖率 0.0 ~ 100.0%
  averageThickness: number;     // 覆盖区域平均膜厚 (微米)
  maxThickness: number;         // 最大局部膜厚 (微米)
  standardComplianceRate: number;// 处于达标区间内的百分比 0.0 ~ 100.0%
}
```

### 4.2 视觉雾化粒子组件 (`SprayParticleEmitter.tsx`)

#### 设计关键点
1. **轻量粒子池 (Object Pool)**：
   - 预分配固定长度为 300 的 `Float32Array`（位置、速度、寿命、大小）；
   - 不在 `useFrame` 中销毁或动态 `new` 对象，所有粒子在寿命归零时原地重置到喷嘴起始点；
2. **挂载在 TCP 局部坐标系**：
   - 直接作为 `tcpLinkRef` 的子节点，或者每帧取 `tcpLink.matrixWorld` 同步；
   - 粒子生成时在喷嘴出口圆域内随机微偏移，沿 $+Z$ 轴施加主速度，并在径向施加轻微扩散速度与阻尼；
3. **着色器/材质表现**：
   - 采用 `THREE.Points` 与软圆形半透明纹理贴图（Alpha Radial Map）；
   - 开启 `depthWrite: false`, `transparent: true`, `blending: THREE.AdditiveBlending` 或 `NormalBlending`，模拟真实细密漆雾的通透消散感。

### 4.3 工件染色引擎 (`SurfaceStainingEngine.ts`)

```typescript
export class SurfaceStainingEngine {
  private mesh: THREE.Mesh;
  private geometry: THREE.BufferGeometry;
  private positions: Float32Array;
  private normals: Float32Array | null;
  private colors: Float32Array;
  private thicknessArray: Float32Array;
  private vertexCount: number = 0;
  
  // 零 GC 临时变量缓存 (Preallocated Scratch Objects)
  private _tcpPos: THREE.Vector3 = new THREE.Vector3();
  private _tcpDir: THREE.Vector3 = new THREE.Vector3();
  private _vertPos: THREE.Vector3 = new THREE.Vector3();
  private _vertNorm: THREE.Vector3 = new THREE.Vector3();
  private _diff: THREE.Vector3 = new THREE.Vector3();
  // [评审 R5] 预分配 scratch 矩阵，供每帧 world→local 变换复用（禁 clone/invert）
  private _invMatScratch: THREE.Matrix4 = new THREE.Matrix4();

  constructor(targetMesh: THREE.Mesh) {
    this.attachMesh(targetMesh);
  }

  public attachMesh(targetMesh: THREE.Mesh): void {
    this.mesh = targetMesh;
    this.geometry = targetMesh.geometry;
    this.positions = this.geometry.attributes.position.array as Float32Array;
    this.vertexCount = this.geometry.attributes.position.count;
    
    // 初始化或确保拥有顶点法线
    if (!this.geometry.attributes.normal) {
      this.geometry.computeVertexNormals();
    }
    this.normals = this.geometry.attributes.normal?.array as Float32Array || null;

    // 分配膜厚数组
    this.thicknessArray = new Float32Array(this.vertexCount);

    // 确保包含 color 属性缓冲
    if (!this.geometry.attributes.color) {
      this.colors = new Float32Array(this.vertexCount * 3);
      // 默认填充底漆灰色 (0.85, 0.85, 0.85)
      this.colors.fill(0.85);
      this.geometry.setAttribute('color', new THREE.BufferAttribute(this.colors, 3));
    } else {
      this.colors = this.geometry.attributes.color.array as Float32Array;
    }
  }

  /**
   * 空间加速与局部更新核心执行步
   */
  public applySprayStep(
    tcpWorldPos: THREE.Vector3,
    tcpWorldDir: THREE.Vector3,
    config: SpraySimConfig,
    deltaSec: number
  ): boolean {
    if (!this.mesh || this.vertexCount === 0) return false;

    // 1. 将 TCP 坐标与方向转换到工件 Mesh 的局部坐标系 (Local Coordinates)
    //    由于扫描网格一般直接挂载在 base_link 下，若有位姿变换，逆矩阵转换可将后续顶点运算降为纯局部坐标计算
    // [评审 R5] 复用预分配 scratch 矩阵，避免每帧 clone()/invert() 产生 GC（违反 §6.1 Zero-GC）
    this._invMatScratch.copy(this.mesh.matrixWorld).invert();
    const invMat = this._invMatScratch;
    this._tcpPos.copy(tcpWorldPos).applyMatrix4(invMat);
    this._tcpDir.copy(tcpWorldDir).transformDirection(invMat).normalize();

    const distMm = config.targetDistanceMm / 1000.0; // 米
    const widthMm = config.sprayWidthMm / 1000.0;
    const tanHalf = (widthMm / 2.0) / distMm;
    const minD = 0.02; // 20mm
    const maxD = distMm * 1.6;

    let modified = false;
    const pos = this.positions;
    const norm = this.normals;
    const thk = this.thicknessArray;
    const count = this.vertexCount;
    const flowRate = config.flowRateMicronsPerSec;

    // 2. 遍历顶点计算沉积 (局部距离判定快速过滤)
    for (let i = 0; i < count; i++) {
      const i3 = i * 3;
      const vx = pos[i3];
      const vy = pos[i3 + 1];
      const vz = pos[i3 + 2];

      const dx = vx - this._tcpPos.x;
      const dy = vy - this._tcpPos.y;
      const dz = vz - this._tcpPos.z;

      // 轴向投影距离
      const dAxial = dx * this._tcpDir.x + dy * this._tcpDir.y + dz * this._tcpDir.z;
      if (dAxial < minD || dAxial > maxD) continue;

      // 径向距离平方
      const radSq = (dx - dAxial * this._tcpDir.x) ** 2 +
                    (dy - dAxial * this._tcpDir.y) ** 2 +
                    (dz - dAxial * this._tcpDir.z) ** 2;
      const coneR = dAxial * tanHalf;
      if (radSq > coneR * coneR) continue;

      // 法向夹角衰减 (若有法线)
      let cosAlpha = 1.0;
      if (norm) {
        const nx = norm[i3];
        const ny = norm[i3 + 1];
        const nz = norm[i3 + 2];
        const dot = -(this._tcpDir.x * nx + this._tcpDir.y * ny + this._tcpDir.z * nz);
        if (dot <= 0.05) continue; // 背光面或大掠射角不着色
        cosAlpha = Math.min(1.0, dot);
      }

      // 高斯沉积增量
      const sigmaSq = (coneR / 2.5) ** 2;
      const wRadial = Math.exp(-radSq / (2.0 * sigmaSq));
      const distNorm = (dAxial - minD) / (maxD - minD);
      const wDist = 1.0 - distNorm * distNorm;

      const deltaT = flowRate * wRadial * wDist * cosAlpha * deltaSec;
      if (deltaT > 0.01) {
        thk[i] += deltaT;
        modified = true;
        this.updateVertexColor(i, thk[i], config);
      }
    }

    if (modified) {
      this.geometry.attributes.color.needsUpdate = true;
    }
    return modified;
  }

  /**
   * 重置所有表面着色（恢复底漆原貌）
   */
  public clearCoating(): void {
    this.thicknessArray.fill(0);
    this.colors.fill(0.85);
    if (this.geometry?.attributes.color) {
      this.geometry.attributes.color.needsUpdate = true;
    }
  }
}
```

#### 4.3.1 渲染前置条件（[评审 R1] 必须修正，否则染色不显示）
1. **材质必须开 `vertexColors: true`**：当前 `Robot3DViewer.tsx` 的 `reconstructed_surface_mesh` 用 `MeshStandardMaterial({ color: #ffffff, ... })`，**未启用顶点色**。只往 `geometry.attributes.color` 写数据而不设 `material.vertexColors = true`，画面不会有任何变化；开 `vertexColors` 后材质 `color` 会与顶点色相乘，需把 `color` 置白作为中性基准。
2. **材质与现有效果兼容**：现有网格带 `transparent: true, opacity: 0.92, polygonOffset`，染色需与之共存（半透叠加会拑制颜色饱和度，realistic 模式下应适当提高漆色对比）。
3. **颜色模式与线性空间一致**：`MeshStandardMaterial` 走 PBR 线性工作流，写入 `attributes.color` 前需明确颜色处于 sRGB 还是 linear（与 renderer 的 `outputColorSpace` 对齐），否则 realistic 漆色与 heatmap 色谱会出现偏色。heatmap 建议走 `MeshBasicMaterial` 或 `toneMapped: false`，避免光照影响色谱读数。
4. **`frustumCulled` 与 geometry 生命周期**：染色不改变位置，无需重置包围体；但 `attachMesh` 必须在 mesh 重载（`meshVersion` 变化）时重新抓取 `attributes` 引用，否则会写入已 dispose 的旧 buffer。

### 4.4 状态联动与双模式触发逻辑 (Triggering Mechanisms)

喷涂物理与染色触发器设计为支持**双通道输入**：

```typescript
// 判断当前是否允许发射粒子并着色
// [评审 R2] 字段以实际代码为准：不存在 currentStepSpraying / stage==='spraying'；
// SimulationState 实际字段：isPlaying / currentStep / currentTcpPose / currentJoints / totalSteps。
const isSprayingActive = useMemo(() => {
  if (!spraySimConfig.enabled) return false;

  // 通道 1: 真机运行/在线调试模式（控制器反馈为唯一准源，status: 1=Moving）
  if (robotState.status === 1) {
    const sprayDoIndex = configuredSprayDoIndex || 1; // 默认 DO 1
    const isDoActive = !!robotState.digital_outputs?.[sprayDoIndex - 1];
    return isDoActive;
  }

  // 通道 2: 离线轨迹预演/仿真模式（需先从 simSteps[currentStep].spraying 读取）
  if (simulationState?.isPlaying) {
    const step = simStepsRef.current?.[simulationState.currentStep];
    return step?.spraying === 'on';
  }

  return false;
}, [spraySimConfig.enabled, robotState.status, robotState.digital_outputs, simulationState]);
```

> **[评审 R2 需先落地的数据打通]** 现 `InteractiveOp.tsx` 构建的 `simSteps` 每项为 `{ q_deg, tcp, pixel, pathIdx }`，**未携带 `spraying` 标志**。实现前必须在生成仿真步时从路径数据透传每个航点的 `spraying`（该字段路径里已有，见 `Robot3DViewer` 对 `p.spraying === 'off'` 的使用），否则通道 2 无法判定何时开喷。

> **[评审 R4 在线真机为空间失准示意]** 通道 1 靠 WebSocket 低频轮询（几 Hz）的 `digital_outputs` 与轮询位姿驱动，而真实 DO 是队列内**空间触发**；时间采样 vs 空间触发必然失同步，在线染色/粒子节奏≠真实喷涂。因此在线定位为“实时示意”，**工艺判断一律以离线预演（通道 2）为准**；且 `status !== 1`（Idle）时需明确停喷后是否保留染色。

---

## 5. 前端交互界面与 HUD 控制面板规范 (Strict English UI Spec)

遵照项目常驻规范《Strict English-Only UI Interface》，所有新界面元素、标签、弹窗、Tooltip、状态胶囊必须 100% 采用专业英文，禁止任何中文字符。

### 5.1 3D 视口内悬浮动作工具条 (HUD Actions Toolbar)
在 `Robot3DViewer` 的右上角操作按钮区（与已有的 `Maximize`、`Mesh Toggle`、`Path Toggle` 平行）增加喷涂仿真专区：

| 控件英文名称 | 类型 | 状态与交互 | 英文 Tooltip 提示 |
|---|---|---|---|
| `Spray Sim` | Toggle Pill / Button | `ON (Cyan Active)` / `OFF (Muted Slate)` | "Toggle 3D spray mist & surface painting simulation" |
| `View Mode` | Segmented Switch | `Realistic` \| `Heatmap` | "Switch between realistic coat and thickness heatmap" |
| `Clear Coat` | Action Button (RotateCcw) | Click to reset | "Clear accumulated coating from workpiece surface" |

### 5.2 喷涂工艺统计指标胶囊 (HUD Metrics Pill)
当 `Spray Sim` 处于开启状态时，在视口顶部中间或左侧显示半透明玻璃拟态指标胶囊：

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ [● SPRAY SIM]  Coverage: 94.2%  |  Avg Film: 52.4 μm  |  Compliance: 91.0%  │
└─────────────────────────────────────────────────────────────────────────────┘
```
- **Coverage**：`coveredVertices / totalVertices * 100%`；
- **Avg Film**：已覆盖区域的平均膜厚（单位：`μm`）；
- **Compliance**：膜厚处于标准目标区间（如 $35 \sim 65\,\mu\text{m}$）内的比例。

> **[评审 R7] 覆盖率分母需修正。** `coveredVertices / totalVertices` 把背面/不可喷面也计入分母，且扫描网格顶点密度不均，导致 Coverage 系统性偏低且不可比。应改为**以可及正面面积加权的分母**（按三角形面积累加，而非顶点计数），并考虑用离屏面积图而非顶点均值。另提醒：`Avg Film`/`Compliance` 在离线模式仅在按 §3.3 完成时间标定后才为定量值，否则 HUD 应标注 `Indicative`。

---

## 6. 工业级最佳实践与性能优化防护 (Robotics Best Practices & Performance)

### 6.1 嵌入式与移动端 GPU 优化 (Mali-G610 on RK3588)
1. **零垃圾回收（Zero-GC per Frame）**：
   - 在 `applySprayStep` 循环体和 `useFrame` 中，严禁出现 `new THREE.Vector3()`、`new THREE.Color()`、`array.map()` 或解构赋值；
   - 所有的临时向量、矩阵与数学运算全部复用类成员变量（Scratch variables）。
2. **动态帧率自适应与节流（Throttle & Subsampling）**：
   - 雾化粒子视效以 60 FPS 渲染保证视觉丝滑度；
   - 表面网格染色计算可根据网格顶点总数自适应节流：
     - 网格顶点数 $< 50,000$：逐帧 60 Hz 实时更新；
     - 网格顶点数 $> 50,000$：采用 30 Hz 步进更新（每隔 1 帧更新一次 `geometry.attributes.color.needsUpdate = true`），极大降低 GPU 显存总线带宽占用。
3. **空间距离粗筛（Bounding Box / Spatial Culling）**：
   - 在进入内层循环前，先对工件网格整体 AABB 与 TCP 距离进行包围球相交判断；如果 TCP 距离工件最近点 $> 300\text{mm}$，直接整帧跳过网格遍历。
4. **[评审 R6] 增量上传与局部遍历（必需，否则 Mali 上不过关）**：
   - **只上传改变区间**：`needsUpdate = true` 会全量重传整块 color buffer（最多 450k float/帧）。应使用 `attribute.addUpdateRange(start, count)`（需 THREE 版本支持）只上传本帧实际改动区间，避免全量显存总线占用；
   - **只遍历候选顶点子集**：把包围球粗筛升级为**顶点空间分桶（均匀网格哈希）**，每帧只遍历落入喷涂锥邻域内的桶，将 O(N) 降为 O(局部)。
   - 30Hz 节流只砍一半、不改变渐近复杂度；Phase 2 是全案性能风险最高点。

### 6.2 工业故障安全互锁 (Fail-Safe Interlock)
1. **异常断喷**：
   - 当机械臂控制器触发报警（`robotState.error_status !== 0`）、急停按键被按下或 WebSocket 断开连接时，前端仿真引擎**必须立即将粒子发射速率归零**，与底层驱动断开喷涂 DO 的 Fail-Safe 规则严格同步；
2. **防穿透与背阴剔除**：
   - 通过顶点法线与喷射轴向的负点乘余弦值 $\cos\alpha > 0.05$ 严格拦截背向面，避免喷雾穿透薄壁工件（如钣金件、裤管反面）导致背面误着色。

### 6.3 已知物理局限（[评审 R8] 实现前必须向使用人员声明）
1. **光斑形状简化**：模型用**圆形圆锥**近似喷幅，而真实无气/空气喷枪多为**扁平椭圆扇形光斑**（可通过改变水平/垂直方向 tanHalf 扩展为椭圆锥）。圆形近似会影响 overlap / 漏喷预测精度。
2. **自遮挡未处理**：$\cos\alpha > 0.05$ 只拦截**朝向喷嘴**的一面，不做真实可见性/阴影剔除；处于锥体内、面向喷嘴但被中间结构遮挡的背面仍会被误染（薄壁、裤管反面尤其）。若需精确需引入 shadow map / GPU 可见性，属后续增强。
3. **沉积为各向同性高斯**：未建模涂料飞散、反弹、环境风、湿度与粘度和叠层湿润，膜厚分布为近似而非 CFD 级仿真。
4. **定位重申**：鉴于上述局限与 §0.1/§3.3，本功能作为**工艺可视化预览与培训/展示**价值明确；若作为**量产工艺定量验收**，需标定、补上自遮挡与扇形光斑，并与实测膜厚对照后才能采信。

---

## 7. 实施计划与里程碑 (Roadmap & Milestones)

| 阶段 | 里程碑任务 | 预计工期 | 产出物 |
|---|---|---|---|
| **Phase 1** | **数据结构与粒子视效基础** | 1 天 | `sprayTypes.ts`, `SprayParticleEmitter.tsx`，实现喷枪末端 300 粒子锥形雾化扩散与消隐视效。 |
| **Phase 2** | **表面网格投影与高斯着色引擎** | 1.5 天 | `SurfaceStainingEngine.ts`，完成无 UV 密集网格的顶点颜色计算、高斯喷斑与厚度累加。 |
| **Phase 3** | **真机 DO 联动与离线预演双驱** | 1 天 | 打通 `robotState.digital_outputs` 与 `SimulationState`，接入 `aisprayer_config.yaml` 喷幅与靶距参数。 |
| **Phase 4** | **热力图模式与全英文 HUD 交互** | 1 天 | 实现 Realistic / Heatmap 切换、指标统计胶囊、清空重置按钮及全屏模式自适应。 |
| **Phase 5** | **RK3588 性能调优与回归测试** | 0.5 天 | 显存与 CPU 占用基准测试，完成代码审查与防退化验收。 |

> **[评审工期提示]** Phase 2（表面染色引擎性能，含 §6.1-R6 增量上传/顶点分桶）是全案风险最高项，1.5 天偏紧，建议先做性能 spike；Phase 3 需含 §4.4-R2 的 `simSteps.spraying` 数据打通（前置必改）。

---

## 8. 附录：配置项扩展说明 (`configs/aisprayer_config.yaml`)

建议后续在 `aisprayer_config.yaml` 的 `spraying` 小节下预留如下可选仿真字段：

```yaml
spraying:
  # 现有核心工艺参数
  spray_dist_mm: 150.0          # 默认喷涂靶距 (mm)
  spray_width_mm: 60.0          # 喷涂幅宽 (mm)
  velocity: 150.0               # 喷涂速度 (mm/s)
  
  # 仿真与数字孪生扩展项 (Simulation & Digital Twin)
  simulation:
    enable_mist_particles: true # 是否开启雾化粒子特效
    particle_count: 300         # 视效粒子数量
    flow_rate_um_s: 15.0        # 标称出漆速率 (微米/秒)
    target_film_min_um: 35.0    # 合格膜厚下限 (微米)
    target_film_max_um: 65.0    # 合格膜厚上限 (微米)
    default_paint_color: "#2563eb" # 默认油漆颜色
```

> **[评审 R9] 配置落位建议。** 上述 `simulation.*` 字段属**纯前端可视化参数**，建议默认只放在前端常量/store，避免污染驱动真机的 `configs/aisprayer_config.yaml`。只从 config 读真机也共用的 `spray_dist_mm` / `spray_width_mm`。若确实需要在 System Config UI 可调，应按项目既有模式在后端 `core/config.py` 的 `CONFIG_REGISTRY` 注册（参照 `spray_on_delay_ms` 做法），而非只加 yaml 键（否则不会出现在 UI且无默认/校验）。
