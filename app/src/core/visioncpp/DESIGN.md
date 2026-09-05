# visioncpp：视觉批处理 C++ CLI 设计（修订稿）

> **状态**：P0–P2 已开工（实现在本目录；未改 `vision/` / FastAPI）。  
> **日期**：2026-09-05  
> **替代**：`app/docs/vision_cpp_cli_refactor_analysis.md`（已作废，以本文件为准）

---

## 0. 硬约束

1. **不改现有 `app/src/core/vision/`**。Web 在替换日之前继续走旧 Python。
2. **新代码与文档都只在本目录** `app/src/core/visioncpp/`。功能经黄金测试稳定后，再把 FastAPI 切到本 CLI，最后才考虑删除旧 `vision`。
3. **不考虑对旧 Python API 的兼容**（不保留 Identity 标定、不保留默认假内参、不保留 YOLO/`from_file` 离线支路）。
4. **优先质量、稳定、性能**。数字以 RK3588 实测为准，不写未测量的「50 倍」。
5. **MobileSAM 不进本仓库、不做一次性 CLI**。交互分割继续留在 `apps/interactive/sam_service.py`。

---

## 1. 结论（修订后）

| 能力 | 现网 | 是否进 visioncpp | 形态 | 真实收益 |
| :--- | :--- | :--- | :--- | :--- |
| 泊松重建 | Open3D + spawn 子进程 | **做** | `vision_cli recon` | **稳**：换掉 aarch64 Open3D 等值面竞态/段错误；去掉每次 2–3s 的解释器+import。速度目标是「纯计算不慢于现网泊松」，不是保证 &lt;0.8s |
| 自动航点 | Trimesh + SciPy | **做** | `vision_cli auto-path` | **快**：C++ 切片/KD-Tree 会快一截，RK3588 先以 **≤500ms** 为验收，再看能否压到 100ms |
| MobileSAM | 常驻 PyTorch | **不做** | 保持现服务 | 一次性 CLI 会把点击从 ~20ms 打到秒级 |

**阶段顺序（与初稿相反）**：先 `recon`（现网会崩后端），再 `auto-path`（只是慢）。  
初稿把 auto-path 写成「Must Have / 50 倍 / &lt;50ms」过满；重建写成「几个文件接入 Kazhdan」过轻。

---

## 2. 现网事实（对照源码，不对照愿望）

Web 路径：

```
POST /templates/{name}/reconstruct  → reconstruction_service（spawn）→ PoissonReconstructor.reconstruct_mesh
POST /templates/{name}/auto_paths   → auto_path_service → JeansAutoWaypoints.plan
POST /templates/{name}/sam/predict  → sam_service（常驻）          ← 本目录不管
```

**重建真实参数**（`reconstruction_service._reconstruct_surface_impl`，不是类默认值）：

| 键 | 现网值 | 说明 |
| :--- | :--- | :--- |
| 深度 | `scan.depth.png`（16-bit mm）或 `.npy` | png 优先 |
| 掩码 | `scan.masks.yaml` → `fillPoly` | &lt;50 像素失败 |
| 内参 | `scan.params.yaml` → 标定文件 → **默认 Gemini K** | 新实现**禁止**默认 K |
| 手眼 | 最新 `calibration_result.yaml` → 全局配置 → **Identity** | 新实现**禁止** Identity |
| `poisson_depth` | **8** | 类默认 9，Web 传 8，CLI 锁 8 |
| `density_threshold` | 0.15 | |
| `voxel_size` | 0.003 m | |
| `normal_radius` | 0.03 m | |
| `smooth_iterations` | 20（Taubin） | |
| `z_min/z_max` | 100 / 3000 mm | |
| `mask_erode_px` | 1 | |
| `flying_pixel_max_grad` | 50 mm/px | |
| 输出 | `scan.mesh.ply` + `scan.mesh.stl`，顶点在 **base**，单位 **米** | |

流水线（必须整段搬，不能只换泊松核）：

1. 读深度 + 栅格化掩码  
2. 掩码内无效深度 `inpaint`（NS, radius=5）  
3. 腐蚀 + Sobel 飞点  
4. 深度→相机系点云（mm）→ /1000 → 米  
5. 体素下采样 3mm + 统计离群  
6. 估计法向，朝相机原点定向  
7. 泊松 depth=8，按密度分位数裁顶点  
8. Taubin 平滑 20 次  
9. `T_camera_to_base` 变到基座  
10. 只留最大面连通块  
11. 写 PLY/STL  

**自动航点真实行为**：

- 输入：`scan.mesh.ply`（米、base）、`scan.masks.yaml`、K、`T_camera_to_base`  
- 无 K 或 T 为 Identity：**已拒绝**（这一点新实现保持）  
- `dedup_radius_mm = 0.5 * row_spacing`  
- 2D 分腿（`split_jeans_mask`，`overlap_px=0`）→ 投影顶点打标签 → 每腿平面切片 → KD-Tree 取顶点法向 → 滑动平均法向 → 工具系 + 写出 `scan.auto.path.yaml`  
- 不修改输入 mesh 的 faces  

现网已有隐患（**只在 visioncpp 里修，不回改 vision**）：

1. 法向滑动平均后模长 &lt;1e-6 时除以 1，得到 **全零法向**。新实现：回退到该点平滑前的法向；再没有则用 `[0,0,1]`，禁止写零向量。  
2. 工具系只用第一帧检测 `x_ref` 与 `z_tool` 平行；之后继承 `prev_x_tool`。大扭转时叉乘退化，现网用 `[0,1,0]` 硬填，姿态会跳。新实现：**每点**检查 `|dot(z_tool, x_ref)|`，超阈则重选辅助轴，并与上一帧 `x_tool` 做符号连续。  
3. 重建缺标定时用 Identity / 假 K，网格看起来成功、航点全错。新实现：**缺 T 或 K 直接失败**。

---

## 3. 初稿哪些判断作废

| 初稿 | 修订 |
| :--- | :--- |
| auto-path 50 倍 / &lt;50ms | 未 profile。RK3588 验收 ≤500ms；更快是加分项 |
| 重建 4–6s → 0.8s、内存 2.5GB → 200MB | spawn+import 占了现网很多时间；单线程 Kazhdan **可能更慢**。验收：无段错误、RSS 峰值低于现网 spawn、纯泊松不差于现网一个数量级 |
| 第一阶段删 `vision_processor` / `visualize` / 测试用 sampler | **不删旧 vision**。这些文件只是不搬进本目录 |
| 先 auto-path 后 recon | **先 recon** |
| 黄金比对 0.1mm / 0.1° | 换泊松核后面片对不上。recon 比 Hausdorff/连通性；auto-path 在**同一张输入 mesh** 上比，坐标 1mm、姿态 2° 起步 |
| Kazhdan「几个文件、中等成本」 | 前后处理（飞点、法向、密度裁、Taubin、连通块、PLY/STL）才是主体 |
| `poisson-depth` 默认 8 当类默认 | Web 是 8，锁 8 |

---

## 4. 本目录要写什么、不写什么

**写（一份实现，recon / auto-path 共用）：**

- 深度读写（png 16UC1 / npy）  
- `scan.masks.yaml` 栅格化（一处，禁止两套）  
- `scan.params.yaml` 读 K 与宽高  
- 标定 yaml 读 `T_camera_to_base`（失败即退出）  
- 点云：反投影、体素、统计滤波、法向、定向  
- 泊松 + 密度裁 + Taubin + 最大连通块 + 刚体变换  
- 网格 IO（PLY + STL）  
- 分腿、外侧缝、平面切片、弧长采样、KD-Tree 法向、去重、法向平滑、工具系  
- `vision_cli`：`recon` / `auto-path`  
- 黄金测试（固定模板目录或合成盒）

**不写（旧 vision 里有、本目录视为死路径）：**

- `VisionProcessor` BPA、Open3D GUI / `visualize` / `recorder`  
- `conformal_sampler` / `surface_sampler` / `zigzag_sampler` / 已删的 `strategies`  
- YOLO `segmenter`、`from_file`、分腿各建一个 mesh  
- `poisson_reconstruct_for_surface_walk`  
- Open3D 可视化、`calculate_cut_direction`  
- 假内参、Identity 手眼、spawn/重试胶水（CLI 崩了由 Python 调方决定是否重试一次）

---

## 5. 架构

```
app/src/core/visioncpp/
  DESIGN.md                 ← 本文件
  CMakeLists.txt
  include/visioncpp/*.hpp
  src/                      几何与 IO，无 httplib、无 Open3D、无 PyTorch
  src/cli_main.cpp          vision_cli
  test/                     黄金测试
  python/cli_client.py      仅本目录自测用；FastAPI 替换日再接到 apps/interactive
```

```
FastAPI（替换日之前不改）
    │
    └─ 替换日后：visioncpp/python/cli_client.py
           └─ subprocess vision_cli recon | auto-path
                  stdout 最后一行 JSON
                  stderr 行日志
```

和 `motion_cli` 同一契约：可执行文件、超时、非零退出码、不把 mesh/深度塞进 stdin。

构建：`app/scripts/build.sh --only visioncpp`（确认开工后加）。复用 `app/cmake/platform.cmake`。默认 `FOLLOW`/`planner` 无关。

---

## 6. 技术选型

| 用途 | 库 | 说明 |
| :--- | :--- | :--- |
| 线性代数 | Eigen3 | 与 motion/follow 同一套 |
| YAML | yaml-cpp | masks / params / 标定 / 航点 |
| 图像 | OpenCV C++ | png、inpaint、Sobel、fillPoly、轮廓分腿 |
| 泊松 | 单线程 Kazhdan PoissonRecon（vendored 源码） | **禁止**链 Open3D。接入成本按「整段前后处理」估，不按「拷几个 .cpp」估 |
| KD-Tree | nanoflann（header-only） | 法向查询、裆部去重 |
| 网格 IO | 自写最小 PLY/STL | 不引入 trimesh |

切片求交：三角形–平面求交 + 折线串，自己写。不链 CGAL。算法与现网一致：平面法向 × 原点、交点去重（1e-4 m）、沿外侧缝轴排序、弧长按 `point_spacing` 插值。

泊松：显式单线程提取等值面。depth 固定默认 8，CLI 可改，Web 对接时传 8。

---

## 7. CLI 规格

```bash
vision_cli recon \
  --template-dir <dir> \
  --calib <calibration_result.yaml> \
  [--poisson-depth 8] [--voxel-size 0.003] [--density-threshold 0.15] \
  [--smooth-iterations 20]

vision_cli auto-path \
  --template-dir <dir> \
  --calib <calibration_result.yaml> \
  [--spray-dist 150] [--row-spacing 60] [--point-spacing 100] \
  [--dedup-radius 30] \
  [--output <dir>/scan.auto.path.yaml]
```

规则：

- `--calib` 必填且必须读到 4×4 `T_camera_to_base`。读不到 → 退出码 2。  
- K 必须来自 `scan.params.yaml`（或标定文件里的 `intrinsic_matrix`）。都没有 → 退出码 2。禁止写死 611.68。  
- `recon` 缺深度或掩码 → 退出码 2。  
- `auto-path` 缺 mesh 或掩码 → 退出码 2。  
- 成功：stdout **仅最后一行** JSON；过程日志走 stderr。  
- 失败：stdout 可有一行 `{"status":"error","error":"..."}`，退出码非 0。

`recon` 成功 JSON：

```json
{"status":"success","vertices":45210,"faces":90320,"elapsed_ms":1200,"files":["scan.mesh.ply","scan.mesh.stl"]}
```

`auto-path` 成功 JSON：

```json
{"status":"success","path_count":1,"point_count":348,"elapsed_ms":180}
```

航点 yaml 字段与现网 `scan.auto.path.yaml` 同构（`paths[].points[]` 的 xyz / rpy / 投影），方便替换日只换生成器、不改前端。这是**文件格式对齐**，不是 Python API 兼容。

---

## 8. 验收

### 8.1 recon（优先）

在 RK3588 上用同一模板连跑 ≥20 次：

- 退出码恒 0，无段错误、无挂死（超时 60s 视为失败）  
- 面数波动相对初稿「71222/23/24」应消失或远小于现网 Open3D  
- RSS 峰值低于现网 spawn 子进程  
- 对同一输入：到参考点云的 Hausdorff（或顶点–点云距离 p95）有记录；**不要求**与旧 PLY 拓扑一致  

### 8.2 auto-path（同一张 mesh）

用现网已生成的 `scan.mesh.ply`（不要用新泊松的网去对旧航点）：

- 与现网 Python `JeansAutoWaypoints` 输出比：对应点位置 **≤1mm**，工具姿态 **≤2°**（先达到再考虑收紧）  
- 无全零法向、无相邻点 rpy 无故翻 180°  
- 输入 mesh 的 face 数 recon/plan 前后不变  

合成盒（两根细盒子当裤腿）进 `test/`，不依赖现场扫描。

### 8.3 不验收

- 与旧 Open3D 网格逐顶点相等  
- MobileSAM  
- macOS 上的 Open3D GUI  

---

## 9. 实施顺序（确认后才执行）

**P0 脚手架**：CMake、`platform.cmake`、`vision_cli --help`、`build.sh --only visioncpp` / `-c --only visioncpp`。

**P1 `recon`**：IO → 前处理 → 泊松 → 后处理 → 黄金/连跑。Python `cli_client.py` 只供本目录手跑。

**P2 `auto-path`**：分腿 + 切片 + 姿态（含 §2 两处 bug 修复）→ 固定 mesh 黄金比对。

**P3 替换日（另批）**：`reconstruction_service` / `auto_path_service` 改调 CLI，去掉 spawn。旧 `vision/` 仍先留着，确认现场没问题再删。

---

## 10. 请确认的决策

1. 先 recon、后 auto-path。  
2. 缺手眼或内参直接失败，不再 Identity / 假 K。  
3. 本阶段不改 `vision/`、不改 FastAPI。  
4. auto-path 黄金阈值 1mm / 2°；recon 不跟旧 PLY 逐点比。  
5. 泊松用单线程 Kazhdan，不链 Open3D。  

确认这五条后按 P0→P1→P2 写代码。
