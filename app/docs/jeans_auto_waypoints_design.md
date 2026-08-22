# 牛仔裤自动航点生成（`jeans_auto_waypoints.py`）设计方案

状态：第 8 节已拍板，实现见 `app/src/core/vision/jeans_auto_waypoints.py`

---

## 1. 背景与目标

现有 3D 之字采点（`conformal_sampler.py` + `test_surface_zigzag.py` / `reconstruction.from_file(split_parts=2)`）的做法是：

1. 在 **2D 掩码上切裤裆**（`split_jeans_mask`），必要时带重叠带；
2. **分别重建两条裤腿 mesh**（或对已有 mesh 做几何切开）；
3. 对每一半各自跑 `SurfaceConformalSampler.sample()`。

问题：

- 切半重建会在裆部切口丢面、再靠 `overlap_px` 补缝，裆部几何不稳定；
- 整裤一条 mesh（交互重建产出的 `scan.mesh.ply/stl`）无法直接复用这套逻辑；
- 采样器只吐 `{point, normal, is_jump}`，没有生产路径 YAML。

本方案新增独立模块 **`app/src/core/vision/jeans_auto_waypoints.py`**（不改 `conformal_sampler.py`），在 **完整单 mesh** 上按掩码识别 1/2 条裤腿，**每条腿用自己的 3D PCA 方向走之字**，裆部重叠区去重，输出与 `scan.raw.path.yaml` 同构的内存字典，由外层落盘为 `scan.auto.path.yaml`。

---

## 2. 非目标

- 不改 `conformal_sampler.py` / `surface_sampler.py` / `jeans_segmentation.py` 的现有行为。
- 本模块 **不读、不写文件路径**；PLY/STL/YAML 的打开与保存由外层（CLI / `interactive` service）完成。
- 不做运动学校验、Viterbi 姿态优化（那是 `path_opt` 对 `scan.opt.path.yaml` 的事）。
- 不把 mesh 沿中线几何切开，也不按腿分别泊松重建。

---

## 3. I/O 契约

外层负责解析与保存；本模块只吃内存对象、只吐内存对象。

| 方向 | 外层 | 本模块 |
|---|---|---|
| Mesh | 读 `scan.mesh.ply` / `.stl` → `trimesh.Trimesh` | 接收 `mesh` |
| Mask | 读 `scan.masks.yaml` → `dict` | 接收 `masks_data` |
| 路径 | 将返回值 `yaml.dump` 为 `scan.auto.path.yaml` | 返回与 raw 同构的 `dict` |

建议 API：

```python
class JeansAutoWaypoints:
    def __init__(self, ..., 见第 6 节构造参数):
        ...

    def plan(self, mesh: trimesh.Trimesh, masks_data: dict) -> dict:
        """返回 scan.auto.path.yaml 的内存结构，不落盘。"""
```

约定：

- `mesh.vertices` 在 **机器人 base 系，单位米**（与 `reconstruction_service` / `SurfaceZigzagSampler` 一致）。若外层读到毫米 mesh，先缩放再传入，或通过构造参数 `mesh_unit="mm"` 让模块换算。
- `masks_data` 为已解析的 YAML（`version` / `masks[].polygons`），多边形为图像像素坐标，与 `scan.jpg` 对齐。
- 不接收文件名、不接收原始字节。PLY/STL 解码、YAML `safe_load` 一律在外层。

---

## 4. 与现状的关键差异

```
现状（切半）                         本方案（整 mesh + 面标记）
─────────────                       ────────────────────────
2D mask ──切裆──► 两张 mask
两张 mask ──各自重建──► mesh_L, mesh_R     一张完整 mesh（不切开）
sampler(mesh_L) / sampler(mesh_R)         2D 切裆仅用于给顶点/面打标签
                                          左腿面子集 PCA 之字
                                          右腿面子集 PCA 之字
                                          重叠带航点去重
                                          抬枪 TCP + 法向平滑
```

「不切成两半」的含义：

- **保留** `split_jeans_mask` 的 2D 裆部分割（只用来分类像素 / 投影后的顶点）；
- **禁止** 对 mesh 做中线平面裁切、禁止按腿重新泊松；
- 每条腿采样时，只用「属于该腿的三角面」做 `mesh_plane` 求交。实现上可用 `trimesh.Trimesh` 的 **face 子集视图**（`submesh` / 临时 faces 拷贝），原 `mesh` 对象不被修改、不当成两份独立重建结果。

---

## 5. 流水线

```
masks_data                    mesh (base, m)
    │                              │
    ▼                              │
栅格化多边形 ──► 整裤 2D mask        │
    │                              │
    ▼                              │
split_jeans_mask               顶点投影到像素 (K, T)
 (1 或 2 张, 含 overlap 带)         │
    │                              ▼
    └────────► 顶点/面标签：left / right / overlap / none
                   │
         ┌─────────┴─────────┐
         ▼                   ▼
   左腿面子集             右腿面子集     （单腿则只走一侧）
   3D PCA 主轴            3D PCA 主轴
   平行切片之字            平行切片之字
   (行距/点距/is_jump)    同左
         │                   │
         └─────────┬─────────┘
                   ▼
            重叠区航点去重
                   ▼
         PathNormalSmoother
                   ▼
      沿法向抬 spray_dist_mm → TCP
      切向连续工具系 → RPY
                   ▼
         scan.auto.path.yaml 字典
```

### 5.1 栅格化掩码

复用 `InteractiveReconstructionService.rasterize_masks` 的语义（全部 `polygons` 填到一张二值图），但输入改为已解析的 `masks_data`，图像尺寸来自构造参数 `image_size=(width, height)`（模板默认 1280×800，与 `scan.params.yaml` 一致）。

### 5.2 一腿 / 两腿判定

直接调用现有 `split_jeans_mask(mask, overlap_px=0)`（只借 2D 切裆打标签）：

- 未检出显著裤裆 → 1 张 mask，整裤按一条腿 PCA 之字，**跳过去重**；
- 检出裤裆 → `[mask_left, mask_right]`，硬分割、无共享带。`overlap_px` 是旧「分腿重建补缝」参数，本算法不切 mesh，不使用。

此处 **只切 2D，不切 mesh**。

### 5.3 顶点 / 面标签（整 mesh 上分类）

将 base 系顶点变到相机系再投影：

\[
p_{\mathrm{cam}} = T_{\mathrm{base}\leftarrow\mathrm{cam}}^{-1}\, p_{\mathrm{base}},\quad
u = f_x x/z + c_x,\quad v = f_y y/z + c_y
\]

`T_camera_to_base`、`K` 由构造函数传入（与重建/手眼标定同一套，平移为米）。

标签规则：

| 投影像素落在 | 顶点标签 |
|---|---|
| 仅 left | `left` |
| 仅 right | `right` |
| left ∩ right | `overlap` |
| 都不在 / 背面 / z≤0 | `none`（不参与该腿采样） |

三角面标签：三个顶点的标签做并集。

- 采样左腿时使用面：`left ∪ overlap`（且不是纯 `right`）；
- 采样右腿时使用面：`right ∪ overlap`。

未提供 `K` / `T_camera_to_base` 时 `plan()` 直接失败，不做剪影降级。

### 5.4 单腿 PCA 之字（核心采样）

对「该腿面子集」的顶点做 3D PCA（与 `SurfaceZigzagSampler` / 2D `JeansZigzagSampler` 同思路，但在 3D）：

1. 主轴 \(v_{\mathrm{long}}\) = 最大特征值方向；符号约定与现网一致（例如 \(v_{\mathrm{long},y} \ge 0\)，避免整条腿方向翻转）。
2. 横向轴 \(v_{\mathrm{trans}} = \mathrm{normalize}(\hat{x} \times v_{\mathrm{long}})\)（\(\hat{x}\) 为相机深度/基座 X，与 conformal/surface sampler 相同）。若退化则改用次主成分。
3. 切片平面法向 = \(v_{\mathrm{trans}}\)，在该腿顶点投影范围内按 `row_spacing_mm` 等距切片（首条线偏置半个行距，避免贴边空喷）。
4. `trimesh.intersections.mesh_plane` **只打在该腿面子集上**，避免一刀同时切到另一条腿。
5. 交点沿 \(v_{\mathrm{long}}\) 排序，按弧长以 `point_spacing_mm` 插值（与 `conformal_sampler` 后半段相同）。
6. 行与行交替反向，形成之字；行首点打 `is_jump=True`。
7. 法向：整 mesh 顶点法向的 KDTree 最近邻（与 conformal 一致，不依赖 rtree）。

默认 `align_outer_edge=True`：在该腿面子集上拟合左右边，选更直的一条作为切片主轴（conformal 外侧缝语义），仍不切 mesh。关闭时退回纯 PCA 主轴。

### 5.5 重叠区去重

两腿都在 overlap 带上采点时，裆部会成对出现。去重在 **两条腿的航点列表都生成之后** 做，mesh 仍完整。

建议策略（稳定、可测）：

1. 右腿航点若到任一左腿航点距离 < `dedup_radius_mm`，则 **丢掉该右腿点**（先左后右）。不依赖 2D overlap 带。
3. 同一条腿内部若因切片重叠出现过近点，按弧长再滤一次（间距 < `0.4 * point_spacing_mm`）。
4. 去重后重排 `index`，保留 `is_jump`（若一行被删空则整行丢掉，下一行的 `is_jump` 仍表示换行）。

不在 3D 里再做一次几何切缝。

### 5.6 后处理（对齐 conformal 流水线）

与 `test_surface_zigzag.py` 在 conformal 之后的处理对齐：

1. `PathNormalSmoother(window_size=...)` 做路径序 1D 法向滑动平均并归一化；
2. 工具 Z = \(-\hat{n}\)，TCP 位置 = 表面点 \(+\) `spray_dist_mm` \(\times \hat{n}\)（毫米）；
3. 切向连续工具系（与 `ManualPathService.smooth_path_waypoints` 相同，避免欧拉 180° 翻转）；
4. 回填 `pixel`、`surface_point_cam_mm`、`surface_normal_cam`、`normal_2d_proj`（`K`/`T` 必填，见第 8 节）。

---

## 6. 构造参数

工艺与标定全部走构造函数，不从文件读。

| 参数 | 默认 | 含义 |
|---|---|---|
| `spray_dist_mm` | `150.0` | 喷高（沿表面法向抬枪，对应 YAML `standoff_distance_mm`） |
| `row_spacing_mm` | `60.0` | 之字行距（横向，宜接近喷幅） |
| `point_spacing_mm` | `100.0` | 沿裤腿方向点距 |
| `image_size` | `(1280, 800)` | 栅格化掩码的 (W, H) |
| `camera_intrinsics` | `None` | 3×3 `K`，两腿投影标签需要 |
| `T_camera_to_base` | `None` | 4×4，米；两腿投影标签需要 |
| `depth_threshold_ratio` | `0.1` | 交给 `split_jeans_mask` 的裤裆深度阈值（只用于判单腿/两腿） |
| `dedup_radius_mm` | 调用方传入 | 重叠去重半径（mm）。**不**按行距比例推算，必须由构造函数给出 |
| `normal_smooth_window` | `5` | `PathNormalSmoother` 窗口 |
| `mesh_unit` | `"m"` | `"m"` / `"mm"`，仅换算顶点，不改输入对象（内部拷贝） |
| `align_outer_edge` | `True` | 默认打开：单腿在 PCA 后再做 conformal 式外侧缝对齐 |

外层可从 `SprayerConfig.spray_distance`、`spray_width`、手眼标定、`scan.params.yaml` 的 `K` 填这些参数，模块本身不依赖 `SprayerConfig`。

---

## 7. 输出结构（对齐 `scan.raw.path.yaml`）

```yaml
paths:
  - path_id: 1
    name: Auto Path
    points:
      - index: 1
        pixel: [u, v]
        surface_point_cam_mm: [x, y, z]
        surface_point_base_mm: [x, y, z]
        surface_normal_base: [nx, ny, nz]
        surface_normal_cam: [nx, ny, nz]
        standoff_distance_mm: 150
        tcp_pose_base: {x, y, z, rx, ry, rz}   # mm / deg, euler xyz
        normal_2d_proj: [dx, dy]
        is_jump: false
        leg_id: 0          # 右腿为 1；腿间衔接点 is_jump: true
standoff_distance_mm: 150.0
type: auto
coordinate_frame: base_link
```

- 数值单位、字段名与 `ManualPathService` 写入的 raw 一致，便于 `path_opt_cli` / 交互页直接读。
- `type: auto` 区别于 `raw` / `opt` / `poi`；外层文件名约定 `scan.auto.path.yaml`。
- `template` / `updated_at` 由外层写入（模块不知道模板名）。
- `is_jump`、`leg_id` 为增量字段，旧读取端可忽略。

左右腿航点串成 **唯一一条** `path_id: 1`（`name: Auto Path`），腿间衔接点标 `is_jump: true`。

---

## 8. Review 结论（已拍板）

1. **一条路径**：左右腿之字串成同一条 `paths[0]`，不拆成两条。
2. **`align_outer_edge` 默认打开**：每条腿 PCA 之后再贴更直的外侧缝，作为切片主轴。
3. **没有相机 `K` / `T_camera_to_base` 时失败**：`plan()` 抛 `JeansAutoWaypointsError`，不做 YZ 剪影降级、不做整裤单 PCA 凑合。
4. **重叠去重半径由构造函数传入** `dedup_radius_mm`（单位 mm），不在模块内写死 `0.5 × 行距`。
5. **外层落点**仍不在本模块：读 ply + masks + 标定，调 `plan()`，写 `scan.auto.path.yaml`。

---

## 9. 模块边界与文件

| 文件 | 动作 |
|---|---|
| `app/src/core/vision/jeans_auto_waypoints.py` | **新建**：本模块唯一实现文件 |
| `app/src/core/vision/conformal_sampler.py` | 不改；切片/弧长/之字逻辑按需 **抄语义** 到新文件，避免反向依赖 |
| `app/src/core/vision/normal_smoother.py` | 复用 |
| `app/src/core/vision/image2d/jeans_segmentation.py` | 复用 `split_jeans_mask` |
| `app/docs/jeans_auto_waypoints_design.md` | 本文 |

单测（实现阶段再写）：假圆柱/两条斜圆柱拼成「假裤子」的 mesh + 合成多边形 mask，断言两腿主轴夹角、重叠点被删、输出含 `tcp_pose_base`。

---

## 10. 实现阶段建议顺序

1. 栅格化 + `split_jeans_mask` + 投影打标签（可用真实 `2026-08-14_151323` 的 mesh/mask 目视左/右/overlap）。
2. 单腿面子集 PCA 之字（先整裤单腿打通）。
3. 两腿 + 去重。
4. 法向平滑 + TCP/RPY，对齐 raw 字段。
5. 薄外层写 `scan.auto.path.yaml`（可选）。

---

## 11. 验收标准（实现后）

- 输入是 `Trimesh` + masks `dict`，输出是 `dict`，模块内无 `open` / 路径拼接。
- 原 `mesh.faces` 数量在 `plan()` 前后不变（不切半）。
- 两腿时各自 PCA 主轴明显不同（悬挂裤典型为两条近平行但位置分离的纵轴），之字行方向跟该腿主轴走，而不是整裤一条对角斜扫。
- 裆部 overlap 内不应出现成对的近距离重复航点。
- 输出可被现有 `path_opt_cli.py -f scan.auto.path.yaml` 加载（至少 `paths[].points[].tcp_pose_base` 齐全）。
