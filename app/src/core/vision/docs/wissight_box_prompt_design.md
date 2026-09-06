# Wissight 检测 → MobileSAM box 精修 设计方案

状态：**已实施完成**（板子实测见 §11，实施期新发现的 bug 见 §11.2）
需求：进入交互分割（点 MobileSAM 按钮）时，先用 wissight 按平台选后端（rknn/onnx/pt）识别物体
（类别由配置文件指定），拿到检测框后交给 MobileSAM 做 box prompt 精修，用户再在结果上点选微调。

---

## 1. 实测依据（Orange Pi 5 Plus / RK3588，`models/wissight.rknn`）

模型规格：YOLOv8-seg，输入 `images [1,3,640,640]`，输出 `output0 [1,49,8400]`
（4 box + 13 类概率 + 32 mask 系数）+ `output1 [1,32,160,160]`（protos）。
13 类为 DeepFashion2 服装：`short_sleeved_shirt, long_sleeved_shirt, short_sleeved_outwear,
long_sleeved_outwear, vest, sling, shorts, trousers, skirt, short_sleeved_dress,
long_sleeved_dress, vest_dress, sling_dress`。

### 1.1 耗时（640×640，单帧，中位数 / 最好 / 最差，5 次）

| 后端 | 耗时 | 结论 |
|---|---|---|
| RKNN NPU（rknnlite） | **122.3 / 120.6 / 128.2 ms** | 板上唯一可用选择 |
| ONNX Runtime CPU 多线程 | 495.3 / 488.0 / 508.4 ms | PC 开发机可用 |
| ONNX Runtime CPU 单线程 | 1726.0 / 1615.7 / 1794.6 ms | 不许限成单线程（见 §7.4） |

只在进入分割模式时跑一次，不进实时链路，122ms 完全可接受。

### 1.2 检出率（`data/template_group/` 全部 8 张 `scan.color.jpg`，1280×800）

| 模板 | conf=0.25 | conf=0.10 |
|---|---|---|
| 2026-08-14_151323 | trousers 0.93 | — |
| 2026-09-02_231440 | trousers 0.95 | — |
| 2026-09-03_011132 | trousers 0.96 | — |
| 2026-09-03_020429 | trousers 0.96 | + short_sleeved_shirt 0.12 |
| 2026-09-03_022722 | trousers 0.95 | — |
| 2026-09-03_225937 | trousers 0.96 | — |
| 2026-08-14_154353 | 无 | sling 0.23（误类） |
| 2026-08-25_215601 | 无 | 无（降到 0.01 仍 0 检出） |

→ 6/8 稳定检出，分数 0.93~0.96，与阈值 0.25 之间有巨大余量；**必须有"检不到就回退手动点选"的路径**；
**类别过滤是硬需求**（阈值一降就会混进 `sling`/`shirt`，选错框）。

### 1.3 box prompt 的分割质量（模板 2026-09-03_020429，真实生产 decoder）

| prompt | mask 面积 | 连通块数 | 最大块 |
|---|---|---|---|
| box only | **14.01 %** | **1** | 143433 |
| box + 1 个背景点 | 13.98 % | 2 | 143157 |
| box + 2 个背景点 | 13.98 % | 2 | 143165 |
| box + 1 个前景点 | 14.56 % | 2 | 149004 |
| 人工原始点集（1 前 1 背） | 11.83 % | **17** | 73830 |
| 只有背景点（无 box） | 1.11 % | 24 | 9551 |

box mask 与人工 mask 的 IoU = **0.844**（`multimask_output=False`）/ 0.852（`True`）。

三个结论，直接决定设计：

1. **box prompt 比人工点更干净**：1 个连通块 vs 人工 17 个碎片。上一轮 MobileSAM 修复里我列为
   "模型能力上限、不敢加后处理"的那个碎片化问题，靠 box prompt 天然解决，不需要动后处理。
2. **加背景点/前景点都是单调收敛的**（面积 14.01→13.98 / →14.56，连通块 1→2），不再有 §上一轮
   那种"点一下换个粒度"的跳变。所以 **box 必须一直留在 prompt 里参与精修**，而不是用完就丢。
3. **box-only 用 `multimask_output=False`（稳定头）即可**：与 `True` 差 0.008 IoU、连通块都是 1，
   取小者（少一个选择分支、与点精修路径一致）。

---

## 2. 交互流程

```
点 MobileSAM 按钮
  ├─ POST sam/init        编码图像（现状不变）
  ├─ POST detect           跑 wissight → 命中类别的框列表（conf/类别来自配置）
  │    ├─ 无结果 → toast「未检出目标，请手动点选」，走现有纯点流程
  │    └─ 有结果 → 取面积最大框 = segBox
  │                └─ POST sam/predict { box: segBox, points: [] } → 初始 polygons 直接显示
  ├─ 画布上以虚线画出 segBox（让用户知道结果从哪来）
  ├─ 左键加前景点 / 右键加背景点：predict { box: segBox, points: [...] }（现状逻辑，仅多带 box）
  ├─ 「清除自动框」按钮：segBox = null → 退回纯点选（防止框把用户困住）
  ├─ commit：MaskData 里连 box 一起存
  └─ save：box 落进 scan.masks.yaml，重放时用同一 box
```

自动结果**不直接 commit**，只作为可编辑初稿：实测人工标的是局部（`mask#1` 的两个点不在 trousers 框
内），检测给的是整体，用户往往还要再点背景切掉一条腿 —— 直接 commit 会逼用户删掉重来。

---

## 3. 文件清单

**新增**

| 文件 | 内容 | 行数 |
|---|---|---|
| `app/src/core/vision/backend.py` | 跨平台后端选择 + RKNN 运行时加载 + logging 污染修复（§4） | ~80 |
| `app/src/core/vision/image2d/wissight_detector.py` | letterbox 前处理、`output0` 解码、NMS、类别/置信过滤（§5） | ~130 |
| `app/src/core/vision/image2d/test_wissight_postprocess.py` | 无硬件单测：坐标反映射、box→prompt 映射、类别名解析 | ~90 |

**修改**

| 文件 | 内容 |
|---|---|
| `mobilesam_session.py` | 删掉本地 5 个后端 helper 改 import `backend.py`；`_run_onnx_decoder`/两个 predictor 的 `predict` 加回 `box`；`MobileSAMSession.predict` 支持 box-only |
| `sam_service.py` | 新增 `detect_action()`；`predict_action`/`save_masks` 接受并透传 `box`；修 polygons-only 不落可视化 |
| `apps/interactive/api.py` | 新增 `POST /templates/{name}/detect`；`PredictRequest.box`；save 透传 `box` |
| `configs/aisprayer_config.yaml` | 新增 `interactive.detector` 段 |
| `frontend/.../InteractiveOp.tsx` | `segBox` state、init 后自动 detect、predict 带 box、清框按钮、commit 带 box |
| `frontend/.../SamMaskOverlay.tsx` | 虚线画检测框 |

---

## 4. `backend.py`：把跨平台后端选择抽成公共实现

现在 `mobilesam_session.py` 里的 `is_rk3588()` / `resolve_device()` / `_model_ready()` / `_size_mb()` /
`_init_rknn()` 就是本仓库的第二份"按平台挑后端"逻辑要做的事，检测器必然要重复一遍 —— 所以先抽出来：

```python
def is_rk3588() -> bool
def model_ready(path) -> bool          # 存在且 >1KB（挡 Git LFS 指针文件）
def size_mb(path) -> str
def resolve_backend(requested, env_names, cfg_value, has_rknn, has_onnx, torch_device) -> str
def load_rknn_runtime(model_path, core_mask="auto") -> rknn | None
```

`resolve_backend` 参数化 env 名与 config 值，避免检测器去读 SAM 的 `MOBILESAM_DEVICE` /
`interactive.sam.backend`。

### 4.1 必须先修的阻断性 bug：rknnlite 污染 `logging` 全局等级表

实测（板上，`rknn-toolkit-lite2 2.3.2`）：

```python
import logging; before = dict(logging._nameToLevel)
from rknnlite.api import RKNNLite
after = dict(logging._nameToLevel)
# before['WARNING'] = 30 -> after['WARNING'] = None
# 被删掉的键： CRITICAL DEBUG ERROR FATAL INFO NOTSET WARN WARNING
# 新增的键：   C D E I W
```

它把标准等级名换成了单字母。后果：**任何在它之后首次 `import torch` 的代码都会炸** ——

```
ValueError: Unknown level: 'WARNING'      # torch/fx/passes/utils/matcher_utils.py: logger.setLevel("WARNING")
```

（`import torch` 在前 → 正常；`from rknn.api import RKNN` 同理。）

为什么现在是安全的：`mobilesam_session.py` 在**模块顶层**就 import torch，RKNNLite 只在 `_init_rknn()`
里延迟 import，所以 torch 总是先落地。**这个新功能会把顺序反过来**：若 `api.py` 顶部先 import 检测器、
检测器又顶层 import RKNNLite，则随后 `mobilesam_session` 的 `import torch` 抛的是 **ValueError，
不是 ImportError，`try/except ImportError` 抓不住 → 后端启动直接崩溃**。

修法（`load_rknn_runtime` 内，约 6 行）：import 前快照 `logging._nameToLevel`，import 后只把
**被删掉的标准键**补回去（不覆盖 rknn 新增的），`mobilesam_session._init_rknn` 一起换用该 helper。

---

## 5. `wissight_detector.py`

```python
CLASS_NAMES = {0: "short_sleeved_shirt", ..., 7: "trousers", ...}   # 与导出模型一致，13 项

class WissightDetector:
    def __init__(self, backend="auto", classes=("trousers",), conf=0.25, iou=0.7, max_boxes=5)
    def detect(self, image_bgr) -> list[Detection]   # Detection: {box xyxy(原图), cls_id, cls_name, score}
```

- **前处理**：`r = min(640/w, 640/h)`，缩放后 **居中** pad 到 640×640（pad 值 114），`/255` → float32。
  注意与 SAM 的 `_resize_and_pad`（右下补 0、不做 `/255`）**不是一回事**，不合并，各自注释说明。
- **输入张量布局**：ONNX 要 NCHW，RKNN 要 NHWC。**RKNN 版必须自己 `/255`**：
  `convert_wissight_to_rknn.py` 的 `rknn.config()` 只给了 `optimization_level=3`，**没有 mean/std**，
  归一化不在模型里（MobileSAM encoder 那次转换是给了 mean/std 的）—— 两者不一样，别照抄。
- **后处理**：`output0` → `(8400,49)`，`xywh`(640 空间) → `xyxy` → 减 pad、除 `r` → clip 到原图 →
  类别概率取 max 过 `conf` → `cv2.dnn.NMSBoxes`（不手写 NMS）→ 按类别白名单过滤 → 面积降序。
  只请求 `output0`，不取 `output1`（protos）—— 本功能只要框，省掉 32×160×160 的 mask 合成。
- 线程：ONNX 后端**不要**限 `intra_op_num_threads=1`（§1.1 实测 1.7s）；板上走 RKNN 不涉及。

---

## 6. 配置项（`configs/aisprayer_config.yaml`）

```yaml
interactive:
  sam:
    backend: "auto"
  detector:                     # 进入分割模式时用 wissight 出框，给 MobileSAM 当初始 prompt
    enabled: true               # false = 完全跳过检测，行为与现在一致
    backend: "auto"             # auto | rknn | onnx | pt
    classes: ["trousers"]       # 可选类别名（13 类见 §1.2）；[] = 不过滤
    conf: 0.25                  # 依据 §1.2：0.25 干净，降到 0.1 会进误类
    iou: 0.7                    # NMS IoU
    max_boxes: 5                # 只影响接口返回，UI 仍只用最大框
```

模型路径不给配置项，按后端固定：`models/wissight.rknn` / `wissight.onnx` / `wissight.pt`
（三个都在，`.rknn` 是符号链接），与 MobileSAM 现有做法一致。类别名写错时启动即报错并列出合法值。

---

## 7. 接口与数据契约

```
POST /templates/{name}/detect      → {detections:[{cls, score, box:[x1,y1,x2,y2]}], backend, elapsed_ms}
POST /templates/{name}/sam/predict  body 新增可选 "box": [x1,y1,x2,y2]；points 可为 []
POST /templates/{name}/sam/save     committed_masks[i] 新增可选 "box"
```

`scan.masks.yaml` 每个 mask 多一个可选键（**老文件没有该键 = None，完全向后兼容**）：

```yaml
- id: 1
  box: [320.9, 113.0, 731.1, 627.0]
  points: [[448, 319], [538, 111]]
  labels: [1, 0]
  score: 0.802
  polygons: [...]
```

三维重建侧（`reconstruction_service.rasterize_masks` / C++ `MaskSet::loadYaml`）只读 `polygons`，
**不需要改 C++**。

### 7.1 `save_masks` 会用 prompt 重跑，所以 box 必须一起存

`sam_service.save_masks()` 不是把页面上的 polygons 原样落盘，而是拿 prompt 在同一张图上
**重新推理一遍**（`session.predict(...)`）再写 polygons。因此自动框若不进 yaml，
就会出现"页面上看到的 mask 和落盘的 mask 不一致"。这是本功能必须持久化 box 的唯一原因。

### 7.2 decoder 侧要加回 box（上一轮删掉的参数，这次是真需要）

`_run_onnx_decoder` 按官方 `SamPredictor.predict` 的约定处理：box 转成 2 个点
`[[x1,y1],[x2,y2]]` + `labels=[2,3]`（2=左上、3=右下），拼在用户点之前；坐标缩放沿用现有的
`*new/orig`（box 与点同为原图坐标，`_embed_points` 内部再 `/img_size`）。

### 7.3 顺手修：`MobileSAMSession.predict` 目前拒绝空点集

`if not self.predictor or not points: return None` —— box-only prompt 的 `points` 就是空的，
会被这一行直接吞掉。要改成 `points` 与 `box` 至少有一个即可。

### 7.4 顺手修：`multimask` 策略没考虑 box

现在写死 `multimask = len(points) == 1`。按 §1.3，**有 box 时必须走稳定头**（`False`），
否则 box-only 会在 3 个粒度间跳。规则改为：
`multimask = (box is None) and len(points) == 1`。

### 7.5 顺手修：无点只有 polygons 的 mask 不进可视化

`save_masks()` 里 `if not pts:` 的分支只写 yaml、`continue`，不参与 `vis_image` 着色
（`scan.masks.jpg` 会缺这一块）。改为用 polygons `fillPoly` 上色。

---

## 8. 非目标（这次不做，避免范围外溢）

- **不做 YOLO 实例 mask 解码**（`output1` protos + 32 系数）：只要框，SAM 出 mask。
- **不整合 `YoloTrousersSegmenter`**（`image2d/segmenter.py`，依赖 ultralytics + `.pt`）：
  它是离线 PC 工具链（`vision_processor` / `reconstruction` CLI / `visualize`）在用，
  板上有 torch-slim/无 ultralytics 时本来就跑不了。留着，不与新检测器合并。
- 不改三维重建的孔洞填充约定（上一轮记录的那个独立问题）。
- 不在实时链路（跟随/喷涂）里调用检测。

---

## 9. 实施顺序与验收

| 步 | 内容 | 验收 |
|---|---|---|
| 1 | `backend.py` + logging 修复，`mobilesam_session.py` 改 import | 现有 `test_sam_decoder_slice`(7)/`test_mobilesam` 全绿；`app/.venv/bin/python -c "from rknnlite.api import RKNNLite; import torch"` 不再抛 ValueError；后端启动日志仍为 `ACTIVE BACKEND: RKNN ... + ONNX Decoder` |
| 2 | `wissight_detector.py` + 无硬件单测 | 8 张模板图上结果与 §1.2 一致（6 检出 / 2 无）；RKNN 与 ONNX 两后端同图框位差 ≤2px |
| 3 | SAM box prompt（decoder + session + service） | 2026-09-03_020429 上 box-only → 面积 ≈14.0%、连通块 1；加 1 背景点 → 面积 ≤14.0%（单调）；存盘 yaml 带 box 且重放 polygons 与页面一致 |
| 4 | api + 前端 | 点 MobileSAM 按钮后无需点击即出现初始 mask；检不到时提示并保留纯手动；虚线框可见；老模板（无 box 的 yaml）能正常回显与再保存 |

工作量估计：1 天（含验证）。

---

## 10. 已拍板的 4 件事（按方案建议执行）

1. **自动结果不直接 commit**，只作可编辑初稿：人工标的 mask 是局部的（实测人工 2 点 → 11.8%
   面积），而自动框给的是整块（14.0%），直接 commit 会把用户不想喷的区域一并圈进去。
2. **多框只取面积最大的一个**，其余仅在 `/detect` 返回体里；8 张现网图全部单框，不做“点框切换”。
3. **默认白名单 `["trousers"]`**：现场只喷裤子；换成上衣类只需改配置里的 `classes`，
   写错名字会在加载时 `ValueError`，不会静默漏检。
4. **`enabled` 默认 true**：进入分割模式多 122ms（一次性），检不到自动回退手动。

---

## 11. 实施结果（Orange Pi 5 Plus / RK3588 实测）

### 11.1 数据链路（与 §1 探针一致，但跑的是生产代码）

| 环节 | 结果 |
|---|---|
| `WissightDetector` 单例 + RKNN | `ACTIVE BACKEND: RKNN (RK3588 NPU) \| wissight.rknn (25.0 MB)` |
| `detect()`（1280×800） | 137.6 ms，`trousers 0.956 box=[320.9, 113.0, 731.1, 627.0]`（与探针逐字一致） |
| box-only → MobileSAM | 1 个连通块，score 0.979 |
| box + 1 背景点 | 2 个连通块，score 0.982（与 §1.3 单调性结论一致） |
| `save_masks` 重放幂等 | `E2E replay identical: True`（从 yaml 回读 box 再存一次，polygons 完全相等） |
| 后端启动链冒烟 | `import main` + OpenAPI 生成 OK，`PredictRequest` 字段 = `box/labels/points` |
| 单测 | `test_wissight_postprocess` 14 项 + `test_sam_decoder_slice` 13 项全绿（无模型、无 NPU） |
| logging 污染修复 | `load_rknn_runtime()` 后 `logging._nameToLevel['WARNING'] == 30`，再 `import torch` 正常 |

### 11.2 实施期新发现并修掉的 bug

1. **`_decode` 里的 `np.clip(..., out=boxes[:, [0, 2]])` 是个空操作** —— `boxes[:, [0, 2]]`
   是 fancy index 出来的副本，写回副本改不到原数组，结果框会溢出到图像外（实测
   `(-380,-620,420,180)` 完全不截断）。改成 `boxes[:, 0::2] = boxes[:, 0::2].clip(...)`，
   并补了截断回归用例。这个不爆异常，只会让边缘目标当 box prompt 时把图外区域一并纳入提示。
2. **letterbox 输出不连续** —— `astype(np.float32)` 默认 `order="K"`，会保留 `[::-1]` 的反向步长，
   `blob.flags["C_CONTIGUOUS"]` 为 False。改 `astype(..., order="C")`，ORT 侧避免隐式拷贝。
3. **`save_masks` 的 polygons-only 分支不画可视化**（§7.5）—— 现在走 `_paint_polygons`，
   实测 `scan.masks.jpg` 被改动 57934 px（修前为 0）。
4. **RKNN/ONNX 加载与后端选择在 `mobilesam_session.py` 里写了一份** —— 上提到 `core/vision/backend.py`
   共用（§4），同时把 `rknnlite` 污染 `logging` 的坑一并修在那里。
5. **死代码清理**：`mobilesam_session` 里未使用的 `ResizeLongestSide` import（RKNN torch 回退分支）、
   `api.py` 里未使用的 `BackgroundTasks`/`List`；定义好但始终没被人用上的 `ToolbarTip`
   组件这次正式接入，SAM 工具条 6 处内联 tooltip 样板（共 30 行）收敛为 6 行。
6. **新模型同类坑的预防**：给 `_decode` 加了 output0 通道宽度契约（49）。否则换成 80 类
   COCO 导出时，前 13 个通道会被当作本项目的 13 个服装类静默错用 —— 与上一轮 MobileSAM
   decoder 图签名坑属同一类，已补单测 `test_wider_graph_with_other_class_count_is_rejected`。

### 11.3 改动清单

| 文件 | 内容 |
|---|---|
| `core/vision/backend.py` | **新增**：`is_rk3588` / `model_ready` / `size_mb` / `pick_backend` / `torch_device` / `load_rknn_runtime`（含 logging 修复） |
| `core/vision/image2d/wissight_detector.py` | **新增**：三后端检测器，只做 box（不解码实例 mask）+ 单例 `get_detector()` |
| `core/vision/image2d/test_wissight_postprocess.py` | **新增**：14 项后处理回归（无需模型/NPU） |
| `core/vision/image2d/mobilesam_session.py` | 改 import 公共底座；删本地重复实现与死 import；`MobileSAMSession.predict` 支持 box |
| `apps/interactive/sam_service.py` | 新增 `detect()`；`predict_action` 透传 box；`save_masks` 持久化+重放 box、修 polygons-only 可视化 |
| `apps/interactive/api.py` | 新增 `POST /templates/{name}/detect`；`PredictRequest.box` |
| `core/config.py` + `configs/aisprayer_config.yaml` | `interactive.detector.{enabled,backend,classes,conf,iou,max_boxes}` |
| `InteractiveOp.tsx` / `InteractiveCanvas.tsx` / `SamMaskOverlay.tsx` / `types.ts` | 进分割模式自动检测 → 可编辑初稿 → 带框精修 → 可一键重新检测/清除框；box 随 mask 提交 |

### 11.4 待你验证的部分

后端逻辑已全部在板子上实测，但**浏览器里的交互还需你自己跑**：启动后端与前端后点
 MobileSAM 按钮，应看到“自动出框（黄色虚线）+ 蓝色初稿”，然后左/右键精修。
工具条新增两个按钮：`Detect Box Prompt`（重新检测）与 `Remove Detected Box`（只删提示、保留已点的点）。

---

## 12. 追加两项（首次上板后提出）

### 12.1 启动日志看不到 wissight 加载

根因：检测器是进程内懒加载单例，只有第一次调 `/detect` 才构造，而 `main` 启动时
只预热了 MobileSAM —— 建会话的 1~2s 算在用户第一次点按钮头上，启动日志里又什么都没有。

修法：`SAMService.initialize()` 末尾调 `_warm_up_detector()`，与 MobileSAM 同处预热并打日志。
`get_detector()` 也顺手改为**先看 `enabled` 再构造**（之前配置关了也会先把权重加载一遍再丢弃）。

实测启动日志：

```
[Detect] >>> ACTIVE BACKEND: RKNN (RK3588 NPU) | wissight.rknn (25.0 MB) <<<
[Detect] Wissight detector initialized successfully in 0.14s [...], sam_refine=True.
```

### 12.2 `interactive.detector.sam_refine` 开关

关掉后不进 MobileSAM，直接用 wissight 自己的实例 mask。为此检测器必须真的解码
`coeffs × protos`（之前刻意只做 box）：

| 位置 | 内容 |
|---|---|
| `wissight_detector.py` | `detect(with_masks=)`；输出**按形状认**（3 维 = 检测头、4 维 = protos，不依赖导出顺序）；`_instance_masks()` 做 einsum + sigmoid + 按框裁剪 + 最近邻上采样贴回原图 |
| `config.py` / yaml | `sam_refine: true`（默认） |
| `sam_service.detect()` | 关精修时直接回传 `polygons` + `score`，回传体带 `sam_refine` 告知前端 |
| `sam_service.save_masks()` | 重放条件从「无点且无框」改成**「无点就不重跑」**：box-only 的 SAM 结果本身幂等（实测重放完全一致），而 detector mask 重跑反而会被换成另一个结果 —— 顺带把原来的 polygons-only 分支合并进来，少一个分支、存盘时少一次推理 |
| `InteractiveOp.tsx` | `sam_refine === false` 且有 polygons 时直接当可编辑初稿，不调 `/sam/predict`；检到框但 mask 无轮廓时自动回退到精修路径 |

实测（同一张 1280×800 模板）：

| 路径 | 耗时 | 结果 |
|---|---|---|
| 开精修（box → MobileSAM） | 135ms + decoder | 1 polygon，score 0.979，覆盖 14.0% |
| 关精修（直接用实例 mask） | 150ms | 1 polygon，score 0.956，覆盖 13.5% |

两者覆盖只差 0.5 个点，说明关掉也能用；代价是 mask 只有 160×160 原型分辨率，边缘会有明显的块状感觉。

新补 4 个单测钉住 mask 解码（均匀 protos 填满框且框外零像素、空间模式按 1/4 比例对齐、
protos 通道数不匹配时降级而不是崩、默认不算 mask）—— 写的时候就靠它们抓出了
`einsum` 下标错一维这个必崩的 bug。

