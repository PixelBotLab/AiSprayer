"""Wissight（YOLOv8-seg）目标检测：给交互分割提供自动提示。

两种用法，由 `interactive.detector.sam_refine` 开关决定：
- 开精修（默认）：只给框，mask 交 MobileSAM 精修 —— 实测框出 1 个连通块，
  人工点选同样目标会碎成十几个。
- 关精修：直接解码 YOLOv8-seg 的实例 mask（coeffs × protos）当结果，不进 MobileSAM。
与 image2d/segmenter.py 里的 YoloTrousersSegmenter 的区别：那个给离线点云/重建链路
产出同分辨率 mask，本模块只给交互界面产出提示。

模型按后端自动选择：RK3588 走 wissight.rknn（NPU，实测 122ms），其他平台走
wissight.onnx（ORT CPU，实测 495ms；限成单线程要 1.7s，所以不限制），
只有 .pt 权重时才退回 ultralytics。
"""
import logging
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from core.config import sprayer_config
from core.vision.backend import (
    MODELS_DIR,
    is_rk3588,
    load_rknn_runtime,
    model_ready,
    pick_backend,
    size_mb,
)

logger = logging.getLogger(__name__)

IMGSZ = 640
PAD_VALUE = 114  # ultralytics letterbox 的填充色，改了会和训练时不一致

DEFAULT_WISSIGHT_RKNN = MODELS_DIR / "wissight.rknn"
DEFAULT_WISSIGHT_ONNX = MODELS_DIR / "wissight.onnx"
DEFAULT_WISSIGHT_PT = MODELS_DIR / "wissight.pt"

# 与 models/wissight.onnx 里的 metadata `names` 一致（DeepFashion2 服装类别），
# 顺序即类别 id，不要改动。
CLASS_NAMES = (
    "short_sleeved_shirt",
    "long_sleeved_shirt",
    "short_sleeved_outwear",
    "long_sleeved_outwear",
    "vest",
    "sling",
    "shorts",
    "trousers",
    "skirt",
    "short_sleeved_dress",
    "long_sleeved_dress",
    "vest_dress",
    "sling_dress",
)

# output0 的通道排布：4 box(xywh) + N 类概率 + 32 mask 系数
NUM_BOX_CHANNELS = 4
NUM_MASK_COEFFS = 32
EXPECTED_OUT_CHANNELS = NUM_BOX_CHANNELS + len(CLASS_NAMES) + NUM_MASK_COEFFS


@dataclass
class Detection:
    """一个检出目标，box 为原图像素坐标 (x1, y1, x2, y2)。"""

    box: Tuple[float, float, float, float]
    cls_id: int
    cls_name: str
    score: float
    # 原图尺寸的 uint8 二值 mask（0/255）；仅 detect(with_masks=True) 时填
    mask: Optional[np.ndarray] = None

    @property
    def area(self) -> float:
        return max(0.0, self.box[2] - self.box[0]) * max(0.0, self.box[3] - self.box[1])

    @property
    def center(self) -> Tuple[int, int]:
        return (int((self.box[0] + self.box[2]) / 2), int((self.box[1] + self.box[3]) / 2))

    def to_dict(self) -> dict:
        # mask 不进回传体：它是 ndarray，界面只需要框（或已经转成 polygons 的结果）
        return {
            "box": [round(v, 1) for v in self.box],
            "cls_id": self.cls_id,
            "cls": self.cls_name,
            "score": round(self.score, 3),
        }


def resolve_classes(names: Optional[Sequence[str]]) -> Optional[set]:
    """把类别名解析成 id 集合；空 = 不过滤，未知名字直接报错（配置写错要响，不要静默漏检）。"""
    if not names:
        return None
    unknown = [n for n in names if n not in CLASS_NAMES]
    if unknown:
        raise ValueError(f"Unknown detector class name(s) {unknown}; valid: {list(CLASS_NAMES)}")
    return {CLASS_NAMES.index(n) for n in names}


class WissightDetector:
    """加载 wissight 并对任意 BGR 图像出框。"""

    def __init__(
        self,
        backend: Optional[str] = None,
        classes: Optional[Sequence[str]] = None,
        conf: Optional[float] = None,
        iou: Optional[float] = None,
        max_boxes: Optional[int] = None,
    ):
        self.conf = float(sprayer_config.detector_conf if conf is None else conf)
        self.iou = float(sprayer_config.detector_iou if iou is None else iou)
        self.max_boxes = int(sprayer_config.detector_max_boxes if max_boxes is None else max_boxes)
        self.class_ids = resolve_classes(
            sprayer_config.detector_classes if classes is None else classes
        )

        self.backend_type = self._resolve_backend(backend)
        self.engine = None
        self.backend_desc = "disabled"
        self._input_name = None
        self._load()

    # ── 后端选择与加载 ────────────────────────────────────────────────
    def _resolve_backend(self, requested: Optional[str]) -> str:
        return pick_backend(
            requested,
            ("WISSIGHT_DEVICE", "WISSIGHT_BACKEND"),
            sprayer_config.detector_backend,
            self._auto_backend,
        )

    @staticmethod
    def _auto_backend() -> str:
        if is_rk3588() and model_ready(DEFAULT_WISSIGHT_RKNN):
            return "rknn"
        if model_ready(DEFAULT_WISSIGHT_ONNX):
            try:
                import onnxruntime  # noqa: F401
                return "onnx"
            except ImportError:
                pass
        if model_ready(DEFAULT_WISSIGHT_PT):
            return "pt"
        return "none"

    def _load(self) -> None:
        models = {
            "rknn": DEFAULT_WISSIGHT_RKNN,
            "onnx": DEFAULT_WISSIGHT_ONNX,
            "pt": DEFAULT_WISSIGHT_PT,
        }
        path = models.get(self.backend_type)
        if path is None or not model_ready(path):
            logger.warning(
                f"[Detect] No usable wissight model for backend '{self.backend_type}' "
                f"(checked wissight.rknn / .onnx / .pt under {MODELS_DIR}); auto-detection disabled, "
                f"interactive segmentation stays manual."
            )
            self.backend_type = "none"
            return

        try:
            if self.backend_type == "rknn":
                # 注意：这份模型的 RKNN 转换没内置 mean/std（与 MobileSAM encoder 不同），
                # 归一化必须在 Python 侧做，见 _letterbox。
                self.engine = load_rknn_runtime(path)
                label = "RKNN (RK3588 NPU)"
            elif self.backend_type == "onnx":
                import onnxruntime as ort

                self.engine = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
                self._input_name = self.engine.get_inputs()[0].name
                label = "ONNX Runtime (CPU)"
            else:
                from ultralytics import YOLO

                self.engine = YOLO(str(path))
                label = "Ultralytics PyTorch"
        except Exception as e:
            logger.error(f"[Detect] Failed to load {path.name} via '{self.backend_type}': {e}")
            self.engine = None
            self.backend_type = "none"
            return

        self.backend_desc = f"{label} | {path.name} ({size_mb(path)})"
        logger.info(f"[Detect] >>> ACTIVE BACKEND: {self.backend_desc} <<<")

    @property
    def available(self) -> bool:
        return self.engine is not None

    # ── 推理 ─────────────────────────────────────────────────────────
    def detect(self, image_bgr: np.ndarray, with_masks: bool = False) -> List[Detection]:
        """返回按面积降序的检测框；with_masks=True 时同时解出实例 mask。"""
        if not self.available:
            return []
        t0 = cv2.getTickCount()
        if self.backend_type == "pt":
            dets = self._detect_torch(image_bgr, with_masks)
        else:
            dets = self._detect_graph(image_bgr, with_masks)
        cost_ms = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000.0
        logger.info(
            f"[Detect] {image_bgr.shape[1]}x{image_bgr.shape[0]} -> {len(dets)} box(s) "
            f"[{', '.join(d.cls_name + ':' + format(d.score, '.2f') for d in dets) or '-'}] "
            f"in {cost_ms:.1f}ms"
        )
        return dets

    def _detect_graph(self, image_bgr: np.ndarray, with_masks: bool) -> List[Detection]:
        """RKNN / ONNX：跑静态图，再自己做解码 + NMS（+ 可选 mask 解码）。"""
        blob, scale, pad_x, pad_y = self._letterbox(image_bgr)
        if self.backend_type == "rknn":
            # NHWC float32，值域 0..1（归一化没做进模型）
            outs = self.engine.inference(
                inputs=[np.ascontiguousarray(blob.transpose(0, 2, 3, 1))])
        else:
            outs = self.engine.run(None, {self._input_name: blob})
        # 按形状而非位置认输出：3 维是检测头，4 维是 protos（导出顺序一变不会静默错用）
        shapes = [np.asarray(o).shape for o in outs]
        protos = None
        head = next((o for o, s in zip(outs, shapes) if len(s) == 3), None)
        if head is None:
            logger.warning(f"[Detect] unexpected model outputs: {shapes}")
            return []
        if with_masks:
            protos = next((np.asarray(o) for o, s in zip(outs, shapes) if len(s) == 4), None)
            if protos is None:
                logger.warning(f"[Detect] no proto output (4D) in {shapes}; instance masks unavailable")
        dets = self._decode(head, scale, pad_x, pad_y, image_bgr.shape, self.conf, protos)
        return self._finalize(dets)

    def _detect_torch(self, image_bgr: np.ndarray, with_masks: bool) -> List[Detection]:
        """ultralytics 自己完成前处理与解码，这里只把结果映射成同一套 Detection。"""
        results = self.engine.predict(image_bgr, conf=self.conf, verbose=False)
        boxes = results[0].boxes if results else None
        if boxes is None or len(boxes) == 0:
            return []
        xyxy = boxes.xyxy.cpu().numpy()
        scores = boxes.conf.cpu().numpy()
        cls_ids = boxes.cls.cpu().numpy().astype(int)
        masks = results[0].masks.data.cpu().numpy() if (with_masks and results[0].masks is not None) else None
        dets = []
        for i in range(len(xyxy)):
            mask = None
            if masks is not None and i < len(masks):
                mask = (masks[i] > 0.5).astype(np.uint8) * 255
                h, w = image_bgr.shape[:2]
                if mask.shape != (h, w):
                    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
            dets.append(Detection(
                box=(float(xyxy[i, 0]), float(xyxy[i, 1]), float(xyxy[i, 2]), float(xyxy[i, 3])),
                cls_id=int(cls_ids[i]),
                cls_name=CLASS_NAMES[cls_ids[i]] if cls_ids[i] < len(CLASS_NAMES) else f"cls_{cls_ids[i]}",
                score=float(scores[i]),
                mask=mask,
            ))
        return self._finalize(dets)

    @staticmethod
    def _letterbox(image_bgr: np.ndarray) -> Tuple[np.ndarray, float, float, float]:
        """等比缩放到 IMGSZ 内、居中补 PAD_VALUE，返回 (NCHW float32[0..1], scale, pad_x, pad_y)。"""
        h, w = image_bgr.shape[:2]
        scale = min(IMGSZ / w, IMGSZ / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        pad_x, pad_y = (IMGSZ - nw) / 2.0, (IMGSZ - nh) / 2.0
        canvas = np.full((IMGSZ, IMGSZ, 3), PAD_VALUE, np.uint8)
        canvas[int(pad_y):int(pad_y) + nh, int(pad_x):int(pad_x) + nw] = cv2.resize(image_bgr, (nw, nh))
        # order="C"：ORT 与 RKNN 都要连续内存，默认的 order="K" 会保留 ::-1 反向步长
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32, order="C") / 255.0
        return blob, scale, pad_x, pad_y

    @staticmethod
    def _decode(
        output0: np.ndarray, scale: float, pad_x: float, pad_y: float, img_shape, conf: float,
        protos: Optional[np.ndarray] = None,
    ) -> List[Detection]:
        """(1, 4+nc+32, 8400) -> 原图坐标系下过阈值的候选框（给 protos 则附带实例 mask）。"""
        pred = np.squeeze(np.asarray(output0)).T  # (8400, 4+nc+32)
        # 静态 shape 导出，通道宽度就是契约。类别数与 CLASS_NAMES 不一致时，前 13 个
        # 通道会被当成对应类别静默错用（检得不准但也不报错），所以宽度不对直接拒。
        if pred.shape[1] != EXPECTED_OUT_CHANNELS:
            logger.warning(
                f"[Detect] output0 has {pred.shape[1]} channels, expected {EXPECTED_OUT_CHANNELS} "
                f"(= {NUM_BOX_CHANNELS} box + {len(CLASS_NAMES)} classes + {NUM_MASK_COEFFS} mask coeffs); "
                f"model and CLASS_NAMES are out of sync."
            )
            return []
        cls_scores = pred[:, NUM_BOX_CHANNELS:NUM_BOX_CHANNELS + len(CLASS_NAMES)]
        scores = cls_scores.max(axis=1)
        keep = np.flatnonzero(scores > conf)
        if keep.size == 0:
            return []
        pred, scores, cls_ids = pred[keep], scores[keep], cls_scores[keep].argmax(axis=1)

        xywh = pred[:, :NUM_BOX_CHANNELS]
        x1 = xywh[:, 0] - xywh[:, 2] / 2.0
        y1 = xywh[:, 1] - xywh[:, 3] / 2.0
        boxes = np.column_stack([x1, y1, x1 + xywh[:, 2], y1 + xywh[:, 3]])
        boxes640 = boxes.copy()  # letterbox 坐标，protos 裁剪要用，必须在逆变换前拿
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_x) / scale
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_y) / scale
        # 必须整体赋值：`boxes[:, [0, 2]]` 是 fancy index 出来的副本，
        # 在它上面 clip(out=...) 改不到原数组，框会溢出到图像外。
        boxes[:, 0::2] = boxes[:, 0::2].clip(0, img_shape[1])
        boxes[:, 1::2] = boxes[:, 1::2].clip(0, img_shape[0])

        masks = (
            WissightDetector._instance_masks(pred[:, -NUM_MASK_COEFFS:], protos, boxes640, boxes, img_shape)
            if protos is not None else [None] * len(scores)
        )
        return [
            Detection(
                box=(float(boxes[i, 0]), float(boxes[i, 1]), float(boxes[i, 2]), float(boxes[i, 3])),
                cls_id=int(cls_ids[i]),
                cls_name=CLASS_NAMES[int(cls_ids[i])],
                score=float(scores[i]),
                mask=masks[i],
            )
            for i in range(len(scores))
        ]

    @staticmethod
    def _instance_masks(
        coeffs: np.ndarray, protos: np.ndarray, boxes640: np.ndarray,
        boxes: np.ndarray, img_shape,
    ) -> List[Optional[np.ndarray]]:
        """YOLOv8-seg 实例 mask：coeffs × protos → sigmoid → 按框裁剪 → 映射回原图。

        protos 是 1/4 网络输入分辨率（640 → 160），而实例 mask 只在其 own 的 box 内
        有效，所以先裁 box 区域再按原图 box 尺寸最近邻上采样贴回，与 ultralytics 一致。
        """
        proto = np.squeeze(np.asarray(protos))  # (32, ph, pw)
        if proto.ndim != 3 or proto.shape[0] != coeffs.shape[1]:
            logger.warning(
                f"[Detect] protos shape {proto.shape} incompatible with {coeffs.shape[1]} coeffs; "
                f"instance masks unavailable."
            )
            return [None] * len(coeffs)
        ph, pw = proto.shape[1], proto.shape[2]
        # 一次算完所有实例，再逐个裁剪；sigmoid 先 clip 指数避免 overflow
        logits = np.einsum("nc,ckw->nkw", coeffs.astype(np.float32), proto)
        masks = 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))
        ratio = pw / float(IMGSZ)  # letterbox(640) → proto 分辨率
        h, w = img_shape[:2]
        out: List[Optional[np.ndarray]] = []
        for i in range(len(coeffs)):
            bx1, by1, bx2, by2 = boxes640[i] * ratio
            cx1, cy1 = int(max(0, np.floor(bx1))), int(max(0, np.floor(by1)))
            cx2, cy2 = int(min(pw, np.ceil(bx2))), int(min(ph, np.ceil(by2)))
            ox1, oy1, ox2, oy2 = (int(round(float(v))) for v in boxes[i])
            ow, oh = ox2 - ox1, oy2 - oy1
            full = np.zeros((h, w), np.uint8)
            crop = masks[i, cy1:cy2, cx1:cx2] if (cx2 > cx1 and cy2 > cy1) else None
            if crop is not None and crop.size and ow > 0 and oh > 0:
                crop = (crop > 0.5).astype(np.uint8) * 255
                full[oy1:oy2, ox1:ox2] = cv2.resize(
                    crop, (ow, oh), interpolation=cv2.INTER_NEAREST)
            out.append(full)
        return out

    def _finalize(self, dets: List[Detection]) -> List[Detection]:
        """类别白名单 + NMS，最后按面积降序（界面取第一个当主体）。"""
        if self.class_ids is not None:
            dets = [d for d in dets if d.cls_id in self.class_ids]
        if not dets:
            return []
        # cv2 的 NMSBoxes 要的是 (x, y, w, h)，不是 xyxy —— 传错只会让抑制范围悄悄失真
        rects = [[d.box[0], d.box[1], d.box[2] - d.box[0], d.box[3] - d.box[1]] for d in dets]
        idx = np.asarray(cv2.dnn.NMSBoxes(rects, [d.score for d in dets], self.conf, self.iou)).reshape(-1)
        kept = [dets[int(i)] for i in idx]
        kept.sort(key=lambda d: d.area, reverse=True)
        return kept[: self.max_boxes]


# 进程内单例：NPU/ORT 会话建立要 1~2s，且一份权重只能持有一个 runtime；加载失败也记在案
# 不每次请求重试 —— 模型没到位就当能力缺失，交互界面回退手动点选，不反复刷日志。
_detector: Optional[WissightDetector] = None


def get_detector() -> Optional[WissightDetector]:
    """按配置返回可用的检测器；未启用或模型不可用时返回 None。"""
    global _detector
    # 先看开关再构造：配置关了就不该花那 1~2s 去加载一份用不上的权重
    if not sprayer_config.detector_enabled:
        return None
    if _detector is None:
        _detector = WissightDetector()
    return _detector if _detector.available else None
