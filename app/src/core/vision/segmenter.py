"""MobileSAM 交互式语义分割模块：支持 RK3588 NPU (RKNN)、ONNX Runtime 与 PyTorch 多后端。

以面向对象方式提供 `MobileSAMSegmenter` 类，管理图像 Embedding 缓存与交互式提示词（点选、框选）掩码预测。
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from core.config import sprayer_config
from core.vision.backend import (
    REPO_ROOT,
    is_rk3588,
    load_rknn_runtime,
    model_ready as _model_ready,
    pick_backend,
    size_mb as _size_mb,
    torch_device as _torch_device,
)

try:
    import torch
    HAS_TORCH = True
    # MobileSAM 是交互式功能，限制单线程避免挤占全部 CPU 资源
    torch.set_num_threads(1)
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass
except ImportError:
    torch = None
    HAS_TORCH = False

logger = logging.getLogger(__name__)

MOBILESAM_DIR = REPO_ROOT / "third_party" / "MobileSAM"
DEFAULT_MOBILESAM_WEIGHTS = REPO_ROOT / "models" / "mobile_sam.pt"

# RKNN image encoder (RK3588 NPU)。decoder 固定使用 ONNX decoder
DEFAULT_MOBILESAM_RKNN_ENCODER = REPO_ROOT / "models" / "mobile_sam_encoder.rknn"

# ONNX 模型
DEFAULT_MOBILESAM_ONNX_ENCODER = REPO_ROOT / "models" / "mobile_sam_encoder.onnx"
DEFAULT_MOBILESAM_ONNX_DECODER = REPO_ROOT / "models" / "mobile_sam_decoder.onnx"


def _has_onnx_models() -> bool:
    """ONNX encoder/decoder 是否均已就绪。"""
    return _model_ready(DEFAULT_MOBILESAM_ONNX_ENCODER) and _model_ready(DEFAULT_MOBILESAM_ONNX_DECODER)


def _auto_backend() -> str:
    """未指定后端时的自动探测优先级：NPU > CUDA > ONNX > MPS > CPU。"""
    if is_rk3588() and _model_ready(DEFAULT_MOBILESAM_RKNN_ENCODER):
        return "rknn"
    if HAS_TORCH and torch.cuda.is_available():
        return "cuda"
    if _has_onnx_models():
        try:
            import onnxruntime  # noqa: F401
            return "onnx"
        except ImportError:
            pass
    if HAS_TORCH and hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def resolve_device(requested: Optional[str] = None) -> str:
    """解析 MobileSAM 推理后端。"""
    return pick_backend(
        requested,
        ("MOBILESAM_DEVICE", "MOBILESAM_BACKEND"),
        sprayer_config.sam_backend,
        _auto_backend,
    )


def _resize_and_pad(image: np.ndarray, img_size: int) -> Tuple[np.ndarray, Tuple[int, int], Tuple[int, int]]:
    """按长边等比缩放 + 右下补零到 img_size，返回 (padded, original_size, input_size)。"""
    orig_h, orig_w = image.shape[:2]
    scale = img_size * 1.0 / max(orig_h, orig_w)
    new_h = int(orig_h * scale + 0.5)
    new_w = int(orig_w * scale + 0.5)
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(
        resized, 0, img_size - new_h, 0, img_size - new_w, cv2.BORDER_CONSTANT, value=(0, 0, 0)
    )
    return padded, (orig_h, orig_w), (new_h, new_w)


# 运行时 decoder 期望的图签名：由 export_onnx_model.py 导出
_DECODER_INPUTS = {
    "image_embeddings", "point_coords", "point_labels", "mask_input", "has_mask_input", "orig_im_size",
}
_DECODER_OUTPUTS = ["masks", "iou_predictions", "low_res_masks"]


def _check_decoder_graph(session, model_path: Path) -> None:
    """校验 ONNX decoder 图签名；不匹配则抛错由加载器回退。"""
    ins = {i.name for i in session.get_inputs()}
    outs = [o.name for o in session.get_outputs()]
    if ins != _DECODER_INPUTS or outs != _DECODER_OUTPUTS:
        raise RuntimeError(
            f"{model_path.name} is not the expected MobileSAM decoder graph "
            f"(got inputs={sorted(ins)}, outputs={outs}). "
            f"Re-export it with third_party/MobileSAM/scripts/export_onnx_model.py."
        )


def _run_onnx_decoder(
    decoder_session,
    features: np.ndarray,
    original_size: Tuple[int, int],
    input_size: Tuple[int, int],
    point_coords: np.ndarray,
    point_labels: np.ndarray,
    multimask_output: bool = True,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """ONNX decoder 推理执行函数。"""
    orig_h, orig_w = original_size
    new_h, new_w = input_size

    coords = np.array(point_coords, dtype=np.float32).reshape(-1, 2)
    coords[:, 0] *= new_w / orig_w
    coords[:, 1] *= new_h / orig_h

    ort_inputs = {
        "image_embeddings": features,
        "point_coords": coords[None, :, :],  # (1, N, 2)
        "point_labels": np.asarray(point_labels, dtype=np.float32)[None, :],  # (1, N)
        "mask_input": np.zeros((1, 1, 256, 256), dtype=np.float32),
        "has_mask_input": np.zeros(1, dtype=np.float32),
        "orig_im_size": np.array([float(orig_h), float(orig_w)], dtype=np.float32),
    }

    masks, iou_predictions, low_res_masks = decoder_session.run(None, ort_inputs)

    masks = masks[0]                      # (C, H, W)
    iou_predictions = iou_predictions[0]  # (C,)
    low_res_masks = low_res_masks[0]

    # SAM mask decoder 有 4 个输出头：0 号为稳定单 mask，1~3 号为 3 个粒度候选。
    # 多提示词或带 box 时切片为 0 号，避免在粒度间跳变导致加背景点反向膨胀。
    if masks.shape[0] == 4:
        keep = slice(1, None) if multimask_output else slice(0, 1)
        masks = masks[keep]
        iou_predictions = iou_predictions[keep]
        low_res_masks = low_res_masks[keep]

    return masks > 0.0, iou_predictions, low_res_masks


class ONNXMobileSAMPredictor:
    """基于 ONNX Runtime 的纯轻量化 MobileSAM 预测器。"""

    def __init__(
        self,
        encoder_path: str = str(DEFAULT_MOBILESAM_ONNX_ENCODER),
        decoder_path: str = str(DEFAULT_MOBILESAM_ONNX_DECODER),
        providers: Optional[List[str]] = None,
    ):
        import onnxruntime as ort

        if providers is None:
            available = ort.get_available_providers()
            providers = []
            if "CUDAExecutionProvider" in available:
                providers.append("CUDAExecutionProvider")
            if "CoreMLExecutionProvider" in available:
                providers.append("CoreMLExecutionProvider")
            providers.append("CPUExecutionProvider")

        logger.info(f"[MobileSAM-ONNX] Loading ONNX models with providers: {providers}")
        self.encoder_session = ort.InferenceSession(encoder_path, providers=providers)
        self.decoder_session = ort.InferenceSession(decoder_path, providers=providers)
        _check_decoder_graph(self.decoder_session, Path(decoder_path))

        self.encoder_path = str(encoder_path)
        self.decoder_path = str(decoder_path)
        self.providers = providers
        active_providers = self.encoder_session.get_providers()
        self.active_provider = active_providers[0] if active_providers else "CPUExecutionProvider"
        self.backend_type = "onnx"
        self.backend_desc = (
            f"ONNX Runtime ({self.active_provider}) | "
            f"Encoder: {Path(encoder_path).name}, Decoder: {Path(decoder_path).name}"
        )
        self.model_files = {"encoder": self.encoder_path, "decoder": self.decoder_path}

        self.img_size = 1024
        self.pixel_mean = np.array([123.675, 116.28, 103.53], dtype=np.float32).reshape(1, 3, 1, 1)
        self.pixel_std = np.array([58.395, 57.12, 57.375], dtype=np.float32).reshape(1, 3, 1, 1)

        self.is_image_set = False
        self.features = None
        self.original_size = None
        self.input_size = None

    def set_image(self, image: np.ndarray, image_format: str = "RGB") -> None:
        assert image_format in ["RGB", "BGR"], f"image_format must be RGB or BGR, got {image_format}"
        if image_format == "BGR":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        self.reset_image()
        padded, self.original_size, self.input_size = _resize_and_pad(image, self.img_size)

        tensor = padded.transpose(2, 0, 1)[None, ...].astype(np.float32)
        tensor = (tensor - self.pixel_mean) / self.pixel_std

        t0 = cv2.getTickCount()
        self.features = self.encoder_session.run(None, {"image": tensor})[0]
        cost_ms = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000.0
        logger.debug(f"[MobileSAM-ONNX] Image encoded in {cost_ms:.1f}ms")

        self.is_image_set = True

    def predict(
        self,
        point_coords: np.ndarray,
        point_labels: np.ndarray,
        multimask_output: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.is_image_set:
            raise RuntimeError("An image must be set with .set_image(...) before mask prediction.")
        return _run_onnx_decoder(
            decoder_session=self.decoder_session,
            features=self.features,
            original_size=self.original_size,
            input_size=self.input_size,
            point_coords=point_coords,
            point_labels=point_labels,
            multimask_output=multimask_output,
        )

    def reset_image(self) -> None:
        self.is_image_set = False
        self.features = None
        self.original_size = None
        self.input_size = None


class RKNNMobileSAMPredictor:
    """在 Rockchip RK3588 NPU 上运行的 MobileSAM 预测器。"""

    def __init__(
        self,
        rknn_encoder_path: str = str(DEFAULT_MOBILESAM_RKNN_ENCODER),
        checkpoint: Optional[str] = None,
        onnx_decoder_path: Optional[str] = str(DEFAULT_MOBILESAM_ONNX_DECODER),
    ):
        self.rknn_encoder_path = str(rknn_encoder_path)
        self.onnx_decoder_path = str(onnx_decoder_path) if onnx_decoder_path else None
        self.checkpoint = str(checkpoint) if checkpoint else None
        self.backend_type = "rknn"
        self.rknn = None
        self._init_rknn()

        self.use_onnx_decoder = False
        self.decoder_session = None
        self.sam_predictor = None

        if self.onnx_decoder_path:
            dec_p = Path(self.onnx_decoder_path)
            if _model_ready(dec_p):
                try:
                    import onnxruntime as ort
                    self.decoder_session = ort.InferenceSession(str(dec_p), providers=["CPUExecutionProvider"])
                    _check_decoder_graph(self.decoder_session, dec_p)
                    self.use_onnx_decoder = True
                    self.backend_desc = (
                        f"RKNN (Rockchip RK3588 NPU) + ONNX Decoder | "
                        f"Encoder: {Path(rknn_encoder_path).name}, Decoder: {dec_p.name}"
                    )
                    self.model_files = {"encoder": self.rknn_encoder_path, "decoder": str(dec_p)}
                    logger.info(f"[MobileSAM-RKNN] Initialized ONNX Runtime Decoder ({dec_p.name}, Zero-PyTorch).")
                except Exception as e:
                    logger.warning(f"[MobileSAM-RKNN] Failed to load ONNX decoder: {e}, falling back to PyTorch.")

        if not self.use_onnx_decoder:
            if not HAS_TORCH:
                raise RuntimeError(
                    "Neither ONNX decoder nor PyTorch is available for MobileSAM decoder on RK3588."
                )
            ckpt = self.checkpoint or str(DEFAULT_MOBILESAM_WEIGHTS)
            self.backend_desc = (
                f"RKNN (Rockchip RK3588 NPU) | "
                f"Encoder: {Path(rknn_encoder_path).name}, Decoder: {Path(ckpt).name}"
            )
            self.model_files = {"encoder": self.rknn_encoder_path, "decoder": ckpt}

            if str(MOBILESAM_DIR) not in sys.path:
                sys.path.insert(0, str(MOBILESAM_DIR))

            from mobile_sam import sam_model_registry, SamPredictor

            logger.info(f"[MobileSAM-RKNN] Loading decoder weights from {ckpt}...")
            sam = sam_model_registry["vit_t"](checkpoint=ckpt)
            sam.eval()

            del sam.image_encoder
            sam.image_encoder = type("DummyEncoder", (), {"img_size": 1024})()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()

            self.sam_predictor = SamPredictor(sam)

        self.img_size = 1024
        self.is_image_set = False
        self.features = None
        self.original_size = None
        self.input_size = None

    def _init_rknn(self):
        self.rknn = load_rknn_runtime(self.rknn_encoder_path)

    def set_image(self, image: np.ndarray, image_format: str = "RGB") -> None:
        assert image_format in ["RGB", "BGR"], f"image_format must be RGB or BGR, got {image_format}"
        if image_format == "BGR":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        self.reset_image()
        padded, self.original_size, self.input_size = _resize_and_pad(image, self.img_size)

        input_tensor = np.expand_dims(padded, axis=0).astype(np.float32)

        t0 = cv2.getTickCount()
        outputs = self.rknn.inference(inputs=[input_tensor])
        cost_ms = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000.0
        logger.debug(f"[MobileSAM-RKNN] NPU image encoding completed in {cost_ms:.1f}ms")

        features_np = outputs[0]  # shape: (1, 256, 64, 64)
        self.features = features_np

        if not self.use_onnx_decoder and self.sam_predictor is not None:
            self.sam_predictor.features = torch.from_numpy(features_np).float()
            self.sam_predictor.original_size = self.original_size
            self.sam_predictor.input_size = self.input_size
            self.sam_predictor.is_image_set = True

        self.is_image_set = True

    def predict(
        self,
        point_coords: np.ndarray,
        point_labels: np.ndarray,
        multimask_output: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.is_image_set:
            raise RuntimeError("An image must be set with .set_image(...) before mask prediction.")

        if self.use_onnx_decoder:
            return _run_onnx_decoder(
                decoder_session=self.decoder_session,
                features=self.features,
                original_size=self.original_size,
                input_size=self.input_size,
                point_coords=point_coords,
                point_labels=point_labels,
                multimask_output=multimask_output,
            )

        return self.sam_predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            multimask_output=multimask_output,
        )

    def reset_image(self) -> None:
        self.is_image_set = False
        self.features = None
        self.original_size = None
        self.input_size = None
        if hasattr(self, "sam_predictor") and self.sam_predictor is not None:
            self.sam_predictor.reset_image()


def load_mobilesam(
    checkpoint: str = str(DEFAULT_MOBILESAM_WEIGHTS),
    device: Optional[str] = None,
    rknn_encoder_path: str = str(DEFAULT_MOBILESAM_RKNN_ENCODER),
    onnx_encoder_path: str = str(DEFAULT_MOBILESAM_ONNX_ENCODER),
    onnx_decoder_path: str = str(DEFAULT_MOBILESAM_ONNX_DECODER),
):
    """统一加载器：按后端优先级实例化 MobileSAM 底层预测器。"""
    device = resolve_device(device)
    onnx_enc, onnx_dec = Path(onnx_encoder_path), Path(onnx_decoder_path)
    onnx_ready = _model_ready(onnx_enc) and _model_ready(onnx_dec)
    torch_dev = _torch_device() or "cpu"

    logger.info("=" * 66)
    logger.info(f"[MobileSAM] Resolving MobileSAM engine backend (target: '{device}')...")

    predictor = None

    # 1. RKNN path (Rockchip RK3588 NPU)
    if device == "rknn":
        rknn_path = Path(rknn_encoder_path)
        if not rknn_path.exists():
            logger.warning(
                f"[MobileSAM] RKNN encoder model not found at {rknn_path.resolve()}, falling back to ONNX/PyTorch."
            )
            device = "onnx" if onnx_ready else torch_dev
        elif not _model_ready(rknn_path):
            logger.warning(
                f"[MobileSAM] RKNN encoder model at {rknn_path.resolve()} is an un-pulled Git LFS pointer. Falling back."
            )
            device = "onnx" if onnx_ready else torch_dev
        else:
            try:
                logger.info(f"[MobileSAM] Loading RKNN (NPU) Backend:")
                logger.info(f"[MobileSAM]   * Encoder (.rknn): {rknn_path.resolve()} ({_size_mb(rknn_path)})")
                predictor = RKNNMobileSAMPredictor(
                    rknn_encoder_path=str(rknn_path),
                    checkpoint=checkpoint,
                    onnx_decoder_path=onnx_decoder_path,
                )
            except Exception as e:
                logger.warning(f"[MobileSAM] RKNN NPU backend failed: {e}. Falling back to ONNX/PyTorch.")
                device = "onnx" if onnx_ready else torch_dev

    # 2. ONNX path
    if predictor is None and device == "onnx":
        if not onnx_ready:
            logger.warning("[MobileSAM] ONNX models not ready; falling back to PyTorch.")
            device = torch_dev
        else:
            try:
                logger.info(f"[MobileSAM] Loading ONNX Runtime Backend:")
                logger.info(f"[MobileSAM]   * Encoder (.onnx): {onnx_enc.resolve()} ({_size_mb(onnx_enc)})")
                logger.info(f"[MobileSAM]   * Decoder (.onnx): {onnx_dec.resolve()} ({_size_mb(onnx_dec)})")
                predictor = ONNXMobileSAMPredictor(
                    encoder_path=str(onnx_enc),
                    decoder_path=str(onnx_dec),
                )
            except Exception as e:
                logger.warning(f"[MobileSAM] ONNX backend not available: {e}. Falling back to PyTorch.")
                device = torch_dev

    # 3. PyTorch path
    if predictor is None:
        if not HAS_TORCH:
            logger.error("[MobileSAM] No usable RKNN/ONNX model and PyTorch is not installed; MobileSAM disabled.")
            logger.info("=" * 66)
            return None

        if str(MOBILESAM_DIR) not in sys.path:
            sys.path.insert(0, str(MOBILESAM_DIR))

        ckpt_p = Path(checkpoint)
        ckpt_size = _size_mb(ckpt_p) if ckpt_p.exists() else "missing"
        gpu_name = f" ({torch.cuda.get_device_name(0)})" if device.startswith("cuda") and torch.cuda.is_available() else ""

        logger.info(f"[MobileSAM] Loading PyTorch Backend on device '{device}'{gpu_name}:")
        logger.info(f"[MobileSAM]   * Model Checkpoint (.pt): {ckpt_p.resolve()} ({ckpt_size})")

        try:
            from mobile_sam import SamPredictor, sam_model_registry
            model = sam_model_registry["vit_t"](checkpoint=checkpoint)
            model.to(device=device)
            model.eval()
            predictor = SamPredictor(model)
            predictor.backend_type = "pytorch"
            predictor.backend_desc = f"PyTorch ({device}{gpu_name}) | Checkpoint: {ckpt_p.name}"
            predictor.model_files = {"checkpoint": str(ckpt_p)}
        except Exception as e:
            logger.error(f"[MobileSAM] Failed to load MobileSAM via PyTorch: {e}", exc_info=True)
            logger.info("=" * 66)
            return None

    logger.info(f"[MobileSAM] >>> ACTIVE BACKEND: {predictor.backend_desc} <<<")
    logger.info("=" * 66)
    return predictor


class MobileSAMSegmenter:
    """MobileSAM 交互分割核心类。

    封装底层推理 Predictor 及当前已编码图像的特征 (Embedding)。
    支持点提示（前景=1, 背景=0）与框提示（box [x1, y1, x2, y2]）。
    """

    def __init__(self, predictor=None, device: Optional[str] = None):
        """
        :param predictor: 已构造的底层 Predictor 对象；若为 None 则通过 load_mobilesam 自动加载
        :param device: 目标推理后端（可选）
        """
        if predictor is not None:
            self.predictor = predictor
        else:
            self.predictor = load_mobilesam(device=device)
        self.image_bgr: Optional[np.ndarray] = None

    @property
    def available(self) -> bool:
        """底层模型是否可用。"""
        return self.predictor is not None

    @property
    def backend_desc(self) -> str:
        """底层后端描述。"""
        if not self.predictor:
            return "disabled"
        return getattr(self.predictor, "backend_desc", type(self.predictor).__name__)

    @property
    def is_image_set(self) -> bool:
        """当前是否已载入图像并完成 Embedding 编码。"""
        if not self.predictor:
            return False
        return bool(getattr(self.predictor, "is_image_set", False))

    def set_image(self, image_bgr: np.ndarray) -> None:
        """传入 BGR 图像，完成长边等比对齐与 NPU/GPU/CPU 特征提取。"""
        self.image_bgr = image_bgr
        if self.predictor:
            self.predictor.set_image(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))

    def reset_image(self) -> None:
        """清空当前缓存图像与 Embedding。"""
        self.image_bgr = None
        if self.predictor:
            self.predictor.reset_image()

    def predict(
        self,
        points: Sequence[Tuple[int, int]],
        labels: Sequence[int],
        box: Optional[Sequence[float]] = None,
    ) -> Tuple[Optional[np.ndarray], float]:
        """执行掩码预测。

        :param points: [(x, y), ...] 点坐标列表
        :param labels: [1, 0, ...] 点标签列表（1 为前景点，0 为背景点）
        :param box: 可选的目标框 (x1, y1, x2, y2)
        :return: (boolean_mask [H, W], iou_score)；无有效提示或模型不可用时返回 (None, 0.0)
        """
        if not self.predictor or (not points and box is None):
            return None, 0.0

        coords = list(points)
        lbls = list(labels)
        if box is not None:
            if len(box) != 4:
                raise ValueError(f"box must have 4 numbers (x1, y1, x2, y2), got {box!r}")
            x1, y1, x2, y2 = (float(v) for v in box)
            coords += [(min(x1, x2), min(y1, y2)), (max(x1, x2), max(y1, y2))]
            lbls += [2, 3]

        multimask = box is None and len(coords) == 1

        masks, scores, _ = self.predictor.predict(
            point_coords=np.array(coords, dtype=np.float32),
            point_labels=np.array(lbls, dtype=np.int32),
            multimask_output=multimask,
        )

        best_idx = int(np.argmax(scores))
        best_mask = masks[best_idx].astype(bool)
        best_score = float(scores[best_idx])

        return best_mask, best_score
