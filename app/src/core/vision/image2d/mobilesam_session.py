import os
import sys
import platform
import logging
from pathlib import Path
from typing import Tuple, Optional, List

import cv2
import numpy as np
try:
    import torch
    HAS_TORCH = True
    # MobileSAM is an interactive feature; it must not consume all CPU cores.
    torch.set_num_threads(1)
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass
except ImportError:
    torch = None
    HAS_TORCH = False

logger = logging.getLogger(__name__)

# Adjust paths based on the project structure
REPO_ROOT = Path(__file__).resolve().parents[5]
MOBILESAM_DIR = REPO_ROOT / "third_party" / "MobileSAM"

DEFAULT_MOBILESAM_WEIGHTS = REPO_ROOT / "models" / "mobile_sam.pt"

# RKNN models (RK3588 NPU)
DEFAULT_MOBILESAM_RKNN_ENCODER = REPO_ROOT / "models" / "mobile_sam_encoder.rknn"
DEFAULT_MOBILESAM_RKNN_DECODER = REPO_ROOT / "models" / "mobile_sam_decoder.rknn"

# ONNX models (Non-RK, x86, macOS, generic Linux)
DEFAULT_MOBILESAM_ONNX_ENCODER = REPO_ROOT / "models" / "mobile_sam_encoder.onnx"
DEFAULT_MOBILESAM_ONNX_DECODER = REPO_ROOT / "models" / "mobile_sam_decoder.onnx"


def is_rk3588() -> bool:
    """Detect if running on a Rockchip RK3588 platform."""
    # 1. Check device tree model
    dt_model_path = Path("/proc/device-tree/model")
    if dt_model_path.exists():
        try:
            model_str = dt_model_path.read_text(errors="ignore").lower()
            if "rk3588" in model_str or "orange pi 5" in model_str:
                return True
        except Exception:
            pass

    # 2. Check device tree compatible string
    dt_compat_path = Path("/proc/device-tree/compatible")
    if dt_compat_path.exists():
        try:
            compat_str = dt_compat_path.read_text(errors="ignore").lower()
            if "rk3588" in compat_str:
                return True
        except Exception:
            pass

    # 3. Check architecture and NPU device node
    if platform.machine() in ("aarch64", "arm64"):
        if Path("/dev/rknpu").exists() or Path("/dev/mpp_service").exists():
            return True

    return False


def get_configured_sam_backend() -> Optional[str]:
    """Read backend configured in configs/aisprayer_config.yaml under interactive.sam.backend."""
    cfg_file = REPO_ROOT / "configs" / "aisprayer_config.yaml"
    if cfg_file.exists():
        try:
            import yaml
            with open(cfg_file, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
                backend = data.get("interactive", {}).get("sam", {}).get("backend")
                if backend and isinstance(backend, str) and backend.strip():
                    return backend.strip().lower()
        except Exception as e:
            logger.debug(f"[MobileSAM] Could not read backend from {cfg_file}: {e}")
    return None


def _model_ready(path: Path) -> bool:
    """模型文件存在、且不是未拉取的 Git LFS 指针文件。"""
    return path.exists() and path.stat().st_size > 1024


def _size_mb(path: Path) -> str:
    return f"{path.stat().st_size / (1024 * 1024):.1f} MB"


def _has_onnx_models() -> bool:
    """ONNX encoder/decoder 是否都就绪。"""
    return _model_ready(DEFAULT_MOBILESAM_ONNX_ENCODER) and _model_ready(DEFAULT_MOBILESAM_ONNX_DECODER)


def _torch_device() -> Optional[str]:
    """PyTorch 后端对应的设备名；torch 未安装（如 docker slim 镜像）时返回 None 交给自动探测。"""
    if not HAS_TORCH:
        return None
    return "cuda" if torch.cuda.is_available() else "cpu"


def resolve_device(requested: str | None = None) -> str:
    """
    Resolve inference device/backend:
    1. Explicit requested device ('rknn', 'onnx', 'cuda', 'mps', 'cpu', 'pt')
    2. Environment variable MOBILESAM_DEVICE or MOBILESAM_BACKEND
    3. Configuration file (configs/aisprayer_config.yaml: interactive.sam.backend)
    4. Auto-detection:
       - On RK3588 with RKNN model: 'rknn'
       - If CUDA available: 'cuda'
       - Non-RK with ONNX models: 'onnx' (faster & lighter than PyTorch CPU)
       - If MPS available: 'mps'
       - Otherwise: 'cpu'
    """
    def _normalize(target: str | None) -> str | None:
        if not target:
            return None
        t = target.lower().strip()
        if t in ("pt", "pytorch", "torch"):
            return _torch_device()
        if t in ("auto", "default"):
            return None
        return t

    norm = _normalize(requested)
    if norm:
        return norm

    env_dev = os.getenv("MOBILESAM_DEVICE") or os.getenv("MOBILESAM_BACKEND")
    norm = _normalize(env_dev)
    if norm:
        return norm

    cfg_backend = get_configured_sam_backend()
    norm = _normalize(cfg_backend)
    if norm:
        return norm

    # Auto-detection:
    # 1. On RK3588, default to RKNN
    if is_rk3588() and DEFAULT_MOBILESAM_RKNN_ENCODER.exists():
        return "rknn"

    # 2. PyTorch CUDA if available
    if HAS_TORCH and torch.cuda.is_available():
        return "cuda"

    # 3. Non-RK platform: prefer ONNX if onnx models exist and onnxruntime is available
    if _has_onnx_models():
        try:
            import onnxruntime
            return "onnx"
        except ImportError:
            pass

    if HAS_TORCH and hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return "mps"

    return "cpu"



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


# 运行时 decoder 期望的图签名：由 third_party/MobileSAM/scripts/export_onnx_model.py（SamOnnxModel）
# 导出 —— 多一个 orig_im_size 输入、在图内把 mask 上采样回原图、按 (masks, iou_predictions,
# low_res_masks) 顺序输出 3 路。tools/convert_mobilesam_to_rknn.py 导的是另一套签名（无
# orig_im_size、只 2 路且 iou 在前），_run_onnx_decoder 是按位置解包的，拿错文件会静默错位，
# 所以加载时就拒绝。
_DECODER_INPUTS = {
    "image_embeddings", "point_coords", "point_labels", "mask_input", "has_mask_input", "orig_im_size",
}
_DECODER_OUTPUTS = ["masks", "iou_predictions", "low_res_masks"]


def _check_decoder_graph(session, model_path: Path) -> None:
    """校验 ONNX decoder 的图签名；不匹配则抛错，由 load_mobilesam 回落到其他后端。"""
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
    """Shared ONNX decoder inference function for both pure-ONNX and RKNN+ONNX pipelines."""
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

    # Unpack batch dimension
    masks = masks[0]                  # (C, H, W)
    iou_predictions = iou_predictions[0]  # (C,)
    low_res_masks = low_res_masks[0]

    # SAM 的 mask decoder 有 4 个输出头：0 号是 multimask_output=False 的“稳定”单 mask，
    # 1~3 号是三个粒度候选。这份 ONNX 导出把 4 路原样输出（未做切片），所以必须在这里
    # 补上与 SamPredictor 一致的切片语义：否则加背景点精修时 argmax 会在不同粒度之间跳变，
    # 表现为“右键点背景反而让 mask 变大”。
    if masks.shape[0] == 4:
        keep = slice(1, None) if multimask_output else slice(0, 1)
        masks = masks[keep]
        iou_predictions = iou_predictions[keep]
        low_res_masks = low_res_masks[keep]

    return masks > 0.0, iou_predictions, low_res_masks


class ONNXMobileSAMPredictor:
    """
    Pure ONNX Runtime implementation of MobileSAM.
    Runs both the Image Encoder and Mask Decoder via ONNXRuntime,
    enabling high-performance inference on x86, macOS, and non-RK Linux without PyTorch overhead.
    """

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

        # NCHW normalized float32
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
    """
    Drop-in replacement for SamPredictor on Rockchip RK3588.
    The heavy Image Encoder (TinyViT, 1024x1024) runs on the RK3588 NPU via RKNN.
    The lightweight Prompt Encoder + Mask Decoder runs via ONNX Runtime on CPU (<15ms, Zero-PyTorch)
    or falls back to PyTorch if ONNX decoder is not available.
    """

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

        # Prefer ONNX decoder for Zero-PyTorch dependency
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
            from mobile_sam.utils.transforms import ResizeLongestSide

            logger.info(f"[MobileSAM-RKNN] Loading decoder weights from {ckpt}...")
            sam = sam_model_registry["vit_t"](checkpoint=ckpt)
            sam.eval()

            # Free heavy image encoder from CPU memory, retain dummy for property compatibility
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
        # 1. Try RKNNLite (board aarch64 runtime)
        try:
            from rknnlite.api import RKNNLite
            self.rknn = RKNNLite()
            ret = self.rknn.load_rknn(self.rknn_encoder_path)
            if ret != 0:
                raise RuntimeError(f"RKNNLite.load_rknn failed, code: {ret}")
            ret = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
            if ret != 0:
                raise RuntimeError(f"RKNNLite.init_runtime failed, code: {ret}")
            logger.info(f"[MobileSAM-RKNN] RKNNLite NPU runtime initialized (AUTO cores).")
            return
        except ImportError:
            pass

        # 2. Try full RKNN Toolkit (e.g. simulation or test host)
        try:
            from rknn.api import RKNN
            self.rknn = RKNN(verbose=False)
            ret = self.rknn.load_rknn(self.rknn_encoder_path)
            if ret != 0:
                raise RuntimeError(f"RKNN.load_rknn failed, code: {ret}")
            ret = self.rknn.init_runtime()
            if ret != 0:
                raise RuntimeError(f"RKNN.init_runtime failed, code: {ret}")
            logger.info(f"[MobileSAM-RKNN] RKNN runtime initialized.")
            return
        except ImportError:
            pass

        raise RuntimeError(
            "Neither 'rknnlite' nor 'rknn' Python module is available. "
            "Install rknn-toolkit-lite2 on RK3588 (or rknn-toolkit2 on PC)."
        )

    def set_image(self, image: np.ndarray, image_format: str = "RGB") -> None:
        """
        Calculates image embeddings using RKNN on the NPU.
        """
        assert image_format in ["RGB", "BGR"], f"image_format must be RGB or BGR, got {image_format}"
        if image_format == "BGR":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        self.reset_image()
        padded, self.original_size, self.input_size = _resize_and_pad(image, self.img_size)

        # NPU input: NHWC float32 (normalization mean/std is built into the RKNN model)
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
    device: str | None = None,
    rknn_encoder_path: str = str(DEFAULT_MOBILESAM_RKNN_ENCODER),
    onnx_encoder_path: str = str(DEFAULT_MOBILESAM_ONNX_ENCODER),
    onnx_decoder_path: str = str(DEFAULT_MOBILESAM_ONNX_DECODER),
):
    """
    Unified loader for MobileSAM supporting RKNN, ONNX, and PyTorch backends.
    """
    device = resolve_device(device)
    onnx_enc, onnx_dec = Path(onnx_encoder_path), Path(onnx_decoder_path)
    onnx_ready = _model_ready(onnx_enc) and _model_ready(onnx_dec)
    # RKNN / ONNX 都不可用时的最终归宿（torch 未安装时就是 cpu，走不到 PyTorch 分支）
    torch_dev = _torch_device() or "cpu"

    logger.info("=" * 66)
    logger.info(f"[MobileSAM] Resolving MobileSAM engine backend (target: '{device}')...")

    predictor = None

    # 1. RKNN path (Rockchip RK3588 NPU)
    if device == "rknn":
        rknn_path = Path(rknn_encoder_path)
        if not rknn_path.exists():
            logger.warning(
                f"[MobileSAM] RKNN encoder model not found at {rknn_path.resolve()}, "
                f"falling back to ONNX/PyTorch."
            )
            device = "onnx" if onnx_ready else torch_dev
        elif not _model_ready(rknn_path):
            logger.warning(
                f"[MobileSAM] RKNN encoder model at {rknn_path.resolve()} is an un-pulled Git LFS pointer "
                f"({rknn_path.stat().st_size} bytes). Run 'git lfs pull' to download actual model weight. "
                f"Falling back to ONNX/PyTorch."
            )
            device = "onnx" if onnx_ready else torch_dev
        else:
            try:
                has_onnx_dec = _model_ready(onnx_dec)
                ckpt_p = Path(checkpoint)
                if has_onnx_dec:
                    dec_desc = f"{onnx_dec.name} ({_size_mb(onnx_dec)}, ONNX - Zero PyTorch)"
                else:
                    dec_desc = f"{ckpt_p.name} (PyTorch)"

                logger.info(f"[MobileSAM] Loading RKNN Backend (Rockchip RK3588 NPU):")
                logger.info(f"[MobileSAM]   * Encoder (.rknn): {rknn_path.resolve()} ({_size_mb(rknn_path)})")
                logger.info(f"[MobileSAM]   * Decoder:        {dec_desc}")
                predictor = RKNNMobileSAMPredictor(
                    rknn_encoder_path=str(rknn_path),
                    checkpoint=checkpoint if not has_onnx_dec else None,
                    onnx_decoder_path=str(onnx_dec) if has_onnx_dec else None,
                )
            except Exception as e:
                logger.warning(
                    f"[MobileSAM] RKNN MobileSAM backend not available: {e}. Falling back to ONNX/PyTorch."
                )
                logger.debug("[MobileSAM] RKNN initialization error details:", exc_info=True)
                device = "onnx" if onnx_ready else torch_dev

    # 2. ONNX Runtime path (Fast, lightweight inference on non-RK platforms)
    if device == "onnx":
        if not (onnx_enc.exists() and onnx_dec.exists()):
            logger.warning(
                f"[MobileSAM] ONNX models not found ({onnx_enc.name}, {onnx_dec.name}), falling back to PyTorch."
            )
            device = torch_dev
        elif not onnx_ready:
            logger.warning(
                f"[MobileSAM] ONNX models at {onnx_enc.name}/{onnx_dec.name} are un-pulled Git LFS pointers. "
                f"Falling back to PyTorch."
            )
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
                logger.warning(
                    f"[MobileSAM] ONNX MobileSAM backend not available: {e}. Falling back to PyTorch."
                )
                logger.debug("[MobileSAM] ONNX initialization error details:", exc_info=True)
                device = torch_dev

    # 3. PyTorch path (CUDA / MPS / CPU fallback)
    if predictor is None:
        if not HAS_TORCH:
            logger.error(
                "[MobileSAM] No usable RKNN/ONNX model and PyTorch is not installed; MobileSAM is disabled."
            )
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



class MobileSAMSession:
    """一张已编码图像及其点选会话。predictor 同时只能持有一张图的 embedding，
    所以一个 predictor 只能对应一个活跃会话（切换模板必须重新 set_image）。"""

    def __init__(self, predictor):
        self.predictor = predictor
        self.image_bgr = None

    def set_image(self, image_bgr: np.ndarray):
        self.image_bgr = image_bgr
        if self.predictor:
            self.predictor.set_image(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))

    def predict(self, points: list[tuple[int, int]], labels: list[int]) -> tuple[np.ndarray | None, float]:
        """
        Returns:
            mask (np.ndarray): Boolean mask array
            score (float): Confidence score
        """
        if not self.predictor or not points:
            return None, 0.0

        coords = np.array(points, dtype=np.float32)
        lbls = np.array(labels, dtype=np.int32)
        # 只点一个前景点时输入是歧义的，SAM 建议在三个粒度候选里取最优；一旦加了补充点
        # （尤其是背景点）就改用稳定单 mask，否则 mask 会在不同粒度之间跳变。
        multimask = len(points) == 1

        masks, scores, _ = self.predictor.predict(
            point_coords=coords,
            point_labels=lbls,
            multimask_output=multimask,
        )

        best_idx = int(np.argmax(scores))
        best_mask = masks[best_idx].astype(bool)
        best_score = float(scores[best_idx])

        return best_mask, best_score
