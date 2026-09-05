import os
import sys
import platform
import logging
from pathlib import Path
from typing import Tuple, Optional, List

import cv2
import numpy as np
import torch

logger = logging.getLogger(__name__)

# MobileSAM is an interactive feature; it must not consume all CPU cores.
torch.set_num_threads(1)
try:
    torch.set_num_interop_threads(1)
except RuntimeError:
    pass

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
            return "cuda" if torch.cuda.is_available() else "cpu"
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
    if torch.cuda.is_available():
        return "cuda"

    # 3. Non-RK platform: prefer ONNX if onnx models exist and onnxruntime is available
    if DEFAULT_MOBILESAM_ONNX_ENCODER.exists() and DEFAULT_MOBILESAM_ONNX_DECODER.exists():
        try:
            import onnxruntime
            return "onnx"
        except ImportError:
            pass

    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return "mps"

    return "cpu"



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
        orig_h, orig_w = image.shape[:2]
        self.original_size = (orig_h, orig_w)

        # Aspect-ratio preserving resize to longest side 1024
        scale = self.img_size * 1.0 / max(orig_h, orig_w)
        new_h = int(orig_h * scale + 0.5)
        new_w = int(orig_w * scale + 0.5)
        self.input_size = (new_h, new_w)

        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        pad_h = self.img_size - new_h
        pad_w = self.img_size - new_w
        padded = cv2.copyMakeBorder(resized, 0, pad_h, 0, pad_w, cv2.BORDER_CONSTANT, value=(0, 0, 0))

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
        point_coords: Optional[np.ndarray] = None,
        point_labels: Optional[np.ndarray] = None,
        box: Optional[np.ndarray] = None,
        mask_input: Optional[np.ndarray] = None,
        multimask_output: bool = True,
        return_logits: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.is_image_set:
            raise RuntimeError("An image must be set with .set_image(...) before mask prediction.")

        orig_h, orig_w = self.original_size
        new_h, new_w = self.input_size

        pts_list = []
        lbls_list = []
        if point_coords is not None and point_labels is not None:
            coords = np.array(point_coords, dtype=np.float32).copy()
            coords[:, 0] = coords[:, 0] * (new_w / orig_w)
            coords[:, 1] = coords[:, 1] * (new_h / orig_h)
            pts_list.append(coords)
            lbls_list.append(np.array(point_labels, dtype=np.float32))

        if box is not None:
            box_pts = np.array(
                [
                    [box[0] * (new_w / orig_w), box[1] * (new_h / orig_h)],
                    [box[2] * (new_w / orig_w), box[3] * (new_h / orig_h)],
                ],
                dtype=np.float32,
            )
            box_lbls = np.array([2.0, 3.0], dtype=np.float32)
            pts_list.append(box_pts)
            lbls_list.append(box_lbls)

        if not pts_list:
            raise ValueError("Must provide point_coords or box.")

        all_coords = np.concatenate(pts_list, axis=0)[None, :, :]  # (1, N, 2)
        all_labels = np.concatenate(lbls_list, axis=0)[None, :]    # (1, N)

        if mask_input is not None:
            has_mask = np.ones(1, dtype=np.float32)
            m_input = mask_input[None, ...] if len(mask_input.shape) == 3 else mask_input
        else:
            has_mask = np.zeros(1, dtype=np.float32)
            m_input = np.zeros((1, 1, 256, 256), dtype=np.float32)

        orig_im_size = np.array([float(orig_h), float(orig_w)], dtype=np.float32)

        ort_inputs = {
            "image_embeddings": self.features,
            "point_coords": all_coords,
            "point_labels": all_labels,
            "mask_input": m_input,
            "has_mask_input": has_mask,
            "orig_im_size": orig_im_size,
        }

        masks, iou_predictions, low_res_masks = self.decoder_session.run(None, ort_inputs)

        # Unpack batch dimension
        masks = masks[0]                  # (C, H, W)
        iou_predictions = iou_predictions[0]  # (C,)
        low_res_masks = low_res_masks[0]

        if not return_logits:
            masks = masks > 0.0

        if not multimask_output and len(iou_predictions) > 1:
            best_idx = int(np.argmax(iou_predictions))
            masks = masks[best_idx : best_idx + 1]
            iou_predictions = iou_predictions[best_idx : best_idx + 1]
            low_res_masks = low_res_masks[best_idx : best_idx + 1]

        return masks, iou_predictions, low_res_masks

    def reset_image(self) -> None:
        self.is_image_set = False
        self.features = None
        self.original_size = None
        self.input_size = None


class RKNNMobileSAMPredictor:
    """
    Drop-in replacement for SamPredictor on Rockchip RK3588.
    The heavy Image Encoder (TinyViT, 1024x1024) runs on the RK3588 NPU via RKNN,
    while the lightweight Prompt Encoder + Mask Decoder runs in PyTorch on CPU (<5ms).
    """

    def __init__(
        self,
        rknn_encoder_path: str = str(DEFAULT_MOBILESAM_RKNN_ENCODER),
        checkpoint: str = str(DEFAULT_MOBILESAM_WEIGHTS),
    ):
        self.rknn_encoder_path = str(rknn_encoder_path)
        self.checkpoint = str(checkpoint)
        self.backend_type = "rknn"
        self.backend_desc = (
            f"RKNN (Rockchip RK3588 NPU) | "
            f"Encoder: {Path(rknn_encoder_path).name}, Decoder: {Path(checkpoint).name}"
        )
        self.model_files = {"encoder": self.rknn_encoder_path, "decoder": self.checkpoint}
        self.rknn = None
        self._init_rknn()

        if str(MOBILESAM_DIR) not in sys.path:
            sys.path.insert(0, str(MOBILESAM_DIR))

        from mobile_sam import sam_model_registry, SamPredictor
        from mobile_sam.utils.transforms import ResizeLongestSide

        logger.info(f"[MobileSAM-RKNN] Loading decoder weights from {checkpoint}...")
        sam = sam_model_registry["vit_t"](checkpoint=checkpoint)
        sam.eval()

        # Free heavy image encoder from CPU memory, retain dummy for property compatibility
        del sam.image_encoder
        sam.image_encoder = type("DummyEncoder", (), {"img_size": 1024})()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        self.sam_predictor = SamPredictor(sam)
        self.transform = ResizeLongestSide(1024)
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
        orig_h, orig_w = image.shape[:2]
        self.original_size = (orig_h, orig_w)

        # Scale image so longest side is 1024
        scale = self.img_size * 1.0 / max(orig_h, orig_w)
        new_h = int(orig_h * scale + 0.5)
        new_w = int(orig_w * scale + 0.5)
        self.input_size = (new_h, new_w)

        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        pad_h = self.img_size - new_h
        pad_w = self.img_size - new_w
        padded = cv2.copyMakeBorder(resized, 0, pad_h, 0, pad_w, cv2.BORDER_CONSTANT, value=(0, 0, 0))

        # NPU input: NHWC float32 or uint8 (normalization mean/std is built into the RKNN model)
        input_tensor = np.expand_dims(padded, axis=0).astype(np.float32)

        t0 = cv2.getTickCount()
        outputs = self.rknn.inference(inputs=[input_tensor])
        cost_ms = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000.0
        logger.debug(f"[MobileSAM-RKNN] NPU image encoding completed in {cost_ms:.1f}ms")

        features_np = outputs[0]  # shape: (1, 256, 64, 64)
        self.features = torch.from_numpy(features_np).float()

        # Update underlying predictor state
        self.sam_predictor.features = self.features
        self.sam_predictor.original_size = self.original_size
        self.sam_predictor.input_size = self.input_size
        self.sam_predictor.is_image_set = True
        self.is_image_set = True

    def predict(
        self,
        point_coords: Optional[np.ndarray] = None,
        point_labels: Optional[np.ndarray] = None,
        box: Optional[np.ndarray] = None,
        mask_input: Optional[np.ndarray] = None,
        multimask_output: bool = True,
        return_logits: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.is_image_set:
            raise RuntimeError("An image must be set with .set_image(...) before mask prediction.")
        return self.sam_predictor.predict(
            point_coords=point_coords,
            point_labels=point_labels,
            box=box,
            mask_input=mask_input,
            multimask_output=multimask_output,
            return_logits=return_logits,
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
            device = "onnx" if (Path(onnx_encoder_path).exists() and Path(onnx_decoder_path).exists()) else "cpu"
        elif rknn_path.stat().st_size < 1024:
            logger.warning(
                f"[MobileSAM] RKNN encoder model at {rknn_path.resolve()} is an un-pulled Git LFS pointer "
                f"({rknn_path.stat().st_size} bytes). Run 'git lfs pull' to download actual model weight. "
                f"Falling back to ONNX/PyTorch."
            )
            device = "onnx" if (Path(onnx_encoder_path).exists() and Path(onnx_decoder_path).exists()) else "cpu"
        else:
            try:
                ckpt_p = Path(checkpoint)
                rknn_size = f"{rknn_path.stat().st_size / (1024 * 1024):.1f} MB"
                ckpt_size = f"{ckpt_p.stat().st_size / (1024 * 1024):.1f} MB" if ckpt_p.exists() else "missing"
                logger.info(f"[MobileSAM] Loading RKNN Backend (Rockchip RK3588 NPU):")
                logger.info(f"[MobileSAM]   * Encoder (.rknn): {rknn_path.resolve()} ({rknn_size})")
                logger.info(f"[MobileSAM]   * Decoder (.pt):   {ckpt_p.resolve()} ({ckpt_size})")
                predictor = RKNNMobileSAMPredictor(
                    rknn_encoder_path=str(rknn_path),
                    checkpoint=checkpoint,
                )
            except Exception as e:
                logger.warning(
                    f"[MobileSAM] RKNN MobileSAM backend not available: {e}. Falling back to ONNX/PyTorch."
                )
                logger.debug("[MobileSAM] RKNN initialization error details:", exc_info=True)
                device = "onnx" if (Path(onnx_encoder_path).exists() and Path(onnx_decoder_path).exists()) else "cpu"

    # 2. ONNX Runtime path (Fast, lightweight inference on non-RK platforms)
    if device == "onnx" and predictor is None:
        enc_p = Path(onnx_encoder_path)
        dec_p = Path(onnx_decoder_path)
        if enc_p.exists() and dec_p.exists():
            if enc_p.stat().st_size < 1024 or dec_p.stat().st_size < 1024:
                logger.warning(
                    f"[MobileSAM] ONNX models at {enc_p.name}/{dec_p.name} are un-pulled Git LFS pointers. "
                    f"Falling back to PyTorch."
                )
                device = "cuda" if torch.cuda.is_available() else "cpu"
            else:
                try:
                    enc_size = f"{enc_p.stat().st_size / (1024 * 1024):.1f} MB"
                    dec_size = f"{dec_p.stat().st_size / (1024 * 1024):.1f} MB"
                    logger.info(f"[MobileSAM] Loading ONNX Runtime Backend:")
                    logger.info(f"[MobileSAM]   * Encoder (.onnx): {enc_p.resolve()} ({enc_size})")
                    logger.info(f"[MobileSAM]   * Decoder (.onnx): {dec_p.resolve()} ({dec_size})")
                    predictor = ONNXMobileSAMPredictor(
                        encoder_path=str(enc_p),
                        decoder_path=str(dec_p),
                    )
                except Exception as e:
                    logger.warning(
                        f"[MobileSAM] ONNX MobileSAM backend not available: {e}. Falling back to PyTorch."
                    )
                    logger.debug("[MobileSAM] ONNX initialization error details:", exc_info=True)
                    device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            logger.warning(
                f"[MobileSAM] ONNX models not found ({enc_p.name}, {dec_p.name}), falling back to PyTorch."
            )
            device = "cuda" if torch.cuda.is_available() else "cpu"

    # 3. PyTorch path (CUDA / MPS / CPU fallback)
    if predictor is None:
        if str(MOBILESAM_DIR) not in sys.path:
            sys.path.insert(0, str(MOBILESAM_DIR))

        ckpt_p = Path(checkpoint)
        ckpt_size = f"{ckpt_p.stat().st_size / (1024 * 1024):.1f} MB" if ckpt_p.exists() else "missing"
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
    def __init__(self, predictor):
        self.predictor = predictor
        self.image_bgr = None
        self.image_rgb = None

    def set_image(self, image_bgr: np.ndarray):
        self.image_bgr = image_bgr
        self.image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        if self.predictor:
            self.predictor.set_image(self.image_rgb)

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
