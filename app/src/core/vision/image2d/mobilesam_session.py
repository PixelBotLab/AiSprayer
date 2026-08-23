import sys
import torch
import numpy as np
import cv2
from pathlib import Path

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

def resolve_device(requested: str | None = None) -> str:
    if requested:
        return requested
    if torch.cuda.is_available():
        return "cuda"
    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return "mps"
    return "cpu"

def load_mobilesam(checkpoint: str = str(DEFAULT_MOBILESAM_WEIGHTS), device: str | None = None):
    device = resolve_device(device)
    if str(MOBILESAM_DIR) not in sys.path:
        sys.path.insert(0, str(MOBILESAM_DIR))
    
    try:
        from mobile_sam import SamPredictor, sam_model_registry
        model = sam_model_registry["vit_t"](checkpoint=checkpoint)
        model.to(device=device)
        model.eval()
        return SamPredictor(model)
    except Exception as e:
        import logging
        logging.getLogger(__name__).error(f"Failed to load MobileSAM: {e}")
        return None

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

