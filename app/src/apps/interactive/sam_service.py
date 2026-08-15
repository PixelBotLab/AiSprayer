import os
import cv2
import yaml
import numpy as np
import logging
import time
from typing import Dict, List, Tuple, Optional

from core.vision.image2d.mobilesam_session import load_mobilesam, MobileSAMSession

logger = logging.getLogger(__name__)

class SAMService:
    def __init__(self):
        self.predictor = None
        self.sessions: Dict[str, MobileSAMSession] = {}

    def initialize(self):
        """Loads the MobileSAM weights into predictor on server startup."""
        if self.predictor is not None:
            return
        try:
            logger.info("Initializing MobileSAM model on server startup...")
            t0 = time.time()
            self.predictor = load_mobilesam()
            if self.predictor:
                logger.info(f"MobileSAM model initialized successfully in {time.time() - t0:.2f}s.")
            else:
                logger.error("MobileSAM initialization returned None.")
        except Exception as e:
            logger.error(f"Failed to initialize MobileSAM: {e}", exc_info=True)

    def get_session(self, template_name: str) -> MobileSAMSession:
        if self.predictor is None:
            self.initialize()
        if template_name not in self.sessions:
            self.sessions[template_name] = MobileSAMSession(self.predictor)
        return self.sessions[template_name]

    def init_template(self, template_path: str, template_name: str) -> bool:
        """Loads scan.jpg for the template and sets it in the predictor."""
        image_path = os.path.join(template_path, "scan.jpg")
        if not os.path.exists(image_path):
            logger.warning(f"[SAM] Image not found for template '{template_name}': {image_path}")
            return False
            
        logger.info(f"[SAM] Loading scan.jpg for template '{template_name}'...")
        t0 = time.time()
        image_bgr = cv2.imread(image_path)
        if image_bgr is None:
            logger.error(f"[SAM] Failed to decode image: {image_path}")
            return False
            
        h, w = image_bgr.shape[:2]
        session = self.get_session(template_name)
        session.set_image(image_bgr)
        logger.info(f"[SAM] MobileSAM image encoded for '{template_name}' ({w}x{h}) in {time.time() - t0:.2f}s.")
        return True

    def _mask_to_polygons(self, mask: np.ndarray) -> List[List[List[int]]]:
        """Convert a boolean mask to a list of polygons using cv2.findContours."""
        mask_uint8 = (mask * 255).astype(np.uint8)
        contours, _ = cv2.findContours(mask_uint8, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        polygons = []
        for cnt in contours:
            cnt = cnt.squeeze(1)  # Nx2
            # Need at least 3 points for a valid polygon
            if len(cnt.shape) == 2 and cnt.shape[0] >= 3:
                polygons.append(cnt.tolist())
        return polygons

    def predict_action(self, template_name: str, points: List[List[int]], labels: List[int]) -> dict:
        """Predict mask based on points and return polygons."""
        session = self.get_session(template_name)
        
        # Convert List[List[int]] to List[Tuple[int, int]]
        points_tuples = [(int(p[0]), int(p[1])) for p in points]
        
        n_fg = sum(1 for l in labels if l == 1)
        n_bg = sum(1 for l in labels if l == 0)
        
        t0 = time.time()
        mask, score = session.predict(points_tuples, labels)
        elapsed = time.time() - t0
        
        if mask is None:
            logger.warning(f"[SAM] Prediction returned empty mask for '{template_name}'.")
            return {"polygons": [], "score": 0.0}
            
        polygons = self._mask_to_polygons(mask)
        logger.info(
            f"[SAM] Prediction for '{template_name}': +fg={n_fg}, -bg={n_bg} -> "
            f"score={score:.3f}, contours={len(polygons)} in {elapsed*1000:.1f}ms"
        )
        return {"polygons": polygons, "score": float(score)}

    def save_masks(self, template_path: str, template_name: str, committed_masks: List[dict]) -> bool:
        """
        committed_masks format:
        [
            {
                "id": 1,
                "points": [[x, y], ...],
                "labels": [1, 0, ...],
            }, ...
        ]
        """
        logger.info(f"[SAM] Starting mask save for template '{template_name}' ({len(committed_masks)} mask(s))...")
        session = self.get_session(template_name)
        if session.image_bgr is None:
            if not self.init_template(template_path, template_name):
                logger.error(f"[SAM] Unable to load image for template '{template_name}'.")
                return False
                
        yaml_data = {"version": "1.0", "template_name": template_name, "masks": []}
        
        vis_image = session.image_bgr.copy().astype(np.float32)
        alpha = 0.5
        colors = [
            (0, 220, 0), (220, 0, 0), (0, 0, 220), (220, 220, 0), (220, 0, 220), (0, 220, 220)
        ]
        
        for idx, mask_data in enumerate(committed_masks):
            # Convert to plain lists of ints to avoid PyYAML !!python/tuple tags
            pts = [[int(p[0]), int(p[1])] for p in mask_data.get("points", [])]
            pts_tuples = [(p[0], p[1]) for p in pts]
            lbls = [int(l) for l in mask_data.get("labels", [])]
            
            if not pts:
                continue
                
            mask, score = session.predict(pts_tuples, lbls)
            if mask is not None and mask.any():
                color = np.array(colors[idx % len(colors)], dtype=np.float32)
                vis_image[mask] = vis_image[mask] * (1.0 - alpha) + color * alpha
                
                polygons = self._mask_to_polygons(mask)
                
                yaml_data["masks"].append({
                    "id": idx + 1,
                    "points": pts,
                    "labels": lbls,
                    "score": float(score),
                    "polygons": polygons
                })
                logger.info(
                    f"[SAM] Mask #{idx+1} processed: {len(pts)} points, "
                    f"score={score:.3f}, {len(polygons)} polygon(s)"
                )
                
        # Save visualization jpg
        output_jpg = os.path.join(template_path, "scan.masks.jpg")
        cv2.imwrite(output_jpg, vis_image.astype(np.uint8))
        jpg_size = os.path.getsize(output_jpg) if os.path.exists(output_jpg) else 0
        logger.info(f"[SAM] Saved mask overlay image: {output_jpg} ({jpg_size / 1024:.1f} KB)")
        
        # Save YAML data
        output_yaml = os.path.join(template_path, "scan.masks.yaml")
        with open(output_yaml, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=None, sort_keys=False)
        yaml_size = os.path.getsize(output_yaml) if os.path.exists(output_yaml) else 0
        logger.info(f"[SAM] Saved mask YAML data: {output_yaml} ({yaml_size / 1024:.1f} KB, {len(yaml_data['masks'])} masks)")
            
        return True

    def get_template_masks(self, template_path: str) -> Optional[dict]:
        """Read and parse scan.masks.yaml if available."""
        yaml_path = os.path.join(template_path, "scan.masks.yaml")
        if not os.path.exists(yaml_path):
            return None
            
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                # Custom loader or full_load to handle tuple tags safely
                data = yaml.full_load(f)
                return data
        except Exception as e:
            logger.error(f"[SAM] Failed to parse masks yaml {yaml_path}: {e}")
            return None

# Global service instance
sam_service = SAMService()
