import os
import cv2
import yaml
import numpy as np
import logging
import time
from typing import List, Tuple, Optional

from core.config import sprayer_config
from core.vision import load_mobilesam, MobileSAMSegmenter, get_detector

logger = logging.getLogger(__name__)

class SAMService:
    def __init__(self):
        self.predictor = None
        # MobileSAM predictor 内部只留一份 image embedding，所以只能有一个活跃会话；
        # 旧实现按模板名缓存多个 session，它们共享同一个 predictor，跨模板预测会静默
        # 用到另一个模板的图像，所以改成“单会话 + 当前模板名”。
        self.session: Optional[MobileSAMSegmenter] = None
        self.loaded_template: Optional[str] = None

    def initialize(self):
        """Loads the MobileSAM weights and the wissight detector on server startup."""
        if self.predictor is not None:
            return
        try:
            logger.info("Initializing MobileSAM model on server startup...")
            t0 = time.time()
            self.predictor = load_mobilesam()
            if self.predictor:
                desc = getattr(self.predictor, "backend_desc", type(self.predictor).__name__)
                logger.info(f"MobileSAM model initialized successfully in {time.time() - t0:.2f}s [{desc}].")
            else:
                logger.error("MobileSAM initialization returned None.")
        except Exception as e:
            logger.error(f"Failed to initialize MobileSAM: {e}", exc_info=True)
        self._warm_up_detector()

    def _warm_up_detector(self):
        """启动时就把 wissight 检测器加载完。

        它是进程内单例，之前只在用户第一次点进分割模式时才构造：建会话的 1~2s 算在
        用户头上，而且启动日志里完全看不到到底加载了哪个模型，所以在这里一并预热并打日志。
        """
        if not sprayer_config.detector_enabled:
            logger.info("[Detect] Wissight detector disabled by config (interactive.detector.enabled=false).")
            return
        try:
            t0 = time.time()
            detector = get_detector()
            if detector is None:
                logger.warning(
                    f"[Detect] Wissight detector unavailable in {time.time() - t0:.2f}s "
                    f"(model missing or failed to load, see log above); segmentation stays manual."
                )
            else:
                logger.info(
                    f"[Detect] Wissight detector initialized successfully in {time.time() - t0:.2f}s "
                    f"[{detector.backend_desc}], sam_refine={sprayer_config.detector_sam_refine}."
                )
        except Exception as e:
            logger.error(f"Failed to initialize wissight detector: {e}", exc_info=True)

    def _ensure_session(self, template_path: str, template_name: str) -> Optional[MobileSAMSegmenter]:
        """返回已编码 template_name 图像的会话；未 init 或已切到其他模板时重新加载图像。
        predictor 只持有一份 embedding，所以这里必须校验当前模板，否则会静默预测另一张图。"""
        if self.session is not None and self.loaded_template == template_name:
            return self.session
        if template_path and self.init_template(template_path, template_name):
            return self.session
        return None

    def init_template(self, template_path: str, template_name: str) -> bool:
        """Loads scan.color.jpg (or legacy scan.jpg) for the template and sets it in the predictor."""
        color_jpg = os.path.join(template_path, "scan.color.jpg")
        legacy_jpg = os.path.join(template_path, "scan.jpg")
        image_path = color_jpg if os.path.exists(color_jpg) else (legacy_jpg if os.path.exists(legacy_jpg) else None)
        if not image_path:
            logger.warning(f"[SAM] Image not found for template '{template_name}' (checked scan.color.jpg and scan.jpg)")
            return False
            
        logger.info(f"[SAM] Loading {os.path.basename(image_path)} for template '{template_name}'...")
        t0 = time.time()
        image_bgr = cv2.imread(image_path)
        if image_bgr is None:
            logger.error(f"[SAM] Failed to decode image: {image_path}")
            return False
            
        h, w = image_bgr.shape[:2]
        if self.predictor is None:
            self.initialize()
        if self.session is None:
            self.session = MobileSAMSegmenter(self.predictor)
        self.session.set_image(image_bgr)
        self.loaded_template = template_name
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

    def detect(self, template_path: str, template_name: str) -> dict:
        """用 wissight 在已编码的模板图上自动检出目标。

        `interactive.detector.sam_refine` 为 true（默认）时只回框，由界面再拿它当 box prompt
        去调 /sam/predict；为 false 时后端直接解出实例 mask 并转成 polygons 一起回传，
        界面拿到就是终稿，不再走 MobileSAM。

        检测器不可用（模型缺失 / 配置关闭）时返回空列表而不是报错，界面据此
        回退到纯手动点选。
        """
        refine = sprayer_config.detector_sam_refine
        detector = get_detector()
        if detector is None:
            logger.info("[Detect] Wissight detector unavailable; fall back to manual clicking.")
            return {"detections": [], "backend": None, "sam_refine": refine}

        session = self._ensure_session(template_path, template_name)
        if session is None or session.image_bgr is None:
            raise RuntimeError(f"Failed to load image for template '{template_name}'")

        dets = detector.detect(session.image_bgr, with_masks=not refine)
        result = {
            "detections": [d.to_dict() for d in dets],
            "backend": detector.backend_desc,
            "sam_refine": refine,
        }
        if refine:
            return result

        # 关精修：面积最大的那个实例 mask 即结果（dets 已按面积降序）
        src = next((d for d in dets if d.mask is not None), None)
        polygons = self._mask_to_polygons(src.mask > 0) if src is not None else []
        result["polygons"] = polygons
        result["score"] = float(src.score) if src is not None else 0.0
        h, w = session.image_bgr.shape[:2]
        area = 0.0 if src is None else float((src.mask > 0).mean())
        logger.info(
            f"[Detect] sam_refine off: using wissight instance mask on {w}x{h} -> "
            f"{len(polygons)} polygon(s), coverage={area * 100:.1f}%"
        )
        if src is not None and not polygons:
            logger.warning("[Detect] instance mask has no valid contour; nothing to show.")
        return result

    def predict_action(
        self,
        template_name: str,
        points: List[List[int]],
        labels: List[int],
        template_path: str = "",
        box: Optional[List[float]] = None,
    ) -> dict:
        """Predict mask based on points and/or a detection box, and return polygons."""
        session = self._ensure_session(template_path, template_name)
        if session is None:
            raise RuntimeError(f"Failed to load SAM session for template '{template_name}'")
        
        # Convert List[List[int]] to List[Tuple[int, int]]
        points_tuples = [(int(p[0]), int(p[1])) for p in points]
        
        n_fg = sum(1 for l in labels if l == 1)
        n_bg = sum(1 for l in labels if l == 0)
        
        t0 = time.time()
        mask, score = session.predict(points_tuples, labels, box=box)
        elapsed = time.time() - t0
        
        if mask is None:
            logger.warning(f"[SAM] Prediction returned empty mask for '{template_name}'.")
            return {"polygons": [], "score": 0.0}
            
        polygons = self._mask_to_polygons(mask)
        logger.info(
            f"[SAM] Prediction for '{template_name}': +fg={n_fg}, -bg={n_bg}, "
            f"box={'-' if box is None else [round(float(v)) for v in box]} -> "
            f"score={score:.3f}, contours={len(polygons)} in {elapsed*1000:.1f}ms"
        )
        return {"polygons": polygons, "score": float(score)}

    @staticmethod
    def _parse_mask_points(mask_data: dict) -> Tuple[List[List[int]], List[int]]:
        """解析提交的点集，兼容两种格式：[[x, y], ...]（yaml 回读）与 [{x, y, label}, ...]（前端）。"""
        pts: List[List[int]] = []
        lbls: List[int] = []
        for p in mask_data.get("points") or []:
            if isinstance(p, dict):
                x, y, label = p.get("x"), p.get("y"), p.get("label")
            elif p and len(p) >= 2:
                x, y, label = p[0], p[1], p[2] if len(p) > 2 else None
            else:
                continue
            if x is None or y is None:
                continue
            pts.append([int(x), int(y)])
            lbls.append(int(label) if label is not None else 1)
        return pts, lbls

    @staticmethod
    def _parse_box(mask_data: dict) -> Optional[Tuple[float, float, float, float]]:
        """解析可选的检测框 (x1, y1, x2, y2)，并规范化成左上/右下顺序。"""
        box = mask_data.get("box")
        if not box or len(box) != 4:
            return None
        x1, y1, x2, y2 = (float(v) for v in box)
        return (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))

    @staticmethod
    def _paint_polygons(image: np.ndarray, polygons: list, color: np.ndarray, alpha: float) -> None:
        """把 polygons 半透明画进可视化图（in-place）。"""
        tmp = np.zeros(image.shape[:2], dtype=np.uint8)
        for poly in polygons:
            arr = np.asarray(poly, dtype=np.int32)
            if arr.ndim == 2 and len(arr) >= 3:
                cv2.fillPoly(tmp, [arr], 255)
        covered = tmp > 0
        image[covered] = image[covered] * (1.0 - alpha) + color * alpha

    def save_masks(self, template_path: str, template_name: str, committed_masks: List[dict]) -> bool:
        """
        committed_masks format:
        [
            {
                "id": 1,
                "points": [[x, y], ...],
                "labels": [1, 0, ...],
                "box": [x1, y1, x2, y2],   # 可选，wissight 检测框
            }, ...
        ]
        落盘前会用点提示重跑一次预测，所以参与预测的 box 必须一起写进 yaml，否则页面
        所见 ≠ 磁盘结果；没有点提示的条目（纯 box / 纯 polygons）则原样保留、不重跑。
        """
        logger.info(f"[SAM] Starting mask save for template '{template_name}' ({len(committed_masks)} mask(s))...")
        session = self._ensure_session(template_path, template_name)
        if session is None:
            logger.error(f"[SAM] Unable to load image for template '{template_name}'.")
            return False
                
        yaml_data = {"version": "1.0", "template_name": template_name, "masks": []}
        
        vis_image = session.image_bgr.copy().astype(np.float32)
        alpha = 0.5
        colors = [
            (0, 220, 0), (220, 0, 0), (0, 0, 220), (220, 220, 0), (220, 0, 220), (0, 220, 220)
        ]
        
        for idx, mask_data in enumerate(committed_masks):
            pts, point_labels = self._parse_mask_points(mask_data)
            # 顶层 labels 优先（前端与 yaml 都会带），缺失时退回点里内嵌的 label，不足的补前景
            lbls = [int(l) for l in (mask_data.get("labels") or []) if l is not None] or point_labels
            lbls = lbls[: len(pts)] + [1] * max(0, len(pts) - len(lbls))
            box = self._parse_box(mask_data)
            color = np.array(colors[idx % len(colors)], dtype=np.float32)

            if not pts:
                # 没有点提示就不重放：box-only 的 SAM 结果是幂等的（实测重放 polygons 完全
                # 一致），而关精修时 polygons 来自 wissight 自己的 mask，重跑反而会把
                # 页面所见换成另一个结果。手填/外部工具产出的 polygons 也走这里原样保留。
                polygons = mask_data.get("polygons") or []
                if polygons:
                    self._paint_polygons(vis_image, polygons, color, alpha)
                    entry = {
                        "id": idx + 1,
                        "points": [],
                        "labels": [],
                        "score": float(mask_data.get("score", 1.0)),
                        "polygons": polygons,
                    }
                    if box is not None:
                        entry["box"] = [round(v, 1) for v in box]
                    yaml_data["masks"].append(entry)
                    logger.info(
                        f"[SAM] Mask #{idx+1} kept as-is (no point prompt): {len(polygons)} polygon(s)"
                    )
                continue
            
            pts_tuples = [(p[0], p[1]) for p in pts]
            mask, score = session.predict(pts_tuples, lbls, box=box)
            if mask is None or not mask.any():
                logger.warning(f"[SAM] Mask #{idx+1} produced an empty mask, dropped from yaml.")
                continue

            vis_image[mask] = vis_image[mask] * (1.0 - alpha) + color * alpha
            polygons = self._mask_to_polygons(mask)

            entry = {
                "id": idx + 1,
                "points": pts,
                "labels": lbls,
                "score": float(score),
                "polygons": polygons,
            }
            if box is not None:
                entry["box"] = [round(v, 1) for v in box]
            yaml_data["masks"].append(entry)
            logger.info(
                f"[SAM] Mask #{idx+1} processed: {len(pts)} points ({lbls}), "
                f"box={'-' if box is None else [round(v) for v in box]}, "
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
            from core.utils.fast_yaml import fast_yaml_load
            with open(yaml_path, 'r', encoding='utf-8') as f:
                return fast_yaml_load(f)
        except Exception as e:
            logger.error(f"[SAM] Failed to parse masks yaml {yaml_path}: {e}")
            return None

# Global service instance
sam_service = SAMService()
