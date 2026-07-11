import os
import cv2
import numpy as np
from abc import ABC, abstractmethod

# backend/ 目录 (本文件位于 backend/src/ 下)
BACKEND_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
# 项目根目录 (SprayAnything/)，config.yaml / models/ 等都挂在这一层
PROJECT_ROOT = os.path.abspath(os.path.join(BACKEND_ROOT, ".."))


def resolve_project_path(path):
    """把相对路径解析为相对项目根目录 (SprayAnything/) 的绝对路径。

    这样无论脚本从哪个工作目录被执行 (backend/、backend/src/、SprayAnything/ ...)，
    config.yaml / models/ 里配置的相对路径都能被正确找到，不用依赖运行时的 cwd。
    """
    if os.path.isabs(path):
        return path
    return os.path.join(PROJECT_ROOT, path)


class BaseSegmenter(ABC):
    """
    Abstract base class for all garment segmentation algorithms.
    """

    def __init__(self, conf=0.5):
        self.conf = conf

    @abstractmethod
    def get_silhouette_polygon(self, image):
        """执行推理并返回裤子的多边形近似轮廓 (xy 格式)。"""
        pass

    @abstractmethod
    def get_mask(self, image):
        """执行推理并返回与输入图像同分辨率 of 裤子布尔掩码 (H, W)。"""
        pass


class SegmenterFactory:
    _registry = {}

    @classmethod
    def register(cls, name):
        def decorator(subclass):
            cls._registry[name] = subclass
            return subclass
        return decorator

    @classmethod
    def create(cls, name, **kwargs):
        if name not in cls._registry:
            raise ValueError(
                f"Unknown segmenter type: '{name}'. Registered types: {list(cls._registry.keys())}"
            )
        return cls._registry[name](**kwargs)


@SegmenterFactory.register("yolo_trousers")
class YoloTrousersSegmenter(BaseSegmenter):
    """
    基于 YOLO 语义分割模型 (wissight.pt) 提取裤子的物理边缘/掩码。
    这种方式比关键点连线更精准，且对背面、侧面具有极强的鲁棒性。
    """

    def __init__(self, model_path="models/wissight.pt", conf=0.5):
        self.conf = conf
        # 默认相对于项目根目录 (SprayAnything/) 下的 models 文件夹
        model_path = resolve_project_path(model_path)

        print(f"[*] YoloTrousersSegmenter: 正在从 {model_path} 加载权重...")
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            print("[+] YoloTrousersSegmenter: 模型加载成功")
        except Exception as e:
            print(f"[-] YoloTrousersSegmenter: 模型加载失败! 将无法使用语义分割功能。错误: {e}")
            self.model = None

    def get_silhouette_polygon(self, image):
        """执行推理并返回裤子的多边形近似轮廓 (xy 格式)。"""
        if self.model is None:
            return None

        results = self.model.predict(image, conf=self.conf, verbose=False)

        if not results or results[0].masks is None:
            print("[!] YoloTrousersSegmenter: 未在当前帧中识别到有效掩模")
            return None

        masks = results[0].masks.xy
        if len(masks) == 0:
            return None

        # 筛选面积最大的掩模
        max_mask = max(masks, key=lambda x: cv2.contourArea(x.astype(np.float32)))

        # 多边形近似，简化边缘点
        epsilon = 0.002 * cv2.arcLength(max_mask.astype(np.float32), True)
        approx = cv2.approxPolyDP(max_mask.astype(np.float32), epsilon, True)

        return approx.reshape(-1, 2).astype(np.int32)

    def get_mask(self, image):
        """执行推理并返回与输入图像同分辨率的裤子布尔掩码 (H, W)。

        直接对接 VisionProcessor.capture_point_cloud 所需的 yolo_mask_2d。
        """
        if self.model is None:
            return None

        h, w = image.shape[:2]
        results = self.model.predict(image, conf=self.conf, verbose=False)

        if not results or results[0].masks is None:
            print("[!] YoloTrousersSegmenter: 未在当前帧中识别到有效掩模")
            return None

        mask_data = results[0].masks.data  # [num_masks, mh, mw]
        if mask_data is None or len(mask_data) == 0:
            return None

        mask_np = mask_data.cpu().numpy()

        # 筛选面积最大的掩模 (裤子应为画面中的主体)
        areas = mask_np.sum(axis=(1, 2))
        best_mask = mask_np[int(np.argmax(areas))]

        # 模型输出分辨率可能与原图不同，缩放回原图尺寸
        best_mask = cv2.resize(best_mask.astype(np.float32), (w, h), interpolation=cv2.INTER_NEAREST)

        return best_mask > 0.5
