from ultralytics import YOLO
import cv2
import numpy as np
import os

class YoloTrousersSegmenter:
    """
    使用 YOLO 语义分割模型 (wissight.pt) 提取裤子的物理边缘。
    这种方式比关键点连线更精准，且对背面、侧面具有极强的鲁棒性。
    """
    def __init__(self, model_path="models/wissight.pt", device=None):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"找不到 YOLO 模型文件: {model_path}")
        
        # 加载模型
        self.model = YOLO(model_path)
        self.device = device
        
    def get_silhouette_polygon(self, image, conf=0.5):
        """
        从图像中提取最像裤子的轮廓。
        :param image: BGR 图像
        :param conf: 置信度阈值
        :return: np.array (Nx2) 轮廓多边形，如果没有检测到则返回 None
        """
        results = self.model.predict(image, conf=conf, device=self.device, verbose=False)
        
        if not results or results[0].masks is None:
            return None
        
        # 寻找面积最大的掩模 (假设就是我们要喷涂的裤子)
        masks = results[0].masks.xy  # 得到多边形坐标列表
        if len(masks) == 0:
            return None
            
        # 按照面积排序
        # 注意：xy 里的坐标已经是原图坐标系的像素值
        max_mask = max(masks, key=lambda x: cv2.contourArea(x.astype(np.float32)))
        
        # 进行多边形近似以减少点数，平滑边缘
        epsilon = 0.002 * cv2.arcLength(max_mask.astype(np.float32), True)
        approx = cv2.approxPolyDP(max_mask.astype(np.float32), epsilon, True)
        
        return approx.reshape(-1, 2).astype(np.int32)

if __name__ == "__main__":
    # 测试代码
    import sys
    test_img = "vision/images/180.jpg"
    if not os.path.exists(test_img):
        print(f"找不到测试图: {test_img}")
        sys.exit(1)
        
    img = cv2.imread(test_img)
    try:
        segmenter = YoloTrousersSegmenter()
        poly = segmenter.get_silhouette_polygon(img)
        if poly is not None:
            print(f"成功提取轮廓，点数: {len(poly)}")
            cv2.polylines(img, [poly], True, (255, 0, 0), 2)
            cv2.imwrite("yolo_seg_test.jpg", img)
            print("结果已保存至 yolo_seg_test.jpg")
        else:
            print("未检测到裤子区域")
    except Exception as e:
        print(f"运行失败: {e}")
