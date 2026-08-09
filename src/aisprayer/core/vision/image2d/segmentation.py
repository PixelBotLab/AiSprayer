import cv2
import numpy as np
import os

class YoloTrousersSegmenter:
    """
    基于 YOLO 语义分割模型 (wissight.pt) 提取裤子的物理边缘。
    这种方式比关键点连线更精准，且对背面、侧面具有极强的鲁棒性。
    """
    def __init__(self, model_path="models/wissight.pt"):
        # 确保路径指向根目录下的 models 文件夹
        if not os.path.isabs(model_path):
            model_path = os.path.join(os.getcwd(), model_path)
            
        print(f"[*] YoloSegmenter: 正在从 {model_path} 加载权重...")
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            print("[+] YoloSegmenter: 模型加载成功")
        except Exception as e:
            print(f"[-] YoloSegmenter: 模型加载失败! 将无法使用语义分割功能。错误: {e}")
            self.model = None

    def get_silhouette_polygon(self, image, conf=0.5):
        """
        执行推理并返回裤子的多边形近似轮廓。
        """
        if self.model is None:
            return None
            
        # 推理
        results = self.model.predict(image, conf=conf, verbose=False)
        
        if not results or results[0].masks is None:
            print("[!] YoloSegmenter: 未在当前帧中识别到有效掩模")
            return None
        
        # 提取最大轮廓的多边形坐标 (xy 格式)
        masks = results[0].masks.xy
        if len(masks) == 0:
            return None
            
        # 筛选面积最大的掩模
        max_mask = max(masks, key=lambda x: cv2.contourArea(x.astype(np.float32)))
        
        # 多边形近似，简化边缘点
        epsilon = 0.002 * cv2.arcLength(max_mask.astype(np.float32), True)
        approx = cv2.approxPolyDP(max_mask.astype(np.float32), epsilon, True)
        
        return approx.reshape(-1, 2).astype(np.int32)

class TrousersSegmenter:
    """
    负责从 2D 图像中提取裤子的物理轮廓 (Silhouette)。
    作为关键点识别的补充，特别是在侧面重叠等复杂工况下提供鲁棒的辅助。
    """
    def __init__(self):
        print("[*] Segmenter: 基础轮廓处理引擎已启动")

    def get_silhouette_grabcut(self, image, bbox, keypoints):
        """
        使用 GrabCut 算法辅助提取轮廓 (当 YOLO 不可用或需精修时)。
        """
        print("[*] Segmenter: 正在执行 GrabCut 辅助提取...")
        x1, y1, x2, y2 = map(int, bbox)
        h, w = image.shape[:2]
        
        # 1. 自动外扩 Bbox
        bw, bh = x2 - x1, y2 - y1
        x1 = max(0, int(x1 - bw * 0.1)); y1 = max(0, int(y1 - bh * 0.1))
        x2 = min(w, int(x2 + bw * 0.1)); y2 = min(h, int(y2 + bh * 0.1))

        # 2. 准备掩模
        mask = np.zeros((h, w), np.uint8)
        rect = (x1, y1, x2-x1, y2-y1)
        
        # 3. 利用关键点设为确定前景 (FGD)
        for kpt in keypoints:
            kx, ky = int(kpt[0]), int(kpt[1])
            if 0 <= kx < w and 0 <= ky < h:
                cv2.circle(mask, (kx, ky), 10, cv2.GC_FGD, -1)

        # 4. 运行迭代
        bgdModel = np.zeros((1, 65), np.float64)
        fgdModel = np.zeros((1, 65), np.float64)
        cv2.grabCut(image, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
        cv2.grabCut(image, mask, rect, bgdModel, fgdModel, 2, cv2.GC_INIT_WITH_MASK)

        # 5. 提取最大轮廓
        mask_final = np.where((mask == 1) | (mask == 3), 255, 0).astype('uint8')
        contours, _ = cv2.findContours(mask_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours: return None
        best_cnt = max(contours, key=cv2.contourArea)
        
        epsilon = 0.002 * cv2.arcLength(best_cnt, True)
        approx = cv2.approxPolyDP(best_cnt, epsilon, True)
        return approx.reshape(-1, 2).astype(np.int32)

    def is_pose_ambiguous(self, keypoints, bbox):
        """
        姿态模糊判定逻辑：
        A. 检查置信度; B. 检查关键点重叠; C. 侧面重叠判定; D. 左右腿翻转判定
        """
        box_w = bbox[2] - bbox[0]
        
        # A. 置信度检查
        if np.sum(keypoints[:, 2] < 0.4) >= 4:
            print("[!] 判定: 置信度过低 (Low Confidence)")
            return True

        # B. 语义混淆判定 (背面视角常见)
        dist_4_14 = np.linalg.norm(keypoints[3, :2] - keypoints[13, :2])
        if dist_4_14 < box_w * 0.05:
            print("[!] 判定: 关键点 4/14 语义重叠，怀疑为背面")
            return True

        # C. 左右腿翻转判定 (最硬标准)
        poly_indices = [0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 2, 1]
        poly_pts = np.array([keypoints[i, :2] for i in poly_indices], dtype=np.float32)
        left_leg_avg = np.mean(poly_pts[1:7, 0])
        right_leg_avg = np.mean(poly_pts[8:13, 0])
        if left_leg_avg > right_leg_avg:
            print("[!] 判定: 左右腿关键点层级翻转 (Flipped Legs)")
            return True

        return False
