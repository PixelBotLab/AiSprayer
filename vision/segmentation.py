import cv2
import numpy as np

class TrousersSegmenter:
    """
    负责从 2D 图像中提取裤子的物理轮廓 (Silhouette)。
    作为关键点识别的补充，特别是在侧面重叠等复杂工况下提供鲁棒的喷涂区域。
    """
    def __init__(self):
        pass

    def get_silhouette_polygon(self, image, bbox, keypoints):
        """
        使用 GrabCut 算法精确提取裤子轮廓。
        GrabCut 结合了颜色分布建模和空间平滑性，比简单的阈值分割更精确。
        """
        x1, y1, x2, y2 = map(int, bbox)
        h, w = image.shape[:2]
        
        # 0. 自动外扩 Bbox (增加 10% 容错空间)，防止框太紧切掉裤脚或边缘
        bw, bh = x2 - x1, y2 - y1
        x1 = max(0, int(x1 - bw * 0.1))
        y1 = max(0, int(y1 - bh * 0.1))
        x2 = min(w, int(x2 + bw * 0.1))
        y2 = min(h, int(y2 + bh * 0.1))

        # 1. 准备 GrabCut 所需的掩模
        mask = np.zeros((h, w), np.uint8)
        
        # 设定矩形区域为“可能前景”
        rect = (x1, y1, x2-x1, y2-y1)
        
        # 2. 利用关键点作为“确定前景”种子
        # 增大标记半径，更强力地保护关键点所在的区域
        for kpt in keypoints:
            kx, ky = int(kpt[0]), int(kpt[1])
            if 0 <= kx < w and 0 <= ky < h:
                cv2.circle(mask, (kx, ky), 10, cv2.GC_FGD, -1)

        # 3. 运行 GrabCut
        bgdModel = np.zeros((1, 65), np.float64)
        fgdModel = np.zeros((1, 65), np.float64)
        
        # 第一次迭代使用矩形初始化
        cv2.grabCut(image, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
        # 第二次迭代结合我们手动标记的关键点种子
        cv2.grabCut(image, mask, rect, bgdModel, fgdModel, 2, cv2.GC_INIT_WITH_MASK)

        # 4. 提取最终掩模 (将确定前景和可能前景都选出来)
        # 0和2是背景，1和3是前景
        mask_final = np.where((mask == 1) | (mask == 3), 255, 0).astype('uint8')

        # 5. 形态学清理
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_final = cv2.morphologyEx(mask_final, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 6. 提取最大轮廓
        contours, _ = cv2.findContours(mask_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        
        # 筛选包含关键点最多的轮廓 (防止抓到远处的杂物)
        best_cnt = None
        max_kpts_inside = -1
        for cnt in contours:
            kpts_inside = 0
            for kpt in keypoints:
                if cv2.pointPolygonTest(cnt, (float(kpt[0]), float(kpt[1])), False) >= 0:
                    kpts_inside += 1
            if kpts_inside > max_kpts_inside:
                max_kpts_inside = kpts_inside
                best_cnt = cnt

        if best_cnt is None:
            return None

        # 7. 多边形近似
        epsilon = 0.002 * cv2.arcLength(best_cnt, True)
        approx = cv2.approxPolyDP(best_cnt, epsilon, True)

        return approx.reshape(-1, 2).astype(np.int32)

    def is_pose_ambiguous(self, keypoints, bbox):
        """
        根据关键点判定当前姿态是否模糊或识别错误 (例如侧面重叠、背面语义混乱)。
        """
        # A. 检查置信度
        low_conf_count = np.sum(keypoints[:, 2] < 0.4)
        if low_conf_count >= 4:
            return True

        # B. 检查 4 号点 (idx 3) 和 14 号点 (idx 13) 的距离
        # 在背面视角，这两个点经常会因为语义混淆而“撞”在一起
        kpt_4 = keypoints[3, :2]
        kpt_14 = keypoints[13, :2]
        dist_4_14 = np.linalg.norm(kpt_4 - kpt_14)
        box_w = bbox[2] - bbox[0]
        
        if dist_4_14 < box_w * 0.05:
            print("[!] 检测到关键点 4 和 14 发生重叠，可能为背面语义混乱")
            return True

        # C. 检查关键点聚类 (侧面重叠判定)
        kpt_l_knee = keypoints[4, :2]
        kpt_r_knee = keypoints[12, :2]
        dist_x = abs(kpt_l_knee[0] - kpt_r_knee[0])
        if dist_x < box_w * 0.08:
            return True

        # D. 检查多边形是否发生“自相交” (Self-intersection)
        # 这是判定“识别乱了”最硬的标准
        poly_indices = [0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 2, 1]
        poly_pts = np.array([keypoints[i, :2] for i in poly_indices], dtype=np.float32)
        
        # 简化版自相交检测：检查左右腿的中点连线是否反向
        # 正常情况下，左腿重心应该在右腿重心左侧
        left_leg_avg = np.mean(poly_pts[1:7, 0])
        right_leg_avg = np.mean(poly_pts[8:13, 0])
        if left_leg_avg > right_leg_avg:
            print("[!] 检测到左右腿关键点层级翻转，切换至轮廓模式")
            return True

        # E. 检查面积比
        poly_area = cv2.contourArea(poly_pts)
        box_area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
        if poly_area < box_area * 0.1:
            return True

        return False
