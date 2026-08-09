import cv2
import numpy as np

from aisprayer.core.vision.image2d.jeans_segmentation import split_jeans_mask

class JeansZigzagSampler:
    """
    牛仔裤之字形（Zigzag）采点器
    功能：根据输入的 2D 掩码图像，自动将裤子切分为两条裤腿，
    并计算每条裤腿的主成分（PCA）方向。随后沿着各自的主方向进行纵列的之字形采点。
    """
    
    def __init__(self):
        pass

    def sample(self, mask, row_spacing, point_spacing, overlap=0):
        """
        对输入掩码进行之字形采点
        
        :param mask: 2D 二值掩码 (H, W)，非零像素表示目标区域
        :param row_spacing: 行间间距（横向列与列之间的距离，像素单位）
        :param point_spacing: 点之间的间距（同一纵列上的点间距，像素单位）
        :param overlap: 两条腿在中线分割时的重叠像素（0表示硬切分）
        :return: 返回一个列表，列表的每一项对应一条裤腿的采点结果，
                 采点结果本身也是一个列表，包含一系列的 (x, y) 坐标点。
                 即 List[List[Tuple[float, float]]]
        """
        # 确保掩码是布尔类型
        if mask.dtype != bool:
            mask_bool = mask > 0
        else:
            mask_bool = mask

        # 使用已有的切分逻辑将裤子掩码切分为 1 或 2 条腿
        leg_masks = split_jeans_mask(mask_bool, overlap_px=overlap)
        
        all_legs_points = []

        for leg_mask in leg_masks:
            points = self._sample_single_leg(leg_mask, row_spacing, point_spacing)
            if points:
                all_legs_points.append(points)
                
        return all_legs_points

    def _sample_single_leg(self, mask, row_spacing, point_spacing):
        """
        对单条裤腿进行纵向之字形采点
        """
        # 获取所有的前景点坐标 (y, x)
        ys, xs = np.where(mask)
        if len(ys) == 0:
            return []
            
        # 构造 (N, 2) 的点集，对应 (x, y)
        pts = np.vstack((xs, ys)).T.astype(np.float32)
        
        # 计算 PCA 主成分分析
        mean, eigenvectors = cv2.PCACompute(pts, mean=None)
        
        # eigenvectors 的第一行是主成分（方差最大，即裤腿的纵向延伸方向）
        # 第二行是次成分（即横向）
        v_main = eigenvectors[0]
        v_trans = eigenvectors[1]
        
        # 构造旋转矩阵，将原始坐标 (x,y) 映射到局部 UV 坐标系下
        # u 为横向 (v_trans), v 为纵向 (v_main)
        R = np.vstack((v_trans, v_main))
        
        # 将所有点投影到 UV 坐标系下
        # uv_pts = (pts - mean) * R.T
        uv_pts = np.dot(pts - mean.flatten(), R.T)
        
        # 获取 UV 坐标系下的包围盒
        u_min, u_max = np.min(uv_pts[:, 0]), np.max(uv_pts[:, 0])
        v_min, v_max = np.min(uv_pts[:, 1]), np.max(uv_pts[:, 1])
        
        # 在 u 方向 (横向) 上按 row_spacing 生成扫描列
        # 为了让列对称，我们可以从 0 向两边延展，或者直接从 u_min 到 u_max
        u_steps = np.arange(u_min, u_max, row_spacing)
        
        zigzag_points_orig = []
        direction_down = True
        
        h, w = mask.shape
        
        for u in u_steps:
            # 在纵向 v 上采样
            v_steps = np.arange(v_min, v_max, point_spacing)
            
            # 交替方向实现之字形 (Zigzag)
            if not direction_down:
                v_steps = v_steps[::-1]
                
            col_points = []
            for v in v_steps:
                # 将 (u, v) 逆变换回原图像坐标 (x, y)
                orig_pt = np.dot(np.array([u, v]), R) + mean.flatten()
                
                # 检查逆变换后的点是否在图像边界内且在 mask 内
                x_idx, y_idx = int(round(orig_pt[0])), int(round(orig_pt[1]))
                if 0 <= x_idx < w and 0 <= y_idx < h:
                    if mask[y_idx, x_idx]:
                        # 保留浮点数坐标，更加精确
                        col_points.append((float(orig_pt[0]), float(orig_pt[1])))
                        
            # 如果当前列采样到了有效点，才加入总列表，并翻转下一次的扫描方向
            if col_points:
                zigzag_points_orig.extend(col_points)
                direction_down = not direction_down
                
        return zigzag_points_orig
