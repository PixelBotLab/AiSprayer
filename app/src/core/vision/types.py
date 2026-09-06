"""core.vision 核心领域数据模型与几何工具。

集中定义目标检测结果对象与相机内参/点云几何换算函数，
避免跨模块冗余定义与符号漂移。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class Detection:
    """一个检出目标，box 为原图像素坐标 (x1, y1, x2, y2)。"""

    box: Tuple[float, float, float, float]
    cls_id: int
    cls_name: str
    score: float
    # 原图尺寸的 uint8 二值 mask（0/255）；仅 detect(with_masks=True) 时填入
    mask: Optional[np.ndarray] = None

    @property
    def area(self) -> float:
        """包围框像素面积。"""
        return max(0.0, self.box[2] - self.box[0]) * max(0.0, self.box[3] - self.box[1])

    @property
    def center(self) -> Tuple[int, int]:
        """包围框中心像素坐标 (cx, cy)。"""
        return (
            int(round((self.box[0] + self.box[2]) / 2.0)),
            int(round((self.box[1] + self.box[3]) / 2.0)),
        )

    def to_dict(self) -> dict:
        """转换为供 Web 前端交互使用的字典（不含大体积 mask ndarray）。"""
        return {
            "box": [round(float(v), 1) for v in self.box],
            "cls_id": int(self.cls_id),
            "cls": str(self.cls_name),
            "score": round(float(self.score), 3),
        }


def k_matrix_to_intrinsics(k: np.ndarray) -> Tuple[float, float, float, float]:
    """把 3x3 相机内参矩阵 K 转换为 (fx, fy, cx, cy)。

    :param k: 3x3 内参矩阵 (numpy 数组或二维序列)
    :return: (fx, fy, cx, cy)
    """
    arr = np.asarray(k, dtype=np.float64)
    if arr.shape != (3, 3):
        raise ValueError(f"k must be 3x3 matrix, got shape {arr.shape}")
    return float(arr[0, 0]), float(arr[1, 1]), float(arr[0, 2]), float(arr[1, 2])


def depth_to_point_cloud(
    depth: np.ndarray,
    intrinsics: Sequence[float],
) -> np.ndarray:
    """将深度图转换为与原图对齐的 2.5D 点云网格 [H, W, 3] (相机坐标系，单位 mm)。

    保留原始 (H, W) 形状而不提前过滤无效点，便于后续直接使用 2D 掩码布尔矩阵索引。

    :param depth: HxW uint16 或 float 深度图 (单位: mm)
    :param intrinsics: (fx, fy, cx, cy) 内参元组或列表
    :return: [H, W, 3] float32 点云矩阵 (x, y, z)
    """
    fx, fy, cx, cy = float(intrinsics[0]), float(intrinsics[1]), float(intrinsics[2]), float(intrinsics[3])
    h, w = depth.shape[:2]
    v, u = np.mgrid[0:h, 0:w]
    z = depth.astype(np.float32)

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    return np.dstack((x, y, z))
