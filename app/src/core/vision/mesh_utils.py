"""网格几何公共工具 (Mesh Geometry Utils)。

下沉与 trimesh 顶点/面索引强相关的纯几何清洗逻辑，供重建器（导出侧治本）与
规划器（消费侧防御）复用，避免平行实现。仅依赖 numpy，保持高扇入、零额外负担。
"""

from __future__ import annotations

from typing import Tuple

import numpy as np


def drop_non_finite_vertices(
    vertices: np.ndarray, faces: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, int]:
    """剔除坐标含 NaN/Inf 的顶点，并同步丢弃引用了这些顶点的面。

    泊松重建 + Taubin 平滑在退化/孤立边界点上会偶发产生非有限坐标顶点；此类顶点
    一旦被 cKDTree 等要求"全有限输入"的下游算子读到即抛 ValueError，导致整条自动
    路径规划失败。这里在顶点层面做一次无损清理：仅移除坏顶点及其引用面，其余几何不变。

    :param vertices: [N, 3] 顶点坐标
    :param faces: [M, 3] 三角面顶点索引
    :return: (保留顶点数组, 索引重映射后的面数组, 被剔除的顶点数)；无坏点时原样返回
    """
    finite = np.isfinite(vertices).all(axis=1)
    if bool(finite.all()):
        return vertices, faces, 0

    n_removed = int(vertices.shape[0] - finite.sum())
    # 旧顶点索引 -> 新顶点索引的紧凑重映射 (被删顶点映射为 -1)
    remap = np.full(vertices.shape[0], -1, dtype=np.int64)
    remap[finite] = np.arange(int(finite.sum()))
    # 仅保留三个顶点全部有限的面，避免悬空索引
    keep_faces = finite[faces].all(axis=1)
    return vertices[finite], remap[faces[keep_faces]], n_removed
