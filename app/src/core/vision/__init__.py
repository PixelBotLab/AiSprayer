"""core.vision: 现代化、扁平化、以 Web 为中心的视觉处理核心模块。

主要提供：
- 目标检测 (WissightDetector / get_detector)
- 交互分割 (MobileSAMSegmenter / load_mobilesam)
- 表面重建 (SurfaceReconstructor)
- 航点规划 (WaypointPlanner / WaypointPlannerError / split_jeans_mask)
- 基础类型与几何工具 (Detection / k_matrix_to_intrinsics / depth_to_point_cloud)
"""

from core.vision.detector import WissightDetector, get_detector
from core.vision.planner import (
    WaypointPlanner,
    WaypointPlannerError,
    split_jeans_mask,
)
from core.vision.reconstructor import SurfaceReconstructor
from core.vision.segmenter import (
    MobileSAMSegmenter,
    load_mobilesam,
)
from core.vision.types import (
    Detection,
    depth_to_point_cloud,
    k_matrix_to_intrinsics,
)

__all__ = [
    # 目标检测
    "WissightDetector",
    "get_detector",
    # 交互分割
    "MobileSAMSegmenter",
    "load_mobilesam",
    # 表面重建
    "SurfaceReconstructor",
    # 航点规划
    "WaypointPlanner",
    "WaypointPlannerError",
    "split_jeans_mask",
    # 基础类型与工具
    "Detection",
    "k_matrix_to_intrinsics",
    "depth_to_point_cloud",
]
