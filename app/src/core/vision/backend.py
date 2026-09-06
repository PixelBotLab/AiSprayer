"""跨平台推理后端（RK3588 NPU / ONNX Runtime / PyTorch）的公共底座。

凡是"这台机器该用哪个后端""NPU 运行时怎么起来"的判断都只在这里实现一次，
MobileSAM（image2d/mobilesam_session.py）与 Wissight 检测器
（image2d/wissight_detector.py）共用，避免各写一份、行为漂移。
"""
import logging
import os
import platform
from pathlib import Path
from typing import Callable, Iterable, Optional

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[4]
MODELS_DIR = REPO_ROOT / "models"


def is_rk3588() -> bool:
    """是否运行在 Rockchip RK3588 上（有 NPU 可用）。"""
    # 1. 设备树 model
    dt_model_path = Path("/proc/device-tree/model")
    if dt_model_path.exists():
        try:
            model_str = dt_model_path.read_text(errors="ignore").lower()
            if "rk3588" in model_str or "orange pi 5" in model_str:
                return True
        except Exception:
            pass

    # 2. 设备树 compatible
    dt_compat_path = Path("/proc/device-tree/compatible")
    if dt_compat_path.exists():
        try:
            compat_str = dt_compat_path.read_text(errors="ignore").lower()
            if "rk3588" in compat_str:
                return True
        except Exception:
            pass

    # 3. 架构 + NPU 设备节点
    if platform.machine() in ("aarch64", "arm64"):
        if Path("/dev/rknpu").exists() or Path("/dev/mpp_service").exists():
            return True

    return False


def model_ready(path) -> bool:
    """模型文件存在、且不是未拉取的 Git LFS 指针文件。"""
    p = Path(path)
    return p.exists() and p.stat().st_size > 1024


def size_mb(path) -> str:
    return f"{Path(path).stat().st_size / (1024 * 1024):.1f} MB"


def torch_device() -> Optional[str]:
    """torch 可用时返回 'cuda'/'cpu'；未安装（如 docker slim 镜像）时返回 None 交给自动探测。"""
    try:
        import torch
    except Exception:
        # 不能只抓 ImportError：见 load_rknn_runtime 里 logging 污染的说明，
        # 晚于 RKNN 导入 torch 可能抛 ValueError 而不是 ImportError。
        return None
    return "cuda" if torch.cuda.is_available() else "cpu"


def normalize_backend(name: Optional[str]) -> Optional[str]:
    """把用户写的后端别名归一化；'auto'/空返回 None，表示交给自动探测。"""
    if not name:
        return None
    t = str(name).lower().strip()
    if t in ("pt", "pytorch", "torch"):
        return torch_device()
    if t in ("auto", "default"):
        return None
    return t


def pick_backend(
    requested: Optional[str],
    env_names: Iterable[str],
    cfg_value: Optional[str],
    auto: Callable[[], str],
) -> str:
    """后端优先级：显式入参 > 环境变量 > 配置文件 > auto() 自动探测。"""
    for value in (requested, *(os.getenv(n) for n in env_names), cfg_value):
        norm = normalize_backend(value)
        if norm:
            return norm
    return auto()


# 标准 logging 等级名：rknn-toolkit-lite2 会把这些名字从全局名字表里删掉
_STANDARD_LEVEL_NAMES = (
    "CRITICAL", "FATAL", "ERROR", "WARNING", "INFO", "DEBUG", "NOTSET", "WARN",
)


def load_rknn_runtime(model_path):
    """加载 RKNN 运行时：板上优先 rknnlite，PC/仿真环境退回完整版 rknn-toolkit2。

    坑（必须知道再改）：rknn-toolkit-lite2 在 import 时会把 `logging._nameToLevel` 里的
    标准等级名（WARNING/ERROR/INFO/...）删掉、换成单字母，于是之后任何
    `logger.setLevel("WARNING")` 都会抛 `ValueError: Unknown level: 'WARNING'` ——
    其中包括**晚一步才被导入的 torch**（torch.fx 的 matcher_utils 就这么干）。
    所以 import 前后做一次快照，只把被删掉的标准名字补回去（不覆盖 RKNN 自己新增的键），
    把污染限制在这一次 import 内。

    :return: 运行时对象（RKNNLite 或 RKNN）
    :raises RuntimeError: 两个模块都不可用或模型加载失败
    """
    saved = {n: logging._nameToLevel[n] for n in _STANDARD_LEVEL_NAMES if n in logging._nameToLevel}
    try:
        try:
            from rknnlite.api import RKNNLite

            rknn = RKNNLite()
            ret = rknn.load_rknn(str(model_path))
            if ret != 0:
                raise RuntimeError(f"RKNNLite.load_rknn failed, code: {ret}")
            ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
            if ret != 0:
                raise RuntimeError(f"RKNNLite.init_runtime failed, code: {ret}")
            logger.info(f"[RKNN] RKNNLite NPU runtime initialized (AUTO cores), model={Path(model_path).name}")
            return rknn
        except ImportError:
            pass

        try:
            from rknn.api import RKNN

            rknn = RKNN(verbose=False)
            ret = rknn.load_rknn(str(model_path))
            if ret != 0:
                raise RuntimeError(f"RKNN.load_rknn failed, code: {ret}")
            ret = rknn.init_runtime()
            if ret != 0:
                raise RuntimeError(f"RKNN.init_runtime failed, code: {ret}")
            logger.info(f"[RKNN] RKNN runtime initialized, model={Path(model_path).name}")
            return rknn
        except ImportError:
            pass

        raise RuntimeError(
            "Neither 'rknnlite' nor 'rknn' Python module is available. "
            "Install rknn-toolkit-lite2 on RK3588 (or rknn-toolkit2 on PC)."
        )
    finally:
        for name, level in saved.items():
            logging._nameToLevel.setdefault(name, level)
