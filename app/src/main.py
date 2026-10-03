from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import sys
import os
import logging
import logging.handlers
import uvicorn
from fastapi.staticfiles import StaticFiles

# Production startup must use one server process. Thread pools are limited below
# before importing OpenCV/PyTorch-backed application modules.
for _name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_name, "1")
os.environ.setdefault("OPENCV_FOR_THREADS_NUM", "1")
os.environ.setdefault("OPENCV_NUM_THREADS", "1")

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src")) # Add original src for aisprayer module

# Configure logging so all modules output to console and file
log_dir = os.path.join(PROJECT_ROOT, "app/logs")
os.makedirs(log_dir, exist_ok=True)

class ColorFormatter(logging.Formatter):
    COLORS = {
        logging.DEBUG: "\033[36m",     # Cyan
        logging.INFO: "\033[32m",      # Green
        logging.WARNING: "\033[33m",   # Yellow
        logging.ERROR: "\033[31m",     # Red
        logging.CRITICAL: "\033[1;31m" # Bold Red
    }
    RESET = "\033[0m"

    def format(self, record):
        color = self.COLORS.get(record.levelno, "")
        record_copy = logging.makeLogRecord(record.__dict__)
        record_copy.levelname = f"{color}{record_copy.levelname}{self.RESET}"
        return super().format(record_copy)

# Console handler with colors
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setFormatter(ColorFormatter("%(asctime)s [%(levelname)s] %(name)s:%(lineno)d: %(message)s", datefmt="%H:%M:%S"))

# File handler without colors (plain text)
file_handler = logging.handlers.TimedRotatingFileHandler(
    os.path.join(log_dir, "backend.log"),
    when="midnight",
    interval=1,
    backupCount=30,  # Keep 30 days of logs
    encoding="utf-8"
)
file_handler.suffix = "%Y-%m-%d"
file_handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(name)s:%(lineno)d: %(message)s", datefmt="%H:%M:%S"))

logging.basicConfig(
    level=logging.INFO,
    handlers=[
        console_handler,
        file_handler
    ]
)

# Silence watchfiles info logs (which spam the console due to OrbbecSDK.log.txt changing)
logging.getLogger("watchfiles.main").setLevel(logging.WARNING)

# 页面状态类接口是亚秒级只读轮询 (角点检测 800ms、臂状态 500ms/3s), 全量记进 access 日志
# 会把真正的动作与报警冲走。这里只静音“这些路径上的 2xx GET”: 任何 POST (运动/开关喷)
# 与任何非 2xx (detail 里带英文报错) 一律保留, 出事时日志仍然是完整的。
_POLLING_GET_PATHS = (
    "/api/robot/state",         # 臂状态 (主推通道是 /api/robot/ws 广播, 部分页仍在轮询)
    "/api/camera/corners",      # 标定板角点检测结果
    "/api/camera/stream_info",  # 推流信息
)


class PollingAccessFilter(logging.Filter):
    """按路径过滤高频只读轮询的访问日志行 (不靠正则套整个文本, 免得过匹配)。"""

    def filter(self, record: logging.LogRecord) -> bool:
        if record.name != "uvicorn.access":
            return True
        args = record.args
        # uvicorn 的 args = (client_addr, method, full_path, http_version, status_code);
        # 形状一旦对不上就照常放行 —— 宁可日志吵, 也不能漏掉异常。
        if not isinstance(args, tuple) or len(args) < 5:
            return True
        method, path, status = str(args[1]), str(args[2]), str(args[4])
        if method != "GET" or not status.startswith("2"):
            return True
        return not path.split("?")[0].startswith(_POLLING_GET_PATHS)


_polling_filter = PollingAccessFilter()
console_handler.addFilter(_polling_filter)
file_handler.addFilter(_polling_filter)

from services.log_service import ws_log_handler, log_service
logging.getLogger().addHandler(ws_log_handler)

from db.database import engine, Base
from apps.camera.api import camera_router
from apps.calib.api import calib_router
from apps.robot.api import robot_router
from apps.follow.api import follow_router
from apps.system.api import sys_router
from apps.interactive.api import router as interactive_router
from contextlib import asynccontextmanager
from apps.camera.services.camera_service import camera_service
from apps.robot.services.robot_service import robot_service

# Create DB tables
Base.metadata.create_all(bind=engine)

logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize logging broadcast
    log_service.initialize()
    
    # Startup: Start hardware services & AI models
    logger.info("Starting background services (Camera, Robot, MobileSAM)...")
    camera_service.start_stream(camera_type="orbbec")
    
    from apps.interactive.sam_service import sam_service
    sam_service.initialize()
    
    from apps.interactive.reconstruction_service import reconstruction_service
    reconstruction_service.warmup()
    
    yield
    # Shutdown: Clean up hardware resources
    logger.info("Shutting down background services...")
    reconstruction_service.shutdown()
    camera_service.stop_stream()
    robot_service.disconnect()

app = FastAPI(title="AiSprayer System API", version="1.0.0", lifespan=lifespan)

# Allow CORS for local frontend development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], # In production, restrict this
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(camera_router)
app.include_router(calib_router)
app.include_router(robot_router)
app.include_router(follow_router)
app.include_router(sys_router)
app.include_router(interactive_router)

# Mount static files for frontend 3D rendering and images
app.mount("/urdf", StaticFiles(directory=os.path.join(PROJECT_ROOT, "app/urdf")), name="urdf")

template_group_path = os.path.join(PROJECT_ROOT, "data/template_group")
os.makedirs(template_group_path, exist_ok=True)
app.mount("/templates", StaticFiles(directory=template_group_path), name="templates")

@app.get("/")
def read_root():
    return {"message": "AiSprayer Backend is running!"}

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=False, log_config=None)
