from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import sys
import os
import logging
import logging.handlers
import uvicorn
from fastapi.staticfiles import StaticFiles

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
console_handler.setFormatter(ColorFormatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s", datefmt="%H:%M:%S"))

# File handler without colors (plain text)
file_handler = logging.handlers.TimedRotatingFileHandler(
    os.path.join(log_dir, "backend.log"),
    when="midnight",
    interval=1,
    backupCount=30,  # Keep 30 days of logs
    encoding="utf-8"
)
file_handler.suffix = "%Y-%m-%d"
file_handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s", datefmt="%H:%M:%S"))

logging.basicConfig(
    level=logging.INFO,
    handlers=[
        console_handler,
        file_handler
    ]
)

# Silence watchfiles info logs (which spam the console due to OrbbecSDK.log.txt changing)
logging.getLogger("watchfiles.main").setLevel(logging.WARNING)

from services.log_service import ws_log_handler, log_service
logging.getLogger().addHandler(ws_log_handler)

from db.database import engine, Base
from apps.calib.api import calib_router
from apps.system.api import sys_router
from contextlib import asynccontextmanager
from services.camera_service import camera_service
from services.robot_service import robot_service

# Create DB tables
Base.metadata.create_all(bind=engine)

logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize logging broadcast
    log_service.initialize()
    
    # Startup: Start hardware services
    logger.info("Starting background services (Camera, Robot)...")
    camera_service.start_stream(camera_type="orbbec")
    yield
    # Shutdown: Clean up hardware resources
    logger.info("Shutting down background services...")
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

app.include_router(calib_router)
app.include_router(sys_router)

# Mount URDF static files for frontend 3D rendering
app.mount("/urdf", StaticFiles(directory=os.path.join(PROJECT_ROOT, "app/urdf")), name="urdf")

@app.get("/")
def read_root():
    return {"message": "AiSprayer Backend is running!"}

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True, log_config=None)
