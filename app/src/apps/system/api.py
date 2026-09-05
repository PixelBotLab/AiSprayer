from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from typing import Dict, Any
from services.setting_service import SettingService
from services.log_service import log_service

sys_router = APIRouter(prefix="/api/system", tags=["System"])

class SettingsUpdate(BaseModel):
    settings: Dict[str, Any]

@sys_router.get("/config")
def get_config():
    settings = SettingService().get_all_settings()
    # Provide defaults if missing
    if "calib_board_cols" not in settings:
        settings["calib_board_cols"] = 9
    if "calib_board_rows" not in settings:
        settings["calib_board_rows"] = 12
    if "robot_ip" not in settings:
        settings["robot_ip"] = "192.168.5.1"
    if "robot_port" not in settings:
        settings["robot_port"] = 29999
    return {"config": settings}

@sys_router.post("/config")
def update_config(req: SettingsUpdate):
    srv = SettingService()
    for k, v in req.settings.items():
        srv.set_value(k, v)
    return {"status": "ok"}

@sys_router.websocket("/logs/ws")
async def websocket_logs(websocket: WebSocket):
    await log_service.connect(websocket)
    try:
        while True:
            # Keep the connection open and wait for client disconnect
            await websocket.receive_text()
    except WebSocketDisconnect:
        log_service.disconnect(websocket)

class CalibrationModeUpdate(BaseModel):
    enabled: bool

@sys_router.post("/camera/calibration_mode")
def set_calibration_mode(req: CalibrationModeUpdate):
    from apps.camera.services.camera_service import camera_service
    camera_service.set_calibration_mode(req.enabled)
    return {"status": "ok", "enabled": req.enabled}

@sys_router.get("/camera/status")
def get_camera_status():
    from apps.camera.services.camera_service import camera_service
    return camera_service.get_status()

@sys_router.websocket("/camera/ws")
async def websocket_camera_status(websocket: WebSocket):
    await websocket.accept()
    from apps.camera.services.camera_service import camera_service
    import asyncio
    
    # 1. Push immediate status on connect
    await websocket.send_json(camera_service.get_status())
    
    loop = asyncio.get_running_loop()
    
    def on_status_change(status: dict):
        if not loop.is_closed():
            loop.call_soon_threadsafe(
                lambda: asyncio.create_task(websocket.send_json(status))
            )
            
    camera_service.register_status_callback(on_status_change)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        camera_service.unregister_status_callback(on_status_change)


