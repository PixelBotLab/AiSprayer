from fastapi import APIRouter
from pydantic import BaseModel
from typing import Dict, Any
from services.setting_service import SettingService

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
