from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import List, Optional
import json

import sys
import os
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from services.camera_service import camera_service
from services.robot_service import robot_service
from apps.calib.services.calibration_service import calibration_service

calib_router = APIRouter(prefix="/api/calib", tags=["Calibration"])

class ConnectRobotReq(BaseModel):
    robot_type: str = "dobot"

class JogReq(BaseModel):
    axis: str
    direction: int
    step: float = 1.0
    speed_l: float = 20.0
    acc_l: float = 20.0
    speed_j: float = 20.0
    acc_j: float = 20.0

class HomeReq(BaseModel):
    speed: float = 20.0
    acc: float = 20.0

class SpeedReq(BaseModel):
    speed_l: float
    acc_l: float
    speed_j: float
    acc_j: float

@calib_router.get("/camera/stream")
def camera_stream():
    if not camera_service.is_streaming():
        if not camera_service.start_stream("orbbec"):
            raise HTTPException(status_code=500, detail="Could not open camera")
    return StreamingResponse(camera_service.generate_mjpeg_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

@calib_router.post("/robot/connect")
def connect_robot(req: ConnectRobotReq):
    from services.setting_service import SettingService
    settings = SettingService()
    ip = settings.get_value("robot_ip", "192.168.5.1")
    port = str(settings.get_value("robot_port", "29999"))
    
    success, msg = robot_service.connect(req.robot_type, ip, port)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "connected"}

@calib_router.post("/robot/disconnect")
def disconnect_robot():
    success, msg = robot_service.disconnect()
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "disconnected"}


@calib_router.post("/robot/jog")
def jog_robot(req: JogReq):
    is_xyz = req.axis in ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
    speed = req.speed_l if is_xyz else req.speed_j
    acc = req.acc_l if is_xyz else req.acc_j
    success, msg = robot_service.jog_step(req.axis, req.direction, req.step, speed=speed, acc=acc)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}

@calib_router.post("/robot/zero")
def robot_zero(req: HomeReq):
    success, msg = robot_service.go_zero(speed=req.speed, acc=req.acc)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}

@calib_router.post("/robot/home")
def robot_home(req: HomeReq):
    success, msg = robot_service.go_home(speed=req.speed, acc=req.acc)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}

@calib_router.get("/robot/speed")
def get_robot_speed():
    speed_l, acc_l, speed_j, acc_j = robot_service.get_speed()
    return {"speed_l": speed_l, "acc_l": acc_l, "speed_j": speed_j, "acc_j": acc_j}

@calib_router.post("/robot/speed")
def set_robot_speed(req: SpeedReq):
    success, msg = robot_service.set_speed(req.speed_l, req.acc_l, req.speed_j, req.acc_j)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}

@calib_router.post("/robot/pause")
def pause_robot():
    success, err = robot_service.pause()
    if not success:
        raise HTTPException(status_code=400, detail=err)
    return {"status": "success"}

@calib_router.post("/robot/resume")
def resume_robot():
    success, err = robot_service.resume()
    if not success:
        raise HTTPException(status_code=400, detail=err)
    return {"status": "success"}

@calib_router.post("/robot/estop")
def estop_robot():
    success, err = robot_service.estop()
    if not success:
        raise HTTPException(status_code=400, detail=err)
    return {"status": "success"}

@calib_router.websocket("/robot/ws")
async def robot_ws(websocket: WebSocket):
    await websocket.accept()
    import asyncio
    loop = asyncio.get_running_loop()
    
    def on_robot_state(data: dict):
        try:
            # Use asyncio to send the data as JSON
            asyncio.run_coroutine_threadsafe(
                websocket.send_text(json.dumps(data)), 
                loop
            )
        except Exception:
            pass

    robot_service.register_ws_callback(on_robot_state)
    
    try:
        while True:
            # Keep connection alive, wait for client to disconnect
            await websocket.receive_text()
    except WebSocketDisconnect:
        robot_service.unregister_ws_callback(on_robot_state)


@calib_router.post("/samples/add")
def add_sample():
    frame = camera_service.get_latest_frame()
    pose, err = robot_service.get_current_pose()
    
    if frame is None or pose is None:
        raise HTTPException(status_code=400, detail=f"Frame or pose missing: {err}")
        
    count = calibration_service.add_sample(frame, pose)
    return {"samples_count": count}

@calib_router.get("/samples")
def list_samples():
    return {"samples": calibration_service.get_samples_info()}

@calib_router.post("/run")
def run_calib():
    res = calibration_service.run_calibration()
    if not res.get("success"):
        raise HTTPException(status_code=400, detail=res.get("error", "Calibration failed"))
    return res
