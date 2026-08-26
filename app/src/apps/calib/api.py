from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect, BackgroundTasks
from fastapi.responses import StreamingResponse, FileResponse
from pydantic import BaseModel
from typing import List, Optional
import json

import sys
import os
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from apps.camera.services.camera_service import camera_service
from services.robot_service import robot_service
from apps.calib.services.calibration_service import calibration_service

calib_router = APIRouter(prefix="/api/calib", tags=["Calibration"])

class ConnectRobotReq(BaseModel):
    robot_type: str = "dobot"

class JogReq(BaseModel):
    axis: str
    direction: int
    step: float = 1.0
    speed_l: float = 10.0
    acc_l: float = 10.0
    speed_j: float = 10.0
    acc_j: float = 10.0

class HomeReq(BaseModel):
    speed: float = 10.0
    acc: float = 10.0

class SpeedReq(BaseModel):
    speed_l: float
    acc_l: float
    speed_j: float
    acc_j: float

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

class JogContinuousReq(BaseModel):
    axis: str
    direction: int

@calib_router.post("/robot/jog_continuous")
def jog_continuous_robot(req: JogContinuousReq):
    success, msg = robot_service.jog_continuous(req.axis, req.direction)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}

@calib_router.post("/robot/zero")
def robot_zero(req: HomeReq):
    success, msg = robot_service.go_zero(speed=req.speed, acc=req.acc)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}

@calib_router.post("/robot/fold")
def robot_fold(req: HomeReq):
    success, msg = robot_service.go_fold(speed=req.speed, acc=req.acc)
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
    diag = robot_service.get_feedback_diagnostics()
    return {
        "speed_l": speed_l,
        "acc_l": acc_l,
        "speed_j": speed_j,
        "acc_j": acc_j,
        "global_speed_factor": robot_service.global_speed_factor,
        "max_tcp_speed_mm_s": robot_service.max_tcp_speed_mm_s,
        "max_joint_speed_deg_s": robot_service.max_joint_speed_deg_s,
        "tcp_speed_actual": diag.get("tcp_speed_actual", [0.0]*6),
        "qd_actual": diag.get("qd_actual", [0.0]*6),
        "load": diag.get("load", 0.0),
        "error_status": diag.get("error_status", 0),
    }

@calib_router.post("/robot/speed")
def set_robot_speed(req: SpeedReq):
    success, msg = robot_service.set_speed(req.speed_l, req.acc_l, req.speed_j, req.acc_j)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}

class GlobalSpeedReq(BaseModel):
    factor: int

@calib_router.post("/robot/global_speed")
def set_global_speed_endpoint(req: GlobalSpeedReq):
    success, msg = robot_service.set_global_speed_factor(req.factor)
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

@calib_router.post("/robot/clear_error")
def robot_clear_error():
    success, err = robot_service.clear_error()
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


@calib_router.get("/sessions")
def list_sessions():
    return {"sessions": calibration_service.get_sessions()}

@calib_router.post("/sessions/new")
def create_session():
    session_id = calibration_service.create_session()
    return {"session_id": session_id}

@calib_router.delete("/sessions/{session_id}")
def delete_session(session_id: str):
    success = calibration_service.delete_session(session_id)
    if not success:
        raise HTTPException(status_code=404, detail="Session not found")
    return {"status": "deleted"}

@calib_router.get("/sessions/{session_id}")
def get_session(session_id: str):
    data = calibration_service.get_session_data(session_id)
    return data

@calib_router.post("/sessions/{session_id}/samples")
def add_sample(session_id: str):
    pose, err = robot_service.get_current_pose()
    if pose is None:
        raise HTTPException(status_code=400, detail=f"Robot pose missing: {err}")
        
    try:
        count = calibration_service.add_sample(session_id, pose)
        return {"samples_count": count}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@calib_router.get("/sessions/{session_id}/images/{filename}")
def get_image(session_id: str, filename: str):
    import os
    img_path = os.path.join(calibration_service.calib_dir, session_id, filename)
    
    if not os.path.exists(img_path):
        raise HTTPException(status_code=404, detail="Image not found")
        
    ext = os.path.splitext(filename)[1].lower()
    media_type = "image/png" if ext == ".png" else "image/jpeg"
    return FileResponse(img_path, media_type=media_type)

@calib_router.get("/sessions/{session_id}/images_with_corners/{filename}")
def get_image_with_corners_route(session_id: str, filename: str):
    from fastapi.responses import Response
    try:
        img_bytes = calibration_service.get_image_with_corners(session_id, filename)
        return Response(content=img_bytes, media_type="image/jpeg")
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Image not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@calib_router.post("/sessions/{session_id}/run")
def run_calib(session_id: str, background_tasks: BackgroundTasks):
    background_tasks.add_task(calibration_service.run_calibration, session_id)
    return {"success": True, "message": "Calibration task started"}

@calib_router.post("/sessions/{session_id}/resample_and_calibrate")
def resample_and_calib_route(session_id: str, background_tasks: BackgroundTasks):
    background_tasks.add_task(calibration_service.resample_and_calibrate, session_id)
    return {"success": True, "message": "Automatic resample and calibration task started"}

@calib_router.get("/sessions/{session_id}/progress")
def get_calib_progress(session_id: str):
    return StreamingResponse(
        calibration_service.stream_progress(session_id),
        media_type="text/event-stream"
    )
