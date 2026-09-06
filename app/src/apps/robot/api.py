# -*- coding: utf-8 -*-
import asyncio
import json
import logging
import os
import sys
from typing import Optional

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from apps.robot.models import (
    ConnectRobotReq, GlobalSpeedReq, HomeReq, JogContinuousReq, JogReq,
    SetDoReq, SpeedReq,
)
from apps.robot.services.robot_service import robot_service
from services.setting_service import SettingService

logger = logging.getLogger(__name__)

robot_router = APIRouter(prefix="/api/robot", tags=["Robot"])


@robot_router.post("/connect")
def connect_robot(req: ConnectRobotReq):
    settings = SettingService()
    ip = settings.get_value("robot_ip", "192.168.5.1")
    port = str(settings.get_value("robot_port", "29999"))

    success, msg = robot_service.connect(req.robot_type, ip, port)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "connected"}


@robot_router.post("/disconnect")
def disconnect_robot():
    success, msg = robot_service.disconnect()
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "disconnected"}


@robot_router.post("/jog")
def jog_robot(req: JogReq):
    is_xyz = req.axis in ["X", "Y", "Z", "Rx", "Ry", "Rz"]
    speed = req.speed_l if is_xyz else req.speed_j
    acc = req.acc_l if is_xyz else req.acc_j
    success, msg = robot_service.jog_step(req.axis, req.direction, req.step, speed=speed, acc=acc)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}


@robot_router.post("/jog_continuous")
def jog_continuous_robot(req: JogContinuousReq):
    success, msg = robot_service.jog_continuous(req.axis, req.direction)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}


@robot_router.post("/zero")
def robot_zero(req: HomeReq):
    success, msg = robot_service.go_zero(speed=req.speed, acc=req.acc)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}


@robot_router.post("/fold")
def robot_fold(req: HomeReq):
    success, msg = robot_service.go_fold(speed=req.speed, acc=req.acc)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}


@robot_router.post("/home")
def robot_home(req: HomeReq):
    success, msg = robot_service.go_home(speed=req.speed, acc=req.acc)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}


@robot_router.get("/speed")
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
        "tcp_speed_actual": diag.get("tcp_speed_actual", [0.0] * 6),
        "qd_actual": diag.get("qd_actual", [0.0] * 6),
        "load": diag.get("load", 0.0),
        "error_status": diag.get("error_status", 0),
    }


@robot_router.post("/speed")
def set_robot_speed(req: SpeedReq):
    success, msg = robot_service.set_speed(req.speed_l, req.acc_l, req.speed_j, req.acc_j)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}


@robot_router.post("/global_speed")
def set_global_speed_endpoint(req: GlobalSpeedReq):
    success, msg = robot_service.set_global_speed_factor(req.factor)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}


@robot_router.post("/pause")
def pause_robot():
    success, err = robot_service.pause()
    if not success:
        raise HTTPException(status_code=400, detail=err)
    return {"status": "success"}


@robot_router.post("/resume")
def resume_robot():
    success, err = robot_service.resume()
    if not success:
        raise HTTPException(status_code=400, detail=err)
    return {"status": "success"}


@robot_router.post("/estop")
def estop_robot():
    success, err = robot_service.estop()
    if not success:
        raise HTTPException(status_code=400, detail=err)
    return {"status": "success"}


@robot_router.post("/clear_error")
def robot_clear_error():
    success, err = robot_service.clear_error()
    if not success:
        raise HTTPException(status_code=400, detail=err)
    return {"status": "success"}


@robot_router.post("/set_do")
def robot_set_do(req: SetDoReq):
    eff_index = req.index if req.index is not None else robot_service.do_index
    success, msg = robot_service.set_do(eff_index, req.status)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok", "index": eff_index, "do_status": req.status}




@robot_router.websocket("/ws")
async def robot_ws(websocket: WebSocket):
    await websocket.accept()
    loop = asyncio.get_running_loop()

    def on_robot_state(data: dict):
        try:
            asyncio.run_coroutine_threadsafe(
                websocket.send_text(json.dumps(data)),
                loop
            )
        except Exception:
            pass

    robot_service.register_ws_callback(on_robot_state)

    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        robot_service.unregister_ws_callback(on_robot_state)
