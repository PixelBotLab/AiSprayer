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
    AimAtPixelReq, ConnectRobotReq, GlobalSpeedReq, GripperActionReq, GripperMoveReq, HomeReq, JogContinuousReq, JogReq,
    MarkCrossReq, SetDoReq, SpeedReq, UiLogReq,
)
from apps.robot.services.live_aim_service import LiveAimError, live_aim_service
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


@robot_router.post("/aim_at_pixel")
def aim_at_pixel(req: AimAtPixelReq):
    """
    Aim the tool axis (a virtual laser beam) onto the sight line of a clicked pixel (single-pose MovJ, no capture / no 3D rebuild).

    A single pixel back-projects to a ray, and the clicked point P is taken on it at the **measured** depth of that
    pixel (`depth_source: measured`), or at `distance_mm` (default spraying.aim_distance_mm, `depth_source: config`)
    when the depth reading is unusable. Two candidate families both put the beam through P, and which one is tried
    first follows from how P was obtained:
    - `swing` (preferred when the depth was measured): the nozzle stays exactly where it is and only the wrist turns,
      so aiming is a pure orientation problem — no reach requirement at all, like a pan-tilt laser. Reported with
      `depth_locked: true` because the hit is locked to that one depth (a measured depth makes this ~1 mm; a guessed
      one would make it tens of centimetres, which is why it is never preferred then).
    - `on-ray` (preferred when the depth is unavailable): the nozzle is also placed on the ray, so the beam is
      **collinear** with the line of sight and hits it at any depth; the nozzle is kept at least the spray standoff in
      front of the lens and never past the clicked point. This one does depend on reach, because positioning the
      nozzle next to the workpiece is a spraying-process requirement, not an aiming one.
    Both families stay in the same batch, so a wrist limit or a near-singular pose degrades to the other family
    instead of failing the request.
    Spraying is always turned off (immediate DO) before the move; the DO is
    switched back on after a successful arrival only when spraying.aim_do_on_arrive is enabled (laser-pointer rig).
    Eye-in-hand aiming additionally requires the clicked image to be paired with the current arm pose: the
    request is refused while the arm has been still for less than spraying.aim_frame_settle_s, or while the
    video stream is stalled/restarting, because the sight ray is composed from the pose of the frame that was
    clicked (a stale image would aim at a direction the camera never saw).
    `beam_angle_deg` is the tracking error against the planned beam direction (not against the sight line: a swing
    solution deliberately aims off the sight line). The true landing-point miss is verified separately by the
    operator marking the laser cross centre by eye (see `/aim_mark_cross`), which is more reliable than the
    previous automatic blob detector.
    """
    try:
        return live_aim_service.aim_at_pixel(
            req.u_px, req.v_px,
            distance_mm=req.distance_mm, speed=req.speed, acc=req.acc)
    except LiveAimError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.exception(f"aim_at_pixel failed unexpectedly: {e}")
        raise HTTPException(status_code=500, detail=f"Live aiming failed: {e}")


@robot_router.post("/aim_mark_cross")
def aim_mark_cross(req: MarkCrossReq):
    """
    Mark the centre of the laser cross by eye (bypassing the fragile blob detector) to measure the true miss.

    The camera is hand-mounted (eye-in-hand), so the frame you clicked for the target and the frame you are
    marking the cross in were taken at two different flange poses. Each pixel is back-projected through its own
    capture-time camera pose (composed from the current joints via the eye-in-hand extrinsic) into one common
    base frame, so the two are directly comparable. Reports, in the base frame: the marked cross centre as a
    3D point (pixel + that pixel's measured depth), the spatial miss versus the aimed target point split into
    lateral and depth components, the angle between the two sight lines, and the perpendicular gap of the real
    hit off the modelled beam line (through the arrived TCP along tool +Z) — a non-zero gap means the physical
    laser axis is not the tool +Z axis the planner assumes. Requires the robot idle with the live image settled,
    and an aim to have run first. The result is also mirrored to the dedicated verification log.
    """
    try:
        return live_aim_service.mark_cross_center(req.u_px, req.v_px)
    except LiveAimError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.exception(f"aim_mark_cross failed unexpectedly: {e}")
        raise HTTPException(status_code=500, detail=f"Cross-centre verification failed: {e}")


@robot_router.post("/aim_ui_log")
def aim_ui_log(req: UiLogReq):
    """
    Mirror a live-aim UI notice into the dedicated verification log.

    So the operator never has to copy-paste what the interface shows (e.g. "cross center at pixel (...), X deg
    off the clicked sight line", "skipped (N red blobs ...)"). The message is stored verbatim; it is already
    English UI text. This is a side-effect-free sink: it always returns ok so a lost notice never blocks the UI.
    """
    live_aim_service.log_ui_notice(req.level, req.message)
    return {"status": "ok"}


@robot_router.get("/state")
def get_robot_state():
    """Query current robot connection status, running status, live pose, and joint angles"""
    is_conn = robot_service.is_connected()
    joints, _ = robot_service.get_current_joint() if is_conn else (None, "")
    pose, _ = robot_service.get_current_pose() if is_conn else (None, "")
    return {
        "connected": is_conn,
        "is_moving": robot_service.is_moving() if is_conn else False,
        "running_status": robot_service.get_running_state() if is_conn else 0,
        "joints": [round(float(j), 2) for j in joints] if joints else None,
        "pose": [round(float(p), 2) for p in pose] if pose else None,
    }


@robot_router.get("/joints")
def get_robot_joints():
    """Query current robot joint positions in degrees (requires connected robot)"""
    if not robot_service.is_connected():
        raise HTTPException(status_code=400, detail="Robot is not connected")
    joints, err = robot_service.get_current_joint()
    if joints is None:
        raise HTTPException(status_code=500, detail=f"Failed to read robot joints: {err}")
    return {"status": "ok", "joints": [round(float(j), 2) for j in joints]}


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
    eff_index = req.index if req.index is not None else robot_service.spray_do_index
    success, msg = robot_service.set_do(eff_index, req.status, immediate=req.immediate)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok", "index": eff_index, "do_status": req.status, "immediate": req.immediate}


# ---------------------------------------------------------------------------
# Junduo Gripper Endpoints (0.0mm Closed ~ 50.0mm Fully Open)
# ---------------------------------------------------------------------------

@robot_router.post("/gripper/move")
def move_gripper(req: GripperMoveReq):
    """Set gripper stroke position (0.0mm Closed ~ 50.0mm Fully Open)"""
    success, msg = robot_service.move_gripper(
        stroke_mm=req.stroke_mm,
        force_percent=req.force_percent,
        speed=req.speed,
        wait_complete=req.wait_complete
    )
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok", "stroke_mm": req.stroke_mm}


@robot_router.post("/gripper/open")
def open_gripper(req: Optional[GripperActionReq] = None):
    """Fully open gripper with optional custom force/speed"""
    fp = req.force_percent if req else None
    sp = req.speed if req else None
    success, msg = robot_service.open_gripper(force_percent=fp, speed=sp)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}


@robot_router.post("/gripper/clamp")
def clamp_gripper(req: Optional[GripperActionReq] = None):
    """Close/clamp gripper with optional custom force/speed"""
    fp = req.force_percent if req else None
    sp = req.speed if req else None
    success, msg = robot_service.clamp_gripper(force_percent=fp, speed=sp)
    if not success:
        raise HTTPException(status_code=400, detail=msg)
    return {"status": "ok"}


@robot_router.get("/gripper/state")
def get_gripper_state():
    """Query current gripper telemetry and connection status"""
    return robot_service.get_gripper_state()


@robot_router.get("/gripper/specs")
def get_gripper_specs():
    """Query gripper hardware specifications (single source of truth from JunduoGripper)"""
    return robot_service.get_gripper_specs()


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
