import os
import sys
from typing import Optional

from fastapi import APIRouter, BackgroundTasks, HTTPException
from fastapi.responses import FileResponse, Response, StreamingResponse
from pydantic import BaseModel

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from apps.calib.services.calibration_service import calibration_service
from core.handeye import (
    MIN_SAMPLES, MOUNTS, RECOMMENDED_SAMPLES,
)

calib_router = APIRouter(prefix="/api/calib", tags=["Calibration"])


@calib_router.get("/sessions")
def list_sessions():
    return {"sessions": calibration_service.get_sessions()}

@calib_router.get("/mounts")
def list_mounts():
    return {
        "mounts": list(MOUNTS),
        # 默认装法走运行时配置 (含 SQLite 覆盖), 与 create_session 回落用的是同一个值
        "default": calibration_service.sprayer.calib_mount,
        "min_samples": dict(MIN_SAMPLES),
        "recommended_samples": dict(RECOMMENDED_SAMPLES),
    }

@calib_router.get("/active")
def get_active_result():
    """Currently published (globally effective) calibration, for the "what will be replaced" prompt."""
    return {"active": calibration_service.get_active_result()}

@calib_router.post("/sessions/{session_id}/publish")
def publish_session_result(session_id: str):
    """
    Publish a solved session result as the globally effective hand-eye calibration.

    Only the global slot (configs/calib/calibration_result.yaml) is overwritten and the
    previous one is rolled to calibration_result.prev.yaml; session folders are untouched.
    """
    try:
        outcome = calibration_service.publish_result(session_id)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to publish calibration result: {e}")
    return {"status": "ok", **outcome}

class NewSessionReq(BaseModel):
    mount: Optional[str] = None

@calib_router.post("/sessions/new")
def create_session(req: NewSessionReq):
    try:
        session_id = calibration_service.create_session(req.mount)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    data = calibration_service.get_session_data(session_id)
    return {"session_id": session_id, "mount": data["mount"],
            "min_samples": data["min_samples"],
            "recommended_samples": data["recommended_samples"]}

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

class SetMountReq(BaseModel):
    mount: str

@calib_router.put("/sessions/{session_id}/mount")
def set_session_mount(session_id: str, req: SetMountReq):
    """
    Re-bind an existing session to another camera mounting (samples are kept, the solver
    must be re-run because the stored result was computed for the previous mounting).
    """
    try:
        outcome = calibration_service.set_session_mount(session_id, req.mount)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    return {"status": "ok", **outcome}

@calib_router.post("/sessions/{session_id}/samples")
def add_sample(session_id: str):
    try:
        count = calibration_service.capture_sample(session_id)
    except Exception as e:
        logger.error(f"Failed to capture sample for session '{session_id}': {e}", exc_info=True)
        raise HTTPException(status_code=400, detail=str(e))
    return {"samples_count": count}

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
