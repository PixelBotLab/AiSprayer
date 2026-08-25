from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException, Request
from fastapi.responses import StreamingResponse
import asyncio
import logging
import requests
from core.config import sprayer_config

from apps.camera.services.camera_service import camera_service
from apps.camera.models import CalibrationModeUpdate

logger = logging.getLogger(__name__)

camera_router = APIRouter(prefix="/api/camera", tags=["Camera"])


@camera_router.get("/stream_info")
def get_stream_info(request: Request):
    """获取所有可用流媒体地址 (HTTP-FLV, RTSP)"""
    host_hdr = request.headers.get("host", "127.0.0.1:8000")
    host_ip = host_hdr.split(":")[0]
    return camera_service.get_stream_info(host_ip)


@camera_router.get("/flv")
def camera_flv_stream():
    """
    100% RK3588 MPP 硬件编码 H.264 HTTP-FLV 实时流代理。
    直通 ZLMediaKit，零 CPU 开销，超低延迟 (~100ms)，前端通过 MSE (mpegts.js) 原生硬解播放。
    """
    if not camera_service.is_streaming():
        if not camera_service.start_stream("orbbec"):
            raise HTTPException(status_code=500, detail="Could not open camera stream")
    
    zlm_port = sprayer_config.config_data.get("hardware", {}).get("camera", {}).get("server", {}).get("zlm_http_port", 8008)
    app_name = sprayer_config.config_data.get("hardware", {}).get("camera", {}).get("streaming", {}).get("app", "live")
    stream_id = sprayer_config.config_data.get("hardware", {}).get("camera", {}).get("streaming", {}).get("stream_id", "orbbec_color")
    flv_url = f"http://127.0.0.1:{zlm_port}/{app_name}/{stream_id}.live.flv"
    
    def iter_flv():
        try:
            with requests.get(flv_url, stream=True, timeout=(3.0, None)) as r:
                if r.status_code == 200:
                    for chunk in r.iter_content(chunk_size=8192):
                        if not camera_service._is_streaming or chunk is None:
                            break
                        yield chunk
                else:
                    logger.warning(f"ZLMediaKit FLV stream returned HTTP {r.status_code}")
        except GeneratorExit:
            pass
        except Exception as e:
            logger.warning(f"FLV stream disconnected: {e}")
            
    return StreamingResponse(iter_flv(), media_type="video/x-flv")


@camera_router.get("/status")
def get_camera_status():
    """查询相机当前状态 (在线、推流、帧率、标定模式)"""
    return camera_service.get_status()


@camera_router.post("/calibration_mode")
def set_calibration_mode(req: CalibrationModeUpdate):
    """开启/关闭相机标定模式 (标定模式下关闭深度流以降低CPU负载并开启角点检测)"""
    camera_service.set_calibration_mode(req.enabled)
    return {
        "status": "ok",
        "calibration_mode": req.enabled,
        "msg": "Calibration mode updated"
    }


@camera_router.get("/intrinsics")
def get_camera_intrinsics():
    """获取相机内参与畸变参数"""
    K, D = camera_service.get_intrinsics()
    if K is None:
        raise HTTPException(status_code=404, detail="Camera intrinsics not available (camera offline)")
    return {
        "status": "ok",
        "intrinsic_matrix": K.tolist() if hasattr(K, "tolist") else K,
        "distortion_coeffs": D.tolist() if hasattr(D, "tolist") else D
    }


@camera_router.get("/corners")
def get_detected_corners():
    """获取当前最新检测到的标定板角点"""
    corners = camera_service.get_latest_corners()
    if corners is None:
        return {"status": "ok", "found": False, "corners": []}
    return {
        "status": "ok",
        "found": True,
        "count": len(corners),
        "corners": corners.reshape(-1, 2).tolist() if hasattr(corners, "reshape") else corners
    }


@camera_router.post("/start")
def start_camera(camera_type: str = "orbbec"):
    """启动相机服务"""
    ok = camera_service.start_stream(camera_type)
    return {"status": "ok" if ok else "error", "streaming": ok}


@camera_router.post("/stop")
def stop_camera():
    """停止相机服务"""
    camera_service.stop_stream()
    return {"status": "ok", "streaming": False}


@camera_router.post("/save_frame")
def save_camera_frame(req: dict):
    """触发 C++ 异步无锁保存当前高清彩色图与深度图到指定目录"""
    save_dir = req.get("save_dir", "data/calib")
    prefix = req.get("prefix", "sample")
    metadata = req.get("metadata", {})
    res = camera_service.save_frame(save_dir=save_dir, prefix=prefix, metadata=metadata)
    if res is None:
        raise HTTPException(status_code=500, detail="Failed to save frame")
    return {"status": "ok", "data": res}


@camera_router.websocket("/ws")
async def websocket_camera_status(websocket: WebSocket):
    """WebSocket 实时相机状态广播"""
    await websocket.accept()
    
    # 1. 发送连接时的即时状态
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
