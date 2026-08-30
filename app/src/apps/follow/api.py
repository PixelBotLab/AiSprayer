# -*- coding: utf-8 -*-
"""
Follow 的 HTTP/WS 入口：把相机服务里的 follow 暴露给页面（启动 / 调零 / 停止 + 实时快照）。

路由很薄，两件事例外，都值得写下来：

1. **错误码分两类**。`503` = 相机服务没起来（`follow` 是进程内跑的，后端不在就没有一切），
   `400` = 请求本身不成立（`follow.arm.mode: real`、标定缺失、调零却没有当前位姿）。
   页面对前者的正确反应是"去起后端"，对后者是"改配置/换个操作" —— 混成一个 400，
   用户就会去点配置，而真正该做的事是启动 `orbbec_camera_service`。

2. **三个按钮都是同步的**。C++ 侧档位切换（重启取流 pipeline）是提交式的：follow_service 先一次短请求提交，
   再轮询 `/follow/status` 的 `switching` 字段等终态（总预算 `follow_service.SWITCH_DEADLINE`，60 s）。
   页面要的是"点下去之后要么成了、要么告诉我为什么没成"，不是一个乐观的 200 —— 所以这里直接等 `_begin` 返回。
"""
from __future__ import annotations

import asyncio
import json
import logging
from typing import List, Optional

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from apps.follow.services.follow_service import follow_service

logger = logging.getLogger(__name__)

follow_router = APIRouter(prefix="/api/follow", tags=["Follow"])


class FollowJointsReq(BaseModel):
    """
    `joints_deg`：此刻仿真臂的 URDF 关节角（度）。页面是唯一知道 URDF viewer 里臂在哪儿的人，
    所以由它带上来；没带就用服务自己的上一步目标，启动再兜底到 home（见 follow_service 的优先级注释）。
    """
    joints_deg: Optional[List[float]] = None


def _fail(msg: str) -> None:
    """
    后端没起 ⇒ 503；其余 ⇒ 400。

    分类看的是 `last_failure_upstream`（**这次**失败够不够得着相机服务），不是可达性快照：
    模式/标定/基线这三类护栏压根不出网，用后者判断会被上一次的残留状态带偏，把"配置不成立"
    报成"后端没起来"，用户就会去重启服务而不是改配置。
    """
    if follow_service.last_failure_upstream:
        raise HTTPException(status_code=503, detail=msg)
    raise HTTPException(status_code=400, detail=msg)


@follow_router.get("/status")
def get_status():
    """一次性快照：臂侧状态 + follow 快照 + 轴映射来源（用哪个必须是可见的）。"""
    return follow_service.status()


@follow_router.post("/start")
def start_follow(req: Optional[FollowJointsReq] = None):
    """① 启动：使能 follow（相机切 640x480 + 硬件 D2C）→ 示教 → 臂基线 = home（或传入位姿）。"""
    ok, msg = follow_service.start(req.joints_deg if req else None)
    if not ok:
        _fail(msg)
    return {"status": "started", "message": msg, "data": follow_service.status()}


@follow_router.post("/zero")
def zero_follow(req: Optional[FollowJointsReq] = None):
    """② 调零：重新示教并把臂基线换成当前位姿 —— 增量归零，臂停在它此刻该在的地方。"""
    ok, msg = follow_service.zero(req.joints_deg if req else None)
    if not ok:
        _fail(msg)
    return {"status": "zeroed", "message": msg, "data": follow_service.status()}


@follow_router.post("/stop")
def stop_follow():
    """③ 停止：关使能（取流退回 hardware.camera）。广播 active=false，页面据此交回控制权。"""
    ok, msg = follow_service.stop()
    if not ok:
        _fail(msg)
    return {"status": "stopped", "message": msg, "data": follow_service.status()}


@follow_router.websocket("/ws")
async def follow_ws(websocket: WebSocket):
    """
    实时通道：`{"type":"follow_state","data":{...follow 快照..., joints_deg, target_pose}}`。
    格式与 `/api/calib/robot/ws` 一致（同一个前端复用同一套 dispatch），去重在服务里做。
    """
    await websocket.accept()
    loop = asyncio.get_running_loop()

    def on_follow_state(data: dict):
        try:
            asyncio.run_coroutine_threadsafe(
                websocket.send_text(json.dumps(data, ensure_ascii=False)), loop)
        except Exception:
            pass       # 客户端正在断开：广播路径不值得为它记一条错误

    follow_service.register_ws_callback(on_follow_state)
    try:
        # 连上就先推一帧：页面要能立刻分辨"后端在跑但没启用"和"我根本没连上"。
        snapshot = await asyncio.to_thread(follow_service.status)
        await websocket.send_text(json.dumps(
            {"type": "follow_state", "data": snapshot}, ensure_ascii=False))
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        follow_service.unregister_ws_callback(on_follow_state)
