# -*- coding: utf-8 -*-
"""
位姿数据面的订阅端：把相机服务 SSE 推来的实时快照读进本进程。

只搬**数据面**。控制面（启动 / 调零 / 停止 / 示教）仍然是 `follow_service` 里那些短小的
HTTP 请求 —— 那些操作要的是"一个明确的成败回执"，用流来做只会把回执变成"等一条也许永远不会
来的事件"，是拿错工具。

为什么不是 WebSocket：相机服务内嵌的 httplib 0.18.3 没有服务端 WS，而这条流是单向的。
SSE 就是"一个永不结束的 GET"，`requests` 原生能读，不需要引入第二个监听端口和第二套握手。

**兜底轮询没有删掉**，这是本模块最重要的一条约定：推送链路坏掉的方式（对端半开、中间设备
静默丢连接、服务端订阅配额被占满）全都是"再也不来数据、但也不报错"，而跟随闭环最怕的正是
安静地停在旧位姿上。所以 `follow_service` 那边仍按老路走：`last_event_age` 超过阈值就自己
拉一次。两条路径解出来的东西**必须**完全一致，因此它们共用同一个入口 `FollowService._ingest()`。
"""
from __future__ import annotations

import json
import logging
import threading
import time
from typing import Any, Callable, Dict, Iterable, Iterator, Optional, Tuple

import requests

logger = logging.getLogger(__name__)

# 连接超时给得紧（本机服务，连不上就是没起）；读超时给得松于服务端心跳的若干倍，
# 让"心跳没了"先由这里判定，而不是靠 TCP 自己发现。服务端心跳 200ms。
CONNECT_TIMEOUT = 2.0
READ_TIMEOUT = 3.0

BACKOFF_BASE = 0.2       # 重连退避起点：本机服务重启一次约 1~2s，没必要从 2s 开始等
BACKOFF_CAP = 5.0        # 上限：服务真没了的时候，别把重连变成刷屏

STREAM_PATH = "/api/v1/camera/follow/stream"


def iter_events(lines: Iterable[bytes]) -> Iterator[Tuple[Optional[str], str]]:
    """
    SSE 帧解析：纯函数，不碰网络 —— 因为这一层是可以离线测的，而它错一次的后果是
    "看着连上了、数据一帧都没进"（比如把心跳注释 `: ping` 当成事件、或者多行 data 只留最后一行）。

    返回 (event_id, data)。只处理 `data:` 组成的一个事件，遇到空行提交；注释行（冒号开头，
    服务端的心跳就是这种）直接丢掉。
    """
    data_lines: list[str] = []
    event_id: Optional[str] = None

    for raw in lines:
        line = raw.decode("utf-8", errors="replace").rstrip("\r\n") if isinstance(raw, bytes) else raw
        if not line:
            if data_lines:
                yield event_id, "\n".join(data_lines)
            data_lines = []
            event_id = None
            continue
        if line.startswith(":"):
            continue                                   # 心跳/注释：只用来证明链路还活着
        field, _, value = line.partition(":")
        value = value[1:] if value.startswith(" ") else value
        if field == "data":
            data_lines.append(value)
        elif field == "id":
            event_id = value
        # 其余字段（event:/retry:）本流没用到；忽略而不是报错，服务端加字段不该弄死订阅端。
    if data_lines:                                     # 流结束时最后一段没有换行收尾
        yield event_id, "\n".join(data_lines)


class PoseStream:
    """
    一条订阅连接的宿主。`start()` 起一条守护线程读流，`stop()` 关掉响应并 join。

    回调 `on_snapshot(dict)` 在**本模块的线程**里被调用，所以它必须是线程安全的；
    `follow_service._ingest()` 本来就在自己的锁下工作，正是为此。

    一条不能写坏的约定：回调是在**本类 `_lock` 之外**调用的。订阅方的回调通常要拿它自己的锁，
    而它自己的锁又常常会在调用 `stats()` / `last_event_age()` 时被持有 —— 两边都拿锁就形成
    `stream→owner` 与 `owner→stream` 的反向嵌套。把回调放到锁外，本类的锁就永远不会跨越用户代码。
    """

    def __init__(self, base_url: str, on_snapshot: Callable[[Dict[str, Any]], None],
                 name: str = "pose-stream") -> None:
        self._url = f"{base_url}{STREAM_PATH}"
        self._on_snapshot = on_snapshot
        self._name = name
        self._thread: Optional[threading.Thread] = None
        self._resp: Optional[requests.Response] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._last_event_mono = 0.0      # 0 = 从没收到过任何事件
        self._last_id: Optional[str] = None
        self._events = 0
        self._bad = 0                    # 载荷解析失败的次数（服务端字段变了的第一现场）
        self._reconnects = 0
        self._last_error = ""

    # ------------------------------------------------------------------ 生命周期
    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            self._last_event_mono = 0.0
            self._thread = threading.Thread(target=self._run, name=self._name, daemon=True)
            self._thread.start()
        logger.info("follow 位姿推送订阅启动: %s", self._url)

    def stop(self) -> None:
        with self._lock:
            thread, resp = self._thread, self._resp
            self._thread = None
        if thread is None:
            return
        self._stop.set()
        # 先关响应再 join：线程正阻塞在 `iter_lines()` 里，只置 event 它不会醒。
        if resp is not None:
            try:
                resp.close()
            except Exception:                            # 关闭失败不影响 join
                pass
        thread.join(timeout=READ_TIMEOUT + 1.0)
        if thread.is_alive():
            logger.warning("%s 线程没能按时退出（还卡在 socket 读上）", self._name)

    @property
    def running(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    # ---------------------------------------------------------------- 可观测状态
    def last_event_age(self) -> Optional[float]:
        """距最近一次收到事件的秒数；None = 这条流从没通过数据（启动中，或从来没连上）。"""
        with self._lock:
            return None if self._last_event_mono == 0.0 else time.monotonic() - self._last_event_mono

    def stats(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "url": self._url,
                "running": self._thread is not None and self._thread.is_alive(),
                "events": self._events,
                "bad_payloads": self._bad,
                "reconnects": self._reconnects,
                "last_event_age_s": (None if self._last_event_mono == 0.0
                                     else round(time.monotonic() - self._last_event_mono, 3)),
                "last_event_id": self._last_id,
                "last_error": self._last_error,
            }

    def _note_event(self, event_id: Optional[str]) -> None:
        with self._lock:
            self._last_event_mono = time.monotonic()
            if event_id:
                self._last_id = event_id
            self._events += 1

    def _note_error(self, msg: str, reconnect: bool) -> None:
        with self._lock:
            self._last_error = msg
            if reconnect:
                self._reconnects += 1

    # ---------------------------------------------------------------------- 主循环
    def _run(self) -> None:
        attempt = 0
        while not self._stop.is_set():
            try:
                self._consume_once()
                attempt = 0                              # 走到这儿 = 连接自然结束（服务关了流）
            except Exception as e:                       # 网络/解析异常都退避重来，线程不许死
                if self._stop.is_set():
                    break
                attempt += 1
                msg = f"位姿推送中断，退避重连：{type(e).__name__}: {e}"
                self._note_error(msg, reconnect=True)
                # 第一次和每 10 次喊一遍：服务没起的时候这里是每秒级的刷屏源。
                if attempt == 1 or attempt % 10 == 0:
                    logger.warning("%s（第 %d 次重连）", msg, attempt)
            if self._stop.is_set():
                break
            self._stop.wait(_backoff_s(attempt))

    def _consume_once(self) -> None:
        headers = {"Accept": "text/event-stream"}
        if self._last_id:
            headers["Last-Event-ID"] = self._last_id     # 服务端不重放，只用来让它知道我们退了多少
        with requests.get(self._url, headers=headers, stream=True,
                          timeout=(CONNECT_TIMEOUT, READ_TIMEOUT)) as resp:
            if resp.status_code != 200:
                body = (resp.text or "")[:160]
                raise RuntimeError(f"HTTP {resp.status_code} {body}")
            self._resp = resp
            try:
                for event_id, data in iter_events(resp.iter_lines()):
                    if self._stop.is_set():
                        return
                    snap = _loads(data)
                    if snap is None:
                        with self._lock:
                            self._bad += 1
                        continue
                    self._note_event(event_id)
                    try:
                        self._on_snapshot(snap)
                    except Exception as e:               # 回调不许杀死订阅线程
                        logger.error("位姿推送回调异常（继续收）: %s", e, exc_info=True)
            finally:
                with self._lock:
                    self._resp = None

    def _consume_for_test(self, lines: Iterable[bytes]) -> int:
        """把 `iter_events` 走一遍并计入 stats，供离线测试用（不起线程、不碰 socket）。返回事件数。"""
        n = 0
        for event_id, data in iter_events(lines):
            snap = _loads(data)
            if snap is None:
                with self._lock:
                    self._bad += 1
                continue
            self._note_event(event_id)
            self._on_snapshot(snap)
            n += 1
        return n


def _backoff_s(attempt: int) -> float:
    """指数退避并封顶。`attempt` 是连续失败次数；0 表示上次是正常结束，立刻重连。"""
    if attempt <= 0:
        return 0.0
    return min(BACKOFF_CAP, BACKOFF_BASE * (2 ** (attempt - 1)))


def _loads(data: str) -> Optional[Dict[str, Any]]:
    try:
        obj = json.loads(data)
    except Exception:
        return None
    return obj if isinstance(obj, dict) else None
