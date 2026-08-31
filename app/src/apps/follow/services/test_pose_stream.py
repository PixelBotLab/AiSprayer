# -*- coding: utf-8 -*-
"""
pose_stream 的离线测试：不连任何服务，只测"能连上之后才会暴露"的那几处算术和解析。

为什么值得单独测：这条链路的失效方式全部是**安静**的 —— SSE 解析错一个字段就是"连上了但
一帧都没进"，退避算错就是"服务重启的那两秒里疯狂重连"，而这两种从页面上看都跟"相机没动"
一模一样。所以把纯函数（帧解析、退避、新鲜度判定）单独摆出来测，而不是只在集成里顺带覆盖。

跑法（unittest 与 pytest 都能跑，本机 venv 里没有 pytest，所以两种都要留）：
    cd app/src && python3 -m apps.follow.services.test_pose_stream
    cd app/src && python3 -m pytest apps/follow/services/test_pose_stream.py
"""
from __future__ import annotations

import json
import os
import sys
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from apps.follow.services.pose_stream import (  # noqa: E402
    BACKOFF_BASE, BACKOFF_CAP, PoseStream, _backoff_s, iter_events,
)


def _frame(**over) -> bytes:
    body = {"status": "ok", "frames": 1}
    body.update(over)
    return json.dumps(body).encode()


def _sse(*lines: bytes) -> list:
    """拼成一帧 SSE 的字节行（含收尾空行）。"""
    out = []
    for ln in lines:
        out.append(ln)
    out.append(b"")
    return out


def _stream(**kw) -> PoseStream:
    return PoseStream("http://127.0.0.1:1", lambda s: None, **kw)


class TestIterEvents(unittest.TestCase):
    """帧解析器：纯函数，也是整条推送链上唯一"错了会一声不响"的一段。"""

    def test_单帧事件解析出载荷与id(self):
        evs = list(iter_events(_sse(b"id: 42", b"data: " + _frame(frames=7))))
        self.assertEqual(evs, [("42", json.dumps({"status": "ok", "frames": 7}))])

    def test_心跳注释不产生事件(self):
        # 服务端没新快照时只发 `: ping`。把它当事件 ⇒ 每 200ms 一次"解析失败的载荷"，
        # 真正的坏载荷就被淹在里面了。
        evs = list(iter_events([b": ping\n\n".rstrip(), b"", b": ping", b"", b"data: " + _frame(), b""]))
        self.assertEqual(len(evs), 1)

    def test_多行data合并成一个事件(self):
        evs = list(iter_events([b'data: {"a":', b"data: 1}", b""]))
        self.assertEqual(len(evs), 1)
        self.assertEqual(evs[0][1], '{"a":\n1}')

    def test_连续两帧各自成事件不串味(self):
        lines = [b"id: 1", b"data: " + _frame(frames=1), b"",
                 b"id: 2", b"data: " + _frame(frames=2), b""]
        evs = list(iter_events(lines))
        self.assertEqual([(i, json.loads(d)["frames"]) for i, d in evs], [("1", 1), ("2", 2)])

    def test_流结尾没有空行也交出最后一帧(self):
        # 服务端被杀时最后一段往往没有收尾空行。丢掉它 = 每次断线都恰好丢最后一帧。
        evs = list(iter_events([b"data: " + _frame(frames=9)]))
        self.assertEqual(len(evs), 1)
        self.assertEqual(json.loads(evs[0][1])["frames"], 9)

    def test_未知字段被忽略而不是报错(self):
        # 服务端以后加 event:/自定义字段 不该弄死已经在跑的订阅端。
        evs = list(iter_events([b"event: pose", b"retry: 200", b"data: " + _frame(), b""]))
        self.assertEqual(len(evs), 1)

    def test_只有注释时一个事件也不交(self):
        # 心跳一整轮 ⇒ 上层不许看到"空事件"，否则每 200ms 白跑一次 _loads 并计一次坏载荷。
        self.assertEqual(list(iter_events([b": ping", b"", b": ping", b""])), [])


class TestBackoff(unittest.TestCase):
    def test_退避指数增长并封顶(self):
        self.assertEqual(_backoff_s(0), 0.0)          # 上次是自然结束 ⇒ 立刻重连
        self.assertAlmostEqual(_backoff_s(1), BACKOFF_BASE)
        self.assertAlmostEqual(_backoff_s(2), BACKOFF_BASE * 2)
        self.assertAlmostEqual(_backoff_s(3), BACKOFF_BASE * 4)
        self.assertAlmostEqual(_backoff_s(30), BACKOFF_CAP)   # 服务没起时不许无限拉长


class TestPoseStreamState(unittest.TestCase):
    """订阅端自己的账本：新鲜度、计数、以及"没线程时不许假装在跑"。"""

    def test_未收到事件时新鲜度为none(self):
        s = _stream()
        self.assertIsNone(s.last_event_age())
        st = s.stats()
        self.assertFalse(st["running"])
        self.assertEqual(st["events"], 0)
        self.assertIsNone(st["last_event_age_s"])

    def test_一帧坏载荷只计数不抛异常(self):
        got = []
        s = PoseStream("http://x", got.append)
        n = s._consume_for_test([b"data: not-json", b"", b"data: " + _frame(frames=3), b""])
        self.assertEqual(n, 1)                        # 只有好的那一帧交给了上层
        self.assertEqual(s.stats()["bad_payloads"], 1)
        self.assertTrue(got)
        self.assertEqual(got[0]["frames"], 3)

    def test_非dict载荷按坏数据处理(self):
        got = []
        s = PoseStream("http://x", got.append)
        self.assertEqual(s._consume_for_test([b"data: [1,2,3]", b""]), 0)
        self.assertEqual(s.stats()["bad_payloads"], 1)
        self.assertFalse(got)

    def test_收到事件后新鲜度开始计时(self):
        s = _stream()
        s._note_event("7")
        age = s.last_event_age()
        self.assertIsNotNone(age)
        self.assertGreaterEqual(age, 0.0)
        self.assertLess(age, 1.0)
        self.assertEqual(s.stats()["last_event_id"], "7")

    def test_连接没起来时不留下运行中的线程(self):
        # 端口上没人监听：线程会退避重连，但**必须还活着**（服务随后起来就能接上），
        # 而且不能把异常抛回调用方。
        s = PoseStream("http://127.0.0.1:1/api", lambda d: None, name="t-dead")
        s.start()
        try:
            time.sleep(0.15)
            self.assertTrue(s.running)
            self.assertEqual(s.stats()["events"], 0)
        finally:
            s.stop()
        self.assertFalse(s.running)
        s.stop()                                      # 幂等：再停一次不许炸

    def test_stop可以安全重复调用(self):
        s = _stream()
        s.stop()
        s.stop()
        self.assertFalse(s.running)


if __name__ == "__main__":
    unittest.main(verbosity=2)
