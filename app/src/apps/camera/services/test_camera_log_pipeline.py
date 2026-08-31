# -*- coding: utf-8 -*-
"""
`camera_service.py` 日志消费链路的无相机回归。

跑法：
    cd app/src && python3 -m apps.camera.services.test_camera_log_pipeline

钉住的是一条真实发生过的故障链（日志背压）：
C++ 子进程的 stdout 由读线程抽排；一旦抽排被下游（python logging）卡住，64KB 管道写满，
C++ 侧所有打日志的线程（含处理 HTTP 请求的那条）被阻塞式 write 冻住 ——
页面上就是"使能 follow 失败，后端服务无响应"，而服务其实活着、切换也成功了。

所以这里要证明三件事：
* 读线程**只做抽排**：哪怕没有发射线程、缓冲早满了，它也把输入全部吃光（不反压）。
* 缓冲是有界的：超过上限丢最旧并计数，不无限涨内存。
* 发射线程把行还原成 python logging（格式解析没被两段式改造弄坏）。
"""
from __future__ import annotations

import io
import logging
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from apps.camera.services.camera_service import CameraService, camera_service  # noqa: E402


class _FakeProc:
    """只模拟用到的两样：可 readline 的 stdout、可 close。"""

    def __init__(self, text: str):
        self.stdout = io.StringIO(text)


class _RecordingHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record)


class TestLogPipelineNoBackpressure(unittest.TestCase):
    """核心契约：抽排永不被下游拖住。这是这次修复的全部意义。"""

    def setUp(self):
        self.svc = CameraService()
        # 单例共享的停止标志在本用例里必须是未置位状态，测完恢复。
        self._prev_stop = camera_service._stop_event.is_set()
        self.svc._stop_event.clear()

    def tearDown(self):
        self.svc._stop_event.set()
        with self.svc._log_cond:
            self.svc._log_cond.notify_all()
        if not self._prev_stop:
            camera_service._stop_event.clear()

    def test_reader_drains_all_input_without_an_emitter(self):
        """没有发射线程（等价于 logging 停摆）时，读线程也必须吃光全部输入。
        旧实现里读线程直接做 logging，这种情形下第一条就会卡死 —— 本用例就是旧代码的死刑判决。"""
        n = self.svc._log_buf.maxlen + 5000     # 故意灌超，验证有界且抽排不停
        proc = _FakeProc("".join(f"[I] [Camera] line-{i}\n" for i in range(n)))

        t = threading.Thread(target=self.svc._stream_cpp_logs, args=(proc,))
        t.start()
        t.join(timeout=10.0)
        self.assertFalse(t.is_alive(), "读线程卡住了 —— 管道抽排被下游拖住，背压故障复活")

        self.assertEqual(len(self.svc._log_buf), self.svc._log_buf.maxlen)
        self.assertEqual(self.svc._log_dropped, 5000)
        # 丢的是最旧的：缓冲里留下的行号必须从 n-8192 开始
        self.assertIn(f"line-{n - self.svc._log_buf.maxlen}", self.svc._log_buf[0])

    def test_emitter_turns_lines_into_python_logging(self):
        handler = _RecordingHandler()
        cpp_logger = logging.getLogger("camera.cpp")
        # 导入链里有人把这个 logger 调到 WARNING（减噪）；本用例验的是链路不是滤级别，钉成 INFO。
        prev_level = cpp_logger.level
        cpp_logger.setLevel(logging.INFO)
        cpp_logger.addHandler(handler)
        try:
            with self.svc._log_cond:
                self.svc._log_buf.append("[I] [Follow] 已受理档位切换请求（目标：使能）\n")
                self.svc._log_buf.append("[W] [Follow] 离群门拦下坏帧: 1.42\n")
                self.svc._log_cond.notify_all()

            emit = threading.Thread(target=self.svc._emit_cpp_logs, daemon=True)
            emit.start()

            deadline = time.monotonic() + 5.0
            while len(handler.records) < 2 and time.monotonic() < deadline:
                time.sleep(0.01)
            self.svc._stop_event.set()
            with self.svc._log_cond:
                self.svc._log_cond.notify_all()
            emit.join(timeout=3.0)

            self.assertEqual(len(handler.records), 2)
            self.assertEqual(handler.records[0].levelno, logging.INFO)
            self.assertIn("已受理档位切换请求", handler.records[0].getMessage())
            self.assertEqual(handler.records[1].levelno, logging.WARNING)
        finally:
            cpp_logger.removeHandler(handler)
            cpp_logger.setLevel(prev_level)

    def test_drop_counter_is_reported_once_per_batch(self):
        """丢行要能被看见（一条 warning），而不是静默吞掉。"""
        handler = _RecordingHandler()
        root = logging.getLogger()
        root.addHandler(handler)
        try:
            with self.svc._log_cond:
                self.svc._log_dropped = 42
                self.svc._log_buf.append("[I] [Camera] x\n")
                self.svc._log_cond.notify_all()

            emit = threading.Thread(target=self.svc._emit_cpp_logs, daemon=True)
            emit.start()

            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if any("丢行" in r.getMessage() for r in handler.records):
                    break
                time.sleep(0.01)
            self.svc._stop_event.set()
            with self.svc._log_cond:
                self.svc._log_cond.notify_all()
            emit.join(timeout=3.0)

            warns = [r.getMessage() for r in handler.records if r.levelno == logging.WARNING]
            self.assertTrue(any("42" in m for m in warns), warns)
        finally:
            root.removeHandler(handler)


if __name__ == "__main__":
    unittest.main(verbosity=2)
