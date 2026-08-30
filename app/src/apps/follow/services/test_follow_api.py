# -*- coding: utf-8 -*-
"""
`/api/follow` 的路由级回归：状态码分类、WS 首帧、可选请求体。

跑法（要后端依赖，`app/.venv` 里才有 fastapi）：
    cd app/src && ../.venv/bin/python -m apps.follow.services.test_follow_api

只挂 `follow_router` 到一个干净的 FastAPI 上，**不 import main**：main 的 lifespan 会把 C++
相机服务连子进程一起拉起来（`camera_service.start_stream`），那是碰硬件的动作，不该是单测的副作用。
这里所有跟 C++ 的交互都用假 `_cpp` 或不可达端口表达，因此整个文件在没有相机、没有机械臂、
后端没起的机器上都能跑。
"""
from __future__ import annotations

import importlib.util
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

_HAS_FASTAPI = importlib.util.find_spec("fastapi") is not None

if _HAS_FASTAPI:                      # 缺依赖时整套跳过，而不是让 gate 变成"看环境颜色"
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from apps.follow.api import follow_router
    from apps.follow.services.follow_service import follow_service


_UNREACHABLE = "http://127.0.0.1:1"    # 端口 1 上没有监听：connect 立刻 refused，不占超时


@unittest.skipUnless(_HAS_FASTAPI, "需要 fastapi/httpx（app/.venv）")
class TestFollowApi(unittest.TestCase):
    def setUp(self):
        from apps.follow.services import follow_service as fs_mod
        self.fs_mod = fs_mod
        self.svc = follow_service
        # 备份：这些用例要动单例，不能把状态漏给同一次进程里的其它用例。
        self._saved = (dict(self.svc._arm), self.svc._R_cb, self.svc._R_cb_source,
                       self.svc._active, self.svc._baseline_q, self.svc._target_q,
                       self.svc._upstream_down, self.svc._fail_upstream,
                       self.svc._last_error, dict(self.svc._snapshot), fs_mod.CPP_BASE_URL)
        self.svc._active = False
        self.svc._baseline_q = None
        self.svc._target_q = None
        self.svc._ik_failed = False
        self.svc._last_error = ""
        self.svc._snapshot = {}
        self.svc._upstream_down = False
        self.svc._fail_upstream = False
        self.svc._arm["mode"] = "sim"
        self.svc._R_cb_source = "单测注入"
        if self.svc._R_cb is None:
            import numpy as np
            self.svc._R_cb = np.eye(3)
        fs_mod.CPP_BASE_URL = _UNREACHABLE           # 默认：相机服务不在
        self.svc._ensure_poller = lambda: None       # 单测里不许起轮询线程
        app = FastAPI()
        app.include_router(follow_router)
        self.client = TestClient(app)

    def tearDown(self):
        (mode, R_cb, src, active, base, tgt, down, failup, err, snap, url) = self._saved
        # _cpp/_ensure_poller 是被用例临时替换的**实例属性**：不逐个清干净，下一个用例会
        # 继承上一个的假实现，然后出现"明明该 503 却成功了"这种最难查的假绿。
        for attr in ("_cpp", "_ensure_poller"):
            self.svc.__dict__.pop(attr, None)
        self.svc._arm = mode
        self.svc._R_cb = R_cb
        self.svc._R_cb_source = src
        self.svc._active = active
        self.svc._baseline_q = base
        self.svc._target_q = tgt
        self.svc._upstream_down = down
        self.svc._fail_upstream = failup
        self.svc._last_error = err
        self.svc._snapshot = snap
        self.fs_mod.CPP_BASE_URL = url

    # ------------------------------------------------------------------ status
    def test_status_always_answers(self):
        """状态查询不该因为后端没起就 500 —— 页面正是靠它显示"后端未连接"。"""
        r = self.client.get("/api/follow/status")
        self.assertEqual(r.status_code, 200)
        body = r.json()
        for key in ("active", "arm_mode", "ik_failed", "joints_deg", "target_pose",
                    "r_cb_source", "camera_service_reachable", "follow"):
            self.assertIn(key, body)
        self.assertFalse(body["camera_service_reachable"])
        self.assertIn("error", body["follow"])

    # ------------------------------------------------------- 503 / 400 的分界
    def test_start_without_camera_service_is_503(self):
        r = self.client.post("/api/follow/start", json={})
        self.assertEqual(r.status_code, 503, r.text)
        self.assertIn("使能 follow 失败", r.json()["detail"])

    def test_real_mode_rejection_stays_400(self):
        """
        护栏类失败**不出网**，所以即使服务当前不可达也必须报 400：否则页面提示"去起后端"，
        而真正该改的是 `follow.arm.mode`。这条是 503/400 分界的反向保险。
        """
        self.svc._arm["mode"] = "real"
        r = self.client.post("/api/follow/start", json={})
        self.assertEqual(r.status_code, 400, r.text)
        self.assertIn("P5", r.json()["detail"])

    def test_cpp_refusal_is_400_not_503(self):
        """C++ 在**但拒绝**（例如标定模式里没有深度流）：够得着 ⇒ 400， remedy 是退出标定模式。"""
        self.svc._cpp = lambda kind, payload=None, timeout=1.0: (False, "标定模式下没有深度流", {})
        r = self.client.post("/api/follow/start", json={})
        self.assertEqual(r.status_code, 400, r.text)
        self.assertFalse(self.svc.last_failure_upstream)

    def test_zero_before_start_is_400(self):
        r = self.client.post("/api/follow/zero", json={})
        self.assertEqual(r.status_code, 400, r.text)
        self.assertIn("先点启动", r.json()["detail"])

    # ------------------------------------------------------------- 成功路径形状
    def test_start_success_returns_state(self):
        import numpy as np
        self.svc._cpp = lambda kind, payload=None, timeout=1.0: (
            True, "", {"enabled": True, "taught": True, "status": "no_frame", "frames": 0})
        self.svc._ensure_poller = lambda: None
        r = self.client.post("/api/follow/start", json={})
        self.assertEqual(r.status_code, 200, r.text)
        body = r.json()
        self.assertTrue(body["data"]["active"])
        self.assertEqual(body["data"]["joints_deg"], self.svc._arm["home_joints_deg"])
        self.assertEqual(len(body["data"]["target_pose"]), 6)
        # 基线 = home：启动按钮的语义就是"臂先回 home，再以那一刻为基准"
        self.assertTrue(np.allclose(body["data"]["arm_baseline_deg"],
                                    self.svc._arm["home_joints_deg"], atol=1e-6))
        self.client.post("/api/follow/stop", json={})

    def test_stop_clears_service_state_even_when_camera_service_is_gone(self):
        """停止必须几乎总是成功：留在"已使能"里退出，会把档位卡在 640x480 上。"""
        self.svc._active = True
        self.svc._target_q = self.svc._baseline_q = __import__("numpy").radians([0, 0, -90, -90, -90, 0])
        r = self.client.post("/api/follow/stop")     # _cpp 打向不可达端口 ⇒ 503
        self.assertEqual(r.status_code, 503, r.text)
        self.assertFalse(self.svc._active)           # 但服务侧状态照样清干净
        self.assertIsNone(self.svc._target_q)

    # ----------------------------------------------------------------- 请求体
    def test_body_is_optional(self):
        """`curl -XPOST /api/follow/stop` 这种裸调用不该吃到一个看不懂的 422。"""
        self.svc._cpp = lambda kind, payload=None, timeout=1.0: (True, "", {})
        self.assertEqual(self.client.post("/api/follow/start").status_code, 200)
        self.assertEqual(self.client.post("/api/follow/zero").status_code, 200)
        self.assertEqual(self.client.post("/api/follow/stop").status_code, 200)

    def test_bad_joints_deg_is_400(self):
        """NaN 走原始字节：stdlib json 连**发**都发不出去（httpx 会直接抛 ValueError），
        但接收侧的 json.loads 吃 NaN —— 所以这条钉的是"外部乱填也不能污染基线"。"""
        r = self.client.post("/api/follow/start",
                             content=b'{"joints_deg": [0, NaN, 0, 0, 0, 0]}',
                             headers={"Content-Type": "application/json"})
        self.assertEqual(r.status_code, 400, r.text)
        self.assertIn("非有限", r.json()["detail"])

    # -------------------------------------------------------------------- WS
    def test_ws_pushes_state_immediately(self):
        """连上就有一帧：页面要能立刻分辨"后端活着但没启用"和"我根本没连上"。"""
        with self.client.websocket_connect("/api/follow/ws") as ws:
            msg = ws.receive_json()
            self.assertEqual(msg["type"], "follow_state")
            for key in ("active", "ik_failed", "joints_deg", "target_pose", "follow"):
                self.assertIn(key, msg["data"])
            self.assertFalse(msg["data"]["active"])
            self.assertIsNone(msg["data"]["joints_deg"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
