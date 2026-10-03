# -*- coding: utf-8 -*-
"""
访问日志降噪过滤器的回归测试。

守住的口径: 只有"清单内的只读轮询路径 + GET + 2xx"才被静音; 任何 POST、任何非 2xx、
任何清单外的路径, 以及 uvicorn 的 record.args 形状对不上的情况, 都必须照常输出 ——
降噪的目的是不把动作与报警冲走, 绝不能把故障本身一起消掉。

运行:
    cd app/src && ../.venv/bin/python -m unittest test_access_log_filter
"""
from __future__ import annotations

import logging
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from main import PollingAccessFilter, _POLLING_GET_PATHS  # noqa: E402


def _access_record(method: str, path: str, status: str) -> logging.LogRecord:
    """按 uvicorn 的真实封包造一条 access 记录: (client_addr, method, path, http_version, status)。"""
    return logging.LogRecord(
        name="uvicorn.access",
        level=logging.INFO,
        pathname=__file__,
        lineno=485,
        msg='%s - "%s %s HTTP/%s" %s',
        args=("127.0.0.1:57566", method, path, "1.1", status),
        exc_info=None,
    )


class PollingAccessFilterTest(unittest.TestCase):
    def setUp(self):
        self.filter = PollingAccessFilter()

    def test_every_configured_polling_path_is_muted_on_ok_get(self):
        """清单里每一条路径都必须真的被静音 (否则等于没降噪)。"""
        self.assertGreaterEqual(len(_POLLING_GET_PATHS), 1)
        for path in _POLLING_GET_PATHS:
            self.assertFalse(self.filter.filter(_access_record("GET", path, "200")), path)

    def test_query_string_does_not_evade_the_filter(self):
        self.assertFalse(self.filter.filter(_access_record("GET", "/api/robot/state?ts=1", "200")))

    def test_action_requests_are_never_muted(self):
        """POST 是动作 (运动/开关喷), 一条都不能少 —— 这正是事后归因要看的东西。"""
        self.assertTrue(self.filter.filter(_access_record("POST", "/api/robot/aim_at_pixel", "200")))
        self.assertTrue(self.filter.filter(_access_record("POST", "/api/robot/state", "200")))
        self.assertTrue(self.filter.filter(_access_record("DELETE", "/api/robot/paths/3", "200")))

    def test_any_non_2xx_answer_is_kept(self):
        """轮询接口出错 (400/500) 时 detail 里带英文原因, 必须留在日志里。"""
        for status in ("400", "404", "500", "503"):
            self.assertTrue(self.filter.filter(_access_record("GET", "/api/robot/state", status)), status)

    def test_other_get_endpoints_still_show_up(self):
        for path in ("/api/camera/intrinsics", "/api/robot/home", "/api/follow/status"):
            self.assertTrue(self.filter.filter(_access_record("GET", path, "200")), path)

    def test_non_access_records_pass_through_untouched(self):
        app_record = logging.LogRecord("apps.robot.api", logging.INFO, __file__, 10, "moved", None, None)
        self.assertTrue(self.filter.filter(app_record))

    def test_unexpected_record_shape_is_kept_not_dropped(self):
        """uvicorn 换了封包结构时宁可继续吵, 也不能顺手把日志过滤成空。"""
        for bad_args in (None, ("only", "two"), "GET /api/robot/state HTTP/1.1 200"):
            record = logging.LogRecord("uvicorn.access", logging.INFO, __file__, 1, "%s", bad_args, None)
            self.assertTrue(self.filter.filter(record), str(bad_args))


if __name__ == "__main__":
    unittest.main(verbosity=2)
