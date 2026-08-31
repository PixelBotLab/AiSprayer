# -*- coding: utf-8 -*-
"""
FollowService：把相机服务里跑的 follow（进程内）变成"页面上能用的跟随"。

三件事：
  1. **代理** C++ 的三个端点（使能 / 示教 / 读快照）。C++ 侧才是唯一懂设备的地方；这里不碰
     pipeline、不碰锁，也就不会出现"两边各自认为相机在某个档位"的分歧。
  2. **合成**：把相机增量映射到臂的基座系，做最近分支 IK，得到仿真臂该去的关节角（`mirror.py`）。
  3. **广播**：按 robot_service 的风格注册回调，WS 层把它变成 `{"type":"follow_state",...}`。

臂的基线**只在示教那一刻取一次**（启动 = home，调零 = 当时位姿），之后每帧都算
`T_target = Δ_base(当前增量) · T_baseline`。不拿实时位姿当基准 —— 那会形成正反馈：臂跟着自己
上一帧的解算误差继续走，静止时的 0.2 mm 噪声也能慢慢把它推走。

真实臂本轮**只预留接口与开关**：`follow.arm.mode: real` 一旦被调用到需要发指令的路径，
直接返回失败说"未接入（P5）"。预留的是接缝，不是一个假装能发、实际什么都不做的空函数。
"""
from __future__ import annotations

import logging
import math
import queue
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import requests

from apps.camera.services.camera_service import CPP_BASE_URL
from apps.follow.mirror import (
    joints_to_target, pose_ctrl_from_target, rotation_camera_to_base,
    rotation_camera_to_base_fallback,
)
from apps.follow.services.pose_stream import PoseStream
from apps.follow.trajectory import JointTrajectorySmoother
from core.config import sprayer_config

logger = logging.getLogger(__name__)

# C++ 侧的使能/关闭是**提交式**的：POST /follow 受理即返回（202），重启取流 pipeline 在它的
# 专用线程里跑，实测百毫秒到几十秒。所以这里的控制类调用分两段：一次短请求提交（TIMEOUT_CONTROL），
# 再轮询 /follow/status 的 switching 字段等终态（总预算 SWITCH_DEADLINE）。示教是同步的快操作，
# 仍用 TIMEOUT_CONTROL；轮询必须是"快"超时 —— 轮询慢了会把页面读数拖成 1 Hz。
TIMEOUT_CONTROL = 20.0
TIMEOUT_POLL = 0.5
SWITCH_DEADLINE = 60.0        # 提交 + 轮询确认的总预算：重启取流再慢也不该超过它
SWITCH_POLL_INTERVAL = 0.3
SWITCH_POLL_TIMEOUT = 5.0     # 单次状态轮询的超时；连续失败几次才判服务没了，容忍偶发抖动

_FOLLOW_PATHS = {
    "toggle": "/api/v1/camera/follow",
    "teach": "/api/v1/camera/follow/teach",
    "status": "/api/v1/camera/follow/status",
}


class FollowService:
    """单例。由 api.py 的路由与一条自己的轮询线程共同访问，所有可变状态都在 `_lock` 下。"""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        # CR5Kinematics 的 cpp 后端用 per-instance ctypes 缓冲（源码注释：one solver per
        # thread）。路由线程要 FK、轮询线程要 IK ⇒ 必须串行化，否则两边共用同一块缓冲。
        self._kin_lock = threading.Lock()
        self._kin = None

        self._ws_callbacks: List[Callable[[dict], None]] = []
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._active = False              # 页面点过"启动"、还没"停止"
        self._baseline_q: Optional[np.ndarray] = None   # rad，示教那一刻的臂位姿
        self._target_q: Optional[np.ndarray] = None     # rad，最近一次成功的目标
        self._ik_failed = False
        self._last_error = ""
        self._snapshot: Dict[str, Any] = {}
        self._last_frames = -1
        self._last_emit_key = None

        # 33 Hz 发射链：轮询线程解出 keyframe → 队列 → 发射线程过平滑器 → WS。
        # 两条线程分开：轮询率决定 keyframe 产生速率（IK 负载），发射率决定页面看到的帧率，
        # 谁也不许拖住谁。
        self._kf_queue: "queue.Queue[np.ndarray]" = queue.Queue()
        self._kf_pushed = 0                            # 轮询线程产生的 keyframe 数（发射日志统计用）
        self._emit_q_deg: Optional[List[float]] = None   # 最近一次发射的 joints（REST 查询用）
        self._last_broadcast_joints: Optional[List[float]] = None  # 发射去重：值不变就不重发/不重算 FK
        self._emit_thread: Optional[threading.Thread] = None

        self._R_cb = None
        self._R_cb_source = ""
        # 路由要把"相机服务没起来"（503，页面该提示后端）和"这个请求本身不成立"（400，配置/
        # 模式/基线问题）分开。靠猜 msg 前缀太脆 ⇒ 由 _cpp 维护一个明确的布尔。
        self._upstream_down = False
        # 可达性是"最近一次访问的结果"，而 503 要的是"**这次**失败是不是因为访问不了"：
        # 模式/标定这类护栏根本不出网，拿上面那个布尔判会把它们错标成后端没起。
        self._fail_upstream = False

        self._arm = self._read_arm_config()
        # 数据面：实时位姿优先由相机服务**推**（pose_stream），推不动了才退回上面那条轮询线程。
        # 流对象**延迟创建**（第一次"启动"时才 new）：导入本模块不该开出任何 socket，而没在跟随
        # 的时候也不该占着服务端一路订阅配额（那边上限只有 4 路）。
        self._push = None
        self._push_enabled = self._arm["push"]
        self._push_stale_s = self._arm["push_stale_ms"] / 1000.0
        self._smoother = JointTrajectorySmoother(math.radians(self._arm["max_joint_vel_deg_s"]))
        self._resolve_camera_to_base()

    # ------------------------------------------------------------------ 配置
    @staticmethod
    def _read_arm_config() -> dict:
        arm = (sprayer_config.config_data.get("follow", {}) or {}).get("arm", {}) or {}
        home = arm.get("home_joints_deg", [0.0, 0.0, -90.0, -90.0, -90.0, 0.0])
        home = [float(v) for v in home]
        if len(home) != 6:
            logger.warning("follow.arm.home_joints_deg 需要 6 个值，收到 %d 个 —— 退回默认 home", len(home))
            home = [0.0, 0.0, -90.0, -90.0, -90.0, 0.0]
        return {
            "mode": str(arm.get("mode", "sim")).lower(),
            "home_joints_deg": home,
            "fallback_euler_deg": [float(v) for v in
                                   (arm.get("camera_to_base_fallback_euler_deg") or [0.0, 0.0, 0.0])][:3],
            "poll_hz": max(1, min(int(arm.get("poll_hz", 20)), 50)),
            "emit_hz": max(5, min(int(arm.get("emit_hz", 33)), 100)),
            # 数据面优先走服务端推送；false = 只要轮询（服务端没编到 SSE 路由、或想对照两种
            # 模式的延迟时用）。关掉它不会让跟随变差，只是退回改动前的行为。
            "push": bool(arm.get("push", True)),
            # 多久没收到推送就判"这条流此刻不算数"、改由轮询兜底。下限要能容下服务端 200ms
            # 心跳 + 一帧周期（15fps≈66ms），上限不能比改动前的轮询周期(50ms)对应的危害更大 ——
            # 取 400ms：约两个心跳周期，坏流在半秒内被接管，而正常运行永远碰不到它。
            "push_stale_ms": max(200, min(int(arm.get("push_stale_ms", 400)), 5000)),
            "max_joint_vel_deg_s": max(1.0, float(arm.get("max_joint_vel_deg_s", 90.0))),
            "teach_save_map": bool(arm.get("teach_save_map", True)),
        }

    def _resolve_camera_to_base(self) -> None:
        """
        相机轴 → 基座轴。优先手眼标定结果，退路才是配置常量 —— **用哪个必须能被看见**，
        因为两者给出的是不同的平移方向映射，悄悄降级比直接失败更危险。
        """
        self._R_cb = None
        self._R_cb_source = ""
        try:
            T = sprayer_config.T_camera_to_base
        except Exception as e:                                   # 解析失败也要能说清是哪儿失败
            self._R_cb_source = f"标定结果读取异常：{e}"
            logger.warning("%s", self._R_cb_source)
            return
        if T is not None:
            try:
                self._R_cb = rotation_camera_to_base(T)
                self._R_cb_source = "手眼标定 T_base_camera（旋转块）"
                logger.info("follow 基座←相机轴映射: %s", self._R_cb_source)
                return
            except ValueError as e:
                self._R_cb_source = f"标定矩阵不可用：{e}"
                logger.warning("%s", self._R_cb_source)
        else:
            self._R_cb_source = ("标定结果缺失，或当前安装是 eye-in-hand（相机位姿不是常量，"
                                 "不能当固定轴映射用）")
        try:
            self._R_cb = rotation_camera_to_base_fallback(self._arm["fallback_euler_deg"])
            self._R_cb_source += " → 已退回配置常量 follow.arm.camera_to_base_fallback_euler_deg（降级，方向会有偏差）"
            logger.warning("follow: %s", self._R_cb_source)
        except ValueError as e:
            self._R_cb = None
            self._R_cb_source = f"退路常量也读不出来：{e}"
            logger.error("follow: %s", self._R_cb_source)

    # ------------------------------------------------------------ C++ 代理
    @property
    def camera_service_down(self) -> bool:
        """True = 最近一次访问相机服务连不上（只描述后端存活，不能用来自分类任意一次失败）。"""
        with self._lock:
            return self._upstream_down

    @property
    def last_failure_upstream(self) -> bool:
        """最近一次被拒绝的调用，是不是因为够不着相机服务。路由据此选 503 / 400。"""
        with self._lock:
            return self._fail_upstream

    def _cpp(self, kind: str, payload: Optional[dict] = None,
             timeout: float = TIMEOUT_POLL) -> Tuple[bool, str, dict]:
        """返回 (ok, msg, data)。C++ 侧的 {code,msg,data} 只在这一层拆开。"""
        url = f"{CPP_BASE_URL}{_FOLLOW_PATHS[kind]}"
        try:
            if payload is None:
                r = requests.get(url, timeout=timeout)
            else:
                r = requests.post(url, json=payload, timeout=timeout)
        except requests.exceptions.Timeout as e:
            # 超时 ≠ 服务不在：它可能只是此刻很慢（曾把"其实已受理"的切换误报成"无响应"）。
            # 不把 _upstream_down 置真，下一次请求照常尝试；真实在性由连接类异常判定。
            return False, f"相机服务响应超时（{timeout:.0f}s，服务可能正忙，稍后重试）: {e}", {}
        except Exception as e:
            with self._lock:
                self._upstream_down = True
            return False, f"相机服务连不上（{url} 没起来？后端进程在跑吗）: {e}", {}
        with self._lock:
            self._upstream_down = False      # 只要有一次真实回复，就认定后端活着
        try:
            body = r.json()
        except Exception:
            return False, f"相机服务返回了非 JSON（HTTP {r.status_code}）: {r.text[:120]}", {}
        data = body.get("data", {}) or {}
        ok = (r.status_code // 100 == 2) and int(body.get("code", -1)) == 0
        return ok, str(body.get("msg", "")), data

    def _fetch_snapshot(self) -> dict:
        ok, msg, data = self._cpp("status")
        if not ok:
            return {"error": msg or "读不到 follow 状态"}
        return data

    def _toggle_and_wait(self, enabled: bool) -> Tuple[bool, str, dict]:
        """
        提交一次档位切换并等它到终态。返回 (ok, msg, 最终快照)。
        C++ 侧三种回应都进轮询：受理（202，switching）、已在切换（409，别人先点了）、
        同态无操作（202 且非 switching）—— 最后一种不用等，当场就是终态。
        """
        ok, msg, data = self._cpp("toggle", {"enabled": enabled}, timeout=TIMEOUT_CONTROL)
        if not ok and not (data or {}).get("switching"):
            # 既没受理也没人在切：是真的被拒了（配置坏/标定模式挡着/服务不在）。
            with self._lock:
                self._fail_upstream = bool(self._upstream_down)
            return False, msg or self._reason_of(data), data
        if not (data or {}).get("switching"):
            return True, msg, data              # 同态无操作：目标态已达成
        return self._wait_switch_done(enabled)

    def _wait_switch_done(self, enabled: bool) -> Tuple[bool, str, dict]:
        """轮询直到 switching 落下：到目标态 = 成；没到 = 把 C++ 记的失败原因原样交回去。"""
        deadline = time.monotonic() + SWITCH_DEADLINE
        fails = 0
        while time.monotonic() < deadline:
            ok, msg, data = self._cpp("status", timeout=SWITCH_POLL_TIMEOUT)
            if not ok:
                fails += 1
                # 连续几次都够不着才判服务没了：单次抖动（比如切换瞬间的端口重建）不该杀掉整个流程。
                if fails >= 3:
                    with self._lock:
                        self._fail_upstream = True
                    return False, msg or "轮询档位切换状态失败（相机服务无响应）", {}
                time.sleep(SWITCH_POLL_INTERVAL)
                continue
            fails = 0
            if data.get("switching"):
                time.sleep(SWITCH_POLL_INTERVAL)
                continue
            if bool(data.get("enabled")) == enabled:
                return True, "", data
            # 没在切也没到目标态：切换失败了。reason 是 C++ 切换线程写的失败原话。
            with self._lock:
                self._fail_upstream = False
            return False, data.get("reason") or "档位切换未达到目标状态", data
        with self._lock:
            self._fail_upstream = False
        return False, f"档位切换超时（{SWITCH_DEADLINE:.0f}s 未完成）：请查相机服务日志", {}

    # --------------------------------------------------------- 三个按钮的语义
    def start(self, joints_deg: Optional[List[float]] = None) -> Tuple[bool, str]:
        """
        ① 启动：使能 follow（相机会切到 `follow.camera` 那一档：仓库里定在 640x480@15，
        因为那是硬件 D2C 的上限档）→ 示教并落盘 → 臂基线 = home。
        `joints_deg` 给了就用它当基线（页面此刻的仿真臂位姿），没给才用 home。
        """
        return self._begin(which="start", joints_deg=joints_deg, teach=True)

    def zero(self, joints_deg: Optional[List[float]] = None) -> Tuple[bool, str]:
        """
        ② 调零：重新示教 + 把臂基线换成**当前**位姿。已经在跟随中途工件被搬走、或装歪了时用，
        不用先停止再启动 —— 基线一换，增量归零，臂停在它此刻该在的地方。
        """
        if not self._active:
            with self._lock:
                self._fail_upstream = False
            return False, "还没启动跟随：调零要求 follow 已使能（先点启动）"
        return self._begin(which="zero", joints_deg=joints_deg, teach=True)

    def stop(self) -> Tuple[bool, str]:
        """③ 停止：关掉使能（取流退回 hardware.camera），并让页面把臂的控制权交回真机状态。"""
        ok, msg, data = self._toggle_and_wait(False)
        with self._lock:
            self._fail_upstream = bool(self._upstream_down)
            self._active = False
            self._baseline_q = None
            self._target_q = None
            self._ik_failed = False
            self._last_frames = -1
            self._last_emit_key = None
            self._emit_q_deg = None
            self._last_broadcast_joints = None   # 清去重基准；发射线程若还在收尾顶多多播一次同值，无害
            if data:
                self._snapshot = data
        # 旧基线下的段不属于下一次启动；队列里的旧 keyframe 一并丢掉。
        self._smoother.reset()
        self._kf_queue = queue.Queue()
        self._stop_stream()             # 必须在锁外（见 _stop_stream）；先断进来的一条再收尾
        self._stop_poller_if_idle()
        self._emit(force=True)               # 让前端拿到"已停止"，从而把 simJoints 置回 null
        if not ok:
            return False, msg
        # 报"实测回到了哪一档"而不是"配置里写的是哪一档"：这两个在设备拒了档位时会不一样，
        # 而运维要看的是相机此刻在跑什么。
        snap = data or {}
        w, h = snap.get("capture_width"), snap.get("capture_height")
        return True, (f"已停止跟随：取流已回到 {w}x{h}，参考地图仍留在相机服务里" if w and h
                      else "已停止跟随：取流已退回 hardware.camera 档位，参考地图仍留在相机服务里")

    def _begin(self, which: str, joints_deg: Optional[List[float]], teach: bool) -> Tuple[bool, str]:
        cfg = self._arm
        if cfg["mode"] != "sim":
            # 真实臂路径本轮不实现。返回失败而不是"接受请求但什么都不发"：后者会让页面以为
            # 臂在跟，而实际上一台真机一动不动 —— 在有人真接上臂之前，这种假装是最坏的失效。
            with self._lock:
                self._fail_upstream = False
            return False, (f"follow.arm.mode={cfg['mode']}：真实臂控制路径未接入（P5），已拒绝。"
                           "改用 mode: sim 只驱动仿真臂。")
        if self._R_cb is None:
            with self._lock:
                self._fail_upstream = False
            return False, f"拿不到基座←相机轴映射，跟随方向无法确定。{self._R_cb_source}"

        baseline, note = self._pick_baseline(which, joints_deg)
        if baseline is None:
            with self._lock:
                self._fail_upstream = False
            return False, note

        if which == "start":
            ok, msg, data = self._toggle_and_wait(True)
            if not ok:
                return False, f"使能 follow 失败：{msg or self._reason_of(data)}"
            with self._lock:
                self._snapshot = data or self._snapshot
        if teach:
            ok, msg, data = self._cpp("teach", {"save_map": cfg["teach_save_map"]},
                                      timeout=TIMEOUT_CONTROL)
            if not ok:
                with self._lock:
                    self._fail_upstream = bool(self._upstream_down)
                return False, f"示教失败：{msg or self._reason_of(data)}"
            with self._lock:
                self._snapshot = data or self._snapshot
                self._fail_upstream = False

        with self._lock:
            self._active = True
            self._baseline_q = baseline
            self._target_q = baseline.copy()   # 增量=0 ⇒ 目标就是基线；页面据此把臂摆到位
            self._ik_failed = False
            self._last_error = ""
            self._last_frames = -1
            # 平滑器从基线起步：发射线程此时还没起（下面才 ensure），当场碰它是安全的。
            self._smoother.reset()
            self._kf_queue = queue.Queue()
            self._smoother.push_target(baseline)
            self._emit_q_deg = [round(float(v), 4) for v in np.degrees(baseline)]
        self._ensure_poller()
        self._ensure_stream()      # 数据面优先走推送；轮询线程留在后面当兜底
        self._emit(force=True)
        logger.info("follow %s：基线=%s°  %s", which,
                    np.round(np.degrees(baseline), 2).tolist(), note)
        return True, (f"{'启动' if which == 'start' else '调零'}完成：已示教并锁定臂基线"
                      + (f"（{note}）" if note else ""))

    @staticmethod
    def _reason_of(data: dict) -> str:
        return str((data or {}).get("reason", "") or "")

    def _pick_baseline(self, which: str,
                       joints_deg: Optional[List[float]]) -> Tuple[Optional[np.ndarray], str]:
        """
        基线优先级（写死在这里，别在三个调用点各排一次序）：
          1) 请求带来的 joints_deg —— 页面才是"此刻仿真臂在哪儿"的唯一知情者（URDF 约定，度）；
          2) 服务自己上一次的目标 —— 页面刚刷新、还没收到 WS 时至少接着自己的数走；
          3) 启动专用兜底 = home；调零**不兜底**（"当前位姿"拿不到就必须喊，而不是偷偷用 home
             当基线 —— 那会把臂瞬移走，而用户刚点的明明是"就停在这儿"）。

        真机反馈**故意不在这里**：`cr5_kinematics` 明写 DH 的 q2/q4 相对 URDF 偏 ±π/2，控制器
        回报的 J2/J4 与 URDF 关节角差 90°，直接拿来当基线会让臂摆到一个错位 90° 的位姿上。
        那条换算属于 P5（真机控制路径），不是这里该顺手补的。
        """
        if joints_deg is not None and len(joints_deg) >= 6:
            q = np.asarray([float(v) for v in joints_deg[:6]], dtype=np.float64)
            if not np.all(np.isfinite(q)):
                return None, "joints_deg 含非有限值"
            return np.radians(q), "基线取自请求里的当前关节角"
        with self._lock:
            last = None if self._target_q is None else self._target_q.copy()
        if last is not None:
            return last, "请求没带关节角，沿用上一次的跟随目标"
        if which == "zero":
            return None, ("调零需要臂的当前位姿当基线：页面没传 joints_deg，也没有可沿用的跟随目标。"
                          "真机关节反馈不能当基线用（控制器 J2/J4 与 URDF 差 ±90°，见 P5）")
        return np.radians(np.asarray(self._arm["home_joints_deg"], dtype=np.float64)), \
            "基线取 home（仿真臂先摆到 home 再锁基准）"

    # ---------------------------------------------------------------- 状态
    def status(self) -> Dict[str, Any]:
        """给 REST 用。在跟随中就返回轮询缓存（不额外压 C++），否则即时拉一次。"""
        with self._lock:
            active = self._active
            snap = dict(self._snapshot)
            baseline = None if self._baseline_q is None else np.degrees(self._baseline_q).tolist()
            target = None if self._target_q is None else np.degrees(self._target_q).tolist()
        if not active:
            # 没在跟随：缓存可能是上一次停止时的，即时拉一次。**必须拉在组装载荷之前** ——
            # 否则载荷里的 camera_service_reachable 说的是"上一次访问"的结果，第一次查状态
            # 就会在相机服务根本没起的情况下报出 reachable=true。
            snap = self._fetch_snapshot()
        with self._lock:
            dp = self._data_plane_locked()
            payload = {
                "active": active,
                "arm_mode": self._arm["mode"],
                "poll_hz": self._arm["poll_hz"],
                "emit_hz": self._arm["emit_hz"],
                "max_joint_vel_deg_s": self._arm["max_joint_vel_deg_s"],
                "home_joints_deg": self._arm["home_joints_deg"],
                "r_cb_source": self._R_cb_source,
                "r_cb_ready": self._R_cb is not None,
                "camera_service_reachable": not self._upstream_down,
                "ik_failed": self._ik_failed,
                "last_error": self._last_error,
                "arm_baseline_deg": baseline,
                "arm_target_deg": target,
                "data_plane": dp,
                "follow": snap,
            }
        # joints_deg / target_pose 与 WS 广播同名：页面用一套渲染代码吃掉两条来源。
        # joints 以 33 Hz 发射流为准（平滑后的）；还没发射过才退回目标值。
        payload["joints_deg"] = self._emit_q_deg or target
        payload["target_pose"] = self._target_pose_deg()
        # 数据面也同名：完整计数只在这条 REST 里给（WS 每 33 Hz 一次，塞进去是白付带宽），
        # 但页面读"此刻走的是哪条路、为什么"两处用的是同一对键。
        payload["data_plane_mode"] = dp["mode"]
        payload["data_plane_reason"] = dp.get("reason", "")
        return payload

    def _state_payload(self) -> dict:
        with self._lock:
            dp = self._data_plane_locked()
            return {
                "active": self._active,
                "arm_mode": self._arm["mode"],
                "r_cb_source": self._R_cb_source,
                "ik_failed": self._ik_failed,
                "last_error": self._last_error,
                # 只带 mode 和退回原因：这条载荷每 33 Hz 发一次，把推送计数器塞进来是白付带宽。
                # 完整计数在 REST status() 里，页面按需查。
                "data_plane_mode": dp["mode"],
                "data_plane_reason": dp.get("reason", ""),
                "arm_baseline_deg": (None if self._baseline_q is None
                                     else [round(float(v), 4) for v in np.degrees(self._baseline_q)]),
                "arm_target_deg": (None if self._target_q is None
                                   else [round(float(v), 4) for v in np.degrees(self._target_q)]),
                "follow": dict(self._snapshot),
            }

    # ------------------------------------------------------- 数据面：推送订阅的生命周期
    def _ensure_stream(self) -> None:
        """
        起订阅线程（幂等）。**不得持 _lock 调用**：见 _stop_stream 里关于死锁的说明。
        流对象在这里才 new —— 导入模块不该连 socket，而"没在跟随"时也不该占服务端订阅配额。
        """
        with self._lock:
            if not self._push_enabled:
                return
            push, first = self._push, self._push is None
        if first:
            # 构造放在锁外、发布放在锁里：PoseStream.__init__ 很轻，但"谁都能重复 new 一个"
            # 这种竞态要用发布检查来收，而不是靠把整段圈进锁里。
            push = PoseStream(CPP_BASE_URL, self._on_push_snapshot)
            with self._lock:
                if self._push is None:
                    self._push = push
                push = self._push
        push.start()

    def _stop_stream(self) -> None:
        """
        停订阅线程并**丢掉引用**。**只能在 _lock 之外调用**：join 要等读者线程退出，而那个线程的
        回调 `_on_push_snapshot` 正等着同一把锁 —— 持锁来停就是自己把自己锁死到超时。
        （_lock 是 RLock，同线程可重入，但跨线程的这一等一抢没有任何可重入能救。）

        引用也一起丢：留着它，停止后的状态会照样报"数据面=push"（`last_event_age` 还是新鲜的），
        而下一次"启动"会另建一条流 —— 那条旧引用只会把"此刻到底谁在推数据"说成一句假话。
        """
        with self._lock:
            push, self._push = self._push, None
        if push is not None:
            # join 带超时：读者可能还卡在 socket 读上（那种情况 PoseStream 自己留日志）。
            # 真迟到一帧也无害 —— _on_push_snapshot 先看 _active，而它已经被上面清掉了。
            push.stop()

    # -------------------------------------------------------------- 轮询/解算
    def _ensure_poller(self) -> None:
        with self._lock:
            poll_alive = self._thread is not None and self._thread.is_alive()
            emit_alive = self._emit_thread is not None and self._emit_thread.is_alive()
            if poll_alive and emit_alive:
                return
            self._stop_evt.clear()
            if not poll_alive:
                self._thread = threading.Thread(target=self._poll_loop, name="follow-poll", daemon=True)
                self._thread.start()
            if not emit_alive:
                self._emit_thread = threading.Thread(target=self._emit_loop, name="follow-emit", daemon=True)
                self._emit_thread.start()

    def _stop_poller_if_idle(self) -> None:
        with self._lock:
            if self._active or (self._thread is None and self._emit_thread is None):
                return
            self._stop_evt.set()
            self._thread = None
            self._emit_thread = None

    def _poll_loop(self) -> None:
        period = 1.0 / self._arm["poll_hz"]
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            try:
                self._poll_once()
            except Exception as e:                                # 轮询线程不许因一次异常就死掉
                logger.error("follow 轮询异常: %s", e, exc_info=True)
                with self._lock:
                    self._last_error = f"轮询异常：{e}"
            dt = time.monotonic() - t0
            if dt < period:
                self._stop_evt.wait(period - dt)

    def _poll_once(self) -> None:
        """
        兜底轮询。推送活着的时候这里**不出网**：数据面已经是服务端推进来的，再每 50 ms 拉一次
        只会和推送抢同一份 C++ 快照（还多出一次 HTTP 往返）。
        留着这条线程的理由是推送会安静地坏（对端半开、中间设备丢连接、订阅配额被占满 —— 全都
        表现为"再也不来数据，但也不报错"），而闭环最怕的就是停在旧位姿上看起来却像跟得好。
        """
        with self._lock:
            if not self._active:
                return
            if self._push_live_locked():
                return
        snap = self._fetch_snapshot()
        if "error" in snap:
            with self._lock:
                self._last_error = snap["error"]
            self._emit()      # 错误态不在 33 Hz 稳态里，得由这里主动推一次让页面看见
            return
        self._ingest(snap)

    def _on_push_snapshot(self, snap: dict) -> None:
        """推送回调（在 pose_stream 自己的线程里被调）。只在跟随中消费，否则丢掉不污染缓存。
        新鲜度由 PoseStream 自己按事件打点（`last_event_age`），这里不再另存一份时钟 ——
        两份"最后一次收到数据是什么时候"迟早会互相矛盾。"""
        with self._lock:
            if not self._active:
                return
        self._ingest(snap)

    def _push_live_locked(self) -> bool:
        """
        调用方必须已持 _lock。判据是"开关开着 + 有流对象 + 最近一帧不旧"三条。
        故意**不**再看订阅线程是否活着：读者线程除了 `stop()` 之外不会退出，而 `stop()` 同时
        就把 `_active` 清了 —— 那条判据落不到任何真实场景上，加上只会让人以为它在防什么。
        """
        if not self._push_enabled or self._push is None:
            return False
        age = self._push.last_event_age()
        return age is not None and age <= self._push_stale_s

    def _data_plane_locked(self) -> dict:
        """状态里如实写明数据面此刻走的是哪条路，以及为什么。"""
        live = self._push_live_locked()
        out = {"mode": ("push" if live else "poll"),
               "push_enabled": self._push_enabled,
               "stale_after_ms": int(self._push_stale_s * 1000)}
        if not self._push_enabled:
            # 这条判据必须在最前面：开关关着时"线程没跑/一帧没有"都只是它的下游症状，
            # 而运维看到"线程未运行"会去查线程 —— 查不到任何东西。
            out["reason"] = "推送开关未开（follow.arm.push=false）"
        elif self._push is None:
            # 订阅还没起过（页面还没点"启动"）。reason 必须**永远在**：字段跟着调用历史出现或
            # 消失，页面就得写两套渲染分支去猜，而猜错的那套正好是推送坏掉的时候。
            out["reason"] = "订阅未启动（点启动后才连）"
        else:
            st = self._push.stats()
            out["push"] = st
            if not live:
                # 退回轮询必须说清原因，否则"推送没生效"会被读成"推送一直是好的、只是臂没动"。
                # 先说"多久没数据"再说"线程没了"：前者是这一行存在的理由（运维据此去看服务端），
                # 后者只是它的下游事实之一，单独当结论会把人支使去查一条本来该停的线程。
                age_s = st.get("last_event_age_s")
                alive = bool(st.get("running"))
                if age_s is None:
                    out["reason"] = "推送尚无一帧" if alive else "推送尚无一帧（订阅线程未起）"
                else:
                    out["reason"] = (f"推送已 {age_s:.1f}s 无数据（>{out['stale_after_ms']}ms）"
                                     "，退回轮询" + ("" if alive else "；订阅线程已退出"))
        return out

    def _ingest(self, snap: dict) -> None:
        """
        一条快照 → 一个臂目标。**推送和轮询共用这一条**：两条路径各自实现"拿到快照该干什么"，
        迟早会对同一帧解出不同的结果，而那比延迟更难查。
        """
        frames = int(snap.get("frames") or 0)
        usable = bool(snap.get("enabled")) and bool(snap.get("has_pose")) and snap.get("status") == "ok"
        with self._lock:
            self._snapshot = snap
            new_frames = frames != self._last_frames
            baseline = None if self._baseline_q is None else self._baseline_q.copy()
            nearest = None if self._target_q is None else self._target_q.copy()
            ready = self._R_cb is not None

        # 同一批帧只解一次：C++ 的 frames 只在真正解算过一帧时才前进，用它当去重键最省。
        # 稳态广播归 33 Hz 发射线程：这里不再逐轮 _emit，否则去重逻辑会把稳态节奏压掉。
        if not (new_frames and usable and baseline is not None and ready):
            return

        best, _T_target, reason = self._solve(snap, baseline, nearest)
        with self._lock:
            self._last_frames = frames
            self._ik_failed = best is None
            self._last_error = reason
            if best is not None:
                self._target_q = best
                # keyframe 进发射链：平滑器会按限速从当前输出位走过去。
                self._kf_queue.put_nowait(best.copy())
                self._kf_pushed += 1
            elif reason:
                # 失败**保持上一目标**：不夹位、不缩增量、不跳去 home。"增量一致"的契约
                # 不能被一次静默的截断破坏 —— 宁可臂停住，也不要它朝一个没测到的方向走。
                logger.warning("follow: 本帧未采用（%s），保持上一目标", reason)

    def _solve(self, snap: dict, baseline: np.ndarray,
               nearest: Optional[np.ndarray]) -> Tuple[Optional[np.ndarray], Any, str]:
        kin = self._get_kin()
        if kin is None or self._R_cb is None:
            return None, np.eye(4), "kinematics 或轴映射不可用"
        with self._kin_lock:
            return joints_to_target(kin, self._R_cb, snap.get("delta_r") or np.eye(3),
                                    snap.get("delta_t_m") or [0.0, 0.0, 0.0], baseline, nearest)

    def _get_kin(self):
        with self._kin_lock:
            if self._kin is None:
                try:
                    from core.hardware.robot.cr5_kinematics import CR5Kinematics
                    self._kin = CR5Kinematics(backend="auto")
                except Exception as e:
                    logger.error("CR5 运动学初始化失败: %s", e)
                    return None
            return self._kin

    # -------------------------------------------------------------- 33 Hz 发射
    def _emit_loop(self) -> None:
        """
        发射线程：每个 tick 让平滑器前进 dt，把输出的关节角原样广播。
        页面上的仿真臂拿到的是限速 + 余弦缓动后的稠密流，不是 20 Hz 的 keyframe 台阶。
        """
        period = 1.0 / self._arm["emit_hz"]
        last = time.monotonic()
        win_start = last
        win_ticks = 0
        while not self._stop_evt.is_set():
            now = time.monotonic()
            dt = now - last
            last = now
            try:
                self._emit_tick(dt)
                win_ticks += 1
            except Exception as e:                          # 发射线程不许因一次异常就死掉
                logger.error("follow 发射异常: %s", e, exc_info=True)
            # 发射率是这条线程存在的全部意义，5 s 一报，实际偏离目标一眼可见。
            if now - win_start >= 5.0:
                with self._lock:
                    kf = self._kf_pushed
                    self._kf_pushed = 0
                logger.info("follow 发射：实际 %.1f Hz（目标 %d Hz），keyframes %d 个/窗，平滑器 %s",
                            win_ticks / (now - win_start), self._arm["emit_hz"], kf,
                            "插值中" if self._smoother.moving else "钉位")
                win_start = now
                win_ticks = 0
            nxt = last + period - time.monotonic()
            if nxt > 0.0:
                self._stop_evt.wait(nxt)

    def _emit_tick(self, dt: float) -> None:
        # 抽干 keyframe 通道：积压多个时只有最新有意义（跟随是"去现在"不是"走历史"）。
        q_kf: Optional[np.ndarray] = None
        while True:
            try:
                q_kf = self._kf_queue.get_nowait()
            except queue.Empty:
                break
        if q_kf is not None:
            self._smoother.push_target(q_kf)
        q = self._smoother.step(dt)
        if q is None:
            return
        joints = [round(float(v), 4) for v in np.degrees(q)]
        # 钉位/静止时值不变：前端本来就按值去重，这里提前跳过，省掉每拍一次带锁的 FK
        # （它与轮询线程的 IK 共用 _kin_lock，33 Hz 空转会和 20 Hz 解算互相挤）。
        if joints == self._last_broadcast_joints:
            return
        self._last_broadcast_joints = joints
        self._broadcast(joints)

    # ---------------------------------------------------------------- 广播
    def register_ws_callback(self, callback: Callable) -> None:
        with self._lock:
            if callback not in self._ws_callbacks:
                self._ws_callbacks.append(callback)

    def unregister_ws_callback(self, callback: Callable) -> None:
        with self._lock:
            if callback in self._ws_callbacks:
                self._ws_callbacks.remove(callback)

    def _broadcast(self, joints_deg: Optional[List[float]]) -> None:
        """组装载荷推给 WS。发射线程 33 Hz 走这里；控制事件（启停/错误）也直接走。"""
        payload = self._state_payload()
        pose = self._pose_of(np.radians(np.asarray(joints_deg))) if joints_deg is not None else None
        with self._lock:
            self._emit_q_deg = joints_deg
            callbacks = list(self._ws_callbacks)
        data = dict(payload)
        data["joints_deg"] = joints_deg
        data["target_pose"] = pose
        for cb in callbacks:
            try:
                cb({"type": "follow_state", "data": data})
            except Exception as e:                                # 一个坏客户端不能拖垮生产者
                logger.error("follow WS 回调失败: %s", e)

    def _emit(self, force: bool = False) -> None:
        # 只给控制事件用（启动/调零/停止/错误）。稳态节奏归 _emit_loop，不走这里的去重。
        payload = self._state_payload()
        snap = payload["follow"] or {}
        with self._lock:
            joints = self._emit_q_deg or payload["arm_target_deg"]
        key = (int(snap.get("frames") or 0), str(snap.get("status")), payload["active"],
               payload["ik_failed"], None if joints is None else tuple(joints))
        with self._lock:
            if not force and key == self._last_emit_key:
                return
            self._last_emit_key = key
        self._broadcast(joints)

    def _pose_of(self, q_rad: Optional[np.ndarray]) -> Optional[List[float]]:
        if q_rad is None:
            return None
        kin = self._get_kin()
        if kin is None:
            return None
        try:
            with self._kin_lock:
                return pose_ctrl_from_target(kin, q_rad)
        except Exception as e:
            logger.warning("目标位姿反算失败: %s", e)
            return None

    def _target_pose_deg(self) -> Optional[List[float]]:
        with self._lock:
            q = None if self._target_q is None else self._target_q.copy()
        return self._pose_of(q)


follow_service = FollowService()
