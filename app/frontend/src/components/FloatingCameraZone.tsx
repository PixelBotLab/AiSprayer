import React, { useState, useEffect, useRef, useCallback } from 'react';
import { Camera, Maximize2, Minimize2, Crosshair, GripHorizontal, X, Zap, Target } from 'lucide-react';
import mpegts from 'mpegts.js';
import { API_BASE, WS_BASE } from '../config';
import { Tooltip } from './common/Tooltip';

interface FloatingCameraZoneProps {
  onClose?: () => void;
}

interface RobotLiveState {
  connected: boolean;
  is_moving: boolean;
}

interface AimNotice {
  kind: 'info' | 'ok' | 'error';
  text: string;
}

// 后端 plan() 选中候选族的英文说明 (两族工具指向都 = 视线方向, 只差指尖站在视线哪儿)
const STRATEGY_TEXT = {
  'on-ray': 'nozzle put on the clicked sight line, beam collinear with it, so the hit does not depend on depth',
  swing: 'nozzle kept where it was and the beam swung onto the clicked point, so the hit is locked to one depth',
} as const;

const FloatingCameraZone: React.FC<FloatingCameraZoneProps> = ({ onClose }) => {
  const [resolution, setResolution] = useState<{ width: number; height: number } | null>(null);
  const [isStreaming, setIsStreaming] = useState(false);

  // ── Live aim: 在本实时画面上点一下, 就把机械臂工具轴线 (虚拟光束) 指到那个像素 ──
  const [aimMode, setAimMode] = useState(false);
  const [robotState, setRobotState] = useState<RobotLiveState | null>(null);
  // 内参分辨率: 视频流分辨率可以与它不同, 点击位置按归一化坐标在两者之间换算
  const [intrinsics, setIntrinsics] = useState<{ width: number; height: number } | null>(null);
  const [aiming, setAiming] = useState(false);
  const [notice, setNotice] = useState<AimNotice | null>(null);
  // 点击标记存归一化坐标: 窗口拖拽/缩放后标记仍跟着同一个像素走
  const [aimMark, setAimMark] = useState<{ x: number; y: number } | null>(null);
  // ── Mark the laser cross centre by eye (bypasses the fragile detector): after an aim lands and the
  //    laser is on, the operator clicks the cross centre; the backend back-projects it through the
  //    current (post-move) camera pose and reports the true miss versus the point that was aimed at.
  const [markMode, setMarkMode] = useState(false);
  const [canMark, setCanMark] = useState(false);
  const [marking, setMarking] = useState(false);
  const [markMark, setMarkMark] = useState<{ x: number; y: number } | null>(null);
  // object-contain 下真实画面的矩形 (剩下的是 letterbox 黑边, 不是相机成像, 不可点击)
  const [contentBox, setContentBox] = useState({ left: 0, top: 0, width: 0, height: 0 });

  // Show a notice AND mirror its exact text into the backend verification log, so the operator never
  // has to copy-paste what the interface shows ("cross center at pixel…", "skipped (N blobs…)", the
  // mark-cross result line…). Fire-and-forget: a lost log write must never block or break the UI.
  const notify = useCallback((kind: AimNotice['kind'], text: string) => {
    setNotice({ kind, text });
    fetch(`${API_BASE}/api/robot/aim_ui_log`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ level: kind, message: text })
    }).catch(() => {
      // logging mirror is best-effort only
    });
  }, []);

  const videoRef = useRef<HTMLVideoElement | null>(null);
  const playerRef = useRef<mpegts.Player | null>(null);
  const reconnectTimerRef = useRef<number | null>(null);
  // 卡死看门狗：MSE/FLV 编码器被 CPU 抢走时，画面停在最后一帧但 socket 不断开，
  // mpegts 不会抛 ERROR——故必须自己监控 currentTime 是否还在前进。但不能太敏感：
  // Mac 软编码、尤其后端 pipeline 短暂软重启时，画面偷几秒又恢复是常态，一旦偷
  // 得太快就整路 teardown+重连，反而把一次微卡顿放大成“每隔几秒自动重启”的闪烁风暴。
  // 因此分两档：先轻量唤一下 play()（很多“冻结”只是 autoPlay 被浏览器策略打断），
  // 只有连续长时间（_STALL_RECONNECT_MS）确认不前进才真正重建流。
  const lastVideoTimeRef = useRef<number>(-1);
  const lastFrameAdvanceMsRef = useRef<number>(Date.now());
  const stallWatchdogRef = useRef<number | null>(null);

  // Floating window state
  const [position, setPosition] = useState({ x: 24, y: 24 });
  const [size, setSize] = useState({ w: 380, h: 280 });
  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  const [isMaximized, setIsMaximized] = useState(false);

  const dragRef = useRef({ startX: 0, startY: 0, initialX: 0, initialY: 0 });
  const resizeRef = useRef({ startX: 0, startY: 0, initialW: 0, initialH: 0 });

  // Stream Connection Setup (Zero-CPU RK3588 MPP Hardware Accelerated via MSE / FLV)
  const startStream = useCallback(() => {
    if (!videoRef.current || !mpegts.isSupported()) {
      console.warn('mpegts.js is not supported on this browser');
      return;
    }

    if (playerRef.current) {
      try {
        playerRef.current.destroy();
      } catch {
        // ignore
      }
      playerRef.current = null;
    }

    try {
      const flvUrl = `${API_BASE}/api/camera/flv`;
      const player = mpegts.createPlayer(
        {
          type: 'flv',
          isLive: true,
          url: flvUrl
        },
        {
          enableWorker: true,
          enableStashBuffer: false,
          stashInitialSize: 128,
          liveBufferLatencyChasing: true,
          liveBufferLatencyMaxLatency: 0.8,
          liveBufferLatencyMinRemain: 0.1,
          autoCleanupSourceBuffer: true
        }
      );

      playerRef.current = player;
      player.attachMediaElement(videoRef.current);
      player.load();
      // 新一路流：重置看门狗基线，从此刻起重新判定前进/停滞。
      lastVideoTimeRef.current = -1;
      lastFrameAdvanceMsRef.current = Date.now();
      const playRes = player.play();
      if (playRes && typeof (playRes as Promise<void>).catch === 'function') {
        (playRes as Promise<void>).catch((e: unknown) => {
          console.warn('Auto play blocked or failed:', e);
        });
      }

      player.on(mpegts.Events.ERROR, (errorType, errorDetail, errorInfo) => {
        console.warn('Stream player error:', errorType, errorDetail, errorInfo);
        setIsStreaming(false);
        if (!reconnectTimerRef.current) {
          reconnectTimerRef.current = window.setTimeout(() => {
            reconnectTimerRef.current = null;
            startStream();
          }, 2000);
        }
      });
    } catch (err) {
      console.error('Failed to initialize mpegts stream player:', err);
      if (!reconnectTimerRef.current) {
        reconnectTimerRef.current = window.setTimeout(() => {
          reconnectTimerRef.current = null;
          startStream();
        }, 2000);
      }
    }
  }, []);

  useEffect(() => {
    startStream();

    // 看门狗：每 1s 检查画面 currentTime 是否仍在前进。真在出帧就刷新基线。
    // 停滞分两档处理：短暂偷住（>= _STALL_NUDGE_MS）先轻量唤一下 play()，避免把
    // 一次可自愈的摸黑当成断流；只有连续长时间（>= _STALL_RECONNECT_MS）确认不前进，
    // 且曾正常播过（lastVideoTime>0），才当作真断流强制重建流。
    const STALL_NUDGE_MS = 2500;
    const STALL_RECONNECT_MS = 8000;
    stallWatchdogRef.current = window.setInterval(() => {
      const v = videoRef.current;
      if (!v) return;
      const t = v.currentTime;
      if (t && t !== lastVideoTimeRef.current) {
        lastVideoTimeRef.current = t;
        lastFrameAdvanceMsRef.current = Date.now();
        return;
      }
      // 还没出过第一帧（刚建流/后端真重启中）不催重连，交给 onLoadedMetadata / ERROR 事件。
      if (lastVideoTimeRef.current <= 0 || reconnectTimerRef.current) return;
      const stalledFor = Date.now() - lastFrameAdvanceMsRef.current;
      if (stalledFor >= STALL_RECONNECT_MS) {
        console.warn(`Live stream stalled ${stalledFor}ms with no frames; reconnecting...`);
        setIsStreaming(false);
        reconnectTimerRef.current = window.setTimeout(() => {
          reconnectTimerRef.current = null;
          startStream();
        }, 1000);
      } else if (stalledFor >= STALL_NUDGE_MS && v.paused && v.readyState >= 2) {
        // 画面已缓冲但播放器被挂起：唤一下就行，不必拆流。
        (v.play() as Promise<void> | undefined)?.catch?.(() => {
          // 唤不动则交给后面的重连档
        });
      }
    }, 1000);

    return () => {
      if (stallWatchdogRef.current) {
        clearInterval(stallWatchdogRef.current);
        stallWatchdogRef.current = null;
      }
      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current);
        reconnectTimerRef.current = null;
      }
      if (playerRef.current) {
        try {
          playerRef.current.destroy();
        } catch {
          // ignore
        }
        playerRef.current = null;
      }
    };
  }, [startStream]);

  // Handle Dragging
  const handleDragStart = (e: React.MouseEvent) => {
    if (isMaximized) return;
    setIsDragging(true);
    dragRef.current = {
      startX: e.clientX,
      startY: e.clientY,
      initialX: position.x,
      initialY: position.y
    };
  };

  // Handle Resizing
  const handleResizeStart = (e: React.MouseEvent) => {
    if (isMaximized) return;
    e.stopPropagation();
    setIsResizing(true);
    resizeRef.current = {
      startX: e.clientX,
      startY: e.clientY,
      initialW: size.w,
      initialH: size.h
    };
  };

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      if (isDragging) {
        const dx = e.clientX - dragRef.current.startX;
        const dy = e.clientY - dragRef.current.startY;
        setPosition({
          x: Math.max(0, dragRef.current.initialX + dx),
          y: Math.max(0, dragRef.current.initialY + dy)
        });
      } else if (isResizing) {
        const dw = e.clientX - resizeRef.current.startX;
        const dh = e.clientY - resizeRef.current.startY;
        setSize({
          w: Math.max(250, resizeRef.current.initialW + dw),
          h: Math.max(180, resizeRef.current.initialH + dh)
        });
      }
    };

    const handleMouseUp = () => {
      setIsDragging(false);
      setIsResizing(false);
    };

    if (isDragging || isResizing) {
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
    }

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging, isResizing]);

  // ── Live aim: 几何换算、状态轮询与动作下发 ────────────────────────────

  // 量出视频元素里“真实画面”那块矩形 (object-contain 居中+等比缩放, 其余是 letterbox 黑边)。
  // 点击层直接套在这个矩形上, 所以坐标换算不需要再考虑黑边。
  const measureContentBox = useCallback(() => {
    const vid = videoRef.current;
    if (!vid || !vid.videoWidth || !vid.videoHeight) return;
    const scale = Math.min(vid.clientWidth / vid.videoWidth, vid.clientHeight / vid.videoHeight);
    const w = vid.videoWidth * scale;
    const h = vid.videoHeight * scale;
    setContentBox({ left: (vid.clientWidth - w) / 2, top: (vid.clientHeight - h) / 2, width: w, height: h });
  }, []);

  useEffect(() => {
    measureContentBox();
    // 窗口尺寸动画过后再量一次, 避开拖拽/最大化途中拿到旧的 clientWidth
    const id = window.setTimeout(measureContentBox, 150);
    return () => window.clearTimeout(id);
  }, [measureContentBox, size.w, size.h, isMaximized, resolution, isStreaming]);

  // 只在 aim 模式开启时拉内参与订阅臂状态 (平时不给后端捣流量)
  // 臂状态走 /api/robot/ws 广播 (与机械臂页、交互页同一通道): 秒级轮询 /api/robot/state
  // 每次都是四次控制器往返, 还会把 access 日志刷满屏幕, 完全没必要。
  useEffect(() => {
    if (!aimMode) return;
    let cancelled = false;
    let ws: WebSocket | null = null;
    let reconnectTimer: number | null = null;

    const loadIntrinsics = async () => {
      try {
        const res = await fetch(`${API_BASE}/api/camera/intrinsics`);
        const data = await res.json();
        if (cancelled) return;
        if (!res.ok || !data.width || !data.height) {
          setIntrinsics(null);
          setNotice({ kind: 'error', text: data.detail || 'Camera intrinsics are unavailable (camera offline).' });
          return;
        }
        setIntrinsics({ width: Number(data.width), height: Number(data.height) });
      } catch {
        if (!cancelled) {
          setIntrinsics(null);
          setNotice({ kind: 'error', text: 'Failed to query camera intrinsics from the backend.' });
        }
      }
    };

    const connect = () => {
      if (cancelled) return;
      try {
        ws = new WebSocket(`${WS_BASE}/api/robot/ws`);
      } catch {
        return;
      }
      ws.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data);
          // status 就是控制器的 running_status (0=Idle, 1=Moving): 与后端互锁同一个真理源
          if (msg.type === 'robot_state' && msg.data) {
            setRobotState({ connected: !!msg.data.connected, is_moving: Number(msg.data.status ?? 0) !== 0 });
          }
        } catch {
          // 单条消息解不开了忽略, 下一帧广播自然恢复
        }
      };
      ws.onclose = () => {
        if (!cancelled) reconnectTimer = window.setTimeout(connect, 2000);
      };
    };

    // 一次性补种: 机械臂从未连过时后端不会推任何广播, 不补这一发就会永远停在
    // "Reading robot state..." 把按钮锁死。只在进 aim 模式时发一次, 后续全走上面的 WS。
    const seedRobotState = async () => {
      try {
        const res = await fetch(`${API_BASE}/api/robot/state`);
        const data = await res.json();
        if (!cancelled) setRobotState({ connected: !!data.connected, is_moving: !!data.is_moving });
      } catch {
        if (!cancelled) setRobotState({ connected: false, is_moving: false });
      }
    };

    loadIntrinsics();
    seedRobotState();
    connect();

    return () => {
      cancelled = true;
      if (reconnectTimer !== null) window.clearTimeout(reconnectTimer);
      if (ws) {
        ws.onclose = null;
        ws.close();
      }
    };
  }, [aimMode]);

  // 状态互锁: 未连接 / 臂在运动中 / 没内参, 都不允许再提交动作 (与 API 层双重拦截)
  const aimLockReason = !robotState
    ? 'Reading robot state...'
    : !robotState.connected
      ? 'Robot is not connected: aiming is unavailable.'
      : robotState.is_moving
        ? 'Robot is moving: all action buttons are locked.'
        : !intrinsics
          ? 'Camera intrinsics unavailable: a pixel cannot be converted to a ray.'
          : null;

  // Mark the cross centre by eye: back-projects the marked pixel through the current (post-move) camera
  // pose into the base frame and reports the true miss versus the point that was aimed at.
  const handleMarkClick = async (e: React.MouseEvent<HTMLDivElement>) => {
    if (marking || aimLockReason || !intrinsics) return;
    const rect = e.currentTarget.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    const nx = (e.clientX - rect.left) / rect.width;
    const ny = (e.clientY - rect.top) / rect.height;
    if (nx < 0 || nx > 1 || ny < 0 || ny > 1) return;

    const payload = { u_px: nx * intrinsics.width, v_px: ny * intrinsics.height };
    setMarkMark({ x: nx, y: ny });
    setMarking(true);
    notify(
      'info',
      `marking the cross centre at pixel (${payload.u_px.toFixed(0)}, ${payload.v_px.toFixed(0)}): `
      + 'back-projecting through the current camera pose and comparing with the aimed point...'
    );

    try {
      const res = await fetch(`${API_BASE}/api/robot/aim_mark_cross`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      const data = await res.json();
      if (!res.ok) {
        notify('error', data.detail || 'Cross-centre verification failed.');
        return;
      }
      const fmt = (a: number[]) => (Array.isArray(a) ? `[${a.map((v) => Number(v).toFixed(0)).join(', ')}]` : 'n/a');
      const num = (v: number | null, unit: string) => (typeof v === 'number' ? `${v}${unit}` : 'n/a');
      const text = Array.isArray(data.cross_point_mm)
        ? `cross centre ${fmt(data.cross_point_mm)} mm at ${num(data.cross_depth_mm, ' mm')} surface; `
          + `${data.sight_angle_deg} deg off the clicked sight line; miss ${num(data.miss_vs_target_mm, ' mm')} off the target `
          + `(lateral ${num(data.miss_lateral_mm, ' mm')}, depth ${num(data.miss_depth_mm, ' mm')}); `
          + `beam axis ${num(data.beam_axis_gap_mm, ' mm')} off the modelled beam`
          + (typeof data.beam_axis_angle_deg === 'number' ? ` (= ${data.beam_axis_angle_deg} deg of laser-vs-tool+Z tilt)` : '')
        : `cross centre pixel (${Number(data.marked_pixel[0]).toFixed(0)}, ${Number(data.marked_pixel[1]).toFixed(0)}): `
          + `${data.sight_angle_deg} deg off the clicked sight line; ${data.note || 'no 3D miss measured'}`;
      notify('ok', text);
    } catch {
      notify('error', 'The cross-mark request did not reach the robot backend.');
    } finally {
      setMarking(false);
      setMarkMode(false);
    }
  };

  const handleAimClick = async (e: React.MouseEvent<HTMLDivElement>) => {
    if (markMode) return handleMarkClick(e);
    if (aiming || aimLockReason || !intrinsics) return;
    const rect = e.currentTarget.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    const nx = (e.clientX - rect.left) / rect.width;
    const ny = (e.clientY - rect.top) / rect.height;
    if (nx < 0 || nx > 1 || ny < 0 || ny > 1) return;

    // 归一化坐标 -> 内参像素 (后端 pixel_ray_to_base 按内参尺寸校验边界)
    const payload = { u_px: nx * intrinsics.width, v_px: ny * intrinsics.height };
    const pixelLabel = `pixel (${payload.u_px.toFixed(0)}, ${payload.v_px.toFixed(0)}) of ${intrinsics.width}x${intrinsics.height}`;
    setAimMark({ x: nx, y: ny });
    setAiming(true);
    // a new aim invalidates the previous verification target until this one lands
    setCanMark(false);
    notify('info', `${pixelLabel}: putting the tool axis on this sight line...`);

    try {
      const res = await fetch(`${API_BASE}/api/robot/aim_at_pixel`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      const data = await res.json();
      if (!res.ok) {
        notify('error', data.detail || 'Live aiming failed.');
        return;
      }
      // 坐标必须显式回显: 目标点的基座坐标是判断“光束打到哪儿”的唯一可归因依据
      const hit = Array.isArray(data.pixel)
        ? `pixel (${Number(data.pixel[0]).toFixed(0)}, ${Number(data.pixel[1]).toFixed(0)})`
        : pixelLabel;
      const target = Array.isArray(data.target_point_mm)
        ? `[${data.target_point_mm.map((v: number) => Number(v).toFixed(0)).join(', ')}] mm`
        : '';
      const how = STRATEGY_TEXT[data.strategy as keyof typeof STRATEGY_TEXT] ?? 'aimed';
      // 深度来源必须回显: on-ray 与深度无关, 但 swing 族的落点就定在这个深度上,
      // depth_source=config 意味着深度没读到、用的是猜测靶距 (现场要知道这一枪有多可信)。
      const movedText =
        `${hit}: beam put through ${target} (depth ${data.display_depth_mm} mm, ${data.depth_source}), `
        + `nozzle ${data.along_ray_mm} mm in front of the lens, `
        + `${data.planned_offset_mm} mm off that point (${how})`;
      // 到位且 (尽力) 亮了激光 -> 解锁"人工点十字中心"按钮 (人眼比碎块检测器可靠得多)。
      setCanMark(true);
      // 读不到控制器反馈时不能显示 "beam error null deg": 降级为 info 并说明未测到
      if (data.beam_angle_deg === null || data.beam_angle_deg === undefined) {
        notify('info', `${movedText}; position feedback is unreadable, so the beam error was not measured.`);
      } else {
        // spot_error_mm = 被点目标点离实际光束线多远 (两族按规划都应为 0, 非零就是没走到或标定偏)
        notify(
          'ok',
          `${movedText}, spot ${data.spot_error_mm} mm off the clicked point (${data.beam_angle_deg} deg off the planned beam), laser DO${data.spray_do_index} ${data.spray_do_state}. `
          + 'Now click the cross centre (Mark-cross button) to measure the true miss by eye.'
        );
      }
    } catch {
      notify('error', 'The aiming request did not reach the robot backend.');
    } finally {
      setAiming(false);
    }
  };

  const containerClasses = isMaximized
    ? 'fixed inset-4 md:inset-6 z-[100] bg-slate-900/98 rounded-xl border border-slate-700/80 shadow-2xl overflow-hidden flex flex-col transition-all duration-200 backdrop-blur-xl animate-in zoom-in-95 duration-150'
    : 'absolute bg-slate-900/95 rounded-xl border border-slate-700/90 shadow-2xl overflow-hidden flex flex-col z-50 transition-shadow duration-200 backdrop-blur-md animate-in fade-in duration-150';

  const containerStyles = isMaximized
    ? {}
    : {
        left: position.x,
        top: position.y,
        width: size.w,
        height: size.h,
        boxShadow: isDragging ? '0 25px 50px -12px rgba(0, 0, 0, 0.8)' : undefined
      };

  return (
    <div className={containerClasses} style={containerStyles}>
      {/* Draggable Header */}
      <div
        className={`shrink-0 px-3.5 py-2 flex justify-between items-center bg-gradient-to-b from-slate-800 to-slate-900 border-b border-slate-700 select-none ${
          isMaximized ? '' : 'cursor-move'
        }`}
        onMouseDown={handleDragStart}
      >
        <div className="flex items-center gap-2">
          <Camera size={15} className="text-blue-400" />
          <h2 className="font-medium text-slate-100 text-xs drop-shadow-md">
            Live Stream {isMaximized ? '(Fullscreen)' : ''}
          </h2>

          {/* Stream Mode Badge */}
          <span className="flex items-center gap-0.5 px-1.5 py-0.5 bg-emerald-500/20 text-emerald-400 border border-emerald-500/30 rounded text-[9px] font-mono tracking-tight font-semibold">
            <Zap size={9} className="text-emerald-400 fill-emerald-400" />
            MPP HW
          </span>

          <span className="relative flex h-1.5 w-1.5 ml-1">
            {isStreaming ? (
              <>
                <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75"></span>
                <span className="relative inline-flex rounded-full h-1.5 w-1.5 bg-emerald-500 shadow-[0_0_6px_rgba(16,185,129,0.7)]"></span>
              </>
            ) : (
              <span className="relative inline-flex rounded-full h-1.5 w-1.5 bg-slate-600"></span>
            )}
          </span>
        </div>

        <div className="flex items-center gap-1.5 text-slate-400">
          {/* Aim mode toggle (single-pose tool-tip pointing, no capture / no rebuild) */}
          <div className="relative group flex items-center">
            <button
              onClick={() => {
                setAimMode((prev) => !prev);
                setNotice(null);
                setAimMark(null);
                setMarkMode(false);
                setMarkMark(null);
                setCanMark(false);
              }}
              className={`transition-colors rounded p-1 ${
                aimMode ? 'bg-amber-500/25 text-amber-300' : 'hover:bg-slate-700 hover:text-white'
              }`}
            >
              <Target size={14} />
            </button>
            <Tooltip
              text={
                aimMode
                  ? 'Exit aim mode'
                  : 'Aim mode: click a point in the live image and the tool axis (a virtual laser beam) is put on that line of sight'
              }
              side="bottom"
              align="end"
              multiline
              maxWidthClass="max-w-[240px]"
            />
          </div>

          {/* Mark-cross toggle: enabled only after an aim has landed, so the operator can point at the */}
          {/* laser cross they actually see and get the true miss, bypassing the fragile blob detector.  */}
          {aimMode && (
            <div className="relative group flex items-center">
              <button
                onClick={() => canMark && !aimLockReason && setMarkMode((prev) => !prev)}
                disabled={!canMark || !!aimLockReason}
                className={`transition-colors rounded p-1 ${
                  markMode
                    ? 'bg-cyan-500/30 text-cyan-300'
                    : canMark && !aimLockReason
                      ? 'hover:bg-slate-700 hover:text-white text-slate-300'
                      : 'opacity-40 cursor-not-allowed'
                }`}
              >
                <Crosshair size={14} />
              </button>
              <Tooltip
                text={
                  !canMark
                    ? 'Aim at a point first; once the arm arrives and the laser is on, click the cross centre here'
                    : markMode
                      ? 'Now click the centre of the laser cross in the image to measure the true miss'
                      : 'Mark the laser cross centre by eye: measures the real hit versus the point that was aimed at (back-projected through the eye-in-hand pose)'
                }
                side="bottom"
                align="end"
                multiline
                maxWidthClass="max-w-[240px]"
              />
            </div>
          )}

          <div className="relative group flex items-center">
            <button
              onClick={() => setIsMaximized(!isMaximized)}
              className={`transition-colors rounded p-1 hover:bg-slate-700 hover:text-white ${
                isMaximized ? 'bg-blue-600/30 text-blue-300' : ''
              }`}
            >
              {isMaximized ? <Minimize2 size={14} /> : <Maximize2 size={14} />}
            </button>
            <Tooltip text={isMaximized ? 'Exit Fullscreen' : 'Fullscreen'} side="bottom" align="end" />
          </div>

          {onClose && (
            <div className="relative group flex items-center">
              <button
                onClick={onClose}
                className="p-1 hover:bg-slate-700 hover:text-white rounded transition-colors"
              >
                <X size={14} />
              </button>
              <Tooltip
                text="Close Live Stream (Can reopen from Left Sidebar)"
                side="bottom"
                align="end"
                multiline
                maxWidthClass="max-w-[220px]"
              />
            </div>
          )}

          {!isMaximized && <GripHorizontal size={14} className="opacity-50" />}
        </div>
      </div>

      {/* Video Content */}
      <div className="flex-1 bg-black flex items-center justify-center relative overflow-hidden group">
        {/* Resolution Overlay */}
        {resolution && (
          <div className="absolute bottom-3 left-3 z-20 bg-slate-900/70 backdrop-blur-md px-2 py-1 rounded border border-slate-700/50 text-[10px] font-mono text-emerald-400 shadow pointer-events-none flex items-center gap-1.5">
            <span>
              {resolution.width}×{resolution.height}
            </span>
            <span className="text-slate-400 font-sans text-[9px]">H.264 HW</span>
          </div>
        )}

        {/* Hardware Accelerated WebRTC Video Element */}
        <video
          ref={videoRef}
          autoPlay
          playsInline
          muted
          className={`absolute inset-0 w-full h-full object-contain z-0 pointer-events-none transition-opacity duration-300 ${
            isStreaming ? 'opacity-100' : 'opacity-0'
          }`}
          onLoadedMetadata={(e) => {
            const vid = e.currentTarget;
            setResolution({
              width: vid.videoWidth,
              height: vid.videoHeight
            });
            setIsStreaming(true);
          }}
        />

        {/* Aim Click Layer - covers exactly the object-contain image area */}
        {aimMode && contentBox.width > 0 && contentBox.height > 0 && (
          <div
            className={`absolute z-20 border ${
              aimLockReason
                ? 'border-slate-600/50 cursor-not-allowed'
                : markMode
                  ? 'border-cyan-400/50 cursor-crosshair hover:bg-cyan-400/5'
                  : 'border-amber-400/40 cursor-crosshair hover:bg-amber-400/5'
            }`}
            style={{ left: contentBox.left, top: contentBox.top, width: contentBox.width, height: contentBox.height }}
            onClick={handleAimClick}
          >
            {aimMark && (
              <div
                className="absolute -translate-x-1/2 -translate-y-1/2 pointer-events-none"
                style={{ left: `${aimMark.x * 100}%`, top: `${aimMark.y * 100}%` }}
              >
                <Target size={18} strokeWidth={2} className="text-amber-400 drop-shadow-[0_0_4px_rgba(251,191,36,0.8)]" />
              </div>
            )}
            {markMark && (
              <div
                className="absolute -translate-x-1/2 -translate-y-1/2 pointer-events-none"
                style={{ left: `${markMark.x * 100}%`, top: `${markMark.y * 100}%` }}
              >
                <Crosshair size={20} strokeWidth={2} className="text-cyan-300 drop-shadow-[0_0_4px_rgba(34,211,238,0.8)]" />
              </div>
            )}
          </div>
        )}

        {/* Aim Status Pill - lock reason / in progress / result (English only) */}
        {aimMode && (
          <div className="absolute bottom-3 right-3 z-30 max-w-[70%] pointer-events-none">
            <div
              className={`px-2 py-1 rounded border text-[10px] shadow backdrop-blur-md break-words ${
                notice?.kind === 'error'
                  ? 'bg-rose-950/70 border-rose-500/40 text-rose-300'
                  : notice?.kind === 'ok'
                    ? 'bg-emerald-950/70 border-emerald-500/40 text-emerald-300'
                    : 'bg-slate-900/70 border-slate-700/60 text-slate-300'
              }`}
            >
              {notice?.text ?? aimLockReason ?? (markMode
                ? 'Click the centre of the laser cross you see to measure the true miss.'
                : 'Click a point in the image to put the tool axis on that sight line.')}
            </div>
          </div>
        )}

        {/* Camera Reticle Overlay */}
        <div className="absolute inset-0 border-[1px] border-blue-500/10 m-3 rounded pointer-events-none z-10">
          <div className="absolute top-0 left-0 w-6 h-6 border-t-2 border-l-2 border-blue-500/40 rounded-tl"></div>
          <div className="absolute top-0 right-0 w-6 h-6 border-t-2 border-r-2 border-blue-500/40 rounded-tr"></div>
          <div className="absolute bottom-0 left-0 w-6 h-6 border-b-2 border-l-2 border-blue-500/40 rounded-bl"></div>
          <div className="absolute bottom-0 right-0 w-6 h-6 border-b-2 border-r-2 border-blue-500/40 rounded-br"></div>
          <Crosshair
            className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 text-blue-500/20 w-10 h-10"
            strokeWidth={1}
          />
        </div>

        {!isStreaming && (
          <div className="text-slate-500 flex flex-col items-center z-0 pointer-events-none">
            <Camera size={36} strokeWidth={1} className="mb-2 opacity-20" />
            <p className="text-xs tracking-wide">Connecting WebRTC Hardware Stream...</p>
          </div>
        )}

        {/* Resize Handle */}
        {!isMaximized && (
          <div
            className="absolute bottom-0 right-0 w-5 h-5 cursor-se-resize flex items-end justify-end p-1 z-30 group"
            onMouseDown={handleResizeStart}
          >
            <div className="w-2.5 h-2.5 border-r-2 border-b-2 border-slate-500 group-hover:border-blue-400 transition-colors"></div>
          </div>
        )}
      </div>
    </div>
  );
};

export default FloatingCameraZone;

