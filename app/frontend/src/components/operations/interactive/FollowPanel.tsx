import React, { useCallback, useEffect, useRef, useState } from 'react';
import { Play, Crosshair, Square, Radio, RefreshCw } from 'lucide-react';
import { API_BASE, WS_BASE } from '../../../config';

/**
 * FollowPanel：文件列表底下那一排三个按钮 + 一行实时读数。
 *
 * 自包含：WS 自己连、自己重连，状态只从 `/api/follow/ws` 的第一帧和后续广播里来（后端在
 * accept 之后会立刻推一帧 `status()`，所以不需要再单独 GET 一次）。对外只有一个出口
 * `onFollowJoints(joints | null)` —— 非空就是"仿真臂该去的 URDF 关节角（度）"，null 就是
 * "我把臂交还给真机状态"。臂与相机**不需要同一个位姿**，要一致的是位移/旋转增量，那部分数学
 * 全在后端（apps/follow/mirror.py），这里只负责把它画出来。
 *
 * 三个按钮都是**等后端返回**的：使能要重启取流档位（640x480 + 硬件 D2C）、示教要收帧，最慢几秒。
 * 期间三个键一起禁掉（`pending`）—— 双击"启动"会打出两次 enable+teach，第二次会把第一次
 * 刚示教好的参考地图换掉。
 */

interface FollowSnapshot {
  enabled: boolean;
  connected: boolean;
  taught: boolean;
  has_pose: boolean;
  status: string;
  estimator: string;
  reason: string;
  // 档位切换进行中（重启取流）：此刻 enabled 还是旧值，状态栏会显示 "switching"。
  switching?: boolean;
  pose_mm: number[];
  pose_rpy_deg: number[];
  norm_t_mm: number;
  norm_r_deg: number;
  holding_last_pose: boolean;
  delta_r: number[][];
  delta_t_m: number[];
  // σ 在没有稠密解算时是 +inf，跨 JSON 边界只能是 null —— null 读作"没有估计值"，不是 0。
  sigma_t_mm: (number | null)[];
  sigma_r_deg: (number | null)[];
  gicp_inliers: number;
  inlier_ratio: number;
  cloud_points: number;
  compute_ms: number;
  fps: number;
  frames: number;
  dropped: number;
  rejected: number;
  smooth_used: number;
  map_hash: number;
  map_voxels: number;
  align: string;
  capture_width: number;
  capture_height: number;
  teach_capture_width: number;
  teach_capture_height: number;
}

interface FollowState {
  active: boolean;
  arm_mode: string;
  r_cb_source: string;
  r_cb_ready?: boolean;
  camera_service_reachable?: boolean;
  home_joints_deg?: number[];
  ik_failed: boolean;
  last_error: string;
  arm_baseline_deg: number[] | null;
  joints_deg: number[] | null;
  target_pose: number[] | null;
  follow: Partial<FollowSnapshot> & { error?: string };
}

interface FollowPanelProps {
  /** 轨迹回放正在驱动同一台仿真臂 —— 两边抢着写会让臂抖，所以互斥。 */
  isReplaying: boolean;
  onFollowJoints: (joints: number[] | null) => void;
}

const fmt = (v: number | null | undefined, digits = 1): string =>
  v === null || v === undefined || !Number.isFinite(v) ? '—' : v.toFixed(digits);

/** |Δt|：增量平移是米，读数要 mm（页面上 mm 才是这个量级该有的单位）。 */
const deltaTransMm = (snap?: Partial<FollowSnapshot>): number | null => {
  if (!snap || !snap.has_pose || !snap.delta_t_m || snap.delta_t_m.length < 3) return null;
  const [x, y, z] = snap.delta_t_m;
  return Math.hypot(x, y, z) * 1000.0;
};

/** |ΔR|：由 trace 反解转角，比先拆欧拉再取模稳（欧拉在 ±90° 会换等价分支）。 */
const deltaRotDeg = (snap?: Partial<FollowSnapshot>): number | null => {
  const dr = snap && snap.has_pose ? snap.delta_r : null;
  if (!dr || dr.length < 3) return null;
  const tr = dr[0][0] + dr[1][1] + dr[2][2];
  const cos = Math.min(1, Math.max(-1, (tr - 1) / 2));
  return (Math.acos(cos) * 180) / Math.PI;
};

export const FollowPanel: React.FC<FollowPanelProps> = ({ isReplaying, onFollowJoints }) => {
  const [state, setState] = useState<FollowState | null>(null);
  const [wsOpen, setWsOpen] = useState(false);
  const [pending, setPending] = useState<'start' | 'zero' | 'stop' | null>(null);
  const [error, setError] = useState('');

  const wsRef = useRef<WebSocket | null>(null);
  const timerRef = useRef<number | null>(null);
  // 父组件每帧都可能重建这个回调；用 ref 兜住，WS effect 才能只跑一次。
  const pushRef = useRef(onFollowJoints);
  pushRef.current = onFollowJoints;
  // 只在"关节角真的变了"时才往外推，避免每个广播帧都惊动一次 3D viewer。
  const lastJointsRef = useRef<string>('');

  useEffect(() => {
    let closed = false;

    const connect = () => {
      if (closed) return;
      let ws: WebSocket;
      try {
        ws = new WebSocket(`${WS_BASE}/api/follow/ws`);
      } catch {
        return;
      }
      wsRef.current = ws;
      ws.onopen = () => {
        setWsOpen(true);
        setError('');
      };
      ws.onmessage = (ev: MessageEvent) => {
        try {
          const msg = JSON.parse(ev.data as string);
          if (msg.type !== 'follow_state') return;
          const next: FollowState = msg.data;
          setState(next);
          const key = next.joints_deg ? next.joints_deg.join(',') : '';
          if (key !== lastJointsRef.current) {
            lastJointsRef.current = key;
            pushRef.current(next.joints_deg && next.joints_deg.length === 6 ? next.joints_deg : null);
          }
        } catch (err) {
          console.warn('follow_state 解析失败:', err);
        }
      };
      ws.onclose = () => {
        setWsOpen(false);
        wsRef.current = null;
        if (!closed) timerRef.current = window.setTimeout(connect, 2000);
      };
      ws.onerror = () => {
        setWsOpen(false);
      };
    };

    connect();
    return () => {
      closed = true;
      if (timerRef.current) window.clearTimeout(timerRef.current);
      wsRef.current?.close();
    };
  }, []);

  // 卸载时把臂交还给真机状态：否则 simJoints 会停在最后一帧跟随位姿上，看着像"卡住了"。
  useEffect(() => () => { pushRef.current(null); lastJointsRef.current = ''; }, []);

  const post = useCallback(async (path: 'start' | 'zero' | 'stop') => {
    setError('');
    try {
      const res = await fetch(`${API_BASE}/api/follow/${path}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        // 启动：让后端用 home 当基线（页面此刻并没有一个比 home 更可信的"当前位置"）。
        // 调零：把最近一次跟随关节角带回去当基线；没收到过就留空，由后端按优先级退档。
        body: JSON.stringify(path === 'zero' ? { joints_deg: state?.joints_deg ?? null } : {}),
      });
      const data = await res.json().catch(() => ({} as any));
      if (!res.ok) {
        const detail = typeof data.detail === 'string' ? data.detail : `HTTP ${res.status}`;
        setError(res.status === 503 ? `后端未就绪：${detail}` : detail);
        return;
      }
      if (data.data) setState(data.data as FollowState);
    } catch (err) {
      // 按钮点了没回音，必须在这儿留下话，否则用户只会觉得"这页面对后端没反应"。
      setError(`请求失败：${err instanceof Error ? err.message : String(err)}`);
    }
  }, [state?.joints_deg]);

  const run = useCallback(async (which: 'start' | 'zero' | 'stop') => {
    if (pending) return;
    setPending(which);
    await post(which);
    if (which === 'stop') {
      lastJointsRef.current = '';
      pushRef.current(null);
    }
    setPending(null);
  }, [pending, post]);

  const snap = state?.follow;
  const active = !!state?.active;
  const busy = pending !== null;
  const disabledStart = busy || active || isReplaying || !wsOpen;
  const disabledZero = busy || !active || isReplaying;
  const disabledStop = busy || (!active && !snap?.enabled);

  const tip = (text: string) => (
    <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
      <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
        {text}
      </div>
    </div>
  );

  const dtMm = deltaTransMm(snap);
  const dRdeg = deltaRotDeg(snap);
  const sigmaT = snap?.sigma_t_mm?.[2] ?? null;      // 沿光轴的重复性最关心，只报这一维
  const running = active && !!snap?.enabled;
  const heldByIk = !!state?.ik_failed;
  const holding = !!snap?.holding_last_pose || heldByIk;

  return (
    <div className="shrink-0 border-t border-slate-800 bg-slate-950/40 select-none">
      {/* 按钮排：与文件列表表头同一套尺寸与配色 */}
      <div className="h-9 px-2.5 flex justify-between items-center">
        <div className="flex items-center gap-1.5 text-[11px] text-slate-300 font-medium min-w-0">
          <Radio size={13} className={running ? 'text-emerald-400' : 'text-slate-500'} />
          <span>Follow</span>
          {!wsOpen && <span className="text-[9px] text-amber-400/80">后端未连接</span>}
          {state && state.arm_mode !== 'sim' && (
            <span className="text-[9px] text-rose-400/80">mode:{state.arm_mode}</span>
          )}
        </div>
        <div className="flex items-center gap-1">
          {/* ① 启动 */}
          <div className="relative group flex items-center justify-center">
            <button
              type="button"
              disabled={disabledStart}
              onClick={() => run('start')}
              className={`w-6 h-6 rounded flex items-center justify-center transition-all ${
                disabledStart
                  ? 'text-slate-600 cursor-not-allowed opacity-40'
                  : 'text-emerald-400 hover:bg-emerald-500/20 hover:text-emerald-300'
              }`}
            >
              {pending === 'start' ? <RefreshCw size={12} className="animate-spin text-emerald-400" /> : <Play size={11} />}
            </button>
            {tip(isReplaying ? '轨迹回放正在驱动仿真臂：先停止回放'
                : active ? '跟随已在运行'
                : '启动：仿真臂回 home + 以当前相机视角示教')}
          </div>

          {/* ② 调零 */}
          <div className="relative group flex items-center justify-center">
            <button
              type="button"
              disabled={disabledZero}
              onClick={() => run('zero')}
              className={`w-6 h-6 rounded flex items-center justify-center transition-all ${
                disabledZero
                  ? 'text-slate-600 cursor-not-allowed opacity-40'
                  : 'text-sky-400 hover:bg-sky-500/20 hover:text-sky-300'
              }`}
            >
              {pending === 'zero' ? <RefreshCw size={12} className="animate-spin text-sky-400" /> : <Crosshair size={12} />}
            </button>
            {tip('调零：重新示教，并把跟随基线换成机械臂当前位姿（增量归零）')}
          </div>

          <div className="w-[1px] h-3 bg-slate-700 mx-0.5" />

          {/* ③ 停止 */}
          <div className="relative group flex items-center justify-center">
            <button
              type="button"
              disabled={disabledStop}
              onClick={() => run('stop')}
              className={`w-6 h-6 rounded flex items-center justify-center transition-all ${
                disabledStop
                  ? 'text-slate-600 cursor-not-allowed opacity-40'
                  : 'text-rose-400 hover:bg-rose-500/20 hover:text-rose-300'
              }`}
            >
              {pending === 'stop' ? <RefreshCw size={12} className="animate-spin text-rose-400" /> : <Square size={11} />}
            </button>
            {tip('停止跟随：取流退回 hardware.camera，仿真臂交回真机状态')}
          </div>
        </div>
      </div>

      {/* 实时读数：跟随量 + 解算质量。省略号交给 truncate，全文在 title 里。 */}
      <div className="px-2.5 pb-1.5 flex flex-col gap-1">
        <div className="flex items-center gap-1.5 text-[10px] font-mono text-slate-400 overflow-hidden whitespace-nowrap">
          <span className={running ? 'text-emerald-300' : 'text-slate-500'}>
            {(snap?.status ?? 'idle')}{snap?.estimator && snap.estimator !== 'none' ? `·${snap.estimator}` : ''}
            {/* "保持"必须显出来：臂停住和臂没动在画面上长得一样，处置却完全不同。 */}
            {holding && <span className="text-amber-400/90"> ·保持</span>}
          </span>
          <span className="text-slate-600">|</span>
          <span title="相机相对示教位的位移增量（基座系映射前）">Δt {fmt(dtMm)} mm</span>
          <span title="旋转增量，由 delta_r 的 trace 反解">ΔR {fmt(dRdeg, 2)}°</span>
          <span className="text-slate-600">|</span>
          <span title="沿光轴的 1σ 重复性；— 表示本帧没有稠密解算">σz {fmt(sigmaT, 2)} mm</span>
          <span title={`${snap?.fps ?? 0} fps, 帧 ${snap?.frames ?? 0}, 丢帧 ${snap?.dropped ?? 0}/拒收 ${snap?.rejected ?? 0}`}>
            {fmt(snap?.fps, 0)}fps
          </span>
        </div>
        {(error || state?.last_error || snap?.reason) && (
          <div
            className={`text-[9px] truncate ${error || heldByIk ? 'text-amber-400/90' : 'text-slate-500'}`}
            title={error || state?.last_error || snap?.reason || ''}
          >
            {error || (heldByIk ? 'IK 失败：保持上一目标' : (snap?.reason || state?.last_error))}
          </div>
        )}
        {running && (
          <div className="text-[9px] text-slate-600 truncate" title={state?.r_cb_source || ''}>
            {`${snap?.capture_width ?? 0}x${snap?.capture_height ?? 0} ${snap?.align ?? ''}`}
            {snap?.teach_capture_width && snap.teach_capture_width !== snap.capture_width
              ? ` (示教 ${snap.teach_capture_width}x${snap.teach_capture_height})` : ''}
            {` · 基线 ${state?.arm_baseline_deg ? state.arm_baseline_deg.map((v) => v.toFixed(1)).join(',') : '—'}`}
          </div>
        )}
      </div>
    </div>
  );
};

export default FollowPanel;
