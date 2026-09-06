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
  // 以下全是**可选**：新增的陀螺通道诊断，老服务不发这些字段时面板照常工作（读不到就不显示）。
  /** 陀螺判定相机静止 ⇒ 旋转通道被有意冻结在冻结点上（平移照常更新）。 */
  gyro_still?: boolean;
  /** 冻结真的生效中（区别于"陀螺说静止但还在 hold-off 窗口里"）。 */
  rot_frozen?: boolean;
  /** 本次已冻结多久（ms）。到上限会被强制解冻 —— 陀螺零偏不能无限期吞掉真实旋转。 */
  frozen_ms?: number;
  /** 离群门拦下的坏帧累计数：视觉解算与陀螺角速度不一致 ⇒ 拒收并沿用上一位姿。 */
  rot_gated?: number;
  gyro?: {
    /** 设备时钟 → 主机时钟的偏移是否已定标。false = 旋转通道的陀螺证据全部无效。 */
    time_ready?: boolean;
    /** T_cam_gyro 是否从设备读到合法非 Identity 旋转。false = ω 还在设备系。 */
    extrinsics_loaded?: boolean;
    buf?: number;
    samples?: number;
    dead_frames?: number;
    bias_dps?: number;
    resid_dps?: number;
    bias_ready?: boolean;
  };
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
  /** 位姿数据面此刻走哪条路：'push' = 相机服务 SSE 推来，'poll' = 后端兜底轮询。
   *  **可选**：推送接入前的服务不发这两个字段，面板照常工作。 */
  data_plane_mode?: string;
  /** 走轮询时的原因（开关未开 / 尚无一帧 / 已 Xs 无数据）。推送正常时为空。 */
  data_plane_reason?: string;
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
          console.warn('Failed to parse follow_state:', err);
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
        setError(res.status === 503 ? `Backend not ready: ${detail}` : detail);
        return;
      }
      if (data.data) setState(data.data as FollowState);
    } catch (err) {
      setError(`Request failed: ${err instanceof Error ? err.message : String(err)}`);
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
  const sigmaT = snap?.sigma_t_mm?.[2] ?? null;
  const running = active && !!snap?.enabled;
  const heldByIk = !!state?.ik_failed;
  const holding = !!snap?.holding_last_pose || heldByIk;

  return (
    <div className="shrink-0 border-t border-slate-800 bg-slate-950/40 select-none">
      {/* Button Row */}
      <div className="h-9 px-2.5 flex justify-between items-center">
        <div className="flex items-center gap-1.5 text-[11px] text-slate-300 font-medium min-w-0">
          <Radio size={13} className={running ? 'text-emerald-400' : 'text-slate-500'} />
          <span>Follow</span>
          {!wsOpen && <span className="text-[9px] text-amber-400/80">Disconnected</span>}
          {state && state.arm_mode !== 'sim' && (
            <span className="text-[9px] text-rose-400/80">mode:{state.arm_mode}</span>
          )}
        </div>
        <div className="flex items-center gap-1">
          {/* Start */}
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
            {tip(isReplaying ? 'Playback active: Stop playback first'
                : active ? 'Follow is running'
                : 'Start: Home arm + Teach from camera view')}
          </div>

          {/* Zero */}
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
            {tip('Zero: Re-teach and reset follow baseline to current arm pose (Δ=0)')}
          </div>

          <div className="w-[1px] h-3 bg-slate-700 mx-0.5" />

          {/* Stop */}
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
            {tip('Stop: Revert camera stream & return arm to physical status')}
          </div>
        </div>
      </div>

      {/* Real-time Telemetry Display */}
      <div className="px-2.5 pb-1.5 flex flex-col gap-1">
        <div className="flex items-center gap-1.5 text-[10px] font-mono text-slate-400 overflow-hidden whitespace-nowrap">
          <span className={running ? 'text-emerald-300' : 'text-slate-500'}>
            {(snap?.status ?? 'idle')}{snap?.estimator && snap.estimator !== 'none' ? `·${snap.estimator}` : ''}
            {holding && <span className="text-amber-400/90"> ·Hold</span>}
          </span>
          <span className="text-slate-600">|</span>
          <span title="Camera displacement relative to taught pose (pre-base mapping)">Δt {fmt(dtMm)} mm</span>
          <span title="Rotation increment from delta_r trace">ΔR {fmt(dRdeg, 2)}°</span>
          {snap?.rot_frozen && (
            <span
              className="text-sky-400/90"
              title={`Gyro stationary, rotation channel frozen for ${fmt(snap.frozen_ms, 0)}ms. Translation updating.`}
            >
              ·Frozen {fmt(snap.frozen_ms, 0)}ms
            </span>
          )}
          <span className="text-slate-600">|</span>
          <span title="1σ repeatability along optical axis; — indicates no dense solution">σz {fmt(sigmaT, 2)} mm</span>
          <span title={`${snap?.fps ?? 0} fps, frames ${snap?.frames ?? 0}, dropped ${snap?.dropped ?? 0}/rejected ${snap?.rejected ?? 0}`}>
            {fmt(snap?.fps, 0)}fps
          </span>
          {snap?.rot_gated ? <span title={`Outlier gate rejected ${snap.rot_gated} frames: vision-gyro disagreement`}>Rej {snap.rot_gated}</span> : null}
        </div>

        {running && snap?.gyro && snap.gyro.time_ready === false && (
          <div className="text-[9px] truncate text-rose-400/90"
               title="Device timestamp offset uncalibrated: gyro samples discarded, using vision-only.">
            Gyro Clock Uncalibrated · Rotation Degraded
          </div>
        )}
        {running && snap?.gyro && (snap.gyro.dead_frames ?? 0) > 0 && (
          <div className="text-[9px] truncate text-rose-400/90"
               title={`Buffer ${snap.gyro.buf ?? 0}, window ${snap.gyro.samples ?? 0}: Frames and gyro out of sync.`}>
            Gyro Window Idle {snap.gyro.dead_frames} Frames · Buf {snap.gyro.buf ?? 0} / Win {snap.gyro.samples ?? 0}
          </div>
        )}
        {running && snap?.gyro && snap.gyro.time_ready && snap.gyro.extrinsics_loaded === false && (
          <div className="text-[9px] truncate text-amber-400/80"
               title="T_cam_gyro extrinsics Identity: stationary gate may fail.">
            Gyro Extrinsics Identity · Gate May Fail
          </div>
        )}
        {(error || state?.last_error || snap?.reason) && (
          <div
            className={`text-[9px] truncate ${error || heldByIk ? 'text-amber-400/90' : 'text-slate-500'}`}
            title={error || state?.last_error || snap?.reason || ''}
          >
            {error || (heldByIk ? 'IK Failed: Holding Last Target' : (snap?.reason || state?.last_error))}
          </div>
        )}
        {running && (
          <div className="text-[9px] text-slate-600 truncate" title={state?.r_cb_source || ''}>
            {`${snap?.capture_width ?? 0}x${snap?.capture_height ?? 0} ${snap?.align ?? ''}`}
            {snap?.teach_capture_width && snap.teach_capture_width !== snap.capture_width
              ? ` (Teach ${snap.teach_capture_width}x${snap.teach_capture_height})` : ''}
            {` · Baseline ${state?.arm_baseline_deg ? state.arm_baseline_deg.map((v) => v.toFixed(1)).join(',') : '—'}`}
            {state?.data_plane_mode === 'push'
              ? <span title="Pose pushed via SSE from camera service; polling on standby"> · Push</span>
              : state?.data_plane_mode === 'poll'
                ? <span className="text-amber-400/80"
                        title={state.data_plane_reason || 'Pose acquired via backend polling'}> · Polling Fallback</span>
                : null}
          </div>
        )}
      </div>
    </div>
  );
};

export default FollowPanel;
