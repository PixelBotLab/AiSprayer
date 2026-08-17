import React, { useState } from 'react';
import {
  ShieldCheck,
  RefreshCw,
  Sparkles,
  Layers,
  HardDrive,
  Check,
  ChevronRight,
  X,
  Compass,
  Zap,
  Eye,
  SlidersHorizontal,
  Route,
} from 'lucide-react';
import type {
  VerificationReport,
  KinematicsParams,
  UrdfTcpInfo,
  ManualPathItem,
  PathStateType,
  PoiConfig,
} from './types';
import { STATE_THEMES } from './types';

interface DiagnosticsDashboardProps {
  activeState: PathStateType;
  rawReport: VerificationReport | null;
  optReport: VerificationReport | null;
  poiReport: VerificationReport | null;
  rawPaths: ManualPathItem[];
  optPaths: ManualPathItem[];
  poiPaths: ManualPathItem[];
  isVerifying: boolean;
  isOptimizing: boolean;
  activeTemplate: string | null;
  kinParams: KinematicsParams;
  poiConfig: PoiConfig;
  urdfTcpInfo: UrdfTcpInfo | null;
  isKinParamsOpen: boolean;
  highlightedPathId?: number | null;
  setKinParams: React.Dispatch<React.SetStateAction<KinematicsParams>>;
  setPoiConfig: React.Dispatch<React.SetStateAction<PoiConfig>>;
  setIsKinParamsOpen: React.Dispatch<React.SetStateAction<boolean>>;
  setHighlightedPathId?: (id: number | null) => void;
  onSelectActiveState: (state: PathStateType) => void;
  onRunDiagnostics: (state?: PathStateType) => void;
  onApplyOptimization: (mode: 'opt' | 'poi') => void;
  onFetchAnchorPose: (source: 'home' | 'live') => void;
  onClose?: () => void;
}

export const DiagnosticsDashboard: React.FC<DiagnosticsDashboardProps> = ({
  activeState,
  rawReport,
  optReport,
  poiReport,
  rawPaths,
  optPaths,
  poiPaths,
  isVerifying,
  isOptimizing,
  activeTemplate,
  kinParams,
  poiConfig,
  urdfTcpInfo,
  isKinParamsOpen,
  setKinParams,
  setPoiConfig,
  setIsKinParamsOpen,
  onSelectActiveState,
  onRunDiagnostics,
  onApplyOptimization,
  onFetchAnchorPose,
  onClose,
}) => {
  const [selectedTab, setSelectedTab] = useState<'matrix' | 'inspector' | 'poi_settings'>('matrix');
  const [selectedPathIndex, setSelectedPathIndex] = useState<number>(0);

  // Active paths list based on selected state
  const currentPaths = activeState === 'poi' ? poiPaths : (activeState === 'opt' ? optPaths : rawPaths);

  const findPathById = (paths: ManualPathItem[], pathId: number | undefined, fallbackIndex: number) => {
    if (pathId !== undefined) {
      const matched = paths.find((p) => p.path_id === pathId);
      if (matched) return matched;
    }
    return paths[fallbackIndex] || null;
  };

  const findWaypointByIndex = (path: ManualPathItem | null, waypointIndex: number | undefined, fallbackIndex: number) => {
    if (!path?.points) return null;
    if (waypointIndex !== undefined) {
      const matched = path.points.find((p) => p.index === waypointIndex);
      if (matched) return matched;
    }
    return path.points[fallbackIndex] || null;
  };

  const poseText = (pose?: { x: number; y: number; z: number; rx: number; ry: number; rz: number } | null) => {
    if (!pose) return <span className="text-slate-600">Missing</span>;
    return <span>XYZ[{Math.round(pose.x)}, {Math.round(pose.y)}, {Math.round(pose.z)}] RPY[{Math.round(pose.rx)}°, {Math.round(pose.ry)}°, {Math.round(pose.rz)}°]</span>;
  };

  const deltaText = (
    base?: { rx: number; ry: number; rz: number } | null,
    target?: { rx: number; ry: number; rz: number } | null
  ) => {
    if (!base || !target) return <span className="text-slate-600">Δ—</span>;
    const dRx = target.rx - base.rx;
    const dRy = target.ry - base.ry;
    const dRz = target.rz - base.rz;
    const changed = Math.max(Math.abs(dRx), Math.abs(dRy), Math.abs(dRz)) > 0.5;
    return (
      <span className={changed ? 'text-amber-300' : 'text-slate-500'}>
        Δ[{dRx.toFixed(1)}°, {dRy.toFixed(1)}°, {dRz.toFixed(1)}°]
      </span>
    );
  };

  const maxVelLimits = [180, 180, 180, 180, 180, 180];

  // Helper to extract peak joint velocity across a report's paths
  const getOverallPeakJoints = (rep: VerificationReport | null): number[] => {
    if (!rep?.path_reports || rep.path_reports.length === 0) return [0, 0, 0, 0, 0, 0];
    const peaks = [0, 0, 0, 0, 0, 0];
    rep.path_reports.forEach((pr) => {
      const v = pr.max_joint_velocity_deg_s || pr.peak_joint_speeds_deg_s || [];
      for (let i = 0; i < 6; i++) {
        peaks[i] = Math.max(peaks[i], v[i] || 0);
      }
    });
    return peaks;
  };

  const rawPeakJoints = getOverallPeakJoints(rawReport);
  const optPeakJoints = getOverallPeakJoints(optReport);
  const poiPeakJoints = getOverallPeakJoints(poiReport);

  const getStatusBadge = (rep: VerificationReport | null) => {
    if (!rep) return <span className="text-[9px] font-mono text-slate-500">N/A</span>;
    const s = rep.summary.status;
    if (s === 'PASS') {
      return (
        <span className="px-1.5 py-0.5 rounded text-[9px] font-mono font-bold bg-emerald-950/60 text-emerald-400 border border-emerald-500/30 flex items-center gap-1 justify-center">
          <Check size={9} /> PASS
        </span>
      );
    }
    if (s === 'WARNING') {
      return (
        <span className="px-1.5 py-0.5 rounded text-[9px] font-mono font-bold bg-amber-950/60 text-amber-400 border border-amber-500/30 flex items-center gap-1 justify-center">
          ⚠️ WARN
        </span>
      );
    }
    return (
      <span className="px-1.5 py-0.5 rounded text-[9px] font-mono font-bold bg-rose-950/60 text-rose-400 border border-rose-500/30 flex items-center gap-1 justify-center">
        ❌ ERR
      </span>
    );
  };

  return (
    <div className="w-full h-full flex flex-col bg-slate-900 overflow-y-auto select-none custom-scrollbar relative font-sans">
      {/* Header Bar */}
      <div className="p-2.5 border-b border-slate-800 bg-slate-950/90 sticky top-0 z-20 flex items-center justify-between">
        <div className="flex items-center gap-1.5 min-w-0">
          <ShieldCheck size={14} className="text-sky-400 shrink-0" />
          <span className="text-[11px] font-bold text-slate-200 tracking-wider truncate">
            DIAGNOSTICS MATRIX
          </span>
        </div>
        <div className="flex items-center gap-1 shrink-0">
          <button
            onClick={() => onRunDiagnostics(activeState)}
            disabled={isVerifying || !activeTemplate}
            className="px-2 py-0.5 text-[9.5px] font-medium rounded bg-slate-800 hover:bg-slate-700 text-sky-300 border border-sky-500/40 flex items-center gap-1 shadow transition-all disabled:opacity-40"
            title="Re-verify kinematics"
          >
            <RefreshCw size={9.5} className={isVerifying ? 'animate-spin' : ''} />
            <span>{isVerifying ? 'Verifying...' : 'Re-verify'}</span>
          </button>
          {onClose && (
            <button
              onClick={onClose}
              className="p-1 rounded hover:bg-slate-800 text-slate-400 hover:text-slate-200 transition-colors ml-0.5"
              title="Close panel"
            >
              <X size={13} />
            </button>
          )}
        </div>
      </div>

      {/* Top 3-Way Mode Switcher & Tab Sub-Nav */}
      <div className="p-2 border-b border-slate-800/80 bg-slate-950/40 flex flex-col gap-2 shrink-0">
        {/* Active State Selector Pills: RAW / OPT / POI */}
        <div className="grid grid-cols-3 gap-1 bg-slate-950 p-1 rounded-lg border border-slate-800/80">
          {(['raw', 'opt', 'poi'] as PathStateType[]).map((st) => {
            const isActive = activeState === st;
            const theme = STATE_THEMES[st];
            const hasData = st === 'raw' ? rawPaths.length > 0 : (st === 'opt' ? optPaths.length > 0 : poiPaths.length > 0);
            return (
              <button
                key={st}
                onClick={() => onSelectActiveState(st)}
                disabled={!hasData && st !== 'raw'}
                className={`py-1 px-1.5 rounded flex items-center justify-between text-[10px] font-bold tracking-wider transition-all disabled:opacity-30 ${
                  isActive
                    ? `${theme.lightBg} ${theme.text} border ${theme.border} shadow-md`
                    : 'text-slate-400 hover:text-slate-200 hover:bg-slate-900/60'
                }`}
              >
                <div className="flex items-center gap-1">
                  <span
                    className="w-1.5 h-1.5 rounded-full"
                    style={{ backgroundColor: theme.hex }}
                  />
                  <span>{theme.label}</span>
                </div>
                {hasData ? (
                  <span className="text-[8.5px] font-mono opacity-80">
                    {st === 'raw' ? rawPaths.length : (st === 'opt' ? optPaths.length : poiPaths.length)}P
                  </span>
                ) : (
                  <span className="text-[8px] font-mono text-slate-600">Empty</span>
                )}
              </button>
            );
          })}
        </div>

        {/* View Mode Tabs: Matrix Table | Waypoints | POI Settings */}
        <div className="flex items-center justify-between text-[10px] font-medium border-b border-slate-800 pb-0.5">
          <div className="flex gap-2">
            <button
              onClick={() => setSelectedTab('matrix')}
              className={`pb-1 flex items-center gap-1 transition-all ${
                selectedTab === 'matrix'
                  ? 'text-sky-400 border-b-2 border-sky-400 font-bold'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <Layers size={11} />
              <span>3-State Matrix</span>
            </button>
            <button
              onClick={() => setSelectedTab('inspector')}
              className={`pb-1 flex items-center gap-1 transition-all ${
                selectedTab === 'inspector'
                  ? 'text-sky-400 border-b-2 border-sky-400 font-bold'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <Eye size={11} />
              <span>Waypoints</span>
            </button>
          </div>
          <button
            onClick={() => {
              setSelectedTab('poi_settings');
              if (!poiConfig.anchor_source || poiConfig.anchor_source === 'home' || (poiConfig.ref_rpy_deg[0] === 0 && poiConfig.ref_rpy_deg[1] === 0 && poiConfig.ref_rpy_deg[2] === 0)) {
                onFetchAnchorPose('home');
                setPoiConfig((prev) => ({
                  ...prev,
                  anchor_source: 'home',
                  ref_rpy_deg: [90.0, 0.0, 90.0],
                  tolerance_rpy_deg: [20.0, 20.0, 180.0],
                }));
              }
            }}
            className={`pb-1 flex items-center gap-1 transition-all ${
              selectedTab === 'poi_settings'
                ? 'text-emerald-400 border-b-2 border-emerald-400 font-bold'
                : 'text-slate-400 hover:text-emerald-300'
            }`}
          >
            <SlidersHorizontal size={11} />
            <span>POI Config</span>
          </button>
        </div>
      </div>

      {/* Main Tab Content */}
      <div className="p-3 space-y-3.5 text-xs flex-1">
        {/* ========================================================================= */}
        {/* TAB 1: 3-STATE MATRIX COMPARISON TABLE (RAW 灰 / OPT 蓝 / POI 绿)           */}
        {/* ========================================================================= */}
        {selectedTab === 'matrix' && (
          <div className="space-y-3">
            {/* Global Metrics Comparison Table */}
            <div className="rounded-lg border border-slate-800 bg-slate-950/60 overflow-hidden shadow-inner">
              <table className="w-full text-[10px] text-left border-collapse">
                <thead>
                  <tr className="bg-slate-950 border-b border-slate-800 text-[9.5px] uppercase font-mono text-slate-400">
                    <th className="p-2 font-semibold">Metric</th>
                    <th className="p-2 font-bold text-slate-300 bg-slate-900/40 text-center border-l border-slate-800">
                      <div className="flex items-center justify-center gap-1">
                        <span className="w-1.5 h-1.5 rounded-full bg-slate-400" />
                        <span>RAW</span>
                      </div>
                    </th>
                    <th className="p-2 font-bold text-sky-400 bg-sky-950/20 text-center border-l border-slate-800">
                      <div className="flex items-center justify-center gap-1">
                        <span className="w-1.5 h-1.5 rounded-full bg-sky-400" />
                        <span>OPT</span>
                      </div>
                    </th>
                    <th className="p-2 font-bold text-emerald-400 bg-emerald-950/20 text-center border-l border-slate-800">
                      <div className="flex items-center justify-center gap-1">
                        <span className="w-1.5 h-1.5 rounded-full bg-emerald-400" />
                        <span>POI</span>
                      </div>
                    </th>
                  </tr>
                </thead>
                <tbody className="divide-y divide-slate-800/80 font-mono">
                  {/* Row 1: Status */}
                  <tr>
                    <td className="p-2 text-slate-400 font-sans font-medium text-[10px]">Status</td>
                    <td className="p-2 text-center bg-slate-900/20 border-l border-slate-800">
                      {getStatusBadge(rawReport)}
                    </td>
                    <td className="p-2 text-center bg-sky-950/10 border-l border-slate-800">
                      {getStatusBadge(optReport)}
                    </td>
                    <td className="p-2 text-center bg-emerald-950/10 border-l border-slate-800">
                      {getStatusBadge(poiReport)}
                    </td>
                  </tr>

                  {/* Row 2: Issues */}
                  <tr>
                    <td className="p-2 text-slate-400 font-sans font-medium text-[10px]">Issues</td>
                    <td className="p-2 text-center text-slate-300 border-l border-slate-800">
                      {rawReport?.summary ? `${rawReport.summary.total_issues}` : '-'}
                    </td>
                    <td className="p-2 text-center text-sky-300 border-l border-slate-800">
                      {optReport?.summary ? `${optReport.summary.total_issues}` : '-'}
                    </td>
                    <td className="p-2 text-center text-emerald-300 border-l border-slate-800">
                      {poiReport?.summary ? `${poiReport.summary.total_issues}` : '-'}
                    </td>
                  </tr>

                  {/* Row 3: MoveL Steps */}
                  <tr>
                    <td className="p-2 text-slate-400 font-sans font-medium text-[10px]">Steps</td>
                    <td className="p-2 text-center text-slate-300 border-l border-slate-800">
                      {rawReport?.summary ? `${rawReport.summary.total_steps}` : '-'}
                    </td>
                    <td className="p-2 text-center text-sky-300 border-l border-slate-800">
                      {optReport?.summary ? `${optReport.summary.total_steps}` : '-'}
                    </td>
                    <td className="p-2 text-center text-emerald-300 border-l border-slate-800">
                      {poiReport?.summary ? `${poiReport.summary.total_steps}` : '-'}
                    </td>
                  </tr>
                </tbody>
              </table>
            </div>

            {/* Peak Joint Velocities Graphic Comparison Bars (J1..J6) */}
            <div className="p-2.5 rounded-lg border border-slate-800 bg-slate-950/40 space-y-2">
              <div className="flex items-center justify-between text-[10px]">
                <span className="font-bold text-slate-300 flex items-center gap-1">
                  <Zap size={11} className="text-amber-400" />
                  <span>Peak Joint Velocity (J1~J6)</span>
                </span>
                <div className="flex items-center gap-2 text-[8.5px] font-mono">
                  <span className="flex items-center gap-0.5 text-slate-400">
                    <span className="w-1.5 h-1.5 bg-slate-400 rounded-sm" /> Raw
                  </span>
                  <span className="flex items-center gap-0.5 text-sky-400">
                    <span className="w-1.5 h-1.5 bg-sky-400 rounded-sm" /> Opt
                  </span>
                  <span className="flex items-center gap-0.5 text-emerald-400">
                    <span className="w-1.5 h-1.5 bg-emerald-400 rounded-sm" /> POI
                  </span>
                </div>
              </div>

              <div className="grid grid-cols-6 gap-1.5 pt-1">
                {[0, 1, 2, 3, 4, 5].map((jIdx) => {
                  const limit = maxVelLimits[jIdx];
                  const vRaw = rawPeakJoints[jIdx] || 0;
                  const vOpt = optPeakJoints[jIdx] || 0;
                  const vPoi = poiPeakJoints[jIdx] || 0;

                  return (
                    <div
                      key={jIdx}
                      className="flex flex-col items-center bg-slate-900/90 p-1.5 rounded border border-slate-800/80 gap-1 group relative"
                    >
                      <span className="text-[9px] font-mono font-bold text-slate-300">
                        J{jIdx + 1}
                      </span>
                      {/* Vertical side-by-side comparative bars */}
                      <div className="h-14 w-full flex items-end justify-center gap-0.5 bg-slate-950 p-0.5 rounded overflow-hidden">
                        {/* Raw Bar */}
                        <div
                          className={`w-1.5 rounded-t transition-all ${
                            vRaw > limit ? 'bg-rose-500' : 'bg-slate-400'
                          }`}
                          style={{ height: `${Math.min(100, (vRaw / limit) * 100)}%` }}
                          title={`Raw J${jIdx + 1}: ${Math.round(vRaw)}°/s`}
                        />
                        {/* Opt Bar */}
                        <div
                          className={`w-1.5 rounded-t transition-all ${
                            vOpt > limit ? 'bg-rose-500' : 'bg-sky-400'
                          }`}
                          style={{ height: `${Math.min(100, (vOpt / limit) * 100)}%` }}
                          title={`Opt J${jIdx + 1}: ${Math.round(vOpt)}°/s`}
                        />
                        {/* POI Bar */}
                        <div
                          className={`w-1.5 rounded-t transition-all ${
                            vPoi > limit ? 'bg-rose-500' : 'bg-emerald-400'
                          }`}
                          style={{ height: `${Math.min(100, (vPoi / limit) * 100)}%` }}
                          title={`POI J${jIdx + 1}: ${Math.round(vPoi)}°/s`}
                        />
                      </div>
                      <span className="text-[8px] font-mono text-slate-400">
                        {Math.round(activeState === 'poi' ? vPoi : (activeState === 'opt' ? vOpt : vRaw))}°/s
                      </span>
                    </div>
                  );
                })}
              </div>
            </div>

            {/* Multi-Path Breakdown Table (All Paths Comparison in 3-State Matrix) */}
            {rawPaths.length > 0 && (
              <div className="rounded-lg border border-slate-800 bg-slate-950/60 overflow-hidden shadow-inner p-2 space-y-1.5">
                <div className="flex items-center justify-between text-[10px] font-bold text-slate-300 px-0.5">
                  <span className="flex items-center gap-1">
                    <Route size={11} className="text-sky-400" />
                    <span>Multi-Path Breakdown ({rawPaths.length} Paths)</span>
                  </span>
                  <span className="text-[8.5px] font-mono text-slate-500">Click row to inspect</span>
                </div>
                <div className="overflow-x-auto max-h-[160px] custom-scrollbar">
                  <table className="w-full text-[9px] font-mono border-collapse">
                    <thead className="bg-slate-900/90 text-slate-400 text-[8.5px] uppercase border-b border-slate-800 sticky top-0">
                      <tr>
                        <th className="p-1 text-left">Path</th>
                        <th className="p-1 text-center text-slate-400">Pts</th>
                        <th className="p-1 text-center text-slate-300">RAW</th>
                        <th className="p-1 text-center text-sky-400">OPT</th>
                        <th className="p-1 text-center text-emerald-400">POI</th>
                      </tr>
                    </thead>
                    <tbody className="divide-y divide-slate-800/60">
                      {rawPaths.map((p, idx) => {
                        const rawPRep = rawReport?.path_reports?.[idx];
                        const optPRep = optReport?.path_reports?.[idx];
                        const poiPRep = poiReport?.path_reports?.[idx];

                        return (
                          <tr
                            key={p.path_id || idx}
                            onClick={() => {
                              setSelectedPathIndex(idx);
                              setSelectedTab('inspector');
                            }}
                            className="hover:bg-slate-800/50 cursor-pointer transition-colors"
                          >
                            <td className="p-1 text-slate-200 font-medium">
                              Path {p.path_id || idx + 1}
                            </td>
                            <td className="p-1 text-center text-slate-500">
                              {p.points?.length || 0}
                            </td>
                            <td className="p-1 text-center">
                              {rawPRep ? (
                                <span className={`px-1 py-0.2 rounded text-[8px] font-bold ${
                                  rawPRep.status === 'PASS' ? 'text-emerald-400 bg-emerald-950/60' : (rawPRep.status === 'WARNING' ? 'text-amber-400 bg-amber-950/60' : 'text-rose-400 bg-rose-950/60')
                                }`}>
                                  {rawPRep.status} {rawPRep.issues?.length > 0 && `(${rawPRep.issues.length})`}
                                </span>
                              ) : <span className="text-slate-600">-</span>}
                            </td>
                            <td className="p-1 text-center">
                              {optPRep ? (
                                <span className={`px-1 py-0.2 rounded text-[8px] font-bold ${
                                  optPRep.status === 'PASS' ? 'text-sky-400 bg-sky-950/60' : (optPRep.status === 'WARNING' ? 'text-amber-400 bg-amber-950/60' : 'text-rose-400 bg-rose-950/60')
                                }`}>
                                  {optPRep.status} {optPRep.issues?.length > 0 && `(${optPRep.issues.length})`}
                                </span>
                              ) : <span className="text-slate-600">-</span>}
                            </td>
                            <td className="p-1 text-center">
                              {poiPRep ? (
                                <span className={`px-1 py-0.2 rounded text-[8px] font-bold ${
                                  poiPRep.status === 'PASS' ? 'text-emerald-400 bg-emerald-950/60' : (poiPRep.status === 'WARNING' ? 'text-amber-400 bg-amber-950/60' : 'text-rose-400 bg-rose-950/60')
                                }`}>
                                  {poiPRep.status} {poiPRep.issues?.length > 0 && `(${poiPRep.issues.length})`}
                                </span>
                              ) : <span className="text-slate-600">-</span>}
                            </td>
                          </tr>
                        );
                      })}
                    </tbody>
                  </table>
                </div>
              </div>
            )}

            {/* Quick Optimization Triggers */}
            <div className="grid grid-cols-2 gap-2 pt-1">
              <button
                onClick={() => onApplyOptimization('opt')}
                disabled={isOptimizing || !activeTemplate || rawPaths.length === 0}
                className="py-1.5 px-2 rounded-lg bg-sky-600 hover:bg-sky-500 text-white font-medium text-[10.5px] flex items-center justify-center gap-1 shadow-md shadow-sky-950/40 transition-all disabled:opacity-40"
              >
                <Sparkles size={12} className={isOptimizing ? 'animate-spin' : ''} />
                <span>Auto-Fix Opt</span>
              </button>
              <button
                onClick={() => onApplyOptimization('poi')}
                disabled={isOptimizing || !activeTemplate || rawPaths.length === 0}
                className="py-1.5 px-2 rounded-lg bg-emerald-600 hover:bg-emerald-500 text-white font-medium text-[10.5px] flex items-center justify-center gap-1 shadow-md shadow-emerald-950/40 transition-all disabled:opacity-40"
              >
                <Compass size={12} className={isOptimizing ? 'animate-spin' : ''} />
                <span>Optimize POI</span>
              </button>
            </div>
          </div>
        )}

        {/* ========================================================================= */}
        {/* TAB 2: WAYPOINT DETAILED COMPARISON TABLE (Waypoint Inspector)              */}
        {/* ========================================================================= */}
        {selectedTab === 'inspector' && (
          <div className="space-y-2.5">
            {/* Path selector tabs */}
            {currentPaths.length > 1 && (
              <div className="flex items-center gap-1 overflow-x-auto pb-1 custom-scrollbar">
                {currentPaths.map((p, pIdx) => (
                  <button
                    key={p.path_id || pIdx}
                    onClick={() => setSelectedPathIndex(pIdx)}
                    className={`px-2 py-0.5 rounded text-[9.5px] font-mono font-medium shrink-0 transition-all ${
                      selectedPathIndex === pIdx
                        ? 'bg-sky-600 text-white shadow'
                        : 'bg-slate-800 text-slate-400 hover:text-slate-200'
                    }`}
                  >
                    Path {p.path_id || pIdx + 1}
                  </button>
                ))}
              </div>
            )}

            {/* Waypoint Rows Table */}
            {currentPaths[selectedPathIndex]?.points?.length ? (
              <div className="rounded-lg border border-slate-800 bg-slate-950/60 overflow-hidden shadow-inner max-h-[280px] overflow-y-auto custom-scrollbar">
                <table className="w-full text-[9px] font-mono border-collapse">
                  <thead className="bg-slate-900/90 text-slate-400 font-mono text-[9px] border-b border-slate-800 sticky top-0">
                    <tr>
                      <th className="p-1.5 text-center">WP</th>
                      <th className="p-1.5 text-left text-slate-300">RAW Pose</th>
                      <th className="p-1.5 text-left text-sky-400">OPT Pose / ΔRaw</th>
                      <th className="p-1.5 text-left text-emerald-400">POI Pose / ΔRaw</th>
                    </tr>
                  </thead>
                  <tbody className="divide-y divide-slate-800/60">
                    {currentPaths[selectedPathIndex].points.map((wp, wIdx) => {
                      const pathId = currentPaths[selectedPathIndex]?.path_id;
                      const waypointIndex = wp.index || wIdx + 1;
                      const rawPath = findPathById(rawPaths, pathId, selectedPathIndex);
                      const optPath = findPathById(optPaths, pathId, selectedPathIndex);
                      const poiPath = findPathById(poiPaths, pathId, selectedPathIndex);
                      const rawWp = findWaypointByIndex(rawPath, waypointIndex, wIdx);
                      const optWp = findWaypointByIndex(optPath, waypointIndex, wIdx);
                      const poiWp = findWaypointByIndex(poiPath, waypointIndex, wIdx);

                      const rawPose = rawWp?.tcp_pose_base || null;
                      const optPose = optWp?.tcp_pose_base || null;
                      const poiPose = poiWp?.tcp_pose_base || null;
                      const hasMismatch = !rawWp || (optPaths.length > 0 && !optWp) || (poiPaths.length > 0 && !poiWp);

                      return (
                        <tr key={`${pathId || selectedPathIndex}-${waypointIndex}`} className="hover:bg-slate-800/40 transition-colors group">
                          <td className="p-1.5 text-center font-bold text-slate-400">
                            <div>#{waypointIndex}</div>
                            {hasMismatch && <div className="text-[8px] text-amber-400">MISMATCH</div>}
                          </td>
                          <td className="p-1.5 text-slate-300 font-mono text-[8.5px]">
                            {poseText(rawPose)}
                          </td>
                          <td className="p-1.5 text-sky-300 font-mono text-[8.5px] space-y-0.5">
                            <div>{poseText(optPose)}</div>
                            <div>{deltaText(rawPose, optPose)}</div>
                          </td>
                          <td className="p-1.5 text-emerald-300 font-mono text-[8.5px] space-y-0.5">
                            <div>{poseText(poiPose)}</div>
                            <div>{deltaText(rawPose, poiPose)}</div>
                          </td>
                        </tr>
                      );
                    })}
                  </tbody>
                </table>
              </div>
            ) : (
              <div className="p-4 text-center text-slate-500 border border-dashed border-slate-800 rounded-lg">
                No waypoints available.
              </div>
            )}
          </div>
        )}

        {/* ========================================================================= */}
        {/* TAB 3: POI CONSTRAINT PARAMETERS CONFIGURATION PANEL                       */}
        {/* ========================================================================= */}
        {selectedTab === 'poi_settings' && (
          <div className="space-y-3 bg-slate-950/40 p-3 rounded-lg border border-slate-800">
            <div className="flex items-center justify-between border-b border-slate-800 pb-1.5">
              <span className="text-[10.5px] font-bold text-emerald-400 flex items-center gap-1">
                <Compass size={12} />
                <span>POI Pose Anchor & Envelope</span>
              </span>
              <span className="text-[8.5px] font-mono text-slate-500 bg-slate-900 px-1.5 py-0.5 rounded border border-slate-800">
                Feature 6
              </span>
            </div>

            {/* Reference Pose Section */}
            <div className="space-y-1.5">
              <div className="flex items-center justify-between text-[10px] text-slate-400">
                <span>Anchor Reference Pose (Rx, Ry, Rz)</span>
                <div className="flex gap-1">
                  <button
                    onClick={() => onFetchAnchorPose('home')}
                    className="px-1.5 py-0.5 text-[8.5px] font-mono rounded bg-slate-800 hover:bg-slate-700 text-sky-300 border border-sky-500/30"
                    title="Read Home Pose"
                  >
                    Home
                  </button>
                  <button
                    onClick={() => onFetchAnchorPose('live')}
                    className="px-1.5 py-0.5 text-[8.5px] font-mono rounded bg-slate-800 hover:bg-slate-700 text-emerald-300 border border-emerald-500/30"
                    title="Capture Live Robot Pose"
                  >
                    Robot
                  </button>
                </div>
              </div>
              <div className="grid grid-cols-3 gap-1.5 font-mono text-[10px]">
                <div className="flex items-center bg-slate-900 border border-slate-800 rounded px-1.5 py-0.5">
                  <span className="text-slate-500 text-[9px] mr-1">Rx:</span>
                  <input
                    type="number"
                    value={poiConfig.ref_rpy_deg[0]}
                    onChange={(e) =>
                      setPoiConfig((prev) => ({
                        ...prev,
                        ref_rpy_deg: [parseFloat(e.target.value) || 0, prev.ref_rpy_deg[1], prev.ref_rpy_deg[2]],
                      }))
                    }
                    className="w-full bg-transparent text-slate-200 focus:outline-none"
                  />
                </div>
                <div className="flex items-center bg-slate-900 border border-slate-800 rounded px-1.5 py-0.5">
                  <span className="text-slate-500 text-[9px] mr-1">Ry:</span>
                  <input
                    type="number"
                    value={poiConfig.ref_rpy_deg[1]}
                    onChange={(e) =>
                      setPoiConfig((prev) => ({
                        ...prev,
                        ref_rpy_deg: [prev.ref_rpy_deg[0], parseFloat(e.target.value) || 0, prev.ref_rpy_deg[2]],
                      }))
                    }
                    className="w-full bg-transparent text-slate-200 focus:outline-none"
                  />
                </div>
                <div className="flex items-center bg-slate-900 border border-slate-800 rounded px-1.5 py-0.5">
                  <span className="text-slate-500 text-[9px] mr-1">Rz:</span>
                  <input
                    type="number"
                    value={poiConfig.ref_rpy_deg[2]}
                    onChange={(e) =>
                      setPoiConfig((prev) => ({
                        ...prev,
                        ref_rpy_deg: [prev.ref_rpy_deg[0], prev.ref_rpy_deg[1], parseFloat(e.target.value) || 0],
                      }))
                    }
                    className="w-full bg-transparent text-slate-200 focus:outline-none"
                  />
                </div>
              </div>
              <div className="mt-2 flex items-center justify-between">
                <span className="text-[9px] text-slate-400">Anchor Source:</span>
                <div className="flex bg-slate-900 border border-slate-800 rounded overflow-hidden">
                  <button
                    className={`px-2 py-0.5 text-[9px] ${poiConfig.anchor_source === 'home' || !poiConfig.anchor_source ? 'bg-indigo-600 text-white' : 'text-slate-400 hover:bg-slate-800'}`}
                    onClick={() => onFetchAnchorPose('home')}
                  >
                    Home
                  </button>
                  <button
                    className={`px-2 py-0.5 text-[9px] ${poiConfig.anchor_source === 'manual' ? 'bg-indigo-600 text-white' : 'text-slate-400 hover:bg-slate-800'}`}
                    onClick={() => setPoiConfig((prev) => ({ ...prev, anchor_source: 'manual' }))}
                  >
                    Manual
                  </button>
                  <button
                    className={`px-2 py-0.5 text-[9px] ${poiConfig.anchor_source === 'raw' ? 'bg-indigo-600 text-white' : 'text-slate-400 hover:bg-slate-800'}`}
                    onClick={() => setPoiConfig((prev) => ({ ...prev, anchor_source: 'raw' }))}
                    title="Use raw trajectory surface normal"
                  >
                    RAW Path
                  </button>
                </div>
              </div>
            </div>

            {/* Tolerance Envelope Section */}
            <div className="space-y-2 pt-1 border-t border-slate-800/80">
              <div className="text-[10px] text-slate-400 font-medium">Tolerance Envelope (±deg)</div>

              {/* Tol Rx */}
              <div className="space-y-0.5">
                <div className="flex justify-between text-[9.5px]">
                  <span className="text-slate-400">Tol ±Rx (Pitch)</span>
                  <span className="font-mono text-emerald-400">±{poiConfig.tolerance_rpy_deg[0]}°</span>
                </div>
                <input
                  type="range"
                  min={0.0}
                  max={45.0}
                  step={0.5}
                  value={poiConfig.tolerance_rpy_deg[0]}
                  onChange={(e) =>
                    setPoiConfig((prev) => ({
                      ...prev,
                      tolerance_rpy_deg: [parseFloat(e.target.value), prev.tolerance_rpy_deg[1], prev.tolerance_rpy_deg[2]],
                    }))
                  }
                  className="w-full accent-emerald-500 h-1 bg-slate-800 rounded"
                />
              </div>

              {/* Tol Ry */}
              <div className="space-y-0.5">
                <div className="flex justify-between text-[9.5px]">
                  <span className="text-slate-400">Tol ±Ry (Yaw)</span>
                  <span className="font-mono text-emerald-400">±{poiConfig.tolerance_rpy_deg[1]}°</span>
                </div>
                <input
                  type="range"
                  min={0.0}
                  max={45.0}
                  step={1.0}
                  value={poiConfig.tolerance_rpy_deg[1]}
                  onChange={(e) =>
                    setPoiConfig((prev) => ({
                      ...prev,
                      tolerance_rpy_deg: [prev.tolerance_rpy_deg[0], parseFloat(e.target.value), prev.tolerance_rpy_deg[2]],
                    }))
                  }
                  className="w-full accent-emerald-500 h-1 bg-slate-800 rounded"
                />
              </div>

              {/* Tol Rz */}
              <div className="space-y-0.5">
                <div className="flex justify-between text-[9.5px]">
                  <span className="text-slate-400">Tol ±Rz (Axial Spin)</span>
                  <span className="font-mono text-emerald-400">±{poiConfig.tolerance_rpy_deg[2]}°</span>
                </div>
                <input
                  type="range"
                  min={0.0}
                  max={180.0}
                  step={5.0}
                  value={poiConfig.tolerance_rpy_deg[2]}
                  onChange={(e) =>
                    setPoiConfig((prev) => ({
                      ...prev,
                      tolerance_rpy_deg: [prev.tolerance_rpy_deg[0], prev.tolerance_rpy_deg[1], parseFloat(e.target.value)],
                    }))
                  }
                  className="w-full accent-emerald-500 h-1 bg-slate-800 rounded"
                />
              </div>
            </div>

            {/* Actions: Apply, Reset, Cancel */}
            <div className="flex items-center gap-1.5 pt-1">
              <button
                onClick={() => onApplyOptimization('poi')}
                disabled={isOptimizing || !activeTemplate || rawPaths.length === 0}
                className="flex-1 py-1.5 px-2 rounded-lg bg-gradient-to-r from-emerald-600 to-teal-600 hover:from-emerald-500 hover:to-teal-500 text-white font-medium text-[10.5px] flex items-center justify-center gap-1 shadow-lg shadow-emerald-950/50 transition-all disabled:opacity-40"
              >
                <Compass size={12} className={isOptimizing ? 'animate-spin' : ''} />
                <span>{isOptimizing ? 'Optimizing POI...' : 'Apply & Optimize'}</span>
              </button>
              <button
                onClick={() => {
                  onFetchAnchorPose('home');
                  setPoiConfig({
                    ref_rpy_deg: [90.0, 0.0, 90.0],
                    tolerance_rpy_deg: [20.0, 20.0, 180.0],
                    anchor_source: 'home',
                  });
                }}
                className="px-2.5 py-1.5 rounded-lg border border-slate-700 bg-slate-800 hover:bg-slate-700 text-slate-300 text-[10px] font-medium transition-colors"
                title="Reset Defaults"
              >
                Reset
              </button>
              <button
                onClick={() => setSelectedTab('matrix')}
                className="px-2 py-1.5 rounded-lg border border-slate-800 hover:bg-slate-800 text-slate-400 text-[10px] transition-colors"
                title="Cancel"
              >
                Cancel
              </button>
            </div>
          </div>
        )}

        {/* MoveL Parameters Accordion */}
        <div className="border border-slate-800 rounded-lg overflow-hidden bg-slate-950/30">
          <button
            onClick={() => setIsKinParamsOpen(!isKinParamsOpen)}
            className="w-full p-2 flex items-center justify-between text-slate-300 hover:bg-slate-800/40 text-[10.5px] font-medium"
          >
            <div className="flex items-center gap-1.5">
              <HardDrive size={12} className="text-sky-400" />
              <span>Interpolation & URDF Config</span>
            </div>
            {isKinParamsOpen ? <ChevronRight size={11} className="rotate-90 transition-transform" /> : <ChevronRight size={11} />}
          </button>

          {isKinParamsOpen && (
            <div className="p-2.5 border-t border-slate-800 space-y-2 bg-slate-950/60 text-slate-300 text-[10px]">
              {/* URDF Tool Link Badge */}
              <div className="flex items-center justify-between text-[9px] font-mono bg-slate-900 p-1.5 rounded border border-slate-800">
                <span className="text-slate-400">Tool: {urdfTcpInfo?.tool_name || 'Flange'}</span>
                <span className="text-sky-400">{urdfTcpInfo?.urdf_source || 'cr5_robot.urdf'}</span>
              </div>

              {/* Step Size Slider */}
              <div className="space-y-0.5">
                <div className="flex justify-between text-[9.5px]">
                  <span className="text-slate-400">Step Size</span>
                  <span className="font-mono text-sky-400">{kinParams.stepSizeMm} mm</span>
                </div>
                <input
                  type="range"
                  min={0.5}
                  max={5.0}
                  step={0.5}
                  value={kinParams.stepSizeMm}
                  onChange={(e) =>
                    setKinParams((prev) => ({ ...prev, stepSizeMm: parseFloat(e.target.value) }))
                  }
                  className="w-full accent-sky-500 h-1 bg-slate-800 rounded"
                />
              </div>

              {/* Speed Slider */}
              <div className="space-y-0.5">
                <div className="flex justify-between text-[9.5px]">
                  <span className="text-slate-400">MoveL Nominal Speed</span>
                  <span className="font-mono text-sky-400">{kinParams.linearSpeedMmS} mm/s</span>
                </div>
                <input
                  type="range"
                  min={20}
                  max={300}
                  step={10}
                  value={kinParams.linearSpeedMmS}
                  onChange={(e) =>
                    setKinParams((prev) => ({ ...prev, linearSpeedMmS: parseInt(e.target.value) }))
                  }
                  className="w-full accent-sky-500 h-1 bg-slate-800 rounded"
                />
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
