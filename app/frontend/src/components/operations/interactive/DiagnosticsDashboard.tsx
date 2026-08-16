import React from 'react';
import {
  ShieldCheck,
  RefreshCw,
  Sparkles,
  Layers,
  HardDrive,
  Undo2,
  Check,
  ChevronRight,
} from 'lucide-react';
import type {
  VerificationReport,
  KinematicsParams,
  UrdfTcpInfo,
  ManualPathItem,
} from './types';

interface DiagnosticsDashboardProps {
  verificationReport: VerificationReport | null;
  isVerifying: boolean;
  isOptimizing: boolean;
  activeTemplate: string | null;
  optPaths: ManualPathItem[];
  usingOptimizedPaths: boolean;
  hasPaths: boolean;
  kinParams: KinematicsParams;
  urdfTcpInfo: UrdfTcpInfo | null;
  isKinParamsOpen: boolean;
  highlightedPathId: number | null;
  setKinParams: React.Dispatch<React.SetStateAction<KinematicsParams>>;
  setIsKinParamsOpen: React.Dispatch<React.SetStateAction<boolean>>;
  setHighlightedPathId: (id: number | null) => void;
  onRunDiagnostics: () => void;
  onApplyOptimization: () => void;
  onToggleUseOptimized: (useOpt: boolean) => void;
}

export const DiagnosticsDashboard: React.FC<DiagnosticsDashboardProps> = ({
  verificationReport,
  isVerifying,
  isOptimizing,
  activeTemplate,
  optPaths,
  usingOptimizedPaths,
  hasPaths,
  kinParams,
  urdfTcpInfo,
  isKinParamsOpen,
  highlightedPathId,
  setKinParams,
  setIsKinParamsOpen,
  setHighlightedPathId,
  onRunDiagnostics,
  onApplyOptimization,
  onToggleUseOptimized,
}) => {
  return (
    <div className="h-full flex flex-col bg-slate-900 overflow-y-auto">
      {/* Header Bar */}
      <div className="p-3 border-b border-slate-800 bg-slate-950/60 sticky top-0 z-10 flex items-center justify-between">
        <div className="flex items-center gap-2">
          <ShieldCheck size={16} className="text-sky-400" />
          <span className="text-xs font-bold text-slate-200 tracking-wider">
            TCP KINEMATICS & VERIFICATION
          </span>
        </div>
        <div className="flex items-center gap-1.5">
          <button
            onClick={onRunDiagnostics}
            disabled={isVerifying || !activeTemplate}
            className="px-2.5 py-1 text-[11px] font-medium rounded bg-slate-800 hover:bg-slate-700 text-sky-300 border border-sky-500/40 flex items-center gap-1 shadow transition-all disabled:opacity-40"
          >
            <RefreshCw size={11} className={isVerifying ? 'animate-spin' : ''} />
            <span>{isVerifying ? 'Verifying...' : 'Re-verify'}</span>
          </button>
        </div>
      </div>

      <div className="p-4 space-y-4 text-xs">
        {/* Verification Summary Banner */}
        {verificationReport ? (
          <div
            className={`p-3 rounded-lg border flex flex-col gap-2 ${
              verificationReport.summary.status === 'PASS'
                ? 'bg-emerald-950/30 border-emerald-500/40 text-emerald-300'
                : verificationReport.summary.status === 'WARNING'
                ? 'bg-amber-950/30 border-amber-500/40 text-amber-300'
                : 'bg-rose-950/30 border-rose-500/40 text-rose-300'
            }`}
          >
            <div className="flex items-center justify-between">
              <span className="font-bold flex items-center gap-1.5 text-xs">
                {verificationReport.summary.status === 'PASS' && '✅ All Paths Feasible'}
                {verificationReport.summary.status === 'WARNING' && '⚠️ Minor Overspeed Warning'}
                {verificationReport.summary.status === 'FAILED' && '❌ Kinematic Issue Detected'}
              </span>
              <span className="text-[10px] px-1.5 py-0.5 rounded font-mono bg-black/40 border border-white/10 uppercase">
                {verificationReport.summary.status}
              </span>
            </div>
            <div className="grid grid-cols-3 gap-2 text-[10px] font-mono text-slate-300 bg-black/20 p-2 rounded">
              <div>Paths: {verificationReport.summary.total_paths}</div>
              <div>Waypoints: {verificationReport.summary.total_waypoints}</div>
              <div>MoveL Steps: {verificationReport.summary.total_steps}</div>
            </div>
          </div>
        ) : (
          <div className="p-4 rounded-lg border border-dashed border-slate-800 text-center text-slate-500">
            No verification report generated yet. Click "Re-verify" to simulate dense MoveL interpolation.
          </div>
        )}

        {/* Optimized Trajectory Switcher (if available) */}
        {optPaths.length > 0 && (
          <div className="p-2.5 rounded-lg bg-sky-950/20 border border-sky-500/30 flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Sparkles size={14} className="text-sky-400" />
              <div className="text-[11px]">
                <div className="font-medium text-slate-200">
                  {usingOptimizedPaths ? 'Using Optimized Trajectory' : 'Optimized Trajectory Available'}
                </div>
                <div className="text-[9px] text-slate-400">
                  Tolerance-based continuous orientation auto-fixed
                </div>
              </div>
            </div>
            <button
              onClick={() => onToggleUseOptimized(!usingOptimizedPaths)}
              className={`px-2.5 py-1 text-[10px] font-medium rounded transition-all flex items-center gap-1 ${
                usingOptimizedPaths
                  ? 'bg-sky-600 text-white shadow-lg shadow-sky-900/50 hover:bg-sky-500'
                  : 'bg-slate-800 text-sky-300 border border-sky-500/40 hover:bg-slate-700'
              }`}
            >
              {usingOptimizedPaths ? <Undo2 size={11} /> : <Check size={11} />}
              <span>{usingOptimizedPaths ? 'Revert to Raw' : 'Apply Opt'}</span>
            </button>
          </div>
        )}

        {/* Per-Path Diagnostic Breakdown & Peak Joint Speeds */}
        {verificationReport?.path_reports && verificationReport.path_reports.length > 0 && (
          <div className="space-y-2.5">
            <div className="text-[11px] font-bold text-slate-300 flex items-center gap-1.5">
              <Layers size={13} className="text-sky-400" />
              <span>Path Feasibility & Joint Velocity</span>
            </div>
            {verificationReport.path_reports.map((pRep, idx) => {
              const isSelected = highlightedPathId === pRep.path_id;
              const maxVel = verificationReport.max_joint_velocities_deg_s || [180, 180, 180, 180, 180, 180];
              return (
                <div
                  key={idx}
                  onClick={() => setHighlightedPathId(isSelected ? null : pRep.path_id)}
                  className={`p-3 rounded-lg border transition-all cursor-pointer ${
                    isSelected
                      ? 'bg-slate-800/90 border-sky-500 shadow-md shadow-sky-950/40'
                      : 'bg-slate-950/40 hover:bg-slate-800/50 border-slate-800'
                  }`}
                >
                  <div className="flex items-center justify-between mb-2">
                    <div className="flex items-center gap-1.5">
                      <span className="font-mono font-bold text-xs text-sky-300">
                        P{pRep.path_id}
                      </span>
                      <span className="text-[10px] text-slate-400 font-mono">
                        ({pRep.total_interpolated} steps)
                      </span>
                    </div>
                    <span
                      className={`text-[9px] px-1.5 py-0.5 rounded font-mono font-bold uppercase ${
                        pRep.status === 'PASS'
                          ? 'bg-emerald-950/60 text-emerald-400 border border-emerald-500/30'
                          : pRep.status === 'WARNING'
                          ? 'bg-amber-950/60 text-amber-400 border border-amber-500/30'
                          : 'bg-rose-950/60 text-rose-400 border border-rose-500/30'
                      }`}
                    >
                      {pRep.status}
                    </span>
                  </div>

                  {/* Joint Speed Progress Bars */}
                  {pRep.peak_joint_speeds_deg_s && (
                    <div className="space-y-1.5">
                      <div className="text-[9px] text-slate-400 font-mono flex justify-between">
                        <span>Peak Joint Velocity (J1..J6)</span>
                        <span>Max Allowable</span>
                      </div>
                      <div className="grid grid-cols-6 gap-1">
                        {pRep.peak_joint_speeds_deg_s.map((spd, jIdx) => {
                          const limit = maxVel[jIdx] || 180;
                          const ratio = Math.min(1.0, spd / limit);
                          const isHigh = ratio > 0.85;
                          const isOver = spd > limit;
                          return (
                            <div key={jIdx} className="flex flex-col gap-0.5">
                              <div className="h-1.5 bg-slate-800 rounded-full overflow-hidden flex">
                                <div
                                  className={`h-full rounded-full transition-all ${
                                    isOver
                                      ? 'bg-rose-500'
                                      : isHigh
                                      ? 'bg-amber-400'
                                      : 'bg-sky-400'
                                  }`}
                                  style={{ width: `${ratio * 100}%` }}
                                />
                              </div>
                              <span className="text-[8px] font-mono text-center text-slate-400">
                                {Math.round(spd)}°
                              </span>
                            </div>
                          );
                        })}
                      </div>
                    </div>
                  )}

                  {/* Issues List on this Path */}
                  {pRep.issues && pRep.issues.length > 0 && (
                    <div className="mt-2.5 pt-2 border-t border-slate-800/80 space-y-1">
                      {pRep.issues.map((iss, iIdx) => (
                        <div
                          key={iIdx}
                          className="text-[9.5px] font-mono text-rose-300 flex items-start gap-1 bg-rose-950/20 p-1 rounded"
                        >
                          <span>⚠️</span>
                          <span>
                            {iss.type}: {iss.message}
                          </span>
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              );
            })}
          </div>
        )}

        {/* Verification Kinematics Parameters Accordion */}
        <div className="border border-slate-800 rounded-lg overflow-hidden bg-slate-950/30">
          <button
            onClick={() => setIsKinParamsOpen(!isKinParamsOpen)}
            className="w-full p-2.5 flex items-center justify-between text-slate-300 hover:bg-slate-800/40 text-[11px] font-medium"
          >
            <div className="flex items-center gap-1.5">
              <HardDrive size={13} className="text-sky-400" />
              <span>MoveL & Tool Parameters</span>
            </div>
            {isKinParamsOpen ? <ChevronRight size={12} className="rotate-90 transition-transform" /> : <ChevronRight size={12} />}
          </button>

          {isKinParamsOpen && (
            <div className="p-3 border-t border-slate-800 space-y-3 bg-slate-950/60 text-slate-300">
              {/* URDF Tool TCP Status Card */}
              <div className="p-2.5 rounded-lg bg-slate-900 border border-slate-800 space-y-1.5">
                <div className="flex items-center justify-between">
                  <span className="text-[10px] font-bold text-slate-300 tracking-wider">
                    ROBOT URDF TOOL TCP
                  </span>
                  <span className="text-[9px] font-mono text-sky-400 bg-sky-950/60 px-1.5 py-0.5 rounded border border-sky-500/30">
                    {urdfTcpInfo?.urdf_source || 'cr5_robot.urdf'}
                  </span>
                </div>
                {urdfTcpInfo && urdfTcpInfo.has_tool ? (
                  <div className="space-y-1 text-[10px] font-mono">
                    <div className="text-slate-400">
                      Tool Link: <span className="text-emerald-400 font-bold">{urdfTcpInfo.tool_name}</span>
                    </div>
                    <div className="text-slate-400">
                      XYZ (mm): <span className="text-slate-200">[{urdfTcpInfo.xyz_mm.map(v => (v >= 0 ? `+${v}` : `${v}`)).join(', ')}]</span>
                    </div>
                    <div className="text-slate-400">
                      RPY (deg): <span className="text-amber-300">[{urdfTcpInfo.rpy_deg.map(v => (v >= 0 ? `+${v}` : `${v}`)).join('°, ')}°]</span>
                    </div>
                  </div>
                ) : (
                  <div className="text-[10px] font-mono text-slate-400">
                    Tool Link: <span className="text-slate-300">Flange (Default)</span> [0, 0, 0] mm
                  </div>
                )}
              </div>

              {/* Step Size Slider */}
              <div className="space-y-1">
                <div className="flex justify-between text-[10px]">
                  <span className="text-slate-400">Interpolation Step</span>
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
                  className="w-full accent-sky-500"
                />
              </div>

              {/* Speed Slider */}
              <div className="space-y-1">
                <div className="flex justify-between text-[10px]">
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
                  className="w-full accent-sky-500"
                />
              </div>
            </div>
          )}
        </div>

        {/* Axial Tolerance Auto-Fix Trigger */}
        <button
          onClick={onApplyOptimization}
          disabled={isOptimizing || !activeTemplate || !hasPaths}
          className="w-full py-2 px-3 text-xs font-medium rounded-lg bg-gradient-to-r from-sky-600 to-indigo-600 hover:from-sky-500 hover:to-indigo-500 text-white shadow-lg shadow-sky-950/50 flex items-center justify-center gap-1.5 transition-all disabled:opacity-40"
        >
          <Sparkles size={14} className={isOptimizing ? 'animate-spin' : ''} />
          <span>{isOptimizing ? 'Optimizing Trajectory...' : 'Auto-Fix Path Orientations'}</span>
        </button>
      </div>
    </div>
  );
};
