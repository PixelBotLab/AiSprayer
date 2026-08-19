import React from 'react';
import Robot3DViewer from './Robot3DViewer';
import JogControlPanel from './JogControlPanel';

interface RobotState {
  pose: number[];
  joint: number[];
  status?: number;
  tcp_speed_actual?: number[];
  qd_actual?: number[];
  load?: number;
  error_status?: number;
  tool_vector_actual?: number[];
}

interface RobotZoneProps {
  robotState: RobotState;
  activeTemplate?: string | null;
  meshVersion?: number;
  pathsVersion?: number;
  pathState?: 'raw' | 'opt' | 'poi';
}

const formatNum = (val?: number, decimals: number = 1, threshold: number = 0.05): string => {
  if (val === undefined || val === null || Math.abs(val) < threshold) {
    return (0).toFixed(decimals);
  }
  const formatted = val.toFixed(decimals);
  return formatted === '-0.0' || formatted === '-0.00' || formatted.startsWith('-0.0') && parseFloat(formatted) === 0
    ? (0).toFixed(decimals)
    : formatted;
};

const formatTcpSpeed = (spd?: number[]) => {
  if (!spd || spd.length === 0) return '0.0 mm/s [0.0, 0.0, 0.0]';
  const vx = spd[0] || 0;
  const vy = spd[1] || 0;
  const vz = spd[2] || 0;
  const normLin = Math.sqrt(vx * vx + vy * vy + vz * vz);
  const mag = formatNum(normLin, 1, 0.05);
  const vec = `[${formatNum(vx, 1)}, ${formatNum(vy, 1)}, ${formatNum(vz, 1)}]`;
  return `${mag} mm/s ${vec}`;
};

const formatQdSpeed = (qd?: number[]) => {
  if (!qd || qd.length === 0) return '0.0 °/s [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]';
  const absMax = Math.max(...qd.map(Math.abs));
  const maxStr = formatNum(absMax, 1, 0.05);
  const list = qd.slice(0, 6).map(v => formatNum(v, 1, 0.05)).join(', ');
  return `${maxStr} °/s [${list}]`;
};

const RobotZone: React.FC<RobotZoneProps> = ({
  robotState,
  activeTemplate = null,
  meshVersion = 0,
  pathsVersion = 0,
  pathState = 'raw',
}) => {
  return (
    <div className="w-full h-full flex flex-col gap-3">
      {/* Robot 3D Viewer with Surface Mesh Overlay & 3D TCP Trajectories */}
      <div className="flex-1 min-h-0 bg-slate-900/80 rounded-xl border border-slate-800 shadow-lg flex flex-col shrink-0 p-1 relative overflow-hidden">
        <Robot3DViewer
          jointAngles={robotState.joint}
          activeTemplate={activeTemplate}
          meshVersion={meshVersion}
          pathsVersion={pathsVersion}
          pathState={pathState}
        />

        {/* Live Robot Speed HUD (Bottom-Left Ultra-Transparent HUD Overlay) */}
        <div className="absolute bottom-2.5 left-2.5 z-10 pointer-events-none flex flex-col gap-0.5 px-2 py-1 font-mono text-[10px] select-none drop-shadow-[0_1px_3px_rgba(0,0,0,0.95)]">
          {/* TCP Speed Row */}
          <div className="flex items-center gap-1.5">
            <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">TCP SPEED:</span>
            <span className="text-sky-300 font-bold tracking-tight">
              {formatTcpSpeed(robotState.tcp_speed_actual)}
            </span>
          </div>
          {/* Joint Speed Row */}
          <div className="flex items-center gap-1.5">
            <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">JNT SPEED:</span>
            <span className="text-emerald-300 font-bold tracking-tight">
              {formatQdSpeed(robotState.qd_actual)}
            </span>
          </div>
        </div>
      </div>

      {/* Jog Control Panel & Connection */}
      <div className="shrink-0">
        <JogControlPanel robotState={robotState} />
      </div>
    </div>
  );
};

export default RobotZone;
