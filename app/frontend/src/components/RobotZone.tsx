import React from 'react';
import Robot3DViewer from './Robot3DViewer';
import JogControlPanel from './JogControlPanel';

interface RobotState {
  pose: number[];
  joint: number[];
  status?: number;
  tcp_speed_actual?: number[];
  tcp_speed_mm_s?: number;
  tcp_speed_actual_mm_s?: number[];
  qd_actual?: number[];
  load?: number;
  error_status?: number;
  tool_vector_actual?: number[];
  hand_type?: number[];    // int8 x4: 手系配置
  tool_index?: number;     // 当前工具坐标系索引
  run_queued_cmd?: number; // 算法队列当前执行段序号
  velocity_ratio?: number;     // 1016 关节速度比例 (%)
  xyz_velocity_ratio?: number; // 1019 笛卡尔位置速度比例 (%)
  r_velocity_ratio?: number;   // 1020 笛卡尔姿态速度比例 (%)
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

// spd: raw 6-component TCP velocity from backend in m/s → convert to mm/s for display
// scalar: pre-computed |Vlin| in mm/s from backend
const formatTcpSpeed = (spd?: number[], scalar?: number): { mag: string; vec: string } => {
  // Convert all 6 components from m/s → mm/s
  const c = (spd || [0, 0, 0, 0, 0, 0]).map(v => (v || 0) * 1000);
  const [vx, vy, vz, rx, ry, rz] = [c[0], c[1], c[2], c[3], c[4], c[5]];
  const mag = scalar !== undefined
    ? formatNum(scalar, 1, 0.05)
    : formatNum(Math.sqrt(vx * vx + vy * vy + vz * vz), 1, 0.05);
  const linVec = `[${formatNum(vx, 2, 0.005)}, ${formatNum(vy, 2, 0.005)}, ${formatNum(vz, 2, 0.005)}]`;
  const rotVec = `[${formatNum(rx, 2, 0.005)}, ${formatNum(ry, 2, 0.005)}, ${formatNum(rz, 2, 0.005)}]`;
  return { mag, vec: `${linVec} | ${rotVec}` };
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
          {(() => {
            const { mag, vec } = formatTcpSpeed(robotState.tcp_speed_actual, robotState.tcp_speed_mm_s);
            return (
              <>
                <div className="flex items-center gap-1.5">
                  <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">TCP SPEED:</span>
                  <span className="text-sky-300 font-bold tracking-tight">{mag} mm/s</span>
                </div>
                <div className="flex items-center gap-1.5 ml-1">
                  <span className="text-slate-400/70 text-[8px] uppercase tracking-wider shrink-0">XYZ|RPY:</span>
                  <span className="text-sky-200/80 tracking-tight text-[8.5px]">{vec}</span>
                </div>
              </>
            );
          })()}
          {/* Joint Speed Row */}
          <div className="flex items-center gap-1.5">
            <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">JNT SPEED:</span>
            <span className="text-emerald-300 font-bold tracking-tight">
              {formatQdSpeed(robotState.qd_actual)}
            </span>
          </div>
          {/* Hand Type & Tool Index Row */}
          <div className="flex items-center gap-2">
            <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">TOOL:</span>
            <span className="text-amber-300 font-bold tracking-tight">
              #{robotState.tool_index ?? 0}
            </span>
            <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0 ml-1">HAND:</span>
            <span className="text-amber-200/80 tracking-tight text-[8.5px]">
              [{(robotState.hand_type ?? [0,0,0,0]).join(', ')}]
            </span>
          </div>
          {/* Speed Ratio Row: J% / XYZ% / R% */}
          <div className="flex items-center gap-1.5">
            <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">SPD%:</span>
            <span className="text-lime-300 font-bold tracking-tight">
              J:{robotState.velocity_ratio ?? 0}%
            </span>
            <span className="text-slate-500/80 text-[8px]">|</span>
            <span className="text-lime-200/80 tracking-tight text-[8.5px]">
              XYZ:{robotState.xyz_velocity_ratio ?? 0}%
            </span>
            <span className="text-slate-500/80 text-[8px]">|</span>
            <span className="text-lime-200/80 tracking-tight text-[8.5px]">
              R:{robotState.r_velocity_ratio ?? 0}%
            </span>
          </div>
          {/* Queue Progress Row — only shown when robot is moving */}
          {(robotState.status === 1) && (
            <div className="flex items-center gap-1.5">
              <span className="text-slate-300/80 text-[8.5px] uppercase tracking-wider font-semibold shrink-0">QUEUE SEG:</span>
              <span className="text-violet-300 font-bold tracking-tight">
                {robotState.run_queued_cmd ?? 0}
              </span>
            </div>
          )}
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
