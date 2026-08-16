import React from 'react';
import Robot3DViewer from './Robot3DViewer';
import JogControlPanel from './JogControlPanel';

interface RobotState {
  pose: number[];
  joint: number[];
  status?: number;
}

interface RobotZoneProps {
  robotState: RobotState;
  activeTemplate?: string | null;
  meshVersion?: number;
  pathsVersion?: number;
  pathState?: 'raw' | 'opt' | 'poi';
}

const RobotZone: React.FC<RobotZoneProps> = ({ robotState, activeTemplate = null, meshVersion = 0, pathsVersion = 0, pathState = 'raw' }) => {
  return (
    <div className="w-full h-full flex flex-col gap-4">
      {/* Robot 3D Viewer with Surface Mesh Overlay & 3D TCP Trajectories */}
      <div className="flex-1 min-h-0 bg-slate-900/80 rounded-xl border border-slate-800 shadow-lg flex flex-col shrink-0 p-1 relative">
        <Robot3DViewer 
          jointAngles={robotState.joint} 
          activeTemplate={activeTemplate}
          meshVersion={meshVersion}
          pathsVersion={pathsVersion}
          pathState={pathState}
        />
      </div>

      {/* Jog Control Panel & Connection */}
      <div className="shrink-0">
        <JogControlPanel robotState={robotState} />
      </div>
    </div>
  );
};

export default RobotZone;
