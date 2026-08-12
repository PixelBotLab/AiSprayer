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
}

const RobotZone: React.FC<RobotZoneProps> = ({ robotState }) => {
  return (
    <div className="w-full h-full flex flex-col gap-4">
      {/* Robot 3D Viewer */}
      <div className="flex-1 min-h-0 bg-slate-900/80 rounded-xl border border-slate-800 shadow-lg backdrop-blur-sm overflow-hidden flex flex-col shrink-0 p-1 relative">
        <Robot3DViewer jointAngles={robotState.joint} />
      </div>

      {/* Jog Control Panel & Connection */}
      <div className="shrink-0">
        <JogControlPanel robotState={robotState} />
      </div>
    </div>
  );
};

export default RobotZone;
