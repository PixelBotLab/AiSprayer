import React, { useState, useEffect, useRef } from 'react';
import FloatingCameraZone from '../components/FloatingCameraZone';
import RobotZone from '../components/RobotZone';
import ConsoleLogZone from '../components/ConsoleLogZone';
import CalibrationOp from '../components/operations/CalibrationOp';

interface RobotState {
  pose: number[];
  joint: number[];
  status?: number;
}

interface WorkspaceViewProps {
  activeTab: string;
}

const WorkspaceView: React.FC<WorkspaceViewProps> = ({ activeTab }) => {
  const [robotState, setRobotState] = useState<RobotState>({ pose: [0,0,0,0,0,0], joint: [0,0,0,0,0,0] });
  const [samples, setSamples] = useState<any[]>([]);
  const [isCapturing, setIsCapturing] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    const connect = () => {
      const ws = new WebSocket('ws://localhost:8000/api/calib/robot/ws');
      ws.onmessage = (e) => {
        try {
          const msg = JSON.parse(e.data);
          if (msg.type === 'robot_state') setRobotState(msg.data);
        } catch {}
      };
      ws.onclose = () => setTimeout(connect, 2000);
      wsRef.current = ws;
    };
    connect();

    const fetchSamples = async () => {
      try {
        const res = await fetch('http://localhost:8000/api/calib/samples');
        if (res.ok) {
          const data = await res.json();
          setSamples(data.samples || []);
        }
      } catch (err) {
        console.error('Failed to fetch samples:', err);
      }
    };
    fetchSamples();

    return () => wsRef.current?.close();
  }, []);

  const handleAddSample = async () => {
    if (isCapturing) return;
    setIsCapturing(true);
    try {
      const res = await fetch('http://localhost:8000/api/calib/samples/add', { method: 'POST' });
      if (res.ok) {
        const listRes = await fetch('http://localhost:8000/api/calib/samples');
        if (listRes.ok) {
          const data = await listRes.json();
          setSamples(data.samples || []);
        }
      } else {
        const err = await res.json();
        alert(`Failed to add sample: ${err.detail}`);
      }
    } catch (err: any) {
      alert(`Error capturing sample: ${err.message}`);
    } finally {
      setIsCapturing(false);
    }
  };

  const renderOperationZone = () => {
    switch (activeTab) {
      case 'calib':
        return <CalibrationOp samples={samples} isCapturing={isCapturing} onAddSample={handleAddSample} />;
      case 'interactive':
        return <div className="p-8 text-slate-400 bg-slate-900/50 rounded-xl border border-slate-800 w-full h-full flex items-center justify-center">2D Interactive Teach (Coming Soon)</div>;
      case 'auto_planner':
        return <div className="p-8 text-slate-400 bg-slate-900/50 rounded-xl border border-slate-800 w-full h-full flex items-center justify-center">3D Auto Planner (Coming Soon)</div>;
      case 'digital_twin':
        return <div className="p-8 text-slate-400 bg-slate-900/50 rounded-xl border border-slate-800 w-full h-full flex items-center justify-center">3D Digital Twin (Coming Soon)</div>;
      default:
        return null;
    }
  };

  return (
    <div className="w-full h-full p-6 bg-slate-950 text-slate-200 overflow-hidden flex gap-6 relative">
      {/* Floating Camera Window */}
      <FloatingCameraZone />
      
      {/* Left Column (60% width) - Operations & Logs */}
      <div className="flex-[3] flex flex-col gap-6 min-w-0 h-full z-0">
        
        {/* Top Left: Operation Zone (Dynamic based on Tab) */}
        <div className="flex-1 min-h-0 flex flex-col">
          {renderOperationZone()}
        </div>

        {/* Bottom Left: Console Log Zone */}
        <div className="h-[350px] shrink-0 min-h-0">
          <ConsoleLogZone />
        </div>
      </div>

      {/* Right Column (40% width) - Robot Zone Full Height */}
      <div className="flex-[2] flex flex-col gap-6 min-w-0 h-full z-0">
        {/* Full Right: Robot Zone */}
        <div className="flex-1 min-h-0">
          <RobotZone robotState={robotState} />
        </div>
      </div>
      
    </div>
  );
};

export default WorkspaceView;
