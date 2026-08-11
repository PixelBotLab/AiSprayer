import React, { useState, useEffect, useRef } from 'react';
import { Camera, Crosshair, Cpu, Maximize, Play, Save, RefreshCw, GripHorizontal } from 'lucide-react';
import Robot3DViewer from '../components/Robot3DViewer';
import JogControlPanel from '../components/JogControlPanel';

interface RobotState {
  pose: number[];
  joint: number[];
}

const CalibView: React.FC = () => {
  const [samples, setSamples] = useState<any[]>([1, 2, 3, 4, 5, 6, 7]);
  const [resolution, setResolution] = useState<{width: number, height: number} | null>(null);
  const [robotState, setRobotState] = useState<RobotState>({ pose: [0,0,0,0,0,0], joint: [0,0,0,0,0,0] });
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
    return () => wsRef.current?.close();
  }, []);

  return (
    <div className="flex h-full w-full p-6 gap-6 bg-slate-950 text-slate-200 overflow-hidden">
      
      {/* Left side: Video & Thumbnails */}
      <div className="flex-[3] flex flex-col gap-5 min-w-0">
        
        {/* Video Stream */}
        <div className="flex-[3] flex flex-col bg-slate-900 rounded-xl border border-slate-800 shadow-xl overflow-hidden relative min-h-0">
          <div className="absolute top-0 left-0 right-0 px-4 py-3 flex justify-between items-center bg-gradient-to-b from-slate-950/90 to-transparent z-10 pointer-events-none">
            <h2 className="font-medium text-slate-100 flex items-center gap-2 text-sm drop-shadow-md">
              <Camera size={16} className="text-blue-400" />
              Live Stream
            </h2>
            <div className="flex gap-2 pointer-events-auto">
              <button className="p-1.5 bg-slate-800/80 hover:bg-slate-700 rounded text-slate-300 transition-colors backdrop-blur-sm border border-slate-700/50 shadow-sm">
                <Maximize size={14} />
              </button>
            </div>
          </div>
          
          <div className="flex-1 bg-black flex items-center justify-center relative overflow-hidden group">
            {/* Resolution Overlay */}
            {resolution && (
              <div className="absolute bottom-4 left-4 z-20 bg-slate-900/60 backdrop-blur-md px-3 py-1.5 rounded-lg border border-slate-700/50 text-xs font-mono text-emerald-400 shadow-lg pointer-events-none">
                {resolution.width} &times; {resolution.height}
              </div>
            )}

            {/* Real Camera Stream */}
            <img 
              src="http://localhost:8000/api/calib/camera/stream" 
              alt="Camera Stream" 
              className="absolute inset-0 w-full h-full object-contain z-0 transition-opacity duration-300"
              onError={(e) => {
                e.currentTarget.style.opacity = '0';
                setResolution(null);
                const fallback = document.getElementById('camera-fallback');
                if (fallback) fallback.style.display = 'flex';
              }}
              onLoad={(e) => {
                e.currentTarget.style.opacity = '1';
                setResolution({
                  width: e.currentTarget.naturalWidth,
                  height: e.currentTarget.naturalHeight
                });
                const fallback = document.getElementById('camera-fallback');
                if (fallback) fallback.style.display = 'none';
              }}
            />
            
            {/* Mock Camera View Overlay */}
            <div className="absolute inset-0 border-[1px] border-blue-500/10 m-4 rounded pointer-events-none z-10">
              <div className="absolute top-0 left-0 w-8 h-8 border-t-2 border-l-2 border-blue-500/50 rounded-tl"></div>
              <div className="absolute top-0 right-0 w-8 h-8 border-t-2 border-r-2 border-blue-500/50 rounded-tr"></div>
              <div className="absolute bottom-0 left-0 w-8 h-8 border-b-2 border-l-2 border-blue-500/50 rounded-bl"></div>
              <div className="absolute bottom-0 right-0 w-8 h-8 border-b-2 border-r-2 border-blue-500/50 rounded-br"></div>
              <Crosshair className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 text-blue-500/20 w-12 h-12" strokeWidth={1} />
            </div>
            
            <div id="camera-fallback" className="text-slate-500 flex flex-col items-center z-0 transition-opacity duration-300">
              <Camera size={48} strokeWidth={1} className="mb-3 opacity-20" />
              <p className="text-sm tracking-wide">Waiting for camera signal...</p>
            </div>
          </div>
        </div>

        {/* Calibration Controls & Samples */}
        <div className="flex-1 max-h-[260px] min-h-[200px] bg-slate-900/80 rounded-xl border border-slate-800 shadow-xl p-4 flex flex-col gap-4 backdrop-blur-sm">
          
          {/* Thumbnails Gallery */}
          <div className="flex-1 relative">
            <div className="absolute top-2 left-2 z-10 text-[10px] font-medium text-blue-400 bg-slate-900/90 backdrop-blur-sm border border-blue-500/30 px-2.5 py-0.5 rounded-full shadow-md">
              {samples.length} Samples
            </div>
            
            <div className="w-full h-full border border-slate-800 border-dashed rounded-lg bg-slate-950/50 flex items-center p-3 gap-3 overflow-x-auto overflow-y-hidden custom-scrollbar pt-8">
              {samples.length === 0 ? (
                <div className="w-full h-full flex flex-col items-center justify-center text-slate-500 text-sm">
                  <RefreshCw size={16} className="opacity-30 mb-2" />
                  No calibration samples
                </div>
              ) : (
                <div className="flex items-center gap-3 h-full">
                  {samples.map((s, idx) => (
                    <div key={idx} className="shrink-0 h-full aspect-video bg-slate-800 rounded-lg border border-slate-700 relative overflow-hidden group shadow-md hover:border-blue-500/50 transition-colors cursor-pointer">
                      <div className="absolute inset-0 flex items-center justify-center text-slate-500 text-xs font-medium">Sample {idx + 1}</div>
                      <div className="absolute bottom-0 inset-x-0 bg-gradient-to-t from-slate-900 to-transparent p-2 text-[10px] text-slate-300 translate-y-full group-hover:translate-y-0 transition-transform">
                        View Details
                      </div>
                    </div>
                  ))}
                </div>
              )}
            </div>
          </div>

          {/* Action Buttons */}
          <div className="flex gap-4 shrink-0">
            <button className="flex-1 py-3 bg-slate-800 hover:bg-slate-700 border border-slate-600 text-slate-200 font-medium rounded-lg shadow-sm transition-all flex items-center justify-center gap-2 text-sm group active:scale-[0.98]">
              <Save size={18} className="text-slate-400 group-hover:text-blue-400 transition-colors" />
              Capture Pose & Image
            </button>

            <button className="flex-1 py-3 bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-500 hover:to-indigo-500 text-white font-medium rounded-lg shadow-lg shadow-blue-900/20 transition-all flex items-center justify-center gap-2 active:scale-[0.98]">
              <Play size={18} fill="currentColor" />
              Execute Calibration
            </button>
          </div>
        </div>
      </div>

      {/* Right side: Controls */}
      <div className="flex-[2] flex flex-col gap-5 pr-2 custom-scrollbar">
        
        {/* Robot 3D Viewer */}
        <div className="flex-1 min-h-[300px] bg-slate-900/80 rounded-xl border border-slate-800 shadow-lg backdrop-blur-sm overflow-hidden flex flex-col shrink-0 p-1 relative">
          <Robot3DViewer jointAngles={robotState.joint} />
        </div>

        {/* Jog Control Panel & Connection */}
        <div className="shrink-0">
          <JogControlPanel robotState={robotState} />
        </div>

      </div>
    </div>
  );
};

export default CalibView;
