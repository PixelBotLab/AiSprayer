import React, { useEffect, useState, useRef } from 'react';
import { Cpu, RotateCcw, Crosshair, Home, Pause, Play, AlertOctagon } from 'lucide-react';

interface RobotState {
  pose: number[];
  joint: number[];
  status?: number;
}

interface JogControlPanelProps {
  robotState?: RobotState;
}

const JogControlPanel: React.FC<JogControlPanelProps> = ({ robotState }) => {
  const [mode, setMode] = useState<'cartesian' | 'joint'>('cartesian');
  const [xyzStep, setXyzStep] = useState<number>(5.0);
  const [angStep, setAngStep] = useState<number>(1.0);
  const [speedL, setSpeedL] = useState<number>(20);
  const [accL, setAccL] = useState<number>(20);
  const [speedJ, setSpeedJ] = useState<number>(20);
  const [accJ, setAccJ] = useState<number>(20);

  // Use prop robotState if provided, else fall back to internal zeros
  const displayState: RobotState = robotState ?? { pose: [0,0,0,0,0,0], joint: [0,0,0,0,0,0] };
  const [wsConnected, setWsConnected] = useState<boolean>(false);
  const [robotConnected, setRobotConnected] = useState<boolean>(false);
  const [connecting, setConnecting] = useState<boolean>(false);
  const [activeAction, setActiveAction] = useState<'home' | 'zero' | null>(null);
  const wsRef = useRef<WebSocket | null>(null);

  const isMoving = displayState.status === 1;
  const disableMotion = !robotConnected || isMoving;

  useEffect(() => {
    if (!isMoving) {
      setActiveAction(null);
    }
  }, [isMoving]);

  useEffect(() => {
    connectWs();
    return () => {
      if (wsRef.current) wsRef.current.close();
    };
  }, []);

  const connectWs = () => {
    const ws = new WebSocket('ws://localhost:8000/api/calib/robot/ws');
    ws.onopen = () => setWsConnected(true);
    ws.onclose = () => {
      setWsConnected(false);
      setTimeout(connectWs, 2000); // Reconnect WebSocket
    };
    ws.onmessage = (_e) => {
      // Robot state is now handled at CalibView level and passed as a prop
    };
    wsRef.current = ws;
  };

  const fetchSpeed = async () => {
    try {
      const spdRes = await fetch('http://localhost:8000/api/calib/robot/speed');
      if (spdRes.ok) {
        const spdData = await spdRes.json();
        setSpeedL(spdData.speed_l || 20);
        setAccL(spdData.acc_l || 20);
        setSpeedJ(spdData.speed_j || 20);
        setAccJ(spdData.acc_j || 20);
      }
    } catch (e) {
      console.error("Failed to fetch initial speed", e);
    }
  };

  const handleConnect = async () => {
    if (connecting) return;
    setConnecting(true);
    
    if (robotConnected) {
      // Disconnect
      try {
        const res = await fetch('http://localhost:8000/api/calib/robot/disconnect', { method: 'POST' });
        if (res.ok) {
          setRobotConnected(false);
        }
      } catch (err) {
        console.error('Disconnect error:', err);
      }
    } else {
      // Connect
      try {
        const res = await fetch('http://localhost:8000/api/calib/robot/connect', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ robot_type: 'dobot' })
        });
        if (res.ok) {
          setRobotConnected(true);
          // Fetch current global speed
          fetchSpeed();
        } else {
          alert('Failed to connect to robot.');
        }
      } catch (err: any) {
        alert(`Connection error: ${err.message}`);
        console.error('Failed to connect:', err);
      }
    }
    setConnecting(false);
  };

  const handleSpeedUpdate = async () => {
    try {
      await fetch('http://localhost:8000/api/calib/robot/speed', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed_l: speedL, acc_l: accL, speed_j: speedJ, acc_j: accJ })
      });
    } catch (err) {
      console.error('Failed to sync speed:', err);
    }
  };

  const handleJog = async (axis: string, direction: number) => {
    const isXyz = ['X', 'Y', 'Z'].includes(axis);
    const stepSize = isXyz ? xyzStep : angStep;
    try {
      const res = await fetch('http://localhost:8000/api/calib/robot/jog', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ axis, direction, step: stepSize, speed_l: speedL, acc_l: accL, speed_j: speedJ, acc_j: accJ })
      });
      if (!res.ok) {
        const error = await res.json();
        alert(`Jog failed: ${error.detail || 'Unknown error'}`);
      }
    } catch (err: any) {
      alert(`Jog error: ${err.message}`);
      console.error(err);
    }
  };

  const handleZero = async () => {
    setActiveAction('zero');
    try {
      const res = await fetch('http://localhost:8000/api/calib/robot/zero', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed: speedJ, acc: accJ })
      });
      if (!res.ok) {
        const error = await res.json();
        alert(`Zero failed: ${error.detail || 'Unknown error'}`);
      }
    } catch (err: any) {
      alert(`Zero error: ${err.message}`);
      console.error('Failed to go zero:', err);
    }
  };

  const handleHome = async () => {
    setActiveAction('home');
    try {
      const res = await fetch('http://localhost:8000/api/calib/robot/home', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed: speedJ, acc: accJ })
      });
      if (!res.ok) {
        const error = await res.json();
        alert(`Home failed: ${error.detail || 'Unknown error'}`);
      }
    } catch (err: any) {
      alert(`Home error: ${err.message}`);
      console.error('Failed to go home:', err);
    }
  };

  const handlePause = async () => {
    try {
      await fetch('http://localhost:8000/api/calib/robot/pause', { method: 'POST' });
    } catch (err: any) { console.error('Pause error:', err); }
  };

  const handleResume = async () => {
    try {
      await fetch('http://localhost:8000/api/calib/robot/resume', { method: 'POST' });
    } catch (err: any) { console.error('Resume error:', err); }
  };

  const handleEstop = async () => {
    try {
      await fetch('http://localhost:8000/api/calib/robot/estop', { method: 'POST' });
    } catch (err: any) {
      alert(`E-Stop triggered!`);
      console.error('E-Stop error:', err);
    }
  };

  const cartesianAxes = [
    { name: 'X', unit: 'mm' }, { name: 'Y', unit: 'mm' }, { name: 'Z', unit: 'mm' },
    { name: 'Rx', unit: '°' }, { name: 'Ry', unit: '°' }, { name: 'Rz', unit: '°' }
  ];
  
  const jointAxes = [
    { name: 'J1', unit: '°' }, { name: 'J2', unit: '°' }, { name: 'J3', unit: '°' },
    { name: 'J4', unit: '°' }, { name: 'J5', unit: '°' }, { name: 'J6', unit: '°' }
  ];

  const currentAxes = mode === 'cartesian' ? cartesianAxes : jointAxes;
  const currentValues = mode === 'cartesian' ? displayState.pose : displayState.joint;

  const formatValue = (idx: number) => {
    let val = currentValues[idx];
    if (val === undefined) return '0.00';
    if (mode === 'cartesian' && idx >= 3) {
       // Backend sends Rx/Ry/Rz in radians, convert to degrees for display
       val = val * (180 / Math.PI);
    }
    if (Math.abs(val) < 0.005) {
      val = 0.0;
    }
    return val.toFixed(2);
  };

  return (
    <div className="bg-slate-900/80 rounded-xl border border-slate-800 p-5 shadow-lg backdrop-blur-sm shrink-0 w-full flex flex-col gap-4 relative">
      {/* Jog Controls Toolbar */}
      <div className="flex justify-between items-center mt-2">
        {/* Column 1: Mode Toggle */}
        <div className="flex flex-col gap-1.5 w-[110px] relative">
          {/* Status LED Indicator */}
          <div className="absolute -top-5 left-0 flex items-center gap-1.5 pointer-events-none">
            <div className={`w-2 h-2 rounded-full transition-colors duration-300 ${!robotConnected ? 'bg-slate-600' : isMoving ? 'bg-amber-400 animate-pulse shadow-[0_0_8px_rgba(251,191,36,0.6)]' : 'bg-emerald-500 shadow-[0_0_8px_rgba(16,185,129,0.6)]'}`}></div>
            <span className="text-[9px] font-medium uppercase tracking-wider text-slate-400">
              {!robotConnected ? 'Offline' : isMoving ? 'Moving' : 'Idle'}
            </span>
          </div>

          <button 
            onClick={() => setMode('cartesian')}
            className={`px-3 h-8 text-xs font-medium rounded-md flex items-center justify-center gap-1.5 transition-colors border ${mode === 'cartesian' ? 'bg-slate-800 text-blue-400 border-slate-700/50' : 'bg-slate-950 text-slate-500 border-slate-800 hover:text-slate-300'}`}
          >
            <Crosshair size={12} /> Cartesian
          </button>
          <button 
            onClick={() => setMode('joint')}
            className={`px-3 h-8 text-xs font-medium rounded-md flex items-center justify-center gap-1.5 transition-colors border ${mode === 'joint' ? 'bg-slate-800 text-blue-400 border-slate-700/50' : 'bg-slate-950 text-slate-500 border-slate-800 hover:text-slate-300'}`}
          >
            <RotateCcw size={12} /> Joint
          </button>
        </div>
        {/* Column 2: Action Buttons (3x2 Grid) */}
        <div className="grid grid-cols-3 grid-rows-2 gap-1.5 flex-1 max-w-[320px]">
          {/* Row 1 */}
          <button 
            onClick={handleConnect}
            disabled={connecting}
            className={`px-2 h-8 text-xs font-medium rounded-md flex items-center justify-center gap-1.5 transition-colors border disabled:opacity-50 ${robotConnected ? 'bg-green-600/20 text-green-500 hover:bg-green-600 hover:text-white border-green-600/30' : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50'}`}
          >
            <Cpu size={12} className="shrink-0" /> <span className="truncate">{connecting ? '...' : robotConnected ? 'Disconnect' : 'Connect'}</span>
          </button>
          <button
            onClick={handleHome}
            disabled={disableMotion}
            className={`px-2 h-8 text-xs font-medium rounded-md flex items-center justify-center gap-1.5 transition-colors border ${
              activeAction === 'home'
                ? 'bg-emerald-600/20 text-emerald-400 border-emerald-500/50 animate-pulse shadow-[0_0_8px_rgba(16,185,129,0.5)] cursor-not-allowed'
                : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50 disabled:opacity-50 disabled:cursor-not-allowed'
            }`}
          >
            <Home size={12} className="shrink-0" /> <span>Home</span>
          </button>
          <button
            onClick={handleZero}
            disabled={disableMotion}
            className={`px-2 h-8 text-xs font-medium rounded-md flex items-center justify-center gap-1.5 transition-colors border ${
              activeAction === 'zero'
                ? 'bg-emerald-600/20 text-emerald-400 border-emerald-500/50 animate-pulse shadow-[0_0_8px_rgba(16,185,129,0.5)] cursor-not-allowed'
                : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50 disabled:opacity-50 disabled:cursor-not-allowed'
            }`}
          >
            <Crosshair size={12} className="shrink-0" /> <span>Zero</span>
          </button>

          {/* Row 2 */}
          <button 
            onClick={handlePause}
            className="px-2 h-8 text-xs font-medium rounded-md flex items-center justify-center gap-1.5 transition-colors bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border border-slate-700/50"
          >
            <Pause size={12} className="shrink-0" /> <span>Pause</span>
          </button>
          <button 
            onClick={handleResume}
            className="px-2 h-8 text-xs font-medium rounded-md flex items-center justify-center gap-1.5 transition-colors bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border border-slate-700/50"
          >
            <Play size={12} className="shrink-0" /> <span>Resume</span>
          </button>
          <button 
            onClick={handleEstop}
            className="px-2 h-8 text-xs font-bold rounded-md flex items-center justify-center gap-1.5 transition-colors bg-red-600/20 text-red-500 hover:bg-red-600 hover:text-white border border-red-600/30"
          >
            <AlertOctagon size={12} className="shrink-0" /> <span>Stop</span>
          </button>
        </div>

        <div className="flex flex-col gap-1.5 w-[140px]">
          <div className="flex items-center gap-2 bg-slate-950/50 px-2 h-8 rounded-md border border-slate-800/50">
            <span className="text-[9px] text-slate-500 font-medium w-6">XYZ:</span>
            <div className="flex gap-1 flex-1">
              {[1.0, 5.0, 10.0, 50.0].map(step => (
                <button 
                  key={step}
                  onClick={() => setXyzStep(step)}
                  className={`flex-1 h-5 text-[9px] rounded border transition-colors ${xyzStep === step ? 'bg-blue-600 border-blue-500 text-white' : 'bg-slate-800 border-slate-700 text-slate-400 hover:bg-slate-700 hover:text-slate-200'}`}
                >
                  {step}
                </button>
              ))}
            </div>
          </div>
          <div className="flex items-center gap-2 bg-slate-950/50 px-2 h-8 rounded-md border border-slate-800/50">
            <span className="text-[9px] text-slate-500 font-medium w-6">ANG:</span>
            <div className="flex gap-1 flex-1">
              {[0.1, 1.0, 5.0, 10.0].map(step => (
                <button 
                  key={step}
                  onClick={() => setAngStep(step)}
                  className={`flex-1 h-5 text-[9px] rounded border transition-colors ${angStep === step ? 'bg-blue-600 border-blue-500 text-white' : 'bg-slate-800 border-slate-700 text-slate-400 hover:bg-slate-700 hover:text-slate-200'}`}
                >
                  {step}
                </button>
              ))}
            </div>
          </div>
        </div>
      </div>

      {/* Speed & Acceleration Sliders */}
      <div className="flex flex-col gap-2 mb-2 bg-slate-950/30 p-2 rounded-lg border border-slate-800/30">
        
        {/* Linear Speed/Acc */}
        <div className="flex gap-4">
          <div className="flex-1 flex flex-col gap-1">
            <div className="flex justify-between text-[10px] text-slate-400 font-medium">
              <span>LIN SPEED</span>
              <span className="text-blue-400">{speedL}%</span>
            </div>
            <input 
              type="range" 
              min="1" max="100" 
              value={speedL} 
              onChange={(e) => setSpeedL(Number(e.target.value))}
              onPointerUp={handleSpeedUpdate}
              onKeyUp={handleSpeedUpdate}
              className="w-full h-1.5 bg-slate-800 rounded-lg appearance-none cursor-pointer accent-blue-500"
            />
          </div>
          <div className="flex-1 flex flex-col gap-1">
            <div className="flex justify-between text-[10px] text-slate-400 font-medium">
              <span>LIN ACCEL</span>
              <span className="text-blue-400">{accL}%</span>
            </div>
            <input 
              type="range" 
              min="1" max="100" 
              value={accL} 
              onChange={(e) => setAccL(Number(e.target.value))}
              onPointerUp={handleSpeedUpdate}
              onKeyUp={handleSpeedUpdate}
              className="w-full h-1.5 bg-slate-800 rounded-lg appearance-none cursor-pointer accent-blue-500"
            />
          </div>
        </div>

        {/* Joint Speed/Acc */}
        <div className="flex gap-4">
          <div className="flex-1 flex flex-col gap-1">
            <div className="flex justify-between text-[10px] text-slate-400 font-medium">
              <span>JNT SPEED</span>
              <span className="text-green-400">{speedJ}%</span>
            </div>
            <input 
              type="range" 
              min="1" max="100" 
              value={speedJ} 
              onChange={(e) => setSpeedJ(Number(e.target.value))}
              onPointerUp={handleSpeedUpdate}
              onKeyUp={handleSpeedUpdate}
              className="w-full h-1.5 bg-slate-800 rounded-lg appearance-none cursor-pointer accent-green-500"
            />
          </div>
          <div className="flex-1 flex flex-col gap-1">
            <div className="flex justify-between text-[10px] text-slate-400 font-medium">
              <span>JNT ACCEL</span>
              <span className="text-green-400">{accJ}%</span>
            </div>
            <input 
              type="range" 
              min="1" max="100" 
              value={accJ} 
              onChange={(e) => setAccJ(Number(e.target.value))}
              onPointerUp={handleSpeedUpdate}
              onKeyUp={handleSpeedUpdate}
              className="w-full h-1.5 bg-slate-800 rounded-lg appearance-none cursor-pointer accent-green-500"
            />
          </div>
        </div>
      </div>

      {/* Axis Rows */}
      <div className="flex flex-col gap-2">
          {currentAxes.map((axis, idx) => (
            <div key={axis.name} className="flex items-center gap-3">
              <button 
                onMouseDown={() => handleJog(axis.name, -1)}
                disabled={disableMotion}
                className="w-10 h-8 flex items-center justify-center bg-slate-800 hover:bg-slate-700 active:bg-blue-600 border border-slate-700 rounded text-slate-300 hover:text-white transition-colors disabled:opacity-50 disabled:cursor-not-allowed disabled:active:bg-slate-800"
              >
                -
              </button>
              
              <div className="flex-1 flex items-center justify-between bg-slate-950 border border-slate-800 rounded px-3 h-8">
                <span className="text-xs font-bold text-slate-500 w-6">{axis.name}</span>
                <span className="text-sm font-mono text-emerald-400 tracking-wider">
                  {formatValue(idx)}
                </span>
                <span className="text-[10px] text-slate-600 w-4">{axis.unit}</span>
              </div>

              <button 
                onMouseDown={() => handleJog(axis.name, 1)}
                disabled={disableMotion}
                className="w-10 h-8 flex items-center justify-center bg-slate-800 hover:bg-slate-700 active:bg-blue-600 border border-slate-700 rounded text-slate-300 hover:text-white transition-colors disabled:opacity-50 disabled:cursor-not-allowed disabled:active:bg-slate-800"
              >
                +
              </button>
            </div>
          ))}
        </div>
        </div>
  );
};

export default JogControlPanel;
