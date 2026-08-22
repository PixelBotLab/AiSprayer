import React, { useEffect, useState, useRef } from 'react';
import { Cpu, Crosshair, Home, Pause, Play, AlertOctagon, Minimize2 } from 'lucide-react';
import { API_BASE, WS_BASE } from '../config';

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

interface JogControlPanelProps {
  robotState?: RobotState;
}

const JogControlPanel: React.FC<JogControlPanelProps> = ({ robotState }) => {
  const [speedL, setSpeedL] = useState<number>(100);
  const [accL, setAccL] = useState<number>(20);
  const [speedJ, setSpeedJ] = useState<number>(20);
  const [accJ, setAccJ] = useState<number>(20);

  const [globalSpeedFactor, setGlobalSpeedFactor] = useState<number>(50);
  const [maxTcpSpeed, setMaxTcpSpeed] = useState<number>(2000);
  const [maxJointSpeed, setMaxJointSpeed] = useState<number>(180);

  // 根据最大值和全局比例计算具体可用区间上限
  const effMaxLinSpeed = Math.max(10, Math.round(maxTcpSpeed * (globalSpeedFactor / 100.0)));
  const effMaxJntSpeed = Math.max(5, Math.round(maxJointSpeed * (globalSpeedFactor / 100.0)));

  // Use prop robotState if provided, else fall back to internal zeros
  const displayState: RobotState = robotState ?? { pose: [0, 0, 0, 0, 0, 0], joint: [0, 0, 0, 0, 0, 0] };
  const [robotConnected, setRobotConnected] = useState<boolean>(false);
  const [connecting, setConnecting] = useState<boolean>(false);
  const [activeAction, setActiveAction] = useState<'home' | 'zero' | 'fold' | null>(null);

  // Track active jog button
  const [activeJog, setActiveJog] = useState<{ axis: string, dir: number } | null>(null);

  const wsRef = useRef<WebSocket | null>(null);

  const isMoving = displayState.status === 1;
  const disableMotion = !robotConnected || isMoving;

  useEffect(() => {
    if (!isMoving) {
      setActiveAction(null);
    }
  }, [isMoving]);

  useEffect(() => {
    fetchSpeed();
    connectWs();
    return () => {
      if (wsRef.current) wsRef.current.close();
    };
  }, []);

  const connectWs = () => {
    const ws = new WebSocket(`${WS_BASE}/api/calib/robot/ws`);
    ws.onopen = () => { };
    ws.onclose = () => {
      setTimeout(connectWs, 2000); // Reconnect WebSocket
    };
    ws.onmessage = (_e) => {
      // Robot state is now handled at CalibView level and passed as a prop
    };
    wsRef.current = ws;
  };

  const fetchSpeed = async () => {
    try {
      const spdRes = await fetch(`${API_BASE}/api/calib/robot/speed`);
      if (spdRes.ok) {
        const spdData = await spdRes.json();
        const gFactor = spdData.global_speed_factor ?? 50;
        const maxTcp = spdData.max_tcp_speed_mm_s ?? 2000;
        const maxJnt = Array.isArray(spdData.max_joint_speed_deg_s)
          ? spdData.max_joint_speed_deg_s[0]
          : (spdData.max_joint_speed_deg_s ?? 180);

        setGlobalSpeedFactor(gFactor);
        setMaxTcpSpeed(maxTcp);
        setMaxJointSpeed(maxJnt);

        const currentEffMaxLin = Math.max(10, Math.round(maxTcp * (gFactor / 100.0)));
        const currentEffMaxJnt = Math.max(5, Math.round(maxJnt * (gFactor / 100.0)));

        if (spdData.speed_l !== undefined && spdData.speed_l !== null) {
          setSpeedL(Math.min(currentEffMaxLin, Math.max(1, Math.round(spdData.speed_l))));
        } else {
          setSpeedL(Math.min(currentEffMaxLin, 100));
        }

        if (spdData.speed_j !== undefined && spdData.speed_j !== null) {
          setSpeedJ(Math.min(currentEffMaxJnt, Math.max(1, Math.round(spdData.speed_j))));
        } else {
          setSpeedJ(Math.min(currentEffMaxJnt, 20));
        }

        setAccL(spdData.acc_l || 20);
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
        const res = await fetch(`${API_BASE}/api/calib/robot/disconnect`, { method: 'POST' });
        if (res.ok) {
          setRobotConnected(false);
        }
      } catch (err) {
        console.error('Disconnect error:', err);
      }
    } else {
      // Connect
      try {
        const res = await fetch(`${API_BASE}/api/calib/robot/connect`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ robot_type: 'dobot' })
        });
        if (res.ok) {
          setRobotConnected(true);
          // Fetch current global speed
          fetchSpeed();
        } else {
          console.error('Failed to connect to robot.');
        }
      } catch (err: any) {
        console.error('Connect error:', err);
      }
    }
    setConnecting(false);
  };

  const handleSpeedUpdate = async () => {
    try {
      await fetch(`${API_BASE}/api/calib/robot/speed`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed_l: speedL, acc_l: accL, speed_j: speedJ, acc_j: accJ })
      });
    } catch (err) {
      console.error('Failed to sync speed:', err);
    }
  };

  const syncGlobalSpeed = async (newFactor: number) => {
    try {
      await fetch(`${API_BASE}/api/calib/robot/global_speed`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ factor: newFactor })
      });
      fetchSpeed(); // Re-fetch to update effective caps
    } catch (err) {
      console.error('Failed to sync global speed factor:', err);
    }
  };

  const handleJogContinuous = async (axis: string, direction: number) => {
    if (direction !== 0) {
      setActiveJog({ axis, dir: direction });
    } else {
      setActiveJog(null);
    }
    try {
      const res = await fetch(`${API_BASE}/api/calib/robot/jog_continuous`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ axis, direction })
      });
      if (!res.ok) {
        const error = await res.json();
        console.error(`Jog failed: ${error.detail || 'Unknown error'}`);
        setActiveJog(null);
      }
    } catch (err: any) {
      console.error(`Jog error: ${err.message}`);
      setActiveJog(null);
    }
  };

  const handleZero = async () => {
    setActiveAction('zero');
    try {
      const res = await fetch(`${API_BASE}/api/calib/robot/zero`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed: speedJ, acc: accJ })
      });
      if (!res.ok) {
        const error = await res.json();
        console.error(`Zero failed: ${error.detail || 'Unknown error'}`);
      }
    } catch (err: any) {
      console.error('Failed to go zero:', err);
    } finally {
      setActiveAction(null);
    }
  };

  const handleHome = async () => {
    setActiveAction('home');
    try {
      const res = await fetch(`${API_BASE}/api/calib/robot/home`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed: speedJ, acc: accJ })
      });
      if (!res.ok) {
        const error = await res.json();
        console.error(`Home failed: ${error.detail || 'Unknown error'}`);
      }
    } catch (err: any) {
      console.error('Failed to go home:', err);
    } finally {
      setActiveAction(null);
    }
  };

  const handleFold = async () => {
    setActiveAction('fold');
    try {
      const res = await fetch(`${API_BASE}/api/calib/robot/fold`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed: speedJ, acc: accJ })
      });
      if (!res.ok) {
        const error = await res.json();
        console.error(`Fold failed: ${error.detail || 'Unknown error'}`);
      }
    } catch (err: any) {
      console.error('Failed to fold robot:', err);
    } finally {
      setActiveAction(null);
    }
  };

  const handlePause = async () => {
    try {
      await fetch(`${API_BASE}/api/calib/robot/pause`, { method: 'POST' });
    } catch (err: any) { console.error('Pause error:', err); }
  };

  const handleResume = async () => {
    try {
      await fetch(`${API_BASE}/api/calib/robot/resume`, { method: 'POST' });
    } catch (err: any) { console.error('Resume error:', err); }
  };

  const handleEstop = async () => {
    try {
      await fetch(`${API_BASE}/api/calib/robot/estop`, { method: 'POST' });
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

  const renderAxisRow = (axis: { name: string, unit: string }, idx: number, isJoint: boolean) => {
    const currentValues = isJoint ? displayState.joint : displayState.pose;

    const formatValue = (idx: number, isJoint: boolean) => {
      let val = currentValues[idx];
      if (val === undefined || val === null) return '0.00';
      if (!isJoint && idx >= 3) {
        val = val * (180 / Math.PI);
      }
      if (Math.abs(val) < 0.005) return '0.00';
      const formatted = val.toFixed(2);
      return formatted === '-0.00' || (formatted.startsWith('-0.0') && parseFloat(formatted) === 0) ? '0.00' : formatted;
    };

    let pulseDuration = '0s';
    const isJoggingNeg = activeJog?.axis === axis.name && activeJog?.dir === -1;
    const isJoggingPos = activeJog?.axis === axis.name && activeJog?.dir === 1;

    if (activeJog && (isJoggingNeg || isJoggingPos)) {
      let speed = 10;
      if (isJoint) {
        let v = (displayState.qd_actual && displayState.qd_actual[idx]) || 0;
        speed = Math.abs(v);
      } else {
        if (displayState.tcp_speed_actual && displayState.tcp_speed_actual.length > idx) {
          let v = displayState.tcp_speed_actual[idx];
          if (idx < 3) {
            // backend tcp_speed_actual is m/s, convert to mm/s
            speed = Math.abs(v) * 1000;
          } else {
            // rad/s to deg/s
            speed = Math.abs(v) * (180 / Math.PI);
          }
        } else {
          speed = speedL;
        }
      }

      // Calculate duration inversely proportional to speed
      // Map 10mm/s -> 1s, 50mm/s -> 0.2s
      speed = Math.max(0.1, speed);
      let d = 10 / speed;
      d = Math.min(1.5, Math.max(0.1, d)); // Clamp between 0.1s and 1.5s
      pulseDuration = d.toFixed(3) + 's';
    }

    const getBtnClass = (isActive: boolean) => `
      w-8 h-7 flex items-center justify-center border rounded transition-colors select-none text-[14px] font-bold
      ${isActive
        ? 'bg-blue-600 border-blue-400 text-white animate-pulse shadow-[0_0_12px_rgba(59,130,246,0.9)] z-10'
        : 'bg-slate-800 border-slate-700 text-slate-300 hover:bg-slate-700 hover:text-white'}
      ${disableMotion && !isActive ? 'opacity-50 cursor-not-allowed hover:bg-slate-800' : ''}
    `;

    return (
      <div key={axis.name} className="flex items-center gap-1">
        <button
          onPointerDown={(e) => { e.currentTarget.releasePointerCapture(e.pointerId); handleJogContinuous(axis.name, -1); }}
          onPointerUp={() => handleJogContinuous(axis.name, 0)}
          onPointerLeave={() => { if (activeJog?.axis === axis.name && activeJog?.dir === -1) handleJogContinuous(axis.name, 0); }}
          onContextMenu={(e) => e.preventDefault()}
          disabled={disableMotion && !isJoggingNeg}
          className={getBtnClass(isJoggingNeg)}
          style={isJoggingNeg ? { animationDuration: pulseDuration } : {}}
        >
          -
        </button>

        <div className="flex-1 flex items-center justify-between bg-slate-950 border border-slate-800 rounded px-1.5 h-7">
          <span className="text-[10px] font-bold text-slate-500 w-[18px]">{axis.name}</span>
          <span className="text-[11px] font-mono text-emerald-400 tracking-wider">
            {formatValue(idx, isJoint)}
          </span>
          <span className="text-[8px] text-slate-600 w-[10px]">{axis.unit}</span>
        </div>

        <button
          onPointerDown={(e) => { e.currentTarget.releasePointerCapture(e.pointerId); handleJogContinuous(axis.name, 1); }}
          onPointerUp={() => handleJogContinuous(axis.name, 0)}
          onPointerLeave={() => { if (activeJog?.axis === axis.name && activeJog?.dir === 1) handleJogContinuous(axis.name, 0); }}
          onContextMenu={(e) => e.preventDefault()}
          disabled={disableMotion && !isJoggingPos}
          className={getBtnClass(isJoggingPos)}
          style={isJoggingPos ? { animationDuration: pulseDuration } : {}}
        >
          +
        </button>
      </div>
    );
  };

  return (
    <div className="bg-slate-900/80 rounded-xl border border-slate-800 p-3 shadow-lg backdrop-blur-sm shrink-0 w-full flex flex-col gap-2.5 relative">
      {/* Jog Controls Toolbar */}
      <div className="flex justify-between items-center mt-1">
        <div className="flex flex-col gap-1 w-[80px] relative">
          {/* Status LED Indicator */}
          <div className="absolute -top-4 left-0 flex items-center gap-1.5 pointer-events-none">
            <div className={`w-2 h-2 rounded-full transition-colors duration-300 ${!robotConnected ? 'bg-slate-600' : isMoving ? 'bg-amber-400 animate-pulse shadow-[0_0_8px_rgba(251,191,36,0.6)]' : 'bg-emerald-500 shadow-[0_0_8px_rgba(16,185,129,0.6)]'}`}></div>
            <span className="text-[9px] font-medium uppercase tracking-wider text-slate-400">
              {!robotConnected ? 'Offline' : isMoving ? 'Moving' : 'Idle'}
            </span>
          </div>
        </div>
        {/* Action Buttons (Single Row) */}
        <div className="flex flex-row gap-1 flex-1 overflow-x-auto hide-scrollbar max-w-full">
          <button
            onClick={handleConnect}
            disabled={connecting}
            className={`px-2 h-7 text-xs font-medium rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors border disabled:opacity-50 ${robotConnected ? 'bg-green-600/20 text-green-500 hover:bg-green-600 hover:text-white border-green-600/30' : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50'}`}
          >
            <Cpu size={12} className="shrink-0" /> <span>{connecting ? '...' : robotConnected ? 'Disconnect' : 'Connect'}</span>
          </button>
          <button
            onClick={handleHome}
            disabled={disableMotion}
            className={`px-2 h-7 text-xs font-medium rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors border ${activeAction === 'home'
                ? 'bg-emerald-600/20 text-emerald-400 border-emerald-500/50 animate-pulse shadow-[0_0_8px_rgba(16,185,129,0.5)] cursor-not-allowed'
                : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50 disabled:opacity-50 disabled:cursor-not-allowed'
              }`}
          >
            <Home size={12} className="shrink-0" /> <span>Home</span>
          </button>
          <button
            onClick={handleZero}
            disabled={disableMotion}
            className={`px-2 h-7 text-xs font-medium rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors border ${activeAction === 'zero'
                ? 'bg-emerald-600/20 text-emerald-400 border-emerald-500/50 animate-pulse shadow-[0_0_8px_rgba(16,185,129,0.5)] cursor-not-allowed'
                : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50 disabled:opacity-50 disabled:cursor-not-allowed'
              }`}
          >
            <Crosshair size={12} className="shrink-0" /> <span>Zero</span>
          </button>
          <button
            onClick={handleFold}
            disabled={disableMotion}
            title="Fold robot to [0, 0, -156°, 0, 0, 0]"
            className={`px-2 h-7 text-xs font-medium rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors border ${activeAction === 'fold'
                ? 'bg-purple-600/20 text-purple-400 border-purple-500/50 animate-pulse shadow-[0_0_8px_rgba(168,85,247,0.5)] cursor-not-allowed'
                : 'bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border-slate-700/50 disabled:opacity-50 disabled:cursor-not-allowed'
              }`}
          >
            <Minimize2 size={12} className="shrink-0" /> <span>Fold</span>
          </button>
          <button
            onClick={handlePause}
            className="px-2 h-7 text-xs font-medium rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border border-slate-700/50"
          >
            <Pause size={12} className="shrink-0" /> <span>Pause</span>
          </button>
          <button
            onClick={handleResume}
            className="px-2 h-7 text-xs font-medium rounded-md flex items-center justify-center gap-1 shrink-0 transition-colors bg-slate-800 text-slate-300 hover:bg-slate-700 hover:text-white border border-slate-700/50"
          >
            <Play size={12} className="shrink-0" /> <span>Resume</span>
          </button>
          <button
            onClick={handleEstop}
            className="px-2 h-7 text-xs font-bold rounded-md flex items-center justify-center gap-1.5 shrink-0 transition-colors bg-red-600/20 text-red-500 hover:bg-red-600 hover:text-white border border-red-600/30"
          >
            <AlertOctagon size={12} className="shrink-0" /> <span>Stop</span>
          </button>
        </div>
      </div>

      {/* Speed & Acceleration Sliders & Motion Dynamics */}
      <div className="flex flex-col gap-1.5 mb-0.5 bg-slate-950/40 p-2 rounded-lg border border-slate-800/40 text-[10px]">
        {/* Motion Dynamics Header & Global Speed */}
        <div className="flex flex-wrap items-center justify-between gap-2 px-0.5 border-b border-slate-800/40 pb-1.5 font-medium">
          <div className="flex items-center gap-2">
            <span className="tracking-wider uppercase text-slate-400 font-semibold text-[9.5px]">Dynamics</span>
            <div className="flex items-center gap-1.5 w-24">
              <input
                type="range"
                className="flex-1 accent-amber-500 h-1 bg-slate-700/50 rounded-lg appearance-none cursor-pointer"
                min="1" max="100" step="1"
                value={globalSpeedFactor}
                onChange={(e) => setGlobalSpeedFactor(parseInt(e.target.value))}
                onMouseUp={(e) => syncGlobalSpeed(parseInt((e.target as HTMLInputElement).value))}
                onTouchEnd={(e) => syncGlobalSpeed(parseInt((e.target as HTMLInputElement).value))}
              />
              <span className="text-amber-400 font-mono font-semibold text-[9px] w-6 text-right shrink-0">{globalSpeedFactor}%</span>
            </div>
          </div>
          <div className="font-mono text-slate-400 flex flex-wrap items-center gap-1.5 text-[9px]">
            <span>TCP Cap: <strong className="text-sky-400">{effMaxLinSpeed} mm/s</strong></span>
            <span className="text-slate-600">|</span>
            <span>JNT Cap: <strong className="text-emerald-400">{effMaxJntSpeed} °/s</strong></span>
            <span className="text-slate-600">|</span>
            <span>Load: <strong className="text-purple-400">{(displayState.load ?? 0).toFixed(2)} kg</strong></span>
          </div>
        </div>

        {/* Linear Speed/Acc */}
        <div className="flex gap-3">
          <div className="flex-1 flex flex-col gap-0.5">
            <div className="flex justify-between items-baseline text-[9.5px] font-medium">
              <span className="text-slate-400">LIN SPEED</span>
              <div className="flex items-center gap-1 font-mono">
                <span className="text-blue-400 font-semibold">{speedL} mm/s</span>
                <span className="text-[8.5px] text-slate-500 font-normal">/ {effMaxLinSpeed}</span>
              </div>
            </div>
            <input
              type="range"
              min="1"
              max={effMaxLinSpeed}
              step="1"
              value={Math.min(speedL, effMaxLinSpeed)}
              onChange={(e) => setSpeedL(Number(e.target.value))}
              onPointerUp={handleSpeedUpdate}
              onKeyUp={handleSpeedUpdate}
              className="w-full h-1 bg-slate-800 rounded-lg appearance-none cursor-pointer accent-blue-500"
            />
          </div>
          <div className="flex-1 flex flex-col gap-0.5">
            <div className="flex justify-between items-baseline text-[9.5px] font-medium">
              <span className="text-slate-400">LIN ACCEL</span>
              <span className="text-blue-400 font-mono font-semibold">{accL}%</span>
            </div>
            <input
              type="range"
              min="1"
              max="100"
              value={accL}
              onChange={(e) => setAccL(Number(e.target.value))}
              onPointerUp={handleSpeedUpdate}
              onKeyUp={handleSpeedUpdate}
              className="w-full h-1 bg-slate-800 rounded-lg appearance-none cursor-pointer accent-blue-500"
            />
          </div>
        </div>

        {/* Joint Speed/Acc */}
        <div className="flex gap-3">
          <div className="flex-1 flex flex-col gap-0.5">
            <div className="flex justify-between items-baseline text-[9.5px] font-medium">
              <span className="text-slate-400">JNT SPEED</span>
              <div className="flex items-center gap-1 font-mono">
                <span className="text-green-400 font-semibold">{speedJ} °/s</span>
                <span className="text-[8.5px] text-slate-500 font-normal">/ {effMaxJntSpeed}</span>
              </div>
            </div>
            <input
              type="range"
              min="1"
              max={effMaxJntSpeed}
              step="1"
              value={Math.min(speedJ, effMaxJntSpeed)}
              onChange={(e) => setSpeedJ(Number(e.target.value))}
              onPointerUp={handleSpeedUpdate}
              onKeyUp={handleSpeedUpdate}
              className="w-full h-1 bg-slate-800 rounded-lg appearance-none cursor-pointer accent-green-500"
            />
          </div>
          <div className="flex-1 flex flex-col gap-0.5">
            <div className="flex justify-between items-baseline text-[9.5px] font-medium">
              <span className="text-slate-400">JNT ACCEL</span>
              <span className="text-green-400 font-mono font-semibold">{accJ}%</span>
            </div>
            <input
              type="range"
              min="1"
              max="100"
              value={accJ}
              onChange={(e) => setAccJ(Number(e.target.value))}
              onPointerUp={handleSpeedUpdate}
              onKeyUp={handleSpeedUpdate}
              className="w-full h-1 bg-slate-800 rounded-lg appearance-none cursor-pointer accent-green-500"
            />
          </div>
        </div>
      </div>

      {/* Axis Rows (Side by Side) */}
      <div className="flex gap-3 mt-1">
        {/* Cartesian (Left) */}
        <div className="flex flex-col gap-1.5 flex-1">
          {cartesianAxes.map((axis, idx) => renderAxisRow(axis, idx, false))}
        </div>

        {/* Divider */}
        <div className="w-[1px] bg-slate-800/60 rounded-full" />

        {/* Joint (Right) */}
        <div className="flex flex-col gap-1.5 flex-1">
          {jointAxes.map((axis, idx) => renderAxisRow(axis, idx, true))}
        </div>
      </div>
    </div>
  );
};

export default JogControlPanel;
