import React, { useState, useRef, useEffect } from 'react';
import { Play, Pause, X, GripVertical } from 'lucide-react';
import { type SimulationState, STATE_THEMES } from './types';
import { Tooltip } from '../../common/Tooltip';

interface SimulationPlaybackHUDProps {
  simulationState: SimulationState;
  onPlayPause: () => void;
  onSeek: (progress: number) => void;
  onSpeedChange: (speed: number) => void;
  onStop: () => void;
}

export const SimulationPlaybackHUD: React.FC<SimulationPlaybackHUDProps> = ({
  simulationState,
  onPlayPause,
  onSeek,
  onSpeedChange,
  onStop,
}) => {
  const [position, setPosition] = useState<{ x: number; y: number } | null>(null);
  const [isDragging, setIsDragging] = useState(false);
  const hudRef = useRef<HTMLDivElement | null>(null);
  const dragStartRef = useRef<{ startX: number; startY: number; initialX: number; initialY: number }>({
    startX: 0,
    startY: 0,
    initialX: 0,
    initialY: 0,
  });

  const handleMouseDown = (e: React.MouseEvent) => {
    // Only drag with left mouse button
    if (e.button !== 0) return;
    // Don't drag when clicking interactive buttons or scrubber
    const target = e.target as HTMLElement;
    if (target.closest('button, input')) return;

    if (!hudRef.current) return;
    const rect = hudRef.current.getBoundingClientRect();
    const parentRect = hudRef.current.parentElement?.getBoundingClientRect() || {
      left: 0,
      top: 0,
      width: window.innerWidth,
      height: window.innerHeight,
    };

    const currentX = position ? position.x : (rect.left - parentRect.left);
    const currentY = position ? position.y : (rect.top - parentRect.top);

    dragStartRef.current = {
      startX: e.clientX,
      startY: e.clientY,
      initialX: currentX,
      initialY: currentY,
    };
    setIsDragging(true);
    e.preventDefault();
  };

  const handleDoubleClick = () => {
    // Double click to snap back to default top-center position
    setPosition(null);
  };

  useEffect(() => {
    if (!isDragging) return;

    const handleMouseMove = (e: MouseEvent) => {
      if (!hudRef.current) return;
      const dx = e.clientX - dragStartRef.current.startX;
      const dy = e.clientY - dragStartRef.current.startY;

      const parentEl = hudRef.current.parentElement;
      const parentWidth = parentEl ? parentEl.clientWidth : window.innerWidth;
      const parentHeight = parentEl ? parentEl.clientHeight : window.innerHeight;
      const selfWidth = hudRef.current.offsetWidth;
      const selfHeight = hudRef.current.offsetHeight;

      const newX = Math.max(8, Math.min(parentWidth - selfWidth - 8, dragStartRef.current.initialX + dx));
      const newY = Math.max(8, Math.min(parentHeight - selfHeight - 8, dragStartRef.current.initialY + dy));

      setPosition({ x: newX, y: newY });
    };

    const handleMouseUp = () => {
      setIsDragging(false);
    };

    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging]);

  const style: React.CSSProperties = position
    ? {
        position: 'absolute',
        left: `${position.x}px`,
        top: `${position.y}px`,
        transform: 'none',
      }
    : {
        position: 'absolute',
        top: '12px',
        left: '50%',
        transform: 'translateX(-50%)',
      };

  const theme = STATE_THEMES[simulationState.activeState] || STATE_THEMES.raw;

  return (
    <div
      ref={hudRef}
      style={style}
      onMouseDown={handleMouseDown}
      onDoubleClick={handleDoubleClick}
      className={`z-40 flex items-center gap-1.5 px-2.5 py-1 rounded-lg bg-slate-950/90 backdrop-blur-md border border-sky-500/30 shadow-xl select-none text-[9.5px] font-mono leading-none transition-shadow ${
        isDragging
          ? 'cursor-grabbing shadow-[0_0_20px_rgba(14,165,233,0.4)] border-sky-400'
          : 'cursor-grab hover:border-sky-500/50'
      }`}
    >
      {/* Drag Grip Handle */}
      <div className="relative group flex items-center">
        <div className="text-slate-500 hover:text-slate-300 transition-colors shrink-0 -ml-0.5 mr-0.5 cursor-grab">
          <GripVertical size={11} />
        </div>
        <Tooltip text="Drag to reposition • Double-click to reset" side="bottom" />
      </div>

      {/* Play/Pause Button */}
      <div className="relative group flex items-center shrink-0">
        <button
          onClick={onPlayPause}
          className="h-5 w-5 rounded bg-sky-600 hover:bg-sky-500 active:scale-95 text-white flex items-center justify-center shadow-sm transition-all cursor-pointer"
        >
          {simulationState.isPlaying ? <Pause size={9} /> : <Play size={9} className="fill-white" />}
        </button>
        <Tooltip text={simulationState.isPlaying ? 'Pause Simulation' : 'Resume Simulation'} side="bottom" />
      </div>

      {/* State / Exec Badge */}
      <span
        className={`text-[8.5px] font-bold font-mono px-1.5 py-0.5 rounded border shrink-0 ${
          simulationState.isRealExec
            ? 'bg-amber-500/20 text-amber-300 border-amber-500/40 animate-pulse'
            : `${theme.bg} ${theme.text} ${theme.border}`
        }`}
      >
        {simulationState.isRealExec ? 'REAL EXEC' : `${simulationState.activeState.toUpperCase()} SIM`}
      </span>

      {/* Scrubber and Progress */}
      <div className="flex items-center gap-1.5 shrink-0">
        <input
          type="range"
          min={0}
          max={1}
          step={0.002}
          value={simulationState.progress}
          onChange={(e) => onSeek(parseFloat(e.target.value))}
          className="w-24 md:w-28 accent-sky-400 h-1 bg-slate-800 rounded cursor-pointer"
        />
        <span className="text-[9.5px] font-bold text-white min-w-[28px] text-right">
          {Math.round(simulationState.progress * 100)}%
        </span>
        <span className="text-[8.5px] text-slate-400">
          ({simulationState.currentStep}/{simulationState.totalSteps})
        </span>
      </div>

      {/* Speed Multiplier Options */}
      {!simulationState.isRealExec && (
        <div className="flex items-center bg-slate-900/90 rounded p-0.5 border border-white/10 text-[9px] font-mono shrink-0">
          {[0.5, 1.0, 2.0, 5.0].map((spd) => (
            <button
              key={spd}
              onClick={() => onSpeedChange(spd)}
              className={`px-1 py-0.5 rounded transition-all cursor-pointer ${
                simulationState.speed === spd
                  ? 'bg-sky-600 text-white font-bold shadow-sm'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              {spd}x
            </button>
          ))}
        </div>
      )}

      {/* Stop / Close Button */}
      <div className="relative group flex items-center shrink-0 ml-0.5">
        <button
          onClick={onStop}
          className="h-5 w-5 rounded bg-slate-900/90 border border-white/10 text-slate-400 hover:text-rose-300 hover:bg-rose-900/40 flex items-center justify-center transition-colors shadow-sm cursor-pointer"
        >
          <X size={10} />
        </button>
        <Tooltip text="Stop Simulation" side="bottom" />
      </div>
    </div>
  );
};
