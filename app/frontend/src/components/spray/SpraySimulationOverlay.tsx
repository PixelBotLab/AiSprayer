import React, { useState, useRef, useEffect } from 'react';
import { Layers, RotateCcw, Activity, GripVertical } from 'lucide-react';
import type { SpraySimConfig, SprayVisualMode, SprayCoverageStats } from './sprayTypes';
import { Tooltip } from '../common/Tooltip';

interface SpraySimulationOverlayProps {
  config: SpraySimConfig;
  stats: SprayCoverageStats;
  isSprayingActive: boolean;
  onModeChange: (mode: SprayVisualMode) => void;
  onClearCoat: () => void;
}

export const SpraySimulationOverlay: React.FC<SpraySimulationOverlayProps> = ({
  config,
  stats,
  isSprayingActive,
  onModeChange,
  onClearCoat,
}) => {
  const [position, setPosition] = useState<{ x: number; y: number } | null>(null);
  const [isDragging, setIsDragging] = useState(false);
  const overlayRef = useRef<HTMLDivElement | null>(null);
  const dragStartRef = useRef<{ startX: number; startY: number; initialX: number; initialY: number }>({
    startX: 0,
    startY: 0,
    initialX: 0,
    initialY: 0,
  });

  const handleMouseDown = (e: React.MouseEvent) => {
    // Only drag with left mouse button
    if (e.button !== 0) return;
    // Don't drag when clicking interactive buttons
    const target = e.target as HTMLElement;
    if (target.closest('button')) return;

    if (!overlayRef.current) return;
    const rect = overlayRef.current.getBoundingClientRect();
    const parentRect = overlayRef.current.parentElement?.getBoundingClientRect() || {
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
    // Double click to snap back to default bottom-center position
    setPosition(null);
  };

  useEffect(() => {
    if (!isDragging) return;

    const handleMouseMove = (e: MouseEvent) => {
      if (!overlayRef.current) return;
      const dx = e.clientX - dragStartRef.current.startX;
      const dy = e.clientY - dragStartRef.current.startY;

      const parentEl = overlayRef.current.parentElement;
      const parentWidth = parentEl ? parentEl.clientWidth : window.innerWidth;
      const parentHeight = parentEl ? parentEl.clientHeight : window.innerHeight;
      const selfWidth = overlayRef.current.offsetWidth;
      const selfHeight = overlayRef.current.offsetHeight;

      // Keep inside parent container bounds
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

  if (!config.enabled) return null;

  const style: React.CSSProperties = position
    ? {
        position: 'absolute',
        left: `${position.x}px`,
        top: `${position.y}px`,
        bottom: 'auto',
        transform: 'none',
      }
    : {
        position: 'absolute',
        bottom: '10px',
        left: '50%',
        transform: 'translateX(-50%)',
      };

  return (
    <div
      ref={overlayRef}
      style={style}
      onMouseDown={handleMouseDown}
      onDoubleClick={handleDoubleClick}
      className={`z-20 flex items-center gap-1.5 px-2 py-1 rounded-lg bg-slate-950/90 backdrop-blur-md border border-cyan-500/30 shadow-xl select-none text-[9.5px] font-mono leading-none transition-shadow ${
        isDragging
          ? 'cursor-grabbing shadow-[0_0_20px_rgba(6,182,212,0.4)] border-cyan-400'
          : 'cursor-grab hover:border-cyan-500/50'
      }`}
    >
      {/* Drag Grip Handle */}
      <div className="relative group flex items-center">
        <div className="text-slate-500 hover:text-slate-300 transition-colors shrink-0 -ml-0.5 mr-0.5 cursor-grab">
          <GripVertical size={11} />
        </div>
        <Tooltip text="Drag to reposition • Double-click to reset" side="top" />
      </div>

      {/* Active Spraying Indicator */}
      <div className="flex items-center gap-1 pr-1.5 border-r border-white/10 shrink-0 pointer-events-none">
        <span className={`w-1.5 h-1.5 rounded-full ${isSprayingActive ? 'bg-cyan-400 animate-ping' : 'bg-slate-500'}`}></span>
        <span className="font-bold text-cyan-400 tracking-wider">SPRAY</span>
      </div>

      {/* Visual Mode Segmented Switcher */}
      <div className="flex items-center bg-slate-900/90 rounded p-0.5 border border-white/10 shrink-0">
        <div className="relative group flex items-center">
          <button
            onClick={() => onModeChange('realistic')}
            className={`px-1.5 py-0.5 rounded transition-all flex items-center gap-1 cursor-pointer ${
              config.visualMode === 'realistic'
                ? 'bg-blue-600 text-white font-medium shadow-sm'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Layers size={9} />
            <span>Realistic</span>
          </button>
          <Tooltip text="Realistic paint coating appearance" side="top" />
        </div>

        <div className="relative group flex items-center">
          <button
            onClick={() => onModeChange('heatmap')}
            className={`px-1.5 py-0.5 rounded transition-all flex items-center gap-1 cursor-pointer ${
              config.visualMode === 'heatmap'
                ? 'bg-emerald-600 text-white font-medium shadow-sm'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Activity size={9} />
            <span>Heatmap</span>
          </button>
          <Tooltip text="Film thickness heatmap inspection" side="top" />
        </div>
      </div>

      {/* Clear Coat Button */}
      <div className="relative group shrink-0 flex items-center">
        <button
          onClick={onClearCoat}
          className="h-5 w-5 rounded bg-slate-900/90 border border-white/10 text-slate-400 hover:text-amber-300 hover:bg-slate-800/80 flex items-center justify-center transition-colors shadow-sm cursor-pointer"
        >
          <RotateCcw size={10} />
        </button>
        <Tooltip text="Clear accumulated coating" side="top" />
      </div>

      <div className="w-px h-3.5 bg-white/10 shrink-0"></div>

      {/* Process Metrics Info */}
      <div className="flex items-center gap-2 shrink-0">
        <div className="relative group flex items-center gap-0.5">
          <span className="text-slate-400">Cov:</span>
          <span className="font-bold text-white">{stats.coveragePercentage}%</span>
          <Tooltip text="Indicative surface coverage percentage" side="top" />
        </div>
        <div className="relative group flex items-center gap-0.5">
          <span className="text-slate-400">Avg:</span>
          <span className="font-bold text-emerald-400">{stats.averageThickness}μm</span>
          <Tooltip text="Average dry film thickness (μm)" side="top" />
        </div>
        <div className="relative group flex items-center gap-0.5">
          <span className="text-slate-400">Std:</span>
          <span className="font-bold text-teal-300">{stats.standardComplianceRate}%</span>
          <Tooltip text="Standard compliance rate [35-65 μm]" side="top" />
        </div>
      </div>
    </div>
  );
};
