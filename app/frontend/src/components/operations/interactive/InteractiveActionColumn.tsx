import React from 'react';
import {
  Camera,
  Sparkles,
  Route,
  Box,
  RefreshCw,
  X,
} from 'lucide-react';

interface InteractiveActionColumnProps {
  hasImage: boolean;
  activeTemplate: string | null;
  isCapturing: boolean;
  isReconstructing: boolean;
  isAutoGenerating: boolean;
  segMode: boolean;
  manualPathMode: boolean;
  onTriggerCapture: () => void;
  onToggleSegMode: () => void;
  onToggleManualPathMode: () => void;
  onTriggerReconstruct: () => void;
  onTriggerAutoPath: () => void;
}

// Wide-stance Jeans with crisp Waypoint nodes & scan paths
const JeansWaypointIcon: React.FC<{ size?: number; className?: string }> = ({ size = 18, className = "text-emerald-400" }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="none"
    xmlns="http://www.w3.org/2000/svg"
    className={className}
  >
    {/* Wide-stance Pants Outline (A-Frame Spread Legs) */}
    <path
      d="M5.5 2.5h13l2.5 19h-5.2l-3.8-10.5-3.8 10.5H3L5.5 2.5z"
      stroke="currentColor"
      strokeWidth="1.8"
      strokeLinecap="round"
      strokeLinejoin="round"
      fill="currentColor"
      fillOpacity="0.12"
    />

    {/* Waistband stitch */}
    <path
      d="M5.8 6.5h12.4"
      stroke="currentColor"
      strokeWidth="1.2"
      strokeOpacity="0.5"
    />

    {/* Left Leg Waypoint Trajectory (Zigzag + Node Dots) */}
    <path
      d="M6 9.5l3 3.5-3 4"
      stroke="#38bdf8"
      strokeWidth="1.5"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
    <circle cx="6" cy="9.5" r="1.4" fill="#38bdf8" />
    <circle cx="9" cy="13" r="1.4" fill="#38bdf8" />
    <circle cx="6" cy="17" r="1.4" fill="#38bdf8" />

    {/* Right Leg Waypoint Trajectory (Zigzag + Node Dots) */}
    <path
      d="M18 9.5l-3 3.5 3 4"
      stroke="#38bdf8"
      strokeWidth="1.5"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
    <circle cx="18" cy="9.5" r="1.4" fill="#38bdf8" />
    <circle cx="15" cy="13" r="1.4" fill="#38bdf8" />
    <circle cx="18" cy="17" r="1.4" fill="#38bdf8" />
  </svg>
);

export const InteractiveActionColumn: React.FC<InteractiveActionColumnProps> = ({
  hasImage,
  activeTemplate,
  isCapturing,
  isReconstructing,
  isAutoGenerating,
  segMode,
  manualPathMode,
  onTriggerCapture,
  onToggleSegMode,
  onToggleManualPathMode,
  onTriggerReconstruct,
  onTriggerAutoPath,
}) => {
  const busy = isCapturing || isReconstructing || isAutoGenerating;
  return (
    <div className="w-full shrink-0 bg-slate-950/80 border-b border-slate-800/80 px-2 py-2 flex items-center justify-between gap-1.5 select-none">
      {/* 1. Capture Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onTriggerCapture}
          disabled={busy || !activeTemplate || segMode || manualPathMode}
          className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
        >
          {isCapturing ? <RefreshCw size={14} className="animate-spin text-sky-400" /> : <Camera size={14} className="text-slate-300" />}
        </button>
        <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
          <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
            Capture 2D Color + 3D Depth Data
          </div>
        </div>
      </div>

      {/* 2. Segment Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onToggleSegMode}
          disabled={!hasImage || busy || manualPathMode}
          className={`w-full h-8 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border ${
            segMode
              ? 'bg-slate-800 text-sky-300 border-sky-500 shadow-md shadow-sky-950/40 ring-1 ring-sky-500/40'
              : 'bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 border-slate-700 hover:border-slate-600'
          }`}
        >
          {segMode ? <X size={14} className="text-rose-400" /> : <Sparkles size={14} className="text-sky-400" />}
        </button>
        <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
          <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
            {segMode ? 'Exit Segmentation Mode' : 'MobileSAM Interactive Segmentation'}
          </div>
        </div>
      </div>

      {/* 3. Reconstruct Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onTriggerReconstruct}
          disabled={!hasImage || busy || segMode || manualPathMode}
          className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
        >
          {isReconstructing ? <RefreshCw size={14} className="animate-spin text-sky-400" /> : <Box size={14} className="text-indigo-400" />}
        </button>
        <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
          <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
            Surface Poisson 3D Mesh Reconstruction
          </div>
        </div>
      </div>

      {/* 4. Auto Jeans Waypoint Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onTriggerAutoPath}
          disabled={!hasImage || busy || segMode || manualPathMode}
          className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
        >
          {isAutoGenerating ? <RefreshCw size={14} className="animate-spin text-emerald-400" /> : <JeansWaypointIcon size={16} className="text-emerald-400" />}
        </button>
        <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
          <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
            Auto Generate Jeans Waypoints from Mesh + Mask
          </div>
        </div>
      </div>

      {/* 5. Manual TCP Path Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onToggleManualPathMode}
          disabled={!hasImage || busy || segMode}
          className={`w-full h-8 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border ${
            manualPathMode
              ? 'bg-slate-800 text-amber-300 border-amber-500 shadow-md shadow-amber-950/40 ring-1 ring-amber-500/40'
              : 'bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 border-slate-700 hover:border-slate-600'
          }`}
        >
          {manualPathMode ? <X size={14} className="text-rose-400" /> : <Route size={14} className="text-amber-400" />}
        </button>
        <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
          <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
            {manualPathMode ? 'Exit Manual TCP Design' : 'Manual TCP Path & Normal Design'}
          </div>
        </div>
      </div>
    </div>
  );
};
