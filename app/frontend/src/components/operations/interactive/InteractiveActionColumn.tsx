import React from 'react';
import {
  Camera,
  Sparkles,
  Route,
  Box,
  RefreshCw,
  X,
  Wand2,
  MousePointer2,
} from 'lucide-react';

interface InteractiveActionColumnProps {
  hasImage: boolean;
  hasMask: boolean;
  hasMesh: boolean;
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

const AutoPathIcon: React.FC<{ size?: number; className?: string }> = ({ size = 18, className = "text-emerald-400" }) => (
  <div className={`flex items-center gap-0.5 ${className}`}>
    <Wand2 size={size * 0.9} strokeWidth={2.5} />
    <Route size={size * 0.9} strokeWidth={2.5} className="rotate-90" />
  </div>
);

const ManualPathIcon: React.FC<{ size?: number; className?: string }> = ({ size = 18, className = "text-amber-400" }) => (
  <div className={`flex items-center gap-0.5 ${className}`}>
    <MousePointer2 size={size * 0.9} strokeWidth={2.5} />
    <Route size={size * 0.9} strokeWidth={2.5} className="rotate-90" />
  </div>
);

export const InteractiveActionColumn: React.FC<InteractiveActionColumnProps> = ({
  hasImage,
  hasMask,
  hasMesh,
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

  const canReconstruct = hasImage && hasMask && !busy && !segMode && !manualPathMode;
  const canAutoPath = hasImage && hasMask && hasMesh && !busy && !segMode && !manualPathMode;
  const canManualPath = hasImage && hasMesh && !busy && !segMode;

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
            {!activeTemplate ? 'Select or create a template first' : 'Capture 2D Color + 3D Depth Data'}
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
            {!hasImage ? 'Capture RGB image first' : (segMode ? 'Exit Segmentation Mode' : 'MobileSAM Interactive Segmentation')}
          </div>
        </div>
      </div>

      {/* 3. Reconstruct Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onTriggerReconstruct}
          disabled={!canReconstruct}
          className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
        >
          {isReconstructing ? <RefreshCw size={14} className="animate-spin text-sky-400" /> : <Box size={14} className="text-indigo-400" />}
        </button>
        <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
          <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
            {!hasImage
              ? 'Capture RGB & depth first'
              : !hasMask
              ? 'Segment mask required to reconstruct 3D'
              : 'Surface Poisson 3D Mesh Reconstruction'}
          </div>
        </div>
      </div>

      {/* 4. Auto Waypoint Path Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onTriggerAutoPath}
          disabled={!canAutoPath}
          className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
        >
          {isAutoGenerating ? <RefreshCw size={14} className="animate-spin text-emerald-400" /> : <AutoPathIcon size={16} className="text-emerald-400" />}
        </button>
        <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
          <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
            {!hasImage
              ? 'Capture RGB & depth first'
              : !hasMask
              ? 'Segment mask required to generate path'
              : !hasMesh
              ? '3D mesh required to generate path'
              : 'Auto Generate Spray Paths from Mesh + Mask'}
          </div>
        </div>
      </div>

      {/* 5. Manual TCP Path Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onToggleManualPathMode}
          disabled={!canManualPath}
          className={`w-full h-8 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border ${
            manualPathMode
              ? 'bg-slate-800 text-amber-300 border-amber-500 shadow-md shadow-amber-950/40 ring-1 ring-amber-500/40'
              : 'bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 border-slate-700 hover:border-slate-600'
          }`}
        >
          {manualPathMode ? <X size={14} className="text-rose-400" /> : <ManualPathIcon size={14} className="text-amber-400" />}
        </button>
        <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
          <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
            {!hasImage
              ? 'Capture RGB & depth first'
              : !hasMesh
              ? '3D mesh required for manual path design'
              : manualPathMode
              ? 'Exit Manual TCP Design'
              : 'Manual TCP Path & Normal Design'}
          </div>
        </div>
      </div>
    </div>
  );
};
