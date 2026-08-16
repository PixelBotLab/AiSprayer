import React from 'react';
import {
  Camera,
  Sparkles,
  Route,
  ShieldCheck,
  Box,
  RefreshCw,
  X,
} from 'lucide-react';

interface InteractiveActionColumnProps {
  hasImage: boolean;
  activeTemplate: string | null;
  isCapturing: boolean;
  isReconstructing: boolean;
  segMode: boolean;
  manualPathMode: boolean;
  showDiagnostics: boolean;
  onTriggerCapture: () => void;
  onToggleSegMode: () => void;
  onToggleManualPathMode: () => void;
  onToggleDiagnostics: () => void;
  onTriggerReconstruct: () => void;
}

export const InteractiveActionColumn: React.FC<InteractiveActionColumnProps> = ({
  hasImage,
  activeTemplate,
  isCapturing,
  isReconstructing,
  segMode,
  manualPathMode,
  showDiagnostics,
  onTriggerCapture,
  onToggleSegMode,
  onToggleManualPathMode,
  onToggleDiagnostics,
  onTriggerReconstruct,
}) => {
  return (
    <div className="w-full shrink-0 bg-slate-950/80 border-b border-slate-800/80 px-2 py-2 flex items-center justify-between gap-1.5 select-none">
      {/* 1. Capture Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onTriggerCapture}
          disabled={isCapturing || isReconstructing || !activeTemplate || segMode || manualPathMode}
          className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
        >
          {isCapturing ? <RefreshCw size={14} className="animate-spin text-sky-400" /> : <Camera size={14} className="text-slate-300" />}
        </button>
        <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
          <div className="bg-slate-950/95 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
            Capture 2D Color + 3D Depth Data
          </div>
        </div>
      </div>

      {/* 2. Segment Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onToggleSegMode}
          disabled={!hasImage || isCapturing || isReconstructing || manualPathMode}
          className={`w-full h-8 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border ${
            segMode
              ? 'bg-slate-800 text-sky-300 border-sky-500 shadow-md shadow-sky-950/40 ring-1 ring-sky-500/40'
              : 'bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 border-slate-700 hover:border-slate-600'
          }`}
        >
          {segMode ? <X size={14} className="text-rose-400" /> : <Sparkles size={14} className="text-sky-400" />}
        </button>
        <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
          <div className="bg-slate-950/95 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
            {segMode ? 'Exit Segmentation Mode' : 'MobileSAM Interactive Segmentation'}
          </div>
        </div>
      </div>

      {/* 3. Reconstruct Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onTriggerReconstruct}
          disabled={!hasImage || isCapturing || isReconstructing || segMode || manualPathMode}
          className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
        >
          {isReconstructing ? <RefreshCw size={14} className="animate-spin text-sky-400" /> : <Box size={14} className="text-indigo-400" />}
        </button>
        <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
          <div className="bg-slate-950/95 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
            Surface Poisson 3D Mesh Reconstruction
          </div>
        </div>
      </div>

      {/* 4. Manual TCP Path Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onToggleManualPathMode}
          disabled={!hasImage || isCapturing || isReconstructing || segMode}
          className={`w-full h-8 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border ${
            manualPathMode
              ? 'bg-slate-800 text-amber-300 border-amber-500 shadow-md shadow-amber-950/40 ring-1 ring-amber-500/40'
              : 'bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 border-slate-700 hover:border-slate-600'
          }`}
        >
          {manualPathMode ? <X size={14} className="text-rose-400" /> : <Route size={14} className="text-amber-400" />}
        </button>
        <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
          <div className="bg-slate-950/95 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
            {manualPathMode ? 'Exit Manual TCP Design' : 'Manual TCP Path & Normal Design'}
          </div>
        </div>
      </div>

      {/* 5. TCP Diagnostics & Optimization Button */}
      <div className="relative group flex-1 flex items-center justify-center">
        <button
          onClick={onToggleDiagnostics}
          disabled={!hasImage}
          className={`w-full h-8 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border ${
            showDiagnostics
              ? 'bg-slate-800 text-sky-300 border-sky-500 shadow-md shadow-sky-950/40 ring-1 ring-sky-500/40'
              : 'bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 border-slate-700 hover:border-slate-600'
          }`}
        >
          <ShieldCheck size={14} className="text-sky-400" />
        </button>
        <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
          <div className="bg-slate-950/95 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
            Toggle 6-DOF Kinematics & Auto-Fix Panel
          </div>
        </div>
      </div>
    </div>
  );
};
