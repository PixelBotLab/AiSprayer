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
    <div className="w-[125px] shrink-0 bg-slate-950/60 flex flex-col justify-end p-2.5 border-l border-slate-800 gap-2 select-none h-full">
      {/* 1. Capture Button */}
      <div className="relative group w-full">
        <button
          onClick={onTriggerCapture}
          disabled={isCapturing || isReconstructing || !activeTemplate || segMode || manualPathMode}
          className="w-full py-2.5 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-40 disabled:cursor-not-allowed border border-slate-700"
        >
          {isCapturing ? <RefreshCw size={14} className="animate-spin text-sky-400" /> : <Camera size={14} className="text-slate-300" />}
          <span>{isCapturing ? 'Wait...' : 'Capture'}</span>
        </button>
        <div className="absolute bottom-full mb-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
          <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
            Capture 2D Color + 3D Depth Data
          </div>
        </div>
      </div>

      {/* 2. Segment Button */}
      <div className="relative group w-full">
        <button
          onClick={onToggleSegMode}
          disabled={!hasImage || isCapturing || isReconstructing || manualPathMode}
          className={`w-full py-2.5 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-40 disabled:cursor-not-allowed border ${
            segMode
              ? 'bg-slate-800 text-sky-300 border-sky-500 shadow-md shadow-sky-950/40 ring-1 ring-sky-500/40'
              : 'bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 border-slate-700'
          }`}
        >
          {segMode ? <X size={14} className="text-rose-400" /> : <Sparkles size={14} className="text-sky-400" />}
          <span>{segMode ? 'Exit Seg' : 'Segment'}</span>
        </button>
        <div className="absolute bottom-full mb-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
          <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
            {segMode ? 'Exit Segmentation Mode' : 'MobileSAM Interactive Segmentation'}
          </div>
        </div>
      </div>

      {/* 3. Manual TCP Path Button */}
      <div className="relative group w-full">
        <button
          onClick={onToggleManualPathMode}
          disabled={!hasImage || isCapturing || isReconstructing || segMode}
          className={`w-full py-2.5 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-40 disabled:cursor-not-allowed border ${
            manualPathMode
              ? 'bg-slate-800 text-amber-300 border-amber-500 shadow-md shadow-amber-950/40 ring-1 ring-amber-500/40'
              : 'bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 border-slate-700'
          }`}
        >
          {manualPathMode ? <X size={14} className="text-rose-400" /> : <Route size={14} className="text-amber-400" />}
          <span>{manualPathMode ? 'Exit TCP' : 'Manual TCP'}</span>
        </button>
        <div className="absolute bottom-full mb-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
          <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
            {manualPathMode ? 'Exit Manual TCP Path Designer' : 'Manual TCP Path Planning'}
          </div>
        </div>
      </div>

      {/* 4. TCP Diagnostics & Optimization Button */}
      <div className="relative group w-full">
        <button
          onClick={onToggleDiagnostics}
          disabled={!hasImage}
          className={`w-full py-2.5 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-40 disabled:cursor-not-allowed border ${
            showDiagnostics
              ? 'bg-slate-800 text-sky-300 border-sky-500 shadow-md shadow-sky-950/40 ring-1 ring-sky-500/40'
              : 'bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 border-slate-700'
          }`}
        >
          <ShieldCheck size={14} className="text-sky-400" />
          <span>{showDiagnostics ? 'Hide Opt' : 'TCP Opt'}</span>
        </button>
        <div className="absolute bottom-full mb-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
          <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
            TCP Kinematics Verification & Tolerance Optimizer
          </div>
        </div>
      </div>

      {/* 5. 3D Reconstruction Button */}
      <div className="relative group w-full">
        <button
          onClick={onTriggerReconstruct}
          disabled={!hasImage || isCapturing || isReconstructing || segMode || manualPathMode}
          className="w-full py-2.5 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-40 disabled:cursor-not-allowed border border-slate-700"
        >
          {isReconstructing ? <RefreshCw size={14} className="animate-spin text-sky-400" /> : <Box size={14} className="text-indigo-400" />}
          <span>{isReconstructing ? 'Building...' : 'Reconstruct'}</span>
        </button>
        <div className="absolute bottom-full mb-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
          <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
            Generate 3D Surface Mesh (.ply)
          </div>
        </div>
      </div>
    </div>
  );
};
