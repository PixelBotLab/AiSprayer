import React from 'react';
import {
  Camera,
  Sparkles,
  Route,
  ShieldCheck,
  Box,
  RefreshCw,
  X,
  Undo2,
  Check,
  Trash2,
  ZoomIn,
  ZoomOut,
  Eye,
  EyeOff,
  Plus,
  Minus,
  Save,
} from 'lucide-react';
import type { WaypointItem, Point } from './types';

interface InteractiveActionColumnProps {
  hasImage: boolean;
  isCapturing: boolean;
  isReconstructing: boolean;
  segMode: boolean;
  manualPathMode: boolean;
  showDiagnostics: boolean;
  currentPoints: Point[];
  currentManualPoints: WaypointItem[];
  selectedPathIdForEdit: number | null;
  manualPathsCount: number;
  standoffDistMm: number;
  showMasksOverlay: boolean;
  showManualPathsOverlay: boolean;
  zoom: number;
  onTriggerCapture: () => void;
  onToggleSegMode: () => void;
  onToggleManualPathMode: () => void;
  onToggleDiagnostics: () => void;
  onTriggerReconstruct: () => void;
  onZoomIn: () => void;
  onZoomOut: () => void;
  onResetZoom: () => void;
  onToggleMasksOverlay: () => void;
  onToggleManualPathsOverlay: () => void;
  onUndoSegPoint: () => void;
  onClearCurrentSegPoints: () => void;
  onCommitCurrentSegMask: () => void;
  onSaveAllSegMasks: () => void;
  onUndoManualPoint: () => void;
  onClearCurrentManualPoints: () => void;
  onCommitManualPath: () => void;
  onSaveManualPaths: () => void;
  onDeleteCurrentPath: () => void;
  setStandoffDistMm: (dist: number) => void;
}

export const InteractiveActionColumn: React.FC<InteractiveActionColumnProps> = ({
  hasImage,
  isCapturing,
  isReconstructing,
  segMode,
  manualPathMode,
  showDiagnostics,
  currentPoints,
  currentManualPoints,
  selectedPathIdForEdit,
  manualPathsCount,
  standoffDistMm,
  showMasksOverlay,
  showManualPathsOverlay,
  zoom,
  onTriggerCapture,
  onToggleSegMode,
  onToggleManualPathMode,
  onToggleDiagnostics,
  onTriggerReconstruct,
  onZoomIn,
  onZoomOut,
  onResetZoom,
  onToggleMasksOverlay,
  onToggleManualPathsOverlay,
  onUndoSegPoint,
  onClearCurrentSegPoints,
  onCommitCurrentSegMask,
  onSaveAllSegMasks,
  onUndoManualPoint,
  onClearCurrentManualPoints,
  onCommitManualPath,
  onSaveManualPaths,
  onDeleteCurrentPath,
  setStandoffDistMm,
}) => {
  return (
    <>
      {/* 1. Right Action Buttons Column (Clean Slate Industrial Theme) */}
      <div className="absolute right-4 top-4 z-20 flex flex-col gap-2 w-32 select-none">
        {/* Capture Button */}
        <div className="relative group w-full">
          <button
            onClick={onTriggerCapture}
            disabled={isCapturing || segMode || manualPathMode}
            className="w-full py-2.5 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-40 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
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

        {/* Segment Button */}
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
              {segMode ? 'Exit Interactive SAM' : 'Interactive SAM Segmentation'}
            </div>
          </div>
        </div>

        {/* Manual TCP Path Button */}
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

        {/* TCP Diagnostics & Optimization Button */}
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

        {/* 3D Reconstruction Button */}
        <div className="relative group w-full">
          <button
            onClick={onTriggerReconstruct}
            disabled={!hasImage || isCapturing || isReconstructing || segMode || manualPathMode}
            className="w-full py-2.5 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-40 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
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

      {/* 2. Floating Zoom & Layer Controls */}
      <div className="absolute left-4 top-4 z-20 flex items-center gap-1 bg-slate-900/80 backdrop-blur border border-slate-700/80 rounded-lg p-1 shadow-lg text-slate-300">
        <button onClick={onZoomIn} className="p-1.5 hover:bg-slate-800 rounded text-slate-300 hover:text-white" title="Zoom In">
          <ZoomIn size={14} />
        </button>
        <button onClick={onZoomOut} className="p-1.5 hover:bg-slate-800 rounded text-slate-300 hover:text-white" title="Zoom Out">
          <ZoomOut size={14} />
        </button>
        <button onClick={onResetZoom} className="px-2 py-1 hover:bg-slate-800 rounded font-mono text-[11px] text-slate-300 hover:text-white" title="Reset Zoom">
          {(zoom * 100).toFixed(0)}%
        </button>
        <div className="w-[1px] h-4 bg-slate-700 mx-1" />
        <button onClick={onToggleMasksOverlay} className={`p-1.5 rounded transition-colors ${showMasksOverlay ? 'bg-sky-500/20 text-sky-300' : 'text-slate-500 hover:text-slate-300'}`} title="Toggle Mask Overlay">
          {showMasksOverlay ? <Eye size={14} /> : <EyeOff size={14} />}
        </button>
        <button onClick={onToggleManualPathsOverlay} className={`p-1.5 rounded transition-colors ${showManualPathsOverlay ? 'bg-amber-500/20 text-amber-300' : 'text-slate-500 hover:text-slate-300'}`} title="Toggle Manual Paths Overlay">
          <Route size={14} />
        </button>
      </div>

      {/* 3. Floating Bottom Toolbar: SAM Segmentation Mode */}
      {segMode && (
        <div className="absolute bottom-6 left-1/2 -translate-x-1/2 z-30 bg-slate-900/90 backdrop-blur-md border border-slate-700/80 rounded-2xl p-2 shadow-2xl flex items-center gap-3 text-xs text-slate-200">
          <div className="flex items-center gap-2 px-3 border-r border-slate-700">
            <span className="w-2.5 h-2.5 rounded-full bg-emerald-500 inline-block animate-pulse" />
            <span className="font-medium text-sky-300">Left Click: FG</span>
            <span className="w-2.5 h-2.5 rounded-full bg-rose-500 inline-block ml-2" />
            <span className="font-medium text-slate-400">Right Click: BG</span>
          </div>

          <div className="flex items-center gap-1">
            <button onClick={onUndoSegPoint} disabled={currentPoints.length === 0} className="px-2.5 py-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-300 disabled:opacity-40 flex items-center gap-1 font-medium transition-colors">
              <Undo2 size={13} />
              <span>Undo</span>
            </button>
            <button onClick={onClearCurrentSegPoints} disabled={currentPoints.length === 0} className="px-2.5 py-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-rose-300 disabled:opacity-40 flex items-center gap-1 font-medium transition-colors">
              <Trash2 size={13} />
              <span>Clear</span>
            </button>
            <button onClick={onCommitCurrentSegMask} disabled={currentPoints.length === 0} className="px-3 py-1.5 rounded-lg bg-sky-600 hover:bg-sky-500 text-white font-medium flex items-center gap-1.5 shadow-md shadow-sky-900/40 disabled:opacity-40 transition-all">
              <Check size={13} />
              <span>Commit Mask</span>
            </button>
            <button onClick={onSaveAllSegMasks} className="px-3 py-1.5 rounded-lg bg-emerald-600 hover:bg-emerald-500 text-white font-medium flex items-center gap-1.5 shadow-md shadow-emerald-900/40 transition-all ml-1">
              <Save size={13} />
              <span>Save Masks</span>
            </button>
          </div>
        </div>
      )}

      {/* 4. Floating Bottom Toolbar: Manual TCP Path Designer */}
      {manualPathMode && (
        <div className="absolute bottom-6 left-1/2 -translate-x-1/2 z-30 bg-slate-900/90 backdrop-blur-md border border-amber-500/40 rounded-2xl p-2 shadow-2xl flex items-center gap-3 text-xs text-slate-200">
          <div className="flex items-center gap-2 px-3 border-r border-slate-700">
            <Route size={14} className="text-amber-400 animate-pulse" />
            <span className="font-medium text-amber-300">
              {selectedPathIdForEdit ? `Editing Path P${selectedPathIdForEdit}` : `Path P${manualPathsCount + 1}`}
            </span>
            <span className="text-[11px] text-slate-400 font-mono">
              ({currentManualPoints.length} points)
            </span>
          </div>

          {/* Standoff Distance Adjustment */}
          <div className="flex items-center gap-1.5 px-2 border-r border-slate-700">
            <span className="text-[11px] text-slate-400 font-mono">Standoff:</span>
            <button onClick={() => setStandoffDistMm(Math.max(50, standoffDistMm - 10))} className="p-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300">
              <Minus size={11} />
            </button>
            <span className="font-mono text-amber-400 text-xs px-1">{standoffDistMm}mm</span>
            <button onClick={() => setStandoffDistMm(Math.min(300, standoffDistMm + 10))} className="p-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300">
              <Plus size={11} />
            </button>
          </div>

          <div className="flex items-center gap-1">
            <button onClick={onUndoManualPoint} disabled={currentManualPoints.length === 0} className="px-2.5 py-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-300 disabled:opacity-40 flex items-center gap-1 font-medium transition-colors">
              <Undo2 size={13} />
              <span>Undo</span>
            </button>
            <button onClick={onClearCurrentManualPoints} disabled={currentManualPoints.length === 0} className="px-2.5 py-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-rose-300 disabled:opacity-40 flex items-center gap-1 font-medium transition-colors">
              <Trash2 size={13} />
              <span>Clear</span>
            </button>
            <button onClick={onCommitManualPath} disabled={currentManualPoints.length === 0} className="px-3 py-1.5 rounded-lg bg-amber-600 hover:bg-amber-500 text-white font-medium flex items-center gap-1.5 shadow-md shadow-amber-900/40 disabled:opacity-40 transition-all">
              <Check size={13} />
              <span>{selectedPathIdForEdit ? 'Update Path' : 'Commit Path'}</span>
            </button>
            <button onClick={onSaveManualPaths} className="px-3 py-1.5 rounded-lg bg-emerald-600 hover:bg-emerald-500 text-white font-medium flex items-center gap-1.5 shadow-md shadow-emerald-900/40 transition-all ml-1">
              <Save size={13} />
              <span>Save Paths</span>
            </button>
            {selectedPathIdForEdit && (
              <button onClick={onDeleteCurrentPath} className="px-2.5 py-1.5 rounded-lg bg-rose-600/80 hover:bg-rose-600 text-white font-medium flex items-center gap-1 transition-all ml-1" title="Delete Selected Path">
                <Trash2 size={13} />
              </button>
            )}
          </div>
        </div>
      )}
    </>
  );
};
