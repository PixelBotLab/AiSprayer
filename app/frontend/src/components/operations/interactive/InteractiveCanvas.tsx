import React, { useRef, useEffect, type MouseEvent, type WheelEvent } from 'react';
import {
  RefreshCw,
  ZoomIn,
  ZoomOut,
  Eye,
  EyeOff,
  Route,
  Undo2,
  Trash2,
  Check,
  Save,
  X,
  Camera,
  Boxes,
  Sparkles,
  CheckCircle2,
  AlertTriangle,
  AlertOctagon,
  Info,
} from 'lucide-react';
import type {
  MaskData,
  ManualPathItem,
  WaypointItem,
  VerificationReport,
  Point,
  LiveNormalInfo,
  PathStateType,
  SimulationState,
  SessionData,
  CanvasNotice,
} from './types';
import { PATH_PALETTE } from './types';
import { PathStateSwitcher } from './PathStateSwitcher';
import { SamMaskOverlay } from './SamMaskOverlay';
import { PathSvgOverlay } from './PathSvgOverlay';
import { TOOLBAR_TIP_CLASS } from './ToolbarTip';

interface InteractiveCanvasProps {
  imageUrl: string | null;
  isLoadingTemplate: boolean;
  isCapturing?: boolean;
  isReconstructing?: boolean;
  isAutoGenerating?: boolean;
  isVerifying?: boolean;
  isOptimizing?: boolean;
  canvasNotice?: CanvasNotice | null;
  onDismissNotice?: () => void;
  segMode: boolean;

  manualPathMode: boolean;
  showMasksOverlay: boolean;
  showManualPathsOverlay: boolean;
  savedMasks: MaskData[];
  committedMasks: MaskData[];
  currentPolygons: number[][][];
  currentPoints: Point[];
  manualPaths: ManualPathItem[];
  currentManualPoints: WaypointItem[];
  selectedPathIdForEdit: number | null;
  highlightedPathId: number | null;
  hoveredWaypoint: WaypointItem | null;
  mousePixel: { u: number; v: number } | null;
  liveNormal: LiveNormalInfo | null;
  natSize: { w: number; h: number } | null;
  verificationReport: VerificationReport | null;
  activeState?: PathStateType;
  simulationState?: SimulationState | null;
  sessionData?: SessionData | null;
  onSelectActiveState?: (state: PathStateType) => void;
  zoom: number;
  pan: { x: number; y: number };
  isPanning: boolean;
  isSpacePressed: boolean;
  setZoom: React.Dispatch<React.SetStateAction<number>>;
  setPan: React.Dispatch<React.SetStateAction<{ x: number; y: number }>>;
  setIsPanning: React.Dispatch<React.SetStateAction<boolean>>;
  setNatSize: React.Dispatch<React.SetStateAction<{ w: number; h: number } | null>>;
  setHighlightedPathId: (id: number | null) => void;
  setHoveredWaypoint: (wp: WaypointItem | null) => void;
  onSelectPathForEdit?: (pathId: number) => void;
  onManualMouseMove?: (e: MouseEvent<SVGSVGElement>) => void;
  onManualImageClick?: (e: MouseEvent<SVGSVGElement>) => void;
  onDeleteSingleWaypoint?: (idx: number) => void;
  onSegImageClick?: (e: MouseEvent<SVGSVGElement>) => void;
  onSegContextMenu?: (e: MouseEvent<SVGSVGElement>) => void;
  onToggleMasksOverlay: () => void;
  onToggleManualPathsOverlay: () => void;
  onUndoSegPoint: () => void;
  onClearCurrentSegPoints: () => void;
  onClearAllMasks: () => void;
  onCommitCurrentSegMask: () => void;
  onSaveAllSegMasks: () => void;
  onExitSegMode: () => void;
  onUndoManualPoint: () => void;
  onClearCurrentManualPoints: () => void;
  onCommitManualPath: () => void;
  onSaveManualPaths: () => void;
  onDeleteCurrentPath: () => void;
  onExitManualPathMode: () => void;
}

export const InteractiveCanvas: React.FC<InteractiveCanvasProps> = ({
  imageUrl,
  isLoadingTemplate,
  isCapturing = false,
  isReconstructing = false,
  isAutoGenerating = false,
  isVerifying = false,
  isOptimizing = false,
  canvasNotice = null,
  onDismissNotice,
  segMode,
  manualPathMode,
  showMasksOverlay,
  showManualPathsOverlay,
  savedMasks,
  committedMasks,
  currentPolygons,
  currentPoints,
  manualPaths,
  currentManualPoints,
  selectedPathIdForEdit,
  highlightedPathId,
  hoveredWaypoint,
  mousePixel,
  liveNormal,
  natSize,
  verificationReport,
  activeState = 'raw',
  simulationState = null,
  sessionData = null,
  onSelectActiveState,
  zoom,
  pan,
  isPanning,
  isSpacePressed,
  setZoom,
  setPan,
  setIsPanning,
  setNatSize,
  setHighlightedPathId,
  setHoveredWaypoint,
  onSelectPathForEdit,
  onManualMouseMove,
  onManualImageClick,
  onDeleteSingleWaypoint,
  onSegImageClick,
  onSegContextMenu,
  onToggleMasksOverlay,
  onToggleManualPathsOverlay,
  onUndoSegPoint,
  onClearCurrentSegPoints,
  onClearAllMasks,
  onCommitCurrentSegMask,
  onSaveAllSegMasks,
  onExitSegMode,
  onUndoManualPoint,
  onClearCurrentManualPoints,
  onCommitManualPath,
  onSaveManualPaths,
  onDeleteCurrentPath,
  onExitManualPathMode,
}) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const panStartRef = useRef({ startX: 0, startY: 0, initialX: 0, initialY: 0 });

  // Global mouse dragging listeners
  useEffect(() => {
    const handleGlobalMouseMove = (e: globalThis.MouseEvent) => {
      if (isPanning) {
        const dx = e.clientX - panStartRef.current.startX;
        const dy = e.clientY - panStartRef.current.startY;
        setPan({
          x: panStartRef.current.initialX + dx,
          y: panStartRef.current.initialY + dy,
        });
      }
    };

    const handleGlobalMouseUp = () => {
      if (isPanning) {
        setIsPanning(false);
      }
    };

    window.addEventListener('mousemove', handleGlobalMouseMove);
    window.addEventListener('mouseup', handleGlobalMouseUp);
    return () => {
      window.removeEventListener('mousemove', handleGlobalMouseMove);
      window.removeEventListener('mouseup', handleGlobalMouseUp);
    };
  }, [isPanning, setIsPanning, setPan]);

  const handleMouseDown = (e: MouseEvent<HTMLDivElement>) => {
    if ((e.button === 0 && !segMode && !manualPathMode) || e.button === 1 || isSpacePressed) {
      e.preventDefault();
      setIsPanning(true);
      panStartRef.current = {
        startX: e.clientX,
        startY: e.clientY,
        initialX: pan.x,
        initialY: pan.y,
      };
    }
  };

  const handleWheel = (e: WheelEvent<HTMLDivElement>) => {
    e.preventDefault();
    if (!containerRef.current) return;

    const rect = containerRef.current.getBoundingClientRect();
    const cursorX = e.clientX - rect.left - rect.width / 2;
    const cursorY = e.clientY - rect.top - rect.height / 2;

    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
    const newZoom = Math.min(Math.max(0.2, zoom * zoomFactor), 15);

    const newPanX = cursorX - (cursorX - pan.x) * (newZoom / zoom);
    const newPanY = cursorY - (cursorY - pan.y) * (newZoom / zoom);

    setZoom(newZoom);
    setPan({ x: newPanX, y: newPanY });
  };

  return (
    <div
      ref={containerRef}
      className={`flex-1 min-w-0 h-full flex flex-col border-r border-slate-800 relative bg-black items-center justify-center overflow-hidden select-none ${
        isPanning
          ? 'cursor-grabbing'
          : segMode || manualPathMode
          ? isSpacePressed
            ? 'cursor-grab'
            : 'cursor-crosshair'
          : 'cursor-grab'
      }`}
      onWheel={handleWheel}
      onMouseDown={handleMouseDown}
    >
      {/* 1. Floating Top-Left Zoom & Layer Controls (Ultra-compact h-6 at left-2 top-2) */}
      <div className="absolute left-2 top-2 z-20 flex items-center gap-0.5 bg-slate-950/70 hover:bg-slate-950/85 backdrop-blur-md border border-white/10 rounded-md px-1 py-0.5 shadow-xl text-slate-300 h-6 select-none transition-all">
        <button
          onClick={() => setZoom((z) => Math.min(15, z * 1.25))}
          className="p-1 hover:bg-white/10 rounded text-slate-400 hover:text-slate-200 transition-colors"
          title="Zoom In"
        >
          <ZoomIn size={11} />
        </button>
        <button
          onClick={() => setZoom((z) => Math.max(0.2, z * 0.8))}
          className="p-1 hover:bg-white/10 rounded text-slate-400 hover:text-slate-200 transition-colors"
          title="Zoom Out"
        >
          <ZoomOut size={11} />
        </button>
        <button
          onClick={() => {
            setZoom(1);
            setPan({ x: 0, y: 0 });
          }}
          className="px-1 hover:bg-white/10 rounded font-mono text-[9px] text-slate-300 hover:text-white transition-colors"
          title="Reset Zoom & Pan"
        >
          {(zoom * 100).toFixed(0)}%
        </button>
        <div className="w-[1px] h-2.5 bg-white/10 mx-0.5" />
        <button
          onClick={onToggleMasksOverlay}
          disabled={savedMasks.length === 0 && committedMasks.length === 0}
          className={`p-1 rounded transition-colors ${
            savedMasks.length === 0 && committedMasks.length === 0
              ? 'text-slate-700 cursor-not-allowed'
              : showMasksOverlay ? 'bg-sky-500/25 text-sky-300' : 'text-slate-500 hover:text-slate-300'
          }`}
          title={savedMasks.length === 0 && committedMasks.length === 0 ? 'No masks available' : 'Toggle Mask Overlay'}
        >
          {showMasksOverlay ? <Eye size={11} /> : <EyeOff size={11} />}
        </button>
        <button
          onClick={onToggleManualPathsOverlay}
          disabled={manualPaths.length === 0}
          className={`p-1 rounded transition-colors ${
            manualPaths.length === 0
              ? 'text-slate-700 cursor-not-allowed'
              : showManualPathsOverlay ? 'bg-amber-500/25 text-amber-300' : 'text-slate-500 hover:text-slate-300'
          }`}
          title={manualPaths.length === 0 ? 'No paths available' : 'Toggle Manual Paths Overlay'}
        >
          <Route size={11} />
        </button>
      </div>

      {onSelectActiveState && activeState && (
        <div className="absolute right-2 top-2 z-20 flex items-center gap-0.5 bg-slate-950/70 hover:bg-slate-950/85 backdrop-blur-md border border-white/10 rounded-md px-1 py-0.5 shadow-xl text-slate-300 h-6 select-none transition-all">
          <PathStateSwitcher
            activeState={activeState}
            onSelect={onSelectActiveState}
            compact
          />
        </div>
      )}

      {/* 2. Atomic Loading Barrier Overlay */}
      {isLoadingTemplate && (
        <div className="absolute inset-0 bg-slate-950/60 backdrop-blur-[2px] flex items-center justify-center z-40 transition-opacity pointer-events-none">
          <div className="flex items-center gap-1.5 px-3 py-1 rounded-full bg-slate-900/90 border border-slate-700 text-slate-300 text-[11px] shadow-lg">
            <RefreshCw size={12} className="animate-spin text-sky-400" />
            <span>Loading...</span>
          </div>
        </div>
      )}

      {/* Action Animation Overlays in Main Canvas (Ultra-compact HUD badges) */}
      {/* Camera Capture Effect */}
      {isCapturing && (
        <div className="absolute inset-0 z-40 pointer-events-none overflow-hidden select-none">
          <div className="absolute inset-0 bg-white/25 animate-camera-shutter-flash" />
          <div className="absolute inset-x-0 h-0.5 bg-gradient-to-r from-transparent via-cyan-400 to-transparent shadow-[0_0_12px_#38bdf8] animate-canvas-scanline" />
          <div className="absolute top-2 left-1/2 -translate-x-1/2 flex items-center gap-1.5 px-3 py-1 rounded-full bg-slate-900/90 border border-cyan-500/40 backdrop-blur-md shadow-lg text-cyan-300 text-[11px] font-medium animate-pulse">
            <Camera size={12} className="animate-spin text-cyan-400" />
            <span>Capturing...</span>
          </div>
        </div>
      )}

      {/* 3D Mesh Reconstruction Effect */}
      {isReconstructing && (
        <div className="absolute inset-0 z-40 pointer-events-none overflow-hidden select-none">
          <div className="absolute inset-0 animate-cyber-grid opacity-40" />
          <div className="absolute inset-x-0 h-0.5 bg-gradient-to-r from-transparent via-emerald-400 to-transparent shadow-[0_0_12px_#34d399] animate-canvas-scanline" />
          <div className="absolute top-2 left-1/2 -translate-x-1/2 flex items-center gap-1.5 px-3 py-1 rounded-full bg-slate-900/90 border border-emerald-500/40 backdrop-blur-md shadow-lg text-emerald-300 text-[11px] font-medium">
            <Boxes size={12} className="animate-bounce text-emerald-400" />
            <span>Reconstructing 3D...</span>
          </div>
        </div>
      )}

      {/* Auto Path Generation Effect */}
      {isAutoGenerating && (
        <div className="absolute inset-0 z-40 pointer-events-none overflow-hidden select-none">
          <div className="absolute inset-x-0 h-0.5 bg-gradient-to-r from-transparent via-amber-400 to-transparent shadow-[0_0_12px_#fbbf24] animate-canvas-scanline" />
          <div className="absolute top-2 left-1/2 -translate-x-1/2 flex items-center gap-1.5 px-3 py-1 rounded-full bg-slate-900/90 border border-amber-500/40 backdrop-blur-md shadow-lg text-amber-300 text-[11px] font-medium">
            <Sparkles size={12} className="animate-spin text-amber-400" />
            <span>Generating paths...</span>
          </div>
        </div>
      )}

      {/* Kinematics Verification Effect */}
      {isVerifying && (
        <div className="absolute inset-0 z-40 pointer-events-none overflow-hidden select-none">
          <div className="absolute inset-x-0 h-0.5 bg-gradient-to-r from-transparent via-sky-400 to-transparent shadow-[0_0_12px_#38bdf8] animate-canvas-scanline" />
          <div className="absolute top-2 left-1/2 -translate-x-1/2 flex items-center gap-1.5 px-3 py-1 rounded-full bg-slate-900/90 border border-sky-500/40 backdrop-blur-md shadow-lg text-sky-300 text-[11px] font-medium">
            <RefreshCw size={12} className="animate-spin text-sky-400" />
            <span>Verifying kinematics...</span>
          </div>
        </div>
      )}

      {/* POI Optimization Effect */}
      {isOptimizing && (
        <div className="absolute inset-0 z-40 pointer-events-none overflow-hidden select-none">
          <div className="absolute inset-x-0 h-0.5 bg-gradient-to-r from-transparent via-purple-400 to-transparent shadow-[0_0_12px_#c084fc] animate-canvas-scanline" />
          <div className="absolute top-2 left-1/2 -translate-x-1/2 flex items-center gap-1.5 px-3 py-1 rounded-full bg-slate-900/90 border border-purple-500/40 backdrop-blur-md shadow-lg text-purple-300 text-[11px] font-medium">
            <Sparkles size={12} className="animate-spin text-purple-400" />
            <span>Optimizing POI...</span>
          </div>
        </div>
      )}

      {/* Floating In-Canvas HUD Notification (Compact Pill) */}
      {canvasNotice && (
        <div
          className={`absolute top-2 left-1/2 -translate-x-1/2 z-50 pointer-events-auto flex items-center gap-1.5 px-3 py-1 rounded-full border backdrop-blur-md shadow-lg animate-hud-slide-in select-none max-w-[80%] transition-all ${
            canvasNotice.type === 'success'
              ? 'bg-slate-900/90 border-emerald-500/40 text-emerald-300'
              : canvasNotice.type === 'error'
              ? 'bg-slate-900/90 border-rose-500/40 text-rose-300'
              : canvasNotice.type === 'warning'
              ? 'bg-slate-900/90 border-amber-500/40 text-amber-300'
              : 'bg-slate-900/90 border-sky-500/40 text-sky-300'
          }`}
        >
          <div className="shrink-0">
            {canvasNotice.type === 'success' && <CheckCircle2 size={12} className="text-emerald-400" />}
            {canvasNotice.type === 'error' && <AlertOctagon size={12} className="text-rose-400" />}
            {canvasNotice.type === 'warning' && <AlertTriangle size={12} className="text-amber-400" />}
            {canvasNotice.type === 'info' && <Info size={12} className="text-sky-400" />}
          </div>
          <span className="truncate max-w-[320px] text-[11px] font-medium leading-none text-slate-200">
            {canvasNotice.message}
          </span>
          {onDismissNotice && (
            <button
              onClick={onDismissNotice}
              className="shrink-0 p-0.5 text-slate-400 hover:text-white rounded-full hover:bg-white/10 transition-colors ml-0.5"
              title="Dismiss"
            >
              <X size={11} />
            </button>
          )}
        </div>
      )}

      {/* 3. Main Scaled Image Viewport */}
      {imageUrl ? (
        <div
          className="relative inline-block max-w-full max-h-full"
          style={{
            transform: `translate(${pan.x}px, ${pan.y}px) scale(${zoom})`,
            transformOrigin: 'center center',
            transition: isPanning ? 'none' : 'transform 0.05s ease-out',
          }}
        >
          <img
            src={imageUrl}
            className="block max-w-full max-h-full object-contain pointer-events-none"
            alt="Captured view"
            onLoad={(e) =>
              setNatSize({
                w: e.currentTarget.naturalWidth,
                h: e.currentTarget.naturalHeight,
              })
            }
          />

          {natSize && (
            <>
              {/* SAM Mask Overlay */}
              <SamMaskOverlay
                segMode={segMode}
                showMasksOverlay={showMasksOverlay}
                savedMasks={savedMasks}
                committedMasks={committedMasks}
                currentPolygons={currentPolygons}
                currentPoints={currentPoints}
                natSize={natSize}
                isReconstructing={isReconstructing}
                isAutoGenerating={isAutoGenerating}
                isVerifying={isVerifying}
                isOptimizing={isOptimizing}
                onSegImageClick={onSegImageClick}
                onSegContextMenu={onSegContextMenu}
              />

              {/* 2D Path SVG Overlay */}
              <PathSvgOverlay
                manualPathMode={manualPathMode}
                showManualPathsOverlay={showManualPathsOverlay}
                manualPaths={manualPaths}
                currentManualPoints={currentManualPoints}
                selectedPathIdForEdit={selectedPathIdForEdit}
                highlightedPathId={highlightedPathId}
                hoveredWaypoint={hoveredWaypoint}
                mousePixel={mousePixel}
                liveNormal={liveNormal}
                natSize={natSize}
                verificationReport={verificationReport}
                activeState={activeState}
                simulationState={simulationState}
                sessionData={sessionData}
                isVerifying={isVerifying}
                isOptimizing={isOptimizing}
                isAutoGenerating={isAutoGenerating}
                onSelectPathForEdit={onSelectPathForEdit}
                setHighlightedPathId={setHighlightedPathId}
                setHoveredWaypoint={setHoveredWaypoint}
                onManualMouseMove={onManualMouseMove}
                onManualImageClick={onManualImageClick}
                onDeleteSingleWaypoint={onDeleteSingleWaypoint}
              />
            </>
          )}
        </div>
      ) : (
        <div className="flex flex-col items-center justify-center text-slate-600 gap-2">
          <p className="text-xs">No scan image available for this template</p>
        </div>
      )}

      {(() => {
        const pt = hoveredWaypoint || (manualPathMode && currentManualPoints.length > 0 ? currentManualPoints[currentManualPoints.length - 1] : null);
        const live = !pt && manualPathMode && mousePixel && liveNormal?.tcpPose ? liveNormal : null;
        if (!pt && !live) return null;
        const tcp = pt?.tcp_pose_base || live?.tcpPose;
        if (!tcp) return null;
        const title = pt
          ? `P${pt.path_id ?? 1} #${pt.index}`
          : 'Cursor';
        const issue = pt
          ? verificationReport?.path_reports
              ?.find((r) => r.path_id === (pt.path_id ?? 1))
              ?.issues?.find((iss) => iss.waypoint_index === pt.index - 1)
          : null;
        return (
          <div className={`absolute left-2 top-10 z-20 flex items-center gap-1.5 h-6 max-w-[calc(100%-1rem)] overflow-hidden ${TOOLBAR_TIP_CLASS} pointer-events-none select-none`}>
            <Route size={11} className="shrink-0 text-slate-400" />
            <span className="text-slate-200">{title}</span>
            <span className="text-white/15">|</span>
            <span className="font-mono">{tcp.x.toFixed(1)}, {tcp.y.toFixed(1)}, {tcp.z.toFixed(1)}</span>
            <span className="text-white/15">|</span>
            <span className="font-mono">{tcp.rx.toFixed(1)}°, {tcp.ry.toFixed(1)}°, {tcp.rz.toFixed(1)}°</span>
            {pt && (
              <>
                <span className="text-white/15">|</span>
                <span className="font-mono text-slate-400">{pt.standoff_distance_mm}mm</span>
              </>
            )}
            {issue && (
              <>
                <span className="text-white/15">|</span>
                <span className="truncate text-rose-300">{issue.type}</span>
              </>
            )}
          </div>
        );
      })()}

      {/* 4. Sleek Minimal Floating Toolbar: SAM Segmentation Mode (h-7 rounded-full icon-only) */}
      {segMode && (
        <div className="absolute bottom-4 left-1/2 -translate-x-1/2 bg-slate-950/70 hover:bg-slate-950/85 backdrop-blur-md border border-sky-500/30 rounded-full px-2.5 h-7 flex items-center gap-1.5 shadow-2xl z-30 transition-all select-none">
          {/* Prompt indicators */}
          <div className="flex items-center gap-1 px-1 border-r border-white/10 text-[10px]">
            <span className="w-1.5 h-1.5 rounded-full bg-emerald-400 inline-block animate-pulse" title="Left Click: Foreground" />
            <span className="w-1.5 h-1.5 rounded-full bg-rose-400 inline-block ml-0.5" title="Right Click: Background" />
          </div>

          {/* Undo Point */}
          <div className="relative group flex items-center">
            <button
              onClick={onUndoSegPoint}
              disabled={currentPoints.length === 0}
              className="p-1 text-slate-300 hover:text-white hover:bg-white/10 rounded-full disabled:opacity-30 transition-colors"
            >
              <Undo2 size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                Undo Point
              </div>
            </div>
          </div>

          {/* Reset Current Points */}
          <div className="relative group flex items-center">
            <button
              onClick={onClearCurrentSegPoints}
              disabled={currentPoints.length === 0}
              className="p-1 text-amber-400 hover:text-amber-300 hover:bg-amber-400/20 rounded-full disabled:opacity-30 transition-colors"
            >
              <RefreshCw size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                Reset Points
              </div>
            </div>
          </div>

          {/* Commit Mask */}
          <div className="relative group flex items-center">
            <button
              onClick={onCommitCurrentSegMask}
              disabled={currentPolygons.length === 0}
              className="p-1 text-emerald-400 hover:text-emerald-300 hover:bg-emerald-400/20 rounded-full disabled:opacity-30 transition-colors"
            >
              <Check size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                Commit Current Mask
              </div>
            </div>
          </div>

          <div className="w-px h-3 bg-white/10 mx-0.5" />

          {/* Clear All Masks */}
          <div className="relative group flex items-center">
            <button
              onClick={onClearAllMasks}
              disabled={committedMasks.length === 0 && currentPoints.length === 0}
              className="p-1 text-rose-400 hover:text-rose-300 hover:bg-rose-400/20 rounded-full disabled:opacity-30 transition-colors"
            >
              <Trash2 size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                Clear All Masks
              </div>
            </div>
          </div>

          {/* Save Masks to YAML */}
          <div className="relative group flex items-center">
            <button
              onClick={onSaveAllSegMasks}
              disabled={committedMasks.length === 0 && currentPolygons.length === 0}
              className="p-1 bg-sky-600/80 hover:bg-sky-600 text-white rounded-full shadow-lg shadow-sky-900/30 border border-sky-400/30 disabled:opacity-30 transition-all active:scale-95"
            >
              <Save size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                Save Masks to YAML
              </div>
            </div>
          </div>

          {/* Exit Seg Mode */}
          <div className="relative group flex items-center">
            <button
              onClick={onExitSegMode}
              className="p-1 text-slate-400 hover:text-slate-200 hover:bg-white/10 rounded-full transition-colors"
            >
              <X size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                Exit Segmentation
              </div>
            </div>
          </div>
        </div>
      )}

      {/* 5. Sleek Minimal Floating Toolbar: Manual TCP Path Designer (h-7 rounded-full icon-only) */}
      {manualPathMode && (
        <div className="absolute bottom-4 left-1/2 -translate-x-1/2 bg-slate-950/70 hover:bg-slate-950/85 backdrop-blur-md border border-amber-500/30 rounded-full px-2.5 h-7 flex items-center gap-1.5 shadow-2xl z-30 transition-all select-none">
          {/* Existing Paths Chips */}
          {manualPaths.length > 0 && (
            <div className="flex items-center gap-1 px-1 border-r border-white/10 max-w-[240px] overflow-x-auto custom-scrollbar">
              {manualPaths.map((p) => {
                const isSelected = selectedPathIdForEdit === p.path_id;
                const color = PATH_PALETTE[(p.path_id - 1) % PATH_PALETTE.length];
                return (
                  <div
                    key={p.path_id}
                    onClick={() => onSelectPathForEdit && onSelectPathForEdit(p.path_id)}
                    className={`flex items-center gap-1 px-2 py-0.2 rounded-full border text-[9px] cursor-pointer select-none transition-all ${
                      isSelected
                        ? 'bg-amber-950/90 border-amber-400 text-amber-100 shadow-md ring-1 ring-amber-400/50'
                        : 'bg-slate-900/90 border-white/10 text-slate-300 hover:border-slate-500 hover:text-white'
                    }`}
                    title={`Click to edit Path ${p.path_id}`}
                  >
                    <span className="w-1.5 h-1.5 rounded-full" style={{ backgroundColor: color }} />
                    <span className="font-bold">P{p.path_id}</span>
                  </div>
                );
              })}
            </div>
          )}

          {/* Undo Waypoint */}
          <div className="relative group flex items-center">
            <button
              onClick={onUndoManualPoint}
              disabled={currentManualPoints.length === 0}
              className="p-1 text-slate-300 hover:text-white hover:bg-white/10 rounded-full disabled:opacity-30 transition-colors"
            >
              <Undo2 size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                Undo Waypoint
              </div>
            </div>
          </div>

          {/* Clear Current Waypoints */}
          <div className="relative group flex items-center">
            <button
              onClick={onClearCurrentManualPoints}
              disabled={currentManualPoints.length === 0}
              className="p-1 text-amber-400 hover:text-amber-300 hover:bg-amber-400/20 rounded-full disabled:opacity-30 transition-colors"
            >
              <RefreshCw size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                Clear Current Points
              </div>
            </div>
          </div>

          {/* Commit Path */}
          <div className="relative group flex items-center">
            <button
              onClick={onCommitManualPath}
              disabled={currentManualPoints.length === 0}
              className="p-1 text-emerald-400 hover:text-emerald-300 hover:bg-emerald-400/20 rounded-full disabled:opacity-30 transition-colors"
            >
              <Check size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                {selectedPathIdForEdit ? 'Update Path' : 'Commit New Path'}
              </div>
            </div>
          </div>

          <div className="w-px h-3 bg-white/10 mx-0.5" />

          {/* Delete Path */}
          {selectedPathIdForEdit && (
            <div className="relative group flex items-center">
              <button
                onClick={onDeleteCurrentPath}
                className="p-1 text-rose-400 hover:text-rose-300 hover:bg-rose-400/20 rounded-full transition-colors"
              >
                <Trash2 size={12} />
              </button>
              <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                  Delete Path {selectedPathIdForEdit}
                </div>
              </div>
            </div>
          )}

          {/* Save Paths */}
          <div className="relative group flex items-center">
            <button
              onClick={onSaveManualPaths}
              disabled={manualPaths.length === 0 && currentManualPoints.length === 0}
              className="p-1 bg-amber-600/80 hover:bg-amber-600 text-white rounded-full shadow-lg shadow-amber-900/30 border border-amber-400/30 disabled:opacity-30 transition-all active:scale-95"
            >
              <Save size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                Save Paths to YAML
              </div>
            </div>
          </div>

          {/* Exit Manual Mode */}
          <div className="relative group flex items-center">
            <button
              onClick={onExitManualPathMode}
              className="p-1 text-slate-400 hover:text-slate-200 hover:bg-white/10 rounded-full transition-colors"
            >
              <X size={12} />
            </button>
            <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                Exit Manual TCP
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};
