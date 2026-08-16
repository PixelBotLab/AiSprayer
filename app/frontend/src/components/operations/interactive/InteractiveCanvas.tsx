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
  Plus,
  Minus,
} from 'lucide-react';
import type { MaskData, ManualPathItem, WaypointItem, VerificationReport, Point } from './types';
import { SamMaskOverlay } from './SamMaskOverlay';
import { PathSvgOverlay } from './PathSvgOverlay';

interface InteractiveCanvasProps {
  imageUrl: string | null;
  isLoadingTemplate: boolean;
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
  liveNormal: { dx: number; dy: number } | null;
  natSize: { w: number; h: number } | null;
  verificationReport: VerificationReport | null;
  zoom: number;
  pan: { x: number; y: number };
  isPanning: boolean;
  isSpacePressed: boolean;
  standoffDistMm: number;
  setZoom: React.Dispatch<React.SetStateAction<number>>;
  setPan: React.Dispatch<React.SetStateAction<{ x: number; y: number }>>;
  setIsPanning: React.Dispatch<React.SetStateAction<boolean>>;
  setNatSize: React.Dispatch<React.SetStateAction<{ w: number; h: number } | null>>;
  setHighlightedPathId: (id: number | null) => void;
  setHoveredWaypoint: (wp: WaypointItem | null) => void;
  setStandoffDistMm: (dist: number) => void;
  onSelectPathForEdit?: (pathId: number) => void;
  onManualMouseMove?: (e: MouseEvent<SVGSVGElement>) => void;
  onManualImageClick?: (e: MouseEvent<SVGSVGElement>) => void;
  onDeleteSingleWaypoint?: (idx: number) => void;
  onSegImageClick?: (e: MouseEvent<HTMLImageElement>) => void;
  onSegContextMenu?: (e: MouseEvent<HTMLImageElement>) => void;
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
  renderPolygons: (polygons: number[][][], fill: string, stroke?: string) => React.ReactNode;
}

export const InteractiveCanvas: React.FC<InteractiveCanvasProps> = ({
  imageUrl,
  isLoadingTemplate,
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
  zoom,
  pan,
  isPanning,
  isSpacePressed,
  standoffDistMm,
  setZoom,
  setPan,
  setIsPanning,
  setNatSize,
  setHighlightedPathId,
  setHoveredWaypoint,
  setStandoffDistMm,
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
  onCommitCurrentSegMask,
  onSaveAllSegMasks,
  onUndoManualPoint,
  onClearCurrentManualPoints,
  onCommitManualPath,
  onSaveManualPaths,
  onDeleteCurrentPath,
  renderPolygons,
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
    // Normal view mode: left-click drag freely pans
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
      {/* 1. Floating Top-Left Zoom & Layer Controls */}
      <div className="absolute left-3 top-3 z-20 flex items-center gap-1 bg-slate-900/80 backdrop-blur border border-slate-700/80 rounded-lg p-1 shadow-lg text-slate-300">
        <button
          onClick={() => setZoom((z) => Math.min(15, z * 1.25))}
          className="p-1.5 hover:bg-slate-800 rounded text-slate-300 hover:text-white"
          title="Zoom In"
        >
          <ZoomIn size={13} />
        </button>
        <button
          onClick={() => setZoom((z) => Math.max(0.2, z * 0.8))}
          className="p-1.5 hover:bg-slate-800 rounded text-slate-300 hover:text-white"
          title="Zoom Out"
        >
          <ZoomOut size={13} />
        </button>
        <button
          onClick={() => {
            setZoom(1);
            setPan({ x: 0, y: 0 });
          }}
          className="px-1.5 py-0.5 hover:bg-slate-800 rounded font-mono text-[10px] text-slate-300 hover:text-white"
          title="Reset Zoom & Pan"
        >
          {(zoom * 100).toFixed(0)}%
        </button>
        <div className="w-[1px] h-3.5 bg-slate-700 mx-0.5" />
        <button
          onClick={onToggleMasksOverlay}
          className={`p-1.5 rounded transition-colors ${
            showMasksOverlay ? 'bg-sky-500/20 text-sky-300' : 'text-slate-500 hover:text-slate-300'
          }`}
          title="Toggle Mask Overlay"
        >
          {showMasksOverlay ? <Eye size={13} /> : <EyeOff size={13} />}
        </button>
        <button
          onClick={onToggleManualPathsOverlay}
          className={`p-1.5 rounded transition-colors ${
            showManualPathsOverlay ? 'bg-amber-500/20 text-amber-300' : 'text-slate-500 hover:text-slate-300'
          }`}
          title="Toggle Manual Paths Overlay"
        >
          <Route size={13} />
        </button>
      </div>

      {/* 2. Atomic Loading Barrier Overlay */}
      {isLoadingTemplate && (
        <div className="absolute inset-0 bg-slate-950/60 backdrop-blur-[2px] flex items-center justify-center z-40 transition-opacity pointer-events-none">
          <div className="flex items-center gap-2 px-3 py-1.5 rounded-lg bg-slate-900/90 border border-slate-700 text-slate-300 text-xs shadow-xl">
            <RefreshCw size={14} className="animate-spin text-sky-400" />
            <span>Loading Template...</span>
          </div>
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
            className={`block max-w-full max-h-full object-contain ${
              segMode ? 'cursor-crosshair' : 'pointer-events-none'
            }`}
            alt="Captured view"
            onLoad={(e) =>
              setNatSize({
                w: e.currentTarget.naturalWidth,
                h: e.currentTarget.naturalHeight,
              })
            }
            onClick={onSegImageClick}
            onContextMenu={onSegContextMenu}
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
                renderPolygons={renderPolygons}
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

      {/* 4. Floating Bottom Toolbar: SAM Segmentation Mode */}
      {segMode && (
        <div className="absolute bottom-5 left-1/2 -translate-x-1/2 z-30 bg-slate-900/95 backdrop-blur-md border border-slate-700/80 rounded-xl p-1.5 shadow-2xl flex items-center gap-2.5 text-xs text-slate-200">
          <div className="flex items-center gap-1.5 px-2 border-r border-slate-700 text-[11px]">
            <span className="w-2 h-2 rounded-full bg-emerald-500 inline-block animate-pulse" />
            <span className="font-medium text-sky-300">Left: FG</span>
            <span className="w-2 h-2 rounded-full bg-rose-500 inline-block ml-1.5" />
            <span className="font-medium text-slate-400">Right: BG</span>
          </div>

          <div className="flex items-center gap-1">
            <button
              onClick={onUndoSegPoint}
              disabled={currentPoints.length === 0}
              className="px-2 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 disabled:opacity-40 flex items-center gap-1 text-[11px] font-medium transition-colors"
            >
              <Undo2 size={12} />
              <span>Undo</span>
            </button>
            <button
              onClick={onClearCurrentSegPoints}
              disabled={currentPoints.length === 0}
              className="px-2 py-1 rounded bg-slate-800 hover:bg-slate-700 text-rose-300 disabled:opacity-40 flex items-center gap-1 text-[11px] font-medium transition-colors"
            >
              <Trash2 size={12} />
              <span>Clear</span>
            </button>
            <button
              onClick={onCommitCurrentSegMask}
              disabled={currentPoints.length === 0}
              className="px-2.5 py-1 rounded bg-sky-600 hover:bg-sky-500 text-white text-[11px] font-medium flex items-center gap-1 shadow-md shadow-sky-900/40 disabled:opacity-40 transition-all"
            >
              <Check size={12} />
              <span>Commit</span>
            </button>
            <button
              onClick={onSaveAllSegMasks}
              className="px-2.5 py-1 rounded bg-emerald-600 hover:bg-emerald-500 text-white text-[11px] font-medium flex items-center gap-1 shadow-md shadow-emerald-900/40 transition-all ml-0.5"
            >
              <Save size={12} />
              <span>Save Masks</span>
            </button>
          </div>
        </div>
      )}

      {/* 5. Floating Bottom Toolbar: Manual TCP Path Designer */}
      {manualPathMode && (
        <div className="absolute bottom-5 left-1/2 -translate-x-1/2 z-30 bg-slate-900/95 backdrop-blur-md border border-amber-500/40 rounded-xl p-1.5 shadow-2xl flex items-center gap-2.5 text-xs text-slate-200">
          <div className="flex items-center gap-1.5 px-2 border-r border-slate-700 text-[11px]">
            <Route size={13} className="text-amber-400 animate-pulse" />
            <span className="font-medium text-amber-300">
              {selectedPathIdForEdit ? `Edit P${selectedPathIdForEdit}` : `Path P${manualPaths.length + 1}`}
            </span>
            <span className="text-[10px] text-slate-400 font-mono">
              ({currentManualPoints.length} pts)
            </span>
          </div>

          {/* Standoff Distance Adjustment */}
          <div className="flex items-center gap-1 px-1.5 border-r border-slate-700">
            <span className="text-[10px] text-slate-400 font-mono">Standoff:</span>
            <button
              onClick={() => setStandoffDistMm(Math.max(50, standoffDistMm - 10))}
              className="p-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300"
            >
              <Minus size={10} />
            </button>
            <span className="font-mono text-amber-400 text-[11px] px-0.5">{standoffDistMm}mm</span>
            <button
              onClick={() => setStandoffDistMm(Math.min(300, standoffDistMm + 10))}
              className="p-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300"
            >
              <Plus size={10} />
            </button>
          </div>

          <div className="flex items-center gap-1">
            <button
              onClick={onUndoManualPoint}
              disabled={currentManualPoints.length === 0}
              className="px-2 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 disabled:opacity-40 flex items-center gap-1 text-[11px] font-medium transition-colors"
            >
              <Undo2 size={12} />
              <span>Undo</span>
            </button>
            <button
              onClick={onClearCurrentManualPoints}
              disabled={currentManualPoints.length === 0}
              className="px-2 py-1 rounded bg-slate-800 hover:bg-slate-700 text-rose-300 disabled:opacity-40 flex items-center gap-1 text-[11px] font-medium transition-colors"
            >
              <Trash2 size={12} />
              <span>Clear</span>
            </button>
            <button
              onClick={onCommitManualPath}
              disabled={currentManualPoints.length === 0}
              className="px-2.5 py-1 rounded bg-amber-600 hover:bg-amber-500 text-white text-[11px] font-medium flex items-center gap-1 shadow-md shadow-amber-900/40 disabled:opacity-40 transition-all"
            >
              <Check size={12} />
              <span>{selectedPathIdForEdit ? 'Update' : 'Commit'}</span>
            </button>
            <button
              onClick={onSaveManualPaths}
              className="px-2.5 py-1 rounded bg-emerald-600 hover:bg-emerald-500 text-white text-[11px] font-medium flex items-center gap-1 shadow-md shadow-emerald-900/40 transition-all ml-0.5"
            >
              <Save size={12} />
              <span>Save</span>
            </button>
            {selectedPathIdForEdit && (
              <button
                onClick={onDeleteCurrentPath}
                className="px-2 py-1 rounded bg-rose-600/80 hover:bg-rose-600 text-white text-[11px] font-medium flex items-center gap-1 transition-all ml-0.5"
                title="Delete Selected Path"
              >
                <Trash2 size={12} />
              </button>
            )}
          </div>
        </div>
      )}
    </div>
  );
};
