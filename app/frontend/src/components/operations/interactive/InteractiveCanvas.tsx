import React, { useRef, useEffect, type MouseEvent, type WheelEvent } from 'react';
import { RefreshCw } from 'lucide-react';
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
  onSegImageClick?: (e: MouseEvent<HTMLImageElement>) => void;
  onSegContextMenu?: (e: MouseEvent<HTMLImageElement>) => void;
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
    if (e.button === 1 || isSpacePressed) {
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
      className={`flex-1 flex flex-col border-r border-slate-800 relative bg-black items-center justify-center overflow-hidden select-none ${
        isPanning
          ? 'cursor-grabbing'
          : segMode
          ? isSpacePressed
            ? 'cursor-grab'
            : 'cursor-crosshair'
          : 'cursor-grab'
      }`}
      onWheel={handleWheel}
      onMouseDown={handleMouseDown}
    >
      {/* Atomic Loading Barrier Overlay */}
      {isLoadingTemplate && (
        <div className="absolute inset-0 bg-slate-950/60 backdrop-blur-[2px] flex items-center justify-center z-40 transition-opacity pointer-events-none">
          <div className="flex items-center gap-2 px-3 py-1.5 rounded-lg bg-slate-900/90 border border-slate-700 text-slate-300 text-xs shadow-xl">
            <RefreshCw size={14} className="animate-spin text-sky-400" />
            <span>Loading Template...</span>
          </div>
        </div>
      )}

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
    </div>
  );
};
