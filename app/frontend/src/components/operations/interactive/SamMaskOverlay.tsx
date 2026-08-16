import React, { type MouseEvent } from 'react';
import type { MaskData, Point } from './types';
import { MASK_COLORS } from './types';

interface SamMaskOverlayProps {
  segMode: boolean;
  showMasksOverlay: boolean;
  savedMasks: MaskData[];
  committedMasks: MaskData[];
  currentPolygons: number[][][];
  currentPoints: Point[];
  natSize: { w: number; h: number };
  onSegImageClick?: (e: MouseEvent<SVGSVGElement>) => void;
  onSegContextMenu?: (e: MouseEvent<SVGSVGElement>) => void;
  renderPolygons: (polygons: number[][][], fill: string, stroke?: string) => React.ReactNode;
}

export const SamMaskOverlay: React.FC<SamMaskOverlayProps> = ({
  segMode,
  showMasksOverlay,
  savedMasks,
  committedMasks,
  currentPolygons,
  currentPoints,
  natSize,
  onSegImageClick,
  onSegContextMenu,
  renderPolygons,
}) => {
  return (
    <>
      {/* 1. VIEW MODE: Render existing saved masks from scan.masks.yaml */}
      {!segMode && showMasksOverlay && savedMasks.length > 0 && (
        <svg
          className="absolute inset-0 w-full h-full pointer-events-none"
          viewBox={`0 0 ${natSize.w} ${natSize.h}`}
        >
          {savedMasks.map((m, idx) => {
            const colorScheme = MASK_COLORS[idx % MASK_COLORS.length];
            return (
              <g key={idx}>
                {renderPolygons(m.polygons, colorScheme.fill, colorScheme.stroke)}
              </g>
            );
          })}
        </svg>
      )}

      {/* 2. SEGMENTATION MODE: Active Interactive SAM Overlay */}
      {segMode && (
        <svg
          className="absolute inset-0 w-full h-full cursor-crosshair"
          viewBox={`0 0 ${natSize.w} ${natSize.h}`}
          onClick={onSegImageClick}
          onContextMenu={onSegContextMenu}
        >
          {/* Committed masks */}
          {committedMasks.map((m, idx) => {
            const colorScheme = MASK_COLORS[idx % MASK_COLORS.length];
            return (
              <g key={idx}>
                {renderPolygons(m.polygons, colorScheme.fill, colorScheme.stroke)}
              </g>
            );
          })}

          {/* Current active predicted mask */}
          {currentPolygons.length > 0 &&
            renderPolygons(
              currentPolygons,
              'rgba(59, 130, 246, 0.45)',
              '#60a5fa'
            )}

          {/* Current user prompt points (Green = FG, Red = BG) */}
          {currentPoints.map((p, idx) => (
            <g key={idx}>
              <circle
                cx={p.x}
                cy={p.y}
                r={8}
                fill="white"
                filter="drop-shadow(0px 0px 3px rgba(0,0,0,0.8))"
              />
              <circle
                cx={p.x}
                cy={p.y}
                r={6}
                fill={p.label === 1 ? '#10b981' : '#ef4444'}
              />
              <text
                x={p.x}
                y={p.y + 0.5}
                textAnchor="middle"
                dominantBaseline="central"
                fill="white"
                fontSize={9}
                fontWeight="bold"
              >
                {p.label === 1 ? '+' : '-'}
              </text>
            </g>
          ))}
        </svg>
      )}
    </>
  );
};
