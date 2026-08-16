import React from 'react';
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

      {/* 2. SEGMENTATION MODE: Active SAM Overlay */}
      {segMode && (
        <svg
          className="absolute inset-0 w-full h-full pointer-events-none"
          viewBox={`0 0 ${natSize.w} ${natSize.h}`}
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
            <circle
              key={idx}
              cx={p.x}
              cy={p.y}
              r={6}
              fill={p.label === 1 ? '#22c55e' : '#ef4444'}
              stroke="#ffffff"
              strokeWidth={2}
            />
          ))}
        </svg>
      )}
    </>
  );
};
