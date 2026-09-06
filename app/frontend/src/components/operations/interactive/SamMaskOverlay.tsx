import React, { type MouseEvent } from 'react';
import type { MaskData, Point } from './types';
import { MASK_COLORS } from './types';

export function polygonsToSvgPath(polygons: number[][][]): string {
  if (!polygons || polygons.length === 0) return '';
  return polygons
    .map((poly) =>
      poly.map((pt, i) => `${i === 0 ? 'M' : 'L'} ${pt[0]} ${pt[1]}`).join(' ') + ' Z'
    )
    .join(' ');
}

interface SamMaskOverlayProps {
  segMode: boolean;
  showMasksOverlay: boolean;
  savedMasks: MaskData[];
  committedMasks: MaskData[];
  currentPolygons: number[][][];
  currentPoints: Point[];
  natSize: { w: number; h: number };
  isReconstructing?: boolean;
  isAutoGenerating?: boolean;
  isVerifying?: boolean;
  isOptimizing?: boolean;
  onSegImageClick?: (e: MouseEvent<SVGSVGElement>) => void;
  onSegContextMenu?: (e: MouseEvent<SVGSVGElement>) => void;
  renderPolygons?: (polygons: number[][][], fill: string, stroke?: string) => React.ReactNode;
}

export const SamMaskOverlay: React.FC<SamMaskOverlayProps> = ({
  segMode,
  showMasksOverlay,
  savedMasks,
  committedMasks,
  currentPolygons,
  currentPoints,
  natSize,
  isReconstructing = false,
  isAutoGenerating = false,
  isVerifying = false,
  isOptimizing = false,
  onSegImageClick,
  onSegContextMenu,
  renderPolygons,
}) => {
  const defaultRenderPolygons = (polygons: number[][][], fill: string, stroke?: string) => (
    <path
      d={polygonsToSvgPath(polygons)}
      fill={fill}
      stroke={stroke || 'none'}
      strokeWidth={1.5}
    />
  );

  const drawPoly = renderPolygons || defaultRenderPolygons;
  const activeMasks = savedMasks.length > 0 ? savedMasks : committedMasks;
  const isAnyDynamicAction = isReconstructing || isAutoGenerating || isVerifying || isOptimizing;

  return (
    <>
      {/* 1. VIEW MODE: Render static saved masks from scan.masks.yaml */}
      {!segMode && showMasksOverlay && savedMasks.length > 0 && !isAnyDynamicAction && (
        <svg
          className="absolute inset-0 w-full h-full pointer-events-none"
          viewBox={`0 0 ${natSize.w} ${natSize.h}`}
        >
          {savedMasks.map((m, idx) => {
            const colorScheme = MASK_COLORS[idx % MASK_COLORS.length];
            return (
              <g key={idx}>
                {drawPoly(m.polygons, colorScheme.fill, colorScheme.stroke)}
              </g>
            );
          })}
        </svg>
      )}

      {/* 2. DYNAMIC ACTION OVERLAYS: High-tech visual effects on workpiece masks */}
      {!segMode && isAnyDynamicAction && activeMasks.length > 0 && (
        <svg
          className="absolute inset-0 w-full h-full pointer-events-none select-none"
          viewBox={`0 0 ${natSize.w} ${natSize.h}`}
        >
          <defs>
            {/* 3D Reconstruction Filters & Patterns */}
            <filter id="mask-reconstruct-glow" x="-20%" y="-20%" width="140%" height="140%">
              <feGaussianBlur stdDeviation="4" result="coloredBlur" />
              <feMerge>
                <feMergeNode in="coloredBlur" />
                <feMergeNode in="SourceGraphic" />
              </feMerge>
            </filter>
            <pattern id="mask-cyber-grid" width="28" height="28" patternUnits="userSpaceOnUse">
              <path d="M 28 0 L 0 0 0 28" fill="none" stroke="rgba(52, 211, 153, 0.28)" strokeWidth="0.8" />
              <circle cx="0" cy="0" r="1.5" fill="#34d399" opacity="0.75" />
            </pattern>
            <linearGradient id="mask-reconstruct-laser" x1="0" y1="0" x2="0" y2="100%">
              <stop offset="0%" stopColor="#34d399" stopOpacity="0" />
              <stop offset="45%" stopColor="#34d399" stopOpacity="0.45" />
              <stop offset="50%" stopColor="#6ee7b7" stopOpacity="0.85" />
              <stop offset="55%" stopColor="#34d399" stopOpacity="0.45" />
              <stop offset="100%" stopColor="#10b981" stopOpacity="0" />
            </linearGradient>

            {/* Auto Path Planning Filters & Patterns */}
            <filter id="mask-autopath-glow" x="-20%" y="-20%" width="140%" height="140%">
              <feGaussianBlur stdDeviation="3.5" result="coloredBlur" />
              <feMerge>
                <feMergeNode in="coloredBlur" />
                <feMergeNode in="SourceGraphic" />
              </feMerge>
            </filter>
            <pattern id="mask-raster-lines" width="100" height="18" patternUnits="userSpaceOnUse">
              <line x1="0" y1="9" x2="100" y2="9" stroke="rgba(245, 158, 11, 0.35)" strokeWidth="1.2" strokeDasharray="8 5" />
            </pattern>
            <linearGradient id="mask-autopath-sweep" x1="0" y1="0" x2="0" y2="100%">
              <stop offset="0%" stopColor="#fbbf24" stopOpacity="0" />
              <stop offset="48%" stopColor="#f59e0b" stopOpacity="0.5" />
              <stop offset="50%" stopColor="#fef08a" stopOpacity="0.9" />
              <stop offset="52%" stopColor="#f59e0b" stopOpacity="0.5" />
              <stop offset="100%" stopColor="#d97706" stopOpacity="0" />
            </linearGradient>
          </defs>

          {activeMasks.map((m, idx) => {
            const pathD = polygonsToSvgPath(m.polygons);
            const clipId = `mask-clip-dyn-${idx}`;

            return (
              <g key={`dyn-mask-${idx}`}>
                <clipPath id={clipId}>
                  <path d={pathD} />
                </clipPath>

                {/* 2.1 3D Reconstruction: Holographic Grid + Sweeping Laser Beam */}
                {isReconstructing && (
                  <>
                    <g clipPath={`url(#${clipId})`}>
                      {/* Background cyber grid inside workpiece mask */}
                      <rect x="0" y="0" width={natSize.w} height={natSize.h} fill="url(#mask-cyber-grid)" />
                      {/* Moving laser scan beam inside workpiece mask */}
                      <rect x="0" y="-80" width={natSize.w} height="80" fill="url(#mask-reconstruct-laser)">
                        <animate
                          attributeName="y"
                          values={`-90;${natSize.h + 90}`}
                          dur="1.8s"
                          repeatCount="indefinite"
                        />
                      </rect>
                    </g>
                    {/* Glowing perimeter contour with dual animated dashed laser beams */}
                    <path
                      d={pathD}
                      fill="rgba(16, 185, 129, 0.12)"
                      stroke="#10b981"
                      strokeWidth={3}
                      strokeDasharray="16 8"
                      strokeLinecap="round"
                      filter="url(#mask-reconstruct-glow)"
                    >
                      <animate attributeName="stroke-dashoffset" values="0;96" dur="1.2s" repeatCount="indefinite" />
                    </path>
                    <path
                      d={pathD}
                      fill="none"
                      stroke="#6ee7b7"
                      strokeWidth={1.5}
                      strokeDasharray="8 8"
                    >
                      <animate attributeName="stroke-dashoffset" values="64;0" dur="1.2s" repeatCount="indefinite" />
                    </path>
                  </>
                )}

                {/* 2.2 Auto Path Planning: Raster Lines + Sweeping Toolpath Laser */}
                {isAutoGenerating && (
                  <>
                    <g clipPath={`url(#${clipId})`}>
                      {/* Simulated raster toolpath guide lines inside workpiece mask */}
                      <rect x="0" y="0" width={natSize.w} height={natSize.h} fill="url(#mask-raster-lines)" />
                      {/* Sweeping laser band across the workpiece */}
                      <rect x="0" y="-80" width={natSize.w} height="80" fill="url(#mask-autopath-sweep)">
                        <animate
                          attributeName="y"
                          values={`-90;${natSize.h + 90}`}
                          dur="1.5s"
                          repeatCount="indefinite"
                        />
                      </rect>
                    </g>
                    {/* Animated amber glowing border */}
                    <path
                      d={pathD}
                      fill="rgba(245, 158, 11, 0.12)"
                      stroke="#f59e0b"
                      strokeWidth={3}
                      strokeDasharray="16 8"
                      strokeLinecap="round"
                      filter="url(#mask-autopath-glow)"
                    >
                      <animate attributeName="stroke-dashoffset" values="0;96" dur="1s" repeatCount="indefinite" />
                    </path>
                  </>
                )}

                {/* 2.3 Kinematics Verification: Sky-Blue Contextual Workpiece Contour */}
                {isVerifying && (
                  <path
                    d={pathD}
                    fill="rgba(56, 189, 248, 0.08)"
                    stroke="#38bdf8"
                    strokeWidth={2}
                    strokeDasharray="12 6"
                    strokeLinecap="round"
                  >
                    <animate attributeName="stroke-dashoffset" values="0;72" dur="1.4s" repeatCount="indefinite" />
                  </path>
                )}

                {/* 2.4 POI Optimization: Purple Contextual Workpiece Aura */}
                {isOptimizing && (
                  <path
                    d={pathD}
                    fill="rgba(192, 132, 252, 0.08)"
                    stroke="#c084fc"
                    strokeWidth={2}
                    strokeDasharray="12 6"
                    strokeLinecap="round"
                  >
                    <animate attributeName="stroke-dashoffset" values="0;72" dur="1.4s" repeatCount="indefinite" />
                  </path>
                )}
              </g>
            );
          })}
        </svg>
      )}

      {/* 3. SEGMENTATION MODE: Active Interactive SAM Overlay */}
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
                {drawPoly(m.polygons, colorScheme.fill, colorScheme.stroke)}
              </g>
            );
          })}

          {/* Current active predicted mask */}
          {currentPolygons.length > 0 &&
            drawPoly(
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
