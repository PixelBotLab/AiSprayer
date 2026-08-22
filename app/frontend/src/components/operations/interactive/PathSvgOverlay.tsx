import React, { type MouseEvent } from 'react';
import type {
  ManualPathItem,
  WaypointItem,
  VerificationReport,
  LiveNormalInfo,
  PathStateType,
  SimulationState,
  SessionData,
} from './types';
import { STATE_THEMES } from './types';

interface PathSvgOverlayProps {
  manualPathMode: boolean;
  showManualPathsOverlay: boolean;
  manualPaths: ManualPathItem[];
  currentManualPoints: WaypointItem[];
  selectedPathIdForEdit: number | null;
  highlightedPathId: number | null;
  hoveredWaypoint: WaypointItem | null;
  mousePixel: { u: number; v: number } | null;
  liveNormal: LiveNormalInfo | null;
  natSize: { w: number; h: number };
  verificationReport: VerificationReport | null;
  activeState?: PathStateType;
  simulationState?: SimulationState | null;
  sessionData?: SessionData | null;
  onSelectPathForEdit?: (pathId: number) => void;
  setHighlightedPathId: (id: number | null) => void;
  setHoveredWaypoint: (wp: WaypointItem | null) => void;
  onManualMouseMove?: (e: MouseEvent<SVGSVGElement>) => void;
  onManualImageClick?: (e: MouseEvent<SVGSVGElement>) => void;
  onDeleteSingleWaypoint?: (idx: number) => void;
}

function projectBasePointToPixel2d(
  posBaseMm: [number, number, number],
  sessionData: SessionData | null | undefined
): [number, number] | null {
  if (!sessionData?.T || sessionData.T.length < 16) return null;
  const T = sessionData.T;
  const dx = posBaseMm[0] - T[3];
  const dy = posBaseMm[1] - T[7];
  const dz = posBaseMm[2] - T[11];
  const Xc = T[0] * dx + T[4] * dy + T[8] * dz;
  const Yc = T[1] * dx + T[5] * dy + T[9] * dz;
  const Zc = T[2] * dx + T[6] * dy + T[10] * dz;
  if (Zc <= 10.0) return null;
  return [
    (sessionData.fx * Xc) / Zc + sessionData.cx,
    (sessionData.fy * Yc) / Zc + sessionData.cy,
  ];
}

function toolZProjection(pt: WaypointItem, sessionData: SessionData | null | undefined): [number, number, number, number] | null {
  const pose = pt.tcp_pose_base;
  const rx = (pose.rx * Math.PI) / 180.0;
  const ry = (pose.ry * Math.PI) / 180.0;
  const rz = (pose.rz * Math.PI) / 180.0;
  const cx = Math.cos(rx), sx = Math.sin(rx);
  const cy = Math.cos(ry), sy = Math.sin(ry);
  const cz = Math.cos(rz), sz = Math.sin(rz);
  const toolZ: [number, number, number] = [
    cz * sy * cx + sz * sx,
    sz * sy * cx - cz * sx,
    cy * cx,
  ];
  const start = projectBasePointToPixel2d([pose.x, pose.y, pose.z], sessionData);
  const axisLenMm = 80.0;
  const end = projectBasePointToPixel2d([
    pose.x + toolZ[0] * axisLenMm,
    pose.y + toolZ[1] * axisLenMm,
    pose.z + toolZ[2] * axisLenMm,
  ], sessionData);
  if (!start || !end) return null;
  return [start[0], start[1], end[0], end[1]];
}

export const PathSvgOverlay: React.FC<PathSvgOverlayProps> = ({
  manualPathMode,
  showManualPathsOverlay,
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
  onSelectPathForEdit,
  setHighlightedPathId,
  setHoveredWaypoint,
  onManualMouseMove,
  onManualImageClick,
  onDeleteSingleWaypoint,
}) => {
  const theme = STATE_THEMES[activeState] || STATE_THEMES.raw;

  return (
    <>
      {/* 1. VIEW/OVERLAY MODE: Render Manual TCP Paths on top of scan.jpg */}
      {!manualPathMode && showManualPathsOverlay && manualPaths.length > 0 && (
        <svg
          className="absolute inset-0 w-full h-full pointer-events-none select-none"
          viewBox={`0 0 ${natSize.w} ${natSize.h}`}
          onMouseLeave={() => setHoveredWaypoint(null)}
        >
          <defs>
            <marker id="view-traj-arrow" markerWidth="5.5" markerHeight="5.5" refX="4.2" refY="2.75" orient="auto">
              <path d="M0,0.6 L0,4.9 L4.9,2.75 z" fill="#38bdf8" />
            </marker>
            <marker id="view-normal-arrow" markerWidth="4" markerHeight="4" refX="3.2" refY="2" orient="auto">
              <path d="M0,0.6 L0,3.4 L3.4,2 z" fill="#ef4444" />
            </marker>
            <marker id="view-tool-z-arrow" markerWidth="5" markerHeight="5" refX="4" refY="2.5" orient="auto">
              <path d="M0,0.5 L0,4.5 L4.5,2.5 z" fill={theme.hex} />
            </marker>
          </defs>

          {manualPaths.map((path, pIdx) => {
            const pts = path.points;
            const pId = path.path_id ?? (pIdx + 1);
            const isHighlighted = highlightedPathId === pId;
            const isCurrentSimPath = simulationState?.isPlaying && (simulationState.currentPathIndex === pIdx);
            const pathStroke = isHighlighted ? '#ffffff' : (isCurrentSimPath ? theme.hex : (STATE_THEMES[activeState]?.hex || '#94a3b8'));

            // Check if there are verification issues reported on this path
            const pathRep = verificationReport?.path_reports?.find((r: any) => r.path_id === pId);
            const pathIssues = pathRep?.issues || [];

            return (
              <g key={pIdx}>
                {/* Glowing Background Line for Highlighted / Focused Path */}
                {isHighlighted &&
                  pts.map((p, i) => {
                    if (i === 0) return null;
                    const prev = pts[i - 1];
                    return (
                      <line
                        key={`hseg-${i}`}
                        x1={prev.pixel[0]}
                        y1={prev.pixel[1]}
                        x2={p.pixel[0]}
                        y2={p.pixel[1]}
                        stroke={theme.hex}
                        strokeWidth={9}
                        strokeOpacity={0.4}
                        strokeLinecap="round"
                        style={{ pointerEvents: 'none' }}
                      />
                    );
                  })}

                {/* Connecting Line with Arrow on EVERY consecutive segment */}
                {pts.map((p, i) => {
                  if (i === 0) return null;
                  const prev = pts[i - 1];
                  const segIdx = i - 1;
                  const hasSegIssue = pathIssues.some((iss: any) => iss.segment_index === segIdx);

                  // If this path is currently simulating, highlight traversed vs pending segments
                  const segProgress = pts.length > 1 ? (i / (pts.length - 1)) : 1;
                  const isTraversed = isCurrentSimPath && simulationState && (simulationState.progress >= segProgress);

                  return (
                    <g key={`vseg-grp-${i}`}>
                      <line
                        key={`vseg-${i}`}
                        x1={prev.pixel[0]}
                        y1={prev.pixel[1]}
                        x2={p.pixel[0]}
                        y2={p.pixel[1]}
                        stroke={hasSegIssue ? '#f43f5e' : (isTraversed ? '#fbbf24' : pathStroke)}
                        strokeWidth={isTraversed ? 4.0 : (isHighlighted ? 3.5 : (hasSegIssue ? 3.0 : 2.5))}
                        markerEnd="url(#view-traj-arrow)"
                        strokeDasharray={hasSegIssue ? '6 3' : undefined}
                        style={{ pointerEvents: 'none' }}
                      />
                    </g>
                  );
                })}

                {/* Verification Issue Warning Beacons */}
                {Array.from(
                  new Set(
                    pathIssues.map((iss: any) =>
                      iss.step_index !== undefined
                        ? Math.min(Math.floor((iss.step_index / (pathRep?.total_interpolated || 1)) * pts.length), pts.length - 1)
                        : (iss.waypoint_index !== undefined ? Math.min(iss.waypoint_index, pts.length - 1) : 0)
                    )
                  )
                ).map((targetWpIdx: any, bIdx: number) => {
                  const pt = pts[targetWpIdx];
                  if (!pt) return null;
                  const [u, v] = pt.pixel;
                  const issuesAtPt = pathIssues.filter((iss: any) => {
                    const idx = iss.step_index !== undefined
                      ? Math.min(Math.floor((iss.step_index / (pathRep?.total_interpolated || 1)) * pts.length), pts.length - 1)
                      : (iss.waypoint_index !== undefined ? Math.min(iss.waypoint_index, pts.length - 1) : 0);
                    return idx === targetWpIdx;
                  });

                  return (
                    <g key={`beacon-grp-${bIdx}`} className="pointer-events-auto cursor-help">
                      <circle cx={u} cy={v} r={8} fill="rgba(244, 63, 94, 0.25)" stroke="#f43f5e" strokeWidth={2} style={{ pointerEvents: 'none' }}>
                        <animate attributeName="r" values="8;24" dur="1.6s" repeatCount="indefinite" />
                        <animate attributeName="opacity" values="0.9;0" dur="1.6s" repeatCount="indefinite" />
                      </circle>
                      <circle cx={u} cy={v} r={8} fill="none" stroke="#f43f5e" strokeWidth={1.5} style={{ pointerEvents: 'none' }}>
                        <animate attributeName="r" values="8;24" begin="0.8s" dur="1.6s" repeatCount="indefinite" />
                        <animate attributeName="opacity" values="0.8;0" begin="0.8s" dur="1.6s" repeatCount="indefinite" />
                      </circle>
                      <circle cx={u} cy={v} r={5.5} fill="#f43f5e" stroke="#ffffff" strokeWidth={1.8} style={{ pointerEvents: 'none' }} />

                      {(() => {
                        const critIssue =
                          issuesAtPt.find((i: any) => i.severity === 'ERROR') ||
                          issuesAtPt.find((i: any) => i.type === 'KINEMATIC_DISCONTINUITY') ||
                          issuesAtPt.find((i: any) => i.type === 'UNREACHABLE' || i.type === 'UNREACHABLE_STEP') ||
                          issuesAtPt.find((i: any) => i.type === 'ELBOW_SINGULARITY' || i.type === 'WRIST_SINGULARITY') ||
                          issuesAtPt[0];

                        const isUnreach =
                          critIssue?.type === 'UNREACHABLE' ||
                          critIssue?.type === 'UNREACHABLE_STEP' ||
                          critIssue?.type === 'KINEMATIC_DISCONTINUITY';
                        const isSing = critIssue?.type === 'ELBOW_SINGULARITY' || critIssue?.type === 'WRIST_SINGULARITY';
                        const label = isUnreach ? '❌ Unreachable' : (isSing ? '⚠️ Singularity' : '⚠️ Overspeed');
                        const badgeWidth = isUnreach ? 96 : (isSing ? 88 : 82);

                        return (
                          <g transform={`translate(${u + 10}, ${v - 12})`} style={{ pointerEvents: 'none' }}>
                            <rect x={0} y={0} width={badgeWidth} height={20} rx={4} fill="rgba(136, 19, 55, 0.95)" stroke="#f43f5e" strokeWidth={1} filter="drop-shadow(0px 2px 4px rgba(0,0,0,0.6))" />
                            <text x={badgeWidth / 2} y={13} textAnchor="middle" fill="#ffe4e6" fontSize={9.5} fontWeight="bold" fontFamily="monospace">
                              {label}
                            </text>
                          </g>
                        );
                      })()}
                    </g>
                  );
                })}

                {/* Start Point Number Badge (P1, P2, P3...) at Point 0 */}
                {pts.length > 0 && (() => {
                  const p0 = pts[0];
                  const [u0, v0] = p0.pixel;
                  const badgeColor = STATE_THEMES[activeState]?.hex || '#64748b';
                  return (
                    <g
                      key={`start-badge-${pId}`}
                      className="pointer-events-auto cursor-pointer"
                      onMouseEnter={() => {
                        setHoveredWaypoint({ ...p0, path_id: pId });
                        setHighlightedPathId(pId);
                      }}
                      onMouseLeave={() => {
                        setHoveredWaypoint(null);
                        setHighlightedPathId(null);
                      }}
                    >
                      <line x1={u0} y1={v0 - 6} x2={u0} y2={v0} stroke={badgeColor} strokeWidth={2} />
                      <circle cx={u0} cy={v0 - 15} r={10.5} fill={badgeColor} stroke="#ffffff" strokeWidth={1.5} filter="drop-shadow(0px 2px 4px rgba(0,0,0,0.75))" />
                      <text x={u0} y={v0 - 14.5} textAnchor="middle" dominantBaseline="central" fill="#ffffff" fontSize={9.5} fontWeight="bold" style={{ pointerEvents: 'none' }}>
                        {`P${pId}`}
                      </text>
                    </g>
                  );
                })()}

                {/* Intermediate Waypoints (1+) */}
                {pts.map((pt, idx) => {
                  const [u, v] = pt.pixel;
                  const dx = pt.normal_2d_proj?.[0] ?? 0;
                  const dy = pt.normal_2d_proj?.[1] ?? 0;
                  const tcpU = u + dx;
                  const tcpV = v + dy;
                  const arrowLen = Math.hypot(dx, dy);
                  const toolZ = toolZProjection(pt, sessionData);
                  const tcpDrawU = toolZ ? toolZ[0] : tcpU;
                  const tcpDrawV = toolZ ? toolZ[1] : tcpV;

                  return (
                    <g
                      key={idx}
                      className="pointer-events-auto cursor-pointer"
                      onMouseEnter={() => {
                        setHoveredWaypoint({ ...pt, path_id: pId });
                        setHighlightedPathId(pId);
                      }}
                      onMouseLeave={() => {
                        setHoveredWaypoint(null);
                        setHighlightedPathId(null);
                      }}
                    >
                      {/* Surface Normal Projection: raw depth normal, kept as red reference. */}
                      {arrowLen > 2 && (
                        <line
                          x1={u}
                          y1={v}
                          x2={tcpU}
                          y2={tcpV}
                          stroke="#ef4444"
                          strokeWidth={1.2}
                          strokeDasharray="3 2"
                          markerEnd="url(#view-normal-arrow)"
                        />
                      )}

                      {/* Tool Z Axis Projection: changes with current-state TCP orientation. */}
                      {toolZ && (
                        <line
                          x1={toolZ[0]}
                          y1={toolZ[1]}
                          x2={toolZ[2]}
                          y2={toolZ[3]}
                          stroke={theme.hex}
                          strokeWidth={2.2}
                          markerEnd="url(#view-tool-z-arrow)"
                        />
                      )}

                      {/* Surface Point (Red dot on surface) */}
                      <circle
                        cx={u}
                        cy={v}
                        r={4.5}
                        fill="#ef4444"
                        stroke="#ffffff"
                        strokeWidth={1.2}
                      />

                      {/* TCP Point with Sequential Step Number Badge */}
                      <g transform={`translate(${tcpDrawU}, ${tcpDrawV})`}>
                        <circle
                          cx={0}
                          cy={0}
                          r={6.5}
                          fill={STATE_THEMES[activeState]?.hex || '#64748b'}
                          stroke="#ffffff"
                          strokeWidth={1.5}
                          filter="drop-shadow(0px 1px 3px rgba(0,0,0,0.6))"
                        />
                        <text
                          x={0}
                          y={0.5}
                          textAnchor="middle"
                          dominantBaseline="central"
                          fill="#ffffff"
                          fontSize={8}
                          fontWeight="bold"
                          fontFamily="monospace"
                        >
                          {pt.index || idx + 1}
                        </text>
                      </g>
                    </g>
                  );
                })}
              </g>
            );
          })}

          {/* 3D/2D REAL-TIME SYNCHRONIZED SIMULATION BEACON (Feature 7) */}
          {simulationState && simulationState.currentPixel && (
            <g
              transform={`translate(${simulationState.currentPixel[0]}, ${simulationState.currentPixel[1]})`}
              style={{ pointerEvents: 'none' }}
            >
              {/* Outer Pulsing Aura Ring */}
              <circle cx={0} cy={0} r={12} fill="none" stroke={theme.hex} strokeWidth={2} opacity={0.8}>
                <animate attributeName="r" values="8;32" dur="1.2s" repeatCount="indefinite" />
                <animate attributeName="opacity" values="0.9;0" dur="1.2s" repeatCount="indefinite" />
              </circle>
              {/* Secondary Pulse */}
              <circle cx={0} cy={0} r={12} fill="none" stroke="#fbbf24" strokeWidth={1.5} opacity={0.6}>
                <animate attributeName="r" values="8;24" begin="0.4s" dur="1.2s" repeatCount="indefinite" />
                <animate attributeName="opacity" values="0.8;0" begin="0.4s" dur="1.2s" repeatCount="indefinite" />
              </circle>
              {/* Solid Bright Core Spray Nozzle Dot */}
              <circle
                cx={0}
                cy={0}
                r={6.5}
                fill={theme.hex}
                stroke="#ffffff"
                strokeWidth={2}
                filter="drop-shadow(0px 0px 8px rgba(255,255,255,0.9))"
              />
              {/* Spray Head Crosshair */}
              <line x1={-9} y1={0} x2={9} y2={0} stroke="#ffffff" strokeWidth={1.2} />
              <line x1={0} y1={-9} x2={0} y2={9} stroke="#ffffff" strokeWidth={1.2} />
            </g>
          )}

        </svg>
      )}

      {/* 2. MANUAL TCP EDIT MODE */}
      {manualPathMode && (
        <svg
          className="absolute inset-0 w-full h-full cursor-crosshair"
          viewBox={`0 0 ${natSize.w} ${natSize.h}`}
          onMouseMove={onManualMouseMove}
          onClick={onManualImageClick}
          onMouseLeave={() => setHoveredWaypoint(null)}
        >
          <defs>
            <marker id="normal-arrow" markerWidth="4" markerHeight="4" refX="3.2" refY="2" orient="auto">
              <path d="M0,0.6 L0,3.4 L3.4,2 z" fill="#ef4444" />
            </marker>
            <marker id="edit-traj-arrow" markerWidth="5.5" markerHeight="5.5" refX="4.2" refY="2.75" orient="auto">
              <path d="M0,0.6 L0,4.9 L4.9,2.75 z" fill="#f59e0b" />
            </marker>
          </defs>

          {/* Committed Paths (Clickable to Edit) */}
          {manualPaths.map((path, pIdx) => {
            if (selectedPathIdForEdit === path.path_id) return null;
            const pts = path.points;
            const pId = path.path_id ?? (pIdx + 1);
            return (
              <g
                key={`committed-${pIdx}`}
                className="cursor-pointer opacity-75 hover:opacity-100 transition-opacity"
                onClick={(e) => {
                  e.stopPropagation();
                  if (onSelectPathForEdit) onSelectPathForEdit(path.path_id);
                }}
              >
                {pts.map((p, i) => {
                  if (i === 0) return null;
                  const prev = pts[i - 1];
                  return (
                    <line key={`cseg-${i}`} x1={prev.pixel[0]} y1={prev.pixel[1]} x2={p.pixel[0]} y2={p.pixel[1]} stroke="#0284c7" strokeWidth={2.5} markerEnd="url(#edit-traj-arrow)" />
                  );
                })}
                {pts.length > 0 && (
                  <g key={`c-start-${pId}`}>
                    <line x1={pts[0].pixel[0]} y1={pts[0].pixel[1] - 5} x2={pts[0].pixel[0]} y2={pts[0].pixel[1]} stroke="#0284c7" strokeWidth={1.5} />
                    <circle cx={pts[0].pixel[0]} cy={pts[0].pixel[1] - 14} r={9.5} fill="#0284c7" stroke="white" strokeWidth={1.5} />
                    <text x={pts[0].pixel[0]} y={pts[0].pixel[1] - 13.5} textAnchor="middle" dominantBaseline="central" fill="white" fontSize={9} fontWeight="bold">
                      {`P${pId}`}
                    </text>
                  </g>
                )}
                {pts.slice(1).map((pt, idx) => (
                  <g key={idx}>
                    <circle cx={pt.pixel[0]} cy={pt.pixel[1]} r={5} fill="#0284c7" stroke="white" strokeWidth={1.2} />
                    <text x={pt.pixel[0]} y={pt.pixel[1] + 0.5} textAnchor="middle" dominantBaseline="central" fill="white" fontSize={7} fontWeight="bold">
                      {idx + 2}
                    </text>
                  </g>
                ))}
              </g>
            );
          })}

          {/* Active Current Path Line Segments */}
          {currentManualPoints.map((p, i) => {
            if (i === 0) return null;
            const prev = currentManualPoints[i - 1];
            return (
              <line key={`curr-seg-${i}`} x1={prev.pixel[0]} y1={prev.pixel[1]} x2={p.pixel[0]} y2={p.pixel[1]} stroke="#f59e0b" strokeWidth={2.8} markerEnd="url(#edit-traj-arrow)" />
            );
          })}

          {/* Live Mouse Move Dashed Line */}
          {currentManualPoints.length > 0 && mousePixel && (
            <g pointerEvents="none">
              <line
                x1={currentManualPoints[currentManualPoints.length - 1].pixel[0]}
                y1={currentManualPoints[currentManualPoints.length - 1].pixel[1]}
                x2={mousePixel.u}
                y2={mousePixel.v}
                stroke="#f59e0b"
                strokeWidth={2.2}
                strokeDasharray="6 3.5"
                markerEnd="url(#edit-traj-arrow)"
                opacity={0.95}
              />
            </g>
          )}

          {/* Real-time Live Normal Vector Arrow at Cursor */}
          {mousePixel && (
            <g pointerEvents="none">
              {(() => {
                const ldx = liveNormal?.dx ?? 0;
                const ldy = liveNormal?.dy ?? 0;
                const lLen = Math.hypot(ldx, ldy);
                return lLen >= 3.0 ? (
                  <line x1={mousePixel.u} y1={mousePixel.v} x2={mousePixel.u + ldx} y2={mousePixel.v + ldy} stroke="#ef4444" strokeWidth={2} strokeLinecap="round" markerEnd="url(#normal-arrow)" />
                ) : (
                  <circle cx={mousePixel.u} cy={mousePixel.v} r={3.5} fill="#ef4444" opacity={0.8} />
                );
              })()}
              <circle cx={mousePixel.u} cy={mousePixel.v} r={4.5} fill="#f59e0b" />
              <circle cx={mousePixel.u} cy={mousePixel.v} r={8.5} fill="none" stroke="#f59e0b" strokeWidth={1.5} opacity={0.7} />
            </g>
          )}

          {/* Active Current Waypoints */}
          {currentManualPoints.map((pt, idx) => {
            const [u, v] = pt.pixel;
            const dx = pt.normal_2d_proj?.[0] ?? 0;
            const dy = pt.normal_2d_proj?.[1] ?? 0;
            const tcpU = u + dx;
            const tcpV = v + dy;
            const isHovered = hoveredWaypoint?.index === pt.index;
            const arrowLen = Math.hypot(dx, dy);

            return (
              <g
                key={`act-wp-${idx}`}
                className="group cursor-pointer"
                onMouseEnter={() => setHoveredWaypoint(pt)}
                onClick={(e) => {
                  e.stopPropagation();
                  if (onDeleteSingleWaypoint) onDeleteSingleWaypoint(idx);
                }}
              >
                {arrowLen >= 3.0 ? (
                  <line x1={u} y1={v} x2={tcpU} y2={tcpV} stroke="#ef4444" strokeWidth={isHovered ? 2.8 : 2} strokeLinecap="round" markerEnd="url(#normal-arrow)" style={{ pointerEvents: 'none' }} />
                ) : (
                  <circle cx={u} cy={v} r={3.5} fill="#ef4444" opacity={0.8} style={{ pointerEvents: 'none' }} />
                )}

                <circle cx={u} cy={v} r={12} fill="transparent" />
                <circle cx={u} cy={v} r={isHovered ? 8 : 6.5} fill="#f59e0b" stroke="#ffffff" strokeWidth={2} filter="drop-shadow(0px 0px 4px rgba(0,0,0,0.8))" />
                <text x={u} y={v + 0.5} textAnchor="middle" dominantBaseline="central" fill="#ffffff" fontSize={isHovered ? 9 : 8} fontWeight="bold" style={{ pointerEvents: 'none' }}>
                  {pt.index}
                </text>
              </g>
            );
          })}
        </svg>
      )}
    </>
  );
};
