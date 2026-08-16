import React, { type MouseEvent } from 'react';
import type { ManualPathItem, WaypointItem, VerificationReport, LiveNormalInfo } from './types';

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
  onSelectPathForEdit?: (pathId: number) => void;
  setHighlightedPathId: (id: number | null) => void;
  setHoveredWaypoint: (wp: WaypointItem | null) => void;
  onManualMouseMove?: (e: MouseEvent<SVGSVGElement>) => void;
  onManualImageClick?: (e: MouseEvent<SVGSVGElement>) => void;
  onDeleteSingleWaypoint?: (idx: number) => void;
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
  onSelectPathForEdit,
  setHighlightedPathId,
  setHoveredWaypoint,
  onManualMouseMove,
  onManualImageClick,
  onDeleteSingleWaypoint,
}) => {
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
          </defs>

          {manualPaths.map((path, pIdx) => {
            const pts = path.points;
            const pId = path.path_id ?? (pIdx + 1);
            const isHighlighted = highlightedPathId === pId;
            const pathStroke = isHighlighted ? '#38bdf8' : '#0284c7';

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
                        stroke="#38bdf8"
                        strokeWidth={9}
                        strokeOpacity={0.35}
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

                  return (
                    <g key={`vseg-grp-${i}`}>
                      <line
                        key={`vseg-${i}`}
                        x1={prev.pixel[0]}
                        y1={prev.pixel[1]}
                        x2={p.pixel[0]}
                        y2={p.pixel[1]}
                        stroke={hasSegIssue ? '#f43f5e' : pathStroke}
                        strokeWidth={isHighlighted ? 3.5 : (hasSegIssue ? 3.0 : 2.5)}
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
                      <line x1={u0} y1={v0 - 6} x2={u0} y2={v0} stroke="#0284c7" strokeWidth={2} />
                      <circle cx={u0} cy={v0 - 15} r={10.5} fill="#0284c7" stroke="#ffffff" strokeWidth={1.5} filter="drop-shadow(0px 2px 4px rgba(0,0,0,0.75))" />
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
                      {arrowLen >= 3.0 ? (
                        <line x1={u} y1={v} x2={tcpU} y2={tcpV} stroke="#ef4444" strokeWidth={2.2} strokeLinecap="round" markerEnd="url(#view-normal-arrow)" style={{ pointerEvents: 'none' }} />
                      ) : (
                        <circle cx={u} cy={v} r={3.5} fill="#ef4444" opacity={0.85} style={{ pointerEvents: 'none' }} />
                      )}

                      {idx > 0 && (
                        <>
                          <circle cx={u} cy={v} r={12} fill="transparent" />
                          <circle cx={u} cy={v} r={isHighlighted ? 7.5 : 6} fill="white" filter="drop-shadow(0px 0px 3px rgba(0,0,0,0.8))" style={{ pointerEvents: 'none' }} />
                          <circle cx={u} cy={v} r={isHighlighted ? 5.5 : 4.5} fill="#0284c7" style={{ pointerEvents: 'none' }} />
                          <text x={u} y={v + 0.5} textAnchor="middle" dominantBaseline="central" fill="white" fontSize={isHighlighted ? 8 : 7} fontWeight="bold" style={{ pointerEvents: 'none' }}>
                            {idx + 1}
                          </text>
                        </>
                      )}
                    </g>
                  );
                })}
              </g>
            );
          })}

          {/* Topmost Tooltip Layer in VIEW mode */}
          {hoveredWaypoint && (() => {
            const pt = hoveredWaypoint;
            const [u, v] = pt.pixel;
            const pId = pt.path_id ?? 1;
            const wpIdx = pt.index;

            const pathRep = verificationReport?.path_reports?.find((r: any) => r.path_id === pId);
            const totalSteps = pathRep?.total_interpolated || 1;
            const matchingIssue = pathRep?.issues?.find((iss: any) => {
              if (iss.waypoint_index !== undefined && iss.waypoint_index === wpIdx - 1) return true;
              if (iss.step_index !== undefined) {
                const approxWp = Math.floor((iss.step_index / totalSteps) * (pathRep.total_interpolated || 1));
                return approxWp === wpIdx - 1;
              }
              return false;
            });

            const tipW = 230, tipH = matchingIssue ? 125 : 100;
            const tipX = Math.min(u + 14, natSize.w - tipW - 10);
            const tipY = Math.max(v - tipH - 10, 10);

            return (
              <g transform={`translate(${tipX}, ${tipY})`} style={{ pointerEvents: 'none' }}>
                <rect x={0} y={0} width={tipW} height={tipH} rx={8} fill="rgba(10, 15, 29, 0.95)" stroke="rgba(56, 189, 248, 0.4)" strokeWidth={1} filter="drop-shadow(0px 8px 16px rgba(0,0,0,0.85))" />
                <rect x={0} y={0} width={tipW} height={22} rx={8} fill="rgba(30, 41, 59, 0.8)" />
                <text x={8} y={15} fill="#38bdf8" fontSize={10.5} fontWeight="bold" fontFamily="monospace">
                  Path {pId} · Point {wpIdx}
                </text>
                <text x={tipW - 8} y={15} textAnchor="end" fill="#93c5fd" fontSize={9} fontFamily="monospace">
                  {pt.standoff_distance_mm}mm
                </text>
                <text x={8} y={38} fill="#94a3b8" fontSize={9.5} fontFamily="monospace">
                  Surf [mm]: <tspan fill="#f1f5f9">[{pt.surface_point_base_mm.map(n => n.toFixed(1)).join(', ')}]</tspan>
                </text>
                <text x={8} y={53} fill="#94a3b8" fontSize={9.5} fontFamily="monospace">
                  TCP [mm]: <tspan fill="#fde047" fontWeight="bold">[{pt.tcp_pose_base.x.toFixed(1)}, {pt.tcp_pose_base.y.toFixed(1)}, {pt.tcp_pose_base.z.toFixed(1)}]</tspan>
                </text>
                <text x={8} y={68} fill="#94a3b8" fontSize={9.5} fontFamily="monospace">
                  Normal: <tspan fill="#34d399">[{pt.surface_normal_base.map(n => n.toFixed(2)).join(', ')}]</tspan>
                </text>
                <text x={8} y={83} fill="#94a3b8" fontSize={9.5} fontFamily="monospace">
                  Euler [°]: <tspan fill="#f59e0b">Rx:{pt.tcp_pose_base.rx}° Ry:{pt.tcp_pose_base.ry}° Rz:{pt.tcp_pose_base.rz}°</tspan>
                </text>
                {matchingIssue && (
                  <text x={8} y={102} fill="#f43f5e" fontSize={9.5} fontWeight="bold" fontFamily="monospace">
                    ⚠️ {matchingIssue.type}: {matchingIssue.message}
                  </text>
                )}
              </g>
            );
          })()}
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
