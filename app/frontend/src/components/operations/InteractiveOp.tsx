import React, { useState, useEffect, useRef, type MouseEvent } from 'react';
import { CustomModal, type ModalConfig } from '../common/CustomModal';
import { API_BASE, WS_BASE } from '../../config';
import {
  Play,
  Pause,
  X,
} from 'lucide-react';

import type {
  FileItem,
  Point,
  MaskData,
  WaypointItem,
  ManualPathItem,
  UrdfTcpInfo,
  KinematicsParams,
  VerificationReport,
  SessionData,
  LiveNormalInfo,
  PathStateType,
  PoiConfig,
  SimulationState,
} from './interactive/types';
import { STATE_THEMES } from './interactive/types';
import { computeNormalClientSide } from './interactive/normalComputation';
import { InteractiveCanvas } from './interactive/InteractiveCanvas';
import { TemplateTopBar } from './interactive/TemplateFileManager';
import { TemplateFileList } from './interactive/TemplateFileList';
import { DiagnosticsDashboard } from './interactive/DiagnosticsDashboard';
import { InteractiveActionColumn } from './interactive/InteractiveActionColumn';

interface InteractiveOpProps {
  externalActiveTemplate?: string | null;
  onTemplateChange?: (templateName: string | null) => void;
  onMeshUpdated?: () => void;
  onPathsUpdated?: () => void;
  onPathStateChange?: (state: PathStateType) => void;
  onSimulationJointsChange?: (joints: number[] | null) => void;
}

function projectBasePointToPixel(
  posBaseMm: [number, number, number],
  T_base_cam: number[],
  intrinsics: { fx: number; fy: number; cx: number; cy: number }
): [number, number] | null {
  if (!T_base_cam || T_base_cam.length < 16) return null;
  const R00 = T_base_cam[0], R01 = T_base_cam[1], R02 = T_base_cam[2], tx = T_base_cam[3];
  const R10 = T_base_cam[4], R11 = T_base_cam[5], R12 = T_base_cam[6], ty = T_base_cam[7];
  const R20 = T_base_cam[8], R21 = T_base_cam[9], R22 = T_base_cam[10], tz = T_base_cam[11];

  const dx = posBaseMm[0] - tx;
  const dy = posBaseMm[1] - ty;
  const dz = posBaseMm[2] - tz;

  const Xc = R00 * dx + R10 * dy + R20 * dz;
  const Yc = R01 * dx + R11 * dy + R21 * dz;
  const Zc = R02 * dx + R12 * dy + R22 * dz;

  if (Zc <= 10.0) return null;

  const u = (intrinsics.fx * Xc) / Zc + intrinsics.cx;
  const v = (intrinsics.fy * Yc) / Zc + intrinsics.cy;
  return [Math.round(u * 10) / 10, Math.round(v * 10) / 10];
}

const InteractiveOp: React.FC<InteractiveOpProps> = ({
  externalActiveTemplate,
  onTemplateChange,
  onMeshUpdated,
  onPathsUpdated,
  onPathStateChange,
  onSimulationJointsChange,
}) => {
  // ─── 1. Template & File State ──────────────────────────────────────────
  const [templates, setTemplates] = useState<string[]>([]);
  const [activeTemplate, setActiveTemplate] = useState<string | null>(externalActiveTemplate || null);
  const [files, setFiles] = useState<FileItem[]>([]);
  const [isLoadingTemplate, setIsLoadingTemplate] = useState<boolean>(false);

  // ─── 2. Image & Viewport State ──────────────────────────────────────────
  const [imageUrl, setImageUrl] = useState<string | null>(null);
  const [hasImage, setHasImage] = useState<boolean>(false);
  const [natSize, setNatSize] = useState<{ w: number; h: number } | null>(null);
  const [zoom, setZoom] = useState<number>(1);
  const [pan, setPan] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [isPanning, setIsPanning] = useState<boolean>(false);
  const [isSpacePressed, setIsSpacePressed] = useState<boolean>(false);

  // ─── 3. SAM Segmentation State ──────────────────────────────────────────
  const [segMode, setSegMode] = useState<boolean>(false);
  const [currentPoints, setCurrentPoints] = useState<Point[]>([]);
  const [currentPolygons, setCurrentPolygons] = useState<number[][][]>([]);
  const [committedMasks, setCommittedMasks] = useState<MaskData[]>([]);
  const [savedMasks, setSavedMasks] = useState<MaskData[]>([]);
  const [showMasksOverlay, setShowMasksOverlay] = useState<boolean>(true);

  // ─── 4. Three-State Manual TCP Path State (RAW / OPT / POI) ──────────────
  const [activeState, setActiveState] = useState<PathStateType>('raw');
  const [manualPathMode, setManualPathMode] = useState<boolean>(false);
  const [manualPaths, setManualPaths] = useState<ManualPathItem[]>([]);
  const [rawPaths, setRawPaths] = useState<ManualPathItem[]>([]);
  const [optPaths, setOptPaths] = useState<ManualPathItem[]>([]);
  const [poiPaths, setPoiPaths] = useState<ManualPathItem[]>([]);
  const [currentManualPoints, setCurrentManualPoints] = useState<WaypointItem[]>([]);
  const [selectedPathIdForEdit, setSelectedPathIdForEdit] = useState<number | null>(null);
  const [standoffDistMm, setStandoffDistMm] = useState<number>(150.0);
  const [showManualPathsOverlay, setShowManualPathsOverlay] = useState<boolean>(true);
  const [hoveredWaypoint, setHoveredWaypoint] = useState<WaypointItem | null>(null);
  const [highlightedPathId, setHighlightedPathId] = useState<number | null>(null);
  const [sessionData, setSessionData] = useState<SessionData | null>(null);
  const [mousePixel, setMousePixel] = useState<{ u: number; v: number } | null>(null);
  const [liveNormal, setLiveNormal] = useState<LiveNormalInfo | null>(null);

  // ─── 5. Three-State Diagnostics & Verification State ─────────────────────
  const [showDiagnostics, setShowDiagnostics] = useState<boolean>(false);
  const [rawReport, setRawReport] = useState<VerificationReport | null>(null);
  const [optReport, setOptReport] = useState<VerificationReport | null>(null);
  const [poiReport, setPoiReport] = useState<VerificationReport | null>(null);
  const [isVerifying, setIsVerifying] = useState<boolean>(false);
  const [isOptimizing, setIsOptimizing] = useState<boolean>(false);
  const [isKinParamsOpen, setIsKinParamsOpen] = useState<boolean>(false);
  const [urdfTcpInfo, setUrdfTcpInfo] = useState<UrdfTcpInfo | null>(null);
  const [kinParams, setKinParams] = useState<KinematicsParams>({
    stepSizeMm: 1.5,
    linearSpeedMmS: 120.0,
  });
  const [poiConfig, setPoiConfig] = useState<PoiConfig>({
    anchor_source: 'home',
    ref_rpy_deg: [90.0, 0.0, 90.0],
    tolerance_rpy_deg: [10.0, 10.0, 180.0],
  });

  // ─── 6. Action Execution State ──────────────────────────────────────────
  const [isCapturing, setIsCapturing] = useState<boolean>(false);
  const [isReconstructing, setIsReconstructing] = useState<boolean>(false);

  // ─── 7. 3D/2D Synchronized Simulation Engine State (Feature 7) ───────────
  const [simulationState, setSimulationState] = useState<SimulationState | null>(null);
  const simAnimFrameRef = useRef<number | null>(null);
  const simDataRef = useRef<{
    steps: Array<{
      q_deg: number[];
      tcp: { x: number; y: number; z: number; rx: number; ry: number; rz: number };
      pixel: [number, number] | null;
      pathIdx: number;
    }>;
    stepIndex: number;
    speedMultiplier: number;
    isPlaying: boolean;
    stateType: PathStateType;
  }>({
    steps: [],
    stepIndex: 0,
    speedMultiplier: 1.0,
    isPlaying: false,
    stateType: 'raw',
  });

  // ─── 8. Modal Dialog Config ─────────────────────────────────────────────
  const [modalConfig, setModalConfig] = useState<ModalConfig | null>(null);

  // ─── 9. Robot Real-Time State (via WebSocket) ──────────────────────────
  const [robotConnected, setRobotConnected] = useState<boolean>(false);
  const robotWsRef = useRef<WebSocket | null>(null);
  // Ref for per-WS-message projection of real TCP → pixel (avoids stale closure)
  const sessionDataRef = useRef<typeof sessionData>(null);
  useEffect(() => { sessionDataRef.current = sessionData; }, [sessionData]);

  // Sync external active template
  useEffect(() => {
    if (externalActiveTemplate && externalActiveTemplate !== activeTemplate) {
      handleSelectTemplate(externalActiveTemplate);
    }
  }, [externalActiveTemplate]);

  // WebSocket: monitor robot connection status + real execution progress
  useEffect(() => {
    let ws: WebSocket | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;

    const connect = () => {
      ws = new WebSocket(`${WS_BASE}/api/calib/robot/ws`);
      robotWsRef.current = ws;

      ws.onopen = () => {};
      ws.onmessage = (e) => {
        try {
          const msg = JSON.parse(e.data) as { type: string; data: any };

          if (msg.type === 'robot_state') {
            // Mark robot as connected any time we get a valid state push
            setRobotConnected(true);

            // In real-exec mode: drive currentPixel + currentJoints from real TCP
            setSimulationState((prev) => {
              if (!prev?.isRealExec) return prev;
              const pose: number[] = msg.data.pose ?? [];
              if (pose.length < 3) return prev;

              // Project TCP XYZ to 2D pixel using calibration
              let pixel: [number, number] | null = null;
              const sd = sessionDataRef.current;
              if (sd?.T && sd.T.length >= 16) {
                pixel = projectBasePointToPixel(
                  [pose[0], pose[1], pose[2]],
                  sd.T,
                  { fx: sd.fx, fy: sd.fy, cx: sd.cx, cy: sd.cy }
                );
              }

              const joints: number[] = (msg.data.joint ?? []).map((v: number) =>
                typeof v === 'number' ? v : 0
              );
              if (onSimulationJointsChange) onSimulationJointsChange(joints);

              return {
                ...prev,
                currentPixel: pixel ?? prev.currentPixel,
                currentJoints: joints.length > 0 ? joints : prev.currentJoints,
                currentTcpPose: {
                  x: pose[0] ?? prev.currentTcpPose.x,
                  y: pose[1] ?? prev.currentTcpPose.y,
                  z: pose[2] ?? prev.currentTcpPose.z,
                  rx: pose[3] ?? prev.currentTcpPose.rx,
                  ry: pose[4] ?? prev.currentTcpPose.ry,
                  rz: pose[5] ?? prev.currentTcpPose.rz,
                },
              };
            });
          } else if (msg.type === 'exec_progress') {
            // Update progress bar and path index from real waypoint events
            const d = msg.data as {
              current_waypoint: number;
              total_waypoints: number;
              path_idx: number;
              total_paths: number;
              progress: number;
            };
            setSimulationState((prev) => {
              if (!prev?.isRealExec) return prev;
              const isDone = d.current_waypoint >= d.total_waypoints;
              return {
                ...prev,
                progress: d.progress,
                currentPathIndex: d.path_idx,
                currentStep: d.current_waypoint,
                totalSteps: d.total_waypoints,
                isPlaying: !isDone,
              };
            });
          }
        } catch {}
      };
      ws.onclose = () => {
        // When WS closes, robot might be disconnected
        setRobotConnected(false);
        reconnectTimer = setTimeout(connect, 3000);
      };
    };

    connect();
    return () => {
      if (reconnectTimer) clearTimeout(reconnectTimer);
      ws?.close();
    };
  }, []);


  // Load template list on mount
  useEffect(() => {
    fetchTemplates();
  }, []);

  // Keyboard space key listener for CAD/Figma canvas panning
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.code === 'Space' && !isSpacePressed && (e.target as HTMLElement).tagName !== 'INPUT') {
        setIsSpacePressed(true);
      }
    };
    const handleKeyUp = (e: KeyboardEvent) => {
      if (e.code === 'Space') {
        setIsSpacePressed(false);
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [isSpacePressed]);

  // ─── Templates API & Atomic Synchronization ─────────────────────────────
  const fetchTemplates = async () => {
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates`);
      if (!res.ok) return;
      const data = await res.json();
      setTemplates(data.templates || []);
      if (!activeTemplate && data.templates && data.templates.length > 0) {
        handleSelectTemplate(data.templates[0]);
      }
    } catch (err) {
      console.error('Failed to fetch templates:', err);
    }
  };

  const handleSelectTemplate = async (templateName: string) => {
    stopSimulation();
    setActiveTemplate(templateName);
    if (onTemplateChange) onTemplateChange(templateName);

    // Reset local edit states
    setSegMode(false);
    setManualPathMode(false);
    setCurrentPoints([]);
    setCurrentPolygons([]);
    setCommittedMasks([]);
    setCurrentManualPoints([]);
    setSelectedPathIdForEdit(null);
    setHighlightedPathId(null);
    setHoveredWaypoint(null);

    // Fetch batch summary atomically
    setIsLoadingTemplate(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${templateName}/summary`);
      if (!res.ok) throw new Error(`Summary status ${res.status}`);
      const summary = await res.json();

      setFiles(summary.files || []);
      setHasImage(summary.has_image || false);
      setSavedMasks(summary.masks || []);

      const raw = summary.raw_paths || [];
      const opt = summary.opt_paths || [];
      const poi = summary.poi_paths || [];
      setRawPaths(raw);
      setOptPaths(opt);
      setPoiPaths(poi);

      // Determine initial active state (prefer POI if exists, then OPT, then RAW)
      let initialSt: PathStateType = 'raw';
      if (poi.length > 0) initialSt = 'poi';
      else if (opt.length > 0) initialSt = 'opt';

      setActiveState(initialSt);
      setManualPaths(initialSt === 'poi' ? poi : (initialSt === 'opt' ? opt : raw));
      if (onPathStateChange) onPathStateChange(initialSt);

      if (summary.standoff_distance_mm) {
        setStandoffDistMm(summary.standoff_distance_mm);
      }

      if (summary.urdf_tcp) {
        setUrdfTcpInfo(summary.urdf_tcp);
      }

      setRawReport(summary.raw_report || null);
      setOptReport(summary.opt_report || null);
      setPoiReport(summary.poi_report || null);

      if (summary.has_image) {
        const newImgUrl = `${API_BASE}/templates/${templateName}/scan.jpg?v=${Date.now()}`;
        // Image-First barrier: Preload image before dropping backdrop
        await new Promise<void>((resolve) => {
          const img = new Image();
          img.onload = () => {
            setImageUrl(newImgUrl);
            setNatSize({ w: img.naturalWidth, h: img.naturalHeight });
            resolve();
          };
          img.onerror = () => {
            setImageUrl(newImgUrl);
            resolve();
          };
          img.src = newImgUrl;
        });
      } else {
        setImageUrl(null);
        setNatSize(null);
      }

      // Load session depth data for client-side normal computation and projection
      await loadSessionData(templateName);
    } catch (err) {
      console.error('Failed to load template atomic summary:', err);
    } finally {
      setIsLoadingTemplate(false);
    }
  };

  const loadSessionData = async (templateName: string) => {
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${templateName}/session_data`);
      if (!res.ok) throw new Error('Session data fetch failed');
      const data = await res.json();
      const b64 = data.depth_flat_b64 as string;
      const binary = atob(b64);
      const bytes = new Uint8Array(binary.length);
      for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
      const depthF32 = new Float32Array(bytes.buffer);
      setSessionData({
        width: data.width,
        height: data.height,
        depth: depthF32,
        fx: data.intrinsics.fx,
        fy: data.intrinsics.fy,
        cx: data.intrinsics.cx,
        cy: data.intrinsics.cy,
        T: data.T_base_camera,
        calib_source: data.calib_source,
      });
    } catch (err) {
      console.warn('Failed to load session depth data:', err);
    }
  };

  const activatePathStateSnapshot = (newState: PathStateType, explicitPaths?: ManualPathItem[]) => {
    const targetPaths = explicitPaths ?? (newState === 'poi' ? poiPaths : (newState === 'opt' ? optPaths : rawPaths));
    stopSimulation();
    setActiveState(newState);
    setManualPaths(targetPaths.map((path) => ({ ...path, points: [...(path.points || [])] })));
    setHoveredWaypoint(null);
    setHighlightedPathId(null);
    setSelectedPathIdForEdit(null);
    setCurrentManualPoints([]);
    if (onPathStateChange) onPathStateChange(newState);
    if (onPathsUpdated) onPathsUpdated();
  };

  // State Switcher (RAW / OPT / POI)
  const handleSelectActiveState = (newState: PathStateType) => {
    activatePathStateSnapshot(newState);
  };

  // ─── SAM Segmentation Handlers ──────────────────────────────────────────
  const handleToggleSegMode = async () => {
    if (segMode) {
      setSegMode(false);
      setCurrentPoints([]);
      setCurrentPolygons([]);
      return;
    }
    if (!activeTemplate || !hasImage) return;
    setSegMode(true);
    setCurrentPoints([]);
    setCurrentPolygons([]);
    if (savedMasks.length > 0) {
      setCommittedMasks([...savedMasks]);
    } else {
      setCommittedMasks([]);
    }

    try {
      await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/sam/init`, {
        method: 'POST',
      });
    } catch (err) {
      console.warn('Failed to init SAM session:', err);
    }
  };

  const handleSegImageClick = async (e: MouseEvent<SVGSVGElement>) => {
    if (!segMode || !natSize || !activeTemplate || isSpacePressed) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const x = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const y = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);

    const newPoints = [...currentPoints, { x, y, label: 1 }];
    setCurrentPoints(newPoints);
    predictMask(newPoints);
  };

  const handleSegContextMenu = (e: MouseEvent<SVGSVGElement>) => {
    if (!segMode || !natSize || !activeTemplate) return;
    e.preventDefault();
    const rect = e.currentTarget.getBoundingClientRect();
    const x = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const y = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);

    const newPoints = [...currentPoints, { x, y, label: 0 }];
    setCurrentPoints(newPoints);
    predictMask(newPoints);
  };

  const predictMask = async (pts: Point[]) => {
    if (!activeTemplate || pts.length === 0) {
      setCurrentPolygons([]);
      return;
    }
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/sam/predict`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          points: pts.map((p) => [p.x, p.y]),
          labels: pts.map((p) => p.label),
        }),
      });
      if (!res.ok) throw new Error('Predict failed');
      const data = await res.json();
      setCurrentPolygons(data.polygons || []);
    } catch (err) {
      console.error('SAM predict error:', err);
    }
  };

  const handleCommitCurrentSegMask = () => {
    if (currentPolygons.length === 0) return;
    const newMask: MaskData = {
      id: committedMasks.length + 1,
      points: [...currentPoints],
      polygons: currentPolygons,
    };
    setCommittedMasks([...committedMasks, newMask]);
    setCurrentPoints([]);
    setCurrentPolygons([]);
  };

  const handleSaveAllSegMasks = async () => {
    if (!activeTemplate) return;
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/sam/save`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ committed_masks: committedMasks }),
      });
      if (!res.ok) throw new Error('Failed to save masks');
      setSavedMasks(committedMasks);
      setSegMode(false);
      await fetchTemplateFiles(activeTemplate);
      setModalConfig({
        isOpen: true,
        title: 'Success',
        message: `Saved ${committedMasks.length} segment masks successfully.`,
        type: 'alert',
      });
    } catch (err: any) {
      setModalConfig({
        isOpen: true,
        title: 'Error',
        message: `Failed to save masks: ${err.message}`,
        type: 'alert',
      });
    }
  };

  const fetchTemplateFiles = async (templateName: string) => {
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${templateName}/files`);
      if (res.ok) {
        const data = await res.json();
        setFiles(data.files || []);
      }
    } catch (err) {
      console.error('Failed to refresh files:', err);
    }
  };

  // ─── Manual TCP Path Handlers ───────────────────────────────────────────
  const handleManualMouseMove = (e: MouseEvent<SVGSVGElement>) => {
    if (!manualPathMode || !natSize || isSpacePressed) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const u = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const v = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);
    setMousePixel({ u, v });

    if (sessionData) {
      const res = computeNormalClientSide(sessionData, u, v, standoffDistMm);
      if (res) {
        setLiveNormal({
          dx: res.proj_dx,
          dy: res.proj_dy,
          surfPointBase: res.surf_base,
          normalBase: res.normal_base,
          tcpPose: {
            x: res.tcp_base[0],
            y: res.tcp_base[1],
            z: res.tcp_base[2],
            rx: res.euler_deg[0],
            ry: res.euler_deg[1],
            rz: res.euler_deg[2],
          },
        });
      }
    }
  };

  const handleManualImageClick = (e: MouseEvent<SVGSVGElement>) => {
    if (!manualPathMode || !natSize || isSpacePressed || !liveNormal?.surfPointBase || !liveNormal?.tcpPose) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const u = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const v = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);

    const newWaypoint: WaypointItem = {
      index: currentManualPoints.length + 1,
      pixel: [u, v],
      surface_point_cam_mm: [0, 0, 0],
      surface_point_base_mm: liveNormal.surfPointBase,
      surface_normal_base: liveNormal.normalBase || [0, 0, 1],
      surface_normal_cam: [0, 0, 1],
      standoff_distance_mm: standoffDistMm,
      tcp_pose_base: liveNormal.tcpPose,
      normal_2d_proj: [liveNormal.dx, liveNormal.dy],
    };

    setCurrentManualPoints([...currentManualPoints, newWaypoint]);
  };

  const handleCommitManualPath = () => {
    if (currentManualPoints.length === 0) return;
    if (selectedPathIdForEdit) {
      setManualPaths(
        manualPaths.map((p) =>
          p.path_id === selectedPathIdForEdit
            ? { ...p, points: currentManualPoints }
            : p
        )
      );
      setSelectedPathIdForEdit(null);
    } else {
      const newPathId = manualPaths.length > 0 ? Math.max(...manualPaths.map((p) => p.path_id)) + 1 : 1;
      const newPath: ManualPathItem = {
        path_id: newPathId,
        name: `Path ${newPathId}`,
        points: currentManualPoints,
      };
      setManualPaths([...manualPaths, newPath]);
    }
    setCurrentManualPoints([]);
  };

  const handleSaveManualPaths = async () => {
    if (!activeTemplate) return;
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/manual_paths?state_type=raw`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          paths: manualPaths,
          standoff_distance_mm: standoffDistMm,
        }),
      });
      if (!res.ok) throw new Error('Failed to save manual paths');
      setRawPaths(manualPaths);
      activatePathStateSnapshot('raw', manualPaths);
      setManualPathMode(false);
      await fetchTemplateFiles(activeTemplate);
      setModalConfig({
        isOpen: true,
        title: 'Success',
        message: `Saved ${manualPaths.length} TCP paths to scan.raw.path.yaml.`,
        type: 'alert',
      });
    } catch (err: any) {
      setModalConfig({
        isOpen: true,
        title: 'Error',
        message: `Failed to save paths: ${err.message}`,
        type: 'alert',
      });
    }
  };

  // ─── Diagnostics & POI Optimization Handlers ────────────────────────────
  const handleRunDiagnostics = async (targetState?: PathStateType) => {
    if (!activeTemplate) return;
    const st = targetState || activeState;
    setIsVerifying(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/verify_paths`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          state_type: st,
          options: {
            step_size_mm: kinParams.stepSizeMm,
            linear_velocity_mm_s: kinParams.linearSpeedMmS,
          },
        }),
      });
      if (!res.ok) throw new Error('Verification failed');
      const data = await res.json();
      if (st === 'poi') setPoiReport(data);
      else if (st === 'opt') setOptReport(data);
      else setRawReport(data);

      setShowDiagnostics(true);
    } catch (err: any) {
      setModalConfig({
        isOpen: true,
        title: 'Verification Error',
        message: err.message,
        type: 'alert',
      });
    } finally {
      setIsVerifying(false);
    }
  };

  const handleApplyOptimization = async (mode: 'opt' | 'poi') => {
    if (!activeTemplate) return;
    setIsOptimizing(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/optimize_paths?mode=${mode}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          mode,
          options: {
            step_size_mm: kinParams.stepSizeMm,
            linear_velocity_mm_s: kinParams.linearSpeedMmS,
          },
          poi_config: poiConfig,
        }),
      });
      if (!res.ok) {
        const errBody = await res.json().catch(() => null);
        throw new Error(errBody?.detail || `${mode.toUpperCase()} Optimization failed`);
      }
      const data = await res.json();

      const rep = data.verification_report || data;
      const paths = (data.optimized_paths || rep.optimized_paths || []) as ManualPathItem[];

      if (mode === 'poi') {
        setPoiPaths(paths);
        setPoiReport(rep);
        activatePathStateSnapshot('poi', paths);
      } else {
        setOptPaths(paths);
        setOptReport(rep);
        activatePathStateSnapshot('opt', paths);
      }

      await fetchTemplateFiles(activeTemplate);
      setShowDiagnostics(true);
    } catch (err: any) {
      setModalConfig({
        isOpen: true,
        title: 'Optimization Error',
        message: err.message,
        type: 'alert',
      });
    } finally {
      setIsOptimizing(false);
    }
  };

  const handleFetchAnchorPose = async (source: 'home' | 'live') => {
    try {
      const res = await fetch(`${API_BASE}/api/interactive/robot/anchor_pose?source=${source}`);
      if (!res.ok) {
        const errBody = await res.json().catch(() => null);
        throw new Error(errBody?.detail || 'Failed to fetch anchor pose');
      }
      const data = await res.json();
      setPoiConfig((prev) => ({
        ...prev,
        anchor_source: data.source || source,
        ref_rpy_deg: data.rpy_deg || prev.ref_rpy_deg,
        tolerance_rpy_deg: data.default_tolerance_rpy_deg || prev.tolerance_rpy_deg,
      }));
      console.log(`[POI] Captured ${source.toUpperCase()} anchor pose: [${(data.rpy_deg || []).map((v: number) => v.toFixed(1)).join(', ')}]°`);
    } catch (err: any) {
      setModalConfig({
        isOpen: true,
        title: 'Anchor Pose Error',
        message: err.message,
        type: 'alert',
      });
    }
  };

  // ─── 3D/2D Synchronized Simulation Engine (Feature 7) ────────────────────
  const startSimulation = (stateType: PathStateType = activeState, targetPathId?: number | null) => {
    stopSimulation();
    const rep = stateType === 'poi' ? poiReport : (stateType === 'opt' ? optReport : rawReport);
    if (!rep?.path_reports || rep.path_reports.length === 0) {
      setModalConfig({
        isOpen: true,
        title: 'Simulation Notice',
        message: `No trajectory simulation steps found for ${stateType.toUpperCase()}. Running diagnostics first...`,
        type: 'alert',
      });
      handleRunDiagnostics(stateType);
      return;
    }

    // Filter target path reports if a specific path is requested
    const targetReports = (targetPathId !== undefined && targetPathId !== null)
      ? rep.path_reports.filter((pr: any, idx: number) => (pr.path_id === targetPathId || idx + 1 === targetPathId))
      : rep.path_reports;

    // Build dense playback steps
    const simSteps: Array<{
      q_deg: number[];
      tcp: { x: number; y: number; z: number; rx: number; ry: number; rz: number };
      pixel: [number, number] | null;
      pathIdx: number;
    }> = [];

    targetReports.forEach((pr, pIdx) => {
      const realPIdx = rep.path_reports ? rep.path_reports.indexOf(pr) : pIdx;
      const tq = pr.trajectory_q || [];
      const tt = pr.trajectory_tcp || [];
      const totalPSteps = Math.min(tq.length, tt.length);

      for (let s = 0; s < totalPSteps; s++) {
        const q_rad = tq[s];
        const q_deg = q_rad.map((r) => (r * 180.0) / Math.PI);
        const tcpArr = tt[s];
        const tcpPose = {
          x: tcpArr[0],
          y: tcpArr[1],
          z: tcpArr[2],
          rx: tcpArr[3],
          ry: tcpArr[4],
          rz: tcpArr[5],
        };

        // Project base 3D point to 2D pixel
        let pixelProj: [number, number] | null = null;
        if (sessionData) {
          pixelProj = projectBasePointToPixel(
            [tcpArr[0], tcpArr[1], tcpArr[2]],
            sessionData.T,
            { fx: sessionData.fx, fy: sessionData.fy, cx: sessionData.cx, cy: sessionData.cy }
          );
        }

        simSteps.push({
          q_deg,
          tcp: tcpPose,
          pixel: pixelProj,
          pathIdx: realPIdx >= 0 ? realPIdx : pIdx,
        });
      }
    });

    if (simSteps.length === 0) {
      setModalConfig({
        isOpen: true,
        title: 'Simulation Notice',
        message: `No interpolated trajectory steps available for ${stateType.toUpperCase()}.`,
        type: 'alert',
      });
      return;
    }

    simDataRef.current = {
      steps: simSteps,
      stepIndex: 0,
      speedMultiplier: simulationState?.speed || 1.0,
      isPlaying: true,
      stateType: stateType,
    };

    setSimulationState({
      isPlaying: true,
      progress: 0,
      speed: simDataRef.current.speedMultiplier,
      currentPathIndex: 0,
      currentStep: 0,
      totalSteps: simSteps.length,
      currentJoints: simSteps[0].q_deg,
      currentTcpPose: simSteps[0].tcp,
      currentPixel: simSteps[0].pixel,
      activeState: stateType,
    });

    // Start 60 FPS animation loop
    let lastTime = performance.now();
    const animate = (time: number) => {
      const dt = (time - lastTime) / 1000.0;
      lastTime = time;

      const sim = simDataRef.current;
      if (!sim.isPlaying || sim.steps.length === 0) return;

      // Advance step based on speed multiplier (nominal MoveL step rate ~ 60 steps/sec)
      const stepIncrement = Math.max(1, Math.round(60 * dt * sim.speedMultiplier));
      sim.stepIndex += stepIncrement;

      if (sim.stepIndex >= sim.steps.length) {
        sim.stepIndex = sim.steps.length - 1;
        sim.isPlaying = false;
      }

      const curr = sim.steps[sim.stepIndex];
      const prog = sim.stepIndex / (sim.steps.length - 1);

      setSimulationState({
        isPlaying: sim.isPlaying,
        progress: prog,
        speed: sim.speedMultiplier,
        currentPathIndex: curr.pathIdx,
        currentStep: sim.stepIndex,
        totalSteps: sim.steps.length,
        currentJoints: curr.q_deg,
        currentTcpPose: curr.tcp,
        currentPixel: curr.pixel,
        activeState: sim.stateType,
      });

      if (onSimulationJointsChange) {
        onSimulationJointsChange(curr.q_deg);
      }

      simAnimFrameRef.current = requestAnimationFrame(animate);
    };

    simAnimFrameRef.current = requestAnimationFrame(animate);
  };

  const pauseSimulation = () => {
    if (simAnimFrameRef.current) cancelAnimationFrame(simAnimFrameRef.current);
    simDataRef.current.isPlaying = false;
    setSimulationState((prev) => (prev ? { ...prev, isPlaying: false } : null));
  };

  const resumeSimulation = () => {
    if (!simulationState || simDataRef.current.steps.length === 0) return;
    simDataRef.current.isPlaying = true;
    setSimulationState((prev) => (prev ? { ...prev, isPlaying: true } : null));

    let lastTime = performance.now();
    const animate = (time: number) => {
      const dt = (time - lastTime) / 1000.0;
      lastTime = time;

      const sim = simDataRef.current;
      if (!sim.isPlaying || sim.steps.length === 0) return;

      const stepIncrement = Math.max(1, Math.round(60 * dt * sim.speedMultiplier));
      sim.stepIndex += stepIncrement;

      if (sim.stepIndex >= sim.steps.length) {
        sim.stepIndex = sim.steps.length - 1;
        sim.isPlaying = false;
      }

      const curr = sim.steps[sim.stepIndex];
      const prog = sim.stepIndex / (sim.steps.length - 1);

      setSimulationState({
        isPlaying: sim.isPlaying,
        progress: prog,
        speed: sim.speedMultiplier,
        currentPathIndex: curr.pathIdx,
        currentStep: sim.stepIndex,
        totalSteps: sim.steps.length,
        currentJoints: curr.q_deg,
        currentTcpPose: curr.tcp,
        currentPixel: curr.pixel,
        activeState: sim.stateType,
      });

      if (onSimulationJointsChange) {
        onSimulationJointsChange(curr.q_deg);
      }

      simAnimFrameRef.current = requestAnimationFrame(animate);
    };

    simAnimFrameRef.current = requestAnimationFrame(animate);
  };

  const seekSimulation = (targetProgress: number) => {
    const sim = simDataRef.current;
    if (sim.steps.length === 0) return;
    const targetIdx = Math.min(
      Math.max(0, Math.floor(targetProgress * (sim.steps.length - 1))),
      sim.steps.length - 1
    );
    sim.stepIndex = targetIdx;
    const curr = sim.steps[targetIdx];

    setSimulationState((prev) =>
      prev
        ? {
          ...prev,
          progress: targetProgress,
          currentPathIndex: curr.pathIdx,
          currentStep: targetIdx,
          currentJoints: curr.q_deg,
          currentTcpPose: curr.tcp,
          currentPixel: curr.pixel,
        }
        : null
    );

    if (onSimulationJointsChange) {
      onSimulationJointsChange(curr.q_deg);
    }
  };

  const stopSimulation = () => {
    if (simAnimFrameRef.current) cancelAnimationFrame(simAnimFrameRef.current);
    simDataRef.current.isPlaying = false;
    simDataRef.current.steps = [];
    setSimulationState(null);
    if (onSimulationJointsChange) {
      onSimulationJointsChange(null);
    }
  };

  const setSimulationSpeed = (speed: number) => {
    simDataRef.current.speedMultiplier = speed;
    setSimulationState((prev) => (prev ? { ...prev, speed } : null));
  };

  // ─── Direct Robot Path Execution with 2D Live Progress ────────────────────
  const handleDirectExecutePath = async (fileName: string, stateType?: PathStateType, pathId?: number | null) => {
    if (!activeTemplate) return;
    const targetState = stateType || activeState;
    handleSelectActiveState(targetState);

    // Count total waypoints in the target paths for progress state initialisation
    const srcPaths = targetState === 'poi' ? poiPaths : (targetState === 'opt' ? optPaths : rawPaths);
    const filteredPaths = (pathId !== null && pathId !== undefined)
      ? srcPaths.filter((p) => p.path_id === pathId)
      : srcPaths;
    const totalWaypoints = filteredPaths.reduce((sum, p) => sum + (p.points?.length ?? 0), 0);

    // 1. Enter real-exec mode: show trajectory overlay, drive dot from real WS TCP data
    stopSimulation();
    setSimulationState({
      isPlaying: true,
      isRealExec: true,
      progress: 0,
      speed: 1.0,
      currentPathIndex: 0,
      currentStep: 0,
      totalSteps: Math.max(totalWaypoints, 1),
      currentJoints: [0, 0, 0, 0, 0, 0],
      currentTcpPose: { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 },
      currentPixel: null,
      activeState: targetState,
    });

    // 2. Dispatch real robot execution asynchronously
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/execute_yaml_path`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          file_name: fileName,
          path_id: pathId,
        }),
      });
      if (!res.ok) {
        const err = await res.json();
        console.error('Robot path execution error:', err.detail || 'Execution failed');
      }
    } catch (err: any) {
      console.error('Failed to execute path on robot:', err);
    } finally {
      // Mark done when HTTP completes (WS exec_progress should have already done this)
      setSimulationState((prev) => prev?.isRealExec ? { ...prev, isPlaying: false } : prev);
    }
  };


  // ─── Actions: Capture & Reconstruct ─────────────────────────────────────
  const handleTriggerCapture = async () => {
    if (!activeTemplate) return;
    setIsCapturing(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/capture`, {
        method: 'POST',
      });
      if (!res.ok) throw new Error('Capture failed');
      await handleSelectTemplate(activeTemplate);
    } catch (err: any) {
      setModalConfig({
        isOpen: true,
        title: 'Capture Error',
        message: err.message,
        type: 'alert',
      });
    } finally {
      setIsCapturing(false);
    }
  };

  const handleTriggerReconstruct = async () => {
    if (!activeTemplate) return;
    setIsReconstructing(true);
    try {
      const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/reconstruct`, {
        method: 'POST',
      });
      if (!res.ok) throw new Error('Reconstruction failed');
      if (onMeshUpdated) onMeshUpdated();
      await handleSelectTemplate(activeTemplate);
      setModalConfig({
        isOpen: true,
        title: 'Reconstruction Finished',
        message: '3D surface mesh successfully generated and loaded into 3D viewer.',
        type: 'alert',
      });
    } catch (err: any) {
      setModalConfig({
        isOpen: true,
        title: 'Reconstruction Error',
        message: err.message,
        type: 'alert',
      });
    } finally {
      setIsReconstructing(false);
    }
  };

  // Helper to render SVG polygon path
  const renderPolygons = (polygons: number[][][], fill: string, stroke?: string) => {
    return (
      <path
        d={polygons.map((poly) => poly.map((pt, i) => `${i === 0 ? 'M' : 'L'} ${pt[0]} ${pt[1]}`).join(' ') + ' Z').join(' ')}
        fill={fill}
        stroke={stroke || 'none'}
        strokeWidth={1.5}
      />
    );
  };

  return (
    <div className="flex-1 flex flex-col h-full w-full bg-slate-950 overflow-hidden relative font-sans select-none">
      {/* Top Horizontal Template Switcher */}
      <TemplateTopBar
        templates={templates}
        activeTemplate={activeTemplate}
        onSelectTemplate={handleSelectTemplate}
        onOpenCreateTemplateModal={() => {
          const now = new Date();
          const defaultName = now.toISOString().replace(/[-:T]/g, '_').slice(0, 15);
          setModalConfig({
            isOpen: true,
            title: 'Create New Template',
            message: 'Enter name for the new template group:',
            type: 'input',
            defaultValue: defaultName,
            onConfirm: async (name) => {
              if (!name) return;
              try {
                const res = await fetch(`${API_BASE}/api/interactive/templates`, {
                  method: 'POST',
                  headers: { 'Content-Type': 'application/json' },
                  body: JSON.stringify({ name }),
                });
                if (!res.ok) throw new Error('Failed to create template');
                await fetchTemplates();
                await handleSelectTemplate(name);
              } catch (err: any) {
                alert(err.message);
              }
            },
          });
        }}
        onOpenDeleteTemplateModal={(t) => {
          setModalConfig({
            isOpen: true,
            title: 'Delete Template',
            message: `Are you sure you want to delete template '${t}'? All associated files will be deleted permanently.`,
            type: 'confirm',
            onConfirm: async () => {
              try {
                const res = await fetch(`${API_BASE}/api/interactive/templates/${t}`, {
                  method: 'DELETE',
                });
                if (!res.ok) throw new Error('Failed to delete template');
                setActiveTemplate(null);
                await fetchTemplates();
              } catch (err: any) {
                alert(err.message);
              }
            },
          });
        }}
      />

      {/* Main Operational 3-Column Area */}
      <div className="flex-1 flex overflow-hidden relative min-h-0">
        {/* Column 1: 2D Image Viewport & Overlays */}
        <InteractiveCanvas
          imageUrl={imageUrl}
          isLoadingTemplate={isLoadingTemplate}
          segMode={segMode}
          manualPathMode={manualPathMode}
          showMasksOverlay={showMasksOverlay}
          showManualPathsOverlay={showManualPathsOverlay}
          savedMasks={savedMasks}
          committedMasks={committedMasks}
          currentPolygons={currentPolygons}
          currentPoints={currentPoints}
          manualPaths={manualPaths}
          currentManualPoints={currentManualPoints}
          selectedPathIdForEdit={selectedPathIdForEdit}
          highlightedPathId={highlightedPathId}
          hoveredWaypoint={hoveredWaypoint}
          mousePixel={mousePixel}
          liveNormal={liveNormal}
          natSize={natSize}
          verificationReport={
            activeState === 'poi' ? poiReport : (activeState === 'opt' ? optReport : rawReport)
          }
          activeState={activeState}
          simulationState={simulationState}
          sessionData={sessionData}
          onSelectActiveState={handleSelectActiveState}
          zoom={zoom}
          pan={pan}
          isPanning={isPanning}
          isSpacePressed={isSpacePressed}
          standoffDistMm={standoffDistMm}
          setZoom={setZoom}
          setPan={setPan}
          setIsPanning={setIsPanning}
          setNatSize={setNatSize}
          setHighlightedPathId={setHighlightedPathId}
          setHoveredWaypoint={setHoveredWaypoint}
          setStandoffDistMm={setStandoffDistMm}
          onSelectPathForEdit={(pathId) => {
            const p = manualPaths.find((it) => it.path_id === pathId);
            if (p) {
              setSelectedPathIdForEdit(pathId);
              setCurrentManualPoints([...p.points]);
              setManualPathMode(true);
            }
          }}
          onManualMouseMove={handleManualMouseMove}
          onManualImageClick={handleManualImageClick}
          onDeleteSingleWaypoint={(idx) => {
            const updated = currentManualPoints.filter((_, i) => i !== idx);
            setCurrentManualPoints(updated.map((wp, i) => ({ ...wp, index: i + 1 })));
          }}
          onSegImageClick={handleSegImageClick}
          onSegContextMenu={handleSegContextMenu}
          onToggleMasksOverlay={() => setShowMasksOverlay(!showMasksOverlay)}
          onToggleManualPathsOverlay={() => setShowManualPathsOverlay(!showManualPathsOverlay)}
          onUndoSegPoint={() => {
            if (currentPoints.length === 0) return;
            const updated = currentPoints.slice(0, -1);
            setCurrentPoints(updated);
            if (updated.length > 0) predictMask(updated);
            else setCurrentPolygons([]);
          }}
          onClearCurrentSegPoints={() => {
            setCurrentPoints([]);
            setCurrentPolygons([]);
          }}
          onCommitCurrentSegMask={handleCommitCurrentSegMask}
          onSaveAllSegMasks={handleSaveAllSegMasks}
          onClearAllMasks={() => {
            setCommittedMasks([]);
            setCurrentPoints([]);
            setCurrentPolygons([]);
          }}
          onExitSegMode={() => {
            setSegMode(false);
            setCurrentPoints([]);
            setCurrentPolygons([]);
          }}
          onUndoManualPoint={() => {
            if (currentManualPoints.length === 0) return;
            setCurrentManualPoints(currentManualPoints.slice(0, -1));
          }}
          onClearCurrentManualPoints={() => setCurrentManualPoints([])}
          onCommitManualPath={handleCommitManualPath}
          onSaveManualPaths={handleSaveManualPaths}
          onDeleteCurrentPath={() => {
            if (selectedPathIdForEdit) {
              setManualPaths(manualPaths.filter((p) => p.path_id !== selectedPathIdForEdit));
              setSelectedPathIdForEdit(null);
              setCurrentManualPoints([]);
            }
          }}
          onExitManualPathMode={() => {
            setManualPathMode(false);
            setCurrentManualPoints([]);
            setSelectedPathIdForEdit(null);
          }}
          renderPolygons={renderPolygons}
        />

        {/* 3D/2D Synchronized Simulation Floating Playback Control Bar (Feature 7) */}
        {simulationState && (
          <div className="absolute top-4 left-1/2 -translate-x-1/2 z-40 bg-slate-950/90 backdrop-blur-md border border-sky-500/40 rounded-full px-4 py-1.5 shadow-2xl flex items-center gap-3 text-slate-200 text-xs select-none">
            {/* Play/Pause Button */}
            <button
              onClick={simulationState.isPlaying ? pauseSimulation : resumeSimulation}
              className="p-1.5 rounded-full bg-sky-600 hover:bg-sky-500 text-white shadow transition-all"
              title={simulationState.isPlaying ? 'Pause Simulation' : 'Resume Simulation'}
            >
              {simulationState.isPlaying ? <Pause size={12} /> : <Play size={12} className="fill-white" />}
            </button>

            {/* State Badge */}
            <span
              className={`text-[9px] font-bold font-mono px-1.5 py-0.5 rounded border ${STATE_THEMES[simulationState.activeState].bg
                } ${STATE_THEMES[simulationState.activeState].text} ${STATE_THEMES[simulationState.activeState].border}`}
            >
              {simulationState.activeState.toUpperCase()} SIM
            </span>

            {/* Step & Progress Scrubber */}
            <div className="flex items-center gap-2">
              <input
                type="range"
                min={0}
                max={1}
                step={0.002}
                value={simulationState.progress}
                onChange={(e) => seekSimulation(parseFloat(e.target.value))}
                className="w-32 accent-sky-400 h-1.5 bg-slate-800 rounded cursor-pointer"
              />
              <span className="text-[10px] font-mono text-slate-400 w-16 text-right">
                {Math.round(simulationState.progress * 100)}% ({simulationState.currentStep}/{simulationState.totalSteps})
              </span>
            </div>

            {/* Speed Multiplier Options */}
            <div className="flex items-center gap-1 bg-slate-900 rounded p-0.5 border border-slate-800 text-[9.5px] font-mono">
              {[0.5, 1.0, 2.0, 5.0].map((spd) => (
                <button
                  key={spd}
                  onClick={() => setSimulationSpeed(spd)}
                  className={`px-1.5 py-0.5 rounded transition-all ${simulationState.speed === spd
                      ? 'bg-sky-600 text-white font-bold'
                      : 'text-slate-400 hover:text-slate-200'
                    }`}
                >
                  {spd}x
                </button>
              ))}
            </div>

            {/* Close / Stop Button */}
            <button
              onClick={stopSimulation}
              className="p-1 text-slate-400 hover:text-rose-400 transition-colors ml-1"
              title="Stop Simulation"
            >
              <X size={14} />
            </button>
          </div>
        )}

        {/* Right Side Panel (w-[320px]): Top Action Toolbar + File List / Diagnostics Dashboard */}
        <div className="w-[320px] shrink-0 border-l border-slate-800 bg-slate-950/40 flex flex-col h-full min-h-0 overflow-hidden">
          {/* Top Horizontal Action Toolbar */}
          <InteractiveActionColumn
            hasImage={hasImage}
            activeTemplate={activeTemplate}
            isCapturing={isCapturing}
            isReconstructing={isReconstructing}
            segMode={segMode}
            manualPathMode={manualPathMode}
            showDiagnostics={showDiagnostics}
            onTriggerCapture={handleTriggerCapture}
            onToggleSegMode={handleToggleSegMode}
            onToggleManualPathMode={() => {
              if (manualPathMode) {
                setCurrentManualPoints([]);
                setSelectedPathIdForEdit(null);
              } else {
                if (activeTemplate && !sessionData) {
                  loadSessionData(activeTemplate);
                }
              }
              setManualPathMode(!manualPathMode);
            }}
            onToggleDiagnostics={() => setShowDiagnostics(!showDiagnostics)}
            onTriggerReconstruct={handleTriggerReconstruct}
          />

          {/* Panel Content (File List or TCP Diagnostics) */}
          <div className="flex-1 min-h-0 overflow-hidden flex flex-col">
            {!showDiagnostics ? (
              <TemplateFileList
                files={files}
                onOpenDeleteFileModal={(f) => {
                  if (!activeTemplate) return;
                  setModalConfig({
                    isOpen: true,
                    title: 'Delete File',
                    message: `Delete file '${f}' from template '${activeTemplate}'?`,
                    type: 'confirm',
                    onConfirm: async () => {
                      try {
                        const res = await fetch(`${API_BASE}/api/interactive/templates/${activeTemplate}/files/${f}`, {
                          method: 'DELETE',
                        });
                        if (!res.ok) throw new Error('Failed to delete file');
                        await handleSelectTemplate(activeTemplate);
                      } catch (err: any) {
                        alert(err.message);
                      }
                    },
                  });
                }}
                rawPaths={rawPaths}
                optPaths={optPaths}
                poiPaths={poiPaths}
                robotConnected={robotConnected}
                onSimulatePath={(st, pId) => startSimulation(st, pId)}
                onExecutePath={(fileName, st, pId) => handleDirectExecutePath(fileName, st, pId)}
                onOpenDiagnostics={(st) => {
                  handleSelectActiveState(st);
                  setShowDiagnostics(true);
                }}
                onSelectFile={(fileName) => {
                  if (fileName.includes('poi.path') || fileName.includes('poi.report')) {
                    handleSelectActiveState('poi');
                  } else if (fileName.includes('opt.path') || fileName.includes('opt.report')) {
                    handleSelectActiveState('opt');
                  } else if (fileName.includes('raw.path') || fileName.includes('raw.report')) {
                    handleSelectActiveState('raw');
                  }
                }}
              />
            ) : (
              <DiagnosticsDashboard
                activeState={activeState}
                rawReport={rawReport}
                optReport={optReport}
                poiReport={poiReport}
                rawPaths={rawPaths}
                optPaths={optPaths}
                poiPaths={poiPaths}
                isVerifying={isVerifying}
                isOptimizing={isOptimizing}
                activeTemplate={activeTemplate}
                kinParams={kinParams}
                poiConfig={poiConfig}
                urdfTcpInfo={urdfTcpInfo}
                isKinParamsOpen={isKinParamsOpen}
                highlightedPathId={highlightedPathId}
                setKinParams={setKinParams}
                setPoiConfig={setPoiConfig}
                setIsKinParamsOpen={setIsKinParamsOpen}
                setHighlightedPathId={setHighlightedPathId}
                onSelectActiveState={handleSelectActiveState}
                onRunDiagnostics={handleRunDiagnostics}
                onApplyOptimization={handleApplyOptimization}
                onFetchAnchorPose={handleFetchAnchorPose}
                onClose={() => setShowDiagnostics(false)}
              />
            )}
          </div>
        </div>
      </div>

      {/* Global Dialog Modal */}
      {modalConfig && <CustomModal config={modalConfig} onClose={() => setModalConfig(null)} />}

    </div>
  );
};

export default InteractiveOp;
export * from './interactive/types';
