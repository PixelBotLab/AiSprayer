import React, { useState, useEffect, useRef, type MouseEvent, type WheelEvent } from 'react';

import { 
  Camera, 
  FolderPlus, 
  Trash2, 
  Image as ImageIcon, 
  ChevronLeft, 
  ChevronRight, 
  FileJson, 
  Layers, 
  Grip, 
  Undo2, 
  Check, 
  X, 
  Save, 
  RefreshCw,
  Eye,
  EyeOff,
  Sparkles,
  FileCode2,
  HardDrive,
  ZoomIn,
  ZoomOut,
  Box,
  Route,
  Plus,
  Minus,
  ShieldCheck
} from 'lucide-react';



import { CustomModal, type ModalConfig } from '../common/CustomModal';


interface FileItem {
  name: string;
  size: number;
  ctime: number;
}

interface Point {
  x: number;
  y: number;
  label: number; // 1 for fg, 0 for bg
}

interface MaskData {
  id?: number;
  points?: Point[];
  polygons: number[][][]; // Array of polygons, each is Array of [x, y]
  score?: number;
}

export interface WaypointItem {
  index: number;
  pixel: [number, number]; // [u, v]
  surface_point_cam_mm: [number, number, number];
  surface_point_base_mm: [number, number, number];
  surface_normal_base: [number, number, number];
  surface_normal_cam: [number, number, number];
  standoff_distance_mm: number;
  tcp_pose_base: {
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
  };
  normal_2d_proj: [number, number]; // [dx, dy]
}

export interface ManualPathItem {
  path_id: number;
  name: string;
  points: WaypointItem[];
}

const COLORS = [
  { fill: 'rgba(16, 185, 129, 0.38)', stroke: '#10b981' },
  { fill: 'rgba(59, 130, 246, 0.38)', stroke: '#3b82f6' },
  { fill: 'rgba(245, 158, 11, 0.38)', stroke: '#f59e0b' },
  { fill: 'rgba(168, 85, 247, 0.38)', stroke: '#a855f7' },
  { fill: 'rgba(236, 72, 153, 0.38)', stroke: '#ec4899' },
  { fill: 'rgba(6, 182, 212, 0.38)', stroke: '#06b6d4' },
];

interface InteractiveOpProps {
  externalActiveTemplate?: string | null;
  onTemplateChange?: (templateName: string | null) => void;
  onMeshUpdated?: () => void;
  onPathsUpdated?: () => void;
}

// ─── Math helpers for client-side normal computation ───────────────────────
function computeToolEuler(normal_base: [number, number, number]): [number, number, number] {
  // Tool Z points towards surface: -normal_base
  let zx = -normal_base[0], zy = -normal_base[1], zz = -normal_base[2];
  const zl = Math.sqrt(zx*zx+zy*zy+zz*zz);
  if (zl > 1e-6) { zx/=zl; zy/=zl; zz/=zl; }
  // Reference X
  let rx = 0, ry = 0, rz = 1;
  if (Math.abs(zx*rx+zy*ry+zz*rz) > 0.92) { rx=1; ry=0; rz=0; }
  // Y = Z x ref, X = Y x Z
  let yx = zy*rz-zz*ry, yy = zz*rx-zx*rz, yz = zx*ry-zy*rx;
  const yl = Math.sqrt(yx*yx+yy*yy+yz*yz);
  if (yl > 1e-6) { yx/=yl; yy/=yl; yz/=yl; }
  let xx = yy*zz-yz*zy, xy = yz*zx-yx*zz, xz = yx*zy-yy*zx;
  const xl = Math.sqrt(xx*xx+xy*xy+xz*xz);
  if (xl > 1e-6) { xx/=xl; xy/=xl; xz/=xl; }
  // XYZ euler from rotation matrix columns [x_tool, y_tool, z_tool]
  // R = [xx, yx, zx; xy, yy, zy; xz, yz, zz]
  const sy = -xz;
  const cy_v = Math.sqrt(xx*xx + xy*xy);
  const rx_e = cy_v > 1e-6 ? Math.atan2(yz, zz) : Math.atan2(-zy, yy);
  const ry_e = Math.atan2(sy, cy_v);
  const rz_e = cy_v > 1e-6 ? Math.atan2(xy, xx) : 0;
  const toDeg = 180 / Math.PI;
  return [+(rx_e*toDeg).toFixed(2), +(ry_e*toDeg).toFixed(2), +(rz_e*toDeg).toFixed(2)];
}
// ───────────────────────────────────────────────────────────────────────────

const InteractiveOp: React.FC<InteractiveOpProps> = ({
  externalActiveTemplate,
  onTemplateChange,
  onMeshUpdated,
  onPathsUpdated
}) => {
  const [templates, setTemplates] = useState<string[]>([]);
  const [activeTemplate, setActiveTemplate] = useState<string | null>(externalActiveTemplate || null);
  const [files, setFiles] = useState<FileItem[]>([]);
  const [isCapturing, setIsCapturing] = useState(false);
  const [isReconstructing, setIsReconstructing] = useState(false);
  const [imageVersion, setImageVersion] = useState(Date.now());
  const scrollRef = useRef<HTMLDivElement>(null);

  // Zoom and Pan State
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [isPanning, setIsPanning] = useState(false);
  const [isSpacePressed, setIsSpacePressed] = useState(false);
  const panStartRef = useRef({ startX: 0, startY: 0, initialX: 0, initialY: 0 });
  const containerRef = useRef<HTMLDivElement>(null);

  // Space key listener for Figma/CAD style canvas panning
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.code === 'Space' && !['INPUT', 'TEXTAREA'].includes((e.target as HTMLElement)?.tagName)) {
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
  }, []);

  // Global Mouse Move and Up for butter-smooth dragging anywhere on screen
  useEffect(() => {
    const handleGlobalMouseMove = (e: globalThis.MouseEvent) => {
      if (isPanning) {
        const dx = e.clientX - panStartRef.current.startX;
        const dy = e.clientY - panStartRef.current.startY;
        setPan({
          x: panStartRef.current.initialX + dx,
          y: panStartRef.current.initialY + dy
        });
      }
    };

    const handleGlobalMouseUp = () => {
      setIsPanning(false);
    };

    if (isPanning) {
      window.addEventListener('mousemove', handleGlobalMouseMove);
      window.addEventListener('mouseup', handleGlobalMouseUp);
    }
    return () => {
      window.removeEventListener('mousemove', handleGlobalMouseMove);
      window.removeEventListener('mouseup', handleGlobalMouseUp);
    };
  }, [isPanning]);

  const handleZoom = (delta: number) => {
    setZoom(prev => {
      const next = Math.min(Math.max(0.5, Number((prev + delta).toFixed(2))), 5.0);
      if (next === 1) setPan({ x: 0, y: 0 });
      return next;
    });
  };

  const resetZoom = () => {
    setZoom(1);
    setPan({ x: 0, y: 0 });
  };

  const handleWheel = (e: WheelEvent) => {
    if (e.ctrlKey || e.metaKey || !segMode) {
      e.preventDefault();
      const zoomFactor = e.deltaY < 0 ? 0.15 : -0.15;
      handleZoom(zoomFactor);
    }
  };

  const handleMouseDown = (e: MouseEvent) => {
    // In view mode (!segMode), Left Click (0) or Middle Click (1) drags the canvas!
    // In segMode, Middle Click (1) or Space/Alt key drags the canvas!
    const canPan = !segMode 
      ? (e.button === 0 || e.button === 1) 
      : (e.button === 1 || isSpacePressed || e.altKey);

    if (canPan) {
      e.preventDefault();
      setIsPanning(true);
      panStartRef.current = {
        startX: e.clientX,
        startY: e.clientY,
        initialX: pan.x,
        initialY: pan.y
      };
    }
  };

  // Segmentation & Overlay State
  const [segMode, setSegMode] = useState(false);
  const [isInitializingSam, setIsInitializingSam] = useState(false);
  const [natSize, setNatSize] = useState<{w: number, h: number} | null>(null);
  const [currentPoints, setCurrentPoints] = useState<Point[]>([]);
  const [currentPolygons, setCurrentPolygons] = useState<number[][][]>([]);
  const [committedMasks, setCommittedMasks] = useState<MaskData[]>([]);
  const [savedMasks, setSavedMasks] = useState<MaskData[]>([]);
  const [showMasksOverlay, setShowMasksOverlay] = useState(false);

  // Manual TCP Path Design State
  const [manualPathMode, setManualPathMode] = useState(false);
  const [standoffDist, setStandoffDist] = useState<number>(150);
  const [manualPaths, setManualPaths] = useState<ManualPathItem[]>([]);
  const [rawManualPaths, setRawManualPaths] = useState<ManualPathItem[]>([]);
  const [optManualPaths, setOptManualPaths] = useState<ManualPathItem[]>([]);
  const [currentManualPoints, setCurrentManualPoints] = useState<WaypointItem[]>([]);
  const [selectedPathIdForEdit, setSelectedPathIdForEdit] = useState<number | null>(null);

  const [showManualPathsOverlay, setShowManualPathsOverlay] = useState(true);
  const [isSamplingPoint, setIsSamplingPoint] = useState(false);
  const [isLoadingSession, setIsLoadingSession] = useState(false);
  const [mousePixel, setMousePixel] = useState<{ u: number, v: number } | null>(null);
  const [hoveredWaypoint, setHoveredWaypoint] = useState<WaypointItem | null>(null);
  const [liveNormal, setLiveNormal] = useState<{ dx: number; dy: number; tcpPose?: any; surfPoint?: any } | null>(null);

  // Palette for distinguishing multiple paths
  const PATH_PALETTE = ['#3b82f6', '#f43f5e', '#f59e0b', '#10b981', '#8b5cf6', '#06b6d4'];

  // View Tabs & TCP Optimization State
  const [activeViewTab, setActiveViewTab] = useState<'2d' | 'diagnostics'>('2d');
  const [showParamsAccordion, setShowParamsAccordion] = useState(false);
  const [isVerifyingPaths, setIsVerifyingPaths] = useState(false);
  const [isOptimizingPaths, setIsOptimizingPaths] = useState(false);
  const [verificationReport, setVerificationReport] = useState<any | null>(null);
  const [rawReportCache, setRawReportCache] = useState<any | null>(null);
  const [optReportCache, setOptReportCache] = useState<any | null>(null);
  const [verifiedPathTab, setVerifiedPathTab] = useState<'raw' | 'opt'>('raw');
  const [displayPathSource, setDisplayPathSource] = useState<'raw' | 'opt'>('raw');
  const [highlightedPathId, setHighlightedPathId] = useState<number | null>(null);

  const handleDeletePath = (pathIdToDelete: number) => {
    if (selectedPathIdForEdit === pathIdToDelete) {
      setSelectedPathIdForEdit(null);
      setCurrentManualPoints([]);
    }
    const updated = manualPaths.filter(p => p.path_id !== pathIdToDelete);
    const reindexed = updated.map((p, idx) => ({
      ...p,
      path_id: idx + 1,
      name: `Manual_Path_${idx + 1}`
    }));
    setManualPaths(reindexed);
    setRawManualPaths(reindexed);
    setOptManualPaths([]);
    setVerificationReport(null);
    setRawReportCache(null);
    setOptReportCache(null);
    if (activeTemplate) {
      localStorage.removeItem(`tcp_verification_${activeTemplate}`);
    }
  };



  // Configurable Kinematic & Tool Parameters
  const [kinParams, setKinParams] = useState({
    tcpOffsetX: 50.0,
    tcpOffsetY: 0.0,
    tcpOffsetZ: 0.0,
    tcpOffsetRx: 0.0,
    tcpOffsetRy: 90.0,
    tcpOffsetRz: 0.0,
    stepSizeMm: 1.5,
    linearSpeedMmS: 120.0,
  });



  // Client-side session cache — loaded once when entering manualPathMode
  interface SessionData {
    width: number;
    height: number;
    depth: Float32Array;  // row-major, mm
    fx: number; fy: number; cx: number; cy: number;
    T: number[];  // 4x4 row-major, translation in mm
    calib_source: string;
  }
  const sessionRef = useRef<SessionData | null>(null);
  const sessionLoadedForRef = useRef<string | null>(null); // template name

  // ─── Pure JS client-side normal computation (100% matches verify_tab.py / geometry_utils.py) ───
  const getRobustDepth = (depth: Float32Array, width: number, height: number, u: number, v: number, maxR: number = 3): number => {
    u = Math.round(u); v = Math.round(v);
    if (u < 0 || u >= width || v < 0 || v >= height) return 0;
    const z0 = depth[v * width + u];
    if (z0 > 100 && z0 < 3000) return z0;
    for (let r = 1; r <= maxR; r++) {
      const valid: number[] = [];
      for (let du = -r; du <= r; du++) {
        for (let dv = -r; dv <= r; dv++) {
          if (Math.abs(du) === r || Math.abs(dv) === r) {
            const nu = u + du, nv = v + dv;
            if (nu >= 0 && nu < width && nv >= 0 && nv < height) {
              const val = depth[nv * width + nu];
              if (val > 100 && val < 3000) valid.push(val);
            }
          }
        }
      }
      if (valid.length > 0) {
        valid.sort((a, b) => a - b);
        return valid[Math.floor(valid.length / 2)];
      }
    }
    return 0;
  };

  const computeNormalClientSide = (u: number, v: number, standoffMm: number): {
    normal_cam: [number, number, number];
    normal_base: [number, number, number];
    surf_cam: [number, number, number];
    surf_base: [number, number, number];
    tcp_base: [number, number, number];
    euler_deg: [number, number, number];
    proj_dx: number;
    proj_dy: number;
  } | null => {
    const s = sessionRef.current;
    if (!s) return null;
    const { width, height, depth, fx, fy, cx, cy, T } = s;
    u = Math.max(0, Math.min(width - 1, Math.round(u)));
    v = Math.max(0, Math.min(height - 1, Math.round(v)));

    const center_z = getRobustDepth(depth, width, height, u, v, 5);
    if (center_z <= 0) return null;

    const surf_cam: [number, number, number] = [
      (u - cx) * center_z / fx,
      (v - cy) * center_z / fy,
      center_z
    ];

    // Cross-neighborhood sampling (step = 5) matching verify_tab compute_local_normal
    const step = 5;
    const zL = getRobustDepth(depth, width, height, u - step, v, 3);
    const zR = getRobustDepth(depth, width, height, u + step, v, 3);
    const zU = getRobustDepth(depth, width, height, u, v - step, 3);
    const zD = getRobustDepth(depth, width, height, u, v + step, 3);

    let normal_cam: [number, number, number] = [0, 0, -1];
    if (zL > 0 && zR > 0 && zU > 0 && zD > 0) {
      const pL = [(u - step - cx) * zL / fx, (v - cy) * zL / fy, zL];
      const pR = [(u + step - cx) * zR / fx, (v - cy) * zR / fy, zR];
      const pU = [(u - cx) * zU / fx, (v - step - cy) * zU / fy, zU];
      const pD = [(u - cx) * zD / fx, (v + step - cy) * zD / fy, zD];

      const v1 = [pR[0] - pL[0], pR[1] - pL[1], pR[2] - pL[2]];
      const v2 = [pD[0] - pU[0], pD[1] - pU[1], pD[2] - pU[2]];

      // n = v1 x v2
      let nx = v1[1] * v2[2] - v1[2] * v2[1];
      let ny = v1[2] * v2[0] - v1[0] * v2[2];
      let nz = v1[0] * v2[1] - v1[1] * v2[0];

      let len = Math.sqrt(nx * nx + ny * ny + nz * nz);
      if (len > 1e-6) {
        nx /= len; ny /= len; nz /= len;
        // Normal must point towards camera (Z < 0)
        if (nz > 0) { nx = -nx; ny = -ny; nz = -nz; }
        normal_cam = [nx, ny, nz];
      }
    }

    // Transform to base frame using T_base_camera (4x4 row-major, translation in mm)
    const R = [
      [T[0], T[1], T[2]], [T[4], T[5], T[6]], [T[8], T[9], T[10]]
    ];
    const t = [T[3], T[7], T[11]];
    const applyR = (v3: number[]) => [
      R[0][0] * v3[0] + R[0][1] * v3[1] + R[0][2] * v3[2],
      R[1][0] * v3[0] + R[1][1] * v3[1] + R[1][2] * v3[2],
      R[2][0] * v3[0] + R[2][1] * v3[1] + R[2][2] * v3[2]
    ];
    const surf_base_arr = applyR(surf_cam).map((val, i) => val + t[i]);
    const surf_base: [number, number, number] = [surf_base_arr[0], surf_base_arr[1], surf_base_arr[2]];
    const nb_arr = applyR(normal_cam);
    const nb_len = Math.sqrt(nb_arr[0] ** 2 + nb_arr[1] ** 2 + nb_arr[2] ** 2);
    const normal_base: [number, number, number] = nb_len > 1e-6
      ? [nb_arr[0] / nb_len, nb_arr[1] / nb_len, nb_arr[2] / nb_len]
      : [0, 0, 1];

    // TCP in base frame (mm): P_tcp = P_surf + standoff * N_base
    const tcp_base: [number, number, number] = [
      surf_base[0] + standoffMm * normal_base[0],
      surf_base[1] + standoffMm * normal_base[1],
      surf_base[2] + standoffMm * normal_base[2]
    ];

    // Euler angles
    const euler_deg = computeToolEuler(normal_base);

    // True Perspective 2D Projection (matches verify_tab draw_point_normal without forced fixed length):
    // p_cam_normal_tip = p_cam_origin + n_cam * standoffMm
    const p_cam_normal_tip = [
      surf_cam[0] + normal_cam[0] * standoffMm,
      surf_cam[1] + normal_cam[1] * standoffMm,
      surf_cam[2] + normal_cam[2] * standoffMm
    ];

    let proj_dx = 0, proj_dy = 0;
    if (p_cam_normal_tip[2] > 50) {
      const u_tip = fx * p_cam_normal_tip[0] / p_cam_normal_tip[2] + cx;
      const v_tip = fy * p_cam_normal_tip[1] / p_cam_normal_tip[2] + cy;
      proj_dx = u_tip - u;
      proj_dy = v_tip - v;
    }

    return { normal_cam, normal_base, surf_cam, surf_base, tcp_base, euler_deg, proj_dx, proj_dy };
  };

  // Load session data once when entering manualPathMode
  const loadSessionData = async (templateName: string) => {
    if (sessionLoadedForRef.current === templateName) return;
    setIsLoadingSession(true);
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${templateName}/session_data`);
      if (!res.ok) throw new Error('Session data load failed');
      const data = await res.json();
      const b64 = data.depth_flat_b64 as string;
      const binary = atob(b64);
      const bytes = new Uint8Array(binary.length);
      for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
      const depthF32 = new Float32Array(bytes.buffer);
      sessionRef.current = {
        width: data.width,
        height: data.height,
        depth: depthF32,
        fx: data.intrinsics.fx,
        fy: data.intrinsics.fy,
        cx: data.intrinsics.cx,
        cy: data.intrinsics.cy,
        T: data.T_base_camera,
        calib_source: data.calib_source
      };
      sessionLoadedForRef.current = templateName;
    } catch (err) {
      console.warn('Failed to load session data, falling back to server-side sampling:', err);
    } finally {
      setIsLoadingSession(false);
    }
  };

  const fetchLiveNormal = (u: number, v: number) => {
    const result = computeNormalClientSide(u, v, standoffDist);
    if (result) {
      setLiveNormal({
        dx: result.proj_dx,
        dy: result.proj_dy,
        tcpPose: {
          x: +result.tcp_base[0].toFixed(2), y: +result.tcp_base[1].toFixed(2), z: +result.tcp_base[2].toFixed(2),
          rx: +result.euler_deg[0].toFixed(2), ry: +result.euler_deg[1].toFixed(2), rz: +result.euler_deg[2].toFixed(2)
        },
        surfPoint: result.surf_base
      });
    }
  };

  // Custom Modal State
  const [modalConfig, setModalConfig] = useState<ModalConfig>({
    isOpen: false,
    type: 'info',
    title: '',
    message: ''
  });

  const showAlert = (title: string, message: string) => {
    setModalConfig({
      isOpen: true,
      type: 'alert',
      title,
      message,
      confirmText: 'Understood'
    });
  };

  const selectTemplate = (templateName: string) => {
    setActiveTemplate(templateName);
    if (onTemplateChange) {
      onTemplateChange(templateName);
    }
  };

  useEffect(() => {
    if (externalActiveTemplate && externalActiveTemplate !== activeTemplate) {
      setActiveTemplate(externalActiveTemplate);
    }
  }, [externalActiveTemplate]);

  const fetchTemplates = async () => {
    try {
      const res = await fetch('http://localhost:8000/api/interactive/templates');
      if (res.ok) {
        const data = await res.json();
        setTemplates(data.templates || []);
        if (data.templates?.length > 0 && !activeTemplate) {
          selectTemplate(data.templates[0]);
        }
      }
    } catch (err) {
      console.error('Failed to fetch templates:', err);
    }
  };

  useEffect(() => {
    fetchTemplates();
  }, []);

  const fetchSavedMasks = async (templateName: string) => {
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${templateName}/masks`);
      if (res.ok) {
        const data = await res.json();
        setSavedMasks(data.masks || []);
      } else {
        setSavedMasks([]);
      }
    } catch (err) {
      console.error('Failed to fetch saved masks:', err);
      setSavedMasks([]);
    }
  };

  const fetchManualPaths = async (templateName: string, useOpt: boolean = false) => {
    try {
      // 1. Fetch raw paths
      const rawRes = await fetch(`http://localhost:8000/api/interactive/templates/${templateName}/manual_paths?use_opt=false`);
      let loadedRawPaths: ManualPathItem[] = [];
      if (rawRes.ok) {
        const data = await rawRes.json();
        loadedRawPaths = data.paths || [];
        setRawManualPaths(loadedRawPaths);
        if (data.standoff_distance_mm) {
          setStandoffDist(Number(data.standoff_distance_mm));
        }
      } else {
        setRawManualPaths([]);
      }

      // 2. Fetch opt paths
      const optRes = await fetch(`http://localhost:8000/api/interactive/templates/${templateName}/manual_paths?use_opt=true`);
      let loadedOptPaths: ManualPathItem[] = [];
      if (optRes.ok) {
        const optData = await optRes.json();
        if (optData.loaded_from === 'scan.manual_opt_paths.yaml') {
          loadedOptPaths = optData.paths || [];
          setOptManualPaths(loadedOptPaths);
        } else {
          setOptManualPaths([]);
        }
      } else {
        setOptManualPaths([]);
      }

      // 3. Set current active paths
      if (useOpt && loadedOptPaths.length > 0) {
        setManualPaths(loadedOptPaths);
        setDisplayPathSource('opt');
      } else {
        setManualPaths(loadedRawPaths);
        setDisplayPathSource('raw');
      }

      // Eagerly fetch both raw and opt diagnostic reports from backend disk
      try {
        const [rawRepRes, optRepRes] = await Promise.all([
          fetch(`http://localhost:8000/api/interactive/templates/${templateName}/verification_report?use_opt=false`),
          fetch(`http://localhost:8000/api/interactive/templates/${templateName}/verification_report?use_opt=true`)
        ]);
        let rCache = null;
        let oCache = null;
        if (rawRepRes.ok) {
          rCache = await rawRepRes.json();
          setRawReportCache(rCache);
        }
        if (optRepRes.ok) {
          oCache = await optRepRes.json();
          setOptReportCache(oCache);
        }
        
        if (verifiedPathTab === 'opt' && oCache) {
          setVerificationReport(oCache);
        } else if (verifiedPathTab === 'raw' && rCache) {
          setVerificationReport(rCache);
        } else if (rCache) {
          setVerificationReport(rCache);
        } else if (oCache) {
          setVerificationReport(oCache);
        }
      } catch (e) {
        console.warn('Could not eager load reports:', e);
      }

    } catch (err) {
      console.error('Failed to fetch manual paths:', err);
      setManualPaths([]);
      setRawManualPaths([]);
      setOptManualPaths([]);
    }
  };



  const fetchFiles = async (templateName: string) => {
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${templateName}/files`);
      if (res.ok) {
        const data = await res.json();
        // Ensure sorted by ctime descending (newest first)
        const sorted = (data.files || []).sort((a: FileItem, b: FileItem) => b.ctime - a.ctime);
        setFiles(sorted);
        setImageVersion(Date.now());
        
        // Load existing masks from masks.yaml and manual paths from scan.manual_paths.yaml
        fetchSavedMasks(templateName);
        fetchManualPaths(templateName);
      }

    } catch (err) {
      console.error('Failed to fetch template files:', err);
    }
  };


  useEffect(() => {
    if (activeTemplate) {
      fetchFiles(activeTemplate);
      // Reset seg mode, manual path mode and view transforms
      setSegMode(false);
      setManualPathMode(false);
      setCurrentPoints([]);
      setCurrentPolygons([]);
      setCommittedMasks([]);
      setCurrentManualPoints([]);
      setZoom(1);
      setPan({ x: 0, y: 0 });
    } else {
      setFiles([]);
      setSavedMasks([]);
      setManualPaths([]);
      setCurrentManualPoints([]);
    }
  }, [activeTemplate]);

  const handleCreateTemplate = () => {
    const defaultName = new Date().toISOString().replace(/T/, '_').replace(/:/g, '').split('.')[0];
    setModalConfig({
      isOpen: true,
      type: 'input',
      title: 'Create New Template',
      message: 'Enter a unique name or timestamp identifier for the template group:',
      placeholder: 'e.g. 2026-08-14_230000',
      defaultValue: defaultName,
      confirmText: 'Create Template',
      onConfirm: async (name) => {
        if (!name) return;
        try {
          const res = await fetch('http://localhost:8000/api/interactive/templates', { 
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name })
          });
          if (res.ok) {
            await fetchTemplates();
            selectTemplate(name);
          } else {
            const err = await res.json();
            showAlert('Creation Failed', err.detail || 'Failed to create template directory.');
          }
        } catch (err: any) {
          showAlert('Network Error', err.message);
        }
      }
    });
  };

  const handleCapture = async () => {
    if (!activeTemplate || isCapturing) return;
    setIsCapturing(true);
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/capture`, { method: 'POST' });
      if (res.ok) {
        fetchFiles(activeTemplate);
      } else {
        const err = await res.json();
        showAlert('Capture Failed', err.detail || 'Failed to capture camera frames.');
      }
    } catch (err: any) {
      showAlert('Capture Error', err.message);
    } finally {
      setIsCapturing(false);
    }
  };

  const handleReconstruct = async () => {
    if (!activeTemplate || isReconstructing) return;
    setIsReconstructing(true);
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/reconstruct`, { method: 'POST' });
      if (res.ok) {
        const data = await res.json();
        fetchFiles(activeTemplate);
        if (onMeshUpdated) {
          onMeshUpdated();
        }
        showAlert(
          'Reconstruction Completed', 
          `Generated surface mesh: ${data.vertices} vertices, ${data.faces} faces.\nSource: ${data.calibration_source}`
        );
      } else {
        const err = await res.json();
        showAlert('Reconstruction Failed', err.detail || 'Failed to reconstruct 3D surface mesh.');
      }
    } catch (err: any) {
      showAlert('Reconstruction Error', err.message);
    } finally {
      setIsReconstructing(false);
    }
  };

  // Save verification report to localStorage
  useEffect(() => {
    if (verificationReport && activeTemplate) {
      try {
        localStorage.setItem(`tcp_verification_${activeTemplate}`, JSON.stringify(verificationReport));
      } catch {}
    }
  }, [verificationReport, activeTemplate]);

  // Sync state to ConsoleLogZone TCP Diagnostics tab
  useEffect(() => {
    window.dispatchEvent(new CustomEvent('tcp_diagnostics_sync', {
      detail: {
        activeTemplate,
        verificationReport,
        verifiedPathTab,
        isVerifying: isVerifyingPaths,
        isOptimizing: isOptimizingPaths,
        kinParams
      }
    }));
  }, [activeTemplate, verificationReport, verifiedPathTab, isVerifyingPaths, isOptimizingPaths, kinParams]);

  // Listen to requests from ConsoleLogZone
  useEffect(() => {
    const onReqVerify = (e: any) => {
      if (e.detail?.kinParams) setKinParams(e.detail.kinParams);
      handleVerifyPaths(e.detail?.useOpt || false);
    };
    const onReqOpt = (e: any) => {
      if (e.detail?.kinParams) setKinParams(e.detail.kinParams);
      handleOptimizePaths();
    };
    const onReqTab = (e: any) => {
      if (e.detail?.tab) switchVerifiedTab(e.detail.tab);
    };

    window.addEventListener('request_tcp_verify', onReqVerify);
    window.addEventListener('request_tcp_optimize', onReqOpt);
    window.addEventListener('request_tcp_tab_switch', onReqTab);

    return () => {
      window.removeEventListener('request_tcp_verify', onReqVerify);
      window.removeEventListener('request_tcp_optimize', onReqOpt);
      window.removeEventListener('request_tcp_tab_switch', onReqTab);
    };
  }, [activeTemplate, isVerifyingPaths, isOptimizingPaths, kinParams, rawReportCache, optReportCache]);

  const handleVerifyPaths = async (useOpt: boolean = false, silent: boolean = false) => {
    if (!activeTemplate || isVerifyingPaths) return;
    setIsVerifyingPaths(true);
    try {
      const payload = {
        use_opt: useOpt,
        options: {
          step_size_mm: Number(kinParams.stepSizeMm),
          linear_velocity_mm_s: Number(kinParams.linearSpeedMmS),
          tcp_offset_xyz_mm: [Number(kinParams.tcpOffsetX), Number(kinParams.tcpOffsetY), Number(kinParams.tcpOffsetZ)],
          tcp_offset_rpy_deg: [Number(kinParams.tcpOffsetRx), Number(kinParams.tcpOffsetRy), Number(kinParams.tcpOffsetRz)]
        }
      };
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/verify_paths`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      if (res.ok) {
        const data = await res.json();
        setVerificationReport(data);
        if (useOpt) {
          setOptReportCache(data);
        } else {
          setRawReportCache(data);
        }
        setVerifiedPathTab(useOpt ? 'opt' : 'raw');

      } else {
        const err = await res.json();
        if (!silent) {
          showAlert('Verification Failed', err.detail || 'Failed to verify TCP paths.');
        }
      }
    } catch (err: any) {
      if (!silent) {
        showAlert('Verification Error', err.message);
      }
    } finally {
      setIsVerifyingPaths(false);
    }
  };

  const switchVerifiedTab = async (tab: 'raw' | 'opt') => {
    setVerifiedPathTab(tab);
    setDisplayPathSource(tab);

    if (tab === 'opt') {
      if (optManualPaths.length > 0) {
        setManualPaths(optManualPaths);
      }
      if (optReportCache) {
        setVerificationReport(optReportCache);
      } else if (activeTemplate) {
        try {
          const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/verification_report?use_opt=true`);
          if (res.ok) {
            const data = await res.json();
            setOptReportCache(data);
            setVerificationReport(data);
          } else {
            handleOptimizePaths();
          }
        } catch {
          handleOptimizePaths();
        }
      }
    } else {
      if (rawManualPaths.length > 0) {
        setManualPaths(rawManualPaths);
      }
      if (rawReportCache) {
        setVerificationReport(rawReportCache);
      } else if (activeTemplate) {
        try {
          const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/verification_report?use_opt=false`);
          if (res.ok) {
            const data = await res.json();
            setRawReportCache(data);
            setVerificationReport(data);
          } else {
            handleVerifyPaths(false);
          }
        } catch {
          handleVerifyPaths(false);
        }
      }
    }
  };

  const handleOptimizePaths = async () => {
    if (!activeTemplate || isOptimizingPaths) return;
    setIsOptimizingPaths(true);
    try {
      const payload = {
        options: {
          step_size_mm: Number(kinParams.stepSizeMm),
          linear_velocity_mm_s: Number(kinParams.linearSpeedMmS),
          tcp_offset_xyz_mm: [Number(kinParams.tcpOffsetX), Number(kinParams.tcpOffsetY), Number(kinParams.tcpOffsetZ)],
          tcp_offset_rpy_deg: [Number(kinParams.tcpOffsetRx), Number(kinParams.tcpOffsetRy), Number(kinParams.tcpOffsetRz)]
        }
      };
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/optimize_paths`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      if (res.ok) {
        const data = await res.json();
        setOptReportCache(data);
        setVerificationReport(data);
        setVerifiedPathTab('opt');
        setDisplayPathSource('opt');
        await fetchManualPaths(activeTemplate, true);
        await fetchFiles(activeTemplate);
        if (onPathsUpdated) onPathsUpdated();
        showAlert(
          'Optimization Complete', 
          `Generated scan.manual_opt_paths.yaml\nStatus: ${data.summary?.status || 'PASS'}, singularities & overspeed resolved.`
        );
      } else {
        const err = await res.json();
        showAlert('Optimization Failed', err.detail || 'Failed to optimize TCP paths.');
      }
    } catch (err: any) {
      showAlert('Optimization Error', err.message);
    } finally {
      setIsOptimizingPaths(false);
    }
  };


  const scrollTabs = (dir: 'left' | 'right') => {
    if (scrollRef.current) {
      scrollRef.current.scrollBy({ left: dir === 'left' ? -200 : 200, behavior: 'smooth' });
    }
  };

  const hasImage = files.some(f => f.name === 'scan.jpg');
  const hasDepth = files.some(f => f.name === 'scan.depth.npy');
  const hasMasks = files.some(f => f.name === 'scan.masks.yaml');
  const hasManualPaths = files.some(f => f.name === 'scan.manual_paths.yaml');
  
  const imageUrl = hasImage && activeTemplate
    ? `http://localhost:8000/templates/${activeTemplate}/scan.jpg?t=${imageVersion}`
    : null;

  const formatFileSize = (bytes: number) => {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  };

  const formatFileDate = (ctime: number) => {
    const d = new Date(ctime * 1000);
    const month = String(d.getMonth() + 1).padStart(2, '0');
    const day = String(d.getDate()).padStart(2, '0');
    const hours = String(d.getHours()).padStart(2, '0');
    const mins = String(d.getMinutes()).padStart(2, '0');
    const secs = String(d.getSeconds()).padStart(2, '0');
    return `${month}-${day} ${hours}:${mins}:${secs}`;
  };

  const getFileBadge = (filename: string) => {
    if (filename.includes('.ply') || filename.includes('.stl')) return { label: 'MESH', color: 'bg-purple-500/20 text-purple-300 border-purple-500/40' };
    if (filename === 'scan.masks.yaml') return { label: 'MASKS', color: 'bg-emerald-500/20 text-emerald-300 border-emerald-500/40' };
    if (filename === 'scan.manual_opt_paths.yaml') return { label: 'OPT TCP', color: 'bg-teal-500/20 text-teal-300 border-teal-500/40' };
    if (filename === 'scan.manual_paths.yaml' || filename.includes('paths.yaml')) return { label: 'MANUAL TCP', color: 'bg-rose-500/20 text-rose-300 border-rose-500/40' };
    if (filename === 'scan.params.yaml') return { label: 'PARAMS', color: 'bg-amber-500/20 text-amber-300 border-amber-500/40' };
    if (filename === 'scan.pcd') return { label: '3D PCD', color: 'bg-cyan-500/20 text-cyan-300 border-cyan-500/40' };
    if (filename === 'scan.depth.npy') return { label: 'DEPTH', color: 'bg-indigo-500/20 text-indigo-300 border-indigo-500/40' };
    if (filename === 'scan.jpg') return { label: 'IMAGE', color: 'bg-blue-500/20 text-blue-300 border-blue-500/40' };
    return null;
  };

  const getFileIcon = (filename: string) => {
    if (filename.includes('.ply') || filename.includes('.stl')) return <Box size={13} className="text-purple-400" />;
    if (filename.endsWith('.jpg') || filename.endsWith('.png')) return <ImageIcon size={13} className="text-blue-400" />;
    if (filename === 'scan.manual_opt_paths.yaml') return <ShieldCheck size={13} className="text-teal-400" />;
    if (filename === 'scan.manual_paths.yaml' || filename.includes('paths.yaml')) return <Route size={13} className="text-rose-400" />;
    if (filename.endsWith('.yaml') || filename.endsWith('.json')) return <FileJson size={13} className="text-amber-400" />;
    if (filename.endsWith('.pcd')) return <Grip size={13} className="text-cyan-400" />;
    if (filename.endsWith('.npy')) return <Layers size={13} className="text-indigo-400" />;
    return <FileCode2 size={13} className="text-slate-400" />;
  };


  // Grouping files into logical operational stages
  const fileCategories = [
    {
      id: 'capture',
      title: 'Capture',
      icon: Camera,
      iconColor: 'text-blue-400',
      badgeColor: 'bg-blue-500/10 text-blue-400 border-blue-500/30',
      files: files.filter(f => ['scan.jpg', 'scan.depth.npy', 'scan.pcd', 'scan.params.yaml'].includes(f.name))
    },
    {
      id: 'segment',
      title: 'Segment',
      icon: Layers,
      iconColor: 'text-emerald-400',
      badgeColor: 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30',
      files: files.filter(f => ['scan.masks.yaml', 'scan.masks.jpg'].includes(f.name))
    },
    {
      id: 'plan',
      title: 'Manual Path',
      icon: Route,
      iconColor: 'text-rose-400',
      badgeColor: 'bg-rose-500/10 text-rose-400 border-rose-500/30',
      files: files.filter(f => f.name.includes('manual_paths') || f.name.includes('paths'))
    },
    {
      id: 'reconstruct',
      title: 'Reconstruct',
      icon: Box,
      iconColor: 'text-purple-400',
      badgeColor: 'bg-purple-500/10 text-purple-400 border-purple-500/30',
      files: files.filter(f => f.name.startsWith('scan.mesh') || f.name.includes('.ply') || f.name.includes('.stl'))
    },
    {
      id: 'other',
      title: 'Other',
      icon: FileCode2,
      iconColor: 'text-slate-400',
      badgeColor: 'bg-slate-800 text-slate-400 border-slate-700',
      files: files.filter(f => 
        !['scan.jpg', 'scan.depth.npy', 'scan.pcd', 'scan.params.yaml', 'scan.masks.yaml', 'scan.masks.jpg'].includes(f.name) &&
        !f.name.includes('paths') &&
        !f.name.startsWith('scan.mesh') && !f.name.includes('.ply') && !f.name.includes('.stl')
      )
    }
  ];

  // --- Segmentation Logic ---
  
  const toggleSegMode = async () => {
    if (segMode) {
      setSegMode(false);
      return;
    }
    
    if (!activeTemplate || !hasImage) return;
    
    setIsInitializingSam(true);
    setSegMode(true);
    setCurrentPoints([]);
    setCurrentPolygons([]);
    
    // Seed committedMasks with existing saved masks if any
    if (savedMasks.length > 0) {
      setCommittedMasks([...savedMasks]);
    } else {
      setCommittedMasks([]);
    }
    
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/sam/init`, { method: 'POST' });
      if (!res.ok) {
        throw new Error('Failed to initialize MobileSAM session.');
      }
    } catch (e: any) {
      showAlert('MobileSAM Initialization Error', e.message || 'Failed to initialize session. Check server logs.');
      setSegMode(false);
    } finally {
      setIsInitializingSam(false);
    }
  };

  const predictMask = async (pts: Point[]) => {
    if (!activeTemplate || pts.length === 0) {
      setCurrentPolygons([]);
      return;
    }
    
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/sam/predict`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          points: pts.map(p => [p.x, p.y]),
          labels: pts.map(p => p.label)
        })
      });
      if (res.ok) {
        const data = await res.json();
        setCurrentPolygons(data.polygons || []);
      }
    } catch (e) {
      console.error("Predict error:", e);
    }
  };

  const handleImageClick = (e: MouseEvent<SVGSVGElement>, label: number) => {
    if (!segMode || isInitializingSam || !natSize || isPanning) return;
    e.preventDefault();
    
    const rect = e.currentTarget.getBoundingClientRect();
    const x = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const y = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);
    
    const newPts = [...currentPoints, { x, y, label }];
    setCurrentPoints(newPts);
    predictMask(newPts);
  };

  const handleUndo = () => {
    const newPts = currentPoints.slice(0, -1);
    setCurrentPoints(newPts);
    predictMask(newPts);
  };

  const handleCommit = () => {
    if (currentPoints.length === 0 || currentPolygons.length === 0) return;
    setCommittedMasks([...committedMasks, {
      points: [...currentPoints],
      polygons: [...currentPolygons]
    }]);
    setCurrentPoints([]);
    setCurrentPolygons([]);
  };

  const handleResetCurrent = () => {
    setCurrentPoints([]);
    setCurrentPolygons([]);
  };

  const handleClearAll = () => {
    setCommittedMasks([]);
    handleResetCurrent();
  };

  const handleSaveMasks = async () => {
    if (!activeTemplate || committedMasks.length === 0) return;
    
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/sam/save`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          committed_masks: committedMasks.map(m => ({
            points: m.points ? m.points.map(p => [p.x, p.y]) : [],
            labels: m.points ? m.points.map(p => p.label) : []
          }))
        })
      });
      
      if (res.ok) {
        setSegMode(false);
        fetchFiles(activeTemplate);
      } else {
        const err = await res.json();
        showAlert('Save Failed', err.detail || 'Failed to write masks.');
      }
    } catch (e: any) {
      showAlert('Save Error', e.message);
    }
  };

  // --- Manual TCP Path Logic ---

  const toggleManualPathMode = () => {
    if (manualPathMode) {
      setManualPathMode(false);
      setSelectedPathIdForEdit(null);
      setCurrentManualPoints([]);
      return;
    }
    if (!activeTemplate || !hasImage) return;
    if (segMode) setSegMode(false);
    setManualPathMode(true);
    setSelectedPathIdForEdit(null);
    setCurrentManualPoints([]);
    // Pre-load depth + calibration for client-side computation
    loadSessionData(activeTemplate);
  };

  const handleSelectPathForEdit = (pathId: number) => {
    const target = manualPaths.find(p => p.path_id === pathId);
    if (!target) return;
    setSelectedPathIdForEdit(pathId);
    setCurrentManualPoints([...target.points]);
  };

  const handleDeselectEditPath = () => {
    setSelectedPathIdForEdit(null);
    setCurrentManualPoints([]);
  };

  const handleDeleteSingleWaypoint = (wpIdx: number) => {
    setCurrentManualPoints(prev => {
      const updated = prev.filter((_, i) => i !== wpIdx);
      return updated.map((pt, i) => ({
        ...pt,
        index: i + 1
      }));
    });
    setHoveredWaypoint(null);
  };

  const handleManualMouseMove = (e: MouseEvent<SVGSVGElement>) => {
    if (!natSize || isPanning) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const u = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const v = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);
    setMousePixel({ u, v });
    fetchLiveNormal(u, v);
  };

  const handleManualImageClick = (e: MouseEvent<SVGSVGElement>) => {
    if (!manualPathMode || !natSize || isPanning || isSamplingPoint || !activeTemplate) return;
    e.preventDefault();

    const rect = e.currentTarget.getBoundingClientRect();
    const u = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const v = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);

    // Client-side path: instant, no HTTP
    const result = computeNormalClientSide(u, v, standoffDist);
    if (result) {
      const newWaypoint: WaypointItem = {
        index: currentManualPoints.length + 1,
        pixel: [u, v],
        surface_point_cam_mm: result.surf_cam,
        surface_point_base_mm: result.surf_base,
        surface_normal_base: result.normal_base,
        surface_normal_cam: result.normal_cam,
        standoff_distance_mm: standoffDist,
        tcp_pose_base: {
          x: +result.tcp_base[0].toFixed(2),
          y: +result.tcp_base[1].toFixed(2),
          z: +result.tcp_base[2].toFixed(2),
          rx: result.euler_deg[0],
          ry: result.euler_deg[1],
          rz: result.euler_deg[2]
        },
        normal_2d_proj: [result.proj_dx, result.proj_dy]
      };
      setCurrentManualPoints(prev => [...prev, newWaypoint]);
      return;
    }

    // Fallback: server-side sampling if session data not loaded yet
    setIsSamplingPoint(true);
    fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/sample_point`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ u, v, standoff_dist_mm: standoffDist })
    }).then(async res => {
      if (res.ok) {
        const data = await res.json();
        const newWaypoint: WaypointItem = {
          index: currentManualPoints.length + 1,
          pixel: data.pixel,
          surface_point_cam_mm: data.surface_point_cam_mm,
          surface_point_base_mm: data.surface_point_base_mm,
          surface_normal_base: data.surface_normal_base,
          surface_normal_cam: data.surface_normal_cam,
          standoff_distance_mm: data.standoff_distance_mm,
          tcp_pose_base: data.tcp_pose_base,
          normal_2d_proj: data.normal_2d_proj
        };
        setCurrentManualPoints(prev => [...prev, newWaypoint]);
      } else {
        const err = await res.json();
        showAlert('Point Sampling Failed', err.detail || 'Could not sample 3D surface point.');
      }
    }).catch((err: any) => {
      showAlert('Point Sampling Error', err.message);
    }).finally(() => {
      setIsSamplingPoint(false);
    });
  };

  const handleManualUndo = () => {
    setCurrentManualPoints(prev => prev.slice(0, -1));
  };

  const handleManualCommit = () => {
    if (currentManualPoints.length === 0) return;
    if (selectedPathIdForEdit !== null) {
      // Update existing path in place
      setManualPaths(prev => prev.map(p => p.path_id === selectedPathIdForEdit ? {
        ...p,
        points: currentManualPoints.map((pt, idx) => ({ ...pt, index: idx + 1 }))
      } : p));
      setSelectedPathIdForEdit(null);
      setCurrentManualPoints([]);
    } else {
      // Append new path
      const newPath: ManualPathItem = {
        path_id: manualPaths.length + 1,
        name: `Manual_Path_${manualPaths.length + 1}`,
        points: currentManualPoints.map((pt, idx) => ({ ...pt, index: idx + 1 }))
      };
      setManualPaths(prev => [...prev, newPath]);
      setCurrentManualPoints([]);
    }
  };

  const handleManualResetCurrent = () => {
    if (selectedPathIdForEdit !== null) {
      const orig = manualPaths.find(p => p.path_id === selectedPathIdForEdit);
      if (orig) setCurrentManualPoints([...orig.points]);
      else setCurrentManualPoints([]);
    } else {
      setCurrentManualPoints([]);
    }
  };

  const handleManualClearAll = () => {
    setManualPaths([]);
    setSelectedPathIdForEdit(null);
    setCurrentManualPoints([]);
  };

  const handleStandoffChange = (newDist: number) => {
    const val = Math.max(10, Math.min(500, newDist));
    setStandoffDist(val);

    // Dynamically update TCP 6D position for current points
    setCurrentManualPoints(prev => prev.map(p => {
      const nx = p.surface_normal_base[0];
      const ny = p.surface_normal_base[1];
      const nz = p.surface_normal_base[2];
      return {
        ...p,
        standoff_distance_mm: val,
        tcp_pose_base: {
          ...p.tcp_pose_base,
          x: Number((p.surface_point_base_mm[0] + val * nx).toFixed(2)),
          y: Number((p.surface_point_base_mm[1] + val * ny).toFixed(2)),
          z: Number((p.surface_point_base_mm[2] + val * nz).toFixed(2)),
        }
      };
    }));

    // Dynamically update TCP 6D position for committed paths
    setManualPaths(prev => prev.map(path => ({
      ...path,
      points: path.points.map(p => {
        const nx = p.surface_normal_base[0];
        const ny = p.surface_normal_base[1];
        const nz = p.surface_normal_base[2];
        return {
          ...p,
          standoff_distance_mm: val,
          tcp_pose_base: {
            ...p.tcp_pose_base,
            x: Number((p.surface_point_base_mm[0] + val * nx).toFixed(2)),
            y: Number((p.surface_point_base_mm[1] + val * ny).toFixed(2)),
            z: Number((p.surface_point_base_mm[2] + val * nz).toFixed(2)),
          }
        };
      })
    })));
  };

  const handleManualSavePaths = async () => {
    if (!activeTemplate) return;
    let allPaths = [...manualPaths];
    if (selectedPathIdForEdit !== null && currentManualPoints.length > 0) {
      allPaths = allPaths.map(p => p.path_id === selectedPathIdForEdit ? {
        ...p,
        points: currentManualPoints.map((pt, idx) => ({ ...pt, index: idx + 1 }))
      } : p);
    } else if (selectedPathIdForEdit === null && currentManualPoints.length > 0) {
      allPaths.push({
        path_id: allPaths.length + 1,
        name: `Manual_Path_${allPaths.length + 1}`,
        points: currentManualPoints.map((pt, idx) => ({ ...pt, index: idx + 1 }))
      });
    }

    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/manual_paths`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          paths: allPaths,
          standoff_distance_mm: standoffDist
        })
      });

      if (res.ok) {
        setManualPaths(allPaths);
        setSelectedPathIdForEdit(null);
        setCurrentManualPoints([]);
        fetchFiles(activeTemplate);
        if (onPathsUpdated) onPathsUpdated();
        showAlert('Save Successful', `Saved ${allPaths.length} manual TCP path(s) to scan.manual_paths.yaml.`);
      } else {
        const err = await res.json();
        showAlert('Save Failed', err.detail || 'Failed to save manual paths.');
      }
    } catch (e: any) {
      showAlert('Save Error', e.message);
    }
  };

  // Render SVG paths from polygons
  const renderPolygons = (polygons: number[][][], fill: string, stroke?: string) => {
    const pathData = polygons.map(poly => {
      if (poly.length < 3) return '';
      return 'M ' + poly.map(p => `${p[0]},${p[1]}`).join(' L ') + ' Z';
    }).join(' ');
    
    if (!pathData) return null;
    return (
      <path 
        d={pathData} 
        fill={fill} 
        stroke={stroke || 'transparent'} 
        strokeWidth={1.5}
        strokeLinejoin="round"
        fillRule="evenodd" 
      />
    );
  };

  return (
    <div className="w-full h-full flex flex-col bg-slate-900/80 rounded-xl border border-slate-800 shadow-xl overflow-hidden backdrop-blur-sm relative">

      {/* Custom Sleek Modal */}
      <CustomModal config={modalConfig} onClose={() => setModalConfig(prev => ({ ...prev, isOpen: false }))} />






      {/* TOP BAR: Templates */}
      <div className="h-14 shrink-0 border-b border-slate-800 bg-slate-950/50 flex items-center px-3 gap-2">
        <button onClick={() => scrollTabs('left')} className="p-1 hover:bg-slate-800 rounded text-slate-500 hover:text-slate-300">
          <ChevronLeft size={16} />
        </button>
        
        <div ref={scrollRef} className="flex-1 flex items-center overflow-x-hidden gap-2 scroll-smooth">
          {templates.map(template => (
            <button
              key={template}
              onClick={() => selectTemplate(template)}
              className={`px-3.5 h-8 shrink-0 rounded-md text-xs font-medium border transition-colors ${
                activeTemplate === template
                  ? 'bg-blue-600/20 text-blue-400 border-blue-500/50 shadow-[0_0_8px_rgba(59,130,246,0.3)]'
                  : 'bg-slate-800 border-slate-700 text-slate-400 hover:text-slate-200 hover:bg-slate-700'
              }`}
            >
              {template}
            </button>
          ))}
        </div>

        <button onClick={() => scrollTabs('right')} className="p-1 hover:bg-slate-800 rounded text-slate-500 hover:text-slate-300">
          <ChevronRight size={16} />
        </button>
        
        <div className="w-px h-6 bg-slate-700 mx-1 shrink-0" />
        <button
          onClick={handleCreateTemplate}
          className="px-3 h-8 shrink-0 rounded-md text-xs font-medium border border-emerald-500/30 bg-emerald-600/10 text-emerald-400 hover:bg-emerald-600/20 transition-colors flex items-center gap-1.5"
        >
          <FolderPlus size={14} /> New
        </button>
      </div>

      {/* VIEW SELECTOR TABS BAR */}
      <div className="h-10 shrink-0 border-b border-slate-800 bg-slate-950/80 flex items-center justify-between px-3">
        <div className="flex items-center gap-1 bg-slate-900 p-0.5 rounded-lg border border-slate-800">
          <button
            onClick={() => setActiveViewTab('2d')}
            className={`px-3 py-1 text-xs font-medium rounded-md transition-all flex items-center gap-1.5 ${
              activeViewTab === '2d'
                ? 'bg-slate-800 text-cyan-300 shadow border border-cyan-500/30'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <ImageIcon size={13} />
            <span>2D Canvas & Paths</span>
          </button>
          <button
            onClick={() => {
              setActiveViewTab('diagnostics');
              if (!verificationReport) {
                handleVerifyPaths(displayPathSource === 'opt');
              }
            }}
            className={`px-3 py-1 text-xs font-medium rounded-md transition-all flex items-center gap-1.5 ${
              activeViewTab === 'diagnostics'
                ? 'bg-emerald-950 text-emerald-300 shadow border border-emerald-500/40 font-semibold'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <ShieldCheck size={13} />
            <span>TCP Diagnostics & Opt</span>
            {verificationReport && (
              <span className={`text-[9px] px-1.5 py-0.2 rounded-full font-bold uppercase ${
                verificationReport.summary?.status === 'PASS'
                  ? 'bg-emerald-900 text-emerald-300'
                  : verificationReport.summary?.status === 'WARNING'
                  ? 'bg-amber-900 text-amber-300'
                  : 'bg-rose-900 text-rose-300'
              }`}>
                {verificationReport.summary?.status}
              </span>
            )}
          </button>
        </div>

        {/* View Header Quick Actions */}
        <div className="flex items-center gap-2 text-xs">
          {activeViewTab === '2d' ? (
            <div className="flex items-center gap-1 bg-slate-900 p-0.5 rounded-lg border border-slate-800">
              <button
                onClick={() => switchVerifiedTab('raw')}
                className={`px-2.5 py-0.5 text-[11px] rounded transition-all ${
                  displayPathSource === 'raw'
                    ? 'bg-slate-800 text-amber-300 font-medium'
                    : 'text-slate-400 hover:text-slate-200'
                }`}
              >
                Raw
              </button>
              <button
                onClick={() => switchVerifiedTab('opt')}
                className={`px-2.5 py-0.5 text-[11px] rounded flex items-center gap-1 transition-all ${
                  displayPathSource === 'opt'
                    ? 'bg-emerald-900/60 text-emerald-300 font-medium'
                    : 'text-slate-400 hover:text-slate-200'
                }`}
              >
                <Sparkles size={10} />
                <span>Opt</span>
              </button>
            </div>
          ) : (
            <div className="flex items-center gap-2">
              <button
                onClick={() => handleVerifyPaths(displayPathSource === 'opt')}
                disabled={isVerifyingPaths || isOptimizingPaths}
                className="px-2.5 py-1 text-[11px] bg-slate-800 hover:bg-slate-700 text-slate-200 rounded-md border border-slate-700 flex items-center gap-1.5 disabled:opacity-50"
              >
                <RefreshCw size={11} className={isVerifyingPaths ? 'animate-spin text-emerald-400' : ''} />
                <span>{isVerifyingPaths ? 'Evaluating...' : 'Re-evaluate'}</span>
              </button>
              <button
                onClick={handleOptimizePaths}
                disabled={isVerifyingPaths || isOptimizingPaths}
                className="px-2.5 py-1 text-[11px] bg-gradient-to-r from-emerald-600 to-teal-600 hover:from-emerald-500 hover:to-teal-500 text-white rounded-md border border-emerald-500/50 flex items-center gap-1.5 shadow-sm disabled:opacity-50 font-semibold"
              >
                <Sparkles size={11} className={isOptimizingPaths ? 'animate-spin' : ''} />
                <span>{isOptimizingPaths ? 'Optimizing...' : 'Auto-Fix (ΔRz)'}</span>
              </button>
            </div>
          )}
        </div>
      </div>

      {/* MAIN CONTENT: 3 Columns */}
      <div className="flex-1 flex min-h-0">
        
        {/* Left Column: 2D Canvas OR Dedicated TCP Diagnostics View */}
        {activeViewTab === 'diagnostics' ? (
          <div className="flex-1 flex flex-col border-r border-slate-800 bg-slate-950 overflow-y-auto p-4 custom-scrollbar text-xs font-mono">
            {!verificationReport ? (
              <div className="flex-1 flex flex-col items-center justify-center py-16 text-slate-500 gap-3">
                <ShieldCheck size={42} className="text-slate-700 animate-pulse" />
                <div className="text-center">
                  <div className="font-semibold text-slate-300 text-sm">No Kinematic Diagnostics Data</div>
                  <div className="text-[11px] text-slate-500 mt-1 max-w-sm">
                    Click "Re-evaluate" or "Auto-Fix" to compute 6-DOF MoveL inverse kinematics and safety limits.
                  </div>
                </div>
                <button
                  onClick={() => handleVerifyPaths(displayPathSource === 'opt')}
                  disabled={isVerifyingPaths || !activeTemplate}
                  className="mt-2 px-4 py-2 bg-emerald-600 hover:bg-emerald-500 text-white rounded-lg text-xs font-semibold shadow-lg shadow-emerald-950/50 flex items-center gap-2 disabled:opacity-40"
                >
                  <RefreshCw size={13} className={isVerifyingPaths ? 'animate-spin' : ''} />
                  <span>{isVerifyingPaths ? 'Evaluating...' : 'Run Kinematics Verification'}</span>
                </button>
              </div>
            ) : (
              <div className="flex flex-col gap-4 max-w-4xl mx-auto w-full">
                {/* Diagnostics Summary Header Banner */}
                <div className="flex flex-wrap items-center justify-between gap-3 bg-slate-900/90 p-3.5 rounded-xl border border-slate-800 shadow-md">
                  <div className="flex items-center gap-3">
                    <div className="flex bg-slate-950 p-0.5 rounded-lg border border-slate-800">
                      <button
                        onClick={() => switchVerifiedTab('raw')}
                        className={`px-3 py-1 text-xs font-medium rounded-md transition-all flex items-center gap-1.5 ${
                          verifiedPathTab === 'raw'
                            ? 'bg-slate-800 text-amber-300 shadow border border-amber-500/30'
                            : 'text-slate-400 hover:text-slate-200'
                        }`}
                      >
                        <span>🛤️ Raw Path</span>
                      </button>
                      <button
                        onClick={() => switchVerifiedTab('opt')}
                        className={`px-3 py-1 text-xs font-medium rounded-md transition-all flex items-center gap-1.5 ${
                          verifiedPathTab === 'opt'
                            ? 'bg-emerald-900/60 text-emerald-300 shadow border border-emerald-500/40 font-semibold'
                            : 'text-slate-400 hover:text-slate-200'
                        }`}
                      >
                        <Sparkles size={12} className="text-emerald-400" />
                        <span>✨ Optimized (Opt)</span>
                      </button>
                    </div>

                    <span className="text-[11px] text-slate-400 hidden sm:block">
                      Source: <span className="text-slate-200 font-semibold">{verificationReport.source_file}</span>
                    </span>
                  </div>

                  {/* Summary Metric Badges */}
                  <div className="flex items-center gap-2">
                    <div className="bg-slate-950 px-2.5 py-1 rounded-lg border border-slate-800 text-[11px]">
                      <span className="text-slate-500">Paths:</span>{' '}
                      <span className="text-slate-200 font-bold">{verificationReport.summary?.total_paths}</span>
                    </div>

                    <div className="bg-slate-950 px-2.5 py-1 rounded-lg border border-slate-800 text-[11px]">
                      <span className="text-slate-500">MoveL Speed:</span>{' '}
                      <span className="text-cyan-400 font-bold">{verificationReport.nominal_speed_mm_s || kinParams.linearSpeedMmS} mm/s</span>
                    </div>

                    <div className="bg-slate-950 px-2.5 py-1 rounded-lg border border-slate-800 text-[11px]">
                      <span className="text-slate-500">Issues:</span>{' '}
                      <span className={`font-bold ${verificationReport.summary?.total_issues === 0 ? 'text-emerald-400' : 'text-rose-400'}`}>
                        {verificationReport.summary?.total_issues}
                      </span>
                    </div>

                    <div className={`px-2.5 py-1 rounded-lg text-[11px] font-bold uppercase flex items-center gap-1.5 border ${
                      verificationReport.summary?.status === 'PASS'
                        ? 'bg-emerald-950 text-emerald-300 border-emerald-500/40'
                        : verificationReport.summary?.status === 'WARNING'
                        ? 'bg-amber-950 text-amber-300 border-amber-500/40'
                        : 'bg-rose-950 text-rose-300 border-rose-500/40'
                    }`}>
                      <span>{verificationReport.summary?.status}</span>
                    </div>
                  </div>
                </div>

                {/* Per-Path Diagnostic Inspection & Waypoint Detail Cards */}
                <div className="grid grid-cols-1 gap-3">
                  {(verificationReport.path_reports || []).map((pRep: any) => {
                    const isPass = pRep.status === 'PASS';
                    const isWarning = pRep.status === 'WARNING';
                    const pathColor = PATH_PALETTE[(pRep.path_id - 1) % PATH_PALETTE.length];
                    const activePathsList = verifiedPathTab === 'opt' ? (optManualPaths.length > 0 ? optManualPaths : manualPaths) : (rawManualPaths.length > 0 ? rawManualPaths : manualPaths);
                    const matchingPath = activePathsList.find(p => p.path_id === pRep.path_id);
                    const ptsCount = matchingPath ? matchingPath.points.length : 0;
                    const startPt = matchingPath && matchingPath.points.length > 0 ? matchingPath.points[0] : null;
                    const endPt = matchingPath && matchingPath.points.length > 0 ? matchingPath.points[matchingPath.points.length - 1] : null;

                    return (
                      <div
                        key={pRep.path_id}
                        className={`p-3.5 rounded-xl border transition-all ${
                          isPass
                            ? 'bg-slate-900/60 border-emerald-800/30'
                            : isWarning
                            ? 'bg-amber-950/20 border-amber-700/40'
                            : 'bg-rose-950/25 border-rose-700/50'
                        }`}
                      >
                        {/* Path Card Header */}
                        <div className="flex flex-wrap items-center justify-between gap-2 mb-2.5">
                          <div className="flex items-center gap-2 font-semibold">
                            <span
                              className="w-3 h-3 rounded-full"
                              style={{ backgroundColor: pathColor }}
                            />
                            <span className="text-slate-100 font-bold text-sm">Path {pRep.path_id}</span>
                            <span className="text-slate-400 text-xs font-normal">({pRep.name})</span>
                            <span className="text-[11px] text-slate-500">
                              · {ptsCount} waypoints · {pRep.total_interpolated} MoveL steps · {pRep.speed_mm_s || kinParams.linearSpeedMmS} mm/s
                            </span>
                            {pRep.speed_mm_s && pRep.speed_mm_s < kinParams.linearSpeedMmS && (
                              <span className="text-[10px] px-1.5 py-0.5 rounded bg-emerald-950/90 text-emerald-300 border border-emerald-500/40 font-mono">
                                ⚡ Safe Speed: {pRep.speed_mm_s} mm/s
                              </span>
                            )}
                          </div>

                          <div className="flex items-center gap-2">
                            {/* Jump to 2D Canvas Button */}
                            <button
                              onClick={() => {
                                setDisplayPathSource(verifiedPathTab);
                                setActiveViewTab('2d');
                              }}
                              className="px-2 py-0.5 rounded text-[10px] bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700 flex items-center gap-1 transition-colors"
                              title="Show this path on 2D image"
                            >
                              <ImageIcon size={10} />
                              <span>View on 2D</span>
                            </button>

                            {isPass ? (
                              <span className="text-emerald-400 text-xs font-semibold px-2 py-0.5 rounded bg-emerald-950 border border-emerald-500/30">
                                ✅ Safe & Executable
                              </span>
                            ) : isWarning ? (
                              <div className="flex items-center gap-1.5">
                                {pRep.recommended_safe_speed_mm_s && pRep.recommended_safe_speed_mm_s < kinParams.linearSpeedMmS && (
                                  <button
                                    onClick={() => {
                                      const newSpd = pRep.recommended_safe_speed_mm_s;
                                      setKinParams({ ...kinParams, linearSpeedMmS: newSpd });
                                      handleVerifyPaths(verifiedPathTab === 'opt');
                                    }}
                                    className="px-2 py-0.5 rounded text-[10px] bg-amber-950 hover:bg-amber-900 text-amber-300 border border-amber-500/50 flex items-center gap-1 font-semibold transition-all shadow-sm"
                                    title="Click to apply recommended MoveL speed to achieve 100% PASS"
                                  >
                                    <span>💡 Safe MoveL: ≤ {pRep.recommended_safe_speed_mm_s} mm/s</span>
                                  </button>
                                )}
                                <span className="text-amber-400 text-xs font-semibold px-2 py-0.5 rounded bg-amber-950 border border-amber-500/30">
                                  ⚠️ Overspeed Warning
                                </span>
                              </div>
                            ) : (
                              <span className="text-rose-400 text-xs font-semibold px-2 py-0.5 rounded bg-rose-950 border border-rose-500/30">
                                ❌ Unreachable
                              </span>
                            )}
                          </div>
                        </div>

                        {/* Waypoints Physical Coordinates Ribbon */}
                        {startPt && endPt && (
                          <div className="bg-slate-950/50 px-2.5 py-1.5 rounded-lg border border-slate-800/60 mb-2.5 flex flex-wrap items-center justify-between text-[10px] text-slate-400 font-mono">
                            <div>
                              <span className="text-slate-500">Start (mm):</span>{' '}
                              <span className="text-slate-200">
                                [{startPt.surface_point_base_mm[0].toFixed(1)}, {startPt.surface_point_base_mm[1].toFixed(1)}, {startPt.surface_point_base_mm[2].toFixed(1)}]
                              </span>
                            </div>
                            <div>
                              <span className="text-slate-500">End (mm):</span>{' '}
                              <span className="text-slate-200">
                                [{endPt.surface_point_base_mm[0].toFixed(1)}, {endPt.surface_point_base_mm[1].toFixed(1)}, {endPt.surface_point_base_mm[2].toFixed(1)}]
                              </span>
                            </div>
                            <div>
                              <span className="text-slate-500">Standoff:</span>{' '}
                              <span className="text-cyan-400">{startPt.standoff_distance_mm || 150} mm</span>
                            </div>
                          </div>
                        )}

                        {/* 6-Axis Max Joint Velocities Grid with Raw vs Opt Comparison */}
                        <div className="bg-slate-950/80 p-2.5 rounded-lg border border-slate-800/80 mb-2.5">
                          <div className="text-[11px] text-slate-400 mb-1.5 flex items-center justify-between">
                            <span>Max Joint Velocities (°/s):</span>
                            <span className="text-[10px] text-slate-500">URDF Limit: ±179.9°/s</span>
                          </div>
                          <div className="grid grid-cols-6 gap-2 text-center font-mono">
                            {pRep.max_joint_velocity_deg_s.map((vel: number, jIdx: number) => {
                              const isOver = vel > 179.9;
                              const rawMatch = rawReportCache?.path_reports?.find((r: any) => r.path_id === pRep.path_id);
                              const rawVel = rawMatch?.max_joint_velocity_deg_s?.[jIdx];
                              const isReduced = verifiedPathTab === 'opt' && rawVel && rawVel > 179.9 && vel <= 179.9;

                              return (
                                <div
                                  key={jIdx}
                                  className={`p-1.5 rounded-md text-[11px] font-semibold border transition-all ${
                                    isOver
                                      ? 'bg-rose-950 text-rose-300 border-rose-600 animate-pulse font-bold'
                                      : isReduced
                                      ? 'bg-emerald-950/80 text-emerald-300 border-emerald-500/80 shadow-sm'
                                      : vel > 120
                                      ? 'bg-amber-950/60 text-amber-300 border-amber-700/50'
                                      : 'bg-slate-900 text-slate-300 border-slate-800'
                                  }`}
                                >
                                  <div className="text-[10px] text-slate-500 font-normal">J{jIdx + 1}</div>
                                  <div className="text-xs">{vel.toFixed(1)}°</div>
                                  {isReduced && (
                                    <div className="text-[9px] text-emerald-400 font-normal font-mono">
                                      was {rawVel.toFixed(0)}°
                                    </div>
                                  )}
                                </div>
                              );
                            })}
                          </div>
                        </div>

                        {/* Issues Breakdown */}
                        {pRep.issues && pRep.issues.length > 0 && (
                          <div className="space-y-1 mt-1 text-[11px]">
                            {pRep.issues.slice(0, 3).map((iss: any, iIdx: number) => (
                              <div key={iIdx} className="flex items-start gap-2 bg-slate-950/60 px-2.5 py-1.5 rounded text-slate-300 border border-slate-800/60">
                                <span className="shrink-0 text-amber-400 font-bold">
                                  {iss.type === 'UNREACHABLE' ? '❌' : '⚠️'} [{iss.type}]
                                </span>
                                <span className="text-slate-200">{iss.detail}</span>
                                {iss.location_xyz && (
                                  <span className="shrink-0 text-slate-500 text-[10px] ml-auto font-mono">
                                    XYZ=[{iss.location_xyz.join(', ')}] mm
                                  </span>
                                )}
                              </div>
                            ))}
                            {pRep.issues.length > 3 && (
                              <div className="text-[11px] text-slate-500 pl-2">
                                + {pRep.issues.length - 3} additional step warnings recorded.
                              </div>
                            )}
                          </div>
                        )}
                      </div>
                    );
                  })}
                </div>

                {/* Collapsible Kinematic Settings & Full TCP Calibration Drawer */}
                <div className="bg-slate-900/80 rounded-xl border border-slate-800 overflow-hidden shadow-lg">
                  <button
                    onClick={() => setShowParamsAccordion(!showParamsAccordion)}
                    className="w-full px-4 py-3 text-xs font-semibold text-slate-300 hover:text-white flex items-center justify-between hover:bg-slate-800/50 transition-colors"
                  >
                    <div className="flex items-center gap-2">
                      <span>⚙️ TCP Calibration & Kinematics Settings</span>
                      <span className="text-[10px] text-slate-500 font-normal hidden sm:inline">
                        (Tool Flange Offsets, Hand-Eye & MoveL Dynamics)
                      </span>
                    </div>
                    <span className="text-slate-400">{showParamsAccordion ? '▲ Collapse' : '▼ Expand'}</span>
                  </button>

                  {showParamsAccordion && (
                    <div className="p-4 border-t border-slate-800 bg-slate-950/90 flex flex-col gap-4 text-[11px]">
                      {/* Section 1: Tool Center Point (TCP) Offsets */}
                      <div>
                        <div className="text-slate-400 font-semibold mb-2 flex items-center justify-between border-b border-slate-800 pb-1">
                          <span>1. Tool Center Point (TCP) Flange Offsets</span>
                          <span className="text-[10px] text-slate-500 font-normal">End-Effector Nozzle Calibration</span>
                        </div>
                        <div className="grid grid-cols-2 md:grid-cols-6 gap-2.5">
                          <div>
                            <label className="text-slate-400 block mb-0.5 text-[10px]">TCP Offset X (mm)</label>
                            <input
                              type="number"
                              step="1"
                              value={kinParams.tcpOffsetX}
                              onChange={e => setKinParams({ ...kinParams, tcpOffsetX: parseFloat(e.target.value) || 0 })}
                              className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200 font-mono"
                            />
                          </div>
                          <div>
                            <label className="text-slate-400 block mb-0.5 text-[10px]">TCP Offset Y (mm)</label>
                            <input
                              type="number"
                              step="1"
                              value={kinParams.tcpOffsetY}
                              onChange={e => setKinParams({ ...kinParams, tcpOffsetY: parseFloat(e.target.value) || 0 })}
                              className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200 font-mono"
                            />
                          </div>
                          <div>
                            <label className="text-slate-400 block mb-0.5 text-[10px]">TCP Offset Z (mm)</label>
                            <input
                              type="number"
                              step="1"
                              value={kinParams.tcpOffsetZ}
                              onChange={e => setKinParams({ ...kinParams, tcpOffsetZ: parseFloat(e.target.value) || 0 })}
                              className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200 font-mono"
                            />
                          </div>
                          <div>
                            <label className="text-slate-400 block mb-0.5 text-[10px]">Offset Rx (°)</label>
                            <input
                              type="number"
                              step="1"
                              value={kinParams.tcpOffsetRx}
                              onChange={e => setKinParams({ ...kinParams, tcpOffsetRx: parseFloat(e.target.value) || 0 })}
                              className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200 font-mono"
                            />
                          </div>
                          <div>
                            <label className="text-slate-400 block mb-0.5 text-[10px]">Offset Ry (°)</label>
                            <input
                              type="number"
                              step="1"
                              value={kinParams.tcpOffsetRy}
                              onChange={e => setKinParams({ ...kinParams, tcpOffsetRy: parseFloat(e.target.value) || 0 })}
                              className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200 font-mono"
                            />
                          </div>
                          <div>
                            <label className="text-slate-400 block mb-0.5 text-[10px]">Offset Rz (°)</label>
                            <input
                              type="number"
                              step="1"
                              value={kinParams.tcpOffsetRz}
                              onChange={e => setKinParams({ ...kinParams, tcpOffsetRz: parseFloat(e.target.value) || 0 })}
                              className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200 font-mono"
                            />
                          </div>
                        </div>
                      </div>

                      {/* Section 2: Motion Kinematics & Slerp Interpolation */}
                      <div>
                        <div className="text-slate-400 font-semibold mb-2 flex items-center justify-between border-b border-slate-800 pb-1">
                          <span>2. Motion Kinematics & Interpolation</span>
                          <span className="text-[10px] text-slate-500 font-normal">Trajectory Resolution & Speed</span>
                        </div>
                        <div className="grid grid-cols-2 md:grid-cols-4 gap-2.5">
                          <div>
                            <label className="text-slate-400 block mb-0.5 text-[10px]">MoveL Velocity (mm/s)</label>
                            <input
                              type="number"
                              step="10"
                              value={kinParams.linearSpeedMmS}
                              onChange={e => setKinParams({ ...kinParams, linearSpeedMmS: parseFloat(e.target.value) || 120 })}
                              className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200 font-mono"
                            />
                          </div>
                          <div>
                            <label className="text-slate-400 block mb-0.5 text-[10px]">Slerp Step Size (mm)</label>
                            <input
                              type="number"
                              step="0.5"
                              value={kinParams.stepSizeMm}
                              onChange={e => setKinParams({ ...kinParams, stepSizeMm: parseFloat(e.target.value) || 1.5 })}
                              className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200 font-mono"
                            />
                          </div>
                          <div className="col-span-2 flex items-end">
                            <button
                              onClick={() => handleVerifyPaths(displayPathSource === 'opt')}
                              disabled={isVerifyingPaths || isOptimizingPaths}
                              className="w-full py-1.5 bg-cyan-600 hover:bg-cyan-500 text-white rounded font-medium shadow transition-all flex items-center justify-center gap-1.5 disabled:opacity-50"
                            >
                              <RefreshCw size={12} className={isVerifyingPaths ? 'animate-spin' : ''} />
                              <span>Apply Settings & Re-Evaluate</span>
                            </button>
                          </div>
                        </div>
                      </div>

                      {/* Section 3: Active Hand-Eye Calibration Info */}
                      <div className="p-2.5 bg-slate-900/60 rounded-lg border border-slate-800 flex items-center justify-between text-[10px] text-slate-400">
                        <span>📷 Hand-Eye Calibration: <strong className="text-slate-200">calib_20260812_230221 (Eye-to-Hand)</strong></span>
                        <span>Camera Pos: <strong className="text-slate-200 font-mono">[141.7, 23.3, 24.4] mm</strong></span>
                      </div>
                    </div>
                  )}
                </div>
              </div>
            )}
          </div>
        ) : (
        <div 
          ref={containerRef}
          className={`flex-1 flex flex-col border-r border-slate-800 relative bg-black items-center justify-center overflow-hidden select-none ${
            isPanning ? 'cursor-grabbing' : (segMode ? (isSpacePressed ? 'cursor-grab' : 'cursor-crosshair') : 'cursor-grab')
          }`}
          onWheel={handleWheel}
          onMouseDown={handleMouseDown}
        >
          {imageUrl ? (
            <div 
              className="relative inline-block max-w-full max-h-full"
              style={{
                transform: `translate(${pan.x}px, ${pan.y}px) scale(${zoom})`,
                transformOrigin: 'center center',
                transition: isPanning ? 'none' : 'transform 0.05s ease-out'
              }}
            >
              <img 
                 src={imageUrl} 
                 className="block max-w-full max-h-full object-contain pointer-events-none" 
                 alt="Captured view" 
                 onLoad={(e) => setNatSize({ w: e.currentTarget.naturalWidth, h: e.currentTarget.naturalHeight })}
              />

              {/* VIEW MODE: Render existing masks from scan.masks.yaml on top of scan.jpg */}
              {!segMode && showMasksOverlay && savedMasks.length > 0 && natSize && (
                <svg
                  className="absolute inset-0 w-full h-full pointer-events-none"
                  viewBox={`0 0 ${natSize.w} ${natSize.h}`}
                >
                  {savedMasks.map((m, idx) => {
                    const colorScheme = COLORS[idx % COLORS.length];
                    return (
                      <g key={idx}>
                        {renderPolygons(m.polygons, colorScheme.fill, colorScheme.stroke)}
                      </g>
                    );
                  })}
                </svg>
              )}

              {/* VIEW/OVERLAY MODE: Render Manual TCP Paths on top of scan.jpg */}
              {!manualPathMode && showManualPathsOverlay && manualPaths.length > 0 && natSize && (
                <svg
                  className="absolute inset-0 w-full h-full"
                  viewBox={`0 0 ${natSize.w} ${natSize.h}`}
                  style={{ cursor: 'default' }}
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
                    const pathColor = PATH_PALETTE[(pId - 1) % PATH_PALETTE.length];
                    const isHighlighted = highlightedPathId === pId;

                    // Check if there are verification issues reported on this path
                    const pathRep = verificationReport?.path_reports?.find((r: any) => r.path_id === pId);
                    const pathIssues = pathRep?.issues || [];

                    return (
                      <g key={pIdx}>
                        {/* Glowing Background Line for Highlighted / Focused Path */}
                        {isHighlighted && pts.map((p, i) => {
                          if (i === 0) return null;
                          const prev = pts[i - 1];
                          return (
                            <line
                              key={`hseg-${i}`}
                              x1={prev.pixel[0]}
                              y1={prev.pixel[1]}
                              x2={p.pixel[0]}
                              y2={p.pixel[1]}
                              stroke={pathColor}
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
                                stroke={hasSegIssue ? '#f43f5e' : pathColor}
                                strokeWidth={isHighlighted ? 3.5 : (hasSegIssue ? 3.0 : 2.5)}
                                markerEnd="url(#view-traj-arrow)"
                                strokeDasharray={hasSegIssue ? '6 3' : undefined}
                                style={{ pointerEvents: 'none' }}
                              />
                            </g>
                          );
                        })}

                        {/* Verification Issue Warning Beacons (Deduplicated per waypoint) */}
                        {Array.from(new Set(pathIssues.map((iss: any) => 
                          iss.step_index !== undefined
                            ? Math.min(Math.floor((iss.step_index / (pathRep.total_interpolated || 1)) * pts.length), pts.length - 1)
                            : (iss.waypoint_index !== undefined ? Math.min(iss.waypoint_index, pts.length - 1) : 0)
                        ))).map((targetWpIdx: any, bIdx: number) => {
                          const pt = pts[targetWpIdx];
                          if (!pt) return null;
                          const [u, v] = pt.pixel;
                          const issuesAtPt = pathIssues.filter((iss: any) => {
                            const idx = iss.step_index !== undefined
                              ? Math.min(Math.floor((iss.step_index / (pathRep.total_interpolated || 1)) * pts.length), pts.length - 1)
                              : (iss.waypoint_index !== undefined ? Math.min(iss.waypoint_index, pts.length - 1) : 0);
                            return idx === targetWpIdx;
                          });

                          return (
                            <g key={`beacon-grp-${bIdx}`} className="pointer-events-auto cursor-help">
                              {/* In-place concentric pulsing ripple halo 1 */}
                              <circle
                                cx={u}
                                cy={v}
                                r={8}
                                fill="rgba(244, 63, 94, 0.25)"
                                stroke="#f43f5e"
                                strokeWidth={2}
                                style={{ pointerEvents: 'none' }}
                              >
                                <animate attributeName="r" values="8;24" dur="1.6s" repeatCount="indefinite" />
                                <animate attributeName="opacity" values="0.9;0" dur="1.6s" repeatCount="indefinite" />
                              </circle>
                              {/* In-place concentric pulsing ripple halo 2 (staggered) */}
                              <circle
                                cx={u}
                                cy={v}
                                r={8}
                                fill="none"
                                stroke="#f43f5e"
                                strokeWidth={1.5}
                                style={{ pointerEvents: 'none' }}
                              >
                                <animate attributeName="r" values="8;24" begin="0.8s" dur="1.6s" repeatCount="indefinite" />
                                <animate attributeName="opacity" values="0.8;0" begin="0.8s" dur="1.6s" repeatCount="indefinite" />
                              </circle>

                              {/* Center Alert Dot */}
                              <circle
                                cx={u}
                                cy={v}
                                r={5.5}
                                fill="#f43f5e"
                                stroke="#ffffff"
                                strokeWidth={1.8}
                                style={{ pointerEvents: 'none' }}
                              />

                              {/* Visual Alert Badge attached to waypoint */}
                              {(() => {
                                const critIssue = issuesAtPt.find((i: any) => i.severity === 'ERROR')
                                  || issuesAtPt.find((i: any) => i.type === 'KINEMATIC_DISCONTINUITY')
                                  || issuesAtPt.find((i: any) => i.type === 'UNREACHABLE' || i.type === 'UNREACHABLE_STEP')
                                  || issuesAtPt.find((i: any) => i.type === 'ELBOW_SINGULARITY' || i.type === 'WRIST_SINGULARITY')
                                  || issuesAtPt[0];
                                
                                const isUnreach = critIssue?.type === 'UNREACHABLE' 
                                  || critIssue?.type === 'UNREACHABLE_STEP' 
                                  || critIssue?.type === 'KINEMATIC_DISCONTINUITY';
                                const isSing = critIssue?.type === 'ELBOW_SINGULARITY' || critIssue?.type === 'WRIST_SINGULARITY';
                                const label = isUnreach ? '❌ Unreachable' : (isSing ? '⚠️ Singularity' : '⚠️ Overspeed');
                                const badgeWidth = isUnreach ? 96 : (isSing ? 88 : 82);

                                return (
                                  <g transform={`translate(${u + 10}, ${v - 12})`} style={{ pointerEvents: 'none' }}>
                                    <rect
                                      x={0}
                                      y={0}
                                      width={badgeWidth}
                                      height={20}
                                      rx={4}
                                      fill="rgba(136, 19, 55, 0.95)"
                                      stroke="#f43f5e"
                                      strokeWidth={1}
                                      filter="drop-shadow(0px 2px 4px rgba(0,0,0,0.6))"
                                    />
                                    <text
                                      x={badgeWidth / 2}
                                      y={13}
                                      textAnchor="middle"
                                      fill="#ffe4e6"
                                      fontSize={9.5}
                                      fontWeight="bold"
                                      fontFamily="monospace"
                                    >
                                      {label}
                                    </text>
                                  </g>
                                );
                              })()}
                            </g>
                          );
                        })}



                        {/* Each Waypoint in Path */}
                        {pts.map((pt, idx) => {
                          const [u, v] = pt.pixel;
                          const dx = pt.normal_2d_proj?.[0] ?? 0;
                          const dy = pt.normal_2d_proj?.[1] ?? 0;
                          const tcpU = u + dx;
                          const tcpV = v + dy;
                          const arrowLen = Math.hypot(dx, dy);

                          return (
                            <g key={idx}
                              onMouseEnter={() => {
                                setHoveredWaypoint({ ...pt, path_id: pId } as any);
                                setHighlightedPathId(pId);
                              }}
                              onMouseLeave={() => {
                                setHoveredWaypoint(null);
                                setHighlightedPathId(null);
                              }}
                              style={{ cursor: 'pointer' }}
                            >
                              {/* Red Normal Offset Arrow (Perspective Foreshortened) */}
                              {arrowLen >= 3.0 ? (
                                <line
                                  x1={u} y1={v} x2={tcpU} y2={tcpV}
                                  stroke="#ef4444" strokeWidth={2.2} strokeLinecap="round"
                                  markerEnd="url(#view-normal-arrow)"
                                  style={{ pointerEvents: 'none' }}
                                />
                              ) : (
                                <circle cx={u} cy={v} r={3.5} fill="#ef4444" opacity={0.85} style={{ pointerEvents: 'none' }} />
                              )}

                              {/* Physical Surface Point Marker — large hit area */}
                              <circle cx={u} cy={v} r={14} fill="transparent" />
                              <circle 
                                cx={u} 
                                cy={v} 
                                r={isHighlighted ? 9 : 7.5} 
                                fill="white" 
                                filter="drop-shadow(0px 0px 4px rgba(0,0,0,0.8))" 
                                style={{ pointerEvents: 'none' }} 
                              />
                              <circle 
                                cx={u} 
                                cy={v} 
                                r={isHighlighted ? 6.5 : 5.5} 
                                fill={pathColor} 
                                style={{ pointerEvents: 'none' }} 
                              />
                              <text 
                                x={u} 
                                y={v + 0.5} 
                                textAnchor="middle" 
                                dominantBaseline="central" 
                                fill="white" 
                                fontSize={isHighlighted ? 9 : 8} 
                                fontWeight="bold" 
                                style={{ pointerEvents: 'none' }}
                              >
                                {idx + 1}
                              </text>
                            </g>
                          );
                        })}
                      </g>
                    );
                  })}


                  {/* Topmost Tooltip Layer in VIEW mode — rendered at the very end of SVG so it is ALWAYS on top */}
                  {!manualPathMode && hoveredWaypoint && (
                    (() => {
                      const pt = hoveredWaypoint;
                      const [u, v] = pt.pixel;
                      const dx = pt.normal_2d_proj?.[0] ?? 0;
                      const dy = pt.normal_2d_proj?.[1] ?? 0;
                      const tcpU = u + dx;
                      const tcpV = v + dy;

                      const isOptView = displayPathSource === 'opt';
                      const pId = (hoveredWaypoint as any).path_id || 1;
                      const curPath = manualPaths.find(p => p.path_id === pId);
                      const rawPath = rawManualPaths.find(p => p.path_id === pId || p.name === curPath?.name);
                      const optPath = optManualPaths.find(p => p.path_id === pId || p.name === curPath?.name);

                      const rawWp = rawPath?.points?.find(p => p.index === pt.index);
                      const optWp = optPath?.points?.find(p => p.index === pt.index);

                      // Check verification report issues
                      const pathRep = verificationReport?.path_reports?.find((r: any) => r.path_id === pId);
                      const isUnreachable = pathRep?.status === 'ERROR' || pathRep?.issues?.some((i: any) => i.type === 'UNREACHABLE' || i.type === 'UNREACHABLE_STEP');
                      const ptIssues = (pathRep?.issues || []).filter((iss: any) => {
                        if (iss.step_index !== undefined) {
                          const mappedWpIdx = Math.min(Math.floor((iss.step_index / (pathRep.total_interpolated || 1)) * (curPath?.points?.length || 1)), (curPath?.points?.length || 1) - 1);
                          return (curPath?.points?.[mappedWpIdx]?.index === pt.index);
                        }
                        return iss.waypoint_index === (pt.index - 1);
                      });

                      const deltaRz = optWp && rawWp ? Number((optWp.tcp_pose_base.rz - rawWp.tcp_pose_base.rz).toFixed(1)) : 0;
                      const hasRealHealing = !isUnreachable && (pathRep?.status === 'WARNING' || ptIssues.length > 0 || Math.abs(deltaRz) > 0.5);
                      const hasIssues = ptIssues.length > 0 || isUnreachable || pathRep?.status === 'WARNING';

                      const cardW = 244;
                      const cardH = isUnreachable ? 104 : ((isOptView && Math.abs(deltaRz) > 0.5) || (!isOptView && hasRealHealing) ? 116 : 92);
                      const tooltipX = Math.max(8, Math.min(tcpU + 10, (natSize?.w || 1280) - cardW - 8));
                      const tooltipY = Math.max(8, Math.min(tcpV - cardH - 10, (natSize?.h || 800) - cardH - 8));

                      const strokeColor = isUnreachable 
                        ? '#f43f5e' 
                        : (isOptView ? '#10b981' : (hasRealHealing ? '#f59e0b' : '#3b82f6'));

                      return (
                        <g transform={`translate(${tooltipX}, ${tooltipY})`} pointerEvents="none">
                          {/* Main HUD Card */}
                          <rect
                            x={0}
                            y={0}
                            width={cardW}
                            height={cardH}
                            rx={7}
                            fill="rgba(2, 6, 23, 0.96)"
                            stroke={strokeColor}
                            strokeWidth={1.5}
                            filter="drop-shadow(0px 8px 24px rgba(0,0,0,0.95))"
                          />
                          {/* Header Bar */}
                          <rect
                            x={0}
                            y={0}
                            width={cardW}
                            height={20}
                            rx={7}
                            fill={isUnreachable ? 'rgba(244, 63, 94, 0.3)' : (isOptView ? 'rgba(16, 185, 129, 0.2)' : (hasIssues ? 'rgba(245, 158, 11, 0.2)' : 'rgba(59, 130, 246, 0.2)'))}
                          />
                          <text x={8} y={13.5} fill={isUnreachable ? '#fca5a5' : (isOptView ? '#34d399' : (hasIssues ? '#fcd34d' : '#93c5fd'))} fontSize={8.5} fontFamily="monospace" fontWeight="bold">
                            {isUnreachable 
                              ? `❌ Unreachable · P${pId} - #${pt.index}`
                              : (isOptView ? `✨ Optimized (Opt) · P${pId} - #${pt.index}` : `🛤️ Raw Path · P${pId} - #${pt.index}`)}
                          </text>

                          {/* Coordinates */}
                          <text x={8} y={32} fill="#94a3b8" fontSize={7.5} fontFamily="monospace">
                            Surf: [{pt.surface_point_base_mm?.[0]?.toFixed(1)}, {pt.surface_point_base_mm?.[1]?.toFixed(1)}, {pt.surface_point_base_mm?.[2]?.toFixed(1)}] mm
                          </text>
                          <text x={8} y={44} fill="#e2e8f0" fontSize={7.5} fontFamily="monospace" fontWeight="bold">
                            TCP:  [{pt.tcp_pose_base.x}, {pt.tcp_pose_base.y}, {pt.tcp_pose_base.z}] mm
                          </text>
                          <text x={8} y={56} fill="#cbd5e1" fontSize={7.5} fontFamily="monospace">
                            Ori:  [{pt.tcp_pose_base.rx}°, {pt.tcp_pose_base.ry}°, {pt.tcp_pose_base.rz}°] (RPY)
                          </text>
                          <text x={8} y={68} fill="#94a3b8" fontSize={7.2} fontFamily="monospace">
                            Normal: [{pt.surface_normal_base?.[0]?.toFixed(2)}, {pt.surface_normal_base?.[1]?.toFixed(2)}, {pt.surface_normal_base?.[2]?.toFixed(2)}] · Dist: {pt.standoff_distance_mm}mm
                          </text>

                          {/* Divider */}
                          <line x1={8} y1={74} x2={cardW - 8} y2={74} stroke="#334155" strokeWidth={1} />

                          {/* Optimization & Verification Status Row */}
                          {isUnreachable ? (
                            <g transform="translate(8, 86)">
                              <text x={0} y={0} fill="#f87171" fontSize={7.8} fontFamily="monospace" fontWeight="bold">
                                ❌ Unreachable: IK out of workspace / singularity
                              </text>
                              <text x={0} y={12} fill="#fca5a5" fontSize={7.2} fontFamily="monospace">
                                ⚠️ Exceeds CR5 reach radius (~850mm). Re-position waypoints.
                              </text>
                            </g>
                          ) : isOptView ? (
                            Math.abs(deltaRz) > 0.5 ? (
                              <g transform="translate(8, 86)">
                                <text x={0} y={0} fill="#34d399" fontSize={7.8} fontFamily="monospace" fontWeight="bold">
                                  🔄 Rotation Auto-Fixed: ΔRz = {deltaRz >= 0 ? `+${deltaRz}` : deltaRz}° (Raw: {rawWp?.tcp_pose_base?.rz}°)
                                </text>
                                <text x={0} y={12} fill="#6ee7b7" fontSize={7.2} fontFamily="monospace">
                                  🛡️ Status: Overspeed & wrist singularity eliminated
                                </text>
                              </g>
                            ) : (
                              <g transform="translate(8, 86)">
                                <text x={0} y={0} fill="#34d399" fontSize={7.8} fontFamily="monospace" fontWeight="bold">
                                  ✅ Kinematics Safe: Continuous & reachable
                                </text>
                                <text x={0} y={12} fill="#6ee7b7" fontSize={7.2} fontFamily="monospace">
                                  🛡️ Trajectory smooth, all joint speeds within limits
                                </text>
                              </g>
                            )
                          ) : hasRealHealing ? (
                            <g transform="translate(8, 86)">
                              <text x={0} y={0} fill="#38bdf8" fontSize={7.8} fontFamily="monospace" fontWeight="bold">
                                ✨ Auto-Fix Available: Recommended Rz={optWp?.tcp_pose_base.rz}° (Δ: {deltaRz >= 0 ? `+${deltaRz}` : deltaRz}°)
                              </text>
                              <text x={0} y={12} fill="#f59e0b" fontSize={7.2} fontFamily="monospace" fontWeight="bold">
                                ⚠️ Raw Issue: {ptIssues[0]?.type || 'JOINT_OVERSPEED'} (Fixable in Diagnostics tab)
                              </text>
                            </g>
                          ) : (
                            <g transform="translate(8, 86)">
                              <text x={0} y={0} fill="#34d399" fontSize={7.8} fontFamily="monospace" fontWeight="bold">
                                ✅ Kinematics Safe: Continuous & reachable
                              </text>
                              <text x={0} y={12} fill="#94a3b8" fontSize={7.2} fontFamily="monospace">
                                🛡️ All 6 joints within speed & workspace limits
                              </text>
                            </g>
                          )}
                        </g>
                      );
                    })()
                  )}


                </svg>
              )}

              {/* EDIT MODE: Interactive Segment SVG */}
              {segMode && natSize && (
                <svg
                  className="absolute inset-0 w-full h-full cursor-crosshair"
                  viewBox={`0 0 ${natSize.w} ${natSize.h}`}
                  onClick={(e) => handleImageClick(e, 1)}
                  onContextMenu={(e) => handleImageClick(e, 0)}
                >
                  {/* Committed Masks */}
                  {committedMasks.map((m, idx) => {
                    const colorScheme = COLORS[idx % COLORS.length];
                    return (
                      <g key={idx}>
                        {renderPolygons(m.polygons, colorScheme.fill, colorScheme.stroke)}
                      </g>
                    );
                  })}
                  
                  {/* Current Active Mask */}
                  {renderPolygons(
                    currentPolygons, 
                    COLORS[committedMasks.length % COLORS.length].fill,
                    COLORS[committedMasks.length % COLORS.length].stroke
                  )}
                  
                  {/* Active Point Markers */}
                  {currentPoints.map((p, idx) => (
                    <g key={idx}>
                      <circle cx={p.x} cy={p.y} r={9} fill="white" filter="drop-shadow(0px 0px 3px rgba(0,0,0,0.8))" />
                      <circle cx={p.x} cy={p.y} r={6.5} fill={p.label === 1 ? '#10b981' : '#ef4444'} />
                      <text x={p.x} y={p.y + 0.5} textAnchor="middle" dominantBaseline="central" fill="white" fontSize={10} fontWeight="bold">
                        {p.label === 1 ? '+' : '-'}
                      </text>
                    </g>
                  ))}
                </svg>
              )}

              {/* EDIT MODE: Interactive Manual TCP Path Design SVG */}
              {manualPathMode && natSize && (
                <svg
                  className="absolute inset-0 w-full h-full cursor-crosshair"
                  viewBox={`0 0 ${natSize.w} ${natSize.h}`}
                  onClick={handleManualImageClick}
                  onMouseMove={handleManualMouseMove}
                  onMouseLeave={() => {
                    setMousePixel(null);
                    setLiveNormal(null);
                    setHoveredWaypoint(null);
                  }}
                >
                  <defs>
                    <marker id="normal-arrow" markerWidth="4" markerHeight="4" refX="3.2" refY="2" orient="auto">
                      <path d="M0,0.6 L0,3.4 L3.4,2 z" fill="#ef4444" />
                    </marker>
                    <marker id="edit-traj-arrow" markerWidth="5.5" markerHeight="5.5" refX="4.2" refY="2.75" orient="auto">
                      <path d="M0,0.6 L0,4.9 L4.9,2.75 z" fill="#f59e0b" />
                    </marker>
                  </defs>

                  {/* 1. Committed Paths (Clickable to Select for Editing) */}
                  {manualPaths.map((path, pIdx) => {
                    if (selectedPathIdForEdit === path.path_id) return null;
                    const pts = path.points;
                    const pathColor = PATH_PALETTE[(path.path_id - 1) % PATH_PALETTE.length];
                    return (
                      <g 
                        key={`committed-${pIdx}`} 
                        className="cursor-pointer opacity-70 hover:opacity-100 transition-opacity"
                        onClick={(e) => {
                          e.stopPropagation();
                          handleSelectPathForEdit(path.path_id);
                        }}
                      >
                        {pts.map((p, i) => {
                          if (i === 0) return null;
                          const prev = pts[i - 1];
                          return (
                            <line
                              key={`cseg-${i}`}
                              x1={prev.pixel[0]}
                              y1={prev.pixel[1]}
                              x2={p.pixel[0]}
                              y2={p.pixel[1]}
                              stroke={pathColor}
                              strokeWidth={2.5}
                              markerEnd="url(#edit-traj-arrow)"
                            />
                          );
                        })}
                        {pts.map((pt, idx) => (
                          <g key={idx}>
                            <circle cx={pt.pixel[0]} cy={pt.pixel[1]} r={6} fill={pathColor} stroke="white" strokeWidth={1.2} />
                            <text x={pt.pixel[0]} y={pt.pixel[1] + 0.5} textAnchor="middle" dominantBaseline="central" fill="white" fontSize={8} fontWeight="bold">
                              {idx + 1}
                            </text>
                          </g>
                        ))}
                      </g>
                    );
                  })}

                  {/* 2. Active Current Path Line Segments with Directional Arrow on EACH segment */}
                  {currentManualPoints.map((p, i) => {
                    if (i === 0) return null;
                    const prev = currentManualPoints[i - 1];
                    return (
                      <line
                        key={`curr-seg-${i}`}
                        x1={prev.pixel[0]}
                        y1={prev.pixel[1]}
                        x2={p.pixel[0]}
                        y2={p.pixel[1]}
                        stroke="#f59e0b"
                        strokeWidth={2.8}
                        markerEnd="url(#edit-traj-arrow)"
                      />
                    );
                  })}

                  {/* 2.5 Live Mouse Move Dashed Line & Arrow from Last Waypoint (Dark Green) */}
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

                  {/* 2.8 Real-time Live Normal Vector Arrow at Cursor */}
                  {mousePixel && (
                    <g pointerEvents="none">
                      {(() => {
                        const ldx = liveNormal?.dx ?? 0;
                        const ldy = liveNormal?.dy ?? 0;
                        const lLen = Math.hypot(ldx, ldy);
                        return lLen >= 3.0 ? (
                          <line
                            x1={mousePixel.u}
                            y1={mousePixel.v}
                            x2={mousePixel.u + ldx}
                            y2={mousePixel.v + ldy}
                            stroke="#ef4444"
                            strokeWidth={2}
                            strokeLinecap="round"
                            markerEnd="url(#normal-arrow)"
                          />
                        ) : (
                          <circle cx={mousePixel.u} cy={mousePixel.v} r={3.5} fill="#ef4444" opacity={0.8} />
                        );
                      })()}
                      {/* Aiming Cursor Target */}
                      <circle cx={mousePixel.u} cy={mousePixel.v} r={4.5} fill="#f59e0b" />
                      <circle cx={mousePixel.u} cy={mousePixel.v} r={8.5} fill="none" stroke="#f59e0b" strokeWidth={1.5} opacity={0.7} />
                    </g>
                  )}

                  {/* 3. Render Active Current Waypoints & Red Normal Vectors */}
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
                        key={`current-${idx}`}
                        onMouseEnter={() => setHoveredWaypoint(pt)}
                        onMouseLeave={() => setHoveredWaypoint(null)}
                        onContextMenu={(e) => {
                          e.preventDefault();
                          e.stopPropagation();
                          handleDeleteSingleWaypoint(idx);
                        }}
                      >
                        {/* Red Surface Normal Arrow pointing from surface to TCP (Sleek) */}
                        {arrowLen >= 3.0 ? (
                          <line
                            x1={u}
                            y1={v}
                            x2={tcpU}
                            y2={tcpV}
                            stroke="#ef4444"
                            strokeWidth={2.2}
                            strokeLinecap="round"
                            markerEnd="url(#normal-arrow)"
                          />
                        ) : (
                          <circle cx={u} cy={v} r={3.5} fill="#ef4444" opacity={0.85} />
                        )}

                        {/* Physical Surface Point Circle */}
                        <circle cx={u} cy={v} r={8.5} fill="white" filter="drop-shadow(0px 0px 4px rgba(0,0,0,0.9))" />
                        <circle cx={u} cy={v} r={6.5} fill="#f59e0b" />
                        <text x={u} y={v + 0.5} textAnchor="middle" dominantBaseline="central" fill="white" fontSize={8.5} fontWeight="bold">
                          {idx + 1}
                        </text>

                        {/* Sleek Mini Waypoint Pill at Normal End with One-Click Delete */}
                        <g 
                          transform={`translate(${tcpU + 4}, ${tcpV - 7})`}
                          className="cursor-pointer"
                          onClick={(e) => {
                            e.stopPropagation();
                            handleDeleteSingleWaypoint(idx);
                          }}
                        >
                          <rect
                            x={0}
                            y={0}
                            width={isHovered ? 44 : 32}
                            height={14}
                            rx={3}
                            fill={isHovered ? "rgba(136, 19, 55, 0.95)" : "rgba(2, 6, 23, 0.85)"}
                            stroke={isHovered ? "#f43f5e" : "rgba(245, 158, 11, 0.5)"}
                            strokeWidth={1}
                          />
                          <text x={isHovered ? 14 : 16} y={7.5} textAnchor="middle" dominantBaseline="central" fill="#fcd34d" fontSize={7.5} fontFamily="monospace" fontWeight="bold">
                            P{idx + 1}
                          </text>
                          {isHovered && (
                            <text x={34} y={7.5} textAnchor="middle" dominantBaseline="central" fill="#f43f5e" fontSize={8.5} fontWeight="bold">
                              ✕
                            </text>
                          )}
                        </g>
                      </g>
                    );
                  })}

                  {/* Topmost Tooltip Layer in EDIT mode — rendered at the very end of SVG */}
                  {manualPathMode && hoveredWaypoint && (
                    (() => {
                      const pt = hoveredWaypoint;
                      const [u, v] = pt.pixel;
                      const dx = pt.normal_2d_proj?.[0] ?? 0;
                      const dy = pt.normal_2d_proj?.[1] ?? 0;
                      const tcpU = u + dx;
                      const tcpV = v + dy;
                      const tooltipX = Math.max(8, Math.min(tcpU + 8, (natSize?.w || 1280) - 190));
                      const tooltipY = Math.max(8, Math.min(tcpV - 82, (natSize?.h || 800) - 86));

                      return (
                        <g transform={`translate(${tooltipX}, ${tooltipY})`} pointerEvents="none">
                          <rect
                            x={0}
                            y={0}
                            width={182}
                            height={78}
                            rx={6}
                            fill="rgba(2, 6, 23, 0.97)"
                            stroke="#f59e0b"
                            strokeWidth={1.5}
                            filter="drop-shadow(0px 8px 18px rgba(0,0,0,0.95))"
                          />
                          <text x={7} y={13} fill="#fcd34d" fontSize={7.8} fontFamily="monospace" fontWeight="bold">
                            Surf: [{pt.surface_point_base_mm?.[0]?.toFixed(1)}, {pt.surface_point_base_mm?.[1]?.toFixed(1)}, {pt.surface_point_base_mm?.[2]?.toFixed(1)}] mm
                          </text>
                          <text x={7} y={25} fill="#fbbf24" fontSize={7.8} fontFamily="monospace" fontWeight="bold">
                            TCP: [{pt.tcp_pose_base.x}, {pt.tcp_pose_base.y}, {pt.tcp_pose_base.z}] mm
                          </text>
                          <text x={7} y={37} fill="#cbd5e1" fontSize={7.2} fontFamily="monospace">
                            Ori: [{pt.tcp_pose_base.rx}°, {pt.tcp_pose_base.ry}°, {pt.tcp_pose_base.rz}°]
                          </text>
                          <text x={7} y={49} fill="#f87171" fontSize={7.2} fontFamily="monospace">
                            N: [{pt.surface_normal_base?.[0]?.toFixed(3)}, {pt.surface_normal_base?.[1]?.toFixed(3)}, {pt.surface_normal_base?.[2]?.toFixed(3)}]
                          </text>
                          <text x={7} y={61} fill="#94a3b8" fontSize={7.2} fontFamily="monospace">
                            Standoff: {pt.standoff_distance_mm}mm · Point #{pt.index}
                          </text>
                          <text x={7} y={71} fill="#f43f5e" fontSize={6.8} fontFamily="monospace" fontWeight="bold">
                            💡 Click pill or Right-click to Delete
                          </text>
                        </g>
                      );
                    })()
                  )}
                </svg>
              )}
            </div>
          ) : (
            <div className="w-full h-full flex flex-col items-center justify-center text-slate-600 gap-2">
              <ImageIcon size={48} className="opacity-20" />
              <span className="text-xs text-slate-500">No scan.jpg in this template</span>
            </div>
          )}

          {/* Loading Indicator for MobileSAM, Point Sampling, or Session Data */}
          {(isInitializingSam || isSamplingPoint || isLoadingSession) && (
            <div className="absolute inset-0 bg-slate-950/70 backdrop-blur-sm flex flex-col items-center justify-center text-blue-400 gap-3 z-40">
              <RefreshCw className="animate-spin text-blue-400" size={32} />
              <div className="flex flex-col items-center">
                <span className="font-semibold text-xs tracking-wide text-slate-100">
                  {isLoadingSession ? "Loading Depth & Calibration Data" : isSamplingPoint ? "Calculating 6D Pose & Surface Normal" : "Encoding Image with MobileSAM"}
                </span>
                <span className="text-[11px] text-slate-400 mt-0.5">
                  {isLoadingSession ? "Preparing client-side normal engine..." : isSamplingPoint ? "Querying depth and calibration..." : "Preparing interactive session..."}
                </span>
              </div>
            </div>
          )}

          {/* Bottom-Left Live Waypoint 6D Telemetry HUD (Unobtrusive floating bar) */}
          {manualPathMode && (hoveredWaypoint || currentManualPoints.length > 0 || (mousePixel && liveNormal?.tcpPose)) && (
            <div className="absolute bottom-14 left-3 z-30 pointer-events-none flex flex-col gap-1 bg-slate-950/85 backdrop-blur-md border border-amber-500/30 rounded-lg px-3 py-1.5 shadow-2xl transition-all">
              {(() => {
                const pt = hoveredWaypoint || currentManualPoints[currentManualPoints.length - 1];
                const displayPt = pt ? {
                  title: hoveredWaypoint ? `Point #${pt.index} (Hovered)` : `Point #${pt.index} (Latest)`,
                  dist: pt.standoff_distance_mm,
                  tcp: pt.tcp_pose_base,
                  surf: pt.surface_point_base_mm
                } : {
                  title: `Cursor Live Pose`,
                  dist: standoffDist,
                  tcp: liveNormal?.tcpPose || { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 },
                  surf: liveNormal?.surfPoint || null
                };

                return (
                  <>
                    <div className="flex items-center justify-between gap-3 text-[10px] border-b border-white/10 pb-0.5">
                      <span className="font-bold text-amber-400 flex items-center gap-1.5">
                        <Route size={11} className="text-amber-400" />
                        {displayPt.title}
                      </span>
                      <span className="text-amber-300 font-mono text-[9px] bg-amber-950/60 px-1.5 py-0.2 rounded border border-amber-500/30">
                        Standoff: {displayPt.dist}mm
                      </span>
                    </div>
                    {displayPt.surf && (
                      <div className="flex items-center gap-2 text-[9px] font-mono text-slate-400">
                        <span className="text-slate-500">Surf [mm]:</span>
                        <span className="text-slate-300">
                          [{Array.isArray(displayPt.surf) ? displayPt.surf.map((v: number) => v.toFixed(1)).join(', ') : '—'}]
                        </span>
                      </div>
                    )}
                    <div className="flex items-center gap-2 text-[9.5px] font-mono text-slate-300">
                      <span className="text-slate-400">TCP [mm]:</span>
                      <span className="text-amber-200 font-bold">
                        [{displayPt.tcp.x}, {displayPt.tcp.y}, {displayPt.tcp.z}]
                      </span>
                      <span className="text-slate-600">|</span>
                      <span className="text-slate-400">Ori [°]:</span>
                      <span className="text-slate-200">
                        [{displayPt.tcp.rx}°, {displayPt.tcp.ry}°, {displayPt.tcp.rz}°]
                      </span>
                    </div>
                  </>
                );
              })()}
            </div>
          )}

          {/* Top Left: Semi-transparent Zoom Controls with Consistent Height (h-7) & Unified Tooltip */}
          <div className="absolute top-3 left-3 flex items-center gap-0.5 bg-slate-950/40 hover:bg-slate-900/60 backdrop-blur-md border border-white/10 rounded-full px-1.5 h-7 z-20 shadow-lg transition-all">
            {/* Zoom In */}
            <div className="relative group flex items-center">
              <button 
                onClick={() => handleZoom(0.25)} 
                className="p-1 hover:bg-white/10 rounded-full text-slate-300 hover:text-white transition-colors"
              >
                <ZoomIn size={12} />
              </button>
              <div className="absolute top-full mt-2 left-0 hidden group-hover:flex flex-col pointer-events-none z-50">
                <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                  Zoom In (+)
                </div>
              </div>
            </div>

            {/* Zoom Out */}
            <div className="relative group flex items-center">
              <button 
                onClick={() => handleZoom(-0.25)} 
                className="p-1 hover:bg-white/10 rounded-full text-slate-300 hover:text-white transition-colors"
              >
                <ZoomOut size={12} />
              </button>
              <div className="absolute top-full mt-2 left-0 hidden group-hover:flex flex-col pointer-events-none z-50">
                <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                  Zoom Out (-)
                </div>
              </div>
            </div>

            <div className="w-px h-3 bg-white/10 mx-0.5" />

            {/* Reset Zoom */}
            <div className="relative group flex items-center">
              <button 
                onClick={resetZoom}
                className="px-1.5 py-0.5 text-[10px] font-mono text-slate-400 hover:text-slate-200 hover:bg-white/10 rounded-full transition-colors"
              >
                {Math.round(zoom * 100)}%
              </button>
              <div className="absolute top-full mt-2 left-1/2 -translate-x-1/2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                  Reset View (100%)
                </div>
              </div>
            </div>
          </div>

          {/* Top Right: Semi-transparent View Status Badges, Mask Switch & Manual Path Switch (Consistent Height h-7) */}
          {!segMode && !manualPathMode && (
            <div className="absolute top-3 right-3 flex items-center gap-2 z-20">
              {/* Glassmorphic Mask Toggle Switch */}
              <div className="relative group flex items-center">
                <button
                  onClick={() => setShowMasksOverlay(!showMasksOverlay)}
                  className={`h-7 px-2.5 rounded-full text-[10px] font-medium border flex items-center gap-2 backdrop-blur-md transition-all shadow-lg select-none ${
                    showMasksOverlay
                      ? 'bg-slate-950/50 hover:bg-slate-900/70 border-emerald-500/40 text-emerald-300 shadow-emerald-950/30'
                      : 'bg-slate-950/40 hover:bg-slate-900/60 border-white/10 text-slate-400 hover:text-slate-200'
                  }`}
                >
                  <div className="flex items-center gap-1.5">
                    {showMasksOverlay ? <Eye size={12} className="text-emerald-400" /> : <EyeOff size={12} className="text-slate-400" />}
                    <span className="tracking-wide">
                      Masks ({savedMasks.length})
                    </span>
                  </div>

                  <div 
                    className={`w-6 h-3 rounded-full transition-colors relative flex items-center px-0.5 border border-white/10 ${
                      showMasksOverlay ? 'bg-emerald-500/80' : 'bg-slate-800/80'
                    }`}
                  >
                    <div 
                      className={`w-2 h-2 rounded-full bg-white transition-transform duration-200 shadow-sm ${
                        showMasksOverlay ? 'translate-x-3' : 'translate-x-0'
                      }`} 
                    />
                  </div>
                </button>

                <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    {showMasksOverlay ? "Click to Hide Masks" : "Click to Show Masks"} · {savedMasks.length} Mask{savedMasks.length > 1 ? 's' : ''} in YAML
                  </div>
                </div>
              </div>

              {/* Glassmorphic Manual Paths Toggle Switch */}
              <div className="relative group flex items-center">
                <button
                  onClick={() => setShowManualPathsOverlay(!showManualPathsOverlay)}
                  className={`h-7 px-2.5 rounded-full text-[10px] font-medium border flex items-center gap-2 backdrop-blur-md transition-all shadow-lg select-none ${
                    showManualPathsOverlay
                      ? 'bg-slate-950/50 hover:bg-slate-900/70 border-rose-500/40 text-rose-300 shadow-rose-950/30'
                      : 'bg-slate-950/40 hover:bg-slate-900/60 border-white/10 text-slate-400 hover:text-slate-200'
                  }`}
                >
                  <div className="flex items-center gap-1.5">
                    <Route size={12} className={showManualPathsOverlay ? "text-rose-400" : "text-slate-400"} />
                    <span className="tracking-wide">
                      Manual Paths ({manualPaths.length})
                    </span>
                  </div>

                  <div 
                    className={`w-6 h-3 rounded-full transition-colors relative flex items-center px-0.5 border border-white/10 ${
                      showManualPathsOverlay ? 'bg-rose-500/80' : 'bg-slate-800/80'
                    }`}
                  >
                    <div 
                      className={`w-2 h-2 rounded-full bg-white transition-transform duration-200 shadow-sm ${
                        showManualPathsOverlay ? 'translate-x-3' : 'translate-x-0'
                      }`} 
                    />
                  </div>
                </button>

                <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    {showManualPathsOverlay ? "Click to Hide Manual Paths" : "Click to Show Manual Paths"} · {manualPaths.length} Path{manualPaths.length > 1 ? 's' : ''} in YAML
                  </div>
                </div>
              </div>

              {/* Resolution Badge */}
              {natSize && (
                <div className="relative group flex items-center">
                  <div className="h-7 px-2.5 rounded-full text-[10px] font-mono bg-slate-950/40 backdrop-blur-md text-slate-400 border border-white/10 shadow-lg flex items-center">
                    {natSize.w}×{natSize.h}
                  </div>
                  <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                    <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                      Resolution: {natSize.w} × {natSize.h} px
                    </div>
                  </div>
                </div>
              )}
            </div>
          )}
          
          {/* SEGMENTATION FLOATING TOOLBAR: Semi-transparent Frosted Glass with Unified Tooltips & Consistent Height (h-7) */}
          {segMode && !isInitializingSam && (
            <div className="absolute bottom-4 left-1/2 -translate-x-1/2 bg-slate-950/45 hover:bg-slate-950/65 backdrop-blur-md border border-white/15 rounded-full px-2.5 h-7 flex items-center gap-1.5 shadow-2xl z-30 transition-all">
              
              {/* Click Indicator */}
              <div className="flex items-center gap-1.5 px-0.5 text-[10px] text-slate-300 border-r border-white/10 pr-2">
                <span className="flex items-center gap-1 text-emerald-400 font-medium"><span className="w-1.5 h-1.5 rounded-full bg-emerald-400 inline-block shadow-[0_0_6px_rgba(52,211,153,0.6)]" />L(+)</span>
                <span className="flex items-center gap-1 text-red-400 font-medium"><span className="w-1.5 h-1.5 rounded-full bg-red-400 inline-block shadow-[0_0_6px_rgba(248,113,113,0.6)]" />R(-)</span>
                {committedMasks.length > 0 && (
                  <span className="text-slate-400 font-mono">| #{committedMasks.length + 1}</span>
                )}
              </div>
              
              {/* Undo Button with Tooltip */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleUndo} 
                  disabled={currentPoints.length === 0} 
                  className="p-1 text-slate-300 hover:text-white hover:bg-white/10 rounded-full disabled:opacity-25 transition-colors"
                >
                  <Undo2 size={13} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Undo Point (z)
                  </div>
                </div>
              </div>

              {/* Commit Button with Tooltip */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleCommit} 
                  disabled={currentPoints.length === 0 || currentPolygons.length === 0} 
                  className="p-1 text-emerald-400 hover:text-emerald-300 hover:bg-emerald-400/20 rounded-full disabled:opacity-25 transition-colors"
                >
                  <Check size={14} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Commit Object (n)
                  </div>
                </div>
              </div>

              <div className="w-px h-3 bg-white/10 mx-0.5" />

              {/* Reset Current Object Button with Tooltip */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleResetCurrent} 
                  disabled={currentPoints.length === 0} 
                  className="p-1 text-amber-400 hover:text-amber-300 hover:bg-amber-400/20 rounded-full disabled:opacity-25 transition-colors"
                >
                  <RefreshCw size={12} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Reset Points (r)
                  </div>
                </div>
              </div>

              {/* Clear All Button with Tooltip */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleClearAll} 
                  disabled={committedMasks.length === 0 && currentPoints.length === 0} 
                  className="p-1 text-rose-400 hover:text-rose-300 hover:bg-rose-400/20 rounded-full disabled:opacity-25 transition-colors"
                >
                  <Trash2 size={12} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Clear All Masks (c)
                  </div>
                </div>
              </div>

              <div className="w-px h-3 bg-white/10 mx-0.5" />

              {/* Save Masks Button with Tooltip */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleSaveMasks} 
                  disabled={committedMasks.length === 0} 
                  className="p-1 bg-blue-600/80 hover:bg-blue-600 text-white rounded-full shadow-lg shadow-blue-900/30 border border-blue-400/30 disabled:opacity-25 transition-all active:scale-95"
                >
                  <Save size={12} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Save to YAML (s)
                  </div>
                </div>
              </div>

              {/* Close/Exit Segment Mode */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={toggleSegMode} 
                  className="p-1 text-slate-400 hover:text-slate-200 hover:bg-white/10 rounded-full transition-colors"
                >
                  <X size={13} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Exit Seg Mode
                  </div>
                </div>
              </div>

            </div>
          )}

          {/* MANUAL TCP FLOATING TOOLBAR: Semi-transparent Frosted Glass with Consistent Height (h-7) & Standoff Distance Adjustment */}
          {manualPathMode && (
            <div className="absolute bottom-4 left-1/2 -translate-x-1/2 bg-slate-950/50 hover:bg-slate-950/70 backdrop-blur-md border border-rose-500/30 rounded-full px-2.5 h-7 flex items-center gap-1.5 shadow-2xl z-30 transition-all">
              
              {/* Existing Paths Chips with Click-to-Select and Individual Delete */}
              {manualPaths.length > 0 && (
                <div className="flex items-center gap-1 px-1 border-r border-white/10 max-w-[280px] overflow-x-auto custom-scrollbar">
                  {manualPaths.map((p) => {
                    const isSelected = selectedPathIdForEdit === p.path_id;
                    const color = PATH_PALETTE[(p.path_id - 1) % PATH_PALETTE.length];
                    return (
                      <div
                        key={p.path_id}
                        onClick={() => handleSelectPathForEdit(p.path_id)}
                        className={`flex items-center gap-1 px-2 py-0.5 rounded-full border text-[9px] cursor-pointer select-none transition-all ${
                          isSelected
                            ? 'bg-rose-950/90 border-rose-400 text-rose-100 shadow-md ring-1 ring-rose-400/50'
                            : 'bg-slate-900/90 border-white/10 text-slate-300 hover:border-slate-500 hover:text-white'
                        }`}
                        title={`Click to edit Path ${p.path_id} (${p.points.length} waypoints)`}
                      >
                        <span
                          className="w-1.5 h-1.5 rounded-full"
                          style={{ backgroundColor: color }}
                        />
                        <span className="font-bold">P{p.path_id}</span>
                        {isSelected && (
                          <span className="text-[8px] text-amber-300 font-mono font-semibold">
                            [edit]
                          </span>
                        )}
                        <button
                          onClick={(e) => {
                            e.stopPropagation();
                            handleDeletePath(p.path_id);
                          }}
                          className="text-slate-500 hover:text-rose-400 p-0.5 rounded hover:bg-white/10 transition-colors ml-0.5"
                          title={`Delete Path ${p.path_id}`}
                        >
                          <Trash2 size={9} />
                        </button>
                      </div>
                    );
                  })}
                </div>
              )}

              {/* Current Path / Active Edit Indicator */}
              <div className="flex items-center gap-1.5 px-0.5 text-[10px] text-slate-300 border-r border-white/10 pr-2">
                {selectedPathIdForEdit !== null ? (
                  <div className="flex items-center gap-1">
                    <span className="flex items-center gap-1 text-amber-400 font-bold">
                      <Route size={11} className="text-amber-400" />
                      Edit P{selectedPathIdForEdit}
                    </span>
                    <span className="text-slate-400 font-mono">({currentManualPoints.length} pts)</span>
                    <button
                      onClick={handleDeselectEditPath}
                      className="ml-1 px-1.5 py-0.2 rounded text-[9px] bg-slate-800 hover:bg-slate-700 text-slate-300 border border-white/10 transition-colors"
                      title="Finish editing and start a new path"
                    >
                      + New
                    </button>
                  </div>
                ) : (
                  <div className="flex items-center gap-1">
                    <span className="flex items-center gap-1 text-rose-400 font-medium">
                      <Route size={12} className="text-rose-400" />
                      Path #{manualPaths.length + 1}
                    </span>
                    <span className="text-slate-400 font-mono">({currentManualPoints.length} pts)</span>
                  </div>
                )}
              </div>

              {/* Standoff Distance Adjustment Pill */}
              <div className="flex items-center gap-1 bg-slate-900/80 border border-white/10 rounded-full px-2 h-5 text-[10px] font-mono text-slate-300">
                <span className="text-slate-400">Dist:</span>
                <button 
                  onClick={() => handleStandoffChange(standoffDist - 10)}
                  className="w-3.5 h-3.5 flex items-center justify-center hover:bg-white/10 rounded text-slate-300 hover:text-white"
                  title="Decrease standoff by 10mm"
                >
                  <Minus size={9} />
                </button>
                <span className="text-rose-300 font-bold min-w-[34px] text-center">{standoffDist}mm</span>
                <button 
                  onClick={() => handleStandoffChange(standoffDist + 10)}
                  className="w-3.5 h-3.5 flex items-center justify-center hover:bg-white/10 rounded text-slate-300 hover:text-white"
                  title="Increase standoff by 10mm"
                >
                  <Plus size={9} />
                </button>
              </div>

              <div className="w-px h-3 bg-white/10 mx-0.5" />

              {/* Undo Button */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleManualUndo} 
                  disabled={currentManualPoints.length === 0} 
                  className="p-1 text-slate-300 hover:text-white hover:bg-white/10 rounded-full disabled:opacity-25 transition-colors"
                >
                  <Undo2 size={13} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Undo Point (z)
                  </div>
                </div>
              </div>

              {/* Commit / New Path Button */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleManualCommit} 
                  disabled={currentManualPoints.length === 0} 
                  className="p-1 text-emerald-400 hover:text-emerald-300 hover:bg-emerald-400/20 rounded-full disabled:opacity-25 transition-colors"
                >
                  <Check size={14} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Commit / New Path (n)
                  </div>
                </div>
              </div>

              <div className="w-px h-3 bg-white/10 mx-0.5" />

              {/* Reset Current Path Button */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleManualResetCurrent} 
                  disabled={currentManualPoints.length === 0} 
                  className="p-1 text-amber-400 hover:text-amber-300 hover:bg-amber-400/20 rounded-full disabled:opacity-25 transition-colors"
                >
                  <RefreshCw size={12} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Reset Points (r)
                  </div>
                </div>
              </div>

              {/* Clear All Paths Button */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleManualClearAll} 
                  disabled={manualPaths.length === 0 && currentManualPoints.length === 0} 
                  className="p-1 text-rose-400 hover:text-rose-300 hover:bg-rose-400/20 rounded-full disabled:opacity-25 transition-colors"
                >
                  <Trash2 size={12} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Clear All Paths (c)
                  </div>
                </div>
              </div>

              <div className="w-px h-3 bg-white/10 mx-0.5" />

              {/* Save Paths Button */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleManualSavePaths} 
                  disabled={manualPaths.length === 0 && currentManualPoints.length === 0} 
                  className="p-1 bg-rose-600/80 hover:bg-rose-600 text-white rounded-full shadow-lg shadow-rose-900/30 border border-rose-400/30 disabled:opacity-25 transition-all active:scale-95"
                >
                  <Save size={12} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Save to YAML (s)
                  </div>
                </div>
              </div>

              {/* Close/Exit Manual Path Mode */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={toggleManualPathMode} 
                  className="p-1 text-slate-400 hover:text-slate-200 hover:bg-white/10 rounded-full transition-colors"
                >
                  <X size={13} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Exit Manual TCP Mode
                  </div>
                </div>
              </div>

            </div>
          )}
        </div>
        )}


        {/* Middle Column: Categorized Grouped File List */}
        <div className="w-[185px] shrink-0 border-r border-slate-800 bg-slate-950/40 flex flex-col min-h-0">
          <div className="h-9 px-2.5 border-b border-slate-800 bg-slate-900/90 flex justify-between items-center shrink-0">
            <div className="flex items-center gap-1.5 text-[11px] text-slate-300 font-medium">
              <HardDrive size={13} className="text-slate-400" />
              <span>Files</span>
            </div>
            <span className="bg-slate-800 text-slate-400 text-[9px] font-mono px-1.5 py-0.5 rounded border border-slate-700">
              {files.length}
            </span>
          </div>

          <div className="flex-1 overflow-y-auto overflow-x-hidden custom-scrollbar p-1.5 flex flex-col gap-2">
            {files.length === 0 ? (
              <div className="flex flex-col items-center justify-center h-32 text-slate-600 text-[11px] gap-1">
                <FileCode2 size={20} className="opacity-30" />
                <span>Empty</span>
              </div>
            ) : (
              fileCategories.map(cat => {
                if (cat.files.length === 0) return null;
                const CatIcon = cat.icon;
                return (
                  <div key={cat.id} className="flex flex-col gap-1">
                    {/* Category Divider Header */}
                    <div className="flex items-center justify-between px-1 pt-1 pb-0.5 border-b border-slate-800/80">
                      <div className="flex items-center gap-1 text-[10px] font-semibold tracking-wider uppercase text-slate-400">
                        <CatIcon size={11} className={cat.iconColor} />
                        <span>{cat.title}</span>
                      </div>
                      <span className={`text-[8px] font-mono px-1 py-0.2 rounded border ${cat.badgeColor}`}>
                        {cat.files.length}
                      </span>
                    </div>

                    {/* Files in this Category */}
                    <div className="flex flex-col gap-1">
                      {cat.files.map(file => {
                        const badge = getFileBadge(file.name);
                        const isMaskYaml = file.name === 'scan.masks.yaml';
                        const isManualPathYaml = file.name.includes('manual_paths') || file.name.includes('paths.yaml');
                        const isMesh = file.name.includes('.ply') || file.name.includes('.stl');
                        return (
                          <div 
                            key={file.name}
                            className={`p-1.5 rounded-lg border transition-all flex flex-col gap-0.5 ${
                              isMaskYaml 
                                ? 'bg-emerald-950/25 border-emerald-500/40 shadow-sm' 
                                : isManualPathYaml
                                  ? 'bg-rose-950/25 border-rose-500/40 shadow-sm'
                                  : isMesh
                                    ? 'bg-purple-950/25 border-purple-500/40 shadow-sm'
                                    : 'bg-slate-900/60 border-slate-800/80 hover:border-slate-700 hover:bg-slate-800/50'
                            }`}
                          >
                            <div className="flex items-center justify-between gap-1">
                              <div className="flex items-center gap-1.5 min-w-0">
                                <div className="p-0.5 rounded bg-slate-800 shrink-0">
                                  {getFileIcon(file.name)}
                                </div>
                                <span className={`text-[11px] font-medium truncate ${
                                  isMaskYaml ? 'text-emerald-300' : (isManualPathYaml ? 'text-rose-300' : (isMesh ? 'text-purple-300' : 'text-slate-200'))
                                }`}>
                                  {file.name}
                                </span>
                              </div>
                              {badge && (
                                <span className={`text-[8px] px-1 py-0.2 rounded border font-mono shrink-0 ${badge.color}`}>
                                  {badge.label}
                                </span>
                              )}
                            </div>
                            
                            <div className="flex items-center justify-between text-[9px] text-slate-500 font-mono px-0.5">
                              <span>{formatFileSize(file.size)}</span>
                              <span>{formatFileDate(file.ctime)}</span>
                            </div>
                          </div>
                        );
                      })}
                    </div>
                  </div>
                );
              })
            )}
          </div>
        </div>

        {/* Right Column: Narrow Action Buttons with Unified Tooltips */}
        <div className="w-[125px] shrink-0 bg-slate-950/60 flex flex-col justify-end p-2.5 border-l border-slate-800 gap-2">
           {/* Capture Button */}
           <div className="relative group w-full">
             <button 
               onClick={handleCapture}
               disabled={isCapturing || isReconstructing || !activeTemplate || segMode || manualPathMode}
               className="w-full py-2.5 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-50 disabled:cursor-not-allowed border border-slate-700"
             >
               {isCapturing ? <RefreshCw size={14} className="animate-spin text-blue-400" /> : <Camera size={14} />}
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
               onClick={toggleSegMode}
               disabled={!hasImage || isCapturing || isReconstructing || manualPathMode}
               className={`w-full py-2.5 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-50 disabled:cursor-not-allowed border ${
                 segMode 
                   ? 'bg-red-600/20 text-red-300 border-red-500/50 hover:bg-red-600/30' 
                   : 'bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-500 hover:to-indigo-500 text-white shadow-blue-900/30 border-blue-500/50'
               }`}
             >
               {segMode ? <X size={14} /> : <Sparkles size={14} />}
               <span>{segMode ? 'Exit' : 'Segment'}</span>
             </button>
             <div className="absolute bottom-full mb-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
               <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                 {segMode ? 'Exit Segmentation Mode' : 'MobileSAM Interactive Segmentation'}
               </div>
             </div>
           </div>

           {/* Manual TCP Path Button */}
           <div className="relative group w-full">
             <button 
               onClick={toggleManualPathMode}
               disabled={!hasImage || isCapturing || isReconstructing || segMode}
               className={`w-full py-2.5 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-50 disabled:cursor-not-allowed border ${
                 manualPathMode 
                   ? 'bg-rose-600/25 text-rose-300 border-rose-500/50 hover:bg-rose-600/35' 
                   : 'bg-gradient-to-r from-rose-600 to-pink-600 hover:from-rose-500 hover:to-pink-500 text-white shadow-rose-900/30 border-rose-500/50'
               }`}
             >
               {manualPathMode ? <X size={14} /> : <Route size={14} />}
               <span>{manualPathMode ? 'Exit' : 'Manual TCP'}</span>
             </button>
             <div className="absolute bottom-full mb-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
               <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                 {manualPathMode ? 'Exit Manual TCP Design' : 'Manual TCP Path & Normal Design'}
               </div>
             </div>
           </div>

            {/* TCP Opt Button (Compact & Sleek) */}
            <div className="relative group w-full">
              <button 
                onClick={() => {
                  if (activeViewTab === 'diagnostics') {
                    setActiveViewTab('2d');
                  } else {
                    setActiveViewTab('diagnostics');
                    if (!verificationReport) {
                      handleVerifyPaths(displayPathSource === 'opt');
                    }
                  }
                }} 
                disabled={!hasImage || !hasManualPaths || isCapturing || isReconstructing || isVerifyingPaths || isOptimizingPaths || segMode || manualPathMode}
                className={`w-full py-2.5 font-medium rounded-lg shadow transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-40 disabled:cursor-not-allowed border ${
                  activeViewTab === 'diagnostics'
                    ? 'bg-emerald-600/30 text-emerald-300 border-emerald-500/60 shadow-emerald-950/40'
                    : 'bg-gradient-to-r from-emerald-600 to-teal-600 hover:from-emerald-500 hover:to-teal-500 text-white shadow-emerald-900/30 border-emerald-500/50'
                }`}
              >
                {isVerifyingPaths ? <RefreshCw size={14} className="animate-spin text-emerald-200" /> : <ShieldCheck size={14} />}
                <span>{isVerifyingPaths ? 'Evaluating...' : 'TCP Opt'}</span>
              </button>
              <div className="absolute bottom-full mb-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
                <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                  {!hasManualPaths ? "Need scan.manual_paths.yaml first" : "Toggle 6-DOF Kinematics & Auto-Fix Panel"}
                </div>
              </div>
            </div>

           {/* Surface Reconstruction Button */}
           <div className="relative group w-full">
             <button 
               onClick={handleReconstruct}
               disabled={!hasImage || !hasMasks || !hasDepth || isCapturing || isReconstructing || segMode || manualPathMode}
               className="w-full py-2.5 bg-gradient-to-r from-purple-600 to-indigo-600 hover:from-purple-500 hover:to-indigo-500 text-white font-medium rounded-lg shadow-lg shadow-purple-900/30 transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-40 disabled:cursor-not-allowed border border-purple-500/50"
             >
               {isReconstructing ? <RefreshCw size={14} className="animate-spin text-purple-200" /> : <Box size={14} />}
               <span>{isReconstructing ? 'Rebuilding...' : 'Reconstruct'}</span>
             </button>
             <div className="absolute bottom-full mb-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
               <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                 {!hasMasks ? "Need scan.masks.yaml first" : "Surface Poisson 3D Mesh Reconstruction"}
               </div>
             </div>
           </div>
           
           {!activeTemplate && (
             <div className="text-center text-[9px] text-slate-500">Select Template</div>
           )}
        </div>
      </div>
    </div>
  );
};

export default InteractiveOp;
