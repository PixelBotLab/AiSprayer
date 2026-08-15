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
  Minus
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
function smallestEigenvector3x3(cov: number[][]): [number, number, number] {
  // Power iteration on (maxEig*I - cov) to find smallest eigenvector of 3x3 symmetric matrix
  // Find approximate largest eigenvalue via Gershgorin
  let maxEig = 0;
  for (let i = 0; i < 3; i++) {
    let row = Math.abs(cov[i][i]);
    for (let j = 0; j < 3; j++) if (j !== i) row += Math.abs(cov[i][j]);
    if (row > maxEig) maxEig = row;
  }
  // Shift matrix: A' = maxEig*I - cov
  const A: number[][] = [
    [maxEig - cov[0][0], -cov[0][1], -cov[0][2]],
    [-cov[1][0], maxEig - cov[1][1], -cov[1][2]],
    [-cov[2][0], -cov[2][1], maxEig - cov[2][2]]
  ];
  // Power iteration on A' converges to eigenvector of largest eigenvalue of A' = smallest of cov
  let v: [number, number, number] = [0.57735, 0.57735, 0.57735];
  for (let iter = 0; iter < 32; iter++) {
    const nv: [number, number, number] = [
      A[0][0]*v[0]+A[0][1]*v[1]+A[0][2]*v[2],
      A[1][0]*v[0]+A[1][1]*v[1]+A[1][2]*v[2],
      A[2][0]*v[0]+A[2][1]*v[1]+A[2][2]*v[2]
    ];
    const len = Math.sqrt(nv[0]**2+nv[1]**2+nv[2]**2);
    if (len < 1e-10) break;
    v = [nv[0]/len, nv[1]/len, nv[2]/len];
  }
  return v;
}

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
  const [currentManualPoints, setCurrentManualPoints] = useState<WaypointItem[]>([]);
  const [showManualPathsOverlay, setShowManualPathsOverlay] = useState(true);
  const [isSamplingPoint, setIsSamplingPoint] = useState(false);
  const [isLoadingSession, setIsLoadingSession] = useState(false);
  const [mousePixel, setMousePixel] = useState<{ u: number, v: number } | null>(null);
  const [hoveredWaypoint, setHoveredWaypoint] = useState<WaypointItem | null>(null);
  const [liveNormal, setLiveNormal] = useState<{ dx: number; dy: number; tcpPose?: any; surfPoint?: any } | null>(null);

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

  // ─── Pure JS client-side normal computation (mirrors verify_tab draw_point_normal logic) ───
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

    // Gather neighborhood depth (21x21 window, outlier-rejected)
    const WIN = 10;
    const center_z = depth[v * width + u];
    const validZ = (center_z > 100 && center_z < 3000) ? center_z : 0;
    const pts3d: number[][] = [];
    for (let dv = -WIN; dv <= WIN; dv++) {
      for (let du = -WIN; du <= WIN; du++) {
        const pu = u + du, pv = v + dv;
        if (pu < 0 || pu >= width || pv < 0 || pv >= height) continue;
        const z = depth[pv * width + pu];
        if (z < 100 || z > 3000) continue;
        if (validZ > 0 && Math.abs(z - validZ) > 40) continue;
        const x = (pu - cx) * z / fx;
        const y = (pv - cy) * z / fy;
        pts3d.push([x, y, z]);
      }
    }

    const center_z_use = validZ > 0 ? validZ : (pts3d.length > 0 ? pts3d.reduce((s, p) => s + p[2], 0) / pts3d.length : 800);
    const surf_cam: [number, number, number] = [
      (u - cx) * center_z_use / fx,
      (v - cy) * center_z_use / fy,
      center_z_use
    ];

    let normal_cam: [number, number, number] = [0, 0, -1];
    if (pts3d.length >= 6) {
      // Compute centroid
      const n = pts3d.length;
      let mx = 0, my = 0, mz = 0;
      for (const p of pts3d) { mx += p[0]; my += p[1]; mz += p[2]; }
      mx /= n; my /= n; mz /= n;
      // 3x3 covariance via outer products
      let c00=0,c01=0,c02=0,c11=0,c12=0,c22=0;
      for (const p of pts3d) {
        const dx=p[0]-mx, dy=p[1]-my, dz=p[2]-mz;
        c00+=dx*dx; c01+=dx*dy; c02+=dx*dz;
        c11+=dy*dy; c12+=dy*dz; c22+=dz*dz;
      }
      // Power-iteration to find smallest eigenvector (normal)
      // Use Jacobi-like approach for 3x3 symmetric: simplified — just pick the cross-product of 2 principal axes
      // For real SVD we approximate: build 2 tangent vectors and cross them
      // Method: use PCA via iteration
      const cov = [[c00,c01,c02],[c01,c11,c12],[c02,c12,c22]];
      const eigvec = smallestEigenvector3x3(cov);
      let nx = eigvec[0], ny = eigvec[1], nz = eigvec[2];
      if (nz > 0) { nx=-nx; ny=-ny; nz=-nz; } // point towards camera
      const len = Math.sqrt(nx*nx+ny*ny+nz*nz);
      if (len > 1e-6) { nx/=len; ny/=len; nz/=len; }
      normal_cam = [nx, ny, nz];
    }

    // Transform to base frame using T_base_camera (4x4 row-major)
    const R = [
      [T[0],T[1],T[2]], [T[4],T[5],T[6]], [T[8],T[9],T[10]]
    ];
    const t = [T[3], T[7], T[11]];
    const applyR = (v3: number[]) => [
      R[0][0]*v3[0]+R[0][1]*v3[1]+R[0][2]*v3[2],
      R[1][0]*v3[0]+R[1][1]*v3[1]+R[1][2]*v3[2],
      R[2][0]*v3[0]+R[2][1]*v3[1]+R[2][2]*v3[2]
    ];
    const surf_base_arr = applyR(surf_cam).map((v, i) => v + t[i]);
    const surf_base: [number, number, number] = [surf_base_arr[0], surf_base_arr[1], surf_base_arr[2]];
    const nb_arr = applyR(normal_cam);
    const nb_len = Math.sqrt(nb_arr[0]**2+nb_arr[1]**2+nb_arr[2]**2);
    const normal_base: [number, number, number] = nb_len > 1e-6
      ? [nb_arr[0]/nb_len, nb_arr[1]/nb_len, nb_arr[2]/nb_len]
      : [0, 0, 1];

    // TCP = surf + standoff * normal
    const tcp_base: [number, number, number] = [
      surf_base[0] + standoffMm * normal_base[0],
      surf_base[1] + standoffMm * normal_base[1],
      surf_base[2] + standoffMm * normal_base[2]
    ];

    // Euler angles
    const euler_deg = computeToolEuler(normal_base);

    // 2D projection: verify_tab method — project tcp_cam point back to image
    const tcp_cam = [
      surf_cam[0] + standoffMm * normal_cam[0],
      surf_cam[1] + standoffMm * normal_cam[1],
      surf_cam[2] + standoffMm * normal_cam[2]
    ];
    let proj_dx = 0, proj_dy = -36;
    if (tcp_cam[2] > 50) {
      const u_tcp = fx * tcp_cam[0] / tcp_cam[2] + cx;
      const v_tcp = fy * tcp_cam[1] / tcp_cam[2] + cy;
      const raw_dx = u_tcp - u;
      const raw_dy = v_tcp - v;
      const mag = Math.sqrt(raw_dx**2 + raw_dy**2);
      if (mag > 1.2) {
        proj_dx = (raw_dx / mag) * 36;
        proj_dy = (raw_dy / mag) * 36;
      }
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

  const fetchManualPaths = async (templateName: string) => {
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${templateName}/manual_paths`);
      if (res.ok) {
        const data = await res.json();
        setManualPaths(data.paths || []);
        if (data.standoff_distance_mm) {
          setStandoffDist(Number(data.standoff_distance_mm));
        }
      } else {
        setManualPaths([]);
      }
    } catch (err) {
      console.error('Failed to fetch manual paths:', err);
      setManualPaths([]);
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

  const scrollTabs = (dir: 'left' | 'right') => {
    if (scrollRef.current) {
      scrollRef.current.scrollBy({ left: dir === 'left' ? -200 : 200, behavior: 'smooth' });
    }
  };

  const hasImage = files.some(f => f.name === 'scan.jpg');
  const hasDepth = files.some(f => f.name === 'scan.depth.npy');
  const hasMasks = files.some(f => f.name === 'scan.masks.yaml');
  
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
    if (filename === 'scan.manual_paths.yaml' || filename.includes('paths.yaml')) return { label: 'MANUAL PATH', color: 'bg-rose-500/20 text-rose-300 border-rose-500/40' };
    if (filename === 'scan.params.yaml') return { label: 'PARAMS', color: 'bg-amber-500/20 text-amber-300 border-amber-500/40' };
    if (filename === 'scan.pcd') return { label: '3D PCD', color: 'bg-cyan-500/20 text-cyan-300 border-cyan-500/40' };
    if (filename === 'scan.depth.npy') return { label: 'DEPTH', color: 'bg-indigo-500/20 text-indigo-300 border-indigo-500/40' };
    if (filename === 'scan.jpg') return { label: 'IMAGE', color: 'bg-blue-500/20 text-blue-300 border-blue-500/40' };
    return null;
  };

  const getFileIcon = (filename: string) => {
    if (filename.includes('.ply') || filename.includes('.stl')) return <Box size={13} className="text-purple-400" />;
    if (filename.endsWith('.jpg') || filename.endsWith('.png')) return <ImageIcon size={13} className="text-blue-400" />;
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
      return;
    }
    if (!activeTemplate || !hasImage) return;
    if (segMode) setSegMode(false);
    setManualPathMode(true);
    setCurrentManualPoints([]);
    // Pre-load depth + calibration for client-side computation
    loadSessionData(activeTemplate);
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
    const newPath: ManualPathItem = {
      path_id: manualPaths.length + 1,
      name: `Manual_Path_${manualPaths.length + 1}`,
      points: [...currentManualPoints]
    };
    setManualPaths(prev => [...prev, newPath]);
    setCurrentManualPoints([]);
  };

  const handleManualResetCurrent = () => {
    setCurrentManualPoints([]);
  };

  const handleManualClearAll = () => {
    setManualPaths([]);
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
    const allPaths = [...manualPaths];
    if (currentManualPoints.length > 0) {
      allPaths.push({
        path_id: allPaths.length + 1,
        name: `Manual_Path_${allPaths.length + 1}`,
        points: [...currentManualPoints]
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

      {/* MAIN CONTENT: 3 Columns */}
      <div className="flex-1 flex min-h-0">
        
        {/* Left Column: Big Image Viewer */}
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
                      <path d="M0,0.6 L0,4.9 L4.9,2.75 z" fill="#f59e0b" />
                    </marker>
                    <marker id="view-normal-arrow" markerWidth="4" markerHeight="4" refX="3.2" refY="2" orient="auto">
                      <path d="M0,0.6 L0,3.4 L3.4,2 z" fill="#ef4444" />
                    </marker>
                  </defs>
                  {manualPaths.map((path, pIdx) => {
                    const pts = path.points;
                    const pathColor = '#f59e0b';
                    return (
                      <g key={pIdx}>
                        {/* Connecting Line with Arrow on EVERY consecutive segment */}
                        {pts.map((p, i) => {
                          if (i === 0) return null;
                          const prev = pts[i - 1];
                          return (
                            <line
                              key={`vseg-${i}`}
                              x1={prev.pixel[0]}
                              y1={prev.pixel[1]}
                              x2={p.pixel[0]}
                              y2={p.pixel[1]}
                              stroke={pathColor}
                              strokeWidth={2.5}
                              markerEnd="url(#view-traj-arrow)"
                              style={{ pointerEvents: 'none' }}
                            />
                          );
                        })}

                        {/* Each Waypoint in Path */}
                        {pts.map((pt, idx) => {
                          const [u, v] = pt.pixel;
                          const dx = pt.normal_2d_proj?.[0] || 15;
                          const dy = pt.normal_2d_proj?.[1] || -40;
                          const tcpU = u + dx;
                          const tcpV = v + dy;

                          return (
                            <g key={idx}
                              onMouseEnter={() => setHoveredWaypoint(pt)}
                              onMouseLeave={() => setHoveredWaypoint(null)}
                              style={{ cursor: 'pointer' }}
                            >
                              {/* Red Normal Offset Arrow */}
                              <line
                                x1={u} y1={v} x2={tcpU} y2={tcpV}
                                stroke="#ef4444" strokeWidth={2.2} strokeLinecap="round"
                                markerEnd="url(#view-normal-arrow)"
                                style={{ pointerEvents: 'none' }}
                              />

                              {/* Physical Surface Point Marker — large hit area */}
                              <circle cx={u} cy={v} r={14} fill="transparent" />
                              <circle cx={u} cy={v} r={7.5} fill="white" filter="drop-shadow(0px 0px 3px rgba(0,0,0,0.8))" style={{ pointerEvents: 'none' }} />
                              <circle cx={u} cy={v} r={5.5} fill={pathColor} style={{ pointerEvents: 'none' }} />
                              <text x={u} y={v + 0.5} textAnchor="middle" dominantBaseline="central" fill="white" fontSize={8} fontWeight="bold" style={{ pointerEvents: 'none' }}>
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
                      const dx = pt.normal_2d_proj?.[0] || 15;
                      const dy = pt.normal_2d_proj?.[1] || -40;
                      const tcpU = u + dx;
                      const tcpV = v + dy;
                      const tooltipX = Math.max(8, Math.min(tcpU + 8, (natSize?.w || 1280) - 190));
                      const tooltipY = Math.max(8, Math.min(tcpV - 72, (natSize?.h || 800) - 76));

                      return (
                        <g transform={`translate(${tooltipX}, ${tooltipY})`} pointerEvents="none">
                          <rect
                            x={0}
                            y={0}
                            width={182}
                            height={68}
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
                  onMouseMove={(e) => {
                    if (!natSize || isPanning) return;
                    const rect = e.currentTarget.getBoundingClientRect();
                    const u = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
                    const v = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);
                    setMousePixel({ u, v });
                    fetchLiveNormal(u, v);
                  }}
                  onMouseLeave={() => {
                    setMousePixel(null);
                    setHoveredWaypoint(null);
                  }}
                >
                  <defs>
                    <marker id="edit-traj-arrow" markerWidth="5.5" markerHeight="5.5" refX="4.2" refY="2.75" orient="auto">
                      <path d="M0,0.6 L0,4.9 L4.9,2.75 z" fill="#f59e0b" />
                    </marker>
                    <marker id="normal-arrow" markerWidth="4" markerHeight="4" refX="3.2" refY="2" orient="auto">
                      <path d="M0,0.6 L0,3.4 L3.4,2 z" fill="#ef4444" />
                    </marker>
                  </defs>

                  {/* 1. Committed Paths */}
                  {manualPaths.map((path, pIdx) => {
                    const pts = path.points;
                    return (
                      <g key={`committed-${pIdx}`} opacity={0.65}>
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
                              stroke="#f59e0b"
                              strokeWidth={2}
                              markerEnd="url(#edit-traj-arrow)"
                            />
                          );
                        })}
                        {pts.map((pt, idx) => (
                          <g key={idx}>
                            <circle cx={pt.pixel[0]} cy={pt.pixel[1]} r={5.5} fill="#f59e0b" stroke="white" strokeWidth={1} />
                            <text x={pt.pixel[0]} y={pt.pixel[1] + 0.5} textAnchor="middle" dominantBaseline="central" fill="white" fontSize={7.5} fontWeight="bold">
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
                      <line
                        x1={mousePixel.u}
                        y1={mousePixel.v}
                        x2={mousePixel.u + (liveNormal?.dx || 15)}
                        y2={mousePixel.v + (liveNormal?.dy || -40)}
                        stroke="#ef4444"
                        strokeWidth={2}
                        strokeLinecap="round"
                        markerEnd="url(#normal-arrow)"
                      />
                      {/* Aiming Cursor Target */}
                      <circle cx={mousePixel.u} cy={mousePixel.v} r={4.5} fill="#f59e0b" />
                      <circle cx={mousePixel.u} cy={mousePixel.v} r={8.5} fill="none" stroke="#f59e0b" strokeWidth={1.5} opacity={0.7} />
                    </g>
                  )}

                  {/* 3. Render Active Current Waypoints & Red Normal Vectors */}
                  {currentManualPoints.map((pt, idx) => {
                    const [u, v] = pt.pixel;
                    const dx = pt.normal_2d_proj?.[0] || 15;
                    const dy = pt.normal_2d_proj?.[1] || -40;
                    const tcpU = u + dx;
                    const tcpV = v + dy;
                    const isHovered = hoveredWaypoint?.index === pt.index;

                    return (
                      <g 
                        key={`current-${idx}`}
                        onMouseEnter={() => setHoveredWaypoint(pt)}
                        onMouseLeave={() => setHoveredWaypoint(null)}
                      >
                        {/* Red Surface Normal Arrow pointing from surface to TCP (Sleek) */}
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

                        {/* Physical Surface Point Circle (Dark Green) */}
                        <circle cx={u} cy={v} r={8.5} fill="white" filter="drop-shadow(0px 0px 4px rgba(0,0,0,0.9))" />
                        <circle cx={u} cy={v} r={6.5} fill="#f59e0b" />
                        <text x={u} y={v + 0.5} textAnchor="middle" dominantBaseline="central" fill="white" fontSize={8.5} fontWeight="bold">
                          {idx + 1}
                        </text>

                        {/* Sleek, Non-Intrusive Mini Waypoint Pill at Normal End */}
                        <g transform={`translate(${tcpU + 4}, ${tcpV - 7})`}>
                          <rect
                            x={0}
                            y={0}
                            width={32}
                            height={14}
                            rx={3}
                            fill="rgba(2, 6, 23, 0.85)"
                            stroke={isHovered ? "#f59e0b" : "rgba(245, 158, 11, 0.5)"}
                            strokeWidth={1}
                          />
                          <text x={16} y={7.5} textAnchor="middle" dominantBaseline="central" fill="#fcd34d" fontSize={7.5} fontFamily="monospace" fontWeight="bold">
                            P{idx + 1}
                          </text>
                        </g>
                      </g>
                    );
                  })}

                  {/* Topmost Tooltip Layer in EDIT mode — rendered at the very end of SVG */}
                  {manualPathMode && hoveredWaypoint && (
                    (() => {
                      const pt = hoveredWaypoint;
                      const [u, v] = pt.pixel;
                      const dx = pt.normal_2d_proj?.[0] || 15;
                      const dy = pt.normal_2d_proj?.[1] || -40;
                      const tcpU = u + dx;
                      const tcpV = v + dy;
                      const tooltipX = Math.max(8, Math.min(tcpU + 8, (natSize?.w || 1280) - 190));
                      const tooltipY = Math.max(8, Math.min(tcpV - 72, (natSize?.h || 800) - 76));

                      return (
                        <g transform={`translate(${tooltipX}, ${tooltipY})`} pointerEvents="none">
                          <rect
                            x={0}
                            y={0}
                            width={182}
                            height={68}
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
              
              {/* Path Indicator */}
              <div className="flex items-center gap-1.5 px-0.5 text-[10px] text-slate-300 border-r border-white/10 pr-2">
                <span className="flex items-center gap-1 text-rose-400 font-medium">
                  <Route size={12} className="text-rose-400" />
                  Manual #{manualPaths.length + 1}
                </span>
                <span className="text-slate-400 font-mono">({currentManualPoints.length} pts)</span>
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
             <div className="text-center text-[9px] text-slate-500">Select template</div>
           )}
        </div>
      </div>
    </div>
  );
};

export default InteractiveOp;
