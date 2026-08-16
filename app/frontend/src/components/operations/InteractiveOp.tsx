import React, { useState, useEffect, type MouseEvent } from 'react';
import { CustomModal, type ModalConfig } from '../common/CustomModal';

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
} from './interactive/types';
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
}

const InteractiveOp: React.FC<InteractiveOpProps> = ({
  externalActiveTemplate,
  onTemplateChange,
  onMeshUpdated,
  onPathsUpdated,
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

  // ─── 4. Manual TCP Path State ───────────────────────────────────────────
  const [manualPathMode, setManualPathMode] = useState<boolean>(false);
  const [manualPaths, setManualPaths] = useState<ManualPathItem[]>([]);
  const [rawPaths, setRawPaths] = useState<ManualPathItem[]>([]);
  const [optPaths, setOptPaths] = useState<ManualPathItem[]>([]);
  const [usingOptimizedPaths, setUsingOptimizedPaths] = useState<boolean>(false);
  const [currentManualPoints, setCurrentManualPoints] = useState<WaypointItem[]>([]);
  const [selectedPathIdForEdit, setSelectedPathIdForEdit] = useState<number | null>(null);
  const [standoffDistMm, setStandoffDistMm] = useState<number>(150.0);
  const [showManualPathsOverlay, setShowManualPathsOverlay] = useState<boolean>(true);
  const [hoveredWaypoint, setHoveredWaypoint] = useState<WaypointItem | null>(null);
  const [highlightedPathId, setHighlightedPathId] = useState<number | null>(null);
  const [sessionData, setSessionData] = useState<SessionData | null>(null);
  const [mousePixel, setMousePixel] = useState<{ u: number; v: number } | null>(null);
  const [liveNormal, setLiveNormal] = useState<LiveNormalInfo | null>(null);

  // ─── 5. Diagnostics & Verification State ────────────────────────────────
  const [showDiagnostics, setShowDiagnostics] = useState<boolean>(false);
  const [verificationReport, setVerificationReport] = useState<VerificationReport | null>(null);
  const [isVerifying, setIsVerifying] = useState<boolean>(false);
  const [isOptimizing, setIsOptimizing] = useState<boolean>(false);
  const [isKinParamsOpen, setIsKinParamsOpen] = useState<boolean>(true);
  const [urdfTcpInfo, setUrdfTcpInfo] = useState<UrdfTcpInfo | null>(null);
  const [kinParams, setKinParams] = useState<KinematicsParams>({
    stepSizeMm: 1.5,
    linearSpeedMmS: 120.0,
  });

  // ─── 6. Action Execution State ──────────────────────────────────────────
  const [isCapturing, setIsCapturing] = useState<boolean>(false);
  const [isReconstructing, setIsReconstructing] = useState<boolean>(false);

  // ─── 7. Modal Dialog Config ─────────────────────────────────────────────
  const [modalConfig, setModalConfig] = useState<ModalConfig | null>(null);

  // Sync external active template
  useEffect(() => {
    if (externalActiveTemplate && externalActiveTemplate !== activeTemplate) {
      handleSelectTemplate(externalActiveTemplate);
    }
  }, [externalActiveTemplate]);

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
      const res = await fetch('http://localhost:8000/api/interactive/templates');
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
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${templateName}/summary`);
      if (!res.ok) throw new Error(`Summary status ${res.status}`);
      const summary = await res.json();

      setFiles(summary.files || []);
      setHasImage(summary.has_image || false);
      setSavedMasks(summary.masks || []);

      const raw = summary.raw_paths || [];
      const opt = summary.opt_paths || [];
      setRawPaths(raw);
      setOptPaths(opt);
      setManualPaths(opt.length > 0 ? opt : raw);
      setUsingOptimizedPaths(opt.length > 0);

      if (summary.standoff_distance_mm) {
        setStandoffDistMm(summary.standoff_distance_mm);
      }

      if (summary.urdf_tcp) {
        setUrdfTcpInfo(summary.urdf_tcp);
      }

      const activeReport = opt.length > 0 ? summary.opt_report : summary.raw_report;
      setVerificationReport(activeReport || null);

      if (summary.has_image) {
        const newImgUrl = `http://localhost:8000/templates/${templateName}/scan.jpg?v=${Date.now()}`;
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

      // Load session depth data for client-side normal computation
      loadSessionData(templateName);
    } catch (err) {
      console.error('Failed to load template atomic summary:', err);
    } finally {
      setIsLoadingTemplate(false);
    }
  };

  const loadSessionData = async (templateName: string) => {
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${templateName}/session_data`);
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
      console.warn('Failed to load session depth data, server-side sampling fallback active:', err);
    }
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
      await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/sam/init`, {
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
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/sam/predict`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          points: pts.map((p) => [p.x, p.y]),
          labels: pts.map((p) => p.label),
        }),
      });
      if (!res.ok) return;
      const data = await res.json();
      setCurrentPolygons(data.polygons || []);
    } catch (err) {
      console.error('Failed to predict SAM mask:', err);
    }
  };

  const handleCommitCurrentSegMask = () => {
    if (currentPolygons.length === 0) return;
    setCommittedMasks([
      ...committedMasks,
      { id: committedMasks.length + 1, points: currentPoints, polygons: currentPolygons },
    ]);
    setCurrentPoints([]);
    setCurrentPolygons([]);
  };

  const fetchTemplateFiles = async (templateName: string) => {
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${templateName}/files`);
      if (res.ok) {
        const data = await res.json();
        setFiles(data.files || []);
      }
    } catch (err) {
      console.error('Failed to fetch template files:', err);
    }
  };

  const handleSaveAllSegMasks = async () => {
    if (!activeTemplate || committedMasks.length === 0) return;
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/sam/save`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          committed_masks: committedMasks.map((m) => ({
            points: m.points ? m.points.map((p) => [p.x, p.y]) : [],
            labels: m.points ? m.points.map((p) => p.label) : [],
          })),
        }),
      });
      if (!res.ok) throw new Error('Failed to save masks');
      setSavedMasks(committedMasks);
      setSegMode(false);
      await fetchTemplateFiles(activeTemplate);
      setModalConfig({
        isOpen: true,
        title: 'Success',
        message: 'Masks saved successfully to scan.masks.yaml',
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

  // ─── Manual TCP Path Designer Handlers ──────────────────────────────────
  const handleManualMouseMove = (e: MouseEvent<SVGSVGElement>) => {
    if (!natSize) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const u = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const v = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);

    if (u < 0 || u >= natSize.w || v < 0 || v >= natSize.h) {
      setMousePixel(null);
      setLiveNormal(null);
      return;
    }

    setMousePixel({ u, v });
    if (sessionData) {
      const norm = computeNormalClientSide(sessionData, u, v, standoffDistMm);
      if (norm) {
        setLiveNormal({
          dx: norm.proj_dx,
          dy: norm.proj_dy,
          surfPointBase: norm.surf_base,
          normalBase: norm.normal_base,
          tcpPose: {
            x: norm.tcp_base[0],
            y: norm.tcp_base[1],
            z: norm.tcp_base[2],
            rx: norm.euler_deg[0],
            ry: norm.euler_deg[1],
            rz: norm.euler_deg[2],
          },
        });
      } else {
        setLiveNormal(null);
      }
    }
  };

  const handleManualImageClick = async (e: MouseEvent<SVGSVGElement>) => {
    if (!manualPathMode || !natSize || !activeTemplate || isSpacePressed) return;
    e.preventDefault();

    const rect = e.currentTarget.getBoundingClientRect();
    const u = Math.round(((e.clientX - rect.left) / rect.width) * natSize.w);
    const v = Math.round(((e.clientY - rect.top) / rect.height) * natSize.h);

    // 1. Client-side instant calculation if sessionData is ready
    if (sessionData) {
      const norm = computeNormalClientSide(sessionData, u, v, standoffDistMm);
      if (norm) {
        const newWp: WaypointItem = {
          index: currentManualPoints.length + 1,
          pixel: [u, v],
          surface_point_cam_mm: norm.surf_cam,
          surface_point_base_mm: norm.surf_base,
          surface_normal_base: norm.normal_base,
          surface_normal_cam: norm.normal_cam,
          standoff_distance_mm: standoffDistMm,
          tcp_pose_base: {
            x: norm.tcp_base[0],
            y: norm.tcp_base[1],
            z: norm.tcp_base[2],
            rx: norm.euler_deg[0],
            ry: norm.euler_deg[1],
            rz: norm.euler_deg[2],
          },
          normal_2d_proj: [norm.proj_dx, norm.proj_dy],
        };
        setCurrentManualPoints((prev) => [...prev, newWp]);
        return;
      }
    }

    // 2. Server-side fallback sampling
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/sample_point`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ u, v, standoff_dist_mm: standoffDistMm }),
      });
      if (!res.ok) throw new Error('Failed to sample point');
      const pointData = await res.json();
      const newWp: WaypointItem = {
        index: currentManualPoints.length + 1,
        pixel: [u, v],
        surface_point_cam_mm: pointData.surface_point_cam_mm,
        surface_point_base_mm: pointData.surface_point_base_mm,
        surface_normal_base: pointData.surface_normal_base,
        surface_normal_cam: pointData.surface_normal_cam,
        standoff_distance_mm: standoffDistMm,
        tcp_pose_base: pointData.tcp_pose_base,
        normal_2d_proj: pointData.normal_2d_proj,
      };
      setCurrentManualPoints((prev) => [...prev, newWp]);
    } catch (err) {
      setModalConfig({
        isOpen: true,
        title: 'Invalid Waypoint',
        message: 'No valid 3D depth found at this pixel location. Please click on the scanned surface.',
        type: 'alert',
      });
    }
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
      setManualPaths([
        ...manualPaths,
        {
          path_id: newPathId,
          name: `Path_${newPathId}`,
          points: currentManualPoints,
        },
      ]);
    }
    setCurrentManualPoints([]);
  };

  const handleSaveManualPaths = async () => {
    if (!activeTemplate) return;
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/manual_paths`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          paths: manualPaths,
          standoff_distance_mm: standoffDistMm,
        }),
      });
      if (!res.ok) throw new Error('Failed to save manual paths');
      setRawPaths(manualPaths);
      setManualPathMode(false);
      await fetchTemplateFiles(activeTemplate);
      if (onPathsUpdated) onPathsUpdated();
      setModalConfig({
        isOpen: true,
        title: 'Success',
        message: `Saved ${manualPaths.length} TCP paths successfully.`,
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

  // ─── Diagnostics & Auto-Fix Handlers ────────────────────────────────────
  const handleRunDiagnostics = async () => {
    if (!activeTemplate) return;
    setIsVerifying(true);
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/verify_paths`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          use_opt: usingOptimizedPaths,
          options: {
            step_size_mm: kinParams.stepSizeMm,
            linear_velocity_mm_s: kinParams.linearSpeedMmS,
          },
        }),
      });
      if (!res.ok) throw new Error('Verification failed');
      const data = await res.json();
      setVerificationReport(data);
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

  const handleApplyOptimization = async () => {
    if (!activeTemplate) return;
    setIsOptimizing(true);
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/optimize_paths`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          options: {
            step_size_mm: kinParams.stepSizeMm,
            linear_velocity_mm_s: kinParams.linearSpeedMmS,
          },
        }),
      });
      if (!res.ok) throw new Error('Optimization failed');
      const data = await res.json();
      setOptPaths(data.optimized_paths || []);
      setManualPaths(data.optimized_paths || []);
      setUsingOptimizedPaths(true);
      setVerificationReport(data.verification_report || null);
      setShowDiagnostics(true);
      if (onPathsUpdated) onPathsUpdated();
      setModalConfig({
        isOpen: true,
        title: 'Optimization Applied',
        message: 'Path orientations auto-fixed using axial rotation tolerance.',
        type: 'alert',
      });
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

  const handleToggleUseOptimized = (useOpt: boolean) => {
    setUsingOptimizedPaths(useOpt);
    setManualPaths(useOpt ? optPaths : rawPaths);
    if (onPathsUpdated) onPathsUpdated();
  };

  // ─── Actions: Capture & Reconstruct ─────────────────────────────────────
  const handleTriggerCapture = async () => {
    if (!activeTemplate) return;
    setIsCapturing(true);
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/capture`, {
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
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/reconstruct`, {
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
                const res = await fetch('http://localhost:8000/api/interactive/templates', {
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
                const res = await fetch(`http://localhost:8000/api/interactive/templates/${t}`, {
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
          verificationReport={verificationReport}
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
                        const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/files/${f}`, {
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
              />
            ) : (
              <DiagnosticsDashboard
                verificationReport={verificationReport}
                isVerifying={isVerifying}
                isOptimizing={isOptimizing}
                activeTemplate={activeTemplate}
                optPaths={optPaths}
                usingOptimizedPaths={usingOptimizedPaths}
                hasPaths={manualPaths.length > 0}
                kinParams={kinParams}
                urdfTcpInfo={urdfTcpInfo}
                isKinParamsOpen={isKinParamsOpen}
                highlightedPathId={highlightedPathId}
                setKinParams={setKinParams}
                setIsKinParamsOpen={setIsKinParamsOpen}
                setHighlightedPathId={setHighlightedPathId}
                onRunDiagnostics={handleRunDiagnostics}
                onApplyOptimization={handleApplyOptimization}
                onToggleUseOptimized={handleToggleUseOptimized}
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
