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
  Box
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
}

const InteractiveOp: React.FC<InteractiveOpProps> = ({
  externalActiveTemplate,
  onTemplateChange,
  onMeshUpdated
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

  const fetchFiles = async (templateName: string) => {
    try {
      const res = await fetch(`http://localhost:8000/api/interactive/templates/${templateName}/files`);
      if (res.ok) {
        const data = await res.json();
        // Ensure sorted by ctime descending (newest first)
        const sorted = (data.files || []).sort((a: FileItem, b: FileItem) => b.ctime - a.ctime);
        setFiles(sorted);
        setImageVersion(Date.now());
        
        // Load existing masks from masks.yaml
        fetchSavedMasks(templateName);
      }
    } catch (err) {
      console.error('Failed to fetch template files:', err);
    }
  };

  useEffect(() => {
    if (activeTemplate) {
      fetchFiles(activeTemplate);
      // Reset seg mode and view transforms
      setSegMode(false);
      setCurrentPoints([]);
      setCurrentPolygons([]);
      setCommittedMasks([]);
      setZoom(1);
      setPan({ x: 0, y: 0 });
    } else {
      setFiles([]);
      setSavedMasks([]);
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
    if (filename === 'scan.params.yaml') return { label: 'PARAMS', color: 'bg-amber-500/20 text-amber-300 border-amber-500/40' };
    if (filename === 'scan.pcd') return { label: '3D PCD', color: 'bg-cyan-500/20 text-cyan-300 border-cyan-500/40' };
    if (filename === 'scan.depth.npy') return { label: 'DEPTH', color: 'bg-indigo-500/20 text-indigo-300 border-indigo-500/40' };
    if (filename === 'scan.jpg') return { label: 'IMAGE', color: 'bg-blue-500/20 text-blue-300 border-blue-500/40' };
    return null;
  };

  const getFileIcon = (filename: string) => {
    if (filename.includes('.ply') || filename.includes('.stl')) return <Box size={13} className="text-purple-400" />;
    if (filename.endsWith('.jpg') || filename.endsWith('.png')) return <ImageIcon size={13} className="text-blue-400" />;
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
                      <text x={p.x} y={p.y + 0.5} textAnchor="middle" dominantBaseline="central" fill="white" fontSize="10" fontWeight="bold">
                        {p.label === 1 ? '+' : '-'}
                      </text>
                    </g>
                  ))}
                </svg>
              )}
            </div>
          ) : (
            <div className="w-full h-full flex flex-col items-center justify-center text-slate-600 gap-2">
              <ImageIcon size={48} className="opacity-20" />
              <span className="text-xs text-slate-500">No scan.jpg in this template</span>
            </div>
          )}

          {/* Loading Indicator for MobileSAM */}
          {isInitializingSam && (
            <div className="absolute inset-0 bg-slate-950/70 backdrop-blur-sm flex flex-col items-center justify-center text-blue-400 gap-3 z-40">
              <RefreshCw className="animate-spin text-blue-400" size={32} />
              <div className="flex flex-col items-center">
                <span className="font-semibold text-xs tracking-wide text-slate-100">Encoding Image with MobileSAM</span>
                <span className="text-[11px] text-slate-400 mt-0.5">Preparing interactive session...</span>
              </div>
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

          {/* Top Right: Semi-transparent View Status Badges & Glassmorphism Mask Switch (Consistent Height h-7) */}
          {!segMode && (
            <div className="absolute top-3 right-3 flex items-center gap-2 z-20">
              {/* Glassmorphic Mask Toggle Switch with Unified Tooltip */}
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

                  {/* iOS-style Mini Switch Knob */}
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

                {/* Unified Dark Glassmorphism Tooltip */}
                <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    {showMasksOverlay ? "Click to Hide Masks" : "Click to Show Masks"} · {savedMasks.length} Mask{savedMasks.length > 1 ? 's' : ''} in YAML
                  </div>
                </div>
              </div>

              {/* Resolution Badge with Unified Height (h-7) & Tooltip */}
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
          
          {/* SEGMENTATION FLOATING TOOLBAR: Semi-transparent Frosted Glass with Unified Tooltips */}
          {segMode && !isInitializingSam && (
            <div className="absolute bottom-4 left-1/2 -translate-x-1/2 bg-slate-950/45 hover:bg-slate-950/65 backdrop-blur-md border border-white/15 rounded-full px-3 py-1.5 flex items-center gap-2 shadow-2xl z-30 transition-all">
              
              {/* Click Indicator */}
              <div className="flex items-center gap-2 px-1 text-[10px] text-slate-300 border-r border-white/10 pr-2.5">
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
                  className="p-1.5 text-slate-300 hover:text-white hover:bg-white/10 rounded-full disabled:opacity-25 transition-colors"
                >
                  <Undo2 size={16} />
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
                  className="p-1.5 text-emerald-400 hover:text-emerald-300 hover:bg-emerald-400/20 rounded-full disabled:opacity-25 transition-colors"
                >
                  <Check size={17} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Commit Object (n)
                  </div>
                </div>
              </div>

              <div className="w-px h-4 bg-white/10" />

              {/* Reset Current Object Button with Tooltip */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleResetCurrent} 
                  disabled={currentPoints.length === 0} 
                  className="p-1.5 text-amber-400 hover:text-amber-300 hover:bg-amber-400/20 rounded-full disabled:opacity-25 transition-colors"
                >
                  <RefreshCw size={15} />
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
                  className="p-1.5 text-rose-400 hover:text-rose-300 hover:bg-rose-400/20 rounded-full disabled:opacity-25 transition-colors"
                >
                  <Trash2 size={15} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Clear All Masks (c)
                  </div>
                </div>
              </div>

              <div className="w-px h-4 bg-white/10" />

              {/* Save Masks Button with Tooltip */}
              <div className="relative group flex items-center justify-center">
                <button 
                  onClick={handleSaveMasks} 
                  disabled={committedMasks.length === 0} 
                  className="p-1.5 bg-blue-600/80 hover:bg-blue-600 text-white rounded-full shadow-lg shadow-blue-900/30 border border-blue-400/30 disabled:opacity-25 transition-all active:scale-95"
                >
                  <Save size={15} />
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
                  className="p-1.5 text-slate-400 hover:text-slate-200 hover:bg-white/10 rounded-full transition-colors"
                >
                  <X size={16} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap">
                    Exit Seg Mode
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
                        const isMesh = file.name.includes('.ply') || file.name.includes('.stl');
                        return (
                          <div 
                            key={file.name}
                            className={`p-1.5 rounded-lg border transition-all flex flex-col gap-0.5 ${
                              isMaskYaml 
                                ? 'bg-emerald-950/25 border-emerald-500/40 shadow-sm' 
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
                                  isMaskYaml ? 'text-emerald-300' : (isMesh ? 'text-purple-300' : 'text-slate-200')
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
               disabled={isCapturing || isReconstructing || !activeTemplate || segMode}
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
               disabled={!hasImage || isCapturing || isReconstructing}
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

           {/* Surface Reconstruction Button */}
           <div className="relative group w-full">
             <button 
               onClick={handleReconstruct}
               disabled={!hasImage || !hasMasks || !hasDepth || isCapturing || isReconstructing || segMode}
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
