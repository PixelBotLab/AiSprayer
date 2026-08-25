import React, { useState, useEffect } from 'react';
import {
  HardDrive,
  FileCode2,
  Camera,
  Layers,
  Route,
  Box,
  Image as ImageIcon,
  Trash2,
  Play,
  Zap,
  Sparkles,
  CheckCheck,
  RefreshCw,
  CheckCircle2,
  AlertTriangle,
} from 'lucide-react';
import type { FileItem, PathStateType, ManualPathItem, VerificationReport } from './types';

interface TemplateFileListProps {
  files: FileItem[];
  activeState: PathStateType;
  rawPaths?: ManualPathItem[];
  autoPaths?: ManualPathItem[];
  autoPoiPaths?: ManualPathItem[];
  poiPaths?: ManualPathItem[];
  rawReport?: VerificationReport | null;
  autoReport?: VerificationReport | null;
  poiReport?: VerificationReport | null;
  autoPoiReport?: VerificationReport | null;
  robotConnected?: boolean;
  isVerifying?: boolean;
  isOptimizing?: boolean;
  onDeleteFile: (f: string) => void;
  onVerifyPath: (state: PathStateType) => void;
  onOptimizePath: (state: PathStateType) => void;
  onSimulatePath?: (state: PathStateType, pathId?: number | null) => void;
  onExecutePath?: (fileName: string, state?: PathStateType, pathId?: number | null) => void;
  onSelectFile?: (fileName: string) => void;
}

export const TemplateFileList: React.FC<TemplateFileListProps> = ({
  files,
  activeState,
  rawPaths = [],
  autoPaths = [],
  autoPoiPaths = [],
  poiPaths = [],
  rawReport = null,
  autoReport = null,
  poiReport = null,
  autoPoiReport = null,
  robotConnected = false,
  isVerifying = false,
  isOptimizing = false,
  onDeleteFile,
  onVerifyPath,
  onOptimizePath,
  onSimulatePath,
  onExecutePath,
  onSelectFile,
}) => {
  const [contextMenu, setContextMenu] = useState<{ x: number; y: number; fileName: string; state?: PathStateType } | null>(null);
  const [selectedFile, setSelectedFile] = useState<string | null>(null);

  // Close menus on outside click
  useEffect(() => {
    const handleOutside = () => {
      setContextMenu(null);
    };
    window.addEventListener('click', handleOutside);
    return () => window.removeEventListener('click', handleOutside);
  }, []);

  useEffect(() => {
    if (selectedFile && !files.some((f) => f.name === selectedFile)) {
      setSelectedFile(null);
    }
  }, [files, selectedFile]);

  const getPathsForFile = (fileName: string) => {
    if (fileName.includes('auto.poi')) return autoPoiPaths || [];
    if (fileName.includes('manual.poi') || fileName.includes('poi.path')) return poiPaths || [];
    if (fileName.includes('auto.path')) return autoPaths || [];
    if (fileName.includes('manual.path') || fileName.includes('raw.path') || fileName.includes('manual_paths')) return rawPaths || [];
    return [];
  };

  const getFileState = (fileName: string): PathStateType | null => {
    if (fileName.includes('auto.poi')) return 'auto_poi';
    if (fileName.includes('manual.poi') || fileName.includes('poi.path')) return 'poi';
    if (fileName.includes('auto.path')) return 'auto';
    if (fileName.includes('manual.path') || fileName.includes('raw.path') || fileName.includes('manual_paths')) return 'raw';
    return null;
  };

  const isPathYaml = (fileName: string) =>
    fileName.endsWith('.path.yaml') || fileName.includes('paths.yaml');

  const reportForState = (st: PathStateType | null) => {
    if (st === 'poi') return poiReport;
    if (st === 'auto_poi') return autoPoiReport;
    if (st === 'auto') return autoReport;
    if (st === 'raw') return rawReport;
    return null;
  };

  const hasRunnablePaths = (fileName: string) =>
    getPathsForFile(fileName).some((p) => (p.points?.length || 0) > 0);

  const hasSimTrajectory = (st: PathStateType | null) => {
    const reports = reportForState(st)?.path_reports || [];
    return reports.some((pr) => (pr.trajectory_q?.length || 0) > 0 && (pr.trajectory_tcp?.length || 0) > 0);
  };

  const isStateVerifiedPass = (st: PathStateType | null) => {
    const rep = reportForState(st);
    return !!rep && (rep.summary?.status === 'PASS' || rep.status === 'PASS');
  };

  const isStateVerifiedFailed = (st: PathStateType | null) => {
    const rep = reportForState(st);
    return !!rep && (rep.summary?.status === 'FAILED' || rep.status === 'FAILED');
  };

  const canSimulateFile = (fileName: string | null) => {
    if (!fileName || !isPathYaml(fileName)) return false;
    const st = getFileState(fileName);
    return !!st && hasRunnablePaths(fileName) && isStateVerifiedPass(st) && hasSimTrajectory(st);
  };

  const canExecuteFile = (fileName: string | null) => {
    if (!fileName || !isPathYaml(fileName) || !robotConnected) return false;
    const st = getFileState(fileName);
    return !!st && hasRunnablePaths(fileName) && isStateVerifiedPass(st) && hasSimTrajectory(st);
  };

  // Determine current effective state for top bar actions
  const effectiveState: PathStateType = (selectedFile ? getFileState(selectedFile) : null) || activeState;
  const currentPaths = getPathsForFile(effectiveState === 'auto_poi' ? 'auto.poi' : (effectiveState === 'poi' ? 'manual.poi' : (effectiveState === 'auto' ? 'auto.path' : 'manual.path')));
  const hasWaypoints = currentPaths.some((p) => (p.points?.length || 0) > 0);
  const isPass = isStateVerifiedPass(effectiveState);
  const isFailed = isStateVerifiedFailed(effectiveState);
  const hasSim = hasSimTrajectory(effectiveState);

  // Validation: Simulation requires valid trajectory & not failed
  const canSim = hasWaypoints && isPass && hasSim;

  // Validation: Physical robot execution strictly requires robot connected + waypoints + verification PASS + trajectory
  const canExec = robotConnected && hasWaypoints && isPass && hasSim;

  const simTooltip = !hasWaypoints
    ? 'No Waypoints to Simulate'
    : isFailed
    ? 'Simulation Blocked: Kinematics Verification FAILED'
    : !isPass || !hasSim
    ? 'Please Verify Path First to Generate Simulation'
    : `Play 3D & 2D Simulation (${effectiveState.toUpperCase()} PASS)`;

  const execTooltip = !robotConnected
    ? 'Robot Not Connected'
    : !hasWaypoints
    ? 'No Waypoints to Execute'
    : isFailed
    ? 'Execution Blocked: Kinematics Verification FAILED'
    : !isPass || !hasSim
    ? 'Execution Blocked: Verify Kinematics First (PASS Required)'
    : `Execute MoveL on Robot (${effectiveState.toUpperCase()} PASS)`;

  const getFileIcon = (fileName: string) => {
    if (fileName.endsWith('.jpg') || fileName.endsWith('.png'))
      return <ImageIcon size={12} className="text-emerald-400 shrink-0" />;
    if (fileName.endsWith('.yaml') || fileName.endsWith('.yml'))
      return <Route size={12} className="text-sky-400 shrink-0" />;
    if (fileName.endsWith('.ply') || fileName.endsWith('.stl'))
      return <Box size={12} className="text-purple-400 shrink-0" />;
    return <FileCode2 size={12} className="text-slate-400 shrink-0" />;
  };

  const getFileBadge = (fileName: string) => {
    if (fileName === 'scan.color.jpg' || fileName === 'scan.jpg') return { label: 'RGB', color: 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30' };
    if (fileName === 'scan.depth.png' || fileName === 'scan.depth.npy' || fileName === 'scan.depth.bin') return { label: 'DEPTH', color: 'bg-cyan-500/10 text-cyan-400 border-cyan-500/30' };
    if (fileName === 'scan.masks.yaml') return { label: 'MASKS', color: 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30' };
    if (fileName.includes('auto.poi.path')) return { label: 'AUTO POI', color: 'bg-teal-500/20 text-teal-300 border-teal-500/40' };
    if (fileName.includes('manual.poi.path') || fileName.includes('poi.path')) return { label: 'MANUAL POI', color: 'bg-emerald-500/20 text-emerald-400 border-emerald-500/40' };
    if (fileName.includes('auto.path')) return { label: 'AUTO', color: 'bg-violet-500/20 text-violet-300 border-violet-500/40' };
    if (fileName.includes('manual.path') || fileName.includes('raw.path') || fileName.includes('manual_paths')) return { label: 'MANUAL', color: 'bg-slate-500/20 text-slate-300 border-slate-500/40' };
    if (fileName.endsWith('.ply') || fileName.endsWith('.stl')) return { label: '3D MESH', color: 'bg-purple-500/10 text-purple-400 border-purple-500/30' };
    return null;
  };

  const formatFileSize = (bytes: number) => {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  };

  const formatFileDate = (timestamp: number) => {
    if (!timestamp) return '';
    const date = new Date(timestamp * 1000);
    return `${date.getHours().toString().padStart(2, '0')}:${date.getMinutes().toString().padStart(2, '0')}`;
  };

  // Filter out any standalone .report.json files since they are unified into .path.yaml
  const validFiles = files.filter(
    (f) => !f.name.endsWith('.report.json') && !f.name.endsWith('.report.yaml')
  );

  const fileCategories = [
    {
      id: 'paths',
      title: 'Path Trajectories',
      icon: Route,
      iconColor: 'text-sky-400',
      badgeColor: 'bg-sky-500/10 text-sky-400 border-sky-500/30',
      files: validFiles.filter((f) => f.name.includes('path')),
    },
    {
      id: 'capture',
      title: 'Capture & Depth',
      icon: Camera,
      iconColor: 'text-blue-400',
      badgeColor: 'bg-blue-500/10 text-blue-400 border-blue-500/30',
      files: validFiles.filter((f) => ['scan.color.jpg', 'scan.jpg', 'scan.depth.png', 'scan.depth.npy', 'scan.depth.bin', 'scan.params.yaml'].includes(f.name)),
    },
    {
      id: 'segment',
      title: 'Segment Masks',
      icon: Layers,
      iconColor: 'text-emerald-400',
      badgeColor: 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30',
      files: validFiles.filter((f) => ['scan.masks.yaml', 'scan.masks.jpg'].includes(f.name)),
    },
    {
      id: 'reconstruct',
      title: 'Reconstruction 3D',
      icon: Box,
      iconColor: 'text-purple-400',
      badgeColor: 'bg-purple-500/10 text-purple-400 border-purple-500/30',
      files: validFiles.filter((f) => f.name.startsWith('scan.mesh') || f.name.includes('.ply') || f.name.includes('.stl')),
    },
    {
      id: 'other',
      title: 'Other Files',
      icon: FileCode2,
      iconColor: 'text-slate-400',
      badgeColor: 'bg-slate-800 text-slate-400 border-slate-700',
      files: validFiles.filter(
        (f) =>
          !f.name.includes('path') &&
          !['scan.color.jpg', 'scan.jpg', 'scan.depth.png', 'scan.depth.npy', 'scan.depth.bin', 'scan.params.yaml', 'scan.masks.yaml', 'scan.masks.jpg'].includes(f.name) &&
          !f.name.startsWith('scan.mesh') &&
          !f.name.includes('.ply') &&
          !f.name.includes('.stl')
      ),
    },
  ];

  return (
    <div className="w-full flex flex-col h-full min-h-0 select-none bg-slate-950/40 relative">
      {/* File List Header with Action Buttons */}
      <div className="h-9 px-2.5 border-b border-slate-800 bg-slate-900/90 flex justify-between items-center shrink-0">
        <div className="flex items-center gap-1.5 text-[11px] text-slate-300 font-medium">
          <HardDrive size={13} className="text-slate-400" />
          <span>Files</span>
        </div>
        <div className="flex items-center gap-1">
          {/* Button 1: Verify Path */}
          <div className="relative group flex items-center justify-center">
            <button
              type="button"
              disabled={isVerifying || isOptimizing || !hasWaypoints}
              onClick={() => onVerifyPath(effectiveState)}
              className={`w-6 h-6 rounded flex items-center justify-center transition-all ${
                !hasWaypoints
                  ? 'text-slate-600 cursor-not-allowed opacity-40'
                  : isVerifying
                  ? 'bg-sky-500/20 text-sky-400'
                  : 'text-sky-400 hover:bg-sky-500/20 hover:text-sky-300'
              }`}
            >
              {isVerifying ? <RefreshCw size={12} className="animate-spin text-sky-400" /> : <CheckCheck size={12} />}
            </button>
            <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                {hasWaypoints ? `Verify Kinematics & Trajectory (${effectiveState.toUpperCase()})` : 'No Waypoints to Verify'}
              </div>
            </div>
          </div>

          {/* Button 2: Optimize POI */}
          <div className="relative group flex items-center justify-center">
            <button
              type="button"
              disabled={isOptimizing || isVerifying || !hasWaypoints}
              onClick={() => onOptimizePath(effectiveState)}
              className={`w-6 h-6 rounded flex items-center justify-center transition-all ${
                !hasWaypoints
                  ? 'text-slate-600 cursor-not-allowed opacity-40'
                  : isOptimizing
                  ? 'bg-emerald-500/20 text-emerald-400'
                  : 'text-emerald-400 hover:bg-emerald-500/20 hover:text-emerald-300'
              }`}
            >
              {isOptimizing ? <RefreshCw size={12} className="animate-spin text-emerald-400" /> : <Sparkles size={12} />}
            </button>
            <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                {hasWaypoints ? `Optimize POI Orientations (${effectiveState.toUpperCase()})` : 'No Waypoints to Optimize'}
              </div>
            </div>
          </div>

          <div className="w-[1px] h-3 bg-slate-700 mx-0.5" />

          {/* Button 3: Simulate */}
          <div className="relative group flex items-center justify-center">
            <button
              type="button"
              disabled={!canSim}
              onClick={() => {
                if (canSim && onSimulatePath) onSimulatePath(effectiveState, null);
              }}
              className={`w-6 h-6 rounded flex items-center justify-center transition-all ${
                canSim ? 'text-sky-300 hover:bg-sky-500/20 hover:text-sky-200' : 'text-slate-600 cursor-not-allowed opacity-40'
              }`}
            >
              <Play size={11} className={canSim ? 'fill-sky-300' : ''} />
            </button>
            <div className="absolute top-full mt-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                {simTooltip}
              </div>
            </div>
          </div>

          {/* Button 4: Execute on Robot */}
          <div className="relative group flex items-center justify-center">
            <button
              type="button"
              disabled={!canExec}
              onClick={() => {
                const fileName = selectedFile || (effectiveState === 'auto_poi' ? 'scan.auto.poi.path.yaml' : (effectiveState === 'poi' ? 'scan.manual.poi.path.yaml' : (effectiveState === 'auto' ? 'scan.auto.path.yaml' : 'scan.manual.path.yaml')));
                if (canExec && onExecutePath) onExecutePath(fileName, effectiveState, null);
              }}
              className={`w-6 h-6 rounded flex items-center justify-center transition-all ${
                canExec ? 'text-emerald-300 hover:bg-emerald-500/20 hover:text-emerald-200' : 'text-slate-600 cursor-not-allowed opacity-40'
              }`}
            >
              <Zap size={11} />
            </button>
            <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-end pointer-events-none z-50">
              <div className="bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                {execTooltip}
              </div>
            </div>
          </div>

          <span className="bg-slate-800 text-slate-400 text-[9px] font-mono px-1.5 py-0.5 rounded border border-slate-700 ml-0.5">
            {validFiles.length}
          </span>
        </div>
      </div>

      {/* Files Category Scroll Container */}
      <div className="flex-1 overflow-y-auto overflow-x-hidden p-1.5 flex flex-col gap-2 custom-scrollbar">
        {validFiles.length === 0 ? (
          <div className="flex flex-col items-center justify-center h-32 text-slate-600 text-[11px] gap-1">
            <FileCode2 size={20} className="opacity-30" />
            <span>Empty</span>
          </div>
        ) : (
          fileCategories.map((cat) => {
            if (cat.files.length === 0) return null;
            const CatIcon = cat.icon;
            return (
              <div key={cat.id} className="flex flex-col gap-1">
                {/* Category Header */}
                <div className="flex items-center justify-between px-1 pt-1 pb-0.5 border-b border-slate-800/80">
                  <div className="flex items-center gap-1 text-[10px] font-semibold tracking-wider uppercase text-slate-400">
                    <CatIcon size={11} className={cat.iconColor} />
                    <span>{cat.title}</span>
                  </div>
                  <span className={`text-[8px] font-mono px-1 py-0.2 rounded border ${cat.badgeColor}`}>
                    {cat.files.length}
                  </span>
                </div>

                {/* File Row Items */}
                <div className="flex flex-col gap-1">
                  {cat.files.map((file) => {
                    const badge = getFileBadge(file.name);
                    const stateType = getFileState(file.name);
                    const isSelected = selectedFile === file.name;
                    const rep = reportForState(stateType);
                    const isVerified = rep && (rep.summary?.status === 'PASS' || rep.status === 'PASS');
                    const hasIssues = rep && (rep.summary?.status === 'FAILED' || rep.status === 'FAILED' || rep.summary?.status === 'WARNING');

                    return (
                      <div
                        key={file.name}
                        onClick={() => {
                          setSelectedFile(file.name);
                          if (onSelectFile) onSelectFile(file.name);
                        }}
                        onContextMenu={(e) => {
                          e.preventDefault();
                          setSelectedFile(file.name);
                          setContextMenu({
                            x: e.clientX,
                            y: e.clientY,
                            fileName: file.name,
                            state: stateType || undefined,
                          });
                        }}
                        className={`p-1.5 rounded-lg border transition-all flex flex-col gap-0.5 group relative cursor-pointer ${
                          isSelected
                            ? 'bg-slate-800/80 border-sky-500/40'
                            : 'bg-slate-900/60 border-slate-800/80 hover:border-slate-700 hover:bg-slate-800/60'
                        }`}
                      >
                        <div className="flex items-center justify-between gap-1">
                          <div className="flex items-center gap-1.5 min-w-0">
                            <div className="p-0.5 rounded bg-slate-800 shrink-0">
                              {getFileIcon(file.name)}
                            </div>
                            <span className="text-[10.5px] font-medium truncate text-slate-200" title={file.name}>
                              {file.name}
                            </span>
                          </div>

                          <div className="flex items-center gap-1 shrink-0">
                            {/* Verification Status Icon */}
                            {isVerified && (
                              <div className="relative group/status flex items-center justify-center">
                                <span className="text-emerald-400">
                                  <CheckCircle2 size={11} />
                                </span>
                                <div className="absolute bottom-full mb-1 hidden group-hover/status:flex flex-col items-center pointer-events-none z-50">
                                  <div className="bg-slate-950/80 backdrop-blur-md border border-emerald-500/30 rounded px-1.5 py-0.5 shadow text-[8px] text-emerald-300 whitespace-nowrap">
                                    Kinematics PASS
                                  </div>
                                </div>
                              </div>
                            )}
                            {hasIssues && (
                              <div className="relative group/status flex items-center justify-center">
                                <span className="text-amber-400">
                                  <AlertTriangle size={11} />
                                </span>
                                <div className="absolute bottom-full mb-1 hidden group-hover/status:flex flex-col items-center pointer-events-none z-50">
                                  <div className="bg-slate-950/80 backdrop-blur-md border border-amber-500/30 rounded px-1.5 py-0.5 shadow text-[8px] text-amber-300 whitespace-nowrap">
                                    {`Status: ${rep?.summary?.status || rep?.status || 'WARN'}`}
                                  </div>
                                </div>
                              </div>
                            )}

                            {(file.name.endsWith('.ply') || file.name.endsWith('.stl')) && (
                              <button
                                onClick={(e) => {
                                  e.stopPropagation();
                                  if (onSelectFile) onSelectFile(file.name);
                                }}
                                className="opacity-0 group-hover:opacity-100 px-1.5 py-0.5 rounded bg-purple-600 hover:bg-purple-500 text-white text-[9px] font-bold flex items-center gap-0.5 shadow transition-all"
                              >
                                <span>👁️ View</span>
                              </button>
                            )}

                            {(file.name.endsWith('.jpg') || file.name.endsWith('.npy')) && (
                              <button
                                onClick={(e) => {
                                  e.stopPropagation();
                                  if (onSelectFile) onSelectFile(file.name);
                                }}
                                className="opacity-0 group-hover:opacity-100 px-1.5 py-0.5 rounded bg-blue-600 hover:bg-blue-500 text-white text-[9px] font-bold flex items-center gap-0.5 shadow transition-all"
                              >
                                <span>🔍 Focus</span>
                              </button>
                            )}

                            {file.name.includes('masks') && (
                              <button
                                onClick={(e) => {
                                  e.stopPropagation();
                                  if (onSelectFile) onSelectFile(file.name);
                                }}
                                className="opacity-0 group-hover:opacity-100 px-1.5 py-0.5 rounded bg-emerald-600 hover:bg-emerald-500 text-white text-[9px] font-bold flex items-center gap-0.5 shadow transition-all"
                              >
                                <span>✨ Layer</span>
                              </button>
                            )}

                            {badge && (
                              <span className={`text-[8px] px-1 py-0.2 rounded border font-mono ${badge.color}`}>
                                {badge.label}
                              </span>
                            )}

                            {/* Delete File Button */}
                            <button
                              onClick={(e) => {
                                e.stopPropagation();
                                onDeleteFile(file.name);
                              }}
                              className="opacity-0 group-hover:opacity-100 hover:text-rose-400 p-0.5 rounded transition-opacity"
                            >
                              <Trash2 size={11} />
                            </button>
                          </div>
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

      {/* Context Menu on Right Click */}
      {contextMenu && (
        <div
          className="fixed z-50 bg-slate-900 border border-slate-700 rounded-lg shadow-2xl p-1 text-[10px] font-medium flex flex-col gap-0.5 w-44 backdrop-blur-md"
          style={{ top: contextMenu.y, left: contextMenu.x }}
          onClick={(e) => e.stopPropagation()}
        >
          <div className="px-2 py-1 text-[9px] text-slate-400 border-b border-slate-800 truncate">
            {contextMenu.fileName}
          </div>
          {contextMenu.state && isPathYaml(contextMenu.fileName) && (
            <>
              <button
                onClick={() => {
                  onVerifyPath(contextMenu.state!);
                  setContextMenu(null);
                }}
                className="px-2 py-1 rounded text-left hover:bg-sky-600 text-slate-200 flex items-center gap-1.5"
              >
                <CheckCheck size={11} className="text-sky-400" />
                <span>Verify Kinematics</span>
              </button>
              <button
                onClick={() => {
                  onOptimizePath(contextMenu.state!);
                  setContextMenu(null);
                }}
                className="px-2 py-1 rounded text-left hover:bg-emerald-600 text-slate-200 flex items-center gap-1.5"
              >
                <Sparkles size={11} className="text-emerald-400" />
                <span>Optimize POI</span>
              </button>
            </>
          )}
          {canSimulateFile(contextMenu.fileName) && contextMenu.state && (
            <button
              onClick={() => {
                if (onSimulatePath) onSimulatePath(contextMenu.state!);
                setContextMenu(null);
              }}
              className="px-2 py-1 rounded text-left hover:bg-sky-600 text-slate-200 flex items-center gap-1.5"
            >
              <Play size={11} className="text-sky-400" />
              <span>Play Simulation</span>
            </button>
          )}
          {canExecuteFile(contextMenu.fileName) && (
            <button
              onClick={() => {
                if (onExecutePath) onExecutePath(contextMenu.fileName, contextMenu.state);
                setContextMenu(null);
              }}
              className="px-2 py-1 rounded text-left hover:bg-emerald-600 text-slate-200 flex items-center gap-1.5"
            >
              <Zap size={11} className="text-emerald-400" />
              <span>Execute MoveL</span>
            </button>
          )}
          <button
            onClick={() => {
              onDeleteFile(contextMenu.fileName);
              setContextMenu(null);
            }}
            className="px-2 py-1 rounded text-left hover:bg-rose-600 text-rose-300 hover:text-white flex items-center gap-1.5"
          >
            <Trash2 size={11} />
            <span>Delete File</span>
          </button>
        </div>
      )}
    </div>
  );
};
