import React, { useState, useEffect } from 'react';
import {
  HardDrive,
  FileCode2,
  Camera,
  Layers,
  Route,
  Box,
  Image as ImageIcon,
  FileBarChart,
  Trash2,
  Play,
  Activity,
  Zap,
} from 'lucide-react';
import type { FileItem, PathStateType, ManualPathItem, VerificationReport } from './types';

interface TemplateFileListProps {
  files: FileItem[];
  rawPaths?: ManualPathItem[];
  autoPaths?: ManualPathItem[];
  autoPoiPaths?: ManualPathItem[];
  poiPaths?: ManualPathItem[];
  rawReport?: VerificationReport | null;
  autoReport?: VerificationReport | null;
  poiReport?: VerificationReport | null;
  autoPoiReport?: VerificationReport | null;
  robotConnected?: boolean;
  onDeleteFile: (f: string) => void;
  onSimulatePath?: (state: PathStateType, pathId?: number | null) => void;
  onExecutePath?: (fileName: string, state?: PathStateType, pathId?: number | null) => void;
  onOpenDiagnostics?: (state: PathStateType) => void;
  onSelectFile?: (fileName: string) => void;
}

export const TemplateFileList: React.FC<TemplateFileListProps> = ({
  files,
  rawPaths = [],
  autoPaths = [],
  autoPoiPaths = [],
  poiPaths = [],
  rawReport = null,
  autoReport = null,
  poiReport = null,
  autoPoiReport = null,
  robotConnected = false,
  onDeleteFile,
  onSimulatePath,
  onExecutePath,
  onOpenDiagnostics,
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
    if (fileName.includes('manual.poi') || fileName.includes('poi.path') || fileName.includes('poi.report')) return poiPaths || [];
    if (fileName.includes('auto.path') || fileName.includes('auto.report')) return autoPaths || [];
    if (fileName.includes('manual.path') || fileName.includes('manual.report') || fileName.includes('raw.path') || fileName.includes('raw.report') || fileName.includes('manual_paths')) return rawPaths || [];
    return [];
  };

  const getFileState = (fileName: string): PathStateType | null => {
    if (fileName.includes('auto.poi')) return 'auto_poi';
    if (fileName.includes('manual.poi') || fileName.includes('poi.path') || fileName.includes('poi.report')) return 'poi';
    if (fileName.includes('auto.path') || fileName.includes('auto.report')) return 'auto';
    if (fileName.includes('manual.path') || fileName.includes('manual.report') || fileName.includes('raw.path') || fileName.includes('raw.report') || fileName.includes('manual_paths')) return 'raw';
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

  const canSimulateFile = (fileName: string | null) => {
    if (!fileName || !isPathYaml(fileName)) return false;
    const st = getFileState(fileName);
    return !!st && hasRunnablePaths(fileName) && hasSimTrajectory(st);
  };

  const canExecuteFile = (fileName: string | null) => {
    if (!fileName || !isPathYaml(fileName) || !robotConnected) return false;
    return !!getFileState(fileName) && hasRunnablePaths(fileName);
  };

  const selectedState = selectedFile ? getFileState(selectedFile) : null;
  const canSim = canSimulateFile(selectedFile);
  const canExec = canExecuteFile(selectedFile);


  const getFileIcon = (fileName: string) => {
    if (fileName.endsWith('.jpg') || fileName.endsWith('.png'))
      return <ImageIcon size={12} className="text-emerald-400 shrink-0" />;
    if (fileName.endsWith('.report.json') || fileName.endsWith('.json'))
      return <FileBarChart size={12} className="text-amber-400 shrink-0" />;
    if (fileName.endsWith('.yaml') || fileName.endsWith('.yml'))
      return <Route size={12} className="text-sky-400 shrink-0" />;
    if (fileName.endsWith('.ply') || fileName.endsWith('.stl'))
      return <Box size={12} className="text-purple-400 shrink-0" />;
    return <FileCode2 size={12} className="text-slate-400 shrink-0" />;
  };

  const getFileBadge = (fileName: string) => {
    if (fileName === 'scan.jpg') return { label: 'RGB', color: 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30' };
    if (fileName === 'scan.depth.npy' || fileName === 'scan.depth.bin') return { label: 'DEPTH', color: 'bg-cyan-500/10 text-cyan-400 border-cyan-500/30' };
    if (fileName === 'scan.masks.yaml') return { label: 'MASKS', color: 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30' };
    if (fileName.includes('auto.poi.path')) return { label: 'AUTO POI', color: 'bg-teal-500/20 text-teal-300 border-teal-500/40' };
    if (fileName.includes('manual.poi.path') || fileName.includes('poi.path')) return { label: 'MANUAL POI', color: 'bg-emerald-500/20 text-emerald-400 border-emerald-500/40' };
    if (fileName.includes('auto.path')) return { label: 'AUTO', color: 'bg-violet-500/20 text-violet-300 border-violet-500/40' };
    if (fileName.includes('manual.path') || fileName.includes('raw.path') || fileName.includes('manual_paths')) return { label: 'MANUAL', color: 'bg-slate-500/20 text-slate-300 border-slate-500/40' };
    if (fileName.includes('auto.poi.report')) return { label: 'AUTO POI DIAG', color: 'bg-teal-500/10 text-teal-300 border-teal-500/30' };
    if (fileName.includes('manual.poi.report') || fileName.includes('poi.report')) return { label: 'MANUAL POI DIAG', color: 'bg-emerald-500/10 text-emerald-300 border-emerald-500/30' };
    if (fileName.includes('auto.report')) return { label: 'AUTO DIAG', color: 'bg-violet-500/10 text-violet-300 border-violet-500/30' };
    if (fileName.includes('manual.report') || fileName.includes('raw.report')) return { label: 'MANUAL DIAG', color: 'bg-slate-500/10 text-slate-400 border-slate-500/30' };
    if (fileName.endsWith('.ply') || fileName.endsWith('.stl')) return { label: '3D MESH', color: 'bg-purple-500/10 text-purple-400 border-purple-500/30' };
    if (fileName.endsWith('.json')) return { label: 'DATA', color: 'bg-amber-500/10 text-amber-400 border-amber-500/30' };
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

  const fileCategories = [
    {
      id: 'paths',
      title: 'Trajectories & Reports',
      icon: Route,
      iconColor: 'text-sky-400',
      badgeColor: 'bg-sky-500/10 text-sky-400 border-sky-500/30',
      files: files.filter((f) => f.name.includes('path') || f.name.includes('report')),
    },
    {
      id: 'capture',
      title: 'Capture & Depth',
      icon: Camera,
      iconColor: 'text-blue-400',
      badgeColor: 'bg-blue-500/10 text-blue-400 border-blue-500/30',
      files: files.filter((f) => ['scan.jpg', 'scan.depth.npy', 'scan.depth.bin', 'scan.pcd', 'scan.params.yaml'].includes(f.name)),
    },
    {
      id: 'segment',
      title: 'Segment Masks',
      icon: Layers,
      iconColor: 'text-emerald-400',
      badgeColor: 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30',
      files: files.filter((f) => ['scan.masks.yaml', 'scan.masks.jpg'].includes(f.name)),
    },
    {
      id: 'reconstruct',
      title: 'Reconstruction 3D',
      icon: Box,
      iconColor: 'text-purple-400',
      badgeColor: 'bg-purple-500/10 text-purple-400 border-purple-500/30',
      files: files.filter((f) => f.name.startsWith('scan.mesh') || f.name.includes('.ply') || f.name.includes('.stl')),
    },
    {
      id: 'other',
      title: 'Other Files',
      icon: FileCode2,
      iconColor: 'text-slate-400',
      badgeColor: 'bg-slate-800 text-slate-400 border-slate-700',
      files: files.filter(
        (f) =>
          !f.name.includes('path') &&
          !f.name.includes('report') &&
          !['scan.jpg', 'scan.depth.npy', 'scan.depth.bin', 'scan.pcd', 'scan.params.yaml', 'scan.masks.yaml', 'scan.masks.jpg'].includes(f.name) &&
          !f.name.startsWith('scan.mesh') &&
          !f.name.includes('.ply') &&
          !f.name.includes('.stl')
      ),
    },
  ];

  return (
    <div className="w-full flex flex-col h-full min-h-0 select-none bg-slate-950/40 relative">
      {/* File List Header */}
      <div className="h-9 px-2.5 border-b border-slate-800 bg-slate-900/90 flex justify-between items-center shrink-0">
        <div className="flex items-center gap-1.5 text-[11px] text-slate-300 font-medium">
          <HardDrive size={13} className="text-slate-400" />
          <span>Files</span>
        </div>
        <div className="flex items-center gap-1">
          <button
            type="button"
            disabled={!canSim}
            onClick={() => {
              if (canSim && selectedState && onSimulatePath) onSimulatePath(selectedState, null);
            }}
            className={`p-1 rounded transition-colors ${
              canSim ? 'text-sky-300 hover:bg-white/10' : 'text-slate-600 cursor-not-allowed'
            }`}
            title={canSim ? `Simulate ${selectedFile}` : 'Select a path file with a verified trajectory'}
          >
            <Play size={11} className={canSim ? 'fill-sky-300' : ''} />
          </button>
          <button
            type="button"
            disabled={!canExec}
            onClick={() => {
              if (canExec && selectedFile && onExecutePath) onExecutePath(selectedFile, selectedState || undefined, null);
            }}
            className={`p-1 rounded transition-colors ${
              canExec ? 'text-emerald-300 hover:bg-white/10' : 'text-slate-600 cursor-not-allowed'
            }`}
            title={canExec ? `Execute ${selectedFile}` : robotConnected ? 'Select a path file with waypoints' : 'Robot offline'}
          >
            <Zap size={11} />
          </button>
          <span className="bg-slate-800 text-slate-400 text-[9px] font-mono px-1.5 py-0.5 rounded border border-slate-700">
            {files.length}
          </span>
        </div>
      </div>

      {/* Files Category Scroll Container */}
      <div className="flex-1 overflow-y-auto overflow-x-hidden p-1.5 flex flex-col gap-2 custom-scrollbar">
        {files.length === 0 ? (
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
                    const isReportJson = file.name.endsWith('.report.json');
                    const isSelected = selectedFile === file.name;

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
                            {isReportJson && (
                              <button
                                onClick={(e) => {
                                  e.stopPropagation();
                                  if (onOpenDiagnostics && stateType) onOpenDiagnostics(stateType);
                                }}
                                className="opacity-0 group-hover:opacity-100 px-1.5 py-0.5 rounded bg-amber-600 hover:bg-amber-500 text-white text-[9px] font-bold flex items-center gap-0.5 shadow transition-all"
                                title="Open TCP Diagnostics"
                              >
                                <Activity size={8} />
                                <span>Diag</span>
                              </button>
                            )}

                            {(file.name.endsWith('.ply') || file.name.endsWith('.stl')) && (
                              <button
                                onClick={(e) => {
                                  e.stopPropagation();
                                  if (onSelectFile) onSelectFile(file.name);
                                }}
                                className="opacity-0 group-hover:opacity-100 px-1.5 py-0.5 rounded bg-purple-600 hover:bg-purple-500 text-white text-[9px] font-bold flex items-center gap-0.5 shadow transition-all"
                                title="View 3D Surface Mesh"
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
                                title="Focus in 2D Canvas"
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
                                title="Toggle Segmentation Mask Layer"
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
                              title="Delete file"
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
          {canSimulateFile(contextMenu.fileName) && contextMenu.state && (
            <button
              onClick={() => {
                if (onSimulatePath) onSimulatePath(contextMenu.state!);
                setContextMenu(null);
              }}
              className="px-2 py-1 rounded text-left hover:bg-sky-600 text-slate-200 flex items-center gap-1.5"
            >
              <Play size={11} className="text-sky-400" />
              <span>Simulate</span>
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
              <span>Execute on Robot</span>
            </button>
          )}
          {contextMenu.fileName.includes('report') && contextMenu.state && (
            <button
              onClick={() => {
                if (onOpenDiagnostics) onOpenDiagnostics(contextMenu.state!);
                setContextMenu(null);
              }}
              className="px-2 py-1 rounded text-left hover:bg-amber-600 text-slate-200 flex items-center gap-1.5"
            >
              <Activity size={11} />
              <span>Inspect Diagnostics</span>
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
