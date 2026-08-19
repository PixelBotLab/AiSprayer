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
import type { FileItem, PathStateType, ManualPathItem } from './types';

interface TemplateFileListProps {
  files: FileItem[];
  rawPaths?: ManualPathItem[];
  optPaths?: ManualPathItem[];
  poiPaths?: ManualPathItem[];
  robotConnected?: boolean;
  onOpenDeleteFileModal: (f: string) => void;
  onSimulatePath?: (state: PathStateType, pathId?: number | null) => void;
  onExecutePath?: (fileName: string, state?: PathStateType, pathId?: number | null) => void;
  onOpenDiagnostics?: (state: PathStateType) => void;
  onSelectFile?: (fileName: string) => void;
}

export const TemplateFileList: React.FC<TemplateFileListProps> = ({
  files,
  rawPaths = [],
  optPaths = [],
  poiPaths = [],
  robotConnected = false,
  onOpenDeleteFileModal,
  onSimulatePath,
  onExecutePath,
  onOpenDiagnostics,
  onSelectFile,
}) => {
  const [contextMenu, setContextMenu] = useState<{ x: number; y: number; fileName: string; state?: PathStateType } | null>(null);

  // Close menus on outside click
  useEffect(() => {
    const handleOutside = () => {
      setContextMenu(null);
    };
    window.addEventListener('click', handleOutside);
    return () => window.removeEventListener('click', handleOutside);
  }, []);

  const getPathsForFile = (fileName: string) => {
    if (fileName.includes('poi.path') || fileName.includes('poi.report')) return poiPaths || [];
    if (fileName.includes('opt.path') || fileName.includes('opt.report') || fileName.includes('opt_paths')) return optPaths || [];
    if (fileName.includes('raw.path') || fileName.includes('raw.report') || fileName.includes('manual_paths')) return rawPaths || [];
    return [];
  };

  const getFileState = (fileName: string): PathStateType | null => {
    if (fileName.includes('poi.path') || fileName.includes('poi.report')) return 'poi';
    if (fileName.includes('opt.path') || fileName.includes('opt.report') || fileName.includes('opt_paths')) return 'opt';
    if (fileName.includes('raw.path') || fileName.includes('raw.report') || fileName.includes('manual_paths')) return 'raw';
    return null;
  };


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
    if (fileName.includes('poi.path')) return { label: 'POI', color: 'bg-emerald-500/20 text-emerald-400 border-emerald-500/40' };
    if (fileName.includes('opt.path') || fileName.includes('opt_paths')) return { label: 'OPT', color: 'bg-sky-500/20 text-sky-400 border-sky-500/40' };
    if (fileName.includes('raw.path') || fileName.includes('manual_paths')) return { label: 'RAW', color: 'bg-slate-500/20 text-slate-300 border-slate-500/40' };
    if (fileName.includes('poi.report')) return { label: 'POI DIAG', color: 'bg-emerald-500/10 text-emerald-300 border-emerald-500/30' };
    if (fileName.includes('opt.report')) return { label: 'OPT DIAG', color: 'bg-sky-500/10 text-sky-300 border-sky-500/30' };
    if (fileName.includes('raw.report')) return { label: 'RAW DIAG', color: 'bg-slate-500/10 text-slate-400 border-slate-500/30' };
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
        <span className="bg-slate-800 text-slate-400 text-[9px] font-mono px-1.5 py-0.5 rounded border border-slate-700">
          {files.length}
        </span>
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
                    const isPathYaml = file.name.endsWith('.path.yaml') || file.name.includes('paths.yaml');
                    const isReportJson = file.name.endsWith('.report.json');

                    return (
                      <div
                        key={file.name}
                        onClick={() => onSelectFile && onSelectFile(file.name)}
                        onContextMenu={(e) => {
                          e.preventDefault();
                          setContextMenu({
                            x: e.clientX,
                            y: e.clientY,
                            fileName: file.name,
                            state: stateType || undefined,
                          });
                        }}
                        className="p-1.5 rounded-lg border transition-all flex flex-col gap-0.5 group relative bg-slate-900/60 border-slate-800/80 hover:border-slate-700 hover:bg-slate-800/60 cursor-pointer"
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
                            {/* Hover Quick Action Buttons (Feature 5 & 7) */}
                            {isPathYaml && (
                              <div className="opacity-0 group-hover:opacity-100 flex items-center gap-1 transition-opacity">
                                {/* Sim Button with Hover Popover */}
                                <div className="relative group/sim">
                                  <button
                                    onClick={(e) => {
                                      e.stopPropagation();
                                      if (onSimulatePath && stateType) onSimulatePath(stateType, null);
                                    }}
                                    className="px-1.5 py-0.5 rounded bg-sky-600 hover:bg-sky-500 text-white text-[9px] font-bold flex items-center gap-0.5 shadow transition-all"
                                    title="3D/2D Simulation"
                                  >
                                    <Play size={8} className="fill-white" />
                                    <span>Sim</span>
                                  </button>

                                  {/* Sim Floating Path Selector Popover */}
                                  {(() => {
                                    const pList = getPathsForFile(file.name);
                                    if (pList.length <= 1) return null;
                                    return (
                                      <div className="absolute right-0 top-full mt-1 z-50 hidden group-hover/sim:flex flex-col bg-slate-900/95 border border-slate-700 rounded-lg shadow-2xl p-1 text-[9.5px] w-40 backdrop-blur-md animate-fadeIn pointer-events-auto">
                                        <div className="px-2 py-0.5 text-[8.5px] font-semibold text-slate-400 border-b border-slate-800 uppercase tracking-wider flex items-center gap-1">
                                          <Play size={8} className="text-sky-400" />
                                          <span>Simulate Path</span>
                                        </div>
                                        <button
                                          onClick={(e) => {
                                            e.stopPropagation();
                                            if (onSimulatePath && stateType) onSimulatePath(stateType, null);
                                          }}
                                          className="px-2 py-1 rounded text-left hover:bg-sky-600 text-slate-200 flex items-center justify-between transition-colors font-medium"
                                        >
                                          <span>All Paths (全部路径)</span>
                                          <span className="text-[8.5px] font-mono text-sky-400">{pList.length}P</span>
                                        </button>
                                        {pList.map((p, idx) => {
                                          const pId = p.path_id ?? (idx + 1);
                                          return (
                                            <button
                                              key={pId}
                                              onClick={(e) => {
                                                e.stopPropagation();
                                                if (onSimulatePath && stateType) onSimulatePath(stateType, pId);
                                              }}
                                              className="px-2 py-1 rounded text-left hover:bg-sky-600 text-slate-200 flex items-center justify-between transition-colors"
                                            >
                                              <span className="truncate">{p.name || `Path ${pId}`}</span>
                                              <span className="text-[8.5px] font-mono text-slate-400">{p.points?.length || 0} pts</span>
                                            </button>
                                          );
                                        })}
                                      </div>
                                    );
                                  })()}
                                </div>

                                {/* Exec Button with Hover Popover — disabled when robot offline */}
                                <div className="relative group/exec">
                                  <button
                                    disabled={!robotConnected}
                                    onClick={(e) => {
                                      e.stopPropagation();
                                      if (robotConnected && onExecutePath) onExecutePath(file.name, stateType || undefined, null);
                                    }}
                                    className={`px-1.5 py-0.5 rounded text-white text-[9px] font-bold flex items-center gap-0.5 shadow transition-all ${
                                      robotConnected
                                        ? 'bg-emerald-600 hover:bg-emerald-500 cursor-pointer'
                                        : 'bg-slate-700 text-slate-500 cursor-not-allowed opacity-60'
                                    }`}
                                    title={robotConnected ? 'Execute on Real Robot' : 'Robot offline — connect first'}
                                  >
                                    <Zap size={8} className={robotConnected ? 'text-white' : 'text-slate-500'} />
                                    <span>Exec</span>
                                  </button>

                                  {/* Exec Floating Path Selector Popover — only when connected */}
                                  {robotConnected && (() => {
                                    const pList = getPathsForFile(file.name);
                                    if (pList.length <= 1) return null;
                                    return (
                                      <div className="absolute right-0 top-full mt-1 z-50 hidden group-hover/exec:flex flex-col bg-slate-900/95 border border-slate-700 rounded-lg shadow-2xl p-1 text-[9.5px] w-40 backdrop-blur-md animate-fadeIn pointer-events-auto">
                                        <div className="px-2 py-0.5 text-[8.5px] font-semibold text-slate-400 border-b border-slate-800 uppercase tracking-wider flex items-center gap-1">
                                          <Zap size={8} className="text-emerald-400" />
                                          <span>Execute on Robot</span>
                                        </div>
                                        <button
                                          onClick={(e) => {
                                            e.stopPropagation();
                                            if (onExecutePath) onExecutePath(file.name, stateType || undefined, null);
                                          }}
                                          className="px-2 py-1 rounded text-left hover:bg-emerald-600 text-slate-200 flex items-center justify-between transition-colors font-medium"
                                        >
                                          <span>All Paths (全部路径)</span>
                                          <span className="text-[8.5px] font-mono text-emerald-400">{pList.length}P</span>
                                        </button>
                                        {pList.map((p, idx) => {
                                          const pId = p.path_id ?? (idx + 1);
                                          return (
                                            <button
                                              key={pId}
                                              onClick={(e) => {
                                                e.stopPropagation();
                                                if (onExecutePath) onExecutePath(file.name, stateType || undefined, pId);
                                              }}
                                              className="px-2 py-1 rounded text-left hover:bg-emerald-600 text-slate-200 flex items-center justify-between transition-colors"
                                            >
                                              <span className="truncate">{p.name || `Path ${pId}`}</span>
                                              <span className="text-[8.5px] font-mono text-slate-400">{p.points?.length || 0} pts</span>
                                            </button>
                                          );
                                        })}
                                      </div>
                                    );
                                  })()}
                                </div>
                              </div>
                            )}

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
                                onOpenDeleteFileModal(file.name);
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
          {contextMenu.state && (
            <button
              onClick={() => {
                if (onSimulatePath) onSimulatePath(contextMenu.state!);
                setContextMenu(null);
              }}
              className="px-2 py-1 rounded text-left hover:bg-sky-600 text-slate-200 flex items-center gap-1.5"
            >
              <Play size={11} className="text-sky-400" />
              <span>Simulate ({contextMenu.state.toUpperCase()})</span>
            </button>
          )}
          {(contextMenu.fileName.endsWith('.path.yaml') || contextMenu.fileName.includes('paths.yaml')) && (
            <button
              onClick={() => {
                if (onExecutePath) onExecutePath(contextMenu.fileName, contextMenu.state);
                setContextMenu(null);
              }}
              className="px-2 py-1 rounded text-left hover:bg-emerald-600 text-slate-200 flex items-center gap-1.5"
            >
              <Play size={11} className="text-emerald-400 fill-emerald-400" />
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
              onOpenDeleteFileModal(contextMenu.fileName);
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
