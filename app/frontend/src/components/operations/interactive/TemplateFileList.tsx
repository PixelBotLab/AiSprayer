import React from 'react';
import {
  HardDrive,
  FileCode2,
  Camera,
  Layers,
  Route,
  Box,
  Image as ImageIcon,
  FileJson,
  Trash2,
} from 'lucide-react';
import type { FileItem } from './types';

interface TemplateFileListProps {
  files: FileItem[];
  onOpenDeleteFileModal: (f: string) => void;
}

export const TemplateFileList: React.FC<TemplateFileListProps> = ({
  files,
  onOpenDeleteFileModal,
}) => {
  const getFileIcon = (fileName: string) => {
    if (fileName.endsWith('.jpg') || fileName.endsWith('.png'))
      return <ImageIcon size={12} className="text-emerald-400 shrink-0" />;
    if (fileName.endsWith('.json'))
      return <FileJson size={12} className="text-amber-400 shrink-0" />;
    if (fileName.endsWith('.yaml') || fileName.endsWith('.yml'))
      return <FileCode2 size={12} className="text-sky-400 shrink-0" />;
    if (fileName.endsWith('.ply') || fileName.endsWith('.stl'))
      return <Box size={12} className="text-purple-400 shrink-0" />;
    return <Route size={12} className="text-slate-400 shrink-0" />;
  };

  const getFileBadge = (fileName: string) => {
    if (fileName === 'scan.jpg') return { label: 'RGB', color: 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30' };
    if (fileName === 'scan.depth.npy' || fileName === 'scan.depth.bin') return { label: 'DEPTH', color: 'bg-cyan-500/10 text-cyan-400 border-cyan-500/30' };
    if (fileName === 'scan.masks.yaml') return { label: 'MASKS', color: 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30' };
    if (fileName.includes('opt_paths') || fileName.includes('opt.paths')) return { label: 'OPT', color: 'bg-sky-500/10 text-sky-400 border-sky-500/30' };
    if (fileName.includes('manual_paths') || fileName.includes('raw.paths')) return { label: 'RAW', color: 'bg-rose-500/10 text-rose-400 border-rose-500/30' };
    if (fileName.endsWith('.ply') || fileName.endsWith('.stl')) return { label: '3D', color: 'bg-purple-500/10 text-purple-400 border-purple-500/30' };
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
      id: 'capture',
      title: 'Capture',
      icon: Camera,
      iconColor: 'text-blue-400',
      badgeColor: 'bg-blue-500/10 text-blue-400 border-blue-500/30',
      files: files.filter((f) => ['scan.jpg', 'scan.depth.npy', 'scan.depth.bin', 'scan.pcd', 'scan.params.yaml'].includes(f.name)),
    },
    {
      id: 'segment',
      title: 'Segment',
      icon: Layers,
      iconColor: 'text-emerald-400',
      badgeColor: 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30',
      files: files.filter((f) => ['scan.masks.yaml', 'scan.masks.jpg'].includes(f.name)),
    },
    {
      id: 'plan',
      title: 'Manual Path',
      icon: Route,
      iconColor: 'text-rose-400',
      badgeColor: 'bg-rose-500/10 text-rose-400 border-rose-500/30',
      files: files.filter((f) => f.name.includes('manual_paths') || f.name.includes('paths')),
    },
    {
      id: 'reconstruct',
      title: 'Reconstruct',
      icon: Box,
      iconColor: 'text-purple-400',
      badgeColor: 'bg-purple-500/10 text-purple-400 border-purple-500/30',
      files: files.filter((f) => f.name.startsWith('scan.mesh') || f.name.includes('.ply') || f.name.includes('.stl')),
    },
    {
      id: 'other',
      title: 'Other',
      icon: FileCode2,
      iconColor: 'text-slate-400',
      badgeColor: 'bg-slate-800 text-slate-400 border-slate-700',
      files: files.filter(
        (f) =>
          !['scan.jpg', 'scan.depth.npy', 'scan.depth.bin', 'scan.pcd', 'scan.params.yaml', 'scan.masks.yaml', 'scan.masks.jpg'].includes(f.name) &&
          !f.name.includes('manual_paths') &&
          !f.name.includes('paths') &&
          !f.name.startsWith('scan.mesh') &&
          !f.name.includes('.ply') &&
          !f.name.includes('.stl')
      ),
    },
  ];

  return (
    <div className="w-full flex flex-col h-full min-h-0 select-none bg-slate-950/40">
      <div className="h-9 px-2.5 border-b border-slate-800 bg-slate-900/90 flex justify-between items-center shrink-0">
        <div className="flex items-center gap-1.5 text-[11px] text-slate-300 font-medium">
          <HardDrive size={13} className="text-slate-400" />
          <span>Files</span>
        </div>
        <span className="bg-slate-800 text-slate-400 text-[9px] font-mono px-1.5 py-0.5 rounded border border-slate-700">
          {files.length}
        </span>
      </div>

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
                  {cat.files.map((file) => {
                    const badge = getFileBadge(file.name);
                    const isMaskYaml = file.name === 'scan.masks.yaml';
                    const isManualPathYaml = file.name.includes('manual_paths') || file.name.includes('paths.yaml');
                    const isMesh = file.name.includes('.ply') || file.name.includes('.stl');
                    return (
                      <div
                        key={file.name}
                        className={`p-1.5 rounded-lg border transition-all flex flex-col gap-0.5 group ${
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
                            <span
                              className={`text-[11px] font-medium truncate ${
                                isMaskYaml
                                  ? 'text-emerald-300'
                                  : isManualPathYaml
                                  ? 'text-rose-300'
                                  : isMesh
                                  ? 'text-purple-300'
                                  : 'text-slate-200'
                              }`}
                              title={file.name}
                            >
                              {file.name}
                            </span>
                          </div>
                          <div className="flex items-center gap-1 shrink-0">
                            {badge && (
                              <span className={`text-[8px] px-1 py-0.2 rounded border font-mono ${badge.color}`}>
                                {badge.label}
                              </span>
                            )}
                            <button
                              onClick={() => onOpenDeleteFileModal(file.name)}
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
    </div>
  );
};
