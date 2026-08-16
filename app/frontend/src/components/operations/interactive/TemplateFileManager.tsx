import React from 'react';
import {
  FolderPlus,
  Trash2,
  Image as ImageIcon,
  FileJson,
  FileCode2,
  Box,
  Route,
} from 'lucide-react';
import type { FileItem } from './types';

interface TemplateFileManagerProps {
  templates: string[];
  activeTemplate: string | null;
  files: FileItem[];
  showFiles: boolean;
  isLoadingTemplate?: boolean;
  onSelectTemplate: (t: string) => void;
  onOpenCreateTemplateModal: () => void;
  onOpenDeleteTemplateModal: (t: string) => void;
  onOpenDeleteFileModal: (f: string) => void;
  onToggleShowFiles: () => void;
}

export const TemplateFileManager: React.FC<TemplateFileManagerProps> = ({
  templates,
  activeTemplate,
  files,
  showFiles,
  onSelectTemplate,
  onOpenCreateTemplateModal,
  onOpenDeleteTemplateModal,
  onOpenDeleteFileModal,
  onToggleShowFiles,
}) => {
  const getFileIcon = (fileName: string) => {
    if (fileName.endsWith('.jpg') || fileName.endsWith('.png'))
      return <ImageIcon size={14} className="text-emerald-400 shrink-0" />;
    if (fileName.endsWith('.json'))
      return <FileJson size={14} className="text-amber-400 shrink-0" />;
    if (fileName.endsWith('.yaml') || fileName.endsWith('.yml'))
      return <FileCode2 size={14} className="text-sky-400 shrink-0" />;
    if (fileName.endsWith('.ply') || fileName.endsWith('.stl'))
      return <Box size={14} className="text-indigo-400 shrink-0" />;
    return <Route size={14} className="text-slate-400 shrink-0" />;
  };

  return (
    <div className="h-10 bg-slate-900 border-b border-slate-800 flex items-center px-4 justify-between select-none z-10 shrink-0">
      {/* Horizontal Template Selector Pills */}
      <div className="flex items-center gap-1.5 overflow-x-auto py-1 scrollbar-none max-w-[65%]">
        <button
          onClick={onOpenCreateTemplateModal}
          className="p-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-sky-400 border border-slate-700 shrink-0 transition-colors flex items-center gap-1 text-xs font-medium px-2"
          title="Create New Template"
        >
          <FolderPlus size={14} />
          <span>New</span>
        </button>
        <div className="h-4 w-[1px] bg-slate-800 mx-1 shrink-0" />
        {templates.map((t) => {
          const isActive = activeTemplate === t;
          return (
            <div
              key={t}
              className={`group flex items-center gap-1.5 px-2.5 py-1 rounded-md text-xs font-medium transition-all shrink-0 cursor-pointer border ${
                isActive
                  ? 'bg-sky-950/80 text-sky-300 border-sky-500/50 shadow-sm'
                  : 'bg-slate-800/60 text-slate-400 border-transparent hover:bg-slate-800 hover:text-slate-200'
              }`}
              onClick={() => onSelectTemplate(t)}
            >
              <span>{t}</span>
              <button
                onClick={(e) => {
                  e.stopPropagation();
                  onOpenDeleteTemplateModal(t);
                }}
                className="opacity-0 group-hover:opacity-100 hover:text-rose-400 p-0.5 rounded transition-opacity"
                title="Delete Template"
              >
                <Trash2 size={12} />
              </button>
            </div>
          );
        })}
      </div>

      {/* Right Side: File List Toggle and Quick Actions */}
      <div className="flex items-center gap-3">
        {activeTemplate && (
          <div className="relative">
            <button
              onClick={onToggleShowFiles}
              className={`px-2.5 py-1 rounded-lg text-xs font-medium flex items-center gap-1.5 transition-colors border ${
                showFiles
                  ? 'bg-slate-800 text-sky-300 border-slate-700'
                  : 'bg-slate-900 text-slate-400 border-slate-800 hover:text-slate-200'
              }`}
            >
              <FileCode2 size={13} />
              <span>Files ({files.length})</span>
            </button>

            {/* Template File List Dropdown */}
            {showFiles && (
              <div className="absolute right-0 top-full mt-2 w-64 bg-slate-900 border border-slate-700 rounded-xl shadow-2xl z-50 p-2 text-xs space-y-1 backdrop-blur-md">
                <div className="text-[10px] font-bold text-slate-400 px-2 py-1 uppercase tracking-wider border-b border-slate-800">
                  Template Artifacts
                </div>
                <div className="max-h-60 overflow-y-auto space-y-0.5 scrollbar-thin">
                  {files.length === 0 ? (
                    <div className="p-3 text-center text-slate-500 text-[11px]">
                      No files generated yet
                    </div>
                  ) : (
                    files.map((file) => (
                      <div
                        key={file.name}
                        className="flex items-center justify-between p-1.5 rounded hover:bg-slate-800 group transition-colors"
                      >
                        <div className="flex items-center gap-2 overflow-hidden">
                          {getFileIcon(file.name)}
                          <span className="truncate text-slate-300" title={file.name}>
                            {file.name}
                          </span>
                        </div>
                        <div className="flex items-center gap-1">
                          <span className="text-[10px] text-slate-500 font-mono">
                            {(file.size / 1024).toFixed(0)} KB
                          </span>
                          <button
                            onClick={() => onOpenDeleteFileModal(file.name)}
                            className="opacity-0 group-hover:opacity-100 hover:text-rose-400 p-1 rounded transition-opacity"
                            title="Delete File"
                          >
                            <Trash2 size={12} />
                          </button>
                        </div>
                      </div>
                    ))
                  )}
                </div>
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};
