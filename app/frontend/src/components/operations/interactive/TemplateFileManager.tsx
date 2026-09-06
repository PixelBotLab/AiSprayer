import React, { useRef, useState, useEffect } from 'react';
import { FolderPlus, Trash2, ChevronLeft, ChevronRight, Check, X } from 'lucide-react';

interface TemplateTopBarProps {
  templates: string[];
  activeTemplate: string | null;
  onSelectTemplate: (t: string) => void;
  onCreateTemplate: (name: string) => Promise<void> | void;
  onDeleteTemplate: (t: string) => Promise<void> | void;
}

export const TemplateTopBar: React.FC<TemplateTopBarProps> = ({
  templates,
  activeTemplate,
  onSelectTemplate,
  onCreateTemplate,
  onDeleteTemplate,
}) => {
  const scrollRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);
  const [isCreating, setIsCreating] = useState(false);
  const [newTemplateName, setNewTemplateName] = useState('');
  const [deletingTemplate, setDeletingTemplate] = useState<string | null>(null);

  useEffect(() => {
    if (isCreating && inputRef.current) {
      inputRef.current.focus();
      inputRef.current.select();
    }
  }, [isCreating]);

  const scrollTabs = (direction: 'left' | 'right') => {
    if (scrollRef.current) {
      scrollRef.current.scrollBy({
        left: direction === 'left' ? -180 : 180,
        behavior: 'smooth',
      });
    }
  };

  const handleStartCreate = () => {
    const now = new Date();
    const pad = (n: number) => String(n).padStart(2, '0');
    const defaultName = `${now.getFullYear()}-${pad(now.getMonth() + 1)}-${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
    setNewTemplateName(defaultName);
    setIsCreating(true);
  };

  const handleCommitCreate = () => {
    if (newTemplateName.trim()) {
      onCreateTemplate(newTemplateName.trim());
    }
    setIsCreating(false);
  };

  const handleCancelCreate = () => {
    setIsCreating(false);
  };

  return (
    <div className="h-9 bg-slate-900 border-b border-slate-800 flex items-center px-2 justify-between select-none shrink-0 z-10 gap-1.5">
      {/* Left Action: New Template Button / Inline Input */}
      {isCreating ? (
        <div className="flex items-center gap-1 bg-slate-800/90 border border-sky-500/50 rounded px-1.5 py-0.5 shrink-0 animate-in fade-in duration-150">
          <input
            ref={inputRef}
            type="text"
            value={newTemplateName}
            onChange={(e) => setNewTemplateName(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === 'Enter') handleCommitCreate();
              if (e.key === 'Escape') handleCancelCreate();
            }}
            className="bg-transparent text-xs text-sky-200 outline-none w-36 font-mono"
            placeholder="Template name"
          />
          <button
            onClick={handleCommitCreate}
            className="p-0.5 text-emerald-400 hover:text-emerald-300 transition-colors"
            title="Confirm (Enter)"
          >
            <Check size={12} />
          </button>
          <button
            onClick={handleCancelCreate}
            className="p-0.5 text-slate-400 hover:text-slate-200 transition-colors"
            title="Cancel (Esc)"
          >
            <X size={12} />
          </button>
        </div>
      ) : (
        <button
          onClick={handleStartCreate}
          className="p-1 rounded bg-slate-800 hover:bg-slate-700 text-sky-400 border border-slate-700 shrink-0 transition-colors flex items-center gap-1 text-xs font-medium px-2"
          title="Create New Template"
        >
          <FolderPlus size={13} />
          <span>New</span>
        </button>
      )}

      <div className="h-4 w-[1px] bg-slate-800 shrink-0" />

      {/* Left Scroll Arrow */}
      <button
        onClick={() => scrollTabs('left')}
        className="p-1 hover:bg-slate-800 text-slate-400 hover:text-slate-200 rounded transition-colors shrink-0"
        title="Scroll Left"
      >
        <ChevronLeft size={15} />
      </button>

      {/* Center Template Tabs Container */}
      <div
        ref={scrollRef}
        className="flex-1 flex items-center gap-1.5 overflow-x-hidden py-0.5 scroll-smooth"
      >
        {templates.map((t) => {
          const isActive = activeTemplate === t;
          const isConfirmingDelete = deletingTemplate === t;

          if (isConfirmingDelete) {
            return (
              <div
                key={t}
                className="flex items-center gap-1 px-2 py-0.5 rounded-md text-xs bg-rose-950/80 border border-rose-500/60 text-rose-200 shrink-0 animate-in fade-in"
                onClick={(e) => e.stopPropagation()}
              >
                <span className="font-semibold text-[11px]">Delete?</span>
                <button
                  onClick={() => {
                    onDeleteTemplate(t);
                    setDeletingTemplate(null);
                  }}
                  className="px-1.5 py-0.2 bg-rose-600 hover:bg-rose-500 text-white rounded text-[10px] font-bold"
                  title="Confirm Delete"
                >
                  Yes
                </button>
                <button
                  onClick={() => setDeletingTemplate(null)}
                  className="px-1.5 py-0.2 bg-slate-800 hover:bg-slate-700 text-slate-300 rounded text-[10px]"
                  title="Cancel"
                >
                  No
                </button>
              </div>
            );
          }

          return (
            <div
              key={t}
              className={`group flex items-center gap-1.5 px-2.5 py-0.5 rounded-md text-xs font-medium transition-all shrink-0 cursor-pointer border ${
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
                  setDeletingTemplate(t);
                }}
                className="opacity-0 group-hover:opacity-100 hover:text-rose-400 p-0.5 rounded transition-opacity"
                title="Delete Template"
              >
                <Trash2 size={11} />
              </button>
            </div>
          );
        })}
      </div>

      {/* Right Scroll Arrow */}
      <button
        onClick={() => scrollTabs('right')}
        className="p-1 hover:bg-slate-800 text-slate-400 hover:text-slate-200 rounded transition-colors shrink-0"
        title="Scroll Right"
      >
        <ChevronRight size={15} />
      </button>
    </div>
  );
};

