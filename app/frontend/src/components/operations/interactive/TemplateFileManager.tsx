import React, { useRef } from 'react';
import { FolderPlus, Trash2, ChevronLeft, ChevronRight } from 'lucide-react';

interface TemplateTopBarProps {
  templates: string[];
  activeTemplate: string | null;
  onSelectTemplate: (t: string) => void;
  onOpenCreateTemplateModal: () => void;
  onOpenDeleteTemplateModal: (t: string) => void;
}

export const TemplateTopBar: React.FC<TemplateTopBarProps> = ({
  templates,
  activeTemplate,
  onSelectTemplate,
  onOpenCreateTemplateModal,
  onOpenDeleteTemplateModal,
}) => {
  const scrollRef = useRef<HTMLDivElement>(null);

  const scrollTabs = (direction: 'left' | 'right') => {
    if (scrollRef.current) {
      scrollRef.current.scrollBy({
        left: direction === 'left' ? -180 : 180,
        behavior: 'smooth',
      });
    }
  };

  return (
    <div className="h-9 bg-slate-900 border-b border-slate-800 flex items-center px-2 justify-between select-none shrink-0 z-10 gap-1.5">
      {/* Left Action: New Template Button */}
      <button
        onClick={onOpenCreateTemplateModal}
        className="p-1 rounded bg-slate-800 hover:bg-slate-700 text-sky-400 border border-slate-700 shrink-0 transition-colors flex items-center gap-1 text-xs font-medium px-2"
        title="Create New Template"
      >
        <FolderPlus size={13} />
        <span>New</span>
      </button>

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
                  onOpenDeleteTemplateModal(t);
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
