import React from 'react';
import { Compass, Pencil, Route, Wand2 } from 'lucide-react';
import type { PathSource, PathStage, PathStateType } from './types';
import { composePathState, pathSourceOf, pathStageOf } from './types';

interface PathStateSwitcherProps {
  activeState: PathStateType;
  onSelect: (state: PathStateType) => void;
  counts?: Partial<Record<PathStateType, number>>;
  compact?: boolean;
}

export const PathStateSwitcher: React.FC<PathStateSwitcherProps> = ({
  activeState,
  onSelect,
  counts,
  compact = false,
}) => {
  const source = pathSourceOf(activeState);
  const stage = pathStageOf(activeState);

  const selectSource = (next: PathSource) => {
    const keepStage = stage === 'poi' && (counts?.[composePathState(next, 'poi')] || 0) > 0;
    onSelect(composePathState(next, keepStage ? 'poi' : 'orig'));
  };

  const selectStage = (next: PathStage) => {
    onSelect(composePathState(source, next));
  };

  const iconBtn = (
    active: boolean,
    onClick: () => void,
    title: string,
    icon: React.ReactNode,
    activeClass: string,
  ) => (
    <button
      type="button"
      onClick={onClick}
      title={title}
      className={`p-1 rounded transition-colors ${
        active ? activeClass : 'text-slate-500 hover:text-slate-300 hover:bg-white/10'
      }`}
    >
      {icon}
    </button>
  );

  const sourceManual = iconBtn(
    source === 'manual',
    () => selectSource('manual'),
    'Manual path',
    <Pencil size={11} />,
    'bg-slate-500/25 text-slate-200',
  );
  const sourceAuto = iconBtn(
    source === 'auto',
    () => selectSource('auto'),
    'Auto path',
    <Wand2 size={11} />,
    'bg-violet-500/25 text-violet-300',
  );
  const stagePath = iconBtn(
    stage === 'orig',
    () => selectStage('orig'),
    'Original path',
    <Route size={11} />,
    source === 'auto' ? 'bg-violet-500/25 text-violet-300' : 'bg-slate-500/25 text-slate-200',
  );
  const stagePoi = iconBtn(
    stage === 'poi',
    () => selectStage('poi'),
    'POI pose',
    <Compass size={11} />,
    source === 'auto' ? 'bg-teal-500/25 text-teal-300' : 'bg-emerald-500/25 text-emerald-300',
  );

  return (
    <div className={`flex items-center gap-0.5 ${compact ? '' : 'w-full'}`}>
      {sourceManual}
      {sourceAuto}
      <div className="w-[1px] h-2.5 bg-white/10 mx-0.5" />
      {stagePath}
      {stagePoi}
      {!compact && counts && (
        <span className="ml-auto pr-1 font-mono text-[9px] tabular-nums text-slate-500">
          {counts[activeState] ?? 0}
        </span>
      )}
    </div>
  );
};
