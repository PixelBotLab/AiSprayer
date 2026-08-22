import React from 'react';

export const TOOLBAR_TIP_CLASS =
  'bg-slate-950/70 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap';

export const ToolbarTip: React.FC<{ children: React.ReactNode; side?: 'top' | 'bottom' }> = ({
  children,
  side = 'top',
}) => (
  <div
    className={`absolute ${side === 'top' ? 'bottom-full mb-1.5' : 'top-full mt-1.5'} hidden group-hover:flex pointer-events-none z-50`}
  >
    <div className={TOOLBAR_TIP_CLASS}>{children}</div>
  </div>
);
