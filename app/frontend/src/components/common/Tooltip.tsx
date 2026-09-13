import React from 'react';

/**
 * Standard Dark Glassmorphic Tooltip Class for AiSprayer.
 * Unified design token across the entire application:
 * - Ultra-compact padding: px-2 py-0.5
 * - Sleek font: text-[9.5px] font-medium text-slate-300 tracking-wide
 * - Dark high-transparency glass background: bg-slate-950/85 backdrop-blur-md
 * - Subtle border & shadow: border border-white/10 shadow-xl shadow-black/40 rounded-md
 * - Hardware accelerated & non-blocking: pointer-events-none whitespace-nowrap z-[100]
 */
export const TOOLTIP_BASE_CLASS =
  'bg-slate-950/85 backdrop-blur-md border border-white/10 text-slate-300 text-[9.5px] font-medium px-2 py-0.5 rounded-md shadow-xl shadow-black/40 whitespace-nowrap pointer-events-none z-[100] transition-opacity duration-150';

export interface TooltipProps {
  children?: React.ReactNode;
  text?: string;
  side?: 'top' | 'bottom' | 'left' | 'right';
  align?: 'center' | 'start' | 'end';
  className?: string;
  multiline?: boolean;
  maxWidthClass?: string;
}

export const Tooltip: React.FC<TooltipProps> = ({
  children,
  text,
  side = 'top',
  align = 'center',
  className = '',
  multiline = false,
  maxWidthClass = 'max-w-xs',
}) => {
  const content = text || children;
  if (!content) return null;

  let posClass = '';
  if (side === 'top') {
    posClass = 'bottom-full mb-1.5';
  } else if (side === 'bottom') {
    posClass = 'top-full mt-1.5';
  } else if (side === 'left') {
    posClass = 'right-full mr-2';
  } else if (side === 'right') {
    posClass = 'left-full ml-2';
  }

  let alignClass = '';
  if (side === 'top' || side === 'bottom') {
    if (align === 'center') alignClass = 'left-1/2 -translate-x-1/2';
    else if (align === 'start') alignClass = 'left-0';
    else if (align === 'end') alignClass = 'right-0';
  } else {
    if (align === 'center') alignClass = 'top-1/2 -translate-y-1/2';
    else if (align === 'start') alignClass = 'top-0';
    else if (align === 'end') alignClass = 'bottom-0';
  }

  const innerClass = multiline
    ? `${TOOLTIP_BASE_CLASS.replace('whitespace-nowrap', 'whitespace-normal')} ${maxWidthClass} leading-relaxed text-left`
    : TOOLTIP_BASE_CLASS;

  return (
    <div
      className={`absolute ${posClass} ${alignClass} hidden group-hover:flex flex-col items-center pointer-events-none z-[100] ${className}`}
    >
      <div className={innerClass}>{content}</div>
    </div>
  );
};
