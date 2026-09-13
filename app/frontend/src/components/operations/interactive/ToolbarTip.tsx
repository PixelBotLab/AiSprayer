import React from 'react';
import { TOOLTIP_BASE_CLASS, Tooltip, type TooltipProps } from '../../common/Tooltip';

export const TOOLBAR_TIP_CLASS = TOOLTIP_BASE_CLASS;

export const ToolbarTip: React.FC<TooltipProps> = (props) => {
  return <Tooltip {...props} />;
};
