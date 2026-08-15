import React, { useState, useEffect, useRef } from 'react';
import { Camera, Maximize2, Minimize2, Crosshair, GripHorizontal, X } from 'lucide-react';

interface FloatingCameraZoneProps {
  onClose?: () => void;
}

const FloatingCameraZone: React.FC<FloatingCameraZoneProps> = ({ onClose }) => {
  const [resolution, setResolution] = useState<{width: number, height: number} | null>(null);
  const [streamUrl, setStreamUrl] = useState("http://localhost:8000/api/calib/camera/stream");
  
  // Floating window state
  const [position, setPosition] = useState({ x: 24, y: 24 });
  const [size, setSize] = useState({ w: 380, h: 280 });
  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  const [isMaximized, setIsMaximized] = useState(false);
  
  const dragRef = useRef({ startX: 0, startY: 0, initialX: 0, initialY: 0 });
  const resizeRef = useRef({ startX: 0, startY: 0, initialW: 0, initialH: 0 });

  // Handle Dragging
  const handleDragStart = (e: React.MouseEvent) => {
    if (isMaximized) return;
    setIsDragging(true);
    dragRef.current = {
      startX: e.clientX,
      startY: e.clientY,
      initialX: position.x,
      initialY: position.y
    };
  };

  // Handle Resizing
  const handleResizeStart = (e: React.MouseEvent) => {
    if (isMaximized) return;
    e.stopPropagation();
    setIsResizing(true);
    resizeRef.current = {
      startX: e.clientX,
      startY: e.clientY,
      initialW: size.w,
      initialH: size.h
    };
  };

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      if (isDragging) {
        const dx = e.clientX - dragRef.current.startX;
        const dy = e.clientY - dragRef.current.startY;
        setPosition({
          x: Math.max(0, dragRef.current.initialX + dx),
          y: Math.max(0, dragRef.current.initialY + dy)
        });
      } else if (isResizing) {
        const dw = e.clientX - resizeRef.current.startX;
        const dh = e.clientY - resizeRef.current.startY;
        setSize({
          w: Math.max(250, resizeRef.current.initialW + dw),
          h: Math.max(180, resizeRef.current.initialH + dh)
        });
      }
    };

    const handleMouseUp = () => {
      setIsDragging(false);
      setIsResizing(false);
    };

    if (isDragging || isResizing) {
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
    }

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging, isResizing]);

  const containerClasses = isMaximized
    ? "fixed inset-4 md:inset-6 z-[100] bg-slate-900/98 rounded-xl border border-slate-700/80 shadow-2xl overflow-hidden flex flex-col transition-all duration-200 backdrop-blur-xl animate-in zoom-in-95 duration-150"
    : "absolute bg-slate-900/95 rounded-xl border border-slate-700/90 shadow-2xl overflow-hidden flex flex-col z-50 transition-shadow duration-200 backdrop-blur-md animate-in fade-in duration-150";

  const containerStyles = isMaximized
    ? {}
    : {
        left: position.x,
        top: position.y,
        width: size.w,
        height: size.h,
        boxShadow: isDragging ? '0 25px 50px -12px rgba(0, 0, 0, 0.8)' : undefined
      };

  return (
    <div className={containerClasses} style={containerStyles}>
      {/* Draggable Header */}
      <div 
        className={`shrink-0 px-3.5 py-2 flex justify-between items-center bg-gradient-to-b from-slate-800 to-slate-900 border-b border-slate-700 select-none ${isMaximized ? '' : 'cursor-move'}`}
        onMouseDown={handleDragStart}
      >
        <div className="flex items-center gap-2">
          <Camera size={15} className="text-blue-400" />
          <h2 className="font-medium text-slate-100 text-xs drop-shadow-md">
            Live Stream {isMaximized ? '(Fullscreen)' : ''}
          </h2>
          <span className="relative flex h-1.5 w-1.5 ml-1">
            {resolution ? (
              <>
                <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75"></span>
                <span className="relative inline-flex rounded-full h-1.5 w-1.5 bg-emerald-500 shadow-[0_0_6px_rgba(16,185,129,0.7)]"></span>
              </>
            ) : (
              <span className="relative inline-flex rounded-full h-1.5 w-1.5 bg-slate-600"></span>
            )}
          </span>
        </div>

        <div className="flex items-center gap-1.5 text-slate-400">
          <button
            onClick={() => setIsMaximized(!isMaximized)}
            className={`transition-colors rounded p-1 hover:bg-slate-700 hover:text-white ${
              isMaximized ? 'bg-blue-600/30 text-blue-300' : ''
            }`}
            title={isMaximized ? "Exit Fullscreen" : "Fullscreen"}
          >
            {isMaximized ? <Minimize2 size={14} /> : <Maximize2 size={14} />}
          </button>
          
          {onClose && (
            <button
              onClick={onClose}
              className="p-1 hover:bg-slate-700 hover:text-white rounded transition-colors"
              title="Close Live Stream (Can reopen from Left Sidebar)"
            >
              <X size={14} />
            </button>
          )}
          
          {!isMaximized && <GripHorizontal size={14} className="opacity-50" />}
        </div>
      </div>
      
      {/* Video Content */}
      <div className="flex-1 bg-black flex items-center justify-center relative overflow-hidden group">
        {/* Resolution Overlay */}
        {resolution && (
          <div className="absolute bottom-3 left-3 z-20 bg-slate-900/70 backdrop-blur-md px-2 py-1 rounded border border-slate-700/50 text-[10px] font-mono text-emerald-400 shadow pointer-events-none">
            {resolution.width}×{resolution.height}
          </div>
        )}

        {/* Real Camera Stream */}
        <img 
          src={streamUrl} 
          alt="Camera Stream" 
          className="absolute inset-0 w-full h-full object-contain z-0 transition-opacity duration-300 pointer-events-none"
          onError={(e) => {
            e.currentTarget.style.opacity = '0';
            setResolution(null);
            const fallback = document.getElementById('camera-fallback');
            if (fallback) fallback.style.display = 'flex';
            
            setTimeout(() => {
              setStreamUrl(`http://localhost:8000/api/calib/camera/stream?t=${Date.now()}`);
            }, 3000);
          }}
          onLoad={(e) => {
            e.currentTarget.style.opacity = '1';
            setResolution({
              width: e.currentTarget.naturalWidth,
              height: e.currentTarget.naturalHeight
            });
            const fallback = document.getElementById('camera-fallback');
            if (fallback) fallback.style.display = 'none';
          }}
        />
        
        {/* Camera Reticle Overlay */}
        <div className="absolute inset-0 border-[1px] border-blue-500/10 m-3 rounded pointer-events-none z-10">
          <div className="absolute top-0 left-0 w-6 h-6 border-t-2 border-l-2 border-blue-500/40 rounded-tl"></div>
          <div className="absolute top-0 right-0 w-6 h-6 border-t-2 border-r-2 border-blue-500/40 rounded-tr"></div>
          <div className="absolute bottom-0 left-0 w-6 h-6 border-b-2 border-l-2 border-blue-500/40 rounded-bl"></div>
          <div className="absolute bottom-0 right-0 w-6 h-6 border-b-2 border-r-2 border-blue-500/40 rounded-br"></div>
          <Crosshair className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 text-blue-500/20 w-10 h-10" strokeWidth={1} />
        </div>
        
        <div id="camera-fallback" className="text-slate-500 flex flex-col items-center z-0 transition-opacity duration-300 pointer-events-none">
          <Camera size={36} strokeWidth={1} className="mb-2 opacity-20" />
          <p className="text-xs tracking-wide">Waiting for camera signal...</p>
        </div>

        {/* Resize Handle */}
        {!isMaximized && (
          <div 
            className="absolute bottom-0 right-0 w-5 h-5 cursor-se-resize flex items-end justify-end p-1 z-30 group"
            onMouseDown={handleResizeStart}
          >
            <div className="w-2.5 h-2.5 border-r-2 border-b-2 border-slate-500 group-hover:border-blue-400 transition-colors"></div>
          </div>
        )}
      </div>
    </div>
  );
};

export default FloatingCameraZone;
