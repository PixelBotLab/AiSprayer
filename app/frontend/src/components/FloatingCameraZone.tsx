import React, { useState, useEffect, useRef } from 'react';
import { Camera, Maximize, Crosshair, GripHorizontal } from 'lucide-react';

const FloatingCameraZone: React.FC = () => {
  const [resolution, setResolution] = useState<{width: number, height: number} | null>(null);
  
  // Floating window state
  const [position, setPosition] = useState({ x: 24, y: 24 });
  const [size, setSize] = useState({ w: 400, h: 300 });
  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  
  const dragRef = useRef({ startX: 0, startY: 0, initialX: 0, initialY: 0 });
  const resizeRef = useRef({ startX: 0, startY: 0, initialW: 0, initialH: 0 });

  // Handle Dragging
  const handleDragStart = (e: React.MouseEvent) => {
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
          w: Math.max(250, resizeRef.current.initialW + dw), // min-width 250
          h: Math.max(200, resizeRef.current.initialH + dh)  // min-height 200
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

  return (
    <div 
      className="absolute bg-slate-900 rounded-xl border border-slate-700 shadow-2xl overflow-hidden flex flex-col z-50 transition-shadow duration-200"
      style={{
        left: position.x,
        top: position.y,
        width: size.w,
        height: size.h,
        boxShadow: isDragging ? '0 25px 50px -12px rgba(0, 0, 0, 0.7)' : undefined
      }}
    >
      {/* Draggable Header */}
      <div 
        className="shrink-0 px-4 py-2.5 flex justify-between items-center bg-gradient-to-b from-slate-800 to-slate-900 border-b border-slate-700 cursor-move select-none"
        onMouseDown={handleDragStart}
      >
        <h2 className="font-medium text-slate-100 flex items-center gap-2 text-sm drop-shadow-md">
          <Camera size={16} className="text-blue-400" />
          Live Stream
        </h2>
        <div className="flex gap-2 text-slate-400">
          <GripHorizontal size={16} />
        </div>
      </div>
      
      {/* Video Content */}
      <div className="flex-1 bg-black flex items-center justify-center relative overflow-hidden group">
        {/* Resolution Overlay */}
        {resolution && (
          <div className="absolute bottom-4 left-4 z-20 bg-slate-900/60 backdrop-blur-md px-3 py-1.5 rounded-lg border border-slate-700/50 text-xs font-mono text-emerald-400 shadow-lg pointer-events-none">
            {resolution.width} &times; {resolution.height}
          </div>
        )}

        {/* Real Camera Stream */}
        <img 
          src="http://localhost:8000/api/calib/camera/stream" 
          alt="Camera Stream" 
          className="absolute inset-0 w-full h-full object-contain z-0 transition-opacity duration-300 pointer-events-none"
          onError={(e) => {
            e.currentTarget.style.opacity = '0';
            setResolution(null);
            const fallback = document.getElementById('camera-fallback');
            if (fallback) fallback.style.display = 'flex';
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
        
        {/* Mock Camera View Overlay */}
        <div className="absolute inset-0 border-[1px] border-blue-500/10 m-4 rounded pointer-events-none z-10">
          <div className="absolute top-0 left-0 w-8 h-8 border-t-2 border-l-2 border-blue-500/50 rounded-tl"></div>
          <div className="absolute top-0 right-0 w-8 h-8 border-t-2 border-r-2 border-blue-500/50 rounded-tr"></div>
          <div className="absolute bottom-0 left-0 w-8 h-8 border-b-2 border-l-2 border-blue-500/50 rounded-bl"></div>
          <div className="absolute bottom-0 right-0 w-8 h-8 border-b-2 border-r-2 border-blue-500/50 rounded-br"></div>
          <Crosshair className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 text-blue-500/20 w-12 h-12" strokeWidth={1} />
        </div>
        
        <div id="camera-fallback" className="text-slate-500 flex flex-col items-center z-0 transition-opacity duration-300 pointer-events-none">
          <Camera size={48} strokeWidth={1} className="mb-3 opacity-20" />
          <p className="text-sm tracking-wide">Waiting for camera signal...</p>
        </div>

        {/* Resize Handle */}
        <div 
          className="absolute bottom-0 right-0 w-6 h-6 cursor-se-resize flex items-end justify-end p-1 z-30 group"
          onMouseDown={handleResizeStart}
        >
          <div className="w-2.5 h-2.5 border-r-2 border-b-2 border-slate-500 group-hover:border-blue-400 transition-colors"></div>
        </div>
      </div>
    </div>
  );
};

export default FloatingCameraZone;
