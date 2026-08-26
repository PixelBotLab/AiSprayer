import React, { useState, useEffect } from 'react';
import { Settings, Crosshair, Route, ListTodo, Maximize2, Minimize2, Camera } from 'lucide-react';
import { WS_BASE } from '../config';

interface LayoutProps {
  children: React.ReactNode;
  activeTab: string;
  setActiveTab: (tab: string) => void;
  isCameraVisible?: boolean;
  setIsCameraVisible?: (visible: boolean) => void;
}

// Professional AI Robotic Spraying (AiSprayer) Brand Logo
const AiRobotSprayerLogo: React.FC<{ size?: number; className?: string }> = ({ size = 24, className = '' }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="none"
    xmlns="http://www.w3.org/2000/svg"
    className={className}
  >
    {/* 1. Heavy Industrial Base Pedestal */}
    <path d="M2.5 21.5H9.5" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" />
    <path d="M4 21.5V18.5H8V21.5" fill="currentColor" />

    {/* 2. Shoulder Revolute Joint */}
    <circle cx="6" cy="16" r="2.4" stroke="currentColor" strokeWidth="1.8" fill="currentColor" fillOpacity="0.25" />
    <circle cx="6" cy="16" r="0.8" fill="currentColor" />

    {/* 3. Articulated Upper Arm Link */}
    <path d="M6 16L10.5 9" stroke="currentColor" strokeWidth="3" strokeLinecap="round" />

    {/* 4. Elbow Joint with AI Neural Pulse Ring */}
    <circle cx="10.5" cy="9" r="2.2" stroke="currentColor" strokeWidth="1.8" fill="currentColor" fillOpacity="0.25" />
    <circle cx="10.5" cy="9" r="0.8" fill="#FBBF24" />

    {/* 5. Forearm Manipulator Link */}
    <path d="M10.5 9L15.5 6.5" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" />

    {/* 6. Precision Spray Gun End-Effector */}
    <path d="M14.5 4.5L18 6.5L16.5 9L13 7Z" fill="currentColor" />
    <path d="M18 6.5L19.5 5.5" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />

    {/* 7. Atomized Spray Fan Cone & Fine Mist Particles */}
    <path d="M20.2 4.2L23.8 2" stroke="#38BDF8" strokeWidth="1.8" strokeLinecap="round" strokeDasharray="1.5 1.5" />
    <path d="M20.5 6.2H24" stroke="#38BDF8" strokeWidth="2" strokeLinecap="round" />
    <path d="M20.2 8.2L23.8 10.4" stroke="#38BDF8" strokeWidth="1.8" strokeLinecap="round" strokeDasharray="1.5 1.5" />
    
    <circle cx="21.8" cy="2.8" r="0.75" fill="#38BDF8" />
    <circle cx="23.2" cy="6.2" r="0.75" fill="#38BDF8" />
    <circle cx="21.8" cy="9.6" r="0.75" fill="#38BDF8" />

    {/* 8. AI Intelligence Star (Top Golden Sparkle) */}
    <path
      d="M10.5 1L11.3 2.6L12.9 3.4L11.3 4.2L10.5 5.8L9.7 4.2L8.1 3.4L9.7 2.6L10.5 1Z"
      fill="#FBBF24"
      stroke="#F59E0B"
      strokeWidth="0.4"
    />
  </svg>
);

const Layout: React.FC<LayoutProps> = ({ 
  children, 
  activeTab, 
  setActiveTab,
  isCameraVisible = false,
  setIsCameraVisible 
}) => {
  const tabs = [
    { id: 'interactive', label: 'Interactive Teach', icon: Route, iconClassName: 'rotate-90' },
    { id: 'task', label: 'Task Execution', icon: ListTodo },
    { id: 'calib', label: 'Calibration', icon: Crosshair },
  ];
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [isCameraOnline, setIsCameraOnline] = useState(false);

  useEffect(() => {
    let ws: WebSocket | null = null;
    let reconnectTimer: any = null;
    let isDisposed = false;

    const connectWs = () => {
      if (isDisposed) return;
      try {
        ws = new WebSocket(`${WS_BASE}/api/camera/ws`);
        ws.onopen = () => {};
        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            setIsCameraOnline(Boolean(data.online));
          } catch {}
        };
        ws.onclose = () => {
          if (!isDisposed) {
            setIsCameraOnline(false);
            reconnectTimer = setTimeout(connectWs, 2000);
          }
        };
        ws.onerror = () => {
          if (ws) ws.close();
        };
      } catch {
        if (!isDisposed) {
          reconnectTimer = setTimeout(connectWs, 2000);
        }
      }
    };

    connectWs();

    return () => {
      isDisposed = true;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      if (ws) ws.close();
    };
  }, []);

  useEffect(() => {
    const handleFullscreenChange = () => {
      setIsFullscreen(!!document.fullscreenElement);
    };

    document.addEventListener('fullscreenchange', handleFullscreenChange);
    document.addEventListener('webkitfullscreenchange', handleFullscreenChange);

    return () => {
      document.removeEventListener('fullscreenchange', handleFullscreenChange);
      document.removeEventListener('webkitfullscreenchange', handleFullscreenChange);
    };
  }, []);

  const toggleFullscreen = () => {
    if (!document.fullscreenElement) {
      if (document.documentElement.requestFullscreen) {
        document.documentElement.requestFullscreen().catch((err) => {
          console.error(`Error attempting to enable fullscreen: ${err.message}`);
        });
      }
    } else {
      if (document.exitFullscreen) {
        document.exitFullscreen().catch((err) => {
          console.error(`Error attempting to exit fullscreen: ${err.message}`);
        });
      }
    }
  };

  const tooltipClass = "absolute left-14 px-3 py-1.5 bg-slate-800 text-slate-100 text-xs font-medium rounded shadow-xl border border-slate-700 backdrop-blur-md opacity-0 invisible group-hover:opacity-100 group-hover:visible transition-all whitespace-nowrap z-[100]";

  return (
    <div className="flex h-screen bg-slate-950 font-sans text-white overflow-hidden relative">
      
      {/* Static Spacer for sidebar */}
      <div className="w-16 shrink-0 bg-slate-900 border-r border-slate-800 hidden md:block"></div>

      {/* Fixed Sidebar */}
      <aside 
        className="absolute left-0 top-0 bottom-0 z-50 flex flex-col border-r border-slate-800 bg-slate-900/95 backdrop-blur-md shadow-2xl w-16"
      >
        <div className="border-b border-slate-800 p-5 shrink-0 flex items-center justify-center">
          <div className="flex items-center justify-center relative group">
            <div className="flex shrink-0 h-9 w-9 items-center justify-center rounded-lg bg-gradient-to-br from-blue-500 to-indigo-600 shadow-lg shadow-blue-500/25 ml-[-4px]">
              <AiRobotSprayerLogo size={21} className="text-white" />
            </div>
            <div className={tooltipClass}>
              AiSprayer
            </div>
          </div>
        </div>

        <nav className="flex-1 space-y-2 px-3 py-4">
          {tabs.map((tab) => {
            const Icon = tab.icon;
            const isActive = activeTab === tab.id;
            return (
              <button
                key={tab.id}
                onClick={() => setActiveTab(tab.id)}
                className={`flex w-full relative items-center justify-center gap-4 rounded-lg px-3 py-3 text-sm transition-all duration-200 group ${
                  isActive
                    ? "bg-blue-500/10 text-blue-400 font-medium"
                    : "text-slate-400 hover:bg-white/[0.05] hover:text-slate-200"
                }`}
              >
                <div className="shrink-0 flex items-center justify-center relative ml-[-2px]">
                  {isActive && <div className="absolute -left-3 w-1 h-5 bg-blue-500 rounded-r-full"></div>}
                  <Icon size={18} className={`${tab.iconClassName || ''} ${isActive ? "text-blue-400" : "text-slate-500 group-hover:text-slate-300"}`} />
                </div>
                <div className={tooltipClass}>
                  {tab.label}
                </div>
              </button>
            );
          })}
        </nav>

        <div className="border-t border-slate-800 p-3 shrink-0 space-y-2">
          {/* Live Camera Stream Toggle Button with Green Breathing Dot */}
          {setIsCameraVisible && (
            <button
              onClick={() => setIsCameraVisible(!isCameraVisible)}
              className={`flex w-full relative items-center justify-center gap-4 rounded-lg px-3 py-3 text-sm transition-all duration-200 group ${
                isCameraVisible
                  ? "bg-blue-500/20 text-blue-300 font-medium border border-blue-500/40 shadow-[0_0_12px_rgba(59,130,246,0.3)]"
                  : "text-slate-400 hover:bg-slate-800 hover:text-white"
              }`}
            >
              <div className="shrink-0 flex items-center justify-center relative ml-[-2px]">
                <Camera size={18} className={isCameraVisible ? "text-blue-400" : "text-slate-400 group-hover:text-blue-300"} />
                {/* Dynamic Camera Online/Offline Indicator Dot */}
                <span className="absolute -top-1.5 -right-2 flex h-2 w-2">
                  {isCameraOnline ? (
                    <>
                      <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75"></span>
                      <span className="relative inline-flex rounded-full h-2 w-2 bg-emerald-500 shadow-[0_0_8px_rgba(16,185,129,0.7)]"></span>
                    </>
                  ) : (
                    <span className="relative inline-flex rounded-full h-2 w-2 bg-slate-600 shadow-sm"></span>
                  )}
                </span>
              </div>
              <div className={tooltipClass}>
                {isCameraVisible 
                  ? (isCameraOnline ? 'Hide Live Camera (Online)' : 'Hide Live Camera (Offline)') 
                  : (isCameraOnline ? 'Show Live Camera (Online)' : 'Show Live Camera (Offline)')}
              </div>
            </button>
          )}

          {/* Fullscreen Toggle Button */}
          <button
            onClick={toggleFullscreen}
            className="flex w-full relative items-center justify-center gap-4 rounded-lg px-3 py-3 text-sm transition-colors text-slate-400 hover:bg-slate-800 hover:text-white group"
          >
            <div className="shrink-0 flex items-center justify-center relative ml-[-2px]">
              {isFullscreen ? (
                <Minimize2 size={18} className="text-slate-400 group-hover:text-blue-400 transition-colors" />
              ) : (
                <Maximize2 size={18} className="text-slate-400 group-hover:text-blue-400 transition-colors" />
              )}
            </div>
            <div className={tooltipClass}>
              {isFullscreen ? 'Exit Fullscreen' : 'Fullscreen'}
            </div>
          </button>

          {/* System Config Button */}
          <button 
            onClick={() => setActiveTab('config')}
            className={`flex w-full relative items-center justify-center gap-4 rounded-lg px-3 py-3 text-sm transition-colors group ${
              activeTab === 'config' ? 'bg-blue-600 text-white' : 'text-slate-400 hover:bg-slate-800 hover:text-white'
            }`}
          >
            <div className="shrink-0 flex items-center justify-center relative ml-[-2px]">
              <Settings size={18} className={activeTab === 'config' ? 'text-white' : 'text-slate-500 group-hover:text-slate-300'} />
            </div>
            <div className={tooltipClass}>
              System Config
            </div>
          </button>
        </div>
      </aside>

      {/* Main Content */}
      <main className="flex-1 flex flex-col relative overflow-hidden bg-slate-950">
        {children}
      </main>
    </div>
  );
};
export default Layout;
