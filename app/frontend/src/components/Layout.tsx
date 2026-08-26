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

// High-Tech AI Robotic Spraying (AiSprayer) Logo
const AiRobotSprayerLogo: React.FC<{ size?: number; className?: string }> = ({ size = 21, className = '' }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeLinecap="round"
    strokeLinejoin="round"
    className={className}
  >
    {/* 1. Tech "AI" Typography (Golden Amber Accent) - Shifted slightly upward */}
    <path d="M1.5 5.8L3.2 0.8L4.9 5.8" stroke="#FBBF24" strokeWidth="1.1" />
    <path d="M2.1 4.0H4.3" stroke="#FBBF24" strokeWidth="1.0" />
    <path d="M6.6 0.8V5.8" stroke="#FBBF24" strokeWidth="1.1" />

    {/* 2. Enlarged AI Neural Sparkle Star - Shifted slightly upward */}
    <path
      d="M10.8 0L11.7 2.1L13.8 3.0L11.7 3.9L10.8 6.0L9.9 3.9L7.8 3.0L9.9 2.1Z"
      fill="#FBBF24"
      stroke="#F59E0B"
      strokeWidth="0.3"
    />

    {/* 3. Flat Base & Mounting Pedestal */}
    <path d="M2 21.2h8" strokeWidth="2.1" />
    <path d="M3.8 21.2v-2.8h4.4v2.8" strokeWidth="1.7" />

    {/* 4. Shoulder Revolute Joint */}
    <circle cx="6" cy="15.2" r="2.1" strokeWidth="1.8" />

    {/* 5. Articulated Upper Arm Link */}
    <path d="M6 13.1L9.6 8.5" strokeWidth="2" />

    {/* 6. Elbow Joint */}
    <circle cx="9.6" cy="8.5" r="1.9" strokeWidth="1.8" />

    {/* 7. Forearm Link */}
    <path d="M11.5 8.5H15.6" strokeWidth="1.9" />

    {/* 8. Spray Gun Nozzle End-Effector */}
    <path d="M15.6 6.8v3.4l2.8-1.7z" strokeWidth="1.7" />
    <path d="M18.4 8.5h1.2" strokeWidth="1.8" />

    {/* 9. Cyan Tech Atomized Spray Fan & Fine Droplets */}
    <path d="M20.2 5.8L23.8 3.5" stroke="#38BDF8" strokeWidth="1.6" strokeDasharray="1.5 1.5" />
    <path d="M20.5 8.5H23.8" stroke="#38BDF8" strokeWidth="1.8" />
    <path d="M20.2 11.2L23.8 13.5" stroke="#38BDF8" strokeWidth="1.6" strokeDasharray="1.5 1.5" />
    <circle cx="22.2" cy="5.2" r="0.6" fill="#38BDF8" stroke="none" />
    <circle cx="23.4" cy="8.5" r="0.6" fill="#38BDF8" stroke="none" />
    <circle cx="22.2" cy="11.8" r="0.6" fill="#38BDF8" stroke="none" />
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
        <div className="border-b border-slate-800 p-3 shrink-0 flex items-center justify-center">
          <div className="flex w-full relative items-center justify-center rounded-lg px-3 py-3 group cursor-default transition-all duration-200 bg-slate-800/40 border border-slate-700/40 hover:bg-slate-800/70 hover:border-slate-700/60 shadow-inner">
            <div className="shrink-0 flex items-center justify-center">
              <AiRobotSprayerLogo size={22} className="text-slate-300 group-hover:text-white transition-colors" />
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
