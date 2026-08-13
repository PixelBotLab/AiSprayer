import React, { useState, useEffect } from 'react';
import { Settings, Crosshair, MousePointer2, GitBranch, Box, Maximize2, Minimize2 } from 'lucide-react';

interface LayoutProps {
  children: React.ReactNode;
  activeTab: string;
  setActiveTab: (tab: string) => void;
}

const Layout: React.FC<LayoutProps> = ({ children, activeTab, setActiveTab }) => {
  const tabs = [
    { id: 'interactive', label: 'Interactive Teach', icon: MousePointer2 },
    { id: 'auto_planner', label: '3D Auto Planner', icon: GitBranch },
    { id: 'digital_twin', label: 'Digital Twin', icon: Box },
    { id: 'calib', label: 'Calibration', icon: Crosshair },
  ];
  const [isFullscreen, setIsFullscreen] = useState(false);

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
            <div className="flex shrink-0 h-9 w-9 items-center justify-center rounded-lg bg-gradient-to-br from-blue-500 to-indigo-600 shadow-lg shadow-blue-500/20 ml-[-4px]">
              <svg className="h-5 w-5 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M9.75 17L9 20l-1 1h8l-1-1-.75-3M3 13h18M5 17h14a2 2 0 002-2V5a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z"/>
              </svg>
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
                  <Icon size={18} className={isActive ? "text-blue-400" : "text-slate-500 group-hover:text-slate-300"} />
                </div>
                <div className={tooltipClass}>
                  {tab.label}
                </div>
              </button>
            );
          })}
        </nav>

        <div className="border-t border-slate-800 p-3 shrink-0 space-y-1">
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
