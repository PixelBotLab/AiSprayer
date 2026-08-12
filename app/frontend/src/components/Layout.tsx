import React, { useState } from 'react';
import { Settings, Crosshair, MousePointer2, GitBranch, Box, ChevronRight } from 'lucide-react';

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
  const [isHovered, setIsHovered] = useState(false);

  return (
    <div className="flex h-screen bg-slate-950 font-sans text-white overflow-hidden relative">
      
      {/* Static Spacer for collapsed sidebar */}
      <div className="w-16 shrink-0 bg-slate-900 border-r border-slate-800 hidden md:block"></div>

      {/* Floating Sidebar */}
      <aside 
        className={`absolute left-0 top-0 bottom-0 z-50 flex flex-col overflow-x-hidden border-r border-slate-800 bg-slate-900/95 backdrop-blur-md shadow-2xl transition-all duration-300 ${isHovered ? 'w-64' : 'w-16'}`}
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
      >
        <div className="border-b border-slate-800 p-5 shrink-0 flex items-center">
          <div className="flex items-center gap-3">
            <div className="flex shrink-0 h-9 w-9 items-center justify-center rounded-lg bg-gradient-to-br from-blue-500 to-indigo-600 shadow-lg shadow-blue-500/20 ml-[-4px]">
              <svg className="h-5 w-5 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M9.75 17L9 20l-1 1h8l-1-1-.75-3M3 13h18M5 17h14a2 2 0 002-2V5a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z"/>
              </svg>
            </div>
            <div className={`transition-opacity duration-300 ${isHovered ? 'opacity-100 w-auto' : 'opacity-0 w-0'}`}>
              <div className="text-sm font-semibold tracking-wide whitespace-nowrap">AiSprayer</div>
              <div className="text-[10px] text-slate-500 whitespace-nowrap">Robot Control Panel</div>
            </div>
          </div>
        </div>

        <nav className="flex-1 space-y-2 px-3 py-4">
          <div className={`px-3 py-2 text-xs font-medium uppercase tracking-wider text-slate-600 transition-opacity duration-300 ${isHovered ? 'opacity-100' : 'opacity-0 hidden'}`}>
            Modules
          </div>
          {tabs.map((tab) => {
            const Icon = tab.icon;
            const isActive = activeTab === tab.id;
            return (
              <button
                key={tab.id}
                onClick={() => setActiveTab(tab.id)}
                className={`flex w-full items-center gap-4 rounded-lg px-3 py-3 text-left text-sm transition-all duration-200 group ${
                  isActive
                    ? "bg-blue-500/10 text-blue-400 font-medium"
                    : "text-slate-400 hover:bg-white/[0.05] hover:text-slate-200"
                }`}
                title={!isHovered ? tab.label : undefined}
              >
                <div className="shrink-0 flex items-center justify-center relative ml-[-2px]">
                  {isActive && !isHovered && <div className="absolute -left-3 w-1 h-5 bg-blue-500 rounded-r-full"></div>}
                  {isActive && isHovered && <div className="absolute -left-3 w-1 h-full bg-blue-500 rounded-r-full"></div>}
                  <Icon size={18} className={isActive ? "text-blue-400" : "text-slate-500 group-hover:text-slate-300"} />
                </div>
                <span className={`whitespace-nowrap transition-opacity duration-300 ${isHovered ? 'opacity-100' : 'opacity-0 hidden'}`}>
                  {tab.label}
                </span>
              </button>
            );
          })}
        </nav>

        <div className="border-t border-slate-800 p-3 shrink-0">
          <button 
            onClick={() => setActiveTab('config')}
            className={`flex w-full items-center gap-4 rounded-lg px-3 py-3 text-sm transition-colors group ${
              activeTab === 'config' ? 'bg-blue-600 text-white' : 'text-slate-400 hover:bg-slate-800 hover:text-white'
            }`}
            title={!isHovered ? "System Config" : undefined}
          >
            <div className="shrink-0 flex items-center justify-center relative ml-[-2px]">
              <Settings size={18} className={activeTab === 'config' ? 'text-white' : 'text-slate-500 group-hover:text-slate-300'} />
            </div>
            <span className={`whitespace-nowrap transition-opacity duration-300 ${isHovered ? 'opacity-100' : 'opacity-0 hidden'}`}>
              System Config
            </span>
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
