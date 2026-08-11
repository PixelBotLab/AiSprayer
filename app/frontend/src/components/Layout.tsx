import React from 'react';
import { Settings } from 'lucide-react';

interface LayoutProps {
  children: React.ReactNode;
  activeTab: string;
  setActiveTab: (tab: string) => void;
}

const Layout: React.FC<LayoutProps> = ({ children, activeTab, setActiveTab }) => {
  const tabs = [
    { id: 'calib', label: '手眼标定 (Calib)' },
    { id: 'interactive', label: '交互示教 (Interactive)' },
    { id: 'auto_planner', label: '3D规划 (Planner)' },
    { id: 'digital_twin', label: '数字孪生 (Twin)' },
  ];

  return (
    <div className="flex h-screen bg-slate-950 font-sans text-white overflow-hidden">
      {/* Sidebar - Matching aibox-mis console style */}
      <aside className="flex w-64 shrink-0 flex-col overflow-y-auto border-r border-slate-800 bg-slate-900/80">
        <div className="border-b border-slate-800 p-5">
          <div className="flex items-center gap-3">
            <div className="flex h-9 w-9 items-center justify-center rounded-lg bg-gradient-to-br from-blue-500 to-indigo-600 shadow-lg shadow-blue-500/20">
              <svg className="h-5 w-5 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5} d="M9.75 17L9 20l-1 1h8l-1-1-.75-3M3 13h18M5 17h14a2 2 0 002-2V5a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z"/>
              </svg>
            </div>
            <div>
              <div className="text-sm font-semibold tracking-wide">AiSprayer</div>
              <div className="text-xs text-slate-500">
                Robot Control Panel
              </div>
            </div>
          </div>
        </div>

        <nav className="flex-1 space-y-1 px-3 py-4">
          <div className="px-3 py-2 text-xs font-medium uppercase tracking-wider text-slate-600">Modules</div>
          {tabs.map((tab) => (
            <button
              key={tab.id}
              onClick={() => setActiveTab(tab.id)}
              className={`flex w-full items-center gap-3 rounded-lg px-3 py-2.5 text-left text-sm transition-all duration-200 ${
                activeTab === tab.id
                  ? "border-r-2 border-blue-500 bg-blue-500/10 text-blue-400 font-medium"
                  : "text-slate-400 hover:bg-white/[0.03] hover:text-slate-200"
              }`}
            >
              {tab.label}
            </button>
          ))}
        </nav>

        <div className="border-t border-slate-800 p-4 space-y-4">
          {/* Status indicators */}
          <div className="space-y-2">
            <div className="flex items-center justify-between px-2 text-xs text-slate-400">
              <span>Robot</span>
              <span className="flex items-center gap-2">
                <span className="h-2 w-2 rounded-full bg-slate-600" /> Offline
              </span>
            </div>
            <div className="flex items-center justify-between px-2 text-xs text-slate-400 pb-2">
              <span>Camera</span>
              <span className="flex items-center gap-2">
                <span className="h-2 w-2 rounded-full bg-emerald-500 shadow-[0_0_8px_rgba(16,185,129,0.8)] animate-pulse" /> Active
              </span>
            </div>
          </div>

          <button 
            onClick={() => setActiveTab('config')}
            className={`flex w-full items-center justify-center gap-2 rounded-lg border py-2 text-sm transition-colors ${activeTab === 'config' ? 'bg-blue-600 border-blue-500 text-white' : 'border-slate-700 text-slate-300 hover:bg-slate-800 hover:text-white'}`}
          >
            <Settings size={16} /> System Config
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
