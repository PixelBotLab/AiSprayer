import React, { useState, useEffect, useRef, useMemo, useCallback } from 'react';
import {
  Terminal,
  Camera,
  Bot,
  Layers,
  Trash2,
  Maximize2,
  Minimize2,
  Search,
  CheckCheck,
  ArrowDown
} from 'lucide-react';
import { WS_BASE } from '../config';

export interface LogEntry {
  time: string;
  level: string;
  logger: string;
  message: string;
}

export type LogTabType = 'system' | 'camera' | 'robot';
export type LogLevelFilter = 'ALL' | 'ERROR' | 'WARNING' | 'INFO';

interface UnreadStats {
  error: number;
  warn: number;
  info: number;
}

// Strictly categorize log entry by logger module name to mutually exclusive domains: Camera, Robot, or System
function categorizeLog(log: LogEntry): LogTabType {
  const loggerName = (log.logger || '').toLowerCase();
  
  // 1. Camera domain loggers (C++ Camera Service, Orbbec driver, Camera FastAPI)
  if (
    loggerName === 'camera.cpp' ||
    loggerName.includes('camera') ||
    loggerName.includes('orbbec') ||
    loggerName.includes('realsense')
  ) {
    return 'camera';
  }
  
  // 2. Robot domain loggers (robot_service, dobot_driver, inexbot_driver, robot_trajectory_controller, etc.)
  if (
    loggerName.includes('robot') ||
    loggerName.includes('dobot') ||
    loggerName.includes('inexbot')
  ) {
    return 'robot';
  }
  
  // 3. All other loggers strictly belong to System & Business (main, reconstruction_service, sam_service, interactive, calib, uvicorn, etc.)
  return 'system';
}

const ConsoleLogZone: React.FC = () => {
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [activeTab, setActiveTab] = useState<LogTabType>('system');
  const [levelFilter, setLevelFilter] = useState<LogLevelFilter>('ALL');
  const [searchQuery, setSearchQuery] = useState('');
  const [isMaximized, setIsMaximized] = useState(false);
  const [autoScroll, setAutoScroll] = useState(true);

  // Unread counters per tab for error/warning/info notification lights
  const [unread, setUnread] = useState<Record<LogTabType, UnreadStats>>({
    system: { error: 0, warn: 0, info: 0 },
    camera: { error: 0, warn: 0, info: 0 },
    robot: { error: 0, warn: 0, info: 0 }
  });

  const scrollRef = useRef<HTMLDivElement>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const activeTabRef = useRef<LogTabType>(activeTab);

  useEffect(() => {
    activeTabRef.current = activeTab;
  }, [activeTab]);

  // Handle new incoming log entry
  const handleNewLog = useCallback((entry: LogEntry) => {
    const category = categorizeLog(entry);
    const lvl = entry.level.toUpperCase();
    const isErr = lvl === 'ERROR' || lvl === 'CRITICAL';
    const isWarn = lvl === 'WARN' || lvl === 'WARNING';
    const currentActive = activeTabRef.current;

    setLogs(prev => {
      const next = [...prev, entry];
      if (next.length > 1000) return next.slice(next.length - 1000);
      return next;
    });

    // Update unread stats for tabs that are NOT currently active
    if (category !== currentActive) {
      setUnread(prev => ({
        ...prev,
        [category]: {
          error: prev[category].error + (isErr ? 1 : 0),
          warn: prev[category].warn + (isWarn ? 1 : 0),
          info: prev[category].info + (!isErr && !isWarn ? 1 : 0)
        }
      }));
    }
  }, []);

  // WebSocket log stream subscription
  useEffect(() => {
    let isMounted = true;
    let ws: WebSocket | null = null;
    let reconnectTimeout: ReturnType<typeof setTimeout>;

    const connect = () => {
      if (!isMounted) return;
      ws = new WebSocket(`${WS_BASE}/api/system/logs/ws`);

      ws.onmessage = (e) => {
        try {
          const entry: LogEntry = JSON.parse(e.data);
          handleNewLog(entry);
        } catch {
          // ignore malformed frame
        }
      };

      ws.onclose = () => {
        if (isMounted) {
          reconnectTimeout = setTimeout(connect, 3000);
        }
      };

      wsRef.current = ws;
    };

    connect();

    return () => {
      isMounted = false;
      clearTimeout(reconnectTimeout);
      if (ws) {
        ws.onclose = null;
        ws.close();
      }
    };
  }, [handleNewLog]);

  // Auto-scroll when logs update
  useEffect(() => {
    if (autoScroll && scrollRef.current) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [logs, autoScroll, activeTab]);

  // Handle user scroll (pause auto-scroll if user scrolled up)
  const handleScroll = () => {
    if (!scrollRef.current) return;
    const { scrollTop, scrollHeight, clientHeight } = scrollRef.current;
    const isAtBottom = scrollHeight - scrollTop - clientHeight < 40;
    setAutoScroll(isAtBottom);
  };

  // Switch Tab and immediately clear unread indicator light & count
  const handleTabClick = (tab: LogTabType) => {
    setActiveTab(tab);
    setUnread(prev => ({
      ...prev,
      [tab]: { error: 0, warn: 0, info: 0 }
    }));
  };

  // Clear all unread badges across all tabs
  const handleMarkAllRead = () => {
    setUnread({
      system: { error: 0, warn: 0, info: 0 },
      camera: { error: 0, warn: 0, info: 0 },
      robot: { error: 0, warn: 0, info: 0 }
    });
  };

  // Clear logs in current view or all
  const clearLogs = () => {
    setLogs([]);
    handleMarkAllRead();
  };

  // Filtered log items: strictly mutually exclusive per tab
  const filteredLogs = useMemo(() => {
    return logs.filter(log => {
      // 1. Strict domain category match
      const cat = categorizeLog(log);
      if (cat !== activeTab) return false;

      // 2. Level filter
      if (levelFilter !== 'ALL') {
        const lvl = log.level.toUpperCase();
        if (levelFilter === 'ERROR' && lvl !== 'ERROR' && lvl !== 'CRITICAL') return false;
        if (levelFilter === 'WARNING' && lvl !== 'WARN' && lvl !== 'WARNING') return false;
        if (levelFilter === 'INFO' && lvl !== 'INFO' && lvl !== 'DEBUG') return false;
      }

      // 3. Search query filter
      if (searchQuery.trim()) {
        const q = searchQuery.toLowerCase();
        const msg = log.message.toLowerCase();
        const loggerName = log.logger.toLowerCase();
        if (!msg.includes(q) && !loggerName.includes(q)) return false;
      }

      return true;
    });
  }, [logs, activeTab, levelFilter, searchQuery]);

  const getLevelColor = (level: string) => {
    switch (level.toUpperCase()) {
      case 'DEBUG':
        return 'text-slate-400';
      case 'INFO':
        return 'text-blue-400';
      case 'WARN':
      case 'WARNING':
        return 'text-amber-400';
      case 'ERROR':
      case 'CRITICAL':
        return 'text-red-400';
      default:
        return 'text-slate-300';
    }
  };

  const getLevelBadge = (level: string) => {
    switch (level.toUpperCase()) {
      case 'DEBUG':
        return 'bg-slate-800 text-slate-400 border-slate-700';
      case 'INFO':
        return 'bg-blue-950/60 text-blue-400 border-blue-800/50';
      case 'WARN':
      case 'WARNING':
        return 'bg-amber-950/60 text-amber-400 border-amber-800/60';
      case 'ERROR':
      case 'CRITICAL':
        return 'bg-red-950/70 text-red-400 border-red-800/80 font-bold';
      default:
        return 'bg-slate-800 text-slate-300 border-slate-700';
    }
  };

  // Render Indicator Light and Severity Badge for each Tab header
  const renderTabBadge = (tab: LogTabType) => {
    const stats = unread[tab];
    
    // Priority 1: Unread Errors (Red Blinking Light & Red Badge)
    if (stats.error > 0) {
      return (
        <span
          className="flex items-center gap-1 ml-1.5 px-1.5 py-0.5 rounded-full bg-red-500/20 border border-red-500/50 text-red-400 text-[10px] font-mono font-bold shadow-[0_0_8px_rgba(239,68,68,0.6)] animate-pulse"
          title={`${stats.error} unread errors`}
        >
          <span className="w-1.5 h-1.5 rounded-full bg-red-500"></span>
          <span>{stats.error}</span>
        </span>
      );
    }

    // Priority 2: Unread Warnings (Yellow Light & Yellow Badge)
    if (stats.warn > 0) {
      return (
        <span
          className="flex items-center gap-1 ml-1.5 px-1.5 py-0.5 rounded-full bg-amber-500/20 border border-amber-500/50 text-amber-400 text-[10px] font-mono font-semibold"
          title={`${stats.warn} unread warnings`}
        >
          <span className="w-1.5 h-1.5 rounded-full bg-amber-400"></span>
          <span>{stats.warn}</span>
        </span>
      );
    }

    // Priority 3: Unread Normal Info (Green Dot / Count)
    if (stats.info > 0) {
      return (
        <span
          className="flex items-center gap-1 ml-1 px-1.5 py-0.5 rounded-full bg-emerald-500/10 border border-emerald-500/30 text-emerald-400 text-[9px] font-mono"
          title={`${stats.info} new logs`}
        >
          <span className="w-1.5 h-1.5 rounded-full bg-emerald-400"></span>
          <span>{stats.info}</span>
        </span>
      );
    }

    return null;
  };

  // Tabs configured with System/Business Logs first
  const tabsConfig = [
    { id: 'system' as LogTabType, label: 'System & Business', icon: Layers },
    { id: 'camera' as LogTabType, label: 'Camera', icon: Camera },
    { id: 'robot' as LogTabType, label: 'Robot', icon: Bot }
  ];

  const containerClasses = isMaximized
    ? 'fixed inset-4 md:inset-8 z-[100] flex flex-col bg-slate-950/98 rounded-xl border border-slate-700/80 shadow-2xl overflow-hidden backdrop-blur-xl transition-all duration-200 animate-in fade-in'
    : 'w-full h-full flex flex-col bg-slate-950 rounded-xl border border-slate-800 shadow-inner overflow-hidden relative min-h-0';

  return (
    <div className={containerClasses}>
      {/* 1. Header Toolbar with Tabs & Indicator Lights */}
      <div className="shrink-0 bg-gradient-to-b from-slate-900 to-slate-950 border-b border-slate-800 px-3 py-2 flex flex-wrap items-center justify-between gap-2 select-none">
        {/* Category Tabs */}
        <div className="flex items-center gap-1 overflow-x-auto custom-scrollbar pb-0.5">
          {tabsConfig.map(tab => {
            const Icon = tab.icon;
            const isActive = activeTab === tab.id;
            return (
              <button
                key={tab.id}
                onClick={() => handleTabClick(tab.id)}
                className={`flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-medium transition-all cursor-pointer ${
                  isActive
                    ? 'bg-slate-800 text-blue-400 border border-slate-700 shadow-sm'
                    : 'text-slate-400 hover:text-slate-200 hover:bg-slate-900/80 border border-transparent'
                }`}
              >
                <Icon size={14} className={isActive ? 'text-blue-400' : 'text-slate-500'} />
                <span>{tab.label}</span>
                {renderTabBadge(tab.id)}
              </button>
            );
          })}
        </div>

        {/* Action Tools & Filters */}
        <div className="flex items-center gap-2">
          {/* Quick Search */}
          <div className="relative flex items-center">
            <Search size={12} className="absolute left-2.5 text-slate-500 pointer-events-none" />
            <input
              type="text"
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              placeholder="Search logs..."
              className="bg-slate-900 border border-slate-700/80 rounded-md pl-7 pr-2 py-1 text-xs text-slate-200 placeholder-slate-500 focus:outline-none focus:border-blue-500 w-28 sm:w-36 transition-all"
            />
          </div>

          {/* Level Filter Selector */}
          <div className="flex items-center bg-slate-900 border border-slate-800 rounded-md p-0.5 text-[11px] font-mono">
            {(['ALL', 'ERROR', 'WARNING', 'INFO'] as LogLevelFilter[]).map(lvl => (
              <button
                key={lvl}
                onClick={() => setLevelFilter(lvl)}
                className={`px-2 py-0.5 rounded transition-colors cursor-pointer ${
                  levelFilter === lvl
                    ? lvl === 'ERROR'
                      ? 'bg-red-950 text-red-300 font-semibold'
                      : lvl === 'WARNING'
                      ? 'bg-amber-950 text-amber-300 font-semibold'
                      : lvl === 'INFO'
                      ? 'bg-blue-950 text-blue-300 font-semibold'
                      : 'bg-slate-700 text-slate-100 font-semibold'
                    : 'text-slate-400 hover:text-slate-200'
                }`}
              >
                {lvl}
              </button>
            ))}
          </div>

          {/* Mark All Read Button */}
          <button
            onClick={handleMarkAllRead}
            className="p-1.5 text-slate-500 hover:text-slate-300 hover:bg-slate-800 rounded transition-colors"
            title="Mark all as read (Dismiss indicators)"
          >
            <CheckCheck size={14} />
          </button>

          {/* Clear Logs Button */}
          <button
            onClick={clearLogs}
            className="p-1.5 text-slate-500 hover:text-red-400 hover:bg-slate-800 rounded transition-colors"
            title="Clear console logs"
          >
            <Trash2 size={14} />
          </button>

          {/* Maximize / Minimize Button */}
          <button
            onClick={() => setIsMaximized(!isMaximized)}
            className={`transition-all flex items-center gap-1 text-xs rounded-md ${
              isMaximized
                ? 'px-2 py-1 bg-blue-600 hover:bg-blue-500 text-white font-medium shadow-md shadow-blue-900/30'
                : 'p-1.5 text-slate-400 hover:text-white hover:bg-slate-800'
            }`}
            title={isMaximized ? 'Exit Fullscreen' : 'Fullscreen Console'}
          >
            {isMaximized ? <Minimize2 size={14} /> : <Maximize2 size={14} />}
          </button>
        </div>
      </div>

      {/* 2. Logs Display Area */}
      <div
        ref={scrollRef}
        onScroll={handleScroll}
        className="flex-1 p-3.5 font-mono text-[11px] text-slate-300 leading-relaxed overflow-y-auto custom-scrollbar flex flex-col gap-1 select-text scroll-smooth"
      >
        {/* Stream Status Bar */}
        <div className="text-slate-500 mb-1.5 flex items-center justify-between text-[10.5px] border-b border-slate-800/80 pb-1.5 select-none">
          <div className="flex items-center gap-2">
            <span className="w-2 h-2 rounded-full bg-emerald-500 animate-pulse"></span>
            <span>Realtime Stream</span>
            <span className="text-slate-600">|</span>
            <span className="text-slate-400">
              Showing {filteredLogs.length} of {logs.length} entries
            </span>
          </div>
          <div className="flex items-center gap-2">
            {!autoScroll && (
              <button
                onClick={() => {
                  setAutoScroll(true);
                  if (scrollRef.current) scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
                }}
                className="flex items-center gap-1 text-blue-400 hover:text-blue-300 bg-blue-950/60 border border-blue-800/50 px-2 py-0.5 rounded text-[10px] cursor-pointer"
              >
                <ArrowDown size={11} />
                <span>Scroll to bottom</span>
              </button>
            )}
            <span className="text-slate-600">{autoScroll ? 'Auto-scroll ON' : 'Paused'}</span>
          </div>
        </div>

        {/* Log Entries */}
        {filteredLogs.length === 0 ? (
          <div className="text-slate-600 italic py-12 text-center select-none flex flex-col items-center gap-2">
            <Terminal size={24} className="opacity-30" />
            <span>No log entries match the current filter</span>
          </div>
        ) : (
          filteredLogs.map((log, idx) => {
            const isMultiline = log.message.includes('\n');
            if (isMultiline) {
              return (
                <div
                  key={idx}
                  className="flex flex-col hover:bg-slate-900/50 px-2.5 py-1.5 rounded-lg transition-colors my-1 border border-slate-800/70 bg-slate-950/80"
                >
                  <div className="flex items-center gap-2 text-[10.5px] border-b border-slate-800/60 pb-1 mb-1 select-none">
                    <span className="text-slate-500">[{log.time}]</span>
                    <span className={`px-1.5 py-0.2 rounded border text-[9.5px] ${getLevelBadge(log.level)}`}>
                      {log.level}
                    </span>
                    <span className="text-slate-400 font-medium">{log.logger}:</span>
                  </div>
                  <pre className="font-mono text-[10.5px] leading-snug text-slate-200 overflow-x-auto whitespace-pre custom-scrollbar py-1 select-text">
                    {log.message}
                  </pre>
                </div>
              );
            }
            return (
              <div
                key={idx}
                className="flex items-start gap-2.5 hover:bg-slate-900/60 px-2 py-0.5 rounded transition-colors group"
              >
                <span className="text-slate-500 shrink-0 select-none text-[10.5px]">[{log.time}]</span>
                <span
                  className={`px-1.5 py-0.2 rounded border text-[9.5px] shrink-0 select-none ${getLevelBadge(
                    log.level
                  )}`}
                >
                  {log.level}
                </span>
                <span
                  className="text-slate-500 shrink-0 select-none hidden sm:inline-block w-28 truncate text-[10.5px]"
                  title={log.logger}
                >
                  {log.logger}:
                </span>
                <span
                  className={`whitespace-pre-wrap break-words flex-1 text-[11px] ${getLevelColor(log.level)}`}
                >
                  {log.message}
                </span>
              </div>
            );
          })
        )}

        {/* Blinking Terminal Prompt Cursor */}
        <div className="mt-2 flex items-center text-slate-500 shrink-0 select-none">
          <span className="mr-2">&gt;</span>
          <span className="w-1.5 h-3 bg-slate-400 animate-pulse inline-block"></span>
        </div>
      </div>
    </div>
  );
};

export default ConsoleLogZone;
