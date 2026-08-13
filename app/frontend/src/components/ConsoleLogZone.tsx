import React, { useState, useEffect, useRef } from 'react';
import { Terminal, Trash2, Maximize2, Minimize2 } from 'lucide-react';

interface LogEntry {
  time: string;
  level: string;
  logger: string;
  message: string;
}

const ConsoleLogZone: React.FC = () => {
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [isMaximized, setIsMaximized] = useState(false);
  const scrollRef = useRef<HTMLDivElement>(null);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    let isMounted = true;
    let ws: WebSocket | null = null;
    let reconnectTimeout: NodeJS.Timeout;

    const connect = () => {
      if (!isMounted) return;
      ws = new WebSocket('ws://localhost:8000/api/system/logs/ws');
      ws.onmessage = (e) => {
        try {
          const entry: LogEntry = JSON.parse(e.data);
          setLogs(prev => {
            const newLogs = [...prev, entry];
            // Keep maximum 500 lines to prevent DOM bloat
            if (newLogs.length > 500) return newLogs.slice(newLogs.length - 500);
            return newLogs;
          });
        } catch {}
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
        ws.onclose = null; // Prevent reconnect logic
        ws.close();
      }
    };
  }, []);

  // Auto-scroll to bottom
  useEffect(() => {
    if (scrollRef.current) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [logs]);

  const getLevelColor = (level: string) => {
    switch (level) {
      case 'DEBUG': return 'text-slate-400';
      case 'INFO': return 'text-blue-400';
      case 'WARNING': return 'text-amber-400';
      case 'ERROR': 
      case 'CRITICAL': return 'text-red-400';
      default: return 'text-slate-300';
    }
  };

  const clearLogs = () => setLogs([]);

  const containerClasses = isMaximized
    ? "fixed inset-4 md:inset-8 z-[100] flex flex-col bg-slate-950/98 rounded-xl border border-slate-700/80 shadow-2xl overflow-hidden backdrop-blur-xl transition-all duration-200 animate-in fade-in"
    : "w-full h-full flex flex-col bg-slate-950 rounded-xl border border-slate-800 shadow-inner overflow-hidden relative min-h-0";

  return (
    <div className={containerClasses}>
      {/* Header */}
      <div className="shrink-0 px-4 py-2 flex items-center justify-between bg-slate-900 border-b border-slate-800">
        <h2 className="font-mono text-xs text-slate-400 flex items-center gap-2 uppercase tracking-widest">
          <Terminal size={14} className="text-slate-500" />
          System Console {isMaximized ? '(Fullscreen Mode)' : ''}
        </h2>
        <div className="flex items-center gap-3">
          <button onClick={clearLogs} className="text-slate-500 hover:text-slate-300 transition-colors p-1 rounded hover:bg-slate-800" title="Clear Console">
            <Trash2 size={14} />
          </button>
          <button 
            onClick={() => setIsMaximized(!isMaximized)} 
            className={`transition-all flex items-center gap-1.5 text-xs rounded-md ${
              isMaximized 
                ? 'px-2.5 py-1 bg-blue-600 hover:bg-blue-500 text-white font-medium shadow-md shadow-blue-900/30' 
                : 'p-1 text-slate-400 hover:text-white hover:bg-slate-800'
            }`} 
            title={isMaximized ? "Exit Fullscreen" : "Fullscreen Console"}
          >
            {isMaximized ? (
              <>
                <Minimize2 size={14} />
                <span>Exit Fullscreen</span>
              </>
            ) : (
              <Maximize2 size={14} />
            )}
          </button>
          <div className="flex gap-1.5 ml-1">
            <div className="w-2.5 h-2.5 rounded-full bg-slate-700"></div>
            <div className="w-2.5 h-2.5 rounded-full bg-slate-700"></div>
            <div className="w-2.5 h-2.5 rounded-full bg-slate-700"></div>
          </div>
        </div>
      </div>

      {/* Logs Area */}
      <div ref={scrollRef} className="flex-1 p-4 font-mono text-[11px] text-slate-300 leading-relaxed overflow-y-auto custom-scrollbar flex flex-col gap-1 scroll-smooth">
        <div className="text-slate-500 mb-2"># AiSprayer Backend Log Stream</div>
        
        {logs.map((log, idx) => (
          <div key={idx} className="flex gap-3 hover:bg-slate-900/50 px-1 -mx-1 rounded transition-colors break-all">
            <span className="text-slate-500 shrink-0 select-none">[{log.time}]</span>
            <span className={`${getLevelColor(log.level)} font-semibold shrink-0 select-none w-16`}>[{log.level}]</span>
            <span className="text-slate-500 shrink-0 select-none hidden md:inline-block w-32 truncate" title={log.logger}>{log.logger}:</span>
            <span className="text-slate-200">{log.message}</span>
          </div>
        ))}
        
        {/* Blinking cursor */}
        <div className="mt-2 flex items-center text-slate-500 shrink-0">
          <span className="mr-2">&gt;</span>
          <span className="w-1.5 h-3 bg-slate-400 animate-pulse inline-block"></span>
        </div>
      </div>
    </div>
  );
};

export default ConsoleLogZone;
