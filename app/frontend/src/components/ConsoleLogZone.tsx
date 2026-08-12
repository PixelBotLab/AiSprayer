import React, { useState, useEffect, useRef } from 'react';
import { Terminal, Trash2 } from 'lucide-react';

interface LogEntry {
  time: string;
  level: string;
  logger: string;
  message: string;
}

const ConsoleLogZone: React.FC = () => {
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const scrollRef = useRef<HTMLDivElement>(null);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    const connect = () => {
      const ws = new WebSocket('ws://localhost:8000/api/system/logs/ws');
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
        setTimeout(connect, 3000);
      };
      wsRef.current = ws;
    };
    
    connect();
    
    return () => {
      if (wsRef.current) wsRef.current.close();
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

  return (
    <div className="w-full h-full flex flex-col bg-slate-950 rounded-xl border border-slate-800 shadow-inner overflow-hidden relative min-h-0">
      {/* Header */}
      <div className="shrink-0 px-4 py-2 flex items-center justify-between bg-slate-900 border-b border-slate-800">
        <h2 className="font-mono text-xs text-slate-400 flex items-center gap-2 uppercase tracking-widest">
          <Terminal size={14} className="text-slate-500" />
          System Console
        </h2>
        <div className="flex items-center gap-3">
          <button onClick={clearLogs} className="text-slate-500 hover:text-slate-300 transition-colors" title="Clear Console">
            <Trash2 size={14} />
          </button>
          <div className="flex gap-1.5">
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
