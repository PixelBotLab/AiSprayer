import React from 'react';
import { Save, Play, RefreshCw } from 'lucide-react';

interface CalibrationOpProps {
  samples: any[];
  isCapturing: boolean;
  onAddSample: () => void;
}

const CalibrationOp: React.FC<CalibrationOpProps> = ({ samples, isCapturing, onAddSample }) => {
  return (
    <div className="w-full h-full bg-slate-900/80 rounded-xl border border-slate-800 shadow-xl p-4 flex flex-col gap-4 backdrop-blur-sm min-h-[200px]">
      {/* Thumbnails Gallery */}
      <div className="flex-1 relative min-h-0">
        <div className="absolute top-2 left-2 z-10 text-[10px] font-medium text-blue-400 bg-slate-900/90 backdrop-blur-sm border border-blue-500/30 px-2.5 py-0.5 rounded-full shadow-md">
          {samples.length} Samples
        </div>
        
        <div className="w-full h-full border border-slate-800 border-dashed rounded-lg bg-slate-950/50 flex items-center p-3 gap-3 overflow-x-auto overflow-y-hidden custom-scrollbar pt-8">
          {samples.length === 0 ? (
            <div className="w-full h-full flex flex-col items-center justify-center text-slate-500 text-sm">
              <RefreshCw size={16} className="opacity-30 mb-2" />
              No calibration samples
            </div>
          ) : (
            <div className="flex items-center gap-3 h-full">
              {samples.map((s, idx) => (
                <div key={idx} className="shrink-0 h-full aspect-video bg-slate-800 rounded-lg border border-slate-700 relative overflow-hidden group shadow-md hover:border-blue-500/50 transition-colors cursor-pointer">
                  <div className="absolute inset-0 flex items-center justify-center text-slate-500 text-xs font-medium">Sample {idx + 1}</div>
                  <div className="absolute bottom-0 inset-x-0 bg-gradient-to-t from-slate-900 to-transparent p-2 text-[10px] text-slate-300 translate-y-full group-hover:translate-y-0 transition-transform">
                    View Details
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>
      </div>

      {/* Action Buttons */}
      <div className="flex gap-4 shrink-0 justify-end">
        <button 
          onClick={onAddSample}
          disabled={isCapturing}
          className="px-6 py-2 bg-slate-800 hover:bg-slate-700 border border-slate-600 text-slate-200 font-medium rounded-lg shadow-sm transition-all flex items-center justify-center gap-2 text-sm group active:scale-[0.98] disabled:opacity-50 disabled:cursor-not-allowed"
        >
          <Save size={16} className={`text-slate-400 transition-colors ${!isCapturing && 'group-hover:text-blue-400'}`} />
          {isCapturing ? 'Capturing...' : 'Capture Pose & Image'}
        </button>

        <button className="px-6 py-2 bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-500 hover:to-indigo-500 text-white font-medium rounded-lg shadow-lg shadow-blue-900/20 transition-all flex items-center justify-center gap-2 text-sm active:scale-[0.98]">
          <Play size={16} fill="currentColor" />
          Execute Calibration
        </button>
      </div>
    </div>
  );
};

export default CalibrationOp;
