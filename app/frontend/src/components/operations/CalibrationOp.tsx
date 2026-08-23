import React, { useState, useEffect, useRef } from 'react';
import { Play, FolderPlus, Trash2, Image as ImageIcon, Camera, ChevronLeft, ChevronRight } from 'lucide-react';
import { CustomModal, type ModalConfig } from '../common/CustomModal';
import { API_BASE } from '../../config';

const CalibrationOp: React.FC = () => {
  const [sessions, setSessions] = useState<string[]>([]);
  const [activeSession, setActiveSession] = useState<string | null>(null);
  const [sessionData, setSessionData] = useState<{ samples: any[], result: any, mode?: string }>({ samples: [], result: null });
  const [activeImage, setActiveImage] = useState<string | null>(null);
  const [isCapturing, setIsCapturing] = useState(false);
  const [isRunning, setIsRunning] = useState(false);
  const [progressData, setProgressData] = useState<{current: number, total: number, status: string} | null>(null);
  const scrollRef = useRef<HTMLDivElement>(null);

  // Custom Modal State
  const [modalConfig, setModalConfig] = useState<ModalConfig>({
    isOpen: false,
    type: 'info',
    title: '',
    message: ''
  });

  const showAlert = (title: string, message: string) => {
    setModalConfig({
      isOpen: true,
      type: 'alert',
      title,
      message,
      confirmText: 'Understood'
    });
  };

  const fetchSessions = async (preferredSession?: string) => {
    try {
      const res = await fetch(`${API_BASE}/api/calib/sessions`);
      if (res.ok) {
        const data = await res.json();
        const list: string[] = data.sessions || [];
        setSessions(list);
        if (list.length > 0) {
          const target = (preferredSession && list.includes(preferredSession))
            ? preferredSession
            : (activeSession && list.includes(activeSession))
            ? activeSession
            : list[0];
          setActiveSession(target);
          fetchSessionData(target);
        } else {
          setActiveSession(null);
          setActiveImage(null);
          setSessionData({ samples: [], result: null });
        }
      }
    } catch (err) {
      console.error('Failed to fetch sessions:', err);
    }
  };

  let calibrationModeTimeout: any = null;

  useEffect(() => {
    fetchSessions();

    if (calibrationModeTimeout) {
      clearTimeout(calibrationModeTimeout);
      calibrationModeTimeout = null;
    }

    // Enable calibration mode for live camera feed
    fetch(`${API_BASE}/api/system/camera/calibration_mode`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: true })
    }).catch(console.error);

    return () => {
      calibrationModeTimeout = setTimeout(() => {
        fetch(`${API_BASE}/api/system/camera/calibration_mode`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ enabled: false })
        }).catch(console.error);
      }, 100);
    };
  }, []);

  const fetchSessionData = async (sessionId: string) => {
    try {
      const res = await fetch(`${API_BASE}/api/calib/sessions/${sessionId}`);
      if (res.ok) {
        const data = await res.json();
        setSessionData({ 
          samples: data.samples || [], 
          result: data.result || null,
          mode: data.mode || undefined
        });
        if (data.samples?.length > 0) {
          setActiveImage(data.samples[0].filename);
        } else {
          setActiveImage(null);
        }
      }
    } catch (err) {
      console.error('Failed to fetch session data:', err);
    }
  };

  useEffect(() => {
    if (!activeSession) return;
    fetchSessionData(activeSession);
  }, [activeSession]);

  useEffect(() => {
    if (activeImage) {
      const el = document.getElementById(`sample-thumb-${activeImage}`);
      if (el) {
        el.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
      }
    }
  }, [activeImage]);

  const handleCreateSession = async () => {
    try {
      const res = await fetch(`${API_BASE}/api/calib/sessions/new`, { method: 'POST' });
      if (res.ok) {
        const data = await res.json();
        await fetchSessions();
        setActiveSession(data.session_id);
      }
    } catch (err) {
      console.error('Failed to create session:', err);
    }
  };

  const handleDeleteSession = (sessionToDelete?: string | React.MouseEvent) => {
    const targetSession = typeof sessionToDelete === 'string' ? sessionToDelete : activeSession;
    if (!targetSession) return;
    setModalConfig({
      isOpen: true,
      type: 'confirm',
      title: 'Delete Calibration Session',
      message: `Are you sure you want to permanently delete session "${targetSession}"? All captured samples and calibration results will be removed.`,
      confirmText: 'Delete Session',
      isDanger: true,
      onConfirm: async () => {
        try {
          const res = await fetch(`${API_BASE}/api/calib/sessions/${targetSession}`, { method: 'DELETE' });
          if (res.ok) {
            if (activeSession === targetSession) {
              setActiveSession(null);
              setActiveImage(null);
              setSessionData({ samples: [], result: null });
            }
            fetchSessions();
          }
        } catch (err) {
          console.error('Failed to delete session:', err);
        }
      }
    });
  };

  const handleCapture = async () => {
    if (!activeSession || isCapturing) return;
    setIsCapturing(true);
    try {
      const res = await fetch(`${API_BASE}/api/calib/sessions/${activeSession}/samples`, { method: 'POST' });
      if (res.ok) {
        const dataRes = await fetch(`${API_BASE}/api/calib/sessions/${activeSession}`);
        if (dataRes.ok) {
          const data = await dataRes.json();
          setSessionData({ 
            samples: data.samples || [], 
            result: data.result || null,
            mode: data.mode || undefined
          });
          if (data.samples?.length > 0) {
            setActiveImage(data.samples[data.samples.length - 1].filename);
          }
        }
      } else {
        const err = await res.json();
        showAlert('Sample Capture Failed', err.detail || 'Failed to capture sample from camera feed.');
      }
    } catch (err: any) {
      showAlert('Capture Error', err.message);
    } finally {
      setIsCapturing(false);
    }
  };

  const handleRunCalibration = async () => {
    if (!activeSession || isRunning) return;
    if (sessionData.samples.length < 3) {
      showAlert('Insufficient Samples', 'At least 3 valid calibration samples are required to solve camera extrinsics.');
      return;
    }
    setIsRunning(true);
    setProgressData({ current: 0, total: sessionData.samples.length, status: 'started' });
    try {
      const res = await fetch(`${API_BASE}/api/calib/sessions/${activeSession}/run`, { method: 'POST' });
      if (!res.ok) {
        throw new Error('Failed to start calibration task');
      }

      const eventSource = new EventSource(`${API_BASE}/api/calib/sessions/${activeSession}/progress`);
      
      eventSource.onmessage = (event) => {
        const data = JSON.parse(event.data);
        if (data.status === 'waiting') return;
        
        setProgressData({
          current: data.current || 0,
          total: data.total || 1,
          status: data.status
        });
        
        if (data.filename) {
          setActiveImage(data.filename);
        }

        if (data.status === 'completed' || data.status === 'error') {
          eventSource.close();
          setIsRunning(false);
          setProgressData(null);
          
          if (data.status === 'completed') {
            fetchSessionData(activeSession);
          } else {
            showAlert('Calibration Failed', 'Optimization failed to converge. Please inspect corner detections.');
          }
        }
      };
      
      eventSource.onerror = () => {
        eventSource.close();
        setIsRunning(false);
        setProgressData(null);
      };
    } catch (err: any) {
      showAlert('Execution Error', err.message);
      setIsRunning(false);
      setProgressData(null);
    }
  };

  const activeImageUrl = activeSession && activeImage 
    ? `${API_BASE}/api/calib/sessions/${activeSession}/images_with_corners/${activeImage}`
    : null;

  const scrollTabs = (dir: 'left' | 'right') => {
    if (scrollRef.current) {
      scrollRef.current.scrollBy({ left: dir === 'left' ? -200 : 200, behavior: 'smooth' });
    }
  };

  return (
    <div className="w-full h-full flex flex-col bg-slate-950 overflow-hidden relative font-sans select-none rounded-xl border border-slate-800">
      
      {/* Custom Sleek Modal */}
      <CustomModal config={modalConfig} onClose={() => setModalConfig(prev => ({ ...prev, isOpen: false }))} />

      {/* TOP BAR: Sessions */}
      <div className="h-9 bg-slate-900 border-b border-slate-800 flex items-center px-2 justify-between select-none shrink-0 z-10 gap-1.5">
        {/* Left Action: New Session Button */}
        <button
          onClick={handleCreateSession}
          className="p-1 rounded bg-slate-800 hover:bg-slate-700 text-sky-400 border border-slate-700 shrink-0 transition-colors flex items-center gap-1 text-xs font-medium px-2"
          title="Create New Session"
        >
          <FolderPlus size={13} />
          <span>New</span>
        </button>

        <div className="h-4 w-[1px] bg-slate-800 shrink-0" />

        {/* Left Scroll Arrow */}
        <button
          onClick={() => scrollTabs('left')}
          className="p-1 hover:bg-slate-800 text-slate-400 hover:text-slate-200 rounded transition-colors shrink-0"
          title="Scroll Left"
        >
          <ChevronLeft size={15} />
        </button>
        
        {/* Center Sessions Container */}
        <div
          ref={scrollRef}
          className="flex-1 flex items-center gap-1.5 overflow-x-hidden py-0.5 scroll-smooth"
        >
          {sessions.map((session) => {
            const isActive = activeSession === session;
            return (
              <div
                key={session}
                className={`group flex items-center gap-1.5 px-2.5 py-0.5 rounded-md text-xs font-medium transition-all shrink-0 cursor-pointer border ${
                  isActive
                    ? 'bg-sky-950/80 text-sky-300 border-sky-500/50 shadow-sm'
                    : 'bg-slate-800/60 text-slate-400 border-transparent hover:bg-slate-800 hover:text-slate-200'
                }`}
                onClick={() => setActiveSession(session)}
              >
                <span>{session}</span>
                <button
                  onClick={(e) => {
                    e.stopPropagation();
                    handleDeleteSession(session);
                  }}
                  className="opacity-0 group-hover:opacity-100 hover:text-rose-400 p-0.5 rounded transition-opacity"
                  title="Delete Session"
                >
                  <Trash2 size={11} />
                </button>
              </div>
            );
          })}
        </div>

        {/* Right Scroll Arrow */}
        <button
          onClick={() => scrollTabs('right')}
          className="p-1 hover:bg-slate-800 text-slate-400 hover:text-slate-200 rounded transition-colors shrink-0"
          title="Scroll Right"
        >
          <ChevronRight size={15} />
        </button>
      </div>

      {/* MAIN CONTENT: 3 Columns */}
      <div className="flex-1 flex min-h-0">
        
        {/* Left Column: Big Image Viewer */}
        <div className="flex-1 flex flex-col border-r border-slate-800 relative bg-black">
          {activeImageUrl ? (
            <img 
              src={activeImageUrl} 
              className="w-full h-full object-contain select-none" 
              alt="calibration sample" 
            />
          ) : (
            <div className="w-full h-full flex items-center justify-center text-slate-700">
              <ImageIcon size={48} className="opacity-20" />
            </div>
          )}

          {/* Progress Overlay */}
          {isRunning && progressData && (
            <div className="absolute inset-x-0 bottom-0 bg-slate-950/80 backdrop-blur-sm p-4 border-t border-emerald-900/50 flex flex-col gap-2">
              <div className="flex justify-between items-center text-xs">
                <span className="text-emerald-400 font-bold uppercase tracking-wider">{progressData.status}</span>
                <span className="text-slate-400 font-mono">{progressData.current} / {progressData.total}</span>
              </div>
              <div className="w-full h-1.5 bg-slate-800 rounded-full overflow-hidden">
                <div 
                  className="h-full bg-emerald-500 transition-all duration-300"
                  style={{ width: `${(progressData.current / Math.max(1, progressData.total)) * 100}%` }}
                />
              </div>
            </div>
          )}
        </div>

        {/* Middle Column: Thumbnails */}
        <div className="w-36 shrink-0 border-r border-slate-800 p-3 overflow-y-auto custom-scrollbar flex flex-col gap-3 bg-slate-950/30">
          {sessionData.samples.length === 0 ? (
            <div className="text-xs text-center text-slate-500 mt-10">Empty session</div>
          ) : (
            sessionData.samples.map(sample => (
              <div 
                key={sample.id}
                id={`sample-thumb-${sample.filename}`}
                onClick={() => setActiveImage(sample.filename)}
                className={`w-full shrink-0 rounded border overflow-hidden flex flex-col bg-slate-900 cursor-pointer transition-all relative group ${
                  activeImage === sample.filename 
                    ? 'border-blue-500 ring-2 ring-blue-500/30 shadow-md' 
                    : 'border-slate-700 hover:border-slate-500 opacity-70 hover:opacity-100'
                }`}
              >
                <div className="w-full aspect-video relative">
                  <img 
                    src={`${API_BASE}/api/calib/sessions/${activeSession}/images/${sample.filename}`} 
                    alt={sample.filename}
                    className="w-full h-full object-cover"
                  />
                  <div className="absolute top-0 right-0 bg-black/60 text-[9px] text-white px-1.5 py-0.5 rounded-bl">
                    #{sample.id}
                  </div>
                </div>
                <div className="p-1.5 flex flex-col gap-1 text-[8px] text-slate-400 font-mono tracking-tight border-t border-slate-800 leading-none">
                  <div className="flex justify-between">
                    <span>X:{sample.pose?.[0]?.toFixed(1) ?? '0.0'}</span>
                    <span>Y:{sample.pose?.[1]?.toFixed(1) ?? '0.0'}</span>
                    <span>Z:{sample.pose?.[2]?.toFixed(1) ?? '0.0'}</span>
                  </div>
                  <div className="flex justify-between text-slate-500">
                    <span>Rx:{sample.pose?.[3]?.toFixed(2) ?? '0.00'}</span>
                    <span>Ry:{sample.pose?.[4]?.toFixed(2) ?? '0.00'}</span>
                    <span>Rz:{sample.pose?.[5]?.toFixed(2) ?? '0.00'}</span>
                  </div>
                </div>
              </div>
            ))
          )}
        </div>

        {/* Right Column: Controls & Result Data */}
        <div className="w-[340px] shrink-0 bg-slate-950/50 flex flex-col overflow-hidden">
          
          {/* Scrollable Results Area */}
          <div className="flex-1 overflow-y-auto custom-scrollbar p-4 flex flex-col gap-4">
            
            {/* Header: Mode Display */}
            {sessionData.mode && (
              <div className="flex justify-between items-center bg-slate-900 border border-slate-800 rounded p-2 shadow-inner">
                <span className="text-[10px] text-slate-500 uppercase tracking-wider font-bold">Calibration Mode</span>
                <span className="text-[10px] text-emerald-400 font-mono uppercase bg-emerald-950/30 px-2 py-0.5 rounded border border-emerald-900/50">
                  {sessionData.mode}
                </span>
              </div>
            )}

            {!sessionData.result ? (
              <div className="flex-1 flex flex-col items-center justify-center text-slate-500 opacity-60">
                <p className="text-xs">No calibration data yet.</p>
                <p className="text-xs mt-1">Capture at least 3 samples.</p>
              </div>
            ) : (
              <div className="flex flex-col gap-5">
                
                {/* Errors */}
                <div className="bg-slate-900 border border-slate-800 rounded-lg p-3 flex justify-between shadow-inner">
                  <div className="flex flex-col">
                    <span className="text-[10px] text-slate-500 mb-1">Reproj Error (mm)</span>
                    <span className="text-base font-mono text-emerald-400 leading-none">{sessionData.result.metadata?.reprojection_error_mm?.toFixed(4) || 'N/A'}</span>
                  </div>
                  <div className="flex flex-col text-right">
                    <span className="text-[10px] text-slate-500 mb-1">Rot Error (deg)</span>
                    <span className="text-base font-mono text-emerald-400 leading-none">{sessionData.result.metadata?.rotation_error_deg?.toFixed(4) || 'N/A'}</span>
                  </div>
                </div>

                {/* Camera Pose (XYZ RPY) */}
                <div className="flex flex-col gap-2">
                  <h4 className="text-[10px] font-bold text-slate-400 uppercase tracking-wider">Camera Pose (Base Frame)</h4>
                  <div className="bg-slate-900 border border-slate-800 rounded p-2 text-[10px] font-mono text-slate-300 grid grid-cols-2 gap-x-2 gap-y-1.5 shadow-inner">
                    <span className="flex justify-between"><span className="text-slate-500">X:</span> {sessionData.result.camera_pose_base?.x?.toFixed(2) ?? '-'}</span>
                    <span className="flex justify-between"><span className="text-slate-500">R:</span> {sessionData.result.camera_pose_base?.roll_deg?.toFixed(2) ?? '-'}°</span>
                    <span className="flex justify-between"><span className="text-slate-500">Y:</span> {sessionData.result.camera_pose_base?.y?.toFixed(2) ?? '-'}</span>
                    <span className="flex justify-between"><span className="text-slate-500">P:</span> {sessionData.result.camera_pose_base?.pitch_deg?.toFixed(2) ?? '-'}°</span>
                    <span className="flex justify-between"><span className="text-slate-500">Z:</span> {sessionData.result.camera_pose_base?.z?.toFixed(2) ?? '-'}</span>
                    <span className="flex justify-between"><span className="text-slate-500">Y:</span> {sessionData.result.camera_pose_base?.yaw_deg?.toFixed(2) ?? '-'}°</span>
                  </div>
                </div>

                {/* Transform Matrix */}
                <div className="flex flex-col gap-2">
                  <h4 className="text-[10px] font-bold text-slate-400 uppercase tracking-wider">Transform Matrix</h4>
                  <div className="bg-slate-900 border border-slate-800 rounded p-2 text-[10px] font-mono text-slate-300 overflow-x-auto whitespace-pre shadow-inner">
                    {sessionData.result.T_base_camera?.map((row: any[], i: number) => (
                      <div key={i} className="flex gap-2">
                        {row.map((val, j) => (
                          <span key={j} className="w-[60px] text-right inline-block">{val.toFixed(4)}</span>
                        ))}
                      </div>
                    ))}
                  </div>
                </div>

                {/* Intrinsics & Params */}
                <div className="flex flex-col gap-2">
                  <h4 className="text-[10px] font-bold text-slate-400 uppercase tracking-wider">Configuration</h4>
                  <div className="bg-slate-900 border border-slate-800 rounded p-2 text-[10px] font-mono text-slate-400 flex flex-col gap-1.5 shadow-inner">
                    <div className="flex justify-between">
                      <span>Model: <span className="text-slate-200">{sessionData.result.camera_params?.camera_model || '-'}</span></span>
                      <span>Res: <span className="text-slate-200">{sessionData.result.camera_params?.width || '-'}x{sessionData.result.camera_params?.height || '-'}</span></span>
                    </div>
                    <div className="flex justify-between">
                      <span>Board: <span className="text-slate-200">{sessionData.result.board_params?.cols || '-'}x{sessionData.result.board_params?.rows || '-'}</span></span>
                      <span>Square: <span className="text-slate-200">{sessionData.result.board_params?.square_size_mm || '-'}mm</span></span>
                    </div>
                  </div>
                </div>

              </div>
            )}
          </div>

          {/* Action Footer */}
          <div className="p-3 bg-slate-950 border-t border-slate-800 flex flex-col gap-3 shrink-0 shadow-[0_-10px_20px_rgba(0,0,0,0.3)]">
            
            <div className="flex justify-between items-center text-[10px] text-slate-400 px-1 font-medium">
              <span>
                {sessionData.result?.metadata ? (
                  <>
                    <span className="text-emerald-400 font-bold">
                      {sessionData.result.metadata.samples_used} / {sessionData.result.metadata.samples_total}
                    </span> Samples Used
                  </>
                ) : (
                  <>
                    <span className="text-emerald-400 font-bold">{sessionData.samples.length}</span> Samples Captured
                  </>
                )}
              </span>
              {sessionData.result?.metadata?.timestamp && (
                <span>{sessionData.result.metadata.timestamp}</span>
              )}
            </div>

            <div className="flex gap-2">
              <button 
                onClick={handleCapture}
                disabled={isCapturing || !activeSession}
                className="flex-1 py-2 bg-blue-600 hover:bg-blue-500 border border-blue-500 text-white font-medium rounded-lg shadow transition-colors flex items-center justify-center gap-1.5 text-xs disabled:opacity-50 disabled:cursor-not-allowed"
              >
                <Camera size={14} />
                {isCapturing ? 'Wait...' : 'Capture'}
              </button>
              <button 
                onClick={handleRunCalibration}
                disabled={isRunning || !activeSession || sessionData.samples.length < 3}
                className="flex-1 py-2 bg-gradient-to-r from-emerald-600 to-teal-600 hover:from-emerald-500 hover:to-teal-500 text-white font-medium rounded-lg shadow-lg shadow-emerald-900/20 transition-all flex items-center justify-center gap-1.5 text-xs active:scale-[0.98] disabled:opacity-50 disabled:cursor-not-allowed"
              >
                <Play size={14} fill="currentColor" />
                {isRunning ? 'Solving...' : 'Execute'}
              </button>
              <button 
                onClick={handleDeleteSession}
                disabled={!activeSession}
                className="px-3 py-2 bg-transparent hover:bg-red-950 border border-red-900/50 hover:border-red-500/50 text-red-500 font-medium rounded-lg transition-colors flex items-center justify-center gap-1 text-xs disabled:opacity-30 disabled:cursor-not-allowed"
                title="Delete Session"
              >
                <Trash2 size={14} />
              </button>
            </div>
          </div>

        </div>
      </div>
    </div>
  );
};

export default CalibrationOp;
