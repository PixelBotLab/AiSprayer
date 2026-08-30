import React, { useState, useEffect, useRef } from 'react';
import { Play, FolderPlus, Trash2, Image as ImageIcon, Camera, ChevronLeft, ChevronRight, RotateCw } from 'lucide-react';
import { CustomModal, type ModalConfig } from '../common/CustomModal';
import { API_BASE } from '../../config';

type MountCatalog = {
  mounts: string[];
  default: string;
  min_samples: Record<string, number>;
  recommended_samples: Record<string, number>;
};

const MOUNT_LABELS: Record<string, string> = {
  'eye-to-hand': 'Eye-to-Hand',
  'eye-in-hand': 'Eye-in-Hand',
};

// 按钮上只显示缩写, 全名与装法说明放 tooltip / aria-label
const MOUNT_ABBREV: Record<string, string> = {
  'eye-to-hand': 'E2H',
  'eye-in-hand': 'EIH',
};

const MOUNT_HINTS: Record<string, string> = {
  'eye-to-hand': 'Camera fixed on the machine base, chessboard mounted on the robot flange.',
  'eye-in-hand': 'Camera mounted on the robot flange, chessboard fixed in the work cell.',
};

const CalibrationOp: React.FC = () => {
  const [sessions, setSessions] = useState<string[]>([]);
  const [activeSession, setActiveSession] = useState<string | null>(null);
  const [sessionData, setSessionData] = useState<{
    samples: any[]; result: any; mount?: string;
    min_samples?: number; recommended_samples?: number;
  }>({ samples: [], result: null });
  const [mountCatalog, setMountCatalog] = useState<MountCatalog | null>(null);
  const [selectedMount, setSelectedMount] = useState<string>('eye-to-hand');
  const [activeImage, setActiveImage] = useState<string | null>(null);
  const [isCapturing, setIsCapturing] = useState(false);
  const [isRunning, setIsRunning] = useState(false);
  const [isResampling, setIsResampling] = useState(false);
  const [progressData, setProgressData] = useState<{current: number, total: number, status: string, message?: string} | null>(null);
  const scrollRef = useRef<HTMLDivElement>(null);

  const minSamples = sessionData.min_samples ?? 3;
  const activeMount = sessionData.mount ?? selectedMount;

  // 两种安装的结果键名不同: 眼在手上标的是相机相对法兰, 不是相对基座
  const resultMount = sessionData.result?.metadata?.hand_eye_mount || sessionData.mount || 'eye-to-hand';
  const isInHand = resultMount === 'eye-in-hand';
  const cameraPose = isInHand ? sessionData.result?.camera_pose_flange : sessionData.result?.camera_pose_base;
  const cameraMatrix = isInHand ? sessionData.result?.T_flange_camera : sessionData.result?.T_base_camera;
  const meta = sessionData.result?.metadata;
  const reprojPx: number | null | undefined = meta?.reprojection_error_px;
  const reprojMm: number | undefined = meta?.translation_error_mm ?? meta?.reprojection_error_mm;
  const quality = meta?.data_quality;

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
        const sessList = data.sessions || [];
        setSessions(sessList);
        
        if (sessList.length > 0) {
          if (preferredSession && sessList.includes(preferredSession)) {
            setActiveSession(preferredSession);
          } else if (!activeSession || !sessList.includes(activeSession)) {
            setActiveSession(sessList[0]);
          }
        } else {
          setActiveSession(null);
          setActiveImage(null);
          setSessionData({ samples: [], result: null });
        }
      }
    } catch (err) {
      console.error('Failed to fetch calibration sessions:', err);
    }
  };

  const fetchSessionData = async (sessionId: string): Promise<any[] | null> => {
    try {
      const res = await fetch(`${API_BASE}/api/calib/sessions/${sessionId}`);
      if (res.ok) {
        const data = await res.json();
        setSessionData({
          samples: data.samples || [],
          result: data.result || null,
          mount: data.mount || undefined,
          min_samples: data.min_samples,
          recommended_samples: data.recommended_samples
        });
        if (data.samples && data.samples.length > 0) {
          // If active image doesn't exist in current samples, set to the last one
          const filenames = data.samples.map((s: any) => s.filename);
          if (!activeImage || !filenames.includes(activeImage)) {
            setActiveImage(filenames[filenames.length - 1]);
          }
        } else {
          setActiveImage(null);
        }
        return data.samples || [];
      }
    } catch (err) {
      console.error(`Failed to fetch session ${sessionId} data:`, err);
    }
    return null;
  };

  const fetchMountCatalog = async () => {
    try {
      const res = await fetch(`${API_BASE}/api/calib/mounts`);
      if (res.ok) {
        const data = await res.json();
        setMountCatalog(data);
        if (data.default) setSelectedMount(data.default);
      }
    } catch (err) {
      console.error('Failed to fetch hand-eye mounts:', err);
    }
  };

  useEffect(() => {
    fetchSessions();
    fetchMountCatalog();
  }, []);

  useEffect(() => {
    if (activeSession) {
      fetchSessionData(activeSession);
    }
  }, [activeSession]);

  const handleCreateSession = async () => {
    try {
      const res = await fetch(`${API_BASE}/api/calib/sessions/new`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mount: selectedMount })
      });
      if (res.ok) {
        const data = await res.json();
        await fetchSessions(data.session_id);
      } else {
        const err = await res.json();
        showAlert('Create Session Failed', err.detail || 'Could not create new session');
      }
    } catch (err: any) {
      showAlert('Network Error', err.message);
    }
  };

  const handleDeleteSession = async (sessionToDelete?: string) => {
    const targetSession = sessionToDelete || activeSession;
    if (!targetSession) return;

    setModalConfig({
      isOpen: true,
      type: 'confirm',
      title: 'Delete Calibration Session',
      message: `Are you sure you want to permanently delete session "${targetSession}"? All captured samples and calibration results will be removed.`,
      confirmText: 'Delete Permanently',
      cancelText: 'Cancel',
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
    if (!activeSession || isCapturing || isRunning || isResampling) return;
    setIsCapturing(true);
    try {
      const res = await fetch(`${API_BASE}/api/calib/sessions/${activeSession}/samples`, { method: 'POST' });
      if (res.ok) {
        const samples = await fetchSessionData(activeSession);
        if (samples && samples.length > 0) {
          setActiveImage(samples[samples.length - 1].filename);
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

  const handleResampleAndCalibrate = async () => {
    if (!activeSession || isRunning || isResampling || isCapturing) return;
    if (sessionData.samples.length < minSamples) {
      showAlert('Insufficient Samples', `'${activeMount}' calibration needs at least ${minSamples} valid waypoints. Current session has ${sessionData.samples.length}.`);
      return;
    }
    setIsResampling(true);
    setProgressData({ current: 0, total: sessionData.samples.length, status: 'started', message: 'Starting robot auto-resampling...' });
    try {
      const res = await fetch(`${API_BASE}/api/calib/sessions/${activeSession}/resample_and_calibrate`, { method: 'POST' });
      if (!res.ok) {
        const err = await res.json().catch(() => ({}));
        throw new Error(err.detail || 'Failed to start resample and calibration task');
      }

      const eventSource = new EventSource(`${API_BASE}/api/calib/sessions/${activeSession}/progress`);
      
      eventSource.onmessage = (event) => {
        const data = JSON.parse(event.data);
        if (data.status === 'waiting') return;
        
        setProgressData({
          current: data.current || 0,
          total: data.total || sessionData.samples.length,
          status: data.status,
          message: data.message
        });
        
        if (data.filename) {
          setActiveImage(data.filename);
        }

        if (data.status === 'completed' || data.status === 'error') {
          eventSource.close();
          setIsResampling(false);
          setProgressData(null);
          
          if (data.status === 'completed') {
            fetchSessionData(activeSession);
          } else {
            showAlert('Resample/Calibration Failed', data.message || 'Auto-resampling or optimization failed. Please check robot status and camera detections.');
          }
        }
      };
      
      eventSource.onerror = () => {
        eventSource.close();
        setIsResampling(false);
        setProgressData(null);
      };
    } catch (err: any) {
      showAlert('Execution Error', err.message);
      setIsResampling(false);
      setProgressData(null);
    }
  };

  const handleRunCalibration = async () => {
    if (!activeSession || isRunning || isResampling || isCapturing) return;
    if (sessionData.samples.length < minSamples) {
      showAlert('Insufficient Samples', `'${activeMount}' calibration needs at least ${minSamples} valid samples. Current session has ${sessionData.samples.length}.`);
      return;
    }
    setIsRunning(true);
    setProgressData({ current: 0, total: sessionData.samples.length, status: 'started', message: 'Solving hand-eye calibration...' });
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
          status: data.status,
          message: data.message
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
          title={`Create New ${MOUNT_LABELS[selectedMount] || selectedMount} Session`}
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
          {(isRunning || isResampling) && progressData && (
            <div className="absolute inset-x-0 bottom-0 bg-slate-950/85 backdrop-blur-sm p-3.5 border-t border-sky-900/50 flex flex-col gap-1.5 z-20">
              <div className="flex justify-between items-center text-xs">
                <div className="flex items-center gap-2">
                  <span className={`font-bold uppercase tracking-wider text-[10px] px-2 py-0.5 rounded ${
                    isResampling ? 'bg-indigo-950 text-indigo-300 border border-indigo-800' : 'bg-emerald-950 text-emerald-300 border border-emerald-800'
                  }`}>
                    {progressData.status}
                  </span>
                  {progressData.message && (
                    <span className="text-slate-300 text-xs font-mono truncate max-w-[320px]">
                      {progressData.message}
                    </span>
                  )}
                </div>
                <span className="text-slate-400 font-mono text-xs">{progressData.current} / {progressData.total}</span>
              </div>
              <div className="w-full h-1.5 bg-slate-800 rounded-full overflow-hidden">
                <div 
                  className={`h-full transition-all duration-300 ${
                    isResampling ? 'bg-gradient-to-r from-indigo-500 to-sky-400' : 'bg-emerald-500'
                  }`}
                  style={{ width: `${(progressData.current / Math.max(1, progressData.total)) * 100}%` }}
                />
              </div>
            </div>
          )}
        </div>

        {/* Middle Column: Thumbnails */}
        <div className="w-28 shrink-0 border-r border-slate-800 p-2 overflow-y-auto custom-scrollbar flex flex-col gap-2 bg-slate-950/30">
          {sessionData.samples.length === 0 ? (
            <div className="text-[10px] text-center text-slate-500 mt-10">Empty session</div>
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
                  <div className="absolute top-0 right-0 bg-black/60 text-[8px] text-white px-1 py-0.2 rounded-bl">
                    #{sample.id}
                  </div>
                </div>
                <div className="p-1 flex flex-col gap-0.5 text-[7.5px] text-slate-400 font-mono tracking-tight border-t border-slate-800 leading-none">
                  <div className="flex justify-between">
                    <span>X:{sample.pose?.[0]?.toFixed(0) ?? '0'}</span>
                    <span>Y:{sample.pose?.[1]?.toFixed(0) ?? '0'}</span>
                    <span>Z:{sample.pose?.[2]?.toFixed(0) ?? '0'}</span>
                  </div>
                </div>
              </div>
            ))
          )}
        </div>

        {/* Right Column: Controls & Result Data (Narrowed to maximize left image view) */}
        <div className="w-[230px] shrink-0 bg-slate-950/50 flex flex-col overflow-hidden">
          
          {/* Scrollable Results Area */}
          <div className="flex-1 overflow-y-auto custom-scrollbar p-2.5 flex flex-col gap-2.5">
            
            {/* Hand-Eye Mount: selectable for a new session, locked once bound */}
            <div className="flex justify-between items-center gap-1.5 bg-slate-900 border border-slate-800 rounded px-2 py-1 shadow-inner">
              <span className="text-[9px] text-slate-500 uppercase tracking-wider font-bold shrink-0"
                    title={activeSession
                      ? "Bound at session creation. Pick a different mount and press New to start a new session."
                      : "Camera mounting for the next session created by New."}>
                Mount
              </span>
              <div className="flex items-center gap-0.5 p-0.5 bg-slate-800/60 rounded-md border border-slate-700">
                {(mountCatalog?.mounts || ['eye-to-hand', 'eye-in-hand']).map((m) => {
                  const bound = activeSession ? activeMount : selectedMount;
                  const isBoundCurrent = !!activeSession;
                  const active = bound === m;
                  return (
                    <button
                      key={m}
                      disabled={isBoundCurrent}
                      onClick={() => setSelectedMount(m)}
                      title={`${MOUNT_LABELS[m] || m}: ${MOUNT_HINTS[m] || m}`}
                      aria-label={MOUNT_LABELS[m] || m}
                      className={`px-1.5 py-1 rounded text-[9px] font-mono font-bold uppercase tracking-wide transition-colors border ${
                        active
                          ? m === 'eye-in-hand'
                            ? 'text-indigo-300 bg-indigo-950/40 border-indigo-700/60'
                            : 'text-emerald-400 bg-emerald-950/40 border-emerald-700/60'
                          : 'text-slate-500 border-transparent hover:text-slate-200 hover:bg-slate-700/60'
                      } ${isBoundCurrent ? 'cursor-not-allowed' : ''}`}
                    >
                      {MOUNT_ABBREV[m] || m}
                    </button>
                  );
                })}
              </div>
            </div>

            {quality?.degenerate && (
              <div className="bg-rose-950/40 border border-rose-900/60 rounded px-2 py-1 text-[8.5px] text-rose-300 leading-tight"
                   title="All samples rotate about nearly the same axis, so the hand-eye transform is not uniquely observable. Capture waypoints with the flange rotated about clearly different axes.">
                Rotation degenerate (axis coverage {quality.axis_coverage?.toFixed(2)}): result may be unreliable.
              </div>
            )}

            {!sessionData.result ? (
              <div className="flex-1 flex flex-col items-center justify-center text-slate-500 opacity-60 text-center py-6">
                <p className="text-[11px]">No calibration data yet.</p>
                <p className="text-[9px] mt-1 text-slate-600">
                  {`${MOUNT_LABELS[activeMount] || activeMount}: need ${minSamples} samples (recommended ${sessionData.recommended_samples ?? '-'})`}
                </p>
                <p className="text-[9px] mt-1 text-slate-600">{MOUNT_HINTS[activeMount]}</p>
              </div>
            ) : (
              <div className="flex flex-col gap-2.5">
                
                {/* Errors */}
                <div className="bg-slate-900 border border-slate-800 rounded p-2 grid grid-cols-3 gap-1 shadow-inner text-[9px]">
                  <div className="flex flex-col">
                    <span className="text-slate-500 text-[8.5px]" title="Mean corner reprojection error in pixels">Reproj</span>
                    <span className="text-xs font-mono text-emerald-400 font-bold leading-tight">
                      {reprojPx != null ? `${reprojPx.toFixed(2)} px` : 'N/A'}
                    </span>
                  </div>
                  <div className="flex flex-col" title="Mean board-position residual of the fitted model in mm">
                    <span className="text-slate-500 text-[8.5px]">Residual</span>
                    <span className="text-xs font-mono text-emerald-400 font-bold leading-tight">
                      {reprojMm != null ? `${reprojMm.toFixed(2)} mm` : 'N/A'}
                    </span>
                  </div>
                  <div className="flex flex-col text-right">
                    <span className="text-slate-500 text-[8.5px]">Rot Err</span>
                    <span className="text-xs font-mono text-emerald-400 font-bold leading-tight">
                      {meta?.rotation_error_deg != null ? `${meta.rotation_error_deg.toFixed(2)}°` : 'N/A'}
                    </span>
                  </div>
                </div>

                {/* Camera Pose (XYZ RPY) */}
                <div className="flex flex-col gap-1">
                  <h4 className="text-[9px] font-bold text-slate-400 uppercase tracking-wider">
                    {`Camera Pose (${isInHand ? 'Flange' : 'Base'} Frame)`}
                  </h4>
                  <div className="bg-slate-900 border border-slate-800 rounded p-1.5 text-[8.5px] font-mono text-slate-300 grid grid-cols-2 gap-x-1.5 gap-y-1 shadow-inner">
                    <span className="flex justify-between"><span className="text-slate-500">X:</span> {cameraPose?.x?.toFixed(1) ?? '-'}</span>
                    <span className="flex justify-between"><span className="text-slate-500">R:</span> {cameraPose?.roll_deg?.toFixed(1) ?? '-'}°</span>
                    <span className="flex justify-between"><span className="text-slate-500">Y:</span> {cameraPose?.y?.toFixed(1) ?? '-'}</span>
                    <span className="flex justify-between"><span className="text-slate-500">P:</span> {cameraPose?.pitch_deg?.toFixed(1) ?? '-'}°</span>
                    <span className="flex justify-between"><span className="text-slate-500">Z:</span> {cameraPose?.z?.toFixed(1) ?? '-'}</span>
                    <span className="flex justify-between"><span className="text-slate-500">Y:</span> {cameraPose?.yaw_deg?.toFixed(1) ?? '-'}°</span>
                  </div>
                </div>

                {/* Transform Matrix */}
                <div className="flex flex-col gap-1">
                  <h4 className="text-[9px] font-bold text-slate-400 uppercase tracking-wider">
                    {`Transform Matrix (${isInHand ? 'T_flange_camera' : 'T_base_camera'})`}
                  </h4>
                  <div className="bg-slate-900 border border-slate-800 rounded p-1.5 text-[8px] font-mono text-slate-300 overflow-x-auto whitespace-pre shadow-inner">
                    {cameraMatrix?.map((row: any[], i: number) => (
                      <div key={i} className="flex justify-between gap-1 leading-tight">
                        {row.map((val, j) => (
                          <span key={j} className="text-right inline-block">{val.toFixed(3)}</span>
                        ))}
                      </div>
                    ))}
                  </div>
                </div>

                {/* Intrinsics & Board Params */}
                <div className="flex flex-col gap-1">
                  <h4 className="text-[9px] font-bold text-slate-400 uppercase tracking-wider">Configuration</h4>
                  <div className="bg-slate-900 border border-slate-800 rounded p-1.5 text-[8.5px] font-mono text-slate-400 flex flex-col gap-1 shadow-inner">
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

            <div className="flex gap-1.5 items-center">
              {/* 1. Capture Button */}
              <div className="relative group flex-1 flex items-center justify-center">
                <button 
                  onClick={handleCapture}
                  disabled={isCapturing || isRunning || isResampling || !activeSession}
                  className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
                >
                  <Camera size={14} className={isCapturing ? "animate-pulse text-sky-400" : "text-slate-300"} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/80 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                    {isCapturing ? 'Capturing Sample...' : 'Capture Single Sample at Current Pose'}
                  </div>
                </div>
              </div>
              
              {/* 2. Resample & Calibrate Button */}
              <div className="relative group flex-1 flex items-center justify-center">
                <button 
                  onClick={handleResampleAndCalibrate}
                  disabled={isRunning || isResampling || isCapturing || !activeSession || sessionData.samples.length < minSamples}
                  className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
                >
                  <RotateCw size={14} className={isResampling ? "animate-spin text-sky-400" : "text-slate-300"} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/80 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                    {isResampling ? 'Resampling Waypoints...' : 'Resample All Waypoints & Calibrate'}
                  </div>
                </div>
              </div>

              {/* 3. Calibrate Button */}
              <div className="relative group flex-1 flex items-center justify-center">
                <button 
                  onClick={handleRunCalibration}
                  disabled={isRunning || isResampling || isCapturing || !activeSession || sessionData.samples.length < minSamples}
                  className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
                >
                  <Play size={14} fill="currentColor" className={isRunning ? "animate-pulse text-sky-400" : "text-slate-300"} />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/80 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                    {isRunning ? 'Solving Calibration...' : 'Calculate Calibration from Samples'}
                  </div>
                </div>
              </div>

              {/* 4. Delete Session Button */}
              <div className="relative group flex-1 flex items-center justify-center">
                <button 
                  onClick={() => handleDeleteSession()}
                  disabled={!activeSession || isRunning || isResampling}
                  className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-red-950/60 hover:to-slate-800 text-slate-300 hover:text-red-400 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-red-900/40"
                >
                  <Trash2 size={14} className="text-slate-300 group-hover:text-red-400 transition-colors" />
                </button>
                <div className="absolute bottom-full mb-2 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
                  <div className="bg-slate-950/80 backdrop-blur-md border border-white/10 rounded-md px-1.5 py-0.5 shadow-xl text-[9px] text-slate-300 whitespace-nowrap">
                    Delete Current Session
                  </div>
                </div>
              </div>
            </div>
          </div>

        </div>
      </div>
    </div>
  );
};

export default CalibrationOp;
