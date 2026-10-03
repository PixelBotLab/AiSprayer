import React, { useState, useEffect, useRef } from 'react';
import { Play, FolderPlus, Trash2, Image as ImageIcon, Camera, ChevronLeft, ChevronRight, RotateCw, Scan, Upload } from 'lucide-react';
import { CustomModal, type ModalConfig } from '../common/CustomModal';
import { TOOLTIP_BASE_CLASS, Tooltip } from '../common/Tooltip';
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

// 可用性三档结论, 与后端 assess_confidence() 的 grade 一一对应 (判定只在后端那一处)
const GRADE_META: Record<string, { label: string; badge: string; box: string }> = {
  OK: {
    label: 'USABLE',
    badge: 'text-emerald-300 bg-emerald-950/60 border-emerald-800/60',
    box: 'border-emerald-900/50 bg-emerald-950/15',
  },
  CAUTION: {
    label: 'CAUTION',
    badge: 'text-amber-300 bg-amber-950/60 border-amber-800/60',
    box: 'border-amber-900/50 bg-amber-950/15',
  },
  NOT_USABLE: {
    label: 'NOT USABLE',
    badge: 'text-rose-300 bg-rose-950/60 border-rose-800/70',
    box: 'border-rose-900/60 bg-rose-950/20',
  },
};

// 数值着色门槛 (px / mm / deg): 与后端 core/handeye/assess_confidence 的 _WARN/_USABLE 常量一一对应
const REPROJ_WARN_PX = 1.5;
const REPROJ_FAIL_PX = 4.0;
const RESIDUAL_WARN_MM = 2.0;
const RESIDUAL_FAIL_MM = 5.0;
const ROT_WARN_DEG = 0.5;
const ROT_FAIL_DEG = 1.5;
// 留一法外参不确定度门槛 (旋转不变范数), 对应后端 _WARN_STD_* / _USABLE_STD_*
const STD_WARN_MM = 1.5;
const STD_FAIL_MM = 5.0;
const STD_WARN_DEG = 0.2;
const STD_FAIL_DEG = 1.0;
// 臂读数与相机观测的相对转角失配 (deg), 对应后端 _WARN/_BLOCK_CONSISTENCY_*
const CONSISTENCY_WARN_DEG = 0.3;
const CONSISTENCY_BLOCK_DEG = 1.0;

// 按 (warn, fail) 双门槛给数值上色: 绿灯不能只因为“没算”就亮绿
const levelColor = (v: number | null | undefined, warn: number, fail: number) =>
  v == null ? 'text-slate-400'
    : v > fail ? 'text-rose-400'
      : v > warn ? 'text-amber-400'
        : 'text-emerald-400';

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
  const [hoveredSessionDelete, setHoveredSessionDelete] = useState<{ x: number; y: number } | null>(null);
  const scrollRef = useRef<HTMLDivElement>(null);

  // 全局生效的那一份标定 (configs/calib/calibration_result.yaml), 发布按钮的作用对象
  const [activeResult, setActiveResult] = useState<any | null>(null);
  const [isPublishing, setIsPublishing] = useState(false);
  // 重新绑定当前 session 装法的请求在飞
  const [isMountBusy, setIsMountBusy] = useState(false);

  // 标定板实时识别: 相机微服务的全局硬件模式 (角点检测 + 把叠加烧进推流)，只由下面的按钮手动进出
  const [isBoardOverlay, setIsBoardOverlay] = useState(false);
  const [isOverlayBusy, setIsOverlayBusy] = useState(false);
  const [boardDetection, setBoardDetection] = useState<{ found: boolean; count: number } | null>(null);
  // 归本视图持有的 ON 状态才在离开时复原，避免把别人开的模式关掉
  const overlayOwnedRef = useRef(false);

  // 采样互锁: 机械臂在动时采到的图与位姿不同步, 直接污染 AX=XB 约束
  const [robotState, setRobotState] = useState<{ connected: boolean; moving: boolean } | null>(null);

  const minSamples = sessionData.min_samples ?? 3;
  const activeMount = sessionData.mount ?? selectedMount;

  // 两种安装的结果键名不同: 眼在手上标的是相机相对法兰, 不是相对基座
  const resultMount = sessionData.result?.metadata?.hand_eye_mount || sessionData.mount || 'eye-to-hand';
  const isInHand = resultMount === 'eye-in-hand';
  const cameraPose = isInHand ? sessionData.result?.camera_pose_flange : sessionData.result?.camera_pose_base;
  const cameraMatrix = isInHand ? sessionData.result?.T_flange_camera : sessionData.result?.T_base_camera;
  const meta = sessionData.result?.metadata;
  // 法兰位姿口径: 没写这个字段的历史结果一律读作 controller_v1 (与后端同一规则)
  const resultFrame = meta?.pose_frame_convention || 'controller_v1';
  const reprojPx: number | null | undefined = meta?.reprojection_error_px;
  const reprojMm: number | undefined = meta?.translation_error_mm ?? meta?.reprojection_error_mm;
  const quality = meta?.data_quality;
  const confidence = meta?.confidence;
  // 旧结果文件没有 confidence 块: 退回只看 degenerate, 保证老 session 的提示不丢
  const blockingIssues: string[] = confidence?.blocking
    ?? (quality?.degenerate
      ? ['Rotation axes are nearly parallel: the hand-eye transform is not observable.']
      : []);
  const cautionIssues: string[] = confidence?.warnings ?? [];
  const gradeMeta = confidence ? GRADE_META[confidence.grade] : undefined;
  const isResultUsable = confidence ? !!confidence.usable : true;
  // 位姿参考系与免标定自检: 求解用的是法兰 (FK of joints) 还是 TCP 读数
  const poseRef = meta?.pose_reference;
  const robotCfg = meta?.robot_configuration;
  const prunedIds: number[] = meta?.pruned_sample_ids ?? [];
  const readout = meta?.readout_consistency;
  const perSampleDeg: Record<string | number, number> = readout?.per_sample_deg ?? {};
  // 旧结果文件里不确定度是分数组, 新口径是旋转不变范数 (标量)
  const stdT: number | null = confidence?.extrinsic_std_translation_mm == null
    ? null
    : Array.isArray(confidence.extrinsic_std_translation_mm)
      ? Math.max(...confidence.extrinsic_std_translation_mm)
      : Number(confidence.extrinsic_std_translation_mm);
  const stdAxes: number[] | null = confidence?.extrinsic_std_axes_mm ?? null;
  const stdR: number | null = confidence?.extrinsic_std_rotation_deg ?? null;
  // 缩略图上的坐标应当是求解真正用到的那个口径: 有法兰位姿 (FK of joints) 就显示它
  const samplePoseList = (sample: any): number[] | undefined => {
    const f = sample.flange_pose;
    if (f) return [f.x, f.y, f.z];
    return sample.pose;
  };
  // 采样互锁 (界面层这一道): 控制器反馈在动或未连接就置灰, 文案说清原因
  const isRobotBlocked = !!robotState && (robotState.moving || !robotState.connected);
  const captureHint = isCapturing ? 'Capturing Sample...'
    : robotState?.moving ? 'Robot is moving. Wait until it stops, then capture.'
      : robotState && !robotState.connected ? 'Robot is not connected.'
        : 'Capture Single Sample at Current Pose';
  // 当前 session 的就是全局在用那一份? 时间戳一并比, 避免发布后又重解一轮时误判为"已生效"
  const isResultActive = !!(activeResult?.exists
    && activeResult?.source_session === activeSession
    && activeResult?.timestamp === meta?.timestamp);

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

  const fetchActiveResult = async () => {
    try {
      const res = await fetch(`${API_BASE}/api/calib/active`);
      if (res.ok) {
        const data = await res.json();
        setActiveResult(data.active || null);
      }
    } catch (err) {
      console.error('Failed to fetch active calibration:', err);
    }
  };

  useEffect(() => {
    fetchSessions();
    fetchMountCatalog();
    fetchActiveResult();
  }, []);

  // 进页只做只读同步: 相机模式是硬件全局状态, 视图不擅自切换
  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        const res = await fetch(`${API_BASE}/api/camera/status`);
        if (!res.ok || cancelled) return;
        const data = await res.json();
        const on = !!data.calibration_mode;
        setIsBoardOverlay(on);
        if (on) overlayOwnedRef.current = true;
      } catch {
        // Camera service offline: keep the switch at OFF
      }
    })();
    return () => { cancelled = true; };
  }, []);

  // 以控制器反馈为唯一准源轮询运行状态: 机械臂在动时置灰采样/重采按钮
  // (后端 POST /sessions/{id}/samples 也会同样拒绝, 这里是界面层的那一道)。
  useEffect(() => {
    let cancelled = false;
    const poll = async () => {
      try {
        const res = await fetch(`${API_BASE}/api/robot/state`);
        if (!res.ok || cancelled) return;
        const data = await res.json();
        setRobotState({ connected: !!data.connected, moving: !!data.is_moving });
      } catch {
        if (!cancelled) setRobotState(null);
      }
    };
    poll();
    const timer = window.setInterval(poll, 500);
    return () => { cancelled = true; window.clearInterval(timer); };
  }, []);

  // ON 时轮询最新检测结果: 叠加只在 800ms 内有检测才画在流上, 需要一个能读到的"没找到"提示
  // (关闭时的清空在开关处理里做, 不在 effect 里同步 setState)
  useEffect(() => {
    if (!isBoardOverlay) return;
    let cancelled = false;
    const poll = async () => {
      try {
        const res = await fetch(`${API_BASE}/api/camera/corners`);
        if (!res.ok || cancelled) return;
        const data = await res.json();
        setBoardDetection({ found: !!data.found, count: data.count || 0 });
      } catch {
        if (!cancelled) setBoardDetection(null);
      }
    };
    poll();
    const timer = window.setInterval(poll, 800);
    return () => { cancelled = true; window.clearInterval(timer); };
  }, [isBoardOverlay]);

  // 离开标定页 = 显式退回常规模式 (深度流与 D2C 对齐归位, 跟随不被长期顶掉)
  useEffect(() => () => {
    if (!overlayOwnedRef.current) return;
    overlayOwnedRef.current = false;
    fetch(`${API_BASE}/api/camera/calibration_mode`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: false }),
      keepalive: true
    }).catch(() => {
      // Best-effort restore; the service keeps its own state and the next visit re-syncs
    });
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
    if (robotState?.moving || robotState?.connected === false) return;
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
    if (robotState?.moving || robotState?.connected === false) return;
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

  const handleToggleBoardOverlay = async () => {
    // 切模式会重启取流管线，不能和抓拍/自动采样的帧落盘撞车
    if (isOverlayBusy || isCapturing || isResampling) return;
    const next = !isBoardOverlay;
    setIsOverlayBusy(true);
    try {
      // 板参数不给: 后端统一回落到 calib.board, 保证叠加用的 pattern 与求解器一致
      const res = await fetch(`${API_BASE}/api/camera/calibration_mode`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: next })
      });
      if (!res.ok) {
        const err = await res.json().catch(() => ({}));
        throw new Error(err.detail || 'Failed to switch camera calibration mode');
      }
      setIsBoardOverlay(next);
      overlayOwnedRef.current = next;
      if (!next) setBoardDetection(null);
    } catch (err: any) {
      showAlert('Board Overlay Error', err.message);
    } finally {
      setIsOverlayBusy(false);
    }
  };

  const activeImageUrl = activeSession && activeImage
    ? `${API_BASE}/api/calib/sessions/${activeSession}/images_with_corners/${activeImage}`
    : null;

  // 把当前 session 的求解结果发布为全局生效标定 (取代手工 cp 到 configs/calib/)
  const handlePublishResult = () => {
    if (!activeSession || !sessionData.result || isPublishing) return;

    const replaces = activeResult?.exists
      ? `The currently active result is ${MOUNT_LABELS[activeResult.mount] || activeResult.mount}`
        + `${activeResult.timestamp ? ` (${activeResult.timestamp})` : ''}; it is rolled over to calibration_result.prev.yaml.`
      : 'No active calibration result exists yet.';

    // 两种装法的消费方不同, 发布 EIH 会让只认恒定 T_base_camera 的链路失效, 必须明说
    const mountImpact = resultMount === 'eye-in-hand'
      ? ' Note: eye-in-hand results have no constant camera-to-base transform, so workpiece follow falls back to its manual extrinsics and 2D-mapped trajectory planning is rejected.'
      : '';

    // 结论不可用时仍允许发布 (现场可能要一份将就用), 但确认框必须把话说清
    const verdictImpact = confidence && !confidence.usable
      ? ` WARNING: this result is judged NOT USABLE (${[...blockingIssues, ...cautionIssues].join(' ')})`
        + ' Publishing it is an explicit override.'
      : confidence
        ? ` Verdict: ${gradeMeta?.label ?? confidence.grade}.`
        : '';

    // 口径与当前运行时不一致时, 发布出去也不会被消费 (eye-in-hand 在加载时就被拒), 必须提前说清
    const frameImpact = activeResult?.runtime_pose_frame
      && resultFrame !== activeResult.runtime_pose_frame
      ? ` WARNING: this session's flange poses are ${resultFrame} while the robot is configured for `
        + `${activeResult.runtime_pose_frame}; extrinsics solved under different pose frames are not `
        + 'interchangeable. Publish only as a temporary stopgap and re-run the calibration.'
      : '';

    setModalConfig({
      isOpen: true,
      type: 'confirm',
      title: 'Publish Calibration Result',
      message: `Publish "${activeSession}" (${MOUNT_LABELS[resultMount] || resultMount}) as the globally effective hand-eye calibration? ${replaces} Session folders are left untouched.${mountImpact}${verdictImpact}${frameImpact}`,
      confirmText: 'Publish',
      cancelText: 'Cancel',
      onConfirm: async () => {
        setIsPublishing(true);
        try {
          const res = await fetch(`${API_BASE}/api/calib/sessions/${activeSession}/publish`, { method: 'POST' });
          const data = await res.json().catch(() => ({}));
          if (!res.ok) throw new Error(data.detail || 'Failed to publish calibration result');
          await fetchActiveResult();
          showAlert(
            'Calibration Published',
            `${activeSession} (${MOUNT_LABELS[resultMount] || resultMount}) is now the active hand-eye calibration. Runtime config was reloaded, no backend restart needed.`
          );
        } catch (err: any) {
          showAlert('Publish Failed', err.message);
        } finally {
          setIsPublishing(false);
        }
      }
    });
  };

  // 换装法: 无 session 时只给"下一个 New"预选; 有 session 时是重新绑定当前 session
  const handleSelectMount = (mount: string) => {
    if (isMountBusy || isRunning || isResampling || isCapturing) return;
    if (!activeSession) {
      setSelectedMount(mount);
      return;
    }
    if (mount === activeMount) return;

    const staleNote = sessionData.result
      ? ` The stored result was solved as ${MOUNT_LABELS[resultMount] || resultMount} and is now stale, re-run the solver.`
      : '';
    setModalConfig({
      isOpen: true,
      type: 'confirm',
      title: 'Re-bind Session Mount',
      message: `Re-bind "${activeSession}" to ${MOUNT_LABELS[mount] || mount}? Captured samples are kept (a sample is just flange pose + image, independent of mounting).${staleNote}`,
      confirmText: 'Re-bind',
      cancelText: 'Cancel',
      onConfirm: async () => {
        setIsMountBusy(true);
        try {
          const res = await fetch(`${API_BASE}/api/calib/sessions/${activeSession}/mount`, {
            method: 'PUT',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ mount })
          });
          const data = await res.json().catch(() => ({}));
          if (!res.ok) throw new Error(data.detail || 'Failed to re-bind session mount');
          setSelectedMount(mount);
          await fetchSessionData(activeSession);
        } catch (err: any) {
          showAlert('Mount Re-bind Failed', err.message);
        } finally {
          setIsMountBusy(false);
        }
      }
    });
  };

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
        <div className="relative group flex items-center shrink-0">
          <button
            onClick={handleCreateSession}
            className="p-1 rounded bg-slate-800 hover:bg-slate-700 text-sky-400 border border-slate-700 shrink-0 transition-colors flex items-center gap-1 text-xs font-medium px-2"
          >
            <FolderPlus size={13} />
            <span>New</span>
          </button>
          <Tooltip text={`Create New ${MOUNT_LABELS[selectedMount] || selectedMount} Session`} side="bottom" />
        </div>

        <div className="h-4 w-[1px] bg-slate-800 shrink-0" />

        {/* Left Scroll Arrow */}
        <div className="relative group flex items-center shrink-0">
          <button
            onClick={() => scrollTabs('left')}
            className="p-1 hover:bg-slate-800 text-slate-400 hover:text-slate-200 rounded transition-colors"
          >
            <ChevronLeft size={15} />
          </button>
          <Tooltip text="Scroll Left" side="bottom" />
        </div>
        
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
                <div className="relative flex items-center">
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      setHoveredSessionDelete(null);
                      handleDeleteSession(session);
                    }}
                    onMouseEnter={(e) => {
                      const r = e.currentTarget.getBoundingClientRect();
                      setHoveredSessionDelete({ x: r.left + r.width / 2, y: r.bottom + 6 });
                    }}
                    onMouseLeave={() => setHoveredSessionDelete(null)}
                    className="opacity-0 group-hover:opacity-100 hover:text-rose-400 p-0.5 rounded transition-opacity"
                  >
                    <Trash2 size={11} />
                  </button>
                </div>
              </div>
            );
          })}
        </div>

        {/* Right Scroll Arrow */}
        <div className="relative group flex items-center shrink-0">
          <button
            onClick={() => scrollTabs('right')}
            className="p-1 hover:bg-slate-800 text-slate-400 hover:text-slate-200 rounded transition-colors"
          >
            <ChevronRight size={15} />
          </button>
          <Tooltip text="Scroll Right" side="bottom" />
        </div>

        {/* Fixed Unclipped Tooltip for Session Delete */}
        {hoveredSessionDelete && (
          <div
            className={`fixed pointer-events-none z-[100] -translate-x-1/2 ${TOOLTIP_BASE_CLASS}`}
            style={{ left: hoveredSessionDelete.x, top: hoveredSessionDelete.y }}
          >
            Delete Session
          </div>
        )}
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
                  {/* 逐样本失配/被裁标记: 让操作员一眼看出该重采哪几帧 */}
                  {(prunedIds.includes(sample.id) || perSampleDeg[sample.id] >= CONSISTENCY_WARN_DEG) && (
                    <div className="absolute bottom-0 right-0 flex flex-col items-end gap-0.5">
                      {prunedIds.includes(sample.id) && (
                        <span className="bg-rose-900/80 text-rose-100 text-[7px] px-1 py-0.5 rounded-tl">
                          DROPPED
                        </span>
                      )}
                      {perSampleDeg[sample.id] >= CONSISTENCY_WARN_DEG && (
                        <span className={`text-[7px] px-1 py-0.5 rounded-tl font-mono ${
                          perSampleDeg[sample.id] >= CONSISTENCY_BLOCK_DEG
                            ? 'bg-rose-900/80 text-rose-100'
                            : 'bg-amber-900/80 text-amber-100'
                        }`}>
                          {`Δ ${perSampleDeg[sample.id].toFixed(2)}°`}
                        </span>
                      )}
                    </div>
                  )}
                </div>
                <div className="p-1 flex flex-col gap-0.5 text-[7.5px] text-slate-400 font-mono tracking-tight border-t border-slate-800 leading-none">
                  <div className="flex justify-between">
                    <span>X:{samplePoseList(sample)?.[0]?.toFixed(0) ?? '0'}</span>
                    <span>Y:{samplePoseList(sample)?.[1]?.toFixed(0) ?? '0'}</span>
                    <span>Z:{samplePoseList(sample)?.[2]?.toFixed(0) ?? '0'}</span>
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
            
            {/* Board Live Detection: manual camera-wide mode switch (never auto-toggled) */}
            <div className="flex flex-col gap-1 bg-slate-900 border border-slate-800 rounded px-2 py-1.5 shadow-inner">
              <div className="flex justify-between items-center gap-1.5">
                <div className="relative group flex items-center shrink-0">
                  <span className="text-[9px] text-slate-500 uppercase tracking-wider font-bold cursor-help flex items-center gap-1">
                    <Scan size={10} className={isBoardOverlay ? 'text-emerald-400' : ''} />
                    <span>Board Overlay</span>
                  </span>
                  <Tooltip
                    multiline
                    side="bottom"
                    align="start"
                    maxWidthClass="max-w-[230px]"
                    text="Detects the chessboard in the camera service and burns the corners into the live stream. Watch it in the Live Camera window (left sidebar). While ON, the depth stream, D2C alignment and workpiece follow are disabled. Leaving this page restores normal mode."
                  />
                </div>
                <button
                  onClick={handleToggleBoardOverlay}
                  disabled={isOverlayBusy || isCapturing || isResampling}
                  aria-label={isBoardOverlay ? 'Disable Live Board Detection' : 'Enable Live Board Detection'}
                  className={`px-2 py-1 rounded text-[9px] font-mono font-bold uppercase tracking-wide transition-colors border ${
                    isBoardOverlay
                      ? 'text-emerald-400 bg-emerald-950/40 border-emerald-700/60 hover:bg-emerald-950/70'
                      : 'text-slate-500 bg-slate-800/60 border-slate-700 hover:text-slate-200 hover:bg-slate-700/60'
                  } ${isOverlayBusy ? 'opacity-40 cursor-wait' : ''}`}
                >
                  {isOverlayBusy ? '...' : isBoardOverlay ? 'ON' : 'OFF'}
                </button>
              </div>
              {isBoardOverlay && (
                <div className="flex justify-between items-center text-[8.5px] font-mono tracking-tight">
                  <span className={boardDetection?.found ? 'text-emerald-400' : 'text-rose-400'}>
                    {boardDetection === null
                      ? 'DETECTING...'
                      : boardDetection.found
                        ? `${boardDetection.count} CORNERS`
                        : 'NO BOARD FOUND'}
                  </span>
                  <span className="text-slate-500">DEPTH OFF</span>
                </div>
              )}
            </div>

            {/* Hand-Eye Mount: bound mount of the open session, click another to re-bind */}
            <div className="flex justify-between items-center gap-1.5 bg-slate-900 border border-slate-800 rounded px-2 py-1 shadow-inner">
              <div className="relative group flex items-center shrink-0">
                <span className="text-[9px] text-slate-500 uppercase tracking-wider font-bold cursor-help">
                  Mount
                </span>
                <Tooltip
                  multiline
                  text={
                    activeSession
                      ? "Mounting bound to the open session. Click the other one to re-bind this session (samples kept, solver must be re-run)."
                      : "Camera mounting for the next session created by New."
                  }
                  side="bottom"
                />
              </div>
              <div className="flex items-center gap-0.5 p-0.5 bg-slate-800/60 rounded-md border border-slate-700">
                {(mountCatalog?.mounts || ['eye-to-hand', 'eye-in-hand']).map((m) => {
                  const bound = activeSession ? activeMount : selectedMount;
                  const active = bound === m;
                  return (
                    <div key={m} className="relative group flex items-center">
                      <button
                        disabled={isMountBusy || isRunning || isResampling || isCapturing}
                        onClick={() => handleSelectMount(m)}
                        aria-label={MOUNT_LABELS[m] || m}
                        className={`px-1.5 py-1 rounded text-[9px] font-mono font-bold uppercase tracking-wide transition-colors border ${
                          active
                            ? m === 'eye-in-hand'
                              ? 'text-indigo-300 bg-indigo-950/40 border-indigo-700/60'
                              : 'text-emerald-400 bg-emerald-950/40 border-emerald-700/60'
                            : 'text-slate-500 border-transparent hover:text-slate-200 hover:bg-slate-700/60'
                        } ${isMountBusy ? 'opacity-40 cursor-wait' : ''}`}
                      >
                        {MOUNT_ABBREV[m] || m}
                      </button>
                      <Tooltip
                        text={`${MOUNT_LABELS[m] || m}: ${MOUNT_HINTS[m] || m}`}
                        side="bottom"
                        align={m === 'eye-in-hand' ? 'end' : 'start'}
                        multiline
                        maxWidthClass="max-w-[240px]"
                      />
                    </div>
                  );
                })}
              </div>
            </div>

            {(blockingIssues.length > 0 || cautionIssues.length > 0) && (
              <div
                className={`rounded border px-2 py-1 text-[8.5px] leading-tight space-y-0.5 ${
                  blockingIssues.length > 0
                    ? 'bg-rose-950/40 border-rose-900/60 text-rose-300'
                    : 'bg-amber-950/30 border-amber-900/50 text-amber-300'
                }`}
              >
                {[...blockingIssues, ...cautionIssues].map((msg, i) => (
                  <p key={i} className="flex gap-1">
                    <span className="font-mono font-bold">!</span>
                    <span>{msg}</span>
                  </p>
                ))}
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
                    <div className="relative group inline-flex items-center">
                      <span className="text-slate-500 text-[8.5px] cursor-help">Reproj</span>
                      <Tooltip text="Mean corner reprojection error in pixels" side="top" />
                    </div>
                    <span className={`text-xs font-mono font-bold leading-tight ${levelColor(reprojPx, REPROJ_WARN_PX, REPROJ_FAIL_PX)}`}>
                      {reprojPx != null ? `${reprojPx.toFixed(2)} px` : 'N/A'}
                    </span>
                  </div>
                  <div className="flex flex-col">
                    <div className="relative group inline-flex items-center">
                      <span className="text-slate-500 text-[8.5px] cursor-help">Residual</span>
                      <Tooltip text="Mean board-position residual of the fitted model in mm" side="top" />
                    </div>
                    <span className={`text-xs font-mono font-bold leading-tight ${levelColor(reprojMm, RESIDUAL_WARN_MM, RESIDUAL_FAIL_MM)}`}>
                      {reprojMm != null ? `${reprojMm.toFixed(2)} mm` : 'N/A'}
                    </span>
                  </div>
                  <div className="flex flex-col text-right">
                    <span className="text-slate-500 text-[8.5px]">Rot Err</span>
                    <span className={`text-xs font-mono font-bold leading-tight ${levelColor(meta?.rotation_error_deg, ROT_WARN_DEG, ROT_FAIL_DEG)}`}>
                      {meta?.rotation_error_deg != null ? `${meta.rotation_error_deg.toFixed(2)}°` : 'N/A'}
                    </span>
                  </div>
                </div>

                {/* Usability verdict: 重投影误差只说明样本自洽度, 能不能用看这份不确定度结论 */}
                {confidence && gradeMeta && (
                  <div className={`border rounded p-2 text-[9px] flex flex-col gap-1 ${gradeMeta.box}`}>
                    <div className="flex items-center justify-between">
                      <div className="relative group inline-flex items-center">
                        <span className="text-slate-500 text-[8.5px] uppercase tracking-wider cursor-help">Extrinsic Uncertainty</span>
                        <Tooltip
                          multiline
                          side="top"
                          maxWidthClass="max-w-[260px]"
                          text="Leave-one-out standard error of the solved camera extrinsic: how far this matrix moves when samples are dropped. Reprojection error alone does not prove the extrinsic is accurate."
                        />
                      </div>
                      <span className={`px-1.5 py-0.5 rounded border text-[8.5px] font-mono font-bold ${gradeMeta.badge}`}>
                        {gradeMeta.label}
                      </span>
                    </div>
                    <div className="flex justify-between font-mono text-[8.5px] text-slate-300">
                      <div className="relative group inline-flex">
                        <span className={levelColor(stdT, STD_WARN_MM, STD_FAIL_MM)}>
                          {stdT != null ? `+/-${stdT.toFixed(2)} mm` : '+/- N/A mm'}
                        </span>
                        <Tooltip
                          multiline
                          side="top"
                          maxWidthClass="max-w-[260px]"
                          text={`Rotation-invariant translation standard error${stdAxes ? ` (per flange axis: ${stdAxes.map(v => `${v.toFixed(2)} mm`).join(', ')})` : ''}. Per-axis values are display only; the verdict uses the invariant norm so it cannot depend on the tool orientation.`}
                        />
                      </div>
                      <span className={levelColor(stdR, STD_WARN_DEG, STD_FAIL_DEG)}>
                        {stdR != null ? `+/-${stdR.toFixed(3)}°` : '+/- N/A°'}
                      </span>
                      <span className="text-slate-500">
                        {`${confidence.jackknife_fits || 0} re-fits`}
                        {confidence.jackknife_refined === false ? ' [closed-form]' : ''}
                      </span>
                    </div>

                    {/* 位姿链路自检: 求解参考系 + 臂读数与相机观测的相对转角失配 */}
                    <div className="flex justify-between font-mono text-[8.5px] text-slate-400">
                      <div className="relative group inline-flex items-center gap-1">
                        <span>Pose ref:</span>
                        <span className={poseRef?.frame === 'tcp' || poseRef?.frame === 'flange+tcp'
                          ? 'text-amber-400' : 'text-slate-300'}>
                          {poseRef?.frame ?? 'n/a'}
                        </span>
                        {robotCfg?.tool_index != null && <span>{`tool ${robotCfg.tool_index}`}</span>}
                        {robotCfg?.user_index != null && <span>{`user ${robotCfg.user_index}`}</span>}
                        <Tooltip
                          multiline
                          side="top"
                          maxWidthClass="max-w-[260px]"
                          text="'flange' means the hand-eye solve used the flange pose from forward kinematics of the joint feedback, which is independent of the pendant tool number. 'tcp' means it fell back to the Cartesian readout of the active tool: the extrinsic then starts at the tool tip and breaks when the tool changes."
                        />
                      </div>
                      <span className="text-slate-500">
                        {prunedIds.length > 0 ? `pruned ${prunedIds.map(i => `#${i}`).join(' ')}` : 'no pruning'}
                      </span>
                    </div>
                    <div className="relative group inline-flex items-center gap-1 font-mono text-[8.5px]">
                      <span className="text-slate-400">Arm vs camera:</span>
                      {readout ? (
                        <span className={levelColor(readout.median_deg, CONSISTENCY_WARN_DEG, CONSISTENCY_BLOCK_DEG)}>
                          {`+/-${readout.median_deg?.toFixed(3)}° med`}
                        </span>
                      ) : (
                        <span className="text-slate-500">N/A</span>
                      )}
                      {readout && (
                        <span className="text-slate-500">{`+/-${readout.p90_deg?.toFixed(3)}° p90`}</span>
                      )}
                      <Tooltip
                        multiline
                        side="top"
                        maxWidthClass="max-w-[280px]"
                        text={`Relative rotation reported by the robot readout vs seen by the camera must be identical (conjugation preserves rotation angle), independent of any extrinsic. ${readout ? `Worst pair ${readout.worst_pair}: arm ${readout.worst_arm_deg?.toFixed(2)}° vs camera ${readout.worst_vision_deg?.toFixed(2)}° over ${readout.pairs} pairs.` : 'No comparable sample pair available.'} A large gap means the pose chain itself is untrustworthy.`}
                      />
                    </div>
                  </div>
                )}

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

            {/* 当前全局生效的标定 (发布按钮的作用对象) */}
            <div className="relative group flex items-center justify-between px-1 text-[8.5px] text-slate-500 -mt-1.5">
              <span className="cursor-help uppercase tracking-wider">
                Active Calibration
                <span className="ml-1.5 font-mono text-slate-300">
                  {activeResult?.exists ? (MOUNT_ABBREV[activeResult.mount] || activeResult.mount) : 'NONE'}
                </span>
                {activeResult?.pose_frame === 'joint_offset_v2' && (
                  <span className="ml-1.5 font-mono text-sky-300">Δq</span>
                )}
              </span>
              <span className={`font-mono truncate max-w-[150px] ${activeResult?.pose_frame_warning ? 'text-amber-400' : ''}`}>
                {activeResult?.exists ? (activeResult.source_session || activeResult.path) : '-'}
              </span>
              <Tooltip
                multiline
                side="top"
                align="end"
                maxWidthClass="max-w-[260px]"
                text={`Globally effective result: ${activeResult?.exists ? activeResult.path : 'none'}. Publishing a session overwrites that file and rolls the previous one to calibration_result.prev.yaml.`
                  + `${activeResult?.pose_frame_warning ? ` Pose frame mismatch: ${activeResult.pose_frame_warning}` : ''}`}
              />
            </div>

            <div className="flex gap-1.5 items-center">
              {/* 1. Capture Button */}
              <div className="relative group flex-1 flex items-center justify-center">
                <button 
                  onClick={handleCapture}
                  disabled={isCapturing || isRunning || isResampling || !activeSession || isRobotBlocked}
                  className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
                >
                  <Camera size={14} className={isCapturing ? "animate-pulse text-sky-400" : "text-slate-300"} />
                </button>
                <Tooltip
                  text={captureHint}
                  side="top"
                  align="start"
                />
              </div>
              
              {/* 2. Resample & Calibrate Button */}
              <div className="relative group flex-1 flex items-center justify-center">
                <button 
                  onClick={handleResampleAndCalibrate}
                  disabled={isRunning || isResampling || isCapturing || !activeSession || isRobotBlocked || sessionData.samples.length < minSamples}
                  className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-slate-700 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-slate-600"
                >
                  <RotateCw size={14} className={isResampling ? "animate-spin text-sky-400" : "text-slate-300"} />
                </button>
                <Tooltip
                  text={isRobotBlocked ? captureHint : (isResampling ? 'Resampling Waypoints...' : 'Resample All Waypoints & Calibrate')}
                  side="top"
                  align="center"
                />
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
                <Tooltip
                  text={isRunning ? 'Solving Calibration...' : 'Calculate Calibration from Samples'}
                  side="top"
                  align="center"
                />
              </div>

              {/* 5. Publish As Active Calibration Button */}
              <div className="relative group flex-1 flex items-center justify-center">
                <button
                  onClick={handlePublishResult}
                  disabled={!sessionData.result || isResultActive || isRunning || isResampling || isPublishing}
                  className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-indigo-950/70 hover:to-slate-800 text-slate-200 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-indigo-800"
                >
                  <Upload size={14} className={isPublishing ? "animate-pulse text-indigo-400" : !isResultUsable ? "text-rose-400" : "text-slate-300"} />
                </button>
                <Tooltip
                  multiline
                  side="top"
                  align="end"
                  maxWidthClass="max-w-[240px]"
                  text={isResultActive
                    ? 'This result is already the active calibration'
                    : !isResultUsable
                      ? `Publish Anyway (${gradeMeta?.label || 'result flagged'}: reprojection ${reprojPx != null ? reprojPx.toFixed(2) : 'N/A'} px, uncertainty ${stdT != null ? `+/-${stdT.toFixed(1)} mm` : 'N/A'})`
                      : 'Publish This Result As Active Calibration (overwrites configs/calib result, previous one is backed up)'}
                />
              </div>

              {/* 6. Delete Session Button */}
              <div className="relative group flex-1 flex items-center justify-center">
                <button 
                  onClick={() => handleDeleteSession()}
                  disabled={!activeSession || isRunning || isResampling}
                  className="w-full h-8 bg-gradient-to-r from-slate-800 to-slate-900 hover:from-red-950/60 hover:to-slate-800 text-slate-300 hover:text-red-400 rounded-lg shadow transition-all flex items-center justify-center active:scale-95 disabled:opacity-30 disabled:cursor-not-allowed border border-slate-700 hover:border-red-900/40"
                >
                  <Trash2 size={14} className="text-slate-300 group-hover:text-red-400 transition-colors" />
                </button>
                <Tooltip
                  text="Delete Current Session"
                  side="top"
                  align="end"
                />
              </div>
            </div>
          </div>

        </div>
      </div>
    </div>
  );
};

export default CalibrationOp;
