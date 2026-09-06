export interface FileItem {
  name: string;
  size: number;
  ctime: number;
}

export interface Point {
  x: number;
  y: number;
  label: number; // 1 for fg, 0 for bg
}

export interface MaskData {
  id?: number;
  points?: Point[];
  labels?: number[];
  polygons: number[][][]; // Array of polygons, each is Array of [x, y]
  score?: number;
}

export interface WaypointItem {
  index: number;
  pixel: [number, number]; // [u, v]
  surface_point_cam_mm: [number, number, number];
  surface_point_base_mm: [number, number, number];
  surface_normal_base: [number, number, number];
  surface_normal_cam: [number, number, number];
  standoff_distance_mm: number;
  tcp_pose_base: {
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
  };
  normal_2d_proj: [number, number]; // [dx, dy]
  path_id?: number;
  spraying?: 'on' | 'off' | string;
  is_jump?: boolean;
}

export interface ManualPathItem {
  path_id: number;
  name: string;
  points: WaypointItem[];
}

export interface UrdfTcpInfo {
  has_tool: boolean;
  tool_name: string;
  xyz_mm: number[];
  rpy_deg: number[];
  urdf_source?: string;
  urdf_url?: string;
}

export interface KinematicsParams {
  stepSizeMm: number;
  linearSpeedMmS: number;
}

export interface VerificationIssue {
  severity: 'WARNING' | 'ERROR';
  type: string;
  message: string;
  step_index?: number;
  waypoint_index?: number;
  segment_index?: number;
  joint_index?: number;
  value?: number;
  limit?: number;
}

export type PathStateType = 'raw' | 'auto' | 'poi' | 'auto_poi';
export type PathSource = 'manual' | 'auto';
export type PathStage = 'orig' | 'poi';

export function pathSourceOf(state: PathStateType): PathSource {
  return state === 'auto' || state === 'auto_poi' ? 'auto' : 'manual';
}

export function pathStageOf(state: PathStateType): PathStage {
  return state === 'poi' || state === 'auto_poi' ? 'poi' : 'orig';
}

export function composePathState(source: PathSource, stage: PathStage): PathStateType {
  if (source === 'auto') return stage === 'poi' ? 'auto_poi' : 'auto';
  return stage === 'poi' ? 'poi' : 'raw';
}

export interface PoiConfig {
  ref_rpy_deg: [number, number, number];
  tolerance_rpy_deg: [number, number, number];
  anchor_source?: 'home' | 'live' | 'manual' | 'raw';
}

export interface SimulationState {
  isPlaying: boolean;
  progress: number; // 0.0 to 1.0
  speed: number; // 0.5, 1.0, 2.0, 5.0
  currentPathIndex: number;
  currentStep: number;
  totalSteps: number;
  currentJoints: number[]; // [J1..J6] in deg
  currentTcpPose: { x: number; y: number; z: number; rx: number; ry: number; rz: number };
  currentPixel: [number, number] | null; // projected [u, v]
  activeState: PathStateType;
  isRealExec?: boolean; // true = driven by real robot WS events, no fake animation loop
}


export interface PathReportItem {
  path_id: number;
  name: string;
  status: 'PASS' | 'WARNING' | 'FAILED' | 'ERROR';
  total_interpolated: number;
  speed_mm_s?: number;
  step_size_mm?: number;
  recommended_safe_speed_mm_s?: number;
  peak_joint_speeds_deg_s?: number[];
  max_joint_velocity_deg_s?: number[];
  max_joint_velocities_deg_s?: number[];
  issues: VerificationIssue[];
  optimized_paths_available?: boolean;
  trajectory_q?: number[][]; // [step][6] in radians or degrees
  trajectory_tcp?: number[][]; // [step][6] [x, y, z, rx, ry, rz]
}

export interface VerificationReport {
  status?: 'PASS' | 'WARNING' | 'FAILED' | 'ERROR';
  summary: {
    status: 'PASS' | 'WARNING' | 'FAILED' | 'ERROR';
    total_paths: number;
    total_waypoints: number;
    total_steps: number;
    total_issues: number;
    singularity_count?: number;
    overspeed_count?: number;
    unreachable_count?: number;
    elapsed_ms?: number;
  };
  state_type?: PathStateType;
  source_file?: string;
  nominal_speed_mm_s?: number;
  slerp_step_mm?: number;
  max_joint_velocities_deg_s?: number[];
  urdf_tcp?: UrdfTcpInfo;
  poi_config?: PoiConfig;
  path_reports?: PathReportItem[];
}

export interface LiveNormalInfo {
  dx: number;
  dy: number;
  surfPointBase?: [number, number, number];
  normalBase?: [number, number, number];
  tcpPose?: { x: number; y: number; z: number; rx: number; ry: number; rz: number };
}

export interface SessionData {
  width: number;
  height: number;
  depth: Float32Array; // row-major, mm
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  T: number[]; // 4x4 row-major, translation in mm
  calib_source: string;
}

export const STATE_THEMES: Record<PathStateType, { label: string; name: string; hex: string; bg: string; border: string; text: string; lightBg: string }> = {
  raw: {
    label: 'RAW',
    name: 'Raw Teach',
    hex: '#94a3b8',
    bg: 'bg-slate-500/20',
    border: 'border-slate-400/40',
    text: 'text-slate-300',
    lightBg: 'bg-slate-800'
  },
  auto: {
    label: 'AUTO',
    name: 'Auto Zigzag',
    hex: '#a78bfa',
    bg: 'bg-violet-500/20',
    border: 'border-violet-400/40',
    text: 'text-violet-300',
    lightBg: 'bg-violet-950/60'
  },
  poi: {
    label: 'POI',
    name: 'Manual POI',
    hex: '#22c55e',
    bg: 'bg-emerald-500/20',
    border: 'border-emerald-400/40',
    text: 'text-emerald-400',
    lightBg: 'bg-emerald-950/60'
  },
  auto_poi: {
    label: 'A-POI',
    name: 'Auto POI',
    hex: '#2dd4bf',
    bg: 'bg-teal-500/20',
    border: 'border-teal-400/40',
    text: 'text-teal-300',
    lightBg: 'bg-teal-950/60'
  }
};

export const MASK_COLORS = [
  { fill: 'rgba(16, 185, 129, 0.38)', stroke: '#10b981' },
  { fill: 'rgba(59, 130, 246, 0.38)', stroke: '#3b82f6' },
  { fill: 'rgba(245, 158, 11, 0.38)', stroke: '#f59e0b' },
  { fill: 'rgba(168, 85, 247, 0.38)', stroke: '#a855f7' },
  { fill: 'rgba(236, 72, 153, 0.38)', stroke: '#ec4899' },
  { fill: 'rgba(6, 182, 212, 0.38)', stroke: '#06b6d4' },
];

export const PATH_PALETTE = [
  '#0284c7', // Sky blue
  '#10b981', // Emerald
  '#f59e0b', // Amber
  '#a855f7', // Purple
  '#ec4899', // Pink
  '#06b6d4', // Cyan
];

export interface CanvasNotice {
  id: number;
  type: 'success' | 'warning' | 'error' | 'info';
  title?: string;
  message: string;
  duration?: number;
}


