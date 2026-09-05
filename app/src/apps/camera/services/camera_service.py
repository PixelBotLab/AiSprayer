import os
import sys
import re
import time
import signal
import ctypes
import atexit
import logging
import subprocess
import threading
import collections
from typing import Optional, Generator, Callable, List, Dict, Any, Tuple
import numpy as np
import cv2
import requests
from core.config import sprayer_config

logger = logging.getLogger(__name__)
cpp_logger = logging.getLogger("camera.cpp")

# Regex to strip ANSI colors and duplicate C++/ZLM headers
ANSI_REGEX = re.compile(r'\x1b\[[0-9;]*m')
CPP_LOG_REGEX = re.compile(r'^\[(DEBUG|INFO\s*|WARN\s*|WARNING|ERROR)\]\s*\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}(?:\.\d+)?\s*(.*)$')
ZLM_LOG_REGEX = re.compile(r'^\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}(?:\.\d+)?\s+([IWDET])\s+(?:\[.*?\]\s*)+(.*)$')

# Linux PR_SET_PDEATHSIG configuration for zero-zombie process termination
PR_SET_PDEATHSIG = 1
try:
    _libc = ctypes.CDLL("libc.so.6")
    _prctl = _libc.prctl
except Exception:
    _prctl = None

CPP_SERVICE_PORT = 18080
CPP_BASE_URL = f"http://127.0.0.1:{CPP_SERVICE_PORT}"


def _set_pdeathsig():
    """Instruct the Linux kernel to send SIGTERM to this child process if its parent process dies."""
    if _prctl is not None:
        _prctl(PR_SET_PDEATHSIG, signal.SIGTERM)


class CameraService:
    """
    Python 相机服务门面单例 (Facade)。
    负责在后台拉起、监控并管理 C++ 高性能硬件加速微服务进程 (orbbec_camera_service)，
    并通过 HTTP REST 接口代理数据流、状态查询与拍照标定指令。
    保证主进程退出时子进程 100% 自动伴随退出 (Zero Zombie Process)。
    """

    def __init__(self):
        self._process: Optional[subprocess.Popen] = None
        self._process_lock = threading.Lock()
        self._is_streaming = False
        self._status_callbacks: List[Callable] = []
        self._monitor_thread: Optional[threading.Thread] = None
        self._log_thread: Optional[threading.Thread] = None
        self._log_emit_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._last_status: Dict[str, Any] = {}

        # C++ 日志消费拆成两段：读线程只抽管道（永不阻塞），发射线程才做 logging（可能慢/卡）。
        # 中间隔一个有界 deque（满了丢最旧）：就算 logging 停摆，管道也不会写满 ——
        # 管道一满，C++ 侧所有打日志的线程（含 HTTP）会被阻塞式 write 冻住，
        # 实测曾把 POST /follow 的响应卡 90+ 秒，页面报"后端服务无响应"（日志背压）。
        self._log_buf: collections.deque = collections.deque(maxlen=8192)
        self._log_cond = threading.Condition()
        self._log_dropped = 0

        # Resolve Project Paths
        cur_dir = os.path.dirname(os.path.abspath(__file__))
        # cur_dir is app/src/apps/camera/services -> 4 levels up to PROJECT_ROOT
        self._project_root = os.path.abspath(os.path.join(cur_dir, "../../../../.."))
        self._bin_path = os.path.join(
            self._project_root,
            "app/src/core/hardware/camera/orbbec_camera_service/bin/orbbec_camera_service"
        )
        self._config_path = os.path.join(self._project_root, "configs/aisprayer_config.yaml")

        # Automatically clean up on Python interpreter exit
        atexit.register(self.stop_stream)
        logger.info(f"CameraService initialized. Binary path: {self._bin_path}")

    def register_status_callback(self, cb: Callable):
        if cb not in self._status_callbacks:
            self._status_callbacks.append(cb)

    def unregister_status_callback(self, cb: Callable):
        if cb in self._status_callbacks:
            self._status_callbacks.remove(cb)

    def _notify_status(self, status: dict):
        for cb in list(self._status_callbacks):
            try:
                cb(status)
            except Exception as e:
                logger.error(f"Error invoking camera status callback: {e}")

    def start_stream(self, camera_type: str = "orbbec") -> bool:
        """
        启动 C++ 相机微服务进程并开启后台健康监控。
        """
        with self._process_lock:
            if self._is_streaming and self._process and self._process.poll() is None:
                logger.info("C++ Camera Service is already running.")
                return True

            # 1. 确保二进制文件存在
            if not os.path.exists(self._bin_path):
                logger.warning(f"C++ Camera binary not found at {self._bin_path}. Attempting build...")
                build_sh = os.path.join(self._project_root, "app/scripts/build.sh")
                try:
                    if os.path.isfile(build_sh):
                        subprocess.run(["bash", build_sh, "--only", "camera"], check=True)
                    else:
                        build_dir = os.path.join(
                            self._project_root,
                            "app/src/core/hardware/camera/orbbec_camera_service/build"
                        )
                        os.makedirs(build_dir, exist_ok=True)
                        subprocess.run(["cmake", ".."], cwd=build_dir, check=True)
                        subprocess.run(["cmake", "--build", ".", "--parallel"], cwd=build_dir, check=True)
                    logger.info("C++ Camera binary built successfully.")
                except Exception as e:
                    logger.error(f"Failed to auto-build C++ camera service: {e}")
                    return False

            # 2. 清理可能遗留的孤儿进程
            self._kill_stale_processes()

            # 3. 设置动态库搜索路径并启动子进程
            env = os.environ.copy()
            third_party_lib = os.path.join(self._project_root, "third_party/install/lib")
            current_ld = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = f"{third_party_lib}:{current_ld}"
            if sys.platform == "darwin":
                current_dyld = env.get("DYLD_LIBRARY_PATH", "")
                env["DYLD_LIBRARY_PATH"] = f"{third_party_lib}:{current_dyld}"

            stats_interval = sprayer_config.config_data.get("hardware", {}).get("camera", {}).get("server", {}).get("stats_interval_sec", 10)
            cmd = [
                self._bin_path,
                "--config", self._config_path,
                "--raw-log",
                "--stats-interval", str(stats_interval)
            ]
            logger.info(f"Spawning C++ Camera Service: {' '.join(cmd)}")

            try:
                self._process = subprocess.Popen(
                    cmd,
                    cwd=self._project_root,
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    preexec_fn=_set_pdeathsig
                )
                self._log_thread = threading.Thread(
                    target=self._stream_cpp_logs,
                    args=(self._process,),
                    daemon=True,
                    name="CppCameraLogReader"
                )
                self._log_thread.start()
                # 发射线程随每次起进程重起；旧发射线程已在 stop_stream 里被 _stop_event 唤退。
                self._log_emit_thread = threading.Thread(
                    target=self._emit_cpp_logs,
                    daemon=True,
                    name="CppCameraLogEmitter"
                )
                self._log_emit_thread.start()
            except Exception as e:
                logger.error(f"Failed to spawn C++ camera service: {e}")
                return False

            self._is_streaming = True
            self._stop_event.clear()

            # 4. 启动后台状态健康监测线程
            if self._monitor_thread is None or not self._monitor_thread.is_alive():
                self._monitor_thread = threading.Thread(target=self._health_monitor_loop, daemon=True)
                self._monitor_thread.start()

            # 等待微服务就绪 (最多等待 3 秒)
            for _ in range(30):
                if self._check_service_alive():
                    logger.info("C++ Camera Service is UP and healthy on port 18080.")
                    return True
                time.sleep(0.1)

            logger.warning("C++ Camera Service spawned, waiting for camera hardware enumeration...")
            return True

    def stop_stream(self):
        """
        优雅停止 C++ 相机微服务进程。
        """
        self._is_streaming = False
        self._stop_event.set()
        with self._log_cond:
            self._log_cond.notify_all()   # 唤醒发射线程，让它看见 _stop_event 后退出

        with self._process_lock:
            proc = self._process
            if proc is not None and proc.poll() is None:
                logger.info(f"Stopping C++ Camera Service (PID: {proc.pid})...")
                try:
                    # 先发送 SIGINT 触发 C++ 优雅退出
                    proc.send_signal(signal.SIGINT)
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    logger.warning("C++ Camera Service did not exit gracefully in 3s, sending SIGKILL...")
                    try:
                        proc.kill()
                    except Exception as e:
                        logger.error(f"Error killing process: {e}")
                except Exception as e:
                    logger.warning(f"Exception stopping C++ camera process: {e}")
                finally:
                    self._process = None

        self._last_status = {"online": False, "streaming": False}
        self._notify_status(self._last_status)
        logger.info("C++ Camera Service stopped.")

    def is_streaming(self) -> bool:
        return self._is_streaming and self._check_service_alive()

    def _check_service_alive(self) -> bool:
        try:
            r = requests.get(f"{CPP_BASE_URL}/api/v1/camera/status", timeout=0.5)
            return r.status_code == 200
        except Exception:
            return False

    @staticmethod
    def _parse_cpp_log_line(raw_line: str) -> Tuple[int, Optional[str]]:
        """
        消费 C++ 服务的标准输出，将其映射为对应的 Python 日志级别与干净的消息内容。
        由于 C++ 服务开启了 --raw-log 开关，C++ 自身已移除时间戳与 ANSI 颜色代码，
        输出格式为极简的: [I] [Camera] 消息内容...
        """
        line = raw_line.strip()
        if not line:
            return logging.INFO, None

        # 1. 极速 O(1) 前缀识别 C++ --raw-log: "[I] [Tag] ..."
        if len(line) >= 4 and line[0] == '[' and line[2] == ']' and line[3] == ' ':
            tag = line[1]
            lvl = logging.INFO
            if tag == 'D':
                lvl = logging.DEBUG
            elif tag == 'W':
                lvl = logging.WARNING
            elif tag == 'E':
                lvl = logging.ERROR
            return lvl, line[4:]

        # 2. 剥离 ZLM 第三方流媒体库原生输出 (若有)
        line_clean = ANSI_REGEX.sub('', line)
        m_zlm = ZLM_LOG_REGEX.match(line_clean)
        if m_zlm:
            zlm_lvl, msg = m_zlm.group(1), m_zlm.group(2).strip()
            lvl_map = {
                'D': logging.DEBUG,
                'I': logging.INFO,
                'W': logging.WARNING,
                'E': logging.ERROR,
                'T': logging.DEBUG
            }
            return lvl_map.get(zlm_lvl, logging.INFO), f"[ZLM] {msg}"

        # 3. 其他原始输出 (如 MPP 硬件编码器信息)
        return logging.INFO, line_clean

    def _stream_cpp_logs(self, proc: subprocess.Popen):
        """
        只干一件事：尽快把 C++ 子进程的 stdout 抽进有界缓冲。这条线程里绝不做 logging ——
        它一旦被 logging 卡住，管道就会写满，反压会把 C++ 服务里所有打日志的线程冻住。
        """
        try:
            if proc.stdout is None:
                return
            for raw_line in iter(proc.stdout.readline, ''):
                if not raw_line:
                    break
                with self._log_cond:
                    if len(self._log_buf) >= self._log_buf.maxlen:
                        self._log_dropped += 1   # deque 会自动丢最旧，这里只计数
                    self._log_buf.append(raw_line)
                    self._log_cond.notify()
        except Exception:
            pass
        finally:
            try:
                if proc.stdout:
                    proc.stdout.close()
            except Exception:
                pass

    def _emit_cpp_logs(self):
        """从缓冲取行、解析、走 python logging。慢、卡都只影响这里，不波及管道抽排。"""
        while not self._stop_event.is_set():
            raw_line = None
            dropped = 0
            with self._log_cond:
                while not self._log_buf and not self._stop_event.is_set():
                    self._log_cond.wait(timeout=1.0)
                if self._log_buf:
                    raw_line = self._log_buf.popleft()
                    dropped = self._log_dropped
                    self._log_dropped = 0
            if dropped:
                logger.warning("C++ 相机日志丢行 %d 条（消费端跟不上，宁可丢日志也不反压冻住服务）", dropped)
            if raw_line is None:
                continue
            level, msg = self._parse_cpp_log_line(raw_line)
            if msg:
                cpp_logger.log(level, msg)

    def _kill_stale_processes(self):
        """Clean up any orphan orbbec_camera_service processes."""
        try:
            subprocess.run(["pkill", "-9", "-x", "orbbec_camera_service"], 
                           timeout=1.0, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    def _health_monitor_loop(self):
        """Background thread monitoring the C++ service and broadcasting status via WebSocket."""
        last_online = False
        while self._is_streaming and not self._stop_event.is_set():
            st = self.get_status()
            current_online = st.get("online", False)
            if current_online != last_online or st != self._last_status:
                self._last_status = st
                last_online = current_online
                self._notify_status(st)

            # Auto-restart if process unexpectedly died while streaming is expected
            if self._is_streaming and self._process and self._process.poll() is not None:
                logger.warning("C++ Camera Service process exited unexpectedly. Restarting...")
                self.start_stream()

            self._stop_event.wait(1.5)

    def get_status(self) -> dict:
        """
        获取当前相机实时状态。
        """
        try:
            r = requests.get(f"{CPP_BASE_URL}/api/v1/camera/status", timeout=0.8)
            if r.status_code == 200:
                data = r.json().get("data", {})
                return {
                    "online": data.get("online", False),
                    "streaming": data.get("streaming", False),
                    "camera_model": data.get("camera_model", "Orbbec"),
                    "serial_number": data.get("serial_number", ""),
                    "firmware_version": data.get("firmware_version", ""),
                    "color_fps": data.get("color_fps", 0.0),
                    "depth_fps": data.get("depth_fps", 0.0),
                    "calibration_mode": data.get("calibration_mode", False),
                    "depth_stream_enabled": data.get("depth_stream_enabled", True),
                    "depth_align_enabled": data.get("depth_align_enabled", True),
                    "encoder": data.get("encoder", "none"),
                    "source": data.get("source", "orbbec"),
                    "intrinsics_loaded": data.get("intrinsics_loaded", False),
                    "total_frames": data.get("total_frames", 0)
                }
        except Exception:
            pass

        return {
            "online": False,
            "streaming": False,
            "camera_model": "Orbbec",
            "calibration_mode": False
        }

    def set_calibration_mode(self, enabled: bool, rows: int = 12, cols: int = 9, square_size_mm: float = 15.0) -> bool:
        """
        动态切换标定模式 / 常规模式。
        """
        try:
            payload = {
                "enabled": enabled,
                "rows": rows,
                "cols": cols,
                "square_size_mm": square_size_mm,
                "draw_corners": True
            }
            r = requests.post(f"{CPP_BASE_URL}/api/v1/camera/calibration_mode", json=payload, timeout=1.5)
            if r.status_code == 200:
                logger.info(f"C++ Camera calibration mode set to: {enabled}")
                self._notify_status(self.get_status())
                return True
        except Exception as e:
            logger.error(f"Failed to set calibration mode: {e}")
        return False

    def get_intrinsics(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        获取相机内参矩阵 (3x3) 与畸变参数 (5x1)。
        """
        try:
            r = requests.get(f"{CPP_BASE_URL}/api/v1/camera/intrinsics", timeout=1.0)
            if r.status_code == 200:
                d = r.json().get("data", {})
                k_mat = np.array(d.get("intrinsic_matrix", []))
                d_coeffs = np.array(d.get("distortion_coeffs", []))
                if k_mat.size == 9:
                    return k_mat, d_coeffs
            else:
                logger.warning(f"Failed to get intrinsics from C++ service, status: {r.status_code}")
        except Exception as e:
            logger.warning(f"Failed to get intrinsics: {e}")
        return None, None

    def get_intrinsics_dict(self) -> Dict[str, Any]:
        """
        获取完整的相机硬件内参字典 (含 width, height, intrinsic_matrix, distortion_coeffs 等)。
        """
        try:
            r = requests.get(f"{CPP_BASE_URL}/api/v1/camera/intrinsics", timeout=1.0)
            if r.status_code == 200:
                return r.json().get("data", {})
            else:
                logger.warning(f"Failed to get intrinsics dict from C++ service, status: {r.status_code}")
        except Exception as e:
            logger.warning(f"Failed to get intrinsics dict: {e}")
        return {}

    def get_stream_info(self, host_ip: str = "127.0.0.1") -> Dict[str, Any]:
        """
        获取当前 C++ 服务的各流媒体 URL 信息 (WebRTC, HTTP-FLV, RTSP, MJPEG)。
        """
        try:
            r = requests.get(f"{CPP_BASE_URL}/api/v1/stream/info", headers={"Host": host_ip}, timeout=1.0)
            if r.status_code == 200:
                return r.json().get("data", {})
            else:
                logger.warning(f"Failed to get stream info from C++ service, status: {r.status_code}")
        except Exception as e:
            logger.warning(f"Failed to get stream info: {e}")
        return {}

    def get_latest_corners(self) -> Optional[np.ndarray]:
        """
        获取最新检测到的标定板角点。
        """
        try:
            r = requests.get(f"{CPP_BASE_URL}/api/v1/camera/corners", timeout=0.5)
            if r.status_code == 200:
                d = r.json().get("data", {})
                if d.get("found", False) and d.get("corners"):
                    return np.array(d["corners"], dtype=np.float32)
        except Exception as e:
            logger.warning(f"Failed to get detected corners: {e}")
        return None

    def save_frame(
        self,
        save_dir: str = "data/calib",
        prefix: str = "sample",
        color_filename: Optional[str] = None,
        depth_filename: Optional[str] = None,
        save_color: bool = True,
        save_depth: bool = True,
        save_info_yaml: bool = True,
        color_format: str = "png",
        depth_format: str = "png_16bit",
        metadata: Optional[Dict[str, Any]] = None
    ) -> Optional[Dict[str, Any]]:
        """
        触发 C++ 底层异步无锁保存高清彩色图、16位深度图与元数据到本地目录 (Zero-Copy)。
        """
        try:
            payload = {
                "save_dir": save_dir,
                "prefix": prefix,
                "save_color": save_color,
                "save_depth": save_depth,
                "save_info_yaml": save_info_yaml,
                "color_format": color_format,
                "depth_format": depth_format,
                "metadata": metadata or {}
            }
            if color_filename:
                payload["color_filename"] = color_filename
            if depth_filename:
                payload["depth_filename"] = depth_filename

            r = requests.post(f"{CPP_BASE_URL}/api/v1/camera/save_frame", json=payload, timeout=6.0)
            if r.status_code == 200:
                return r.json().get("data", {})
            else:
                logger.error(f"C++ save_frame returned error status: {r.status_code}, response: {r.text}")
        except Exception as e:
            logger.error(f"Failed to trigger C++ save_frame: {e}", exc_info=True)
        return None


# 全局单例
camera_service = CameraService()
