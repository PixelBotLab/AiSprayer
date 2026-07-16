# -*- coding: utf-8 -*-
import os
import sys
import time
import cv2
import numpy as np
import yaml
from datetime import datetime

from PyQt5.QtWidgets import QMainWindow, QTabWidget, QMessageBox, QApplication, QFileDialog
from PyQt5.QtCore import QTimer, Qt, QPoint
from PyQt5.QtGui import QImage, QPixmap

# Setup paths
if getattr(sys, 'frozen', False):
    PROJECT_ROOT = os.path.dirname(sys.executable)
else:
    PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../.."))
    if PROJECT_ROOT not in sys.path:
        sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.hardware.robot.factory import get_robot
from aisprayer.core.hardware.robot.base_driver import RobotPose
from aisprayer.core.hardware.camera.factory import get_camera
from aisprayer.utils.config_helper import load_config, get_abs_path
from aisprayer.utils.hardware_helper import verify_hardware_consistency

from aisprayer.tools.calib.calib_ui.core.calib_solver import (
    clean_calibration_data, evaluate_data_diversity, 
    optimize_extrinsics_solve, calculate_rotation_error
)
from aisprayer.tools.calib.calib_ui.core.geometry_utils import (
    get_robust_depth, compute_local_normal, calculate_tool_orientation,
    calculate_tool_orientation_with_roll
)
from aisprayer.tools.calib.calib_ui.core.route_executor import VerificationRouteExecutor

from aisprayer.tools.calib.calib_ui.ui.calibrate_tab import CalibrateTab
from aisprayer.tools.calib.calib_ui.ui.verify_tab import VerifyTab

class CalibMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AI Sprayer - Robotic Calibration GUI")
        self.resize(3000, 2100)

        # Load global configuration
        self.config_path = os.path.join(PROJECT_ROOT, "configs/aisprayer_config.yaml")
        self.config = load_config(self.config_path, PROJECT_ROOT)
        self.h_cfg = self.config.get("hardware", {})
        self.c_cfg = self.config.get("calib", {})
        self.b_cfg = self.c_cfg.get("board", {})

        # Setup parameters
        self.camera_model = self.h_cfg.get("camera", {}).get("model", "orbbec")
        self.default_ip = self.h_cfg.get("robot", {}).get("ip", "192.168.2.14")
        self.default_port = str(self.h_cfg.get("robot", {}).get("port", "6001"))
        self.default_output_dir = get_abs_path(self.c_cfg.get("capture", {}).get("output_dir", "data/calib"), PROJECT_ROOT)

        # Board params
        self.board_rows = self.b_cfg.get("rows", 12)
        self.board_cols = self.b_cfg.get("cols", 9)
        self.square_size = self.b_cfg.get("square_size_mm", 15.0)
        self.pattern_size = (self.board_cols - 1, self.board_rows - 1)

        # Hardware variables
        self.cam = None
        self.cam_connected = False
        
        self.robot = None
        self.robot_connected = False

        self.current_color = None
        self.current_depth = None
        
        # Save state
        self.save_dir = None
        self.yaml_path = None
        self.cap_info = None

        # Preview & selection state
        self.selected_sample_id = None
        self.preview_image = None

        # Verify state
        self.calib_data = None
        self.target_uv = None
        self.target_n_cam = None
        self.target_pose_data = None

        # Verify drawing & sequence execution state
        self.verify_items = []
        self.current_draw_points = []
        self.current_draw_poses = []
        self.hover_pos = None
        
        self.executor = VerificationRouteExecutor(self.robot, self.config)

        # Initialize user interface
        self.init_ui()

        # Connect Camera at startup
        self.start_camera()

        # Frame update timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        self.last_heartbeat_time = 0

        # 默认加载最近的标定结果
        latest_calib = None
        latest_time = 0
        calib_root = os.path.join(PROJECT_ROOT, "data", "calib")
        if os.path.exists(calib_root):
            for root, dirs, files in os.walk(calib_root):
                for file in files:
                    if file == "calibration_result.yaml":
                        full_path = os.path.join(root, file)
                        try:
                            mtime = os.path.getmtime(full_path)
                            if mtime > latest_time:
                                latest_time = mtime
                                latest_calib = full_path
                        except Exception:
                            pass

        if not latest_calib:
            p_cfg = self.config.get("vision", {}).get("planner", {})
            default_calib = get_abs_path(p_cfg.get("calib_path", "configs/calib/calibration_result.yaml"), PROJECT_ROOT)
            if os.path.exists(default_calib):
                latest_calib = default_calib

        if latest_calib:
            self.load_calib_file(latest_calib)

    def init_ui(self):
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.statusBar().showMessage("Ready")

        self.calibrate_tab = CalibrateTab(self)
        self.verify_tab = VerifyTab(self)

        self.tabs.addTab(self.calibrate_tab, "Calibrate")
        self.tabs.addTab(self.verify_tab, "Verify")

        # Alias widget references for compatibility with existing code
        self.cap_video = self.calibrate_tab.cap_video
        self.samples_layout = self.calibrate_tab.samples_layout
        self.samples_group = self.calibrate_tab.samples_group
        self.samples_scroll = self.calibrate_tab.samples_scroll
        self.samples_content = self.calibrate_tab.samples_content
        self.cam_group = self.calibrate_tab.cam_group
        self.btn_reconnect_cam = self.calibrate_tab.btn_reconnect_cam
        self.conn_group = self.calibrate_tab.conn_group
        self.txt_ip = self.calibrate_tab.txt_ip
        self.txt_port = self.calibrate_tab.txt_port
        self.combo_robot_type = self.calibrate_tab.combo_robot_type
        self.btn_connect_robot = self.calibrate_tab.btn_connect_robot
        self.lbl_save_dir = self.calibrate_tab.lbl_save_dir
        self.jog_inputs = self.calibrate_tab.jog_inputs
        self.spin_step_xyz = self.calibrate_tab.spin_step_xyz
        self.spin_step_abc = self.calibrate_tab.spin_step_abc
        self.txt_calib_log = self.calibrate_tab.txt_calib_log

        self.ver_video = self.verify_tab.ver_video
        self.lbl_calib_path = self.verify_tab.lbl_calib_path
        self.lbl_coord_cam = self.verify_tab.lbl_coord_cam
        self.lbl_coord_base = self.verify_tab.lbl_coord_base
        self.lbl_normal_base = self.verify_tab.lbl_normal_base
        self.lbl_real_x = self.verify_tab.lbl_real_x
        self.lbl_real_y = self.verify_tab.lbl_real_y
        self.lbl_real_z = self.verify_tab.lbl_real_z
        self.lbl_real_a = self.verify_tab.lbl_real_a
        self.lbl_real_b = self.verify_tab.lbl_real_b
        self.lbl_real_c = self.verify_tab.lbl_real_c
        self.spin_ver_offset = self.verify_tab.spin_ver_offset
        self.btn_ver_move = self.verify_tab.btn_ver_move
        self.btn_go_home = self.verify_tab.btn_go_home
        self.spin_safe_x = self.verify_tab.spin_safe_x
        self.spin_safe_y = self.verify_tab.spin_safe_y
        self.spin_safe_z = self.verify_tab.spin_safe_z
        self.spin_safe_a = self.verify_tab.spin_safe_a
        self.spin_safe_b = self.verify_tab.spin_safe_b
        self.spin_safe_c = self.verify_tab.spin_safe_c
        self.btn_move_safe = self.verify_tab.btn_move_safe
        self.slider_speed = self.verify_tab.slider_speed
        self.slider_movl_speed = self.verify_tab.slider_movl_speed
        self.spin_tool_num = self.verify_tab.spin_tool_num
        self.slider_acc = self.verify_tab.slider_acc
        self.slider_dec = self.verify_tab.slider_dec
        self.progress_bar = self.verify_tab.progress_bar
        self.txt_verify_log = self.verify_tab.txt_verify_log

    def start_camera(self):
        """
        初始化并启动相机设备。
        步骤：
          1. 实例化指定型号的相机驱动（如 Orbbec），并尝试开启数据流。
          2. 连接成功后，读取当前相机的分辨率和内参。
          3. 若当前未选择数据保存路径，则默认创建一个以当前日期命名的标定样本目录。
        """
        try:
            self.cam_group.setTitle("Camera: Connecting...")
            self.cam_group.setStyleSheet("QGroupBox::title { color: #FF9800; font-weight: bold; }")
            QApplication.processEvents()

            # 通过工厂方法获取并启动相机
            self.cam = get_camera(self.camera_model)
            self.cam.start()
            self.cam_connected = True
            
            self.cam_group.setTitle("Camera: Connected")
            self.cam_group.setStyleSheet("QGroupBox::title { color: #4CAF50; font-weight: bold; }")
            
            if not self.save_dir:
                ts = datetime.now().strftime("%Y%m%d")
                self.setup_save_dir(os.path.join(self.default_output_dir, f"calib_{ts}"))
        except Exception as e:
            self.cam_connected = False
            self.cam_group.setTitle("Camera: Offline")
            self.cam_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")
            print(f"Camera start failed: {e}")

    def toggle_robot_connection(self):
        """
        切换机器人控制连接状态。
        如果是断开状态：
          - 获取 UI 输入的 IP 和端口。
          - 实例化 InexbotDriver 控制器并进行非阻塞启动 (startup)。
          - 若建立连接成功，将驱动对象绑定给运动执行器，并读取实时位姿以刷新 Jog 面板的输入控件值。
        如果是连接状态：
          - 断开连接，释放网络套接字句柄。
        """
        if not self.robot_connected:
            ip = self.txt_ip.text().strip()
            port = self.txt_port.text().strip()

            self.conn_group.setTitle("Robot Connection: Connecting...")
            self.conn_group.setStyleSheet("QGroupBox::title { color: #FF9800; font-weight: bold; }")
            QApplication.processEvents()

            try:
                robot_type = self.combo_robot_type.currentText().strip()
                self.robot = get_robot(robot_type, ip=ip, port=port)
                if self.robot is None:
                    raise ValueError(f"Failed to instantiate robot driver for type: {robot_type}")
                self.executor.robot = self.robot
                if not self.robot.startup(timeout=5.0):
                    QMessageBox.critical(self, "Error", "Robot startup failed! Make sure controller IP/Port are correct.")
                    self.robot = None
                    self.executor.robot = None
                    self.conn_group.setTitle("Robot: Disconnected")
                    self.conn_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")
                    return
 
                self.robot_connected = True
                self.btn_connect_robot.setText("Disconnect Robot")
                self.btn_connect_robot.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
                
                self.conn_group.setTitle("Robot: Connected")
                self.conn_group.setStyleSheet("QGroupBox::title { color: #4CAF50; font-weight: bold; }")

                # Sync speed setting from UI to robot
                self.change_robot_speed(self.slider_speed.value())
                self.change_robot_tool_number(self.spin_tool_num.value())
 
                # 读取当前机械臂的物理姿态填充到 UI 面板
                self.read_robot_pose()
            except Exception as e:
                self.robot = None
                self.executor.robot = None
                self.conn_group.setTitle("Robot: Disconnected")
                self.conn_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")
                QMessageBox.critical(self, "Error", f"Failed to connect to robot: {e}")
        else:
            self.disconnect_robot()
 
    def disconnect_robot(self):
        """断开机器人连接，重置内部状态及 UI 文字样式。"""
        if self.robot:
            try: self.robot.shutdown()
            except: pass
            self.robot = None
            self.executor.robot = None
        self.robot_connected = False
        self.btn_connect_robot.setText("Connect Robot")
        self.btn_connect_robot.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        self.conn_group.setTitle("Robot: Disconnected")
        self.conn_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")

    def select_save_dir(self):
        d = QFileDialog.getExistingDirectory(self, "Select Save Directory", PROJECT_ROOT)
        if d:
            if os.path.exists(os.path.join(d, "calibration_info.yaml")):
                target_dir = d
            else:
                ts = datetime.now().strftime("%Y%m%d")
                target_dir = os.path.join(d, f"calib_{ts}")
            self.setup_save_dir(target_dir)

    def setup_save_dir(self, d):
        self.save_dir = d
        os.makedirs(self.save_dir, exist_ok=True)
        self.yaml_path = os.path.join(self.save_dir, "calibration_info.yaml")
        self.lbl_save_dir.setText(self.save_dir)
        
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r', encoding='utf-8') as f:
                self.cap_info = yaml.safe_load(f)
            if self.cam_connected and self.cam:
                ok, msg = verify_hardware_consistency(live=self.cam, scan=self.cap_info.get("camera_params", {}))
                if not ok:
                    self.txt_calib_log.append(f"[Warning] Camera params mismatch: {msg}")
            self.txt_calib_log.append(f"Loaded {len(self.cap_info.get('samples', []))} samples from {self.yaml_path}")
        else:
            K, D = None, None
            if self.cam_connected and self.cam:
                K, D = self.cam.get_intrinsics()
            self.cap_info = {
                "version": "1.0",
                "camera_params": {
                    "camera_model": self.camera_model,
                    "width": self.cam.width if (self.cam_connected and self.cam) else 640,
                    "height": self.cam.height if (self.cam_connected and self.cam) else 480,
                    "intrinsic_matrix": K.tolist() if K is not None else [],
                    "distortion_coeffs": D.tolist() if D is not None else []
                },
                "board_params": {
                    "rows": self.board_rows,
                    "cols": self.board_cols,
                    "square_size_mm": self.square_size,
                    "pattern_size_inner": [self.pattern_size[0], self.pattern_size[1]]
                },
                "samples": []
            }
            with open(self.yaml_path, 'w', encoding='utf-8') as f:
                yaml.dump(self.cap_info, f, default_flow_style=False)
            self.txt_calib_log.append(f"Initialized new calibration save info at: {self.yaml_path}")
        
        self.refresh_samples_list()

    def update_frame(self):
        """
        主循环更新定时器（每30ms触发一次，约33FPS）。
        负责处理以下任务：
          1. 维持机器人心跳：限制每 2.0s 向 Inexbot 发送一次状态查询，以防止驱动层 socket 超时断开。
          2. 监控验证运动执行进度：当运动器处于 moving 状态且距离上一次指令发出超过 0.8s 时，
             如果机器人控制器报告空闲，代表当前路点运动已完成，触发状态机执行下一个路点。
          3. 节流读取机器人坐标：每 150ms 读取一次机械臂物理姿态，以减轻网络通信负荷，并同步更新 UI 进度条。
          4. 渲染视频帧：
             - 如果选中了侧边栏的样本图片，则停止显示实时流，改为显示带有棋盘格角点检测绿线的样本静态预览图。
             - 如果没有选中样本且相机在线，获取实时 RGB+Depth 图像帧，并分别向 Calibrate 和 Verify 界面进行图像与三维法线向量箭头叠加渲染。
        """
        # 1. 维持机器人套接字活跃心跳（防止长连接断开）
        if self.robot_connected and self.robot:
            if time.time() - self.last_heartbeat_time > 2.0:
                try:
                    self.robot.get_running_state()
                except:
                    pass
                self.last_heartbeat_time = time.time()

        # 2. 验证路径非阻塞跟踪监控
        if self.executor.is_moving():
            if self.robot_connected and self.robot:
                now = time.time()
                # 设定 0.8s 的保护时间间隔，避免在刚发送指令的瞬间误判为已经到达
                if now - self.executor.last_move_cmd_time > 0.8:
                    if self.robot.is_robot_idle():
                        self.executor.completed_waypoints += 1
                        self.execute_next_verify_step()

        # 3. 节流（每150ms）读取机械臂实时坐标并更新运动进度
        if self.robot_connected and self.robot:
            now = time.time()
            if not hasattr(self, 'last_pose_query_time') or now - self.last_pose_query_time > 0.15:
                self.last_pose_query_time = now
                self.update_realtime_pose_and_progress()
        else:
            # 机器人离线时清除坐标显示
            if hasattr(self, 'lbl_real_x') and self.lbl_real_x.text() != "X: N/A":
                self.lbl_real_x.setText("X: N/A")
                self.lbl_real_y.setText("Y: N/A")
                self.lbl_real_z.setText("Z: N/A")
                self.lbl_real_a.setText("A: N/A")
                self.lbl_real_b.setText("B: N/A")
                self.lbl_real_c.setText("C: N/A")

        # 4. 选择样本预览时的图片渲染
        active_tab = self.tabs.currentIndex()
        if active_tab == 0 and self.selected_sample_id is not None and self.preview_image is not None:
            display_frame = self.preview_image.copy()
            gray = cv2.cvtColor(self.preview_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
            if ret:
                cv2.drawChessboardCorners(display_frame, self.pattern_size, corners, ret)
            
            cv2.putText(display_frame, f"PREVIEW - SAMPLE {self.selected_sample_id}", (15, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2, cv2.LINE_AA)
            cv2.putText(display_frame, "Click item again to return to Live Video", (15, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1, cv2.LINE_AA)
            self.render_image_to_label(display_frame, self.cap_video)
            return

        if not self.cam_connected or not self.cam:
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Camera Offline. Click 'Retry Camera' to connect.", (50, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            self.render_image_to_label(placeholder, self.cap_video)
            self.render_image_to_label(placeholder, self.ver_video)
            return

        color, depth = self.cam.get_frame()
        if color is None:
            return

        self.current_color = color.copy()
        self.current_depth = depth.copy() if depth is not None else None

        if active_tab == 0:
            display_frame = color.copy()
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
            if ret:
                cv2.drawChessboardCorners(display_frame, self.pattern_size, corners, ret)
            
            if self.cap_info is not None:
                count = len(self.cap_info.get('samples', []))
                text = f"Samples: {count}"
            else:
                text = "Samples: -"
            cv2.putText(display_frame, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            self.render_image_to_label(display_frame, self.cap_video)

        elif active_tab == 1:
            display_frame = color.copy()

            def draw_point_normal(img, u, v, n_cam, color_norm, thickness=2):
                if n_cam is not None and self.calib_data:
                    K = np.array(self.calib_data["camera_params"]["intrinsic_matrix"])
                    fx, fy = K[0, 0], K[1, 1]
                    cx, cy = K[0, 2], K[1, 2]
                    z_val = float(self.current_depth[v, u]) if self.current_depth is not None else 0.0
                    if z_val > 0:
                        x_val = (u - cx) * z_val / fx
                        y_val = (v - cy) * z_val / fy
                        p_cam_origin = np.array([x_val, y_val, z_val])
                        p_cam_normal_tip = p_cam_origin + n_cam * 150.0
                        if p_cam_normal_tip[2] > 0:
                            u_tip = int(fx * p_cam_normal_tip[0] / p_cam_normal_tip[2] + cx)
                            v_tip = int(fy * p_cam_normal_tip[1] / p_cam_normal_tip[2] + cy)
                            cv2.arrowedLine(img, (u_tip, v_tip), (u, v), color_norm, thickness, tipLength=0.4)

            def draw_dashed_line(img, pt1, pt2, color, thickness=1, gap=6):
                dist = np.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])
                if dist == 0:
                    return
                n_segments = int(dist / gap)
                if n_segments <= 0:
                    cv2.line(img, pt1, pt2, color, thickness, cv2.LINE_AA)
                    return
                pts_x = np.linspace(pt1[0], pt2[0], n_segments + 1)
                pts_y = np.linspace(pt1[1], pt2[1], n_segments + 1)
                for i in range(0, n_segments, 2):
                    start = (int(pts_x[i]), int(pts_y[i]))
                    end_idx = min(i + 1, n_segments)
                    end = (int(pts_x[end_idx]), int(pts_y[end_idx]))
                    cv2.line(img, start, end, color, thickness, cv2.LINE_AA)

            for item in self.verify_items:
                is_visited = (item.get("status") == "visited")
                item_color = (255, 0, 0) if is_visited else (0, 255, 0)
                norm_color = (180, 50, 50) if is_visited else (0, 0, 255)
                
                points = item["points"]
                poses = item["pose_data"]
                
                for idx, pt in enumerate(points):
                    cv2.circle(display_frame, pt, 5, item_color, -1)
                    if idx < len(poses):
                        draw_point_normal(display_frame, pt[0], pt[1], poses[idx]["n_cam"], norm_color, thickness=2)
                
                if len(points) > 1:
                    for i in range(len(points) - 1):
                        cv2.arrowedLine(display_frame, points[i], points[i+1], item_color, 2, tipLength=0.15)
                
                if points:
                    first_pt = points[0]
                    cv2.putText(display_frame, f"#{item['id']}", (first_pt[0] + 8, first_pt[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(display_frame, f"#{item['id']}", (first_pt[0] + 8, first_pt[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, item_color, 1, cv2.LINE_AA)

            if self.current_draw_points:
                temp_color = (0, 165, 255)
                temp_norm_color = (0, 0, 255)
                for idx, pt in enumerate(self.current_draw_points):
                    cv2.circle(display_frame, pt, 5, temp_color, -1)
                    if idx < len(self.current_draw_poses):
                        draw_point_normal(display_frame, pt[0], pt[1], self.current_draw_poses[idx]["n_cam"], temp_norm_color, thickness=2)
                
                if len(self.current_draw_points) > 1:
                    for i in range(len(self.current_draw_points) - 1):
                        cv2.arrowedLine(display_frame, self.current_draw_points[i], self.current_draw_points[i+1], temp_color, 2, tipLength=0.15)

                # 鼠标移动时绘制到当前悬停点的虚拟线段及虚拟法向量箭头
                if self.hover_pos is not None:
                    last_pt = self.current_draw_points[-1]
                    virtual_color = (255, 255, 0) # 虚拟引导元素采用红色
                    # 细线宽、虚线段表示
                    draw_dashed_line(display_frame, last_pt, self.hover_pos, virtual_color, thickness=1, gap=6)
                    
                    uh, vh = self.hover_pos
                    if self.current_depth is not None and self.calib_data:
                        K = np.array(self.calib_data["camera_params"]["intrinsic_matrix"])
                        n_cam_h = compute_local_normal(self.current_depth, uh, vh, K)
                        if n_cam_h is not None:
                            # 虚拟法线箭头也为红色，采用较细的粗细度 1
                            draw_point_normal(display_frame, uh, vh, n_cam_h, virtual_color, thickness=1)
                
                first_pt = self.current_draw_points[0]
                cv2.putText(display_frame, "Drawing...", (first_pt[0] + 8, first_pt[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, temp_color, 1, cv2.LINE_AA)

            self.render_image_to_label(display_frame, self.ver_video)

    def render_image_to_label(self, frame, label):
        h, w, c = frame.shape
        if hasattr(label, 'aspect_ratio'):
            new_ratio = h / w
            if abs(label.aspect_ratio - new_ratio) > 0.001:
                label.aspect_ratio = new_ratio
                label.updateGeometry()
        
        bytes_per_line = 3 * w
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg)
        label.setPixmap(pix.scaled(label.width(), label.height(), Qt.KeepAspectRatio))

    def capture_sample(self):
        if not self.robot_connected or not self.robot:
            QMessageBox.warning(self, "Warning", "Robot is not connected. Connect the robot to read its pose for capture.")
            return
        if not self.save_dir:
            QMessageBox.warning(self, "Warning", "No save folder selected.")
            return
        if self.current_color is None:
            QMessageBox.warning(self, "Warning", "No camera color stream.")
            return

        pose = self.robot.get_current_pose()
        if pose is None:
            QMessageBox.warning(self, "Warning", "Could not get current robot pose.")
            return

        pose_list = pose.to_list()
        if any(v < -2500.0 for v in pose_list[:3]):
            QMessageBox.warning(self, "Error", "Invalid pose data obtained from robot SDK.")
            return

        count = len(self.cap_info["samples"]) + 1
        img_name = f"image_{count:03d}.png"
        img_path = os.path.join(self.save_dir, img_name)

        cv2.imwrite(img_path, self.current_color)

        sample = {
            "id": count,
            "image_file": img_name,
            "robot_pose": {
                "x": round(pose.x, 3), "y": round(pose.y, 3), "z": round(pose.z, 3),
                "a": round(pose.a, 5), "b": round(pose.b, 5), "c": round(pose.c, 5)
            }
        }
        self.cap_info["samples"].append(sample)

        with open(self.yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.cap_info, f, default_flow_style=False)

        self.txt_calib_log.append(f"[OK] Captured Sample {count}: {img_name}")
        self.txt_calib_log.append(f"     Pose: X:{pose.x:.1f} Y:{pose.y:.1f} Z:{pose.z:.1f} A:{pose.a:.4f} B:{pose.b:.4f} C:{pose.c:.4f}")
        self.refresh_samples_list()

    def run_calibration(self):
        if not self.save_dir or not os.path.exists(self.yaml_path):
            QMessageBox.warning(self, "Warning", "Save directory does not contain calibration info!")
            return

        self.txt_calib_log.clear()
        self.txt_calib_log.append("====== Starting Calibration Process ======")

        timer_was_active = self.timer.isActive()
        if timer_was_active:
            self.timer.stop()

        progress = None
        try:
            with open(self.yaml_path, 'r', encoding='utf-8') as f:
                info = yaml.safe_load(f)

            total_samples = len(info.get("samples", []))
            if total_samples < 3:
                self.txt_calib_log.append("[-] Calibration requires at least 3 samples.")
                return

            K = np.array(info["camera_params"]["intrinsic_matrix"])
            D = np.array(info["camera_params"]["distortion_coeffs"])
            pattern_size = tuple(info["board_params"]["pattern_size_inner"])
            sq_size = info["board_params"]["square_size_mm"]

            objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * sq_size

            all_samples = []
            self.txt_calib_log.append("[*] Extracting corners and solving PnP...")

            from PyQt5.QtWidgets import QProgressDialog
            progress = QProgressDialog("Loading calibration samples...", "Cancel", 0, total_samples, self)
            progress.setWindowModality(Qt.WindowModal)
            progress.setWindowTitle("Calibration Process")
            progress.setMinimumDuration(0)
            progress.setValue(0)
            QApplication.processEvents()

            for idx, s in enumerate(info["samples"]):
                if progress.wasCanceled():
                    self.txt_calib_log.append("[-] Calibration cancelled by user.")
                    return

                progress.setLabelText(f"Processing sample {idx+1}/{total_samples} (ID: {s['id']})...")
                progress.setValue(idx)
                QApplication.processEvents()

                pose = s["robot_pose"]
                if any(abs(v) > 2500 for v in [pose['x'], pose['y'], pose['z']]):
                    self.txt_calib_log.append(f"  [!] Skipped sample {s['id']} (invalid position)")
                    continue

                img_path = os.path.join(self.save_dir, s["image_file"])
                img = cv2.imread(img_path)
                if img is None:
                    self.txt_calib_log.append(f"  [!] Skipped sample {s['id']} (image load failed)")
                    continue

                display_img = img.copy()
                ret, corners = cv2.findChessboardCorners(img, pattern_size, None)
                if ret:
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
                    cv2.drawChessboardCorners(display_img, pattern_size, corners, ret)
                    
                    _, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
                    R_cb, _ = cv2.Rodrigues(rvec)
                    all_samples.append({
                        "id": s["id"], 
                        "R_cb": R_cb, 
                        "t_cb": tvec.flatten(), 
                        "pose": pose
                    })
                else:
                    self.txt_calib_log.append(f"  [!] Failed to extract corners from sample {s['id']}")

                status_text = f"Sample ID {s['id']}: {'SUCCESS' if ret else 'FAILED'}"
                text_color = (0, 255, 0) if ret else (0, 0, 255)
                cv2.putText(display_img, status_text, (15, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2, cv2.LINE_AA)
                self.render_image_to_label(display_img, self.cap_video)
                
                time.sleep(0.15)
                QApplication.processEvents()

            progress.setValue(total_samples)
            progress.close()
            progress = None

            if len(all_samples) < 3:
                self.txt_calib_log.append("[-] Verification failed: insufficient valid chessboard corners found.")
                return

            self.txt_calib_log.append("[*] Cleaning data based on motion displacement...")
            QApplication.processEvents()
            clean_thr = self.c_cfg.get("cleaning_threshold", 0.05)
            samples = clean_calibration_data(all_samples, threshold=clean_thr, log_callback=self.txt_calib_log.append)

            if len(samples) < 3:
                self.txt_calib_log.append("[-] Calibration failed: not enough clean samples.")
                return

            diversity = evaluate_data_diversity(samples)
            self.txt_calib_log.append(f"[*] Diversity score: {diversity['score']:.1f} / 100")
            QApplication.processEvents()

            self.txt_calib_log.append("[*] Searching for optimal camera extrinsics & chessboard offset...")
            QApplication.processEvents()
            best_res = optimize_extrinsics_solve(samples)
            if not best_res:
                self.txt_calib_log.append("[-] Optimization solver failed.")
                return

            R_bc, t_bc, t_off, err, order, s_vec = best_res
            r_err_mean = calculate_rotation_error(samples, best_res)

            self.txt_calib_log.append("\n====== Result ======")
            self.txt_calib_log.append(f"Euler Sequence: {order}, Signs: {s_vec}")
            self.txt_calib_log.append(f"Chessboard TCP Offset: {np.round(t_off, 2)} mm")
            self.txt_calib_log.append(f"Mean Reprojection Translation Error: {err:.3f} mm")
            self.txt_calib_log.append(f"Mean Angular Error: {r_err_mean:.3f}°")

            T_bc = np.eye(4)
            T_bc[:3, :3] = R_bc
            T_bc[:3, 3] = t_bc
            xyz = t_bc.tolist()
            from scipy.spatial.transform import Rotation as R_tool
            rpy = R_tool.from_matrix(R_bc).as_euler('xyz', degrees=True).tolist()

            output_res = {
                "metadata": {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "source_data_dir": self.save_dir,
                    "reprojection_error_mm": float(err),
                    "rotation_error_deg": float(r_err_mean),
                    "samples_total": len(all_samples),
                    "samples_used": len(samples),
                    "optimization_config": {
                        "axis_order": order,
                        "sign_vector": [int(x) for x in s_vec]
                    }
                },
                "camera_pose_base": {
                    "x": xyz[0], "y": xyz[1], "z": xyz[2],
                    "roll_deg": rpy[0], "pitch_deg": rpy[1], "yaw_deg": rpy[2]
                },
                "T_base_camera": T_bc.tolist(),
                "camera_params": info["camera_params"],
                "board_params": info["board_params"],
                "chessboard_offset": t_off.tolist()
            }

            calib_out_path = os.path.join(self.save_dir, "calibration_result.yaml")
            with open(calib_out_path, 'w', encoding='utf-8') as f:
                yaml.dump(output_res, f, default_flow_style=False)
            
            self.txt_calib_log.append(f"\n[+] Saved calibration matrix to: {calib_out_path}")
            QMessageBox.information(self, "Success", f"Calibration computed successfully!\nError: {err:.3f} mm\nSaved to folder.")

            self.load_calib_file(calib_out_path)

        except Exception as e:
            self.txt_calib_log.append(f"[-] Calibration crashed: {e}")
            QMessageBox.critical(self, "Error", f"Calibration execution failed: {e}")
        finally:
            if progress is not None:
                progress.close()
            if timer_was_active:
                self.timer.start(30)

    def refresh_samples_list(self):
        self.calibrate_tab.refresh_samples_list()

    def handle_sample_select(self, sid):
        if self.selected_sample_id == sid:
            self.selected_sample_id = None
            self.preview_image = None
        else:
            self.selected_sample_id = sid
            sample = next((s for s in self.cap_info["samples"] if s["id"] == sid), None)
            if sample:
                img_path = os.path.join(self.save_dir, sample["image_file"])
                if os.path.exists(img_path):
                    self.preview_image = cv2.imread(img_path)
                else:
                    self.preview_image = None
            else:
                self.preview_image = None
                
        self.refresh_samples_list()

    def delete_sample(self, sid):
        if not self.cap_info or "samples" not in self.cap_info:
            return
        
        reply = QMessageBox.question(
            self, 'Confirm Delete', 
            f"Are you sure you want to delete Sample {sid}?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            if self.selected_sample_id == sid:
                self.selected_sample_id = None
                self.preview_image = None

            samples = self.cap_info["samples"]
            idx = next((i for i, s in enumerate(samples) if s["id"] == sid), -1)
            if idx != -1:
                s = samples[idx]
                img_path = os.path.join(self.save_dir, s["image_file"])
                if os.path.exists(img_path):
                    try:
                        os.remove(img_path)
                    except Exception as e:
                        print(f"Failed to delete image file: {e}")
                
                samples.pop(idx)
                
                for new_idx, s in enumerate(samples):
                    old_id = s["id"]
                    new_id = new_idx + 1
                    s["id"] = new_id
                    old_img_name = s["image_file"]
                    new_img_name = f"image_{new_id:03d}.png"
                    if old_img_name != new_img_name:
                        old_img_path = os.path.join(self.save_dir, old_img_name)
                        new_img_path = os.path.join(self.save_dir, new_img_name)
                        if os.path.exists(old_img_path):
                            try:
                                os.rename(old_img_path, new_img_path)
                                s["image_file"] = new_img_name
                            except Exception as e:
                                print(f"Failed to rename image file: {e}")
                
                with open(self.yaml_path, 'w', encoding='utf-8') as f:
                    yaml.dump(self.cap_info, f, default_flow_style=False)
                
                self.txt_calib_log.append(f"[OK] Deleted Sample {sid} and updated remaining indices.")
                self.refresh_samples_list()

    def read_robot_pose(self):
        if not self.robot_connected or not self.robot:
            QMessageBox.warning(self, "Warning", "Robot is not connected. Connect the robot first.")
            return

        pose = self.robot.get_current_pose()
        if pose is None:
            QMessageBox.warning(self, "Warning", "Could not query current pose from robot.")
            return

        self.jog_inputs['X'].setValue(pose.x)
        self.jog_inputs['Y'].setValue(pose.y)
        self.jog_inputs['Z'].setValue(pose.z)
        self.jog_inputs['A'].setValue(np.degrees(pose.a))
        self.jog_inputs['B'].setValue(np.degrees(pose.b))
        self.jog_inputs['C'].setValue(np.degrees(pose.c))

    def move_to_jog_pose(self):
        if not self.robot_connected or not self.robot:
            QMessageBox.warning(self, "Warning", "Robot is not connected. Connect the robot first.")
            return

        x = self.jog_inputs['X'].value()
        y = self.jog_inputs['Y'].value()
        z = self.jog_inputs['Z'].value()
        a = np.radians(self.jog_inputs['A'].value())
        b = np.radians(self.jog_inputs['B'].value())
        c = np.radians(self.jog_inputs['C'].value())

        target_pose = RobotPose(x, y, z, a, b, c)

        planner_cfg = self.config.get("vision", {}).get("planner", {})
        lim = planner_cfg.get("workspace_limits", {})
        if lim:
            if not (lim.get("x", [-9999, 9999])[0] <= x <= lim.get("x", [-9999, 9999])[1]):
                QMessageBox.warning(self, "Out of Workspace", f"Target X ({x}) exceeds safety limits.")
                return
            if not (lim.get("y", [-9999, 9999])[0] <= y <= lim.get("y", [-9999, 9999])[1]):
                QMessageBox.warning(self, "Out of Workspace", f"Target Y ({y}) exceeds safety limits.")
                return
            if not (lim.get("z", [-9999, 9999])[0] <= z <= lim.get("z", [-9999, 9999])[1]):
                QMessageBox.warning(self, "Out of Workspace", f"Target Z ({z}) exceeds safety limits.")
                return

        if not self.robot.is_reachable(target_pose, "MOVJ"):
            QMessageBox.warning(self, "Unreachable", "The target jog pose is kinematics-unreachable (MOVJ).")
            return

        self.txt_calib_log.append(f"[Move] Target: X:{x:.2f} Y:{y:.2f} Z:{z:.2f} A:{self.jog_inputs['A'].value():.2f} B:{self.jog_inputs['B'].value():.2f} C:{self.jog_inputs['C'].value():.2f}")
        self.robot.move_j(target_pose, wait=False)

    def jog_step(self, axis, direction):
        if not self.robot_connected or not self.robot:
            QMessageBox.warning(self, "Warning", "Robot is not connected. Connect the robot first.")
            return

        step_xyz = self.spin_step_xyz.value()
        step_abc = self.spin_step_abc.value()

        current_val = self.jog_inputs[axis].value()
        if axis in ['X', 'Y', 'Z']:
            new_val = current_val + direction * step_xyz
        else:
            new_val = current_val + direction * step_abc
            if new_val > 180.0: new_val -= 360.0
            if new_val < -180.0: new_val += 360.0

        self.jog_inputs[axis].setValue(new_val)
        self.move_to_jog_pose()

    def select_calib_file(self):
        from PyQt5.QtWidgets import QFileDialog
        f, _ = QFileDialog.getOpenFileName(self, "Select Calibration File", PROJECT_ROOT, "YAML Files (*.yaml)")
        if f:
            self.load_calib_file(f)

    def load_calib_file(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                self.calib_data = yaml.safe_load(f)
            self.lbl_calib_path.setText(path)
            self.btn_ver_move.setEnabled(False)
            self.target_uv = None
            self.target_n_cam = None
            self.target_pose_data = None
            if hasattr(self, 'txt_verify_log'):
                self.txt_verify_log.append(f"[OK] Loaded calibration file: {path}")
            
            # Auto-enable Move button if loaded and verify items exist
            if self.verify_items:
                self.btn_ver_move.setEnabled(True)
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Could not load calibration file: {e}")
            if hasattr(self, 'txt_verify_log'):
                self.txt_verify_log.append(f"[-] Failed to load calibration file: {e}")

    def clear_last_point(self):
        if self.current_draw_points:
            pt = self.current_draw_points.pop()
            self.current_draw_poses.pop()
            self.txt_verify_log.append(f"[*] Cleared last point in current segment: U={pt[0]}, V={pt[1]}")
            if not self.current_draw_points:
                self.lbl_coord_cam.setText("N/A")
                self.lbl_coord_base.setText("N/A")
                self.lbl_normal_base.setText("N/A")
        elif self.verify_items:
            last_item = self.verify_items[-1]
            if last_item["points"]:
                pt = last_item["points"].pop()
                last_item["pose_data"].pop()
                self.txt_verify_log.append(f"[*] Cleared last point from Item #{last_item['id']}: U={pt[0]}, V={pt[1]}")
                if not last_item["points"]:
                    self.verify_items.pop()
                    self.txt_verify_log.append(f"[*] Item #{last_item['id']} has no points remaining, removed.")
            else:
                self.verify_items.pop()
                self.txt_verify_log.append(f"[*] Removed empty Item #{last_item['id']}.")

            if not self.verify_items:
                self.btn_ver_move.setEnabled(False)
                self.lbl_coord_cam.setText("N/A")
                self.lbl_coord_base.setText("N/A")
                self.lbl_normal_base.setText("N/A")
        else:
            self.txt_verify_log.append("[!] No points to clear.")

    def finish_current_draw_item(self, warn_if_empty=True):
        if not self.current_draw_points:
            if warn_if_empty:
                QMessageBox.warning(self, "Warning", "No points drawn in current segment yet.")
            return

        item_id = len(self.verify_items) + 1
        item = {
            "id": item_id,
            "points": list(self.current_draw_points),
            "pose_data": list(self.current_draw_poses),
            "status": "pending"
        }
        self.verify_items.append(item)
        
        self.txt_verify_log.append(f"[OK] Finished Item #{item_id} (contains {len(self.current_draw_points)} points).")
        
        self.current_draw_points.clear()
        self.current_draw_poses.clear()
        self.hover_pos = None
        
        if self.calib_data:
            self.btn_ver_move.setEnabled(True)

    def clear_draw_items(self):
        self.verify_items.clear()
        self.current_draw_points.clear()
        self.current_draw_poses.clear()
        self.lbl_coord_cam.setText("N/A")
        self.lbl_coord_base.setText("N/A")
        self.lbl_normal_base.setText("N/A")
        self.executor.stop()
        self.btn_ver_move.setText("Move Robot (Execute Route)")
        self.btn_ver_move.setEnabled(False)
        self.txt_verify_log.append("[*] Cleared all drawn points and lines.")

    def stop_verify_execution(self):
        self.executor.stop()
        self.btn_ver_move.setText("Move Robot (Execute Route)")
        self.btn_ver_move.setEnabled(True)
        self.progress_bar.setVisible(False)
        self.txt_verify_log.append("[!] Robot route execution stopped.")

    def update_realtime_pose_and_progress(self):
        if not self.robot_connected or not self.robot:
            self.lbl_real_x.setText("X: N/A")
            self.lbl_real_y.setText("Y: N/A")
            self.lbl_real_z.setText("Z: N/A")
            self.lbl_real_a.setText("A: N/A")
            self.lbl_real_b.setText("B: N/A")
            self.lbl_real_c.setText("C: N/A")
            return
            
        try:
            curr_pose = self.robot.get_current_pose()
            if not curr_pose:
                return
            
            pose_list = curr_pose.to_list()
            if any(v < -2500.0 for v in pose_list[:3]):
                return
                
            self.lbl_real_x.setText(f"X: {curr_pose.x:.2f}")
            self.lbl_real_y.setText(f"Y: {curr_pose.y:.2f}")
            self.lbl_real_z.setText(f"Z: {curr_pose.z:.2f}")
            self.lbl_real_a.setText(f"A: {curr_pose.a:.4f}")
            self.lbl_real_b.setText(f"B: {curr_pose.b:.4f}")
            self.lbl_real_c.setText(f"C: {curr_pose.c:.4f}")
            
            if self.executor.is_moving():
                if self.executor.total_path_distance > 0:
                    percent, d_completed = self.executor.get_progress_percent(curr_pose)
                    self.progress_bar.setValue(percent)
                    self.progress_bar.setFormat(f"Progress: %p% ({int(d_completed)}/{int(self.executor.total_path_distance)} mm)")
        except Exception:
            pass

    def execute_next_verify_step(self):
        if time.time() - self.executor.last_move_cmd_time < 0.8:
            return

        if self.executor.exec_item_index >= len(self.verify_items):
            self.executor.stop()
            self.btn_ver_move.setText("Move Robot (Execute Route)")
            self.btn_ver_move.setEnabled(True)
            self.progress_bar.setValue(100)
            self.progress_bar.setVisible(False)
            self.txt_verify_log.append("[OK] All verify items executed successfully!")
            return

        current_item = self.verify_items[self.executor.exec_item_index]
        poses = current_item["pose_data"]

        if self.executor.exec_waypoint_index >= len(poses):
            current_item["status"] = "visited"
            self.txt_verify_log.append(f"[OK] Item #{current_item['id']} finished.")
            self.executor.exec_item_index += 1
            self.executor.exec_waypoint_index = 0
            self.execute_next_verify_step()
            return

        pose_info = poses[self.executor.exec_waypoint_index]
        offset = self.spin_ver_offset.value()
        p_dest = pose_info["p_base"] + pose_info["n_base"] * offset
        a = pose_info["a"]
        b = pose_info["b"]
        c = pose_info["c"]

        target_pose = RobotPose(p_dest[0], p_dest[1], p_dest[2], a, b, c)

        ok, err_msg = self.executor.check_safety_limit(p_dest)
        if not ok:
            msg = f"Item #{current_item['id']} Pt #{self.executor.exec_waypoint_index + 1} {err_msg}"
            QMessageBox.warning(self, "Safety Limit", msg)
            self.txt_verify_log.append(f"  [!] Safety Limit: {msg}")
            self.stop_verify_execution()
            return

        if self.executor.exec_waypoint_index == 0:
            move_mode = "MOVJ"
        else:
            move_mode = "MOVL"

        if not self.robot.is_reachable(target_pose, move_mode):
            self.txt_verify_log.append(f"  [*] Target pose unreachable ({move_mode}), trying to adjust roll to find a reachable pose...")
            adjusted_pose = self.try_adjust_unreachable_pose(target_pose, pose_info, move_mode)
            if adjusted_pose is not None:
                target_pose = adjusted_pose
                a = target_pose.a
                b = target_pose.b
                c = target_pose.c
            else:
                msg = f"Item #{current_item['id']} Pt #{self.executor.exec_waypoint_index + 1} target pose is kinematics-unreachable ({move_mode}) even after roll adjustment."
                QMessageBox.warning(self, "Unreachable", msg)
                self.txt_verify_log.append(f"  [!] Unreachable: {msg}")
                self.stop_verify_execution()
                return

        acc = float(self.slider_acc.value())
        dec = float(self.slider_dec.value())
        tool_num = int(self.spin_tool_num.value())

        if move_mode == "MOVJ":
            self.txt_verify_log.append(f"[*] Moving ({move_mode}) to Item #{current_item['id']} Pt #{self.executor.exec_waypoint_index + 1}: X={p_dest[0]:.1f}, Y={p_dest[1]:.1f}, Z={p_dest[2]:.1f}, A={a:.4f}, B={b:.4f}, C={c:.4f} (Acc: {acc:.0f}%, Dec: {dec:.0f}%, Tool: {tool_num})")
            self.robot.move_j(target_pose, acc=acc, dec=dec, tool_num=tool_num, wait=False)
        else:
            movl_speed = float(self.slider_movl_speed.value())
            self.txt_verify_log.append(f"[*] Moving ({move_mode}) to Item #{current_item['id']} Pt #{self.executor.exec_waypoint_index + 1}: X={p_dest[0]:.1f}, Y={p_dest[1]:.1f}, Z={p_dest[2]:.1f}, A={a:.4f}, B={b:.4f}, C={c:.4f} (Speed: {movl_speed:.1f} mm/s, Acc: {acc:.0f}%, Dec: {dec:.0f}%, Tool: {tool_num})")
            self.robot.move_l(target_pose, velocity=movl_speed, acc=acc, dec=dec, tool_num=tool_num, wait=False)
        self.executor.last_move_cmd_time = time.time()

        self.executor.exec_waypoint_index += 1

    def try_adjust_unreachable_pose(self, target_pose, pose_info, move_mode):
        """
        当目标位姿在运行中不可达时，尝试通过调整工具绕 Z 轴的滚转角（Roll）来寻找一个可达的姿态。
        在保持 Z 轴（即喷涂法线方向）完全对齐的前提下，在 [-45°, 45°] 范围内以 5° 为步长搜索。
        """
        if "n_base" not in pose_info:
            return None

        n_base = pose_info["n_base"]
        p_dest = np.array([target_pose.x, target_pose.y, target_pose.z])

        opt_cfg = self.calib_data.get("metadata", {}).get("optimization_config", {})
        order = opt_cfg.get("axis_order", "ZYX")
        s_vec = opt_cfg.get("sign_vector", [1, 1, 1])

        # 优先搜索接近 0 的小旋转偏置
        search_angles_deg = []
        for angle in range(5, 46, 5):
            search_angles_deg.append(float(angle))
            search_angles_deg.append(float(-angle))

        for deg in search_angles_deg:
            roll_rad = np.radians(deg)
            a_new, b_new, c_new = calculate_tool_orientation_with_roll(n_base, roll_rad, order, s_vec)
            candidate_pose = RobotPose(p_dest[0], p_dest[1], p_dest[2], a_new, b_new, c_new)
            
            if self.robot.is_reachable(candidate_pose, move_mode):
                self.txt_verify_log.append(f"  [+] Found reachable pose by rotating roll {deg}° around tool Z-axis.")
                return candidate_pose

        return None


    def handle_verify_click(self, pos):
        """
        处理验证页面视频预览区域的鼠标点击事件。
        算法与变换步骤：
          1. 坐标系还原 (Label -> Native Pixel)：
             由于视频被缩放居中显示在 QLabel 中，点击得到的坐标 `pos` 带有偏置和缩放。
             本算法逆向推导，计算偏移量 `ox, oy` 以及缩放比 `scale`，将 QLabel 像素坐标转换为相机原始画面分辨率坐标 `(u, v)`。
          2. 获取稳健深度 (Pixel -> Depth)：
             调用 `get_robust_depth` 检索 `(u, v)` 处深度值 `z_val`，若深度无效（如棋盘格阴影区）则返回。
          3. 图像坐标系转相机三维坐标系 (2D -> 3D Camera Frame)：
             使用相机内参 K：
               X_cam = (u - cx) * Z / fx
               Y_cam = (v - cy) * Z / fy
               Z_cam = Z (即 z_val)
             由此计算出该点在相机系下的物理坐标 `p_cam`。
          4. 相机坐标系转机器人基座坐标系 (Camera Frame -> Base Frame)：
             读取已加载的手眼外参矩阵 `T_base_camera`（包含旋转 `R_bc` 与平移 `t_bc`）：
               P_base = R_bc @ P_cam + t_bc
             得到喷涂面上的物理目标位置。
          5. 估算平面法向量并计算对齐欧拉角 (Normal & TCP Orientation)：
             - 调用 `compute_local_normal` 在 `(u, v)` 的 5 像素十字邻域估算相机系下的局部法向量 `n_cam`。
             - 将法向量旋转至机器人基准坐标系：`n_base = R_bc @ n_cam`。
             - 根据外参指定的欧拉角顺规和极性符号，调用 `calculate_tool_orientation` 计算喷头垂直该平面所需的欧拉角 A, B, C。
          6. 数据封装：
             将点位信息 `(u, v)` 及物理姿态数据打包追加到当前正在绘制的折线轨迹点集内。
        """
        if not self.calib_data:
            QMessageBox.warning(self, "Warning", "Please load a calibration result file first.")
            return
        if self.current_depth is None:
            QMessageBox.warning(self, "Warning", "No depth stream available.")
            return
        if self.current_color is None:
            return

        # 1. 还原 QLabel 缩放及偏置，映射到相机原生像素分辨率 (u, v)
        H_native, W_native = self.current_color.shape[:2]

        lw = self.ver_video.width()
        lh = self.ver_video.height()

        scale = min(lw / W_native, lh / H_native)
        sw = W_native * scale
        sh = H_native * scale

        ox = (lw - sw) / 2
        oy = (lh - sh) / 2

        click_x = pos.x() - ox
        click_y = pos.y() - oy

        u = int(click_x / scale)
        v = int(click_y / scale)

        if not (0 <= u < W_native and 0 <= v < H_native):
            return

        # 2. 检索鲁棒深度
        z_val = get_robust_depth(self.current_depth, u, v)
        if z_val <= 0:
            self.lbl_coord_cam.setText("Invalid Depth (0)")
            self.lbl_coord_base.setText("N/A")
            self.lbl_normal_base.setText("N/A")
            self.txt_verify_log.append(f"  [!] Invalid depth value at pixel coordinate U={u}, V={v}")
            return

        # 3. 图像系转相机三维物理系
        K = np.array(self.calib_data["camera_params"]["intrinsic_matrix"])
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        x_val = (u - cx) * z_val / fx
        y_val = (v - cy) * z_val / fy
        p_cam = np.array([x_val, y_val, z_val])

        # 4. 相机系转机器人基座系
        T_bc = np.array(self.calib_data["T_base_camera"])
        R_bc = T_bc[:3, :3]
        t_bc = T_bc[:3, 3]

        p_base = R_bc @ p_cam + t_bc

        # 5. 计算法线与喷枪对齐朝向
        n_cam = compute_local_normal(self.current_depth, u, v, K)
        if n_cam is not None:
            n_base = R_bc @ n_cam
            opt_cfg = self.calib_data.get("metadata", {}).get("optimization_config", {})
            order = opt_cfg.get("axis_order", "ZYX")
            s_vec = opt_cfg.get("sign_vector", [1, 1, 1])

            try:
                a, b, c = calculate_tool_orientation(n_base, order, s_vec)

                pose_data = {
                    "p_base": p_base,
                    "n_base": n_base,
                    "n_cam": n_cam,
                    "a": a, "b": b, "c": c
                }
                
                self.current_draw_points.append((u, v))
                self.current_draw_poses.append(pose_data)
                
                self.lbl_coord_cam.setText(f"X:{x_val:.1f} Y:{y_val:.1f} Z:{z_val:.1f}")
                self.lbl_coord_base.setText(f"X:{p_base[0]:.1f} Y:{p_base[1]:.1f} Z:{p_base[2]:.1f}")
                self.lbl_normal_base.setText(f"X:{n_base[0]:.2f} Y:{n_base[1]:.2f} Z:{n_base[2]:.2f}")
                
                self.txt_verify_log.append(f"[*] Added point to current segment: U={u}, V={v}. Base: X={p_base[0]:.1f}, Y={p_base[1]:.1f}, Z={p_base[2]:.1f}, A={a:.4f}, B={b:.4f}, C={c:.4f}")
            except Exception as e:
                self.txt_verify_log.append(f"  [!] Euler translation failed: {e}")
        else:
            self.lbl_normal_base.setText("Normal estimation failed")
            self.txt_verify_log.append(f"  [!] Failed to estimate surface normal vector")

    def handle_verify_hover(self, pos):
        """
        处理验证页面视频预览区域的鼠标移动事件。
        如果当前正在绘制折线段（即 `self.current_draw_points` 不为空），
        则将鼠标在 QLabel 上的当前坐标转换为相机原生像素坐标，用于绘制虚拟引导线与虚拟法向量。
        """
        if not self.current_draw_points or self.current_color is None:
            self.hover_pos = None
            return

        H_native, W_native = self.current_color.shape[:2]

        lw = self.ver_video.width()
        lh = self.ver_video.height()

        scale = min(lw / W_native, lh / H_native)
        sw = W_native * scale
        sh = H_native * scale

        ox = (lw - sw) / 2
        oy = (lh - sh) / 2

        click_x = pos.x() - ox
        click_y = pos.y() - oy

        u = int(click_x / scale)
        v = int(click_y / scale)

        if 0 <= u < W_native and 0 <= v < H_native:
            self.hover_pos = (u, v)
        else:
            self.hover_pos = None

    def move_to_verification_pose(self):
        if not self.robot_connected or not self.robot:
            QMessageBox.warning(self, "Warning", "Robot is not connected. Connect the robot first.")
            return

        if self.current_draw_points:
            self.finish_current_draw_item()

        if not self.verify_items:
            QMessageBox.warning(self, "Warning", "No routes or points defined to move.")
            return

        self.executor.robot = self.robot
        curr_pose = self.robot.get_current_pose()
        offset = self.spin_ver_offset.value()
        self.executor.start(self.verify_items, offset, curr_pose)

        self.btn_ver_move.setEnabled(False)
        self.btn_ver_move.setText("Moving...")
        self.progress_bar.setValue(0)
        if self.executor.total_path_distance > 0:
            self.progress_bar.setFormat(f"Progress: 0% (0/{int(self.executor.total_path_distance)} mm)")
        else:
            self.progress_bar.setFormat("Progress: 100%")
        self.progress_bar.setVisible(True)
        self.txt_verify_log.append(f"[*] Starting robot route execution (Total Distance: {self.executor.total_path_distance:.1f} mm)...")
        
        self.execute_next_verify_step()

    def move_to_safe_pose(self):
        if not self.robot_connected or not self.robot:
            QMessageBox.warning(self, "Warning", "Robot is not connected. Connect the robot first.")
            return

        x = self.spin_safe_x.value()
        y = self.spin_safe_y.value()
        z = self.spin_safe_z.value()
        a = np.radians(self.spin_safe_a.value())
        b = np.radians(self.spin_safe_b.value())
        c = np.radians(self.spin_safe_c.value())

        target_pose = RobotPose(x, y, z, a, b, c)

        planner_cfg = self.config.get("vision", {}).get("planner", {})
        lim = planner_cfg.get("workspace_limits", {})
        if lim:
            if not (lim.get("x", [-9999, 9999])[0] <= x <= lim.get("x", [-9999, 9999])[1]):
                QMessageBox.warning(self, "Out of Workspace", f"Safe Pose Target X ({x}) exceeds safety limits.")
                return
            if not (lim.get("y", [-9999, 9999])[0] <= y <= lim.get("y", [-9999, 9999])[1]):
                QMessageBox.warning(self, "Out of Workspace", f"Safe Pose Target Y ({y}) exceeds safety limits.")
                return
            if not (lim.get("z", [-9999, 9999])[0] <= z <= lim.get("z", [-9999, 9999])[1]):
                QMessageBox.warning(self, "Out of Workspace", f"Safe Pose Target Z ({z}) exceeds safety limits.")
                return

        if not self.robot.is_reachable(target_pose, "MOVJ"):
            QMessageBox.warning(self, "Unreachable", "The target safe pose is kinematics-unreachable (MOVJ).")
            return

        acc = float(self.slider_acc.value())
        dec = float(self.slider_dec.value())
        tool_num = int(self.spin_tool_num.value())

        self.txt_verify_log.append(f"[*] Moving robot to safe position: X:{x:.2f} Y:{y:.2f} Z:{z:.2f} A:{self.spin_safe_a.value():.2f} B:{self.spin_safe_b.value():.2f} C:{self.spin_safe_c.value():.2f} (Acc: {acc:.0f}%, Dec: {dec:.0f}%, Tool: {tool_num})")
        self.robot.move_j(target_pose, acc=acc, dec=dec, tool_num=tool_num)
        self.read_robot_pose()

        actual_pose = self.robot.get_current_pose()
        if actual_pose is not None:
            self.txt_verify_log.append(f"[OK] Reached safe position: X:{actual_pose.x:.2f} Y:{actual_pose.y:.2f} Z:{actual_pose.z:.2f} A:{np.degrees(actual_pose.a):.2f} B:{np.degrees(actual_pose.b):.2f} C:{np.degrees(actual_pose.c):.2f}")
        else:
            self.txt_verify_log.append("[-] Failed to query actual reached pose.")

    def change_robot_tool_number(self, tool_num):
        if not self.robot_connected or not self.robot:
            return
        try:
            self.robot.set_tool_number(tool_num)
            self.txt_verify_log.append(f"[*] Set robot tool number to {tool_num}")
        except Exception as e:
            self.txt_verify_log.append(f"[-] Failed to set robot tool number: {e}")

    def change_robot_speed(self, speed_val):
        if not self.robot_connected or not self.robot:
            return
        try:
            self.robot.set_global_speed(speed_val)
            self.txt_verify_log.append(f"[*] Set robot global speed to {speed_val}%")
        except Exception as e:
            self.txt_verify_log.append(f"[-] Failed to set robot global speed: {e}")

    def robot_go_home(self):
        if not self.robot_connected or not self.robot:
            QMessageBox.warning(self, "Warning", "Robot is not connected. Connect the robot first.")
            return

        try:
            self.txt_verify_log.append("[*] Moving robot to home position...")
            ret = self.robot.go_home()
            if ret == 0:
                self.txt_verify_log.append("[OK] Go Home command sent successfully.")
            else:
                self.txt_verify_log.append(f"[-] Go Home command failed with code: {ret}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to move to home position: {e}")

    def closeEvent(self, event):
        self.disconnect_robot()
        if self.cam:
            try: self.cam.stop()
            except: pass
        event.accept()
