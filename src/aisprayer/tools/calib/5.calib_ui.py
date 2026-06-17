#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys

# Fix PyQt5 conflict with OpenCV's built-in Qt plugins
try:
    import cv2  # Must import cv2 first because it overwrites QT_QPA_PLATFORM_PLUGIN_PATH
    import PyQt5
    pyqt5_dir = os.path.dirname(PyQt5.__file__)
    plugins_dir = os.path.join(pyqt5_dir, "Qt5", "plugins")
    if os.path.exists(plugins_dir):
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = plugins_dir
    else:
        plugins_dir_alt = os.path.join(pyqt5_dir, "Qt", "plugins")
        if os.path.exists(plugins_dir_alt):
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = plugins_dir_alt
except Exception as e:
    pass

import time
import numpy as np
import yaml
import glob
from datetime import datetime
from scipy.spatial.transform import Rotation as R_tool

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QTabWidget, QLabel, QPushButton, 
                             QLineEdit, QFileDialog, QMessageBox, QGroupBox, 
                             QFormLayout, QDoubleSpinBox, QTextEdit, QGridLayout)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPoint
from PyQt5.QtGui import QImage, QPixmap

# Setup paths
if getattr(sys, 'frozen', False):
    PROJECT_ROOT = os.path.dirname(sys.executable)
else:
    PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
    sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.hardware.robot.inexbot_driver import InexbotDriver, RobotPose
from aisprayer.core.hardware.camera.factory import get_camera
from aisprayer.utils.config_helper import load_config, get_abs_path
from aisprayer.utils.hardware_helper import verify_hardware_consistency

class ClickableLabel(QLabel):
    """A QLabel that emits custom clicked signal with relative coordinates."""
    clicked_pos = pyqtSignal(QPoint)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 480)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #111; border: 1px solid #333;")

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked_pos.emit(event.pos())

class CalibUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AI Sprayer - Robotic Calibration GUI")
        self.resize(1400, 900)

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

        # Verify state
        self.calib_data = None
        self.target_uv = None
        self.target_n_cam = None
        self.target_pose_data = None

        # Initialize user interface
        self.init_ui()

        # Connect Camera at startup
        self.start_camera()

        # Frame update timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        self.last_heartbeat_time = 0

    def init_ui(self):
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.init_capture_calib_tab()
        self.init_verify_tab()

    def init_capture_calib_tab(self):
        tab = QWidget()
        main_layout = QHBoxLayout(tab)

        # Left Column: Camera View
        left_layout = QVBoxLayout()
        self.cap_video = QLabel()
        self.cap_video.setMinimumSize(640, 480)
        self.cap_video.setAlignment(Qt.AlignCenter)
        self.cap_video.setStyleSheet("background-color: #000; border: 2px solid #222;")
        left_layout.addWidget(self.cap_video)

        # Status Line
        status_layout = QHBoxLayout()
        self.lbl_cam_status = QLabel("Camera: Disconnected")
        self.lbl_cam_status.setStyleSheet("font-size: 12px; font-weight: bold; color: #f44336;")
        self.btn_reconnect_cam = QPushButton("Retry Camera")
        self.btn_reconnect_cam.setFixedWidth(100)
        self.btn_reconnect_cam.clicked.connect(self.start_camera)

        status_layout.addWidget(self.lbl_cam_status)
        status_layout.addWidget(self.btn_reconnect_cam)
        status_layout.addStretch()
        
        left_layout.addLayout(status_layout)
        main_layout.addLayout(left_layout, 3)

        # Right Column: Controls Splitter
        right_layout = QVBoxLayout()

        # 1. Connection Group
        conn_group = QGroupBox("Robot Connection")
        conn_layout = QHBoxLayout(conn_group)
        conn_layout.setSpacing(5)
        self.txt_ip = QLineEdit(self.default_ip)
        self.txt_ip.setFixedWidth(90)
        self.txt_port = QLineEdit(self.default_port)
        self.txt_port.setFixedWidth(40)
        self.btn_connect_robot = QPushButton("Connect Robot")
        #self.btn_connect_robot.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        self.btn_connect_robot.clicked.connect(self.toggle_robot_connection)
        
        self.lbl_robot_status = QLabel("Robot: Disconnected")
        self.lbl_robot_status.setStyleSheet("font-size: 12px; font-weight: bold; color: #f44336;")

        conn_layout.addWidget(QLabel("IP:"))
        conn_layout.addWidget(self.txt_ip)
        conn_layout.addSpacing(8)
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.txt_port)
        conn_layout.addSpacing(30)
        conn_layout.addWidget(self.btn_connect_robot)
        conn_layout.addSpacing(10)
        conn_layout.addWidget(self.lbl_robot_status)
        conn_layout.addStretch()
        right_layout.addWidget(conn_group)

        # 2. Capture & Calibration Group
        cap_group = QGroupBox("Calibration Capture & Process")
        cap_vbox = QVBoxLayout(cap_group)

        # Select Dir Row
        dir_layout = QHBoxLayout()
        self.btn_sel_dir = QPushButton("Select Calib Dir")
        self.btn_sel_dir.clicked.connect(self.select_save_dir)
        self.lbl_save_dir = QLabel("Not Selected")
        self.lbl_save_dir.setWordWrap(True)
        dir_layout.addWidget(self.btn_sel_dir)
        dir_layout.addWidget(self.lbl_save_dir, 1)
        cap_vbox.addLayout(dir_layout)

        # Control buttons
        self.lbl_samples_count = QLabel("Samples: 0")
        self.lbl_samples_count.setStyleSheet("font-weight: bold;")
        cap_vbox.addWidget(self.lbl_samples_count)

        btn_row = QHBoxLayout()
        self.btn_capture = QPushButton("Capture")
        #self.btn_capture.setMinimumHeight(40)
        #self.btn_capture.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.btn_capture.clicked.connect(self.capture_sample)

        self.btn_run_calib = QPushButton("Calibrate")
        #self.btn_run_calib.setMinimumHeight(40)
        #self.btn_run_calib.setStyleSheet("background-color: #E91E63; color: white; font-weight: bold;")
        self.btn_run_calib.clicked.connect(self.run_calibration)

        btn_row.addWidget(self.btn_capture)
        btn_row.addWidget(self.btn_run_calib)
        cap_vbox.addLayout(btn_row)

        self.txt_calib_log = QTextEdit()
        self.txt_calib_log.setReadOnly(True)
        self.txt_calib_log.setLineWrapMode(QTextEdit.NoWrap)
        # Patch append to prepend datetime and keep horizontal scrollbar at the left
        _orig_append = self.txt_calib_log.append
        def log_append(text):
            _orig_append(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {text}")
            self.txt_calib_log.horizontalScrollBar().setValue(0)
        self.txt_calib_log.append = log_append
        #self.txt_calib_log.setStyleSheet("background-color: #1e1e1e; color: #a9b7c6; font-family: monospace;")
        cap_vbox.addWidget(self.txt_calib_log)

        right_layout.addWidget(cap_group)

        # 3. Jogging Group
        jog_group = QGroupBox("Robot Jogging (XYZ in mm, ABC in deg)")
        jog_grid = QGridLayout(jog_group)

        self.jog_inputs = {}
        axes = ['X', 'Y', 'Z', 'A', 'B', 'C']
        
        # Ranges: XYZ in mm [-2500, 2500], ABC in degrees [-360, 360]
        for idx, axis in enumerate(axes):
            lbl = QLabel(f"{axis}:")
            spin = QDoubleSpinBox()
            if axis in ['X', 'Y', 'Z']:
                spin.setRange(-2500.0, 2500.0)
                spin.setValue(0.0)
                spin.setSuffix(" mm")
            else:
                spin.setRange(-360.0, 360.0)
                spin.setValue(0.0)
                spin.setSuffix(" °")
            
            spin.setDecimals(2)
            self.jog_inputs[axis] = spin

            btn_minus = QPushButton("-")
            btn_plus = QPushButton("+")
            
            #btn_minus.setFixedWidth(40)
            #btn_plus.setFixedWidth(40)
            
            btn_minus.clicked.connect(lambda checked, a=axis: self.jog_step(a, -1))
            btn_plus.clicked.connect(lambda checked, a=axis: self.jog_step(a, 1))

            jog_grid.addWidget(lbl, idx, 0)
            jog_grid.addWidget(spin, idx, 1)
            jog_grid.addWidget(btn_minus, idx, 2)
            jog_grid.addWidget(btn_plus, idx, 3)

        # Step size selectors
        step_layout = QHBoxLayout()
        step_layout.addWidget(QLabel("Step XYZ (mm):"))
        self.spin_step_xyz = QDoubleSpinBox()
        self.spin_step_xyz.setRange(0.1, 200.0)
        self.spin_step_xyz.setValue(10.0)
        step_layout.addWidget(self.spin_step_xyz)

        step_layout.addWidget(QLabel("Step ABC (°):"))
        self.spin_step_abc = QDoubleSpinBox()
        self.spin_step_abc.setRange(0.1, 45.0)
        self.spin_step_abc.setValue(5.0)
        step_layout.addWidget(self.spin_step_abc)
        jog_grid.addLayout(step_layout, 6, 0, 1, 4)

        # Execute & Read buttons
        exec_layout = QHBoxLayout()
        self.btn_read_pos = QPushButton("Read Current Pose")
        self.btn_read_pos.clicked.connect(self.read_robot_pose)
        self.btn_move_to_jog = QPushButton("Move to Target Pose")
        #self.btn_move_to_jog.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
        self.btn_move_to_jog.clicked.connect(self.move_to_jog_pose)

        exec_layout.addWidget(self.btn_read_pos)
        exec_layout.addWidget(self.btn_move_to_jog)
        jog_grid.addLayout(exec_layout, 7, 0, 1, 4)

        right_layout.addWidget(jog_group)
        main_layout.addLayout(right_layout, 2)

        tab.setLayout(main_layout)
        self.tabs.addTab(tab, "Calibrate")

    def init_verify_tab(self):
        tab = QWidget()
        main_layout = QHBoxLayout(tab)

        # Left Column: Clickable Video View
        left_layout = QVBoxLayout()
        self.ver_video = ClickableLabel()
        self.ver_video.clicked_pos.connect(self.handle_verify_click)
        left_layout.addWidget(self.ver_video)
        
        self.lbl_ver_tip = QLabel("Instruction: Load a calibration file, click anywhere on the image to compute target pose.")
        self.lbl_ver_tip.setStyleSheet("color: #bbb; italic: true;")
        left_layout.addWidget(self.lbl_ver_tip)
        main_layout.addLayout(left_layout, 3)

        # Right Column: Config & Action
        right_layout = QVBoxLayout()

        # Load file
        file_group = QGroupBox("Load Calibration Matrix")
        file_vbox = QVBoxLayout(file_group)
        self.lbl_calib_path = QLabel("Calibration File: Not Loaded")
        self.lbl_calib_path.setWordWrap(True)
        self.btn_load_calib = QPushButton("Load Calibration Result (.yaml)")
        self.btn_load_calib.clicked.connect(self.select_calib_file)
        file_vbox.addWidget(self.btn_load_calib)
        file_vbox.addWidget(self.lbl_calib_path)
        file_group.setLayout(file_vbox)
        right_layout.addWidget(file_group)

        # Coordinates info
        coord_group = QGroupBox("Target Coords & Surface Normal")
        coord_form = QFormLayout(coord_group)
        self.lbl_coord_cam = QLabel("N/A")
        self.lbl_coord_base = QLabel("N/A")
        self.lbl_normal_base = QLabel("N/A")
        coord_form.addRow("Cam (XYZ mm):", self.lbl_coord_cam)
        coord_form.addRow("Base (XYZ mm):", self.lbl_coord_base)
        coord_form.addRow("Normal (Base):", self.lbl_normal_base)
        coord_group.setLayout(coord_form)
        right_layout.addWidget(coord_group)

        # Action panel
        action_group = QGroupBox("Verification Movement")
        action_vbox = QVBoxLayout(action_group)
        
        offset_layout = QHBoxLayout()
        offset_layout.addWidget(QLabel("Offset (Z-axis normal distance, mm):"))
        self.spin_ver_offset = QDoubleSpinBox()
        self.spin_ver_offset.setRange(-500.0, 500.0)
        self.spin_ver_offset.setValue(100.0)
        offset_layout.addWidget(self.spin_ver_offset)
        action_vbox.addLayout(offset_layout)

        self.btn_ver_move = QPushButton("Move Robot (Perpendicular to Surface)")
        self.btn_ver_move.setEnabled(False)
        self.btn_ver_move.setMinimumHeight(50)
        self.btn_ver_move.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold; font-size: 14px;")
        self.btn_ver_move.clicked.connect(self.move_to_verification_pose)
        action_vbox.addWidget(self.btn_ver_move)
        action_group.setLayout(action_vbox)
        
        right_layout.addWidget(action_group)
        right_layout.addStretch()
        main_layout.addLayout(right_layout, 1)

        tab.setLayout(main_layout)
        self.tabs.addTab(tab, "Verify")

        # Load default result file if exists
        p_cfg = self.config.get("vision", {}).get("planner", {})
        default_calib = get_abs_path(p_cfg.get("calib_path", "configs/calib/calibration_result.yaml"), PROJECT_ROOT)
        if os.path.exists(default_calib):
            self.load_calib_file(default_calib)

    def start_camera(self):
        try:
            self.lbl_cam_status.setText("Camera: Connecting...")
            self.lbl_cam_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #FF9800;")
            QApplication.processEvents()

            self.cam = get_camera(self.camera_model)
            self.cam.start()
            self.cam_connected = True
            
            self.lbl_cam_status.setText("Camera: Connected")
            self.lbl_cam_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #4CAF50;")
            
            # Setup default output dir automatically if not chosen
            if not self.save_dir:
                ts = datetime.now().strftime("%Y%m%d")
                self.setup_save_dir(os.path.join(self.default_output_dir, f"calib_{ts}"))
        except Exception as e:
            self.cam_connected = False
            self.lbl_cam_status.setText("Camera: Offline")
            self.lbl_cam_status.setStyleSheet("font-size: 12px; font-weight: bold; color: #f44336;")
            print(f"Camera start failed: {e}")

    def toggle_robot_connection(self):
        if not self.robot_connected:
            ip = self.txt_ip.text().strip()
            port = self.txt_port.text().strip()

            self.lbl_robot_status.setText("Robot: Connecting...")
            self.lbl_robot_status.setStyleSheet("font-size: 12px; font-weight: bold; color: #FF9800;")
            QApplication.processEvents()

            try:
                self.robot = InexbotDriver(ip=ip, port=port)
                if not self.robot.startup(timeout=5.0):
                    QMessageBox.critical(self, "Error", "Robot startup failed! Make sure controller IP/Port are correct.")
                    self.robot = None
                    self.lbl_robot_status.setText("Robot: Disconnected")
                    self.lbl_robot_status.setStyleSheet("font-size: 12px; font-weight: bold; color: #f44336;")
                    return

                self.robot_connected = True
                self.btn_connect_robot.setText("Disconnect Robot")
                self.btn_connect_robot.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
                
                self.lbl_robot_status.setText("Robot: Connected")
                self.lbl_robot_status.setStyleSheet("font-size: 12px; font-weight: bold; color: #4CAF50;")

                # Read current pose to populate jogging GUI
                self.read_robot_pose()

            except Exception as e:
                self.robot = None
                self.lbl_robot_status.setText("Robot: Disconnected")
                self.lbl_robot_status.setStyleSheet("font-size: 12px; font-weight: bold; color: #f44336;")
                QMessageBox.critical(self, "Error", f"Failed to connect to robot: {e}")
        else:
            self.disconnect_robot()

    def disconnect_robot(self):
        if self.robot:
            try: self.robot.shutdown()
            except: pass
            self.robot = None
        self.robot_connected = False
        self.btn_connect_robot.setText("Connect Robot")
        self.btn_connect_robot.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        self.lbl_robot_status.setText("Robot: Disconnected")
        self.lbl_robot_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #f44336;")

    def select_save_dir(self):
        d = QFileDialog.getExistingDirectory(self, "Select Save Directory", PROJECT_ROOT)
        if d:
            # If the selected directory already contains calibration data, use it directly.
            # Otherwise, append a calib_YYYYMMDD subdirectory.
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
        
        # Load or initialize yaml configuration
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r', encoding='utf-8') as f:
                self.cap_info = yaml.safe_load(f)
            # Verify hardware compatibility
            if self.cam_connected and self.cam:
                ok, msg = verify_hardware_consistency(live=self.cam, scan=self.cap_info.get("camera_params", {}))
                if not ok:
                    self.txt_calib_log.append(f"[Warning] Camera params mismatch: {msg}")
            self.txt_calib_log.append(f"Loaded {len(self.cap_info.get('samples', []))} samples from {self.yaml_path}")
        else:
            # Create new structure
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

        self.lbl_samples_count.setText(f"Samples: {len(self.cap_info.get('samples', []))}")

    def update_frame(self):
        # Keepalive querying to prevent connection dropouts
        if self.robot_connected and self.robot:
            if time.time() - self.last_heartbeat_time > 2.0:
                try:
                    self.robot.get_running_state()
                except:
                    pass
                self.last_heartbeat_time = time.time()

        if not self.cam_connected or not self.cam:
            # Render empty frame placeholder
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

        active_tab = self.tabs.currentIndex()
        if active_tab == 0:
            # Display color frame and corners
            display_frame = color.copy()
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
            if ret:
                cv2.drawChessboardCorners(display_frame, self.pattern_size, corners, ret)
            self.render_image_to_label(display_frame, self.cap_video)

        elif active_tab == 1:
            # Display verification screen
            display_frame = color.copy()
            if self.target_uv and self.calib_data:
                u, v = self.target_uv
                cv2.drawMarker(display_frame, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

                # Draw projected 3D normal vector
                if self.target_n_cam is not None:
                    K = np.array(self.calib_data["camera_params"]["intrinsic_matrix"])
                    fx, fy = K[0, 0], K[1, 1]
                    cx, cy = K[0, 2], K[1, 2]

                    z_val = float(self.current_depth[v, u]) if self.current_depth is not None else 0.0
                    if z_val > 0:
                        x_val = (u - cx) * z_val / fx
                        y_val = (v - cy) * z_val / fy
                        
                        p_cam_origin = np.array([x_val, y_val, z_val])
                        p_cam_normal_tip = p_cam_origin + self.target_n_cam * 50.0  # 50 mm normal arrow
                        
                        # Project normal tip back to screen
                        if p_cam_normal_tip[2] > 0:
                            u_tip = int(fx * p_cam_normal_tip[0] / p_cam_normal_tip[2] + cx)
                            v_tip = int(fy * p_cam_normal_tip[1] / p_cam_normal_tip[2] + cy)
                            cv2.arrowedLine(display_frame, (u, v), (u_tip, v_tip), (0, 0, 255), 3, tipLength=0.3)

            self.render_image_to_label(display_frame, self.ver_video)

    def render_image_to_label(self, frame, label):
        h, w, c = frame.shape
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

        # Check for invalid SDK values
        pose_list = pose.to_list()
        if any(v < -2500.0 for v in pose_list[:3]):
            QMessageBox.warning(self, "Error", "Invalid pose data obtained from robot SDK.")
            return

        count = len(self.cap_info["samples"]) + 1
        img_name = f"image_{count:03d}.png"
        img_path = os.path.join(self.save_dir, img_name)

        # Save photo
        cv2.imwrite(img_path, self.current_color)

        # Add sample metadata
        sample = {
            "id": count,
            "image_file": img_name,
            "robot_pose": {
                "x": round(pose.x, 3), "y": round(pose.y, 3), "z": round(pose.z, 3),
                "a": round(pose.a, 5), "b": round(pose.b, 5), "c": round(pose.c, 5)
            }
        }
        self.cap_info["samples"].append(sample)

        # Save info yaml file
        with open(self.yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.cap_info, f, default_flow_style=False)

        self.lbl_samples_count.setText(f"Samples: {count}")
        self.txt_calib_log.append(f"[OK] Captured Sample {count}: {img_name}")
        self.txt_calib_log.append(f"     Pose: X:{pose.x:.1f} Y:{pose.y:.1f} Z:{pose.z:.1f} A:{pose.a:.4f} B:{pose.b:.4f} C:{pose.c:.4f}")

    def run_calibration(self):
        if not self.save_dir or not os.path.exists(self.yaml_path):
            QMessageBox.warning(self, "Warning", "Save directory does not contain calibration info!")
            return

        self.txt_calib_log.clear()
        self.txt_calib_log.append("====== Starting Calibration Process ======")

        try:
            with open(self.yaml_path, 'r', encoding='utf-8') as f:
                info = yaml.safe_load(f)

            if len(info.get("samples", [])) < 3:
                self.txt_calib_log.append("[-] Calibration requires at least 3 samples.")
                return

            K = np.array(info["camera_params"]["intrinsic_matrix"])
            D = np.array(info["camera_params"]["distortion_coeffs"])
            pattern_size = tuple(info["board_params"]["pattern_size_inner"])
            sq_size = info["board_params"]["square_size_mm"]

            # Initialize 3D points
            objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * sq_size

            all_samples = []
            self.txt_calib_log.append("[*] Extracting corners and solving PnP...")

            for s in info["samples"]:
                pose = s["robot_pose"]
                if any(abs(v) > 2500 for v in [pose['x'], pose['y'], pose['z']]):
                    self.txt_calib_log.append(f"  [!] Skipped sample {s['id']} (invalid position)")
                    continue

                img_path = os.path.join(self.save_dir, s["image_file"])
                img = cv2.imread(img_path)
                if img is None:
                    self.txt_calib_log.append(f"  [!] Skipped sample {s['id']} (image load failed)")
                    continue

                ret, corners = cv2.findChessboardCorners(img, pattern_size, None)
                if ret:
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
                    
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

            if len(all_samples) < 3:
                self.txt_calib_log.append("[-] Verification failed: insufficient valid chessboard corners found.")
                return

            # Clean data (displacement check)
            self.txt_calib_log.append("[*] Cleaning data based on motion displacement...")
            clean_thr = self.c_cfg.get("cleaning_threshold", 0.05)
            samples = self.clean_calibration_data(all_samples, threshold=clean_thr)

            if len(samples) < 3:
                self.txt_calib_log.append("[-] Calibration failed: not enough clean samples.")
                return

            # Evaluate diversity
            diversity = self.evaluate_data_diversity(samples)
            self.txt_calib_log.append(f"[*] Diversity score: {diversity['score']:.1f} / 100")

            # Optimization search
            self.txt_calib_log.append("[*] Searching for optimal camera extrinsics & chessboard offset...")
            best_res = self.optimize_extrinsics_solve(samples)
            if not best_res:
                self.txt_calib_log.append("[-] Optimization solver failed.")
                return

            R_bc, t_bc, t_off, err, order, s_vec = best_res
            r_err_mean = self.calculate_rotation_error(samples, best_res)

            self.txt_calib_log.append("\n====== Result ======")
            self.txt_calib_log.append(f"Euler Sequence: {order}, Signs: {s_vec}")
            self.txt_calib_log.append(f"Chessboard TCP Offset: {np.round(t_off, 2)} mm")
            self.txt_calib_log.append(f"Mean Reprojection Translation Error: {err:.3f} mm")
            self.txt_calib_log.append(f"Mean Angular Error: {r_err_mean:.3f}°")

            # Save full calibration output configuration
            T_bc = np.eye(4)
            T_bc[:3, :3] = R_bc
            T_bc[:3, 3] = t_bc
            xyz = t_bc.tolist()
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
                        "sign_vector": list(s_vec)
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

            # Load it into the verify tab automatically
            self.load_calib_file(calib_out_path)

        except Exception as e:
            self.txt_calib_log.append(f"[-] Calibration crashed: {e}")
            QMessageBox.critical(self, "Error", f"Calibration execution failed: {e}")

    def clean_calibration_data(self, all_samples, threshold=0.05):
        if not all_samples:
            return []
        base = all_samples[0]
        samples = [base]
        self.txt_calib_log.append(f"  [KEEP] Sample {base['id']}: Reference Base")
        for i in range(1, len(all_samples)):
            s = all_samples[i]
            p_base = np.array([base["pose"]["x"], base["pose"]["y"], base["pose"]["z"]])
            p_curr = np.array([s["pose"]["x"], s["pose"]["y"], s["pose"]["z"]])
            dist_r = np.linalg.norm(p_curr - p_base)
            dist_c = np.linalg.norm(s["t_cb"] - base["t_cb"])
            
            if dist_r < 10.0:
                samples.append(s)
                continue
                
            ratio = dist_c / dist_r
            if (1.0 - threshold) < ratio < (1.0 + threshold):
                samples.append(s)
                self.txt_calib_log.append(f"  [KEEP] Sample {s['id']}: Displacement ratio = {ratio:.3f}")
            else:
                # Check for rotation
                r_base = np.array([base["pose"]["a"], base["pose"]["b"], base["pose"]["c"]])
                r_curr = np.array([s["pose"]["a"], s["pose"]["b"], s["pose"]["c"]])
                rot_diff = np.linalg.norm(r_curr - r_base)
                
                if rot_diff > 0.05:  # Rotated > 3 deg
                    if 0.80 < ratio < 1.20:
                        samples.append(s)
                        self.txt_calib_log.append(f"  [KEEP*] Sample {s['id']}: Displacement ratio = {ratio:.3f} (orientation change detected)")
                    else:
                        self.txt_calib_log.append(f"  [DROP] Sample {s['id']}: Displacement ratio = {ratio:.3f} (excessive deviation)")
                else:
                    self.txt_calib_log.append(f"  [DROP] Sample {s['id']}: Displacement ratio = {ratio:.3f}")
        return samples

    def evaluate_data_diversity(self, samples):
        pos_x = [s["pose"]["x"] for s in samples]
        pos_y = [s["pose"]["y"] for s in samples]
        pos_z = [s["pose"]["z"] for s in samples]
        rot_a = [s["pose"]["a"] for s in samples]
        rot_b = [s["pose"]["b"] for s in samples]
        rot_c = [s["pose"]["c"] for s in samples]

        # Check if rotation is in radians
        is_radians = np.max(np.abs([rot_a, rot_b, rot_c])) < 7.0
        if is_radians:
            rot_a = np.degrees(rot_a)
            rot_b = np.degrees(rot_b)
            rot_c = np.degrees(rot_c)

        ptp_xyz = [np.ptp(pos_x), np.ptp(pos_y), np.ptp(pos_z)]
        ptp_abc = [np.ptp(rot_a), np.ptp(rot_b), np.ptp(rot_c)]

        p_score = min(1.0, np.mean(ptp_xyz) / 300.0)
        r_score = min(1.0, np.mean(ptp_abc) / 30.0)

        score = (p_score * 0.4 + r_score * 0.6) * 100.0
        return {"score": score}

    def optimize_extrinsics_solve(self, samples):
        best_err = float('inf')
        best_res = None
        
        test_orders = ['ZYX', 'XYZ', 'YXZ', 'YZX', 'ZXY', 'XZY', 'zyx', 'xyz', 'yxz', 'yzx', 'zxy', 'xzy']
        signs = [(1,1,1), (1,1,-1), (1,-1,1), (-1,1,1), (1,-1,-1), (-1,1,-1), (-1,-1,1), (-1,-1,-1)]

        for order in test_orders:
            for s_vec in signs:
                t_off = np.zeros(3)
                R_base_cam = np.eye(3)
                t_base_cam = np.zeros(3)
                
                P_robot = np.array([[s["pose"]["x"], s["pose"]["y"], s["pose"]["z"]] for s in samples])
                P_cam = np.array([s["t_cb"] for s in samples])
                
                try:
                    R_bt_list = []
                    for s in samples:
                        # pose euler angles are in radians
                        euler_angles = [s["pose"]['a'] * s_vec[0], s["pose"]['b'] * s_vec[1], s["pose"]['c'] * s_vec[2]]
                        R_bt_list.append(R_tool.from_euler(order, euler_angles).as_matrix())
                except:
                    continue

                try:
                    # Alternating optimization iteration
                    for _ in range(15):
                        # Step A: Estimate camera rotation
                        P_board_base = P_robot + np.array([R @ t_off for R in R_bt_list])
                        cA = np.mean(P_board_base, axis=0)
                        cB = np.mean(P_cam, axis=0)
                        
                        H = (P_board_base - cA).T @ (P_cam - cB)
                        U, S, Vt = np.linalg.svd(H)
                        R_cam_base = Vt.T @ U.T
                        if np.linalg.det(R_cam_base) < 0:
                            Vt[2, :] *= -1
                            R_cam_base = Vt.T @ U.T
                        R_base_cam = R_cam_base.T
                        
                        # Step B: Least squares for camera position translation and board TCP offset
                        A_mat = []
                        B_mat = []
                        for i in range(len(samples)):
                            A_mat.append(np.hstack([np.eye(3), -R_bt_list[i]]))
                            B_mat.append(P_robot[i] - R_base_cam @ P_cam[i])
                        
                        res_lstsq, _, _, _ = np.linalg.lstsq(np.vstack(A_mat), np.hstack(B_mat), rcond=None)
                        t_base_cam = res_lstsq[:3]
                        t_off = res_lstsq[3:]
                    
                    if np.linalg.norm(t_off) > 500.0:
                        continue

                    # Calculate residual projection error
                    t_errs = []
                    for i in range(len(samples)):
                        p_board_base = P_robot[i] + R_bt_list[i] @ t_off
                        p_cam_pred = R_cam_base @ (p_board_base - t_base_cam)
                        t_errs.append(np.linalg.norm(p_cam_pred - P_cam[i]))
                    
                    mean_err = np.mean(t_errs)
                    if mean_err < best_err:
                        best_err = mean_err
                        best_res = (R_base_cam, t_base_cam, t_off, mean_err, order, np.array(s_vec))
                except:
                    continue
        return best_res

    def calculate_rotation_error(self, samples, best_res):
        R_bc, t_bc, t_off, err, order, s_vec = best_res
        r_errs = []
        T_tt_list = []
        for s in samples:
            p = s["pose"]
            euler_angles = [p['a'] * s_vec[0], p['b'] * s_vec[1], p['c'] * s_vec[2]]
            R_bt = R_tool.from_euler(order, euler_angles).as_matrix()
            
            T_bt = np.eye(4)
            T_bt[:3, :3] = R_bt
            T_bt[:3, 3] = [p['x'], p['y'], p['z']]
            
            T_cb = np.eye(4)
            T_cb[:3, :3] = s["R_cb"]
            T_cb[:3, 3] = s["t_cb"]
            
            T_bc_mat = np.eye(4)
            T_bc_mat[:3, :3] = R_bc
            T_bc_mat[:3, 3] = t_bc
            
            T_tt = np.linalg.inv(T_bt) @ T_bc_mat @ T_cb
            T_tt_list.append(T_tt)
        
        avg_T_tt = np.mean(T_tt_list, axis=0)
        U, _, Vt = np.linalg.svd(avg_T_tt[:3, :3])
        avg_T_tt[:3, :3] = U @ Vt
        
        for T in T_tt_list:
            r_diff = T[:3, :3].T @ avg_T_tt[:3, :3]
            trace_val = (np.trace(r_diff) - 1.0) / 2.0
            angle_val = np.arccos(np.clip(trace_val, -1.0, 1.0))
            r_errs.append(np.degrees(angle_val))
            
        return np.mean(r_errs)

    # ═══════════════════════════════════════════════
    #   Jogging Controls
    # ═══════════════════════════════════════════════
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

        # Check safety boundaries in configurations
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

        # Check accessibility
        if not self.robot.is_reachable(target_pose, "MOVJ"):
            QMessageBox.warning(self, "Unreachable", "The target jog pose is kinematics-unreachable (MOVJ).")
            return

        # Send command blocking
        self.txt_calib_log.append(f"[Move] Target: X:{x:.2f} Y:{y:.2f} Z:{z:.2f} A:{self.jog_inputs['A'].value():.2f} B:{self.jog_inputs['B'].value():.2f} C:{self.jog_inputs['C'].value():.2f}")
        self.robot.move_j(target_pose)
        self.read_robot_pose()

        actual_pose = self.robot.get_current_pose()
        if actual_pose is not None:
            self.txt_calib_log.append(f"[Move] Actual: X:{actual_pose.x:.2f} Y:{actual_pose.y:.2f} Z:{actual_pose.z:.2f} A:{np.degrees(actual_pose.a):.2f} B:{np.degrees(actual_pose.b):.2f} C:{np.degrees(actual_pose.c):.2f}")
        else:
            self.txt_calib_log.append("[Move] Failed to query actual reached pose.")



    def jog_step(self, axis, direction):
        if not self.robot_connected or not self.robot:
            QMessageBox.warning(self, "Warning", "Robot is not connected. Connect the robot first.")
            return

        # Read current pose values on widgets
        step_xyz = self.spin_step_xyz.value()
        step_abc = self.spin_step_abc.value()

        current_val = self.jog_inputs[axis].value()
        if axis in ['X', 'Y', 'Z']:
            new_val = current_val + direction * step_xyz
        else:
            new_val = current_val + direction * step_abc
            # Wrap around angles
            if new_val > 180.0: new_val -= 360.0
            if new_val < -180.0: new_val += 360.0

        self.jog_inputs[axis].setValue(new_val)
        self.move_to_jog_pose()

    # ═══════════════════════════════════════════════
    #   Verification Logic
    # ═══════════════════════════════════════════════
    def select_calib_file(self):
        f, _ = QFileDialog.getOpenFileName(self, "Select Calibration File", PROJECT_ROOT, "YAML Files (*.yaml)")
        if f:
            self.load_calib_file(f)

    def load_calib_file(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                self.calib_data = yaml.safe_load(f)
            self.lbl_calib_path.setText(os.path.basename(path))
            self.btn_ver_move.setEnabled(False)
            self.target_uv = None
            self.target_n_cam = None
            self.target_pose_data = None
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Could not load calibration file: {e}")

    def get_robust_depth(self, depth_map, u, v, max_r=5):
        h, w = depth_map.shape
        z = float(depth_map[v, u])
        if z > 0:
            return z
        # Search outward in concentric square rings for first valid depth pixel
        for r in range(1, max_r + 1):
            valid = []
            for du in range(-r, r+1):
                for dv in range(-r, r+1):
                    nu, nv = u + du, v + dv
                    if 0 <= nu < w and 0 <= nv < h:
                        val = float(depth_map[nv, nu])
                        if val > 0:
                            valid.append(val)
            if valid:
                return float(np.median(valid))
        return 0.0

    def handle_verify_click(self, pos):
        if not self.calib_data:
            QMessageBox.warning(self, "Warning", "Please load a calibration result file first.")
            return
        if self.current_depth is None:
            QMessageBox.warning(self, "Warning", "No depth stream available.")
            return

        # Map QLabel coordinates back to camera native frame resolution
        if self.current_color is None:
            return
        H_native, W_native = self.current_color.shape[:2]

        # Calculate coordinates mappings
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

        self.target_uv = (u, v)

        # Depth value
        z_val = self.get_robust_depth(self.current_depth, u, v)
        if z_val <= 0:
            self.lbl_coord_cam.setText("Invalid Depth (0)")
            self.lbl_coord_base.setText("N/A")
            self.lbl_normal_base.setText("N/A")
            self.btn_ver_move.setEnabled(False)
            return

        K = np.array(self.calib_data["camera_params"]["intrinsic_matrix"])
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # Calculate camera 3D space
        x_val = (u - cx) * z_val / fx
        y_val = (v - cy) * z_val / fy
        p_cam = np.array([x_val, y_val, z_val])
        self.lbl_coord_cam.setText(f"X:{x_val:.1f} Y:{y_val:.1f} Z:{z_val:.1f}")

        # Map to robot Base space
        T_bc = np.array(self.calib_data["T_base_camera"])
        R_bc = T_bc[:3, :3]
        t_bc = T_bc[:3, 3]

        p_base = R_bc @ p_cam + t_bc
        self.lbl_coord_base.setText(f"X:{p_base[0]:.1f} Y:{p_base[1]:.1f} Z:{p_base[2]:.1f}")

        # Compute normal vector using neighborhood depth map
        n_cam = self.compute_local_normal(self.current_depth, u, v, K)
        if n_cam is not None:
            self.target_n_cam = n_cam
            n_base = R_bc @ n_cam
            self.lbl_normal_base.setText(f"X:{n_base[0]:.2f} Y:{n_base[1]:.2f} Z:{n_base[2]:.2f}")

            # Compute tool orientation perpendicular to plane (Z-axis points into surface: Z_tool = -n_base)
            z_tool = -n_base
            if abs(np.dot([0.0, 1.0, 0.0], z_tool)) < 0.98:
                x_tool = np.cross([0.0, 1.0, 0.0], z_tool)
            else:
                x_tool = np.cross([1.0, 0.0, 0.0], z_tool)
            x_tool /= np.linalg.norm(x_tool)
            y_tool = np.cross(z_tool, x_tool)

            R_bt = np.column_stack([x_tool, y_tool, z_tool])

            # Convert to Euler configuration saved in metadata
            opt_cfg = self.calib_data.get("metadata", {}).get("optimization_config", {})
            order = opt_cfg.get("axis_order", "ZYX")
            s_vec = opt_cfg.get("sign_vector", [1, 1, 1])

            try:
                euler = R_tool.from_matrix(R_bt).as_euler(order, degrees=False)
                a = euler[0] / s_vec[0]
                b = euler[1] / s_vec[1]
                c = euler[2] / s_vec[2]

                self.target_pose_data = {
                    "p_base": p_base,
                    "n_base": n_base,
                    "a": a, "b": b, "c": c
                }
                self.btn_ver_move.setEnabled(True)
            except Exception as e:
                self.btn_ver_move.setEnabled(False)
                print(f"Euler translation failed: {e}")
        else:
            self.target_n_cam = None
            self.lbl_normal_base.setText("Normal estimation failed")
            self.btn_ver_move.setEnabled(False)

    def compute_local_normal(self, depth_map, u, v, K):
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        
        step = 5
        pts = {}
        for label, du, dv in [('L', -step, 0), ('R', step, 0), ('U', 0, -step), ('D', 0, step)]:
            nu, nv = u + du, v + dv
            if 0 <= nu < depth_map.shape[1] and 0 <= nv < depth_map.shape[0]:
                z = self.get_robust_depth(depth_map, nu, nv, max_r=3)
                if z > 0:
                    x = (nu - cx) * z / fx
                    y = (nv - cy) * z / fy
                    pts[label] = np.array([x, y, z])

        if 'L' in pts and 'R' in pts and 'U' in pts and 'D' in pts:
            v1 = pts['R'] - pts['L']
            v2 = pts['D'] - pts['U']
            n = np.cross(v1, v2)
            norm = np.linalg.norm(n)
            if norm > 1e-6:
                n /= norm
                # Normal must point towards camera (negative Z direction)
                if n[2] > 0:
                    n = -n
                return n
        return None

    def move_to_verification_pose(self):
        if not self.robot_connected or not self.robot:
            QMessageBox.warning(self, "Warning", "Robot is not connected. Connect the robot first.")
            return
        if not self.target_pose_data:
            return

        offset = self.spin_ver_offset.value()
        p_dest = self.target_pose_data["p_base"] + self.target_pose_data["n_base"] * offset
        a = self.target_pose_data["a"]
        b = self.target_pose_data["b"]
        c = self.target_pose_data["c"]

        target_pose = RobotPose(p_dest[0], p_dest[1], p_dest[2], a, b, c)

        # Check safety limits
        planner_cfg = self.config.get("vision", {}).get("planner", {})
        lim = planner_cfg.get("workspace_limits", {})
        if lim:
            if not (lim.get("x", [-9999, 9999])[0] <= p_dest[0] <= lim.get("x", [-9999, 9999])[1]):
                QMessageBox.warning(self, "Safety Limit", f"Target destination X ({p_dest[0]:.1f}) is out of workspace limits.")
                return
            if not (lim.get("y", [-9999, 9999])[0] <= p_dest[1] <= lim.get("y", [-9999, 9999])[1]):
                QMessageBox.warning(self, "Safety Limit", f"Target destination Y ({p_dest[1]:.1f}) is out of workspace limits.")
                return
            if not (lim.get("z", [-9999, 9999])[0] <= p_dest[2] <= lim.get("z", [-9999, 9999])[1]):
                QMessageBox.warning(self, "Safety Limit", f"Target destination Z ({p_dest[2]:.1f}) is out of workspace limits.")
                return

        # Check accessibility
        if not self.robot.is_reachable(target_pose, "MOVL"):
            QMessageBox.warning(self, "Unreachable", "The target safety position is out of the robot's physical reach.")
            return

        # Execute non-blockingly
        self.robot.move_l(target_pose, wait=False)

    def closeEvent(self, event):
        self.disconnect_robot()
        if self.cam:
            try: self.cam.stop()
            except: pass
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CalibUI()
    window.show()
    sys.exit(app.exec_())
