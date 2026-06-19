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
                             QFormLayout, QDoubleSpinBox, QTextEdit, QGridLayout,
                             QSizePolicy, QScrollArea, QFrame, QProgressDialog, QProgressBar)
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

class ClickableFrame(QFrame):
    clicked = pyqtSignal()
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit()

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
        self.exec_state = "idle"
        self.exec_item_index = 0
        self.exec_waypoint_index = 0

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
        main_layout = QVBoxLayout(tab)

        # Top area: Video (left) and Controls (right)
        top_layout = QHBoxLayout()

        # Left Column: Camera View
        left_layout = QVBoxLayout()
        self.cap_video = QLabel()
        self.cap_video.setMinimumSize(400, 225)
        self.cap_video.setAlignment(Qt.AlignCenter)
        self.cap_video.setStyleSheet("background-color: #000; border: 2px solid #222;")
        left_layout.addWidget(self.cap_video)
        
        top_layout.addLayout(left_layout, 10)

        # Middle Column: Captured Samples list
        self.samples_group = QGroupBox("Samples")
        self.samples_group.setFixedWidth(150)
        middle_vbox = QVBoxLayout(self.samples_group)
        middle_vbox.setContentsMargins(5, 5, 5, 5)
        
        self.samples_scroll = QScrollArea()
        self.samples_scroll.setWidgetResizable(True)
        self.samples_scroll.setStyleSheet("QScrollArea { border: none; background-color: transparent; }")
        
        self.samples_content = QWidget()
        self.samples_content.setStyleSheet("QWidget { background-color: transparent; }")
        self.samples_layout = QVBoxLayout(self.samples_content)
        self.samples_layout.setContentsMargins(0, 0, 0, 0)
        self.samples_layout.setSpacing(5)
        
        self.samples_scroll.setWidget(self.samples_content)
        middle_vbox.addWidget(self.samples_scroll)
        
        top_layout.addWidget(self.samples_group, 2)

        # Right Column: Controls Splitter
        right_layout = QVBoxLayout()

        # 0. Camera Status Group (above Robot Connection)
        self.cam_group = QGroupBox("Camera Connection: Disconnected")
        self.cam_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")
        cam_layout = QHBoxLayout(self.cam_group)
        cam_layout.setContentsMargins(5, 5, 5, 5)
        self.btn_reconnect_cam = QPushButton("Retry Camera")
        self.btn_reconnect_cam.setFixedWidth(100)
        self.btn_reconnect_cam.clicked.connect(self.start_camera)

        cam_layout.addWidget(self.btn_reconnect_cam)
        cam_layout.addStretch()
        right_layout.addWidget(self.cam_group)

        # 1. Connection Group
        self.conn_group = QGroupBox("Robot Connection: Disconnected")
        self.conn_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")
        conn_layout = QHBoxLayout(self.conn_group)
        conn_layout.setContentsMargins(5, 5, 5, 5)
        conn_layout.setSpacing(5)
        self.txt_ip = QLineEdit(self.default_ip)
        self.txt_ip.setFixedWidth(90)
        self.txt_port = QLineEdit(self.default_port)
        self.txt_port.setFixedWidth(40)
        self.btn_connect_robot = QPushButton("Connect Robot")
        self.btn_connect_robot.clicked.connect(self.toggle_robot_connection)

        conn_layout.addWidget(QLabel("IP:"))
        conn_layout.addWidget(self.txt_ip)
        conn_layout.addSpacing(8)
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.txt_port)
        conn_layout.addSpacing(15)
        conn_layout.addWidget(self.btn_connect_robot)
        conn_layout.addStretch()
        right_layout.addWidget(self.conn_group)

        # 2. Capture & Calibration Group (Directory & Info)
        cap_group = QGroupBox("Calibration Directory & Info")
        cap_vbox = QVBoxLayout(cap_group)
        cap_vbox.setContentsMargins(5, 5, 5, 5)

        # Select Dir Row
        dir_layout = QHBoxLayout()
        self.btn_sel_dir = QPushButton("Select Calib Dir")
        self.btn_sel_dir.clicked.connect(self.select_save_dir)
        self.lbl_save_dir = QLabel("Not Selected")
        self.lbl_save_dir.setWordWrap(True)
        dir_layout.addWidget(self.btn_sel_dir)
        dir_layout.addWidget(self.lbl_save_dir, 1)
        cap_vbox.addLayout(dir_layout)

        right_layout.addWidget(cap_group)

        # 3. Jogging & Actions Group
        jog_group = QGroupBox("Robot Jogging & Actions")
        jog_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        jog_layout = QVBoxLayout(jog_group)
        jog_layout.setContentsMargins(5, 5, 5, 5)

        # Coordinates grid (X, Y, Z, A, B, C)
        jog_grid = QGridLayout()
        self.jog_inputs = {}
        axes = ['X', 'Y', 'Z', 'A', 'B', 'C']
        
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
        jog_layout.addLayout(jog_grid)
        jog_layout.addStretch()

        # 2x2 grid of buttons to fit the narrower control panel
        buttons_grid = QGridLayout()
        
        self.btn_capture = QPushButton("Capture")
        self.btn_capture.clicked.connect(self.capture_sample)
        
        self.btn_run_calib = QPushButton("Calibrate")
        self.btn_run_calib.clicked.connect(self.run_calibration)
        
        self.btn_read_pos = QPushButton("Read Current Pose")
        self.btn_read_pos.clicked.connect(self.read_robot_pose)
        
        self.btn_move_to_jog = QPushButton("Move to Target Pose")
        self.btn_move_to_jog.clicked.connect(self.move_to_jog_pose)

        buttons_grid.addWidget(self.btn_read_pos, 0, 0)
        buttons_grid.addWidget(self.btn_move_to_jog, 0, 1)
        buttons_grid.addWidget(self.btn_capture, 1, 0)
        buttons_grid.addWidget(self.btn_run_calib, 1, 1)
        jog_layout.addLayout(buttons_grid)
        
        right_layout.addWidget(jog_group)
        
        top_layout.addLayout(right_layout, 3)
        
        # Add top layout to main layout
        main_layout.addLayout(top_layout, 0)

        # Bottom Area: txt_calib_log (spanning entire width)
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
        
        main_layout.addWidget(self.txt_calib_log, 1)

        tab.setLayout(main_layout)
        self.tabs.addTab(tab, "Calibrate")

    def init_verify_tab(self):
        tab = QWidget()
        main_layout = QVBoxLayout(tab)

        # Top area: Video/Left and Config/Right
        top_layout = QHBoxLayout()

        # Left Column: Clickable Video View
        left_layout = QVBoxLayout()
        self.ver_video = ClickableLabel()
        self.ver_video.clicked_pos.connect(self.handle_verify_click)
        left_layout.addWidget(self.ver_video)
        
       # self.lbl_ver_tip = QLabel("Instruction: Click image to draw points/polylines. Press 'Finish Current Segment' to finalize.")
       # self.lbl_ver_tip.setStyleSheet("color: #bbb; italic: true;")
       # left_layout.addWidget(self.lbl_ver_tip)
        top_layout.addLayout(left_layout, 3)

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

        # Real-time Robot Pose Group
        pose_group = QGroupBox("Real-time Robot Pose")
        pose_grid = QGridLayout(pose_group)
        self.lbl_real_x = QLabel("X: N/A")
        self.lbl_real_y = QLabel("Y: N/A")
        self.lbl_real_z = QLabel("Z: N/A")
        self.lbl_real_a = QLabel("A: N/A")
        self.lbl_real_b = QLabel("B: N/A")
        self.lbl_real_c = QLabel("C: N/A")
        
        # Style them to look professional and clear
        for lbl in [self.lbl_real_x, self.lbl_real_y, self.lbl_real_z, 
                    self.lbl_real_a, self.lbl_real_b, self.lbl_real_c]:
            lbl.setStyleSheet("font-family: monospace; font-size: 11px; font-weight: bold;")
            
        pose_grid.addWidget(self.lbl_real_x, 0, 0)
        pose_grid.addWidget(self.lbl_real_y, 0, 1)
        pose_grid.addWidget(self.lbl_real_z, 0, 2)
        pose_grid.addWidget(self.lbl_real_a, 1, 0)
        pose_grid.addWidget(self.lbl_real_b, 1, 1)
        pose_grid.addWidget(self.lbl_real_c, 1, 2)
        right_layout.addWidget(pose_group)

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

        # Draw segment/point controls
        self.btn_finish_line = QPushButton("Finish Current Segment / Point")
        #self.btn_finish_line.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.btn_finish_line.clicked.connect(self.finish_current_draw_item)
        action_vbox.addWidget(self.btn_finish_line)

        self.btn_clear_last = QPushButton("Clear Last Point")
        self.btn_clear_last.clicked.connect(self.clear_last_point)
        action_vbox.addWidget(self.btn_clear_last)

        self.btn_clear_items = QPushButton("Clear All Draw Items")
        #self.btn_clear_items.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.btn_clear_items.clicked.connect(self.clear_draw_items)
        action_vbox.addWidget(self.btn_clear_items)

        self.btn_ver_move = QPushButton("Move Robot (Execute Route)")
        self.btn_ver_move.setEnabled(False)
        #self.btn_ver_move.setMinimumHeight(40)
        #self.btn_ver_move.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold; font-size: 13px;")
        self.btn_ver_move.clicked.connect(self.move_to_verification_pose)
        action_vbox.addWidget(self.btn_ver_move)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setVisible(False)
        action_vbox.addWidget(self.progress_bar)
        action_group.setLayout(action_vbox)
        
        right_layout.addWidget(action_group)
        right_layout.addStretch()
        top_layout.addLayout(right_layout, 1)

        main_layout.addLayout(top_layout, 0)

        # Bottom Area: txt_verify_log (spanning entire width)
        self.txt_verify_log = QTextEdit()
        self.txt_verify_log.setReadOnly(True)
        self.txt_verify_log.setLineWrapMode(QTextEdit.NoWrap)
        
        # Patch append to prepend datetime
        _orig_append = self.txt_verify_log.append
        def log_append(text):
            _orig_append(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {text}")
            self.txt_verify_log.horizontalScrollBar().setValue(0)
        self.txt_verify_log.append = log_append
        
        main_layout.addWidget(self.txt_verify_log, 1)

        tab.setLayout(main_layout)
        self.tabs.addTab(tab, "Verify")

        # Load default/latest result file if exists
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

    def start_camera(self):
        try:
            self.cam_group.setTitle("Camera Connection: Connecting...")
            self.cam_group.setStyleSheet("QGroupBox::title { color: #FF9800; font-weight: bold; }")
            QApplication.processEvents()

            self.cam = get_camera(self.camera_model)
            self.cam.start()
            self.cam_connected = True
            
            self.cam_group.setTitle("Camera Connection: Connected")
            self.cam_group.setStyleSheet("QGroupBox::title { color: #4CAF50; font-weight: bold; }")
            
            # Setup default output dir automatically if not chosen
            if not self.save_dir:
                ts = datetime.now().strftime("%Y%m%d")
                self.setup_save_dir(os.path.join(self.default_output_dir, f"calib_{ts}"))
        except Exception as e:
            self.cam_connected = False
            self.cam_group.setTitle("Camera Connection: Offline")
            self.cam_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")
            print(f"Camera start failed: {e}")

    def toggle_robot_connection(self):
        if not self.robot_connected:
            ip = self.txt_ip.text().strip()
            port = self.txt_port.text().strip()

            self.conn_group.setTitle("Robot Connection: Connecting...")
            self.conn_group.setStyleSheet("QGroupBox::title { color: #FF9800; font-weight: bold; }")
            QApplication.processEvents()

            try:
                self.robot = InexbotDriver(ip=ip, port=port)
                if not self.robot.startup(timeout=5.0):
                    QMessageBox.critical(self, "Error", "Robot startup failed! Make sure controller IP/Port are correct.")
                    self.robot = None
                    self.conn_group.setTitle("Robot Connection: Disconnected")
                    self.conn_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")
                    return

                self.robot_connected = True
                self.btn_connect_robot.setText("Disconnect Robot")
                self.btn_connect_robot.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
                
                self.conn_group.setTitle("Robot Connection: Connected")
                self.conn_group.setStyleSheet("QGroupBox::title { color: #4CAF50; font-weight: bold; }")

                # Read current pose to populate jogging GUI
                self.read_robot_pose()

            except Exception as e:
                self.robot = None
                self.conn_group.setTitle("Robot Connection: Disconnected")
                self.conn_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")
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
        self.conn_group.setTitle("Robot Connection: Disconnected")
        self.conn_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")

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
        
        self.refresh_samples_list()

    def update_frame(self):
        # Keepalive querying to prevent connection dropouts
        if self.robot_connected and self.robot:
            if time.time() - self.last_heartbeat_time > 2.0:
                try:
                    self.robot.get_running_state()
                except:
                    pass
                self.last_heartbeat_time = time.time()

        # Route execution monitoring
        if hasattr(self, 'exec_state') and self.exec_state == "moving":
            if self.robot_connected and self.robot:
                now = time.time()
                if now - self.last_move_cmd_time > 0.8:
                    if self.robot.is_robot_idle():
                        if hasattr(self, 'completed_waypoints'):
                            self.completed_waypoints += 1
                        self.execute_next_verify_step()

        # Query/Update real-time robot pose and progress (throttled to 150ms)
        if self.robot_connected and self.robot:
            now = time.time()
            if not hasattr(self, 'last_pose_query_time') or now - self.last_pose_query_time > 0.15:
                self.last_pose_query_time = now
                self.update_realtime_pose_and_progress()
        else:
            # If disconnected, clear the coordinate panel to "N/A"
            if hasattr(self, 'lbl_real_x') and self.lbl_real_x.text() != "X: N/A":
                self.lbl_real_x.setText("X: N/A")
                self.lbl_real_y.setText("Y: N/A")
                self.lbl_real_z.setText("Z: N/A")
                self.lbl_real_a.setText("A: N/A")
                self.lbl_real_b.setText("B: N/A")
                self.lbl_real_c.setText("C: N/A")

        # If a sample is selected and we are on the Calibrate tab, show its preview image
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
            
            # Draw samples count overlay on the frame
            if self.cap_info is not None:
                count = len(self.cap_info.get('samples', []))
                text = f"Samples: {count}"
            else:
                text = "Samples: -"
            cv2.putText(display_frame, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            self.render_image_to_label(display_frame, self.cap_video)

        elif active_tab == 1:
            # Display verification screen
            display_frame = color.copy()

            def draw_point_normal(img, u, v, n_cam, color_norm):
                if n_cam is not None and self.calib_data:
                    K = np.array(self.calib_data["camera_params"]["intrinsic_matrix"])
                    fx, fy = K[0, 0], K[1, 1]
                    cx, cy = K[0, 2], K[1, 2]
                    z_val = float(self.current_depth[v, u]) if self.current_depth is not None else 0.0
                    if z_val > 0:
                        x_val = (u - cx) * z_val / fx
                        y_val = (v - cy) * z_val / fy
                        p_cam_origin = np.array([x_val, y_val, z_val])
                        p_cam_normal_tip = p_cam_origin + n_cam * 50.0  # 50 mm normal arrow
                        if p_cam_normal_tip[2] > 0:
                            u_tip = int(fx * p_cam_normal_tip[0] / p_cam_normal_tip[2] + cx)
                            v_tip = int(fy * p_cam_normal_tip[1] / p_cam_normal_tip[2] + cy)
                            cv2.arrowedLine(img, (u, v), (u_tip, v_tip), color_norm, 2, tipLength=0.2)

            # Draw all committed items (polylines/points)
            for item in self.verify_items:
                is_visited = (item.get("status") == "visited")
                # Visited -> Blue (255, 0, 0), Pending -> Green (0, 255, 0)
                item_color = (255, 0, 0) if is_visited else (0, 255, 0)
                norm_color = (180, 50, 50) if is_visited else (0, 0, 255)
                
                points = item["points"]
                poses = item["pose_data"]
                
                # Draw markers
                for idx, pt in enumerate(points):
                    cv2.circle(display_frame, pt, 5, item_color, -1)
                    if idx < len(poses):
                        draw_point_normal(display_frame, pt[0], pt[1], poses[idx]["n_cam"], norm_color)
                
                # Draw direction lines if it's a polyline
                if len(points) > 1:
                    for i in range(len(points) - 1):
                        cv2.arrowedLine(display_frame, points[i], points[i+1], item_color, 2, tipLength=0.15)
                
                # Draw Item ID label next to the first point
                if points:
                    first_pt = points[0]
                    cv2.putText(display_frame, f"#{item['id']}", (first_pt[0] + 8, first_pt[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(display_frame, f"#{item['id']}", (first_pt[0] + 8, first_pt[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, item_color, 1, cv2.LINE_AA)

            # Draw the active/temporary item currently being drawn
            if self.current_draw_points:
                temp_color = (0, 165, 255)  # Orange
                temp_norm_color = (0, 0, 255)
                for idx, pt in enumerate(self.current_draw_points):
                    cv2.circle(display_frame, pt, 5, temp_color, -1)
                    if idx < len(self.current_draw_poses):
                        draw_point_normal(display_frame, pt[0], pt[1], self.current_draw_poses[idx]["n_cam"], temp_norm_color)
                
                if len(self.current_draw_points) > 1:
                    for i in range(len(self.current_draw_points) - 1):
                        cv2.arrowedLine(display_frame, self.current_draw_points[i], self.current_draw_points[i+1], temp_color, 2, tipLength=0.15)
                
                # Draw temporary label "Drawing..." next to first point
                first_pt = self.current_draw_points[0]
                cv2.putText(display_frame, "Drawing...", (first_pt[0] + 8, first_pt[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, temp_color, 1, cv2.LINE_AA)

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

        self.txt_calib_log.append(f"[OK] Captured Sample {count}: {img_name}")
        self.txt_calib_log.append(f"     Pose: X:{pose.x:.1f} Y:{pose.y:.1f} Z:{pose.z:.1f} A:{pose.a:.4f} B:{pose.b:.4f} C:{pose.c:.4f}")
        self.refresh_samples_list()

    def run_calibration(self):
        if not self.save_dir or not os.path.exists(self.yaml_path):
            QMessageBox.warning(self, "Warning", "Save directory does not contain calibration info!")
            return

        self.txt_calib_log.clear()
        self.txt_calib_log.append("====== Starting Calibration Process ======")

        # Stop timer to avoid overwriting cap_video
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

            # Initialize 3D points
            objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * sq_size

            all_samples = []
            self.txt_calib_log.append("[*] Extracting corners and solving PnP...")

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

                # Overlay status text on image and display
                status_text = f"Sample ID {s['id']}: {'SUCCESS' if ret else 'FAILED'}"
                text_color = (0, 255, 0) if ret else (0, 0, 255)
                cv2.putText(display_img, status_text, (15, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2, cv2.LINE_AA)
                self.render_image_to_label(display_img, self.cap_video)
                
                # Small sleep/delay to allow viewing corner detection
                time.sleep(0.15)
                QApplication.processEvents()

            progress.setValue(total_samples)
            progress.close()
            progress = None

            if len(all_samples) < 3:
                self.txt_calib_log.append("[-] Verification failed: insufficient valid chessboard corners found.")
                return

            # Clean data (displacement check)
            self.txt_calib_log.append("[*] Cleaning data based on motion displacement...")
            QApplication.processEvents()
            clean_thr = self.c_cfg.get("cleaning_threshold", 0.05)
            samples = self.clean_calibration_data(all_samples, threshold=clean_thr)

            if len(samples) < 3:
                self.txt_calib_log.append("[-] Calibration failed: not enough clean samples.")
                return

            # Evaluate diversity
            diversity = self.evaluate_data_diversity(samples)
            self.txt_calib_log.append(f"[*] Diversity score: {diversity['score']:.1f} / 100")
            QApplication.processEvents()

            # Optimization search
            self.txt_calib_log.append("[*] Searching for optimal camera extrinsics & chessboard offset...")
            QApplication.processEvents()
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

            # Load it into the verify tab automatically
            self.load_calib_file(calib_out_path)

        except Exception as e:
            self.txt_calib_log.append(f"[-] Calibration crashed: {e}")
            QMessageBox.critical(self, "Error", f"Calibration execution failed: {e}")
        finally:
            if progress is not None:
                progress.close()
            if timer_was_active:
                self.timer.start(30)

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
                        best_res = (R_base_cam, t_base_cam, t_off, mean_err, order, s_vec)
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
            self.lbl_calib_path.setText(path)
            self.btn_ver_move.setEnabled(False)
            self.target_uv = None
            self.target_n_cam = None
            self.target_pose_data = None
            if hasattr(self, 'txt_verify_log'):
                self.txt_verify_log.append(f"[OK] Loaded calibration file: {path}")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Could not load calibration file: {e}")
            if hasattr(self, 'txt_verify_log'):
                self.txt_verify_log.append(f"[-] Failed to load calibration file: {e}")

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

    def clear_last_point(self):
        if self.current_draw_points:
            # Pop from active drawing segment
            pt = self.current_draw_points.pop()
            self.current_draw_poses.pop()
            self.txt_verify_log.append(f"[*] Cleared last point in current segment: U={pt[0]}, V={pt[1]}")
            # Reset coordinate displays if segment becomes empty
            if not self.current_draw_points:
                self.lbl_coord_cam.setText("N/A")
                self.lbl_coord_base.setText("N/A")
                self.lbl_normal_base.setText("N/A")
        elif self.verify_items:
            # Pop from last committed item
            last_item = self.verify_items[-1]
            if last_item["points"]:
                pt = last_item["points"].pop()
                last_item["pose_data"].pop()
                self.txt_verify_log.append(f"[*] Cleared last point from Item #{last_item['id']}: U={pt[0]}, V={pt[1]}")
                # If item is empty, delete item entirely
                if not last_item["points"]:
                    self.verify_items.pop()
                    self.txt_verify_log.append(f"[*] Item #{last_item['id']} has no points remaining, removed.")
            else:
                self.verify_items.pop()
                self.txt_verify_log.append(f"[*] Removed empty Item #{last_item['id']}.")

            # Update movement button state if no items left
            if not self.verify_items:
                self.btn_ver_move.setEnabled(False)
                self.lbl_coord_cam.setText("N/A")
                self.lbl_coord_base.setText("N/A")
                self.lbl_normal_base.setText("N/A")
        else:
            self.txt_verify_log.append("[!] No points to clear.")

    def finish_current_draw_item(self):
        if not self.current_draw_points:
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
        
        # Reset current drawing state
        self.current_draw_points.clear()
        self.current_draw_poses.clear()
        
        # Enable the execute route button since we have items
        self.btn_ver_move.setEnabled(True)

    def clear_draw_items(self):
        self.verify_items.clear()
        self.current_draw_points.clear()
        self.current_draw_poses.clear()
        self.lbl_coord_cam.setText("N/A")
        self.lbl_coord_base.setText("N/A")
        self.lbl_normal_base.setText("N/A")
        self.exec_state = "idle"
        self.btn_ver_move.setText("Move Robot (Execute Route)")
        self.btn_ver_move.setEnabled(False)
        self.txt_verify_log.append("[*] Cleared all drawn points and lines.")

    def stop_verify_execution(self):
        self.exec_state = "idle"
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
            
            # Check for invalid values from SDK
            pose_list = curr_pose.to_list()
            if any(v < -2500.0 for v in pose_list[:3]):
                return
                
            # Update display labels
            self.lbl_real_x.setText(f"X: {curr_pose.x:.2f}")
            self.lbl_real_y.setText(f"Y: {curr_pose.y:.2f}")
            self.lbl_real_z.setText(f"Z: {curr_pose.z:.2f}")
            self.lbl_real_a.setText(f"A: {curr_pose.a:.4f}")
            self.lbl_real_b.setText(f"B: {curr_pose.b:.4f}")
            self.lbl_real_c.setText(f"C: {curr_pose.c:.4f}")
            
            # Update progress bar if moving
            if hasattr(self, 'exec_state') and self.exec_state == "moving":
                if hasattr(self, 'total_path_distance') and self.total_path_distance > 0:
                    p_curr = np.array([curr_pose.x, curr_pose.y, curr_pose.z])
                    # Retrieve flat index of current target waypoint
                    flat_idx = self.waypoint_flat_map.get((self.exec_item_index, self.exec_waypoint_index - 1))
                    if flat_idx is not None and flat_idx < len(self.flat_path_points):
                        p_target = self.flat_path_points[flat_idx]
                        s_len = self.segment_lengths[flat_idx - 1]
                        d_completed_prior = self.cumulative_distances[flat_idx - 1]
                        
                        d_to_target = np.linalg.norm(p_target - p_curr)
                        if s_len > 0:
                            d_seg_progress = max(0.0, min(s_len, s_len - d_to_target))
                        else:
                            d_seg_progress = 0.0
                            
                        d_completed = d_completed_prior + d_seg_progress
                        progress_percent = int((d_completed / self.total_path_distance) * 100)
                        progress_percent = max(0, min(100, progress_percent))
                        
                        self.progress_bar.setValue(progress_percent)
                        self.progress_bar.setFormat(f"Progress: %p% ({int(d_completed)}/{int(self.total_path_distance)} mm)")
        except Exception:
            pass

    def execute_next_verify_step(self):
        # Cooldown check
        if hasattr(self, 'last_move_cmd_time') and time.time() - self.last_move_cmd_time < 0.8:
            return

        if self.exec_item_index >= len(self.verify_items):
            # All items finished!
            self.exec_state = "idle"
            self.btn_ver_move.setText("Move Robot (Execute Route)")
            self.btn_ver_move.setEnabled(True)
            self.progress_bar.setValue(100)
            self.progress_bar.setVisible(False)
            self.txt_verify_log.append("[OK] All verify items executed successfully!")
            return

        current_item = self.verify_items[self.exec_item_index]
        poses = current_item["pose_data"]

        if self.exec_waypoint_index >= len(poses):
            # Current item finished! Mark it as visited (for coloring)
            current_item["status"] = "visited"
            self.txt_verify_log.append(f"[OK] Item #{current_item['id']} finished.")
            # Move to next item
            self.exec_item_index += 1
            self.exec_waypoint_index = 0
            self.execute_next_verify_step()
            return

        # Move to the current waypoint of the current item
        pose_info = poses[self.exec_waypoint_index]
        offset = self.spin_ver_offset.value()
        p_dest = pose_info["p_base"] + pose_info["n_base"] * offset
        a = pose_info["a"]
        b = pose_info["b"]
        c = pose_info["c"]

        target_pose = RobotPose(p_dest[0], p_dest[1], p_dest[2], a, b, c)

        # Check safety limits
        planner_cfg = self.config.get("vision", {}).get("planner", {})
        lim = planner_cfg.get("workspace_limits", {})
        if lim:
            for axis, idx in [("x", 0), ("y", 1), ("z", 2)]:
                val = p_dest[idx]
                limits = lim.get(axis, [-9999, 9999])
                if not (limits[0] <= val <= limits[1]):
                    msg = f"Item #{current_item['id']} Pt #{self.exec_waypoint_index + 1} target destination {axis.upper()} ({val:.1f}) out of workspace limits."
                    QMessageBox.warning(self, "Safety Limit", msg)
                    self.txt_verify_log.append(f"  [!] Safety Limit: {msg}")
                    self.stop_verify_execution()
                    return

        # Determine motion type (move_j for first waypoint of any item, move_l for others)
        if self.exec_waypoint_index == 0:
            move_mode = "MOVJ"
        else:
            move_mode = "MOVL"

        # Check accessibility
        if not self.robot.is_reachable(target_pose, move_mode):
            msg = f"Item #{current_item['id']} Pt #{self.exec_waypoint_index + 1} target pose is kinematics-unreachable ({move_mode})."
            QMessageBox.warning(self, "Unreachable", msg)
            self.txt_verify_log.append(f"  [!] Unreachable: {msg}")
            self.stop_verify_execution()
            return

        # Execute motion non-blockingly
        self.txt_verify_log.append(f"[*] Moving ({move_mode}) to Item #{current_item['id']} Pt #{self.exec_waypoint_index + 1}: X={p_dest[0]:.1f}, Y={p_dest[1]:.1f}, Z={p_dest[2]:.1f}, A={a:.4f}, B={b:.4f}, C={c:.4f}")
        if move_mode == "MOVJ":
            self.robot.move_j(target_pose, wait=False)
        else:
            self.robot.move_l(target_pose, wait=False)
        self.last_move_cmd_time = time.time()

        # Advance waypoint index for next check
        self.exec_waypoint_index += 1

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

        # Depth value
        z_val = self.get_robust_depth(self.current_depth, u, v)
        if z_val <= 0:
            self.lbl_coord_cam.setText("Invalid Depth (0)")
            self.lbl_coord_base.setText("N/A")
            self.lbl_normal_base.setText("N/A")
            self.txt_verify_log.append(f"  [!] Invalid depth value at pixel coordinate U={u}, V={v}")
            return

        K = np.array(self.calib_data["camera_params"]["intrinsic_matrix"])
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # Calculate camera 3D space
        x_val = (u - cx) * z_val / fx
        y_val = (v - cy) * z_val / fy
        p_cam = np.array([x_val, y_val, z_val])

        # Map to robot Base space
        T_bc = np.array(self.calib_data["T_base_camera"])
        R_bc = T_bc[:3, :3]
        t_bc = T_bc[:3, 3]

        p_base = R_bc @ p_cam + t_bc

        # Compute normal vector using neighborhood depth map
        n_cam = self.compute_local_normal(self.current_depth, u, v, K)
        if n_cam is not None:
            n_base = R_bc @ n_cam

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

                pose_data = {
                    "p_base": p_base,
                    "n_base": n_base,
                    "n_cam": n_cam,
                    "a": a, "b": b, "c": c
                }
                
                self.current_draw_points.append((u, v))
                self.current_draw_poses.append(pose_data)
                
                # Show active point coordinates in labels
                self.lbl_coord_cam.setText(f"X:{x_val:.1f} Y:{y_val:.1f} Z:{z_val:.1f}")
                self.lbl_coord_base.setText(f"X:{p_base[0]:.1f} Y:{p_base[1]:.1f} Z:{p_base[2]:.1f}")
                self.lbl_normal_base.setText(f"X:{n_base[0]:.2f} Y:{n_base[1]:.2f} Z:{n_base[2]:.2f}")
                
                self.txt_verify_log.append(f"[*] Added point to current segment: U={u}, V={v}. Base: X={p_base[0]:.1f}, Y={p_base[1]:.1f}, Z={p_base[2]:.1f}, A={a:.4f}, B={b:.4f}, C={c:.4f}")
                
            except Exception as e:
                self.txt_verify_log.append(f"  [!] Euler translation failed: {e}")
        else:
            self.lbl_normal_base.setText("Normal estimation failed")
            self.txt_verify_log.append(f"  [!] Failed to estimate surface normal vector")

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

        # First, finish current drawing if there's any active segment
        if self.current_draw_points:
            self.finish_current_draw_item()

        if not self.verify_items:
            QMessageBox.warning(self, "Warning", "No routes or points defined to move.")
            return

        # Start execution
        self.exec_state = "moving"
        self.exec_item_index = 0
        self.exec_waypoint_index = 0
        self.last_move_cmd_time = 0
        self.total_waypoints = sum(len(item["pose_data"]) for item in self.verify_items)
        self.completed_waypoints = 0
        self.last_progress_query_time = 0

        # Build flat path list for Cartesian distance progress tracking
        self.flat_path_points = []
        self.waypoint_flat_map = {}
        
        curr_pose = self.robot.get_current_pose()
        if curr_pose:
            p0 = np.array([curr_pose.x, curr_pose.y, curr_pose.z])
        else:
            p0 = np.array([0.0, 0.0, 0.0])
            
        self.flat_path_points.append(p0)
        
        offset = self.spin_ver_offset.value()
        flat_idx = 1
        for i_idx, item in enumerate(self.verify_items):
            for w_idx, pose_info in enumerate(item["pose_data"]):
                p_dest = pose_info["p_base"] + pose_info["n_base"] * offset
                self.flat_path_points.append(p_dest)
                self.waypoint_flat_map[(i_idx, w_idx)] = flat_idx
                flat_idx += 1
                
        # Calculate lengths
        self.segment_lengths = []
        self.cumulative_distances = [0.0]
        for i in range(1, len(self.flat_path_points)):
            dist = np.linalg.norm(self.flat_path_points[i] - self.flat_path_points[i-1])
            self.segment_lengths.append(dist)
            self.cumulative_distances.append(self.cumulative_distances[-1] + dist)
            
        self.total_path_distance = self.cumulative_distances[-1]

        self.btn_ver_move.setEnabled(False)
        self.btn_ver_move.setText("Moving...")
        self.progress_bar.setValue(0)
        if self.total_path_distance > 0:
            self.progress_bar.setFormat(f"Progress: 0% (0/{int(self.total_path_distance)} mm)")
        else:
            self.progress_bar.setFormat("Progress: 100%")
        self.progress_bar.setVisible(True)
        self.txt_verify_log.append(f"[*] Starting robot route execution (Total Distance: {self.total_path_distance:.1f} mm)...")
        
        # Reset all items status to pending so we can visualize them changing color as we execute
        for item in self.verify_items:
            item["status"] = "pending"

        self.execute_next_verify_step()

    def refresh_samples_list(self):
        # Clear existing layout items
        while self.samples_layout.count():
            child = self.samples_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        if not self.cap_info or "samples" not in self.cap_info:
            return
            
        samples = self.cap_info["samples"]
        self.samples_group.setTitle(f"Samples ({len(samples)})")
        
        for s in samples:
            item_widget = ClickableFrame()
            item_widget.setFrameShape(QFrame.StyledPanel)
            
            is_selected = (self.selected_sample_id == s["id"])
            if is_selected:
                item_widget.setStyleSheet("""
                    QFrame {
                        background-color: #1a3322;
                        border: 2px solid #4CAF50;
                        border-radius: 4px;
                        margin-bottom: 2px;
                    }
                """)
            else:
                item_widget.setStyleSheet("""
                    QFrame {
                        background-color: #2b2b2b;
                        border: 1px solid #3c3f41;
                        border-radius: 4px;
                        margin-bottom: 2px;
                    }
                    QFrame:hover {
                        background-color: #353535;
                        border-color: #4b6eaf;
                    }
                """)
            
            v_layout = QVBoxLayout(item_widget)
            v_layout.setContentsMargins(4, 4, 4, 4)
            v_layout.setSpacing(4)
            
            # Top row: ID and Delete button
            top_row = QHBoxLayout()
            top_row.setContentsMargins(0, 0, 0, 0)
            
            lbl_id = QLabel(f"<b>#{s['id']}</b>")
            lbl_id.setStyleSheet("color: #4CAF50; font-size: 11px;")
            
            btn_del = QPushButton("×")
            btn_del.setFixedSize(16, 16)
            btn_del.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    color: #777;
                    border: none;
                    font-size: 14px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    color: #ef5350;
                }
            """)
            btn_del.clicked.connect(lambda checked, sid=s["id"]: self.delete_sample(sid))
            
            top_row.addWidget(lbl_id)
            top_row.addStretch()
            top_row.addWidget(btn_del)
            v_layout.addLayout(top_row)
            
            # Thumbnail label
            lbl_thumb = QLabel()
            lbl_thumb.setFixedSize(100, 62)
            lbl_thumb.setAlignment(Qt.AlignCenter)
            lbl_thumb.setStyleSheet("background-color: #000; border-radius: 2px;")
            
            img_path = os.path.join(self.save_dir, s["image_file"]) if self.save_dir else ""
            if img_path and os.path.exists(img_path):
                pix = QPixmap(img_path)
                if not pix.isNull():
                    lbl_thumb.setPixmap(pix.scaled(100, 62, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                else:
                    lbl_thumb.setText("Error")
            else:
                lbl_thumb.setText("No Img")
                lbl_thumb.setStyleSheet("color: #666; font-size: 10px; background-color: #111;")
            
            v_layout.addWidget(lbl_thumb, 0, Qt.AlignCenter)
            
            # Coordinates label
            pose = s["robot_pose"]
            lbl_info = QLabel()
            lbl_info.setAlignment(Qt.AlignCenter)
            lbl_info.setStyleSheet("color: #a9b7c6; font-family: monospace; font-size: 9px; line-height: 110%;")
            lbl_info.setText(
                f"X: {pose['x']:.1f}<br/>"
                f"Y: {pose['y']:.1f}<br/>"
                f"Z: {pose['z']:.1f}<br/>"
                f"A: {pose['a']:.3f}<br/>"
                f"B: {pose['b']:.3f}<br/>"
                f"C: {pose['c']:.3f}"
            )
            v_layout.addWidget(lbl_info)
            
            # Click connection
            item_widget.clicked.connect(lambda sid=s["id"]: self.handle_sample_select(sid))
            
            self.samples_layout.addWidget(item_widget)
            
        self.samples_layout.addStretch()

    def handle_sample_select(self, sid):
        if self.selected_sample_id == sid:
            self.selected_sample_id = None
            self.preview_image = None
        else:
            self.selected_sample_id = sid
            # Cache the loaded preview image once
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
            # If the deleted sample was the selected one, reset selection
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
