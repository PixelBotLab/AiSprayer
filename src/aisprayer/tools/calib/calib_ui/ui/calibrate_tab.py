# -*- coding: utf-8 -*-
import os
from datetime import datetime
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QLineEdit, QGroupBox, QDoubleSpinBox, 
                             QTextEdit, QGridLayout, QSizePolicy, QScrollArea, 
                             QFrame, QMessageBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from aisprayer.tools.calib.calib_ui.ui.widgets import ClickableFrame, AspectLabel

class CalibrateTab(QWidget):
    def __init__(self, main_win):
        super().__init__(main_win)
        self.main_win = main_win
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        # Top area: Video (left) and Controls (right)
        top_layout = QHBoxLayout()

        # Left Column: Camera View
        left_layout = QVBoxLayout()
        self.cap_video = AspectLabel()
        self.cap_video.setMinimumSize(400, 225)
        self.cap_video.setAlignment(Qt.AlignCenter)
        self.cap_video.setStyleSheet("background-color: #000; border: 2px solid #222;")
        left_layout.addWidget(self.cap_video)
        
        top_layout.addLayout(left_layout, 2)

        # Middle Column: Captured Samples list
        self.samples_group = QGroupBox("Samples")
        self.samples_group.setFixedWidth(380)
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
        
        top_layout.addWidget(self.samples_group, 0)

        # Right Column: Scroll Area for Controls
        self.controls_scroll = QScrollArea()
        self.controls_scroll.setWidgetResizable(True)
        self.controls_scroll.setFrameShape(QFrame.NoFrame)
        self.controls_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        controls_widget = QWidget()
        right_layout = QVBoxLayout(controls_widget)
        right_layout.setContentsMargins(0, 0, 5, 0)
        right_layout.setSpacing(10)

        # 0. Camera Status Group (above Robot Connection)
        self.cam_group = QGroupBox("Camera: Disconnected")
        self.cam_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")
        cam_layout = QHBoxLayout(self.cam_group)
        cam_layout.setContentsMargins(5, 5, 5, 5)
        self.btn_reconnect_cam = QPushButton("Retry Camera")
        self.btn_reconnect_cam.setFixedWidth(300)
        self.btn_reconnect_cam.clicked.connect(self.main_win.start_camera)

        cam_layout.addWidget(self.btn_reconnect_cam)
        cam_layout.addStretch()
        right_layout.addWidget(self.cam_group)

        # 1. Connection Group
        self.conn_group = QGroupBox("Robot: Disconnected")
        self.conn_group.setStyleSheet("QGroupBox::title { color: #f44336; font-weight: bold; }")
        conn_layout = QHBoxLayout(self.conn_group)
        conn_layout.setContentsMargins(5, 5, 5, 5)
        conn_layout.setSpacing(5)
        self.txt_ip = QLineEdit(self.main_win.default_ip)
        self.txt_ip.setFixedWidth(300)
        self.txt_port = QLineEdit(self.main_win.default_port)
        self.txt_port.setFixedWidth(100)
        self.btn_connect_robot = QPushButton("Connect Robot")
        self.btn_connect_robot.clicked.connect(self.main_win.toggle_robot_connection)

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
        cap_group = QGroupBox("Calib Directory")
        cap_vbox = QVBoxLayout(cap_group)
        cap_vbox.setContentsMargins(5, 5, 5, 5)

        # Select Dir Row
        dir_layout = QHBoxLayout()
        self.btn_sel_dir = QPushButton("Select Calib Dir")
        self.btn_sel_dir.clicked.connect(self.main_win.select_save_dir)
        self.lbl_save_dir = QLabel("Not Selected")
        self.lbl_save_dir.setWordWrap(True)
        dir_layout.addWidget(self.btn_sel_dir)
        dir_layout.addWidget(self.lbl_save_dir, 1)
        cap_vbox.addLayout(dir_layout)

        right_layout.addWidget(cap_group)

        # 3. Jogging & Actions Group
        jog_group = QGroupBox("Robot Jogging")
        jog_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        jog_layout = QVBoxLayout(jog_group)
        jog_layout.setContentsMargins(5, 5, 5, 5)

        # Coordinates grid (X, Y, Z, A, B, C)
        jog_grid = QGridLayout()
        jog_grid.setColumnStretch(0, 0)
        jog_grid.setColumnStretch(1, 3)
        jog_grid.setColumnStretch(2, 1)
        jog_grid.setColumnStretch(3, 1)
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
            
            btn_minus.clicked.connect(lambda checked, a=axis: self.main_win.jog_step(a, -1))
            btn_plus.clicked.connect(lambda checked, a=axis: self.main_win.jog_step(a, 1))

            jog_grid.addWidget(lbl, idx, 0)
            jog_grid.addWidget(spin, idx, 1)
            jog_grid.addWidget(btn_minus, idx, 2)
            jog_grid.addWidget(btn_plus, idx, 3)

        # Step size selectors
        step_layout = QHBoxLayout()
        step_layout.addWidget(QLabel("Step XYZ (mm):"))
        self.spin_step_xyz = QDoubleSpinBox()
        self.spin_step_xyz.setRange(0.1, 200.0)
        self.spin_step_xyz.setValue(50.0)
        step_layout.addWidget(self.spin_step_xyz)

        step_layout.addWidget(QLabel("Step ABC (°):"))
        self.spin_step_abc = QDoubleSpinBox()
        self.spin_step_abc.setRange(0.1, 45.0)
        self.spin_step_abc.setValue(10.0)
        step_layout.addWidget(self.spin_step_abc)
        
        jog_grid.addLayout(step_layout, 6, 0, 1, 4)
        jog_layout.addLayout(jog_grid)
        jog_layout.addStretch()

        # 2x2 grid of buttons
        buttons_grid = QGridLayout()
        
        self.btn_capture = QPushButton("Capture")
        self.btn_capture.clicked.connect(self.main_win.capture_sample)
        
        self.btn_run_calib = QPushButton("Calibrate")
        self.btn_run_calib.clicked.connect(self.main_win.run_calibration)
        
        self.btn_read_pos = QPushButton("Read Current Pose")
        self.btn_read_pos.clicked.connect(self.main_win.read_robot_pose)
        
        self.btn_move_to_jog = QPushButton("Move to Target Pose")
        self.btn_move_to_jog.clicked.connect(self.main_win.move_to_jog_pose)

        buttons_grid.addWidget(self.btn_read_pos, 0, 0)
        buttons_grid.addWidget(self.btn_move_to_jog, 0, 1)
        buttons_grid.addWidget(self.btn_capture, 1, 0)
        buttons_grid.addWidget(self.btn_run_calib, 1, 1)
        jog_layout.addLayout(buttons_grid)
        
        right_layout.addWidget(jog_group)
        
        self.controls_scroll.setWidget(controls_widget)
        
        top_layout.addWidget(self.controls_scroll, 1)
        main_layout.addLayout(top_layout, 0)

        # Bottom Area: txt_calib_log (spanning entire width)
        self.txt_calib_log = QTextEdit()
        self.txt_calib_log.setReadOnly(True)
        self.txt_calib_log.setLineWrapMode(QTextEdit.NoWrap)
        
        # Patch append to prepend datetime
        _orig_append = self.txt_calib_log.append
        def log_append(text):
            _orig_append(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {text}")
            self.txt_calib_log.horizontalScrollBar().setValue(0)
        self.txt_calib_log.append = log_append
        
        main_layout.addWidget(self.txt_calib_log, 1)

    def refresh_samples_list(self):
        # Clear existing layout items
        while self.samples_layout.count():
            child = self.samples_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        if not self.main_win.cap_info or "samples" not in self.main_win.cap_info:
            return
            
        samples = self.main_win.cap_info["samples"]
        self.samples_group.setTitle(f"Samples ({len(samples)})")
        
        for s in samples:
            item_widget = ClickableFrame()
            item_widget.setFrameShape(QFrame.StyledPanel)
            
            is_selected = (self.main_win.selected_sample_id == s["id"])
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
            lbl_id.setStyleSheet("color: #4CAF50;")
            
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
            btn_del.clicked.connect(lambda checked, sid=s["id"]: self.main_win.delete_sample(sid))
            
            top_row.addWidget(lbl_id)
            top_row.addStretch()
            top_row.addWidget(btn_del)
            v_layout.addLayout(top_row)
            
            # Thumbnail label
            lbl_thumb = QLabel()
            lbl_thumb.setFixedSize(320, 240)
            lbl_thumb.setAlignment(Qt.AlignCenter)
            lbl_thumb.setStyleSheet("background-color: #000; border-radius: 2px;")
            
            img_path = os.path.join(self.main_win.save_dir, s["image_file"]) if self.main_win.save_dir else ""
            if img_path and os.path.exists(img_path):
                pix = QPixmap(img_path)
                if not pix.isNull():
                    lbl_thumb.setPixmap(pix.scaled(320, 240, Qt.KeepAspectRatio, Qt.SmoothTransformation))
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
            lbl_info.setStyleSheet("color: #a9b7c6; font-family: monospace; font-size: 12px; line-height: 120%;")
            lbl_info.setText(
                f"X:{pose['x']:.1f} Y:{pose['y']:.1f} Z:{pose['z']:.1f}<br/>"
                f"A:{pose['a']:.3f} B:{pose['b']:.3f} C:{pose['c']:.3f}"
            )
            v_layout.addWidget(lbl_info)
            
            # Click connection
            item_widget.clicked.connect(lambda sid=s["id"]: self.main_win.handle_sample_select(sid))
            
            self.samples_layout.addWidget(item_widget)
            
        self.samples_layout.addStretch()
