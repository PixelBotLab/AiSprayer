# -*- coding: utf-8 -*-
from datetime import datetime
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QGroupBox, QFormLayout, QGridLayout, 
                             QDoubleSpinBox, QProgressBar, QTextEdit, QScrollArea, QFrame)
from PyQt5.QtCore import Qt
from aisprayer.tools.calib.calib_ui.ui.widgets import ClickableLabel

class VerifyTab(QWidget):
    def __init__(self, main_win):
        super().__init__(main_win)
        self.main_win = main_win
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        # Top area: Video/Left and Config/Right
        top_layout = QHBoxLayout()

        # Left Column: Clickable Video View
        left_layout = QVBoxLayout()
        self.ver_video = ClickableLabel()
        self.ver_video.clicked_pos.connect(self.main_win.handle_verify_click)
        self.ver_video.right_clicked.connect(lambda: self.main_win.finish_current_draw_item(warn_if_empty=False))
        self.ver_video.mouse_moved.connect(self.main_win.handle_verify_hover)
        left_layout.addWidget(self.ver_video)
        
        top_layout.addLayout(left_layout, 2)

        # Right Column: Scroll Area for Controls
        self.controls_scroll = QScrollArea()
        self.controls_scroll.setWidgetResizable(True)
        self.controls_scroll.setFrameShape(QFrame.NoFrame)
        self.controls_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        controls_widget = QWidget()
        right_layout = QVBoxLayout(controls_widget)
        right_layout.setContentsMargins(0, 0, 5, 0)
        right_layout.setSpacing(10)

        # Load file
        file_group = QGroupBox("Load Calibration")
        file_vbox = QVBoxLayout(file_group)
        self.lbl_calib_path = QLabel("Calibration File: Not Loaded")
        self.lbl_calib_path.setWordWrap(True)
        self.btn_load_calib = QPushButton("Load Calibration Result (.yaml)")
        self.btn_load_calib.clicked.connect(self.main_win.select_calib_file)
        file_vbox.addWidget(self.btn_load_calib)
        file_vbox.addWidget(self.lbl_calib_path)
        file_group.setLayout(file_vbox)
        right_layout.addWidget(file_group)

        # Coordinates info
        coord_group = QGroupBox("Target Coordinates")
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
        pose_group = QGroupBox("Robot Pose")
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
            lbl.setStyleSheet("font-family: monospace; font-weight: bold;")
            
        pose_grid.addWidget(self.lbl_real_x, 0, 0)
        pose_grid.addWidget(self.lbl_real_y, 0, 1)
        pose_grid.addWidget(self.lbl_real_z, 0, 2)
        pose_grid.addWidget(self.lbl_real_a, 1, 0)
        pose_grid.addWidget(self.lbl_real_b, 1, 1)
        pose_grid.addWidget(self.lbl_real_c, 1, 2)
        right_layout.addWidget(pose_group)

        # Action panel
        action_group = QGroupBox("Verification Move")
        action_vbox = QVBoxLayout(action_group)
        
        offset_layout = QHBoxLayout()
        offset_layout.addWidget(QLabel("Normal Offset (mm):"))
        self.spin_ver_offset = QDoubleSpinBox()
        self.spin_ver_offset.setRange(-500.0, 500.0)
        self.spin_ver_offset.setValue(100.0)
        offset_layout.addWidget(self.spin_ver_offset)
        action_vbox.addLayout(offset_layout)

        # Draw segment/point controls
        self.btn_finish_line = QPushButton("Finish Current Segment / Point")
        self.btn_finish_line.clicked.connect(lambda: self.main_win.finish_current_draw_item(warn_if_empty=True))
        action_vbox.addWidget(self.btn_finish_line)

        self.btn_clear_last = QPushButton("Clear Last Point")
        self.btn_clear_last.clicked.connect(self.main_win.clear_last_point)
        action_vbox.addWidget(self.btn_clear_last)

        self.btn_clear_items = QPushButton("Clear All Draw Items")
        self.btn_clear_items.clicked.connect(self.main_win.clear_draw_items)
        action_vbox.addWidget(self.btn_clear_items)

        self.btn_ver_move = QPushButton("Move Robot (Execute Route)")
        self.btn_ver_move.setEnabled(False)
        self.btn_ver_move.clicked.connect(self.main_win.move_to_verification_pose)
        action_vbox.addWidget(self.btn_ver_move)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setVisible(False)
        action_vbox.addWidget(self.progress_bar)
        action_group.setLayout(action_vbox)
        
        right_layout.addWidget(action_group)
        right_layout.addStretch()
        
        self.controls_scroll.setWidget(controls_widget)
        
        top_layout.addWidget(self.controls_scroll, 1)
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
