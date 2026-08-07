# -*- coding: utf-8 -*-
from datetime import datetime
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QGroupBox, QFormLayout, QGridLayout, 
                             QDoubleSpinBox, QProgressBar, QTextEdit, QScrollArea, QFrame,
                             QSpinBox, QSlider)
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

        # Safe Position Group
        safe_group = QGroupBox("Safe Position")
        safe_grid = QGridLayout(safe_group)
        
        self.spin_safe_x = QDoubleSpinBox()
        self.spin_safe_x.setRange(-2500.0, 2500.0)
        self.spin_safe_x.setValue(500.0)
        self.spin_safe_x.setSuffix(" mm")
        self.spin_safe_x.setDecimals(2)
        
        self.spin_safe_y = QDoubleSpinBox()
        self.spin_safe_y.setRange(-2500.0, 2500.0)
        self.spin_safe_y.setValue(-300.0)
        self.spin_safe_y.setSuffix(" mm")
        self.spin_safe_y.setDecimals(2)
        
        self.spin_safe_z = QDoubleSpinBox()
        self.spin_safe_z.setRange(-2500.0, 2500.0)
        self.spin_safe_z.setValue(1250.0)
        self.spin_safe_z.setSuffix(" mm")
        self.spin_safe_z.setDecimals(2)
        
        self.spin_safe_a = QDoubleSpinBox()
        self.spin_safe_a.setRange(-360.0, 360.0)
        self.spin_safe_a.setValue(180.0)
        self.spin_safe_a.setSuffix(" °")
        self.spin_safe_a.setDecimals(2)
        
        self.spin_safe_b = QDoubleSpinBox()
        self.spin_safe_b.setRange(-360.0, 360.0)
        self.spin_safe_b.setValue(80.0)
        self.spin_safe_b.setSuffix(" °")
        self.spin_safe_b.setDecimals(2)
        
        self.spin_safe_c = QDoubleSpinBox()
        self.spin_safe_c.setRange(-360.0, 360.0)
        self.spin_safe_c.setValue(0.0)
        self.spin_safe_c.setSuffix(" °")
        self.spin_safe_c.setDecimals(2)
        
        safe_grid.addWidget(QLabel("X:"), 0, 0)
        safe_grid.addWidget(self.spin_safe_x, 0, 1)
        safe_grid.addWidget(QLabel("A:"), 0, 2)
        safe_grid.addWidget(self.spin_safe_a, 0, 3)
        
        safe_grid.addWidget(QLabel("Y:"), 1, 0)
        safe_grid.addWidget(self.spin_safe_y, 1, 1)
        safe_grid.addWidget(QLabel("B:"), 1, 2)
        safe_grid.addWidget(self.spin_safe_b, 1, 3)
        
        safe_grid.addWidget(QLabel("Z:"), 2, 0)
        safe_grid.addWidget(self.spin_safe_z, 2, 1)
        safe_grid.addWidget(QLabel("C:"), 2, 2)
        safe_grid.addWidget(self.spin_safe_c, 2, 3)
        
        # Predefined positions movement row inside Safe Position group box
        move_btn_layout = QHBoxLayout()
        self.btn_go_home = QPushButton("Go Home (Move to Home)")
        self.btn_go_home.clicked.connect(self.main_win.robot_go_home)
        
        self.btn_move_safe = QPushButton("Go to Safe Position")
        self.btn_move_safe.clicked.connect(self.main_win.move_to_safe_pose)
        
        move_btn_layout.addWidget(self.btn_go_home)
        move_btn_layout.addWidget(self.btn_move_safe)
        safe_grid.addLayout(move_btn_layout, 3, 0, 1, 4)
        
        right_layout.addWidget(safe_group)

        # Action panel
        action_group = QGroupBox("Verification Move")
        action_vbox = QVBoxLayout(action_group)
        
        # Offset and Tool Number row
        config_row_layout = QHBoxLayout()
        config_row_layout.addWidget(QLabel("Normal Offset:"))
        self.spin_ver_offset = QDoubleSpinBox()
        self.spin_ver_offset.setRange(-500.0, 500.0)
        self.spin_ver_offset.setValue(200.0)
        self.spin_ver_offset.setSuffix(" mm")
        config_row_layout.addWidget(self.spin_ver_offset)
        
        config_row_layout.addSpacing(15)
        
        config_row_layout.addWidget(QLabel("Tool Number:"))
        self.spin_tool_num = QSpinBox()
        self.spin_tool_num.setRange(0, 15)
        self.spin_tool_num.setValue(0)
        self.spin_tool_num.valueChanged.connect(self.main_win.change_robot_tool_number)
        config_row_layout.addWidget(self.spin_tool_num)
        action_vbox.addLayout(config_row_layout)

        # Sliders grid layout for aligned starting points
        sliders_grid = QGridLayout()
        sliders_grid.setColumnStretch(0, 0)
        sliders_grid.setColumnStretch(1, 1)
        sliders_grid.setColumnStretch(2, 0)
        
        # Row 0: Global Speed
        lbl_global_speed = QLabel("Global Speed:")
        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(1, 100)
        self.slider_speed.setValue(40)
        self.lbl_speed_val = QLabel("40 %")
        self.lbl_speed_val.setFixedWidth(65)
        
        def on_speed_changed(val):
            self.lbl_speed_val.setText(f"{val} %")
            self.main_win.change_robot_speed(val)
            
        self.slider_speed.valueChanged.connect(on_speed_changed)
        
        sliders_grid.addWidget(lbl_global_speed, 0, 0)
        sliders_grid.addWidget(self.slider_speed, 0, 1)
        sliders_grid.addWidget(self.lbl_speed_val, 0, 2)
        
        # Row 1: MOVL Speed
        lbl_movl_speed = QLabel("MOVL Speed:")
        self.slider_movl_speed = QSlider(Qt.Horizontal)
        self.slider_movl_speed.setRange(10, 1000)
        self.slider_movl_speed.setValue(100)
        self.lbl_movl_speed_val = QLabel("100 mm/s")
        self.lbl_movl_speed_val.setFixedWidth(65)
        self.slider_movl_speed.valueChanged.connect(lambda val: self.lbl_movl_speed_val.setText(f"{val} mm/s"))
        
        sliders_grid.addWidget(lbl_movl_speed, 1, 0)
        sliders_grid.addWidget(self.slider_movl_speed, 1, 1)
        sliders_grid.addWidget(self.lbl_movl_speed_val, 1, 2)
        
        # Row 2: Acc
        lbl_acc = QLabel("Acc:")
        self.slider_acc = QSlider(Qt.Horizontal)
        self.slider_acc.setRange(1, 100)
        self.slider_acc.setValue(80)
        self.lbl_acc_val = QLabel("80 %")
        self.lbl_acc_val.setFixedWidth(65)
        self.slider_acc.valueChanged.connect(lambda val: self.lbl_acc_val.setText(f"{val} %"))
        
        sliders_grid.addWidget(lbl_acc, 2, 0)
        sliders_grid.addWidget(self.slider_acc, 2, 1)
        sliders_grid.addWidget(self.lbl_acc_val, 2, 2)
        
        # Row 3: Dec
        lbl_dec = QLabel("Dec:")
        self.slider_dec = QSlider(Qt.Horizontal)
        self.slider_dec.setRange(1, 100)
        self.slider_dec.setValue(80)
        self.lbl_dec_val = QLabel("80 %")
        self.lbl_dec_val.setFixedWidth(65)
        self.slider_dec.valueChanged.connect(lambda val: self.lbl_dec_val.setText(f"{val} %"))
        
        sliders_grid.addWidget(lbl_dec, 3, 0)
        sliders_grid.addWidget(self.slider_dec, 3, 1)
        sliders_grid.addWidget(self.lbl_dec_val, 3, 2)
        
        # Row 4: CP Ratio
        lbl_cp = QLabel("CP Ratio:")
        self.slider_cp = QSlider(Qt.Horizontal)
        self.slider_cp.setRange(0, 100)
        self.slider_cp.setValue(50)
        self.lbl_cp_val = QLabel("50 %")
        self.lbl_cp_val.setFixedWidth(65)
        self.slider_cp.valueChanged.connect(lambda val: self.lbl_cp_val.setText(f"{val} %"))
        
        sliders_grid.addWidget(lbl_cp, 4, 0)
        sliders_grid.addWidget(self.slider_cp, 4, 1)
        sliders_grid.addWidget(self.lbl_cp_val, 4, 2)
        
        action_vbox.addLayout(sliders_grid)

        # Draw segment/point & verification route execution controls (2x2 Grid)
        verify_buttons_grid = QGridLayout()
        
        self.btn_clear_last = QPushButton("Clear Last Point")
        self.btn_clear_last.clicked.connect(self.main_win.clear_last_point)
        
        self.btn_clear_items = QPushButton("Clear All Draw Items")
        self.btn_clear_items.clicked.connect(self.main_win.clear_draw_items)
        
        self.btn_finish_line = QPushButton("Finish Current Segment / Point")
        self.btn_finish_line.clicked.connect(lambda: self.main_win.finish_current_draw_item(warn_if_empty=True))
        
        self.btn_ver_move = QPushButton("Move Robot (Execute Route)")
        self.btn_ver_move.setEnabled(False)
        self.btn_ver_move.clicked.connect(self.main_win.move_to_verification_pose)
        
        verify_buttons_grid.addWidget(self.btn_clear_last, 0, 0)
        verify_buttons_grid.addWidget(self.btn_clear_items, 0, 1)
        verify_buttons_grid.addWidget(self.btn_finish_line, 1, 0)
        verify_buttons_grid.addWidget(self.btn_ver_move, 1, 1)
        
        action_vbox.addLayout(verify_buttons_grid)



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
