# test1_en.py
from PyQt5.QtWidgets import QWidget, QPushButton, QTextEdit, QLabel, QLineEdit, QFormLayout
from PyQt5.QtCore import pyqtSignal, Qt


class WidgetApp(QWidget):

    set_edit_text_signal = pyqtSignal(str)  # Signal to update QTextEdit text
    del_lineedit_signal = pyqtSignal()

    def __init__(self, socketFd):
        super().__init__()
        self.socketFd = socketFd
        self.initUI()

        self.set_edit_text_signal.connect(self.set_label_text)
        self.del_lineedit_signal.connect(self.clear_edit_text)

    def initUI(self):
        self.setWindowTitle("Upper Computer")
        self.resize(1050, 600)

        self.Text_status = QLabel('Servo Stopped', self)
        self.Text_status.setGeometry(20, 20, 120, 30)
        self.Text_status.setAlignment(Qt.AlignCenter)

        self.Global = QPushButton('Test Button', self)
        self.Global.setGeometry(20, 70, 130, 30)
        self.Global.clicked.connect(lambda: self.set_globe())

        self.poweron = QPushButton('Power ON', self)
        self.poweron.setGeometry(170, 70, 80, 30)
        self.poweron.clicked.connect(lambda: self.power_ctr('on'))

        self.poweroff = QPushButton('Power OFF', self)
        self.poweroff.setGeometry(260, 70, 80, 30)
        self.poweroff.clicked.connect(lambda: self.power_ctr('off'))

        self.clear_error1 = QPushButton('Clear Error', self)
        self.clear_error1.setGeometry(350, 70, 100, 30)
        self.clear_error1.clicked.connect(lambda: self.clear_error())

        self.test_demo = QPushButton('Test', self)
        self.test_demo.setGeometry(460, 70, 80, 30)
        self.test_demo.clicked.connect(lambda: self.test())

        self.job_upload_directory = QPushButton('Upload Job File', self)
        self.job_upload_directory.setGeometry(550, 70, 130, 30)
        self.job_upload_directory.clicked.connect(lambda: self.job_upload_by_directory())

        self.job_run_times = QPushButton('Set Job Run Times', self)
        self.job_run_times.setGeometry(690, 70, 150, 30)
        self.job_run_times.clicked.connect(lambda: self.set_job_times())

        self.queue = QPushButton('Start Motion Queue', self)
        self.queue.setGeometry(20, 110, 150, 30)
        name_star = 'star'
        self.queue.clicked.connect(lambda: self.queue_cmd(name_star))

        self.send_queue_motion = QPushButton('Send Queue', self)
        self.send_queue_motion.setGeometry(180, 110, 150, 30)
        name_send = 'send'
        self.send_queue_motion.clicked.connect(lambda: self.queue_cmd(name_send))

        self.movj = QPushButton('MOVJ', self)
        self.movj.setGeometry(20, 150, 60, 30)
        name_movj = 'movj'
        self.movj.clicked.connect(lambda: self.queue_cmd(name_movj))

        self.movl = QPushButton('MOVL', self)
        self.movl.setGeometry(90, 150, 60, 30)
        name_movl = 'movl'
        self.movl.clicked.connect(lambda: self.queue_cmd(name_movl))

        # Right text box
        self.Edit = QTextEdit(self)
        self.Edit.setPlainText('')
        self.Edit.setGeometry(530, 350, 500, 230)

        # Bottom text box
        self.Edit1 = QTextEdit(self)
        self.Edit1.setPlainText('')
        self.Edit1.setGeometry(10, 350, 500, 230)

        # Coordinate input boxes
        form_widget = QWidget(self)
        form_widget.setGeometry(550, 150, 220, 180)
        form_layout = QFormLayout()
        form_widget.setLayout(form_layout)

        self.point1 = QLineEdit(self)
        self.point2 = QLineEdit(self)
        self.point3 = QLineEdit(self)
        self.point4 = QLineEdit(self)
        self.point5 = QLineEdit(self)
        self.point6 = QLineEdit(self)

        form_layout.addRow("Axis 1", self.point1)
        form_layout.addRow("Axis 2", self.point2)
        form_layout.addRow("Axis 3", self.point3)
        form_layout.addRow("Axis 4", self.point4)
        form_layout.addRow("Axis 5", self.point5)
        form_layout.addRow("Axis 6", self.point6)

        form_widget2 = QWidget(self)
        form_widget2.setGeometry(790, 150, 200, 180)

        form_layout2 = QFormLayout()
        form_widget2.setLayout(form_layout2)

        self.axis1 = QLineEdit(self)
        self.axis2 = QLineEdit(self)
        self.axis3 = QLineEdit(self)
        self.axis4 = QLineEdit(self)
        self.axis5 = QLineEdit(self)
        self.axis6 = QLineEdit(self)

        self.axis1.setText('0')
        self.axis2.setText('0')
        self.axis3.setText('0')
        self.axis4.setText('0')
        self.axis5.setText('0')
        self.axis6.setText('0')

        form_layout2.addRow(self.axis1)
        form_layout2.addRow(self.axis2)
        form_layout2.addRow(self.axis3)
        form_layout2.addRow(self.axis4)
        form_layout2.addRow(self.axis5)
        form_layout2.addRow(self.axis6)

    def set_globe(self):
        from main import set_globe
        set_globe(self.socketFd)

    def test(self):
        from main import test_7000
        test_7000(self.socketFd)

    def set_job_times(self):
        from main import set_job_times
        set_job_times(self.socketFd)

    def job_upload_by_directory(self):
        from main import job_upload_by_directory
        job_upload_by_directory(self.socketFd)

    def power_ctr(self, name):
        from main import power_ctr
        power_ctr(self.socketFd, name, self)

    def queue_cmd(self, name):
        from main import queue_cmd
        queue_cmd(self.socketFd, name, self)

    def update_edit_text(self, text):
        self.Edit.append(text)

    def clear_edit_text(self):
        self.Edit.clear()

    def clear_error(self):
        from main import clear_error
        clear_error(self.socketFd, self)

    def update_edit1_text(self, text):
        self.Edit1.append(text)

    def set_label_text(self, text):
        self.Text_status.setText(text)
        if text == 'Servo Ready':
            self.Text_status.setStyleSheet('background-color: yellow;')
        elif text == 'Servo Alarm':
            self.Text_status.setStyleSheet('background-color: red;')
        elif text == 'Servo Stopped':
            self.Text_status.setStyleSheet('background-color: rgba(0, 0, 0, 0);')
        elif text == 'Servo Running':
            self.Text_status.setStyleSheet('background-color: green;')

    def get_axis1_text(self): return self.axis1.text()
    def get_axis2_text(self): return self.axis2.text()
    def get_axis3_text(self): return self.axis3.text()
    def get_axis4_text(self): return self.axis4.text()
    def get_axis5_text(self): return self.axis5.text()
    def get_axis6_text(self): return self.axis6.text()

    def set_point1_text(self, num): self.point1.setText(str(num))
    def set_point2_text(self, num): self.point2.setText(str(num))
    def set_point3_text(self, num): self.point3.setText(str(num))
    def set_point4_text(self, num): self.point4.setText(str(num))
    def set_point5_text(self, num): self.point5.setText(str(num))
    def set_point6_text(self, num): self.point6.setText(str(num))
