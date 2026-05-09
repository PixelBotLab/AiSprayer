# test1.py
from PyQt5.QtWidgets import QWidget, QPushButton, QTextEdit, QLabel, QListWidget, QLineEdit, QFormLayout
from PyQt5.QtCore import pyqtSignal, Qt

class WidgetApp(QWidget):

    set_edit_text_signal = pyqtSignal(str)  # 定义一个信号用于更新 QTextEdit 文本
    del_lineedit_signal = pyqtSignal()

    def __init__(self, socketFd):
        super().__init__()
        self.socketFd = socketFd
        self.initUI()

        self.set_edit_text_signal.connect(self.set_label_text)
        self.del_lineedit_signal.connect(self.clear_edit_text)

    def initUI(self):
        self.setWindowTitle("上位机")
        self.resize(1000, 600)

        self.Text_status = QLabel('伺服停止', self)
        self.Text_status.setGeometry(20, 20, 100, 30)
        self.Text_status.setAlignment(Qt.AlignCenter)

        self.Global = QPushButton('测试按钮', self)
        self.Global.setGeometry(20, 70, 125, 30)
        self.Global.clicked.connect(lambda: self.set_globe())

        self.poweron = QPushButton('上使能', self)
        self.poweron.setGeometry(170, 70, 50, 30)
        self.poweron.clicked.connect(lambda: self.power_ctr('on'))


        self.poweroff = QPushButton('下使能', self)
        self.poweroff.setGeometry(230, 70, 50, 30)
        self.poweroff.clicked.connect(lambda: self.power_ctr('off'))

        self.clear_error1 = QPushButton('清错', self)
        self.clear_error1.setGeometry(290, 70, 50, 30)
        self.clear_error1.clicked.connect(lambda: self.clear_error())

        self.test_demo = QPushButton('测试',self)
        self.test_demo.setGeometry(350, 70, 50, 30)
        self.test_demo.clicked.connect(lambda: self.test())

        self.job_upload_directory = QPushButton('上传作业文件',self)
        self.job_upload_directory.setGeometry(410, 70, 110, 30)
        self.job_upload_directory.clicked.connect(lambda: self.job_upload_by_directory())

        self.job_run_times = QPushButton('设置作业运行次数',self)
        self.job_run_times.setGeometry(530, 70, 110, 30)
        self.job_run_times.clicked.connect(lambda: self.set_job_times())


        self.queue = QPushButton('开始运动队列', self)
        self.queue.setGeometry(20, 110, 100, 30)
        name_star = 'star'
        self.queue.clicked.connect(lambda: self.queue_cmd(name_star))

        self.send_queue_motion = QPushButton('发送队列', self)
        self.send_queue_motion.setGeometry(120, 110, 100, 30)
        name_send = 'send'
        self.send_queue_motion.clicked.connect(lambda: self.queue_cmd(name_send))

        self.movj = QPushButton('movj', self)
        self.movj.setGeometry(20, 150, 50, 30)
        name_movj = 'movj'
        self.movj.clicked.connect(lambda: self.queue_cmd(name_movj))

        self.movl = QPushButton('movl', self)
        self.movl.setGeometry(70, 150, 50, 30)
        name_movl = 'movl'
        self.movl.clicked.connect(lambda: self.queue_cmd(name_movl))

        #右侧文本框
        self.Edit = QTextEdit(self)
        self.Edit.setPlainText('')
        self.Edit.setGeometry(500, 350, 500, 300)

        #下方文本框
        self.Edit1 = QTextEdit(self)
        self.Edit1.setPlainText('')
        self.Edit1.setGeometry(0,350,500,300)

        #坐标文本框
        form_widget = QWidget(self)
        form_widget.setGeometry(550, 150, 200, 300)  # 设置容器小部件的几何属性

        # 创建一个表单布局
        form_layout = QFormLayout()
        form_widget.setLayout(form_layout)

        # 创建文本框并添加到表单布局中
        self.point1 = QLineEdit(self)
        self.point2 = QLineEdit(self)
        self.point3 = QLineEdit(self)
        self.point4 = QLineEdit(self)
        self.point5 = QLineEdit(self)
        self.point6 = QLineEdit(self)  
        
        # 为每个文本框设置标签
        form_layout.addRow("1轴", self.point1)
        form_layout.addRow("2轴", self.point2)
        form_layout.addRow("3轴", self.point3)
        form_layout.addRow("4轴", self.point4)
        form_layout.addRow("5轴", self.point5)
        form_layout.addRow("6轴", self.point6)

        form_widget2 = QWidget(self)
        form_widget2.setGeometry(770, 150, 200, 300)

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

        form_layout2.addRow( self.axis1)
        form_layout2.addRow( self.axis2)
        form_layout2.addRow( self.axis3)
        form_layout2.addRow( self.axis4)
        form_layout2.addRow( self.axis5)
        form_layout2.addRow( self.axis6)

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
        # self.Edit.setPlainText(text)
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
        if text == '伺服就绪':
            self.Text_status.setStyleSheet('background-color: yellow;')
        elif text == '伺服报警':
            self.Text_status.setStyleSheet('background-color: red;')
        elif text == '伺服停止':
            self.Text_status.setStyleSheet('background-color:  rgba(0, 0, 0, 0);')
        elif text == '伺服运行':
            self.Text_status.setStyleSheet('background-color: green;')

    def get_axis1_text(self):
        return self.axis1.text()

    def get_axis2_text(self):
        return self.axis2.text()

    def get_axis3_text(self):
        return self.axis3.text()

    def get_axis4_text(self):
        return self.axis4.text()

    def get_axis5_text(self):
        return self.axis5.text()

    def get_axis6_text(self):
        return self.axis6.text()
    
    def set_point1_text(self, num):
        self.point1.setText(str(num))  

    def set_point2_text(self, num):
        self.point2.setText(str(num))

    def set_point3_text(self, num):
        self.point3.setText(str(num))

    def set_point4_text(self, num):
        self.point4.setText(str(num))

    def set_point5_text(self, num):
        self.point5.setText(str(num))

    def set_point6_text(self, num):
        self.point6.setText(str(num))
    
