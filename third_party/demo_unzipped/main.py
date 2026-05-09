import nrc_interface as aa
import sys
from PyQt5.QtWidgets import QApplication
from Widget import WidgetApp  # 从 test1.py 导入 WidgetApp 类
import time
from PyQt5.QtCore import pyqtSignal , QThread
import threading
import ctypes

Size = 0
pos = aa.VectorDouble()

def connect(ip, port):
    fd = aa.connect_robot(ip, port)
    print("初始化控制器ID: ", fd)
    return fd

def set_globe(socketFd):
    param = aa.RobotJointParam()
    status = aa.get_robot_joint_param(socketFd, 1, param)
    print('-------', param.reducRatio)
    dhparam = aa.RobotDHParam()
    status = aa.get_robot_dh_param(socketFd, dhparam)
    print('-------', dhparam.L1)


def clear_error(socketFd, window):
    aa.clear_error(socketFd)
    window.update_edit1_text('清错')

def power_ctr( socketFd, name, window):
    if name == 'on':
        aa.set_servo_poweron(socketFd)
        window.update_edit1_text('上使能')
    elif name == 'off':
        aa.set_servo_poweroff(socketFd)
        window.update_edit1_text('下使能')
    else:
        print('error')

def test(socketFd):
    print('------------------------------------')
    print('开启测试函数.....')
    pos1 = aa.VectorDouble()
    robot_pos = aa.VectorDouble()
    sync_pos  = aa.VectorDouble()
    axis_pos = [50,40,10,0,0,0,0]
    for value in axis_pos:
        pos1.append(value)  # 使用 append 方法逐个添加值
    cmd1 = aa.MoveCmd()
    cmd1.targetPosType = aa.PosType_data
    cmd1.targetPosValue = pos1
    param = aa.ToolParam()
    toolnum = 0
    toolnum = aa.get_tool_hand_number(socketFd, toolnum)
    print('toolnum:' , toolnum[1])
    toolnum = 1
    aa.get_tool_hand_param(socketFd, toolnum, param)
    print('x:' , param.X, "     Y:", param.Y)
    # test_secra_move(socketFd)
    # while True:
    #     aa.get_current_position(socketFd, 0, robot_pos)
    #     print('关节 pos:', list(robot_pos))
    #     time.sleep(1.5)
    #     aa.get_current_position(socketFd, 1, robot_pos)
    #     print('直角 pos:', list(robot_pos))
    #     time.sleep(1.5)
        # aa.get_current_extra_position(socketFd,sync_pos)
        # time.sleep(1.5)
        # print('sync_pos:', list(sync_pos))


def test_7000(socketFd):
    print('开启测试7000.....')
    socket_7000 =  aa.connect_robot("192.168.1.13", "7000")
    servomovepara = aa.ServoMovePara()
    pos = aa.VectorVectorDouble()
    time = aa.VectorDouble()
    time_pos = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550]
    for value in time_pos:
        time.append(value)
    axis_pos = [[0,0,0,0,0,0,0],
                [0,0,0,0,0,1,0],
                [0,0,0,0,0,2,0],
                [0,0,0,0,0,3,0],
                [0,0,0,0,0,4,0],
                [0,0,0,0,0,5,0],
                [0,0,0,0,0,6,0],
                [0,0,0,0,0,7,0],
                [0,0,0,0,0,8,0],
                [0,0,0,0,0,9,0],
                [0,0,0,0,0,10,0]]  
    
    for value in axis_pos:
        pos.append(value)
    for k in range(11):
        servomovepara.pos = pos
    for j in range(11):
        servomovepara.timeStamp = time
    servomovepara.runMode = 0
    servomovepara.clearBuffer = True
    servomovepara.targetMode = 1
    servomovepara.coord = 0
    servomovepara.size = 11
    result =  aa.servo_move(socket_7000, servomovepara)
    print("return " , result)
        
        
    




def test_secra_move(socketFd):
    position = [[459,-0.1,796,-3.14,0,0],
                [491,44,796,-3.14,0,0],
                [565,-27,796,-3.14,0,0],
                [563,23,796,-3.14,0,0]]
    pos1 = aa.VectorVectorDouble()
    for pos in position:
        pos1.append(pos)
    
    pos2 = aa.VectorDouble()
    axis = [0,0,0,0,0,0,0]
    for pos_1 in axis:
        pos2.append(pos_1)
    cmd1 = aa.MoveCmd()
    cmd1.targetPosType = aa.PosType_data
    cmd1.targetPosValue = pos2
    cmd1.velocity = 50
    cmd1.acc = 50
    cmd1.dec = 50
    cmd1.coord = 0
    # cmd1.configuration = 1
    aa.robot_movej(socketFd,cmd1)
    aa.robot_moves(socketFd, pos1, 200, 1, 50, 50)
    # aa.queue_motion_set_status(socketFd,True)
    # aa.queue_motion_push_back_moveJ(socketFd,cmd1)
    # aa.queue_motion_push_back_moveJ(socketFd,cmd1)
    # pos1.clear()
    # axis_pos = [50,40,10,0,0,0,0]
    # aa.queue_motion_push_back_moveJ(socketFd,cmd1)
    # aa.queue_motion_send_to_controller(socketFd, 4)         #2插入的指令的数量
    # print("1111")
    jug = 1
    while jug != 0:
        #status的类似是一个int类型，获取到状态之后status会变成一个列表，所有再次调用接口的时候需要重新定义status为int类型传入
        status = 1
        status = aa.get_robot_running_state(socketFd, status) 
        jug = status[1]
    time.sleep(0.5)


    
    


def set_job_times(socketFd):
    print('设置作业文件运行次数')
    aa.job_run_times(socketFd,0)


def job_upload_by_directory(socketFd):
    a = 0
    # while True:
    #     print('job_upload_by_file return : ' ,aa.job_upload_by_file(socketFd,'/home/ad/python上位机/ABC/teach.JBR'))
    #     # time.sleep(1)
    #     print('上传次数: ' , a)
    #     a += 1
    #     if a == 2000:
    #         break
    print('job_upload_by_file return : ' ,aa.job_upload_by_file(socketFd,'/home/ad/python上位机/ABC/teach.JBR'))
            



def job_download_by_directory(socketFd):
    aa.job_download_by_directory(socketFd,'/home/ad/python上位机/download')



def queue_cmd(socketFd, name, window):
    global Size
    global pos
    # 确保线程在主线程退出前完成
    if name == 'star':
        result = aa.queue_motion_set_status(socketFd, True)
    elif name == 'movj':
        cmd = get_pos(window)
        cmd.velocity = 80
        cmd.acc = 100
        cmd.dec = 100
        print('cmd: ', list(cmd.targetPosValue))
        result = aa.queue_motion_push_back_moveJ(socketFd, cmd)
        if result == 0:
            Size += 1
        else:
            print('指令插入失败')
        # 更新窗口的 QTextEdit 文本
        window.update_edit_text("movJ")
    elif name == 'movl':
        cmd = get_pos(window)
        cmd.velocity = 80
        cmd.acc = 100
        cmd.dec = 100
        print('cmd: ', list(cmd.targetPosValue))
        result = aa.queue_motion_push_back_moveL(socketFd, cmd)
        print(result)
        if result == 0:
            Size += 1
        else:
            print('指令插入失败')
        # 更新窗口的 QTextEdit 文本
        window.update_edit_text("movL")
    elif name == 'send':
        result =  aa.queue_motion_send_to_controller(socketFd, Size)
        print('运动队列长度: ', Size)
        time.sleep(0.5)
        Size = 0
        jug = 1
        while jug != 0:
            #status的类似是一个int类型，获取到状态之后status会变成一个列表，所有再次调用接口的时候需要重新定义status为int类型传入
            status = 1
            status = aa.get_robot_running_state(socketFd, status) 
            jug = status[1]
        time.sleep(0.5)
        window.clear_edit_text()


class get_status(QThread):
    update_label_signal = pyqtSignal(str)  # 定义一个信号用于更新 QLabel 文本

    def __init__(self, socketFd):
        super().__init__()
        self.socketFd = socketFd


    def run(self):
        while True:
            time.sleep(1)
            servo_status(self)
            get_position()




def servo_status(self):
    status = 0
    status = aa.get_servo_state(self.socketFd, status)
    #接口获取到的值返回的是一个列表，例如：这里的status[0]表示接口的返回值，status[1]表示当前的伺服状态
    judge = status[1]
    if judge == 0:
        self.update_label_signal.emit('伺服停止')
    elif judge == 1:
        self.update_label_signal.emit('伺服就绪')
    elif judge == 2:
        self.update_label_signal.emit('伺服报警')
    elif judge == 3:
        self.update_label_signal.emit('伺服运行')
    else:
        print('error retuen : ', judge)

            
def get_pos(window):
    # 从文本框获取值并转换为浮点数
    time.sleep(1.5)
    pos = aa.VectorDouble()
    axis = [
        float(window.get_axis1_text() or '0'),
        float(window.get_axis2_text() or '0'),
        float(window.get_axis3_text() or '0'),
        float(window.get_axis4_text() or '0'),
        float(window.get_axis5_text() or '0'),
        float(window.get_axis6_text() or '0')
    ]

    # 创建并填充 VectorDouble
    for value in axis:
        pos.append(value)  # 使用 append 方法逐个添加值

    cmd = aa.MoveCmd()
    cmd.targetPosType = aa.PosType_data
    cmd.targetPosValue = pos
    print('cmd:', list(cmd.targetPosValue))
    
    return cmd

def get_position():
    pos = aa.VectorDouble()
    coord = 0  # 获取关节坐标
    aa.get_current_position(socketFd, coord, pos)
    
    # 将 pos 中的值分别设置到相应的文本框
    for i in range(6):
        getattr(window, f'set_point{i+1}_text')(pos[i])

def python_callback(msg_id, msg):
    print("recv: ", msg_id, msg)

def error_or_warning_message_handler(msg_type,msg,msg_code):
    print(f"Cte Msg:Type: {msg_type}, Code: {msg_code}, Msg: {msg}")

if __name__ == "__main__":
    socketFd = connect("192.168.1.13", "6001")
    if socketFd > 0:
        # aa.recv_message(socketFd, python_callback)
        # time.sleep(1)
        # result =  aa.set_receive_error_or_warnning_message_callback(socketFd, error_or_warning_message_handler)
        # print("set_receive_error_or_warnning_message_callback return : ", result)
        # 创建应用程序实例
        app = QApplication(sys.argv)

        # 创建并显示窗口
        window = WidgetApp(socketFd)
        window.show()

        #创建线程获取伺服状态
        thread = get_status(socketFd)
        thread.update_label_signal.connect(window.set_label_text)  # 连接信号与槽函数
        thread.start()

        # 进入应用程序主循环
        sys.exit(app.exec_())
        
    #等待线程结束
    thread.join()
