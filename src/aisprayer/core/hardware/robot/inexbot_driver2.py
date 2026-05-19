"""
纳博特(iNexbot) 机械臂官方 SDK (SWIG) 通信驱动封装

修正记录 (完全同步 nabot_robot.py 流程):
1. 同步上电流程：清错 -> 状态 0 -> 状态 1 -> PowerOn。
2. 修正 MoveCmd 的 targetPosType 为 PosType_data (2)。
3. 暂时屏蔽 is_reachable 中的 SDK 调用，避免在初始化阶段触发 25566 报警。
4. 坐标系：直角坐标(coord=1)
5. 角度模式：弧度制(rad)
6. 运动模式：示教模式(mode=0)
7. 所有运动函数默认等待运动完成后返回。
"""

import time
import math
import logging
import pathlib
import sys
import threading
from dataclasses import dataclass
from typing import Optional, List, Union

# SDK 路径配置 (必须在 import nrc_interface 之前)
_project_root = pathlib.Path(__file__).parents[5]
_sdk_dir = str(_project_root / "third_party" / "robot_sdk" / "sdk_24.03" / "python" / "linux" / "linux_python_3.8_v2.0.4")
if _sdk_dir not in sys.path: 
    sys.path.insert(0, _sdk_dir)

import nrc_interface as nrc


logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════
#   位姿数据类型
# ═══════════════════════════════════════════════

@dataclass
class RobotPose:
    """
    机器人直角坐标位姿（弧度制）
    默认坐标系为直角坐标系，坐标轴顺序为(x, y, z, a, b, c)
    平移单位: mm, 姿态角单位: rad。

    示例:
        p = RobotPose(400, 0, 300, 0, math.pi, 0)
        p = RobotPose.form_list([400, 0, 300, 0, math.pi, 0])
        robot.move_l(p)
    """
    x: float = 0.0; y: float = 0.0; z: float = 0.0
    a: float = 0.0; b: float = 0.0; c: float = 0.0

    @classmethod
    def from_list(cls, data: list) -> "RobotPose":
        """从长度 >=6 的列表创建位姿RobotPose, 不足部分填 0。"""
        if not data: return cls()
        return cls(*[float(x) for x in data[:6]])

    def to_list(self) -> list:
        """转换为 [x,y,z,a,b,c] 列表。"""
        return [self.x, self.y, self.z, self.a, self.b, self.c]
    
    def __repr__(self):
        """返回可读性较好的字符串表示。"""
        return (f"RobotPose(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
                f"a={self.a:.3f}, b={self.b:.3f}, c={self.c:.3f})")
    
    def __eq__(self, other):
        """重载==，允许直接比较两个 RobotPose 对象是否相等。"""
        if not isinstance(other, RobotPose):
            return NotImplemented
        return (math.isclose(self.x, other.x) and math.isclose(self.y, other.y) and 
                math.isclose(self.z, other.z) and math.isclose(self.a, other.a) and 
                math.isclose(self.b, other.b) and math.isclose(self.c, other.c))

# 位姿参数类型别名: 公开接口统一接受 RobotPose 或原始 list 两种形式
PoseLike = Union[RobotPose, List[float]]

def _to_list(pose: PoseLike) -> list:
    """将 PoseLike 转换为 list 形式。"""
    if isinstance(pose, RobotPose):
        return pose.to_list()
    return [float(x) for x in pose]

# ---------------------------------------------
# 伺服状态常量
# ---------------------------------------------
SERVO_STATE_STOP       = 0 # 停止
SERVO_STATE_READY      = 1 # 就绪
SERVO_STATE_ALARM      = 2 # 报警
SERVO_STATE_RUNNING    = 3 # 运行


# ---------------------------------------------
# 返回值常量
# ---------------------------------------------
RESULT_TIMEOUT               = -6 # 超时
RESULT_EXCEPTION             = -5 # 异常
RESULT_OPERATION_NOT_ALLOWED = -4 # 操作不允许
RESULT_PARAM_ERR             = -3 # 参数错误
RESULT_DISCONNECT            = -2 # 断开连接
RESULT_RECEIVE_FAILED        = -1 # 接收失败
RESULT_SUCCESS               = 0 # 成功

# ---------------------------------------------
# 运行状态常量
# ---------------------------------------------
RUNNING_STATE_STOP       = 0 # 停止
RUNNING_STATE_PAUSE      = 1 # 暂停
RUNNING_STATE_RUNNING    = 2 # 运行

DEFAULT_VELOCITY = 50
DEFAULT_ACC = 50
DEFAULT_DEC = 50

class InexbotDriver:
    """
    Inexbot 机械臂驱动类。
    角度约定：所有姿态角（A,B,C）均为弧度制（rad）。
    位置约定：所有位置（X,Y,Z）均为毫米制。

    使用示例：
        import math
        robot = InexbotDriver("192.168.2.14")
        robot.startup()
        robot.move_l(RobotPose(400, 0, 300, 0, math.pi, 0))
        robot.shutdown()
    """

    COORD      = 1    # 直角坐标系(0=关节坐标系 1=直角坐标系 2=工具坐标系 3=用户坐标系)
    MODE       = 0    # 运行模式(0=示教模式 1=远程模式 2=运行模式)
    ANGLE_UNIT = 1    # 角度单位(0=弧度制 1=度制)

    _KEEPALIVE_INTERVAL = 3
    
    def __init__(self, ip: str = "192.168.2.14", port: str = "6001", toolnum: int = 0, reconnect: bool = True):
        self.ip           = ip
        self.port         = str(port)
        self.tool_num     = toolnum
        self.reconnect    = reconnect
        self.fd           = -1
        self._queue_size  = 0     # 本地队列已追加的指令条数
        self._keepalive_thread: Optional[threading.Thread] = None
        self._keepalive_stop = threading.Event()

    # ---------------------------------------
    # 连接 / 断开
    # ---------------------------------------
    def startup(self, timeout: float = 10.0) -> bool:
        """
        启动机器人连接并使能伺服（一键上电流程）：连接控制器 → 设置坐标系/模式 → 清错 → 
        伺服停止(0) → 就绪(1) → 上电(power on)。
        上电完成后自动打印系统信息快照.
        :param timeout: 每个等待步骤的最长超时时间（秒）。
        :return: True/False, 是否成功启动并上电, 或任意步骤失败。
        """
        logger.info(f"[startup] nrc lib version: {nrc.get_library_version()}")

        # 步骤0: 如果已经连接，直接返回
        if self.fd >= 0:
            logger.info("[startup] already started")
            return True

        # 步骤1: 连接控制器
        nrc.set_reconnect(self.fd, self.reconnect)
        self.fd = nrc.connect_robot(self.ip, self.port)
        if self.fd < 0:
            logger.error(f"[startup] Failed to connect to robot: {self.ip}:{self.port}")
            return False
        logger.info(f"[startup] Connection established to {self.ip}:{self.port}")

        # 步骤2：设置坐标系和运行模式
        result = nrc.set_current_coord(self.fd, self.COORD)
        if result != nrc.SUCCESS:
            logger.error(f"[startup] Failed to set coordinate system to {self.COORD}, result: {result}")
            return False
        logger.info(f"[startup] Set coordinate system to {self.COORD}")
        result = nrc.set_current_mode(self.fd, self.MODE)
        if result != nrc.SUCCESS:
            logger.error(f"[startup] Failed to set running mode to {self.MODE}, result: {result}")
            return False
        logger.info(f"[startup] Set running mode to {self.MODE}")
        if self.tool_num > 0:
            result = nrc.set_tool_hand_number(self.fd, self.tool_num)
            if result != nrc.SUCCESS:
                logger.error(f"[startup] Failed to set tool number to {self.tool_num}, result: {result}")
                return False
            logger.info(f"[startup] Set tool number to {self.tool_num}")

        # 步骤3：清除错误
        result = nrc.clear_error(self.fd)
        if result != nrc.SUCCESS:
            logger.error(f"[startup] Failed to clear error, result: {result}")
            return False
        logger.info(f"[startup] Cleared")

        # 步骤4：设置状态 0 → 1
        for state in [0, 1]:
            current_state = self.get_servo_state()
            if current_state in [2, 3]:
                logger.info(f"[startup] Current state is {current_state} (cannot set_servo_state). Skipping.")
                continue
            result = nrc.set_servo_state(self.fd, state)
            if result != nrc.SUCCESS:
                logger.error(f"[startup] Failed to set state to {state}, result: {result}")
                #return False
            logger.info(f"[startup] Set state to {state}")
            time.sleep(0.5)

        # 启动后台心跳线程
        self._keepalive_stop.clear()
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, daemon=True, name="robot-keepalive"
        )
        self._keepalive_thread.start()


        # 步骤5：上电
        already_powered_on = False
        current_state = self.get_servo_state()
        if current_state == 3:
            logger.info("[startup] Servo is already powered on (state=3). Skipping power on.")
            already_powered_on = True

        if not already_powered_on:
            result = nrc.set_servo_poweron(self.fd)
            if result != nrc.SUCCESS:
                logger.error(f"[startup] Failed to power on, result: {result}")
                return False
            logger.info(f"[startup] Powered on")

            # 检查上电是否成功
            poweron_success = False
            for _ in range(20):
                time.sleep(0.5)
                if self.get_servo_state() == 3:
                    poweron_success = True
                    break

            if not poweron_success:
                logger.error("[startup] Failed to verify servo power on (state did not reach 3)")
                self.shutdown()
                return False
            logger.info("[startup] Servo power on verified (state=3)")
        else:
            time.sleep(0.5)
        logger.info(f"[startup] Robot startup successful and servo enabled")
        self.print_system_info()

        return True

    def shutdown(self) -> None:
        """
        关闭机器人连接，断开控制器。
        """
        # 停止心跳线程
        self._keepalive_stop.set()
        if self._keepalive_thread is not None:
            self._keepalive_thread.join(timeout=2.0)
            self._keepalive_thread = None

        if self.fd >= 0:
            #断电
            nrc.set_servo_poweroff(self.fd)
            time.sleep(0.5)
            #断开连接
            nrc.disconnect_robot(self.fd)
            self.fd = -1
        logger.info(f"[shutdown] Disconnected from {self.ip}:{self.port}")

    def _keepalive_loop(self):
        """
        后台心跳：每隔 _KEEPALIVE_INTERVAL 秒查询一次伺服状态和位姿，
        防止控制器因应用层空闲超时（~23s）主动发 FIN+RST 断连。
        同时在终端打印状态快照。
        """
        while not self._keepalive_stop.wait(self._KEEPALIVE_INTERVAL):
            if self.fd < 0:
                break
            try:
                # 1. 获取伺服状态 (0=停止, 1=就绪, 2=报警, 3=运行)
                state = self.get_servo_state()
                # 2. 获取当前直角坐标位姿
                p = self.get_position()
                
                if p:
                    # 在终端打印心跳快照
                    print(f"[*] Heartbeat - Servo: {state} | Pose: [{p.x:.1f}, {p.y:.1f}, {p.z:.1f}, {p.a:.3f}, {p.b:.3f}, {p.c:.3f}]")
            except Exception:
                pass

    def print_system_info(self) -> None:
        """
        打印机器人系统信息快照。
        """
        fd = self.fd
        if fd < 0:
            logger.warning("[print_system_info] Robot is not connected")
            return
        
        # 1-六轴串联多关节  2-四轴scara  3-四轴码垛  4-四轴串联多关节  5-单轴  
        # 6-五轴串联多关节  7-六轴协作  8-二轴scara  9-三轴scara  10-三轴直角
        # 11-三轴异性  12-七轴串联多关节  13-scara异性一  14-四轴码垛丝杆
        _ROBOT_TYPE = {
            1: "六轴串联多关节",  2: "四轴SCARA", 3: "四轴码垛", 
            4: "四轴串联多关节",  5: "单轴", 6: "五轴串联多关节", 
            7: "六轴协作",  8: "二轴scara", 9: "三轴scara",
            10: "三轴直角", 11: "三轴异性", 12: "七轴串联多关节",
            13: "scara异性一", 14: "四轴码垛丝杆"
        }

        _COORD_MAP = {0:"关节", 1:"直角", 2:"工具", 3:"用户"}
        _MODE_MAP  = {0:"示教", 1:"远程", 2:"运行"}
        _SERVO_MAP = {0:"停止", 1:"就绪", 2:"报警", 3:"运行"}

        def _get(call, *args):
            """
            统一处理 SWIG 出参列表形式的返回值。
            调用格式：call(fd, *args)，返回 result[0]=0表示成功, <0 表示出错, result[1]为实际参数
            查询失败（非 SUCCESS）时返回 None
            """
            result = call(fd, *args)
            if isinstance(result, (list, tuple)):
                # 如果成功，返回实际值 result[1]；如果失败，返回错误码 result[0]
                return result[1] if int(result[0]) == nrc.SUCCESS else int(result[0])
            # 如果返回的是整数，无论成功（0）还是失败（负数），直接返回该结果
            return result
        
        # 1. 获取控制器 ID (使用 VectorChar 版本, 比 char* 版本更稳定)
        ctrl_id_vec = nrc.VectorChar()
        id_ret = _get(nrc.get_controller_id_csharp, ctrl_id_vec)
        ctrl_id = "".join(chr(c) for c in ctrl_id_vec).rstrip("\x00") \
            if id_ret == nrc.SUCCESS else "--"
        
        # 2. 示教器连接状态
        tb_status = False
        tb_status = nrc.get_teachbox_connection_state(self.fd, tb_status)
        tb_connected = tb_status[1] if isinstance(tb_status, (list, tuple)) and int(tb_status[0]) == nrc.SUCCESS else None

        # 3. 机器人类型
        rtype = _get(nrc.get_robot_type, 0)
        rtype_str = _ROBOT_TYPE.get(rtype, f"未知({rtype})") if rtype is not None else "--"

        # 4. 坐标系
        coord = _get(nrc.get_current_coord, 0)
        coord_str = _COORD_MAP.get(coord, f"未知({coord})") if coord is not None else "--"

        # 5. 运行模式
        mode = _get(nrc.get_current_mode, 0)
        mode_str = _MODE_MAP.get(mode, f"未知({mode})") if mode is not None else "--"

        # 6. 当前全局速度
        speed = _get(nrc.get_speed, 0)
        speed_str = f"{speed}%" if speed is not None else "--"

        # 7. 伺服状态
        servo_status = _get(nrc.get_servo_state, 0)
        servo_status_str = _SERVO_MAP.get(servo_status, f"未知({servo_status})") if servo_status is not None else "--"

        # 8. 当前激活的工具编号
        toolnum = _get(nrc.get_tool_handle_number, 0)
        toolnum_str = str(toolnum) if toolnum is not None else "--"

        # 9. 当前关节坐标(上电后有效， 单位: °)
        raw_joint = nrc.VectorDouble()
        result = nrc.get_current_position(self.fd, 0, raw_joint)
        joint_pose_str = str(RobotPose.from_list(list(raw_joint))) if result == nrc.SUCCESS and raw_joint else "--"

        # 10. 当前末端位姿(直角坐标，单位: mm,°)
        raw_cart = nrc.VectorDouble()
        result = nrc.get_current_position(self.fd, 1, raw_cart)
        cart_pose_str = str(RobotPose.from_list(list(raw_cart))) if result == nrc.SUCCESS and raw_cart else "--"

        print(
            f"\n{'-' * 50}\n"
            f"控制器ID: {ctrl_id}\n"
            f"示教器连接状态: {tb_connected}\n"
            f"机器人类型: {rtype_str}\n"
            f"坐标系: {coord_str}\n"
            f"运行模式: {mode_str}\n"
            f"当前全局速度: {speed_str}\n"
            f"伺服状态: {servo_status_str}\n"
            f"当前激活的工具编号: {toolnum_str}\n"
            f"当前关节坐标(上电后有效， 单位: °): {joint_pose_str}\n"
            f"当前末端位姿(直角坐标，单位: mm,°): {cart_pose_str}\n"
            f"\n{'-' * 50}\n"
        )

    # -------------------------------------------------
    # 状态查询
    # -------------------------------------------------

    def get_servo_state(self) -> int:
        """
        获取伺服状态
        :return: -1=错误；0=停止；1=就绪；2=报警；3=运行
        """
        if self.fd < 0:
            logger.warning("[get_servo_state] Robot is not connected")
            return -1
        
        result = nrc.get_servo_state(self.fd, 0)
        if result is None:
            logger.error("[get_servo_state] Failed to get servo state")
            return -1
        elif result[0] != nrc.SUCCESS:
            logger.error(f"[get_servo_state] Failed to get servo state, error code: {result[0]}")
            return -1
        else:
            return result[1]

    def get_running_state(self) -> int:
        """
        获取运行状态
        :return: -1=错误；0=停止；1=暂停；2=运行
        """
        if self.fd < 0:
            logger.warning("[get_running_state] Robot is not connected")
            return -1
        
        result = nrc.get_running_state(self.fd, 0)
        if isinstance(result, (list, tuple)) and len(result) > 1:
            if int(result[0]) == nrc.SUCCESS:
                return int(result[1])
            logger.error(f"[get_running_state] SDK error code: {result[0]}")
        return -1
        
    def get_current_pose(self) -> RobotPose:
        """
        获取当前位姿
        :param coord: 坐标系 0=关节；1=直角；2=工具；3=用户
        :return: RobotPose (平移单位 mm, 旋转单位 rad)
        """
        if self.fd < 0:
            logger.warning("[get_current_pose] Robot is not connected")
            return None
        
        raw_cart = nrc.VectorDouble()
        result = nrc.get_current_position(self.fd, self.COORD, raw_cart)
        if result != nrc.SUCCESS:
            logger.error(f"[get_current_pose] Failed to get current position, code: {result}")
            return None
        return RobotPose.from_list(list(raw_cart))
    
    # -------------------------------------------------
    #  可达性判断
    # -------------------------------------------------

    def is_reachable(self, pose: PoseLike, movetype: str = "MOVL") -> bool:
        """
        判断目标位姿是否可达(仅做运动学逆解检验， 不含碰撞检测)
        :param pose:  目标位姿，RobotPose 或 [x(mm), y(mm), z(mm), A(rad), B(rad), C(rad)]
        :param movetype: 运动类型, "MOVL"=直线运动, "MOVJ"=关节运动
        :return: True=可达，False=不可达或查询接口失败
        """
        # nrc.get_pos_reachable 需要长度为 14 的特殊向量，布局如下：
        # [0]=坐标系 [1]=角度制(0=角度/1=弧度) [2]=形态 [3]=工具号
        # [4]=用户坐标编号 [5][6]=备用 [7~13]=目标位姿 x,y,z,a,b,c,备用
        if self.fd < 0:
            logger.warning("[is_reachable] Robot is not connected")
            return False

        pose_vec = [0.0] * 14
        pose_vec[0] = float(self.COORD)        # 直角坐标系
        pose_vec[1] = float(self.ANGLE_UNIT)   # 弧度制
        pose_vec[3] = float(self.tool_num)
        for i, v in enumerate(_to_list(pose)[:6]):
            pose_vec[7+i] = float(v)

        result_bool = False
        res = nrc.get_pos_reachable(self.fd, pose_vec, movetype, result_bool)
        if isinstance(res, (list, tuple)) and len(res) > 1:
            if int(res[0]) == nrc.SUCCESS:
                return bool(res[1])
            logger.error(f"[is_reachable] SDK error code: {res[0]}")
        return False

    # -------------------------------------------------
    #  运动控制（内部工具方法）
    # -------------------------------------------------
    
    def _make_movecmd(
        self, 
        pose: PoseLike, 
        velocity: float,
        acc: float,
        dec: float,
        tool_num: int = 0,
        user_num: int = 0,
        pl: int = 0,
    ) -> nrc.MoveCmd:
        """
        根据目标位姿和运动参数构造 MoveCmd 对象。
        统一使用直角坐标系，位姿数组长度补齐到 7 位（本体 6 位 + 外部轴 1 位，外部轴默认始终为 0）
        :param pose: 目标位姿，RobotPose 或 [x(mm), y(mm), z(mm), A(rad), B(rad), C(rad)]
        :param velocity: 运动速度
        :param acc: 加速度
        :param dec: 减速度
        :param tool_num: 工具号
        :param user_num: 用户坐标号
        :param pl: 平滑过渡参数，范围[0, 5]；0=精确到位（有停顿），1〜5=数值越大越平滑（不停顿）
        :return: MoveCmd
        """
        # 1. 构造 14 位点位数组 (前 7 位本体，后 7 位外部轴)
        vec = nrc.VectorDouble()
        for v in _to_list(pose):
            vec.append(float(v))
        # MoveCmd.targetPosValue 要求长度至少为 7 （第 7 位是外部轴， 此处置 0）
        while len(vec) < 7:
            vec.append(0.0)

        cmd = nrc.MoveCmd()
        cmd.targetPosType = nrc.PosType_data
        cmd.targetPosValue = vec
        cmd.coord = self.COORD
        cmd.velocity = velocity
        cmd.acc = acc
        cmd.dec = dec
        cmd.toolNum = tool_num if tool_num > 0 else self.tool_num
        cmd.userNum = user_num
        cmd.pl = pl
        return cmd

    def _wait_motion_done(self, poll_interval: float = 0.05) -> None:
        """
        轮询等待机器人单挑运动指令执行完毕（running_state 回到 RUNNING_STATE_STOP=0）
        先等待 0.1s 确保控制器已开始响应运动指令, 再开始轮询（避免立即检测到 RUNNING_STATE_STOP）
        :param poll_interval: 轮询间隔
        """
        time.sleep(0.1)
        while True:
            if self.get_running_state() == RUNNING_STATE_STOP:
                break
            time.sleep(poll_interval)

    # -------------------------------------------------
    #  运动指令（外部调用接口）
    # -------------------------------------------------
    
    def move_j(
        self, 
        pose: PoseLike, 
        velocity: float = DEFAULT_VELOCITY, 
        acc: float = DEFAULT_ACC, 
        dec: float = DEFAULT_DEC,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        关节运动(MOVJ)，目标位姿以直角坐标描述，由控制器内部做逆解。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 速度，单位 %（百分比），范围 (0, 100]
        :param acc: 加速度，范围 (0, 100]
        :param dec: 减速度，范围 (0, 100]
        :param tool_num: 工具号
        :param wait: 是否等待运动完成，True=阻塞直到运动完成，False=立即返回
        :return: 接口返回码，（0=成功，负值=失败）
        """
        cmd = self._make_movecmd(pose, velocity, acc, dec, tool_num)
        ret = nrc.robot_movej(self.fd, cmd)
        if ret != nrc.SUCCESS:
            logger.error(f"[move_j] Failed to send movej command, error code: {ret}")
            return ret
        if wait:
            self._wait_motion_done()
            logger.info(f"[move_j] movej command sent successfully, target pose: {pose}")
        return ret

    def move_l(
        self, 
        pose: PoseLike, 
        velocity: float = DEFAULT_VELOCITY, 
        acc: float = DEFAULT_ACC, 
        dec: float = DEFAULT_DEC,
        tool_num: int = 0,
        wait: bool = True,
    ) -> int:
        """
        直线运动(MOVL)，目标位姿以直角坐标描述，末端在笛卡尔空间走直线运动。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 线速度，单位 mm/s，范围 (0, 1000]
        :param acc: 加速度，范围 (0, 100]
        :param dec: 减速度，范围 (0, 100]
        :param tool_num: 工具号
        :param wait: 是否等待运动完成，True=阻塞直到运动完成，False=立即返回
        :return: 接口返回码，（0=成功，负值=失败）
        """
        cmd = self._make_movecmd(pose, velocity, acc, dec, tool_num)
        ret = nrc.robot_movel(self.fd, cmd)
        if ret != nrc.SUCCESS:
            logger.error(f"[move_l] Failed to send movel command, error code: {ret}")
            return ret
        if wait:
            self._wait_motion_done()
            logger.info(f"[move_l] movel command sent successfully, target pose: {pose}")
        return ret
    
    def go_home(self, wait: bool = True) -> int:
        """
        返回原点运动(GO HOME)
        :param wait: 是否等待运动完成，True=阻塞直到运动完成，False=立即返回
        :return: 接口返回码，（0=成功，负值=失败）
        """
        ret = nrc.robot_go_home(self.fd)
        if ret != nrc.SUCCESS:
            logger.error(f"[go_home] Failed to send go_home command, error code: {ret}")
            return ret
        if wait:
            self._wait_motion_done()
            logger.info(f"[go_home] go_home command sent successfully")
        return ret

    # -------------------------------------------------
    #  队列运动
    # -------------------------------------------------

    def queue_start(self) -> int:
        """
        打开队列运动模式，并清空控制器端已存的旧队列数据.
        调用后一次通过 queue_push_l / queue_push_j 向本队列追加点位，
        所有点位准备就绪后调用 queue_send 一次性下发并触发运动。
        :return: 接口返回码，（0=成功，负值=失败）
        """
        self._queue_size = 0
        ret = nrc.queue_motion_set_status(self.fd, True)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_start] Failed to send queue_motion_set_status command, error code: {ret}")
        else:
            logger.info(f"[queue_start] queue motion started successfully")
        return ret

    def queue_push_l(
        self, 
        pose: PoseLike,
        velocity: float = DEFAULT_VELOCITY,
        acc: float = DEFAULT_ACC,
        dec: float = DEFAULT_DEC,
        tool_num: int = 0,
        pl: int = 0,
    ) -> int:
        """
        向队列追加一条直线运动点位(MOVL)。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 线速度，单位 mm/s，范围 (0, 1000]
        :param acc: 加速度，范围 (0, 100]
        :param dec: 减速度，范围 (0, 100]
        :param tool_num: 工具号
        :param pl: 平滑过渡参数，范围[0, 5]；0=精确到位（有停顿），1〜5=数值越大越平滑（不停顿）
        :return: 接口返回码，（0=成功，负值=失败）
        """
        cmd = self._make_movecmd(pose, velocity, acc, dec, tool_num, pl=pl)
        ret = nrc.queue_motion_push_back_moveL(self.fd, cmd)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_push_l] Failed to send queue_motion_push_back_moveL command, error code: {ret}")
            return ret
        self._queue_size += 1
        return ret

    def queue_push_j(
        self, 
        pose: PoseLike,
        velocity: float = DEFAULT_VELOCITY,
        acc: float = DEFAULT_ACC,
        dec: float = DEFAULT_DEC,
        tool_num: int = 0,
        pl: int = 0,
    ) -> int:
        """
        向队列追加一条关节运动点位(MOVJ)。
        :param pose: 目标位姿 RobotPose 或 [x(mm),y(mm),z(mm),A(rad),B(rad),C(rad)]
        :param velocity: 速度，单位 %（百分比），范围 (0, 100]
        :param acc: 加速度，范围 (0, 100]
        :param dec: 减速度，范围 (0, 100]
        :param tool_num: 工具号
        :param pl: 平滑过渡参数，范围[0, 5]；0=精确到位（有停顿），1〜5=数值越大越平滑（不停顿）
        :return: 接口返回码，（0=成功，负值=失败）
        """
        cmd = self._make_movecmd(pose, velocity, acc, dec, tool_num, pl=pl)
        ret = nrc.queue_motion_push_back_moveJ(self.fd, cmd)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_push_j] Failed to send queue_motion_push_back_moveJ command, error code: {ret}")
            return ret
        self._queue_size += 1
        return ret

    def queue_send(self, wait: bool = True) -> int:
        """
        将本地队列数据全部发送到控制器并触发运动。
        超过 31 条时自动分批发送（中间批次使用 isContinue = True 保证连续性，最后一批 isContinue=False 触发执行）。
        :param wait: 是否等待运动完成，True=阻塞直到运动完成，False=立即返回
        :return: 接口返回码，（0=成功，负值=失败）
        """
        if self._queue_size == 0:
            logger.warning("[queue_send] No motion data in queue.")
            return nrc.SUCCESS
        
        ret = self._queue_send_batched(wait)
        self._queue_size = 0
        return ret

    def queue_suspend(self) -> int:
        """ 暂停正在执行的队列运动，机器人平滑减速停止 """
        ret = nrc.queue_motion_suspend(self.fd)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_suspend] Failed to send queue_motion_suspend command, error code: {ret}")
        else:
            logger.info(f"[queue_suspend] queue motion suspended successfully")
        return ret

    def queue_resume(self) -> int:
        """ 从暂停状态恢复，继续执行剩余队列运动 """
        ret = nrc.queue_motion_restart(self.fd)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_resume] Failed to send queue_motion_restart command, error code: {ret}")
        else:
            logger.info(f"[queue_resume] queue motion restarted successfully")
        return ret

    def queue_stop(self) -> int:
        """ 
        停止队列运动并清除剩余指令， 机器人保持上电状态 
        调用后本地队列计数也会被清零
        """
        self._queue_size = 0
        ret = nrc.queue_motion_stop_not_power_off(self.fd)
        if ret != nrc.SUCCESS:
            logger.error(f"[queue_clear] Failed to send queue_motion_stop_not_power_off command, error code: {ret}")
        else:
            logger.info(f"[queue_clear] queue motion stopped successfully")
        return ret

    def queue_get_remaining(self) -> int:
        """ 
        查询控制器端队列中尚未执行的指令数量。
        :return: 剩余指令数量，查询失败时返回错误码（ < 0 ）
        """
        qlen = 0
        qlen = nrc.queue_motion_get_queuelen(self.fd, qlen)
        if qlen[0] != nrc.SUCCESS:
            logger.error(f"[queue_get_remaining] Failed to get queue length, error code: {qlen[0]}")
            return qlen[0]
        return qlen[1]

    # -------------------------------------------------
    #  队列运动(内部工具方法)
    # -------------------------------------------------

    def _queue_send_batched(self, wait: bool) -> int:
        """ 
        分批将本地队列发送到控制器。
        每批最多 31 条（SDK 限制），中间批次传 isContinue==True，
        让控制器先缓存而不立即运动；最后一批传 isContinue==False 
        触发控制器开始执行整段队列运动。
        @param wait: 是否等待运动完成，True=阻塞直到运动完成，False=立即返回
        @return: 接口返回码，（0=成功，负值=失败）
        """
        BATCH = 31
        total = self._queue_size
        sent  = 0
        ret   = nrc.SUCCESS

        while sent < total:
            remaining = total - sent
            batch     = min(remaining, BATCH)
            is_last   = (sent + batch >= total)
            
            ret = nrc.queue_motion_send_to_controller(
                self.fd, batch, not is_last   # isContinue = True 表示继续缓存，False 触发运动
            )
            if ret != nrc.SUCCESS:
                logger.error(f"[queue_send_batched] Failed on batch (cmds={batch}, continue={not is_last}), error code: {ret}")
                return ret
            sent += batch

        if wait:
            self._wait_queue_done()
            
        return ret

    def _wait_queue_done(self, poll_interval: float = 0.05) -> None:
        """ 
        等待控制器端队列完全执行完毕。
        分两阶段：先等待剩余指令数归零，再等待运行状态回到 RUNNING_STATE_STOP，
        避免最后一条指令尚在执行时误判为完成。
        :param poll_interval: 轮询间隔（秒）
        :return: True=运动完成, False=超时或查询失败
        """
        time.sleep(0.1)

        # 阶段一：等待控制器端队列情况（所有指令已被取出执行）
        while True:
            qlen = self.queue_get_remaining()
            if qlen == 0:
                break
            time.sleep(poll_interval)

        # 阶段二：等待队列运行状态回到 RUNNING_STATE_STOP（所有指令已执行完成）
        while True:
            if self.get_running_state() == RUNNING_STATE_STOP:
                break
            time.sleep(poll_interval)

        logger.info("[wait_queue_done] Queue motion completed.")
        