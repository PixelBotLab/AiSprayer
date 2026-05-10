import yaml
import time
import logging
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from aisprayer.core.hardware.robot.inexbot_driver2 import InexbotDriver, RobotPose

logger = logging.getLogger(__name__)

class RobotTrajectoryController:
    """
    AiSprayer 机器人轨迹控制器 (模块化重构版)
    支持单次连接执行多个规划任务。
    """
    def __init__(self, ip: str, port: str = "6001", ready_dist_mm: float = 300.0, tool_num: int = 0):
        """
        :param ip: 机器人控制器 IP
        :param port: 机器人控制器端口
        :param ready_dist_mm: 准备位相对于首个喷涂点的后撤距离 (mm)
        :param tool_num: 使用的工具编号
        """
        self.ready_dist = float(ready_dist_mm)
        self.tool_num = int(tool_num)
        self.robot_ip = ip
        self.robot_port = str(port)
        
        # 初始化机器人驱动
        self.robot = InexbotDriver(ip=self.robot_ip, port=self.robot_port, toolnum=self.tool_num)
        self.home_pose = None
        
        logger.info(f"[Controller] 初始化。IP: {self.robot_ip}:{self.robot_port}, 工具号: {self.tool_num}, 准备位后撤: {self.ready_dist}mm")

    def startup(self):
        """开启机器人连接并使能伺服，并自动回到 Home"""
        logger.info(">>> 正在启动机器人并回到 Home...")
        if self.robot.startup():
            self.robot.go_home(wait=True)
            self.home_pose = self.robot.get_current_pose()
            logger.info(f"已到达 Home: {self.home_pose}")
            time.sleep(3)
            return True
        return False

    def shutdown(self):
        """回到 Home 后断开机器人连接"""
        logger.info(">>> 正在执行收尾归位并关闭连接...")
        try:
            self.robot.go_home(wait=True)
            time.sleep(3)
        except Exception as e:
            logger.error(f"归位失败: {e}")
        self.robot.shutdown()

    def go_avoidance(self):
        """移动到避让位置 (Avoidance)"""
        if self.home_pose is None:
            # 如果没去过 Home，先获取当前位姿作为姿态参考
            self.home_pose = self.robot.get_current_pose()
        
        avoid_pose = RobotPose(x=500, y=200, z=self.home_pose.z, 
                               a=self.home_pose.a, b=self.home_pose.b, c=self.home_pose.c)
        logger.info(f">>> 移动到避让位置: {avoid_pose}")
        self.robot.move_j(avoid_pose, wait=True)
        time.sleep(3)

    def _get_normal_base(self, abc):
        """根据姿态计算基座坐标系下的后撤法向"""
        rot = R.from_euler('XYZ', abc, degrees=False)
        rot_mat = rot.as_matrix()
        return -rot_mat[:, 2]

    def execute(self, plan_path: str, velocity: float = 150.0, pl: int = 0):
        """
        执行单个规划文件的喷涂任务 (准备位 -> 喷涂)
        :param plan_path: plan.yaml 文件路径
        :param velocity: 喷涂时的直线运动速度 (mm/s)
        :param pl: 平滑参数 (0-5, 0为精确到位)
        """
        if not os.path.exists(plan_path):
            logger.error(f"找不到规划文件: {plan_path}")
            return False

        logger.info(f"[*] 开始执行规划任务: {plan_path}")
        with open(plan_path, 'r') as f:
            plan_data = yaml.safe_load(f)
        
        columns = plan_data.get("columns", [])
        if not columns:
            logger.error("规划文件中没有发现有效的纵列数据")
            return False

        # --- 步骤 1: 准备位 (Ready) ---
        first_pt = columns[0][0]
        first_pos = np.array(first_pt["pos"])
        first_abc = np.array(first_pt["abc"])
        
        normal = self._get_normal_base(first_abc)
        ready_pos = first_pos + normal * self.ready_dist
        ready_pose = RobotPose.from_list(ready_pos.tolist() + first_abc.tolist())
        
        logger.info(f">>> 移动到任务准备位: {ready_pose}")
        self.robot.move_j(ready_pose, wait=True)
        logger.info("准备位等待 5s...")
        time.sleep(5)

        # --- 步骤 2: 队列喷涂 ---
        logger.info(">>> 开始队列喷涂...")
        for i, col in enumerate(columns):
            logger.info(f"--- 纵列 {i+1}/{len(columns)} ---")
            
            self.robot.queue_start()
            for j, pt in enumerate(col):
                pose = RobotPose.from_list(pt["pos"] + pt["abc"])
                logger.info(f"  -> Queuing point {j+1}: {pose}")
                self.robot.queue_push_l(pose, velocity=velocity, pl=pl)
            
            self.robot.queue_send(wait=True)
            
            if i < len(columns) - 1:
                logger.info("纵列切换停顿 1s...")
                time.sleep(1)
        
        logger.info(f"[OK] 规划任务执行完毕: {plan_path}")
        return True