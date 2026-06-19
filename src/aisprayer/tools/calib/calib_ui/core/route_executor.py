# -*- coding: utf-8 -*-
import time
import numpy as np
from aisprayer.core.hardware.robot.inexbot_driver import RobotPose

class VerificationRouteExecutor:
    """
    非阻塞式机器人验证路径运动执行状态机及进度管理器。
    
    职责：
      1. 负责管理画线/打点验证路线的行进状态（idle/moving）。
      2. 展平由用户绘制的多段折线路径点，生成顺规的目标点列表。
      3. 在下发指令前进行软件层面的安全工作空间边界（workspace_limits）检验。
      4. 基于笛卡尔空间的累计实际行进距离，动态估算并返回逼真的运动进度百分比。
    """
    def __init__(self, robot, config):
        self.robot = robot
        self.config = config
        self.state = "idle"  # 状态机当前状态: "idle"（空闲） 或 "moving"（执行路径中）
        self.verify_items = []  # 待验证的线段与单点列表
        self.exec_item_index = 0  # 当前正在执行的线/点 ID 索引
        self.exec_waypoint_index = 0  # 当前线段中正在执行的目标路点索引
        self.last_move_cmd_time = 0  # 上一次下发非阻塞运动指令的系统时间戳
        self.total_waypoints = 0  # 所有验证项目的总路点数
        self.completed_waypoints = 0  # 已完成的路点数
        
        # 路径进度物理距离跟踪变量
        self.flat_path_points = []  # 包含起点在内的展平后三维点序列
        self.waypoint_flat_map = {}  # 映射元组 (item_idx, waypoint_idx) -> flat_path_points 中的一维索引
        self.segment_lengths = []  # 每一段折线段的物理距离 (mm)
        self.cumulative_distances = [0.0]  # 折线累积总路程序列，例如 [0.0, 50.0, 120.0, 200.0]
        self.total_path_distance = 0.0  # 全程总路线笛卡尔空间位移距离 (mm)

    def start(self, verify_items, offset, current_pose):
        """
        开始路径轨迹验证。
        参数：
          verify_items: 由 UI 绘制的待跑路径集合
          offset: 法向量上的垂直安全偏移高度 (mm)
          current_pose: 当前机器人的实时位姿（用于作为运动起点）
        """
        self.verify_items = verify_items
        for item in self.verify_items:
            item["status"] = "pending"

        self.state = "moving"
        self.exec_item_index = 0
        self.exec_waypoint_index = 0
        self.last_move_cmd_time = 0
        self.total_waypoints = sum(len(item["pose_data"]) for item in self.verify_items)
        self.completed_waypoints = 0

        # 1. 展平折线路径以支持距离积分计算进度
        self.flat_path_points = []
        self.waypoint_flat_map = {}
        
        # 将机器人当前物理位置作为路径的第 0 个点
        if current_pose:
            p0 = np.array([current_pose.x, current_pose.y, current_pose.z])
        else:
            p0 = np.array([0.0, 0.0, 0.0])
        self.flat_path_points.append(p0)
        
        flat_idx = 1
        for i_idx, item in enumerate(self.verify_items):
            for w_idx, pose_info in enumerate(item["pose_data"]):
                # 目标点 = 物理表面三维位置 + 法线方向 * 偏置距离
                p_dest = pose_info["p_base"] + pose_info["n_base"] * offset
                self.flat_path_points.append(p_dest)
                # 记录二维多级索引到展平列表一维索引的映射
                self.waypoint_flat_map[(i_idx, w_idx)] = flat_idx
                flat_idx += 1
                
        # 2. 积分计算各段长度及总路程
        self.segment_lengths = []
        self.cumulative_distances = [0.0]
        for i in range(1, len(self.flat_path_points)):
            dist = np.linalg.norm(self.flat_path_points[i] - self.flat_path_points[i-1])
            self.segment_lengths.append(dist)
            self.cumulative_distances.append(self.cumulative_distances[-1] + dist)
            
        self.total_path_distance = self.cumulative_distances[-1]

    def stop(self):
        """停止/强退当前路线验证运动"""
        self.state = "idle"

    def is_moving(self):
        """当前是否正在运行运动"""
        return self.state == "moving"

    def check_safety_limit(self, p_dest):
        """
        检查目标三维点是否超出设定的工作空间安全界限。
        参数：
          p_dest: 基座系下的 [X, Y, Z] 目标坐标 (mm)
        返回：
          (bool, err_msg): 是否安全及错误说明
        """
        planner_cfg = self.config.get("vision", {}).get("planner", {})
        lim = planner_cfg.get("workspace_limits", {})
        if lim:
            for axis, idx in [("x", 0), ("y", 1), ("z", 2)]:
                val = p_dest[idx]
                limits = lim.get(axis, [-9999, 9999])
                if not (limits[0] <= val <= limits[1]):
                    return False, f"Target destination {axis.upper()} ({val:.1f}) out of workspace limits {limits}."
        return True, ""

    def get_progress_percent(self, curr_pose):
        """
        基于当前机器人末端实际物理坐标，精准计算整条折线轨迹的行进进度百分比。
        设计思路：
          - 获取机器人当前的 [X, Y, Z] 位置。
          - 确定当前行进的折线段索引 flat_idx。
          - 提取该段起点的累积已走完距离 d_completed_prior。
          - 估计当前线段内的进度：段长 s_len 减去到该段终点的剩余距离 d_to_target。
          - 积分求和并归一化输出百分比 (0~100) 以及累计行进毫米数。
        """
        if not self.is_moving() or self.total_path_distance <= 0:
            return 0, 0.0
        p_curr = np.array([curr_pose.x, curr_pose.y, curr_pose.z])
        # 当前正在朝向的目标点对应的一维索引
        flat_idx = self.waypoint_flat_map.get((self.exec_item_index, self.exec_waypoint_index - 1))
        if flat_idx is not None and flat_idx < len(self.flat_path_points):
            p_target = self.flat_path_points[flat_idx]
            s_len = self.segment_lengths[flat_idx - 1]
            d_completed_prior = self.cumulative_distances[flat_idx - 1]
            
            # 计算到线段终点的欧氏距离
            d_to_target = np.linalg.norm(p_target - p_curr)
            if s_len > 0:
                # 容错处理：确保线段内进度在 0 到段长 s_len 之间
                d_seg_progress = max(0.0, min(s_len, s_len - d_to_target))
            else:
                d_seg_progress = 0.0
                
            d_completed = d_completed_prior + d_seg_progress
            percent = int((d_completed / self.total_path_distance) * 100)
            return max(0, min(100, percent)), d_completed
        return 0, 0.0
