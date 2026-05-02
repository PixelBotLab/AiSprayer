#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys, cv2, yaml, numpy as np, time
# 确保能找到项目根目录下的模块
sys.path.append(os.getcwd())

from camera.orbbec_driver import OrbbecDriver
from robot.inexbot_driver import InexbotDriver, RobotPose, MODE_RUN, MODE_TEACH
from scipy.spatial.transform import Rotation as R_tool_scipy

class Aligner:
    # 定义安全过渡位置 (Transition Point)
    SAFE_POSE_LIST = [460.0, 125.0, 1077.0, -3.141, 0.0, 0.0]
    DEFAULT_SPEED = 50  # 统一默认速度 (mm/s)
    
    # SDK 错误码映射
    RESULT_MAP = {
        0: "成功 (SUCCESS)",
        -1: "接收失败 (RECEIVE_FAILED)",
        -2: "连接断开 (DISCONNECT)",
        -3: "参数错误 (PARAM_ERR) - 请检查位姿是否在软限位内",
        -4: "操作被禁止 (OPERATION_NOT_ALLOWED) - 请检查是否上电或模式正确",
        -5: "异常 (EXCEPTION)",
        -6: "超时 (TIMEOUT)"
    }

    def __init__(self, robot_ip="192.168.2.14", calib_res_path="calib/data/calib_20260501/calibration_result.yaml"):
        # 加载全量标定结果 (包含外参、内参、标定板参数)
        if not os.path.exists(calib_res_path):
            raise FileNotFoundError(f"找不到标定结果文件: {calib_res_path}")
        with open(calib_res_path, 'r') as f:
            res = yaml.safe_load(f)
        
        self.T_base_camera = np.array(res["T_base_camera"])
        self.K = np.array(res["camera_params"]["intrinsic_matrix"])
        self.D = np.array(res["camera_params"]["distortion_coeffs"])
        self.pattern_size = tuple(res["board_params"]["pattern_size_inner"])
        self.sq_size = res["board_params"]["square_size_mm"]
        
        # 打印加载的信息
        print("\n" + "="*50)
        print(" [参数加载成功] ")
        print(f" 相机内参 K:\n{self.K}")
        print(f" 畸变系数 D: {self.D}")
        print(f" 标定板规格: {self.pattern_size}, 间距: {self.sq_size}mm")
        print(f" 手眼外参 T_base_camera:\n{self.T_base_camera}")
        print("="*50 + "\n")
        
        # 初始化驱动
        self.cam = OrbbecDriver()
        self.robot = InexbotDriver(ip=robot_ip)
        
        # 预计算
        self.objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2) * self.sq_size
        self.center_offset_obj = np.array([(self.pattern_size[0]-1)*self.sq_size/2, (self.pattern_size[1]-1)*self.sq_size/2, 0])

    def get_board_pose_in_base(self, curr_R, safe_offset=120.0):
        """识别标定板并返回其位姿"""
        color_img, _ = self.cam.get_frame()
        if color_img is None: return None
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
        if not ret: return None
        
        _, rvec, tvec = cv2.solvePnP(self.objp, corners, self.K, self.D)
        R_cam_board, _ = cv2.Rodrigues(rvec)
        
        # 核心修正：沿法线方向退让
        center_with_offset_obj = self.center_offset_obj + np.array([0, 0, -safe_offset])
        p_target_cam = R_cam_board @ center_with_offset_obj + tvec.flatten()
        p_target_base = self.T_base_camera[:3, :3] @ p_target_cam + self.T_base_camera[:3, 3]
        
        R_base_board = self.T_base_camera[:3, :3] @ R_cam_board
        R_flip = R_base_board @ np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        if np.trace(curr_R.T @ R_flip) > np.trace(curr_R.T @ R_base_board):
            R_base_board = R_flip
            
        abc_base = R_tool_scipy.from_matrix(R_base_board).as_euler('XYZ', degrees=False)
        return np.concatenate([p_target_base, abc_base])

    def check_move_result(self, res, stage_name):
        """检查并打印运动结果"""
        if res == 0:
            return True
        err_msg = self.RESULT_MAP.get(res, f"未知错误 ({res})")
        print(f"[-] {stage_name} 指令执行失败: {err_msg}")
        return False

    def align_to_board(self, safe_offset=120.0):
        """执行对准流程"""
        print("[*] 正在获取机器人当前状态...")
        curr_pose = self.robot.get_current_pose()
        if curr_pose is None: return False
        
        # 修正：RobotPose 对象转列表后再取 euler
        curr_pose_list = curr_pose.to_list()
        curr_R = R_tool_scipy.from_euler('XYZ', curr_pose_list[3:6]).as_matrix()

        print("[*] 正在识别目标...")
        poses = []
        for _ in range(5):
            p = self.get_board_pose_in_base(curr_R, safe_offset)
            if p is not None: poses.append(p)
            time.sleep(0.1)
        
        if not poses:
            print("[-] 未能检测到标定板"); return False
        
        target_pose_vec = np.mean(poses, axis=0)
        target_pose_obj = RobotPose.from_list(target_pose_vec.tolist())

        print("\n" + "-"*30)
        print(f"当前位姿: {curr_pose_list}")
        print(f"安全中继: {self.SAFE_POSE_LIST}")
        print(f"目标位姿: {target_pose_vec.tolist()}")
        print("-"*30)
        
        confirm = input("\n请确认。回车 (Enter) 开始运动，其他键取消: ")
        if confirm.strip() != "": return False

        safe_pose_obj = RobotPose.from_list(self.SAFE_POSE_LIST)
        print("[*] 1/2 移动至安全过渡点...")
        res1 = self.robot.move_j(safe_pose_obj, velocity=self.DEFAULT_SPEED)
        if not self.check_move_result(res1, "安全过渡点"): return False
        self.robot.wait_motion_done()
        
        # 拆分步骤 2: 保持 Home 的姿态 (即 SAFE_POSE 的 ABC) 移动到目标 XYZ
        step2_pose_obj = RobotPose(
            x=target_pose_obj.x, y=target_pose_obj.y, z=target_pose_obj.z,
            a=self.SAFE_POSE_LIST[3], b=self.SAFE_POSE_LIST[4], c=self.SAFE_POSE_LIST[5]
        )
        print("[*] 2/3 保持 Home 姿态平移至目标位置...")
        res2 = self.robot.move_j(step2_pose_obj, velocity=self.DEFAULT_SPEED)
        if not self.check_move_result(res2, "平移目标点"): return False
        self.robot.wait_motion_done()
        
        # 拆分步骤 3: 在目标位置原地调整姿态
        print("[*] 3/3 原地调整姿态至最终目标...")
        res3 = self.robot.move_j(target_pose_obj, velocity=self.DEFAULT_SPEED)
        if not self.check_move_result(res3, "最终目标姿态"): return False
        self.robot.wait_motion_done()
        print("[+] 自动对准运动全部完成。")
        return True

    def run(self):
        try:
            self.cam.start()
            if not self.robot.connect():
                print("[-] 机器人连接失败"); return
            
            print("[*] 已连接机器人。正在切换模式并上电...")
            self.robot.set_mode(MODE_TEACH) # 纳博特通常在示教模式下上电
            if self.robot.servo_power_on():
                print("[+] 伺服上电成功")
            else:
                print("[-] 伺服上电失败，请检查急停或报错"); return

            print("[*] 正在运动至 Home...")
            safe_pose_obj = RobotPose.from_list(self.SAFE_POSE_LIST)
            res_safe = self.robot.move_j(safe_pose_obj, velocity=self.DEFAULT_SPEED)
            if self.check_move_result(res_safe, "初始安全过渡"):
                self.robot.wait_motion_done()
                print("[*] 安全点位运动完成。1/2")
                time.sleep(1)
                res_home = self.robot.go_home(velocity=self.DEFAULT_SPEED)
                if self.check_move_result(res_home, "初始回Home"):
                    self.robot.wait_motion_done()
                    print("[*] Home运动完成。2/2")
                else:
                    print("[-] 初始回Home失败"); return
            else:
                print("[-] 无法移动至初始安全点"); return
            
            while True:
                print("\n" + "="*50)
                print(" 视觉自动对准工具 (v2.1) ")
                print(" 'g': 开始自动对准 (Go)")
                print(" 'h': 回到 Home (经由安全点)")
                print(" 'q': 退出程序")
                print("="*50)
                
                cmd = input("请输入指令: ").strip().lower()
                if cmd == 'g':
                    self.align_to_board()
                elif cmd == 'h':
                    res = self.robot.move_j(safe_pose_obj, velocity=self.DEFAULT_SPEED)
                    if self.check_move_result(res, "回Home安全过渡"):
                        self.robot.wait_motion_done()
                        self.robot.go_home()
                    else:
                        print("[-] 无法执行回Home运动")
                elif cmd == 'q':
                    break
        finally:
            self.cam.stop()

if __name__ == "__main__":
    Aligner().run()
