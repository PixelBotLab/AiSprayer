#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys, cv2, yaml, numpy as np, time

# 1. 路径锚定策略
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.append(os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.hardware.camera.factory import get_camera
from aisprayer.core.hardware.robot.inexbot_driver import InexbotDriver, RobotPose, MODE_RUN, MODE_TEACH
from scipy.spatial.transform import Rotation as R_tool_scipy
from aisprayer.utils.config_helper import load_config, get_abs_path
from aisprayer.utils.hardware_helper import verify_hardware_consistency

class Aligner:
    # 默认值 (如果配置中没有)
    SAFE_POSE_LIST = [460.0, 125.0, 1077.0, -3.141, 0.0, 0.0]
    DEFAULT_SPEED = 50 

    # 机器人指令返回码映射
    RESULT_MAP = {
        0: "成功 (Success)",
        -1: "连接失败 (Connection Fail)",
        1: "常规错误 (General Error)",
        2: "参数非法 (Invalid Params)",
        3: "超时 (Timeout)",
        4: "正在运行中 (Busy)",
        5: "系统报警 (Alarm Active)"
    }

    def __init__(self, config_dict):
        h_cfg = config_dict.get("hardware", {})
        c_cfg = config_dict.get("calib", {})
        a_cfg = c_cfg.get("align", {})
        
        self.robot_ip = h_cfg.get("robot", {}).get("ip", "192.168.2.14")
        self.robot_port = h_cfg.get("robot", {}).get("port", 6001)
        self.SAFE_POSE_LIST = a_cfg.get("safe_pose", self.SAFE_POSE_LIST)
        self.DEFAULT_SPEED = a_cfg.get("speed", self.DEFAULT_SPEED)
        
        # 2. 标定文件一致性校验 (只关注 calib 自身的配置或默认路径)
        raw_path = c_cfg.get("result_path", "configs/calib/calibration_result.yaml")
        calib_path = get_abs_path(raw_path, PROJECT_ROOT)
        
        if not os.path.exists(calib_path):
            raise FileNotFoundError(f"找不到标定文件: {calib_path}")
            
        with open(calib_path, 'r') as f:
            res = yaml.safe_load(f)
            
        # 3. 记录参数 (暂不启动硬件)
        self.calib_res = res  # 暂存标定结果用于后续校验
        self.T_base_camera = np.array(res["T_base_camera"])
        self.K = np.array(res["camera_params"]["intrinsic_matrix"])
        self.D = np.array(res["camera_params"]["distortion_coeffs"])
        self.pattern_size = tuple(res["board_params"]["pattern_size_inner"])
        self.sq_size = res["board_params"]["square_size_mm"]
        
        # 初始化驱动对象
        calib_w = res.get("camera_params", {}).get("width", 640)
        calib_h = res.get("camera_params", {}).get("height", 480)
        camera_model = res.get("camera_params", {}).get("camera_model", "orbbec")
        self.cam = get_camera(camera_model, width=calib_w, height=calib_h)
        self.robot = InexbotDriver(ip=self.robot_ip, port=self.robot_port)
        
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
        p_center_cam = R_cam_board @ self.center_offset_obj + tvec.flatten()
        print(f"[Board] 检测到中心 (Camera): X={p_center_cam[0]:.2f}, Y={p_center_cam[1]:.2f}, Z={p_center_cam[2]:.2f}")
        
        p_center_base = self.T_base_camera[:3, :3] @ p_center_cam + self.T_base_camera[:3, 3]

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
        # 1. 先启动相机
        print(f"[*] 正在初始化相机 ({self.calib_res.get('camera_params', {}).get('camera_model')})...")
        try:
            self.cam.start()
            if not hasattr(self.cam, "pipeline") or self.cam.pipeline is None:
                print("[-] 相机启动异常。")
                return
            # 硬件一致性校验
            ok, msg = verify_hardware_consistency(live=self.cam, calib=self.calib_res)
            if not ok:
                print(f"\n[CRITICAL] 硬件校验失败！\n  原因: {msg}")
                self.cam.stop(); return
        except Exception as e:
            print(f"[-] 相机启动失败: {e}")
            return

        # 2. 再连接机器人
        print(f"[*] 正在连接机器人 ({self.robot_ip}:{self.robot_port})...")
        if not self.robot.connect():
            print(f"[-] 无法连接到机器人 {self.robot_ip}:{self.robot_port}，请检查网络配置。")
            self.cam.stop()
            return
            
        print(f"[*] 已连接机器人 ({self.robot_ip}:{self.robot_port})。正在上电...")
        self.robot.set_mode(MODE_TEACH) 
        if not self.robot.servo_power_on():
            print("[-] 伺服上电失败")
            self.cam.stop(); self.robot.disconnect(); return

        # 3. 硬件全部就绪后，再打印参数信息
        print("\n" + "="*50)
        print(" [系统初始化成功] ")
        print(f" 相机内参 K:\n{self.K}")
        print(f" 畸变系数 D: {self.D}")
        print(f" 标定板规格: {self.pattern_size}, 间距: {self.sq_size}mm")
        print(f" 手眼外参 T_base_camera:\n{self.T_base_camera}")
        print("="*50 + "\n")

        try:
            print("[+] 正在运动至 Home...")
            safe_pose_obj = RobotPose.from_list(self.SAFE_POSE_LIST)
            res_safe = self.robot.move_j(safe_pose_obj, velocity=self.DEFAULT_SPEED)
            if self.check_move_result(res_safe, "初始安全过渡"):
                self.robot.wait_motion_done()
                self.robot.go_home(velocity=self.DEFAULT_SPEED)
                self.robot.wait_motion_done()
            
            while True:
                print("\n" + "="*50)
                print(f" 视觉自动对准工具 (v2.2) | IP: {self.robot_ip}")
                print(" 'g': 开始自动对准 (Go)")
                print(" 'h': 回到 Home")
                print(" 'q': 退出程序")
                print("="*50)
                
                cmd = input("请输入指令: ").strip().lower()
                if cmd == 'g':
                    self.align_to_board()
                elif cmd == 'h':
                    self.robot.move_j(safe_pose_obj, velocity=self.DEFAULT_SPEED)
                    self.robot.wait_motion_done()
                    self.robot.go_home()
                elif cmd == 'q':
                    break
        finally:
            self.cam.stop()
            self.robot.disconnect()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="自动对准工具 (生产版)")
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs/aisprayer_config.yaml"), help="配置文件路径")
    args = parser.parse_args()
    
    cfg = load_config(args.config, PROJECT_ROOT)
    Aligner(cfg).run()
