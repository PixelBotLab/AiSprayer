#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Intel RealSense D430/D435 相机驱动封装
适配项目 OrbbecDriver 接口模式。
"""

import sys
import time
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("错误: 请安装 pyrealsense2: pip install pyrealsense2")
    sys.exit(1)

class RealSenseDriver:
    """Intel RealSense 相机驱动封装类"""

    def __init__(self, width=1280, height=720, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.config = None
        self.align = None
        self.intrinsics = None
        self._running = False

    def start(self):
        """启动相机"""
        try:
            # 检查是否有 RealSense 设备连接
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                raise RuntimeError("未检测到 RealSense 设备！")

            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # 配置彩色流和深度流
            self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

            # 启动管线
            profile = self.pipeline.start(self.config)

            # 获取内参
            color_profile = profile.get_stream(rs.stream.color)
            self.intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

            # 对齐深度到彩色
            self.align = rs.align(rs.stream.color)
            self._running = True

            # 等待相机自动曝光稳定
            print("[*] RealSense 相机正在预热...")
            for _ in range(15):
                self.pipeline.wait_for_frames(5000)
            
            print(f"[+] RealSense 相机已启动: {self.width}x{self.height}@{self.fps}fps")
            
        except Exception as e:
            raise RuntimeError(f"RealSense 相机启动失败: {e}")

    def get_frame(self):
        """获取一帧对齐后的彩色图和深度图"""
        if not self._running:
            return None, None

        try:
            frames = self.pipeline.wait_for_frames(5000)
            if not frames:
                return None, None
                
            # 深度对齐到彩色
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                return None, None

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            return color_image, depth_image
            
        except Exception as e:
            print(f"[-] RealSense 获取帧失败: {e}")
            return None, None

    def get_intrinsics(self):
        """返回内参矩阵和畸变系数 (对接 OrbbecDriver 格式)"""
        if self.intrinsics is None:
            return None, None
        
        # 内参矩阵 K
        k = np.array([
            [self.intrinsics.fx, 0, self.intrinsics.ppx],
            [0, self.intrinsics.fy, self.intrinsics.ppy],
            [0, 0, 1]
        ])
        
        # 畸变系数
        d = np.array(self.intrinsics.coeffs)
        
        return k, d

    def stop(self):
        """停止相机"""
        if self._running and self.pipeline is not None:
            try:
                self.pipeline.stop()
            except:
                pass
            finally:
                self._running = False
