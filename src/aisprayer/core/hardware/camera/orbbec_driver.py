#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Orbbec Gemini 336 相机原生驱动封装 (独立版)
基于 pyorbbecsdk 实现。
"""

import sys
import os
import logging
import numpy as np
import cv2
import time

try:
    try:
        from pyorbbecsdk2 import (
            Pipeline, Config, OBSensorType, OBFormat,
            OBFrameAggregateOutputMode, AlignFilter, OBStreamType
        )
    except ImportError:
        from pyorbbecsdk import (
            Pipeline, Config, OBSensorType, OBFormat,
            OBFrameAggregateOutputMode, AlignFilter, OBStreamType
        )
except ImportError:
    print("错误: 未找到 pyorbbecsdk 或 pyorbbecsdk2。请确认安装。")
    sys.exit(1)

logger = logging.getLogger(__name__)

class OrbbecDriver:
    """Orbbec Gemini 336 相机原生驱动"""

    def __init__(self, width=1280, height=800):
        self.width = width
        self.height = height
        self.model_name = "orbbec"
        self.pipeline = None
        self.config = None
        self.align_filter = None
        self.color_intrinsic = None
        self.color_distortion = None
        self._depth_enabled = False
        self._running = False

    def start(self):
        """启动相机"""
        try:
            self.pipeline = Pipeline()
        except Exception as e:
            raise RuntimeError(f"未检测到 Orbbec 设备: {e}")

        try:
            # 配置彩色流
            color_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            color_profile = None
            
            # 尝试列表：优先使用传入的分辨率，否则依次尝试 1280x720, 640x480 或默认
            for w, h in [(self.width, self.height), (1280, 720), (640, 480), (0, 0)]:
                try:
                    # 尝试不同格式 (RGB 是生产首选，MJPG/YUYV 作为备份)
                    for fmt in [OBFormat.RGB, OBFormat.MJPG, OBFormat.YUYV]:
                        try:
                            color_profile = color_profile_list.get_video_stream_profile(w, h, fmt)
                            if color_profile: 
                                print(f"[+] 成功匹配彩色流模式: {w}x{h} ({fmt})")
                                break
                        except Exception as e:
                            print(f"    [-] 尝试 {w}x{h} ({fmt}) 失败: {e}")
                            continue
                    if color_profile: break
                    print(f"[-] 分辨率 {w}x{h} 在当前设备上所有格式均不可用。")
                except Exception as e:
                    print(f"    [!] 分辨率尝试过程出错: {e}")
                    continue
            
            if not color_profile:
                color_profile = color_profile_list.get_default_video_stream_profile()

            # 配置深度流
            depth_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            depth_profile = depth_profile_list.get_default_video_stream_profile()

            self.config = Config()
            self.config.enable_stream(color_profile)
            self.config.enable_stream(depth_profile)
            self.config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
            
            self.pipeline.start(self.config)
            
            # 核心修正：从实际加载的 profile 中更新分辨率
            # 这样即便发生 Fallback，self.width 和 self.height 也是绝对准确的
            self.width = color_profile.get_width()
            self.height = color_profile.get_height()

            # 保存内参 (Gemini 336L 可能在启动后才能获取内参)
            self.color_intrinsic = color_profile.get_intrinsic()
            self.color_distortion = color_profile.get_distortion()
            
            # 设置深度对齐
            self.align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
            self._depth_enabled = True
            self._running = True
            
            # 预热
            for _ in range(10):
                self.pipeline.wait_for_frames(5000)
            
            print(f"Orbbec 相机已启动: {self.width}x{self.height}")
            
        except Exception as e:
            raise RuntimeError(f"相机初始化失败: {e}")

    def get_frame(self):
        """获取一帧彩色图和深度图"""
        if not self._running:
            return None, None

        try:
            frames = self.pipeline.wait_for_frames(5000)
            if frames is None:
                return None, None

            # 深度对齐到彩色
            aligned_frames = self.align_filter.process(frames)
            if aligned_frames is None:
                return None, None
            aligned_frames = aligned_frames.as_frame_set()

            # 解析彩色
            color_frame = aligned_frames.get_color_frame()
            if color_frame is None:
                return None, None
            
            color_data = np.frombuffer(color_frame.get_data(), dtype=np.uint8)
            color_width = color_frame.get_width()
            color_height = color_frame.get_height()
            color_format = color_frame.get_format()

            if color_format == OBFormat.RGB:
                color_image = color_data.reshape((color_height, color_width, 3))
                color_image = color_image[:, :, ::-1].copy()  # RGB -> BGR
            elif color_format == OBFormat.BGR:
                color_image = color_data.reshape((color_height, color_width, 3))
            elif color_format == OBFormat.MJPG:
                color_image = cv2.imdecode(color_data, cv2.IMREAD_COLOR)
            elif color_format == OBFormat.YUYV:
                yuyv = color_data.reshape((color_height, color_width, 2))
                color_image = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUYV)
            else:
                return None, None

            # 解析深度
            depth_frame = aligned_frames.get_depth_frame()
            if depth_frame is None:
                return color_image, None
            
            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            depth_image = depth_data.reshape((depth_frame.get_height(), depth_frame.get_width()))

            return color_image, depth_image

        except Exception as e:
            logger.warning(f"获取帧失败: {e}")
            return None, None

    def get_intrinsics(self):
        """返回内参矩阵和畸变系数"""
        if self.color_intrinsic is None:
            return None, None
        intr = self.color_intrinsic
        k = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]])
        dist = self.color_distortion
        d = np.array([dist.k1, dist.k2, dist.p1, dist.p2, dist.k3])
        return k, d

    def stop(self):
        if self._running and self.pipeline:
            self.pipeline.stop()
            self._running = False
