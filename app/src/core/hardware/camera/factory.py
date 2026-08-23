#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机工厂模块：统一管理不同型号相机的实例化
"""

def get_camera(camera_type="orbbec", width=1280, height=800, fps=15):
    """
    根据类型获取相机驱动实例
    
    Args:
        camera_type (str): "orbbec" 或 "realsense"
        width, height, fps: 相机配置参数
    
    Returns:
        相机驱动实例
    """
    camera_type = camera_type.lower()
    
    if camera_type == "orbbec":
        from core.hardware.camera.orbbec_driver import OrbbecDriver
        return OrbbecDriver(width=width, height=height, fps=fps)
        
    elif camera_type == "realsense":
        from core.hardware.camera.realsense_driver import RealSenseDriver
        return RealSenseDriver(width=width, height=height, fps=fps)
        
    else:
        raise ValueError(f"未知的相机类型: {camera_type}. 目前支持: orbbec, realsense")
