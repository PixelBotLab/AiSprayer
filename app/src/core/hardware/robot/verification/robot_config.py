"""
Robot configuration and URDF XML parser module.
Handles loading joint limits, velocities, tool link TCP offsets, and computing
the TCP transformation matrix T_tcp and its inverse T_tcp_inv.
"""

import os
import math
import logging
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

logger = logging.getLogger(__name__)


from core.config import (
    get_configured_robot_config,
    get_configured_optimization_config,
)


def load_limits_from_urdf(urdf_path: str = None) -> dict:
    """
    Parses joint limit angles (lower, upper in deg) and max joint velocity (deg/s) from URDF XML.
    """
    if urdf_path is None:
        urdf_path, _ = get_configured_robot_config()

    joint_limits_deg = {}
    joint_max_vel_deg_s = [180.0] * 6

    if urdf_path and os.path.exists(urdf_path):
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            for joint in root.findall('joint'):
                name = joint.get('name', '')
                limit = joint.find('limit')
                if limit is not None:
                    lower_rad = float(limit.get('lower', -math.pi))
                    upper_rad = float(limit.get('upper', math.pi))
                    vel_rad_s = float(limit.get('velocity', math.pi))
                    
                    joint_limits_deg[name] = (
                        round(math.degrees(lower_rad), 2),
                        round(math.degrees(upper_rad), 2)
                    )
                    # Map joint1..joint6 velocity
                    for j_idx in range(1, 7):
                        if f"joint{j_idx}" == name:
                            joint_max_vel_deg_s[j_idx - 1] = round(math.degrees(vel_rad_s), 2)
        except Exception as e:
            logger.warning(f"Could not parse joint limits from URDF {urdf_path}: {e}")

    return {
        "joint_limits": joint_limits_deg,
        "joint_limits_deg": joint_limits_deg,
        "max_joint_vel_deg_s": joint_max_vel_deg_s,
        "joint_max_vel_deg_s": joint_max_vel_deg_s,
        "urdf_path": urdf_path,
        "urdf_source": os.path.basename(urdf_path) if urdf_path else None
    }


def load_tcp_from_urdf(urdf_path: str = None, target_tcp_name: str = None) -> dict:
    """
    Parses TCP offset (XYZ mm, RPY deg) from URDF attached to Link6.
    """
    if urdf_path is None or target_tcp_name is None:
        cfg_urdf, cfg_tcp = get_configured_robot_config()
        if urdf_path is None:
            urdf_path = cfg_urdf
        if target_tcp_name is None:
            target_tcp_name = cfg_tcp

    tcp_info = {
        "has_tool": False,
        "tool_name": "flange",
        "xyz_mm": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "urdf_source": os.path.basename(urdf_path) if urdf_path else None
    }

    if urdf_path and os.path.exists(urdf_path):
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            best_score = -1
            for joint in root.findall('joint'):
                parent = joint.find('parent')
                child = joint.find('child')
                if parent is not None and parent.get('link') in ['Link6', 'link6', 'flange']:
                    child_name = child.get('link', '') if child is not None else ''
                    origin = joint.find('origin')
                    if origin is not None:
                        xyz_str = origin.get('xyz', '0 0 0').split()
                        rpy_str = origin.get('rpy', '0 0 0').split()
                        xyz_m = [float(v) for v in xyz_str]
                        rpy_rad = [float(v) for v in rpy_str]
                        xyz_mm = [round(v * 1000.0, 2) for v in xyz_m]
                        rpy_deg = [round(math.degrees(v), 2) for v in rpy_rad]
                        
                        score = 0
                        if target_tcp_name and (child_name.lower() == target_tcp_name.lower() or target_tcp_name.lower() in child_name.lower()):
                            score = 1000
                        elif any(k in child_name.lower() for k in ['laser', 'nozzle', 'tcp']):
                            score = 100
                        elif 'tip' in child_name.lower():
                            score = 80
                        elif 'gun' in child_name.lower():
                            score = 50
                        elif 'tool' in child_name.lower():
                            score = 30

                        if score > best_score:
                            best_score = score
                            tcp_info = {
                                "has_tool": True,
                                "tool_name": child_name,
                                "xyz_mm": xyz_mm,
                                "rpy_deg": rpy_deg,
                                "urdf_source": os.path.basename(urdf_path)
                            }
        except Exception as e:
            logger.warning(f"Could not parse TCP from URDF {urdf_path}: {e}")

    return tcp_info


class RobotConfig:
    """
    Manages active robot kinematic parameters, limits, and TCP transformations.
    """
    def __init__(
        self,
        urdf_path: str = None,
        max_joint_vel_deg_s: list[float] = None
    ):
        configured_urdf, _ = get_configured_robot_config()
        self.urdf_path = urdf_path or configured_urdf
        self.urdf_info = load_limits_from_urdf(self.urdf_path)
        self.urdf_tcp = load_tcp_from_urdf(self.urdf_path)

        if max_joint_vel_deg_s is not None:
            self.max_joint_vel_deg_s = np.array(max_joint_vel_deg_s, dtype=np.float64)
        else:
            self.max_joint_vel_deg_s = np.array(
                self.urdf_info.get("max_joint_vel_deg_s", [180.0] * 6),
                dtype=np.float64
            )

        # TCP transformation relative to flange
        self.T_tcp = np.eye(4, dtype=np.float64)
        self.T_tcp_inv = np.eye(4, dtype=np.float64)

        if self.urdf_tcp.get("has_tool", False):
            self.set_tcp_offset(
                self.urdf_tcp["xyz_mm"],
                self.urdf_tcp["rpy_deg"]
            )

        self.joint_min_rad, self.joint_max_rad = self._limits_rad_from_urdf()

    def _limits_rad_from_urdf(self) -> tuple[np.ndarray, np.ndarray]:
        """CR5 defaults, overwritten by URDF joint1..joint6 when present."""
        joint_min = np.array([-2.0 * math.pi, -math.pi, -2.86159, -math.pi, -math.pi, -2.0 * math.pi], dtype=np.float64)
        joint_max = np.array([ 2.0 * math.pi,  math.pi,  2.86159,  math.pi,  math.pi,  2.0 * math.pi], dtype=np.float64)
        limits = self.urdf_info.get("joint_limits_deg") or self.urdf_info.get("joint_limits") or {}
        for i in range(6):
            name = f"joint{i + 1}"
            if name in limits:
                lo_deg, hi_deg = limits[name]
                joint_min[i] = math.radians(float(lo_deg))
                joint_max[i] = math.radians(float(hi_deg))
        return joint_min, joint_max

    def set_tcp_offset(self, xyz_mm: list[float], rpy_deg: list[float]):
        """
        Sets the TCP transform matrix T_flange_tool in meters.
        """
        xyz_m = np.array(xyz_mm, dtype=np.float64) / 1000.0
        r_mat = R_scipy.from_euler('xyz', rpy_deg, degrees=True).as_matrix()

        self.T_tcp = np.eye(4, dtype=np.float64)
        self.T_tcp[:3, :3] = r_mat
        self.T_tcp[:3, 3] = xyz_m
        self.T_tcp_inv = np.linalg.inv(self.T_tcp)
        logger.info(f"🔧 [RobotConfig] Set Tool TCP Offset: XYZ={xyz_mm}mm, RPY={rpy_deg}°")
