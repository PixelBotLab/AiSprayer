#pragma once

#include "motion/conventions.hpp"

#include <string>

namespace motion {

struct DhParams {
  static constexpr double d1 = 0.147;
  static constexpr double a2 = -0.427;
  static constexpr double a3 = -0.357;
  static constexpr double d4 = 0.141;
  static constexpr double d5 = 0.116;
  static constexpr double d6 = 0.105;
};

struct RobotLimits {
  JointVec min_rad = (JointVec() << -2.0 * kPi, -kPi, -2.86159, -kPi, -kPi, -2.0 * kPi).finished();
  JointVec max_rad = (JointVec() <<  2.0 * kPi,  kPi,  2.86159,  kPi,  kPi,  2.0 * kPi).finished();
  Eigen::Matrix<double, 6, 1> max_vel_deg_s =
      (Eigen::Matrix<double, 6, 1>() << 180, 180, 180, 180, 180, 180).finished();
};

struct ToolOffset {
  bool has_tool = false;
  std::string tool_name = "flange";
  std::string urdf_source;
  Eigen::Vector3d xyz_mm{0, 0, 0};
  Eigen::Vector3d rpy_deg{0, 0, 0};
  Transform T_tcp = Transform::Identity();      // flange → tool, 米
  Transform T_tcp_inv = Transform::Identity();

  void SetFromRoundedMmDeg(const Eigen::Vector3d& xyz_mm_in,
                           const Eigen::Vector3d& rpy_deg_in) {
    xyz_mm = xyz_mm_in;
    rpy_deg = rpy_deg_in;
    T_tcp = PoseFromCtrlMmDeg(xyz_mm, rpy_deg);
    T_tcp_inv = T_tcp.inverse();
    has_tool = true;
  }
};

struct RobotModel {
  std::string urdf_path;
  RobotLimits limits;
  ToolOffset tool;
};

// URDF 只提供限位与 TCP；DH 不从 URDF 解析。
// TCP 与 Python 一致：先 round(mm,2)/round(deg,2) 再建矩阵。
bool LoadRobotModelFromUrdf(const std::string& urdf_path,
                            const std::string& tool_name,
                            RobotModel& out,
                            std::string* err);

}  // namespace motion
