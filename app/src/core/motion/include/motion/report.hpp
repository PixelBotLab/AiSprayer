#pragma once

#include "motion/conventions.hpp"
#include "motion/robot_model.hpp"
#include "motion/types.hpp"

#include <array>
#include <string>
#include <vector>

namespace motion {

struct Issue {
  std::string type;
  std::string severity;  // ERROR | WARNING
  int segment_index = 0;
  int step_index = 0;
  std::string detail;
  Eigen::Vector3d location_xyz_mm{0, 0, 0};
};

struct PathVerifyReport {
  int path_id = 0;
  std::string name;
  std::string status = "PASS";  // PASS | WARNING | FAILED
  int total_interpolated = 0;
  double speed_mm_s = 120.0;
  double step_size_mm = 1.5;
  double recommended_safe_speed_mm_s = 120.0;
  std::array<double, 6> max_joint_velocities_deg_s{{180, 180, 180, 180, 180, 180}};
  std::array<double, 6> peak_joint_speeds_deg_s{{0, 0, 0, 0, 0, 0}};
  std::vector<Issue> issues;
  std::vector<JointVec> trajectory_q;
  std::vector<std::array<double, 6>> trajectory_tcp;  // x,y,z mm + rx,ry,rz deg
};

struct VerifySummary {
  std::string status = "PASS";
  int total_paths = 0;
  int total_waypoints = 0;
  int total_steps = 0;
  int total_issues = 0;
  int singularity_count = 0;
  int overspeed_count = 0;
  int unreachable_count = 0;
};

struct VerifyReport {
  VerifySummary summary;
  double nominal_speed_mm_s = 120.0;
  double slerp_step_mm = 1.5;
  std::array<double, 6> max_joint_velocities_deg_s{{180, 180, 180, 180, 180, 180}};
  ToolOffset urdf_tcp;
  std::vector<PathVerifyReport> path_reports;
};

struct VerifyOptions {
  double step_mm = 1.5;
  double speed_mm_s = 120.0;
};

struct OptimizeOptions {
  AxisGrid grid_x{-5, 5, 2};
  AxisGrid grid_y{-5, 5, 2};
  AxisGrid grid_z{-30, 30, 5};  // 与 aisprayer_config.yaml spraying.grid_tol_z_deg 一致
  int beam_width = 32;
  int max_candidates_per_branch = 16;
  int movel_checks_min = 10;
  int movel_checks_max = 100;
  double movel_spacing_mm = 5.0;
  Eigen::Vector3d weight_zero_dev{1.0, 1.0, 0.01};
  JointVec joint_weights = (JointVec() << 1.0, 1.2, 1.0, 0.8, 0.8, 0.5).finished();
  // 密集 MoveL 复核开关；采样步长/速度由传入的 ChainVerifier 自带 VerifyOptions 决定。
  bool dense_verify = true;

  std::string Validate() const;
};

struct OptimizeResult {
  PathItem path;
  bool modified = false;
  std::vector<JointVec> joints_rad;
  PathVerifyReport verify;
  double elapsed_ms = 0.0;
};

}  // namespace motion
