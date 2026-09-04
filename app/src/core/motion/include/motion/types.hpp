#pragma once

#include "motion/conventions.hpp"

#include <optional>
#include <string>
#include <vector>

namespace motion {

struct Waypoint {
  int index = 0;
  Eigen::Vector2i pixel{0, 0};
  Eigen::Vector3d surface_point_m{0, 0, 0};
  Eigen::Vector3d surface_normal{0, 0, 1};
  Transform tcp_pose = Transform::Identity();  // 控制器帧，米
  double standoff_m = 0.15;
  bool spraying = true;
  bool is_jump = false;
  // 优化只改 tcp_pose，其余字段必须原样透传回 yaml（Python 版是 deepcopy 语义）。
  std::optional<Eigen::Vector2d> normal_2d_proj;
  bool has_is_jump = false;
};

struct PathItem {
  int path_id = 0;
  std::string name;
  std::vector<Waypoint> points;
  // 喷涂面密集采样点（mm），优化不使用但需透传。
  std::vector<Eigen::Vector3d> dense_surface_points_mm;
};

struct PathDocument {
  std::string template_name;
  std::string type;
  std::string state_type;
  std::string source_file;
  std::string coordinate_frame = "base_link";
  double standoff_distance_mm = 150.0;
  double execution_speed_mm_s = 120.0;
  std::vector<PathItem> paths;
};

struct Anchor {
  std::optional<Eigen::Matrix3d> R;
  Eigen::Vector3d tol_deg{10.0, 10.0, 180.0};
  bool has_global() const { return R.has_value(); }
};

struct AnchorSpec {
  std::string source = "config";  // home | config | raw | live
  Eigen::Vector3d ref_rpy_deg{90.0, 0.0, 90.0};
  Eigen::Vector3d tol_deg{10.0, 10.0, 180.0};
  JointVec home_joints_rad = (JointVec() << 0, 0, -kPi / 2, -kPi / 2, -kPi / 2, 0).finished();
};

}  // namespace motion
