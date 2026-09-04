#pragma once

#include "motion/report.hpp"
#include "motion/types.hpp"

#include <string>

namespace motion {

// 与 configs/aisprayer_config.yaml 的 spraying / hardware.robot 对齐
struct SprayingConfig {
  std::string urdf_path;
  std::string tool_name = "gripper_tip_link";
  std::string anchor_source = "config";  // config | home | raw
  Eigen::Vector3d ref_rpy_deg{90.0, 0.0, 90.0};
  Eigen::Vector3d tol_deg{10.0, 10.0, 30.0};
  AxisGrid grid_x{-5.0, 5.0, 2.0};
  AxisGrid grid_y{-5.0, 5.0, 2.0};
  AxisGrid grid_z{-30.0, 30.0, 5.0};
  double speed_mm_s = 150.0;
  double step_mm = 2.0;
};

bool LoadSprayingConfig(const std::string& yaml_path, SprayingConfig& out, std::string* err);

// 写回 *.poi.path.yaml 顶层的 poi_config 块，供 web 回显实际生效的锚点约束。
struct PoiConfig {
  std::string mode = "absolute_anchor_tolerance";
  std::string anchor_source = "config";
  Eigen::Vector3d ref_rpy_deg{90.0, 0.0, 90.0};
  Eigen::Vector3d tolerance_rpy_deg{10.0, 10.0, 30.0};
  bool has_ref_rpy = true;  // anchor_source=raw 时 Python 侧写 null
};

bool LoadPathYaml(const std::string& path, PathDocument& out, std::string* err);
// verify / poi 均可为空；非空时按重构前 Python 版 _clean_report_data 的 schema 落盘。
bool SavePathYaml(const std::string& path, const PathDocument& doc, const VerifyReport* verify,
                  const PoiConfig* poi, std::string* err);

std::string JsonReportVerify(const VerifyReport& report, double elapsed_ms, bool success,
                             const std::string& message);
std::string JsonReportOptimize(const OptimizeResult& result, const VerifyReport* all,
                               bool success, const std::string& message);

}  // namespace motion
