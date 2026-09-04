#include "motion/io.hpp"

#include <iomanip>
#include <sstream>

namespace motion {
namespace {

void Arr6(std::ostringstream& os, const std::array<double, 6>& a) {
  os << "[";
  for (int i = 0; i < 6; ++i) {
    if (i) os << ",";
    os << std::fixed << std::setprecision(4) << a[i];
  }
  os << "]";
}

void Joint6(std::ostringstream& os, const JointVec& q) {
  os << "[";
  for (int i = 0; i < 6; ++i) {
    if (i) os << ",";
    os << std::fixed << std::setprecision(6) << q[i];
  }
  os << "]";
}

}  // namespace

std::string JsonReportVerify(const VerifyReport& report, double elapsed_ms, bool success,
                             const std::string& message) {
  std::ostringstream os;
  os << "{";
  os << "\"success\":" << (success ? "true" : "false");
  os << ",\"action\":\"verify\"";
  os << ",\"elapsed_ms\":" << std::fixed << std::setprecision(3) << elapsed_ms;
  os << ",\"status\":\"" << report.summary.status << "\"";
  os << ",\"summary\":{";
  os << "\"status\":\"" << report.summary.status << "\"";
  os << ",\"total_paths\":" << report.summary.total_paths;
  os << ",\"total_waypoints\":" << report.summary.total_waypoints;
  os << ",\"total_steps\":" << report.summary.total_steps;
  os << ",\"total_interpolated\":" << report.summary.total_steps;
  os << ",\"total_issues\":" << report.summary.total_issues;
  os << ",\"singularity_count\":" << report.summary.singularity_count;
  os << ",\"overspeed_count\":" << report.summary.overspeed_count;
  os << ",\"unreachable_count\":" << report.summary.unreachable_count;
  os << "}";
  if (!report.path_reports.empty()) {
    os << ",\"peak_joint_speeds_deg_s\":";
    Arr6(os, report.path_reports[0].peak_joint_speeds_deg_s);
    os << ",\"recommended_safe_speed_mm_s\":"
       << report.path_reports[0].recommended_safe_speed_mm_s;
    os << ",\"total_interpolated\":" << report.path_reports[0].total_interpolated;
    if (!report.path_reports[0].trajectory_q.empty()) {
      os << ",\"first_q\":";
      Joint6(os, report.path_reports[0].trajectory_q.front());
      os << ",\"last_q\":";
      Joint6(os, report.path_reports[0].trajectory_q.back());
    }
  }
  os << ",\"issues_count\":" << report.summary.total_issues;
  if (!message.empty()) os << ",\"message\":\"" << message << "\"";
  os << ",\"urdf_tcp\":{";
  os << "\"has_tool\":" << (report.urdf_tcp.has_tool ? "true" : "false");
  os << ",\"tool_name\":\"" << report.urdf_tcp.tool_name << "\"";
  os << ",\"xyz_mm\":[" << std::setprecision(2) << report.urdf_tcp.xyz_mm[0] << ","
     << report.urdf_tcp.xyz_mm[1] << "," << report.urdf_tcp.xyz_mm[2] << "]";
  os << ",\"rpy_deg\":[" << report.urdf_tcp.rpy_deg[0] << "," << report.urdf_tcp.rpy_deg[1]
     << "," << report.urdf_tcp.rpy_deg[2] << "]";
  os << "}";
  os << "}";
  return os.str();
}

std::string JsonReportOptimize(const OptimizeResult& result, const VerifyReport* all,
                               bool success, const std::string& message) {
  std::ostringstream os;
  os << "{";
  os << "\"success\":" << (success ? "true" : "false");
  os << ",\"action\":\"optimize\"";
  os << ",\"elapsed_ms\":" << std::fixed << std::setprecision(3) << result.elapsed_ms;
  os << ",\"was_modified\":" << (result.modified ? "true" : "false");
  if (all) {
    os << ",\"status\":\"" << all->summary.status << "\"";
    os << ",\"summary\":{";
    os << "\"status\":\"" << all->summary.status << "\"";
    os << ",\"total_paths\":" << all->summary.total_paths;
    os << ",\"total_waypoints\":" << all->summary.total_waypoints;
    os << ",\"total_steps\":" << all->summary.total_steps;
    os << ",\"total_issues\":" << all->summary.total_issues;
    os << "}";
  } else {
    os << ",\"status\":\"" << result.verify.status << "\"";
  }
  if (!message.empty()) os << ",\"message\":\"" << message << "\"";
  os << "}";
  return os.str();
}

}  // namespace motion
