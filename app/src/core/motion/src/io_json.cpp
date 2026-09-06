#include "motion/io.hpp"

#include <cstdio>
#include <iomanip>
#include <sstream>

namespace motion {
// 异常文本会直接拼进 JSON 字符串，引号/反斜杠/控制字符必须转义，否则 Python 侧解析失败。
// 定义在命名空间外，cli_main 的 EmitError 也走同一份实现。
std::string JsonEscapeString(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

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
  if (!message.empty()) os << ",\"message\":\"" << JsonEscapeString(message) << "\"";
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
  os << ",\"objective\":" << std::setprecision(4) << result.objective;
  // 容差阶梯择优：采纳档可能比请求档更紧，Python/前端需要知道实际用的是哪个包络。
  os << ",\"adopted_tolerance_rpy_deg\":[";
  for (int i = 0; i < 3; ++i) {
    if (i) os << ",";
    os << std::setprecision(2) << result.adopted_tol_deg[i];
  }
  os << "]";
  if (!result.ladder.empty()) {
    os << ",\"tolerance_ladder\":[";
    for (size_t i = 0; i < result.ladder.size(); ++i) {
      const auto& r = result.ladder[i];
      if (i) os << ",";
      os << "{\"tol_deg\":[" << std::setprecision(2) << r.tol_deg[0] << "," << r.tol_deg[1] << ","
         << r.tol_deg[2] << "],\"status\":\"" << r.status << "\",\"peak_deg_s\":"
         << std::setprecision(2) << r.peak_deg_s << ",\"peak_ratio\":" << std::setprecision(4)
         << r.peak_ratio << ",\"max_pointing_deg\":" << std::setprecision(2) << r.max_pointing_deg
         << ",\"objective\":" << r.objective << ",\"elapsed_ms\":" << std::setprecision(2)
         << r.elapsed_ms;
      if (!r.error.empty()) os << ",\"error\":\"" << JsonEscapeString(r.error) << "\"";
      os << "}";
    }
    os << "]";
  }
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
  if (!message.empty()) os << ",\"message\":\"" << JsonEscapeString(message) << "\"";
  os << "}";
  return os.str();
}

}  // namespace motion
