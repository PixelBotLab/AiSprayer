#include "motion/robot_model.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <tinyxml2.h>

namespace motion {
namespace {

// 对齐 Python3 round(x, 2)。double 的 0.251675*1000=251.674999…，
// nearbyint(x*100) 会因二次放大踩到 .5 进位成 251.68；这里按绝对值做半入并回推 1e-10。
double Round2(double v) {
  const double s = v >= 0.0 ? 1.0 : -1.0;
  return s * std::floor(std::abs(v) * 100.0 + 0.5 - 1e-10) / 100.0;
}

int TcpScore(const std::string& child, const std::string& target) {
  auto lower = [](std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
  };
  const std::string c = lower(child);
  const std::string t = lower(target);
  if (!t.empty() && (c == t || c.find(t) != std::string::npos)) return 1000;
  if (c.find("laser") != std::string::npos || c.find("nozzle") != std::string::npos ||
      c.find("tcp") != std::string::npos)
    return 100;
  if (c.find("tip") != std::string::npos) return 80;
  if (c.find("gun") != std::string::npos) return 50;
  if (c.find("tool") != std::string::npos) return 30;
  return 0;
}

}  // namespace

bool LoadRobotModelFromUrdf(const std::string& urdf_path, const std::string& tool_name,
                            RobotModel& out, std::string* err) {
  out.urdf_path = urdf_path;
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
    if (err) *err = "failed to parse URDF: " + urdf_path;
    return false;
  }
  const auto* root = doc.RootElement();
  if (!root) {
    if (err) *err = "URDF has no root";
    return false;
  }

  int best = -1;
  for (const auto* joint = root->FirstChildElement("joint"); joint;
       joint = joint->NextSiblingElement("joint")) {
    const char* name = joint->Attribute("name");
    const auto* limit = joint->FirstChildElement("limit");
    if (name && limit) {
      for (int i = 1; i <= 6; ++i) {
        if (std::string(name) == ("joint" + std::to_string(i))) {
          const double lo = limit->DoubleAttribute("lower", -kPi);
          const double hi = limit->DoubleAttribute("upper", kPi);
          const double vel = limit->DoubleAttribute("velocity", kPi);
          out.limits.min_rad[i - 1] = Rad(Round2(Deg(lo)));
          out.limits.max_rad[i - 1] = Rad(Round2(Deg(hi)));
          out.limits.max_vel_deg_s[i - 1] = Round2(Deg(vel));
        }
      }
    }

    const auto* parent = joint->FirstChildElement("parent");
    const auto* child = joint->FirstChildElement("child");
    const auto* origin = joint->FirstChildElement("origin");
    if (!parent || !child || !origin) continue;
    const char* plink = parent->Attribute("link");
    const char* clink = child->Attribute("link");
    if (!plink || !clink) continue;
    const std::string parent_link = plink;
    if (parent_link != "Link6" && parent_link != "link6" && parent_link != "flange") continue;
    const int score = TcpScore(clink, tool_name);
    if (score <= best) continue;
    best = score;
    double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
    const char* xyz = origin->Attribute("xyz");
    const char* rpy = origin->Attribute("rpy");
    if (xyz) std::sscanf(xyz, "%lf %lf %lf", &x, &y, &z);
    if (rpy) std::sscanf(rpy, "%lf %lf %lf", &rx, &ry, &rz);
    out.tool.tool_name = clink;
    out.tool.urdf_source = urdf_path.substr(urdf_path.find_last_of("/\\") + 1);
    out.tool.SetFromRoundedMmDeg(Eigen::Vector3d(Round2(x * kMmPerM), Round2(y * kMmPerM),
                                                 Round2(z * kMmPerM)),
                                 Eigen::Vector3d(Round2(Deg(rx)), Round2(Deg(ry)), Round2(Deg(rz))));
  }
  if (best < 0) {
    out.tool.has_tool = false;
    out.tool.tool_name = "flange";
  }
  return true;
}

}  // namespace motion
