#include "follow/pose_io.hpp"

#include <cmath>
#include <limits>

namespace follow {
namespace {

constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
// cos(ry) 小于它就是万向锁：90° 附近 1e-9 相当于 ry 距 ±90° 约 6e-6 度。
constexpr double kGimbalEps = 1e-9;

}  // namespace

bool DobotPose::finite() const {
  return std::isfinite(x_mm) && std::isfinite(y_mm) && std::isfinite(z_mm) &&
         std::isfinite(rx_deg) && std::isfinite(ry_deg) && std::isfinite(rz_deg);
}

DobotPose to_dobot(const Eigen::Isometry3d& T) {
  const Eigen::Matrix3d& R = T.linear();
  DobotPose p;
  p.x_mm = T.translation().x() * 1000.0;
  p.y_mm = T.translation().y() * 1000.0;
  p.z_mm = T.translation().z() * 1000.0;

  // R = Rz·Ry·Rx 展开后：R20 = -sin(ry)，R00 = cos(ry)cos(rz)，R10 = cos(ry)sin(rz)，
  // R21 = cos(ry)sin(rx)，R22 = cos(ry)cos(rx)。
  const double cy = std::hypot(R(0, 0), R(1, 0));
  p.ry_deg = std::atan2(-R(2, 0), cy) * kRad2Deg;
  if (cy > kGimbalEps) {
    p.rx_deg = std::atan2(R(2, 1), R(2, 2)) * kRad2Deg;
    p.rz_deg = std::atan2(R(1, 0), R(0, 0)) * kRad2Deg;
  } else {
    // |ry| = 90°：rz 与 rx 不可分，固定 rz = 0。此时 R20 = ∓1，
    // R01 = ±sin(rx)，R11 = cos(rx)。
    const double s = R(2, 0) < 0.0 ? 1.0 : -1.0;
    p.rx_deg = std::atan2(s * R(0, 1), R(1, 1)) * kRad2Deg;
    p.rz_deg = 0.0;
  }
  return p;
}

Eigen::Isometry3d from_dobot(const DobotPose& p) {
  const double ax = p.rx_deg * kDeg2Rad;
  const double ay = p.ry_deg * kDeg2Rad;
  const double az = p.rz_deg * kDeg2Rad;
  Eigen::Matrix3d R = (Eigen::AngleAxisd(az, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(ay, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(ax, Eigen::Vector3d::UnitX()))
                          .toRotationMatrix();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = Eigen::Vector3d(p.x_mm * 0.001, p.y_mm * 0.001, p.z_mm * 0.001);
  return T;
}

}  // namespace follow
