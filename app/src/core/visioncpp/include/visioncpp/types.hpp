#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <vector>

namespace visioncpp {

struct VisionError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;

struct CameraIntrinsics {
    double fx = 0, fy = 0, cx = 0, cy = 0;
    int width = 0, height = 0;

    Mat3 matrix() const {
        Mat3 k = Mat3::Identity();
        k(0, 0) = fx;
        k(1, 1) = fy;
        k(0, 2) = cx;
        k(1, 2) = cy;
        return k;
    }

    bool valid() const {
        return fx > 1e-6 && fy > 1e-6 && width > 0 && height > 0;
    }
};

struct OrientedSample {
    Vec3 point = Vec3::Zero();
    Vec3 normal = Vec3::UnitZ();
    bool is_jump = false;
    int leg_id = 0;
};

struct Waypoint {
    int index = 0;
    int pixel_u = 0, pixel_v = 0;
    Vec3 surface_point_cam_mm = Vec3::Zero();
    Vec3 surface_point_base_mm = Vec3::Zero();
    Vec3 surface_normal_base = Vec3::UnitZ();
    Vec3 surface_normal_cam = Vec3::UnitZ();
    double standoff_distance_mm = 150.0;
    Vec3 tcp_xyz_mm = Vec3::Zero();
    Vec3 tcp_rpy_deg = Vec3::Zero();
    double n2d_u = 0, n2d_v = 0;
    bool is_jump = false;
    int leg_id = 0;
};

struct PathDoc {
    std::vector<Waypoint> points;
    double standoff_distance_mm = 150.0;
};

enum class ExitCode {
    Ok = 0,
    Failed = 1,
    MissingInput = 2,
};

}  // namespace visioncpp
