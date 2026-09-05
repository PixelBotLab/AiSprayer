#include "visioncpp/tool_frame.hpp"

#include <algorithm>
#include <cmath>

namespace visioncpp {

Mat3 ToolFrame::compute(const Vec3& n_base) {
    Vec3 n = n_base;
    const double nlen = n.norm();
    if (nlen < 1e-9) n = Vec3::UnitX();
    else n /= nlen;

    const Vec3 z_tool = -n;
    Vec3 x_ref = has_prev_ ? prev_x_ : Vec3::UnitZ();
    if (std::abs(z_tool.dot(x_ref)) > 0.92) {
        x_ref = (std::abs(z_tool.dot(Vec3::UnitX())) > 0.92) ? Vec3::UnitY() : Vec3::UnitX();
    }
    Vec3 y_tool = z_tool.cross(x_ref);
    double ylen = y_tool.norm();
    if (ylen < 1e-9) {
        x_ref = (std::abs(z_tool.dot(Vec3::UnitY())) < 0.92) ? Vec3::UnitY() : Vec3::UnitZ();
        y_tool = z_tool.cross(x_ref);
        ylen = y_tool.norm();
        if (ylen < 1e-9) y_tool = Vec3::UnitY();
        else y_tool /= ylen;
    } else {
        y_tool /= ylen;
    }
    Vec3 x_tool = y_tool.cross(z_tool);
    double xlen = x_tool.norm();
    if (xlen < 1e-9) x_tool = Vec3::UnitX();
    else x_tool /= xlen;

    if (has_prev_ && x_tool.dot(prev_x_) < 0) {
        x_tool = -x_tool;
        y_tool = -y_tool;
    }
    prev_x_ = x_tool;
    has_prev_ = true;

    Mat3 R;
    R.col(0) = x_tool;
    R.col(1) = y_tool;
    R.col(2) = z_tool;
    return R;
}

Vec3 ToolFrame::rpyXyzDeg(const Mat3& R) {
    // scipy Rotation.as_euler('xyz'): R = Rz(c) @ Ry(b) @ Rx(a)
    const double r20 = std::clamp(R(2, 0), -1.0, 1.0);
    double b = std::asin(-r20);
    double a = 0, c = 0;
    if (std::abs(R(2, 0)) < 0.999999) {
        a = std::atan2(R(2, 1), R(2, 2));
        c = std::atan2(R(1, 0), R(0, 0));
    } else {
        a = std::atan2(-R(0, 1), R(1, 1));
        c = 0;
    }
    constexpr double rad2deg = 180.0 / 3.14159265358979323846;
    return {a * rad2deg, b * rad2deg, c * rad2deg};
}

}  // namespace visioncpp
