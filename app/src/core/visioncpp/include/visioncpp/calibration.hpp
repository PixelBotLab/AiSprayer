#pragma once

#include "visioncpp/types.hpp"

#include <optional>
#include <string>

namespace visioncpp {

class Calibration {
public:
    static Calibration load(const std::string& yaml_path);
    static CameraIntrinsics loadScanParams(const std::string& params_yaml);

    const Mat4& T_camera_to_base() const { return T_; }
    bool hasK() const { return k_.has_value(); }
    Mat3 K() const { return k_.value_or(Mat3::Zero()); }

    static bool isIdentity(const Mat4& T, double eps = 1e-9);

private:
    Mat4 T_ = Mat4::Identity();
    std::optional<Mat3> k_;
};

}  // namespace visioncpp
