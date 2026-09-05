#include "visioncpp/calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

namespace visioncpp {
namespace {

Mat3 readMat3(const YAML::Node& n) {
    if (!n || !n.IsSequence() || n.size() != 3) {
        throw VisionError("expected 3x3 matrix in yaml");
    }
    Mat3 m;
    for (int r = 0; r < 3; ++r) {
        if (!n[r] || n[r].size() != 3) throw VisionError("expected 3x3 matrix row");
        for (int c = 0; c < 3; ++c) m(r, c) = n[r][c].as<double>();
    }
    return m;
}

Mat4 readMat4(const YAML::Node& n) {
    if (!n || !n.IsSequence() || n.size() != 4) {
        throw VisionError("expected 4x4 matrix in yaml");
    }
    Mat4 m;
    for (int r = 0; r < 4; ++r) {
        if (!n[r] || n[r].size() != 4) throw VisionError("expected 4x4 matrix row");
        for (int c = 0; c < 4; ++c) m(r, c) = n[r][c].as<double>();
    }
    return m;
}

}  // namespace

bool Calibration::isIdentity(const Mat4& T, double eps) {
    return (T - Mat4::Identity()).cwiseAbs().maxCoeff() < eps;
}

Calibration Calibration::load(const std::string& yaml_path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        throw VisionError(std::string("cannot read calib yaml: ") + e.what());
    }
    YAML::Node tnode = root["T_base_camera"] ? root["T_base_camera"] : root["T_camera_to_base"];
    if (!tnode) {
        throw VisionError("calib yaml missing T_base_camera / T_camera_to_base");
    }
    Calibration cal;
    cal.T_ = readMat4(tnode);
    cal.T_(0, 3) /= 1000.0;
    cal.T_(1, 3) /= 1000.0;
    cal.T_(2, 3) /= 1000.0;
    if (isIdentity(cal.T_)) {
        throw VisionError("T_camera_to_base is Identity; refuse to continue");
    }
    if (root["camera_params"] && root["camera_params"]["intrinsic_matrix"]) {
        cal.k_ = readMat3(root["camera_params"]["intrinsic_matrix"]);
    }
    return cal;
}

CameraIntrinsics Calibration::loadScanParams(const std::string& params_yaml) {
    YAML::Node root = YAML::LoadFile(params_yaml);
    const YAML::Node cam = root["camera_params"] ? root["camera_params"] : root;
    CameraIntrinsics k;
    if (cam["width"]) k.width = cam["width"].as<int>();
    if (cam["height"]) k.height = cam["height"].as<int>();
    if (cam["intrinsic_matrix"]) {
        const Mat3 m = readMat3(cam["intrinsic_matrix"]);
        k.fx = m(0, 0);
        k.fy = m(1, 1);
        k.cx = m(0, 2);
        k.cy = m(1, 2);
    }
    return k;
}

}  // namespace visioncpp
