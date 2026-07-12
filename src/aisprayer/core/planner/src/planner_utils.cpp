#include "planner_utils.hpp"
#include <stdexcept>
#include <iostream>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace aisprayer::planner {

using json = nlohmann::json;

/**
 * @brief 检查一个浮点数是否为有限值。
 * @note 用于在解析 JSON 时避免出现 NaN 或 Inf 导致后续计算崩溃。
 */
bool isFinite(const double value)
{
  return std::isfinite(value);
}

bool CameraCalibration::project(const Eigen::Vector3d& p_base, double& u, double& v) const
{
  if (!valid) return false;

  Eigen::Vector4d p_cam_4 = T_camera_base * p_base.homogeneous();
  if (p_cam_4.z() <= 0) return false;

  double x_n = p_cam_4.x() / p_cam_4.z();
  double y_n = p_cam_4.y() / p_cam_4.z();

  double x_d = x_n;
  double y_d = y_n;
  if (distortion_coeffs.size() >= 5) {
    double r2 = x_n * x_n + y_n * y_n;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double k1 = distortion_coeffs[0];
    double k2 = distortion_coeffs[1];
    double p1 = distortion_coeffs[2];
    double p2 = distortion_coeffs[3];
    double k3 = distortion_coeffs[4];

    double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    x_d = x_n * radial + 2.0 * p1 * x_n * y_n + p2 * (r2 + 2.0 * x_n * x_n);
    y_d = y_n * radial + p1 * (r2 + 2.0 * y_n * y_n) + 2.0 * p2 * x_n * y_n;
  }

  double fx = intrinsic_matrix(0, 0);
  double fy = intrinsic_matrix(1, 1);
  double cx = intrinsic_matrix(0, 2);
  double cy = intrinsic_matrix(1, 2);

  u = fx * x_d + cx;
  v = fy * y_d + cy;
  return true;
}

CameraCalibration loadCalibration(const std::string& calib_path)
{
  CameraCalibration calib;
  try {
    if (calib_path.empty() || !std::filesystem::exists(calib_path)) {
      std::cerr << "Warning: calibration file not provided or not found, 2D mappings will be skipped.\n";
      return calib;
    }
    YAML::Node root = YAML::LoadFile(calib_path);
    const YAML::Node t_node = root["T_base_camera"];
    if (t_node.IsSequence() && t_node.size() == 4) {
      for (int i = 0; i < 4; ++i) {
        if (t_node[i].IsSequence() && t_node[i].size() == 4) {
          for (int j = 0; j < 4; ++j) {
            double val = t_node[i][j].as<double>();
            if (j == 3 && i < 3) val /= 1000.0; // mm to meters
            calib.T_base_camera(i, j) = val;
          }
        }
      }
      calib.T_camera_base = calib.T_base_camera.inverse();
    }
    const YAML::Node cam_params = root["camera_params"];
    if (cam_params.IsMap()) {
      const YAML::Node int_node = cam_params["intrinsic_matrix"];
      if (int_node.IsSequence() && int_node.size() == 3) {
        for (int i = 0; i < 3; ++i) {
          if (int_node[i].IsSequence() && int_node[i].size() == 3) {
            for (int j = 0; j < 3; ++j) {
              calib.intrinsic_matrix(i, j) = int_node[i][j].as<double>();
            }
          }
        }
      }
      const YAML::Node dist_node = cam_params["distortion_coeffs"];
      if (dist_node.IsSequence()) {
        for (std::size_t i = 0; i < dist_node.size(); ++i) {
          calib.distortion_coeffs.push_back(dist_node[i].as<double>());
        }
      }
    }
    calib.valid = true;
    std::cout << "Successfully loaded camera calibration from " << calib_path << '\n';
  } catch (const std::exception& e) {
    std::cerr << "Error loading calibration: " << e.what() << "\n2D mappings will be skipped.\n";
  }
  return calib;
}

/**
 * @brief 从 JSON 对象中提取必需的数字字段。
 * @note 如果该字段不存在或不是有限数字，将会抛出 runtime_error，确保数据完整性。
 */
double requiredNumber(const json& object, const char* name)
{
  if (!object.contains(name) || !object.at(name).is_number())
    throw std::runtime_error(std::string("json field '") + name + "' must be a number");

  const double value = object.at(name).get<double>();
  if (!isFinite(value))
    throw std::runtime_error(std::string("json field '") + name + "' must be finite");
  return value;
}

/**
 * @brief 从 JSON 对象中提取必需的无符号整数索引字段。
 * @note 主要用于解析数组或网格的下标，如果字段缺失、非整数或是负数，则抛出异常。
 */
std::size_t requiredIndex(const json& object, const char* name)
{
  if (!object.contains(name) ||
      (!object.at(name).is_number_unsigned() && !object.at(name).is_number_integer()))
    throw std::runtime_error(std::string("json field '") + name + "' must be a non-negative integer");
  if (object.at(name).is_number_integer() && object.at(name).get<std::int64_t>() < 0)
    throw std::runtime_error(std::string("json field '") + name + "' must be a non-negative integer");
  return object.at(name).get<std::size_t>();
}

/**
 * @brief 解析 JSON 格式的目标位姿 (位置 + 四元数)，并转换为 Eigen::Isometry3d。
 * @note 期望 JSON 包含 x, y, z, qw, qx, qy, qz 字段。如果缺失，会包含所在的 stroke 和 point 索引来提示报错。
 */
Eigen::Isometry3d parseTargetPose(const json& value, std::size_t stroke_index, std::size_t point_index)
{
  if (!value.is_object())
    throw std::runtime_error("stroke " + std::to_string(stroke_index) + ", point " + std::to_string(point_index) +
                             " must be an object");

  const double x = requiredNumber(value, "x");
  const double y = requiredNumber(value, "y");
  const double z = requiredNumber(value, "z");
  const Eigen::Quaterniond quaternion(requiredNumber(value, "qw"),
                                      requiredNumber(value, "qx"),
                                      requiredNumber(value, "qy"),
                                      requiredNumber(value, "qz"));

  Eigen::Isometry3d pose;
  pose.linear() = quaternion.toRotationMatrix();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

/**
 * @brief 将 Eigen::Isometry3d 格式的位姿转换为 JSON 对象。
 * @note 输出的 JSON 包含平移量 x, y, z 以及旋转的四元数 qx, qy, qz, qw。
 */
json poseJson(const Eigen::Isometry3d& pose)
{
  const Eigen::Quaterniond quaternion(pose.rotation());
  return { { "x", pose.translation().x() },
           { "y", pose.translation().y() },
           { "z", pose.translation().z() },
           { "qw", quaternion.w() },
           { "qx", quaternion.x() },
           { "qy", quaternion.y() },
           { "qz", quaternion.z() } };
}

} // namespace aisprayer::planner
