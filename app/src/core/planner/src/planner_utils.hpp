#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

namespace aisprayer::planner {

// Shared struct for Process Planner
/**
 * @brief 表示一个规划好的喷涂轨迹，包含轨迹在网格中的索引、源信息以及对应的表面与TCP位姿序列。
 */
struct PlannedStroke
{
    std::size_t mesh_index;
    std::string mesh_source;
    std::vector<Eigen::Isometry3d> surface_poses;
    std::vector<Eigen::Isometry3d> tcp_poses;
};

/**
 * @brief 相机标定数据结构，保存从机械臂基座到相机的变换矩阵。
 */
struct CameraCalibration
{
  Eigen::Matrix4d T_base_camera = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_camera_base = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d intrinsic_matrix = Eigen::Matrix3d::Identity();
  std::vector<double> distortion_coeffs;

  bool valid = false;

  bool project(const Eigen::Vector3d& p_base, double& u, double& v) const;
};

/**
 * @brief 从 YAML 文件中加载相机标定数据。
 */
CameraCalibration loadCalibration(const std::string& calib_path);

/**
 * @brief 网格物体沿 PCA 最长轴的主方向信息。
 * @note 包含计算出的最长轴方向 (2D 投影) 以及该轴对应的旋转角度 (度数)。
 */
struct MeshPCAInfo {
  Eigen::Vector2d axis;
  double angle_deg;
};

// Shared JSON conversions
/**
 * @brief 将 Eigen 变换矩阵转换为 JSON 对象。
 */
nlohmann::json poseJson(const Eigen::Isometry3d& pose);
/**
 * @brief 从 JSON 解析目标位姿。
 */
Eigen::Isometry3d parseTargetPose(const nlohmann::json& value, std::size_t stroke_index, std::size_t point_index);
/**
 * @brief 从 JSON 对象中获取指定的浮点数值，若不存在则抛出异常。
 */
double requiredNumber(const nlohmann::json& object, const char* name);
/**
 * @brief 从 JSON 对象中获取指定的索引值，若不存在则抛出异常。
 */
std::size_t requiredIndex(const nlohmann::json& object, const char* name);

} // namespace aisprayer::planner
