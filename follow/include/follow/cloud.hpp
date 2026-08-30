// 深度图 → 相机系点云。输入要求：CV_16UC1，单位毫米，已与彩色 D2C 对齐。
#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "follow/types.hpp"

namespace follow {

struct CloudStats {
  int total_samples = 0;
  int kept = 0;
  int rejected_range = 0;
  int rejected_nonfinite = 0;
};

// 针孔反投影。调用方须先保证 k.valid()。
Eigen::Vector3d unproject(const CameraIntrinsics& k, double u_px, double v_px, double z_m);

// z_m 无效时返回 0。uv 为全分辨率像素坐标。
double depth_at_m(const cv::Mat& depth_mm, const cv::Point2f& uv, const CameraIntrinsics& k,
                  double zmin_m, double zmax_m);

// stride < 1 时按 1 处理。内参非法时返回空云并把 status 置为 kConfigInvalid
// —— 让调用方拿到明确原因，而不是产出一堆 inf 再交给下游。
std::vector<Eigen::Vector3f> depth_to_cloud(const cv::Mat& depth_mm, const CameraIntrinsics& k,
                                            double zmin_m, double zmax_m, int stride,
                                            CloudStats* stats = nullptr,
                                            Status* status = nullptr);

}  // namespace follow
