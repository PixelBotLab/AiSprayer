// 描述子互近邻 + 3D-3D 稀疏配准（给稠密配准当平移初值，以及它退化时顶上）。
#pragma once

#include <optional>
#include <random>
#include <vector>

#include <opencv2/core.hpp>

#include "follow/cloud.hpp"
#include "follow/types.hpp"

namespace follow {

// 互近邻 + Lowe ratio。两帧描述子必须同维度，否则返回空（混用不同前端时会发生）。
std::vector<cv::DMatch> mutual_nn(const cv::Mat& a, const cv::Mat& b, float ratio = 0.8f);

// 3D-3D 刚体（Umeyama，无尺度）。点数 < 3 或退化返回 nullopt。
std::optional<Eigen::Isometry3d> umeyama_3d(const std::vector<Eigen::Vector3d>& src,
                                            const std::vector<Eigen::Vector3d>& dst);

struct SparseDeltaParams {
  double zmin_m = 0.30;
  double zmax_m = 2.50;
  double inlier_dist_m = 0.02;  // 内点残差门限（米）
  int min_inliers = 12;
  int ransac_iters = 32;
  double min_sample_area_m2 = 4e-4;  // 3 点最小解三角形面积下限，挡共线采样
};

struct SparseDelta {
  Eigen::Isometry3d T_prev_from_curr = Eigen::Isometry3d::Identity();  // 当前相机系 → 上一帧相机系
  int inliers = 0;
  int candidates = 0;
};

// rng 由调用方持有并播种：RANSAC 结果必须可复现，否则回归测试没有意义，
// 且 rand() 非线程安全。
std::optional<SparseDelta> sparse_delta(const FeatureFrame& prev, const FeatureFrame& curr,
                                        const cv::Mat& depth_prev_mm, const cv::Mat& depth_curr_mm,
                                        const CameraIntrinsics& k, const SparseDeltaParams& p,
                                        std::mt19937& rng);

}  // namespace follow
