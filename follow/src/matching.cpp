#include "follow/matching.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include <opencv2/features2d.hpp>

namespace follow {

std::vector<cv::DMatch> mutual_nn(const cv::Mat& a, const cv::Mat& b, float ratio) {
  if (a.empty() || b.empty() || a.cols != b.cols || a.type() != b.type()) {
    return {};
  }
  cv::BFMatcher matcher(cv::NORM_L2);
  std::vector<std::vector<cv::DMatch>> knn_ab, knn_ba;
  matcher.knnMatch(a, b, knn_ab, 2);
  matcher.knnMatch(b, a, knn_ba, 2);

  std::vector<int> ba(b.rows, -1);
  for (const auto& kn : knn_ba) {
    if (kn.size() < 2) {
      continue;
    }
    if (kn[0].distance < ratio * kn[1].distance) {
      ba[kn[0].queryIdx] = kn[0].trainIdx;
    }
  }

  std::vector<cv::DMatch> out;
  for (const auto& kn : knn_ab) {
    if (kn.size() < 2) {
      continue;
    }
    if (kn[0].distance >= ratio * kn[1].distance) {
      continue;
    }
    if (ba[kn[0].trainIdx] != kn[0].queryIdx) {
      continue;
    }
    out.push_back(kn[0]);
  }
  return out;
}

std::optional<Eigen::Isometry3d> umeyama_3d(const std::vector<Eigen::Vector3d>& src,
                                            const std::vector<Eigen::Vector3d>& dst) {
  if (src.size() < 3 || src.size() != dst.size()) {
    return std::nullopt;
  }
  Eigen::MatrixXd A(3, static_cast<int>(src.size()));
  Eigen::MatrixXd B(3, static_cast<int>(dst.size()));
  for (size_t i = 0; i < src.size(); ++i) {
    if (!src[i].allFinite() || !dst[i].allFinite()) {
      return std::nullopt;
    }
    A.col(static_cast<int>(i)) = src[i];
    B.col(static_cast<int>(i)) = dst[i];
  }
  const Eigen::Matrix4d T = Eigen::umeyama(A, B, false);
  if (!T.allFinite()) {
    return std::nullopt;
  }
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.matrix() = T;
  return iso;
}

namespace {

double triangle_area(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
  return 0.5 * (b - a).cross(c - a).norm();
}

}  // namespace

std::optional<SparseDelta> sparse_delta(const FeatureFrame& prev, const FeatureFrame& curr,
                                        const cv::Mat& depth_prev_mm, const cv::Mat& depth_curr_mm,
                                        const CameraIntrinsics& k, const SparseDeltaParams& p,
                                        std::mt19937& rng) {
  if (!k.valid() || prev.empty() || curr.empty()) {
    return std::nullopt;
  }

  const auto matches = mutual_nn(curr.desc, prev.desc);
  std::vector<Eigen::Vector3d> src, dst;  // src=当前帧相机系, dst=上一帧相机系
  src.reserve(matches.size());
  dst.reserve(matches.size());
  for (const auto& m : matches) {
    if (m.queryIdx < 0 || m.trainIdx < 0 ||
        static_cast<size_t>(m.queryIdx) >= curr.uv_px.size() ||
        static_cast<size_t>(m.trainIdx) >= prev.uv_px.size()) {
      continue;
    }
    const double zp = depth_at_m(depth_prev_mm, prev.uv_px[m.trainIdx], k, p.zmin_m, p.zmax_m);
    const double zc = depth_at_m(depth_curr_mm, curr.uv_px[m.queryIdx], k, p.zmin_m, p.zmax_m);
    if (zp <= 0.0 || zc <= 0.0) {
      continue;
    }
    src.push_back(unproject(k, curr.uv_px[m.queryIdx].x, curr.uv_px[m.queryIdx].y, zc));
    dst.push_back(unproject(k, prev.uv_px[m.trainIdx].x, prev.uv_px[m.trainIdx].y, zp));
  }

  const int n = static_cast<int>(src.size());
  if (n < std::max(3, p.min_inliers)) {
    return std::nullopt;
  }

  // 内点门限：0.02 m ⇒ 残差平方 4e-4。原实现写 < 0.04 并注释 "2 cm"，
  // 实际放进来的是 20 cm 的粗差，坏解会被烤进参考地图。
  const double thresh_sq = p.inlier_dist_m * p.inlier_dist_m;
  std::uniform_int_distribution<int> pick(0, n - 1);

  std::optional<Eigen::Isometry3d> best;
  int best_n = 0;
  for (int it = 0; it < p.ransac_iters; ++it) {
    const int i0 = pick(rng), i1 = pick(rng), i2 = pick(rng);
    if (i0 == i1 || i1 == i2 || i0 == i2) {
      continue;
    }
    // 三点共线时 Umeyama 仍返回一个行列式正常的旋转，但绕多余自由度是任意值。
    if (triangle_area(src[i0], src[i1], src[i2]) < p.min_sample_area_m2) {
      continue;
    }
    const auto T = umeyama_3d({src[i0], src[i1], src[i2]}, {dst[i0], dst[i1], dst[i2]});
    if (!T) {
      continue;
    }
    int inn = 0;
    for (int i = 0; i < n; ++i) {
      if (((*T) * src[i] - dst[i]).squaredNorm() < thresh_sq) {
        ++inn;
      }
    }
    if (inn > best_n) {
      best_n = inn;
      best = T;
    }
  }

  if (!best || best_n < p.min_inliers) {
    return std::nullopt;
  }

  std::vector<Eigen::Vector3d> si, di;
  si.reserve(static_cast<size_t>(best_n));
  di.reserve(static_cast<size_t>(best_n));
  for (int i = 0; i < n; ++i) {
    if (((*best) * src[i] - dst[i]).squaredNorm() < thresh_sq) {
      si.push_back(src[i]);
      di.push_back(dst[i]);
    }
  }
  const auto refined = umeyama_3d(si, di);
  if (!refined) {
    return std::nullopt;
  }

  SparseDelta out;
  out.T_prev_from_curr = *refined;
  out.inliers = static_cast<int>(si.size());
  out.candidates = n;
  return out;
}

}  // namespace follow
