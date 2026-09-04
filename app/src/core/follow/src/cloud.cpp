#include "follow/cloud.hpp"

#include <cmath>

namespace follow {

Eigen::Vector3d unproject(const CameraIntrinsics& k, double u_px, double v_px, double z_m) {
  return {(u_px - k.cx) * z_m / k.fx, (v_px - k.cy) * z_m / k.fy, z_m};
}

double depth_at_m(const cv::Mat& depth_mm, const cv::Point2f& uv, const CameraIntrinsics& k,
                  double zmin_m, double zmax_m) {
  if (depth_mm.empty() || depth_mm.type() != CV_16UC1 || !k.valid()) {
    return 0.0;
  }
  const int u = static_cast<int>(std::lround(uv.x));
  const int v = static_cast<int>(std::lround(uv.y));
  if (u < 0 || v < 0 || u >= depth_mm.cols || v >= depth_mm.rows) {
    return 0.0;
  }
  const double z_m = depth_mm.at<uint16_t>(v, u) * 0.001;
  // NaN 躲得过 `z < zmin || z > zmax`（任何与 NaN 的比较都是 false），所以这里
  // 必须先做 isfinite；毫米读出来的 z 本身不会 NaN，但别依赖上游的诚实。
  if (!std::isfinite(z_m) || z_m < zmin_m || z_m > zmax_m) {
    return 0.0;
  }
  return z_m;
}

std::vector<Eigen::Vector3f> depth_to_cloud(const cv::Mat& depth_mm, const CameraIntrinsics& k,
                                            double zmin_m, double zmax_m, int stride,
                                            CloudStats* stats, Status* status) {
  const int step = std::max(1, stride);
  auto fail = [&](Status s) {
    if (status) {
      *status = s;
    }
    return std::vector<Eigen::Vector3f>();
  };

  if (!k.valid()) {
    return fail(Status::kConfigInvalid);
  }
  if (depth_mm.empty() || depth_mm.type() != CV_16UC1) {
    return fail(Status::kNoDepth);
  }
  if (!(zmin_m > 0.0) || zmax_m <= zmin_m) {
    return fail(Status::kConfigInvalid);
  }

  CloudStats local;
  local.total_samples = ((depth_mm.rows + step - 1) / step) * ((depth_mm.cols + step - 1) / step);
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(static_cast<size_t>(local.total_samples) / 2);

  for (int v = 0; v < depth_mm.rows; v += step) {
    const uint16_t* row = depth_mm.ptr<uint16_t>(v);
    for (int u = 0; u < depth_mm.cols; u += step) {
      const double z_m = row[u] * 0.001;
      if (z_m < zmin_m || z_m > zmax_m) {
        ++local.rejected_range;
        continue;
      }
      const Eigen::Vector3d p = unproject(k, u, v, z_m);
      if (!p.allFinite()) {
        ++local.rejected_nonfinite;
        continue;
      }
      pts.emplace_back(p.cast<float>());
      ++local.kept;
    }
  }

  if (stats) {
    *stats = local;
  }
  if (status) {
    *status = pts.empty() ? Status::kNoDepth : Status::kOk;
  }
  return pts;
}

}  // namespace follow
