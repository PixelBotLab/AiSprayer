// RK3588：NPU SuperPoint 给初值，CPU small_gicp 扫局部地图做精配。
// 不要把两路位姿「平均」或拼 R+t；按置信度二选一。不用 ROS。
//
// 分工
// ----
// SuperPoint（NPU）: 纹理对应。白墙上海报、弱几何时能给出 yaw/沿平面滑动。
// small_gicp（A76）: 稠密几何。桌子、墙角、型材给出米制 6DoF。
// SuperPoint 不替代 GICP，只做两件事：
//    1) 给 GICP 平移初值（以及平面退化时的稀疏 3D-3D 回退）
//    2) GICP 信息矩阵退化时，用稀疏 3D-3D 顶上
//
// 336L 陀螺（不是完整 VINS）
// --------------------------
// 加计不要积分成位移。陀螺只做一件事：两帧之间积旋转，当作 GICP 的 R_init。
// 快转时 SuperPoint 糊、匹配少，陀螺仍然准（几十毫秒 bias 可忽略）。
// 初值允许「陀螺的 R + SuperPoint 的 t」——这只是初值，输出仍是 GICP（或 SP 回退）。
// 最终位姿不要改成「陀螺 R + 视觉 t」直接发给 ServoP。
// 这不是 OpenVINS：没有 bias 估计、没有加速度约束、遮挡超过几百毫秒仍会丢。
//
// 第一版不要上 LightGlue：300 个点的互近邻匹配在 CPU 上很便宜。
//
// 数据流（每帧）
//    对齐 RGB-D + 缓冲陀螺（图像时间戳之前的样本）
//     → NPU SuperPoint → 3D-3D Umeyama 得 T_sp（主要用它的平移）
//     → 陀螺积 ΔR
//     → T_init.R = ΔR，T_init.t = T_sp.t（否则恒速平移）
//     → GICP(当前云 → 局部地图, T_init)
//     → 输出 T_gicp；退化则 T_sp；再没有则跟丢
//     → 推进滑动窗口地图

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/registration/registration_result.hpp>

namespace follow {

struct CameraK {
  float fx = 0, fy = 0, cx = 0, cy = 0;
};

struct SpFrame {
  std::vector<cv::Point2f> uv;  // 像素
  cv::Mat desc;                 // Nx256 float32，行向量已 L2 归一
};

struct TrackStatus {
  bool ok = false;
  bool used_superpoint_fallback = false;
  bool used_gyro_init = false;
  bool gicp_degenerate = false;
  int sp_inliers = 0;
  int gicp_inliers = 0;
  Eigen::Isometry3d T_world_cam = Eigen::Isometry3d::Identity();
};

struct GyroSample {
  int64_t ts_ns = 0;
  Eigen::Vector3d omega_cam = Eigen::Vector3d::Zero();  // rad/s，相机/IMU 系（336L 近似重合）
};

// 两图像时刻之间把陀螺积成 ΔR（body 右乘）。样本须已是相机系。
// 返回 R_{prev<-curr}，即 T_prev_from_curr 的旋转部分。
inline Eigen::Matrix3d integrate_gyro(const std::deque<GyroSample>& buf, int64_t t0_ns,
                                      int64_t t1_ns) {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  if (t1_ns <= t0_ns) {
    return R;
  }
  const GyroSample* prev = nullptr;
  for (const auto& s : buf) {
    if (s.ts_ns <= t0_ns) {
      prev = &s;
      continue;
    }
    if (s.ts_ns > t1_ns) {
      break;
    }
    const int64_t ta = prev ? std::max(prev->ts_ns, t0_ns) : t0_ns;
    const double dt = static_cast<double>(s.ts_ns - ta) * 1e-9;
    if (dt <= 0.0) {
      prev = &s;
      continue;
    }
    const Eigen::Vector3d w = s.omega_cam;
    const double th = w.norm();
    if (th > 1e-12) {
      R = R * Eigen::AngleAxisd(th * dt, w / th).toRotationMatrix();
    }
    prev = &s;
  }
  return R;
}

inline Eigen::Vector3d unproject(const CameraK& k, float u, float v, float z_m) {
  return {(u - k.cx) * z_m / k.fx, (v - k.cy) * z_m / k.fy, z_m};
}

// 深度图：uint16 毫米，0 为无效。与彩色已对齐。
inline std::vector<Eigen::Vector3f> depth_to_cloud(const cv::Mat& depth_mm, const CameraK& k,
                                                   float zmin, float zmax, int stride) {
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(depth_mm.rows * depth_mm.cols / (stride * stride) / 2);
  for (int v = 0; v < depth_mm.rows; v += stride) {
    const uint16_t* row = depth_mm.ptr<uint16_t>(v);
    for (int u = 0; u < depth_mm.cols; u += stride) {
      const float z = row[u] * 0.001f;
      if (z < zmin || z > zmax) {
        continue;
      }
      const Eigen::Vector3d p = unproject(k, static_cast<float>(u), static_cast<float>(v), z);
      pts.emplace_back(p.cast<float>());
    }
  }
  return pts;
}

inline float depth_at(const cv::Mat& depth_mm, const cv::Point2f& uv, float zmin, float zmax) {
  const int u = static_cast<int>(std::lround(uv.x));
  const int v = static_cast<int>(std::lround(uv.y));
  if (u < 0 || v < 0 || u >= depth_mm.cols || v >= depth_mm.rows) {
    return 0.f;
  }
  const float z = depth_mm.at<uint16_t>(v, u) * 0.001f;
  if (z < zmin || z > zmax) {
    return 0.f;
  }
  return z;
}

// 互近邻 + Lowe。desc 是 Nx256。
inline std::vector<cv::DMatch> mutual_nn(const cv::Mat& a, const cv::Mat& b, float ratio = 0.8f) {
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

// 3D-3D 刚体：Eigen Umeyama。点数 < 3 返回 nullopt。
inline std::optional<Eigen::Isometry3d> umeyama_3d(const std::vector<Eigen::Vector3d>& src,
                                                   const std::vector<Eigen::Vector3d>& dst) {
  if (src.size() < 3 || src.size() != dst.size()) {
    return std::nullopt;
  }
  Eigen::MatrixXd A(3, static_cast<int>(src.size()));
  Eigen::MatrixXd B(3, static_cast<int>(dst.size()));
  for (int i = 0; i < static_cast<int>(src.size()); ++i) {
    A.col(i) = src[i];
    B.col(i) = dst[i];
  }
  const Eigen::Matrix4d T = Eigen::umeyama(A, B, false);
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.matrix() = T;
  return iso;
}

// 用深度把匹配变成相机系 3D-3D，再 RANSAC 式剔一次（重投误差）。
// 返回 T_prev_from_curr（把当前相机系点映回上一帧相机系）。
inline std::optional<std::pair<Eigen::Isometry3d, int>> superpoint_delta(
    const SpFrame& prev, const SpFrame& curr, const std::vector<cv::DMatch>& matches,
    const cv::Mat& depth_prev, const cv::Mat& depth_curr, const CameraK& k, float zmin,
    float zmax) {
  std::vector<Eigen::Vector3d> src, dst;
  src.reserve(matches.size());
  dst.reserve(matches.size());
  for (const auto& m : matches) {
    const float zp = depth_at(depth_prev, prev.uv[m.trainIdx], zmin, zmax);
    const float zc = depth_at(depth_curr, curr.uv[m.queryIdx], zmin, zmax);
    if (zp <= 0.f || zc <= 0.f) {
      continue;
    }
    src.push_back(unproject(k, curr.uv[m.queryIdx].x, curr.uv[m.queryIdx].y, zc));
    dst.push_back(unproject(k, prev.uv[m.trainIdx].x, prev.uv[m.trainIdx].y, zp));
  }
  if (src.size() < 12) {
    return std::nullopt;
  }

  // 简单 RANSAC：多次随机 3 点 Umeyama，留内点再估一次
  std::optional<Eigen::Isometry3d> best;
  int best_n = 0;
  const int n = static_cast<int>(src.size());
  for (int it = 0; it < 32; ++it) {
    const int i0 = rand() % n, i1 = rand() % n, i2 = rand() % n;
    if (i0 == i1 || i1 == i2 || i0 == i2) {
      continue;
    }
    auto T = umeyama_3d({src[i0], src[i1], src[i2]}, {dst[i0], dst[i1], dst[i2]});
    if (!T) {
      continue;
    }
    int inn = 0;
    for (int i = 0; i < n; ++i) {
      if (((*T) * src[i] - dst[i]).squaredNorm() < 0.04) {  // 2 cm
        ++inn;
      }
    }
    if (inn > best_n) {
      best_n = inn;
      best = T;
    }
  }
  if (!best || best_n < 12) {
    return std::nullopt;
  }
  std::vector<Eigen::Vector3d> si, di;
  for (int i = 0; i < n; ++i) {
    if (((*best) * src[i] - dst[i]).squaredNorm() < 0.04) {
      si.push_back(src[i]);
      di.push_back(dst[i]);
    }
  }
  auto T2 = umeyama_3d(si, di);
  if (!T2) {
    return std::nullopt;
  }
  return std::make_pair(*T2, static_cast<int>(si.size()));
}

// 6x6 GICP Hessian：最小特征值过小 = 某个自由度不可观（典型：大平面）。
inline bool hessian_degenerate(const Eigen::Matrix<double, 6, 6>& H, double ratio = 1e-3) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H);
  const auto ev = es.eigenvalues();
  if (ev.maxCoeff() <= 1e-12) {
    return true;
  }
  return ev.minCoeff() / ev.maxCoeff() < ratio;
}

class LocalMap {
 public:
  explicit LocalMap(int keep_scans = 12) : keep_(keep_scans) {}

  void push_world(const std::vector<Eigen::Vector3f>& cam_pts, const Eigen::Isometry3d& T_wc) {
    std::vector<Eigen::Vector3f> w;
    w.reserve(cam_pts.size());
    for (const auto& p : cam_pts) {
      w.push_back((T_wc * p.cast<double>()).cast<float>());
    }
    scans_.push_back(std::move(w));
    while (static_cast<int>(scans_.size()) > keep_) {
      scans_.pop_front();
    }
  }

  std::vector<Eigen::Vector3f> concat() const {
    std::vector<Eigen::Vector3f> all;
    size_t n = 0;
    for (const auto& s : scans_) {
      n += s.size();
    }
    all.reserve(n);
    for (const auto& s : scans_) {
      all.insert(all.end(), s.begin(), s.end());
    }
    return all;
  }

  bool empty() const { return scans_.empty(); }

 private:
  int keep_;
  std::deque<std::vector<Eigen::Vector3f>> scans_;
};

class SpGicpOdometry {
 public:
  struct Params {
    CameraK k;
    float zmin = 0.30f;
    float zmax = 2.50f;
    int depth_stride = 4;  // 640x480 → 约 2 万点再体素
    double voxel = 0.03;   // 3 cm，工位跟随够用
    double max_corr = 0.12;     // GICP 最大对应距离
    int gicp_threads = 4;       // 绑 A76，不要 8
    int min_sp_inliers = 12;
    int min_gicp_inliers = 80;
    size_t gyro_buf_max = 4096;
  };

  explicit SpGicpOdometry(Params p) : p_(std::move(p)) {
    gicp_.num_threads = p_.gicp_threads;
    gicp_.downsampling_resolution = p_.voxel;
    gicp_.max_correspondence_distance = p_.max_corr;
    gicp_.type = small_gicp::RegistrationSetting::GICP;
    gicp_.max_iterations = 15;
  }

  // 336L 陀螺回调：与图像同一硬件时钟，单位 rad/s，坐标系与相机近似重合。
  void push_gyro(int64_t ts_ns, const Eigen::Vector3d& omega_cam) {
    if (!gyro_buf_.empty() && ts_ns <= gyro_buf_.back().ts_ns) {
      return;
    }
    gyro_buf_.push_back({ts_ns, omega_cam});
    while (gyro_buf_.size() > p_.gyro_buf_max) {
      gyro_buf_.pop_front();
    }
  }

  TrackStatus track(const SpFrame& sp, const cv::Mat& depth_mm, int64_t ts_ns) {
    TrackStatus st;
    auto cloud = depth_to_cloud(depth_mm, p_.k, p_.zmin, p_.zmax, p_.depth_stride);
    if (cloud.size() < 200) {
      return st;
    }

    if (!have_prev_) {
      T_wc_ = Eigen::Isometry3d::Identity();
      map_.push_world(cloud, T_wc_);
      prev_sp_ = sp;
      prev_depth_ = depth_mm.clone();
      prev_ts_ns_ = ts_ns;
      have_prev_ = true;
      st.ok = true;
      st.T_world_cam = T_wc_;
      return st;
    }

    // --- SuperPoint：ΔT_prev_from_curr，初值主要用它的平移 ---
    const auto matches = mutual_nn(sp.desc, prev_sp_.desc);
    auto sp_delta = superpoint_delta(prev_sp_, sp, matches, prev_depth_, depth_mm, p_.k, p_.zmin,
                                     p_.zmax);
    const int sp_n = (sp_delta && sp_delta->second >= p_.min_sp_inliers) ? sp_delta->second : 0;
    st.sp_inliers = sp_n;

    // --- 陀螺 ΔR + 视觉 t → 只作 T_init ---
    const Eigen::Matrix3d dR_gyro = integrate_gyro(gyro_buf_, prev_ts_ns_, ts_ns);
    Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
    if (!gyro_buf_.empty()) {
      dT.linear() = dR_gyro;
      st.used_gyro_init = true;
    } else {
      dT.linear() = T_vel_.linear();
    }
    if (sp_n > 0) {
      dT.translation() = sp_delta->first.translation();
    } else {
      dT.translation() = T_vel_.translation();
    }
    const Eigen::Isometry3d T_init = T_wc_ * dT;

    // --- GICP：当前相机点云 → 世界系局部地图 ---
    const auto target = map_.concat();
    small_gicp::RegistrationSetting set = gicp_;
    const small_gicp::RegistrationResult g = small_gicp::align(target, cloud, T_init, set);
    st.gicp_inliers = static_cast<int>(g.num_inliers);
    st.gicp_degenerate = hessian_degenerate(g.H);

    const bool gicp_ok =
        g.converged && st.gicp_inliers >= p_.min_gicp_inliers && !st.gicp_degenerate;

    const Eigen::Isometry3d T_prev = T_wc_;
    if (gicp_ok) {
      T_wc_ = g.T_target_source;
      st.ok = true;
    } else if (sp_delta && sp_n >= p_.min_sp_inliers) {
      T_wc_ = T_prev * sp_delta->first;
      st.ok = true;
      st.used_superpoint_fallback = true;
    } else {
      // 跟丢：地图不动、停 ServoP。时间戳仍推进，避免下次把陀螺积成一大段。
      prev_sp_ = sp;
      prev_depth_ = depth_mm.clone();
      prev_ts_ns_ = ts_ns;
      return st;
    }

    T_vel_ = T_prev.inverse() * T_wc_;
    map_.push_world(cloud, T_wc_);
    prev_sp_ = sp;
    prev_depth_ = depth_mm.clone();
    prev_ts_ns_ = ts_ns;
    while (!gyro_buf_.empty() && gyro_buf_.front().ts_ns + 200000000 < ts_ns) {
      gyro_buf_.pop_front();
    }
    st.T_world_cam = T_wc_;
    return st;
  }

  Eigen::Isometry3d T() const { return T_wc_; }

 private:
  Params p_;
  small_gicp::RegistrationSetting gicp_;
  LocalMap map_;
  bool have_prev_ = false;
  SpFrame prev_sp_;
  cv::Mat prev_depth_;
  int64_t prev_ts_ns_ = 0;
  std::deque<GyroSample> gyro_buf_;
  Eigen::Isometry3d T_wc_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_vel_ = Eigen::Isometry3d::Identity();
};

}  // namespace follow
