// follow 模块的共享类型。单位约定：内部一律 SI（米 / 弧度 / 纳秒），
// 跨进程边界（ServoP、标定 yaml）才用 mm+deg，且只在 pose_io 那一层转换。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace follow {

// 针孔内参，像素单位。不含畸变系数：输入要求是已 D2C 对齐并去畸变的彩色系。
struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  int width = 0;
  int height = 0;

  // fx<=0 会让 unproject 除零，产生 inf/NaN 点云；而 NaN 躲得过所有范围比较，
  // 所以这道校验必须在构造期做，不能指望下游过滤。
  bool valid() const {
    const bool finite = std::isfinite(fx) && std::isfinite(fy) && std::isfinite(cx) && std::isfinite(cy);
    return finite && fx > 1e-6 && fy > 1e-6 && width > 0 && height > 0;
  }
};

enum class Status {
  kOk,              // 有位姿，可观
  kDegenerate,      // 解算器"成功"但自由度不可观 —— 绝不能当 kOk 用
  kOutOfEnvelope,   // 与参考地图重叠不足：需要重新示教，不是跟丢
  kLost,            // 两个解算器都失败
  kRotGated,        // 帧间旋转与陀螺积分互验失败：疑似坏帧，位姿保持上一可信值（P3 离群门）
  kNoDepth,         // 有效深度点不足
  kStaleInput,      // 帧过旧 / 时间戳倒退
  kConfigInvalid,   // 内参或参数不可用
  kDeviceGone,      // 相机被拔出
};

inline const char* to_string(Status s) {
  switch (s) {
    case Status::kOk: return "ok";
    case Status::kDegenerate: return "degenerate";
    case Status::kOutOfEnvelope: return "out_of_envelope";
    case Status::kLost: return "lost";
    case Status::kRotGated: return "rot_gated";
    case Status::kNoDepth: return "no_depth";
    case Status::kStaleInput: return "stale_input";
    case Status::kConfigInvalid: return "config_invalid";
    case Status::kDeviceGone: return "device_gone";
  }
  return "unknown";
}

inline bool usable(Status s) { return s == Status::kOk; }

// 特征前端输出。
// 契约：uv_px 必须是**全分辨率彩色图**的像素坐标。前端内部若为适配网络输入而
// resize，必须自己乘回缩放系数；下游拿它直接查深度图，不再做任何尺度换算。
struct FeatureFrame {
  std::vector<cv::Point2f> uv_px;
  cv::Mat desc;    // N x desc.cols() CV_32F，行已 L2 归一
  cv::Size image_size;
  int64_t ts_ns = 0;

  int count() const { return static_cast<int>(uv_px.size()); }
  bool empty() const { return uv_px.empty() || desc.empty(); }
};

struct GyroSample {
  int64_t ts_ns = 0;
  Eigen::Vector3d omega_cam_rad_s = Eigen::Vector3d::Zero();  // 相机/IMU 系，rad/s
};

// integrate_gyro 的结果。裸 bool 不算证据：调用方需要知道到底用了几个样本、
// 覆盖了多少时间、以及最后一条样本是不是已经很旧。
struct GyroDelta {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // R_{t0<-t1}，右乘 body 速率积分
  int samples_used = 0;
  int64_t span_ns = 0;   // 实际积分覆盖的时间跨度
  int64_t gap_end_ns = 0;  // 最后一样本到 t1 的缺口（IMU 停更的量度）
  bool stale = false;    // 覆盖不足或无样本：不要拿它当 R_init

  bool valid() const { return samples_used > 0 && !stale; }
};

// 把 [t0,t1] 之间的陀螺样本积成旋转（body 右乘）。样本须已旋到相机系，且按时间升序。
// 返回 R_{t0<-t1}：t1 时刻相机系坐标映回 t0 时刻相机系。
// 推导：R(t1) = R(t0)·ΔR_body ⇒ R_{t0<-t1} = R(t0)^T R(t1) = ΔR_body。
// buf 做成模板：运行期是 deque（有界头尾裁剪），单测里是 vector，不为了适配签名而拷一份。
template <typename SampleBuf>
inline GyroDelta integrate_gyro(const SampleBuf& buf, int64_t t0_ns, int64_t t1_ns,
                                int64_t max_gap_ns = 100'000'000) {
  GyroDelta out;
  if (t1_ns <= t0_ns || buf.empty()) {
    out.stale = true;
    return out;
  }

  const GyroSample* prev = nullptr;
  int64_t first_used = 0;
  int64_t last_ts = 0;
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
    prev = &s;
    if (dt <= 0.0) {
      continue;
    }
    last_ts = s.ts_ns;
    if (out.samples_used == 0) {
      first_used = ta;
    }
    ++out.samples_used;

    const double th = s.omega_cam_rad_s.norm();
    if (th > 1e-12) {
      out.R = out.R * Eigen::AngleAxisd(th * dt, s.omega_cam_rad_s / th).toRotationMatrix();
    }
  }

  out.span_ns = out.samples_used > 0 ? last_ts - first_used : 0;
  out.gap_end_ns = out.samples_used > 0 ? t1_ns - last_ts : t1_ns - t0_ns;
  // 积分覆盖不足请求区间的一半，或末样本到图像时刻还有明显缺口 → 认为不可信。
  out.stale = out.samples_used == 0 || out.gap_end_ns > max_gap_ns ||
              out.span_ns + max_gap_ns < (t1_ns - t0_ns);
  return out;
}

}  // namespace follow
