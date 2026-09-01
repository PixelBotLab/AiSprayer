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

// 设备给的 T_cam_gyro 有时是全零、不是 Identity。isApprox(I) 过不去会被当成"已标定"，
// 之后 ω 全变成 0，静止检测永远判静。正交性放宽到 1e-2：出厂矩阵常是 float。
inline bool is_valid_rotation(const Eigen::Matrix3d& R) {
  if (!R.allFinite()) {
    return false;
  }
  return std::abs(R.determinant() - 1.0) < 0.05 &&
         (R * R.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-2);
}

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
  int64_t span_ns = 0;   // 真实样本覆盖的时间跨度（不含外推段）
  int64_t gap_end_ns = 0;  // 最后一条真实样本到 t1 的缺口（IMU 停更的量度）
  int64_t extrap_ns = 0;   // gap_end 里用末样本角速度常值补积掉的时长，见 integrate_gyro
  bool stale = false;      // 无可信覆盖：不要拿它当 R_init

  // 真实样本与补积段都不足以覆盖这个区间才算无效 —— "窗口内一条都没到货"是常态
  // （交付延迟 ≥ 一个帧周期时），此时末样本常值外推仍是比"假设没动"好得多的初值。
  bool valid() const { return !stale; }
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
  const GyroSample* last = nullptr;
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
    last = &s;
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

  // 末段补积：IMU 交付比帧交付晚约半个帧周期（真机 66ms 窗口尾部固定缺 25~34ms），
  // 所以"窗口尾部没有样本"是结构，不是故障。不补的话 R_init 和离群门用的 ΔR 只覆盖
  // 一半时间，转速越高少转越多（15 fps 下 10 dps 就少算 0.33°，而 P3 门的标定目标是 2°、
  // 现场配到 10°）—— 门限一收紧就会在好帧上误触发，且触发后不更新参照，是正反馈。
  // 判据只用**最后一条到货样本**的角速度
  // 常值外推（33ms 尺度上足够准），窗口内一条都没到货时退回 t0 前最后一条。
  // 缺口大过 max_gap_ns 时不补：那是 IMU 停更，拿半秒前的速率外推等于凭空造旋转。
  const GyroSample* tail = last ? last : prev;
  if (tail != nullptr) {
    const int64_t miss = t1_ns - tail->ts_ns;
    if (miss > 0 && miss <= max_gap_ns) {
      const double dt = static_cast<double>(miss) * 1e-9;
      const double th = tail->omega_cam_rad_s.norm();
      if (th > 1e-12) {
        out.R = out.R * Eigen::AngleAxisd(th * dt, tail->omega_cam_rad_s / th).toRotationMatrix();
      }
      out.extrap_ns = miss;
    }
  }
  // 既没有真实样本、也没补出东西来 ⇒ 这个区间完全没被测量；缺口超上限 ⇒ IMU 停更。
  out.stale = (out.samples_used == 0 && out.extrap_ns == 0) || out.gap_end_ns > max_gap_ns;
  return out;
}

}  // namespace follow
