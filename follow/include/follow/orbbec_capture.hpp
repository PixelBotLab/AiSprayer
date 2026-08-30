// Orbbec Gemini 336L 取流。这份接口里的每个默认值和每条注释都对应一次实测，不是设计偏好：
//
//   * **彩色比深度晚上线约 333 ms**（1280x800@15 下暖机 5 个帧组、@30 下 10 个 —— 固定时长而
//     非固定帧数）。所以 open() 必须等到**第一个成对帧组**才算就绪，不能"拿到第一帧深度就返回"：
//     否则示教会在没有彩色的时候发生，冻结出来的参考地图是一个瞎的基准，之后每一帧都跟它比。
//   * **硬件 D2C 支持情况**：getD2CDepthProfileList(..., ALIGN_D2C_HW_MODE) 实测在 640x480、640x400、
//     640x360、480x270、424x240 等分辨率下均**完整支持硬件 D2C（ASIC 计算，0% 主机 CPU 占用，最高 90fps）**；
//     而在 848x480、1280x720、1280x800 上不支持硬件 D2C（返回 0 个 profile），只能回退到 SW D2C（实测
//     SW D2C 在 848x480 约 16~22 ms/帧，1280x800 约 29~35 ms/帧）。因此最大硬件 D2C 分辨率为 640x480。
//   * 因此**回退到 ALIGN_DISABLE 不是降级，是错**：整条下游（depth_to_cloud、特征→3D）都假设
//     深度和彩色共用一套内参。不对齐时深度有自己的 fx=624.013/cx=642，彩色是 611.684/643.429，
//     两者差 23.735 mm 基线 —— 拿彩色内参反投影原始深度会得到一个自洽但错位的场景，配准会"很
//     准"地量到错的东西。allow_unaligned 只作为排障逃生口，且必须显式打开。
//   * 对齐后的深度落在**彩色分辨率**网格上（实测 1280x800 请求 → 交付 depth 1280x800），所以
//     用 rgb 内参反投影它。至于 rgb 图像本身是否已被 SDK 整平：设备给的 rgb_distortion 是
//     rational 模型 k1=-0.032/k2=0.0345/k3=-0.012（角上约 5~8 px），depth_distortion 全零，
//     本机实测没能定论（判据见 tools/analyze_d2c.py）。这里**照实上报**，不在设备层偷偷假设。
//   * valueScale 实测 = 1 ⇒ 原始 uint16 就是毫米，follow 内部一律米，换算只在这一层做一次。
//   * getTimeStampUs 是**设备运行时**时钟（两次运行末值分别 1.46e9/1.54e9 us 且单调），不是主机
//     时钟。对外时间戳一律用它换算到 ns —— 混用主机时钟会让"帧过旧"的判断随风摇摆。
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "follow/device_lock.hpp"
#include "follow/types.hpp"

namespace follow {

enum class Align { kHwD2c, kSwD2c, kNone };

const char* to_string(Align a);

// OpenCV 顺序的 5 系数畸变（有理模型时 k3 在分母恒为 1 的前提下仍按分子用）。
struct Distortion {
  std::string model = "none";
  double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0;

  bool all_zero() const;
  // 归一化半径 r 处沿半径方向的像素位移。把"畸变能不能忽略"变成一个可打印的数，而不是注释里
  // 的一句"应该已经去畸变了"。
  double radial_shift_px(const CameraIntrinsics& k, double r_norm) const;
  double corner_shift_px(const CameraIntrinsics& k) const;
};

// 设备自报的标定，按**实际启用的分辨率**取（getCameraParamWithProfile，不是 158 组里猜一行）。
struct DeviceCalib {
  CameraIntrinsics color;
  CameraIntrinsics raw_depth;
  Distortion color_dist;
  Distortion depth_dist;
  double baseline_mm = 0.0;   // D2C 平移的 x 分量，实测 -23.735
  Eigen::Matrix3d R_cam_gyro = Eigen::Matrix3d::Identity();  // 陀螺仪系 -> 相机系旋转矩阵
  Eigen::Vector3d t_cam_gyro = Eigen::Vector3d::Zero();      // 陀螺仪系 -> 相机系平移向量(mm)
  bool has_imu = false;
  int gyro_sample_rate_hz = 0;
  bool valid() const { return color.valid(); }
};

struct CaptureParams {
  int width = 848;
  int height = 480;
  int fps = 30;
  int first_pair_timeout_ms = 3000;  // 暖机 333 ms 的余量
  int frame_timeout_ms = 1000;
  bool allow_unaligned = false;      // 只给排障；生产路径下 false 是正确性要求
  bool enable_imu = true;            // 开启板载 6 轴 IMU 陀螺仪数据流
  std::string lock_path;             // 空 = 不做进程间仲裁（回放/单测）
};

struct RgbdFrame {
  cv::Mat color;      // CV_8UC3 BGR，全分辨率
  cv::Mat depth_mm;   // CV_16UC1，已与彩色同网格
  int64_t ts_ns = 0;        // 深度帧设备时间戳（ns）
  int64_t color_ts_ns = 0;  // 彩色帧设备时间戳；两者差即配对抖动
  Align align = Align::kNone;
};

struct CaptureHealth {
  int64_t frames = 0;
  int64_t unpaired_framesets = 0;  // 只含深度、被丢弃的暖机/丢帧帧组
  int64_t dropouts = 0;            // 间隔 > 1.5 个帧周期
  double period_ms = 0.0;          // 滑动平均
  double max_period_ms = 0.0;
  double d2c_offset_ms = 0.0;      // color_ts - depth_ts 滑动平均，实测常量 +0.347
  bool device_present = true;
  bool lock_held = false;
  std::string last_error;
};

// 不是线程安全的：一个实例一条取流线程。设备拔出由 SDK 回调置位，wait_frame 会以
// Status 可表达的方式报出来，而不是空转到超时。
class OrbbecCapture {
 public:
  OrbbecCapture();
  ~OrbbecCapture();
  OrbbecCapture(const OrbbecCapture&) = delete;
  OrbbecCapture& operator=(const OrbbecCapture&) = delete;

  // 枚举设备名，供启动日志和"到底有没有插"的判断用。**不能**拿它当互斥判据。
  static std::vector<std::string> list_devices(std::string* err);

  // 取锁 → 开设备 → 选 profile → 对齐阶梯(HW→SW→DISABLE) → 等第一个成对帧组 → 读标定。
  // 任何一步失败都把原因写进 err 并回滚已拿的锁。
  bool open(const CaptureParams& p, std::string* err);
  void close();

  // 阻塞拿一帧。false = 出错/设备没了/超时，原因在 err 与 health().last_error。
  bool wait_frame(RgbdFrame* out, std::string* err);

  // 取出并清空积攒的陀螺仪数据（样本已旋至相机坐标系，单位 rad/s，时间升序）
  bool drain_gyro_samples(std::vector<GyroSample>* out);

  const DeviceCalib& calib() const;
  Align align() const;
  CaptureHealth health() const;
  bool is_open() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> im_;
};

}  // namespace follow
