#include "follow/orbbec_capture.hpp"

#include <sys/time.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

#include <libobsensor/ObSensor.hpp>

#include <opencv2/imgproc.hpp>

namespace follow {
namespace {

// 帧周期判定用的宽容度：实测暖机后 Δ 常量 +0.347 ms、周期 33.36 ms，所以 1.5 倍已经是很大的
// 让步，再宽就变成"永远不报 dropout"。
constexpr double kDropoutFactor = 1.5;

const char* format_name(OBFormat f) {
  switch (f) {
    case OB_FORMAT_BGR: return "BGR";
    case OB_FORMAT_RGB: return "RGB";
    case OB_FORMAT_MJPG: return "MJPG";
    case OB_FORMAT_Y16: return "Y16";
    case OB_FORMAT_YUYV: return "YUYV";
    default: return "other";
  }
}

const char* distortion_model_name(OBCameraDistortionModel m) {
  switch (m) {
    case OB_DISTORTION_NONE: return "none";
    case OB_DISTORTION_MODIFIED_BROWN_CONRADY: return "modified_brown_conrady";
    case OB_DISTORTION_INVERSE_BROWN_CONRADY: return "inverse_brown_conrady";
    case OB_DISTORTION_BROWN_CONRADY: return "brown_conrady";
    case OB_DISTORTION_BROWN_CONRADY_K6: return "brown_conrady_k6";
    case OB_DISTORTION_KANNALA_BRANDT4: return "kannala_brandt4";
  }
  return "unknown";
}

Distortion to_distortion(const OBCameraDistortion& d) {
  Distortion out;
  out.model = distortion_model_name(d.model);
  out.k1 = d.k1;
  out.k2 = d.k2;
  out.k3 = d.k3;
  out.p1 = d.p1;
  out.p2 = d.p2;
  return out;
}

CameraIntrinsics to_intrinsics(const OBCameraIntrinsic& i, int w, int h) {
  CameraIntrinsics k;
  k.fx = i.fx;
  k.fy = i.fy;
  k.cx = i.cx;
  k.cy = i.cy;
  k.width = w;
  k.height = h;
  return k;
}

// 彩色格式必须挑死：OB_FORMAT_ANY 会挑中 MJPG，而 pipeline 用 MJPG 起得来、**一帧彩色都不交
// 付**（实测），量出来像个"取流正常"。RGB 退而求其次，但要显式换算成 BGR 再交给下游。
struct PickedColor {
  std::shared_ptr<ob::VideoStreamProfile> prof;
  bool needs_swap = false;
  std::string detail;
};

PickedColor pick_color(const std::shared_ptr<ob::StreamProfileList>& list, int w, int h, int fps) {
  PickedColor out;
  for (const OBFormat f : {OB_FORMAT_BGR, OB_FORMAT_RGB}) {
    std::shared_ptr<ob::VideoStreamProfile> p;
    try {
      p = list->getVideoStreamProfile(w, h, f, fps);
    } catch (const ob::Error&) {
      continue;
    }
    if (p) {
      out.prof = p;
      out.needs_swap = (f == OB_FORMAT_RGB);
      out.detail = format_name(f);
      return out;
    }
  }
  return out;
}

std::shared_ptr<ob::VideoStreamProfile> pick_depth(
    const std::shared_ptr<ob::StreamProfileList>& list, int w, int h, int fps) {
  try {
    return list->getVideoStreamProfile(w, h, OB_FORMAT_Y16, fps);
  } catch (const ob::Error&) {
    return nullptr;
  }
}

}  // namespace

const char* to_string(Align a) {
  switch (a) {
    case Align::kHwD2c: return "hw_d2c";
    case Align::kSwD2c: return "sw_d2c";
    case Align::kNone: return "none";
  }
  return "unknown";
}

bool Distortion::all_zero() const {
  return k1 == 0.0 && k2 == 0.0 && k3 == 0.0 && p1 == 0.0 && p2 == 0.0;
}

double Distortion::radial_shift_px(const CameraIntrinsics& k, double r_norm) const {
  if (all_zero() || r_norm <= 0.0) {
    return 0.0;
  }
  const double r2 = r_norm * r_norm;
  const double rad = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
  // 切向项同样按 OpenCV 写在归一化坐标上；这里取纯径向位移作为"最坏有多少像素"的估计。
  return (rad - 1.0) * r_norm * k.fx;
}

// 取四个角里最坏的那个。不能用 hypot(宽/2, 高/2) 那类写法：主轴不同的两个偏移拼起来既不是任何
// 一个真实像素，而且 cx 一旦落在 width/2 附近，半径会缩到接近 0 —— 正好在这个数存在的理由
// （"畸变能不能忽略"）上谎报成"能忽略"。
double Distortion::corner_shift_px(const CameraIntrinsics& k) const {
  if (!k.valid()) {
    return 0.0;
  }
  double worst = 0.0;
  for (const double u : {0.0, static_cast<double>(k.width)}) {
    for (const double v : {0.0, static_cast<double>(k.height)}) {
      const double r = std::hypot((u - k.cx) / k.fx, (v - k.cy) / k.fy);
      worst = std::max(worst, std::abs(radial_shift_px(k, r)));
    }
  }
  return worst;
}

struct OrbbecCapture::Impl {
  CaptureParams p;
  Align align = Align::kNone;
  DeviceCalib calib;
  DeviceLock lock;
  CaptureHealth health;
  mutable std::mutex mtx;

  std::unique_ptr<ob::Context> ctx;
  std::shared_ptr<ob::Device> dev;
  std::unique_ptr<ob::Pipeline> pipe;
  std::shared_ptr<ob::Config> cfg;
  std::shared_ptr<ob::StreamProfileList> color_list;
  std::shared_ptr<ob::VideoStreamProfile> color_prof;
  bool color_needs_swap = false;

  std::shared_ptr<ob::Sensor> gyro_sensor;
  std::deque<GyroSample> gyro_queue;
  mutable std::mutex gyro_mtx;

  std::atomic<bool> present{true};
  OBCallbackId cb_id{};
  bool cb_registered = false;

  double depth_scale_mm = 1.0;
  int color_w = 0, color_h = 0, depth_w = 0, depth_h = 0;
  double nominal_period_ms = 33.3;
  double prev_dev_ts_ms = -1.0;
  bool opened = false;

  ~Impl() { shutdown(); }

  std::string note(const std::string& why) {
    std::lock_guard<std::mutex> g(mtx);
    health.last_error = why;
    return why;
  }

  void shutdown() {
    if (gyro_sensor) {
      try {
        gyro_sensor->stop();
      } catch (const ob::Error&) {}
      gyro_sensor.reset();
    }
    {
      std::lock_guard<std::mutex> g(gyro_mtx);
      gyro_queue.clear();
    }
    if (pipe) {
      try {
        pipe->stop();
      } catch (const ob::Error& e) {
        note(std::string("pipeline stop 异常: ") + e.getMessage());
      }
    }
    pipe.reset();
    cfg.reset();
    color_prof.reset();
    color_list.reset();
    dev.reset();
    if (ctx && cb_registered) {
      try {
        ctx->unregisterDeviceChangedCallback(cb_id);
      } catch (const ob::Error&) {
        // 回调注销失败不影响退出：进程都要走了
      }
      cb_registered = false;
    }
    // Context 必须最后销毁：它持有设备与回调的底层资源。
    ctx.reset();
    lock.release();
    opened = false;
  }
};

std::vector<std::string> OrbbecCapture::list_devices(std::string* err) {
  std::vector<std::string> out;
  try {
    // 单独造一个 Context 只为枚举：会真的初始化 SDK（几百 ms），所以只在启动阶段调一次，
    // 不要放进重连循环。
    ob::Context c;
    auto dl = c.queryDeviceList();
    if (!dl) {
      return out;
    }
    for (uint32_t i = 0; i < dl->deviceCount(); ++i) {
      auto info = dl->getDevice(i)->getDeviceInfo();
      out.push_back(std::string(info->name()) + " SN=" + info->serialNumber() + " FW=" +
                    info->firmwareVersion());
    }
  } catch (const ob::Error& e) {
    if (err) {
      *err = std::string("枚举设备失败: ") + e.getMessage();
    }
  }
  return out;
}

OrbbecCapture::OrbbecCapture() : im_(std::make_unique<Impl>()) {}

OrbbecCapture::~OrbbecCapture() { close(); }

bool OrbbecCapture::open(const CaptureParams& p, std::string* err) {
  close();
  Impl& s = *im_;
  s.p = p;
  s.nominal_period_ms = p.fps > 0 ? 1000.0 / static_cast<double>(p.fps) : 33.3;

  if (!p.lock_path.empty()) {
    DeviceLock::Busy busy;
    if (!s.lock.acquire(p.lock_path, err, &busy)) {
      return false;  // 拿不到锁就**不碰设备**：抢开一台已被占用的相机是数据损坏的源头
    }
  }

  try {
    s.ctx = std::make_unique<ob::Context>();
    s.cb_id = s.ctx->registerDeviceChangedCallback(
        [&s](std::shared_ptr<ob::DeviceList> removed, std::shared_ptr<ob::DeviceList> added) {
          // 回调线程不是取流线程：只置位，不在这里碰 pipeline。
          const uint32_t nr = removed ? removed->deviceCount() : 0;
          const uint32_t na = added ? added->deviceCount() : 0;
          if (nr > 0 && na == 0) {
            s.present.store(false);
          } else if (na > 0) {
            s.present.store(true);
          }
        });
    s.cb_registered = true;

    auto dl = s.ctx->queryDeviceList();
    if (!dl || dl->deviceCount() == 0) {
      s.shutdown();
      if (err) {
        *err = "没有可用 Orbbec 设备（/dev/video* 在不代表 SDK 能开；已释放锁）";
      }
      return false;
    }
    s.dev = dl->getDevice(0);
    s.pipe = std::make_unique<ob::Pipeline>(s.dev);
    s.cfg = std::make_shared<ob::Config>();

    auto picked = pick_color(s.pipe->getStreamProfileList(OB_SENSOR_COLOR), p.width, p.height,
                             p.fps);
    s.color_list = s.pipe->getStreamProfileList(OB_SENSOR_COLOR);
    if (!picked.prof) {
      s.shutdown();
      if (err) {
        *err = "没有 " + std::to_string(p.width) + "x" + std::to_string(p.height) + "@" +
               std::to_string(p.fps) +
               " 的 BGR/RGB 彩色 profile —— 换分辨率重试（**别用 OB_FORMAT_ANY**：它会挑中 "
               "MJPG 并且不交付彩色帧）";
      }
      return false;
    }
    s.color_prof = picked.prof;
    s.color_needs_swap = picked.needs_swap;
    auto depth = pick_depth(s.pipe->getStreamProfileList(OB_SENSOR_DEPTH), p.width, p.height,
                            p.fps);
    if (!depth) {
      s.shutdown();
      if (err) {
        *err = "没有 " + std::to_string(p.width) + "x" + std::to_string(p.height) + "@" +
               std::to_string(p.fps) + " 的 Y16 深度 profile";
      }
      return false;
    }
    s.cfg->enableStream(s.color_prof);
    s.cfg->enableStream(depth);
    s.pipe->enableFrameSync();

    // 对齐阶梯。这台机器 HW 一定失败（实测），所以阶梯的实际作用是"记录到底用了哪种"。
    struct Try {
      OBAlignMode mode;
      Align tag;
    };
    const Try tries[] = {{ALIGN_D2C_HW_MODE, Align::kHwD2c},
                         {ALIGN_D2C_SW_MODE, Align::kSwD2c},
                         {ALIGN_DISABLE, Align::kNone}};
    std::string ladder;
    bool started = false;
    for (const Try& t : tries) {
      try {
        s.cfg->setAlignMode(t.mode);
        s.pipe->start(s.cfg);
        s.align = t.tag;
        started = true;
        break;
      } catch (const ob::Error& e) {
        ladder += std::string(to_string(t.tag)) + "(" + e.getMessage() + ") ";
      }
    }
    if (!started) {
      s.shutdown();
      if (err) {
        *err = "三种对齐模式都起不来: " + ladder;
      }
      return false;
    }
    if (s.align == Align::kNone && !p.allow_unaligned) {
      s.shutdown();
      if (err) {
        *err = "只能以**不对齐**启动，而下游假设深度与彩色共用一套内参：这会让配准自洽地量到"
               "错位的场景。确认要这样跑再加 allow_unaligned。阶梯: " +
               ladder;
      }
      return false;
    }

    // 等第一个**成对**帧组：彩色的暖机时长固定（约 333 ms），按帧数跳过是错的。
    const int deadline_ms = p.first_pair_timeout_ms;
    bool got_pair = false;
    int unpaired = 0;
    std::shared_ptr<ob::FrameSet> first;
    {
      std::lock_guard<std::mutex> g(s.mtx);
      s.health.unpaired_framesets = 0;
    }
    for (auto t0 = std::chrono::steady_clock::now();
         std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - t0)
             .count() < deadline_ms;) {
      std::shared_ptr<ob::FrameSet> fs;
      try {
        fs = s.pipe->waitForFrameset(200);
      } catch (const ob::Error&) {
        continue;
      }
      if (!fs) {
        continue;
      }
      if (fs->depthFrame() && fs->getFrame(OB_FRAME_COLOR)) {
        first = fs;
        got_pair = true;
        break;
      }
      ++unpaired;
    }
    if (!got_pair) {
      s.shutdown();
      if (err) {
        *err = "启动后 " + std::to_string(deadline_ms) +
               " ms 内没拿到**成对**帧组（丢弃 " + std::to_string(unpaired) +
               " 个只含深度的帧组）—— 彩色没上线的话示教就是一个瞎基准，不能继续";
      }
      return false;
    }
    {
      std::lock_guard<std::mutex> g(s.mtx);
      s.health.unpaired_framesets = unpaired;
    }

    auto df = first->depthFrame();
    s.depth_scale_mm = df->getValueScale();
    if (std::abs(s.depth_scale_mm - 1.0) > 1e-9) {
      s.shutdown();
      if (err) {
        *err = "深度 valueScale=" + std::to_string(s.depth_scale_mm) +
               " ≠ 1，而本模块的契约是 CV_16UC1 毫米（实测该机型为 1）。要么换 profile，要么"
               "把接口扩成 CV_32FC1 —— 悄悄 round 会把单位问题变成毫米级系统误差";
      }
      return false;
    }
    s.depth_w = df->width();
    s.depth_h = df->height();
    // 彩色尺寸用**请求值**：Frame 基类拿不到 width/height，而不做 as<ColorFrame>()
    // 是刻意的（见 wait_frame 里同一条）。dataSize 校验在取帧时兜住。
    s.color_w = s.p.width;
    s.color_h = s.p.height;

    // 标定按**实际启用的分辨率**取。158 组列表里没有分辨率键，只能这样问。
    try {
      const OBCameraParam cp =
          s.pipe->getCameraParamWithProfile(s.color_w, s.color_h, s.depth_w, s.depth_h);
      s.calib.color = to_intrinsics(cp.rgbIntrinsic, s.color_w, s.color_h);
      s.calib.raw_depth = to_intrinsics(cp.depthIntrinsic, s.depth_w, s.depth_h);
      s.calib.color_dist = to_distortion(cp.rgbDistortion);
      s.calib.depth_dist = to_distortion(cp.depthDistortion);
      s.calib.baseline_mm = cp.transform.trans[0];
    } catch (const ob::Error& e) {
      s.shutdown();
      if (err) {
        *err = std::string("按分辨率取标定失败: ") + e.getMessage();
      }
      return false;
    }
    if (!s.calib.valid()) {
      s.shutdown();
      if (err) {
        *err = "设备自报内参非法 fx=" + std::to_string(s.calib.color.fx) +
               " fy=" + std::to_string(s.calib.color.fy) + " —— 下游 unproject 会产出 inf/NaN";
      }
      return false;
    }
    // 对齐后的深度是彩色分辨率 ⇒ 用彩色内参反投影。若设备给的两套焦距差得离谱而分辨率却相同，
    // 说明取到的标定组不对，宁可现在报错。
    if (s.align != Align::kNone && s.depth_w == s.color_w && s.calib.raw_depth.fx > 0.0 &&
        std::abs(s.calib.color.fx / s.calib.raw_depth.fx - 1.0) > 0.25) {
      s.shutdown();
      if (err) {
        *err = "彩色 fx=" + std::to_string(s.calib.color.fx) + " 与深度 fx=" +
               std::to_string(s.calib.raw_depth.fx) + " 差 >25%，但交付分辨率却是彩色尺寸："
               "标定组与 profile 不匹配";
      }
      return false;
    }

    // 读取传感器间外参矩阵（GYRO -> COLOR）
    s.calib.gyro_extrinsics_loaded = false;
    try {
      OBCalibrationParam calib_param = s.pipe->getCalibrationParam(s.cfg);
      const auto& e_gyro_color = calib_param.extrinsics[OB_SENSOR_GYRO][OB_SENSOR_COLOR];
      s.calib.R_cam_gyro << e_gyro_color.rot[0], e_gyro_color.rot[1], e_gyro_color.rot[2],
                            e_gyro_color.rot[3], e_gyro_color.rot[4], e_gyro_color.rot[5],
                            e_gyro_color.rot[6], e_gyro_color.rot[7], e_gyro_color.rot[8];
      s.calib.t_cam_gyro << e_gyro_color.trans[0], e_gyro_color.trans[1], e_gyro_color.trans[2];
      // 全零旋转不是 Identity：isApprox(I) 过不去会被当成已标定，ω 全变成 0。
      if (!is_valid_rotation(s.calib.R_cam_gyro)) {
        s.calib.R_cam_gyro = Eigen::Matrix3d::Identity();
        s.calib.t_cam_gyro = Eigen::Vector3d::Zero();
      } else if (!(s.calib.R_cam_gyro.isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
                   s.calib.t_cam_gyro.norm() < 1e-6)) {
        s.calib.gyro_extrinsics_loaded = true;
      }
    } catch (const ob::Error&) {
      s.calib.R_cam_gyro = Eigen::Matrix3d::Identity();
      s.calib.t_cam_gyro = Eigen::Vector3d::Zero();
    }

    // 开启 IMU 陀螺仪数据流
    if (p.enable_imu) {
      try {
        auto gyro_sensor = s.dev->getSensor(OB_SENSOR_GYRO);
        if (gyro_sensor) {
          auto gyro_profiles = gyro_sensor->getStreamProfileList();
          if (gyro_profiles && gyro_profiles->count() > 0) {
            auto gprof = gyro_profiles->getProfile(0);
            s.gyro_sensor = gyro_sensor;
            s.gyro_sensor->start(gprof, [&s](std::shared_ptr<ob::Frame> frame) {
              if (!frame) return;
              auto gf = frame->as<ob::GyroFrame>();
              if (!gf) return;
              auto val = gf->getValue();
              int64_t ts_ns = static_cast<int64_t>(gf->getTimeStampUs()) * 1000;
              Eigen::Vector3d omega_raw(val.x, val.y, val.z);
              Eigen::Vector3d omega_cam = s.calib.R_cam_gyro * omega_raw;

              std::lock_guard<std::mutex> g(s.gyro_mtx);
              s.gyro_queue.push_back(GyroSample{ts_ns, omega_cam});
              if (s.gyro_queue.size() > 500) {
                s.gyro_queue.pop_front();
              }
            });
            s.calib.has_imu = true;
            auto gp = gprof->as<ob::GyroStreamProfile>();
            s.calib.gyro_sample_rate_hz = gp ? static_cast<int>(gp->getSampleRate()) : 200;
          }
        }
      } catch (const ob::Error&) {
        s.calib.has_imu = false;
      }
    }

    s.opened = true;
    return true;
  } catch (const ob::Error& e) {
    const std::string msg = std::string("Orbbec 错误: ") + e.getMessage();
    s.shutdown();
    if (err) {
      *err = msg;
    }
    return false;
  } catch (const std::exception& e) {
    const std::string msg = std::string("取流启动异常: ") + e.what();
    s.shutdown();
    if (err) {
      *err = msg;
    }
    return false;
  }
}

void OrbbecCapture::close() {
  if (im_->opened || im_->pipe) {
    im_->shutdown();
  }
}

bool OrbbecCapture::wait_frame(RgbdFrame* out, std::string* err) {
  Impl& s = *im_;
  if (!s.opened || !s.pipe) {
    if (err) {
      *err = "未打开设备";
    }
    return false;
  }
  if (!s.present.load()) {
    if (err) {
      *err = "设备已拔出（回调报告）";
    }
    std::lock_guard<std::mutex> g(s.mtx);
    s.health.device_present = false;
    s.health.last_error = *err;
    return false;
  }

  std::shared_ptr<ob::FrameSet> fs;
  try {
    fs = s.pipe->waitForFrameset(s.p.frame_timeout_ms);
  } catch (const ob::Error& e) {
    if (err) {
      *err = std::string("waitForFrameset: ") + e.getMessage();
    }
    return false;
  }
  if (!fs) {
    std::lock_guard<std::mutex> g(s.mtx);
    ++s.health.dropouts;
    s.health.last_error = "等待帧超时 " + std::to_string(s.p.frame_timeout_ms) + " ms";
    if (err) {
      *err = s.health.last_error;
    }
    return false;
  }

  auto df = fs->depthFrame();
  auto cfr = fs->getFrame(OB_FRAME_COLOR);
  if (!df || !cfr) {
    // 暖机之后这种帧组代表真丢帧：不拿它当一帧，也不静默用上一帧顶替（那会让"位姿不动"伪装成
    // "工件不动"）。
    std::lock_guard<std::mutex> g(s.mtx);
    ++s.health.unpaired_framesets;
    if (err) {
      *err = "帧组缺彩色或缺深度（set=" + std::to_string(fs->getCount()) + "）";
    }
    return false;
  }
  // 不做 as<ob::ColorFrame>()：Frame::is<>() 按 getType() 判，type 一旦不是 COLOR 就抛，
  // 而这里要的 getData/getDataSize/getTimeStampUs 全在基类上。
  const auto& cf = cfr;
  if (cf->getDataSize() == 0 || df->getDataSize() == 0) {
    if (err) {
      *err = "帧数据为空";
    }
    return false;
  }

  const int dw = df->width(), dh = df->height();
  const int cw = s.color_w, ch = s.color_h;
  if (cf->getDataSize() < static_cast<uint32_t>(cw) * static_cast<uint32_t>(ch) * 3u ||
      df->getDataSize() < static_cast<uint32_t>(dw) * static_cast<uint32_t>(dh) * 2u) {
    if (err) {
      *err = "帧尺寸与声明不符: color " + std::to_string(cf->getDataSize()) + " B (期望 " +
             std::to_string(cw * ch * 3) + "), depth " + std::to_string(df->getDataSize());
    }
    return false;
  }
  if (dw != cw || dh != ch) {
    if (err) {
      *err = "对齐没生效：深度 " + std::to_string(dw) + "x" + std::to_string(dh) + " ≠ 彩色 " +
             std::to_string(cw) + "x" + std::to_string(ch);
    }
    return false;
  }

  // 帧组内存会被 SDK 回收，必须 clone 才能带出这一帧的生命周期。
  cv::Mat color(ch, cw, CV_8UC3, cf->getData());
  cv::Mat depth(dh, dw, CV_16UC1, df->getData());
  out->color = color.clone();
  out->depth_mm = depth.clone();
  if (s.color_needs_swap) {
    cv::cvtColor(out->color, out->color, cv::COLOR_RGB2BGR);
  }
  out->ts_ns = static_cast<int64_t>(df->getTimeStampUs()) * 1000;
  out->color_ts_ns = static_cast<int64_t>(cf->getTimeStampUs()) * 1000;
  out->align = s.align;

  const double d_ms = static_cast<double>(out->ts_ns) * 1e-6;
  const double c_ms = static_cast<double>(out->color_ts_ns) * 1e-6;
  std::lock_guard<std::mutex> g(s.mtx);
  ++s.health.frames;
  s.health.d2c_offset_ms += (c_ms - d_ms - s.health.d2c_offset_ms) * 0.05;
  if (s.prev_dev_ts_ms > 0.0) {
    const double gap = d_ms - s.prev_dev_ts_ms;
    s.health.period_ms += (gap - s.health.period_ms) * 0.1;
    s.health.max_period_ms = std::max(s.health.max_period_ms, gap);
    if (gap > kDropoutFactor * s.nominal_period_ms) {
      ++s.health.dropouts;
    }
  } else {
    s.health.period_ms = s.nominal_period_ms;
  }
  s.prev_dev_ts_ms = d_ms;
  s.health.device_present = s.present.load();
  s.health.lock_held = s.lock.held();
  return true;
}

bool OrbbecCapture::drain_gyro_samples(std::vector<GyroSample>* out) {
  if (!out) return false;
  out->clear();
  std::lock_guard<std::mutex> g(im_->gyro_mtx);
  if (im_->gyro_queue.empty()) {
    return false;
  }
  out->assign(im_->gyro_queue.begin(), im_->gyro_queue.end());
  im_->gyro_queue.clear();
  return true;
}

const DeviceCalib& OrbbecCapture::calib() const { return im_->calib; }

Align OrbbecCapture::align() const { return im_->align; }

bool OrbbecCapture::is_open() const { return im_->opened; }

CaptureHealth OrbbecCapture::health() const {
  std::lock_guard<std::mutex> g(im_->mtx);
  CaptureHealth h = im_->health;
  h.device_present = im_->present.load();
  h.lock_held = im_->lock.held();
  return h;
}

}  // namespace follow
