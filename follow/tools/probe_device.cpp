// P4 第一步：把"没有相机就只能假设"的那些事变成量出来的数。
//
// 这台板子上以前只有 MPP 编解码节点，所以下面每一条都是**猜的**；现在相机插上了，逐条问设备：
//   1) 有哪些 color/depth profile，BGR 到底在不在（CPU 前端要 BGR，RGB 得自己换）；
//   2) 1280x800 是否真的只有 SW D2C（query_hw_d2c.cpp 在 FW 1.4.60 上留了这句话，FW 可能已经
//      变了），以及 SW D2C 到底吃多少 CPU —— 这个数决定 follow 用哪个分辨率；
//   3) 设备自报内参与 data/ 里那份标定值差多少（follow 用哪一份，是正确性问题不是风格问题）；
//   4) 深度单位与 valueScale（follow 内部一律米，这里是唯一跨界的地方，必须由设备说清楚）；
//   5) 实测帧间隔与 getTimeStampUs 的域（设备时还是主机时，单调吗，跳变吗）。
//
// 它是**诊断工具**，不是产品代码：不链 libfollow，只做设备问答。产品取流在 orbbec_capture.cpp。
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <sys/resource.h>

#include <libobsensor/ObSensor.hpp>

namespace {

struct Cfg {
  int width = 848;
  int height = 480;
  int fps = 30;
  int frames = 60;
  bool force_sw = false;   // 跳过 HW，直接问 SW
  bool no_align = false;   // 完全不对齐（原始双流）
  bool no_sync = false;    // 不开 enableFrameSync：排障用，同步本身可能是彩色帧消失的原因
  std::string dump_dir;    // 非空 = 落一对原始 RGB-D + 按分辨率取到的标定参数，给离线判畸变用
};

double cpu_seconds() {
  rusage ru{};
  ::getrusage(RUSAGE_SELF, &ru);
  const auto sec = [](const timeval& t) { return static_cast<double>(t.tv_sec) + t.tv_usec * 1e-6; };
  return sec(ru.ru_utime) + sec(ru.ru_stime);
}

void print_profiles(ob::Pipeline& pipe, OBSensorType sensor, const char* label) {
  std::shared_ptr<ob::StreamProfileList> list;
  try {
    list = pipe.getStreamProfileList(sensor);
  } catch (const ob::Error& e) {
    std::printf("  %-5s profile 列表读取失败: %s\n", label, e.getMessage());
    return;
  }
  if (!list || list->count() == 0) {
    std::printf("  %-5s 没有任何 profile\n", label);
    return;
  }

  if (sensor == OB_SENSOR_GYRO) {
    std::printf("  %s (%zu 条 profile):\n", label, static_cast<size_t>(list->count()));
    for (uint32_t i = 0; i < list->count(); ++i) {
      auto p = list->getProfile(i)->as<ob::GyroStreamProfile>();
      std::printf("      Gyro profile [%u]: sampleRate=%d fullScale=%d format=%d\n",
                  i, static_cast<int>(p->getSampleRate()), static_cast<int>(p->getFullScaleRange()),
                  static_cast<int>(p->format()));
    }
    return;
  }
  if (sensor == OB_SENSOR_ACCEL) {
    std::printf("  %s (%zu 条 profile):\n", label, static_cast<size_t>(list->count()));
    for (uint32_t i = 0; i < list->count(); ++i) {
      auto p = list->getProfile(i)->as<ob::AccelStreamProfile>();
      std::printf("      Accel profile [%u]: sampleRate=%d fullScale=%d format=%d\n",
                  i, static_cast<int>(p->getSampleRate()), static_cast<int>(p->getFullScaleRange()),
                  static_cast<int>(p->format()));
    }
    return;
  }

  // (w,h,fps) -> 该组合下出现过的格式集合。表格按分辨率聚合，不然一次列出 60 行没法看。
  std::map<std::string, std::vector<std::string>> rows;
  for (uint32_t i = 0; i < list->count(); ++i) {
    auto p = list->getProfile(i)->as<ob::VideoStreamProfile>();
    char key[48];
    std::snprintf(key, sizeof(key), "%4dx%-4d @%3dfps", p->width(), p->height(), p->fps());
    const int fmt = static_cast<int>(p->format());
    rows[key].push_back(fmt == OB_FORMAT_BGR      ? "BGR"
                       : fmt == OB_FORMAT_RGB     ? "RGB"
                       : fmt == OB_FORMAT_MJPG    ? "MJPG"
                       : fmt == OB_FORMAT_YUYV    ? "YUYV"
                       : fmt == OB_FORMAT_Y16     ? "Y16"
                       : fmt == OB_FORMAT_Y8      ? "Y8"
                                                  : "fmt" + std::to_string(fmt));
  }
  std::printf("  %s (%zu 条 profile，按分辨率聚合):\n", label, static_cast<size_t>(list->count()));
  for (const auto& kv : rows) {
    std::string uniq;
    for (const auto& f : kv.second) {
      if (uniq.find(f) == std::string::npos) {
        uniq += (uniq.empty() ? "" : " ") + f;
      }
    }
    std::printf("      %s  %s\n", kv.first.c_str(), uniq.c_str());
  }
}

void print_all_d2c_offerings(ob::Pipeline& pipe) {
  std::shared_ptr<ob::StreamProfileList> list;
  try {
    list = pipe.getStreamProfileList(OB_SENSOR_COLOR);
  } catch (const ob::Error& e) {
    std::printf("  读取 color profile 列表失败: %s\n", e.getMessage());
    return;
  }
  if (!list || list->count() == 0) return;

  std::printf("\n  ==== D2C 对齐支持矩阵扫描 ====\n");
  std::map<std::pair<int, int>, std::vector<int>> res_fps_map;
  for (uint32_t i = 0; i < list->count(); ++i) {
    auto p = list->getProfile(i)->as<ob::VideoStreamProfile>();
    auto key = std::make_pair(p->width(), p->height());
    auto& fps_list = res_fps_map[key];
    if (std::find(fps_list.begin(), fps_list.end(), p->fps()) == fps_list.end()) {
      fps_list.push_back(p->fps());
    }
  }

  for (const auto& kv : res_fps_map) {
    const int w = kv.first.first;
    const int h = kv.first.second;
    for (int fps : kv.second) {
      std::shared_ptr<ob::VideoStreamProfile> color = nullptr;
      try {
        color = list->getVideoStreamProfile(w, h, OB_FORMAT_ANY, fps);
      } catch (...) {
        continue;
      }
      if (!color) continue;

      int hw_count = 0, sw_count = 0;
      std::string hw_info, sw_info;
      try {
        auto dp_hw = pipe.getD2CDepthProfileList(color, ALIGN_D2C_HW_MODE);
        if (dp_hw && dp_hw->count() > 0) {
          hw_count = dp_hw->count();
          auto p0 = dp_hw->getProfile(0)->as<ob::VideoStreamProfile>();
          hw_info = std::to_string(p0->width()) + "x" + std::to_string(p0->height());
        }
      } catch (...) {}

      try {
        auto dp_sw = pipe.getD2CDepthProfileList(color, ALIGN_D2C_SW_MODE);
        if (dp_sw && dp_sw->count() > 0) {
          sw_count = dp_sw->count();
          auto p0 = dp_sw->getProfile(0)->as<ob::VideoStreamProfile>();
          sw_info = std::to_string(p0->width()) + "x" + std::to_string(p0->height());
        }
      } catch (...) {}

      if (hw_count > 0 || sw_count > 0) {
        std::printf("  %4dx%-4d @%2dfps | HW D2C: %s (depth=%s, %d profiles) | SW D2C: %s (depth=%s, %d profiles)\n",
                    w, h, fps,
                    hw_count > 0 ? "支持 [YES]" : "不支持",
                    hw_count > 0 ? hw_info.c_str() : "none", hw_count,
                    sw_count > 0 ? "支持 [YES]" : "不支持",
                    sw_count > 0 ? sw_info.c_str() : "none", sw_count);
      } else {
        std::printf("  %4dx%-4d @%2dfps | HW D2C: 不支持 | SW D2C: 不支持\n", w, h, fps);
      }
    }
  }
}

void print_device_params(ob::Device& dev) {
  try {
    auto list = dev.getCalibrationCameraParamList();
    const uint32_t n = list->getCount();
    std::printf("  设备标定参数组: %u 组\n", n);
    for (uint32_t i = 0; i < n; ++i) {
      const OBCameraParam p = list->getCameraParam(i);
      std::printf(
          "    [%u] rgb fx=%.3f fy=%.3f cx=%.3f cy=%.3f | depth fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n",
          i, p.rgbIntrinsic.fx, p.rgbIntrinsic.fy, p.rgbIntrinsic.cx, p.rgbIntrinsic.cy,
          p.depthIntrinsic.fx, p.depthIntrinsic.fy, p.depthIntrinsic.cx, p.depthIntrinsic.cy);
      std::printf("          rgb distort model=%d k1=%.5f k2=%.5f k3=%.5f k4=%.5f k5=%.5f k6=%.5f "
                  "p1=%.5f p2=%.5f | mirrored=%d\n",
                  static_cast<int>(p.rgbDistortion.model), p.rgbDistortion.k1, p.rgbDistortion.k2,
                  p.rgbDistortion.k3, p.rgbDistortion.k4, p.rgbDistortion.k5, p.rgbDistortion.k6,
                  p.rgbDistortion.p1, p.rgbDistortion.p2, p.isMirrored ? 1 : 0);
      std::printf("          d2c transform t=(%.2f, %.2f, %.2f) mm\n", p.transform.trans[0],
                  p.transform.trans[1], p.transform.trans[2]);
    }
  } catch (const ob::Error& e) {
    std::printf("  标定参数读取失败: %s\n", e.getMessage());
  }
}

struct Stats {
  int frames = 0;
  int no_depth = 0;
  int with_color = 0;
  // 帧组构成 + 两路各自的设备时间戳：enableFrameSync 到底有没有配对，只有分开的速率能证。
  int set_count[4] = {0, 0, 0, 0};
  uint32_t color_bytes = 0;
  uint64_t d_ts0 = 0, d_ts1 = 0, c_ts0 = 0, c_ts1 = 0;
  int color_backwards = 0;
  double min_mm = 1e18, max_mm = -1e18, sum_valid = 0.0;
  long long pixels = 0, valid = 0;
  double max_gap_ms = 0.0, sum_gap_ms = 0.0;
  double prev_t_ms = -1.0;
  uint64_t last_ts_us = 0;
  int ts_backwards = 0, ts_jumps = 0;
  bool dumped = false;
};

const char* align_name(OBAlignMode m) {
  return m == ALIGN_D2C_HW_MODE ? "HW_D2C" : (m == ALIGN_D2C_SW_MODE ? "SW_D2C" : "DISABLE");
}

bool write_raw(const std::string& path, const void* data, size_t n) {
  std::ofstream f(path, std::ios::binary);
  if (!f || !data || !n) {
    return false;
  }
  f.write(static_cast<const char*>(data), static_cast<std::streamsize>(n));
  return static_cast<bool>(f);
}

// 落一对帧 + **按分辨率取到**的标定参数（getCameraParamWithProfile，不是 158 组里猜一行）。
// 离线用这对数据判断 SW D2C 交付的图像是否已去畸变：把对齐后的深度按 pinhole 投回彩色系，
// 若边缘随半径系统性偏移 → 彩色仍是畸变的，follow 必须自己 undistort 特征点。
void dump_pair(ob::Pipeline& pipe, const Cfg& c, const char* align,
               const std::shared_ptr<ob::Frame>& cf, const std::shared_ptr<ob::DepthFrame>& df) {
  std::error_code ec;
  std::filesystem::create_directories(c.dump_dir, ec);
  if (ec) {
    std::printf("  建目录 %s 失败: %s\n", c.dump_dir.c_str(), ec.message().c_str());
    return;
  }
  const int dw = df->width(), dh = df->height();
  char name[160];
  std::snprintf(name, sizeof(name), "depth_%dx%d.y16", dw, dh);
  const std::string dp = c.dump_dir + "/" + name;
  if (!write_raw(dp, df->getData(), df->getDataSize())) {
    std::printf("  写深度失败: %s\n", dp.c_str());
    return;
  }
  if (!cf) {
    std::printf("  这一帧组没有彩色，只落了深度\n");
    return;
  }
  const int cw = c.width, ch = c.height;
  std::snprintf(name, sizeof(name), "color_%dx%d.bgr", cw, ch);
  const std::string cp = c.dump_dir + "/" + name;
  if (!write_raw(cp, cf->getData(), cf->getDataSize())) {
    std::printf("  写彩色失败: %s\n", cp.c_str());
    return;
  }
  OBCameraParam p{};
  bool have = true;
  try {
    p = pipe.getCameraParamWithProfile(cw, ch, dw, dh);
  } catch (const ob::Error& e) {
    std::printf("  getCameraParamWithProfile(%dx%d,%dx%d) 失败: %s\n", cw, ch, dw, dh,
                e.getMessage());
    have = false;
  }
  const std::string tp = c.dump_dir + "/param.txt";
  std::ofstream t(tp);
  t << "align=" << align << "\n";
  t << "color " << cw << " " << ch << " " << cf->getDataSize() << " fmt=" << (int)cf->format()
    << " ts_us=" << cf->getTimeStampUs() << "\n";
  t << "depth " << dw << " " << dh << " " << df->getDataSize() << " fmt=" << (int)df->format()
    << " valueScale=" << df->getValueScale() << " ts_us=" << df->getTimeStampUs() << "\n";
  if (have) {
    t << "rgb_intrinsic " << p.rgbIntrinsic.fx << " " << p.rgbIntrinsic.fy << " "
      << p.rgbIntrinsic.cx << " " << p.rgbIntrinsic.cy << "\n";
    t << "depth_intrinsic " << p.depthIntrinsic.fx << " " << p.depthIntrinsic.fy << " "
      << p.depthIntrinsic.cx << " " << p.depthIntrinsic.cy << "\n";
    t << "rgb_distortion " << (int)p.rgbDistortion.model << " " << p.rgbDistortion.k1 << " "
      << p.rgbDistortion.k2 << " " << p.rgbDistortion.k3 << " " << p.rgbDistortion.k4 << " "
      << p.rgbDistortion.k5 << " " << p.rgbDistortion.k6 << " " << p.rgbDistortion.p1 << " "
      << p.rgbDistortion.p2 << "\n";
    t << "depth_distortion " << (int)p.depthDistortion.model << " " << p.depthDistortion.k1 << " "
      << p.depthDistortion.k2 << " " << p.depthDistortion.k3 << " " << p.depthDistortion.k4 << " "
      << p.depthDistortion.k5 << " " << p.depthDistortion.k6 << " " << p.depthDistortion.p1 << " "
      << p.depthDistortion.p2 << "\n";
    t << "transform_rot";
    for (float r : p.transform.rot) t << " " << r;
    t << "\ntransform_trans_mm";
    for (float v : p.transform.trans) t << " " << v;
    t << "\nis_mirrored " << (p.isMirrored ? 1 : 0) << "\n";
  }
  std::printf("  已落: %s + depth_*.y16 + %s\n", cp.c_str(), tp.c_str());
}

void run_capture(ob::Pipeline& pipe, const Cfg& c, Stats* st) {
  auto cfg = std::make_shared<ob::Config>();
  auto color_list = pipe.getStreamProfileList(OB_SENSOR_COLOR);
  auto depth_list = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
  // 格式梯子不是风格：OB_FORMAT_ANY 在这台机器上挑中 MJPG，pipeline 照常 start 但**一帧彩色
  // 都不交付** —— 量出来的"取流正常"其实是深度单飞。驱动里 BGR→RGB→ANY 的顺序就是踩过这个坑。
  std::shared_ptr<ob::VideoStreamProfile> color = nullptr;
  for (const OBFormat fmt : {OB_FORMAT_BGR, OB_FORMAT_RGB, OB_FORMAT_ANY}) {
    try {
      color = color_list->getVideoStreamProfile(c.width, c.height, fmt, c.fps);
    } catch (const ob::Error&) {
      continue;
    }
    if (color) break;
  }
  std::shared_ptr<ob::VideoStreamProfile> depth = nullptr;
  for (const OBFormat fmt : {OB_FORMAT_Y16, OB_FORMAT_ANY}) {
    try {
      depth = depth_list->getVideoStreamProfile(c.width, c.height, fmt, c.fps);
    } catch (const ob::Error&) {
      continue;
    }
    if (depth) break;
  }
  if (!depth) {
    depth = depth_list->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, c.fps);
  }
  if (!color || !depth) {
    std::printf("  拿不到 %dx%d@%d 的 %s profile —— 换一个分辨率再问\n", c.width, c.height, c.fps,
                color ? "depth" : (depth ? "color" : "color/depth"));
    return;
  }
  std::printf("  选中 color: %ux%u fmt=%d @%ufps | depth: %ux%u fmt=%d @%ufps\n", color->width(),
              color->height(), static_cast<int>(color->format()), color->fps(), depth->width(),
              depth->height(), static_cast<int>(depth->format()), depth->fps());
  cfg->enableStream(color);
  cfg->enableStream(depth);
  if (!c.no_sync) {
    pipe.enableFrameSync();
  }

  OBAlignMode mode = ALIGN_DISABLE;
  const int attempts = c.no_align ? 1 : (c.force_sw ? 2 : 3);
  bool started = false;
  for (int a = 0; a < attempts && !started; ++a) {
    const OBAlignMode try_mode =
        a == 0 ? ALIGN_D2C_HW_MODE : (a == 1 ? ALIGN_D2C_SW_MODE : ALIGN_DISABLE);
    if (c.no_align && a == 0) {
      mode = ALIGN_DISABLE;
    } else if (c.force_sw && a == 0) {
      mode = ALIGN_D2C_SW_MODE;
    } else {
      mode = try_mode;
    }
    try {
      cfg->setAlignMode(mode);
      const double cpu0 = cpu_seconds();
      const auto t0 = std::chrono::steady_clock::now();
      pipe.start(cfg);
      const auto t1 = std::chrono::steady_clock::now();
      std::printf("  启动 pipeline: %s（耗时 %.0f ms，CPU %.0f ms）\n",
                  mode == ALIGN_D2C_HW_MODE
                      ? "HW D2C"
                      : (mode == ALIGN_D2C_SW_MODE ? "SW D2C" : "不对齐"),
                  std::chrono::duration<double, std::milli>(t1 - t0).count(),
                  (cpu_seconds() - cpu0) * 1000.0);
      started = true;
    } catch (const ob::Error& e) {
      std::printf("  %s 启动失败: %s\n",
                  mode == ALIGN_D2C_HW_MODE ? "HW D2C" : "SW D2C", e.getMessage());
    }
  }
  if (!started) {
    std::printf("  三种对齐模式都起不来 —— 这台机器上取流不可用，P4 到此为止\n");
    return;
  }

  const double cpu_start = cpu_seconds();
  const auto wall_start = std::chrono::steady_clock::now();
  double first_gap = 0.0;
  bool have_first = false;
  while (st->frames < c.frames) {
    std::shared_ptr<ob::FrameSet> fs;
    try {
      fs = pipe.waitForFrameset(1000);
    } catch (const ob::Error& e) {
      std::printf("  waitForFrameset 失败: %s\n", e.getMessage());
      break;
    }
    if (!fs) {
      std::printf("  超时：1000 ms 没等到帧\n");
      break;
    }
    const auto now = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(now - wall_start).count();
    if (!have_first) {
      first_gap = ms;
      have_first = true;
      // 彩色帧按 OB_FRAME_COLOR 取不到时，唯一能分清"设备没发"和"发了但类型不同"的证据就是这
      // 个列表。SDK 2.x 开 frameSync 后合并帧组里的彩色可能标成 OB_FRAME_VIDEO(0)。
      std::printf("  首个帧组含 %u 帧:", fs->getCount());
      for (uint32_t i = 0; i < fs->getCount(); ++i) {
        auto f = fs->getFrameByIndex(i);
        if (!f) {
          std::printf(" [%u]=null", i);
          continue;
        }
        std::printf(" [%u]type=%d fmt=%d size=%u", i, static_cast<int>(f->getType()),
                    static_cast<int>(f->format()), f->getDataSize());
      }
      std::printf("\n");
    }
    auto df = fs->depthFrame();
    // 用基类指针：colorFrame() 内部要 as<ColorFrame>()，而换名字后的那一帧按 ColorFrame 取不到。
    std::shared_ptr<ob::Frame> cf = fs->getFrame(OB_FRAME_COLOR);
    if (!cf) {
      cf = fs->getFrame(OB_FRAME_VIDEO);   // 见上：同步帧组里彩色换了名字
    }
    if (!df) {
      ++st->no_depth;
      continue;
    }
    ++st->frames;
    ++st->set_count[std::min<uint32_t>(fs->getCount(), 3u)];
    const uint64_t dts = df->getTimeStampUs();
    if (!st->d_ts0) st->d_ts0 = dts;
    st->d_ts1 = dts;
    uint64_t cts = 0;
    if (cf) {
      ++st->with_color;
      cts = cf->getTimeStampUs();
      if (!st->color_bytes) {
        st->color_bytes = cf->getDataSize();
        std::printf("  首个彩色帧: type=%d fmt=%d size=%u B（%dx%d BGR 应为 %d）\n",
                    static_cast<int>(cf->getType()), static_cast<int>(cf->format()),
                    cf->getDataSize(), c.width, c.height, c.width * c.height * 3);
      }
      if (st->c_ts1 && cts < st->c_ts1) ++st->color_backwards;
      if (!st->c_ts0) st->c_ts0 = cts;
      st->c_ts1 = cts;
    }
    if (st->frames <= 17) {
      char cb[64];
      if (cts) {
        std::snprintf(cb, sizeof(cb), "c=%8.1f Δ=%+7.3f",
                      (static_cast<double>(cts) - static_cast<double>(st->d_ts0)) / 1000.0,
                      (static_cast<double>(cts) - static_cast<double>(dts)) / 1000.0);
      } else {
        std::snprintf(cb, sizeof(cb), "c=    无      --");
      }
      std::printf("    #%2d d=%8.1f ms  %s  set=%u\n", st->frames,
                  (static_cast<double>(dts) - static_cast<double>(st->d_ts0)) / 1000.0, cb,
                  fs->getCount());
    }
    // 暖机（彩色比深度晚上线约 333 ms）之后再落，落下来的才是一对同一时刻的帧。
    if (!c.dump_dir.empty() && cf && st->frames >= 15 && !st->dumped) {
      st->dumped = true;
      dump_pair(pipe, c, align_name(mode), cf, df);
    }
    const double t_frame = ms - first_gap;
    if (st->prev_t_ms >= 0.0) {
      const double gap = t_frame - st->prev_t_ms;
      st->sum_gap_ms += gap;
      if (gap > st->max_gap_ms) {
        st->max_gap_ms = gap;
      }
    }
    st->prev_t_ms = t_frame;

    const uint64_t ts = df->getTimeStampUs();
    if (st->last_ts_us && ts < st->last_ts_us) {
      ++st->ts_backwards;
    }
    if (st->last_ts_us && ts > st->last_ts_us + 200'000) {
      ++st->ts_jumps;  // >200 ms 的空洞：掉帧或被重钉
    }
    st->last_ts_us = ts;

    const int w = df->width(), h = df->height();
    const float scale = df->getValueScale();
    const uint16_t* px = reinterpret_cast<const uint16_t*>(df->getData());
    if (!px || df->getDataSize() < static_cast<uint32_t>(w) * static_cast<uint32_t>(h) * 2u) {
      std::printf("  第 %d 帧深度数据不完整（size=%u 期望=%d）\n", st->frames,
                  df->getDataSize(), w * h * 2);
      continue;
    }
    if (st->frames == 1) {
      std::printf("  depth 帧: %dx%d format=%d valueScale=%g data=%u B | color type=%d fmt=%d size=%u B\n",
                  w, h, static_cast<int>(df->format()), scale, df->getDataSize(),
                  cf ? static_cast<int>(cf->getType()) : -1, cf ? static_cast<int>(cf->format()) : -1,
                  cf ? cf->getDataSize() : 0u);
    }
    long long v = 0;
    double lo = 1e18, hi = -1e18, sum = 0.0;
    for (int i = 0; i < w * h; ++i) {
      if (px[i] == 0) {
        continue;
      }
      const double mm = px[i] * scale;
      ++v;
      sum += mm;
      if (mm < lo) lo = mm;
      if (mm > hi) hi = mm;
    }
    st->pixels += static_cast<long long>(w) * h;
    st->valid += v;
    if (v) {
      st->min_mm = std::min(st->min_mm, lo);
      st->max_mm = std::max(st->max_mm, hi);
      st->sum_valid += sum / static_cast<double>(v);
    }
  }
  const double cpu = cpu_seconds() - cpu_start;
  const double wall =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_start)
          .count() -
      first_gap;
  std::printf("\n  ---- 实测（%s，目标 %dx%d@%d，取 %d 帧）----\n",
              mode == ALIGN_D2C_HW_MODE ? "HW D2C" : (mode == ALIGN_D2C_SW_MODE ? "SW D2C" : "无对齐"),
              c.width, c.height, c.fps, st->frames);
  if (st->frames > 1 && wall > 0) {
    std::printf("  帧率 %.2f fps（平均周期 %.1f ms，最大间隔 %.1f ms）\n",
                1000.0 * (st->frames - 1) / wall, wall / (st->frames - 1), st->max_gap_ms);
  }
  std::printf("  取流期间 CPU 时间 %.2f s ⇒ 每帧 %.1f ms（%.0f%% 单核）\n", cpu,
              st->frames ? cpu * 1000.0 / st->frames : 0.0,
              wall > 0 ? 100.0 * cpu * 1000.0 / wall : 0.0);
  if (st->pixels) {
    std::printf("  深度有效 %.1f%%  全局最近 %.0f mm 最远 %.0f mm  平均 %.0f mm\n",
                100.0 * static_cast<double>(st->valid) / static_cast<double>(st->pixels),
                st->min_mm, st->max_mm,
                st->frames ? st->sum_valid / st->frames : 0.0);
  }
  std::printf("  时间戳(us) 单调性: 倒退 %d 次, >200ms 空洞 %d 次, 末值 %llu\n", st->ts_backwards,
              st->ts_jumps, static_cast<unsigned long long>(st->last_ts_us));
  if (st->no_depth) {
    std::printf("  %d 个同步帧组里没有 depth\n", st->no_depth);
  }
  std::printf("  帧组大小: 0帧=%d 1帧=%d 2帧=%d >=3帧=%d\n", st->set_count[0], st->set_count[1],
              st->set_count[2], st->set_count[3]);
  const auto dev_rate = [](uint64_t t0, uint64_t t1, int n) {
    return (n > 1 && t1 > t0) ? 1e6 * static_cast<double>(n - 1) / static_cast<double>(t1 - t0) : 0.0;
  };
  std::printf("  设备时钟速率: depth %.2f fps (%d 帧) | color %.2f fps (%d 帧, 倒退 %d 次)\n",
              dev_rate(st->d_ts0, st->d_ts1, st->frames), st->frames,
              dev_rate(st->c_ts0, st->c_ts1, st->with_color), st->with_color, st->color_backwards);
  std::printf("  彩色配对 %d/%d —— 配不上对时特征前端会拿到空图，不能算取流成功\n", st->with_color,
              st->frames);
  pipe.stop();
}

int probe_main(int argc, char** argv) {
  Cfg c;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--w" && i + 1 < argc) {
      c.width = std::atoi(argv[++i]);
    } else if (a == "--h" && i + 1 < argc) {
      c.height = std::atoi(argv[++i]);
    } else if (a == "--fps" && i + 1 < argc) {
      c.fps = std::atoi(argv[++i]);
    } else if (a == "--n" && i + 1 < argc) {
      c.frames = std::atoi(argv[++i]);
    } else if (a == "--sw") {
      c.force_sw = true;
    } else if (a == "--no-align") {
      c.no_align = true;
    } else if (a == "--no-sync") {
      c.no_sync = true;
    } else if (a == "--dump" && i + 1 < argc) {
      c.dump_dir = argv[++i];
    } else {
      std::printf("用法: follow_probe_device [--w 848] [--h 480] [--fps 30] [--n 60] [--sw] "
                  "[--no-align] [--no-sync] [--dump 目录]\n");
      return 2;
    }
  }

  try {
    ob::Context ctx;
    auto dl = ctx.queryDeviceList();
    if (!dl || dl->deviceCount() == 0) {
      std::printf("没有 Orbbec 设备（UAC 节点在不代表 SDK 能开，见 follow/.gitignore 旁的说明）\n");
      return 1;
    }
    auto dev = dl->getDevice(0);
    auto info = dev->getDeviceInfo();
    // name()/serialNumber()/firmwareVersion()/connectionType() 都是 const char*（SDK 2.9.3）
    std::printf("设备: %s  SN=%s  FW=%s  连接=%s\n", info->name(), info->serialNumber(),
                info->firmwareVersion(), info->connectionType());
    print_device_params(*dev);

    ob::Pipeline pipe(dev);
    print_profiles(pipe, OB_SENSOR_COLOR, "color");
    print_profiles(pipe, OB_SENSOR_DEPTH, "depth");
    print_profiles(pipe, OB_SENSOR_GYRO, "gyro");
    print_profiles(pipe, OB_SENSOR_ACCEL, "accel");

    try {
      auto cfg = std::make_shared<ob::Config>();
      auto color_list = pipe.getStreamProfileList(OB_SENSOR_COLOR);
      auto color = color_list->getVideoStreamProfile(c.width, c.height, OB_FORMAT_ANY, c.fps);
      if (color) cfg->enableStream(color);
      auto depth_list = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
      auto depth = depth_list->getVideoStreamProfile(c.width, c.height, OB_FORMAT_ANY, c.fps);
      if (depth) cfg->enableStream(depth);
      
      OBCalibrationParam calib = pipe.getCalibrationParam(cfg);
      std::printf("\n  ==== 传感器间外参矩阵 (Extrinsics) ====\n");
      const auto& e_gyro_color = calib.extrinsics[OB_SENSOR_GYRO][OB_SENSOR_COLOR];
      std::printf("  [GYRO -> COLOR 外参]:\n");
      std::printf("    Rot = [%.4f, %.4f, %.4f;\n           %.4f, %.4f, %.4f;\n           %.4f, %.4f, %.4f]\n",
                  e_gyro_color.rot[0], e_gyro_color.rot[1], e_gyro_color.rot[2],
                  e_gyro_color.rot[3], e_gyro_color.rot[4], e_gyro_color.rot[5],
                  e_gyro_color.rot[6], e_gyro_color.rot[7], e_gyro_color.rot[8]);
      std::printf("    Trans = (%.3f, %.3f, %.3f) mm\n",
                  e_gyro_color.trans[0], e_gyro_color.trans[1], e_gyro_color.trans[2]);

      const auto& e_color_gyro = calib.extrinsics[OB_SENSOR_COLOR][OB_SENSOR_GYRO];
      std::printf("  [COLOR -> GYRO 外参]:\n");
      std::printf("    Rot = [%.4f, %.4f, %.4f;\n           %.4f, %.4f, %.4f;\n           %.4f, %.4f, %.4f]\n",
                  e_color_gyro.rot[0], e_color_gyro.rot[1], e_color_gyro.rot[2],
                  e_color_gyro.rot[3], e_color_gyro.rot[4], e_color_gyro.rot[5],
                  e_color_gyro.rot[6], e_color_gyro.rot[7], e_color_gyro.rot[8]);
      std::printf("    Trans = (%.3f, %.3f, %.3f) mm\n",
                  e_color_gyro.trans[0], e_color_gyro.trans[1], e_color_gyro.trans[2]);
    } catch (const ob::Error& e) {
      std::printf("  读取 OBCalibrationParam 失败: %s\n", e.getMessage());
    }

    print_all_d2c_offerings(pipe);

    Stats st{};
    run_capture(pipe, c, &st);
    // 只有深度 = 特征前端拿不到任何东西，跟着冻结地图配准还能"看着很准"，不能算过。
    return (st.frames > 0 && st.with_color > 0) ? 0 : 1;
  } catch (const ob::Error& e) {
    std::printf("ObError: %s\n", e.getMessage());
    return 1;
  }
}

}  // namespace

int main(int argc, char** argv) {
  return probe_main(argc, argv);
}
