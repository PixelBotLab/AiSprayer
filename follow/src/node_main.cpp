// follow_node —— 独立进程，独占 Orbbec，输出「相对示教位」的 6DoF 修正。
//
// 生命周期只有三条路径，每条都必须留下可归因的退出码：
//   配置非法 → 2；相机被别的进程占着 → 3；设备错误/拔线 → 4；没有可用的参考地图 → 5。
// 之所以把"锁被占用"单独分出来：它的处置动作是"去停相机服务"，不是"重启 follow"，
// 而这两种在运维上极易混淆 —— 尤其 18080 端口在相机没插时也可能是 LISTEN 的。
//
// dry_run 是默认值，而且**只能往更安全的方向改**：控制层（P5）没接进来之前，
// dry_run=false 会被强制改回 true 并大声说明。让一个自称在发臂、其实什么都没发的进程
// 存在，比让它拒绝启动危险得多。
#include <signal.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "follow/config_loader.hpp"
#include "follow/frontend.hpp"
#include "follow/health_server.hpp"
#include "follow/logger.hpp"
#include "follow/odometry.hpp"
#include "follow/orbbec_capture.hpp"
#include "follow/pose_io.hpp"
#include "follow/reference_map.hpp"
#include "follow/teach.hpp"
#include "follow/types.hpp"

namespace follow {
namespace {

constexpr int kExitOk = 0;
constexpr int kExitConfig = 2;
constexpr int kExitLockBusy = 3;
constexpr int kExitDevice = 4;
constexpr int kExitNoMap = 5;

std::atomic<int> g_signal{0};

void on_signal(int sig) { g_signal.store(sig); }

void install_signals() {
  struct sigaction sa {};
  sa.sa_handler = on_signal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
  // 健康服务器写 socket 时对端断开不该杀掉进程。
  signal(SIGPIPE, SIG_IGN);
}

struct Args {
  std::string config_path;
  bool teach = false;
  bool list_devices = false;
  bool want_help = false;
  int frames = 0;  // >0 覆盖 runtime.max_cycles
  bool debug = false;
};

void usage(const char* argv0) {
  std::printf(
      "用法: %s [选项]\n"
      "  --config PATH    配置文件（默认 <root>/configs/aisprayer_config.yaml 的 follow: 块）\n"
      "  --teach          重新示教：取一帧，冻结成参考地图并落盘\n"
      "  --frames N       只跑 N 帧后退出（自测用）\n"
      "  --list-devices   枚举设备后退出\n"
      "  --debug          打开 debug 日志\n",
      argv0);
}

bool parse_args(int argc, char** argv, Args* a, std::string* err) {
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&](const char* name, std::string* dst) {
      if (i + 1 >= argc) {
        *err = std::string(name) + " 缺参数";
        return false;
      }
      *dst = argv[++i];
      return true;
    };
    if (k == "--help" || k == "-h") {
      a->want_help = true;
      return false;
    } else if (k == "--config") {
      if (!next("--config", &a->config_path)) return false;
    } else if (k == "--teach") {
      a->teach = true;
    } else if (k == "--list-devices") {
      a->list_devices = true;
    } else if (k == "--debug") {
      a->debug = true;
    } else {
      std::string v;
      if (k == "--frames") {
        if (!next("--frames", &v)) return false;
      } else {
        *err = "不认识参数: " + k;
        return false;
      }
      try {
        a->frames = std::stoi(v);
      } catch (const std::exception&) {
        *err = "--frames 需要整数，得到 " + v;
        return false;
      }
      if (a->frames <= 0) {
        *err = "--frames 必须为正";
        return false;
      }
    }
  }
  return true;
}

int64_t steady_now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// 状态没变时不要每帧重复刷屏（15 fps 下 4 条/秒会把有用信息冲掉），但状态**变化**必须立刻报。
struct EdgeLog {
  std::string last;
  int64_t repeats = 0;
  bool fire(const std::string& s) {
    if (s == last) {
      ++repeats;
      return false;
    }
    if (repeats > 0) {
      LOG_AT(LogLevel::kInfo, "node") << "  （上一条状态重复 " << repeats << " 次）";
    }
    repeats = 0;
    last = s;
    return true;
  }
};

// 多帧均值示教 → 冻结参考地图 → 落盘 → 读回校验哈希（实现在 src/teach.cpp，与 follow_pose 共用：
// 「基准是怎么来的」这件事绝不允许有两个版本）。
std::unique_ptr<ReferenceMap> teach_map(OrbbecCapture& cap, const TrackParams& tp,
                                        const std::string& path, std::string* err) {
  auto map = std::make_unique<ReferenceMap>();
  if (!teach_reference(cap, tp, path, map.get(), err, 10)) {
    return nullptr;
  }
  return map;
}

void fill_correction(const TrackResult& r, HealthSnapshot* s) {
  const DobotPose p = to_dobot(r.T_ref_cam);
  s->correction = {{p.x_mm, p.y_mm, p.z_mm, p.rx_deg, p.ry_deg, p.rz_deg}};
  double st = 0.0, sr = 0.0;
  for (int i = 0; i < 3; ++i) {
    st = std::max(st, r.unc.trans_sigma_mm[i]);
    sr = std::max(sr, r.unc.rot_sigma_deg[i]);
  }
  s->sigma_t_mm = st;
  s->sigma_r_deg = sr;
  s->inlier_ratio = r.inlier_ratio;
  s->gicp_inliers = r.gicp_inliers;
  s->cloud_points = r.cloud_points;
}

int run(const Args& args) {
  std::string err;
  FollowConfig cfg;
  if (!load_config(args.config_path, &cfg, &err)) {
    LOG_AT(LogLevel::kError, "cfg") << err;
    return kExitConfig;
  }
  if (args.debug) {
    set_log_level(LogLevel::kDebug);
  } else {
    set_log_level(cfg.log_level == "debug" ? LogLevel::kDebug : LogLevel::kInfo);
  }

  const std::string root = find_project_root();
  const ConfigProblems probs = check_config(&cfg, root);
  for (const auto& p : probs.items) {
    if (!p.fatal) {
      LOG_AT(LogLevel::kWarn, "cfg") << p.text;
    }
  }
  if (!probs.ok()) {
    LOG_AT(LogLevel::kError, "cfg") << "配置有 " << probs.fatals() << " 条致命问题:\n" << probs.joined();
    return kExitConfig;
  }
  if (args.frames > 0) {
    cfg.max_cycles = args.frames;
  }
  if (!cfg.dry_run) {
    LOG_AT(LogLevel::kWarn, "node")
        << "runtime.dry_run=false 被忽略：控制层（FollowController/ServoP，P5）尚未接入本进程，"
           "现在不会也不该向臂发任何指令。强制按 dry_run 运行。";
    cfg.dry_run = true;
  }
  LOG_AT(LogLevel::kInfo, "cfg") << "\n" << describe(cfg);

  if (args.list_devices) {
    std::vector<std::string> dev = OrbbecCapture::list_devices(&err);
    for (const auto& d : dev) {
      LOG_AT(LogLevel::kInfo, "dev") << d;
    }
    if (!err.empty()) {
      LOG_AT(LogLevel::kError, "dev") << err;
      return kExitDevice;
    }
    LOG_AT(LogLevel::kInfo, "dev") << "共 " << dev.size() << " 台";
    return kExitOk;
  }

  HealthServer health;
  HealthSnapshot snap;
  snap.dry_run = cfg.dry_run;
  snap.map_path = cfg.map_path;
  if (!health.start(cfg.health_port, &err)) {
    LOG_AT(LogLevel::kError, "health") << err;
    return kExitConfig;
  }
  LOG_AT(LogLevel::kInfo, "health") << "监听 127.0.0.1:" << cfg.health_port << "（GET /health / /ready, POST /teach）";

  OrbbecCapture cap;
  snap.state = "capturing";
  snap.lock_held = false;
  health.update(snap);
  if (!cap.open(cfg.capture, &err)) {
    const bool busy = err.find("锁被") != std::string::npos;
    LOG_AT(LogLevel::kError, busy ? "lock" : "dev") << err;
    snap.state = "stopped";
    snap.last_error = err;
    health.update(snap);
    health.stop();
    return busy ? kExitLockBusy : kExitDevice;
  }
  const DeviceCalib& cal = cap.calib();
  snap.align = to_string(cap.align());
  snap.device_present = true;
  snap.lock_held = true;
  LOG_AT(LogLevel::kInfo, "dev")
      << "已开流 对齐=" << snap.align << "  内参 fx=" << cal.color.fx << " fy=" << cal.color.fy
      << " cx=" << cal.color.cx << " cy=" << cal.color.cy
      << " (" << cal.color.width << "x" << cal.color.height << ")";
  // 设备自报的彩色畸变：SW D2C 到底有没有把彩色图整平，本机没能定论（判据见 tools/analyze_d2c.py）。
  // 这里把它量出来报上去，而不是在注释里假设它不重要 —— 角上超过 2 px 就值得盯。
  const double shift = cal.color_dist.corner_shift_px(cal.color);
  LOG_AT(LogLevel::kInfo, "dev") << "彩色畸变 model=" << cal.color_dist.model << " 角上最坏 "
                                 << shift << " px  深度畸变 " << cal.depth_dist.model
                                 << "  基线 " << cal.baseline_mm << " mm";
  if (shift > 2.0) {
    LOG_AT(LogLevel::kWarn, "dev")
        << "彩色畸变角上 " << shift << " px 且未经去畸变：若参考地图与实时帧用了不同的去畸变设置，"
           "修正量会带一个径向系统偏差。";
  }

  TrackParams tp = cfg.track;
  tp.k = cal.color;  // 设备自报、按实际启用的分辨率取；配置里不写内参就是这个理由
  if (!tp.k.valid()) {
    LOG_AT(LogLevel::kError, "dev") << "设备自报内参非法，后续反投影会产出 inf/NaN";
    cap.close();
    health.stop();
    return kExitConfig;
  }

  std::string f_err;
  auto fe = make_frontend(cfg.frontend_kind, cfg.frontend, &f_err);
  if (!fe) {
    LOG_AT(LogLevel::kError, "frontend") << f_err;
    cap.close();
    health.stop();
    return kExitConfig;
  }
  LOG_AT(LogLevel::kInfo, "frontend") << fe->name() << " 就绪";

  std::unique_ptr<ReferenceMap> map;
  const bool have_file = std::filesystem::exists(cfg.map_path);
  if (args.teach || !have_file) {
    snap.state = "teaching";
    health.update(snap);
    if (!args.teach) {
      LOG_AT(LogLevel::kWarn, "teach")
          << "参考地图不存在（" << cfg.map_path << "），自动示教。生产上应当显式 --teach。";
    }
    map = teach_map(cap, tp, cfg.map_path, &err);
    if (!map) {
      LOG_AT(LogLevel::kError, "teach") << "示教失败: " << err;
      snap.state = "stopped";
      snap.last_error = err;
      health.update(snap);
      cap.close();
      health.stop();
      return kExitNoMap;
    }
  } else {
    map = std::make_unique<ReferenceMap>();
    if (!map->load(cfg.map_path, &err)) {
      // 读不回来就退出，绝不"顺手重示教一张"：那是拿现场随便一帧去替换基准，
      // 而现场那一帧可能正好是工件被挪走/夹爪挡住的时候。
      LOG_AT(LogLevel::kError, "teach") << "参考地图存在但加载失败 " << cfg.map_path << ": " << err
                                       << "。确认工件在位后用 --teach 重建。";
      snap.state = "stopped";
      snap.last_error = err;
      health.update(snap);
      cap.close();
      health.stop();
      return kExitNoMap;
    }
    LOG_AT(LogLevel::kInfo, "teach")
        << "载入参考地图 " << cfg.map_path << "  体素=" << map->info().map_voxels
        << "  hash=" << std::hex << map->info().content_hash << std::dec;
    if (std::fabs(map->info().voxel_m - tp.voxel_m) > 1e-9) {
      LOG_AT(LogLevel::kWarn, "teach")
          << "地图 voxel=" << map->info().voxel_m << " m 与配置 track.voxel_m=" << tp.voxel_m
          << " 不一致：对应点门按配置的算，用 --teach 重建才不会半新半旧。";
    }
  }

  snap.map_hash = map->info().content_hash;
  snap.map_points = static_cast<int64_t>(map->info().raw_points);
  snap.map_built_ts_ns = map->info().built_ts_ns;

  const int64_t t_start = steady_now_ms();
  const int64_t max_cycles = cfg.max_cycles;
  int64_t cycle = 0;
  bool device_gone = false;
  EdgeLog edge;

  std::unique_ptr<Tracker> tracker;  // 每次示教都重建：跟丢计数和上一个可信位姿都属于旧基准
  auto reset_tracker = [&]() {
    tracker = std::make_unique<Tracker>(tp, *map, 0x5EEDu);
  };
  reset_tracker();
  snap.state = "tracking";
  health.update(snap);

  double compute_ema = 0.0;
  while (!device_gone && g_signal.load() == 0 && (max_cycles == 0 || cycle < max_cycles)) {
    if (health.take_teach_request()) {
      snap.state = "teaching";
      health.update(snap);
      std::string te;
      auto fresh = teach_map(cap, tp, cfg.map_path, &te);
      if (fresh) {
        map = std::move(fresh);
        reset_tracker();
        snap.map_hash = map->info().content_hash;
        snap.map_points = static_cast<int64_t>(map->info().raw_points);
        snap.map_built_ts_ns = map->info().built_ts_ns;
        snap.last_error.clear();
        LOG_AT(LogLevel::kInfo, "teach") << "重新示教完成，新基准 hash=" << std::hex
                                         << snap.map_hash << std::dec;
      } else {
        snap.last_error = "重新示教失败: " + te;
        LOG_AT(LogLevel::kError, "teach") << snap.last_error;
      }
      snap.state = "tracking";
      health.update(snap);
    }

    RgbdFrame fr;
    if (!cap.wait_frame(&fr, &err)) {
      const CaptureHealth ch = cap.health();
      snap.device_present = ch.device_present;
      snap.dropouts = ch.dropouts;
      snap.unpaired_framesets = ch.unpaired_framesets;
      if (!ch.device_present) {
        device_gone = true;
        snap.last_error = err;
        break;
      }
      ++snap.bad_frames;
      if (edge.fire("wait_frame")) {
        LOG_AT(LogLevel::kWarn, "node") << "取帧失败: " << err;
      }
      health.update(snap);
      continue;
    }

    // 将积攒的板载 IMU 陀螺仪样本注入跟踪器
    std::vector<GyroSample> gyros;
    if (cap.drain_gyro_samples(&gyros)) {
      for (const auto& gs : gyros) {
        tracker->push_gyro(gs.ts_ns, gs.omega_cam_rad_s);
      }
    }

    const int64_t t0 = steady_now_ms();
    FeatureFrame ff = fe->extract(fr.color, fr.ts_ns);
    TrackResult r = tracker->track(ff, fr.depth_mm, fr.ts_ns);
    const double ms = static_cast<double>(steady_now_ms() - t0);
    compute_ema = compute_ema <= 0.0 ? ms : (0.8 * compute_ema + 0.2 * ms);
    ++cycle;

    fill_correction(r, &snap);
    snap.status = to_string(r.status);
    snap.estimator = to_string(r.estimator);
    snap.frames = cycle;
    snap.uptime_ms = steady_now_ms() - t_start;
    const CaptureHealth ch = cap.health();
    snap.period_ms = ch.period_ms;
    snap.fps = ch.period_ms > 0.0 ? 1000.0 / ch.period_ms : 0.0;
    snap.dropouts = ch.dropouts;
    snap.unpaired_framesets = ch.unpaired_framesets;
    snap.device_present = ch.device_present;
    snap.compute_ms = compute_ema;
    snap.last_error.clear();
    health.update(snap);

    char corr[128];
    std::snprintf(corr, sizeof(corr), "dt=[%+.2f %+.2f %+.2f]mm dr=[%+.3f %+.3f %+.3f]deg",
                  snap.correction[0], snap.correction[1], snap.correction[2], snap.correction[3],
                  snap.correction[4], snap.correction[5]);
    const std::string line = std::string(corr) + "  " + snap.estimator + " inl=" +
                             std::to_string(r.gicp_inliers) + " ratio=" +
                             std::to_string(r.inlier_ratio) + " sigma_t=" +
                             std::to_string(snap.sigma_t_mm) + "mm sigma_r=" +
                             std::to_string(snap.sigma_r_deg) + "deg " + std::to_string(ms) + "ms";
    // 每帧一行：dry-run 阶段这就是全部输出，"看不到"等于"没法验收"。
    LOG_AT(LogLevel::kInfo, "track") << "#" << cycle << " t=" << (fr.ts_ns / 1000000) << "ms "
                                     << snap.status << "  " << line;
    if (r.status != Status::kOk && edge.fire(snap.status)) {
      LOG_AT(LogLevel::kWarn, "track") << "状态 " << snap.status << "：修正量按上一个可信位姿保持"
                                       << "（" << line << "）";
    }
  }

  if (g_signal.load() != 0) {
    LOG_AT(LogLevel::kInfo, "node") << "收到信号 " << g_signal.load() << "，收尾退出";
  }
  const CaptureHealth end = cap.health();
  LOG_AT(LogLevel::kInfo, "node")
      << "共 " << cycle << " 帧，设备帧率 " << end.period_ms << " ms/帧（最差 "
      << end.max_period_ms << " ms），dropout=" << end.dropouts
      << "，未配对帧组=" << end.unpaired_framesets;
  snap.state = "stopped";
  snap.uptime_ms = steady_now_ms() - t_start;
  health.update(snap);
  tracker.reset();
  fe.reset();
  cap.close();
  health.stop();
  if (device_gone) {
    LOG_AT(LogLevel::kError, "dev") << "设备离线: " << snap.last_error;
    return kExitDevice;
  }
  return kExitOk;
}

}  // namespace
}  // namespace follow

int main(int argc, char** argv) {
  follow::install_signals();
  follow::Args args;
  std::string err;
  if (!follow::parse_args(argc, argv, &args, &err)) {
    if (args.want_help) {
      follow::usage(argv[0]);
      return follow::kExitOk;
    }
    if (!err.empty()) {
      std::fprintf(stderr, "%s\n", err.c_str());
    }
    return follow::kExitConfig;
  }
  return follow::run(args);
}
