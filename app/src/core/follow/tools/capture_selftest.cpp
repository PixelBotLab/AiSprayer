// P4 设备层的验收门。**跑在插着相机的板子上**，退出码非零就是没过。
//
// 它考的是三件在合成数据上永远考不到的事：
//   1) 独占仲裁真的挡住了第二路打开 —— 抢开一台已被占用的相机不是"降级"，是数据损坏；
//   2) 交付的确实是**成对**的 BGR + 毫米深度，且深度与彩色同网格（下游一套内参的前提）；
//   3) 设备自报的内参/畸变被如实带出来，而不是被"应该已经去畸变了"这句注释替掉。
//
// 时钟类断言只看量级：RK3588 有 DVFS，绝对值会飘；但"彩色配对率""网格一致""valueScale"
// 这些是结构性的，飘不了。
#include <sys/resource.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "follow/device_lock.hpp"
#include "follow/orbbec_capture.hpp"

namespace follow {
namespace {

int g_fail = 0;

std::string fmt(const char* f, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return std::string(buf);
}

void check(bool ok, const std::string& what) {
  std::printf("  %s %s\n", ok ? "OK  " : "FAIL", what.c_str());
  if (!ok) {
    ++g_fail;
  }
}

double cpu_ms() {
  struct rusage ru {};
  ::getrusage(RUSAGE_SELF, &ru);
  const auto sec = [](const timeval& t) { return static_cast<double>(t.tv_sec) + t.tv_usec * 1e-6; };
  return (sec(ru.ru_utime) + sec(ru.ru_stime)) * 1000.0;
}

// 锁必须真的互斥。flock 按 open file description 计数，所以同进程内两个 fd 就会互相挡住 ——
// 这条不需要另起进程也能测出来。
void test_lock(const std::string& path) {
  std::printf("\n[仲裁] %s\n", path.c_str());
  DeviceLock a, b;
  std::string err;
  check(a.acquire(path, &err), "第一把锁拿到：" + (err.empty() ? std::string("ok") : err));

  DeviceLock::Busy busy;
  const bool second = b.acquire(path, &err, &busy);
  check(!second, "第二把锁被拒（拿到就是互斥失效）");
  check(busy.held_by_other, "被拒时能区分出\"别人在用\"而不是笼统的失败");
  check(busy.holder_pid == static_cast<int>(::getpid()),
        "报得出持锁 pid（实测同进程也报得出来）=" + std::to_string(busy.holder_pid));

  a.release();
  err.clear();
  check(b.acquire(path, &err), "释放后能再拿到：" + (err.empty() ? std::string("ok") : err));
  b.release();
}

void test_capture(int w, int h, int fps, const std::string& lock_path, int frames) {
  std::printf("\n[取流] %dx%d@%d\n", w, h, fps);
  CaptureParams p;
  p.width = w;
  p.height = h;
  p.fps = fps;
  p.lock_path = lock_path;

  // 先占着锁，open() 必须在**碰设备之前**就被挡住
  DeviceLock hog;
  std::string herr;
  if (!hog.acquire(lock_path, &herr)) {
    check(false, "自己占不上锁，这条没法测: " + herr);
    return;
  }
  {
    OrbbecCapture busy_cap;
    std::string err;
    const bool ok = busy_cap.open(p, &err);
    check(!ok, "锁被占用时 open() 失败（开了才是灾难）");
    const bool actionable = err.find("锁") != std::string::npos &&
                            err.find_first_of("0123456789") != std::string::npos;
    check(actionable, "失败原因点得出是谁占着（带 pid 才可操作），不是笼统的打不开: [" + err + "]");
  }
  hog.release();

  OrbbecCapture cap;
  const double t0 = cpu_ms();
  std::string err;
  if (!cap.open(p, &err)) {
    check(false, "open() 成功: " + err);
    return;
  }
  check(true, "open() 成功，对齐=" + std::string(to_string(cap.align())) + "，暖机丢弃 " +
                  std::to_string(cap.health().unpaired_framesets) + " 个只含深度的帧组");
  std::printf("       启动耗时 %.0f ms（含等第一个成对帧组）\n", cpu_ms() - t0);

  const DeviceCalib& c = cap.calib();
  std::printf("       rgb fx=%.3f fy=%.3f cx=%.3f cy=%.3f (%dx%d) | depth fx=%.3f cx=%.3f cy=%.3f\n",
              c.color.fx, c.color.fy, c.color.cx, c.color.cy, c.color.width, c.color.height,
              c.raw_depth.fx, c.raw_depth.cx, c.raw_depth.cy);
  std::printf("       rgb 畸变 %s k1=%.5f k2=%.5f k3=%.5f p1=%.5f p2=%.5f ⇒ 角上最坏 %.2f px\n",
              c.color_dist.model.c_str(), c.color_dist.k1, c.color_dist.k2, c.color_dist.k3,
              c.color_dist.p1, c.color_dist.p2, c.color_dist.corner_shift_px(c.color));
  std::printf("       depth 畸变 %s（全零=%d）| D2C 基线 %.3f mm\n", c.depth_dist.model.c_str(),
              c.depth_dist.all_zero() ? 1 : 0, c.baseline_mm);
  check(c.valid(), "内参合法（fx/fy>0 且有限）");
  check(c.color.width == w && c.color.height == h,
        "标定组按启用的分辨率取到（" + std::to_string(c.color.width) + "x" +
            std::to_string(c.color.height) + "）");

  int got = 0, bad_grid = 0, bad_type = 0;
  int64_t ts_first = 0, ts_last = 0;
  long long valid = 0, pixels = 0;
  double min_mm = 1e18, max_mm = -1e18;
  const double cpu1 = cpu_ms();
  const auto wall0 = std::chrono::steady_clock::now();
  while (got < frames) {
    RgbdFrame f;
    std::string ferr;
    if (!cap.wait_frame(&f, &ferr)) {
      check(false, "第 " + std::to_string(got) + " 帧取不到: " + ferr);
      break;
    }
    if (f.color.type() != CV_8UC3) {
      ++bad_type;
    }
    if (f.depth_mm.type() != CV_16UC1 || f.depth_mm.rows != f.color.rows ||
        f.depth_mm.cols != f.color.cols) {
      ++bad_grid;
    }
    ++got;
    if (!ts_first) ts_first = f.ts_ns;
    ts_last = f.ts_ns;
    for (int y = 0; y < f.depth_mm.rows; y += 7) {
      const uint16_t* row = f.depth_mm.ptr<uint16_t>(y);
      for (int x = 0; x < f.depth_mm.cols; x += 7) {
        ++pixels;
        if (row[x] == 0 || row[x] >= 65000) {
          continue;
        }
        ++valid;
        min_mm = std::min(min_mm, static_cast<double>(row[x]));
        max_mm = std::max(max_mm, static_cast<double>(row[x]));
      }
    }
  }
  const double cpu = cpu_ms() - cpu1;
  const double wall = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - wall0).count();
  const CaptureHealth hs = cap.health();
  const double dev_fps =
      (got > 1 && ts_last > ts_first)
          ? 1e9 * static_cast<double>(got - 1) / static_cast<double>(ts_last - ts_first)
          : 0.0;
  std::printf("       %d 帧：墙钟 %.1f fps | 设备时钟 %.1f fps | EMA 周期 %.1f ms max %.1f ms | "
              "dropout=%ld | 取流+拷贝 CPU %.1f ms/帧\n",
              got, got > 1 ? 1000.0 * (got - 1) / wall : 0.0, dev_fps, hs.period_ms,
              hs.max_period_ms, static_cast<long>(hs.dropouts), got ? cpu / got : 0.0);
  check(bad_type == 0, "彩色一律 CV_8UC3 BGR（不合格 " + std::to_string(bad_type) + "）");
  check(bad_grid == 0, "深度与彩色同网格 CV_16UC1（不合格 " + std::to_string(bad_grid) + "）");
  const double valid_pct = pixels ? 100.0 * static_cast<double>(valid) / static_cast<double>(pixels)
                                  : 0.0;
  check(valid_pct > 10.0, fmt("深度有效率 %.1f%% > 10%%（近处 %.0f mm，远 %.0f mm）", valid_pct,
                              min_mm, max_mm));
  check(std::abs(hs.d2c_offset_ms) < 5.0,
        fmt("配对偏差 |color−depth| = %.3f ms < 5 ms", hs.d2c_offset_ms));
  cap.close();
  check(!cap.is_open(), "close() 之后不再生称打开");

  DeviceLock again;
  const bool free_again = again.acquire(lock_path, &err);
  check(free_again, "close() 释放了锁（没释放就是下次开机启不动的根因）" +
                        (free_again ? std::string() : ": " + err));
  again.release();
}

int selftest_main(int argc, char** argv) {
  std::string lock_path = "/tmp/follow_orbbec.lock";
  int w = 848, h = 480, fps = 30, frames = 60;
  bool no_device = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--lock" && i + 1 < argc) {
      lock_path = argv[++i];
    } else if (a == "--w" && i + 1 < argc) {
      w = std::atoi(argv[++i]);
    } else if (a == "--h" && i + 1 < argc) {
      h = std::atoi(argv[++i]);
    } else if (a == "--fps" && i + 1 < argc) {
      fps = std::atoi(argv[++i]);
    } else if (a == "--n" && i + 1 < argc) {
      frames = std::atoi(argv[++i]);
    } else if (a == "--no-device") {
      no_device = true;
    } else {
      std::printf("用法: follow_capture_selftest [--lock 路径] [--w 848] [--h 480] [--fps 30] "
                  "[--n 60] [--no-device]\n");
      return 2;
    }
  }

  std::printf("follow_capture_selftest：设备层验收（退出码非零 = 没过）\n");
  test_lock(lock_path);
  if (no_device) {
    std::printf("\n（--no-device：跳过取流，只验仲裁）\n");
    return g_fail == 0 ? 0 : 1;
  }
  std::string lerr;
  const auto devs = OrbbecCapture::list_devices(&lerr);
  for (const auto& d : devs) {
    std::printf("  看到设备: %s\n", d.c_str());
  }
  if (devs.empty()) {
    std::printf("  %s\n", lerr.empty() ? "没有 Orbbec 设备" : lerr.c_str());
    ++g_fail;
    return 1;
  }
  test_capture(w, h, fps, lock_path, frames);
  std::printf("\n结论: %s（%d 条断言未过）\n", g_fail == 0 ? "设备层过门" : "没过", g_fail);
  return g_fail == 0 ? 0 : 1;
}

}  // namespace
}  // namespace follow

int main(int argc, char** argv) { return follow::selftest_main(argc, argv); }
