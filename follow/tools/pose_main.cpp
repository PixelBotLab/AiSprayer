// follow_pose —— 手拿着相机移动，控制台实时打印位姿（空间坐标 + 姿态）。
//
// 它存在的理由只有一个：把「这套算法报的数对不对」变成一件能拿尺子量出来的事。
// 因此它和 follow_node 共用**同一份**配置解析、**同一个**示教实现、**同一个** Tracker
// 和**同一个** to_dobot 欧拉角约定 —— 演示里量对的东西，到生产上才会以同样的方式对。
// 任何在这里另写一遍的数学，都是未来「demo 对、节点错」的成因。
#include <signal.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "follow/config_loader.hpp"
#include "follow/frontend.hpp"
#include "follow/logger.hpp"
#include "follow/odometry.hpp"
#include "follow/orbbec_capture.hpp"
#include "follow/pose_io.hpp"
#include "follow/pose_smoother.hpp"
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
}

int64_t steady_now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// 「手持不动时读数应该不动」是用户唯一不需要设备就能做的准确性验收。判据用**1 秒滑动窗口内
// 的逐轴跨度**，不用相邻两帧之差：本机实测静止时逐轴 sd 就有 0.2~0.8 mm，帧间差是它的 √2 倍，
// 任何小于噪声的"帧间阈值"永远咬不住，静止段统计会恒为 0（第一版就是这么错的）。
// 阈值放宽到 5 mm / 0.5° 后，慢移动仍可能被算进"静止"——那只会把 sd 推**大**，
// 报出来的重复性偏保守，不会反过来吹。
constexpr int kWinFrames = 15;  // @15fps = 1 秒
constexpr double kStillRangeMm = 5.0;
constexpr double kStillRangeDeg = 0.5;

constexpr double kRad2Deg = 57.29577951308232;

// 刷屏的根源不是"打印太频繁"，是**拿上一帧当基准**：手持时逐轴 sd 就有 0.3~1.3 mm，任何能
// 抓住真实移动的帧间阈值都会把噪声一起放出来。所以阈值比的是**上一次打印**，不是上一帧。
// 副作用要说清楚：低于阈值的漂移会累计而不打印（10 次 0.9 mm = 9 mm），所以另加一条心跳，
// 顺带把"自上次打印以来的最大偏离"报出来 —— 沉默是有内容撑着的，不是没在算。
constexpr double kDeadbandTmm = 1.0;
constexpr double kDeadbandRdeg = 0.1;
constexpr int64_t kHeartbeatMs = 5000;
// 平滑窗口的默认值在 follow/pose_smoother.hpp（kDefaultSmoothFrames）：相机服务里的 worker
// 用的就是同一个数，改一处两边一起变，不会出现"demo 稳、页面抖"。

// 启动横幅用颜色分块；非终端（管道/重定向）或设了 NO_COLOR 时必须关掉，否则转义序列会
// 变成字面上的 ^[[36m 混进日志里。
struct Palette {
  bool on = false;
  const char* dim = "";
  const char* cyan = "";
  const char* green = "";
  const char* yellow = "";
  const char* red = "";
  const char* bold = "";
  const char* off = "";

  Palette() {
    if (!isatty(STDOUT_FILENO) || ::getenv("NO_COLOR") != nullptr) {
      return;
    }
    on = true;
    dim = "\033[2m";
    cyan = "\033[36m";
    green = "\033[32m";
    yellow = "\033[33m";
    red = "\033[31m";
    bold = "\033[1m";
    off = "\033[0m";
  }
  // 正常行不着色：一屏全是颜色等于没有颜色。只有"这帧不可信"才值得跳出来。
  const char* status(const char* s) const {
    if (!on) {
      return "";
    }
    return std::strcmp(s, "ok") == 0 ? "" : (std::strcmp(s, "degenerate") == 0 ? yellow : red);
  }
  // 先按宽度补空格再包颜色：转义序列自己也占字符数，直接放进 %-8s 会把整列拉歪。
  std::string wrap(const char* code, const std::string& s) const {
    if (code == nullptr || *code == '\0') {
      return s;
    }
    return std::string(code) + s + off;
  }
};

struct Args {
  std::string config_path;
  int frames = 0;  // >0 = 跑这么多帧就退出（没有终端时也能自测）
  double deadband_t = kDeadbandTmm;
  double deadband_r = kDeadbandRdeg;
  int smooth = kDefaultSmoothFrames;
  bool debug = false;
  bool want_help = false;
};

void usage(const char* argv0) {
  std::printf(
      "用法: %s [选项]\n"
      "  -c, --config PATH     配置文件（默认 <项目根>/configs/aisprayer_config.yaml）\n"
      "  -n, --frames N        跑 N 帧后退出（默认一直跑；s 或 Ctrl-C 停止）\n"
      "  -t, --deadband-mm V   打印阈值：与**上次打印**的任一轴位移差 > V mm 才打印（默认 %.1f）\n"
      "  -r, --deadband-deg V  打印阈值：任一轴姿态差 > V deg 才打印（默认 %.1f）\n"
      "  -m, --smooth N        显示位姿 = 最近 N 帧平均（默认 %.0f；1 = 显示原始单帧，噪声大时会刷屏）\n"
      "      --debug           打开 debug 日志\n"
      "  -h, --help            这条说明\n\n"
      "运行中: z 或 Enter = 以当前画面重新调零，q 或 s = 停止并打印本段精度小结。\n"
      "行只在任一轴与上次打印相差超过阈值时才打印；另有 %.0f 秒一次的心跳行报「其间最大偏离」，"
      "静默不等于没在算。\n",
      argv0, kDeadbandTmm, kDeadbandRdeg, static_cast<double>(kDefaultSmoothFrames),
      static_cast<double>(kHeartbeatMs) / 1000.0);
}

bool parse_args(int argc, char** argv, Args* a, std::string* err) {
  for (int i = 1; i < argc; ++i) {
    const std::string s = argv[i];
    const char* nxt = i + 1 < argc ? argv[i + 1] : nullptr;
    auto need = [&](const char* flag) -> bool {
      if (!nxt) {
        *err = std::string(flag) + " 需要一个参数";
        return false;
      }
      return true;
    };
    if (s == "-h" || s == "--help") {
      a->want_help = true;
      return false;
    } else if (s == "-c" || s == "--config") {
      if (!need(s.c_str())) {
        return false;
      }
      a->config_path = argv[++i];
    } else if (s == "-n" || s == "--frames") {
      if (!need(s.c_str())) {
        return false;
      }
      const std::string v = argv[++i];
      a->frames = std::atoi(v.c_str());
      if (a->frames < 0 || (a->frames == 0 && v != "0")) {
        *err = "--frames 需要一个非负整数，收到 " + v + "（0 = 一直跑到按 s）";
        return false;
      }
    } else if (s == "-m" || s == "--smooth") {
      if (!need(s.c_str())) {
        return false;
      }
      const std::string v = argv[++i];
      a->smooth = std::atoi(v.c_str());
      if (a->smooth < 1 || a->smooth > 30) {
        *err = "--smooth 需要 1~30 之间的整数（1 = 原始单帧），收到 " + v;
        return false;
      }
    } else if (s == "--debug") {
      a->debug = true;
    } else if (s == "-t" || s == "--deadband-mm" || s == "-r" || s == "--deadband-deg") {
      if (!need(s.c_str())) {
        return false;
      }
      const std::string v = argv[++i];
      char* end = nullptr;
      const double d = std::strtod(v.c_str(), &end);
      if (end == v.c_str() || *end != '\0' || !std::isfinite(d) || d < 0.0) {
        *err = s + " 需要一个非负实数，收到 " + v + "（0 = 每帧都打，正是这个阈值要避免的刷屏）";
        return false;
      }
      if (s == "-t" || s == "--deadband-mm") {
        a->deadband_t = d;
      } else {
        a->deadband_r = d;
      }
    } else {
      *err = "无法识别的参数: " + s + "（--help 看用法）";
      return false;
    }
  }
  return true;
}

// Enter / q 必须在取帧循环里顺手查一次，绝不能阻塞等键盘：等一次输入就把画面冻住，
// 而用户最想调零的时刻恰恰是手里正在动的时候。
class KeyPoller {
 public:
  KeyPoller() {
    if (!isatty(STDIN_FILENO)) {
      return;  // 重定向/管道下不碰 termios，否则会把调用方的终端设置改坏
    }
    if (tcgetattr(STDIN_FILENO, &orig_) != 0) {
      return;
    }
    termios raw = orig_;
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      on_ = true;
    }
  }
  ~KeyPoller() { restore(); }
  KeyPoller(const KeyPoller&) = delete;
  KeyPoller& operator=(const KeyPoller&) = delete;

  void restore() {
    if (on_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
      on_ = false;
    }
  }
  // 0 = 无输入。一次吞掉多个字节：误按连发时不把残字符漏给下一帧（否则一次调零键可能被
  // 读成两次，而调零会换基准）。
  int poll() {
    if (!on_) {
      return 0;
    }
    unsigned char buf[16];
    const ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
    int key = 0;
    for (ssize_t i = 0; i < n; ++i) {
      const unsigned char c = static_cast<unsigned char>(std::tolower(buf[i]));
      if (buf[i] == 3 || c == 'q' || c == 's') {  // 3 = Ctrl-C：本进程自己收尾，把终端改回来
        return buf[i] == 3 ? 3 : c;
      }
      if (c == 'z') {
        key = 'z';
      } else if (buf[i] == '\n' || buf[i] == '\r') {
        key = '\n';
      }
    }
    return key;
  }
  bool interactive() const { return on_; }

 private:
  termios orig_ {};
  bool on_ = false;
};

struct Agg {
  int n = 0;
  double mn = 0.0, mx = 0.0, sum = 0.0, sum2 = 0.0;
  void add(double v) {
    if (n == 0) {
      mn = mx = v;
    } else {
      mn = std::min(mn, v);
      mx = std::max(mx, v);
    }
    ++n;
    sum += v;
    sum2 += v * v;
  }
  double mean() const { return n ? sum / n : NAN; }
  double sd() const {
    if (n < 2) {
      return NAN;
    }
    const double var = (sum2 - sum * sum / n) / (n - 1);
    return var > 0.0 ? std::sqrt(var) : 0.0;
  }
};

std::string fmt_axis(const Agg& a) {
  if (a.n == 0) {
    return "无数据";
  }
  char buf[96];
  std::snprintf(buf, sizeof(buf), "min %+.2f max %+.2f 均值 %+.2f sd %.3f", a.mn, a.mx, a.mean(),
                a.sd());
  return buf;
}

std::string fmt_pair(const Agg& a) {
  if (a.n == 0) {
    return "无数据";
  }
  char buf[96];
  std::snprintf(buf, sizeof(buf), "均值 %.3f 最大 %.3f", a.mean(), a.mx);
  return buf;
}

struct Pose {
  std::array<double, 3> t {}, r {};  // mm / deg
  double norm_t = 0.0;               // |t| mm
  double norm_r = 0.0;               // 轴角 |θ| deg
};

// 打印闸门。四种情况必须出声，其余静默但继续统计：
//   first = 本段（调零后）第一帧，不然用户看到的第一个数没有基准；
//   state = 状态或解算器变了（out_of_envelope / lost 被藏住是最糟的失效）；
//   move  = 任一轴相对**上一次打印**超过阈值；
//   beat  = 超过心跳周期还没出声 —— 顺带报出"其间最大偏离"，让沉默可解释（否则看起来像卡死）。
class PrintGate {
 public:
  PrintGate(double dt_mm, double dr_deg) : dt_(dt_mm), dr_(dr_deg) {}

  enum class Why { kSkip, kFirst, kState, kMove, kBeat };

  Why check(const Pose& p, const std::string& key, int64_t now_ms) {
    Why w = Why::kSkip;
    if (!have_) {
      w = Why::kFirst;
    } else {
      for (int i = 0; i < 3; ++i) {
        dev_t_ = std::max(dev_t_, std::fabs(p.t[i] - last_.t[i]));
        dev_r_ = std::max(dev_r_, std::fabs(p.r[i] - last_.r[i]));
      }
      if (key != state_) {
        w = Why::kState;
      } else if (dev_t_ > dt_ || dev_r_ > dr_) {
        w = Why::kMove;
      } else if (now_ms - last_print_ms_ >= kHeartbeatMs) {
        w = Why::kBeat;
      }
    }
    if (w == Why::kSkip) {
      ++quiet_;
      return w;
    }
    snap_t_ = dev_t_;
    snap_r_ = dev_r_;
    snap_quiet_ = quiet_;
    dev_t_ = dev_r_ = 0.0;
    quiet_ = 0;
    suppressed_ += snap_quiet_;
    last_ = p;
    state_ = key;
    have_ = true;
    last_print_ms_ = now_ms;
    return w;
  }

  double snap_t() const { return snap_t_; }
  double snap_r() const { return snap_r_; }
  int64_t snap_quiet() const { return snap_quiet_; }
  int64_t suppressed() const { return suppressed_; }
  double dt() const { return dt_; }
  double dr() const { return dr_; }

 private:
  double dt_, dr_;
  Pose last_ {};
  std::string state_;
  bool have_ = false;
  double dev_t_ = 0.0, dev_r_ = 0.0;
  int64_t quiet_ = 0;
  int64_t suppressed_ = 0;
  double snap_t_ = 0.0, snap_r_ = 0.0;
  int64_t snap_quiet_ = 0;
  int64_t last_print_ms_ = 0;
};

// 显示位姿的 N 帧平均在 follow/pose_smoother.hpp —— 相机服务里推给页面/臂的那一路用的是
// 同一个类，所以"演示里看着稳"才蕴含"页面上看着稳"。这里只留消费方。

// 一段 = 两次调零之间。参考系在调零那一刻换了，所以运动包络和静止重复性都必须按段算：
// 跨两个零点求 min/max 等于把两套坐标里的数混在一起，得到的"范围"没有任何物理含义。
class Segment {
 public:
  void note(const Pose& p, const std::array<double, 3>& sig_t, const std::array<double, 3>& sig_r,
            double jump_t, double jump_r, double ms, const std::string& state_key) {
    if (frames_ == 0) {
      first_ = p;
    }
    ++frames_;
    for (int i = 0; i < 3; ++i) {
      tt_[i].add(p.t[i]);
      rr_[i].add(p.r[i]);
      st_[i].add(sig_t[i]);
      sr_[i].add(sig_r[i]);
    }
    cms_.add(ms);
    hist_[state_key]++;
    max_abs_t_ = std::max(max_abs_t_, p.norm_t);
    max_abs_r_ = std::max(max_abs_r_, p.norm_r);
    if (has_prev_) {
      max_jump_t_ = std::max(max_jump_t_, jump_t);
      max_jump_r_ = std::max(max_jump_r_, jump_r);
    }
    has_prev_ = true;

    win_.push_back(p);
    if (static_cast<int>(win_.size()) > kWinFrames) {
      win_.pop_front();
    }
    if (static_cast<int>(win_.size()) == kWinFrames) {
      std::array<Agg, 3> wsd_t {}, wsd_r {};
      for (const Pose& q : win_) {
        for (int i = 0; i < 3; ++i) {
          wsd_t[i].add(q.t[i]);
          wsd_r[i].add(q.r[i]);
        }
      }
      double range_t = 0.0, range_r = 0.0;
      for (int i = 0; i < 3; ++i) {
        // 每个窗口都记一份 sd，所以即使全程都在动，quiet_ 的最小值也是"最安静的那 1 秒"的波动。
        quiet_t_[i].add(wsd_t[i].sd());
        quiet_r_[i].add(wsd_r[i].sd());
        range_t = std::max(range_t, wsd_t[i].mx - wsd_t[i].mn);
        range_r = std::max(range_r, wsd_r[i].mx - wsd_r[i].mn);
      }
      if (range_t <= kStillRangeMm && range_r <= kStillRangeDeg) {
        for (int i = 0; i < 3; ++i) {
          still_t_[i].add(wsd_t[i].sd());
          still_r_[i].add(wsd_r[i].sd());
        }
        ++still_wins_;
      }
    }
  }

  int64_t frames() const { return frames_; }

  void report(const char* why) const {
    std::printf("\n—— 本段小结（%s）共 %lld 帧 ——\n", why, static_cast<long long>(frames_));
    if (frames_ == 0) {
      std::printf("  一帧都没跑成，没有可报的数。\n");
      return;
    }
    const char* ax[3] = {"X(右)", "Y(下)", "Z(前)"};
    std::printf("  调零后第 1 帧读数: t=[%+.2f %+.2f %+.2f]mm |t|=%.2f  —— 手持不动时这里应接近 0，\n",
                first_.t[0], first_.t[1], first_.t[2], first_.norm_t);
    std::printf("      它就是「示教那一帧 vs 紧接着的一帧」的自洽性检查，>2mm 说明示教后相机被移动过。\n");
    for (int i = 0; i < 3; ++i) {
      std::printf("  位置 %-5s mm  %s\n", ax[i], fmt_axis(tt_[i]).c_str());
    }
    const char* ra[3] = {"rx", "ry", "rz"};
    for (int i = 0; i < 3; ++i) {
      std::printf("  姿态 %-5s deg %s\n", ra[i], fmt_axis(rr_[i]).c_str());
    }
    std::printf("  运动幅度  最远离调零点 %.2f mm / %.2f deg；最大单帧跳变 %.2f mm / %.2f deg\n",
                max_abs_t_, max_abs_r_, max_jump_t_, max_jump_r_);
    std::printf("  估计器自报 1σ  t: %s / %s / %s mm   r: %s / %s / %s deg\n",
                fmt_pair(st_[0]).c_str(), fmt_pair(st_[1]).c_str(), fmt_pair(st_[2]).c_str(),
                fmt_pair(sr_[0]).c_str(), fmt_pair(sr_[1]).c_str(), fmt_pair(sr_[2]).c_str());
    if (quiet_t_[0].n > 0) {
      std::printf("  1 秒窗口波动  最安静的一档 sd:");
      for (int i = 0; i < 3; ++i) {
        std::printf(" %s %.3f", ax[i], quiet_t_[i].mn);
      }
      for (int i = 0; i < 3; ++i) {
        std::printf(" %s %.3f", ra[i], quiet_r_[i].mn);
      }
      std::printf(" （mm/deg，全程 %d 个窗口里最小的一个，含慢移动）\n", quiet_t_[0].n);
    }
    if (still_wins_ == 0) {
      std::printf("  静止重复性  没抓到静止窗口（要求 1 秒内逐轴跨度 <= %.1f mm / %.1f deg）。"
                  "手持时稳住两三秒，这一项就是拿尺子量不出来的那部分误差。\n",
                  kStillRangeMm, kStillRangeDeg);
    } else {
      std::printf("  静止重复性  %d 个静止 1 秒窗口内的逐轴 sd：\n", still_wins_);
      for (int i = 0; i < 3; ++i) {
        std::printf("      %s 均值 %.3f 最大 %.3f mm\n", ax[i], still_t_[i].mean(), still_t_[i].mx);
      }
      for (int i = 0; i < 3; ++i) {
        std::printf("      %s   均值 %.3f 最大 %.3f deg\n", ra[i], still_r_[i].mean(),
                    still_r_[i].mx);
      }
      // 自报 σ 只是"配准自己觉得多准"。实测静止 sd 明显大于它，说明 σ 偏乐观 —— 而
      // track.trans_sigma_mm / rot_sigma_deg 两条可观测门是拿 σ 判的，这个比值就是该不该
      // 收紧门限的直接证据。
      std::printf("  对照  实测静止 sd ÷ 自报 1σ（>2 说明 σ 偏乐观，门限该重校）: ");
      for (int i = 0; i < 3; ++i) {
        char buf[24];
        std::snprintf(buf, sizeof(buf), "%.1f",
                      st_[i].mean() > 1e-9 ? still_t_[i].mean() / st_[i].mean() : NAN);
        std::printf("%s %s× ", ax[i], st_[i].mean() > 1e-9 ? buf : "-");
      }
      std::printf("\n");
    }
    std::printf("  状态  ");
    for (const auto& kv : hist_) {
      std::printf("%s×%d  ", kv.first.c_str(), kv.second);
    }
    std::printf("\n  计算耗时  均值 %.1f ms  最大 %.1f ms\n", cms_.mean(), cms_.mx);
  }

 private:
  int64_t frames_ = 0;
  Pose first_ {};
  std::array<Agg, 3> tt_ {}, rr_ {}, st_ {}, sr_ {};
  std::array<Agg, 3> still_t_ {}, still_r_ {};
  std::array<Agg, 3> quiet_t_ {}, quiet_r_ {};
  int still_wins_ = 0;
  double max_abs_t_ = 0.0, max_abs_r_ = 0.0, max_jump_t_ = 0.0, max_jump_r_ = 0.0;
  Agg cms_;
  std::map<std::string, int> hist_;
  std::deque<Pose> win_;
  bool has_prev_ = false;
};

Pose make_pose(const Eigen::Isometry3d& T) {
  const DobotPose dp = to_dobot(T);  // 与发给臂的数同一套换算，演示才不会自相矛盾
  Pose p;
  p.t = {{dp.x_mm, dp.y_mm, dp.z_mm}};
  p.r = {{dp.rx_deg, dp.ry_deg, dp.rz_deg}};
  p.norm_t = std::sqrt(p.t[0] * p.t[0] + p.t[1] * p.t[1] + p.t[2] * p.t[2]);
  p.norm_r = Eigen::AngleAxisd(T.rotation()).angle() * kRad2Deg;
  return p;
}

double rot_step_deg(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle() * kRad2Deg;
}

void print_pose_line(const Palette& pal, const PrintGate& gate, int64_t n, const Pose& p,
                     const TrackResult& r, double st_max, double sr_max, double ms) {
  char state[16], est[16];
  std::snprintf(state, sizeof(state), "%-8s", to_string(r.status));
  std::snprintf(est, sizeof(est), "%-6s", to_string(r.estimator));
  std::printf(
      "#%6lld  X=%+8.2f Y=%+8.2f Z=%+8.2f mm  rx=%+7.2f ry=%+7.2f rz=%+7.2f deg"
      "  |t|=%7.2f |r|=%6.2f  %s/%s inl=%5d(%4.2f) sT=%5.2f sR=%5.3f %5.1fms",
      static_cast<long long>(n), p.t[0], p.t[1], p.t[2], p.r[0], p.r[1], p.r[2], p.norm_t, p.norm_r,
      pal.wrap(pal.status(to_string(r.status)), state).c_str(), est, r.gicp_inliers, r.inlier_ratio,
      st_max, sr_max, ms);
  if (r.gyro_pushed > 0 || r.gyro_samples > 0 || r.gyro_bias_ready) {
    std::printf("  gyro=%d/%d pushed/samples bias=%.3f resid=%.3f deg/s still=%s used=%s",
                r.gyro_pushed, r.gyro_samples, r.gyro_bias_rad_s * kRad2Deg,
                r.gyro_resid_rad_s * kRad2Deg, r.gyro_still ? "YES" : "NO",
                r.gyro_used ? "YES" : "NO");
  }
  if (gate.snap_quiet() > 0) {
    std::printf("  %s〔静默 %lld 帧 · 其间最大偏离 %.2fmm/%.3fdeg〕%s", pal.dim,
                static_cast<long long>(gate.snap_quiet()), gate.snap_t(), gate.snap_r(), pal.off);
  }
  if (r.status != Status::kOk) {
    // 出包络/跟丢时报的位姿是**上一个可信值**，不标出来就会被当成当前读数拿去判断。
    std::printf("  %s<保持上一位姿>%s", pal.red, pal.off);
  }
  std::printf("%s\n", pal.off);
  std::fflush(stdout);
}

// 启动横幅。**在拿到第一个真实帧组之后**才打印，不是在 open() 之后：open() 只知道"请求了
// 什么"，而"设备实际交付了什么"（对齐后深度落到彩色网格、每个流真正的宽高）必须看帧本身。
// 把两者混成一行"848x480@15"就是放弃了一次免费的一致性检查。
void print_banner(const Palette& pal, const FollowConfig& cfg, const OrbbecCapture& cap,
                  const RgbdFrame& fr, const CaptureHealth& ch, bool interactive,
                  const Args& args) {
  const DeviceCalib& cal = cap.calib();
  std::printf("\n%s══ 流与帧率 %s\n", pal.cyan, pal.off);
  std::printf("  %s彩色%s  请求 %dx%d@%d fps   交付 %dx%d %d通道  内参 fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n",
              pal.bold, pal.off, cfg.capture.width, cfg.capture.height, cfg.capture.fps,
              fr.color.cols, fr.color.rows, fr.color.channels(), cal.color.fx, cal.color.fy,
              cal.color.cx, cal.color.cy);
  std::printf("  %s深度%s  请求 %dx%d@%d fps   交付 %dx%d CV_16UC1（1 LSB = 1 mm）  内参 fx=%.2f cx=%.2f (%dx%d)\n",
              pal.bold, pal.off, cfg.capture.width, cfg.capture.height, cfg.capture.fps,
              fr.depth_mm.cols, fr.depth_mm.rows, cal.raw_depth.fx, cal.raw_depth.cx,
              cal.raw_depth.width, cal.raw_depth.height);
  std::printf("        对齐=%s  基线=%.3f mm  彩色畸变 %s 角上最坏 %.2f px\n",
              to_string(cap.align()), cal.baseline_mm, cal.color_dist.model.c_str(),
              cal.color_dist.corner_shift_px(cal.color));
  if (ch.period_ms > 0.0) {
    std::printf("  %s实测%s  帧周期 %.2f ms = %.2f fps（最差 %.2f ms）  色深时间差 %+.2f ms  未配对帧组 %lld\n",
                pal.bold, pal.off, ch.period_ms, 1000.0 / ch.period_ms, ch.max_period_ms,
                ch.d2c_offset_ms, static_cast<long long>(ch.unpaired_framesets));
  }
  if (cal.has_imu) {
    std::printf("  %s惯导%s  板载 IMU 陀螺仪已开流  采样率 ~%d Hz  外参 %s  t=(%.2f, %.2f, %.2f) mm\n",
                pal.bold, pal.off, cal.gyro_sample_rate_hz,
                cal.gyro_extrinsics_loaded ? "T_cam_gyro" : "Identity（设备未标定，零偏残差会随姿态变）",
                cal.t_cam_gyro[0], cal.t_cam_gyro[1], cal.t_cam_gyro[2]);
  } else {
    std::printf("  %s惯导%s  未开启或设备无 IMU 陀螺仪\n", pal.bold, pal.off);
  }

  std::printf("\n%s参考系%s = 上一次调零那一刻的相机坐标轴：X=右  Y=下  Z=前(沿光轴指向工件)，单位 mm。\n",
              pal.cyan, pal.off);
  std::printf("姿态 rx/ry/rz = 内禀 'xyz'（定系矩阵乘序 R = Rz(rz)·Ry(ry)·Rx(rx)，deg），与发臂的 "
              "ServoP、与 apps/calib 的 DOBOT_EULER_SEQ 同一约定。\n");
  std::printf("本工具与相机服务里的 follow 库跑的是%s同一份 libfollow%s：这里的读数就是页面仿真臂"
              "看到的增量，两边不一致即其中一边接错了。\n",
              pal.bold, pal.off);
  std::printf("拿尺子验收：朝工件推进 100 mm → Z 应增大 ~100；向右移 → X 增大；向上抬 → Y 减小。\n");
  std::printf("只在任一轴与%s上次打印%s相差 > %.1f mm / %.1f deg 时打印一行；静默 %lld 秒会插一行心跳，"
              "报出「其间最大偏离」——静默是在算，不是卡住\n",
              pal.bold, pal.off, args.deadband_t, args.deadband_r,
              static_cast<long long>(kHeartbeatMs / 1000));
  std::printf("%s%s%s\n", pal.dim,
              interactive ? "z 或 Enter = 重新调零，q 或 s = 停止并打印精度小结。"
                          : "（stdin 不是终端：调零键不可用，跑满 --frames 后退出。）",
              pal.off);
  std::printf("\n");
  std::fflush(stdout);
}

int run(const Args& args) {
  std::string err;
  FollowConfig cfg;
  if (!load_config(args.config_path, &cfg, &err)) {
    LOG_AT(LogLevel::kError, "cfg") << err;
    return kExitConfig;
  }
  set_log_level(args.debug ? LogLevel::kDebug : LogLevel::kInfo);

  const std::string root = find_project_root();
  const ConfigProblems probs = check_config(&cfg, root);
  if (!probs.ok()) {
    LOG_AT(LogLevel::kError, "cfg") << "配置有 " << probs.fatals() << " 条致命问题:\n"
                                    << probs.joined();
    return kExitConfig;
  }
  for (const auto& p : probs.items) {
    if (!p.fatal) {
      LOG_AT(LogLevel::kWarn, "cfg") << p.text;
    }
  }

  OrbbecCapture cap;
  if (!cap.open(cfg.capture, &err)) {
    const bool busy = err.find("锁被") != std::string::npos;
    LOG_AT(LogLevel::kError, busy ? "lock" : "dev") << err;
    return busy ? kExitLockBusy : kExitDevice;
  }
  TrackParams tp = cfg.track;
  tp.k = cap.calib().color;  // 设备自报、按实际启用分辨率取
  LOG_AT(LogLevel::kInfo, "dev")
      << "已开流 " << cfg.capture.width << "x" << cfg.capture.height << "@" << cfg.capture.fps
      << "  fx=" << tp.k.fx << " fy=" << tp.k.fy << " cx=" << tp.k.cx << " cy=" << tp.k.cy
      << "  对齐=" << to_string(cap.align());

  std::string f_err;
  auto fe = make_frontend(cfg.frontend_kind, cfg.frontend, &f_err);
  if (!fe) {
    LOG_AT(LogLevel::kError, "frontend") << f_err;
    cap.close();
    return kExitConfig;
  }

  auto map = std::make_unique<ReferenceMap>();
  if (!teach_reference(cap, tp, "", map.get(), &err, cfg.teach_frames)) {  // 使用与服务相同的示教帧数
    LOG_AT(LogLevel::kError, "teach") << "示教失败: " << err;
    cap.close();
    return kExitNoMap;
  }
  auto tracker = std::make_unique<Tracker>(tp, *map, 0x5EEDu);

  KeyPoller keys;
  const Palette pal;
  PrintGate gate(args.deadband_t, args.deadband_r);
  PoseSmoother smoother(args.smooth);
  bool banner_done = false;

  Segment seg;
  Eigen::Isometry3d prev_T = Eigen::Isometry3d::Identity();
  bool have_prev = false;
  int64_t total = 0;
  const int64_t max_frames = args.frames;
  while (g_signal.load() == 0 && (max_frames == 0 || total < max_frames)) {
    RgbdFrame fr;
    if (!cap.wait_frame(&fr, &err)) {
      const CaptureHealth ch = cap.health();
      if (!ch.device_present) {
        LOG_AT(LogLevel::kError, "dev") << "设备离线: " << err;
        seg.report("设备离线");
        cap.close();
        return kExitDevice;
      }
      LOG_AT(LogLevel::kWarn, "dev") << "取帧失败: " << err;
      continue;
    }
    if (!banner_done) {
      banner_done = true;
      print_banner(pal, cfg, cap, fr, cap.health(), keys.interactive(), args);
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

    const Pose p = make_pose(r.T_ref_cam);
    std::array<double, 3> sig_t {{std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN()}};
    std::array<double, 3> sig_r = sig_t;
    for (int i = 0; i < 3; ++i) {
      sig_t[i] = r.unc.trans_sigma_mm[i];
      sig_r[i] = r.unc.rot_sigma_deg[i];
    }
    const Eigen::Isometry3d& T = r.T_ref_cam;
    const double jump_t = have_prev ? (T.translation() - prev_T.translation()).norm() * 1000.0 : 0.0;
    const double jump_r = have_prev ? rot_step_deg(prev_T, T) : 0.0;
    prev_T = T;
    have_prev = true;
    ++total;
    const double st_max = std::max(sig_t[0], std::max(sig_t[1], sig_t[2]));
    const double sr_max = std::max(sig_r[0], std::max(sig_r[1], sig_r[2]));
    const std::string state_key =
        std::string(to_string(r.status)) + "/" + to_string(r.estimator);
    seg.note(p, sig_t, sig_r, jump_t, jump_r, ms, state_key);

    smoother.push(T);
    const Pose disp_p = (smoother.size() > 1) ? make_pose(smoother.value()) : p;

    // **统计一帧都不落**（使用原始单帧），显示与死区闸门使用平滑后的位姿：
    // 静止单帧噪声约 1~2 mm，通过 5 帧滑动平均将噪声压到 0.5 mm 以下，
    // 从而使 1.0 mm 死区闸门能够有效拦截静止噪点，避免刷屏。
    const PrintGate::Why why = gate.check(disp_p, state_key, steady_now_ms());
    if (why != PrintGate::Why::kSkip) {
      // 直接写 stdout 而不走 logger：这是给人边移动边读的数，时间戳前缀会把一列数值挤断。
      print_pose_line(pal, gate, total, disp_p, r, st_max, sr_max, ms);
    }

    const int key = keys.poll();
    if (key == 'q' || key == 's' || key == 3) {
      break;
    }
    if (key == '\n' || key == 'z') {
      seg.report("重新调零");
      std::string te;
      auto fresh = std::make_unique<ReferenceMap>();
      if (!teach_reference(cap, tp, "", fresh.get(), &te, cfg.teach_frames)) {  // 与首次示教使用同一帧数
        LOG_AT(LogLevel::kError, "teach") << "重新调零失败: " << te << "（沿用旧基准）";
      } else {
        map = std::move(fresh);
        tracker = std::make_unique<Tracker>(tp, *map, 0x5EEDu);  // 跟丢计数属于旧基准
        seg = Segment();
        gate = PrintGate(args.deadband_t, args.deadband_r);  // 新基准的第一帧必须出声
        smoother = PoseSmoother(args.smooth);
        have_prev = false;  // 跨零点的"单帧跳变"是两套坐标相减，不是运动
        LOG_AT(LogLevel::kInfo, "teach") << "已重新调零 体素=" << map->info().map_voxels
                                        << " hash=" << std::hex << map->info().content_hash << std::dec;
      }
    } else if (key != 0) {
      std::printf("（未识别的按键 %d）z/Enter=重新调零  q/s=停止\n", key);
    }
  }

  if (g_signal.load() != 0) {
    std::printf("\n收到信号 %d，收尾。\n", g_signal.load());
  }
  seg.report(args.frames > 0 ? "--frames 跑满" : "退出");
  const CaptureHealth end = cap.health();
  std::printf("\n全程 %lld 帧，设备周期 %.2f ms（最差 %.2f ms）≈ %.1f fps，dropout=%lld\n",
              static_cast<long long>(total), end.period_ms, end.max_period_ms,
              end.period_ms > 0.0 ? 1000.0 / end.period_ms : 0.0,
              static_cast<long long>(end.dropouts));
  std::printf("其中 %lld 帧因变化未超过 %.1f mm / %.1f deg 没有单独打印——**统计用的是全部 %lld 帧**，"
              "阈值只筛显示，不筛样本。\n",
              static_cast<long long>(gate.suppressed()), args.deadband_t, args.deadband_r,
              static_cast<long long>(total));
  std::printf("想复核尺度：调零后把相机朝工件推进恰好 100 mm，看 Z 的读数差；左右/上下同理看 X/Y。\n\n");
  keys.restore();
  tracker.reset();
  fe.reset();
  cap.close();
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
