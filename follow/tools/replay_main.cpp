// P3 回放端 = 验收门本身。读一份合成序列，按清单里的参数跑真实产品代码（depth→cloud、
// 冻结参考地图、CPU 特征前端、GICP/稀疏两路求解、可观测门），把估计和**构造真值**逐帧对比。
//
// 三条不可商量的规矩：
//  1) 参数一律来自清单，不在这里重复默认值 —— 两端不一致时量出来的"精度"错得很像成功；
//  2) 每一帧的状态必须符合清单里那条 expect：出包络不许报成跟丢，退化不许报成正常；
//  3) 第 0 帧（示教帧）只查状态不计误差 —— 参考地图就是从它建的，拿它算残差等于自己改
//     自己的作业。噪声是逐帧独立播种的，所以"多次实现"这件事已经在帧间发生了。
//
// 退出码：全部门通过 = 0，任何一条不过 = 1（可以直接挂到 CI / 板端脚本上）。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "follow/cloud.hpp"
#include "follow/frontend.hpp"
#include "follow/odometry.hpp"
#include "follow/reference_map.hpp"
#include "follow/types.hpp"
#include "sequence_io.hpp"
#include "synth_scene.hpp"

namespace follow {
namespace synth {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGateTransP95Mm = 2.0;   // 计划里定的门：喷涂公差毫米级
constexpr double kGateRotP95Deg = 0.2;

struct Stats {
  std::vector<double> v;
  void add(double x) { v.push_back(x); }
  size_t n() const { return v.size(); }
  double mean() const {
    double s = 0.0;
    for (double x : v) s += x;
    return v.empty() ? 0.0 : s / static_cast<double>(v.size());
  }
  double sd() const {
    if (v.size() < 2) return 0.0;
    const double m = mean();
    double s = 0.0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / static_cast<double>(v.size() - 1));
  }
  double pctl(double q) const {
    if (v.empty()) return 0.0;
    std::vector<double> s = v;
    std::sort(s.begin(), s.end());
    return s[static_cast<size_t>(q * static_cast<double>(s.size() - 1) + 0.5)];
  }
  // 带符号序列直接取 p95 得到的是"+2σ 的位置"，不是误差大小 —— 报告里要的是后者。
  double pctl_abs(double q) const {
    Stats a;
    a.v.reserve(v.size());
    for (double x : v) {
      a.v.push_back(std::abs(x));
    }
    return a.pctl(q);
  }
};

// 按轴拆开转角误差 = log(truthᵀ·got) 的（轴×角）。不用 eulerAngles：它给的是某个欧拉序列
// 下的三元组，单位阵附近常常分解成 (π,-π,π)，报告会被自己的约定骗。
Eigen::Vector3d rot_err_vec_deg(const Eigen::Isometry3d& got, const Eigen::Isometry3d& truth) {
  const Eigen::AngleAxisd aa(truth.linear().transpose() * got.linear());
  return aa.angle() * aa.axis() * 180.0 / kPi;
}

double rot_norm_deg(const Eigen::Isometry3d& got, const Eigen::Isometry3d& truth) {
  const Eigen::Matrix3d R = truth.linear().transpose() * got.linear();
  const double c = std::min(1.0, std::max(-1.0, (R.trace() - 1.0) * 0.5));
  return std::acos(c) * 180.0 / kPi;
}

std::string describe(const TrackResult& r) {
  auto join3 = [](const double* a) {
    return "[" + std::to_string(a[0]) + ", " + std::to_string(a[1]) + ", " + std::to_string(a[2]) +
           "]";
  };
  char tail[128];
  std::snprintf(tail, sizeof(tail), " s2=%.2e aniso=%.1f/%.1f ratio=%.2f in=%d/%zu",
                r.unc.residual_var_scale, r.unc.trans_anisotropy(), r.unc.rot_anisotropy(),
                r.inlier_ratio, r.gicp_inliers, r.cloud_points);
  return std::string(to_string(r.status)) + " est=" + to_string(r.estimator) +
         " sig_t=" + join3(r.unc.trans_sigma_mm) + " sig_r=" + join3(r.unc.rot_sigma_deg) + tail;
}

struct Report {
  std::string name;
  int frames = 0;
  int scored = 0;
  std::map<std::string, int> hist;
  Stats dt_norm, dr_norm;
  Stats dt[3], dr[3];
  Stats sig_t[3], sig_r[3];
  Stats ms_feat, ms_gicp, ms_sparse, ms_none;
  std::vector<std::string> violations;
};

bool run_case(const std::string& dir, uint32_t tracker_seed, bool verbose, Report* rep,
              std::string* err) {
  SequenceSpec s;
  if (!load_sequence(dir, &s, err)) {
    return false;
  }
  rep->name = s.name;
  rep->frames = static_cast<int>(s.frames.size());

  FrontendParams fp;
  std::string fe_err;
  std::unique_ptr<FeatureFrontend> fe = make_frontend("cpu", fp, &fe_err);
  if (!fe) {
    *err = "特征前端创建失败：" + fe_err;
    return false;
  }

  // ---- 示教：把第 0 帧的几何冻进参考地图 ----
  cv::Mat teach_depth, teach_color;
  if (!read_frame(dir, s.frames.front(), &teach_depth, &teach_color, err)) {
    return false;
  }
  if (teach_depth.cols != s.k.width || teach_depth.rows != s.k.height) {
    *err = "深度图尺寸 " + std::to_string(teach_depth.cols) + "x" + std::to_string(teach_depth.rows) +
           " 与清单内参 " + std::to_string(s.k.width) + "x" + std::to_string(s.k.height) +
           " 不一致：uv 会整体错位，测出来的数字没有意义";
    return false;
  }
  Status cs = Status::kOk;
  const std::vector<Eigen::Vector3f> teach_cloud =
      depth_to_cloud(teach_depth, s.k, s.zmin_m, s.zmax_m, s.depth_stride, nullptr, &cs);
  if (cs != Status::kOk || teach_cloud.size() < 200) {
    *err = "示教帧点云不足（" + std::to_string(teach_cloud.size()) + " 点, " + to_string(cs) + "）";
    return false;
  }
  TeachFrame tf;
  tf.cam_pts = teach_cloud;
  tf.T_ref_cam = s.frames.front().T_ref_cam;  // 参考系 = 示教帧相机系 ⇒ 单位变换
  ReferenceMap map;
  if (!map.build_from_frames({tf}, s.voxel_m, 1'000'000, err)) {
    return false;
  }

  TrackParams p;
  p.k = s.k;
  p.zmin_m = s.zmin_m;
  p.zmax_m = s.zmax_m;
  p.depth_stride = s.depth_stride;
  p.voxel_m = s.voxel_m;
  p.max_corr_m = s.max_corr_m;
  p.threads = 4;
  Tracker tracker(p, map, tracker_seed);

  for (const SequenceFrame& f : s.frames) {
    cv::Mat depth, color;
    if (!read_frame(dir, f, &depth, &color, err)) {
      return false;
    }
    if (depth.size() != teach_depth.size()) {
      *err = "第 " + std::to_string(f.index) + " 帧尺寸和示教帧不一致";
      return false;
    }

    FeatureFrame ff;
    ff.ts_ns = f.ts_ns;
    if (!color.empty()) {
      const auto t0 = std::chrono::steady_clock::now();
      ff = fe->extract(color, f.ts_ns);
      const auto t1 = std::chrono::steady_clock::now();
      rep->ms_feat.add(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }

    const auto t0 = std::chrono::steady_clock::now();
    const TrackResult r = tracker.track(ff, depth, f.ts_ns);
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (r.estimator == Estimator::kGicp) {
      rep->ms_gicp.add(ms);
    } else if (r.estimator == Estimator::kSparse) {
      rep->ms_sparse.add(ms);
    } else {
      rep->ms_none.add(ms);
    }

    rep->hist[to_string(r.status)]++;
    if (!expect_accepts(f.expect, r.status)) {
      rep->violations.push_back("  帧 " + std::to_string(f.index) + " [" + f.tag + "] expect=" +
                                to_string(f.expect) + " 实际=" + describe(r));
    }
    const Eigen::Vector3d dt = (r.T_ref_cam.translation() - f.T_ref_cam.translation()) * 1000.0;
    const Eigen::Vector3d dr = rot_err_vec_deg(r.T_ref_cam, f.T_ref_cam);

    // 示教帧不计误差：地图就是从它建的，残差天然接近零。
    const bool usable_for_stats = f.index > 0 && r.status == Status::kOk && f.expect == Expect::kOk;
    if (usable_for_stats) {
      ++rep->scored;
      rep->dt_norm.add(dt.norm());
      rep->dr_norm.add(rot_norm_deg(r.T_ref_cam, f.T_ref_cam));
      for (int a = 0; a < 3; ++a) {
        rep->dt[a].add(dt[a]);
        rep->dr[a].add(dr[a]);
        rep->sig_t[a].add(r.unc.trans_sigma_mm[a]);
        rep->sig_r[a].add(r.unc.rot_sigma_deg[a]);
      }
    }
    if (verbose) {
      std::printf("    [%2d %-9s] %-46s |dt|=%7.3f |dR|=%7.4f %6.1f ms\n"
                  "         e_t=[%+7.3f %+7.3f %+7.3f] mm  e_r=[%+7.3f %+7.3f %+7.3f] deg\n"
                  "         est=[%+8.4f %+8.4f %+8.4f]  tru=[%+8.4f %+8.4f %+8.4f] m\n",
                  f.index, f.tag.c_str(), describe(r).c_str(), dt.norm(), dr.norm(), ms, dt.x(),
                  dt.y(), dt.z(), dr.x(), dr.y(), dr.z(), r.T_ref_cam.translation().x(),
                  r.T_ref_cam.translation().y(), r.T_ref_cam.translation().z(),
                  f.T_ref_cam.translation().x(), f.T_ref_cam.translation().y(),
                  f.T_ref_cam.translation().z());
    }
  }
  return true;
}

const char* axis_name(int a) {
  static const char* n[6] = {"tx", "ty", "tz", "rx", "ry", "rz"};
  return n[a];
}

void print_hist(const char* label, const Stats& s) {
  if (s.n() == 0) {
    std::printf("   %s 无样本\n", label);
    return;
  }
  std::printf("   %s n=%zu p50=%7.2f p95=%7.2f max=%7.2f\n", label, s.n(), s.pctl(0.5),
              s.pctl(0.95), s.pctl(1.0));
}

void print_timing_hist(const Report& rep) {
  struct { const char* name; const Stats* s; } rows[] = {
      {"gicp", &rep.ms_gicp}, {"sparse", &rep.ms_sparse}, {"无解", &rep.ms_none}};
  static const double edges[] = {10.0, 20.0, 33.0, 50.0, 66.0, 100.0, 200.0};
  std::printf(
      "   耗时 ms（RK3588 有 DVFS，绝对值只看量级）  桶: <10 10-20 20-33 33-50 50-66 66-100 "
      "100-200 >=200\n");
  for (const auto& row : rows) {
    if (row.s->n() == 0) {
      continue;
    }
    int bucket[8] = {0};
    for (double x : row.s->v) {
      int b = 0;
      while (b < 7 && x >= edges[b]) {
        ++b;
      }
      ++bucket[b];
    }
    std::string line;
    for (int b = 0; b < 8; ++b) {
      line += " " + std::to_string(bucket[b]);
    }
    std::printf("     %-7s n=%zu p50=%6.1f p95=%6.1f max=%6.1f:%s\n", row.name, row.s->n(),
                row.s->pctl(0.5), row.s->pctl(0.95), row.s->pctl(1.0), line.c_str());
  }
}

// 返回 false = 这个用例没过门。
bool print_report(const Report& rep, bool gate) {
  std::printf("\n== %-12s %2d 帧  计分 %d\n", rep.name.c_str(), rep.frames, rep.scored);
  std::printf("   状态:");
  for (const auto& kv : rep.hist) {
    std::printf(" %s=%d", kv.first.c_str(), kv.second);
  }
  std::printf("\n");
  if (rep.scored > 0) {
    std::printf("   轴        bias      sd    p95|e|   σ自报   σ/sd\n");
    for (int a = 0; a < 3; ++a) {
      const double sd = rep.dt[a].sd();
      const double sg = rep.sig_t[a].mean();
      std::printf("   %-6s %+9.3f %8.3f %8.3f %8.3f %6.2f\n", axis_name(a), rep.dt[a].mean(), sd,
                  rep.dt[a].pctl_abs(0.95), sg, sd > 1e-9 ? sg / sd : 0.0);
    }
    for (int a = 0; a < 3; ++a) {
      const double sd = rep.dr[a].sd();
      const double sg = rep.sig_r[a].mean();
      std::printf("   %-6s %+9.4f %8.4f %8.4f %8.4f %6.2f\n", axis_name(a + 3), rep.dr[a].mean(), sd,
                  rep.dr[a].pctl_abs(0.95), sg, sd > 1e-9 ? sg / sd : 0.0);
    }
    std::printf("   |dt| p95 = %.3f mm   |dR| p95 = %.4f deg\n", rep.dt_norm.pctl(0.95),
                rep.dr_norm.pctl(0.95));
  }
  print_timing_hist(rep);
  print_hist("   特征前端", rep.ms_feat);
  for (const auto& v : rep.violations) {
    std::printf("   违规!\n%s\n", v.c_str());
  }
  if (!gate) {
    std::printf("   结果: 不计分（无 kOk 计分帧，本用例考的是状态不是精度）\n");
    return rep.violations.empty();
  }
  const bool t_ok = rep.dt_norm.pctl(0.95) < kGateTransP95Mm;
  const bool r_ok = rep.dr_norm.pctl(0.95) < kGateRotP95Deg;
  const bool pass = rep.violations.empty() && t_ok && r_ok;
  std::printf("   结果: %s  (状态违规 %zu 条, |dt|p95 %s, |dR|p95 %s)\n", pass ? "PASS" : "FAIL",
              rep.violations.size(), t_ok ? "达标" : "超标", r_ok ? "达标" : "超标");
  return pass;
}

int replay_main(int argc, char** argv) {
  std::string root = "follow/out/synth";
  std::vector<std::string> dirs;
  uint32_t seed = 0x5EEDu;
  bool verbose = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](std::string* dst) {
      if (i + 1 >= argc) {
        return false;
      }
      *dst = argv[++i];
      return true;
    };
    if (a == "--root") {
      if (!next(&root)) return 2;
    } else if (a == "--seed") {
      std::string v;
      if (!next(&v)) return 2;
      seed = static_cast<uint32_t>(std::stoul(v));
    } else if (a == "--verbose" || a == "-v") {
      verbose = true;
    } else if (a == "--help" || a == "-h") {
      std::printf("用法: follow_replay [--root 目录] [序列目录...] [--seed N] [-v]\n");
      return 0;
    } else {
      dirs.push_back(a);
    }
  }
  if (dirs.empty()) {
    std::error_code ec;
    if (!std::filesystem::is_directory(root, ec)) {
      std::fprintf(stderr, "找不到目录 %s（先跑 gen_synth --out %s）\n", root.c_str(), root.c_str());
      return 2;
    }
    for (const auto& e : std::filesystem::directory_iterator(root, ec)) {
      if (e.is_directory() && std::filesystem::exists(e.path() / "sequence.yaml")) {
        dirs.push_back(e.path().string());
      }
    }
    std::sort(dirs.begin(), dirs.end());
  }
  if (dirs.empty()) {
    std::fprintf(stderr, "%s 下没有含 sequence.yaml 的子目录\n", root.c_str());
    return 2;
  }

  std::printf("follow_replay: %zu 份序列, tracker 种子 0x%X, 门 |dt|p95<%.1f mm |dR|p95<%.2f deg\n",
              dirs.size(), seed, kGateTransP95Mm, kGateRotP95Deg);
  int failed = 0;
  int scored_frames = 0;
  int total_frames = 0;
  for (const auto& d : dirs) {
    Report rep;
    std::string err;
    if (!run_case(d, seed, verbose, &rep, &err)) {
      std::fprintf(stderr, "  %-40s 失败: %s\n", d.c_str(), err.c_str());
      ++failed;
      continue;
    }
    total_frames += rep.frames;
    scored_frames += rep.scored;
    if (!print_report(rep, rep.scored > 0)) {
      ++failed;
    }
  }
  std::printf("\n总计 %d 帧 / 计分 %d 帧, %d 个用例未过门\n", total_frames, scored_frames, failed);
  return failed == 0 ? 0 : 1;
}

}  // namespace
}  // namespace synth
}  // namespace follow

int main(int argc, char** argv) {
  return follow::synth::replay_main(argc, argv);
}
