// P2 门：稠密配准必须回到构造真值；几何退化必须说实话（不能报 kOk）；参考地图必须
// 真的"冻得住"（存/读/损坏都能认出来）。
//
// 场景是解析的（射线 vs 原始体），所以"当前帧"与"真值位姿"之间不存在估计误差 ——
// 误差全部来自配准本身。这也是 P3 渲染器的雏形。
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "follow/cloud.hpp"
#include "follow/odometry.hpp"
#include "follow/reference_map.hpp"
#include "follow/types.hpp"

namespace follow {
namespace testing_detail {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kVoxelM = 0.03;

CameraIntrinsics rigK() {
  CameraIntrinsics k;
  k.fx = 611.683837890625;
  k.fy = 611.6983642578125;
  k.cx = 643.4285278320312;
  k.cy = 405.1534118652344;
  k.width = 1280;
  k.height = 800;
  return k;
}

Eigen::Isometry3d make_T(double tx_mm, double ty_mm, double tz_mm, double rx_deg, double ry_deg,
                         double rz_deg) {
  // 两个 AngleAxis 相乘得到的是 Quaternion，必须显式落回矩阵
  const Eigen::Matrix3d R = (Eigen::AngleAxisd(rz_deg * kPi / 180.0, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(ry_deg * kPi / 180.0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(rx_deg * kPi / 180.0, Eigen::Vector3d::UnitX()))
                                .toRotationMatrix();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = Eigen::Vector3d(tx_mm, ty_mm, tz_mm) * 0.001;
  return T;
}

double trans_err_mm(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return (a.translation() - b.translation()).norm() * 1000.0;
}

double rot_err_deg(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  const Eigen::Matrix3d R = a.linear().transpose() * b.linear();
  const double c = std::min(1.0, std::max(-1.0, (R.trace() - 1.0) * 0.5));
  return std::acos(c) * 180.0 / kPi;
}

// 按轴拆开转角误差 = log(truthᵀ·got) 的 (轴×角)。不要用 eulerAngles：它给的是某个欧拉
// 序列下的三元组，单位阵附近常常分解成 (π, -π, π)，MC 里第一版因此报出 "bias=179.99 deg"。
Eigen::Vector3d rot_err_vec_deg(const Eigen::Isometry3d& got, const Eigen::Isometry3d& truth) {
  const Eigen::AngleAxisd aa(truth.linear().transpose() * got.linear());
  return aa.angle() * aa.axis() * 180.0 / kPi;
}

std::string pose_msg(const char* what, const Eigen::Isometry3d& got, const Eigen::Isometry3d& truth) {
  return std::string(what) + ": |dt|=" + std::to_string(trans_err_mm(got, truth)) + " mm |dR|=" +
         std::to_string(rot_err_deg(got, truth)) + " deg";
}

// ---- 参考系里的解析场景。返回沿射线的最近命中参数（0 = 未命中），射线 o + t·d ----

double hit_plane(const Eigen::Vector3d& o, const Eigen::Vector3d& d, const Eigen::Vector3d& n,
                 const Eigen::Vector3d& p0, const Eigen::AlignedBox3d& win) {
  const double dn = n.dot(d);
  if (std::abs(dn) < 1e-9) {
    return 0.0;
  }
  const double t = n.dot(p0 - o) / dn;
  if (t <= 0.0) {
    return 0.0;
  }
  const Eigen::Vector3d p = o + t * d;
  return win.contains(p) ? t : 0.0;
}

// 单一大平面：喷涂工位里最常见的退化源。
double hit_wall(const Eigen::Vector3d& o, const Eigen::Vector3d& d) {
  const Eigen::AlignedBox3d win(Eigen::Vector3d(-2.0, -2.0, 0.5), Eigen::Vector3d(2.0, 2.0, 1.5));
  return hit_plane(o, d, Eigen::Vector3d::UnitZ(), Eigen::Vector3d(0, 0, 1.0), win);
}

// 工件：带肋的高度场平板 + 一个机加工方台 + 一个圆顶凸台。
//
// 为什么不是"一块大平面 + 几个附件"：附件只占视野 5% 时，面内平移和绕视轴的
// 倾角在 GICP 代价里几乎可以互相替换 —— 相机横移 δx 让正对相机的平面看起来
// 像被俯仰了 δx/z。实测这样的场景 x 平移 20 mm 收敛到 (-11.3, +11.5) mm + 2.3°
// 转角，而求解器自报 σ=(8, 17.5) mm —— 两个数吻合，说明**估计器在说实话，是场
// 景没信息**。拿它当精度基准只会测出自洽的错觉。所以基准几何必须每个方向都有
// 真实的深度梯度（肋 = 连续梯度，方台 = 深度断崖，圆顶 = 曲率）。
//
// 高度场没有遮挡，射线求交就是定点迭代：斜率 < 0.22 ⇒ 三次迭代收敛到亚微米。
double surface_z(double x, double y) {
  double z = 0.90 + 0.005 * std::sin(2.0 * kPi * x / 0.15) + 0.004 * std::sin(2.0 * kPi * y / 0.11);
  if (x > 0.18 && x < 0.34 && y > -0.16 && y < -0.04) {
    z = 0.845;  // 方台：边缘是一条 55 mm 的深度台阶
  }
  const double dx = x + 0.20;
  const double dy = y - 0.12;
  const double r2 = dx * dx + dy * dy;
  if (r2 < 0.10 * 0.10) {
    z = std::min(z, 0.80 + 0.06 * r2 / (0.10 * 0.10));  // 圆顶，顶点朝相机
  }
  return z;
}

double hit_workpiece(const Eigen::Vector3d& o, const Eigen::Vector3d& d) {
  if (std::abs(d.z()) < 1e-9) {
    return 0.0;
  }
  const Eigen::AlignedBox3d win(Eigen::Vector3d(-0.45, -0.28, 0.5), Eigen::Vector3d(0.45, 0.28, 1.5));
  double t = (0.90 - o.z()) / d.z();
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d p = o + t * d;
    const double next = (surface_z(p.x(), p.y()) - o.z()) / d.z();
    if (std::abs(next - t) < 1e-9) {
      t = next;
      break;
    }
    t = next;
  }
  if (t <= 0.0) {
    return 0.0;
  }
  return win.contains(o + t * d) ? t : 0.0;
}

// 入参是**相机在参考系里的位姿** T_cam_ref，和 TrackResult::T_ref_cam 同一约定：参考系里
// 的场景是静止的，所以把相机系射线按 T_cam_ref 搬到参考系求交，命中点再按 T_ref_cam 搬回
// 相机系取 z。约定写反的话，测出来的"回收误差"会变成两倍的 commanded motion（本文件
// 一开始就踩过：GICP 完全收敛，但收敛到 R(+2θ) 上，看起来像求解器偏了 2.3 mm/°）。
// block>1 时一次算一个 block×block 的常数块：追踪器按 depth_stride 采样，全分辨率
// 逐像素求交在这里只是白烧 CPU（P3 的渲染器要出彩色图，那时才需要逐像素）。
// noise_mm：加在**深度**上的高斯噪声的**标准差**（不是加在 3D 点上），这就是真实深度传感器
// 的误差模型。它同时也是 GICP 的输入合法性要求：完全无噪声的合成人造平面，其局部协方差是
// 严格秩亏的，而 GICPFactor 用普通 .inverse()，结果直接是 NaN —— 完美平面落在求解器的
// 有效输入域之外，这点在实机上不会遇到（深度永远有噪声），但在测试里必须显式加。
// 踩过的坑：normal_distribution 的签名是 (mean, stddev)，写成 (sigma, 0.0) 会得到一个**恒定
// 偏置**而不是噪声；而地图和测试帧用同一个默认参数渲染，恒定偏置是共模的，配准时完全抵消
// —— 单次跑看不出任何异常，只有蒙特卡洛换种子后 sd 严格为 0 才暴露。所以噪声必须和"sd 非零"
// 一起被断言，见 NoiseSdMatchesPredictedSigma。
cv::Mat render_depth(const CameraIntrinsics& k, const Eigen::Isometry3d& T_cam_ref,
                     double (*hit)(const Eigen::Vector3d&, const Eigen::Vector3d&), int block = 4,
                     double noise_mm = 0.5, uint32_t seed = 20260830u) {
  cv::Mat img = cv::Mat::zeros(k.height, k.width, CV_16UC1);
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, noise_mm * 0.001);
  const Eigen::Isometry3d T_ref_cam = T_cam_ref.inverse();
  const Eigen::Vector3d origin = T_cam_ref.translation();
  const Eigen::Matrix3d R_cam_ref = T_cam_ref.linear();
  const int b = std::max(1, block);
  for (int v0 = 0; v0 < k.height; v0 += b) {
    for (int u0 = 0; u0 < k.width; u0 += b) {
      const Eigen::Vector3d d_cam((u0 + 0.5 - k.cx) / k.fx, (v0 + 0.5 - k.cy) / k.fy, 1.0);
      const double t = hit(origin, R_cam_ref * d_cam);
      if (t <= 0.0) {
        continue;
      }
      const Eigen::Vector3d p_cam = T_ref_cam * (origin + t * R_cam_ref * d_cam);
      const double z_mm = (p_cam.z() + noise(rng)) * 1000.0;
      if (!(z_mm > 0.0) || z_mm > 65535.0) {
        continue;
      }
      const ushort value = static_cast<ushort>(std::lround(z_mm));
      const int v1 = std::min(k.height, v0 + b);
      const int u1 = std::min(k.width, u0 + b);
      for (int v = v0; v < v1; ++v) {
        ushort* row = img.ptr<ushort>(v);
        for (int u = u0; u < u1; ++u) {
          row[u] = value;
        }
      }
    }
  }
  return img;
}

std::vector<Eigen::Vector3f> cloud_of(const cv::Mat& depth_mm, const CameraIntrinsics& k, int stride) {
  return depth_to_cloud(depth_mm, k, 0.30, 2.50, stride);
}

TrackParams rigParams(const CameraIntrinsics& k) {
  TrackParams p;
  p.k = k;
  p.depth_stride = 4;
  p.voxel_m = kVoxelM;
  p.threads = 4;
  return p;
}

// 示教 = 把参考位姿那一帧的几何冻进地图。
struct Rig {
  CameraIntrinsics k = rigK();
  cv::Mat ref_depth;
  std::vector<Eigen::Vector3f> ref_cloud;
  ReferenceMap map;
  TrackParams p;
  std::string err;
  bool ok = false;

  explicit Rig(bool plane = false) {
    ref_depth = render_depth(k, Eigen::Isometry3d::Identity(), plane ? hit_wall : hit_workpiece);
    ref_cloud = cloud_of(ref_depth, k, 4);
    TeachFrame f;
    f.cam_pts = ref_cloud;
    f.T_ref_cam = Eigen::Isometry3d::Identity();
    // 构造函数里不能用 ASSERT_*（它要 return）。构建失败会让 map 为空，
    // 于是每条用例自己的 status 断言就是那条更清楚的失败信息。
    ok = map.build_from_frames({f}, kVoxelM, 1'000'000'000, &err);
    EXPECT_TRUE(ok) << err;
    p = rigParams(k);
  }
};

// 失败时要看的数。缺了它们，kLost 和 kDegenerate 长得一模一样，而"看着很准的退化"和
// "真退化"也只能靠 s² 与各向异性比值分开 —— 绝对 σ 单独摆出来会误导（见 LargePlane...）。
std::string describe(const TrackResult& r) {
  auto join3 = [](const double* v) {
    return "[" + std::to_string(v[0]) + ", " + std::to_string(v[1]) + ", " + std::to_string(v[2]) + "]";
  };
  char tail[160];
  std::snprintf(tail, sizeof(tail), " s2=%.2e aniso=%.1f/%.1f rank=%d cost_per_in=%.3f",
                r.unc.residual_var_scale, r.unc.trans_anisotropy(), r.unc.rot_anisotropy(),
                r.unc.rank_deficient ? 1 : 0,
                r.gicp_inliers > 0 ? r.gicp_cost / static_cast<double>(r.gicp_inliers) : 0.0);
  return std::string(to_string(r.status)) + " est=" + to_string(r.estimator) +
         " pts=" + std::to_string(r.cloud_points) + " gicp_in=" + std::to_string(r.gicp_inliers) +
         " sp_in=" + std::to_string(r.sparse_inliers) + " ratio=" + std::to_string(r.inlier_ratio) +
         " iters=" + std::to_string(r.iterations) + " conv=" + (r.converged ? "1" : "0") +
         " gyro=" + (r.gyro_used ? "1" : "0") + " sig_t_mm=" + join3(r.unc.trans_sigma_mm) +
         " sig_r_deg=" + join3(r.unc.rot_sigma_deg) + tail +
         (r.unc.solver_failed ? " SOLVER_FAILED" : "");
}

std::string tmp_path(const char* name) {
  return "/tmp/follow_test_" + std::string(name);
}

double vec_mean(const std::vector<double>& v) {
  double s = 0.0;
  for (double x : v) {
    s += x;
  }
  return v.empty() ? 0.0 : s / static_cast<double>(v.size());
}

double vec_sd(const std::vector<double>& v) {
  if (v.size() < 2) {
    return 0.0;
  }
  const double m = vec_mean(v);
  double s = 0.0;
  for (double x : v) {
    s += (x - m) * (x - m);
  }
  return std::sqrt(s / static_cast<double>(v.size() - 1));
}

double vec_pctl(std::vector<double> v, double q) {
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  return v[static_cast<size_t>(q * static_cast<double>(v.size() - 1) + 0.5)];
}

// 一个自由度上的三行对照：真实散布 vs 求解器声称的散布 vs 两者的系统偏置。
struct AxisStats {
  double bias = 0.0;
  double sd = 0.0;
  double sigma = 0.0;
};

struct MonteCarlo {
  int draws = 0;
  int not_ok = 0;
  double mean_s2 = 0.0;
  double mean_aniso_t = 0.0;
  double mean_aniso_r = 0.0;
  double p95_trans_mm = 0.0;
  double p95_rot_deg = 0.0;
  AxisStats trans[3];
  AxisStats rot[3];
  std::string last_status;
};

// 同一个指令位姿，换 kDraws 个独立的深度噪声实现各跑一次（地图始终冻结不动，Tracker
// 每帧冷启动）。σ 声称的是"一次测量的散布"，所以直接量散布和它并排看 —— 单次跑给不出
// 这个对照。种子序列固定，因此整段可复现。
MonteCarlo monte_carlo(const Rig& rig, const Eigen::Isometry3d& truth,
                       double (*hit)(const Eigen::Vector3d&, const Eigen::Vector3d&), int draws,
                       uint32_t first_seed = 1000u) {
  std::vector<Eigen::Vector3d> et, er;
  std::vector<double> dt_norm, dr_norm, s2;
  double sig_t[3] = {0, 0, 0};
  double sig_r[3] = {0, 0, 0};
  double aniso_t = 0.0, aniso_r = 0.0;
  MonteCarlo mc;
  for (int i = 0; i < draws; ++i) {
    const cv::Mat frame =
        render_depth(rig.k, truth, hit, 4, 0.5, first_seed + static_cast<uint32_t>(i) * 7919u);
    Tracker t(rig.p, rig.map);
    const TrackResult r = t.track(FeatureFrame{}, frame, 1);
    mc.last_status = to_string(r.status);
    if (r.status != Status::kOk) {
      ++mc.not_ok;
    }
    et.push_back((r.T_ref_cam.translation() - truth.translation()) * 1000.0);
    er.push_back(rot_err_vec_deg(r.T_ref_cam, truth));
    dt_norm.push_back(trans_err_mm(r.T_ref_cam, truth));
    dr_norm.push_back(rot_err_deg(r.T_ref_cam, truth));
    s2.push_back(r.unc.residual_var_scale);
    aniso_t += r.unc.trans_anisotropy();
    aniso_r += r.unc.rot_anisotropy();
    for (int a = 0; a < 3; ++a) {
      sig_t[a] += r.unc.trans_sigma_mm[a];
      sig_r[a] += r.unc.rot_sigma_deg[a];
    }
  }
  std::vector<double> col_t[3], col_r[3];
  for (const auto& v : et) {
    for (int a = 0; a < 3; ++a) {
      col_t[a].push_back(v[a]);
    }
  }
  for (const auto& v : er) {
    for (int a = 0; a < 3; ++a) {
      col_r[a].push_back(v[a]);
    }
  }
  mc.draws = draws;
  mc.mean_s2 = vec_mean(s2);
  mc.mean_aniso_t = aniso_t / static_cast<double>(draws);
  mc.mean_aniso_r = aniso_r / static_cast<double>(draws);
  mc.p95_trans_mm = vec_pctl(dt_norm, 0.95);
  mc.p95_rot_deg = vec_pctl(dr_norm, 0.95);
  for (int a = 0; a < 3; ++a) {
    mc.trans[a] = {vec_mean(col_t[a]), vec_sd(col_t[a]), sig_t[a] / static_cast<double>(draws)};
    mc.rot[a] = {vec_mean(col_r[a]), vec_sd(col_r[a]), sig_r[a] / static_cast<double>(draws)};
  }
  return mc;
}

std::string axis_msg(const AxisStats& s) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), "bias=%8.4f sd=%8.4f sigma=%8.4f", s.bias, s.sd, s.sigma);
  return buf;
}

}  // namespace

// ------------------------------------------------ 稠密配准

TEST(Registration, SelfRegistrationReturnsIdentityWithinTenthMillimetre) {
  Rig rig;
  ASSERT_GE(rig.ref_cloud.size(), 2000u) << "场景太小，测不出什么";
  Tracker t(rig.p, rig.map);

  const TrackResult r = t.track(FeatureFrame{}, rig.ref_depth, 1'000'000'000);
  ASSERT_EQ(r.status, Status::kOk) << describe(r);
  EXPECT_EQ(r.estimator, Estimator::kGicp);
  const Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  EXPECT_LT(trans_err_mm(r.T_ref_cam, truth), 0.1) << pose_msg("self", r.T_ref_cam, truth);
  EXPECT_LT(rot_err_deg(r.T_ref_cam, truth), 0.02) << pose_msg("self", r.T_ref_cam, truth);
  EXPECT_GT(r.inlier_ratio, 0.9) << r.inlier_ratio;
  EXPECT_TRUE(r.converged);
  EXPECT_FALSE(r.unc.solver_failed);
  // 全自由度都该是被测量过的，否则下面几条"退化"测试没有对照。
  EXPECT_TRUE(r.unc.within(rig.p.max_trans_sigma_mm, rig.p.max_rot_sigma_deg,
                           rig.p.max_group_anisotropy)) << describe(r);
}

TEST(Registration, RecoversKnownRigidMotionFromColdSeed) {
  Rig rig;
  const Eigen::Isometry3d truth = make_T(10.0, -20.0, 30.0, 0.6, -0.9, 1.2);
  const cv::Mat moved = render_depth(rig.k, truth, hit_workpiece);

  Tracker t(rig.p, rig.map);
  // 冷启动：没有上一帧、没有特征，初值就是单位阵 —— 求解器必须自己走完这 30 mm。
  const TrackResult r = t.track(FeatureFrame{}, moved, 1'000'000'000);
  ASSERT_EQ(r.status, Status::kOk) << describe(r);
  // 单次噪声实现只断言"行为对"（能走完整段运动、落在方案门限内）；"多准"归
  // NoiseSdMatchesPredictedSigma 那条蒙特卡洛管，它跑 16 个种子而不是赌一个。
  EXPECT_LT(trans_err_mm(r.T_ref_cam, truth), 2.0) << pose_msg("cold", r.T_ref_cam, truth);
  EXPECT_LT(rot_err_deg(r.T_ref_cam, truth), 0.2) << pose_msg("cold", r.T_ref_cam, truth);
}

// 指令 1/5/10/20 mm 的台阶，回收出来的位移必须**跟着台阶走**。
// 这条测的不是精度，是"那个方向上到底有没有信息"：曾经有一版场景（大平面+几个附件）
// x 指令 20 mm 只回收 8 mm，误差范数看着像精度问题，实际是该维度根本没被测量。
TEST(Registration, ScanPatternOfTranslationsIsTracked) {
  Rig rig;
  std::vector<double> got_x;
  for (double mm : {1.0, 5.0, 10.0, 20.0}) {
    const Eigen::Isometry3d truth = make_T(mm, 0.0, 0.0, 0.0, 0.0, 0.0);
    const cv::Mat frame = render_depth(rig.k, truth, hit_workpiece);
    Tracker t(rig.p, rig.map);
    const TrackResult r = t.track(FeatureFrame{}, frame, 1'000'000'000);
    ASSERT_EQ(r.status, Status::kOk) << mm << " mm: " << describe(r);
    got_x.push_back(r.T_ref_cam.translation().x() * 1000.0);
    // 1 mm 的台阶要分辨得出来：偏置明显小于台阶本身。
    EXPECT_NEAR(got_x.back(), mm, 1.0) << mm << " mm " << pose_msg("x", r.T_ref_cam, truth);
  }
  for (size_t i = 1; i < got_x.size(); ++i) {
    const double step = got_x[i] - got_x[i - 1];
    const double want = (i == 1) ? 4.0 : (i == 2 ? 5.0 : 10.0);
    // 台阶门必须比单点门（±1.0）宽：单点各自合格时，差值最差能差 2.0，写成 0.5 会被自己
    // 绊倒（实测 0.556 mm）。这一条要挡的是"信息塌了"——x 只回收 19% 时最后一步只有
    // 2 mm，1.5 mm 的门离它还有一个数量级。亚毫米精度归 NoiseSdMatchesPredictedSigma。
    EXPECT_NEAR(step, want, 1.5) << "step " << i << " recovered " << step << " mm";
  }
}

// 单一大平面：GICP 会"收敛"，但面内两条平移和绕法线的转角根本没被测量。
// 报 kOk 就等于告诉控制器"这一维我量到了"，那是假证据。
TEST(Registration, LargePlaneReportsDegenerateNotOk) {
  Rig rig(/*plane=*/true);
  ASSERT_GE(rig.ref_cloud.size(), 2000u);

  const Eigen::Isometry3d truth = make_T(0.0, 0.0, 20.0, 0.5, 0.0, 0.0);
  const cv::Mat frame = render_depth(rig.k, truth, hit_wall);
  Tracker t(rig.p, rig.map);
  const TrackResult r = t.track(FeatureFrame{}, frame, 1'000'000'000);
  EXPECT_EQ(r.status, Status::kDegenerate) << describe(r);
  EXPECT_EQ(r.estimator, Estimator::kGicp) << "几何对上了，退化不该把估计器换掉";
  EXPECT_TRUE(r.converged) << "求解器自己觉得收敛了 —— 这正是只看 converged 会被骗到的地方";
  EXPECT_GT(r.inlier_ratio, 0.9) << r.inlier_ratio;
  // 分开"量到了"和"没量到"的是**组内比值**。绝对 σ 在这个场景里反而是帮倒忙的：不可观
  // 方向上残差恒等于零 ⇒ s² 塌到 6e-6（带肋工件 5e-5），乘完之后面内 σ_t=[0.063 0.063] mm
  // 比好场景的 0.15 mm 还小，而真实误差是 10 mm。下面那条 EXPECT_LT 就是这个陷阱的复现：
  // 只有比值门（15）能挡住它，比值不含尺度，s² 骗不到。见 NoiseSdMatchesPredictedSigma。
  EXPECT_GT(r.unc.trans_anisotropy(), rig.p.max_group_anisotropy) << describe(r);
  EXPECT_GT(r.unc.rot_anisotropy(), rig.p.max_group_anisotropy) << describe(r);
  EXPECT_LT(r.unc.trans_sigma_mm[0], rig.p.max_trans_sigma_mm) << "绝对门放过了面内 x —— 已知陷阱";
  EXPECT_LT(r.unc.trans_sigma_mm[2], r.unc.trans_sigma_mm[0]) << "沿光轴的平移必须比面内松紧分明";
  // 状态非 ok 但仍采用本帧解：五个方向的实测不该为一个没量到的维度陪葬。
  EXPECT_LT(std::abs(r.T_ref_cam.translation().z() - 0.020), 0.002)
      << pose_msg("wall", r.T_ref_cam, truth);
}

// 出包络 ≠ 跟丢：前者要重新示教，后者会自己恢复。混在一起的代价是操作员等一个
// 永远不会来的恢复。
TEST(Registration, OutOfEnvelopeIsNotLost) {
  Rig rig;
  // 相机被移开 500 mm（沿 -z 后退）：几何仍然满视野、仍然在量程内，因此不是 kNoDepth；
  // 但它与冻结基准的间距远超对应点门，因此也不是"跟丢后该重新找"。
  const Eigen::Isometry3d far_away = make_T(0.0, 0.0, -500.0, 0.0, 0.0, 0.0);
  const cv::Mat frame = render_depth(rig.k, far_away, hit_workpiece);

  Tracker t(rig.p, rig.map);
  const TrackResult r = t.track(FeatureFrame{}, frame, 1'000'000'000);
  EXPECT_EQ(r.status, Status::kOutOfEnvelope) << describe(r);
  EXPECT_LT(r.inlier_ratio, rig.p.min_inlier_ratio) << r.inlier_ratio;
  EXPECT_TRUE(r.T_ref_cam.matrix().isApprox(Eigen::Isometry3d::Identity().matrix(), 1e-15));
}

TEST(Registration, NoDepthAndBadConfigAreToldApart) {
  Rig rig;
  const cv::Mat black = cv::Mat::zeros(rig.k.height, rig.k.width, CV_16UC1);
  {
    Tracker t(rig.p, rig.map);
    const TrackResult r = t.track(FeatureFrame{}, black, 1'000'000'000);
    EXPECT_EQ(r.status, Status::kNoDepth) << describe(r);
  }
  {
    TrackParams bad = rig.p;
    bad.k.fx = 0.0;  // 除零会产生 inf 点云，必须在最前面就报出来
    Tracker t(bad, rig.map);
    EXPECT_EQ(t.track(FeatureFrame{}, rig.ref_depth, 1).status, Status::kConfigInvalid);
  }
  {
    ReferenceMap empty;
    Tracker t(rig.p, empty);
    EXPECT_EQ(t.track(FeatureFrame{}, rig.ref_depth, 1).status, Status::kConfigInvalid);
  }
}

// 故障时保持上一目标：这条是 P5 控制器的安全垫，必须在追踪器这一层就成立。
TEST(Registration, HoldsLastAdoptedPoseThroughFaults) {
  Rig rig;
  const Eigen::Isometry3d truth = make_T(0.0, 0.0, 0.0, 0.0, 0.0, 0.8);
  const cv::Mat frame = render_depth(rig.k, truth, hit_workpiece);
  const cv::Mat black = cv::Mat::zeros(rig.k.height, rig.k.width, CV_16UC1);

  Tracker t(rig.p, rig.map);
  const TrackResult ok = t.track(FeatureFrame{}, frame, 1'000'000'000);
  ASSERT_EQ(ok.status, Status::kOk) << describe(ok);

  for (int64_t ts = 2; ts <= 4; ++ts) {
    const TrackResult bad = t.track(FeatureFrame{}, black, ts * 1'000'000'000);
    EXPECT_EQ(bad.status, Status::kNoDepth) << describe(bad);
    EXPECT_TRUE(bad.T_ref_cam.matrix().isApprox(ok.T_ref_cam.matrix(), 1e-15))
        << "盲帧把位姿动了：" << pose_msg("hold", bad.T_ref_cam, ok.T_ref_cam);
    EXPECT_TRUE(t.T().matrix().isApprox(ok.T_ref_cam.matrix(), 1e-15));
  }

  // 恢复：从保持住的位置继续，不该有跳变（旧版忘了重置帧间速度，这里会突然多推一段）。
  const TrackResult back = t.track(FeatureFrame{}, frame, 5'000'000'000);
  EXPECT_EQ(back.status, Status::kOk) << describe(back);
  EXPECT_LT(trans_err_mm(back.T_ref_cam, ok.T_ref_cam), 0.5)
      << pose_msg("recovered", back.T_ref_cam, ok.T_ref_cam);
}

TEST(Registration, ReversedTimestampIsStaleInput) {
  Rig rig;
  Tracker t(rig.p, rig.map);
  EXPECT_EQ(t.track(FeatureFrame{}, rig.ref_depth, 5'000'000'000).status, Status::kOk);
  const TrackResult r = t.track(FeatureFrame{}, rig.ref_depth, 4'000'000'000);
  EXPECT_EQ(r.status, Status::kStaleInput) << to_string(r.status);
  // 乱序帧不许污染参考帧时间戳，否则下一帧的间隔就错了。
  const TrackResult next = t.track(FeatureFrame{}, rig.ref_depth, 6'000'000'000);
  EXPECT_EQ(next.status, Status::kOk) << describe(next);
}

// 换种子做蒙特卡洛。σ 声称的是"一次测量的散布"，那就直接量散布，和求解器自报的 σ 并排看。
// 三个用途，每一个都是单次跑给不出来的：
// (a) 证明噪声真的进了管线 —— sd 严格为 0 就说明"噪声"退化成了共模偏置（本文件真踩过，
//     见 render_depth 的注释：地图和测试帧共用同一个噪声实现时，配准会把它整个吸掉）；
// (b) 判 σ 偏乐观还是偏悲观 —— 可观测门的阈值因此落在测量值上而不是猜；
// (c) 分轴 bias。系统性偏置不会被 σ 覆盖 —— 它是另一类错误，1 mm 的偏置就是 1 mm 的
//     喷涂误差，不会因为"散布很小"而变得可以接受。
TEST(Registration, NoiseSdMatchesPredictedSigma) {
  constexpr int kDraws = 16;
  Rig rig;  // 带肋工件 + 方台 + 圆顶：每个方向都有真实深度梯度
  ASSERT_GE(rig.ref_cloud.size(), 2000u) << "场景太小，测不出散布";

  struct Cmd {
    const char* name;
    double tx, ty, tz, rz;
  };
  // mix 一次激励全部六个自由度；yaw1 单独压最弱的那个转角方向。
  for (const Cmd& c : {Cmd{"mix", 10, -20, 30, 0.0}, Cmd{"yaw1", 0, 0, 0, 1.0}}) {
    const Eigen::Isometry3d truth = make_T(c.tx, c.ty, c.tz, 0.0, 0.0, c.rz);
    const MonteCarlo mc = monte_carlo(rig, truth, hit_workpiece, kDraws);
    const std::string tag = std::string(c.name) + " ";
    EXPECT_EQ(mc.not_ok, 0) << tag << "好场景不该有任何一帧被判退化：" << mc.last_status;
    // 方案门：平移 p95 < 2 mm、旋转 p95 < 0.2°。
    EXPECT_LT(mc.p95_trans_mm, 2.0) << tag << "p95 |dt| = " << mc.p95_trans_mm << " mm";
    EXPECT_LT(mc.p95_rot_deg, 0.2) << tag << "p95 |dR| = " << mc.p95_rot_deg << " deg";
    for (int a = 0; a < 3; ++a) {
      const std::string mt = tag + "t" + std::to_string(a) + " " + axis_msg(mc.trans[a]);
      const std::string mr = tag + "r" + std::to_string(a) + " " + axis_msg(mc.rot[a]);
      // (a) 噪声确实在管线里。sd==0 意味着它又退化成了和地图共模的常量偏置。
      EXPECT_GT(mc.trans[a].sd, 1e-3) << mt << " —— 噪声没生效？";
      EXPECT_GT(mc.rot[a].sd, 1e-4) << mr << " —— 噪声没生效？";
      // (b) σ 不许比真实散布乐观（系统低估自己的错误是这类估计器最危险的失效模式），
      //     也不许悲观到门限失去意义。16 个样本算出的 sd 自身就有 ±18% 的统计误差，
      //     所以两侧分别留 2x / 4x —— 这条卡的是"量级对不对"，不是小数点。
      EXPECT_LT(mc.trans[a].sigma, mc.trans[a].sd * 4.0) << mt << " —— σ 过度悲观";
      EXPECT_GT(mc.trans[a].sigma, mc.trans[a].sd * 0.5) << mt << " —— σ 过度乐观";
      EXPECT_LT(mc.rot[a].sigma, mc.rot[a].sd * 4.0) << mr << " —— σ 过度悲观";
      EXPECT_GT(mc.rot[a].sigma, mc.rot[a].sd * 0.5) << mr << " —— σ 过度乐观";
      // (c) 偏置是另一类错误，散布再小也不掩盖它。
      EXPECT_LT(std::abs(mc.trans[a].bias), 1.0) << mt;
      EXPECT_LT(std::abs(mc.rot[a].bias), 0.1) << mr;
    }
    // 可观测门的阈值就是这些数定的（见 odometry.hpp），所以它们得留在日志里，不能只活在注释里。
    std::printf("[mc good %s] s2=%.2e p95 |dt|=%.3f mm |dR|=%.4f deg aniso_t=%.1f aniso_r=%.1f\n"
                "   t sd=(%.4f %.4f %.4f) sigma=(%.4f %.4f %.4f) mm\n"
                "   r sd=(%.5f %.5f %.5f) sigma=(%.5f %.5f %.5f) deg\n",
                c.name, mc.mean_s2, mc.p95_trans_mm, mc.p95_rot_deg, mc.mean_aniso_t,
                mc.mean_aniso_r, mc.trans[0].sd, mc.trans[1].sd,
                mc.trans[2].sd, mc.trans[0].sigma, mc.trans[1].sigma, mc.trans[2].sigma,
                mc.rot[0].sd, mc.rot[1].sd, mc.rot[2].sd, mc.rot[0].sigma, mc.rot[1].sigma,
                mc.rot[2].sigma);
  }
}

// 这一条是各向异性门存在的全部理由。大平面上，只看绝对 σ 会放行一个真实误差 10 mm 的解：
// 不可观方向上残差恒等于零 ⇒ 那里的 s² 塌得比好场景还狠（实测 6e-6 对 5e-5）⇒ 乘完之后
// 面内 σ_t=[0.063 0.063] mm 比好场景的 0.15 mm 还"准"。比值不含尺度，s² 骗不到它。
TEST(Registration, AbsoluteSigmaAloneWouldPassThePlane) {
  constexpr int kDraws = 16;
  Rig rig(/*plane=*/true);
  const Eigen::Isometry3d truth = make_T(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const MonteCarlo mc = monte_carlo(rig, truth, hit_wall, kDraws);
  // 面内 x 命令 10 mm，这一维在平面上没有任何约束：解停在初值上，误差就是命令本身。
  EXPECT_GT(std::abs(mc.trans[0].bias), 5.0) << "平面量到了面内平移？场景变了 " << axis_msg(mc.trans[0]);
  EXPECT_EQ(mc.not_ok, kDraws) << "每一帧都必须报非 kOk：" << mc.not_ok << "/" << kDraws << " " << mc.last_status;
  // 自报的 1σ 比真实误差小两个数量级 —— "σ 很小"这句话本身就是错的。
  EXPECT_LT(mc.trans[0].sigma, std::abs(mc.trans[0].bias) * 0.01)
      << "陷阱没复现：s² 不再塌了？" << axis_msg(mc.trans[0]);
  EXPECT_LT(mc.mean_s2, 2e-5) << "s² = " << mc.mean_s2 << "，好场景的量级是 5e-5";

  Tracker t(rig.p, rig.map);
  const cv::Mat frame = render_depth(rig.k, truth, hit_wall, 4, 0.5, 1000u);
  const TrackResult r = t.track(FeatureFrame{}, frame, 1);
  const std::string msg = describe(r) + " " + axis_msg(mc.trans[0]);
  EXPECT_LT(r.unc.trans_sigma_mm[0], rig.p.max_trans_sigma_mm) << msg << " —— 绝对平移门看着是过的";
  EXPECT_LT(r.unc.rot_sigma_deg[0], rig.p.max_rot_sigma_deg) << msg << " —— 绝对转角门看着是过的";
  EXPECT_FALSE(r.unc.within(rig.p.max_trans_sigma_mm, rig.p.max_rot_sigma_deg,
                            rig.p.max_group_anisotropy))
      << msg << " —— 组合门必须判它不可观";
  EXPECT_GT(r.unc.trans_anisotropy(), rig.p.max_group_anisotropy) << msg;
  // 挡它的必须是比值门，而不是特征值下限顺手报的 rank 亏：平面仍然有非零信息（体素有展宽），
  // λ_min 远没掉到 1e-9·λ_max。这条若翻红，uncertainty.hpp / odometry.hpp 里"只有比值门
  // 挡得住"那句就得改 —— 那句话现在是从这里来的，不是从推理来的。
  EXPECT_FALSE(r.unc.rank_deficient) << msg;
  std::printf("[mc wall x10] s2=%.2e aniso_t=%.1f aniso_r=%.1f  真实 x 误差 bias=%.2f mm sd=%.3f"
              " 而自报 sigma=%.4f mm\n",
              mc.mean_s2, mc.mean_aniso_t, mc.mean_aniso_r, mc.trans[0].bias, mc.trans[0].sd,
              mc.trans[0].sigma);
}

// ------------------------------------------------ 参考地图：冻得住、认得出

TEST(ReferenceMap, BuildAppliesTeachPoseAndDropsNonFinitePoints) {
  const CameraIntrinsics k = rigK();
  std::vector<Eigen::Vector3f> pts = cloud_of(render_depth(k, Eigen::Isometry3d::Identity(), hit_workpiece), k, 4);
  ASSERT_FALSE(pts.empty());
  pts.push_back(Eigen::Vector3f(NAN, 0.f, 1.f));
  pts.push_back(Eigen::Vector3f(0.f, 0.f, std::numeric_limits<float>::infinity()));

  const Eigen::Isometry3d T = make_T(100.0, 200.0, 300.0, 0.0, 0.0, 0.0);
  TeachFrame f;
  f.cam_pts = pts;
  f.T_ref_cam = T;
  ReferenceMap map;
  std::string err;
  ASSERT_TRUE(map.build_from_frames({f}, kVoxelM, 42, &err)) << err;
  EXPECT_EQ(map.info().raw_points, pts.size() - 2) << "非有限点必须进不了基准";
  ASSERT_GT(map.points().size(), 0u);
  ASSERT_EQ(map.scans().size(), 1u);
  ASSERT_EQ(map.scans()[0].size(), pts.size() - 2);
  // 检查变换是否真的作用到点上（降采样后的 points() 会错位，这里要看未降采样的那份）
  EXPECT_NEAR(double(map.scans()[0][0].x()), double(pts[0].x()) + 0.1, 1e-5)
      << "示教位姿没作用到点云上，地图系就是错的";
  EXPECT_NEAR(double(map.scans()[0][0].z()), double(pts[0].z()) + 0.3, 1e-5);
  EXPECT_EQ(map.info().built_ts_ns, 42);
}

TEST(ReferenceMap, BuildRejectsUnusableInput) {
  ReferenceMap map;
  std::string err;
  EXPECT_FALSE(map.build({}, kVoxelM, 0, &err)) << "空示教必须失败：" << err;
  EXPECT_TRUE(map.empty());
  EXPECT_FALSE(map.build(std::vector<std::vector<Eigen::Vector3f>>(1, std::vector<Eigen::Vector3f>(10)),
                         kVoxelM, 0, &err))
      << err;
  std::vector<Eigen::Vector3f> junk(500, Eigen::Vector3f(0.1f, 0.2f, 0.9f));
  EXPECT_FALSE(map.build({junk}, -1.0, 0, &err)) << "负体素尺寸必须被拒：" << err;
  EXPECT_FALSE(map.build({junk}, std::numeric_limits<double>::quiet_NaN(), 0, &err)) << err;
  EXPECT_TRUE(map.empty());
}

TEST(ReferenceMap, SaveLoadRoundTripKeepsTheSameBaseline) {
  Rig rig;
  const std::string path = tmp_path("roundtrip.map");
  std::string err;
  ASSERT_TRUE(rig.map.save(path, &err)) << err;

  ReferenceMap loaded;
  ASSERT_TRUE(loaded.load(path, &err)) << err;
  EXPECT_EQ(loaded.info().content_hash, rig.map.info().content_hash);
  EXPECT_EQ(loaded.info().map_voxels, rig.map.info().map_voxels);
  EXPECT_EQ(loaded.info().raw_points, rig.map.info().raw_points);
  EXPECT_DOUBLE_EQ(loaded.info().voxel_m, rig.map.info().voxel_m);
  EXPECT_EQ(loaded.info().built_ts_ns, rig.map.info().built_ts_ns);

  // 认得出同一份基准，还要用得起：换到读回来的地图上，同一帧必须给同一个位姿。
  Tracker a(rig.p, rig.map);
  Tracker b(rig.p, loaded);
  const TrackResult ra = a.track(FeatureFrame{}, rig.ref_depth, 1);
  const TrackResult rb = b.track(FeatureFrame{}, rig.ref_depth, 1);
  ASSERT_EQ(ra.status, Status::kOk) << describe(ra);
  ASSERT_EQ(rb.status, Status::kOk) << describe(rb);
  EXPECT_LT(trans_err_mm(ra.T_ref_cam, rb.T_ref_cam), 0.01) << pose_msg("save/load", rb.T_ref_cam, ra.T_ref_cam);
  EXPECT_LT(rot_err_deg(ra.T_ref_cam, rb.T_ref_cam), 0.005) << pose_msg("save/load", rb.T_ref_cam, ra.T_ref_cam);
  std::remove(path.c_str());
}

TEST(ReferenceMap, LoadRejectsCorruptionInsteadOfGuessing) {
  Rig rig;
  const std::string path = tmp_path("corrupt.map");
  std::string err;
  ASSERT_TRUE(rig.map.save(path, &err)) << err;

  auto write_file = [&](const std::vector<char>& buf) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(buf.data(), static_cast<std::streamsize>(buf.size()));
  };
  std::vector<char> good;
  {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    const size_t n = static_cast<size_t>(f.tellg());
    f.seekg(0);
    good.resize(n);
    f.read(good.data(), static_cast<std::streamsize>(n));
  }
  ASSERT_GT(good.size(), 1000u);
  auto rejects = [&](const std::vector<char>& buf, const char* want) {
    write_file(buf);
    ReferenceMap m;
    std::string why;
    EXPECT_FALSE(m.load(path, &why)) << "这份文件被接受了：" << want;
    EXPECT_TRUE(m.empty()) << "拒绝之后还留着半份地图";
    EXPECT_NE(why.find(want), std::string::npos) << "报的是另一件事: " << why;
  };

  {  // 整份文件不是这个格式
    std::vector<char> buf;
    const std::string junk = "this is not a reference map at all, not even close";
    buf.assign(junk.begin(), junk.end());
    rejects(buf, "magic");
  }
  {  // 截断
    std::vector<char> buf = good;
    buf.resize(buf.size() / 2);
    rejects(buf, "越界");
  }
  {  // 版本不匹配：重新示教，不是猜兼容
    std::vector<char> buf = good;
    buf[4] = static_cast<char>(buf[4] + 1);
    rejects(buf, "版本");
  }
  {  // 字节序/内容错位：magic 认不出来
    std::vector<char> buf = good;
    std::swap(buf[0], buf[3]);
    rejects(buf, "magic");
  }
  {  // 点数据被改了一个 bit：基准不能被人悄悄换掉
    std::vector<char> buf = good;
    buf[buf.size() - 20] = static_cast<char>(buf[buf.size() - 20] ^ 0x08);
    rejects(buf, "哈希");
  }
  {  // 尾部多出一截
    std::vector<char> buf = good;
    buf.insert(buf.end(), 8, '\0');
    rejects(buf, "哈希字段位置不对");
  }
  std::remove(path.c_str());
}

TEST(ReferenceMap, RebuildIsReproducible) {
  const CameraIntrinsics k = rigK();
  const std::vector<Eigen::Vector3f> pts =
      cloud_of(render_depth(k, Eigen::Isometry3d::Identity(), hit_workpiece), k, 4);
  ReferenceMap a, b;
  std::string err;
  ASSERT_TRUE(a.build({pts}, kVoxelM, 7, &err)) << err;
  ASSERT_TRUE(b.build({pts}, kVoxelM, 7, &err)) << err;
  EXPECT_EQ(a.info().content_hash, b.info().content_hash);
  EXPECT_EQ(a.info().map_voxels, b.info().map_voxels);
  ASSERT_EQ(a.points().size(), b.points().size());
  for (size_t i = 0; i < a.points().size(); ++i) {
    ASSERT_TRUE(a.points().point(i).isApprox(b.points().point(i), 0.0)) << i;
  }
}

TEST(ReferenceMap, VoxelSizeIsPartOfTheIdentity) {
  // 同一份点、不同体素尺寸 = 不同的求解精度。信息字段要如实分开，别让它看起来像同一份基准。
  const CameraIntrinsics k = rigK();
  const std::vector<Eigen::Vector3f> pts =
      cloud_of(render_depth(k, Eigen::Isometry3d::Identity(), hit_workpiece), k, 4);
  ReferenceMap fine, coarse;
  std::string err;
  ASSERT_TRUE(fine.build({pts}, 0.02, 7, &err)) << err;
  ASSERT_TRUE(coarse.build({pts}, 0.05, 7, &err)) << err;
  EXPECT_EQ(fine.info().content_hash, coarse.info().content_hash) << "哈希只该覆盖点数据";
  EXPECT_NE(fine.info().map_voxels, coarse.info().map_voxels);
}

}  // namespace testing_detail
}  // namespace follow
