// P1 数学核心的单元测试。这些不是覆盖率数字，每条都对应评审里一个具体缺陷。
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "follow/cloud.hpp"
#include "follow/frontend.hpp"
#include "follow/gyro_filter.hpp"
#include "follow/matching.hpp"
#include "follow/pose_io.hpp"
#include "follow/types.hpp"
#include "follow/uncertainty.hpp"

namespace follow {
namespace testing_detail {

// 测试夹具用的真实内参：来自 data/template_group/2026-08-25_215601/scan.params.yaml
CameraIntrinsics realK() {
  CameraIntrinsics k;
  k.fx = 611.683837890625;
  k.fy = 611.6983642578125;
  k.cx = 643.4285278320312;
  k.cy = 405.1534118652344;
  k.width = 1280;
  k.height = 800;
  return k;
}

inline cv::Point2f project(const CameraIntrinsics& k, const Eigen::Vector3d& p) {
  return {static_cast<float>(k.fx * p.x() / p.z() + k.cx),
          static_cast<float>(k.fy * p.y() / p.z() + k.cy)};
}

inline void stamp(cv::Mat& depth_mm, const cv::Point2f& uv, double z_m) {
  const int u = static_cast<int>(std::lround(uv.x));
  const int v = static_cast<int>(std::lround(uv.y));
  if (u < 0 || v < 0 || u >= depth_mm.cols || v >= depth_mm.rows) return;
  depth_mm.at<uint16_t>(v, u) = static_cast<uint16_t>(z_m * 1000.0 + 0.5);
}

// N 个互不相同、归一化的描述子；同一份矩阵当两帧用可保证 i↔i 唯一匹配。
inline cv::Mat idDescriptors(int n, unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<float> g(0.f, 1.f);
  cv::Mat d(n, 32, CV_32F);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < 32; ++j) d.at<float>(i, j) = g(rng);
  }
  normalize_descriptors(d);
  return d;
}

}  // namespace testing_detail
}  // namespace follow

using namespace follow;
using namespace follow::testing_detail;

// ---------------------------------------------------------------- 内参与点云

TEST(Cloud, RejectsZeroIntrinsicsInsteadOfEmittingNaN) {
  CameraIntrinsics bad;  // 全 0：原实现在这里会产出 inf/NaN 并且无人拦截
  cv::Mat depth(60, 80, CV_16UC1, cv::Scalar(1000));
  CloudStats stats;
  Status st = Status::kOk;
  auto pts = depth_to_cloud(depth, bad, 0.3, 2.5, 2, &stats, &st);
  EXPECT_TRUE(pts.empty());
  EXPECT_EQ(st, Status::kConfigInvalid);
}

TEST(Cloud, RejectsWrongDepthMatType) {
  cv::Mat as_float(60, 80, CV_32FC1, cv::Scalar(1.0f));
  Status st = Status::kOk;
  auto pts = depth_to_cloud(as_float, realK(), 0.3, 2.5, 2, nullptr, &st);
  EXPECT_TRUE(pts.empty());
  EXPECT_EQ(st, Status::kNoDepth);
}

TEST(Cloud, MatchesAnalyticPinhole) {
  const CameraIntrinsics k = realK();
  cv::Mat depth(k.height, k.width, CV_16UC1, cv::Scalar(1000));  // 整幅 1 m 平面
  CloudStats stats;
  Status st = Status::kOk;
  auto pts = depth_to_cloud(depth, k, 0.3, 2.5, 4, &stats, &st);
  EXPECT_EQ(st, Status::kOk);
  EXPECT_EQ(stats.kept, static_cast<int>(pts.size()));
  ASSERT_FALSE(pts.empty());

  const Eigen::Vector3d expect_center = unproject(k, k.cx, k.cy, 1.0);
  EXPECT_NEAR(expect_center.x(), 0.0, 1e-9);
  EXPECT_NEAR(expect_center.y(), 0.0, 1e-9);
  EXPECT_NEAR(expect_center.z(), 1.0, 1e-9);

  // 针孔公式逐个坐标精确比对。取 (644,404) 是因为它在 stride=4 的采样格点上
  // （主点 643.43,405.15 本身不在格点上，按「中心附近」找会假失败）。
  const double gu = 644, gv = 404, gz = 1.0;
  const Eigen::Vector3d want((gu - k.cx) * gz / k.fx, (gv - k.cy) * gz / k.fy, gz);
  bool found = false;
  for (const auto& p : pts) {
    if ((p.cast<double>() - want).norm() < 1e-6) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "格点 (644,404) 的反投影与解析针孔模型不一致";
}

TEST(Cloud, DepthAtRejectsOutOfRangeAndNaN) {
  const CameraIntrinsics k = realK();
  cv::Mat depth(10, 10, CV_16UC1, cv::Scalar(0));
  depth.at<uint16_t>(5, 5) = 1000;
  EXPECT_DOUBLE_EQ(depth_at_m(depth, {5.4f, 4.6f}, k, 0.3, 2.5), 1.0);
  EXPECT_DOUBLE_EQ(depth_at_m(depth, {0, 0}, k, 0.3, 2.5), 0.0);        // 0 = 无效
  EXPECT_DOUBLE_EQ(depth_at_m(depth, {99, 99}, k, 0.3, 2.5), 0.0);      // 越界
  EXPECT_DOUBLE_EQ(depth_at_m(depth, {5, 5}, k, 1.5, 2.5), 0.0);        // 范围外
}

// ---------------------------------------------------------------- 稀疏配准

class MatchingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    k = realK();
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> xy(-0.18f, 0.18f);
    std::uniform_real_distribution<float> zz(0.85f, 1.15f);
    n = 60;
    prev3.resize(n);
    for (int i = 0; i < n; ++i) {
      prev3[i] = Eigen::Vector3d(xy(rng), xy(rng), zz(rng));
      if (prev3[i].z() < 0.3 || prev3[i].z() > 2.5) prev3[i].z() = 1.0;
    }
    T_prev_from_curr = Eigen::Isometry3d::Identity();
    T_prev_from_curr.linear() = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ())
                                    .toRotationMatrix();
    T_prev_from_curr.translation() = Eigen::Vector3d(0.010, -0.006, 0.004);
  }

  // 由 T_prev_from_curr 反出当前帧的 3D 与像素，并填两帧的深度图。
  void buildFrames(std::vector<Eigen::Vector3d>& curr3, FeatureFrame& prev, FeatureFrame& curr,
                   cv::Mat& depth_prev, cv::Mat& depth_curr) {
    const Eigen::Isometry3d T_curr_from_prev = T_prev_from_curr.inverse();
    curr3.assign(static_cast<size_t>(n), Eigen::Vector3d::Zero());
    prev.uv_px.assign(static_cast<size_t>(n), cv::Point2f());
    curr.uv_px.assign(static_cast<size_t>(n), cv::Point2f());
    depth_prev = cv::Mat(k.height, k.width, CV_16UC1, cv::Scalar(0));
    depth_curr = cv::Mat(k.height, k.width, CV_16UC1, cv::Scalar(0));
    for (int i = 0; i < n; ++i) {
      curr3[i] = T_curr_from_prev * prev3[i];
      const cv::Point2f up = project(k, prev3[i]);
      const cv::Point2f uc = project(k, curr3[i]);
      prev.uv_px[i] = up;
      curr.uv_px[i] = uc;
      stamp(depth_prev, up, prev3[i].z());
      stamp(depth_curr, uc, curr3[i].z());
    }
    cv::Mat d = idDescriptors(n, 7);
    prev.desc = d.clone();
    curr.desc = d.clone();
    prev.image_size = curr.image_size = cv::Size(k.width, k.height);
  }

  CameraIntrinsics k;
  int n = 0;
  std::vector<Eigen::Vector3d> prev3;
  Eigen::Isometry3d T_prev_from_curr;
};

TEST_F(MatchingTest, RecoversKnownRigidDelta) {
  std::vector<Eigen::Vector3d> curr3;
  FeatureFrame prev, curr;
  cv::Mat dp, dc;
  buildFrames(curr3, prev, curr, dp, dc);

  SparseDeltaParams p;
  std::mt19937 rng(123);
  const auto r = sparse_delta(prev, curr, dp, dc, k, p, rng);
  ASSERT_TRUE(r.has_value());
  EXPECT_GE(r->inliers, n - 5);

  const double t_err = (r->T_prev_from_curr.translation() - T_prev_from_curr.translation()).norm();
  const Eigen::Matrix3d Rd = r->T_prev_from_curr.linear().transpose() * T_prev_from_curr.linear();
  const double ang = std::acos(std::min(1.0, std::max(-1.0, (Rd.trace() - 1.0) / 2.0)));
  std::printf("[sparse_delta] t_err=%.4f mm  rot_err=%.4f deg  inliers=%d/%d\n", t_err * 1000.0,
              ang * 180.0 / M_PI, r->inliers, n);
  EXPECT_LT(t_err, 5e-3) << "平移误差应远小于 5 mm（亚像素反投影误差量级）";
  EXPECT_LT(ang * 180.0 / M_PI, 0.5);
}

TEST_F(MatchingTest, IsDeterministicForFixedSeed) {
  std::vector<Eigen::Vector3d> curr3;
  FeatureFrame prev, curr;
  cv::Mat dp, dc;
  buildFrames(curr3, prev, curr, dp, dc);
  SparseDeltaParams p;

  std::mt19937 r1(9), r2(9);
  const auto a = sparse_delta(prev, curr, dp, dc, k, p, r1);
  const auto b = sparse_delta(prev, curr, dp, dc, k, p, r2);
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_DOUBLE_EQ((a->T_prev_from_curr.matrix() - b->T_prev_from_curr.matrix()).norm(), 0.0);
}

TEST_F(MatchingTest, InlierGateIsCentimetreNotDecimetre) {
  std::vector<Eigen::Vector3d> curr3;
  FeatureFrame prev, curr;
  cv::Mat dp, dc;
  buildFrames(curr3, prev, curr, dp, dc);

  // 追加 20 对粗差：只挪当前帧那一个端点，配对本身就不可能被任何刚体解释。
  // （若两端同向平移 d，残差只有 |(I-R)d| ≈ 2 mm，那是合法内点，测不出门限。）
  const int m = 20;
  cv::Mat extra = idDescriptors(m, 99);
  cv::vconcat(std::vector<cv::Mat>{prev.desc, extra}, prev.desc);
  cv::vconcat(std::vector<cv::Mat>{curr.desc, extra}, curr.desc);
  for (int i = 0; i < m; ++i) {
    const Eigen::Vector3d pc = curr3[i] + Eigen::Vector3d(0.10, 0.05, -0.05);
    curr.uv_px.push_back(project(k, pc));
    prev.uv_px.push_back(project(k, prev3[i]));
    stamp(dc, curr.uv_px.back(), pc.z());
    stamp(dp, prev.uv_px.back(), prev3[i].z());
  }
  n += m;

  SparseDeltaParams good;  // inlier_dist_m = 0.02
  std::mt19937 rng(5);
  const auto r = sparse_delta(prev, curr, dp, dc, k, good, rng);
  ASSERT_TRUE(r.has_value());
  EXPECT_LE(r->inliers, n - m + 2)
      << "10 cm 粗差被当成内点了 —— 门限仍是分米级而不是厘米级";

  // 差分对照：同一批数据在旧的 0.04 m²（= 20 cm）门限下会全数漏进来。
  SparseDeltaParams bad = good;
  bad.inlier_dist_m = 0.20;
  std::mt19937 rng2(5);
  const auto r_bad = sparse_delta(prev, curr, dp, dc, k, bad, rng2);
  ASSERT_TRUE(r_bad.has_value());
  EXPECT_GT(r_bad->inliers, r->inliers + m / 2) << "旧门限没能复现出这个缺陷，测试可能测了个别的东西";
}

TEST_F(MatchingTest, RejectsWhenTooFewMatches) {
  FeatureFrame prev, curr;
  prev.desc = idDescriptors(3, 3);
  curr.desc = idDescriptors(3, 3);
  prev.uv_px.assign(3, cv::Point2f(640.f, 400.f));
  curr.uv_px.assign(3, cv::Point2f(641.f, 400.f));
  prev.image_size = curr.image_size = cv::Size(k.width, k.height);
  cv::Mat dp(40, 40, CV_16UC1, cv::Scalar(1000)), dc = dp.clone();
  SparseDeltaParams p;
  std::mt19937 rng(1);
  EXPECT_FALSE(sparse_delta(prev, curr, dp, dc, k, p, rng).has_value());
}

TEST(Matching, MutualNnRejectsDimensionMismatch) {
  cv::Mat a = idDescriptors(20, 1);           // 32 维
  cv::Mat b = idDescriptors(20, 1).colRange(0, 16).clone();  // 16 维
  EXPECT_TRUE(mutual_nn(a, b).empty());
  cv::Mat empty;
  EXPECT_TRUE(mutual_nn(empty, a).empty());
}

// ---------------------------------------------------------------- 可观测性

TEST(Uncertainty, UsesRotationFirstTangentOrdering) {
  // small_gicp 的 J 分块是 [rx ry rz tx ty tz]（registration.cpp 里
  // J.block<3,3>(0,0)=R·skew(p)、(0,3)=-R）。这条测试把这个约定钉死：
  // 只在索引 0（绕 X 的旋转）上放空方向，必须只让 rot_sigma_deg[0] 爆炸。
  //
  // 注意 σ 的绝对值有上界：伪逆把 λ 夹在 rel_eig_floor*λmax 之上，所以判据
  // 用的是「比其他自由度高几个数量级」，不是「大于某个魔数」。
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity() * 1e8;
  H(0, 0) = 0.0;
  const Uncertainty u = hessian_uncertainty(H, 1e-9);
  EXPECT_TRUE(u.rank_deficient);
  EXPECT_GT(u.rot_sigma_deg[0], 1e3 * u.rot_sigma_deg[1]);
  EXPECT_GT(u.rot_sigma_deg[0], 1e3 * u.rot_sigma_deg[2]);
  EXPECT_GT(u.rot_sigma_deg[0], 1e3 * u.trans_sigma_mm[0]);
  EXPECT_GT(u.rot_sigma_deg[0], 1e3 * u.trans_sigma_mm[2]);
  // 其余五个自由度仍然紧，说明零方向没有串扰到平移块
  for (int i = 1; i < 3; ++i) {
    EXPECT_LT(u.rot_sigma_deg[i], 0.1);
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_LT(u.trans_sigma_mm[i], 1.0);
  }
}

TEST(Uncertainty, WellConditionedHessianPassesLimits) {
  // 各向同性的单位阵只是「不退化」，不是「精度高」：H=I 的含义就是 1σ=1 rad、
  // 1m。用 1e8 的信息量才是工位跟随该有的量级（1σ = 0.1 mm / 0.006°）。
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity() * 1e8;
  const Uncertainty u = hessian_uncertainty(H);
  EXPECT_FALSE(u.rank_deficient);
  EXPECT_FALSE(u.solver_failed);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(u.trans_sigma_mm[i], 0.1, 1e-6);
    EXPECT_NEAR(u.rot_sigma_deg[i], 1e-4 * 180.0 / M_PI, 1e-6);
    EXPECT_LT(u.trans_sigma_mm[i], 1.0);
    EXPECT_LT(u.rot_sigma_deg[i], 0.1);
  }
  EXPECT_TRUE(u.within(10.0, 1.0, 15.0));
}

TEST(Uncertainty, SurvivesNonFiniteHessian) {
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Constant(NAN);
  const Uncertainty u = hessian_uncertainty(H);
  EXPECT_TRUE(u.solver_failed);
  EXPECT_FALSE(u.within(10.0, 1.0, 15.0));
  // 失败必须表现为"精度无限差"，不能停在默认的 0（= 完美的假象）
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isinf(u.rot_sigma_deg[i]));
    EXPECT_TRUE(std::isinf(u.trans_sigma_mm[i]));
  }
}

// s² 是纯尺度：σ 乘 sqrt(s²)，比值不变。蒙特卡洛实测它把 190 倍的过度悲观补回来
// （test_registration 的 NoiseSdMatchesPredictedSigma 直接对量）。
TEST(Uncertainty, ResidualVarianceScaleMultipliesSigma) {
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity() * 1e8;
  const Uncertainty raw = hessian_uncertainty(H);
  const Uncertainty scaled = hessian_uncertainty(H, 0.25);
  EXPECT_DOUBLE_EQ(scaled.residual_var_scale, 0.25);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(scaled.trans_sigma_mm[i], 0.5 * raw.trans_sigma_mm[i], 1e-9);
    EXPECT_NEAR(scaled.rot_sigma_deg[i], 0.5 * raw.rot_sigma_deg[i], 1e-9);
  }
  // 残差恒为零（重放同一帧）时 s²=0。报 σ=0 等于报"无限精确"，必须当失败处理。
  EXPECT_TRUE(hessian_uncertainty(H, 0.0).solver_failed);
  EXPECT_TRUE(hessian_uncertainty(H, -1.0).solver_failed);
  EXPECT_TRUE(hessian_uncertainty(H, std::numeric_limits<double>::quiet_NaN()).solver_failed);
}

// 被 s² 洗白的退化：大平面上不可观方向的残差恒为零，s² 塌下去之后绝对 σ 反而比好场景
// 还小（实测 [0.063 0.063 0.002] mm 而真实误差 10 mm）。绝对门对它彻底失效，只有组内
// 比值能挡住 —— 比值不含尺度，所以 s² 骗不到它。
TEST(Uncertainty, AnisotropyGateCatchesWhatAbsoluteGateMisses) {
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity() * 1e8;
  // 把 tx 方向的信息量削弱 1000 倍 ⇒ σ_tx 放大 31.6 倍，其余不动
  H(3, 3) = 1e8 / 1000.0;
  const Uncertainty u = hessian_uncertainty(H, 1e-5);
  EXPECT_LT(u.trans_sigma_mm[0], 2.0) << "绝对 σ 门被 s² 压过去了，这条就是那个场景的复现";
  EXPECT_GT(u.trans_anisotropy(), 15.0);
  EXPECT_FALSE(u.within(2.0, 0.2, 15.0));
  // 同一个各向异性放在转向上也要能挡住（大平面最常见的退化形式就是 yaw）
  Eigen::Matrix<double, 6, 6> Hr = Eigen::Matrix<double, 6, 6>::Identity() * 1e8;
  Hr(2, 2) = 1e8 / 1000.0;
  const Uncertainty ur = hessian_uncertainty(Hr, 1e-5);
  EXPECT_GT(ur.rot_anisotropy(), 15.0);
  EXPECT_FALSE(ur.within(2.0, 0.2, 15.0));
}

TEST(Extrinsics, RejectsZeroMatrixThatIsNotIdentity) {
  Eigen::Matrix3d zero = Eigen::Matrix3d::Zero();
  EXPECT_FALSE(is_valid_rotation(zero)) << "全零旋转会被当成已标定，ω 全变成 0";
  EXPECT_TRUE(is_valid_rotation(Eigen::Matrix3d::Identity()));
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  EXPECT_TRUE(is_valid_rotation(R));
  R.row(0) = R.row(1);
  EXPECT_FALSE(is_valid_rotation(R));
}

// ---------------------------------------------------------------- 陀螺

TEST(Gyro, MatchesAnalyticConstantRateIntegration) {
  std::vector<GyroSample> buf;
  const double w = 0.5;  // rad/s 绕 Z
  const int64_t dt_ns = 5'000'000;  // 200 Hz
  for (int i = 0; i <= 20; ++i) {
    GyroSample s;
    s.ts_ns = i * dt_ns;
    s.omega_cam_rad_s = Eigen::Vector3d(0, 0, w);
    buf.push_back(s);
  }
  const int64_t t0 = 0, t1 = 20 * dt_ns;  // 0.1 s
  const GyroDelta d = integrate_gyro(buf, t0, t1);
  EXPECT_TRUE(d.valid());
  EXPECT_EQ(d.samples_used, 20);
  EXPECT_NEAR(d.gap_end_ns, 0, 1);
  const Eigen::Matrix3d expect =
      Eigen::AngleAxisd(w * 0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_LT((d.R - expect).norm(), 1e-9);
}

TEST(Gyro, ReportsStaleWhenImuStoppedUpdating) {
  std::vector<GyroSample> buf;
  for (int i = 0; i < 4; ++i) {
    GyroSample s;
    s.ts_ns = i * 5'000'000;
    s.omega_cam_rad_s = Eigen::Vector3d(0.1, 0, 0);
    buf.push_back(s);
  }
  // 最后一帧图像在 660 ms 后，IMU 早就不动了：绝不能被当成"用了陀螺初值"
  const GyroDelta d = integrate_gyro(buf, 0, 660'000'000);
  EXPECT_TRUE(d.stale);
  EXPECT_FALSE(d.valid());
  EXPECT_GT(d.gap_end_ns, 500'000'000);
}

TEST(Gyro, EmptyAndOutOfWindowBuffersAreNotSilentlyUsable) {
  std::vector<GyroSample> empty;
  EXPECT_TRUE(integrate_gyro(empty, 0, 100).stale);

  std::vector<GyroSample> future;
  GyroSample s;
  s.ts_ns = 1'000'000'000;
  future.push_back(s);
  const GyroDelta d = integrate_gyro(future, 0, 100'000'000);
  EXPECT_EQ(d.samples_used, 0);
  EXPECT_TRUE(d.stale);
}

// P0-1 回归：陀螺用设备单调钟、帧用主机 epoch，两者差 6 个数量级时积分窗口一个样本都框
// 不进来，而**输出看起来完全正常** —— 三档初值的第 2 档、离群门、静止冻结全部静默停摆。
// 对齐在取流层做，本库不认墙上时钟，所以这里能测的是它的两个可测面：失效留下的唯一痕迹
// 恰好是 FollowWorker::checkGyroChannel 赖以报警的那条指纹；以及平移原点之后积分真的恢复。
TEST(Gyro, ClockDomainMismatchIsSilentButLeavesExactlyTheTripwireFingerprint) {
  // 实测量级（follow_probe_device）：336L 的 getTimeStampUs 末值 1.57e9 µs ≈ 上电 26 min；
  // 主机 system_clock epoch ≈ 1.788e18 ns。混用的代价就是这个倍数。
  constexpr int64_t kHostEpochNs = 1'788'000'000'000'000'000LL;
  constexpr int64_t kDeviceNs = 1'570'000'000'000LL;
  constexpr int64_t kFrameNs = 66'000'000;   // 15 fps
  constexpr int64_t kGyroNs = 5'000'000;     // 200 Hz
  constexpr double kW = 0.5;                 // rad/s 绕 Z

  std::vector<GyroSample> device_buf;
  for (int i = 0; i < 40; ++i) {
    device_buf.push_back(GyroSample{kDeviceNs + i * kGyroNs, Eigen::Vector3d(0, 0, kW)});
  }
  ASSERT_FALSE(device_buf.empty());

  const int64_t t0 = kHostEpochNs;
  const int64_t t1 = kHostEpochNs + kFrameNs;
  const GyroDelta dead = integrate_gyro(device_buf, t0, t1);

  // 静默：不崩、不 NaN，只是"没有样本"。这就是它在页面上和"相机没动"长得一样的原因。
  EXPECT_EQ(dead.samples_used, 0) << "两域错位本该框不到任何样本";
  EXPECT_TRUE(dead.stale);
  EXPECT_FALSE(dead.valid());
  EXPECT_EQ(dead.span_ns, 0);
  EXPECT_EQ(dead.gap_end_ns, t1 - t0) << "零覆盖时缺口应正好等于整个请求区间";
  // 指纹：缓冲非空（IMU 在跑）而窗口积不到 ⇒ 时间域不同。checkGyroChannel 判的是这一**对**，
  // 而不能只看 gap_end —— 单看它分不出"IMU 停更"和"两域错位"，两者都让 samples_used 为 0，
  // 但前者该去查取流、后者该去查时间基，处置完全不同。
  EXPECT_GT(device_buf.size(), 0u);

  // 修法：只平移原点、保留设备 dt。（取到达时间打戳会把 burst 内的 dt 压成 0，那是另一种
  // 静默少算 —— 见 gyro_time_base.hpp。）offset 让最后一条设备样本落在帧前 5 ms。
  const int64_t offset = t0 - kDeviceNs - kGyroNs;
  std::vector<GyroSample> host_buf;
  for (const GyroSample& s : device_buf) {
    host_buf.push_back(GyroSample{s.ts_ns + offset, s.omega_cam_rad_s});
  }
  const GyroDelta fixed = integrate_gyro(host_buf, t0, t1);
  EXPECT_GT(fixed.samples_used, 0) << "平移后仍积不到样本：修的就不是域问题了";
  EXPECT_TRUE(fixed.valid());
  EXPECT_LT(fixed.gap_end_ns, kGyroNs) << "IMU 其实很新，不该报大缺口";
  // 角度与已覆盖时长严格成比例 ⇒ 设备 dt 没被换算吃掉。
  const Eigen::Matrix3d expect =
      Eigen::AngleAxisd(kW * static_cast<double>(fixed.span_ns) * 1e-9, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  EXPECT_LT((fixed.R - expect).norm(), 1e-9);
}

// ---------------------------------------------------------------- 陀螺静止检测器（P1）
// 契约：判据是"减掉零偏后的残差模"；窗口没攒满前不表态；进入静止要连续确认（慢进）；
// 退出只需一个窗口均值超 exit（快出）。退路是写死的：上电头 100 ms 报静止会把真正的
// 初始运动冻掉。零偏两阶段（累积均值 → 只在安静期 EMA），见 gyro_filter.hpp 类头。

TEST(GyroStill, NoVerdictBeforeWindowIsFull) {
  GyroStillDetector d;  // window_samples = 20
  for (int i = 0; i < 19; ++i) {
    d.push(Eigen::Vector3d::Zero());
  }
  EXPECT_FALSE(d.valid());
  EXPECT_FALSE(d.still()) << "窗口没攒满时 still() 必须当无效";
  d.push(Eigen::Vector3d::Zero());
  EXPECT_TRUE(d.valid());
}

TEST(GyroStill, StillNeedsSustainedQuietAndExitsFast) {
  GyroStillDetector d;  // enter=0.008, exit=0.017, window=20, confirm=20
  const Eigen::Vector3d quiet(0.005, 0.0, 0.0);
  // 攒满窗口（20 推）后还要连续确认：第 38 推时 confirm=19，差一推不许表态。
  for (int i = 0; i < 38; ++i) {
    d.push(quiet);
  }
  ASSERT_TRUE(d.valid());
  EXPECT_FALSE(d.still()) << "确认推数不够就宣布静止 = 迟进失效";
  d.push(quiet);
  EXPECT_TRUE(d.still());
  // 恒定输入在累积均值下残差≈0：这正是"零偏不当运动"的第一半，见下面的专门用例。
  EXPECT_LT(d.recent_resid_rad_s(), GyroStillDetector::Params{}.enter_rad_s);
  EXPECT_NEAR(d.bias_rad_s(), 0.005, 1e-9);

  // 迟滞：enter < 均值 < exit 的抖动不解冻（实测相机放稳时的陀螺抖动落在这条带里）。
  for (int i = 0; i < 5; ++i) {
    d.push(Eigen::Vector3d(0.03, 0.0, 0.0));
  }
  EXPECT_TRUE(d.still()) << "enter/exit 之间的抖动不该解冻";
  // 超 exit 的运动：窗口滑动到均值超门就退，不等任何确认（快出是安全侧）。
  for (int i = 0; i < 20; ++i) {
    d.push(Eigen::Vector3d(0.1, 0.0, 0.0));
  }
  EXPECT_FALSE(d.still());

  // 换档即归零：旧样本的"静止"结论不属于新档位。
  d.reset();
  EXPECT_FALSE(d.valid());
  EXPECT_FALSE(d.still());
}

// 这一条是整个改写存在的理由：336L 实测静止时 |ω| ≈ 0.99°/s（≈0.0173 rad/s）且峰值≈均值，
// 即"恒定零偏"。旧版直接对 |ω| 设 1.15°/s 的门，余量 16%，零偏一漂就静默失效（永远判不出
// 静止 → P1 冻结从不触发，且外部看不出来）。减掉零偏后同样的输入必须判成静止。
TEST(GyroStill, MeasuredConstantBiasIsNotMistakenForMotion) {
  GyroStillDetector::Params p;
  p.bias_bootstrap_samples = 100;  // 缩短 bootstrap，让用例能在几百推内跑完
  GyroStillDetector d(p);
  const Eigen::Vector3d bias(0.0173, -0.0011, 0.0006);  // 模 ≈ 0.99°/s
  std::mt19937 rng(7);
  std::normal_distribution<double> jitter(0.0, 0.0005);  // 静止噪声级
  for (int i = 0; i < 300; ++i) {
    d.push(Eigen::Vector3d(bias.x() + jitter(rng), bias.y() + jitter(rng), bias.z() + jitter(rng)));
  }
  EXPECT_TRUE(d.bias_ready());
  EXPECT_GT(d.bias_rad_s(), 0.01) << "零偏没学到了实测的量级";
  EXPECT_TRUE(d.still()) << "恒定零偏被当成了运动：P1 冻结将永久失效";
  EXPECT_LT(d.recent_resid_rad_s(), p.enter_rad_s);

  // 零偏之上叠加真转动（0.05 rad/s ≈ 2.9°/s）：必须立刻不算静止。
  for (int i = 0; i < 40; ++i) {
    d.push(bias + Eigen::Vector3d(0.05, 0.0, 0.0));
  }
  EXPECT_FALSE(d.still()) << "叠加真转动后还判静止 = 门限形同虚设";
}

// 零偏学习必须被"安静"门控住：否则一段匀速转动会被 EMA 慢慢吸进零偏，转完再也判不出静止
// —— 那是比"判不出静止"更糟的状态（把运动当成了设备本身的偏置，永久污染）。
TEST(GyroStill, BiasDoesNotAbsorbSustainedRotationAfterBootstrap) {
  GyroStillDetector::Params p;
  p.bias_bootstrap_samples = 100;
  GyroStillDetector d(p);
  for (int i = 0; i < 200; ++i) {
    d.push(Eigen::Vector3d(0.002, 0.0, 0.0));  // 安静期：零偏锁到 0.002
  }
  ASSERT_TRUE(d.bias_ready());
  ASSERT_TRUE(d.still());
  const double bias_before = d.bias_rad_s();

  // 持续转动 20 万样本（≈17 分钟 @200Hz）：不门控的 EMA 早就把它吸进去了。
  for (int i = 0; i < 200000; ++i) {
    d.push(Eigen::Vector3d(0.0, 0.0, 0.2));
  }
  EXPECT_FALSE(d.still()) << "匀速转动中报静止";
  EXPECT_NEAR(d.bias_rad_s(), bias_before, 0.002) << "零偏把一段持续转动吸了进去";
}

TEST(GyroStill, NaNSamplesAreDroppedNotAveraged) {
  GyroStillDetector d;
  // 先攒进静止；之后 NaN 样本必须被挡在窗口外，而不是把均值毒成 NaN。
  for (int i = 0; i < 39; ++i) {
    d.push(Eigen::Vector3d(0.005, 0.0, 0.0));
  }
  ASSERT_TRUE(d.still());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 30; ++i) {
    d.push(Eigen::Vector3d(nan, nan, nan));
  }
  EXPECT_TRUE(d.still()) << "NaN 样本把静止结论毒掉了";
  EXPECT_TRUE(std::isfinite(d.recent_resid_rad_s()));
  EXPECT_TRUE(std::isfinite(d.bias_rad_s())) << "NaN 进了零偏估计：它不会自己漂出去";
}

TEST(GyroStill, ContradictoryParamsAreClampedAtConstruction) {
  GyroStillDetector::Params p;
  p.window_samples = 0;      // 非法：会除零/死循环的方向都要在构造期挡住（夹回 1）
  p.confirm_samples = 0;
  p.exit_rad_s = 0.0;        // < enter：迟滞方向写反，会被夹回 enter（迟滞不能消失）
  GyroStillDetector d(p);
  d.push(Eigen::Vector3d::Zero());
  // window 被夹成 1：一推就攒满。若没夹住（0），这里要么除零要么永不 valid。
  EXPECT_TRUE(d.valid());
  EXPECT_TRUE(d.still()) << "confirm 夹成 1 后，一个安静样本就该表态";
}

// ---------------------------------------------------------------- 前端契约

TEST(Frontend, CpuFrontendHonoursFullResolutionUvContract) {
  FrontendParams fp;
  std::string err;
  auto fe = make_frontend("cpu", fp, &err);
  ASSERT_NE(fe, nullptr) << err;

  cv::Mat img(480, 640, CV_8UC3);
  cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
  cv::GaussianBlur(img, img, cv::Size(0, 0), 1.2);  // 让角点数量落在合理区间
  const FeatureFrame fr = fe->extract(img, 12345);

  EXPECT_EQ(fr.image_size, img.size());
  EXPECT_EQ(fr.desc.type(), CV_32F);
  EXPECT_EQ(fr.desc.rows, static_cast<int>(fr.uv_px.size()));
  EXPECT_EQ(fr.ts_ns, 12345);
  ASSERT_FALSE(fr.empty());
  for (const auto& uv : fr.uv_px) {
    EXPECT_GE(uv.x, 0.f);
    EXPECT_LE(uv.x, static_cast<float>(img.cols));
    EXPECT_GE(uv.y, 0.f);
    EXPECT_LE(uv.y, static_cast<float>(img.rows));
  }
  for (int i = 0; i < fr.desc.rows; ++i) {
    EXPECT_NEAR(cv::norm(fr.desc.row(i)), 1.0, 1e-4) << "第 " << i << " 行未归一";
  }
}

TEST(Frontend, UnknownKindReportsInsteadOfThrowing) {
  FrontendParams fp;
  std::string err;
  EXPECT_EQ(make_frontend("not-a-thing", fp, &err), nullptr);
  EXPECT_NE(err.find("not-a-thing"), std::string::npos) << err;
}

// ------------------------------------------------ 单位边界（mm/deg）

namespace {
Eigen::Isometry3d make_T(double rx_deg, double ry_deg, double rz_deg, const Eigen::Vector3d& t_m) {
  const Eigen::Matrix3d R = (Eigen::AngleAxisd(rz_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(ry_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(rx_deg * M_PI / 180.0, Eigen::Vector3d::UnitX()))
                                .toRotationMatrix();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = t_m;
  return T;
}
}  // namespace

TEST(PoseIO, HandComputedAnchors) {
  // 任何一侧（C++ 或转发给 ServoP 的 Python）改了欧拉约定，这几条就会红。
  const DobotPose p = to_dobot(make_T(-20.0, 30.0, 90.0, {0.1, -0.2, 1.234}));
  EXPECT_NEAR(p.rx_deg, -20.0, 1e-9);
  EXPECT_NEAR(p.ry_deg, 30.0, 1e-9);
  EXPECT_NEAR(p.rz_deg, 90.0, 1e-9);
  EXPECT_NEAR(p.x_mm, 100.0, 1e-9);
  EXPECT_NEAR(p.y_mm, -200.0, 1e-9);
  EXPECT_NEAR(p.z_mm, 1234.0, 1e-6);

  // 反证：同一组角按 Rx·Ry·Rz 解释得到的是另一个矩阵 —— 说明上面那组锚点
  // 真的有鉴别力，不是"两个约定恰好给出同样数字"。
  const Eigen::Matrix3d wrong =
      (Eigen::AngleAxisd(-20.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(90.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()))
          .toRotationMatrix();
  EXPECT_GT((wrong - make_T(-20.0, 30.0, 90.0, Eigen::Vector3d::Zero()).linear()).norm(), 1e-3);

  const DobotPose id = to_dobot(Eigen::Isometry3d::Identity());
  EXPECT_DOUBLE_EQ(id.x_mm, 0.0);
  EXPECT_DOUBLE_EQ(id.rx_deg, 0.0);
  EXPECT_DOUBLE_EQ(id.ry_deg, 0.0);
  EXPECT_DOUBLE_EQ(id.rz_deg, 0.0);
}

TEST(PoseIO, GimbalLockStillReconstructsTheSameRotation) {
  // ry = ±90°：rz 与 rx 不可分，角度不唯一，但矩阵必须复现。
  // 恰好 ±90 走专门的退化分支；89.999999 走普通分支，此时 rx/rz 各自的误差被
  // 1/cos(ry) ≈ 6e7 放大 —— 矩阵里 1e-16 的舍入会写成 1e-9 rad 的角度，实测重建
  // 偏差 2e-8。这不是实现的缺陷，正是需要退化分支的原因，所以两处门限量级不同。
  const double tol_lock = 1e-6;  // 0.00006°，臂根本分辨不出来
  for (double ry : {90.0, -90.0, 89.999999, -89.999999}) {
    const bool exact = std::abs(std::abs(ry) - 90.0) < 1e-9;
    const double tol = exact ? 1e-9 : tol_lock;
    const Eigen::Isometry3d T = make_T(37.0, ry, -115.0, {0.2, 0.05, -0.9});
    const DobotPose p = to_dobot(T);
    ASSERT_TRUE(p.finite()) << ry;
    const Eigen::Isometry3d back = from_dobot(p);
    EXPECT_LT((back.linear() - T.linear()).norm(), tol) << "ry=" << ry;
    EXPECT_LT((back.translation() - T.translation()).norm(), 1e-9) << "ry=" << ry;
  }
}

TEST(PoseIO, RandomRoundTripPreservesMatrixAndMillimetres) {
  std::mt19937 rng(20260830);
  std::uniform_real_distribution<double> ang(-179.0, 179.0);
  std::uniform_real_distribution<double> m(-3.0, 3.0);
  for (int i = 0; i < 2000; ++i) {
    const Eigen::Isometry3d T = make_T(ang(rng), ang(rng), ang(rng), {m(rng), m(rng), m(rng)});
    const DobotPose p = to_dobot(T);
    ASSERT_TRUE(p.finite());
    const Eigen::Isometry3d back = from_dobot(p);
    EXPECT_LT((back.linear() - T.linear()).norm(), 1e-9) << "第 " << i << " 组";
    EXPECT_LT((back.translation() - T.translation()).norm(), 1e-9) << "第 " << i << " 组";
  }
  // 米→毫米不能有隐性缩放：0.5 m 必须正好 500 mm
  EXPECT_DOUBLE_EQ(to_dobot(make_T(0, 0, 0, {0.5, -0.25, 0.0})).x_mm, 500.0);
}

TEST(PoseIO, NonFinitePoseIsCaughtBeforeItBecomesMotion) {
  DobotPose p = to_dobot(make_T(0, 0, 0, {0.1, 0.2, 0.3}));
  EXPECT_TRUE(p.finite());
  p.z_mm = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(p.finite());
  DobotPose q = p;
  q.ry_deg = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(q.finite());
}

// ------------------------------------------------ 真机数据回归
// data/template_group/2026-08-25_215601 是实机采的对齐 RGB-D（有效深度 92.1%，
// 740–2589 mm，内参 fx 611.68 fy 611.70 cx 643.43 cy 405.15 @1280x800）。
// 相机没接的这台机器上，这是唯一一份「不是我们自己造出来」的输入。

namespace {
std::string scan_file(const char* name) {
#ifdef FOLLOW_SCAN_DIR
  const std::string p = std::string(FOLLOW_SCAN_DIR) + "/" + name;
  std::ifstream probe(p);
  return probe.good() ? p : std::string();
#else
  (void)name;
  return {};
#endif
}
}  // namespace

TEST(RealScan, MillimetreDepthContractEndToEnd) {
  const std::string path = scan_file("scan.depth.png");
  if (path.empty()) {
    GTEST_SKIP() << "没有真机扫描数据，跳过单位回归";
  }
  // 16 位 PNG 必须用 IMREAD_UNCHANGED 读；下一条测试就是这个坑本身。
  const cv::Mat depth = cv::imread(path, cv::IMREAD_UNCHANGED);
  ASSERT_EQ(depth.type(), CV_16UC1);

  const double valid_frac = static_cast<double>(cv::countNonZero(depth)) / depth.total();
  EXPECT_GT(valid_frac, 0.85) << "有效深度比例与实测 92.1% 差太多，数据是不是被改过";

  const CameraIntrinsics k = realK();
  CloudStats stats;
  Status st = Status::kOk;
  // zmax 放到 3 m，否则会把真实场景裁掉，测不到上界
  auto pts = depth_to_cloud(depth, k, 0.30, 3.00, 4, &stats, &st);
  ASSERT_EQ(st, Status::kOk);
  EXPECT_GT(pts.size(), 10000u);
  EXPECT_EQ(stats.kept, static_cast<int>(pts.size()));

  float zlo = 1e9f, zhi = -1e9f;
  for (const auto& p : pts) {
    ASSERT_TRUE(p.allFinite());
    zlo = std::min(zlo, p.z());
    zhi = std::max(zhi, p.z());
  }
  // 毫米→米是唯一解释：点云 z 的范围必须等于有效深度范围 /1000
  const cv::Mat valid = depth > 0;
  double nzmin = 0, nzmax = 0;
  cv::minMaxIdx(depth, &nzmin, &nzmax, nullptr, nullptr, valid);
  printf("[real_scan] 深度非零范围 %.0f–%.0f mm，有效 %.1f%%；点云 z %.3f–%.3f m，%zu 点\n", nzmin,
         nzmax, valid_frac * 100.0, zlo, zhi, pts.size());
  EXPECT_NEAR(zlo, nzmin * 0.001, 1e-3);
  EXPECT_NEAR(zhi, nzmax * 0.001, 1e-3);
}

TEST(RealScan, RejectsDefaultImreadOfSixteenBitPng) {
  const std::string path = scan_file("scan.depth.png");
  if (path.empty()) {
    GTEST_SKIP() << "没有真机扫描数据";
  }
  // 坑本身：cv::imread 默认把 16UC1 PNG 降成 8UC3。旧代码里没有类型检查，
  // 会把 BGR 的某个通道当深度用 —— 数值上"看起来"仍在 0.3–2.5 里，静默出错。
  const cv::Mat wrong = cv::imread(path);
  ASSERT_EQ(wrong.channels(), 3) << "OpenCV 行为变了，这条测试的前提要重看";
  Status st = Status::kOk;
  const auto pts = depth_to_cloud(wrong, realK(), 0.30, 2.50, 4, nullptr, &st);
  EXPECT_TRUE(pts.empty());
  EXPECT_EQ(st, Status::kNoDepth);
}

TEST(RealScan, CpuFrontendCostOnRealFrame) {
  const std::string path = scan_file("scan.color.jpg");
  if (path.empty()) {
    GTEST_SKIP() << "没有真机扫描数据";
  }
  const cv::Mat color = cv::imread(path);
  ASSERT_FALSE(color.empty());

  FrontendParams fp;
  std::string err;
  auto fe = make_frontend("cpu", fp, &err);
  ASSERT_NE(fe, nullptr) << err;

  const int reps = 15;
  auto time_at = [&](const cv::Size& sz) {
    cv::Mat img = color;
    if (sz.width != color.cols) {
      cv::resize(color, img, sz);
    }
    std::vector<double> ms;
    int nfeat = 0;
    for (int i = 0; i < reps; ++i) {
      const auto t0 = std::chrono::steady_clock::now();
      const FeatureFrame fr = fe->extract(img, i);
      const auto t1 = std::chrono::steady_clock::now();
      ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
      nfeat = static_cast<int>(fr.uv_px.size());
    }
    std::sort(ms.begin(), ms.end());
    printf("[frontend] %s @%dx%d: 中位 %.1f ms  min %.1f  max %.1f  特征 %d 个（预算 %d）\n",
           fe->name(), img.cols, img.rows, ms[reps / 2], ms.front(), ms.back(), nfeat,
           fp.max_features);
    EXPECT_GT(nfeat, 20);
  };
  time_at(color.size());     // 扫描存档分辨率
  time_at(cv::Size(848, 480));  // P4 计划的取流默认
}
