// P0 冒烟测试：验证 follow 将要依赖的 small_gicp API 路径确实成立，并量出实时预算。
// 这些断言是 P2 设计的地基，不是回归测试；对不上就说明假设错了，要先改设计。

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/registration/registration_result.hpp>

namespace {

using Vec3fList = std::vector<Eigen::Vector3f>;

// 一个有结构的场景：若干长方体表面点 + 一点散布，模拟工装/型材。
Vec3fList make_scene(unsigned seed, int per_face = 400) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> u(0.f, 1.f);
  Vec3fList pts;
  const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> boxes = {
      {Eigen::Vector3f(-0.5f, -0.3f, 0.8f), Eigen::Vector3f(0.5f, 0.3f, 1.4f)},
      {Eigen::Vector3f(-0.2f, 0.4f, 0.6f), Eigen::Vector3f(0.1f, 0.8f, 1.1f)},
      {Eigen::Vector3f(0.3f, -0.6f, 0.5f), Eigen::Vector3f(0.8f, -0.2f, 1.2f)},
  };
  for (const auto& b : boxes) {
    const Eigen::Vector3f lo = b.first, hi = b.second;
    const Eigen::Vector3f d = hi - lo;
    for (int f = 0; f < 6; ++f) {
      const int a = f / 2, b1 = (a + 1) % 3, b2 = (a + 2) % 3;
      for (int i = 0; i < per_face; ++i) {
        Eigen::Vector3f p = lo;
        p[a] = (f % 2 == 0) ? lo[a] : hi[a];
        p[b1] = lo[b1] + u(rng) * d[b1];
        p[b2] = lo[b2] + u(rng) * d[b2];
        pts.push_back(p);
      }
    }
  }
  return pts;
}

double pose_error_m(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  const Eigen::Matrix3d R = a.linear().transpose() * b.linear();
  const double ang = std::acos(std::min(1.0, std::max(-1.0, (R.trace() - 1.0) / 2.0)));
  return (a.translation() - b.translation()).norm() * 1000.0 + ang * 180.0 / M_PI;
}

small_gicp::RegistrationSetting settings(int type, int threads) {
  small_gicp::RegistrationSetting s;
  s.type = static_cast<small_gicp::RegistrationSetting::RegistrationType>(type);
  s.num_threads = threads;
  s.max_correspondence_distance = 0.15;
  s.max_iterations = 20;
  s.downsampling_resolution = 0.02;
  s.voxel_resolution = 0.05;
  return s;
}

// 真值位姿：小角度旋转 + 毫米级平移，正是工位要检测的量级。
Eigen::Isometry3d ground_truth() {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(1.5 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.8 * M_PI / 180.0, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  T.translation() = Eigen::Vector3d(0.012, -0.008, 0.005);
  return T;
}

}  // namespace

// 1. sketch 里 `align(target_vec, source_vec, T_init, setting)` 的写法是合法的
//    （我原先怀疑它编不过，实测：templated Eigen 接口存在）。
TEST(SmallGicpApi, EigenVectorInterfaceCompilesAndConverges) {
  const Vec3fList scene = make_scene(42);
  const Eigen::Isometry3d Tgt = ground_truth();

  Vec3fList target;
  target.reserve(scene.size());
  for (const auto& p : scene) {
    target.emplace_back((Tgt * p.cast<double>()).cast<float>());
  }

  auto r = small_gicp::align(target, scene, Eigen::Isometry3d::Identity(), settings(2 /*GICP*/, 4));
  EXPECT_TRUE(r.converged);
  EXPECT_GT(r.num_inliers, scene.size() / 2u);
  // 结果类型必须是 Isometry3d，否则 follow 里的 `T_wc_ = g.T_target_source` 赋值不成立。
  const Eigen::Isometry3d est = r.T_target_source;
  EXPECT_LT((est.translation() - Tgt.translation()).norm(), 2e-3)
      << "Eigen-interface translation err (m): " << (est.translation() - Tgt.translation()).norm();
}

// 2. P2 的设计前提：冻结的 GaussianVoxelMap 可以当 target，每帧只预处理源云。
TEST(SmallGicpApi, FrozenVoxelMapAlignsPreprocessedSource) {
  const Vec3fList scene = make_scene(7);
  const Eigen::Isometry3d Tgt = ground_truth();

  auto raw_target = std::make_shared<small_gicp::PointCloud>(scene);
  Vec3fList target_pts;
  target_pts.reserve(scene.size());
  for (const auto& p : scene) {
    target_pts.emplace_back((Tgt * p.cast<double>()).cast<float>());
  }
  auto raw_target_gt = std::make_shared<small_gicp::PointCloud>(target_pts);
  const small_gicp::GaussianVoxelMap::Ptr map =
      small_gicp::create_gaussian_voxelmap(*raw_target_gt, 0.05);
  ASSERT_NE(map, nullptr);
  ASSERT_GT(map->size(), 100u) << "voxel map too sparse to register against";

  // 源云要带法向/协方差，GICPFactor 才成立 —— preprocess_points 提供这个。
  auto [src_pc, src_kd] = small_gicp::preprocess_points(*raw_target, 0.02, 10, 1);
  ASSERT_EQ(src_pc->covs.size(), src_pc->size());

  small_gicp::RegistrationSetting s = settings(3 /*VGICP*/, 4);
  const small_gicp::RegistrationResult r =
      small_gicp::align(*map, *src_pc, Eigen::Isometry3d::Identity(), s);

  EXPECT_TRUE(r.converged);
  const double err = pose_error_m(r.T_target_source, Tgt);
  EXPECT_LT(err, 1.0) << "map-based pose error (mm+deg): " << err;
}

// 3. 实时预算：冻结地图路径必须显著便宜于每帧重建目标。这条决定 15fps 可达性。
TEST(SmallGicpApi, FrozenMapPathIsCheaperThanRebuildingTargetEveryFrame) {
  const Vec3fList scene = make_scene(11, 900);
  const int n = 5;

  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    auto r = small_gicp::align(scene, scene, Eigen::Isometry3d::Identity(), settings(2, 4));
    (void)r;
  }
  const double eigen_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / n;

  Vec3fList shifted;
  shifted.reserve(scene.size());
  for (const auto& p : scene) {
    shifted.emplace_back(p.x() + 0.01f, p.y(), p.z());
  }
  auto [map_src, map_kd] = small_gicp::preprocess_points(
      small_gicp::PointCloud(shifted), 0.02, 10, 4);
  const small_gicp::GaussianVoxelMap::Ptr map =
      small_gicp::create_gaussian_voxelmap(*map_src, 0.05);

  t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    auto [src, src_kd2] =
        small_gicp::preprocess_points(small_gicp::PointCloud(scene), 0.02, 10, 4);
    small_gicp::RegistrationSetting s = settings(3, 4);
    auto r = small_gicp::align(*map, *src, Eigen::Isometry3d::Identity(), s);
    (void)r;
  }
  const double map_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / n;

  std::printf("[timing] align(Eigen vec)  = %.2f ms/call (%zu+%zu pts)\n", eigen_ms,
              scene.size(), scene.size());
  std::printf("[timing] align(frozen map) = %.2f ms/call (source preprocessed per frame)\n", map_ms);
  EXPECT_LT(map_ms, eigen_ms) << "frozen-map path did not beat the per-frame rebuild path";
}

// 4. num_threads >= 2 时库自己承认有 run-by-run 不确定性。量化它，决定验收门是否单线程跑。
TEST(SmallGicpApi, QuantifyMultithreadNondeterminism) {
  const Vec3fList scene = make_scene(23, 700);
  const Eigen::Isometry3d Tgt = ground_truth();
  Vec3fList target;
  target.reserve(scene.size());
  for (const auto& p : scene) {
    target.emplace_back((Tgt * p.cast<double>()).cast<float>());
  }

  auto spread = [&](int threads) {
    double worst = 0.0;
    Eigen::Isometry3d first;
    for (int i = 0; i < 5; ++i) {
      auto r = small_gicp::align(target, scene, Eigen::Isometry3d::Identity(), settings(2, threads));
      if (i == 0) {
        first = r.T_target_source;
      } else {
        worst = std::max(worst, (r.T_target_source.translation() - first.translation()).norm());
      }
    }
    return worst;
  };

  const double s1 = spread(1), s4 = spread(4);
  std::printf("[determinism] translation spread: threads=1 -> %.3e m, threads=4 -> %.3e m\n", s1, s4);
  EXPECT_LT(s1, 1e-6) << "single-thread registration should be deterministic";
}

// 5. 单一平面：刻画退化场景下 small_gicp 的真实报数，作为 P1 新检测器的设计依据。
//    实测结论（本机）：ratio=6.7e-4，而 converged=1、inliers≈8.7k。
//    即「收敛 + 内点多」完全不能当可观测性证据，且 1e-3 这个门限离真实值太近，不可信。
TEST(SmallGicpApi, CharacterisesDegeneratePlanarScene) {
  std::mt19937 rng(5);
  std::uniform_real_distribution<float> u(-1.f, 1.f);
  Vec3fList plane;
  for (int i = 0; i < 20000; ++i) {
    plane.emplace_back(u(rng), u(rng), 1.0f);
  }
  // 面内平移：理想上不可观，但配准器照样会「成功」。
  Vec3fList shifted;
  shifted.reserve(plane.size());
  for (const auto& p : plane) {
    shifted.emplace_back(p.x() + 0.02f, p.y() + 0.01f, p.z());
  }

  auto r = small_gicp::align(plane, shifted, Eigen::Isometry3d::Identity(), settings(2, 4));
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(r.H);
  ASSERT_EQ(es.info(), Eigen::Success);
  const Eigen::VectorXd ev = es.eigenvalues();
  const double ratio = ev.minCoeff() / ev.maxCoeff();
  std::printf("[planar] H eigenvalues min=%.3e max=%.3e ratio=%.3e (converged=%d inliers=%zu)\n",
              ev.minCoeff(), ev.maxCoeff(), ratio, int(r.converged), r.num_inliers);

  // 面内不可观，但残差非零：ratio 只比 sketch 的 1e-3 小 1.5 倍，稍有结构就翻过门限。
  EXPECT_LT(ratio, 1e-3) << "wall scene eigenvalue spread:\n" << ev.transpose();
  EXPECT_GT(ratio, 1e-6) << "wall is NOT numerically singular at 1e-6; the old assumption was wrong";

  // 这两条是故意「通过」的：证明光看收敛和内点数会漏掉退化。
  EXPECT_TRUE(r.converged) << "GICP reports convergence on a degenerate wall";
  EXPECT_GT(r.num_inliers, 1000u) << "inlier count looks healthy on a degenerate wall";
}
