#include "follow/odometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>

namespace follow {
namespace {

// 法向/协方差估计的邻域数。GICP 的 Mahalanobis 权重就建立在这上面，跳过这一步
// 直接喂裸点云会让配准退化成 ICP。
constexpr int kNormalNeighbors = 10;
constexpr int kMinDensePoints = 12;

// 收敛判据：1 mm / 0.1°。比要达到的精度（P3 门：2 mm / 0.2°）低一个档次，
// 所以"没收敛"真的意味着没收敛，而不是被阈值掐断。
constexpr double kTranslationEpsM = 1e-3;
constexpr double kRotationEpsRad = 0.1 * M_PI / 180.0;

struct DenseSolution {
  bool ran = false;  // 求解器真的跑完了（不等于结果可信）
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
  // GICP 的总加权代价 e = Σ 0.5·ρᵢ，ρᵢ 是第 i 条对应点的马氏距离平方。和 inliers 一起
  // 给出模型自估的残差方差 s² = 2e/(3N) —— H⁺ 必须乘上 s² 才是协方差，见 uncertainty.hpp。
  double cost = 0.0;
  size_t inliers = 0;
  size_t used = 0;  // 降采样后真正参与配准的点数（包络比例的分母）
  int iterations = 0;
  bool converged = false;
};

DenseSolution align_dense(const ReferenceMap& map, const std::vector<Eigen::Vector3f>& cloud,
                          const Eigen::Isometry3d& T_init, const TrackParams& p) {
  DenseSolution s;
  if (map.empty() || cloud.size() < static_cast<size_t>(kMinDensePoints)) {
    return s;
  }

  const int threads = std::max(1, p.threads);
  auto pre = small_gicp::preprocess_points(cloud, p.voxel_m, kNormalNeighbors, threads);
  const small_gicp::PointCloud::Ptr& src = pre.first;
  if (!src || src->size() < static_cast<size_t>(kMinDensePoints)) {
    return s;
  }
  s.used = src->size();

  // 绕开 small_gicp::align(GaussianVoxelMap&, PointCloud&, …) 这个便捷函数：它内部
  // 根本不把 max_correspondence_distance 写进 rejector（源码里那行只在
  // align(PointCloud&, PointCloud&, KdTree&, …) 分支出现），于是对应点门停在
  // DistanceRejector 的默认值 1.0 m² —— 一米内的体素全算对应点。后果不是"精度差一点"，
  // 而是 num_inliers ≈ 全部点数，包络判据（inlier_ratio）彻底失去分辨力，
  // 出包络会被误报成正常。这里直接组装 Registration，让 rejector 成为可控项。
  using Solver = small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>;
  Solver solver;
  solver.reduction.num_threads = threads;
  // 对应点门不能超过搜索结构的可达半径：参考地图只在自己的 3x3x3 体素邻域里找最近邻，
  // 体素均值最远在 ~2.6·voxel 外。门开得比这更大不会"匹配得更远"，只会让求解器一个
  // 对应点都找不到 —— 而那会被上层读成 kOutOfEnvelope，是个彻底的误诊。
  const double reach_m = 2.6 * map.voxel_m();
  const double gate_m = std::min(p.max_corr_m, reach_m);
  solver.rejector.max_dist_sq = gate_m * gate_m;
  solver.criteria.translation_eps = kTranslationEpsM;
  solver.criteria.rotation_eps = kRotationEpsRad;
  solver.optimizer.max_iterations = std::max(1, p.max_iters);

  const auto res = solver.align(map.voxel_map(), *src, map.voxel_map(), T_init);
  // GICPFactor 用的是普通 3x3 .inverse()（不是伪逆）：局部协方差秩亏时 mahalanobis
  // 直接变 inf/NaN，于是 T 也是 NaN。NaN 躲得过所有范围比较，所以这里必须挡一次 ——
  // 一个 NaN 位姿流到 ServoP 就是一次不可预测的臂动作。
  if (!res.T_target_source.matrix().allFinite() || !res.H.allFinite()) {
    return s;  // ran=false：调用方看到的是"没有稠密解"，而不是一个假解
  }
  s.ran = true;
  s.T = res.T_target_source;
  s.H = res.H;
  s.inliers = res.num_inliers;
  s.cost = res.error;
  s.iterations = static_cast<int>(res.iterations);
  s.converged = res.converged;
  return s;
}

Uncertainty no_uncertainty() {
  Uncertainty u;
  u.solver_failed = true;
  for (int i = 0; i < 3; ++i) {
    u.rot_sigma_deg[i] = std::numeric_limits<double>::infinity();
    u.trans_sigma_mm[i] = std::numeric_limits<double>::infinity();
  }
  return u;
}

}  // namespace

Tracker::Tracker(TrackParams p, const ReferenceMap& map, uint32_t seed)
    : p_(std::move(p)), map_(map), rng_(seed) {
  // 稠密路径读 p_.zmin_m/zmax_m，稀疏路径读 p_.sparse.zmin_m/zmax_m —— 两套数各走各的时，
  // 同一条深度管线会对"哪些像素有效"给出两种答案，且两边都不报错。构造期强制单一来源。
  p_.sparse.zmin_m = p_.zmin_m;
  p_.sparse.zmax_m = p_.zmax_m;
}

void Tracker::push_gyro(int64_t ts_ns, const Eigen::Vector3d& omega_cam_rad_s) {
  if (!omega_cam_rad_s.allFinite()) {
    return;  // 一个 NaN 样本会把整段积分染成 NaN，而 NaN 的 R 能通过所有比较
  }
  gyro_.push_back(GyroSample{ts_ns, omega_cam_rad_s});
  while (gyro_.size() > p_.gyro_buf_max) {
    gyro_.pop_front();
  }
}

TrackResult Tracker::track(const FeatureFrame& curr, const cv::Mat& depth_mm, int64_t ts_ns) {
  TrackResult r = track_impl(curr, depth_mm, ts_ns);

  // 只有真出了点云的帧才进"上一帧"。坏帧当参考帧会把下一帧的初值一起带坏；
  // 时间戳倒退的帧更不能覆盖 prev_ts_ns_，否则下一次同样的乱序会被当成正常。
  if (r.status != Status::kConfigInvalid && r.status != Status::kStaleInput &&
      r.status != Status::kNoDepth) {
    prev_sp_ = curr;
    prev_depth_ = depth_mm.clone();  // 取流层会复用缓冲，这里必须断开别名
    prev_ts_ns_ = ts_ns;
    have_frame_ = true;
  }
  return r;
}

TrackResult Tracker::track_impl(const FeatureFrame& curr, const cv::Mat& depth_mm, int64_t ts_ns) {
  TrackResult r;
  r.unc = no_uncertainty();

  auto hold = [&](Status s) {
    r.status = s;
    r.estimator = Estimator::kNone;
    r.T_ref_cam = T_last_good_;  // 保持上一目标
    // 运动链断了：不清 T_vel_ 的话，恢复时的初值会从故障前的外推接着推，
    // 表现成"故障后第一帧突然跳一段"。
    T_vel_ = Eigen::Isometry3d::Identity();
    return r;
  };
  auto adopt = [&](Status s, Estimator e, const Eigen::Isometry3d& T) {
    r.status = s;
    r.estimator = e;
    T_vel_ = T_last_good_.inverse() * T;
    T_last_good_ = T;
    r.T_ref_cam = T;
    if (e == Estimator::kGicp) {
      sparse_streak_ = 0;
    } else {
      ++sparse_streak_;
    }
    return r;
  };

  Status cloud_status = Status::kOk;
  CloudStats stats;
  const std::vector<Eigen::Vector3f> cloud = depth_to_cloud(depth_mm, p_.k, p_.zmin_m, p_.zmax_m,
                                                             p_.depth_stride, &stats, &cloud_status);
  if (cloud_status != Status::kOk) {
    r.status = cloud_status;
    r.estimator = Estimator::kNone;
    r.T_ref_cam = T_last_good_;
    return r;  // 配置/输入问题：不动任何状态，也没资格当参考帧
  }
  if (have_frame_ && ts_ns <= prev_ts_ns_) {
    r.status = Status::kStaleInput;
    r.estimator = Estimator::kNone;
    r.T_ref_cam = T_last_good_;
    return r;
  }
  if (map_.empty()) {
    return hold(Status::kConfigInvalid);  // 没有冻结基准，谈不上"相对示教位的修正"
  }
  if (static_cast<int>(cloud.size()) < p_.min_cloud_points) {
    r.cloud_points = cloud.size();
    return hold(Status::kNoDepth);
  }

  // ---- 帧间初值：稠密配准对初值极敏感，这一步决定它是收敛还是爬到隔壁的体素上 ----
  std::optional<SparseDelta> sp;
  if (have_frame_ && !curr.empty() && !prev_sp_.empty() && !prev_depth_.empty()) {
    sp = sparse_delta(prev_sp_, curr, prev_depth_, depth_mm, p_.k, p_.sparse, rng_);
  }
  r.sparse_inliers = sp ? sp->inliers : 0;

  GyroDelta gyro;
  if (have_frame_) {
    gyro = integrate_gyro(gyro_, prev_ts_ns_, ts_ns, p_.gyro_max_gap_ns);
  }

  // 三档，从"有实测"往"只有惯性/只有历史"退：
  Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
  if (sp) {
    dT = sp->T_prev_from_curr;  // 1) 特征 3D-3D：六自由度都有深度支撑
  } else if (gyro.valid()) {
    // 2) 陀螺只补旋转。平移留 0 —— 没有深度依据的外推平移比不外推更危险。
    dT.linear() = gyro.R;
    r.gyro_used = true;
  } else {
    // 3) 沿用上一帧刚测到的帧间运动（相机固定时它≈单位阵，等价于"假设没动"）。
    dT = T_vel_;
  }
  const Eigen::Isometry3d T_init = T_last_good_ * dT;

  // ---- 稠密配准到冻结参考地图 ----
  const DenseSolution g = align_dense(map_, cloud, T_init, p_);
  r.cloud_points = g.used > 0 ? g.used : cloud.size();
  r.gicp_inliers = static_cast<int>(g.inliers);
  r.gicp_cost = g.cost;
  r.iterations = g.iterations;
  r.converged = g.converged;
  r.inlier_ratio = r.cloud_points > 0 ? static_cast<double>(g.inliers) / static_cast<double>(r.cloud_points) : 0.0;

  const int min_inliers = std::max(1, p_.min_gicp_inliers);
  const bool overlapped = g.ran && r.inlier_ratio >= p_.min_inlier_ratio;

  // 包络判据放在收敛判据之前：初值来自上一帧的实测运动，所以"重叠突然不够"不是
  // 求解器的问题，是场景相对参考几何不见了/换件了。这时候稀疏解救不了 —— 它测的是
  // 帧间运动，恰恰不含"相对示教位"的信息。
  if (g.ran && !overlapped) {
    return hold(Status::kOutOfEnvelope);
  }

  if (overlapped && g.converged && g.inliers >= static_cast<size_t>(min_inliers)) {
    // s² = 2·cost/(3·N)：每条对应点 3 个自由度，GICP 自估的残差协方差正确时 E[ρ]=3 ⇒ s²≈1。
    // 实测工位场景 s² ~ 1e-4，即 GICP 假设的残差比真实残差大方差 100 倍（体素几何展宽 vs
    // 深度噪声）；σ 不乘 sqrt(s²) 就会把 0.1 mm 的精度报成 15 mm —— 见 uncertainty.hpp。
    const double s2 = 2.0 * g.cost / (3.0 * static_cast<double>(g.inliers));
    r.unc = hessian_uncertainty(g.H, std::max(s2, p_.min_residual_var_scale));
    const bool observable = r.unc.within(p_.max_trans_sigma_mm, p_.max_rot_sigma_deg,
                                         p_.max_group_anisotropy);
    // 退化时仍然采用这个解：可观方向是真测量，不可观方向因为该方向上梯度≈0 而
    // LM 步长 delta = -b/(λ_i+λ) 天然留在初值上。整帧冻住的代价是把其余五个方向的
    // 实测也一起丢掉 —— 而"一大片平面"正是喷涂工位的常态，不是极端情况。
    // 状态仍然报 kDegenerate：让上层知道某一维没被测量，而不是假装量到了。
    return adopt(observable ? Status::kOk : Status::kDegenerate, Estimator::kGicp, g.T);
  }

  // ---- 几何可用但求解失败：特征递推当替补，受连击上限约束 ----
  if (overlapped && sp && sparse_streak_ < std::max(1, p_.max_sparse_streak)) {
    return adopt(Status::kOk, Estimator::kSparse, T_last_good_ * sp->T_prev_from_curr);
  }

  return hold(Status::kLost);
}

}  // namespace follow
