#include "follow/odometry.hpp"

#include <algorithm>
#include <chrono>
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

// 分阶段耗时。上层只有一个聚合的 compute_ms，而"这一帧慢"至少有三条完全不同的成因
// （点云变稠密 / 掉到稀疏回退 / GICP 迭代变多），三者的处置也各不相同 —— 没有这一层
// 就只能猜，而 §6 里第一轮正是靠猜把收益算错的。
inline int64_t now_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
inline double us2ms(int64_t us) { return static_cast<double>(us) * 1e-3; }

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

// 两个旋转的测地线距离（度）。P3 离群门用它比较"视觉测出的帧间旋转"与"陀螺积分出的帧间旋转"。
// trace 必须夹进 [-1,1]：浮点噪声让它偶尔越界，而 acos 在界外吐 NaN —— NaN 能穿过一切比较。
double rot_geodesic_deg(const Eigen::Matrix3d& Ra, const Eigen::Matrix3d& Rb) {
  const Eigen::Matrix3d dR = Ra.transpose() * Rb;
  const double c = std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) / 2.0));
  return std::acos(c) * 180.0 / M_PI;
}

}  // namespace

Tracker::Tracker(TrackParams p, const ReferenceMap& map, uint32_t seed)
    : p_(std::move(p)), map_(map), rng_(seed), still_det_(p_.gyro_still) {
  // 稠密路径读 p_.zmin_m/zmax_m，稀疏路径读 p_.sparse.zmin_m/zmax_m —— 两套数各走各的时，
  // 同一条深度管线会对"哪些像素有效"给出两种答案，且两边都不报错。构造期强制单一来源。
  p_.sparse.zmin_m = p_.zmin_m;
  p_.sparse.zmax_m = p_.zmax_m;
}

void Tracker::push_gyro(int64_t ts_ns, const Eigen::Vector3d& omega_cam_rad_s) {
  if (!omega_cam_rad_s.allFinite()) {
    return;  // 一个 NaN 样本会把整段积分染成 NaN，而 NaN 的 R 能通过所有比较
  }

  // 同源样本喂静止检测器：它只用向量、不碰时间轴，与积分路径不会各算各的。
  // 注意：静止检测器必须吃裸数据，因为它自己内部负责估计零偏。
  still_det_.push(omega_cam_rad_s);
  ++gyro_this_frame_;  // 与时间戳无关的"IMU 还活着"计数：静止判据的存活前提（见 track_impl）

  // 写入缓冲的数据必须扣掉已经学到的零偏，否则 integrate_gyro 长时间积分时纯零偏会积出
  // 巨大的假旋转（1°/s 零偏 * 2 秒 = 2°），导致离群门误判并永久死锁。
  Eigen::Vector3d omega_clean = omega_cam_rad_s;
  if (still_det_.bias_ready()) {
    omega_clean -= still_det_.bias_vec();
  }
  gyro_.push_back(GyroSample{ts_ns, omega_clean});

  // 按**时间**裁剪，不是只按条数：一帧的积分窗口只有 66 ms 宽（离群门的窗口另有上限），而
  // gyro_buf_max 条 @200Hz 是 20 秒历史 —— integrate_gyro 每帧从头走一遍那些样本却一个都用
  // 不到。条数上限留着，只兜"时间戳乱序让时间条件永远不成立"这一种异常。
  while (gyro_.size() > 2 && ts_ns - gyro_.front().ts_ns > p_.gyro_horizon_ns) {
    gyro_.pop_front();
  }
  while (gyro_.size() > p_.gyro_buf_max) {
    gyro_.pop_front();
  }
}

TrackResult Tracker::track(const FeatureFrame& curr, const cv::Mat& depth_mm, int64_t ts_ns) {
  TrackResult r = track_impl(curr, depth_mm, ts_ns);
  gyro_this_frame_ = 0;  // 一帧的账在这一帧结：下一帧的存活判据只看下一帧收到的样本

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
    last_good_ts_ns_ = ts_ns;   // 离群门的积分起点：必须和"T_last_good_ 是哪一帧"绑定
    have_last_good_ts_ = true;
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
  const int64_t t_cloud = now_us();
  const std::vector<Eigen::Vector3f> cloud = depth_to_cloud(depth_mm, p_.k, p_.zmin_m, p_.zmax_m,
                                                             p_.depth_stride, &stats, &cloud_status);
  r.cloud_ms = us2ms(now_us() - t_cloud);
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
    const int64_t t_sparse = now_us();
    sp = sparse_delta(prev_sp_, curr, prev_depth_, depth_mm, p_.k, p_.sparse, rng_);
    r.sparse_ms = us2ms(now_us() - t_sparse);
  }
  r.sparse_inliers = sp ? sp->inliers : 0;

  GyroDelta gyro;
  gyro.stale = true;  // 首帧没有区间可积：valid() 现在等价于 !stale，不能靠默认值兜底
  if (have_frame_) {
    gyro = integrate_gyro(gyro_, prev_ts_ns_, ts_ns, p_.gyro_max_gap_ns);
  }
  r.gyro_samples = gyro.samples_used;
  r.gyro_span_ns = gyro.span_ns;
  r.gyro_gap_end_ns = gyro.gap_end_ns;
  r.gyro_extrap_ns = gyro.extrap_ns;
  r.gyro_buf = static_cast<int>(gyro_.size());
  r.gyro_pushed = gyro_this_frame_;
  r.gyro_resid_rad_s = still_det_.recent_resid_rad_s();
  r.gyro_bias_rad_s = still_det_.bias_rad_s();
  r.gyro_bias_ready = still_det_.bias_ready();
  // P1 静止判定随帧报出。这里刻意**不**拿 gyro.valid() 当前提：检测器的判据只用样本向量、
  // 与时间轴无关，用它自己的时间无关性去 gate 一个时间相关的积分结果，等于把"陀螺与帧不同
  // 时间域"这类故障伪装成"相机一直在动"—— 服务路径上真实发生过，且从外部完全看不出来。
  // 取而代之的两个存活条件也都与时间域无关：
  //  · bias_ready：零偏还在累积均值阶段，残差门限没有意义；
  //  · 本帧收到了新样本：IMU 停更时窗口会一直卡在最后一批旧样本上，必须当"不知道"。
  r.gyro_still = still_det_.still() && r.gyro_bias_ready && r.gyro_pushed > 0;

  // P3 离群门：候选解的旋转增量必须与**同一段时间**的陀螺积分一致（测地线距离 ≤ 门限）。
  // 陀螺是唯一能在"结果被采纳之前"独立验证旋转的信息源：一次坏帧（滑坡/遮挡后的假收敛）
  // 的帧间旋转会和陀螺差出度级角度，而正常帧两者差在噪声级（0.0几度）。
  // 退化场景不误伤：弱方向上 GICP 的旋转留在初值上不动（见下面 adopt 的注释），
  // 帧间旋转≈初值链上的运动，与陀螺仍一致。
  //
  // **区间两边必须严格对齐**，这是本门最容不下错的一点：稠密候选是相对 T_last_good_ 算的，
  // 而 last_good 可能已经隔着若干被 hold 的帧。若陀螺只积一帧，门误差 ≈ (区间长度差)×角速度
  // ⇒ 转速越高越容易过门 ⇒ 拦下一次之后每帧都更越过门，正反馈锁死；且相机停下也解不开
  // （累计旋转仍超门），只能重新示教。所以按候选各自的参照时刻积分对应区间。
  const auto rot_gate = [&](const Eigen::Isometry3d& T_cand, const Eigen::Isometry3d& T_ref,
                            int64_t ref_ts_ns) {
    if (p_.gyro_rot_gate_deg <= 0.0 || ref_ts_ns <= 0) {
      return false;
    }
    const int64_t span = ts_ns - ref_ts_ns;
    // 区间超过缓冲跨度就不判：几秒的窗口里陀螺零偏本身就已经是门限量级，互验没有意义。
    if (span <= 0 || span > p_.gyro_horizon_ns) {
      return false;
    }
    // 上一帧就是参照帧时（绝大多数情况）直接复用上面算好的单帧积分，不重复遍历缓冲。
    const GyroDelta g = (ref_ts_ns == prev_ts_ns_)
                            ? gyro
                            : integrate_gyro(gyro_, ref_ts_ns, ts_ns, p_.gyro_max_gap_ns);
    if (!g.valid()) {
      return false;  // 这段没有可用样本（IMU 停更 / 时间域不同）⇒ 不判，而不是"判成坏帧"
    }
    r.rot_gate_err_deg =
        rot_geodesic_deg(g.R, T_ref.rotation().transpose() * T_cand.rotation());
    // 动态门限：下限 + 斜率 × 本区间陀螺实际转角。转角项自动吸收了帧间隔与被 hold 的帧数
    // （区间越长、转得越多 ⇒ 门越宽），所以低速误伤与高速惰性不再是需要折中的同一件事。
    const double gyro_angle_deg =
        Eigen::AngleAxisd(g.R).angle() * 180.0 / M_PI;
    r.rot_gate_limit_deg =
        p_.gyro_rot_gate_deg + std::max(0.0, p_.gyro_rot_gate_relax) * gyro_angle_deg;
    return r.rot_gate_err_deg > r.rot_gate_limit_deg;
  };

  // 三档，从"有实测"往"只有惯性/只有历史"退：
  Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
  if (sp) {
    dT = sp->T_prev_from_curr;  // 1) 特征 3D-3D：六自由度都有深度支撑
  } else if (gyro.valid()) {
    // 2) 旋转信陀螺，平移信上一帧实测运动的常速外推。
    // 只给旋转、平移留 0 曾在相机持续平移时坑过稠密配准：初值整体偏掉，
    // GICP 不收敛直接掉到 lost。T_vel_ 的平移是实测（不是凭空外推），静止时它≈0，
    // 退化为原来的纯陀螺旋转档，不引入新风险。
    dT.linear() = gyro.R;
    dT.translation() = T_vel_.translation();
    r.gyro_used = true;
  } else {
    // 3) 沿用上一帧刚测到的帧间运动（相机固定时它≈单位阵，等价于"假设没动"）。
    dT = T_vel_;
  }
  const Eigen::Isometry3d T_init = T_last_good_ * dT;

  // ---- 稠密配准到冻结参考地图 ----
  const int64_t t_dense = now_us();
  const DenseSolution g = align_dense(map_, cloud, T_init, p_);
  r.dense_ms = us2ms(now_us() - t_dense);
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
    // 参照 = T_last_good_ 及其所属时刻。没有它就没有合法区间（不能拿"未知"当"没动"）。
    if (rot_gate(g.T, T_last_good_, have_last_good_ts_ ? last_good_ts_ns_ : 0)) {
      // 疑似坏帧：不采纳、位姿保持上一可信值。不更新 T_last_good_ 也就不会污染后续初值；
      // 本帧仍可当下一帧的稀疏参照（track() 里 prev 的更新条件不含 kRotGated）。
      r.rot_gated = true;
      return hold(Status::kRotGated);
    }
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
    const Eigen::Isometry3d T_sp = T_last_good_ * sp->T_prev_from_curr;
    // 参照 = 上一帧：sp->T_prev_from_curr 测的就是"prev → curr"这一段，陀螺也只积这一段。
    if (rot_gate(T_sp, T_last_good_, prev_ts_ns_)) {
      r.rot_gated = true;
      return hold(Status::kRotGated);
    }
    return adopt(Status::kOk, Estimator::kSparse, T_sp);
  }

  return hold(Status::kLost);
}

}  // namespace follow
