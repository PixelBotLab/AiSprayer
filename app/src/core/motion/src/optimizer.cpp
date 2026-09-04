#include "motion/optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace motion {
namespace {

constexpr double kSegmentTravelDeg = 120.0;
constexpr double kSegmentTravelCapDeg = 170.0;
constexpr double kSegmentTravelDegPerMm = 0.9;

JointVec DefaultSeed() {
  return (JointVec() << 0.0, 0.0, -kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0).finished();
}

struct DpNode {
  Transform T;
  JointVec q = JointVec::Zero();
  JointVec q_branch = JointVec::Zero();
  double zero_dev = 0.0;
  int branch_id = 0;
  int ew_family = 0;
};

struct CandPack {
  std::vector<Transform> T;
  std::vector<double> geo;
  std::vector<double> zero;
  std::vector<JointVec> q;
  std::vector<int> pose_idx;
  std::vector<int> branch_id;
};

// 候选排序分：零位偏差为主，测地距为次。分支内取前 max_candidates_per_branch 个。
double PoseScore(const CandPack& pack, int m) {
  const size_t pi = static_cast<size_t>(pack.pose_idx[static_cast<size_t>(m)]);
  return pack.zero[pi] + 0.01 * pack.geo[pi];
}

DpNode MakeNode(const CandPack& pack, int m) {
  const size_t mm = static_cast<size_t>(m);
  const size_t pi = static_cast<size_t>(pack.pose_idx[mm]);
  DpNode nd;
  nd.T = pack.T[pi];
  nd.q = pack.q[mm];
  nd.q_branch = nd.q;
  nd.zero_dev = pack.zero[pi];
  nd.branch_id = pack.branch_id[mm];
  nd.ew_family = nd.branch_id & 3;
  return nd;
}

// 展开全部候选（不做分支内截断），用于 beam 剪枝后整层不可达时的回退重试。
std::vector<DpNode> Materialize(const CandPack& pack) {
  std::vector<DpNode> nodes;
  nodes.reserve(pack.q.size());
  for (size_t m = 0; m < pack.q.size(); ++m) nodes.push_back(MakeNode(pack, static_cast<int>(m)));
  return nodes;
}

Eigen::Matrix3d AxisRot(double deg, int axis) {
  const double a = Rad(deg), c = std::cos(a), s = std::sin(a);
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
  if (axis == 0) {
    m << 1, 0, 0, 0, c, -s, 0, s, c;
  } else if (axis == 1) {
    m << c, 0, s, 0, 1, 0, -s, 0, c;
  } else {
    m << c, -s, 0, s, c, 0, 0, 0, 1;
  }
  return m;
}

std::vector<Eigen::Matrix3d> PrecomputeOffsets(const AxisGrid& gx, const AxisGrid& gy,
                                               const AxisGrid& gz) {
  const auto xs = ExpandAxisGrid(gx);
  const auto ys = ExpandAxisGrid(gy);
  const auto zs = ExpandAxisGrid(gz);
  std::vector<Eigen::Matrix3d> Rx, Ry, Rz;
  Rx.reserve(xs.size());
  Ry.reserve(ys.size());
  Rz.reserve(zs.size());
  for (double x : xs) Rx.push_back(AxisRot(x, 0));
  for (double y : ys) Ry.push_back(AxisRot(y, 1));
  for (double z : zs) Rz.push_back(AxisRot(z, 2));
  std::vector<Eigen::Matrix3d> out;
  out.reserve(xs.size() * ys.size() * zs.size());
  for (const auto& rz : Rz)
    for (const auto& ry : Ry)
      for (const auto& rx : Rx) out.push_back(rz * ry * rx);
  return out;
}

Eigen::Matrix3d ProjectToAnchor(const Eigen::Matrix3d& R_cand, const Eigen::Matrix3d& R_anc,
                                const Eigen::Vector3d& tol_deg) {
  Eigen::Vector3d rel = CtrlRpyDegFromRot(R_anc.transpose() * R_cand);
  rel[0] = Wrap180(rel[0]);
  rel[1] = Wrap180(rel[1]);
  rel[2] = Wrap180(rel[2]);
  Eigen::Vector3d clipped;
  for (int i = 0; i < 3; ++i) {
    const double t = std::abs(tol_deg[i]);
    clipped[i] = std::min(t, std::max(-t, rel[i]));
  }
  if ((clipped - rel).cwiseAbs().maxCoeff() <= 1e-12) return R_cand;
  return R_anc * RotFromCtrlRpyDeg(clipped);
}

std::optional<JointVec> UnwrapOnto(const JointVec& q_sol, const JointVec& q_ref,
                                   const Cr5Kinematics& kin) {
  const JointVec q_u = q_ref + WrapPi(q_sol - q_ref);
  if (kin.IsJointValid(q_u)) return q_u;
  if (kin.IsJointValid(q_sol)) return q_sol;
  return std::nullopt;
}

bool IsSafeQ(const Cr5Kinematics& kin, const JointVec& q, const Transform& T_gun,
             const ToolOffset& tool) {
  if (!kin.IsJointValid(q)) return false;
  if (std::abs(std::sin(q[4])) < kSingSin || std::abs(std::sin(q[2])) < kSingSin) return false;
  const Transform T_urdf = CtrlToUrdf(T_gun * tool.T_tcp_inv);
  return ShoulderHalfRad(T_urdf) >= kShoulderHalfRad;
}

std::vector<double> AlphasFor(int n_mid, std::unordered_map<int, std::vector<double>>& cache) {
  auto it = cache.find(n_mid);
  if (it != cache.end()) return it->second;
  std::vector<double> a;
  a.reserve(static_cast<size_t>(n_mid + 1));
  const int n = n_mid + 2;
  for (int i = 1; i < n; ++i) a.push_back(static_cast<double>(i) / static_cast<double>(n - 1));
  cache[n_mid] = a;
  return a;
}

struct QuatKey {
  long long x, y, z, w;
  bool operator==(const QuatKey& o) const {
    return x == o.x && y == o.y && z == o.z && w == o.w;
  }
};
struct QuatKeyHash {
  size_t operator()(const QuatKey& k) const {
    return static_cast<size_t>(k.x * 1315423911LL ^ k.y ^ (k.z << 7) ^ (k.w << 13));
  }
};
QuatKey MakeQuatKey(const Eigen::Vector4d& q) {
  Eigen::Vector4d qq = q[3] < 0.0 ? -q : q;
  auto rnd = [](double v) { return static_cast<long long>(std::llround(v * 1e5)); };
  return {rnd(qq[0]), rnd(qq[1]), rnd(qq[2]), rnd(qq[3])};
}

std::pair<std::vector<DpNode>, CandPack> GenerateCandidates(
    const Cr5Kinematics& kin, const ToolOffset& tool, const Transform& T_nom,
    const std::optional<Eigen::Matrix3d>& R_anchor, const Eigen::Vector3d& tol,
    const std::vector<Eigen::Matrix3d>& R_off, const OptimizeOptions& opt) {
  const Eigen::Vector3d pos = T_nom.translation();
  const Eigen::Matrix3d R_nom = T_nom.linear();
  const size_t N = R_off.size();
  std::vector<Eigen::Matrix3d> R_cands(N);
  for (size_t i = 0; i < N; ++i) {
    R_cands[i] = R_nom * R_off[i];
    if (R_anchor) R_cands[i] = ProjectToAnchor(R_cands[i], *R_anchor, tol);
  }

  std::vector<double> geo(N), zero(N);
  std::vector<Eigen::Vector4d> quats(N);
  // key → keep 中的槽位。Python 侧 dict.values() 是「首次插入序」，而 unordered_map
  // 的迭代序未定义；候选顺序会经 pose_idx 传导到后面 stable_sort 的平局判定，
  // 所以这里必须显式保留首次出现序，否则结果随 STL 实现漂移。
  std::unordered_map<QuatKey, size_t, QuatKeyHash> slot_of;
  std::vector<size_t> keep;
  for (size_t i = 0; i < N; ++i) {
    geo[i] = GeodesicDeg(R_nom, R_cands[i]);
    quats[i] = QuatXyzw(R_cands[i]);
    Eigen::Vector3d e = CtrlRpyDegFromRot(R_nom.transpose() * R_cands[i]);
    e[0] = Wrap180(e[0]);
    e[1] = Wrap180(e[1]);
    e[2] = Wrap180(e[2]);
    zero[i] = e.cwiseAbs2().dot(opt.weight_zero_dev);
    const QuatKey key = MakeQuatKey(quats[i]);
    auto it = slot_of.find(key);
    if (it == slot_of.end()) {
      slot_of.emplace(key, keep.size());
      keep.push_back(i);
    } else if (geo[i] < geo[keep[it->second]]) {
      keep[it->second] = i;  // 同姿态取测地距更小者，槽位（即顺序）不变
    }
  }
  const int P = static_cast<int>(keep.size());
  CandPack empty;
  if (P == 0) return {{}, empty};

  std::vector<Transform> T_ctrl(P), T_urdf(P);
  std::vector<double> geo_keep(P), zero_keep(P);
  for (int i = 0; i < P; ++i) {
    const size_t src = keep[static_cast<size_t>(i)];
    T_ctrl[i] = Transform::Identity();
    T_ctrl[i].linear() = R_cands[src];
    T_ctrl[i].translation() = pos;
    T_urdf[i] = CtrlToUrdf(T_ctrl[i] * tool.T_tcp_inv);
    geo_keep[i] = geo[src];
    zero_keep[i] = zero[src];
  }

  std::vector<JointVec> q_sols(static_cast<size_t>(P) * 8);
  std::vector<int> n_sols(P);
  kin.IkBatch(T_urdf.data(), P, q_sols.data(), n_sols.data());

  CandPack pack;
  std::vector<int> remap(P, -1);
  int kept = 0;
  for (int i = 0; i < P; ++i) {
    if (n_sols[i] <= 0) continue;
    if (ShoulderHalfRad(T_urdf[i]) < kShoulderHalfRad) continue;
    remap[i] = kept++;
    pack.T.push_back(T_ctrl[i]);
    pack.geo.push_back(geo_keep[i]);
    pack.zero.push_back(zero_keep[i]);
  }
  if (kept == 0) return {{}, empty};

  for (int i = 0; i < P; ++i) {
    if (remap[i] < 0) continue;
    for (int k = 0; k < n_sols[i] && k < 8; ++k) {
      const JointVec& q = q_sols[static_cast<size_t>(i) * 8 + k];
      if (!kin.IsJointValid(q)) continue;
      if (std::abs(std::sin(q[4])) < kSingSin || std::abs(std::sin(q[2])) < kSingSin) continue;
      pack.q.push_back(q);
      pack.pose_idx.push_back(remap[i]);
      pack.branch_id.push_back(BranchKey(q));
    }
  }
  if (pack.q.empty()) return {{}, empty};

  std::vector<std::vector<int>> buckets(8);
  for (size_t m = 0; m < pack.q.size(); ++m) {
    buckets[static_cast<size_t>(pack.branch_id[m])].push_back(static_cast<int>(m));
  }
  std::vector<DpNode> fast;
  for (int b = 0; b < 8; ++b) {
    auto& idx = buckets[static_cast<size_t>(b)];
    std::stable_sort(idx.begin(), idx.end(), [&](int a, int b2) {
      const double sa = PoseScore(pack, a);
      const double sb = PoseScore(pack, b2);
      if (sa < sb) return true;
      if (sa > sb) return false;
      return a < b2;
    });
    const int take = std::min(opt.max_candidates_per_branch, static_cast<int>(idx.size()));
    for (int t = 0; t < take; ++t) fast.push_back(MakeNode(pack, idx[static_cast<size_t>(t)]));
  }
  if (fast.empty()) fast = Materialize(pack);
  return {fast, pack};
}

struct EdgeOut {
  bool ok = false;
  double cost = std::numeric_limits<double>::infinity();
  JointVec q = JointVec::Zero();
};

EdgeOut CheckMoveL(const Cr5Kinematics& kin, const ToolOffset& tool, const OptimizeOptions& opt,
                   const DpNode& a, const DpNode& b, const JointVec& q_start, bool is_jump,
                   std::unordered_map<int, std::vector<double>>& alpha_cache) {
  EdgeOut out;
  if (is_jump) {
    auto hint = UnwrapOnto(b.q_branch, q_start, kin);
    if (!hint) hint = b.q_branch;
    if (!IsSafeQ(kin, *hint, b.T, tool)) return out;
    const JointVec dq = WrapPi(*hint - q_start);
    double cost = 0.0;
    for (int j = 0; j < 6; ++j) {
      const double d = Deg(dq[j]);
      cost += opt.joint_weights[j] * d * d;
    }
    out.ok = true;
    out.cost = cost;
    out.q = *hint;
    return out;
  }

  auto hint = UnwrapOnto(b.q_branch, q_start, kin);
  if (!hint) return out;
  if ((a.ew_family) != (b.ew_family)) return out;

  const Transform Tf_a = a.T * tool.T_tcp_inv;
  const Transform Tf_b = b.T * tool.T_tcp_inv;
  const Eigen::Vector3d p0 = Tf_a.translation();
  const Eigen::Vector3d p1 = Tf_b.translation();
  const double dist_mm = (p1 - p0).norm() * kMmPerM;
  double travel = 0.0;
  {
    const JointVec dq = WrapPi(*hint - q_start);
    for (int j = 0; j < 6; ++j) travel = std::max(travel, std::abs(Deg(dq[j])));
  }
  const double travel_lim =
      std::min(kSegmentTravelCapDeg, std::max(kSegmentTravelDeg, kSegmentTravelDegPerMm * dist_mm));
  if (travel > travel_lim) return out;

  const int n_est = static_cast<int>(std::lround(dist_mm / opt.movel_spacing_mm));
  const int n_mid = std::min(opt.movel_checks_max, std::max(opt.movel_checks_min, n_est));
  const auto alphas = AlphasFor(n_mid, alpha_cache);

  MoveLQuery q;
  q.p_start_m = p0;
  q.p_end_m = p1;
  q.quat1_xyzw = QuatXyzw(Tf_a.linear());
  q.quat2_xyzw = QuatXyzw(Tf_b.linear());
  q.q_start = q_start;
  q.alphas = alphas;
  q.q_branch_end = b.q_branch;
  q.check_end_branch = true;
  q.max_jump_rad = Rad(kBranchJumpDeg);
  q.match_rad = Rad(kEndBranchMatchDeg);
  q.weights = opt.joint_weights;
  auto walk = SegmentChecker(kin).Walk(q);
  if (!walk) return out;
  out.ok = true;
  out.cost = walk->cost;
  out.q = walk->q_end;
  return out;
}

void BeamKeep(std::vector<double>& cost, int beam) {
  std::vector<int> finite;
  for (int i = 0; i < static_cast<int>(cost.size()); ++i) {
    if (std::isfinite(cost[static_cast<size_t>(i)])) finite.push_back(i);
  }
  if (static_cast<int>(finite.size()) <= beam) return;
  std::sort(finite.begin(), finite.end(),
            [&](int a, int b) { return cost[static_cast<size_t>(a)] < cost[static_cast<size_t>(b)]; });
  for (size_t i = static_cast<size_t>(beam); i < finite.size(); ++i) {
    cost[static_cast<size_t>(finite[i])] = std::numeric_limits<double>::infinity();
  }
}

}  // namespace

std::string OptimizeOptions::Validate() const {
  if (beam_width < 8) return "beam_width must be >= 8";
  if (max_candidates_per_branch < 1) return "max_candidates_per_branch must be >= 1";
  if (movel_spacing_mm <= 0.0) return "movel_spacing_mm must be > 0";
  if (movel_checks_min < 1 || movel_checks_max < movel_checks_min) {
    return "movel_checks_min/max must satisfy 1 <= min <= max";
  }
  return {};
}

ViterbiOptimizer::ViterbiOptimizer(const Cr5Kinematics& kin, const ToolOffset& tool,
                                   OptimizeOptions opt, const ChainVerifier* verifier)
    : kin_(kin), tool_(tool), opt_(std::move(opt)), verifier_(verifier) {}

OptimizeResult ViterbiOptimizer::Optimize(const PathItem& path, const Anchor& anchor,
                                          std::optional<JointVec> init_q) const {
  OptimizeResult result;
  result.path = path;
  if (path.points.empty()) return result;

  const auto t0 = std::chrono::steady_clock::now();
  const JointVec q_seed = init_q.value_or(DefaultSeed());
  const auto R_off = PrecomputeOffsets(opt_.grid_x, opt_.grid_y, opt_.grid_z);
  const int n = static_cast<int>(path.points.size());

  const auto t_cands0 = std::chrono::steady_clock::now();
  std::vector<std::vector<DpNode>> stages(n);
  std::vector<CandPack> packs(n);
  for (int i = 0; i < n; ++i) {
    std::optional<Eigen::Matrix3d> R_a;
    if (anchor.has_global()) {
      R_a = *anchor.R;
    } else if (anchor.tol_deg.norm() > 0.0) {
      R_a = path.points[static_cast<size_t>(i)].tcp_pose.linear();
    }
    auto [fast, pack] = GenerateCandidates(kin_, tool_, path.points[static_cast<size_t>(i)].tcp_pose,
                                           R_a, anchor.tol_deg, R_off, opt_);
    if (fast.empty() && pack.q.empty()) {
      throw std::runtime_error("Waypoint [" + std::to_string(i) +
                               "] has no non-singular in-limit IK inside the attitude envelope.");
    }
    stages[static_cast<size_t>(i)] = std::move(fast);
    packs[static_cast<size_t>(i)] = std::move(pack);
  }
  {
    int total_fast = 0, total_full = 0;
    for (int i = 0; i < n; ++i) {
      total_fast += static_cast<int>(stages[static_cast<size_t>(i)].size());
      total_full += static_cast<int>(packs[static_cast<size_t>(i)].q.size());
    }
    const double t_cands_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_cands0).count();
    std::cerr << "⏱️ [SprayOpt] Stage candidates generated: " << n << " waypoints, fast_candidates="
              << total_fast << " (diverse 8-branch pruned), full_nodes=" << total_full
              << ", elapsed=" << std::fixed << std::setprecision(2) << t_cands_ms << " ms\n"
              << std::flush;
  }

  if (n == 1) {
    auto& s0 = stages[0];
    int best = 0;
    double best_c = std::numeric_limits<double>::infinity();
    for (int j = 0; j < static_cast<int>(s0.size()); ++j) {
      const double c =
          s0[static_cast<size_t>(j)].zero_dev + WrapPi(s0[static_cast<size_t>(j)].q - q_seed).squaredNorm();
      if (c < best_c) {
        best_c = c;
        best = j;
      }
    }
    const DpNode& picked = s0[static_cast<size_t>(best)];
    result.modified = !picked.T.linear().isApprox(path.points[0].tcp_pose.linear(), 1e-6);
    result.path.points[0].tcp_pose = picked.T;
    result.joints_rad.push_back(picked.q);
    return result;
  }

  const auto t_dp0 = std::chrono::steady_clock::now();
  std::vector<std::vector<double>> dp_cost(n);
  std::vector<std::vector<int>> dp_parent(n);
  std::vector<std::vector<JointVec>> dp_q(n);
  for (int i = 0; i < n; ++i) {
    const size_t m = stages[static_cast<size_t>(i)].size();
    dp_cost[static_cast<size_t>(i)].assign(m, std::numeric_limits<double>::infinity());
    dp_parent[static_cast<size_t>(i)].assign(m, -1);
    dp_q[static_cast<size_t>(i)].assign(m, JointVec::Zero());
  }

  for (int j = 0; j < static_cast<int>(stages[0].size()); ++j) {
    auto& node = stages[0][static_cast<size_t>(j)];
    auto q0 = UnwrapOnto(node.q, q_seed, kin_);
    if (!q0 || !IsSafeQ(kin_, *q0, node.T, tool_)) continue;
    const JointVec d = WrapPi(*q0 - q_seed);
    double extra = 0.0;
    for (int k = 0; k < 6; ++k) extra += opt_.joint_weights[k] * Deg(d[k]) * Deg(d[k]);
    dp_cost[0][static_cast<size_t>(j)] = node.zero_dev + 0.05 * extra;
    dp_q[0][static_cast<size_t>(j)] = *q0;
    node.q = *q0;
  }
  BeamKeep(dp_cost[0], opt_.beam_width);

  std::unordered_map<int, std::vector<double>> alpha_cache;
  struct LayerOut {
    std::vector<double> cost;
    std::vector<int> parent;
    std::vector<JointVec> qq;
    int tested = 0;
    int valid = 0;
  };
  auto eval_layer = [&](int i, std::vector<DpNode>& curr) -> LayerOut {
    LayerOut out;
    const size_t m = curr.size();
    out.cost.assign(m, std::numeric_limits<double>::infinity());
    out.parent.assign(m, -1);
    out.qq.assign(m, JointVec::Zero());
    const bool is_jump = path.points[static_cast<size_t>(i)].is_jump ||
                         !path.points[static_cast<size_t>(i)].spraying;
    for (int pk = 0; pk < static_cast<int>(stages[static_cast<size_t>(i - 1)].size()); ++pk) {
      if (!std::isfinite(dp_cost[static_cast<size_t>(i - 1)][static_cast<size_t>(pk)])) continue;
      const JointVec& q_prev = dp_q[static_cast<size_t>(i - 1)][static_cast<size_t>(pk)];
      const auto& prev = stages[static_cast<size_t>(i - 1)][static_cast<size_t>(pk)];
      for (int cj = 0; cj < static_cast<int>(curr.size()); ++cj) {
        ++out.tested;
        const EdgeOut e = CheckMoveL(kin_, tool_, opt_, prev, curr[static_cast<size_t>(cj)], q_prev,
                                     is_jump, alpha_cache);
        if (!e.ok) continue;
        ++out.valid;
        const double total =
            dp_cost[static_cast<size_t>(i - 1)][static_cast<size_t>(pk)] + e.cost +
            curr[static_cast<size_t>(cj)].zero_dev;
        if (total < out.cost[static_cast<size_t>(cj)]) {
          out.cost[static_cast<size_t>(cj)] = total;
          out.parent[static_cast<size_t>(cj)] = pk;
          out.qq[static_cast<size_t>(cj)] = e.q;
        }
      }
    }
    return out;
  };

  for (int i = 1; i < n; ++i) {
    const auto t_seg0 = std::chrono::steady_clock::now();
    int edges_tested = 0;
    int valid_edges = 0;
    LayerOut layer = eval_layer(i, stages[static_cast<size_t>(i)]);
    edges_tested += layer.tested;
    valid_edges += layer.valid;
    bool any = false;
    for (double v : layer.cost)
      if (std::isfinite(v)) any = true;
    if (!any && static_cast<int>(packs[static_cast<size_t>(i)].q.size()) >
                    static_cast<int>(stages[static_cast<size_t>(i)].size())) {
      const int full_n = static_cast<int>(packs[static_cast<size_t>(i)].q.size());
      std::cerr << "⚠️ [SprayOpt] Segment " << (i - 1) << "->" << i << " failed with pruned candidates ("
                << stages[static_cast<size_t>(i)].size() << "). Triggering adaptive fallback with ALL "
                << full_n << " candidates...\n"
                << std::flush;
      stages[static_cast<size_t>(i)] = Materialize(packs[static_cast<size_t>(i)]);
      layer = eval_layer(i, stages[static_cast<size_t>(i)]);
      edges_tested += layer.tested;
      valid_edges += layer.valid;
      any = false;
      for (double v : layer.cost)
        if (std::isfinite(v)) any = true;
    }
    BeamKeep(layer.cost, opt_.beam_width);
    const double t_seg_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_seg0).count();
    std::cerr << "⏱️ [SprayOpt] DP Segment " << (i - 1) << "->" << i << ": edges=" << edges_tested
              << " (feasible=" << valid_edges << "), elapsed=" << std::fixed << std::setprecision(2)
              << t_seg_ms << " ms\n"
              << std::flush;
    if (!any) {
      throw std::runtime_error("Global search failed at segment " + std::to_string(i - 1) + "->" +
                               std::to_string(i));
    }
    dp_cost[static_cast<size_t>(i)] = std::move(layer.cost);
    dp_parent[static_cast<size_t>(i)] = std::move(layer.parent);
    dp_q[static_cast<size_t>(i)] = std::move(layer.qq);
  }
  {
    const double t_dp_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_dp0).count();
    std::cerr << "⏱️ [SprayOpt] Total Viterbi DP search: " << (n - 1)
              << " segments, elapsed=" << std::fixed << std::setprecision(2) << t_dp_ms << " ms\n"
              << std::flush;
  }

  int last = 0;
  double best = std::numeric_limits<double>::infinity();
  for (int j = 0; j < static_cast<int>(dp_cost.back().size()); ++j) {
    if (dp_cost.back()[static_cast<size_t>(j)] < best) {
      best = dp_cost.back()[static_cast<size_t>(j)];
      last = j;
    }
  }
  std::vector<int> chain(n);
  chain[static_cast<size_t>(n - 1)] = last;
  for (int i = n - 1; i > 0; --i) {
    last = dp_parent[static_cast<size_t>(i)][static_cast<size_t>(last)];
    // 代价有限的节点必然有父节点；若不变式被破坏，宁可报错也不要负索引越界。
    if (last < 0) {
      throw std::runtime_error("Backtrack broken at waypoint " + std::to_string(i) +
                               " (no parent for a finite-cost node).");
    }
    chain[static_cast<size_t>(i - 1)] = last;
  }

  result.joints_rad.resize(n);
  bool modified = false;
  for (int i = 0; i < n; ++i) {
    const auto& nd = stages[static_cast<size_t>(i)][static_cast<size_t>(chain[static_cast<size_t>(i)])];
    if (!nd.T.linear().isApprox(path.points[static_cast<size_t>(i)].tcp_pose.linear(), 1e-6)) {
      modified = true;
    }
    result.path.points[static_cast<size_t>(i)].tcp_pose = nd.T;
    result.joints_rad[static_cast<size_t>(i)] = dp_q[static_cast<size_t>(i)][static_cast<size_t>(chain[static_cast<size_t>(i)])];
  }
  result.modified = modified;

  if (verifier_ && opt_.dense_verify && n >= 2) {
    result.verify = verifier_->Verify(result.path, q_seed);
    bool hard = result.verify.status == "FAILED";
    for (const auto& iss : result.verify.issues) {
      if (iss.severity == "ERROR" || iss.type.find("SINGULARITY") != std::string::npos) hard = true;
    }
    if (hard) {
      throw std::runtime_error("Dense MoveL verifier rejected the DP path.");
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return result;
}

Anchor ResolveAnchor(const AnchorSpec& spec, const Cr5Kinematics& kin, const PathItem& path) {
  Anchor a;
  a.tol_deg = spec.tol_deg;
  const std::string src = spec.source;
  if (src == "raw") {
    a.R = std::nullopt;
    return a;
  }
  if (src == "home") {
    Eigen::Vector3d xyz, rpy;
    kin.FkController(spec.home_joints_rad, xyz, rpy);
    a.R = RotFromCtrlRpyDeg(rpy);
    return a;
  }
  // config / live：RPY 已由调用方解析好
  a.R = RotFromCtrlRpyDeg(spec.ref_rpy_deg);
  (void)path;
  return a;
}

}  // namespace motion
