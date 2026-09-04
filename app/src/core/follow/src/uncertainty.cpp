#include "follow/uncertainty.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

namespace follow {

namespace {
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

// 求解失败时 σ 必须是 +inf：留在默认值 0 上，直接读 σ 的调用方会看到"完美精度"。
Uncertainty failed_uncertainty() {
  Uncertainty u;
  u.solver_failed = true;
  for (int i = 0; i < 3; ++i) {
    u.rot_sigma_deg[i] = std::numeric_limits<double>::infinity();
    u.trans_sigma_mm[i] = std::numeric_limits<double>::infinity();
  }
  return u;
}
}  // namespace

Uncertainty hessian_uncertainty(const Eigen::Matrix<double, 6, 6>& H, double residual_var_scale,
                                double rel_eig_floor) {
  // scale 必须是正的有限数。s² 退化到 0 意味着"两张点云完全重合"，此时 σ=0 会被上层读成
  // "这一维量得极准"，而真相是"没有任何信息可用于估计不确定度"。宁可报求解失败。
  if (!H.allFinite() || !(residual_var_scale > 0.0) || !std::isfinite(residual_var_scale)) {
    return failed_uncertainty();
  }
  Uncertainty u;
  u.residual_var_scale = residual_var_scale;

  const Eigen::Matrix<double, 6, 6> Hs = 0.5 * (H + H.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(Hs);
  if (es.info() != Eigen::Success) {
    return failed_uncertainty();
  }
  u.eigenvalues = es.eigenvalues();

  const double lmax = std::max(u.eigenvalues.maxCoeff(), 0.0);
  const double floor_v = rel_eig_floor * lmax;
  if (u.eigenvalues.minCoeff() <= floor_v) {
    u.rank_deficient = true;
  }

  // H^+ = V diag(1/max(λ, floor)) V^T，协方差 = s² · H^+
  Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();
  for (int i = 0; i < 6; ++i) {
    const double l = std::max(u.eigenvalues[i], floor_v);
    const double w = l > 0.0 ? 1.0 / l : std::numeric_limits<double>::infinity();
    C += w * es.eigenvectors().col(i) * es.eigenvectors().col(i).transpose();
  }

  for (int i = 0; i < 3; ++i) {
    const double vr = std::max(0.0, C(i, i));
    const double vt = std::max(0.0, C(i + 3, i + 3));
    u.rot_sigma_deg[i] = std::sqrt(residual_var_scale * vr) * kRad2Deg;
    u.trans_sigma_mm[i] = std::sqrt(residual_var_scale * vt) * 1000.0;
  }
  return u;
}

}  // namespace follow
