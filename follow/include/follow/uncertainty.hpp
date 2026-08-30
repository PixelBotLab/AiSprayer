// 稠密配准结果的可观测性判据。
//
// 报的是**每个自由度各自的 1σ**（毫米 / 度），门限因此是物理量。这里有两件事分开做：
//
// 一、尺度。GICP 的信息矩阵 H 加权用的是 (C_target + C_source)⁻¹，而那两个 C 是体素内的
//     几何展宽而不是传感器噪声，所以 H⁺ 直接开根号会偏大两个数量级，必须乘上残差自估的
//     s²（见 hessian_uncertainty）。乘完之后 σ 和真实散布对得上（1.0~1.6 倍内）。
//
// 二、判据。**不用跨组的 λ_min/λ_max**：6x6 信息矩阵里旋转列和平移列量纲不同
//     （J_rot = R·skew(p) 单位 m/rad，J_trans = -R 无量纲），比值随场景尺度漂 —— 实测同
//     一面大墙给出 6.7e-4，而原门限 1e-3，稍有多余结构就翻过去。但**组内**的 σ 比值没有这
//     个问题（三个平移同为 mm，三个转角同为 deg，量纲自己抵消），而且对 s² 完全免疫 —— 这
//     正是它必须存在的理由，见 Uncertainty::trans_anisotropy。
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>

namespace follow {

struct Uncertainty {
  // 顺序遵循 small_gicp 的切空间约定：[rx, ry, rz, tx, ty, tz]（旋转在前）。
  double rot_sigma_deg[3] = {0, 0, 0};
  double trans_sigma_mm[3] = {0, 0, 0};
  Eigen::Matrix<double, 6, 1> eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
  bool solver_failed = false;   // SelfAdjointEigenSolver 没收敛
  bool rank_deficient = false;  // 有方向落在数值零上
  // 实际乘上的 s²。正常工位在 1e-5 ~ 1e-4 量级；贴着调用方给的下限说明残差被钳过
  // （两张几乎相同的点云），这时 σ 偏小的方向要按不可信处理。
  double residual_var_scale = 1.0;

  // 组内各向异性 = 最差方向 / 最好方向的 σ 比。**乘 s² 不改变它** —— 这是它存在的全部理由：
  // 单一大平面上不可观方向的残差恒等于零，s² 因此塌到 6e-6（带肋工件 9e-5），于是那几维的
  // 绝对 σ 反而变得比好场景更小（实测 σ_t=[0.063 0.063 0.002] mm 而真实误差 9.99 mm）。
  // 只有比值不会被这个机制骗过去：工件 ~10/7，大平面 32/27。数由
  // test_registration 的 AbsoluteSigmaAloneWouldPassThePlane 打印并断言。
  double trans_anisotropy() const { return group_anisotropy(trans_sigma_mm); }
  double rot_anisotropy() const { return group_anisotropy(rot_sigma_deg); }

  // 三个条件全过才算"六个自由度都被测到了"：
  //  1) 绝对 σ 门（物理量，喷涂公差级别）—— 挡"整帧没几条有效约束"的烂解；
  //  2) 各向异性门 —— 挡被 s² 洗白的退化方向；
  //  3) rank_deficient —— 挡数值上严格奇异的情况。
  bool within(double trans_limit_mm, double rot_limit_deg, double aniso_limit) const {
    if (solver_failed || rank_deficient) {
      return false;
    }
    for (double s : trans_sigma_mm) {
      if (!(s <= trans_limit_mm)) {
        return false;
      }
    }
    for (double s : rot_sigma_deg) {
      if (!(s <= rot_limit_deg)) {
        return false;
      }
    }
    return !(trans_anisotropy() > aniso_limit) && !(rot_anisotropy() > aniso_limit);
  }

 private:
  static double group_anisotropy(const double* v) {
    const double lo = std::min(std::min(v[0], v[1]), v[2]);
    const double hi = std::max(std::max(v[0], v[1]), v[2]);
    if (!(lo > 0.0) || !std::isfinite(hi)) {
      return std::numeric_limits<double>::infinity();
    }
    return hi / lo;
  }
};

// rel_eig_floor：小于 rel_eig_floor * λ_max 的特征值按零处理（伪逆里取倒数前钳住），
// 使不可观方向表现为"很大的 σ"而不是 inf。
//
// residual_var_scale = s²：协方差取 s²·H⁺，不是 H⁺。
// 为什么必须乘：GICP 的权重是 (C_target + C_source)⁻¹，而那两个 C 是**体素内的几何展宽**
// （毫米量级，说的是"这一小块面有多平"），不是深度噪声。H⁺ 只有在每条对应点的残差协方差
// 恰好等于那个 C 时才是协方差；实际残差是零点几毫米的传感器噪声，差两个数量级 ⇒ 直接对 H⁺
// 开根号得到的 σ 系统性偏大 ~190 倍。用残差自估 s² = 2·cost/(3·N)（模型正确时 ≈ 1）补尺度。
// 实测（合成工件 + 0.5 mm 深度噪声，16 次独立噪声实现，命令运动 mix(10,-20,30) mm，由
// test_registration 的 NoiseSdMatchesPredictedSigma 打印）：
//   H⁺ 直接开根号   σ_t = [13.5  17.2   1.8 ] mm      ← 旧数据，同一个场景
//   乘 sqrt(s²)     σ_t = [0.151 0.185 0.018] mm
//   蒙特卡洛真散布  sd  = [0.132 0.154 0.017] mm
// 各向异性（哪个方向弱）一直是对的，缺的只是尺度 —— 但**绝对 mm 门不能用来判退化**：
// 不可观方向的残差恒为零，那里的 s² 会塌得比好场景还狠（大平面 6.4e-6 vs 工件 9.0e-5），
// 乘完之后大平面的 σ_t=[0.063 0.063 0.002] mm 比好场景还小，而真实误差是 9.99 mm。
// 退化由组内比值判，见 Uncertainty::trans_anisotropy。
// s² ≤ 0 或非有限时按求解失败处理（全 +inf）—— 返回一个"0 不确定度"比返回 inf 危险得多。
Uncertainty hessian_uncertainty(const Eigen::Matrix<double, 6, 6>& H,
                                double residual_var_scale = 1.0, double rel_eig_floor = 1e-9);

}  // namespace follow
