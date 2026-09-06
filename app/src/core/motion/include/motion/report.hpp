#pragma once

#include "motion/conventions.hpp"
#include "motion/robot_model.hpp"
#include "motion/types.hpp"

#include <array>
#include <string>
#include <vector>

namespace motion {

struct Issue {
  std::string type;
  std::string severity;  // ERROR | WARNING
  int segment_index = 0;
  int step_index = 0;
  std::string detail;
  Eigen::Vector3d location_xyz_mm{0, 0, 0};
};

struct PathVerifyReport {
  int path_id = 0;
  std::string name;
  std::string status = "PASS";  // PASS | WARNING | FAILED
  int total_interpolated = 0;
  double speed_mm_s = 120.0;
  double step_size_mm = 1.5;
  double recommended_safe_speed_mm_s = 120.0;
  std::array<double, 6> max_joint_velocities_deg_s{{180, 180, 180, 180, 180, 180}};
  std::array<double, 6> peak_joint_speeds_deg_s{{0, 0, 0, 0, 0, 0}};
  std::vector<Issue> issues;
  std::vector<JointVec> trajectory_q;
  std::vector<std::array<double, 6>> trajectory_tcp;  // x,y,z mm + rx,ry,rz deg
};

struct VerifySummary {
  std::string status = "PASS";
  int total_paths = 0;
  int total_waypoints = 0;
  int total_steps = 0;
  int total_issues = 0;
  int singularity_count = 0;
  int overspeed_count = 0;
  int unreachable_count = 0;
};

struct VerifyReport {
  VerifySummary summary;
  double nominal_speed_mm_s = 120.0;
  double slerp_step_mm = 1.5;
  std::array<double, 6> max_joint_velocities_deg_s{{180, 180, 180, 180, 180, 180}};
  ToolOffset urdf_tcp;
  std::vector<PathVerifyReport> path_reports;
};

struct VerifyOptions {
  double step_mm = 1.5;
  double speed_mm_s = 120.0;
};

// 容差阶梯中的一档：一次完整 DP + 密集校验的结果摘要。
// 只用于日志/报表/择优，不参与轨迹数据本身。
struct LadderRung {
  Eigen::Vector3d tol_deg{0.0, 0.0, 0.0};  // 该档实际使用的锚点包络
  // PASS | WARNING | FAILED | ERROR(该档抛异常) | UNVERIFIED(关闭了密集校验)
  std::string status = "ERROR";
  double peak_deg_s = 0.0;   // max_j 峰值关节角速度 (deg/s)
  double peak_ratio = 0.0;   // max_j peak_j / limit_j；无密集校验时为 0（不参与早停）
  double max_pointing_deg = 0.0;  // 优化后枪尖法向相对原始法向的最大偏量 (deg)
  double objective = 0.0;    // DP 总代价 J
  double elapsed_ms = 0.0;
  std::string error;         // status==ERROR 时的异常信息
};

struct OptimizeOptions {
  AxisGrid grid_x{-5, 5, 2};
  AxisGrid grid_y{-5, 5, 2};
  AxisGrid grid_z{-30, 30, 5};  // 与 aisprayer_config.yaml spraying.grid_tol_z_deg 一致
  int beam_width = 32;
  int max_candidates_per_branch = 16;
  int movel_checks_min = 10;
  int movel_checks_max = 100;
  double movel_spacing_mm = 5.0;
  // 候选姿态相对**该航点名义(法向)姿态**的偏离惩罚权重（Rx/Ry 为倾角、Rz 为绕枪轴自旋），
  // 不是「离机械臂零位的远近」；自旋权重刻意压到 0.01，因为圆喷嘴绕轴自旋不影响漆雾。
  Eigen::Vector3d weight_zero_dev{1.0, 1.0, 0.01};
  JointVec joint_weights = (JointVec() << 1.0, 1.2, 1.0, 0.8, 0.8, 0.5).finished();
  // 密集 MoveL 复核开关；采样步长/速度由传入的 ChainVerifier 自带 VerifyOptions 决定。
  bool dense_verify = true;

  // ── 容差阶梯择优（Monotonicity Guard）──────────────────────────────────────
  // 大容差包络在几何上包含小容差包络，理论上最优解不应变差；但 DP 的目标
  // J = Σ Δq² + 姿态偏置 **不含峰值角速度**，且候选集与 beam 剪枝都随容差变化，
  // 实测「放大容差」会让峰值明显变差（见 docs/optimizer_monotonicity_improvement_proposal.md
  // §5–§6：同一工件 [10,10,50] → 43.6°/s，[30,30,180] → 133.8°/s）。
  // 这里用「多档包络各跑一次 + 按与容差无关的标尺择优」把包含关系变成构造性保证：
  // 每一档的解都落在用户请求的包络内，密集校验又与容差无关，所以返回的解
  // 不会比阶梯里任何一档差。代价是最多多跑几档（单档 ~1.5 s），由早停阈值收敛。
  bool tol_ladder = true;
  // 相对请求包络的收紧比例（逐分量乘）；请求档(1.0)总是第一档，不在此列出。
  std::vector<double> tol_ladder_scales{0.5, 1.0 / 3.0, 0.25};
  // 早停：某档 PASS 且峰值 ≤ 该比例 × 关节限速时不再继续收紧（0 表示跑完全部档位）。
  double tol_ladder_stop_peak_ratio = 0.3;
  // 指向偏量护栏：收紧包络会牺牲法向跟随（实测峰值 133.8→44 伴随指向偏量 18.7°→45.9°）。
  // >0 时，某档的最大指向偏量超过请求档 + 该值就弃用该档；0 = 不限制（默认，只看运动学质量）。
  double tol_ladder_max_pointing_deg = 0.0;

  std::string Validate() const;
};

struct OptimizeResult {
  PathItem path;
  bool modified = false;
  std::vector<JointVec> joints_rad;
  PathVerifyReport verify;
  double elapsed_ms = 0.0;
  double objective = 0.0;  // DP 回溯终点的累计代价 J（诊断与择优用）
  // 采纳解实际使用的包络：阶梯择优后可能比请求的更紧，报表/落盘必须回显它。
  Eigen::Vector3d adopted_tol_deg{0.0, 0.0, 0.0};
  std::vector<LadderRung> ladder;  // 各档摘要；未启用阶梯时为空
};

}  // namespace motion
