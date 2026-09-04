#pragma once

#include "motion/kinematics.hpp"
#include "motion/optimizer.hpp"
#include "motion/report.hpp"
#include "motion/types.hpp"

#include <iosfwd>
#include <string>

namespace motion {

// 与 Python poi_optimizer._format_comparison_report / path_opt_cli 对齐的过程与结果表。
// 全部写 stderr，避免污染 stdout 的 JSON。
void PrintOptimizePreamble(std::ostream& os, const std::string& input, const PathItem& path,
                           const Cr5Kinematics& kin, const AnchorSpec& spec, const Anchor& anchor,
                           const OptimizeOptions& opt, double speed_mm_s, double step_mm);

void PrintOptimizeReport(std::ostream& os, const Cr5Kinematics& kin, const PathItem& raw,
                         const OptimizeResult& result, const Anchor& anchor,
                         const Eigen::Vector3d& tol_deg, const JointVec& home_rad,
                         const PathVerifyReport* verify, const std::string& output_path);

void PrintVerifyReport(std::ostream& os, const VerifyReport& report, double elapsed_ms);

}  // namespace motion
