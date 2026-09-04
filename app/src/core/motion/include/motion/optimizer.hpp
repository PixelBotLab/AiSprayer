#pragma once

#include "motion/kinematics.hpp"
#include "motion/report.hpp"
#include "motion/segment.hpp"
#include "motion/types.hpp"
#include "motion/verifier.hpp"

namespace motion {

class ViterbiOptimizer {
 public:
  ViterbiOptimizer(const Cr5Kinematics& kin, const ToolOffset& tool,
                   OptimizeOptions opt, const ChainVerifier* verifier = nullptr);

  OptimizeResult Optimize(const PathItem& path, const Anchor& anchor,
                          std::optional<JointVec> init_q = std::nullopt) const;

 private:
  const Cr5Kinematics& kin_;
  const ToolOffset& tool_;
  OptimizeOptions opt_;
  const ChainVerifier* verifier_;
};

Anchor ResolveAnchor(const AnchorSpec& spec, const Cr5Kinematics& kin,
                     const PathItem& path);

}  // namespace motion
