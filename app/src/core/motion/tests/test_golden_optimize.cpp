#include "motion/io.hpp"
#include "motion/kinematics.hpp"
#include "motion/optimizer.hpp"
#include "motion/robot_model.hpp"
#include "motion/verifier.hpp"

#include <cmath>
#include <iostream>
#include <string>

#ifndef REPO_ROOT
#define REPO_ROOT "."
#endif

using namespace motion;

static int g_fail = 0;
#define CHECK(cond)                                                                    \
  do {                                                                                 \
    if (!(cond)) {                                                                     \
      std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << " " << #cond << "\n";     \
      ++g_fail;                                                                        \
    }                                                                                  \
  } while (0)

int main() {
  const std::string root = REPO_ROOT;
  const std::string urdf = root + "/app/urdf/cr5_robot_with_my_tools.urdf";
  const std::string auto_yaml = root + "/data/template_group/2026-09-03_225937/scan.auto.path.yaml";
  const std::string poi_yaml = root + "/data/template_group/2026-09-03_225937/scan.auto.poi.path.yaml";

  RobotModel model;
  std::string err;
  if (!LoadRobotModelFromUrdf(urdf, "gripper_tip_link", model, &err)) {
    std::cerr << err << "\n";
    return 1;
  }
  PathDocument raw, poi;
  if (!LoadPathYaml(auto_yaml, raw, &err) || !LoadPathYaml(poi_yaml, poi, &err)) {
    std::cerr << err << "\n";
    return 1;
  }
  CHECK(raw.paths.size() == 1 && poi.paths.size() == 1);
  CHECK(raw.paths[0].points.size() == 81);
  CHECK(poi.paths[0].points.size() == 81);

  Cr5Kinematics kin(model.limits);
  OptimizeOptions oopt;
  oopt.grid_x = {-5, 5, 2};
  oopt.grid_y = {-5, 5, 2};
  oopt.grid_z = {-30, 30, 5};
  oopt.dense_verify = true;
  CHECK(oopt.Validate().empty());

  VerifyOptions vopt;
  vopt.step_mm = 1.5;
  vopt.speed_mm_s = 120.0;
  ChainVerifier verifier(kin, model.tool, vopt);
  ViterbiOptimizer optimizer(kin, model.tool, oopt, &verifier);

  AnchorSpec spec;
  spec.source = "config";
  spec.ref_rpy_deg = {90.0, 0.0, 90.0};
  spec.tol_deg = {10.0, 10.0, 30.0};
  const Anchor anchor = ResolveAnchor(spec, kin, raw.paths[0]);

  const OptimizeResult got = optimizer.Optimize(raw.paths[0], anchor);
  CHECK(got.path.points.size() == 81);
  CHECK(got.verify.status == "PASS" || got.verify.issues.empty());

  double max_geo = 0.0, max_pos_mm = 0.0;
  int mismatch = 0;
  for (size_t i = 0; i < got.path.points.size(); ++i) {
    const auto& a = got.path.points[i].tcp_pose;
    const auto& b = poi.paths[0].points[i].tcp_pose;
    const double dpos = (a.translation() - b.translation()).norm() * kMmPerM;
    const double geo = GeodesicDeg(a.linear(), b.linear());
    max_pos_mm = std::max(max_pos_mm, dpos);
    max_geo = std::max(max_geo, geo);
    if (dpos > 0.05 || geo > 0.05) ++mismatch;
  }
  std::cout << "optimize vs poi: max_pos_mm=" << max_pos_mm << " max_geo_deg=" << max_geo
            << " mismatch=" << mismatch << "/" << got.path.points.size()
            << " elapsed_ms=" << got.elapsed_ms << "\n";
  CHECK(max_pos_mm < 0.05);
  CHECK(max_geo < 0.05);
  CHECK(mismatch == 0);

  if (g_fail) {
    std::cerr << g_fail << " golden optimize checks failed\n";
    return 1;
  }
  std::cout << "test_golden_optimize OK\n";
  return 0;
}
