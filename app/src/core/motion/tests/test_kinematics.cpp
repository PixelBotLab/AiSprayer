#include "motion/kinematics.hpp"

#include <cmath>
#include <iostream>

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
  // 欧拉往返
  for (double rx = -170; rx <= 170; rx += 40) {
    for (double ry = -80; ry <= 80; ry += 40) {
      for (double rz = -170; rz <= 170; rz += 40) {
        const Eigen::Vector3d in(rx, ry, rz);
        const Eigen::Vector3d out = CtrlRpyDegFromRot(RotFromCtrlRpyDeg(in));
        CHECK((out - in).cwiseAbs().maxCoeff() < 1e-9);
      }
    }
  }

  Cr5Kinematics kin;
  const JointVec home =
      (JointVec() << 0, 0, -kPi / 2, -kPi / 2, -kPi / 2, 0).finished();
  const Transform T = kin.Fk(home);
  JointVec sols[8];
  const int n = kin.Ik(T, sols);
  CHECK(n > 0);
  bool found = false;
  for (int i = 0; i < n; ++i) {
    const Transform T2 = kin.Fk(sols[i]);
    if ((T2.translation() - T.translation()).norm() < 1e-9 &&
        GeodesicDeg(T2.linear(), T.linear()) < 1e-6) {
      found = true;
    }
  }
  CHECK(found);

  auto best = kin.BestIk(T, home);
  CHECK(best.has_value());
  CHECK((*best - home).cwiseAbs().maxCoeff() < 1e-6);

  Eigen::Vector3d xyz, rpy;
  kin.FkController(home, xyz, rpy);
  CHECK(std::isfinite(xyz[0]) && std::isfinite(rpy[0]));

  if (g_fail) {
    std::cerr << g_fail << " checks failed\n";
    return 1;
  }
  std::cout << "test_kinematics OK\n";
  return 0;
}
