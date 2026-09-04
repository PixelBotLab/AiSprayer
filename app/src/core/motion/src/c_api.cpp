#include "motion/kinematics.hpp"
#include "motion/segment.hpp"

#include <cstring>

using motion::Cr5Kinematics;
using motion::JointVec;
using motion::RobotLimits;
using motion::SegmentChecker;

extern "C" {

void c_ur_forward(const double* q, double* T) { Cr5Kinematics::DhFk(q, T); }

int c_ur_inverse(const double* T, double* q_sols, double q6_des) {
  return Cr5Kinematics::DhIk(T, q_sols, q6_des);
}

void c_forward(const double* q, double* T) { Cr5Kinematics::FkRaw(q, T); }

int c_inverse(const double* T, double* q_sols) { return Cr5Kinematics::IkRaw(T, q_sols); }

void c_compute_fk(const double* j, double* eetrans, double* eerot) {
  double T[16];
  Cr5Kinematics::FkRaw(j, T);
  for (int i = 0; i < 3; ++i) {
    eetrans[i] = T[i * 4 + 3];
    eerot[i * 3 + 0] = T[i * 4 + 0];
    eerot[i * 3 + 1] = T[i * 4 + 1];
    eerot[i * 3 + 2] = T[i * 4 + 2];
  }
}

int c_compute_ik(const double* eetrans, const double* eerot, double* q_sols) {
  double T[16];
  for (int i = 0; i < 3; ++i) {
    T[i * 4 + 3] = eetrans[i];
    T[i * 4 + 0] = eerot[i * 3 + 0];
    T[i * 4 + 1] = eerot[i * 3 + 1];
    T[i * 4 + 2] = eerot[i * 3 + 2];
  }
  T[12] = 0;
  T[13] = 0;
  T[14] = 0;
  T[15] = 1;
  return Cr5Kinematics::IkRaw(T, q_sols);
}

void c_forward_controller(const double* j, double* xyz, double* rpy) {
  Cr5Kinematics kin;
  Eigen::Map<const JointVec> q(j);
  Eigen::Map<Eigen::Vector3d> xyz_mm(xyz);
  Eigen::Map<Eigen::Vector3d> rpy_deg(rpy);
  Eigen::Vector3d xyz_v, rpy_v;
  kin.FkController(q, xyz_v, rpy_v);
  xyz_mm = xyz_v;
  rpy_deg = rpy_v;
}

int c_inverse_controller(const double* xyz, const double* rpy, double* q_sols) {
  Cr5Kinematics kin;
  JointVec sols[8];
  const int n = kin.IkController({xyz[0], xyz[1], xyz[2]}, {rpy[0], rpy[1], rpy[2]}, sols);
  for (int i = 0; i < n; ++i) std::memcpy(q_sols + i * 6, sols[i].data(), 6 * sizeof(double));
  return n;
}

int c_get_best_ik(const double* T, const double* q_curr, const double* joint_min,
                  const double* joint_max, const double* weights, double* q_out) {
  RobotLimits lim;
  for (int i = 0; i < 6; ++i) {
    lim.min_rad[i] = joint_min[i];
    lim.max_rad[i] = joint_max[i];
  }
  return Cr5Kinematics(lim).BestIkRaw(T, q_curr, weights, q_out);
}

int c_walk_movel(const double* p_start, const double* p_end, const double* quat1,
                 const double* quat2, const double* q_start, const double* alphas, int n_alphas,
                 const double* q_branch_end, int check_end_branch, double max_jump_rad,
                 double match_rad, const double* joint_min, const double* joint_max,
                 const double* weights, double deg2_from_rad2, double* q_end_out,
                 double* cost_out) {
  RobotLimits lim;
  for (int i = 0; i < 6; ++i) {
    lim.min_rad[i] = joint_min[i];
    lim.max_rad[i] = joint_max[i];
  }
  Cr5Kinematics kin(lim);
  return SegmentChecker(kin).WalkRaw(p_start, p_end, quat1, quat2, q_start, alphas, n_alphas,
                                     q_branch_end, check_end_branch, max_jump_rad, match_rad,
                                     weights, deg2_from_rad2, q_end_out, cost_out);
}

void c_inverse_batch(const double* T_batch, int n, double* q_sols_batch, int* n_sols) {
  for (int i = 0; i < n; ++i) {
    n_sols[i] = Cr5Kinematics::IkRaw(T_batch + i * 16, q_sols_batch + i * 48);
  }
}

}  // extern "C"
