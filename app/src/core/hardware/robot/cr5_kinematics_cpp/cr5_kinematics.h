#ifndef CR5_KINEMATICS_H
#define CR5_KINEMATICS_H

#include <vector>

// High-level CR5 kinematics in the URDF frame (DH q2/q4 offsets applied here).
// ctypes entry points are in c_wrapper.cpp (c_forward, c_inverse, …).
// Python twin: CR5Kinematics(backend="python"|"cpp").

namespace cr5_kinematics {
  
  // =========================================================================
  // STANDARD INTERFACES (Using 4x4 Transformation Matrix, URDF Coordinate Frame)
  // =========================================================================
  
  // @param q: input joint angles [6]
  // @param T: output 4x4 transformation matrix (row-major)
  void forward(const double* q, double* T);

  // @param T: input 4x4 transformation matrix (row-major)
  // @param q_sols: output 6-DoF joint solutions. Can have up to 8 solutions.
  //                array should be of size 8*6.
  // @return: number of solutions found (0 to 8)
  int inverse(const double* T, double* q_sols);


  // =========================================================================
  // UTILITY INTERFACES (IKFast-style, URDF Coordinate Frame)
  // =========================================================================

  // @param j: input joint angles [6]
  // @param eetrans: output translation [3] (x, y, z)
  // @param eerot: output rotation matrix [9] (row-major 3x3)
  void ComputeFk(const double* j, double* eetrans, double* eerot);

  // @param eetrans: input translation [3] (x, y, z)
  // @param eerot: input rotation matrix [9] (row-major 3x3)
  // @param vsolutions: output vector of joint solutions, each solution is a vector of 6 doubles
  // @return: true if at least one solution is found
  bool ComputeIk(const double* eetrans, const double* eerot, std::vector<std::vector<double>>& vsolutions);

  // Fast path: write up to 8 solutions into q_sols[8*6], same layout as inverse().
  int ComputeIk(const double* eetrans, const double* eerot, double* q_sols);

  // =========================================================================
  // CONTROLLER INTERFACES (Mapped to Dobot Controller Coordinate Frame)
  // Base rotated 180 deg around Z. Tool permuted (X_t=-Y_u, Y_t=-Z_u, Z_t=X_u)
  // =========================================================================

  // @param j: input joint angles [6]
  // @param xyz: output translation in mm [3]
  // @param rpy: output Euler ZYX angles in degrees [3] (rx, ry, rz)
  void forward_controller(const double* j, double* xyz, double* rpy);

  // @param xyz: input translation in mm [3]
  // @param rpy: input Euler ZYX angles in degrees [3] (rx, ry, rz)
  // @param vsolutions: output vector of joint solutions, each is a vector of 6 doubles (radians)
  // @return: number of solutions found
  int inverse_controller(const double* xyz, const double* rpy, std::vector<std::vector<double>>& vsolutions);

  // Fast path: write up to 8 solutions into q_sols[8*6], same layout as inverse().
  int inverse_controller(const double* xyz, const double* rpy, double* q_sols);

  // =========================================================================
  // PATH-OPT HOT PATHS (used by SprayWaypointOptimizer)
  // =========================================================================

  // Nearest in-limit IK branch to q_curr. weights may be null (all 1).
  // Returns 1 if a solution is written to q_out, else 0.
  int get_best_ik(
      const double* T,
      const double* q_curr,
      const double* joint_min,
      const double* joint_max,
      const double* weights,
      double* q_out);

  // Controller-frame MoveL walk: lerp position, slerp quat, track IK.
  // Returns 1 if the edge is feasible; writes q_end_out and cost_out.
  int walk_movel(
      const double* p_start,
      const double* p_end,
      const double* quat1,
      const double* quat2,
      const double* q_start,
      const double* alphas,
      int n_alphas,
      const double* q_branch_end,
      int check_end_branch,
      double max_jump_rad,
      double match_rad,
      const double* joint_min,
      const double* joint_max,
      const double* weights,
      double deg2_from_rad2,
      double* q_end_out,
      double* cost_out);

  // n URDF poses (row-major 4×4 packed). q_sols_batch is n*48, n_sols is n.
  void inverse_batch(const double* T_batch, int n, double* q_sols_batch, int* n_sols);

}

#endif // CR5_KINEMATICS_H
