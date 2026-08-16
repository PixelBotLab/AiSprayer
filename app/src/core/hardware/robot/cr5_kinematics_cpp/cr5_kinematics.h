#ifndef CR5_KINEMATICS_H
#define CR5_KINEMATICS_H

#include <vector>

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

}

#endif // CR5_KINEMATICS_H
