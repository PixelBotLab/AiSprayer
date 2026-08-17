#include "cr5_kinematics.h"

// C ABI for ctypes. All pointers are caller-owned:
//   q / xyz / rpy / eetrans : 6 or 3 doubles
//   T                       : 16 doubles, row-major 4×4
//   eerot                   : 9 doubles, row-major 3×3
//   q_sols                  : 48 doubles (8 solutions × 6 joints)
// c_forward / c_inverse / c_compute_* use the URDF frame (DH offsets applied inside).
// c_ur_* is the raw DH core (no URDF offset, no [-π, π] wrap).

namespace ur_kinematics {
    void forward(const double* q, double* T);
    int inverse(const double* T, double* q_sols, double q6_des);
}

extern "C" {
    void c_ur_forward(const double* q, double* T) {
        ur_kinematics::forward(q, T);
    }
    int c_ur_inverse(const double* T, double* q_sols, double q6_des) {
        return ur_kinematics::inverse(T, q_sols, q6_des);
    }

    void c_forward(const double* q, double* T) {
        cr5_kinematics::forward(q, T);
    }
    int c_inverse(const double* T, double* q_sols) {
        return cr5_kinematics::inverse(T, q_sols);
    }
    void c_compute_fk(const double* j, double* eetrans, double* eerot) {
        cr5_kinematics::ComputeFk(j, eetrans, eerot);
    }
    int c_compute_ik(const double* eetrans, const double* eerot, double* q_sols) {
        return cr5_kinematics::ComputeIk(eetrans, eerot, q_sols);
    }
    void c_forward_controller(const double* j, double* xyz, double* rpy) {
        cr5_kinematics::forward_controller(j, xyz, rpy);
    }
    int c_inverse_controller(const double* xyz, const double* rpy, double* q_sols) {
        return cr5_kinematics::inverse_controller(xyz, rpy, q_sols);
    }
}
