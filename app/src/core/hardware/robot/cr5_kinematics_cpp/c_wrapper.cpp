#include <vector>

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
}
