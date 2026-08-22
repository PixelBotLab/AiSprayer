#include "cr5_kinematics.h"

#include <cmath>
#include <cstring>

// Hot-path helpers for SprayWaypointOptimizer:
//   get_best_ik     — nearest in-limit branch without heap traffic
//   walk_movel      — one C call per DP edge (slerp + IK + jump/singularity)
//   inverse_batch   — candidate-generation IK without per-pose ctypes

namespace cr5_kinematics {
int inverse(const double* T, double* q_sols);
}

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSingSin = 0.052335956242943835;      // sin(3°)
constexpr double kShoulderHalf = 0.05235987755982989;  // 3°
constexpr double kD4 = 0.141;
constexpr double kD6 = 0.105;
constexpr double kJointTol = 1e-4;

inline double wrap_pi(double x) {
    x = std::fmod(x + kPi, 2.0 * kPi);
    if (x < 0.0) {
        x += 2.0 * kPi;
    }
    return x - kPi;
}

inline bool is_joint_valid(const double* q, const double* mn, const double* mx) {
    for (int i = 0; i < 6; ++i) {
        if (q[i] < mn[i] - kJointTol || q[i] > mx[i] + kJointTol) {
            return false;
        }
    }
    return true;
}

// Same indexing as CR5Kinematics._shoulder_q1_half_separation_rad.
inline double shoulder_half(const double* T) {
    const double t02 = -T[0];
    const double t03 = -T[3];
    const double t12 = -T[4];
    const double t13 = -T[7];
    const double a = kD6 * t12 - t13;
    const double b = kD6 * t02 - t03;
    const double r = a * a + b * b;
    if (r <= 1e-16) {
        return 0.0;
    }
    const double ratio = kD4 / std::sqrt(r);
    if (ratio >= 1.0) {
        return 0.0;
    }
    if (ratio <= -1.0) {
        return kPi;
    }
    return std::acos(ratio);
}

// T_urdf = controller_matrix_to_urdf(T_ctrl), row-major 4×4.
inline void ctrl_to_urdf(const double* Tc, double* Tu) {
    Tu[0] = -Tc[2];
    Tu[1] = Tc[0];
    Tu[2] = Tc[1];
    Tu[3] = -Tc[3];
    Tu[4] = -Tc[6];
    Tu[5] = Tc[4];
    Tu[6] = Tc[5];
    Tu[7] = -Tc[7];
    Tu[8] = Tc[10];
    Tu[9] = -Tc[8];
    Tu[10] = -Tc[9];
    Tu[11] = Tc[11];
    Tu[12] = 0.0;
    Tu[13] = 0.0;
    Tu[14] = 0.0;
    Tu[15] = 1.0;
}

// Unit-quaternion slerp → row-major 3×3. quat = [x, y, z, w].
inline void quat_slerp_to_R(const double* q1, const double* q2, double alpha, double* R) {
    double dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
    double q2m[4] = {q2[0], q2[1], q2[2], q2[3]};
    if (dot < 0.0) {
        q2m[0] = -q2m[0];
        q2m[1] = -q2m[1];
        q2m[2] = -q2m[2];
        q2m[3] = -q2m[3];
        dot = -dot;
    }
    double q[4];
    if (dot > 0.9995) {
        const double om = 1.0 - alpha;
        q[0] = om * q1[0] + alpha * q2m[0];
        q[1] = om * q1[1] + alpha * q2m[1];
        q[2] = om * q1[2] + alpha * q2m[2];
        q[3] = om * q1[3] + alpha * q2m[3];
        const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] /= n;
        q[1] /= n;
        q[2] /= n;
        q[3] /= n;
    } else {
        const double theta = std::acos(dot > 1.0 ? 1.0 : (dot < -1.0 ? -1.0 : dot));
        const double s = std::sin(theta);
        const double w1 = std::sin((1.0 - alpha) * theta) / s;
        const double w2 = std::sin(alpha * theta) / s;
        q[0] = w1 * q1[0] + w2 * q2m[0];
        q[1] = w1 * q1[1] + w2 * q2m[1];
        q[2] = w1 * q1[2] + w2 * q2m[2];
        q[3] = w1 * q1[3] + w2 * q2m[3];
    }
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    R[0] = 1.0 - 2.0 * (y * y + z * z);
    R[1] = 2.0 * (x * y - z * w);
    R[2] = 2.0 * (x * z + y * w);
    R[3] = 2.0 * (x * y + z * w);
    R[4] = 1.0 - 2.0 * (x * x + z * z);
    R[5] = 2.0 * (y * z - x * w);
    R[6] = 2.0 * (x * z - y * w);
    R[7] = 2.0 * (y * z + x * w);
    R[8] = 1.0 - 2.0 * (x * x + y * y);
}

inline bool unwrap_onto(const double* q_sol, const double* q_ref,
                        const double* mn, const double* mx, double* q_out) {
    double q_u[6];
    for (int i = 0; i < 6; ++i) {
        q_u[i] = q_ref[i] + wrap_pi(q_sol[i] - q_ref[i]);
    }
    if (is_joint_valid(q_u, mn, mx)) {
        std::memcpy(q_out, q_u, 6 * sizeof(double));
        return true;
    }
    if (is_joint_valid(q_sol, mn, mx)) {
        std::memcpy(q_out, q_sol, 6 * sizeof(double));
        return true;
    }
    return false;
}

}  // namespace

namespace cr5_kinematics {

int get_best_ik(
    const double* T,
    const double* q_curr,
    const double* joint_min,
    const double* joint_max,
    const double* weights,
    double* q_out
) {
    double sols[48];
    const int n = inverse(T, sols);
    double best_dist = 1e300;
    bool found = false;
    double best[6];
    const double w0 = weights ? weights[0] : 1.0;
    const double w1 = weights ? weights[1] : 1.0;
    const double w2 = weights ? weights[2] : 1.0;
    const double w3 = weights ? weights[3] : 1.0;
    const double w4 = weights ? weights[4] : 1.0;
    const double w5 = weights ? weights[5] : 1.0;
    const double ww[6] = {w0, w1, w2, w3, w4, w5};

    for (int i = 0; i < n; ++i) {
        const double* sol = sols + i * 6;
        if (!is_joint_valid(sol, joint_min, joint_max)) {
            continue;
        }
        double d[6];
        double unwrapped[6];
        double dist = 0.0;
        for (int j = 0; j < 6; ++j) {
            d[j] = wrap_pi(sol[j] - q_curr[j]);
            unwrapped[j] = q_curr[j] + d[j];
            dist += ww[j] * d[j] * d[j];
        }
        const double* cand = nullptr;
        if (is_joint_valid(unwrapped, joint_min, joint_max)) {
            cand = unwrapped;
        } else if (is_joint_valid(sol, joint_min, joint_max)) {
            cand = sol;
        }
        if (cand != nullptr && dist < best_dist) {
            best_dist = dist;
            std::memcpy(best, cand, 6 * sizeof(double));
            found = true;
        }
    }
    if (!found) {
        return 0;
    }
    std::memcpy(q_out, best, 6 * sizeof(double));
    return 1;
}

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
    double* cost_out
) {
    double prev[6];
    std::memcpy(prev, q_start, 6 * sizeof(double));
    double acc = 0.0;
    double T_ctrl[16];
    T_ctrl[12] = 0.0;
    T_ctrl[13] = 0.0;
    T_ctrl[14] = 0.0;
    T_ctrl[15] = 1.0;

    for (int k = 0; k < n_alphas; ++k) {
        const double a = alphas[k];
        const double om = 1.0 - a;
        T_ctrl[3] = om * p_start[0] + a * p_end[0];
        T_ctrl[7] = om * p_start[1] + a * p_end[1];
        T_ctrl[11] = om * p_start[2] + a * p_end[2];

        double R[9];
        quat_slerp_to_R(quat1, quat2, a, R);
        T_ctrl[0] = R[0];
        T_ctrl[1] = R[1];
        T_ctrl[2] = R[2];
        T_ctrl[4] = R[3];
        T_ctrl[5] = R[4];
        T_ctrl[6] = R[5];
        T_ctrl[8] = R[6];
        T_ctrl[9] = R[7];
        T_ctrl[10] = R[8];

        double T_urdf[16];
        ctrl_to_urdf(T_ctrl, T_urdf);

        double nxt[6];
        if (!get_best_ik(T_urdf, prev, joint_min, joint_max, weights, nxt)) {
            return 0;
        }
        if (!is_joint_valid(nxt, joint_min, joint_max)) {
            return 0;
        }
        if (std::fabs(std::sin(nxt[4])) < kSingSin || std::fabs(std::sin(nxt[2])) < kSingSin) {
            return 0;
        }
        if (shoulder_half(T_urdf) < kShoulderHalf) {
            return 0;
        }

        double max_abs = 0.0;
        for (int j = 0; j < 6; ++j) {
            const double dq = wrap_pi(nxt[j] - prev[j]);
            const double ad = std::fabs(dq);
            if (ad > max_abs) {
                max_abs = ad;
            }
            acc += weights[j] * dq * dq;
        }
        if (max_abs > max_jump_rad) {
            return 0;
        }
        std::memcpy(prev, nxt, 6 * sizeof(double));
    }

    if (check_end_branch) {
        double q_target[6];
        if (!unwrap_onto(q_branch_end, prev, joint_min, joint_max, q_target)) {
            return 0;
        }
        for (int j = 0; j < 6; ++j) {
            if (std::fabs(wrap_pi(prev[j] - q_target[j])) > match_rad) {
                return 0;
            }
        }
    }

    *cost_out = acc * deg2_from_rad2;
    std::memcpy(q_end_out, prev, 6 * sizeof(double));
    return 1;
}

void inverse_batch(const double* T_batch, int n, double* q_sols_batch, int* n_sols) {
    for (int i = 0; i < n; ++i) {
        n_sols[i] = inverse(T_batch + i * 16, q_sols_batch + i * 48);
    }
}

}  // namespace cr5_kinematics
