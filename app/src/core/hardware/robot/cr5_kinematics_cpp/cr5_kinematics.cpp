#include "cr5_kinematics.h"
#include <cmath>
#include <cstring>

#define CR5_PARAMS
namespace ur_kinematics {
    const double d1 = 0.147;
    const double a2 = -0.427;
    const double a3 = -0.357;
    const double d4 = 0.141;
    const double d5 = 0.116;
    const double d6 = 0.105;

    void forward(const double* q, double* T);
    int inverse(const double* T, double* q_sols, double q6_des);
}

namespace {

void fill_sols(std::vector<std::vector<double>>& vsolutions, const double* q_sols, int num_sols) {
    vsolutions.resize(num_sols);
    for (int i = 0; i < num_sols; ++i) {
        vsolutions[i].resize(6);
        std::memcpy(vsolutions[i].data(), q_sols + i * 6, 6 * sizeof(double));
    }
}

// T_urdf = T_base_inv * T_ctrl * T_tool_inv, with R_ctrl = Rz(rz)*Ry(ry)*Rx(rx).
void controller_pose_to_urdf(const double* xyz_mm, const double* rpy_deg, double* T_urdf) {
    const double rx = rpy_deg[0] * M_PI / 180.0;
    const double ry = rpy_deg[1] * M_PI / 180.0;
    const double rz = rpy_deg[2] * M_PI / 180.0;
    const double sx = sin(rx), cx = cos(rx);
    const double sy = sin(ry), cy = cos(ry);
    const double sz = sin(rz), cz = cos(rz);

    const double r00 = cz * cy;
    const double r01 = cz * sy * sx - sz * cx;
    const double r02 = cz * sy * cx + sz * sx;
    const double r10 = sz * cy;
    const double r11 = sz * sy * sx + cz * cx;
    const double r12 = sz * sy * cx - cz * sx;
    const double r20 = -sy;
    const double r21 = cy * sx;
    const double r22 = cy * cx;

    const double x = xyz_mm[0] / 1000.0;
    const double y = xyz_mm[1] / 1000.0;
    const double z = xyz_mm[2] / 1000.0;

    T_urdf[0] = -r02; T_urdf[1] =  r00; T_urdf[2] =  r01; T_urdf[3] = -x;
    T_urdf[4] = -r12; T_urdf[5] =  r10; T_urdf[6] =  r11; T_urdf[7] = -y;
    T_urdf[8] =  r22; T_urdf[9] = -r20; T_urdf[10] = -r21; T_urdf[11] = z;
    T_urdf[12] = 0.0; T_urdf[13] = 0.0; T_urdf[14] = 0.0; T_urdf[15] = 1.0;
}

}  // namespace

namespace cr5_kinematics {
    
    // =========================================================================
    // STANDARD INTERFACES
    // =========================================================================

    // URDF FK: convert q2/q4 by −π/2 into DH, then analytical product of transforms.
    void forward(const double* q_urdf, double* T) {
        double q_dh[6];
        q_dh[0] = q_urdf[0];
        q_dh[1] = q_urdf[1] - M_PI / 2.0;
        q_dh[2] = q_urdf[2];
        q_dh[3] = q_urdf[3] - M_PI / 2.0;
        q_dh[4] = q_urdf[4];
        q_dh[5] = q_urdf[5];

        ur_kinematics::forward(q_dh, T);
    }

    // URDF IK: DH inverse (q6_des=0), then q2/q4 += π/2 and wrap each joint to [-π, π].
    // Up to 8 solutions written densely into q_sols[8*6].
    int inverse(const double* T, double* q_sols) {
        double q_dh_sols[8 * 6];
        int num_sols = ur_kinematics::inverse(T, q_dh_sols, 0.0);

        for (int i = 0; i < num_sols; i++) {
            q_sols[i * 6 + 0] = q_dh_sols[i * 6 + 0];
            q_sols[i * 6 + 1] = q_dh_sols[i * 6 + 1] + M_PI / 2.0;
            q_sols[i * 6 + 2] = q_dh_sols[i * 6 + 2];
            q_sols[i * 6 + 3] = q_dh_sols[i * 6 + 3] + M_PI / 2.0;
            q_sols[i * 6 + 4] = q_dh_sols[i * 6 + 4];
            q_sols[i * 6 + 5] = q_dh_sols[i * 6 + 5];

            // Normalize to [-pi, pi]
            for (int j = 0; j < 6; j++) {
                while (q_sols[i * 6 + j] > M_PI) q_sols[i * 6 + j] -= 2 * M_PI;
                while (q_sols[i * 6 + j] < -M_PI) q_sols[i * 6 + j] += 2 * M_PI;
            }
        }
        return num_sols;
    }

    // =========================================================================
    // UTILITY INTERFACES (IKFast-style)
    // =========================================================================

    void ComputeFk(const double* j, double* eetrans, double* eerot) {
        double T[16];
        forward(j, T);
        for(int i=0; i<3; ++i) {
            eetrans[i] = T[i*4+3];
            eerot[i*3+0] = T[i*4+0];
            eerot[i*3+1] = T[i*4+1];
            eerot[i*3+2] = T[i*4+2];
        }
    }

    int ComputeIk(const double* eetrans, const double* eerot, double* q_sols) {
        double T[16];
        for (int i = 0; i < 3; ++i) {
            T[i * 4 + 3] = eetrans[i];
            T[i * 4 + 0] = eerot[i * 3 + 0];
            T[i * 4 + 1] = eerot[i * 3 + 1];
            T[i * 4 + 2] = eerot[i * 3 + 2];
        }
        T[12] = 0; T[13] = 0; T[14] = 0; T[15] = 1;
        return inverse(T, q_sols);
    }

    bool ComputeIk(const double* eetrans, const double* eerot, std::vector<std::vector<double>>& vsolutions) {
        double q_sols[8 * 6];
        int num_sols = ComputeIk(eetrans, eerot, q_sols);
        fill_sols(vsolutions, q_sols, num_sols);
        return num_sols > 0;
    }

    // =========================================================================
    // CONTROLLER INTERFACES (Mapped to Dobot Controller Coordinate Frame)
    // =========================================================================

    void forward_controller(const double* j, double* xyz, double* rpy) {
        double T_urdf[16];
        forward(j, T_urdf);

        // Apply Base and Tool transforms
        double T_bt[16];
        for(int k=0; k<16; k++) T_bt[k] = T_urdf[k];
        for(int k=0; k<4; k++) {
            T_bt[0*4+k] = -T_urdf[0*4+k];
            T_bt[1*4+k] = -T_urdf[1*4+k];
        }
        
        double T_ctrl[16];
        for(int k=0; k<3; k++) {
            T_ctrl[k*4+0] = -T_bt[k*4+1];
            T_ctrl[k*4+1] = -T_bt[k*4+2];
            T_ctrl[k*4+2] =  T_bt[k*4+0];
            T_ctrl[k*4+3] =  T_bt[k*4+3];
        }

        xyz[0] = T_ctrl[3] * 1000.0;
        xyz[1] = T_ctrl[7] * 1000.0;
        xyz[2] = T_ctrl[11] * 1000.0;

        double sol_rz = atan2(T_ctrl[1*4+0], T_ctrl[0*4+0]);
        double sol_ry = atan2(-T_ctrl[2*4+0], sqrt(T_ctrl[2*4+1]*T_ctrl[2*4+1] + T_ctrl[2*4+2]*T_ctrl[2*4+2]));
        double sol_rx = atan2(T_ctrl[2*4+1], T_ctrl[2*4+2]);

        rpy[0] = sol_rx * 180.0 / M_PI;
        rpy[1] = sol_ry * 180.0 / M_PI;
        rpy[2] = sol_rz * 180.0 / M_PI;
    }

    int inverse_controller(const double* xyz, const double* rpy, double* q_sols) {
        double T_urdf[16];
        controller_pose_to_urdf(xyz, rpy, T_urdf);
        return inverse(T_urdf, q_sols);
    }

    int inverse_controller(const double* xyz, const double* rpy, std::vector<std::vector<double>>& vsolutions) {
        double q_sols[8 * 6];
        int num_sols = inverse_controller(xyz, rpy, q_sols);
        fill_sols(vsolutions, q_sols, num_sols);
        return num_sols;
    }
}
