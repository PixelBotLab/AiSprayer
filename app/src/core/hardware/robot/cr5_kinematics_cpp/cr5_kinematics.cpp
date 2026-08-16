#include "cr5_kinematics/cr5_kinematics.h"
#include <cmath>
#include <iostream>

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

namespace cr5_kinematics {
    
    // =========================================================================
    // STANDARD INTERFACES
    // =========================================================================

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

    bool ComputeIk(const double* eetrans, const double* eerot, std::vector<std::vector<double>>& vsolutions) {
        double T[16];
        for(int i=0; i<3; ++i) {
            T[i*4+3] = eetrans[i];
            T[i*4+0] = eerot[i*3+0];
            T[i*4+1] = eerot[i*3+1];
            T[i*4+2] = eerot[i*3+2];
        }
        T[3*4+0] = 0; T[3*4+1] = 0; T[3*4+2] = 0; T[3*4+3] = 1;

        double q_sols[8*6];
        int num_sols = inverse(T, q_sols);

        vsolutions.clear();
        for(int i=0; i<num_sols; ++i) {
            std::vector<double> sol(6);
            for(int j=0; j<6; ++j) {
                sol[j] = q_sols[i*6+j];
            }
            vsolutions.push_back(sol);
        }
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

    int inverse_controller(const double* xyz, const double* rpy, std::vector<std::vector<double>>& vsolutions) {
        double rx = rpy[0] * M_PI / 180.0;
        double ry = rpy[1] * M_PI / 180.0;
        double rz = rpy[2] * M_PI / 180.0;

        double R_z[9] = { cos(rz), -sin(rz), 0,  sin(rz), cos(rz), 0,  0, 0, 1 };
        double R_y[9] = { cos(ry), 0, sin(ry),   0, 1, 0,  -sin(ry), 0, cos(ry) };
        double R_x[9] = { 1, 0, 0,   0, cos(rx), -sin(rx),   0, sin(rx), cos(rx) };

        // R_ctrl = R_z * R_y * R_x
        double R_zy[9];
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                R_zy[i*3+j] = R_z[i*3+0]*R_y[0*3+j] + R_z[i*3+1]*R_y[1*3+j] + R_z[i*3+2]*R_y[2*3+j];
            }
        }
        double R_ctrl[9];
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                R_ctrl[i*3+j] = R_zy[i*3+0]*R_x[0*3+j] + R_zy[i*3+1]*R_x[1*3+j] + R_zy[i*3+2]*R_x[2*3+j];
            }
        }

        double T_ctrl[16] = {
            R_ctrl[0], R_ctrl[1], R_ctrl[2], xyz[0] / 1000.0,
            R_ctrl[3], R_ctrl[4], R_ctrl[5], xyz[1] / 1000.0,
            R_ctrl[6], R_ctrl[7], R_ctrl[8], xyz[2] / 1000.0,
            0,         0,         0,         1
        };

        // T_urdf = T_base_inv * T_ctrl * T_tool_inv
        // T_base_inv = RotZ(-180) = RotZ(180) -> x=-x, y=-y
        // T_tool_inv = [0 -1 0; 0 0 -1; 1 0 0]
        
        double T_bt[16];
        for(int i=0; i<16; i++) T_bt[i] = T_ctrl[i];
        for(int i=0; i<4; i++) {
            T_bt[0*4+i] = -T_ctrl[0*4+i];
            T_bt[1*4+i] = -T_ctrl[1*4+i];
        }

        double T_urdf[16];
        for(int i=0; i<3; i++) {
            T_urdf[i*4+0] =  T_bt[i*4+2];
            T_urdf[i*4+1] = -T_bt[i*4+0];
            T_urdf[i*4+2] = -T_bt[i*4+1];
            T_urdf[i*4+3] =  T_bt[i*4+3];
        }
        T_urdf[12] = 0; T_urdf[13] = 0; T_urdf[14] = 0; T_urdf[15] = 1;

        double q_sols[8*6];
        int num_sols = inverse(T_urdf, q_sols);

        vsolutions.clear();
        for(int i=0; i<num_sols; ++i) {
            std::vector<double> sol(6);
            for(int j=0; j<6; ++j) {
                sol[j] = q_sols[i*6+j];
            }
            vsolutions.push_back(sol);
        }
        return num_sols;
    }
}
