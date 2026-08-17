#include "cr5_kinematics.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <vector>
#include <string>
#include <functional>

// Keep in sync with test_cr5_kinematics.py
static const int kNumWarmup = 200;
static const int kNumIterations = 100000;
static const double kSampleQDeg[6] = {45.0, -30.0, 60.0, 0.0, 45.0, 10.0};

static const double kFuncConfigsDeg[][6] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {45.0, -30.0, 60.0, 10.0, 45.0, -20.0},
    {-90.0, 45.0, -45.0, 90.0, -90.0, 180.0},
    {120.0, -60.0, 30.0, -45.0, 60.0, 90.0},
    {180.159, -0.293, 90.653, 90.066, 90.035, 0.077},
    {0.028, 0.009, -90.029, 90.066, 90.035, 0.077},
};
static const int kNumFuncConfigs = 6;

static int g_failures = 0;

static void expect(bool cond, const std::string& msg) {
    if (!cond) {
        g_failures++;
        std::cout << "  FAIL: " << msg << std::endl;
    }
}

static void deg_to_rad(const double* deg, double* rad) {
    for (int i = 0; i < 6; i++) rad[i] = deg[i] * M_PI / 180.0;
}

static double wrap_pi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static double ang_diff(double a, double b) {
    return std::abs(wrap_pi(a - b));
}

static bool joints_match(const double* a, const double* b, double tol_rad) {
    for (int j = 0; j < 6; j++) {
        if (ang_diff(a[j], b[j]) > tol_rad) return false;
    }
    return true;
}

static bool find_matching_sol(const double* q_sols, int n, const double* q_target, double tol_rad) {
    for (int i = 0; i < n; i++) {
        if (joints_match(q_sols + i * 6, q_target, tol_rad)) return true;
    }
    return false;
}

static double mat4_pos_err_mm(const double* Ta, const double* Tb) {
    double dx = Ta[3] - Tb[3];
    double dy = Ta[7] - Tb[7];
    double dz = Ta[11] - Tb[11];
    return std::sqrt(dx * dx + dy * dy + dz * dz) * 1000.0;
}

static double mat4_rot_err(const double* Ta, const double* Tb) {
    double s = 0.0;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            double d = Ta[r * 4 + c] - Tb[r * 4 + c];
            s += d * d;
        }
    }
    return std::sqrt(s);
}

struct BenchResult {
    std::string name;
    double avg_us;
    double freq_hz;
    int sols_per;
};

static BenchResult bench(const std::string& name, std::function<int()> fn) {
    volatile int sink = 0;
    for (int i = 0; i < kNumWarmup; i++) sink += fn();

    auto t0 = std::chrono::steady_clock::now();
    int total = 0;
    for (int i = 0; i < kNumIterations; i++) total += fn();
    auto t1 = std::chrono::steady_clock::now();
    sink += total;

    double elapsed = std::chrono::duration<double>(t1 - t0).count();
    BenchResult r;
    r.name = name;
    r.avg_us = (elapsed * 1e6) / kNumIterations;
    r.freq_hz = kNumIterations / elapsed;
    r.sols_per = (total > 0) ? (total / kNumIterations) : 0;
    (void)sink;
    return r;
}

static void print_bench_table(const std::vector<BenchResult>& rows) {
    std::cout << "\n========== STRESS TEST (C++) ==========" << std::endl;
    std::cout << "warmup=" << kNumWarmup << "  iterations=" << kNumIterations
              << "  sample_q_deg=[45, -30, 60, 0, 45, 10]" << std::endl;
    std::cout << std::left << std::setw(28) << "Interface"
              << std::right << std::setw(14) << "avg (us)"
              << std::setw(14) << "throughput"
              << std::setw(12) << "sols/call" << std::endl;
    std::cout << std::string(68, '-') << std::endl;
    for (const auto& r : rows) {
        std::cout << std::left << std::setw(28) << r.name
                  << std::right << std::fixed << std::setprecision(2)
                  << std::setw(14) << r.avg_us
                  << std::setw(10) << (int)r.freq_hz << " Hz"
                  << std::setw(12) << r.sols_per << std::endl;
    }
}

void test_dobot_controller_cases() {
    std::cout << "========== DOBOT CONTROLLER CASES ==========" << std::endl;
    const double cases[][6] = {
        {180.159, -0.293, 90.653, 90.066, 90.035, 0.077},
        {0.028, 0.009, -90.029, 90.066, 90.035, 0.077},
    };
    const char* names[] = {"Case 1", "Case 2"};
    const double ik_tol = 0.5 * M_PI / 180.0;

    for (int c = 0; c < 2; c++) {
        double q_target[6];
        deg_to_rad(cases[c], q_target);

        std::cout << names[c] << " Joint Angles (deg): ";
        for (int i = 0; i < 6; i++) std::cout << cases[c][i] << " ";
        std::cout << std::endl;

        double xyz[3], rpy[3];
        cr5_kinematics::forward_controller(q_target, xyz, rpy);
        std::cout << names[c] << " TCP Position (mm): XYZ = "
                  << xyz[0] << " " << xyz[1] << " " << xyz[2] << std::endl;
        std::cout << names[c] << " TCP Orientation (deg): RPY = "
                  << rpy[0] << " " << rpy[1] << " " << rpy[2] << std::endl;

        std::vector<std::vector<double>> sols;
        int num_sols = cr5_kinematics::inverse_controller(xyz, rpy, sols);
        expect(num_sols > 0, std::string(names[c]) + " inverse_controller found 0 solutions");

        double q_flat[8 * 6];
        for (int i = 0; i < num_sols; i++) {
            for (int j = 0; j < 6; j++) q_flat[i * 6 + j] = sols[i][j];
        }
        bool found = find_matching_sol(q_flat, num_sols, q_target, ik_tol);
        expect(found, std::string(names[c]) + " did not recover target q");

        std::cout << "Found " << num_sols << " IK solutions from controller interface." << std::endl;
        for (int i = 0; i < num_sols; i++) {
            std::cout << "  Solution " << i << " (deg): ";
            bool match = joints_match(q_flat + i * 6, q_target, ik_tol);
            for (int j = 0; j < 6; j++) {
                std::cout << std::fixed << std::setprecision(3)
                          << sols[i][j] * 180.0 / M_PI << " ";
            }
            if (match) std::cout << " <-- MATCHES TARGET";
            std::cout << std::endl;
        }

        std::cout << "--- Verifying FK for all " << num_sols << " IK solutions ---" << std::endl;
        for (int i = 0; i < num_sols; i++) {
            double sol_xyz[3], sol_rpy[3];
            cr5_kinematics::forward_controller(q_flat + i * 6, sol_xyz, sol_rpy);
            std::cout << "  Sol " << i << " FK -> XYZ (mm): "
                      << std::setw(10) << std::fixed << std::setprecision(3)
                      << sol_xyz[0] << " " << std::setw(10) << sol_xyz[1] << " "
                      << std::setw(10) << sol_xyz[2]
                      << " | Euler ZYX (rx, ry, rz): "
                      << std::setw(10) << sol_rpy[0] << " "
                      << std::setw(10) << sol_rpy[1] << " "
                      << std::setw(10) << sol_rpy[2] << std::endl;
            double pos = std::sqrt((xyz[0] - sol_xyz[0]) * (xyz[0] - sol_xyz[0]) +
                                   (xyz[1] - sol_xyz[1]) * (xyz[1] - sol_xyz[1]) +
                                   (xyz[2] - sol_xyz[2]) * (xyz[2] - sol_xyz[2]));
            expect(pos < 0.05, std::string(names[c]) + " closed-loop pos error >= 0.05 mm");
        }
        if (found) {
            std::cout << names[c] << " SUCCESS: recovered the measured target configuration.\n" << std::endl;
        } else {
            std::cout << names[c] << " FAILED to find the target configuration.\n" << std::endl;
        }
    }
}

void test_functional() {
    std::cout << "========== FUNCTIONAL TEST (C++) ==========" << std::endl;
    const double ik_tol = 0.5 * M_PI / 180.0;  // 0.5 deg

    for (int c = 0; c < kNumFuncConfigs; c++) {
        double q[6];
        deg_to_rad(kFuncConfigsDeg[c], q);
        std::cout << "Config " << c << " deg: "
                  << kFuncConfigsDeg[c][0] << " " << kFuncConfigsDeg[c][1] << " "
                  << kFuncConfigsDeg[c][2] << " " << kFuncConfigsDeg[c][3] << " "
                  << kFuncConfigsDeg[c][4] << " " << kFuncConfigsDeg[c][5] << std::endl;

        // 1) forward: URDF 4x4
        double T[16];
        cr5_kinematics::forward(q, T);
        expect(std::abs(T[15] - 1.0) < 1e-12, "forward T[3,3] != 1");
        expect(std::abs(T[12]) < 1e-12 && std::abs(T[13]) < 1e-12 && std::abs(T[14]) < 1e-12,
               "forward last row != [0,0,0,1]");

        // 2) inverse: closed loop FK -> IK -> FK, recover q
        double q_sols[8 * 6];
        int n_inv = cr5_kinematics::inverse(T, q_sols);
        expect(n_inv > 0, "inverse found 0 solutions");
        expect(find_matching_sol(q_sols, n_inv, q, ik_tol), "inverse did not recover target q");
        for (int i = 0; i < n_inv; i++) {
            double T_sol[16];
            cr5_kinematics::forward(q_sols + i * 6, T_sol);
            expect(mat4_pos_err_mm(T, T_sol) < 0.05, "inverse sol position error >= 0.05 mm");
            expect(mat4_rot_err(T, T_sol) < 1e-3, "inverse sol rotation error >= 1e-3");
        }

        // 3) ComputeFk: must match forward unpack
        double eetrans[3], eerot[9];
        cr5_kinematics::ComputeFk(q, eetrans, eerot);
        expect(std::abs(eetrans[0] - T[3]) < 1e-12 &&
               std::abs(eetrans[1] - T[7]) < 1e-12 &&
               std::abs(eetrans[2] - T[11]) < 1e-12,
               "ComputeFk translation != forward");
        for (int r = 0; r < 3; r++) {
            for (int col = 0; col < 3; col++) {
                expect(std::abs(eerot[r * 3 + col] - T[r * 4 + col]) < 1e-12,
                       "ComputeFk rotation != forward");
            }
        }

        // 4) ComputeIk: same count as inverse, recover q, closed loop
        std::vector<std::vector<double>> vsols;
        bool ok_ik = cr5_kinematics::ComputeIk(eetrans, eerot, vsols);
        expect(ok_ik && (int)vsols.size() == n_inv, "ComputeIk solution count != inverse");
        double q_cik[8 * 6];
        for (int i = 0; i < (int)vsols.size(); i++) {
            for (int j = 0; j < 6; j++) q_cik[i * 6 + j] = vsols[i][j];
        }
        expect(find_matching_sol(q_cik, (int)vsols.size(), q, ik_tol),
               "ComputeIk did not recover target q");

        // Controller pair (same header, Dobot frame)
        double xyz[3], rpy[3];
        cr5_kinematics::forward_controller(q, xyz, rpy);
        std::vector<std::vector<double>> csols;
        int n_ctrl = cr5_kinematics::inverse_controller(xyz, rpy, csols);
        expect(n_ctrl > 0, "inverse_controller found 0 solutions");
        double q_ctrl[8 * 6];
        for (int i = 0; i < n_ctrl; i++) {
            for (int j = 0; j < 6; j++) q_ctrl[i * 6 + j] = csols[i][j];
        }
        expect(find_matching_sol(q_ctrl, n_ctrl, q, ik_tol),
               "inverse_controller did not recover target q");
        for (int i = 0; i < n_ctrl; i++) {
            double xyz2[3], rpy2[3];
            cr5_kinematics::forward_controller(q_ctrl + i * 6, xyz2, rpy2);
            double pos = std::sqrt((xyz[0] - xyz2[0]) * (xyz[0] - xyz2[0]) +
                                   (xyz[1] - xyz2[1]) * (xyz[1] - xyz2[1]) +
                                   (xyz[2] - xyz2[2]) * (xyz[2] - xyz2[2]));
            expect(pos < 0.05, "controller closed-loop position error >= 0.05 mm");
        }
    }

    if (g_failures == 0) {
        std::cout << "FUNCTIONAL: PASS (" << kNumFuncConfigs << " configs x 4 APIs + controller)\n" << std::endl;
    } else {
        std::cout << "FUNCTIONAL: FAIL (" << g_failures << " assertions)\n" << std::endl;
    }
}

void run_stress() {
    double q[6];
    deg_to_rad(kSampleQDeg, q);

    double T[16];
    cr5_kinematics::forward(q, T);
    double eetrans[3], eerot[9];
    cr5_kinematics::ComputeFk(q, eetrans, eerot);
    double xyz[3], rpy[3];
    cr5_kinematics::forward_controller(q, xyz, rpy);

    double q_sols[8 * 6];
    std::vector<std::vector<double>> vsols;
    double T_out[16];
    double ee_t[3], ee_r[9];
    double xyz_out[3], rpy_out[3];

    std::vector<BenchResult> rows;
    rows.push_back(bench("1. forward", [&]() {
        cr5_kinematics::forward(q, T_out);
        return 1;
    }));
    rows.push_back(bench("2. inverse", [&]() {
        return cr5_kinematics::inverse(T, q_sols);
    }));
    rows.push_back(bench("3. ComputeFk", [&]() {
        cr5_kinematics::ComputeFk(q, ee_t, ee_r);
        return 1;
    }));
    rows.push_back(bench("4. ComputeIk", [&]() {
        cr5_kinematics::ComputeIk(eetrans, eerot, vsols);
        return (int)vsols.size();
    }));
    rows.push_back(bench("5. forward_controller", [&]() {
        cr5_kinematics::forward_controller(q, xyz_out, rpy_out);
        return 1;
    }));
    rows.push_back(bench("6. inverse_controller", [&]() {
        return cr5_kinematics::inverse_controller(xyz, rpy, vsols);
    }));

    print_bench_table(rows);
}

int main() {
    test_dobot_controller_cases();
    test_functional();
    run_stress();
    return g_failures == 0 ? 0 : 1;
}
