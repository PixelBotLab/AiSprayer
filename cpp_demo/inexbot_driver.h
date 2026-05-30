#pragma once

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <cstring>
#include "cpp/interface/nrc_api.h"

// Helper to get current timestamp string
inline std::string get_current_time_str() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() % 1000;

    std::stringstream ss;
    struct tm buf;
    localtime_r(&time_t_now, &buf);
    ss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << millis;
    return ss.str();
}

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__))

#define LOG_INFO std::cout << "[" << get_current_time_str() << "][" << __FILENAME__ << ":" << __LINE__ << "] "
#define LOG_WARN std::cout << "\033[33m[" << get_current_time_str() << "][" << __FILENAME__ << ":" << __LINE__ << "] "
#define LOG_WARN_END "\033[0m" << std::endl
#define LOG_ERROR std::cerr << "\033[31m[" << get_current_time_str() << "][" << __FILENAME__ << ":" << __LINE__ << "] "
#define LOG_ERROR_END "\033[0m" << std::endl

// Struct representing cartesian pose of the robot
struct RobotPose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    RobotPose() = default;
    RobotPose(double x_, double y_, double z_, double a_, double b_, double c_)
        : x(x_), y(y_), z(z_), a(a_), b(b_), c(c_) {}

    static RobotPose from_list(const std::vector<double>& data) {
        RobotPose p;
        if (data.size() >= 6) {
            p.x = data[0];
            p.y = data[1];
            p.z = data[2];
            p.a = data[3];
            p.b = data[4];
            p.c = data[5];
        }
        return p;
    }

    std::vector<double> to_list() const {
        return {x, y, z, a, b, c};
    }

    bool operator==(const RobotPose& other) const {
        const double eps = 1e-5;
        return std::abs(x - other.x) < eps &&
               std::abs(y - other.y) < eps &&
               std::abs(z - other.z) < eps &&
               std::abs(a - other.a) < eps &&
               std::abs(b - other.b) < eps &&
               std::abs(c - other.c) < eps;
    }
};
class InexbotDriver {
public:
    // Constants matching the configuration in inexbot_driver.py
    static constexpr int COORD = 1;       // Cartesian Coordinate System (直角坐标系)
    static constexpr int MODE = 0;        // Running mode (0=Teach mode, 示教模式)
    static constexpr int ANGLE_UNIT = 1;  // Radian mode (弧度制)

    // Servo State Constants
    static constexpr int SERVO_STATE_STOP = 0;
    static constexpr int SERVO_STATE_READY = 1;
    static constexpr int SERVO_STATE_ALARM = 2;
    static constexpr int SERVO_STATE_RUNNING = 3;

    // Running State Constants
    static constexpr int RUNNING_STATE_STOP = 0;
    static constexpr int RUNNING_STATE_PAUSE = 1;
    static constexpr int RUNNING_STATE_RUNNING = 2;

    static constexpr double DEFAULT_VELOCITY = 50.0;
    static constexpr double DEFAULT_ACC = 50.0;
    static constexpr double DEFAULT_DEC = 50.0;

    InexbotDriver(const std::string& ip = "192.168.2.14", 
                  const std::string& port = "6001", 
                  int tool_num = 0, 
                  bool reconnect = true);
    ~InexbotDriver();

    bool startup(double timeout_sec = 10.0);
    void shutdown();

    int get_servo_state();
    int get_running_state();
    RobotPose get_current_pose();
    bool is_reachable_l(const RobotPose& pose);
    bool is_reachable_j(const RobotPose& pose);

    int move_j(const RobotPose& pose, 
               double velocity = DEFAULT_VELOCITY, 
               double acc = DEFAULT_ACC, 
               double dec = DEFAULT_DEC, 
               int tool_num = 0, 
               bool wait = true);

    int move_l(const RobotPose& pose, 
               double velocity = DEFAULT_VELOCITY, 
               double acc = DEFAULT_ACC, 
               double dec = DEFAULT_DEC, 
               int tool_num = 0, 
               bool wait = true);

    int go_home(bool wait = true);

    // High-level queue execution method
    int execute_queue(const std::vector<RobotPose>& poses,
                      const std::string& move_type = "L",
                      double velocity = DEFAULT_VELOCITY,
                      double acc = DEFAULT_ACC,
                      double dec = DEFAULT_DEC,
                      int tool_num = 0,
                      int pl = 0,
                      bool wait = true);

    void print_system_info();

private:
    std::string m_ip;
    std::string m_port;
    int m_tool_num;
    bool m_reconnect;
    SOCKETFD m_fd;
    int m_queue_size;

    // std::thread m_keepalive_thread;
    // std::atomic<bool> m_keepalive_stop;

    // void _keepalive_loop();
    bool _is_reachable(const RobotPose& pose, const std::string& movetype);
    MoveCmd _make_movecmd(const RobotPose& pose, 
                         double velocity, 
                         double acc, 
                         double dec, 
                         int tool_num = 0, 
                         int user_num = 0, 
                         int pl = 0);
    void _wait_motion_done(double poll_interval_sec = 0.05);
    int _queue_send_batched(bool wait);
    void _wait_queue_done(double poll_interval_sec = 0.05);

    // Private queue helper methods
    int queue_start();
    int queue_push_l(const RobotPose& pose, 
                     double velocity = DEFAULT_VELOCITY, 
                     double acc = DEFAULT_ACC, 
                     double dec = DEFAULT_DEC, 
                     int tool_num = 0, 
                     int pl = 0);
    int queue_push_j(const RobotPose& pose, 
                     double velocity = DEFAULT_VELOCITY, 
                     double acc = DEFAULT_ACC, 
                     double dec = DEFAULT_DEC, 
                     int tool_num = 0, 
                     int pl = 0);
    int queue_send(bool wait = true);
    int queue_suspend();
    int queue_resume();
    int queue_stop();
    int queue_get_remaining();
};
