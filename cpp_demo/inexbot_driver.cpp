#include "inexbot_driver.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cstdio>

// Helper to convert Result code to string
static std::string result_to_string(int res_code) {
    switch (res_code) {
        case TIMEOUT: return "TIMEOUT (-6)";
        case EXCEPTION: return "EXCEPTION (-5)";
        case OPERATION_NOT_ALLOWED: return "OPERATION_NOT_ALLOWED (-4)";
        case PARAM_ERR: return "PARAM_ERR (-3)";
        case DISCONNECT: return "DISCONNECT (-2)";
        case RECEIVE_FAILED: return "RECEIVE_FAILED (-1)";
        case SUCCESS: return "SUCCESS (0)";
        default: return "UNKNOWN (" + std::to_string(res_code) + ")";
    }
}

// Static callback for robot error or warning messages
static void on_robot_error_warning(int messageType, const char* message, int messageCode) {
    std::string type_str = "UNKNOWN";
    if (messageType == 1) {
        type_str = "Warning";
    } else if (messageType == 2) {
        type_str = "Error";
    }
    
    if (messageType == 2) {
        LOG_ERROR << "Robot " << type_str << " message received: Code=" << messageCode 
                  << ", Info: " << (message ? message : "--") << LOG_ERROR_END;
    } else {
        LOG_INFO << "Robot " << type_str << " message received: Code=" << messageCode 
                 << ", Info: " << (message ? message : "--") << std::endl;
    }
}

// Static callback for robot reconnection events
static void on_robot_reconnect() {
    LOG_INFO << "Robot connection re-established!" << std::endl;
}

// Static callback for robot system/custom messages
static void on_robot_recv_message(int messageID, const char* message) {
    LOG_INFO << "[CALLBACK] Robot Message - ID: " << messageID 
             << " | Content: " << (message ? message : "--") << std::endl;
}

// Static callback for robot real-time state messages
static void on_robot_state_message(const char* message) {
    static auto last_print_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    // Limit state logs printing to prevent console flooding
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 2) {
        last_print_time = now;
        LOG_INFO << "[CALLBACK] Robot State (throttled 2s): " << (message ? message : "--") << std::endl;
    }
}

// Constructor
InexbotDriver::InexbotDriver(const std::string& ip, const std::string& port, int tool_num, bool reconnect)
    : m_ip(ip), m_port(port), m_tool_num(tool_num), m_reconnect(reconnect), m_fd(-1), m_queue_size(0)/*, m_keepalive_stop(false)*/ {}

// Destructor
InexbotDriver::~InexbotDriver() {
    shutdown();
}

// Startup
bool InexbotDriver::startup(double timeout_sec) {
    LOG_INFO << "[API] get_library_version() returned: " << ::get_library_version() << std::endl;

    if (m_fd >= 0) {
        LOG_INFO << "[startup] already started" << std::endl;
        return true;
    }

    // Connect to robot
    m_fd = ::connect_robot(m_ip, m_port);
    LOG_INFO << "[API] connect_robot(ip=\"" << m_ip << "\", port=\"" << m_port << "\") returned: " << m_fd << std::endl;
    if (m_fd < 0) {
        LOG_ERROR << "[startup] Failed to connect to robot: " << m_ip << ":" << m_port << LOG_ERROR_END;
        return false;
    }

    // Set reconnect
    Result res = ::set_reconnect(m_fd, m_reconnect);
    LOG_INFO << "[API] set_reconnect(reconnect=" << (m_reconnect ? "true" : "false") 
              << ") returned: " << result_to_string(res) << std::endl;

    // Register error/warning callback
    res = ::set_receive_error_or_warnning_message_callback(m_fd, on_robot_error_warning);
    LOG_INFO << "[API] set_receive_error_or_warnning_message_callback() returned: " << result_to_string(res) << std::endl;

    // Register reconnect callback
    res = ::set_reconnect_callback(m_fd, on_robot_reconnect);
    LOG_INFO << "[API] set_reconnect_callback() returned: " << result_to_string(res) << std::endl;

    // Register system/custom message callback
    res = ::recv_message(m_fd, on_robot_recv_message);
    LOG_INFO << "[API] recv_message() returned: " << result_to_string(res) << std::endl;

    // Register robot state callback
    res = ::robot_state_callback(m_fd, on_robot_state_message);
    LOG_INFO << "[API] robot_state_callback() returned: " << result_to_string(res) << std::endl;

    // Set coordinate system
    res = ::set_current_coord(m_fd, COORD);
    LOG_INFO << "[API] set_current_coord(coord=" << COORD << ") returned: " << result_to_string(res) << std::endl;
    if (res != SUCCESS) {
        ::disconnect_robot(m_fd);
        m_fd = -1;
        return false;
    }

    // Set running mode
    res = ::set_current_mode(m_fd, MODE);
    LOG_INFO << "[API] set_current_mode(mode=" << MODE << ") returned: " << result_to_string(res) << std::endl;
    if (res != SUCCESS) {
        ::disconnect_robot(m_fd);
        m_fd = -1;
        return false;
    }

    // Set tool hand number if > 0
    if (m_tool_num > 0) {
        res = ::set_tool_hand_number(m_fd, m_tool_num);
        LOG_INFO << "[API] set_tool_hand_number(tool=" << m_tool_num << ") returned: " << result_to_string(res) << std::endl;
        if (res != SUCCESS) {
            ::disconnect_robot(m_fd);
            m_fd = -1;
            return false;
        }
    }

    // Clear error
    res = ::clear_error(m_fd);
    LOG_INFO << "[API] clear_error() returned: " << result_to_string(res) << std::endl;
    if (res != SUCCESS) {
        ::disconnect_robot(m_fd);
        m_fd = -1;
        return false;
    }

    // Set state 0 -> 1
    for (int state : {0, 1}) {
        int current_state = -1;
        Result state_res = ::get_servo_state(m_fd, current_state);
        if (state_res == SUCCESS) {
            LOG_INFO << "[startup] Current servo state: " << current_state << std::endl;
            if (current_state == 2 || current_state == 3) {
                LOG_INFO << "[startup] Current state is " << current_state << " (cannot set_servo_state). Skipping." << std::endl;
                continue;
            }
        } else {
            LOG_ERROR << "[startup] Failed to get current servo state before setting to " << state 
                      << ", result: " << result_to_string(state_res) << LOG_ERROR_END;
        }

        res = ::set_servo_state(m_fd, state);
        LOG_INFO << "[API] set_servo_state(state=" << state << ") returned: " << result_to_string(res) << std::endl;
        if (res != SUCCESS) {
            LOG_ERROR << "[startup] Failed to set state to " << state << ", result: " << result_to_string(res) << LOG_ERROR_END;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Start background keepalive thread
    // m_keepalive_stop = false;
    // m_keepalive_thread = std::thread(&InexbotDriver::_keepalive_loop, this);

    int current_state = -1;
    bool already_powered_on = false;
    Result state_res = ::get_servo_state(m_fd, current_state);
    if (state_res == SUCCESS && current_state == 3) {
        LOG_INFO << "[startup] Servo is already powered on (state=3). Skipping power on." << std::endl;
        already_powered_on = true;
    }

    if (!already_powered_on) {
        // Power on
        res = ::set_servo_poweron(m_fd);
        LOG_INFO << "[API] set_servo_poweron() returned: " << result_to_string(res) << std::endl;
        if (res != SUCCESS) {
            shutdown();
            return false;
        }

        // Check if power on is successful
        bool poweron_success = false;
        for (int i = 0; i < 20; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            int check_state = -1;
            if (::get_servo_state(m_fd, check_state) == SUCCESS && check_state == 3) {
                poweron_success = true;
                break;
            }
        }

        if (!poweron_success) {
            LOG_ERROR << "[startup] Failed to verify servo power on (state did not reach 3)" << LOG_ERROR_END;
            shutdown();
            return false;
        }
        LOG_INFO << "[startup] Servo power on verified (state=3)" << std::endl;
    } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    LOG_INFO << "[startup] Robot startup successful and servo enabled" << std::endl;
    print_system_info();

    return true;
}

// Shutdown
void InexbotDriver::shutdown() {
    // m_keepalive_stop = true;
    // if (m_keepalive_thread.joinable()) {
    //     m_keepalive_thread.join();
    // }

    if (m_fd >= 0) {
        Result res = ::set_servo_poweroff(m_fd);
        LOG_INFO << "[API] set_servo_poweroff() returned: " << result_to_string(res) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        res = ::disconnect_robot(m_fd);
        LOG_INFO << "[API] disconnect_robot() returned: " << result_to_string(res) << std::endl;
        m_fd = -1;
    }
}

/*
// Keepalive Loop
void InexbotDriver::_keepalive_loop() {
    const int interval_ms = 3000;
    while (!m_keepalive_stop) {
        // Sleep in small chunks so we can wake up quickly
        for (int i = 0; i < interval_ms / 100; ++i) {
            if (m_keepalive_stop) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (m_keepalive_stop || m_fd < 0) break;

        try {
            int state = get_servo_state();
            RobotPose p = get_current_pose();
            LOG_INFO << "[*] Heartbeat - Servo: " << state 
                      << " | Pose: [" 
                      << std::fixed << std::setprecision(1) << p.x << ", " 
                      << p.y << ", " 
                      << p.z << ", " 
                      << std::setprecision(3) << p.a << ", " 
                      << p.b << ", " 
                      << p.c << "]" << std::endl;
        } catch (...) {
            // Ignore
        }
    }
}
*/

// Get Servo State
int InexbotDriver::get_servo_state() {
    if (m_fd < 0) return -1;
    int status = -1;
    Result res = ::get_servo_state(m_fd, status);
    LOG_INFO << "[API] get_servo_state() returned: " << result_to_string(res) << ", status: " << status << std::endl;
    if (res != SUCCESS) return -1;
    return status;
}

// Get Running State
int InexbotDriver::get_running_state() {
    if (m_fd < 0) return -1;
    int status = -1;
    Result res = ::get_robot_running_state(m_fd, status);
    LOG_INFO << "[API] get_robot_running_state() returned: " << result_to_string(res) << ", status: " << status << std::endl;
    if (res != SUCCESS) return -1;
    return status;
}

// Get Current Pose
RobotPose InexbotDriver::get_current_pose() {
    RobotPose pose;
    if (m_fd < 0) return pose;
    std::vector<double> pos;
    Result res = ::get_current_position(m_fd, COORD, pos);
    LOG_INFO << "[API] get_current_position(coord=" << COORD << ") returned: " << result_to_string(res) 
              << ", pos size: " << pos.size() << std::endl;
    if (res == SUCCESS && pos.size() >= 6) {
        pose = RobotPose::from_list(pos);
    }
    return pose;
}

// Is Reachable L
bool InexbotDriver::is_reachable_l(const RobotPose& pose) {
    return _is_reachable(pose, "MOVL");
}

// Is Reachable J
bool InexbotDriver::is_reachable_j(const RobotPose& pose) {
    return _is_reachable(pose, "MOVJ");
}

// Internal Reachable helper
bool InexbotDriver::_is_reachable(const RobotPose& pose, const std::string& movetype) {
    if (m_fd < 0) return false;
    std::vector<double> pose_vec(14, 0.0);
    pose_vec[0] = static_cast<double>(COORD);
    pose_vec[1] = static_cast<double>(ANGLE_UNIT);
    pose_vec[3] = static_cast<double>(m_tool_num);
    
    std::vector<double> pose_list = pose.to_list();
    for (size_t i = 0; i < 6; ++i) {
        pose_vec[7 + i] = pose_list[i];
    }

    bool result_bool = false;
    Result res = ::get_pos_reachable(m_fd, pose_vec, movetype, result_bool);
    LOG_INFO << "[API] get_pos_reachable(movetype=\"" << movetype 
              << "\") returned: " << result_to_string(res) 
              << ", reachable: " << (result_bool ? "true" : "false") << std::endl;
    if (res == SUCCESS) {
        return result_bool;
    }
    return false;
}

// MoveCmd Maker
MoveCmd InexbotDriver::_make_movecmd(const RobotPose& pose, double velocity, double acc, double dec, int tool_num, int user_num, int pl) {
    MoveCmd cmd;
    cmd.targetPosType = PosType::data;
    if (cmd.targetPosValue.size() < 14) {
        cmd.targetPosValue.resize(14, 0.0);
    }
    cmd.targetPosValue[0] = pose.x;
    cmd.targetPosValue[1] = pose.y;
    cmd.targetPosValue[2] = pose.z;
    cmd.targetPosValue[3] = pose.a;
    cmd.targetPosValue[4] = pose.b;
    cmd.targetPosValue[5] = pose.c;
    cmd.targetPosValue[6] = 0.0;
    for (int i = 7; i < 14; ++i) {
        cmd.targetPosValue[i] = 0.0;
    }
    cmd.coord = COORD;
    cmd.velocity = velocity;
    cmd.acc = acc;
    cmd.dec = dec;
    cmd.toolNum = (tool_num > 0) ? tool_num : m_tool_num;
    cmd.userNum = user_num;
    cmd.pl = pl;
    return cmd;
}

// Wait Motion Done
void InexbotDriver::_wait_motion_done(double poll_interval_sec) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (true) {
        if (get_running_state() == RUNNING_STATE_STOP) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(poll_interval_sec * 1000)));
    }
}

// Move J
int InexbotDriver::move_j(const RobotPose& pose, double velocity, double acc, double dec, int tool_num, bool wait) {
    MoveCmd cmd = _make_movecmd(pose, velocity, acc, dec, tool_num);
    Result ret = ::robot_movej(m_fd, cmd);
    LOG_INFO << "[API] robot_movej() returned: " << result_to_string(ret) << std::endl;
    if (ret != SUCCESS) {
        LOG_ERROR << "[move_j] Failed to send movej command, error code: " << result_to_string(ret) << LOG_ERROR_END;
        return static_cast<int>(ret);
    }
    if (wait) {
        _wait_motion_done();
        LOG_INFO << "[move_j] movej command sent successfully, target pose: "
                  << "RobotPose(x=" << pose.x << ", y=" << pose.y << ", z=" << pose.z 
                  << ", a=" << pose.a << ", b=" << pose.b << ", c=" << pose.c << ")" << std::endl;
    }
    return static_cast<int>(ret);
}

// Move L
int InexbotDriver::move_l(const RobotPose& pose, double velocity, double acc, double dec, int tool_num, bool wait) {
    MoveCmd cmd = _make_movecmd(pose, velocity, acc, dec, tool_num);
    Result ret = ::robot_movel(m_fd, cmd);
    LOG_INFO << "[API] robot_movel() returned: " << result_to_string(ret) << std::endl;
    if (ret != SUCCESS) {
        LOG_ERROR << "[move_l] Failed to send movel command, error code: " << result_to_string(ret) << LOG_ERROR_END;
        return static_cast<int>(ret);
    }
    if (wait) {
        _wait_motion_done();
        LOG_INFO << "[move_l] movel command sent successfully, target pose: "
                  << "RobotPose(x=" << pose.x << ", y=" << pose.y << ", z=" << pose.z 
                  << ", a=" << pose.a << ", b=" << pose.b << ", c=" << pose.c << ")" << std::endl;
    }
    return static_cast<int>(ret);
}

// Go Home
int InexbotDriver::go_home(bool wait) {
    Result ret = ::robot_go_home(m_fd);
    LOG_INFO << "[API] robot_go_home() returned: " << result_to_string(ret) << std::endl;
    if (ret != SUCCESS) {
        LOG_ERROR << "[go_home] Failed to send go_home command, error code: " << result_to_string(ret) << LOG_ERROR_END;
        return static_cast<int>(ret);
    }
    if (wait) {
        _wait_motion_done();
        LOG_INFO << "[go_home] go_home command sent successfully" << std::endl;
    }
    return static_cast<int>(ret);
}

// Queue Start
int InexbotDriver::queue_start() {
    m_queue_size = 0;
    Result ret = ::queue_motion_set_status(m_fd, true);
    LOG_INFO << "[API] queue_motion_set_status(true) returned: " << result_to_string(ret) << std::endl;
    if (ret != SUCCESS) {
        LOG_ERROR << "[queue_start] Failed to send queue_motion_set_status command, error code: " << result_to_string(ret) << LOG_ERROR_END;
    } else {
        LOG_INFO << "[queue_start] queue motion started successfully" << std::endl;
    }
    return static_cast<int>(ret);
}

// Queue Push L
int InexbotDriver::queue_push_l(const RobotPose& pose, double velocity, double acc, double dec, int tool_num, int pl) {
    MoveCmd cmd = _make_movecmd(pose, velocity, acc, dec, tool_num, 0, pl);
    Result ret = ::queue_motion_push_back_moveL(m_fd, cmd);
    LOG_INFO << "[API] queue_motion_push_back_moveL() returned: " << result_to_string(ret) << std::endl;
    if (ret != SUCCESS) {
        LOG_ERROR << "[queue_push_l] Failed to send queue_motion_push_back_moveL command, error code: " << result_to_string(ret) << LOG_ERROR_END;
        return static_cast<int>(ret);
    }
    m_queue_size++;
    return static_cast<int>(ret);
}

// Queue Push J
int InexbotDriver::queue_push_j(const RobotPose& pose, double velocity, double acc, double dec, int tool_num, int pl) {
    MoveCmd cmd = _make_movecmd(pose, velocity, acc, dec, tool_num, 0, pl);
    Result ret = ::queue_motion_push_back_moveJ(m_fd, cmd);
    LOG_INFO << "[API] queue_motion_push_back_moveJ() returned: " << result_to_string(ret) << std::endl;
    if (ret != SUCCESS) {
        LOG_ERROR << "[queue_push_j] Failed to send queue_motion_push_back_moveJ command, error code: " << result_to_string(ret) << LOG_ERROR_END;
        return static_cast<int>(ret);
    }
    m_queue_size++;
    return static_cast<int>(ret);
}

// Queue Send
int InexbotDriver::queue_send(bool wait) {
    if (m_queue_size == 0) {
        LOG_INFO << "[queue_send] No motion data in queue." << std::endl;
        return 0;
    }
    int ret = _queue_send_batched(wait);
    m_queue_size = 0;
    return ret;
}

// Queue Suspend
int InexbotDriver::queue_suspend() {
    Result ret = ::queue_motion_suspend(m_fd);
    LOG_INFO << "[API] queue_motion_suspend() returned: " << result_to_string(ret) << std::endl;
    if (ret != SUCCESS) {
        LOG_ERROR << "[queue_suspend] Failed to send queue_motion_suspend command, error code: " << result_to_string(ret) << LOG_ERROR_END;
    } else {
        LOG_INFO << "[queue_suspend] queue motion suspended successfully" << std::endl;
    }
    return static_cast<int>(ret);
}

// Queue Resume
int InexbotDriver::queue_resume() {
    Result ret = ::queue_motion_restart(m_fd);
    LOG_INFO << "[API] queue_motion_restart() returned: " << result_to_string(ret) << std::endl;
    if (ret != SUCCESS) {
        LOG_ERROR << "[queue_resume] Failed to send queue_motion_restart command, error code: " << result_to_string(ret) << LOG_ERROR_END;
    } else {
        LOG_INFO << "[queue_resume] queue motion restarted successfully" << std::endl;
    }
    return static_cast<int>(ret);
}

// Queue Stop
int InexbotDriver::queue_stop() {
    m_queue_size = 0;
    Result ret = ::queue_motion_stop_not_power_off(m_fd);
    LOG_INFO << "[API] queue_motion_stop_not_power_off() returned: " << result_to_string(ret) << std::endl;
    if (ret != SUCCESS) {
        LOG_ERROR << "[queue_stop] Failed to send queue_motion_stop_not_power_off command, error code: " << result_to_string(ret) << LOG_ERROR_END;
    } else {
        LOG_INFO << "[queue_stop] queue motion stopped successfully" << std::endl;
    }
    return static_cast<int>(ret);
}

// Queue Get Remaining
int InexbotDriver::queue_get_remaining() {
    if (m_fd < 0) return -1;
    int len = 0;
    Result res = ::queue_motion_get_queuelen(m_fd, len);
    LOG_INFO << "[API] queue_motion_get_queuelen() returned: " << result_to_string(res) << ", len: " << len << std::endl;
    if (res != SUCCESS) return static_cast<int>(res);
    return len;
}

// Execute Queue with RobotPoses and a vector of move types
int InexbotDriver::execute_queue(const std::vector<RobotPose>& poses,
                                 const std::string& move_type,
                                 double velocity,
                                 double acc,
                                 double dec,
                                 int tool_num,
                                 int pl,
                                 bool wait) {
    if (poses.empty()) {
        LOG_INFO << "[execute_queue] Empty poses list." << std::endl;
        return 0;
    }
    int ret = queue_start();
    if (ret != SUCCESS) return ret;

    for (size_t i = 0; i < poses.size(); ++i) {
        // Default: last point has pl=0 to stop at the destination
        int current_pl = pl;
        if (i == poses.size() - 1) {
            current_pl = 0;
        }

        if (move_type == "J" || move_type == "j" || move_type == "MOVEJ") {
            ret = queue_push_j(poses[i], velocity, acc, dec, tool_num, current_pl);
        } else {
            ret = queue_push_l(poses[i], velocity, acc, dec, tool_num, current_pl);
        }

        if (ret != SUCCESS) {
            queue_stop();
            return ret;
        }
    }
    int ret_send = queue_send(wait);
    if (wait) {
        Result ret_status = ::queue_motion_set_status(m_fd, false);
        if (ret_status != SUCCESS) {
            LOG_ERROR << "[execute_queue] Failed to disable queue motion status, error code: " << result_to_string(ret_status) << LOG_ERROR_END;
            if (ret_send == SUCCESS) {
                ret_send = static_cast<int>(ret_status);
            }
        }
    }
    return ret_send;
}

// Queue Send Batched
int InexbotDriver::_queue_send_batched(bool wait) {
    const int BATCH = 31;
    int total = m_queue_size;
    int sent = 0;
    Result ret = SUCCESS;

    while (sent < total) {
        int remaining = total - sent;
        int batch = std::min(remaining, BATCH);
        bool is_last = (sent + batch >= total);

        ret = ::queue_motion_send_to_controller(m_fd, batch, !is_last);
        LOG_INFO << "[API] queue_motion_send_to_controller(batch=" << batch << ", continue=" << (!is_last ? "true" : "false") 
                  << ") returned: " << result_to_string(ret) << std::endl;
        if (ret != SUCCESS) {
            LOG_ERROR << "[queue_send_batched] Failed on batch (cmds=" << batch 
                      << ", continue=" << (!is_last ? "true" : "false") 
                      << "), error code: " << result_to_string(ret) << LOG_ERROR_END;
            return static_cast<int>(ret);
        }
        sent += batch;
    }

    if (wait) {
        _wait_queue_done();
    }
    return static_cast<int>(ret);
}

// Wait Queue Done
void InexbotDriver::_wait_queue_done(double poll_interval_sec) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (true) {
        int qlen = queue_get_remaining();
        if (qlen <= 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(poll_interval_sec * 1000)));
    }

    while (true) {
        if (get_running_state() == RUNNING_STATE_STOP) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(poll_interval_sec * 1000)));
    }
    LOG_INFO << "[wait_queue_done] Queue motion completed." << std::endl;
}

// Print System Info
void InexbotDriver::print_system_info() {
    if (m_fd < 0) {
        LOG_INFO << "[print_system_info] Robot is not connected" << std::endl;
        return;
    }

    std::string ctrl_id = "--";
    std::vector<char> id_vec;
    Result res = ::get_controller_id_csharp(m_fd, id_vec);
    LOG_INFO << "[API] get_controller_id_csharp() returned: " << result_to_string(res) << std::endl;
    if (res == SUCCESS) {
        ctrl_id = std::string(id_vec.begin(), id_vec.end());
        while (!ctrl_id.empty() && ctrl_id.back() == '\0') {
            ctrl_id.pop_back();
        }
    }

    bool tb_connected = false;
    std::string tb_connected_str = "--";
    res = ::get_teachbox_connection_status(m_fd, tb_connected);
    LOG_INFO << "[API] get_teachbox_connection_status() returned: " << result_to_string(res) 
              << ", connected: " << (tb_connected ? "true" : "false") << std::endl;
    if (res == SUCCESS) {
        tb_connected_str = tb_connected ? "Connected" : "Disconnected";
    }

    int rtype = -1;
    std::string rtype_str = "--";
    res = ::get_robot_type(m_fd, rtype);
    LOG_INFO << "[API] get_robot_type() returned: " << result_to_string(res) << ", type: " << rtype << std::endl;
    if (res == SUCCESS) {
        switch (rtype) {
            case 1: rtype_str = "六轴串联多关节"; break;
            case 2: rtype_str = "四轴SCARA"; break;
            case 3: rtype_str = "四轴码垛"; break;
            case 4: rtype_str = "四轴串联多关节"; break;
            case 5: rtype_str = "单轴"; break;
            case 6: rtype_str = "五轴串联多关节"; break;
            case 7: rtype_str = "六轴协作"; break;
            case 8: rtype_str = "二轴scara"; break;
            case 9: rtype_str = "三轴scara"; break;
            case 10: rtype_str = "三轴直角"; break;
            case 11: rtype_str = "三轴异性"; break;
            case 12: rtype_str = "七轴串联多关节"; break;
            case 13: rtype_str = "scara异性一"; break;
            case 14: rtype_str = "四轴码垛丝杆"; break;
            default: rtype_str = "未知(" + std::to_string(rtype) + ")"; break;
        }
    }

    int coord = -1;
    std::string coord_str = "--";
    res = ::get_current_coord(m_fd, coord);
    LOG_INFO << "[API] get_current_coord() returned: " << result_to_string(res) << ", coord: " << coord << std::endl;
    if (res == SUCCESS) {
        switch (coord) {
            case 0: coord_str = "关节"; break;
            case 1: coord_str = "直角"; break;
            case 2: coord_str = "工具"; break;
            case 3: coord_str = "用户"; break;
            default: coord_str = "未知(" + std::to_string(coord) + ")"; break;
        }
    }

    int mode = -1;
    std::string mode_str = "--";
    res = ::get_current_mode(m_fd, mode);
    LOG_INFO << "[API] get_current_mode() returned: " << result_to_string(res) << ", mode: " << mode << std::endl;
    if (res == SUCCESS) {
        switch (mode) {
            case 0: mode_str = "示教"; break;
            case 1: mode_str = "远程"; break;
            case 2: mode_str = "运行"; break;
            default: mode_str = "未知(" + std::to_string(mode) + ")"; break;
        }
    }

    int speed = -1;
    std::string speed_str = "--";
    res = ::get_speed(m_fd, speed);
    LOG_INFO << "[API] get_speed() returned: " << result_to_string(res) << ", speed: " << speed << std::endl;
    if (res == SUCCESS) {
        speed_str = std::to_string(speed) + "%";
    }

    int servo_status = -1;
    std::string servo_status_str = "--";
    res = ::get_servo_state(m_fd, servo_status);
    LOG_INFO << "[API] get_servo_state() returned: " << result_to_string(res) << ", status: " << servo_status << std::endl;
    if (res == SUCCESS) {
        switch (servo_status) {
            case 0: servo_status_str = "停止"; break;
            case 1: servo_status_str = "就绪"; break;
            case 2: servo_status_str = "报警"; break;
            case 3: servo_status_str = "运行"; break;
            default: servo_status_str = "未知(" + std::to_string(servo_status) + ")"; break;
        }
    }

    int toolnum = -1;
    std::string toolnum_str = "--";
    res = ::get_tool_hand_number(m_fd, toolnum);
    LOG_INFO << "[API] get_tool_hand_number() returned: " << result_to_string(res) << ", tool: " << toolnum << std::endl;
    if (res == SUCCESS) {
        toolnum_str = std::to_string(toolnum);
    }

    std::vector<double> raw_joint;
    std::string joint_pose_str = "--";
    res = ::get_current_position(m_fd, 0, raw_joint);
    LOG_INFO << "[API] get_current_position(coord=0) returned: " << result_to_string(res) 
              << ", pos size: " << raw_joint.size() << std::endl;
    if (res == SUCCESS && !raw_joint.empty()) {
        RobotPose p = RobotPose::from_list(raw_joint);
        char buf[256];
        snprintf(buf, sizeof(buf), "RobotPose(x=%.3f, y=%.3f, z=%.3f, a=%.3f, b=%.3f, c=%.3f)", p.x, p.y, p.z, p.a, p.b, p.c);
        joint_pose_str = buf;
    }

    std::vector<double> raw_cart;
    std::string cart_pose_str = "--";
    res = ::get_current_position(m_fd, 1, raw_cart);
    LOG_INFO << "[API] get_current_position(coord=1) returned: " << result_to_string(res) 
              << ", pos size: " << raw_cart.size() << std::endl;
    if (res == SUCCESS && !raw_cart.empty()) {
        RobotPose p = RobotPose::from_list(raw_cart);
        char buf[256];
        snprintf(buf, sizeof(buf), "RobotPose(x=%.3f, y=%.3f, z=%.3f, a=%.3f, b=%.3f, c=%.3f)", p.x, p.y, p.z, p.a, p.b, p.c);
        cart_pose_str = buf;
    }

    LOG_INFO << "\n--------------------------------------------------\n"
              << "控制器ID: " << ctrl_id << "\n"
              << "示教器连接状态: " << tb_connected_str << "\n"
              << "机器人类型: " << rtype_str << "\n"
              << "坐标系: " << coord_str << "\n"
              << "运行模式: " << mode_str << "\n"
              << "当前全局速度: " << speed_str << "\n"
              << "伺服状态: " << servo_status_str << "\n"
              << "当前激活的工具编号: " << toolnum_str << "\n"
              << "当前关节坐标(上电后有效， 单位: °): " << joint_pose_str << "\n"
              << "当前末端位姿(直角坐标，单位: mm,°): " << cart_pose_str << "\n"
              << "--------------------------------------------------\n" << std::endl;
}
