#include <iostream>
#include <chrono>
#include <thread>
#include "cpp/parameter/nrc_define.h"
#include "inexbot_driver.h"

int main(int argc, char* argv[]) {
    std::string ip = "192.168.2.14";
    std::string port = "6001";
    if (argc > 1) {
        ip = argv[1];
    }
    if (argc > 2) {
        port = argv[2];
    }

    LOG_INFO << "=========================================" << std::endl;
    LOG_INFO << "  iNexbot C++ SDK 24.03 Driver Demo" << std::endl;
    LOG_INFO << "=========================================" << std::endl;
    LOG_INFO << "Connecting to robot at " << ip << ":" << port << std::endl;

    InexbotDriver robot(ip, port);
    if (!robot.startup()) {
        LOG_ERROR << "Failed to startup robot connection and power on. Exiting." << LOG_ERROR_END;
        return -1;
    }

    // Get current Cartesian Pose
    RobotPose start_pose = robot.get_current_pose();
    LOG_INFO << "Successfully started. Current Pose: x=" << start_pose.x 
              << ", y=" << start_pose.y << ", z=" << start_pose.z 
              << ", a=" << start_pose.a << ", b=" << start_pose.b << ", c=" << start_pose.c << std::endl;

    // Test a basic point-to-point move if reachable (using a small offset from starting pose)
    // Be very safe by keeping offsets small or querying reachability
    RobotPose target1 = start_pose;
    target1.z -= 200.0; // move down by 200mm

    LOG_INFO << "Checking reachability of target 1 (200mm lower) for MOVL: " << std::endl;
    if (robot.is_reachable_l(target1)) {
        LOG_INFO << "Target 1 is reachable. Executing move_l..." << std::endl;
        robot.move_l(target1, 20.0, 50.0, 50.0); // low velocity 20mm/s for safety
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Move back to starting pose
        LOG_INFO << "Moving back to start pose..." << std::endl;
        robot.move_l(start_pose, 20.0, 50.0, 50.0);
    } else {
        LOG_ERROR << "Target 1 is not reachable for MOVL!" << LOG_ERROR_END;
    }

    // Test move_j
    LOG_INFO << "Checking reachability of target 1 (200mm lower) for MOVJ: " << std::endl;
    if (robot.is_reachable_j(target1)) {
        LOG_INFO << "Target 1 is reachable. Executing move_j..." << std::endl;
        robot.move_j(target1, 10.0, 50.0, 50.0); // low velocity 10% for safety
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Move back to starting pose
        LOG_INFO << "Moving back to start pose..." << std::endl;
        robot.move_j(start_pose, 10.0, 50.0, 50.0);
    } else {
        LOG_ERROR << "Target 1 is not reachable for MOVJ!" << LOG_ERROR_END;
    }

    // Move back to home pose
    LOG_INFO << "Moving back to home pose..." << std::endl;
    int ret = robot.go_home();
    if (SUCCESS != ret) {
        LOG_ERROR << "Moving back to home failed: " << ret << LOG_ERROR_END;
    } else {
        LOG_INFO << "Moving back to home success" << std::endl;
    }

    // Test Queue motion using the new execute_queue helper
    LOG_INFO << "\nTesting queue motion using execute_queue with MOVL (a small triangle)..." << std::endl;
    RobotPose pt1 = start_pose;
    pt1.x += 100.0;
    
    RobotPose pt2 = start_pose;
    pt2.x += 100.0;
    pt2.y += 100.0;
    
    RobotPose pt3 = start_pose; // back to start

    if (robot.is_reachable_l(pt1) && robot.is_reachable_l(pt2)) {
        std::vector<RobotPose> poses = {pt1, pt2, pt3};
        LOG_INFO << "Sending MOVL queue motion..." << std::endl;
        robot.execute_queue(poses, "L", 30.0, 50.0, 50.0, 0, 1, true);
        LOG_INFO << "MOVL queue motion finished." << std::endl;
    } else {
        LOG_ERROR << "MOVL queue motion points are not reachable!" << LOG_ERROR_END;
    }

    LOG_INFO << "\nTesting queue motion using execute_queue with MOVJ (a small triangle)..." << std::endl;
    if (robot.is_reachable_j(pt1) && robot.is_reachable_j(pt2)) {
        std::vector<RobotPose> poses = {pt1, pt2, pt3};
        LOG_INFO << "Sending MOVJ queue motion..." << std::endl;
        robot.execute_queue(poses, "J", 10.0, 50.0, 50.0, 0, 1, true);
        LOG_INFO << "MOVJ queue motion finished." << std::endl;
    } else {
        LOG_ERROR << "MOVJ queue motion points are not reachable!" << LOG_ERROR_END;
    }

    RobotPose current_pose = robot.get_current_pose();
    LOG_INFO << "Current Pose: x=" << current_pose.x 
              << ", y=" << current_pose.y << ", z=" << current_pose.z 
              << ", a=" << current_pose.a << ", b=" << current_pose.b << ", c=" << current_pose.c << std::endl;
    // compare start_pose and current_pose
    if (current_pose == start_pose) {
        LOG_INFO << "Current Pose is equal to start Pose" << std::endl;
    } else {
        LOG_WARN << "Current Pose is not equal to start Pose" << LOG_WARN_END;
    }

    LOG_INFO << "Shutting down driver..." << std::endl;
    robot.shutdown();
    LOG_INFO << "Demo finished." << std::endl;
    return 0;
}
