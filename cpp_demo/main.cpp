#include <iostream>
#include <chrono>
#include <thread>
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
    target1.z += 10.0; // move up by 10mm

    LOG_INFO << "Checking reachability of target 1 (10mm higher): " << std::endl;
    if (robot.is_reachable(target1, "MOVL")) {
        LOG_INFO << "Target 1 is reachable. Executing move_l..." << std::endl;
        robot.move_l(target1, 20.0, 50.0, 50.0); // low velocity 20mm/s for safety
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Move back to starting pose
        LOG_INFO << "Moving back to start pose..." << std::endl;
        robot.move_l(start_pose, 20.0, 50.0, 50.0);
    } else {
        LOG_ERROR << "Target 1 is not reachable!" << LOG_ERROR_END;
    }

    // Test Queue motion
    LOG_INFO << "\nTesting queue motion (a small triangle)..." << std::endl;
    RobotPose pt1 = start_pose;
    pt1.x += 10.0;
    
    RobotPose pt2 = start_pose;
    pt2.x += 10.0;
    pt2.y += 10.0;
    
    RobotPose pt3 = start_pose; // back to start

    if (robot.is_reachable(pt1, "MOVL") && robot.is_reachable(pt2, "MOVL")) {
        robot.queue_start();
        robot.queue_push_l(pt1, 30.0, 50.0, 50.0, 0, 1); // pl=1 for blending
        robot.queue_push_l(pt2, 30.0, 50.0, 50.0, 0, 1);
        robot.queue_push_l(pt3, 30.0, 50.0, 50.0, 0, 0); // pl=0 to stop at the end
        LOG_INFO << "Sending queue motion..." << std::endl;
        robot.queue_send(true); // wait for completion
        LOG_INFO << "Queue motion finished." << std::endl;
    } else {
        LOG_ERROR << "Queue motion points are not reachable!" << LOG_ERROR_END;
    }

    LOG_INFO << "Shutting down driver..." << std::endl;
    robot.shutdown();
    LOG_INFO << "Demo finished." << std::endl;
    return 0;
}
