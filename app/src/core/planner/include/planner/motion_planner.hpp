#pragma once

#include <cstddef>
#include <string>

namespace aisprayer::planner
{

/**
 * Runtime settings for the motion-only planner.
 *
 * Motion planning uses Tesseract's KDL solver with joint limits. It is linked
 * with the existing Tesseract dependency stack and does not require ROS.
 */
struct MotionPlannerConfig
{
  std::string input_path;
  std::string urdf_path;
  std::string srdf_path;
  std::string output_directory;
  std::string manipulator_group;
  std::string tcp_frame;
  std::string base_link{ "base_link" };
  std::string angle_unit{ "deg" };
  std::size_t thread_count{ 6 };
  double position_tolerance{ 0.005 };
  double orientation_tolerance{ 15.0 };
  bool ik_only{ false };

};

/**
 * Loads TCP strokes, solves them with KDL, plans LINEAR spray segments in
 * parallel, then plans collision-checked FREESPACE transitions serially.
 *
 * Throws std::runtime_error for invalid input or planning failures. On failure,
 * no trajectory.json is written.
 */
void runMotionPlanner(const MotionPlannerConfig& config);

}  // namespace aisprayer::planner
