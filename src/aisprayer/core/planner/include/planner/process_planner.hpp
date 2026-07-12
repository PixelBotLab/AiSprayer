#pragma once

#include <string>

namespace aisprayer::planner
{

/**
 * Parameters used to produce surface and TCP spray paths.
 *
 * Meshes are specified as one comma-separated list to preserve their input
 * order, which is required for deterministic seam de-duplication.
 */
struct ProcessPlannerOptions
{
  std::string mesh_paths;
  std::string output_directory;
  double standoff{ 0.20 };
  double row_spacing{ 0.10 };
  double point_spacing{ 0.01 };
  bool straight_lines{ false };
  std::string direction;
  std::string image_horizontal;
  std::string calibration_path;
  double seam_dedup_distance{ -1.0 };
};

/**
 * Generates Noether surface paths and their transformed TCP targets.
 *
 * On success, writes path_surface.json and tcp_targets.json to
 * ProcessPlannerOptions::output_directory. Throws std::exception on invalid
 * input or a mesh/path generation failure.
 */
class ProcessPlanner
{
public:
  explicit ProcessPlanner(ProcessPlannerOptions options);

  void run() const;

private:
  ProcessPlannerOptions options_;
};

}  // namespace aisprayer::planner
