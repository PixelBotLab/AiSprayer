#include "process_planner.hpp"
#include "motion_planner.hpp"

#include <cxxopts.hpp>

#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

namespace
{
[[nodiscard]] std::size_t parseThreadCount(const std::string& value)
{
  std::size_t parsed_characters = 0;
  unsigned long long parsed = 0;
  try
  {
    parsed = std::stoull(value, &parsed_characters);
  }
  catch (const std::exception&)
  {
    throw std::runtime_error("--threads must be a positive integer");
  }
  if (parsed_characters != value.size() || parsed == 0 ||
      parsed > static_cast<unsigned long long>(std::numeric_limits<std::size_t>::max()))
    throw std::runtime_error("--threads must be a positive integer");
  return static_cast<std::size_t>(parsed);
}

std::vector<std::string> split(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

}  // namespace

int main(int argc, char** argv)
{
  try
  {
    cxxopts::Options options("aisprayer_planner", "AiSprayer unified planner for process and motion");
    
    // Unified options
    options.add_options("General")
        ("h,help", "Print usage")
        ("o,output", "Output JSON file (e.g. trajectory.json)", cxxopts::value<std::string>());

    // Process Planner options
    options.add_options("Process")
        ("m,mesh", "Input OBJ path(s), comma-separated and processed in order", cxxopts::value<std::string>())
        ("distance", "Spray standoff in meters", cxxopts::value<double>()->default_value("0.20"))
        ("r,row-spacing", "Raster row spacing in meters", cxxopts::value<double>()->default_value("0.10"))
        ("p,point-spacing", "Waypoint spacing in meters", cxxopts::value<double>()->default_value("0.01"))
        ("straight-lines", "Replace raster segments with straight start-to-end strokes",
         cxxopts::value<bool>()->default_value("false"))
        ("d,direction", "Raster direction vector: x,y,z", cxxopts::value<std::string>()->default_value(""))
        ("image-horizontal", "Image-right axis in robot-base coordinates: x,y,z",
         cxxopts::value<std::string>()->default_value(""))
        ("calibration", "Calibration YAML with T_base_camera",
         cxxopts::value<std::string>()->default_value(""))
        ("seam-dedup-distance", "Cross-mesh de-duplication distance in meters",
         cxxopts::value<double>()->default_value("-1"));

    // Motion Planner options
    options.add_options("Motion")
        ("motion", "Enable motion planning pipeline", cxxopts::value<bool>()->default_value("false"))
        ("urdf", "Robot URDF file", cxxopts::value<std::string>())
        ("srdf", "Robot SRDF file", cxxopts::value<std::string>())
        ("group", "Manipulator group name", cxxopts::value<std::string>())
        ("tcp", "TCP frame name", cxxopts::value<std::string>())
        ("base-link", "Manipulator base frame", cxxopts::value<std::string>()->default_value("base_link"))
        ("position-tolerance", "Cartesian position tolerance in metres",
         cxxopts::value<double>()->default_value("0.005"))
        ("orientation-tolerance", "Cartesian orientation tolerance in degrees",
         cxxopts::value<double>()->default_value("15.0"))
        ("angle-unit", "Joint output unit: deg or rad", cxxopts::value<std::string>()->default_value("deg"))
        ("threads", "Positive worker count", cxxopts::value<std::string>()->default_value("6"))
        ("ik-only", "Validate TCP targets with KDL and write reachable joint seeds without TrajOpt",
         cxxopts::value<bool>()->default_value("false"));

    const auto arguments = options.parse(argc, argv);
    if (arguments.count("help"))
    {
      std::cout << options.help() << '\n';
      return 0;
    }

    if (!arguments.count("mesh"))
    {
      std::cerr << "Missing required general/process options: --mesh\n\n" << options.help() << '\n';
      return 2;
    }
    
    bool run_motion = arguments["motion"].as<bool>();
    if (run_motion)
    {
      constexpr const char* required_motion_options[] = { "urdf", "srdf", "group", "tcp" };
      for (const char* option : required_motion_options)
      {
        if (!arguments.count(option))
        {
          std::cerr << "Missing required motion option when --motion is enabled: --" << option << "\n\n";
          return 2;
        }
      }
    }

    // 1. Process Planning
    aisprayer::planner::ProcessConfig process_config;
    process_config.standoff = arguments["distance"].as<double>();
    process_config.row_spacing = arguments["row-spacing"].as<double>();
    process_config.point_spacing = arguments["point-spacing"].as<double>();
    process_config.straight_lines = arguments["straight-lines"].as<bool>();
    process_config.direction = arguments["direction"].as<std::string>();
    process_config.image_horizontal = arguments["image-horizontal"].as<std::string>();
    process_config.seam_dedup_distance = arguments["seam-dedup-distance"].as<double>();

    std::string mesh_str = arguments["mesh"].as<std::string>();
    std::vector<std::string> mesh_paths = split(mesh_str, ',');
    std::string calib_path = arguments["calibration"].as<std::string>();

    std::cout << "--- Starting Process Planning ---\n";
    aisprayer::planner::ProcessPlanner process_planner(process_config);
    
    auto process_targets = process_planner.plan(mesh_paths, calib_path);
    if (!process_targets) {
        std::cerr << "Process planning failed.\n";
        return 1;
    }
    
    if (!run_motion && arguments.count("output")) {
        std::string out_file = arguments["output"].as<std::string>();
        if (!process_planner.save(*process_targets, out_file)) {
            std::cerr << "Failed to save process_targets.\n";
            return 1;
        }
    }
    std::cout << "Process planning completed successfully.\n";

    nlohmann::json final_json = *process_targets;

    // 2. Motion Planning (Optional)
    if (run_motion)
    {
        std::cout << "\n--- Starting Motion Planning ---\n";
        aisprayer::planner::MotionConfig motion_config;
        motion_config.urdf_path = arguments["urdf"].as<std::string>();
        motion_config.srdf_path = arguments["srdf"].as<std::string>();
        motion_config.manipulator_group = arguments["group"].as<std::string>();
        motion_config.tcp_frame = arguments["tcp"].as<std::string>();
        motion_config.base_link = arguments["base-link"].as<std::string>();
        motion_config.position_tolerance = arguments["position-tolerance"].as<double>();
        motion_config.orientation_tolerance = arguments["orientation-tolerance"].as<double>();
        motion_config.angle_unit = arguments["angle-unit"].as<std::string>();
        motion_config.thread_count = parseThreadCount(arguments["threads"].as<std::string>());
        motion_config.ik_only = arguments["ik-only"].as<bool>();

        aisprayer::planner::MotionPlanner motion_planner(motion_config);
        
        // Connect the pipeline by passing the JSON directly in memory
        auto trajectory = motion_planner.plan(*process_targets);
        if (!trajectory) {
            std::cerr << "Motion planning failed.\n";
            return 1;
        }
        
        if (arguments.count("output")) {
            std::string out_file = arguments["output"].as<std::string>();
            if (!motion_planner.save(*trajectory, out_file)) {
                std::cerr << "Failed to save trajectory.\n";
                return 1;
            }
        }
        std::cout << "Motion planning completed successfully.\n";
        
        final_json = *trajectory;
    }
    else
    {
        std::cout << "\nMotion planning skipped (--motion not specified).\n";
    }

    // --- Print Statistics ---
    std::cout << "\n=== Summary Statistics ===\n";
    if (final_json.contains("strokes")) {
        auto& strokes = final_json["strokes"];
        std::size_t num_strokes = strokes.size();
        std::size_t total_uv = 0;
        std::size_t total_surface = 0;
        std::size_t total_tcp = 0;
        std::size_t total_joints = 0;
        
        std::cout << "Total Strokes: " << num_strokes << "\n";
        
        for (std::size_t i = 0; i < num_strokes; ++i) {
            std::size_t stroke_uv = 0;
            std::size_t stroke_surface = 0;
            std::size_t stroke_tcp = 0;
            std::size_t stroke_joints = 0;
            
            if (strokes[i].contains("points")) {
                auto& points = strokes[i]["points"];
                for (const auto& pt : points) {
                    if (pt.contains("u") && pt.contains("v")) stroke_uv++;
                    if (pt.contains("surface_x")) stroke_surface++;
                    if (pt.contains("x")) stroke_tcp++;
                    if (pt.contains("joint_positions")) stroke_joints++;
                }
            }
            std::cout << "  Stroke " << i << " points -> UV: " << stroke_uv 
                      << ", Surface: " << stroke_surface 
                      << ", TCP: " << stroke_tcp 
                      << ", Joints: " << stroke_joints << "\n";
            
            total_uv += stroke_uv;
            total_surface += stroke_surface;
            total_tcp += stroke_tcp;
            total_joints += stroke_joints;
        }
        
        std::cout << "Total Points -> UV: " << total_uv 
                  << ", Surface: " << total_surface 
                  << ", TCP: " << total_tcp 
                  << ", Joints: " << total_joints << "\n";

        if (final_json.contains("ik_stats")) {
            std::cout << "\n  --- IK Prefilter Stats ---\n"
                      << "  Exact match: " << final_json["ik_stats"].value("exact_match", 0) << "\n"
                      << "  Rescued via Z-axis sampling: " << final_json["ik_stats"].value("rescued_via_z_axis_sampling", 0) << "\n"
                      << "  Failed: " << final_json["ik_stats"].value("failed", 0) << "\n";
        }
    }
    std::cout << "==========================\n\n";

    return 0;
  }
  catch (const cxxopts::exceptions::exception& error)
  {
    std::cerr << "Invalid command line: " << error.what() << '\n';
  }
  catch (const std::exception& error)
  {
    std::cerr << "Planner execution failed: " << error.what() << '\n';
  }
  return 1;
}
