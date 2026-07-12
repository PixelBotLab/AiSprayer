#include <planner/process_planner.hpp>

#include <exception>
#include <iostream>
#include <string>
#include <utility>

#include <cxxopts.hpp>

int main(int argc, char** argv)
{
  try
  {
    cxxopts::Options options("process_planner", "Noether surface-process planner");
    options.add_options()
        ("m,mesh", "Input OBJ path(s), comma-separated and processed in order", cxxopts::value<std::string>())
        ("o,outdir", "Directory for path_surface.json and tcp_targets.json", cxxopts::value<std::string>())
        ("distance", "Spray standoff in meters", cxxopts::value<double>()->default_value("0.20"))
        ("r,row-spacing", "Raster row spacing in meters", cxxopts::value<double>()->default_value("0.10"))
        ("p,point-spacing", "Waypoint spacing in meters", cxxopts::value<double>()->default_value("0.01"))
        ("straight-lines", "Replace raster segments with straight start-to-end strokes",
         cxxopts::value<bool>()->default_value("false"))
        ("d,direction", "Raster direction vector: x,y,z", cxxopts::value<std::string>()->default_value(""))
        ("image-horizontal", "Image-right axis in robot-base coordinates: x,y,z",
         cxxopts::value<std::string>()->default_value(""))
        ("calibration", "Calibration YAML with T_base_camera; its first rotation column is image-right",
         cxxopts::value<std::string>()->default_value(""))
        ("seam-dedup-distance", "Cross-mesh de-duplication distance in meters; negative uses half row spacing",
         cxxopts::value<double>()->default_value("-1"))
        ("h,help", "Print usage");

    const cxxopts::ParseResult result = options.parse(argc, argv);
    if (result.count("help") || !result.count("mesh") || !result.count("outdir"))
    {
      std::cout << options.help() << '\n';
      return result.count("help") ? 0 : 2;
    }

    aisprayer::planner::ProcessPlannerOptions process_options;
    process_options.mesh_paths = result["mesh"].as<std::string>();
    process_options.output_directory = result["outdir"].as<std::string>();
    process_options.standoff = result["distance"].as<double>();
    process_options.row_spacing = result["row-spacing"].as<double>();
    process_options.point_spacing = result["point-spacing"].as<double>();
    process_options.straight_lines = result["straight-lines"].as<bool>();
    process_options.direction = result["direction"].as<std::string>();
    process_options.image_horizontal = result["image-horizontal"].as<std::string>();
    process_options.calibration_path = result["calibration"].as<std::string>();
    process_options.seam_dedup_distance = result["seam-dedup-distance"].as<double>();

    aisprayer::planner::ProcessPlanner(std::move(process_options)).run();
    return 0;
  }
  catch (const std::exception& exception)
  {
    std::cerr << "Process planning failed: " << exception.what() << '\n';
    return 1;
  }
}
