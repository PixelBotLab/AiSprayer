#include "planner/motion_planner.hpp"

#include <cxxopts.hpp>

#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

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
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    cxxopts::Options options("motion_planner", "KDL and TrajOpt motion planner for TCP spray strokes");
    options.add_options()
        ("input", "Input tcp_targets.json", cxxopts::value<std::string>())
        ("urdf", "Robot URDF file", cxxopts::value<std::string>())
        ("srdf", "Robot SRDF file", cxxopts::value<std::string>())
        ("o,outdir", "Output directory", cxxopts::value<std::string>())
        ("group", "Manipulator group name", cxxopts::value<std::string>())
        ("tcp", "TCP frame name", cxxopts::value<std::string>())
        ("base-link", "Manipulator base frame", cxxopts::value<std::string>()->default_value("base_link"))
        ("position-tolerance",
         "Cartesian position tolerance in metres",
         cxxopts::value<double>()->default_value("0.005"))
        ("orientation-tolerance",
         "Cartesian orientation tolerance in degrees",
         cxxopts::value<double>()->default_value("15.0"))
        ("angle-unit", "Joint output unit: deg or rad", cxxopts::value<std::string>()->default_value("deg"))
        ("threads", "Positive worker count", cxxopts::value<std::string>()->default_value("6"))
        ("ik-only", "Validate TCP targets with KDL and write reachable joint seeds without TrajOpt",
         cxxopts::value<bool>()->default_value("false"))
        ("h,help", "Print usage");

    const auto arguments = options.parse(argc, argv);
    if (arguments.count("help"))
    {
      std::cout << options.help() << '\n';
      return 0;
    }

    constexpr const char* required_options[] = { "input", "urdf", "srdf", "outdir", "group", "tcp" };
    for (const char* option : required_options)
    {
      if (!arguments.count(option))
      {
        std::cerr << "Missing required option --" << option << "\n\n" << options.help() << '\n';
        return 2;
      }
    }

    aisprayer::planner::MotionPlannerConfig config;
    config.input_path = arguments["input"].as<std::string>();
    config.urdf_path = arguments["urdf"].as<std::string>();
    config.srdf_path = arguments["srdf"].as<std::string>();
    config.output_directory = arguments["outdir"].as<std::string>();
    config.manipulator_group = arguments["group"].as<std::string>();
    config.tcp_frame = arguments["tcp"].as<std::string>();
    config.base_link = arguments["base-link"].as<std::string>();
    config.position_tolerance = arguments["position-tolerance"].as<double>();
    config.orientation_tolerance = arguments["orientation-tolerance"].as<double>();
    config.angle_unit = arguments["angle-unit"].as<std::string>();
    config.thread_count = parseThreadCount(arguments["threads"].as<std::string>());
    config.ik_only = arguments["ik-only"].as<bool>();

    aisprayer::planner::runMotionPlanner(config);
    return 0;
  }
  catch (const cxxopts::exceptions::exception& error)
  {
    std::cerr << "Invalid command line: " << error.what() << '\n';
  }
  catch (const std::exception& error)
  {
    std::cerr << "Motion planning failed: " << error.what() << '\n';
  }
  return 1;
}
