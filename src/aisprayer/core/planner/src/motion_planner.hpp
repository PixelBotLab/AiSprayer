#pragma once

#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <nlohmann/json.hpp>

// Forward declarations to avoid heavy includes in the header
namespace tesseract::environment { class Environment; }
namespace tesseract::kinematics { class KinematicGroup; }

namespace aisprayer::planner {

struct MotionConfig {
    std::string urdf_path;
    std::string srdf_path;
    std::string manipulator_group;
    std::string base_link;
    std::string tcp_frame;
    double position_tolerance = 0.02;
    double orientation_tolerance = 1.0;
    std::string angle_unit = "deg";
    int thread_count = 1;
    bool ik_only = false;
};

// Abstract the final trajectory point
struct TrajectoryPoint {
    std::vector<double> joints;
    std::string motion_type;
    bool segment_start = false;
};

class MotionPlanner {
public:
    explicit MotionPlanner(const MotionConfig& config);
    ~MotionPlanner();

    std::optional<nlohmann::json> plan(const std::string& tcp_targets_file);
    std::optional<nlohmann::json> plan(const nlohmann::json& input_json);
    bool save(const nlohmann::json& trajectory, const std::string& output_file) const;

private:
    MotionConfig config_;
};

} // namespace aisprayer::planner
