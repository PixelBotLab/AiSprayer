#pragma once

#include <string>
#include <vector>
#include <optional>
#include <nlohmann/json.hpp>

namespace aisprayer::planner {

struct ProcessConfig {
    double standoff = 0.20;
    double row_spacing = 0.04;
    double point_spacing = 0.01;
    bool straight_lines = false;
    double seam_dedup_distance = -1.0; // Negative means default to row_spacing * 0.5
    std::string direction = "";
    std::string image_horizontal = "";
    int smoothing_window = 15;
    double merge_gap_threshold = 0.06;
};

class ProcessPlanner {
public:
    explicit ProcessPlanner(const ProcessConfig& config);

    std::optional<nlohmann::json> plan(const std::vector<std::string>& mesh_paths, const std::string& calib_path);
    bool save(const nlohmann::json& process_targets, const std::string& output_file) const;

private:
    ProcessConfig config_;
};

} // namespace aisprayer::planner
