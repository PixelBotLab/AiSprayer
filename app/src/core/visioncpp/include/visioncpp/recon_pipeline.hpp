#pragma once

#include "visioncpp/types.hpp"

#include <string>
#include <vector>

namespace visioncpp {

struct ReconOptions {
    int poisson_depth = 8;
    double density_threshold = 0.15;
    double voxel_size = 0.003;
    double normal_radius = 0.03;
    int smooth_iterations = 20;
    double z_min_mm = 100.0;
    double z_max_mm = 3000.0;
    int mask_erode_px = 1;
    double flying_pixel_max_grad = 50.0;
};

struct ReconResult {
    int vertices = 0;
    int faces = 0;
    double elapsed_ms = 0;
    std::vector<std::string> files;
};

class ReconPipeline {
public:
    static ReconResult run(const std::string& template_dir,
                           const std::string& calib_path,
                           const std::string& output_dir,
                           const ReconOptions& opt);
};

}  // namespace visioncpp
