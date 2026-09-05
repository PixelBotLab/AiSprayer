#pragma once

#include "visioncpp/mesh.hpp"
#include "visioncpp/types.hpp"

#include <string>

namespace visioncpp {

class MaskSet;

struct AutoPathOptions {
    double spray_dist_mm = 150.0;
    double row_spacing_mm = 40.0;
    double point_spacing_mm = 100.0;
    double dedup_radius_mm = 20.0;
    double depth_threshold_ratio = 0.1;
    int normal_smooth_window = 5;
    bool align_outer_edge = true;
};

class AutoPathPlanner {
public:
    static PathDoc plan(const Mesh& mesh,
                        const MaskSet& masks,
                        const CameraIntrinsics& k,
                        const Mat4& T_camera_to_base,
                        const AutoPathOptions& opt);

    static void writeYaml(const std::string& path, const PathDoc& doc);
};

}  // namespace visioncpp
