#pragma once

#include "visioncpp/mesh.hpp"
#include "visioncpp/point_cloud.hpp"

namespace visioncpp {

class PoissonRecon {
public:
    static Mesh reconstruct(const PointCloud& cloud, int depth, double density_threshold);
};

Mesh hoppeReconstruct(const PointCloud& cloud, double spacing, double trim_quantile);

}  // namespace visioncpp
