#pragma once

#include "visioncpp/depth_map.hpp"
#include "visioncpp/types.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace visioncpp {

class PointCloud {
public:
    std::vector<Vec3> points;
    std::vector<Vec3> normals;

    static PointCloud fromDepth(const DepthMap& depth,
                                const CameraIntrinsics& k,
                                const cv::Mat& mask_u8);

    void voxelDownsample(double voxel_size);
    void removeNonFinite();
    void removeStatisticalOutliers(int nb_neighbors = 20, double std_ratio = 2.0);
    void estimateNormals(double radius, int max_nn);
    void orientNormalsTowardsOrigin();
    void bounds(Vec3& bmin, Vec3& bmax, size_t* finite_count = nullptr) const;
};

}  // namespace visioncpp
