#pragma once

#include "visioncpp/types.hpp"

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace visioncpp {

class MaskSet {
public:
    static MaskSet loadYaml(const std::string& path, int height, int width);

    const cv::Mat& mask() const { return mask_; }
    cv::Mat asBoolU8() const;

    std::vector<cv::Mat> splitLegs(double depth_threshold_ratio = 0.1,
                                   double overlap_px = 0.0) const;

private:
    cv::Mat mask_;  // CV_8U 0/255
};

}  // namespace visioncpp
