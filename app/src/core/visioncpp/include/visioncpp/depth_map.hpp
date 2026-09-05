#pragma once

#include "visioncpp/types.hpp"

#include <opencv2/core.hpp>
#include <string>

namespace visioncpp {

class DepthMap {
public:
    static DepthMap loadFromTemplate(const std::string& template_dir);

    int rows() const { return depth_.rows; }
    int cols() const { return depth_.cols; }
    const cv::Mat& mat() const { return depth_; }
    cv::Mat& mat() { return depth_; }

    void inpaintHoles(const cv::Mat& hole_mask_u8);

private:
    cv::Mat depth_;  // CV_32F, millimeters
};

}  // namespace visioncpp
