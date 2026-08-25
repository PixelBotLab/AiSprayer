#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include "logger.hpp"

namespace orbbec_service {

class RgaProcessor {
public:
    RgaProcessor();
    ~RgaProcessor();

    bool init(int width, int height);
    void release();

    // Convert BGR888 or RGB888 image to NV12 (YUV420SP)
    bool bgrToNv12(const cv::Mat& src_bgr, uint8_t* dst_nv12, int dst_width = 0, int dst_height = 0);
    bool rgbToNv12(const uint8_t* src_rgb, int src_w, int src_h, uint8_t* dst_nv12, int dst_w, int dst_h);

    bool isRgaAvailable() const { return rga_available_; }

private:
    bool rga_available_ = false;
    int src_width_ = 0;
    int src_height_ = 0;
    std::mutex mutex_;
};

} // namespace orbbec_service
