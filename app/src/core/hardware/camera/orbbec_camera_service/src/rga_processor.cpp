#include "rga_processor.hpp"
#include <rga/im2d.hpp>
#include <rga/rga.h>
#include <rga/RgaApi.h>
#include <opencv2/imgproc.hpp>
#include <fcntl.h>
#include <unistd.h>

namespace orbbec_service {

RgaProcessor::RgaProcessor() {}

RgaProcessor::~RgaProcessor() {
    release();
}

bool RgaProcessor::init(int width, int height) {
    std::lock_guard<std::mutex> lock(mutex_);
    src_width_ = width;
    src_height_ = height;

    int fd = open("/dev/rga", O_RDWR);
    if (fd >= 0) {
        close(fd);
        rga_available_ = true;
        c_RkRgaInit();
        LOG_INFO("RGA", "Rockchip RGA 2D hardware acceleration initialized successfully. Base resolution: ", 
                 width, "x", height);
    } else {
        rga_available_ = false;
        LOG_WARN("RGA", "Cannot open /dev/rga. Falling back to CPU software color space conversion.");
    }
    return true;
}

void RgaProcessor::release() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (rga_available_) {
        c_RkRgaDeInit();
        rga_available_ = false;
        LOG_INFO("RGA", "RGA hardware context released.");
    }
}

bool RgaProcessor::bgrToNv12(const cv::Mat& src_bgr, uint8_t* dst_nv12, int dst_width, int dst_height) {
    if (src_bgr.empty() || dst_nv12 == nullptr) {
        LOG_ERROR("RGA", "Invalid input or destination buffer for bgrToNv12");
        return false;
    }

    int in_w = src_bgr.cols;
    int in_h = src_bgr.rows;
    int out_w = (dst_width > 0) ? dst_width : in_w;
    int out_h = (dst_height > 0) ? dst_height : in_h;

    std::lock_guard<std::mutex> lock(mutex_);

    if (rga_available_) {
        rga_buffer_t src_buf = wrapbuffer_virtualaddr(
            (void*)src_bgr.data, in_w, in_h, RK_FORMAT_BGR_888, in_w, in_h
        );
        rga_buffer_t dst_buf = wrapbuffer_virtualaddr(
            (void*)dst_nv12, out_w, out_h, RK_FORMAT_YCbCr_420_SP, out_w, out_h
        );

        IM_STATUS status = imcvtcolor(src_buf, dst_buf, RK_FORMAT_BGR_888, RK_FORMAT_YCbCr_420_SP);
        if (status == IM_STATUS_SUCCESS) {
            return true;
        }
        LOG_WARN("RGA", "imcvtcolor failed with status: ", (int)status, ", fallback to software.");
    }

    // Software fallback
    cv::Mat yuv_i420;
    cv::cvtColor(src_bgr, yuv_i420, cv::COLOR_BGR2YUV_I420);

    // Convert I420 (YYYY... U... V...) to NV12 (YYYY... UVUV...)
    int y_size = out_w * out_h;
    int uv_size = y_size / 4;
    const uint8_t* y_src = yuv_i420.data;
    const uint8_t* u_src = y_src + y_size;
    const uint8_t* v_src = u_src + uv_size;

    uint8_t* y_dst = dst_nv12;
    uint8_t* uv_dst = dst_nv12 + y_size;

    std::memcpy(y_dst, y_src, y_size);
    for (int i = 0; i < uv_size; ++i) {
        uv_dst[2 * i] = u_src[i];
        uv_dst[2 * i + 1] = v_src[i];
    }
    return true;
}

bool RgaProcessor::rgbToNv12(const uint8_t* src_rgb, int src_w, int src_h, uint8_t* dst_nv12, int dst_w, int dst_h) {
    if (src_rgb == nullptr || dst_nv12 == nullptr) {
        return false;
    }

    int out_w = (dst_w > 0) ? dst_w : src_w;
    int out_h = (dst_h > 0) ? dst_h : src_h;

    std::lock_guard<std::mutex> lock(mutex_);

    if (rga_available_) {
        rga_buffer_t src_buf = wrapbuffer_virtualaddr(
            (void*)src_rgb, src_w, src_h, RK_FORMAT_RGB_888, src_w, src_h
        );
        rga_buffer_t dst_buf = wrapbuffer_virtualaddr(
            (void*)dst_nv12, out_w, out_h, RK_FORMAT_YCbCr_420_SP, out_w, out_h
        );

        IM_STATUS status = imcvtcolor(src_buf, dst_buf, RK_FORMAT_RGB_888, RK_FORMAT_YCbCr_420_SP);
        if (status == IM_STATUS_SUCCESS) {
            return true;
        }
    }

    cv::Mat rgb_mat(src_h, src_w, CV_8UC3, (void*)src_rgb);
    cv::Mat bgr_mat;
    cv::cvtColor(rgb_mat, bgr_mat, cv::COLOR_RGB2BGR);
    return bgrToNv12(bgr_mat, dst_nv12, out_w, out_h);
}

} // namespace orbbec_service
