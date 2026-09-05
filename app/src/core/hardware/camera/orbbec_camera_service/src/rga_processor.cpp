#include "rga_processor.hpp"

#ifdef HAS_RGA
#include <rga/im2d.hpp>
#include <rga/rga.h>
#include <rga/RgaApi.h>
#endif

#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include <opencv2/imgproc.hpp>

namespace orbbec_service {

RgaProcessor::RgaProcessor() {}

RgaProcessor::~RgaProcessor() {
    release();
}

bool RgaProcessor::init(int width, int height) {
    std::lock_guard<std::mutex> lock(mutex_);
    src_width_ = width;
    src_height_ = height;

#ifdef HAS_RGA
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
#else
    rga_available_ = false;
    LOG_INFO("RGA", "Built without Rockchip RGA; using CPU color conversion. Base resolution: ",
             width, "x", height);
#endif
    return true;
}

void RgaProcessor::release() {
    std::lock_guard<std::mutex> lock(mutex_);
#ifdef HAS_RGA
    if (rga_available_) {
        c_RkRgaDeInit();
        rga_available_ = false;
        LOG_INFO("RGA", "RGA hardware context released.");
    }
#else
    rga_available_ = false;
#endif
}

static bool cpuBgrToNv12(const cv::Mat& src_bgr, uint8_t* dst_nv12, int out_w, int out_h) {
    cv::Mat resized;
    const cv::Mat* src = &src_bgr;
    if (src_bgr.cols != out_w || src_bgr.rows != out_h) {
        cv::resize(src_bgr, resized, cv::Size(out_w, out_h), 0, 0, cv::INTER_LINEAR);
        src = &resized;
    }

    cv::Mat yuv_i420;
    cv::cvtColor(*src, yuv_i420, cv::COLOR_BGR2YUV_I420);

    const int y_size = out_w * out_h;
    const int uv_size = y_size / 4;
    if (yuv_i420.total() < static_cast<size_t>(y_size + 2 * uv_size)) {
        return false;
    }

    const uint8_t* y_src = yuv_i420.data;
    const uint8_t* u_src = y_src + y_size;
    const uint8_t* v_src = u_src + uv_size;

    std::memcpy(dst_nv12, y_src, static_cast<size_t>(y_size));
    uint8_t* uv_dst = dst_nv12 + y_size;
    for (int i = 0; i < uv_size; ++i) {
        uv_dst[2 * i] = u_src[i];
        uv_dst[2 * i + 1] = v_src[i];
    }
    return true;
}

bool RgaProcessor::bgrToNv12(const cv::Mat& src_bgr, uint8_t* dst_nv12, int dst_width, int dst_height) {
    if (src_bgr.empty() || dst_nv12 == nullptr) {
        LOG_ERROR("RGA", "Invalid input or destination buffer for bgrToNv12");
        return false;
    }

    const int in_w = src_bgr.cols;
    const int in_h = src_bgr.rows;
    const int out_w = (dst_width > 0) ? dst_width : in_w;
    const int out_h = (dst_height > 0) ? dst_height : in_h;

    std::lock_guard<std::mutex> lock(mutex_);

#ifdef HAS_RGA
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
#endif

    return cpuBgrToNv12(src_bgr, dst_nv12, out_w, out_h);
}

}  // namespace orbbec_service
