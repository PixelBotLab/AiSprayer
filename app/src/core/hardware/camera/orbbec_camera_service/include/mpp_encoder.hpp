#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <cstdint>
#include <rockchip/rk_mpi.h>
#include <rockchip/rk_mpi_cmd.h>
#include <rockchip/rk_venc_cmd.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_packet.h>
#include <rockchip/mpp_buffer.h>
#include "logger.hpp"

namespace orbbec_service {

class MppEncoder {
public:
    MppEncoder();
    ~MppEncoder();

    bool init(int width, int height, int fps = 30, int bitrate_kbps = 2500);
    void release();

    // Get pointer to direct MPP DMA/DRM frame buffer (for Zero-Copy RGA rendering)
    void* getFrameBufferPtr();

    // Direct hardware encode from MPP frame buffer (Zero memcpy from CPU!)
    bool encodeDirect(uint8_t* out_h264_buf, int max_out_len, int& out_len, 
                      bool& is_keyframe, uint64_t pts = 0);

    bool isInitialized() const { return initialized_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    int getFps() const { return fps_; }

private:
    bool setupEncoderConfig();

private:
    bool initialized_ = false;
    int width_ = 0;
    int height_ = 0;
    int hor_stride_ = 0;
    int ver_stride_ = 0;
    int fps_ = 30;
    int bitrate_kbps_ = 2500;
    uint64_t frame_count_ = 0;

    MppCtx ctx_ = nullptr;
    MppApi* mpi_ = nullptr;
    MppEncCfg cfg_ = nullptr;
    MppBufferGroup buf_group_ = nullptr;
    MppBuffer frm_buf_ = nullptr;

    std::vector<uint8_t> header_buf_;
    std::mutex mutex_;
};

} // namespace orbbec_service
