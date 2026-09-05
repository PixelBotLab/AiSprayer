#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

namespace orbbec_service {

// Same call surface as MppEncoder so main.cpp can keep one pipeline.
// Used only when HAS_MPP is off (not compiled into the RK3588 binary).
class SoftwareH264Encoder {
public:
    SoftwareH264Encoder();
    ~SoftwareH264Encoder();

    bool init(int width, int height, int fps = 30, int bitrate_kbps = 2500);
    void release();

    void* getFrameBufferPtr();

    bool encodeDirect(uint8_t* out_h264_buf, int max_out_len, int& out_len,
                      bool& is_keyframe, uint64_t pts = 0);

    bool isInitialized() const { return initialized_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    int getFps() const { return fps_; }

private:
    bool initialized_ = false;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 30;
    int bitrate_kbps_ = 2500;
    uint64_t frame_count_ = 0;

    void* encoder_ = nullptr;  // ISVCEncoder*
    std::vector<uint8_t> nv12_buf_;
    std::vector<uint8_t> i420_buf_;
    std::mutex mutex_;
};

}  // namespace orbbec_service
