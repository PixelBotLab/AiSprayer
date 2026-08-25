#pragma once

#include <string>
#include <memory>
#include <mutex>
#include "types.hpp"
#include "logger.hpp"

// Forward declaration of ZLM types
typedef struct mk_media_t *mk_media;
typedef struct mk_track_t *mk_track;

namespace orbbec_service {

class ZlmStreamer {
public:
    ZlmStreamer();
    ~ZlmStreamer();

    bool init(const AppConfig& config);
    void release();

    // Push H.264 NALU frame into ZLMediaKit virtual media source
    bool pushH264(const uint8_t* h264_data, int data_len, uint64_t dts_ms, uint64_t pts_ms);

    StreamInfo getStreamInfo(const std::string& host_ip = "127.0.0.1") const;
    bool isRunning() const { return running_; }

private:
    bool running_ = false;
    AppConfig config_;
    mk_media media_ = nullptr;
    mk_track track_ = nullptr;
    std::mutex mutex_;
};

} // namespace orbbec_service
