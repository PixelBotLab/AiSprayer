#include "zlm_streamer.hpp"
#include <mk_mediakit.h>
#include <cstring>
#include <sstream>

namespace orbbec_service {

static void API_CALL onZlmLog(int level, const char *file, int line, const char *function, const char *message) {
    // Suppress mundane session disconnect and unhandled websocket warnings from cluttering terminal
    if (message && std::strstr(message, "http server do not support websocket")) {
        return;
    }
    if (level >= 3) {
        LOG_WARN("ZLM", message ? message : "");
    }
}

ZlmStreamer::ZlmStreamer() {}

ZlmStreamer::~ZlmStreamer() {
    release();
}

bool ZlmStreamer::init(const AppConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;

    LOG_INFO("ZLM", "Initializing ZLMediaKit Streaming Engine...");

    mk_events events;
    std::memset(&events, 0, sizeof(events));
    events.on_mk_log = onZlmLog;
    mk_events_listen(&events);

    mk_config cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.thread_num = 2;
    cfg.log_level = 3; // Warn / Error
    cfg.log_mask = LOG_CALLBACK; // Intercept logs via callback
    mk_env_init(&cfg);

    // Start RTSP Server
    uint16_t rtsp_p = mk_rtsp_server_start(config_.rtsp_port, 0);
    LOG_INFO("ZLM", "Started RTSP server on port: ", rtsp_p);

    // Start WebRTC / HTTP Server
    uint16_t http_p = mk_http_server_start(config_.zlm_http_port, 0);
    LOG_INFO("ZLM", "Started ZLM HTTP/WebRTC server on port: ", http_p);

    // Start RTC Server for WebRTC
    uint16_t rtc_p = mk_rtc_server_start(config_.zlm_http_port);
    LOG_INFO("ZLM", "Started WebRTC RTC server on port: ", rtc_p);

    // Start RTMP Server
    uint16_t rtmp_p = mk_rtmp_server_start(config_.rtmp_port, 0);
    LOG_INFO("ZLM", "Started RTMP server on port: ", rtmp_p);

    // Create Virtual Media Source
    media_ = mk_media_create("__defaultVhost__", config_.stream_app.c_str(), config_.stream_id.c_str(), 0, 0, 0);
    if (media_ == nullptr) {
        LOG_ERROR("ZLM", "mk_media_create failed for stream: ", config_.stream_app, "/", config_.stream_id);
        return false;
    }

    codec_args v_args;
    std::memset(&v_args, 0, sizeof(v_args));
    v_args.video.width = config_.stream_width;
    v_args.video.height = config_.stream_height;
    v_args.video.fps = config_.stream_fps;

    track_ = mk_track_create(MKCodecH264, &v_args);
    if (track_ == nullptr) {
        LOG_ERROR("ZLM", "mk_track_create H264 failed");
        mk_media_release(media_);
        media_ = nullptr;
        return false;
    }

    mk_media_init_track(media_, track_);
    mk_media_init_complete(media_);

    running_ = true;
    LOG_INFO("ZLM", "ZLMediaKit Media Stream published: [vhost: __defaultVhost__, app: ", 
             config_.stream_app, ", stream: ", config_.stream_id, "]");
    return true;
}

void ZlmStreamer::release() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) return;

    if (media_) {
        mk_media_release(media_);
        media_ = nullptr;
    }
    if (track_) {
        mk_track_unref(track_);
        track_ = nullptr;
    }

    mk_stop_all_server();
    running_ = false;
    LOG_INFO("ZLM", "ZLMediaKit Streaming Engine stopped.");
}

bool ZlmStreamer::pushH264(const uint8_t* h264_data, int data_len, uint64_t dts_ms, uint64_t pts_ms) {
    if (!running_ || media_ == nullptr || h264_data == nullptr || data_len <= 0) {
        return false;
    }

    int ret = mk_media_input_h264(media_, h264_data, data_len, dts_ms, pts_ms);
    return ret == 1;
}

StreamInfo ZlmStreamer::getStreamInfo(const std::string& host_ip) const {
    StreamInfo info;
    info.stream_id = config_.stream_id;
    info.width = config_.stream_width;
    info.height = config_.stream_height;
    info.fps = config_.stream_fps;

    std::stringstream ss_rtsp;
    ss_rtsp << "rtsp://" << host_ip << ":" << config_.rtsp_port << "/" << config_.stream_app << "/" << config_.stream_id;
    info.rtsp_url = ss_rtsp.str();

    std::stringstream ss_webrtc;
    ss_webrtc << "http://" << host_ip << ":" << config_.zlm_http_port << "/index/api/webrtc?app=" 
              << config_.stream_app << "&stream=" << config_.stream_id << "&type=play";
    info.webrtc_url = ss_webrtc.str();

    std::stringstream ss_flv;
    ss_flv << "http://" << host_ip << ":" << config_.zlm_http_port << "/" << config_.stream_app 
           << "/" << config_.stream_id << ".live.flv";
    info.http_flv_url = ss_flv.str();

    return info;
}

} // namespace orbbec_service
