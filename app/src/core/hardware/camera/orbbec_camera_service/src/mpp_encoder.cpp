#include "mpp_encoder.hpp"
#include <rockchip/rk_venc_cfg.h>
#include <cstring>

namespace orbbec_service {

#define MPP_ALIGN(x, a) (((x) + (a)-1) & ~((a)-1))

MppEncoder::MppEncoder() {}

MppEncoder::~MppEncoder() {
    release();
}

bool MppEncoder::init(int width, int height, int fps, int bitrate_kbps) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_) {
        release();
    }

    width_ = width;
    height_ = height;
    fps_ = (fps > 0) ? fps : 30;
    bitrate_kbps_ = (bitrate_kbps > 0) ? bitrate_kbps : 2500;

    // Rockchip MPP alignment requirements: 16 bytes for width, 16 bytes for height
    hor_stride_ = MPP_ALIGN(width_, 16);
    ver_stride_ = MPP_ALIGN(height_, 16);

    MPP_RET ret = mpp_create(&ctx_, &mpi_);
    if (ret != MPP_OK || ctx_ == nullptr || mpi_ == nullptr) {
        LOG_ERROR("MPP", "mpp_create failed, error code: ", (int)ret);
        return false;
    }

    ret = mpp_init(ctx_, MPP_CTX_ENC, MPP_VIDEO_CodingAVC);
    if (ret != MPP_OK) {
        LOG_ERROR("MPP", "mpp_init MPP_CTX_ENC failed, error code: ", (int)ret);
        release();
        return false;
    }

    ret = mpp_enc_cfg_init(&cfg_);
    if (ret != MPP_OK || cfg_ == nullptr) {
        LOG_ERROR("MPP", "mpp_enc_cfg_init failed, error code: ", (int)ret);
        release();
        return false;
    }

    if (!setupEncoderConfig()) {
        LOG_ERROR("MPP", "setupEncoderConfig failed");
        release();
        return false;
    }

    // Allocate buffer group and frame buffer
    size_t frame_size = hor_stride_ * ver_stride_ * 3 / 2;
    ret = mpp_buffer_group_get_internal(&buf_group_, MPP_BUFFER_TYPE_DRM);
    if (ret != MPP_OK) {
        ret = mpp_buffer_group_get_internal(&buf_group_, MPP_BUFFER_TYPE_ION);
    }
    if (ret != MPP_OK) {
        ret = mpp_buffer_group_get_internal(&buf_group_, MPP_BUFFER_TYPE_NORMAL);
    }

    if (ret != MPP_OK || buf_group_ == nullptr) {
        LOG_ERROR("MPP", "mpp_buffer_group_get_internal failed: ", (int)ret);
        release();
        return false;
    }

    ret = mpp_buffer_get(buf_group_, &frm_buf_, frame_size);
    if (ret != MPP_OK || frm_buf_ == nullptr) {
        LOG_ERROR("MPP", "mpp_buffer_get frm_buf failed: ", (int)ret);
        release();
        return false;
    }

    // Extract SPS/PPS header
    MppPacket header_pkt = nullptr;
    ret = mpi_->control(ctx_, MPP_ENC_GET_EXTRA_INFO, &header_pkt);
    if (ret == MPP_OK && header_pkt != nullptr) {
        void* ptr = mpp_packet_get_pos(header_pkt);
        size_t len = mpp_packet_get_length(header_pkt);
        if (ptr && len > 0) {
            header_buf_.assign((uint8_t*)ptr, (uint8_t*)ptr + len);
            LOG_INFO("MPP", "Extracted H.264 SPS/PPS extra info header (", len, " bytes)");
        }
        mpp_packet_deinit(&header_pkt);
    }

    initialized_ = true;
    frame_count_ = 0;

    LOG_INFO("MPP", "Rockchip MPP Hardware H.264 Encoder initialized: ", 
             width_, "x", height_, " @ ", fps_, "fps (stride: ", hor_stride_, "x", ver_stride_, 
             ", bitrate: ", bitrate_kbps_, " kbps)");
    return true;
}

bool MppEncoder::setupEncoderConfig() {
    MPP_RET ret = mpi_->control(ctx_, MPP_ENC_GET_CFG, cfg_);
    if (ret != MPP_OK) {
        LOG_ERROR("MPP", "MPP_ENC_GET_CFG failed: ", (int)ret);
        return false;
    }

    // Preparation config
    mpp_enc_cfg_set_s32(cfg_, "prep:width", width_);
    mpp_enc_cfg_set_s32(cfg_, "prep:height", height_);
    mpp_enc_cfg_set_s32(cfg_, "prep:hor_stride", hor_stride_);
    mpp_enc_cfg_set_s32(cfg_, "prep:ver_stride", ver_stride_);
    mpp_enc_cfg_set_s32(cfg_, "prep:format", MPP_FMT_YUV420SP); // NV12

    // Rate control config
    mpp_enc_cfg_set_s32(cfg_, "rc:mode", MPP_ENC_RC_MODE_CBR);
    mpp_enc_cfg_set_s32(cfg_, "rc:bps_target", bitrate_kbps_ * 1024);
    mpp_enc_cfg_set_s32(cfg_, "rc:bps_max", bitrate_kbps_ * 1024 * 5 / 4);
    mpp_enc_cfg_set_s32(cfg_, "rc:bps_min", bitrate_kbps_ * 1024 * 3 / 4);
    mpp_enc_cfg_set_s32(cfg_, "rc:fps_in_flex", 0);
    mpp_enc_cfg_set_s32(cfg_, "rc:fps_in_num", fps_);
    mpp_enc_cfg_set_s32(cfg_, "rc:fps_in_denorm", 1);
    mpp_enc_cfg_set_s32(cfg_, "rc:fps_out_flex", 0);
    mpp_enc_cfg_set_s32(cfg_, "rc:fps_out_num", fps_);
    mpp_enc_cfg_set_s32(cfg_, "rc:fps_out_denorm", 1);
    mpp_enc_cfg_set_s32(cfg_, "rc:gop", fps_); // 1 GOP = 1 second

    // Codec type & profile
    mpp_enc_cfg_set_s32(cfg_, "codec:type", MPP_VIDEO_CodingAVC);
    mpp_enc_cfg_set_s32(cfg_, "h264:profile", 100); // High Profile
    mpp_enc_cfg_set_s32(cfg_, "h264:level", 41);
    mpp_enc_cfg_set_s32(cfg_, "h264:cabac_en", 1);
    mpp_enc_cfg_set_s32(cfg_, "h264:cabac_idc", 0);
    mpp_enc_cfg_set_s32(cfg_, "h264:trans8x8", 1);

    ret = mpi_->control(ctx_, MPP_ENC_SET_CFG, cfg_);
    if (ret != MPP_OK) {
        LOG_ERROR("MPP", "MPP_ENC_SET_CFG failed: ", (int)ret);
        return false;
    }
    return true;
}

void MppEncoder::release() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;

    if (frm_buf_) {
        mpp_buffer_put(frm_buf_);
        frm_buf_ = nullptr;
    }
    if (buf_group_) {
        mpp_buffer_group_put(buf_group_);
        buf_group_ = nullptr;
    }
    if (cfg_) {
        mpp_enc_cfg_deinit(cfg_);
        cfg_ = nullptr;
    }
    if (ctx_) {
        mpp_destroy(ctx_);
        ctx_ = nullptr;
        mpi_ = nullptr;
    }

    initialized_ = false;
    LOG_INFO("MPP", "MppEncoder resources released.");
}

void* MppEncoder::getFrameBufferPtr() {
    if (!initialized_ || frm_buf_ == nullptr) return nullptr;
    return mpp_buffer_get_ptr(frm_buf_);
}

bool MppEncoder::encodeDirect(uint8_t* out_h264_buf, int max_out_len, int& out_len, 
                              bool& is_keyframe, uint64_t pts) {
    if (!initialized_ || out_h264_buf == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // Initialize MppFrame directly referencing existing hardware buffer (0 CPU memcpy!)
    MppFrame frame = nullptr;
    MPP_RET ret = mpp_frame_init(&frame);
    if (ret != MPP_OK || frame == nullptr) {
        return false;
    }

    mpp_frame_set_width(frame, width_);
    mpp_frame_set_height(frame, height_);
    mpp_frame_set_hor_stride(frame, hor_stride_);
    mpp_frame_set_ver_stride(frame, ver_stride_);
    mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
    mpp_frame_set_buffer(frame, frm_buf_);
    mpp_frame_set_pts(frame, pts > 0 ? pts : (frame_count_ * 1000000 / fps_));

    // Put Frame
    ret = mpi_->encode_put_frame(ctx_, frame);
    mpp_frame_deinit(&frame);
    if (ret != MPP_OK) {
        LOG_ERROR("MPP", "encode_put_frame failed: ", (int)ret);
        return false;
    }

    // Get Packet
    MppPacket packet = nullptr;
    ret = mpi_->encode_get_packet(ctx_, &packet);
    if (ret != MPP_OK || packet == nullptr) {
        LOG_ERROR("MPP", "encode_get_packet failed: ", (int)ret);
        return false;
    }

    void* pkt_ptr = mpp_packet_get_pos(packet);
    size_t pkt_len = mpp_packet_get_length(packet);

    // Check if keyframe by scanning H.264 NAL header
    is_keyframe = false;
    if (pkt_ptr && pkt_len > 4) {
        const uint8_t* p = (const uint8_t*)pkt_ptr;
        for (size_t i = 0; i + 4 < pkt_len; ++i) {
            if (p[i] == 0x00 && p[i+1] == 0x00 && p[i+2] == 0x01) {
                int nal_type = p[i+3] & 0x1F;
                if (nal_type == 5 || nal_type == 7) {
                    is_keyframe = true;
                    break;
                }
            } else if (p[i] == 0x00 && p[i+1] == 0x00 && p[i+2] == 0x00 && p[i+3] == 0x01) {
                int nal_type = p[i+4] & 0x1F;
                if (nal_type == 5 || nal_type == 7) {
                    is_keyframe = true;
                    break;
                }
            }
        }
    }

    out_len = 0;

    // Prepend SPS/PPS header to IDR frame if needed
    if (is_keyframe && !header_buf_.empty()) {
        bool has_sps = false;
        if (pkt_len >= 4 && ((uint8_t*)pkt_ptr)[0] == 0x00 && ((uint8_t*)pkt_ptr)[1] == 0x00 && 
            ((uint8_t*)pkt_ptr)[2] == 0x00 && ((uint8_t*)pkt_ptr)[3] == 0x01 && 
            (((uint8_t*)pkt_ptr)[4] & 0x1F) == 7) {
            has_sps = true;
        }

        if (!has_sps && (int)(header_buf_.size() + pkt_len) <= max_out_len) {
            std::memcpy(out_h264_buf, header_buf_.data(), header_buf_.size());
            out_len += header_buf_.size();
        }
    }

    if (pkt_ptr && pkt_len > 0 && (out_len + (int)pkt_len <= max_out_len)) {
        std::memcpy(out_h264_buf + out_len, pkt_ptr, pkt_len);
        out_len += pkt_len;
    }

    mpp_packet_deinit(&packet);
    frame_count_++;
    return out_len > 0;
}

} // namespace orbbec_service
