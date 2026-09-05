#include "software_encoder.hpp"

#include <cstring>

#include <wels/codec_api.h>
#include <wels/codec_app_def.h>
#include <wels/codec_def.h>

#include "logger.hpp"

namespace orbbec_service {

static void nv12_to_i420(const uint8_t* nv12, uint8_t* i420, int w, int h) {
    const int y_size = w * h;
    const int uv_size = y_size / 4;
    std::memcpy(i420, nv12, static_cast<size_t>(y_size));
    const uint8_t* uv = nv12 + y_size;
    uint8_t* u = i420 + y_size;
    uint8_t* v = u + uv_size;
    for (int i = 0; i < uv_size; ++i) {
        u[i] = uv[2 * i];
        v[i] = uv[2 * i + 1];
    }
}

SoftwareH264Encoder::SoftwareH264Encoder() = default;

SoftwareH264Encoder::~SoftwareH264Encoder() {
    release();
}

bool SoftwareH264Encoder::init(int width, int height, int fps, int bitrate_kbps) {
    std::lock_guard<std::mutex> lock(mutex_);
    release();

    width_ = width;
    height_ = height;
    fps_ = (fps > 0) ? fps : 30;
    bitrate_kbps_ = (bitrate_kbps > 0) ? bitrate_kbps : 2500;
    if (width_ <= 0 || height_ <= 0 || (width_ % 2) != 0 || (height_ % 2) != 0) {
        LOG_ERROR("OpenH264", "invalid size ", width_, "x", height_, " (need even dimensions)");
        return false;
    }

    ISVCEncoder* enc = nullptr;
    if (WelsCreateSVCEncoder(&enc) != 0 || enc == nullptr) {
        LOG_ERROR("OpenH264", "WelsCreateSVCEncoder failed");
        return false;
    }

    SEncParamExt param;
    memset(&param, 0, sizeof(param));
    enc->GetDefaultParams(&param);
    param.iUsageType = CAMERA_VIDEO_REAL_TIME;
    param.fMaxFrameRate = static_cast<float>(fps_);
    param.iPicWidth = width_;
    param.iPicHeight = height_;
    param.iTargetBitrate = bitrate_kbps_ * 1000;
    param.iMaxBitrate = param.iTargetBitrate * 2;
    param.iRCMode = RC_BITRATE_MODE;
    param.uiIntraPeriod = static_cast<unsigned>(fps_);
    param.iNumRefFrame = 1;
    param.iSpatialLayerNum = 1;
    param.iEntropyCodingModeFlag = 0;
    param.sSpatialLayers[0].iVideoWidth = width_;
    param.sSpatialLayers[0].iVideoHeight = height_;
    param.sSpatialLayers[0].fFrameRate = static_cast<float>(fps_);
    param.sSpatialLayers[0].iSpatialBitrate = param.iTargetBitrate;

    if (enc->InitializeExt(&param) != 0) {
        LOG_ERROR("OpenH264", "InitializeExt failed");
        WelsDestroySVCEncoder(enc);
        return false;
    }

    int video_format = videoFormatI420;
    enc->SetOption(ENCODER_OPTION_DATAFORMAT, &video_format);

    encoder_ = enc;
    nv12_buf_.assign(static_cast<size_t>(width_ * height_ * 3 / 2), 0);
    i420_buf_.assign(static_cast<size_t>(width_ * height_ * 3 / 2), 0);
    initialized_ = true;
    frame_count_ = 0;
    LOG_INFO("OpenH264", "software encoder ready ", width_, "x", height_,
             " @ ", fps_, "fps, ", bitrate_kbps_, " kbps");
    return true;
}

void SoftwareH264Encoder::release() {
    if (encoder_ != nullptr) {
        auto* enc = static_cast<ISVCEncoder*>(encoder_);
        enc->Uninitialize();
        WelsDestroySVCEncoder(enc);
        encoder_ = nullptr;
    }
    nv12_buf_.clear();
    i420_buf_.clear();
    initialized_ = false;
}

void* SoftwareH264Encoder::getFrameBufferPtr() {
    return initialized_ && !nv12_buf_.empty() ? nv12_buf_.data() : nullptr;
}

bool SoftwareH264Encoder::encodeDirect(uint8_t* out_h264_buf, int max_out_len, int& out_len,
                                       bool& is_keyframe, uint64_t pts) {
    (void)pts;
    std::lock_guard<std::mutex> lock(mutex_);
    out_len = 0;
    is_keyframe = false;
    if (!initialized_ || encoder_ == nullptr || out_h264_buf == nullptr || max_out_len <= 0) {
        return false;
    }

    nv12_to_i420(nv12_buf_.data(), i420_buf_.data(), width_, height_);

    auto* enc = static_cast<ISVCEncoder*>(encoder_);
    SSourcePicture pic;
    memset(&pic, 0, sizeof(pic));
    pic.iColorFormat = videoFormatI420;
    pic.iPicWidth = width_;
    pic.iPicHeight = height_;
    pic.iStride[0] = width_;
    pic.iStride[1] = width_ / 2;
    pic.iStride[2] = width_ / 2;
    pic.pData[0] = i420_buf_.data();
    pic.pData[1] = i420_buf_.data() + width_ * height_;
    pic.pData[2] = pic.pData[1] + (width_ * height_ / 4);

    SFrameBSInfo info;
    memset(&info, 0, sizeof(info));
    if (enc->EncodeFrame(&pic, &info) != 0 || info.eFrameType == videoFrameTypeInvalid) {
        LOG_WARN("OpenH264", "EncodeFrame failed");
        return false;
    }
    if (info.eFrameType == videoFrameTypeSkip) {
        return false;
    }

    is_keyframe = (info.eFrameType == videoFrameTypeIDR || info.eFrameType == videoFrameTypeI);

    int written = 0;
    for (int layer = 0; layer < info.iLayerNum; ++layer) {
        const SLayerBSInfo& li = info.sLayerInfo[layer];
        int offset = 0;
        for (int n = 0; n < li.iNalCount; ++n) {
            const int nal_len = li.pNalLengthInByte[n];
            if (written + nal_len > max_out_len) {
                LOG_ERROR("OpenH264", "output buffer too small");
                return false;
            }
            std::memcpy(out_h264_buf + written, li.pBsBuf + offset, static_cast<size_t>(nal_len));
            written += nal_len;
            offset += nal_len;
        }
    }
    out_len = written;
    frame_count_++;
    return out_len > 0;
}

}  // namespace orbbec_service
