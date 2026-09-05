#include "replay_frame_source.hpp"
#include "logger.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <algorithm>
#include <cctype>
#include <map>
#include <thread>

namespace fs = std::filesystem;

namespace orbbec_service {

namespace {

std::string toLowerCopy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool nameHasDepthToken(const std::string& path) {
    const std::string name = toLowerCopy(fs::path(path).filename().string());
    return name.find("depth") != std::string::npos;
}

// scan.color.jpg / scan.depth.png / image_001.depth.png / color_001.jpg → 同一配对键
std::string replayPairKey(const std::string& path) {
    std::string stem = toLowerCopy(fs::path(path).stem().string());
    auto strip_suffix = [&](const char* sfx) {
        const size_t n = std::char_traits<char>::length(sfx);
        if (stem.size() > n && stem.compare(stem.size() - n, n, sfx) == 0) {
            stem.resize(stem.size() - n);
        }
    };
    auto strip_prefix = [&](const char* pfx) {
        const size_t n = std::char_traits<char>::length(pfx);
        if (stem.size() > n && stem.compare(0, n, pfx) == 0) {
            stem = stem.substr(n);
        }
    };
    for (int i = 0; i < 3; ++i) {
        strip_suffix(".color");
        strip_suffix(".depth");
        strip_suffix("_color");
        strip_suffix("_depth");
        strip_suffix("-color");
        strip_suffix("-depth");
        strip_prefix("color.");
        strip_prefix("depth.");
        strip_prefix("color_");
        strip_prefix("depth_");
        strip_prefix("color-");
        strip_prefix("depth-");
    }
    return stem;
}

}  // namespace

ReplayFrameSource::ReplayFrameSource() {}

ReplayFrameSource::~ReplayFrameSource() {}

bool ReplayFrameSource::init(const std::string& replay_path, int width, int height, int fps) {
    width_ = (width > 0) ? width : 1280;
    height_ = (height > 0) ? height : 800;
    fps_ = (fps > 0) ? fps : 30;
    path_ = replay_path;
    frame_index_ = 0;
    current_file_idx_ = 0;
    is_file_replay_ = false;
    items_.clear();

    if (replay_path.empty() || replay_path == "synthetic") {
        LOG_INFO("Replay", "Initialized Synthetic ReplayFrameSource (explicit). ",
                 width_, "x", height_, " @ ", fps_, " fps. "
                 "Frames are mock test patterns; do not use for follow / calibration.");
        last_frame_tp_ = std::chrono::steady_clock::now();
        return true;
    }

    if (!loadReplayFiles(replay_path)) {
        return false;
    }

    is_file_replay_ = true;
    size_t depth_n = 0;
    for (const auto& it : items_) {
        if (!it.depth.empty()) {
            ++depth_n;
        }
    }
    LOG_INFO("Replay", "Initialized ReplayFrameSource from path: ", replay_path,
             " (Found ", items_.size(), " color frames, ", depth_n, " paired depth, resolution: ",
             width_, "x", height_, " @ ", fps_, " fps)");
    last_frame_tp_ = std::chrono::steady_clock::now();
    return true;
}

bool ReplayFrameSource::loadReplayFiles(const std::string& path) {
    std::error_code ec;
    if (!fs::exists(path, ec) || ec) {
        LOG_ERROR("Replay", "Replay path does not exist: ", path);
        return false;
    }

    std::vector<std::string> color_files;
    std::vector<std::string> depth_files;

    if (fs::is_directory(path, ec)) {
        std::vector<std::string> all_files;
        for (const auto& entry : fs::directory_iterator(path, ec)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            std::string ext = toLowerCopy(entry.path().extension().string());
            if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp") {
                all_files.push_back(entry.path().string());
            }
        }
        std::sort(all_files.begin(), all_files.end());

        for (const auto& f : all_files) {
            if (nameHasDepthToken(f)) {
                depth_files.push_back(f);
            } else {
                color_files.push_back(f);
            }
        }
    } else if (fs::is_regular_file(path, ec)) {
        if (nameHasDepthToken(path)) {
            LOG_ERROR("Replay", "Replay path is a depth file, need a color image or a directory: ", path);
            return false;
        }
        color_files.push_back(path);
    } else {
        LOG_ERROR("Replay", "Replay path is neither a file nor a directory: ", path);
        return false;
    }

    if (color_files.empty()) {
        LOG_ERROR("Replay", "No color images found under: ", path);
        return false;
    }

    std::map<std::string, std::string> depth_by_key;
    for (const auto& d : depth_files) {
        depth_by_key[replayPairKey(d)] = d;
    }

    items_.reserve(color_files.size());
    for (const auto& c : color_files) {
        Item item;
        item.color = c;
        auto it = depth_by_key.find(replayPairKey(c));
        if (it != depth_by_key.end()) {
            item.depth = it->second;
        }
        items_.push_back(std::move(item));
    }
    return true;
}

bool ReplayFrameSource::getNextFrame(FrameData& out_frame) {
    auto now = std::chrono::steady_clock::now();
    double target_interval_ms = 1000.0 / std::max(1, fps_);
    double elapsed_ms = std::chrono::duration<double, std::milli>(now - last_frame_tp_).count();
    if (elapsed_ms < target_interval_ms) {
        int sleep_ms = static_cast<int>(target_interval_ms - elapsed_ms);
        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }
    last_frame_tp_ = std::chrono::steady_clock::now();

    auto sys_now = std::chrono::system_clock::now();
    uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(sys_now.time_since_epoch()).count();
    uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(sys_now.time_since_epoch()).count();

    out_frame = FrameData{};
    out_frame.frame_index = frame_index_++;
    out_frame.timestamp_ms = now_ms;
    out_frame.device_ts_us = now_us;
    out_frame.track_ts_ns = static_cast<int64_t>(now_us) * 1000;

    if (!is_file_replay_ || items_.empty()) {
        generateSyntheticFrame(out_frame);
        return true;
    }

    const Item& item = items_[current_file_idx_];
    current_file_idx_ = (current_file_idx_ + 1) % items_.size();

    cv::Mat raw_color = cv::imread(item.color, cv::IMREAD_COLOR);
    if (raw_color.empty()) {
        LOG_WARN("Replay", "Failed to read color frame: ", item.color);
        return false;
    }
    if (raw_color.cols != width_ || raw_color.rows != height_) {
        cv::resize(raw_color, out_frame.color, cv::Size(width_, height_));
    } else {
        out_frame.color = raw_color;
    }
    out_frame.has_color = true;

    if (item.depth.empty()) {
        return true;
    }

    cv::Mat raw_depth = cv::imread(item.depth, cv::IMREAD_UNCHANGED);
    if (raw_depth.empty()) {
        LOG_WARN("Replay", "Failed to read depth frame: ", item.depth);
        return true;
    }
    if (raw_depth.type() != CV_16UC1) {
        LOG_WARN("Replay", "Depth file is not 16-bit single channel, ignored: ", item.depth);
        return true;
    }
    if (raw_depth.cols != width_ || raw_depth.rows != height_) {
        cv::resize(raw_depth, out_frame.depth, cv::Size(width_, height_), 0, 0, cv::INTER_NEAREST);
    } else {
        out_frame.depth = raw_depth;
    }
    out_frame.has_depth = true;
    return true;
}

void ReplayFrameSource::generateSyntheticFrame(FrameData& out_frame) {
    out_frame.color = cv::Mat(height_, width_, CV_8UC3);
    out_frame.depth = cv::Mat(height_, width_, CV_16UC1);

    int phase = static_cast<int>((frame_index_ * 6) % width_);

    for (int r = 0; r < height_; ++r) {
        cv::Vec3b* ptr = out_frame.color.ptr<cv::Vec3b>(r);
        uint16_t* dptr = out_frame.depth.ptr<uint16_t>(r);
        for (int c = 0; c < width_; ++c) {
            uint8_t b = static_cast<uint8_t>((c * 255) / width_);
            uint8_t g = static_cast<uint8_t>((r * 255) / height_);
            uint8_t red = 60;
            if (std::abs(c - phase) < 18) {
                b = 255; g = 255; red = 255;
            }
            ptr[c] = cv::Vec3b(b, g, red);
            dptr[c] = static_cast<uint16_t>(800 + (c * 1200 / width_));
        }
    }

    std::string text1 = "SYNTHETIC FRAME SOURCE (NOT A CAMERA)";
    std::string text2 = "Frame: " + std::to_string(out_frame.frame_index) +
                        " | " + std::to_string(width_) + "x" + std::to_string(height_) +
                        " @" + std::to_string(fps_) + "fps";
    std::string text3 = "TS: " + std::to_string(out_frame.timestamp_ms) + " ms";

    cv::putText(out_frame.color, text1, cv::Point(40, 60),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(out_frame.color, text1, cv::Point(40, 60),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

    cv::putText(out_frame.color, text2, cv::Point(40, 110),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(out_frame.color, text2, cv::Point(40, 110),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    cv::putText(out_frame.color, text3, cv::Point(40, 150),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(out_frame.color, text3, cv::Point(40, 150),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    out_frame.has_color = true;
    out_frame.has_depth = true;
}

void ReplayFrameSource::reset() {
    frame_index_ = 0;
    current_file_idx_ = 0;
    last_frame_tp_ = std::chrono::steady_clock::now();
}

CameraIntrinsics ReplayFrameSource::getIntrinsics() const {
    CameraIntrinsics intr;
    intr.camera_model = is_file_replay_ ? "replay" : "synthetic";
    intr.width = width_;
    intr.height = height_;
    // 零矩阵：没有设备内参。调用方必须看 CameraStatus::intrinsics_loaded，不得当标定结果用。
    intr.intrinsic_matrix = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    intr.distortion_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};
    intr.distortion_model = "none";
    intr.depth_scale = 1.0;
    return intr;
}

} // namespace orbbec_service
