#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include "types.hpp"

namespace orbbec_service {

class ReplayFrameSource {
public:
    ReplayFrameSource();
    ~ReplayFrameSource();

    // path 为空或 "synthetic"：合成彩条（显式）。其它路径必须能扫到彩色图，否则返回 false。
    bool init(const std::string& replay_path, int width, int height, int fps);
    bool getNextFrame(FrameData& out_frame);
    void reset();

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    int getFps() const { return fps_; }
    bool isFileReplay() const { return is_file_replay_; }
    CameraIntrinsics getIntrinsics() const;

private:
    struct Item {
        std::string color;
        std::string depth;  // 空 = 这一帧没有配对深度，不得合成假深度
    };

    void generateSyntheticFrame(FrameData& out_frame);
    bool loadReplayFiles(const std::string& path);

private:
    std::string path_;
    int width_ = 1280;
    int height_ = 800;
    int fps_ = 30;
    uint64_t frame_index_ = 0;
    bool is_file_replay_ = false;
    std::vector<Item> items_;
    size_t current_file_idx_ = 0;

    std::chrono::steady_clock::time_point last_frame_tp_;
};

} // namespace orbbec_service
