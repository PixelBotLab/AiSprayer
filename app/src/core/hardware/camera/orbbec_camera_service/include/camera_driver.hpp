#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <opencv2/core.hpp>
#include "types.hpp"
#include "logger.hpp"

// Forward declaration of Orbbec SDK classes
namespace ob {
    class Context;
    class Pipeline;
    class Device;
    class Config;
}

namespace orbbec_service {

class CameraDriver {
public:
    CameraDriver();
    ~CameraDriver();

    bool init(const AppConfig& config);
    bool start();
    void stop();

    bool isRunning() const { return running_; }
    bool isConnected() const { return connected_; }

    // Fetch the latest aligned frame data (non-blocking)
    bool getLatestFrame(FrameData& out_frame);

    // Event-driven frame waiting: blocks until next camera hardware frame arrives
    bool waitForNextFrame(FrameData& out_frame, int timeout_ms = 100);

    // Dynamically switch calibration mode:
    // If enabled == true: disables depth stream and depth alignment (0% depth CPU load)
    // If enabled == false: enables depth stream and depth alignment
    bool setCalibrationMode(bool enabled);
    bool isCalibrationMode() const { return calib_mode_; }

    // Get current status & intrinsics
    CameraStatus getStatus();
    CameraIntrinsics getIntrinsics();

private:
    void captureLoop();
    bool tryConnectDevice();
    bool configureAndStartPipeline();
    void resetHardwareConnection();
    void updateFpsStats();

private:
    AppConfig config_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> calib_mode_{false};
    int consecutive_timeouts_ = 0;

    std::unique_ptr<ob::Context> ctx_;
    std::unique_ptr<ob::Pipeline> pipe_;
    std::shared_ptr<ob::Device> device_;

    std::thread capture_thread_;
    std::mutex pipe_mutex_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    FrameData latest_frame_;
    uint64_t last_consumed_frame_id_ = 0;

    CameraStatus status_;
    CameraIntrinsics intrinsics_;

    uint64_t frame_count_ = 0;
    uint64_t last_stat_time_ms_ = 0;
    uint64_t last_stat_frames_ = 0;
};

} // namespace orbbec_service
