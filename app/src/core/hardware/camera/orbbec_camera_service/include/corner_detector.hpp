#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <chrono>
#include "types.hpp"
#include "logger.hpp"

namespace orbbec_service {

struct WorkerLatencyStats {
    uint64_t count = 0;
    double sum_ms = 0.0;
    double min_ms = 1e9;
    double max_ms = 0.0;
    double last_ms = 0.0;

    double getAvg() const { return count > 0 ? (sum_ms / count) : 0.0; }
    double getMin() const { return count > 0 ? min_ms : 0.0; }
    double getMax() const { return count > 0 ? max_ms : 0.0; }
};

class CornerDetector {
public:
    CornerDetector();
    ~CornerDetector();

    bool init();
    void stop();

    void setConfig(const CalibrationConfig& config);
    CalibrationConfig getConfig();

    // Asynchronous feed: non-blocking, updates background worker target frame
    void feedFrame(const cv::Mat& bgr_image);

    // Draw cached detected corners onto image for video stream overlay (< 0.2ms)
    void drawOverlay(cv::Mat& bgr_image);

    // Get latest detected corners
    bool getLatestCorners(DetectedCorners& out_corners);

    // Get worker execution latency stats (accurate per-detection metric)
    WorkerLatencyStats getWorkerStats(bool reset = true);

    // Check if OpenCL / Mali GPU is active
    bool isGpuAccelerated() const { return gpu_accelerated_; }

private:
    void workerLoop();
    bool processInternal(const cv::Mat& bgr_image, const CalibrationConfig& cfg, DetectedCorners& out_corners);

private:
    CalibrationConfig config_;
    std::mutex config_mutex_;

    bool gpu_accelerated_ = false;

    // Async worker thread
    std::atomic<bool> running_{false};
    std::thread worker_thread_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    cv::Mat pending_frame_;
    bool has_new_frame_ = false;

    // Detection results cache
    std::mutex result_mutex_;
    DetectedCorners cached_corners_;
    uint64_t last_detection_time_ms_ = 0;
    WorkerLatencyStats worker_stats_;
};

} // namespace orbbec_service
