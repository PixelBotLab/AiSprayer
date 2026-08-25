#include "corner_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <pthread.h>
#include <sched.h>

namespace orbbec_service {

static void bindThreadToBigCores() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    // RK3588 Big Cores: CPU 4, 5, 6, 7 (Cortex-A76 @ 2.35~2.40 GHz)
    CPU_SET(4, &cpuset);
    CPU_SET(5, &cpuset);
    CPU_SET(6, &cpuset);
    CPU_SET(7, &cpuset);

    pthread_t thread = pthread_self();
    pthread_setname_np(thread, "corner_worker");
    int ret = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (ret == 0) {
        LOG_INFO("CornerDetector", "Corner Worker thread successfully PINNED to RK3588 Cortex-A76 Big Cores (CPUs 4, 5, 6, 7).");
    } else {
        LOG_WARN("CornerDetector", "Failed to pin worker thread to big cores, errno: ", ret);
    }
}

CornerDetector::CornerDetector() {
    init();
}

CornerDetector::~CornerDetector() {
    stop();
}

bool CornerDetector::init() {
    if (running_) return true;

    // 1. Enable ARM NEON / SIMD Hardware Instruction-Level Optimization in OpenCV
    cv::setUseOptimized(true);
    LOG_INFO("CornerDetector", "OpenCV Hardware SIMD optimizations: ", 
             (cv::useOptimized() ? "ENABLED (ARM NEON/SIMD)" : "Disabled"));

    // Limit OpenCV CPU concurrency to 2 threads on Cortex-A76 big cores
    cv::setNumThreads(2);

    // 2. Enable Mali GPU / OpenCL Acceleration if available
    cv::ocl::setUseOpenCL(true);
    if (cv::ocl::haveOpenCL() && cv::ocl::useOpenCL()) {
        cv::ocl::Device dev = cv::ocl::Device::getDefault();
        gpu_accelerated_ = true;
        LOG_INFO("CornerDetector", "Mali GPU OpenCL acceleration detected: [Device: ", 
                 dev.name(), ", Vendor: ", dev.vendorName(), ", Compute Units: ", dev.maxComputeUnits(), 
                 ", OpenCL: ", dev.driverVersion(), "]");
    } else {
        gpu_accelerated_ = false;
        LOG_WARN("CornerDetector", "OpenCL is not available, using ARM NEON multi-threaded CPU.");
    }

    // 3. Start Asynchronous Worker Thread
    running_ = true;
    worker_thread_ = std::thread(&CornerDetector::workerLoop, this);
    LOG_INFO("CornerDetector", "Asynchronous Corner Detection Worker thread initialized.");
    return true;
}

void CornerDetector::stop() {
    if (!running_) return;

    running_ = false;
    frame_cv_.notify_all();

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    LOG_INFO("CornerDetector", "Corner Detection Worker thread stopped.");
}

void CornerDetector::setConfig(const CalibrationConfig& config) {
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_ = config;
    }

    if (!config.enabled) {
        std::lock_guard<std::mutex> r_lock(result_mutex_);
        cached_corners_.found = false;
        cached_corners_.corners.clear();
    }

    LOG_INFO("CornerDetector", "Updated CalibrationConfig: enabled=", config.enabled,
             ", type=", config.board_type, ", rows=", config.rows, ", cols=", config.cols,
             ", square_size_mm=", config.square_size_mm, ", draw=", config.draw_corners);
}

CalibrationConfig CornerDetector::getConfig() {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void CornerDetector::feedFrame(const cv::Mat& bgr_image) {
    if (bgr_image.empty()) return;

    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        if (!config_.enabled) return;
    }

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        pending_frame_ = bgr_image.clone();
        has_new_frame_ = true;
    }
    frame_cv_.notify_one();
}

void CornerDetector::workerLoop() {
    // Pin worker thread to Cortex-A76 Big Cores (CPU 4-7)
    bindThreadToBigCores();

    while (running_) {
        cv::Mat work_mat;
        CalibrationConfig cur_cfg;

        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_cv_.wait(lock, [this]() {
                return !running_ || has_new_frame_;
            });

            if (!running_) break;

            work_mat = pending_frame_;
            has_new_frame_ = false;
        }

        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            cur_cfg = config_;
        }

        if (!cur_cfg.enabled || work_mat.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        DetectedCorners out_corners;
        processInternal(work_mat, cur_cfg, out_corners);

        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            cached_corners_ = out_corners;
            last_detection_time_ms_ = out_corners.timestamp_ms;

            worker_stats_.count++;
            worker_stats_.sum_ms += out_corners.detection_time_ms;
            if (out_corners.detection_time_ms < worker_stats_.min_ms) worker_stats_.min_ms = out_corners.detection_time_ms;
            if (out_corners.detection_time_ms > worker_stats_.max_ms) worker_stats_.max_ms = out_corners.detection_time_ms;
            worker_stats_.last_ms = out_corners.detection_time_ms;
        }

        // Throttle detection frequency to ~7-10 Hz (sleep 100ms) to prevent pinning CPU cores at 100%
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool CornerDetector::detectSync(const cv::Mat& bgr_image, DetectedCorners& out_corners) {
    CalibrationConfig cur_cfg = getConfig();
    return processInternal(bgr_image, cur_cfg, out_corners);
}

bool CornerDetector::processInternal(const cv::Mat& bgr_image, const CalibrationConfig& cfg, DetectedCorners& out_corners) {
    if (bgr_image.empty()) return false;

    cv::Size pattern_size = cfg.getPatternSize();
    auto t_start = std::chrono::high_resolution_clock::now();

    // 100% Full Original Resolution (No downscaling) to preserve subpixel accuracy
    cv::Mat gray;
    if (bgr_image.channels() == 3) {
        cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = bgr_image;
    }

    std::vector<cv::Point2f> corners;
    bool found = false;

    // 1. First attempt: Modern Sector-Based Chessboard Detector (Extremely fast, robust, no quad backtracking)
    try {
        found = cv::findChessboardCornersSB(gray, pattern_size, corners, 
                                            cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    } catch (...) {
        found = false;
    }

    // 2. Fallback attempt: Standard Chessboard Detector if SB fails
    if (!found) {
        int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK;
        found = cv::findChessboardCorners(gray, pattern_size, corners, flags);
        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01));
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    auto now_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    out_corners.found = found;
    out_corners.timestamp_ms = now_ms;
    out_corners.pattern_cols = pattern_size.width;
    out_corners.pattern_rows = pattern_size.height;
    out_corners.corners = corners;
    out_corners.detection_time_ms = duration_ms;

    if (found) {
        LOG_INFO("CornerDetector", "Found ", corners.size(), " chessboard corners on full resolution (", 
                 gray.cols, "x", gray.rows, ") in ", duration_ms, " ms");
    }
    return found;
}

void CornerDetector::drawOverlay(cv::Mat& bgr_image) {
    if (bgr_image.empty()) return;

    CalibrationConfig cfg;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        if (!config_.enabled || !config_.draw_corners) return;
        cfg = config_;
    }

    DetectedCorners corners;
    uint64_t last_time = 0;
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (!cached_corners_.found) return;
        corners = cached_corners_;
        last_time = last_detection_time_ms_;
    }

    auto now_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // Expire overlay if no detection within 800ms
    if (now_ms - last_time > 800) {
        return;
    }

    cv::Size pattern_size(corners.pattern_cols, corners.pattern_rows);
    cv::drawChessboardCorners(bgr_image, pattern_size, corners.corners, corners.found);
}

bool CornerDetector::getLatestCorners(DetectedCorners& out_corners) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    out_corners = cached_corners_;
    return cached_corners_.found;
}

WorkerLatencyStats CornerDetector::getWorkerStats(bool reset) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    WorkerLatencyStats st = worker_stats_;
    if (reset) {
        worker_stats_.count = 0;
        worker_stats_.sum_ms = 0.0;
        worker_stats_.min_ms = 1e9;
        worker_stats_.max_ms = 0.0;
    }
    return st;
}

} // namespace orbbec_service
