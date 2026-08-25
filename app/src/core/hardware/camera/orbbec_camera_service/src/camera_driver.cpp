#include "camera_driver.hpp"
#include <libobsensor/ObSensor.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>

namespace orbbec_service {

CameraDriver::CameraDriver() {}

CameraDriver::~CameraDriver() {
    stop();
}

bool CameraDriver::init(const AppConfig& config) {
    config_ = config;
    calib_mode_ = config.calib_default.enabled;
    status_.calibration_mode = calib_mode_;
    status_.depth_stream_enabled = !calib_mode_;
    status_.depth_align_enabled = (!calib_mode_ && config.enable_depth_align);

    LOG_INFO("Camera", "Initializing Orbbec Camera Driver (Target resolution: ", 
             config_.camera_width, "x", config_.camera_height, " @ ", config_.camera_fps, 
             "fps, Initial Mode: ", (calib_mode_ ? "Calibration (Depth OFF)" : "Normal (Depth ON)"), ")");
    return true;
}

bool CameraDriver::start() {
    if (running_) {
        LOG_WARN("Camera", "Camera is already running.");
        return true;
    }

    running_ = true;
    capture_thread_ = std::thread(&CameraDriver::captureLoop, this);
    LOG_INFO("Camera", "Camera capture thread started.");
    return true;
}

void CameraDriver::stop() {
    if (!running_) return;

    running_ = false;
    frame_cv_.notify_all();

    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    resetHardwareConnection();
    LOG_INFO("Camera", "Camera driver stopped.");
}

void CameraDriver::resetHardwareConnection() {
    std::lock_guard<std::mutex> lock(pipe_mutex_);
    try {
        if (pipe_) {
            pipe_->stop();
            pipe_.reset();
        }
    } catch (...) {
        pipe_.reset();
    }
    device_.reset();
    ctx_.reset();

    connected_ = false;
    consecutive_timeouts_ = 0;
    {
        std::lock_guard<std::mutex> f_lock(frame_mutex_);
        status_.online = false;
        status_.streaming = false;
    }
}

bool CameraDriver::tryConnectDevice() {
    try {
        // Reset old handles completely so libusb frees any zombie USB interfaces
        {
            std::lock_guard<std::mutex> lock(pipe_mutex_);
            if (pipe_) {
                try { pipe_->stop(); } catch (...) {}
                pipe_.reset();
            }
            device_.reset();
            ctx_.reset();
            ctx_ = std::make_unique<ob::Context>();
        }

        auto dev_list = ctx_->queryDeviceList();
        if (dev_list->deviceCount() == 0) {
            LOG_WARN("Camera", "No Orbbec device found on USB bus. Waiting for reconnection...");
            return false;
        }

        device_ = dev_list->getDevice(0);
        auto dev_info = device_->getDeviceInfo();

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            status_.camera_model = dev_info->name();
            status_.serial_number = dev_info->serialNumber();
            status_.firmware_version = dev_info->firmwareVersion();
        }

        LOG_INFO("Camera", "Found Orbbec Device: [Model: ", dev_info->name(), 
                 ", SN: ", dev_info->serialNumber(), ", FW: ", dev_info->firmwareVersion(), "]");

        {
            std::lock_guard<std::mutex> lock(pipe_mutex_);
            pipe_ = std::make_unique<ob::Pipeline>(device_);
            if (!configureAndStartPipeline()) {
                return false;
            }
        }

        // Fetch camera intrinsics
        try {
            auto cam_param = pipe_->getCameraParam();
            std::lock_guard<std::mutex> lock(frame_mutex_);
            intrinsics_.width = cam_param.rgbIntrinsic.width > 0 ? cam_param.rgbIntrinsic.width : config_.camera_width;
            intrinsics_.height = cam_param.rgbIntrinsic.height > 0 ? cam_param.rgbIntrinsic.height : config_.camera_height;
            intrinsics_.intrinsic_matrix = {
                {cam_param.rgbIntrinsic.fx, 0.0, cam_param.rgbIntrinsic.cx},
                {0.0, cam_param.rgbIntrinsic.fy, cam_param.rgbIntrinsic.cy},
                {0.0, 0.0, 1.0}
            };
            intrinsics_.distortion_coeffs = {
                cam_param.rgbDistortion.k1,
                cam_param.rgbDistortion.k2,
                cam_param.rgbDistortion.p1,
                cam_param.rgbDistortion.p2,
                cam_param.rgbDistortion.k3
            };
            LOG_INFO("Camera", "Camera Intrinsics loaded: fx=", cam_param.rgbIntrinsic.fx, 
                     ", fy=", cam_param.rgbIntrinsic.fy, ", cx=", cam_param.rgbIntrinsic.cx, 
                     ", cy=", cam_param.rgbIntrinsic.cy);
        } catch (const ob::Error& e) {
            LOG_WARN("Camera", "Could not query camera intrinsics from device (", e.getMessage(), "), using defaults.");
        }

        consecutive_timeouts_ = 0;
        connected_ = true;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            status_.online = true;
            status_.streaming = true;
        }
        return true;
    } catch (const ob::Error& e) {
        LOG_ERROR("Camera", "Orbbec SDK Error during device connection: ", e.getMessage());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("Camera", "Standard exception during device connection: ", e.what());
        return false;
    }
}

bool CameraDriver::configureAndStartPipeline() {
    if (!pipe_) return false;

    auto pipe_config = std::make_shared<ob::Config>();

    // 1. Configure Color Stream
    try {
        auto color_profiles = pipe_->getStreamProfileList(OB_SENSOR_COLOR);
        std::shared_ptr<ob::VideoStreamProfile> color_profile = nullptr;
        try {
            color_profile = color_profiles->getVideoStreamProfile(config_.camera_width, config_.camera_height, 
                                                                  OB_FORMAT_BGR, config_.camera_fps);
        } catch (...) {
            try {
                color_profile = color_profiles->getVideoStreamProfile(config_.camera_width, config_.camera_height, 
                                                                      OB_FORMAT_RGB, config_.camera_fps);
            } catch (...) {
                color_profile = color_profiles->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, 
                                                                      OB_FORMAT_ANY, config_.camera_fps);
            }
        }
        if (color_profile) {
            pipe_config->enableStream(color_profile);
            LOG_INFO("Camera", "Enabled Color Stream: ", color_profile->width(), "x", color_profile->height(), 
                     " @ ", color_profile->fps(), "fps (Format: ", (int)color_profile->format(), ")");
        }
    } catch (const ob::Error& e) {
        LOG_WARN("Camera", "Failed to configure color stream profile: ", e.getMessage());
    }

    // 2. Configure Depth Stream (ONLY in Normal Mode, automatically disabled in Calibration Mode)
    if (!calib_mode_) {
        try {
            auto depth_profiles = pipe_->getStreamProfileList(OB_SENSOR_DEPTH);
            std::shared_ptr<ob::VideoStreamProfile> depth_profile = nullptr;
            try {
                depth_profile = depth_profiles->getVideoStreamProfile(config_.camera_width, config_.camera_height, 
                                                                      OB_FORMAT_Y16, config_.camera_fps);
            } catch (...) {
                depth_profile = depth_profiles->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, 
                                                                      OB_FORMAT_ANY, config_.camera_fps);
            }
            if (depth_profile) {
                pipe_config->enableStream(depth_profile);
                LOG_INFO("Camera", "Enabled Depth Stream: ", depth_profile->width(), "x", depth_profile->height(), 
                         " @ ", depth_profile->fps(), "fps");
            }
        } catch (const ob::Error& e) {
            LOG_WARN("Camera", "Failed to configure depth stream profile: ", e.getMessage());
        }
    } else {
        LOG_INFO("Camera", "Calibration Mode Active: Depth stream reading and alignment are \033[33mDISABLED\033[0m (0% Depth CPU load).");
    }

    // 3. Configure Alignment and Start Pipeline
    bool started = false;
    if (!calib_mode_ && config_.enable_depth_align) {
        // First attempt: Hardware D2C Alignment (Camera on-chip ASIC, 0% Host CPU)
        try {
            pipe_config->setAlignMode(ALIGN_D2C_HW_MODE);
            pipe_->start(pipe_config);
            started = true;
            LOG_INFO("Camera", "Started pipeline with Hardware D2C Alignment (ASIC on-chip, 0% Host CPU).");
        } catch (const ob::Error& e) {
            LOG_INFO("Camera", "Hardware D2C align not supported (", e.getMessage(), "), trying Software D2C...");
            try {
                pipe_config->setAlignMode(ALIGN_D2C_SW_MODE);
                pipe_->start(pipe_config);
                started = true;
                LOG_INFO("Camera", "Started pipeline with Software D2C Alignment.");
            } catch (const ob::Error& e2) {
                LOG_WARN("Camera", "Software D2C start failed (", e2.getMessage(), "), trying direct stream...");
            }
        }
    }

    if (!started) {
        try {
            pipe_config->setAlignMode(ALIGN_DISABLE);
            pipe_->start(pipe_config);
            started = true;
            LOG_INFO("Camera", "Started pipeline with Direct Streams (D2C align disabled).");
        } catch (const ob::Error& e) {
            LOG_ERROR("Camera", "Pipeline start failed: ", e.getMessage());
            return false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        status_.calibration_mode = calib_mode_;
        status_.depth_stream_enabled = !calib_mode_;
        status_.depth_align_enabled = (!calib_mode_ && config_.enable_depth_align);
    }
    return true;
}

bool CameraDriver::setCalibrationMode(bool enabled) {
    if (calib_mode_ == enabled && connected_) {
        LOG_INFO("Camera", "Camera is already in mode: ", (enabled ? "Calibration Mode" : "Normal Mode"));
        return true;
    }

    LOG_INFO("Camera", ">>> Dynamically Switching Mode: ", 
             (enabled ? "\033[32m[CALIBRATION MODE: Depth OFF, D2C Alignment OFF]\033[0m"
                      : "\033[36m[NORMAL OPERATION MODE: Depth ON, D2C Alignment ON]\033[0m"));

    calib_mode_ = enabled;

    std::lock_guard<std::mutex> lock(pipe_mutex_);
    if (!pipe_ || !connected_) {
        return true;
    }

    try {
        pipe_->stop();
    } catch (const ob::Error& e) {
        LOG_WARN("Camera", "Exception during pipeline stop: ", e.getMessage());
    }

    return configureAndStartPipeline();
}

void CameraDriver::captureLoop() {
    LOG_INFO("Camera", "Entering high-speed capture loop.");
    last_stat_time_ms_ = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    while (running_) {
        if (!connected_) {
            if (!tryConnectDevice()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                continue;
            }
        }

        std::shared_ptr<ob::FrameSet> frameset = nullptr;
        {
            std::lock_guard<std::mutex> lock(pipe_mutex_);
            if (!connected_ || !pipe_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            try {
                frameset = pipe_->waitForFrameset(1000);
            } catch (const ob::Error& e) {
                LOG_WARN("Camera", "waitForFrameset exception (", e.getMessage(), ")");
                frameset = nullptr;
            }
        }

        if (!frameset) {
            consecutive_timeouts_++;
            LOG_WARN("Camera", "waitForFrameset timeout (", consecutive_timeouts_, "/3). Checking connection...");
            if (consecutive_timeouts_ >= 3) {
                LOG_ERROR("Camera", "3 consecutive frame timeouts. Camera hardware disconnected or USB bus reset. Initiating automatic re-connection...");
                resetHardwareConnection();
            }
            continue;
        }

        consecutive_timeouts_ = 0;

        try {
            auto color_frame = frameset->colorFrame();
            auto depth_frame = frameset->depthFrame();

            FrameData cur_frame;
            cur_frame.frame_index = frame_count_++;
            cur_frame.timestamp_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            // Process Color Frame
            if (color_frame && color_frame->dataSize() > 0) {
                int w = color_frame->width();
                int h = color_frame->height();
                OBFormat fmt = color_frame->format();

                if (fmt == OB_FORMAT_BGR) {
                    cur_frame.color = cv::Mat(h, w, CV_8UC3, color_frame->data()).clone();
                    cur_frame.has_color = true;
                } else if (fmt == OB_FORMAT_RGB) {
                    cv::Mat rgb(h, w, CV_8UC3, color_frame->data());
                    cv::cvtColor(rgb, cur_frame.color, cv::COLOR_RGB2BGR);
                    cur_frame.has_color = true;
                } else if (fmt == OB_FORMAT_MJPG) {
                    std::vector<uint8_t> mjpg_buf((uint8_t*)color_frame->data(), 
                                                  (uint8_t*)color_frame->data() + color_frame->dataSize());
                    cur_frame.color = cv::imdecode(mjpg_buf, cv::IMREAD_COLOR);
                    cur_frame.has_color = !cur_frame.color.empty();
                }
            }

            // Process Depth Frame (Z16) - active only when depth stream is enabled
            if (depth_frame && depth_frame->dataSize() > 0) {
                int w = depth_frame->width();
                int h = depth_frame->height();
                cur_frame.depth = cv::Mat(h, w, CV_16UC1, depth_frame->data()).clone();
                cur_frame.has_depth = true;
            }

            // Update Latest Frame
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = cur_frame;
            }
            frame_cv_.notify_all();

            updateFpsStats();

        } catch (const ob::Error& e) {
            LOG_ERROR("Camera", "Exception in capture loop: ", e.getMessage());
            resetHardwareConnection();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } catch (const std::exception& e) {
            LOG_ERROR("Camera", "Standard exception in capture loop: ", e.what());
            resetHardwareConnection();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    LOG_INFO("Camera", "Capture loop terminated.");
}

void CameraDriver::updateFpsStats() {
    auto now_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    if (now_ms - last_stat_time_ms_ >= 2000) {
        double elapsed_sec = (now_ms - last_stat_time_ms_) / 1000.0;
        uint64_t frames = frame_count_ - last_stat_frames_;
        double fps = (elapsed_sec > 0.0) ? (frames / elapsed_sec) : 0.0;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            status_.color_fps = fps;
            status_.depth_fps = calib_mode_ ? 0.0 : fps;
            status_.total_frames = frame_count_;
        }

        last_stat_time_ms_ = now_ms;
        last_stat_frames_ = frame_count_;
    }
}

bool CameraDriver::getLatestFrame(FrameData& out_frame) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!latest_frame_.has_color && !latest_frame_.has_depth) {
        return false;
    }
    out_frame = latest_frame_;
    return true;
}

bool CameraDriver::waitForNextFrame(FrameData& out_frame, int timeout_ms) {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    bool ok = frame_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() {
        return !running_ || (latest_frame_.frame_index > last_consumed_frame_id_);
    });

    if (!ok || !running_ || (!latest_frame_.has_color && !latest_frame_.has_depth)) {
        return false;
    }

    last_consumed_frame_id_ = latest_frame_.frame_index;
    out_frame = latest_frame_;
    return true;
}

CameraStatus CameraDriver::getStatus() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return status_;
}

CameraIntrinsics CameraDriver::getIntrinsics() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return intrinsics_;
}

} // namespace orbbec_service
