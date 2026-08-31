#include "camera_driver.hpp"
#include <libobsensor/ObSensor.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cmath>
#include <unistd.h>

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

void CameraDriver::stopPipelineAndSensors() {
    if (gyro_sensor_) {
        try {
            gyro_sensor_->stop();
        } catch (const ob::Error& e) {
            LOG_WARN("Camera", "Exception during gyro_sensor_->stop(): ", e.getMessage());
        } catch (...) {}
        gyro_sensor_.reset();
    }
    {
        std::lock_guard<std::mutex> g_lock(gyro_mtx_);
        gyro_queue_.clear();
    }
    // 时间基随停流作废：设备重启会换时间戳原点，主机侧的排队路径也重建了。留着旧偏移动起来
    // 像"标定还在"，实际上把一个新未知量当成常量。
    gyro_time_base_.reset();
    has_imu_ = false;

    if (pipe_) {
        try {
            pipe_->stop();
        } catch (const ob::Error& e) {
            LOG_WARN("Camera", "Exception during pipe_->stop(): ", e.getMessage());
        } catch (...) {}
    }

    {
        std::lock_guard<std::mutex> f_lock(frame_mutex_);
        latest_frame_ = FrameData{};
    }
}

void CameraDriver::resetHardwareConnection() {
    std::lock_guard<std::mutex> lock(pipe_mutex_);
    stopPipelineAndSensors();
    if (pipe_) {
        pipe_.reset();
    }
    device_.reset();
    ctx_.reset();
    // 设备句柄一放掉就交还独占锁：下一次 tryConnectDevice 会重新排队取锁，而这段空档正是
    // follow_pose / follow_node 能合法接手相机的时机。锁由内核持有，所以进程被杀也不会留残锁。
    if (dev_lock_.held()) {
        dev_lock_.release();
        lock_notice_logged_ = false;   // 重连时若还是拿不到，要再喊一次而不是一直闷着
    }

    connected_ = false;
    consecutive_timeouts_ = 0;
    soft_restart_attempts_ = 0;        // 硬重连后账本重开
    {
        std::lock_guard<std::mutex> f_lock(status_mutex_);
        status_.online = false;
        status_.streaming = false;
        // 这些字段描述的是"那次连接的设备状态"，设备没了就必须一起失效 —— 否则轮询方会读到
        // 上一档的 640x480 / intrinsics_loaded=true，而实际上一帧都拿不到。
        status_.capture_width = 0;
        status_.capture_height = 0;
        status_.capture_fps = 0;
        status_.intrinsics_loaded = false;
        status_.gyro_extrinsics_loaded = false;
        status_.depth_align_mode = "disabled";
        status_.depth_align_enabled = false;
    }
}

bool CameraDriver::trySoftPipelineRestart() {
    // 前提：调用方（captureLoop）此刻没持 pipe_mutex_。这里自己拿，与硬重连/档位切换同级别互斥。
    std::lock_guard<std::mutex> lock(pipe_mutex_);
    if (!running_ || !device_) {
        return false;
    }
    try {
        stopPipelineAndSensors();
        pipe_.reset();
        pipe_ = std::make_unique<ob::Pipeline>(device_);   // 设备不动，只重建取流通道
        if (!configureAndStartPipeline()) {
            return false;
        }
    } catch (const ob::Error& e) {
        LOG_WARN("Camera", "软重启 pipeline 失败（", e.getMessage(), "），升级到硬重连");
        return false;
    } catch (const std::exception& e) {
        LOG_WARN("Camera", "软重启 pipeline 异常（", e.what(), "），升级到硬重连");
        return false;
    }
    return true;
}

bool CameraDriver::tryConnectDevice() {
    try {
        // 独占仲裁：在碰 SDK 之前拿锁。顺序很重要 —— ob::Context 一构造就开始枚举/占用 USB
        // 设备，那时再拿锁已经晚了（而且拿"设备打不开"当"有人在用"的判据会把配置错误误报）。
        // 拿不到锁 → 直接 return false，交给 captureLoop 现成的 1500ms 重连节奏去等。
        if (!dev_lock_.held()) {
            if (config_.device_lock_path.empty()) {
                if (!lock_notice_logged_) {
                    lock_notice_logged_ = true;
                    LOG_WARN("Camera", "未配置 follow.camera.lock_path：跳过相机独占仲裁。",
                             "此时另起 follow_pose/follow_node 会与本服务在 libusb 层抢设备。");
                }
            } else {
                std::string lock_err;
                follow::DeviceLock::Busy busy;
                if (!dev_lock_.acquire(config_.device_lock_path, &lock_err, &busy)) {
                    if (!lock_notice_logged_) {
                        lock_notice_logged_ = true;
                        LOG_ERROR("Camera", "打不开相机：", lock_err,
                                  (busy.held_by_other ? "（等待对方释放，每 1500ms 重试一次）" : ""));
                    }
                    return false;
                }
                LOG_INFO("Camera", "相机独占锁已获取: ", config_.device_lock_path,
                         " (持有者 pid=", (int)getpid(), ")");
            }
        }

        // Reset old handles completely so libusb frees any zombie USB interfaces
        {
            std::lock_guard<std::mutex> lock(pipe_mutex_);
            stopPipelineAndSensors();
            if (pipe_) {
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
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_.camera_model = dev_info->name();
            status_.serial_number = dev_info->serialNumber();
            status_.firmware_version = dev_info->firmwareVersion();
        }

        LOG_INFO("Camera", "Found Orbbec Device: [Model: ", dev_info->name(), 
                 ", SN: ", dev_info->serialNumber(), ", FW: ", dev_info->firmwareVersion(), "]");

        {
            std::lock_guard<std::mutex> lock(pipe_mutex_);
            pipe_ = std::make_unique<ob::Pipeline>(device_);
            // 内参由 configureAndStartPipeline() 末尾的 refreshIntrinsics() 取：那里才知道"刚启起来
            // 的是哪一档"。以前这段写在连接路径里，而模式切换是原地重启 pipeline 不重连设备，
            // 于是换档后 fx 还是旧分辨率的 —— 会产出"自洽但全错"的点云。
            if (!configureAndStartPipeline()) {
                return false;
            }
        }

        consecutive_timeouts_ = 0;
        soft_restart_attempts_ = 0;    // 新连接成功，账本重开
        connected_ = true;
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
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

    // 本次要开的那一档。follow 使能时切到 follow.camera.*（实测 640x480@15 才能走硬件 D2C），
    // 否则用 hardware.camera 里配的档位。**彩色和深度必须同名同率**：硬件 D2C 要求两者分辨率
    // 一致，而 follow 前端拿内参反投影时尺寸不一致的帧只能整帧丢。
    const int want_w = follow_mode_ ? follow_width_ : config_.camera_width;
    const int want_h = follow_mode_ ? follow_height_ : config_.camera_height;
    const int want_fps = follow_mode_ ? follow_fps_ : config_.camera_fps;
    LOG_INFO("Camera", "取流档位: ", want_w, "x", want_h, " @ ", want_fps,
             "fps (", (follow_mode_ ? "follow 模式" : "hardware.camera 配置"),
             (calib_mode_ ? ", 标定模式: 深度流关闭" : ""), ")");

    auto pipe_config = std::make_shared<ob::Config>();

    // 实际交付的档位（不是"请求的档位"）—— 设备完全可以给你一个最接近的。状态里必须报这个，
    // 否则配置改了 800x600 而设备给 640x480，没人看得出内参和网格已经对不上。
    int delivered_w = 0, delivered_h = 0, delivered_fps = 0;

    // 1. Configure Color Stream
    try {
        auto color_profiles = pipe_->getStreamProfileList(OB_SENSOR_COLOR);
        std::shared_ptr<ob::VideoStreamProfile> color_profile = nullptr;
        try {
            color_profile = color_profiles->getVideoStreamProfile(want_w, want_h, 
                                                                  OB_FORMAT_BGR, want_fps);
        } catch (...) {
            try {
                color_profile = color_profiles->getVideoStreamProfile(want_w, want_h, 
                                                                      OB_FORMAT_RGB, want_fps);
            } catch (...) {
                color_profile = color_profiles->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, 
                                                                      OB_FORMAT_ANY, want_fps);
            }
        }
        if (color_profile) {
            pipe_config->enableStream(color_profile);
            delivered_w = (int)color_profile->width();
            delivered_h = (int)color_profile->height();
            delivered_fps = (int)color_profile->fps();
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
                depth_profile = depth_profiles->getVideoStreamProfile(want_w, want_h, 
                                                                      OB_FORMAT_Y16, want_fps);
            } catch (...) {
                depth_profile = depth_profiles->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, 
                                                                      OB_FORMAT_ANY, want_fps);
            }
            if (depth_profile) {
                pipe_config->enableStream(depth_profile);
                LOG_INFO("Camera", "Enabled Depth Stream: ", depth_profile->width(), "x", depth_profile->height(), 
                         " @ ", depth_profile->fps(), "fps");
                if ((int)depth_profile->width() != delivered_w || (int)depth_profile->height() != delivered_h) {
                    // 不致命（软件对齐照样能跑），但必须说：硬件 D2C 在这种档位下一定不可用。
                    LOG_WARN("Camera", "彩色档 ", delivered_w, "x", delivered_h, " 与深度档 ",
                             depth_profile->width(), "x", depth_profile->height(),
                             " 不一致：硬件 D2C 不可用，深度将被重采样对齐到彩色。");
                }
            }
        } catch (const ob::Error& e) {
            LOG_WARN("Camera", "Failed to configure depth stream profile: ", e.getMessage());
        }
    } else {
        LOG_INFO("Camera", "Calibration Mode Active: Depth stream reading and alignment are \033[33mDISABLED\033[0m (0% Depth CPU load).");
    }

    // 3. Configure Alignment and Start Pipeline
    bool started = false;
    std::string align_mode = "disabled";   // "hw" / "sw" / "disabled"：交付的深度到底有没有对齐到彩色
    if (!calib_mode_ && config_.enable_depth_align) {
        // First attempt: Hardware D2C Alignment (Camera on-chip ASIC, 0% Host CPU)
        try {
            pipe_config->setAlignMode(ALIGN_D2C_HW_MODE);
            pipe_->start(pipe_config);
            started = true;
            align_mode = "hw";
            LOG_INFO("Camera", "Started pipeline with Hardware D2C Alignment (ASIC on-chip, 0% Host CPU).");
        } catch (const ob::Error& e) {
            LOG_INFO("Camera", "Hardware D2C align not supported (", e.getMessage(), "), trying Software D2C...");
            try {
                pipe_config->setAlignMode(ALIGN_D2C_SW_MODE);
                pipe_->start(pipe_config);
                started = true;
                align_mode = "sw";
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
            align_mode = "disabled";
            LOG_INFO("Camera", "Started pipeline with Direct Streams (D2C align disabled).");
        } catch (const ob::Error& e) {
            LOG_ERROR("Camera", "Pipeline start failed: ", e.getMessage());
            return false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.calibration_mode = calib_mode_;
        status_.depth_stream_enabled = !calib_mode_;
        // 报**实际跑起来的**对齐方式，不是配置里想要的。ALIGN_DISABLE 时深度帧不在彩色像素
        // 坐标系里，follow 前端拿彩色内参反投影它会得到一个方向正确但全错的点云 —— 这种
        // "看不出异常的错"只能靠状态字段暴露出来（worker 侧也据此拒绝跑）。
        status_.depth_align_enabled = (align_mode != "disabled");
        status_.depth_align_mode = align_mode;
        status_.capture_width = delivered_w;
        status_.capture_height = delivered_h;
        status_.capture_fps = delivered_fps;
        status_.follow_profile = follow_mode_.load();
    }

    // 每次原地重启都重新取内参：档位可能已经变了（follow/标定切换），而 getCameraParam()
    // 只有在本 pipeline 起来之后才反映当前流配置。
    // 取不到内参**不**把视频流一起拖垮：编码器只要像素不要内参，而 follow 必须要内参 ——
    // 所以这里只记一笔 intrinsics_loaded=false，由 worker 侧拒绝跑（拿默认 fx 反投影出来的
    // 是"自洽但全错"的点云，宁可不跑）。
    if (!refreshIntrinsics()) {
        LOG_WARN("Camera", "Pipeline 已起来但内参未刷新：视频流继续，follow 将拒绝运行。");
    }

    // 4. 读取出厂标定外参 T_cam_gyro 并开启 200Hz 板载 IMU 陀螺仪流。
    //    两个门都要过：follow 档（非 follow 没有积分窗口）+ follow.camera.enable_imu。
    //    以前这里不看 enable_imu，yaml 关掉以后独立工具认、相机服务不认 —— 运维会以为关了。
    if (follow_mode_ && config_.enable_imu) {
        bool gyro_extrinsics_loaded = false;
        try {
            auto calib = pipe_->getCalibrationParam(pipe_config);
            auto e_gyro_color = calib.extrinsics[OB_SENSOR_GYRO][OB_SENSOR_COLOR];
            R_cam_gyro_ << e_gyro_color.rot[0], e_gyro_color.rot[1], e_gyro_color.rot[2],
                           e_gyro_color.rot[3], e_gyro_color.rot[4], e_gyro_color.rot[5],
                           e_gyro_color.rot[6], e_gyro_color.rot[7], e_gyro_color.rot[8];
            t_cam_gyro_ << e_gyro_color.trans[0], e_gyro_color.trans[1], e_gyro_color.trans[2];

            // 某些设备不抛异常、直接给全零旋转。那不是 Identity，isApprox(I) 过不去，
            // 若不拦会被当成"已标定"—— 后续 ω 全变成 0，静止检测永远判静。
            const bool valid_rotation = follow::is_valid_rotation(R_cam_gyro_);
            const bool looks_unspecified =
                R_cam_gyro_.isApprox(Eigen::Matrix3d::Identity(), 1e-6) && t_cam_gyro_.norm() < 1e-6;
            if (!valid_rotation) {
                LOG_WARN("Camera", "陀螺外参 T_cam_gyro 不是合法旋转（det=",
                         R_cam_gyro_.determinant(), "）：回退 Identity。零偏补偿残差会随相机姿态变化，",
                         "静止门可能进不去 —— 看 /follow/status 的 gyro.resid_dps");
                R_cam_gyro_ = Eigen::Matrix3d::Identity();
                t_cam_gyro_ = Eigen::Vector3d::Zero();
            } else if (looks_unspecified) {
                LOG_WARN("Camera", "陀螺外参 T_cam_gyro 读取成功但为 Identity（可能设备未标定）：",
                         "陀螺数据将在设备坐标系，零偏补偿会因相机姿态产生残差");
            } else {
                gyro_extrinsics_loaded = true;
                LOG_INFO("Camera", "陀螺外参 T_cam_gyro 已加载：R=[",
                         R_cam_gyro_(0,0), ",", R_cam_gyro_(0,1), ",", R_cam_gyro_(0,2), "; ",
                         R_cam_gyro_(1,0), ",", R_cam_gyro_(1,1), ",", R_cam_gyro_(1,2), "; ",
                         R_cam_gyro_(2,0), ",", R_cam_gyro_(2,1), ",", R_cam_gyro_(2,2), "] t=[",
                         t_cam_gyro_(0), ",", t_cam_gyro_(1), ",", t_cam_gyro_(2), "] mm");
            }
        } catch (const ob::Error& e) {
            R_cam_gyro_ = Eigen::Matrix3d::Identity();
            t_cam_gyro_ = Eigen::Vector3d::Zero();
            LOG_WARN("Camera", "陀螺外参 T_cam_gyro 读取失败（", e.getMessage(), "）：",
                     "回退到 Identity，零偏补偿残差会随相机姿态变化（可达 0.5~0.7°/s），",
                     "静止门可能进不去 —— 看 /follow/status 的 gyro.resid_dps");
        }

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_.gyro_extrinsics_loaded = gyro_extrinsics_loaded;
        }

        try {
            if (device_) {
                auto gyro_sensor = device_->getSensor(OB_SENSOR_GYRO);
                if (gyro_sensor) {
                    auto gyro_profiles = gyro_sensor->getStreamProfileList();
                    if (gyro_profiles && gyro_profiles->count() > 0) {
                        auto gprof = gyro_profiles->getProfile(0);
                        gyro_sensor_ = gyro_sensor;
                        gyro_sensor_->start(gprof, [this](std::shared_ptr<ob::Frame> frame) {
                            if (!frame) return;
                            auto gf = frame->as<ob::GyroFrame>();
                            if (!gf) return;
                            auto val = gf->getValue();
                            // getTimeStampUs() 是**设备自开机 µs**。入队前必须经 GyroTimeBase
                            // 换到与 FrameData::track_ts_ns 同一域；不换算时 follow 的积分窗口
                            // 一帧样本也框不到，现象只是"陀螺像是一直在动"（静默失效）。
                            const int64_t ts_ns = gyro_time_base_.toHostNs(gf->getTimeStampUs());
                            if (ts_ns == 0) {
                                return;  // 时间基还没定标（约 0.5 s）：丢掉，别把两个域混进队列
                            }
                            Eigen::Vector3d omega_raw(val.x, val.y, val.z);
                            Eigen::Vector3d omega_cam = R_cam_gyro_ * omega_raw;

                            std::lock_guard<std::mutex> g(gyro_mtx_);
                            gyro_queue_.push_back(follow::GyroSample{ts_ns, omega_cam});
                            if (gyro_queue_.size() > 500) {
                                gyro_queue_.pop_front();
                            }
                        });
                        has_imu_ = true;
                        LOG_INFO("Camera", "Started IMU Gyro stream (~200Hz) with hardware extrinsics; "
                                           "时间基定标中（前 ", GyroTimeBase::kProbePairs,
                                 " 帧配对），期间陀螺样本先丢弃");
                    }
                }
            }
        } catch (const ob::Error& e) {
            LOG_INFO("Camera", "IMU Gyro not available: ", e.getMessage());
            has_imu_ = false;
        }
    } else {
        has_imu_ = false;
        R_cam_gyro_ = Eigen::Matrix3d::Identity();
        t_cam_gyro_ = Eigen::Vector3d::Zero();
        if (follow_mode_ && !config_.enable_imu) {
            LOG_INFO("Camera", "follow.camera.enable_imu=false：不启动陀螺流"
                               "（帧间初值 / 离群门 / 静止冻结 / 示教静止门全部跳过）");
        }
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.gyro_extrinsics_loaded = false;
    }

    return true;
}

bool CameraDriver::refreshIntrinsics() {
    if (!pipe_) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.intrinsics_loaded = false;
        return false;
    }

    int cb_w = 0, cb_h = 0;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        cb_w = status_.capture_width;
        cb_h = status_.capture_height;
    }

    try {
        auto cam_param = pipe_->getCameraParam();
        std::lock_guard<std::mutex> lock(status_mutex_);
        intrinsics_.width = cam_param.rgbIntrinsic.width > 0 ? cam_param.rgbIntrinsic.width : cb_w;
        intrinsics_.height = cam_param.rgbIntrinsic.height > 0 ? cam_param.rgbIntrinsic.height : cb_h;
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
        status_.intrinsics_loaded = true;
        if (intrinsics_.width != cb_w || intrinsics_.height != cb_h) {
            // 不致命但必须喊出来：内参网格和交付帧尺寸不一致时，反投影出来的点云是自洽的错。
            LOG_WARN("Camera", "内参尺寸 ", intrinsics_.width, "x", intrinsics_.height,
                     " 与交付帧 ", cb_w, "x", cb_h, " 不一致，深度点云不可信。");
        }
        LOG_INFO("Camera", "Camera Intrinsics loaded (", intrinsics_.width, "x", intrinsics_.height,
                 "): fx=", cam_param.rgbIntrinsic.fx,
                 ", fy=", cam_param.rgbIntrinsic.fy, ", cx=", cam_param.rgbIntrinsic.cx,
                 ", cy=", cam_param.rgbIntrinsic.cy);
    } catch (const ob::Error& e) {
        LOG_WARN("Camera", "Could not query camera intrinsics from device (", e.getMessage(), "), using defaults.");
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.intrinsics_loaded = false;
        return false;
    }
    return true;
}

bool CameraDriver::setCalibrationMode(bool enabled) {
    if (calib_mode_ == enabled && connected_) {
        LOG_INFO("Camera", "Camera is already in mode: ", (enabled ? "Calibration Mode" : "Normal Mode"));
        return true;
    }

    // 标定模式关掉了深度流，follow 没有深度就跑不了 —— 所以进标定时显式退出 follow 档，
    // 而不是留一个"开着但每帧 no_depth"的 follow。worker 看到 follow_mode_ 变 false 会自行停表。
    if (enabled && follow_mode_) {
        LOG_INFO("Camera", "进入标定模式：follow 取流档位自动关闭（深度流已关，跟随无法运行）。");
        follow_mode_ = false;   // follow_width_/... 不用动：只在 follow_mode_ 为真时才读
    }

    LOG_INFO("Camera", ">>> Dynamically Switching Mode: ", 
             (enabled ? "\033[32m[CALIBRATION MODE: Depth OFF, D2C Alignment OFF]\033[0m"
                      : "\033[36m[NORMAL OPERATION MODE: Depth ON, D2C Alignment ON]\033[0m"));

    calib_mode_ = enabled;

    std::lock_guard<std::mutex> lock(pipe_mutex_);
    if (!pipe_ || !connected_) {
        return true;
    }

    stopPipelineAndSensors();
    return configureAndStartPipeline();
}

bool CameraDriver::setFollowProfile(bool enabled, int width, int height, int fps,
                                    std::string* err) {
    if (enabled && calib_mode_) {
        const std::string msg = "标定模式下深度流是关的，follow 起不来：先退出标定模式再使能 follow。";
        LOG_ERROR("Camera", msg);
        if (err) *err = msg;
        return false;
    }

    const int w = width > 0 ? width : 640;
    const int h = height > 0 ? height : 480;
    const int f_fps = fps > 0 ? fps : 15;

    int64_t t0 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> lock(pipe_mutex_);
    int64_t t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (t1 - t0 > 100) {
        LOG_INFO("Camera", "setFollowProfile: waited ", (t1 - t0), "ms for pipe_mutex_");
    }

    if (follow_mode_ == enabled && follow_width_ == w && follow_height_ == h && follow_fps_ == f_fps) {
        LOG_INFO("Camera", "follow 档位无需切换（enabled=", (enabled ? "true" : "false"),
                 ", ", w, "x", h, " @ ", f_fps, "fps）");
        return true;
    }

    // 记住切换前的状态：新档位起不来时要退回这里，而不是把设备留在半套配置上。
    const bool prev_follow = follow_mode_.load();
    const int prev_w = follow_width_, prev_h = follow_height_, prev_fps = follow_fps_;

    follow_width_ = w;
    follow_height_ = h;
    follow_fps_ = f_fps;
    follow_mode_ = enabled;

    if (!pipe_ || !connected_) {
        // 设备还没起来：标志已经改好，tryConnectDevice 会按新档位配置 pipeline。
        LOG_INFO("Camera", "follow 档位已记下（", (enabled ? "开" : "关"), " ", w, "x", h, " @ ", f_fps,
                 "fps），但 pipeline 尚未运行 —— 连接时自动生效。");
        return true;
    }

    LOG_INFO("Camera", ">>> follow ", (enabled ? "使能" : "关闭"), "：取流切到 ",
             (enabled ? std::to_string(w) + "x" + std::to_string(h) + " @ " + std::to_string(f_fps) + "fps"
                      : std::to_string(config_.camera_width) + "x" + std::to_string(config_.camera_height) +
                            " @ " + std::to_string(config_.camera_fps) + "fps（hardware.camera）"));

    int64_t t_stop0 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    stopPipelineAndSensors();
    int64_t t_stop1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    LOG_INFO("Camera", "stopPipelineAndSensors done (took ", (t_stop1 - t_stop0), "ms)");

    int64_t t_start0 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    bool cfg_ok = configureAndStartPipeline();
    int64_t t_start1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    LOG_INFO("Camera", "configureAndStartPipeline done: ", (cfg_ok ? "success" : "failed"), " (took ", (t_start1 - t_start0), "ms)");

    if (cfg_ok) {
        return true;
    }

    LOG_ERROR("Camera", "新档位启动失败，回滚到切换前的取流配置。");
    follow_mode_ = prev_follow;
    follow_width_ = prev_w;
    follow_height_ = prev_h;
    follow_fps_ = prev_fps;
    stopPipelineAndSensors();
    const bool rolled_back = configureAndStartPipeline();
    // 回滚成功 ≠ 切换成功：档位现在停在**切换前**那一档，调用方必须知道自己没拿到 follow 档。
    if (err) {
        *err = "follow 档位 " + std::to_string(w) + "x" + std::to_string(h) + "@" +
               std::to_string(f_fps) + " 启动失败，取流" +
               (rolled_back ? "已回滚到切换前档位" : "回滚也失败，等自动重连");
    }
    return false;
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
            const uint64_t silent_ms = last_frameset_ms_ > 0 ?
                ((uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count() - last_frameset_ms_) : 0;
            LOG_WARN("Camera", "waitForFrameset timeout (", consecutive_timeouts_, "/3)，帧流已停滞 ",
                     silent_ms, "ms. Checking connection...");
            if (consecutive_timeouts_ >= 3) {
                consecutive_timeouts_ = 0;
                // 分级恢复：先软重启（不动设备/不重新枚举），失败两次才升级硬重连。
                // 实测硬重连后帧率经常掉到 3~4 fps，还会触发内核级重新枚举，把一次短暂停滞
                // 放大成十几秒的故障循环 —— 所以设备没真死之前不动它。
                if (soft_restart_attempts_ < 2) {
                    soft_restart_attempts_++;
                    frames_since_recovery_ = 0;
                    LOG_WARN("Camera", "连续 3 次无帧：先软重启 pipeline（第 ", soft_restart_attempts_,
                             "/2 次，不碰 USB）");
                    if (trySoftPipelineRestart()) {
                        LOG_INFO("Camera", "软重启成功：取流通道已重建，设备与 USB 未动");
                        continue;
                    }
                    LOG_ERROR("Camera", "软重启失败，升级硬重连（设备将重新枚举）");
                } else {
                    LOG_ERROR("Camera", "软重启连续失败两次，相机可能真的断了。Initiating automatic re-connection...");
                }
                resetHardwareConnection();
            }
            continue;
        }

        consecutive_timeouts_ = 0;
        last_frameset_ms_ = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        if (soft_restart_attempts_ > 0 && ++frames_since_recovery_ >= 150) {
            // 恢复后连续 150 帧（~10 s @15fps）正常才算稳住：清账本，下一次停滞再从软重启开始。
            LOG_INFO("Camera", "帧流已稳定 10 s，恢复账本清零");
            soft_restart_attempts_ = 0;
            frames_since_recovery_ = 0;
        }

        try {
            auto color_frame = frameset->colorFrame();
            auto depth_frame = frameset->depthFrame();

            FrameData cur_frame;
            cur_frame.frame_index = frame_count_++;
            cur_frame.timestamp_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            cur_frame.device_ts_us =
                (depth_frame && depth_frame->getTimeStampUs() > 0)
                    ? depth_frame->getTimeStampUs()
                    : ((color_frame && color_frame->getTimeStampUs() > 0)
                           ? color_frame->getTimeStampUs()
                           : 0);

            // 陀螺时间基定标：配对的必须是"**这一帧**的设备戳 ↔ 刚给它打的主机戳"。
            // 冻结的是最小 USB 延迟那一对。follow 不能再用本帧的到达时刻当积分窗口 —— 本帧
            // 延迟一旦比最小值大出几十/几百 ms（336L 实测 350~450 ms），66 ms 窗口刚好框不到
            // 已经按最小延迟换算过的陀螺样本，现象就是"缓冲有样本、积分恒为 0"。
            if (gyro_time_base_.offerPair(cur_frame.device_ts_us,
                                          static_cast<int64_t>(cur_frame.timestamp_ms) * 1000000)) {
                LOG_INFO("Camera", "陀螺时间基已定标: offset=",
                         gyro_time_base_.offset_ns() / 1000000, "ms, 配对=",
                         GyroTimeBase::kProbePairs, "帧, 延迟抖动=",
                         gyro_time_base_.spread_ns() / 1000000, "ms, 定标前丢弃样本=",
                         gyro_time_base_.dropped_before_ready());
            }
            if (gyro_time_base_.ready() && cur_frame.device_ts_us > 0) {
                cur_frame.track_ts_ns = gyro_time_base_.toHostNs(cur_frame.device_ts_us);
            }

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
            std::lock_guard<std::mutex> lock(status_mutex_);
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
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

CameraIntrinsics CameraDriver::getIntrinsics() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return intrinsics_;
}

bool CameraDriver::drainGyroSamples(std::vector<follow::GyroSample>* out) {
    if (!out) return false;
    out->clear();
    std::lock_guard<std::mutex> lock(gyro_mtx_);
    if (gyro_queue_.empty()) return false;
    out->assign(gyro_queue_.begin(), gyro_queue_.end());
    gyro_queue_.clear();
    return true;
}

} // namespace orbbec_service
