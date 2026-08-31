#include <iostream>
#include <memory>
#include <thread>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cstdio>
#include <iomanip>

#include "types.hpp"
#include "logger.hpp"
#include "config.hpp"
#include "camera_driver.hpp"
#include "rga_processor.hpp"
#include "mpp_encoder.hpp"
#include "zlm_streamer.hpp"
#include "corner_detector.hpp"
#include "async_disk_writer.hpp"
#include "http_server.hpp"
#include "follow_worker.hpp"
#include "follow/config_loader.hpp"

using namespace orbbec_service;

static std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int sig) {
    LOG_INFO("Main", "Received signal ", sig, ", initiating graceful shutdown...");
    g_shutdown_requested = true;
}

void printBanner() {
    std::cout << "\033[36m"
              << "========================================================================\n"
              << "     _    _ ____                                                        \n"
              << "    / \\  (_) ___| _ __  _ __ __ _ _   _  ___ _ __                       \n"
              << "   / _ \\ | \\___ \\| '_ \\| '__/ _` | | | |/ _ \\ '__|                      \n"
              << "  / ___ \\| |___) | |_) | | | (_| | |_| |  __/ |                         \n"
              << " /_/   \\_\\_|____/| .__/|_|  \\__,_|\\__, |\\___|_|                         \n"
              << "                 |_|              |___/                                 \n"
              << "     RK3588 High-Performance Orbbec C++ Camera Microservice            \n"
              << "========================================================================\n"
              << "\033[0m" << std::endl;
}

struct LatencyMetric {
    uint64_t count = 0;
    double sum_ms = 0.0;
    double min_ms = 1e9;
    double max_ms = 0.0;

    void update(double ms) {
        count++;
        sum_ms += ms;
        if (ms < min_ms) min_ms = ms;
        if (ms > max_ms) max_ms = ms;
    }

    double getAvg() const {
        return count > 0 ? (sum_ms / count) : 0.0;
    }

    double getMin() const {
        return count > 0 ? min_ms : 0.0;
    }

    double getMax() const {
        return count > 0 ? max_ms : 0.0;
    }

    void reset() {
        count = 0;
        sum_ms = 0.0;
        min_ms = 1e9;
        max_ms = 0.0;
    }
};

struct PipelineStats {
    LatencyMetric rga_cvt;
    LatencyMetric mpp_enc;
    LatencyMetric zlm_push;
    LatencyMetric total_pipeline;

    void reset() {
        rga_cvt.reset();
        mpp_enc.reset();
        zlm_push.reset();
        total_pipeline.reset();
    }
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::string config_path = "";
    bool raw_log = false;
    int stats_interval_sec = -1;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--raw-log" || arg == "--simple-log") {
            raw_log = true;
        } else if ((arg == "--stats-interval" || arg == "--report-interval" || arg == "-i") && i + 1 < argc) {
            try {
                stats_interval_sec = std::stoi(argv[++i]);
            } catch (...) {
                stats_interval_sec = 10;
            }
        }
    }

    if (raw_log) {
        Logger::getInstance().setSimpleFormat(true);
    } else {
        printBanner();
    }

    // 1. Load Configuration
    AppConfig config = ConfigLoader::load(config_path);

    if (stats_interval_sec < 0) {
        stats_interval_sec = config.stats_interval_sec;
    }

    // 1b. follow 的配置：读的是**同一个 yaml 的同一个 follow: 块**，解析器只有 follow::load_config
    // 一个（在这里抄一份的结果是"新键在那边不认"，而且没人报错）。相机服务从这份配置里只取两样
    // 东西 —— 设备锁路径和取流档位，其余全归 FollowWorker。
    follow::FollowConfig fcfg;
    std::string follow_block_reason;
    std::string ferr;
    if (!follow::load_config(config_path, &fcfg, &ferr)) {
        follow_block_reason = ferr;
        LOG_ERROR("Follow", "配置读取失败，follow 不可用（相机服务照常运行）: ", ferr);
    } else {
        const follow::ConfigProblems fprobs = follow::check_config(&fcfg, follow::find_project_root());
        for (const auto& p : fprobs.items) {
            if (p.fatal) {
                LOG_ERROR("Follow", "配置致命: ", p.text);
            } else {
                LOG_WARN("Follow", "配置提示: ", p.text);
            }
        }
        if (!fprobs.ok()) {
            follow_block_reason = fprobs.joined();
        }
    }
    // 锁路径为空 ⇒ CameraDriver 不做进程间仲裁（并会为此单独喊一次）。配置坏了就不能拿一个可能
    // 不对的锁路径去挡别人，所以这里连同锁一起放弃。
    config.device_lock_path = follow_block_reason.empty() ? fcfg.capture.lock_path : std::string();
    // 即便 follow 配置被 block，这个键仍按解析结果交给 CameraDriver：默认 true，关了必须两边一起关。
    config.enable_imu = fcfg.capture.enable_imu;

    LOG_INFO("Main", "Initializing Core Subsystems on RK3588 (Stats report interval: ", stats_interval_sec, "s)...");

    // 2. Instantiate Subsystems
    auto camera = std::make_shared<CameraDriver>();
    auto rga = std::make_shared<RgaProcessor>();
    auto mpp = std::make_shared<MppEncoder>();
    auto zlm = std::make_shared<ZlmStreamer>();
    auto corner_detector = std::make_shared<CornerDetector>();
    auto disk_writer = std::make_shared<AsyncDiskWriter>();

    // 3. Initialize Subsystems
    corner_detector->setConfig(config.calib_default);
    
    if (!rga->init(config.stream_width, config.stream_height)) {
        LOG_WARN("Main", "RGA Processor initialization returned warning.");
    }

    if (!mpp->init(config.stream_width, config.stream_height, config.stream_fps, config.stream_bitrate_kbps)) {
        LOG_ERROR("Main", "MPP Hardware Encoder initialization failed!");
    }

    if (!zlm->init(config)) {
        LOG_ERROR("Main", "ZLMediaKit Streamer initialization failed!");
    }

    if (!disk_writer->init(config, 2)) {
        LOG_ERROR("Main", "Async Disk Writer initialization failed!");
    }

    if (!camera->init(config)) {
        LOG_ERROR("Main", "Camera Driver initialization failed!");
    }

    // follow 作为库跑在本进程里（默认不使能）。线程一上来就睡着，除非有人 POST /follow。
    auto follow_worker = std::make_shared<FollowWorker>(camera, fcfg);
    if (!follow_block_reason.empty()) {
        follow_worker->setBlocked(follow_block_reason);   // setEnabled 会拒绝，/follow/status 报同一条
    }
    follow_worker->start();

    // Start Camera Stream
    camera->start();

    // 4. Start HTTP Server
    auto http_server = std::make_shared<HttpServer>(camera, corner_detector, zlm, disk_writer,
                                                    follow_worker);
    http_server->start(config.http_port);

    LOG_INFO("Main", "All services started successfully! Entering main processing loop.");

    // Buffers for streaming pipeline
    int nv12_size = config.stream_width * config.stream_height * 3 / 2;
    std::vector<uint8_t> h264_buffer(nv12_size);

    uint64_t frame_index = 0;
    auto last_log_time = std::chrono::steady_clock::now();
    PipelineStats stats;

    // Direct pointer to MPP DMA frame buffer for hardware Zero-Copy rendering
    void* mpp_frm_ptr = mpp->getFrameBufferPtr();

    // 5. Main Streaming & Processing Pipeline Loop (Event-driven)
    while (!g_shutdown_requested) {
        FrameData cur_frame;
        // Event-driven: wait directly on camera hardware frame ready interrupt
        if (!camera->waitForNextFrame(cur_frame, 100) || !cur_frame.has_color || cur_frame.color.empty()) {
            continue;
        }

        auto t_pipe_start = std::chrono::high_resolution_clock::now();
        cv::Mat display_mat = cur_frame.color; // Shallow reference (Zero-copy!)

        // Stage 1: Feed frame to Asynchronous Corner Detection Worker (Non-blocking)
        auto calib_cfg = corner_detector->getConfig();
        if (calib_cfg.enabled) {
            corner_detector->feedFrame(cur_frame.color);
            if (calib_cfg.draw_corners) {
                display_mat = cur_frame.color.clone(); // Only clone when drawing is active!
                corner_detector->drawOverlay(display_mat);
            }
        }

        // Stage 2: RGA Hardware Direct Color Space Conversion (BGR888 -> Direct MPP DMA Buffer, Zero memcpy!)
        auto t_rga_start = std::chrono::high_resolution_clock::now();
        uint8_t* dst_ptr = (uint8_t*)mpp_frm_ptr;
        bool rga_ok = false;
        if (dst_ptr != nullptr) {
            rga_ok = rga->bgrToNv12(display_mat, dst_ptr, config.stream_width, config.stream_height);
        }
        auto t_rga_end = std::chrono::high_resolution_clock::now();
        double rga_ms = std::chrono::duration<double, std::milli>(t_rga_end - t_rga_start).count();
        if (rga_ok) stats.rga_cvt.update(rga_ms);

        if (rga_ok) {
            // Stage 3: MPP Direct Hardware H.264 Encoding (Zero memcpy!)
            int out_h264_len = 0;
            bool is_keyframe = false;
            uint64_t pts = cur_frame.timestamp_ms;

            auto t_mpp_start = std::chrono::high_resolution_clock::now();
            bool mpp_ok = mpp->encodeDirect(h264_buffer.data(), h264_buffer.size(), 
                                            out_h264_len, is_keyframe, pts);
            auto t_mpp_end = std::chrono::high_resolution_clock::now();
            double mpp_ms = std::chrono::duration<double, std::milli>(t_mpp_end - t_mpp_start).count();
            if (mpp_ok) stats.mpp_enc.update(mpp_ms);

            if (mpp_ok) {
                // Stage 4: Push NALU into ZLMediaKit
                auto t_zlm_start = std::chrono::high_resolution_clock::now();
                zlm->pushH264(h264_buffer.data(), out_h264_len, pts, pts);
                auto t_zlm_end = std::chrono::high_resolution_clock::now();
                double zlm_ms = std::chrono::duration<double, std::milli>(t_zlm_end - t_zlm_start).count();
                stats.zlm_push.update(zlm_ms);
            }
        }

        auto t_pipe_end = std::chrono::high_resolution_clock::now();
        double pipe_ms = std::chrono::duration<double, std::milli>(t_pipe_end - t_pipe_start).count();
        stats.total_pipeline.update(pipe_ms);

        frame_index++;

        // Periodic Health & Performance Latency Report (Default every 10 seconds, controlled by --stats-interval)
        auto now = std::chrono::steady_clock::now();
        if (stats_interval_sec > 0 &&
            std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= stats_interval_sec) {
            CameraStatus st = camera->getStatus();
            StreamInfo s_info = zlm->getStreamInfo("127.0.0.1");

            std::stringstream ss_corner, ss_rga, ss_mpp, ss_zlm, ss_total;
            ss_rga << std::fixed << std::setprecision(2) << "avg = " << std::setw(5) << stats.rga_cvt.getAvg() 
                   << " ms | min = " << std::setw(5) << stats.rga_cvt.getMin() 
                   << " ms | max = " << std::setw(5) << stats.rga_cvt.getMax() << " ms";
            ss_mpp << std::fixed << std::setprecision(2) << "avg = " << std::setw(5) << stats.mpp_enc.getAvg() 
                   << " ms | min = " << std::setw(5) << stats.mpp_enc.getMin() 
                   << " ms | max = " << std::setw(5) << stats.mpp_enc.getMax() << " ms";
            ss_zlm << std::fixed << std::setprecision(2) << "avg = " << std::setw(5) << stats.zlm_push.getAvg() 
                   << " ms | min = " << std::setw(5) << stats.zlm_push.getMin() 
                   << " ms | max = " << std::setw(5) << stats.zlm_push.getMax() << " ms";
            ss_total << std::fixed << std::setprecision(2) << "avg = " << std::setw(5) << stats.total_pipeline.getAvg() 
                     << " ms | min = " << std::setw(5) << stats.total_pipeline.getMin() 
                     << " ms | max = " << std::setw(5) << stats.total_pipeline.getMax() << " ms";

            LOG_INFO("Status", "=== [Service Health & Stage Latency Report] ===");
            LOG_INFO("Status", "  Camera Device   : [Model: ", st.camera_model, ", Online: ", (st.online ? "YES" : "NO"), 
                     ", Capture FPS: ", st.color_fps, ", Depth Stream: ", (st.depth_stream_enabled ? "\033[32mON\033[0m" : "\033[33mOFF (Calib Mode)\033[0m"), 
                     ", Depth Align: ", (st.depth_align_enabled ? "\033[32mON\033[0m" : "\033[33mOFF\033[0m"), 
                     ", Total Frames: ", st.total_frames, "]");
            // 报**实际交付**的档位（不是 hardware.camera 里写的），因为 follow 使能时它会被整档换掉，
            // 而内参是按哪一档取的直接决定点云对不对。align=disabled 时深度根本不在彩色像素系里。
            LOG_INFO("Status", "  Delivered Stream: ", st.capture_width, "x", st.capture_height,
                     " @ ", st.capture_fps, "fps  align=", st.depth_align_mode,
                     "  intrinsics=", (st.intrinsics_loaded ? "loaded" : "DEFAULT(untrusted)"),
                     "  profile=", (st.follow_profile ? "follow" : "hardware.camera"));

            const FollowSnapshot fs = follow_worker->snapshot();
            char pose_buf[128];
            std::snprintf(pose_buf, sizeof(pose_buf), "X=%+.1f Y=%+.1f Z=%+.1f mm  rx=%+.2f ry=%+.2f rz=%+.2f deg",
                          fs.pose_mm[0], fs.pose_mm[1], fs.pose_mm[2],
                          fs.pose_rpy_deg[0], fs.pose_rpy_deg[1], fs.pose_rpy_deg[2]);
            LOG_INFO("Status", "  Follow (in-proc): [Enabled: ", (fs.enabled ? "\033[32mON\033[0m" : "\033[33mOFF\033[0m"),
                     ", ", fs.status, "/", fs.estimator, ", Taught: ", (fs.taught ? "YES" : "NO"),
                     ", ", pose_buf, ", inl=", fs.gicp_inliers, "(", fs.inlier_ratio, ")",
                     ", compute=", fs.compute_ms, "ms, fps=", fs.fps,
                     ", frames=", fs.frames, ", dropped=", fs.dropped, ", rejected=", fs.rejected, "]");
            if (!fs.reason.empty()) {
                // 不 ok 时位姿是**上一个可信值**，这一行就是把那件事说给人听，否则读数会被当真。
                LOG_INFO("Status", "    \033[33m> \033[0m", fs.reason,
                         (fs.holding_last_pose ? "  <位姿保持上一可信值>" : ""));
            }
            
            if (calib_cfg.enabled) {
                WorkerLatencyStats w_stats = corner_detector->getWorkerStats(true);
                ss_corner << std::fixed << std::setprecision(2) << "avg = " << std::setw(5) << w_stats.getAvg() 
                          << " ms | min = " << std::setw(5) << w_stats.getMin() 
                          << " ms | max = " << std::setw(5) << w_stats.getMax() << " ms (Worker samples: " << w_stats.count << ")";
                LOG_INFO("Status", "  Calibration Mode: \033[32m[ENABLED]\033[0m (Grid: ", calib_cfg.rows, "x", calib_cfg.cols, 
                         ", Square: ", calib_cfg.square_size_mm, "mm, Draw: ", calib_cfg.draw_corners, ")");
                LOG_INFO("Status", "  * [Corner Worker] : ", ss_corner.str());
            } else {
                LOG_INFO("Status", "  Calibration Mode: \033[33m[DISABLED]\033[0m (Post /api/v1/camera/calibration_mode to enable)");
                LOG_INFO("Status", "  * [Corner Worker] : OFF (skipped)");
            }

            LOG_INFO("Status", "  * [RGA BGR->NV12] : ", ss_rga.str());
            LOG_INFO("Status", "  * [MPP H.264 Enc] : ", ss_mpp.str());
            LOG_INFO("Status", "  * [ZLM StreamPush]: ", ss_zlm.str());
            LOG_INFO("Status", "  * [Total Pipeline]: ", ss_total.str());
            LOG_INFO("Status", "  Hardware Engines: RGA2D=ACTIVE | MPP=ACTIVE | Mali-GPU=", 
                     (corner_detector->isGpuAccelerated() ? "ACTIVE (OpenCL)" : "Disabled"));
            LOG_INFO("Status", "  Endpoints: HTTP-FLV: ", s_info.http_flv_url, " | RTSP: ", s_info.rtsp_url);

            stats.reset();
            last_log_time = now;
        }
    }

    LOG_INFO("Main", "Stopping all services...");
    http_server->stop();
    follow_worker->stop();   // 必须先于 camera：它是帧的消费者，也是 follow 档位的发起者
    camera->stop();
    disk_writer->stop();
    corner_detector->stop();
    zlm->release();
    mpp->release();
    rga->release();

    LOG_INFO("Main", "AiSprayer Orbbec Camera Service cleanly exited. Goodbye!");
    return 0;
}
