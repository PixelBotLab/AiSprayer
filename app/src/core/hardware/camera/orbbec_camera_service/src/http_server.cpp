#include "http_server.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cmath>

namespace orbbec_service {

using json = nlohmann::json;

HttpServer::HttpServer(
    std::shared_ptr<CameraDriver> camera,
    std::shared_ptr<CornerDetector> corner_detector,
    std::shared_ptr<ZlmStreamer> streamer,
    std::shared_ptr<AsyncDiskWriter> disk_writer,
    std::shared_ptr<FollowWorker> follow)
    : camera_(camera), corner_detector_(corner_detector), streamer_(streamer), disk_writer_(disk_writer),
      follow_(follow) {}

HttpServer::~HttpServer() {
    stop();
}

bool HttpServer::start(int port) {
    port_ = port;
    server_ = std::make_unique<httplib::Server>();

    setupRoutes();

    running_ = true;
    server_thread_ = std::thread([this]() {
        LOG_INFO("HTTP", "HTTP REST API Server listening on http://0.0.0.0:", port_);
        server_->listen("0.0.0.0", port_);
        LOG_INFO("HTTP", "HTTP REST API Server stopped.");
    });

    return true;
}

void HttpServer::stop() {
    if (!running_) return;

    running_ = false;
    if (server_) {
        server_->stop();
    }
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    server_.reset();
    LOG_INFO("HTTP", "HttpServer shutdown complete.");
}

namespace {

// follow 的 σ 在"这一帧没有稠密解"时是 +inf。nlohmann 恰好会把非有限浮点写成 null，
// 但那是版本相关的默认行为，不是约定 —— 而 "null = 本帧没测到" 是前端和 app 侧都要依赖的
// 语义（当成 0 会把门限判断变成"精度无限好"）。所以在这里显式化。
json finite_or_null(double v) {
    return std::isfinite(v) ? json(v) : json(nullptr);
}

}  // namespace

json HttpServer::followSnapshotJson(const FollowSnapshot& fs) {
    json j;
    j["enabled"] = fs.enabled;
    j["switching"] = fs.switching;
    j["connected"] = fs.connected;
    j["taught"] = fs.taught;
    j["has_pose"] = fs.has_pose;
    j["status"] = fs.status;
    j["estimator"] = fs.estimator;
    j["reason"] = fs.reason;

    j["pose_mm"] = {fs.pose_mm[0], fs.pose_mm[1], fs.pose_mm[2]};
    j["pose_rpy_deg"] = {fs.pose_rpy_deg[0], fs.pose_rpy_deg[1], fs.pose_rpy_deg[2]};
    j["norm_t_mm"] = fs.norm_t_mm;
    j["norm_r_deg"] = fs.norm_r_deg;
    j["holding_last_pose"] = fs.holding_last_pose;

    json dr = json::array();
    for (int i = 0; i < 3; ++i) {
        dr.push_back({fs.delta_r[i * 3 + 0], fs.delta_r[i * 3 + 1], fs.delta_r[i * 3 + 2]});
    }
    j["delta_r"] = dr;                                   // 行主序 3x3
    j["delta_t_m"] = {fs.delta_t_m[0], fs.delta_t_m[1], fs.delta_t_m[2]};

    j["sigma_t_mm"] = {finite_or_null(fs.sigma_t_mm[0]), finite_or_null(fs.sigma_t_mm[1]),
                       finite_or_null(fs.sigma_t_mm[2])};
    j["sigma_r_deg"] = {finite_or_null(fs.sigma_r_deg[0]), finite_or_null(fs.sigma_r_deg[1]),
                        finite_or_null(fs.sigma_r_deg[2])};
    j["gicp_inliers"] = fs.gicp_inliers;
    j["inlier_ratio"] = fs.inlier_ratio;
    j["gicp_cost"] = fs.gicp_cost;
    j["cloud_points"] = fs.cloud_points;

    j["compute_ms"] = fs.compute_ms;
    j["fps"] = fs.fps;
    j["frames"] = fs.frames;
    j["dropped"] = fs.dropped;
    j["rejected"] = fs.rejected;
    j["smooth_used"] = fs.smooth_used;

    j["map_hash"] = fs.map_hash;
    j["map_voxels"] = fs.map_voxels;
    j["map_path"] = fs.map_path;

    j["align"] = fs.align;
    j["capture_width"] = fs.capture_width;
    j["capture_height"] = fs.capture_height;
    j["teach_capture_width"] = fs.teach_capture_width;
    j["teach_capture_height"] = fs.teach_capture_height;
    j["snapshot_ts_ms"] = fs.snapshot_ts_ms;
    return j;
}

void HttpServer::setupRoutes() {
    // 1. GET /api/v1/camera/status
    server_->Get("/api/v1/camera/status", [this](const httplib::Request& req, httplib::Response& res) {
        CameraStatus st = camera_->getStatus();
        auto calib_cfg = corner_detector_->getConfig();
        st.calibration_mode = calib_cfg.enabled;

        json j;
        j["code"] = 0;
        j["msg"] = "success";
        j["data"] = {
            {"online", st.online},
            {"streaming", st.streaming},
            {"camera_model", st.camera_model},
            {"serial_number", st.serial_number},
            {"firmware_version", st.firmware_version},
            {"color_fps", st.color_fps},
            {"depth_fps", st.depth_fps},
            {"encoder", st.encoder},
            {"calibration_mode", st.calibration_mode},
            {"depth_stream_enabled", st.depth_stream_enabled},
            {"depth_align_enabled", st.depth_align_enabled},
            {"depth_align_mode", st.depth_align_mode},
            {"capture_width", st.capture_width},
            {"capture_height", st.capture_height},
            {"capture_fps", st.capture_fps},
            {"follow_profile", st.follow_profile},
            {"intrinsics_loaded", st.intrinsics_loaded},
            {"total_frames", st.total_frames},
            {"dropped_frames", st.dropped_frames}
        };

        res.set_content(j.dump(), "application/json");
        LOG_DEBUG("HTTP", "GET /api/v1/camera/status -> 200 OK");
    });

    // 2. POST /api/v1/camera/start
    server_->Post("/api/v1/camera/start", [this](const httplib::Request& req, httplib::Response& res) {
        bool ok = camera_->start();
        json j;
        j["code"] = ok ? 0 : -1;
        j["msg"] = ok ? "Camera started successfully" : "Failed to start camera";
        res.set_content(j.dump(), "application/json");
        LOG_INFO("HTTP", "POST /api/v1/camera/start -> ", ok ? "Success" : "Failed");
    });

    // 3. POST /api/v1/camera/stop
    server_->Post("/api/v1/camera/stop", [this](const httplib::Request& req, httplib::Response& res) {
        camera_->stop();
        json j;
        j["code"] = 0;
        j["msg"] = "Camera stopped successfully";
        res.set_content(j.dump(), "application/json");
        LOG_INFO("HTTP", "POST /api/v1/camera/stop -> Success");
    });

    // 4. GET /api/v1/camera/intrinsics
    server_->Get("/api/v1/camera/intrinsics", [this](const httplib::Request& req, httplib::Response& res) {
        CameraIntrinsics intr = camera_->getIntrinsics();
        json j;
        j["code"] = 0;
        j["msg"] = "success";
        j["data"] = {
            {"camera_model", intr.camera_model},
            {"width", intr.width},
            {"height", intr.height},
            {"intrinsic_matrix", intr.intrinsic_matrix},
            {"distortion_coeffs", intr.distortion_coeffs},
            {"distortion_model", intr.distortion_model},
            {"depth_scale", intr.depth_scale}
        };
        res.set_content(j.dump(), "application/json");
        LOG_INFO("HTTP", "GET /api/v1/camera/intrinsics -> 200 OK");
    });

    // 5. POST /api/v1/camera/calibration_mode
    server_->Post("/api/v1/camera/calibration_mode", [this](const httplib::Request& req, httplib::Response& res) {
        try {
            json body = json::parse(req.body);
            CalibrationConfig cfg = corner_detector_->getConfig();

            if (body.contains("enabled")) cfg.enabled = body["enabled"].get<bool>();
            if (body.contains("board_type")) cfg.board_type = body["board_type"].get<std::string>();
            if (body.contains("rows")) cfg.rows = body["rows"].get<int>();
            if (body.contains("cols")) cfg.cols = body["cols"].get<int>();
            if (body.contains("square_size_mm")) cfg.square_size_mm = body["square_size_mm"].get<double>();
            if (body.contains("draw_corners")) cfg.draw_corners = body["draw_corners"].get<bool>();

            // 1. Update Corner Detector
            corner_detector_->setConfig(cfg);

            // 1b. 切模式**之前**先记下 follow 现在开着没：这是"是否被顶掉"的唯一可靠口径。
            const bool follow_was_enabled = follow_ && follow_->enabled();

            // 2. Update Camera Driver streaming & depth alignment mode
            // If enabled == true: turns OFF depth stream & D2C alignment (0% Depth CPU overhead)
            // If enabled == false: turns ON depth stream & D2C alignment
            camera_->setCalibrationMode(cfg.enabled);

            // 3. 进标定时把 follow 关掉。关档现在是**提交式**的（切换在 worker 线程里跑，
            //    重启取流可能要几十秒，不能压在这个请求里）：这里只提交并如实报告，收敛由
            //    轮询确认。标定模式本身由 CameraDriver 的 calib_mode_ 标志已生效，不受影响。
            if (cfg.enabled && follow_was_enabled && follow_) {
                std::string follow_err;
                bool follow_busy = false;
                if (!follow_->setEnabled(false, &follow_err, &follow_busy) && !follow_busy) {
                    LOG_WARN("HTTP", "切标定模式时提交关闭 follow 失败：", follow_err);
                }
            }
            // "被顶掉了没"的口径：关档请求已提交即算（目标态已定，切换只是时间问题）。
            const bool follow_auto_disabled =
                cfg.enabled && follow_was_enabled &&
                !(follow_ && follow_->enabled() && !follow_->switching());

            cv::Size p_size = cfg.getPatternSize();
            json j;
            j["code"] = 0;
            j["msg"] = cfg.enabled ? "Calibration mode enabled (Depth stream & alignment DISABLED, Corner detection ENABLED)" 
                                   : "Normal mode enabled (Depth stream & alignment ENABLED, Corner detection DISABLED)";
            j["data"] = {
                {"calibration_mode", cfg.enabled},
                {"corner_detection", cfg.enabled},
                {"depth_stream", !cfg.enabled},
                {"depth_alignment", !cfg.enabled},
                {"board_type", cfg.board_type},
                {"rows", cfg.rows},
                {"cols", cfg.cols},
                {"square_size_mm", cfg.square_size_mm},
                {"pattern_size", {p_size.width, p_size.height}},
                {"follow_enabled_after", follow_ ? follow_->enabled() : false},
                {"follow_auto_disabled", follow_auto_disabled}
            };
            res.set_content(j.dump(), "application/json");
            LOG_INFO("HTTP", "POST /api/v1/camera/calibration_mode -> calibration_mode=", cfg.enabled, 
                     ", depth_stream=", !cfg.enabled, ", corner_detector=", cfg.enabled);
        } catch (const std::exception& e) {
            json j;
            j["code"] = -1;
            j["msg"] = std::string("Invalid JSON payload: ") + e.what();
            res.status = 400;
            res.set_content(j.dump(), "application/json");
            LOG_ERROR("HTTP", "POST /api/v1/camera/calibration_mode error: ", e.what());
        }
    });

    // 6. GET /api/v1/camera/corners
    server_->Get("/api/v1/camera/corners", [this](const httplib::Request& req, httplib::Response& res) {
        DetectedCorners corners;
        bool found = corner_detector_->getLatestCorners(corners);

        json j;
        j["code"] = 0;
        j["msg"] = "success";
        json corner_list = json::array();
        for (const auto& pt : corners.corners) {
            corner_list.push_back({pt.x, pt.y});
        }

        j["data"] = {
            {"found", found},
            {"timestamp_ms", corners.timestamp_ms},
            {"pattern_size", {corners.pattern_cols, corners.pattern_rows}},
            {"corners_count", corners.corners.size()},
            {"corners", corner_list},
            {"detection_time_ms", corners.detection_time_ms}
        };
        res.set_content(j.dump(), "application/json");
        LOG_DEBUG("HTTP", "GET /api/v1/camera/corners -> found=", found);
    });

    // 7. POST /api/v1/camera/save_frame
    server_->Post("/api/v1/camera/save_frame", [this](const httplib::Request& req, httplib::Response& res) {
        try {
            json body = json::parse(req.body);
            SaveFrameRequest save_req;
            if (body.contains("save_dir")) save_req.save_dir = body["save_dir"].get<std::string>();
            if (body.contains("prefix")) save_req.prefix = body["prefix"].get<std::string>();
            if (body.contains("color_filename")) save_req.color_filename = body["color_filename"].get<std::string>();
            if (body.contains("depth_filename")) save_req.depth_filename = body["depth_filename"].get<std::string>();
            if (body.contains("save_color")) save_req.save_color = body["save_color"].get<bool>();
            if (body.contains("save_depth")) save_req.save_depth = body["save_depth"].get<bool>();
            if (body.contains("save_info_yaml")) save_req.save_info_yaml = body["save_info_yaml"].get<bool>();
            if (body.contains("color_format")) save_req.color_format = body["color_format"].get<std::string>();
            if (body.contains("depth_format")) save_req.depth_format = body["depth_format"].get<std::string>();
            if (body.contains("metadata")) save_req.metadata = body["metadata"];

            FrameData cur_frame;
            if (!camera_->getLatestFrame(cur_frame)) {
                json j;
                j["code"] = -2;
                j["msg"] = "No frame available from camera";
                res.status = 503;
                res.set_content(j.dump(), "application/json");
                LOG_ERROR("HTTP", "save_frame failed: No frame available");
                return;
            }

            CameraIntrinsics intr = camera_->getIntrinsics();
            DetectedCorners corners;
            corner_detector_->getLatestCorners(corners);

            auto future_res = disk_writer_->saveFrameAsync(save_req, cur_frame, intr, corners);
            auto status = future_res.wait_for(std::chrono::seconds(5));

            if (status == std::future_status::ready) {
                SaveFrameResult r = future_res.get();
                json j;
                j["code"] = r.success ? 0 : -1;
                j["msg"] = r.success ? "Frame saved successfully" : r.error_msg;
                j["data"] = {
                    {"frame_id", r.frame_id},
                    {"timestamp_ms", r.timestamp_ms},
                    {"color_file", r.color_file},
                    {"depth_file", r.depth_file},
                    {"info_file", r.info_file},
                    {"corners_found", r.corners_found}
                };
                res.set_content(j.dump(), "application/json");
                LOG_INFO("HTTP", "POST /api/v1/camera/save_frame -> Success: ", r.color_file, ", ", r.depth_file);
            } else {
                json j;
                j["code"] = -3;
                j["msg"] = "Disk write timeout (5s)";
                res.status = 504;
                res.set_content(j.dump(), "application/json");
                LOG_ERROR("HTTP", "POST /api/v1/camera/save_frame -> Timeout");
            }
        } catch (const std::exception& e) {
            json j;
            j["code"] = -1;
            j["msg"] = std::string("save_frame error: ") + e.what();
            res.status = 400;
            res.set_content(j.dump(), "application/json");
            LOG_ERROR("HTTP", "POST /api/v1/camera/save_frame exception: ", e.what());
        }
    });

    // 8. GET /api/v1/stream/info
    server_->Get("/api/v1/stream/info", [this](const httplib::Request& req, httplib::Response& res) {
        std::string host = req.get_header_value("Host");
        std::string host_ip = "127.0.0.1";
        if (!host.empty()) {
            size_t colon_pos = host.find(':');
            host_ip = (colon_pos != std::string::npos) ? host.substr(0, colon_pos) : host;
        }

        StreamInfo info = streamer_->getStreamInfo(host_ip);
        json j;
        j["code"] = 0;
        j["msg"] = "success";
        j["data"] = {
            {"stream_id", info.stream_id},
            {"webrtc_url", info.webrtc_url},
            {"rtsp_url", info.rtsp_url},
            {"http_flv_url", info.http_flv_url},
            {"width", info.width},
            {"height", info.height},
            {"fps", info.fps}
        };
        res.set_content(j.dump(), "application/json");
        LOG_DEBUG("HTTP", "GET /api/v1/stream/info -> 200 OK");
    });

    // 9. GET /api/v1/camera/latest_frame.jpg
    server_->Get("/api/v1/camera/latest_frame.jpg", [this](const httplib::Request& req, httplib::Response& res) {
        FrameData frame;
        if (camera_->getLatestFrame(frame) && frame.has_color && !frame.color.empty()) {
            cv::Mat draw_img = frame.color.clone();
            corner_detector_->drawOverlay(draw_img);

            std::vector<uint8_t> jpg_buf;
            cv::imencode(".jpg", draw_img, jpg_buf, {cv::IMWRITE_JPEG_QUALITY, 85});
            res.set_content((const char*)jpg_buf.data(), jpg_buf.size(), "image/jpeg");
        } else {
            res.status = 503;
            res.set_content("No frame available", "text/plain");
        }
    });

    // 9.1 GET /api/v1/camera/latest_depth.png (16-bit PNG Depth map)
    server_->Get("/api/v1/camera/latest_depth.png", [this](const httplib::Request& req, httplib::Response& res) {
        FrameData frame;
        if (camera_->getLatestFrame(frame) && frame.has_depth && !frame.depth.empty()) {
            std::vector<uint8_t> png_buf;
            cv::imencode(".png", frame.depth, png_buf);
            res.set_content((const char*)png_buf.data(), png_buf.size(), "image/png");
        } else {
            res.status = 503;
            res.set_content("No depth frame available", "text/plain");
        }
    });

    // ---- follow（进程内跟随）--------------------------------------------------
    // 三条路由的响应体都是 followSnapshotJson()：点了开关之后的返回、示教之后的返回、
    // 和页面轮询到的返回必须是同一个形状，前端不需要按"哪个接口回的"分两套解析。

    auto follow_unavailable = [](httplib::Response& res, const char* route) {
        json j;
        j["code"] = -1;
        j["msg"] = "follow worker not available";
        res.status = 503;
        res.set_content(j.dump(), "application/json");
        LOG_ERROR("HTTP", route, " -> follow worker 不存在");
    };

    // 10. POST /api/v1/camera/follow {"enabled": true|false}
    //     enabled 是**必填**：这个开关会重启取流 pipeline（切 640x480 + 硬件 D2C），
    //     而 follow 的默认状态是关。允许"解析不到就当默认值"意味着一个畸形请求能把相机档位
    //     悄悄改掉 —— 宁可 400。
    //     受理即返回（202），切换在 worker 的专用线程里跑：重启 pipeline 实测最长几十秒，
    //     同步压在这里会直接超过客户端的请求超时（旧版"使能失败、接口超时"的根因）。
    //     进度/结果由 GET /api/v1/camera/follow/status 的 switching 字段轮询。
    //     409 = 已有一次切换在跑（调用方该去轮询，不是重试）。
    server_->Post("/api/v1/camera/follow", [this, follow_unavailable](const httplib::Request& req, httplib::Response& res) {
        if (!follow_) { follow_unavailable(res, "POST /api/v1/camera/follow"); return; }
        bool enabled = false;
        try {
            json body = json::parse(req.body);
            if (!body.contains("enabled") || !body["enabled"].is_boolean()) {
                json j;
                j["code"] = -1;
                j["msg"] = "Missing or non-boolean required field: enabled";
                res.status = 400;
                res.set_content(j.dump(), "application/json");
                LOG_WARN("HTTP", "POST /api/v1/camera/follow -> 400（enabled 必填且必须是布尔）");
                return;
            }
            enabled = body["enabled"].get<bool>();
        } catch (const std::exception& e) {
            json j;
            j["code"] = -1;
            j["msg"] = std::string("Invalid JSON payload: ") + e.what();
            res.status = 400;
            res.set_content(j.dump(), "application/json");
            LOG_ERROR("HTTP", "POST /api/v1/camera/follow error: ", e.what());
            return;
        }

        std::string err;
        bool busy = false;
        const bool ok = follow_->setEnabled(enabled, &err, &busy);
        json j;
        if (ok) {
            j["code"] = 0;
            j["msg"] = follow_->switching()
                           ? std::string(enabled ? "follow 使能已受理，切换进行中" : "follow 关闭已受理，切换进行中")
                           : std::string(enabled ? "follow enabled" : "follow disabled");
            res.status = 202;   // 受理（含无需切换的同态请求）：真正终态由轮询确认
        } else {
            j["code"] = -1;
            j["msg"] = err;
            res.status = busy ? 409 : 503;
        }
        j["data"] = followSnapshotJson(follow_->snapshot());
        res.set_content(j.dump(), "application/json");
        LOG_INFO("HTTP", "POST /api/v1/camera/follow enabled=", enabled, " -> ",
                 ok ? "accepted" : (busy ? "busy" : "failed"), ok ? "" : (": " + err));
    });

    // 11. POST /api/v1/camera/follow/teach {"save_map": false}
    server_->Post("/api/v1/camera/follow/teach", [this, follow_unavailable](const httplib::Request& req, httplib::Response& res) {
        if (!follow_) { follow_unavailable(res, "POST /api/v1/camera/follow/teach"); return; }
        bool save_map = false;
        if (!req.body.empty()) {
            try {
                json body = json::parse(req.body);
                if (body.contains("save_map")) save_map = body["save_map"].get<bool>();
            } catch (const std::exception& e) {
                json j;
                j["code"] = -1;
                j["msg"] = std::string("Invalid JSON payload: ") + e.what();
                res.status = 400;
                res.set_content(j.dump(), "application/json");
                LOG_ERROR("HTTP", "POST /api/v1/camera/follow/teach error: ", e.what());
                return;
            }
        }

        // 没使能时深度流不一定在 follow 档位上，示教收上来的基准可能和运行期不是一套几何 ——
        // 这种情况明确拒绝（409），而不是建一个"看着成功、每帧都在跟坏基准比"的地图。
        if (!follow_->enabled()) {
            json j;
            j["code"] = -1;
            j["msg"] = "follow 未使能：先 POST /api/v1/camera/follow {\"enabled\":true} 再示教";
            j["data"] = followSnapshotJson(follow_->snapshot());
            res.status = 409;
            res.set_content(j.dump(), "application/json");
            LOG_WARN("HTTP", "POST /api/v1/camera/follow/teach -> 409（follow 未使能）");
            return;
        }

        std::string err;
        const bool ok = follow_->teach(save_map, &err);
        json j;
        j["code"] = ok ? 0 : -1;
        j["msg"] = ok ? (save_map ? "reference map taught and saved" : "reference map taught") : err;
        j["data"] = followSnapshotJson(follow_->snapshot());
        if (!ok) res.status = 503;
        res.set_content(j.dump(), "application/json");
        LOG_INFO("HTTP", "POST /api/v1/camera/follow/teach save_map=", save_map, " -> ",
                 ok ? "ok" : ("failed: " + err));
    });

    // 12. GET /api/v1/camera/follow/status
    //     只读快照：绝不在 HTTP 线程里跑 GICP（一帧几十毫秒，会把 REST 接口卡成串行队列）。
    server_->Get("/api/v1/camera/follow/status", [this, follow_unavailable](const httplib::Request& req, httplib::Response& res) {
        if (!follow_) { follow_unavailable(res, "GET /api/v1/camera/follow/status"); return; }
        json j;
        j["code"] = 0;
        j["msg"] = "success";
        j["data"] = followSnapshotJson(follow_->snapshot());
        res.set_content(j.dump(), "application/json");
    });
}

} // namespace orbbec_service
