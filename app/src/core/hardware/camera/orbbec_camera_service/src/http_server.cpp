#include "http_server.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>

namespace orbbec_service {

using json = nlohmann::json;

HttpServer::HttpServer(
    std::shared_ptr<CameraDriver> camera,
    std::shared_ptr<CornerDetector> corner_detector,
    std::shared_ptr<ZlmStreamer> streamer,
    std::shared_ptr<AsyncDiskWriter> disk_writer)
    : camera_(camera), corner_detector_(corner_detector), streamer_(streamer), disk_writer_(disk_writer) {}

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

            // 2. Update Camera Driver streaming & depth alignment mode
            // If enabled == true: turns OFF depth stream & D2C alignment (0% Depth CPU overhead)
            // If enabled == false: turns ON depth stream & D2C alignment
            camera_->setCalibrationMode(cfg.enabled);

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
                {"pattern_size", {p_size.width, p_size.height}}
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
}

} // namespace orbbec_service
