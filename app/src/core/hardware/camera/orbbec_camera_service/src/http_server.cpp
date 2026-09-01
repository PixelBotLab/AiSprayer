#include "http_server.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cmath>
#include "pose_broker.hpp"

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

    // SSE 订阅者会**长期占用一个线程池线程**（provider 不返回 ⇒ 这条连接的 handler 不结束）。
    // httplib 默认池是 max(8, cores-1)，本机 8 核 ⇒ 一共 8 路，而 follow stream 的上限就有 4 路：
    // 几个残留客户端就能把控制面（开关/示教/状态）全排在后面，表象是"点了停止没反应"。
    // 这些线程绝大多数时间在 CV 上睡着，多给十几个不花钱。
    server_->new_task_queue = [] { return new httplib::ThreadPool(24); };

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
    // 先叫醒在等位姿的 SSE provider，再停 server：httplib 的 stop() 要等这些长连接的 handler 自己
    // 结束，而 handler 每次醒来只做一次判定就回去接着睡。不先握手的话，每条残留连接都会让停机
    // 多等一个心跳周期，而这条链路上唯一"该退出了"的信号只有 broker 能给。
    if (follow_) follow_->poseBroker().shutdown();
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

// 一路 SSE 订阅占的名额。还名额有两个可能的时机：httplib 的 resource releaser（响应对象
// 析构时）和这个 guard 自己的析构 —— 两者谁先到都必须只还一次，还不掉的话配额会被慢慢吃光，
// 表现是"推送连上几次之后再也不让连"。
class StreamSlot {
public:
    explicit StreamSlot(PoseBroker& broker) : broker_(broker.subscribe() ? &broker : nullptr) {}
    ~StreamSlot() { release(); }
    StreamSlot(const StreamSlot&) = delete;
    StreamSlot& operator=(const StreamSlot&) = delete;
    bool ok() const { return broker_ != nullptr; }
    void release() {
        if (broker_) {
            broker_->unsubscribe();
            broker_ = nullptr;
        }
    }

private:
    PoseBroker* broker_;   // 空 = 不持有名额（要么没抢到，要么已经还掉了）
};

/** 单调毫秒：只用来掐心跳节奏，所以不需要墙上时间。 */
inline int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 一条连接的读游标。first 的意义：刚接上来的客户端必须先拿到**当前**这一帧，而不是等下一次
// 快照写 —— 没有新帧时（相机挡着、还没示教）它至少要等一个心跳才有内容可看。
struct StreamState {
    uint64_t seen = 0;
    bool first = true;
    int64_t last_out_ms = 0;       // 上一次往这条连接写任何字节（事件或心跳）的时刻
    /**
     * 上一次**真的发出去**的那份快照的内容指纹。
     * 按"快照被重写过"来发会虚胖一倍：worker 的空闲分支每 kIdleSleepMs 也要往前挪一次
     * snapshot_ts_ms（实测 15 fps 的解算对着 30 事件/秒的流），而订阅者拿到的是除时钟外
     * 逐字节相同的一帧。留着 frames + status 两个键当指纹：它们合起来覆盖"解出了一帧"和
     * "状态变了（等待/受阻/丢目标）"两类真正需要立刻知道的事。
     *
     * 去重之后"等了 200ms 没人写快照"就不再等于"该发心跳了"：空闲重写照样把 waitNewer
     * 叫醒，于是一条静止的流可以整秒不吐一个字节 —— 而对端的读超时只有几秒。所以心跳另按
     * 墙上时间（last_out_ms）算，见 provider。
     */
    int64_t sent_frames = -1;
    std::string sent_status;
};

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
    // 分阶段耗时：compute_ms 是聚合数，任何性能改动在它上面都无法归因（第一轮就把收益算在了
    // 单路径成本上，而实测收益来自"掉进稀疏回退的帧占比"）。四段之和 < compute 是正常的
    // （中间还有守卫、平滑与快照拷贝）。sparse 只在该帧真跑了特征互验时非零。
    j["stage"] = {{"extract_ms", fs.extract_ms},   // 特征前端（ORB/SIFT 提取）
                  {"cloud_ms", fs.cloud_ms},       // 深度→点云（随点数走）
                  {"sparse_ms", fs.sparse_ms},     // 帧间特征互验（六自由度实测运动）
                  {"dense_ms", fs.dense_ms}};      // GICP 稠密配准（配合 iterations 看迭代成本）
    j["fps"] = fs.fps;
    j["frames"] = fs.frames;
    j["dropped"] = fs.dropped;
    j["rejected"] = fs.rejected;
    j["smooth_used"] = fs.smooth_used;
    j["gyro_still"] = fs.gyro_still;     // 陀螺的静止判据（P1）。注意它只是"该冻"，不等于"在冻"
    j["rot_frozen"] = fs.rot_frozen;     // 旋转通道此刻真的被冻住（时限兜底或暂停期会让它 ≠ 上一条）
    j["frozen_ms"] = fs.frozen_ms;       // 本次冻结已持续多久
    j["rot_gated"] = fs.rot_gated;       // 离群门累计拦截计数（P3）
    j["rot_gate_err_deg"] = fs.rot_gate_err_deg;  // 本帧"视觉旋转 vs 陀螺旋转"的测地线偏差：调门限看它
    j["rot_gate_limit_deg"] = fs.rot_gate_limit_deg;  // 本帧实际生效的动态门限；门随转角走，err 单独看无意义
    // 陀螺链路自检，单独一组：这几个字段是给"功能没生效"和"生效了但场景不对"分诊用的，
    // 页面正常路径不读它们（gyro_still=false 到底是"相机在动"还是"根本没样本"，就看这里）。
    j["gyro"] = {{"time_ready", fs.gyro_time_ready},   // 设备→主机时间基是否已定标
                 {"extrinsics_loaded", fs.gyro_extrinsics_loaded},  // T_cam_gyro 是否读到合法非 Identity 旋转
                 {"buf", fs.gyro_buf},                 // 跟踪器缓冲长度
                 {"samples", fs.gyro_frame_samples},   // 本帧积分窗口真正用到的样本数
                 {"pushed", fs.gyro_frame_pushed},     // 本帧交付到达的样本数（与时间戳无关）
                 // 覆盖率 S/P：实测结构值 ~0.58（IMU 交付比帧交付慢半帧，尾部样本解算时还没到货），
                 // 所以它低于 1 不是故障，明显低于 0.58 才是真在丢样本。后两个是指认数：三条时间轴
                 // 各比主机快多少，真机 1.0005 / 0.9992 ⇒ 戳都在该在的轴上。
                 {"coverage", fs.gyro_cov},
                 {"frame_dev_ratio", fs.gyro_frame_dev_ratio},
                 {"gyro_stamp_ratio", fs.gyro_stamp_ratio},
                 // 覆盖账：span_ms 含常值补积 ⇒ ≈帧周期才说明旋转没少算；extrap_ms 是其中"预测"
                 // 而非"测量"的时长；gap_end_ms 只剩没补上的残缺口（IMU 真停更才非零）。
                 {"span_ms", fs.gyro_span_ms},
                 {"extrap_ms", fs.gyro_extrap_ms},
                 {"gap_end_ms", fs.gyro_gap_end_ms},
                 {"dead_frames", fs.gyro_dead_frames}, // 连续"有缓冲却积不到样本"的帧数
                 // 断供与域错配是两件事，必须各有一个字段：alive=false 持续 ⇒ 静止冻结/离群门/
                 // 帧间初值三条一起失效（现象只是"放下相机后旋转还在晃"），而 dead_frames 只抓
                 // 得到"缓冲里有样本却框不到"那一种。
                 {"alive", fs.gyro_alive},             // 本帧是否真有陀螺样本到货
                 {"starved_frames", fs.gyro_starved_frames},  // 连续零样本交付的帧数（≥30 已报 ERROR）
                 {"callbacks", fs.gyro_callbacks},  // 回调累计数：两次快照的差/时间差≈200/s 才说明
                                                    // 传感器在出帧；不涨=设备侧死，在涨而 pushed=0=时间基丢的
                 {"bias_dps", fs.gyro_bias_dps},       // 零偏估计（度/秒）
                 {"resid_dps", fs.gyro_resid_dps},     // 残差模均值（度/秒），静止门限比的是它
                 {"bias_ready", fs.gyro_bias_ready}};  // false ⇒ 静止结论还不可信

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
            {"gyro_extrinsics_loaded", st.gyro_extrinsics_loaded},
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

    // 13. GET /api/v1/camera/follow/stream  (SSE，数据面)
    //     位姿是"推"下来的，不再要调用方按周期来问。**控制面仍然是普通的 HTTP 请求/响应**
    //     （开/关/示教都要一个明确的成败回执），这里搬走的只有实时位姿这一条流。
    //
    //     载荷 = followSnapshotJson()，和 /follow/status 的 data 字段逐字节同一份序列化。
    //     两条路径共用一个函数不是图省事：否则"轮询看到的"和"推送看到的"迟早会漂移成两套字段，
    //     而那种漂移都是靠前端"咦这个字段怎么没值"发现的，代价很高。
    //
    //     为什么是 SSE 而不是 WebSocket：本进程里的 httplib 0.18.3 没有服务端 WS（只有客户端），
    //     而数据面是单向的 —— 双向能力在这里换不来任何东西，却要引入一次协议升级、一套帧编解码
    //     和一处新的心跳/半开连接问题。SSE 就是"一个永不结束的 GET"，浏览器和 requests 都原生认。
    //
    //     重连**不补历史**：这是 latest-value 流，不是事件日志。跟随闭环只关心当前位姿，把中间
    //     帧排队重放只会把网络抖动变成臂的抖动。要判断"我们是否真的跟上了流"，看载荷里的
    //     frames（真正解算过的帧数，单调）而不是这里的 seq。
    server_->Get("/api/v1/camera/follow/stream",
                 [this, follow_unavailable](const httplib::Request& req, httplib::Response& res) {
        if (!follow_) { follow_unavailable(res, "GET /api/v1/camera/follow/stream"); return; }
        PoseBroker& broker = follow_->poseBroker();

        // 先抢名额再挂 provider：满了就回一个明确的 503。让客户端"连上了但一帧都没有"是这里
        // 最坏的失败方式 —— 它和"服务挂了""网络断了"在页面上长得一模一样。
        auto slot = std::make_shared<StreamSlot>(broker);
        if (!slot->ok()) {
            json j;
            j["code"] = -1;
            j["msg"] = "follow stream 订阅已满（上限 " + std::to_string(PoseBroker::kMaxSubscribers) +
                       "）：先关掉不再使用的订阅者";
            res.status = 503;
            res.set_content(j.dump(), "application/json");
            LOG_WARN("HTTP", "GET /api/v1/camera/follow/stream -> 503（订阅已满）");
            return;
        }

        const std::string last_id = req.get_header_value("Last-Event-ID");
        auto st = std::make_shared<StreamState>();
        st->seen = broker.revision();
        res.set_header("Cache-Control", "no-cache");
        // 中间有任何反代（nginx 默认会缓冲）都会把小帧攒成一大段，推送延迟直接变成"看不见"。
        res.set_header("X-Accel-Buffering", "no");
        LOG_INFO("HTTP", "follow stream 订阅接入（当前 ", broker.subscribers(), " 路，last-event-id=",
                 last_id.empty() ? "-" : last_id, "）");

        // provider 是在 handler **返回之后**才被反复调用的，所以闭包里只能放自己有生命周期的东西：
        // 这里抓 follow_ 的副本（shared_ptr，把 worker 的命脉握在自己手里）而不是抓栈上引用。
        auto follow = follow_;
        res.set_chunked_content_provider(
            "text/event-stream",
            [follow, slot, st](size_t /*offset*/, httplib::DataSink& sink) -> bool {
                PoseBroker& broker = follow->poseBroker();
                // 每次被调用只做"一次唤醒 + 一次写"就返回。**绝不在 provider 里自己循环**：
                // httplib 在每次调用 provider 之前检查 strm.is_writable()、之后检查是否正在停机，
                // 所以"及时返回"就是对端断开和服务停机唯一的检测点。
                if (broker.stopping()) { sink.done(); return true; }
                const bool newer = broker.waitNewer(st->seen, PoseBroker::kHeartbeatMs);
                if (newer) st->seen = broker.revision();   // 唤醒要用掉，哪怕这次什么都不发
                json j = HttpServer::followSnapshotJson(follow->snapshot());

                // 内容变了才发事件。**缺键即算变了**：序列化器哪天不再发 frames/status，
                // 宁可退回"每 30ms 一帧重复"也别静默丢掉真正的新数据。
                const bool changed =
                    st->first || !j.contains("frames") || !j.contains("status") ||
                    j["frames"].get<int64_t>() != st->sent_frames ||
                    j["status"].get<std::string>() != st->sent_status;
                // !newer 就是 waitNewer 自己超时了 ⇒ 该发心跳；newer 但内容没变时按墙上时间补
                // （空闲重写会不停叫醒我们，光靠"超时"永远轮不到心跳）。
                const bool ping_due = !newer || nowMs() - st->last_out_ms >= PoseBroker::kHeartbeatMs;
                if (!changed && !ping_due) return true;

                std::string head, body;
                if (changed) {
                    if (j.contains("frames")) st->sent_frames = j["frames"].get<int64_t>();
                    if (j.contains("status")) st->sent_status = j["status"].get<std::string>();
                    j["seq"] = st->seen;   // 全局快照写号：**不是**帧号，也不能当丢帧判据（见上）
                    head = "id: " + std::to_string(st->seen) + "\n";
                    body = "data: " + j.dump() + "\n\n";
                    st->first = false;
                } else {
                    // 心跳：没有新内容时也要写字节。写失败 ⇒ 对端已经不在了，立刻结束这一路，
                    // 名额随即回收（这是残留连接唯一会被发现的时刻）。
                    static const char kPing[] = ": ping\n\n";
                    body.assign(kPing, sizeof(kPing) - 1);
                }
                st->last_out_ms = nowMs();
                if (!head.empty() && !sink.write(head.data(), head.size())) return false;
                return sink.write(body.data(), body.size());
            },
            // chunked body 的生命周期比 handler 的栈帧长（provider 是在 handler 返回之后才被
            // 反复调用的），所以名额不能靠 handler 里的栈对象归还。两处都会调：正常结束走
            // releaser，写失败走 provider 返回 false —— slot 内部幂等，谁先到谁还。
            [slot](bool /*success*/) { slot->release(); });
    });
}

} // namespace orbbec_service
