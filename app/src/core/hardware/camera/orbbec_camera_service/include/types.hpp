#pragma once

#include <string>
#include <vector>
#include <memory>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>

namespace orbbec_service {

struct FrameData {
    uint64_t frame_index = 0;
    uint64_t timestamp_ms = 0;   // 主机到达时刻（日志 / 存盘 / 帧率）
    uint64_t device_ts_us = 0;   // 设备开机 µs：深度优先，否则彩色；0 = 本帧没带设备戳
    // 与陀螺同一时间域：GyroTimeBase::toHostNs(device_ts_us)。未定标为 0，follow 必须回退
    // 到 timestamp_ms——那时陀螺样本也会被丢掉，两条路径一起空转，不会各走各的钟。
    int64_t track_ts_ns = 0;
    cv::Mat color;        // BGR888 or RGB888, CV_8UC3
    cv::Mat depth;        // Z16, 16-bit depth in mm, CV_16UC1
    bool has_color = false;
    bool has_depth = false;
};

struct CameraIntrinsics {
    std::string camera_model = "orbbec";
    int width = 1280;
    int height = 800;
    std::vector<std::vector<double>> intrinsic_matrix = {
        {611.68, 0.0, 643.42},
        {0.0, 611.69, 405.15},
        {0.0, 0.0, 1.0}
    };
    std::vector<double> distortion_coeffs = {-0.032, 0.034, 0.0003, 0.0003, -0.011};
    std::string distortion_model = "plumb_bob";
    double depth_scale = 1.0; // 1.0 = mm
};

struct CalibrationConfig {
    bool enabled = false;
    std::string board_type = "chessboard";
    int rows = 12;            // Grid total rows
    int cols = 9;             // Grid total cols
    double square_size_mm = 15.0;
    bool draw_corners = true;

    // Inner pattern size (cols - 1, rows - 1)
    cv::Size getPatternSize() const {
        return cv::Size(std::max(1, cols - 1), std::max(1, rows - 1));
    }
};

struct DetectedCorners {
    bool found = false;
    uint64_t timestamp_ms = 0;
    int pattern_cols = 8;
    int pattern_rows = 11;
    std::vector<cv::Point2f> corners;
    double detection_time_ms = 0.0;
};

struct CameraStatus {
    bool online = false;
    bool streaming = false;
    std::string camera_model = "Orbbec Gemini";
    std::string serial_number = "Unknown";
    std::string firmware_version = "Unknown";
    double color_fps = 0.0;
    double depth_fps = 0.0;
    std::string encoder = "RK_MPP_H264";
    double temperature_c = 0.0;
    bool calibration_mode = false;
    bool depth_stream_enabled = true;
    bool depth_align_enabled = true;
    // 交付分辨率与配置分辨率**不是一回事**：follow 使能时取流档位会被切到 follow.camera.*。
    // 内参是按哪一档取的必须由状态说清楚 —— 用 1280x800 的 fx 去反投影 640x480 的网格，
    // 产出的点云自洽但全错，而且下游看不出任何异常。
    int capture_width = 0;
    int capture_height = 0;
    int capture_fps = 0;
    bool follow_profile = false;
    // 实际起来的对齐方式："hw" / "sw" / "disabled"。"disabled" 时深度帧**不在**彩色像素坐标系里，
    // 拿彩色内参反投影会得出自洽但全错的点云 —— 所以它必须是可查的状态，而不是只体现在日志里。
    std::string depth_align_mode = "disabled";
    // 内参是设备给的还是编译期默认值。默认值只在 follow 的判据里等于"没有内参"。
    bool intrinsics_loaded = false;
    // 陀螺外参是否从设备读到非 Identity 值。false = 回退到 Identity，零偏补偿残差会因相机姿态变化。
    bool gyro_extrinsics_loaded = false;
    uint64_t total_frames = 0;
    uint64_t dropped_frames = 0;
};

struct SaveFrameRequest {
    std::string save_dir = "data/calib";
    std::string prefix = "sample_001";
    std::string color_filename;             // If specified (e.g. "image_001.png" or "scan.jpg"), overrides default naming
    std::string depth_filename;             // If specified (e.g. "scan.depth.png"), overrides default naming
    bool save_color = true;
    bool save_depth = true;
    bool save_info_yaml = true;
    std::string color_format = "png";       // "png" or "jpg"
    std::string depth_format = "png_16bit"; // "png_16bit", "raw"
    bool save_pointcloud = false;
    nlohmann::json metadata = nlohmann::json::object();
};

struct SaveFrameResult {
    bool success = false;
    uint64_t frame_id = 0;
    uint64_t timestamp_ms = 0;
    std::string color_file;
    std::string depth_file;
    std::string info_file;
    bool corners_found = false;
    std::string error_msg;
};

struct StreamInfo {
    std::string stream_id = "orbbec_color";
    std::string webrtc_url;
    std::string rtsp_url;
    std::string http_flv_url;
    int width = 1280;
    int height = 800;
    int fps = 30;
};

struct AppConfig {
    std::string project_root = "";
    std::string data_root = "data";
    
    // Camera params
    int camera_width = 1280;
    int camera_height = 800;
    int camera_fps = 30;
    bool enable_depth_align = true;
    // follow.camera.enable_imu：Follow 档位下要不要起板载陀螺流。默认开，与 CaptureParams
    // 一致。关了就整条 IMU 链路（初值 / 离群门 / 静止冻结 / 示教静止门）一起空转，必须由
    // CameraDriver 认这个键，不能只让独立工具 follow_pose 认。
    bool enable_imu = true;
    
    // Video streaming params
    int stream_width = 1280;
    int stream_height = 800;
    int stream_fps = 30;
    int stream_bitrate_kbps = 2500;
    int rtsp_port = 554;
    int rtmp_port = 1935;
    int http_port = 18080;
    int zlm_http_port = 8000;
    int stats_interval_sec = 10;
    std::string stream_app = "live";
    std::string stream_id = "orbbec_color";

    // Calibration default
    CalibrationConfig calib_default;

    // 设备仲裁锁（flock 文件，例 /home/.../AiSprayer/.orbbec.lock）。follow 那三个独立工具
    // （follow_node / follow_pose / follow_capture_selftest）本来就抢这把锁，本进程过去不拿 ——
    // 于是"谁先 open 设备"由 libusb 决定，现场表现为两路都装作在跑。现在双方都拿，冲突变成
    // 一句能读的"被 PID x 占用"。值由 main.cpp 从 follow.camera.lock_path 灌进来（全项目只允许
    // 一处解析那个键），已经解析成绝对路径。空 = 不做进程间仲裁（回放/单测）。
    std::string device_lock_path = "";
};

} // namespace orbbec_service
