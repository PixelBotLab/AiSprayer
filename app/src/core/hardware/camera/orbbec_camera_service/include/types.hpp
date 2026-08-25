#pragma once

#include <string>
#include <vector>
#include <memory>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>

namespace orbbec_service {

struct FrameData {
    uint64_t frame_index = 0;
    uint64_t timestamp_ms = 0;
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
};

} // namespace orbbec_service
