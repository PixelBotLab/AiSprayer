#pragma once

#include "types.hpp"
#include "logger.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace orbbec_service {

class ConfigLoader {
public:
    static AppConfig load(const std::string& config_path = "") {
        AppConfig cfg;
        
        // Locate project root by traversing upwards
        std::filesystem::path search_path = std::filesystem::current_path();
        for (int i = 0; i < 8; ++i) {
            if (std::filesystem::exists(search_path / "configs" / "aisprayer_config.yaml")) {
                cfg.project_root = search_path.string();
                break;
            }
            if (search_path == search_path.parent_path()) break;
            search_path = search_path.parent_path();
        }
        if (cfg.project_root.empty()) {
            cfg.project_root = std::filesystem::current_path().string();
        }

        // Default directly to the unified project configuration
        std::string yaml_file = config_path;
        if (yaml_file.empty()) {
            yaml_file = (std::filesystem::path(cfg.project_root) / "configs" / "aisprayer_config.yaml").string();
        }

        LOG_INFO("Config", "Loading unified configuration from: ", yaml_file);

        if (std::filesystem::exists(yaml_file)) {
            try {
                YAML::Node root = YAML::LoadFile(yaml_file);

                // 1. Camera hardware params: check hardware.camera or root.camera
                YAML::Node cam_node;
                if (root["hardware"] && root["hardware"]["camera"]) {
                    cam_node = root["hardware"]["camera"];
                } else if (root["camera"]) {
                    cam_node = root["camera"];
                }

                if (cam_node) {
                    if (cam_node["width"]) cfg.camera_width = cam_node["width"].as<int>();
                    if (cam_node["height"]) cfg.camera_height = cam_node["height"].as<int>();
                    if (cam_node["fps"]) cfg.camera_fps = cam_node["fps"].as<int>();
                    if (cam_node["enable_depth_align"]) cfg.enable_depth_align = cam_node["enable_depth_align"].as<bool>();

                    // Check if server is nested under hardware.camera
                    if (cam_node["server"]) {
                        auto s = cam_node["server"];
                        if (s["http_port"]) cfg.http_port = s["http_port"].as<int>();
                        if (s["zlm_http_port"]) cfg.zlm_http_port = s["zlm_http_port"].as<int>();
                        if (s["rtsp_port"]) cfg.rtsp_port = s["rtsp_port"].as<int>();
                        if (s["rtmp_port"]) cfg.rtmp_port = s["rtmp_port"].as<int>();
                        if (s["stats_interval_sec"]) cfg.stats_interval_sec = s["stats_interval_sec"].as<int>();
                        if (s["data_root"]) cfg.data_root = s["data_root"].as<std::string>();
                    }

                    // Check if streaming is nested under hardware.camera
                    if (cam_node["streaming"]) {
                        auto st = cam_node["streaming"];
                        if (st["width"]) cfg.stream_width = st["width"].as<int>();
                        if (st["height"]) cfg.stream_height = st["height"].as<int>();
                        if (st["fps"]) cfg.stream_fps = st["fps"].as<int>();
                        if (st["bitrate_kbps"]) cfg.stream_bitrate_kbps = st["bitrate_kbps"].as<int>();
                        if (st["app"]) cfg.stream_app = st["app"].as<std::string>();
                        if (st["stream_id"]) cfg.stream_id = st["stream_id"].as<std::string>();
                    }
                }

                // 2. Server params fallback (top-level server node)
                if (root["server"]) {
                    auto s = root["server"];
                    if (s["http_port"]) cfg.http_port = s["http_port"].as<int>();
                    if (s["zlm_http_port"]) cfg.zlm_http_port = s["zlm_http_port"].as<int>();
                    if (s["rtsp_port"]) cfg.rtsp_port = s["rtsp_port"].as<int>();
                    if (s["rtmp_port"]) cfg.rtmp_port = s["rtmp_port"].as<int>();
                    if (s["data_root"]) cfg.data_root = s["data_root"].as<std::string>();
                }

                // 3. Streaming params fallback (top-level streaming node)
                if (root["streaming"]) {
                    auto st = root["streaming"];
                    if (st["width"]) cfg.stream_width = st["width"].as<int>();
                    if (st["height"]) cfg.stream_height = st["height"].as<int>();
                    if (st["fps"]) cfg.stream_fps = st["fps"].as<int>();
                    if (st["bitrate_kbps"]) cfg.stream_bitrate_kbps = st["bitrate_kbps"].as<int>();
                    if (st["app"]) cfg.stream_app = st["app"].as<std::string>();
                    if (st["stream_id"]) cfg.stream_id = st["stream_id"].as<std::string>();
                }

                // 4. Calibration default params (top-level calib node)
                if (root["calib"] && root["calib"]["board"]) {
                    auto b = root["calib"]["board"];
                    if (b["rows"]) cfg.calib_default.rows = b["rows"].as<int>();
                    if (b["cols"]) cfg.calib_default.cols = b["cols"].as<int>();
                    if (b["square_size_mm"]) cfg.calib_default.square_size_mm = b["square_size_mm"].as<double>();
                }

                if (root["calib"] && root["calib"]["capture"] && root["calib"]["capture"]["output_dir"]) {
                    cfg.data_root = root["calib"]["capture"]["output_dir"].as<std::string>();
                }

                LOG_INFO("Config", "Unified config parsed: http_port=", cfg.http_port, 
                         ", zlm_http_port=", cfg.zlm_http_port,
                         ", rtsp_port=", cfg.rtsp_port,
                         ", camera_res=", cfg.camera_width, "x", cfg.camera_height,
                         ", stream_res=", cfg.stream_width, "x", cfg.stream_height,
                         ", calib_board=", cfg.calib_default.rows, "x", cfg.calib_default.cols);
            } catch (const std::exception& e) {
                LOG_WARN("Config", "Failed to parse unified YAML file (", e.what(), "), using default config.");
            }
        } else {
            LOG_WARN("Config", "Config file not found at: ", yaml_file, ", using builtin defaults.");
        }

        LOG_INFO("Config", "Project root: ", cfg.project_root);
        LOG_INFO("Config", "Data root: ", cfg.data_root);
        return cfg;
    }
};

} // namespace orbbec_service
