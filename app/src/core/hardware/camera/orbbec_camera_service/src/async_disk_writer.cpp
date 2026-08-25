#include "async_disk_writer.hpp"
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>

namespace orbbec_service {

AsyncDiskWriter::AsyncDiskWriter() {}

AsyncDiskWriter::~AsyncDiskWriter() {
    stop();
}

bool AsyncDiskWriter::init(const AppConfig& config, int worker_threads) {
    config_ = config;
    running_ = true;

    for (int i = 0; i < worker_threads; ++i) {
        workers_.emplace_back(&AsyncDiskWriter::workerLoop, this, i);
    }

    LOG_INFO("DiskWriter", "Async Disk Persistence Engine initialized with ", 
             worker_threads, " background worker threads.");
    return true;
}

void AsyncDiskWriter::stop() {
    if (!running_) return;

    running_ = false;
    queue_cv_.notify_all();

    for (auto& w : workers_) {
        if (w.joinable()) {
            w.join();
        }
    }
    workers_.clear();

    LOG_INFO("DiskWriter", "Async Disk Persistence Engine stopped.");
}

std::string AsyncDiskWriter::resolveSavePath(const std::string& input_dir) {
    std::filesystem::path p(input_dir);
    if (p.is_absolute()) {
        return p.string();
    }

    // Relative to project root
    std::filesystem::path root(config_.project_root);
    if (input_dir.find("data/") == 0) {
        return (root / input_dir).string();
    }
    return (root / "data" / input_dir).string();
}

std::future<SaveFrameResult> AsyncDiskWriter::saveFrameAsync(
    const SaveFrameRequest& req, 
    const FrameData& frame, 
    const CameraIntrinsics& intrinsics, 
    const DetectedCorners& corners) {

    auto promise = std::make_shared<std::promise<SaveFrameResult>>();
    auto future = promise->get_future();

    SaveTask task;
    task.request = req;
    task.frame = frame;
    task.intrinsics = intrinsics;
    task.corners = corners;
    task.promise = promise;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.push(task);
    }
    queue_cv_.notify_one();

    return future;
}

SaveFrameResult AsyncDiskWriter::saveFrameSync(
    const SaveFrameRequest& req, 
    const FrameData& frame, 
    const CameraIntrinsics& intrinsics, 
    const DetectedCorners& corners) {

    SaveTask task;
    task.request = req;
    task.frame = frame;
    task.intrinsics = intrinsics;
    task.corners = corners;
    return processTask(task);
}

void AsyncDiskWriter::workerLoop(int worker_id) {
    LOG_INFO("DiskWriter", "Disk writer worker thread #", worker_id, " started.");

    while (running_) {
        SaveTask task;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !running_ || !task_queue_.empty();
            });

            if (!running_ && task_queue_.empty()) {
                break;
            }

            task = task_queue_.front();
            task_queue_.pop();
        }

        SaveFrameResult res = processTask(task);
        if (task.promise) {
            task.promise->set_value(res);
        }
    }

    LOG_INFO("DiskWriter", "Disk writer worker thread #", worker_id, " exited.");
}

SaveFrameResult AsyncDiskWriter::processTask(const SaveTask& task) {
    auto t_start = std::chrono::high_resolution_clock::now();
    SaveFrameResult result;
    result.frame_id = task.frame.frame_index;
    result.timestamp_ms = task.frame.timestamp_ms;
    result.corners_found = task.corners.found;

    std::string full_dir = resolveSavePath(task.request.save_dir);
    try {
        std::filesystem::create_directories(full_dir);
    } catch (const std::exception& e) {
        result.success = false;
        result.error_msg = std::string("Failed to create directory: ") + e.what();
        LOG_ERROR("DiskWriter", result.error_msg);
        return result;
    }

    std::filesystem::path dir_path(full_dir);
    std::string prefix = task.request.prefix.empty() ? "sample_001" : task.request.prefix;

    // 1. Save Color Image
    if (task.request.save_color && task.frame.has_color && !task.frame.color.empty()) {
        std::string ext = (task.request.color_format == "jpg" || task.request.color_format == "jpeg") ? ".jpg" : ".png";
        std::filesystem::path color_path;
        if (!task.request.color_filename.empty()) {
            color_path = dir_path / task.request.color_filename;
        } else {
            color_path = dir_path / (prefix + "_color" + ext);
        }

        std::vector<int> compression_params;
        if (color_path.extension() == ".png") {
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(3); // Fast compression
        } else {
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(95);
        }

        bool ok = cv::imwrite(color_path.string(), task.frame.color, compression_params);
        if (ok) {
            result.color_file = std::filesystem::relative(color_path, config_.project_root).string();
            LOG_INFO("DiskWriter", "Saved Color frame to: ", result.color_file);
        } else {
            LOG_ERROR("DiskWriter", "Failed to write color image to: ", color_path.string());
        }
    }

    // 2. Save Depth Image (16-bit)
    if (task.request.save_depth && task.frame.has_depth && !task.frame.depth.empty()) {
        std::filesystem::path depth_path;
        if (!task.request.depth_filename.empty()) {
            depth_path = dir_path / task.request.depth_filename;
        } else {
            depth_path = dir_path / (prefix + (task.request.depth_format == "raw" ? "_depth.raw" : "_depth.png"));
        }

        if (depth_path.extension() == ".raw") {
            std::ofstream ofs(depth_path.string(), std::ios::binary);
            if (ofs.is_open()) {
                ofs.write((const char*)task.frame.depth.data, task.frame.depth.total() * task.frame.depth.elemSize());
                ofs.close();
                result.depth_file = std::filesystem::relative(depth_path, config_.project_root).string();
                LOG_INFO("DiskWriter", "Saved Depth raw binary to: ", result.depth_file);
            }
        } else {
            // Default: 16-bit PNG (Z16 lossless)
            std::vector<int> depth_params = {cv::IMWRITE_PNG_COMPRESSION, 3};
            bool ok = cv::imwrite(depth_path.string(), task.frame.depth, depth_params);
            if (ok) {
                result.depth_file = std::filesystem::relative(depth_path, config_.project_root).string();
                LOG_INFO("DiskWriter", "Saved Depth (16-bit lossless) to: ", result.depth_file);
            } else {
                LOG_ERROR("DiskWriter", "Failed to write 16-bit depth PNG to: ", depth_path.string());
            }
        }
    }

    // 3. Save Metadata Info YAML
    if (task.request.save_info_yaml) {
        try {
            std::filesystem::path info_path = dir_path / (prefix + "_info.yaml");
            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "frame_id" << YAML::Value << task.frame.frame_index;
            out << YAML::Key << "timestamp_ms" << YAML::Value << task.frame.timestamp_ms;
            out << YAML::Key << "camera_model" << YAML::Value << task.intrinsics.camera_model;
        out << YAML::Key << "image_width" << YAML::Value << task.intrinsics.width;
        out << YAML::Key << "image_height" << YAML::Value << task.intrinsics.height;
        out << YAML::Key << "depth_scale_mm" << YAML::Value << task.intrinsics.depth_scale;

        // Intrinsics
        out << YAML::Key << "intrinsic_matrix" << YAML::Value << YAML::BeginSeq;
        for (const auto& row : task.intrinsics.intrinsic_matrix) {
            out << YAML::Flow << YAML::BeginSeq;
            for (double val : row) out << val;
            out << YAML::EndSeq;
        }
        out << YAML::EndSeq;

        // Distortion
        out << YAML::Key << "distortion_coeffs" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (double d : task.intrinsics.distortion_coeffs) out << d;
        out << YAML::EndSeq;

        // Corners
        out << YAML::Key << "corners_found" << YAML::Value << task.corners.found;
        if (task.corners.found) {
            out << YAML::Key << "corners_count" << YAML::Value << (int)task.corners.corners.size();
            out << YAML::Key << "pattern_cols" << YAML::Value << task.corners.pattern_cols;
            out << YAML::Key << "pattern_rows" << YAML::Value << task.corners.pattern_rows;
            out << YAML::Key << "corners_2d" << YAML::Value << YAML::BeginSeq;
            for (const auto& pt : task.corners.corners) {
                out << YAML::Flow << YAML::BeginSeq << pt.x << pt.y << YAML::EndSeq;
            }
            out << YAML::EndSeq;
        }

        // Custom metadata (e.g., robot_pose)
        if (!task.request.metadata.empty()) {
            out << YAML::Key << "metadata" << YAML::Value;
            YAML::Node meta_node = YAML::Load(task.request.metadata.dump());
            out << meta_node;
        }

        out << YAML::EndMap;

        std::ofstream ofs(info_path.string());
        if (ofs.is_open()) {
            ofs << out.c_str() << std::endl;
            ofs.close();
            result.info_file = std::filesystem::relative(info_path, config_.project_root).string();
            LOG_INFO("DiskWriter", "Saved Frame metadata YAML to: ", result.info_file);
        }
        } catch (const std::exception& e) {
            LOG_WARN("DiskWriter", "Exception writing info YAML: ", e.what());
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    result.success = true;
    LOG_INFO("DiskWriter", "Frame [", result.frame_id, "] persisted successfully in ", duration_ms, " ms");
    return result;
}

} // namespace orbbec_service
