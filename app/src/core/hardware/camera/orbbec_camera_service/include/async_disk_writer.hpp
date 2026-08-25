#pragma once

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <future>
#include "types.hpp"
#include "logger.hpp"

namespace orbbec_service {

struct SaveTask {
    SaveFrameRequest request;
    FrameData frame;
    CameraIntrinsics intrinsics;
    DetectedCorners corners;
    std::shared_ptr<std::promise<SaveFrameResult>> promise;
};

class AsyncDiskWriter {
public:
    AsyncDiskWriter();
    ~AsyncDiskWriter();

    bool init(const AppConfig& config, int worker_threads = 2);
    void stop();

    // Async save frame to disk
    std::future<SaveFrameResult> saveFrameAsync(
        const SaveFrameRequest& req, 
        const FrameData& frame, 
        const CameraIntrinsics& intrinsics, 
        const DetectedCorners& corners
    );

    // Synchronous helper
    SaveFrameResult saveFrameSync(
        const SaveFrameRequest& req, 
        const FrameData& frame, 
        const CameraIntrinsics& intrinsics, 
        const DetectedCorners& corners
    );

private:
    void workerLoop(int worker_id);
    SaveFrameResult processTask(const SaveTask& task);
    std::string resolveSavePath(const std::string& input_dir);

private:
    AppConfig config_;
    std::atomic<bool> running_{false};
    std::queue<SaveTask> task_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::vector<std::thread> workers_;
};

} // namespace orbbec_service
