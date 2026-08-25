#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include "types.hpp"
#include "camera_driver.hpp"
#include "corner_detector.hpp"
#include "zlm_streamer.hpp"
#include "async_disk_writer.hpp"
#include "logger.hpp"

namespace httplib {
    class Server;
}

namespace orbbec_service {

class HttpServer {
public:
    HttpServer(
        std::shared_ptr<CameraDriver> camera,
        std::shared_ptr<CornerDetector> corner_detector,
        std::shared_ptr<ZlmStreamer> streamer,
        std::shared_ptr<AsyncDiskWriter> disk_writer
    );
    ~HttpServer();

    bool start(int port = 18080);
    void stop();

private:
    void setupRoutes();

private:
    std::shared_ptr<CameraDriver> camera_;
    std::shared_ptr<CornerDetector> corner_detector_;
    std::shared_ptr<ZlmStreamer> streamer_;
    std::shared_ptr<AsyncDiskWriter> disk_writer_;

    std::unique_ptr<httplib::Server> server_;
    std::thread server_thread_;
    std::atomic<bool> running_{false};
    int port_ = 18080;
};

} // namespace orbbec_service
