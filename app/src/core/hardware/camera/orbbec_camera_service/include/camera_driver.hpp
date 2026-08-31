#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "types.hpp"
#include "logger.hpp"
#include "gyro_time_base.hpp"
#include "follow/device_lock.hpp"
#include "follow/types.hpp"

// Forward declaration of Orbbec SDK classes
namespace ob {
    class Context;
    class Pipeline;
    class Device;
    class Config;
    class Sensor;
}

namespace orbbec_service {

class CameraDriver {
public:
    CameraDriver();
    ~CameraDriver();

    bool init(const AppConfig& config);
    bool start();
    void stop();

    bool isRunning() const { return running_; }
    bool isConnected() const { return connected_; }

    // Fetch the latest aligned frame data (non-blocking)
    bool getLatestFrame(FrameData& out_frame);

    // Event-driven frame waiting: blocks until next camera hardware frame arrives
    bool waitForNextFrame(FrameData& out_frame, int timeout_ms = 100);

    // Dynamically switch calibration mode:
    // If enabled == true: disables depth stream and depth alignment (0% depth CPU load)
    // If enabled == false: enables depth stream and depth alignment
    bool setCalibrationMode(bool enabled);
    bool isCalibrationMode() const { return calib_mode_; }

    // 工位跟随 (follow) 使能时把取流切到指定档位（实测 640x480@15：Gemini 336L 的硬件 D2C 只到
    // 640x480，848x480 起只能软对齐，1280x800 实测要吃 29~35 ms 主机 CPU）。enabled=false 时
    // 恢复 hardware.camera 里配的档位。套路与 setCalibrationMode 一致：置标志 → pipe_->stop()
    // → configureAndStartPipeline()。两条约束写在这里，别留给调用方猜：
    //   * 与标定模式互斥 —— 标定模式关掉了深度流，没有深度 follow 就跑不了；
    //   * 换档后必须重新取内参（refreshIntrinsics），否则拿旧分辨率的 fx 去反投影新网格。
    bool setFollowProfile(bool enabled, int width, int height, int fps,
                          std::string* err = nullptr);
    bool isFollowProfile() const { return follow_mode_; }

    // Get current status & intrinsics
    CameraStatus getStatus();
    CameraIntrinsics getIntrinsics();

    // 陀螺仪数据消费接口（由 FollowWorker 消费）
    bool drainGyroSamples(std::vector<follow::GyroSample>* out);
    bool hasImu() const { return has_imu_; }

    // 陀螺时间基（见 gyro_time_base.hpp）。未就绪时 drainGyroSamples 必然交不出样本，
    // 而"IMU 在跑"和"样本能用于积分"是两件事 —— 只报 hasImu() 会把前者当成后者，
    // 于是域不同这种故障在外部看起来就是"陀螺一直在动"，所以这几个数必须能读出来。
    bool gyroTimeBaseReady() const { return gyro_time_base_.ready(); }
    int64_t gyroTimeOffsetNs() const { return gyro_time_base_.offset_ns(); }
    int64_t gyroTimeSpreadNs() const { return gyro_time_base_.spread_ns(); }
    uint64_t gyroDroppedBeforeReady() const { return gyro_time_base_.dropped_before_ready(); }

private:
    void captureLoop();
    bool tryConnectDevice();
    bool configureAndStartPipeline();
    void stopPipelineAndSensors();
    // 从**当前已启动的** pipeline 取 rgb 内参/畸变，并刷新 intrinsics_ 与 status_.capture_*。
    // 以前这段只在 tryConnectDevice 里做一次，而模式切换是原地重启 pipeline、不重连设备 ——
    // 于是分辨率换了、内参没换，这是会产出"自洽但全错"点云的那种 bug。
    bool refreshIntrinsics();
    void resetHardwareConnection();
    // 出流停滞时的第一档自救：只重启 pipeline，不拆设备、不重新枚举 USB。
    // 硬重连会把设备从总线上拽下来重新枚举，重枚举后帧率经常掉到 3~4 fps，
    // 反而把一次短暂停滞放大成长时间故障 —— 所以先试软的，失败才升级。
    bool trySoftPipelineRestart();
    void updateFpsStats();

private:
    AppConfig config_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> calib_mode_{false};
    int consecutive_timeouts_ = 0;
    // 分级恢复账本：软重启连续失败几次后才升级到硬重连；以及恢复后连续正常帧数（够多才算稳住，清零账本）。
    int soft_restart_attempts_ = 0;
    int frames_since_recovery_ = 0;
    uint64_t last_frameset_ms_ = 0;   // 上一次拿到 frameset 的时刻（诊断用：停滞了多久）

    // follow 取流档位（follow_mode_ 是原子，其余三个只在持 pipe_mutex_ 时读写）
    std::atomic<bool> follow_mode_{false};
    int follow_width_ = 0;
    int follow_height_ = 0;
    int follow_fps_ = 0;

    // 板载 IMU 陀螺仪
    std::shared_ptr<ob::Sensor> gyro_sensor_;
    std::deque<follow::GyroSample> gyro_queue_;
    std::mutex gyro_mtx_;
    Eigen::Matrix3d R_cam_gyro_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_cam_gyro_ = Eigen::Vector3d::Zero();
    bool has_imu_ = false;
    // 陀螺样本进队列前一律经 GyroTimeBase 换到与 FrameData::track_ts_ns 同一域。
    // 忘了这一步的后果不是崩溃而是静默：follow 的积分窗口框不到任何样本。
    GyroTimeBase gyro_time_base_;

    // 独占设备的进程间仲裁。**必须在 make_unique<ob::Context>() 之前拿到**：拿不到就不碰 SDK，
    // 交给现有的 1500 ms 重连循环去等，而不是两路 pipeline 抢同一颗 USB 设备。
    // 锁由内核持有，进程被 SIGKILL 也自动释放 —— 所以不存在"清残留锁文件"那种代码（那会删掉
    // 别人正持有的锁）。它也挡不住同进程内开两路 pipeline，那只能靠代码评审。
    follow::DeviceLock dev_lock_;
    // 取锁失败/未配锁路径时只说一次：tryConnectDevice 被重连循环每 1500ms 调一次，
    // 每次一行会把日志刷满而信息量还是那一条。
    bool lock_notice_logged_ = false;

    std::unique_ptr<ob::Context> ctx_;
    std::unique_ptr<ob::Pipeline> pipe_;
    std::shared_ptr<ob::Device> device_;

    std::thread capture_thread_;
    std::mutex pipe_mutex_;
    // frame_mutex_ **只**保护帧数据这条链：latest_frame_ / frame_cv_ / last_consumed_frame_id_。
    // 状态量拆到 status_mutex_（见下）：以前两者共用一把锁，任何一次取流侧的长持有都会把
    // getStatus()/getIntrinsics() 一起拖住（实测档位切换后一次 getStatus 被堵 9.5s），
    // follow 使能因此整体超过 HTTP 客户端的 20s 超时。
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    FrameData latest_frame_;
    uint64_t last_consumed_frame_id_ = 0;

    // status_/intrinsics_ 专用锁：写方全是"改几个字段"的短临界区，读方（HTTP/worker 轮询）
    // 拿到就能走，绝不为帧生产排队。
    mutable std::mutex status_mutex_;
    CameraStatus status_;
    CameraIntrinsics intrinsics_;

    uint64_t frame_count_ = 0;
    uint64_t last_stat_time_ms_ = 0;
    uint64_t last_stat_frames_ = 0;
};

} // namespace orbbec_service
