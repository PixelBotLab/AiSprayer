// follow 作为**库**跑在本进程里，这个类是它和相机服务之间的那道墙。
//
// 为什么不在这里再 new 一个 follow::OrbbecCapture：那会在同一台设备上开出第二个
// ob::Pipeline —— 而本进程已经 take 了 .orbbec.lock，正是为了不让第二路取流存在。设备只有
// 一条取流路径，所以 follow 消费 CameraDriver **已经在交付**的那一份对齐帧。代价写清楚，
// 别留给下一个读代码的人猜：
//   * 拿不到厂商 SDK 的设备时间戳。FrameData 上是主机毫秒时钟，粒度比 15 fps 的帧周期粗，
//     于是"同一毫秒内的两帧"无法区分 —— 处理成丢帧（见 loop()），不造假 +1ns 时间。
//   * 板载 IMU 陀螺仪由 CameraDriver 以 ~200Hz 独立流启动并读取硬件外参 T_cam_gyro，
//     每帧配准前由 drainGyroSamples 注入跟踪器：①帧间旋转初值 ②离群帧互验门（P3）
//     ③静止检测（P1：静止时旋转通道冻结）。示教期另做静止门（P2）。
//
// 线程模型：一条自己的工作线程 + 一份 latest 快照（HTTP 侧只读快照，绝不在请求线程里算 GICP）。
// 帧从 CameraDriver::getLatestFrame 取，**不能用 waitForNextFrame**：那个接口会推进驱动里的
// 共享游标 last_consumed_frame_id_，主循环和 worker 会互相把对方的帧吃掉，表现是"两边都掉帧
// 但谁也看不出谁在读"。所以这里自己按 frame_index 去重；算不过来时丢帧并计数（丢了不是错误，
// 悄悄算了旧帧才是错误）。
//
// 与 follow_pose 的一致性：示教走同一个 build_reference_map()（follow/teach_core.hpp），
// 平滑走同一个 PoseSmoother()，输出走同一个 to_dobot()。所以"follow_pose 里量对的数"才
// 蕴含"服务里推给页面的数是同一套算法算的"，而不是两份实现各自看着对。
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include "camera_driver.hpp"
#include "follow/config_loader.hpp"
#include "follow/frontend.hpp"
#include "follow/odometry.hpp"
#include "follow/pose_smoother.hpp"
#include "follow/reference_map.hpp"

namespace orbbec_service {

// 一次快照 = 一帧的完整读数。字段与 follow_pose 那一行打印一一对应（数值取自同一批计算），
// 这样页面上看到的数、日志里看到的数、和将来发给臂的数是同一个东西。
//
// 单位：pose_* 是 mm / deg（Dobot 约定：内禀 'xyz'，定系矩阵乘序 R = Rz(rz)·Ry(ry)·Rx(rx)），
// delta_t_m 是米、delta_r 是行主序 3x3。
// 跨界只在这里换单位，内部（follow）仍然 SI —— 和 pose_io.hpp 的规矩一致。
struct FollowSnapshot {
    bool enabled = false;
    bool connected = false;        // 相机在线
    bool taught = false;           // 有冻结参考地图
    bool has_pose = false;         // 至少解出过一个可信位姿
    // 词表 = follow::to_string(Status) 加上 worker 独有的三个：
    //   "disabled"（没使能）/ "no_map"（没示教）/ "no_frame"（没有新帧可算）
    std::string status = "disabled";
    std::string estimator = "none";
    std::string reason;            // 人可读的一行：门超限、尺寸不符、对齐没起来……
    bool switching = false;        // 档位切换进行中（重启取流 pipeline，实测最长几十秒）：
                                   // 此刻 enabled 还是旧值，调用方应继续轮询而不是当成终态

    // 显示位姿（N 帧平均后），= follow_pose 打印的那一行
    double pose_mm[3] = {0.0, 0.0, 0.0};
    double pose_rpy_deg[3] = {0.0, 0.0, 0.0};   // rx/ry/rz，R = Rz·Ry·Rx
    double norm_t_mm = 0.0;
    double norm_r_deg = 0.0;

    // 相对示教位的增量（示教系 ← 当前相机系）。app 侧把它乘到臂基准上，所以这里给的是矩阵
    // 而不是欧拉：欧拉上做乘法会错，矩阵不会。
    double delta_r[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double delta_t_m[3] = {0.0, 0.0, 0.0};

    double sigma_t_mm[3] = {0.0, 0.0, 0.0};
    double sigma_r_deg[3] = {0.0, 0.0, 0.0};
    int gicp_inliers = 0;
    double inlier_ratio = 0.0;
    double gicp_cost = 0.0;
    size_t cloud_points = 0;
    bool holding_last_pose = false;   // status != ok 时 pose_* 是上一个可信值，不是本帧读数

    double compute_ms = 0.0;
    double fps = 0.0;
    uint64_t frames = 0;              // 真正解算过的帧数
    uint64_t dropped = 0;             // 有新帧但没赶上（算不过来 / 时间戳同毫秒）
    uint64_t rejected = 0;            // 帧被守卫挡下（无深度、尺寸不符、对齐未开……）
    uint64_t rot_gated = 0;           // 被陀螺离群门拦下的帧数（P3：帧间旋转与陀螺积分不一致）
    bool gyro_still = false;          // 陀螺确认相机静止：旋转通道已冻在冻结点上（P1）
    int smooth_used = 0;              // 平均窗口里现在有几帧

    uint64_t map_hash = 0;            // 换工件、换体素都会变；重启后能确认还是那份基准
    size_t map_voxels = 0;
    std::string map_path;

    // 本帧与示教帧各自的取流档位。两者不同时点云采样密度差一截，修正量仍然算得出但门会变严，
    // 所以必须报出来而不是假装"米制空间与分辨率无关"。
    std::string align = "disabled";
    int capture_width = 0, capture_height = 0;
    int teach_capture_width = 0, teach_capture_height = 0;
    int64_t snapshot_ts_ms = 0;       // 主机时间：app 侧据此判断快照是不是 stale
};

class FollowWorker {
public:
    FollowWorker(std::shared_ptr<CameraDriver> camera, const follow::FollowConfig& cfg);
    ~FollowWorker();

    void start();
    void stop();

    // 由 HTTP 线程调用。**提交式**：受理一次档位切换后立即返回（切换在专用线程里跑），
    // 慢的部分是重启取流 pipeline（stop → 换档 → start → 重取内参），实测百毫秒到几十秒；
    // 把它同步压在 HTTP 线程里会直接超过客户端的请求超时（这就是旧版"使能失败、接口超时"
    // 的根因）。起不来/拒绝受理时把原因写进 err；*busy（可空）报"已有一次切换在跑"，
    // HTTP 据此给 409 而不是 503。进度/结果由 /follow/status 的 switching 字段轮询。
    bool setEnabled(bool enabled, std::string* err, bool* busy = nullptr);
    bool enabled() const { return enabled_.load(); }
    bool switching() const { return switching_.load(); }

    // follow 自己的配置读坏了（yaml 类型错、check_config 报致命）时用这条：相机服务**照样要跑**
    // （follow 是可选功能，不该因为一个可选功能的配置错就把视频流拖下水），但 setEnabled 必须
    // 拒绝并给出原话，而 /follow/status 要报同一个原因 —— 否则运维看到的是"点了没反应"。
    void setBlocked(const std::string& reason);
    std::string blockedReason() const;

    // 收 N 张深度帧做时间均值，冻结成新的参考地图，并**原子地**换给工作线程（旧地图由
    // shared_ptr 保命，正在配准的那一帧不会被抽走）。save_map 时才落盘。
    bool teach(bool save_map, std::string* err);

    FollowSnapshot snapshot() const;

private:
    void loop();
    // 档位切换线程体：setEnabled 受理的活在这里干完。失败原因落 switch_err_ 并写进快照，
    // 轮询方据此把"切换失败"和"还在切"分开。
    void switchTask(bool enable);
    bool doEnable(std::string* err);
    bool doDisable(std::string* err);
    follow::TrackParams trackParamsForCurrentStream() const;
    // 一帧能不能拿去解算。**loop() 和 teach() 共用同一条判据**：如果示教用的帧是运行期会被
    // 拒掉的帧，那基准本身就建在坏数据上，之后每帧都在跟一个不该存在的东西比。
    bool frameUsable(const FrameData& fd, std::string* why) const;

    std::shared_ptr<CameraDriver> camera_;
    follow::FollowConfig cfg_;

    std::atomic<bool> running_{false};
    std::atomic<bool> enabled_{false};
    std::thread thread_;

    // 档位切换（重启取流）的专用线程：HTTP 只受理不干活。switching_ 在切换开始时置位、
    // 线程末尾清除；switch_err_ 记最近一次切换的失败原因（空 = 没失败过/已清零）。
    std::atomic<bool> switching_{false};
    std::atomic<bool> switch_target_{false};
    std::thread switch_thread_;
    std::mutex switch_err_mutex_;
    std::string switch_err_;

    // 参考地图的代际：teach() 换图 → map_gen_+1 → 工作线程下一轮重建 Tracker。
    // 用 shared_ptr 而不是直接换对象，是因为 Tracker 持的是 const ReferenceMap&。
    // **state_mutex_ 保护的就是这一组**：map_ / map_gen_ / map_path_ / teach_cap_*_ / tracker_dirty_。
    // frontend_ / tracker_ / smoother_ 不在锁里，因为它们只在工作线程被读写：setEnabled() 先建好
    // frontend_ 再发布 enabled_；teach() 换图时只举 tracker_dirty_，平滑窗口由工作线程自己重置。
    mutable std::mutex state_mutex_;
    std::shared_ptr<follow::ReferenceMap> map_;
    uint64_t map_gen_ = 0;
    std::string map_path_;   // teach(save_map=true) 时落盘的那个路径；空 = 只在内存里
    int teach_cap_w_ = 0;
    int teach_cap_h_ = 0;

    std::unique_ptr<follow::FeatureFrontend> frontend_;
    std::unique_ptr<follow::Tracker> tracker_;
    uint64_t tracker_gen_ = 0;
    // 换图靠 map_gen_ 看出来；换档位（使能/关闭/被标定模式顶掉）不换图，但同样必须重建
    // Tracker —— 它里面的"上一帧 + 上一帧深度 + 跟丢计数"都属于旧档位。这一位就是给后者用的。
    bool tracker_dirty_ = true;
    std::string frontend_error_;   // 只由切换线程（switchTask/doEnable）读写
    follow::PoseSmoother smoother_;

    FollowSnapshot snap_;
    mutable std::mutex snap_mutex_;
    // 配置坏掉时的"这台机器上 follow 跑不了"原因。由 main 在 start() 之前写一次，
    // 之后只读 ⇒ 借用 snap_mutex_ 保护（它和快照是一对必须一致的数据：状态说 disabled，原因也说为什么）。
    std::string blocked_reason_;

    uint64_t last_frame_index_ = 0;
    bool have_frame_index_ = false;
    int64_t last_ts_ns_ = 0;

    // 只由工作线程读写，最后原样抄进快照 ⇒ 不需要锁。
    uint64_t frames_ = 0;        // 真正解算过的帧
    uint64_t dropped_ = 0;       // 有新帧但没赶上 / 同毫秒不可区分
    uint64_t rejected_ = 0;      // 被 frameUsable 挡下
    uint64_t rot_gated_ = 0;     // 被陀螺离群门拦下（P3）
    uint64_t still_frames_ = 0;  // 静止冻结生效的帧数（P1，退出静止时随摘要一起打）
    // P1 静止冻结：陀螺确认相机没在转时，旋转输出冻在冻结点上（平移照常更新）。
    // 冻住的是"进入静止那一刻的平滑旋转"，不是单帧值 —— 否则冻结本身会引入一次跳变。
    bool rot_frozen_ = false;
    Eigen::Matrix3d frozen_R_ = Eigen::Matrix3d::Identity();
    int64_t still_since_ms_ = 0;
    double last_log_norm_rad_s_ = 0.0;
    double fps_ = 0.0;
    int64_t fps_window_ms_ = 0;
    uint64_t fps_window_frames_ = 0;
    std::string last_reject_reason_;

    // 实时遥测日志状态（与 follow_pose 格式对齐，便于日志观察）
    bool has_logged_pose_ = false;
    double last_log_x_ = 0.0;
    double last_log_y_ = 0.0;
    double last_log_z_ = 0.0;
    double last_log_rx_ = 0.0;
    double last_log_ry_ = 0.0;
    double last_log_rz_ = 0.0;
    std::string last_log_state_;
    int64_t last_log_time_ms_ = 0;
    int64_t quiet_log_frames_ = 0;
    double quiet_max_dt_ = 0.0;
    double quiet_max_dr_ = 0.0;
};

} // namespace orbbec_service
