#include "follow_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "follow/pose_io.hpp"
#include "follow/teach_core.hpp"
#include "follow/types.hpp"
#include "logger.hpp"

namespace orbbec_service {

namespace {

int64_t steady_now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// 服务的内参是 3x3 嵌套 vector（JSON 友好），follow 要的是标量四件套。这里只搬运，不"补一个
// 看起来合理的默认值" —— 默认值由 intrinsics_loaded 守卫挡在外面，见 frameUsable()。
follow::CameraIntrinsics to_follow_k(const CameraIntrinsics& in) {
    follow::CameraIntrinsics k;
    k.width = in.width;
    k.height = in.height;
    if (in.intrinsic_matrix.size() >= 2 && in.intrinsic_matrix[0].size() >= 3 &&
        in.intrinsic_matrix[1].size() >= 3) {
        k.fx = in.intrinsic_matrix[0][0];
        k.cx = in.intrinsic_matrix[0][2];
        k.fy = in.intrinsic_matrix[1][1];
        k.cy = in.intrinsic_matrix[1][2];
    }
    return k;
}

// 与 follow_pose / follow_node 同一个种子：RANSAC 可复现，三处量出来的数才对得上。
constexpr uint32_t kTrackerSeed = 0x5EEDu;

constexpr double kRad2Deg = 180.0 / M_PI;

}  // namespace

FollowWorker::FollowWorker(std::shared_ptr<CameraDriver> camera, const follow::FollowConfig& cfg)
    : camera_(std::move(camera)), cfg_(cfg), smoother_(follow::kDefaultSmoothFrames) {}

FollowWorker::~FollowWorker() {
    stop();
}

void FollowWorker::start() {
    if (running_) return;
    running_ = true;
    thread_ = std::thread(&FollowWorker::loop, this);
    // 默认不使能 ⇒ 这条线程醒来也只是睡着，不碰前端、不算 GICP。配置来源必须能从日志还原。
    LOG_INFO("Follow", "worker 线程已起（默认不使能，不占算力）。配置: ",
             cfg_.source.empty() ? std::string("内置默认（没读到 yaml）") : cfg_.source);
}

void FollowWorker::stop() {
    if (!running_) return;
    running_ = false;
    // 先把切换线程等完：它正在重启取流，工作线程和它同时碰 CameraDriver 没问题（驱动自己加锁），
    // 但 frontend_/tracker_ 的释放必须等两边都停下。
    if (switch_thread_.joinable()) switch_thread_.join();
    if (thread_.joinable()) thread_.join();
    // tracker_/frontend_ 只在工作线程和切换线程里被使用，所以必须等 join 之后再释放。
    tracker_.reset();
    frontend_.reset();
}

follow::TrackParams FollowWorker::trackParamsForCurrentStream() const {
    follow::TrackParams tp = cfg_.track;
    tp.k = to_follow_k(camera_->getIntrinsics());   // 设备自报、按**当前启用的分辨率**取
    return tp;
}

void FollowWorker::setBlocked(const std::string& reason) {
    std::lock_guard<std::mutex> lock(snap_mutex_);
    blocked_reason_ = reason;
    snap_.enabled = false;
    snap_.status = "disabled";
    snap_.reason = reason.empty() ? "follow 未使能" : ("follow 配置不可用：" + reason);
}

std::string FollowWorker::blockedReason() const {
    std::lock_guard<std::mutex> lock(snap_mutex_);
    return blocked_reason_;
}

bool FollowWorker::setEnabled(bool enabled, std::string* err, bool* busy) {
    if (busy) *busy = false;
    // 配置坏掉时这里是最先撞到它的地方：把原话交回给点按钮的人，而不是让下面某一步"莫名其妙
    // 失败了"。相机服务本身不受影响 —— follow 是可选功能。
    const std::string blocked = blockedReason();
    if (!blocked.empty()) {
        if (err) *err = "follow 配置不可用：" + blocked;
        LOG_ERROR("Follow", "拒绝使能：follow 配置不可用：", blocked);
        return false;
    }

    if (switching_.load()) {
        // 已有一次切换在跑：不排队、不叠加 —— 两次方向相反的切换叠在一起会把档位搞成未知态。
        // 调用方的正确动作是轮询 /follow/status 等这一次切完。
        if (busy) *busy = true;
        if (err) *err = std::string("档位切换进行中（目标：") +
                        (switch_target_.load() ? "使能" : "关闭") + "），请轮询状态等它完成";
        return false;
    }

    if (enabled == enabled_.load()) {
        LOG_INFO("Follow", "使能状态未变（enabled=", (enabled ? "true" : "false"), "），不重复切档。");
        return true;
    }

    // 受理：置切换态 → 快照立刻反映"在切了"（否则页面点了按钮看不到任何变化）→ 起线程干活。
    switch_target_ = enabled;
    {
        std::lock_guard<std::mutex> lock(switch_err_mutex_);
        switch_err_.clear();
    }
    switching_ = true;
    {
        std::lock_guard<std::mutex> lock(snap_mutex_);
        snap_.switching = true;
        snap_.status = "switching";
        snap_.reason = enabled
                           ? std::string("档位切换中：取流切往 follow 档（重启 pipeline，可能要几十秒）")
                           : std::string("档位切换中：取流退回 hardware.camera 档");
        snap_.snapshot_ts_ms = steady_now_ms();
    }
    if (switch_thread_.joinable()) switch_thread_.join();   // 上一次已切完（switching_ 为假），收尾 join
    switch_thread_ = std::thread(&FollowWorker::switchTask, this, enabled);
    LOG_INFO("Follow", "已受理档位切换请求（目标：", (enabled ? "使能" : "关闭"), "），切换线程已起。");
    return true;
}

void FollowWorker::switchTask(bool enable) {
    // 整个慢活在这里同步跑完 —— 只是不再压在 HTTP 线程上。失败时回滚由 doEnable 内部完成。
    std::string err;
    const int64_t t0 = steady_now_ms();
    const bool ok = enable ? doEnable(&err) : doDisable(&err);
    {
        std::lock_guard<std::mutex> lock(switch_err_mutex_);
        switch_err_ = ok ? std::string() : err;
    }
    switching_ = false;
    {
        std::lock_guard<std::mutex> lock(snap_mutex_);
        snap_.switching = false;
        if (!ok) {
            snap_.status = "disabled";
            snap_.reason = "档位切换失败：" + err;
        }
        snap_.snapshot_ts_ms = steady_now_ms();
    }
    if (ok) {
        LOG_INFO("Follow", "档位切换完成（", (enable ? "使能" : "关闭"), "，耗时 ",
                 (steady_now_ms() - t0), "ms）");
    } else {
        LOG_ERROR("Follow", "档位切换失败（", (enable ? "使能" : "关闭"), "，耗时 ",
                  (steady_now_ms() - t0), "ms）：", err);
    }
}

bool FollowWorker::doDisable(std::string* err) {
    // 先落档位再落 enabled_：反过来的话工作线程会拿新档位的帧去解旧坐标里的东西。
    if (!camera_->setFollowProfile(false, 0, 0, 0, err)) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        tracker_dirty_ = true;   // 档位换了，Tracker 的"上一帧"是另一套网格
    }
    enabled_ = false;
    {
        // 停机即作废位姿：旧档位下解出的 T 属于另一套像素网格，页面继续拿它推臂就是拿
        // "停机前的最后一帧"当实时值。档位字段一起更新，因为"退回 hardware.camera"正是
        // 这次操作要确认的结果。
        const CameraStatus cst = camera_->getStatus();
        std::lock_guard<std::mutex> lock(snap_mutex_);
        snap_.enabled = false;
        snap_.status = "disabled";
        snap_.estimator = "none";
        snap_.has_pose = false;
        snap_.holding_last_pose = false;
        snap_.smooth_used = 0;
        snap_.capture_width = cst.capture_width;
        snap_.capture_height = cst.capture_height;
        snap_.align = cst.depth_align_mode;
        snap_.reason = "follow 已关闭：取流回到 hardware.camera 档位，参考地图仍留在内存里";
        snap_.snapshot_ts_ms = steady_now_ms();
    }
    LOG_INFO("Follow", "已关闭：取流回到 hardware.camera 档位；参考地图仍留在内存里，重新使能即可续用。");
    return true;
}

bool FollowWorker::doEnable(std::string* err) {
    // frontend_ 在发布 enabled_ **之前**建好，工作线程才可以在不加锁的情况下用它。
    // 建失败就把原因原样交回去 —— "superpoint 模型找不到"这类配置错要让点按钮的人当场看到。
    if (!frontend_) {
        int64_t tf0 = steady_now_ms();
        std::string f_err;
        frontend_ = follow::make_frontend(cfg_.frontend_kind, cfg_.frontend, &f_err);
        if (!frontend_) {
            frontend_error_ = f_err;
            LOG_ERROR("Follow", "特征前端创建失败: ", f_err);
            if (err) *err = "特征前端创建失败: " + f_err;
            return false;
        }
        frontend_error_.clear();
        LOG_INFO("Follow", "特征前端创建完成: ", frontend_->name(), " (took ", (steady_now_ms() - tf0), "ms)");
    }

    int64_t t_prof0 = steady_now_ms();
    if (!camera_->setFollowProfile(true, cfg_.capture.width, cfg_.capture.height, cfg_.capture.fps,
                                  err)) {
        return false;
    }
    LOG_INFO("Follow", "camera_->setFollowProfile 完成 (took ", (steady_now_ms() - t_prof0), "ms)");

    int64_t t_st0 = steady_now_ms();
    bool have_map = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        tracker_dirty_ = true;
        // teach_cap_*_ 不清零：本进程里 follow 档位是固定的，关开一次不会改变"这份基准是在
        // 哪一档建的"。清掉等于把已知的来源丢掉，档位与基准是否配套的检查会永久失效。
        have_map = static_cast<bool>(map_ && !map_->empty());
    }
    enabled_ = true;
    {
        // 同上：这次响应的 status 不能还停留在上一轮的 disabled。taught 决定下一步该点哪个。
        // 只碰 snap_mutex_ —— 和 state_mutex_ 嵌套会与工作线程的取锁顺序相反。
        const CameraStatus cst = camera_->getStatus();
        std::lock_guard<std::mutex> lock(snap_mutex_);
        snap_.enabled = true;
        snap_.taught = have_map;
        snap_.status = have_map ? "no_frame" : "no_map";
        snap_.has_pose = false;
        snap_.holding_last_pose = false;
        snap_.reason = have_map
                           ? std::string("已使能：等新档位下的第一帧解算")
                           : std::string("已使能，但还没有参考地图：POST /api/v1/camera/follow/teach");
        snap_.capture_width = cst.capture_width;
        snap_.capture_height = cst.capture_height;
        snap_.align = cst.depth_align_mode;
        snap_.snapshot_ts_ms = steady_now_ms();
    }
    LOG_INFO("Follow", "setEnabled snap 更新完成 (took ", (steady_now_ms() - t_st0), "ms)");
    LOG_INFO("Follow", "已使能：取流 ", cfg_.capture.width, "x", cfg_.capture.height, "@",
             cfg_.capture.fps, "fps（640x480 是硬件 D2C 的上限档），示教帧数=", cfg_.teach_frames,
             "。下一步 POST /api/v1/camera/follow/teach。");
    // 陀螺能力一次性讲清楚，出问题时日志里能对上号：三个用途，各自可关。
    LOG_INFO("Follow", "陀螺能力：①静止冻结（陀螺确认静止时旋转通道冻住、平移照常更新）②离群门 ",
             cfg_.track.gyro_rot_gate_deg, "°（帧间旋转 vs 陀螺积分互验，0=关）③示教静止门 ",
             cfg_.teach_max_motion_deg_s, "°/s",
             camera_->hasImu() ? "（0=关）" : "（本机无 IMU，自动跳过）");
    return true;
}

bool FollowWorker::frameUsable(const FrameData& fd, std::string* why) const {
    const CameraStatus st = camera_->getStatus();
    const CameraIntrinsics in = camera_->getIntrinsics();

    if (!st.online || !st.streaming) {
        *why = "相机不在线";
        return false;
    }
    // 内参必须是设备给的。编译期那套默认值（fx≈611.68 @1280x800）"看起来像个内参"，而拿它去
    // 反投影另一档分辨率的网格，会得到一整个自洽但全错的场景 —— 最坏的一类失效：下游一切正常。
    if (!st.intrinsics_loaded) {
        *why = "内参未从设备读到（getCameraParam 失败），拒绝反投影";
        return false;
    }
    if (!fd.has_depth || fd.depth.empty()) {
        *why = "本帧没有深度（标定模式关掉了深度流？）";
        return false;
    }
    if (!fd.has_color || fd.color.empty()) {
        *why = "本帧没有彩色，特征前端无从下手";
        return false;
    }
    // 对齐阶梯最后落到 ALIGN_DISABLE 时，深度有自己的一套 fx/cx（实测基线差 23.735 mm），而
    // follow 整条下游都假设深度落在彩色像素系里。这不是"精度降级"，是错位。
    if (st.depth_align_mode == "disabled") {
        *why = "深度未对齐到彩色（align=disabled），彩色内参反投影会得到错位的场景";
        return false;
    }
    if (fd.depth.cols != fd.color.cols || fd.depth.rows != fd.color.rows) {
        *why = "深度 " + std::to_string(fd.depth.cols) + "x" + std::to_string(fd.depth.rows) +
               " 与彩色 " + std::to_string(fd.color.cols) + "x" + std::to_string(fd.color.rows) +
               " 网格不同";
        return false;
    }
    if (in.width != fd.depth.cols || in.height != fd.depth.rows) {
        // 换档瞬间最常见：pipeline 起来了、手里这帧还是旧档位的。守卫放在这里，不靠时序运气。
        *why = "内参 " + std::to_string(in.width) + "x" + std::to_string(in.height) + " 与帧 " +
               std::to_string(fd.depth.cols) + "x" + std::to_string(fd.depth.rows) +
               " 不同（换档瞬间或内参未刷新）";
        return false;
    }
    if (fd.depth.type() != CV_16UC1) {
        *why = "深度类型不是 CV_16UC1（1 LSB = 1 mm 的约定不成立了）";
        return false;
    }
    return true;
}

bool FollowWorker::teach(bool save_map, std::string* err) {
    // 切换进行中收上来的帧属于新旧两档混合：建出来的基准和运行期几何对不上，这种示教宁可拒绝。
    if (switching_.load()) {
        if (err) *err = "档位切换进行中：等切换完成再示教（示教帧必须来自跟随用的那一档取流）。";
        return false;
    }
    const int need = std::max(1, cfg_.teach_frames);
    const int64_t t_start = steady_now_ms();
    // 15 fps 下 10 帧约 0.7 s。200 ms/帧是给"设备实际出流更慢"留的余量；超了就是真拿不到帧，
    // 不是再等等就好 —— 宁可带着已收到的帧数报失败，也不把 HTTP 线程挂在那儿。
    const int64_t deadline_ms = t_start + 1000 + static_cast<int64_t>(need) * 200;

    std::vector<cv::Mat> depths;
    depths.reserve(need);
    uint64_t last_index = 0;
    bool have_index = false;
    int64_t last_ts_ns = 0;
    int cap_w = 0, cap_h = 0;

    // P2 示教静止门：示教窗口内顺路统计陀螺角速度。与工作线程共用同一个样本队列，这里拿到的是
    // 其中一部分 —— 判断"动没动"只需样本代表性，不需要独占。各用各的量，不混时间戳。
    const bool motion_gate_on = cfg_.teach_max_motion_deg_s > 0.0 && camera_->hasImu();
    double motion_sum_rad_s = 0.0;
    double motion_max_rad_s = 0.0;
    uint64_t motion_samples = 0;

    while (static_cast<int>(depths.size()) < need) {
        if (!running_) {
            if (err) *err = "服务正在退出，示教中止。";
            return false;
        }
        if (!enabled_.load()) {
            if (err) *err = "follow 未使能：先启动再示教（示教帧必须来自跟随用的那一档取流）。";
            return false;
        }
        if (steady_now_ms() > deadline_ms) {
            const std::string msg = "示教收帧超时：只拿到 " + std::to_string(depths.size()) + "/" +
                                    std::to_string(need) + " 帧（相机在出流吗？档位被标定模式顶掉了？）";
            LOG_ERROR("Follow", msg);
            if (err) *err = msg;
            return false;
        }

        if (motion_gate_on) {
            std::vector<follow::GyroSample> gy;
            if (camera_->drainGyroSamples(&gy)) {
                for (const auto& g : gy) {
                    const double n = g.omega_cam_rad_s.norm();
                    if (!std::isfinite(n)) continue;
                    motion_sum_rad_s += n;
                    motion_max_rad_s = std::max(motion_max_rad_s, n);
                    ++motion_samples;
                }
            }
        }

        FrameData fd;
        if (!camera_->getLatestFrame(fd) ||
            (have_index && fd.frame_index == last_index)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        last_index = fd.frame_index;
        have_index = true;

        std::string why;
        if (!frameUsable(fd, &why)) {
            // 换档瞬间可能偶发残留上一档未出完的帧或内参对齐中的中间帧：丢弃并等待下一帧，
            // 直到收集齐 need 帧或触发 deadline_ms 超时保护。
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        // 不 clone：CameraDriver 交出来的就是它自己 clone 的私有缓冲，且换帧是整体赋值不是原地
        // 改写，我们的 Mat 句柄靠引用计数自会保住这块内存。（follow_device 那边要 clone，是因为
        // 它自己复用取流缓冲 —— 两层的契约不同，别照抄。）
        depths.push_back(fd.depth);
        last_ts_ns = static_cast<int64_t>(fd.timestamp_ms) * 1000000;
        cap_w = fd.depth.cols;
        cap_h = fd.depth.rows;
    }

    if (motion_gate_on) {
        // 样本太少（刚使能、IMU 还没出流）时不表态：宁可不拦，也不拿三五个样本定生死。
        if (motion_samples >= 20) {
            const double avg_deg_s =
                motion_sum_rad_s / static_cast<double>(motion_samples) * kRad2Deg;
            const double max_deg_s = motion_max_rad_s * kRad2Deg;
            if (avg_deg_s > cfg_.teach_max_motion_deg_s) {
                char msg[256];
                std::snprintf(msg, sizeof(msg),
                              "示教被静止门拒绝：收帧窗口内相机在动（平均 %.2f°/s、峰值 %.2f°/s > "
                              "门限 %.2f°/s）。停稳相机再示教，否则基准里会烙进运动畸变。",
                              avg_deg_s, max_deg_s, cfg_.teach_max_motion_deg_s);
                LOG_ERROR("Follow", msg);
                if (err) *err = msg;
                return false;
            }
            LOG_INFO("Follow", "示教静止门通过：收帧窗口内角速度 平均 ", avg_deg_s,
                     "°/s、峰值 ", max_deg_s, "°/s（门限 ", cfg_.teach_max_motion_deg_s,
                     "°/s，样本 ", motion_samples, "）");
        } else {
            LOG_WARN("Follow", "示教静止门：陀螺样本不足（", motion_samples, " 个），本次不拦");
        }
    }

    const follow::TrackParams tp = trackParamsForCurrentStream();
    auto fresh = std::make_shared<follow::ReferenceMap>();
    const std::string save_path = save_map ? cfg_.map_path : std::string();
    if (!follow::build_reference_map(depths, tp, last_ts_ns, save_path, fresh.get(), err)) {
        LOG_ERROR("Follow", "示教失败: ", err && !err->empty() ? *err : std::string("(未给原因)"));
        return false;
    }

    uint64_t hash = 0;
    size_t voxels = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        map_ = std::move(fresh);
        ++map_gen_;
        map_path_ = save_path;
        teach_cap_w_ = cap_w;
        teach_cap_h_ = cap_h;
        tracker_dirty_ = true;   // 新基准 ⇒ 旧的"上一帧 / 跟丢计数"全部作废
        hash = map_->info().content_hash;
        voxels = map_->info().map_voxels;
    }
    LOG_INFO("Follow", "已示教 ", depths.size(), " 帧时间均值 @", cap_w, "x", cap_h,
             "  体素=", voxels, "  hash=", hash,
             save_path.empty() ? std::string("（未落盘）") : std::string("  → " + save_path));

    // 示教的结果当场写进快照。等工作线程下一轮再反映是不够的：POST /follow/teach 的响应体是
    // 页面"调零"按钮唯一信任的即时反馈，返回 taught=false 会让一次成功的示教看起来像失败。
    // 位姿类字段仍然只由工作线程写 —— 这里只补示教自己负责的那几项。
    {
        std::lock_guard<std::mutex> lock(snap_mutex_);
        snap_.enabled = enabled_.load();
        snap_.taught = true;
        snap_.map_hash = hash;
        snap_.map_voxels = voxels;
        snap_.map_path = save_path;
        snap_.teach_capture_width = cap_w;
        snap_.teach_capture_height = cap_h;
        snap_.status = "no_frame";
        snap_.reason = "刚示教：等新档位下的第一帧解算";
        snap_.holding_last_pose = false;
        snap_.snapshot_ts_ms = steady_now_ms();
    }
    return true;
}

void FollowWorker::loop() {
    constexpr int kIdleSleepMs = 30;
    constexpr int kWaitFrameSleepMs = 4;
    constexpr int64_t kFpsWindowMs = 2000;

    fps_window_ms_ = steady_now_ms();

    while (running_) {
        // ---- 状态同步：换图（map_gen_）或换档（tracker_dirty_）都要重建 Tracker --------
        std::shared_ptr<follow::ReferenceMap> map;
        uint64_t gen = 0;
        bool dirty = false;
        std::string map_path;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            map = map_;
            gen = map_gen_;
            dirty = tracker_dirty_;
            map_path = map_path_;
            tracker_dirty_ = false;
        }
        if (dirty || gen != tracker_gen_) {
            tracker_.reset();
            tracker_gen_ = gen;
            have_frame_index_ = false;
            last_ts_ns_ = 0;
            has_logged_pose_ = false;
            quiet_log_frames_ = 0;
            quiet_max_dt_ = 0.0;
            quiet_max_dr_ = 0.0;
            rot_frozen_ = false;   // Tracker 重建 ⇒ 静止检测器也归零，旧冻结点不属于新档位/新图
            still_frames_ = 0;
            if (map && !map->empty() && frontend_) {
                tracker_ = std::make_unique<follow::Tracker>(trackParamsForCurrentStream(), *map,
                                                             kTrackerSeed);
                smoother_ = follow::PoseSmoother(follow::kDefaultSmoothFrames);
                LOG_INFO("Follow", "Tracker 已重建（map_gen=", gen, "，档位变化=", dirty,
                         "，体素=", map->info().map_voxels, "）");
            }
        }

        const bool profile_live = camera_->isFollowProfile();

        // ---- 未使能，或档位已被顶掉：报一个"确实没在算"的快照，然后接着睡 ----------------
        // 后半个条件是"被顶掉"：进标定模式时 CameraDriver 会自动关掉 follow 档（那里没深度）。
        // 那时还报 enabled=true，页面就会显示一个"在跟随却没有深度"的状态 —— 假活在跑。
        if (!enabled_.load() || !profile_live) {
            if (enabled_.load() && !profile_live) {
                if (switching_.load() && !switch_target_.load()) {
                    // 正在执行"关闭"切换：档位先落、enabled_ 由切换线程落。这里不抢方向盘，
                    // 否则两条路径各自写快照，页面会先看到 disabled 再看到 switching，状态抖动。
                } else {
                    enabled_ = false;
                    LOG_WARN("Follow", "取流已不在 follow 档（多半切到标定模式了），follow 自动关闭。");
                }
            }
            const CameraStatus cst = camera_->getStatus();
            const bool in_calib = camera_->isCalibrationMode();
            std::lock_guard<std::mutex> lock(snap_mutex_);
            snap_.enabled = false;
            snap_.connected = camera_->isConnected();
            snap_.status = "disabled";
            snap_.estimator = "none";
            // 关闭后页面仍要看一眼档位有没有退回 hardware.camera —— 快照的档位字段不能只在
            // "解算过的帧"里才有值，否则停止跟随这件事本身就没法被确认。
            snap_.align = cst.depth_align_mode;
            snap_.capture_width = cst.capture_width;
            snap_.capture_height = cst.capture_height;
            // 分清三种情形，因为下一步动作各不相同："在标定模式"要先退出那个模式（这里不问
            // "是谁关的"：HTTP 路由和工作线程都会关，谁先动手都会让另一种问法得到错的回答）；
            // "配置不可用"要先修 yaml；剩下才是"没人开"，点一下就好。
            if (in_calib) {
                snap_.reason = "标定模式下没有深度流，follow 跑不了：先退出标定模式再使能 follow";
            } else {
                snap_.reason = blocked_reason_.empty()
                                   ? std::string("follow 未使能")
                                   : ("follow 配置不可用：" + blocked_reason_);
            }
            snap_.has_pose = false;
            snap_.holding_last_pose = false;
            snap_.snapshot_ts_ms = steady_now_ms();
            std::this_thread::sleep_for(std::chrono::milliseconds(kIdleSleepMs));
            continue;
        }

        if (!tracker_) {
            std::lock_guard<std::mutex> lock(snap_mutex_);
            snap_.enabled = true;
            snap_.connected = camera_->isConnected();
            snap_.taught = false;
            snap_.status = "no_map";
            snap_.reason = "还没有参考地图：POST /api/v1/camera/follow/teach 示教一次";
            {
                const CameraStatus cst = camera_->getStatus();
                snap_.align = cst.depth_align_mode;
                snap_.capture_width = cst.capture_width;
                snap_.capture_height = cst.capture_height;
            }
            snap_.map_path = map_path;
            snap_.snapshot_ts_ms = steady_now_ms();
            std::this_thread::sleep_for(std::chrono::milliseconds(kIdleSleepMs));
            continue;
        }

        // ---- 取一帧新帧：自己去重，绝不碰驱动的共享游标 --------------------------------
        FrameData fd;
        if (!camera_->getLatestFrame(fd) || (have_frame_index_ && fd.frame_index == last_frame_index_)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kWaitFrameSleepMs));
            std::lock_guard<std::mutex> lock(snap_mutex_);
            snap_.enabled = true;
            snap_.connected = camera_->isConnected();
            // 只有本来就"没有任何读数"时才改成 no_frame：算完一帧之后 status 是本帧真结果，
            // 不能因为"这一毫秒没有新帧"就被写成 no_frame —— 那会把 out_of_envelope 藏掉。
            if (!snap_.has_pose) {
                snap_.status = "no_frame";
                snap_.reason = "还没有新帧可算（相机在出流吗）";
            }
            snap_.snapshot_ts_ms = steady_now_ms();
            continue;
        }
        last_frame_index_ = fd.frame_index;
        have_frame_index_ = true;

        std::string why;
        if (!frameUsable(fd, &why)) {
            ++rejected_;
            std::lock_guard<std::mutex> lock(snap_mutex_);
            snap_.enabled = true;
            snap_.connected = true;
            snap_.rejected = rejected_;
            snap_.status = "config_invalid";
            snap_.reason = why;
            snap_.snapshot_ts_ms = steady_now_ms();
            // 守卫失败通常持续一小段（换档、拔线）。同一原因只喊一次，否则日志会被淹掉而信息
            // 量还是那一条。
            if (why != last_reject_reason_) {
                last_reject_reason_ = why;
                LOG_WARN("Follow", "帧被守卫挡下: ", why);
            }
            continue;
        }
        last_reject_reason_.clear();

        // 主机毫秒换算。粒度比帧周期粗，同一毫秒内的两帧在这里不可区分 —— 与其造一个 +1ns 的
        // 假时间，不如记成一次丢帧（15 fps = 66 ms/帧，实测极少发生，真发生说明取流在抖）。
        const int64_t ts_ns = static_cast<int64_t>(fd.timestamp_ms) * 1000000;
        if (ts_ns <= last_ts_ns_) {
            ++dropped_;
            std::lock_guard<std::mutex> lock(snap_mutex_);
            snap_.dropped = dropped_;
            snap_.snapshot_ts_ms = steady_now_ms();
            continue;
        }
        last_ts_ns_ = ts_ns;
        const int64_t t0 = steady_now_ms();

        // 注入积累的板载 IMU 陀螺仪样本（~200Hz 高频旋转初值）
        std::vector<follow::GyroSample> gyros;
        if (camera_->drainGyroSamples(&gyros)) {
            for (const auto& gs : gyros) {
                tracker_->push_gyro(gs.ts_ns, gs.omega_cam_rad_s);
            }
        }

        follow::FeatureFrame ff = frontend_->extract(fd.color, ts_ns);
        follow::TrackResult r = tracker_->track(ff, fd.depth, ts_ns);
        const double ms = static_cast<double>(steady_now_ms() - t0);
        ++frames_;

        // P3 离群门：帧间旋转与同窗口陀螺积分偏差超门限 ⇒ 这帧不采纳（Tracker 已 hold 上一可信值）。
        // 坏帧通常成串出现（遮挡抖动、反光闪烁），每一帧都记一笔，事后按累计数看严重程度。
        if (r.rot_gated) {
            ++rot_gated_;
            LOG_WARN("Follow", "离群门拦下坏帧：帧间旋转与陀螺积分偏差 ", r.rot_gate_err_deg,
                     "° > ", cfg_.track.gyro_rot_gate_deg, "°，保持上一可信位姿（累计 ",
                     rot_gated_, " 帧）");
        }

        // 统计一帧不落，报出去的是平滑位姿 —— 与 follow_pose 完全同一条策略：静止单帧噪声
        // 1~2 mm 比读数名还大，N 帧平均压到 sd/√N 才有意义（见 pose_smoother.hpp）。
        smoother_.push(r.T_ref_cam);
        Eigen::Isometry3d disp_T = (smoother_.size() > 1) ? smoother_.value() : r.T_ref_cam;

        // P1 静止冻结：陀螺确认相机没在转 ⇒ 旋转冻在进入静止那一刻的平滑旋转上，平移照常更新。
        // 迟进快出的迟滞在检测器里（gyro_filter.hpp）；这里只做冻/解冻与记账。
        if (r.gyro_still && r.estimator != follow::Estimator::kNone) {
            if (!rot_frozen_) {
                frozen_R_ = disp_T.rotation();
                still_since_ms_ = steady_now_ms();
                still_frames_ = 0;
                LOG_INFO("Follow", "陀螺确认静止：旋转通道冻结（冻在进入静止时的平滑旋转上，平移照常更新）");
            }
            disp_T.linear() = frozen_R_;
            ++still_frames_;
        } else if (rot_frozen_) {
            LOG_INFO("Follow", "陀螺检测到运动：旋转通道解冻（本次静止冻结了 ", still_frames_,
                     " 帧 / ", (steady_now_ms() - still_since_ms_), " ms，其间旋转抖动被整体抹掉）");
            rot_frozen_ = false;
            still_frames_ = 0;
        }

        const follow::DobotPose dp = follow::to_dobot(disp_T);   // 与发臂同一套换算：mm + ZYX deg

        const CameraStatus cst = camera_->getStatus();

        std::lock_guard<std::mutex> lock(snap_mutex_);
        snap_.enabled = true;
        snap_.connected = true;
        snap_.taught = true;
        snap_.status = follow::to_string(r.status);
        snap_.estimator = follow::to_string(r.estimator);
        snap_.has_pose = (r.estimator != follow::Estimator::kNone);
        snap_.holding_last_pose = (r.status != follow::Status::kOk);
        snap_.pose_mm[0] = dp.x_mm;
        snap_.pose_mm[1] = dp.y_mm;
        snap_.pose_mm[2] = dp.z_mm;
        snap_.pose_rpy_deg[0] = dp.rx_deg;
        snap_.pose_rpy_deg[1] = dp.ry_deg;
        snap_.pose_rpy_deg[2] = dp.rz_deg;
        snap_.norm_t_mm = std::sqrt(dp.x_mm * dp.x_mm + dp.y_mm * dp.y_mm + dp.z_mm * dp.z_mm);
        snap_.norm_r_deg = Eigen::AngleAxisd(disp_T.rotation()).angle() * kRad2Deg;
        // 增量给矩阵不给欧拉角：app 侧要把它乘到臂的基准上，欧拉角相乘是错的。
        for (int i = 0; i < 3; ++i) {
            snap_.delta_t_m[i] = disp_T.translation()[i];
            snap_.sigma_t_mm[i] = r.unc.trans_sigma_mm[i];
            snap_.sigma_r_deg[i] = r.unc.rot_sigma_deg[i];
            for (int j = 0; j < 3; ++j) {
                snap_.delta_r[i * 3 + j] = disp_T.rotation()(i, j);
            }
        }
        snap_.gicp_inliers = r.gicp_inliers;
        snap_.inlier_ratio = r.inlier_ratio;
        snap_.gicp_cost = r.gicp_cost;
        snap_.cloud_points = r.cloud_points;
        snap_.compute_ms = ms;
        snap_.smooth_used = smoother_.used();
        snap_.gyro_still = r.gyro_still;   // 本帧时刻的陀螺静止结论（页面/日志排查用）
        snap_.rot_gated = rot_gated_;      // 离群门累计拦截计数
        snap_.frames = frames_;
        snap_.dropped = dropped_;
        snap_.rejected = rejected_;
        snap_.fps = fps_;
        snap_.align = cst.depth_align_mode;
        snap_.capture_width = fd.depth.cols;
        snap_.capture_height = fd.depth.rows;
        snap_.map_hash = map ? map->info().content_hash : 0;
        snap_.map_voxels = map ? map->info().map_voxels : 0;
        snap_.map_path = map_path;
        snap_.teach_capture_width = teach_cap_w_;
        snap_.teach_capture_height = teach_cap_h_;
        snap_.snapshot_ts_ms = steady_now_ms();
        snap_.reason.clear();
        if (snap_.holding_last_pose) {
            // 出包络/跟丢时报的位姿是**上一个可信值**，不标出来就会被当成当前读数拿去判断。
            if (r.status == follow::Status::kDegenerate) {
                snap_.reason = "至少一个自由度本帧没测到（σ 超门限）；sigma_* 里 null = 无估计";
            } else if (r.status == follow::Status::kOutOfEnvelope) {
                snap_.reason = "与参考几何重叠不足：要重新示教，不是跟丢";
            } else if (r.status == follow::Status::kLost) {
                snap_.reason = "两个解算器都失败：位姿保持上一可信值";
            } else if (r.status == follow::Status::kRotGated) {
                snap_.reason = "帧间旋转与陀螺积分互验失败（疑似坏帧）：位姿保持上一可信值";
            } else {
                snap_.reason = std::string("状态 ") + snap_.status + "：位姿保持上一可信值";
            }
        } else if (teach_cap_w_ != 0 && (teach_cap_w_ != fd.depth.cols || teach_cap_h_ != fd.depth.rows)) {
            // 米制空间里地图与分辨率无关，但采样密度有关：点数掉一档，min_cloud_points /
            // inlier_ratio 这些门就会变严。所以它是个提示，不是错误。
            snap_.reason = "示教档位 " + std::to_string(teach_cap_w_) + "x" +
                           std::to_string(teach_cap_h_) + " 与当前 " +
                           std::to_string(fd.depth.cols) + "x" + std::to_string(fd.depth.rows) +
                           " 不同：几何仍可比，但点密度变了，门会更严";
        }

        // 打印实时跟踪遥测日志（与 follow_pose 格式对齐，便于日志分析和观察）
        bool should_log = false;
        if (!has_logged_pose_) {
            should_log = true;
            has_logged_pose_ = true;
        } else {
            double dev_t = 0.0, dev_r = 0.0;
            dev_t = std::max({std::fabs(dp.x_mm - last_log_x_),
                              std::fabs(dp.y_mm - last_log_y_),
                              std::fabs(dp.z_mm - last_log_z_)});
            dev_r = std::max({std::fabs(dp.rx_deg - last_log_rx_),
                              std::fabs(dp.ry_deg - last_log_ry_),
                              std::fabs(dp.rz_deg - last_log_rz_)});
            quiet_max_dt_ = std::max(quiet_max_dt_, dev_t);
            quiet_max_dr_ = std::max(quiet_max_dr_, dev_r);

            const std::string cur_state = std::string(follow::to_string(r.status)) + "/" + follow::to_string(r.estimator);
            if (cur_state != last_log_state_) {
                should_log = true;
            } else if (dev_t > 1.0 || dev_r > 0.1) {
                should_log = true;
            } else if (steady_now_ms() - last_log_time_ms_ >= 5000) {
                should_log = true;
            }
        }

        if (should_log) {
            char log_buf[512];
            const double sT_max = std::max({r.unc.trans_sigma_mm[0], r.unc.trans_sigma_mm[1], r.unc.trans_sigma_mm[2]});
            const double sR_max = std::max({r.unc.rot_sigma_deg[0], r.unc.rot_sigma_deg[1], r.unc.rot_sigma_deg[2]});

            int len = std::snprintf(
                log_buf, sizeof(log_buf),
                "#%6lld  X=%+8.2f Y=%+8.2f Z=%+8.2f mm  rx=%+7.2f ry=%+7.2f rz=%+7.2f deg  |t|=%7.2f |r|=%6.2f  %-8s/%-6s inl=%5d(%4.2f) sT=%5.2f sR=%5.3f %5.1fms",
                static_cast<long long>(frames_), dp.x_mm, dp.y_mm, dp.z_mm, dp.rx_deg, dp.ry_deg, dp.rz_deg,
                snap_.norm_t_mm, snap_.norm_r_deg, follow::to_string(r.status), follow::to_string(r.estimator),
                r.gicp_inliers, r.inlier_ratio, sT_max, sR_max, ms);

            if (quiet_log_frames_ > 0 && len < (int)sizeof(log_buf) - 64) {
                len += std::snprintf(log_buf + len, sizeof(log_buf) - len,
                                     "  〔静默 %lld 帧 · 其间最大偏离 %.2fmm/%.3fdeg〕",
                                     static_cast<long long>(quiet_log_frames_), quiet_max_dt_, quiet_max_dr_);
            }
            if (snap_.holding_last_pose && len < (int)sizeof(log_buf) - 32) {
                len += std::snprintf(log_buf + len, sizeof(log_buf) - len, "  <保持上一位姿>");
            }
            if (rot_frozen_ && len < (int)sizeof(log_buf) - 48) {
                len += std::snprintf(log_buf + len, sizeof(log_buf) - len,
                                     "  〔静止·旋转冻结 %lld 帧〕",
                                     static_cast<long long>(still_frames_));
            }

            LOG_INFO("Follow", log_buf);

            last_log_x_ = dp.x_mm;
            last_log_y_ = dp.y_mm;
            last_log_z_ = dp.z_mm;
            last_log_rx_ = dp.rx_deg;
            last_log_ry_ = dp.ry_deg;
            last_log_rz_ = dp.rz_deg;
            last_log_state_ = std::string(follow::to_string(r.status)) + "/" + follow::to_string(r.estimator);
            last_log_time_ms_ = steady_now_ms();
            quiet_log_frames_ = 0;
            quiet_max_dt_ = 0.0;
            quiet_max_dr_ = 0.0;
        } else {
            ++quiet_log_frames_;
        }

        // 帧率窗口 2 s，与 CameraDriver 的口径一致 ⇒ 两处报的 fps 能互相印证。
        const int64_t now_ms = steady_now_ms();
        if (now_ms - fps_window_ms_ >= kFpsWindowMs) {
            fps_ = static_cast<double>(frames_ - fps_window_frames_) * 1000.0 /
                   static_cast<double>(now_ms - fps_window_ms_);
            fps_window_ms_ = now_ms;
            fps_window_frames_ = frames_;
        }
    }
}

FollowSnapshot FollowWorker::snapshot() const {
    std::lock_guard<std::mutex> lock(snap_mutex_);
    FollowSnapshot s = snap_;
    // 开关状态以原子标志为准，不以快照为准：快照由工作线程按轮次写，刚点完按钮就来读会拿到
    // "还没关"的旧值。同一次响应里 enabled=false 却 status=ok 是自相矛盾，一并纠正。
    s.enabled = enabled_.load();
    // switching 同理：切换末尾先清原子标志再写快照，这里以原子标志为准才不会把"刚切完"读成"还在切"。
    s.switching = switching_.load();
    if (!s.enabled && !s.switching) {
        s.status = "disabled";
        s.holding_last_pose = false;
        if (s.reason.empty()) s.reason = blocked_reason_.empty() ? "follow 未使能" : ("follow 配置不可用：" + blocked_reason_);
    }
    return s;
}

} // namespace orbbec_service
