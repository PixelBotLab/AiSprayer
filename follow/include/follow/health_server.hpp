// follow_node 的只读健康端点。端口默认 18081（18080 是相机服务，8008 是 ZLM）。
//
// 存在的理由不是"能 curl 一下"：follow 是独立进程，Python 门面只能靠这个端点判断
// 子进程是否真的在工作 —— 进程活着但取流卡死、或参考地图根本没建，都必须是可观测的。
// 因此这里报的是**判据**而不是心跳：frames / fps / dropout / 上一次解算状态 / 参考地图哈希。
//
// 状态由主循环 update() 推进，服务器线程只读快照（一把互斥锁，值拷贝）。POST /teach
// 不直接建图：示教必须在取流线程里发生，所以这里只置一个标志位，由主循环取走。
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace follow {

struct HealthSnapshot {
  // "starting" | "capturing" | "teaching" | "tracking" | "waiting_teach" | "stopped"
  std::string state = "starting";
  std::string status = "config_invalid";   // to_string(Status)
  std::string estimator = "none";
  std::string align = "none";
  std::string last_error;

  int64_t frames = 0;
  double fps = 0.0;
  double period_ms = 0.0;
  double compute_ms = 0.0;       // 上一帧全链路耗时
  int64_t dropouts = 0;
  int64_t unpaired_framesets = 0;
  int64_t bad_frames = 0;        // wait_frame 返回 false 但设备还在

  // 相对示教位的修正量，给运维看：mm + deg（与 pose_io 出界一致）
  std::array<double, 6> correction{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double sigma_t_mm = 0.0;       // 最坏方向 1σ
  double sigma_r_deg = 0.0;
  double inlier_ratio = 0.0;
  int gicp_inliers = 0;
  size_t cloud_points = 0;

  bool device_present = true;
  bool lock_held = false;
  std::string map_path;
  uint64_t map_hash = 0;         // 参考地图内容哈希：换工件一定换哈希
  int64_t map_points = 0;
  int64_t map_built_ts_ns = 0;
  int64_t uptime_ms = 0;
  bool dry_run = true;
};

class HealthServer {
 public:
  HealthServer();
  ~HealthServer();
  HealthServer(const HealthServer&) = delete;
  HealthServer& operator=(const HealthServer&) = delete;

  // 绑定失败（端口被占）返回 false 并写 err。刻意不"换个端口再试"：上层是按端口找
  // 这个进程的，悄悄换端口等于让门面找不到它。
  bool start(int port, std::string* err);
  void stop();

  void update(const HealthSnapshot& s);
  HealthSnapshot snapshot() const;
  int port() const { return port_; }

  // 主循环每帧调用：有 POST /teach 待处理时返回 true 并清零。
  bool take_teach_request();
  void request_teach();

 private:
  struct Impl;
  std::unique_ptr<Impl> im_;
  int port_ = 0;
};

std::string to_json(const HealthSnapshot& s);

}  // namespace follow
