// 陀螺时间基对齐：把设备时钟的陀螺样本换算到"和帧时间戳同一个域"。
//
// 为什么必须有这一步。336L 的 GyroFrame::getTimeStampUs() 是**自设备上电起的微秒数**
//（实测末值 1.57e9 µs ≈ 26 min），而本服务的 FrameData::timestamp_ms 是主机
// system_clock 的 epoch 毫秒 —— 两者差 6 个数量级。follow 的陀螺用法里有两条依赖
// "样本落在 [上一帧, 本帧] 这个区间里"：帧间旋转初值、以及离群门的互验区间。域不同时
// 区间里一个样本也框不进来，而失效现象是**静默的**：积分返回"没有样本"，于是初值降级、
// 门不参与，输出看起来完全正常，只有精度悄悄变差（见 follow/include/follow/odometry.hpp
// 顶上那段时间域说明）。
//
// 为什么不能改成"到达即打时间戳"。陀螺是一包多帧（burst）交付的：一次回调里可能有若干
// 条样本，它们的设备时间戳相差 5 ms，而主机侧看它们同一时刻到达。用到达时间就把这段
// dt 压成 0 ⇒ 积分出来的旋转直接少算。所以**必须保留设备 dt，只平移原点**。
//
// 平移量为什么要用"最小值"而不是均值：同一对时间戳满足
//     host_recv = device_ts + 真实钟差 + 单向传输与排队延迟
// 延迟恒为正且抖动，所以所有候选里最小的那个最接近真实钟差（NTP 选优的同一条理由）。
// 标定只做一次并冻结：后续任何"重新估计"都会在时间轴上打一个台阶，而台阶比常量误差毒得多
// —— 常量误差会让门的区间整体平移同样长度（误差二阶），台阶则直接变成 ω×Δ 的一阶误差。
//
// 精度够不够：帧周期 66 ms，主机与设备钟差漂移在几十 ppm 量级 ⇒ 分钟尺度里约 1~2 ms，
// 对应门误差 5°/s × 2 ms = 0.01°，比 1.0° 的门限低两个数量级。静止检测器更不依赖时间轴
//（它只看向量模），完全不受影响。
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>

namespace orbbec_service {

class GyroTimeBase {
 public:
  // 采几个配对就定标。8 个 frameset ≈ 0.5 s（15 fps），够把 USB 排队抖动取到最小值，
  // 又不会让"标定未完成 ⇒ 陀螺一直空转"的窗口长到影响示教。
  static constexpr int kProbePairs = 8;

  // 帧侧调用：喂一对"该 frameset 的设备时间戳(µs) ↔ 本进程打上的主机时间戳(ns)"。
  // 已完成标定后调用是廉价的空操作。device_us 为 0 表示这一帧没带设备时间戳，跳过。
  // 返回 true 只在这一次调用把标定定下来的那一帧出现 —— 让调用方恰好打一条日志，
  // 不必每帧轮询 ready()。
  bool offerPair(uint64_t device_us, int64_t host_ns);

  // 陀螺侧调用：设备 µs → 主机 ns。未标定完成时返回 0，调用方**必须丢弃**该样本：
  // 混进两个域的样本会让缓冲不再单调，那比没有样本更难查。
  int64_t toHostNs(uint64_t device_us) const;

  bool ready() const { return offset_ns_.load() != 0; }
  int64_t offset_ns() const { return offset_ns_.load(); }

  // --- 诊断：区分"标定还没完成"和"标定了但质量很差"，这两种的处置完全不同 ---
  int pairs_used() const;                            // 实际参与定标的配对数
  int64_t spread_ns() const { return spread_ns_; }   // 这些配对候选之间的最大离散度 ≈ 延迟抖动上界
  uint64_t dropped_before_ready() const { return dropped_.load(); }

  // 停流/换档/重连时调用：设备可能已经重启（时间戳原点变了），主机侧的排队路径也重建了。
  void reset();

 private:
  mutable std::mutex mtx_;
  int64_t best_candidate_ns_ = 0;
  int64_t worst_candidate_ns_ = 0;
  int collected_ = 0;
  int64_t spread_ns_ = 0;
  std::atomic<int64_t> offset_ns_{0};
  mutable std::atomic<uint64_t> dropped_{0};  // toHostNs() 是读路径，计数是诊断而非状态
};

}  // namespace orbbec_service
