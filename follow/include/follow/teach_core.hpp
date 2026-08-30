// 示教的设备无关核心：吃 N 张**已配对、已 D2C 对齐**的深度帧，产出冻结参考地图。
//
// 拆成两层是有原因的：「基准是怎么来的」这件事全项目只允许有一份实现，但"深度帧从哪来"
// 有两种答案 —— 独立进程自己开 Orbbec（follow_node / follow_pose），或者复用相机服务已经在
// 交付的那一路帧（follow 作为库跑在 orbbec_service 里）。所以把与设备无关的那半截放在
// libfollow（本文件），取帧的那半截留在 follow_device 的 teach_reference()。
//
// 这一层刻意不 include orbbec_capture.hpp：libfollow 的可构建性就是它的验收条件之一。
// 放在这里而不是"顺手让相机服务也链 follow_device"，是因为后者会把厂商 SDK 的符号要求
// 带进一个已经有自己取流实现的进程 —— 同一台设备两个 driver，比没有互斥更糟。
//
// 单位：深度图 1 LSB = 1 mm（Orbbec valueScale 实测 1），出参点云是米。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "follow/odometry.hpp"  // TrackParams（内参、zmin/zmax、stride、voxel、点数门都在这）
#include "follow/reference_map.hpp"

namespace follow {

// 时间域均值 + 反投影 + 冻结体素地图 + 可选落盘回读。
//
// 为什么要多帧均值而不是取单帧：结构光的散斑是**逐帧独立**的，同一个像素下一帧就换了值，
// 而示教出来的几何要当基准用很久。N 帧均值把散斑压成 √N 分之一，比事后调 voxel_m 划算
// （voxel 变大丢的是真实几何）。均值只信"至少 N/3 帧都有效"的像素：一次性的坏点被剔除，
// 而一直读不出来的地方本来也不该进基准。
//
// save_path 非空时落盘并**立刻读回比对内容哈希**：示教时内存里那张和下次启动从盘上拿到的
// 那张必须是同一份几何，否则基准在重启时悄悄换了，而所有修正量看起来仍然正常。
// 失败写 err 并返回 false（深度尺寸不一致、点数不足、写不进去、读不回来各有各的文案）。
bool build_reference_map(const std::vector<cv::Mat>& depth_frames, const TrackParams& tp,
                         int64_t built_ts_ns, const std::string& save_path, ReferenceMap* out,
                         std::string* err);

}  // namespace follow
