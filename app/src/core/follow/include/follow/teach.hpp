// 示教 = 取一帧、反投影、冻结成参考几何。设备相关的这半截只有一个实现，因为
// "基准是怎么来的"这件事绝不允许有两个版本。
//
// 分界线画在设备无关的 build_reference_map()（follow/teach_core.hpp）上：
//   * follow_node / follow_pose —— 用本文件的壳，自己开 Orbbec 凑 N 帧；
//   * 相机服务（follow 作为库跑在它里面）—— 直接喂它已经在交付的那一路深度帧。
// 三条路径共用同一个构建器 ⇒ "演示里基准是这份几何"才蕴含"服务里的基准也是它"，
// 而不是两套代码各自对。
#pragma once

#include <string>

#include "follow/odometry.hpp"
#include "follow/orbbec_capture.hpp"
#include "follow/reference_map.hpp"

namespace follow {

// 从 cap 收 num_frames 张深度帧，交给 build_reference_map()。参考系刻意取**示教那帧的相机系**：
// 于是之后每帧解出的 T_ref_cam 直接就是相对示教位的修正量，而 follow_pose 里它直接就是
// "相机在第一帧坐标系中的位姿"。内参合法性由核心层查，这里不再重复一遍。
//
// save_path 非空时落盘，并且**立刻读回比对内容哈希**（理由见 teach_core.hpp）。
// 失败写 err 并返回 false（一帧都没取到、点太少、深度非法、写不进去、读不回来，各有各的文案）。
bool teach_reference(OrbbecCapture& cap, const TrackParams& tp, const std::string& save_path,
                     ReferenceMap* out, std::string* err, int num_frames = 10);

}  // namespace follow
