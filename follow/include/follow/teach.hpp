// 示教 = 取一帧、反投影、冻结成参考几何。follow_node 和 follow_pose 共用这一份，
// 因为"基准是怎么来的"这件事绝不允许有两个版本。
#pragma once

#include <string>

#include "follow/odometry.hpp"
#include "follow/orbbec_capture.hpp"
#include "follow/reference_map.hpp"

namespace follow {

// 参考系刻意取**这一帧的相机系**：于是之后每帧解出的 T_ref_cam 直接就是相对示教位的修正量，
// 而 follow_pose 里它直接就是"相机在第一帧坐标系中的位姿"。
//
// save_path 非空时落盘，并且**立刻读回比对内容哈希**：示教时内存里那张和下次启动从盘上
// 拿到的那张必须是同一份几何，否则基准在重启时悄悄换了，而所有修正量看起来仍然正常。
// 失败写 err 并返回 false（点太少、深度非法、写不进去、读不回来，各有各的文案）。
bool teach_reference(OrbbecCapture& cap, const TrackParams& tp, const std::string& save_path,
                     ReferenceMap* out, std::string* err, int num_frames = 10);

}  // namespace follow
