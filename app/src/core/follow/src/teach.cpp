#include "follow/teach.hpp"

#include <algorithm>

#include "follow/logger.hpp"
#include "follow/teach_core.hpp"

namespace follow {

bool teach_reference(OrbbecCapture& cap, const TrackParams& tp, const std::string& save_path,
                     ReferenceMap* out, std::string* err, int num_frames) {
  if (out == nullptr || err == nullptr) {
    return false;
  }
  num_frames = std::max(1, num_frames);
  LOG_AT(LogLevel::kInfo, "teach") << "取示教均值帧 (计划 " << num_frames << " 帧)…";

  // 这一层只剩「从设备上凑齐 N 张深度帧」。均值、反投影、冻结、落盘回读全在
  // build_reference_map()（src/teach_core.cpp）—— 相机服务里那条路径共用同一份实现，
  // 所以"演示里基准是这份几何"才蕴含"服务里的基准也是它"，而不是两套代码各自对。
  std::vector<cv::Mat> depth_frames;
  depth_frames.reserve(static_cast<size_t>(num_frames));
  int64_t last_ts_ns = 0;

  for (int i = 0; i < num_frames; ++i) {
    RgbdFrame fr;
    if (!cap.wait_frame(&fr, err)) {
      if (depth_frames.empty()) {
        return false;
      }
      // 中途取帧失败：用已经凑到的这些，一次 dropout 不该把整个示教否掉。但 err 里
      // 留着的那句超时必须擦掉 —— 下面成功返回 true 时，调用方读到的不该是残留错误。
      err->clear();
      break;
    }
    // clone：取流层会复用同一块深度缓冲，而示教要跨 N 帧累加。
    depth_frames.push_back(fr.depth_mm.clone());
    last_ts_ns = fr.ts_ns;
  }

  return build_reference_map(depth_frames, tp, last_ts_ns, save_path, out, err);
}

}  // namespace follow
