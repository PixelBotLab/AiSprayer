#include "follow/teach_core.hpp"

#include <algorithm>
#include <filesystem>

#include "follow/cloud.hpp"
#include "follow/logger.hpp"

namespace follow {

bool build_reference_map(const std::vector<cv::Mat>& depth_frames, const TrackParams& tp,
                         int64_t built_ts_ns, const std::string& save_path, ReferenceMap* out,
                         std::string* err) {
  if (out == nullptr || err == nullptr) {
    return false;
  }
  // fx<=0 会让 unproject 除零产出 inf/NaN，而 NaN 躲得过所有范围比较 —— 这道校验必须在
  // 用内参之前做，不能指望下游过滤。放在核心层而不是取帧层，两条调用路径才都躲不掉。
  if (!tp.k.valid()) {
    *err = "内参非法，反投影会产出 inf/NaN 点云（而 NaN 躲得过所有范围比较）";
    return false;
  }
  if (depth_frames.empty()) {
    *err = "示教没有收到任何深度帧";
    return false;
  }

  cv::Mat accum_depth = cv::Mat::zeros(depth_frames[0].size(), CV_32FC1);
  cv::Mat valid_count = cv::Mat::zeros(depth_frames[0].size(), CV_32SC1);
  const int rows = depth_frames[0].rows;
  const int cols = depth_frames[0].cols;
  int frames_collected = 0;

  for (const cv::Mat& depth : depth_frames) {
    // 尺寸不一致绝不能"跳过就当没看见"：基准几何会由一部分分辨率的像素构成，而运行期
    // 反投影用的是另一套网格，错位在数值上完全看不出来。
    if (depth.rows != rows || depth.cols != cols) {
      *err = "示教帧尺寸不一致（第 " + std::to_string(frames_collected + 1) + " 帧 " +
             std::to_string(depth.cols) + "x" + std::to_string(depth.rows) + " vs " +
             std::to_string(cols) + "x" + std::to_string(rows) + "）";
      return false;
    }
    for (int r = 0; r < rows; ++r) {
      const uint16_t* ptr_src = depth.ptr<uint16_t>(r);
      float* ptr_acc = accum_depth.ptr<float>(r);
      int32_t* ptr_cnt = valid_count.ptr<int32_t>(r);
      for (int c = 0; c < cols; ++c) {
        const uint16_t d = ptr_src[c];
        if (d > 0) {
          ptr_acc[c] += static_cast<float>(d);
          ptr_cnt[c] += 1;
        }
      }
    }
    ++frames_collected;
  }

  // 均值深度图 (CV_16UC1，滤除散斑噪点)
  cv::Mat avg_depth = cv::Mat::zeros(accum_depth.size(), CV_16UC1);
  const int min_valid_frames = std::max(1, frames_collected / 3);
  for (int r = 0; r < avg_depth.rows; ++r) {
    const float* ptr_acc = accum_depth.ptr<float>(r);
    const int32_t* ptr_cnt = valid_count.ptr<int32_t>(r);
    uint16_t* ptr_dst = avg_depth.ptr<uint16_t>(r);
    for (int c = 0; c < avg_depth.cols; ++c) {
      if (ptr_cnt[c] >= min_valid_frames) {
        ptr_dst[c] = static_cast<uint16_t>(std::round(ptr_acc[c] / static_cast<float>(ptr_cnt[c])));
      }
    }
  }

  CloudStats cs;
  const std::vector<Eigen::Vector3f> pts =
      depth_to_cloud(avg_depth, tp.k, tp.zmin_m, tp.zmax_m, tp.depth_stride, &cs);
  if (static_cast<int>(pts.size()) < tp.min_cloud_points) {
    *err = "示教均值帧只有 " + std::to_string(pts.size()) + " 个点（需 >= " +
           std::to_string(tp.min_cloud_points) + "）；采样 " + std::to_string(cs.total_samples) +
               " 中 距离外 " + std::to_string(cs.rejected_range) + " 非有限 " +
           std::to_string(cs.rejected_nonfinite) +
           "。检查工件是否在视场内、zmin/zmax 是否覆盖工作距离。";
    return false;
  }

  std::vector<TeachFrame> frames{TeachFrame{pts, Eigen::Isometry3d::Identity()}};
  if (!out->build_from_frames(frames, tp.voxel_m, built_ts_ns, err)) {
    return false;
  }
  if (save_path.empty()) {
    return true;
  }
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(save_path).parent_path(), ec);
  if (!out->save(save_path, err)) {
    return false;
  }
  ReferenceMap back;
  if (!back.load(save_path, err)) {
    *err = "参考地图写完读不回来（这份不能用）: " + *err;
    return false;
  }
  if (back.info().content_hash != out->info().content_hash) {
    *err = "参考地图落盘前后哈希不一致：" + std::to_string(out->info().content_hash) + " vs " +
           std::to_string(back.info().content_hash);
    return false;
  }
  LOG_AT(LogLevel::kInfo, "teach")
      << "冻结参考地图(均值 " << frames_collected << " 帧) " << save_path
      << "  原始点=" << out->info().raw_points
      << "  体素=" << out->info().map_voxels << "  voxel=" << out->info().voxel_m << " m"
      << "  hash=" << std::hex << out->info().content_hash << std::dec
      << "  ts=" << std::dec << out->info().built_ts_ns;
  return true;
}

}  // namespace follow
