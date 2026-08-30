// 示教时冻结下来的参考几何。运行期只读。
//
// 为什么是「冻结」而不是滑动窗口里程计：工位跟存在的意义是「相对示教位修正」。
// 地图若持续吞实时帧，"正确位置"就被慢慢拖向工件此刻的样子 —— 修正量随时间自发
// 衰减到零，而且没有上界。那是喷涂质量问题，攒久了是安全问题。代价写清楚：可用
// 范围受限于与参考几何的重叠度，出了包络必须报 kOutOfEnvelope 并要操作员重新
// 示教，绝不能自动把当前帧当新基准悄悄续上。
//
// 地图所在的坐标系就叫「参考系」：build_from_frames 里第一帧的 T_ref_cam 决定它。
// 追踪器输出的 T_ref_cam 直接就是相对示教位的修正量。
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

// small_gicp 的 GaussianVoxelMap 是模板别名（using GaussianVoxelMap =
// IncrementalVoxelMap<GaussianVoxel>），没法前向声明，只能在这里带进来。
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/points/point_cloud.hpp>

namespace follow {

// 一帧示教数据：相机系点云（米）+ 它在参考系里的位姿。
struct TeachFrame {
  std::vector<Eigen::Vector3f> cam_pts;
  Eigen::Isometry3d T_ref_cam = Eigen::Isometry3d::Identity();
};

struct ReferenceMapInfo {
  size_t raw_points = 0;      // 合并后、体素前的点数
  size_t map_voxels = 0;      // 高斯体素数
  size_t scans = 0;
  double voxel_m = 0.0;
  int64_t built_ts_ns = 0;
  uint64_t content_hash = 0;  // 对点数据本身取哈希，重启后能确认是同一份基准
};

class ReferenceMap {
 public:
  ReferenceMap();
  ~ReferenceMap();
  ReferenceMap(ReferenceMap&&);
  ReferenceMap& operator=(ReferenceMap&&);
  ReferenceMap(const ReferenceMap&) = delete;
  ReferenceMap& operator=(const ReferenceMap&) = delete;

  // 构建是串行的（small_gicp 的 create_gaussian_voxelmap 本身就单线程），所以同一份
  // 输入必然得到同一份地图 —— 不需要为了复现性去传 threads=1。示教是离线动作，
  // 实测 6 万点在毫秒级，不值得为它开并行分支（并行体素降采样还会引入 run-to-run 差异）。
  bool build_from_frames(const std::vector<TeachFrame>& frames, double voxel_m,
                         int64_t built_ts_ns, std::string* error);
  bool build(const std::vector<std::vector<Eigen::Vector3f>>& scans_ref, double voxel_m,
             int64_t built_ts_ns, std::string* error);

  bool empty() const { return map_ == nullptr; }
  const small_gicp::GaussianVoxelMap& voxel_map() const;
  // 降采样并估完协方差后的参考点云 —— 地图真正编码的那份几何。
  const small_gicp::PointCloud& points() const;
  // 变换到参考系、未降采样的原始示教点（落盘的就是这份）。
  const std::vector<std::vector<Eigen::Vector3f>>& scans() const { return scans_; }
  double voxel_m() const { return info_.voxel_m; }
  const ReferenceMapInfo& info() const { return info_; }

  // 落盘/读回。格式带 magic + version + 内容哈希：换了工件、换了体素尺寸、文件被
  // 截断，都要在加载时立刻报出来，而不是悄悄换个基准跑起来。
  bool save(const std::string& path, std::string* error) const;
  bool load(const std::string& path, std::string* error);

 private:
  std::shared_ptr<small_gicp::PointCloud> points_;
  std::shared_ptr<small_gicp::GaussianVoxelMap> map_;
  std::vector<std::vector<Eigen::Vector3f>> scans_;
  ReferenceMapInfo info_;
};

}  // namespace follow
