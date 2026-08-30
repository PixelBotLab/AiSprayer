#include "follow/reference_map.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <utility>

#include <small_gicp/registration/registration_helper.hpp>

namespace follow {
namespace {

constexpr uint32_t kMagic = 0x50414D52;  // 小端字节序：'R','M','A','P'
constexpr uint32_t kFormatVersion = 1;
constexpr size_t kMaxScans = 64;
constexpr size_t kMaxPoints = 8'000'000;  // 单帧和总量的上限：load 里逐帧读时也要按累计值卡
constexpr int kNormalNeighbors = 10;  // 与 odometry.cpp 的源点处理保持一致，两边的协方差量纲才对得上

uint64_t fnv1a(const char* p, size_t n, uint64_t h) {
  for (size_t i = 0; i < n; ++i) {
    h ^= static_cast<uint8_t>(p[i]);
    h *= 0x100000001b3ULL;
  }
  return h;
}

template <typename T>
void put_le(std::vector<char>& buf, const T& v) {
  const char* p = reinterpret_cast<const char*>(&v);
  buf.insert(buf.end(), p, p + sizeof(T));
}

template <typename T>
bool get_le(const std::vector<char>& buf, size_t& off, T& v) {
  if (off + sizeof(T) > buf.size()) {
    return false;
  }
  std::memcpy(&v, buf.data() + off, sizeof(T));
  off += sizeof(T);
  return true;
}

}  // namespace

ReferenceMap::ReferenceMap() = default;
ReferenceMap::~ReferenceMap() = default;
ReferenceMap::ReferenceMap(ReferenceMap&&) = default;
ReferenceMap& ReferenceMap::operator=(ReferenceMap&&) = default;

const small_gicp::GaussianVoxelMap& ReferenceMap::voxel_map() const { return *map_; }
const small_gicp::PointCloud& ReferenceMap::points() const { return *points_; }

bool ReferenceMap::build_from_frames(const std::vector<TeachFrame>& frames, double voxel_m,
                                     int64_t built_ts_ns, std::string* error) {
  std::vector<std::vector<Eigen::Vector3f>> scans;
  scans.reserve(frames.size());
  for (const auto& f : frames) {
    std::vector<Eigen::Vector3f> w;
    w.reserve(f.cam_pts.size());
    for (const auto& p : f.cam_pts) {
      if (!p.allFinite()) {
        continue;
      }
      w.push_back((f.T_ref_cam * p.cast<double>()).cast<float>());
    }
    scans.push_back(std::move(w));
  }
  return build(scans, voxel_m, built_ts_ns, error);
}

bool ReferenceMap::build(const std::vector<std::vector<Eigen::Vector3f>>& scans_ref, double voxel_m,
                         int64_t built_ts_ns, std::string* error) {
  auto fail = [&](const std::string& msg) {
    map_.reset();
    points_.reset();
    scans_.clear();
    info_ = ReferenceMapInfo();
    if (error) {
      *error = msg;
    }
    return false;
  };

  if (!(voxel_m > 0.0) || !std::isfinite(voxel_m)) {
    return fail("voxel_m 必须是正的有限值，收到 " + std::to_string(voxel_m));
  }
  if (scans_ref.empty() || scans_ref.size() > kMaxScans) {
    return fail("示教帧数量不合法: " + std::to_string(scans_ref.size()));
  }

  std::vector<Eigen::Vector3f> merged;
  size_t total = 0;
  for (const auto& s : scans_ref) {
    total += s.size();
  }
  if (total == 0 || total > kMaxPoints) {
    return fail("参考点数不合法: " + std::to_string(total));
  }
  merged.reserve(total);
  for (const auto& s : scans_ref) {
    merged.insert(merged.end(), s.begin(), s.end());
  }
  if (merged.size() < 100) {
    return fail("参考点太少（" + std::to_string(merged.size()) + "），撑不起一个基准地图");
  }

  // 必须先 preprocess_points 再建图：GaussianVoxel::add 累加的是**输入点自己的协方差**
  // （cov += T·cov(points,i)·Tᵀ），把裸点云直接塞进去会让每个体素的高斯是零矩阵，
  // 而 GICPFactor 用的是普通 3x3 .inverse()（不是伪逆）—— 零协方差当场变 NaN 位姿。
  // 降采样分辨率取体素尺寸：一 voxel 一个高斯，既不重复也不漏。
  // 一次成型：IncrementalVoxelMap 每 insert 10 次会做一次 LRU 淘汰（horizon=100），
  // 分多批 insert 会把先放进去的体素悄悄删掉。所以这里合并成一份点、只 insert 一次。
  auto pre = small_gicp::preprocess_points(merged, voxel_m, kNormalNeighbors, 1);
  const small_gicp::PointCloud::Ptr& cloud = pre.first;
  if (!cloud || cloud->size() < 100) {
    return fail("参考点降采样后只剩 " + std::to_string(cloud ? cloud->size() : 0) +
                " 个：体素尺寸相对场景太大");
  }
  auto map = small_gicp::create_gaussian_voxelmap(*cloud, voxel_m);
  if (map == nullptr || map->size() == 0) {
    return fail("GaussianVoxelMap 构建后为空：体素尺寸相对场景太大？");
  }
  // 关键：默认 search_offsets=1，只查查询点自己那一个 voxel。于是初值只要偏出超过
  // 一个体素，最近邻就返回"没有邻居"，GICP 会拿着 0 个对应点原地"收敛"—— 看起来
  // 像场景不重叠（kOutOfEnvelope），实际是搜索结构够不着。27 = 3x3x3 邻域。
  map->set_search_offsets(27);

  // 内容哈希只对点数据取，体素尺寸不进哈希 —— 同一份点换个 voxel 仍是同一份基准。
  uint64_t h = 1469598103934665603ULL;
  for (const auto& s : scans_ref) {
    h = fnv1a(reinterpret_cast<const char*>(s.data()), s.size() * sizeof(Eigen::Vector3f), h);
  }

  points_ = pre.first;
  map_ = std::move(map);
  scans_ = scans_ref;
  info_ = ReferenceMapInfo();
  info_.raw_points = merged.size();
  info_.map_voxels = map_->size();
  info_.scans = scans_.size();
  info_.voxel_m = voxel_m;
  info_.built_ts_ns = built_ts_ns;
  info_.content_hash = h;
  if (error) {
    error->clear();
  }
  return true;
}

bool ReferenceMap::save(const std::string& path, std::string* error) const {
  if (empty()) {
    if (error) {
      *error = "地图是空的，没什么可存";
    }
    return false;
  }
  std::vector<char> buf;
  put_le(buf, kMagic);
  put_le(buf, kFormatVersion);
  put_le(buf, info_.voxel_m);
  put_le(buf, info_.built_ts_ns);
  const uint32_t n_scans = static_cast<uint32_t>(scans_.size());
  put_le(buf, n_scans);
  for (const auto& s : scans_) {
    const uint32_t n = static_cast<uint32_t>(s.size());
    put_le(buf, n);
    const char* p = reinterpret_cast<const char*>(s.data());
    buf.insert(buf.end(), p, p + static_cast<size_t>(n) * sizeof(float) * 3);
  }
  uint64_t h = 1469598103934665603ULL;
  for (const auto& s : scans_) {
    h = fnv1a(reinterpret_cast<const char*>(s.data()), s.size() * sizeof(Eigen::Vector3f), h);
  }
  put_le(buf, h);

  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f) {
    if (error) {
      *error = "打不开写入路径: " + path;
    }
    return false;
  }
  f.write(buf.data(), static_cast<std::streamsize>(buf.size()));
  f.close();
  if (!f) {
    if (error) {
      *error = "写入未完成: " + path;
    }
    return false;
  }
  if (error) {
    error->clear();
  }
  return true;
}

bool ReferenceMap::load(const std::string& path, std::string* error) {
  auto fail = [&](const std::string& msg) {
    map_.reset();
    points_.reset();
    scans_.clear();
    info_ = ReferenceMapInfo();
    if (error) {
      *error = msg;
    }
    return false;
  };

  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) {
    return fail("打不开地图文件: " + path);
  }
  const std::streamoff sz = f.tellg();
  if (sz <= 0 || static_cast<size_t>(sz) < 4 * sizeof(uint32_t) + sizeof(uint64_t)) {
    return fail("地图文件过小: " + std::to_string(static_cast<long long>(sz)) + " 字节");
  }
  f.seekg(0);
  std::vector<char> buf(static_cast<size_t>(sz));
  f.read(buf.data(), sz);
  if (!f) {
    return fail("地图文件读取失败: " + path);
  }

  size_t off = 0;
  uint32_t magic = 0, version = 0;
  if (!get_le(buf, off, magic) || magic != kMagic) {
    return fail("magic 不对（字节序或格式变了）");
  }
  if (!get_le(buf, off, version) || version != kFormatVersion) {
    return fail("地图格式版本 " + std::to_string(version) + " 与本工程（" +
                std::to_string(kFormatVersion) + "）不匹配，重新示教而不是猜兼容");
  }
  double voxel_m = 0.0;
  int64_t ts = 0;
  uint32_t n_scans = 0;
  if (!get_le(buf, off, voxel_m) || !get_le(buf, off, ts) || !get_le(buf, off, n_scans)) {
    return fail("地图头部被截断");
  }
  if (n_scans == 0 || n_scans > kMaxScans) {
    return fail("地图里的示教帧数量不合法: " + std::to_string(n_scans));
  }

  std::vector<std::vector<Eigen::Vector3f>> scans;
  scans.reserve(n_scans);
  size_t read_points = 0;
  for (uint32_t i = 0; i < n_scans; ++i) {
    uint32_t n = 0;
    if (!get_le(buf, off, n)) {
      return fail("第 " + std::to_string(i) + " 帧长度字段不合法");
    }
    read_points += n;
    if (n > kMaxPoints || read_points > kMaxPoints) {
      return fail("第 " + std::to_string(i) + " 帧点数越界（累计 " + std::to_string(read_points) +
                  "）：长度字段不可信");
    }
    const size_t bytes = static_cast<size_t>(n) * sizeof(Eigen::Vector3f);
    if (off + bytes + sizeof(uint64_t) > buf.size()) {
      return fail("第 " + std::to_string(i) + " 帧点数据越界：文件被截断");
    }
    // 目标类型写成 float* 而不是 Vector3f*：Eigen 的向量类不是 trivially
    // copy-assignable，直接 memcpy 会触发 -Wclass-memaccess（内存布局上确实连续）。
    std::vector<Eigen::Vector3f> s(n);
    std::memcpy(reinterpret_cast<float*>(s.data()), buf.data() + off, bytes);
    off += bytes;
    scans.push_back(std::move(s));
  }
  uint64_t stored_hash = 0;
  if (!get_le(buf, off, stored_hash) || off != buf.size()) {
    return fail("哈希字段位置不对，文件多余或不足");
  }
  uint64_t h = 1469598103934665603ULL;
  for (const auto& s : scans) {
    h = fnv1a(reinterpret_cast<const char*>(s.data()), s.size() * sizeof(Eigen::Vector3f), h);
  }
  if (h != stored_hash) {
    return fail("内容哈希不符：文件损坏或被人改过，拒绝当基准用");
  }

  std::string build_err;
  if (!build(scans, voxel_m, ts, &build_err)) {
    return fail("地图重建失败: " + build_err);
  }
  if (error) {
    error->clear();
  }
  return true;
}

}  // namespace follow
