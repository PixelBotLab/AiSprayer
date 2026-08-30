// P3 验收工具的解析场景：真实工件网格 + 三角 BVH 光线求交 → 真值由构造保证的合成序列。
//
// 这是**验收工具**，不是产品代码：不链进 libfollow，运行期一行都不碰。它存在的理由是
// 这台机器上没有相机 —— 而"能算对"这件事必须在这一轮就被证明，不能等硬件。网格用磁盘上
// 那份真扫描件（24937 顶点 / 48915 三角，机器人基座系，米），内参用那份真标定值，所以
// 场景的尺度、纹理频率、遮挡关系都是实的，只有"动"是命令出来的。
//
// 约定（和 TrackResult::T_ref_cam 同一套，别反过来）：T_ref_cam 是**相机在参考系里的位姿**，
// 把相机系坐标映进参考系。参考系里的几何静止，因此"工件动了"与"相机动了"在纯视觉下是同
// 一件事，跟着示教位算出来的修正量就是那个差。
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "follow/types.hpp"

namespace follow {
namespace synth {

struct Mesh {
  std::vector<Eigen::Vector3f> verts;
  std::vector<std::array<int32_t, 3>> tris;
  size_t tri_count() const { return tris.size(); }
  Eigen::AlignedBox3f bbox() const;
};

// 只支持本项目真会遇到的形态：ascii 或 binary_little_endian，element vertex 带 float32
// 的 x/y/z（允许穿插其它属性，按步长跳过），element face 带 uchar 计数 + 3 个索引。
// 遇到别的形态一律报错，不猜：静默读歪一个网格，后面所有"精度"数字都没有意义。
bool load_ply(const std::string& path, Mesh* out, std::string* err);

// 三角网格的 AABB 层次（质心 median split，叶子 ≤8 面）。8 万像素 × 5 万面不能暴力求交。
class Scene {
 public:
  explicit Scene(Mesh m);

  bool valid() const { return !nodes_.empty(); }
  const Mesh& mesh() const { return mesh_; }
  const std::vector<int32_t>& leaf_triangles() const { return prim_; }

  // 参考系射线 o + t·d（d 不必单位化，返回的 t 相对该长度）。最近命中：t>0，命中点/法线可选
  // （法线是面法线，指向 d 的反侧）；未命中返回 0。t_max<=0 表示不设上界。
  double cast(const Eigen::Vector3d& o, const Eigen::Vector3d& d, Eigen::Vector3d* point = nullptr,
              Eigen::Vector3d* normal = nullptr, double t_max = 0.0) const;

 private:
  static constexpr int kLeafTris = 8;
  static constexpr int kMaxDepth = 48;

  struct Node {
    Eigen::Vector3d lo = Eigen::Vector3d::Zero();
    Eigen::Vector3d hi = Eigen::Vector3d::Zero();
    int left = -1;    // 内部节点：子节点下标
    int right = -1;
    int begin = 0;    // 叶子：prim_ 区间
    int count = 0;
  };
  // 返回新建节点的下标。递归中 nodes_ 会扩容，所以任何引用都不能跨调用持有。
  int build_subtree(int begin, int end, int depth);
  // 射线进入盒子的距离；不相交（或整体在 t_max 之外）返回 -1。inv_d 须是 1/d 且 d 无零分量。
  static double aabb_entry(const Eigen::Vector3d& o, const Eigen::Vector3d& inv_d,
                           const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, double t_max);

  Mesh mesh_;
  std::vector<Node> nodes_;
  std::vector<int32_t> prim_;  // 叶子里的三角形下标
};

struct RenderParams {
  // block×block 常数块。追踪器按 depth_stride 采样，逐像素求交只是白烧 CPU；
  // P3 里只有彩色图需要接近逐像素（特征前端要看细节），那时 block=1。
  int block = 4;
  // 加在**深度**上的高斯噪声**标准差**（毫米）。必须是标准差而不是均值：写反会得到一个
  // 恒定偏置，而地图与测试帧共用同一实现时它是共模的，配准把它整个吸掉（P2 真踩过）。
  double noise_mm = 0.5;
  int blur_samples = 1;      // >1：沿曝光期取 N 个子位姿，取最近命中
  double blur_rot_deg = 0.0;  // 曝光期绕相机自身 y 轴扫过的总角度，中心对齐命令位姿。
                              // 取最近命中而不是平均深度：平均会在轮廓处造出真实相机不可能
                              // 产生的中间值，那是在给求解器喂一个现实中不存在的错误。
  double hole_fraction = 0.0;  // 传感器丢点：按比例把像素置 0（0=无空洞）
  uint32_t seed = 20260830u;   // 固定 ⇒ 整段可复现；蒙特卡洛时显式换它
};

struct ColorParams {
  int block = 1;
  // 表面反照率的格子周期（米）。**在参考系里按位置取**，所以纹理跟着工件走 —— 贴在图像上
  // 的话特征前端会跟着一块幻灯片走，测不出真东西。<=0 退化成均匀灰：无纹理但有几何。
  double texture_period_m = 0.03;
  double ambient = 0.35;
  // 指向光源的方向（参考系）。**必须在相机这一侧**：可见面的法线被 Scene::cast 翻向相机，
  // 在参考系里 z 分量是负的，光若从背后照过来 Lambert 项会被 max(0,·) 整个夹没，场景只剩
  // ambient —— 那时"无纹理"和"有纹理"两个用例其实长得一样，测了个寂寞。
  Eigen::Vector3d light_dir_ref = Eigen::Vector3d(0.3, -0.4, -0.9).normalized();
  double noise_gray = 0.02;  // 加在归一化亮度上的传感器噪声（标准差）
  uint32_t seed = 20260830u;
};

// CV_16UC1 毫米，未命中/空洞为 0。内参非法或场景为空时返回空 Mat。
cv::Mat render_depth(const CameraIntrinsics& k, const Eigen::Isometry3d& T_ref_cam, const Scene& s,
                     const RenderParams& p = {});

// CV_8UC3 BGR。几何与 render_depth 用同一套光线，所以彩色和深度像素级对齐（D2C 已做的结果）。
cv::Mat render_color(const CameraIntrinsics& k, const Eigen::Isometry3d& T_ref_cam, const Scene& s,
                     const ColorParams& p = {});

// 把位姿拆成 (tx ty tz) 米 + 轴角 (rx ry rz)（rad，模长即角度）—— 真值文件用这种写法，
// 不必约定欧拉序列，也就不会因为一个序列假设把整份真值读歪。
std::array<double, 6> pose_to_log(const Eigen::Isometry3d& T);
Eigen::Isometry3d log_to_pose(const std::array<double, 6>& v);

}  // namespace synth
}  // namespace follow
