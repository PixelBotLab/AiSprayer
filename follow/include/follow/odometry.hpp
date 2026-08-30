// 每帧把当前相机点云配到「示教时冻结的参考地图」上，输出相对示教位的 6DoF 修正。
//
// 与旧版（滑动窗口里程计）的三点不同，都是语义层面的：
//  1) 地图不吞实时帧 ⇒ 没有"正确位置"被工件当前姿态慢慢拖走的问题；
//  2) 输出的 T_ref_cam 是相对示教系的绝对量，不是帧间增量 ⇒ 不存在累积漂移，
//     也不需要一个 T0/invert() 的首帧特例；
//  3) 状态是枚举不是 bool：GICP "收敛"和 GICP "可信"是两件事，出包络和跟丢也
//     是两件事（前者要重新示教，后者会自己恢复）。
//
// 分工不变：NPU/CPU 特征只给平移初值和几何退化时的替补；稠密 6DoF 由 GICP 给。
// 两个解算器的位姿**不做平均、不做 R+t 拼接**，按判据二选一。
#pragma once

#include <cstdint>
#include <deque>
#include <random>
#include <vector>

#include <opencv2/core.hpp>

#include "follow/cloud.hpp"
#include "follow/matching.hpp"
#include "follow/reference_map.hpp"
#include "follow/types.hpp"
#include "follow/uncertainty.hpp"

namespace follow {

enum class Estimator {
  kNone,   // 没有可用位姿
  kGicp,   // 稠密几何
  kSparse, // 特征 3D-3D 替补（精度低一个量级，控制器要按更宽的余量用）
};

inline const char* to_string(Estimator e) {
  switch (e) {
    case Estimator::kNone: return "none";
    case Estimator::kGicp: return "gicp";
    case Estimator::kSparse: return "sparse";
  }
  return "unknown";
}

struct TrackParams {
  CameraIntrinsics k;
  double zmin_m = 0.30;
  double zmax_m = 2.50;
  int depth_stride = 4;
  double voxel_m = 0.03;

  // 对应点门。两个坑，都在 odometry.cpp 里处理了：
  //  1) small_gicp 的 align(GaussianVoxelMap&,…) 便捷函数**不会**把它写进 rejector，
  //     于是门停在 DistanceRejector 默认的 1.0 m² —— 所以这里绕开便捷函数直接组装；
  //  2) 参考地图只在 3x3x3 体素邻域内找最近邻，门大于 ~2.6·voxel_m 就够不着了，
  //     实际用的门是 min(本值, 2.6·voxel_m)。默认值按 voxel 0.03 取（6 个体素半宽）。
  double max_corr_m = 0.05;
  int threads = 4;  // 绑 A76，不要 8
  int max_iters = 15;

  SparseDeltaParams sparse;
  int min_cloud_points = 200;
  int min_gicp_inliers = 80;

  // 可观测门：任一自由度 1σ 超限、或组内各向异性超限 ⇒ kDegenerate。
  // 三个数都是在同一份数据上量出来的（合成工件 0.5 mm 深度噪声，工作距离 0.9 m，16 次
  // 独立噪声实现的蒙特卡洛；下面这行就是测试自己打印的，改门限时重跑它，别改注释）：
  //   TEST(Registration, NoiseSdMatchesPredictedSigma) / AbsoluteSigmaAloneWouldPassThePlane
  //                        σ_t (mm)                 σ_r (deg)          aniso t / r
  //   工件 mix(10,-20,30)  [0.151 0.185 0.018]      [0.006 0.004 0.029]  10.1 / 7.4
  //   工件 yaw1°           [0.081 0.094 0.010]      [0.004 0.002 0.016]   8.9 / 6.7
  //   整幅大平面 x10       [0.063 0.063 0.002]      [0.000 0.000 0.005]  31.6 / 26.7
  // 绝对门取的是"修正量还有没有用"这个物理判据：喷涂公差毫米级，所以 σ>2 mm 或 σ>0.2° 的
  // 方向上这个修正量不值得驱动手臂。好场景离门限有 10 倍余量。
  // **但注意上表第三行**：大平面的绝对 σ 比好场景还小 —— 不可观方向残差恒等于零，s² 塌得
  // 比好场景还狠（6.4e-6 对 9.0e-5），乘完之后面内 σ 只有 0.063 mm，而实测误差是 9.99 mm。
  // 绝对门对这个场景完全是瞎的，分开两种场景的只有比值（10 对 32，门限取在 15），而且它不含
  // 尺度、s² 骗不到它。平面的 rank_deficient 是 false（λ_min 没掉到 1e-9·λ_max），所以挡住的
  // 确实只有比值门 —— 这三条都写成了断言，不只是这段推导。
  // 三个门里**平移比值门余量最薄**（10.1 对 15，只有 1.5 倍）。真实扫描的表面粗糙度和
  // D2C 边缘坏点都会把它往上推，P3 必须在真点云上限位重校，不能假设这里留够了。
  double max_trans_sigma_mm = 2.0;
  double max_rot_sigma_deg = 0.2;
  double max_group_anisotropy = 15.0;

  // s² 的下限。残差恒为零时（重放示教帧本身、两张完全相同的合成点云）s²→0，σ 会一起塌成
  // 0 —— 那是"精确到无限"的假证据。1e-6 即最多允许把 σ 压到 sqrt(H⁺) 的 1/1000；实测正常
  // 工位 s² 落在 2e-5 ~ 9e-5，连真大平面也有 6e-6，所以这一档只在病态情形下才咬得住。
  double min_residual_var_scale = 1e-6;

  // 包络门：与参考几何的重叠比例低于此值 ⇒ 出了示教范围，要重新示教，不是跟丢。
  double min_inlier_ratio = 0.30;

  // 稀疏替补的连击上限（帧）。sparse_delta 测的是「相对上一帧」的运动，只靠它就把
  // 一条递推链攒起来了 —— 而那正是冻结地图要消灭的无界漂移。超过这个帧数仍没有稠密
  // 确认，就不再接受递推位姿，报 kLost 让上层知道该重新示教/检查遮挡。
  // 15 帧 @15fps = 1 s：够穿过一次抖动/短暂遮挡，不够悄悄漂走。
  int max_sparse_streak = 15;

  // 陀螺缓冲与可信窗口（P4 之前不接实机数据，接口和单测先留着）。
  size_t gyro_buf_max = 4096;
  int64_t gyro_max_gap_ns = 100'000'000;
};

struct TrackResult {
  Status status = Status::kConfigInvalid;
  Estimator estimator = Estimator::kNone;
  // kOk / kDegenerate：本帧的解。kDegenerate 表示至少一个自由度本帧没被测量到
  //   （σ 超限），它留在初值上而不是跟着观测走 —— 上层若不接受"部分可观测"，就把它当故障。
  // 其余状态：上一个可信位姿，保持不变。也就是说"故障时保持上一目标"是默认行为，
  //   不需要调用方另外实现。
  Eigen::Isometry3d T_ref_cam = Eigen::Isometry3d::Identity();
  // 只有 estimator == kGicp 时是本次求解的协方差。其它情况刻意置为 solver_failed +
  // 全 +inf：让 `within(...)` 恒为 false，避免"默认 0 看起来像很准"把稀疏解骗过去。
  Uncertainty unc;
  int sparse_inliers = 0;
  int gicp_inliers = 0;
  // GICP 的总加权代价。运维上它比 σ 更好用：除以 inlier 数就是"平均每条对应点的
  // 马氏残差"，正常应远小于 1（见 uncertainty.hpp 里 s² 的说明），变大说明场景在漂
  // 或者参考地图已经不对应当前工件了，而这两个都不会体现在收敛标志上。
  double gicp_cost = 0.0;
  size_t cloud_points = 0;     // 体素降采样后真正参与配准的点数（没跑到那一步时是原始点数）
  double inlier_ratio = 0.0;   // gicp_inliers / cloud_points —— 包络判据
  int iterations = 0;
  bool converged = false;
  bool gyro_used = false;      // 只在稀疏解缺席时顶上来当旋转初值，见 odometry.cpp
};

class Tracker {
 public:
  // seed 固定 ⇒ RANSAC 可复现；测试里显式换种子做蒙特卡洛。
  explicit Tracker(TrackParams p, const ReferenceMap& map, uint32_t seed = 0x5EEDu);

  // 336L 陀螺：与图像同一硬件时钟，rad/s，相机系。P4 之前没有调用方。
  void push_gyro(int64_t ts_ns, const Eigen::Vector3d& omega_cam_rad_s);

  // curr 的 uv_px 必须是与 depth_mm 同一套内参下的全分辨率像素坐标。
  TrackResult track(const FeatureFrame& curr, const cv::Mat& depth_mm, int64_t ts_ns);

  // 最后一个可信位姿（示教系 ← 相机）。
  Eigen::Isometry3d T() const { return T_last_good_; }
  const TrackParams& params() const { return p_; }

 private:
  TrackResult track_impl(const FeatureFrame& curr, const cv::Mat& depth_mm, int64_t ts_ns);

  TrackParams p_;
  const ReferenceMap& map_;
  std::mt19937 rng_;

  bool have_frame_ = false;
  FeatureFrame prev_sp_;
  cv::Mat prev_depth_;  // clone，不是浅引用：取流层会复用同一块深度缓冲
  int64_t prev_ts_ns_ = 0;
  std::deque<GyroSample> gyro_;

  Eigen::Isometry3d T_last_good_ = Eigen::Isometry3d::Identity();  // 对外报告的可信位姿
  Eigen::Isometry3d T_vel_ = Eigen::Isometry3d::Identity();        // 帧间运动（常数外推用）
  int sparse_streak_ = 0;  // 连续没有稠密确认的帧数
};

}  // namespace follow
