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
#include "follow/gyro_filter.hpp"
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

  // 陀螺缓冲与可信窗口。陀螺在本库里有三个用途，优先级从高到低：
  //  ① 帧间旋转初值的第二档降级（见 track_impl，sparse 缺席时顶上）；
  //  ② P3 离群门：帧间旋转与陀螺积分互验，不一致的疑似坏帧不采纳（gyro_rot_gate_deg）；
  //  ③ P1 静止检测（still_det_）：相机没在转时旋转噪声被上层冻住，见 gyro_filter.hpp。
  // **①②依赖陀螺样本与帧时间戳同一时间域**：336L 的 getTimeStampUs 是"自开机 µs"（实测末值
  // 1.57e9 µs）。服务路径必须把帧和陀螺都经 GyroTimeBase::toHostNs 换到同一主机域；若帧仍用
  // 到达时刻、陀螺按最小 USB 延迟换算，①② 会静默全灭（积分窗口一个样本都框不进来）。这里只留
  // tripwire：TrackResult::gyro_samples 恒为 0 而 gyro_buf 非空，就是那种失效的唯一外部痕迹。
  size_t gyro_buf_max = 4096;         // 只作为乱序/异常时的兜底上限
  int64_t gyro_max_gap_ns = 100'000'000;
  // 样本保留跨度 = 离群门允许判定的最长区间，**刻意是同一个数**：留得比门的区间短，门要积分
  // 的那段样本已经被裁掉 ⇒ 门永远走"没样本所以不判"的退路，静默失效且无从发现。
  // 1 s @200Hz = 200 条，代价可忽略；上界则受零偏漂移限制（几秒窗口里零偏本身就是门限量级）。
  int64_t gyro_horizon_ns = 1'000'000'000;

  // P3 离群门（度）：采纳帧的帧间旋转与**同一时间区间**陀螺积分的测地线距离超过动态门限
  // ⇒ kRotGated。0 = 关闭。
  //
  // 门限不是常数，而是 `本值 + gyro_rot_gate_relax × 该区间陀螺积分转角`。理由是真机实测：
  // 绝对门限和帧间转角量级根本不同阶 —— 15 fps 下单帧 66 ms，115 °/s 也只转 7.6°，任何视觉解
  // 都落在旧配的 10° 以内 ⇒ 手持旋转到 115 °/s 全程 0 次触发，两帧 lost 全靠 GICP 自己失败才发现
  // （见 docs/follow_review_20260831.md §13.3）。绝对门限要么低速误伤、要么高速惰性，只能二者
  // 择一；跟着转角缩放才同时成立：静止/低速时收到 2° 以内照拦，快速转动时同步放宽不误伤。
  // 下限取 2.0°：正常帧两者差在噪声级（0.0 几度），一次真滑坡/坏帧至少是度级；退化场景里 GICP
  // 的弱方向旋转留在初值上不动（见 track_impl 注释），与陀螺仍一致，不会误伤。
  // 区间必须两边一致（见 track_impl 的 rot_gate）：视觉量的是"距上次采纳"，陀螺就得积分
  // 同一段，否则一次误拦会自我放大成永久锁死。
  double gyro_rot_gate_deg = 2.0;

  // 动态项斜率（°/°）：本区间陀螺每转 1°，门限额外放宽这么多。
  // 0.5 = 允许视觉与陀螺的旋转差达到"实际转角的 50% + 下限"。留这个斜率而不是收得更紧，
  // 是因为高速下两边确有可算的失配来源（时间基准残余 δt×角速度：115 °/s × 10 ms ≈ 1.1°，
  // 已占去 7.6° 转角的 15%），0.5 给了约 3 倍余量；同时它仍远低于 1.0 —— 斜率取到 1.0 就等于
  // 回到旧 10° 门的惰性状态。真机分布要靠逐帧日志里的 gate=err/limit 复核后再定值。
  double gyro_rot_gate_relax = 0.5;

  // P1 静止检测（冻结旋转通道由消费方实施，这里只负责判"静没静"）。
  GyroStillDetector::Params gyro_still;

  // 一次静止冻结的最长时限（毫秒，0 = 不限）。这是"恒定慢转与恒定零偏不可区分"的兜底：
  // 检测器的零偏只在安静期更新，但再谨慎也架不住一段足够长的匀速转动被慢慢当成安静。
  // 有这个上限，误冻的损害就是 `exit 门限 × 上限` 的有界台阶，解冻后靠平滑窗口自愈；
  // 没有它，误判可以无限期挂着旋转输出。由消费方实施（本库不认墙上时钟）。
  int gyro_max_freeze_ms = 1500;
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
  // 分阶段耗时（ms）。只有聚合数的时候"变慢了"是一次无法归因的观测：建云随点数走、
  // 稀疏回退只在特征退化时贵、GICP 随迭代数走 —— 三条的处置完全不同，所以必须分开报。
  // 没跑到的阶段保持 0（"没跑"和"跑得飞快"必须能区分开）。
  double cloud_ms = 0.0;
  double sparse_ms = 0.0;
  double dense_ms = 0.0;
  bool gyro_used = false;      // 只在稀疏解缺席时顶上来当旋转初值，见 odometry.cpp
  bool gyro_still = false;     // 陀螺确认相机没在转：消费方据此冻住旋转通道（P1）
  bool rot_gated = false;      // 本帧被 P3 离群门拦下：帧间旋转与陀螺积分不一致，未采纳
  double rot_gate_err_deg = 0.0;  // 互验的测地线距离，超门才拦；日志与调参看这个数
  // 本帧实际生效的门限（= 下限 + 斜率 × 该区间陀螺转角）。门限不再是常数，只报 err 无法复核
  // 判据：err=3° 在静止帧是明显离群、在 115 °/s 帧是噪声级。0 = 本帧没走到判据（门关/区间非法/无样本）。
  double rot_gate_limit_deg = 0.0;
  // 陀螺诊断字段。存在的唯一理由：陀螺的三种用途全都可能因为"时间域不一致 / IMU 停更 /
  // 零偏没估出来"而静默变成没生效，而那时**其余一切看起来都正常**。消费方必须把这几个数
  // 报出去（见 FollowSnapshot 的同名字段）。
  int gyro_samples = 0;        // 本帧积分窗口内真正用到的样本数（0 可能是时间域错，也可能只是交付滞后）
  int gyro_buf = 0;            // 陀螺缓冲当前长度（按 gyro_horizon_ns 裁剪后）
  int gyro_pushed = 0;         // 本帧收到的新样本数：持续为 0 ⇒ IMU 停更/没起流（与时间戳无关）
  // 积分被真实样本覆盖的跨度、"末样本到本帧时刻"的缺口、以及其中被常值外推补掉的时长。
  // 存在的理由：S/P<1 有两种完全不同的成因 —— "窗口里的样本被跳过"（span 仍≈帧周期，只是
  // 节点稀）与"末段样本还没到货"（span 明显短、gap 就是缺失量）。真机实测是后者：66ms 窗口
  // 固定缺尾部 33ms（IMU 交付比帧交付慢半帧）。不补就会让 R_init 与离群门少转半帧，
  // 所以 integrate_gyro 现在补积，extrap_ns 即补的量：span+extrap≈帧周期才是健康态。
  int64_t gyro_span_ns = 0;
  int64_t gyro_gap_end_ns = 0;
  int64_t gyro_extrap_ns = 0;
  double gyro_bias_rad_s = 0.0;  // 静止检测器当前的零偏估计（模）
  double gyro_resid_rad_s = 0.0; // 最近窗口的残差模均值，与 enter/exit 门限直接可比
  bool gyro_bias_ready = false;  // 零偏还在 bootstrap 阶段 ⇒ gyro_still 尚不可信
};

class Tracker {
 public:
  // seed 固定 ⇒ RANSAC 可复现；测试里显式换种子做蒙特卡洛。
  explicit Tracker(TrackParams p, const ReferenceMap& map, uint32_t seed = 0x5EEDu);

  // 336L 陀螺：rad/s，相机系。**要求与 track() 的 ts_ns 同一时间域**（服务路径：帧与陀螺都走
  // GyroTimeBase::toHostNs）；不同域时这里不会崩，但积分窗口框不到任何样本，①② 两档静默失效。
  // 同一份样本也喂给静止检测器（它只用向量、不碰时间轴），所以两个消费方永远吃同一批数据。
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
  // T_last_good_ 属于哪一帧。离群门拿"视觉相对上次采纳的旋转"去比，陀螺就必须积分同一段
  // （last_good_ts_ns_ → 当前帧），不能只积一帧 —— 中间的帧被 hold 过就会两边区间不等长，
  // 差值随被拦帧数线性增长 ⇒ 一次误拦自我放大成永久锁死。
  int64_t last_good_ts_ns_ = 0;
  bool have_last_good_ts_ = false;
  int sparse_streak_ = 0;  // 连续没有稠密确认的帧数
  int gyro_this_frame_ = 0;  // push_gyro 累加、track() 归零：与时间戳无关的 IMU 存活计数
  GyroStillDetector still_det_;  // 与 gyro_ 同源样本；构造参数来自 TrackParams::gyro_still
};

}  // namespace follow
