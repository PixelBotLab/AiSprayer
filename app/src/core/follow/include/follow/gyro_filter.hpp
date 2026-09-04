// 陀螺「静止检测器」：判断相机此刻是不是没在转。
//
// 为什么需要它（P1 静止冻结）：静止时旋转通道的全部抖动来自 GICP 的单帧估计噪声
//（单帧 σ_r ~0.005°，但退化场景能大一到两个量级）。陀螺对"没在转"的判断独立于视觉：
// 一旦确认静止，旋转输出就冻在冻结点上一动不动，平移照常更新 —— 这比加宽平滑窗口划算，
// 因为窗口对"真运动"和"噪声"一视同仁，而陀螺分得清。
//
// 设计约束，都是吃过亏的口径：
//  * **只对样本序列负责，不碰绝对时间**。服务路径里帧与陀螺曾经各走各的钟（设备 µs vs 主机
//    到达时刻，见 gyro_time_base.hpp），两套钟混着用就是悄悄的对齐错误；检测器只需要"最近
//    这一小段的抖动有多大"，与时间轴无关，因此对那类错误免疫。
//  * **判据是"减掉零偏之后的残差模"，不是 |ω| 本身**。336L 实测静止时 |ω| ≈ 0.99°/s 且
//    峰值≈均值 —— 那是恒定零偏，不是抖动（真运动不会连续几秒保持同一个值）。直接对 |ω| 设门
//    等于把门限架在零偏之上：本板零偏 0.99°/s，而旧门限 1.15°/s 只剩 16% 余量，零偏一随温度
//    漂就静默失效。减掉零偏后门限能压到噪声级，慢速转动也才因此可辨。
//    ⚠ 极限说清楚：恒定零偏与恒定角速度在数学上是同一个信号，没有任何瞬时判据分得开。
//      这里靠"零偏只在静止期更新"（Phase 2 的门控）避免把运动吸进零偏，而**剩余的兜底在
//      消费方**：FollowWorker 的最长冻结时限（gyro_max_freeze_ms），它把"慢速匀速转动被误判
//      为静止"的损害从无上界变成有界且自愈。
//  * **迟滞**：进入静止要连续 confirm 个窗口低于 enter 门；退出只要一个窗口高于 exit 门
//    （exit > enter）。不对称是有意的：迟进快出 —— 宁可晚冻 100 ms，也不能在相机已经动了
//    之后还冻着旋转。
//  * **没攒够样本之前不表态**。valid() 为假时调用方必须当成"不知道"，而不是"静止"：
//    上电头 100 ms 就报静止，会把真正的初始运动冻掉。
//
// 单位：rad/s（与 GyroSample 一致，跨界才换度 —— 见 pose_io.hpp 的规矩）。
#pragma once

#include <cmath>
#include <cstdint>
#include <deque>

#include <Eigen/Core>

namespace follow {

class GyroStillDetector {
 public:
  struct Params {
    // 下面两个门限针对的是**减掉零偏后的残差模**，不是 |ω| 本身。
    double enter_rad_s = 0.008;  // ~0.46°/s：残差低于它算"没在转"
    double exit_rad_s = 0.017;   // ~1.0°/s：残差高于它立即退出静止。必须 > enter（迟滞）
    int window_samples = 20;     // 均值窗口。@200Hz = 100 ms，够滤掉单样本毛刺
    int confirm_samples = 20;    // 连续多少样本低于 enter 才宣布静止。@200Hz = 100 ms

    // 零偏两阶段：Phase 1 用前 bias_bootstrap_samples 个样本取累积均值（一步到位），
    // Phase 2 只在"残差已经低于 enter"时用 α 很小的 EMA 缓慢跟温度漂移。
    // 门控是刻意的：不门控则一段匀速转动会被 EMA 吸进零偏，转完就再也判不出静止了。
    int bias_bootstrap_samples = 1000;  // @200Hz = 5 s。示教流程本就先静置相机
    double bias_alpha = 0.001;          // τ ≈ 1000 样本 = 5 s：只跟热漂移，不跟运动
  };

  GyroStillDetector() : GyroStillDetector(Params{}) {}
  // 注意不能写成 (Params p = Params())：Params 的默认成员初始化器在嵌套类完结前
  // 不能被同层的默认实参引用，GCC 会当场报错。
  explicit GyroStillDetector(Params p) : p_(p) {
    // 参数自洽在构造期钉死，不留给运行期猜：窗口/确认长度为正，迟滞方向正确。
    if (p_.window_samples < 1) p_.window_samples = 1;
    if (p_.confirm_samples < 1) p_.confirm_samples = 1;
    if (p_.bias_bootstrap_samples < 1) p_.bias_bootstrap_samples = 1;
    if (p_.bias_alpha <= 0.0 || p_.bias_alpha > 1.0) p_.bias_alpha = 0.001;
    if (p_.exit_rad_s < p_.enter_rad_s) p_.exit_rad_s = p_.enter_rad_s;
  }

  // 喂一个陀螺样本（相机系角速度）。非有限值直接不计：一个 NaN 样本会毒化窗口均值，而
  // "均值是 NaN"能通过一切大小比较；更糟的是它会进零偏 EMA 并永久留在里面。
  void push(const Eigen::Vector3d& omega_rad_s) {
    if (!omega_rad_s.allFinite()) {
      return;
    }
    ++total_samples_;

    // 残差用**更新前**的零偏算：一个样本不该影响对自己这一拍的判据 —— 否则一次突发的大角
    // 速度会先把零偏拉过去、再把自己冲销成"静止"。
    const double resid = (omega_rad_s - bias_).norm();
    resid_.push_back(resid);
    while (static_cast<int>(resid_.size()) > p_.window_samples) {
      resid_.pop_front();
    }

    // --- 零偏两阶段（见类头注释）---
    if (bias_n_ < p_.bias_bootstrap_samples) {
      bias_ += (omega_rad_s - bias_) / static_cast<double>(++bias_n_);
    } else if (quiet_) {
      bias_ += (omega_rad_s - bias_) * p_.bias_alpha;
    }

    if (static_cast<int>(resid_.size()) < p_.window_samples) {
      return;  // 窗口没攒满：不表态
    }
    valid_ = true;
    double sum = 0.0;
    for (double v : resid_) sum += v;
    const double avg = sum / static_cast<double>(resid_.size());
    recent_resid_rad_s_ = avg;
    // 门控学习与迟滞用的都是这一位。取窗口平均而不是本样本：单点毛刺不该关掉零偏学习。
    quiet_ = (avg < p_.enter_rad_s);

    if (still_) {
      if (avg > p_.exit_rad_s) {
        still_ = false;   // 快出：相机动了，旋转通道立刻解冻
        confirm_ = 0;
      }
    } else {
      if (quiet_) {
        if (++confirm_ >= p_.confirm_samples) {
          still_ = true;  // 慢进：连续确认才冻
        }
      } else {
        confirm_ = 0;
      }
    }
  }

  // 最近窗口均值够不够长。为假时 still() 的返回值没有意义。
  bool valid() const { return valid_; }
  bool still() const { return valid_ && still_; }

  // 诊断数：「功能没生效」和「生效了但场景不对」是两种完全不同的处置，必须分得开。
  double recent_resid_rad_s() const { return valid_ ? recent_resid_rad_s_ : 0.0; }
  double bias_rad_s() const { return bias_.norm(); }
  const Eigen::Vector3d& bias_vec() const { return bias_; }
  // 零偏还在累积均值阶段（未走完 bootstrap）⇒ 静止结论不可信，消费方应当再等。
  bool bias_ready() const { return bias_n_ >= p_.bias_bootstrap_samples; }
  uint64_t total_samples() const { return total_samples_; }

  // 换档/换图/重启跟踪时调用：旧档位的"静止"结论不属于新档位，零偏也是旧时钟段的。
  void reset() {
    resid_.clear();
    bias_.setZero();
    bias_n_ = 0;
    still_ = false;
    valid_ = false;
    confirm_ = 0;
    quiet_ = false;
    recent_resid_rad_s_ = 0.0;
    total_samples_ = 0;
  }

 private:
  Params p_;
  std::deque<double> resid_;       // 窗口：|ω − bias| 的样本序列
  Eigen::Vector3d bias_ = Eigen::Vector3d::Zero();
  int bias_n_ = 0;
  bool still_ = false;
  bool valid_ = false;
  bool quiet_ = false;             // 最近一个窗口是否低于 enter（迟滞与零偏门控共用）
  int confirm_ = 0;
  double recent_resid_rad_s_ = 0.0;
  uint64_t total_samples_ = 0;
};

}  // namespace follow
