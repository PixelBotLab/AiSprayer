// 陀螺「静止检测器」：判断相机此刻是不是没在转。
//
// 为什么需要它（P1 静止冻结）：静止时旋转通道的全部抖动来自 GICP 的单帧估计噪声
//（单帧 σ_r ~0.005°，但退化场景能大一到两个量级）。陀螺对"没在转"的判断独立于视觉：
// 一旦确认静止，旋转输出就冻在冻结点上一动不动，平移照常更新 —— 这比加宽平滑窗口划算，
// 因为窗口对"真运动"和"噪声"一视同仁，而陀螺分得清。
//
// 设计约束，都是吃过亏的口径：
//  * **只对样本序列负责，不碰绝对时间**。服务路径里帧时间戳是主机毫秒、陀螺是硬件时钟，
//    两套钟混着用就是悄悄的对齐错误；检测器只需要"最近这一小段的模有多大"，与时间轴无关。
//  * **迟滞**：进入静止要连续 confirm 个窗口低于 enter 门；退出只需一帧超过 exit 门
//    （exit > enter）。不对称是有意的：迟进快出 —— 宁可晚冻 100 ms，也不能在相机已经动了
//    之后还冻着旋转。
//  * **没攒够样本之前不表态**。valid() 为假时调用方必须当成"不知道"，而不是"静止"：
//    上电头 100 ms 就报静止，会把真正的初始运动冻掉。
//
// 单位：rad/s（与 GyroSample 一致，跨界才换度 —— 见 pose_io.hpp 的规矩）。
#pragma once

#include <cmath>
#include <deque>

#include <Eigen/Core>

namespace follow {

class GyroStillDetector {
 public:
  struct Params {
    double enter_rad_s = 0.02;      // ~1.1°/s：低于它算"没在转"。336L 静止噪声实测远小于此
    double exit_rad_s = 0.05;       // ~2.9°/s：高于它立即退出静止。必须 > enter（迟滞）
    int window_samples = 20;        // 均值窗口。@200Hz = 100 ms，够滤掉单样本毛刺
    int confirm_samples = 20;       // 连续多少帧低于 enter 才宣布静止。@200Hz = 100 ms
  };

  GyroStillDetector() : GyroStillDetector(Params{}) {}
  // 注意不能写成 (Params p = Params())：Params 的默认成员初始化器在嵌套类完结前
  // 不能被同层的默认实参引用，GCC 会当场报错。
  explicit GyroStillDetector(Params p) : p_(p) {
    // 参数自洽在构造期钉死，不留给运行期猜：窗口/确认长度为正，迟滞方向正确。
    if (p_.window_samples < 1) p_.window_samples = 1;
    if (p_.confirm_samples < 1) p_.confirm_samples = 1;
    if (p_.exit_rad_s < p_.enter_rad_s) p_.exit_rad_s = p_.enter_rad_s;
  }

  // 喂一个陀螺样本（相机系角速度）。非有限值按 0 处理以外的方式挡掉：
  // 一个 NaN 样本会毒化窗口均值，而"均值是 NaN"能通过一切大小比较。
  void push(const Eigen::Vector3d& omega_rad_s) {
    if (!omega_rad_s.allFinite()) {
      return;
    }
    norms_.push_back(omega_rad_s.norm());
    while (static_cast<int>(norms_.size()) > p_.window_samples) {
      norms_.pop_front();
    }
    if (static_cast<int>(norms_.size()) < p_.window_samples) {
      return;  // 窗口没攒满：不表态
    }
    valid_ = true;
    double sum = 0.0;
    for (double v : norms_) sum += v;
    const double avg = sum / static_cast<double>(norms_.size());
    recent_avg_ = avg;

    if (still_) {
      if (avg > p_.exit_rad_s) {
        still_ = false;   // 快出：相机动了，旋转通道立刻解冻
        confirm_ = 0;
      }
    } else {
      if (avg < p_.enter_rad_s) {
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
  // 最近窗口内的平均角速度模（rad/s）。日志与示教运动统计共用这一个数。
  double recent_norm_rad_s() const { return valid_ ? recent_avg_ : 0.0; }

  // 换档/换图/重启跟踪时调用：旧档位的"静止"结论不属于新档位。
  void reset() {
    norms_.clear();
    still_ = false;
    valid_ = false;
    confirm_ = 0;
    recent_avg_ = 0.0;
  }

 private:
  Params p_;
  std::deque<double> norms_;
  bool still_ = false;
  bool valid_ = false;
  int confirm_ = 0;
  double recent_avg_ = 0.0;
};

}  // namespace follow
