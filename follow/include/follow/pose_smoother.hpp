// 显示位姿 = 最近 N 帧的平均。原来写在 follow_pose 里，现在两个消费方共用：
//
//   * follow_pose —— 控制台那一行；
//   * 相机服务里的 follow worker —— 推给页面/臂的那一路实时数。
//
// 放在一起的理由和 follow_pose 与节点共用示教实现是同一条：**给人读的那个数必须由同一段
// 代码算出来**，否则"演示里看着稳"就不再蕴含"页面上看着稳"，两边各自调各自的窗口，最后
// 谁也说不清哪个才是真相。
//
// 为什么需要它：静止实测的单帧逐轴噪声 sd 就有 ~2 mm，而"1 mm 就打印/就发臂"是用户要的
// 粒度 —— 噪声比阈值还大时，死区再调也挡不住刷屏（follow_pose 实测 90 帧仍打 61 行）。
// N 帧平均把噪声压到 sd/√N，代价是约 N/2 帧（@15fps、N=5 时 0.17 s）的显示滞后。给人读的
// 数，这笔交易划算。**精度统计仍然用全部原始单帧**，这里只影响"报出来的是哪一个数"。
#pragma once

#include <deque>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace follow {

// 5 帧 = 把 ~2 mm 的单帧噪声压到 <1 mm，正好让 follow_pose 的 1.0 mm 死区闸门有意义。
// 再大就开始在 15 fps 上吃进肉眼可见的滞后。
constexpr int kDefaultSmoothFrames = 5;

class PoseSmoother {
 public:
  explicit PoseSmoother(int n) : n_(n < 1 ? 1 : n) {}

  void push(const Eigen::Isometry3d& T) {
    buf_.push_back(T);
    if (static_cast<int>(buf_.size()) > n_) {
      buf_.pop_front();
    }
  }

  // 契约：至少 push 过一次之后再读（空窗口的均值是 0/0，不是"没有修正"）。
  Eigen::Isometry3d value() const {
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Matrix3d r = Eigen::Matrix3d::Zero();
    for (const auto& T : buf_) {
      t += T.translation();
      r += T.rotation();
    }
    const double inv = 1.0 / static_cast<double>(buf_.size());
    t *= inv;
    r *= inv;
    // 平均出来的矩阵不是正交阵，直接拿去解欧拉角会得到带尺度误差的旋转。投影回 SO(3)：
    // R = U·diag(1,1,det(UVᵀ))·Vᵀ，这是欧氏意义下最近的正交阵。窗口内角度跨度 < 1° 时
    // 这一步几乎不改变结果，但它保证"平均"不会悄悄变成一个不合法的姿态。
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(r, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d u = svd.matrixU(), v = svd.matrixV();
    Eigen::Matrix3d R = u * v.transpose();
    if (R.determinant() < 0.0) {
      Eigen::Matrix3d flip = Eigen::Matrix3d::Identity();
      flip(2, 2) = -1.0;
      R = u * flip * v.transpose();
    }
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.linear() = R;
    out.translation() = t;
    return out;
  }

  int used() const { return static_cast<int>(buf_.size()); }
  int size() const { return n_; }

 private:
  int n_;
  std::deque<Eigen::Isometry3d> buf_;
};

}  // namespace follow
