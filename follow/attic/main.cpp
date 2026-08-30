// 3588 上：Orbbec 对齐 RGB-D → SuperPoint(NPU) + small_gicp(CPU) → 相对第一帧位姿。
// 取流部分请接 Orbbec SDK v2 C++（与 Python 示例同一套 AlignFilter / D2C），这里用接口占位。
// 编译：见同目录 CMakeLists.txt。不要链 ROS。

#include "follow_odometry.hpp"
#include "superpoint_rknn.hpp"

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// 把 Orbbec FrameSet 填进这两个 Mat：color BGR 或 Gray，depth uint16 毫米、已对齐到彩色。
struct RgbdFrame {
  cv::Mat bgr;
  cv::Mat depth_mm;
  follow::CameraK k;
};

static Eigen::Isometry3d invert(const Eigen::Isometry3d& T) {
  Eigen::Isometry3d inv = Eigen::Isometry3d::Identity();
  inv.linear() = T.linear().transpose();
  inv.translation() = -T.linear().transpose() * T.translation();
  return inv;
}

int main(int argc, char** argv) {
  const char* rknn = argc > 1 ? argv[1] : "superpoint.rknn";
  follow::SuperPointRknn sp(rknn);

  follow::SpGicpOdometry::Params p;
  // p.k 从 color_profile.get_intrinsic() 填 fx fy cx cy
  follow::SpGicpOdometry odom(p);

  Eigen::Isometry3d T0;
  bool have_T0 = false;

  // while (RgbdFrame f = wait_orbbec()) {
  //   cv::Mat gray;
  //   cv::cvtColor(f.bgr, gray, cv::COLOR_BGR2GRAY);
  //   auto kpts = sp.infer(gray);
  //   auto st = odom.track(kpts, f.depth_mm);
  //   if (!st.ok) { std::cerr << "lost\n"; continue; }  // 接臂：停 ServoP
  //   if (!have_T0) { T0 = st.T_world_cam; have_T0 = true; continue; }
  //   const Eigen::Isometry3d T_rel = invert(T0) * st.T_world_cam;
  //   const auto t = T_rel.translation();
  //   std::cout << "dx=" << t.x() << " dy=" << t.y() << " dz=" << t.z()
  //             << (st.used_superpoint_fallback ? "  [SP fallback]" : "")
  //             << (st.gicp_degenerate ? "  [GICP degenerate]" : "") << "\n";
  // }
  (void)T0;
  (void)have_T0;
  std::cerr << "接上 Orbbec wait_for_frames 后取消 main 里的注释即可跑。模型: " << rknn << "\n";
  return 0;
}
