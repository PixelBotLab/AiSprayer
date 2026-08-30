// 合成序列的落盘格式：一帧一帧的深度/彩色图 + 一份真值清单。
//
// 为什么要清单而不是让回放端"自己看着办"：参考地图、体素尺寸、噪声模型、每一帧的命令位姿
// 只要有一端和另一端不一致，量出来的"精度"就是错的，而且错得很像成功。所以生成端把**全部**
// 影响结果的参数写进清单，回放端只读不问。
//
// 坐标约定同 synth_scene.hpp：T_ref_cam 是相机在参考系里的位姿（米），清单里存 t + 行主序
// 3x3 旋转 —— 不存欧拉角也不存轴角，回放端因此不需要任何"序列假设"。参考系 = 示教帧的
// 相机系，所以示教帧的真值就是单位变换。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "follow/types.hpp"

namespace follow {
namespace synth {

enum class Expect : uint8_t {
  kOk,          // 必须报 kOk
  kDegenerate,  // 必须报"不可观"，绝不能报 kOk
  kLost,        // 两个解算器都失败：报 kLost / kNoDepth
  kOutOfEnv,    // 出包络：要重新示教，不是跟丢
};

const char* to_string(Expect e);
bool expect_accepts(Expect want, Status got);  // 断言用：这一帧允许哪些状态

struct SequenceFrame {
  int index = 0;
  std::string depth_file;
  std::string color_file;   // 空 = 不写彩色（纯几何场景）
  int64_t ts_ns = 0;
  Eigen::Isometry3d T_ref_cam = Eigen::Isometry3d::Identity();
  Expect expect = Expect::kOk;
  std::string tag;          // "x10" / "wall" / "recovered"，只出现在报告里
  // 空 = 用清单里的 source_mesh。非空 = **这一帧是从别的东西渲染出来的**（换件用例、生成
  // 出来的大平面）。溯源写错比不写更糟：一份声称来自真扫描件的平面序列会让人按工件去解读它。
  std::string source;
};

struct SequenceSpec {
  int version = 1;
  std::string name;
  std::string source_mesh;      // 本序列的**默认**来源，只作溯源记录；帧可用 source 覆盖
  CameraIntrinsics k;
  // 渲染参数（回放不用，但报告里要有：同一份序列换个噪声种子结果差多少，是精度的一部分）
  int block = 4;
  double noise_mm = 0.5;
  int blur_samples = 1;
  double blur_rot_deg = 0.0;
  double hole_fraction = 0.0;
  // 彩色：固定 block=1（逐像素），只有纹理周期是可选项。<=0 = 均匀灰，即"无纹理但有几何"
  // 那个用例 —— 不记进清单的话，特征前端看到的是什么场景就没人知道了。
  double texture_period_m = 0.03;
  uint32_t seed = 20260830u;   // 每帧实际用 seed + index：同一份噪声实现会让"重复性"恒为 0
  // 追踪参数：生成端按场景尺度定，回放端必须原样用
  double voxel_m = 0.01;
  int depth_stride = 4;
  double max_corr_m = 0.05;
  // 深度有效范围。回放端建参考地图时的 depth→cloud 必须和 Tracker 内部用同一套，否则
  // 地图和测试帧来自两个不同的点集 —— 那量出来的不是精度，是两套裁剪的差。
  double zmin_m = 0.30;
  double zmax_m = 2.50;
  // 参考地图 = 第 0 帧（示教帧）的几何。参考系就是这一帧的相机系，所以它的真值是单位变换，
  // Tracker 的冷启动初值也恰好是对的 —— 没有"示教帧选哪一帧"这个选项。
  std::vector<SequenceFrame> frames;
};

// sequence.yaml。写完立刻能被 Load 读回是这两条的测试内容（见 test_synth）。
bool save_sequence(const std::string& dir, const SequenceSpec& s, std::string* err);
bool load_sequence(const std::string& dir, SequenceSpec* s, std::string* err);

// CV_16UC1 毫米 / CV_8UC3。文件名取自帧（与 read_frame 对称）。color_file 为空的帧只写深度。
bool write_frame(const std::string& dir, const SequenceFrame& f, const cv::Mat& depth_mm,
                 const cv::Mat& color_bgr, std::string* err);
bool read_frame(const std::string& dir, const SequenceFrame& f, cv::Mat* depth_mm,
                cv::Mat* color_bgr, std::string* err);

// 把网格从基座系摆进参考系。参考系**取示教帧的相机系**：于是清单里示教帧的真值恒等于单位
// 变换，而 Tracker 冷启动的 T_last_good_=Identity 恰好就是它 —— 换任何别的参考系，第一帧都会
// 从一个错了一米多的初值起步，测的就不再是算法而是初值了。
// 质心落到光轴上距相机 distance_m 处，姿态就近取真实标定的那个轴置换（差的 ~0.5° 是标定
// 残差，不是 P3 要测的东西）。返回 T_ref_base。
Eigen::Isometry3d place_mesh(const Eigen::Vector3d& centroid_base, double distance_m);

}  // namespace synth
}  // namespace follow
