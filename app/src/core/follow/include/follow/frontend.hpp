// 特征前端接口。两条实现：CPU（开箱即用）与 RKNN SuperPoint（需板端 .rknn 模型）。
//
// 契约（被 frontend_parity 测试钉住）：
//   - FeatureFrame::uv_px 必须是全分辨率彩色图像素坐标，前端内部若 resize 必须自己映射回来
//   - FeatureFrame::desc 必须是 CV_32F 且按行 L2 归一，好让下游 BFMatcher(NORM_L2) 对
//     两种前端都用同一套代码
//   - FeatureFrame::image_size 必须等于输入图尺寸
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "follow/types.hpp"

namespace follow {

struct FrontendParams {
  // CPU 前端的预算按「能塞进帧预算」定，不是按「能检测多少」定。本机实测（真实
  // 1280x800 工位图）：400 特征 + patch12 全流程 93 ms —— 单这一项就超了 15 fps
  // 的 66 ms。200 特征 + patch10 @848x480 约 14 ms(检测) + 24 ms(描述子)。
  // 前端只负责给 GICP 一个平移初值、以及在几何退化时顶上稀疏解，不需要大预算。
  // 生产路径仍是 RKNN SuperPoint（NPU），CPU 这条是没模型时的降级。
  int max_features = 200;
  double quality_level = 0.01;  // CPU 前端：Shi-Tomasi 质量阈值
  int min_distance_px = 16;     // CPU 前端：角点最小间距，避免局部扎堆
  std::string rknn_model_path = "superpoint.rknn";
  cv::Size net_input{640, 480};  // RKNN 前端：网络固定输入尺寸
};

class FeatureFrontend {
 public:
  virtual ~FeatureFrontend() = default;
  virtual FeatureFrame extract(const cv::Mat& color_bgr_or_gray, int64_t ts_ns) = 0;
  virtual const char* name() const = 0;
};

// kind: "cpu" | "superpoint"。失败返回 nullptr 并写 error，不抛异常 ——
// 配置错误要在启动阶段报给人看，不该让进程在取流线程里炸。
std::unique_ptr<FeatureFrontend> make_frontend(const std::string& kind, const FrontendParams& p,
                                               std::string* error);

// 行 L2 归一（原地）。两种前端共用。
void normalize_descriptors(cv::Mat& desc);

}  // namespace follow
