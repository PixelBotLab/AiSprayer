// SuperPoint → RKNN。把灰度图送进 NPU，取出像素坐标 + 256 维描述子。
//
// 板端准备（在 x86 上用 rknn-toolkit2 转一次即可）：
//   1. 导出固定尺寸 ONNX，建议 1x1x480x640 或 NHWC 1x480x640x1，灰度 [0,1]
//   2. python convert.py --target rk3588 → superpoint.rknn
//   3. 两种常见输出，infer() 里按你的模型改 query：
//        A) 已做 NMS：keypoints [1,N,2]、scores [1,N]、descriptors [1,N,256]
//        B) 原版：semi [1,65,H/8,W/8] + desc [1,256,H/8,W/8]，CPU 做 NMS
//
// 不要第一版就上 LightGlue。描述子互近邻在 follow_odometry.hpp。

#pragma once

#include "follow_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#ifdef HAS_RKNN
#include "rknn_api.h"
#endif

namespace follow {

class SuperPointRknn {
public:
  explicit SuperPointRknn(const std::string& rknn_path, int top_k = 400) : top_k_(top_k) {
#ifdef HAS_RKNN
    std::ifstream f(rknn_path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("打不开 " + rknn_path);
    const auto sz = f.tellg();
    f.seekg(0);
    std::vector<char> buf(static_cast<size_t>(sz));
    f.read(buf.data(), sz);
    const int ret = rknn_init(&ctx_, buf.data(), buf.size(), 0, nullptr);
    if (ret != RKNN_SUCC) throw std::runtime_error("rknn_init 失败");
    rknn_input_output_num io{};
    rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io));
    n_in_ = io.n_input;
    n_out_ = io.n_output;
#else
    (void)rknn_path;
#endif
  }

  ~SuperPointRknn() {
#ifdef HAS_RKNN
    if (ctx_) rknn_destroy(ctx_);
#endif
  }

  SuperPointRknn(const SuperPointRknn&) = delete;
  SuperPointRknn& operator=(const SuperPointRknn&) = delete;

  // gray: CV_8UC1，将 resize 到模型输入（默认 640x480）。
  SpFrame infer(const cv::Mat& gray_full) {
    cv::Mat gray;
    cv::resize(gray_full, gray, cv::Size(w_, h_));
    gray.convertTo(gray, CV_32F, 1.0 / 255.0);
    const float sx = static_cast<float>(gray_full.cols) / static_cast<float>(w_);
    const float sy = static_cast<float>(gray_full.rows) / static_cast<float>(h_);

#ifdef HAS_RKNN
    rknn_input in{};
    in.index = 0;
    in.type = RKNN_TENSOR_FLOAT32;
    in.fmt = RKNN_TENSOR_NHWC; // 若你的模型是 NCHW，改这里
    in.size = static_cast<uint32_t>(w_ * h_ * sizeof(float));
    in.buf = gray.data;
    rknn_inputs_set(ctx_, 1, &in);
    rknn_run(ctx_, nullptr);

    std::vector<rknn_output> out(n_out_);
    for (uint32_t i = 0; i < n_out_; ++i) {
      out[i].want_float = 1;
      out[i].is_prealloc = 0;
    }
    rknn_outputs_get(ctx_, n_out_, out.data(), nullptr);

    SpFrame fr;
    // ---- 按导出格式 A：keypoints + descriptors ----
    // 用 rknn_query 看 dims。下面假设 out[0]=Nx2 xy，out[1]=Nx256。
    // 若是格式 B（heatmap），改成 nms_heatmap(...) 。
    decode_keypoints_desc(out, fr, sx, sy);

    rknn_outputs_release(ctx_, n_out_, out.data());
    return fr;
#else
    (void)sx;
    (void)sy;
    throw std::runtime_error("编译时加 -DHAS_RKNN，并链接 librknnrt");
#endif
  }

private:
#ifdef HAS_RKNN
  void decode_keypoints_desc(std::vector<rknn_output>& out, SpFrame& fr, float sx, float sy) {
    // 占位：请用 rknn_query(RKNN_QUERY_OUTPUT_ATTR) 打印真实 dims 后改。
    // 常见：out[0] 1xNx2，out[last] 1xNx256。
    if (out.size() < 2) throw std::runtime_error("SuperPoint 输出数量不对");
    const float* xy = static_cast<const float*>(out[0].buf);
    const int n = std::min(top_k_, static_cast<int>(out[0].size / (2 * sizeof(float))));
    const float* desc = static_cast<const float*>(out.back().buf);
    fr.uv.reserve(n);
    fr.desc = cv::Mat(n, 256, CV_32F);
    for (int i = 0; i < n; ++i) {
      fr.uv.emplace_back(xy[2 * i] * sx, xy[2 * i + 1] * sy);
      float* row = fr.desc.ptr<float>(i);
      std::memcpy(row, desc + i * 256, 256 * sizeof(float));
      const float nrm = cv::norm(fr.desc.row(i));
      if (nrm > 1e-6f) fr.desc.row(i) /= nrm;
    }
  }
#endif

  int top_k_ = 400;
  int w_ = 640;
  int h_ = 480;
#ifdef HAS_RKNN
  rknn_context ctx_ = 0;
  uint32_t n_in_ = 0, n_out_ = 0;
#endif
};

} // namespace follow
