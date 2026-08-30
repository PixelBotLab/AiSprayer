#include "follow/frontend.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#ifdef HAS_RKNN
#include "rknn_api.h"
#endif

namespace follow {

void normalize_descriptors(cv::Mat& desc) {
  if (desc.empty()) {
    return;
  }
  for (int i = 0; i < desc.rows; ++i) {
    cv::normalize(desc.row(i), desc.row(i), 1.0, 0.0, cv::NORM_L2);
  }
}

namespace {

cv::Mat to_gray(const cv::Mat& img) {
  if (img.channels() == 1) {
    return img;
  }
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  return gray;
}

// Shi-Tomasi 角点 + SIFT 描述子。
//
// 检测器为什么不用 SIFT 自己的：前端存在的意义是「在 GICP 没有几何可抓的地方
// （白墙上的海报、弱纹理工装）给出均匀铺开的对应点」。goodFeaturesToTrack 是按
// 网格均匀给的；SIFT/FAST 检测器按响应排序，预算会全花在一个高对比度角落上。
//
// 描述子为什么不用 ORB：ORB 输出 32 字节二值，转成 CV_32F 喂 BFMatcher(NORM_L2)
// 后，位向量的 L2 距离是 Hamming 的平方根，同一个 0.8 比例门在两个前端上含义
// 不同（0.8 L2 == 0.64 Hamming）。SIFT 是 128 维 float，和 SuperPoint 同量纲，
// P3 的 parity 测试才有意义。
class CpuFeatureFrontend final : public FeatureFrontend {
 public:
  explicit CpuFeatureFrontend(const FrontendParams& p) : p_(p) {
    sift_ = cv::SIFT::create(p_.max_features, 3, 0.01, 10.0);
  }

  FeatureFrame extract(const cv::Mat& color_bgr_or_gray, int64_t ts_ns) override {
    FeatureFrame fr;
    if (color_bgr_or_gray.empty()) {
      return fr;
    }
    const cv::Mat gray = to_gray(color_bgr_or_gray);
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(gray, corners, p_.max_features, p_.quality_level,
                            static_cast<double>(p_.min_distance_px));

    // 边缘外的点先自己剔掉：这样 compute() 没有理由再丢任何一个，
    // 描述子行号与 uv_px 下标严格一一对应。
    std::vector<cv::KeyPoint> kpts;
    kpts.reserve(corners.size());
    for (const auto& c : corners) {
      if (c.x < kPatchSizePx || c.y < kPatchSizePx || c.x >= gray.cols - kPatchSizePx ||
          c.y >= gray.rows - kPatchSizePx) {
        continue;
      }
      kpts.emplace_back(c, kPatchSizePx);
    }
    if (kpts.empty()) {
      return fr;
    }

    cv::Mat raw;
    sift_->compute(gray, kpts, raw);  // N x 128 CV_32F
    if (raw.empty() || raw.rows != static_cast<int>(kpts.size())) {
      return fr;  // 到这里只可能是实现违反契约，不是常见的「点被剔了」
    }

    fr.uv_px.reserve(kpts.size());
    for (const auto& kp : kpts) {
      fr.uv_px.emplace_back(kp.pt);  // 全分辨率，无需换算
    }
    if (raw.type() != CV_32F) {
      raw.convertTo(fr.desc, CV_32F);
    } else {
      fr.desc = raw;
    }
    normalize_descriptors(fr.desc);
    fr.image_size = gray.size();
    fr.ts_ns = ts_ns;
    return fr;
  }

  const char* name() const override { return "cpu-sift"; }

 private:
  static constexpr float kPatchSizePx = 10.0f;  // 描述子耗时随它涨（见 FrontendParams 注释）

  FrontendParams p_;
  cv::Ptr<cv::SIFT> sift_;
};

#ifdef HAS_RKNN
// 固定尺寸 SuperPoint：要求模型已做 NMS，输出 [1,N,2] 关键点 + [1,N,D] 描述子。
// 原版 heatmap 输出（semi + desc）不在这里做 NMS —— 检测到形状不对就在构造期
// 明确报错，而不是像旧实现那样按 out[0].size 反推 N 然后越界读。
class RknnFeatureFrontend final : public FeatureFrontend {
 public:
  RknnFeatureFrontend(const std::string& path, const FrontendParams& p) : p_(p) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("打不开模型: " + path);
    const std::streamoff sz = f.tellg();
    if (sz <= 0) throw std::runtime_error("模型文件为空: " + path);
    f.seekg(0);
    std::vector<char> buf(static_cast<size_t>(sz));
    f.read(buf.data(), sz);

    if (rknn_init(&ctx_, buf.data(), static_cast<uint32_t>(buf.size()), 0, nullptr) != RKNN_SUCC) {
      throw std::runtime_error("rknn_init 失败: " + path);
    }

    rknn_input_output_num io{};
    check(rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io)), "查询 IO 数量失败");
    in_attr_.resize(io.n_input);
    out_attr_.resize(io.n_output);
    for (uint32_t i = 0; i < io.n_input; ++i) {
      in_attr_[i].index = i;
      check(rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &in_attr_[i], sizeof(rknn_tensor_attr)),
            "查询输入属性失败");
    }
    for (uint32_t i = 0; i < io.n_output; ++i) {
      out_attr_[i].index = i;
      check(rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attr_[i], sizeof(rknn_tensor_attr)),
            "查询输出属性失败");
    }
    if (io.n_input < 1) throw std::runtime_error("模型没有输入张量");

    // 网络输入尺寸以模型自报为准，别信硬编码的 640x480。
    const rknn_tensor_attr& ia = in_attr_[0];
    if (ia.n_dims != 4) throw std::runtime_error("只接受 4 维输入张量, n_dims=" + std::to_string(ia.n_dims));
    const bool nchw = ia.fmt == RKNN_TENSOR_NCHW;
    net_n_ = static_cast<int>(ia.dims[0]);
    net_h_ = static_cast<int>(nchw ? ia.dims[2] : ia.dims[1]);
    net_w_ = static_cast<int>(nchw ? ia.dims[3] : ia.dims[2]);
    const int net_c = static_cast<int>(nchw ? ia.dims[1] : ia.dims[3]);
    if (net_h_ <= 0 || net_w_ <= 0 || net_c != 1) {
      throw std::runtime_error("输入尺寸不合法: NCHW=" + std::to_string(net_n_) + "," +
                               std::to_string(net_c) + "," + std::to_string(net_h_) + "x" +
                               std::to_string(net_w_));
    }

    resolve_outputs();
  }

  ~RknnFeatureFrontend() override {
    if (ctx_) rknn_destroy(ctx_);
  }
  RknnFeatureFrontend(const RknnFeatureFrontend&) = delete;
  RknnFeatureFrontend& operator=(const RknnFeatureFrontend&) = delete;

  FeatureFrame extract(const cv::Mat& color_bgr_or_gray, int64_t ts_ns) override {
    FeatureFrame fr;
    if (color_bgr_or_gray.empty()) {
      return fr;
    }
    cv::Mat gray;
    cv::resize(to_gray(color_bgr_or_gray), gray, cv::Size(net_w_, net_h_));
    cv::Mat f32;
    gray.convertTo(f32, CV_32F, 1.0 / 255.0);

    rknn_input in{};
    in.index = 0;
    in.type = RKNN_TENSOR_FLOAT32;
    in.fmt = in_attr_[0].fmt;
    in.size = static_cast<uint32_t>(f32.total() * f32.elemSize());
    in.buf = f32.data;
    in.pass_through = 0;
    check(rknn_inputs_set(ctx_, 1, &in), "rknn_inputs_set 失败");
    check(rknn_run(ctx_, nullptr), "rknn_run 失败");

    std::vector<rknn_output> out(out_attr_.size());
    for (size_t i = 0; i < out.size(); ++i) {
      out[i].index = static_cast<uint32_t>(i);
      out[i].want_float = 1;
      out[i].is_prealloc = 0;
    }
    check(rknn_outputs_get(ctx_, static_cast<uint32_t>(out.size()), out.data(), nullptr),
          "rknn_outputs_get 失败");

    const float* xy = static_cast<const float*>(out[kp_idx_].buf);
    const float* ds = static_cast<const float*>(out[desc_idx_].buf);
    const int n = std::min(top_k(), kp_count());
    const float sx = static_cast<float>(color_bgr_or_gray.cols) / static_cast<float>(net_w_);
    const float sy = static_cast<float>(color_bgr_or_gray.rows) / static_cast<float>(net_h_);

    fr.uv_px.reserve(static_cast<size_t>(n));
    fr.desc = cv::Mat(n, desc_dim_, CV_32F);
    const float* xy_end = static_cast<const float*>(out[kp_idx_].buf) + out[kp_idx_].size / sizeof(float);
    const float* ds_end = static_cast<const float*>(out[desc_idx_].buf) + out[desc_idx_].size / sizeof(float);
    int valid = 0;
    for (int i = 0; i < n; ++i) {
      const float* kp = xy + 2 * i;
      const float* dsc = ds + static_cast<size_t>(i) * desc_dim_;
      if (kp + 2 > xy_end || dsc + desc_dim_ > ds_end) {
        break;  // 模型自报的元素数与实际 buffer 大小不一致，按能安全读到的为止
      }
      fr.uv_px.emplace_back(kp[0] * sx, kp[1] * sy);  // 映回全分辨率
      std::memcpy(fr.desc.ptr<float>(i), dsc, static_cast<size_t>(desc_dim_) * sizeof(float));
      ++valid;
    }
    if (valid < n) {
      fr.desc = fr.desc.rowRange(0, valid).clone();
    }
    normalize_descriptors(fr.desc);
    fr.image_size = color_bgr_or_gray.size();
    fr.ts_ns = ts_ns;

    rknn_outputs_release(ctx_, static_cast<uint32_t>(out.size()), out.data());
    return fr;
  }

  const char* name() const override { return "superpoint-rknn"; }

 private:
  static void check(int ret, const char* what) {
    if (ret != RKNN_SUCC) {
      throw std::runtime_error(std::string(what) + " (ret=" + std::to_string(ret) + ")");
    }
  }

  // 注意 max_features 在两条实现里语义不同：CPU 是检测预算，这里是模型 NMS 输出
  // 的前 k 个（已按分数排好）。同一个字段，两种花法。
  int top_k() const { return p_.max_features > 0 ? p_.max_features : kp_count(); }
  int kp_count() const { return kp_count_; }

  static std::string dims_of(const rknn_tensor_attr& a) {
    std::string s = "[";
    for (uint32_t i = 0; i < a.n_dims; ++i) {
      if (i) {
        s += ',';
      }
      s += std::to_string(a.dims[i]);
    }
    return s + "]";
  }

  void resolve_outputs() {
    for (size_t i = 0; i < out_attr_.size(); ++i) {
      const rknn_tensor_attr& a = out_attr_[i];
      // 关键点：最后一维为 2 的 [1,N,2]
      if (a.n_dims == 3 && a.dims[2] == 2 && kp_idx_ < 0) {
        kp_idx_ = static_cast<int>(i);
        kp_count_ = static_cast<int>(a.dims[1]);
      }
      // 描述子：[1,N,D]，D 是常见描述子长度之一
      if (a.n_dims == 3 && kp_idx_ != static_cast<int>(i)) {
        const int d = static_cast<int>(a.dims[2]);
        if ((d == 64 || d == 128 || d == 256) && desc_idx_ < 0) {
          desc_idx_ = static_cast<int>(i);
          desc_dim_ = d;
        }
      }
    }
    if (kp_idx_ < 0 || desc_idx_ < 0) {
      std::string shapes;
      for (const auto& a : out_attr_) {
        shapes += dims_of(a) + ' ';
      }
      throw std::runtime_error(
          "只支持已做 NMS 的 SuperPoint 导出 (keypoints [1,N,2] + descriptors [1,N,D])。"
          "若模型是原版 heatmap 输出 (semi + desc)，请在导出时把 NMS 包进图里。实际输出: " +
          shapes);
    }
  }

  FrontendParams p_;
  rknn_context ctx_ = 0;
  std::vector<rknn_tensor_attr> in_attr_, out_attr_;
  int net_w_ = 0, net_h_ = 0, net_n_ = 1;
  int kp_idx_ = -1, desc_idx_ = -1, desc_dim_ = 0, kp_count_ = 0;
};
#endif  // HAS_RKNN

}  // namespace

std::unique_ptr<FeatureFrontend> make_frontend(const std::string& kind, const FrontendParams& p,
                                               std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) {
      *error = msg;
    }
    return std::unique_ptr<FeatureFrontend>();
  };

  if (kind == "cpu" || kind.empty()) {
    return std::make_unique<CpuFeatureFrontend>(p);
  }
  if (kind == "superpoint" || kind == "rknn") {
#ifdef HAS_RKNN
    try {
      return std::make_unique<RknnFeatureFrontend>(p.rknn_model_path, p);
    } catch (const std::exception& e) {
      return fail(e.what());
    }
#else
    return fail("编译时未开启 HAS_RKNN，无法使用 " + p.rknn_model_path);
#endif
  }
  return fail("未知的前端类型: " + kind + "（可用: cpu, superpoint）");
}

}  // namespace follow
