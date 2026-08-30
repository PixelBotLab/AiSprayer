// 分辨率与预算的裁决数据：把 follow_probe_device --dump 落下来的**真实工位图**喂给 CPU 特征
// 前端，量它到底吃多少毫秒。为什么不能拿合成图上的数代替：合成图的纹理分布是我造的，角点数量
// 就是参数写的那个；真实工位有反光、有虚焦、有大片无纹理，goodFeaturesToTrack 的耗时随图像
// 内容走。预算是按内容最坏情况定的，所以只能量真的。
//
// 打印：每个尺寸 × 每个 max_features 的 p50/p95 毫秒，以及检测到的特征数。
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "follow/frontend.hpp"

namespace follow {
namespace {

double pctl(std::vector<double> v, double q) {
  std::sort(v.begin(), v.end());
  return v[static_cast<size_t>(q * static_cast<double>(v.size() - 1) + 0.5)];
}

int bench_main(int argc, char** argv) {
  std::string dir = "follow/out/real";
  int rounds = 12;
  if (argc > 1) {
    dir = argv[1];
  }
  if (argc > 2) {
    rounds = std::atoi(argv[2]);
  }
  cv::Mat full;
  {
    std::ifstream probe(dir + "/color_1280x800.bgr", std::ios::binary | std::ios::ate);
    if (!probe) {
      std::printf("找不到 %s/color_1280x800.bgr —— 先跑 follow_probe_device --dump %s\n",
                  dir.c_str(), dir.c_str());
      return 2;
    }
    const size_t n = static_cast<size_t>(probe.tellg());
    std::vector<uint8_t> buf(n);
    probe.seekg(0);
    probe.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(n));
    if (n != 1280u * 800u * 3u) {
      std::printf("彩色大小 %zu ≠ 1280*800*3\n", n);
      return 2;
    }
    full = cv::Mat(800, 1280, CV_8UC3, buf.data()).clone();
  }

  std::printf("真实工位图 %dx%d，%d 轮\n", full.cols, full.rows, rounds);
  std::printf("  %-14s %-6s  p50 ms   p95 ms   特征数\n", "尺寸", "maxfeat");
  for (const int feat : {200, 400}) {
    for (const int scale : {100, 66, 50}) {  // 1280 → 1280 / 848 / 640
      const int w = full.cols * scale / 100, h = full.rows * scale / 100;
      cv::Mat img;
      if (scale == 100) {
        img = full;
      } else {
        cv::resize(full, img, cv::Size(w, h), 0, 0, cv::INTER_AREA);
      }
      FrontendParams fp;
      fp.max_features = feat;
      std::string err;
      auto fe = make_frontend("cpu", fp, &err);
      if (!fe) {
        std::printf("  前端创建失败: %s\n", err.c_str());
        return 1;
      }
      int got = 0;
      std::vector<double> ms;
      for (int i = 0; i < rounds; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        const FeatureFrame f = fe->extract(img, static_cast<int64_t>(i) * 33'000'000);
        const auto t1 = std::chrono::steady_clock::now();
        ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        if (i == 0) {
          got = f.count();
        }
      }
      std::printf("  %4dx%-9d %-6d  %6.1f   %6.1f   %d\n", w, h, feat, pctl(ms, 0.5),
                  pctl(ms, 0.95), got);
    }
  }
  return 0;
}

}  // namespace
}  // namespace follow

int main(int argc, char** argv) { return follow::bench_main(argc, argv); }
