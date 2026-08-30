// follow_node 的全部可配置项，以及它们的校验。
//
// 三条规则，都是这台板子上吃过亏才写的：
//  1) **默认值是量出来的**。848x480@15 / max_features=200 来自 bench_frontend 在真实工位图上的
//     耗时表；first_pair_timeout 来自实测 333 ms 的彩色暖机；zmax 来自数据集深度范围 740~2589 mm。
//     没有一条是"设计偏好"。
//  2) **一次报全所有问题**。运维改坏一个数（把 fps 写成 thirty、把 max_corr_m 写成 5），只报第一
//     个错会让人改三轮。所以校验返回的是问题清单，而且分 fatal/warning。
//  3) **类型错误不能静默**。yaml-cpp 的 as<T>() 抛异常，吞掉它等于把"配置没生效"伪装成"用了默认值"。
//     这里逐个键捕获并指名道姓地报出来。
//
// 单位：与内部一致用 SI（米/度/秒）；键名后缀标明单位，不允许含糊。
#pragma once

#include <string>
#include <vector>

#include "follow/frontend.hpp"
#include "follow/odometry.hpp"
#include "follow/orbbec_capture.hpp"

namespace follow {

struct ConfigProblem {
  bool fatal = false;
  std::string text;
};

struct ConfigProblems {
  std::vector<ConfigProblem> items;
  bool ok() const;
  // 换行分隔的可读清单；空则返回 "无"。
  std::string joined() const;
  size_t fatals() const;
  size_t warnings() const;
};

struct FollowConfig {
  FollowConfig() {
    // CaptureParams 的 struct 默认是设备层"能谈下来的最高档"（30 fps），而 follow 的预算是
    // 按 15 fps 量的。内置默认必须是量出来的那一档 —— 没有配置文件的机器不能悄悄跑成超预算。
    capture.fps = 15;
  }

  // --- 相机 / 取流 ---
  CaptureParams capture;         // lock_path 已解析成绝对路径
  std::string lock_path_rel = ".orbbec.lock";

  // --- 特征前端 ---
  std::string frontend_kind = "cpu";
  FrontendParams frontend;

  // --- 配准与判据 ---
  TrackParams track;             // 不含 k：内参在运行时来自设备自报，配置里写它只会骗人

  // --- 示教 ---
  std::string map_path_rel = "follow/out/reference.frmap";
  std::string map_path;          // 解析后
  int teach_frames = 1;

  // --- 标定与安装方式 ---
  std::string mount = "eye-to-hand";
  std::string calib_path_rel = "configs/calib/calibration_result.yaml";
  std::string calib_path;

  // --- 运行 ---
  bool dry_run = true;
  bool enable_servo_p = false;
  int health_port = 18081;
  int max_cycles = 0;            // 0 = 一直跑；>0 给自测用
  std::string log_level = "info";

  std::string source;            // 实际读到的文件；空 = 全用内置默认
  std::vector<std::string> notes;  // "键 X 不存在，用默认 Y" 之外的补充说明
};

// 从可执行文件位置和当前目录向上找 configs/aisprayer_config.yaml 所在目录。
// 找不到返回空串（调用方据此决定是报错还是用相对路径硬跑）。
std::string find_project_root();

// path 为空时取 <root>/configs/aisprayer_config.yaml 的 follow: 块。
//   * 显式给了路径而文件读不到 → fatal（不能假装"没配就是默认"）。
//   * 文件里根本没有 follow: 块 → 用内置默认并在 notes 里写清楚。
// 无论哪种情况，返回 true 表示 cfg 可用（数值默认总是自洽的）；只有类型/结构错误才 false。
bool load_config(const std::string& path, FollowConfig* cfg, std::string* err);

// 加载之后再查一遍。把相对路径解析成绝对路径，返回问题清单（含 fatal）。
ConfigProblems check_config(FollowConfig* cfg, const std::string& root);

// 把生效配置原样打印出来：日志里必须能还原"这次跑的到底是哪组数"。
std::string describe(const FollowConfig& cfg);

}  // namespace follow
