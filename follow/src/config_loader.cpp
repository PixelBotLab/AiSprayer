#include "follow/config_loader.hpp"

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace follow {
namespace {

constexpr int kMaxRootHops = 8;

// 取值并把类型错误指名道姓报出来。静默吞掉 as<T>() 异常等于把「配置没生效」
// 伪装成「用了默认值」，是最难查的一类故障。
template <typename T>
void fetch(const YAML::Node& node, const std::string& key, const char* expect,
           const std::string& prefix, T* dst, std::vector<std::string>* errors) {
  if (!node.IsMap()) {
    return;  // 整段结构不对：问题由 section() 报，这里再索引只会抛异常
  }
  const YAML::Node v = node[key];
  if (!v) {
    return;
  }
  try {
    *dst = v.as<T>();
  } catch (const YAML::Exception& e) {
    errors->push_back(prefix + key + " 期望 " + expect + "，实际读不出来: " + e.msg);
  }
}

// 取子块。**必须**先判 IsMap：把 camera: 少缩进一层写成 `camera: 848` 时，yaml-cpp 对标量
// 节点再取 [key] 会抛 "operator[] call on a scalar" —— 不判就是把"配置有错字"升级成
// "进程启动即崩"，而崩在启动阶段是最难归因的一种失败。
YAML::Node section(const YAML::Node& parent, const std::string& key, const std::string& prefix,
                   std::vector<std::string>* errors) {
  if (!parent.IsMap()) {
    return YAML::Node();
  }
  const YAML::Node n = parent[key];
  if (!n) {
    return YAML::Node();
  }
  if (!n.IsMap()) {
    errors->push_back(prefix + key + " 应是一个映射（下面挂子键），实际是一个值 —— 检查缩进");
    return YAML::Node();
  }
  return n;
}

void add(std::vector<ConfigProblem>* out, bool fatal, const std::string& text) {
  out->push_back(ConfigProblem{fatal, text});
}

// 定长数值序列（关节角 / 欧拉角）。长度不对在**这里**报：等到 check_config 再报，就得为每个
// 键把值搬过去一遍，而"键名只出现在一处"是这份配置报错能不能对上号的关键。
void fetch_vec(const YAML::Node& node, const std::string& key, size_t want,
               const std::string& prefix, std::vector<double>* dst,
               std::vector<std::string>* errors) {
  if (!node.IsMap()) {
    return;
  }
  const YAML::Node v = node[key];
  if (!v) {
    return;
  }
  if (!v.IsSequence()) {
    errors->push_back(prefix + key + " 期望 " + std::to_string(want) + " 个数的序列，实际不是一个列表");
    return;
  }
  std::vector<double> got;
  for (const auto& e : v) {
    try {
      got.push_back(e.as<double>());
    } catch (const YAML::Exception& ex) {
      errors->push_back(prefix + key + " 有一个元素不是数: " + ex.msg);
      return;
    }
  }
  if (got.size() != want) {
    errors->push_back(prefix + key + " 需要 " + std::to_string(want) + " 个值，实到 " +
                      std::to_string(got.size()) + " 个");
    return;
  }
  for (double x : got) {
    if (!std::isfinite(x)) {
      errors->push_back(prefix + key + " 含非有限值（nan/inf），拒绝");
      return;
    }
  }
  *dst = std::move(got);
}

std::string abs_or_empty(const std::string& root, const std::string& p) {
  if (p.empty()) {
    return {};
  }
  std::filesystem::path path(p);
  if (path.is_absolute() || root.empty()) {
    return path.lexically_normal().string();
  }
  return (std::filesystem::path(root) / path).lexically_normal().string();
}

bool is_regular_file(const std::string& p) {
  std::error_code ec;
  return !p.empty() && std::filesystem::is_regular_file(p, ec);
}

}  // namespace

bool ConfigProblems::ok() const {
  return std::none_of(items.begin(), items.end(), [](const ConfigProblem& p) { return p.fatal; });
}

std::string ConfigProblems::joined() const {
  if (items.empty()) {
    return "无";
  }
  std::ostringstream os;
  for (size_t i = 0; i < items.size(); ++i) {
    os << (items[i].fatal ? "错误: " : "警告: ") << items[i].text;
    if (i + 1 < items.size()) {
      os << "\n";
    }
  }
  return os.str();
}

size_t ConfigProblems::fatals() const {
  return std::count_if(items.begin(), items.end(), [](const ConfigProblem& p) { return p.fatal; });
}

size_t ConfigProblems::warnings() const { return items.size() - fatals(); }

std::string find_project_root() {
  namespace fs = std::filesystem;
  std::vector<fs::path> starts;
  std::error_code ec;
  starts.push_back(fs::current_path(ec));
  // 从二进制自己的位置再找一遍：服务是被 systemd/Python 拉起来的，cwd 不可控。
  char buf[4096];
  const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (n > 0) {
    buf[n] = '\0';
    starts.push_back(fs::path(buf).parent_path());
  }
  for (const fs::path& s0 : starts) {
    if (s0.empty()) {
      continue;
    }
    fs::path p = fs::weakly_canonical(s0, ec);
    for (int i = 0; i < kMaxRootHops && !p.empty(); ++i) {
      if (fs::exists(p / "configs" / "aisprayer_config.yaml")) {
        return p.string();
      }
      const fs::path up = p.parent_path();
      if (up == p) {
        break;
      }
      p = up;
    }
  }
  return {};
}

bool load_config(const std::string& path, FollowConfig* cfg, std::string* err) {
  if (cfg == nullptr) {
    if (err) {
      *err = "cfg 是空指针";
    }
    return false;
  }
  *cfg = FollowConfig{};

  const std::string root = find_project_root();
  std::string file = path;
  const bool explicit_path = !file.empty();
  if (!explicit_path) {
    file = root.empty()
               ? std::string("configs/aisprayer_config.yaml")
               : (std::filesystem::path(root) / "configs" / "aisprayer_config.yaml").string();
  }

  std::error_code ec;
  if (!std::filesystem::exists(file, ec)) {
    if (explicit_path) {
      if (err) {
        *err = "配置文件读不到: " + file;
      }
      return false;
    }
    // 没给路径且找不到仓库配置：跑内置默认（都是量出来的值），但要说清楚。
    cfg->notes.push_back("没找到 " + file + " —— 全部使用内置默认值");
    return true;
  }

  YAML::Node root_node;
  try {
    root_node = YAML::LoadFile(file);
  } catch (const YAML::Exception& e) {
    if (err) {
      *err = "配置文件解析失败 " + file + ": " + e.msg;
    }
    return false;
  }

  std::vector<std::string> errors;
  if (!root_node.IsMap()) {
    if (err) {
      *err = "配置文件顶层不是一个映射（整份文件是空的就是这种情况）: " + file;
    }
    return false;
  }
  const YAML::Node raw = root_node["follow"];
  if (!raw || raw.IsNull()) {
    // 键不存在，或只写了一个空的 `follow:` 占位：两者都按"没有覆盖项，用内置默认"处理。
    cfg->source = file;
    cfg->notes.push_back(file + " 里没有可用的 follow: 块 —— 全部使用内置默认值");
    return true;
  }
  if (!raw.IsMap()) {
    if (err) {
      *err = "follow: 应是一个映射（下面挂 camera/frontend/track/teach/runtime/arm），实际是一个值 —— 检查缩进";
    }
    return false;
  }
  const YAML::Node& f = raw;
  cfg->source = file;

  fetch<std::string>(f, "mount", "字符串", "follow.", &cfg->mount, &errors);

  const YAML::Node cam = section(f, "camera", "follow.", &errors);
  {
    fetch<int>(cam, "width", "整数", "follow.camera.", &cfg->capture.width, &errors);
    fetch<int>(cam, "height", "整数", "follow.camera.", &cfg->capture.height, &errors);
    fetch<int>(cam, "fps", "整数", "follow.camera.", &cfg->capture.fps, &errors);
    fetch<int>(cam, "frame_timeout_ms", "整数", "follow.camera.", &cfg->capture.frame_timeout_ms,
               &errors);
    fetch<int>(cam, "first_pair_timeout_ms", "整数", "follow.camera.",
               &cfg->capture.first_pair_timeout_ms, &errors);
    fetch<bool>(cam, "allow_unaligned", "布尔", "follow.camera.", &cfg->capture.allow_unaligned,
                &errors);
    fetch<bool>(cam, "enable_imu", "布尔", "follow.camera.", &cfg->capture.enable_imu, &errors);
    fetch<std::string>(cam, "lock_path", "路径", "follow.camera.", &cfg->lock_path_rel, &errors);
  }

  const YAML::Node fe = section(f, "frontend", "follow.", &errors);
  {
    fetch<std::string>(fe, "kind", "字符串", "follow.frontend.", &cfg->frontend_kind, &errors);
    fetch<int>(fe, "max_features", "整数", "follow.frontend.", &cfg->frontend.max_features,
               &errors);
    fetch<double>(fe, "quality_level", "小数", "follow.frontend.", &cfg->frontend.quality_level,
                  &errors);
    fetch<int>(fe, "min_distance_px", "整数", "follow.frontend.", &cfg->frontend.min_distance_px,
               &errors);
    fetch<std::string>(fe, "rknn_model_path", "路径", "follow.frontend.",
                       &cfg->frontend.rknn_model_path, &errors);
  }

  const YAML::Node tr = section(f, "track", "follow.", &errors);
  {
    fetch<double>(tr, "zmin_m", "小数", "follow.track.", &cfg->track.zmin_m, &errors);
    fetch<double>(tr, "zmax_m", "小数", "follow.track.", &cfg->track.zmax_m, &errors);
    fetch<int>(tr, "depth_stride", "整数", "follow.track.", &cfg->track.depth_stride, &errors);
    fetch<double>(tr, "voxel_m", "小数", "follow.track.", &cfg->track.voxel_m, &errors);
    fetch<double>(tr, "max_corr_m", "小数", "follow.track.", &cfg->track.max_corr_m, &errors);
    fetch<int>(tr, "threads", "整数", "follow.track.", &cfg->track.threads, &errors);
    fetch<int>(tr, "max_iters", "整数", "follow.track.", &cfg->track.max_iters, &errors);
    fetch<int>(tr, "min_cloud_points", "整数", "follow.track.", &cfg->track.min_cloud_points,
               &errors);
    fetch<int>(tr, "min_gicp_inliers", "整数", "follow.track.", &cfg->track.min_gicp_inliers,
               &errors);
    fetch<double>(tr, "trans_sigma_mm", "小数", "follow.track.", &cfg->track.max_trans_sigma_mm,
                  &errors);
    fetch<double>(tr, "rot_sigma_deg", "小数", "follow.track.", &cfg->track.max_rot_sigma_deg,
                  &errors);
    fetch<double>(tr, "group_anisotropy", "小数", "follow.track.",
                  &cfg->track.max_group_anisotropy, &errors);
    fetch<double>(tr, "min_residual_var_scale", "小数", "follow.track.",
                  &cfg->track.min_residual_var_scale, &errors);
    fetch<double>(tr, "min_inlier_ratio", "小数", "follow.track.", &cfg->track.min_inlier_ratio,
                  &errors);
    fetch<int>(tr, "max_sparse_streak", "整数", "follow.track.", &cfg->track.max_sparse_streak,
               &errors);
    fetch<double>(tr, "gyro_rot_gate_deg", "小数", "follow.track.",
                  &cfg->track.gyro_rot_gate_deg, &errors);
    fetch<double>(tr, "sparse_inlier_dist_m", "小数", "follow.track.",
                  &cfg->track.sparse.inlier_dist_m, &errors);
    fetch<int>(tr, "sparse_min_inliers", "整数", "follow.track.", &cfg->track.sparse.min_inliers,
               &errors);
  }

  const YAML::Node te = section(f, "teach", "follow.", &errors);
  {
    fetch<std::string>(te, "map_path", "路径", "follow.teach.", &cfg->map_path_rel, &errors);
    fetch<int>(te, "frames", "整数", "follow.teach.", &cfg->teach_frames, &errors);
    fetch<double>(te, "max_motion_deg_s", "小数", "follow.teach.", &cfg->teach_max_motion_deg_s,
                  &errors);
  }

  const YAML::Node rt = section(f, "runtime", "follow.", &errors);
  {
    fetch<bool>(rt, "dry_run", "布尔", "follow.runtime.", &cfg->dry_run, &errors);
    fetch<bool>(rt, "enable_servo_p", "布尔", "follow.runtime.", &cfg->enable_servo_p, &errors);
    fetch<int>(rt, "health_port", "整数", "follow.runtime.", &cfg->health_port, &errors);
    fetch<int>(rt, "max_cycles", "整数", "follow.runtime.", &cfg->max_cycles, &errors);
    fetch<std::string>(rt, "calib_path", "路径", "follow.runtime.", &cfg->calib_path_rel, &errors);
    fetch<std::string>(rt, "log_level", "字符串", "follow.runtime.", &cfg->log_level, &errors);
  }

  // arm 段**只有 Python 后端读**（C++ 不发臂，一轮也不发）。仍然在这里解析并校验：这是
  // follow_node 的自测通道，读到同一份配置就得对同一份配置负责 —— 否则后端拿着一个
  // 写错的 home 角去示教，而 `follow_pose` 全程一声不吭。
  const YAML::Node arm = section(f, "arm", "follow.", &errors);
  {
    fetch<std::string>(arm, "mode", "字符串", "follow.arm.", &cfg->arm_mode, &errors);
    fetch_vec(arm, "home_joints_deg", 6, "follow.arm.", &cfg->arm_home_joints_deg, &errors);
    fetch_vec(arm, "camera_to_base_fallback_euler_deg", 3, "follow.arm.",
              &cfg->arm_fallback_euler_deg, &errors);
    fetch<int>(arm, "poll_hz", "整数", "follow.arm.", &cfg->arm_poll_hz, &errors);
    fetch<int>(arm, "emit_hz", "整数", "follow.arm.", &cfg->arm_emit_hz, &errors);
    fetch<double>(arm, "max_joint_vel_deg_s", "小数", "follow.arm.",
                  &cfg->arm_max_joint_vel_deg_s, &errors);
    fetch<bool>(arm, "teach_save_map", "布尔", "follow.arm.", &cfg->arm_teach_save_map, &errors);
  }

  if (!errors.empty()) {
    std::ostringstream os;
    os << "配置项类型错误（这些键根本没生效，先修它们）:";
    for (const auto& e : errors) {
      os << "\n  " << e;
    }
    if (err) {
      *err = os.str();
    }
    return false;
  }
  return true;
}

ConfigProblems check_config(FollowConfig* cfg, const std::string& root) {
  ConfigProblems out;
  auto* items = &out.items;
  for (const auto& n : cfg->notes) {
    add(items, false, n);
  }

  cfg->map_path = abs_or_empty(root, cfg->map_path_rel);
  cfg->calib_path = abs_or_empty(root, cfg->calib_path_rel);
  cfg->capture.lock_path = abs_or_empty(root, cfg->lock_path_rel);

  // --- 相机 ---
  const int w = cfg->capture.width, h = cfg->capture.height;
  if (w < 320 || h < 240 || w > 4096 || h > 4096) {
    add(items, true,
        "camera.width/height = " + std::to_string(w) + "x" + std::to_string(h) +
            " 不合理（320x240 ~ 4096x4096）");
  }
  if (cfg->capture.fps < 1 || cfg->capture.fps > 30) {
    add(items, true, "camera.fps = " + std::to_string(cfg->capture.fps) +
                         " 超范围。本机 336L 实测可交付到 30；更高只会让下游丢帧。");
  } else if (cfg->capture.fps > 15) {
    add(items, false, "camera.fps = " + std::to_string(cfg->capture.fps) +
                          "：预算按 15 fps 量出来的（848x480 + 200 特征 ≈ 53 ms/帧）。");
  }
  if (w > 640) {
    add(items, false,
        "camera.width = " + std::to_string(w) + " 超出硬件 D2C 档（对齐只到 640x480）：相机服务里跑"
        "follow 会退回软对齐，把这份开销压到主机 CPU 上；独立跑 follow_pose 不受影响。");
  }
  if (cfg->capture.first_pair_timeout_ms < 500) {
    add(items, true, "camera.first_pair_timeout_ms = " +
                         std::to_string(cfg->capture.first_pair_timeout_ms) +
                         " 小于实测彩色暖机 333 ms 的安全余量，open() 会在没有彩色时返回。");
  }
  if (cfg->capture.frame_timeout_ms <= 0) {
    add(items, true, "camera.frame_timeout_ms 必须 > 0，否则取流卡死时进程不报错也不退出。");
  }
  if (cfg->capture.allow_unaligned) {
    add(items, true,
          "camera.allow_unaligned=true 会让深度和彩色各用一套内参（fx 624.013/cx 642 对 "
          "611.684/643.429，基线 23.735 mm），而 depth_to_cloud 只有一套 k —— 排障专用，不能上生产。");
  }
  if (cfg->capture.lock_path.empty()) {
    add(items, false,
        "camera.lock_path 为空：本进程不与相机服务互斥，两边同时开设备会得到未定义行为。");
  } else if (!std::filesystem::path(cfg->capture.lock_path).is_absolute()) {
    add(items, true,
        "camera.lock_path 必须是绝对路径（锁文件的意义在于与 cwd 无关）: " + cfg->capture.lock_path);
  }

  // --- 前端 ---
  if (cfg->frontend_kind != "cpu" && cfg->frontend_kind != "superpoint") {
    add(items, true, "frontend.kind = " + cfg->frontend_kind + " 不认识（cpu | superpoint）");
  }
  if (cfg->frontend_kind == "superpoint") {
    const std::string model = abs_or_empty(root, cfg->frontend.rknn_model_path);
    if (!is_regular_file(model)) {
      add(items, true, "frontend.kind=superpoint 但模型文件不存在: " + model +
                           "。rknn-toolkit2 只有 x86 版，本板转不出模型 —— 用 kind: cpu。");
    }
  }
  if (cfg->frontend.max_features < 16 || cfg->frontend.max_features > 2048) {
    add(items, true, "frontend.max_features = " + std::to_string(cfg->frontend.max_features) +
                         " 不合理（16~2048）");
  } else if (cfg->frontend.max_features > 200 && cfg->frontend_kind == "cpu") {
    add(items, false, "frontend.max_features=" + std::to_string(cfg->frontend.max_features) +
                          " 在 CPU 前端上超预算（真实 1280x800 工位图 400 特征全流程 93 ms）。");
  }
  if (cfg->frontend.min_distance_px < 4) {
    add(items, true, "frontend.min_distance_px = " + std::to_string(cfg->frontend.min_distance_px) +
                         " 会让角点在局部扎堆，互近邻的误配率显著上升。");
  }
  if (!(cfg->frontend.quality_level > 0.0) || cfg->frontend.quality_level > 1.0) {
    add(items, true, "frontend.quality_level 必须在 (0,1]。");
  }

  // --- 配准 ---
  const TrackParams& t = cfg->track;
  if (!(t.zmin_m > 0.0) || !(t.zmax_m > t.zmin_m) || t.zmax_m > 10.0) {
    add(items, true, "track.zmin_m/zmax_m = " + std::to_string(t.zmin_m) + "~" +
                         std::to_string(t.zmax_m) +
                         " m 不合理（0<zmin<zmax<=10）。数据集实测深度 0.74~2.59 m。");
  }
  if (t.depth_stride < 1 || t.depth_stride > 16) {
    add(items, true, "track.depth_stride = " + std::to_string(t.depth_stride) + " 应在 1~16。");
  }
  if (!(t.voxel_m > 0.001) || t.voxel_m > 0.2) {
    add(items, true, "track.voxel_m 应在 (0.001, 0.2] 米。");
  }
  if (!(t.max_corr_m > 0.0)) {
    add(items, true, "track.max_corr_m 必须 > 0。");
  } else if (t.max_corr_m > 2.6 * t.voxel_m) {
    add(items, false, "track.max_corr_m = " + std::to_string(t.max_corr_m) +
                          " 大于 2.6·voxel_m = " + std::to_string(2.6 * t.voxel_m) +
                          "：参考地图只在 3x3x3 体素邻域找最近邻，超出部分够不着，实际门限被截断。");
  }
  if (t.threads < 1 || t.threads > 8) {
    add(items, true, "track.threads = " + std::to_string(t.threads) +
                         " 应在 1~8（RK3588 4xA76+4xA55，绑 A76 用 4）。");
  }
  if (t.max_iters < 1) {
    add(items, true, "track.max_iters 必须 >= 1。");
  }
  if (!(t.min_inlier_ratio > 0.0) || t.min_inlier_ratio > 1.0) {
    add(items, true,
        "track.min_inlier_ratio 必须在 (0,1] —— 它是包络判据，越界等于关掉出包络检测。");
  }
  if (!(t.max_trans_sigma_mm > 0.0) || !(t.max_rot_sigma_deg > 0.0)) {
    add(items, true, "track.trans_sigma_mm / rot_sigma_deg 必须 > 0，否则可观测门恒假。");
  }
  if (!(t.max_group_anisotropy > 1.0)) {
    add(items, true, "track.group_anisotropy 必须 > 1。整幅大平面的各向异性实测 31.6/26.7，正常工件 "
                     "8.9~10.1 —— 取 <=1 把所有场景判成退化，取 20 以上会放过单一大平面。");
  }
  if (!(t.min_residual_var_scale > 0.0)) {
    add(items, true,
        "track.min_residual_var_scale 必须 > 0，否则残差恒为零时 σ 会塌成 0（假的「精确到无限」）。");
  }
  if (t.max_sparse_streak < 0 || t.max_sparse_streak > 200) {
    add(items, true, "track.max_sparse_streak = " + std::to_string(t.max_sparse_streak) +
                         " 不合理。稀疏解算的是帧间递推，攒久了就是无界漂移；15 帧 @15fps = 1 s。");
  }
  if (t.min_cloud_points < 12 || t.min_gicp_inliers < 12) {
    add(items, true, "track.min_cloud_points / min_gicp_inliers 至少 12，否则 6DoF 解没有意义。");
  }

  // --- 示教 ---
  if (cfg->map_path.empty()) {
    add(items, true, "teach.map_path 为空。");
  }
  if (cfg->teach_frames < 1 || cfg->teach_frames > 50) {
    add(items, true, "teach.frames = " + std::to_string(cfg->teach_frames) +
                         " 应在 1~50 之间（示教时间域多帧深度均值滤波去噪）。");
  }

  // --- 标定 / 安装方式 ---
  if (cfg->mount != "eye-to-hand") {
    add(items, true, "mount = " + cfg->mount +
                         "：眼在手上时相机在法兰上，臂一动图像就变，「工件动了」与「相机动了」在纯"
                         "视觉下不可辨识 —— 必须按帧从 30004 反馈口减掉 tool_vector_actual（~200 Hz，"
                         "无时间戳，还要做主机时钟对齐）。那是独立阶段，不是一个配置开关。");
  }
  if (!is_regular_file(cfg->calib_path)) {
    add(items, false, "runtime.calib_path 指向的文件不存在: " + cfg->calib_path +
                          "（dry-run 只报相机系修正量，不需要它；真要发臂时必须存在）。");
  }

  // --- 运行 ---
  if (cfg->health_port <= 0 || cfg->health_port > 65535) {
    add(items, true, "runtime.health_port = " + std::to_string(cfg->health_port) + " 不是合法端口。");
  } else if (cfg->health_port == 18080 || cfg->health_port == 8008) {
    add(items, true, "runtime.health_port = " + std::to_string(cfg->health_port) +
                         " 与相机服务（18080）/ ZLM（8008）冲突。follow 用 18081。");
  }
  if (cfg->max_cycles < 0) {
    add(items, true, "runtime.max_cycles 不能为负（0 = 一直跑）。");
  }
  if (cfg->enable_servo_p && cfg->dry_run) {
    add(items, true, "runtime.enable_servo_p=true 同时 dry_run=true 是矛盾配置：到底发不发臂？");
  }
  if (!cfg->dry_run) {
    add(items, false, "dry_run=false：本进程会向 CR5 发 ServoP（33 Hz，mm+deg，基座系绝对目标）。"
                      "确认臂路径上没有人、且急停可用。");
  }
  if (cfg->log_level != "debug" && cfg->log_level != "info" && cfg->log_level != "warn" &&
      cfg->log_level != "error") {
    add(items, false, "runtime.log_level = " + cfg->log_level + " 不认识，按 info 处理。");
  }

  // --- 机械臂镜像（值由 Python 后端消费，C++ 只负责"这份配置是不是自洽的"）---
  if (cfg->arm_mode != "sim" && cfg->arm_mode != "real") {
    add(items, true, "arm.mode = " + cfg->arm_mode + " 不认识（sim | real）");
  } else if (cfg->arm_mode == "real") {
    // **警告，不是错误**：follow 默认关闭，一个还没接上的开关绝不能让相机服务连推流都起不来。
    // 真正的拒绝发生在点击时（follow_service：真实臂控制路径未接入 P5）。
    add(items, false,
        "arm.mode=real：真实臂控制路径未接入（P5），跟随按钮会被后端拒绝 —— 仿真镜像请用 sim。");
  }
  if (cfg->arm_poll_hz < 1 || cfg->arm_poll_hz > 50) {
    add(items, false, "arm.poll_hz = " + std::to_string(cfg->arm_poll_hz) +
                          " 超出 1~50：后端会夹到区间端点再用（上界是 C++ 快照接口的实测容量，"
                          "再高只是把自板的轮询开销堆在取流线程旁边）。");
  }
  if (cfg->arm_emit_hz < 5 || cfg->arm_emit_hz > 100) {
    add(items, false, "arm.emit_hz = " + std::to_string(cfg->arm_emit_hz) +
                          " 超出 5~100：后端会夹到区间端点再用（关节流发射频率，仿真臂的刷新率就是它）。");
  }
  if (cfg->arm_max_joint_vel_deg_s <= 0.0 || cfg->arm_max_joint_vel_deg_s > 1000.0) {
    add(items, true, "arm.max_joint_vel_deg_s = " + std::to_string(cfg->arm_max_joint_vel_deg_s) +
                         " 不合理：轨迹平滑的关节角限速必须为正（参考 CR5 手动模式上限，典型 30~180）。");
  }
  if (cfg->teach_max_motion_deg_s < 0.0 || cfg->teach_max_motion_deg_s > 90.0) {
    add(items, false, "teach.max_motion_deg_s = " + std::to_string(cfg->teach_max_motion_deg_s) +
                          " 超出 0~90（0 = 关闭静止门）：按端点夹回再用。");
  }
  if (cfg->track.gyro_rot_gate_deg < 0.0) {
    add(items, false, "track.gyro_rot_gate_deg 为负，按 0（关闭离群门）处理。");
  }
  return out;
}

std::string describe(const FollowConfig& c) {
  std::ostringstream os;
  os << "配置来源=" << (c.source.empty() ? "内置默认" : c.source) << "\n";
  os << "  camera   : " << c.capture.width << "x" << c.capture.height << "@" << c.capture.fps
     << "  首帧对超时=" << c.capture.first_pair_timeout_ms << "ms  帧超时="
     << c.capture.frame_timeout_ms << "ms  allow_unaligned="
     << (c.capture.allow_unaligned ? "是" : "否") << "\n";
  os << "  lock     : " << (c.capture.lock_path.empty() ? "(不互斥)" : c.capture.lock_path) << "\n";
  os << "  frontend : " << c.frontend_kind << "  max_features=" << c.frontend.max_features
     << "  min_dist=" << c.frontend.min_distance_px << "px\n";
  os << "  track    : z " << c.track.zmin_m << "~" << c.track.zmax_m
     << " m  stride=" << c.track.depth_stride << "  voxel=" << c.track.voxel_m
     << " m  corr=" << c.track.max_corr_m << " m  threads=" << c.track.threads << "\n";
  os << "  gates    : sigma_t<" << c.track.max_trans_sigma_mm << "mm  sigma_r<"
     << c.track.max_rot_sigma_deg << "deg  aniso<" << c.track.max_group_anisotropy
     << "  inlier_ratio>=" << c.track.min_inlier_ratio
     << "  sparse_streak<=" << c.track.max_sparse_streak << "\n";
  os << "  teach    : " << c.map_path << "  frames=" << c.teach_frames
     << "  max_motion=" << c.teach_max_motion_deg_s << "deg/s\n";
  os << "  runtime  : mount=" << c.mount << "  dry_run=" << (c.dry_run ? "是" : "否")
     << "  servo_p=" << (c.enable_servo_p ? "开" : "关") << "  health=:" << c.health_port
     << "  max_cycles=" << c.max_cycles << "\n";
  os << "  arm      : mode=" << c.arm_mode << "  home=[";
  for (size_t i = 0; i < c.arm_home_joints_deg.size(); ++i) {
    os << (i ? "," : "") << c.arm_home_joints_deg[i];
  }
  os << "]deg  poll=" << c.arm_poll_hz << "Hz  emit=" << c.arm_emit_hz
     << "Hz  max_joint_vel=" << c.arm_max_joint_vel_deg_s
     << "deg/s  teach_save_map="
     << (c.arm_teach_save_map ? "是" : "否") << "  fallback_euler=[";
  for (size_t i = 0; i < c.arm_fallback_euler_deg.size(); ++i) {
    os << (i ? "," : "") << c.arm_fallback_euler_deg[i];
  }
  os << "]deg （本段由 Python 后端消费，见 follow_service）\n";
  return os.str();
}

}  // namespace follow
