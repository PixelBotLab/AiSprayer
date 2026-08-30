#include "sequence_io.hpp"

#include <sys/stat.h>

#include <cmath>
#include <cstdio>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

namespace follow {
namespace synth {
namespace {

constexpr const char* kManifest = "sequence.yaml";

std::string join(const std::string& dir, const std::string& name) {
  if (name.empty()) {
    return {};
  }
  if (name.front() == '/') {
    return name;  // 已是绝对路径：调用方给的就是完整路径
  }
  if (dir.empty()) {
    return name;
  }
  return dir + (dir.back() == '/' ? "" : "/") + name;
}

bool is_dir(const std::string& p) {
  struct stat st {};
  return ::stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool is_regular(const std::string& p) {
  struct stat st {};
  return ::stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

// 清单里少了/坏了任何一个字段的报错都要指名道姓：一份读不完的清单必须当场拒掉，
// 让生成端去改，而不是回落到默认值上跑出个"看着很象话"的错误结果。
double need_double(const YAML::Node& m, const std::string& key, std::string* err) {
  const YAML::Node v = m[key];
  if (!v) {
    *err = "清单缺字段 " + key;
    return 0.0;
  }
  try {
    const double d = v.as<double>();
    if (!std::isfinite(d)) {
      *err = "清单字段 " + key + " 不是有限数";
    }
    return d;
  } catch (const YAML::Exception&) {
    *err = "清单字段 " + key + " 不是数值";
    return 0.0;
  }
}

int need_int(const YAML::Node& m, const std::string& key, std::string* err) {
  const YAML::Node v = m[key];
  if (!v) {
    *err = "清单缺字段 " + key;
    return 0;
  }
  try {
    return v.as<int>();
  } catch (const YAML::Exception&) {
    *err = "清单字段 " + key + " 不是整数";
    return 0;
  }
}

long long need_i64(const YAML::Node& m, const std::string& key, std::string* err) {
  const YAML::Node v = m[key];
  if (!v) {
    *err = "清单缺字段 " + key;
    return 0;
  }
  try {
    return v.as<long long>();
  } catch (const YAML::Exception&) {
    *err = "清单字段 " + key + " 不是整数";
    return 0;
  }
}

uint32_t need_u32(const YAML::Node& m, const std::string& key, std::string* err) {
  const long long ll = need_i64(m, key, err);
  if (ll < 0 || ll > 4294967295LL) {
    if (err->empty()) {
      *err = "清单字段 " + key + " 超出 uint32 范围";
    }
    return 0;
  }
  return static_cast<uint32_t>(ll);
}

std::string need_string(const YAML::Node& m, const std::string& key, std::string* err) {
  const YAML::Node v = m[key];
  if (!v) {
    *err = "清单缺字段 " + key;
    return {};
  }
  try {
    return v.as<std::string>();
  } catch (const YAML::Exception&) {
    *err = "清单字段 " + key + " 不是字符串";
    return {};
  }
}

std::vector<double> need_seq(const YAML::Node& m, const std::string& key, size_t n,
                             std::string* err) {
  const YAML::Node v = m[key];
  std::vector<double> out;
  if (!v || !v.IsSequence() || v.size() != n) {
    *err = "清单字段 " + key + " 应为 " + std::to_string(n) + " 个数值";
    return out;
  }
  try {
    for (const auto& e : v) {
      const double d = e.as<double>();
      if (!std::isfinite(d)) {
        *err = "清单字段 " + key + " 含非有限值";
        return {};
      }
      out.push_back(d);
    }
  } catch (const YAML::Exception&) {
    *err = "清单字段 " + key + " 元素不是数值";
    out.clear();
  }
  return out;
}

void emit_pose(YAML::Emitter& y, const Eigen::Isometry3d& T) {
  const Eigen::Vector3d t = T.translation();
  const Eigen::Matrix3d R = T.linear();
  y << YAML::Key << "t" << YAML::Value << YAML::BeginSeq << t.x() << t.y() << t.z() << YAML::EndSeq;
  // 行主序 9 个：存矩阵不存欧拉角/轴角，是为了回放端不做任何"约定假设"。
  y << YAML::Key << "R" << YAML::Value << YAML::BeginSeq;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      y << R(i, j);
    }
  }
  y << YAML::EndSeq;
}

Expect expect_from(const std::string& s, bool* ok) {
  *ok = true;
  if (s == "ok") return Expect::kOk;
  if (s == "degenerate") return Expect::kDegenerate;
  if (s == "lost") return Expect::kLost;
  if (s == "out_of_envelope") return Expect::kOutOfEnv;
  *ok = false;
  return Expect::kOk;
}

}  // namespace

const char* to_string(Expect e) {
  switch (e) {
    case Expect::kOk: return "ok";
    case Expect::kDegenerate: return "degenerate";
    case Expect::kLost: return "lost";
    case Expect::kOutOfEnv: return "out_of_envelope";
  }
  return "unknown";
}

bool expect_accepts(Expect want, Status got) {
  switch (want) {
    case Expect::kOk:
      return got == Status::kOk;
    case Expect::kDegenerate:
      // 刻意不留余地：这一帧存在的意义就是"解算器知道自己不可观"。报 kOk 是 bug，
      // 报 kLost 说明它连场景都没看清，同样不能算通过。
      return got == Status::kDegenerate;
    case Expect::kLost:
      return got == Status::kLost || got == Status::kNoDepth;
    case Expect::kOutOfEnv:
      // 出包络和跟丢必须是两个答复：前者要重新示教，后者会自己恢复。混起来运维就废了。
      return got == Status::kOutOfEnvelope;
  }
  return false;
}

bool save_sequence(const std::string& dir, const SequenceSpec& s, std::string* err) {
  if (!is_dir(dir)) {
    *err = "输出目录不存在：" + dir + "（生成端负责建，避免把结果写进打错的路径）";
    return false;
  }
  YAML::Emitter y;
  y << YAML::BeginMap;
  y << YAML::Key << "version" << YAML::Value << s.version;
  y << YAML::Key << "name" << YAML::Value << s.name;
  y << YAML::Key << "source_mesh" << YAML::Value << s.source_mesh;
  y << YAML::Key << "intrinsics" << YAML::Value << YAML::BeginMap;
  y << YAML::Key << "fx" << YAML::Value << s.k.fx;
  y << YAML::Key << "fy" << YAML::Value << s.k.fy;
  y << YAML::Key << "cx" << YAML::Value << s.k.cx;
  y << YAML::Key << "cy" << YAML::Value << s.k.cy;
  y << YAML::Key << "width" << YAML::Value << s.k.width;
  y << YAML::Key << "height" << YAML::Value << s.k.height;
  y << YAML::EndMap;
  y << YAML::Key << "render" << YAML::Value << YAML::BeginMap;
  y << YAML::Key << "block" << YAML::Value << s.block;
  y << YAML::Key << "noise_mm" << YAML::Value << s.noise_mm;
  y << YAML::Key << "blur_samples" << YAML::Value << s.blur_samples;
  y << YAML::Key << "blur_rot_deg" << YAML::Value << s.blur_rot_deg;
  y << YAML::Key << "hole_fraction" << YAML::Value << s.hole_fraction;
  y << YAML::Key << "texture_period_m" << YAML::Value << s.texture_period_m;
  y << YAML::Key << "seed" << YAML::Value << static_cast<long long>(s.seed);
  y << YAML::EndMap;
  y << YAML::Key << "tracking" << YAML::Value << YAML::BeginMap;
  y << YAML::Key << "voxel_m" << YAML::Value << s.voxel_m;
  y << YAML::Key << "depth_stride" << YAML::Value << s.depth_stride;
  y << YAML::Key << "max_corr_m" << YAML::Value << s.max_corr_m;
  y << YAML::Key << "zmin_m" << YAML::Value << s.zmin_m;
  y << YAML::Key << "zmax_m" << YAML::Value << s.zmax_m;
  y << YAML::EndMap;
  y << YAML::Key << "frames" << YAML::Value << YAML::BeginSeq;
  for (const auto& f : s.frames) {
    y << YAML::BeginMap;
    y << YAML::Key << "index" << YAML::Value << f.index;
    y << YAML::Key << "depth" << YAML::Value << f.depth_file;
    if (!f.color_file.empty()) {
      y << YAML::Key << "color" << YAML::Value << f.color_file;
    }
    y << YAML::Key << "ts_ns" << YAML::Value << static_cast<long long>(f.ts_ns);
    y << YAML::Key << "expect" << YAML::Value << to_string(f.expect);
    if (!f.tag.empty()) {
      y << YAML::Key << "tag" << YAML::Value << f.tag;
    }
    if (!f.source.empty()) {
      y << YAML::Key << "source" << YAML::Value << f.source;
    }
    emit_pose(y, f.T_ref_cam);
    y << YAML::EndMap;
  }
  y << YAML::EndSeq;
  y << YAML::EndMap;

  std::FILE* fp = std::fopen(join(dir, kManifest).c_str(), "wb");
  if (!fp) {
    *err = "清单写入失败：" + join(dir, kManifest);
    return false;
  }
  const std::string out = std::string(y.c_str()) + "\n";
  const size_t written = std::fwrite(out.data(), 1, out.size(), fp);
  const bool flush_ok = std::fclose(fp) == 0;
  if (written != out.size() || !flush_ok) {
    *err = "清单未写完：" + join(dir, kManifest);
    return false;
  }
  return true;
}

bool load_sequence(const std::string& dir, SequenceSpec* s, std::string* err) {
  const std::string path = join(dir, kManifest);
  if (!is_regular(path)) {
    *err = "找不到清单：" + path;
    return false;
  }
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& ex) {
    *err = "清单解析失败：" + std::string(ex.what());
    return false;
  }
  if (!root.IsMap()) {
    *err = "清单顶层不是映射：" + path;
    return false;
  }
  SequenceSpec out;
  auto fail = [&](const std::string& msg) {
    *err = msg;
    return false;
  };

  const int version = need_int(root, "version", err);
  if (!err->empty()) return false;
  if (version != 1) {
    return fail("清单版本 " + std::to_string(version) + " 不被本回放端支持（只认 1）");
  }
  out.version = version;
  out.name = need_string(root, "name", err);
  if (!err->empty()) return false;
  out.source_mesh = need_string(root, "source_mesh", err);
  if (!err->empty()) return false;

  const YAML::Node kin = root["intrinsics"];
  if (!kin) {
    return fail("清单缺 intrinsics");
  }
  out.k.fx = need_double(kin, "fx", err);
  out.k.fy = need_double(kin, "fy", err);
  out.k.cx = need_double(kin, "cx", err);
  out.k.cy = need_double(kin, "cy", err);
  out.k.width = need_int(kin, "width", err);
  out.k.height = need_int(kin, "height", err);
  if (!err->empty()) return false;
  if (!out.k.valid()) {
    // 内参非法时 unproject 会造出 inf/NaN 点云，而 NaN 躲得过所有范围比较 —— 必须在入口拒。
    return fail("清单内参非法（fx/fy/width/height）");
  }

  const YAML::Node rin = root["render"];
  if (!rin) {
    return fail("清单缺 render");
  }
  out.block = need_int(rin, "block", err);
  out.noise_mm = need_double(rin, "noise_mm", err);
  out.blur_samples = need_int(rin, "blur_samples", err);
  out.blur_rot_deg = need_double(rin, "blur_rot_deg", err);
  out.hole_fraction = need_double(rin, "hole_fraction", err);
  out.texture_period_m = need_double(rin, "texture_period_m", err);
  out.seed = need_u32(rin, "seed", err);
  if (!err->empty()) return false;
  // 越界的渲染参数不会崩，只会悄悄换一个场景 —— 量出来就是"看着很象话的错误精度"。
  if (out.block < 1 || out.blur_samples < 1 || out.noise_mm < 0.0 || out.blur_rot_deg < 0.0 ||
      out.hole_fraction < 0.0 || out.hole_fraction >= 1.0) {
    return fail("清单 render 参数越界（block/blur_samples/noise_mm/blur_rot_deg/hole_fraction）");
  }

  const YAML::Node tin = root["tracking"];
  if (!tin) {
    return fail("清单缺 tracking");
  }
  out.voxel_m = need_double(tin, "voxel_m", err);
  out.depth_stride = need_int(tin, "depth_stride", err);
  out.max_corr_m = need_double(tin, "max_corr_m", err);
  out.zmin_m = need_double(tin, "zmin_m", err);
  out.zmax_m = need_double(tin, "zmax_m", err);
  if (!err->empty()) return false;
  if (out.voxel_m <= 0.0 || out.depth_stride < 1 || out.max_corr_m <= 0.0) {
    return fail("清单 tracking 参数非正（voxel_m/depth_stride/max_corr_m）");
  }
  if (!(out.zmin_m > 0.0) || out.zmax_m <= out.zmin_m) {
    return fail("清单 tracking 深度范围非法（zmin_m/zmax_m）");
  }

  const YAML::Node fs = root["frames"];
  if (!fs || !fs.IsSequence() || fs.size() == 0) {
    return fail("清单 frames 缺失或为空");
  }
  for (const auto& fn : fs) {
    if (!fn.IsMap()) {
      return fail("清单里有一帧不是映射");
    }
    SequenceFrame f;
    f.index = need_int(fn, "index", err);
    f.depth_file = need_string(fn, "depth", err);
    if (!err->empty()) return false;
    f.ts_ns = need_i64(fn, "ts_ns", err);
    if (!err->empty()) return false;
    if (fn["color"]) {
      f.color_file = need_string(fn, "color", err);
      if (!err->empty()) return false;
    }
    if (fn["tag"]) {
      f.tag = need_string(fn, "tag", err);
      if (!err->empty()) return false;
    }
    const std::string exp = fn["expect"] ? need_string(fn, "expect", err) : std::string("ok");
    if (!err->empty()) return false;
    bool ok = false;
    f.expect = expect_from(exp, &ok);
    if (!ok) {
      return fail("清单第 " + std::to_string(f.index) + " 帧 expect 值未知：" + exp);
    }
    const std::vector<double> t = need_seq(fn, "t", 3, err);
    const std::vector<double> rmat = need_seq(fn, "R", 9, err);
    if (!err->empty()) return false;
    // 只查正交性到 1e-6，不重投影：清单里的位姿是真值，把它"修"到正交会掩盖生成端的 bug。
    Eigen::Matrix3d R;
    R << rmat[0], rmat[1], rmat[2], rmat[3], rmat[4], rmat[5], rmat[6], rmat[7], rmat[8];
    if ((R * R.transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-6 || R.determinant() < 0.0) {
      return fail("清单第 " + std::to_string(f.index) + " 帧 R 不是正常旋转矩阵");
    }
    f.T_ref_cam = Eigen::Isometry3d(R);
    f.T_ref_cam.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
    out.frames.push_back(f);
  }
  for (int i = 0; i < static_cast<int>(out.frames.size()); ++i) {
    if (out.frames[i].index != i) {
      return fail("清单 frames 下标不连续（第 " + std::to_string(i) + " 项 index=" +
                  std::to_string(out.frames[i].index) + "）");
    }
    // 时间戳倒退/重复会让 odometry 报 kStaleInput —— 那测的是清单的错，不是算法的错。
    if (i > 0 && out.frames[i].ts_ns <= out.frames[i - 1].ts_ns) {
      return fail("清单第 " + std::to_string(i) + " 帧 ts_ns 不是严格递增");
    }
  }
  const Eigen::Matrix3d R0 = out.frames.front().T_ref_cam.linear();
  const Eigen::Vector3d t0 = out.frames.front().T_ref_cam.translation();
  if ((R0 - Eigen::Matrix3d::Identity()).norm() > 1e-9 || t0.norm() > 1e-9) {
    return fail("清单第 0 帧不是单位变换：参考系必须就是示教帧的相机系（见 sequence_io.hpp）");
  }
  *s = out;
  return true;
}

bool write_frame(const std::string& dir, const SequenceFrame& f, const cv::Mat& depth_mm,
                 const cv::Mat& color_bgr, std::string* err) {
  if (f.depth_file.empty()) {
    *err = "帧 " + std::to_string(f.index) + " 没有 depth 文件名";
    return false;
  }
  if (depth_mm.empty() || depth_mm.type() != CV_16UC1) {
    *err = "深度图必须是 CV_16UC1 毫米（帧 " + std::to_string(f.index) + "）";
    return false;
  }
  if (!color_bgr.empty()) {
    if (color_bgr.type() != CV_8UC3) {
      *err = "彩色图必须是 CV_8UC3（帧 " + std::to_string(f.index) + "）";
      return false;
    }
    if (color_bgr.size() != depth_mm.size()) {
      *err = "彩色/深度尺寸不一致（帧 " + std::to_string(f.index) + "）";
      return false;
    }
    if (f.color_file.empty()) {
      *err = "传了彩色图但清单这帧没记 color 文件名（帧 " + std::to_string(f.index) + "）";
      return false;
    }
  } else if (!f.color_file.empty()) {
    *err = "清单记了 color 文件名但渲染没出图（帧 " + std::to_string(f.index) + "）";
    return false;
  }
  // 两份都是 PNG：无损压缩。合成序列是验收证据，JPEG 的块效应会伪装成"边缘特征变差"。
  if (!cv::imwrite(join(dir, f.depth_file), depth_mm)) {
    *err = "深度图写入失败：" + join(dir, f.depth_file);
    return false;
  }
  if (!color_bgr.empty() && !cv::imwrite(join(dir, f.color_file), color_bgr)) {
    *err = "彩色图写入失败：" + join(dir, f.color_file);
    return false;
  }
  return true;
}

bool read_frame(const std::string& dir, const SequenceFrame& f, cv::Mat* depth_mm,
                cv::Mat* color_bgr, std::string* err) {
  const std::string dpath = join(dir, f.depth_file);
  *depth_mm = cv::imread(dpath, cv::IMREAD_UNCHANGED);
  if (depth_mm->empty()) {
    *err = "深度图读不到：" + dpath;
    return false;
  }
  if (depth_mm->type() != CV_16UC1) {
    *err = "深度图类型不是 CV_16UC1：" + dpath;
    return false;
  }
  if (f.color_file.empty()) {
    if (color_bgr) {
      color_bgr->release();
    }
    return true;
  }
  if (!color_bgr) {
    *err = "这一帧有彩色图但调用方没给输出缓冲";
    return false;
  }
  const std::string cpath = join(dir, f.color_file);
  *color_bgr = cv::imread(cpath, cv::IMREAD_COLOR);
  if (color_bgr->empty()) {
    *err = "彩色图读不到：" + cpath;
    return false;
  }
  if (color_bgr->size() != depth_mm->size()) {
    *err = "彩色/深度尺寸不一致：" + cpath;
    return false;
  }
  return true;
}

Eigen::Isometry3d place_mesh(const Eigen::Vector3d& centroid_base, double distance_m) {
  // 真实标定（data/calib/calib_20260826_184313）的列向量大意是 camX→base −Y、
  // camY→base −Z、camZ→base +X。这里就近取到精确的轴置换：差的那 ~0.5° 是标定残差，
  // 不是 P3 要测的东西 —— P3 要测的是"给定已知运动，能不能量回来"。
  Eigen::Matrix3d R_cam_to_base;
  R_cam_to_base << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  Eigen::Isometry3d T_ref_base = Eigen::Isometry3d::Identity();
  T_ref_base.linear() = R_cam_to_base.transpose();  // 参考系 = 示教相机系 ⇒ base→cam
  // 质心落到光轴上距相机 distance_m 处：t = (0,0,d) − R·centroid。
  T_ref_base.translation() =
      Eigen::Vector3d(0.0, 0.0, distance_m) - T_ref_base.linear() * centroid_base;
  return T_ref_base;
}

}  // namespace synth
}  // namespace follow
