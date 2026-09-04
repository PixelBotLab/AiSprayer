// P3 生成端。真扫描网格 + 真标定内参 + 命令 6DoF ⇒ 一串**真值由构造保证**的 RGB-D 帧。
//
// 为什么非要自己渲染：这台机器上没有相机，而"能算对"必须在这一轮就被证明，不能等硬件。
// 几何是实的（24937 顶点的扫描件）、内参是实的、尺度是实的，只有"动"是命令出来的 —— 所以
// 回放端和真值之间不存在"估计误差"以外的第二种误差来源。
//
// 参考系 = 示教帧（第 0 帧）的相机系，网格先按 place_mesh 烘进参考系再建 BVH，因此后面每
// 一帧只需要给一个 T_ref_cam。所有命令用**轴角**写，不写欧拉序列。
//
// 场景里**没有背景**：只有工件。加一块正面平行大面会把点云主体变成没有梯度的平面（P2 量过：
// 有效几何只占视野一小块时，面内平移和俯仰在代价里几乎可互换，"精度"是自洽的错觉），而出
// 包络那个用例也依赖"工件移出视野"真的能让对应点断掉。空掉的像素深度为 0，本来就会被 zmin
// 裁掉。大平面单独作为一个用例（wall），不在别的用例里污染几何。
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "follow/types.hpp"
#include "sequence_io.hpp"
#include "synth_scene.hpp"

namespace follow {
namespace synth {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kStandoffM = 0.65;         // 工件中心到相机的距离
constexpr double kVoxelM = 0.01;            // 比 P2 的 0.03 细一档：0.65 m 处工件只有 ~0.43 m
constexpr int64_t kFrameNs = 66'666'667;    // 15 fps

// 那份真标定值。出处 data/template_group/2026-08-25_215601/scan.params.yaml。
// 和 test_core / test_registration 里的 rigK() 是同一组数，故意不共享代码：三处独立写死，
// 一处被改错时另外两处会不一致地失败，比一个"看起来更干净"的共享常量更能守住真值。
CameraIntrinsics rig_k() {
  CameraIntrinsics k;
  k.fx = 611.683837890625;
  k.fy = 611.6983642578125;
  k.cx = 643.4285278320312;
  k.cy = 405.1534118652344;
  k.width = 1280;
  k.height = 800;
  return k;
}

// 命令位姿：平移 mm、旋转用轴角（分量模长 = 角度，单位 deg）。走 log_to_pose 这条与
// pose_to_log 配对的通道 —— 生成端与工具自检共用一份实现，欧拉序列没有机会被猜错。
Eigen::Isometry3d pose_of(double x_mm, double y_mm, double z_mm, double wx_deg, double wy_deg,
                          double wz_deg) {
  const double d2r = kPi / 180.0;
  return log_to_pose({x_mm * 1e-3, y_mm * 1e-3, z_mm * 1e-3, wx_deg * d2r, wy_deg * d2r,
                      wz_deg * d2r});
}

struct Step {
  std::string tag;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Expect expect = Expect::kOk;
  const Scene* scene = nullptr;  // null = 用用例的默认场景；非 null = 相机前换了别的东西
  std::string source;            // 空 = 用 Case::source。非空 = 这几帧的几何另有出处
  bool blank = false;     // 深度全零：模拟传感器这一帧什么都没回（走 kNoDepth）
  bool nocolor = false;   // 不出彩色：前端拿到空 FeatureFrame，只剩几何
};

struct Case {
  std::string name;
  const Scene* scene = nullptr;
  std::string source;  // 本用例几何的出处（写进清单做溯源）
  int block = 4;
  double noise_mm = 0.5;
  int blur_samples = 1;
  double blur_rot_deg = 0.0;
  double hole_fraction = 0.0;
  double texture_period_m = 0.03;
  uint32_t seed = 20260830u;
  std::vector<Step> steps;
};

Mesh bake(const Mesh& in, const Eigen::Isometry3d& T_ref_base) {
  Mesh out = in;
  for (auto& v : out.verts) {
    v = (T_ref_base * v.cast<double>()).cast<float>();
  }
  return out;
}

// 一整块正面平行大平面：喷涂工位最常见的退化源（车身侧板、工装台面）。细分只是给 BVH 东西
// 可分，几何上仍然共面，所以深度值和"两个三角形"的版本一模一样。
Mesh make_wall(double half_m, double z_m) {
  constexpr int n = 8;
  Mesh m;
  m.verts.reserve(static_cast<size_t>(n + 1) * (n + 1));
  for (int j = 0; j <= n; ++j) {
    for (int i = 0; i <= n; ++i) {
      m.verts.emplace_back(static_cast<float>(-half_m + 2.0 * half_m * i / n),
                           static_cast<float>(-half_m + 2.0 * half_m * j / n),
                           static_cast<float>(z_m));
    }
  }
  auto at = [n](int i, int j) { return static_cast<int32_t>(j * (n + 1) + i); };
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      m.tris.push_back({at(i, j), at(i + 1, j), at(i + 1, j + 1)});
      m.tris.push_back({at(i, j), at(i + 1, j + 1), at(i, j + 1)});
    }
  }
  return m;
}

Step mk(const std::string& tag, const Eigen::Isometry3d& T, Expect e = Expect::kOk) {
  Step s;
  s.tag = tag;
  s.T = T;
  s.expect = e;
  return s;
}

// 换件：相机前面出现的是一份**参考地图从没见过**的几何。这是 kOutOfEnvelope 真正的物理
// 来源 —— 不是"工件动得太多"（刚体平移且仍在视野里时，当前点云仍然逐点落在冻结的壳上，
// 出包络反而是错的），而是"看着像同一个工位，其实已经不是那个工件了"。
Step mk_at(const std::string& tag, const Scene* scene, const std::string& source,
           const Eigen::Isometry3d& T, Expect e) {
  Step s = mk(tag, T, e);
  s.scene = scene;
  s.source = source;
  return s;
}

bool emit_case(const std::string& root, const Case& c, const CameraIntrinsics& k,
               const std::string& mesh_path, std::string* err) {
  const std::string dir = root + "/" + c.name;
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    *err = "建目录失败：" + dir + "（" + ec.message() + "）";
    return false;
  }

  SequenceSpec s;
  s.name = c.name;
  s.source_mesh = c.source.empty() ? mesh_path : c.source;
  s.k = k;
  s.block = c.block;
  s.noise_mm = c.noise_mm;
  s.blur_samples = c.blur_samples;
  s.blur_rot_deg = c.blur_rot_deg;
  s.hole_fraction = c.hole_fraction;
  s.texture_period_m = c.texture_period_m;
  s.seed = c.seed;
  s.voxel_m = kVoxelM;
  s.depth_stride = 4;
  s.max_corr_m = 0.05;

  RenderParams rp;
  rp.block = c.block;
  rp.noise_mm = c.noise_mm;
  rp.blur_samples = c.blur_samples;
  rp.blur_rot_deg = c.blur_rot_deg;
  rp.hole_fraction = c.hole_fraction;

  ColorParams cp;
  cp.block = 1;  // 彩色逐像素：特征前端要看的就是像素级细节，块状彩色是在喂它一个假场景
  cp.texture_period_m = c.texture_period_m;

  char name[64];
  for (size_t i = 0; i < c.steps.size(); ++i) {
    const Step& st = c.steps[i];
    SequenceFrame f;
    f.index = static_cast<int>(i);
    f.ts_ns = static_cast<int64_t>(i) * kFrameNs;
    f.T_ref_cam = st.T;
    f.expect = st.expect;
    f.tag = st.tag;
    std::snprintf(name, sizeof(name), "%03d.depth.png", f.index);
    f.depth_file = name;
    if (!st.nocolor) {
      std::snprintf(name, sizeof(name), "%03d.color.png", f.index);
      f.color_file = name;
    }
    f.source = st.source;
    const Scene* sc = st.scene ? st.scene : c.scene;
    if (!sc) {
      *err = "第 " + std::to_string(f.index) + " 帧没有场景（Case.scene 和 Step.scene 都是空的）";
      return false;
    }

    cv::Mat depth;
    if (st.blank) {
      depth = cv::Mat::zeros(k.height, k.width, CV_16UC1);
    } else {
      rp.seed = c.seed + static_cast<uint32_t>(f.index);  // 每帧一份独立的噪声实现
      depth = render_depth(k, st.T, *sc, rp);
    }
    if (depth.empty()) {
      *err = "第 " + std::to_string(f.index) + " 帧渲染出空深度图（场景为空或内参非法）";
      return false;
    }
    if (i == 0 && !st.blank) {
      // 生成端最容易犯的错就藏在这里：工件整个跑到视野外，整份序列全是空图，回放端"全都
      // kNoDepth"也能自洽地跑完 —— 那 0 个证据却能打印出一张漂亮的表。示教帧必须有像素。
      // 几何判据必须**关掉丢点**来量：holes 用例的 50% 空洞是设计，不是场景空了。
      RenderParams geom = rp;
      geom.hole_fraction = 0.0;
      const cv::Mat full = render_depth(k, st.T, *sc, geom);
      const double geo_frac = full.empty()
                                  ? 0.0
                                  : static_cast<double>(cv::countNonZero(full)) /
                                        static_cast<double>(full.rows * full.cols);
      if (geo_frac < 0.05) {
        *err = "示教帧只有 " + std::to_string(geo_frac * 100.0) + "% 像素打到工件，场景基本是空的";
        return false;
      }
      // 绝对条数门槛：几何占比够但真空洞把点数削光时，参考地图会退化成几十个孤立体素，
      // 回放端照样能"自洽地"跑出一个好看的表。0.01 m 体素的地图至少要点到几千个点。
      const int kept = cv::countNonZero(depth);
      if (kept < 5000) {
        *err = "示教帧只有 " + std::to_string(kept) + " 个有效像素（< 5000），参考地图立不住";
        return false;
      }
    }
    cv::Mat color;
    if (!st.nocolor) {
      cp.seed = c.seed + static_cast<uint32_t>(f.index);
      color = render_color(k, st.T, *sc, cp);
      if (color.empty()) {
        *err = "第 " + std::to_string(f.index) + " 帧渲染出空彩色图";
        return false;
      }
    }
    if (!write_frame(dir, f, depth, color, err)) {
      return false;
    }
    s.frames.push_back(f);
  }
  return save_sequence(dir, s, err);
}

// 一维台阶序列：先出去再回来。相邻帧最大台阶 10 mm —— 15 fps 下这是"工件在动"，不是跳变，
// 所以它测的是跟踪而不是重新捕获。
std::vector<Step> ramp(char axis, const double* amps, int n) {
  std::vector<Step> v;
  char tag[24];
  for (int i = 0; i < n; ++i) {
    std::snprintf(tag, sizeof(tag), "%c%.0f", axis, amps[i]);
    const double a = amps[i];
    const Eigen::Isometry3d T = axis == 'x'     ? pose_of(a, 0, 0, 0, 0, 0)
                                : axis == 'y'   ? pose_of(0, a, 0, 0, 0, 0)
                                                : pose_of(0, 0, a, 0, 0, 0);
    v.push_back(mk(tag, T));
  }
  return v;
}

std::vector<Case> build_cases(const Scene& piece, const Scene& wall) {
  std::vector<Case> all;

  {  // 静止：修正量恒为零，看的是 bias（噪声是唯一来源）和 σ 有没有说实话
    Case c;
    c.name = "zero";
    c.scene = &piece;
    for (int i = 0; i < 8; ++i) {
      c.steps.push_back(mk("zero" + std::to_string(i), Eigen::Isometry3d::Identity()));
    }
    all.push_back(c);
  }

  {  // 平移扫描：三根轴各自出去再回来，幅度 1/5/10/20 mm
    Case c;
    c.name = "scan";
    c.scene = &piece;
    static const double ax[] = {0, 1, 5, 10, 20, 10, 5, 1, 0};
    static const double ay[] = {2, 5, 10, 20, 10, 5, 2};
    static const double az[] = {2, 5, 10, 20, 10, 5, 2};
    c.steps = ramp('x', ax, 9);
    for (auto& s : ramp('y', ay, 7)) c.steps.push_back(s);
    c.steps.push_back(mk("home", Eigen::Isometry3d::Identity()));
    for (auto& s : ramp('z', az, 7)) c.steps.push_back(s);
    all.push_back(c);
  }

  {  // 旋转扫描：三轴各自一遍。哪根轴最差让报告说，不让注释先下结论
    Case c;
    c.name = "rot";
    c.scene = &piece;
    static const double a[] = {0.1, 0.2, 0.5, 1.0, 0.5, 0.2, 0.1};
    c.steps.push_back(mk("teach", Eigen::Isometry3d::Identity()));
    for (int ax = 0; ax < 3; ++ax) {
      char tag[24];
      for (int i = 0; i < 7; ++i) {
        std::snprintf(tag, sizeof(tag), "r%c%.1f", 'x' + ax, a[i]);
        const double d = a[i];
        const Eigen::Isometry3d T = ax == 0     ? pose_of(0, 0, 0, d, 0, 0)
                                    : ax == 1   ? pose_of(0, 0, 0, 0, d, 0)
                                                : pose_of(0, 0, 0, 0, 0, d);
        c.steps.push_back(mk(tag, T));
      }
      std::snprintf(tag, sizeof(tag), "r%chome", 'x' + ax);
      c.steps.push_back(mk(tag, Eigen::Isometry3d::Identity()));
    }
    all.push_back(c);
  }

  {  // 大平面：至少三个自由度不可观，必须报 kDegenerate。x20 那一帧是本用例的重点 ——
    // 面内平移没有任何梯度约束，求解器会留在初值上（误差 = 命令量），而绝对 σ 反而比好
    // 场景还小。能把它和"真准"分开的只有各向异性比值，见 AbsoluteSigmaAloneWouldPassThePlane。
    Case c;
    c.name = "wall";
    c.scene = &wall;
    c.source = "make_wall(half=1.0, z=0.65) 生成的正面平行平面，不是扫描件";
    c.steps = {mk("teach", Eigen::Isometry3d::Identity(), Expect::kDegenerate),
               mk("wx10", pose_of(10, 0, 0, 0, 0, 0), Expect::kDegenerate),
               mk("wy10", pose_of(0, 10, 0, 0, 0, 0), Expect::kDegenerate),
               mk("wz10", pose_of(0, 0, 10, 0, 0, 0), Expect::kDegenerate),
               mk("wrz1", pose_of(0, 0, 0, 0, 0, 1.0), Expect::kDegenerate),
               mk("wx20", pose_of(20, 0, 0, 0, 0, 0), Expect::kDegenerate)};
    all.push_back(c);
  }

  {  // 无纹理有几何：特征一条有用的都找不到，只剩稠密几何 —— 前端挂掉时系统该还能跟
    Case c;
    c.name = "textureless";
    c.scene = &piece;
    c.texture_period_m = -1.0;
    c.steps = {mk("teach", Eigen::Isometry3d::Identity()),
               mk("tx5", pose_of(5, 0, 0, 0, 0, 0)),
               mk("ty5", pose_of(0, 5, 0, 0, 0, 0)),
               mk("tz5", pose_of(0, 0, 5, 0, 0, 0)),
               mk("tx10", pose_of(10, 0, 0, 0, 0, 0)),
               mk("try0.5", pose_of(0, 0, 0, 0, 0.5, 0))};
    all.push_back(c);
  }

  {  // 快转 + 曝光模糊：绕相机自身 y 扫过 2°，逐子位姿取最近命中（不平均深度）
    Case c;
    c.name = "blur";
    c.scene = &piece;
    c.blur_samples = 5;
    c.blur_rot_deg = 2.0;
    static const double a[] = {0.0, 0.5, 1.0, 1.5, 2.0, 1.5, 1.0, 0.5};
    char tag[24];
    for (int i = 0; i < 8; ++i) {
      std::snprintf(tag, sizeof(tag), "blur%.1f", a[i]);
      c.steps.push_back(mk(tag, pose_of(0, 0, 0, 0, a[i], 0)));
    }
    all.push_back(c);
  }

  {  // 丢点：一半像素没有回波。黑漆反光/量程外的日常形态，不是极端情况
    Case c;
    c.name = "holes";
    c.scene = &piece;
    c.hole_fraction = 0.5;
    c.steps = {mk("teach", Eigen::Isometry3d::Identity()),
               mk("hx5", pose_of(5, 0, 0, 0, 0, 0)),
               mk("hz5", pose_of(0, 0, 5, 0, 0, 0)),
               mk("hx10", pose_of(10, 0, 0, 0, 0, 0))};
    all.push_back(c);
  }

  {  // 跟丢 → 恢复。这里要先把一个直觉钉死：**"动得很多"不等于"出包络"**。250 mm 横向跳变
    // 在 650 mm 工作距离、fx=611.7 下把工件挪 ~235 px（工件本身 ~404 px 宽），它仍在视野里，
    // 刚体几何一字未改，当前点云逐点还落在冻结的壳上 —— 正确答案就是"跟住了，在新位置"。
    // 把这种帧报成出包络会让人去重新示教一个根本没变的工位，那是比丢跟踪更贵的错误。
    // 真正出包络的是"看到的已经不是那个工件"，单独一个用例（swap）测。
    Case c;
    c.name = "lost";
    c.scene = &piece;
    Step blank = mk("blank", Eigen::Isometry3d::Identity(), Expect::kLost);
    blank.blank = true;
    c.steps = {mk("teach", Eigen::Isometry3d::Identity()),
               mk("x5", pose_of(5, 0, 0, 0, 0, 0)),
               mk("jump250", pose_of(250, 0, 0, 0, 0, 0)),
               mk("jump150", pose_of(150, 0, 0, 0, 0, 0)),
               blank,
               mk("back5", pose_of(5, 0, 0, 0, 0, 0)),
               mk("back10", pose_of(10, 0, 0, 0, 0, 0)),
               mk("back0", Eigen::Isometry3d::Identity())};
    all.push_back(c);
  }

  {  // 换件 = 唯一的出包络用例。参考地图是示教时冻结的那层壳，工装上换了别的东西之后，当前
    // 点云落在地图上根本没有几何的地方 ⇒ 对应点断掉。连续两帧都必须是 kOutOfEnvelope（它是
    // 一个持续状态，不是单帧毛刺），而工件回来后的**第一帧**就要恢复（T_vel_ 在 hold 里重置
    // 过，没重置的话这里会看到一段"故障前的外推接着推"的跳变）。
    Case c;
    c.name = "swap";
    c.scene = &piece;
    const std::string other = "make_wall(half=1.0, z=0.65)：工装上换成了别的东西（大平面）";
    c.steps = {mk("teach", Eigen::Isometry3d::Identity()),
               mk("x5", pose_of(5, 0, 0, 0, 0, 0)),
               mk_at("swapped", &wall, other, Eigen::Isometry3d::Identity(), Expect::kOutOfEnv),
               mk_at("still", &wall, other, pose_of(5, 0, 0, 0, 0, 0), Expect::kOutOfEnv),
               mk("piece_back", pose_of(5, 0, 0, 0, 0, 0)),
               mk("piece_home", Eigen::Isometry3d::Identity())};
    all.push_back(c);
  }

  {  // 只有深度没有彩色：取流层只回深度时的样子（前端整条链路缺席）
    Case c;
    c.name = "depthonly";
    c.scene = &piece;
    Step a = mk("teach", Eigen::Isometry3d::Identity());
    Step b = mk("dx5", pose_of(5, 0, 0, 0, 0, 0));
    Step d = mk("dz10", pose_of(0, 0, 10, 0, 0, 0));
    a.nocolor = b.nocolor = d.nocolor = true;
    c.steps = {a, b, d};
    all.push_back(c);
  }

  return all;
}

int gen_main(int argc, char** argv) {
  std::string out = "app/src/core/follow/out/synth";
  std::string mesh = std::string(FOLLOW_SCAN_DIR) + "/scan.mesh.ply";
  std::string only;
  bool have_seed = false;
  uint32_t seed = 0;  // 未给 --seed 时用各用例里写死的默认种子（证据要能一字不变地复现）
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](std::string* dst) {
      if (i + 1 >= argc) {
        return false;
      }
      *dst = argv[++i];
      return true;
    };
    auto next_seed = [&](uint32_t* dst) {
      std::string v;
      if (!next(&v)) {
        return false;
      }
      char* end = nullptr;
      const unsigned long parsed = std::strtoul(v.c_str(), &end, 10);
      if (end == v.c_str() || *end != '\0' || parsed > 4294967200UL) {
        return false;
      }
      *dst = static_cast<uint32_t>(parsed);
      return true;
    };
    if (a == "--out") {
      if (!next(&out)) return 2;
    } else if (a == "--mesh") {
      if (!next(&mesh)) return 2;
    } else if (a == "--only") {
      if (!next(&only)) return 2;
    } else if (a == "--seed") {
      if (!next_seed(&seed)) {
        std::fprintf(stderr, "--seed 必须是 [0, 4294967200] 的整数（每帧还要加帧号，不能回绕）\n");
        return 2;
      }
      have_seed = true;
    } else {
      std::printf("用法: gen_synth [--out 目录] [--mesh x.ply] [--only case] [--seed N]\n");
      return 2;
    }
  }

  std::string err;
  Mesh raw;
  if (!load_ply(mesh, &raw, &err)) {
    std::fprintf(stderr, "网格读取失败 %s: %s\n", mesh.c_str(), err.c_str());
    return 1;
  }
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& v : raw.verts) {
    centroid += v.cast<double>();
  }
  centroid /= static_cast<double>(raw.verts.size());
  const Scene piece(bake(raw, place_mesh(centroid, kStandoffM)));
  const Scene wall(make_wall(1.0, kStandoffM));
  if (!piece.valid() || !wall.valid()) {
    std::fprintf(stderr, "场景构建失败\n");
    return 1;
  }
  const Eigen::Vector3f sz = raw.bbox().sizes();
  std::printf("mesh %s\n  %zu verts %zu tris  bbox %.3f x %.3f x %.3f m  centroid (%.3f, %.3f, %.3f)\n",
              mesh.c_str(), raw.verts.size(), raw.tri_count(), sz.x(), sz.y(), sz.z(), centroid.x(),
              centroid.y(), centroid.z());

  int made = 0;
  uint32_t used_seed = 0;
  for (const Case& proto : build_cases(piece, wall)) {
    if (!only.empty() && only != proto.name) {
      continue;
    }
    Case c = proto;
    if (have_seed) {
      c.seed = seed;
    }
    used_seed = c.seed;
    err.clear();
    if (!emit_case(out, c, rig_k(), mesh, &err)) {
      std::fprintf(stderr, "用例 %s 生成失败: %s\n", c.name.c_str(), err.c_str());
      return 1;
    }
    std::printf("  %-12s %2zu 帧 -> %s/%s\n", c.name.c_str(), c.steps.size(), out.c_str(),
                c.name.c_str());
    ++made;
  }
  if (made == 0) {
    std::fprintf(stderr, "一个用例都没生成（--only 拼错了？）\n");
    return 1;
  }
  // 报告的数字只有和噪声实现一起看才有意义：换种子重跑是蒙特卡洛，不是"复现失败"。
  std::printf("  base seed = %u（每帧 seed = base + 帧号）\n", used_seed);
  return 0;
}

}  // namespace
}  // namespace synth
}  // namespace follow

int main(int argc, char** argv) {
  return follow::synth::gen_main(argc, argv);
}
