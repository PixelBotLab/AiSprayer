// 配置层与健康快照的回归。全部是纯逻辑，不需要相机、不需要网络 —— 一台没插设备的机器
// 必须能把这些检查跑绿，这也是 follow_app 单独成层而不长进 libfollow 的理由之一。
//
// 这里钉住的不是"能读到值"，而是三条失败模式：
//  1) 类型写错的键**必须指名道姓报出来**（静默用默认值 = 运维以为改了其实没改）；
//  2) 致命项和调参项必须分开（fps=60 该拒绝，max_corr_m 偏大只该提示）；
//  3) 安全相关的默认不能被配置悄悄放宽（eye-in-hand、allow_unaligned、发臂与 dry_run 同时开）。
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include "follow/config_loader.hpp"
#include "follow/health_server.hpp"

namespace follow {
namespace {

const std::string kRoot = "/tmp/follow_cfg_test";

std::string tmp(const std::string& name) { return kRoot + "/" + name; }

void write_yaml(const std::string& name, const std::string& body) {
  const std::string path = tmp(name);
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
  std::ofstream f(path);
  f << body;
  ASSERT_TRUE(f.good()) << path;
}

bool mentions(const ConfigProblems& p, const std::string& needle, bool fatal) {
  for (const auto& i : p.items) {
    if (i.fatal == fatal && i.text.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

bool has_fatal(const ConfigProblems& p, const std::string& needle) {
  return mentions(p, needle, true);
}

bool has_warning(const ConfigProblems& p, const std::string& needle) {
  return mentions(p, needle, false);
}

// 只含一个空 follow: 块的配置 —— 用来把"内置默认值"和仓库里那份 yaml 解耦，
// 否则改一次生产配置就会让单测跟着变。
FollowConfig with_defaults() {
  write_yaml("defaults.yaml", "follow: {}\n");
  FollowConfig c;
  std::string err;
  EXPECT_TRUE(load_config(tmp("defaults.yaml"), &c, &err)) << err;
  EXPECT_EQ(c.capture.width, 848);  // bench_frontend 在真实工位图上量出来的预算
  EXPECT_EQ(c.capture.fps, 15);
  EXPECT_EQ(c.frontend.max_features, 200);
  EXPECT_TRUE(c.dry_run);
  EXPECT_EQ(c.health_port, 18081);
  EXPECT_TRUE(c.notes.empty()) << c.notes.size();
  return c;
}

TEST(Config, DefaultsAreSelfConsistent) {
  const FollowConfig base = with_defaults();
  FollowConfig c = base;
  const ConfigProblems p = check_config(&c, "/some/root");
  EXPECT_TRUE(p.ok()) << p.joined();
  EXPECT_EQ(c.capture.lock_path, "/some/root/.orbbec.lock");
  EXPECT_EQ(c.map_path, "/some/root/follow/out/reference.frmap");
  // 传进来的 root 是空时，锁路径就成了相对路径 —— 那必须报致命，不能默默用一个跟 cwd 绑定的锁。
  FollowConfig d = base;
  const ConfigProblems q = check_config(&d, "");
  EXPECT_TRUE(has_fatal(q, "绝对路径")) << q.joined();
}

TEST(Config, TypeErrorsAreNamedNotSwallowed) {
  write_yaml("bad_types.yaml",
             "follow:\n"
             "  camera:\n"
             "    fps: thirty\n"
             "    width: 848\n"
             "  runtime:\n"
             "    dry_run: maybe\n");
  FollowConfig c;
  std::string err;
  EXPECT_FALSE(load_config(tmp("bad_types.yaml"), &c, &err));
  EXPECT_NE(err.find("follow.camera.fps"), std::string::npos) << err;
  EXPECT_NE(err.find("follow.runtime.dry_run"), std::string::npos) << err;
  // 两个都要报：只报第一个会让人改三轮。
  EXPECT_NE(err.find('\n'), std::string::npos) << err;
}

TEST(Config, ExplicitMissingFileIsFatal) {
  FollowConfig c;
  std::string err;
  EXPECT_FALSE(load_config(tmp("nope.yaml"), &c, &err));
  EXPECT_NE(err.find("读不到"), std::string::npos) << err;
}

TEST(Config, AbsentBlockFallsBackButSaysSo) {
  write_yaml("no_block.yaml", "hardware:\n  camera:\n    width: 1280\n");
  FollowConfig c;
  std::string err;
  ASSERT_TRUE(load_config(tmp("no_block.yaml"), &c, &err)) << err;
  EXPECT_EQ(c.capture.width, 848);  // 刻意不继承 hardware.camera
  EXPECT_EQ(c.capture.fps, 15);
  EXPECT_FALSE(c.notes.empty());
  const ConfigProblems p = check_config(&c, "/r");
  EXPECT_TRUE(p.ok()) << p.joined();
  EXPECT_TRUE(has_warning(p, "follow: 块"));
}

// yaml-cpp 对标量节点取 [key] 会抛异常。少缩进一层就会踩到，而"配置写错 → 进程启动即崩"
// 是现场最难归因的一种失败。
TEST(Config, MisindentedBlockIsReportedNotThrown) {
  write_yaml("scalar_cam.yaml", "follow:\n  camera: 848\n");
  FollowConfig c;
  std::string err;
  ASSERT_FALSE(load_config(tmp("scalar_cam.yaml"), &c, &err));
  EXPECT_NE(err.find("follow.camera"), std::string::npos) << err;

  write_yaml("scalar_follow.yaml", "follow: 848\n");
  FollowConfig d;
  std::string derr;
  ASSERT_FALSE(load_config(tmp("scalar_follow.yaml"), &d, &derr));
  EXPECT_NE(derr.find("follow:"), std::string::npos) << derr;

  write_yaml("scalar_track.yaml", "follow:\n  track: [1, 2]\n  camera:\n    fps: 15\n");
  FollowConfig e;
  std::string eerr;
  ASSERT_FALSE(load_config(tmp("scalar_track.yaml"), &e, &eerr));
  EXPECT_NE(eerr.find("follow.track"), std::string::npos) << eerr;
}

TEST(Config, ValuesFromYamlWin) {
  write_yaml("ok.yaml",
             "follow:\n"
             "  camera:\n"
             "    width: 640\n"
             "    height: 480\n"
             "    fps: 30\n"
             "  track:\n"
             "    voxel_m: 0.02\n"
             "    threads: 2\n");
  FollowConfig c;
  std::string err;
  ASSERT_TRUE(load_config(tmp("ok.yaml"), &c, &err)) << err;
  EXPECT_EQ(c.capture.width, 640);
  EXPECT_EQ(c.track.voxel_m, 0.02);
  EXPECT_EQ(c.track.threads, 2);
  const ConfigProblems p = check_config(&c, "/r");
  EXPECT_TRUE(p.ok()) << p.joined();           // 640x480@30 合法
  EXPECT_TRUE(has_warning(p, "camera.fps"));   // 但超预算，必须提示
}

TEST(Config, FatalRules) {
  struct Case {
    const char* name;
    void (*mutate)(FollowConfig*);
    const char* needle;
  };
  const Case cases[] = {
      {"fps", [](FollowConfig* c) { c->capture.fps = 60; }, "camera.fps"},
      {"size", [](FollowConfig* c) { c->capture.width = 64; }, "width/height"},
      {"warmup", [](FollowConfig* c) { c->capture.first_pair_timeout_ms = 100; },
       "first_pair_timeout_ms"},
      {"frame_timeout", [](FollowConfig* c) { c->capture.frame_timeout_ms = 0; },
       "frame_timeout_ms"},
      {"unaligned", [](FollowConfig* c) { c->capture.allow_unaligned = true; }, "allow_unaligned"},
      {"kind", [](FollowConfig* c) { c->frontend_kind = "gpu"; }, "frontend.kind"},
      {"features", [](FollowConfig* c) { c->frontend.max_features = 8; }, "max_features"},
      {"quality", [](FollowConfig* c) { c->frontend.quality_level = 0.0; }, "quality_level"},
      {"zrange", [](FollowConfig* c) { c->track.zmax_m = 0.1; }, "zmin_m/zmax_m"},
      {"stride", [](FollowConfig* c) { c->track.depth_stride = 0; }, "depth_stride"},
      {"voxel", [](FollowConfig* c) { c->track.voxel_m = 0.0; }, "voxel_m"},
      {"corr", [](FollowConfig* c) { c->track.max_corr_m = -1.0; }, "max_corr_m"},
      {"threads", [](FollowConfig* c) { c->track.threads = 0; }, "threads"},
      {"ratio", [](FollowConfig* c) { c->track.min_inlier_ratio = 1.5; }, "min_inlier_ratio"},
      {"sigma", [](FollowConfig* c) { c->track.max_rot_sigma_deg = 0.0; }, "trans_sigma_mm"},
      {"aniso", [](FollowConfig* c) { c->track.max_group_anisotropy = 0.5; }, "group_anisotropy"},
      {"varscale", [](FollowConfig* c) { c->track.min_residual_var_scale = 0.0; },
       "min_residual_var_scale"},
      {"streak", [](FollowConfig* c) { c->track.max_sparse_streak = 5000; }, "max_sparse_streak"},
      {"teachframes", [](FollowConfig* c) { c->teach_frames = 0; }, "teach.frames"},
      {"mount", [](FollowConfig* c) { c->mount = "eye-in-hand"; }, "mount"},
      {"port", [](FollowConfig* c) { c->health_port = 18080; }, "health_port"},
      {"cycles", [](FollowConfig* c) { c->max_cycles = -1; }, "max_cycles"},
      {"servo_vs_dry",
       [](FollowConfig* c) {
         c->dry_run = true;
         c->enable_servo_p = true;
       },
       "矛盾"},
  };
  for (const auto& tc : cases) {
    FollowConfig c = with_defaults();
    tc.mutate(&c);
    const ConfigProblems p = check_config(&c, "/r");
    EXPECT_FALSE(p.ok()) << tc.name << " 应当被拒: " << p.joined();
    EXPECT_TRUE(has_fatal(p, tc.needle))
        << tc.name << " 没报到 \"" << tc.needle << "\":\n" << p.joined();
  }
}

TEST(Config, TunablesWarnInsteadOfFailing) {
  FollowConfig c = with_defaults();
  c.track.max_corr_m = 0.5;       // 远超 2.6·voxel，会被体素邻域截断
  c.frontend.max_features = 400;  // CPU 前端超预算
  c.dry_run = false;              // 发臂警告
  const ConfigProblems p = check_config(&c, "/r");
  EXPECT_TRUE(p.ok()) << p.joined();  // 三个都只是提示，不该拦启动
  EXPECT_GE(p.warnings(), 3u) << p.joined();
  EXPECT_TRUE(has_warning(p, "2.6·voxel_m"));
  EXPECT_TRUE(has_warning(p, "ServoP"));
  EXPECT_TRUE(has_warning(p, "max_features"));
}

TEST(Config, DescribeShowsEveryGate) {
  const FollowConfig c = with_defaults();
  const std::string d = describe(c);
  for (const char* needle : {"848x480", "18081", "dry_run", "sigma_t", "inlier_ratio", "voxel"}) {
    EXPECT_NE(d.find(needle), std::string::npos) << needle;
  }
}

// 健康快照是 Python 门面唯一的观测口：NaN 不能变成非法 JSON 字面量，控制字符不能提前
// 终结一个字符串 —— 那两种情况的故障现象都是"服务没响应"，而真因写在上一行的错误文本里。
TEST(Health, JsonKeepsNonFiniteAndControlCharsOut) {
  HealthSnapshot s;
  s.state = "tracking";
  s.status = "out_of_envelope";
  s.sigma_t_mm = std::nan("");
  s.sigma_r_deg = 1.5;
  s.inlier_ratio = 0.2;
  s.last_error = std::string("带\"引号\" 反斜杠\\ 换行\n制表\t控制字符\x01") + "结束";
  const std::string j = to_json(s);
  EXPECT_NE(j.find("\"sigma_t_mm\":null"), std::string::npos) << j;
  EXPECT_NE(j.find("\"sigma_r_deg\":1.5"), std::string::npos) << j;
  EXPECT_NE(j.find("\\\"引号\\\""), std::string::npos) << j;
  EXPECT_NE(j.find("\\\\"), std::string::npos) << j;
  EXPECT_NE(j.find("\\n"), std::string::npos) << j;
  EXPECT_NE(j.find("\\u0001"), std::string::npos) << j;
  // 字面换行/制表/control 一旦原样出现在字符串值里，JSON 就废了。
  const std::string body = j.substr(j.find("\"last_error\""));
  EXPECT_EQ(body.find('\n'), std::string::npos);
  EXPECT_EQ(body.find('\t'), std::string::npos);
}

TEST(Health, CorrectionIsThreeElementArrays) {
  HealthSnapshot s;
  s.correction = {{1.5, -2.5, 3.0, 0.1, 0.2, 0.3}};
  const std::string j = to_json(s);
  EXPECT_NE(j.find("\"correction_mm\":[1.5,-2.5,3]"), std::string::npos) << j;
  EXPECT_NE(j.find("\"correction_deg\":[0.1,0.2,0.3]"), std::string::npos) << j;
}

}  // namespace
}  // namespace follow
