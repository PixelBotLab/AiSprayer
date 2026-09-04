#include "cli_report.hpp"
#include "motion/io.hpp"
#include "motion/kinematics.hpp"
#include "motion/optimizer.hpp"
#include "motion/robot_model.hpp"
#include "motion/verifier.hpp"

#include <CLI/CLI.hpp>

#include <chrono>
#include <iostream>
#include <optional>
#include <sstream>
#include <vector>

namespace {

std::vector<double> SplitCsv(const std::string& s) {
  std::vector<double> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (!tok.empty()) out.push_back(std::stod(tok));
  }
  return out;
}

motion::AxisGrid ParseGrid(const std::string& s, const motion::AxisGrid& def) {
  const auto v = SplitCsv(s);
  if (v.size() != 3) return def;
  return {v[0], v[1], v[2]};
}

int EmitError(int code, const std::string& action, const std::string& msg) {
  std::cout << "{\"success\":false,\"action\":\"" << action << "\",\"message\":\"" << msg
            << "\"}\n";
  return code;
}

// poi yaml 的 source_file 记的是同目录内的文件名，不是调用时传入的绝对路径。
std::string BaseName(const std::string& p) {
  const auto n = p.find_last_of("/\\");
  return n == std::string::npos ? p : p.substr(n + 1);
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"AiSprayer motion CLI (verify / optimize / fk / ik)"};
  app.require_subcommand(1);

  std::string input, output, urdf, tool = "gripper_tip_link";
  std::string config_path;
  std::string seed_deg = "0,0,-90,-90,-90,0";
  double speed = 120.0, step = 1.5;
  int path_id = -1;

  app.add_option("--config", config_path,
                 "aisprayer_config.yaml（读取 spraying.poi_* / grid_tol_* / robot_urdf）");

  auto* verify = app.add_subcommand("verify", "Dense MoveL kinematic verification (input: *.auto.path.yaml)");
  verify->add_option("--input", input, "待验证路径 yaml（应为 scan.auto.path.yaml，不是 poi）")->required();
  verify->add_option("--urdf", urdf, "robot URDF（可被 --config 填充）");
  verify->add_option("--tool-tcp", tool, "TCP link name");
  verify->add_option("--speed", speed, "Cartesian speed mm/s");
  verify->add_option("--step", step, "interpolation step mm");
  verify->add_option("--path-id", path_id, "optional path_id filter");
  verify->add_option("--seed", seed_deg, "init joints deg (default Home)");

  std::string anchor_source = "config";
  std::string ref_rpy = "90,0,90";
  std::string anchor_tol = "10,10,30";
  std::string grid_x = "-5,5,2", grid_y = "-5,5,2", grid_z = "-30,30,5";
  std::string home_joints = "0,0,-90,-90,-90,0";
  int beam = 32, max_per_branch = 16;
  bool no_dense = false;

  auto* optimize = app.add_subcommand("optimize", "Viterbi optimize scan.auto.path.yaml → poi");
  optimize->add_option("--input", input, "待优化路径 yaml（应为 scan.auto.path.yaml）")->required();
  optimize->add_option("--output", output, "output yaml");
  optimize->add_option("--urdf", urdf, "robot URDF（可被 --config 填充）");
  optimize->add_option("--tool-tcp", tool);
  optimize->add_option("--anchor-source", anchor_source, "config | home | raw");
  optimize->add_option("--ref-rpy", ref_rpy, "锚点中心 Rx,Ry,Rz deg（config/live）");
  optimize->add_option("--anchor-tol", anchor_tol, "锚点包络 ±Rx,±Ry,±Rz deg");
  optimize->add_option("--grid-x", grid_x, "工具系 X 搜索网格 min,max,step");
  optimize->add_option("--grid-y", grid_y, "工具系 Y 搜索网格 min,max,step");
  optimize->add_option("--grid-z", grid_z, "工具系 Z 搜索网格 min,max,step");
  optimize->add_option("--home-joints", home_joints);
  optimize->add_option("--beam-width", beam);
  optimize->add_option("--max-candidates-per-branch", max_per_branch);
  optimize->add_option("--speed", speed);
  optimize->add_option("--step", step);
  optimize->add_flag("--no-dense-verify", no_dense);

  std::string joints = "0,0,-90,-90,-90,0";
  std::string pose;
  auto* fk = app.add_subcommand("fk", "Forward kinematics (controller mm/deg)");
  fk->add_option("--joints", joints, "6 joints in degrees")->required();

  auto* ik = app.add_subcommand("ik", "Inverse kinematics (controller mm/deg)");
  ik->add_option("--pose", pose, "x,y,z,rx,ry,rz")->required();
  ik->add_option("--seed", joints, "seed joints deg");
  ik->add_option("--urdf", urdf);
  ik->add_option("--tool-tcp", tool);

  CLI11_PARSE(app, argc, argv);

  // 生效值：先取 CLI 默认 → 有 --config 时用配置覆盖未显式给出的项 → CLI 显式值最高优先。
  motion::SprayingConfig eff;
  eff.ref_rpy_deg = {90.0, 0.0, 90.0};
  eff.tol_deg = {10.0, 10.0, 30.0};

  auto resolve_config = [&]() -> int {
    CLI::App* sub = verify->parsed() ? verify : (optimize->parsed() ? optimize : nullptr);
    auto given = [&](const char* name) {
      return sub != nullptr && sub->get_option_no_throw(name) != nullptr &&
             sub->get_option(name)->count() > 0;
    };
    if (!config_path.empty()) {
      std::string err;
      if (!motion::LoadSprayingConfig(config_path, eff, &err)) return EmitError(3, "cli", err);
      if (urdf.empty()) urdf = eff.urdf_path;
      if (!given("--tool-tcp") && !eff.tool_name.empty()) tool = eff.tool_name;
      if (!given("--speed")) speed = eff.speed_mm_s;
      if (!given("--step")) step = eff.step_mm;
      if (!given("--anchor-source")) anchor_source = eff.anchor_source;
    }
    if (given("--ref-rpy")) {
      const auto v = SplitCsv(ref_rpy);
      if (v.size() != 3) return EmitError(2, "cli", "--ref-rpy needs 3 values");
      eff.ref_rpy_deg = {v[0], v[1], v[2]};
    }
    if (given("--anchor-tol")) {
      const auto v = SplitCsv(anchor_tol);
      if (v.size() != 3) return EmitError(2, "cli", "--anchor-tol needs 3 values");
      eff.tol_deg = {v[0], v[1], v[2]};
    }
    if (given("--grid-x")) eff.grid_x = ParseGrid(grid_x, eff.grid_x);
    if (given("--grid-y")) eff.grid_y = ParseGrid(grid_y, eff.grid_y);
    if (given("--grid-z")) eff.grid_z = ParseGrid(grid_z, eff.grid_z);
    if (optimize->parsed()) {
      std::cerr << "[motion_cli] anchor=" << anchor_source << " ref_rpy=["
                << eff.ref_rpy_deg.transpose() << "] tol=[" << eff.tol_deg.transpose()
                << "] grid_z=[" << eff.grid_z.min_deg << "," << eff.grid_z.max_deg << ","
                << eff.grid_z.step_deg << "] urdf=" << urdf << "\n";
    }
    return 0;
  };

  try {
    if (int rc = resolve_config()) return rc;
    if ((verify->parsed() || optimize->parsed()) && urdf.empty()) {
      return EmitError(2, "cli", "missing --urdf (or --config with hardware.robot.robot_urdf)");
    }
    if (verify->parsed()) {
      motion::RobotModel model;
      std::string err;
      if (!motion::LoadRobotModelFromUrdf(urdf, tool, model, &err)) {
        return EmitError(3, "verify", err);
      }
      motion::PathDocument doc;
      if (!motion::LoadPathYaml(input, doc, &err)) return EmitError(3, "verify", err);
      if (path_id >= 0) {
        std::vector<motion::PathItem> keep;
        for (auto& p : doc.paths)
          if (p.path_id == path_id) keep.push_back(std::move(p));
        doc.paths = std::move(keep);
      }
      motion::Cr5Kinematics kin(model.limits);
      motion::VerifyOptions opt;
      opt.step_mm = step;
      opt.speed_mm_s = speed;
      motion::ChainVerifier v(kin, model.tool, opt);
      std::optional<motion::JointVec> seed;
      const auto seed_vals = SplitCsv(seed_deg);
      if (seed_vals.size() == 6) {
        motion::JointVec q;
        for (int i = 0; i < 6; ++i) q[i] = motion::Rad(seed_vals[i]);
        seed = q;
      }
      const auto t0 = std::chrono::steady_clock::now();
      const motion::VerifyReport report = v.VerifyAll(doc.paths, seed);
      const auto t1 = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      motion::PrintVerifyReport(std::cerr, report, ms);
      std::cout << motion::JsonReportVerify(report, ms, true, "") << "\n";
      return 0;
    }

    if (optimize->parsed()) {
      motion::RobotModel model;
      std::string err;
      if (!motion::LoadRobotModelFromUrdf(urdf, tool, model, &err)) {
        return EmitError(3, "optimize", err);
      }
      motion::PathDocument doc;
      if (!motion::LoadPathYaml(input, doc, &err)) return EmitError(3, "optimize", err);
      motion::Cr5Kinematics kin(model.limits);
      motion::OptimizeOptions oopt;
      oopt.grid_x = eff.grid_x;
      oopt.grid_y = eff.grid_y;
      oopt.grid_z = eff.grid_z;
      oopt.beam_width = beam;
      oopt.max_candidates_per_branch = max_per_branch;
      oopt.dense_verify = !no_dense;
      if (auto e = oopt.Validate(); !e.empty()) return EmitError(2, "optimize", e);

      motion::VerifyOptions vopt;
      vopt.step_mm = step;
      vopt.speed_mm_s = speed;
      motion::ChainVerifier verifier(kin, model.tool, vopt);
      motion::ViterbiOptimizer optimizer(kin, model.tool, oopt, &verifier);

      motion::AnchorSpec spec;
      spec.source = anchor_source;
      spec.ref_rpy_deg = eff.ref_rpy_deg;
      spec.tol_deg = eff.tol_deg;
      const auto home = SplitCsv(home_joints);
      if (home.size() == 6) {
        for (int i = 0; i < 6; ++i) spec.home_joints_rad[i] = motion::Rad(home[i]);
      }

      motion::PathDocument out_doc = doc;
      std::optional<motion::JointVec> last_q;
      motion::OptimizeResult last;
      double total_ms = 0.0;
      bool any_modified = false;
      for (size_t i = 0; i < doc.paths.size(); ++i) {
        const auto anchor = motion::ResolveAnchor(spec, kin, doc.paths[i]);
        motion::PrintOptimizePreamble(std::cerr, input, doc.paths[i], kin, spec, anchor, oopt,
                                      speed, step);
        last = optimizer.Optimize(doc.paths[i], anchor, last_q);
        out_doc.paths[i] = last.path;
        total_ms += last.elapsed_ms;
        any_modified = any_modified || last.modified;
        if (!last.joints_rad.empty()) last_q = last.joints_rad.back();
      }
      last.elapsed_ms = total_ms;
      last.modified = any_modified;
      // 与 Python 版一致：由 auto 优化出来的状态叫 auto_poi，web 模板状态机认这个值。
      out_doc.type = "auto_poi";
      out_doc.state_type = "auto_poi";
      out_doc.source_file = BaseName(input);
      out_doc.execution_speed_mm_s = speed;
      auto all = verifier.VerifyAll(out_doc.paths);
      if (!output.empty()) {
        motion::PoiConfig poi;
        poi.anchor_source = anchor_source;
        poi.mode = (anchor_source == "raw") ? "per_waypoint_nominal_envelope"
                                            : "absolute_anchor_tolerance";
        poi.ref_rpy_deg = spec.ref_rpy_deg;
        poi.tolerance_rpy_deg = spec.tol_deg;
        poi.has_ref_rpy = (anchor_source != "raw");
        if (!motion::SavePathYaml(output, out_doc, &all, &poi, &err)) {
          return EmitError(3, "optimize", err);
        }
      }
      const motion::PathVerifyReport* first =
          all.path_reports.empty() ? nullptr : &all.path_reports[0];
      const motion::Anchor last_anchor = motion::ResolveAnchor(spec, kin, doc.paths.back());
      motion::PrintOptimizeReport(std::cerr, kin, doc.paths.back(), last, last_anchor, spec.tol_deg,
                                  spec.home_joints_rad, first, output);
      std::cout << motion::JsonReportOptimize(last, &all, true, "") << "\n";
      return 0;
    }

    if (fk->parsed()) {
      const auto qdeg = SplitCsv(joints);
      if (qdeg.size() != 6) return EmitError(2, "fk", "need 6 joints");
      motion::JointVec q;
      for (int i = 0; i < 6; ++i) q[i] = motion::Rad(qdeg[i]);
      motion::Cr5Kinematics kin;
      Eigen::Vector3d xyz, rpy;
      kin.FkController(q, xyz, rpy);
      std::cout << "{\"success\":true,\"action\":\"fk\",\"xyz_mm\":[" << xyz[0] << "," << xyz[1]
                << "," << xyz[2] << "],\"rpy_deg\":[" << rpy[0] << "," << rpy[1] << "," << rpy[2]
                << "]}\n";
      return 0;
    }

    if (ik->parsed()) {
      const auto p = SplitCsv(pose);
      if (p.size() != 6) return EmitError(2, "ik", "need x,y,z,rx,ry,rz");
      motion::RobotLimits limits;
      if (!urdf.empty()) {
        motion::RobotModel model;
        std::string err;
        if (!motion::LoadRobotModelFromUrdf(urdf, tool, model, &err)) return EmitError(3, "ik", err);
        limits = model.limits;
      }
      motion::Cr5Kinematics kin(limits);
      const auto seed_deg = SplitCsv(joints);
      motion::JointVec seed = motion::JointVec::Zero();
      if (seed_deg.size() == 6)
        for (int i = 0; i < 6; ++i) seed[i] = motion::Rad(seed_deg[i]);
      const motion::Transform T =
          motion::PoseFromCtrlMmDeg({p[0], p[1], p[2]}, {p[3], p[4], p[5]});
      auto best = kin.BestIk(motion::CtrlToUrdf(T), seed);
      if (!best) return EmitError(4, "ik", "no IK");
      std::cout << "{\"success\":true,\"action\":\"ik\",\"q_rad\":[";
      for (int i = 0; i < 6; ++i) {
        if (i) std::cout << ",";
        std::cout << (*best)[i];
      }
      std::cout << "]}\n";
      return 0;
    }
  } catch (const std::exception& e) {
    const std::string act = optimize->parsed() ? "optimize" : (verify->parsed() ? "verify" : "cli");
    return EmitError(4, act, e.what());
  }
  return 2;
}
