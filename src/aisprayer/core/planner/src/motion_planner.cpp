#include "motion_planner.hpp"
#include "planner_utils.hpp"
#include "thread_pool.hpp"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/command_language/cartesian_waypoint.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract/command_language/poly/instruction_poly.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/command_language/poly/state_waypoint_poly.h>
#include <tesseract/command_language/state_waypoint.h>
#include <tesseract/environment/commands/add_contact_managers_plugin_info_command.h>
#include <tesseract/environment/commands/add_kinematics_information_command.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/motion_planners/planner.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract/motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract/motion_planners/types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace aisprayer::planner {

namespace {

using json = nlohmann::json;
using tesseract::command_language::CartesianWaypoint;
using tesseract::command_language::CartesianWaypointPoly;
using tesseract::command_language::CompositeInstruction;
using tesseract::command_language::CompositeInstructionOrder;
using tesseract::command_language::MoveInstruction;
using tesseract::command_language::MoveInstructionType;
using tesseract::command_language::StateWaypoint;
using tesseract::command_language::StateWaypointPoly;
using tesseract::common::JointState;
using tesseract::common::ManipulatorInfo;
using tesseract::common::ProfileDictionary;
using tesseract::environment::Environment;
using tesseract::motion_planners::PlannerRequest;
using tesseract::motion_planners::TrajOptMotionPlanner;

constexpr int kTcpTargetsSchemaVersion = 2;
constexpr double kDuplicateJointTolerance = 1e-8;
constexpr const char* kKdlPluginName = "KDLInvKinChainNR_JL";
constexpr const char* kKdlFactoryName = "KDLInvKinChainNR_JLFactory";
constexpr const char* kSprayPlannerName = "kdl_spray_planner";
constexpr const char* kTransitionPlannerName = "kdl_transition_planner";
constexpr const char* kSprayProfileName = "spray_profile";
constexpr std::size_t kMinTrajOptLinearPoints = 4;

struct TargetPose {
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  std::size_t original_point_index{ 0 };
};

struct InputStroke {
  std::size_t mesh_index{ 0 };
  std::size_t source_stroke_index{ 0 };
  std::vector<TargetPose> points;
};

struct ReachableStroke {
  std::size_t sequence_index{ 0 };
  std::size_t mesh_index{ 0 };
  std::size_t source_stroke_index{ 0 };
  std::vector<TargetPose> targets;
  std::vector<Eigen::VectorXd> seeds;
};

struct InternalTrajectoryPoint {
  Eigen::VectorXd joints;
  std::string motion_type;
  bool segment_start{ false };
};

struct StrokePlan {
  std::size_t sequence_index{ 0 };
  std::vector<InternalTrajectoryPoint> points;
  Eigen::VectorXd start_joints;
  Eigen::VectorXd end_joints;
};

/**
 * @brief 解析 TCP JSON 文件以生成针对 MotionPlanner 的输入格式 (InputStroke)。
 * @note 内部处理版本兼容以及坐标转换。如果提取发现严重格式错误则抛出异常。
 */
std::vector<InputStroke> loadTcpTargetsFromJson(const json& document)
{
  int version = requiredNumber(document, "version");
  if (version != 1 && version != 2)
    throw std::runtime_error("Unsupported target json schema version");
    
  if (!document.contains("strokes") || !document.at("strokes").is_array())
    throw std::runtime_error("JSON missing 'strokes' array");

  std::vector<InputStroke> strokes;
  const json& strokes_json = document.at("strokes");
  strokes.reserve(strokes_json.size());

  for (std::size_t stroke_index = 0; stroke_index < strokes_json.size(); ++stroke_index) {
    const json& source = strokes_json.at(stroke_index);
    if (!source.is_object() || !source.contains("points") || !source.at("points").is_array())
      throw std::runtime_error("stroke " + std::to_string(stroke_index) + " is malformed");

    InputStroke stroke;
    stroke.mesh_index = requiredIndex(source, "mesh_index");
    stroke.source_stroke_index = requiredIndex(source, "stroke_index");
    stroke.points.reserve(source.at("points").size());
    for (std::size_t point_index = 0; point_index < source.at("points").size(); ++point_index) {
      TargetPose p;
      p.pose = parseTargetPose(source.at("points").at(point_index), stroke_index, point_index);
      p.original_point_index = point_index;
      stroke.points.push_back(p);
    }
    strokes.push_back(std::move(stroke));
  }
  if (strokes.empty())
    throw std::runtime_error("JSON contains no strokes");
  return strokes;
}

std::vector<InputStroke> loadTcpTargets(const std::string& path)
{
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Unable to open TCP target file: " + path);
  json document = json::parse(file);
  return loadTcpTargetsFromJson(document);
}

/**
 * @brief 从指定文件加载文本内容并返回整个字符串。
 * @note 用于快捷读取 URDF 或 SRDF 机器人模型文件内容，如果文件不存在则抛出异常。
 */
std::string loadTextFile(const std::string& path)
{
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Unable to open file: " + path);
  return { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
}

bool isFiniteVector(const Eigen::VectorXd& value) { return value.size() > 0 && value.allFinite(); }

void appendDistinct(std::vector<InternalTrajectoryPoint>& output, const std::vector<InternalTrajectoryPoint>& incoming)
{
  for (const InternalTrajectoryPoint& point : incoming) {
    if (!output.empty() && point.joints.size() == output.back().joints.size() &&
        (point.joints - output.back().joints).norm() <= kDuplicateJointTolerance)
      continue;
    output.push_back(point);
  }
}

/**
 * @brief 解析并提取 TrajOpt 规划后的轨迹数据到内部平坦的轨迹点列表中。
 * @note 只提取 FREESPACE 或 LINEAR 状态点。first_is_segment_start 用于标识新连续线段的起点。
 */
std::vector<InternalTrajectoryPoint> flattenResponse(const tesseract::motion_planners::PlannerResponse& response,
                                                     const std::string& motion_type,
                                                     bool first_is_segment_start)
{
  std::vector<InternalTrajectoryPoint> points;
  const auto flattened = response.results.flatten(&tesseract::command_language::moveFilter);
  for (const auto& instruction : flattened) {
    const auto move = instruction.get().template as<tesseract::command_language::MoveInstructionPoly>();
    if (!move.getWaypoint().isStateWaypoint()) continue;
    const auto state = move.getWaypoint().template as<StateWaypointPoly>();
    const Eigen::VectorXd position = state.getPosition();
    if (!isFiniteVector(position)) throw std::runtime_error("TrajOpt returned a non-finite joint position");
    points.push_back({ position, motion_type, first_is_segment_start && points.empty() });
  }
  return points;
}

struct PlanningContext {
  std::shared_ptr<Environment> environment;
  std::shared_ptr<ProfileDictionary> profiles;
  std::shared_ptr<const tesseract::kinematics::KinematicGroup> kinematics;
  ManipulatorInfo manipulator;
  std::vector<std::string> joint_names;
};

/**
 * @brief 根据配置 (URDF, SRDF) 创建并初始化 Tesseract 运动学与碰撞检查环境上下文。
 * @note KDL 和 TrajOpt 初始化都在这里完成。抛出 runtime_error 当模型加载或者插件调用失败。
 */
PlanningContext createPlanningContext(const MotionConfig& config, const std::string& urdf, const std::string& srdf)
{
  PlanningContext context;
  context.environment = std::make_shared<Environment>();
  const auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
  if (!context.environment->init(urdf, srdf, locator))
    throw std::runtime_error("Failed to initialize the Tesseract environment");

  auto kinematics_info = context.environment->getKinematicsInformation();
  YAML::Node kdl_config;
  kdl_config["base_link"] = config.base_link;
  kdl_config["tip_link"] = config.tcp_frame;
  kdl_config["position_eps"] = 1e-5;
  kdl_config["position_iterations"] = 200;

  auto& inverse_plugins = kinematics_info.kinematics_plugin_info.inv_plugin_infos[config.manipulator_group];
  inverse_plugins.default_plugin = kKdlPluginName;
  inverse_plugins.plugins.clear();
  inverse_plugins.plugins[kKdlPluginName] = { kKdlFactoryName, kdl_config };
  context.environment->applyCommand(
      std::make_shared<tesseract::environment::AddKinematicsInformationCommand>(kinematics_info));

  tesseract::common::ContactManagersPluginInfo contact_plugins;
  contact_plugins.discrete_plugin_infos.default_plugin = "BulletDiscreteBVHManager";
  contact_plugins.discrete_plugin_infos.plugins["BulletDiscreteBVHManager"] = { "BulletDiscreteBVHManagerFactory", YAML::Node() };
  contact_plugins.continuous_plugin_infos.default_plugin = "BulletCastBVHManager";
  contact_plugins.continuous_plugin_infos.plugins["BulletCastBVHManager"] = { "BulletCastBVHManagerFactory", YAML::Node() };
  context.environment->applyCommand(
      std::make_shared<tesseract::environment::AddContactManagersPluginInfoCommand>(contact_plugins));

  const auto active_info = context.environment->getKinematicsInformation();
  const auto group_it = active_info.kinematics_plugin_info.inv_plugin_infos.find(config.manipulator_group);
  if (group_it == active_info.kinematics_plugin_info.inv_plugin_infos.end() ||
      group_it->second.default_plugin != kKdlPluginName)
    throw std::runtime_error("Tesseract did not retain the requested KDL plugin configuration");

  context.kinematics = context.environment->getKinematicGroup(config.manipulator_group, kKdlPluginName);
  if (!context.kinematics)
    throw std::runtime_error("Unable to load KDL plugin");
  
  context.joint_names = context.kinematics->getJointNames();
  if (context.joint_names.empty()) throw std::runtime_error("KDL group exposes no joints");

  context.manipulator.manipulator = config.manipulator_group;
  context.manipulator.tcp_frame = config.tcp_frame;
  context.manipulator.working_frame = config.base_link;
  context.profiles = std::make_shared<ProfileDictionary>();

  auto move_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();
  move_profile->cartesian_constraint_config.enabled = false;
  move_profile->cartesian_cost_config.enabled = true;
  move_profile->cartesian_cost_config.use_tolerance_override = true;
  move_profile->cartesian_cost_config.lower_tolerance = Eigen::VectorXd::Zero(6);
  move_profile->cartesian_cost_config.lower_tolerance << -config.position_tolerance, -config.position_tolerance, -config.position_tolerance, -config.orientation_tolerance * M_PI / 180.0, -config.orientation_tolerance * M_PI / 180.0, -M_PI;
  move_profile->cartesian_cost_config.upper_tolerance = Eigen::VectorXd::Zero(6);
  move_profile->cartesian_cost_config.upper_tolerance << config.position_tolerance, config.position_tolerance, config.position_tolerance, config.orientation_tolerance * M_PI / 180.0, config.orientation_tolerance * M_PI / 180.0, M_PI;
  move_profile->cartesian_cost_config.coeff = (Eigen::VectorXd(6) << 5.0, 5.0, 5.0, 5.0, 5.0, 0.0).finished();

  auto composite_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
  composite_profile->collision_cost_config.enabled = true;
  composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.03, 20.0);
  composite_profile->collision_constraint_config.enabled = false;

  auto transition_composite_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
  transition_composite_profile->collision_cost_config = composite_profile->collision_cost_config;
  transition_composite_profile->collision_constraint_config.enabled = false;
  transition_composite_profile->smooth_accelerations = false;
  transition_composite_profile->smooth_jerks = false;

  context.profiles->addProfile(kSprayPlannerName, kSprayProfileName, move_profile);
  context.profiles->addProfile(kSprayPlannerName, kSprayProfileName, composite_profile);
  context.profiles->addProfile(kTransitionPlannerName, kSprayProfileName, move_profile);
  context.profiles->addProfile(kTransitionPlannerName, kSprayProfileName, transition_composite_profile);
  
  return context;
}

/**
 * @brief 获取一个全零的关节初始种子，用于作为逆解计算的首个猜测。
 */
Eigen::VectorXd homeSeed(const PlanningContext& context) {
  return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(context.joint_names.size()));
}

/**
 * @brief 尝试以指定的 seed 开始解析一条连续的目标轨迹，以确认逆解的完全可行性。
 * @note 如果序列中任意一点的 IK 找不到解或者成本太高，该函数将直接返回 std::nullopt，代表整条轨迹在这个 seed 下无法走通。
 */
std::optional<std::vector<Eigen::VectorXd>> resolveSeedChain(
    const std::vector<TargetPose>& targets, Eigen::VectorXd seed, const PlanningContext& context)
{
  std::vector<Eigen::VectorXd> seeds;
  seeds.reserve(targets.size());
  
  // 遍历轨迹的所有目标姿态，检查该初始种子是否能够求解完整的 IK 序列
  for (const TargetPose& target : targets) {
    const tesseract::kinematics::KinGroupIKInput ik_input(
        target.pose, context.manipulator.working_frame, context.manipulator.tcp_frame);
    
    // 使用当前 seed 寻找所有可能的逆解
    const auto solutions = context.kinematics->calcInvKin(ik_input, seed);
      
    Eigen::VectorXd best_solution;
    double best_cost = std::numeric_limits<double>::infinity();
    // 选择与当前 seed (或者上一个点的解) 在关节空间欧氏距离最近的最优解
    for (const Eigen::VectorXd& solution : solutions) {
      if (solution.size() != seed.size() || !isFiniteVector(solution)) continue;
      const double cost = (solution - seed).squaredNorm();
      if (cost < best_cost) {
        best_cost = cost;
        best_solution = solution;
      }
    }
    
    // 如果对任意点无解，说明整条路径无法完成，直接放弃
    if (!isFiniteVector(best_solution)) return std::nullopt;
    
    // 将该点的解作为下一个点逆解的 seed
    seed = best_solution;
    seeds.push_back(best_solution);
  }
  return seeds;
}

std::vector<ReachableStroke> filterReachableStrokes(
    const std::vector<InputStroke>& input_strokes, const MotionConfig& config, const std::string& urdf, const std::string& srdf,
    std::size_t& out_exact, std::size_t& out_retry, std::size_t& out_failed)
{
  PlanningContext context = createPlanningContext(config, urdf, srdf);
  std::vector<ReachableStroke> reachable;

  out_exact = 0;
  out_retry = 0;
  out_failed = 0;

  for (const InputStroke& input_stroke : input_strokes) {
    ReachableStroke fragment;
    Eigen::VectorXd previous_seed = homeSeed(context);
    fragment.mesh_index = input_stroke.mesh_index;
    fragment.source_stroke_index = input_stroke.source_stroke_index;

    // 内部帮助 Lambda 函数：当一段成功追踪的轨迹断裂时，如果其长度达标，则保存为一个合法的 ReachableStroke
    const auto flush_fragment = [&reachable, &fragment]() {
      if (fragment.targets.size() >= kMinTrajOptLinearPoints) {
        fragment.sequence_index = reachable.size();
        reachable.push_back(std::move(fragment));
      }
    };

    // 逐个遍历原始输入点，验证 KDL 逆解是否可达
    for (const TargetPose& original_target : input_stroke.points) {
      TargetPose target = original_target;

      auto tryIK = [&](const Eigen::Isometry3d& pose) -> Eigen::VectorXd {
        const tesseract::kinematics::KinGroupIKInput ik_input(
            pose, context.manipulator.working_frame, context.manipulator.tcp_frame);
        const auto solutions = context.kinematics->calcInvKin(ik_input, previous_seed);
        
        Eigen::VectorXd best_solution;
        double best_cost = std::numeric_limits<double>::infinity();
        for (const Eigen::VectorXd& solution : solutions) {
          if (solution.size() != previous_seed.size() || !isFiniteVector(solution)) continue;
          const double cost = (solution - previous_seed).squaredNorm();
          if (cost < best_cost) { best_cost = cost; best_solution = solution; }
        }
        return best_solution;
      };

      Eigen::VectorXd best_solution = tryIK(target.pose);

      if (isFiniteVector(best_solution)) {
        out_exact++;
      } else {
        // 如果初始姿态无解，则尝试绕 TCP 的 Z 轴 (喷嘴轴线) 进行旋转采样
        constexpr int num_samples = 24; // 360度 / 24 = 15度步长
        for (int i = 1; i < num_samples; ++i) {
          // 交替在正负方向上扩大搜索范围
          double angle = (i % 2 == 1 ? 1 : -1) * ((i + 1) / 2) * (2.0 * M_PI / num_samples);
          Eigen::Isometry3d sampled_pose = target.pose * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
          
          best_solution = tryIK(sampled_pose);
          if (isFiniteVector(best_solution)) {
            out_retry++;
            target.pose = sampled_pose; // 更新为采样成功的姿态，使得后续优化及写入使用该姿态
            break;
          }
        }
      }

      // 如果某一个点无解，则之前的连续线段将被切断（flush_fragment）
      if (!isFiniteVector(best_solution)) {
        out_failed++;
        flush_fragment();
        fragment = ReachableStroke{}; // 重置并开启下一段追踪
        fragment.mesh_index = input_stroke.mesh_index;
        fragment.source_stroke_index = input_stroke.source_stroke_index;
        continue;
      }
      
      // 更新 seed 并保存到当前累积的 fragment 中
      previous_seed = best_solution;
      fragment.targets.push_back(target);
      fragment.seeds.push_back(best_solution);
    }
    // 处理末尾段
    flush_fragment();
  }

  if (reachable.empty())
    throw std::runtime_error("KDL could not produce any reachable stroke with at least " + std::to_string(kMinTrajOptLinearPoints) + " points");
  return reachable;
}

/**
 * @brief 构建并运行 TrajOpt 线性笛卡尔规划 (LINEAR) 问题。
 * @note 这是底层的 TrajOpt 问题求解入口，包含 1 个自由段过渡(用于对齐起点)和 N 个直线段，并返回响应对象。
 */
tesseract::motion_planners::PlannerResponse solveLinearProgram(
    const ReachableStroke& stroke, const std::vector<Eigen::VectorXd>& seeds, const PlanningContext& context)
{
  CompositeInstruction program(kSprayProfileName, context.manipulator, CompositeInstructionOrder::ORDERED);
  program.push_back(MoveInstruction(StateWaypointPoly{ StateWaypoint(context.joint_names, seeds.front()) }, MoveInstructionType::FREESPACE, kSprayProfileName, context.manipulator));
  for (std::size_t point_index = 0; point_index < stroke.targets.size(); ++point_index) {
    CartesianWaypoint waypoint(stroke.targets.at(point_index).pose);
    JointState seed;
    seed.joint_names = context.joint_names;
    seed.position = seeds.at(point_index);
    waypoint.setSeed(seed);
    program.push_back(MoveInstruction(CartesianWaypointPoly{ waypoint }, MoveInstructionType::LINEAR, kSprayProfileName, context.manipulator));
  }
  TrajOptMotionPlanner planner(kSprayPlannerName);
  PlannerRequest request;
  request.env = context.environment;
  request.instructions = program;
  request.profiles = context.profiles;
  return planner.solve(request);
}

/**
 * @brief 为单个 ReachableStroke 执行高级的运动规划包装。
 * @note 在其内部，如果当前 seed 规划失败，它会尝试使用机器人回原点姿态作为 seed 进行重试求解，提高规划鲁棒性。
 */
StrokePlan planStroke(const ReachableStroke& stroke, const MotionConfig& config, const std::string& urdf, const std::string& srdf)
{
  PlanningContext context = createPlanningContext(config, urdf, srdf);
  if (stroke.targets.size() < 2 || stroke.targets.size() != stroke.seeds.size())
    throw std::runtime_error("Internal error: invalid reachable stroke");

  auto response = solveLinearProgram(stroke, stroke.seeds, context);
  if (!response.successful) {
    PlanningContext retry_context = createPlanningContext(config, urdf, srdf);
    // 重新通过 Chain 逻辑获取 seed
    const auto retry_seeds = resolveSeedChain(stroke.targets, homeSeed(retry_context), retry_context);
    if (retry_seeds.has_value())
      response = solveLinearProgram(stroke, *retry_seeds, retry_context);
    if (!response.successful)
      throw std::runtime_error("TrajOpt LINEAR planning failed for stroke " + std::to_string(stroke.sequence_index));
  }

  StrokePlan result;
  result.sequence_index = stroke.sequence_index;
  result.points = flattenResponse(response, "LINEAR", true);
  if (result.points.size() < 2)
    throw std::runtime_error("TrajOpt LINEAR planning returned fewer than two states");
  result.start_joints = result.points.front().joints;
  result.end_joints = result.points.back().joints;
  return result;
}

/**
 * @brief 规划从前一个线段末端到下一个线段起点的自由空间过渡路径 (FREESPACE)。
 * @note 在两个无关的喷涂线段之间生成纯关节空间的平滑运动轨迹。
 */
std::vector<InternalTrajectoryPoint> planTransition(const Eigen::VectorXd& from, const Eigen::VectorXd& to, const MotionConfig& config, const std::string& urdf, const std::string& srdf)
{
  PlanningContext context = createPlanningContext(config, urdf, srdf);
  CompositeInstruction program(kSprayProfileName, context.manipulator, CompositeInstructionOrder::ORDERED);
  program.push_back(MoveInstruction(StateWaypointPoly{ StateWaypoint(context.joint_names, from) }, MoveInstructionType::FREESPACE, kSprayProfileName, context.manipulator));
  program.push_back(MoveInstruction(StateWaypointPoly{ StateWaypoint(context.joint_names, to) }, MoveInstructionType::FREESPACE, kSprayProfileName, context.manipulator));
  TrajOptMotionPlanner planner(kTransitionPlannerName);
  PlannerRequest request;
  request.env = context.environment;
  request.instructions = program;
  request.profiles = context.profiles;
  const auto response = planner.solve(request);
  if (!response.successful) throw std::runtime_error("TrajOpt FREESPACE transition failed");
  return flattenResponse(response, "FREESPACE", false);
}

} // namespace

/**
 * @brief MotionPlanner 构造函数。
 * @note 内部不会在此时直接实例化 Tesseract 环境，因为多线程并发下的 TrajOpt 执行需要各自创建 Local Environment，我们仅保存相关的机器人配置及模型文本字符串。
 */
MotionPlanner::MotionPlanner(const MotionConfig& config) : config_(config) {
}

MotionPlanner::~MotionPlanner() = default;

/**
 * @brief 通过目标位姿 JSON 文件路径触发运动规划。
 */
std::optional<nlohmann::json> MotionPlanner::plan(const std::string& tcp_targets_file)
{
  std::ifstream file(tcp_targets_file);
  if (!file) {
    std::cerr << "Unable to open file: " << tcp_targets_file << '\n';
    return std::nullopt;
  }
  json document = json::parse(file);
  return plan(document);
}

/**
 * @brief 通过内存中的 JSON 对象触发运动规划。
 */
std::optional<nlohmann::json> MotionPlanner::plan(const nlohmann::json& original_json)
{
  std::vector<InputStroke> input_strokes;
  try {
    input_strokes = loadTcpTargetsFromJson(original_json);
  } catch (const std::exception& e) {
    std::cerr << "Motion planning failed to load targets from JSON: " << e.what() << '\n';
    return std::nullopt;
  }
  std::string urdf;
  std::string srdf;
  try {
    urdf = loadTextFile(config_.urdf_path);
    srdf = loadTextFile(config_.srdf_path);
  } catch (const std::exception& e) {
    std::cerr << "Motion planning failed to load robot URDF/SRDF: " << e.what() << '\n';
    return std::nullopt;
  }

  try {
    // 预求解 IK 以保证规划结果在给定的关节限位和无干涉范围内
    std::size_t exact_match = 0, rescued = 0, failed = 0;
    const std::vector<ReachableStroke> reachable_strokes = filterReachableStrokes(input_strokes, config_, urdf, srdf, exact_match, rescued, failed);
    
    // 2. 利用线程池并行地对各个独立的子线段 (Stroke) 发起 TrajOpt LINEAR 笛卡尔规划
    ThreadPool pool(config_.thread_count);
    std::vector<std::future<StrokePlan>> futures;
    futures.reserve(reachable_strokes.size());
    for (const ReachableStroke& stroke : reachable_strokes) {
      futures.push_back(pool.submit([this, &urdf, &srdf, stroke]() {
        return planStroke(stroke, config_, urdf, srdf);
      }));
    }

    // 收集并行计算的结果
    std::vector<StrokePlan> stroke_plans;
    stroke_plans.reserve(futures.size());
    for (auto& future : futures) stroke_plans.push_back(future.get());

    // 对所有得到的轨迹按照 sequence_index 严格排序以保证时序正确
    std::sort(stroke_plans.begin(), stroke_plans.end(), [](const StrokePlan& lhs, const StrokePlan& rhs) {
      return lhs.sequence_index < rhs.sequence_index;
    });

    // 3. 构建结果 JSON 对象 (深拷贝 process 的原始数据)
    json output_json = original_json;

    // 首先初始化所有输出点状态为 UNPLANNED_OR_IK_FAILED，默认假设所有点都被放弃了
    if (output_json.contains("strokes") && output_json["strokes"].is_array()) {
      for (auto& stroke : output_json["strokes"]) {
        if (stroke.contains("points") && stroke["points"].is_array()) {
          for (auto& point : stroke["points"]) {
            point["status"] = "UNPLANNED_OR_IK_FAILED";
          }
        }
      }
    }

    json transitions = json::array();
    json chronological_trajectory = json::array();

    // 4. 将每段成功规划出的 TrajOpt 轨迹注入回原始的 JSON 结构中
    for (std::size_t stroke_index = 0; stroke_index < stroke_plans.size(); ++stroke_index) {
      const StrokePlan& current = stroke_plans.at(stroke_index);
      const ReachableStroke& reachable = reachable_strokes.at(current.sequence_index);

      if (stroke_index > 0) {
        const StrokePlan& previous = stroke_plans.at(stroke_index - 1);
        std::vector<InternalTrajectoryPoint> freespace = planTransition(previous.end_joints, current.start_joints, config_, urdf, srdf);

        json transition_json = json::array();
        for (const auto& pt : freespace) {
          json pub_pt;
          std::vector<double> joints(pt.joints.size());
          for (std::size_t j = 0; j < joints.size(); ++j) {
            joints[j] = config_.angle_unit == "deg" ? pt.joints[j] * 180.0 / M_PI : pt.joints[j];
          }
          pub_pt["joint_positions"] = joints;
          pub_pt["motion_type"] = pt.motion_type;
          pub_pt["segment_start"] = pt.segment_start;
          pub_pt["angle_unit"] = config_.angle_unit;
          transition_json.push_back(pub_pt);
          chronological_trajectory.push_back(pub_pt);
        }
        transitions.push_back(transition_json);
      }

      // TrajOpt 返回的 current.points 包含 1 个用来抵达该段起点的 FREESPACE 点 + N 个 LINEAR 点
      if (current.points.size() != reachable.targets.size() + 1) {
        std::cerr << "Warning: Stroke plan point count mismatch!\n";
      } else {
        // 根据记录的原始索引，精确回填 joint_positions 和 SUCCESS 状态
        for (std::size_t i = 0; i < reachable.targets.size(); ++i) {
          const auto& target = reachable.targets[i];
          const auto& internal_pt = current.points[i + 1]; // 跳过首个自由点

          std::vector<double> joints(internal_pt.joints.size());
          for (std::size_t j = 0; j < joints.size(); ++j) {
            joints[j] = config_.angle_unit == "deg" ? internal_pt.joints[j] * 180.0 / M_PI : internal_pt.joints[j];
          }

          auto& out_pt = output_json["strokes"][reachable.source_stroke_index]["points"][target.original_point_index];
          out_pt["joint_positions"] = joints;
          out_pt["motion_type"] = internal_pt.motion_type;
          out_pt["status"] = "SUCCESS";
          out_pt["ik_solver"] = kKdlPluginName;
          out_pt["collision_checked"] = true;
          out_pt["angle_unit"] = config_.angle_unit;
          
          chronological_trajectory.push_back(out_pt);
        }
      }
    }

    output_json["transitions"] = transitions;
    output_json["trajectory"] = chronological_trajectory;
    
    // 注入统计信息
    output_json["ik_stats"] = {
      {"exact_match", exact_match},
      {"rescued_via_z_axis_sampling", rescued},
      {"failed", failed}
    };

    return output_json;
  } catch (const std::exception& e) {
    std::cerr << "Motion planning failed: " << e.what() << '\n';
    return std::nullopt;
  }
}

/**
 * @brief 将注入运动规划信息的 JSON 结果保存至磁盘。
 * @note 输出文件中不仅包含完整 3D 几何特征数据，也对所有的机器人关节角度轨迹做了一一对应输出，方便后期进行联合调试与渲染展示。
 */
bool MotionPlanner::save(const nlohmann::json& trajectory, const std::string& output_file) const
{
  std::filesystem::path path(output_file);
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream output(output_file);
  if (!output) {
    std::cerr << "Unable to open trajectory output: " << output_file << '\n';
    return false;
  }
  output << trajectory.dump(4) << '\n';
  return true;
}

} // namespace aisprayer::planner
