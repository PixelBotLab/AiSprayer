#include "planner/motion_planner.hpp"
#include "planner/thread_pool.hpp"

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
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace aisprayer::planner
{
namespace
{
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

constexpr int kTcpTargetsSchemaVersion = 1;
constexpr double kQuaternionNormTolerance = 1e-6;
constexpr double kDuplicateJointTolerance = 1e-8;
constexpr const char* kKdlPluginName = "KDLInvKinChainNR_JL";
constexpr const char* kKdlFactoryName = "KDLInvKinChainNR_JLFactory";
constexpr const char* kSprayPlannerName = "kdl_spray_planner";
constexpr const char* kTransitionPlannerName = "kdl_transition_planner";
constexpr const char* kSprayProfileName = "spray_profile";
// TrajOpt's default composite profile requires at least five states per planned segment
// (one free-space start state plus the linear-motion states), so any IK-reachable
// fragment with fewer linear points cannot be optimized and must be dropped.
constexpr std::size_t kMinTrajOptLinearPoints = 4;

struct TargetPose
{
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
};

struct InputStroke
{
  std::size_t mesh_index{ 0 };
  std::size_t source_stroke_index{ 0 };
  std::vector<TargetPose> points;
};

struct ReachableStroke
{
  std::size_t sequence_index{ 0 };
  std::size_t mesh_index{ 0 };
  std::size_t source_stroke_index{ 0 };
  std::vector<TargetPose> targets;
  std::vector<Eigen::VectorXd> seeds;
};

struct TrajectoryPoint
{
  Eigen::VectorXd joints;
  std::string motion_type;
  bool segment_start{ false };
};

struct StrokePlan
{
  std::size_t sequence_index{ 0 };
  std::vector<TrajectoryPoint> points;
  Eigen::VectorXd start_joints;
  Eigen::VectorXd end_joints;
};

[[nodiscard]] bool isFinite(const double value)
{
  return std::isfinite(value);
}

[[nodiscard]] double requiredNumber(const json& object, const char* name)
{
  if (!object.contains(name) || !object.at(name).is_number())
    throw std::runtime_error(std::string("tcp_targets.json field '") + name + "' must be a number");

  const double value = object.at(name).get<double>();
  if (!isFinite(value))
    throw std::runtime_error(std::string("tcp_targets.json field '") + name + "' must be finite");
  return value;
}

[[nodiscard]] std::size_t requiredIndex(const json& object, const char* name)
{
  if (!object.contains(name) ||
      (!object.at(name).is_number_unsigned() && !object.at(name).is_number_integer()))
    throw std::runtime_error(std::string("tcp_targets.json field '") + name + "' must be a non-negative integer");
  if (object.at(name).is_number_integer() && object.at(name).get<std::int64_t>() < 0)
    throw std::runtime_error(std::string("tcp_targets.json field '") + name + "' must be a non-negative integer");
  return object.at(name).get<std::size_t>();
}

[[nodiscard]] TargetPose parseTargetPose(const json& value, std::size_t stroke_index, std::size_t point_index)
{
  if (!value.is_object())
    throw std::runtime_error("stroke " + std::to_string(stroke_index) + ", point " + std::to_string(point_index) +
                             " must be an object");

  const double x = requiredNumber(value, "x");
  const double y = requiredNumber(value, "y");
  const double z = requiredNumber(value, "z");
  const Eigen::Quaterniond quaternion(requiredNumber(value, "qw"),
                                      requiredNumber(value, "qx"),
                                      requiredNumber(value, "qy"),
                                      requiredNumber(value, "qz"));
  if (std::abs(quaternion.norm() - 1.0) > kQuaternionNormTolerance)
    throw std::runtime_error("stroke " + std::to_string(stroke_index) + ", point " + std::to_string(point_index) +
                             " has a non-unit quaternion");

  TargetPose target;
  target.pose.translation() = Eigen::Vector3d(x, y, z);
  target.pose.linear() = quaternion.normalized().toRotationMatrix();
  return target;
}

[[nodiscard]] std::vector<InputStroke> loadTcpTargets(const std::string& input_path)
{
  std::ifstream input(input_path);
  if (!input)
    throw std::runtime_error("Unable to open TCP target file: " + input_path);

  json document;
  try
  {
    input >> document;
  }
  catch (const json::exception& error)
  {
    throw std::runtime_error("Invalid JSON in " + input_path + ": " + error.what());
  }

  if (!document.is_object() || !document.contains("schema_version") || !document.at("schema_version").is_number_integer())
    throw std::runtime_error("tcp_targets.json must contain an integer schema_version");
  if (document.at("schema_version").get<int>() != kTcpTargetsSchemaVersion)
    throw std::runtime_error("Unsupported tcp_targets.json schema_version; expected " +
                             std::to_string(kTcpTargetsSchemaVersion));
  if (!document.contains("strokes") || !document.at("strokes").is_array())
    throw std::runtime_error("tcp_targets.json must contain a strokes array");

  std::vector<InputStroke> strokes;
  const json& source_strokes = document.at("strokes");
  strokes.reserve(source_strokes.size());
  for (std::size_t stroke_index = 0; stroke_index < source_strokes.size(); ++stroke_index)
  {
    const json& source = source_strokes.at(stroke_index);
    if (!source.is_object())
      throw std::runtime_error("stroke " + std::to_string(stroke_index) + " must be an object");
    if (!source.contains("points") || !source.at("points").is_array())
      throw std::runtime_error("stroke " + std::to_string(stroke_index) + " must contain a points array");
    if (source.at("points").size() < 2)
      throw std::runtime_error("stroke " + std::to_string(stroke_index) + " has fewer than two points");

    InputStroke stroke;
    stroke.mesh_index = requiredIndex(source, "mesh_index");
    stroke.source_stroke_index = requiredIndex(source, "stroke_index");
    stroke.points.reserve(source.at("points").size());
    for (std::size_t point_index = 0; point_index < source.at("points").size(); ++point_index)
      stroke.points.push_back(parseTargetPose(source.at("points").at(point_index), stroke_index, point_index));
    strokes.push_back(std::move(stroke));
  }

  if (strokes.empty())
    throw std::runtime_error("tcp_targets.json contains no strokes");
  return strokes;
}

[[nodiscard]] std::string loadTextFile(const std::string& path)
{
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("Unable to open file: " + path);
  return { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
}

struct PlanningContext
{
  std::shared_ptr<Environment> environment;
  std::shared_ptr<ProfileDictionary> profiles;
  std::shared_ptr<const tesseract::kinematics::KinematicGroup> kinematics;
  ManipulatorInfo manipulator;
  std::vector<std::string> joint_names;
};

[[nodiscard]] PlanningContext createPlanningContext(const MotionPlannerConfig& config,
                                                    const std::string& urdf,
                                                    const std::string& srdf)
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
  contact_plugins.discrete_plugin_infos.plugins["BulletDiscreteBVHManager"] =
      { "BulletDiscreteBVHManagerFactory", YAML::Node() };
  contact_plugins.continuous_plugin_infos.default_plugin = "BulletCastBVHManager";
  contact_plugins.continuous_plugin_infos.plugins["BulletCastBVHManager"] =
      { "BulletCastBVHManagerFactory", YAML::Node() };
  context.environment->applyCommand(
      std::make_shared<tesseract::environment::AddContactManagersPluginInfoCommand>(contact_plugins));

  const auto active_info = context.environment->getKinematicsInformation();
  const auto group_it = active_info.kinematics_plugin_info.inv_plugin_infos.find(config.manipulator_group);
  if (group_it == active_info.kinematics_plugin_info.inv_plugin_infos.end() ||
      group_it->second.default_plugin != kKdlPluginName)
    throw std::runtime_error("Tesseract did not retain the requested KDL plugin configuration for group '" +
                             config.manipulator_group + "'");

  context.kinematics = context.environment->getKinematicGroup(config.manipulator_group, kKdlPluginName);
  if (context.kinematics == nullptr)
    throw std::runtime_error("Unable to load KDL plugin '" + std::string(kKdlPluginName) + "' for group '" +
                             config.manipulator_group + "'");

  context.joint_names = context.kinematics->getJointNames();
  if (context.joint_names.empty())
    throw std::runtime_error("KDL group '" + config.manipulator_group + "' exposes no joints");

  context.manipulator.manipulator = config.manipulator_group;
  context.manipulator.tcp_frame = config.tcp_frame;
  context.manipulator.working_frame = config.base_link;

  context.profiles = std::make_shared<ProfileDictionary>();
  auto move_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>();
  // NOTE: TrajOptDefaultMoveProfile's constructor disables cartesian_cost_config and
  // leaves cartesian_constraint_config enabled with zero tolerance, and neither one's
  // tolerance fields take effect unless use_tolerance_override is also set. We want a
  // soft cost (not a hard equality constraint) so TrajOpt can trade a small amount of
  // pose accuracy for feasibility when it conflicts with collision avoidance, and so
  // rotation about the spray tool's symmetric tip axis (last component) stays free
  // instead of being forced into an exact-yaw match.
  move_profile->cartesian_constraint_config.enabled = false;
  move_profile->cartesian_cost_config.enabled = true;
  move_profile->cartesian_cost_config.use_tolerance_override = true;
  move_profile->cartesian_cost_config.lower_tolerance = Eigen::VectorXd::Zero(6);
  move_profile->cartesian_cost_config.lower_tolerance << -config.position_tolerance, -config.position_tolerance,
      -config.position_tolerance, -config.orientation_tolerance * M_PI / 180.0, -config.orientation_tolerance * M_PI / 180.0, -M_PI;
  move_profile->cartesian_cost_config.upper_tolerance = Eigen::VectorXd::Zero(6);
  move_profile->cartesian_cost_config.upper_tolerance << config.position_tolerance, config.position_tolerance,
      config.position_tolerance, config.orientation_tolerance * M_PI / 180.0, config.orientation_tolerance * M_PI / 180.0, M_PI;
  move_profile->cartesian_cost_config.coeff =
      (Eigen::VectorXd(6) << 5.0, 5.0, 5.0, 5.0, 5.0, 0.0).finished();

  auto composite_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
  composite_profile->collision_cost_config.enabled = true;
  composite_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.03, 20.0);
  // The default composite profile also enables a hard collision *constraint* (zero
  // clearance) on top of the soft collision cost above. Combined with the cartesian
  // pose cost, this hard constraint can make some strokes numerically infeasible for
  // the SQP solver even though the soft cost alone already keeps the path well clear
  // of the sprayed surface, so rely on the cost only.
  composite_profile->collision_constraint_config.enabled = false;

  // FREESPACE transitions are a plain 2-state (from, to) joint move, which does not
  // have enough states for the acceleration (needs >= 3) or jerk (needs >= 5)
  // smoothing costs that TrajOptDefaultCompositeProfile enables by default; TrajOpt
  // throws when building the problem in that case. Use a separate composite profile
  // for the transition planner with only velocity smoothing enabled.
  auto transition_composite_profile = std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>();
  transition_composite_profile->collision_cost_config = composite_profile->collision_cost_config;
  transition_composite_profile->collision_constraint_config.enabled = false;
  transition_composite_profile->smooth_accelerations = false;
  transition_composite_profile->smooth_jerks = false;

  // The profile namespace passed to addProfile/getProfile must match the
  // TrajOptMotionPlanner's own name (its constructor argument), not a generic
  // "TrajOpt" label -- otherwise these profiles are silently never found and every
  // solve quietly falls back to the unconfigured TrajOptDefault*Profile (zero-
  // tolerance cartesian equality constraint, default collision margins, etc).
  context.profiles->addProfile(kSprayPlannerName, kSprayProfileName, move_profile);
  context.profiles->addProfile(kSprayPlannerName, kSprayProfileName, composite_profile);
  context.profiles->addProfile(kTransitionPlannerName, kSprayProfileName, move_profile);
  context.profiles->addProfile(kTransitionPlannerName, kSprayProfileName, transition_composite_profile);
  return context;
}

[[nodiscard]] Eigen::VectorXd homeSeed(const PlanningContext& context)
{
  // A zero vector is valid only as an initial seed; subsequent seeds use the
  // preceding accepted KDL solution to preserve waypoint continuity.
  return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(context.joint_names.size()));
}

[[nodiscard]] bool isFiniteVector(const Eigen::VectorXd& value)
{
  return value.size() > 0 && value.allFinite();
}

// Re-solves KDL IK for an already-reachable stroke's target poses starting from a
// fresh seed. Used as a fallback when TrajOpt cannot optimize the joint-configuration
// branch chosen by the continuous seed chain in preSolveIk (see planStroke): the
// greedy "closest to previous solution" branch selection occasionally lands on a
// configuration that is locally very hard for the SQP solver even though the poses
// themselves are reachable and collision-free from another branch.
[[nodiscard]] std::optional<std::vector<Eigen::VectorXd>> resolveSeedChain(
    const std::vector<TargetPose>& targets, Eigen::VectorXd seed, const PlanningContext& context)
{
  std::vector<Eigen::VectorXd> seeds;
  seeds.reserve(targets.size());
  for (const TargetPose& target : targets)
  {
    const tesseract::kinematics::KinGroupIKInput ik_input(
        target.pose, context.manipulator.working_frame, context.manipulator.tcp_frame);
    const auto solutions = context.kinematics->calcInvKin(ik_input, seed);

    Eigen::VectorXd best_solution;
    double best_cost = std::numeric_limits<double>::infinity();
    for (const Eigen::VectorXd& solution : solutions)
    {
      if (solution.size() != seed.size() || !isFiniteVector(solution))
        continue;
      const double cost = (solution - seed).squaredNorm();
      if (cost < best_cost)
      {
        best_cost = cost;
        best_solution = solution;
      }
    }

    if (!isFiniteVector(best_solution))
      return std::nullopt;
    seed = best_solution;
    seeds.push_back(best_solution);
  }
  return seeds;
}

[[nodiscard]] std::vector<ReachableStroke> preSolveIk(const std::vector<InputStroke>& strokes,
                                                       PlanningContext& context)
{
  std::vector<ReachableStroke> reachable;
  Eigen::VectorXd previous_seed = homeSeed(context);

  for (const InputStroke& input_stroke : strokes)
  {
    ReachableStroke fragment;
    fragment.sequence_index = reachable.size();
    fragment.mesh_index = input_stroke.mesh_index;
    fragment.source_stroke_index = input_stroke.source_stroke_index;

    const auto flush_fragment = [&reachable, &fragment]() {
      if (fragment.targets.size() >= kMinTrajOptLinearPoints)
      {
        fragment.sequence_index = reachable.size();
        reachable.push_back(std::move(fragment));
      }
    };

    for (const TargetPose& target : input_stroke.points)
    {
      const tesseract::kinematics::KinGroupIKInput ik_input(
          target.pose, context.manipulator.working_frame, context.manipulator.tcp_frame);
      const auto solutions = context.kinematics->calcInvKin(ik_input, previous_seed);

      Eigen::VectorXd best_solution;
      double best_cost = std::numeric_limits<double>::infinity();
      for (const Eigen::VectorXd& solution : solutions)
      {
        if (solution.size() != previous_seed.size() || !isFiniteVector(solution))
          continue;
        const double cost = (solution - previous_seed).squaredNorm();
        if (cost < best_cost)
        {
          best_cost = cost;
          best_solution = solution;
        }
      }

      if (!isFiniteVector(best_solution))
      {
        flush_fragment();
        fragment = ReachableStroke{};
        fragment.mesh_index = input_stroke.mesh_index;
        fragment.source_stroke_index = input_stroke.source_stroke_index;
        continue;
      }

      previous_seed = best_solution;
      fragment.targets.push_back(target);
      fragment.seeds.push_back(best_solution);
    }
    flush_fragment();
  }

  if (reachable.empty())
    throw std::runtime_error("KDL could not produce any reachable stroke with at least " +
                              std::to_string(kMinTrajOptLinearPoints) + " points");
  return reachable;
}

[[nodiscard]] std::vector<TrajectoryPoint> flattenResponse(const tesseract::motion_planners::PlannerResponse& response,
                                                            const std::string& motion_type,
                                                            bool first_is_segment_start)
{
  std::vector<TrajectoryPoint> points;
  const auto flattened = response.results.flatten(&tesseract::command_language::moveFilter);
  for (const auto& instruction : flattened)
  {
    const auto move = instruction.get().template as<tesseract::command_language::MoveInstructionPoly>();
    if (!move.getWaypoint().isStateWaypoint())
      continue;
    const auto state = move.getWaypoint().template as<StateWaypointPoly>();
    const Eigen::VectorXd position = state.getPosition();
    if (!isFiniteVector(position))
      throw std::runtime_error("TrajOpt returned a non-finite joint position");
    points.push_back({ position, motion_type, first_is_segment_start && points.empty() });
  }
  return points;
}

[[nodiscard]] tesseract::motion_planners::PlannerResponse solveLinearProgram(
    const ReachableStroke& stroke, const std::vector<Eigen::VectorXd>& seeds, const PlanningContext& context)
{
  CompositeInstruction program(kSprayProfileName, context.manipulator, CompositeInstructionOrder::ORDERED);
  program.push_back(MoveInstruction(StateWaypointPoly{ StateWaypoint(context.joint_names, seeds.front()) },
                                    MoveInstructionType::FREESPACE,
                                    kSprayProfileName,
                                    context.manipulator));
  for (std::size_t point_index = 0; point_index < stroke.targets.size(); ++point_index)
  {
    CartesianWaypoint waypoint(stroke.targets.at(point_index).pose);
    JointState seed;
    seed.joint_names = context.joint_names;
    seed.position = seeds.at(point_index);
    waypoint.setSeed(seed);
    program.push_back(MoveInstruction(CartesianWaypointPoly{ waypoint },
                                      MoveInstructionType::LINEAR,
                                      kSprayProfileName,
                                      context.manipulator));
  }

  TrajOptMotionPlanner planner(kSprayPlannerName);
  PlannerRequest request;
  request.env = context.environment;
  request.instructions = program;
  request.profiles = context.profiles;
  return planner.solve(request);
}

[[nodiscard]] StrokePlan planStroke(const ReachableStroke& stroke,
                                    const MotionPlannerConfig& config,
                                    const std::string& urdf,
                                    const std::string& srdf)
{
  PlanningContext context = createPlanningContext(config, urdf, srdf);
  if (stroke.targets.size() < 2 || stroke.targets.size() != stroke.seeds.size())
    throw std::runtime_error("Internal error: invalid reachable stroke " + std::to_string(stroke.sequence_index));

  auto response = solveLinearProgram(stroke, stroke.seeds, context);
  if (!response.successful)
  {
    // The continuous "closest to previous solution" seed chain used by preSolveIk
    // occasionally lands this stroke on a redundant-joint branch that is reachable
    // and collision-free but numerically hard for the SQP solver to optimize. Retry
    // once from a fresh home-seeded IK chain, on a brand-new environment/context
    // (some collision-manager state appears to persist within a reused Environment
    // across solves and can influence convergence), before giving up: any resulting
    // joint discontinuity relative to the previous stroke is absorbed by the
    // FREESPACE transition planned between strokes, so this cannot affect trajectory
    // validity.
    PlanningContext retry_context = createPlanningContext(config, urdf, srdf);
    const std::optional<std::vector<Eigen::VectorXd>> retry_seeds =
        resolveSeedChain(stroke.targets, homeSeed(retry_context), retry_context);
    const std::string primary_error = response.message;
    if (retry_seeds.has_value())
      response = solveLinearProgram(stroke, *retry_seeds, retry_context);
    if (!response.successful)
      throw std::runtime_error("TrajOpt LINEAR planning failed for stroke " + std::to_string(stroke.sequence_index) +
                               ": " + primary_error);
  }

  StrokePlan result;
  result.sequence_index = stroke.sequence_index;
  result.points = flattenResponse(response, "LINEAR", true);
  if (result.points.size() < 2)
    throw std::runtime_error("TrajOpt LINEAR planning returned fewer than two states for stroke " +
                             std::to_string(stroke.sequence_index));
  result.start_joints = result.points.front().joints;
  result.end_joints = result.points.back().joints;
  return result;
}

[[nodiscard]] std::vector<TrajectoryPoint> planTransition(const Eigen::VectorXd& from,
                                                           const Eigen::VectorXd& to,
                                                           const MotionPlannerConfig& config,
                                                           const std::string& urdf,
                                                           const std::string& srdf)
{
  PlanningContext context = createPlanningContext(config, urdf, srdf);
  if (from.size() != static_cast<Eigen::Index>(context.joint_names.size()) ||
      to.size() != static_cast<Eigen::Index>(context.joint_names.size()))
    throw std::runtime_error("FREESPACE transition joint count does not match the KDL group");

  CompositeInstruction program(kSprayProfileName, context.manipulator, CompositeInstructionOrder::ORDERED);
  program.push_back(MoveInstruction(StateWaypointPoly{ StateWaypoint(context.joint_names, from) },
                                    MoveInstructionType::FREESPACE,
                                    kSprayProfileName,
                                    context.manipulator));
  program.push_back(MoveInstruction(StateWaypointPoly{ StateWaypoint(context.joint_names, to) },
                                    MoveInstructionType::FREESPACE,
                                    kSprayProfileName,
                                    context.manipulator));

  TrajOptMotionPlanner planner(kTransitionPlannerName);
  PlannerRequest request;
  request.env = context.environment;
  request.instructions = program;
  request.profiles = context.profiles;
  const auto response = planner.solve(request);
  if (!response.successful)
    throw std::runtime_error("TrajOpt FREESPACE transition failed: " + response.message);

  auto points = flattenResponse(response, "FREESPACE", false);
  if (points.size() < 2)
    throw std::runtime_error("TrajOpt FREESPACE transition returned fewer than two states");
  return points;
}

void appendDistinct(std::vector<TrajectoryPoint>& output, const std::vector<TrajectoryPoint>& incoming)
{
  for (const TrajectoryPoint& point : incoming)
  {
    if (!output.empty() && point.joints.size() == output.back().joints.size() &&
        (point.joints - output.back().joints).norm() <= kDuplicateJointTolerance)
      continue;
    output.push_back(point);
  }
}

[[nodiscard]] json serializeTrajectory(const std::vector<TrajectoryPoint>& points, const std::string& angle_unit)
{
  json result = json::array();
  for (const TrajectoryPoint& point : points)
  {
    json joints = json::array();
    for (Eigen::Index index = 0; index < point.joints.size(); ++index)
    {
      const double value = angle_unit == "deg" ? point.joints[index] * 180.0 / M_PI : point.joints[index];
      joints.push_back(value);
    }
    result.push_back({ { "joint_positions", joints },
                       { "time_from_start", 0.0 },
                       { "segment_start", point.segment_start },
                       { "motion_type", point.motion_type },
                       { "angle_unit", angle_unit },
                       { "ik_solver", kKdlPluginName },
                       { "collision_checked", true } });
  }
  return result;
}

void writeTrajectory(const std::vector<TrajectoryPoint>& trajectory, const MotionPlannerConfig& config)
{
  if (trajectory.empty())
    throw std::runtime_error("Planning completed without trajectory points");

  std::error_code directory_error;
  std::filesystem::create_directories(config.output_directory, directory_error);
  if (directory_error)
    throw std::runtime_error("Unable to create output directory '" + config.output_directory + "': " +
                             directory_error.message());

  // Writing through a temporary file guarantees a failed run cannot expose a
  // partial trajectory.json to visualization or downstream execution tools.
  const std::filesystem::path output_path = std::filesystem::path(config.output_directory) / "trajectory.json";
  const std::filesystem::path temporary_path = output_path.string() + ".tmp";
  {
    std::ofstream output(temporary_path);
    if (!output)
      throw std::runtime_error("Unable to open temporary trajectory output: " + temporary_path.string());
    output << serializeTrajectory(trajectory, config.angle_unit).dump(4) << '\n';
    if (!output)
      throw std::runtime_error("Failed while writing temporary trajectory output: " + temporary_path.string());
  }
  std::filesystem::rename(temporary_path, output_path, directory_error);
  if (directory_error)
  {
    std::filesystem::remove(temporary_path);
    throw std::runtime_error("Unable to finalize trajectory output '" + output_path.string() + "': " +
                             directory_error.message());
  }
}

void validateConfig(const MotionPlannerConfig& config)
{
  if (config.input_path.empty() || config.urdf_path.empty() || config.srdf_path.empty() || config.output_directory.empty() ||
      config.manipulator_group.empty() || config.tcp_frame.empty() || config.base_link.empty())
    throw std::runtime_error("--input, --urdf, --srdf, --outdir, --group, --tcp, and --base-link are required");
  if (config.thread_count == 0)
    throw std::runtime_error("--threads must be a positive integer");
  if (config.thread_count > 256)
    throw std::runtime_error("--threads exceeds the implementation limit of 256");
  if (config.position_tolerance < 0.0 || !isFinite(config.position_tolerance))
    throw std::runtime_error("--position-tolerance must be finite and non-negative");
  if (config.orientation_tolerance < 0.0 || !isFinite(config.orientation_tolerance))
    throw std::runtime_error("--orientation-tolerance must be finite and non-negative");
  if (config.angle_unit != "deg" && config.angle_unit != "rad")
    throw std::runtime_error("--angle-unit must be 'deg' or 'rad'");
}
}  // namespace

void runMotionPlanner(const MotionPlannerConfig& config)
{
  validateConfig(config);
  const std::vector<InputStroke> input_strokes = loadTcpTargets(config.input_path);
  const std::string urdf = loadTextFile(config.urdf_path);
  const std::string srdf = loadTextFile(config.srdf_path);

  // Pre-solving is deliberately serial: each point uses the preceding accepted
  // KDL solution as its seed, making branch selection deterministic.
  PlanningContext ik_context = createPlanningContext(config, urdf, srdf);
  const std::vector<ReachableStroke> reachable_strokes = preSolveIk(input_strokes, ik_context);

  if (config.ik_only)
  {
    std::vector<TrajectoryPoint> trajectory;
    for (const ReachableStroke& stroke : reachable_strokes)
    {
      for (std::size_t index = 0; index < stroke.seeds.size(); ++index)
        trajectory.push_back({ stroke.seeds[index], index == 0 ? "FREESPACE" : "LINEAR", index == 0 });
    }
    writeTrajectory(trajectory, config);
    return;
  }

  ThreadPool pool(config.thread_count);
  std::vector<StrokePlan> stroke_plans;
  stroke_plans.reserve(reachable_strokes.size());

  // Bound the submitted work to one pool-sized batch. If any task fails, get()
  // rethrows and no not-yet-submitted stroke enters the pool.
  for (std::size_t batch_start = 0; batch_start < reachable_strokes.size();)
  {
    const std::size_t batch_end = std::min(batch_start + config.thread_count, reachable_strokes.size());
    std::vector<std::future<StrokePlan>> futures;
    futures.reserve(batch_end - batch_start);
    for (std::size_t index = batch_start; index < batch_end; ++index)
    {
      const ReachableStroke stroke = reachable_strokes.at(index);
      futures.push_back(pool.submit([&config, &urdf, &srdf, stroke]() {
        return planStroke(stroke, config, urdf, srdf);
      }));
    }
    for (std::future<StrokePlan>& future : futures)
      stroke_plans.push_back(future.get());
    batch_start = batch_end;
  }

  // Results are collected by original sequence index, never by completion time.
  std::sort(stroke_plans.begin(), stroke_plans.end(), [](const StrokePlan& lhs, const StrokePlan& rhs) {
    return lhs.sequence_index < rhs.sequence_index;
  });

  std::vector<TrajectoryPoint> trajectory;
  for (std::size_t index = 0; index < stroke_plans.size(); ++index)
  {
    const StrokePlan& current = stroke_plans.at(index);
    if (index != 0)
    {
      const StrokePlan& previous = stroke_plans.at(index - 1);
      appendDistinct(trajectory, planTransition(previous.end_joints, current.start_joints, config, urdf, srdf));
    }
    appendDistinct(trajectory, current.points);
  }
  writeTrajectory(trajectory, config);
}

}  // namespace aisprayer::planner
