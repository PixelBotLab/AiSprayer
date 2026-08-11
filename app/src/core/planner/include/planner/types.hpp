#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace aisprayer::planner
{

/**
 * A Cartesian position and unit quaternion orientation.
 *
 * Quaternion components use the x, y, z, w convention.
 */
struct Pose
{
  double x{ 0.0 };
  double y{ 0.0 };
  double z{ 0.0 };
  double qx{ 0.0 };
  double qy{ 0.0 };
  double qz{ 0.0 };
  double qw{ 1.0 };

  [[nodiscard]] bool isFinite() const noexcept;
  [[nodiscard]] bool hasUnitQuaternion(double tolerance = 1e-6) const noexcept;
  void validate(double quaternion_tolerance = 1e-6) const;
};

/** An ordered, indivisible process stroke. */
struct Stroke
{
  std::size_t mesh_index{ 0 };
  std::size_t stroke_index{ 0 };
  std::vector<Pose> points;

  void validate(double quaternion_tolerance = 1e-6) const;
};

/**
 * Versioned transport document exchanged by process and motion planning.
 *
 * The scalar process parameters correspond to the values stored in the
 * process_parameters object of tcp_targets.json.
 */
struct TcpTargetsDocument
{
  static constexpr std::uint32_t kSchemaVersion = 1;

  std::uint32_t schema_version{ kSchemaVersion };
  double standoff{ 0.0 };
  double row_spacing{ 0.0 };
  double point_spacing{ 0.0 };
  std::vector<Stroke> strokes;

  void validate(double quaternion_tolerance = 1e-6) const;
};

enum class MotionType
{
  FREESPACE,
  LINEAR,
};

/** A robot joint-space trajectory sample. */
struct TrajectoryPoint
{
  std::vector<double> joint_positions;
  double time_from_start{ 0.0 };
  bool segment_start{ false };
  MotionType motion_type{ MotionType::FREESPACE };

  void validate() const;
};

}  // namespace aisprayer::planner
