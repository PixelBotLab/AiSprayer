#include "planner/types.hpp"

#include <cmath>
#include <sstream>

namespace aisprayer::planner
{
namespace
{

void requireFinite(const double value, const char* name)
{
  if (!std::isfinite(value))
  {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

}  // namespace

bool Pose::isFinite() const noexcept
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(qx) &&
         std::isfinite(qy) && std::isfinite(qz) && std::isfinite(qw);
}

bool Pose::hasUnitQuaternion(const double tolerance) const noexcept
{
  if (!std::isfinite(tolerance) || tolerance < 0.0 || !isFinite())
  {
    return false;
  }

  const double squared_norm = qx * qx + qy * qy + qz * qz + qw * qw;
  return std::abs(squared_norm - 1.0) <= tolerance;
}

void Pose::validate(const double quaternion_tolerance) const
{
  if (!std::isfinite(quaternion_tolerance) || quaternion_tolerance < 0.0)
  {
    throw std::invalid_argument("quaternion_tolerance must be finite and non-negative");
  }
  if (!isFinite())
  {
    throw std::invalid_argument("Pose values must be finite");
  }
  if (!hasUnitQuaternion(quaternion_tolerance))
  {
    throw std::invalid_argument("Pose quaternion must be normalized");
  }
}

void Stroke::validate(const double quaternion_tolerance) const
{
  if (points.size() < 2)
  {
    throw std::invalid_argument("Stroke must contain at least two points");
  }

  for (std::size_t index = 0; index < points.size(); ++index)
  {
    try
    {
      points[index].validate(quaternion_tolerance);
    }
    catch (const std::invalid_argument& error)
    {
      std::ostringstream message;
      message << "Stroke point " << index << " is invalid: " << error.what();
      throw std::invalid_argument(message.str());
    }
  }
}

void TcpTargetsDocument::validate(const double quaternion_tolerance) const
{
  if (schema_version != kSchemaVersion)
  {
    throw std::invalid_argument("unsupported tcp targets schema version");
  }

  requireFinite(standoff, "standoff");
  requireFinite(row_spacing, "row_spacing");
  requireFinite(point_spacing, "point_spacing");
  if (standoff < 0.0 || row_spacing <= 0.0 || point_spacing <= 0.0)
  {
    throw std::invalid_argument(
        "standoff must be non-negative; row_spacing and point_spacing must be positive");
  }
  if (strokes.empty())
  {
    throw std::invalid_argument("TcpTargetsDocument must contain at least one stroke");
  }

  for (std::size_t index = 0; index < strokes.size(); ++index)
  {
    try
    {
      strokes[index].validate(quaternion_tolerance);
    }
    catch (const std::invalid_argument& error)
    {
      std::ostringstream message;
      message << "Stroke " << index << " is invalid: " << error.what();
      throw std::invalid_argument(message.str());
    }
  }
}

void TrajectoryPoint::validate() const
{
  if (joint_positions.empty())
  {
    throw std::invalid_argument("TrajectoryPoint must contain joint positions");
  }
  if (!std::isfinite(time_from_start) || time_from_start < 0.0)
  {
    throw std::invalid_argument("time_from_start must be finite and non-negative");
  }

  for (const double position : joint_positions)
  {
    requireFinite(position, "joint position");
  }
}

}  // namespace aisprayer::planner
