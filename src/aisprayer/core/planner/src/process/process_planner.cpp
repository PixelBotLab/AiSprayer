#include <planner/process_planner.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vtkCleanPolyData.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/core/types.h>
#include <noether_tpp/tool_path_modifiers/fixed_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/moving_average_orientation_smoothing_modifier.h>
#include <noether_tpp/tool_path_modifiers/raster_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/snake_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/uniform_spacing_modifier.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/fixed_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/centroid_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>

namespace aisprayer::planner
{
namespace
{
using json = nlohmann::json;

class LongestAxisDirectionGenerator : public noether::DirectionGenerator
{
public:
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    if (cloud.empty())
      throw std::runtime_error("Cannot determine a raster direction for an empty mesh");

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud.makeShared());
    const Eigen::Vector3d longest_axis = pca.getEigenVectors().col(0).cast<double>().normalized();
    if (!longest_axis.allFinite())
      throw std::runtime_error("PCA produced an invalid raster direction");

    std::cout << "  Auto raster direction (PCA longest axis): " << longest_axis.transpose() << '\n';
    return longest_axis;
  }
};

std::vector<std::string> splitMeshes(const std::string& value)
{
  std::vector<std::string> meshes;
  std::stringstream stream(value);
  std::string mesh;
  while (std::getline(stream, mesh, ','))
  {
    if (mesh.empty())
      throw std::runtime_error("--mesh contains an empty path");
    meshes.push_back(mesh);
  }
  if (meshes.empty())
    throw std::runtime_error("--mesh must contain at least one OBJ path");
  return meshes;
}

Eigen::Vector3d parseVector3d(const std::string& value)
{
  std::stringstream stream(value);
  std::string token;
  std::vector<double> values;
  while (std::getline(stream, token, ','))
    values.push_back(std::stod(token));

  if (values.size() != 3)
    throw std::runtime_error("Direction vector must be 'x,y,z', got: " + value);

  const Eigen::Vector3d vector(values[0], values[1], values[2]);
  if (!vector.allFinite() || vector.norm() == 0.0)
    throw std::runtime_error("Direction vector must be finite and non-zero: " + value);
  return vector;
}

void cleanMesh(pcl::PolygonMesh& mesh)
{
  vtkSmartPointer<vtkPolyData> vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh, vtk_mesh);

  vtkNew<vtkCleanPolyData> cleaner;
  cleaner->SetInputData(vtk_mesh);
  cleaner->ToleranceIsAbsoluteOn();
  cleaner->SetAbsoluteTolerance(1e-5);
  cleaner->ConvertLinesToPointsOff();
  cleaner->ConvertPolysToLinesOff();
  cleaner->ConvertStripsToPolysOn();
  cleaner->Update();

  vtkNew<vtkTriangleFilter> triangle_filter;
  triangle_filter->SetInputConnection(cleaner->GetOutputPort());
  triangle_filter->Update();

  pcl::PolygonMesh cleaned;
  pcl::io::vtk2mesh(triangle_filter->GetOutput(), cleaned);
  mesh = std::move(cleaned);
}

void ensureVertexNormals(pcl::PolygonMesh& mesh)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  if (cloud->empty())
    throw std::runtime_error("Mesh has no vertices after cleaning");

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimator;
  estimator.setInputCloud(cloud);
  estimator.setKSearch(20);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  estimator.compute(*normals);

  for (pcl::Normal& normal : normals->points)
  {
    if (!std::isfinite(normal.normal_x) || !std::isfinite(normal.normal_y) || !std::isfinite(normal.normal_z))
    {
      normal.normal_x = 0.0F;
      normal.normal_y = 0.0F;
      normal.normal_z = 1.0F;
    }
  }

  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  pcl::concatenateFields(*cloud, *normals, cloud_with_normals);
  pcl::toPCLPointCloud2(cloud_with_normals, mesh.cloud);
}

void orderRastersLeftToRight(noether::ToolPaths& paths, const Eigen::Vector3d& image_right)
{
  const Eigen::Vector3d right = image_right.normalized();
  std::vector<noether::ToolPathSegment> segments;
  for (noether::ToolPath& path : paths)
  {
    for (noether::ToolPathSegment& segment : path)
    {
      if (!segment.empty())
        segments.push_back(std::move(segment));
    }
  }

  std::stable_sort(segments.begin(), segments.end(), [&right](const auto& lhs, const auto& rhs) {
    Eigen::Vector3d lhs_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d rhs_mean = Eigen::Vector3d::Zero();
    for (const Eigen::Isometry3d& waypoint : lhs)
      lhs_mean += waypoint.translation();
    for (const Eigen::Isometry3d& waypoint : rhs)
      rhs_mean += waypoint.translation();
    return (lhs_mean / static_cast<double>(lhs.size())).dot(right) <
           (rhs_mean / static_cast<double>(rhs.size())).dot(right);
  });

  paths.clear();
  paths.reserve(segments.size());
  for (noether::ToolPathSegment& segment : segments)
    paths.push_back(noether::ToolPath{ std::move(segment) });
}

void straightenRasterSegments(noether::ToolPaths& paths)
{
  for (noether::ToolPath& path : paths)
  {
    for (noether::ToolPathSegment& segment : path)
    {
      if (segment.size() < 2)
        continue;

      const Eigen::Isometry3d start = segment.front();
      const Eigen::Isometry3d end = segment.back();
      Eigen::Isometry3d midpoint = Eigen::Isometry3d::Identity();
      midpoint.translation() = 0.5 * (start.translation() + end.translation());
      midpoint.linear() =
          Eigen::Quaterniond(start.rotation()).slerp(0.5, Eigen::Quaterniond(end.rotation())).toRotationMatrix();
      segment = { start, midpoint, end };
    }
  }
}

void lockStraightSegmentOrientations(noether::ToolPaths& paths)
{
  for (noether::ToolPath& path : paths)
  {
    for (noether::ToolPathSegment& segment : path)
    {
      if (segment.empty())
        continue;
      const Eigen::Matrix3d rotation = segment.front().rotation();
      for (Eigen::Isometry3d& waypoint : segment)
        waypoint.linear() = rotation;
    }
  }
}

void removeNaNWaypoints(noether::ToolPaths& paths)
{
  for (noether::ToolPath& path : paths)
  {
    for (noether::ToolPathSegment& segment : path)
    {
      segment.erase(std::remove_if(segment.begin(), segment.end(), [](const Eigen::Isometry3d& waypoint) {
                      return !waypoint.matrix().allFinite();
                    }),
                    segment.end());
    }
    path.erase(std::remove_if(path.begin(), path.end(), [](const noether::ToolPathSegment& segment) {
                 return segment.empty();
               }),
               path.end());
  }
  paths.erase(std::remove_if(paths.begin(), paths.end(), [](const noether::ToolPath& path) {
                return path.empty();
              }),
              paths.end());
}

noether::ToolPaths removeCoveredSegments(const noether::ToolPaths& paths,
                                         const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& covered_surface,
                                         double dedup_distance,
                                         std::size_t& removed_waypoints)
{
  if (covered_surface->empty() || dedup_distance <= 0.0)
    return paths;

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(covered_surface);
  const float dedup_distance_sq = static_cast<float>(dedup_distance * dedup_distance);
  noether::ToolPaths filtered_paths;

  for (const noether::ToolPath& path : paths)
  {
    noether::ToolPath filtered_path;
    for (const noether::ToolPathSegment& segment : path)
    {
      noether::ToolPathSegment retained;
      for (const Eigen::Isometry3d& waypoint : segment)
      {
        const Eigen::Vector3d position = waypoint.translation();
        const pcl::PointXYZ query(position.x(), position.y(), position.z());
        std::vector<int> nearest_index;
        std::vector<float> nearest_distance_sq;
        const bool covered = tree.nearestKSearch(query, 1, nearest_index, nearest_distance_sq) > 0 &&
                             nearest_distance_sq.front() <= dedup_distance_sq;
        if (covered)
        {
          ++removed_waypoints;
          if (retained.size() >= 2)
            filtered_path.push_back(std::move(retained));
          retained.clear();
        }
        else
        {
          retained.push_back(waypoint);
        }
      }
      if (retained.size() >= 2)
        filtered_path.push_back(std::move(retained));
    }
    if (!filtered_path.empty())
      filtered_paths.push_back(std::move(filtered_path));
  }
  return filtered_paths;
}

json poseJson(const Eigen::Isometry3d& pose)
{
  Eigen::Quaterniond quaternion(pose.rotation());
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() <= std::numeric_limits<double>::epsilon())
    throw std::runtime_error("Cannot serialize a pose with an invalid orientation");
  quaternion.normalize();
  return {
    { "x", pose.translation().x() }, { "y", pose.translation().y() }, { "z", pose.translation().z() },
    { "qx", quaternion.x() }, { "qy", quaternion.y() }, { "qz", quaternion.z() }, { "qw", quaternion.w() }
  };
}

// TrajOpt's default composite profile includes a joint-jerk cost term that requires
// at least five states per planned segment (one free-space start state plus the
// linear-motion states). Strokes shorter than this cannot be optimized downstream by
// motion_planner, so they are dropped here rather than failing later mid-pipeline.
constexpr std::size_t kMinTrajOptLinearPoints = 4;

void appendPaths(const noether::ToolPaths& paths,
                 std::size_t mesh_index,
                 double standoff,
                 json& surface_points,
                 json& tcp_strokes,
                 std::size_t& next_stroke_index,
                 std::size_t& skipped_short_strokes)
{
  constexpr double kPi = 3.14159265358979323846;
  for (const noether::ToolPath& path : paths)
  {
    for (const noether::ToolPathSegment& segment : path)
    {
      if (segment.size() < kMinTrajOptLinearPoints)
      {
        if (!segment.empty())
          ++skipped_short_strokes;
        continue;
      }

      json tcp_points = json::array();
      bool segment_start = true;
      for (const Eigen::Isometry3d& surface_pose : segment)
      {
        json surface_point = poseJson(surface_pose);
        surface_point["segment_start"] = segment_start;
        surface_points.push_back(std::move(surface_point));

        Eigen::Isometry3d tcp_pose = surface_pose;
        tcp_pose.translate(Eigen::Vector3d(0.0, 0.0, standoff));
        tcp_pose.rotate(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
        tcp_points.push_back(poseJson(tcp_pose));
        segment_start = false;
      }
      tcp_strokes.push_back(
          { { "mesh_index", mesh_index }, { "stroke_index", next_stroke_index++ }, { "points", std::move(tcp_points) } });
    }
  }
}

void writeJson(const std::filesystem::path& path, const json& value)
{
  std::ofstream output(path);
  if (!output)
    throw std::runtime_error("Could not write output file: " + path.string());
  output << value.dump(4) << '\n';
  if (!output)
    throw std::runtime_error("Failed while writing output file: " + path.string());
}
}  // namespace

ProcessPlanner::ProcessPlanner(ProcessPlannerOptions options) : options_(std::move(options)) {}

void ProcessPlanner::run() const
{
  if (!std::isfinite(options_.standoff) || options_.standoff < 0.0)
    throw std::runtime_error("--distance must be finite and non-negative");
  if (!std::isfinite(options_.row_spacing) || options_.row_spacing <= 0.0)
    throw std::runtime_error("--row-spacing must be finite and positive");
  if (!std::isfinite(options_.point_spacing) || options_.point_spacing <= 0.0)
    throw std::runtime_error("--point-spacing must be finite and positive");

  const double dedup_distance =
      options_.seam_dedup_distance < 0.0 ? options_.row_spacing * 0.5 : options_.seam_dedup_distance;
  if (!std::isfinite(dedup_distance) || dedup_distance < 0.0)
    throw std::runtime_error("--seam-dedup-distance must be finite or negative for the default");

  std::error_code error;
  std::filesystem::create_directories(options_.output_directory, error);
  if (error)
    throw std::runtime_error("Failed to create output directory " + options_.output_directory + ": " + error.message());

  json surface_points = json::array();
  json tcp_strokes = json::array();
  std::size_t stroke_index = 0;
  std::size_t skipped_short_strokes = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr covered_surface(new pcl::PointCloud<pcl::PointXYZ>);
  const std::vector<std::string> mesh_paths = splitMeshes(options_.mesh_paths);

  for (std::size_t mesh_index = 0; mesh_index < mesh_paths.size(); ++mesh_index)
  {
    const std::string& mesh_path = mesh_paths[mesh_index];
    std::cout << "Loading mesh " << mesh_index << " from " << mesh_path << '\n';
    pcl::PolygonMesh mesh;
    if (pcl::io::loadOBJFile(mesh_path, mesh) < 0)
      throw std::runtime_error("Mesh load failed: " + mesh_path);

    cleanMesh(mesh);
    ensureVertexNormals(mesh);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *mesh_vertices);

    std::unique_ptr<noether::DirectionGenerator> direction_generator;
    if (options_.direction.empty())
      direction_generator = std::make_unique<LongestAxisDirectionGenerator>();
    else
      direction_generator = std::make_unique<noether::FixedDirectionGenerator>(parseVector3d(options_.direction));

    auto origin_generator = std::make_unique<noether::CentroidOriginGenerator>();
    noether::PlaneSlicerRasterPlanner planner(std::move(direction_generator), std::move(origin_generator));
    planner.setLineSpacing(options_.row_spacing);
    planner.generateRastersBidirectionally(true);
    noether::ToolPaths paths = planner.plan(mesh);

    for (noether::ToolPath& path : paths)
    {
      path.erase(std::remove_if(path.begin(), path.end(), [](const noether::ToolPathSegment& segment) {
                   return segment.size() < 4;
                 }),
                 path.end());
    }
    paths.erase(std::remove_if(paths.begin(), paths.end(), [](const noether::ToolPath& path) {
                  return path.empty();
                }),
                paths.end());

    if (options_.straight_lines)
      straightenRasterSegments(paths);
    paths = noether::UniformSpacingModifier(options_.point_spacing, 1, true).modify(std::move(paths));
    paths = noether::RasterOrganizationModifier{}.modify(std::move(paths));
    if (!options_.image_horizontal.empty())
      orderRastersLeftToRight(paths, parseVector3d(options_.image_horizontal));
    paths = noether::SnakeOrganizationModifier{}.modify(std::move(paths));
    removeNaNWaypoints(paths);

    if (!covered_surface->empty())
    {
      std::size_t removed_waypoints = 0;
      paths = removeCoveredSegments(paths, covered_surface, dedup_distance, removed_waypoints);
      std::cout << "  Removed " << removed_waypoints << " seam-overlap waypoints\n";
    }

    paths = noether::FixedOrientationModifier(Eigen::Vector3d::UnitY()).modify(std::move(paths));
    paths = noether::MovingAverageOrientationSmoothingModifier(5).modify(std::move(paths));
    if (options_.straight_lines)
      lockStraightSegmentOrientations(paths);
    removeNaNWaypoints(paths);

    appendPaths(paths, mesh_index, options_.standoff, surface_points, tcp_strokes, stroke_index, skipped_short_strokes);
    *covered_surface += *mesh_vertices;
  }

  if (tcp_strokes.empty())
    throw std::runtime_error("No valid Noether spray strokes were generated");

  json parameters = {
    { "standoff", options_.standoff },
    { "row_spacing", options_.row_spacing },
    { "point_spacing", options_.point_spacing },
    { "straight_lines", options_.straight_lines },
    { "direction", options_.direction.empty() ? json(nullptr) : json(options_.direction) },
    { "image_horizontal", options_.image_horizontal.empty() ? json(nullptr) : json(options_.image_horizontal) },
    { "seam_dedup_distance", dedup_distance }
  };
  const std::filesystem::path output_directory(options_.output_directory);
  writeJson(output_directory / "path_surface.json", surface_points);
  writeJson(output_directory / "tcp_targets.json",
            { { "schema_version", 1 }, { "process_parameters", std::move(parameters) }, { "strokes", std::move(tcp_strokes) } });
  std::cout << "Saved " << surface_points.size() << " surface points and " << stroke_index
            << " TCP strokes to " << output_directory << '\n';
  if (skipped_short_strokes > 0)
    std::cout << "  Dropped " << skipped_short_strokes
              << " seam-remnant fragment(s) shorter than " << kMinTrajOptLinearPoints
              << " points (too short for downstream TrajOpt planning)\n";
}

}  // namespace aisprayer::planner
