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
#include <yaml-cpp/yaml.h>

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

struct CameraCalibration
{
  Eigen::Matrix4d T_base_camera = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_camera_base = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d intrinsic_matrix = Eigen::Matrix3d::Identity();
  std::vector<double> distortion_coeffs;

  bool valid = false;

  bool project(const Eigen::Vector3d& p_base, double& u, double& v) const
  {
    if (!valid) return false;

    // 1. Transform to camera frame
    Eigen::Vector4d p_cam_4 = T_camera_base * p_base.homogeneous();
    if (p_cam_4.z() <= 0) return false; // Behind camera

    // 2. Normalize
    double x_n = p_cam_4.x() / p_cam_4.z();
    double y_n = p_cam_4.y() / p_cam_4.z();

    // 3. Distortion
    double x_d = x_n;
    double y_d = y_n;
    if (distortion_coeffs.size() >= 5)
    {
      double r2 = x_n * x_n + y_n * y_n;
      double r4 = r2 * r2;
      double r6 = r4 * r2;
      double k1 = distortion_coeffs[0];
      double k2 = distortion_coeffs[1];
      double p1 = distortion_coeffs[2];
      double p2 = distortion_coeffs[3];
      double k3 = distortion_coeffs[4];

      double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
      x_d = x_n * radial + 2.0 * p1 * x_n * y_n + p2 * (r2 + 2.0 * x_n * x_n);
      y_d = y_n * radial + p1 * (r2 + 2.0 * y_n * y_n) + 2.0 * p2 * x_n * y_n;
    }

    // 4. Intrinsic projection
    double fx = intrinsic_matrix(0, 0);
    double fy = intrinsic_matrix(1, 1);
    double cx = intrinsic_matrix(0, 2);
    double cy = intrinsic_matrix(1, 2);

    u = fx * x_d + cx;
    v = fy * y_d + cy;

    return true;
  }
};

CameraCalibration loadCameraCalibration(const std::string& path)
{
  CameraCalibration calib;
  if (path.empty()) return calib;

  try
  {
    YAML::Node root = YAML::LoadFile(path);

    const YAML::Node t_node = root["T_base_camera"];
    if (t_node.IsSequence() && t_node.size() == 4)
    {
      for (int i = 0; i < 4; ++i)
      {
        if (t_node[i].IsSequence() && t_node[i].size() == 4)
        {
          for (int j = 0; j < 4; ++j)
          {
            double val = t_node[i][j].as<double>();
            if (j == 3 && i < 3) {
              val /= 1000.0; // Convert mm to meters
            }
            calib.T_base_camera(i, j) = val;
          }
        }
      }
      calib.T_camera_base = calib.T_base_camera.inverse();
    }

    const YAML::Node cam_params = root["camera_params"];
    if (cam_params.IsMap())
    {
      const YAML::Node int_node = cam_params["intrinsic_matrix"];
      if (int_node.IsSequence() && int_node.size() == 3)
      {
        for (int i = 0; i < 3; ++i)
        {
          if (int_node[i].IsSequence() && int_node[i].size() == 3)
          {
            for (int j = 0; j < 3; ++j)
            {
              calib.intrinsic_matrix(i, j) = int_node[i][j].as<double>();
            }
          }
        }
      }

      const YAML::Node dist_node = cam_params["distortion_coeffs"];
      if (dist_node.IsSequence())
      {
        for (std::size_t i = 0; i < dist_node.size(); ++i)
        {
          calib.distortion_coeffs.push_back(dist_node[i].as<double>());
        }
      }
    }

    calib.valid = true;
  }
  catch (const YAML::Exception& e)
  {
    std::cerr << "Failed to load camera calibration: " << e.what() << '\n';
  }

  return calib;
}



struct MeshPCAInfo {
  Eigen::Vector2d axis;
  double angle_deg;
};

MeshPCAInfo organizePathsBy2DPCA(noether::ToolPaths& paths, const CameraCalibration& calib)
{
  MeshPCAInfo info;
  info.axis = Eigen::Vector2d::UnitX();
  info.angle_deg = 0.0;

  if (!calib.valid || paths.empty()) return info;

  // 1. Gather all 2D points to compute PCA
  std::vector<Eigen::Vector2d> points_2d;
  for (const auto& path : paths)
  {
    for (const auto& segment : path)
    {
      for (const auto& pose : segment)
      {
        double u = 0, v = 0;
        if (calib.project(pose.translation(), u, v))
        {
          points_2d.push_back(Eigen::Vector2d(u, v));
        }
      }
    }
  }

  if (points_2d.size() < 2) return info;

  // Mean center
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& pt : points_2d) {
    mean += pt;
  }
  mean /= points_2d.size();

  // Covariance matrix
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (const auto& pt : points_2d) {
    Eigen::Vector2d diff = pt - mean;
    cov += diff * diff.transpose();
  }
  cov /= (points_2d.size() - 1);

  // Compute Eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  Eigen::Vector2d longest_axis = eig.eigenvectors().col(1);
  if (longest_axis.x() < 0) {
    longest_axis = -longest_axis; // Ensure it points to the right (positive X)
  }

  double angle_rad = std::atan2(longest_axis.y(), longest_axis.x());
  
  info.axis = longest_axis;
  info.angle_deg = angle_rad * 180.0 / M_PI;

  // We want to rotate the coordinate system so that the longest axis becomes VERTICAL.
  // In image coordinates, vertical means along the Y-axis (v-axis).
  // The angles for vertical are PI/2 or -PI/2.
  // We choose the one that requires the minimum rotation angle.
  double rot_angle = 0;
  if (angle_rad >= 0) {
      rot_angle = M_PI / 2.0 - angle_rad;
  } else {
      rot_angle = -M_PI / 2.0 - angle_rad;
  }

  // Rotation matrix to rotate points by rot_angle
  Eigen::Matrix2d R;
  R << std::cos(rot_angle), -std::sin(rot_angle),
       std::sin(rot_angle),  std::cos(rot_angle);

  struct ColumnInfo {
    noether::ToolPath path;
    double mean_u_rot;
  };

  std::vector<ColumnInfo> columns;
  for (auto& path : paths)
  {
    if (path.empty()) continue;

    noether::ToolPathSegment merged;
    for (const auto& segment : path) {
      merged.insert(merged.end(), segment.begin(), segment.end());
    }
    if (merged.empty()) continue;
    
    // Evaluate start and end points in rotated coordinates
    double start_u = 0, start_v = 0;
    calib.project(merged.front().translation(), start_u, start_v);
    Eigen::Vector2d start_rot = R * Eigen::Vector2d(start_u, start_v);
    
    double end_u = 0, end_v = 0;
    calib.project(merged.back().translation(), end_u, end_v);
    Eigen::Vector2d end_rot = R * Eigen::Vector2d(end_u, end_v);

    // We want the segment to point DOWN in the rotated 2D projection (v_rot increases)
    if (start_rot.y() > end_rot.y()) {
      std::reverse(merged.begin(), merged.end());
    }

    double sum_u_rot = 0;
    size_t sz = merged.size();
    for (const auto& pose : merged)
    {
      double u = 0, v = 0;
      calib.project(pose.translation(), u, v);
      Eigen::Vector2d rot = R * Eigen::Vector2d(u, v);
      sum_u_rot += rot.x();
    }
    
    noether::ToolPath new_path;
    new_path.push_back(std::move(merged));
    columns.push_back({std::move(new_path), sum_u_rot / sz});
  }

  // Sort columns left-to-right (by mean_u_rot)
  std::sort(columns.begin(), columns.end(), [](const ColumnInfo& a, const ColumnInfo& b){
    return a.mean_u_rot < b.mean_u_rot;
  });

  paths.clear();
  bool go_down = true; 
  for (auto& col : columns)
  {
    // The segment currently goes DOWN. If snake says UP, reverse it.
    if (!go_down) {
      std::reverse(col.path.front().begin(), col.path.front().end());
    }
    paths.push_back(std::move(col.path));
    go_down = !go_down; // Alternate
  }
  
  return info;
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

struct PlannedStroke
{
  std::size_t mesh_index;
  std::string mesh_source;
  std::vector<Eigen::Isometry3d> surface_poses;
  std::vector<Eigen::Isometry3d> tcp_poses;
};

void appendPaths(const noether::ToolPaths& paths,
                 std::size_t mesh_index,
                 const std::string& mesh_source,
                 double standoff,
                 std::vector<PlannedStroke>& strokes,
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

      PlannedStroke stroke;
      stroke.mesh_index = mesh_index;
      stroke.mesh_source = mesh_source;
      stroke.surface_poses.reserve(segment.size());
      stroke.tcp_poses.reserve(segment.size());
      for (const Eigen::Isometry3d& surface_pose : segment)
      {
        Eigen::Isometry3d tcp_pose = surface_pose;
        tcp_pose.translate(Eigen::Vector3d(0.0, 0.0, standoff));
        tcp_pose.rotate(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
        stroke.surface_poses.push_back(surface_pose);
        stroke.tcp_poses.push_back(tcp_pose);
      }
      strokes.push_back(std::move(stroke));
    }
  }
}

void orderTcpStrokesLeftToRight(std::vector<PlannedStroke>& strokes, const Eigen::Vector3d& image_right)
{
  std::stable_sort(strokes.begin(), strokes.end(), [&image_right](const PlannedStroke& lhs, const PlannedStroke& rhs) {
    const auto mean_projection = [&image_right](const PlannedStroke& stroke) {
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (const Eigen::Isometry3d& pose : stroke.tcp_poses)
        mean += pose.translation();
      return (mean / static_cast<double>(stroke.tcp_poses.size())).dot(image_right);
    };
    return mean_projection(lhs) < mean_projection(rhs);
  });
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

  const CameraCalibration calib = loadCameraCalibration(options_.calibration_path);
  if (calib.valid) {
    std::cout << "Loaded camera calibration. T_base_camera:\n" << calib.T_base_camera << '\n';
  } else {
    std::cout << "Warning: Invalid or missing camera calibration.\n";
  }

  std::error_code error;
  std::filesystem::create_directories(options_.output_directory, error);
  if (error)
    throw std::runtime_error("Failed to create output directory " + options_.output_directory + ": " + error.message());

  json surface_points = json::array();
  json tcp_strokes = json::array();
  json mesh_infos = json::array();
  std::size_t skipped_short_strokes = 0;
  std::vector<PlannedStroke> planned_strokes;
  pcl::PointCloud<pcl::PointXYZ>::Ptr covered_surface(new pcl::PointCloud<pcl::PointXYZ>);
  const std::vector<std::string> mesh_paths = splitMeshes(options_.mesh_paths);

  for (std::size_t mesh_index = 0; mesh_index < mesh_paths.size(); ++mesh_index)
  {
    const std::string& mesh_path = mesh_paths[mesh_index];
    std::string mesh_source = std::filesystem::path(mesh_path).filename().string();
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
    
    // Use the custom 2D PCA based organization instead of the default Noether ones
    MeshPCAInfo pca_info = organizePathsBy2DPCA(paths, calib);
    
    json info_json = {
      { "mesh_source", mesh_source },
      { "pca_axis_u", pca_info.axis.x() },
      { "pca_axis_v", pca_info.axis.y() },
      { "pca_angle_deg", pca_info.angle_deg }
    };
    mesh_infos.push_back(std::move(info_json));
    
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

    appendPaths(paths, mesh_index, mesh_source, options_.standoff, planned_strokes, skipped_short_strokes);
    *covered_surface += *mesh_vertices;
  }

  if (planned_strokes.empty())
    throw std::runtime_error("No valid Noether spray strokes were generated");

  // Sort after applying the TCP standoff as well. Surface normals can shift the
  // visible TCP lines laterally, and sorting all meshes together prevents the
  // input OBJ order from splitting an otherwise continuous left-to-right sweep.
  // orderTcpStrokesLeftToRight(planned_strokes, image_right);

  for (std::size_t stroke_index = 0; stroke_index < planned_strokes.size(); ++stroke_index)
  {
    const PlannedStroke& stroke = planned_strokes.at(stroke_index);
    json tcp_points = json::array();
    for (std::size_t point_index = 0; point_index < stroke.tcp_poses.size(); ++point_index)
    {
      json surface_point = poseJson(stroke.surface_poses.at(point_index));
      surface_point["segment_start"] = point_index == 0;
      surface_point["mesh_source"] = stroke.mesh_source;
      
      double u = 0, v = 0;
      if (calib.project(stroke.surface_poses.at(point_index).translation(), u, v))
      {
        surface_point["u"] = u;
        surface_point["v"] = v;
      }
      
      surface_points.push_back(std::move(surface_point));
      tcp_points.push_back(poseJson(stroke.tcp_poses.at(point_index)));
    }
    tcp_strokes.push_back(
        { { "mesh_index", stroke.mesh_index }, { "stroke_index", stroke_index }, { "points", std::move(tcp_points) } });
  }

  json parameters = {
    { "standoff", options_.standoff },
    { "row_spacing", options_.row_spacing },
    { "point_spacing", options_.point_spacing },
    { "straight_lines", options_.straight_lines },
    { "direction", options_.direction.empty() ? json(nullptr) : json(options_.direction) },
    { "calibration_path", options_.calibration_path.empty() ? json(nullptr) : json(options_.calibration_path) },
    { "seam_dedup_distance", dedup_distance }
  };
  const std::filesystem::path output_directory(options_.output_directory);
  writeJson(output_directory / "path_surface.json",
            { { "mesh_info", std::move(mesh_infos) }, { "surface_points", std::move(surface_points) } });
  writeJson(output_directory / "tcp_targets.json",
            { { "schema_version", 1 }, { "process_parameters", std::move(parameters) }, { "strokes", std::move(tcp_strokes) } });
  std::cout << "Saved " << surface_points.size() << " surface points and " << planned_strokes.size()
            << " TCP strokes to " << output_directory << '\n';
  if (skipped_short_strokes > 0)
    std::cout << "  Dropped " << skipped_short_strokes
              << " seam-remnant fragment(s) shorter than " << kMinTrajOptLinearPoints
              << " points (too short for downstream TrajOpt planning)\n";
}

}  // namespace aisprayer::planner
