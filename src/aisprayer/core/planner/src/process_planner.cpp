#include "process_planner.hpp"
#include "planner_utils.hpp"

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

namespace aisprayer::planner {

namespace {
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

/**
 * @brief 对输入的网格进行清洗（移除重复顶点、孤立点）并统一转换至三角网格。
 */
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

/**
 * @brief 计算网格顶点的法向量，因为路径规划器 PlaneSlicerRasterPlanner 需要法向信息。
 */
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

  for (pcl::Normal& normal : normals->points) {
    if (!std::isfinite(normal.normal_x) || !std::isfinite(normal.normal_y) || !std::isfinite(normal.normal_z)) {
      normal.normal_x = 0.0F;
      normal.normal_y = 0.0F;
      normal.normal_z = 1.0F;
    }
  }

  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  pcl::concatenateFields(*cloud, *normals, cloud_with_normals);
  pcl::toPCLPointCloud2(cloud_with_normals, mesh.cloud);
}

/**
 * @brief 按照网格 2D 投影的最长轴对工具路径进行栅格化 (Raster) 组织排列。
 * @note 将原始路径旋转至与最长轴平行以进行光顺扫描，规划完后再旋转回原本坐标系。
 */
MeshPCAInfo organizePathsBy2DPCA(noether::ToolPaths& paths, const CameraCalibration& calib)
{
  MeshPCAInfo info;
  info.axis = Eigen::Vector2d::UnitX();
  info.angle_deg = 0.0;
  if (!calib.valid || paths.empty()) return info;

  std::vector<Eigen::Vector2d> points_2d;
  for (const auto& path : paths) {
    for (const auto& segment : path) {
      for (const auto& pose : segment) {
        double u, v;
        if (calib.project(pose.translation(), u, v)) {
          points_2d.push_back(Eigen::Vector2d(u, v));
        }
      }
    }
  }

  if (points_2d.size() < 3) return info;

  // 1. 计算点集的质心 (均值)
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& pt : points_2d) mean += pt;
  mean /= points_2d.size();

  // 2. 构建协方差矩阵并求解特征值与特征向量
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (const auto& pt : points_2d) {
    Eigen::Vector2d d = pt - mean;
    cov += d * d.transpose();
  }
  cov /= points_2d.size();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
  // 特征值升序排列，因此 col(1) 对应最大的特征值（最长主成分轴）
  Eigen::Vector2d primary_axis = solver.eigenvectors().col(1);
  if (primary_axis.y() < 0) primary_axis = -primary_axis;

  // 3. 计算对齐至垂直方向 (Y轴正方向，M_PI/2) 所需的旋转角度
  double angle = std::atan2(primary_axis.y(), primary_axis.x());
  double target_angle = M_PI / 2.0; 
  double angle_diff = target_angle - angle;
  
  // 保持角度在 [-90, 90] 度之间
  if (angle_diff > M_PI / 2.0) angle_diff -= M_PI;
  if (angle_diff < -M_PI / 2.0) angle_diff += M_PI;

  // 4. 计算旋转后的轴与角度
  Eigen::Rotation2Dd rotation(angle_diff);
  Eigen::Vector2d new_axis = rotation * primary_axis;
  info.axis = new_axis;
  info.angle_deg = angle_diff * 180.0 / M_PI;
  
  std::cout << "  2D PCA Longest Axis (u,v): " << primary_axis.transpose() 
            << ", Angle correction: " << info.angle_deg << " degrees.\n";

  struct ColumnInfo {
    noether::ToolPath path;
    double mean_u_rot;
  };

  std::vector<ColumnInfo> columns;
  for (auto& path : paths) {
    if (path.empty()) continue;
    
    // 1. 合并由于曲面不连续或障碍物导致断开的同一列栅格段
    noether::ToolPathSegment merged;
    for (const auto& seg : path)
      merged.insert(merged.end(), seg.begin(), seg.end());
    
    // 2. 计算合并后线段在校正后 (对齐主轴) 坐标系中的 U 轴平均位置
    double sum_u_rot = 0;
    int count = 0;
    for (const auto& pose : merged) {
      double u, v;
      if (calib.project(pose.translation(), u, v)) {
        Eigen::Vector2d rotated = rotation * Eigen::Vector2d(u, v);
        sum_u_rot += rotated.x();
        count++;
      }
    }
    double mean_u_rot = count > 0 ? (sum_u_rot / count) : 0;

    // 3. 判断该线段方向，统一调整为由上至下 (V轴减小)
    Eigen::Vector3d top_pt = merged.front().translation();
    Eigen::Vector3d bot_pt = merged.back().translation();
    double u_t=0, v_t=0, u_b=0, v_b=0;
    calib.project(top_pt, u_t, v_t);
    calib.project(bot_pt, u_b, v_b);
    Eigen::Vector2d rt = rotation * Eigen::Vector2d(u_t, v_t);
    Eigen::Vector2d rb = rotation * Eigen::Vector2d(u_b, v_b);
    
    // 如果首点在尾点的下方，则反转点序，使得单条线段方向一致
    if (rt.y() > rb.y()) {
      std::reverse(merged.begin(), merged.end());
    }
    
    noether::ToolPath new_path;
    new_path.push_back(merged);
    columns.push_back({new_path, mean_u_rot});
  }

  // 4. 根据 U 轴坐标，从左到右对栅格列进行排序
  std::sort(columns.begin(), columns.end(), [](const ColumnInfo& a, const ColumnInfo& b) {
    return a.mean_u_rot < b.mean_u_rot;
  });

  // 5. 将排序后的路径串联起来，并实行之字形 (Zig-Zag) 反转
  paths.clear();
  bool reverse_next = false;
  for (auto& col : columns) {
    if (reverse_next) {
      // 当前列需要从下往上走
      for (auto& seg : col.path) {
        std::reverse(seg.begin(), seg.end());
      }
    }
    paths.push_back(col.path);
    reverse_next = !reverse_next; // 奇偶交替
  }
  
  return info;
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

/**
 * @brief 从生成的路径中移除含有 NaN 或者无穷大的无效路点。
 */
void removeNaNWaypoints(noether::ToolPaths& paths)
{
  for (noether::ToolPath& path : paths) {
    for (noether::ToolPathSegment& segment : path) {
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

// TrajOpt's default composite profile includes a joint-jerk cost term that requires
// at least five states per planned segment (one free-space start state plus the
// linear-motion states). Strokes shorter than this cannot be optimized downstream by
// motion_planner, so they are dropped here rather than failing later mid-pipeline.
constexpr std::size_t kMinTrajOptLinearPoints = 4;

/**
 * @brief 将 Noether 规划的轨迹追加到全局的 stroke 列表中。
 * @note 每个点都包含在网格表面的位姿 (surface_pose) 以及沿法向退后 standoff 距离的 TCP 位姿 (tcp_pose)。
 */
void appendPaths(const noether::ToolPaths& paths,
                 std::size_t mesh_index,
                 const std::string& mesh_source,
                 double standoff,
                 std::vector<PlannedStroke>& strokes,
                 std::size_t& skipped_short_strokes)
{
  for (const noether::ToolPath& path : paths) {
    for (const noether::ToolPathSegment& segment : path) {
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
      for (const Eigen::Isometry3d& waypoint : segment) {
        stroke.surface_poses.push_back(waypoint);
        Eigen::Isometry3d target = waypoint;
        target.translation() += waypoint.linear().col(2) * standoff;
        target.linear() *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
        stroke.tcp_poses.push_back(target);
      }
      strokes.push_back(std::move(stroke));
    }
  }
}

/**
 * @brief 解析字符串格式的 3D 向量 (x,y,z)。
 * @note 如果格式不正确或者向量长度为零，将抛出异常。
 */
Eigen::Vector3d parseVector3d(const std::string& value)
{
  std::stringstream stream(value);
  std::string token;
  std::vector<double> values;
  while (std::getline(stream, token, ','))
    values.push_back(std::stod(token));

  if (values.size() != 3)
    throw std::runtime_error("Direction vector must be 'x,y,z', got: " + value);
  return { values[0], values[1], values[2] };
}
} // namespace

/**
 * @brief ProcessPlanner 构造函数。
 * @note 初始化喷涂工艺配置参数 (行间距，点间距，方向等)。
 */
ProcessPlanner::ProcessPlanner(const ProcessConfig& config) : config_(config) {}

/**
 * @brief 核心执行函数：读入一系列 OBJ 网格并执行喷涂工艺规划。
 * @note 首先对网格切片、再对齐长轴方向进行栅格化和间距过滤，最后输出合并后的 JSON 结果对象。如果中途发生严重错误则返回 std::nullopt。
 */
std::optional<json> ProcessPlanner::plan(const std::vector<std::string>& mesh_paths, const std::string& calib_path)
{
  CameraCalibration calib = loadCalibration(calib_path);
  std::vector<PlannedStroke> all_strokes;
  json mesh_infos = json::array();
  std::size_t skipped_short_strokes = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr covered_surface(new pcl::PointCloud<pcl::PointXYZ>);

  for (std::size_t mesh_index = 0; mesh_index < mesh_paths.size(); ++mesh_index)
  {
    const std::string& mesh_path = mesh_paths[mesh_index];
    std::cout << "Planning for mesh " << (mesh_index + 1) << " of " << mesh_paths.size() << ": " << mesh_path << '\n';

    pcl::PolygonMesh mesh;
    if (pcl::io::loadOBJFile(mesh_path, mesh) < 0) {
      std::cerr << "Failed to load mesh: " << mesh_path << '\n';
      continue;
    }

    cleanMesh(mesh);
    ensureVertexNormals(mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *mesh_vertices);

    std::cout << "  Generating tool paths for mesh...\n";

    // 2. 根据设置选择直切 (沿设定方向) 或是自适应长轴切片
    std::unique_ptr<noether::DirectionGenerator> direction_generator;
    if (!config_.direction.empty()) {
      direction_generator = std::make_unique<noether::FixedDirectionGenerator>(parseVector3d(config_.direction));
    } else if (!config_.straight_lines) {
      direction_generator = std::make_unique<LongestAxisDirectionGenerator>();
    } else {
      direction_generator = std::make_unique<noether::PrincipalAxisDirectionGenerator>();
    }
    
    auto origin_generator = std::make_unique<noether::CentroidOriginGenerator>();
    noether::PlaneSlicerRasterPlanner planner(std::move(direction_generator), std::move(origin_generator));
    planner.setLineSpacing(config_.row_spacing);
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

    if (config_.straight_lines)
      straightenRasterSegments(paths);

    // 5. 对轨迹进行等间距重采样，保持运动匀速，并滤除距离过近的点防止机器臂抖动
    double dedup_dist = config_.seam_dedup_distance < 0.0 ? config_.row_spacing * 0.5 : config_.seam_dedup_distance;
    paths = noether::UniformSpacingModifier(config_.point_spacing, 1, true).modify(std::move(paths));

    // 3. 执行轨迹排布与优化
    paths = noether::RasterOrganizationModifier{}.modify(std::move(paths));

    MeshPCAInfo pca_info;
    pca_info.axis = Eigen::Vector2d::UnitX();
    pca_info.angle_deg = 0;

    if (!config_.straight_lines) {
      // 启用 2D PCA 对齐策略，生成之字形扫描线
      pca_info = organizePathsBy2DPCA(paths, calib);
    } else {
      // 如果固定了方向，则根据 image_horizontal 强行重排轨迹顺序
      if (calib.valid && !config_.image_horizontal.empty()) {
        Eigen::Vector3d req_axis = parseVector3d(config_.image_horizontal);
        Eigen::Vector3d req_axis_cam = calib.T_camera_base.block<3,3>(0,0) * req_axis;
        Eigen::Vector2d imageDownAxis(req_axis_cam.x(), req_axis_cam.y());
        if (imageDownAxis.norm() > 1e-6) imageDownAxis.normalize();

        auto organization_modifier = std::make_unique<noether::RasterOrganizationModifier>();
        paths = organization_modifier->modify(paths);
      }
    }

    removeNaNWaypoints(paths);

    if (!covered_surface->empty())
    {
      std::size_t removed_waypoints = 0;
      paths = removeCoveredSegments(paths, covered_surface, dedup_dist, removed_waypoints);
      std::cout << "  Removed " << removed_waypoints << " seam-overlap waypoints\n";
    }

    // 4. 对工具方向进行平滑，避免急剧扭转
    paths = noether::FixedOrientationModifier(Eigen::Vector3d::UnitY()).modify(std::move(paths));
    paths = noether::MovingAverageOrientationSmoothingModifier(5).modify(std::move(paths));
    
    if (config_.straight_lines)
      lockStraightSegmentOrientations(paths);

    // 滤除包含 NaN 或无穷大元素的无效路点
    removeNaNWaypoints(paths);

    // 6. 追加到输出容器中
    appendPaths(paths, mesh_index, mesh_path, config_.standoff, all_strokes, skipped_short_strokes);
    *covered_surface += *mesh_vertices;

    json mesh_info = {
        {"mesh_index", mesh_index},
        {"mesh_source", mesh_path},
        {"pca_longest_axis_u", pca_info.axis.x()},
        {"pca_longest_axis_v", pca_info.axis.y()},
        {"pca_angle_correction_deg", pca_info.angle_deg}
    };
    mesh_infos.push_back(std::move(mesh_info));
  }

  // Combine into unified JSON structure
  json combined_strokes = json::array();
  for (std::size_t stroke_index = 0; stroke_index < all_strokes.size(); ++stroke_index)
  {
    const PlannedStroke& stroke = all_strokes[stroke_index];
    json combined_points = json::array();

    for (std::size_t point_index = 0; point_index < stroke.tcp_poses.size(); ++point_index)
    {
      json combined_point = poseJson(stroke.tcp_poses.at(point_index));
      json surface_pose = poseJson(stroke.surface_poses.at(point_index));
      
      combined_point["surface_x"] = surface_pose["x"];
      combined_point["surface_y"] = surface_pose["y"];
      combined_point["surface_z"] = surface_pose["z"];
      combined_point["surface_qw"] = surface_pose["qw"];
      combined_point["surface_qx"] = surface_pose["qx"];
      combined_point["surface_qy"] = surface_pose["qy"];
      combined_point["surface_qz"] = surface_pose["qz"];
      
      combined_point["segment_start"] = point_index == 0;
      combined_point["mesh_source"] = stroke.mesh_source;
      
      double u = 0, v = 0;
      if (calib.project(stroke.surface_poses.at(point_index).translation(), u, v))
      {
        combined_point["u"] = u;
        combined_point["v"] = v;
      }
      
      combined_points.push_back(std::move(combined_point));
    }

    combined_strokes.push_back({ 
        { "mesh_index", stroke.mesh_index }, 
        { "stroke_index", stroke_index }, 
        { "points", std::move(combined_points) } 
    });
  }

  return json{
      {"version", 2},
      {"mesh_info", std::move(mesh_infos)},
      {"strokes", std::move(combined_strokes)}
  };
}

/**
 * @brief 将 plan 返回的 JSON 对象写入指定的输出文件路径。
 * @note 自动创建缺失的目录，并进行结构化缩进 (dump(4)) 保存以方便阅读和调试。
 */
bool ProcessPlanner::save(const json& process_targets, const std::string& output_file) const
{
  std::filesystem::path path(output_file);
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }
  
  std::ofstream out_file(output_file);
  if (out_file) {
      out_file << process_targets.dump(4);
  } else {
      std::cerr << "Failed to write " << output_file << "\n";
      return false;
  }

  return true;
}

} // namespace aisprayer::planner
