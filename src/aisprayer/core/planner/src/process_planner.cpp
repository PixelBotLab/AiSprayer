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
#include <vtkPolyDataNormals.h>
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
 * @note 使用 VTK 根据三角面片的环绕方向计算法向，比 PCL 的点云邻域 PCA 更稳定，且不需要 view_point。
 */
void ensureVertexNormals(pcl::PolygonMesh& mesh, const Eigen::Vector3d& view_point)
{
  vtkSmartPointer<vtkPolyData> vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh, vtk_mesh);

  vtkNew<vtkPolyDataNormals> normal_generator;
  normal_generator->SetInputData(vtk_mesh);
  normal_generator->SplittingOff(); // 保持顶点数量不变，防止网格被切开
  normal_generator->ConsistencyOn(); // 强制统一网格内部的三角面片环绕顺序
  normal_generator->AutoOrientNormalsOff(); // 关闭容易出错的包围盒外翻
  normal_generator->Update();

  pcl::PolygonMesh cleaned;
  pcl::io::vtk2mesh(normal_generator->GetOutput(), cleaned);
  mesh = std::move(cleaned);

  // 统一提取出点云和法线
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud_with_normals);

  // 绝对可靠的朝向修正：由于网格是单目深度相机拍摄的，所有真实的可见表面必然都是朝向相机的！
  // 逐顶点检查，如果法线背向相机（点乘 < 0），则将其翻转。
  // 这样既保留了 VTK 三角面片法向的光顺度（无边缘卷曲），又 100% 保证了全局朝向绝对向外。
  for (auto& pt : cloud_with_normals->points) {
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    Eigen::Vector3d n(pt.normal_x, pt.normal_y, pt.normal_z);
    Eigen::Vector3d dir_to_cam = view_point - p;
    if (n.dot(dir_to_cam) < 0) {
      pt.normal_x = -pt.normal_x;
      pt.normal_y = -pt.normal_y;
      pt.normal_z = -pt.normal_z;
    }
  }

  pcl::toPCLPointCloud2(*cloud_with_normals, mesh.cloud);
}

/**
 * @brief 按照网格 2D 投影的最长轴对工具路径进行栅格化 (Raster) 组织排列。
 * @note 将原始路径旋转至与最长轴平行以进行光顺扫描，规划完后再旋转回原本坐标系。
 */
MeshPCAInfo organizePathsBy2DPCA(noether::ToolPaths& paths, const CameraCalibration& calib, double merge_gap_threshold)
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
    
    // 1. 计算整列在校正后坐标系中的 U 轴平均位置
    double sum_u_rot = 0;
    int count = 0;
    for (const auto& seg : path) {
      for (const auto& pose : seg) {
        double u, v;
        if (calib.project(pose.translation(), u, v)) {
          Eigen::Vector2d rotated = rotation * Eigen::Vector2d(u, v);
          sum_u_rot += rotated.x();
          count++;
        }
      }
    }
    double mean_u_rot = count > 0 ? (sum_u_rot / count) : 0;

    // 2. 判断该列整体方向，统一调整为由上至下 (V轴减小)
    Eigen::Vector3d top_pt = path.front().front().translation();
    Eigen::Vector3d bot_pt = path.back().back().translation();
    double u_t=0, v_t=0, u_b=0, v_b=0;
    calib.project(top_pt, u_t, v_t);
    calib.project(bot_pt, u_b, v_b);
    Eigen::Vector2d rt = rotation * Eigen::Vector2d(u_t, v_t);
    Eigen::Vector2d rb = rotation * Eigen::Vector2d(u_b, v_b);
    
    // 如果整体方向是向上的，则将点序反转，使得方向一致
    if (rt.y() > rb.y()) {
      std::reverse(path.begin(), path.end());
      for (auto& seg : path) {
        std::reverse(seg.begin(), seg.end());
      }
    }

    // 3. 智能条件合并：跨越微小破洞，但保留真实大间隙
    noether::ToolPath new_path;
    if (!path.empty()) {
      noether::ToolPathSegment current_merged = path.front();
      for (size_t i = 1; i < path.size(); ++i) {
        const auto& seg = path[i];
        if (seg.empty()) continue;
        
        // 计算两段之间的物理距离
        double gap = (current_merged.back().translation() - seg.front().translation()).norm();
        if (gap < merge_gap_threshold) { // Threshold for merging small holes/noise
          current_merged.insert(current_merged.end(), seg.begin(), seg.end());
        } else {
          // 如果是大间隙（如裤裆），则保留为独立 stroke
          new_path.push_back(current_merged);
          current_merged = seg;
        }
      }
      new_path.push_back(current_merged);
    }
    
    columns.push_back({new_path, mean_u_rot});
  }

  // 3. 根据 U 轴坐标，从左到右对栅格列进行排序
  std::sort(columns.begin(), columns.end(), [](const ColumnInfo& a, const ColumnInfo& b) {
    return a.mean_u_rot < b.mean_u_rot;
  });

  // 4. 将排序后的路径串联起来，并实行之字形 (Zig-Zag) 反转
  paths.clear();
  bool reverse_next = false;
  for (auto& col : columns) {
    if (reverse_next) {
      // 当前列需要从下往上走
      std::reverse(col.path.begin(), col.path.end());
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
    Eigen::Vector3d view_point(0, 0, 1000);
    if (calib.valid) {
      view_point = calib.T_base_camera.block<3,1>(0,3);
    }
    ensureVertexNormals(mesh, view_point);

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

    // 3. 执行轨迹排布与优化 (移除前置的 RasterOrganizationModifier，防止破坏原始切片列结构)

    MeshPCAInfo pca_info;
    pca_info.axis = Eigen::Vector2d::UnitX();
    pca_info.angle_deg = 0;

    if (!config_.straight_lines) {
      // 启用 2D PCA 对齐策略，生成之字形扫描线
      pca_info = organizePathsBy2DPCA(paths, calib, config_.merge_gap_threshold);
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
    paths = noether::MovingAverageOrientationSmoothingModifier(config_.smoothing_window).modify(std::move(paths));

    // 5. 边缘法向共识修正：
    //    用中间稳定区域 (去掉首尾各 20%) 的法向量平均值作为"共识参考法向"，
    //    只纠正那些偏离共识超过 20 度的边缘点，而不是暴力地钳制首尾所有点。
    //    这样既能修复真正发散的边缘异常点，又完全保留圆柱表面真实的法向变化曲线。
    const double kBoundaryAngleThresholdRad = 12.0 * M_PI / 180.0; // 12° 足以抓住 14-17° 的边缘异常，又不会误判正常圆柱弧面变化(< 10°)
    for (auto& path : paths) {
      for (auto& seg : path) {
        const int n = static_cast<int>(seg.size());
        if (n < 6) continue;

        // 从中间60%的点计算稳定共识法向（Z轴，单位向量取和后归一化）
        int stable_start = n / 5;         // 跳过前 20%
        int stable_end   = n - n / 5;     // 跳过后 20%
        Eigen::Vector3d consensus_z = Eigen::Vector3d::Zero();
        for (int i = stable_start; i < stable_end; ++i) {
          consensus_z += seg[i].linear().col(2);
        }
        if (consensus_z.norm() < 1e-6) continue;
        consensus_z.normalize();

        // 对每个点检查其法向是否偏离共识超过阈值，超过则用共识替换
        for (int i = 0; i < n; ++i) {
          Eigen::Vector3d cur_z = seg[i].linear().col(2);
          double cos_angle = cur_z.dot(consensus_z);
          cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
          if (std::acos(cos_angle) > kBoundaryAngleThresholdRad) {
            // 用共识Z轴重建旋转矩阵，保留X轴方向（走线方向）
            Eigen::Vector3d cur_x = seg[i].linear().col(0);
            Eigen::Vector3d new_y = consensus_z.cross(cur_x);
            if (new_y.norm() < 1e-6) continue;
            new_y.normalize();
            Eigen::Vector3d new_x = new_y.cross(consensus_z);
            new_x.normalize();
            seg[i].linear().col(0) = new_x;
            seg[i].linear().col(1) = new_y;
            seg[i].linear().col(2) = consensus_z;
          }
        }
      }
    }

    // 6. 边缘扭转（twist）对齐：
    //    上面的共识修正已经确保了所有点的 Z 轴（法向）是正确的。
    //    但 idx 0 和末尾点有时因平滑窗口边界效应，X 轴（行进方向）与相邻点
    //    出现 90°~180° 的大扭转，产生 SO(3) 意义上的巨大姿态跳变。
    //    修复：直接把首/末点的 X 轴对齐到其最近邻的 X 轴（保留 Z 轴不变），
    //    从而消除扭转跳变，同时完全不影响中间所有正常点的圆柱弧面法向变化。
    auto alignTwistToNeighbor = [](Eigen::Isometry3d& pt, const Eigen::Isometry3d& neighbor) {
      const Eigen::Vector3d z = pt.linear().col(2);       // 保留已经修正好的 Z（法向）
      const Eigen::Vector3d ref_x = neighbor.linear().col(0); // 借用邻居的 X（行进方向）
      Eigen::Vector3d new_y = z.cross(ref_x);
      if (new_y.norm() < 1e-6) return;
      new_y.normalize();
      Eigen::Vector3d new_x = new_y.cross(z);
      new_x.normalize();
      pt.linear().col(0) = new_x;
      pt.linear().col(1) = new_y;
      // pt.linear().col(2) = z; // Z 不变
    };

    for (auto& path : paths) {
      for (auto& seg : path) {
        const int n = static_cast<int>(seg.size());
        if (n < 3) continue;
        alignTwistToNeighbor(seg[0],   seg[1]);     // 首点对齐到第二个点
        alignTwistToNeighbor(seg[n-1], seg[n-2]);   // 末点对齐到倒数第二个点
      }
    }

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
