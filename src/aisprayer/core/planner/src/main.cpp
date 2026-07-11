#include <iostream>
#include <fstream>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <cxxopts.hpp>

// PCL & VTK
#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vtkCleanPolyData.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

// Noether
#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/core/types.h>
#include <noether_tpp/tool_path_modifiers/direction_of_travel_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/moving_average_orientation_smoothing_modifier.h>
#include <noether_tpp/tool_path_modifiers/raster_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/snake_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/fixed_orientation_modifier.h>
#include <noether_tpp/tool_path_modifiers/uniform_spacing_modifier.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/fixed_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/centroid_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>

// Tesseract
#include <tesseract/environment/environment.h>
#include <tesseract/srdf/kinematics_information.h>
#include <tesseract/environment/commands/add_kinematics_information_command.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/plugin_info.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/environment/commands/add_contact_managers_plugin_info_command.h>
#include <yaml-cpp/yaml.h>

#include <tesseract/command_language/poly/instruction_poly.h>
#include <tesseract/command_language/poly/state_waypoint_poly.h>
#include <tesseract/command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract/command_language/state_waypoint.h>
#include <tesseract/command_language/cartesian_waypoint.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/motion_planners/planner.h>
#include <tesseract/motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract/collision/types.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/motion_planners/types.h>

using namespace tesseract::environment;
using namespace tesseract::scene_graph;
using namespace tesseract::common;
using namespace tesseract::command_language;
using namespace tesseract::motion_planners;
using json = nlohmann::json;

namespace
{

class LongestAxisDirectionGenerator : public noether::DirectionGenerator {
public:
    Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);

        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloud.makeShared());
        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

        // PlaneSlicerRasterPlanner 将此向量作为切平面内的路径方向；切平面的法向
        // 则由 direction × surface_normal 计算。因此应直接使用 PCA 最长主轴
        // (col(0))，让每条往返喷涂线沿裤腿而非横跨裤腿。
        const Eigen::Vector3d longest_axis = eigen_vectors.col(0).cast<double>().normalized();
        std::cout << "  Auto raster direction (PCA longest axis): "
                  << longest_axis.transpose() << std::endl;
        return longest_axis;
    }
};


std::vector<std::string> split(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

Eigen::Vector3d parseVector3d(const std::string& s)
{
  std::vector<double> vals;
  std::stringstream ss(s);
  std::string token;
  while (std::getline(ss, token, ','))
  {
    vals.push_back(std::stod(token));
  }
  if (vals.size() != 3)
  {
    throw std::runtime_error("Direction vector must be 'x,y,z', got: " + s);
  }
  return Eigen::Vector3d(vals[0], vals[1], vals[2]);
}

Eigen::Vector3d toolPathMeanPosition(const noether::ToolPath& path)
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  std::size_t count = 0;
  for (const noether::ToolPathSegment& segment : path)
  {
    for (const Eigen::Isometry3d& waypoint : segment)
    {
      sum += waypoint.translation();
      ++count;
    }
  }

  if (count == 0)
    throw std::runtime_error("Cannot order an empty tool path");

  return sum / static_cast<double>(count);
}

void orderRastersLeftToRight(noether::ToolPaths& paths, const Eigen::Vector3d& image_right)
{
  const Eigen::Vector3d right = image_right.normalized();
  std::vector<noether::ToolPathSegment> raster_segments;
  for (noether::ToolPath& path : paths)
  {
    for (noether::ToolPathSegment& segment : path)
    {
      if (!segment.empty())
        raster_segments.push_back(std::move(segment));
    }
  }

  // A Noether ToolPath can contain several disconnected intersections. Sort
  // every physical raster segment globally rather than only sorting their
  // parent containers; otherwise one group can yield the visual order 2→4→3.
  std::stable_sort(
      raster_segments.begin(),
      raster_segments.end(),
      [&right](const noether::ToolPathSegment& a, const noether::ToolPathSegment& b) {
        Eigen::Vector3d a_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d b_mean = Eigen::Vector3d::Zero();
        for (const Eigen::Isometry3d& waypoint : a)
          a_mean += waypoint.translation();
        for (const Eigen::Isometry3d& waypoint : b)
          b_mean += waypoint.translation();
        a_mean /= static_cast<double>(a.size());
        b_mean /= static_cast<double>(b.size());
        return a_mean.dot(right) < b_mean.dot(right);
      });

  paths.clear();
  paths.reserve(raster_segments.size());
  for (noether::ToolPathSegment& segment : raster_segments)
    paths.push_back(noether::ToolPath{std::move(segment)});
}

void straightenRasterSegments(noether::ToolPaths& paths)
{
  for (noether::ToolPath& path : paths)
  {
    for (noether::ToolPathSegment& segment : path)
    {
      if (segment.size() < 2)
        continue;

      // Plane slicing follows every local wrinkle of the reconstructed mesh.
      // For the requested spray process, each raster stroke should instead be
      // the direct chord from its first surface point to its last surface point.
      const Eigen::Isometry3d start = segment.front();
      const Eigen::Isometry3d end = segment.back();
      Eigen::Isometry3d midpoint = Eigen::Isometry3d::Identity();
      midpoint.translation() = 0.5 * (start.translation() + end.translation());
      midpoint.linear() =
          Eigen::Quaterniond(start.rotation()).slerp(0.5, Eigen::Quaterniond(end.rotation())).toRotationMatrix();
      segment.clear();
      segment.push_back(start);
      // UniformSpacingModifier uses a degree-1 spline and requires at least
      // degree + 2 waypoints; this collinear midpoint preserves a straight line.
      segment.push_back(midpoint);
      segment.push_back(end);
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

      // A varying normal changes the later standoff translation point-by-point,
      // which bends an otherwise straight surface stroke. Keep one approximate
      // tool attitude for the entire stroke so the TCP path is also straight.
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
      segment.erase(std::remove_if(segment.begin(), segment.end(), [](const Eigen::Isometry3d& wp) {
        return std::isnan(wp.translation().x()) || std::isnan(wp.translation().y()) || std::isnan(wp.translation().z()) ||
               std::isnan(wp.linear()(0, 0));
      }), segment.end());
    }
    // Remove empty segments
    path.erase(std::remove_if(path.begin(), path.end(), [](const noether::ToolPathSegment& seg) {
      return seg.empty();
    }), path.end());
  }
  // Remove empty paths
  paths.erase(std::remove_if(paths.begin(), paths.end(), [](const noether::ToolPath& p) {
    return p.empty();
  }), paths.end());
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
      noether::ToolPathSegment retained_segment;
      for (const Eigen::Isometry3d& waypoint : segment)
      {
        const Eigen::Vector3d position = waypoint.translation();
        const pcl::PointXYZ query(position.x(), position.y(), position.z());
        std::vector<int> nearest_index;
        std::vector<float> nearest_distance_sq;
        const bool is_covered =
            tree.nearestKSearch(query, 1, nearest_index, nearest_distance_sq) > 0 &&
            nearest_distance_sq.front() <= dedup_distance_sq;

        if (is_covered)
        {
          ++removed_waypoints;
          // A removed overlap point divides the original raster into independent
          // spray strokes; do not connect the remaining endpoints across it.
          if (retained_segment.size() >= 2)
            filtered_path.push_back(std::move(retained_segment));
          retained_segment.clear();
        }
        else
        {
          retained_segment.push_back(waypoint);
        }
      }

      if (retained_segment.size() >= 2)
        filtered_path.push_back(std::move(retained_segment));
    }

    if (!filtered_path.empty())
      filtered_paths.push_back(std::move(filtered_path));
  }

  return filtered_paths;
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

  vtkNew<vtkTriangleFilter> tri_filter;
  tri_filter->SetInputConnection(cleaner->GetOutputPort());
  tri_filter->Update();

  pcl::PolygonMesh cleaned;
  vtkSmartPointer<vtkPolyData> cleaned_vtk = tri_filter->GetOutput();
  pcl::io::vtk2mesh(cleaned_vtk, cleaned);

  mesh = std::move(cleaned);
}

void ensureVertexNormals(pcl::PolygonMesh& mesh)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setKSearch(20);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  // Replace NaN normals to prevent downstream Noether/Slerp math from blowing up
  for (std::size_t i = 0; i < normals->size(); ++i) {
      if (std::isnan(normals->points[i].normal_x) || 
          std::isnan(normals->points[i].normal_y) || 
          std::isnan(normals->points[i].normal_z)) {
          normals->points[i].normal_x = 0.0f;
          normals->points[i].normal_y = 0.0f;
          normals->points[i].normal_z = 1.0f;
      }
  }

  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  pcl::concatenateFields(*cloud, *normals, cloud_with_normals);
  pcl::toPCLPointCloud2(cloud_with_normals, mesh.cloud);
}

void extractTargetMatrices(const noether::ToolPaths& paths,
                           std::vector<Eigen::Isometry3d>& out_matrices,
                           std::vector<bool>& out_segment_starts,
                           json& out_json_waypoints,
                           double standoff)
{
  for (const noether::ToolPath& path : paths)
  {
    for (const noether::ToolPathSegment& segment : path)
    {
      bool is_first_waypoint_in_segment = true;
      for (const Eigen::Isometry3d& waypoint : segment)
      {
        const Eigen::Vector3d p = waypoint.translation();
        const Eigen::Quaterniond q(waypoint.rotation());

        json pt;
        pt["x"] = p.x(); pt["y"] = p.y(); pt["z"] = p.z();
        pt["qx"] = q.x(); pt["qy"] = q.y(); pt["qz"] = q.z(); pt["qw"] = q.w();
        pt["segment_start"] = is_first_waypoint_in_segment;
        out_json_waypoints.push_back(pt);
            
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translate(p);
        pose.rotate(q);
        
        static bool first_print = true;
        if (first_print) {
            std::cout << "\n--- DEBUG INFO ---" << std::endl;
            std::cout << "Original Surface Point from Noether: " << pose.translation().transpose() << std::endl;
            std::cout << "Original Z-axis (Normal): " << pose.linear().col(2).transpose() << std::endl;
        }
        
        pose.translate(Eigen::Vector3d(0, 0, standoff));
        pose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
        
        if (first_print) {
            std::cout << "Final TCP Point: " << pose.translation().transpose() << std::endl;
            std::cout << "Final Z-axis (Nozzle): " << pose.linear().col(2).transpose() << std::endl;
            std::cout << "------------------\n" << std::endl;
            first_print = false;
        }
        
        out_matrices.push_back(pose);
        out_segment_starts.push_back(is_first_waypoint_in_segment);
        is_first_waypoint_in_segment = false;
      }
    }
  }
}


}  // namespace

// Load file to string
std::string loadFile(const std::string& path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("Could not open file: " + path);
    }
    return std::string((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
}

int main(int argc, char** argv) {
    cxxopts::Options options("trajectory_planner_cli", "Unified Noether + Tesseract Trajectory Planner");

    options.add_options()
        ("urdf", "URDF File Path", cxxopts::value<std::string>())
        ("srdf", "SRDF File Path", cxxopts::value<std::string>())
        ("m,mesh", "Input Mesh Path (jeans_smoothed.obj)", cxxopts::value<std::string>())
        ("o,outdir", "Output Directory for JSON files", cxxopts::value<std::string>())
        ("distance", "Spray Distance (standoff)", cxxopts::value<double>()->default_value("0.20"))
        ("group", "Manipulator Group Name", cxxopts::value<std::string>()->default_value("cr5_group"))
        ("tcp", "TCP Frame Name", cxxopts::value<std::string>()->default_value("Link6"))
        ("r,row_spacing", "Row Spacing", cxxopts::value<double>()->default_value("0.10"))
        ("p,point_spacing", "Point Spacing", cxxopts::value<double>()->default_value("0.01"))
        ("position-tolerance", "Cartesian position tolerance in meters (0 keeps exact points)",
         cxxopts::value<double>()->default_value("0.005"))
        ("straight-lines", "Replace each raster segment with a straight start-to-end stroke",
         cxxopts::value<bool>()->default_value("false"))
        ("angle_unit", "deg or rad", cxxopts::value<std::string>()->default_value("deg"))
        ("d,direction", "Direction Vector 'x,y,z'", cxxopts::value<std::string>()->default_value(""))
        ("image-horizontal", "Camera image-right axis expressed in robot-base coordinates 'x,y,z'",
         cxxopts::value<std::string>()->default_value(""))
        ("seam-dedup-distance", "Remove later-leg path points within this distance (m) of an earlier leg mesh; "
         "negative means half the row spacing", cxxopts::value<double>()->default_value("-1"))
        ("kdl-only", "Write reachable KDL joint seeds with Cartesian poses; skip TrajOpt (simulation only)",
         cxxopts::value<bool>()->default_value("false"))
        ("noether-only", "Only output Noether surface path, skip Tesseract", cxxopts::value<bool>()->default_value("false"))
        ("h,help", "Print usage");

    auto result = options.parse(argc, argv);

    if (result.count("help") || !result.count("urdf") || !result.count("srdf") || !result.count("mesh") || !result.count("outdir")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    std::string urdf_path = result["urdf"].as<std::string>();
    std::string srdf_path = result["srdf"].as<std::string>();
    std::string mesh_path = result["mesh"].as<std::string>();
    std::string outdir = result["outdir"].as<std::string>();
    double standoff = result["distance"].as<double>();
    std::string group_name = result["group"].as<std::string>();
    std::string tcp_name = result["tcp"].as<std::string>();
    
    double spray_width = result["row_spacing"].as<double>();
    double pt_spacing = result["point_spacing"].as<double>();
    double position_tolerance = result["position-tolerance"].as<double>();
    bool straight_lines = result["straight-lines"].as<bool>();
    std::string angle_unit = result["angle_unit"].as<std::string>();
    std::string cut_dir_str = result["direction"].as<std::string>();
    std::string image_horizontal_str = result["image-horizontal"].as<std::string>();
    double seam_dedup_distance = result["seam-dedup-distance"].as<double>();
    bool kdl_only = result["kdl-only"].as<bool>();
    bool noether_only = result["noether-only"].as<bool>();
    if (seam_dedup_distance < 0.0)
      seam_dedup_distance = spray_width * 0.5;
    if (position_tolerance < 0.0)
      throw std::runtime_error("--position-tolerance must be non-negative");

    std::cout << "======================================" << std::endl;
    std::cout << " Planner Configuration:" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "  URDF Path:     " << urdf_path << std::endl;
    std::cout << "  SRDF Path:     " << srdf_path << std::endl;
    std::cout << "  Mesh Path:     " << mesh_path << std::endl;
    std::cout << "  Output Dir:    " << outdir << std::endl;
    std::cout << "  Distance:      " << standoff << " m" << std::endl;
    std::cout << "  Group Name:    " << group_name << std::endl;
    std::cout << "  TCP Name:      " << tcp_name << std::endl;
    std::cout << "  Row Spacing:   " << spray_width << " m" << std::endl;
    std::cout << "  Point Spacing: " << pt_spacing << " m" << std::endl;
    std::cout << "  Position Tol:  " << position_tolerance << " m" << std::endl;
    std::cout << "  Raster Shape:  " << (straight_lines ? "Straight End-to-End" : "Surface Following") << std::endl;
    std::cout << "  Angle Unit:    " << angle_unit << std::endl;
    std::cout << "  Cut Direction: "
              << (cut_dir_str.empty() ? "Auto (PCA Longest Axis Per Mesh)" : cut_dir_str) << std::endl;
    std::cout << "  Raster Order:  "
              << (image_horizontal_str.empty() ? "Planner Default" : "Image Left -> Right") << std::endl;
    std::cout << "  Seam Dedup:    " << seam_dedup_distance << " m" << std::endl;
    std::cout << "======================================" << std::endl;

    std::string path_surface_file = outdir + "/path_surface.json";
    std::string trajectory_file = outdir + "/trajectory.json";
    std::error_code output_dir_error;
    std::filesystem::create_directories(outdir, output_dir_error);
    if (output_dir_error) {
        std::cerr << "Failed to create output directory " << outdir << ": "
                  << output_dir_error.message() << std::endl;
        return 1;
    }
    
    std::vector<Eigen::Isometry3d> target_matrices;

    // --- PHASE 1: NOETHER PATH GENERATION ---
    std::vector<std::string> mesh_paths = split(mesh_path, ',');
    std::vector<std::vector<Eigen::Isometry3d>> all_target_matrices;
    std::vector<std::vector<bool>> all_segment_starts;
    json surface_json = json::array();
    pcl::PointCloud<pcl::PointXYZ>::Ptr covered_surface(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& m_path : mesh_paths) {
        try {
            std::cout << "Loading mesh from " << m_path << std::endl;
            pcl::PolygonMesh mesh;
            if (pcl::io::loadOBJFile(m_path, mesh) < 0) {
                throw std::runtime_error("Mesh load failed: " + m_path);
            }
            cleanMesh(mesh);
            ensureVertexNormals(mesh);
            pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_vertices(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(mesh.cloud, *mesh_vertices);

            std::unique_ptr<noether::DirectionGenerator> dir_gen;
            if (cut_dir_str.empty()) {
                dir_gen = std::make_unique<LongestAxisDirectionGenerator>();
            } else {
                Eigen::Vector3d cut_direction = parseVector3d(cut_dir_str);
                dir_gen = std::make_unique<noether::FixedDirectionGenerator>(cut_direction);
            }
            auto origin_gen = std::make_unique<noether::CentroidOriginGenerator>();

            noether::PlaneSlicerRasterPlanner mesh_planner(std::move(dir_gen), std::move(origin_gen));
            mesh_planner.setLineSpacing(spray_width);
            mesh_planner.generateRastersBidirectionally(true);

            
            noether::ToolPaths paths = mesh_planner.plan(mesh);

            // Filter out short segments that crash spline fitting
            for (auto& path : paths) {
                path.erase(std::remove_if(path.begin(), path.end(), [](const noether::ToolPathSegment& seg) {
                    return seg.size() < 4; 
                }), path.end());
            }
            paths.erase(std::remove_if(paths.begin(), paths.end(), [](const noether::ToolPath& path) {
                return path.empty();
            }), paths.end());

            if (straight_lines) {
                straightenRasterSegments(paths);
            }

            paths = noether::UniformSpacingModifier(pt_spacing, 1, true).modify(std::move(paths));
            paths = noether::RasterOrganizationModifier{}.modify(std::move(paths));
            if (!image_horizontal_str.empty()) {
                orderRastersLeftToRight(paths, parseVector3d(image_horizontal_str));
            }
            paths = noether::SnakeOrganizationModifier{}.modify(std::move(paths));
            removeNaNWaypoints(paths);
            if (!covered_surface->empty()) {
                std::size_t removed_waypoints = 0;
                paths = removeCoveredSegments(paths, covered_surface, seam_dedup_distance, removed_waypoints);
                std::cout << "  -> Removed " << removed_waypoints
                          << " duplicate overlap waypoints from this leg." << std::endl;
            }
            paths = noether::FixedOrientationModifier(Eigen::Vector3d::UnitY()).modify(std::move(paths));
            paths = noether::MovingAverageOrientationSmoothingModifier(5).modify(std::move(paths));
            if (straight_lines) {
                lockStraightSegmentOrientations(paths);
            }

            std::vector<Eigen::Isometry3d> target_matrices;
            std::vector<bool> segment_starts;
            extractTargetMatrices(paths, target_matrices, segment_starts, surface_json, standoff);
            if (!target_matrices.empty()) {
                all_target_matrices.push_back(target_matrices);
                all_segment_starts.push_back(segment_starts);
            }
            *covered_surface += *mesh_vertices;
            
        } catch (const std::exception& e) {
            std::cerr << "Noether planning failed for mesh " << m_path << ": " << e.what() << std::endl;
            return 1;
        }
    }

    if (all_target_matrices.empty()) {
        std::cerr << "No valid targets found in Noether output across any meshes." << std::endl;
        return 1;
    }
    
    std::ofstream out_surf(path_surface_file);
    out_surf << surface_json.dump(4);
    out_surf.close();
    std::cout << "Noether toolpaths generated & saved to: " << path_surface_file << " (Points: " << surface_json.size() << ")" << std::endl;

    if (noether_only) {
        std::cout << "noether-only flag provided, exiting successfully." << std::endl;
        return 0;
    }

// --- PHASE 2: TESSERACT TRAJECTORY OPTIMIZATION ---
    std::string urdf_string = loadFile(urdf_path);
    std::string srdf_string = loadFile(srdf_path);

    auto env = std::make_shared<Environment>();
    auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    if (!env->init(urdf_string, srdf_string, locator)) {
        std::cerr << "Failed to initialize Tesseract Environment." << std::endl;
        return 1;
    }

    std::cout << "======================================" << std::endl;
    auto kin_info = env->getKinematicsInformation();
    
    // KDL directly evaluates the URDF kinematic chain, so its FK, IK, joint
    // limits, and the geometry used by collision checking remain consistent.
    // This avoids relying on an OPW parameterization whose dimensions and
    // joint-zero conventions do not match this CR5 URDF.
    YAML::Node kdl_config;
    kdl_config["base_link"] = "base_link";
    kdl_config["tip_link"] = tcp_name;
    kdl_config["position_eps"] = 1e-5;
    kdl_config["position_iterations"] = 200;

    kin_info.kinematics_plugin_info.inv_plugin_infos[group_name].default_plugin = "KDLInvKinChainNR_JL";
    kin_info.kinematics_plugin_info.inv_plugin_infos[group_name].plugins["KDLInvKinChainNR_JL"] = {
        "KDLInvKinChainNR_JLFactory", kdl_config
    };
    
    env->applyCommand(std::make_shared<tesseract::environment::AddKinematicsInformationCommand>(kin_info));

    // Refresh kin_info for printing
    auto active_kin_info = env->getKinematicsInformation();
    if (active_kin_info.kinematics_plugin_info.inv_plugin_infos.find(group_name) != active_kin_info.kinematics_plugin_info.inv_plugin_infos.end()) {
        std::cout << " Active IK Plugin: " << active_kin_info.kinematics_plugin_info.inv_plugin_infos.at(group_name).default_plugin << std::endl;
    } else {
        std::cout << " Active IK Plugin: Default Numerical" << std::endl;
    }
    std::cout << "======================================" << std::endl;

    tesseract::common::ContactManagersPluginInfo plugin_info;
    plugin_info.discrete_plugin_infos.default_plugin = "BulletDiscreteBVHManager";
    plugin_info.discrete_plugin_infos.plugins["BulletDiscreteBVHManager"] = {"BulletDiscreteBVHManagerFactory", YAML::Node()};
    
    plugin_info.continuous_plugin_infos.default_plugin = "BulletCastBVHManager";
    plugin_info.continuous_plugin_infos.plugins["BulletCastBVHManager"] = {"BulletCastBVHManagerFactory", YAML::Node()};
    
    env->applyCommand(std::make_shared<tesseract::environment::AddContactManagersPluginInfoCommand>(plugin_info));
    
    ManipulatorInfo manip_info;
    manip_info.manipulator = group_name;
    manip_info.tcp_frame = tcp_name;
    manip_info.working_frame = "base_link";
    
    std::cout << "Configuring manipulator: " << manip_info.manipulator << ", TCP: " << manip_info.tcp_frame << std::endl;

    Eigen::VectorXd current_joints(6);
    // Use the URDF/SRDF "home" posture as the numerical IK seed rather than an
    // unrelated hard-coded pose. These values are stored in radians in the SRDF.
    current_joints << 0.0, 0.4337, -1.4695, -0.2602, 1.7175, 0.0;

    auto profiles = std::make_shared<ProfileDictionary>();
    auto move_profile = std::make_shared<TrajOptDefaultMoveProfile>();
    
    // Restore tolerances as requested by user
    move_profile->cartesian_cost_config.lower_tolerance = Eigen::VectorXd::Zero(6);
    move_profile->cartesian_cost_config.lower_tolerance << -position_tolerance, -position_tolerance, -position_tolerance,
        -15.0 * M_PI / 180.0, -15.0 * M_PI / 180.0, -M_PI;
    
    move_profile->cartesian_cost_config.upper_tolerance = Eigen::VectorXd::Zero(6);
    move_profile->cartesian_cost_config.upper_tolerance << position_tolerance, position_tolerance, position_tolerance,
        15.0 * M_PI / 180.0, 15.0 * M_PI / 180.0, M_PI;
    
    move_profile->cartesian_cost_config.coeff = Eigen::VectorXd::Zero(6);
    move_profile->cartesian_cost_config.coeff << 5.0, 5.0, 5.0, 5.0, 5.0, 0.0;

    auto comp_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
    // Restore original collision config (commented out)
    comp_profile->collision_cost_config.enabled = true;
    comp_profile->collision_cost_config = trajopt_common::TrajOptCollisionConfig(0.03, 20.0);

    profiles->addProfile("TrajOpt", "spray_profile", move_profile);
    profiles->addProfile("TrajOpt", "spray_profile", comp_profile);

    TrajOptMotionPlanner planner("spray_planner");
    json output_json = json::array();
    Eigen::VectorXd current_segment_start_joints = current_joints;

    
    // Flatten all target matrices while preserving Noether segment boundaries. A transition
    // between raster strokes or between the two leg meshes must be a non-spray motion.
    std::vector<Eigen::Isometry3d> flat_target_matrices;
    std::vector<bool> flat_segment_starts;

    for (size_t mesh_idx = 0; mesh_idx < all_target_matrices.size(); ++mesh_idx) {
        const auto& mesh_matrices = all_target_matrices[mesh_idx];
        flat_target_matrices.insert(flat_target_matrices.end(), mesh_matrices.begin(), mesh_matrices.end());
        const auto& segment_starts = all_segment_starts[mesh_idx];
        flat_segment_starts.insert(flat_segment_starts.end(), segment_starts.begin(), segment_starts.end());
    }

    // Surface rows are ordered before the standoff is applied. Different surface
    // normals can shift the TCP laterally enough to make the visible TCP rows
    // appear as 2→4→3. Reorder the final TCP strokes by their own image-right
    // coordinate so both the simulated robot and rendered TCP path are left→right.
    if (!image_horizontal_str.empty()) {
        const Eigen::Vector3d image_right = parseVector3d(image_horizontal_str).normalized();
        std::vector<std::pair<size_t, size_t>> stroke_ranges;
        for (size_t start = 0; start < flat_target_matrices.size();) {
            size_t end = start + 1;
            while (end < flat_target_matrices.size() && !flat_segment_starts[end]) {
                ++end;
            }
            stroke_ranges.emplace_back(start, end);
            start = end;
        }
        std::stable_sort(
            stroke_ranges.begin(),
            stroke_ranges.end(),
            [&flat_target_matrices, &image_right](const auto& lhs, const auto& rhs) {
                Eigen::Vector3d lhs_mean = Eigen::Vector3d::Zero();
                Eigen::Vector3d rhs_mean = Eigen::Vector3d::Zero();
                for (size_t i = lhs.first; i < lhs.second; ++i)
                    lhs_mean += flat_target_matrices[i].translation();
                for (size_t i = rhs.first; i < rhs.second; ++i)
                    rhs_mean += flat_target_matrices[i].translation();
                lhs_mean /= static_cast<double>(lhs.second - lhs.first);
                rhs_mean /= static_cast<double>(rhs.second - rhs.first);
                return lhs_mean.dot(image_right) < rhs_mean.dot(image_right);
            });

        std::vector<Eigen::Isometry3d> ordered_targets;
        std::vector<bool> ordered_segment_starts;
        json ordered_surface_json = json::array();
        ordered_targets.reserve(flat_target_matrices.size());
        ordered_segment_starts.reserve(flat_segment_starts.size());
        for (const auto& [start, end] : stroke_ranges) {
            for (size_t i = start; i < end; ++i) {
                ordered_targets.push_back(flat_target_matrices[i]);
                const bool is_stroke_start = i == start;
                ordered_segment_starts.push_back(is_stroke_start);
                json surface_point = surface_json.at(i);
                surface_point["segment_start"] = is_stroke_start;
                ordered_surface_json.push_back(surface_point);
            }
        }
        flat_target_matrices = std::move(ordered_targets);
        flat_segment_starts = std::move(ordered_segment_starts);
        surface_json = std::move(ordered_surface_json);
    }

    auto kinematic_group = env->getKinematicGroup(group_name, "KDLInvKinChainNR_JL");
    if (kinematic_group == nullptr) {
        std::cerr << "Failed to create KDL kinematic group for " << group_name << std::endl;
        return 1;
    }
    
    // Automatically retrieve joint names from the URDF kinematic group
    const std::vector<std::string> joint_names = kinematic_group->getJointNames();

    // Pre-solve every surface waypoint with KDL. Each solved joint state becomes
    // the seed for the next point, preventing numerical IK from hopping between
    // unrelated branches along a longitudinal spray stroke. The process allows
    // rotation around the tool Z axis because it does not change the spray
    // direction and the Cartesian profile already permits free yaw.
    std::vector<Eigen::Isometry3d> reachable_target_matrices;
    std::vector<bool> reachable_segment_starts;
    std::vector<Eigen::VectorXd> joint_seeds;
    json reachable_surface_json = json::array();
    reachable_target_matrices.reserve(flat_target_matrices.size());
    reachable_segment_starts.reserve(flat_segment_starts.size());
    joint_seeds.reserve(flat_target_matrices.size());
    Eigen::VectorXd ik_seed = current_joints;
    const Eigen::VectorXd legacy_seed =
        (Eigen::VectorXd(6) << 0.0, 0.0, -M_PI / 2.0, M_PI / 2.0, M_PI / 2.0, 0.0).finished();
    const Eigen::VectorXd zero_seed = Eigen::VectorXd::Zero(6);
    const std::vector<double> tool_roll_candidates = {
        0.0, M_PI / 6.0, -M_PI / 6.0, M_PI / 3.0, -M_PI / 3.0,
        M_PI / 2.0, -M_PI / 2.0, M_PI, -M_PI
    };
    bool start_new_stroke = false;
    std::size_t skipped_waypoints = 0;

    for (size_t i = 0; i < flat_target_matrices.size(); ++i) {
        const Eigen::Isometry3d nominal_pose = flat_target_matrices[i];
        const bool is_new_stroke = flat_segment_starts[i] || start_new_stroke;
        const std::vector<Eigen::VectorXd> seed_candidates = {
            ik_seed, current_joints, legacy_seed, zero_seed
        };
        bool solved = false;
        double best_cost = std::numeric_limits<double>::infinity();
        Eigen::VectorXd best_solution;
        Eigen::Isometry3d best_pose = Eigen::Isometry3d::Identity();

        // Consider every equivalent roll about the nozzle axis, then select
        // the IK branch closest to the previous joint state. This maintains
        // continuity without rejecting reachable points near a singularity.
        for (double tool_roll : tool_roll_candidates) {
            Eigen::Isometry3d candidate_pose = nominal_pose;
            candidate_pose.rotate(Eigen::AngleAxisd(tool_roll, Eigen::Vector3d::UnitZ()));
            tesseract::kinematics::KinGroupIKInput ik_input(
                candidate_pose, manip_info.working_frame, manip_info.tcp_frame
            );

            for (const Eigen::VectorXd& candidate_seed : seed_candidates) {
                const auto ik_solutions = kinematic_group->calcInvKin(ik_input, candidate_seed);
                if (ik_solutions.empty()) {
                    continue;
                }

                for (const Eigen::VectorXd& solution : ik_solutions) {
                    const double cost = (solution - ik_seed).squaredNorm() + 1e-4 * tool_roll * tool_roll;
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_solution = solution;
                        best_pose = candidate_pose;
                        solved = true;
                    }
                }
            }
        }

        if (solved) {
            ik_seed = best_solution;
            joint_seeds.push_back(ik_seed);
            reachable_target_matrices.push_back(best_pose);
            reachable_segment_starts.push_back(is_new_stroke);
            json surface_point = surface_json.at(i);
            surface_point["segment_start"] = is_new_stroke;
            reachable_surface_json.push_back(surface_point);
            start_new_stroke = false;
        }

        if (!solved) {
            std::cerr << "KDL failed to find an IK solution for surface waypoint " << i
                      << "; omitting it and starting a new stroke at the next reachable point." << std::endl;
            start_new_stroke = true;
            ++skipped_waypoints;
        }
    }
    flat_target_matrices = std::move(reachable_target_matrices);
    flat_segment_starts = std::move(reachable_segment_starts);
    surface_json = std::move(reachable_surface_json);
    if (flat_target_matrices.empty()) {
        std::cerr << "No reachable surface waypoints remain after KDL filtering." << std::endl;
        return 1;
    }

    // Replace the visualization data with the same executable subset so skipped
    // points are visibly represented as path breaks rather than false spray lines.
    std::ofstream reachable_surface_file(path_surface_file);
    reachable_surface_file << surface_json.dump(4);
    reachable_surface_file.close();
    std::cout << "KDL pre-solved " << joint_seeds.size() << " surface waypoints"
              << " and skipped " << skipped_waypoints << " unreachable points." << std::endl;

    if (kdl_only) {
        json kdl_output = json::array();
        for (size_t i = 0; i < joint_seeds.size(); ++i) {
            const Eigen::Isometry3d& pose = flat_target_matrices[i];
            const Eigen::Quaterniond quat(pose.rotation());

            json joint_positions = json::array();
            for (int joint_idx = 0; joint_idx < joint_seeds[i].size(); ++joint_idx) {
                double joint_value = joint_seeds[i][joint_idx];
                if (angle_unit == "deg") {
                    joint_value = joint_value * 180.0 / M_PI;
                }
                joint_positions.push_back(joint_value);
            }

            json point;
            point["joint_positions"] = joint_positions;
            point["x"] = pose.translation().x();
            point["y"] = pose.translation().y();
            point["z"] = pose.translation().z();
            point["qx"] = quat.x();
            point["qy"] = quat.y();
            point["qz"] = quat.z();
            point["qw"] = quat.w();
            point["segment_start"] = flat_segment_starts[i];
            point["motion_type"] = flat_segment_starts[i] ? "FREESPACE" : "LINEAR";
            point["ik_solver"] = "KDLInvKinChainNR_JL";
            point["collision_checked"] = false;
            point["time_from_start"] = 0.0;
            kdl_output.push_back(point);
        }

        std::ofstream out_file(trajectory_file);
        out_file << kdl_output.dump(4);
        out_file.close();
        std::cout << "Saved " << kdl_output.size() << " KDL-only simulation waypoints to "
                  << trajectory_file << " (collision checking and TrajOpt were skipped)." << std::endl;
        return 0;
    }

    bool planning_failed = false;
    size_t stroke_start = 0;
    size_t stroke_index = 0;
    while (stroke_start < flat_target_matrices.size()) {
        size_t stroke_end = stroke_start + 1;
        while (stroke_end < flat_target_matrices.size() && !flat_segment_starts[stroke_end]) {
            ++stroke_end;
        }
        std::cout << "Planning spray stroke " << (stroke_index + 1) << ": "
                  << stroke_start << " to " << stroke_end << " / " << flat_target_matrices.size() << std::endl;

        CompositeInstruction program("spray_profile", manip_info, CompositeInstructionOrder::ORDERED);
        StateWaypointPoly start_waypoint{StateWaypoint(joint_names, current_segment_start_joints)};
        MoveInstruction start_move(start_waypoint, MoveInstructionType::FREESPACE, "spray_profile", manip_info);
        program.push_back(start_move);

        for (size_t j = stroke_start; j < stroke_end; ++j) {
            const auto& pose = flat_target_matrices[j];
            CartesianWaypoint cartesian_waypoint(pose);
            tesseract::common::JointState waypoint_seed;
            waypoint_seed.joint_names = joint_names;
            waypoint_seed.position = joint_seeds[j];
            cartesian_waypoint.setSeed(waypoint_seed);
            CartesianWaypointPoly cart_waypoint{cartesian_waypoint};

            const MoveInstructionType move_type =
                (j == stroke_start) ? MoveInstructionType::FREESPACE : MoveInstructionType::LINEAR;
            MoveInstruction move(cart_waypoint, move_type, "spray_profile", manip_info);
            program.push_back(move);
        }

        PlannerRequest request;
        request.env = env;
        request.instructions = program;
        request.profiles = profiles;

        PlannerResponse response = planner.solve(request);
        if (!response.successful) {
            std::cerr << "Segment planning failed or hit iteration limit: " << response.message << std::endl;
            planning_failed = true;
            break;
        }

        try {
            auto flatten_results = response.results.flatten(&tesseract::command_language::moveFilter);
            bool is_first_point_in_segment = true;
            
            for (const auto& instruction : flatten_results) {
                auto move = instruction.get().template as<tesseract::command_language::MoveInstructionPoly>();
                if (move.getWaypoint().isStateWaypoint()) {
                    auto state = move.getWaypoint().template as<tesseract::command_language::StateWaypointPoly>();
                    
                    json wp_json = json::array();
                    for (int k = 0; k < state.getPosition().size(); ++k) {
                        double joint_val = state.getPosition()[k];
                        if (angle_unit == "deg") {
                            joint_val = joint_val * 180.0 / M_PI;
                        }
                        wp_json.push_back(joint_val);
                    }
                    
                    if (is_first_point_in_segment && stroke_index != 0) {
                        is_first_point_in_segment = false;
                        continue;
                    }
                    is_first_point_in_segment = false;
                    
                    json point;
                    point["joint_positions"] = wp_json;
                    point["time_from_start"] = 0.0;
                    output_json.push_back(point);
                    
                    current_segment_start_joints = state.getPosition();
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to parse results: " << e.what() << std::endl;
        }

        stroke_start = stroke_end;
        ++stroke_index;
    }
    
    if (planning_failed) {
        std::cerr << "Trajectory was not written because at least one planning segment is infeasible." << std::endl;
        return 1;
    }

    if (output_json.size() > 0) {
        std::ofstream out_file(trajectory_file);
        out_file << output_json.dump(4);
        out_file.close();
        std::cout << "Saved complete segmented trajectory to " << trajectory_file << " with " << output_json.size() << " points." << std::endl;
    } else {
        std::cerr << "Output JSON is empty!" << std::endl;
    }

    return 0;
}
