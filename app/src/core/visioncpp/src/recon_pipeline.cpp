#include "visioncpp/recon_pipeline.hpp"

#include "visioncpp/calibration.hpp"
#include "visioncpp/depth_map.hpp"
#include "visioncpp/kdtree.hpp"
#include "visioncpp/mask_set.hpp"
#include "visioncpp/point_cloud.hpp"
#include "visioncpp/poisson_recon.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

namespace visioncpp {
namespace {

cv::Mat erodeMask(const cv::Mat& mask_u8, int erode_px) {
    if (erode_px <= 0) return mask_u8.clone();
    const int k = erode_px * 2 + 1;
    cv::Mat out;
    cv::erode(mask_u8, out, cv::Mat::ones(k, k, CV_8U));
    return out;
}

cv::Mat flyingPixelValid(const cv::Mat& depth_f32, double max_grad) {
    cv::Mat gx, gy, mag;
    cv::Sobel(depth_f32, gx, CV_32F, 1, 0, 3);
    cv::Sobel(depth_f32, gy, CV_32F, 0, 1, 3);
    cv::magnitude(gx, gy, mag);
    cv::Mat edge = mag > static_cast<float>(max_grad);
    cv::dilate(edge, edge, cv::Mat::ones(3, 3, CV_8U));
    return edge == 0;
}

CameraIntrinsics resolveK(const std::string& template_dir, const Calibration& calib, int w, int h) {
    CameraIntrinsics k;
    const std::string params = template_dir + "/scan.params.yaml";
    if (std::filesystem::exists(params)) {
        try {
            k = Calibration::loadScanParams(params);
        } catch (...) {
            k = {};
        }
    }
    if (!k.valid() && calib.hasK()) {
        const Mat3 m = calib.K();
        k.fx = m(0, 0);
        k.fy = m(1, 1);
        k.cx = m(0, 2);
        k.cy = m(1, 2);
        k.width = w;
        k.height = h;
    }
    if (!k.valid()) {
        throw VisionError("camera K missing (scan.params.yaml and calib); refuse default K");
    }
    if (k.width <= 0) k.width = w;
    if (k.height <= 0) k.height = h;
    return k;
}

}  // namespace

ReconResult ReconPipeline::run(const std::string& template_dir,
                               const std::string& calib_path,
                               const std::string& output_dir,
                               const ReconOptions& opt) {
    const auto t0 = std::chrono::steady_clock::now();
    if (!std::filesystem::exists(template_dir)) {
        throw VisionError("template dir not found: " + template_dir);
    }
    const std::string masks_path = template_dir + "/scan.masks.yaml";
    if (!std::filesystem::exists(masks_path)) {
        throw VisionError("scan.masks.yaml not found");
    }

    DepthMap depth = DepthMap::loadFromTemplate(template_dir);
    const Calibration calib = Calibration::load(calib_path);
    const CameraIntrinsics k = resolveK(template_dir, calib, depth.cols(), depth.rows());
    const MaskSet masks = MaskSet::loadYaml(masks_path, depth.rows(), depth.cols());

    cv::Mat valid = (depth.mat() > static_cast<float>(opt.z_min_mm))
                    & (depth.mat() < static_cast<float>(opt.z_max_mm));
    cv::Mat holes = masks.mask() & ~valid;
    depth.inpaintHoles(holes);
    valid = (depth.mat() > static_cast<float>(opt.z_min_mm))
            & (depth.mat() < static_cast<float>(opt.z_max_mm));

    cv::Mat eroded = erodeMask(masks.mask(), opt.mask_erode_px);
    cv::Mat flying = opt.flying_pixel_max_grad > 0 ? flyingPixelValid(depth.mat(), opt.flying_pixel_max_grad)
                                                   : cv::Mat::ones(depth.mat().size(), CV_8U);
    cv::Mat combined = eroded & valid & flying;

    auto tick = std::chrono::steady_clock::now();
    auto mark = [&](const char* name) {
        const auto now = std::chrono::steady_clock::now();
        std::cerr << "recon: " << name << " "
                  << std::chrono::duration<double, std::milli>(now - tick).count() << " ms\n";
        tick = now;
    };

    PointCloud cloud = PointCloud::fromDepth(depth, k, combined);
    cloud.removeNonFinite();
    {
        std::vector<Vec3> kept;
        kept.reserve(cloud.points.size());
        for (const auto& p : cloud.points) {
            if (p.cwiseAbs().maxCoeff() < 20.0) kept.push_back(p);
        }
        cloud.points.swap(kept);
        cloud.normals.clear();
    }
    std::cerr << "recon: raw_pts=" << cloud.points.size()
              << " mask_nz=" << cv::countNonZero(combined) << "\n";
    cloud.voxelDownsample(opt.voxel_size);
    cloud.removeNonFinite();
    std::cerr << "recon: after_voxel=" << cloud.points.size() << "\n";
    mark("unproject+voxel");
    cloud.removeStatisticalOutliers(20, 2.0);
    cloud.removeNonFinite();
    {
        Vec3 bmin, bmax;
        size_t finite = 0;
        cloud.bounds(bmin, bmax, &finite);
        std::cerr << "recon: after_outlier=" << cloud.points.size()
                  << " finite=" << finite
                  << " bbox=[" << bmin.transpose() << "] [" << bmax.transpose() << "]\n";
    }
    mark("outlier");
    cloud.estimateNormals(opt.normal_radius, 30);
    cloud.orientNormalsTowardsOrigin();
    cloud.removeNonFinite();
    std::cerr << "recon: normals=" << cloud.normals.size() << "\n";
    mark("normals");

    Mesh mesh;
    try {
        mesh = PoissonRecon::reconstruct(cloud, opt.poisson_depth, opt.density_threshold);
        std::cerr << "recon: kernel=kazhdan verts=" << mesh.vertexCount()
                  << " faces=" << mesh.faceCount() << "\n";
    } catch (const VisionError& e) {
        std::cerr << "recon: kazhdan failed (" << e.what() << "), hoppe fallback\n";
        mesh = hoppeReconstruct(cloud, std::max(opt.voxel_size, 0.004), opt.density_threshold);
        std::cerr << "recon: kernel=hoppe verts=" << mesh.vertexCount()
                  << " faces=" << mesh.faceCount() << "\n";
    }
    {
        const double max_dist = std::max(3.0 * opt.voxel_size, 0.012);
        KdTree tree;
        tree.build(cloud.points);
        std::vector<char> drop(mesh.vertices.size(), 0);
        for (size_t i = 0; i < mesh.vertices.size(); ++i) {
            const int j = tree.nearest(mesh.vertices[i]);
            if (j < 0 || (mesh.vertices[i] - cloud.points[j]).norm() > max_dist) drop[i] = 1;
        }
        mesh.removeVerticesByMask(drop);
        std::cerr << "recon: after cloud-trim verts=" << mesh.vertexCount()
                  << " faces=" << mesh.faceCount() << " max_dist=" << max_dist << "\n";
    }
    mark("poisson+trim");
    if (opt.smooth_iterations > 0) mesh.taubinSmooth(opt.smooth_iterations);
    mesh.transform(calib.T_camera_to_base());
    mesh.keepLargestComponent();
    mark("smooth+export-prep");
    std::cerr << "recon: final verts=" << mesh.vertexCount() << " faces=" << mesh.faceCount() << "\n";
    if (mesh.vertexCount() < 10 || mesh.faceCount() < 1) {
        throw VisionError("reconstruction produced an empty mesh");
    }

    std::filesystem::create_directories(output_dir);
    const std::string ply = output_dir + "/scan.mesh.ply";
    const std::string stl = output_dir + "/scan.mesh.stl";
    mesh.savePly(ply);
    mesh.saveStl(stl);

    const auto t1 = std::chrono::steady_clock::now();
    ReconResult r;
    r.vertices = mesh.vertexCount();
    r.faces = mesh.faceCount();
    r.elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.files = {"scan.mesh.ply", "scan.mesh.stl"};
    return r;
}

}  // namespace visioncpp
