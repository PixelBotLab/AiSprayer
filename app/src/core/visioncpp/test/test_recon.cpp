#include "visioncpp/calibration.hpp"
#include "visioncpp/depth_map.hpp"
#include "visioncpp/kdtree.hpp"
#include "visioncpp/mask_set.hpp"
#include "visioncpp/mesh.hpp"
#include "visioncpp/point_cloud.hpp"
#include "visioncpp/recon_pipeline.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

#ifndef REPO_ROOT
#define REPO_ROOT "."
#endif

static int fails = 0;
#define CHECK(cond, msg)                                                 \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::cerr << "FAIL: " << msg << "\n";                        \
            ++fails;                                                     \
        }                                                                \
    } while (0)

int main() {
    using namespace visioncpp;
    const std::string tdir = std::string(REPO_ROOT) + "/data/template_group/2026-09-03_225937";
    const std::string calib = std::string(REPO_ROOT) + "/configs/calib/calibration_result.yaml";

    {
        bool threw = false;
        try {
            DepthMap::loadFromTemplate("/no/such/dir");
        } catch (const VisionError&) {
            threw = true;
        }
        CHECK(threw, "missing depth fails");
    }

    std::ifstream probe(tdir + "/scan.depth.png");
    if (!probe) {
        std::cerr << "skip template recon IO (data missing)\n";
        return fails ? 1 : 0;
    }
    probe.close();

    DepthMap depth = DepthMap::loadFromTemplate(tdir);
    CHECK(depth.rows() == 800 && depth.cols() == 1280, "depth size 1280x800");
    MaskSet masks = MaskSet::loadYaml(tdir + "/scan.masks.yaml", depth.rows(), depth.cols());
    CHECK(cv::countNonZero(masks.mask()) > 50, "mask area");
    auto legs = masks.splitLegs(0.1, 0.0);
    CHECK(legs.size() == 1 || legs.size() == 2, "leg split");

    Calibration cal = Calibration::load(calib);
    CHECK(!Calibration::isIdentity(cal.T_camera_to_base()), "calib not identity");
    CameraIntrinsics k = Calibration::loadScanParams(tdir + "/scan.params.yaml");
    CHECK(k.valid(), "K from scan.params.yaml");
    CHECK(std::abs(k.fx - 611.683837890625) < 1e-3, "fx matches template");

    cv::Mat valid = (depth.mat() > 100.f) & (depth.mat() < 3000.f);
    cv::Mat combined = masks.mask() & valid;
    PointCloud cloud = PointCloud::fromDepth(depth, k, combined);
    CHECK(cloud.points.size() > 1000, "point cloud from depth");
    cloud.voxelDownsample(0.003);
    CHECK(cloud.points.size() > 200, "voxel downsample still populated");

    Mesh ref = Mesh::loadPly(tdir + "/scan.mesh.ply");
    CHECK(ref.vertexCount() > 1000 && ref.faceCount() > 1000, "reference ply");
    KdTree tree;
    tree.build(ref.vertices);
    int hits = 0;
    for (size_t i = 0; i < std::min<size_t>(cloud.points.size(), 200); ++i) {
        const Vec3 p_base = (cal.T_camera_to_base() * cloud.points[i].homogeneous()).head<3>();
        const int j = tree.nearest(p_base);
        if (j >= 0 && (ref.vertices[j] - p_base).norm() < 0.03) ++hits;
    }
    CHECK(hits > 50, "depth points land near existing mesh (30mm)");
    std::cout << "recon IO/precheck: depth=" << depth.cols() << "x" << depth.rows()
              << " cloud=" << cloud.points.size() << " mesh=" << ref.vertexCount()
              << " near_hits=" << hits << "\n";

    if (fails) {
        std::cerr << fails << " check(s) failed\n";
        return 1;
    }
    std::cout << "test_recon ok\n";
    return 0;
}
