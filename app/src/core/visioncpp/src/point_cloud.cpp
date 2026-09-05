#include "visioncpp/point_cloud.hpp"

#include "visioncpp/kdtree.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <thread>
#include <unordered_map>

namespace visioncpp {
namespace {

struct VoxelKey {
    int64_t x, y, z;
    bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelHash {
    size_t operator()(const VoxelKey& k) const {
        return (static_cast<size_t>(k.x) * 73856093) ^ (static_cast<size_t>(k.y) * 19349663)
               ^ (static_cast<size_t>(k.z) * 83492791);
    }
};

unsigned workerCount() {
    const unsigned hc = std::thread::hardware_concurrency();
    if (!hc) return 1;
    return std::min(hc, 4u);
}

template <typename Fn>
void parallelFor(size_t n, Fn&& fn) {
    const unsigned w = workerCount();
    if (n < 512 || w <= 1) {
        for (size_t i = 0; i < n; ++i) fn(i);
        return;
    }
    std::vector<std::thread> ts;
    ts.reserve(w);
    const size_t chunk = (n + w - 1) / w;
    for (unsigned t = 0; t < w; ++t) {
        const size_t a = static_cast<size_t>(t) * chunk;
        const size_t b = std::min(n, a + chunk);
        if (a >= b) break;
        ts.emplace_back([a, b, &fn]() {
            for (size_t i = a; i < b; ++i) fn(i);
        });
    }
    for (auto& th : ts) th.join();
}

}  // namespace

PointCloud PointCloud::fromDepth(const DepthMap& depth, const CameraIntrinsics& k, const cv::Mat& mask_u8) {
    if (mask_u8.size() != depth.mat().size()) {
        throw VisionError("mask/depth size mismatch");
    }
    PointCloud cloud;
    cloud.points.reserve(static_cast<size_t>(cv::countNonZero(mask_u8)));
    const cv::Mat& d = depth.mat();
    for (int r = 0; r < d.rows; ++r) {
        const float* drow = d.ptr<float>(r);
        const uint8_t* mrow = mask_u8.ptr<uint8_t>(r);
        for (int c = 0; c < d.cols; ++c) {
            if (!mrow[c]) continue;
            const double z = drow[c];
            if (!std::isfinite(z) || z <= 0) continue;
            const double x = (c - k.cx) * z / k.fx;
            const double y = (r - k.cy) * z / k.fy;
            if (!std::isfinite(x) || !std::isfinite(y)) continue;
            cloud.points.emplace_back(x / 1000.0, y / 1000.0, z / 1000.0);
        }
    }
    if (cloud.points.size() < 50) {
        throw VisionError("masked point count too small for reconstruction");
    }
    return cloud;
}

void PointCloud::voxelDownsample(double voxel_size) {
    if (voxel_size <= 0 || points.empty()) return;
    std::unordered_map<VoxelKey, std::pair<Vec3, int>, VoxelHash> acc;
    acc.reserve(points.size());
    const double inv = 1.0 / voxel_size;
    for (const auto& p : points) {
        if (!p.allFinite()) continue;
        VoxelKey key{static_cast<int64_t>(std::floor(p.x() * inv)),
                     static_cast<int64_t>(std::floor(p.y() * inv)),
                     static_cast<int64_t>(std::floor(p.z() * inv))};
        auto& e = acc[key];
        if (e.second == 0) e.first = Vec3::Zero();
        e.first += p;
        e.second += 1;
    }
    std::vector<Vec3> out;
    out.reserve(acc.size());
    for (const auto& kv : acc) out.push_back(kv.second.first / static_cast<double>(kv.second.second));
    points.swap(out);
    normals.clear();
}

void PointCloud::removeNonFinite() {
    std::vector<Vec3> kept_p;
    std::vector<Vec3> kept_n;
    kept_p.reserve(points.size());
    const bool has_n = normals.size() == points.size();
    if (has_n) kept_n.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        if (!points[i].allFinite()) continue;
        if (has_n && !normals[i].allFinite()) continue;
        kept_p.push_back(points[i]);
        if (has_n) kept_n.push_back(normals[i]);
    }
    points.swap(kept_p);
    if (has_n) normals.swap(kept_n);
    else normals.clear();
}

void PointCloud::bounds(Vec3& bmin, Vec3& bmax, size_t* finite_count) const {
    bmin = Vec3::Constant(std::numeric_limits<double>::infinity());
    bmax = Vec3::Constant(-std::numeric_limits<double>::infinity());
    size_t n = 0;
    for (const auto& p : points) {
        if (!p.allFinite()) continue;
        bmin = bmin.cwiseMin(p);
        bmax = bmax.cwiseMax(p);
        ++n;
    }
    if (finite_count) *finite_count = n;
}

void PointCloud::removeStatisticalOutliers(int nb_neighbors, double std_ratio) {
    if (points.size() < static_cast<size_t>(nb_neighbors + 1)) return;
    KdTree tree;
    tree.build(points);
    std::vector<double> mean_d(points.size(), 0);
    parallelFor(points.size(), [&](size_t i) {
        std::vector<int> nn;
        std::vector<double> d2;
        tree.knn(points[i], nb_neighbors + 1, nn, &d2);
        double s = 0;
        int c = 0;
        for (size_t j = 0; j < nn.size(); ++j) {
            if (nn[j] == static_cast<int>(i) || !std::isfinite(d2[j]) || d2[j] < 0) continue;
            s += std::sqrt(d2[j]);
            ++c;
        }
        mean_d[i] = c ? s / c : 0;
    });
    int finite = 0;
    double mu = 0;
    for (double v : mean_d) {
        if (std::isfinite(v)) {
            mu += v;
            ++finite;
        }
    }
    if (finite < 10) return;
    mu /= static_cast<double>(finite);
    double var = 0;
    for (double v : mean_d) {
        if (!std::isfinite(v)) continue;
        var += (v - mu) * (v - mu);
    }
    var /= static_cast<double>(finite);
    const double sigma = std::sqrt(var);
    const double thr = mu + std_ratio * sigma;
    std::vector<Vec3> kept;
    kept.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        if (std::isfinite(mean_d[i]) && mean_d[i] <= thr) kept.push_back(points[i]);
    }
    if (kept.size() < 50) return;
    points.swap(kept);
    normals.clear();
}

void PointCloud::estimateNormals(double radius, int max_nn) {
    normals.assign(points.size(), Vec3::UnitZ());
    if (points.size() < 3) return;
    KdTree tree;
    tree.build(points);
    parallelFor(points.size(), [&](size_t i) {
        std::vector<int> nn;
        tree.radius(points[i], radius, max_nn, nn);
        if (static_cast<int>(nn.size()) < 3) {
            tree.knn(points[i], std::min(max_nn, static_cast<int>(points.size())), nn);
        }
        if (nn.size() < 3) return;
        Vec3 mean = Vec3::Zero();
        for (int j : nn) mean += points[j];
        mean /= static_cast<double>(nn.size());
        Mat3 cov = Mat3::Zero();
        for (int j : nn) {
            const Vec3 d = points[j] - mean;
            cov += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Mat3> es(cov);
        Vec3 n = es.eigenvectors().col(0);
        if (n.norm() > 1e-15) n.normalize();
        else n = Vec3::UnitZ();
        normals[i] = n;
    });
}

void PointCloud::orientNormalsTowardsOrigin() {
    if (normals.size() != points.size()) return;
    for (size_t i = 0; i < points.size(); ++i) {
        if (normals[i].dot(points[i]) > 0) normals[i] = -normals[i];
    }
}

}  // namespace visioncpp
