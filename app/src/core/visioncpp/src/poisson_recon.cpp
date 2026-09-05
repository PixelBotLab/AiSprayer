#include "visioncpp/poisson_recon.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif

#include "PreProcessor.h"
#include "Reconstructors.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace visioncpp {
namespace {

using namespace PoissonRecon;

template <typename Real, unsigned int Dim>
struct CloudStream : public Reconstructor::InputOrientedSampleStream<Real, Dim> {
    explicit CloudStream(const PointCloud& cloud) {
        data_.reserve(cloud.points.size());
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            Sample s;
            for (unsigned int d = 0; d < Dim; ++d) {
                s.p[d] = static_cast<Real>(cloud.points[i][d]);
                s.n[d] = static_cast<Real>(i < cloud.normals.size() ? cloud.normals[i][d] : 0);
            }
            const Real n2 = s.n[0] * s.n[0] + s.n[1] * s.n[1] + s.n[2] * s.n[2];
            if (n2 < static_cast<Real>(1e-12) || !std::isfinite(n2)) {
                s.n[0] = 0;
                s.n[1] = 0;
                s.n[2] = 1;
            }
            data_.push_back(s);
        }
        std::cerr << "poisson: stream samples=" << data_.size() << "\n";
    }
    void reset(void) override { i_ = 0; }
    bool read(Point<Real, Dim>& p, Point<Real, Dim>& n) override {
        if (i_ >= data_.size()) return false;
        p = data_[i_].p;
        n = data_[i_].n;
        ++i_;
        return true;
    }

private:
    struct Sample {
        Point<Real, Dim> p, n;
    };
    std::vector<Sample> data_;
    size_t i_ = 0;
};

template <typename Real, unsigned int Dim>
struct VertStream : public Reconstructor::OutputLevelSetVertexStream<Real, Dim> {
    explicit VertStream(Mesh& mesh) : mesh_(mesh) {}
    size_t size() const override { return mesh_.vertices.size(); }
    size_t write(const Point<Real, Dim>& p, const Point<Real, Dim>&, const Real& density) override {
        mesh_.vertices.emplace_back(p[0], p[1], p[2]);
        mesh_.densities.push_back(static_cast<double>(density));
        return mesh_.vertices.size() - 1;
    }

private:
    Mesh& mesh_;
};

struct FaceStream : public Reconstructor::OutputFaceStream<2> {
    explicit FaceStream(Mesh& mesh) : mesh_(mesh) {}
    size_t size() const override { return mesh_.faces.size(); }
    size_t write(const std::vector<node_index_type>& poly) override {
        if (poly.size() < 3) return mesh_.faces.size();
        for (size_t i = 1; i + 1 < poly.size(); ++i) {
            mesh_.faces.emplace_back(static_cast<int>(poly[0]), static_cast<int>(poly[i]),
                                     static_cast<int>(poly[i + 1]));
        }
        return mesh_.faces.size() - 1;
    }

private:
    Mesh& mesh_;
};

double quantile(std::vector<double> v, double q) {
    if (v.empty()) return 0;
    q = std::clamp(q, 0.0, 1.0);
    const size_t i = static_cast<size_t>(std::clamp(q * (v.size() - 1), 0.0, static_cast<double>(v.size() - 1)));
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(i), v.end());
    return v[i];
}

}  // namespace

Mesh PoissonRecon::reconstruct(const PointCloud& cloud, int depth, double density_threshold) {
    if (cloud.points.size() < 50) throw VisionError("too few points for poisson");
    if (cloud.normals.size() != cloud.points.size()) {
        throw VisionError("poisson requires oriented points");
    }

    // CG at depth=8 is faster serial on RK3588; ASYNC made the solve ~2x slower.
    ThreadPool::ParallelizationType = ThreadPool::ParallelType::NONE;

    using ReconType = Reconstructor::Poisson;
    constexpr unsigned int Dim = 3;
    using Real = float;
    constexpr unsigned int FEMSig =
        FEMDegreeAndBType<ReconType::DefaultFEMDegree, ReconType::DefaultFEMBoundary>::Signature;
    using FEMSigs = IsotropicUIntPack<Dim, FEMSig>;
    using Implicit = Reconstructor::Implicit<Real, Dim, FEMSigs>;
    using Solver = ReconType::Solver<Real, Dim, FEMSigs>;

    ReconType::SolutionParameters<Real> params;
    params.verbose = true;
    params.depth = static_cast<unsigned int>(std::max(1, depth));
    // Keep library defaults: fullDepth=5, base/solve/kernel = -1 (adaptive).
    // Forcing fullDepth==depth builds a complete 256^3 tree (~16M nodes, ~150s).

    CloudStream<Real, Dim> samples(cloud);
    Implicit* implicit = Solver::Solve(samples, params);
    if (!implicit) throw VisionError("Kazhdan Poisson Solve returned null");

    Mesh mesh;
    VertStream<Real, Dim> vstream(mesh);
    FaceStream fstream(mesh);
    Reconstructor::LevelSetExtractionParameters extract;
    extract.linearFit = false;
    extract.outputDensity = true;
    extract.forceManifold = true;
    extract.polygonMesh = false;
    extract.verbose = false;
    implicit->extractLevelSet(vstream, fstream, extract);
    std::cerr << "poisson: extracted verts=" << mesh.vertices.size()
              << " faces=" << mesh.faces.size()
              << " iso=" << implicit->isoValue << "\n";
    delete implicit;

    if (mesh.vertices.empty() || mesh.faces.empty()) {
        throw VisionError("poisson produced an empty mesh");
    }
    if (!mesh.densities.empty() && density_threshold > 0) {
        const double cut = quantile(mesh.densities, density_threshold);
        std::vector<char> drop(mesh.vertices.size(), 0);
        for (size_t i = 0; i < mesh.densities.size(); ++i) {
            if (mesh.densities[i] < cut) drop[i] = 1;
        }
        mesh.removeVerticesByMask(drop);
    }
    return mesh;
}

}  // namespace visioncpp
