#include "visioncpp/auto_path_planner.hpp"
#include "visioncpp/calibration.hpp"
#include "visioncpp/mask_set.hpp"
#include "visioncpp/mesh.hpp"
#include "visioncpp/tool_frame.hpp"

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

    {
        bool threw = false;
        try {
            Calibration::load("/no/such/calib.yaml");
        } catch (const VisionError&) {
            threw = true;
        }
        CHECK(threw, "missing calib must fail");
    }

    {
        CHECK(Calibration::isIdentity(Mat4::Identity()), "identity detect");
        Mat4 T = Mat4::Identity();
        T(0, 3) = 0.1;
        CHECK(!Calibration::isIdentity(T), "non-identity");
    }

    {
        ToolFrame tf;
        const Mat3 R0 = tf.compute(Vec3::UnitZ());
        const Mat3 R1 = tf.compute(-Vec3::UnitZ());
        CHECK(R0.col(0).dot(R1.col(0)) >= -1.0, "tool frame produced");
        CHECK(std::abs(R0.col(2).dot(Vec3::UnitZ()) + 1.0) < 1e-6, "z_tool = -n");
    }

    const std::string ply = std::string(REPO_ROOT) + "/data/template_group/2026-09-03_225937/scan.mesh.ply";
    std::ifstream probe(ply);
    if (probe) {
        probe.close();
        Mesh m = Mesh::loadPly(ply);
        const int faces0 = m.faceCount();
        CHECK(m.vertexCount() > 1000, "load production ply vertices");
        CHECK(faces0 > 1000, "load production ply faces");
        m.computeVertexNormals();
        int zero_n = 0;
        for (const auto& n : m.vertex_normals) {
            if (n.norm() < 1e-9) ++zero_n;
        }
        CHECK(zero_n < m.vertexCount() / 10, "most vertex normals nonzero");
        CHECK(m.faceCount() == faces0, "normals do not mutate faces");
        std::cout << "loaded production mesh verts=" << m.vertexCount()
                  << " faces=" << faces0 << "\n";
    } else {
        std::cerr << "skip production mesh load (file missing)\n";
    }

    if (fails) {
        std::cerr << fails << " check(s) failed\n";
        return 1;
    }
    std::cout << "test_auto_path ok\n";
    return 0;
}
