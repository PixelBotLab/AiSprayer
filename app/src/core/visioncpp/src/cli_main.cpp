#include "visioncpp/auto_path_planner.hpp"
#include "visioncpp/calibration.hpp"
#include "visioncpp/mask_set.hpp"
#include "visioncpp/mesh.hpp"
#include "visioncpp/recon_pipeline.hpp"
#include "visioncpp/types.hpp"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

void usage() {
    std::cerr <<
        "Usage:\n"
        "  vision_cli recon --template-dir DIR --calib FILE [--output-dir DIR]\n"
        "                   [--poisson-depth 8] [--voxel-size 0.003]\n"
        "                   [--density-threshold 0.15] [--smooth-iterations 20]\n"
        "  vision_cli auto-path --template-dir DIR --calib FILE [--output FILE]\n"
        "                       [--spray-dist 150] [--row-spacing 40]\n"
        "                       [--point-spacing 100] [--dedup-radius 20]\n";
}

std::string arg(int argc, char** argv, const char* key, const char* def = nullptr) {
    for (int i = 0; i < argc - 1; ++i) {
        if (std::strcmp(argv[i], key) == 0) return argv[i + 1];
    }
    if (def) return def;
    return {};
}

bool hasFlag(int argc, char** argv, const char* key) {
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], key) == 0) return true;
    }
    return false;
}

double argd(int argc, char** argv, const char* key, double def) {
    const std::string s = arg(argc, argv, key);
    return s.empty() ? def : std::stod(s);
}

int argi(int argc, char** argv, const char* key, int def) {
    const std::string s = arg(argc, argv, key);
    return s.empty() ? def : std::stoi(s);
}

int fail(visioncpp::ExitCode code, const std::string& msg) {
    std::cout << "{\"status\":\"error\",\"error\":\"" << msg << "\"}\n";
    std::cerr << "error: " << msg << "\n";
    return static_cast<int>(code);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || hasFlag(argc, argv, "-h") || hasFlag(argc, argv, "--help")) {
        usage();
        return argc < 2 ? 2 : 0;
    }
    const std::string cmd = argv[1];
    try {
        if (cmd == "recon") {
            const std::string tdir = arg(argc, argv, "--template-dir");
            const std::string calib = arg(argc, argv, "--calib");
            if (tdir.empty() || calib.empty()) {
                return fail(visioncpp::ExitCode::MissingInput, "--template-dir and --calib are required");
            }
            visioncpp::ReconOptions opt;
            opt.poisson_depth = argi(argc, argv, "--poisson-depth", 8);
            opt.voxel_size = argd(argc, argv, "--voxel-size", 0.003);
            opt.density_threshold = argd(argc, argv, "--density-threshold", 0.15);
            opt.smooth_iterations = argi(argc, argv, "--smooth-iterations", 20);
            const std::string out = arg(argc, argv, "--output-dir", tdir.c_str());
            const auto r = visioncpp::ReconPipeline::run(tdir, calib, out, opt);
            std::cout << "{\"status\":\"success\",\"vertices\":" << r.vertices
                      << ",\"faces\":" << r.faces
                      << ",\"elapsed_ms\":" << static_cast<int>(r.elapsed_ms + 0.5)
                      << ",\"files\":[\"scan.mesh.ply\",\"scan.mesh.stl\"]}\n";
            return 0;
        }
        if (cmd == "auto-path") {
            const std::string tdir = arg(argc, argv, "--template-dir");
            const std::string calib_path = arg(argc, argv, "--calib");
            if (tdir.empty() || calib_path.empty()) {
                return fail(visioncpp::ExitCode::MissingInput, "--template-dir and --calib are required");
            }
            const std::string mesh_path = tdir + "/scan.mesh.ply";
            const std::string masks_path = tdir + "/scan.masks.yaml";
            const std::string params_path = tdir + "/scan.params.yaml";
            if (!std::filesystem::exists(mesh_path)) {
                return fail(visioncpp::ExitCode::MissingInput, "scan.mesh.ply not found");
            }
            if (!std::filesystem::exists(masks_path)) {
                return fail(visioncpp::ExitCode::MissingInput, "scan.masks.yaml not found");
            }
            const auto calib = visioncpp::Calibration::load(calib_path);
            visioncpp::CameraIntrinsics k;
            if (std::filesystem::exists(params_path)) {
                k = visioncpp::Calibration::loadScanParams(params_path);
            }
            if (!k.valid() && calib.hasK()) {
                const auto m = calib.K();
                k.fx = m(0, 0);
                k.fy = m(1, 1);
                k.cx = m(0, 2);
                k.cy = m(1, 2);
                k.width = 1280;
                k.height = 800;
            }
            if (!k.valid()) {
                return fail(visioncpp::ExitCode::MissingInput, "camera K missing; refuse default K");
            }
            visioncpp::AutoPathOptions opt;
            opt.spray_dist_mm = argd(argc, argv, "--spray-dist", 150);
            opt.row_spacing_mm = argd(argc, argv, "--row-spacing", 40);
            opt.point_spacing_mm = argd(argc, argv, "--point-spacing", 100);
            opt.dedup_radius_mm = argd(argc, argv, "--dedup-radius", 20);
            const auto mesh = visioncpp::Mesh::loadPly(mesh_path);
            const auto masks = visioncpp::MaskSet::loadYaml(masks_path, k.height, k.width);
            const auto t0 = std::chrono::steady_clock::now();
            const auto doc = visioncpp::AutoPathPlanner::plan(mesh, masks, k, calib.T_camera_to_base(), opt);
            const auto t1 = std::chrono::steady_clock::now();
            const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            const std::string out = arg(argc, argv, "--output", (tdir + "/scan.auto.path.yaml").c_str());
            visioncpp::AutoPathPlanner::writeYaml(out, doc);
            std::cout << "{\"status\":\"success\",\"path_count\":1,\"point_count\":" << doc.points.size()
                      << ",\"elapsed_ms\":" << static_cast<int>(ms + 0.5) << "}\n";
            return 0;
        }
        return fail(visioncpp::ExitCode::MissingInput, "unknown command (recon|auto-path)");
    } catch (const visioncpp::VisionError& e) {
        const std::string msg = e.what();
        const bool missing = msg.find("missing") != std::string::npos || msg.find("not found") != std::string::npos
                             || msg.find("refuse") != std::string::npos || msg.find("Identity") != std::string::npos;
        return fail(missing ? visioncpp::ExitCode::MissingInput : visioncpp::ExitCode::Failed, msg);
    } catch (const std::exception& e) {
        return fail(visioncpp::ExitCode::Failed, e.what());
    }
}
