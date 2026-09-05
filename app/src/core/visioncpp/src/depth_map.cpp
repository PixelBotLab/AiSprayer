#include "visioncpp/depth_map.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/photo.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>

namespace visioncpp {
namespace {

cv::Mat loadNpy2d(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw VisionError("cannot open npy: " + path);
    char magic[6];
    in.read(magic, 6);
    if (!in || std::memcmp(magic, "\x93NUMPY", 6) != 0) {
        throw VisionError("invalid npy magic: " + path);
    }
    uint8_t major = 0, minor = 0;
    in.read(reinterpret_cast<char*>(&major), 1);
    in.read(reinterpret_cast<char*>(&minor), 1);
    uint32_t header_len = 0;
    if (major == 1) {
        uint16_t len16 = 0;
        in.read(reinterpret_cast<char*>(&len16), 2);
        header_len = len16;
    } else {
        in.read(reinterpret_cast<char*>(&header_len), 4);
    }
    std::string header(header_len, '\0');
    in.read(header.data(), header_len);

    const bool le = header.find("'<") != std::string::npos || header.find("\"<") != std::string::npos
                    || header.find("'<") != std::string::npos || header.find("<i") != std::string::npos
                    || header.find("<u") != std::string::npos || header.find("<f") != std::string::npos;
    (void)le;
    const bool is_u16 = header.find("'<u2'") != std::string::npos || header.find("\"<u2\"") != std::string::npos
                        || header.find("|u2") != std::string::npos || header.find("<u2") != std::string::npos;
    const bool is_f32 = header.find("'<f4'") != std::string::npos || header.find("\"<f4\"") != std::string::npos
                        || header.find("<f4") != std::string::npos;
    const bool is_f64 = header.find("'<f8'") != std::string::npos || header.find("\"<f8\"") != std::string::npos
                        || header.find("<f8") != std::string::npos;

    auto shape_pos = header.find("shape");
    if (shape_pos == std::string::npos) throw VisionError("npy missing shape: " + path);
    auto paren = header.find('(', shape_pos);
    auto close = header.find(')', paren);
    std::string shape = header.substr(paren + 1, close - paren - 1);
    int rows = 0, cols = 0;
    const auto comma = shape.find(',');
    rows = std::stoi(shape.substr(0, comma));
    std::string rest = shape.substr(comma + 1);
    while (!rest.empty() && (rest[0] == ' ' || rest[0] == ',')) rest.erase(0, 1);
    cols = rest.empty() ? 1 : std::stoi(rest);
    if (rows <= 0 || cols <= 0) throw VisionError("npy bad shape: " + path);

    cv::Mat out(rows, cols, CV_32F);
    if (is_u16) {
        std::vector<uint16_t> buf(static_cast<size_t>(rows) * cols);
        in.read(reinterpret_cast<char*>(buf.data()), buf.size() * 2);
        for (size_t i = 0; i < buf.size(); ++i) out.at<float>(static_cast<int>(i)) = static_cast<float>(buf[i]);
    } else if (is_f32) {
        in.read(reinterpret_cast<char*>(out.data), static_cast<size_t>(rows) * cols * 4);
    } else if (is_f64) {
        std::vector<double> buf(static_cast<size_t>(rows) * cols);
        in.read(reinterpret_cast<char*>(buf.data()), buf.size() * 8);
        for (size_t i = 0; i < buf.size(); ++i) out.at<float>(static_cast<int>(i)) = static_cast<float>(buf[i]);
    } else {
        throw VisionError("unsupported npy dtype (need u16/f32/f64): " + path);
    }
    return out;
}

}  // namespace

DepthMap DepthMap::loadFromTemplate(const std::string& template_dir) {
    DepthMap dm;
    const std::string png = template_dir + "/scan.depth.png";
    const std::string npy = template_dir + "/scan.depth.npy";
    cv::Mat raw = cv::imread(png, cv::IMREAD_UNCHANGED);
    if (!raw.empty()) {
        if (raw.type() == CV_16UC1) {
            raw.convertTo(dm.depth_, CV_32F);
        } else if (raw.type() == CV_32FC1) {
            dm.depth_ = raw;
        } else {
            throw VisionError("scan.depth.png must be 16UC1 or 32FC1");
        }
        return dm;
    }
    std::ifstream probe(npy, std::ios::binary);
    if (!probe) {
        throw VisionError("missing scan.depth.png / scan.depth.npy in " + template_dir);
    }
    probe.close();
    dm.depth_ = loadNpy2d(npy);
    return dm;
}

void DepthMap::inpaintHoles(const cv::Mat& hole_mask_u8) {
    if (hole_mask_u8.empty() || cv::countNonZero(hole_mask_u8) == 0) return;
    cv::Mat mask;
    cv::threshold(hole_mask_u8, mask, 0, 255, cv::THRESH_BINARY);
    cv::Mat src16, filled16;
    depth_.convertTo(src16, CV_16U);
    cv::inpaint(src16, mask, filled16, 5.0, cv::INPAINT_NS);
    cv::Mat filled32;
    filled16.convertTo(filled32, CV_32F);
    filled32.copyTo(depth_, mask);
}

}  // namespace visioncpp
