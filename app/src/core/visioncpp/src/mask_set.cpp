#include "visioncpp/mask_set.hpp"

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

namespace visioncpp {
namespace {

cv::Mat keepLargestCc(const cv::Mat& m) {
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(m, labels, stats, centroids, 8, CV_32S);
    if (n <= 1) return m;
    int best = 1, best_area = stats.at<int>(1, cv::CC_STAT_AREA);
    for (int i = 2; i < n; ++i) {
        const int a = stats.at<int>(i, cv::CC_STAT_AREA);
        if (a > best_area) {
            best_area = a;
            best = i;
        }
    }
    return labels == best;
}

}  // namespace

MaskSet MaskSet::loadYaml(const std::string& path, int height, int width) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        throw VisionError(std::string("cannot read masks yaml: ") + e.what());
    }
    const YAML::Node items = root["masks"];
    if (!items || !items.IsSequence() || items.size() == 0) {
        throw VisionError("no masks defined in masks yaml");
    }
    MaskSet set;
    set.mask_ = cv::Mat::zeros(height, width, CV_8UC1);
    int n_poly = 0;
    for (const auto& item : items) {
        const YAML::Node polys = item["polygons"];
        if (!polys) continue;
        for (const auto& poly : polys) {
            if (!poly || poly.size() < 3) continue;
            std::vector<cv::Point> pts;
            pts.reserve(poly.size());
            for (const auto& p : poly) {
                if (!p || p.size() < 2) continue;
                pts.emplace_back(p[0].as<int>(), p[1].as<int>());
            }
            if (pts.size() < 3) continue;
            const cv::Point* ptr = pts.data();
            const int n = static_cast<int>(pts.size());
            cv::fillPoly(set.mask_, &ptr, &n, 1, cv::Scalar(255));
            ++n_poly;
        }
    }
    if (n_poly == 0 || cv::countNonZero(set.mask_) < 50) {
        throw VisionError("mask area is empty or too small");
    }
    return set;
}

cv::Mat MaskSet::asBoolU8() const { return mask_.clone(); }

std::vector<cv::Mat> MaskSet::splitLegs(double depth_threshold_ratio, double overlap_px) const {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return {mask_ > 0};

    auto it = std::max_element(contours.begin(), contours.end(),
                               [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
    std::vector<cv::Point> c = *it;
    std::vector<int> hull;
    cv::convexHull(c, hull, false, false);
    if (static_cast<int>(hull.size()) < 3) return {mask_ > 0};

    std::vector<cv::Vec4i> defects;
    cv::convexityDefects(c, hull, defects);
    if (defects.empty()) return {mask_ > 0};

    const cv::Rect br = cv::boundingRect(c);
    double max_depth = 0;
    cv::Point start, end, far;
    bool found = false;
    for (const auto& d : defects) {
        const cv::Point s = c[d[0]];
        const cv::Point e = c[d[1]];
        const cv::Point f = c[d[2]];
        const double depth = d[3] / 256.0;
        const double dist_se = std::hypot(static_cast<double>(s.x - e.x), static_cast<double>(s.y - e.y));
        if (depth > max_depth && dist_se > 0.2 * br.width && depth > depth_threshold_ratio * br.height) {
            const int min_x = std::min(s.x, e.x);
            const int max_x = std::max(s.x, e.x);
            if (min_x < f.x && f.x < max_x) {
                max_depth = depth;
                start = s;
                end = e;
                far = f;
                found = true;
            }
        }
    }
    if (!found) return {mask_ > 0};
    if (start.x > end.x) std::swap(start, end);

    const cv::Point2d V(end.x - start.x, end.y - start.y);
    const double v_norm = std::hypot(V.x, V.y);
    if (v_norm < 1e-6) return {mask_ > 0};

    cv::Mat mask_bool = mask_ > 0;
    cv::Mat left = cv::Mat::zeros(mask_.size(), CV_8UC1);
    cv::Mat right = cv::Mat::zeros(mask_.size(), CV_8UC1);
    overlap_px = std::max(overlap_px, 0.0);
    for (int y = 0; y < mask_.rows; ++y) {
        const uint8_t* row = mask_bool.ptr<uint8_t>(y);
        uint8_t* lrow = left.ptr<uint8_t>(y);
        uint8_t* rrow = right.ptr<uint8_t>(y);
        for (int x = 0; x < mask_.cols; ++x) {
            if (!row[x]) continue;
            const double signed_d = ((x - far.x) * V.x + (y - far.y) * V.y) / v_norm;
            if (signed_d <= overlap_px) lrow[x] = 255;
            if (signed_d > -overlap_px) rrow[x] = 255;
        }
    }
    left = keepLargestCc(left);
    right = keepLargestCc(right);
    return {left > 0, right > 0};
}

}  // namespace visioncpp
