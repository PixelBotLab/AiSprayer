#include "visioncpp/plane_slicer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace visioncpp {
namespace {

constexpr double kMerge = 1e-8;
constexpr double kZero = 1e-13;

Vec3 unitize(const Vec3& v) {
    const double n = v.norm();
    if (n < 1e-15) return Vec3::Zero();
    return v / n;
}

bool uniqueValueInRow(const int s[3], int* unique_col) {
    *unique_col = -1;
    for (int val : {-1, 1}) {
        int count = 0, col = -1;
        for (int i = 0; i < 3; ++i) {
            if (s[i] == val) {
                ++count;
                col = i;
            }
        }
        if (count == 1) *unique_col = col;
    }
    return *unique_col >= 0;
}

void planeLine(const Vec3& origin, const Vec3& normal, const Vec3& a, const Vec3& b,
               bool segments, Vec3* hit, bool* valid) {
    const Vec3 dir = unitize(b - a);
    const Vec3 n = unitize(normal);
    const double t = n.dot(origin - a);
    const double den = n.dot(dir);
    *valid = std::abs(den) > kZero;
    if (segments) {
        const double test = n.dot(origin - b);
        const bool different = (t >= 0) != (test >= 0) ? std::signbit(t) != std::signbit(test) : (t != 0 || test != 0);
        // match numpy sign: sign(0)=0, so sign(t)!=sign(test) when one is 0 and other isn't, or opposite
        auto sgn = [](double x) { return x > 0 ? 1 : (x < 0 ? -1 : 0); };
        const bool different_sides = sgn(t) != sgn(test);
        const bool nonzero = std::abs(t) > kZero || std::abs(test) > kZero;
        *valid = *valid && different_sides && nonzero;
        (void)different;
    }
    if (!*valid) return;
    *hit = a + (t / den) * dir;
}

}  // namespace

std::vector<std::array<Vec3, 2>> PlaneSlicer::meshPlane(const Mesh& mesh,
                                                        const Vec3& plane_normal,
                                                        const Vec3& plane_origin) {
    std::vector<std::array<Vec3, 2>> lines;
    if (mesh.faces.empty()) return lines;

    std::vector<int8_t> vsign(mesh.vertices.size(), 0);
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        const double d = (mesh.vertices[i] - plane_origin).dot(plane_normal);
        if (d < -kMerge) vsign[i] = -1;
        else if (d > kMerge) vsign[i] = 1;
    }

    for (const auto& f : mesh.faces) {
        const int s[3] = {vsign[f[0]], vsign[f[1]], vsign[f[2]]};
        int sorted[3] = {s[0], s[1], s[2]};
        std::sort(sorted, sorted + 3);
        int coded = 14;
        for (int i = 0; i < 3; ++i) coded += sorted[i] << (3 - i);

        const bool basic = (coded == 4 || coded == 12);
        const bool one_vertex = (coded == 8);
        const bool one_edge = (coded == 16);

        if (basic) {
            int uniq = -1;
            if (!uniqueValueInRow(s, &uniq)) continue;
            const int i0 = uniq;
            const int i1 = (uniq + 1) % 3;
            const int i2 = (uniq + 2) % 3;
            Vec3 h0, h1;
            bool v0 = false, v1 = false;
            planeLine(plane_origin, plane_normal, mesh.vertices[f[i0]], mesh.vertices[f[i1]], false, &h0, &v0);
            planeLine(plane_origin, plane_normal, mesh.vertices[f[i0]], mesh.vertices[f[i2]], false, &h1, &v1);
            if (v0 && v1) lines.push_back({h0, h1});
        } else if (one_vertex) {
            int on = -1, a = -1, b = -1;
            for (int i = 0; i < 3; ++i) {
                if (s[i] == 0) on = i;
                else if (a < 0) a = i;
                else b = i;
            }
            if (on < 0 || a < 0 || b < 0) continue;
            Vec3 hit;
            bool ok = false;
            planeLine(plane_origin, plane_normal, mesh.vertices[f[a]], mesh.vertices[f[b]], false, &hit, &ok);
            if (ok) lines.push_back({mesh.vertices[f[on]], hit});
        } else if (one_edge) {
            int offs[2], n_off = 0;
            Vec3 pts[2];
            int n_on = 0;
            for (int i = 0; i < 3; ++i) {
                if (s[i] == 0 && n_on < 2) pts[n_on++] = mesh.vertices[f[i]];
                else offs[n_off++] = i;
            }
            if (n_on == 2) lines.push_back({pts[0], pts[1]});
            (void)offs;
        }
    }
    return lines;
}

}  // namespace visioncpp
