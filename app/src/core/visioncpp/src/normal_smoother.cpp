#include "visioncpp/normal_smoother.hpp"

#include <cmath>
#include <vector>

namespace visioncpp {

void NormalSmoother::smooth(std::vector<OrientedSample>& samples) const {
    if (samples.empty() || static_cast<int>(samples.size()) < window_ || window_ <= 1) return;
    const int n = static_cast<int>(samples.size());
    const int pad = window_ / 2;
    std::vector<Vec3> padded(n + 2 * pad);
    for (int i = 0; i < pad; ++i) padded[i] = samples.front().normal;
    for (int i = 0; i < n; ++i) padded[i + pad] = samples[i].normal;
    for (int i = 0; i < pad; ++i) padded[n + pad + i] = samples.back().normal;

    for (int i = 0; i < n; ++i) {
        Vec3 acc = Vec3::Zero();
        for (int k = 0; k < window_; ++k) acc += padded[i + k];
        acc /= static_cast<double>(window_);
        const double len = acc.norm();
        if (len < 1e-6) {
            acc = samples[i].normal;
            const double nlen = acc.norm();
            if (nlen < 1e-9) acc = Vec3::UnitZ();
            else acc /= nlen;
        } else {
            acc /= len;
        }
        samples[i].normal = acc;
    }
}

}  // namespace visioncpp
