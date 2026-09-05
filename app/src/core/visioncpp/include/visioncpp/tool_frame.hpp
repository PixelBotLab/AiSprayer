#pragma once

#include "visioncpp/types.hpp"

namespace visioncpp {

class ToolFrame {
public:
    Mat3 compute(const Vec3& n_base);
    static Vec3 rpyXyzDeg(const Mat3& R);

private:
    Vec3 prev_x_ = Vec3::Zero();
    bool has_prev_ = false;
};

}  // namespace visioncpp
