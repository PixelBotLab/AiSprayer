#pragma once

#include "visioncpp/mesh.hpp"

#include <array>
#include <vector>

namespace visioncpp {

class PlaneSlicer {
public:
    static std::vector<std::array<Vec3, 2>> meshPlane(const Mesh& mesh,
                                                      const Vec3& plane_normal,
                                                      const Vec3& plane_origin);
};

}  // namespace visioncpp
