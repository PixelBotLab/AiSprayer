#pragma once

#include "visioncpp/types.hpp"

namespace visioncpp {

class NormalSmoother {
public:
    explicit NormalSmoother(int window_size = 5) : window_(window_size) {}
    void smooth(std::vector<OrientedSample>& samples) const;

private:
    int window_ = 5;
};

}  // namespace visioncpp
