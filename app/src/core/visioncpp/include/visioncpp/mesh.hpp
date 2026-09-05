#pragma once

#include "visioncpp/types.hpp"

#include <string>
#include <vector>

namespace visioncpp {

class Mesh {
public:
    std::vector<Vec3> vertices;
    std::vector<Eigen::Vector3i> faces;
    std::vector<Vec3> vertex_normals;
    std::vector<double> densities;

    static Mesh loadPly(const std::string& path);
    void savePly(const std::string& path) const;
    void saveStl(const std::string& path) const;

    void computeVertexNormals();
    void taubinSmooth(int iterations, double lambda = 0.5, double mu = -0.53);
    void weldVertices(double eps);
    void keepLargestComponent();
    void removeVerticesByMask(const std::vector<char>& remove);
    void transform(const Mat4& T);

    int vertexCount() const { return static_cast<int>(vertices.size()); }
    int faceCount() const { return static_cast<int>(faces.size()); }
};

}  // namespace visioncpp
