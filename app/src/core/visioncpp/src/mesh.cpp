#include "visioncpp/mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <numeric>
#include <sstream>
#include <thread>
#include <unordered_map>

namespace visioncpp {
namespace {

uint32_t readU32(std::istream& in, bool le) {
    uint8_t b[4];
    in.read(reinterpret_cast<char*>(b), 4);
    if (le) return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8)
                    | (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24);
    return (static_cast<uint32_t>(b[0]) << 24) | (static_cast<uint32_t>(b[1]) << 16)
           | (static_cast<uint32_t>(b[2]) << 8) | static_cast<uint32_t>(b[3]);
}

float readF32(std::istream& in, bool le) {
    uint32_t u = readU32(in, le);
    float f;
    std::memcpy(&f, &u, 4);
    return f;
}

void writeU32le(std::ostream& out, uint32_t v) {
    uint8_t b[4] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8),
                    static_cast<uint8_t>(v >> 16), static_cast<uint8_t>(v >> 24)};
    out.write(reinterpret_cast<char*>(b), 4);
}

void writeF32le(std::ostream& out, float v) {
    uint32_t u;
    std::memcpy(&u, &v, 4);
    writeU32le(out, u);
}

struct PlyProp {
    std::string name;
    std::string type;  // float, double, uchar, int, ...
    bool is_list = false;
    std::string count_type;
    std::string item_type;
};

int typeSize(const std::string& t) {
    if (t == "char" || t == "uchar" || t == "int8" || t == "uint8") return 1;
    if (t == "short" || t == "ushort" || t == "int16" || t == "uint16") return 2;
    if (t == "int" || t == "uint" || t == "int32" || t == "uint32" || t == "float" || t == "float32") return 4;
    if (t == "double" || t == "float64") return 8;
    return 4;
}

double readNumber(std::istream& in, const std::string& t, bool binary, bool le) {
    if (!binary) {
        double v = 0;
        in >> v;
        return v;
    }
    if (t == "float" || t == "float32") return readF32(in, le);
    if (t == "double" || t == "float64") {
        uint8_t b[8];
        in.read(reinterpret_cast<char*>(b), 8);
        uint64_t u = 0;
        if (le) {
            for (int i = 0; i < 8; ++i) u |= static_cast<uint64_t>(b[i]) << (8 * i);
        } else {
            for (int i = 0; i < 8; ++i) u |= static_cast<uint64_t>(b[i]) << (8 * (7 - i));
        }
        double d;
        std::memcpy(&d, &u, 8);
        return d;
    }
    if (t == "uchar" || t == "uint8") {
        uint8_t v = 0;
        in.read(reinterpret_cast<char*>(&v), 1);
        return v;
    }
    if (t == "char" || t == "int8") {
        int8_t v = 0;
        in.read(reinterpret_cast<char*>(&v), 1);
        return v;
    }
    if (t == "int" || t == "int32") return static_cast<int32_t>(readU32(in, le));
    if (t == "uint" || t == "uint32") return readU32(in, le);
    if (t == "short" || t == "int16") {
        uint8_t b[2];
        in.read(reinterpret_cast<char*>(b), 2);
        int16_t v = le ? static_cast<int16_t>(b[0] | (b[1] << 8))
                       : static_cast<int16_t>(b[1] | (b[0] << 8));
        return v;
    }
    if (t == "ushort" || t == "uint16") {
        uint8_t b[2];
        in.read(reinterpret_cast<char*>(b), 2);
        uint16_t v = le ? static_cast<uint16_t>(b[0] | (b[1] << 8))
                        : static_cast<uint16_t>(b[1] | (b[0] << 8));
        return v;
    }
    throw VisionError("unsupported ply type: " + t);
}

}  // namespace

Mesh Mesh::loadPly(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw VisionError("cannot open ply: " + path);
    std::string line;
    std::getline(in, line);
    if (line.find("ply") != 0) throw VisionError("not a ply: " + path);
    bool binary = false, le = true;
    int nvert = 0, nface = 0;
    std::vector<PlyProp> vprops, fprops;
    std::vector<PlyProp>* cur = nullptr;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream ss(line);
        std::string tok;
        ss >> tok;
        if (tok == "format") {
            std::string fmt;
            ss >> fmt;
            binary = fmt.find("binary") != std::string::npos;
            le = fmt.find("big") == std::string::npos;
        } else if (tok == "element") {
            std::string name;
            int n = 0;
            ss >> name >> n;
            if (name == "vertex") {
                nvert = n;
                cur = &vprops;
            } else if (name == "face") {
                nface = n;
                cur = &fprops;
            } else {
                cur = nullptr;
            }
        } else if (tok == "property" && cur) {
            PlyProp p;
            std::string t;
            ss >> t;
            if (t == "list") {
                p.is_list = true;
                ss >> p.count_type >> p.item_type >> p.name;
            } else {
                p.type = t;
                ss >> p.name;
            }
            cur->push_back(p);
        } else if (tok == "end_header") {
            break;
        }
    }

    int ix = -1, iy = -1, iz = -1;
    for (int i = 0; i < static_cast<int>(vprops.size()); ++i) {
        if (vprops[i].name == "x") ix = i;
        if (vprops[i].name == "y") iy = i;
        if (vprops[i].name == "z") iz = i;
    }
    if (ix < 0 || iy < 0 || iz < 0) throw VisionError("ply missing x/y/z: " + path);

    Mesh mesh;
    mesh.vertices.resize(nvert);
    for (int i = 0; i < nvert; ++i) {
        std::vector<double> vals(vprops.size(), 0);
        for (size_t p = 0; p < vprops.size(); ++p) {
            if (vprops[p].is_list) throw VisionError("unexpected list property on vertex");
            vals[p] = readNumber(in, vprops[p].type, binary, le);
        }
        mesh.vertices[i] = Vec3(vals[ix], vals[iy], vals[iz]);
    }

    int list_idx = -1;
    for (int i = 0; i < static_cast<int>(fprops.size()); ++i) {
        if (fprops[i].is_list) list_idx = i;
    }
    if (list_idx < 0) throw VisionError("ply face missing vertex list: " + path);

    mesh.faces.reserve(nface);
    for (int i = 0; i < nface; ++i) {
        for (int p = 0; p < static_cast<int>(fprops.size()); ++p) {
            if (!fprops[p].is_list) {
                readNumber(in, fprops[p].type, binary, le);
                continue;
            }
            const int count = static_cast<int>(readNumber(in, fprops[p].count_type, binary, le));
            std::vector<int> idx(count);
            for (int k = 0; k < count; ++k) idx[k] = static_cast<int>(readNumber(in, fprops[p].item_type, binary, le));
            for (int k = 1; k + 1 < count; ++k) {
                mesh.faces.emplace_back(idx[0], idx[k], idx[k + 1]);
            }
        }
    }
    return mesh;
}

void Mesh::savePly(const std::string& path) const {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw VisionError("cannot write ply: " + path);
    out << "ply\nformat binary_little_endian 1.0\n";
    out << "element vertex " << vertices.size() << "\n";
    out << "property float x\nproperty float y\nproperty float z\n";
    out << "element face " << faces.size() << "\n";
    out << "property list uchar int vertex_indices\nend_header\n";
    for (const auto& v : vertices) {
        writeF32le(out, static_cast<float>(v.x()));
        writeF32le(out, static_cast<float>(v.y()));
        writeF32le(out, static_cast<float>(v.z()));
    }
    for (const auto& f : faces) {
        const uint8_t n = 3;
        out.write(reinterpret_cast<const char*>(&n), 1);
        writeU32le(out, static_cast<uint32_t>(f[0]));
        writeU32le(out, static_cast<uint32_t>(f[1]));
        writeU32le(out, static_cast<uint32_t>(f[2]));
    }
}

void Mesh::saveStl(const std::string& path) const {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw VisionError("cannot write stl: " + path);
    char header[80] = {};
    std::memcpy(header, "visioncpp", 9);
    out.write(header, 80);
    const uint32_t n = static_cast<uint32_t>(faces.size());
    writeU32le(out, n);
    for (const auto& f : faces) {
        const Vec3 a = vertices[f[0]], b = vertices[f[1]], c = vertices[f[2]];
        Vec3 nrm = (b - a).cross(c - a);
        const double len = nrm.norm();
        if (len > 1e-15) nrm /= len;
        writeF32le(out, static_cast<float>(nrm.x()));
        writeF32le(out, static_cast<float>(nrm.y()));
        writeF32le(out, static_cast<float>(nrm.z()));
        for (int k = 0; k < 3; ++k) {
            writeF32le(out, static_cast<float>(vertices[f[k]].x()));
            writeF32le(out, static_cast<float>(vertices[f[k]].y()));
            writeF32le(out, static_cast<float>(vertices[f[k]].z()));
        }
        uint16_t attr = 0;
        out.write(reinterpret_cast<char*>(&attr), 2);
    }
}

void Mesh::computeVertexNormals() {
    vertex_normals.assign(vertices.size(), Vec3::Zero());
    for (const auto& f : faces) {
        const Vec3& a = vertices[f[0]];
        const Vec3& b = vertices[f[1]];
        const Vec3& c = vertices[f[2]];
        Vec3 fn = (b - a).cross(c - a);
        const double fnlen = fn.norm();
        if (fnlen < 1e-15) continue;
        fn /= fnlen;
        const Vec3 e0 = (b - a).normalized();
        const Vec3 e1 = (c - a).normalized();
        const Vec3 e2 = (c - b).normalized();
        const Vec3 e3 = (a - b).normalized();
        const Vec3 e4 = (a - c).normalized();
        const Vec3 e5 = (b - c).normalized();
        const double ang0 = std::acos(std::clamp(e0.dot(e1), -1.0, 1.0));
        const double ang1 = std::acos(std::clamp(e2.dot(e3), -1.0, 1.0));
        const double ang2 = std::acos(std::clamp(e4.dot(e5), -1.0, 1.0));
        vertex_normals[f[0]] += ang0 * fn;
        vertex_normals[f[1]] += ang1 * fn;
        vertex_normals[f[2]] += ang2 * fn;
    }
    for (auto& n : vertex_normals) {
        const double len = n.norm();
        if (len > 1e-15) n /= len;
        else n = Vec3::Zero();
    }
}

void Mesh::taubinSmooth(int iterations, double lambda, double mu) {
    if (iterations <= 0 || vertices.empty() || faces.empty()) return;
    std::vector<std::vector<int>> adj(vertices.size());
    for (const auto& f : faces) {
        for (int i = 0; i < 3; ++i) {
            const int a = f[i], b = f[(i + 1) % 3];
            adj[a].push_back(b);
            adj[b].push_back(a);
        }
    }
    for (auto& n : adj) {
        std::sort(n.begin(), n.end());
        n.erase(std::unique(n.begin(), n.end()), n.end());
    }

    auto step = [&](double coef) {
        std::vector<Vec3> next = vertices;
        const size_t n = vertices.size();
        const unsigned hc = std::thread::hardware_concurrency();
        const unsigned w = hc ? std::min(hc, 4u) : 1;
        auto body = [&](size_t i) {
            if (adj[i].empty()) return;
            Vec3 avg = Vec3::Zero();
            for (int j : adj[i]) avg += vertices[j];
            avg /= static_cast<double>(adj[i].size());
            next[i] = vertices[i] + coef * (avg - vertices[i]);
        };
        if (n < 512 || w <= 1) {
            for (size_t i = 0; i < n; ++i) body(i);
        } else {
            std::vector<std::thread> ts;
            const size_t chunk = (n + w - 1) / w;
            for (unsigned t = 0; t < w; ++t) {
                const size_t a = static_cast<size_t>(t) * chunk;
                const size_t b = std::min(n, a + chunk);
                if (a >= b) break;
                ts.emplace_back([a, b, &body]() {
                    for (size_t i = a; i < b; ++i) body(i);
                });
            }
            for (auto& th : ts) th.join();
        }
        vertices.swap(next);
    };
    for (int i = 0; i < iterations; ++i) {
        step(lambda);
        step(mu);
    }
}

void Mesh::weldVertices(double eps) {
    if (vertices.empty()) return;
    eps = std::max(eps, 1e-9);
    const double inv = 1.0 / eps;
    struct Key {
        int64_t x, y, z;
        bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct Hash {
        size_t operator()(const Key& k) const {
            return (static_cast<size_t>(k.x) * 73856093) ^ (static_cast<size_t>(k.y) * 19349663)
                   ^ (static_cast<size_t>(k.z) * 83492791);
        }
    };
    std::unordered_map<Key, int, Hash> first;
    std::vector<int> remap(vertices.size(), -1);
    std::vector<Vec3> nv;
    std::vector<double> nd;
    nv.reserve(vertices.size());
    first.reserve(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        const Key key{static_cast<int64_t>(std::llround(vertices[i].x() * inv)),
                      static_cast<int64_t>(std::llround(vertices[i].y() * inv)),
                      static_cast<int64_t>(std::llround(vertices[i].z() * inv))};
        auto it = first.find(key);
        if (it == first.end()) {
            remap[i] = static_cast<int>(nv.size());
            first.emplace(key, remap[i]);
            nv.push_back(vertices[i]);
            if (!densities.empty()) nd.push_back(densities[i]);
        } else {
            remap[i] = it->second;
        }
    }
    std::vector<Eigen::Vector3i> nf;
    nf.reserve(faces.size());
    for (const auto& f : faces) {
        const int a = remap[f[0]], b = remap[f[1]], c = remap[f[2]];
        if (a < 0 || b < 0 || c < 0 || a == b || b == c || a == c) continue;
        nf.emplace_back(a, b, c);
    }
    vertices.swap(nv);
    faces.swap(nf);
    if (!densities.empty()) densities.swap(nd);
    vertex_normals.clear();
}

void Mesh::keepLargestComponent() {
    if (faces.empty()) return;
    std::unordered_map<uint64_t, std::vector<int>> edge_faces;
    auto ek = [](int a, int b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) | static_cast<uint32_t>(b);
    };
    for (int fi = 0; fi < static_cast<int>(faces.size()); ++fi) {
        for (int i = 0; i < 3; ++i) {
            edge_faces[ek(faces[fi][i], faces[fi][(i + 1) % 3])].push_back(fi);
        }
    }
    std::vector<std::vector<int>> adj(faces.size());
    for (const auto& kv : edge_faces) {
        const auto& v = kv.second;
        for (size_t i = 0; i < v.size(); ++i) {
            for (size_t j = i + 1; j < v.size(); ++j) {
                adj[v[i]].push_back(v[j]);
                adj[v[j]].push_back(v[i]);
            }
        }
    }
    std::vector<int> comp(faces.size(), -1);
    int ncomp = 0, best = 0, best_sz = 0;
    std::vector<int> stack;
    for (int i = 0; i < static_cast<int>(faces.size()); ++i) {
        if (comp[i] >= 0) continue;
        int sz = 0;
        stack = {i};
        comp[i] = ncomp;
        while (!stack.empty()) {
            const int u = stack.back();
            stack.pop_back();
            ++sz;
            for (int v : adj[u]) {
                if (comp[v] < 0) {
                    comp[v] = ncomp;
                    stack.push_back(v);
                }
            }
        }
        if (sz > best_sz) {
            best_sz = sz;
            best = ncomp;
        }
        ++ncomp;
    }
    if (ncomp <= 1) return;
    std::vector<Eigen::Vector3i> kept;
    kept.reserve(best_sz);
    for (int i = 0; i < static_cast<int>(faces.size()); ++i) {
        if (comp[i] == best) kept.push_back(faces[i]);
    }
    faces.swap(kept);
    std::vector<char> used(vertices.size(), 0);
    for (const auto& f : faces) {
        used[f[0]] = used[f[1]] = used[f[2]] = 1;
    }
    std::vector<int> remap(vertices.size(), -1);
    std::vector<Vec3> nv;
    nv.reserve(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        if (!used[i]) continue;
        remap[i] = static_cast<int>(nv.size());
        nv.push_back(vertices[i]);
    }
    for (auto& f : faces) {
        f[0] = remap[f[0]];
        f[1] = remap[f[1]];
        f[2] = remap[f[2]];
    }
    vertices.swap(nv);
    densities.clear();
    vertex_normals.clear();
}

void Mesh::removeVerticesByMask(const std::vector<char>& remove) {
    if (remove.size() != vertices.size()) throw VisionError("remove mask size mismatch");
    std::vector<int> remap(vertices.size(), -1);
    std::vector<Vec3> nv;
    std::vector<double> nd;
    nv.reserve(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        if (remove[i]) continue;
        remap[i] = static_cast<int>(nv.size());
        nv.push_back(vertices[i]);
        if (!densities.empty()) nd.push_back(densities[i]);
    }
    std::vector<Eigen::Vector3i> nf;
    nf.reserve(faces.size());
    for (const auto& f : faces) {
        const int a = remap[f[0]], b = remap[f[1]], c = remap[f[2]];
        if (a < 0 || b < 0 || c < 0) continue;
        nf.emplace_back(a, b, c);
    }
    vertices.swap(nv);
    faces.swap(nf);
    densities.swap(nd);
    vertex_normals.clear();
}

void Mesh::transform(const Mat4& T) {
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 t = T.block<3, 1>(0, 3);
    for (auto& v : vertices) v = R * v + t;
    for (auto& n : vertex_normals) n = (R * n).normalized();
}

}  // namespace visioncpp
