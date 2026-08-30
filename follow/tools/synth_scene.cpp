#include "synth_scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>

namespace follow {
namespace synth {
namespace {

constexpr double kEps = 1e-12;
constexpr double kPi = 3.14159265358979323846;

std::string token_at(const std::string& line, size_t n) {
  std::istringstream ss(line);
  std::string t;
  for (size_t i = 0; ss >> t; ++i) {
    if (i == n) {
      return t;
    }
  }
  return {};
}

int prop_bytes(const std::string& t) {
  if (t == "char" || t == "uchar" || t == "int8" || t == "uint8") {
    return 1;
  }
  if (t == "short" || t == "ushort" || t == "int16" || t == "uint16") {
    return 2;
  }
  if (t == "int" || t == "uint" || t == "float" || t == "int32" || t == "uint32" ||
      t == "float32") {
    return 4;
  }
  if (t == "double" || t == "long" || t == "ulong" || t == "int64" || t == "uint64" ||
      t == "float64") {
    return 8;
  }
  return 0;
}

// 只在已确认小端、且类型已由 prop_bytes() 校验过的字节上调用。
double read_scalar(const char* src, const std::string& type) {
  if (type == "float" || type == "float32") {
    float v;
    std::memcpy(&v, src, 4);
    return v;
  }
  if (type == "double" || type == "float64") {
    double v;
    std::memcpy(&v, src, 8);
    return v;
  }
  if (type == "uchar" || type == "uint8") {
    return static_cast<unsigned char>(*src);
  }
  if (type == "char" || type == "int8") {
    return static_cast<signed char>(*src);
  }
  if (type == "ushort" || type == "uint16") {
    uint16_t v;
    std::memcpy(&v, src, 2);
    return v;
  }
  if (type == "short" || type == "int16") {
    int16_t v;
    std::memcpy(&v, src, 2);
    return v;
  }
  int32_t v;
  std::memcpy(&v, src, 4);
  return v;
}

struct Prop {
  std::string type;        // 标量类型；list 时是**元素**类型
  std::string name;
  std::string count_type;  // list 的计数字段类型
  bool is_list = false;
};

struct Element {
  std::string name;
  size_t count = 0;
  std::vector<Prop> props;
};

struct VertexLayout {
  std::vector<int> offset;   // 每个属性到行首的字节偏移
  size_t stride = 0;
  int x = -1, y = -1, z = -1;  // 属性下标
};

}  // namespace

Eigen::AlignedBox3f Mesh::bbox() const {
  Eigen::AlignedBox3f b;
  for (const auto& v : verts) {
    b.extend(v);
  }
  return b;
}

bool load_ply(const std::string& path, Mesh* out, std::string* err) {
  auto fail = [&](const std::string& m) {
    if (err) {
      *err = m;
    }
    return false;
  };
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return fail("打不开 " + path);
  }
  std::string line;
  if (!std::getline(f, line) || line.rfind("ply", 0) != 0) {
    return fail("不是 PLY（首行不是 'ply'）");
  }
  bool binary = true;
  std::vector<Element> elements;
  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line == "end_header") {
      break;
    }
    const std::string kw = token_at(line, 0);
    if (kw == "format") {
      const std::string fmt = token_at(line, 1);
      if (fmt == "ascii") {
        binary = false;
      } else if (fmt == "binary_little_endian") {
        binary = true;
      } else {
        // 大端：这台板子上不会有这种文件，而按错字节序读出来的网格是"看起来正常"的垃圾，
        // 所以宁可拒绝。
        return fail("不支持的 PLY format: " + fmt);
      }
    } else if (kw == "element") {
      Element e;
      e.name = token_at(line, 1);
      e.count = static_cast<size_t>(std::stoull(token_at(line, 2)));
      elements.push_back(std::move(e));
    } else if (kw == "property") {
      if (elements.empty()) {
        return fail("property 出现在 element 之前");
      }
      Prop p;
      const std::string t1 = token_at(line, 1);
      if (t1 == "list") {
        // property list <计数类型> <元素类型> <名字>
        p.is_list = true;
        p.count_type = token_at(line, 2);
        p.type = token_at(line, 3);
        p.name = token_at(line, 4);
        if (prop_bytes(p.count_type) == 0) {
          return fail("list 计数类型不支持: " + p.count_type);
        }
      } else {
        p.type = t1;
        p.name = token_at(line, 2);
      }
      elements.back().props.push_back(std::move(p));
    }
    // 其它关键字（comment/obj_info/自定义）忽略：它们不该改变顶点与面数据，真改了由下面
    // 的数量一致性与索引越界校验抓住。
  }

  int vi = -1, fi = -1;
  for (size_t i = 0; i < elements.size(); ++i) {
    if (elements[i].name == "vertex") {
      vi = static_cast<int>(i);
    } else if (elements[i].name == "face") {
      fi = static_cast<int>(i);
    }
  }
  if (vi < 0 || fi < 0) {
    return fail("PLY 里找不到 element vertex / element face");
  }

  VertexLayout vl;
  {
    const auto& props = elements[vi].props;
    for (size_t i = 0; i < props.size(); ++i) {
      const Prop& p = props[i];
      if (p.is_list) {
        return fail("vertex 元素含变长属性，无法定步长读取");
      }
      const int sz = prop_bytes(p.type);
      if (sz == 0) {
        return fail("vertex 属性类型不支持: " + p.type);
      }
      vl.offset.push_back(static_cast<int>(vl.stride));
      vl.stride += static_cast<size_t>(sz);
      if (p.name == "x") {
        vl.x = static_cast<int>(i);
      } else if (p.name == "y") {
        vl.y = static_cast<int>(i);
      } else if (p.name == "z") {
        vl.z = static_cast<int>(i);
      } else if (p.name == "red" || p.name == "green" || p.name == "blue" || p.name == "alpha") {
        // 本工具的反照率是程序生成的，读进来的顶点颜色会被静默丢掉 —— 那会让一次"纹理跟踪"
        // 测试测到假东西，所以宁可直接拒绝并说明原因。
        return fail("vertex 含颜色属性但本工具忽略它：请改用无颜色网格");
      }
    }
  }
  if (vl.x < 0 || vl.y < 0 || vl.z < 0) {
    return fail("vertex 元素缺 x/y/z");
  }
  // 面的索引属性：vertex_indices（旧写出器）或 vertex_index。它前面的固定属性按步长跳过。
  int f_fixed_bytes = 0;
  std::string idx_type;    // 索引的元素类型
  std::string count_type;  // 计数的类型
  {
    const auto& props = elements[fi].props;
    bool found = false;
    for (const Prop& p : props) {
      if (!p.is_list) {
        if (found) {
          return fail("face 的 vertex_indices 之后还有固定属性，本工具不支持");
        }
        const int sz = prop_bytes(p.type);
        if (sz == 0) {
          return fail("face 属性类型不支持: " + p.type);
        }
        f_fixed_bytes += sz;
        continue;
      }
      if (found) {
        return fail("face 元素有两个变长属性");
      }
      if (p.name != "vertex_indices" && p.name != "vertex_index") {
        return fail("face 的变长属性不叫 vertex_indices: " + p.name);
      }
      found = true;
      idx_type = p.type;
      count_type = p.count_type;
    }
    if (!found) {
      return fail("face 元素里找不到 vertex_indices");
    }
    if (prop_bytes(idx_type) == 0) {
      return fail("面索引类型不支持: " + idx_type);
    }
  }

  Mesh m;
  m.verts.resize(elements[vi].count);
  m.tris.reserve(elements[fi].count);
  if (binary) {
    std::vector<char> vrow(vl.stride);
    const auto& vprops = elements[vi].props;
    for (size_t i = 0; i < elements[vi].count; ++i) {
      if (!f.read(vrow.data(), static_cast<std::streamsize>(vl.stride))) {
        return fail("顶点数据在第 " + std::to_string(i) + " 个处截断");
      }
      auto at = [&](int prop) {
        return static_cast<float>(read_scalar(vrow.data() + vl.offset[prop], vprops[prop].type));
      };
      m.verts[i] = Eigen::Vector3f(at(vl.x), at(vl.y), at(vl.z));
    }
    const int count_bytes = prop_bytes(count_type);
    const int idx_bytes = prop_bytes(idx_type);
    std::vector<char> skip(f_fixed_bytes), one(std::max(count_bytes, idx_bytes));
    for (size_t i = 0; i < elements[fi].count; ++i) {
      if (f_fixed_bytes && !f.read(skip.data(), f_fixed_bytes)) {
        return fail("面数据截断（固定属性）");
      }
      if (!f.read(one.data(), count_bytes)) {
        return fail("面数据截断（顶点计数）");
      }
      const int n = static_cast<int>(read_scalar(one.data(), count_type));
      if (n < 3 || n > 8) {
        return fail("面 " + std::to_string(i) + " 的顶点数 " + std::to_string(n) + " 不合理");
      }
      std::array<int32_t, 3> tri = {-1, -1, -1};
      for (int j = 0; j < n; ++j) {
        if (!f.read(one.data(), idx_bytes)) {
          return fail("面数据截断（索引）");
        }
        if (j < 3) {
          tri[j] = static_cast<int32_t>(read_scalar(one.data(), idx_type));
        }
      }
      m.tris.push_back(tri);  // 多边形只取前三个顶点：扫描件里 trimesh 出的全是三角形
    }
  } else {
    for (size_t i = 0; i < elements[vi].count; ++i) {
      if (!std::getline(f, line)) {
        return fail("ascii 顶点数据截断");
      }
      std::istringstream ss(line);
      std::vector<double> vals;
      double v;
      while (ss >> v) {
        vals.push_back(v);
      }
      if (vals.size() < elements[vi].props.size()) {
        return fail("ascii 顶点行字段不足");
      }
      m.verts[i] = Eigen::Vector3f(static_cast<float>(vals[vl.x]), static_cast<float>(vals[vl.y]),
                                   static_cast<float>(vals[vl.z]));
    }
    for (size_t i = 0; i < elements[fi].count; ++i) {
      if (!std::getline(f, line)) {
        return fail("ascii 面数据截断");
      }
      std::istringstream ss(line);
      int n = 0;
      ss >> n;
      if (n < 3 || n > 8) {
        return fail("ascii 面 " + std::to_string(i) + " 顶点数 " + std::to_string(n));
      }
      std::array<int32_t, 3> tri = {-1, -1, -1};
      for (int j = 0; j < n && ss; ++j) {
        long long t = -1;
        ss >> t;
        if (j < 3) {
          tri[j] = static_cast<int32_t>(t);
        }
      }
      m.tris.push_back(tri);
    }
  }

  // 到这里才算"读对了"：数量与 header 一致、索引不越界、坐标全有限。截断文件的尾部字节
  // 常常能凑出看起来合理的三角形，所以这三条缺一不可。
  if (m.verts.size() != elements[vi].count || m.tris.size() != elements[fi].count) {
    return fail("读到的元素数与 header 声明不一致");
  }
  for (const auto& t : m.tris) {
    for (int32_t i : t) {
      if (i < 0 || static_cast<size_t>(i) >= m.verts.size()) {
        return fail("面索引越界: " + std::to_string(i));
      }
    }
  }
  for (const auto& v : m.verts) {
    if (!v.allFinite()) {
      return fail("顶点含 NaN/inf");
    }
  }
  *out = std::move(m);
  return true;
}

// ---------------------------------------------------------------- Scene

namespace {
// 三角形的质心与包围盒。用 double：网格在基座系 1.7 m 处，float 只剩 ~1e-7 m 分辨率，
// 而这里要区分的是亚毫米级的命中顺序。
void tri_bounds(const Mesh& m, int32_t tri, Eigen::Vector3d* c, Eigen::Vector3d* lo,
                Eigen::Vector3d* hi) {
  const std::array<int32_t, 3>& i = m.tris[tri];
  const Eigen::Vector3d a = m.verts[i[0]].cast<double>();
  const Eigen::Vector3d b = m.verts[i[1]].cast<double>();
  const Eigen::Vector3d d = m.verts[i[2]].cast<double>();
  *c = (a + b + d) / 3.0;
  *lo = a.cwiseMin(b).cwiseMin(d);
  *hi = a.cwiseMax(b).cwiseMax(d);
}
}  // namespace

Scene::Scene(Mesh m) : mesh_(std::move(m)) {
  const size_t n = mesh_.tri_count();
  if (n == 0) {
    return;
  }
  prim_.resize(n);
  for (size_t i = 0; i < n; ++i) {
    prim_[i] = static_cast<int32_t>(i);
  }
  nodes_.reserve(n / 4);
  build_subtree(0, static_cast<int>(n), 0);
}

int Scene::build_subtree(int begin, int end, int depth) {
  Node nd;
  nd.count = end - begin;
  for (int i = begin; i < end; ++i) {
    Eigen::Vector3d c, lo, hi;
    tri_bounds(mesh_, prim_[i], &c, &lo, &hi);
    if (i == begin) {
      nd.lo = lo;
      nd.hi = hi;
    } else {
      nd.lo = nd.lo.cwiseMin(lo);
      nd.hi = nd.hi.cwiseMax(hi);
    }
  }
  const int self = static_cast<int>(nodes_.size());
  nodes_.push_back(nd);

  if (nd.count <= kLeafTris || depth > kMaxDepth) {
    nodes_[self].begin = begin;
    return self;
  }
  Eigen::Vector3d ext = nd.hi - nd.lo;
  int axis = 0;
  ext.maxCoeff(&axis);
  const int mid = begin + nd.count / 2;
  // 质心 median split。不上 SAH：扫描件各向同性，建树时间比查询质量更值得省，而叶子
  // 只有 8 面时遍历已经足够浅。
  std::nth_element(prim_.begin() + begin, prim_.begin() + mid, prim_.begin() + end,
                   [&](int32_t a, int32_t b) {
                     Eigen::Vector3d ca, cb, lo, hi;
                     tri_bounds(mesh_, a, &ca, &lo, &hi);
                     tri_bounds(mesh_, b, &cb, &lo, &hi);
                     return ca[axis] < cb[axis];
                   });
  // 递归里 nodes_ 会扩容，所以只能在返回后用下标写回，不能持引用。
  const int left = build_subtree(begin, mid, depth + 1);
  const int right = build_subtree(mid, end, depth + 1);
  nodes_[self].left = left;
  nodes_[self].right = right;
  return self;
}

double Scene::aabb_entry(const Eigen::Vector3d& o, const Eigen::Vector3d& inv_d,
                         const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, double t_max) {
  const Eigen::Vector3d t01 = (lo - o).cwiseProduct(inv_d);
  const Eigen::Vector3d t11 = (hi - o).cwiseProduct(inv_d);
  const Eigen::Vector3d tn = t01.cwiseMin(t11);
  const Eigen::Vector3d tf = t01.cwiseMax(t11);
  const double t_near = std::max(std::max(tn[0], tn[1]), tn[2]);
  const double t_far = std::min(std::min(tf[0], tf[1]), tf[2]);
  if (t_far < std::max(t_near, 0.0) || t_near > t_max) {
    return -1.0;
  }
  return t_near;
}

double Scene::cast(const Eigen::Vector3d& o, const Eigen::Vector3d& d, Eigen::Vector3d* point,
                   Eigen::Vector3d* normal, double t_max) const {
  if (!valid() || d.squaredNorm() < kEps) {
    return 0.0;
  }
  // 某个分量为零时 1/d 会得到 inf，而 (lo-o)*inf 在 lo==o 时是 NaN —— NaN 的区间测试直接
  // 判"不相交"，于是恰好穿过盒子边界的射线会漏。钳一个最小值：只有这种退化情况下方向被
  // 扰动 1e-12，三角形求交用的仍是原始 d。
  Eigen::Vector3d dd = d;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(dd[i]) < 1e-12) {
      dd[i] = dd[i] < 0.0 ? -1e-12 : 1e-12;
    }
  }
  const Eigen::Vector3d inv_d = dd.cwiseInverse();
  const double limit = t_max > 0.0 ? t_max : std::numeric_limits<double>::infinity();
  double best_t = limit;
  int best_tri = -1;

  std::vector<int> stack;
  stack.reserve(64);
  stack.push_back(0);
  while (!stack.empty()) {
    const int slot = stack.back();
    stack.pop_back();
    const Node& nd = nodes_[slot];
    if (aabb_entry(o, inv_d, nd.lo, nd.hi, best_t) < 0.0) {
      continue;
    }
    if (nd.left < 0) {
      for (int i = 0; i < nd.count; ++i) {
        const int32_t tri = prim_[nd.begin + i];
        const std::array<int32_t, 3>& ix = mesh_.tris[tri];
        const Eigen::Vector3d v0 = mesh_.verts[ix[0]].cast<double>();
        const Eigen::Vector3d e1 = mesh_.verts[ix[1]].cast<double>() - v0;
        const Eigen::Vector3d e2 = mesh_.verts[ix[2]].cast<double>() - v0;
        const Eigen::Vector3d pv = d.cross(e2);
        const double det = e1.dot(pv);
        if (std::abs(det) < 1e-15) {
          continue;  // 射线与三角形平面近平行
        }
        const double inv_det = 1.0 / det;
        const Eigen::Vector3d tv = o - v0;
        const double u = tv.dot(pv) * inv_det;
        if (u < -1e-9 || u > 1.0 + 1e-9) {
          continue;
        }
        const Eigen::Vector3d qv = tv.cross(e1);
        const double v = d.dot(qv) * inv_det;
        if (v < -1e-9 || u + v > 1.0 + 1e-9) {
          continue;
        }
        const double t = e2.dot(qv) * inv_det;
        if (t > 1e-9 && t < best_t) {
          best_t = t;
          best_tri = tri;
        }
      }
      continue;
    }
    const double el = aabb_entry(o, inv_d, nodes_[nd.left].lo, nodes_[nd.left].hi, best_t);
    const double er = aabb_entry(o, inv_d, nodes_[nd.right].lo, nodes_[nd.right].hi, best_t);
    if (el < 0.0 && er < 0.0) {
      continue;
    }
    // 近的先查（栈 LIFO ⇒ 远的先 push）：先拿到近命中后，best_t 一收紧就能整块剪掉远子树。
    if (el >= 0.0 && (er < 0.0 || el <= er)) {
      stack.push_back(nd.right);
      stack.push_back(nd.left);
    } else if (er >= 0.0) {
      stack.push_back(nd.left);
      stack.push_back(nd.right);
    }
  }
  if (best_tri < 0) {
    return 0.0;
  }
  if (point) {
    *point = o + best_t * d;
  }
  if (normal) {
    const std::array<int32_t, 3>& ix = mesh_.tris[best_tri];
    Eigen::Vector3d n = (mesh_.verts[ix[1]].cast<double>() - mesh_.verts[ix[0]].cast<double>())
                            .cross(mesh_.verts[ix[2]].cast<double>() - mesh_.verts[ix[0]].cast<double>());
    if (n.squaredNorm() < kEps) {
      n = Eigen::Vector3d::UnitZ();
    }
    n.normalize();
    if (n.dot(d) > 0.0) {
      n = -n;  // 总是朝向相机一侧：网格不必是闭合的，也不必有外向法线
    }
    *normal = n;
  }
  return best_t;
}

// ---------------------------------------------------------------- 渲染

cv::Mat render_depth(const CameraIntrinsics& k, const Eigen::Isometry3d& T_ref_cam, const Scene& s,
                     const RenderParams& p) {
  if (!k.valid() || !s.valid()) {
    return cv::Mat();
  }
  const int b = std::max(1, p.block);
  const int nsub = std::max(1, p.blur_samples);
  cv::Mat img = cv::Mat::zeros(k.height, k.width, CV_16UC1);
  std::mt19937 rng(p.seed);
  std::normal_distribution<double> noise(0.0, p.noise_mm * 0.001);
  std::uniform_real_distribution<double> drop(0.0, 1.0);

  // 曝光期内的子位姿：绕相机自身 y 轴扫过 blur_rot_deg，中心对齐命令位姿。
  // 相机中心不变（绕自身轴转），所以每个子位姿只需要一对旋转矩阵 —— 求逆必须在像素
  // 循环之外做完，block=1 时那是 100 万次 4x4 求逆。
  struct SubPose {
    Eigen::Matrix3d ref_from_cam;
    Eigen::Matrix3d cam_from_ref;
    Eigen::Vector3d origin;
  };
  std::vector<SubPose> subs;
  subs.reserve(nsub);
  const Eigen::Vector3d origin = T_ref_cam.translation();
  for (int q = 0; q < nsub; ++q) {
    const double blend = nsub == 1 ? 0.0
                         : (static_cast<double>(q) / static_cast<double>(nsub - 1) - 0.5) *
                               p.blur_rot_deg * kPi / 180.0;
    const Eigen::Matrix3d R = T_ref_cam.linear() *
                              Eigen::AngleAxisd(blend, Eigen::Vector3d::UnitY()).toRotationMatrix();
    subs.push_back({R, R.transpose(), origin});
  }

  for (int v0 = 0; v0 < k.height; v0 += b) {
    for (int u0 = 0; u0 < k.width; u0 += b) {
      const Eigen::Vector3d d_cam((u0 + 0.5 - k.cx) / k.fx, (v0 + 0.5 - k.cy) / k.fy, 1.0);
      // 取**最近**命中，不平均深度 —— 平均会在轮廓处造出根本不存在的中间值，那是在给
      // 求解器喂一个真实相机不会产生的错误。
      double z = std::numeric_limits<double>::infinity();
      for (const SubPose& sp : subs) {
        const Eigen::Vector3d dir = sp.ref_from_cam * d_cam;
        const double t = s.cast(sp.origin, dir);
        if (t <= 0.0) {
          continue;
        }
        // 相机系坐标 = Rᵀ·(命中点 − 相机中心)。少减这个 origin，单位旋转下得到的是"从参考
        // 原点量的深度"：沿光轴的平移于是完全进不了深度图（实测命令 10 mm 靠近，中心像素反而
        // +0.5 mm），而横向平移仍然会移动图像 —— 结果是把"范围轴不可观"这个假象演给求解器看。
        const Eigen::Vector3d hit_ref = sp.origin + t * dir;
        const double zq = (sp.cam_from_ref * (hit_ref - sp.origin)).z();
        if (zq < z) {
          z = zq;
        }
      }
      if (!std::isfinite(z)) {
        continue;
      }
      if (p.hole_fraction > 0.0 && drop(rng) < p.hole_fraction) {
        continue;
      }
      const double z_mm = (z + noise(rng)) * 1000.0;  // 噪声加在深度上，每个 block 一次
      if (!(z_mm > 0.0) || z_mm > 65535.0) {
        continue;
      }
      const ushort value = static_cast<ushort>(std::lround(z_mm));
      const int v1 = std::min(k.height, v0 + b);
      const int u1 = std::min(k.width, u0 + b);
      for (int v = v0; v < v1; ++v) {
        ushort* row = img.ptr<ushort>(v);
        for (int u = u0; u < u1; ++u) {
          row[u] = value;
        }
      }
    }
  }
  return img;
}

namespace {
// 反照率在**参考系**里按位置取（跟着工件走）。贴在图像上等于往场景里放一块幻灯片：
// 特征前端会稳稳跟住它，而它和几何运动毫无关系，测不出真东西。
double albedo_at(const Eigen::Vector3d& p_ref, double period_m) {
  if (!(period_m > 0.0)) {
    return 0.55;  // 无纹理：只有几何
  }
  const auto q = (p_ref / period_m).array().floor();
  const int parity = static_cast<int>(q[0] + 2.0 * q[1] + 3.0 * q[2]);
  return (parity & 1) ? 0.75 : 0.30;
}
}  // namespace

cv::Mat render_color(const CameraIntrinsics& k, const Eigen::Isometry3d& T_ref_cam, const Scene& s,
                     const ColorParams& cp) {
  if (!k.valid() || !s.valid()) {
    return cv::Mat();
  }
  const int b = std::max(1, cp.block);
  cv::Mat img = cv::Mat::zeros(k.height, k.width, CV_8UC3);
  std::mt19937 rng(cp.seed);
  std::normal_distribution<double> noise(0.0, cp.noise_gray);
  const Eigen::Vector3d origin = T_ref_cam.translation();
  const Eigen::Matrix3d R = T_ref_cam.linear();

  for (int v0 = 0; v0 < k.height; v0 += b) {
    for (int u0 = 0; u0 < k.width; u0 += b) {
      const Eigen::Vector3d d_cam((u0 + 0.5 - k.cx) / k.fx, (v0 + 0.5 - k.cy) / k.fy, 1.0);
      const Eigen::Vector3d dir = R * d_cam;
      Eigen::Vector3d hit, n;
      const double t = s.cast(origin, dir, &hit, &n);
      if (t <= 0.0) {
        continue;
      }
      // cast 出来的法线已归一且总是朝向相机，所以 Lambert 项直接点积即可。
      const double lambert = std::max(0.0, n.dot(cp.light_dir_ref));
      const double gray = std::min(1.0, std::max(0.0, albedo_at(hit, cp.texture_period_m) *
                                                        (cp.ambient + (1.0 - cp.ambient) * lambert) +
                                                     noise(rng)));
      const uchar value = static_cast<uchar>(std::lround(gray * 255.0));
      const int v1 = std::min(k.height, v0 + b);
      const int u1 = std::min(k.width, u0 + b);
      for (int v = v0; v < v1; ++v) {
        cv::Vec3b* row = img.ptr<cv::Vec3b>(v);
        for (int u = u0; u < u1; ++u) {
          row[u] = cv::Vec3b(value, value, value);
        }
      }
    }
  }
  return img;
}

std::array<double, 6> pose_to_log(const Eigen::Isometry3d& T) {
  const Eigen::AngleAxisd aa(T.linear());
  const Eigen::Vector3d w = aa.angle() * aa.axis();
  return {T.translation().x(), T.translation().y(), T.translation().z(), w[0], w[1], w[2]};
}

Eigen::Isometry3d log_to_pose(const std::array<double, 6>& v) {
  const Eigen::Vector3d w(v[3], v[4], v[5]);
  const double th = w.norm();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (th > kEps) {
    T.linear() = Eigen::AngleAxisd(th, w / th).toRotationMatrix();
  }
  T.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
  return T;
}

}  // namespace synth
}  // namespace follow
