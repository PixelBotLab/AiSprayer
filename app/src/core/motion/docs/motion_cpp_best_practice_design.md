# Motion 模块 C++ 重构设计方案（最佳实践版）

> 本文与 `motion/docs/motion_cpp_refactor_design.md`（下称"原方案"）是**并列的两份方案**，用于对比选型。
> 本文的所有结论都基于对现有 Python/C++ 代码的逐行核查，第 2 节列出了核查中发现的、会影响架构决策的事实偏差。

---

## 0. 结论先行

| 决策点 | 原方案 | 本方案 | 为什么 |
| :--- | :--- | :--- | :--- |
| **交付产物** | 单一 `motion_cli` 可执行文件 | **`libmotion_core.a` + `motion_cli` + `libmotion_c.so`** 三产物 | `app/src/apps/follow` 在**实时线程**里调 FK/IK（见 §2.1）。单点 IK 约 0.4µs，进程冷启动约 3~5ms，走 CLI 等于慢 1 万倍。批处理走 CLI，实时走进程内 `.so` |
| **抽象接口数量** | 5 个纯虚接口 + Builder 模式 | **1 个纯虚接口**（`IRobotKinematics`），其余用值语义具体类 | 目前只有 1 种机器人、1 个插值器、1 个验证器、1 个优化器。为不存在的第二实现造抽象，与"简洁易懂"直接冲突 |
| **锚点模式** | `IAnchorStrategy` 策略模式（3 个子类） | **一个 `Anchor` 值对象**（`optional<Matrix3d> R` + `Vector3d tol`） | 4 种模式（config/home/raw/live）的差异只在"R 从哪来"，属于**输入解析**，不是运行期行为多态（见 §5.3） |
| **配置构造** | `OptimizationOptionsBuilder` 链式调用 | **聚合初始化 + `validate()`** | 纯数据结构上套 Builder 只增加代码量，不增加安全性 |
| **智能指针** | 大量 `shared_ptr` 成员 | **值语义 / `const&`**，仅求解器用 `unique_ptr` | 单进程、所有权明确、无共享生命周期需求 |
| **性能目标** | 优化 5~15ms、验证 <2ms | **优化 100~500ms（单线程）/ 30~80ms（8 线程）；验证 <10ms**，且先测基线再定 | 原目标与算法量级不符，按默认参数估算差 1~2 个数量级（见 §9） |
| **正确性保障** | 未涉及 | **黄金数据回归 + 影子双跑**，作为一等公民 | 这是本次重构**最大的风险**，也是原方案最大的缺口（见 §8） |
| **DH 参数来源** | `RobotModelConfig.load_from_urdf` 解析 | **DH 硬编码在解析解里，URDF 只提供限位与 TCP** | 原方案会误导实现者去写一个不该存在的 DH 解析器（见 §2.2） |
| **stdout 协议** | 单行 JSON | 单行 JSON，且**强制** stdout 只有 JSON、日志与进度全走 stderr | C++ 侧任何 `std::cout` 调试输出都会让 Python 的 `json.loads` 崩掉 |

---

## 1. 目标与范围

### 1.1 迁移范围（按现有代码实测规模）

| 现有 Python 实现 | 行数 | 迁移目标 |
| :--- | :--- | :--- |
| `verification/path_opt.py`（`SprayWaypointOptimizer`，Viterbi 主体） | 1148 | `motion/src/optimize/` |
| `verification/kinematic_chain_verifier.py`（密采样验证） | ~400 | `motion/src/verify/` |
| `verification/path_interpolator.py`（MoveL + Slerp） | ~130 | `motion/src/core/interpolator.*` |
| `verification/robot_config.py`（URDF 限位/TCP 解析） | ~190 | `motion/src/io/robot_config.*` |
| `verification/poi_optimizer.py`（锚点模式路由 + 报告） | ~460 | 拆分：锚点解析入 CLI 层，报告入 `io/` |
| `cr5_kinematics.py` + `cr5_ur_kin.py`（FK/IK/奇异点） | 734 + ~190 | `motion/src/core/kinematics/`（大部分已有 C++ 对应） |
| `cr5_kinematics_cpp/*.cpp`（现有 C++ 热路径） | ~700 | **平移复用**，不重写 |
| `path_opt_cli.py`（CLI + ASCII 报告） | ~600 | `motion/src/cli/` |

### 1.2 非目标（明确不做）

* **不迁移** `AxialSpinOptimizer`（`verification/axial_optimizer.py`）。核查确认它未被任何 Web 路由调用，属死代码，迁移前先确认删除。
* **不迁移**驱动层（`dobot_driver.py` / `inexbot_driver.py` / `dobot_api.py`），它们是通信协议实现，与运动学无关。
* **不追求** Python 侧零运动学代码。Python 保留一层薄绑定，既服务 `follow` 实时链路，也作为回归比对的 oracle（§8）。

---

## 2. 现状事实核查（对原方案的修正）

以下 6 点均经代码核查确认，直接影响架构决策。

### 2.1 `follow` 模块在实时线程里使用运动学 —— 纯 CLI 方案会打断这条链路

`app/src/apps/follow/services/follow_service.py:61-66` 明确记录了这个约束：

```61:66:app/src/apps/follow/services/follow_service.py
    def __init__(self) -> None:
        self._lock = threading.RLock()
        # CR5Kinematics 的 cpp 后端用 per-instance ctypes 缓冲（源码注释：one solver per
        # thread）。路由线程要 FK、轮询线程要 IK ⇒ 必须串行化，否则两边共用同一块缓冲。
        self._kin_lock = threading.Lock()
        self._kin = None
```

`follow/mirror.py:154` 在跟随回路中逐帧调用 `kin.get_best_ik(...)`。这类调用**必须**在进程内完成。

**决策**：`motion/` 输出**三个**产物，共享同一份 core 静态库：

```
libmotion_core.a   ← 所有算法，单一真源，无 IO 无 CLI
   ├── motion_cli        （可执行）批处理：verify / optimize / fk / ik
   └── libmotion_c.so    （共享库）稳定 C ABI，供 Python ctypes 实时调用
```

`libmotion_c.so` 直接**取代** `cr5_kinematics_cpp/libur_kin.so`，导出兼容的 C 符号，`cr5_kinematics.py` 只需改库查找路径。这样"一份 C++ 真源"的目标达成，且 `follow` 不受影响。

### 2.2 DH 参数是硬编码的，URDF 只提供限位和 TCP

原方案 `RobotModelConfig.load_from_urdf(path, tool_name)` 暗示从 URDF 解析运动学模型，实际并非如此：

* DH 参数硬编码两份：`cr5_ur_kin.py:29-35` 与 `cr5_ur_kin.cpp:49-57`（`CR5_PARAMS`），值为 `d1=0.147, a2=-0.427, a3=-0.357, d4=0.141, d5=0.116, d6=0.105`（米）。
* URDF 只被 `verification/robot_config.py` 用来读**关节限位**（`<limit lower/upper/velocity>`）和 **TCP 偏置**（parent 为 Link6 的 fixed joint）。

**决策**：`RobotModel` 明确分成两部分 —— `DhParams`（编译期常量，`constexpr`）与 `RobotLimits`/`ToolOffset`（运行期从 URDF 读）。绝不写"从 URDF 推导 DH"的代码。

### 2.3 Web 层的真实痛点不是"多进程"，而是"验证同步阻塞主进程"

原方案称 Python 层普遍采用 `multiprocessing.Process + Pipe`。核查结果更细：

* **POI 优化**确实用了子进程：`path_verification_service.py:269-295`（`spawn` 上下文 + `Pipe(duplex=False)` + `Queue` 日志 + `terminate()` 超时强杀）。
* **验证没有**：`api.py:422-437` 是 `def` 而非 `async def`，`verify_template_paths`（`path_verification_service.py:349-406`）**在 uvicorn worker 主进程内同步跑完**。密采样上万步期间整个 Web 服务无响应。

这反而**强化**了 CLI 方案的价值：验证也应该出进程。同时说明 §7 的 Python 适配器必须有 `timeout` 和崩溃处理 —— 现有子进程代码在子进程段错误时 `parent_conn.recv()` 会永久阻塞（`path_verification_service.py:278-279`）。

### 2.4 Viterbi 主体还在 Python，C++ 只加速了 3 个热路径

原方案"复用并扩展当前高效的 C++ `walk_movel_controller`"低估了工作量。`cr5_path_opt.cpp` 只有 3 个导出函数：`get_best_ik`(149-200)、`walk_movel`(202-296)、`inverse_batch`(298-302)。

**留在 Python、必须新写 C++ 的**：姿态网格预计算、锚点包络投影、四元数去重、8 分支分桶 top-K、beam 截断、自适应全量回退、DP 主循环与回溯、dense_verify 门控。这是本次重构的**主要工作量**，路线图必须据此排期（§10）。

### 2.5 锚点有 4 种模式，且 `live` 是 `config` 的运行期变体

| 模式 | 入口 | R_anchor 来源 |
| :--- | :--- | :--- |
| `home` | `path_opt_cli.py:172-175` | Home 关节 `[0,0,-90,-90,-90,0]` 做 FK |
| `config` | `path_verification_service.py:454-467` | 配置项 `spraying.poi_ref_rpy_deg` |
| `raw` | `poi_optimizer.py:364-366` | 无全局锚点，逐点用自身名义姿态 |
| `live` | `api.py:444-453` | 读机械臂当前 TCP 的 RPY，**随后把 `anchor_source` 改写成 `config`** |

投影算法是**相对欧拉盒裁剪**，不是球面测地投影（`path_opt.py:400-414`）：`R_rel = R_anchorᵀ·R_cand` → 转 euler-xyz → wrap 到 [-180,180) → 逐轴 clip 到 ±tol → `R_anchor · R(clipped)`。这个细节必须原样保留，否则结果不可比对。

### 2.6 C++ 侧依赖已经全部就位

`system_deps.sh:10` 已安装 `libeigen3-dev`、`libyaml-cpp-dev`、`libcli11-dev`。板端实测确认：

```
/usr/include/eigen3, /usr/include/yaml-cpp/yaml.h, /usr/include/CLI/CLI.hpp 均存在
cmake 3.28.3, g++ 13.3.0 (aarch64), nproc=8, Python 3.12.3
```

无需 vendored 依赖，无需改 `system_deps.sh`。`nproc=8` 是 §9 并行化目标的依据。

---

## 3. 三条设计原则

本方案刻意用"少抽象"换"易懂"，与原方案"严格 SOLID"是明确的取舍分歧。

**原则一：抽象必须由第二个实现来证明。**
只在真实存在第二实现或真实需要替换时才引入接口。当前唯一满足的是 `IRobotKinematics`（Python/C++ 双后端已存在，且未来有 Nova/UR 需求）。`ITrajectoryInterpolator`、`IKinematicVerifier`、`IPathOptimizer`、`IAnchorStrategy` 四个接口目前各只有一个实现，引入它们等于用间接层换取零收益，并让"跳转到定义"永远落在纯虚函数上。

**原则二：数据是数据，行为是行为。**
`Waypoint`、`Anchor`、`OptimizeOptions`、`VerifyReport` 都是纯聚合体（`struct` + 公开成员 + `validate()`），可拷贝、可比较、可直接序列化。算法类持有配置的 `const&`，不持有所有权。

**原则三：单位与坐标系约定进类型系统，不进注释。**
这是现有代码最贵的错误来源（`follow/mirror.py:11-15` 有一段代价惨重的记录："矩阵看着完全正常、IK 却一个解都没有"）。见 §6。

---

## 4. 目录结构

放在**仓库根级 `motion/`**，与 `follow/`、`third_party/` 同级 —— 它是独立的 C++ 子项目，与 `follow/` 现有约定一致（`follow/CMakeLists.txt` 的 RK3588 编译参数可直接复用）。

> 顺带清理：`app/src/core/motion/{docs,src}` 是空目录，应删除，避免两个 motion 目录并存造成困惑。

```
motion/
├── CMakeLists.txt              # C++17, Eigen3, yaml-cpp, CLI11；复用 follow/ 的 RK3588 flags
├── build.sh                    # 一键构建 + 安装 libmotion_c.so 到 Python 可见路径
├── docs/
│   ├── motion_cpp_refactor_design.md        # 原方案
│   └── motion_cpp_best_practice_design.md   # 本文
├── include/motion/             # 对外头文件（tests / cli / c_api 共用）
│   ├── conventions.hpp         # ★ 单位与坐标系契约，全模块第一个 include
│   ├── types.hpp               # Waypoint / Pose / Anchor / JointVec
│   ├── robot_model.hpp         # DhParams(constexpr) + RobotLimits + ToolOffset
│   ├── kinematics.hpp          # IRobotKinematics + Cr5Kinematics
│   ├── interpolator.hpp        # MoveL + Slerp 稠密插值
│   ├── verifier.hpp            # ChainVerifier + VerifyReport
│   ├── optimizer.hpp           # ViterbiOptimizer + OptimizeOptions
│   └── report.hpp              # Issue / Severity / 汇总结构
├── src/
│   ├── core/                   # 无 IO 依赖的纯算法（→ libmotion_core.a）
│   │   ├── kinematics/{dh_solver.cpp, cr5_kinematics.cpp, singularity.cpp}
│   │   ├── interpolator.cpp
│   │   ├── verifier.cpp
│   │   └── optimize/{candidates.cpp, edge_cost.cpp, viterbi.cpp}
│   ├── io/                     # YAML/JSON/URDF，隔离在 core 之外
│   │   ├── robot_config.cpp    # URDF 限位 + TCP（tinyxml2 或 yaml-cpp 同源方案）
│   │   ├── path_yaml.cpp       # scan.*.path.yaml 读写
│   │   └── json_report.cpp     # stdout 单行 JSON
│   ├── cli/
│   │   ├── main.cpp            # CLI11 子命令分发，只做参数→Options 的翻译
│   │   ├── cmd_verify.cpp
│   │   ├── cmd_optimize.cpp
│   │   ├── cmd_kin.cpp         # fk / ik，供调试与脚本
│   │   └── anchor_resolve.cpp  # ★ 4 种锚点模式在这里塌缩成一个 Anchor 值
│   └── c_api/
│       └── motion_c.cpp        # → libmotion_c.so，兼容现有 libur_kin 符号
└── tests/
    ├── unit/{test_kinematics.cpp, test_interpolator.cpp, test_verifier.cpp, test_viterbi.cpp}
    ├── golden/                 # ★ 从 Python 实现导出的黄金数据（§8）
    │   ├── cases/*.yaml
    │   └── expect/*.json
    └── test_golden_regression.cpp
```

**分层只有 3 层，依赖单向**：`core`（纯算法，可单测，不含 IO） ← `io` ← `cli` / `c_api`。原方案的 5 层被压成 3 层，因为"领域模型层"是头文件而非层，"运动学层/轨迹层/优化层"在依赖上是同级的。

---

## 5. 核心接口设计

### 5.1 约定契约（`conventions.hpp`）

**这是全模块最重要的一个文件**，把 §2 核查出的所有换算陷阱固定下来。

```cpp
namespace motion {

// ── 单位约定（不可协商）────────────────────────────────────────────────
//  内部计算：长度 米(m)，角度 弧度(rad)
//  外部边界：YAML/JSON/控制器 报文 长度 毫米(mm)，角度 度(deg)
//  换算只允许发生在 io/ 与 c_api/ 层，core/ 内部出现 1000.0 视为 bug。
inline constexpr double kMmPerM = 1000.0;

// ── 姿态约定 ─────────────────────────────────────────────────────────
//  Dobot 控制器报文 [rx, ry, rz](deg) 满足 R = Rz(rz)·Ry(ry)·Rx(rx)
//  等价于 scipy Rotation.from_euler("xyz", ...)（小写=外旋），
//  也等价于内旋 ZYX。全仓库只此一套，已在
//  apps/calib/services/hand_eye/geometry.py:29-35 实测确认 (<1e-15)。
Eigen::Matrix3d RotFromCtrlRpyDeg(const Eigen::Vector3d& rpy_deg);
Eigen::Vector3d CtrlRpyDegFromRot(const Eigen::Matrix3d& R);

// ── URDF 关节 ↔ DH 关节 ──────────────────────────────────────────────
//  q_dh[1] = q_urdf[1] - π/2 ,  q_dh[3] = q_urdf[3] - π/2
//  逆向 +π/2 后必须 wrap 到 [-π, π]；解析 IK 输出域是 [0, 2π)。
//  多圈限位（J1/J6 可达 ±2π）需要 ExpandMultiTurn() 展开别名解。
JointVec UrdfToDh(const JointVec& q_urdf);
JointVec DhToUrdf(const JointVec& q_dh);

// ── 业务硬约束常量（原散落各处，集中于此）──────────────────────────────
inline constexpr double kSingularityDeg   = 3.0;   // |sin(q3)|,|sin(q5)| 阈值
inline constexpr double kShoulderSepDeg   = 6.0;   // 两 q1 支分离角下限
inline constexpr double kBranchJumpDeg    = 45.0;  // IK 分支跳变判定
inline constexpr double kEndBranchMatchDeg = 5.0;  // MoveL 终点同支容差
}  // namespace motion
```

配套两条**编译期/启动期断言**，防止约定被静默改坏：

* 单测 `test_kinematics.cpp` 断言 `CtrlRpyDegFromRot(RotFromCtrlRpyDeg(x)) == x` 在随机姿态下 <1e-12；
* 单测断言 C++ FK 与 Python `forward_controller` 在同一组关节角上位置差 <0.05mm（沿用 `test_follow_mirror.py:42` 的既有档位）。

### 5.2 类型（`types.hpp`）

```cpp
namespace motion {

using JointVec = Eigen::Matrix<double, 6, 1>;   // rad
using Transform = Eigen::Isometry3d;            // 米制齐次变换

// 航点：与 YAML 字段一一对应，无行为
struct Waypoint {
  int index = 0;
  Eigen::Vector2i pixel{0, 0};
  Eigen::Vector3d surface_point_m{0, 0, 0};
  Eigen::Vector3d surface_normal{0, 0, 1};
  Transform tcp_pose = Transform::Identity();   // 控制器帧，米
  double standoff_m = 0.15;
  bool spraying = true;
  bool is_jump = false;
};

// 一条路径
struct PathItem {
  int path_id = 0;
  std::string name;
  std::vector<Waypoint> points;
};
}  // namespace motion
```

用 `Eigen::Isometry3d` 而非原方案的"`Vector3d position_mm` + `Quaterniond`"自定义 `Pose3D`：Isometry3d 自带乘法、求逆、`.linear()`/`.translation()`，省掉一整套手写转换代码和随之而来的 bug。位置单位统一为米，字段名带 `_m` 后缀。

### 5.3 锚点：值对象而非策略模式

这是与原方案最实质的设计分歧。原方案为 `home`/`config`/`raw` 三种模式建了 `IAnchorStrategy` + 3 个子类。但核查（§2.5）表明，三种模式的差别**只是 `R_anchor` 从哪里算出来**，算完之后优化器的行为完全一致。把它做成运行期多态，等于让优化器持有一个只会被调用一次的多态对象。

```cpp
// include/motion/types.hpp
struct Anchor {
  // nullopt = raw 模式：无全局锚点，逐点用自身名义姿态做包络中心
  std::optional<Eigen::Matrix3d> R;
  Eigen::Vector3d tol_deg{10.0, 10.0, 180.0};

  bool has_global() const { return R.has_value(); }
};

// src/cli/anchor_resolve.cpp —— 4 种模式在 CLI 层塌缩成一个值
Anchor ResolveAnchor(const AnchorSpec& spec, const Cr5Kinematics& kin);
//   home   → kin.Fk(home_joints).linear()
//   config → RotFromCtrlRpyDeg(spec.ref_rpy_deg)
//   live   → 同 config（RPY 由 Python 侧从机械臂读好后传入 --ref-rpy）
//   raw    → std::nullopt
```

优化器只看到 `const Anchor&`。新增第五种模式只需在 `ResolveAnchor` 里加一个分支 —— 一个 `switch` 分支比一个新类 + 注册 + 工厂更易懂，且改动面更小。

### 5.4 唯一的纯虚接口（`kinematics.hpp`）

```cpp
namespace motion {

// 唯一值得抽象的接口：Python/C++ 双后端已存在，且 Nova/UR 是明确的路线图需求。
class IRobotKinematics {
 public:
  virtual ~IRobotKinematics() = default;

  virtual Transform Fk(const JointVec& q_urdf) const = 0;
  virtual int Ik(const Transform& T_urdf, JointVec* out_sols) const = 0;  // 最多 8 解

  // 以 q_seed 为种子选最近无翻转分支；无解返回 nullopt
  virtual std::optional<JointVec> BestIk(const Transform& T_urdf,
                                         const JointVec& q_seed) const = 0;

  virtual const RobotLimits& limits() const = 0;
};

class Cr5Kinematics final : public IRobotKinematics {
 public:
  explicit Cr5Kinematics(RobotLimits limits);
  // ... 覆写上述 4 个方法

  // CR5 专有、不进接口的能力（放进接口会污染其他机型）
  SingularityFlags CheckSingularity(const JointVec& q, const Transform& T) const;
  int IkBatch(const Transform* T, int n, JointVec* out, int* n_sols) const;
  std::optional<MoveLWalk> WalkMoveL(const MoveLQuery& q) const;
};
}  // namespace motion
```

注意 `CheckSingularity` / `IkBatch` / `WalkMoveL` **刻意不放进接口**：它们的语义（肩/肘/腕三类奇异、8 解布局）是 6R 球腕解析解特有的，写进通用接口会在接入其他机型时立刻变成负担。这是对原方案 `IRobotKinematics` 包含 `check_singularity` 的一处修正。

### 5.5 验证器与优化器（具体类，值语义）

```cpp
// verifier.hpp
class ChainVerifier {
 public:
  ChainVerifier(const IRobotKinematics& kin, const ToolOffset& tool,
                VerifyOptions opt);
  VerifyReport Verify(const PathItem& path, std::optional<JointVec> init_q) const;

 private:
  const IRobotKinematics& kin_;   // 不拥有：生命周期由调用方保证，无需 shared_ptr
  const ToolOffset& tool_;
  VerifyOptions opt_;
  Interpolator interp_;           // 值成员：无状态、可拷贝，无需接口
};

// optimizer.hpp
struct OptimizeOptions {
  AxisGrid grid_x{-5, 5, 2};      // deg (min, max, step)
  AxisGrid grid_y{-5, 5, 2};
  AxisGrid grid_z{-180, 180, 10};
  int beam_width = 32;
  int max_candidates_per_branch = 16;
  int movel_checks_min = 10, movel_checks_max = 100;
  double movel_spacing_mm = 5.0;
  double verify_step_mm = 1.5;
  double verify_speed_mm_s = 120.0;
  Eigen::Vector3d weight_zero_dev{1.0, 1.0, 0.01};        // 倾角贵、自旋便宜
  JointVec joint_weights{1.0, 1.2, 1.0, 0.8, 0.8, 0.5};
  bool dense_verify = true;
  int threads = 0;                                         // 0 = 硬件并发数

  std::string Validate() const;   // 返回空串=合法；非空=错误描述
};

class ViterbiOptimizer {
 public:
  ViterbiOptimizer(const Cr5Kinematics& kin, const ToolOffset& tool,
                   OptimizeOptions opt);
  OptimizeResult Optimize(const PathItem& path, const Anchor& anchor) const;
};
```

**默认值全部取自现有 Python 实现的实测默认**（`path_opt_cli.py:105-128`、`path_opt.py:439,491-493`），保证行为等价的起点一致。

对比原方案的 Builder：

```cpp
// 原方案（12 行，需维护 10 个 setter + build() + 校验分散）
OptimizationOptions o = OptimizationOptionsBuilder()
    .set_tool_grid_x(-5.0, 5.0, 2.0).set_beam_width(32) /* ... */ .build();

// 本方案（改哪个写哪个，其余走默认；校验集中在一处）
OptimizeOptions o;                       // 全部默认 = 与 Python 现状等价
o.beam_width = 64;
if (auto err = o.Validate(); !err.empty()) return Fail(err);
```

---

## 6. 稠密轨迹与 IO 策略

**这是原方案未涉及、但会直接决定实际端到端耗时的一项。**

核查发现 `data/template_group/2026-09-03_225937/scan.auto.poi.path.yaml` 中 `verification.trajectory_q` 有约 **5178 步 × 6 关节**，加上 `trajectory_tcp` 合计约 1.3 万行 YAML。yaml-cpp 的 emitter 在这个量级上会成为**主要耗时项**，可能远超算法本身。

**决策**：

1. `scan.*.path.yaml` **只写摘要与结论**：`status`、`summary`、`issues[]`、`peak_joint_speeds_deg_s`、`recommended_safe_speed_mm_s`、`spray_opt_joints_deg`。
2. 稠密轨迹写**侧车文件** `scan.*.traj.npy`（或 `.f64.bin` + 头部维度），由 `--dump-trajectory <path>` 显式开启，默认关闭。Python 侧 `np.load` 零解析成本，前端需要时按需拉取。
3. 保留一个 `--legacy-inline-trajectory` 开关，在过渡期写回旧的内联格式，保证前端不改也能跑（`InteractiveOp.tsx` 现在读 YAML 内的 `verification`）。

必须保持不变的输入契约：`paths[].points[].tcp_pose_base`（mm/deg，euler-xyz）、`spraying`/`is_jump` 语义、`poi_config` 块、`standoff_distance_mm`。文件名映射沿用 `path_verification_service.py:19-28` 的 canonical/legacy 双轨规则。

---

## 7. CLI 协议与 Python 集成

### 7.1 三条铁律

1. **stdout 只有一行 JSON**，成功失败都是。`motion/src/cli/main.cpp` 里除报告输出外禁止 `std::cout`；日志、进度、ASCII 表格全部走 **stderr**。原方案未作此约束，而 C++ 侧任何一句调试打印都会让 Python 的 `json.loads` 抛异常 —— 这是最容易踩且最难查的集成 bug。
2. **退出码有语义**：`0` 成功；`2` 参数错误；`3` 输入文件/URDF 无法解析；`4` 算法失败（无可行解）；`>=128` 信号致死。Python 据此区分"该重试"和"该报错"。
3. **长任务有进度**：优化可能到秒级，`--progress` 时向 **stderr** 逐行输出 NDJSON（`{"stage":"dp","done":37,"total":63}`），Python 侧可流式读取喂给前端，无需另建 IPC。

### 7.2 子命令

```bash
# 验证
motion_cli verify --input scan.auto.path.yaml --urdf app/urdf/cr5_robot_with_my_tools.urdf \
    --tool-tcp gripper_tip_link --speed 120 --step 1.5 [--path-id N] [--dump-trajectory F]

# 优化（锚点 4 模式；live 由 Python 读臂后以 config+--ref-rpy 形式传入）
motion_cli optimize --input scan.auto.path.yaml --output scan.auto.poi.path.yaml \
    --urdf ... --tool-tcp ... --anchor-source home|config|raw --ref-rpy rx,ry,rz \
    --anchor-tol 10,10,180 --grid-x -5,5,2 --grid-y -5,5,2 --grid-z -180,180,10 \
    --beam-width 32 --threads 0 [--no-dense-verify] [--progress]

# 运动学调试（黄金数据比对与人工排查用）
motion_cli fk --joints 0,0,-90,-90,-90,0
motion_cli ik --pose x,y,z,rx,ry,rz --seed 0,0,-90,-90,-90,0
```

### 7.3 Python 适配器（补齐原方案缺失的健壮性）

原方案的 `MotionCliClient` 用 `subprocess.run(..., check=True)`，缺 `timeout`、缺 stderr 处理、崩溃时 `json.loads` 会二次抛错并丢失真实原因。补齐版：

```python
class MotionCliError(RuntimeError):
    def __init__(self, msg, *, exit_code=None, stderr=""):
        super().__init__(msg)
        self.exit_code, self.stderr = exit_code, stderr


class MotionCliClient:
    """motion_cli 的同步封装。stdout 恒为单行 JSON，stderr 为日志/进度。"""

    def __init__(self, cli_bin: str, default_timeout: float = 120.0):
        self._bin, self._timeout = cli_bin, default_timeout

    def _run(self, args: list[str], timeout: float | None) -> dict:
        try:
            p = subprocess.run([self._bin, *args], capture_output=True, text=True,
                               timeout=timeout or self._timeout)
        except subprocess.TimeoutExpired as e:
            raise MotionCliError(f"motion_cli 超时: {e.timeout}s") from e

        tail = "\n".join(p.stderr.strip().splitlines()[-20:])   # 只留尾部，避免日志刷屏
        if p.returncode != 0:
            # 算法失败(4) 仍会输出合法 JSON；段错误等则不会
            try:
                return {**json.loads(p.stdout), "exit_code": p.returncode}
            except (json.JSONDecodeError, ValueError):
                raise MotionCliError(f"motion_cli 异常退出 ({p.returncode})",
                                     exit_code=p.returncode, stderr=tail)
        try:
            return json.loads(p.stdout)
        except json.JSONDecodeError as e:
            raise MotionCliError(f"stdout 非合法 JSON: {p.stdout[:200]!r}",
                                 stderr=tail) from e
```

用它替换 `path_verification_service.py:141-306` 的 165 行子进程管理代码（`spawn` 上下文、日志 `Queue`、抽日志线程、`Pipe` 死锁防护、`terminate()` 强杀）。**净减约 150 行 Python，且验证路径顺带获得进程隔离**（§2.3）。

FastAPI 路由改为 `async def` + `run_in_executor`（或 `anyio.to_thread`），彻底解除主进程阻塞。

---

## 8. 正确性保障：黄金数据回归 + 影子双跑

**原方案完全没有这一节，而这是重构失败的头号原因。** Viterbi DP 涉及大量浮点比较与并列取舍，`std::sort` 的不稳定性、`std::priority_queue` 的入队顺序、`-ffast-math`、多线程归约顺序，任何一处都会让 DP 选到**代价相同但姿态不同**的另一条路径。结果"看起来对、但和 Python 不一样"，无法判断是改进还是回归。

### 8.1 黄金数据集（阶段 0 就要做完）

在动手写 C++ **之前**，先用现有 Python 实现导出黄金数据，存入 `motion/tests/golden/`：

| 层级 | 用例 | 断言容差 |
| :--- | :--- | :--- |
| FK/IK | 10k 组随机关节角 → 位姿；含奇异附近与限位边界 | 位置 <1e-9 m，姿态 <1e-9 rad |
| 插值 | 3 条真实路径的稠密序列（含 `is_jump` 段） | 逐步 <1e-9 |
| 候选生成 | 单航点全网格候选集（分支 id、zero_dev、geo_deg） | 集合相等，代价 <1e-9 |
| 边代价 | 200 个真实相邻航点对的 `walk_movel` 结果 | `cost` <1e-6，`q_end` <1e-9 |
| 验证报告 | `data/template_group/` 下所有现存模板 | `issues[]` 类型/severity/索引**完全一致** |
| 优化结果 | 同上 | 见 §8.2 分级判定 |

### 8.2 优化结果的分级判定

不要求逐位相等（不现实），改用三级判定：

1. **总代价** `total_cost` 相对差 <1e-6 —— 必须通过。
2. **每点姿态**与 Python 结果的测地角差 <1e-6° —— 若不过，进入第 3 级。
3. **代价并列**：若姿态不同但总代价差 <1e-9，判定为**合法并列**，记录到 `tie_report` 供人工确认，不算失败。同时要求 C++ 侧的 tie-breaking 规则**显式确定化**（按 `(cost, branch_id, pose_idx)` 字典序），使结果**可复现**。

配套约束：

* **禁用** `-ffast-math` / `-Ofast`（只用 `-O3`），禁止改变浮点语义的优化。
* 并行化只允许在**无归约顺序依赖**的维度上做（候选生成按航点并行、边评估按边并行后按固定索引写回），DP 主循环串行。
* 单测锁定：同一输入连续跑 10 次，结果必须逐位一致（防止多线程引入不确定性）。

### 8.3 影子双跑（上线过渡期）

`path_verification_service` 加配置开关 `motion.engine: python | cpp | shadow`：

* `python`：走现有实现（回退路径，全程保留）；
* `cpp`：走 `motion_cli`；
* `shadow`：两者都跑，返回 Python 结果，把差异写日志。

在真实作业上跑满一到两周 `shadow`、差异清零后再切 `cpp`。这条路径要求 **Python 实现在整个阶段 1~5 中保持可用**，因此 §1.2 明确"不追求 Python 侧零运动学代码"。

---

## 9. 性能：先测基线，再定分级目标

### 9.1 为什么原方案的目标不可信

原方案称"整条轨迹优化控制在 5~15ms 以内，验证耗时 <2ms"。按默认参数与实测 IK 吞吐（`path_opt_cli.py:169` 记录 C++ IK 约 2.3 MHz，即约 0.43µs/次）做量级估算：

* **候选生成**：网格 `6 × 6 × 37 = 1332` 个偏移旋转/航点，每个批量 IK（8 解）→ 约 1332 次 IK ≈ 0.6ms/航点。64 航点 ≈ **约 40ms**，已经超出原方案上限。
* **边评估**：beam 32 × 每层候选上限 8×16=128 → 约 4096 条边/段；每条边 MoveL 抽检 10~100 步、每步一次 `BestIk`。取 20 步 → 约 8.2 万次 IK ≈ 35ms/段。63 段 ≈ **约 2.2s**。O(1) 预筛（肘腕家族 + 行程上限，`path_opt.py:835-863`）能砍掉大部分边，乐观按 5~10 倍计 → **约 220~440ms**。

所以 5~15ms 与算法量级差 1~2 个数量级。按此目标验收，只会导致为凑数字而砍精度。

### 9.2 分级目标（64 航点典型路径，RK3588）

| 阶段 | 目标 | 手段 |
| :--- | :--- | :--- |
| 基线（先测） | 记录 Python 现状实测值 | `path_opt.py:933-939,1032-1040` 已有计时点，直接采集 |
| C++ 单线程 | **100~500ms** | 消除 Python 解释器与 ctypes 边界开销；候选/边数据结构改为连续内存（SoA） |
| C++ 8 线程 | **30~80ms** | 候选生成按航点并行、边评估按边并行（DP 主循环保持串行，见 §8.2） |
| 验证 | **算法 <10ms**（5178 步）；端到端受 IO 支配 | 5178 次 IK ≈ 2.2ms + 插值；故 §6 的侧车文件策略是端到端提速的关键 |

以上均为**待验证的估算**，阶段 1 完成后用真实数据修正本表。任何性能数字都应带上"在哪台机器、哪条路径、哪组参数"。

---

## 10. 实施路线图

每个阶段都以"可与 Python 对比的产物"结束，任一阶段失败都能停在上一阶段而不影响生产（因为 `motion.engine` 默认仍是 `python`）。

| 阶段 | 任务 | 验收标准 |
| :--- | :--- | :--- |
| **0. 黄金数据** | 用现有 Python 实现导出 §8.1 全部黄金数据；搭 CMake 骨架与 `conventions.hpp`；确认删除 `axial_optimizer.py` 与空目录 `app/src/core/motion/` | `motion/tests/golden/` 就位；`cmake --build` 通过；采集到性能基线 |
| **1. 运动学 + C ABI** | 平移 `cr5_ur_kin.cpp`/`cr5_kinematics.cpp`/`cr5_path_opt.cpp` 进 `core/kinematics`；输出 `libmotion_c.so` 取代 `libur_kin.so`；URDF 限位/TCP 解析 | FK/IK 黄金用例全绿；`follow` 与 `cr5_kinematics.py` 换库后所有现有 Python 测试通过（含 `test_follow_mirror.py`、`test_cr5_kinematics.py`） |
| **2. 插值 + 验证 + CLI 骨架** | `Interpolator`、`ChainVerifier`、`path_yaml.cpp`、`json_report.cpp`、`motion_cli verify` | 全部现存模板的 `issues[]` 与 Python 逐条一致；stdout 是合法单行 JSON；日志确认全在 stderr |
| **3. 优化器** | 候选生成 → 8 桶 top-K → 边代价 → beam → DP → 回溯 → 自适应回退 → dense_verify 门控 | §8.2 三级判定通过；确定性单测（10 次逐位一致）通过；单线程性能达标 |
| **4. Web 接入（shadow）** | `MotionCliClient` + `motion.engine` 开关；路由改 `async def` + executor；进度流接前端 | `shadow` 模式在真实作业上零差异；删除 `path_verification_service.py:141-306` 的子进程代码 |
| **5. 并行化与收尾** | 按 §9.2 并行；侧车轨迹文件；切 `engine: cpp`；Python 算法实现降级为 oracle（仅测试引用） | 8 线程性能达标；前端交互无卡顿；`shadow` 观察期结束 |

阶段 1 是**风险最高**的一步（要动 `follow` 实时链路依赖的库），但也是收益最确定的一步；建议单独出一个 PR 并保留 `libur_kin.so` 一个版本周期作为回退。

---

## 11. 风险清单

| 风险 | 影响 | 应对 |
| :--- | :--- | :--- |
| DP 并列取舍导致结果与 Python 不同 | 无法判断改进/回归，最难排查 | §8.2 确定化 tie-breaking + 分级判定 + 禁 fast-math |
| 阶段 1 换库打断 `follow` 实时链路 | 跟随功能失效 | C ABI 保持符号兼容；保留旧 `.so` 一个周期；先跑 `test_follow_mirror.py` |
| 欧拉/单位约定实现错 | "矩阵看着对、IK 无解"（`follow/mirror.py:11-15` 已有先例） | `conventions.hpp` 单一出处 + 往返单测 |
| yaml-cpp emit 上万行成为新瓶颈 | 端到端没有变快，重构收益被吃掉 | §6 侧车文件；阶段 2 就测 IO 耗时占比 |
| 迁移工作量被低估（DP 主体全新写） | 排期失控 | §2.4 已重新界定范围；阶段 3 单独排期 |
| 抽象过度导致后续难改 | 违背"简洁易懂"初衷 | §3 原则一：接口必须由第二个实现来证明 |
| Python oracle 被提前删除 | 失去比对基准，回归无从判定 | 阶段 5 之前禁止删除；`shadow` 观察期结束才降级 |

---

## 12. 原方案中应当保留的部分

本文对原方案的抽象层级、性能目标和事实描述提出了修正，但以下判断是正确的，本方案直接沿用：

1. **CLI + JSON 取代 `multiprocessing.Process + Pipe`** —— 方向完全正确，且核查发现收益比原方案说的更大（验证路径当前根本没有隔离，§2.3）。
2. **崩溃隔离与内存回收由 OS 兜底** —— 长期运行的 Web 服务确实因此受益。
3. **`IRobotKinematics` 抽象** —— 五个接口里唯一站得住的一个，本方案保留（但收窄了它的方法集，§5.4）。
4. **`motion/` 独立子项目 + Eigen/yaml-cpp/CLI11 选型** —— 依赖已在板端就位（§2.6），与 `follow/` 的既有约定一致。
5. **`types.hpp` 中把工件表面点、法向、standoff、`spraying`、`is_jump` 收进统一航点聚合根** —— 解决了当前坐标与工艺字段分散的真实问题。
6. **分阶段路线图的骨架**（运动学 → 插值/验证 → 优化 → IO/CLI → Web 接入）—— 本方案在其前面插入"阶段 0 黄金数据"，并给每阶段补上可验证的验收标准与回退路径。
