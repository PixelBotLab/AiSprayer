# Motion C++ 模块

本文描述 `app/src/core/motion` **当前实现**（不是早期方案稿）。
模块把 CR5 的 FK/IK、MoveL 密采样验证、Viterbi 姿态优化抽成独立 C++ 库与 CLI。
**原 Python 实现未改**，仍是生产路径；C++ 以黄金数据对齐、供后续替换。

---

## 1. 目标与范围

已完成：

- 解析 IK / BestIk / 控制器帧 FK·IK（DH 闭式解原样迁入，不改运算顺序）
- MoveL 笛卡尔插值 + 连续逆解追踪（`ChainVerifier`）
- 工具系网格 + 锚点包络 + 8 分支分桶 + beam + 自适应全量回退的 Viterbi DP
- 写出可被现有 Web 当 `auto_poi` 读的 `*.poi.path.yaml`
- `motion_cli`：`verify` / `optimize` / `fk` / `ik`
- 兼容旧 `libur_kin` 符号的 `libmotion_c.so`（尚未替换 Python 侧加载路径）

明确不做 / 尚未做：

- 不改 `app/src/core/hardware/robot` 下的 Python
- 不迁 `AxialSpinOptimizer`、驱动层
- Web 尚未切到 `motion_cli`（无 `MotionCliClient`、无 `motion.engine` 开关）
- 无多线程边评估；DP 主循环串行
- 稠密轨迹仍内联写在 yaml 的 `verification.trajectory_*`（无侧车 `.npy`）

---

## 2. 落地原则

1. **不为单一实现造接口。** 只有 CR5，没有 `IRobotKinematics` / `IPathOptimizer` / `IAnchorStrategy`。`Cr5Kinematics`、`Interpolator`、`ChainVerifier`、`ViterbiOptimizer` 都是具体类。
2. **数据是聚合体，算法持 `const&`。** `Waypoint`、`Anchor`、`OptimizeOptions` 可拷贝；求解器不拥有机器人模型。
3. **单位与坐标系写进 `conventions.hpp`，不靠注释。** 内部米/弧度，边界毫米/度。
4. **stdout 只有一行 JSON。** 过程表、DP 进度、对照表全部走 stderr。
5. **DH 硬编码，URDF 只给限位和 TCP。** 绝不从 URDF 推导 DH。
6. **禁用改变浮点语义的优化。** `motion_core` 编译加 `-fno-fast-math`。候选去重按首次出现序，网格用 `min + i*step`。

---

## 3. 目录与构建产物

模块在仓库内路径是 `app/src/core/motion`（不是仓库根级 `motion/`）。扁平一层，无 `legacy/`、无三层子目录。

```
app/src/core/motion/
├── CMakeLists.txt
├── include/motion/
│   ├── conventions.hpp     # 单位、欧拉、网格、分支号
│   ├── types.hpp           # Waypoint / PathItem / Anchor
│   ├── robot_model.hpp     # DhParams / RobotLimits / ToolOffset
│   ├── kinematics.hpp      # Cr5Kinematics
│   ├── segment.hpp         # Interpolator + SegmentChecker
│   ├── verifier.hpp        # ChainVerifier
│   ├── optimizer.hpp       # ViterbiOptimizer + ResolveAnchor
│   ├── report.hpp          # Issue / VerifyReport / OptimizeOptions
│   └── io.hpp              # yaml / json / spraying 配置
├── src/
│   ├── kinematics.cpp
│   ├── segment.cpp
│   ├── verifier.cpp
│   ├── optimizer.cpp
│   ├── io_urdf.cpp
│   ├── io_path.cpp
│   ├── io_json.cpp
│   ├── c_api.cpp
│   ├── cli_main.cpp
│   ├── cli_report.cpp      # stderr 过程表（不进 core）
│   └── cli_report.hpp
├── scripts/
│   ├── build.sh            # cmake -S <motion> -B <motion>/build
│   └── run.sh              # 默认黄金参数跑 optimize
├── tests/
│   ├── test_kinematics.cpp
│   ├── test_golden_verify.cpp
│   ├── test_golden_optimize.cpp
│   ├── compare_poi_yaml.py
│   ├── compare_opt_vs_poi.py
│   └── regen_py_poi.py
└── docs/
    └── motion.md           # 本文
```

CMake 三个产物，共享同一份算法：

| 产物 | 来源 | 职责 |
| :--- | :--- | :--- |
| `libmotion_core.a` | kinematics / segment / verifier / optimizer | 纯算法，无 yaml |
| `libmotion_io.a` | io_urdf / io_path / io_json | yaml-cpp + tinyxml2 |
| `motion_cli` | cli_main + cli_report | 批处理入口 |
| `libmotion_c.so` | c_api.cpp → core | 稳定 C ABI，兼容旧 `libur_kin` 符号 |

依赖：C++17、Eigen3、yaml-cpp、tinyxml2、CLI11（系统头）。ARM64 复用 cortex-a76/a55 编译旗标。

---

## 4. 约定

出处：`include/motion/conventions.hpp`。

| 项 | 约定 |
| :--- | :--- |
| 内部 | 长度米、角度弧度；位姿用 `Eigen::Isometry3d` |
| 外部（yaml / CLI / 控制器） | 毫米、度 |
| 欧拉 | Dobot `[rx,ry,rz]`：`R = Rz·Ry·Rx` ≡ scipy `from_euler("xyz")` |
| URDF ↔ DH | `q_dh[1] = q_urdf[1] − π/2`，`q_dh[3] = q_urdf[3] − π/2`；IK 输出域 `[0, 2π)` |
| 控制器帧 ↔ URDF | `CtrlToUrdf` 与旧 `controller_matrix_to_urdf` 逐元素一致 |
| DH（米） | `d1=0.147, a2=-0.427, a3=-0.357, d4=0.141, d5=0.116, d6=0.105` |
| 奇异 | `\|sin(q3)\|`、`\|sin(q5)\|` 阈值 `sin(3°)`；肩部分离半角 3° |
| 分支跳变 | 45°；MoveL 终点同支 5° |
| 网格展开 | `min + i*step`，上界与 `numpy.arange(min, max+0.5*step, step)` 对齐 |
| TCP | URDF 读出后 `round(mm,2)` / `round(deg,2)` 再建矩阵（对齐 Python 银行家舍入，如 `251.67`） |

默认种子（Home）：`[0, 0, -90, -90, -90, 0]` 度。

---

## 5. 核心类型与类

### 5.1 领域数据

```cpp
struct Waypoint {          // 内部米；yaml 边界再换 mm
  int index;
  Eigen::Vector2i pixel;
  Eigen::Vector3d surface_point_m, surface_normal;
  Transform tcp_pose;      // 控制器帧
  double standoff_m;
  bool spraying, is_jump;
  std::optional<Eigen::Vector2d> normal_2d_proj;
};

struct Anchor {            // 值对象，不是策略类
  std::optional<Eigen::Matrix3d> R;   // nullopt = raw（逐点名义姿态）
  Eigen::Vector3d tol_deg;
};

struct OptimizeOptions {   // 聚合初始化 + Validate()
  AxisGrid grid_x{-5,5,2}, grid_y{-5,5,2}, grid_z{-30,30,5};
  int beam_width = 32;
  int max_candidates_per_branch = 16;
  bool dense_verify = true;
  // ... 边抽检步数、权重
};
```

优化只改 `tcp_pose`。`pixel` / `surface_*` / `standoff` / `normal_2d_proj` / 路径级 `dense_surface_points_base_mm` 原样透传（Python deepcopy 语义）。

### 5.2 运动学 `Cr5Kinematics`

单类，无虚接口。

- `Fk` / `Ik` / `BestIk`：URDF 关节、URDF 法兰系
- `FkController` / `IkController`：毫米 + 度
- `CheckSingularity` / `IkBatch` / `IsJointValid`
- 静态热路径 `DhFk` / `DhIk` / `FkRaw` / `IkRaw`：表达式与旧 `cr5_ur_kin` 逐位相同，供 C ABI

`walk_movel` **不在**运动学类里，在 `SegmentChecker`（插值 + 逐步 `BestIk`）。

### 5.3 插值与段检查

- `Interpolator`：航点 → 稠密 `DenseStep`（枪尖 + 法兰控制器帧 + `dt` + `is_jump`）
- `SegmentChecker::Walk`：一段 MoveL 是否可连续跟踪；跳跃段只评终点距离与奇异

### 5.4 验证 `ChainVerifier`

- `Verify(path, init_q)` / `VerifyAll(paths, init_q)`
- `init_q` 只作用于第一条路径，后续以上一条末关节续接
- `Diagnose` 返回 `SingularityFlags`，避免每步重复算变换
- 统计各轴峰值角速度与 `recommended_safe_speed_mm_s`

### 5.5 优化 `ViterbiOptimizer`

构造：`ViterbiOptimizer(kin, tool, opt, verifier*)`。
入口：`Optimize(path, anchor, init_q)`。

锚点在 CLI 层塌成一个 `Anchor`：

| `anchor_source` | `R` 来源 |
| :--- | :--- |
| `config` / `live` | `RotFromCtrlRpyDeg(ref_rpy_deg)`（live 由调用方先读臂再当 config 传入） |
| `home` | Home 关节 `FkController` 的 RPY |
| `raw` | `R = nullopt`，逐点用自身名义姿态做包络中心 |

投影是**相对欧拉盒裁剪**（不是球面测地）：`R_rel = R_anchorᵀ · R_cand` → euler-xyz → wrap → 逐轴 clip 到 ±tol → `R_anchor · R(clipped)`。

---

## 6. 优化管线

对每个航点：

1. 工具系网格旋转 → 锚点硬包络 → 批量 IK
2. 四元数去重（**首次出现序**，不用 `unordered_map` 迭代序）
3. 8 分支分桶，每桶最多 `max_candidates_per_branch`（默认 16）→ 快速候选
4. 第一层：相对种子的小惩罚 + `BeamKeep`
5. 对段 `i-1 → i`：有限代价父节点 × 本层候选，每条边一次 `CheckMoveL`
   - `is_jump` 或 `spraying=off`：跳跃边，不走稠密直线
   - 精简候选不通且全量更多 → 自适应回退全量，**两边 `edges`/`feasible` 累加**
6. 回溯；`parent == -1` 抛错，不负索引越界
7. 可选 `dense_verify`：用 `ChainVerifier` 密采样硬门；ERROR / 奇异则拒绝整条 DP

单点路径：`modified` 按姿态是否变化判定，不走 DP。

stderr 过程行（与 Python `path_opt.py` 同格式）：

```
⏱️ [SprayOpt] Stage candidates generated: N waypoints, fast_candidates=... full_nodes=..., elapsed=... ms
⚠️ [SprayOpt] Segment i-1->i failed with pruned candidates (...). Triggering adaptive fallback...
⏱️ [SprayOpt] DP Segment i-1->i: edges=... (feasible=...), elapsed=... ms
⏱️ [SprayOpt] Total Viterbi DP search: N-1 segments, elapsed=... ms
```

`edges` = `CheckMoveL` 次数；`feasible` = `ok` 的边数。

CLI 在 DP 前后还会打 1–8 节对照表（输入、引擎、锚点、参数、三张对照、校验、保存路径），实现在 `cli_report.cpp`。

---

## 7. YAML 契约

输入必须是未优化的 `scan.auto.path.yaml`（不是 poi）。
输出要能被现有 Python Web 当 `auto_poi` 读。

写出时必须满足：

- `spraying: 'on'` / `'off'` **单引号**（YAML 1.1 裸 `on` 会变成 bool）
- `type` / `state_type` = `auto_poi`
- `source_file` 用 basename，不是调用时的绝对路径
- 顶层有 `poi_config`、`updated_at`
- `verification` 含 `urdf_tcp`、`path_reports`、`trajectory_q` / `trajectory_tcp`、`max_joint_velocities_deg_s`
- 浮点用定点字符串写出，避免 yaml-cpp `%g` 把 `150.0` 写成 `150`

`--config configs/aisprayer_config.yaml` 读取：

| 配置键 | 用途 |
| :--- | :--- |
| `hardware.robot.robot_urdf` / `robot_tcp` | URDF 与工具 link |
| `spraying.velocity` / `slerp_step_mm` | 未显式给 `--speed`/`--step` 时的默认（仓库配置现为 150 / 2.0） |
| `spraying.poi_anchor_source` / `poi_ref_rpy_deg` / `poi_tolerance_rpy_deg` | 锚点 |
| `spraying.grid_tol_{x,y,z}_deg` | 搜索网格 |

优先级：CLI 显式值 > `--config` > 代码默认。要对黄金 poi，必须显式 `--speed 120 --step 1.5`。

---

## 8. CLI 协议

二进制：`app/src/core/motion/build/motion_cli`。

### 8.1 铁律

1. stdout **只有一行 JSON**（成功失败都是）。
2. 退出码：`0` 成功；`2` 参数错误；`3` 文件/URDF 解析失败；`4` 算法失败（无 IK / DP 不通 / 密采样拒绝）。
3. 过程与表格只写 stderr。

### 8.2 子命令

```bash
# 验证
motion_cli --config configs/aisprayer_config.yaml verify \
  --input data/.../scan.auto.path.yaml \
  --speed 120 --step 1.5 [--path-id N] [--seed 0,0,-90,-90,-90,0]

# 优化 → poi
motion_cli --config configs/aisprayer_config.yaml optimize \
  --input data/.../scan.auto.path.yaml \
  --output build/out/scan.auto.cpp.poi.path.yaml \
  --anchor-source config --ref-rpy 90,0,90 --anchor-tol 10,10,30 \
  --grid-x -5,5,2 --grid-y -5,5,2 --grid-z -30,30,5 \
  --speed 120 --step 1.5 [--no-dense-verify]

# 调试
motion_cli fk --joints 0,0,-90,-90,-90,0
motion_cli ik --pose x,y,z,rx,ry,rz --seed 0,0,-90,-90,-90,0
```

`--urdf` / `--tool-tcp` 可被 `--config` 填充。`live` 不单独实现：由调用方读臂后以 `config + --ref-rpy` 传入。

### 8.3 stdout JSON（摘要）

verify：

```json
{"success":true,"action":"verify","elapsed_ms":8.420,"status":"PASS",
 "summary":{"status":"PASS","total_paths":1,"total_waypoints":81,"total_steps":5178,
            "total_issues":0,"singularity_count":0,"overspeed_count":0,"unreachable_count":0},
 "peak_joint_speeds_deg_s":[...],"recommended_safe_speed_mm_s":120.0}
```

optimize：

```json
{"success":true,"action":"optimize","elapsed_ms":1410.120,"was_modified":true,
 "status":"PASS","summary":{"status":"PASS","total_paths":1,"total_waypoints":81,
            "total_steps":5178,"total_issues":0}}
```

失败同样是一行 JSON：`{"success":false,"action":"...","message":"..."}`。

---

## 9. C ABI（`libmotion_c.so`）

符号与旧 `cr5_kinematics_cpp` 对齐，便于日后只改 Python 库路径：

`c_ur_forward` / `c_ur_inverse`、`c_forward` / `c_inverse`、`c_compute_fk` / `c_compute_ik`、
`c_forward_controller` / `c_inverse_controller`、`c_get_best_ik`、`c_walk_movel`、`c_inverse_batch`。

当前 Python 仍加载原 `libur_kin.so`，未切到本库。

---

## 10. 构建、运行、测试

在仓库根目录：

```bash
app/src/core/motion/scripts/build.sh
ctest --test-dir app/src/core/motion/build --output-on-failure

# 默认：config 锚点 [90,0,90]，容差 [10,10,30]，网格 ±5/±5/±30，speed 120 / step 1.5
app/src/core/motion/scripts/run.sh

# 显式子命令（仍带 --config）
app/src/core/motion/scripts/run.sh verify --input data/.../scan.auto.path.yaml --speed 120 --step 1.5
```

`run.sh` 找不到 `motion_cli` 时会先跑 `build.sh`。默认输出：
`app/src/core/motion/build/out/scan.auto.cpp.poi.path.yaml`。

| 测试 | 断言 |
| :--- | :--- |
| `test_kinematics` | 欧拉往返 <1e-9；Home FK↔IK 闭合；`BestIk` 回到种子 |
| `test_golden_verify` | 黄金路径 81 点、5178 步、PASS、峰值角速度与首关节对齐 |
| `test_golden_optimize` | 相对重构前 `scan.auto.poi.path.yaml`，位置 <0.05 mm、测地角 <0.05° |

黄金输入：`data/template_group/2026-09-03_225937/scan.auto.path.yaml`。
对照物：同目录重构前 Python 产出的 `scan.auto.poi.path.yaml`。

辅助脚本（不进 ctest）：`tests/compare_poi_yaml.py` 做三份 yaml 键集合/轨迹比对；`regen_py_poi.py` 用旧 Python 再跑一份对照。

实测（同一条 81 点路径，锚点 config，speed 120 / step 1.5，RK3588）：

| 引擎 | 内核 | 墙钟 |
| :--- | :--- | :--- |
| 旧 Python（IK 已是 `libur_kin`） | ~17.8 s | ~20.5 s |
| 本模块 C++ 单线程 | ~1.4 s | ~1.7 s |

量级由边评估的 IK 次数决定（约 80 段 × 最多 4096 边 × 每边多次 `BestIk`），不是毫秒级。yaml 内联 5178 步轨迹仍是写出侧的主要成本，目前按 Web 兼容选择内联。

---

## 11. 相对早期两份方案的取舍

早期 `motion_cpp_refactor_design.md` 与 `motion_cpp_best_practice_design.md` 已合并进本文。实际采纳：

| 点 | 早期方案 A | 早期方案 B / 现状 |
| :--- | :--- | :--- |
| 位置 | 仓库根 `motion/` | `app/src/core/motion` |
| 接口 | 5 个纯虚 + Builder + 策略类 | 零接口；`Anchor` 值对象；`Validate()` |
| 产物 | 仅 CLI | core.a + cli + c.so |
| 性能目标 | 优化 5–15 ms | 实测约 1.4 s（81 点），与算法量级相符 |
| 轨迹落盘 | 未细说 / 建议侧车 | 内联 `verification`，对齐现 Web |
| Web 接入 | 立即 `MotionCliClient` | **未做**；Python 保持原样 |
| 正确性 | 未写 | 黄金回归 + 禁 fast-math + 确定化顺序 |

保留下来的判断：CLI + 单行 JSON 做进程隔离；DH 不从 URDF 读；航点聚合根带表面点/法向/standoff/`spraying`/`is_jump`。
