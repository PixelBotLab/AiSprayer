# follow 工位跟随系统：设计、实现与维护手册

> **这份文档的定位**：它是 follow 这套功能**唯一的**完整说明。代码注释解释"这一行为什么这么写"，
> 这份文档解释"整条链路为什么长这样、改哪里会连带坏在哪里、以及出问题时按什么顺序查"。
>
> **所有数字都带出处**：文中标 `path:line` 的一律可直接跳过去核对。凡是"实测"二字出现的
> 地方，都对应一条能重跑的测试或工具命令（见 §11）。没有实测支撑的推断会被明确标成
> **未验证**，不要当结论用。
>
> **阅读顺序建议**：第一次接手读 §1–§3（功能、链路、单位口径），改算法读 §4–§5，
> 改集成读 §6–§8，出事读 §13，动手改之前读 §12 与 §14。

---

## 0. 一页速览：改哪儿，看哪节，会连带坏什么

| 你要动的地方 | 章节 | 连带影响 |
|---|---|---|
| 配准精度、判据门、σ 门限 | §4.7 §4.8 | `follow_replay` 的 p95 门会动；页面 σz 读数含义变 |
| 分辨率/帧率/对齐方式 | §3 §6.1 | 内参、点密度、`min_cloud_points`、硬件 D2C 可用性一起变 |
| 示教逻辑（帧数、均值、落盘） | §4.2 §4.3 | 参考地图内容哈希变 ⇒ 旧 `.frmap` 必须重示教 |
| 陀螺仪 | §5 | `GyroTimeBase` 把设备 µs 换到主机域；`enable_imu` 两条路径都认 |
| 臂的映射/IK | §7 | `mirror.py` 是唯一数学出处，改它必须同步改 29 条单测 |
| 页面按钮、实时刷新 | §8 | 仿真臂只有一个写入者，互斥在入口 |
| 配置键 | §9 | C++ 与 Python 各读一份，struct 默认 / yaml / 代码硬编码三处不一致 |
| 加新状态/新字段 | §6.4 §6.5 | 快照字段有 4 个消费方（页面、日志、REST、WS）要对齐 |

**一句话架构**：`follow/` 是一个**只懂算法**的静态库；它有两个消费方——独立工具 `follow_pose`
（验证与量测用）和相机服务里的 `FollowWorker`（生产用）。相机服务把结果通过 HTTP 快照交给
Python 后端，Python 后端做臂侧镜像与 IK，再通过 WebSocket 推给页面驱动仿真臂。
链路上**没有任何一个环节自己发明第二套算法**，这是全部设计约束里最强的一条。

---

## 1. 功能定义与验收标准

### 1.1 这个功能到底是什么

眼在工位上方（**eye-to-hand**，`follow.camera` 固定不动），相机看着工件。示教那一刻冻结一份
工件的参考几何；之后每一帧解出"相机相对示教位动了多少"，得到 6DoF 修正量。臂不需要和相机
位姿相同，**要一致的是位移与旋转的增量**：相机沿基座 X 走了 50 mm，仿真臂就沿基座 X 走 50 mm。

数学上这个"增量映射"是共轭，不是右乘，理由写在 `apps/follow/mirror.py:21-27`，是整条链路里
最容易被改错的一处（详见 §7.1）。

### 1.2 明确不做什么（避免后来者顺手"补全"）

- **不做滑动窗口里程计**。地图只吃示教帧，不吃实时帧。理由与代价见 `follow/include/follow/reference_map.hpp:1-10`。
- **不做两个解算器的加权融合**。GICP 与稀疏解按判据二选一，不平均不拼接（`follow/include/follow/odometry.hpp:10-11`）。
- **本轮不发真实臂**。`follow.arm.mode: real` 被点击即拒绝（§7.5）。预留的是接缝，不是空函数。
- **eye-in-hand 不支持**。`follow.mount` 只允许 `eye-to-hand`；相机在法兰上时"工件动了"和
  "相机动了"纯视觉不可辨识，必须按帧减掉臂反馈，那是独立阶段（`configs/aisprayer_config.yaml:101`）。

### 1.3 验收标准（可机判的那一份）

| 项 | 门 | 由谁保证 |
|---|---|---|
| 平移精度 | 每用例 \|Δt\| 的 p95 < **2.0 mm** | `follow/tools/replay_main.cpp:35` |
| 旋转精度 | 每用例 \|ΔR\| 的 p95 < **0.2°** | 同上 `:36` |
| 状态语义 | 全程无"状态违规"（退化不当 ok、出包络不当 lost） | `replay_main.cpp:300-307` |
| 单位口径 | 只有 `pose_io.cpp` 跨 SI/mm 界 | §4.11 + `test_core` 的 `PoseIO.*` |
| 配置自洽 | 仓库里那份 yaml 自己过校验 | `test_config.cpp` 的 `Config.RepoConfigIsValid` |
| 无硬件可验证 | 上述全部**不需要相机也不需要臂** | §11.1 |

最后一行是刻意的：物理相机是这台机器上最稀缺的资源，所以"算法对不对"必须能在没有相机的
时候回答。`follow_replay` 吃合成数据集（`out/synth`），10 个用例 98 帧，覆盖已知真值的运动。

---

## 2. 端到端链路：10 跳与每一跳的数据形态

```
 [C++] CameraDriver 取流 (640x480@15, 硬件 D2C)
   │  ①  FrameData{color BGR, depth CV_16UC1(mm), frame_index, timestamp_ms=主机 ms}
   ▼
 [C++] FollowWorker::loop            follow_worker.cpp:341
   │  ②  注入陀螺样本 (:490) → FeatureFrontend::extract → Tracker::track
   │      内部全程 SI：米 / 弧度 / 纳秒
   ▼
 [C++] FollowSnapshot                follow_worker.hpp:48-95
   │  ③  跨界换单位：pose_*=mm/deg，delta_t_m=米，delta_r=行主序 3x3
   ▼
 [C++] GET /api/v1/camera/follow/status   http_server.cpp:507  {code,msg,data}
   │  ④  JSON。σ 的 +inf 被写成 null（http_server.cpp:58-65）
   ▼
 [Py ] FollowService._fetch_snapshot  follow_service.py:172   超时 0.5 s
   │  ⑤  dict；frames 字段当去重键（:396-406）
   ▼
 [Py ] joints_to_target              mirror.py:123
   │  ⑥  Δ_base · T_baseline → URDF 帧 → 最近分支 IK → joints(rad)
   │      _kin_lock 串行（follow_service.py:54-56）
   ▼
 [Py ] _emit / WS follow_state        follow_service.py:454   api.py:87
   │  ⑦  {…follow 快照…, joints_deg(度), target_pose(mm+deg)}；五元组去重
   ▼
 [TS ] FollowPanel                   FollowPanel.tsx:125-139
   │  ⑧  只在"关节角字符串真的变了"时才往外推（:131-134）
   ▼
 [TS ] InteractiveOp → WorkspaceView  InteractiveOp.tsx:265-267 / WorkspaceView.tsx:65
   │  ⑨  simJoints（度）→ effectiveRobotState.joint
   ▼
 [TS ] Robot3DViewer.setJointValue   （弧度，URDF 约定）
      ⑩  页面看到的臂
```

### 2.1 单位口径表（这张表是防事故的，不是科普）

| 量 | 单位 | 出处 | 消费方 |
|---|---|---|---|
| 点云、`delta_t_m`、`T_*` 矩阵平移 | **米** | `types.hpp:1-2`、`mirror.py:11-15` | 全链路内部 |
| `depth` 图 | **1 LSB = 1 mm**，`CV_16UC1` | `teach_core.hpp:12`；类型守卫 `follow_worker.cpp:232` | 反投影 |
| `pose_mm` / `target_pose[:3]` | **毫米** | `follow_worker.hpp:45-47` | 页面显示 |
| `pose_rpy_deg` / `target_pose[3:]` | **度**，`R = Rz(rz)·Ry(ry)·Rx(rx)` | `pose_io.hpp:6-10` | 页面、ServoP |
| `sigma_t_mm` / `sigma_r_deg` | **毫米 / 度**；无估计 ⇒ `null` | `uncertainty.hpp:25-27`、`http_server.cpp:58-65` | 判据、页面 σz |
| 关节角（内部、URDF、页面） | 内部 **rad**，边界 **deg** | `mirror.py:126`、`FollowPanel.tsx:134` | IK、viewer |
| 时间戳 | **ns**（C++ 内部） | `types.hpp:68` | 去重、陀螺积分窗口 |

**踩过两次的同一个坑，写清楚**：`cr5_kinematics` 的 `forward/inverse/controller_matrix_to_urdf/
get_best_ik` 全部吃**米制 4x4**，而 `forward_controller` 返回的是 **mm 位姿向量**。两者差 1000，
混用的表现不是崩溃，而是"矩阵看着完全正常，IK 一个解都没有"——一个自洽的错误。所以
`tcp_pose_ctrl_from_joints` 里那行 `T[:3,3] *= MM_TO_M`（`mirror.py:108`）不能省也不能移到别处，
且由 `test_follow_mirror.tcp_matrix_is_metric` 钉住。

### 2.2 姿态欧拉约定：只有一个说法

Dobot 的 `[rx,ry,rz]` 是**内禀 'xyz'**，等价于**定系矩阵乘序** `R = Rz(rz)·Ry(ry)·Rx(rx)`。
这句话在三处必须一致：`pose_io.hpp:6-10`、`core/handeye/geometry.py`、
`follow/tools/pose_main.cpp` 的横幅。已对本机 `cr5_kinematics`（Python）与 `cr5_kinematics_cpp`
数值核对：300 组随机关节 FK→tuple→矩阵偏差 9.4e-16，FK→IK 300/300 复现同一位姿。

**两条硬规矩**：
1. 不要把"内禀 xyz"读成 `Rx·Ry·Rz` —— 顺序反了，而且反了以后大部分场景看着还是对的。
2. **不要拿三元欧拉比相等**。`|ry|→90°` 附近反解会返回一个等价分支，矩阵相同、三元组不同
   （`pose_io.cpp:12` `kGimbalEps=1e-9`；对应测试 `PoseIO.GimbalLockStillReconstructsTheSameRotation`
   与 `test_follow_mirror.gimbal_pose_returns_equivalent_branch_not_the_same_numbers`）。

---

## 3. 取流层

follow 自己不抢设备。设备只有一条取流路径，所以有下面这套约束。

### 3.1 进程间独占锁

- 全项目唯一一把相机 flock：`.orbbec.lock`（相对项目根，路径以配置键为准）。
- **相机服务启动即持有**：`camera_driver.cpp` `tryConnectDevice()`（`dev_lock_.acquire`），`flock(LOCK_EX|LOCK_NB)`，文件里写 PID 文本，
  抢不到时报"相机锁被进程 N 持有"。
- 独立工具（`follow_pose` / `follow_node` / `follow_probe_device`）拿不到锁就明确失败退出，
  不会和services抢。这是 `follow_pose` 能随时验证的前提。
- 锁路径来自 `follow.camera.lock_path`，但**由相机服务解释**（`main.cpp:153`）。注意那行还带了
  一个门：follow 配置解析失败时把 `device_lock_path` 置空 ⇒ follow 坏了一个可选键，不该连带
  让相机服务失去独占能力（宁可少一层仲裁，也不要"服务起不来"）。

### 3.2 档位与对齐阶梯（follow 使能时会重启 pipeline）

| 步骤 | 代码 | 说明 |
|---|---|---|
| 选档 | `configureAndStartPipeline()` 开头（`want_w/want_h/want_fps`） | `follow_mode_` 真 ⇒ 用 `follow.camera.*`，否则用 `hardware.camera` |
| 彩色/深度同名同率 | 同上函数那段注释 | 硬件 D2C 要求两者分辨率一致；不一致时 follow 前端只能整帧丢 |
| 报**实际交付**档位 | `delivered_w/h/fps` 与 `status_.follow_profile` | 设备完全可以给你一个最接近的；状态里必须报实测值 |
| 对齐阶梯 | 同函数 `align_mode` 三级 try/catch | HW D2C → SW D2C → `ALIGN_DISABLE`，逐级 try/catch 降级并记 `depth_align_mode` |

硬件 D2C 只在 **640x480 / 640x400 / 640x360 / 480x270 / 424x240** 这几档可用，且**0% 主机 CPU**；
软件 D2C 在 848x480 上约 16–22 ms、1280x800 约 29–35 ms —— 这就是仓库里把 follow 档定在
640x480@15 的全部理由（`configs/aisprayer_config.yaml:104-106`）。配置里写超过 640 宽，
`check_config` 会警告"超出硬件 D2C 档…服务里跑 follow 会退回软对齐…独立跑 follow_pose 不受影响"
（`follow/src/config_loader.cpp:366`）。

彩色比深度晚上线约 **333 ms**，所以 `first_pair_timeout_ms: 3000` 等的是第一个**成对**帧组
（`orbbec_capture.hpp:69`）。

### 3.3 取流健康度（不是心跳，是判据）

`CaptureHealth`（`orbbec_capture.hpp:84-92`）报：`unpaired_framesets`（只含深度被丢弃的帧组）、
`dropouts`（间隔 > 1.5 个名义帧周期）、`period_ms`（EMA 0.1）、`max_period_ms`、
`d2c_offset_ms`（`color_ts - depth_ts` 滑动平均，实测常量 **+0.347 ms**）。
这些数在 `follow_pose` / `follow_node` 一路完整；相机服务侧因为拿的是 `FrameData`，
只保留 worker 自己的 `frames / dropped / rejected / fps / compute_ms`。

---

## 4. 算法内核

这一节全部在 `libfollow` 里，**不依赖 Orbbec SDK、不依赖 HTTP、不依赖 Python**，
所以能在没有相机的板子上完整跑通。可构建性本身就是验收条件（`teach_core.hpp:8-10`）。

### 4.1 冻结参考地图

`ReferenceMap`（`reference_map.hpp`）= 示教帧点云 + `small_gicp::GaussianVoxelMap`。

- **坐标系**：叫"参考系"，由 `build_from_frames` 里第一帧的 `T_ref_cam` 决定；示教时它就是单位阵
  （`teach_core.cpp:84`），于是追踪器输出的 `T_ref_cam` **直接就是相对示教位的修正量**，
  不需要 T0/invert() 首帧特例，也没有累积漂移（`odometry.hpp:4-7`）。
- **地图构建**：`small_gicp::preprocess_points(merged, voxel_m, 10, 1)` + `create_gaussian_voxelmap`
  （`reference_map.cpp:113,119`）。串行、单线程：示教是离线动作，6 万点毫秒级，而并行体素降采样
  会引入 run-to-run 差异（`reference_map.hpp:51-53`）。同一份输入必然同一份地图。
- **内容哈希**：FNV-1a-64 只对**点数据字节**取哈希（`reference_map.cpp:19`），`voxel_m` 刻意**不在**
  哈希里 —— 体素尺寸是"同一份几何的不同索引方式"，但它影响身份，所以另有
  `ReferenceMap.VoxelSizeIsPartOfTheIdentity` 一条断言管着它。换工件一定换哈希，重启后能确认
  还是不是那份基准（页面上 `map_hash` 就是这个数）。
- **规模上限**：`kMaxScans=64`、`kMaxPoints=8'000'000`（`reference_map.cpp:15-16`），`load` 里逐帧按
  累计值卡，防截断文件与恶意文件。

### 4.2 示教：多帧时间域均值

`build_reference_map(depth_frames, tp, ts, save_path, out, err)`（`teach_core.hpp:36-38`）：

1. 逐像素累加深度与有效计数（`teach_core.cpp:28-29`）。
2. **只信"至少 N/3 帧有效"的像素**（`:60` `min_valid_frames = max(1, frames/3)`）：一次性坏点被剔除，
   一直读不出来的地方本来也不该进基准。
3. 反投影成相机系米制点云 ⇒ 一个 `TeachFrame{pts, Identity}` ⇒ 建图。

为什么均值而不是取单帧：结构光散斑**逐帧独立**，同一像素下一帧就换值，而基准要用很久。
N 帧均值把散斑压到 1/√N；比事后调大 `voxel_m` 划算（voxel 变大丢的是真实几何）。

服务侧收帧：`follow_worker.cpp:239-302`，`need = cfg_.teach_frames`（yaml 里 **10**），
**deadline = `t_start + 1000 + need*200` ms**（`:244`）。超了就带着"只拿到 k/N 帧"报失败，
绝不把 HTTP 线程挂住。示教帧走的是**和运行期同一条** `frameUsable()` 守卫（`:280`）——
建在"运行期会被拒掉的帧"上的基准，之后每一帧都在跟坏几何比。

> **注意 clone 契约不同**：服务侧 `depths.push_back(fd.depth)` 不 clone（`:287-290`），因为
> `CameraDriver` 交出来的已经是它自己的私有缓冲、换帧是整体赋值；`follow_device` 那边必须
> clone，因为它复用取流缓冲。**两层的契约不同，不要互相照抄。**

### 4.3 `.frmap` 落盘格式

小端；`magic 0x50414D52`（字节序上即 `"RMAP"`）+ `version 1` + 每 scan 的原始点（米）+ 尾部
FNV-1a-64 内容哈希（`reference_map.cpp:13-17`）。`save` 之后**立刻读回比对哈希**
（`teach_core.hpp:33-35`）：示教时内存里那张和下次启动从盘上拿到的那张必须是同一份几何，
否则基准在重启时悄悄换了，而所有修正量看起来仍然正常 —— 那是最难查的一类故障。

### 4.4 特征前端（`frontend.cpp`）

CPU 前端 `name() == "cpu-sift"`（`:101`），RKNN SuperPoint 在 `HAS_RKNN` 下（本板转不出模型，
所以生产实际跑的是 CPU 这条降级路径）。

```
goodFeaturesToTrack(gray, corners, max_features, quality_level, min_distance_px)   :62-63
  → 边缘 kPatchSizePx=10 px 内的点先自己剔掉                                      :70-74
  → cv::SIFT::create(max_features, 3, 0.01, 10.0) 只算描述子                       :52
  → 行 L2 归一                                                                     :95
```

三个"为什么"（都写在 `frontend.cpp:41-48`，改之前先读它）：

- **检测器为什么不用 SIFT 自己的**：前端存在的意义是在 GICP 抓不到几何的地方（白墙海报、弱纹理
  工装）给出**均匀铺开**的对应点；`goodFeaturesToTrack` 按网格给，SIFT/FAST 按响应排序，
  预算会全花在一个高对比度角落上。
- **描述子为什么不用 ORB**：ORB 输出 32 字节二值，转 CV_32F 喂 `BFMatcher(NORM_L2)` 后，位向量的
  L2 距离是 Hamming 的平方根 ⇒ 同一个 0.8 比例门在两个前端上含义不同（0.8 L2 == 0.64 Hamming）。
  SIFT 128 维 float 与 SuperPoint 同量纲，parity 测试才有意义。
- **预算为什么是 200 而不是能检测多少就检测多少**：本机真实 1280x800 工位图上 400 特征 +
  patch12 全流程 **93 ms**，单这一项就超了 15 fps 的 66 ms 预算；200 特征 + patch10 @848x480
  约 14 ms(检测) + 24 ms(描述子)（`frontend.hpp:21-25`）。

**接口契约**（`frontend.hpp:3-7`，被 `Frontend.CpuFrontendHonoursFullResolutionUvContract` 钉住）：
`uv_px` 必须是**全分辨率彩色图**的像素坐标（前端内部若 resize 必须自己乘回来）、`desc` 必须
`CV_32F` 且按行 L2 归一、`image_size` 必须等于输入图尺寸。下游拿 `uv_px` 直接查深度图，
不再做任何尺度换算。

`make_frontend` 失败返回 `nullptr` + 写 error，**不抛异常**（`frontend.hpp:40-43`）：配置错误要在
启动阶段报给人看，不该让进程在取流线程里炸。服务侧对应 `follow_worker.cpp:140-151`。

### 4.5 互近邻与稀疏 3D-3D（`matching.cpp`）

- `mutual_nn`：`BFMatcher(NORM_L2)` + 双向 knn(k=2) + Lowe ratio 0.8（`:11-20`）。两帧描述子维度
  不同直接返回空（混用不同前端时会发生，`Matching.MutualNnRejectsDimensionMismatch`）。
- `sparse_delta`：候选数 `n < max(3, min_inliers=12)` 时返回 **`nullopt`**（`:105`）而不是
  `Status` —— 上层"稀疏解缺席"和"稀疏解失败"是两条不同路径，别混。
- RANSAC 3 点最小解 + **三角形面积门 `min_sample_area_m2 = 4e-4`**（`matching.hpp:28`）挡共线采样，
  内点门 `inlier_dist_m = 0.02 m`（⇒ `thresh_sq = 4e-4`）、`ransac_iters = 32`，最后 **Umeyama 无尺度**
  精修（`matching.cpp:60`）。
- **rng 由调用方持有并播种**（`matching.hpp:37-38`）：RANSAC 结果必须可复现，否则回归测试没有
  意义；且 `rand()` 非线程安全。`Tracker` 默认种子 `0x5EED`（`odometry.hpp:131`）。
- 测的是"相对上一帧"的运动 ⇒ **它不含"相对示教位"的信息**，所以它只能当替补，而且必须受连击
  上限约束（§4.8）。

### 4.6 稠密 GICP 组装（两个坑都在这里）

`align_dense`（`odometry.cpp:39-99`）刻意**绕开** `small_gicp::align(GaussianVoxelMap&, …)` 便捷函数，
直接组装 `Registration<GICPFactor, ParallelReductionOMP>`。两个原因（`odometry.hpp:51-56`）：

1. 便捷函数**不会**把 `max_corr_correspondence_distance` 写进 rejector ⇒ 门停在
   `DistanceRejector` 默认的 **1.0 m**，等于没有门。
2. 高斯体素地图只在 **3×3×3 邻域**内找最近邻（`set_search_offsets(27)`），门大于
   `2.6·voxel_m` 就够不着 ⇒ 实际生效的门是
   `gate_m = std::min(p.max_corr_m, reach_m)`（`odometry.cpp:67`），`reach_m = 2.6·voxel_m`。

**这就是 `max_corr_m: 0.035` 与 `voxel_m: 0.015` 是配对的**：yaml 注释里那句"实际生效门 =
min(此值, 2.6*voxel_m)"（`configs/aisprayer_config.yaml:125`）不是装饰。只调其中一个，会得到
一个看着收紧了其实没动的门。
`threads = 4` 是绑 4 个 A76，不要 8（RK3588 大小核，8 线程会被调度到 A55 上并把尾帧拖长）。

源点处理：`depth_to_cloud` 按 `depth_stride` 抽稀 + `[zmin,zmax]` 卡范围（`cloud.cpp`）；
`kNormalNeighbors = 10` 在 `odometry.cpp:18` 与 `reference_map.cpp:17` **两边必须一致**，
否则协方差量纲对不上。

### 4.7 不确定性：尺度与判据是两件事

GICP 的信息矩阵 `H` 加权用的是 `(C_target + C_source)⁻¹`，而那两个 `C` 是**体素内的几何展宽**
（毫米量级，说的是"这一小块面有多平"），**不是**深度噪声。`H⁺` 只有在每条对应点残差协方差
恰好等于那个 `C` 时才是协方差；实际残差是零点几毫米的传感器噪声 ⇒ 直接对 `H⁺` 开根号得到的 σ
系统性偏大约 **190 倍**。

修正：乘上残差自估的 `s² = 2·cost/(3·inliers)`（每条对应点 3 个自由度，模型正确时 `E[ρ]=3 ⇒ s²≈1`），
即 `σ = sqrt(s² · H⁺)`（`odometry.cpp:235`、`uncertainty.hpp:78-91`）。
`H⁺` 用对称化后的 `H` 做特征分解 + 伪逆（特征值小于 `rel_eig_floor·λ_max` 按零处理，`1e-9`），
所以不可观方向表现为"很大的 σ"而不是 `inf`。

实测（合成工件 + 0.5 mm 深度噪声，16 次独立噪声实现，命令运动 `mix(10,-20,30) mm`，
由 `test_registration.NoiseSdMatchesPredictedSigma` 打印）：

| 算法 | σ_t (mm) |
|---|---|
| `H⁺` 直接开根号 | `[13.5, 17.2, 1.8]` ← 偏大 |
| 乘 `sqrt(s²)` | `[0.151, 0.185, 0.018]` |
| 蒙特卡洛真散布 | `[0.132, 0.154, 0.017]` ✅ |

**判据（这是本模块最反直觉的一条）**：绝对 σ 门**不能**用来判退化。不可观方向的残差恒等于零，
那里的 `s²` 塌得比好场景还狠（大平面 `6.4e-6` vs 带肋工件 `9.0e-5`），乘完之后大平面
`σ_t = [0.063, 0.063, 0.002] mm` **比好场景还小，而真实误差 9.99 mm**。

所以用了**组内各向异性** = 最差方向 / 最好方向的 σ 比（`uncertainty.hpp:35-41`）：

- 它**不含尺度**，乘 `s²` 不改变它 ⇒ 骗不过去；
- 组内三个平移同为 mm、三个转角同为 deg ⇒ **量纲自己抵消**，绕开了 6×6 `H` 里
  旋转列（`J_rot = R·skew(p)`，单位 m/rad）与平移列（`J_trans = -R`，无量纲）跨组比值的荒谬
  （同一大墙跨组比值 6.7e-4，而旧门限 1e-3，稍有多余结构就翻过去）。

数值：工件 ≈ **10 / 7**，整幅大平面 ≈ **32 / 27**，门限取 **15**。
`Uncertainty.AbsoluteSigmaAloneWouldPassThePlane` 会把这些数**打印出来并断言**，
`rank_deficient` 在大平面场景下是 **false**（λ_min 没掉到 1e-9·λ_max），所以挡住它的确实只有比值门。

### 4.8 判据门全表 = 状态机

按代码执行顺序（`odometry.cpp:162-251`），**顺序本身就是语义**，不要重排：

| 序 | 条件 | 结果 | 语义 / 处置 |
|---|---|---|---|
| 1 | `cloud_status != kOk`（内参非法、depth 类型错） | 直接返回该状态，**不动任何状态** | 配置/输入问题，没资格当参考帧（`:166-171`） |
| 2 | `ts_ns <= prev_ts_ns_` | `stale_input` | 保持上一目标（`:172-177`） |
| 3 | 地图空 | `hold(config_invalid)` | 没有基准谈不上"相对示教位的修正" |
| 4 | `cloud.size() < min_cloud_points(200)` | `hold(no_depth)` | 有效深度点不足 |
| 5 | `g.ran && inlier_ratio < 0.30` | `hold(out_of_envelope)` | **要重新示教，不是跟丢**。包络门放在收敛门**之前**（`:224-229`） |
| 6 | 收敛 + `inliers >= 80`：σ_t>2.0 mm 或 σ_r>0.2° 或 aniso>15 或 rank_deficient | `adopt(kDegenerate, gicp)` | 仍采用该解（可观方向是真测量），但**告诉上层这一维没测到** |
| 7 | 同上但三条件全过 | `adopt(ok, gicp)`，`sparse_streak_=0` | |
| 8 | 几何可用但求解失败 + 有稀疏解 + `sparse_streak_ < 15` | `adopt(ok, sparse)` | 递推，精度低一个量级 |
| 9 | 其余 | `hold(lost)` | 两个解算器都失败 |

`hold()` 与 `adopt()` 的差别（`:139-160`）就是"**故障时保持上一目标**"这个默认行为的实现：
- `hold`：`T_ref_cam = T_last_good_`，并且**清 `T_vel_ = Identity`** —— 不清的话恢复时的初值会从
  故障前的外推接着推，表现成"故障后第一帧突然跳一段"。
- `adopt`：`T_vel_ = T_last_good_.inverse() * T`（新的帧间运动），并更新 `sparse_streak_`。

`max_sparse_streak = 15` @15fps = **1 s**：够穿过一次抖动/短暂遮挡，不够悄悄漂走
（`odometry.hpp:93-97`）。这是冻结地图语义的守门人 —— 稀疏解测的是帧间增量，连续用它就是在
重新发明里程计。

`Estimator::kNone` 时 `unc` 刻意置成 `solver_failed` + 全 `+inf`（`odometry.hpp:112-114`），让
`within(...)` 恒为 false。原因："默认 0 看起来像很准"会把稀疏解骗过去。
`gicp_cost` 在运维上比 σ 好用：除以 inlier 数就是"平均每条对应点的马氏残差"，正常应远小于 1，
变大说明场景在漂或地图已不对应当前工件 —— 而这两个都不体现在收敛标志上（`odometry.hpp:117-119`）。

状态词表**只有 8 个**（`types.hpp:34-57`）：`ok / degenerate / out_of_envelope / lost / no_depth /
stale_input / config_invalid / device_gone`，加 worker 独有的三个：`disabled / no_map / no_frame`
（`follow_worker.hpp:53-55`）。加词必须同步改：页面判断、`FollowSnapshot::reason` 分派
（`follow_worker.cpp:558-575`）、以及 §13 排障表。

### 4.9 三档运动初值

稠密配准对初值极敏感 —— 这一步决定它是收敛还是爬到隔壁体素上（`odometry.cpp:186-210`）：

| 档 | 条件 | 给什么 | 为什么 |
|---|---|---|---|
| 1 | 稀疏解存在 | 完整 6DoF `sp->T_prev_from_curr` | 六自由度都有深度支撑 |
| 2 | 无稀疏解但陀螺 `valid()` | **只给旋转** `dT.linear() = gyro.R`，**平移留 0** | 没有深度依据的外推平移比不外推更危险 |
| 3 | 都没有 | `dT = T_vel_`（常数速度外推） | 相机固定时它 ≈ 单位阵，等价于"假设没动" |

然后 `T_init = T_last_good_ * dT`（右乘 = 在相机自身轴上加运动，这是对的，因为它描述的是
**相机的运动**；§7.1 里给臂的增量左乘，因为那描述的是**基座系里要走的位移**。两个乘法方向
都对，含义不同，别互相"顺手统一"。）

### 4.10 `PoseSmoother`（`pose_smoother.hpp`）

**显示位姿 = 最近 N 帧平均**，`kDefaultSmoothFrames = 5`（`:25`）。两个消费方共用同一个实现：
`follow_pose` 控制台那一行，和服务里推给页面/臂的那一路（`:1-8`）。

- 为什么需要它：静止实测单帧逐轴噪声 sd 就有 ~2 mm，而"1 mm 就打印/就发臂"是用户要的粒度；
  噪声比阈值还大时死区再调也挡不住刷屏（`follow_pose` 实测 90 帧仍打 61 行）。N 帧平均把噪声
  压到 `sd/√N`，代价约 N/2 帧滞后（@15fps、N=5 ⇒ **0.17 s**）。
- **精度统计仍用全部原始单帧** —— 这里只影响"报出来的是哪一个数"。
- 平均做法：平移算术均值 + 旋转矩阵算术均值，然后 **SVD 投影回 SO(3)**
  `R = U·diag(1,1,det(UVᵀ))·Vᵀ`（`:49-59`）。不投影的话"平均出来的矩阵不是正交阵"，直接解欧拉角
  会得到带尺度误差的旋转。窗口内角度跨度 <1° 时这步几乎不改变结果，但它保证"平均"不会悄悄
  变成一个不合法的姿态。
- 契约：**至少 push 过一次之后再读**（空窗口的均值是 0/0）。服务侧用
  `smoother_.size() > 1 ? value() : r.T_ref_cam` 绕开（`follow_worker.cpp:506`）。

### 4.11 单位与跨界：只有一个文件允许

`follow/include/follow/pose_io.hpp:1-4` 是整个模块的单位闸门：内部一律 SI，只有这个文件跨到
mm + deg。`to_dobot` / `from_dobot` 互逆（±180° 归一化意义下），`DobotPose::finite()`（`:21-27`）
是送臂前最后一道闸 —— **任何非有限值都不能变成电机指令**（`PoseIO.NonFinitePoseIsCaughtBeforeItBecomesMotion`）。

同类"必须在构造期挡 NaN"的地方还有 `CameraIntrinsics::valid()`（`types.hpp:26-31`）：`fx<=0`
会让 `unproject` 除零产生 inf/NaN 点云，**而 NaN 躲得过所有范围比较**，所以不能指望下游过滤。
`Tracker::push_gyro` 同理（`odometry.cpp:111-113`）：一个 NaN 样本会把整段积分染成 NaN，
而 NaN 的 R 能通过所有比较。

---

## 5. IMU（板载陀螺仪）优化专章

### 5.1 它在系统里干什么：**只当旋转初值，不当测量**

陀螺仪**不进入**输出的位姿，也不参与任何加权融合。它唯一的用途是在"稀疏解缺席"的那一刻给稠密
配准一个旋转初值（§4.9 第 2 档），让 GICP 不至于从"上一帧位置"直接爬飞。TrackResult 里
`gyro_used` 只表示"这一帧用了它当初值"（`odometry.hpp:125`）。

这是刻意的降级定位：336L 的陀螺零偏与尺度因子未标定，拿它积分出来的绝对姿态当测量会污染
判据；当粗初值则刚好够用。

### 5.2 数据通路

```
ob::GyroFrame (~200 Hz, 独立 sensor 回调)
  → omega_raw → omega_cam = R_cam_gyro · omega_raw     // 设备自报出厂外参
  → 有界队列 (500 条)                                   // 满了丢最旧
  → 每帧 drain → Tracker::push_gyro (gyro_buf_max=4096) // 双端裁剪
  → integrate_gyro(buf, prev_ts, ts, max_gap_ns=100ms)
```

- 独立路径：`orbbec_capture.cpp:462-501`（读外参 `:458-467`、起流 `:470-501`、`sampleRate` 自报 `:495`），
  帧时间戳 `:613`，注入 `node_main.cpp:386-393`。
- 服务路径：`camera_driver.cpp` `configureAndStartPipeline()` 第 4 段（`// 4. 读取出厂标定外参 T_cam_gyro`）（`OB_SENSOR_GYRO` 起流 + 回调 push）与
  `CameraDriver::drainGyroSamples`、注入 `follow_worker.cpp:490-496`。
- 只用了 `T_cam_gyro` 的**旋转**部分；平移 `t_cam_gyro` 读了但没用（陀螺只给角速率，平移外参
  对纯旋转积分无影响）。

### 5.3 积分与可信判据（`types.hpp:91-139`）

`R_{t0<-t1}` = 逐样本 `R = R · AngleAxisd(|ω|·dt, ω/|ω|)`（**body 右乘**，`:129`）。
推导：`R(t1) = R(t0)·ΔR_body ⇒ R_{t0<-t1} = R(t0)ᵀR(t1) = ΔR_body`。

返回的不是 `bool` 而是 `GyroDelta{R, samples_used, span_ns, gap_end_ns, stale}`
（`:79-89`）—— **裸 bool 不算证据**，调用方需要知道到底用了几个样本、覆盖了多少时间、
最后一条样本是不是已经很旧。`stale` 的三个条件（`:136-137`）：

1. `samples_used == 0`（窗口里没有样本）；
2. `gap_end_ns > max_gap_ns`（末样本到图像时刻缺口 > 100 ms ⇒ IMU 停更）；
3. `span_ns + max_gap_ns < (t1_ns - t0_ns)`（积分覆盖不足请求区间）。

三条各有一条单测（`Gyro.MatchesAnalyticConstantRateIntegration` / `Gyro.ReportsStaleWhenImuStoppedUpdating`
/ `Gyro.EmptyAndOutOfWindowBuffersAreNotSilentlyUsable`）。

### 5.4 `enable_imu: false` 的效力 —— **两条路径都认它**

`follow.camera.enable_imu` 在两处同时生效：

| 路径 | 判断点 | 关了会怎样 |
|---|---|---|
| 独立工具 | `orbbec_capture.cpp` 起陀螺流处 | 不起 `OB_SENSOR_GYRO`，`calib.has_imu=false` |
| 相机服务 | `camera_driver.cpp` `configureAndStartPipeline()` 第 4 段：`follow_mode_ && config_.enable_imu` | 不起陀螺流，打一条 info；`has_imu_=false` |

键从同一份 yaml 进 `FollowConfig.capture.enable_imu`，再由 `main.cpp` 抄到
`AppConfig.enable_imu` 交给 `CameraDriver`。默认 `true`。关了 = 整条 IMU 链路
（帧间初值 / 离群门 / 静止冻结 / 示教静止门）一起空转，这是刻意的：
只关一半会让"配置关掉了陀螺"和"服务里其实还在积"各说各话。

启动日志必须能还原这个键（`describe()` 的 `camera` 行有 `enable_imu=`，
`FollowWorker::doEnable` 的"陀螺能力"行也会点名）。

### 5.5 两条路径的时间域 —— **已用 GyroTimeBase 对齐（离线钉死）**

| 路径 | 图像帧 `ts_ns` | 陀螺样本 `ts_ns` | 同域？ |
|---|---|---|---|
| `follow_pose` / `follow_node` | `depthFrame()->getTimeStampUs()*1000` | `gyroFrame()->getTimeStampUs()*1000` | ✅ 同一设备时钟 |
| 相机服务内 follow | `FrameData.track_ts_ns = GyroTimeBase::toHostNs(device_ts_us)`；未定标时退回 `timestamp_ms`（同时丢掉陀螺样本） | `toHostNs(getTimeStampUs())`；未定标返回 0 并丢弃 | ✅ 同一主机域（定标后） |

336L 的 `getTimeStampUs()` 是**自开机 µs**（实测末值 ~1.57e9），`timestamp_ms` 是主机
epoch 毫秒。直接混用会静默框空积分窗口 —— 这是曾经的现场。修法不是改公共
`timestamp_ms`（编码器 / 存图还要用到达时刻），而是：

1. `GyroTimeBase` 用 8 对"本帧设备戳 ↔ 本帧主机到达时刻"取**最小延迟**那一对冻结钟差
   （NTP 同一条理由：延迟恒正，最小值最接近真钟差）。见 `gyro_time_base.hpp`。
2. 陀螺入队前一律 `toHostNs()`；未就绪返回 0，调用方必须丢弃（混域比没样本更难查）。
3. follow 跟踪用 `track_ts_ns`，**不能**拿本帧到达时刻当积分窗口：USB 抖动实测
   350~450 ms，定标冻结的是最小延迟，本帧再晚几十/几百 ms 时 66 ms 窗口刚好框空。
4. 定标完成那一帧会从到达时刻切到更早的设备钟，`FollowWorker` 必须重置 `last_ts_ns_`，
   否则 Tracker 当 stale 丢掉。

离线钉死在 `test_gyro_time_base.cpp`：`GyroTimeBaseSeam.CalibratedOutputIsUsableByFollowIntegrator`
与 `ArrivalTimeMissesWhenUsbJitterExceedsFramePeriod`。现场指纹是
`gyro.buf > 0 && gyro.samples == 0` 连续 ≥30 帧 ⇒ `checkGyroChannel()` 打 ERROR；
快照字段 `gyro.time_ready / buf / samples / dead_frames` 给外部看（不是已经去掉的
`gyro_used` 布尔 —— 那个只说"这一帧当初值没用陀螺"，分诊不够）。

停流 / 换档 / 重连必须 `gyro_time_base_.reset()`：设备可能换了时间戳原点。

---

## 6. 相机服务集成（follow 作为库跑在本进程里）

### 6.1 为什么不是第二个进程 / 第二个 Pipeline

`follow_worker.hpp:1-20` 是这一节的原始出处，三条约束：

1. **不能在这里再 `new follow::OrbbecCapture`**：那会在同一台设备上开出第二个 `ob::Pipeline`。
   本进程已经 take 了 `.orbbec.lock`，正是为了不让第二路取流存在。所以 follow 消费
   `CameraDriver` **已经在交付**的那一份对齐帧。
2. **绝不链 `follow_device`**（`orbbec_camera_service/CMakeLists.txt:59-62`）：那里面是第二个
   Context/Pipeline/取流线程；同一进程两份 driver 比两份进程更难查。也**绝不链 `follow_health`**：
   状态由本进程自己的 `http_server` 报，两处报同一个数必然不一致。
   实际只链 `follow` + `follow_config`（`:86-87`）。
3. **跟踪时间戳必须与陀螺同一域**：`FrameData.timestamp_ms` 仍是主机到达时刻（日志 / 存盘 /
   帧率，粒度比 15 fps 的 66 ms 粗，同毫秒两帧当丢帧计）。跟踪用的是
   `track_ts_ns = GyroTimeBase::toHostNs(device_ts_us)`（§5.5）。未定标时退回到达时刻，
   同时丢掉陀螺样本 —— 两条路径一起空转，不会各走各的钟。

代价与收益都写在这里，别留给下一个读代码的人猜。

### 6.2 使能时序：`setFollowProfile` 的 7 步

`camera_driver.cpp` `setFollowProfile()`：

1. **标定模式下直接拒绝**（`:472-477`）：那里深度流是关的，follow 起不来。文案直接告诉
   运维"先退出标定模式再使能 follow"。
2. 参数兜底 `w/h/fps <= 0 ⇒ 640/480/15`（`:479-481`）。
3. **无需切换就返回**（`:484-488`）：`enabled`、`w`、`h`、`fps` 全等 ⇒ 不重启 pipeline。
   这条存在的意义是防止页面上的重复点击导致反复断流。
4. 记住切换前状态（`:490-497`）。
5. pipeline 还没起时只记标志（`:499-504`），连接时自动生效。
6. `pipe_->stop()` → `configureAndStartPipeline()`（`:511-519`）。
7. **失败回滚**到切换前档位（`:521-537`），且**回滚成功 ≠ 切换成功**：错误串里明确说
   "取流已回滚到切换前档位"或"回滚也失败，等自动重连"，调用方（和页面）必须知道
   自己**没拿到** follow 档。

`FollowWorker::setEnabled`（`follow_worker.cpp:91-189`）在它外面包了：配置不可用则拒绝（`:97`）、
状态未变不重复切档（`:102`）、关闭时**作废位姿并同步档位字段**（`:108-134`，因为旧档位下解出的 T
属于另一套像素网格，页面继续拿它推臂就是拿"停机前的最后一帧"当实时值）、前端在发布 `enabled_`
**之前**建好（`:138-151`，工作线程才可以在不加锁的情况下用它）。

### 6.3 互斥矩阵（谁会顶掉谁）

| 事件 | follow 的反应 | 代码 |
|---|---|---|
| 进标定模式 | HTTP 路由**主动**关 follow；工作线程看到档位被顶掉也会自己关 | `http_server.cpp:210-227`；`follow_worker.cpp:380-417` |
| 出标定模式 | 不自动恢复（默认关是功能的一部分） | — |
| 停止跟随 | 取流退回 `hardware.camera` 档；参考地图**留在内存**里，重新使能即可续用 | `follow_worker.cpp:108-135` |
| 换档 / 换图 | `tracker_dirty_` 或 `map_gen_` 变 ⇒ **重建 Tracker** | `follow_worker.cpp:349-378` |
| 有人点"启动"两次 | 双击会打出两次 enable+teach，第二次把刚示教好的地图换掉 ⇒ 前端 `pending` 期间三个键一起禁 | `FollowPanel.tsx:14-16, 184-193` |

标定模式那条路由还额外报 `follow_auto_disabled`（`http_server.cpp:226-227, 244-245`）——
"是否被顶掉"用**切换前后 enabled 的实际差别**判断，不去问"是谁关的"：HTTP 路由和工作线程都会关，
谁先动手都会让另一种问法得到错的回答。

### 6.4 线程模型与两把锁

- 一条 worker 线程 + 一份 `snap_` 快照；**HTTP 线程只读快照，绝不在请求线程里跑 GICP**
  （一帧几十毫秒，会把 REST 接口卡成串行队列）。
- 两把锁：`state_mutex_`（map/gen/dirty）与 `snap_mutex_`（快照）。**规矩：调
  `camera_->getStatus()` 必须在拿 `snap_mutex_` 之前**（如 `follow_worker.cpp:390-392, 509-511`），
  因为 `getStatus()` 内部有驱动自己的锁，反过来拿会形成锁序倒置。
- 取帧只能用 `getLatestFrame` + 自己按 `frame_index` 去重，**不能用 `waitForNextFrame`**：那个接口
  会推进驱动里的共享游标 `last_consumed_frame_id_`，主循环和 worker 会互相把对方的帧吃掉，
  表现是"两边都掉帧但谁也看不出谁在读"（`follow_worker.cpp:12-16, 438-455`）。
  算不过来时丢帧并计数：**丢了不是错误，悄悄算了旧帧才是错误。**

### 6.5 快照字段与 HTTP 契约

`FollowSnapshot`（`follow_worker.hpp:48-95`）↔ `followSnapshotJson()`（`http_server.cpp:67-117`）
↔ TS `interface FollowSnapshot`（`FollowPanel.tsx:19-53`）—— **三处字段必须同时改**。

三个端点（全部返回同一份 `followSnapshotJson`）：

| 端点 | 行 | 失败语义 |
|---|---|---|
| `POST /api/v1/camera/follow {"enabled":bool}` | `http_server.cpp:420-459` | **400**：`enabled` 缺失或非布尔 ⇒ 拒绝。绝不"解析不到就当默认值"，那会让一个畸形请求把相机档位改掉 |
| `POST /api/v1/camera/follow/teach {"save_map":bool}` | `:461-504` | **409**：未使能时拒收（示教帧必须来自跟随那一档）；**503**：worker 建图失败 |
| `GET /api/v1/camera/follow/status` | `:505-514` | 永远 200，只读 |

`sigma_*` 在无稠密解时是 `+inf`，nlohmann 会把非有限浮点写成 `null`（`http_server.cpp:58-65`
的 `finite_or_null`）⇒ **JSON 层的 `null` 读作"没有估计值"，不是 0**。页面同样用 `'—'` 显示
（`FollowPanel.tsx:76-77`）。

`/api/v1/camera/status` 里的 `follow_profile` **只是一个布尔**（`:145`）：follow 的实时数字**只能**
从 `/follow/status` 拿。别指望状态接口里嵌了快照。

### 6.6 配置坏了怎么办：`setBlocked`

`follow` 的配置读坏（yaml 类型错、`check_config` 报致命）时，**相机服务照样要跑** —— 一个可选
功能的配置错不该把视频流拖下水。做法（`main.cpp:129-153, 188-193`）：

```
load_config 失败 或 check_config 有 fatal
  → follow_block_reason 非空
  → config.device_lock_path 置空（§3.1 那条门）
  → worker->setBlocked(reason)
```

`setBlocked` 之后：`setEnabled` 一律拒绝并把原话交回去（`follow_worker.cpp:91-99`），
而 `/follow/status` 的 `reason` 报**同一条**原因（`:408-410`）。否则运维看到的是"点了没反应"。
三种"没在算"的情形在 `reason` 里被分开（`:402-411`），因为下一步动作各不相同：
在标定模式 ⇒ 先退出那个模式；配置不可用 ⇒ 先修 yaml；剩下才是"没人开" ⇒ 点一下就好。

---

## 7. 臂侧：镜像数学与 IK

**所有臂侧数学集中在 `app/src/apps/follow/mirror.py` 一个文件里，是纯函数**：不碰网络、不碰设备、
不读配置，所以它能在 `test_follow_mirror.py` 里被完整钉住，而不需要相机或机械臂在场。

### 7.1 增量映射：左乘 + 共轭，基准冻结

```
Δ_base = [ R_cb·ΔR·R_cbᵀ | R_cb·Δt ]        # delta_to_base, mirror.py:91-94
T_target = Δ_base · T_baseline               # 左乘，mirror.py:148
```

- **为什么左乘**：用户要"位移和旋转的增量保持一致"。相机沿基座 X 走 50 mm，臂就该沿基座 X 走
  50 mm。一个在 c 系里写作 (R,t) 的运动换到基座描述就是共轭 `(R_cb·R·R_cbᵀ, R_cb·t)`。
  **右乘**（`T_baseline · Δ`）表达的是"沿臂自己当前工具轴动"，那是另一种物理运动：同一个相机
  平移在 home 朝下和朝前时会把臂甩向两个不同方向，与"增量一致"直接矛盾。
- **给矩阵不给欧拉**：快照里的 `delta_r/delta_t_m` 是矩阵+向量（`follow_worker.hpp:65-66`），
  因为**在欧拉上做乘法一定会错**。全程不在欧拉角上乘任何东西，跨帧/跨约定一律走 4×4
  （`mirror.py:149-151`）。
- **基线只在示教那一刻取一次**（启动 = home，调零 = 当时位姿），之后每帧都是
  `Δ_base(当前增量) · T_baseline`。**不能拿实时位姿当基准** —— 那会形成正反馈：臂跟着自己
  上一帧的解算误差继续走，静止时 0.2 mm 的噪声也能慢慢把它推走。
  这条由 `test_follow_mirror.baseline_not_replaced_by_live_pose` 钉住。
- **只用到 `R_cb` 的旋转部分**（`rotation_camera_to_base`，`mirror.py:40-55`）：相机装在哪儿不影响
  增量映射。这顺带绕开了"标定文件平移是 mm、`core.config` 的 resolver 又除成 m"这个单位分叉。
  **别把它的平移喂进来再乘回去。** 正交性必须查（`err > 1e-6` 或 `det <= 0` 直接抛）——
  拿一个被截断/改坏的矩阵去共轭，得到的是一个自洽但全错的运动。

### 7.2 `R_cb` 从哪来，以及降级必须可见

`FollowService._resolve_camera_to_base()`（`follow_service.py:102-134`）：

1. 优先 `sprayer_config.T_camera_to_base`（手眼标定结果）⇒ `_R_cb_source = "手眼标定 T_base_camera（旋转块）"`。
2. 标定缺失、读失败、矩阵不正交、或当前安装是 **eye-in-hand**（相机位姿不是常量，不能当固定轴
   映射用）⇒ 退到配置常量 `follow.arm.camera_to_base_fallback_euler_deg`，
   并在 `_R_cb_source` 后面**追加**"→ 已退回配置常量（降级，方向会有偏差）"。
3. 两条都失败 ⇒ `_R_cb = None`，`_begin` 直接拒绝（`follow_service.py:231-234`）。

**用哪一个必须能被看见**：`r_cb_source` 进 REST 状态、进 WS 广播、进页面 tooltip
（`FollowPanel.tsx:313`）。因为两者给出**不同的平移方向映射**，悄悄降级比直接失败更危险。
（本机实测：走的是第 1 条，即真正的手眼标定结果。）

### 7.3 IK

`joints_to_target`（`mirror.py:123-162`）一步到底：

```
基线(6 rad) 校验有限性
  → T_base = forward_controller(q)  → pose_to_matrix → 平移除成米
  → Δ = delta_to_base(R_cb, delta_r, delta_t_m)      // 米
  → T_target_ctrl = Δ @ T_base
  → T_target_urdf = controller_matrix_to_urdf(...)   // 跨帧，同为米制，无单位换算
  → best = get_best_ik(T_target_urdf, nearest_to)
  → is_joint_valid(best) ? 返回 : "ik_out_of_limits"
```

- `nearest_to` = **上一次的目标**（不是当前测量）：它只影响"用哪种姿势到达"，不影响到达哪里。
  这保证了不会每帧在两个 IK 分支之间来回跳。
- 失败时 `return None, T_target_ctrl, "ik_failed"`，**不夹位、不缩增量**（`mirror.py:155-158`）。
  保持上一目标由调用方负责：在这里悄悄截断，"增量一致"这个契约就破了。
- 后端 `CR5Kinematics(backend="auto")` 在本机解析为 **cpp**（`libur_kin.so` 在）。

### 7.4 运动学必须串行：`_kin_lock`

`follow_service.py:54-56`：

> CR5Kinematics 的 cpp 后端用 per-instance ctypes 缓冲（源码注释：*one solver per thread*）。
> 路由线程要 FK、轮询线程要 IK ⇒ 必须串行化，否则两边共用同一块缓冲。

两个入口（`_solve` `:428`、`_get_kin` `:433`、`_target_pose_deg` `:486`）都在 `_kin_lock` 里。
新增任何碰 `kin` 的代码路径都必须走这把锁。

### 7.5 真实臂：预留的接缝长什么样

- 配置：`follow.arm.mode: sim | real`。
- C++ 校验：`real` 是**警告不是错误**（`config_loader.cpp:510-516`）—— 名字里点明 P5，并且
  "一个还没接线的开关绝不能拦住取流启动"。未知 mode 才是 fatal。
- Python：`_begin` 第一步就拒绝（`follow_service.py:224-230`）：

  > 返回失败而不是"接受请求但什么都不发"：后者会让页面以为臂在跟，而实际上一台真机一动不动 ——
  > 在有人真接上臂之前，这种假装是最坏的失效。

- 页面：`arm_mode !== 'sim'` 时显示红色 `mode:xxx`（`FollowPanel.tsx:225-227`）。
- 其余 `runtime.dry_run` / `enable_servo_p` 在 `configs/aisprayer_config.yaml:143-144`：
  `dry_run: true`，且 P5 控制层接入前 `false` 也会被强制回 `true`。
- 关节反馈**故意不进基线**（`follow_service.py:288-290`）：`cr5_kinematics` 的 DH 里 q2/q4 相对
  URDF 偏 **±π/2**，控制器回报的 J2/J4 与 URDF 关节角差 90°，直接拿来当基线会让臂摆到错位
  90° 的位姿上。那条换算属于 P5，不是这里顺手补的。
  `调零` 因此在"页面没传 joints_deg 且无可沿用目标"时**明确拒绝**而不是退到 home
  （`follow_service.py:301-303`）—— 那会把臂瞬移走，而用户刚点的明明是"就停在这儿"。

### 7.6 三个按钮的确切语义

| 按钮 | REST | 做了什么 | 护栏 |
|---|---|---|---|
| ① 启动 | `POST /api/follow/start` | 使能 follow（切档）→ 示教并落盘 → **基线 = 请求里的关节角，否则 home** → 起轮询 | `mode!=sim`、`R_cb is None`、基线取不到、使能失败、示教失败 —— 各自一条文案（`_begin:222-273`） |
| ② 调零 | `POST /api/follow/zero` | 重新示教 + **基线换成当前位姿** ⇒ 增量归零，臂停在它此刻该在的地方；不用先停止再启动 | 未 active 直接拒；无位姿可取时拒绝（不兜底 home） |
| ③ 停止 | `POST /api/follow/stop` | 关使能（取流退回 hardware.camera）+ 清 `_baseline_q/_target_q/_active` + **强制广播 `active=false`** 让页面交回控制权 | 即使相机服务已经没了也必须清本地状态（`test_follow_api.stop_clears_service_state_even_when_camera_service_is_gone`） |

停止的文案报的是**实测回到了哪一档**（`capture_width x capture_height`），不是"配置里写的是哪一档"：
这两个在设备拒了档位时会不一样，而运维要看的是相机此刻在跑什么（`follow_service.py:215-220`）。

### 7.7 轮询与广播的节流

- 轮询周期 = `1 / follow.arm.poll_hz`（默认 **20 Hz**，clamp 到 1~50，`follow_service.py:98`）。
  **这个数同时就是页面看到的刷新率**，因为服务是拉 C++ 快照而不是被推。
- 去重键 1：`snap["frames"]` —— C++ 的 `frames` 只在**真正解算过一帧**时才前进，用它当"这批帧
  解过没有"的键最省（`follow_service.py:405`）。
- 闸门 2：`enabled && has_pose && status == "ok"` 才解（`:397`）。`degenerate` **不驱动臂** ——
  那一帧至少有一维没测到。
- 去重键 2：`_emit` 的 `(frames, status, active, ik_failed, tuple(target))` 五元组（`:458-459`），
  只在变化时广播。
- 失败保持上一目标：`best is None` 时**不覆盖 `_target_q`**（`:415-420`）。
  注意 `_last_frames = frames` **在失败分支里也要推进**，否则同一批帧会被反复重试而同步不进去。
- 轮询线程不许因一次异常就死掉（`:377-380`）。
- 超时是两条：`TIMEOUT_CONTROL = 8.0`（档位切换要重启 pipeline、示教要收 N 帧）、
  `TIMEOUT_POLL = 0.5`（**轮询慢了会把页面读数拖成 1 Hz**）（`:37-40`）。

---

## 8. 页面与仿真臂联动的优化

### 8.1 布局与样式

`FollowPanel` 贴在右侧文件列表**底下**、`flex-1` 容器**外面** —— 文件多了要能滚，而这排按钮是
常驻控制，不该被列表挤下去（`InteractiveOp.tsx:1505-1509`）。按钮排与文件列表表头同一套尺寸与
配色（`FollowPanel.tsx:219-220`），三个按钮 `w-6 h-6`，中间用一条竖分隔线把"停止"和"启动/调零"
分开（`:266`）。

### 8.2 只有一个写入者

仿真臂只有一个写入者：`onSimulationJointsChange`。轨迹回放和跟随都往它上面推关节角，
两边同时写等于每帧换一次目标，臂会抖成抽奖（`InteractiveOp.tsx:259-267`）。
**互斥放在入口，不在下游做优先级仲裁**：

- 回放在跑 ⇒ `isReplaying` 进 `FollowPanel`，启动/调零两个键禁用，tooltip 明说
  "轨迹回放正在驱动仿真臂：先停止回放"（`FollowPanel.tsx:198-199, 244`）。
- 跟随在跑 ⇒ `startSimulation` 弹模态挡回放，文案要求**用户显式**停跟随
  （`InteractiveOp.tsx:769-778`）："而不是我们替他停 —— 那样相机那边的示教状态就成了页面上的
  一个意外。"

`WorkspaceView.effectiveRobotState` 在 `simJoints` 为 `null` 时退回真机状态
（`WorkspaceView.tsx:65-66`）—— 这就是"交回控制权"的实现方式：推 `null`，不是推一个home 角。

### 8.3 实时通道的四个优化点

1. **只在关节角真的变了时才惊动 3D viewer**：`lastJointsRef` 存 `joints_deg.join(',')`，
   不等才 `pushRef.current(...)`（`FollowPanel.tsx:107, 131-135`）。后端已经去重了一次，
   这里再去一次是因为**广播帧里很多内容（fps、reason）会变而关节角不变**。
2. **`onFollowJoints` 用 ref 兜住**：父组件每帧都可能重建这个回调，用 ref 才能让 WS 的
   `useEffect` 依赖数组为空、只跑一次（`:103-105`）。
3. **WS 断开固定 2 s 重连**（`:140-144`），无退避。刻意简单：这个通道 20 Hz，一次失败重连
   的成本可忽略，而指数退避会把"后端刚起来"变成"页面上要等几十秒"。**已知粗糙处**，
   如果将来后端重启频繁再改。
4. **连上就先推一帧**（`api.py:105-108`）：页面要能立刻分辨"后端在跑但没启用"和"我根本没连上"。
   所以不需要额外 GET 一次 —— `accept` 之后立刻推 `status()`。

### 8.4 读数行的取舍（不是排版问题）

`Δt`（mm）、`ΔR`（deg）、`σz`、`fps`，外加"·保持"标记（`FollowPanel.tsx:287-319`）：

- **`ΔR` 由 `delta_r` 的 trace 反解**，不拆欧拉（`:86-93`）：`acos((tr-1)/2)`，欧拉在 ±90° 会换
  等价分支，模长不会。
- **σ 只报 z 轴**：沿光轴的重复性是这个量级最关心的一维，四列全展会把注意力从"在不在跟"上
  抢走。`—` 表示本帧没有稠密解算。
- **"保持"必须显出来**（`:292-293`）：臂停住和臂没动在画面上长得一样，处置却完全不同。
  `holding = snap.holding_last_pose || state.ik_failed`（`:215`）—— 两种"停住"的来源在 C++ 侧
  和 Python 侧各一处，必须都并进同一个视觉标记。
- 档位、示教档位差异、当前基线只在 `running` 时显示为一行小字，全文在 `title` 里（`:312-319`）；
  示教档位 ≠ 当前档位时**必须**显示，因为点密度变了会让 §4.8 的门变严（C++ 侧对应
  `follow_worker.cpp:568-575`）。
- 省略号交给 `truncate`，全文在 `title`（`:304-311`）。

---

## 9. 配置参考

### 9.1 生效配置（`configs/aisprayer_config.yaml` 的 `follow:` 块，`:100-163`）

| 键 | 值 | 消费方 | 备注 |
|---|---|---|---|
| `mount` | `eye-to-hand` | C++ | 只接受这一个 |
| `camera.width/height/fps` | `640/480/15` | **两个** | 硬件 D2C 上限档；>640 宽 ⇒ 警告 |
| `camera.lock_path` | `.orbbec.lock` | 相机服务 | 路径以本键为准（§3.1） |
| `camera.first_pair_timeout_ms` | `3000` | 独立工具 | 彩色暖机 333 ms 的余量 |
| `camera.allow_unaligned` | `false` | 独立工具 | `true` 只给排障；生产下 false 是正确性要求 |
| `camera.enable_imu` | `true` | **两条路径** | 独立工具 + 相机服务（`AppConfig.enable_imu`）；见 §5.4 |
| `frontend.kind` | `cpu` | 两个 | `superpoint` 需板端 `.rknn`，本板转不出来 |
| `frontend.max_features/quality_level/min_distance_px` | `200/0.01/16` | 两个 | 预算按"能塞进帧预算"定（§4.4） |
| `track.zmin_m/zmax_m` | `0.30/2.50` | 两个 | 实测数据集深度 0.74~2.59 m |
| `track.depth_stride` | `3` | 两个 | struct 默认是 **4** |
| `track.voxel_m` | `0.015` | 两个 | struct 默认是 **0.03** |
| `track.max_corr_m` | `0.035` | 两个 | **必须和 voxel_m 配对**（§4.6） |
| `track.threads/max_iters` | `4/15` | 两个 | 绑 4 个 A76，不要 8 |
| `track.min_cloud_points/min_gicp_inliers` | `200/80` | 两个 | §4.8 门 4/6 |
| `track.trans_sigma_mm/rot_sigma_deg/group_anisotropy` | `2.0/0.2/15.0` | 两个 | §4.7 §4.8；**未对真实静态重复性重标**（§12.3） |
| `track.min_inlier_ratio` | `0.30` | 两个 | 包络门 |
| `track.max_sparse_streak` | `15` | 两个 | @15fps = 1 s |
| `track.sparse_inlier_dist_m/sparse_min_inliers` | `0.02/12` | 两个 | |
| `teach.map_path` | `follow/out/reference.frmap` | 两个 | |
| `teach.frames` | `10` | 两个 | **三处默认值不一致**（§9.3） |
| `runtime.dry_run/enable_servo_p` | `true/false` | 独立工具 | P5 前 `false` 会被强制回 `true` |
| `runtime.health_port` | `18081` | 独立进程 | **只属于 `follow_node`**（§9.4） |
| `arm.mode` | `sim` | Python（C++ 校验） | `real` ⇒ 警告 + 点击即拒 |
| `arm.home_joints_deg` | `[0,0,-90,-90,-90,0]` | Python | 启动基线 |
| `arm.camera_to_base_fallback_euler_deg` | `[0,0,0]` | Python | 降级路径，会改变方向映射 |
| `arm.poll_hz` | `20` | Python | **就是页面刷新率**；C++ 侧越界只警告并点名 Python 的 clamp |
| `arm.teach_save_map` | `true` | Python→C++ | false ⇒ 只在内存，重启即失 |

### 9.2 为什么臂配置要在 C++ 里读一遍（它根本不发臂）

`config_loader.cpp:312-322`、`config_loader.hpp:73-77` 给出了理由，这条很容易"看起来多余"而被删：

> `follow:` 块是两边共用的一份文件：写错一个键（`mod`、`poll_hz: 二十`）如果没人解析，
> 就要等到有人点了"启动"才炸，而那时的报错已经隔了两层。启动时一次报全，是这份配置唯一的
> 廉价检查点。

C++ **读并校验**，Python **读并使用**。二者都从同一个 yaml 拿，不存在第二份配置文件。

### 9.3 三套默认值不一致 —— 改任何一处都要同步三处

| 量 | struct 默认 | yaml | 代码硬编码 |
|---|---|---|---|
| `depth_stride` | `4`（`odometry.hpp:48`） | `3` | — |
| `voxel_m` | `0.03` | `0.015` | — |
| `max_corr_m` | `0.05` | `0.035` | `reach = 2.6·voxel` 硬编码 |
| `teach_frames` | `1`（`config_loader.hpp:59`） | `10` | 服务 `need = max(1, cfg_.teach_frames)`，**deadline 里按 `need*200`**（`follow_worker.cpp:240-244`） |
| `capture.fps` | `30`（`orbbec_capture.hpp:68`） | `15` | `FollowConfig()` 构造期显式改成 15（`config_loader.hpp:39-43`） |
| 平滑窗口 | `5`（`kDefaultSmoothFrames`） | — | `follow_pose -m` 可覆盖（1~30） |

`capture.fps` 那一行注释就是这类坑的标准处置：**内置默认必须是量出来的那一档**，
"没有配置文件的机器不能悄悄跑成超预算"。新增键时照这个模式做：struct 默认 = 实测默认 =
yaml 值，做不到就在 `Config.DefaultsAreSelfConsistent` / `Config.RepoConfigIsValid` 里写明为什么。

### 9.4 两个端口，别混

| 端口 | 谁 | 说明 |
|---|---|---|
| **18080** | 相机服务 | follow 的观测口是 `/api/v1/camera/follow/status`，**不是** 18081 |
| **18081** | `follow_node` 的 `HealthServer` | 只有独立进程用（`health_server.hpp:1-8`） |
| 8008 | ZLM | 视频分发 |

---

## 10. 实测数据（RK3588）

| 项 | 数值 | 来源 |
|---|---|---|
| 前端全流程 1280x800 / 400 特征 / patch12 | **93 ms** | `frontend.hpp:22-24`、`bench_frontend` |
| 前端 848x480 / 200 特征 / patch10 | ≈14 ms 检测 + 24 ms 描述子 | 同上 |
| 单帧预算 @15 fps | **66.7 ms** | — |
| GICP 一帧 p50 | **3.9–6.6 ms** | `follow_replay` 分档耗时表 |
| 硬件 D2C | 0% 主机 CPU（640x480 档） | `query_hw_d2c` / `configureAndStartPipeline()` 的 `ALIGN_D2C_HW_MODE` 分支 |
| 软件 D2C | 848x480 ≈16–22 ms；1280x800 ≈29–35 ms | — |
| 静止单帧逐轴噪声 sd | **~2 mm** | `pose_smoother.hpp:10-13` |
| `s²` 正常工位 | `2e-5 ~ 9e-5`；真大平面也有 `6e-6` | `odometry.hpp:85-88` |
| 内参（`out/real/param.txt`） | `fx 611.684 / fy 611.698 / cx 643.429 / cy 405.153` | 1280x800 档 |
| D2C 基线差 | `x = -23.7351 mm` | `orbbec_capture.hpp:57`；守卫理由 `follow_worker.cpp:213-218` |
| 彩色-深度时间差 | 常量 `+0.347 ms` | `orbbec_capture.hpp:90` |
| 彩色暖机 | 约 `333 ms` | `orbbec_capture.hpp:69` |
| σ 自报 vs 真实散布 | 比值 **27.9× / 4.4× / 10.7×**（三轴） | `follow_pose` 的"静止重复性 vs σ 自报"段 |

最后一行是 §12.3 那条待办的原始证据。

---

## 11. 验证与测试：怎么证明没改坏

### 11.1 无相机、无臂的门（**每次提交前跑这些就够**）

```bash
# C++ 算法与配置（4 个套件 55 条 TEST，≈4.4 s）
cd follow/build && ctest --output-on-failure

# 回归门槛（合成数据集，10 用例 / 98 帧）—— 注意必须从仓库根跑
follow/build/follow_replay --root out/synth

# Python 后端（39 条：mirror 29 + api 10，≈0.5 s）
cd app/src && app/.venv/bin/python -m unittest \
    apps.follow.services.test_follow_mirror apps.follow.services.test_follow_api

# 手眼几何（欧拉约定共用的那一层）
cd app/src && app/.venv/bin/python -m unittest core.handeye.test_hand_eye
# 运动学（unittest discover 会因 helper 包报 ImportError，直接点名模块）
cd app/src && app/.venv/bin/python -m core.hardware.robot.test_cr5_kinematics
```

`follow_replay` 必须从仓库根目录跑 —— 数据集在 `./out/synth`，从 `follow/build` 里跑会找不到。
门是 `|Δt|p95 < 2.0 mm`、`|ΔR|p95 < 0.2°`，**且**"状态违规 0 条"（`replay_main.cpp:300-307`）。

### 11.2 `follow_pose`：随时可验证的那一条路

```bash
cd follow/build && ./follow_pose            # 有相机时；默认 dry_run，只打印不发臂
./follow_pose -h                            # -t 死区 mm / -r 死区 deg / -m 平滑帧数
```

关键参数：`kDeadbandTmm = 1.0`、`kDeadbandRdeg = 0.1`、`smooth = 5`、心跳 5 s、
静止窗口 `kStillRangeMm = 5.0 / kStillRangeDeg = 0.5`（`pose_main.cpp:72-73, 131-133`）。
收尾会打印"静止重复性 vs σ 自报"的比值表（`:485-505`），判据是比值 > 2 就提示 σ 门该重标。

**为什么必须留着它**（`configs/aisprayer_config.yaml` 的 `follow:` 段头注释里也写了）：它是
`follow_pose` 与相机服务**共用同一份 libfollow** 的证明。页面出问题时先跑一遍 `follow_pose`：
控制台读数与 `/follow/status` 的 `pose_mm/pose_rpy_deg` 不一致，就说明**其中一边接错了**，
而不是"再调调参数"。这条判断省掉的排查时间是这套架构存在的主要理由之一。

### 11.3 需要相机在场的验证清单（**硬件稀缺，攒够一批再上机**）

- 使能后 `/follow/status` 的 `capture_width/height` 是否真到 640x480、`align == "hw"`。
- 示教 10 帧收帧是否在 deadline 内、`map_hash`/`map_voxels` 合理、`.frmap` 落盘可读回。
- 缓慢平移/旋转工件时 `estimator` 是否稳定在 `gicp`、`sigma_t_mm` 是否 < 2 mm。
- 遮挡 → 恢复：`sparse` 连击是否 ≤15、超限时是否 `lost` 且位姿保持。
- 手动快速晃动相机时 `frames/dropped/rejected` 与 `compute_ms` 的关系。
- 使能后 `/follow/status` 的 `gyro.time_ready=true`、`gyro.samples` 在晃相机时 > 0
  （时间基已离线钉死，这条确认设备在场时定标真的完成）。
- 页面：三按钮各点一次、刷新页面后 WS 重连并恢复显示、回放/跟随互斥两条路径。

### 11.4 测试覆盖到了什么 / **没**覆盖什么

`test_core`(22) / `test_registration`(16) / `test_config`(17) / `test_small_gicp_api`（4 条特征化）。
Python 29 + 10 条。命名测试点里明确钉住的契约：单位口径、NaN 拦截、冻结地图不被实时帧污染、
退化不当 ok、出包络不当 lost、时间戳倒退、坏输入不被当参考帧、示教落盘往返、
自重建可复现、真实帧前端耗时。

**没有自动化覆盖的（别以为测过了）**：

- C++ `FollowWorker` 的任何行为（无 mock 的 `CameraDriver`，只能上机或用 `/follow/status` 手测）。
- HTTP 端点的 400/409/503 分支（只测了 Python 侧的分类）。
- 前端组件（无 JS 测试）。
- 陀螺第 2 档在**真实相机抖动**下的收益（离线接缝已测，上机只验证定标完成）。
- 多帧示教在真实散斑下的收益（合成数据的散斑模型不真）。

---

## 12. 已知缺口与维护红线

### 12.1 缺口清单（按影响排序）

| # | 缺口 | 影响 | 位置 |
|---|---|---|---|
| 1 | **快照陈旧无门限** | `_poll_once` 只看 `frames` 变没变；C++ 卡死但 HTTP 还活着 ⇒ 页面显示一个"活着但其实没动"的数。`snapshot_ts_ms` 字段已经有了，没人用它 | `follow_service.py:396-406`；`follow_worker.hpp:94` |
| 2 | **臂配置只在启动时解析一次** | 改 yaml 后必须重启 app 后端，页面上没有任何提示 | `follow_service.py:81` |
| 3 | ~~`gyro_used` 不可观测 + 时间域存疑~~ **已修** | 时间基见 §5.5；快照 `gyro.*` 组 | `gyro_time_base.*` / `followSnapshotJson` |
| 4 | ~~`stop()` 不清 `_last_error`~~ **已修** | 停止时清掉，成功/失败都清 | `follow_service.py` `stop()` |
| 5 | `_emit` 去重键不含 `reason` | `reason` 单独变化时不广播（页面 tooltip 会旧一拍） | `follow_service.py:458-459` |
| 6 | **eye-in-hand 的语义在两侧不一致** | C++ `mount` 只接受 `eye-to-hand`；Python 的降级文案里提到它 ⇒ 读到那句话的人会以为有这个模式 | `follow_service.py:125-126` |
| 7 | 无奇异性/速度与加速度检查 | 仿真臂无妨；真机 P5 必须先有 | — |
| 8 | `forward_all` 是 stub ⇒ **雅可比不可用** | 现在没法做可操作度判据，也不能用它来判断奇异 | `cr5_kinematics` |
| 9 | 真实臂控制路径（**P5**） | 见 §12.2 | `follow_service.py:224-230` |
| 10 | `device_gone` 状态**没有任何产生点** | 8 个状态词里有 1 个永远不出现。拔电相机的表现是 `no_frame` + `connected=false`，不是 `device_gone` | `types.hpp:42,54` |
| 11 | 前端未用的字段 | `FollowPanel` 的 TS 接口里列了 `map_hash/map_voxels/teach_*` 等但当前 UI 不显示 —— 不是 bug，是"字段先到位，页面按需取"，但会误导 | `FollowPanel.tsx:19-53` |

### 12.2 P5（真实臂）要补的东西，按顺序

1. `FollowController`：`T_base_camera` 从标定来（乘在 `to_dobot` **之前**，`pose_io.hpp:12-13` 明确
   说了本文件不管坐标系换算）。
2. **关节反馈换算**：控制器 J2/J4 与 URDF 差 **±90°**（§7.5 末），不做这步就别把真机角度当基线。
3. 发送节流与限速：`ServoP` 有最小发送间隔；当前 `_emit` 的节流粒度是 20 Hz 且只去重不改速率。
4. 奇异性 / 可操作度：需要真雅可比（§12.1 #8）。
5. `dry_run` 与 `enable_servo_p` 两道闸必须**都在**才算"要发"，任一为假只打印。
6. 断连保护：`last_failure_upstream` 目前只影响 HTTP 状态码，P5 时它必须是"停发"的条件。

### 12.3 σ 门限该重标一次（有数据，缺结论）

`follow_pose` 实测静止单帧散布与自报 1σ 的比值是 **27.9× / 4.4× / 10.7×**（三轴，§10 末行）。
方向是"σ 自报偏乐观"。但这是**在示教档位、静态、单一场景**下量的，且静止散布里混着平滑窗口
残余与显示链路量化。要做的事：

1. 上机跑一次 ≥5 min 的静止 `follow_pose`，取"静止重复性 vs σ 自报"段（`pose_main.cpp:485-505`）；
2. 用真实档位（640x480）而不是合成数据重算门限；
3. 改 `max_trans_sigma_mm / max_rot_sigma_deg / max_group_anisotropy`
   （`odometry.hpp:81-83` ↔ `configs/aisprayer_config.yaml:130-132`）**同时**更新 `odometry.hpp:64-88`
   里那张实测表 —— 注释里明写"改门限时重跑测试，别改注释"，指的就是 `NoiseSdMatchesPredictedSigma`。
   **平移比值门的余量最薄（10.1 对 15，只有 1.5 倍）**，真实扫描的表面粗糙度和 D2C 边缘坏点都
   会把它往上推（`odometry.hpp:79-80`）。

### 12.4 注释与代码已经漂移的三处（**按维护提示处理，不是 bug**）

| 位置 | 说的 | 实际 | 处理 |
|---|---|---|---|
| `odometry.hpp:99` | "P4 之前不接实机数据，接口和单测先留着" | 两条路径都在喂真陀螺 | 注释该更新（含 `max_gap`/`gyro_buf_max` 的真实地位） |
| `odometry.hpp:133` | "P4 之前没有调用方" | `node_main.cpp:390`、`follow_worker.cpp:494` 都是调用方 | 同上 |
| `follow_worker.hpp` 顶注释 | 曾写"拿不到厂商 SDK 的设备时间戳" | 跟踪已走 `track_ts_ns`；`timestamp_ms` 仍是到达时刻 | 头注释已改成 §5.5 那套 |

发现新漂移就往这张表里加一行，别默默改代码留旧注释 —— 这套代码的注释是**判据的一部分**，
好几条注释本身就对应一条断言。

### 12.5 维护红线（六条，每条都是踩出来的）

1. **`libfollow` 不许 include `orbbec_capture.hpp`**，也不许出现 mm/deg（只有 `pose_io.*` 允许）。
   它不依赖设备是"没有相机也能验证"的前提。
2. **不新建第二份取流实现、第二个 `ob::Pipeline`、第二个健康端口**（§6.1）。
3. **示教、平滑、`to_dobot` 必须与 `follow_pose` 共用同一份实现** —— "给人读的那个数必须由同一段
   代码算出来"，否则"演示里看着稳"不再蕴含"页面上看着稳"（`pose_smoother.hpp:1-8`）。
4. **不在欧拉角上做乘法**；跨坐标系只走 4×4。
5. **失败时保持上一目标**：不夹位、不缩增量、不跳 home。宁可臂停住，也不要它朝一个没测到的方向走。
6. **一个可选功能的配置错不许拖垮主功能**：follow 配置坏了 ⇒ `setBlocked` + 相机服务照常跑（§6.6）。

---

## 13. 排障手册（现象 → 判据 → 代码位置）

### 13.1 先分诊：数在哪一跳断了

```bash
curl -s localhost:18080/api/v1/camera/follow/status | python3 -m json.tool | head -20
curl -s localhost:8000/api/follow/status | python3 -m json.tool | head -20   # 8000 = app/src/main.py:132
```

第一个有、第二个没有 ⇒ 断在 C++→Python（后端/`CPP_BASE_URL`/503）。
两个都有但 `frames` 不涨 ⇒ 断在取流或 worker。`frames` 涨而页面臂不动 ⇒ 断在 WS 或前端所有权。

### 13.2 对照表

| 现象 | 判据（看哪个字段） | 位置 |
|---|---|---|
| 点"启动"报 503 + "相机服务无响应" | 后端 `/follow/status` 都拿不到 | `follow_service.py:149-161`；`api.py:41-51` |
| 点"启动"报 400 且说 `mode` | 配置是 `real` | `follow_service.py:224-230` |
| 400 且说"拿不到基座←相机轴映射" | `r_cb_ready=false`；`r_cb_source` 就是原因 | `follow_service.py:231-234` |
| `status: "disabled"`、reason 说标定模式 | `in_calib` | `follow_worker.cpp:405-406` |
| `status: "disabled"`、reason 说"配置不可用" | 被 `setBlocked` | `follow_worker.cpp:78-90, 408-410` |
| `status: "no_map"` | 没示教，或 Tracker 重建时地图为空 | `follow_worker.cpp:419-436` |
| `status: "no_frame"` 且 `connected: true` | 相机在出流但取不到**新**帧：档位没起来、或 `getLatestFrame` 一直被主循环吃掉 | `follow_worker.cpp:438-455` |
| `status: "config_invalid"` + reason 提到内参 | 内参未从设备读到 / 内参与帧尺寸不符（换档瞬间最常见） | `follow_worker.cpp:199-231` |
| reason 提到 `align=disabled` | 对齐阶梯落到底，深度不在彩色像素系里 ⇒ **这是错位不是降精度** | `follow_worker.cpp:213-218` |
| 反复 `out_of_envelope` | `inlier_ratio < 0.30` ⇒ **要重新示教**，不是跟丢；调零即可 | `odometry.cpp:227-229` |
| `degenerate` | 至少一维 σ 超门或各向异性超 15；`sigma_*` 里 `null` = 无估计 | `odometry.cpp:237-243` |
| 页面显示"·保持" | `holding_last_pose`（C++ 保持）**或** `ik_failed`（Python 保持）—— 两者处置完全不同 | `follow_worker.cpp:558-566`；`follow_service.py:415-420` |
| 臂不动但 `Δt` 在变 | 大概率 `ik_failed`（目标够不着）。基线是不是不对？`arm_baseline_deg` 看一眼 | `mirror.py:155-160` |
| `dropped` 一直涨 | 解一帧超 66 ms，或同毫秒两帧（取流在抖） | `follow_worker.cpp:477-486` |
| `rejected` 一直涨 | 守卫长期不过（换档、拔线、标定模式）。同一原因只喊一次，日志里能看到第一条 | `follow_worker.cpp:457-475` |
| 示教报"收帧超时 k/N" | `deadline = 1000 + need*200 ms`；相机在出流吗、档位被标定模式顶掉了 | `follow_worker.cpp:242-268` |
| 停止后页面臂卡住 | `active=false` 的强制广播没到 / WS 没重连 / 回放占着臂 | `follow_service.py:212`；`FollowPanel.tsx:140-159` |
| 页面读数与 `follow_pose` 不一致 | **架构级故障**：两边应共用同一份 libfollow，不一致即其中一边接错了 | §11.2 |
| `sigma_*` 全是 `null` | 没有稠密解（`estimator: sparse/none`），不是"精度极高" | `http_server.cpp:58-65` |
| 换工件但 `map_hash` 没变 | 示教没生效，或沿用了旧 `.frmap` | `reference_map.cpp:19` 哈希语义 |
| 重启后行为变了而配置没改 | `describe()` 打进日志的生效配置对一遍；注意 §9.3 三套默认值 | `config_loader.cpp` 的 `describe` |

### 13.3 日志里该看到的三条

1. 启动时 `Follow` 的生效配置 dump（`describe()`，含 `arm : mode=… poll=…Hz` 一行）。
2. `>>> follow 使能：取流切到 640x480 @ 15fps`（`setFollowProfile()` 里的 `LOG_INFO("Camera", ">>> follow ")`）。
3. 运行期跟随遥测，格式与 `follow_pose` 对齐（`follow_worker.cpp:576-623`）：节流条件是
   "首次 / 状态变化 / \|Δ\|>0.5 mm 或 0.1° / 每 1 s 心跳"。

---

## 14. 改动检查清单

**动算法（`follow/src`）**
- [ ] `cd follow/build && ctest` 全绿
- [ ] `follow_replay --root out/synth` 全过门（从仓库根跑）
- [ ] 若改了门限/σ：同步 `odometry.hpp:64-88` 的实测表，并重跑打印那张表的测试
- [ ] 若改了状态词：§4.8 表 + `follow_worker.cpp:558-575` + §13.2 三处同步

**动集成（`follow_worker.*` / `camera_driver.*` / `http_server.*`）**
- [ ] 相机服务照常构建（**别用 `grep -icE "warning|error"` 判断构建成败**，零匹配时 grep 退出码 1
      会让干净的构建看着像失败；看 `Built target orbbec_camera_service`）
- [ ] 新快照字段：`FollowSnapshot` + `followSnapshotJson` + TS interface 三处同时改
- [ ] 没引入第二个 Pipeline、第二个健康端口、`follow_device`/`follow_health` 链接
- [ ] 需要上机的项攒进 §11.3 一次做完

**动 Python 侧（`apps/follow`）**
- [ ] `app/.venv/bin/python -m unittest apps.follow.services.test_follow_mirror apps.follow.services.test_follow_api`
- [ ] 碰 `kin` 的新路径进 `_kin_lock`
- [ ] 新错误分类：503 还是 400？`_fail_upstream` 设了吗？
- [ ] 失败时臂**保持上一目标**，不夹位不缩增量
- [ ] 新配置键：C++ 读并校验 + Python 读并使用 + 三处默认值对齐（§9.3）

**动前端**
- [ ] 保持"只有一个写入者"（§8.2）
- [ ] 关节角去重推帧仍在（`lastJointsRef`）
- [ ] 卸载交还 `null`，停止交还 `null`
- [ ] 无 JS 测试 ⇒ 必须手点一遍三按钮 + 刷新 + 回放互斥两条路径，并说明你点过了

**收尾**
- [ ] `follow_pose` 的读数与 `/follow/status` 的 `pose_mm/pose_rpy_deg` 对得上
- [ ] 本文件的 §12.4 漂移表是否需要加行
- [ ] **不要提交**（提交由用户做）

---

## 15. 术语表

| 词 | 指什么 |
|---|---|
| **参考系** | 示教那一刻的相机系。`T_ref_cam` 的 a 侧 |
| **档位** | 取流的分辨率/帧率/对齐方式组合：`hardware.camera` 档 vs `follow.camera` 档 |
| **包络** | 当前点云与冻结参考几何的重叠度；出包络 = 场景相对参考不见了，要重新示教 |
| **退化 (degenerate)** | 解算"成功"但至少一个自由度本帧没被测到 —— 绝不能当 ok 用 |
| **保持 (hold)** | 故障时输出上一个可信位姿；页面上标成"·保持" |
| **基线 (baseline)** | 示教那一刻臂的关节角；目标 = 增量 · 基线。**不是**实时位姿 |
| **镜像 (mirror)** | 相机增量映射到臂基座系并解 IK 这一段数学（`mirror.py`） |
| **R_cb** | 相机轴 → 基座轴的旋转，`T_base_camera` 的旋转块 |
| **降级 (fallback)** | 没有标定结果时用配置常量当 R_cb；**必须在状态里可见** |
| **两个消费方** | `follow_pose`（验证/量测）与相机服务 `FollowWorker`（生产），共用 libfollow |
