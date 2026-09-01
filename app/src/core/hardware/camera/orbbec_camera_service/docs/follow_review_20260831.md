# Follow 功能全面复查报告（2026-08-31 轮次）

> 复查对象：`follow/`（算法库）、`app/src/core/hardware/camera/orbbec_camera_service/`（实时数据面）、
> `app/src/apps/follow/` + `app/frontend/.../FollowPanel.tsx`（接入层），以及板上留存的实测日志。
> 本文只写**新发现与已被证据推翻的旧结论**，不复述 `follow_system_design.md` 里仍然成立的设定。
> 每条都带可点开的证据；标 **[已核实]** 的是本次亲自读码或跑出命令得到的，
> 标 **[待上机]** 的是只有静态证据、需要相机在场复测的。

> **⚠ 先读 §11（9/1 第二轮）。** 第二轮做了两件第一轮没做的事：
> ① **重建后再跑门禁** —— 于是发现 §2 的 **P0-1「红门」是假警报**（我跑的是 19:34 的旧 `test_config`，
> 而 `test_config.cpp` 23:37 已同步；重建后 `ctest` 4/4 全绿），并且**§5.4 里"S>0 ⇒ 时间域对齐成立"
> 这条推论被证伪**（实测 `S/P = 0.611`，恒定，见 §11.2）。
> ② 把最后一次运行（新二进制 23:43:55 构建、进程 23:45:02 启动，**晚于 `9dd6bef` 全部源码改动**）
> 单独切出来做统计 —— 这是**陀螺链路唯一一份现场数据**。
> 凡 §0/§2/§5 与 §11 冲突的地方，**以 §11 为准**。

---

## 0. 一页结论

| 维度 | 判定 | 关键证据 |
|---|---|---|
| 算法内核正确性 | **好**（时间域切换丢帧只是潜在路径，新 run 里 `dropped=0`） | `follow_replay` 10 用例 98 帧全 PASS；`odometry.cpp:202` 与 `follow_worker.cpp:700` 的分歧 |
| 回归门禁 | ~~当前是红的~~ **绿**（第二轮重建后 4/4 过，§11.1） | `cmake --build . -j4 && ctest` → 100%；第一轮的失败是 19:34 的旧 `test_config` |
| 实时预算 | **不合格但已明显改善**：旧构建 14.3% → 新构建 **6.2%** 超 66.6ms | 旧窗 16,726 行 p50 51/p95 88/max 220；新窗 1,575 行 p50 51/p95 69/max 109（§11.5） |
| 端到端反馈 | **差**：退化时页面反而更像"跟住了"（第二轮复核代码仍是原样） | `follow_service.py:622` + `:709-711` 双重早退；新 run 里 `ik_failed` 仍有 7 次只落 WARN |
| 取流稳定性 | **明显改善**：新 run 里停滞归零，但样本只有 10 分钟 | `帧流已停滞` 旧窗 175 → 新窗 **0**；`dropped=0 rejected=0` 每个 10s 快照都是 |
| 陀螺/时间域新链路 | **有新 P0**：对齐"看起来"成立，实测积分只覆盖 **61%** 的帧间旋转 | 1,575/1,575 行带 `gyro=`；`S/P = 0.611`，静止/运动/快转/慢转四层全是同一个数（§11.2） |

**最该先做的三件事（第二轮修订）**：① 先做**陀螺时间尺度的三数自述**（帧设备戳间隔 / 陀螺戳平均间隔 / 主机到达间隔），
把 §11.2 那个 1.63× 的常数钉死成"设备钟 vs 交付"哪一种，然后**整条链路改在设备时间域跑**，
顺带删掉 `GyroTimeBase` 的 354~460 ms 抖动与 133 ms 台阶（§11.3）；
② 阶段耗时 + 绑核（新 run 的 `compute_ms` 仍是单一聚合数，`[Status]` 里也没有 follow 分阶段）；
③ 让"状态退化"必然产生一次对外广播（与第一轮同一条，代码未变）。

---

## 1. 本次实际执行过的验证（可复现）

| 动作 | 结果 |
|---|---|
| `cd follow/build && ctest` | **75% 通过**；`config` 套件失败，失败点 `test_config.cpp:339/340/343` |
| `follow/build/follow_replay --root out/synth`（仓库根跑） | PASS：98 帧 / 计分 80 帧 / 0 用例未过门，状态违规 0 条 |
| `follow/build/follow_bench_frontend follow/out/real` + `taskset` | 见 §5.2 的大小核矩阵 |
| `app/.venv/bin/python -m unittest`（follow 三个套件） | **70 条全绿**，0.41 s |
| `orbbec_camera_service/build/test_gyro_time_base` / `test_pose_broker` | 9 条 + 7 条全绿 |
| 日志解析 | `app/logs/backend.log.2026-08-31` 18,925 条逐帧遥测；`backend.log`（9/1 00:00–00:06，新构建）604 条 |
| 硬件事实 | `/proc/device-tree/model` = **Orange Pi 5 Plus（RK3588）**；cpu0-3 = Cortex-A55 @1.8 GHz，cpu4-7 = Cortex-A76 @2.35 GHz（`midr_el1` 0xd05/0xd0b）；两档 governor 均为 `performance` |

**必须记住的一条口径**：最近两次提交（`392dd78` 08-31 19:36、`9dd6bef` 09-01 00:04，主题为陀螺时间域对齐与
`enable_imu` 生效）**晚于几乎所有可用日志**。所以本报告里凡是拿日志数字说话的性能/状态结论，
描述的是**修复前的现场**；只有 §4 里关于时间域的那批问题是修复后代码里读出来的，属于 **[待上机]**。

---

## 2. P0 清单（今天就该动手）

### ~~P0-1 提交前必跑的门是红的~~ **[已撤回 — 假警报，见 §11.1]**

第一轮记录到的失败**是真的发生过**，但不是仓库状态，是我用了旧构建产物：

```
$ cd follow/build && ctest          # ← test_config 可执行文件是 19:34 构建的
75% tests passed, 1 tests failed out of 4
  FAILED  Config.RepoConfigIsValid
    enter_rad_s = 0.007854  vs  0.80°/s    exit_rad_s = 0.010472  vs  1.50°/s    max_freeze_ms 30000 vs 0
$ cmake --build . -j4 && ctest      # ← 重建后
100% tests passed, 0 tests failed out of 4     (small_gicp_api / core / registration / config)
```

根因：`9dd6bef` 自己就在 `follow/test/test_config.cpp`（mtime 23:37:03）里把 `:365/:366/:369` 三个期望改成了
`0.45 * kDeg2Rad` / `0.60 * kDeg2Rad` / `30000`，而我跑的二进制停在 19:34。**这条在提交时已经是绿的。**

仍然成立的一半（降级为 P2 备注）：`gyro_filter.hpp:41-49` 与 `odometry.hpp:129` 的结构体默认
（enter 0.008 / **exit 0.017 / bootstrap 1000 / freeze 1500**）与生产 yaml（0.45°/s、0.60°/s、100、30000）
依然不是一套数。现在这条已经被测试**显式钉住并写了注释**（"两边数值本来就不同…撞对了才会绿"），
所以不再是"无人知道的漂移"；真正会踩到它的是**不走 yaml 的构造路径**（`follow_node`、单测里的
`with_defaults()`），那些地方 `bias_bootstrap_samples=1000` ≈ 5 s 零偏窗，与 `configs` 注释里
"5 s 太长，会把拿起相机的运动吸进零偏"的论证直接冲突。

**动作**：收敛成一处 `kDefault*` 常量表；并把"**先 `cmake --build` 再 `ctest`**"写进验收命令（见 §10）。


### P0-2 帧预算没有余量，且尾部是"阵发性击穿" **[已核实，37,226 帧]**

```
合并样本 n=37,226   p50 51.0  p90 74.0  p95 87.0  p99 102.0  max 220.0   mean 54.3 ms
> 66.6 ms（15 fps 周期）：5,043 帧 = 13.55%        > 80 ms：7.27%
分文件：
  backend.log.2026-08-31  n=18,925  p50 51  p95 87  max 220   超时 13.5%
  backend.log（9/1 新构建） n=   604  p50 49  p95 61  max  79   超时  1.0%
```

三个关键判断：

1. **不是"运动时慢"**：静默合并行（≈静止）超时率 13.8%，非静默行 13.4% —— 与运动状态无关。
2. **是争抢**：按分钟分桶，好的分钟 0–2%，坏的分钟 37% / 55% / 85%（17:48、18:10、22:23…）。
   坏分钟里 `WARNING` 行数是正常分钟的 **26 倍**（181 vs 7）。同一时刻系统在干别的事。
3. **失败路径比成功路径贵得多**（日志 agent 交叉表，口径为逐行代表值）：
   `ok/gicp` p50 50ms、超时 4.3%；**`ok/sparse` p50 84ms、超时 86.8%、max 220ms**。
   原因清楚：走稀疏替补时，GICP 的 25 轮迭代已经白烧完，又叠上 matching + RANSAC。

**这条的根因不在算法，在"没有任何一处能告诉你是哪一段慢"**：全链路只有一个总量字段
`snap_.compute_ms`（`follow_worker.cpp:788`），前端 / 反投影 / GICP / 稀疏各占多少，日志和快照里都不存在。
性能工作必须先补这一步，否则每一次"优化"都无法证伪。

### P0-3 声称的绑核根本不存在，小核代价实测 5.0× **[已核实]**

`follow/include/follow/odometry.hpp:58` 写"threads=4 绑 A76，不要 8"，`configs/aisprayer_config.yaml:126`
重复同一句话，`corner_detector.cpp:44` 写"Limit OpenCV CPU concurrency to 2 threads on Cortex-A76 big cores"。

**全仓库没有任何 `sched_setaffinity` / `pthread_setaffinity_np` / `GOMP_CPU_AFFINITY` / `OMP_PLACES` /
`taskset`**（grep 确认，first-party 代码零命中；`run.sh` 也没有）。三处注释描述的是一个不存在的机制。

用真实工位图实测 CPU 特征前端（`follow_bench_frontend`，同一张图只改可用核）：

| maxfeat | 核配置 | 640x400 | 844x528 | 1280x800 |
|---|---|---|---|---|
| 200 | 4×A76（4-7） | **19.7 ms** | 30.6 | 88.4 |
| 200 | 2×A76（4,5） | 26.8 ms | 38.4 | 96.1 |
| 200 | 4×A55（0-3） | **99.0 ms** | 160.3 | 408.4 |
| 200 | 1×A55（0） | **184.5 ms** | 253.9 | 511.2 |

同一份工作飘到 A55 是 **5.0×**，飘到单核是 **9.4×**。640x480@15 的整帧预算是 66.6 ms，
而前端一项在 A55 上就要 99 ms —— 日志里 max=220 ms 的那几帧与此完全同签名。

更值得注意的结构性问题：**`track.threads` 管不到真正的大头**。它只喂给 small_gicp
（`follow/src/odometry.cpp:47` 的 `preprocess_points` 与 `:62` 的 `reduction.num_threads`），
而占一帧 1/3～1/2 的 OpenCV 前端，其并行度是被**无关模块** `CornerDetector::init()` 里的
全局 `cv::setNumThreads(2)`（`corner_detector.cpp:45`）决定的 —— 也就是表格里 26.8 ms 那一行，
而不是 19.7 ms 那一行。 follow 的算力由标定模块顺手定，且没有任何一处文档说明这层依赖。

**动作（性价比最高的一刀）**：`FollowWorker::loop()` 入口 `pthread_setaffinity_np` 到 cpu4-7、
GICP 的 OpenMP 显式 `omp_set_num_threads(4)` + `OMP_PROC_BIND=close`、把 `cv::setNumThreads` 的所有权
收到服务启动处并写明"follow 前端依赖它"，同时把 `corner_detector` 的全局副作用改成局部 `cv::parallel_` 范围。
预期：尾部从 p95 87ms 拉回 ~61ms 一档（9/1 新构建实测值），单次改动 -7 ms/帧。

---

## 3. P1 清单（正确性与反馈）

### 3.1 陀螺时间域（修复后代码里的新风险） **[待上机]**

| # | 结论 | 证据 | 动作 |
|---|---|---|---|
| 1 | **定标完成那一帧只重置了 worker 的时间戳，没重建 Tracker** ⇒ 每次使能确定性丢约 6 帧 | `follow_worker.cpp:700-702` 只置 `last_ts_ns_=0`；而 `odometry.cpp:202` 判的是 Tracker 内部 `prev_ts_ns_`。钟差一步从"到达时刻"跳到"设备换算时刻"（早约 440 ms ⇒ ≈6 帧 @15 fps），期间连续 `kStaleInput`，且这些帧仍被计入 `frames_`（`:728`）⇒ **fps 虚高** | 检测到 `using_device_ts_` 翻转时同时置 `tracker_dirty_`；或给 `Tracker` 加 `reset_timestamps()` |
| 2 | **时间基被重新定标，上层完全不知情**：每次停流/软重启都 `reset()`，Tracker 的陀螺缓冲里会同时存在两套 offset ⇒ 非单调 ⇒ `types.hpp:124` 在第一条旧域样本处截断，①②两条用途一起失效，最长要等 `gyro_buf_max=4096`（≈20 s）把它挤出去 | `camera_driver.cpp:71`（reset 点）；`follow_worker.cpp:349-378`（没有对应的重建触发） | `GyroTimeBase` 暴露单调 epoch，worker 每帧比对并重建 Tracker |
| 3 | **积分窗口"框向未来"没有防护，而且现有自检查不到它**：offset 取 8 对候选的**最小值**，定标后 USB 延迟一旦变小，`track_ts_ns` 会跑到已到达样本的前沿之前 ⇒ `GyroDelta::stale=true`，但 `samples_used>0` ⇒ `checkGyroChannel` 的告警判据永远不触发 | `gyro_time_base.cpp:22,31`；`follow/include/follow/types.hpp:138-140`；`follow_worker.cpp:365-380` | 分位数+余量代替纯最小值；把 `stale` / `gap_end_ns` 放进 `TrackResult` 并上报 |
| 4 | **示教与跟踪抢同一条陀螺队列**：`teach()` 与 `loop()` 各自 `drainGyroSamples()`（清空式），示教的 1~3 s 窗口里跟踪侧必然 `samples_used==0` | `follow_worker.cpp:412,434` vs `:719`；注释 `:402-403` 只论证了示教侧够用，没论证被抢的一方 | 双订阅（peek 不 clear），或示教期间显式声明跟踪降级 |
| 5 | **重建 Tracker 连陀螺零偏一起销毁**：换档/换图都重建，而 `GyroStillDetector` 内嵌在 Tracker 里 ⇒ 零偏定标从头开始（默认 1000 样本≈5 s） | `follow_worker.cpp:561-583`；`gyro_filter.hpp:49` | 偏置与样本缓冲外置到 worker 持有 |

### 3.2 快照字段在说谎（下游只能靠巧合躲过）

| # | 结论 | 证据 |
|---|---|---|
| 6 | `holding_last_pose = (status != kOk)` 把 **`kDegenerate` 标成"保持旧位姿"**，但退化帧是**本帧新采纳的解** | `follow_worker.cpp:766` vs `odometry.cpp:341 adopt(...)`；定义见 `follow_worker.hpp:83` |
| 7 | `has_pose` 头文件写"至少解出过一个可信位姿"，实现是"本帧解出了"。app 侧靠 `has_pose && status=="ok"` 双条件才侥幸没错 —— **安全依赖下游多查一个字段** | `follow_worker.hpp:57` vs `follow_worker.cpp:765`；`follow_service.py:612` |
| 8 | `snapshot_ts_ms` 是 C++ 进程的 `steady_clock`，却被注释成"app 据此判 stale"（`follow_worker.hpp:116`）。**跨进程不可比**，Python 实际用的是自己收到的时刻 | `follow_worker.cpp:625` 等；`http_server.cpp:189` 原样外发 |
| ⇒ | **正确修法**：C++ 侧在下发前算好 `snapshot_age_ms = steady_now_ms() - snapshot_ts_ms` 并只发这个派生量。设计文档 §12.1 缺口 #1 提的"用 `snapshot_ts_ms`"是做不到对的，照原样修会写出一个永远为假的判据 | |

### 3.3 平滑与恢复

| # | 结论 | 证据 | 动作 |
|---|---|---|---|
| 9 | **平滑窗口被未采纳帧灌满**：`smoother_.push(r.T_ref_cam)` 无条件执行，`lost/rot_gated/out_of_envelope` 推的都是同一个 `T_last_good_` ⇒ N=5 窗口被复制品占满，恢复后修正量被稀释 4/5，输出以约 5 帧斜坡爬回，**且这段滞后不体现在任何字段上** | `follow_worker.cpp:747` | 仅 `estimator != kNone` 时 push，或 hold 计数达阈值时清窗 |
| 10 | 空窗口 `value()` 返回 inf/NaN（`1.0/buf_.size()`），契约只写在注释里 | `pose_smoother.hpp:46`、`:38`；消费者 `follow_worker.cpp:748` 手动绕开 | 空窗口返回 Identity（低成本消除一整类风险） |

### 3.4 判据门：与文档论证值偏离，且 config 不拦

| # | 项 | yaml 现值 | 代码/文档论证值 | 现场证据 |
|---|---|---|---|---|
| 11 | `gyro_rot_gate_deg` | 10.0 | `odometry.hpp:116-120` 论证 1.0° | 旧配置 1° 时 `rot_gated` 占 1.38%（270 行）且**与 \|r\| 完全无关**（点二列相关 −0.033，被拦帧 \|r\| 中位数 11.80° 反而低于 ok 帧的 15.23°）；新配置 10° 的后效无现场数据 |
| 12 | `min_inlier_ratio` | 0.10 | `odometry.hpp:92` 论证 0.30 | `out_of_envelope` 1,559 帧；ratio p01=0.120、min=0.00，0.09~0.11 悬停带 27 行 |
| 13 | `gyro_max_freeze_ms` | 30000 | `follow_worker.cpp:318-331` 自己建议 3000~10000 | 旧值 1500 时"静止冻结到达时限"**164 次**强制解冻（相机静置在桌上每 3 s 抖一次） |

`config_loader.cpp:439-447` 不校验 `gyro_still.{window,confirm,bootstrap}`，`gyro_max_freeze_ms<0` 只 warn
⇒ 第 11~13 条这类"把安全门调到不咬"的改动，**没有任何一道门会拦**。
另外值得记一笔：1° 的教训说明这个门的**形式**不对 —— 它比的是 66 ms 单帧窗口里的旋转增量，
而 1°/66 ms 就是 15°/s，属正常手持速度。正确做法是把门限写成 **°/s 的角速度限**（或按积分区间长度归一），
而不是在 1° 和 10° 之间找常数。

### 3.5 数据竞争 **[已核实其中两条]**

| # | 竞争 | 证据 |
|---|---|---|
| 14 | `teach_cap_w_/h_`：**一把字段两把锁** —— `teach()` 在 `state_mutex_` 下写（`follow_worker.cpp:510-511`），`loop()` 在 `snap_mutex_` 下读（`:814,:831`）；`follow_worker.hpp:190` 的注释自认这组归 `state_mutex_` | 两把锁互不排斥 ⇒ 真 UB（换档瞬间可能按错的档位建图） |
| 15 | `has_imu_` 裸 `bool`：`camera_driver.cpp:449,461` 写（持 `pipe_mutex_`），`camera_driver.hpp:69` 无锁读 | 同上 |
| 16 | `R_cam_gyro_` 在重配 pipeline 时写（`camera_driver.cpp:380,394`），在 SDK 陀螺回调 `:441` 无锁读 ⇒ 撕裂矩阵会产出一条错样本，而 `allFinite` 挡不住"有限但错"的值 | |
| 17 | `GyroTimeBase::spread_ns()` 无锁读 `spread_ns_`（`gyro_time_base.hpp:53`），写方持 `mtx_` | 诊断量，影响小 |
| 18 | **空闲分支持锁睡眠**：`std::this_thread::sleep_for(kIdleSleepMs)` 落在 `PublishOnRelease` 作用域内（`follow_worker.cpp:626`、`:645`）⇒ disabled/no_map 时 `snapshot()` 读侧每 30 ms 被堵一次，`broker_.publish()` 也被推迟 | 我把它从"疑似 P0"下调为 P1：不破坏正确性，只造成读延迟与发布节奏异常，且会放大 §5.4 的 SSE 空转 |

### 3.6 端到端反馈（P5 之前必须补的那一类）

| # | 结论 | 证据 |
|---|---|---|
| 19 | **状态退化不产生任何对外流量**：`_ingest` 在 `status != "ok"` 时直接 `return`（不 `_emit`），而 33 Hz 发射线程在"关节角与上次相同"时也 `return`。`lost/out_of_envelope/rot_gated` 恰恰是"保持上一目标 ⇒ 关节角不变"，两个早退条件**同时命中** ⇒ 页面继续显示 `ok·15fps`。这不是排版问题，是与"失败时保持上一目标"（红线 5）共振的静默失效 | `follow_service.py:622`、`:709-711` |
| 20 | `start` 在"示教被拒"时，C++ 侧已 `enabled=true` 而 Python 侧 `_active=False`，且失败路径不 `_emit`、前端 `post()` 失败也不刷新 ⇒ 页面 `enabled` 陈旧、**停止键被禁**（`FollowPanel.tsx:228`）、相机卡在 640x480 档 | `follow_service.py:344`、`:312-347`；现场：示教失败见 §4 的 6 次 `内参 640x480 与帧 1280x800 不同` |
| 21 | **`R_cb` 只在构造时解析一次**（`follow_service.py:107` 是 `_resolve_camera_to_base` 的唯一调用点），且 `sprayer_config.reload()` 全仓只有 `core/config.py:28` 一处 ⇒ 页面上重新做完手眼标定，运行中的后端继续用降级恒等阵，直到重启。与 §12.1 缺口 #2 同一类，但后果重得多（**方向映射错**） | |
| 22 | 停→立刻起会复活僵尸线程：`_stop_poller_if_idle:514-520` 置共享 `_stop_evt` 并清引用，`_ensure_poller:506` 对**新线程** `clear()` **同一个事件对象**，而旧线程此刻正卡在 0.5 s 的 `_cpp` 里 ⇒ 双 poll + 双 emit，`step()` 每拍跑两次（**关节空间限速被双倍执行**）、WS 重复广播 | `follow_service.py:506,514-520` |
| 23 | 平滑器 `_smoother` 被三处触碰：`_begin:357-359`（持 `_lock`）、`stop():298`（**在 `with self._lock` 块外**）、`_emit_tick:702-703`（不持锁）。`step()` 过完 `q_out` 判空后被 `reset()` ⇒ `self.v` 变 None ⇒ `TypeError` 被 `:677` 吞掉丢一帧。与 `trajectory.py:28-29` 自述的"单生产者"契约不符 | |
| 24 | 应用关闭不回收 follow：`main.py:99-101` 只停 camera/robot，不调 `follow_service.stop()` ⇒ 后端重启后相机仍停在 follow 档（可人工恢复，但每次都要点一次停止） | |
| 25 | **IK 失败 440 次 / 18 个分钟段，页面上不可见**。同时遥测显示跟随幅值本身就大：\|t\| p50 **103 mm**、p95 471 mm、max 821 mm；\|r\| p50 15.1°、max 80.7°。`ik_failed` 集中时段的 \|t\| p50 是 151.5 mm（其余时段 84.3 mm）⇒ 不是数值 bug，是**工作空间/基线摆放**问题被当成"保持上一目标"静默吸收了。按红线 5 不夹位是对的，但必须**显式停机并提示**，而不是让臂悄悄停住 | `follow_service.py` 的 `ik_failed` 分支；日志 `本帧未采用（ik_failed）` ×440 |

### 3.7 "同一份实现"红线出现参数分叉 **[已核实]**

`node_main.cpp:151` 调 `teach_reference(..., 10)` 写死 10 帧（`teach.hpp:26` 默认也是 10），
而相机服务用 `cfg_.teach_frames`（`follow_worker.cpp:389`，yaml 现值 **20**，且该键自己的注释还写着"采集 10 帧"）。
示教是**参考几何的唯一来源**，`follow_pose` 与页面服务现在会在同一工件上建出**不同的基准**，
而设计文档 §11.2 立论正是"两边共用同一段代码 ⇒ 控制台读数与页面读数可互为证据"。
代码本身共用对了，参数分叉了。修法：`node_main` 也读 `cfg.teach.frames`，yaml 注释同步。

---

## 4. 功能完整性：闭环 / 半实现 / 未接线

| 环节 | 状态 | 依据 |
|---|---|---|
| 使能 → 切 640x480 硬件 D2C 档 | **可用** | 日志确认 `Delivered Stream: 640x480 @ 15fps align=hw profile=follow` |
| 示教（多帧时间域均值） | **可用**，但有档位竞态 | `teach_core.cpp:60-71` 确实做了逐像素 `accum/valid_count` 时间均值 **[已核实]**；现场 6 次因"内参 640x480 与帧 1280x800 不同"被守卫拒（`follow_worker.cpp:305`）—— 说明 **enable 返回后 pipeline 还在换档**，teach 缺少"等档位真正落地再收帧"这一手 |
| 相对位姿解算（GICP + 稀疏替补 + 判据门） | **可用**，回归全绿 | `follow_replay` 0 未过门 |
| 陀螺三用途（静止冻结 / 离群门 / 帧间初值） | **半可用**：链路已通，但现场证据只有 1,575/19,529 行，且 §3.1 的 5 条风险全在其中 | 带 `gyro=` 的遥测只占 8.1% |
| SSE 推送 + 轮询兜底 | **可用但脆**：`ChunkedEncodingError: Response ended prematurely` 17 次；订阅槽 4 个曾被占满并回 503 两次 | `pose_stream.py`；日志 15:01:58 |
| 镜像数学 + IK + 仿真臂 | **可用**，但 `ik_failed` 静默 | §3.6 #25；`mirror.py:91-94,148` 的共轭+左乘经复核正确 |
| 示教落盘/读回哈希校验 | **可用** | `teach_core.cpp:91-104` |
| 真实臂（P5） | **明确未接线，且护栏正确**：`arm.mode!=sim` 点击即拒（`follow_service.py:314-320`）；`dry_run=false` 在 `node_main.cpp:199-203` 强制回退 | 不是缺陷 |
| `follow.runtime.dry_run` / `enable_servo_p` | **Python 侧零消费方**（grep `app/src` 无命中），仅 C++ 独立工具用 ⇒ 把它们当"发臂总闸"是误读，yaml 注释应写明这一点 | |
| `runtime.health_port 18081` | app 侧**无消费点**（只有 `follow_node` 用）⇒ 页面看不到独立健康快照，`HealthSnapshot` 里那批陀螺诊断字段（`odometry.hpp:160-165`）在生产路径不可达 | 设计文档 §12.1 未记 |
| `Status::kDeviceGone` | **仍然没有产生点**：19,529 行遥测里一次都没出现；拔电表现为 `no_frame`+`connected=false` | 与 §12.1 #10 一致，未修 |
| 解析但未消费的配置键 | `track.sparse.ransac_iters`、`track.gyro_buf_max`、`track.gyro_max_gap_ns`、`frontend.net_input`、`arm.push`/`push_stale_ms`（后两个在 Python 用，C++ 侧只校验） | 死键要在 yaml 里标出来 |
| 前端未渲染的后端字段 | `camera_service_reachable`、`r_cb_ready`、`target_pose`、`arm_target_deg`、`poll_hz/emit_hz` ⇒ **"相机服务没起"与"轴映射已降级"在页面上看不见**，而 `r_cb_source` 只在 running 时挂在一行 title 上 | `FollowPanel.tsx:78-96,370`；违反 §12.5 红线"降级必须可见"的精神 |

### 与设计文档 §12.1 缺口清单的对照（本次复核）

| 旧缺口 | 现状 |
|---|---|
| #1 快照陈旧无门限 | **仍在**，且原方案不可行（见 §3.2 #8，`steady_clock` 跨进程）→ 改为下发 `snapshot_age_ms` |
| #2 臂配置只在启动时解析 | **仍在**，并新增同类且更严重的 #21（`R_cb`） |
| #3 `gyro_used` 不可观测 | 部分已修（`gyro.*` 字段），但 `gyro_pushed` 只在日志里（`follow_worker.cpp:894`），**不进快照/JSON** ⇒ 唯一能区分"IMU 停更"与"时间域错"的字段到不了外部 |
| #4 `stop()` 不清 `_last_error` | 已修（70 条测试绿） |
| #5 `_emit` 去重键不含 `reason` | **仍在**，且实际影响面比原文描述的大（见 #19：状态变化整体不外发） |
| #6~#11 | 状态未变（eye-in-hand 文案、奇异性/速度检查缺失、`forward_all` stub、P5、`device_gone`、前端未用字段） |
| §12.3「σ 门限该重标」 | **有新数据可下结论了**：`follow_replay` 逐轴 σ/sd 普遍 **0.32~0.75**（自报偏乐观但同量级），而现场遥测里 `sT=inf` 与 `ok/sparse` 高度同时出现 ⇒ 真正该重标的是"退化时 `+inf` 与判据门 `trans_sigma_mm=2.0` 之间没有中间态"，不是系数 |

---

## 5. 日志取证（修复前的现场基线，用来证伪优化效果）

### 5.1 覆盖率

- 逐帧遥测 19,529 行（代表帧数 96,333）；帧号归零重启 25 次；`Follow (in-proc)` 10 s 快照 1,202 个。
- ON 总时长 **2.79 h**；ON 且已建图的 fps 分布：**p05 9.47 / p50 14.97 / min 0.02；<15 fps 占 69.6%，<10 占 6.5%**。
- 有效帧率最低的 run：08:58 = **2.14 Hz**、18:32 = 11.97、18:59 = 12.60、07:25 = 10.85。

### 5.2 状态机（行占比 → 覆盖帧占比）

`ok 92.16%（gicp 86.76 / sparse 7.56）`、`out_of_envelope 2.22%`、`lost 2.04%`、`rot_gated 1.38%`、`degenerate 0.03%`。
最长连段：`lost` 342 帧（08:58:59..09:03:02）、`out_of_envelope` 576 帧（07:26:45..07:27:32）、`lost` 449 帧（19:04:03..19:04:48）。

### 5.3 真正吃掉跟随质量的是上游，不是算力

| 现象 | 次数 | 数字 |
|---|---|---|
| `waitForFrameset timeout` | 258 | 其中 181 条带"帧流已停滞"，**p50 2001 ms / p95 3002 / max 8211 ms** |
| `Device is deactivated/disconnected`（stop 路径抛） | 156 | 81 `pipe_->stop` + 75 `gyro_sensor_->stop` |
| `连续 N 次无帧：先软重启 pipeline` | 58 | **成功仅 1 次**（16:11:38），失败 56 次（`This device has no valid sensor!`） |
| 自动硬重连（3 consecutive frame timeouts） | 25 | 02:16、02:19、02:24、07:06..07:43 |
| OrbbecSDK 告警/天 | — | 08/24~08/30 ≈ 430~515，**08/31 = 3,930（8× 恶化）**；分类里 HID `interrupt transfer failed` 144 次（IMU 通道）、`data size does not match` 271、`no matching stream profile` 190 |
| 下游 5 s 发射窗 `keyframes==0` | 113/1,213 = **9.3%（≈565 s 零位姿输出）** | 最长 08:59:07..09:03:03 ≈ **240 s**；且这些窗口里"实际 32.7 Hz"仍报正常、只标"平滑器 钉位" |
| `dropped` / `rejected` 增量 | +123 / +55（0.123% / 0.055%） | ⇒ **真实丢帧不在这两个计数器里**：饥饿发生在驱动取流侧，worker 只是拿不到新帧 |

**结论**：`compute_ms` 的 13.5% 超时只是第二等问题；第一等是"取流停滞 2~8 s + 恢复状态机几乎不工作（1/56）"。
把优化精力全投在 GICP 上是投错了地方。建议新增一个**取流饥饿判据**：`stall_ms > 2× 帧周期` 持续即
ERROR 并把状态降为 `no_frame`（现在只有 `waitForFrameset` 的 warn，且"停滞 0ms"出现过 3 次，说明这条日志本身的量纲也可疑）。

### 5.4 陀螺链路实测健康度（最后一个 run，1,575 行）

- `P`（每帧推入样本）mean **8.37** ⇒ **有效 IMU 率 ≈ 125 Hz，不是配置与所有注释假设的 200 Hz**。
  这不是小事：`gyro_still_window: 20`（注释"@200Hz=100 ms"）实际是 160 ms，`bias_bootstrap: 100` 实际 0.8 s，
  `resid` 的均值窗口长度全都不是注释说的那个时间。**静止门的迟滞带（0.45/0.60，比值仅 1.33）
  在 160 ms 均值窗下的抖动余量需要重估。**
- ~~`S`（参与积分样本）mean 5.12；`S==0` 仅 1 帧（0.06%）⇒ 时间域对齐在这个 run 里是**成立的**。~~
  **这条推论在第二轮被证伪（§11.2）**：`S>0` 只证明窗口框到了**某些**样本，不证明窗口覆盖了**这一帧的整个周期**。
  实测 `S/P = 0.611` 且在静止/运动/快转/慢转各层都是同一个数 ⇒ 积分只覆盖了 61% 的帧间旋转，是恒定系统偏差。
- `resid` p50 13.79 dps、`still=YES` 时 p50 0.05 / max 0.59 ⇒ 静止判据自洽（但当时相机正在被手持晃动）。
- **`陀螺通道嫌疑失效` ERROR 8 次**（19:10..22:35），报错里 `offset=17881470582xx ms` 是 **epoch 量级**
  ⇒ 那批 run 的钟差换算仍是错的（正是 `9dd6bef` 要修的东西）；**修复后的现场只有这一个 run**。

### 5.5 SSE / 契约

`ChunkedEncodingError: Response ended prematurely` 17 次；503 订阅已满 2 次（3 秒内堆到 4 路，
`pose_broker.hpp:33` 的 `kMaxSubscribers=4` 一旦被占满，推送永久退化为轮询）；
`/tmp/svc_sse3.log` 里出现**冻结快照重复 341 行**（`frames=1905 fps=14.8957 compute=51ms` 一字不变）。
`PoseStream.stop()` 的 `_stop` 判据在事件循环体内（`pose_stream.py:197-199`），而心跳注释不产生事件
⇒ 静默流上退出只能赌 `resp.close()` 打断阻塞读，join 上限 `READ_TIMEOUT+1`≈4 s（`:119-124`）；
每次泄漏一路，四次之后推送面死掉。**动作**：把 `_stop` 判据下移到逐行读取处，并把服务端上限提到 ≥8。

---

## 6. 性能：按"可归因 → 可证伪 → 才有收益"排序

### 第 0 步（前置，不做则后面全部无法验证）
把 `compute_ms` 拆开：`frontend_ms` / `cloud_ms` / `gicp_ms` / `sparse_ms` + `iterations` + `gicp_cost`，
进 `FollowSnapshot` 与日志尾行（`follow_worker.cpp:788` 那一处改成 5 个）。
`odometry.hpp:117-119` 已经论证过 `gicp_cost` 比 σ 更实用，但它**根本没上报**（`health_server.hpp:21-53` 里也没有）。
μs 级计时（现在 `follow_worker.cpp:719` 用整毫秒 `steady_now_ms()`，一帧 51ms 的分辨率是 ±0.5ms，
而优化目标就是几毫秒）。

### 抓手清单（预估按 §5.2 实测矩阵与代码位置推得）

| 优先 | 动作 | 证据位置 | 预估 | 风险/守门 |
|---|---|---|---|---|
| 1 | **绑核 + OMP 显式**（A76 池），前端 `cv::setNumThreads` 归位 | `odometry.hpp:58`、`corner_detector.cpp:45`、无 affinity | 尾部 p95 −26 ms；前端 −7 ms | 需留 1 个大核给 SSE/HTTP，别 8 活全占 |
| 2 | **稀疏替补通道加 deadline**（超时即 `hold(lost)` 而不是继续烧 RANSAC） | `odometry.cpp:334`、`:218`；`ok/sparse` p50 84 ms | 消掉 6.97% 的 >80 ms 尖峰 | 要保证"失败保持上一目标"语义不变（红线 5） |
| 3 | **源点云法向在更粗体素上算**（帧间位移 ≪ voxel，源侧不需要和参考地图同规格） | `odometry.cpp:47 preprocess_points(cloud, voxel_m, 10, 4)` | −8~15 ms | 用 `follow_replay` 的 \|Δt\|p95<2mm / \|ΔR\|p95<0.2° 守门 |
| 4 | **`max_iters` 25 → 8~12**（先上报 `iterations` 再定） | `odometry.cpp:71`；yaml:127 | −3~10 ms（退化平面场景最明显：`wall` 用例 gicp p50 54.5 ms） | 同上 |
| 5 | **前端抽取条件化 / 降频**（`FeatureFrame` 只被 `odometry.cpp:218` 的稀疏通道消费） | `follow_worker.cpp:725`；`odometry.cpp:276,334` | −15~20 ms | **最大单项但最危险**：稀疏解是 GICP 的 1 档初值。只在 `\|Δ\|>死区` 或 `sparse_streak>0` 时抽，并在 `rot`/`scan` 用例上回归 |
| 6 | 每帧 3 次 `getIntrinsics()`/`getStatus()` **值返回**（深拷贝 `vector<vector<double>>`+4 个 string+一次 mutex） | `follow_worker.cpp:77,270,757`；`camera_driver.cpp:825-833` | −1~3 ms | 缓存 + dirty 标志 |
| 7 | 每帧堆分配：`std::vector<GyroSample>`（`:718`）、`factors`（small_gicp `registration.hpp:41`）、`prev_depth_.clone()`（`odometry.cpp:156`）、`cloud.cpp:53 reserve(total/2)` | | −2~4 ms + 抖动 | 双缓冲/常驻复用 |
| 8 | SSE 轻指纹：现在每次唤醒都 `snapshot()` + 全量 `followSnapshotJson()` 再判 changed（`http_server.cpp:664`），空闲时仍以 ≈33 Hz 发布 ⇒ 4 路订阅每秒约 200 次"构造后丢弃" | | 省 CPU（给帧预算让路） | 不能破坏心跳语义（`:116-117` 论证过为什么必须重写） |

---

## 7. 测试与门禁有效性

| 现状 | 问题 |
|---|---|
| `follow/build ctest` 4 套件 | **绿（第二轮重建后 4/4，见 §11.1）**。但注意两条：① 必须先 `cmake --build` 否则结论无效；② 它只有 4 个套件、4.8 s，覆盖面见下面几行 |
| 相机服务的 2 条单测（`test/`，16 条） | 不在同一条 `ctest` 入口里 ⇒ 跑设计文档 §11.1 那套"提交前必跑"**根本不会覆盖到它们**，而它们恰好是唯一钉住时间域接缝的测试 |
| `FollowWorker` 零自动化（已核实 `test/` 里无引用） | 本报告 §3.1~§3.5 的绝大多数问题都在这一层，且全是有界、可用假 driver 测的 |
| Python 70 条全绿，但**真实契约零覆盖** | `_cpp` 被逐用例替换成 lambda（`test_follow_api.py:123,136,165`），快照由自造 `_snap()` 提供（`test_follow_mirror.py:353-357`），`_ensure_poller/_ensure_stream` 被 lambda 短路 ⇒ C++ 侧任何字段改名（`delta_t_m/has_pose/frames/switching`）都会"全套仍绿而运行时每帧判 unusable" |
| `python -m unittest discover -s apps/follow` 报 not importable | `app/src/apps/follow/` 缺 `__init__.py`（`apps/camera`、`apps/interactive` 都有）⇒ CI 若用 discover 会**静默收集 0 条** |
| 前端 0 测试 | 与 §12.1 一致 |

**建议补的 6 类（都能在无机无臂条件下跑）**：
① `FollowWorker` 用 fake `CameraDriver` 钉住"定标切换帧不丢 stale"、"hold 帧不进平滑窗"、
"`kDegenerate` 的 `holding_last_pose=false`"；
② 契约测试：读 `followSnapshotJson` 的字段清单 ↔ Python 侧读取键清单做集合差，改名即红；
③ 线程生命周期：stop→start 竞态（#22）与 `_smoother` 并发（#23）；
④ 时间基 epoch 变化时的 Tracker 重建（#2）；
⑤ `ctest` 合并入口把相机服务那 16 条一起收进来；
⑥ 一条 ≥5 min 静止的 `follow_pose` 长跑（把 §12.3 悬着的 σ 结论做掉）。

---

## 8. 建议执行顺序（每批都能独立提交、独立证伪）

**批 1 · 半天，恢复保护网 + 让耗时可见**
1.1 ~~修红门~~ 已随 `9dd6bef` 绿（§11.1）。**换成：陀螺时间尺度三数自述 + `|1-S/P|>0.15` 报警（§11.2）**。
1.2 阶段耗时 + `iterations`/`gicp_cost`/`gyro_pushed` 进快照与日志（§6 第 0 步）。
1.3 绑核 + OMP + `cv::setNumThreads` 归位（§2 P0-3）。
1.4 `app/src/apps/follow/__init__.py` 补上；`ctest` 收编相机服务的 16 条。
验证：`cmake --build . -j4 && ctest` 全绿；同一场景跑 3 min，p95 从 **69 ms**（§11.5 新基线，不是旧窗的 87）降到 ≤61 ms，
且 **`S/P` 从 0.611 回到 1.0±0.15**（这条是批 1 真正的主验收项）；`follow_replay` 仍全过。

**批 2 · 1~2 天，时间域与陀螺链路收口（全部 [待上机]）**
2.1 定标翻转/时间基换 epoch 时重建 Tracker（#1、#2）。
2.2 陀螺队列双订阅，示教不再抢跟踪（#4）。
2.3 `stale`/`gap_end_ns` 上报 + 分位数定标（#3）。
2.4 门限回设计值并把 `gyro_rot_gate_deg` 改写成 **°/s** 形式；`config_loader` 对越界值判 fatal（#11~#13）。
2.5 **按实测 125 Hz 重标 `gyro_still_window/confirm/bias_bootstrap` 与迟滞带**（§5.4）。
2.6 陀螺零偏外置，换档不销毁（#5）。
验证：上机 5 min，判据是 `S==0` 帧数 = 0、`rot_gated` 占比 < 0.1%、"陀螺通道嫌疑失效" 0 次、
示教期间跟踪侧 `samples_used>0`。

**批 3 · 1 天，端到端不再静默**
3.1 状态指纹（`frames/status/enabled/reason/ik_failed`）独立触发 `_emit`（#19）。
3.2 `snapshot_age_ms` 派生量 + Python/UI 停滞判据（#8、§12.1#1）。
3.3 `ik_failed` 连续计数进 UI；超阈值显式提示"目标已出工作空间"（#25）。
3.4 `R_cb` 在 `_begin` 重解析（#21）；teach 前等档位真正落地（§4 竞态）。
3.5 线程 generation token + `stop()` 在锁内 + 应用关闭回收（#22~#24）。
3.6 `holding_last_pose`/`has_pose` 按定义修（#6、#7），并同步 `FollowPanel`。
验证：遮住相机 ⇒ 页面 1 s 内出现 `lost`；`/follow/status` 的 `age_ms` 在 worker 卡死时单调增长。

**批 4 · 性能专项（在批 1.2 的数据之上做）**
4.1 源点云粗体素法向；4.2 `max_iters` 依 `iterations` 实测定值；4.3 前端条件化抽取；
4.4 分配复用（GyroSample/factors/prev_depth）；4.5 SSE 轻指纹。
验证：每项单独一个 commit，`follow_replay` 精度门不破 + 现场 p50 ≤ 40 ms、p99 ≤ 55 ms（即留 1/3 余量）。

---

## 9. 本次被推翻或下调的结论（防止后面按错的说法定罪）

1. **板子是 RK3588，CMake 注释没错**：`/proc/device-tree/model` = Orange Pi 5 Plus；
   "目标芯片标注错误"的说法不成立（且本板确为 4×A76 + 4×A55，绑核问题依然成立）。
2. **"多帧示教实为空壳"不成立**：`teach_core.cpp:60-71` 的逐像素时间均值是真实有效的，
   配置项 `teach.frames` 也确实被 `follow_worker.cpp:389` 消费。真正"不可达"的是
   `ReferenceMap` 的 64-scan 多位姿融合接口（`reference_map.cpp:120-140`）——
   `teach_core.cpp:84` 把整次示教塌缩成一个 `TeachFrame{pts, Identity}` 喂进去。
   这是**未使用的复杂度**（P2，可删或改注释），不是功能缺失。
3. **"CPU-SIFT 丢边界点会整帧作废"下调为观察项**：`frontend.cpp:70-74` 已经把边界角点手动剔除后才
   `compute()`，因此 `:82` 的 `raw.rows != kpts.size()` 确实只在"实现违反契约"时命中。
4. **`out_of_envelope` / `rot_gated` 并非"几乎不触发"**：修复前现场里分别有 1,559 帧与 572 帧
   （旧配置 `gyro_rot_gate_deg=1.0`）。真正的问题是**门限形式不对 + 无校验**（§3.4）。
5. **`/api/api/v1/...` 双前缀不是生产 bug**：来自测试自造的 `base_url`，
   `CPP_BASE_URL` 实际不带 `/api`（`apps/camera/services/camera_service.py:35`）。
6. **空闲支路的持锁睡眠从 P0 下调为 P1**：影响是读侧延迟与发布节奏，不破坏正确性。

---

## 10. 附录：复现命令

```bash
# 门禁。**必须先重建**：follow/build 里的可执行文件比源码旧 6 小时就会给出假结论（§11.1 的教训）。
cd follow/build && cmake --build . -j4 && ctest --output-on-failure
./test_config --gtest_filter=Config.RepoConfigIsValid

# 精度回归（必须从仓库根跑，数据集在 ./out/synth）
follow/build/follow_replay --root out/synth

# 大小核矩阵（本报告 §2 P0-3 的数据）
for spec in 4-7 0-3 4,5 0; do taskset -c $spec follow/build/follow_bench_frontend follow/out/real 10; done

# 后端测试（含真实 SSE 接缝）
cd app/src && ../.venv/bin/python -m unittest \
    apps.follow.services.test_follow_mirror apps.follow.services.test_follow_api \
    apps.follow.services.test_pose_stream

# 相机服务的两条接缝（当前不在 ctest 入口里）
app/src/core/hardware/camera/orbbec_camera_service/build/test_gyro_time_base
app/src/core/hardware/camera/orbbec_camera_service/build/test_pose_broker

# 拓扑与调度事实（绑核结论的前提）
tr -d '\000' < /proc/device-tree/model; cat /sys/devices/system/cpu/cpu*/regs/identification/midr_el1

# 切出"最后一次运行"（§11 全部数字的口径）：以最后一次 CameraService initialized 为界
python3 -c "
ls=open('app/logs/backend.console.log',encoding='utf-8',errors='replace').read().splitlines()
cut=max(i for i,l in enumerate(ls) if 'CameraService initialized' in l)
print('新构建窗口起始行',cut+1,ls[cut][:20])"
# 判断某个 run 用的是哪个构建：比对二进制 mtime 与该 run 的启动时刻
ls -la --time-style=full-iso app/src/core/hardware/camera/orbbec_camera_service/bin/orbbec_camera_service
```

日志取证口径（§5 全部数字可复查）：
`app/logs/backend.log.2026-08-31`（18,925 条逐帧）+ `app/logs/backend.log`（604 条，9/1 00:00–00:06）
+ `app/logs/backend.console.log`（18,301 条逐帧，覆盖 07:23–次日 00:06：其 8/31 部分是上面第一个文件的子集，
其 9/1 部分是第二个文件的超集）
+ `/tmp/svc_sse{,2,3}.log`（服务侧 HTTP/档位/SSE 视角）
+ `Log/OrbbecSDK.log.txt`（67,747 行 SDK 层，按日告警分布见 §5.3）。

**§11 的口径与 §5 不同**：§11 只用 `backend.console.log` 里**最后一次运行**那一段
（行 31608 起，23:45:02 启动 → 00:06:19 关闭，1,575 条逐帧遥测），因为那才是 23:43:55 构建的二进制跑出来的数据。

---

## 11. 第二轮复查（9/1）：最新日志 + 重建复测

口径：`backend.console.log` **最后一次运行**（行 31608 起，23:45:02 启动 → 00:06:19 关闭）。
判据链：二进制 `bin/orbbec_camera_service` mtime = **23:43:55**；`9dd6bef` 改动的全部文件 mtime
≤ **23:37:03** ⇒ **这一段用的是修复后的构建**，是陀螺链路目前唯一的现场数据。
样本量：1,575 条逐帧遥测 / 20 分钟 / 帧号 12,380→18,043（Δ=5,663 帧 ÷ 377 s = **15.02 fps**，与 `Capture FPS: 14.985` 吻合）。

### 11.1 门禁复测：P0-1 撤回（并已就地改掉）

`cmake --build . -j4 && ctest` → **4/4 全绿**（small_gicp_api / core / registration / config）。
`9dd6bef` 自己就把 `test_config.cpp:365-369` 的期望改成了 `0.45/0.60/30000`，即第一轮那条"红门"
在提交时已经是绿的；我读到的失败来自 19:34 的旧可执行文件。

**可复用的教训（这条比原结论更有价值）**：这个仓库的构建目录和源码可以相差 6 小时以上而无人提示，
所以"跑测试"必须先重建。§10 的复现命令已按此改写。

### 11.2 【新 P0】陀螺积分只覆盖 61% 的帧间旋转，且是**恒定**系统偏差 **[已核实，现场]**

`gyro=P/S` 两个字段的定义决定了它们相减就是一个免费的对齐诊断：

| 字段 | 来源 | 计的是什么 |
|---|---|---|
| `P = r.gyro_pushed` | `odometry.cpp:118` `push_gyro()` 里 `++gyro_this_frame_`；`track()` 一回来就清零（`:149`） | **主机侧**本帧处理周期内**交付到达**的样本数 ⇒ 与时间戳无关 |
| `S = r.gyro_samples` | `integrate_gyro(gyro_, prev_ts_ns_, ts_ns, …)`（`types.hpp:108`，窗口 `t0<ts≤t1`） | **设备域**帧间隔 `[prev_ts, curr_ts]` 内框到的样本数 |

同一个缓冲、同一段真实时间 ⇒ **若时间轴自洽，S ≈ P**。实测（1,575 行）：

```
P 均值 8.37（min 4 / max 13）   S 均值 5.12   ⇒  S/P = 0.611
S<P 1548 行 (98.3%)   S==P 15 行   S>P 12 行   S max 16
联合分布 top: (8,5)×755  (9,5)×388  (8,4)×182  (9,6)×111
```

**且与运动状态无关**（这一条排除了"抖动/排队"解释）：

| 分层 | n | S/P |
|---|---|---|
| `still=YES` | 228 | 0.602 |
| `still=NO` | 1347 | 0.613 |
| `resid > 20 dps` | 584 | 0.619 |
| `resid < 1 dps` | 274 | 0.606 |

⇒ 积分窗口在设备域里只有 **≈40.7 ms** 宽，而真实帧周期是 66.6 ms —— **少了 26 ms / 39%**。
两种机制都能产生这个数，且都不能从现有输出里区分（这正是"静默失效"的形状）：

- **A｜陀螺戳比真实时间慢 1.63×**：样本按标称 5 ms（200 Hz）打戳，实际每 7.96 ms 才到一条
  ⇒ `integrate_gyro` 里 `dt = (s.ts - ta)·1e-9` 系统性偏小 ⇒ **积出的角度只有真值的 ~61%**，
  旋转初值偏短、且离群门里陀螺侧恒小于视觉侧。
- **B｜帧设备戳不是"这一帧的真实曝光时刻"**，而是某个更快子流（例如 25 Hz 硬同步 tick）的计数
  ⇒ 窗口只有 40.7 ms 宽。单条 `dt` 是对的，但**每帧有一段固定的 26 ms 旋转盲区**。

（第三种解释"设备晶振比主机慢 1.63×"**可以直接排除**：ppm 级晶振不可能差 63%，
所以 0.611 一定来自"戳的语义"而不是"钟的速率"——A 与 B 都是语义问题。）

> **✅ 定论（9/1 真机复测，实施细节见 §12）：A、B 都不成立，是机制 C。**
> 插桩实测 `frame_dev_ratio = 1.0005`、`gyro_stamp_ratio = 0.9992` ⇒ **两路戳都忠实于真实时间**，
> 不是语义错；而 `span_ms = 33.1 / gap_end_ms = 33.6`，两者相加正好等于一个帧周期 66.7 ms
> ⇒ 窗口的**尾部固定缺半帧**：IMU 样本的端到端交付比图像晚约 25~34 ms，解算这一帧时尾部那
> 几条还没到货（`buf=124` 也证明它们随后就到了，不是丢样本）。
> **已修**：`integrate_gyro` 按最后一条到货样本的角速度把缺口常值补积掉（≤ `gyro_max_gap_ns`
> 才补，真停更不补），修复后 `span_ms = 66.7 / gap_end_ms = 0 / extrap_ms = 25`。
> **推论也一并修正**：`S/P ≈ 0.58` 是**结构值**不是故障，§11.2 动作项 2 里那条
> `|1 - S/P| > 0.15` 的报警会永久误响，已改成"戳比率偏离 1 超 5%"或"覆盖率跌破 0.30"才喊。

**为什么它现在完全没有症状**：静止检测器只看向量模、不碰时间轴（`odometry.cpp:123` 注释即此意），
所以 `still`/`resid` 一切正常；`inl` 0.81、`sT` 0.13、`sR` 0.010 也都不依赖陀螺尺度；
`rot_gated` 本 run = **0** —— 而 0 **不是好消息**：门限 10°，而 15 fps 下 p50 残差 13.79 dps 对应的
帧间旋转只有 ~0.92°，其 39% 是 0.36°，**这个门在当前尺度下对真实运动永远不会触发**（要 >16°/帧才拦得住）。

**动作**：
1. **三数自述**（约 10 行，一次定标周期即可判）：每帧报 `frame_dev_delta_ms`、`gyro_stamp_delta_mean_ms`、
   `host_recv_delta_ms` 到 `/follow/status`。A ⇒ 第二个 ≈5.0 而第三个 ≈7.96；B ⇒ 第一个 ≈40.7 而第三个 ≈66.6。
2. **把它变成会报警的不变量**：仿照 `checkGyroChannel`（`follow_worker.cpp:363`）加一条
   `|1 - S/P| > 0.15` 持续 N 帧 ⇒ ERROR。**现在这条完全靠人读日志算，等于没有。**
3. 判据修好之前，**`gyro_rot_gate_deg` 应当显式承认它当前是惰性的**（文档 + `describe()` 输出里写明），
   别让人以为"0 拦截 = 有互验保护"。

### 11.3 【新 P1】定标自述暴露 `GyroTimeBase` 头注释里的精度论证不成立 **[已核实，现场]**

```
23:45:09  陀螺时间基已定标: offset=1788147058385ms, 配对=8帧, 延迟抖动=354ms, 定标前丢弃样本=0
23:46:04  陀螺时间基已定标: offset=1788147058252ms, 配对=8帧, 延迟抖动=460ms, 定标前丢弃样本=52
```

- **`延迟抖动` 就是 `spread_ns_ = worst - best`（`gyro_time_base.cpp:28`），实测 354 / 460 ms。**
  而 `gyro_time_base.hpp:21-23` 的论证是"主机与设备钟差漂移几十 ppm ⇒ 分钟尺度 1~2 ms ⇒
  对应门误差 0.01°，比门限低两个数量级，**精度够**"。**现场这个候选离散度是 354 ms，比该论证大 ~200×**
  ⇒ 注释里那条"够不够"只算了晶振漂移，没算 USB 排队延迟的离散度。
- **但要说清危害走哪条路，别把 §11.2 的 0.611 记在它头上。** 陀螺戳与帧戳**同源同钟**、且乘**同一个**冻结常数：
  积分窗口宽度 = 两帧设备戳之差 ⇒ 常数在减法里**完全抵消**，所以**恒定偏移量本身对积分是无害的**。
  这个 354 ms 真正伤到的是另外两条：
  1. **重定标时的台阶**（下一条，实测 133 ms）：台阶不会抵消，因为它换的是"减法里的那个常数"。
  2. **`using_device_ts == false` 的回落分支**（`follow_worker.cpp:705` 用 `fd.timestamp_ms`，那是**主机到达时刻**，
     比设备域晚 350~460 ms）⇒ 一旦定标未完成或未生效，帧窗口会整体平移半秒，而陀螺样本被 `toHostNs()==0` 丢掉，
     两者**不在同一个域**——这正是头注释自己警告的"混进两个域的样本会让缓冲不再单调"。
- **两次定标相差 133 ms**（…385 vs …252，中间只隔 55 s，设备没重启）。头注释明写
  "标定只做一次并冻结：后续任何重新估计都会在时间轴上打一个台阶，而台阶比常量误差毒得多"。
  **但 `stop`/`start`/换档路径调 `gyro_time_base_.reset()`（`camera_driver.cpp:71`）⇒ 每次切 follow 档必然重定标 ⇒ 必然打一个台阶。**
  本 run 里恰好两次定标都发生在 `follow` 使能（23:46:10）之前，所以没造成 `stale_input`；
  **P5 之后一旦在跟踪中途切档/重连，就会吃到 133 ms 级别的一阶台阶**（13.79 dps × 0.133 s ≈ 1.8°）。
- `定标前丢弃样本=52` ⇒ 每次重定标会**空掉 ~0.4 s 的陀螺**（窗口内无样本 ⇒ 初值降级 + 门不参与）。
- 顺带：第一轮 §3.1 里"时间域切换造成确定性丢帧"降级 —— 本 run `dropped=0`，因为定标都在使能前完成；
  路径仍在（`follow_worker.cpp:700` 重置的是 worker 的 `last_ts_ns_`，而 `odometry.cpp:202` 查的是 Tracker 的 `prev_ts_ns_`，
  中途重定标时那一帧仍会变 `kStaleInput`）。**属于 [待上机]：需要一次"跟踪中途关流再开流"的操作才能复现。**

**建议（能把上面三条一起消掉）**：陀螺与帧戳**同源同钟**（都是 `getTimeStampUs()` 自设备上电 µs），
所以积分与窗口**根本不需要主机时间**。改成：`FrameData` 保留 `device_ts_us`，`Tracker` 全程在设备域比较与积分，
`GyroTimeBase` 只保留给"和主机日志/快照时间对齐"这一个用途（甚至可删）。
这样就没有钟差配对、没有 354 ms 抖动、没有重定标台阶、没有 reset 时机问题，`stale` 判据也更准。

### 11.4 【新 P1】静止冻结每 60 s 强制解冻一次：**30000 ms 不是解法** **[已核实，现场]**

```
23:46:43  静止冻结到达时限 30000 ms：强制解冻并暂停 30000 ms… 本次冻住旋转 450 帧
23:47:43  …（同上）451 帧   距上次 60s      23:48:43 … 450 帧  距上次 60s
23:49:49 … 449 帧 66s   23:50:48 … 450 帧 59s   23:52:01 … 448 帧 73s   23:53:02 … 61s   23:54:01 … 59s
```

10 分钟里 **18 次**，间隔稳定 59~73 s = 30 s 冻结 + 30 s 暂停 ⇒ **"暂停 30000 ms"确实被遵守了，机制没 bug。**
问题是后果：相机真静止时，系统在"冻结 30 s / 解冻 30 s"之间永久循环，**解冻的那 30 s 里 GICP 的
旋转高频噪声（`sR` 0.010°，但 450 帧的随机游走远大于它）会重新进臂**，于是臂每 30 s 微动一下。
`follow_log_telemetry_guide.md` §4 给的建议（"生产环境把 `gyro_max_freeze_ms` 设大或 0"）**不足**：
设大只是把周期拉长，设 0 等于关掉整个静止保护。

**动作**：时限的作用是把"恒定慢速转动"与"真静止"分开，而这两路**本来就能分开**——
`still=YES` **且** 视觉帧间旋转 `|ΔR| < ε`（GICP 自己的输出，与时间轴无关）⇒ 才冻结且**不设时限**；
只满足陀螺那一路 ⇒ 保留时限。这样静止 20 分钟也不会被强制解冻，也不用在日志里刷 18 条 218 字符的 WARN。

### 11.5 修复效果量化（同一统计口径，旧窗 16,726 行 vs 新窗 1,575 行）

| 解算器 | 旧窗占比 | 旧窗 p50 / p95 / max / 超支 | 新窗占比 | 新窗 p50 / p95 / max / 超支 |
|---|---|---|---|---|
| `gicp` | 84.96% | 49 / 65 / 138 / **4.3%** | 96.63% | 51 / 64 / 109 / **3.1%** |
| `sparse` | 8.58% | 85 / 103 / 220 / **86.4%** | 2.48% | 79 / 87 / 89 / **100%** |
| `none` | 6.46% | 66 / 103 / 145 / **49.9%** | 0.89% | 73 / 89 / 89 / **78.6%** |
| 合计 | — | 51 / 88 / 220 / **14.27%** | — | 51 / 69 / 109 / **6.16%** |

**这张表最重要的读法不是"变好了 2.3 倍"，而是"为什么变好"**：三条路径各自的 p50/p95 **几乎没动**
（gicp 49→51、65→64）。总超支率从 14.27% 掉到 6.16%，**全部来自路径占比迁移**
（sparse 8.58%→2.48%、none 6.46%→0.89%）。也就是说 `9dd6bef` 的收益是
**"陀螺初值变准 ⇒ GICP 一次就收敛 ⇒ 不必再走 CPU-SIFT"**，不是任何一处算得更快了。

两个直接推论，都推翻/改向第一轮 §6 的抓手排序：

1. **最贵的不是 GICP，是稀疏回退路径**（单路径 100% 超预算，且它一出现就把整帧周期作废）。
   第一轮把"OpenCV 前端线程数 / 绑核"排在前面仍然成立，但**收益测算要按"sparse 出现率"重排**：
   把 sparse 从 2.5% 压到 0 大约能再拿回 2.5% 的超支率（→ 3.6%），而 GICP 侧优化只能碰那 3.1%。
2. **口径警告**：新窗只有 20 分钟、一次连续跟手测试、无重启；旧窗含 25 次重启、多种档位与 08:58 那种
   2.14 Hz 的极端 run。所以 6.16% 是**当前最好条件下的数**，不能当因果结论，也不能外推到全天。
   §5.3 那条"第一等是上游停滞"的判断**没有被新窗推翻**（新窗停滞 0 次只说明这 20 分钟设备健康）。

**统计口径警告（两轮同病，读所有超支百分比前先看这条）**：`compute_ms` 是**按打印行**统计的，
而静止段会被"静默合并"压成一行（本 run 252/1,575 行是合并行，最长一条代表 77 帧）。
合并行落在哪一帧就只带那一帧的耗时 ⇒ **6.16% / 14.27% 都是"行级"估计，不是帧级精确值**；
且运动帧几乎不合并、超支也更集中在运动帧（新窗 `still=NO` 6.6% vs `still=YES` 3.5%），
所以这个数偏向高估"整段跟手期间的超支率"、低估"静止段的帧数权重"。
**帧级精确值必须等 §6 第 0 步的分阶段计数落进 `/follow/status` 之后才能拿到。**

同口径下的其它改善（同样只 20 分钟样本）：

| 现象 | 旧窗 | 新窗 |
|---|---|---|
| `帧流已停滞` | 175 | **0** |
| `ik_failed`（Python 侧 WARN） | 363 | **7** |
| `rot_gated` | 270 | **0**（但见 §11.2：门在当前尺度下本就不会触发） |
| `out_of_envelope` / `lost` / `degenerate` | 425 / 385 / 5 | 1 / 13 / 1 |
| `dropped` `rejected`（每个 10 s 快照） | +123 / +55 | **0 / 0** |
| 带 `gyro=` 的遥测行 | 0% | **100%** |
| `follow 发射` 实测 | — | 32.7~33.0 Hz，`keyframes 74~75/窗` ⇒ 33 Hz 发射面正常 |

### 11.6 第二轮复核后**仍然成立**的第一轮结论（没有被这次日志推翻）

| 条目 | 为什么仍然成立 |
|---|---|
| P0-2 无法归因 | 新 run 的 `[Status]` 阶段耗时只有 `RGA BGR->NV12 / MPP H.264 Enc / ZLM StreamPush / Total Pipeline`（都是编码推流侧），follow 只有聚合的 `compute=55ms`。**一条 follow 分阶段数都没有。** |
| P0-3 不绑核 | 与新日志无关（本 run 也没绑核配置）。且现在更值得做：单路径成本没动 ⇒ 大小核漂移会原样打进 6.16%。 |
| §3.6 端到端静默（第一轮列在 P0「最该先做的三件事」③） | `follow_service.py` mtime 23:26:56 = 我第一轮读的就是这份；第二轮按行复核 `:606-623`、`:709-711` **原文未变** ⇒ 双重早退仍在。新 run 里 `ik_failed` 7 次仍只有 WARN。 |
| 3.2 快照字段说谎 / 3.5 数据竞争 / 3.7 红线参数分叉 | 均为静态代码事实；本轮复核 `node_main.cpp:151` 仍 `teach_reference(..., 10)`，而服务侧日志确认走的是 **20 帧**（见 §11.7），分叉仍在。 |
| 5.5 SSE 契约（`kMaxSubscribers=4`、`stop()` 泄漏） | 新 run 里 `connection open` 18 次、无 503，**但那是"这 20 分钟没触发"而非"已修"**：`pose_stream.py` mtime 停在 14:27，未随 `9dd6bef` 改动。 |

### 11.7 新出现的两条（第一轮无数据）

1. **示教均值 20 帧已在服务路径生效**：`[teach] 冻结参考地图(均值 20 帧) …/reference.frmap 原始点=30348 体素=6933 voxel=0.02 hash=dc443e`。
   ⇒ 第一轮 §9-2 的判断（"多帧示教不是空壳"）被这条现场确认；但 `node_main.cpp:151` 的硬编码 **10** 仍在，
   所以"同一个示教动作在 `follow_node` 与相机服务里产出两张不同的图"这条**继续成立**（3.7）。
2. **`[ZLM] Stamp expired is abnormal: -758547` ×7**（23:54:46 前后，均在手持快速移动段）。
   负值 = 推流时间戳超前于系统时钟 0.76 s。发生在编码/推流侧（`RGA`/`MPP`/`ZLM` 三段耗时同时段也有尖峰：
   `RGA max 10.97 ms`、`MPP max 11.15 ms`），**不影响 follow 数值链路**，但会影响页面预览的流畅度与
   任何拿视频时间戳做事的人。列为观察项：若之后要在预览上叠画位姿，这条会先变成 bug。

### 11.8 对 §8 路线图的修订

```
批 1（半天）  ~~修红门~~ → 换成：
   1.1' 三数自述 + `|1-S/P|>0.15` 报警（§11.2）—— 比原 1.1 价值高得多，且是后面所有陀螺工作的前提
   1.2' 阶段耗时（frontend/cloud/gicp/sparse + iterations）与绑核（P0-2 / P0-3 不变）
   1.3' 验收命令改成"先 build 再 ctest"（§11.1 的教训）
批 2（1~2 天）时间域：按 §11.3 的建议**整链改设备域**（而不是继续修补 GyroTimeBase），
   并把"重定标 = 打台阶"从注释警告变成代码约束（中途 reset 需显式打日志 + 冻结判据 1 帧）
批 3（1 天）  端到端不静默：状态指纹广播、`snapshot_age_ms`、`ik_failed` 上页面、R_cb 重解析、线程代际令牌
   （§3.6 全部不变；新日志只是把次数从 363 降到 7，没有改变"降不到 0 且不可见"这件事）
批 4（1~2 天）性能：优先级按 §11.5 重排 —— 先攻 sparse 出现率与单路径成本，再谈 GICP 微调
新增（可选，1 小时）静止冻结改双判据（§11.4），消掉每 60 s 一次的强制解冻与 18 条/10 min 的 WARN 噪声
```

### 11.9 本轮（第二轮）被推翻的结论清单

1. **P0-1「红门」整条撤回** —— 旧构建造成的假警报；`9dd6bef` 提交时已是绿的。保留其中"结构体默认与 yaml 不一致"
   作 P2（且它现在已被测试注释显式记录）。
2. **§5.4「S>0 ⇒ 时间域对齐成立」撤回** —— 正确判据是 `S/P≈1`，实测 0.611。已在原处标注并指向 §11.2。
3. **§3.1「时间域切换造成确定性丢帧」降级为潜在路径** —— 本 run `dropped=0`（定标都在使能前完成），
   路径代码仍在，需要"中途关开流"才能复现。
4. **"13.5% 超支" 这个总数不能再直接用于新构建** —— 新构建 6.16%，且改善来自路径占比而非单路径成本（§11.5）。
5. **`rot_gated=0` 不等于"离群门工作正常"** —— 在 `S/P=0.611` 与 10° 门限下，这个门对真实运动本来就不会触发。
6. **`follow_log_telemetry_guide.md` §4 的建议要改** —— "把 `gyro_max_freeze_ms` 设大或 0" 不足以解决
   每 60 s 强制解冻的循环（§11.4）。

## 12. 真机插桩定论与已落地修复（9/1，本地未提交）

设备：Gemini 336L SN CPCV553001B1 / FW 1.4.60，follow 档 640x480@hw，`--raw-log`。

### 12.1 定位链（每一步都是可复现的数）

1. 先加**三数自述**（§11.2 动作项 1）：`frame_dev_ratio`（帧设备戳推进 / 主机到达推进）、
   `gyro_stamp_ratio`（burst 末样本戳推进 / 主机到达推进）、`coverage`（滚动 S/P）。
   分母只认主机到达间隔一条，所以三条时间轴里必然两条≈1，错的那条自己站出来。
2. 真机读数：`frame_dev_ratio = 1.0005`、`gyro_stamp_ratio = 0.9992`、`coverage = 0.577`。
   ⇒ **A、B 两个"戳语义错"的猜测同时被否**（原报告里我写"必然有一个≈0.61"是错的推理：
   窗口宽度与样本戳比率都不是那个比值该等于 1 的量）。
3. 再加覆盖账 `span_ms` / `gap_end_ms`：真机 `33.1 / 33.6`，相加 = 66.7 ms = 整一个帧周期。
   ⇒ 坐实**机制 C：窗口尾部固定没到货**（不是被跳过、不是丢样本 —— `buf=124` 说明它们随后就到）。

### 12.2 修复（`integrate_gyro` 末段常值补积）

`follow/include/follow/types.hpp`：积分后用**最后一条到货样本**（窗口内一条都没有时退回 `t0`
前最后一条）的角速度把 `t1 - tail.ts` 这段补积掉；`miss > max_gap_ns` 时不补（那是 IMU 停更，
拿半秒前的速率外推等于凭空造旋转）。新增 `GyroDelta::extrap_ns` 让"补了多少"可读出。

为什么补积而不是别的三条路：
- **等 33 ms 再解算** —— 直接给整条跟随链路加半帧延迟，控制上不可接受；
- **只把区间缩到 `t1 - gap`** —— 门的两侧区间不再相同（正是 §P3 注释里点名的"最容不下错的一点"），
  且初值仍然少转半帧，等于没修；
- **只重标门限单位** —— 承认门惰性，但初值少转半帧这个真实误差还在（快速平移时 GICP 会不收敛）。

危害的真实大小：`R_init` 与离群门的陀螺侧**恒少算半帧旋转**。20 dps 下少 0.66°，而门限
`gyro_rot_gate_deg = 1.0` ⇒ 平移越快越容易把好帧拦下来，而拦下来又不更新参照 ⇒ 正反馈锁死。
本 run `resid` 只有 0.05 dps（工位静止），所以一直没症状 —— 这是"没暴露"，不是"不存在"。

顺带修掉的两个隐藏坑（同一个函数里）：
- `stale` 的第三条判据 `span + max_gap < t1 - t0` 与第二条**恒等**（`first_used` 数学上必等于 `t0`，
  所以 `span + gap_end == t1 - t0`）—— 写了等于没写，删掉并把语义收敛成两条。
- `valid()` 原来是 `samples_used > 0 && !stale`。补积之后"窗口内 0 条"变成高帧率档的正常工况，
  所以改成 `!stale`；**同时** `odometry.cpp` 里首帧那个默认构造的 `GyroDelta` 必须显式 `stale = true`，
  否则首帧会把单位阵当成"陀螺测到的旋转"采信（这个坑是新语义引入的，已一并堵上）。
- `checkGyroChannel` 的失效指纹收紧成 `samples == 0 && extrap == 0 && buf > 0`：只看 `samples == 0`
  会在 30 fps 档天天误报，而两域错位时 `extrap` 必然也是 0（末样本比 `t1` 老几个数量级）。

### 12.3 修复后复测（同一台设备、同一档位）

```
coverage=0.587  span_ms=66.74（含 extrap_ms=24.97）  gap_end_ms=0.00
frame_dev_ratio=1.0003  gyro_stamp_ratio=1.0012  pushed=9 samples=6 buf=124
status=ok frames=450 fps=14.93 est=gicp rot_gated=0 |t|=0.25mm |r|=0.01deg  [E] 行数=0
```
⇒ 有效覆盖从 33.1 ms 变成 66.7 ms = 帧周期，残缺口归零；输出侧无回归；新报警在结构值 0.58 下保持沉默。

单测：`follow/build ctest` 4/4 绿，新增 `Gyro.ExtrapolatesStructuralTailGapWithoutFakingCoverage`
（15 fps 半窗缺席必须补满、30 fps 整窗缺席仍可用、真停更 300 ms 不许外推蒙混）；
`app/.../build ctest` 2/2 绿（`GyroTimeBaseSeam` 的角度比例断言已随新语义改成 `span + extrap`）。

**仍未验证的一项（要人动手）**：补积后的旋转与视觉旋转在**真实平移下**是否仍在 1° 门内。
这需要手持相机做 10~30 dps 的慢转并看 `rot_gate_err_deg`，静态场景给不出这个证据。

### 12.4 阶段耗时（1.2）第一次量出来的结果

`compute_ms` 拆成 `extract / cloud / sparse / dense` 四段 + `iterations`（µs 粒度，进
`/follow/status` 的 `stage` 组、逐帧遥测行 `it= [ex cl sp dn]`）。生产档位 640x480@15 hw：

| 阶段 | 修前 p50 | 修前 p95 | 说明 |
|---|---|---|---|
| `extract`（CPU 特征前端） | **28.3 ms** | 36.7 | **单帧成本的最大头（53%），不是 GICP** |
| `dense`（GICP） | 21.9 ms | 27.3 | 7.4k 对应点 |
| `sparse`（帧间特征互验） | 1.2 ms | 1.8 | 96% 的帧都在跑，很便宜 |
| `cloud`（深度→点云） | 0.5 ms | 0.5 | 可忽略 |
| `compute` 合计 | **53.0 ms** | 58.0 | 四段相加 51.9 ≈ 合计，插桩自洽 |

三条**推翻既有说法**的读数：
1. §6 把优化重心放在稠密配准上，实测重心在**特征前端**（53% vs 41%）。
2. ~~`iterations` 恒为 0 或 1 ⇒ 该指标没有区分度~~ **此条已撤回（静止场景的采样偏差）**：
   静态 run 里 GICP 本来就一步收敛，看到的 0/1 不代表这个数没信息量。09:24 手持移动+旋转实测
   `iters p50=1 / p95=8 / max=24`（上限 25），**它恰恰是全链路最有区分度的单一指标** —— 见 §13。
   因此 §11.5 用"一次收敛率提高"解释性能改善那条推断在**静止工况下依然不成立**（两边都是 1 次收敛，
   `dense_ms` 的 −49% 来自绑核与点数，不来自迭代数），但在运动工况下 `iterations` 是可用的诊断量。
   两个场景要分开写，不能拿一个场景的读数去否定指标本身。
3. `sparse 参与帧占比 96%`：帧间互验几乎每帧都在跑，和"`estimator=sparse` 出现率 2.5%"是
   **两个不同的数**（前者是初值 tier-1 参与率，后者是稠密失败后的回退率）—— 之前把这两个
   混为一谈过，拆开后必须写清各自含义。

### 12.5 绑核 + OpenCV 并行度归位（1.3）实测收益

`FollowWorker::loop()` 开头：线程命名 `follow_worker`、`pthread_setaffinity_np` 到 4-7、
`cv::setNumThreads(cfg_.track.threads=4)` 并打出生效值（此前 `track.threads` 的注释一直写着
"绑 A76"，但全进程 76 条线程里 71 条允许集是 `0-7`，一处都没设过）。

| 指标 | 绑核前 p50 / p95 | 绑核后 p50 / p95 | Δ |
|---|---|---|---|
| `compute` | 53.0 / 58.0 | **37.0 / 44.0** | **−30% / −24%** |
| `dense` | 21.9 / 27.3 | **11.1 / 12.2** | **−49%** |
| `extract` | 28.3 / 36.7 | 23.6 / 34.6 | −17% |

输出侧无回归：`fps 14.96 dropped 0 rot_gated 0 |t|=0.17mm |r|=0.007°`，`[E]` 行数 0；
单帧占预算从 79% 降到 55%。

**没吃到的那一半，原因是可查的**：`/proc/<pid>/task/*/status` 显示 OpenCV 并行池的 3 条 helper
线程允许集仍是 `0-7`，实测落在 `cpu2`（A55）上跑 —— OpenCV 的池是**进程级、由第一个触发并行段的
线程创建**，跟随线程自己绑核带不动已经建好的池。所以 `extract` 只降了 17%。
要把这 23.6 ms 继续压下去，方向不是再绑一次核，而是让池的**创建**发生在大核上
（或在 follow 前端显式禁用 OpenCV 并行、改由本线程串行跑 NEON 路径 —— 后者要实测才能选）。

**未验证**：本次只验了"跟随 + 空闲推流"。绑核后 follow 家族（含继承亲和性的 OMP/GICP 线程）
最多同时占 4 条 A76，**编码 / SSE / HTTP 在同一档位下的尾部延迟还没测**，这决定了
"要不要给它们留一条大核"（§6 表格第 1 行的原警告依然有效）。

## 13. 第三轮：手持移动+旋转实测（09:24–09:27，468 帧）

这一 run 是 §12.3 里"要人动手"的那一项：真实角速度下同时验**陀螺通道是否真在起作用**和
**离群门是否守得住**。结论分两半：陀螺侧修好了，配准侧暴露出新瓶颈。

### 13.1 陀螺侧：机制 C 的修复在真实运动下成立

| 读数 | 值 | 判读 |
|---|---|---|
| `cov` | p50 0.59 / p95 0.63 / max 0.70 | 与静止 run 的 0.57~0.61 一致 ⇒ **缺口是结构性的、与速度无关**，坐实机制 C |
| `gyro_samples` | min 6，**0 样本帧 = 0** | 每个窗口都有真样本进来；补积尾巴（`extrap`）在跑 |
| `gyro_pushed` | min 0 | 交付滞后仍偶发（尾部半帧没到），但已被 `extrap` 吸收 |
| `resid`（陀螺角速率） | p50 28 / p95 71 / **max 115 dps** | 相机确实在转，不是静止场景冒充的运动场景 |
| `still YES` | 31 / 468 | 手持停顿被正确识别；冻结 2 次到达 30 s 上限后强制解冻（既有兜底，符合设计） |

`ik_failed` 11 次是 Python 侧"本帧不采纳、保持上一目标"，与配准失败不同源，属正常守卫。

### 13.2 配准侧：高角速度下 GICP 跑满迭代 → 退化 → 丢帧（新瓶颈）

| 指标 | p50 | p95 | max |
|---|---|---|---|
| `iters` | 1 | 8 | **24**（上限 25；≥10 的 22 帧，≥20 的 13 帧） |
| `compute` | 35.0 | 48.5 | 85.0 |
| `extract` | 23.4 | 28.2 | 35.2 |
| `dense` | 9.6 | 23.1 | **62.7** |
| `inl` | 4887 | 6296 | 7425 |

状态分布：`ok/gicp 459` · `ok/sparse 7` · **`lost/none 2`**。典型退化链（帧号连续，时间相隔 11 s）：

```
09:26:05 #870   ok/sparse   inl=1864(0.62) sT=inf sR=inf it=24 [ex 21.6 dn 21.7] gyro=9/5 cov=0.59 resid=22.3
09:26:16 #1038  lost/none  <保持上一位姿>   sT=inf sR=inf it=24 [ex 20.1 dn 35.0] resid=32.9  |r|=48.6°
09:26:16 #1041  lost/none  <保持上一位姿>   sT=inf sR=inf it=24 [ex 25.6 dn 46.3] resid=85.5  |t|=121.8mm
```

`|r|` 峰值 53.6°、`|t|` 峰值 121.8 mm —— 单帧真实位移已经大到初值不够准，GICP 从 25 次迭代预算
里跑不出来，于是稠密失败、稀疏回退也失败、最终 hold 上一位姿。**运动速度现在是第一约束，CPU 不是。**

### 13.3 最关键的一条负面结论：离群门全程 0 触发，丢的帧没有任何独立信息源拦截

全场 `rot_gated` 计数为 0（该门触发即打 WARN"离群门拦下坏帧"，日志中一条都没有），而生效门限是
`gyro_rot_gate_deg: 10.0`。含义要说准：

- 门没触发**不代表门工作正常**，而是 10° 门限在这个速度下根本不会触发 —— 单帧 66 ms、115 dps 时
  陀螺积分给出的帧间旋转本身只有约 7.6°，任何视觉解都在 10° 以内。这与 §11.9 第 5 条是同一个错误
  的复现（那次是"静止时 rot_gated=0 不能证明门有效"，这次是"运动时同样不能"）。
- 于是 #1038/#1041 两帧的 `lost` 是**靠 GICP 自己失败才发现的**，陀螺这个独立信息源全程作壁上观。
  `gyro_used` 只在初值层参与，拦截层等于关闭。
- 门限应该按"单帧内陀螺角位移量级"标定而不是给一个绝对度数：15 fps 下 10° 门 ≈ 150 dps 才开始有意义。
  可落地的做法是让门限随帧间隔缩放（`gate = k · gyro_angle_this_frame + b`），或直接降到 2~3°
  （§12.2 补积修复之后，尾巴缺口已经补上，降低门限不再有"把外推误差当离群"的风险）。

### 13.4 下一步候选（按性价比）

1. ~~**离群门限改成随帧间隔缩放 / 降到 2~3°**~~ ✅ **已实施**，见 §13.6。
2. `iters` 是运动工况下最有区分度的单一指标（p50 1 → p95 8 → max 24），**建议进 20 Hz pose 推送负载**，
   作为"跟不跟得上"的实时健康位；≥20 连续 N 帧即提示减速。
3. 运动模糊 / 点云质量：`inl` 掉到 1864 与 2930 的两帧是退化前兆，值得同时上报 `inlier_ratio` 的
   短窗斜率，比看单点值更早。
4. 想真正吃下 115 dps，方向不是继续压 CPU（`compute` p50 35 ms 已在预算内），而是**给稠密配准更好的
   初值**：`gyro_used` 目前只在稀疏解缺席时顶上来，可以把陀螺积分升为 tier-0 旋转初值（平移仍由
   稀疏解给），这是结构性改动，单独评估。

### 13.5 本轮（第三轮）修正的自述

- **§12.4 第 2 条撤回**："`iterations` 恒为 0/1 ⇒ 无区分度"是静止采样偏差，见 §12.4 就地更正。
- §11.9 第 5 条的教训在运动场景**再次应验**：`rot_gated=0` 在任何速度下都不能当作"门有效"的证据，
  必须交叉核对门限与当帧陀螺角位移量级。

### 13.6 实施：离群门改成动态门限（本地改+验证，未提交）

判据从 `err > 常数` 换成

```
limit = gyro_rot_gate_deg + gyro_rot_gate_relax × 本区间陀螺实际转角
```

转角取 `AngleAxis(integrate_gyro(...)).angle()`，所以**帧间隔与被 hold 的帧数被自动吸收**
（区间越长、转得越多 ⇒ 门越宽），不需要额外的 dt 参数。生产值 `2.0 + 0.5·θ`：静止 ⇒ 门 2°，
115 °/s 单帧（θ=7.6°）⇒ 门 5.8°，仍是旧 10° 门的一半不到，但低速那一档从"永远不触发"变成"能拦"。

| 文件 | 改动 |
|---|---|
| `follow/include/follow/odometry.hpp` | `gyro_rot_gate_deg` 语义改为**零速下限**（默认 1.0→2.0）；新增 `gyro_rot_gate_relax`（默认 0.5）；`TrackResult` 新增 `rot_gate_limit_deg` |
| `follow/src/odometry.cpp` | `rot_gate` 内计算动态门限并写出 `rot_gate_limit_deg`；`relax` 负值按 0 夹住 |
| `follow/src/config_loader.cpp` | 读 `gyro_rot_gate_relax`、有限性检查、两条体检、启动摘要打印"下限+斜率" |
| `configs/aisprayer_config.yaml` | `gyro_rot_gate_deg: 10.0 → 2.0`，新增 `gyro_rot_gate_relax: 0.5`，注释写明 10° 已被真机证伪 |
| `app/.../follow_worker.{hpp,cpp}` | 快照 `rot_gate_limit_deg`；拦截 WARN 打印**生效门限**而非配置常数；逐帧遥测新增 `gate=err/limit`；使能日志打印两个系数 |
| `app/.../http_server.cpp` | `/follow/status` 输出 `rot_gate_limit_deg` |
| `follow/test/test_registration.cpp` | 新增 `GyroRotGateLimitScalesWithGyroAngle`；锁死回归用例显式置 `relax=0`；`describe()` 打印 err/limit |

**为什么必须双重防护负斜率**：`relax < 0` 会让门限随转角收缩，区间够长时变负 —— 而互验误差恒非负，
于是**每一帧都被拦下 = 跟随永久锁死**，比"门关着"糟得多。config_loader 报警 + 运行期 `std::max(0.0, ·)` 夹住。
`relax >= 1` 也单独报一条：门放宽得比实际转角还快，等于回到 §13.3 的惰性状态。

**验证**：`follow/build ctest` 4/4 绿（其中 `--gtest_filter='*GyroRotGate*'` 6/6，含新用例钉的
`limit == 1.0 + 0.5×1.5`、`relax=0` 时精确退回 1.0、无样本时 `limit==0`）；
`app/.../build ctest` 2/2 绿。真机端到端（Gemini 336L 在线、15 fps）：

```
使能日志 ②离群门（动态）下限 2° + 斜率 0.5°/° 陀螺转角
#1   首帧            gate=0.00/0.00      ← 没有合法参照区间，报 0 而不是假数
#77  静止 still=YES  gate=0.02/2.00      ← 下限档：err 比门限低两个数量级，正常放行
/follow/status       rot_gate_limit_deg=2.0005  rot_gate_err_deg=0.0596  rot_gated=0
```

**斜率已用真机数据定值**（09:57:44–09:58:34 手持移动+旋转，307 个有门限的帧，见 §13.7 同期）：

| 量 | p50 | p95 | max |
|---|---|---|---|
| `limit`（动态门限） | 3.50° | 6.32° | 9.74° |
| `err`（视觉 vs 陀螺） | 0.38° | 1.64° | 4.76° |
| **`err/limit`** | **0.11** | **0.40** | 1.07 |

307 帧里 `>0.5` 只有 8 帧、`>0.7` 只有 2 帧、`>1.0` 恰好 1 帧 —— 即 **0.5 的斜率既没松到惰性、
也没紧到误伤，落在预设判据（p95 在 0.3~0.7 之间）的正中，维持 0.5 不动**。

**这条链第一次真的拦下了一个坏帧**（旧的 10° 绝对门下 468 帧 0 次）：

```
#155 ok/sparse  gate=0.14/3.05   resid=56 dps
#156 ok/sparse  gate=0.65/2.74
#157 ok/gicp    gate=1.64/4.77   sT=0.58 σ 健康
#158 rot_gated  gate=4.76/4.44   inl=3719(0.80)  ← 拦下：该区间陀螺只转了 4.9°，视觉报了约两倍
#159 ok/gicp    gate=1.71/7.93   sT=0.55 sR=0.052 ← 立刻用健康 σ 恢复，无级联锁死
#160 lost/none  gate=0.00/0.00   it=24 dn=48.2ms
```

拦下的是"重叠率仍然 0.80、看起来完全正常"的一帧 —— 这正是 §13.3 说的"没有任何独立信息源能看见"
那一类，现在有了。#160 的 lost 是另一件事（初值不够好 ⇒ 稠密跑满迭代），门不该也不需要管它。

### 13.7 使能 503（"使能失败，超时"）复现与判定

用户报告：本轮启动 follow 时报使能超时。日志证实，并给出精确的失败点：

```
09:57:16 [Follow] 已受理档位切换请求… / setFollowProfile 完成 (took 362ms) / setEnabled snap 更新完成 (took 89ms) / 已使能…
09:57:17 [Status] === Report === … Delivered Stream: 640x480 @ 15fps profile=follow
09:57:32 uvicorn.access: "POST /api/follow/start HTTP/1.1" 503      ← C++ 早就成功了，Python 判死
09:57:44 [HTTP] POST /api/v1/camera/follow enabled=1 -> accepted（第二次点，"使能状态未变，不重复切档"）
09:57:45 teach ok → 200
```

- **C++ 侧 1 s 内就成功了**，503 是 Python 判的：`_wait_switch_done` 里 `fails >= 3` 规则 =
  3 × `SWITCH_POLL_TIMEOUT`(5 s) + 2 × 0.3 s ≈ **15.9 s**，与 09:57:16→09:57:32 的 16 s 完全吻合。
  此时 `SWITCH_DEADLINE`(60 s) 还剩 44 s —— **数失败次数而不是看剩余预算，把一次瞬时卡顿升级成了硬失败**。
- 卡的是"取 follow 快照"这条路径，不是 Python 单侧假象：C++ 自己的 `[Status]` 报告块在
  `Delivered Stream` 与 `Follow (in-proc)` 之间断档 15 s（`backend.console.log` 里同一块断档 ~2 s，
  说明断档真实存在，只是时长被日志中继打时间戳放大）。`/follow/status` 走的是同一把锁，于是三次轮询全超时。
- **与本轮（动态门限）改动无关，可以确定**：门限代码只在 `Tracker::track()` 内执行，而 `track()` 要到
  示教出地图之后才第一次被调用 —— 本次 teach 在 09:57:45，比失败晚 29 s，且失败时快照里 `frames=0`。
  本轮 diff 没有一处触碰使能 / 切档 / 快照 / SSE 路径。
- **不能排除的是本会话更早的绑核改动**：`FollowWorker::loop()` 把跟随线程钉到 4 条 A76 并
  `cv::setNumThreads(4)`，而 §12.5 我当时明确留了一句"编码 / SSE / HTTP 在同一档位下的尾部延迟还没测"。
  这次卡的位置恰好是那条警告指的地址。判定实验很便宜：`track.threads` 设 2（或注释掉那次
  `pthread_setaffinity_np`），反复开关 follow 十次，看 15 s 断档是否还出现。
- 另有一个独立因素，且**是我造成的**：09:56:49–51（使能之前、纯推流阶段）就有
  `waitForFrameset timeout (1/3)~(3/3)` 并触发 pipeline 软重启 —— 进程从启动就不健康。
  直接诱因很可能是我在 09:51:56–09:54 手工起过一个服务实例占着设备随后 kill。历史日志里同一事件
  Aug 25 出现 3 次、Aug 26 出现 30 次，所以这个抖动本身不是新问题。

**下一步（按优先级）**：① 把 `fails >= 3` 改成"连续失败**且**已用预算超过某个比例才判死"，
或让单次轮询超时随重试递增；② 查 `snap_mutex_` 是否在持锁期间向 SSE broker 发布
（`PublishOnRelease` 这个类型名暗示发布就发生在出作用域时，即可能仍持锁）—— 若是，慢客户端会直接
拖住 `/follow/status`；③ 绑核与 HTTP/SSE 尾部延迟的对照实验（上面那条，10 次开关即可判定）。

### 13.8 第四轮（10:17–10:21）：陀螺整轮 0 样本 —— 归因 + 断供报警（本地改+验证，未提交）

**现象**：`still` 恒 `NO`、`gate=0.00/0.00`、示教打印"陀螺样本不足（0 个）"，806/806 帧 `gyro_pushed=0`。
用户侧看到的是"相机放桌上后仿真臂还在微晃"——平移通道 0.7~0.8 mm、旋转 0.10~0.15° 的噪声没人冻住，
因为 P1 的判据是 `still && bias_ready && pushed>0`，三个条件全被断供掐掉。**全程一条 ERROR 都没有。**

**归因（结论：设备侧 IMU 流锁死，不是本轮改动，也不是时间域/参数问题）**

指纹在驱动自己的日志里，两句话就能定死：

| 轮次 | `定标前丢弃样本` | 含义 |
| :--- | :--- | :--- |
| 09:25:03（健康） | 51 | 定标的 8 帧窗口（≈0.26 s）里 IMU 回调按 200 Hz 正常到货，只是时间基还没就绪所以全丢 |
| 09:57:17（健康） | 52 | 同上 |
| 10:18:27（故障） | **0** | 窗口内**一次回调都没进过** |

`dropped_` 只在 `stopPipelineAndSensors()` 里 `reset()` 清零，而清零发生在 IMU `start()` **之前**
⇒ "定标前丢弃=0" 只能读成"这一整轮的陀螺回调数为 0"，与队列、窗口、阈值、门限都无关。
`camera_driver.cpp` 对 HEAD 零改动、yaml 只有 8 行（离群门两项 + 注释）、`enable_imu` 仍为 true、
`Started IMU Gyro stream` 与 `T_cam_gyro 已加载` 都照常打印 —— 也就是说 **`start()` 成功 ≠ 在出数**。

**当场复现（10:53 新起一个干净进程，与页面/后端无关）**：五次 enable（含三次连续关开）全部
`定标窗口内 IMU 回调=0`，`gyro.callbacks` 累计值始终为 0。所以这不是"那一轮运气差"，是设备被锁在
一个不再交付 IMU 的状态里，且**跨进程存活**。

已排除的分支（都留了数）：
- 时间基没定标 / 被 `reset()`：`time_ready=true`、offset 与相邻健康轮同量级；
- 跟踪器窗口/`gyro_horizon` 太短：回调计数本身就是 0，还没走到采样；
- 队列 500 上限溢出：无样本可溢；
- 另一个进程抢设备：`/proc/*/fd` 扫过，只有 1 个进程持有 usbfs 节点；
- **USB 自动挂起**（`ec7a77b` 把 `run.sh` 里关 autosuspend 那段删了，改由 udev 规则负责，
  而现场 `power/control` 仍是 `auto` ⇒ 一度高度可疑）：`runtime_suspended_time = 1068 ms` /
  `connected_duration = 84 740 597 ms` —— 开机至今几乎没挂起过，**这条排除**；`control=auto` 只是没被
  规则改成 `on`，实际 `autosuspend_delay_ms=-1`；
- 拔线重枚举：`dmesg` 在 09:58–10:21 无 disconnect；且 09:58:44 那次重新枚举之后 10:17 那轮仍然哑
  ⇒ 重新枚举救不回来，指向需要**断电复位**（VBUS 未断的软复位不足以清掉 IMU 子系统状态）。

**待验判据（下一步唯一动作）**：相机断电重插（或重启板子）后起服务使能 follow，看
`定标窗口内 IMU 回调` 是否回到 ≈50。回到 ⇒ 结论成立，设备侧锁死；仍为 0 ⇒ 回到 SDK/驱动侧继续查
（下一个可试的是 `ob::Device::hardwareReset()` 与"换 profile 时重建 Sensor 句柄"）。
锁死是否由"今天一天内几十次 `gyro_sensor_->start()/stop()` 循环"累积出来，只能靠计数验证：
每次 enable 都是一次 start/stop。

**落地的报警（本轮改动）**

失效不喊，就等于让运维从"臂在微晃"倒推三跳到"IMU 没样本"。补了两条有名有姓的 ERROR 和一个可查字段：

1. **驱动侧最早的可判点**（`camera_driver.cpp` captureLoop 定标成功处）：定标窗口内回调增量为 0
   ⇒ 立刻 ERROR，指认传感器侧。它在使能后 ≈0.6 s 就喊，比帧级判据早 1.4 s，且不依赖有没有示教。
   未启动陀螺流的档位（硬件档 / `enable_imu=false`）改打"IMU 流=本档未启动"，避免与故障长得一样。
2. **帧侧通用判据**（`checkGyroChannel` 拆成两条独立 tripwire）：旧实现遇到 `gyro_buf<=0` 就
   `return`（把"整条流死了"当成链路健康）⇒ 806 帧静默。现在
   ① 断供：`gyro_pushed==0` 连续 30 帧 ⇒ ERROR，并用**驱动回调计数增量**自动分成因
   （"传感器侧：一次回调都没进" / "时间基侧：到了 N 次却一条都没入队"）；
   ② 时间域错配：保留原判据（`buf>0 && samples==0 && extrap==0`），并修掉文案里那句
   "只有静止检测还在工作" —— 断供时它也不工作。
3. **可观测量**：`FollowSnapshot` 新增 `gyro_alive / gyro_starved_frames / gyro_callbacks`，
   经 `/follow/status` 的 `gyro` 组出到页面（`alive=false` 持续 = 在跟但陀螺已死，与"没在跟"要分开显示）。

**验证**：`cmake --build` 通过、`ctest` 2/2；真机端到端 —— 使能+示教后 1.94 s（第 30 帧）打出
`[Follow] 陀螺断供：连续 30 帧零样本交付（缓冲 0 条）。成因 — 传感器侧：… IMU 回调一次都没进`，
状态侧 `alive=false, starved_frames 432→479 单调上涨, callbacks=0, time_ready=true`，
而 `status=ok fps=15.06` —— 正是"输出看起来完全正常"的那类失效，现在有名有姓。
