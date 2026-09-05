# AiSprayer 多平台兼容方案（基于源码审查）

> **文档版本**: 1.2.0  
> **日期**: 2026-09-05  
> **状态**: 已按评审意见落地实现（未提交 git）  
> **1.2.0**: 保留 Python 相机栈；macOS 真机 Orbbec；统一 `app/scripts/build.sh`；RK3588 仍编 MPP/RGA，不链 OpenH264。
> **目标平台**: RK3588 Linux aarch64、Linux x86_64、macOS（Apple Silicon arm64 / Intel x86_64）  
> **硬约束**: RK3588 上现有 MPP / RGA / 大核绑定 / 零拷贝流水线不得退化；不保留旧 Python 相机栈等历史兼容；优先代码质量、稳定性和性能。

---

## 0. 对 v1.0 草案的审查结论

v1.0 正确指出了「CMake 硬编码 Cortex-A76、链接 `.so`、Linux 亲和性、脚本 GNU 扩展」这几类根因，但有多处与仓库现状不符。1.1 按源码重写，不沿用未核实的结论。

**v1.0 判断正确、应保留**

- 相机 / follow / motion 的 CMake 把 `-mcpu=cortex-a76.cortex-a55` 写进了通用编译选项。
- `librga` / `librockchip_mpp` / `librknnrt` / `libOrbbecSDK.so` 按绝对 `.so` 路径链接，macOS / x86 无法链接。
- `pthread_setaffinity_np` 在 Darwin 上不存在；x86 核数不足时绑 4–7 会失败。
- `run.sh` 使用 GNU `xargs -r`；`motion/scripts/build.sh` 与 `third_party/build.sh` 依赖 `nproc`。
- RGA 源码已有 CPU 降级，但 `#include <rga/...>` 使非 RK 平台无法编译。
- 根目录 `requirements.txt` 引用了不存在的拆分文件。

**v1.0 高估或写错（1.1 已改）**

| v1.0 说法 | 源码事实 |
| :--- | :--- |
| 用 `/dev/mpp_service` 等节点在 CMake 期探测 RK3588 | 交叉编译、Docker、无设备容器都会误判。应以 **OS+arch + 可覆盖 option** 为准，库文件用 `EXISTS` 校验，设备节点只做运行时探测（RGA 已如此）。 |
| 抽 `IColorProcessor` / `IVideoEncoder` 虚接口 | 30 fps 热路径上加 vtable 没有收益。RGA/MPP 应用 **编译期宏** 隔离，保持现有函数签名。 |
| 软编码用 OpenCV `VideoWriter`，或改 JPEG/MJPEG | 前端 `FloatingCameraZone` 走 `mpegts.js` + HTTP-FLV（`/api/camera/flv`）。软编码必须仍输出 **Annex-B H.264 NALU**，送入现有 `ZlmStreamer::pushH264`。 |
| `orbbec_driver.py` 的 `sys.exit(1)` 会杀死 FastAPI | 生产启动路径是 `camera_service.py` → C++ 子进程，**不 import** `orbbec_driver`。该 `sys.exit` 只伤害独立脚本 / `factory.py`。 |
| `realsense_driver.py` 全工程无引用 | `factory.py` 与 `configs/aisprayer_config.yaml` 仍把它列为可选类型；只是生产不再走这条 Python 栈。 |
| follow 在 RK3588 上用 RKNN SuperPoint | 生产配置 `follow.frontend.kind: "cpu"`，注释写明本板转不出 `.rknn`。CPU 前端（GFTT+SIFT）已经是策略实现。 |
| MobileSAM 将用 MPS | `mobilesam_session.py` 的 `resolve_device()` **已经**按 CUDA → MPS → CPU 选择。 |
| `camera_service.py` 的 `libc.so.6` 会在 macOS 崩 | 已有 `try/except`，`_prctl is None` 时跳过，不崩。缺的是 `DYLD_LIBRARY_PATH`。 |
| `hostname -I` 会让 `run.sh` 退出 | 脚本无 `set -e`，空结果回落 `127.0.0.1`。真正会报错的是 `xargs -r`（同样无 `set -e`，端口可能清不掉）。 |
| 删除 `third_party/src/rga/samples` | `third_party/build.sh` 从该源码树编 `librga`。samples 约 **8.5 MB**，不是「大量磁盘」，且不该动厂商树。 |
| Inexbot SDK 是跨平台阻塞项 | `robot_service` 仅在 `robot_type == "inexbot"` 时才 import。生产配置是 `dobot`。它是历史包袱，不是编译阻塞。 |
| 机械臂两套驱动「格式化重复、应抽到 robot_service」 | 未发现可安全合并的公共格式化器；Dobot 与 Inexbot 协议不同。此项从重构清单移除。 |

**v1.0 漏掉的硬问题**

1. **`third_party/build.sh` 是比各模块 CMake 更大的锁定源**：`nproc`、`dpkg`、`-mcpu=cortex-a76...`、只认 `lib*.so`、无条件编 RGA/RKNN/Faiss。
2. 相机与 follow 的 CMake **连 arch 都不判断**就把 A76 flag 加上；motion 至少包了 `aarch64|arm64`，但会误伤 Apple Silicon。
3. `pthread_setname_np`：Linux 两参数，macOS 只接受当前线程名一个参数。只包亲和性不够。
4. 相机 CMake 写死 `-fopenmp`；follow `find_package(OpenMP REQUIRED)`。Apple Clang 默认无 libomp。
5. `FOLLOW_ENABLE_RKNN` 默认 **ON**，相机会 `add_subdirectory(follow)` 并链接 `librknnrt.so`，即使运行时用 CPU 前端。
6. `main.cpp` 在 MPP 初始化失败时继续跑，但 `mpp_frm_ptr == nullptr` 时整段推流被跳过——**RK3588 上 MPP 失败也会静默无画面**，非 RK 更是必然。
7. `RgaProcessor::rgbToNv12` 持锁后再调 `bgrToNv12`（同把非递归锁）→ 死锁。当前 `main.cpp` 只调 `bgrToNv12`，属潜伏缺陷。
8. CPU I420 回退不缩放：`cvtColor` 按源尺寸，memcpy 按 `out_w*out_h`，分辨率不一致会越界。
9. 存在两套相机栈：C++ `orbbec_camera_service`（生产）与 Python `orbbec_driver` / `realsense_driver` / `factory.py`（遗留）。
10. C++ `planner`（Tesseract / PCL / Noether）不在 Web 应用关键路径上，不应塞进本轮兼容。
11. `pyorbbecsdk2` 只出现在遗留 Python 驱动里；`app/requirements.txt` 却无条件依赖它。wheel 仅有 **cp312** 的 linux_aarch64 / linux_x86_64。
12. `query_hw_d2c.cpp` 自带 `main()`，未列入 CMake，是未接入构建的诊断残件。
13. `third_party/build.sh` 仍编 Faiss，`app/src` 内 **零引用**。
14. `MppEncoder::encode()` 注释写 Legacy fallback，实现只是 `memcpy` 进 MPP DMA 再 `encodeDirect`，**不是**软件编码器。
15. `follow_worker.cpp` 用了 `cpu_set_t` 但只 `#include <pthread.h>`，依赖 glibc 间接带上 `<sched.h>`；可移植编译会裸奔失败。
16. `InexbotDriver` 未实现 `BaseRobotDriver.get_current_joint`；`get_current_pose` 未做度→弧度（Dobot 做了）。生产默认 dobot，属遗留缺陷，不是本轮平台阻塞。
17. `reconstruction_service.py` 已记录 open3d 0.19 aarch64 Poisson 等值面竞态/段错误，并用子进程隔离。这是 **RK3588 上已有的稳定性补丁**，跨平台时不要改回进程内直接跑。
18. 前端 FLV + `mpegts.js` 在 Safari 上 MSE 支持弱；macOS 开发以 Chrome/Firefox 验收，不把 WebRTC 当本轮必做项。

---

## 1. 系统实际怎么跑（兼容方案必须贴这条路径）

```
run.sh
  ├─ FastAPI (app/src/main.py)
  │    lifespan:
  │      camera_service.start_stream()     # 拉起 C++ 子进程，不是 Python OrbbecDriver
  │      sam_service.initialize()          # MobileSAM，已支持 MPS
  │    业务: 标定 / follow / interactive / robot(Dobot TCP)
  │    运动学: ctypes → libmotion_c.(so|dylib)  （kinematics.py 已会找 dylib）
  │    路径验证: subprocess → motion_cli
  └─ Vite 前端
       FloatingCameraZone → GET /api/camera/flv → 反代 ZLM HTTP-FLV
```

C++ 相机进程热路径（`main.cpp` 218–271 行）：

```
CameraDriver::waitForNextFrame
  →（标定）CornerDetector::feedFrame
  → RgaProcessor::bgrToNv12(display, mpp_dma_ptr)    # 零拷贝写入 MPP DMA
  → MppEncoder::encodeDirect(...)
  → ZlmStreamer::pushH264(...)                       # mk_media_input_h264
```

前端不消费 WebRTC。`third_party/build.sh` 编 ZLM 时 `ENABLE_WEBRTC=OFF`，`ZlmStreamer::getStreamInfo` 仍拼 `webrtc_url`，当前 UI 不用，属文档/API 残留。

**含义**：兼容层必须保住「H.264 NALU → ZLM → HTTP-FLV → mpegts.js」。换 MJPEG / `VideoWriter` 等于改前端协议，不作为本方案路径。

---

## 2. 不兼容根因（按构建 / 运行分层）

### 2.1 构建系统：无条件 RK3588 代码生成

| 位置 | 事实 | 在非 RK 上的结果 |
| :--- | :--- | :--- |
| `orbbec_camera_service/CMakeLists.txt:9-10` | 无 `if`，直接加 `-mcpu=cortex-a76.cortex-a55 -mtune=cortex-a76` 与 `-fopenmp` | Clang/gcc 报 unknown target CPU；Apple Clang 不认 `-fopenmp` |
| 同文件 `:31` | `link_directories(/usr/lib/aarch64-linux-gnu)` | x86/mac 上无效，本身不致命 |
| 同文件 `:94-97` | 硬链 `libOrbbecSDK.so` `librga.so` `librockchip_mpp.so` `libmk_api.so` | 缺库或后缀为 `.dylib` 时链接失败 |
| `follow/CMakeLists.txt:14-15` | 同样无条件 A76 flag | 同上 |
| 同文件 `:28` | 同样写死 aarch64 库目录 | 同上 |
| 同文件 `:65,97-100` | `FOLLOW_ENABLE_RKNN` 默认 ON，链 `librknnrt.so`，定义 `HAS_RKNN` | 非 RK 无此库即失败。相机 `add_subdirectory(follow)` 会吃到这个默认值 |
| `motion/CMakeLists.txt:13-16` | `CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64\|arm64"` 时加 A76 flag | **Apple Silicon 的 `arm64` 命中**，macOS 编 motion 失败 |
| 不存在 `app/cmake/` | 三套 CMake 各写一遍 | 行为会再次分叉 |

`planner/CMakeLists.txt` 无 A76 flag，但依赖 Tesseract/PCL/Noether，本轮不纳入（见第 8 节）。

### 2.2 源码：头文件与系统调用绑定

**编译期不可移植（缺头文件即失败）**

- `rga_processor.cpp:2-4`：`<rga/im2d.hpp>` 等。`init()` 已用 `open("/dev/rga")` 做运行时降级（22–32 行），**编译仍必须有 RGA 头**。
- `mpp_encoder.hpp:7-12` 与 `mpp_encoder.cpp:2`：全套 Rockchip MPP 头。无软件编码器，也无 `#ifdef`。
- `camera_driver.cpp:2`：`<libobsensor/ObSensor.hpp>`。当前 `third_party/install/lib/libOrbbecSDK.so.2.9.3` 是 **aarch64 ELF**。
- `follow/src/frontend.cpp`：`HAS_RKNN` 下的 SuperPoint 已隔离；默认 ON 仍会链 NPU runtime。
- `corner_detector.cpp:5,9-20` 与 `follow_worker.cpp:655-661`：`<sched.h>`、`cpu_set_t`、`pthread_setaffinity_np`、两参数 `pthread_setname_np`。

**运行期可降级（已写，但编译不到）**

- RGA 打不开 `/dev/rga` 时走 `cv::cvtColor(BGR2YUV_I420)` + I420→NV12。
- follow `make_frontend("cpu")` 不依赖 NPU；生产 yaml 已是 `kind: cpu`。
- `camera_service.py:28-32` 加载 `libc.so.6` 失败则不做 `PR_SET_PDEATHSIG`。

**脚本 / Python 环境**

- `third_party/build.sh:47`：`JOBS=$(nproc)`，`set -euo pipefail` → macOS 直接退出。
- 同文件 `:322`：`dpkg -L libopenblas-dev`；`:371-372`：编 OrbbecSDK 时再灌 A76 flag；产物只认 `lib*.so`。
- `app/scripts/run.sh:48,110`：`xargs -r`（BSD xargs 非法选项）。
- `app/scripts/env.sh:71-76`：对 `third_party/*.whl` 全装。现有两个 wheel 均为 **pyorbbecsdk2 2.1.1 cp312**（aarch64 / x86_64）。macOS 会静默失败（`|| true`）；aarch64 会多试一次 x86 wheel。
- `app/requirements.txt` 写死 `pyorbbecsdk2>=2.1.0`。该包只被遗留 Python 驱动使用；macOS 上 PyPI 通常无 wheel。
- `system_deps.sh`：`apt-get` / `dpkg`，不能在 macOS 跑。
- `orbbec_camera_service/run.sh`、`motion/scripts/build.sh`、`planner/run_planner.sh`：均用 `nproc`。
- `motion/scripts/build.sh:8` 回显写死 `libmotion_c.so`；CMake 在 macOS 会出 `libmotion_c.dylib`（`kinematics.py:26` 已覆盖）。

### 2.3 已核实的二进制架构

本机 `third_party/install`（2026-09-05）：

| 文件 | `file(1)` |
| :--- | :--- |
| `libOrbbecSDK.so.2.9.3` | ELF aarch64 |
| `librga.so` / `librockchip_mpp.so` / `librknnrt.so` / `libmk_api.so` | 随板构建的 aarch64（RK 专用库在 x86/mac 无意义） |
| `inexbot_v24_03_py38/_nrc_host.so` | **ELF x86-64**（RK 与 mac 都加载不了） |

`libfaiss.a` 已安装，应用代码未使用。

---

## 3. 设计原则

1. **RK3588 零退化**：MPP DMA 零拷贝、RGA `imcvtcolor`、`encodeDirect`、绑 A76 4–7，保持现有实现与调用顺序。隔离用编译期宏，不在热路径加虚函数，不为「抽象完整」再拷一份 RGA/MPP。
2. **按能力裁剪，不按「假装自己是 3588」**：Linux aarch64 默认开 RK 能力；Apple / x86 默认关。允许 `-DENABLE_RK3588=OFF` 在板子上做对照构建。
3. **CMake 期不探测 `/dev/*`**：设备节点只用于运行时（已有 RGA 逻辑）。交叉编译与 CI 必须可重复。
4. **推流协议不变**：任何平台的编码器都输出 H.264 NALU，走现有 ZLM + `/api/camera/flv`。
5. **不保留旧 Python 相机栈**：生产已切 C++。`orbbec_driver.py` / `realsense_driver.py` / `factory.py` / `test_camera.py` / `visualize.py --camera` 按死代码处理，而不是再包一层 HAL。
6. **本轮不做历史兼容**：可删失效入口、改 CMake 默认、改脚本。不维持「根目录 requirements 拆分」「Inexbot 必装」等旧约定。
7. **能少建类型就少建**：follow 已有 `FeatureFrontend`；亲和性抽一个头文件即可；色彩/编码用 `#ifdef` + 同签名函数，不新开接口层级。

---

## 4. 方案

### 4.1 统一平台模块：`app/cmake/platform.cmake`

三份 CMake 改为 `include()` 这一份。探测顺序：

```cmake
if(APPLE)
  set(AIS_PLATFORM "macos")
elseif(UNIX)
  set(AIS_PLATFORM "linux")
endif()

# Apple Silicon 是 arm64，但绝不是 RK3588
if(AIS_PLATFORM STREQUAL "linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(ENABLE_RK3588_DEFAULT ON)
else()
  set(ENABLE_RK3588_DEFAULT OFF)
endif()
option(ENABLE_RK3588 "Rockchip MPP/RGA/NPU" ${ENABLE_RK3588_DEFAULT})

# 库按「存在」开关，避免 option=ON 但 third_party 未装时configure 成功、link 失败
set(TP "${PROJECT_ROOT}/third_party/install")
if(ENABLE_RK3588)
  foreach(_lib IN ITEMS librga.so librockchip_mpp.so)
    if(NOT EXISTS "${TP}/lib/${_lib}")
      message(FATAL_ERROR "ENABLE_RK3588=ON 但缺少 ${TP}/lib/${_lib}，先跑 third_party/build.sh 或 -DENABLE_RK3588=OFF")
    endif()
  endforeach()
  add_compile_definitions(HAS_RGA=1 HAS_MPP=1)
endif()

option(FOLLOW_ENABLE_RKNN "RKNN SuperPoint（生产 yaml 为 cpu，默认关）" OFF)
# 仅当显式打开且库存在时链 librknnrt 并定义 HAS_RKNN
```

架构 flag（只加优化，不加错误的 CPU 名）：

| 条件 | flag |
| :--- | :--- |
| `ENABLE_RK3588` | 现有 `-mcpu=cortex-a76.cortex-a55 -mtune=cortex-a76 -ftree-vectorize -fomit-frame-pointer`（一字不改） |
| macOS | 不加 `-mcpu=apple-m1`（伤 Intel / 新芯片）。`-O3` + 工程原有 `-Wall` |
| Linux x86_64 | `-O3`，**不**默认 `-mavx2` / `x86-64-v3`（老机器、虚拟机、Rosetta 会挂）。需要时 `-DAIS_X86_MARCH=x86-64-v3` |
| 其他 | `-O3` |

OpenMP：一律 `find_package(OpenMP)`，用 `OpenMP::OpenMP_CXX`，**删除**相机 CMake 里手写的 `-fopenmp`。macOS 文档写明 `brew install libomp` 并设 `OpenMP_ROOT`。缺 OpenMP 时 follow/small_gicp 应 `FATAL_ERROR` 并给出安装说明，而不是偷偷编成单线程还以为在测性能。

链接：`find_library` / 按 `CMAKE_SHARED_LIBRARY_SUFFIX` 找 `mk_api`、`OrbbecSDK`，禁止写死 `.so`。

### 4.2 相机进程：编译期分流，热路径不动

#### 色彩转换

保留 `RgaProcessor` 文件，头文件与 RGA 调用包在 `HAS_RGA` 里。无 RGA 时同一 `.cpp` 只编 CPU 路径（从现有 73–92 行抽出），并修两处缺陷：

- 尺寸不一致先 `cv::resize` 再转，或直接 `cvtColor` 到目标尺寸（OpenCV 无一步 BGR→NV12 缩放时：resize + `COLOR_BGR2YUV_I420`）。
- `rgbToNv12` 不得在持锁时调 `bgrToNv12`；抽无锁内部函数，或删除未使用的 `rgbToNv12`（全仓库只有声明和定义，`main.cpp` 未调用）。

`main.cpp` 仍写 `rga->bgrToNv12(...)`，RK3588 目标码与现在一致。

#### 视频编码

| 平台 | 实现 | 输出 |
| :--- | :--- | :--- |
| `HAS_MPP` | 现有 `MppEncoder::encodeDirect`，DMA 指针仍由 `getFrameBufferPtr()` 提供 | Annex-B H.264 |
| 非 MPP | 新增 `SoftwareH264Encoder`（OpenH264，BSD）。输入 NV12/BGR，输出 Annex-B | 同一 `pushH264` |

不采用：

- OpenCV `VideoWriter`（不是为逐帧 NALU 推流设计，依赖系统 ffmpeg 插件）。
- 把 ZLM 打开 `ENABLE_FFMPEG` 当编码器（体积与许可证复杂，且把编码推进 ZLM 后 RK 路径反而分叉）。
- 首选 JPEG/MJPEG（要改 `FloatingCameraZone`）。

`main.cpp` 伪代码（编译期，不是虚函数）：

```cpp
#ifdef HAS_MPP
    auto enc = std::make_shared<MppEncoder>();
#else
    auto enc = std::make_shared<SoftwareH264Encoder>();
#endif
    // 色彩：有 MPP 时写入 enc->getFrameBufferPtr()；无则写入普通 nv12 缓冲
    // 编码：HAS_MPP 走 encodeDirect；否则 encode(nv12)
    zlm->pushH264(...);
```

软件编码器失败必须在日志里报致命错误并反映到 `/api/v1/camera/status`（例如 `encoder: "none"`），禁止再出现「进程活着、FLV 永远无数据」。此修复 **RK3588 在 MPP init 失败时同样受益**。

OpenH264 放入 `third_party`，由 `build.sh` 在非 RK 配置构建；RK 配置不链它，避免板端多一个无用依赖。

#### 取流：Orbbec 与 Replay

`CameraDriver` 继续是 Orbbec 独占设备封装（含 D2C、IMU、换档、标定关深度）。不要把它改成大而全的 HAL。

在其旁增加 **Replay 源**（名称可 `ReplayFrameSource`），满足 `waitForNextFrame` / `getLatestFrame` / 内参查询这些消费者已经在用的契约：

- 配置建议：`hardware.camera.source: auto | orbbec | replay`（不要发明未实现的 `allow_mock`）。
- `auto`：Orbbec 枚举失败则 replay；无样本则保持 offline，**不**假装在线。
- 样本：优先已有 `data/template_group/...` 的 color + depth + 内参 yaml，按配置 fps 节拍推送。
- Replay 必须能支撑：前端画面、标定 UI 看图、interactive 读已有扫描、follow 离线回归。真机示教 / 硬件 D2C / IMU 在 replay 下明确标 `degraded`。

无 Orbbec SDK 的平台（典型：macOS 无官方 v2 二进制）只编 Replay + 软编码，`HAS_ORBBEC=0`。Linux x86 若官方 OrbbecSDK_v2 能编过，则 `HAS_ORBBEC=1`，可插真实 Gemini。

`follow` 作为子项目时继续 **不** 链 `follow_device`（现有注释已说明原因：同进程第二份 Pipeline）。这一点保持。

#### 线程名与亲和性：`platform_thread.hpp`

一处实现，两处调用替换（`corner_detector.cpp`、`follow_worker.cpp`）：

```cpp
inline void set_current_thread_name(const char* name);
inline bool pin_to_cpus(const int* cpus, size_t n);  // 失败只打日志
```

- Linux：现有 `pthread_setname_np(pthread_self(), name)` + `pthread_setaffinity_np`。
- Darwin：`pthread_setname_np(name)`；亲和性空操作返回 `true`（内核 QoS）。
- 绑核前用 `sysconf(_SC_NPROCESSORS_ONLN)` 丢掉越界 CPU，避免 x86 四核上 `EINVAL`。
- RK3588 调用点仍传 `{4,5,6,7}`，日志文案保持「A76」。

不要在通用 Linux 上「尽量绑 4–7」——那是 RK 拓扑，不是 x86 大核。非 `ENABLE_RK3588` 不绑。

### 4.3 follow / motion（已有的可复用，少做新抽象）

**follow**

- 算法核（`libfollow` + CPU 前端 + small_gicp）三平台都编。
- `FOLLOW_ENABLE_RKNN` 默认改为 **OFF**。板端若将来有 `.rknn`，显式 `-DFOLLOW_ENABLE_RKNN=ON`。与当前生产 yaml 一致，不是功能回退。
- `follow_device` / `follow_node` 仅在 standalone + `HAS_ORBBEC` 时编。

**motion**

- 去掉 `aarch64|arm64` → A76 的误判；只在 `ENABLE_RK3588` 时加该 flag。
- 依赖仅 Eigen / yaml-cpp / tinyxml2，是三平台里最容易先打通的 C++ 模块。
- `build.sh`：`JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)`；产物提示同时写 so/dylib。
- Python 侧已能加载 dylib，只需 C++ 能编过。

### 4.4 `third_party/build.sh` 按配置构建

这是兼容能否落地的前提。建议：

```
bash third_party/build.sh                  # 自动选配置
bash third_party/build.sh --profile rk3588
bash third_party/build.sh --profile generic
```

| 组件 | rk3588 | generic（x86 Linux / macOS） |
| :--- | :--- | :--- |
| RGA / MPP / RKNN | 保持现状（MPP 段已注释、用预装 `.so` 也可以，不要为「整洁」改板端已工作的路径） | **跳过** |
| ZLMediaKit | 现状 | 编，产物 `libmk_api${CMAKE_SHARED_LIBRARY_SUFFIX}` |
| OrbbecSDK_v2 | 现状（可保留 A76 flag） | Linux x86：官方源码、**不加** A76 flag；macOS：探测官方是否支持，不支持则跳过并依赖 Replay |
| OpenH264 | 跳过 | 构建并安装 |
| Faiss | **停止默认构建**（应用未用）。`--only faiss` 保留给将来 | 跳过 |
| `nproc` / `dpkg` | 可用 | 用 `sysctl` / brew 前缀；禁止 `dpkg -L` 作为唯一路径 |

`JOBS` 必须在脚本开头可移植。`set -euo pipefail` 保留。

### 4.5 Python 与启动脚本

**`app/requirements.txt`**

- 去掉无条件的 `pyorbbecsdk2`（只服务即将删除的 Python 驱动）。
- 保留 `open3d` / `torch`（interactive 重建与 MobileSAM）。macOS / x86 用官方 wheel；RK aarch64 维持现有已验证安装方式，不在本方案里重写 torch 源码编译。

**`app/scripts/env.sh`**

- 按 `uname -s` / `uname -m` / `python` tag 过滤 wheel：只装匹配的 `*linux_aarch64.whl` / `*linux_x86_64.whl`；Darwin 跳过并打印原因。
- 不把根目录失效 `requirements.txt` 当作入口。

**`app/scripts/run.sh`**

```bash
# 清端口：不用 xargs -r
pids=$(lsof -t -iTCP:"$port" -sTCP:LISTEN 2>/dev/null || true)
# 可移植拆分 pid，逐个 kill

# LAN IP
if [ "$(uname -s)" = Darwin ]; then
  LAN_IP=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || true)
else
  LAN_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
fi
LAN_IP=${LAN_IP:-127.0.0.1}
```

`pkill -f "main.py"` 过于宽，实施时收窄到本仓库路径或记录 PID 文件。属稳定性修复，与平台无关。

**`camera_service.py`**

- 库路径：Darwin 用 `DYLD_LIBRARY_PATH`，Linux 用 `LD_LIBRARY_PATH`。
- `_set_pdeathsig` 保持「仅 Linux 且 libc 加载成功」。
- 自动构建改为 `cmake --build . --parallel`，不要 `make -j4`。
- 构建失败时：**不要**在 lifespan 里把整个 FastAPI 拉倒（现在已经是 `return False`）。补一条清晰日志：指出缺 ENABLE_RK3588 / 缺 Replay 样本 / 缺二进制。
- macOS 上 `preexec_fn` 可用；不必为 Windows 做分支（非目标平台）。

### 4.6 明确不做的事

- 不把 C++ `planner` 迁到 macOS（Tesseract 依赖链）。Web 端自动路径走的是 Python `JeansAutoWaypoints`。
- 不把 Inexbot 编进 aarch64/mac。需要时再在 x86 Linux 上按需加载。
- 不在 CMake 里读 `/proc/device-tree/compatible` 当默认值。
- 不新增 Python `MockCameraDriver` 去替代 C++ 取流。
- 不删 `third_party/src/rga/**`。

---

## 5. 死代码、重复与应修缺陷（实施时一并处理）

### 5.1 建议删除

| 路径 | 依据 | 动作 |
| :--- | :--- | :--- |
| 根目录 `requirements.txt` | `-r requirements-common.txt` / `requirements-vision.txt`，仓库中不存在 | 删除 |
| 根目录 `build_calib_ui.sh` | 依赖 conda env `inexbot`；入口 `src/aisprayer/tools/calib/5.calib_ui.py` **已不存在**；全仓库无脚本/CI 调用 | 删除 |
| `app/src/core/hardware/camera/orbbec_driver.py` | 仅 `factory.py` / `test_camera.py` / `visualize.py --camera` 使用；生产不走 | 删除 |
| `app/src/core/hardware/camera/realsense_driver.py` | 同上；配置里的 `realsense` 选项一并去掉 | 删除 |
| `app/src/core/hardware/camera/factory.py` | 只为上面两个驱动服务 | 删除 |
| `app/src/core/hardware/camera/test_camera.py` | 独立旧预览脚本 | 删除 |
| `orbbec_camera_service/src/query_hw_d2c.cpp` | 自带 `main()`，未进 CMake | 删除，或移到 `tools/` 并单独 target（若仍要现场查 D2C） |
| `visualize.py` 的 `--camera` 分支 | 依赖即将删除的 factory | 删该分支，保留读已有 `scan_dir` 的可视化 |

配置：`hardware.camera.model` 去掉 `realsense`；只保留 C++ 服务认识的字段。

### 5.2 不要删

| 路径 | 原因 |
| :--- | :--- |
| `third_party/src/rga/`（含 samples） | `build.sh` 的 RGA 源 |
| `inexbot_*` | 非阻塞；删驱动是产品决定，不是平台前提。若删，需同时改 factory / 配置 / 前端日志关键字 |
| `app/src/core/planner/` | 本轮范围外，不是死代码 |
| `third_party/MobileSAM` / `sam3` | 运行时或研究依赖 |

### 5.3 重复，应收敛

1. 三处 A76 flag + aarch64 `link_directories` → `platform.cmake`。
2. 两处绑核循环 → `platform_thread.hpp`。
3. 多处 `nproc` → 一小段可复用的 jobs 探测（脚本内函数即可，不必上框架）。

### 5.4 缺陷（与平台无关的也修）

| 等级 | 位置 | 问题 | 修法 |
| :--- | :--- | :--- | :--- |
| 高 | `rga_processor.cpp:95-122` | `rgbToNv12` 持锁调用 `bgrToNv12` 死锁 | 抽无锁实现或删除该函数 |
| 高 | `rga_processor.cpp:73-92` | CPU 回退不缩放，`out_* != in_*` 时 memcpy 越界 | resize 或按源尺寸写并校验长度 |
| 高 | `main.cpp:174-176,216-271` | MPP 失败仍转圈，无画面且 status 仍像在推流 | 编码失败写入 status；非 MPP 走软编 |
| 中 | `follow/CMakeLists.txt:65` | RKNN 默认 ON，与生产 yaml 不符，拖死非 RK 链接 | 默认 OFF |
| 中 | `motion/CMakeLists.txt:13` | `arm64` 包含 Apple Silicon | 改走 `ENABLE_RK3588` |
| 中 | `orbbec_driver.py:28` / `realsense_driver.py:16` | 导入期 `sys.exit` | 随文件删除即消失；若暂留则改抛 `ImportError` |
| 中 | `camera_service.py:118-119` | `make -j4` | `cmake --build --parallel` |
| 中 | `follow_worker.cpp:1-8` vs `:656` | 用 `cpu_set_t` 却未含 `<sched.h>` | 并入 `platform_thread.hpp` 时显式包含 |
| 中 | `mpp_encoder.cpp:266-282` | `encode()` 只是拷进 DMA，易被误当成软编码 | 非 MPP 平台不要复用此函数；另写 OpenH264 |
| 中 | `inexbot_driver.py` | 缺 `get_current_joint`；笛卡尔角未转弧度 | 本轮不修也可（生产是 dobot）。若保留联调路径再补 |
| 低 | `zlm_streamer.cpp:122-125` | 广告 WebRTC 但 ZLM 编时关闭 | 按 `ENABLE_WEBRTC` 省略字段，或文档标明仅 FLV |
| 低 | `run.sh` `pkill -f main.py` | 误杀其它项目 | PID 文件或匹配本仓库路径 |
| 低 | `FloatingCameraZone.tsx` | Safari 对 MSE+FLV 支持差 | mac 开发用 Chromium；不为本轮开 WebRTC |

---

## 6. 兼容矩阵（实施后的目标行为）

| 能力 | RK3588 | Linux x86_64 | macOS |
| :--- | :--- | :--- | :--- |
| FastAPI + 前端 | 保持现状 | 同左 | 同左 |
| 推流 | **MPP 零拷贝 H.264 → ZLM → FLV** | OpenH264 → 同一 ZLM/FLV | 同 x86 |
| 色彩 | **RGA；失败才 CPU** | CPU（修过的 I420/NV12） | 同 x86 |
| 角点 / follow 线程 | **绑 A76 4–7** | 不绑，OpenMP 并行 | 不绑；OpenMP via libomp |
| 取流 | 真机 Orbbec | 真机（若 SDK 编过）或 Replay | Replay（有官方 SDK 再开真机） |
| follow 特征 | 默认 CPU（现状）；可选 RKNN | CPU | CPU |
| follow 配准 | small_gicp + OpenMP | 同左 | 同左 |
| 机械臂 | Dobot TCP | Dobot TCP | Dobot TCP |
| 运动学 | `libmotion_c.so` | 同左 | `libmotion_c.dylib`（加载逻辑已有） |
| MobileSAM | CPU，1 线程（现状） | CUDA 或 CPU | **已有 MPS** |
| C++ planner | 可选，本轮不改 | 可选 | **不做** |
| Inexbot | 不可用（x86 `.so`） | 可按需加载 | 不可用 |

RK3588 列与现在的生产行为对齐；其它列是降级，不是「另一套产品」。

---

## 7. 分阶段实施（批准后再改代码）

阶段之间可停下来在 RK3588 上回归。**每阶段合并前必须在板上跑通相机 30 fps 与 follow CPU 前端。**

### 阶段 A — 清理与平台无关缺陷（不改行为面）

- 删第 5.1 节文件；配置去掉 realsense。
- 修 RGA CPU 回退越界与 `rgbToNv12` 死锁（板端 RGA 成功路径不走这些分支，但失败回退会走）。
- `FOLLOW_ENABLE_RKNN` 默认 OFF（板上无 `.rknn`，运行行为不变）。
- 根目录失效 requirements、旧打包脚本删除。

**验收**：板上 `run.sh`，画面、标定、follow 与现在一致；`ldd orbbec_camera_service` 仍有 rga/mpp。

### 阶段 B — CMake / 脚本 / third_party 分流

- 落地 `app/cmake/platform.cmake`，三模块 include。
- motion 去掉 Apple Silicon 误伤。
- `build.sh` / `run.sh` / `env.sh` / `camera_service.py` 按 4.4–4.5 改。
- `third_party/build.sh` 增加 `generic` 配置。

**验收**：板上 `ENABLE_RK3588=ON` 原样通过；macOS / x86 上 **motion 先独立编过**（依赖最少）。

### 阶段 C — 相机进程可在非 RK 编过并出画

- `HAS_RGA` / `HAS_MPP` 包头文件与链接。
- OpenH264 软编 + 仍用 `pushH264`。
- Replay 源 + `hardware.camera.source`。
- `platform_thread.hpp` 替换两处亲和性 / 线程名。
- status 区分 `RK_MPP_H264` / `OPENH264` / `none`。

**验收**：

- 板：RGA+MPP 路径，绑核日志仍在，帧率与 CPU 占用与阶段 A 对比无回归。
- macOS / x86：无相机也能 Replay 出 FLV；有相机的 x86 再开 Orbbec。

### 阶段 D — 矩阵验证

- RK3588：30 fps 推流、标定角点、follow CPU、手眼采集、interactive 扫描。
- Linux x86：motion 单测 + 相机服务构建 + Replay；有硬件再加真机。
- macOS：motion + 相机服务（Replay）+ FastAPI + 前端 FLV + MobileSAM MPS。
- `orbbec_camera_service` 已有的离线单测（`test_gyro_time_base`、`test_pose_broker`）三平台都跑。
- follow 的 `test_core` / `test_registration` / `test_config` 在无相机机器上跑（本来就是这个设计）。

---

## 8. 范围外（避免方案膨胀）

- C++ `aisprayer_planner` 与 `system_deps.sh` 的 apt/Tesseract 栈。
- Windows。
- 为 Inexbot 做 aarch64/mac 移植或再包一层机器人 HAL。
- 在板上启用 SuperPoint（缺可转换工具链，且生产未用）。
- 把 ZLM 打开 WebRTC 当「兼容项」（前端没用）。
- 清理整个 `third_party/src` 厂商树。

---

## 9. 风险与「零退化」如何保证

| 风险 | 为什么会出 | 控制 |
| :--- | :--- | :--- |
| 宏把 RK 路径编丢 | `#else` 写错、CMake 默认翻了 | 板上禁止用 `ENABLE_RK3588=OFF` 当发布构建；CI/手工对比阶段 A 的日志（RGA init success、PINNED to ... 4–7、encoder 字段） |
| 抽象层多一次拷贝 | 软编缓冲误用到 RK | RK 继续 `getFrameBufferPtr()` + `encodeDirect`；软编代码整份不进 `HAS_MPP` 翻译单元 |
| OpenH264 误链到板上 | generic 库混进 install | `platform.cmake` 在 `ENABLE_RK3588` 时不 `find_library(openh264)` |
| Replay 掩盖真机失败 | `auto` 在现场偷偷回放 | 现场配置 `source: orbbec`；auto 仅开发机；status 必须带 `source` |
| 改 CPU 回退伤到 RGA 失败路径 | 阶段 A 修越界 | 板上可用 `chmod` 暂时挡 `/dev/rga` 做一次回退冒烟（可选） |

---

## 10. 已拍板的决策（实施中）

1. **保留旧 Python 相机栈**（`orbbec_driver.py` / `realsense_driver.py` / `factory.py`）。只把导入期 `sys.exit` 改成 `ImportError`。生产路径仍是 C++ 服务。
2. **软编码用 OpenH264**，前端协议仍是 HTTP-FLV。
3. **macOS 使用真机 Orbbec**（官方 SDK 可构建）；无相机时服务保持原有重连等待，不偷偷 Replay。
4. **`FOLLOW_ENABLE_RKNN` 默认 OFF**（与生产 yaml 的 `kind: cpu` 一致）。
5. **C++ planner / Inexbot** 本轮不动。
6. **统一入口**：`app/scripts/build.sh` 按 host 选择 `rk3588` / `generic`。

RK3588 热路径（RGA → MPP DMA → `encodeDirect` → `pushH264`）保持同一套源码与链接库，`software_encoder.cpp` 不编进板端二进制。
