# 基于 RK3588 硬件加速的 Orbbec C++ 相机服务系统设计方案

> **文件路径**: `app/src/core/hardware/camera/orbbec_camera_service/docs/orbbec_camera_service_design.md`  
> **服务名称**: `orbbec_camera_service`  
> **设计版本**: v1.0.0  
> **适用平台**: Rockchip RK3588 (Linux aarch64 / Ubuntu 22.04 / 20.04)

---

## 1. 项目背景与设计目标

### 1.1 现状与痛点
在原有的 Python 架构（如 `camera_service.py`）中，存在以下性能瓶颈与架构痛点：
1. **Python GIL 与高 CPU 占用**：在 Python 进程中进行 OpenCV 软编码 MJPEG、彩色与深度数据解析、以及棋盘格角点检测，会占用单个 CPU 核心近 100% 算力，导致帧率受限（通常只能跑 15fps 以下）且发热量大。
2. **零拷贝缺失与内存拷贝开销**：原实现中图像数据在 Python/C 绑定、NumPy 数组及网络传输间频繁发生内存拷贝。
3. **推流协议单一与高延迟**：原有方案采用 HTTP MJPEG 流，带宽占用高（单路可达 10~30Mbps），延迟在 200~500ms 左右，无法满足低延迟交互要求。
4. **与上层应用耦合**：直接通过 Python 传输大数组 raw 数据的开销巨大。对于机器人标定与喷涂巡检等业务，上层往往只需触发保存成对的 RGB + 深度文件，无需拉取原始全分辨率数据流到 Python 业务内存。

### 1.2 本方案设计目标
1. **C++ 原生高性能服务**：基于 C++17 重构相机采集、处理、推流与保存流水线，常驻后台运行，提供 HTTP REST API 供 Python/前端控制。
2. **榨干 RK3588 硬件性能**：
   - **RGA 2D 硬件加速**：颜色空间转换（RGB/BGR $\to$ NV12/YUV420P）、缩放、裁剪完全由 RGA 硬件完成，CPU 零开销。
   - **MPP 硬件编解码**：使用 Rockchip MPP 硬件视频编码器进行 H.264/H.265 实时编码，单帧编码耗时 $< 2\text{ms}$，CPU 占用 $< 3\%$。
   - **DMA-BUF / 内存对齐管理**：统一内存管理，实现相机驱动 $\to$ RGA $\to$ MPP 的零拷贝/低拷贝高效流转。
3. **ZLMediaKit 现代低延迟推流**：集成 `libmk_api.so`（ZLMediaKit），输出 RTSP / WebRTC / RTMP / HTTP-FLV 等低延迟流，WebRTC 端到端延迟 $< 100\text{ms}$。
4. **实时角点检测与动态叠加**：在开启标定模式时，对彩色流执行亚像素棋盘格角点检测，并将角点信息实时叠加于编码视频流中供前端可视化，同时对外提供角点坐标查询接口。
5. **异步无锁磁盘 I/O 引擎**：支持通过 HTTP API 指定目录，将对齐的高清彩色图（PNG/JPG）、16位深度图（16-bit PNG/RAW/BIN）及标定元数据异步保存到本地文件，保障取流和推流 0 丢帧、0 卡顿。

---

## 2. 系统整体架构

```mermaid
flowchart TB
    subgraph Hardware["RK3588 硬件层"]
        CAM["Orbbec 3D 深度相机\n(Gemini/Astra 系列)"]
        RGA["Rockchip RGA 2D 硬件加速引擎"]
        MPP["Rockchip MPP 硬件编码器 (H.264/H.265)"]
        DISK["NVMe / eMMC 本地存储"]
    end

    subgraph CoreService["orbbec_camera_service (C++ 核心服务)"]
        subgraph CaptureModule["1. 采集子系统"]
            OBSDK["OrbbecSDK v2 驱动封装\n(Pipeline / StreamProfile)"]
            ALIGN["硬件/软件 D2C 对齐模块"]
            MEMPOOL["DMA-BUF / 内存池分配器"]
        end

        subgraph ProcessModule["2. 图像处理与角点检测"]
            CORNER_DET["标定角点检测引擎\n(OpenCV findChessboardCorners / 多线程)"]
            OVERLAY["图像标记与水印叠加"]
        end

        subgraph StreamModule["3. 硬件编解码与推流子系统"]
            RGA_PROC["RGA 色彩转换\nRGB/BGR -> NV12"]
            MPP_ENC["MPP 硬件 H.264 编码器"]
            ZLM["ZLMediaKit (mk_api)\nRTSP / WebRTC / HTTP-FLV"]
        end

        subgraph StorageModule["4. 异步持久化存储引擎"]
            ASYNC_QUEUE["无锁/双缓冲抓拍队列"]
            FILE_WRITER["异步文件落地 Worker\n(RGB PNG/JPG + 16-bit Depth PNG)"]
        end

        subgraph HttpModule["5. HTTP 控制与 API 子系统"]
            HTTP_SRV["HTTP REST Server (轻量嵌入式 / httplib)"]
        end
    end

    subgraph UpperLayer["上层业务系统"]
        CALIB_PY["标定服务 (calibration_service.py)"]
        WEB_UI["前端 Web 界面 / 标定 UI\n(WebRTC 播放器 / HTTP API)"]
    end

    %% 数据流连接
    CAM -->|USB / UVC| OBSDK
    OBSDK --> MEMPOOL
    MEMPOOL --> ALIGN
    ALIGN -->|Color Frame| CORNER_DET
    ALIGN -->|Color & Depth| ASYNC_QUEUE
    CORNER_DET -->|Corners Data| OVERLAY
    ALIGN -->|Color Frame| OVERLAY
    OVERLAY -->|RGB/BGR| RGA_PROC
    RGA_PROC -->|RGA NV12 DMA-BUF| MPP_ENC
    MPP_ENC -->|H.264 NALU| ZLM

    ASYNC_QUEUE --> FILE_WRITER
    FILE_WRITER -->|Direct File I/O| DISK

    %% HTTP 控制流
    CALIB_PY -->|REST API (抓拍/切模式/调参)| HTTP_SRV
    WEB_UI -->|REST API| HTTP_SRV
    HTTP_SRV -->|配置与抓拍指令| StorageModule
    HTTP_SRV -->|启停与模式控制| ProcessModule
    HTTP_SRV -->|相机参数控制| CaptureModule

    %% 视频流拉取
    ZLM -->|WebRTC / RTSP 视频流| WEB_UI
```

---

## 3. 核心流水线与数据流设计

### 3.1 零拷贝与高效多线程并发流水线

为保证 30fps+ 极速流转且不丢帧，内部采用**四级解耦线程流水线**：

```
[线程1: 驱动采集] ──(Triple Buffer)──> [线程2: 角点检测与叠加] ──(DMA-BUF)──> [线程3: RGA+MPP 编码推流]
         │
         └─────────────(无锁队列)────────────> [线程4: 异步文件落地存储]
```

1. **采集线程 (`CaptureThread`)**：
   - 维持 OrbbecSDK 驱动的 `waitForFrameset()` 高速循环。
   - 提取 Color Frame (RGB888/YUYV) 与 Depth Frame (Z16)。
   - 支持硬件级或驱动级 D2C（Depth-to-Color）对齐，确保彩色与深度像素对齐。
2. **标定角点检测线程 (`CornerDetectThread`)**：
   - **常态模式**：跳过检测，0 开销。
   - **标定模式 (`calibration_mode=true`)**：以指定频次（如 15~30Hz）对彩色图灰度化并执行 `cv::findChessboardCornersSB` 或 `findChessboardCorners` + `cornerSubPix`。
   - 计算结果（检测成功标志、亚像素角点坐标、检测耗时）缓存到原子结构体中，供视频流叠加和 HTTP API 查询。
3. **RGA + MPP 硬件编码推流线程 (`EncoderStreamThread`)**：
   - 取得最新带角点标记的 RGB 图像。
   - 调用 **RGA 硬件** 将 `RGB888` 极速转换为 `NV12` 格式（耗时 $< 0.8\text{ms}$）。
   - 将 NV12 缓冲区送入 **Rockchip MPP 编码器**，输出 H.264 编码数据包（耗时 $< 1.5\text{ms}$）。
   - 调用 `mk_media_input_h264()` 将 NALU 数据包推入 ZLMediaKit 的虚拟媒体源。
4. **异步文件落地存储线程 (`AsyncDiskWriterPool`)**：
   - 当接收到 HTTP 抓拍请求时，以深拷贝或引用计数方式锁定当前最新成对帧。
   - 将抓拍任务封装推入任务队列，由专门的后台 Worker 线程负责编码为 PNG/JPG 并写入 NVMe/eMMC。
   - **绝对不阻塞采集主循环与视频推流**。

---

## 4. RK3588 硬件加速极致优化方案

### 4.1 RGA (Rockchip 2D Graphic Accelerator) 色彩转换加速
- **问题**：MPP 硬件编码器对输入图像要求为 `NV12` (YUV420SP) 格式，且要求内存物理对齐（通常宽度 16/64 字节对齐）。CPU 执行 `cv::cvtColor(RGB2YUV_I420)` 会消耗 15%~25% 单核算力。
- **优化**：
  1. 使用 `librga.so` 的 `im2d` API。
  2. 初始化输入 `rga_buffer_t in_buf = wrapbuffer_virtualaddr(rgb_ptr, width, height, RK_FORMAT_RGB_888)`。
  3. 初始化输出 `rga_buffer_t out_buf = wrapbuffer_virtualaddr(nv12_ptr, width, height, RK_FORMAT_YCbCr_420_SP)`。
  4. 执行 `imcvtcolor(in_buf, out_buf, RK_FORMAT_RGB_888, RK_FORMAT_YCbCr_420_SP)`。
  5. 硬件耗时仅约 **0.5 ~ 0.8 ms**（1280x800 分辨率），CPU 占用几乎为 0。

### 4.2 MPP 硬件 H.264/H.265 编码加速
- **问题**：软件编码（如 x264/libvpx）极度消耗 CPU，在 ARM 架构上推流 1080P/720P 易掉帧。
- **优化**：
  1. 初始化 MPP 编码器上下文：`mpp_create(&mpp_ctx, &mpp_api)`，类型为 `MPP_CTX_ENC`，格式为 `MPP_VIDEO_CodingAVC` (H.264)。
  2. 配置编码参数：
     - 分辨率（如 1280x800 / 1920x1080）及水平对齐跨度 `hor_stride`。
     - 速率控制：CBR 恒定码率（默认 2000 kbps），GOP 大小设为 30（1 秒 1 个 I 帧，保证前端拉流秒开）。
     - 编码预设：极低延迟模式（`MPP_ENC_RC_INTRA_REFRESH` 或关闭 B 帧，`b_frame = 0`）。
  3. 将 RGA 生成的 NV12 缓冲区封装为 `MppFrame` 送入编码器，取出 `MppPacket`。
  4. 硬件编码耗时 **1.0 ~ 1.5 ms**。

### 4.3 内存对齐与内存池 (Memory Pool)
- 预先分配固定数量（如 8 个）连续对齐内存块（满足 MPP 与 RGA 跨度要求）。
- 环形复用，杜绝在采集推流主链路中的 `malloc` / `free` 引起的内存碎片与抖动。

---

## 5. ZLMediaKit 推流与媒体分发设计

### 5.1 虚拟媒体流与发布路径
服务初始化时调用 `mk_api` 启动轻量 ZLM 引擎并创建虚拟推流节点：
- **VHost**: `__defaultVhost__`
- **App**: `live`
- **Stream ID**: `orbbec_color`
- **媒体源协议支持**：
  - **RTSP**: `rtsp://<IP>:554/live/orbbec_color`（延迟约 100~200ms）
  - **WebRTC**: `http://<IP>:80/index/api/webrtc?app=live&stream=orbbec_color&type=play`（延迟约 50~100ms，适合 Web 浏览器端实时交互）
  - **HTTP-FLV**: `http://<IP>:80/live/orbbec_color.live.flv`

### 5.2 推流调用时序
```c
// 伪代码流程
mk_media media = mk_media_create("__defaultVhost__", "live", "orbbec_color", 0, 0, 0);
codec_args v_args = {0};
v_args.codec_id = MKCodecH264;
mk_track v_track = mk_track_create(MKCodecH264, &v_args);
mk_media_init_track(media, v_track);
mk_media_init_complete(media);

// 每产生一帧 MPP 编码的 NALU:
mk_media_input_h264(media, nalu_data, nalu_len, pts, dts);
```

---

## 6. HTTP REST API 详细设计

服务默认监听 `0.0.0.0:18080`（端口可配置）。

### 6.1 接口概览

| 请求路径 | 方法 | 功能说明 | 适用场景 |
| :--- | :--- | :--- | :--- |
| `/api/v1/camera/status` | `GET` | 查询相机状态与健康信息 | 心跳与监控 |
| `/api/v1/camera/start` | `POST` | 启动相机并开始推流 | 初始化/重连 |
| `/api/v1/camera/stop` | `POST` | 停止相机推流 | 待机节能 |
| `/api/v1/camera/config` | `GET/POST` | 获取/设置分辨率、曝光、增益等 | 相机调优 |
| `/api/v1/camera/intrinsics`| `GET` | 获取当前相机内参与畸变参数 | 标定/三维重建 |
| `/api/v1/camera/calibration_mode` | `POST` | 开启/关闭标定模式与角点绘制 | 标定流程 |
| `/api/v1/camera/corners` | `GET` | 获取最新检测到的标定角点 | 标定采点验证 |
| `/api/v1/camera/save_frame` | `POST` | **核心抓拍接口**：保存彩色+深度数据到指定目录 | 标定采集/作业留存 |
| `/api/v1/stream/info` | `GET` | 查询实时视频流播放地址列表 | 前端接入 |
| `/api/v1/camera/follow` | `POST` | 使能/关闭工位跟随（切换取流档位，默认关闭） | 实时跟随 |
| `/api/v1/camera/follow/teach` | `POST` | 以当前画面冻结一版参考基准（可选落盘） | 示教/换工件 |
| `/api/v1/camera/follow/status` | `GET` | 读取跟随快照：位姿、增量、σ、丢帧与判据状态 | 仿真臂镜像/排障 |

---

### 6.2 接口详细规范

#### 1. 查询相机状态 (`GET /api/v1/camera/status`)
- **响应示例**：
```json
{
  "code": 0,
  "msg": "success",
  "data": {
    "online": true,
    "streaming": true,
    "camera_model": "Orbbec Gemini 335L",
    "serial_number": "AY2B...",
    "firmware_version": "1.8.10",
    "color_fps": 29.8,
    "depth_fps": 29.8,
    "encoder": "RK_MPP_H264",
    "temperature_c": 38.5,
    "calibration_mode": true
  }
}
```

#### 2. 相机内参查询 (`GET /api/v1/camera/intrinsics`)
- **响应示例**：
```json
{
  "code": 0,
  "msg": "success",
  "data": {
    "camera_model": "orbbec",
    "width": 1280,
    "height": 800,
    "intrinsic_matrix": [
      [611.68, 0.0, 643.42],
      [0.0, 611.69, 405.15],
      [0.0, 0.0, 1.0]
    ],
    "distortion_coeffs": [-0.032, 0.034, 0.0003, 0.0003, -0.011],
    "distortion_model": "plumb_bob",
    "depth_scale": 1.0
  }
}
```

#### 3. 标定模式与角点绘制配置 (`POST /api/v1/camera/calibration_mode`)
- **请求参数**：
```json
{
  "enabled": true,
  "board_type": "chessboard",
  "cols": 9,
  "rows": 12,
  "square_size_mm": 15.0,
  "draw_corners": true
}
```
- **响应示例**：
```json
{
  "code": 0,
  "msg": "Calibration mode enabled",
  "data": {
    "enabled": true,
    "pattern_size": [8, 11]
  }
}
```

#### 4. 获取最新检测到的标定角点 (`GET /api/v1/camera/corners`)
- **响应示例**：
```json
{
  "code": 0,
  "msg": "success",
  "data": {
    "found": true,
    "timestamp_ms": 1724508900123,
    "pattern_size": [8, 11],
    "corners_count": 88,
    "corners": [
      [124.5, 230.1],
      [145.2, 230.3],
      ...
    ]
  }
}
```

#### 5. 保存帧数据（核心抓拍接口） (`POST /api/v1/camera/save_frame`)
- **路径解析规则**：
  - 默认存储根目录基准为当前项目目录下的 `data/`。
  - 统一支持项目相对路径（例如 `"data/calib/calib_20260824_220000"` 或 `"calib/calib_20260824_220000"`），不使用 `/home/...` 等硬编码绝对路径。
  - C++ 服务在写入时会自动创建不存在的父目录（`std::filesystem::create_directories`）。

- **请求参数**：
```json
{
  "save_dir": "data/calib/calib_20260824_220000",  // 相对项目 data 目录的路径
  "prefix": "sample_001",
  "save_color": true,
  "save_depth": true,
  "color_format": "png",       // "png" 或 "jpg"
  "depth_format": "png_16bit",  // "png_16bit" (16位深度图), "raw" (原始二进制), "npy"
  "save_pointcloud": false,    // 是否生成并保存 PCD/PLY 点云
  "metadata": {                // 附加业务元数据（可选，自动写入对应 yaml/json 文件）
    "robot_pose": {
      "x": 120.5, "y": -45.2, "z": 300.0,
      "rx": 0.05, "ry": -0.12, "rz": 1.57
    }
  }
}
```
- **执行行为**：
  1. C++ 服务瞬间锁定当前时刻严格硬件时间戳对齐的彩色帧与深度帧。
  2. 异步在目标目录生成：
     - `sample_001_color.png` (8-bit BGR/RGB)
     - `sample_001_depth.png` (16-bit 无损灰度深度图，单位毫米 mm)
     - `sample_001_info.yaml` (包含内参、时间戳、抓拍时机械臂位姿等元数据)
- **响应示例**：
```json
{
  "code": 0,
  "msg": "Frame saved successfully",
  "data": {
    "frame_id": 15892,
    "timestamp_ms": 1724508920450,
    "color_file": "data/calib/calib_20260824_220000/sample_001_color.png",
    "depth_file": "data/calib/calib_20260824_220000/sample_001_depth.png",
    "info_file": "data/calib/calib_20260824_220000/sample_001_info.yaml",
    "corners_found": true
  }
}
```

#### 6. 视频推流地址查询 (`GET /api/v1/stream/info`)
- **响应示例**：
```json
{
  "code": 0,
  "msg": "success",
  "data": {
    "stream_id": "orbbec_color",
    "webrtc_url": "http://127.0.0.1:18080/webrtc/live/orbbec_color",
    "rtsp_url": "rtsp://127.0.0.1:554/live/orbbec_color",
    "http_flv_url": "http://127.0.0.1:18080/live/orbbec_color.live.flv",
    "resolution": "1280x800",
    "fps": 30
  }
}
```

---

#### 7. 使能/关闭工位跟随 (`POST /api/v1/camera/follow`)
- **请求体**：`{"enabled": true}`（`enabled` 必填且必须是布尔，否则 400）
- **副作用**：使能会**重启取流 pipeline** —— 切到 `follow.camera` 档位（640x480@15 + 硬件 D2C）
  并重新取内参；关闭时退回 `hardware.camera` 档位。百毫秒级，接口同步返回结果。
- **响应**：`code=0` 表示档位已经切换，`data` 为切换后的快照；起不来时 `code=-1`、HTTP **503**，
  `msg` 是原话（例如档位被标定模式占用、follow 配置读坏了）。

```json
{"code": 0, "msg": "follow enabled",
 "data": {"enabled": true, "status": "no_map", "taught": false, "align": "hardware",
          "capture_width": 640, "capture_height": 480}}
```

#### 8. 示教参考基准 (`POST /api/v1/camera/follow/teach`)
- **请求体**：`{"save_map": false}`（可省略，默认只在内存里建基准）
- 收 `follow.teach.frames` 帧做时间域深度均值，冻结成新的参考地图并**原子换给**工作线程
  （正在配准的那一帧不会被抽走）。`save_map: true` 时同时落盘到 `follow.teach.map_path`。
- 失败同样回 503 + `msg`，且**不会**留下半张地图：判据与运行期取帧共用同一条 `frameUsable`。

#### 9. 跟随快照 (`GET /api/v1/camera/follow/status`)
只读一份 latest 快照——**绝不在 HTTP 线程里跑 GICP**（一帧几十毫秒，会把 REST 卡成串行队列）。
`/api/v1/camera/status` 里只有一个 `follow_profile` 布尔（当前 pipeline 跑的是 `follow.camera` 还是
`hardware.camera` 档位）；跟随的实时数字只有这一条路能拿到，app 后端按 `follow.arm.poll_hz` 轮询它。

```json
{
  "code": 0, "msg": "success",
  "data": {
    "enabled": true, "connected": true, "taught": true, "has_pose": true,
    "status": "ok", "estimator": "gicp", "reason": "",
    "pose_mm": [812.4, -37.1, 1455.8], "pose_rpy_deg": [1.2, -0.4, 0.7],
    "norm_t_mm": 3.9, "norm_r_deg": 1.5, "holding_last_pose": false,
    "delta_r": [[0.999, -0.012, 0.004], [0.012, 0.999, -0.008], [-0.004, 0.008, 0.999]],
    "delta_t_m": [0.0031, -0.0008, 0.0012],
    "sigma_t_mm": [0.17, 0.19, 0.22], "sigma_r_deg": [0.03, 0.02, null],
    "gicp_inliers": 412, "inlier_ratio": 0.71, "gicp_cost": 0.00042, "cloud_points": 580,
    "compute_ms": 28.4, "fps": 14.8, "frames": 1204, "dropped": 3, "rejected": 0,
    "smooth_used": 5,
    "map_hash": 1234567890, "map_voxels": 3021, "map_path": ".../follow/out/reference.frmap",
    "align": "hardware", "capture_width": 640, "capture_height": 480,
    "teach_capture_width": 640, "teach_capture_height": 480, "snapshot_ts_ms": 1724990000000
  }
}
```
- `status` 词表 = `follow::to_string(Status)` 再加 worker 独有的 `disabled` / `no_map` / `no_frame`。
- `status != ok` 时 `pose_*` 是**上一个可信值**（`holding_last_pose=true`），不是本帧读数。
- `sigma_*` 里的非有限值序列化成 `null`，不能变成非法 JSON 字面量。
- `capture_*` 与 `teach_capture_*` 不一致时修正量仍然算得出，但点云采样密度差一截、门会变严，
  所以必须报出来，而不是假装"米制空间与分辨率无关"。

### 6.3 follow 作为库跑在本进程里的约束

`follow_worker.hpp/.cpp` 是 follow 与相机服务之间那道唯一的墙。三条不可让的设计约束，都是
"这台板子上只有一条取流路径"逼出来的：

1. **不再 new 一个 `follow::OrbbecCapture`**。本进程启动时就拿到了 `.orbbec.lock`（路径来自
   `follow.camera.lock_path`，全项目唯一定义处），正是为了不让第二路 `ob::Pipeline` 存在；
   因此 follow 消费的是 `CameraDriver` **已经在交付**的那一份对齐帧。
2. **工作线程用 `getLatestFrame` + `frame_index` 去重，不能用 `waitForNextFrame`**：后者会推进
   驱动里的共享游标，主循环与 worker 互相吃掉对方的帧，现象是"两边都掉帧但谁也看不出谁在读"。
   算不过来时**丢帧并计数**（`dropped`）—— 丢了不是错误，悄悄算了旧帧才是错误。同一毫秒内的两帧
   无法区分（`FrameData` 上是主机时钟，没有厂商设备时间戳），同样按丢帧处理，不造假 `+1ns`。
3. **板载 IMU 陀螺流由 `CameraDriver` 以 ~200Hz 独立启动**（`camera_driver.cpp:351-382`，出厂外参
   `T_cam_gyro` 在同函数 `:328-339` 读取），每帧配准前由 `FollowWorker` 注入跟踪器
   （`follow_worker.cpp:490-496` → `CameraDriver::drainGyroSamples`）。它**只当旋转初值、不当测量**
   （三档初值的第 2 档，见 `odometry.cpp:198-209`）。⚠ 服务里的图像帧时间戳是**主机 `system_clock`**
   （`camera_driver.cpp:586-587`），陀螺样本是**设备时间**（`:364`），两者是否同域尚未实测 —— 不同域
   的后果是这一档静默失效。详见 `follow_system_design.md` §5.5。

与 `follow_pose` 的一致性是这个模块能验收的前提：示教走同一个 `build_reference_map()`
（`follow/teach_core.hpp`）、平滑走同一个 `PoseSmoother`、输出走同一个 `to_dobot()`。所以
"follow_pose 里量对的数"才蕴含"服务推给页面的数是同一套算法算的"，而不是两份实现各自看着对。

`follow` 的配置读坏了（yaml 类型错、`check_config` 报致命）时，服务**照样要起来**：
`FollowWorker::setBlocked(reason)` 记下原因，`setEnabled` 一律拒绝，`/follow/status` 报同一条原话。
否则运维看到的只是"点了没反应"。

> 端到端链路、算法判据门全表、IMU、臂侧 IK、仿真臂联动、配置参考、实测数据、测试清单、
> 排障手册与维护红线：**见同目录 `follow_system_design.md`**。本节只是本服务视角的三条硬约束。

---

## 7. 模块划分与代码组织结构

规划将代码置于 `app/src/core/hardware/camera/orbbec_camera_service/` 目录下：

```
app/src/core/hardware/camera/orbbec_camera_service/
├── CMakeLists.txt                      # 构建配置
├── docs/                               # 文档目录
│   ├── orbbec_camera_service_design.md # 本设计方案
│   └── follow_system_design.md         # follow 工位跟随：完整设计与维护手册
├── include/                            # 核心头文件
│   ├── camera_driver.hpp               # OrbbecSDK v2 封装（设备管理、数据流、D2C）
│   ├── corner_detector.hpp             # 亚像素角点检测与标记叠加模块
│   ├── rga_processor.hpp               # RK3588 RGA 2D 硬件色彩空间转换与缩放
│   ├── mpp_encoder.hpp                 # RK3588 MPP 硬件 H.264 编码器封装
│   ├── zlm_streamer.hpp                # ZLMediaKit 推流封装（RTSP/WebRTC 虚拟源）
│   ├── async_disk_writer.hpp           # 异步文件存储引擎（无锁队列 + 文件落地）
│   ├── http_server.hpp                 # REST API HTTP 服务器
│   ├── follow_worker.hpp               # follow 作为库跑在本进程里的那道墙（见 6.3）
│   ├── config.hpp                      # 统一配置结构（configs/aisprayer_config.yaml 解析）
│   └── types.hpp                       # 通用数据结构定义（FrameData, Pose, Intrinsics）
├── src/                                # 源码实现
│   ├── main.cpp                        # 程序入口与服务主循环
│   ├── camera_driver.cpp
│   ├── corner_detector.cpp
│   ├── rga_processor.cpp
│   ├── mpp_encoder.cpp
│   ├── zlm_streamer.cpp
│   ├── async_disk_writer.cpp
│   ├── http_server.cpp
│   ├── follow_worker.cpp
│   └── query_hw_d2c.cpp
└── run.sh                              # 启动脚本
```

---

## 8. 构建依赖与第三方库集成

本服务依托于已在 `third_party/install` 构建好的高性能 C++ 依赖库：

| 依赖组件 | 提供形式 | 作用 |
| :--- | :--- | :--- |
| **OrbbecSDK v2** | `third_party/install/lib/libOrbbecSDK.so`<br>`third_party/install/include/libobsensor` | 相机原生驱动、数据流采集与硬件内参读取 |
| **ZLMediaKit** | `third_party/install/lib/libmk_api.so`<br>`third_party/install/include/mk_mediakit.h` | 视频流推流、WebRTC / RTSP 媒体服务器 |
| **Rockchip RGA**| `third_party/install/lib/librga.so`<br>`third_party/install/include/rga` | 2D 图像硬件极速色彩转换（RGB $\to$ NV12） |
| **Rockchip MPP**| 系统库 `/usr/lib/aarch64-linux-gnu/librockchip_mpp.so` | 硬件 H.264/H.265 编码 |
| **OpenCV 4.x** | 系统包 (`libopencv-dev`) | 图像角点检测 (`findChessboardCorners`)、PNG 编码与绘制 |
| **cpp-httplib** | 单头文件 (`httplib.h`) | 轻量嵌入式 HTTP REST API 服务器 |
| **nlohmann/json** | 单头文件 (`json.hpp`) | JSON 解析与响应格式化 |
| **yaml-cpp** | 系统包 (`libyaml-cpp-dev`) | 配置文件加载与标定信息保存 |

---

## 9. 上层 Python 服务对接与平滑迁移

### 9.1 接口 1:1 完整映射对照表

本 C++ 服务的 HTTP 接口完全覆盖并增强了现有 `app/src/services/camera_service.py` 的全部方法：

| 现有 Python 方法 (`camera_service.py`) | 对应 C++ HTTP API / 机制 | 参数 / 行为对应说明 |
| :--- | :--- | :--- |
| `start_stream(camera_type="orbbec")` | `POST /api/v1/camera/start` | 启动 OrbbecSDK 驱动采集与 MPP/RGA 推流流水线 |
| `stop_stream()` | `POST /api/v1/camera/stop` | 停止数据流采集并释放 MPP/RGA 硬件资源 |
| `is_streaming()` | `GET /api/v1/camera/status` | 响应中的 `data.streaming` 字段 |
| `get_status()` | `GET /api/v1/camera/status` | 返回 `{"online": bool, "streaming": bool, "color_fps": float, "camera_model": str, ...}` |
| `get_intrinsics()` | `GET /api/v1/camera/intrinsics` | 返回相机硬件内参矩阵 $K$ (3x3) 与畸变系数 $D$ (5x1) |
| `set_calibration_mode(enabled)` | `POST /api/v1/camera/calibration_mode` | 支持传入 `{"enabled": bool, "rows": 12, "cols": 9, "square_size_mm": 15.0}` 动态切换 |
| `get_latest_frame()` (获取彩色图) | 1. 抓拍落地: `POST /api/v1/camera/save_frame`<br>2. 兼容接口: `GET /api/v1/camera/latest_frame.jpg` | 推荐使用 `save_frame` 避免大图传输开销；同时提供抓取单张最新 JPG/PNG 的兼容接口 |
| `get_latest_depth()` (获取深度图) | `POST /api/v1/camera/save_frame` (`save_depth=true`) | 自动在指定目录保存 16-bit 无损灰度深度图 (`.png`) 或原始二进制 (`.raw`) |
| `generate_mjpeg_stream()` | 1. `GET /api/v1/stream/mjpeg` (兼容流)<br>2. `GET /api/v1/stream/info` (获取 WebRTC/RTSP 地址) | 既提供标准 MJPEG 流供 FastAPI 代理，更推荐前端直接使用极低延迟的 WebRTC 流 |
| *(新增)* 获取当前帧角点坐标 | `GET /api/v1/camera/corners` | 返回当前帧检测到的亚像素角点坐标列表，免除 Python 重复计算 |
| *(新增)* 相机参数热调 | `GET/POST /api/v1/camera/config` | 动态查询与配置曝光时间、增益、激光器功率、工作模式等 |

---

### 9.2 Python 零改动平替适配器 (`camera_service.py` 改造方案)

为了让现有业务模块（`main.py`、`apps/calib/api.py`、`apps/interactive/api.py`、`apps/system/api.py`）无需任何重构，可以直接将 `app/src/services/camera_service.py` 改造为轻量 HTTP 客户端单例：

```python
# app/src/services/camera_service.py (重构后的极简代理单例)
import requests
import numpy as np
import cv2
from typing import Optional, Tuple, Dict, Any, Generator

CAMERA_API_BASE = "http://127.0.0.1:18080/api/v1"

class CameraService:
    def __init__(self, base_url: str = CAMERA_API_BASE):
        self._base = base_url

    def start_stream(self, camera_type: str = "orbbec") -> bool:
        resp = requests.post(f"{self._base}/camera/start", json={"camera_type": camera_type}, timeout=3.0)
        return resp.status_code == 200 and resp.json().get("code") == 0

    def stop_stream(self):
        requests.post(f"{self._base}/camera/stop", timeout=2.0)

    def is_streaming(self) -> bool:
        status = self.get_status()
        return status.get("streaming", False)

    def get_status(self) -> Dict[str, Any]:
        try:
            resp = requests.get(f"{self._base}/camera/status", timeout=1.0)
            if resp.status_code == 200:
                data = resp.json().get("data", {})
                return {
                    "online": data.get("online", False),
                    "streaming": data.get("streaming", False),
                    "has_frame": data.get("online", False),
                    "camera_type": data.get("camera_model", "orbbec"),
                    "color_fps": data.get("color_fps", 0.0)
                }
        except Exception:
            pass
        return {"online": False, "streaming": False, "has_frame": False, "camera_type": "orbbec"}

    def get_intrinsics(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        try:
            resp = requests.get(f"{self._base}/camera/intrinsics", timeout=2.0)
            if resp.status_code == 200 and resp.json().get("code") == 0:
                d = resp.json()["data"]
                return np.array(d["intrinsic_matrix"], dtype=np.float64), np.array(d["distortion_coeffs"], dtype=np.float64)
        except Exception:
            pass
        return None, None

    def set_calibration_mode(self, enabled: bool, rows: int = 12, cols: int = 9, square_size_mm: float = 15.0):
        payload = {
            "enabled": enabled,
            "board_type": "chessboard",
            "rows": rows,
            "cols": cols,
            "square_size_mm": square_size_mm,
            "draw_corners": True
        }
        requests.post(f"{self._base}/camera/calibration_mode", json=payload, timeout=2.0)

    def save_frame(self, save_dir: str, prefix: str, metadata: dict = None) -> dict:
        """核心新增接口：命令 C++ 服务将彩色与深度数据直接保存至指定目录"""
        payload = {
            "save_dir": save_dir,
            "prefix": prefix,
            "save_color": True,
            "save_depth": True,
            "color_format": "png",
            "depth_format": "png_16bit",
            "metadata": metadata or {}
        }
        resp = requests.post(f"{self._base}/camera/save_frame", json=payload, timeout=5.0)
        return resp.json().get("data", {})

    def generate_mjpeg_stream(self) -> Generator[bytes, None, None]:
        """兼容原 FastAPI 的 MJPEG 流转发代理"""
        with requests.get(f"{self._base}/stream/mjpeg", stream=True, timeout=5.0) as r:
            for chunk in r.iter_content(chunk_size=4096):
                if chunk:
                    yield chunk

camera_service = CameraService()
```

---

## 10. 预期性能收益对比

| 性能指标 | 原有 Python 实现 (`camera_service.py`) | 本设计方案 (C++ + RK3588 硬件加速) | 预期提升 |
| :--- | :--- | :--- | :--- |
| **推流协议与延迟** | HTTP MJPEG (延迟 250~500ms) | WebRTC / RTSP (延迟 50~100ms) | 延迟降低 **70%~80%** |
| **CPU 总体占用率** | 60% ~ 100% (单核满载，软编+Python开销) | **5% ~ 12%** (硬件 MPP 编码 + RGA) | 算力大幅释放 |
| **推流帧率与稳定性** | 10 ~ 15 fps (易出现卡顿丢帧) | **30 fps 满帧恒定** (无卡顿) | 帧率翻倍 |
| **传输带宽占用** | 15 ~ 30 Mbps (MJPEG 粗糙压缩) | **1.5 ~ 3.5 Mbps** (H.264 硬件编码) | 带宽降低 **85%** |
| **抓拍落盘对取流影响** | 写入磁盘时造成主线程卡顿丢帧 | 异步无锁队列，**0 卡顿、0 丢帧** | 完全解耦 |
| **相机支持与扩展** | 仅支持基础取图 | 完整支持 Orbbec v2，支持参数热调与点云扩展 | 架构更健壮 |

---

## 11. 后续演进路线建议

1. **第一阶段（核心推流与抓拍）**：
   - 实现 `CameraDriver` 封装 OrbbecSDK v2 采集。
   - 实现 `RgaProcessor` + `MppEncoder` + `ZlmStreamer` 跑通 H.264 WebRTC/RTSP 推流。
   - 实现 `HttpServer` 提供基础状态查询与 `/save_frame` 异步落盘。
2. **第二阶段（标定与角点检测优化）**：
   - 集成 `CornerDetector`，在标定模式下检测棋盘格角点并叠加入推流。
   - 调试亚像素角点精度与稳定性。
3. **第三阶段（高级功能与工业级健壮性）**：
   - 增加相机异常断开自动重连（UVC/USB Watchdog）。
   - 增加点云（PCD/PLY）生成与异步保存支持。
   - 上线并迁移现有 Python 标定业务。
