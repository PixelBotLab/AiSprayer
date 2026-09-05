# AiSprayer Docker 容器化部署指南 (RK3588 / x86_64)

本文档说明如何使用 Docker 将 AiSprayer 完整项目（包含 C++ 硬件加速相机服务、流媒体服务、NPU/ONNX 视觉模型以及前端管理界面）打包为开箱即用的 Docker 镜像，并在其他 RK3588 板子上快速部署。

---

## 一、关键特性

1. **板载硬件加速全透传 (RK3588)**：
   - **RKNPU**: 支持 3 核心 NPU 高速运行 MobileSAM、SuperPoint、Wissight 等 `.rknn` 模型。
   - **Rockchip MPP**: 硬件 H.264 视频编码（1280x800 @ 30fps，低 CPU 占用）。
   - **Rockchip RGA**: 2D 硬件图像缩放、色彩转换与合成加速。
   - **Orbbec 3D 相机**: USB 设备总线透传与热插拔自适应。
2. **Zero-PyTorch 极限瘦身**：
   - 全面改用 **RKNN + ONNX Runtime**，生产镜像内**不包含** PyTorch、TorchVision、Timm。
   - 镜像体积由原先的 **~4.5 GB 缩减至 ~1.2 GB**（降低 73% 存储占用）。
   - 极大加快了在 RK3588 开发板上的镜像传输与解压速度。
3. **数据与配置解耦持久化**：
   - 自动挂载外部 `configs/`、`data/`（点云数据、标定结果、模板组）及 `app/logs/`。
   - 修改配置或查看日志无需重新打包镜像。

---

## 二、快速使用

### 1. 构建 Docker 镜像

在 AiSprayer 项目根目录下执行：

```bash
# 自动识别当前机器架构进行构建 (在 RK3588 上默认构建 aisprayer:rk3588-latest)
bash docker/build.sh

# 显式指定构建 RK3588 或 x86 镜像
bash docker/build.sh --platform rk3588
bash docker/build.sh --platform x86_64

# 自定义镜像 tag
bash docker/build.sh -t aisprayer:v1.0
```

> **提示**：构建前脚本会自动检查 `models/` 下的 `.rknn` 与 `.onnx` 模型权重。如果存在未拉取的 Git LFS 指针文件，会友好告警提示。

---

### 2. 运行容器 (以 RK3588 开发板为例)

使用随附的 `docker/run.sh` 脚本可一键完成设备节点挂载并启动全套服务：

```bash
# 后台启动全套服务（推荐）
bash docker/run.sh

# 停止并删除运行中的容器
bash docker/run.sh --stop

# 重启容器
bash docker/run.sh --restart

# 进入容器内部进行交互式调试排查
bash docker/run.sh -i
```

启动成功后，可在浏览器或客户端访问：
- **Web 前端界面**: `http://<RK3588_IP>:5173`
- **FastAPI 核心 API**: `http://<RK3588_IP>:8000` (Swagger 文档: `http://<RK3588_IP>:8000/docs`)
- **C++ 相机微服务 REST API**: `http://<RK3588_IP>:18080`
- **RTSP 实时视频流**: `rtsp://<RK3588_IP>:8554/live/orbbec_color`
- **HTTP-FLV 实时视频流**: `http://<RK3588_IP>:8008/live/orbbec_color.flv`

---

## 三、部署到其他 RK3588 开发板

将制作好的 Docker 镜像导出并迁移至目标 RK3588 板子：

### 1. 导出镜像（在编译板或开发机上）
```bash
docker save aisprayer:rk3588-latest | gzip > aisprayer_rk3588_latest.tar.gz
```

### 2. 传输到目标板
```bash
scp aisprayer_rk3588_latest.tar.gz user@target-rk3588-ip:/home/user/
# 同时复制项目中的 configs/ 与 docker/ 目录
scp -r configs docker user@target-rk3588-ip:/home/user/aisprayer/
```

### 3. 导入并启动（在目标 RK3588 板上）
```bash
# 导入镜像
docker load -i aisprayer_rk3588_latest.tar.gz

# 启动容器
cd /home/user/aisprayer
bash docker/run.sh -t aisprayer:rk3588-latest
```

---

## 四、手动 `docker run` 参数参考

如果不使用 `docker/run.sh` 脚本，可使用标准命令直接启动：

```bash
docker run -d \
    --name aisprayer \
    --restart unless-stopped \
    --privileged \
    --net=host \
    --device /dev/mpp_service:/dev/mpp_service \
    --device /dev/rga:/dev/rga \
    --device /dev/dri:/dev/dri \
    -v /dev/bus/usb:/dev/bus/usb \
    -v $(pwd)/data:/app/data \
    -v $(pwd)/configs:/app/configs \
    -v $(pwd)/app/logs:/app/app/logs \
    aisprayer:rk3588-latest
```
