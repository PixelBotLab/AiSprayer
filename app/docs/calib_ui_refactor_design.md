# 智能喷涂系统 (AiSprayer App) 全局架构与详细设计文档

## 1. 背景与核心目标
原有的 `src/aisprayer` 代码（尤其是基于 PyQt 的 `calib_ui` 及各种测试脚本）存在逻辑与界面强耦合、代码结构分散等问题。
随着系统复杂度的增加，本项目需要演进为一个**独立、统一的 Web 化软件平台**。

本设计的核心目标为：
1. **构建企业级独立应用 (`app`)**：采用前后端分离架构，前端使用 React，后端使用 FastAPI，彻底摆脱 PyQt。底层核心算法（视觉、规划、硬件）统一沉淀至新工程的 `core` 层。
2. **服务化与解耦**：将公共底层能力（相机流、机器人控制、系统配置）提取为全局服务 (Global Services)。
3. **支持未来四大核心业务模块**的平滑演进与集成：
   - **(一) 手眼标定系统 (Calibration)**：引导式操作，图像点到机器人的坐标系矩阵解算。
   - **(二) 2D 视频交互示教 (Interactive Teach)**：在实时相机视频流上手工框选、打点或画线，后端生成路点驱动机械臂执行动作。
   - **(三) 3D 视觉全自动路径规划 (Auto Surface Planner)**：融合结构光/深度相机，基于 3D 点云与表面分析（如 `test_surface_zigzag.py`），全自动生成复杂表面（如悬挂的裤子）的喷涂轨迹并驱动机械臂。
   - **(四) 3D 数字孪生系统 (Digital Twin 3D)**：在 Web 网页中，通过 3D 引擎实时渲染机械臂的位姿动画，并结合生成的点云/模型实现“所见即所得”的三维可视化呈现。

---

## 2. 全局技术栈与目录架构

### 2.1 技术栈选型
* **前端 (Web)**: **React + Vite + TailwindCSS** (参考 `~/dev/aibox-mis/` 风格)。3D 可视化部分采用 **Three.js**（通过 `@react-three/fiber`）渲染机械臂及目标物体（如裤子）。
* **后端 (Python)**: **FastAPI**。通过 RESTful 提供控制 API，通过 WebSocket 实时推送机器人关节位姿、标定进度和点云更新；通过 HTTP MJPEG 推送实时视频流。
* **数据库 (本地存储)**: **SQLite** (搭配 SQLAlchemy 或简单 ORM)，负责持久化系统设置（IP、标定参数）及运行日志。

### 2.2 核心目录架构规划
```text
app/
├── docs/                     # 系统设计及接口文档
├── frontend/                 # React + Vite + TailwindCSS 纯前端工程
└── src/                      # Python 后端源码
    ├── core/                 # 核心算法与硬件底座 (自原 aisprayer/core 平移适配)
    │   ├── hardware/         # 机器人、相机等底层驱动 (如 dobot_driver)
    │   ├── vision/           # 点云重建、2D分割、法向量平滑 (reconstruction 等)
    │   ├── planner/          # Zigzag采样、保角采样、轨迹规划算法
    │   └── config.py         # 核心常量
    ├── db/                   # 数据库层
    │   ├── database.py       # SQLite 连接引擎
    │   └── models.py         # 配置项 (Settings) 及其他持久化表结构
    ├── services/             # 跨模块的【全局基础服务层】
    │   ├── setting_service.py # 负责 SQLite 设置的统一读写
    │   ├── camera_service.py  # 相机生命周期及图像帧分发
    │   ├── robot_service.py   # 机器人通信、点动(Jog)、坐标系切换、高速轮询状态
    │   └── model_3d_service.py# 管理 URDF/STL 模型的加载与传输，支持数字孪生
    ├── apps/                 # 具体的【领域业务应用层】(按模块隔离，解耦业务)
    │   ├── calib/            # (一) 手眼标定应用 (原 calib_ui 重构)
    │   │   ├── calibration_service.py
    │   │   └── api.py
    │   ├── interactive/      # (二) 2D 交互示教应用 (选点/划线)
    │   │   ├── draw_route_service.py
    │   │   └── api.py
    │   ├── auto_planner/     # (三) 3D 自动视觉规划应用 (裤子表面等)
    │   │   ├── surface_zigzag_service.py 
    │   │   └── api.py
    │   └── digital_twin/     # (四) 数字孪生后端 (提供模型文件流、点云快照流等)
    │       └── api.py
    └── main.py               # FastAPI 启动主入口，装载所有 apps 的 Router
```

---

## 3. 数据库设计 (Database Design)

系统采用 **SQLite** 作为本地持久化数据库。

### 3.1 表结构设计

#### 1. `sys_settings` (系统配置表)
用于存储全局软硬件配置，以 Key-Value 形式动态存储。
| 字段名 | 类型 | 说明 |
|---|---|---|
| `key` | VARCHAR(50) | 主键，配置键名 (如 `robot_ip`, `camera_index`, `calib_save_dir`) |
| `value` | VARCHAR(255) | 配置项的值 (存储为 JSON 字符串或纯文本) |
| `description`| VARCHAR(255) | 配置项说明文字 |
| `updated_at` | DATETIME | 最后修改时间 |

#### 2. `calib_records` (标定历史记录表)
存储历次标定解算的结果，便于回溯和切换。
| 字段名 | 类型 | 说明 |
|---|---|---|
| `id` | INTEGER | 主键自增 |
| `timestamp` | DATETIME | 标定完成时间 |
| `matrix_data` | TEXT | 标定外参矩阵 (4x4 浮点数组序列化 JSON) |
| `reproj_error`| FLOAT | 标定重投影误差 |
| `samples_count`| INTEGER | 参与计算的样本数量 |

#### 3. `path_templates` (路径模板表)
用于存储交互示教或视觉自动生成的经典路径模板。
| 字段名 | 类型 | 说明 |
|---|---|---|
| `id` | INTEGER | 主键自增 |
| `name` | VARCHAR(100) | 模板名称 (如 "标准裤子Zigzag路径") |
| `type` | VARCHAR(50) | 路径类型 (`interactive_2d`, `auto_surface_3d`) |
| `path_data` | TEXT | 序列化的三维路点 JSON 数据 (`[{x,y,z,rx,ry,rz}, ...]`) |
| `created_at` | DATETIME | 创建时间 |

---

## 4. 核心类与服务设计 (Class Design)

所有服务类 (`Service`) 控制在 500 行以内，严格遵循单一职责原则。

### 4.1 全局基础服务 (Global Services)

#### `SettingService` (配置服务)
* **职责**：封装对 `sys_settings` 表的操作。
* **核心方法**：
  * `get_value(key: str, default: Any) -> Any`
  * `set_value(key: str, value: Any)`
  * `get_all_settings() -> dict`

#### `RobotService` (机器人服务)
* **职责**：硬件通信层 (`dobot_driver`) 的封装隔离，负责推送 WebSocket 状态。
* **核心方法**：
  * `connect(ip: str, port: int) -> bool`
  * `disconnect()`
  * `get_current_pose() -> list[float]`
  * `move_to_pose(pose: list[float], speed: float)`
  * `jog_step(axis: str, direction: int)`
  * `start_status_polling(callback_ws)`: 开启后台线程，获取关节角度并触发回调。

#### `CameraService` (相机流服务)
* **职责**：相机设备生命周期管理。
* **核心方法**：
  * `start_stream(camera_index: int)`
  * `stop_stream()`
  * `get_latest_frame() -> np.ndarray`
  * `generate_mjpeg_stream() -> Generator`: 为 FastAPI 提供 HTTP Streaming 响应。

### 4.2 领域业务服务 (App Services)

#### `CalibrationService` (手眼标定服务)
* **职责**：整合图像采样与标定矩阵算法解算。
* **核心方法**：
  * `add_sample(image, robot_pose) -> int`
  * `delete_sample(index: int)`
  * `run_calibration() -> dict`: 调用底层 `calib_solver` 解算外参。
  * `save_to_db(matrix, error)`

#### `InteractiveDrawService` (2D 交互示教服务)
* **职责**：处理前端画布传来的 2D 像素坐标，生成三维空间规划。
* **核心方法**：
  * `pixels_to_world(pixels: list[tuple], depth: float, calib_matrix: np.ndarray) -> list[Pose]`

#### `SurfacePlannerService` (3D 自动视觉规划服务)
* **职责**：封装复杂的 3D 视觉处理流。
* **核心方法**：
  * `capture_point_cloud() -> PointCloud`
  * `generate_zigzag_path(pcd: PointCloud, width: float, spacing: float) -> list[Pose]`: 调用底层 `test_surface_zigzag.py` 算法。
  * `smooth_normals(path: list[Pose]) -> list[Pose]`

---

## 5. 界面与交互设计 (UI/Interface Design)

前端使用 **React + TailwindCSS** 开发，布局采用经典的后台管理系统 (MIS) 风格。

### 5.1 全局 Layout 布局
* **顶侧导航栏 (Header)**：Logo、当前激活的应用模块切换 Tabs (如 `手眼标定` | `交互示教` | `3D规划` | `数字孪生`)。
* **右侧全局状态栏 (Status Panel)**：显示 `机器人状态`、`相机状态`、`全局设置齿轮`。
* **主工作区 (Main Content)**：根据顶部 Tab 动态加载内容。

### 5.2 各业务模块界面设计

#### (一) 手眼标定界面 (Calib View)
* **左侧视图**：实时拉取 MJPEG 视频流。
* **右侧控制台**：IP配置及连接按钮；六轴点动 (Jog) 十字键；【抓取样本】列表与管理；【执行标定】按钮及结果回显。

#### (二) 2D 交互示教界面 (Interactive Teach View)
* **左侧视图**：视频流上叠加 `<canvas>`，支持鼠标绘制连续线段/框选。
* **右侧控制台**：工具栏 (画笔、橡皮)、预计轨迹点列表、【测试运行】按钮。

#### (三) 3D 视觉自动路径界面 (Auto Planner View)
* **左侧视图**：Toggle 切换 `2D流` 和 `Three.js 点云视图`。
* **右侧控制台**：Zigzag参数表单 (Spacing、Depth)；【开始3D扫描】；【生成轨迹】并在左侧 3D 视图渲染红色路径线；【下发执行】。

#### (四) 3D 数字孪生呈现 (Digital Twin View)
* **主视图**：全屏 WebGL 画布 (React-Three-Fiber)。
* **渲染内容**：加载 URDF 模型，通过 WebSocket 接收 Joint 数据实现 1:1 镜像动画。叠加扫描的裤子表面模型。

---

## 6. 实施推进计划

1. **第一阶段：基建与标定打通** (当前主要目标)
   - 搭建 `app/` 骨架及 SQLite 配置。
   - 实现全局的 `RobotService` 和 `CameraService`。
   - 完成 `calib` 标定模块重构，以及对应的前端 React 界面，彻底跑通闭环。
2. **第二阶段：2D 交互扩展**
   - 扩展前端图像交互层，在 `apps/interactive` 中加入路径转换逻辑。
3. **第三阶段：3D 算法与可视化**
   - 在前端集成 Three.js。
   - 封装视觉相关逻辑进 `auto_planner` API，打通从扫描到生成 3D 轨迹的闭环。
