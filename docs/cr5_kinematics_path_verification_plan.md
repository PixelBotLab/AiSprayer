# CR5 机械臂运动学移植与离线链式 TCP 路径校验/姿态修正系统方案

## 一、 背景与系统目标

在基于越疆（Dobot）CR5 V3 机器人的喷涂/加工工作流中，上位机通过交互界面手动设计或自动生成工件表面 TCP 喷涂路径（保存于 `scan.manual_paths.yaml`）。

在真实下发 `MoveL` 笛卡尔直线运动指令前，必须在上位机进行**高可靠的离线链式运动学仿真与安全包络分析**：
1. **杜绝急停与报警**：彻底规避因笛卡尔直线穿越奇异点（如腕关节奇异 $q_5 \approx 0^\circ$）、关节瞬时超速（$\Delta q / \Delta t > \omega_{max}$）以及解构型跳跃突变导致的控制器急停。
2. **利用喷涂工艺容差自动修正**：当局部路径遇到不可达或超速时，利用喷枪绕法向自转对称性（TCP Z 轴旋转自由度）及小角度倾角容差（$\le 10^\circ$），在上位机自动探索并平滑修正姿态，输出 100% 可执行的安全 MoveL 轨迹。
3. **闭环交互验证**：在交互设计区提供直观的一键验证、诊断报告、异常高亮与微调应用功能。

---

## 二、 系统架构设计与数学解耦流水线

您总结的流程极其精准！在喷涂作业中，视觉与工艺计算得到的是**枪尖 TCP 位姿（$T_{gun}$）**，而机械臂运动学求解的是**法兰盘末端位姿（$T_{flange}$）**。

因此，整个系统的闭环链路必须包含 **TCP 偏移解耦剥离（$T_{flange} = T_{gun} \cdot T_{tcp}^{-1}$）**：

```
[视觉/工艺生成: 稀疏 Gun-Tip Waypoints]
                  │
                  ▼
[模块一: 笛卡尔空间密集插值 (LERP + SLERP)]
                  │ (生成密集枪嘴位姿 T_gun)
                  ▼
[模块二: TCP 偏移解耦剥离 (T_flange = T_gun * T_tcp_inv)]
                  │ (换算为机械臂法兰盘位姿 T_flange)
                  ▼
[模块三: 解析解 IK 求解 (inverse() ➔ 8 组候选解)]
                  │
                  ▼
[模块四: 链式递推与分支选择 (Nearest Branch Selection)]
                  │
                  ▼
       ┌────────────────────────┐
       │   安全与运动学多维校验   │ ──(限位 / 关节速度 / 奇异度)
       └────────────────────────┘
                  │
         ┌────────┴────────┐
     [通过]             [未通过]
         │                 │
         │                 ▼
         │      [模块五: 容差自愈修正 (Auto-Fix)]
         │                 │ (绕枪轴自旋 ψ ∈ [-π, π] / 倾角微调)
         │                 ▼
         │         [更新 Waypoint 姿态并重新密化校验]
         │                 │
         └────────┬────────┘
                  ▼
[下发原始/修正后的稀疏 Waypoints 给 Dobot MoveL]
```

### 数学变换定义：
1. **$T_{tcp}$ (Flange $\to$ Gun Tip)**：由喷枪夹具机械尺寸决定（例如在 URDF 中为 $X+50\text{mm}, Ry+90^\circ$）。
2. **$T_{flange} = T_{gun} \cdot T_{tcp}^{-1}$**：通过右乘 $T_{tcp}$ 的逆矩阵，将空间中枪尖目标位姿无损转换为法兰盘中心位姿。
3. **$T_{controller} \leftrightarrow T_{urdf}$**：转换为 Dobot 驱动层所需的坐标系定义（Base 旋转 $180^\circ$ 及轴系置换）。


---

## 三、 三步实现计划

### 第一步：完整移植 `cr5_kinematics` 到 Python

**目标目录**：`app/src/core/hardware/robot/`

#### 1. 模块拆分与实现内容
- **`cr5_ur_kin.py`（底层解析几何内核）**：
  - 严格依据 `cr5_ur_kin.cpp` 移植 6-DOF 几何闭式解析算法。
  - 实现 `forward(q)`：输出 $4 \times 4$ 齐次变换矩阵。
  - 实现 `forward_all(q)`：输出全部连杆中间坐标系 $T_1 \sim T_6$。
  - 实现 `inverse(T, q6_des=0)`：解析求解最多 8 组封闭逆解，包含完整的几何分支（肩部左右、肘部上下、腕部翻转）以及奇异点零阈值（`ZERO_THRESH = 1e-8`）截断保护。
  - 实现 `jacobian(q)` 与 `manipulability(q)`：用于微观运动学分析与奇异度定量评估。
- **`cr5_kinematics.py`（高层封装与控制器适配）**：
  - 封装 `CR5Kinematics` 求解器类。
  - **DH 与 URDF 偏置适配**：$q_2, q_4$ 自动转换 $\pm \pi/2$。
  - **Dobot 控制器坐标系无缝互转**：
    - `forward_controller(q_rad) -> (xyz_mm, rpy_deg)`：支持 Dobot 默认的 Base 旋转 $180^\circ$、Tool 轴向置换及 Euler ZYX 欧拉角。
    - `inverse_controller(xyz_mm, rpy_deg) -> list[q_rad]`：直接接收实测 TCP 坐标（mm / deg）并返回标准关节弧度。
  - **关节限位与多圈别名展开**：
    - CR5 标准限位配置（$J_1 \sim J_5: [-180^\circ, 180^\circ], J_6: [-360^\circ, 360^\circ]$）。
    - 自动进行 $\pm 2\pi$ 别名展开，确保不遗漏限位内合法解。
  - **近邻平滑选解（`get_best_ik`）**：
    - 输入目标位姿与当前关节角 $q_{curr}$，计算加权关节角距离，选出连续性最佳的最优解。
- **`test_cr5_kinematics.py`（单元测试与压测）**：
  - 复刻原 C++ `test_cr5_kinematics.cpp` 与 `kinematics_benchmark_test.cpp` 的所有测试用例。
  - 校验与 Dobot 实测控制器位姿的残差（位置残差 $< 0.05\text{ mm}$，姿态残差 $< 0.5^\circ$）。
  - 进行 10,000 次 IK 耗时压测与正确率比对。

---

### 第二步：实现离线链式校验器与容差姿态自动修正器

**新建模块**：`app/src/core/hardware/robot/cr5_path_verifier.py`

#### 1. 笛卡尔空间高密插值器（Dense Cartesian Interpolator）
- 对于每一段路径（从 Waypoint $k$ 到 Waypoint $k+1$）：
  - 按空间步长 $\Delta s \le 1.0\text{ mm}$ 进行离散化。
  - 位置：三维线性插值（Lerp）。
  - 姿态：四元数球面线性插值（Slerp）。
  - 计算各插值点的时间戳 $\Delta t = \Delta s / v_{cartesian}$（默认工艺线速度 $v = 100 \sim 200\text{ mm/s}$）。

#### 2. 链式连续解追踪与动力学诊断（Chain IK Tracker & Diagnostics）
- **连续性追踪**：从轨迹首点初始关节构型（或默认待机姿态）出发，后续各微观插值点以 $q_{t-1}$ 为参考，利用 `get_best_ik` 选取最邻近解 $q_t$。
- **诊断指标**：
  1. **逆解可达性（Reachability）**：是否存在无解点。
  2. **关节超限（Joint Limits Violation）**：是否存在超出软限位点。
  3. **关节角速度与超速（Joint Velocity Over-limit）**：
     $$\dot{q}_i(t) = \frac{|q_i(t) - q_i(t-1)|}{\Delta t} > \omega_{max, i}$$
  4. **奇异点指数（Singularity Proximity）**：
     - 腕部奇异：$|q_5(t)| < 3.0^\circ$。
     - 肘部奇异：$|q_3(t)| < 5.0^\circ$ 或 $|q_3(t) - 180^\circ| < 5.0^\circ$。
     - 可操作度指标（Yoshikawa Manipulability Index）：$w = \sqrt{\det(J J^T)} < \epsilon$。

#### 3. 喷涂工艺容差自动微调与姿态优化（Tolerance-based Auto Adjustment）
- **喷涂工艺几何特性**：圆形/对称喷幅允许喷枪绕 TCP 法向（Z 轴）任意旋转（自转自由度 $\psi \in [-180^\circ, 180^\circ]$），且允许小角度倾角偏移（$\theta \le 5^\circ \sim 10^\circ$）。
- **优化算法**：
  - 当某段轨迹检测到奇异点或关节超速时，在允许的容差锥空间内对该段航点的 TCP 姿态进行网格搜索与连续性平滑优化。
  - 目标函数：最小化最大关节速度 $\max(\dot{q})$，最大化奇异点距离，确保整条路径连续可达。
  - 修正完成后自动更新航点的 `tcp_pose_base`（x, y, z, rx, ry, rz）。

---

### 第三步：后端 API 与前端交互界面集成

#### 1. 后端接口扩展 (`app/src/apps/interactive/api.py` & `path_service.py`)
- **`POST /api/interactive/templates/{template_name}/verify_paths`**：
  - 对当前模板的 `scan.manual_paths.yaml` 进行全量链式校验。
  - 返回校验结果 JSON：
    ```json
    {
      "success": true,
      "summary": {
        "status": "WARNING",  // "PASS" | "WARNING" | "ERROR"
        "total_paths": 2,
        "total_interpolated_points": 3450,
        "max_joint_velocity_deg_s": [45.2, 38.1, 112.5, 65.0, 192.4, 85.0],
        "has_singularity": true,
        "singularity_points_count": 12
      },
      "paths_report": [
        {
          "path_id": 1,
          "status": "WARNING",
          "issues": [
            {
              "type": "SINGULARITY_WRIST",
              "waypoint_index": 3,
              "interpolated_index": 215,
              "detail": "J5 angle 1.2 deg (near wrist singularity)",
              "location_xyz": [691.77, 132.77, -3.37]
            }
          ]
        }
      ],
      "suggested_optimization_available": true
    }
    ```
#### 2. 前端交互设计区界面增强 (`InteractiveOp.tsx`)
- **右侧主操作栏新增同级功能按钮**：
  - 在右侧操作工具栏中（与 `Segment`、`Manual TCP`、`Reconstruct` 同级），新增 **【TCP 路径优化 (Optimize TCP)】** / **【路径校验】** 主按钮。
  - 支持快捷交互：点击直接触发对当前模板路径（`scan.manual_paths.yaml`）的运动学校验与容差修正。
- **视觉反馈与状态展示**：
  - **状态徽章（Badge）**：路径列表中直观显示绿色的 `PASS`、橙色的 `WARN` 或红色的 `ERROR`。
  - **3D / 2D 异常路段高亮**：在 SVG 2D 俯视图和 3D 点云视口中，将发生奇异或超速的插值轨迹段标为**红色/黄色预警线段**。
  - **诊断抽屉/弹窗**：点击可查看 6 关节角速度曲线分布图、最小奇异度距离、各轴速度峰值，并可一键确认保存为 `scan.manual_opt_paths.yaml`。

---

## 四、 实施路线图与验收标准

| 阶段 | 交付内容 | 验收指标 |
| :--- | :--- | :--- |
| **阶段 1** | `cr5_ur_kin.py` & `cr5_kinematics.py` 移植与单测 | 1. 通过 100% Dobot 实测用例，位置残差 $<0.05\text{mm}$，姿态 $<0.5^\circ$<br>2. 10,000 次 IK 求解测试正常运行 |
| **阶段 2** | `cr5_path_verifier.py` 链式插值、超速/奇异诊断与容差优化 | 1. 正确对 `scan.manual_paths.yaml` 进行毫米级插值与连续逆解追踪<br>2. 能精准识别出 $q_5 \to 0$ 及关节突变点，并能通过绕法向微调消除异常 |
| **阶段 3** | 后端 API 与前端 `InteractiveOp.tsx` 按钮/UI 联调 | 1. 用户可在界面一键点击“验证路径”并看到即时诊断结果<br>2. 点击“自动微调”后成功修复轨迹并在 3D/2D 界面更新展示 |
