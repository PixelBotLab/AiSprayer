// follow 模块里**唯一**允许跨越 SI(mm/deg) 边界的文件。
//
// 内部一律 SI：米、弧度、Eigen::Isometry3d。
// 对外一律 Dobot 控制器表示：毫米 + 度，基座系绝对目标。
//
// 欧拉约定（已在本机对着 app/src/core/hardware/robot/cr5_kinematics.py 与
// cr5_kinematics_cpp/cr5_kinematics.cpp 数值核对，300 组随机关节 FK→tuple→矩阵
// 偏差 9.4e-16，FK→IK 300/300 复现同一位姿）：
//     R = Rz(rz) · Ry(ry) · Rx(rx)      （内旋 ZYX / 外旋 xyz，同一件事）
// 也就是 ServoP(x, y, z, rx, ry, rz) 里那三个角的意义。
//
// 这里**不做**坐标系换算。基座系 ← 相机系 的 T_base_cam 来自标定结果，由
// FollowController 在调用本文件之前乘上去；本文件只管单位和欧拉。
#pragma once

#include <Eigen/Geometry>

namespace follow {

// Dobot 控制器 / ServoP 的位姿参数。字段后缀标明单位，不允许含糊。
struct DobotPose {
  double x_mm = 0.0, y_mm = 0.0, z_mm = 0.0;
  double rx_deg = 0.0, ry_deg = 0.0, rz_deg = 0.0;

  // 送臂前的最后一道闸：任何非有限值都不能变成电机指令。
  bool finite() const;
};

// SI → Dobot。T.linear() 必须是正交阵（调用方保证：GICP/Umeyama 的输出就是）。
// 万向锁（|ry| → 90°）时取 rz = 0，把剩下的自由度并到 rx 上，结果矩阵仍然正确。
DobotPose to_dobot(const Eigen::Isometry3d& T);

// Dobot → SI。与 to_dobot 互逆（在 ±180° 归一化意义下）。
Eigen::Isometry3d from_dobot(const DobotPose& p);

}  // namespace follow
