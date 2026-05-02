"""
手眼标定 —— 机械臂运动与位姿数据采集脚本

功能：
1. 连接纳博特机械臂
2. 围绕标定板中心生成多组不同位姿（位置 + 姿态变化）
3. 依次移动到各位姿，等待到位后记录位姿数据
4. 将所有位姿数据保存到 JSON 文件，供后续手眼标定使用

使用方法：
  - 自动模式：python handeye_collect.py --ip 192.168.1.13 --auto
  - 手动模式：python handeye_collect.py --ip 192.168.1.13 (每步按回车确认)
"""

import argparse
import json
import math
import time
import logging
from datetime import datetime
from typing import List

from inexbot_driver import (
    InexbotDriver, RobotPose, JointPose,
    COORD_ACS, COORD_MCS, 
    MODE_TEACH, MODE_REMOTE, MODE_RUN
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger(__name__)


def generate_calibration_poses(
    center: RobotPose,
    num_poses: int = 20,
    xy_range: float = 80.0,
    z_range: float = 60.0,
    angle_range: float = 0.35,
) -> List[RobotPose]:
    """
    围绕中心位姿生成一组标定采集位姿。

    策略：在球面上均匀采样不同的位移和旋转偏移，
    确保位姿的多样性（手眼标定精度的关键因素）。

    参数：
        center:       标定中心位姿（机械臂大致正对标定板的位姿）
        num_poses:    生成位姿数量
        xy_range:     XY 平面内最大偏移量 (mm)
        z_range:      Z 方向最大偏移量 (mm)
        angle_range:  每个姿态轴最大旋转偏移量 (弧度), 约 ±20°
    """
    poses = []
    golden_ratio = (1 + math.sqrt(5)) / 2

    for i in range(num_poses):
        t = i / max(num_poses - 1, 1)
        theta = 2 * math.pi * i / golden_ratio
        phi = math.acos(1 - 2 * (i + 0.5) / num_poses)

        dx = xy_range * math.sin(phi) * math.cos(theta) * t
        dy = xy_range * math.sin(phi) * math.sin(theta) * t
        dz = z_range * (math.cos(phi) * 0.5)

        da = angle_range * math.sin(theta) * math.sin(phi)
        db = angle_range * math.cos(theta) * math.sin(phi)
        dc = angle_range * 0.5 * math.cos(phi)

        pose = RobotPose(
            x=center.x + dx,
            y=center.y + dy,
            z=center.z + dz,
            a=center.a + da,
            b=center.b + db,
            c=center.c + dc,
        )
        poses.append(pose)

    return poses


def save_results(poses: List[dict], filepath: str):
    """保存采集结果到 JSON 文件"""
    output = {
        "description": "手眼标定 - 机械臂位姿采集数据",
        "robot": "iNexbot",
        "coordinate_system": "MCS (直角坐标系/世界坐标系)",
        "position_unit": "mm",
        "orientation_unit": "radian",
        "orientation_type": "euler_angle_ABC (A=Rx, B=Ry, C=Rz)",
        "note": "旋转顺序请根据实际验证结果确认 (大概率 ZYX 外旋)",
        "collected_at": datetime.now().isoformat(),
        "num_poses": len(poses),
        "poses": poses,
    }

    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(output, f, ensure_ascii=False, indent=2)

    logger.info(f"已保存 {len(poses)} 组位姿数据到 {filepath}")


def main():
    parser = argparse.ArgumentParser(description="手眼标定 - 机械臂位姿数据采集")
    parser.add_argument("--ip", type=str, default="192.168.2.14",
                        help="机械臂控制器 IP 地址")
    parser.add_argument("--port", type=int, default=6001,
                        help="通信端口号")
    parser.add_argument("--num-poses", type=int, default=20,
                        help="采集位姿数量 (建议 15-25)")
    parser.add_argument("--velocity", type=float, default=15,
                        help="运动速度百分比 (MOVJ: 1-100)")
    parser.add_argument("--auto", action="store_true",
                        help="自动模式，不需要每步手动确认")
    parser.add_argument("--settle-time", type=float, default=1.0,
                        help="到位后额外等待时间(秒)，确保完全静止")
    parser.add_argument("--output", type=str, default=None,
                        help="输出文件路径 (默认: calibration_poses_时间戳.json)")
    parser.add_argument("--xy-range", type=float, default=80.0,
                        help="XY 平面偏移范围 (mm)")
    parser.add_argument("--z-range", type=float, default=60.0,
                        help="Z 方向偏移范围 (mm)")
    parser.add_argument("--angle-range", type=float, default=0.35,
                        help="姿态偏移范围 (弧度, 0.35≈20°)")
    args = parser.parse_args()

    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"calibration_poses_{timestamp}.json"

    # ──────── 第 1 步：连接机械臂 ────────

    robot = InexbotDriver(ip=args.ip, port=args.port)
    if not robot.connect():
        logger.error("无法连接到机械臂，请检查 IP 和网络")
        return

    try:
        print("\n" + "=" * 60)
        print("  手眼标定 - 机械臂位姿数据采集")
        print("=" * 60)
        print(f"\n已连接到机械臂 {args.ip}:{args.port}")
        print(f"计划采集 {args.num_poses} 组位姿\n")

        # ──────── 第 2 步：伺服上电，切换到示教模式 ────────

        # 必须在示教模式下上电，且保持在示教模式 (C2200特性)
        robot.set_mode(MODE_TEACH)
        print("切换到示教模式")
        time.sleep(0.5)
        
        robot.servo_power_on()
        print("伺服上电成功")
        
        robot.set_coord(COORD_MCS)
        print("切换到MCS坐标系成功")
        time.sleep(0.5)
        print("go to home position")
        robot.go_home()
        print("go to home position successfully")
        time.sleep(1)

        # ──────── 第 3 步：记录中心位姿 ────────
        center_pose = robot.get_current_pose()
        center_joints = robot.get_current_joints()
        print(f"\n中心位姿(MCS): {center_pose}")
        print(f"中心关节(ACS): {center_joints}")
        
        # ──────── 第 4 步：生成标定位姿 ────────
        print("\n请先将机械臂手动移动到标定板正上方（或正对面），")
        print("确保相机能清晰看到完整的标定板。")
        input("\n>>> 移动到位后按回车键记录中心位姿...")


        target_poses = generate_calibration_poses(
            center=center_pose,
            num_poses=args.num_poses,
            xy_range=args.xy_range,
            z_range=args.z_range,
            angle_range=args.angle_range,
        )

        # 可达性预检
        reachable_poses = []
        for i, pose in enumerate(target_poses):
            if robot.is_reachable(pose, move_type=0):
                reachable_poses.append(pose)
            else:
                logger.warning(f"位姿 #{i} 不可达，已跳过")

        if len(reachable_poses) < 10:
            print(f"\n警告：只有 {len(reachable_poses)} 个位姿可达（建议至少 15 个）")
            print("请尝试减小偏移范围参数 (--xy-range, --z-range, --angle-range)")
            if len(reachable_poses) == 0:
                return

        target_poses = reachable_poses

        print(f"\n可达位姿: {len(target_poses)} / {args.num_poses}")
        print(f"XY 偏移范围: ±{args.xy_range}mm")
        print(f"Z 偏移范围: ±{args.z_range}mm")
        print(f"姿态偏移范围: ±{math.degrees(args.angle_range):.1f}°")
        print(f"\n运行模式: {'自动' if args.auto else '手动 (每步需按回车确认)'}")

        input("\n>>> 按回车开始采集...")

        # ──────── 第 5 步：逐一移动并采集 ────────

        collected_poses = []
        failed_count = 0

        for i, target in enumerate(target_poses):
            print(f"\n{'─' * 50}")
            print(f"[{i + 1}/{len(target_poses)}] 移动到目标位姿...")
            print(f"  目标: {target}")

            if not args.auto:
                user_input = input(
                    "  >>> 按回车执行移动 (输入 's' 跳过, 'q' 退出): "
                ).strip().lower()
                if user_input == "q":
                    print("用户中止采集")
                    break
                if user_input == "s":
                    print("  已跳过此位姿")
                    continue

            try:
                robot.move_to_pose(target, velocity=args.velocity)
                done = robot.wait_motion_done(timeout=20.0)
                if not done:
                    logger.warning("  运动超时，跳过此位姿")
                    failed_count += 1
                    continue

                time.sleep(args.settle_time)

                actual_pose = robot.get_current_pose()
                actual_joints = robot.get_current_joints()
                print(f"  实际: {actual_pose}")

                pose_record = {
                    "index": len(collected_poses),
                    "target": {
                        "x": target.x, "y": target.y, "z": target.z,
                        "a": target.a, "b": target.b, "c": target.c,
                    },
                    "actual": {
                        "x": actual_pose.x, "y": actual_pose.y,
                        "z": actual_pose.z, "a": actual_pose.a,
                    "joints": actual_joints.to_list(),
                }
                collected_poses.append(pose_record)
                print(f"  ✓ 已记录第 {len(collected_poses)} 组数据")

            except Exception as e:
                logger.error(f"  ✗ 移动失败: {e}")
                failed_count += 1
                if failed_count >= 3:
                    logger.error("连续失败次数过多，终止采集")
                    break

        # ──────── 第 6 步：回到中心位姿 ────────

        print(f"\n{'─' * 50}")
        print("正在回到中心位姿...")
        try:
            robot.move_to_pose(center_pose, velocity=args.velocity)
            robot.wait_motion_done()
        except Exception as e:
            logger.warning(f"回到中心位姿失败: {e}")

        # ──────── 第 7 步：保存结果 ────────

        if collected_poses:
            save_results(collected_poses, args.output)
            print(f"\n{'=' * 60}")
            print(f"  采集完成！")
            print(f"  成功: {len(collected_poses)} 组")
            print(f"  失败: {failed_count} 组")
            print(f"  数据文件: {args.output}")
            print(f"{'=' * 60}")
        else:
            print("\n没有采集到任何数据")

    except KeyboardInterrupt:
        print("\n\n用户中断，正在安全退出...")
    finally:
        try:
            robot.servo_power_off()
        except Exception:
            pass
        robot.disconnect()


if __name__ == "__main__":
    main()
