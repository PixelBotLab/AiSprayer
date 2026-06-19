# -*- coding: utf-8 -*-
import numpy as np
from scipy.spatial.transform import Rotation as R_tool

def get_robust_depth(depth_map, u, v, max_r=5):
    """
    鲁棒深度像素检索。
    原理：
      红外/双目深度相机在物体边缘、高反光区域或者噪点处，深度图经常会出现“空洞”（深度值为0）。
      如果直接读取 (u, v) 处的深度，极易得到 0 或噪声极大的离群值。
      本函数以 (u, v) 为中心进行“同心环外扩检索”：
      - 首先检查当前像素值是否大于0。
      - 若为0，则以外圈半径 r = 1, 2, ..., max_r 进行正方形环形外扩，收集该同心环内所有非零的有效深度值。
      - 返回收集到的有效深度点集的中位数 (median)，以最大程度抑制脉冲噪点和边缘飞点。
    """
    h, w = depth_map.shape
    z = float(depth_map[v, u])
    if z > 0:
        return z
    # 同心环向外层逐步检索
    for r in range(1, max_r + 1):
        valid = []
        for du in range(-r, r+1):
            for dv in range(-r, r+1):
                # 仅保留外圈正方形边缘的像素，避免重复读取内圈已检索过的像素
                if abs(du) == r or abs(dv) == r:
                    nu, nv = u + du, v + dv
                    if 0 <= nu < w and 0 <= nv < h:
                        val = float(depth_map[nv, nu])
                        if val > 0:
                            valid.append(val)
        if valid:
            # 返回中位数，保证对边缘噪点具有高度鲁棒性
            return float(np.median(valid))
    return 0.0

def compute_local_normal(depth_map, u, v, K):
    """
    估算局部平面的相机系法向量。
    原理与步骤：
      1. 利用十字邻域采样：在像素系下，以 (u, v) 为中心向左、右、上、下偏移 step = 5 像素，得到 4 个采样点。
      2. 通过 get_robust_depth 获取这 4 个采样点对应的鲁棒深度值，并利用相机内参 K 投射回相机系三维坐标。
         公式: X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy
      3. 计算切向量：
         - 水平切向量 v1 = 右侧三维点 - 左侧三维点
         - 垂直切向量 v2 = 下方三维点 - 上方三维点
      4. 切向量做叉乘：
         n = v1 x v2，并进行单位化归一。
      5. 法线方向修正：
         约定法线应指向相机镜头方向，故要求 Z 轴分量小于 0 (n[2] < 0)。若非，则取反。
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    
    step = 5
    pts = {}
    # 定义十字采样的四个方向
    for label, du, dv in [('L', -step, 0), ('R', step, 0), ('U', 0, -step), ('D', 0, step)]:
        nu, nv = u + du, v + dv
        if 0 <= nu < depth_map.shape[1] and 0 <= nv < depth_map.shape[0]:
            z = get_robust_depth(depth_map, nu, nv, max_r=3)
            if z > 0:
                # 反投影到相机物理三维坐标
                x = (nu - cx) * z / fx
                y = (nv - cy) * z / fy
                pts[label] = np.array([x, y, z])

    # 必须保证十字邻域的 4 个方向均采到了有效的 3D 坐标
    if 'L' in pts and 'R' in pts and 'U' in pts and 'D' in pts:
        v1 = pts['R'] - pts['L']
        v2 = pts['D'] - pts['U']
        n = np.cross(v1, v2)
        norm = np.linalg.norm(n)
        if norm > 1e-6:
            n /= norm
            # 保证法向量指向相机方向 (Z < 0)
            if n[2] > 0:
                n = -n
            return n
    return None

def calculate_tool_orientation(n_base, order="ZYX", sign_vector=(1, 1, 1)):
    """
    基于工作表面法向量计算机器人末端姿态欧拉角（法线垂直对齐）。
    原理与步骤：
      喷涂标定时，需要让机器人末端工具喷头垂直于被喷涂的工作表面。也就是让工具坐标系的 Z_tool 轴与平面的负法向量对齐：
         Z_tool = -n_base (法线由平面指向外侧，所以对齐需要取反，指向表面内侧)
      有了 Z_tool 后，由于旋转自由度缺约束（可以绕工具Z轴任意旋转），通常需要构造一组相互垂直的正交基：
      1. 选择一参考辅助向量（如 [0, 1, 0] Y轴），计算 X_tool = Y_ref x Z_tool。
         如果 Z_tool 与 [0, 1, 0] 极度共线（夹角余弦 > 0.98），为了防止叉乘退化为0，切换参考辅助向量为 [1, 0, 0] X轴。
      2. 归一化 X_tool：X_tool /= norm(X_tool)。
      3. 计算 Y_tool：Y_tool = Z_tool x X_tool，此时 X_tool, Y_tool, Z_tool 构成完整的工具旋转正交基矩阵 R_bt = [X_tool, Y_tool, Z_tool]。
      4. 通过 scipy 的 Rotation 模块把 R_bt 按照特定的顺规（如 ZYX）转换回对应的 Euler 角，并结合符号向量（sign_vector）映射输出 A, B, C。
    """
    z_tool = -n_base
    # 避免叉乘共线退化
    if abs(np.dot([0.0, 1.0, 0.0], z_tool)) < 0.98:
        x_tool = np.cross([0.0, 1.0, 0.0], z_tool)
    else:
        x_tool = np.cross([1.0, 0.0, 0.0], z_tool)
    x_tool /= np.linalg.norm(x_tool)
    y_tool = np.cross(z_tool, x_tool)

    # 组装成工具旋转矩阵
    R_bt = np.column_stack([x_tool, y_tool, z_tool])
    # 转换为机器人所需的欧拉角格式
    euler = R_tool.from_matrix(R_bt).as_euler(order, degrees=False)
    # 根据机械臂系统极性修正
    a = euler[0] / sign_vector[0]
    b = euler[1] / sign_vector[1]
    c = euler[2] / sign_vector[2]
    return a, b, c
