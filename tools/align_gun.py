import trimesh
import numpy as np
import os

def align_gun(face_index=0):
    base_dir = "../app/urdf/meshes/dobot_gazebo_sim/"
    filepath = os.path.join(base_dir, "my_tools.stl")
    
    print(f"Loading {filepath}...")
    mesh = trimesh.load_mesh(filepath)
    
    # 获取所有的共面网格（即平面区域）
    facets = mesh.facets
    facets_area = mesh.facets_area
    sorted_idx = np.argsort(facets_area)[::-1]
    
    print("\n--- Top 10 平面区域（可能是法兰安装面）---")
    for i in range(min(10, len(sorted_idx))):
        idx = sorted_idx[i]
        normal = mesh.face_normals[facets[idx][0]]
        area = facets_area[idx]
        center = mesh.triangles[facets[idx]].mean(axis=(0,1))
        print(f"Region {i}: Area = {area:.2f} | Normal = [{normal[0]:.2f}, {normal[1]:.2f}, {normal[2]:.2f}] | Center = [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}]")
    
    print(f"\n自动选择 Region {face_index} 作为安装圆饼面进行对齐...")
    chosen_idx = sorted_idx[face_index]
    target_normal = mesh.face_normals[facets[chosen_idx][0]]
    target_center = mesh.triangles[facets[chosen_idx]].mean(axis=(0,1))
    
    # 计算旋转矩阵：把安装面的法线对齐到 -Z 轴（这样喷枪就会朝着 +Z 轴方向伸出，符合机械臂法兰坐标系）
    # 如果装反了，可以把 [0, 0, -1] 改成 [0, 0, 1]
    rotation_matrix = trimesh.geometry.align_vectors(target_normal, [0, 0, -1])
    mesh.apply_transform(rotation_matrix)
    
    # 旋转后，中心点的位置也变了，我们需要把它重新平移到原点 (0,0,0)
    # 这样圆饼的中心就会完美地贴在法兰原点上
    new_center = trimesh.transformations.translation_matrix(-np.dot(rotation_matrix[:3,:3], target_center))
    mesh.apply_transform(new_center)
    mesh.export(filepath)
    
    # 自动分离多材质组件：喷枪枪身/相机/夹爪本体（黑色）、支架（铁银色）、夹爪手指（哑光银）、喷枪枪嘴（金色）
    bodies = mesh.split()
    if len(bodies) >= 2:
        black_bodies = []
        silver_bodies = []
        fingers_bodies = []
        nozzle_mesh = None

        for b in bodies:
            ext = b.extents
            # 1. 相机: ~125mm 宽长条形
            if max(ext) > 120 and min(ext) > 25:
                black_bodies.append(b)
            # 2. 相机支架: ~120mm 宽薄板支架
            elif max(ext) > 115 and min(ext) < 25:
                silver_bodies.append(b)
            # 3. 喷枪: 圆柱体 -> 切分为枪身(黑) 与 枪嘴(金)
            elif abs(ext[0] - ext[1]) < 2.0 and ext[2] > 60:
                fc = b.triangles.mean(axis=1)
                # 枪嘴尖端位于最前端 (Z 轴靠前部分)
                tip_z_threshold = b.bounds[1][2] - 10.0
                noz_mask = fc[:, 2] >= tip_z_threshold if (b.bounds[1][2] - b.bounds[0][2]) > 20.0 else np.zeros(len(fc), dtype=bool)
                if noz_mask.any() and (~noz_mask).any():
                    noz = trimesh.Trimesh(vertices=b.vertices, faces=b.faces[noz_mask], process=True)
                    gun_b = trimesh.Trimesh(vertices=b.vertices, faces=b.faces[~noz_mask], process=True)
                    nozzle_mesh = noz
                    black_bodies.append(gun_b)
                else:
                    black_bodies.append(b)
            # 4. 主支架: 法兰安装圆盘 (厚度 <= 6mm) 或主框体 (三角面数 > 2000)
            elif ext[2] < 6.0 or len(b.faces) > 2000:
                silver_bodies.append(b)
            # 5. 夹爪手指与滑块夹具 (长指 dz ~ 100mm 或 薄块 10mm)
            elif max(ext) > 95 or (min(ext) <= 12.0 and max(ext) <= 50.0):
                fingers_bodies.append(b)
            # 6. 夹爪执行器本体
            else:
                black_bodies.append(b)

        if black_bodies and silver_bodies and fingers_bodies:
            black_mesh = trimesh.util.concatenate(black_bodies)
            silver_mesh = trimesh.util.concatenate(silver_bodies)
            fingers_mesh = trimesh.util.concatenate(fingers_bodies)
            black_out = os.path.join(base_dir, "my_tools_black.stl")
            silver_out = os.path.join(base_dir, "my_tools_silver.stl")
            fingers_out = os.path.join(base_dir, "my_tools_fingers.stl")
            black_mesh.export(black_out)
            silver_mesh.export(silver_out)
            fingers_mesh.export(fingers_out)
            if nozzle_mesh:
                noz_out = os.path.join(base_dir, "my_tools_laser_nozzle.stl")
                nozzle_mesh.export(noz_out)
                # 兼容旧命名
                nozzle_mesh.export(os.path.join(base_dir, "my_tools_nozzle.stl"))
            print(f"已同步生成多材质部件: 激光头+相机+夹爪本体(黑色), 支架(铁银色), 夹爪手指(哑光银), 激光头出光嘴(金色)")

    print(f"\n对齐完成！激光头复合工具已经按照 Region {face_index} 对齐到了原点。请在页面 3D 视图中查看。")
    print("（如果发现选错面了，可以把 align_gun.py 里的数字改成 1 或 2 再运行）")

if __name__ == '__main__':
    import sys
    face_index = int(sys.argv[1])
    align_gun(face_index)
