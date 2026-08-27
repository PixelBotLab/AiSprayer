import trimesh
import numpy as np
import os

def align_gun(face_index=0):
    base_dir = "app/urdf/meshes/dobot_gazebo_sim/"
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
    print(f"\n对齐完成！喷枪已经按照 Region {face_index} 对齐到了原点。请重新编译并在 RViz 中查看。")
    print("（如果发现选错面了，可以打开这个脚本把 align_gun(face_index=0) 里的数字改成 1 或 2 再运行）")

if __name__ == '__main__':
    import sys
    face_index = int(sys.argv[1])
    align_gun(face_index)
