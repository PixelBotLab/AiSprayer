import cv2
import numpy as np
import os

def merge_plan_images(data_dir, sub_dirs, output_path):
    """
    将多个目录下的 plan.png 合并成一张大图。
    """
    images = []
    labels = []
    
    for sub in sub_dirs:
        img_path = os.path.join(data_dir, sub, "plan.png")
        if os.path.exists(img_path):
            img = cv2.imread(img_path)
            # 在图像底部增加一个空白区域写字
            h, w = img.shape[:2]
            label_area = np.zeros((40, w, 3), dtype=np.uint8)
            cv2.putText(label_area, f"Angle: {sub}", (w // 2 - 50, 25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            combined = np.vstack([img, label_area])
            images.append(combined)
            labels.append(sub)

    if not images:
        print("[-] Merge: 未找到任何 plan.png 图像")
        return

    # 布局：2x2
    # 如果只有 4 张，我们做成 2x2
    if len(images) == 4:
        top = np.hstack([images[0], images[1]])
        bottom = np.hstack([images[2], images[3]])
        final = np.vstack([top, bottom])
    else:
        # 否则直接横向排列
        final = np.hstack(images)

    cv2.imwrite(output_path, final)
    print(f"[+] 合并完成: {output_path}")

if __name__ == "__main__":
    data_root = "vision/data"
    angles = ["0", "90", "180", "270"]
    out_file = os.path.join(data_root, "plan_all.png")
    merge_plan_images(data_root, angles, out_file)
