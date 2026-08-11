import argparse
import cv2
import sys
import os
import numpy as np

# 将项目根目录加入 sys.path，以便能够正确导入 aisprayer 包 (src 目录)
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../.."))
SRC_DIR = os.path.join(PROJECT_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from aisprayer.core.vision.image2d.segmenter import SegmenterFactory
from aisprayer.core.vision.image2d.zigzag_sampler import JeansZigzagSampler

def main():
    parser = argparse.ArgumentParser(description="Test Zigzag Sampler on an Image")
    default_image = os.path.join(PROJECT_ROOT, "data/runs/0/scan.jpg")
    parser.add_argument("--image", type=str, default=default_image, help="Path to input image")
    parser.add_argument("--segmenter", type=str, default="sam3.1", choices=["yolo_trousers", "sam3.1"], help="Segmenter type")
    parser.add_argument("--model", type=str, default="models/sam3.1_multiplex/sam3.1_multiplex.pt", help="Path to segmentation model")
    parser.add_argument("--row_spacing", type=float, default=20.0, help="Row spacing for zigzag")
    parser.add_argument("--point_spacing", type=float, default=10.0, help="Point spacing for zigzag")
    parser.add_argument("--overlap", type=int, default=0, help="Overlap pixels between split legs (0 to disable overlap)")
    args = parser.parse_args()

    # 解析图片路径 (支持相对当前终端目录的路径或绝对路径)
    image_path = os.path.abspath(args.image)
    if not os.path.exists(image_path):
        print(f"[-] 图片不存在: {image_path}")
        return

    # 1. 读取原图
    print(f"[*] 正在读取图片: {image_path}")
    image = cv2.imread(image_path)
    if image is None:
        print("[-] 图片读取失败！请检查文件是否损坏")
        return

    # 2. 初始化分割模型并获取掩码
    print(f"[*] 正在初始化 {args.segmenter} 分割模型: {args.model}")
    segmenter = SegmenterFactory.create(args.segmenter, model_path=args.model, conf=0.5)
    
    print("[*] 正在执行语义分割...")
    mask = segmenter.get_mask(image)
    
    if mask is None:
        print("[-] 未能从图像中提取到有效掩模。")
        return

    # 3. 对掩码执行之字形采点
    print(f"[*] 开始之字形采点 (行间距={args.row_spacing}, 点间距={args.point_spacing}, 重叠={args.overlap})")
    sampler = JeansZigzagSampler()
    all_legs_points = sampler.sample(mask, args.row_spacing, args.point_spacing, overlap=args.overlap)

    # 4. 可视化绘制
    vis_image = image.copy()
    
    # 半透明叠加掩码背景（绿色）
    mask_overlay = np.zeros_like(image)
    mask_overlay[mask > 0] = [0, 255, 0]
    cv2.addWeighted(mask_overlay, 0.3, vis_image, 1.0, 0, vis_image)

    # 为不同裤腿分配不同颜色（红、蓝、黄、紫）
    colors = [(0, 0, 255), (255, 0, 0), (0, 255, 255), (255, 0, 255)] 

    print(f"[*] 成功切分为 {len(all_legs_points)} 条腿进行采点。")
    for leg_idx, points in enumerate(all_legs_points):
        print(f"  - 第 {leg_idx + 1} 条腿: 共采集 {len(points)} 个点")
        color = colors[leg_idx % len(colors)]
        
        # 绘制该条腿的连续轨迹线和点
        for i in range(len(points)):
            pt1 = (int(round(points[i][0])), int(round(points[i][1])))
            
            # 画点
            cv2.circle(vis_image, pt1, 2, color, -1)
            
            # 画线连接到下一个点
            if i < len(points) - 1:
                pt2 = (int(round(points[i+1][0])), int(round(points[i+1][1])))
                cv2.line(vis_image, pt1, pt2, color, 1)

    print("[*] 正在弹窗显示结果，请在弹出的图片窗口上按任意键关闭。")
    
    # 如果图片太大，为了在屏幕上能看清，适当缩放显示
    h, w = vis_image.shape[:2]
    max_h, max_w = 800, 1200
    scale = min(max_h / h, max_w / w)
    if scale < 1.0:
        vis_image = cv2.resize(vis_image, (int(w * scale), int(h * scale)))

    # 显示结果
    cv2.imshow("Zigzag Sampling Result", vis_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
