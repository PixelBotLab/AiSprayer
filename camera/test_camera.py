"""
Orbbec 相机简单测试脚本
"""
import cv2
import sys
import os

# 确保能找到项目根目录下的包
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

try:
    from camera.orbbec_driver import OrbbecDriver
    try:
        import pyorbbecsdk2 as sdk
    except ImportError:
        import pyorbbecsdk as sdk
    print(f"SDK 版本: {sdk.__version__ if hasattr(sdk, '__version__') else '未知'}")
except ImportError as e:
    print(f"导入失败: {e}")
    sys.exit(1)

def main():
    cam = OrbbecDriver()
    try:
        print("正在启动相机...")
        cam.start()
        print("相机启动成功！按 'q' 退出预览。")
        
        while True:
            color, depth = cam.get_frame()
            if color is not None:
                cv2.imshow("Color", color)
            if depth is not None:
                # 归一化深度图以便显示
                depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                cv2.imshow("Depth", depth_vis)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"运行出错: {e}")
    finally:
        cam.stop()
        cv2.destroyAllWindows()
        print("相机已关闭。")

if __name__ == "__main__":
    main()
