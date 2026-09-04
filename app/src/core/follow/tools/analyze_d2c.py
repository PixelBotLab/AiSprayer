# follow_probe_device --dump 的配套分析：判断 SDK 的 SW D2C 把深度重采样到**哪一张彩色网格**上。
#
# 为什么必须问这个：`CameraIntrinsics` 明确写着"不含畸变系数：输入要求是已去畸变的彩色系"，
# 而设备实测给出 rgb_distortion model=4（k1=-0.032, k2=0.0345, k3=-0.012）且 depth_distortion
# 全零。两者不可能同时成立 —— 要么彩色已整平（那 model=4 就不该再用），要么彩色仍是畸变的
# （那 pinhole 假设在图像边缘差约 8 px，比 2 mm 的门限大一个量级，且合成数据永远测不出来）。
#
# 判据：物体的**几何轮廓**在深度图里和在彩色图里是同一条边。按半径分桶量两者的 u 向偏移：
#   偏移≈0 且无半径趋势 → 深度落在**畸变**彩色网格上（图像就是畸变的，follow 必须自己处理）
#   偏移随半径增长且与 pinhole 预测同形 → 深度落在**理想**彩色网格上（图像被整平过）
import sys
import numpy as np
import cv2


def load(d):
    p = {}
    for line in open(d + "/param.txt"):
        t = line.split()
        if t:
            p[t[0]] = [x for x in t[1:]]
    cw, ch = int(p["color"][0]), int(p["color"][1])
    dw, dh = int(p["depth"][0]), int(p["depth"][1])
    color = np.fromfile(d + "/color_%dx%d.bgr" % (cw, ch), np.uint8).reshape(ch, cw, 3)
    depth = np.fromfile(d + "/depth_%dx%d.y16" % (dw, dh), np.uint16).reshape(dh, dw)
    f = lambda k, i: float(p[k][i])
    rgb = dict(fx=f("rgb_intrinsic", 0), fy=f("rgb_intrinsic", 1),
               cx=f("rgb_intrinsic", 2), cy=f("rgb_intrinsic", 3))
    dist = [float(x) for x in p["rgb_distortion"][1:]]
    return color, depth.astype(np.float64), rgb, dist


def grad_x(m, ok):
    g = np.zeros_like(m)
    g[:, 2:] = np.abs(m[:, 2:] - m[:, :-2])
    both = ok[:, 2:] & ok[:, :-2]
    g[:, 2:] *= both
    return g


def xcorr_shift(a, b, maxs):
    """返回 b 相对 a 的最佳平移（像素），两者都是非负边缘强度剖面。"""
    a = a - a.mean()
    b = b - b.mean()
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-6 or nb < 1e-6:
        return None
    best, bs = 0.0, None
    for s in range(-maxs, maxs + 1):
        if s >= 0:
            u, v = a[s:], b[:len(b) - s]
        else:
            u, v = a[:s], b[-s:]
        if len(u) < 8:
            continue
        c = float(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v) + 1e-9))
        if c > best:
            best, bs = c, s
    return (bs, best) if bs is not None else None


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "app/src/core/follow/out/real"
    color, depth, K, dist = load(d)
    h, w = depth.shape
    ok = (depth > 0) & (depth < 65000)
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY).astype(np.float64)
    cgx = np.abs(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3))
    dgx = grad_x(depth, ok)
    sm = lambda a: cv2.boxFilter(a, -1, (1, 5))
    cgx, dgx = sm(cgx), sm(dgx)

    n = np.hypot((np.arange(w) - K["cx"]) / K["fx"], 0.0)  # 归一化半径（按列）
    print("%s  彩色 %dx%d  深度有效 %.1f%%  rgb fx=%.3f cx=%.3f" %
          (d, color.shape[1], color.shape[0], 100.0 * ok.mean(), K["fx"], K["cx"]))
    print("   r_norm      列(≈px)  预测畸变位移 px   实测Δu(深度−彩色)      样本  平均相关")
    # pinhole 假设下，畸变图像点相对理想图像点偏移多少
    for r in np.arange(0.05, 1.25, 0.15):
        cols = np.where((n >= r) & (n < r + 0.15))[0]
        if len(cols) < 8:
            continue
        offs, cors = [], []
        for y0 in range(20, h - 20, 6):
            for x0 in cols[::4]:
                a, b = x0 - 6, x0 + 40
                if a < 2 or b > w - 2:
                    continue
                win = ok[y0, a:b]
                if win.mean() < 0.8:
                    continue
                r_ = xcorr_shift(dgx[y0, a:b], cgx[y0, a:b], 12)
                if r_:
                    offs.append(r_[0])
                    cors.append(r_[1])
        if len(offs) < 10:
            continue
        offs = np.array(offs, float)
        # 理论值：把归一化坐标 r 过一遍畸变模型，看像素化后偏多少
        x = r
        k1, k2, k3, k4, k5, k6, p1, p2 = (dist + [0] * 8)[:8]
        r2 = x * x
        rad = (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3) / (1 + k4 * r2 + k5 * r2**2 + k6 * r2**3)
        dx = (x * rad - x) * K["fx"] + (2 * p1 * x * x + p2 * (r2 + 2 * x * x)) * K["fx"]
        print("   %.2f-%.2f  %5.0f-%-5.0f      %+7.2f          %+6.2f ±%-5.2f  %5d   %.2f" %
              (r, r + 0.15, r * K["fx"], (r + .15) * K["fx"], dx, offs.mean(), offs.std(),
               len(offs), np.mean(cors)))
    print("\n读法：实测Δu 若在各半径都≈0 → 彩色图是畸变的（深度落在畸变网格）；若 Δu ≈ −预测值 → 彩色已整平。")


main()
