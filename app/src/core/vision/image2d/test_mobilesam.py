"""本地点选测试：鼠标点图上的物体，用 MobileSAM 出该物体 mask。

操作：
  左键     前景点（可连点多个）
  右键     背景点（可连点多个，用来抠掉多切的区域）
  z / 退格 撤销上一个点
  n / 回车 确认当前物体，开始下一个
  r        清空当前物体的点
  c        清空全部
  s        保存叠加图
  q / ESC  退出
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from mobilesam_session import resolve_device, load_mobilesam, MobileSAMSession

WINDOW_NAME = "MobileSAM click"

MASK_COLOR_BGR = (0, 220, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MobileSAM 鼠标点选出 mask")
    parser.add_argument("--image", type=str, required=True, help="Input image path")
    parser.add_argument("--output", type=str, default="output_mask.jpg", help="Output overlay image path")
    parser.add_argument("--mobile_sam", type=str, default=None, help="MobileSAM weights path (optional)")
    parser.add_argument("--device", type=str, default=None, help="Device to run MobileSAM on ('cuda', 'mps', 'cpu', 'rknn', 'onnx')")
    parser.add_argument("--rknn_encoder", type=str, default=None, help="Path to RKNN encoder model for RK3588 (optional)")
    parser.add_argument(
        "--max_display",
        type=int,
        default=1280,
        help="窗口最长边像素，仅影响显示，点击会映射回原图坐标",
    )
    parser.add_argument("--alpha", type=float, default=0.7)
    return parser.parse_args()





def display_scale(image_hw: tuple[int, int], max_display: int) -> float:
    h, w = image_hw
    longest = max(h, w)
    if longest <= max_display:
        return 1.0
    return max_display / float(longest)


def overlay_masks(
    image_bgr: np.ndarray,
    committed: list[np.ndarray],
    current_mask: np.ndarray | None,
    alpha: float,
) -> np.ndarray:
    out = image_bgr.astype(np.float32)
    color = np.array(MASK_COLOR_BGR, dtype=np.float32)
    for mask in committed:
        out[mask] = out[mask] * (1.0 - alpha) + color * alpha
    if current_mask is not None and current_mask.any():
        out[current_mask] = out[current_mask] * (1.0 - alpha) + color * alpha
    return out.astype(np.uint8)


def draw_points(
    canvas: np.ndarray,
    points: list[tuple[int, int]],
    labels: list[int],
    scale: float,
) -> None:
    for (x, y), label in zip(points, labels):
        px, py = int(round(x * scale)), int(round(y * scale))
        color = (0, 220, 0) if label == 1 else (0, 0, 255)
        mark = "+" if label == 1 else "-"
        cv2.circle(canvas, (px, py), 8, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(canvas, (px, py), 6, color, -1, cv2.LINE_AA)
        cv2.putText(
            canvas,
            mark,
            (px + 8, py - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )


def put_help(canvas: np.ndarray, status: str) -> None:
    lines = [
        status,
        "L +fg (multi)  R -bg (multi)  |  z undo  n next  r reset  c clear  s save  q quit",
    ]
    y = 24
    for line in lines:
        cv2.putText(
            canvas,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        y += 24

class ClickSession:
    def __init__(self, predictor, image_bgr: np.ndarray, alpha: float):
        self.session = MobileSAMSession(predictor)
        self.session.set_image(image_bgr)
        self.image_bgr = image_bgr
        self.alpha = alpha
        self.points: list[tuple[int, int]] = []
        self.labels: list[int] = []
        self.current_mask: np.ndarray | None = None
        self.current_score = 0.0
        self.committed: list[np.ndarray] = []
        self.committed_scores: list[float] = []

    def add_point(self, x: int, y: int, label: int) -> None:
        self.points.append((x, y))
        self.labels.append(label)
        self.refresh_mask()

    def undo(self) -> None:
        if not self.points:
            return
        self.points.pop()
        self.labels.pop()
        if self.points:
            self.refresh_mask()
        else:
            self.current_mask = None
            self.current_score = 0.0

    def reset_current(self) -> None:
        self.points.clear()
        self.labels.clear()
        self.current_mask = None
        self.current_score = 0.0

    def commit(self) -> bool:
        if self.current_mask is None or not self.current_mask.any():
            return False
        self.committed.append(self.current_mask)
        self.committed_scores.append(self.current_score)
        self.reset_current()
        return True

    def clear_all(self) -> None:
        self.committed.clear()
        self.committed_scores.clear()
        self.reset_current()

    def refresh_mask(self) -> None:
        mask, score = self.session.predict(self.points, self.labels)
        if mask is not None:
            self.current_mask = mask
            self.current_score = float(score)
        else:
            self.current_mask = None
            self.current_score = 0.0

    def render(self, scale: float) -> np.ndarray:
        vis = overlay_masks(
            self.image_bgr, self.committed, self.current_mask, self.alpha
        )
        if scale != 1.0:
            h, w = vis.shape[:2]
            vis = cv2.resize(
                vis,
                (int(round(w * scale)), int(round(h * scale))),
                interpolation=cv2.INTER_AREA,
            )
        draw_points(vis, self.points, self.labels, scale)
        n_fg = sum(1 for lab in self.labels if lab == 1)
        n_bg = sum(1 for lab in self.labels if lab == 0)
        n_obj = len(self.committed)
        if self.points:
            status = (
                f"object {n_obj + 1}  +fg={n_fg}  -bg={n_bg}  "
                f"score={self.current_score:.3f}  committed={n_obj}"
            )
        else:
            status = f"click object: L add fg, R add bg  committed={n_obj}"
        put_help(vis, status)
        return vis

    def save(self, output_path: Path) -> None:
        masks = list(self.committed)
        if self.current_mask is not None and self.current_mask.any():
            masks.append(self.current_mask)
        vis = overlay_masks(self.image_bgr, masks, None, self.alpha)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        ok = cv2.imwrite(str(output_path), vis)
        if not ok:
            raise RuntimeError(f"保存失败: {output_path}")

def main() -> None:
    args = parse_args()
    image_path = Path(args.image)
    output_path = Path(args.output)
    device = resolve_device(args.device)
    kwargs = {"device": device}
    if args.rknn_encoder:
        kwargs["rknn_encoder_path"] = args.rknn_encoder

    if args.mobile_sam:
        weight_path = Path(args.mobile_sam)
        if not weight_path.exists():
            raise FileNotFoundError(f"找不到 MobileSAM: {weight_path}")
        predictor = load_mobilesam(checkpoint=str(weight_path), **kwargs)
    else:
        predictor = load_mobilesam(**kwargs)

    image_bgr = cv2.imread(str(image_path))
    if image_bgr is None:
        raise RuntimeError(f"无法读取图片: {image_path}")
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    scale = display_scale(image_bgr.shape[:2], args.max_display)

    print(f"device={device}")
    print(f"input={image_path}  size={image_bgr.shape[1]}x{image_bgr.shape[0]}")
    print(f"display_scale={scale:.3f}")
    print("编码图像中...")

    session = ClickSession(predictor, image_bgr, args.alpha)
    print("编码完成，打开窗口后点物体即可。")

    pending: list[tuple[int, int, int]] = []

    def on_mouse(event: int, x: int, y: int, _flags: int, _userdata) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            pending.append((int(round(x / scale)), int(round(y / scale)), 1))
        elif event == cv2.EVENT_RBUTTONDOWN:
            pending.append((int(round(x / scale)), int(round(y / scale)), 0))

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    cv2.imshow(WINDOW_NAME, session.render(scale))

    while True:
        key = cv2.waitKey(20) & 0xFF
        if pending:
            x, y, label = pending.pop(0)
            h, w = image_bgr.shape[:2]
            x = int(np.clip(x, 0, w - 1))
            y = int(np.clip(y, 0, h - 1))
            kind = "前景+" if label == 1 else "背景-"
            session.add_point(x, y, label)
            n_fg = sum(1 for lab in session.labels if lab == 1)
            n_bg = sum(1 for lab in session.labels if lab == 0)
            print(
                f"{kind} ({x}, {y})  当前 +fg={n_fg} -bg={n_bg}  "
                f"score={session.current_score:.3f}"
            )
            cv2.imshow(WINDOW_NAME, session.render(scale))
            continue

        if key in (27, ord("q")):
            break
        if key in (ord("z"), 8):
            session.undo()
            cv2.imshow(WINDOW_NAME, session.render(scale))
        elif key in (ord("n"), 13):
            if session.commit():
                print(f"已确认物体，当前共 {len(session.committed)} 个")
            cv2.imshow(WINDOW_NAME, session.render(scale))
        elif key == ord("r"):
            session.reset_current()
            cv2.imshow(WINDOW_NAME, session.render(scale))
        elif key == ord("c"):
            session.clear_all()
            cv2.imshow(WINDOW_NAME, session.render(scale))
        elif key == ord("s"):
            session.save(output_path)
            print(f"已保存: {output_path}")

    cv2.destroyAllWindows()
    if session.committed or (
        session.current_mask is not None and session.current_mask.any()
    ):
        session.save(output_path)
        print(f"退出时已保存: {output_path}")


if __name__ == "__main__":
    main()

