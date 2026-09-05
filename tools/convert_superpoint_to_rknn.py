#!/usr/bin/env python3
"""
Convert SuperPoint PyTorch Model to Static ONNX and RKNN (RK3588, FP16).

This script:
1. Defines the complete SuperPoint architecture including:
   - Shared CNN Backbone (VGG-style)
   - Detector Head with unflattening to full resolution heatmap
   - MaxPool-based NMS and Top-K keypoint selection ([1, N, 2])
   - Descriptor Head with dense bilinear upsampling and keypoint gathering ([1, N, 256])
2. Loads pretrained weights (superpoint_v1.pth).
3. Exports a clean, static ONNX graph targeting 640x480 resolution (opset 14).
4. Converts the ONNX model to an RK3588 RKNN FP16 model using RKNN-Toolkit2.
5. Matches the exact output format expected by app/src/core/follow/src/frontend.cpp:
   - keypoints: [1, N, 2] (x, y coordinates in pixel space)
   - descriptors: [1, N, 256] (L2-normalized descriptors)

Usage:
  python tools/convert_superpoint_to_rknn.py [--weights models/superpoint_v1.pth] [--k 500] [--height 480] [--width 640]
"""

import argparse
import os
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS_URL = (
    "https://github.com/magicleap/SuperPointPretrainedNetwork/raw/master/superpoint_v1.pth"
)


class SuperPointFull(nn.Module):
    """
    End-to-End SuperPoint with in-graph NMS and Top-K Descriptor Sampling.
    Outputs:
      - keypoints: [1, K, 2] (float32 pixel coordinates [x, y])
      - descriptors: [1, K, 256] (float32 L2-normalized feature vectors)
    """

    def __init__(self, k: int = 500, h: int = 480, w: int = 640, nms_radius: int = 4):
        super().__init__()
        self.k = k
        self.h = h
        self.w = w
        self.nms_radius = nms_radius

        self.relu = nn.ReLU(inplace=True)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
        c1, c2, c3, c4, c5, d1 = 64, 64, 128, 128, 256, 256

        # Shared Encoder (VGG-style)
        self.conv1a = nn.Conv2d(1, c1, kernel_size=3, stride=1, padding=1)
        self.conv1b = nn.Conv2d(c1, c1, kernel_size=3, stride=1, padding=1)
        self.conv2a = nn.Conv2d(c1, c2, kernel_size=3, stride=1, padding=1)
        self.conv2b = nn.Conv2d(c2, c2, kernel_size=3, stride=1, padding=1)
        self.conv3a = nn.Conv2d(c2, c3, kernel_size=3, stride=1, padding=1)
        self.conv3b = nn.Conv2d(c3, c3, kernel_size=3, stride=1, padding=1)
        self.conv4a = nn.Conv2d(c3, c4, kernel_size=3, stride=1, padding=1)
        self.conv4b = nn.Conv2d(c4, c4, kernel_size=3, stride=1, padding=1)

        # Detector Head
        self.convPa = nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
        self.convPb = nn.Conv2d(c5, 65, kernel_size=1, stride=1, padding=0)

        # Descriptor Head
        self.convDa = nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
        self.convDb = nn.Conv2d(c5, d1, kernel_size=1, stride=1, padding=0)

        # Static 4-pixel border mask to eliminate boundary false detections
        border = 4
        mask = torch.ones(1, 1, h, w, dtype=torch.float32)
        mask[:, :, :border, :] = 0.0
        mask[:, :, -border:, :] = 0.0
        mask[:, :, :, :border] = 0.0
        mask[:, :, :, -border:] = 0.0
        self.register_buffer("border_mask", mask)

    def forward(self, x: torch.Tensor):
        # 1. Feature Backbone
        x1 = self.relu(self.conv1a(x))
        x1 = self.relu(self.conv1b(x1))
        p1 = self.pool(x1)

        x2 = self.relu(self.conv2a(p1))
        x2 = self.relu(self.conv2b(x2))
        p2 = self.pool(x2)

        x3 = self.relu(self.conv3a(p2))
        x3 = self.relu(self.conv3b(x3))
        p3 = self.pool(x3)

        x4 = self.relu(self.conv4a(p3))
        feats = self.relu(self.conv4b(x4))  # [1, 128, Hc, Wc]

        # 2. Detector Head
        cPa = self.relu(self.convPa(feats))
        semi = self.convPb(cPa)  # [1, 65, Hc, Wc]
        dense = F.softmax(semi, dim=1)
        nodust = dense[:, :-1, :, :]  # [1, 64, Hc, Wc]

        # Unflatten to full resolution heatmap [1, 1, H, W]
        Hc = self.h // 8
        Wc = self.w // 8
        nodust_p = nodust.permute(0, 2, 3, 1)  # [1, Hc, Wc, 64]
        heatmap_patch = nodust_p.reshape(1, Hc, Wc, 8, 8)
        heatmap_perm = heatmap_patch.permute(0, 1, 3, 2, 4)  # [1, Hc, 8, Wc, 8]
        heatmap = heatmap_perm.reshape(1, 1, self.h, self.w)

        # 3. NMS using MaxPool2d
        kernel = self.nms_radius * 2 + 1
        pooled = F.max_pool2d(heatmap, kernel_size=kernel, stride=1, padding=self.nms_radius)
        is_max = (heatmap == pooled).float()
        scores = heatmap * is_max * self.border_mask

        # Top-K Keypoint Selection
        flat_scores = scores.view(1, -1)
        _, topk_indices = torch.topk(flat_scores, self.k, dim=1)
        topk_y = (topk_indices // self.w).float()
        topk_x = (topk_indices % self.w).float()
        keypoints = torch.stack([topk_x, topk_y], dim=-1)  # [1, K, 2]

        # 4. Descriptor Head
        cDa = self.relu(self.convDa(feats))
        desc_coarse = self.convDb(cDa)  # [1, 256, Hc, Wc]
        desc_coarse = F.normalize(desc_coarse, p=2, dim=1)

        # Dense bilinear upsampling followed by index gather (NPU friendly)
        desc_dense = F.interpolate(
            desc_coarse, size=(self.h, self.w), mode="bilinear", align_corners=False
        )
        desc_flat = desc_dense.view(1, 256, self.h * self.w)

        idx_exp = topk_indices.unsqueeze(1).expand(1, 256, self.k)
        desc_gathered = torch.gather(desc_flat, 2, idx_exp)  # [1, 256, K]
        descriptors = desc_gathered.permute(0, 2, 1)  # [1, K, 256]
        descriptors = F.normalize(descriptors, p=2, dim=-1)

        return keypoints, descriptors


def download_weights(target_path: Path):
    if target_path.exists():
        return
    print(f"[*] Downloading SuperPoint weights to {target_path}...")
    target_path.parent.mkdir(parents=True, exist_ok=True)
    urllib.request.urlretrieve(DEFAULT_WEIGHTS_URL, str(target_path))
    print("[+] Download complete.")


def export_onnx(model: nn.Module, onnx_path: Path, h: int, w: int) -> bool:
    print(f"[*] Exporting SuperPoint to ONNX ({onnx_path})...")
    dummy_input = torch.randn(1, 1, h, w, dtype=torch.float32)

    torch.onnx.export(
        model,
        dummy_input,
        str(onnx_path),
        export_params=True,
        opset_version=14,
        do_constant_folding=True,
        input_names=["image"],
        output_names=["keypoints", "descriptors"],
    )

    import onnx
    import onnxruntime as ort

    onnx_model = onnx.load(str(onnx_path))
    onnx.checker.check_model(onnx_model)

    # Validate with ONNX Runtime
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    test_in = np.random.rand(1, 1, h, w).astype(np.float32)
    outs = session.run(None, {"image": test_in})

    kpts, descs = outs[0], outs[1]
    print(f"    - Validated ONNX Output keypoints:   shape={kpts.shape}, dtype={kpts.dtype}")
    print(f"    - Validated ONNX Output descriptors: shape={descs.shape}, dtype={descs.dtype}")
    print(f"[+] ONNX export succeeded: {onnx_path} ({onnx_path.stat().st_size / (1024*1024):.1f} MB)")
    return True


def convert_to_rknn(onnx_path: Path, rknn_path: Path, target_platform: str = "rk3588") -> bool:
    print(f"[*] Converting ONNX to RKNN FP16 ({rknn_path}) for {target_platform}...")
    try:
        from rknn.api import RKNN
    except ImportError:
        print("[!] Error: rknn-toolkit2 is not installed in the current environment.")
        return False

    rknn = RKNN(verbose=False)
    rknn.config(
        target_platform=target_platform,
        optimization_level=3,
    )

    print(f"[*] Loading ONNX model into RKNN...")
    ret = rknn.load_onnx(model=str(onnx_path))
    if ret != 0:
        print(f"[!] RKNN load_onnx failed with error code: {ret}")
        return False

    print(f"[*] Building RKNN FP16 model (do_quantization=False)...")
    ret = rknn.build(do_quantization=False)
    if ret != 0:
        print(f"[!] RKNN build failed with error code: {ret}")
        return False

    print(f"[*] Exporting RKNN model to {rknn_path}...")
    ret = rknn.export_rknn(export_path=str(rknn_path))
    if ret != 0:
        print(f"[!] RKNN export_rknn failed with error code: {ret}")
        return False

    rknn.release()
    print(f"[+] RKNN model successfully exported: {rknn_path} ({rknn_path.stat().st_size / (1024*1024):.1f} MB)")
    return True


def main():
    parser = argparse.ArgumentParser(description="Convert SuperPoint to ONNX and RKNN (RK3588, FP16)")
    parser.add_argument(
        "--weights",
        type=str,
        default=str(REPO_ROOT / "models" / "superpoint_v1.pth"),
        help="Path to superpoint_v1.pth",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=str(REPO_ROOT / "models"),
        help="Directory to save converted models",
    )
    parser.add_argument("--k", type=int, default=500, help="Number of keypoints to extract (default: 500)")
    parser.add_argument("--height", type=int, default=480, help="Input image height (default: 480)")
    parser.add_argument("--width", type=int, default=640, help="Input image width (default: 640)")
    parser.add_argument("--platform", type=str, default="rk3588", help="Target Rockchip platform (default: rk3588)")

    args = parser.parse_args()

    weights_path = Path(args.weights)
    if not weights_path.exists():
        download_weights(weights_path)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    onnx_file = out_dir / "superpoint.onnx"
    rknn_fp16_file = out_dir / f"superpoint_{args.platform}_fp16.rknn"
    rknn_symlink = out_dir / "superpoint.rknn"

    # 1. Instantiate PyTorch model and load weights
    print(f"[*] Loading PyTorch SuperPoint model (k={args.k}, {args.width}x{args.height})...")
    model = SuperPointFull(k=args.k, h=args.height, w=args.width)
    weights = torch.load(str(weights_path), map_location="cpu")
    model.load_state_dict(weights, strict=False)
    model.eval()

    # 2. Export to ONNX
    ok = export_onnx(model, onnx_file, args.height, args.width)
    if not ok:
        sys.exit(1)

    # 3. Convert to RKNN
    ok = convert_to_rknn(onnx_file, rknn_fp16_file, target_platform=args.platform)
    if not ok:
        sys.exit(1)

    # 4. Create symlink
    if rknn_symlink.is_symlink() or rknn_symlink.exists():
        rknn_symlink.unlink()
    rknn_symlink.symlink_to(rknn_fp16_file.name)
    print(f"[+] Symlink created: {rknn_symlink.name} -> {rknn_fp16_file.name}")

    print("\n" + "=" * 65)
    print("[+] SuperPoint RKNN Conversion Complete!")
    print(f"    - ONNX: {onnx_file} ({onnx_file.stat().st_size / (1024*1024):.1f} MB)")
    print(f"    - RKNN: {rknn_fp16_file} ({rknn_fp16_file.stat().st_size / (1024*1024):.1f} MB)")
    print(f"    - Link: {rknn_symlink}")
    print("=" * 65)


if __name__ == "__main__":
    main()
