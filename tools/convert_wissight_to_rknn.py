#!/usr/bin/env python3
"""
Convert Wissight YOLOv8-seg PyTorch Model to Static ONNX and RKNN (RK3588, FP16).

This script:
1. Loads models/wissight.pt via Ultralytics.
2. Exports static ONNX model (640x640, opset 12).
3. Converts the ONNX model to an RK3588 RKNN FP16 model using RKNN-Toolkit2.
4. Outputs:
   - images: [1, 3, 640, 640]
   - output0: [1, 49, 8400] (4 box coords + 13 class probabilities + 32 mask coefficients)
   - output1: [1, 32, 160, 160] (mask prototype tensors)

Usage:
  python tools/convert_wissight_to_rknn.py [--weights models/wissight.pt] [--platform rk3588]
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def export_onnx(weights_path: Path, onnx_path: Path, imgsz: int = 640) -> bool:
    print(f"[*] Loading PyTorch YOLO model from {weights_path}...")
    from ultralytics import YOLO

    model = YOLO(str(weights_path))

    print(f"[*] Exporting to static ONNX ({onnx_path}, imgsz={imgsz})...")
    exported_path = model.export(
        format="onnx",
        imgsz=imgsz,
        opset=12,
        dynamic=False,
    )

    p = Path(exported_path)
    if p != onnx_path and p.exists():
        if onnx_path.exists():
            onnx_path.unlink()
        p.rename(onnx_path)

    print(f"[+] ONNX model exported: {onnx_path} ({onnx_path.stat().st_size / (1024*1024):.1f} MB)")
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
        print(f"[!] RKNN load_onnx failed with code: {ret}")
        return False

    print(f"[*] Building RKNN FP16 model (do_quantization=False)...")
    ret = rknn.build(do_quantization=False)
    if ret != 0:
        print(f"[!] RKNN build failed with code: {ret}")
        return False

    print(f"[*] Exporting RKNN model to {rknn_path}...")
    ret = rknn.export_rknn(export_path=str(rknn_path))
    if ret != 0:
        print(f"[!] RKNN export_rknn failed with code: {ret}")
        return False

    rknn.release()
    print(f"[+] RKNN model successfully exported: {rknn_path} ({rknn_path.stat().st_size / (1024*1024):.1f} MB)")
    return True


def main():
    parser = argparse.ArgumentParser(description="Convert Wissight YOLOv8-seg to ONNX and RKNN (RK3588, FP16)")
    parser.add_argument(
        "--weights",
        type=str,
        default=str(REPO_ROOT / "models" / "wissight.pt"),
        help="Path to wissight.pt",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=str(REPO_ROOT / "models"),
        help="Directory to save converted models",
    )
    parser.add_argument("--imgsz", type=int, default=640, help="Input image size (default: 640)")
    parser.add_argument("--platform", type=str, default="rk3588", help="Target Rockchip platform (default: rk3588)")

    args = parser.parse_args()

    weights_path = Path(args.weights)
    if not weights_path.exists():
        print(f"[!] Error: Model weights not found at {weights_path}")
        sys.exit(1)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    onnx_file = out_dir / "wissight.onnx"
    rknn_fp16_file = out_dir / f"wissight_{args.platform}_fp16.rknn"
    rknn_symlink = out_dir / "wissight.rknn"

    # 1. Export ONNX
    ok = export_onnx(weights_path, onnx_file, imgsz=args.imgsz)
    if not ok:
        sys.exit(1)

    # 2. Convert to RKNN
    ok = convert_to_rknn(onnx_file, rknn_fp16_file, target_platform=args.platform)
    if not ok:
        sys.exit(1)

    # 3. Create symlink
    if rknn_symlink.is_symlink() or rknn_symlink.exists():
        rknn_symlink.unlink()
    rknn_symlink.symlink_to(rknn_fp16_file.name)
    print(f"[+] Symlink created: {rknn_symlink.name} -> {rknn_fp16_file.name}")

    print("\n" + "=" * 65)
    print("[+] Wissight YOLOv8-seg RKNN Conversion Complete!")
    print(f"    - ONNX: {onnx_file} ({onnx_file.stat().st_size / (1024*1024):.1f} MB)")
    print(f"    - RKNN: {rknn_fp16_file} ({rknn_fp16_file.stat().st_size / (1024*1024):.1f} MB)")
    print(f"    - Link: {rknn_symlink}")
    print("=" * 65)


if __name__ == "__main__":
    main()
