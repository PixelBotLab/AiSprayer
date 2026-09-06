#!/usr/bin/env python3
"""
Convert MobileSAM PyTorch checkpoint (mobile_sam.pt) to RKNN FP16 format for Rockchip NPUs (e.g., RK3588).

MobileSAM consists of two main stages:
1. Image Encoder (TinyViT):
   - Input: RGB image (1, 3, 1024, 1024)
   - Output: Image embeddings (1, 256, 64, 64)
   - RKNN target: mobile_sam_encoder.rknn (FP16)
2. Prompt Encoder + Mask Decoder:
   - Input: Image embeddings (1, 256, 64, 64), point prompts, mask input
   - Output: iou_predictions (1, 4), low_res_masks (1, 4, 256, 256)
   - RKNN target: mobile_sam_decoder.rknn (FP16)

IMPORTANT - the decoder graph produced here is NOT the one the runtime uses.
`core/vision/image2d/mobilesam_session.py` expects the graph exported by
third_party/MobileSAM/scripts/export_onnx_model.py (SamOnnxModel), which takes an extra
`orig_im_size` input, upsamples the masks to the original resolution inside the graph and
returns 3 outputs (masks, iou_predictions, low_res_masks). The wrapper below calls
`mask_decoder.predict_masks()` (bypassing `MaskDecoder.forward`), so it emits 4 heads and no
`orig_im_size` - incompatible signatures. Therefore this script writes its decoder ONNX as a
throwaway intermediate next to the RKNN output name, never as `models/mobile_sam_decoder.onnx`,
which would break the running app. The 4-head slicing is done on the caller side
(_run_onnx_decoder), and a graph signature check rejects the wrong file at load time.
"""

import os
import sys
import argparse
from pathlib import Path
import torch
import torch.nn as nn
import warnings

# Add third_party/MobileSAM
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "third_party" / "MobileSAM"))

from mobile_sam import sam_model_registry
from rknn.api import RKNN


class MobileSamDecoderForRKNN(nn.Module):
    """Static graph wrapper for SAM Mask Decoder + Prompt Encoder for RKNN."""

    def __init__(self, model):
        super().__init__()
        self.prompt_encoder = model.prompt_encoder
        self.mask_decoder = model.mask_decoder
        self.img_size = float(model.image_encoder.img_size)

    def _embed_points(self, point_coords: torch.Tensor, point_labels: torch.Tensor) -> torch.Tensor:
        point_coords = (point_coords + 0.5) / self.img_size
        point_embedding = self.prompt_encoder.pe_layer._pe_encoding(point_coords)
        point_labels = point_labels.unsqueeze(-1).expand_as(point_embedding)

        point_embedding = point_embedding * (point_labels != -1) + self.prompt_encoder.not_a_point_embed.weight * (
            point_labels == -1
        )
        for i in range(self.prompt_encoder.num_point_embeddings):
            point_embedding = point_embedding + self.prompt_encoder.point_embeddings[i].weight * (
                point_labels == i
            )
        return point_embedding

    def _embed_masks(self, input_mask: torch.Tensor, has_mask_input: torch.Tensor) -> torch.Tensor:
        mask_embedding = has_mask_input * self.prompt_encoder.mask_downscaling(input_mask)
        mask_embedding = mask_embedding + (1.0 - has_mask_input) * self.prompt_encoder.no_mask_embed.weight.reshape(
            1, -1, 1, 1
        )
        return mask_embedding

    def forward(
        self,
        image_embeddings: torch.Tensor,
        point_coords: torch.Tensor,
        point_labels: torch.Tensor,
        mask_input: torch.Tensor,
        has_mask_input: torch.Tensor,
    ):
        sparse_embedding = self._embed_points(point_coords, point_labels)
        dense_embedding = self._embed_masks(mask_input, has_mask_input)
        low_res_masks, iou_predictions = self.mask_decoder.predict_masks(
            image_embeddings=image_embeddings,
            image_pe=self.prompt_encoder.get_dense_pe(),
            sparse_prompt_embeddings=sparse_embedding,
            dense_prompt_embeddings=dense_embedding,
        )
        return iou_predictions, low_res_masks


def export_encoder_onnx(sam, output_onnx_path: str, img_size: int = 1024, opset: int = 16):
    print(f"--> [1/4] Exporting Image Encoder to ONNX ({output_onnx_path}, img_size={img_size})...")
    encoder = sam.image_encoder
    encoder.eval()

    dummy_input = torch.randn(1, 3, img_size, img_size)

    with warnings.catch_warnings():
        warnings.filterwarnings("ignore")
        torch.onnx.export(
            encoder,
            dummy_input,
            output_onnx_path,
            export_params=True,
            verbose=False,
            opset_version=opset,
            do_constant_folding=True,
            input_names=["image"],
            output_names=["image_embeddings"],
        )
    print(f"--> Image Encoder ONNX export complete.")


def convert_encoder_rknn(
    onnx_path: str,
    rknn_path: str,
    target_platform: str = "rk3588",
    do_quantization: bool = False,
):
    print(f"--> [2/4] Converting Image Encoder to RKNN ({rknn_path}), target={target_platform}, FP16...")
    rknn = RKNN(verbose=False)

    # Pixel normalization (standard MobileSAM / SAM normalization)
    rknn.config(
        target_platform=target_platform,
        mean_values=[[123.675, 116.28, 103.53]],
        std_values=[[58.395, 57.12, 57.375]],
    )

    ret = rknn.load_onnx(model=onnx_path)
    if ret != 0:
        raise RuntimeError(f"Failed to load encoder ONNX model {onnx_path}, code: {ret}")

    ret = rknn.build(do_quantization=do_quantization)
    if ret != 0:
        raise RuntimeError(f"Failed to build encoder RKNN model, code: {ret}")

    ret = rknn.export_rknn(rknn_path)
    if ret != 0:
        raise RuntimeError(f"Failed to export encoder RKNN model to {rknn_path}, code: {ret}")

    rknn.release()
    print(f"--> Image Encoder RKNN conversion complete.")


def export_decoder_onnx(
    sam,
    output_onnx_path: str,
    num_points: int = 2,
    embed_dim: int = 256,
    embed_size: int = 64,
    opset: int = 16,
):
    print(f"--> [3/4] Exporting Mask Decoder to ONNX ({output_onnx_path})...")
    dec = MobileSamDecoderForRKNN(sam)
    dec.eval()

    mask_input_size = (4 * embed_size, 4 * embed_size)
    dummy_inputs = (
        torch.randn(1, embed_dim, embed_size, embed_size),
        torch.randint(0, 1024, (1, num_points, 2)).float(),
        torch.randint(0, 4, (1, num_points)).float(),
        torch.randn(1, 1, *mask_input_size),
        torch.tensor([0.0]),
    )

    with warnings.catch_warnings():
        warnings.filterwarnings("ignore")
        torch.onnx.export(
            dec,
            dummy_inputs,
            output_onnx_path,
            export_params=True,
            verbose=False,
            opset_version=opset,
            do_constant_folding=True,
            input_names=["image_embeddings", "point_coords", "point_labels", "mask_input", "has_mask_input"],
            output_names=["iou_predictions", "low_res_masks"],
        )
    print(f"--> Mask Decoder ONNX export complete.")


def convert_decoder_rknn(
    onnx_path: str,
    rknn_path: str,
    target_platform: str = "rk3588",
    do_quantization: bool = False,
):
    print(f"--> [4/4] Converting Mask Decoder to RKNN ({rknn_path}), target={target_platform}, FP16...")
    rknn = RKNN(verbose=False)
    rknn.config(target_platform=target_platform)

    ret = rknn.load_onnx(onnx_path)
    if ret != 0:
        raise RuntimeError(f"Failed to load decoder ONNX model {onnx_path}, code: {ret}")

    ret = rknn.build(do_quantization=do_quantization)
    if ret != 0:
        raise RuntimeError(f"Failed to build decoder RKNN model, code: {ret}")

    ret = rknn.export_rknn(rknn_path)
    if ret != 0:
        raise RuntimeError(f"Failed to export decoder RKNN model to {rknn_path}, code: {ret}")

    rknn.release()
    print(f"--> Mask Decoder RKNN conversion complete.")


def update_symlink(source_filename: str, link_path: Path):
    if link_path.exists() or link_path.is_symlink():
        link_path.unlink()
    try:
        os.symlink(source_filename, link_path)
    except OSError:
        import shutil
        shutil.copyfile(link_path.parent / source_filename, link_path)


def main():
    parser = argparse.ArgumentParser(description="Convert MobileSAM checkpoint to RKNN FP16")
    parser.add_argument("--checkpoint", type=str, default="models/mobile_sam.pt", help="Path to mobile_sam.pt")
    parser.add_argument("--model-type", type=str, default="vit_t", help="SAM model type (default: vit_t)")
    parser.add_argument("--target", type=str, default="rk3588", help="Target Rockchip platform (default: rk3588)")
    parser.add_argument("--output-dir", type=str, default="models", help="Output directory for RKNN models")
    parser.add_argument("--img-size", type=int, default=1024, help="Encoder input resolution (default: 1024)")
    parser.add_argument("--opset", type=int, default=16, help="ONNX opset version (default: 16)")
    args = parser.parse_args()

    checkpoint_path = Path(args.checkpoint).resolve()
    if not checkpoint_path.exists():
        print(f"Error: Checkpoint file {checkpoint_path} does not exist.")
        sys.exit(1)

    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading MobileSAM from {checkpoint_path}...")
    sam = sam_model_registry[args.model_type](checkpoint=str(checkpoint_path))
    sam.eval()

    # 1 & 2: Export and Convert Image Encoder
    encoder_onnx = str(out_dir / f"mobile_sam_encoder_{args.img_size}.onnx")
    export_encoder_onnx(sam, encoder_onnx, img_size=args.img_size, opset=args.opset)

    encoder_rknn_filename = f"mobile_sam_encoder_{args.img_size}_{args.target}_fp16.rknn"
    encoder_rknn = str(out_dir / encoder_rknn_filename)
    convert_encoder_rknn(
        onnx_path=encoder_onnx,
        rknn_path=encoder_rknn,
        target_platform=args.target,
        do_quantization=False,
    )
    update_symlink(encoder_rknn_filename, out_dir / "mobile_sam_encoder.rknn")

    # 3 & 4: Export and Convert Mask Decoder
    # NOTE: deliberately NOT named mobile_sam_decoder.onnx - that file belongs to the runtime
    # (exported by third_party/MobileSAM/scripts/export_onnx_model.py) and must not be overwritten.
    decoder_onnx = str(out_dir / f"mobile_sam_decoder_{args.target}_rknn_src.onnx")
    export_decoder_onnx(sam, decoder_onnx, opset=args.opset)

    decoder_rknn_filename = f"mobile_sam_decoder_{args.target}_fp16.rknn"
    decoder_rknn = str(out_dir / decoder_rknn_filename)
    convert_decoder_rknn(
        onnx_path=decoder_onnx,
        rknn_path=decoder_rknn,
        target_platform=args.target,
        do_quantization=False,
    )
    update_symlink(decoder_rknn_filename, out_dir / "mobile_sam_decoder.rknn")

    print("\n========================================================")
    print("MobileSAM RKNN FP16 Conversion Finished Successfully!")
    print(f"1. Encoder RKNN: {encoder_rknn}")
    print(f"   Symlink:      {out_dir / 'mobile_sam_encoder.rknn'}")
    print(f"2. Decoder RKNN: {decoder_rknn}")
    print(f"   Symlink:      {out_dir / 'mobile_sam_decoder.rknn'}")
    print("========================================================")


if __name__ == "__main__":
    main()
