# -*- coding: utf-8 -*-
"""
`_run_onnx_decoder` 的候选切片回归（无模型权重、无 NPU，纯 numpy，可直接跑）。

钉住的是 RKNN/ONNX 后端接入时丢掉的一段 SAM 语义：mask decoder 有 4 个输出头，0 号是
`multimask_output=False` 的"稳定"单 mask，1~3 号是三个粒度候选；而 ONNX 导出把 4 路原样
返回。旧实现改成"在 4 路里按 iou_predictions argmax"，于是每加一个背景点，选中的头就会在
不同粒度之间跳变，表现为"右键点背景反而让 mask 变大"（真实事故回归，见 backend 日志
`+fg=1, -bg=N -> contours` 从 2 涨到 20）。

顺带钉住另一件事：`_check_decoder_graph` 必须拒掉 tools/convert_mobilesam_to_rknn.py 导出的那种图
（无 orig_im_size、只 2 路输出且 iou 在前）—— `_run_onnx_decoder` 是按位置解包的，输出数量
碰巧对上时就会静默地把 iou 当成 mask 用，所以必须按名字和顺序显式校验。

跑法：
    cd app/src && python3 -m core.vision.image2d.test_sam_decoder_slice
"""
from __future__ import annotations

import os
import sys
import unittest

from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from core.vision.image2d.mobilesam_session import (  # noqa: E402
    _check_decoder_graph,
    _run_onnx_decoder,
)


class _NodeInfo:
    """只带 name 的张量描述，足够模拟 ort 的 get_inputs/get_outputs。"""

    def __init__(self, name: str):
        self.name = name


# tools/convert_mobilesam_to_rknn.py 导出的那套（不兼容）签名，用于验证加载时会被拒
RKNN_SRC_SIGNATURE = (
    ["image_embeddings", "point_coords", "point_labels", "mask_input", "has_mask_input"],
    ["iou_predictions", "low_res_masks"],
)


class FakeDecoder:
    """伪造 decoder 图：4 路 mask 面积依次 4/16/36/64 像素，iou 最大的是第 3 路。"""

    def __init__(self, heads: int = 4, signature=None):
        self.heads = heads
        self.signature = signature
        self.feed = None

    def get_inputs(self):
        names = self.signature[0] if self.signature else [
            "image_embeddings", "point_coords", "point_labels",
            "mask_input", "has_mask_input", "orig_im_size",
        ]
        return [_NodeInfo(n) for n in names]

    def get_outputs(self):
        names = self.signature[1] if self.signature else ["masks", "iou_predictions", "low_res_masks"]
        return [_NodeInfo(n) for n in names]

    def run(self, _output_names, feed):
        self.feed = feed
        masks = np.zeros((1, self.heads, 8, 8), np.float32)
        for i in range(self.heads):
            side = 2 * (i + 1)
            masks[0, i, :side, :side] = 1.0
        iou = np.array([0.1, 0.2, 0.3, 0.9][: self.heads], dtype=np.float32)[None, :]
        low_res = np.zeros((1, self.heads, 4, 4), np.float32)
        return masks, iou, low_res


def decode(decoder, multimask_output):
    return _run_onnx_decoder(
        decoder_session=decoder,
        features=np.zeros((1, 256, 64, 64), np.float32),
        original_size=(8, 8),
        input_size=(8, 8),
        point_coords=[[1, 1], [5, 5]],
        point_labels=[1, 0],
        multimask_output=multimask_output,
    )


class TestMultimaskSlicing(unittest.TestCase):
    def test_single_mask_takes_stable_head_not_best_iou(self):
        """精修（含背景点）必须拿 0 号稳定头，面积 4；argmax 会拿到 iou 最高的 3 号（面积 64）。"""
        masks, ious, low_res = decode(FakeDecoder(), multimask_output=False)
        self.assertEqual(masks.shape[0], 1)
        self.assertEqual(int(masks[0].sum()), 4)
        self.assertEqual(ious.shape, (1,))
        self.assertAlmostEqual(float(ious[0]), 0.1)
        self.assertEqual(low_res.shape[0], 1)

    def test_multimask_keeps_the_three_granularity_heads(self):
        masks, ious, _ = decode(FakeDecoder(), multimask_output=True)
        self.assertEqual(masks.shape[0], 3)
        np.testing.assert_allclose(ious, [0.2, 0.3, 0.9], rtol=1e-6)
        # 调用方（MobileSAMSession）按 score argmax，应落在最粗的 3 号头上
        self.assertEqual(int(masks[int(np.argmax(ious))].sum()), 64)

    def test_already_sliced_export_is_passed_through(self):
        """导出图若已按 SAM 约定切好（1 路），不得再切。"""
        masks, _, _ = decode(FakeDecoder(heads=1), multimask_output=False)
        self.assertEqual(masks.shape[0], 1)
        self.assertEqual(int(masks[0].sum()), 4)

    def test_masks_are_binary_logits_thresholded(self):
        masks, _, _ = decode(FakeDecoder(), multimask_output=False)
        self.assertTrue(np.isfinite(masks).all())
        self.assertTrue(set(np.unique(masks)) <= {False, True})

    def test_points_are_scaled_to_input_size(self):
        dec = FakeDecoder()
        _run_onnx_decoder(
            decoder_session=dec,
            features=np.zeros((1, 256, 64, 64), np.float32),
            original_size=(100, 200),   # (h, w)
            input_size=(50, 100),
            point_coords=[[100, 50]],
            point_labels=[1],
            multimask_output=True,
        )
        np.testing.assert_allclose(dec.feed["point_coords"], [[[50.0, 25.0]]], rtol=1e-6)
        np.testing.assert_allclose(dec.feed["orig_im_size"], [100.0, 200.0])
        self.assertEqual(dec.feed["point_coords"].dtype, np.float32)
        self.assertEqual(dec.feed["has_mask_input"].sum(), 0.0)


class TestGraphSignatureGuard(unittest.TestCase):
    """拿错 decoder 文件时必须加载失败，而不是按位置解包后静默错位。"""

    def test_runtime_graph_is_accepted(self):
        _check_decoder_graph(FakeDecoder(), Path("mobile_sam_decoder.onnx"))

    def test_rknn_export_graph_is_rejected(self):
        dec = FakeDecoder(signature=RKNN_SRC_SIGNATURE)
        with self.assertRaisesRegex(RuntimeError, "not the expected MobileSAM decoder graph"):
            _check_decoder_graph(dec, Path("mobile_sam_decoder.onnx"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
