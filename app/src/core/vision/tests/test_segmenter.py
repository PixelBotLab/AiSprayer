"""MobileSAM 解码切片、图签名校验与会话提示词回归测试（无模型权重依赖，纯 CPU numpy 验证）。"""

from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from core.vision.segmenter import (
    MobileSAMSegmenter,
    _check_decoder_graph,
    _run_onnx_decoder,
)


class _NodeInfo:
    """只带 name 的张量描述，模拟 onnxruntime 的 get_inputs/get_outputs。"""

    def __init__(self, name: str):
        self.name = name


# 不兼容图签名，用于校验加载拦截
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
        self.assertEqual(int(masks[int(np.argmax(ious))].sum()), 64)

    def test_already_sliced_export_is_passed_through(self):
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
            original_size=(100, 200),
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
    def test_runtime_graph_is_accepted(self):
        _check_decoder_graph(FakeDecoder(), Path("mobile_sam_decoder.onnx"))

    def test_rknn_export_graph_is_rejected(self):
        dec = FakeDecoder(signature=RKNN_SRC_SIGNATURE)
        with self.assertRaisesRegex(RuntimeError, "not the expected MobileSAM decoder graph"):
            _check_decoder_graph(dec, Path("mobile_sam_decoder.onnx"))


class FakePredictor:
    def __init__(self):
        self.call = None

    def predict(self, point_coords, point_labels, multimask_output):
        self.call = (np.asarray(point_coords), np.asarray(point_labels), multimask_output)
        n = 3 if multimask_output else 1
        return np.ones((n, 4, 4), bool), np.linspace(0.3, 0.9, n), np.zeros((n, 4, 4))


def session_with_box(box, points=(), labels=()):
    pred = FakePredictor()
    MobileSAMSegmenter(pred).predict(list(points), list(labels), box=box)
    return pred.call


class TestSessionBoxPrompt(unittest.TestCase):
    def test_box_becomes_two_corner_points(self):
        coords, lbls, multimask = session_with_box([320.9, 113.0, 731.1, 627.0])
        np.testing.assert_allclose(coords, [[320.9, 113.0], [731.1, 627.0]], rtol=1e-6)
        np.testing.assert_array_equal(lbls, [2, 3])
        self.assertFalse(multimask)

    def test_corner_order_is_normalized_and_box_alone_is_a_valid_prompt(self):
        coords, _, _ = session_with_box([731.1, 627.0, 320.9, 113.0])
        np.testing.assert_allclose(coords, [[320.9, 113.0], [731.1, 627.0]], rtol=1e-6)

    def test_box_and_points_are_merged(self):
        coords, lbls, multimask = session_with_box([10, 20, 30, 40], points=[(5, 5)], labels=[0])
        np.testing.assert_allclose(coords, [[5, 5], [10, 20], [30, 40]], rtol=1e-6)
        np.testing.assert_array_equal(lbls, [0, 2, 3])
        self.assertFalse(multimask)

    def test_single_point_without_box_still_uses_multimask(self):
        _, _, multimask = session_with_box(None, points=[(5, 5)], labels=[1])
        self.assertTrue(multimask)

    def test_no_prompt_returns_none(self):
        pred = FakePredictor()
        mask, score = MobileSAMSegmenter(pred).predict([], [])
        self.assertIsNone(mask)
        self.assertEqual(score, 0.0)
        self.assertIsNone(pred.call)

    def test_malformed_box_raises_instead_of_silently_ignoring(self):
        with self.assertRaisesRegex(ValueError, "box must have 4 numbers"):
            session_with_box([10, 20, 30])


if __name__ == "__main__":
    unittest.main(verbosity=2)
