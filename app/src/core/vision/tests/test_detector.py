"""Wissight 目标检测后处理与 Mask 解码回归测试（无模型权重依赖，纯 CPU numpy 验证）。"""

from __future__ import annotations

import unittest

import numpy as np

from core.vision.detector import (
    CLASS_NAMES,
    IMGSZ,
    WissightDetector,
    resolve_classes,
)
from core.vision.types import Detection

TROUSERS = CLASS_NAMES.index("trousers")
SHIRT = CLASS_NAMES.index("short_sleeved_shirt")


def make_detector(classes=("trousers",), conf=0.25, iou=0.7, max_boxes=5) -> WissightDetector:
    """绕过 __init__ 的模型文件加载，构造一个专用于后处理算法测试的实例。"""
    det = WissightDetector.__new__(WissightDetector)
    det.conf = conf
    det.iou = iou
    det.max_boxes = max_boxes
    det.class_ids = resolve_classes(classes)
    return det


def create_detection(x1, y1, x2, y2, cls_id=TROUSERS, score=0.9) -> Detection:
    return Detection(
        box=(float(x1), float(y1), float(x2), float(y2)),
        cls_id=int(cls_id),
        cls_name=CLASS_NAMES[cls_id],
        score=float(score),
    )


class TestClassResolution(unittest.TestCase):
    def test_empty_means_no_filter(self):
        self.assertIsNone(resolve_classes(None))
        self.assertIsNone(resolve_classes([]))

    def test_names_map_to_ids(self):
        self.assertEqual(
            resolve_classes(["trousers", "skirt"]),
            {TROUSERS, CLASS_NAMES.index("skirt")},
        )

    def test_unknown_name_raises(self):
        with self.assertRaisesRegex(ValueError, "Unknown detector class name"):
            resolve_classes(["trousers", "jeans"])


class TestLetterbox(unittest.TestCase):
    def test_centered_pad_and_value_range(self):
        img = np.zeros((800, 1280, 3), np.uint8)
        blob, scale, pad_x, pad_y = WissightDetector._letterbox(img)
        self.assertEqual(blob.shape, (1, 3, IMGSZ, IMGSZ))
        self.assertEqual(blob.dtype, np.float32)
        self.assertAlmostEqual(scale, 0.5)
        self.assertEqual((pad_x, pad_y), (0.0, 120.0))
        self.assertAlmostEqual(float(blob[0, 0, IMGSZ - 1, 320]), 114.0 / 255.0)

    def test_input_is_contiguous_for_ort_and_rknn(self):
        img = np.random.randint(0, 255, (800, 1280, 3), dtype=np.uint8)
        blob, _, _, _ = WissightDetector._letterbox(img)
        self.assertTrue(blob.flags["C_CONTIGUOUS"])


class TestDecode(unittest.TestCase):
    def test_box_is_mapped_back_to_original_coordinates(self):
        output0 = np.zeros((1, 4 + len(CLASS_NAMES) + 32, 8400), np.float32)
        output0[0, 0:4, 5] = [320.0, 320.0, 200.0, 300.0]
        output0[0, 4 + TROUSERS, 5] = 0.9

        dets = WissightDetector._decode(output0, 0.5, 0.0, 120.0, (800, 1280), conf=0.25)

        self.assertEqual(len(dets), 1)
        self.assertEqual(dets[0].cls_name, "trousers")
        self.assertAlmostEqual(dets[0].score, 0.9)
        self.assertEqual([round(v) for v in dets[0].box], [440, 100, 840, 700])

    def test_boxes_are_clipped_to_the_image(self):
        output0 = np.zeros((1, 4 + len(CLASS_NAMES) + 32, 8400), np.float32)
        output0[0, 0:4, 7] = [10.0, 10.0, 400.0, 400.0]
        output0[0, 4 + TROUSERS, 7] = 0.9

        dets = WissightDetector._decode(output0, 0.5, 0.0, 120.0, (800, 1280), conf=0.25)

        self.assertEqual(len(dets), 1)
        self.assertEqual([round(v) for v in dets[0].box], [0, 0, 420, 180])

    def test_low_score_is_dropped(self):
        output0 = np.zeros((1, 4 + len(CLASS_NAMES) + 32, 8400), np.float32)
        output0[0, 0:4, 5] = [320.0, 320.0, 200.0, 300.0]
        output0[0, 4 + TROUSERS, 5] = 0.2
        self.assertEqual(WissightDetector._decode(output0, 0.5, 0.0, 120.0, (800, 1280), 0.25), [])

    def test_unexpected_output_width_is_reported(self):
        self.assertEqual(WissightDetector._decode(np.zeros((1, 4, 10), np.float32), 1, 0, 0, (800, 800), 0.25), [])

    def test_wider_graph_with_other_class_count_is_rejected(self):
        wrong = np.zeros((1, 4 + 80 + 32, 100), np.float32)
        wrong[0, 0:4, 5] = [320.0, 320.0, 200.0, 300.0]
        wrong[0, 4 + 7, 5] = 0.9
        self.assertEqual(
            WissightDetector._decode(wrong, 1.0, 0.0, 0.0, (800, 1280), conf=0.25), []
        )


class TestFinalize(unittest.TestCase):
    def test_overlapping_boxes_are_suppressed(self):
        det = make_detector(classes=None)
        kept = det._finalize([
            create_detection(100, 100, 300, 500, score=0.9),
            create_detection(104, 103, 302, 503, score=0.8),
            create_detection(600, 100, 700, 200, score=0.7),
        ])
        self.assertEqual(len(kept), 2)

    def test_class_whitelist_filters(self):
        det = make_detector(classes=("trousers",))
        kept = det._finalize([
            create_detection(0, 0, 100, 100, cls_id=TROUSERS),
            create_detection(200, 0, 300, 100, cls_id=SHIRT),
        ])
        self.assertEqual([d.cls_name for d in kept], ["trousers"])

    def test_sorted_by_area_and_truncated(self):
        det = make_detector(classes=None, max_boxes=2)
        kept = det._finalize([
            create_detection(0, 0, 10, 10),
            create_detection(100, 100, 400, 600),
            create_detection(500, 0, 700, 200),
        ])
        self.assertEqual(len(kept), 2)
        self.assertEqual(kept[0].area, 300 * 500)
        self.assertEqual(kept[1].area, 200 * 200)

    def test_empty_input(self):
        self.assertEqual(make_detector()._finalize([]), [])


class TestInstanceMasks(unittest.TestCase):
    HEAD = 4 + len(CLASS_NAMES) + 32

    def _output0(self, coeffs, xywh=(320.0, 320.0, 200.0, 300.0), score=0.9):
        out = np.zeros((1, self.HEAD, 8400), np.float32)
        out[0, 0:4, 5] = list(xywh)
        out[0, 4 + TROUSERS, 5] = score
        out[0, -32:, 5] = coeffs
        return out

    def test_uniform_protos_fill_the_whole_box(self):
        coeffs = np.zeros(32, np.float32)
        coeffs[0] = 100.0
        protos = np.ones((1, 32, 160, 160), np.float32)

        dets = WissightDetector._decode(
            self._output0(coeffs), 1.0, 0.0, 0.0, (640, 640), conf=0.25, protos=protos
        )

        self.assertEqual(len(dets), 1)
        self.assertEqual(dets[0].mask.shape, (640, 640))
        area = int(dets[0].mask.sum()) // 255
        self.assertAlmostEqual(area, 200 * 300, delta=200 * 300 * 0.03)
        self.assertEqual(int(dets[0].mask[:, :220].sum()), 0)
        self.assertEqual(int(dets[0].mask[470:, :].sum()), 0)

    def test_proto_spatial_pattern_is_cropped_by_the_box(self):
        coeffs = np.zeros(32, np.float32)
        coeffs[0] = 10.0
        protos = np.ones((1, 32, 160, 160), np.float32)
        protos[0, 0, :, 80:] = -1.0

        dets = WissightDetector._decode(
            self._output0(coeffs), 1.0, 0.0, 0.0, (640, 640), conf=0.25, protos=protos
        )

        mask = dets[0].mask
        area = int(mask.sum()) // 255
        self.assertAlmostEqual(area, 100 * 300, delta=100 * 300 * 0.05)
        self.assertEqual(int(mask[:, 320:].sum()), 0)

    def test_proto_channel_mismatch_degrades_instead_of_crashing(self):
        coeffs = np.zeros(32, np.float32)
        coeffs[0] = 100.0
        dets = WissightDetector._decode(
            self._output0(coeffs),
            1.0,
            0.0,
            0.0,
            (640, 640),
            conf=0.25,
            protos=np.ones((1, 8, 160, 160), np.float32),
        )
        self.assertEqual(len(dets), 1)
        self.assertIsNone(dets[0].mask)

    def test_mask_is_absent_by_default(self):
        coeffs = np.zeros(32, np.float32)
        dets = WissightDetector._decode(
            self._output0(coeffs), 0.5, 0.0, 120.0, (800, 1280), conf=0.25
        )
        self.assertEqual(len(dets), 1)
        self.assertIsNone(dets[0].mask)


if __name__ == "__main__":
    unittest.main(verbosity=2)
