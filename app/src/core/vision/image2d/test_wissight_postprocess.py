"""Wissight 检测后处理的回归测试（不依赖模型文件与 NPU）。

钉住五件容易静默出错的事：
1. letterbox 的逆变换 —— 减 pad 与除 scale 的顺序错了，框会整体偏移，而图上看着"差不多"；
2. 框必须被截到图内 —— 负坐标当 box prompt 会污染整张 embedding；
3. `cv2.dnn.NMSBoxes` 要 (x, y, w, h) 而不是 xyxy —— 传错不会报错，只会让抑制范围悄悄失真；
4. 类别白名单、面积降序与 output0 宽度契约 —— 界面只取第一个框，而类别数变了时前 13 个
   通道会被当成原类别静默错用；
5. 实例 mask 解码（关掉 sam_refine 时它就是终稿）—— box 外必须为 0、裁剪比例要对、
   protos 与系数不匹配时宁可降级也不能崩。

跑法：
    cd app/src && python3 -m core.vision.image2d.test_wissight_postprocess
"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from core.vision.image2d.wissight_detector import (  # noqa: E402
    CLASS_NAMES,
    IMGSZ,
    Detection,
    WissightDetector,
    resolve_classes,
)

TROUSERS = CLASS_NAMES.index("trousers")
SHIRT = CLASS_NAMES.index("short_sleeved_shirt")


def make_detector(classes=("trousers",), conf=0.25, iou=0.7, max_boxes=5):
    """绕过 __init__ 的模型加载，只要一个能做后处理的实例。"""
    det = WissightDetector.__new__(WissightDetector)
    det.conf = conf
    det.iou = iou
    det.max_boxes = max_boxes
    det.class_ids = resolve_classes(classes)
    return det


def detection(x1, y1, x2, y2, cls_id=TROUSERS, score=0.9):
    return Detection(box=(x1, y1, x2, y2), cls_id=cls_id, cls_name=CLASS_NAMES[cls_id], score=score)


class TestClassResolution(unittest.TestCase):
    def test_empty_means_no_filter(self):
        self.assertIsNone(resolve_classes(None))
        self.assertIsNone(resolve_classes([]))

    def test_names_map_to_ids(self):
        self.assertEqual(resolve_classes(["trousers", "skirt"]), {TROUSERS, CLASS_NAMES.index("skirt")})

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
        # 填充区是 PAD_VALUE/255，不是 0：MobileSAM 的 _resize_and_pad 才是右下补 0，两者别混用
        self.assertAlmostEqual(float(blob[0, 0, IMGSZ - 1, 320]), 114.0 / 255.0)

    def test_input_is_contiguous_for_ort_and_rknn(self):
        img = np.random.randint(0, 255, (800, 1280, 3), dtype=np.uint8)
        blob, _, _, _ = WissightDetector._letterbox(img)
        self.assertTrue(blob.flags["C_CONTIGUOUS"])


class TestDecode(unittest.TestCase):
    def test_box_is_mapped_back_to_original_coordinates(self):
        # 1280x800 -> scale 0.5, pad_y 120；在 640 空间放一个 xywh=(320,320,200,300) 的框
        output0 = np.zeros((1, 4 + len(CLASS_NAMES) + 32, 8400), np.float32)
        output0[0, 0:4, 5] = [320.0, 320.0, 200.0, 300.0]
        output0[0, 4 + TROUSERS, 5] = 0.9

        dets = WissightDetector._decode(output0, 0.5, 0.0, 120.0, (800, 1280), conf=0.25)

        self.assertEqual(len(dets), 1)
        self.assertEqual(dets[0].cls_name, "trousers")
        self.assertAlmostEqual(dets[0].score, 0.9)
        # x: (220-0)/0.5=440, (420-0)/0.5=840；y: (170-120)/0.5=100, (470-120)/0.5=700
        self.assertEqual([round(v) for v in dets[0].box], [440, 100, 840, 700])

    def test_boxes_are_clipped_to_the_image(self):
        """框外候选必须被截断：负坐标送去当 SAM box prompt 会污染整个 embedding。"""
        output0 = np.zeros((1, 4 + len(CLASS_NAMES) + 32, 8400), np.float32)
        output0[0, 0:4, 7] = [10.0, 10.0, 400.0, 400.0]  # 中心近左上角，反变换后为负
        output0[0, 4 + TROUSERS, 7] = 0.9

        dets = WissightDetector._decode(output0, 0.5, 0.0, 120.0, (800, 1280), conf=0.25)

        self.assertEqual(len(dets), 1)
        # 640 空间 xyxy=(-190,-190,210,210) -> 原图 (-380,-620,420,180) -> 截断 (0,0,420,180)
        self.assertEqual([round(v) for v in dets[0].box], [0, 0, 420, 180])

    def test_low_score_is_dropped(self):
        output0 = np.zeros((1, 4 + len(CLASS_NAMES) + 32, 8400), np.float32)
        output0[0, 0:4, 5] = [320.0, 320.0, 200.0, 300.0]
        output0[0, 4 + TROUSERS, 5] = 0.2
        self.assertEqual(WissightDetector._decode(output0, 0.5, 0.0, 120.0, (800, 1280), 0.25), [])

    def test_unexpected_output_width_is_reported(self):
        self.assertEqual(WissightDetector._decode(np.zeros((1, 4, 10), np.float32), 1, 0, 0, (800, 800), 0.25), [])

    def test_wider_graph_with_other_class_count_is_rejected(self):
        """4+80+32 的 COCO 导出：前 13 个通道会被当成本项目的类别，必须拒而不是猜。"""
        wrong = np.zeros((1, 4 + 80 + 32, 100), np.float32)
        wrong[0, 0:4, 5] = [320.0, 320.0, 200.0, 300.0]
        wrong[0, 4 + 7, 5] = 0.9  # COCO 里的 "tie" 通道，绝不能被当成 trousers
        self.assertEqual(
            WissightDetector._decode(wrong, 1.0, 0.0, 0.0, (800, 1280), conf=0.25), []
        )


class TestFinalize(unittest.TestCase):
    def test_overlapping_boxes_are_suppressed(self):
        """两条几乎重合的框只应留一个 —— 只有按 (x,y,w,h) 传才对，用 xyxy 会把 w/h 算成 2 倍。"""
        det = make_detector(classes=None)
        kept = det._finalize([
            detection(100, 100, 300, 500, score=0.9),
            detection(104, 103, 302, 503, score=0.8),
            detection(600, 100, 700, 200, score=0.7),
        ])
        self.assertEqual(len(kept), 2)

    def test_class_whitelist_filters(self):
        det = make_detector(classes=("trousers",))
        kept = det._finalize([detection(0, 0, 100, 100, cls_id=TROUSERS),
                              detection(200, 0, 300, 100, cls_id=SHIRT)])
        self.assertEqual([d.cls_name for d in kept], ["trousers"])

    def test_sorted_by_area_and_truncated(self):
        det = make_detector(classes=None, max_boxes=2)
        kept = det._finalize([
            detection(0, 0, 10, 10),
            detection(100, 100, 400, 600),
            detection(500, 0, 700, 200),
        ])
        self.assertEqual(len(kept), 2)
        self.assertEqual(kept[0].area, 300 * 500)
        self.assertEqual(kept[1].area, 200 * 200)

    def test_empty_input(self):
        self.assertEqual(make_detector()._finalize([]), [])


class TestInstanceMasks(unittest.TestCase):
    """关掉 sam_refine 时，这里解出的 mask 就是落盘的最终结果，没有任何人能发现它偏了。"""

    HEAD = 4 + len(CLASS_NAMES) + 32

    def _output0(self, coeffs, xywh=(320.0, 320.0, 200.0, 300.0), score=0.9):
        out = np.zeros((1, self.HEAD, 8400), np.float32)
        out[0, 0:4, 5] = list(xywh)
        out[0, 4 + TROUSERS, 5] = score
        out[0, -32:, 5] = coeffs
        return out

    def test_uniform_protos_fill_the_whole_box(self):
        # protos 全 1、系数把第一通道拉到 +100 -> sigmoid 处处接近 1 -> 整个 box 被填上
        coeffs = np.zeros(32, np.float32)
        coeffs[0] = 100.0
        protos = np.ones((1, 32, 160, 160), np.float32)

        dets = WissightDetector._decode(
            self._output0(coeffs), 1.0, 0.0, 0.0, (640, 640), conf=0.25, protos=protos
        )

        self.assertEqual(len(dets), 1)
        self.assertEqual(dets[0].mask.shape, (640, 640))
        # box = xywh(320,320,200,300) -> xyxy (220,170,420,470)
        area = int(dets[0].mask.sum()) // 255
        self.assertAlmostEqual(area, 200 * 300, delta=200 * 300 * 0.03)
        # box 外必须一个像素都没有，否则整张图都会被当成喷漆区域
        self.assertEqual(int(dets[0].mask[:, :220].sum()), 0)
        self.assertEqual(int(dets[0].mask[470:, :].sum()), 0)

    def test_proto_spatial_pattern_is_cropped_by_the_box(self):
        """proto 左正右负：mask 应只占 box 的左半，且按 640->160 的 1/4 比例对齐。"""
        coeffs = np.zeros(32, np.float32)
        coeffs[0] = 10.0
        protos = np.ones((1, 32, 160, 160), np.float32)
        protos[0, 0, :, 80:] = -1.0  # 只有通道 0 有空间模式，右半为负

        dets = WissightDetector._decode(
            self._output0(coeffs), 1.0, 0.0, 0.0, (640, 640), conf=0.25, protos=protos
        )

        mask = dets[0].mask
        area = int(mask.sum()) // 255
        # box 在 proto 空间是 x in [55,105)，跨过 proto 中线 80 -> 只保留原图 x in [220,320)
        self.assertAlmostEqual(area, 100 * 300, delta=100 * 300 * 0.05)
        self.assertEqual(int(mask[:, 320:].sum()), 0)

    def test_proto_channel_mismatch_degrades_instead_of_crashing(self):
        """protos 通道数与系数对不上：宁可不给 mask，也不能拿错的系数拼出个看上去正常的轮廓。"""
        coeffs = np.zeros(32, np.float32)
        coeffs[0] = 100.0
        dets = WissightDetector._decode(
            self._output0(coeffs), 1.0, 0.0, 0.0, (640, 640), conf=0.25,
            protos=np.ones((1, 8, 160, 160), np.float32),
        )
        self.assertEqual(len(dets), 1)
        self.assertIsNone(dets[0].mask)

    def test_mask_is_absent_by_default(self):
        """开精修（默认）时不传 protos，不要白算一份 1280x800 的 mask。"""
        coeffs = np.zeros(32, np.float32)
        dets = WissightDetector._decode(
            self._output0(coeffs), 0.5, 0.0, 120.0, (800, 1280), conf=0.25
        )
        self.assertEqual(len(dets), 1)
        self.assertIsNone(dets[0].mask)


if __name__ == "__main__":
    unittest.main(verbosity=2)
