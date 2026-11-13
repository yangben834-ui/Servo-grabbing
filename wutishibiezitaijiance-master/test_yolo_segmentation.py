"""PT/RKNN shared segmentation post-processing tests."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from yolo_segmentation import (
    IoUTracker,
    LetterboxInfo,
    postprocess_segmentation,
    resolve_model_path,
)
from bottle_grasp_pipeline import YoloSegmenter, select_tracker


class FakeTensor:
    def __init__(self, value):
        self.value = np.asarray(value)

    def __getitem__(self, index):
        return FakeTensor(self.value[index])

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.value.copy()

    def __int__(self):
        return int(self.value)

    def __float__(self):
        return float(self.value)


class YoloSegmentationTest(unittest.TestCase):
    def test_tracker_selection_and_rknn_fallback(self):
        self.assertEqual(select_tracker(None, "pt"), ("iou", "iou"))
        self.assertEqual(select_tracker("  ", "pt"), ("iou", "iou"))
        self.assertEqual(select_tracker("botsort", "pt"), ("botsort", "botsort"))
        self.assertEqual(select_tracker("custom.yaml", "pt"), ("custom.yaml", "custom.yaml"))
        self.assertEqual(select_tracker("ocsort", "rknn"), ("ocsort", "iou"))

    def test_ultralytics_tracker_results_are_normalized(self):
        captured = {}
        box = SimpleNamespace(
            xyxy=FakeTensor([[1.2, 0.0, 7.9, 3.8]]),
            id=FakeTensor([12]),
            cls=FakeTensor([0]),
            conf=FakeTensor([0.87]),
        )
        result = SimpleNamespace(
            masks=SimpleNamespace(data=FakeTensor(np.ones((1, 2, 4)))),
            boxes=[box],
        )

        def fake_track(image, **kwargs):
            captured.update(kwargs)
            return [result]

        segmenter = YoloSegmenter.__new__(YoloSegmenter)
        segmenter.wrapper = SimpleNamespace(track=fake_track)
        segmenter.requested_tracker = "ocsort"
        segmenter.config = SimpleNamespace(
            confidence=0.2, iou_threshold=0.4, image_size=640, classes=[0]
        )
        detections = segmenter._track_with_ultralytics(
            np.zeros((4, 8, 3), dtype=np.uint8)
        )
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].track_id, 12)
        self.assertEqual(detections[0].bbox_xyxy.tolist(), [1, 0, 7, 3])
        self.assertEqual(detections[0].mask.shape, (4, 8))
        self.assertTrue(captured["persist"])
        self.assertTrue(captured["tracker"].endswith("ocsort.yaml"))

    def test_model_suffix_follows_architecture(self):
        with patch("pathlib.Path.is_file", return_value=True):
            self.assertEqual(resolve_model_path("model.rknn", "amd64"), "model.pt")
            self.assertEqual(resolve_model_path("model.pt", "arm64"), "model.rknn")

    def test_raw_output_is_filtered_nms_and_masked(self):
        # 4 box + 1 class + 2 mask coefficients, in RKNN NCHW layout.
        prediction = np.array([[
            [4.0, 4.1], [4.0, 4.1], [4.0, 4.0], [4.0, 4.0],
            [0.9, 0.8], [10.0, 10.0], [0.0, 0.0],
        ]], dtype=np.float32)
        prototypes = np.zeros((1, 2, 2, 2), dtype=np.float32)
        prototypes[:, 0] = 1.0
        info = LetterboxInfo(1.0, 0, 0, 8, 8, (8, 8))
        detections = postprocess_segmentation(
            prediction, prototypes, info, confidence=0.5, iou_threshold=0.5
        )
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].bbox_xyxy.tolist(), [2, 2, 6, 6])
        self.assertTrue(detections[0].mask[5, 5])
        self.assertFalse(detections[0].mask[0, 0])

    def test_tracker_reuses_id(self):
        info = LetterboxInfo(1.0, 0, 0, 8, 8, (8, 8))
        prediction = np.array([[[4], [4], [4], [4], [0.9], [10], [0]]], np.float32)
        prototypes = np.ones((1, 2, 2, 2), np.float32)
        tracker = IoUTracker()
        first = postprocess_segmentation(prediction, prototypes, info, 0.5)
        second = postprocess_segmentation(prediction, prototypes, info, 0.5)
        tracker.update(first)
        tracker.update(second)
        self.assertEqual(first[0].track_id, second[0].track_id)

    def test_letterbox_mask_and_box_share_original_coordinates(self):
        # 8x4 original -> 8x8 input with two rows of padding at top/bottom.
        info = LetterboxInfo(1.0, 0, 2, 8, 4, (4, 8))
        prediction = np.array(
            [[[4], [4], [4], [4], [0.9], [10], [0]]], dtype=np.float32
        )
        prototypes = np.ones((1, 2, 2, 2), dtype=np.float32)
        detection = postprocess_segmentation(
            prediction, prototypes, info, confidence=0.5
        )[0]
        self.assertEqual(detection.bbox_xyxy.tolist(), [2, 0, 6, 3])
        rows, columns = np.nonzero(detection.mask)
        self.assertGreater(len(rows), 0)
        self.assertGreaterEqual(columns.min(), detection.bbox_xyxy[0])
        self.assertLessEqual(columns.max(), detection.bbox_xyxy[2])


if __name__ == "__main__":
    unittest.main()
