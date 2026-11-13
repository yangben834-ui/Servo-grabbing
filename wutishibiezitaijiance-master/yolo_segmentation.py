"""PT/RKNN YOLO segmentation inference with shared NumPy post-processing."""

from dataclasses import dataclass
from pathlib import Path
import platform
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class SegmentationDetection:
    track_id: int
    class_id: int
    confidence: float
    bbox_xyxy: np.ndarray
    mask: np.ndarray


@dataclass
class LetterboxInfo:
    scale: float
    pad_x: int
    pad_y: int
    resized_width: int
    resized_height: int
    original_shape: Tuple[int, int]


def resolve_model_path(model_path: str, machine: Optional[str] = None) -> str:
    """Select the weight suffix from the CPU architecture, keeping the stem."""
    arch = (machine or platform.machine()).lower()
    suffixes = {
        "x86_64": ".pt",
        "amd64": ".pt",
        "aarch64": ".rknn",
        "arm64": ".rknn",
    }
    if arch not in suffixes:
        raise RuntimeError(f"不支持的 CPU 架构: {arch}")
    selected = str(Path(model_path).with_suffix(suffixes[arch]))
    if not Path(selected).is_file():
        raise FileNotFoundError(f"当前平台模型不存在: {selected}")
    return selected


def letterbox(image: np.ndarray, size: int) -> Tuple[np.ndarray, LetterboxInfo]:
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    new_width = max(1, int(round(width * scale)))
    new_height = max(1, int(round(height * scale)))
    pad_x = (size - new_width) // 2
    pad_y = (size - new_height) // 2
    resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
    output = np.full((size, size, 3), 114, dtype=np.uint8)
    output[pad_y : pad_y + new_height, pad_x : pad_x + new_width] = resized
    return output, LetterboxInfo(
        scale, pad_x, pad_y, new_width, new_height, (height, width)
    )


def _nms(boxes: np.ndarray, scores: np.ndarray, threshold: float) -> np.ndarray:
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        index = int(order[0])
        keep.append(index)
        if order.size == 1:
            break
        rest = order[1:]
        left_top = np.maximum(boxes[index, :2], boxes[rest, :2])
        right_bottom = np.minimum(boxes[index, 2:], boxes[rest, 2:])
        intersection = np.prod(np.maximum(right_bottom - left_top, 0), axis=1)
        area_a = np.prod(np.maximum(boxes[index, 2:] - boxes[index, :2], 0))
        area_b = np.prod(np.maximum(boxes[rest, 2:] - boxes[rest, :2], 0), axis=1)
        iou = intersection / np.maximum(area_a + area_b - intersection, 1e-7)
        order = rest[iou <= threshold]
    return np.asarray(keep, dtype=np.int32)


def _restore_mask(mask: np.ndarray, info: LetterboxInfo) -> np.ndarray:
    """Remove letterbox padding from an input-resolution binary mask."""
    cropped = mask[
        info.pad_y : info.pad_y + info.resized_height,
        info.pad_x : info.pad_x + info.resized_width,
    ]
    height, width = info.original_shape
    return cv2.resize(
        cropped.astype(np.uint8),
        (width, height),
        interpolation=cv2.INTER_NEAREST,
    ).astype(bool)


def postprocess_segmentation(
    prediction: np.ndarray,
    prototypes: np.ndarray,
    info: LetterboxInfo,
    confidence: float,
    classes: Optional[Sequence[int]] = None,
    iou_threshold: float = 0.45,
) -> List[SegmentationDetection]:
    """Decode both PT and RKNN raw YOLO segmentation outputs."""
    prediction = np.asarray(prediction)
    prototypes = np.asarray(prototypes)
    if prediction.ndim == 3:
        prediction = prediction[0]
    if prototypes.ndim == 4:
        prototypes = prototypes[0]
    if prediction.ndim != 2 or prototypes.ndim != 3:
        raise ValueError("YOLO 输出维度不正确")
    mask_channels = prototypes.shape[0]
    minimum_channels = 5 + mask_channels
    # Exported heads are normally [channels, candidates]; also accept the
    # already-transposed [candidates, channels] form.
    if prediction.shape[0] >= minimum_channels and (
        prediction.shape[1] < minimum_channels
        or prediction.shape[0] <= prediction.shape[1]
    ):
        prediction = prediction.T
    class_count = prediction.shape[1] - 4 - mask_channels
    if class_count <= 0:
        raise ValueError("无法从 YOLO 输出推断类别数")

    class_scores = prediction[:, 4 : 4 + class_count]
    class_ids = np.argmax(class_scores, axis=1)
    scores = class_scores[np.arange(len(prediction)), class_ids]
    selected = scores >= confidence
    if classes is not None:
        selected &= np.isin(class_ids, np.asarray(classes))
    prediction, scores, class_ids = prediction[selected], scores[selected], class_ids[selected]
    if not len(prediction):
        return []

    xywh = prediction[:, :4]
    boxes = np.column_stack(
        (xywh[:, 0] - xywh[:, 2] / 2, xywh[:, 1] - xywh[:, 3] / 2,
         xywh[:, 0] + xywh[:, 2] / 2, xywh[:, 1] + xywh[:, 3] / 2)
    )
    keep = []
    for class_id in np.unique(class_ids):
        indices = np.flatnonzero(class_ids == class_id)
        keep.extend(indices[_nms(boxes[indices], scores[indices], iou_threshold)])
    keep = np.asarray(sorted(keep, key=lambda i: scores[i], reverse=True), dtype=np.int32)
    boxes, scores, class_ids = boxes[keep], scores[keep], class_ids[keep]
    coefficients = prediction[keep, 4 + class_count :]

    proto_height, proto_width = prototypes.shape[1:]
    logits = coefficients.astype(np.float32) @ prototypes.astype(np.float32).reshape(mask_channels, -1)
    masks = logits.reshape(-1, proto_height, proto_width)
    detections = []
    original_height, original_width = info.original_shape
    # One resized dimension always fills the square network input exactly.
    input_size = max(
        info.pad_x + info.resized_width,
        info.pad_y + info.resized_height,
    )
    for index, (box, mask) in enumerate(zip(boxes, masks)):
        # Upsample logits first and crop second. Cropping at prototype
        # resolution and then resizing lets interpolation leak outside boxes.
        input_mask = cv2.resize(
            mask, (input_size, input_size), interpolation=cv2.INTER_LINEAR
        ) > 0.0
        x1 = max(0, min(input_size, int(np.ceil(box[0]))))
        y1 = max(0, min(input_size, int(np.ceil(box[1]))))
        x2 = max(0, min(input_size, int(np.ceil(box[2]))))
        y2 = max(0, min(input_size, int(np.ceil(box[3]))))
        cropped_mask = np.zeros_like(input_mask)
        cropped_mask[y1:y2, x1:x2] = input_mask[y1:y2, x1:x2]
        restored_box = box.copy()
        restored_box[[0, 2]] = (restored_box[[0, 2]] - info.pad_x) / info.scale
        restored_box[[1, 3]] = (restored_box[[1, 3]] - info.pad_y) / info.scale
        restored_box[[0, 2]] = np.clip(restored_box[[0, 2]], 0, original_width - 1)
        restored_box[[1, 3]] = np.clip(restored_box[[1, 3]], 0, original_height - 1)
        restored_box = restored_box.astype(np.int32)
        restored_mask = _restore_mask(cropped_mask, info)
        # Keep rounding at the final resize from creating one-pixel spillover.
        box_mask = np.zeros_like(restored_mask)
        rx1, ry1, rx2, ry2 = restored_box
        box_mask[ry1 : ry2 + 1, rx1 : rx2 + 1] = True
        restored_mask &= box_mask
        detections.append(SegmentationDetection(
            track_id=-1,
            class_id=int(class_ids[index]),
            confidence=float(scores[index]),
            bbox_xyxy=restored_box,
            mask=restored_mask,
        ))
    return detections


class IoUTracker:
    """Small dependency-free tracker used identically by both inference backends."""

    def __init__(self, threshold: float = 0.3):
        self.threshold = threshold
        self.previous = {}
        self.next_id = 1

    def update(self, detections: List[SegmentationDetection]) -> None:
        available = set(self.previous)
        current = {}
        for detection in detections:
            best_id, best_iou = None, self.threshold
            for track_id in available:
                old_class, old_box = self.previous[track_id]
                if old_class != detection.class_id:
                    continue
                pair = np.stack((detection.bbox_xyxy, old_box)).astype(np.float32)
                intersection = np.prod(np.maximum(np.minimum(pair[0, 2:], pair[1, 2:]) -
                                                   np.maximum(pair[0, :2], pair[1, :2]), 0))
                areas = np.prod(np.maximum(pair[:, 2:] - pair[:, :2], 0), axis=1)
                iou = intersection / max(float(areas.sum() - intersection), 1e-7)
                if iou > best_iou:
                    best_id, best_iou = track_id, iou
            if best_id is None:
                best_id = self.next_id
                self.next_id += 1
            else:
                available.remove(best_id)
            detection.track_id = best_id
            current[best_id] = (detection.class_id, detection.bbox_xyxy.copy())
        self.previous = current
