"""YOLO 实例分割与几何抓取估计的单帧处理管线。"""

from dataclasses import dataclass, field
from time import perf_counter
from typing import List, Optional, Sequence

import cv2
import numpy as np

from yolo_segmentation import (
    IoUTracker,
    letterbox,
    postprocess_segmentation,
    resolve_model_path,
    SegmentationDetection,
)

from grasp_pose_estimator import (
    GeometryConfig,
    GraspPose,
    GraspPoseEstimator,
    Intrinsics,
    depth_to_environment_cloud,
    estimate_grasp_pose,
    mask_to_point_cloud,
)


@dataclass
class InferenceConfig:
    model_path: str = "best_train_multi"
    confidence: float = 0.05
    classes: Optional[Sequence[int]] = None
    tracker: Optional[str] = None
    image_size: int = 640
    iou_threshold: float = 0.45
    class_names: Sequence[str] = ("bottle",)


@dataclass
class GraspCandidate:
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: np.ndarray
    mask: np.ndarray
    point_cloud: np.ndarray
    pose: GraspPose


@dataclass
class CandidateTiming:
    """单个检测实例的几何处理耗时，单位 ms。"""

    track_id: int
    point_cloud_ms: float
    pose_estimation_ms: float
    result_pack_ms: float

    @property
    def total_ms(self):
        return self.point_cloud_ms + self.pose_estimation_ms + self.result_pack_ms


@dataclass
class FrameTimings:
    """单帧管线各阶段耗时，均使用单调高精度时钟测量。"""

    inference_ms: float = 0.0
    mask_prepare_ms: float = 0.0
    environment_cloud_ms: float = 0.0
    table_normal_ms: float = 0.0
    candidates_ms: float = 0.0
    total_ms: float = 0.0
    candidate_details: List[CandidateTiming] = field(default_factory=list)

    def format_ms(self):
        details = ",".join(
            f"id={item.track_id}:cloud={item.point_cloud_ms:.2f}/"
            f"pose={item.pose_estimation_ms:.2f}/pack={item.result_pack_ms:.2f}"
            for item in self.candidate_details
        ) or "none"
        return (
            f"pipeline={self.total_ms:.2f}ms "
            f"yolo={self.inference_ms:.2f} mask={self.mask_prepare_ms:.2f} "
            f"env_cloud={self.environment_cloud_ms:.2f} "
            f"table={self.table_normal_ms:.2f} "
            f"objects={self.candidates_ms:.2f} details=[{details}]"
        )


@dataclass
class FrameResult:
    candidates: List[GraspCandidate] = field(default_factory=list)
    environment_cloud: Optional[np.ndarray] = None
    table_normal: Optional[np.ndarray] = None
    timings: FrameTimings = field(default_factory=FrameTimings)


def select_tracker(requested: Optional[str], backend: str):
    """Return normalized requested/effective trackers for the chosen backend."""
    requested = (requested or "iou").strip() or "iou"
    if requested.lower() == "iou" or backend == "rknn":
        return requested, "iou"
    return requested, requested


class YoloSegmenter:
    """根据 CPU 架构选择 PT/RKNN，后处理和跟踪由两端共享。"""

    def __init__(self, config: Optional[InferenceConfig] = None):
        self.config = config or InferenceConfig()
        if self.config.image_size <= 0:
            raise ValueError("image_size 必须是正整数")
        self.model_path = resolve_model_path(self.config.model_path)
        self.backend = "rknn" if self.model_path.endswith(".rknn") else "pt"
        self.requested_tracker, self.effective_tracker = select_tracker(
            self.config.tracker, self.backend
        )
        self.tracker = (
            IoUTracker() if self.effective_tracker.lower() == "iou" else None
        )
        if self.backend == "pt":
            import torch
            from ultralytics import YOLO

            self.torch = torch
            wrapper = YOLO(self.model_path)
            self.wrapper = wrapper
            self.model = wrapper.model.eval()
            # Obtain the same decoded raw head exported to RKNN (37x8400 here),
            # without Ultralytics predict/track/NMS/mask post-processing.
            self.model.model[-1].end2end = False
            self._names = wrapper.names
        else:
            try:
                from rknnlite.api import RKNNLite
            except ImportError:
                from rknn.api import RKNN as RKNNLite
            self.model = RKNNLite()
            if self.model.load_rknn(self.model_path) != 0:
                raise RuntimeError(f"RKNN 模型加载失败: {self.model_path}")
            if self.model.init_runtime() != 0:
                raise RuntimeError("RKNN 运行时初始化失败")
            self._names = dict(enumerate(self.config.class_names))

    @property
    def names(self):
        return self._names

    def track(self, color_image: np.ndarray):
        if self.effective_tracker.lower() != "iou":
            return self._track_with_ultralytics(color_image)

        input_image, info = letterbox(color_image, self.config.image_size)
        rgb = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
        if self.backend == "pt":
            tensor = self.torch.from_numpy(rgb.transpose(2, 0, 1)).unsqueeze(0)
            tensor = tensor.to(next(self.model.parameters()).device).float() / 255.0
            with self.torch.inference_mode():
                output = self.model(tensor)[0]
            prediction, prototypes = output
            prediction = prediction.detach().cpu().numpy()
            prototypes = prototypes.detach().cpu().numpy()
        else:
            outputs = self.model.inference(inputs=[rgb], data_format=["nhwc"])
            prediction = next(item for item in outputs if np.asarray(item).ndim == 3)
            prototypes = next(item for item in outputs if np.asarray(item).ndim == 4)
        detections = postprocess_segmentation(
            prediction, prototypes, info, self.config.confidence,
            self.config.classes, self.config.iou_threshold,
        )
        self.tracker.update(detections)
        return detections

    def _track_with_ultralytics(self, color_image: np.ndarray):
        """Run a tracker supported by Ultralytics and normalize its Results."""
        tracker = self.requested_tracker
        aliases = {
            "bytetrack": "bytetrack.yaml",
            "botsort": "botsort.yaml",
            "ocsort": "ocsort.yaml",
        }
        tracker = aliases.get(tracker.lower(), tracker)
        results = self.wrapper.track(
            color_image,
            persist=True,
            verbose=False,
            conf=self.config.confidence,
            iou=self.config.iou_threshold,
            imgsz=self.config.image_size,
            classes=self.config.classes,
            tracker=tracker,
        )
        detections = []
        height, width = color_image.shape[:2]
        for result in results:
            masks = (
                result.masks.data.detach().cpu().numpy()
                if result.masks is not None
                else None
            )
            for index, box in enumerate(result.boxes):
                if masks is None or index >= len(masks):
                    continue
                mask = masks[index]
                if mask.shape != (height, width):
                    mask = cv2.resize(
                        mask, (width, height), interpolation=cv2.INTER_NEAREST
                    )
                coordinates = box.xyxy[0].detach().cpu().numpy()
                coordinates[[0, 2]] = np.clip(coordinates[[0, 2]], 0, width - 1)
                coordinates[[1, 3]] = np.clip(coordinates[[1, 3]], 0, height - 1)
                track_id = int(box.id[0]) if box.id is not None else -1
                detections.append(
                    SegmentationDetection(
                        track_id=track_id,
                        class_id=int(box.cls[0]),
                        confidence=float(box.conf[0]),
                        bbox_xyxy=coordinates.astype(np.int32),
                        mask=mask > 0.5,
                    )
                )
        return detections


class BottleGraspPipeline:
    """接收一帧对齐 RGB-D 数据，输出结构化抓取候选。"""

    def __init__(
        self,
        segmenter: YoloSegmenter,
        geometry_config: Optional[GeometryConfig] = None,
    ):
        self.segmenter = segmenter
        self.estimator = GraspPoseEstimator(geometry_config)

    def process(
        self,
        color_image: np.ndarray,
        depth_image: np.ndarray,
        intrinsics: Intrinsics,
    ) -> FrameResult:
        frame_start = perf_counter()
        timings = FrameTimings()
        if color_image.shape[:2] != depth_image.shape:
            raise ValueError("彩色图和深度图尺寸必须一致")

        stage_start = perf_counter()
        results = self.segmenter.track(color_image)
        timings.inference_ms = (perf_counter() - stage_start) * 1000.0

        stage_start = perf_counter()
        object_mask = np.zeros(depth_image.shape, dtype=bool)
        for detection in results:
            object_mask |= detection.mask
        timings.mask_prepare_ms = (perf_counter() - stage_start) * 1000.0

        config = self.estimator.config
        stage_start = perf_counter()
        environment_cloud = depth_to_environment_cloud(
            depth_image, object_mask, intrinsics, config
        )
        timings.environment_cloud_ms = (
            perf_counter() - stage_start
        ) * 1000.0
        stage_start = perf_counter()
        table_normal = self.estimator.update_table_normal(environment_cloud)
        timings.table_normal_ms = (perf_counter() - stage_start) * 1000.0
        candidates = []

        candidates_start = perf_counter()
        for detection in results:
            mask = detection.mask
            candidate_start = perf_counter()
            point_cloud = mask_to_point_cloud(
                depth_image, mask, intrinsics, config
            )
            point_cloud_done = perf_counter()
            pose = estimate_grasp_pose(
                point_cloud, table_normal, config
            )
            pose_done = perf_counter()
            if pose is None:
                timings.candidate_details.append(
                    CandidateTiming(
                        track_id=detection.track_id,
                        point_cloud_ms=(point_cloud_done - candidate_start) * 1000.0,
                        pose_estimation_ms=(pose_done - point_cloud_done) * 1000.0,
                        result_pack_ms=0.0,
                    )
                )
                continue
            class_id = detection.class_id
            candidates.append(
                GraspCandidate(
                    track_id=detection.track_id,
                    class_id=class_id,
                    class_name=str(self.segmenter.names[class_id]),
                    confidence=detection.confidence,
                    bbox_xyxy=detection.bbox_xyxy,
                    mask=mask,
                    point_cloud=point_cloud,
                    pose=pose,
                )
            )
            pack_done = perf_counter()
            timings.candidate_details.append(
                CandidateTiming(
                    track_id=detection.track_id,
                    point_cloud_ms=(point_cloud_done - candidate_start) * 1000.0,
                    pose_estimation_ms=(pose_done - point_cloud_done) * 1000.0,
                    result_pack_ms=(pack_done - pose_done) * 1000.0,
                )
            )

        timings.candidates_ms = (perf_counter() - candidates_start) * 1000.0
        timings.total_ms = (perf_counter() - frame_start) * 1000.0
        return FrameResult(
            candidates=candidates,
            environment_cloud=environment_cloud,
            table_normal=table_normal,
            timings=timings,
        )
