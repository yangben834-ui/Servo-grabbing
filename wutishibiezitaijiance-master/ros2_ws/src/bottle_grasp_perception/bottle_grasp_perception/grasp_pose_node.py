"""Synchronized RGB-D bottle grasp perception node for ROS 2."""

from array import array
import copy
import math
from time import perf_counter
from typing import Iterable

import cv2
import message_filters
import numpy as np
import rclpy
from bottle_grasp_interfaces.msg import GraspCandidatePoints as GraspCandidateMsg
from bottle_grasp_interfaces.msg import GraspCandidateArray
from bottle_grasp_interfaces.msg import SelectedCandidate
from bottle_grasp_interfaces.srv import GetGraspCandidates
from geometry_msgs.msg import Point, Point32, TransformStamped
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from bottle_grasp_pipeline import BottleGraspPipeline, InferenceConfig, YoloSegmenter
from grasp_pose_estimator import GeometryConfig
from grasp_visualization import draw_frame_result


def rotation_matrix_to_quaternion(rotation: np.ndarray):
    """Convert a proper 3x3 rotation matrix to normalized (x, y, z, w)."""
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.asarray((x, y, z, w), dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return quaternion


def _unpack_image(message: Image, dtype, channels=1):
    """Read a possibly row-padded ROS Image without depending on cv_bridge."""
    dtype = np.dtype(dtype)
    bytes_per_pixel = dtype.itemsize * channels
    packed_step = message.width * bytes_per_pixel
    if message.step < packed_step:
        raise ValueError(
            f"Invalid image step {message.step}; expected at least {packed_step}"
        )
    expected_size = message.step * message.height
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < expected_size:
        raise ValueError(
            f"Image data is truncated: {raw.size} bytes, expected {expected_size}"
        )
    packed = raw[:expected_size].reshape(message.height, message.step)[
        :, :packed_step
    ].copy()
    byte_order = ">" if message.is_bigendian else "<"
    pixels = packed.view(dtype.newbyteorder(byte_order))
    shape = (message.height, message.width)
    if channels > 1:
        shape += (channels,)
    # astype converts non-native byte order and detaches the message buffer.
    return pixels.reshape(shape).astype(dtype, copy=False)


def image_to_bgr(message: Image):
    encoding = message.encoding.lower()
    if encoding == "bgr8":
        return _unpack_image(message, np.uint8, 3)
    if encoding == "rgb8":
        return _unpack_image(message, np.uint8, 3)[..., ::-1].copy()
    if encoding == "bgra8":
        return _unpack_image(message, np.uint8, 4)[..., :3].copy()
    if encoding == "rgba8":
        return _unpack_image(message, np.uint8, 4)[..., (2, 1, 0)].copy()
    if encoding in ("mono8", "8uc1"):
        gray = _unpack_image(message, np.uint8)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(
        f"Unsupported color encoding {message.encoding}; "
        "expected bgr8, rgb8, bgra8, rgba8 or mono8"
    )


def bgr_to_image(image: np.ndarray, header):
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Debug image must be an HxWx3 uint8 BGR array")
    contiguous = np.ascontiguousarray(image)
    message = Image()
    message.header = header
    message.height, message.width = contiguous.shape[:2]
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = message.width * 3
    # ROS 2 Humble 对 bytes 会逐元素做两轮 Python 校验；array('B') 可走快速路径。
    message.data = array("B", contiguous.tobytes())
    return message


def points_to_cloud(points: np.ndarray, max_points: int = 1024):
    """Return up to max_points float32 XYZ points, sampled across the input order."""
    if isinstance(max_points, bool) or not isinstance(max_points, int) or max_points < 1:
        raise ValueError("max_points must be a positive integer")
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Object point cloud must be an Nx3 array")
    if len(points) > max_points:
        indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
        points = points[indices]
    return [Point32(x=float(x), y=float(y), z=float(z)) for x, y, z in points]


class BottleGraspNode(Node):
    def __init__(self):
        super().__init__("bottle_grasp_perception")
        self._declare_parameters()
        self.latest_result = None
        self.frame_count = 0
        # 圆柱拟合的上一次状态; None 表示还没判过, 用于只在翻转时打日志
        self.cylinder_fit_ok = None
        self.selected_track_id = None
        self.target_missing_since_ns = None
        self.tf_broadcaster = (
            TransformBroadcaster(self)
            if self.get_parameter("publish_tf").value
            else None
        )

        class_ids_text = self.get_parameter("class_ids").value.strip()
        class_ids = (
            [int(value.strip()) for value in class_ids_text.split(",")]
            if class_ids_text
            else None
        )
        tracker = self.get_parameter("tracker").value.strip() or None
        inference_config = InferenceConfig(
            model_path=self.get_parameter("model_path").value,
            confidence=self.get_parameter("confidence").value,
            image_size=self.get_parameter("model_input_size").value,
            classes=class_ids,
            tracker=tracker,
        )
        geometry_config = GeometryConfig(
            min_point_count=self.get_parameter("min_point_count").value,
            max_point_count=self.get_parameter("max_point_count").value,
            depth_outlier_threshold=self.get_parameter("depth_outlier_threshold").value,
            environment_point_step=self.get_parameter("environment_point_step").value,
            environment_max_depth=self.get_parameter("environment_max_depth").value,
            table_ransac_iterations=self.get_parameter("table_ransac_iterations").value,
            table_distance_threshold=self.get_parameter("table_distance_threshold").value,
            table_min_inliers=self.get_parameter("table_min_inliers").value,
            table_max_sample_points=self.get_parameter("table_max_sample_points").value,
            table_normal_smoothing=self.get_parameter("table_normal_smoothing").value,
            upright_angle_threshold_deg=self.get_parameter("upright_angle_threshold_deg").value,
            use_cylinder_axis=self.get_parameter("use_cylinder_axis").value,
            cylinder_ransac_iterations=self.get_parameter("cylinder_ransac_iterations").value,
            cylinder_max_sample_points=self.get_parameter("cylinder_max_sample_points").value,
            cylinder_distance_threshold=self.get_parameter("cylinder_distance_threshold").value,
            cylinder_min_inlier_ratio=self.get_parameter("cylinder_min_inlier_ratio").value,
            cylinder_min_radius=self.get_parameter("cylinder_min_radius").value,
            cylinder_max_radius=self.get_parameter("cylinder_max_radius").value,
            cylinder_axial_trim=self.get_parameter("cylinder_axial_trim").value,
        )
        segmenter = YoloSegmenter(inference_config)
        self.get_logger().info(
            f"YOLO backend={segmenter.backend}, model={segmenter.model_path}"
            f"tracker_requested={segmenter.requested_tracker}, "
            f"tracker_effective={segmenter.effective_tracker}"
        )
        self.pipeline = BottleGraspPipeline(segmenter, geometry_config)

        self.candidates_pub = self.create_publisher(
            GraspCandidateArray, "~/candidates", 10
        )
        self.selected_candidate_pub = self.create_publisher(
            SelectedCandidate, "~/selected_candidate", 10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, "~/markers", 10
        )
        self.debug_pub = self.create_publisher(
            Image, "~/debug_image", qos_profile_sensor_data
        )
        self.create_service(
            GetGraspCandidates, "~/get_candidates", self._get_candidates
        )

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        self.color_sub = message_filters.Subscriber(
            self, Image, color_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, info_topic, qos_profile=qos_profile_sensor_data
        )
        queue_size = self.get_parameter("sync_queue_size").value
        if self.get_parameter("approximate_sync").value:
            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [self.color_sub, self.depth_sub, self.info_sub],
                queue_size,
                self.get_parameter("sync_slop").value,
            )
        else:
            self.synchronizer = message_filters.TimeSynchronizer(
                [self.color_sub, self.depth_sub, self.info_sub], queue_size
            )
        self.synchronizer.registerCallback(self._image_callback)
        self.get_logger().info(
            f"Listening: color={color_topic}, depth={depth_topic}, info={info_topic}"
        )

    def _declare_parameters(self):
        parameters = {
            "model_path": "best_train_multi",
            "confidence": 0.05,
            "model_input_size": 640,
            "class_ids": "",
            "tracker": "",
            "color_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/depth/camera_info",
            "approximate_sync": True,
            "sync_queue_size": 10,
            "sync_slop": 0.05,
            "depth_16u_scale": 0.001,
            "frame_id_override": "",
            "publish_debug_image": True,
            "publish_markers": True,
            "publish_tf": False,
            "marker_axis_length": 0.06,
            "max_result_age": 0.5,
            "target_lost_timeout": 10.0,
            "timing_log_enabled": True,
            "timing_log_every_n": 1,
            "min_point_count": 100,
            "max_point_count": 6000,
            "depth_outlier_threshold": 0.05,
            "environment_point_step": 4,
            "environment_max_depth": 1.0,
            "table_ransac_iterations": 80,
            "table_distance_threshold": 0.008,
            "table_min_inliers": 500,
            "table_max_sample_points": 8000,
            "table_normal_smoothing": 0.2,
            "upright_angle_threshold_deg": 25.0,
            "use_cylinder_axis": False,
            "cylinder_ransac_iterations": 300,
            "cylinder_max_sample_points": 1500,
            "cylinder_distance_threshold": 0.004,
            "cylinder_min_inlier_ratio": 0.35,
            "cylinder_min_radius": 0.010,
            "cylinder_max_radius": 0.060,
            "cylinder_axial_trim": 0.25,
        }
        self.declare_parameter(
            "publish_point_count", 1024,
            ParameterDescriptor(
                description="Maximum object points published per candidate; no padding.",
                integer_range=[IntegerRange(from_value=1, to_value=2147483647, step=1)],
            ),
        )
        for name, value in parameters.items():
            self.declare_parameter(name, value)

    def _depth_to_meters(self, message: Image):
        if message.encoding.upper() in ("16UC1", "MONO16"):
            depth = _unpack_image(message, np.uint16)
            return depth.astype(np.float32) * self.get_parameter("depth_16u_scale").value
        if message.encoding.upper() == "32FC1":
            return _unpack_image(message, np.float32)
        raise ValueError(
            f"Unsupported depth encoding {message.encoding}; expected 16UC1 or 32FC1"
        )

    def _image_callback(self, color_msg, depth_msg, info_msg):
        callback_start = perf_counter()
        try:
            stage_start = perf_counter()
            color = image_to_bgr(color_msg)
            depth = self._depth_to_meters(depth_msg)
            if color.shape[:2] != depth.shape:
                raise ValueError(
                    f"RGB/depth size mismatch: {color.shape[:2]} vs {depth.shape}"
                )
            if info_msg.width != color.shape[1] or info_msg.height != color.shape[0]:
                raise ValueError("CameraInfo dimensions do not match the synchronized images")
            intrinsics = (
                float(info_msg.k[0]), float(info_msg.k[4]),
                float(info_msg.k[2]), float(info_msg.k[5]),
            )
            conversion_ms = (perf_counter() - stage_start) * 1000.0

            result = self.pipeline.process(color, depth, intrinsics)
            self._log_cylinder_fit(result.candidates)
            frame_id = (
                self.get_parameter("frame_id_override").value
                or color_msg.header.frame_id
            )
            stage_start = perf_counter()
            output = self._make_result_message(result.candidates, color_msg.header, frame_id)
            self.latest_result = output
            self.candidates_pub.publish(output)
            self._update_selected_target(output.candidates)
            self._publish_selected_candidate(output.candidates)
            result_publish_ms = (perf_counter() - stage_start) * 1000.0

            marker_ms = 0.0
            if self.get_parameter("publish_markers").value:
                stage_start = perf_counter()
                self.markers_pub.publish(self._make_markers(output))
                marker_ms = (perf_counter() - stage_start) * 1000.0

            debug_ms = 0.0
            debug_draw_ms = 0.0
            debug_pack_ms = 0.0
            debug_publish_ms = 0.0
            if self.get_parameter("publish_debug_image").value:
                stage_start = perf_counter()
                debug = draw_frame_result(color, result, intrinsics)
                draw_done = perf_counter()
                debug_msg = bgr_to_image(debug, color_msg.header)
                debug_msg.header.frame_id = frame_id
                pack_done = perf_counter()
                self.debug_pub.publish(debug_msg)
                publish_done = perf_counter()
                debug_draw_ms = (draw_done - stage_start) * 1000.0
                debug_pack_ms = (pack_done - draw_done) * 1000.0
                debug_publish_ms = (publish_done - pack_done) * 1000.0
                debug_ms = (publish_done - stage_start) * 1000.0

            tf_ms = 0.0
            if self.tf_broadcaster is not None:
                stage_start = perf_counter()
                self._broadcast_transforms(output)
                tf_ms = (perf_counter() - stage_start) * 1000.0

            callback_ms = (perf_counter() - callback_start) * 1000.0
            self.frame_count += 1
            log_every_n = max(
                1, self.get_parameter("timing_log_every_n").value
            )
            if (
                self.get_parameter("timing_log_enabled").value
                and self.frame_count % log_every_n == 0
            ):
                environment_count = (
                    len(result.environment_cloud)
                    if result.environment_cloud is not None
                    else 0
                )
                self.get_logger().info(
                    f"TIMING frame={self.frame_count} "
                    f"callback={callback_ms:.2f}ms convert={conversion_ms:.2f} "
                    f"{result.timings.format_ms()} "
                    f"msg_publish={result_publish_ms:.2f} "
                    f"markers={marker_ms:.2f} debug={debug_ms:.2f} "
                    f"(draw={debug_draw_ms:.2f}/pack={debug_pack_ms:.2f}/"
                    f"publish={debug_publish_ms:.2f}) "
                    f"tf={tf_ms:.2f} candidates={len(result.candidates)} "
                    f"env_points={environment_count}"
                )
        except Exception as error:
            self.get_logger().error(f"Frame processing failed: {error}")

    def _log_cylinder_fit(self, candidates):
        """圆柱拟合状态翻转时才打日志, 免得逐帧刷屏。"""
        if not self.get_parameter("use_cylinder_axis").value:
            return
        for candidate in candidates:
            ok = candidate.pose.radius is not None
            if ok == self.cylinder_fit_ok:
                continue
            if ok:
                stage = "已生效" if self.cylinder_fit_ok is None else "恢复"
                self.get_logger().info(
                    f"圆柱轴拟合: {stage} r={candidate.pose.radius * 1000:.1f}mm"
                )
            else:
                self.get_logger().warn(
                    "圆柱轴拟合: 失败, 抓取点退回可见表面质心 "
                    f"({candidate.pose.fit_reason})"
                )
            self.cylinder_fit_ok = ok

    def _make_result_message(self, candidates, source_header, frame_id):
        array = GraspCandidateArray()
        array.header = copy.deepcopy(source_header)
        array.header.frame_id = frame_id
        array.perception_valid = bool(candidates)
        for candidate in candidates:
            message = GraspCandidateMsg()
            message.header = array.header
            message.track_id = candidate.track_id
            message.class_id = candidate.class_id
            message.class_name = candidate.class_name
            message.confidence = candidate.confidence
            message.grasp_mode = candidate.pose.mode
            message.axis_table_angle_deg = candidate.pose.axis_table_angle_deg
            message.object_points = points_to_cloud(
                candidate.point_cloud, self.get_parameter("publish_point_count").value
            )
            message.point_count = len(message.object_points)
            point_ratio = min(
                1.0,
                len(candidate.point_cloud) / float(self.pipeline.estimator.config.max_point_count),
            )
            message.quality = candidate.confidence * point_ratio
            message.pose.position.x = float(candidate.pose.position[0])
            message.pose.position.y = float(candidate.pose.position[1])
            message.pose.position.z = float(candidate.pose.position[2])
            quaternion = rotation_matrix_to_quaternion(candidate.pose.rotation)
            message.pose.orientation.x = float(quaternion[0])
            message.pose.orientation.y = float(quaternion[1])
            message.pose.orientation.z = float(quaternion[2])
            message.pose.orientation.w = float(quaternion[3])
            array.candidates.append(message)
        return array

    def _make_markers(self, result):
        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
        length = self.get_parameter("marker_axis_length").value
        colors = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        for candidate_index, candidate in enumerate(result.candidates):
            rotation = self._quaternion_to_rotation(candidate.pose.orientation)
            center = np.asarray((candidate.pose.position.x, candidate.pose.position.y, candidate.pose.position.z))
            for axis_index, color in enumerate(colors):
                marker = Marker()
                marker.header = result.header
                marker.ns = "grasp_axes"
                marker.id = candidate_index * 4 + axis_index
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.scale.x, marker.scale.y, marker.scale.z = 0.008, 0.015, 0.02
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 1.0)
                endpoint = center + rotation[:, axis_index] * length
                marker.points = [Point(x=float(center[0]), y=float(center[1]), z=float(center[2])), Point(x=float(endpoint[0]), y=float(endpoint[1]), z=float(endpoint[2]))]
                marker_array.markers.append(marker)
            text = Marker()
            text.header = result.header
            text.ns = "grasp_labels"
            text.id = candidate_index * 4 + 3
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = copy.deepcopy(candidate.pose.position)
            text.pose.position.z += 0.05
            text.scale.z = 0.025
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f"ID {candidate.track_id}: {candidate.grasp_mode} q={candidate.quality:.2f}"
            marker_array.markers.append(text)
        return marker_array

    @staticmethod
    def _quaternion_to_rotation(quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        return np.asarray((
            (1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w),
            (2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w),
            (2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y),
        ))

    def _broadcast_transforms(self, result):
        transforms = []
        for candidate in result.candidates:
            transform = TransformStamped()
            transform.header = result.header
            transform.child_frame_id = f"grasp_candidate_{candidate.track_id}"
            transform.transform.translation.x = candidate.pose.position.x
            transform.transform.translation.y = candidate.pose.position.y
            transform.transform.translation.z = candidate.pose.position.z
            transform.transform.rotation = candidate.pose.orientation
            transforms.append(transform)
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    def _update_selected_target(self, candidates):
        """Keep one target locked until it has been absent for the timeout."""
        candidate_ids = {candidate.track_id for candidate in candidates}
        if self.selected_track_id is None:
            if candidate_ids:
                self.selected_track_id = min(candidate_ids)
                self.target_missing_since_ns = None
                self.get_logger().info(
                    f"Selected target track_id={self.selected_track_id}"
                )
            return

        if self.selected_track_id in candidate_ids:
            self.target_missing_since_ns = None
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.target_missing_since_ns is None:
            self.target_missing_since_ns = now_ns
            self.get_logger().warn(
                f"Target track_id={self.selected_track_id} is missing"
            )
            return

        timeout = self.get_parameter("target_lost_timeout").value
        missing_seconds = (now_ns - self.target_missing_since_ns) * 1e-9
        if missing_seconds < timeout or not candidate_ids:
            return

        previous_track_id = self.selected_track_id
        self.selected_track_id = min(candidate_ids)
        self.target_missing_since_ns = None
        self.get_logger().warn(
            f"Target track_id={previous_track_id} was missing for "
            f"{missing_seconds:.3f}s; selected track_id={self.selected_track_id}"
        )

    def _publish_selected_candidate(self, candidates):
        """Publish the locked target only while it is visible in this frame."""
        selected = next(
            (
                candidate
                for candidate in candidates
                if candidate.track_id == self.selected_track_id
            ),
            None,
        )
        if selected is None:
            return
        message = SelectedCandidate()
        message.header = selected.header
        message.track_id = selected.track_id
        message.grasp_pose = selected.pose
        message.grasp_mode = selected.grasp_mode
        self.selected_candidate_pub.publish(message)

    def _get_candidates(self, request, response):
        if self.latest_result is None:
            response.success = False
            response.message = "No perception result has been received"
            return response
        stamp = self.latest_result.header.stamp
        stamp_seconds = stamp.sec + stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds
        max_age = self.get_parameter("max_result_age").value
        if max_age > 0.0 and age > max_age:
            response.success = False
            response.message = f"Latest result is stale ({age:.3f}s)"
            return response
        response.result = copy.deepcopy(self.latest_result)
        if request.track_id >= 0:
            requested_track_id = request.track_id
        else:
            requested_track_id = self.selected_track_id
        if requested_track_id is not None:
            response.result.candidates = [
                item for item in response.result.candidates
                if item.track_id == requested_track_id
            ]
            response.result.perception_valid = bool(response.result.candidates)
        else:
            response.result.candidates = []
            response.result.perception_valid = False
        response.success = response.result.perception_valid
        if response.success:
            response.message = "ok"
        elif request.track_id < 0 and self.selected_track_id is not None:
            response.message = (
                f"Selected target track_id={self.selected_track_id} is currently missing"
            )
        else:
            response.message = "No matching valid candidate"
        return response


def main(args: Iterable[str] = None):
    rclpy.init(args=args)
    node = None
    try:
        node = BottleGraspNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
