"""Bottle 抓取结果的 OpenCV/Open3D 调试可视化。"""

import colorsys
import cv2
import numpy as np

AXIS_LENGTH = 0.06
ENVIRONMENT_COLOR = (128, 128, 128)


def get_track_color(track_id):
    hue = (track_id * 0.618033988749895) % 1.0
    red, green, blue = colorsys.hsv_to_rgb(hue, 0.75, 1.0)
    return tuple(int(channel * 255) for channel in (blue, green, red))


def project_point(point, intrinsics):
    fx, fy, cx, cy = intrinsics
    if point[2] <= 0:
        return None
    return int(round(fx * point[0] / point[2] + cx)), int(
        round(fy * point[1] / point[2] + cy)
    )


def draw_grasp_pose(image, center, rotation, intrinsics):
    origin = project_point(center, intrinsics)
    if origin is None:
        return
    for axis_id, color in enumerate(
        ((0, 0, 255), (0, 255, 0), (255, 0, 0))
    ):
        endpoint = project_point(
            center + rotation[:, axis_id] * AXIS_LENGTH, intrinsics
        )
        if endpoint is not None:
            cv2.arrowedLine(image, origin, endpoint, color, 2, tipLength=0.2)
    closing, lateral = rotation[:, 0], rotation[:, 1]
    for side in (-1.0, 1.0):
        finger_center = center + side * AXIS_LENGTH * 0.55 * closing
        start = project_point(
            finger_center - AXIS_LENGTH * 0.325 * lateral, intrinsics
        )
        end = project_point(
            finger_center + AXIS_LENGTH * 0.325 * lateral, intrinsics
        )
        if start is not None and end is not None:
            cv2.line(image, start, end, (0, 255, 255), 2)
    cv2.circle(image, origin, 5, (255, 255, 255), -1)


def draw_frame_result(image, frame_result, intrinsics):
    """在图像副本上绘制候选，不修改相机输入帧。"""
    output = image.copy()
    height, width = output.shape[:2]
    for candidate in frame_result.candidates:
        color = get_track_color(candidate.track_id)
        x1, y1, x2, y2 = candidate.bbox_xyxy
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(width - 1, x2), min(height - 1, y2)
        output[candidate.mask] = (
            output[candidate.mask] * 0.6
            + np.asarray(color, dtype=np.uint8) * 0.4
        ).astype(np.uint8)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        draw_grasp_pose(
            output, candidate.pose.position, candidate.pose.rotation, intrinsics
        )
        cv2.putText(
            output,
            f"ID:{candidate.track_id} {candidate.class_name[:3]} "
            f"{candidate.confidence*100:.0f}% {candidate.pose.mode}:"
            f"{candidate.pose.axis_table_angle_deg:.0f}deg "
            f"{candidate.pose.position[2]:.2f}m",
            (x1, max(15, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )
    return output


class PointCloudVisualizer:
    def __init__(self):
        try:
            import open3d as o3d
        except ImportError as error:
            raise RuntimeError(
                "3D 点云可视化需要 Open3D；请安装 open3d，"
                "或使用 --no-point-cloud 关闭 3D 窗口"
            ) from error
        self.o3d = o3d
        self.visualizer = o3d.visualization.Visualizer()
        if not self.visualizer.create_window(
            window_name="Object Point Cloud and Grasp Pose",
            width=960,
            height=720,
        ):
            raise RuntimeError("Open3D 3D 窗口创建失败")
        self.point_cloud = o3d.geometry.PointCloud()
        self.grasp_lines = o3d.geometry.LineSet()
        self.view_initialized = False
        self.visualizer.add_geometry(self.point_cloud)
        self.visualizer.add_geometry(self.grasp_lines)
        option = self.visualizer.get_render_option()
        option.background_color = np.asarray((0.05, 0.05, 0.05))
        option.point_size = 2.0

    def update(self, frame_result):
        clouds = []
        if frame_result.environment_cloud is not None:
            clouds.append((frame_result.environment_cloud, ENVIRONMENT_COLOR))
        poses = []
        for candidate in frame_result.candidates:
            clouds.append(
                (candidate.point_cloud, get_track_color(candidate.track_id))
            )
            poses.append((candidate.pose.position, candidate.pose.rotation))
        if clouds:
            points = np.concatenate([item[0] for item in clouds], axis=0)
            colors = np.concatenate(
                [
                    np.tile(
                        np.asarray(item[1][::-1], dtype=np.float64) / 255.0,
                        (len(item[0]), 1),
                    )
                    for item in clouds
                ], axis=0,
            )
        else:
            points = np.empty((0, 3), dtype=np.float64)
            colors = np.empty((0, 3), dtype=np.float64)
        self.point_cloud.points = self.o3d.utility.Vector3dVector(points)
        self.point_cloud.colors = self.o3d.utility.Vector3dVector(colors)

        vertices, lines, line_colors = [], [], []
        for center, rotation in poses:
            origin_id = len(vertices)
            vertices.append(center)
            for axis_id, color in enumerate(
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
            ):
                vertices.append(center + rotation[:, axis_id] * AXIS_LENGTH)
                lines.append((origin_id, len(vertices) - 1))
                line_colors.append(color)
            closing, lateral = rotation[:, 0], rotation[:, 1]
            for side in (-1.0, 1.0):
                finger_center = center + side * AXIS_LENGTH * 0.55 * closing
                start_id = len(vertices)
                vertices.extend((
                    finger_center - AXIS_LENGTH * 0.325 * lateral,
                    finger_center + AXIS_LENGTH * 0.325 * lateral,
                ))
                lines.append((start_id, start_id + 1))
                line_colors.append((1.0, 1.0, 0.0))
        self.grasp_lines.points = self.o3d.utility.Vector3dVector(
            np.asarray(vertices, dtype=np.float64).reshape((-1, 3))
        )
        self.grasp_lines.lines = self.o3d.utility.Vector2iVector(
            np.asarray(lines, dtype=np.int32).reshape((-1, 2))
        )
        self.grasp_lines.colors = self.o3d.utility.Vector3dVector(
            np.asarray(line_colors, dtype=np.float64).reshape((-1, 3))
        )
        self.visualizer.update_geometry(self.point_cloud)
        self.visualizer.update_geometry(self.grasp_lines)
        if clouds and not self.view_initialized:
            self.visualizer.reset_view_point(True)
            self.view_initialized = True
        alive = self.visualizer.poll_events()
        self.visualizer.update_renderer()
        return alive

    def close(self):
        self.visualizer.destroy_window()
