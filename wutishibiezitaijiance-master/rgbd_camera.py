"""RealSense 和 Orbbec RGB-D 相机的统一访问接口。"""

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30

# Orbbec 启动后首帧常常拿不到（帧同步 + 首帧丢弃），单次等待会误判为无数据
ORBBEC_STARTUP_ATTEMPTS = 10
ORBBEC_STARTUP_TIMEOUT_MS = 500


class RealSenseCamera:
    """输出对齐到彩色图的 BGR 图像、米制深度图和彩色相机内参。"""

    def __init__(self, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, fps=DEFAULT_FPS):
        try:
            import numpy as np
            import pyrealsense2 as rs
        except ImportError as error:
            raise RuntimeError(
                "使用 RealSense 需要安装 numpy 和 pyrealsense2"
            ) from error

        self.np = np
        self.rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipeline.start(config)

        self.depth_scale = (
            profile.get_device().first_depth_sensor().get_depth_scale()
        )
        intr = profile.get_stream(
            rs.stream.color
        ).as_video_stream_profile().get_intrinsics()
        self.intrinsics = (intr.fx, intr.fy, intr.ppx, intr.ppy)

        self.align = rs.align(rs.stream.color)
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.hole = rs.hole_filling_filter()

    def read(self):
        frames = self.align.process(self.pipeline.wait_for_frames())
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None

        depth_frame = self.spatial.process(depth_frame)
        depth_frame = self.temporal.process(depth_frame)
        depth_frame = self.hole.process(depth_frame)

        color = self.np.asanyarray(color_frame.get_data())
        depth = self.np.asanyarray(depth_frame.get_data()).astype(self.np.float32)
        depth *= self.depth_scale
        return color, depth

    def stop(self):
        self.pipeline.stop()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()


class OrbbecCamera:
    """输出对齐到彩色图的 BGR 图像、米制深度图和彩色相机内参。"""

    def __init__(self, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, fps=DEFAULT_FPS):
        try:
            import cv2
            import numpy as np
            from pyorbbecsdk import (
                AlignFilter,
                Config,
                OBFormat,
                OBSensorType,
                OBStreamType,
                Pipeline,
            )
        except ImportError as error:
            raise RuntimeError(
                "使用 Orbbec 需要安装 numpy、opencv-python 和 "
                "pyorbbecsdk2（导入名为 pyorbbecsdk）"
            ) from error

        self.cv2 = cv2
        self.np = np
        self.pipeline = Pipeline()
        config = Config()
        color_profiles = self.pipeline.get_stream_profile_list(
            OBSensorType.COLOR_SENSOR
        )
        try:
            color_profile = color_profiles.get_video_stream_profile(
                width, height, OBFormat.RGB, fps
            )
        except Exception:
            color_profile = color_profiles.get_video_stream_profile(
                0, 0, OBFormat.RGB, 0
            )

        depth_profiles = self.pipeline.get_stream_profile_list(
            OBSensorType.DEPTH_SENSOR
        )
        config.enable_stream(color_profile)
        config.enable_stream(depth_profiles.get_default_video_stream_profile())

        try:
            self.pipeline.enable_frame_sync()
        except Exception as error:
            print(f"Orbbec帧同步未启用: {error}")

        self.align = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        self.pipeline.start(config)
        for _ in range(ORBBEC_STARTUP_ATTEMPTS):
            if self.pipeline.wait_for_frames(ORBBEC_STARTUP_TIMEOUT_MS) is not None:
                break
        else:
            self.pipeline.stop()
            raise RuntimeError(
                f"未收到 Orbbec 相机数据（已重试 {ORBBEC_STARTUP_ATTEMPTS} 次）"
            )

        intr = self.pipeline.get_camera_param().rgb_intrinsic
        self.intrinsics = (intr.fx, intr.fy, intr.cx, intr.cy)
        self.ob_format = OBFormat

    def _color_to_bgr(self, color_frame):
        width = color_frame.get_width()
        height = color_frame.get_height()
        frame_format = color_frame.get_format()
        data = self.np.frombuffer(color_frame.get_data(), dtype=self.np.uint8)

        if frame_format == self.ob_format.RGB:
            return self.cv2.cvtColor(
                data.reshape((height, width, 3)), self.cv2.COLOR_RGB2BGR
            )
        if frame_format == getattr(self.ob_format, "BGR", None):
            return data.reshape((height, width, 3)).copy()
        if frame_format == getattr(self.ob_format, "MJPG", None):
            return self.cv2.imdecode(data, self.cv2.IMREAD_COLOR)
        raise RuntimeError(f"暂不支持 Orbbec 彩色格式: {frame_format}")

    def read(self):
        frames = self.pipeline.wait_for_frames(1000)
        if frames is None:
            return None, None
        frames = self.align.process(frames)
        if frames is None:
            return None, None

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None

        try:
            color = self._color_to_bgr(color_frame)
            depth = self.np.frombuffer(
                depth_frame.get_data(), dtype=self.np.uint16
            ).reshape(
                (depth_frame.get_height(), depth_frame.get_width())
            )
        except ValueError:
            return None, None

        depth = (
            depth.astype(self.np.float32)
            * depth_frame.get_depth_scale()
            / 1000.0
        )
        if depth.shape != color.shape[:2]:
            depth = self.cv2.resize(
                depth,
                (color.shape[1], color.shape[0]),
                interpolation=self.cv2.INTER_NEAREST,
            )
        return color, depth

    def stop(self):
        self.pipeline.stop()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()


def create_camera(camera_type, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, fps=DEFAULT_FPS):
    """按名称创建相机，支持 ``realsense`` 和 ``orbbec``。"""
    camera_classes = {
        "realsense": RealSenseCamera,
        "orbbec": OrbbecCamera,
    }
    try:
        camera_class = camera_classes[camera_type.lower()]
    except (AttributeError, KeyError) as error:
        supported = ", ".join(camera_classes)
        raise ValueError(f"不支持的相机类型: {camera_type!r}；可选: {supported}") from error
    return camera_class(width=width, height=height, fps=fps)


__all__ = ["RealSenseCamera", "OrbbecCamera", "create_camera"]
