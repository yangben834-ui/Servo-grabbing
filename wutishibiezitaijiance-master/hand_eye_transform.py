"""相机光学坐标系与机械臂基坐标系之间的手眼变换。

标定文件由 hand_eye_calibrate.py 生成，结构为：

    transform_matrix: 4x4 齐次矩阵
    metadata:
      frame_convention: camera_to_base   # eye_to_hand 时为该值
      calibration_mode: eye_to_hand

语义为 ``p_base = R @ p_cam + t``，与 agx_visual_servo 的 C++
``HandEyeTransform`` 以及 bottle_grasp_executor 的 ``_camera_to_base`` 保持一致。
"""

import os
import warnings

import numpy as np
import yaml

EXTRINSICS_FILENAME = "hand_eye_extrinsics.yaml"
DEFAULT_HAND_EYE_PATH = "/home/zdl/H3001_grasp/hand_eye_calibrate/H3003"
EXPECTED_FRAME_CONVENTION = "camera_to_base"
ORTHONORMALITY_TOLERANCE = 1e-6


def resolve_extrinsics_path(path):
    """接受标定目录或 yaml 文件路径，返回外参 yaml 的实际路径。"""
    if path is None:
        return None
    if os.path.isdir(path):
        return os.path.join(path, EXTRINSICS_FILENAME)
    return path


def matrix_to_quaternion(rotation):
    """旋转矩阵转 ``(x, y, z, w)`` 四元数，只依赖 numpy。"""
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f"旋转矩阵形状应为 3x3，实际为 {matrix.shape}")

    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale

    quaternion = np.array([x, y, z, w], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


class HandEyeTransform:
    """相机光学坐标系 -> 基坐标系的固定刚体变换（eye_to_hand）。"""

    def __init__(self, matrix, source=""):
        matrix = np.asarray(matrix, dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError(f"手眼矩阵形状应为 4x4，实际为 {matrix.shape}")
        self.matrix = matrix
        self.source = str(source)
        self.rotation = matrix[:3, :3].copy()
        self.translation = matrix[:3, 3].copy()

    @classmethod
    def from_yaml(cls, path):
        """从标定目录或 yaml 文件加载外参，方向不符时直接报错。"""
        resolved = resolve_extrinsics_path(path)
        if not resolved or not os.path.exists(resolved):
            raise FileNotFoundError(f"未找到手眼标定文件: {path}")

        with open(resolved, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        if not isinstance(data, dict) or "transform_matrix" not in data:
            raise ValueError(f"标定文件缺少 transform_matrix: {resolved}")

        matrix = np.asarray(data["transform_matrix"], dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError(
                f"手眼矩阵形状错误: {matrix.shape}, 期望 4x4: {resolved}"
            )

        convention = (data.get("metadata") or {}).get("frame_convention")
        if convention is None:
            warnings.warn(
                f"标定文件未标注 frame_convention, 按 {EXPECTED_FRAME_CONVENTION} "
                f"处理: {resolved}"
            )
        elif convention != EXPECTED_FRAME_CONVENTION:
            # 方向搞反会静默产出完全错误的坐标, 所以直接拒绝
            raise ValueError(
                f"标定文件方向为 {convention!r}, 本模块只接受 "
                f"{EXPECTED_FRAME_CONVENTION!r}: {resolved}"
            )

        transform = cls(matrix, resolved)
        deviation = float(
            np.max(np.abs(transform.rotation @ transform.rotation.T - np.eye(3)))
        )
        if deviation > ORTHONORMALITY_TOLERANCE:
            warnings.warn(
                f"手眼旋转矩阵正交性偏差 {deviation:.2e}, 建议重新标定: {resolved}"
            )
        return transform

    def point_camera_to_base(self, point):
        """变换一个点（含平移）。"""
        point = np.asarray(point, dtype=np.float64)
        if point.shape != (3,):
            raise ValueError(f"点坐标形状应为 (3,)，实际为 {point.shape}")
        return self.rotation @ point + self.translation

    def vector_camera_to_base(self, vector):
        """变换一个方向向量（不含平移）。"""
        vector = np.asarray(vector, dtype=np.float64)
        if vector.shape != (3,):
            raise ValueError(f"向量形状应为 (3,)，实际为 {vector.shape}")
        return self.rotation @ vector

    def rotation_camera_to_base(self, rotation):
        """变换一个姿态: ``R_base = R_ext @ R_cam``。"""
        rotation = np.asarray(rotation, dtype=np.float64)
        if rotation.shape != (3, 3):
            raise ValueError(f"旋转矩阵形状应为 3x3，实际为 {rotation.shape}")
        return self.rotation @ rotation

    def quaternion_camera_to_base(self, rotation):
        """变换一个姿态并返回 ``(x, y, z, w)`` 四元数。"""
        return matrix_to_quaternion(self.rotation_camera_to_base(rotation))
