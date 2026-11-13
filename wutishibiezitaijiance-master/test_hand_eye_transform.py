"""不依赖相机和标定数据的手眼变换回归测试。"""

import os
import tempfile
import unittest
import warnings

import numpy as np
import yaml

from hand_eye_transform import (
    HandEyeTransform,
    matrix_to_quaternion,
    resolve_extrinsics_path,
)


def quaternion_to_matrix(quaternion):
    x, y, z, w = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_z(angle):
    cos, sin = np.cos(angle), np.sin(angle)
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


class HandEyeTransformTest(unittest.TestCase):
    def setUp(self):
        self.rotation = rotation_z(0.7)
        self.translation = np.array([0.5, -0.2, 0.1])
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = self.translation
        self.transform = HandEyeTransform(matrix, "synthetic")

    def assert_valid_rotation(self, rotation):
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=6)

    def test_point_transform_matches_manual_formula(self):
        point = np.array([0.1, 0.2, 0.3])
        expected = self.rotation @ point + self.translation
        np.testing.assert_allclose(
            self.transform.point_camera_to_base(point), expected, atol=1e-12
        )

    def test_vector_transform_ignores_translation(self):
        vector = np.array([0.0, 0.0, 1.0])
        np.testing.assert_allclose(
            self.transform.vector_camera_to_base(vector),
            self.rotation @ vector,
            atol=1e-12,
        )

    def test_point_transform_is_invertible(self):
        point = np.array([-0.3, 0.05, 0.42])
        base = self.transform.point_camera_to_base(point)
        back = self.rotation.T @ (base - self.translation)
        np.testing.assert_allclose(back, point, atol=1e-12)

    def test_rotation_transform_composes_and_stays_valid(self):
        camera_rotation = rotation_z(-0.4)
        base_rotation = self.transform.rotation_camera_to_base(camera_rotation)
        np.testing.assert_allclose(
            base_rotation, self.rotation @ camera_rotation, atol=1e-12
        )
        self.assert_valid_rotation(base_rotation)

    def test_quaternion_reproduces_transformed_rotation(self):
        camera_rotation = rotation_z(1.1)
        quaternion = self.transform.quaternion_camera_to_base(camera_rotation)
        self.assertAlmostEqual(float(np.linalg.norm(quaternion)), 1.0, places=9)
        expected = self.rotation @ camera_rotation
        np.testing.assert_allclose(
            quaternion_to_matrix(quaternion), expected, atol=1e-9
        )

    def test_matrix_to_quaternion_for_identity_and_axis_rotations(self):
        np.testing.assert_allclose(
            np.abs(matrix_to_quaternion(np.eye(3))), [0, 0, 0, 1], atol=1e-12
        )
        for angle in (0.3, 1.9, -2.5):
            quaternion = matrix_to_quaternion(rotation_z(angle))
            np.testing.assert_allclose(
                quaternion_to_matrix(quaternion), rotation_z(angle), atol=1e-9
            )

    def test_rejects_non_rotation_matrix_shape(self):
        with self.assertRaises(ValueError):
            HandEyeTransform(np.eye(3))

    def test_point_transform_rejects_wrong_shape(self):
        with self.assertRaises(ValueError):
            self.transform.point_camera_to_base([0.1, 0.2])


class HandEyeTransformYamlTest(unittest.TestCase):
    def write_yaml(self, directory, matrix, convention="camera_to_base"):
        path = os.path.join(directory, "hand_eye_extrinsics.yaml")
        with open(path, "w", encoding="utf-8") as handle:
            yaml.safe_dump(
                {
                    "transform_matrix": np.asarray(matrix).tolist(),
                    "metadata": {"frame_convention": convention},
                },
                handle,
            )
        return path

    def setUp(self):
        self.matrix = np.eye(4)
        self.matrix[:3, :3] = rotation_z(0.25)
        self.matrix[:3, 3] = [0.6, 0.03, 0.04]
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_loads_from_directory_path(self):
        self.write_yaml(self.tmp.name, self.matrix)
        transform = HandEyeTransform.from_yaml(self.tmp.name)
        np.testing.assert_allclose(transform.matrix, self.matrix, atol=1e-12)
        self.assertEqual(
            transform.source,
            os.path.join(self.tmp.name, "hand_eye_extrinsics.yaml"),
        )

    def test_resolve_accepts_directory_and_file(self):
        path = self.write_yaml(self.tmp.name, self.matrix)
        self.assertEqual(resolve_extrinsics_path(self.tmp.name), path)
        self.assertEqual(resolve_extrinsics_path(path), path)

    def test_rejects_inverted_frame_convention(self):
        # 方向搞反会静默产出错误坐标, 必须直接报错
        self.write_yaml(self.tmp.name, self.matrix, convention="base_to_camera")
        with self.assertRaises(ValueError):
            HandEyeTransform.from_yaml(self.tmp.name)

    def test_warns_when_convention_missing(self):
        path = os.path.join(self.tmp.name, "hand_eye_extrinsics.yaml")
        with open(path, "w", encoding="utf-8") as handle:
            yaml.safe_dump({"transform_matrix": self.matrix.tolist()}, handle)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            HandEyeTransform.from_yaml(path)
        self.assertTrue(any("frame_convention" in str(w.message) for w in caught))

    def test_warns_on_non_orthonormal_rotation(self):
        matrix = np.eye(4)
        matrix[:3, :3] = rotation_z(0.25) * 1.05
        self.write_yaml(self.tmp.name, matrix)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            HandEyeTransform.from_yaml(self.tmp.name)
        self.assertTrue(any("正交性" in str(w.message) for w in caught))

    def test_missing_file_raises(self):
        with self.assertRaises(FileNotFoundError):
            HandEyeTransform.from_yaml(os.path.join(self.tmp.name, "nope"))


if __name__ == "__main__":
    unittest.main()
