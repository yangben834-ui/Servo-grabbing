"""不依赖相机、模型权重和 GUI 的几何算法回归测试。"""

import unittest
import numpy as np

from grasp_pose_estimator import (
    GeometryConfig,
    estimate_cylinder_axis,
    estimate_grasp_pose,
    estimate_table_normal,
    mask_to_point_cloud,
)

# 合成竖直瓶: 轴沿相机系 Y (图像里竖直), 轴线过 (AXIS_X, *, AXIS_Z)
AXIS_X = 0.02
AXIS_Z = 0.6
BOTTLE_RADIUS = 0.03
# 瓶高 18cm, 明显大于 mask 泄漏进来的桌面范围, 否则 PCA 主轴会被桌面带走
BOTTLE_HALF_HEIGHT = 0.09
BOTTLE_AXIS = np.array((0.0, 1.0, 0.0))
# 桌面法向与瓶轴平行 -> 判定为 side, 与真机的站立瓶一致
TABLE_NORMAL = np.array((0.0, -1.0, 0.0))


def half_shell_cloud(rng, count=800, noise=0.001):
    """朝向相机的半个圆柱壳: 只保留法向指向相机的那一半表面。"""
    phi = rng.uniform(-np.pi / 2.0, np.pi / 2.0, size=count)
    y = rng.uniform(-BOTTLE_HALF_HEIGHT, BOTTLE_HALF_HEIGHT, size=count)
    x = AXIS_X + BOTTLE_RADIUS * np.sin(phi)
    z = AXIS_Z - BOTTLE_RADIUS * np.cos(phi)
    points = np.column_stack((x, y, z))
    return points + rng.normal(0.0, noise, size=points.shape)


def table_patch(rng, count=400, half_extent=0.06):
    """瓶底那一圈桌面, 是 mask 偏松时最典型的背景污染。"""
    x = rng.uniform(-half_extent, half_extent, size=count) + AXIS_X
    z = rng.uniform(AXIS_Z - half_extent, AXIS_Z + half_extent, size=count)
    y = BOTTLE_HALF_HEIGHT + rng.normal(0.0, 0.001, size=count)
    return np.column_stack((x, y, z))


def distance_to_bottle_axis(point):
    """点到真实瓶轴直线的距离。"""
    delta = point - np.array((AXIS_X, 0.0, AXIS_Z))
    along = np.dot(delta, BOTTLE_AXIS) * BOTTLE_AXIS
    return float(np.linalg.norm(delta - along))


class GraspPoseEstimatorTest(unittest.TestCase):
    def setUp(self):
        self.config = GeometryConfig(
            min_point_count=20,
            table_min_inliers=100,
            random_seed=7,
        )
        self.cylinder_config = GeometryConfig(
            min_point_count=20,
            table_min_inliers=100,
            random_seed=7,
            use_cylinder_axis=True,
        )
        self.rng = np.random.default_rng(3)

    def assert_valid_rotation(self, rotation):
        np.testing.assert_allclose(
            rotation.T @ rotation, np.eye(3), atol=1e-6
        )
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=6)

    def test_mask_to_point_cloud_uses_metric_depth(self):
        depth = np.ones((10, 10), dtype=np.float32) * 0.5
        mask = np.ones((10, 10), dtype=bool)
        points = mask_to_point_cloud(
            depth, mask, (100.0, 100.0, 4.5, 4.5), self.config
        )
        self.assertEqual(points.shape, (100, 3))
        np.testing.assert_allclose(points[:, 2], 0.5)
        np.testing.assert_allclose(np.median(points, axis=0), (0, 0, 0.5))

    def test_estimate_table_normal(self):
        xy = self.rng.uniform(-0.4, 0.4, size=(1000, 2))
        z = 0.7 + self.rng.normal(0, 0.0005, size=(1000, 1))
        points = np.column_stack((xy, z))
        normal = estimate_table_normal(points, self.config)
        self.assertIsNotNone(normal)
        self.assertLess(normal[2], -0.999)

    def test_upright_bottle_generates_side_grasp(self):
        points = self.rng.normal(size=(1000, 3)) * (0.01, 0.01, 0.10)
        points += (0.15, 0.0, 0.6)
        pose = estimate_grasp_pose(
            points, np.array((0.0, 0.0, -1.0)), self.config
        )
        self.assertEqual(pose.mode, "side")
        self.assertLess(pose.axis_table_angle_deg, 5.0)
        self.assert_valid_rotation(pose.rotation)

    def test_horizontal_bottle_generates_top_grasp(self):
        points = self.rng.normal(size=(1000, 3)) * (0.10, 0.01, 0.01)
        points += (0.0, 0.0, 0.6)
        pose = estimate_grasp_pose(
            points, np.array((0.0, 0.0, -1.0)), self.config
        )
        self.assertEqual(pose.mode, "top")
        self.assertGreater(pose.axis_table_angle_deg, 85.0)
        self.assert_valid_rotation(pose.rotation)

    def test_cylinder_fit_recovers_bottle_radius(self):
        clean = half_shell_cloud(self.rng)
        fit = estimate_cylinder_axis(clean, BOTTLE_AXIS, self.cylinder_config)
        self.assertTrue(fit.ok, fit.reject_reason)
        self.assertAlmostEqual(fit.radius, BOTTLE_RADIUS, delta=0.002)
        # 干净的半壳: 张角就是可见的那半圈
        self.assertGreater(fit.arc_deg, 170.0)
        self.assertGreater(fit.inlier_ratio, 0.9)

        # 掺进瓶底桌面后半径仍应稳住 (背景点靠 RANSAC 排除)
        dirty = np.vstack((half_shell_cloud(self.rng), table_patch(self.rng)))
        fit = estimate_cylinder_axis(dirty, BOTTLE_AXIS, self.cylinder_config)
        self.assertTrue(fit.ok, fit.reject_reason)
        self.assertAlmostEqual(fit.radius, BOTTLE_RADIUS, delta=0.002)
        self.assertGreater(fit.inlier_ratio, 0.5)

    def test_cylinder_fit_moves_grasp_point_onto_axis(self):
        cloud = np.vstack(
            (half_shell_cloud(self.rng), table_patch(self.rng))
        )
        surface_pose = estimate_grasp_pose(cloud, TABLE_NORMAL, self.config)
        axis_pose = estimate_grasp_pose(
            cloud, TABLE_NORMAL, self.cylinder_config
        )

        # 关闭开关时行为不变: 仍是可见表面质心
        self.assertIsNone(surface_pose.radius)
        np.testing.assert_array_equal(
            surface_pose.position, np.median(cloud, axis=0)
        )

        self.assertIsNotNone(axis_pose.radius)
        self.assertEqual(axis_pose.mode, "side")
        self.assert_valid_rotation(axis_pose.rotation)
        # surface_position 始终留着, 便于对照检查位移方向
        np.testing.assert_array_equal(
            axis_pose.surface_position, np.median(cloud, axis=0)
        )
        # 表面质心天然偏在瓶壁一侧, 拟合后应落到轴上
        self.assertGreater(
            distance_to_bottle_axis(surface_pose.position), 0.012
        )
        self.assertLess(distance_to_bottle_axis(axis_pose.position), 0.004)
        # 位移是径向的且指向远离相机的一侧(相机在原点, 瓶轴比可见面更远), 量级约 2r/pi
        shift = axis_pose.position - axis_pose.surface_position
        radial_shift = shift - np.dot(shift, BOTTLE_AXIS) * BOTTLE_AXIS
        radial_length = float(np.linalg.norm(radial_shift))
        self.assertGreater(radial_length, 0.010)
        self.assertLess(radial_length, 0.030)
        self.assertGreater(
            float(np.dot(shift, axis_pose.surface_position)), 0.0
        )

    def test_non_cylinder_cloud_falls_back_to_surface_centroid(self):
        # 平坦的桌面区块: 没有圆柱结构, 应当拒绝拟合
        xy = self.rng.uniform(-0.15, 0.15, size=(1500, 2))
        z = 0.6 + self.rng.normal(0, 0.001, size=(1500, 1))
        cloud = np.column_stack((xy, z))
        pose = estimate_grasp_pose(cloud, TABLE_NORMAL, self.cylinder_config)
        self.assertIsNone(pose.radius)
        self.assertTrue(pose.fit_reason)
        np.testing.assert_array_equal(pose.position, np.median(cloud, axis=0))
        self.assert_valid_rotation(pose.rotation)

    def test_cylinder_fit_is_deterministic_with_seed(self):
        cloud = np.vstack(
            (half_shell_cloud(self.rng), table_patch(self.rng))
        )
        first = estimate_cylinder_axis(
            cloud, BOTTLE_AXIS, self.cylinder_config
        )
        second = estimate_cylinder_axis(
            cloud, BOTTLE_AXIS, self.cylinder_config
        )
        self.assertTrue(first.ok, first.reject_reason)
        self.assertEqual(first.radius, second.radius)
        np.testing.assert_array_equal(first.axis_point, second.axis_point)


if __name__ == "__main__":
    unittest.main()
