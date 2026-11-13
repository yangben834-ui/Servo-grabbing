"""ROS message regression tests; source the built ROS workspace before running."""

import unittest
from types import SimpleNamespace

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import Header

from bottle_grasp_interfaces.msg import GraspCandidateArray
from bottle_grasp_interfaces.srv import GetGraspCandidates
from bottle_grasp_perception.grasp_pose_node import BottleGraspNode, points_to_cloud


class GraspMessagesTest(unittest.TestCase):
    def test_cloud_sampling_and_empty_cloud(self):
        for count in (0, 638, 1024, 6000):
            with self.subTest(count=count):
                points = np.arange(count * 6, dtype=np.float64).reshape(count, 6)[:, ::2]
                cloud = points_to_cloud(points)
                self.assertEqual(len(cloud), min(count, 1024))
                expected = points if count <= 1024 else points[
                    np.linspace(0, count - 1, 1024, dtype=np.int64)]
                actual = np.array([(p.x, p.y, p.z) for p in cloud]).reshape(-1, 3)
                np.testing.assert_array_equal(actual, expected.astype(np.float32))

    def test_configurable_limit(self):
        points = np.arange(18000, dtype=np.float32).reshape(6000, 3)
        for limit in (1, 512, 2048, 8000):
            self.assertEqual(len(points_to_cloud(points, limit)), min(6000, limit))
        for invalid in (0, -1, 1.5, True):
            with self.assertRaises(ValueError):
                points_to_cloud(points, invalid)

    def test_result_and_service_serialization_preserve_cloud_and_pose(self):
        node = SimpleNamespace(
            pipeline=SimpleNamespace(estimator=SimpleNamespace(
                config=SimpleNamespace(max_point_count=6000))),
            get_parameter=lambda name: SimpleNamespace(
                value=2048 if name == "publish_point_count" else 0.06),
            _quaternion_to_rotation=BottleGraspNode._quaternion_to_rotation,
        )
        candidate = SimpleNamespace(
            track_id=7, class_id=0, class_name="bottle", confidence=0.9,
            point_cloud=np.array([[0.1, 0.2, 0.5], [0.2, 0.3, 0.6]]),
            pose=SimpleNamespace(mode="upright", axis_table_angle_deg=90.0,
                                 position=np.array([0.1, 0.2, 0.5]), rotation=np.eye(3)),
        )
        header = Header(frame_id="original")
        header.stamp.sec = 123
        result = BottleGraspNode._make_result_message(node, [candidate], header, "camera")
        self.assertEqual(header.frame_id, "original")
        BottleGraspNode._make_markers(node, result)
        result = deserialize_message(serialize_message(result), GraspCandidateArray)
        self.assertTrue(result.perception_valid)
        message = result.candidates[0]
        self.assertEqual(message.header.frame_id, "camera")
        self.assertEqual(message.header.stamp.sec, 123)
        self.assertEqual(message.point_count, len(message.object_points))
        self.assertEqual(message.pose.position.z, 0.5)
        response = GetGraspCandidates.Response(success=True, result=result)
        restored = deserialize_message(serialize_message(response), GetGraspCandidates.Response)
        np.testing.assert_allclose(
            [(p.x, p.y, p.z) for p in restored.result.candidates[0].object_points], candidate.point_cloud)
        candidate.point_cloud = np.arange(18000, dtype=np.float32).reshape(6000, 3)
        large = BottleGraspNode._make_result_message(node, [candidate], header, "camera")
        large = deserialize_message(serialize_message(large), GraspCandidateArray)
        self.assertEqual(large.candidates[0].point_count, 2048)
        self.assertEqual(len(large.candidates[0].object_points), 2048)
        self.assertAlmostEqual(large.candidates[0].quality, candidate.confidence)
        empty = BottleGraspNode._make_result_message(node, [], header, "camera")
        self.assertFalse(empty.perception_valid)
        self.assertEqual(len(empty.candidates), 0)


if __name__ == "__main__":
    unittest.main()
