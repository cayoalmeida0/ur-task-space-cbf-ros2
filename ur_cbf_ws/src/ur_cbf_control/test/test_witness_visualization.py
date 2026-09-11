import unittest

from builtin_interfaces.msg import Time
import numpy as np
from visualization_msgs.msg import Marker

from ur_cbf_control.self_collision_cbf import formulate_self_collision_cbf
from ur_cbf_control.self_collision_cbf import SelfCollisionDistances
from ur_cbf_control.witness_visualization import build_witness_marker_array


class WitnessVisualizationTest(unittest.TestCase):
    def setUp(self):
        distances = SelfCollisionDistances(
            distances=np.array((0.08, 0.02)),
            jacobian=np.zeros((2, 6)),
            pair_labels=("safe_pair", "violating_pair"),
            first_witness_points=np.array(((0.0, 0.0, 0.0), (1.0, 2.0, 3.0))),
            second_witness_points=np.array(((0.08, 0.0, 0.0), (1.02, 2.0, 3.0))),
            geometry_source="test",
        )
        self.constraints = formulate_self_collision_cbf(
            distances,
            safe_distance=0.03,
            gain=5.0,
        )

    def test_closest_mode_publishes_only_minimum_pair(self):
        result = build_witness_marker_array(
            self.constraints,
            mode="closest",
            frame_id="base",
            stamp=Time(),
        )

        self.assertEqual(len(result.markers), 3)
        self.assertEqual(result.markers[0].action, Marker.DELETEALL)
        line, points = result.markers[1:]
        self.assertEqual(line.type, Marker.LINE_LIST)
        self.assertEqual(points.type, Marker.SPHERE_LIST)
        self.assertEqual(line.header.frame_id, "base")
        self.assertEqual(len(line.points), 2)
        self.assertAlmostEqual(line.points[0].x, 1.0)
        self.assertAlmostEqual(line.points[1].x, 1.02)
        self.assertAlmostEqual(line.color.r, 1.0)

    def test_all_mode_publishes_every_pair(self):
        result = build_witness_marker_array(
            self.constraints,
            mode="all",
            frame_id="base",
            stamp=Time(),
        )

        self.assertEqual(len(result.markers), 5)
        self.assertEqual(result.markers[1].color.g, 0.9)
        self.assertEqual(result.markers[3].color.r, 1.0)

    def test_off_mode_clears_previous_markers(self):
        result = build_witness_marker_array(
            self.constraints,
            mode="off",
            frame_id="base",
            stamp=Time(),
        )

        self.assertEqual(len(result.markers), 1)
        self.assertEqual(result.markers[0].action, Marker.DELETEALL)


if __name__ == "__main__":
    unittest.main()
