import unittest

from ur_cbf_control.trajectory import COMPLEX_WAYPOINT_OFFSETS_M
from ur_cbf_control.trajectory import CHALLENGING_WAYPOINT_OFFSETS_M
from ur_cbf_control.trajectory import resolve_trajectory_waypoints


class TrajectoryProfileTest(unittest.TestCase):
    def test_simple_profile_preserves_requested_offset(self):
        self.assertEqual(
            resolve_trajectory_waypoints("simple", (0.01, -0.02, 0.03)),
            ((0.01, -0.02, 0.03),),
        )

    def test_complex_profile_is_multi_axis_and_returns_to_start(self):
        waypoints = resolve_trajectory_waypoints(
            "complex",
            (0.0, 0.0, 0.01),
        )
        self.assertEqual(waypoints, COMPLEX_WAYPOINT_OFFSETS_M)
        self.assertEqual(len(waypoints), 5)
        self.assertEqual(waypoints[-1], (0.0, 0.0, 0.0))
        self.assertTrue(any(point[0] != 0.0 for point in waypoints))
        self.assertTrue(any(point[1] != 0.0 for point in waypoints))
        self.assertTrue(any(point[2] != 0.0 for point in waypoints))

    def test_unknown_profile_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "simple, complex ou challenging"):
            resolve_trajectory_waypoints("unknown", (0.0, 0.0, 0.01))

    def test_challenging_profile_descends_and_alternates_x_side(self):
        waypoints = resolve_trajectory_waypoints(
            "challenging",
            (0.0, 0.0, 0.01),
        )
        self.assertEqual(waypoints, CHALLENGING_WAYPOINT_OFFSETS_M)
        self.assertEqual(len(waypoints), 5)
        self.assertEqual(waypoints[-1], (0.0, 0.0, 0.0))
        self.assertTrue(any(point[0] >= 0.18 for point in waypoints))
        self.assertTrue(any(point[0] <= -0.18 for point in waypoints))
        self.assertTrue(any(point[2] <= -0.32 for point in waypoints))

    def test_non_finite_simple_offset_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "tres valores finitos"):
            resolve_trajectory_waypoints(
                "simple",
                (0.0, float("nan"), 0.01),
            )


if __name__ == "__main__":
    unittest.main()
