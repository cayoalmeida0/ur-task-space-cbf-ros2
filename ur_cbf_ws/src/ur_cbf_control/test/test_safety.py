import math
import unittest

from ur_cbf_control.safety import JointStateValidationError
from ur_cbf_control.safety import build_velocity_command
from ur_cbf_control.safety import is_state_stale
from ur_cbf_control.safety import reorder_joint_state
from ur_cbf_control.safety import zero_velocity_command


CONTROLLER_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class SafetyTest(unittest.TestCase):
    def test_reorders_joint_state_by_name(self):
        message_names = (
            "elbow_joint",
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        )
        state = reorder_joint_state(
            CONTROLLER_ORDER,
            message_names,
            positions=(3.0, 2.0, 1.0, 4.0, 5.0, 6.0),
            velocities=(0.3, 0.2, 0.1, 0.4, 0.5, 0.6),
        )

        self.assertEqual(state.names, CONTROLLER_ORDER)
        self.assertEqual(state.positions, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
        self.assertEqual(state.velocities, (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))

    def test_rejects_missing_joint(self):
        with self.assertRaisesRegex(JointStateValidationError, "ausentes"):
            reorder_joint_state(
                CONTROLLER_ORDER,
                message_names=CONTROLLER_ORDER[:-1],
                positions=(0.0,) * 5,
                velocities=(0.0,) * 5,
            )

    def test_rejects_duplicate_names(self):
        names = list(CONTROLLER_ORDER)
        names[-1] = names[-2]
        with self.assertRaisesRegex(JointStateValidationError, "duplicados"):
            reorder_joint_state(
                CONTROLLER_ORDER,
                message_names=names,
                positions=(0.0,) * 6,
                velocities=(0.0,) * 6,
            )

    def test_rejects_non_finite_state(self):
        positions = [0.0] * 6
        positions[2] = math.nan
        with self.assertRaisesRegex(JointStateValidationError, "NaN"):
            reorder_joint_state(
                CONTROLLER_ORDER,
                message_names=CONTROLLER_ORDER,
                positions=positions,
                velocities=(0.0,) * 6,
            )

    def test_places_velocity_at_target_joint(self):
        command = build_velocity_command(
            CONTROLLER_ORDER,
            target_joint="elbow_joint",
            requested_velocity=0.03,
            max_abs_velocity=0.05,
        )

        self.assertEqual(command.values, (0.0, 0.0, 0.03, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(command.applied_velocity, 0.03)
        self.assertFalse(command.saturated)

    def test_saturates_velocity_symmetrically(self):
        positive = build_velocity_command(
            CONTROLLER_ORDER,
            target_joint="elbow_joint",
            requested_velocity=0.2,
            max_abs_velocity=0.05,
        )
        negative = build_velocity_command(
            CONTROLLER_ORDER,
            target_joint="elbow_joint",
            requested_velocity=-0.2,
            max_abs_velocity=0.05,
        )

        self.assertAlmostEqual(positive.applied_velocity, 0.05)
        self.assertAlmostEqual(negative.applied_velocity, -0.05)
        self.assertTrue(positive.saturated and negative.saturated)

    def test_zero_command_dimension_comes_from_configuration(self):
        self.assertEqual(zero_velocity_command(CONTROLLER_ORDER), (0.0,) * 6)
        self.assertEqual(
            zero_velocity_command(("joint_a", "joint_b")),
            (0.0, 0.0),
        )

    def test_state_timeout_uses_monotonic_receipt_time(self):
        self.assertFalse(is_state_stale(10.0, 10.2, timeout=0.25))
        self.assertTrue(is_state_stale(10.0, 10.3, timeout=0.25))
        self.assertTrue(is_state_stale(10.0, 9.9, timeout=0.25))


if __name__ == "__main__":
    unittest.main()
