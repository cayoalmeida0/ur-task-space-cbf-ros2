import math
import unittest

import numpy as np

from ur_cbf_control.nominal_control import compute_position_control
from ur_cbf_control.nominal_control import damped_least_squares
from ur_cbf_control.nominal_control import limit_vector_norm
from ur_cbf_control.nominal_control import NominalControlError
from ur_cbf_control.nominal_control import reorder_vector
from ur_cbf_control.nominal_control import saturate_joint_velocity


class NominalControlTest(unittest.TestCase):
    def test_reorders_between_model_and_controller(self):
        reordered = reorder_vector(
            values=(1.0, 2.0, 3.0),
            source_names=("joint_a", "joint_b", "joint_c"),
            target_names=("joint_c", "joint_a", "joint_b"),
        )
        self.assertEqual(reordered, (3.0, 1.0, 2.0))

    def test_rejects_incompatible_joint_sets(self):
        with self.assertRaisesRegex(NominalControlError, "incompativeis"):
            reorder_vector(
                values=(1.0, 2.0),
                source_names=("joint_a", "joint_b"),
                target_names=("joint_a", "joint_c"),
            )

    def test_limits_cartesian_norm_without_changing_direction(self):
        limited, saturated = limit_vector_norm((3.0, 4.0, 0.0), 2.0)
        np.testing.assert_allclose(limited, (1.2, 1.6, 0.0))
        self.assertTrue(saturated)

    def test_damped_solution_matches_identity_case(self):
        damping = 0.1
        solution = damped_least_squares(
            np.eye(3),
            task_velocity=(0.1, -0.2, 0.3),
            damping=damping,
        )
        expected = np.array((0.1, -0.2, 0.3)) / (1.0 + damping**2)
        np.testing.assert_allclose(solution, expected)

    def test_damped_solution_remains_finite_at_singularity(self):
        jacobian = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
            ]
        )
        solution = damped_least_squares(
            jacobian,
            task_velocity=(0.01, 0.01, 0.01),
            damping=0.05,
        )
        self.assertEqual(solution.shape, (4,))
        self.assertTrue(np.all(np.isfinite(solution)))

    def test_saturates_each_joint_symmetrically(self):
        limited, saturated = saturate_joint_velocity(
            values=(0.2, -0.3, 0.01),
            max_abs_velocity=(0.1, 0.2, 0.05),
        )
        np.testing.assert_allclose(limited, (0.1, -0.2, 0.01))
        self.assertTrue(saturated)

    def test_position_control_derives_dimension_and_controller_order(self):
        result = compute_position_control(
            error=(0.01, 0.0),
            translational_jacobian=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            model_joint_names=("joint_a", "joint_b", "joint_c"),
            controller_joint_names=("joint_c", "joint_a", "joint_b"),
            gains=(1.0, 1.0),
            damping=0.01,
            max_cartesian_speed=0.02,
            max_abs_joint_velocity=0.1,
        )
        self.assertEqual(len(result.model_velocity), 3)
        self.assertAlmostEqual(
            result.controller_velocity[1],
            result.model_velocity[0],
        )
        self.assertAlmostEqual(result.error_norm, 0.01)

    def test_position_control_rejects_non_finite_jacobian(self):
        with self.assertRaisesRegex(NominalControlError, "Jacobiano"):
            compute_position_control(
                error=(0.01, 0.0, 0.0),
                translational_jacobian=((math.nan,), (0.0,), (0.0,)),
                model_joint_names=("joint_a",),
                controller_joint_names=("joint_a",),
                gains=1.0,
                damping=0.01,
                max_cartesian_speed=0.02,
                max_abs_joint_velocity=0.1,
            )

    def test_zero_error_produces_zero_joint_velocity(self):
        result = compute_position_control(
            error=(0.0, 0.0, 0.0),
            translational_jacobian=np.eye(3),
            model_joint_names=("joint_a", "joint_b", "joint_c"),
            controller_joint_names=("joint_a", "joint_b", "joint_c"),
            gains=1.0,
            damping=0.05,
            max_cartesian_speed=0.01,
            max_abs_joint_velocity=0.1,
        )
        np.testing.assert_allclose(result.controller_velocity, (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
