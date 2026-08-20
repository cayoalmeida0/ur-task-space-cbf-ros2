import math
from types import SimpleNamespace
import unittest

import numpy as np

from ur_cbf_control.nominal_control import damped_least_squares
from ur_cbf_control.qp_control import BoxConstrainedQpSolver
from ur_cbf_control.qp_control import compute_qp_position_control
from ur_cbf_control.qp_control import QpControlError


class FailedSolver:
    def setup(self, **kwargs):
        self.dimension = len(kwargs["q"])

    def update(self, **kwargs):
        pass

    def solve(self, raise_error=False):
        info = SimpleNamespace(status="maximum iterations reached", status_val=7)
        return SimpleNamespace(x=np.zeros(self.dimension), info=info)


class QpControlTest(unittest.TestCase):
    def test_matches_damped_least_squares_when_bounds_are_inactive(self):
        jacobian = np.array(
            [
                [1.0, 0.2, 0.0, 0.0],
                [0.0, 0.8, 0.1, 0.0],
                [0.0, 0.0, 0.6, 0.2],
            ]
        )
        velocity = np.array((0.01, -0.005, 0.002))
        damping = 0.05
        expected = damped_least_squares(jacobian, velocity, damping)
        actual, diagnostics = BoxConstrainedQpSolver(
            absolute_tolerance=1e-8,
            relative_tolerance=1e-8,
        ).solve(
            jacobian=jacobian,
            task_velocity=velocity,
            damping=damping,
            max_abs_joint_velocity=1.0,
        )
        np.testing.assert_allclose(actual, expected, rtol=1e-3, atol=2e-7)
        self.assertFalse(diagnostics.constraint_active)
        self.assertIn(diagnostics.status_value, (1, 2))

    def test_enforces_joint_bounds_inside_qp(self):
        solution, diagnostics = BoxConstrainedQpSolver().solve(
            jacobian=np.eye(3),
            task_velocity=(1.0, -1.0, 0.01),
            damping=0.01,
            max_abs_joint_velocity=(0.1, 0.2, 0.05),
        )
        np.testing.assert_allclose(solution, (0.1, -0.2, 0.009999), atol=2e-6)
        self.assertEqual(diagnostics.active_upper, (0,))
        self.assertEqual(diagnostics.active_lower, (1,))
        self.assertLessEqual(diagnostics.max_bound_violation, 1e-5)

    def test_reuses_workspace_and_accepts_dimension_change(self):
        solver = BoxConstrainedQpSolver()
        _, first = solver.solve(
            jacobian=np.eye(2),
            task_velocity=(0.01, 0.0),
            damping=0.05,
            max_abs_joint_velocity=0.1,
        )
        second_solution, second = solver.solve(
            jacobian=np.eye(2),
            task_velocity=(0.0, 0.01),
            damping=0.05,
            max_abs_joint_velocity=0.1,
        )
        solution, third = solver.solve(
            jacobian=np.eye(3, 4),
            task_velocity=(0.01, 0.0, 0.0),
            damping=0.05,
            max_abs_joint_velocity=0.1,
        )
        self.assertFalse(first.reused_workspace)
        self.assertTrue(second.reused_workspace)
        self.assertFalse(third.reused_workspace)
        np.testing.assert_allclose(
            second_solution,
            damped_least_squares(np.eye(2), (0.0, 0.01), 0.05),
            rtol=1e-3,
            atol=2e-7,
        )
        self.assertEqual(solution.shape, (4,))

    def test_controller_reorders_solution_by_joint_name(self):
        result = compute_qp_position_control(
            error=(0.01, 0.0),
            translational_jacobian=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            model_joint_names=("joint_a", "joint_b", "joint_c"),
            controller_joint_names=("joint_c", "joint_a", "joint_b"),
            gains=(1.0, 1.0),
            damping=0.01,
            max_cartesian_speed=0.02,
            max_abs_joint_velocity=0.1,
            solver=BoxConstrainedQpSolver(),
        )
        self.assertEqual(len(result.model_velocity), 3)
        self.assertAlmostEqual(
            result.controller_velocity[1],
            result.model_velocity[0],
        )
        self.assertFalse(result.joint_constraint_active)

    def test_zero_error_produces_zero_velocity(self):
        result = compute_qp_position_control(
            error=(0.0, 0.0, 0.0),
            translational_jacobian=np.eye(3),
            model_joint_names=("a", "b", "c"),
            controller_joint_names=("a", "b", "c"),
            gains=1.0,
            damping=0.05,
            max_cartesian_speed=0.01,
            max_abs_joint_velocity=0.1,
            solver=BoxConstrainedQpSolver(),
        )
        np.testing.assert_allclose(result.controller_velocity, np.zeros(3))

    def test_rejects_non_finite_jacobian(self):
        with self.assertRaisesRegex(QpControlError, "Jacobiano"):
            BoxConstrainedQpSolver().solve(
                jacobian=((math.nan,),),
                task_velocity=(0.0,),
                damping=0.05,
                max_abs_joint_velocity=0.1,
            )

    def test_rejects_incompatible_limit_dimension(self):
        with self.assertRaisesRegex(QpControlError, "Quantidade de limites"):
            BoxConstrainedQpSolver().solve(
                jacobian=np.eye(3),
                task_velocity=(0.0, 0.0, 0.0),
                damping=0.05,
                max_abs_joint_velocity=(0.1, 0.1),
            )

    def test_solver_failure_is_reported(self):
        solver = BoxConstrainedQpSolver(solver_factory=FailedSolver)
        with self.assertRaisesRegex(QpControlError, "nao encontrou"):
            solver.solve(
                jacobian=np.eye(2),
                task_velocity=(0.01, 0.0),
                damping=0.05,
                max_abs_joint_velocity=0.1,
            )


if __name__ == "__main__":
    unittest.main()
