import numpy as np
import pytest

from ur_cbf_control.cylinder_obstacle_cbf import (
    evaluate_cylinder_obstacle_distances,
    formulate_cylinder_obstacle_cbf,
)
from ur_cbf_control.cylinder_obstacle_cbf import _signed_distance_and_gradient


class Cylinder:
    radius = 0.02
    height = 0.10


class Link:
    def __init__(self):
        self.col_objects = [(Cylinder(), np.eye(4))]


class Robot:
    links = [Link()]

    def jac_geo(self, _q, _axis, mode="python"):
        assert mode == "python"
        jacobian = np.zeros((6, 1))
        jacobian[0, 0] = 1.0
        transform = np.eye(4)
        transform[0, 3] = 0.30
        transform[2, 3] = 0.20
        return [jacobian], [transform]


class RobotNearTableTop(Robot):
    def jac_geo(self, _q, _axis, mode="python"):
        jacobians, transforms = super().jac_geo(_q, _axis, mode)
        transforms[0][0, 3] = 0.09
        return jacobians, transforms


class RobotAboveTableCenter(Robot):
    def jac_geo(self, _q, _axis, mode="python"):
        jacobians, transforms = super().jac_geo(_q, _axis, mode)
        transforms[0][0, 3] = 0.0
        transforms[0][2, 3] = 0.20
        return jacobians, transforms


def test_cylinder_obstacle_distance_has_a_differential_constraint():
    distances = evaluate_cylinder_obstacle_distances(
        Robot(),
        [0.0],
        center_xy=[0.0, 0.0],
        radius=0.08,
        height=0.15,
        geometry_source="test",
    )
    constraints = formulate_cylinder_obstacle_cbf(
        distances,
        safe_distance=0.03,
        gain=5.0,
    )

    assert distances.count == 1
    assert distances.distances[0] > 0.0
    assert constraints.matrix.shape == (1, 1)
    assert constraints.lower_bound[0] < 0.0
    assert constraints.matrix[0, 0] > 0.0


def test_table_witness_runs_from_robot_volume_surface_to_physical_top():
    distances = evaluate_cylinder_obstacle_distances(
        RobotAboveTableCenter(),
        [0.0],
        center_xy=[0.0, 0.0],
        radius=0.08,
        height=0.15,
        geometry_source="test",
    )

    primitive_radius = np.sqrt(0.02**2 + (0.10 / 2.0) ** 2)
    np.testing.assert_allclose(
        distances.first_witness_points[0],
        [0.0, 0.0, 0.20 - primitive_radius],
    )
    np.testing.assert_allclose(
        distances.second_witness_points[0],
        [0.0, 0.0, 0.15],
    )
    assert np.linalg.norm(
        distances.first_witness_points[0] - distances.second_witness_points[0]
    ) == pytest.approx(abs(distances.distances[0]))
    assert distances.distances[0] == pytest.approx(0.05 - primitive_radius)


def test_cylinder_signed_distance_uses_the_closest_cap_inside_the_table():
    distance, gradient = _signed_distance_and_gradient(
        np.array((0.02, 0.0, 0.145)),
        center_xy=np.array((0.0, 0.0)),
        radius=0.10,
        z_min=0.0,
        z_max=0.15,
    )

    assert distance == pytest.approx(-0.005)
    np.testing.assert_allclose(gradient, (0.0, 0.0, 1.0))


def test_cylinder_obstacle_inflates_the_table_caps_by_the_volume_radius():
    distances = evaluate_cylinder_obstacle_distances(
        RobotNearTableTop(),
        [0.0],
        center_xy=[0.0, 0.0],
        radius=0.08,
        height=0.15,
        geometry_source="test",
    )

    assert distances.distances[0] < 0.0
