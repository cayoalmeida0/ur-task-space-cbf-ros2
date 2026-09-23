import numpy as np
import pytest

from ur_cbf_control.cylinder_obstacle_cbf import (
    evaluate_cylinder_obstacle_distances,
    formulate_cylinder_obstacle_cbf,
)


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
