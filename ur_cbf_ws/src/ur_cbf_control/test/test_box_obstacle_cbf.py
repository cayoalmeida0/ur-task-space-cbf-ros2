import numpy as np
import pytest

from ur_cbf_control.box_obstacle_cbf import (
    _signed_distance_and_gradient,
    evaluate_box_obstacle_distances,
    formulate_box_obstacle_cbf,
)


class Ball:
    radius = 0.01


class Link:
    def __init__(self):
        self.col_objects = [(Ball(), np.eye(4))]


class Robot:
    links = [Link()]

    def jac_geo(self, _q, _axis, mode="python"):
        assert mode == "python"
        jacobian = np.zeros((6, 1))
        jacobian[0, 0] = 1.0
        transform = np.eye(4)
        transform[0, 3] = 0.10
        return [jacobian], [transform]


def test_box_obstacle_distance_has_a_differential_constraint():
    distances = evaluate_box_obstacle_distances(
        Robot(),
        [0.0],
        center=[0.0, 0.0, 0.0],
        size=[0.04, 0.04, 0.04],
        geometry_source="test",
    )
    constraints = formulate_box_obstacle_cbf(
        distances,
        safe_distance=0.005,
        gain=5.0,
    )

    assert distances.count == 1
    assert distances.distances[0] == pytest.approx(0.07)
    np.testing.assert_allclose(distances.first_witness_points[0], [0.09, 0.0, 0.0])
    np.testing.assert_allclose(distances.second_witness_points[0], [0.02, 0.0, 0.0])
    assert np.linalg.norm(
        distances.first_witness_points[0] - distances.second_witness_points[0]
    ) == pytest.approx(distances.distances[0])
    assert constraints.matrix.shape == (1, 1)
    assert constraints.matrix[0, 0] > 0.0
    assert constraints.lower_bound[0] < 0.0


def test_box_signed_distance_uses_the_closest_face_inside_the_cube():
    distance, gradient, closest = _signed_distance_and_gradient(
        np.array((0.0, 0.0, 0.015)),
        center=np.zeros(3),
        half_extents=np.array((0.02, 0.02, 0.02)),
    )

    assert distance == pytest.approx(-0.005)
    np.testing.assert_allclose(gradient, (0.0, 0.0, 1.0))
    np.testing.assert_allclose(closest, (0.0, 0.0, 0.02))


def test_box_obstacle_can_exclude_intentional_fingertip_contact_pairs():
    robot = Robot()
    robot.links[0].col_objects.append((Ball(), np.eye(4)))

    distances = evaluate_box_obstacle_distances(
        robot,
        [0.0],
        center=[0.0, 0.0, 0.0],
        size=[0.04, 0.04, 0.04],
        geometry_source="test",
        excluded_pair_labels=["link_0_obj_0"],
    )

    assert distances.count == 1
    assert distances.pair_labels == ("link_0_obj_1__cube",)
