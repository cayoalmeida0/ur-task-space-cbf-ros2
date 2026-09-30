import numpy as np
import pytest

from ur_cbf_control.collision_spheres import (
    RobotCollisionSpheresError,
    evaluate_robot_collision_spheres,
)


class Ball:
    radius = 0.02


class Box:
    width = 0.04
    depth = 0.06
    height = 0.08


class Link:
    def __init__(self, primitive, attached):
        self.col_objects = [(primitive, attached)]


class Robot:
    def __init__(self):
        attached_ball = np.eye(4)
        attached_ball[:3, 3] = [0.1, 0.0, 0.0]
        attached_box = np.eye(4)
        attached_box[:3, 3] = [0.05, 0.0, 0.0]
        self.links = [Link(Ball(), attached_ball), Link(Box(), attached_box)]

    def jac_geo(self, _q, _axis, mode="python"):
        assert mode == "python"
        first = np.eye(4)
        first[:3, 3] = [1.0, 2.0, 3.0]
        first[:3, :3] = [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        second = np.eye(4)
        second[:3, 3] = [-1.0, 0.0, 0.0]
        return [np.zeros((6, 2)), np.zeros((6, 2))], [first, second]


def test_collision_spheres_use_dh_and_attached_transforms_and_real_dimensions():
    spheres = evaluate_robot_collision_spheres(Robot(), [0.0, 0.0])

    assert spheres.labels == ("link_0_obj_0", "link_1_obj_0")
    np.testing.assert_allclose(spheres.centers, [[1.0, 2.1, 3.0], [-0.95, 0.0, 0.0]])
    np.testing.assert_allclose(
        spheres.radii,
        [0.02, 0.5 * np.sqrt(0.04**2 + 0.06**2 + 0.08**2)],
    )


@pytest.mark.parametrize(
    "configuration",
    [[0.0], [0.0, np.nan]],
)
def test_collision_spheres_reject_invalid_configuration(configuration):
    with pytest.raises(RobotCollisionSpheresError):
        evaluate_robot_collision_spheres(Robot(), configuration)


class Unsupported:
    pass


def test_collision_spheres_reject_unsupported_primitive():
    robot = Robot()
    robot.links[0].col_objects = [(Unsupported(), np.eye(4))]
    with pytest.raises(RobotCollisionSpheresError):
        evaluate_robot_collision_spheres(robot, [0.0, 0.0])
