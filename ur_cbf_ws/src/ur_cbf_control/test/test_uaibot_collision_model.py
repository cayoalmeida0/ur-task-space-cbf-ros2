import unittest

import numpy as np

from ur_cbf_control.uaibot_collision_model import UR3E_UAIBOT_PRIMITIVES
from ur_cbf_control.uaibot_collision_model import UaibotCollisionModelError
from ur_cbf_control.uaibot_collision_model import (
    validate_uaibot_ur3e_collision_model,
)


class Ball:
    pass


class Cylinder:
    pass


class Box:
    pass


class FakeLink:
    def __init__(self):
        self.col_objects = []


def make_robot():
    robot = type("Robot", (), {})()
    robot.links = [FakeLink() for _ in range(6)]
    primitive_types = {"Ball": Ball, "Cylinder": Cylinder, "Box": Box}
    for spec in UR3E_UAIBOT_PRIMITIVES:
        primitive = primitive_types[spec.primitive_type]()
        if spec.primitive_type == "Ball":
            primitive.radius = spec.dimensions[0]
        elif spec.primitive_type == "Cylinder":
            primitive.radius, primitive.height = spec.dimensions
        else:
            primitive.width, primitive.depth, primitive.height = spec.dimensions
        robot.links[spec.link_index].col_objects.append(
            (primitive, np.asarray(spec.htm, dtype=float))
        )
    return robot


class UaibotCollisionModelTest(unittest.TestCase):
    def test_accepts_exact_versioned_geometry(self):
        robot = make_robot()
        validate_uaibot_ur3e_collision_model(robot)
        self.assertEqual(len(UR3E_UAIBOT_PRIMITIVES), 19)

    def test_rejects_changed_primitive_count(self):
        robot = make_robot()
        robot.links[5].col_objects.pop()
        with self.assertRaisesRegex(UaibotCollisionModelError, "Contagem"):
            validate_uaibot_ur3e_collision_model(robot)

    def test_rejects_changed_transform(self):
        robot = make_robot()
        robot.links[2].col_objects[1][1][0, 3] += 1e-3
        with self.assertRaisesRegex(UaibotCollisionModelError, "Transformacao"):
            validate_uaibot_ur3e_collision_model(robot)

    def test_rejects_changed_dimensions(self):
        robot = make_robot()
        robot.links[5].col_objects[3][0].width += 1e-3
        with self.assertRaisesRegex(UaibotCollisionModelError, "Dimensoes"):
            validate_uaibot_ur3e_collision_model(robot)


if __name__ == "__main__":
    unittest.main()
