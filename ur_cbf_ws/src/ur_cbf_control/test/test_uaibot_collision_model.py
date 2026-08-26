import unittest

import numpy as np

from ur_cbf_control.uaibot_collision_model import (
    configure_ur3e_rg2_project_collision_model,
)
from ur_cbf_control.uaibot_collision_model import UR3E_RG2_PROJECT_PRIMITIVES
from ur_cbf_control.uaibot_collision_model import UR3E_UAIBOT_PRIMITIVES
from ur_cbf_control.uaibot_collision_model import UaibotCollisionModelError
from ur_cbf_control.uaibot_collision_model import (
    validate_uaibot_ur3e_factory_model,
)
from ur_cbf_control.uaibot_collision_model import (
    validate_ur3e_rg2_project_collision_model,
)


class Primitive:
    def __init__(self, *, htm, name, color, opacity, **dimensions):
        self.htm = np.asarray(htm, dtype=float)
        self.name = name
        self.color = color
        self.opacity = opacity
        for key, value in dimensions.items():
            setattr(self, key, value)


class Ball(Primitive):
    pass


class Cylinder(Primitive):
    pass


class Box(Primitive):
    pass


class FakeUaibot:
    Ball = Ball
    Cylinder = Cylinder
    Box = Box


class FakeLink:
    def __init__(self):
        self._col_objects = []

    @property
    def col_objects(self):
        return self._col_objects

    def attach_col_object(self, primitive, htm):
        self._col_objects.append([primitive, np.asarray(htm, dtype=float)])


def _primitive_from_spec(spec):
    common = {
        "htm": np.asarray(spec.htm, dtype=float),
        "name": spec.identifier,
        "color": "blue",
        "opacity": 0.3,
    }
    if spec.primitive_type == "Ball":
        return Ball(radius=spec.dimensions[0], **common)
    if spec.primitive_type == "Cylinder":
        return Cylinder(
            radius=spec.dimensions[0], height=spec.dimensions[1], **common
        )
    return Box(
        width=spec.dimensions[0],
        depth=spec.dimensions[1],
        height=spec.dimensions[2],
        **common,
    )


def make_factory_robot():
    robot = type("Robot", (), {})()
    robot.links = [FakeLink() for _ in range(6)]
    for spec in UR3E_UAIBOT_PRIMITIVES:
        robot.links[spec.link_index].attach_col_object(
            _primitive_from_spec(spec),
            np.asarray(spec.htm, dtype=float),
        )
    return robot


class UaibotCollisionModelTest(unittest.TestCase):
    def test_accepts_pinned_factory_geometry(self):
        robot = make_factory_robot()
        validate_uaibot_ur3e_factory_model(robot)
        self.assertEqual(len(UR3E_UAIBOT_PRIMITIVES), 19)

    def test_project_arm_specs_are_independent_from_factory_contract(self):
        for index, (factory_spec, project_spec) in enumerate(zip(
            UR3E_UAIBOT_PRIMITIVES[:13],
            UR3E_RG2_PROJECT_PRIMITIVES[:13],
        )):
            self.assertIsNot(factory_spec, project_spec)
            if index not in {4, 5}:
                self.assertEqual(factory_spec, project_spec)

        for index in (4, 5):
            self.assertAlmostEqual(UR3E_UAIBOT_PRIMITIVES[index].htm[2][3], 0.05)
            self.assertAlmostEqual(
                UR3E_RG2_PROJECT_PRIMITIVES[index].htm[2][3],
                0.027,
            )

    def test_replaces_generic_gripper_with_rg2_capsule(self):
        robot = make_factory_robot()
        configure_ur3e_rg2_project_collision_model(robot, FakeUaibot)
        validate_ur3e_rg2_project_collision_model(robot)

        self.assertEqual(len(UR3E_RG2_PROJECT_PRIMITIVES), 16)
        self.assertEqual(
            tuple(len(link.col_objects) for link in robot.links),
            (1, 3, 3, 2, 2, 5),
        )
        distal = robot.links[5].col_objects
        self.assertEqual(
            tuple(type(item[0]).__name__ for item in distal),
            ("Cylinder", "Cylinder", "Cylinder", "Ball", "Ball"),
        )
        self.assertAlmostEqual(distal[2][0].radius, 0.090)
        self.assertAlmostEqual(distal[2][0].height, 0.110)
        self.assertAlmostEqual(distal[3][0].radius, 0.090)
        self.assertAlmostEqual(distal[4][0].radius, 0.090)
        self.assertAlmostEqual(distal[2][1][2, 3], 0.110)
        self.assertAlmostEqual(distal[3][1][2, 3], 0.055)
        self.assertAlmostEqual(distal[4][1][2, 3], 0.165)

    def test_rejects_changed_factory_primitive_count(self):
        robot = make_factory_robot()
        robot.links[5].col_objects.pop()
        with self.assertRaisesRegex(UaibotCollisionModelError, "Contagem"):
            configure_ur3e_rg2_project_collision_model(robot, FakeUaibot)

    def test_rejects_changed_project_transform(self):
        robot = make_factory_robot()
        configure_ur3e_rg2_project_collision_model(robot, FakeUaibot)
        robot.links[5].col_objects[2][1][2, 3] += 1e-3
        with self.assertRaisesRegex(UaibotCollisionModelError, "Transformacao"):
            validate_ur3e_rg2_project_collision_model(robot)

    def test_rejects_changed_project_dimensions(self):
        robot = make_factory_robot()
        configure_ur3e_rg2_project_collision_model(robot, FakeUaibot)
        robot.links[5].col_objects[2][0].radius += 1e-3
        with self.assertRaisesRegex(UaibotCollisionModelError, "Dimensoes"):
            validate_ur3e_rg2_project_collision_model(robot)


if __name__ == "__main__":
    unittest.main()
