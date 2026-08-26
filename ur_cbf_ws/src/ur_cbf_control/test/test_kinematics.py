import unittest
from unittest.mock import patch

import numpy as np

from ur_cbf_control.kinematics import KinematicsError
from ur_cbf_control.kinematics import UaibotKinematics
from ur_cbf_control.kinematics import homogeneous_transform_from_xyz_rpy


class FakeLink:
    def __init__(self, d=0.0):
        self._d = float(d)
        self._col_objects = []

    @property
    def d(self):
        return self._d

    @property
    def col_objects(self):
        return self._col_objects


class FakePrimitive:
    def __init__(self):
        self.htm = np.eye(4)

    def copy(self):
        return FakePrimitive()

    def set_ani_frame(self, htm):
        self.htm = np.asarray(htm, dtype=float)


class FakeDistanceUtils:
    call_kwargs = None

    @classmethod
    def compute_dist(cls, _first, _second, **kwargs):
        cls.call_kwargs = kwargs
        return (
            np.zeros((3, 1)),
            np.array(((0.08,), (0.0,), (0.0,))),
            0.08,
            (1.0, 0.0),
        )


class ThreeValueDistanceUtils(FakeDistanceUtils):
    @classmethod
    def compute_dist(cls, _first, _second, **kwargs):
        cls.call_kwargs = kwargs
        return np.zeros((3, 1)), np.ones((3, 1)), 1.0


class FakeRobot:
    def __init__(self, joint_count=3, d_values=None):
        if d_values is None:
            d_values = [0.0] * joint_count
        self.links = [FakeLink(d) for d in d_values]
        if joint_count >= 3:
            self.links[0]._col_objects.append((FakePrimitive(), np.eye(4)))
            self.links[2]._col_objects.append((FakePrimitive(), np.eye(4)))
        self.htm_n_eef = None

    def set_htm_to_eef(self, htm):
        self.htm_n_eef = np.asarray(htm, dtype=float)

    def jac_geo(self, q, axis, mode):
        joint_count = len(self.links)
        if axis == "dh":
            jacobians = [np.zeros((6, joint_count)) for _ in self.links]
            jacobians[0][0, 0] = 1.0
            return (
                jacobians,
                [np.eye(4) for _ in self.links],
            )
        jacobian = np.zeros((6, joint_count))
        jacobian[: min(3, joint_count), : min(3, joint_count)] = np.eye(
            min(3, joint_count)
        )
        htm = np.eye(4)
        htm[:3, 3] = (0.1, 0.2, 0.3)
        return jacobian, htm

class UaibotKinematicsTest(unittest.TestCase):
    def test_applies_explicit_end_effector_offset(self):
        robot = FakeRobot(joint_count=2)
        UaibotKinematics(
            robot=robot,
            model_joint_names=("joint_a", "joint_b"),
            eef_offset_xyz=(0.0, 0.0, 0.15),
            eef_offset_rpy=(0.0, 0.0, -np.pi / 2.0),
            mode="python",
        )
        np.testing.assert_allclose(robot.htm_n_eef[:3, 3], (0.0, 0.0, 0.15))
        np.testing.assert_allclose(
            robot.htm_n_eef[:3, :3],
            ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            atol=1e-12,
        )

    def test_rpy_transform_uses_urdf_fixed_axis_convention(self):
        transform = homogeneous_transform_from_xyz_rpy(
            (0.1, 0.2, 0.3),
            (np.pi / 2.0, 0.0, 0.0),
        )
        np.testing.assert_allclose(transform[:3, 3], (0.1, 0.2, 0.3))
        np.testing.assert_allclose(
            transform[:3, :3],
            ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),
            atol=1e-12,
        )

    def test_evaluate_derives_jacobian_width_from_model(self):
        adapter = UaibotKinematics(
            robot=FakeRobot(joint_count=4),
            model_joint_names=("j1", "j2", "j3", "j4"),
            eef_offset_xyz=(0.0, 0.0, 0.0),
            eef_offset_rpy=(0.0, 0.0, 0.0),
        )
        state = adapter.evaluate((0.0, 0.0, 0.0, 0.0))
        self.assertEqual(state.position, (0.1, 0.2, 0.3))
        self.assertEqual(state.translational_jacobian.shape, (3, 4))

    def test_evaluate_self_collision_uses_all_nonadjacent_uaibot_pairs(self):
        robot = FakeRobot(joint_count=4)
        adapter = UaibotKinematics(
            robot=robot,
            model_joint_names=("j1", "j2", "j3", "j4"),
            eef_offset_xyz=(0.0, 0.0, 0.0),
            eef_offset_rpy=(0.0, 0.0, 0.0),
            mode="python",
            distance_utils=FakeDistanceUtils,
        )
        with patch(
            "ur_cbf_control.kinematics."
            "validate_ur3e_rg2_project_collision_model"
        ) as validate_geometry:
            distances = adapter.evaluate_self_collision(
                (0.0, 0.0, 0.0, 0.0),
                tolerance=1e-3,
                max_iterations=15,
            )
            adapter.evaluate_self_collision(
                (0.0, 0.0, 0.0, 0.0),
                tolerance=1e-3,
                max_iterations=15,
            )
        validate_geometry.assert_called_once_with(robot)
        self.assertEqual(distances.count, 1)
        self.assertAlmostEqual(distances.minimum_distance, 0.08)
        self.assertAlmostEqual(distances.jacobian[0, 0], -1.0)
        self.assertEqual(FakeDistanceUtils.call_kwargs["mode"], "python")
        self.assertEqual(FakeDistanceUtils.call_kwargs["tol"], 1e-3)
        self.assertEqual(FakeDistanceUtils.call_kwargs["no_iter_max"], 15)

    def test_rejects_unexpected_uaibot_distance_return_contract(self):
        robot = FakeRobot(joint_count=3)
        adapter = UaibotKinematics(
            robot=robot,
            model_joint_names=("j1", "j2", "j3"),
            eef_offset_xyz=(0.0, 0.0, 0.0),
            eef_offset_rpy=(0.0, 0.0, 0.0),
            mode="python",
            distance_utils=ThreeValueDistanceUtils,
        )
        with patch(
            "ur_cbf_control.kinematics."
            "validate_ur3e_rg2_project_collision_model"
        ):
            with self.assertRaisesRegex(KinematicsError, "quatro valores"):
                adapter.evaluate_self_collision((0.0, 0.0, 0.0))

    def test_rejects_joint_count_mismatch(self):
        with self.assertRaisesRegex(KinematicsError, "Quantidade"):
            UaibotKinematics(
                robot=FakeRobot(joint_count=3),
                model_joint_names=("joint_a", "joint_b"),
                eef_offset_xyz=(0.0, 0.0, 0.0),
                eef_offset_rpy=(0.0, 0.0, 0.0),
            )

    def test_rejects_non_finite_configuration(self):
        adapter = UaibotKinematics(
            robot=FakeRobot(joint_count=2),
            model_joint_names=("joint_a", "joint_b"),
            eef_offset_xyz=(0.0, 0.0, 0.0),
            eef_offset_rpy=(0.0, 0.0, 0.0),
        )
        with self.assertRaisesRegex(KinematicsError, "NaN"):
            adapter.evaluate((0.0, np.nan))

    def test_rejects_model_without_explicit_adapter(self):
        class FakeRobotFactory:
            @staticmethod
            def create_ur_ur3e(**kwargs):
                return FakeRobot(joint_count=2)

        class FakeUaibot:
            Robot = FakeRobotFactory
            Utils = FakeDistanceUtils

        with patch.dict("sys.modules", {"uaibot": FakeUaibot}):
            with self.assertRaisesRegex(KinematicsError, "ainda nao possui"):
                UaibotKinematics.create(
                    ur_type="ur5e",
                    model_joint_names=("joint_a", "joint_b"),
                    eef_offset_xyz=(0.0, 0.0, 0.0),
                    eef_offset_rpy=(0.0, 0.0, 0.0),
                )

    def test_rejects_invalid_end_effector_orientation(self):
        with self.assertRaisesRegex(KinematicsError, "eef_offset_rpy"):
            UaibotKinematics(
                robot=FakeRobot(joint_count=2),
                model_joint_names=("joint_a", "joint_b"),
                eef_offset_xyz=(0.0, 0.0, 0.0),
                eef_offset_rpy=(0.0, np.nan, 0.0),
            )

    @staticmethod
    def _create_with_fake_ur3e(d5):
        robot = FakeRobot(
            joint_count=6,
            d_values=(0.15185, 0.0, 0.0, 0.13105, d5, 0.0921),
        )

        class FakeRobotFactory:
            @staticmethod
            def create_ur_ur3e(**kwargs):
                return robot

        class FakeUaibot:
            Robot = FakeRobotFactory
            Utils = FakeDistanceUtils

        with patch.dict("sys.modules", {"uaibot": FakeUaibot}), patch(
            "ur_cbf_control.kinematics."
            "configure_ur3e_rg2_project_collision_model"
        ) as configure_geometry:
            adapter = UaibotKinematics.create(
                ur_type="ur3e",
                model_joint_names=("j1", "j2", "j3", "j4", "j5", "j6"),
                eef_offset_xyz=(0.0, 0.0, 0.218),
                eef_offset_rpy=(0.0, 0.0, -np.pi / 2.0),
            )
        configure_geometry.assert_called_once_with(robot, FakeUaibot)
        return robot, adapter

    def test_corrects_known_uaibot_ur3e_d5_value(self):
        robot, adapter = self._create_with_fake_ur3e(0.08535 + 0.02)
        self.assertAlmostEqual(robot.links[4].d, 0.08535)
        self.assertEqual(adapter.model_name, "ur3e_official_ros2_description")
        self.assertEqual(adapter.requested_mode, "auto")
        self.assertEqual(adapter.mode, "python")
        self.assertEqual(len(adapter.model_corrections), 1)
        self.assertEqual(
            adapter.model_corrections[0].as_record()["parameter"],
            "wrist_2_d_m",
        )

    def test_preserves_model_that_already_uses_official_d5(self):
        robot, adapter = self._create_with_fake_ur3e(0.08535)
        self.assertAlmostEqual(robot.links[4].d, 0.08535)
        self.assertEqual(adapter.model_corrections, ())

    def test_rejects_unknown_uaibot_ur3e_d5_value(self):
        with self.assertRaisesRegex(KinematicsError, "d5 inesperado"):
            self._create_with_fake_ur3e(0.09535)

    def test_rejects_explicit_cpp_mode_for_corrected_ur3e(self):
        class FakeRobotFactory:
            @staticmethod
            def create_ur_ur3e(**kwargs):
                return FakeRobot(joint_count=6)

        class FakeUaibot:
            Robot = FakeRobotFactory

        with patch.dict("sys.modules", {"uaibot": FakeUaibot}):
            with self.assertRaisesRegex(KinematicsError, "c\\+\\+"):
                UaibotKinematics.create(
                    ur_type="ur3e",
                    model_joint_names=("j1", "j2", "j3", "j4", "j5", "j6"),
                    eef_offset_xyz=(0.0, 0.0, 0.218),
                    eef_offset_rpy=(0.0, 0.0, -np.pi / 2.0),
                    mode="c++",
                )


if __name__ == "__main__":
    unittest.main()
