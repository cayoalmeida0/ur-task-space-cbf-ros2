import unittest
from unittest.mock import patch

import numpy as np

from ur_cbf_control.kinematics import KinematicsError
from ur_cbf_control.kinematics import UaibotKinematics
from ur_cbf_control.kinematics import homogeneous_transform_from_xyz_rpy


class FakeLink:
    def __init__(self, d=0.0):
        self._d = float(d)

    @property
    def d(self):
        return self._d


class FakeRobot:
    def __init__(self, joint_count=3, d_values=None):
        if d_values is None:
            d_values = [0.0] * joint_count
        self.links = [FakeLink(d) for d in d_values]
        self.htm_n_eef = None

    def set_htm_to_eef(self, htm):
        self.htm_n_eef = np.asarray(htm, dtype=float)

    def jac_geo(self, q, axis, mode):
        joint_count = len(self.links)
        jacobian = np.zeros((6, joint_count))
        jacobian[: min(3, joint_count), : min(3, joint_count)] = np.eye(
            min(3, joint_count)
        )
        htm = np.eye(4)
        htm[:3, 3] = (0.1, 0.2, 0.3)
        return jacobian, htm

    def compute_dist_auto(self, **kwargs):
        joint_count = len(self.links)

        class Structure:
            no_items = 1
            dist_vect = np.array(((0.08,),))
            jac_dist_mat = np.ones((1, joint_count))

            def __getitem__(self, index):
                return type(
                    "Item",
                    (),
                    {
                        "link_number_1": 0,
                        "link_col_obj_number_1": 0,
                        "link_number_2": 2,
                        "link_col_obj_number_2": 1,
                    },
                )()

        self.distance_arguments = kwargs
        return Structure()


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
        )
        with patch(
            "ur_cbf_control.kinematics.validate_uaibot_ur3e_collision_model"
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
        self.assertEqual(robot.distance_arguments["mode"], "python")
        self.assertTrue(np.isinf(robot.distance_arguments["max_dist"]))
        self.assertIsNone(robot.distance_arguments["old_dist_struct"])

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

        with patch.dict("sys.modules", {"uaibot": FakeUaibot}):
            adapter = UaibotKinematics.create(
                ur_type="ur3e",
                model_joint_names=("j1", "j2", "j3", "j4", "j5", "j6"),
                eef_offset_xyz=(0.0, 0.0, 0.218),
                eef_offset_rpy=(0.0, 0.0, -np.pi / 2.0),
            )
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
