import unittest
from unittest.mock import patch

import numpy as np

from ur_cbf_control.kinematics import KinematicsError
from ur_cbf_control.kinematics import UaibotKinematics
from ur_cbf_control.kinematics import homogeneous_transform_from_xyz_rpy


class FakeRobot:
    def __init__(self, joint_count=3):
        self.links = [object() for _ in range(joint_count)]
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


if __name__ == "__main__":
    unittest.main()
