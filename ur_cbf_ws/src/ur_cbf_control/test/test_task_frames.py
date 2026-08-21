import math
import unittest

from ur_cbf_control.task_frames import SUPPORTED_ONROBOT_TYPES
from ur_cbf_control.task_frames import get_task_frame_spec


class TaskFrameSpecTest(unittest.TestCase):
    def test_supported_grippers_are_explicit(self):
        self.assertEqual(SUPPORTED_ONROBOT_TYPES, ("rg2", "rg6"))

    def test_rg2_controls_the_center_between_closed_fingers(self):
        spec = get_task_frame_spec("rg2")
        self.assertEqual(spec.controlled_frame, "gripper_tcp")
        self.assertEqual(spec.eef_offset_xyz, (0.0, 0.0, 0.218))
        self.assertEqual(spec.eef_offset_rpy, (0.0, 0.0, -math.pi / 2.0))

    def test_rg6_uses_its_larger_tcp_offset(self):
        spec = get_task_frame_spec("rg6")
        self.assertEqual(spec.controlled_frame, "gripper_tcp")
        self.assertEqual(spec.eef_offset_xyz, (0.0, 0.0, 0.268))

    def test_unknown_gripper_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "nao suportado"):
            get_task_frame_spec("unknown")


if __name__ == "__main__":
    unittest.main()
