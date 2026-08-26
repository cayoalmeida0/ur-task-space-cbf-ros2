import unittest

import numpy as np

from ur_cbf_control.self_collision_cbf import formulate_self_collision_cbf
from ur_cbf_control.self_collision_cbf import SelfCollisionCbfError
from ur_cbf_control.self_collision_cbf import SelfCollisionDistances

class SelfCollisionCbfTest(unittest.TestCase):
    def test_formulates_first_order_kinematic_cbf(self):
        distances = SelfCollisionDistances(
            distances=np.array((0.08, 0.03)),
            jacobian=np.array(((1.0, 0.0), (0.0, -2.0))),
            pair_labels=("pair_a", "pair_b"),
            geometry_source="test",
        )
        constraints = formulate_self_collision_cbf(
            distances,
            safe_distance=0.05,
            gain=4.0,
        )
        np.testing.assert_allclose(constraints.barrier_values, (0.03, -0.02))
        np.testing.assert_allclose(constraints.lower_bound, (-0.12, 0.08))
        np.testing.assert_allclose(constraints.matrix, distances.jacobian)
        self.assertEqual(constraints.closest_pair, "pair_b")
        self.assertAlmostEqual(constraints.minimum_distance, 0.03)

    def test_rejects_negative_distance(self):
        distances = SelfCollisionDistances(
            distances=np.array((-0.01,)),
            jacobian=np.array(((1.0,),)),
            pair_labels=("pair",),
            geometry_source="test",
        )
        with self.assertRaisesRegex(SelfCollisionCbfError, "nao negativas"):
            formulate_self_collision_cbf(
                distances,
                safe_distance=0.05,
                gain=1.0,
            )

    def test_rejects_inconsistent_shapes(self):
        distances = SelfCollisionDistances(
            distances=np.array((0.1, 0.2)),
            jacobian=np.array(((1.0,),)),
            pair_labels=("a", "b"),
            geometry_source="test",
        )
        with self.assertRaisesRegex(SelfCollisionCbfError, "Quantidade"):
            formulate_self_collision_cbf(
                distances,
                safe_distance=0.05,
                gain=1.0,
            )


if __name__ == "__main__":
    unittest.main()
