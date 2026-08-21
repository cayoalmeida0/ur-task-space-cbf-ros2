from types import SimpleNamespace
import unittest

import numpy as np

from ur_cbf_control.self_collision_cbf import distances_from_uaibot_structure
from ur_cbf_control.self_collision_cbf import formulate_self_collision_cbf
from ur_cbf_control.self_collision_cbf import SelfCollisionCbfError
from ur_cbf_control.self_collision_cbf import SelfCollisionDistances


class FakeDistanceStructure:
    def __init__(self):
        self.no_items = 2
        self.dist_vect = np.array(((0.08,), (0.03,)))
        self.jac_dist_mat = np.array(((1.0, 0.0), (0.0, -2.0)))
        self.items = (
            SimpleNamespace(
                link_number_1=0,
                link_col_obj_number_1=1,
                link_number_2=3,
                link_col_obj_number_2=0,
            ),
            SimpleNamespace(
                link_number_1=1,
                link_col_obj_number_1=0,
                link_number_2=4,
                link_col_obj_number_2=2,
            ),
        )

    def __getitem__(self, index):
        return self.items[index]


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

    def test_converts_public_uaibot_distance_structure(self):
        result = distances_from_uaibot_structure(
            FakeDistanceStructure(),
            joint_count=2,
        )
        self.assertEqual(result.count, 2)
        self.assertEqual(result.closest_pair, "link_1_obj_0__link_4_obj_2")
        self.assertEqual(result.geometry_source, "uaibot_internal_collision_objects")
        np.testing.assert_allclose(result.distances, (0.08, 0.03))

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
