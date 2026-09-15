import numpy as np
import pytest

from ur_cbf_control.workspace_boundary import WorkspaceBoundaryCbfError
from ur_cbf_control.workspace_boundary import formulate_workspace_boundary_cbf
from ur_cbf_control.workspace_boundary import workspace_boundary_vertices


BOUNDS = [-0.45, 0.45, -0.55, 0.55, 0.05, 0.90]


def test_workspace_cbf_has_six_plane_constraints_in_axis_order():
    constraints = formulate_workspace_boundary_cbf(
        [0.0, -0.44, 0.69],
        np.eye(3, 6),
        bounds=BOUNDS,
        safety_margin=0.05,
        gain=7.0,
    )

    assert constraints.matrix.shape == (6, 6)
    expected_matrix = np.zeros((6, 6))
    expected_matrix[0::2, :3] = np.eye(3)
    expected_matrix[1::2, :3] = -np.eye(3)
    np.testing.assert_allclose(constraints.matrix, expected_matrix)
    np.testing.assert_allclose(
        constraints.barrier_values,
        [0.55, 0.55, 0.21, 1.09, 0.44, 0.16],
    )
    np.testing.assert_allclose(
        constraints.lower_bound,
        -7.0 * constraints.barrier_values,
    )


def test_workspace_vertices_form_closed_box():
    vertices = workspace_boundary_vertices(BOUNDS)
    assert vertices.shape == (8, 3)
    np.testing.assert_allclose(vertices[0], [-0.45, -0.55, 0.05])
    np.testing.assert_allclose(vertices[6], [0.45, 0.55, 0.90])


@pytest.mark.parametrize(
    "bounds",
    [
        [-0.1, -0.1, -0.7, 0.7, 0.2, 0.9],
        [-0.1, 0.1, -0.7, 0.7, 0.2, 0.9],
    ],
)
def test_workspace_cbf_rejects_invalid_or_too_large_margin(bounds):
    with pytest.raises(WorkspaceBoundaryCbfError):
        formulate_workspace_boundary_cbf(
            [0.0, 0.0, 0.5],
            np.zeros((3, 6)),
            bounds=bounds,
            safety_margin=0.05,
            gain=7.0,
        )
