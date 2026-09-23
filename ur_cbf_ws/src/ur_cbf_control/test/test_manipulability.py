import pytest

from ur_cbf_control.manipulability import evaluate_manipulability


def test_evaluate_manipulability_reports_task_jacobian_metrics():
    metrics = evaluate_manipulability(
        [[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.5]]
    )

    assert metrics["sigma_min"] == pytest.approx(0.5)
    assert metrics["sigma_max"] == pytest.approx(2.0)
    assert metrics["condition_number"] == pytest.approx(4.0)
    assert metrics["yoshikawa_index"] == pytest.approx(1.0)
