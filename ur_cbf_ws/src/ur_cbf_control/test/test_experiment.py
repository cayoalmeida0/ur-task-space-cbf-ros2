from datetime import datetime, timezone
import json
import math
from pathlib import Path
import tempfile
import unittest

from ur_cbf_control.experiment import evaluate_control_timing
from ur_cbf_control.experiment import ExperimentDataError
from ur_cbf_control.experiment import write_experiment_record


class ControlTimingTest(unittest.TestCase):
    def test_slow_simulation_does_not_trigger_simulated_timeout(self):
        timing = evaluate_control_timing(
            start_simulated=10.0,
            current_simulated=20.0,
            start_wall=100.0,
            current_wall=200.0,
            max_simulated=30.0,
            max_wall=180.0,
        )
        self.assertEqual(timing.simulated_seconds, 10.0)
        self.assertEqual(timing.wall_seconds, 100.0)
        self.assertFalse(timing.simulated_limit_reached)
        self.assertFalse(timing.wall_limit_reached)

    def test_simulated_timeout_is_independent_from_wall_time(self):
        timing = evaluate_control_timing(
            start_simulated=10.0,
            current_simulated=40.0,
            start_wall=100.0,
            current_wall=120.0,
            max_simulated=30.0,
            max_wall=180.0,
        )
        self.assertTrue(timing.simulated_limit_reached)
        self.assertFalse(timing.wall_limit_reached)

    def test_wall_timeout_protects_against_extreme_slowdown(self):
        timing = evaluate_control_timing(
            start_simulated=10.0,
            current_simulated=11.0,
            start_wall=100.0,
            current_wall=280.0,
            max_simulated=30.0,
            max_wall=180.0,
        )
        self.assertFalse(timing.simulated_limit_reached)
        self.assertTrue(timing.wall_limit_reached)

    def test_rejects_clock_rollback(self):
        with self.assertRaisesRegex(ExperimentDataError, "simulado retrocedeu"):
            evaluate_control_timing(
                start_simulated=10.0,
                current_simulated=9.0,
                start_wall=100.0,
                current_wall=101.0,
                max_simulated=30.0,
                max_wall=180.0,
            )

    def test_rejects_non_finite_time(self):
        with self.assertRaisesRegex(ExperimentDataError, "finito"):
            evaluate_control_timing(
                start_simulated=10.0,
                current_simulated=math.nan,
                start_wall=100.0,
                current_wall=101.0,
                max_simulated=30.0,
                max_wall=180.0,
            )


class ExperimentRecordTest(unittest.TestCase):
    def test_writes_structured_json_with_safe_filename(self):
        instant = datetime(2026, 8, 20, 16, 30, tzinfo=timezone.utc)
        with tempfile.TemporaryDirectory() as temporary:
            output = write_experiment_record(
                record={"result": "approved", "metrics": {"error": 0.001}},
                directory=temporary,
                experiment_id="cartesian test/001",
                recorded_at=instant,
            )
            self.assertEqual(
                output.name,
                "cartesian_test_001_20260820T163000.000000Z.json",
            )
            payload = json.loads(Path(output).read_text(encoding="utf-8"))
            self.assertEqual(payload["result"], "approved")
            self.assertEqual(payload["recorded_at_utc"], "2026-08-20T16:30:00Z")

    def test_rejects_non_finite_json_value(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(ExperimentDataError, "JSON valido"):
                write_experiment_record(
                    record={"error": math.nan},
                    directory=temporary,
                    experiment_id="test_001",
                )

    def test_rejects_naive_timestamp(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(ExperimentDataError, "fuso horario"):
                write_experiment_record(
                    record={"result": "approved"},
                    directory=temporary,
                    experiment_id="test_001",
                    recorded_at=datetime(2026, 8, 20),
                )


if __name__ == "__main__":
    unittest.main()
