"""Trajectory admission must never retune the accepted control backend."""
from copy import deepcopy
import unittest
from unittest.mock import patch

from simulations.x650_ts20_figure8_v1 import task


class FrozenTrajectoryTaskTests(unittest.TestCase):
    def setUp(self):
        self.baseline = {"controller": "mpc", "numerical_backend": {"library_sha256": "frozen"},
                         "mpc_config": {"tip_position_weight": 400., "solve_budget_ms": 20.},
                         "feedforward_enabled": False, "pid_profile": "/frozen/pid.json",
                         "ts20_inputs": {"plant_manifest": "/frozen/plant.json"}}
        self.binding = {"path": "/frozen/task.json", "sha256": task.BASELINE_SHA256}
        self.hold = {"tip_hold": {"path": "/frozen/hold.json", "sha256": task.HOLD_CERTIFICATE_SHA256}}
        self.mock = patch.object(task, "baseline_config", side_effect=lambda _: deepcopy(self.baseline))
        self.mock.start()
        self.addCleanup(self.mock.stop)

    def test_normal_changes_only_reference_and_declared_protocol(self):
        cfg = task.build_task_config("normal", self.binding, self.hold)
        self.assertEqual(task.validate_task_config(cfg), cfg)
        for key, value in self.baseline.items():
            self.assertEqual(cfg[key], value)
        self.assertEqual(cfg["duration_s"], 32.)
        self.assertEqual(cfg["trajectory_parameters"]["period_s"], 12.)
        self.assertEqual(cfg["trajectory_parameters"]["amplitude_x"], .25)

    def test_fast_requires_normal_certificate(self):
        cfg = task.build_task_config("fast", self.binding, self.hold)
        with self.assertRaisesRegex(ValueError, "normal is required"):
            task.validate_task_config(cfg)
        cfg["trajectory_gates"]["normal"] = {"path": "/new/normal.json", "sha256": "new"}
        self.assertEqual(task.validate_task_config(cfg)["duration_s"], 24.)
        self.assertEqual(cfg["trajectory_parameters"]["period_s"], 8.)

    def test_rejects_weight_limit_backend_and_trajectory_changes(self):
        for field, key, value in [("mpc_config", "tip_position_weight", 401.),
                                  ("mpc_config", "solve_budget_ms", 100.),
                                  ("numerical_backend", "library_sha256", "other"),
                                  ("trajectory_parameters", "amplitude_x", .075),
                                  ("figure8_protocol", "tip_rms_max_m", .2)]:
            with self.subTest(field=field, key=key):
                cfg = task.build_task_config("normal", self.binding, self.hold)
                cfg[field][key] = value
                with self.assertRaisesRegex(ValueError, "changed"):
                    task.validate_task_config(cfg)

    def test_rejects_omitted_hold_gate(self):
        cfg = task.build_task_config("normal", self.binding, {})
        with self.assertRaisesRegex(ValueError, "Complete hold"):
            task.validate_task_config(cfg)

    def test_rejects_unknown_stage(self):
        with self.assertRaisesRegex(ValueError, "Only normal"):
            task.build_task_config("small", self.binding, self.hold)


if __name__ == "__main__":
    unittest.main()
