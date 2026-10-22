"""New-plant admission tests; synthetic files never certify a flight."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_ts20_hold_v1 import task


class Ts20TaskTests(unittest.TestCase):
    def config(self, stage="hover_folded"):
        cfg = task.build_task_config(stage)
        cfg["ts20_inputs"]["oracle_validation"] = str(ROOT / "synthetic_oracle.json")
        cfg["ts20_inputs"]["native_library"] = str(ROOT / "synthetic_native.so")
        cfg["ts20_inputs"]["native_library_sha256"] = "1" * 64
        if stage == "tip_hold":
            cfg["ts20_inputs"]["base_response_prior"] = str(ROOT / "synthetic_prior.json")
        return cfg

    def test_only_reviewed_three_stages_are_enabled(self):
        for stage in task.STAGES:
            cfg = self.config(stage)
            self.assertEqual(task.validate_task_config(cfg), cfg)
        for stage in ("small", "normal", "fast", "response_probe"):
            with self.subTest(stage=stage), self.assertRaises(ValueError):
                task.build_task_config(stage)

    def test_new_model_and_deadline_cannot_be_silently_relaxed(self):
        for key, value in (("model", task.original.MODEL), ("control_period_s", .04),
                           ("feedforward_enabled", True), ("duration_s", 2.)):
            cfg = self.config()
            cfg[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                task.validate_task_config(cfg)
        cfg = self.config()
        cfg["ts20_protocol"]["max_control_interval_s"] = .06
        with self.assertRaises(ValueError):
            task.validate_task_config(cfg)

    def test_hardware_supply_and_load_are_never_inferred_qualified(self):
        qualification = task.hold_protocol()["qualification"]
        self.assertEqual(qualification["hardware_supply_v"], 24)
        self.assertEqual(qualification["identified_supply_v"], 20)
        self.assertFalse(qualification["supply_24v_qualified"])
        self.assertFalse(qualification["actual_aerial_arm_load_qualified"])
        self.assertFalse(task.hold_protocol()["trajectory_stages_enabled"])

    def test_frozen_profile_uses_canonical_types(self):
        cfg = self.config()
        cfg["ts20_protocol"]["whole_declared_window_required"] = 1
        with self.assertRaises(ValueError):
            task.validate_task_config(cfg)
        cfg = self.config()
        cfg["disturbance_enabled"] = 0
        with self.assertRaises(ValueError):
            task.validate_task_config(cfg)

    def test_old_ideal_joint_certificate_cannot_unlock_new_stage(self):
        cfg = self.config("joint_probe")
        cfg["stage_evidence"]["hover_folded"] = str(ROOT / "synthetic_old.json")
        old = {"kind": "x650_native_stage_evidence", "stage": "hover_folded", "complete": True, "passed": True}
        with patch.object(task, "condition", return_value={"test": "new plant"}), \
                patch.object(task, "read", return_value=old), patch.object(Path, "is_file", return_value=True):
            with self.assertRaisesRegex(ValueError, "Old-plant"):
                task.validate_stage_gates(cfg, {"model_sha256": "a" * 64, "source_urdf_sha256": "b" * 64})

    def test_base_prior_does_not_become_a_new_response_gate(self):
        cfg = self.config("tip_hold")
        self.assertEqual(task.required_gates(cfg), ["hover_folded", "joint_probe"])
        self.assertIn("not newly identified", cfg["ts20_protocol"]["base_response_status"])
        cfg["ts20_inputs"]["base_response_prior"] = None
        with self.assertRaisesRegex(ValueError, "base_response_prior"):
            task.validate_task_config(cfg)

    def test_damped_is_a_separate_preset_with_unchanged_acceptance(self):
        old = task.hold_protocol()
        new = task.damped_hold_protocol()
        for key in old:
            if key != "kind":
                self.assertEqual(old[key], new[key])
        self.assertEqual(new["mit_gain_profile"]["kp"], 6.)
        self.assertEqual(new["mit_gain_profile"]["kd"], .25)
        self.assertEqual(new["joint_probe_velocity_max_rad_s"], .2)
        cfg = task.build_damped_task_config()
        cfg["ts20_inputs"]["oracle_validation"] = str(ROOT / "synthetic_damped_oracle.json")
        cfg["ts20_inputs"]["native_library"] = str(ROOT / "synthetic_native.so")
        cfg["ts20_inputs"]["native_library_sha256"] = "1" * 64
        self.assertEqual(task.validate_task_config(cfg), cfg)
        self.assertEqual(task.variant(cfg)["airframe"], 4996)
        self.assertEqual(task.variant(self.config())["airframe"], 4997)
        cfg["model"] = task.MODEL
        with self.assertRaises(ValueError):
            task.validate_task_config(cfg)

    def test_predictor_uses_explicit_bound_gain_and_rejects_cross_variant(self):
        cfg = task.build_damped_task_config()
        cfg["ts20_inputs"]["native_library"] = str(ROOT / "synthetic_native.so")
        cfg["ts20_inputs"]["native_library_sha256"] = "1" * 64
        actual = {"parameters": {"kp": 6., "kd": .25, "bus_period_s": .005}}
        with patch.object(task, "read", return_value=actual), \
                patch("x650_wholebody.ts20_prediction.Ts20PredictionModel") as factory:
            task.predictor_from_bound_plant(cfg, {"test": "motion"})
            factory.assert_called_once_with({"test": "motion"}, kp=6., kd=.25, bus_period_s=.005)
            factory.return_value.enable_native.assert_called_once()
        actual["parameters"]["kd"] = 3276 / 32768
        with patch.object(task, "read", return_value=actual):
            with self.assertRaisesRegex(ValueError, "differs from selected variant"):
                task.predictor_from_bound_plant(cfg, {})


if __name__ == "__main__":
    unittest.main()
