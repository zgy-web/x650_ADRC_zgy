"""Reject unrelated or modified native evidence before commanding a flight."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("x650_task", ROOT/"simulations/x650_wholebody_mpc_v1/task.py")
task = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)


class TaskTests(unittest.TestCase):
    def test_platform_owns_user_scenario_recording_directory(self):
        from amsim.scenario_registry import get
        selected = get("x650_wholebody_mpc_v1")
        self.assertEqual(selected.source, "user")
        self.assertEqual(selected.module, "simulations/x650_wholebody_mpc_v1/run_case.py")

    def test_commissioning_has_no_circular_requirement_and_formal_has_gates(self):
        self.assertEqual(task.required_gates(task.build_task_config()), [])
        cfg = task.build_task_config("normal", "mpc")
        self.assertEqual(cfg["trajectory_parameters"]["period_s"], 12.)
        self.assertIn("pid_baseline", task.required_gates(cfg))
        self.assertIn("response_identification", task.required_gates(cfg))
        self.assertIn("small", task.required_gates(cfg))
        self.assertIn("normal", task.required_gates(task.build_task_config("fast")))

    def test_caller_cannot_silently_change_physical_experiment(self):
        cfg = task.build_task_config()
        cfg["model"] = "x500_underfold"
        with self.assertRaises(ValueError): task.validate_task_config(cfg)
        with self.assertRaises(ValueError): task.build_task_config("hover_folded", "mpc")

    def test_archived_tasks_default_to_no_diagnostic_and_only_reviewed_hover_is_allowed(self):
        cfg = task.build_task_config()
        del cfg["diagnostic_profile"]
        self.assertIsNone(task.validate_task_config(cfg)["diagnostic_profile"])
        from diagnostic_parameters import PROFILE
        cfg["diagnostic_profile"] = PROFILE
        self.assertEqual(task.validate_task_config(cfg)["diagnostic_profile"], PROFILE)
        other = task.build_task_config("joint_probe")
        other["diagnostic_profile"] = PROFILE
        with self.assertRaises(ValueError):
            task.validate_task_config(other)

    def test_diagnostic_flags_cannot_unlock_later_canonical_stage(self):
        cfg = task.build_task_config("joint_probe")
        provenance = {"model_sha256": "a"*64, "source_urdf_sha256": "b"*64}
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            raw = folder/"native.csv"
            raw.write_text("synthetic fixture")
            for gate in task.required_gates(cfg):
                path = folder/(gate+".json")
                path.write_text(json.dumps(dict(kind="x650_native_stage_evidence", stage=gate, complete=True, passed=True,
                    simulation_backend="gazebo_px4", **provenance, diagnostic_only=True,
                    artifacts={raw.name: task.sha(raw)})))
                cfg["stage_evidence"][gate] = str(path)
            with self.assertRaisesRegex(ValueError, "Invalid or mismatched"):
                task.validate_stage_gates(cfg, provenance)

    def test_fixed_hover_reference_is_explicit_and_archives_remain_compatible(self):
        cfg = task.build_task_config()
        del cfg["hover_reference_profile"]
        self.assertIsNone(task.validate_task_config(cfg)["hover_reference_profile"])
        cfg["hover_reference_profile"] = {"name": "fixed_world_position_v1", "world_sdf": str(ROOT/"test_world.sdf")}
        self.assertEqual(task.validate_task_config(cfg)["hover_reference_profile"], cfg["hover_reference_profile"])
        other = task.build_task_config("small")
        other["hover_reference_profile"] = cfg["hover_reference_profile"]
        with self.assertRaises(ValueError): task.validate_task_config(other)
        cfg["hover_reference_profile"]["world_sdf"] = "relative.sdf"
        with self.assertRaises(ValueError): task.validate_task_config(cfg)

    def test_native_evidence_is_bound_to_plant_and_raw_bytes(self):
        cfg = task.build_task_config("joint_probe")
        provenance = {"model_sha256": "a"*64, "source_urdf_sha256": "b"*64}
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            raw = root/"native.csv"
            raw.write_text("sim_s,x\n0,0\n1,1\n")
            for stage in task.required_gates(cfg):
                evidence = dict(kind="x650_native_stage_evidence", stage=stage, complete=True, passed=True,
                                simulation_backend="gazebo_px4", **provenance, artifacts={raw.name: task.sha(raw)})
                path = root/(stage+".json")
                path.write_text(json.dumps(evidence))
                cfg["stage_evidence"][stage] = str(path)
            self.assertEqual(len(task.validate_stage_gates(cfg, provenance)), 2)
            with self.assertRaises(ValueError):
                task.validate_stage_gates(cfg, {**provenance, "model_sha256": "c"*64})
            raw.write_text("modified")
            with self.assertRaises(ValueError): task.validate_stage_gates(cfg, provenance)

    def test_boolean_does_not_replace_evidence_file(self):
        cfg = task.build_task_config("joint_probe")
        cfg["stage_evidence"] = {"native_ground": True, "hover_folded": True}
        with self.assertRaises((TypeError, ValueError)):
            task.validate_stage_gates(cfg, {"model_sha256": "a"*64, "source_urdf_sha256": "b"*64})


if __name__ == "__main__": unittest.main()
