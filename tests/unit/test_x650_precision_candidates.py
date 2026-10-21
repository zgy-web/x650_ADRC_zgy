"""Candidate-driver sequencing and evidence checks; subprocesses are all mocks."""
from copy import deepcopy
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("x650_candidate_driver_test", ROOT/"scripts/run_x650_precision_candidates.py")
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)
from task import build_task_config
from diagnostic_parameters import expected_record


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


class CandidateFixture:
    def __init__(self, root, failure=None):
        self.root, self.failure, self.calls, self.flight_count = root, failure, [], 0
        (root/"runs").mkdir(parents=True)
        self.task = build_task_config()
        self.task.update(diagnostic_profile=deepcopy(driver.PRECISION_PROFILE),
            precision_test_profile="hover20_steps36_v1",
            hover_reference_profile={"name": "fixed_world_position_v1", "world_sdf": str(root/"world.sdf")})
        self.task_path = root/"base.json"
        write(self.task_path, self.task)
        self.provenance = dict(model_sha256="a"*64, source_urdf_sha256="b"*64, flight_profile_sha256="c"*64,
            airframe_sha256="d"*64, native_recorder_sha256="e"*64, preparation_report_sha256="f"*64, total_mass_kg=3.495)
        self.controls = {name: expected_record(1., 9) for name in driver.load_parameter_evidence().CORE_CONTROL_PARAMETERS}
        self.controls.update({name: expected_record(value, 9) for name, value in driver.PID_CANDIDATES["baseline"].items()})
        self.baseline = self.root/"runs/x650_wholebody_mpc_v1_baseline"
        self.make_run(self.baseline, self.task)
        self.analysis_python = root/"mock-analysis-python"
        self.analysis_python.write_text("Never executed; mocked subprocess", encoding="utf-8")

    def make_run(self, folder, task):
        metadata = dict(task=deepcopy(task), complete=True, landed_and_disarmed=True,
            diagnostic_parameters_restored=True, diagnostic_restore_pending=False,
            model_sha256="a"*64, source_urdf_sha256="b"*64,
            physical_profile={"total_mass": 3.495, "synthetic_test_fixture": True}, provenance=self.provenance.copy(),
            actual_pid_parameters={name: record["value"] for name, record in self.controls.items()},
            pid_tuning_candidate=deepcopy(task["pid_tuning_candidate"]))
        metadata["actual_pid_parameters"].update(task["pid_tuning_candidate"] or {})
        write(folder/"controller_metadata.json", metadata)
        write(folder/"controller_task.json", task)
        for label in ("before", "after"):
            write(folder/("px4_parameters_"+label+".json"), {"parameters": self.controls})
        return metadata

    def preflight(self, task):
        result = self.provenance.copy()
        if self.failure == "preflight_physics":
            result["model_sha256"] = "9"*64
        return NS(), result

    def process(self, command, *, cwd, env, stdout, stderr):
        self.calls.append({"command": command, "env": env.copy()})
        if command[1].endswith("launch.py"):
            self.flight_count += 1
            task = driver.read_json(Path(command[-1]))
            folder = self.root/f"runs/x650_wholebody_mpc_v1_candidate_{self.flight_count}"
            metadata = self.make_run(folder, task)
            launch = self.root/f"runs/x650_wb_launch_candidate_{self.flight_count}"
            write(launch/"launch.json", dict(complete=True, runtime_closed=True, task=task, run_directories=[str(folder)]))
            if self.failure == "launch":
                return NS(returncode=1)
            if self.failure == "pending":
                write(self.root/".amsim/x650_diagnostic_restore_pending.json", {"pending": True})
            if self.failure == "ambiguous_run":
                self.make_run(self.root/f"runs/x650_wholebody_mpc_v1_extra_{self.flight_count}", task)
            if self.failure == "restoration":
                metadata["diagnostic_parameters_restored"] = False
            if self.failure == "wrong_task":
                metadata["task"]["duration_s"] += 1
            if self.failure == "physics":
                metadata["physical_profile"]["total_mass"] = 4.
            if self.failure == "actual_gain":
                metadata["actual_pid_parameters"]["MPC_XY_VEL_P_ACC"] += .5
            if self.failure == "before_controls":
                altered = deepcopy(self.controls)
                altered["MC_ROLL_P"] = expected_record(2., 9)
                for label in ("before", "after"):
                    write(folder/("px4_parameters_"+label+".json"), {"parameters": altered})
            write(folder/"controller_metadata.json", metadata)
            return NS(returncode=0)
        folder = Path(command[command.index("--run-dir")+1])
        output = Path(command[command.index("--output")+1])
        if self.failure == "analysis":
            return NS(returncode=2)
        if "hover_precision" in command[1]:
            report = dict(source_run=folder.name, native_only=False,
                measurement_period_pid={"matches_controller_metadata": self.failure != "ulog_pid"},
                ulog_control_chain={"fixed_world_target_verified_within_2mm": True})
            write(output/"precision_audit.json", report)
        else:
            write(output/"position_steps_audit.json", {"source_run": folder.name})
        return NS(returncode=0)

    def run(self, **changes):
        options = dict(root=self.root, output=self.root/"new_batch", base_task=self.task_path,
            candidates=["velocity_low", "velocity_high"], run_process=self.process,
            preflight_check=self.preflight, launch_python="mock-launch-python",
            analysis_python=self.analysis_python)
        options.update(changes)
        return driver.run_batch(**options)


class PrecisionCandidateTests(unittest.TestCase):
    def test_two_candidates_are_sequential_exactly_matched_and_recovered(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture = CandidateFixture(Path(tmp))
            result = fixture.run()
            self.assertTrue(result["complete"])
            self.assertEqual(fixture.flight_count, 2)
            self.assertEqual([row["candidate"] for row in result["trials"]], ["velocity_low", "velocity_high"])
            for call, trial in zip(fixture.calls, result["trials"]):
                self.assertEqual(call["env"]["PYTHONPATH"], "/usr/lib/python3/dist-packages")
                self.assertTrue(all(call["env"][key] == "1" for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")))
                self.assertTrue(trial["evidence"]["parameters_restored"])
                self.assertEqual(trial["task_sha256"], trial["evidence"]["task_sha256"])
                self.assertTrue(trial["preflight_physics_verified"])
            self.assertTrue(driver.read_json(fixture.root/"new_batch/batch_status.json")["complete"])
            self.assertEqual(driver.read_json(fixture.task_path), fixture.task)

    def test_failed_or_ambiguous_evidence_stops_before_second_flight(self):
        for failure in ("launch", "pending", "restoration", "wrong_task", "physics", "actual_gain", "before_controls", "ambiguous_run"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp:
                fixture = CandidateFixture(Path(tmp), failure)
                with self.assertRaises(RuntimeError):
                    fixture.run()
                self.assertEqual(fixture.flight_count, 1)
                result = driver.read_json(fixture.root/"new_batch/batch_status.json")
                self.assertFalse(result["complete"])
                self.assertEqual(result["trials"][0]["phase"], "failed")

    def test_preflight_physics_drift_prevents_any_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture = CandidateFixture(Path(tmp), "preflight_physics")
            with self.assertRaisesRegex(RuntimeError, "before launch"):
                fixture.run()
            self.assertEqual(fixture.flight_count, 0)

    def test_analysis_uses_clean_pythonpath_and_finishes_before_next_flight(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture = CandidateFixture(Path(tmp))
            result = fixture.run(analyze=True)
            self.assertTrue(result["complete"])
            self.assertEqual(len(fixture.calls), 6)
            for index, call in enumerate(fixture.calls):
                if index % 3:
                    self.assertNotIn("PYTHONPATH", call["env"])
                    self.assertNotIn("--native-only", call["command"])
                else:
                    self.assertTrue(call["command"][1].endswith("launch.py"))
            self.assertTrue(all(item["complete"] for trial in result["trials"] for item in trial["analysis"]))

    def test_analysis_failure_or_failed_ulog_certification_stops_batch(self):
        for failure in ("analysis", "ulog_pid"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp:
                fixture = CandidateFixture(Path(tmp), failure)
                with self.assertRaises(RuntimeError):
                    fixture.run(analyze=True)
                self.assertEqual(fixture.flight_count, 1)

    def test_existing_runtime_or_output_is_never_repaired_or_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture = CandidateFixture(Path(tmp))
            write(fixture.root/".amsim/runtime.json", {"owned_elsewhere": True})
            with self.assertRaisesRegex(RuntimeError, "inspection"):
                fixture.run()
            self.assertEqual(driver.read_json(fixture.root/".amsim/runtime.json"), {"owned_elsewhere": True})
            existing = fixture.root/"existing"
            existing.mkdir()
            with self.assertRaisesRegex(ValueError, "new directory"):
                fixture.run(output=existing)
            self.assertEqual(fixture.flight_count, 0)

    def test_multiple_baseline_task_matches_require_explicit_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture = CandidateFixture(Path(tmp))
            fixture.make_run(fixture.root/"runs/x650_wholebody_mpc_v1_repeated_base", fixture.task)
            with self.assertRaisesRegex(RuntimeError, "disambiguate"):
                fixture.run()
            self.assertEqual(fixture.flight_count, 0)
            self.assertTrue(fixture.run(baseline_run=fixture.baseline)["complete"])


if __name__ == "__main__":
    unittest.main()
