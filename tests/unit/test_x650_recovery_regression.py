"""Regression-runner provenance and exit accounting; no native control is run."""
from __future__ import annotations
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"scripts"))
import run_x650_recovery_regression as runner


class RecoveryRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.folder = Path(cls.temp.name)
        cls.config = {"nodes": 15, "node_dt": .04, "max_dt": .05}
        def put(name, value):
            path = cls.folder/name
            path.write_bytes(json.dumps(value, allow_nan=False).encode())
            return runner.file_identity(path)
        libraries = {}
        for name in ("core", "optimization", "predictor"):
            path = cls.folder/(name+".so")
            path.write_bytes(name.encode())
            libraries[name] = runner.file_identity(path)
        cls.report = {"kind": "x650_cpp_original_sequence_reconstruction_v1", "schema_version": 1,
            "complete": True, "all_original_outcomes_reproduced": True,
            "stopped_at_original_rejection": True, "memory_reconstructed_in_cpp": True,
            "history_from_proposals": False, "counterfactual_closed_loop_claimed": False,
            "physical_validation_claimed": False, "row_count": 379, "step_count_after": 378,
            "recording_sha256": "f"*64, "loaded_code": libraries, "rows": []}
        for seq in range(1, 380):
            record = put(f"{seq}.record.json", {"sequence": seq})
            snapshot, proposal = put(f"{seq}.snapshot.cdr", {"sequence": seq}), put(f"{seq}.proposal.cdr", {"sequence": seq})
            source = {"sequence": seq, "record_identity": record,
                      "snapshot_identity": snapshot, "proposal_identity": proposal}
            source_receipt = put(f"{seq}.row.json", source)
            fixture = {"schema_version": 1, "label": {"sequence": seq}, "config": cls.config,
                "memory": {"step_count": seq-1}, "input": {"t": seq*.02},
                "provenance": {"recording_sha256": "f"*64, "memory_from_original_core_prefix": True,
                    "recorded_physical_state": True, "row_identity": source_receipt,
                    "source_snapshot_identity": snapshot, "source_proposal_identity": proposal}}
            receipt = put(f"{seq}.fixture.json", fixture)
            row = {"sequence": seq, "original_status": 0 if seq < 379 else 2,
                   "returned": seq < 379, "fixture": receipt, "input_task_time_s": seq*.02}
            if seq < 379:
                row.update(original_proposal_match=True, original_full_result_match=True,
                           maximum_proposal_absolute_error=0., maximum_full_result_absolute_error=0.)
            else:
                row.update(original_failure_reproduced=True, error="QP infeasible",
                           original_error="QP infeasible", memory_unchanged=True)
            cls.report["rows"].append(row)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def saved_report(self, report=None):
        directory = Path(tempfile.mkdtemp(dir=self.folder))
        path = directory/"report.json"
        path.write_text(json.dumps(self.report if report is None else report), encoding="utf-8")
        return path, runner.file_identity(path)["sha256"]

    def test_full_original_receipts_and_378_379_classification(self):
        path, digest = self.saved_report()
        report, cases, config = runner.load_original(path, digest)
        self.assertEqual(len(cases), 379)
        self.assertTrue(all(c["expectation"] == "unchanged" for c in cases[:378]))
        self.assertEqual(cases[-1]["expectation"], "recovered")
        self.assertEqual(config["value"], self.config)

    def test_failed_reconstruction_cannot_be_used_as_new_oracle(self):
        report = deepcopy(self.report)
        report["all_original_outcomes_reproduced"] = False
        path, digest = self.saved_report(report)
        with self.assertRaisesRegex(ValueError, "did not qualify"):
            runner.load_original(path, digest)

    def test_missing_original_call_is_rejected(self):
        report = deepcopy(self.report)
        report["rows"].pop(20)
        path, digest = self.saved_report(report)
        with self.assertRaisesRegex(ValueError, "378 successes"):
            runner.load_original(path, digest)

    def test_tampered_fixture_size_or_hash_cannot_be_skipped(self):
        report = deepcopy(self.report)
        report["rows"][17]["fixture"]["sha256"] = "0"*64
        path, digest = self.saved_report(report)
        with self.assertRaisesRegex(ValueError, "receipt differs"):
            runner.load_original(path, digest)

    def test_mixed_controller_parameters_are_rejected(self):
        report = deepcopy(self.report)
        original = runner.checker.strict_json(report["rows"][17]["fixture"]["path"])
        original["config"]["max_dt"] = .06
        path, _ = self.saved_report()
        changed = path.parent/"mixed.fixture.json"
        changed.write_text(json.dumps(original), encoding="utf-8")
        report["rows"][17]["fixture"] = runner.file_identity(changed)
        path, digest = self.saved_report(report)
        with self.assertRaisesRegex(ValueError, "mix controller configurations"):
            runner.load_original(path, digest)

    def test_wrong_original_report_hash_rejected_before_execution(self):
        path, _ = self.saved_report()
        with self.assertRaisesRegex(ValueError, "report SHA differs"):
            runner.load_original(path, "0"*64)

    def test_expected_original_failure_exit_one_is_not_ignored(self):
        valid = {"timed_out": False, "spawn_error": None, "returncode": 1,
                 "raw": {"path": "exists"}}
        runner.check_exit(valid, expected=1, label="Old379")
        with self.assertRaisesRegex(ValueError, "expected 0, actual 1"):
            runner.check_exit(valid, expected=0, label="New379")
        for change in ({"returncode": -9}, {"timed_out": True}, {"spawn_error": "not found"}, {"raw": None}):
            with self.assertRaisesRegex(ValueError, "exit/output differs"):
                runner.check_exit(dict(valid, **change), expected=1, label="Old379")

    def test_failed_native_exit_preserves_both_processes_and_fails_case(self):
        directory = Path(tempfile.mkdtemp(dir=self.folder))/"case"
        case = {"sequence": 379, "fixture": self.report["rows"][-1]["fixture"],
                "value": runner.checker.strict_json(self.report["rows"][-1]["fixture"]["path"]),
                "expectation": "recovered"}
        args = SimpleNamespace(old_executable=Path("old"), new_executable=Path("new"),
                               predictor=Path("p"), process_timeout_s=10.)
        def execute(executable, *unused):
            return {"returncode": 2 if str(executable) == "new" else 1,
                    "timed_out": False, "spawn_error": None, "raw": {"path": "unread"}}
        with patch.object(runner, "execute", side_effect=execute) as process:
            result = runner.run_case(case, args, {"predictor": "p"}, {}, directory)
        self.assertEqual(process.call_count, 2)
        self.assertFalse(result["passed"])
        self.assertIn("New native process exit/output differs", result["error"])
        self.assertFalse(runner.checker.strict_json(directory/"audit.json")["passed"])
        self.assertTrue((directory/"case_result.json").is_file())

    def test_evidence_files_cannot_be_overwritten(self):
        path, _ = self.saved_report()
        original = path.read_bytes()
        with self.assertRaises(FileExistsError):
            runner.write_new(path, {"changed": True})
        self.assertEqual(path.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
