"""Admission guards for the fresh chain's current software/deployment proof."""
from pathlib import Path
import json
import shutil
import sys
import unittest
from unittest.mock import patch
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_ts20_hold_v1 import launch_chain_v4 as launch


class CurrentSoftwareProofTests(unittest.TestCase):
    def setUp(self):
        parent = ROOT.parent / "tmp"
        parent.mkdir(exist_ok=True)
        # Use the workspace's inherited permissions. Python's mode-0700
        # TemporaryDirectory applies an incompatible DACL in the Windows
        # sandbox, before any of these actual guard assertions can run.
        self.root = (parent / ("current_ts20_proof_" + uuid4().hex)).resolve()
        self.root.relative_to(parent.resolve())
        self.root.mkdir()
        def cleanup():
            self.root.resolve().relative_to(parent.resolve())
            shutil.rmtree(self.root)
        self.addCleanup(cleanup)
        cpp = "control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction_native.cpp"
        self.sources = [*launch.REQUIRED_SOURCES, cpp]
        for name in self.sources + ["scripts/amsim", "scenarios/fixture.yaml", "src/amsim/a.py",
                                    "simulations/x650_ts20_hold_v1/launch_chain_v4.py"]:
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(("fixture " + name + "\n").encode())
        self.predictor_sha = launch.task.sha(self.root / self.sources[0])
        self.patch = patch.object(launch, "NEW_PREDICTOR_SHA256", self.predictor_sha)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.library = self.root / "native.so"
        self.library.write_bytes(b"fixture library\n")
        self.oracle = self.root / "new_oracle.json"
        self.oracle.write_text(json.dumps({"passed": True, "candidate_sha256": self.predictor_sha}))
        self.report = self.root / "regression.log"
        self.report.write_text("120 tests, no failures\n")
        self.matrix = self.root / "matrix.json"
        self.matrix.write_text(json.dumps({"total": 30, "passed": 30}))
        self.proof = self.root / "current_proof.json"
        self.document = {
            "no_flight": True,
            "deployment_sources": {name: launch.task.sha(self.root / name) for name in self.sources},
            "native_library": {"path": str(self.library), "sha256": launch.task.sha(self.library),
                               "cpp_path": cpp, "cpp_sha256": launch.task.sha(self.root / cpp)},
            "oracle": {"path": str(self.oracle), "sha256": launch.task.sha(self.oracle),
                       "candidate_sha256": self.predictor_sha, "pass": True,
                       "protocol_sha256": "a" * 64},
            "package_regression": {"tests": 120, "failures": 0, "errors": 0, "skipped": 1,
                                   "report_path": str(self.report), "report_sha256": launch.task.sha(self.report)},
            "software_matrix": {"total": 30, "passed": 30, "failures": [], "max_wall_ms": 30.,
                                "summary_path": str(self.matrix), "summary_sha256": launch.task.sha(self.matrix)},
        }
        self.cfg = {"scenario": "fixture", "ts20_inputs": {
            "native_library": str(self.library), "native_library_sha256": launch.task.sha(self.library),
            "oracle_validation": str(self.oracle)}}
        self.save_proof()

    def save_proof(self):
        self.proof.write_text(json.dumps(self.document))
        self.proof_sha = launch.task.sha(self.proof)

    def verify(self):
        return launch.verify_software_proof(self.root, self.cfg, self.proof, self.proof_sha)

    def test_current_proof_and_all_actual_deployment_sources_match(self):
        checked = self.verify()
        self.assertEqual(checked["deployment_sources"], self.document["deployment_sources"])
        self.assertEqual(checked["sha256"], self.proof_sha)

    def test_same_proof_hash_does_not_hide_changed_mpc_source(self):
        (self.root / launch.REQUIRED_SOURCES[2]).write_bytes(b"changed actual MPC\n")
        with self.assertRaisesRegex(RuntimeError, "deployment source differs"):
            self.verify()

    def test_same_proof_hash_does_not_hide_changed_native_library(self):
        self.library.write_bytes(b"different library\n")
        with self.assertRaisesRegex(RuntimeError, "native task library differs"):
            self.verify()

    def test_task_cannot_reuse_an_old_oracle(self):
        old = self.root / "old_oracle.json"
        old.write_text("{}")
        self.cfg["ts20_inputs"]["oracle_validation"] = str(old)
        with self.assertRaisesRegex(RuntimeError, "new matching oracle"):
            self.verify()

    def test_failed_matrix_cannot_be_promoted_by_a_matching_proof_hash(self):
        self.document["software_matrix"].update(passed=29, failures=["deadline"])
        self.save_proof()
        with self.assertRaisesRegex(RuntimeError, "30-case deadline acceptance"):
            self.verify()

    def test_unreviewed_proof_bytes_reject(self):
        self.proof.write_text(self.proof.read_text() + " ")
        with self.assertRaisesRegex(RuntimeError, "proof hash differs"):
            self.verify()

    def test_archive_binds_new_proof_and_source_bytes_and_refuses_tampering(self):
        checked = self.verify()
        output, run = self.root / "launch", self.root / "actual_run"
        output.mkdir()
        expected = {"src/amsim/a.py": launch.task.sha(self.root / "src/amsim/a.py")}
        with patch.object(launch, "git_snapshot", return_value={"head_value": "fixture"}), \
                patch.object(launch.task, "source_files_for_task", return_value=[self.root / "src/amsim/a.py"]):
            before = launch.capture_before(self.root, output, self.cfg, {"source_files_sha256": expected}, checked)
            after = launch.capture_after(self.root, output, self.cfg, before, [str(run)])
        (output / "launch.json").write_text(json.dumps({"complete": True, "runtime_closed": True,
            "run_directories": [str(run)], "provenance": {"source_files_sha256": expected},
            "platform_provenance": {"file": "platform_provenance.json",
                                    "sha256": launch.task.sha(output / "platform_provenance.json")}}))
        self.assertTrue(after["software_binding_unchanged"])
        self.assertTrue(launch.verify_platform(output, run, expected_software_sha256=self.proof_sha)["verified"])
        (output / "software_sources.zip").write_bytes(b"replacement source archive")
        with self.assertRaisesRegex(RuntimeError, "software evidence differs"):
            launch.verify_platform(output, run, expected_software_sha256=self.proof_sha)


if __name__ == "__main__":
    unittest.main()
