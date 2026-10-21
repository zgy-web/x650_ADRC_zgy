"""Research loading and applied/restored PID identity on synthetic audited files."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))
import test_x650_upper_stage_audit as stage_tests
spec = importlib.util.spec_from_file_location("upper_response_research_test", ROOT/"scripts/identify_x650_upper_response.py")
identifier = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = identifier
spec.loader.exec_module(identifier)


class UpperResponseResearchTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.run, self.launch, self.ulog = stage_tests.complete_fixture(self.root/"training", seed=1)
        self.acceptance = self.root/"training/audit"
        stage_tests.audit.audit(self.run, self.launch, self.acceptance, ulog_loader=lambda _: self.ulog)

    def test_applied_42_differ_from_restored_and_native_250hz_is_observation(self):
        run = identifier.load_run(self.run, audit_path=self.acceptance)
        before = identifier.json_read(self.run/"px4_parameters_before.json")["parameters"]
        self.assertNotEqual(before["MPC_XY_VEL_P_ACC"]["value"], run["pid_parameters"]["MPC_XY_VEL_P_ACC"])
        self.assertEqual(len(run["pid_parameters"]), 42)
        self.assertEqual(len(run["truth_times"]), 7501)
        self.assertEqual(len(run["command_times"]), 1501)
        self.assertEqual(run["identity"]["control_interface"], "raw_ev_world_pva_v2")
        self.assertTrue(run["feedback_velocity_proxy"]["passed"])

    def test_original_provisional_research_cannot_enter_legacy_path(self):
        with self.assertRaisesRegex(ValueError, "research|Research|diagnostic|Diagnostic"):
            identifier.load_run(self.run)
        with self.assertRaisesRegex(ValueError, "together"):
            identifier.analyze_runs(self.run, self.run, train_audit=self.acceptance)

    def test_independent_native_train_holdout_keep_zero_delay_and_complete_condition(self):
        holdout, launch, ulog = stage_tests.complete_fixture(self.root/"holdout", seed=3)
        acceptance = self.root/"holdout/audit"
        stage_tests.audit.audit(holdout, launch, acceptance, ulog_loader=lambda _: ulog)
        output = self.root/"identification"
        report = identifier.write_identification(self.run, holdout, output,
            train_audit=self.acceptance, holdout_audit=acceptance)
        self.assertTrue(report["passed"], report.get("reasons"))
        model = identifier.json_read(output/"response_model.json")
        np.testing.assert_allclose(model["base_time_constant_s"], [.16, .2, .25], atol=.0005, rtol=0.)
        self.assertEqual(model["schema_version"], 2)
        self.assertEqual(model["feedback_contract"], "px4_estimator_receipt_enu_v1")
        self.assertEqual(model["operating_condition_sha256"], identifier.json_hash(model["operating_condition"]))
        self.assertEqual(model["command_delay_s"], 0.)
        self.assertTrue(report["eligible_as_stage_gate"])
        self.assertTrue(report["upper_command_verified"])
        self.assertTrue(any(Path(path).name == "upper_stage_audit.json" for path in report["artifacts"]))
        self.assertEqual(report["artifacts"]["response_model.json"], identifier.sha(output/"response_model.json"))

    def test_copying_accepted_train_run_does_not_create_holdout(self):
        with self.assertRaisesRegex(ValueError, "independent"):
            identifier.analyze_runs(self.run, self.run, train_audit=self.acceptance, holdout_audit=self.acceptance)

    def test_missing_upper_flag_or_wrong_run_or_changed_artifact_rejected(self):
        path = self.acceptance/"research_stage.json"
        original = path.read_bytes()
        for fault in ("flag", "owner", "artifact"):
            with self.subTest(fault=fault):
                path.write_bytes(original)
                document = identifier.json_read(path)
                if fault == "flag": document["upper_command_verified"] = False
                elif fault == "owner": document["source_run_directory"] = str(self.root/"another")
                else: document["artifacts"][str(self.run/"px4.ulg")] = "0"*64
                stage_tests.save(path, document)
                with self.assertRaises(ValueError):
                    identifier.load_run(self.run, audit_path=self.acceptance)

    def test_proxy_without_pass_cannot_be_retagged_as_accepted(self):
        report_path = self.acceptance/"upper_stage_audit.json"
        report = identifier.json_read(report_path)
        report["feedback_velocity_proxy"]["metrics"]["axis_rmse"][2] = .011
        stage_tests.save(report_path, report)
        certificate = identifier.json_read(self.acceptance/"research_stage.json")
        certificate["artifacts"][str(report_path)] = identifier.sha(report_path)
        stage_tests.save(self.acceptance/"research_stage.json", certificate)
        with self.assertRaisesRegex(ValueError, "proxy"):
            identifier.load_run(self.run, audit_path=self.acceptance)


if __name__ == "__main__":
    unittest.main()
