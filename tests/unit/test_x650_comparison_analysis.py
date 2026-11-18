import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("comparison_analysis", ROOT / "simulations/x650_control_comparison_v1/analysis.py")
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


def row(t, error=(0., 0., 0.), stamp=None, target=(0., 0., 1.5), **extra):
    result = {"t": t, "sim_time_s": 100. + t, "truth_stamp_s": 100. + (t if stamp is None else stamp), "dt": .02,
              "compute_ms": 1., "limited": False, "tilt_deg": 1., "closure_m": 1e-7, "axis_rad": 1e-5,
              "q0": 0., "q1": 0., "q0_target": 0., "q1_target": 0.}
    for i, a in enumerate("xyz"):
        result["truth_" + a] = target[i] + error[i]
        result["target_" + a] = target[i]
        result["v" + a + "_cmd"] = 0.
    result.update(extra)
    return result


class MetricTests(unittest.TestCase):
    def test_stage_a_windows_distinguish_initial_band_from_sustained_recovery(self):
        rows = [row(i / 10, (0.03, 0.04, 0.) if i == 121 else
                    (.1, 0., 0.) if 100 <= i < 120 else (0., 0., 0.), clock_receipt_age_s=.01)
                for i in range(301)]
        result = analysis.summarize_trace(rows, "force")
        window = result["windows"]["active"]
        self.assertTrue(window["coverage_complete"])
        self.assertAlmostEqual(window["observed_span_s"], 1.9)
        self.assertAlmostEqual(window["iae_m_s"], .19)
        self.assertAlmostEqual(window["rmse_m"], .1)
        self.assertTrue(result["recovery"]["initially_in_band"])
        self.assertAlmostEqual(result["recovery_time_s"], .2)
        self.assertEqual(result["timing"]["clock_receipt_age_s"]["p99"], .01)
        self.assertEqual(result["timing"]["wall_send_interval_s"]["n"], 0)
        partial = analysis.summarize_trace(rows[:110], "force")
        self.assertFalse(partial["windows"]["active"]["coverage_complete"])
        self.assertIsNone(partial["windows"]["after"]["iae_m_s"])
        self.assertIsNone(partial["recovery"]["initially_in_band"])

    def test_irregular_truth_is_deduplicated_and_time_weighted(self):
        rows = [row(0., (0., 0., 0.)), row(.5, (0., 0., 0.), stamp=0.),
                row(1., (2., 0., 0.)), row(1.5, (2., 0., 0.), stamp=1.), row(4., (0., 0., 0.))]
        result = analysis.summarize_trace(rows, "hover")
        self.assertEqual(result["truth_samples"], 3)
        self.assertAlmostEqual(result["position_rmse_m"], np.sqrt(2.))
        self.assertAlmostEqual(result["last_5s_bias_m"]["x"], 1.)
        self.assertFalse(result["measurement_complete"])
        self.assertFalse(result["accuracy_accepted"])

    def test_complete_good_hover_and_partial_run_cannot_both_pass(self):
        rows = [row(float(t), (.01, .02, -.02)) for t in np.arange(0., 30.001, .05)]
        result = analysis.summarize_trace(rows, "hover")
        self.assertTrue(result["accuracy_accepted"])
        self.assertAlmostEqual(result["position_rmse_m"], .03)
        self.assertFalse(analysis.summarize_trace(rows[:100], "hover")["accuracy_accepted"])

    def test_recovery_requires_continuous_observation_and_two_second_hold(self):
        times = np.arange(0., 30.001, .05)
        rows = [row(float(t), (.1 if 10. <= t < 14. else 0., 0., 0.)) for t in times]
        result = analysis.summarize_trace(rows, "force")
        self.assertAlmostEqual(result["recovery_time_s"], 2.)
        self.assertAlmostEqual(result["disturbance_peak_error_m"], .1)
        # Two isolated good points cannot manufacture a two-second recovery.
        self.assertIsNone(analysis._recovery(np.array([12., 14.]), np.zeros(2), 12., 30.))
        not_recovered = [row(float(t), (.1 if t >= 10. else 0., 0., 0.)) for t in times]
        self.assertIsNone(analysis.summarize_trace(not_recovered, "force")["recovery_time_s"])

    def test_step_overshoot_uses_return_direction(self):
        rows = []
        for t in np.arange(0., 35.001, .05):
            target = [0., 0., 1.5]
            if 5. <= t < 10.: target[0] += .2
            if 15. <= t < 20.: target[1] += .2
            if 25. <= t < 30.: target[2] += .2
            error = (.04 if 6. <= t < 6.5 else -.03 if 11. <= t < 11.5 else 0., 0., 0.)
            rows.append(row(float(t), error, target=target))
        result = analysis.summarize_trace(rows, "step")
        self.assertAlmostEqual(result["steps"][0]["overshoot_percent"], 20.)
        self.assertAlmostEqual(result["steps"][1]["overshoot_percent"], 15.)
        self.assertEqual(result["steps"][1]["direction"], -1.)

    def test_invalid_data_cannot_be_scored_as_pass(self):
        for rows in ([], [row(0.)], [row(0.), row(.1, compute_ms=float("nan"))],
                     [row(0.), row(.1, truth_x=float("nan"))], [row(1.), row(0.)],
                     [row(0.), row(.1, stamp=.2)]):
            with self.assertRaises(ValueError):
                analysis.summarize_trace(rows, "hover")

    def test_trace_read_from_csv_preserves_boolean_and_actual_command_metrics(self):
        rows = [row(0., vx_cmd=0., limited="False", compute_ms=1.),
                row(1., vx_cmd=.2, limited="True", compute_ms=21.),
                row(2., vx_cmd=.1, limited="False", compute_ms=2.)]
        result = analysis.summarize_trace(rows, "identify")
        self.assertAlmostEqual(result["limited_fraction"], .5)
        self.assertAlmostEqual(result["command_total_variation_m_s"], .3)
        self.assertAlmostEqual(result["command_slew_max_m_s2"], .2)
        self.assertEqual(result["deadline_20ms_exceeded_count"], 1)

    def test_constraint_boundary_metrics_do_not_depend_on_adapter_clipping(self):
        rows = [row(0., vx_cmd=0., limited=False), row(.4, vx_cmd=.4, limited=False),
                row(1., vx_cmd=.4, limited=False)]
        result = analysis.summarize_trace(rows, "identify")
        self.assertEqual(result["limited_fraction"], 0.)
        self.assertAlmostEqual(result["speed_boundary_fraction"], .8)
        self.assertAlmostEqual(result["slew_boundary_fraction"], .4)


class ReportTests(unittest.TestCase):
    def test_new_batch_process_evidence_cannot_be_missing_reused_or_dirty(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            _, _, manifest = self.evidence_fixture(root)
            manifest.update(schema_version=2, process_audit_required=True, partition="test")
            item = manifest["runs"][0]
            audit_path = root / "attempt.audit.json"
            item.update(process_audit_file=audit_path.name, process_audit_accepted=True,
                        process_audit_trial_id="this-trial")
            clean = {"clean": True, "processes": [], "errors": []}
            audit = dict(schema_version=1, partition="test", algorithm="pid", case="hover",
                         trial_id="this-trial", before=clean, after=clean, launch_attempted=True, accepted=True)
            for variant, accepted in (({}, True), ({"trial_id": "another-trial"}, False),
                    ({"after": dict(clean, processes=[{"pid": 10}])}, False)):
                audit_path.write_text(json.dumps(dict(audit, **variant)))
                item["process_audit_sha256"] = analysis._sha(audit_path)
                (root / "manifest.json").write_text(json.dumps(manifest))
                self.assertEqual(analysis.build_report(root, plots=False)["evidence_accepted"], accepted)
            audit_path.unlink()
            self.assertFalse(analysis.build_report(root, plots=False)["evidence_accepted"])

    def evidence_fixture(self, root):
        folder = root / "run"; folder.mkdir()
        config = json.dumps({"model_sha256": "physical-plant"})
        for name, content in (("controller_config.json", config), ("response_model.json", "{}")):
            (root / name).write_text(content); (folder / name).write_text(content)
        (folder / "controllers.py").write_text("# frozen controller")
        config_hash = analysis._sha(folder / "controller_config.json")
        response_hash = analysis._sha(folder / "response_model.json")
        parameters = {}
        for phase in ("before", "after"):
            filename = f"px4_parameters_{phase}.json"
            (folder / filename).write_text(json.dumps({"contract_matches": True, "parameters_unchanged": True}))
            parameters[phase] = dict(file=filename, sha256=analysis._sha(folder / filename),
                contract_matches=True, parameters_unchanged=True)
        result = dict(algorithm="pid", case="hover", model_sha256="physical-plant", execution_success=False,
                      error="synthetic failure with preserved parameter evidence", parameters=parameters,
                      config_sha256=config_hash, response_model_sha256=response_hash)
        (folder / "result.json").write_text(json.dumps(result))
        item = dict(algorithm="pid", case="hover", repetition=1, result_dir="run", returncode=1, finished=True)
        manifest = dict(runs=[item], schedule=[dict(algorithm="pid", case="hover", repetition=1)],
                        config_sha256=config_hash, model_sha256=response_hash,
                        source_sha256={"controllers.py": analysis._sha(folder / "controllers.py")})
        (root / "manifest.json").write_text(json.dumps(manifest))
        return folder, result, manifest

    def test_failed_partial_and_platform_runs_stay_in_denominator(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            runs = []
            for i, success in enumerate((True, False)):
                folder = root / f"run{i}"; folder.mkdir()
                rows = [row(float(t), (.01 if success else .3, 0., 0.))
                        for t in np.arange(0., 30.001 if success else 5.001, .05)]
                with (folder / "control_trace.csv").open("w", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
                (folder / "result.json").write_text(json.dumps({"execution_success": success, "model_sha256": "same", "error": None if success else "solver failed"}))
                runs.append(dict(algorithm="mpc", case="hover", repetition=i + 1, result_dir=folder.name, returncode=0 if success else 1))
            runs.append(dict(algorithm="mpc", case="hover", repetition=3, result_dir=None, returncode=1))
            (root / "manifest.json").write_text(json.dumps({"runs": runs}))
            result = analysis.build_report(root, plots=False)
            group = result["groups"][0]
            self.assertEqual(group["attempted"], 3)
            self.assertEqual(group["metrics_available"], 2)
            self.assertEqual(group["execution_successes"], 1)
            self.assertEqual(group["measurement_complete"], 1)
            self.assertEqual(group["accuracy_accepted"], 1)
            self.assertAlmostEqual(group["metrics"]["position_rmse_m"]["mean"], .155)
            self.assertIn("analysis_error", result["runs"][2])
            self.assertIn("solver failed", (root / "README_CN.md").read_text())
            self.assertTrue((root / "summary.csv").exists())
            self.assertFalse(result["evidence_accepted"])
            self.assertFalse(result["runs"][0]["evidence_accepted"])

    def test_valid_failure_evidence_does_not_require_execution_success(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name); self.evidence_fixture(root)
            report = analysis.build_report(root, plots=False)
            self.assertTrue(report["runs"][0]["evidence_accepted"])
            self.assertTrue(report["evidence_accepted"])
            self.assertFalse(report["runs"][0]["execution_success"])
            self.assertFalse(report["runs"][0]["accuracy_accepted"])

    def test_algorithm_case_and_physical_model_must_match_frozen_batch(self):
        for field, replacement, check in (("algorithm", "mpc", "algorithm_matches_manifest"),
                                         ("case", "force", "case_matches_manifest"),
                                         ("model_sha256", "different-plant", "physical_model_matches_frozen_config")):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as name:
                root = Path(name); folder, result, _ = self.evidence_fixture(root)
                result[field] = replacement
                (folder / "result.json").write_text(json.dumps(result))
                report = analysis.build_report(root, plots=False)
                self.assertFalse(report["runs"][0]["evidence_checks"][check])
                self.assertFalse(report["evidence_accepted"])

    def test_changed_or_missing_parameter_evidence_is_not_accepted(self):
        for mode in ("missing_after", "changed_parameters", "snapshot_tampered"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as name:
                root = Path(name); folder, result, _ = self.evidence_fixture(root)
                if mode == "missing_after":
                    result["parameters"].pop("after")
                elif mode == "changed_parameters":
                    result["parameters"]["after"]["parameters_unchanged"] = False
                else:
                    (folder / "px4_parameters_before.json").write_text("modified")
                (folder / "result.json").write_text(json.dumps(result))
                report = analysis.build_report(root, plots=False)
                self.assertFalse(report["runs"][0]["parameter_evidence"]["verified"])
                self.assertFalse(report["evidence_accepted"])

    def test_batch_requires_complete_schedule_and_no_interrupted_attempts(self):
        for mode in ("unfinished", "not_all_started", "interrupted", "batch_config_changed"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as name:
                root = Path(name); _, _, manifest = self.evidence_fixture(root)
                if mode == "unfinished":
                    manifest["runs"][0]["finished"] = False
                elif mode == "not_all_started":
                    manifest["schedule"].append(dict(algorithm="mpc", case="hover", repetition=1))
                elif mode == "interrupted":
                    manifest["interrupted_attempts"] = [manifest["runs"][0].copy()]
                else:
                    (root / "controller_config.json").write_text(json.dumps({"model_sha256": "changed"}))
                (root / "manifest.json").write_text(json.dumps(manifest))
                self.assertFalse(analysis.build_report(root, plots=False)["evidence_accepted"])

    def test_unsent_solver_and_deadline_failures_are_counted_without_trace(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name); runs = []
            results = [dict(execution_success=False, error="deadline", deadline_failure_ms=25.),
                       dict(execution_success=False, error="solver", controller_failure={"t": .04, "compute_ms": 31.,
                            "diagnostics": {"solver_status": "run time limit reached"}})]
            for i, result in enumerate(results):
                folder = root / f"run{i}"; folder.mkdir()
                (folder / "result.json").write_text(json.dumps(result))
                runs.append(dict(algorithm="mpc", case="hover", repetition=i + 1, result_dir=folder.name, returncode=1))
            (root / "manifest.json").write_text(json.dumps({"runs": runs}))
            result = analysis.build_report(root, plots=False)
            group = result["groups"][0]
            self.assertEqual(group["metrics_available"], 0)
            self.assertEqual(group["missing_traces"], 2)
            self.assertEqual(group["controller_failures"], 1)
            self.assertEqual(group["unsent_failure_calls"], 2)
            self.assertEqual(group["deadline_exceeded_count_total"], 2)
            self.assertEqual(group["metrics"]["compute_ms.max"]["mean"], 28.)
            self.assertIn("run time limit reached", (root / "README_CN.md").read_text())

    def test_terminal_timing_is_added_to_partial_trace_and_deduplicated(self):
        terminal = analysis._terminal_controller_events({"deadline_failure_ms": 25.,
            "controller_failure": {"t": 2., "compute_ms": 25., "diagnostics": {"solver_status": "solved"}}})
        self.assertEqual(len(terminal), 1)
        timing = analysis._controller_timing([row(0., compute_ms=1.), row(1., compute_ms=2.)], terminal)
        self.assertEqual(timing["calls_with_timing"], 3)
        self.assertEqual(timing["compute_ms"]["max"], 25.)
        self.assertEqual(timing["deadline_20ms_exceeded_count"], 1)

    def test_interrupted_attempt_is_retained_in_summary_and_denominator(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            item = dict(algorithm="pid", case="hover", repetition=1, result_dir=None, returncode=1)
            (root / "manifest.json").write_text(json.dumps({"runs": [item], "interrupted_attempts": [dict(item, returncode=None, finished=False)]}))
            result = analysis.build_report(root, plots=False)
            self.assertEqual(result["attempted"], 2)
            self.assertEqual(result["interrupted_attempt_count"], 1)
            self.assertEqual(result["groups"][0]["attempted"], 2)
            self.assertEqual(len(result["interrupted_attempts"]), 1)
            self.assertEqual(result["interrupted_attempts"][0]["attempt_origin"], "interrupted")

    def test_frozen_snapshots_are_compared_with_manifest(self):
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            (folder / "controller_config.json").write_text("config")
            (folder / "response_model.json").write_text("model")
            (folder / "controllers.py").write_text("code")
            config_hash = analysis._sha(folder / "controller_config.json")
            response_hash = analysis._sha(folder / "response_model.json")
            manifest = {"config_sha256": config_hash, "model_sha256": response_hash,
                        "source_sha256": {"controllers.py": analysis._sha(folder / "controllers.py")}}
            result = {"config_sha256": config_hash, "response_model_sha256": response_hash}
            self.assertEqual(analysis._input_evidence(folder, result, manifest)["status"], "verified")
            (folder / "controllers.py").write_text("changed code")
            evidence = analysis._input_evidence(folder, result, manifest)
            self.assertEqual(evidence["status"], "mismatch")
            self.assertIn("controllers.py", evidence["mismatches"])
            (folder / "controllers.py").unlink()
            self.assertEqual(analysis._input_evidence(folder, result, manifest)["status"], "missing")


if __name__ == "__main__":
    unittest.main()
