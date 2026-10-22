"""Synthetic mixed-stage audit checks; no fixture is flight evidence."""
from copy import deepcopy
import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for path in (ROOT/"scripts", ROOT/"simulations/x650_wholebody_mpc_v1",
             ROOT/"control_packages/x650_wholebody_v1", ROOT/"control_packages/x650_wholebody_v1/tests"):
    sys.path.insert(0, str(path))
import audit_x650_mixed_stage as audit
from accepted_response_model import MODEL_KIND, STRUCTURE, FF_CONTRACT
from mixed_mpc_protocol import build_mixed_mpc_protocol, controller_config, baseline_config
from run_case import FigureEight
from x650_wholebody.adapter import ControllerAdapter
from x650_wholebody.baseline import BaselineController
from x650_wholebody.mpc import MpcController, MIXED_STATE_ORDER
from test_model import example_model


def save(path, value):
    Path(path).write_text(json.dumps(value, allow_nan=False)+"\n", encoding="utf-8")


class MixedStageAuditTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.folder = Path(temp.name).resolve()
        self.protocol = build_mixed_mpc_protocol()
        self.model = example_model(elbow_offset_body_m=(0., .04, 0.))
        self.metadata = {"physical_profile": self.model.metadata(),
                         "reference_base_enu_m": [0., 0., 2.], "reference_rotation": np.eye(3).tolist()}
        self.center = audit.tip_world([0., 0., 2.], np.eye(3), [0., 0.], self.model)
        self.task = {"stage": "tip_hold", "controller": "mpc"}
        self.window = [10., 10.12]
        self.times = np.round(np.arange(0., .120001, .02), 9)
        self.control, self.entries = [], []
        for t in self.times:
            self.control.append({"phase": "measure", "sim_s": 10.+t, "t_s": t,
                "feedback_ax": .1, "feedback_ay": -.2, "feedback_az": .3,
                "base_velocity_target_x": 0., "base_velocity_target_y": 0., "base_velocity_target_z": 0.,
                "qd1_cmd": 0., "qd2_cmd": 0.})
            self.entries.append({"phase": "measure", "sim_s": 10.+t, "t_s": t, "diagnostics": {
                "allocation": {"tip_ref": self.center.tolist(), "base_velocity_cmd": [0., 0., 0.], "qd_cmd": [0., 0.]},
                "acceleration_feedforward_policy": "zero", "acceleration_feedforward_applied": False,
                "command_acceleration_world": [0., 0., 0.], "solver_success": True,
                "accepted_optimum": True, "previous_plan_fallback": False,
                "prediction_response_kind": "xy_second_order_z_first_order_v1", "prediction_state_order": list(MIXED_STATE_ORDER),
                "prediction_response_identified": True,
                "primal_violation": 1e-9, "kkt_stationarity_inf": 1e-5, "solve_ms": 2.,
                "solve_budget_ms": 20., "solve_budget_exceeded": False,
                "measured_acceleration_enu": [.1, -.2, .3], "initial_prediction_acceleration_xy": [.1, -.2]}})

    def solver_audit(self):
        with (self.folder/"controller_diagnostics.jsonl").open("w", encoding="utf-8") as stream:
            for entry in self.entries:
                stream.write(json.dumps(entry)+"\n")
        return audit.mixed_solver_audit(self.folder, self.control, self.metadata, self.task, self.protocol, self.window)

    def test_solver_accepts_only_paired_actual_acceleration_and_zero_sent_feedforward(self):
        result = self.solver_audit()
        self.assertTrue(result["passed"])
        self.assertEqual(result["samples"], len(self.times))
        self.assertTrue(result["zero_acceleration_ff_verified"])
        original = deepcopy(self.entries)
        for key, value in (("measured_acceleration_enu", [0., 0., 0.]),
                           ("initial_prediction_acceleration_xy", [0., 0.]),
                           ("command_acceleration_world", [0., .01, 0.]),
                           ("acceleration_feedforward_policy", "legacy"),
                           ("previous_plan_fallback", True), ("solver_success", False),
                           ("prediction_state_order", list(MIXED_STATE_ORDER[:10]))):
            self.entries = deepcopy(original)
            self.entries[2]["diagnostics"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.solver_audit()

    def test_solver_quality_budget_and_boolean_metrics_cannot_be_relabelled(self):
        self.entries[0]["diagnostics"].update(solve_ms=20.001, solve_budget_exceeded=True)
        self.assertFalse(self.solver_audit()["passed"])
        self.entries[0]["diagnostics"]["solve_budget_exceeded"] = False
        with self.assertRaisesRegex(ValueError, "timing flag"):
            self.solver_audit()
        original = deepcopy(self.entries)
        original[0]["diagnostics"].update(solve_ms=2., solve_budget_exceeded=False)
        for key, value in (("primal_violation", 3e-7), ("kkt_stationarity_inf", .003),
                           ("solve_ms", True), ("solve_ms", float("nan")), ("solve_budget_ms", 30.)):
            self.entries = deepcopy(original)
            self.entries[1]["diagnostics"][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.solver_audit()

    def test_missing_duplicate_shifted_or_wrong_allocator_commands_fail(self):
        original = deepcopy(self.entries)
        modifications = [lambda: self.entries.pop(), lambda: self.entries.append(deepcopy(self.entries[-1])),
            lambda: self.entries[1].update(sim_s=10.021),
            lambda: self.entries[1]["diagnostics"]["allocation"].update(tip_ref=(self.center+[.001, 0., 0.]).tolist()),
            lambda: self.entries[1]["diagnostics"]["allocation"].update(base_velocity_cmd=[.01, 0., 0.]),
            lambda: self.entries[1]["diagnostics"]["allocation"].update(qd_cmd=[.01, 0.])]
        for modification in modifications:
            self.entries = deepcopy(original)
            modification()
            with self.assertRaises(ValueError):
                self.solver_audit()

    def test_phase_marks_exclude_boundary_prestream_and_old_missing_phase_is_rejected(self):
        prestream = deepcopy(self.entries[0])
        prestream.update(phase="settle", t_s=-1.)
        self.entries.insert(0, prestream)
        self.assertTrue(self.solver_audit()["passed"])
        self.entries[2].pop("phase")
        with self.assertRaisesRegex(ValueError, "Every sent command"):
            self.solver_audit()

    def test_pid_also_requires_zero_ff_and_same_original_target_but_no_fake_solver(self):
        self.task["controller"] = "pid_ff"
        for entry in self.entries:
            d = entry["diagnostics"]
            entry["diagnostics"] = {key: d[key] for key in ("allocation", "acceleration_feedforward_policy",
                "acceleration_feedforward_applied", "command_acceleration_world")}
        result = self.solver_audit()
        self.assertTrue(result["passed"])
        self.assertFalse(result["solver_applicable"])
        self.entries[0]["diagnostics"]["command_acceleration_world"][0] = .01
        with self.assertRaises(ValueError):
            self.solver_audit()

    def eight(self, stage="fast"):
        return {"stage": stage, "trajectory_parameters": {"amplitude_x": .25, "amplitude_z": .1,
            "period_s": 8., "cycles": 2, "start_s": 2., "ramp_s": 2.}}

    def test_native_scoring_trajectory_matches_execution_at_all_ramp_boundaries(self):
        task = self.eight()
        trajectory = FigureEight(self.center, **task["trajectory_parameters"])
        times = np.unique(np.r_[np.linspace(0., 24., 1201), [2., 4., 18., 20.],
                                [2.-1e-8, 2.+1e-8, 4.-1e-8, 4.+1e-8, 18.-1e-8, 18.+1e-8]])
        expected = np.asarray([trajectory.sample(t)[0] for t in times])
        np.testing.assert_allclose(audit.declared_tip_positions(task, self.center, times), expected, atol=1e-15)
        with self.assertRaises(ValueError):
            audit.declared_tip_positions(task, self.center, [0., np.nan])

    def test_recorded_targets_allow_actual_short_hold_but_not_offset_or_fitted_phase(self):
        task, window = self.eight(), [10., 16.]
        times = np.round(np.arange(0., 6.0001, .02), 9)
        control = [{"phase": "measure", "sim_s": 10.+t, "t_s": t} for t in times]
        targets = audit.declared_tip_positions(task, self.center, np.maximum(0., times-.02))
        flight = [{"sim_time_s": 10.+t, **{"tip_target_world_"+axis+"_m": p[i] for i, axis in enumerate("xyz")}}
                  for t, p in zip(times, targets)]
        result = audit.logged_tip_target_audit(flight, control, task, self.center, window)
        self.assertTrue(result["verified"])
        flight[200]["tip_target_world_x_m"] += .001
        with self.assertRaisesRegex(ValueError, "original-time held"):
            audit.logged_tip_target_audit(flight, control, task, self.center, window)
        late = audit.declared_tip_positions(task, self.center, np.maximum(0., times-.15))
        for row, p in zip(flight, late):
            row.update({"tip_target_world_"+axis+"_m": p[i] for i, axis in enumerate("xyz")})
        with self.assertRaises(ValueError):
            audit.logged_tip_target_audit(flight, control, task, self.center, window)

    def accepted_model(self):
        return {"kind": MODEL_KIND, "schema_version": 1,
            **{name: True for name in ("identified", "passed", "response_identified", "eligible_as_stage_gate",
                                      "research_acceptance_verified", "ground_truth_aided")},
            "diagnostic_only": False, "model_structure": STRUCTURE, "state_order": ["vx", "ax", "vy", "ay", "vz"],
            "gain": [1., 1., 1.], "bias_m_s": [0., 0., 0.], "command_delay_s": 0.,
            "acceleration_feedforward_contract": FF_CONTRACT, "joint_response_identified": False,
            "full_body_dynamics_identified": False, "parameters": {
                "x": {"wn_rad_s": 2.8, "zeta": .7}, "y": {"wn_rad_s": 2.9, "zeta": .8}, "z": {"tau_s": .2}}}

    def test_executed_metadata_requires_actual_frozen_model_weights_and_policy_without_solving(self):
        gains, model = {"MPC_XY_P": 1.7, "MPC_Z_P": .9}, self.accepted_model()
        for controller in ("pid_ff", "mpc"):
            task = {**self.task, "controller": controller}
            allocator = (MpcController(self.model, gains, config=controller_config(self.protocol, model)) if controller == "mpc"
                         else BaselineController(self.model, gains, config=baseline_config(self.protocol)))
            self.metadata["controller_details"] = ControllerAdapter(allocator, self.model, gains,
                acceleration_feedforward_policy="zero").metadata()
            with patch("x650_wholebody.mpc.minimize", side_effect=AssertionError("no optimizer permitted")):
                self.assertTrue(audit.verify_controller_details(self.metadata, task, self.protocol, model, gains)["verified"])
            self.metadata["controller_details"]["acceleration_feedforward_policy"] = "legacy"
            with self.assertRaises(ValueError):
                audit.verify_controller_details(self.metadata, task, self.protocol, model, gains)

    def archive_fixture(self):
        binding, archived = {}, {}
        launch = self.folder/"launch"
        launch.mkdir()
        for key in ("model", "evidence", "preregistration", "source_archive", "candidate", "training_report"):
            source, copied = self.folder/(key+".source"), launch/(key+".snapshot")
            source.write_text("Synthetic "+key)
            copied.write_bytes(source.read_bytes())
            digest = audit.precision.sha(source)
            binding[key] = {"path": str(source), "sha256": digest}
            archived[key] = {"path": str(copied), "sha256": digest, "source_path": str(source)}
        metadata = {"provenance": {"accepted_response_binding": binding}}
        document = {"provenance": metadata["provenance"], "accepted_response_archive": archived}
        save(launch/"launch.json", document)
        return launch, metadata, binding, document

    def test_all_six_model_snapshots_are_bound_and_corruption_or_missing_member_fails(self):
        launch, meta, binding, document = self.archive_fixture()
        self.assertEqual(len(audit.verify_model_archive(launch, meta, binding)), 6)
        missing = deepcopy(document)
        missing["accepted_response_archive"].pop("training_report")
        save(launch/"launch.json", missing)
        with self.assertRaises(ValueError):
            audit.verify_model_archive(launch, meta, binding)
        save(launch/"launch.json", document)
        Path(document["accepted_response_archive"]["model"]["path"]).write_text("changed")
        with self.assertRaises(ValueError):
            audit.verify_model_archive(launch, meta, binding)

    def report(self, controller="pid_ff", passed=True):
        common = {key: {} for key in ("operating_condition", "accepted_response_binding")}
        common.update({key: "fixture" for key in ("operating_condition_sha256", "pid_parameters_sha256", "model_sha256",
            "source_urdf_sha256", "feedback_contract", "control_interface")})
        common.update({key: True for key in ("full_window_ulog_verified", "parameter_restore_verified",
            "upper_command_verified", "feedback_contract_verified")})
        return {**common, "kind": audit.AUDIT_KIND, "stage": "small", "source_stage": "small", "controller": controller,
            "source_run": "synthetic", "source_run_directory": str(self.folder), "passed": passed,
            "stage_acceptance": {"passed": passed}, "ground_acceptance": {"passed": True},
            "mixed_mpc_protocol": self.protocol, "mixed_mpc_protocol_sha256": audit.protocol_hash(self.protocol),
            "acceleration_feedforward": "zero"}

    def test_certificate_filename_baseline_alias_and_immutable_source_binding(self):
        output, report = self.folder/"accepted", self.report()
        source = ROOT/"scripts/audit_x650_mixed_stage.py"
        with patch.object(audit, "analyze", return_value=(report, {str(source): audit.precision.sha(source)})):
            audit.audit(self.folder, self.folder, output)
        self.assertFalse((output/"upper_stage_audit.json").exists())
        for filename in (audit.AUDIT_FILENAME, "research_stage.json", "research_ground.json", "pid_baseline.json"):
            raw = (output/filename).read_bytes()
            self.assertTrue(raw.endswith(b"\n"))
            self.assertIsInstance(json.loads(raw), dict)
        certificate = json.loads((output/"pid_baseline.json").read_text())
        self.assertEqual(certificate["stage"], "pid_baseline")
        self.assertEqual(certificate["source_stage"], "small")
        self.assertEqual(certificate["controller"], "pid_ff")
        self.assertEqual(certificate["acceleration_feedforward"], "zero")
        self.assertEqual(certificate["mixed_mpc_protocol"], self.protocol)
        self.assertFalse(any(Path(path).suffix == ".py" for path in certificate["artifacts"]))
        saved = json.loads((output/audit.AUDIT_FILENAME).read_text())["analysis_source_archive"]
        with zipfile.ZipFile(saved["path"]) as zipped:
            for member, info in saved["members"].items():
                self.assertEqual(hashlib.sha256(zipped.read(member)).hexdigest(), info["sha256"])

    def test_native_error_keeps_constant_bias_in_full_window_acceptance(self):
        self.solver_audit()  # Persist paired diagnostics for the helper.
        for name, records in (("control_trace.csv", self.control), ("flight_samples.csv", [
                {"phase": "measure", "sim_time_s": 10.+t,
                 **{"tip_target_world_"+axis+"_m": self.center[i] for i, axis in enumerate("xyz")}}
                for t in self.times])):
            with (self.folder/name).open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(records[0]))
                writer.writeheader()
                writer.writerows(records)
        self.metadata.update(folded_before_landing=True,
            provenance={"preparation_report_sha256": self.metadata["physical_profile"]["source_report_sha256"]})
        physical = {"t": self.times, "q": np.zeros((len(self.times), 2)),
                    "tip": np.tile(self.center+[.06, 0., 0.], (len(self.times), 1))}
        with patch.object(audit.joint_audit, "native_tip_audit", return_value={"passed": True}):
            result = audit.mixed_stage_metrics(self.folder, self.metadata, self.task, self.protocol,
                                              self.window, physical, {"synthetic_prechecks": True})
        self.assertFalse(result["passed"])
        self.assertAlmostEqual(result["native_tip_error"]["rmse_3d"], .06)
        self.assertFalse(result["criteria"]["tip_rms_within_predeclared_limit"])

    def test_failed_criteria_or_exception_never_issue_stage_or_baseline_certificates(self):
        for failed in (False, True):
            output = self.folder/("exception" if failed else "criteria")
            with patch.object(audit, "analyze", side_effect=ValueError("synthetic invalid") if failed else None,
                              return_value=(self.report(passed=False), {})):
                with self.assertRaises(ValueError):
                    audit.audit(self.folder, self.folder, output)
            self.assertTrue((output/audit.AUDIT_FILENAME).is_file())
            self.assertFalse((output/"research_stage.json").exists())
            self.assertFalse((output/"pid_baseline.json").exists())
            json.loads((output/audit.AUDIT_FILENAME).read_text())


if __name__ == "__main__":
    unittest.main()
