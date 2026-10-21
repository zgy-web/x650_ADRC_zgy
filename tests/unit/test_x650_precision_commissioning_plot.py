"""Synthetic plotting contracts; these fixtures are not flight evidence."""
from copy import deepcopy
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("x650_commissioning_plot_test",
    ROOT/"scripts/plot_x650_precision_commissioning.py")
plot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot)
from diagnostic_parameters import PRECISION_PROFILE
from hover_reference import PINNED_PX4


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2)+"\n", encoding="utf-8")


def fixture(folder, stage):
    run, audit = folder/("synthetic_"+stage), folder/("synthetic_"+stage+"_audit")
    run.mkdir(parents=True)
    audit.mkdir()
    duration = 20. if stage == "hover_folded" else 30.
    t = np.arange(round(duration/.004)+1)*.004
    actual_pid = {name: 1. for name in plot.CONTROL_NAMES}
    condition = {"kind": "x650_research_operating_condition", "schema_version": 1,
        "research_profile": "precision_ev_selected_pid_v1", "scope": "simulation_truth_aided_ev_commissioning",
        "model_sha256": "a"*64, "estimator_parameters": PRECISION_PROFILE,
        "observer": {"rate_hz": 50., "gaussian_noise": 0., "measurement_point": "base_link"},
        "coordinate_contract": "raw_ev_world", "px4_source_contract_commit": PINNED_PX4,
        "control_interface": "fixed_world_position_v1", "pid_parameters_sha256": plot.canonical_hash(actual_pid),
        "world_sdf_sha256": "b"*64, "ground_truth_aided": True,
        "gps_performance_claim": False, "hardware_performance_claim": False}
    task = {"stage": stage, "duration_s": duration}
    metadata = {"complete": True, "landed_and_disarmed": True, "task": task,
        "operating_condition": condition, "operating_condition_sha256": plot.canonical_hash(condition),
        "actual_pid_parameters": actual_pid, "model_sha256": "a"*64, "source_urdf_sha256": "c"*64,
        "reference_base_enu_m": [0., 0., 2.], "task_start_sim_s": 40., "task_end_sim_s": 40.+duration}
    write_json(run/"controller_metadata.json", metadata)
    write_json(run/"controller_task.json", task)
    write_json(run/"hover_reference.json", {"target_world_enu_m": [0., 0., 2.], "frame_mode": "raw_ev_world"})
    qref, dqref = plot.joint_helpers.reference(t, duration) if stage == "joint_probe" else (np.zeros((len(t), 2)), np.zeros((len(t), 2)))
    q = qref+np.column_stack((.0004*np.sin(t), .0003*np.sin(2*t)))
    fields = ["sim_s", "valid_base", "valid_joint_state", "valid_tip", *["base_p"+a for a in "xyz"],
              *["base_q"+a for a in "wxyz"], "arm_joint_1_q", "arm_joint_2_q",
              "arm_joint_1_qd", "arm_joint_2_qd", *["tip_"+a for a in "xyz"]]
    with (run/"native_trace.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for index, time in enumerate(t):
            writer.writerow([40.+time, 1, 1, 1, .001*np.sin(time), .0008*np.cos(time), 2.+.0002*np.sin(2*time),
                             1, 0, 0, 0, *q[index], *dqref[index], 0, 0, 0])
    write_json(run/"native_trace_manifest.json", {"complete": True, "sha256": plot.sha(run/"native_trace.csv"),
        "model_sha256": "a"*64, "source_urdf_sha256": "c"*64})
    segments = {"whole": {"sample_count": len(t), "actual_interval_task_s": [0., duration],
                           "q_tracking_rmse_deg": [.016, .012], "q_tracking_peak_deg": [.023, .018]}}
    segment = (duration-4.)/3.
    for index, name in enumerate(("J1_only", "J2_only", "both")):
        segments[name] = {"declared_interval_task_s": [2.+index*segment, 2.+(index+1)*segment]}
    names = ("controller_metadata.json", "controller_task.json", "hover_reference.json", "native_trace.csv", "native_trace_manifest.json")
    report = {"kind": "x650_precision_research_stage_audit", "stage": stage, "simulation_backend": "gazebo_px4",
        **{key: True for key in ("complete", "passed", "research_acceptance_verified", "full_window_ulog_verified",
                               "parameter_restore_verified", "ground_truth_aided")},
        "operating_condition": condition, "operating_condition_sha256": plot.canonical_hash(condition),
        "pid_parameters_sha256": condition["pid_parameters_sha256"], "source_run": run.name,
        "model_sha256": "a"*64, "source_urdf_sha256": "c"*64,
        "window_sim_s": [40., 40.+duration], "target_world_enu_m": [0., 0., 2.],
        "position_native_world_enu": {"unit": "m", "sample_count": len(t), "duration_s": duration,
                                      "rmse_3d": .0011, "peak_3d": .0015},
        "stage_acceptance": {"segments": segments},
        "inputs_sha256": {"/original/server/runs/"+run.name+"/"+name: plot.sha(run/name) for name in names}}
    write_json(audit/"precision_stage_audit.json", report)
    certificate = {"kind": "x650_native_stage_evidence", "stage": stage, "source_run": run.name,
        "eligible_as_stage_gate": True, "passed": True, "research_acceptance_verified": True,
        "operating_condition_sha256": plot.canonical_hash(condition),
        "artifacts": {"/original/server/audits/"+audit.name+"/precision_stage_audit.json": plot.sha(audit/"precision_stage_audit.json")}}
    write_json(audit/"research_stage.json", certificate)
    return run, audit


class CommissioningPlotTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.hover = fixture(self.folder, "hover_folded")
        self.joint = fixture(self.folder, "joint_probe")

    def test_full_native_samples_and_existing_reference_are_preserved(self):
        value = plot.load_stage(*self.joint, "joint_probe")
        self.assertEqual(len(value["data"]["t"]), 7501)
        np.testing.assert_allclose(value["qref"], plot.joint_helpers.reference(value["data"]["t"], 30.)[0])
        self.assertEqual(value["position_metrics"]["rmse_3d"], .0011)
        self.assertEqual(value["phases"][0]["start_s"], 2.)

    def test_integer_equivalent_readbacks_use_same_float_digest_without_weakening_contract(self):
        values = {name: 1. for name in plot.CONTROL_NAMES}
        expected = plot.actual_pid_digest(values)
        values["MC_AIRMODE"] = 1
        values["MPC_USE_HTE"] = 1
        self.assertEqual(plot.actual_pid_digest(values), expected)
        values["MC_AIRMODE"] = 0
        self.assertNotEqual(plot.actual_pid_digest(values), expected)
        values["MC_AIRMODE"] = True
        with self.assertRaisesRegex(ValueError, "finite numeric"):
            plot.actual_pid_digest(values)
        values["MC_AIRMODE"] = 1.
        values["unexpected_parameter"] = values.pop("MPC_USE_HTE")
        with self.assertRaisesRegex(ValueError, "42 canonical"):
            plot.actual_pid_digest(values)

    def test_delivered_metadata_with_integer_control_parameters_is_accepted(self):
        run, audit = self.hover
        metadata = plot.read_json(run/"controller_metadata.json")
        metadata["actual_pid_parameters"].update(MC_AIRMODE=1, MPC_USE_HTE=1)
        write_json(run/"controller_metadata.json", metadata)
        report = plot.read_json(audit/"precision_stage_audit.json")
        for source in report["inputs_sha256"]:
            if Path(source).name == "controller_metadata.json":
                report["inputs_sha256"][source] = plot.sha(run/"controller_metadata.json")
        write_json(audit/"precision_stage_audit.json", report)
        certificate = plot.read_json(audit/"research_stage.json")
        certificate["artifacts"] = {source: plot.sha(audit/"precision_stage_audit.json") for source in certificate["artifacts"]}
        write_json(audit/"research_stage.json", certificate)
        value = plot.load_stage(*self.hover, "hover_folded")
        self.assertEqual(value["condition"]["pid_parameters_sha256"],
                         plot.canonical_hash({name: 1. for name in plot.CONTROL_NAMES}))

    def test_rejected_or_wrong_stage_audit_cannot_plot(self):
        path = self.hover[1]/"precision_stage_audit.json"
        value = plot.read_json(path)
        value["passed"] = False
        write_json(path, value)
        with self.assertRaisesRegex(ValueError, "accepted research"):
            plot.load_stage(*self.hover, "hover_folded")
        with self.assertRaisesRegex(ValueError, "joint_probe audit"):
            plot.load_stage(*self.hover, "joint_probe")

    def test_certificate_binding_and_original_native_hash_are_required(self):
        (self.hover[0]/"native_trace.csv").write_text("tampered")
        with self.assertRaisesRegex(ValueError, "accepted original run"):
            plot.load_stage(*self.hover, "hover_folded")
        report_path = self.joint[1]/"precision_stage_audit.json"
        value = plot.read_json(report_path)
        value["position_native_world_enu"]["rmse_3d"] = .012
        write_json(report_path, value)
        with self.assertRaisesRegex(ValueError, "bind this audit"):
            plot.load_stage(*self.joint, "joint_probe")

    def test_pair_cannot_change_pid_condition(self):
        hover = plot.load_stage(*self.hover, "hover_folded")
        joint = plot.load_stage(*self.joint, "joint_probe")
        joint["condition"] = deepcopy(joint["condition"])
        joint["condition"]["pid_parameters_sha256"] = "f"*64
        with self.assertRaisesRegex(ValueError, "operating conditions"):
            plot.verify_pair(hover, joint)

    def test_one_scheduler_tail_step_is_plotted_without_recomputing_joint_metrics(self):
        run, audit = self.joint
        native_path = run/"native_trace.csv"
        with native_path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        tail = dict(rows[-1])
        tail["sim_s"] = "70.004"
        with native_path.open("a", newline="", encoding="utf-8") as stream:
            csv.DictWriter(stream, fieldnames=list(tail)).writerow(tail)
        metadata = plot.read_json(run/"controller_metadata.json")
        metadata["task_end_sim_s"] = 70.004
        write_json(run/"controller_metadata.json", metadata)
        native_manifest = plot.read_json(run/"native_trace_manifest.json")
        native_manifest["sha256"] = plot.sha(native_path)
        write_json(run/"native_trace_manifest.json", native_manifest)
        report = plot.read_json(audit/"precision_stage_audit.json")
        report["window_sim_s"][1] = 70.004
        report["position_native_world_enu"].update(sample_count=7502, duration_s=30.004)
        report["inputs_sha256"] = {source: plot.sha(run/Path(source).name) for source in report["inputs_sha256"]}
        write_json(audit/"precision_stage_audit.json", report)
        certificate = plot.read_json(audit/"research_stage.json")
        certificate["artifacts"] = {source: plot.sha(audit/"precision_stage_audit.json") for source in certificate["artifacts"]}
        write_json(audit/"research_stage.json", certificate)
        value = plot.load_stage(*self.joint, "joint_probe")
        self.assertEqual(len(value["data"]["t"]), 7502)
        self.assertAlmostEqual(value["data"]["t"][-1], 30.004)
        self.assertEqual(value["joint_metrics"]["sample_count"], 7501)
        self.assertEqual(value["joint_metrics"]["q_tracking_rmse_deg"], [.016, .012])

    def test_render_manifest_copies_audit_metrics_and_never_overwrites(self):
        output = self.folder/"explicit_synthetic_fixture_plot"
        result = plot.generate(*self.hover, *self.joint, output)
        self.assertTrue(result["complete"])
        self.assertFalse(result["native_curves_resampled"])
        self.assertEqual(result["runs"]["joint"]["native_samples_plotted"], 7501)
        self.assertEqual(result["runs"]["joint"]["position_native_world_enu"]["rmse_3d"], .0011)
        for entry in result["outputs"]:
            self.assertGreater(entry["bytes"], 10000)
            self.assertEqual(plot.sha(output/entry["path"]), entry["sha256"])
        with self.assertRaisesRegex(ValueError, "new plot output"):
            plot.generate(*self.hover, *self.joint, output)


if __name__ == "__main__":
    unittest.main()
