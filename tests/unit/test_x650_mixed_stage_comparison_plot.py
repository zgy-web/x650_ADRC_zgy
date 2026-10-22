"""Synthetic viewer checks; these fixtures are not flight/acceptance evidence."""
import copy
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"scripts"))
import plot_x650_mixed_stage_comparison as plot
from mixed_mpc_protocol import build_mixed_mpc_protocol


def write_json(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def fixture(root, controller, start, center):
    folder = root/controller
    folder.mkdir()
    protocol = build_mixed_mpc_protocol()
    task = dict(controller=controller, stage="tip_hold", duration_s=.2, control_period_s=.02,
                mixed_mpc_protocol=protocol)
    metadata = dict(task=task, model_sha256="a"*64, source_urdf_sha256="b"*64,
                    task_start_sim_s=start, task_end_sim_s=start+.2,
                    reference_base_enu_m=[0, 0, 1], physical_profile={"fixture": "synthetic"})
    write_json(folder/"controller_metadata.json", metadata)
    write_json(folder/"controller_task.json", task)
    native = []
    for t in np.linspace(0, .2, 51):
        native.append(dict(sim_s=start+t, valid_base=1, valid_joint_state=1, valid_tip=1,
            base_px=0, base_py=0, base_pz=1, base_qw=1, base_qx=0, base_qy=0, base_qz=0,
            arm_joint_1_q=0, arm_joint_2_q=0, arm_joint_1_qd=0, arm_joint_2_qd=0,
            tip_x=center[0]+.01, tip_y=center[1], tip_z=center[2]))
    write_csv(folder/"native_trace.csv", native)
    controls, diagnostics = [], []
    for t in np.linspace(0, .2, 11):
        controls.append(dict(sim_s=start+t, phase="measure", qd1_cmd=0, qd2_cmd=0, compute_ms=3))
        diagnostics.append(dict(sim_s=start+t, phase="measure", diagnostics={"solve_ms": 2}))
    write_csv(folder/"control_trace.csv", controls)
    (folder/"controller_diagnostics.jsonl").write_text("\n".join(json.dumps(row) for row in diagnostics), encoding="utf-8")
    physical = plot.native_reader.native_arrays(plot.csv_rows(folder/"native_trace.csv"), start, .2, [0, 0, 1])
    report = {key: True for key in plot.FLAGS}
    report.update(kind="x650_precision_mixed_stage_audit", schema_version=3,
        controller=controller, source_run=folder.name, source_run_directory=str(folder.resolve()),
        simulation_backend="gazebo_px4", acceleration_feedforward="zero", no_fitted_origin_phase_or_mean=True,
        stage="tip_hold", source_stage="tip_hold", model_sha256="a"*64, source_urdf_sha256="b"*64,
        pid_parameters_sha256="c"*64, feedback_contract="fixture", control_interface="fixture",
        mixed_mpc_protocol=protocol, mixed_mpc_protocol_sha256=plot.canonical(protocol),
        operating_condition={"fixture": "synthetic"}, operating_condition_sha256=plot.canonical({"fixture": "synthetic"}),
        window_sim_s=[start, start+.2], stage_acceptance={"passed": True, "criteria": {"fixture": True},
            "reference_center_world_enu_m": center,
            "native_tip_error": plot.precision.metrics(start+physical["t"], physical["tip"]-center, "m")},
        accepted_response_binding={key: {"sha256": "d"*64} for key in
            ("model", "evidence", "preregistration", "source_archive", "candidate", "training_report")},
        inputs_sha256={str(p.resolve()): plot.sha(p) for p in folder.iterdir()})
    path = folder/"mixed_mpc_stage_audit.json"
    write_json(path, report)
    return path, folder


class ComparisonPlotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.pid = fixture(self.root, "pid_ff", 10., [0, 0, .6])
        self.mpc = fixture(self.root, "mpc", 25., [.001, -.002, .6001])

    def tearDown(self):
        self.temp.cleanup()

    def test_render_uses_native_position_and_keeps_prebound_centers_and_bias(self):
        # Any numerical fit or controller optimization in the viewer is a bug.
        with patch("scipy.optimize.least_squares", side_effect=AssertionError("no fit")), \
             patch("scipy.optimize.minimize", side_effect=AssertionError("no optimize")):
            manifest = plot.generate(*self.pid, *self.mpc, self.root/"plots")
        self.assertFalse(manifest["eligible_as_stage_gate"])
        self.assertFalse(manifest["refitting"])
        self.assertEqual(len(manifest["outputs_sha256"]), 4)
        for path, digest in manifest["outputs_sha256"].items():
            self.assertEqual(plot.sha(path), digest)
            self.assertGreater(Path(path).stat().st_size, 1000)
        for run in manifest["runs"]:
            self.assertAlmostEqual(run["native_tip_error"]["rmse_3d"], .01)
            self.assertAlmostEqual(run["native_tip_error"]["mean_vector"][0], .01)
        self.assertNotEqual(manifest["runs"][0]["reference_center_world_enu_m"], manifest["runs"][1]["reference_center_world_enu_m"])
        self.assertIsNone(manifest["runs"][0]["maximum_optimizer_solve_ms"])
        self.assertEqual(manifest["runs"][1]["maximum_optimizer_solve_ms"], 2.)

    def test_rejects_unpassed_and_nested_unpassed_audit(self):
        original = plot.read(self.pid[0])
        for change in ("passed", "nested"):
            report = copy.deepcopy(original)
            if change == "passed": report["passed"] = False
            else: report["stage_acceptance"]["criteria"]["fixture"] = False
            write_json(self.pid[0], report)
            with self.assertRaisesRegex(ValueError, "accepted|criteria"):
                plot.load_run(*self.pid, "pid_ff")

    def test_rejects_changed_native_and_mismatched_statistics(self):
        path = self.pid[1]/"native_trace.csv"
        path.write_text(path.read_text()+"\n")
        with self.assertRaisesRegex(ValueError, "missing or changed"):
            plot.load_run(*self.pid, "pid_ff")
        report = plot.read(self.pid[0])
        report["inputs_sha256"][str(path.resolve())] = plot.sha(path)
        report["stage_acceptance"]["native_tip_error"]["rmse_3d"] = 0.
        write_json(self.pid[0], report)
        with self.assertRaisesRegex(ValueError, "do not reproduce"):
            plot.load_run(*self.pid, "pid_ff")

    def test_rejects_different_stage_model_protocol_or_accepted_model(self):
        pid, mpc = plot.load_run(*self.pid, "pid_ff"), plot.load_run(*self.mpc, "mpc")
        for key in ("stage", "model_sha256", "mixed_mpc_protocol_sha256", "pid_parameters_sha256"):
            other = copy.deepcopy(mpc)
            other["report"][key] = "different"
            with self.assertRaisesRegex(ValueError, "matched comparison"):
                plot.verify_pair(pid, other)
        mpc["report"]["accepted_response_binding"]["model"]["sha256"] = "e"*64
        with self.assertRaisesRegex(ValueError, "response evidence differs"):
            plot.verify_pair(pid, mpc)

    def test_existing_output_preserved_and_orientation_is_world(self):
        output = self.root/"existing"
        output.mkdir()
        marker = output/"keep.txt"
        marker.write_text("keep")
        with self.assertRaisesRegex(ValueError, "new output directory"):
            plot.generate(*self.pid, *self.mpc, output)
        self.assertEqual(marker.read_text(), "keep")
        angle = np.pi/4
        np.testing.assert_allclose(plot.rpy_degrees([[np.cos(angle), 0, 0, np.sin(angle)]]), [[0, 0, 90]], atol=1e-10)


if __name__ == "__main__":
    unittest.main()
