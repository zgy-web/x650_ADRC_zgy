"""Synthetic unit fixtures; these files are not native flight evidence."""
import copy
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("step_analysis", ROOT/"scripts/analyze_x650_position_steps.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class PositionStepAnalysisTests(unittest.TestCase):
    def fixture(self):
        anchor = np.array([0., 0., 1.5])
        reference = dict(kind="x650_fixed_world_hover_reference", schema_version=1, profile="fixed_world_position_v1",
            frame_mode="raw_ev_world", target_frozen=True, truth_feedback_in_setpoint=False,
            fit_estimator_translation=False, coordinate_frame=1, type_mask=2552,
            px4_source_contract_commit=audit.physics.PINNED_PX4,
            target_world_enu_m=anchor.tolist(), target_local_ned_m=[0., 0., 8.5], fixed_yaw_ned_rad=0.,
            px4_reference_at_bind=dict(ref_lat=47., ref_lon=8., ref_alt=10., xy_valid=True, z_valid=True,
                xy_global=False, z_global=True, xy_reset_counter=0, z_reset_counter=0, heading_reset_counter=0),
            world={"source_sha256": "SYNTHETIC"}, source_sha256=audit.sha(audit.DEFAULT_REFERENCE_SOURCE))
        protocol = audit.module(audit.DEFAULT_PROBE_SOURCE, "step_fixture_protocol").probe_metadata()
        events = []
        for segment in protocol["segments"]:
            bound = copy.deepcopy(reference)
            target = anchor+segment["target_offset_world_enu_m"]
            bound["target_world_enu_m"] = target.tolist()
            bound["target_local_ned_m"] = [target[1], target[0], 10.-target[2]]
            segment["reference"] = bound
            events.append(dict(segment_index=segment["segment_index"], first_send_sim_s=10.+segment["start_s"],
                elapsed_s=segment["start_s"], position_target_local_ned_m=bound["target_local_ned_m"], target_world_enu_m=target.tolist()))
        metadata = dict(complete=True, landed_and_disarmed=True, task_start_sim_s=10., task_end_sim_s=30.,
            experiment_end_sim_s=66., position_step_send_events=events, position_step_probe=protocol,
            model_sha256="SYNTHETIC_MODEL", source_urdf_sha256="SYNTHETIC_URDF")
        return anchor, reference, protocol, metadata

    def test_source_contract_and_fixed_target_reject_recenter_or_protocol_drift(self):
        anchor, reference, protocol, metadata = self.fixture()
        segments, contract = audit.verified_segments(protocol, reference, audit.DEFAULT_PROBE_SOURCE, audit.DEFAULT_REFERENCE_SOURCE)
        self.assertEqual(len(segments), 6)
        self.assertEqual(contract["settling_continuous_duration_s"], 2.)
        protocol["segments"][1]["reference"]["target_world_enu_m"][0] += .01
        with self.assertRaisesRegex(ValueError, "absolute target"):
            audit.verified_segments(protocol, reference, audit.DEFAULT_PROBE_SOURCE, audit.DEFAULT_REFERENCE_SOURCE)
        _, reference, protocol, _ = self.fixture()
        protocol["analysis_contract"]["settling_speed_m_s"] = .1
        with self.assertRaisesRegex(ValueError, "protocol"):
            audit.verified_segments(protocol, reference, audit.DEFAULT_PROBE_SOURCE, audit.DEFAULT_REFERENCE_SOURCE)

    def test_actual_first_send_controls_timing_and_missing_send_is_rejected(self):
        anchor, reference, protocol, metadata = self.fixture()
        for event in metadata["position_step_send_events"]:
            event["first_send_sim_s"] += .016
            event["elapsed_s"] += .016
        windows = audit.actual_windows(metadata, protocol["segments"], anchor)
        self.assertAlmostEqual(windows[0]["actual_window_sim_s"][0], 30.016)
        self.assertAlmostEqual(windows[0]["command_send_delay_s"], .016)
        self.assertAlmostEqual(windows[-1]["observation_duration_s"], 5.984)
        metadata["position_step_send_events"].pop(2)
        with self.assertRaisesRegex(ValueError, "six steps"):
            audit.actual_windows(metadata, protocol["segments"], anchor)

    def test_return_step_sign_rise_and_overshoot_use_commanded_endpoints(self):
        anchor, reference, protocol, metadata = self.fixture()
        segment = protocol["segments"][1]
        t = np.arange(0., 6.0001, .004)
        position = np.tile(anchor, (len(t), 1)); position[:, 0] = .15*np.exp(-t/.3)
        velocity = np.zeros_like(position); velocity[:, 0] = -.5*np.exp(-t/.3)
        native = dict(t=t+36., position=position, velocity=velocity, tilt=np.zeros(len(t)))
        report = audit.response_metrics(native, segment, [36., 42.], anchor, protocol["analysis_contract"])
        self.assertEqual(report["direction"], -1)
        self.assertAlmostEqual(report["rise_10_90_s"], .3*np.log(9), places=4)
        self.assertAlmostEqual(report["overshoot_m"], 0.)
        self.assertTrue(report["settling"]["settled_through_window_end"])
        native["position"][1000, 0] = -.03
        report = audit.response_metrics(native, segment, [36., 42.], anchor, protocol["analysis_contract"])
        self.assertAlmostEqual(report["overshoot_percent_of_commanded_step"], 20.)
        self.assertAlmostEqual(report["cross_axis_position_error"]["peak_3d"], 0.)

    def test_high_speed_or_short_window_does_not_count_as_settled(self):
        t = np.arange(0., 6.0001, .004)
        error, velocity = np.zeros((len(t), 3)), np.zeros((len(t), 3))
        velocity[:, 1] = .04
        result = audit.settling(t, error, velocity)
        self.assertFalse(result["settled_2s_observed"])
        self.assertIsNone(result["first_qualified_entry_time_s"])
        velocity[:] = 0.; error[t < 4.1, 0] = .03
        self.assertFalse(audit.settling(t, error, velocity)["settled_2s_observed"])
        error[:] = 0.; error[t > 4., 0] = .03
        result = audit.settling(t, error, velocity)
        self.assertTrue(result["settled_2s_observed"])
        self.assertFalse(result["settled_through_window_end"])

    def test_rise_censored_or_never_reached_is_not_invented(self):
        t = np.arange(0., 6.0001, .004)
        self.assertEqual(audit.first_crossing(t, np.full(len(t), .2), .1)["status"], "left_censored")
        self.assertEqual(audit.first_crossing(t, np.full(len(t), .2), .9)["status"], "not_reached")

    def test_complete_native_pipeline_and_joint_validity_gate(self):
        anchor, reference, protocol, metadata = self.fixture()
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)/"SYNTHETIC_CODE_CHECK"; folder.mkdir()
            for filename, document in (("controller_metadata.json", metadata), ("hover_reference.json", reference),
                                       ("position_step_probe.json", protocol)):
                (folder/filename).write_text(json.dumps(document), encoding="utf-8")
            path = folder/"native_trace.csv"
            names = ["sim_s", "valid_base", "valid_joint_state", "valid_tip", *["base_p"+a for a in "xyz"],
                *["base_v"+a for a in "xyz"], *["base_q"+a for a in "wxyz"], "arm_joint_1_q", "arm_joint_2_q",
                "arm_joint_1_qd", "arm_joint_2_qd", "tip_x", "tip_y", "tip_z"]
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=names); writer.writeheader()
                for t in np.arange(10., 66.0001, .004):
                    row = {key: 0 for key in names}
                    row.update(sim_s=t, valid_base=1, valid_joint_state=1, valid_tip=1, base_qw=1., base_pz=1.5)
                    writer.writerow(row)
            manifest = dict(complete=True, sha256=audit.sha(path), model_sha256=metadata["model_sha256"],
                            source_urdf_sha256=metadata["source_urdf_sha256"], task_window_sim_s=[10., 66.])
            (folder/"native_trace_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            result = audit.analyze(folder, Path(temp)/"audit")
            self.assertEqual(len(result["segments"]), 6)
            self.assertFalse(result["all_segments_observed_2s_settling"])
            self.assertIsNone(result["segments"][0]["rise_10_90_s"])
            self.assertAlmostEqual(result["segments"][0]["position_error_world_enu"]["rmse_3d"], .15)
            self.assertTrue(result["segments"][1]["settling"]["settled_through_window_end"])
            lines = path.read_text().splitlines()
            fields = lines[1].split(","); fields[3] = "0"; lines[1] = ",".join(fields)
            path.write_text("\n".join(lines)+"\n")
            with self.assertRaisesRegex(ValueError, "valid native"):
                audit.native_manipulator_validity(path, [10., 66.])


if __name__ == "__main__":
    unittest.main()
