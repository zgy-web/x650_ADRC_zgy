"""Independent synthetic witnesses; these tests are never flight evidence."""
from copy import deepcopy
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
import unittest
from unittest.mock import patch
from uuid import uuid4

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for path in (ROOT, ROOT / "scripts", Path(__file__).parent, ROOT / "control_packages/x650_wholebody_v1"):
    sys.path.insert(0, str(path))
import audit_x650_mixed_feedback as audit
from simulations.x650_ts20_hold_v1 import audit as ts20_audit
from test_x650_mixed_estimated_feedback import audit_fixture


class CausalFeedbackAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ulog, cls.native_acceleration, reference, _, controls, feedback = audit_fixture()
        cls.original = (reference, controls[:3], feedback[:3])

    def fixture(self):
        reference, controls, feedback = deepcopy(self.original)
        window = [float(controls[0]["sim_s"]), float(controls[-1]["sim_s"])]
        states = []
        for stamp in np.round(np.arange(window[0]-.004, window[1]+.0041, .004), 9):
            row = {"sim_s": float(stamp)}
            for j in (1, 2):
                row.update({f"q{j}_physical": .02*j+.001*(stamp-window[0]),
                            f"q{j}_internal": .02*j+.001*(stamp-window[0]),
                            f"dq{j}_internal": .001,
                            f"r{j}": .03*j+.02*(stamp-window[0]),
                            f"u{j}": .02, f"upper_sequence{j}": 1., f"receipt{j}_s": window[0]-.01})
            states.append(row)
        executions, solves = [], []
        for index, (row, entry) in enumerate(zip(controls, feedback)):
            now = float(row["sim_s"])
            source = min(states, key=lambda value: abs(value["sim_s"]-(now-.004)))
            diagnostic = entry["feedback"]
            raw = diagnostic["callbacks"].pop("joints")
            diagnostic["raw_ros_joint_diagnostic"] = {"used_for_control": False,
                "header_stamp_s": now+.004, "source_age_s": -.004,
                "callback_sequence": raw["sequence"]}
            diagnostic["joint_observation"] = {"source_kind": audit.TS20_JOINT_PROVIDER_KIND,
                "generation": 7+index*3, "control_time_s": now, "source_stamp_s": source["sim_s"],
                "source_age_limit_s": .025, "preselected_once": True, "projection_applied": False}
            diagnostic["joint_stamp_s"] = row["joint_stamp_s"] = source["sim_s"]
            diagnostic["joint_source_age_s"] = now-source["sim_s"]
            state = diagnostic["control_state"]
            state["q_rad"] = [source[f"q{j}_physical"] for j in (1, 2)]
            state["dq_rad_s"] = [source[f"dq{j}_internal"] for j in (1, 2)]
            for j in (1, 2):
                row[f"q{j}"] = state["q_rad"][j-1]
                row[f"qd{j}"] = state["dq_rad_s"][j-1]
            actual = {key: deepcopy(state[field]) for key, field in (
                ("position", "position_enu_m"), ("rotation", "rotation_flu_to_enu"),
                ("velocity", "velocity_enu_m_s"), ("omega_body", "omega_flu_rad_s"),
                ("q", "q_rad"), ("dq", "dq_rad_s"), ("acceleration_enu", "acceleration_enu_m_s2"))}
            actual.update(stamp_s=diagnostic["state_stamp_s"], control_time_s=now,
                          joint_stamp_s=source["sim_s"])
            executions.append({"event": "controller_step_returned", "phase": "measure", "t_s": row["t_s"],
                "state": actual, "feedback_snapshot": deepcopy(diagnostic),
                "compute_before": {"processed_estimator_clock_s": now}})
            solves.append({"phase": "measure", "sim_s": now, "t_s": row["t_s"],
                "diagnostics": {"ts20_reference_state_rad": [.03*j+.02*(now-window[0]) for j in (1, 2)]}})
        return reference, controls, feedback, window, states, executions, solves

    def check(self, fixture):
        reference, controls, feedback, window, states, execution, _ = fixture
        return audit.feedback_check_mixed(controls, feedback, reference, window,
            expected_joint_provider_kind=audit.TS20_JOINT_PROVIDER_KIND,
            joint_provider_records=states, execution_records=execution)

    def test_positive_actual_source_and_same_execution_with_raw_future_diagnostic(self):
        fixture = self.fixture()
        report, _ = self.check(fixture)
        self.assertTrue(report["joint_provider"]["same_round_serialized_snapshot_binding"])
        self.assertTrue(report["joint_provider"]["physical_q_endpoint_dq_verified"])
        self.assertFalse(report["joint_provider"]["opaque_python_object_identity_audited_offline"])
        self.assertNotIn("joints", report["maximum_callback_receipt_age_s"])
        self.assertEqual(report["joint_provider"]["generation_range"], [7, 13])
        reference_report, selected = ts20_audit.causal_reference_binding(fixture[4], fixture[2], fixture[6], fixture[3])
        self.assertTrue(reference_report["verified"])
        self.assertEqual(len(selected), 3)

    def test_default_old_branch_preserves_ros_callback_requirement(self):
        reference, controls, feedback = deepcopy(self.original)
        window = [float(controls[0]["sim_s"]), float(controls[-1]["sim_s"])]
        report, _ = audit.feedback_check_mixed(controls, feedback, reference, window)
        self.assertNotIn("joint_provider", report)
        fixture = self.fixture()
        with self.assertRaisesRegex(ValueError, "Missing independently aged"):
            audit.feedback_check_mixed(fixture[1], fixture[2], fixture[0], fixture[3])

    def test_declaration_cannot_replace_actual_source_or_execution(self):
        f = self.fixture()
        with self.assertRaisesRegex(ValueError, "actual actuator and execution"):
            audit.feedback_check_mixed(f[1], f[2], f[0], f[3],
                expected_joint_provider_kind=audit.TS20_JOINT_PROVIDER_KIND)
        with self.assertRaisesRegex(ValueError, "explicitly expected"):
            audit.feedback_check_mixed(f[1], f[2], f[0], f[3],
                joint_provider_records=f[4], execution_records=f[5])
        with self.assertRaisesRegex(ValueError, "Unreviewed"):
            audit.feedback_check_mixed(f[1], f[2], f[0], f[3], expected_joint_provider_kind="self_claim",
                joint_provider_records=f[4], execution_records=f[5])
        f[5].pop()
        with self.assertRaisesRegex(ValueError, "one successful execution"):
            self.check(f)

    def test_forged_consistent_metadata_state_and_execution_still_fail_source(self):
        f = self.fixture()
        f[1][1]["q1"] = .5
        f[2][1]["feedback"]["control_state"]["q_rad"][0] = .5
        f[5][1]["state"]["q"][0] = .5
        f[5][1]["feedback_snapshot"] = deepcopy(f[2][1]["feedback"])
        with self.assertRaisesRegex(ValueError, "actual physical-q/endpoint-dq"):
            self.check(f)

    def test_future_stale_generation_and_observation_contract_reject(self):
        for fault in ("future", "stale", "generation", "projection", "raw_control", "limit"):
            f = self.fixture()
            diagnostic = f[2][1]["feedback"]
            observation = diagnostic["joint_observation"]
            if fault in ("future", "stale"):
                stamp = float(f[1][1]["sim_s"])+(.004 if fault == "future" else -.026)
                observation["source_stamp_s"] = diagnostic["joint_stamp_s"] = f[1][1]["joint_stamp_s"] = stamp
                diagnostic["joint_source_age_s"] = float(f[1][1]["sim_s"])-stamp
            elif fault == "generation": observation["generation"] = 7
            elif fault == "projection": observation["projection_applied"] = True
            elif fault == "raw_control": diagnostic["raw_ros_joint_diagnostic"]["used_for_control"] = True
            else: observation["source_age_limit_s"] = .25
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                self.check(f)

    def test_execution_snapshot_clock_and_values_must_match_same_round(self):
        for fault in ("clock", "token", "q", "stamp", "body"):
            f = self.fixture()
            execution = f[5][1]
            if fault == "clock": execution["compute_before"]["processed_estimator_clock_s"] += .004
            elif fault == "token": execution["feedback_snapshot"]["joint_observation"]["generation"] += 1
            elif fault == "q": execution["state"]["q"][0] += .001
            elif fault == "stamp": execution["state"]["joint_stamp_s"] += .004
            else: execution["state"]["position"][0] += .1
            with self.subTest(fault=fault), self.assertRaisesRegex(ValueError, "Execution"):
                self.check(f)

    def test_px4_frame_receipt_reset_and_acceleration_guards_are_not_bypassed(self):
        for fault in ("frame", "receipt", "reset", "acceleration"):
            f = self.fixture()
            diagnostic = f[2][1]["feedback"]
            if fault == "frame": diagnostic["control_state"]["position_enu_m"][0] += 1.
            elif fault == "receipt": diagnostic["callbacks"]["position"]["sim_age_s"] = .101
            elif fault == "reset": diagnostic["px4_snapshot"]["extra_reset_counters"]["quat_reset_counter"] = 1
            else: diagnostic["control_state"]["acceleration_enu_m_s2"][0] += 1.
            f[5][1]["feedback_snapshot"] = deepcopy(diagnostic)
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                self.check(f)

    def test_reference_uses_selected_history_without_postflight_late_rows(self):
        f = self.fixture()
        f[6][1]["diagnostics"]["ts20_reference_state_rad"][0] += .00001
        with self.assertRaisesRegex(ValueError, "reference does not use"):
            ts20_audit.causal_reference_binding(f[4], f[2], f[6], f[3])
        f = self.fixture()
        # Such a past row can be in the final file yet absent at selection.
        # It must never revise the original frozen reference history.
        selected_stamp = f[2][1]["feedback"]["joint_observation"]["source_stamp_s"]
        row = next(row for row in f[4] if selected_stamp < row["sim_s"] <= f[1][1]["sim_s"])
        row["upper_sequence1"], row["receipt1_s"], row["u1"] = 2., row["sim_s"], .1
        row["r1"] = 99.
        report, _ = ts20_audit.causal_reference_binding(f[4], f[2], f[6], f[3])
        self.assertTrue(report["verified"])
        self.assertFalse(report["postflight_later_rows_used_to_revise_history"])

    def test_selected_reference_requires_actual_legal_consumption_declarations(self):
        for fault in ("sequence", "future_receipt"):
            f = self.fixture()
            stamp = f[2][1]["feedback"]["joint_observation"]["source_stamp_s"]
            row = min(f[4], key=lambda row: abs(row["sim_s"]-stamp))
            if fault == "sequence": row["upper_sequence1"] = 1.5
            else: row["receipt1_s"] = stamp+.004
            with self.subTest(fault=fault), self.assertRaisesRegex(ValueError, "consumed-command evidence"):
                ts20_audit.causal_reference_binding(f[4], f[2], f[6], f[3])

    def test_file_hash_admission_precedes_provider_metadata_acceptance(self):
        parent = (ROOT.parent / "tmp").resolve()
        parent.mkdir(exist_ok=True)
        folder = parent / ("causal_audit_test_"+uuid4().hex)
        folder.mkdir()
        def cleanup():
            folder.resolve().relative_to(parent)
            shutil.rmtree(folder)
        self.addCleanup(cleanup)
        reference = self.fixture()[0]
        document = {"kind": "x650_px4_estimated_feedback", "schema_version": 2,
            "profile": audit.ACCELERATION_PROFILE, "feedback_contract": audit.ACCELERATION_CONTRACT,
            "frame_mode": "raw_ev_world", "truth_feedback": False, "truth_clock": False,
            "reference": reference,
            "max_age_s": {"state": .10, "flags": 1.25, "joint": .25, "wall": 4., "flags_wall": 10.},
            "joint_input_source_kind": audit.TS20_JOINT_PROVIDER_KIND}
        (folder / "estimated_feedback.json").write_text(json.dumps(document))
        captures = {}
        for name in ("ts20_state.csv", "ts20_packets.csv"):
            path = folder / name
            path.write_bytes(b"actual original fixture\n")
            captures[name] = {"sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        (folder / "ts20_recording_manifest.json").write_text(json.dumps({"complete": True, "captures": captures}))
        (folder / "ts20_state.csv").write_bytes(b"forged source changed after recording\n")
        with self.assertRaisesRegex(ValueError, "source bytes changed"):
            ts20_audit.causal_feedback_check(folder, {"estimated_feedback": document}, [], reference,
                                            [10., 40.], None, [])

    def test_complete_file_contract_calls_actual_feedback_acceleration_packet_and_reference_checks(self):
        f = self.fixture()
        reference, controls, feedback, window, states, execution, solves = f
        parent = (ROOT.parent / "tmp").resolve()
        parent.mkdir(exist_ok=True)
        folder = parent / ("causal_audit_test_"+uuid4().hex)
        folder.mkdir()
        def cleanup():
            folder.resolve().relative_to(parent)
            shutil.rmtree(folder)
        self.addCleanup(cleanup)
        packets = []
        for index, send in enumerate(np.arange(9.98, 10.055, .005)):
            packet = {"sequence": index+1, "send_s": float(send), "apply_s": float(send+.003346379092576424)}
            for j in (1, 2):
                packet[f"qref{j}"], packet[f"dqref{j}"] = .01*j, .02
            packets.append(packet)
        for state in states:
            for prefix, clock_key in (("last", "send_s"), ("active", "apply_s")):
                packet = [packet for packet in packets if packet[clock_key] <= state["sim_s"]][-1]
                state["last_send_sequence" if prefix == "last" else "active_sequence"] = packet["sequence"]
                state["last_send_s" if prefix == "last" else "active_send_s"] = packet["send_s"]
                state["last_send_apply_s" if prefix == "last" else "active_apply_s"] = packet["apply_s"]
                for j in (1, 2):
                    state[f"{prefix}_qref{j}" if prefix == "active" else f"last_qref{j}"] = packet[f"qref{j}"]
                    state[f"{prefix}_dqref{j}" if prefix == "active" else f"last_dqref{j}"] = packet[f"dqref{j}"]
        def write_rows(name, records):
            with (folder / name).open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(records[0]))
                writer.writeheader()
                writer.writerows(records)
        write_rows("ts20_state.csv", states)
        write_rows("ts20_packets.csv", packets)
        query = np.round(np.arange(window[0], window[1]+.0001, .004), 9)
        acceleration = self.native_acceleration
        native = [{"sim_s": float(t), "valid_accel": 1,
                   **{f"base_a{axis}": float(np.interp(t, acceleration["t"], acceleration["acceleration"][:, i]))
                      for i, axis in enumerate("xyz")}} for t in query]
        write_rows("native_trace.csv", native)
        for name, records in (("feedback_trace.jsonl", feedback), ("control_execution_trace.jsonl", execution)):
            (folder / name).write_text("".join(json.dumps(record)+"\n" for record in records))
        def digest(name):
            return hashlib.sha256((folder / name).read_bytes()).hexdigest()
        captures = {name: {"sha256": digest(name)} for name in ("ts20_state.csv", "ts20_packets.csv")}
        (folder / "ts20_recording_manifest.json").write_text(json.dumps({"complete": True, "captures": captures}))
        (folder / "control_execution_trace_manifest.json").write_text(json.dumps({"recording_closed": True,
            "experiment_complete": True, "recording_error": None, "failure_count": 0,
            "sha256": digest("control_execution_trace.jsonl")}))
        document = {"kind": "x650_px4_estimated_feedback", "schema_version": 2,
            "profile": audit.ACCELERATION_PROFILE, "feedback_contract": audit.ACCELERATION_CONTRACT,
            "frame_mode": "raw_ev_world", "truth_feedback": False, "truth_clock": False,
            "reference": reference,
            "max_age_s": {"state": .10, "flags": 1.25, "joint": .25, "wall": 4., "flags_wall": 10.},
            "extra_reset_counters_at_bind": {"vxy_reset_counter": 0, "vz_reset_counter": 0, "quat_reset_counter": 0},
            "joint_input_source_kind": audit.TS20_JOINT_PROVIDER_KIND}
        (folder / "estimated_feedback.json").write_text(json.dumps(document))
        # The source/reset/acceleration checks run on real synthetic ULog and
        # files. Only unrelated full rigid-body CSV parsing is isolated here.
        with patch.object(ts20_audit.precision, "native_states", return_value={"t": query}):
            report = ts20_audit.causal_feedback_check(folder, {"estimated_feedback": document}, controls,
                                                    reference, window, self.ulog, solves)
        self.assertTrue(report["feedback"]["joint_provider"]["verified"])
        self.assertTrue(report["acceleration"]["verified"])
        self.assertTrue(report["selected_source_packet_binding"]["verified"])
        self.assertTrue(report["reference_state_binding"]["verified"])


if __name__ == "__main__":
    unittest.main()
