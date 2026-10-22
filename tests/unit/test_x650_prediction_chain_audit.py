"""Synthetic trace/forecast contracts, never flight or identification evidence."""
from copy import deepcopy
from pathlib import Path
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
for path in (ROOT/"scripts", ROOT/"simulations/x650_wholebody_mpc_v1", ROOT/"control_packages/x650_wholebody_v1",
             ROOT/"control_packages/x650_wholebody_v1/tests"):
    sys.path.insert(0, str(path))
import audit_x650_prediction_chain as audit
from execution_trace import ExecutionTrace, evidence_value
from mixed_mpc_protocol import build_prediction_chain_protocol
from x650_wholebody.api import ControlState, EndpointTarget
from x650_wholebody import prediction_chain as core_chain
from x650_wholebody.mpc import mixed_response_matrices
from test_model import example_model
from test_mixed_mpc import RESPONSE


def save(path, value):
    Path(path).write_text(json.dumps(evidence_value(value), allow_nan=False)+"\n", encoding="utf-8")


def save_lines(path, records):
    Path(path).write_text("".join(json.dumps(evidence_value(value), allow_nan=False)+"\n" for value in records), encoding="utf-8")


class PredictionChainAuditTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.folder = Path(temporary.name)
        self.protocol = build_prediction_chain_protocol()
        self.model = example_model(elbow_offset_body_m=(0., .04, 0.))
        self.control, self.feedback, self.diagnostics, history = [], [], [], []
        self.metadata = {"task": {"controller": "mpc"}, "physical_profile": self.model.metadata(),
                         "controller_details": {"allocator": {"response_model": RESPONSE}}}
        trace = ExecutionTrace(self.folder)
        self.addCleanup(trace.close)
        self.settings = self.protocol["mpc_controller_config"]["prediction_chain"]
        for i, now in enumerate((9.98, 10., 10.02, 10.04)):
            phase, t = ("settle", -1.) if i < 2 else ("measure", now-10.)
            stamp = now-.01
            state = ControlState([.1, -.1, 1.], Rotation.from_euler("xyz", [.01, -.02, .03]).as_matrix(),
                [.03, -.02, .01], [.01, .02, -.01], [.001, -.002], [.03, -.02], now,
                acceleration_enu=[.02, -.03, .04], control_time_s=now, joint_stamp_s=stamp)
            fb = {"state_stamp_s": now, "raw_marker": {"sequence": i}, "control_state": {
                "position_enu_m": state.position, "velocity_enu_m_s": state.velocity,
                "rotation_flu_to_enu": state.rotation, "omega_flu_rad_s": state.omega_body,
                "q_rad": state.q, "dq_rad_s": state.dq, "acceleration_enu_m_s2": state.acceleration_enu}}
            target = EndpointTarget([.1, .2, .3])
            clock = lambda now=now, stamp=stamp: {"wall_monotonic_s": __import__("time").monotonic(),
                "processed_estimator_clock_s": now, "joint_header_stamp_s": stamp}
            u = np.array([.004, -.003, .002, .03, -.02])
            row = {"sim_s": now, "t_s": t, "phase": phase, "control_dt_s": .02,
                "joint_stamp_s": stamp, "qd1_cmd": u[3], "qd2_cmd": u[4], "pva_sent": True,
                "sent_coordinate_frame": 1, "sent_type_mask": 2048, "sent_yaw_ned": .1,
                **{"base_velocity_target_"+a: u[j] for j, a in enumerate("xyz")}}
            payload = {"coordinate_frame": 1, "type_mask": 2048, "yaw": .1, "yaw_rate": 0.}
            for name in ("position", "velocity", "acceleration"):
                payload[name] = [0., 0., 0.]
                row.update({"sent_"+name+"_target_"+a: 0. for a in "ned"})
            if phase == "measure":
                trace.controller_step(SimpleNamespace(step=lambda *args: None), t, .02, state, target,
                    feedback=fb, phase=phase, clock_snapshot=clock)
                intervals = np.r_[.02, np.full(14, .04)]
                plan = np.tile(u, (15, 1))
                plan[:, :3] += np.arange(15)[:, None]*[.001, -.002, .0003]
                plan[:, 3:] += np.arange(15)[:, None]*[.001, .0005]
                predicted = []
                x = np.r_[state.position, state.velocity, state.q, state.dq, state.acceleration_enu[:2]]
                for dt, command in zip(intervals, plan):
                    A, B = mixed_response_matrices(dt, RESPONSE)
                    x = A@x+B@command
                    predicted.append(x.copy())
                predicted = np.asarray(predicted)
                joint = core_chain.joint_prediction(t, np.cumsum(intervals), intervals, state,
                    [(when-10., command[3:]) for when, command in history], self.settings, [.45, .35])
                z = (plan/np.array([.55, .35, .45, .45, .35])).reshape(-1)
                for j, (_, q, dq, Q, D) in enumerate(joint["nodes"]):
                    predicted[j, 6:8], predicted[j, 8:10] = q+Q@z, dq+D@z
                context = core_chain.attitude_context(state, np.cumsum(intervals), self.settings)
                output = core_chain.endpoint_outputs(predicted, plan, context, self.model, RESPONSE["parameters"])
                d = {"allocation": {"tip_ref": target.position}, "node_intervals_s": intervals,
                    "predicted_commands": plan, "predicted_states": predicted,
                    "prediction_initial_joint_position": joint["q_current"],
                    "prediction_initial_joint_velocity": joint["dq_current"],
                    "predicted_rotations_flu_to_enu": output["rotations"], "predicted_tip_positions": output["position"],
                    "prediction_attitude_initial_residual_rotvec": context["initial_residual_rotvec"]}
                self.diagnostics.append({"phase": phase, "sim_s": now, "t_s": t, "diagnostics": evidence_value(d)})
                self.feedback.append({"phase": phase, "sim_s": now, "t_s": t, "feedback": evidence_value(fb)})
            context = {"sim_s": now, "t_s": t, "phase": phase, "control_dt_s": .02, "measured_joint_stamp_s": stamp}
            for j in (1, 2):
                trace.publish("joint:arm_joint_"+str(j), {"joint": "arm_joint_"+str(j), "velocity_rad_s": u[j+2]},
                    lambda: None, clock_snapshot=clock, context=context)
            trace.publish("px4_pva", payload, lambda: None, clock_snapshot=clock, context=context)
            self.control.append(row)
            history.append((now, u.copy()))
        self.metadata["control_execution_trace"] = trace.close(experiment_complete=True)
        self.metadata["activation_command_history"] = [{"control_time_s": when, "task_time_s": when-10., "command": value.tolist()}
            for when, value in history[:2]]
        save(self.folder/"controller_metadata.json", self.metadata)
        save(self.folder/"control_trace.csv", self.control)  # hash-only here; caller already parsed the actual CSV
        self.persist()

    def persist(self):
        save_lines(self.folder/"controller_diagnostics.jsonl", self.diagnostics)
        save_lines(self.folder/"feedback_trace.jsonl", self.feedback)

    def run_check(self):
        with patch("x650_wholebody.mpc.minimize", side_effect=AssertionError("Audit must not solve")), \
             patch("x650_wholebody.prediction_chain.joint_prediction", side_effect=AssertionError("Audit must be independent")):
            return audit.audit_prediction_chain_execution(self.folder, self.control, self.metadata, self.protocol, [10., 10.04])

    def test_independent_base_joint_attitude_and_endpoint_replay_matches_core_outputs(self):
        result = self.run_check()
        self.assertTrue(result["verified"])
        self.assertTrue(result["prediction_replayed"])
        self.assertFalse(result["optimizer_executed"])
        self.assertLess(max(result["maximum_replay_errors"].values()), 1e-12)
        self.assertEqual(result["samples"], 2)

    def test_finite_but_wrong_initial_joint_node_attitude_and_tip_forecasts_fail(self):
        original = deepcopy(self.diagnostics)
        paths = [("prediction_initial_joint_position", (0,)), ("prediction_initial_joint_velocity", (1,)),
            ("predicted_states", (3, 6)), ("predicted_states", (3, 8)), ("predicted_states", (3, 11)),
            ("predicted_rotations_flu_to_enu", (2, 1, 2)), ("predicted_tip_positions", (2, 0))]
        for field, indices in paths:
            self.diagnostics = deepcopy(original)
            value = self.diagnostics[0]["diagnostics"][field]
            for index in indices[:-1]:
                value = value[index]
            value[indices[-1]] += .001
            self.persist()
            with self.subTest(field=field, indices=indices), self.assertRaisesRegex(ValueError, "mismatch"):
                self.run_check()

    def test_actual_publish_and_activation_or_consumed_feedback_mismatch_fails(self):
        original = deepcopy(self.control)
        self.control[2]["qd1_cmd"] += .01
        with self.assertRaisesRegex(ValueError, "joint publication"):
            self.run_check()
        self.control = original
        self.metadata["activation_command_history"][-1]["command"][3] += .01
        with self.assertRaisesRegex(ValueError, "activation command"):
            self.run_check()
        self.metadata["activation_command_history"][-1]["command"][3] -= .01
        self.feedback[0]["feedback"]["raw_marker"]["sequence"] = 99
        self.persist()
        with self.assertRaisesRegex(ValueError, "raw feedback"):
            self.run_check()

    def test_trace_sha_closure_and_exception_are_not_optional(self):
        path = self.folder/"control_execution_trace.jsonl"
        original = path.read_bytes()
        path.write_bytes(original+b"\n")
        with self.assertRaisesRegex(ValueError, "hash-bound"):
            self.run_check()
        path.write_bytes(original)
        manifest = deepcopy(self.metadata["control_execution_trace"])
        manifest["recording_closed"] = False
        save(self.folder/"control_execution_trace_manifest.json", manifest)
        self.metadata["control_execution_trace"] = manifest
        with self.assertRaisesRegex(ValueError, "incomplete"):
            self.run_check()
        manifest["recording_closed"] = True
        records = audit.lines(path)
        next(e for e in records if e["event"] == "controller_step_returned")["event"] = "controller_step_exception"
        save_lines(path, records)
        manifest["sha256"] = audit.sha(path)
        save(self.folder/"control_execution_trace_manifest.json", manifest)
        self.metadata["control_execution_trace"] = manifest
        with self.assertRaisesRegex(ValueError, "Unexpected/missing"):
            self.run_check()

    def test_pid_has_same_publication_history_checks_without_fake_mpc_forecast(self):
        self.metadata["task"]["controller"] = "pid_ff"
        for row in self.diagnostics:
            row["diagnostics"] = {"allocation": row["diagnostics"]["allocation"]}
        self.persist()
        result = self.run_check()
        self.assertTrue(result["verified"])
        self.assertFalse(result["prediction_replayed"])


if __name__ == "__main__":
    unittest.main()
