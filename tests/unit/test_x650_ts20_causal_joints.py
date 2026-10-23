"""Explicit causal encoder handoff; pure synthetic streams, no flight/ROS I/O."""
from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for p in (ROOT, ROOT/"tests/unit", ROOT/"control_packages/x650_wholebody_v1"):
    sys.path.insert(0, str(p))
from test_x650_estimated_feedback import fixture
from simulations.x650_wholebody_mpc_v1.estimated_feedback import Px4EstimatedFeedback
from simulations.x650_wholebody_mpc_v1 import run_case as shared
from simulations.x650_ts20_hold_v1.run_case import Ts20Extension
from x650_wholebody.api import ControlState
from x650_wholebody.ts20_runtime import Ts20Runtime


class Tail:
    def __init__(self, rows):
        self.rows, self.calls, self.path = rows, 0, Path("synthetic-not-a-file")
    def read(self):
        self.calls += 1
        rows, self.rows = self.rows, []
        return rows
    def close(self):
        pass


def source():
    predictor = NS(bus_period_s=.005, delay_s=.003346379092576424)
    packets = []
    for i in range(5):
        packet = dict(send_s=10.+i*.005, apply_s=10.+i*.005+predictor.delay_s, sequence=i)
        for j in (1, 2):
            packet.update({f"qref{j}": 0., f"dqref{j}": 0., f"tau{j}": 0., f"kp{j}": 6., f"kd{j}": .25})
        packets.append(packet)
    rows = []
    for i in range(1, 5):
        at = 10.+i*.004
        row = dict(sim_s=at, interval_begin_s=at-.004, step_dt_s=.004)
        for j in (1, 2):
            row.update({f"q{j}_physical": .02*j, f"q{j}_internal": .02*j,
                f"dq{j}_physical": .123, f"dq{j}_internal": .234,
                f"r{j}": .01*i, f"u{j}": .2 if i == 4 else 0.,
                f"receipt{j}_s": 10.012 if i == 4 else 10., f"upper_sequence{j}": 1 if i == 4 else 0.})
        for prefix, index in (("last_", int(np.floor((at-10.)/.005+1e-8))),
                              ("active_", int(np.floor((at-10.-predictor.delay_s)/.005+1e-8)))):
            packet = packets[max(0, index)]
            row["last_send_sequence" if prefix == "last_" else "active_sequence"] = max(0, index)
            row["last_send_s" if prefix == "last_" else "active_send_s"] = packet["send_s"]
            row["last_send_apply_s" if prefix == "last_" else "active_apply_s"] = packet["apply_s"]
            for j in (1, 2):
                row[f"{prefix}qref{j}"] = row[f"{prefix}dqref{j}"] = 0.
        rows.append(row)
    runtime = Ts20Runtime("not-read-s.csv", "not-read-p.csv", predictor)
    runtime.states, runtime.packets = Tail(rows), Tail(packets)
    return runtime


def state_for(observation):
    return ControlState([2., 3., 1.5], np.eye(3), [0.]*3, [0.]*3, observation.q,
        observation.dq, 10., control_time_s=observation.control_time_s,
        joint_stamp_s=observation.source_stamp_s)


def feedback(runtime, *, declared=True):
    node, reference = fixture()
    node.estimator_clock_s = 10.012
    node.data["position"].ax, node.data["position"].ay, node.data["position"].az = 1., 2., -3.
    kwargs = {"joint_source_kind": "ts20_physical_q_endpoint_dq_v1"} if declared else {}
    adapter = Px4EstimatedFeedback(reference, node, require_acceleration=True,
                                    require_prediction_timing=True, **kwargs)
    return node, adapter


class CausalJointTests(unittest.TestCase):
    def test_selected_object_binds_once_without_csv_read_or_projection(self):
        runtime = source()
        observation = runtime.select_joint_observation(10.012)
        before = (runtime.states.calls, runtime.packets.calls)
        state, stamp = runtime.measured_state(state_for(observation), observation=observation)
        context = runtime.context(state, .012)
        self.assertEqual((runtime.states.calls, runtime.packets.calls), before)
        self.assertEqual(stamp, 10.012)
        np.testing.assert_array_equal(state.q, [.02, .04])
        np.testing.assert_array_equal(state.dq, [.234, .234])
        np.testing.assert_array_equal(context["reference_state"]["r"], [.03, .03])
        with self.assertRaises(FrozenInstanceError):
            observation.source_stamp_s = 99.

    def test_rows_and_commands_are_immutable_and_new_cache_data_cannot_change_snapshot(self):
        runtime = source()
        observation = runtime.select_joint_observation(10.012)
        state, _ = runtime.measured_state(state_for(observation), observation=observation)
        old = runtime.context(state, .012)
        with self.assertRaises(TypeError):
            runtime._rows[0]["r1"] = 99.
        with self.assertRaises(TypeError):
            runtime._command_events[0][1][0] = 99.
        runtime._packets.clear()
        runtime._command_events.clear()
        newer = runtime.context(state, .012)
        np.testing.assert_array_equal(old["reference_state"]["r"], newer["reference_state"]["r"])
        self.assertEqual(len(old["packet_history"]), len(newer["packet_history"]))
        self.assertTrue(all(np.all(u == 0.) for _, u in newer["command_history"]))

    def test_equal_forged_old_or_other_runtime_tokens_cannot_bind(self):
        runtime = source()
        original = runtime.select_joint_observation(10.012)
        for wrong in (replace(original), source().select_joint_observation(10.012)):
            with self.assertRaisesRegex(RuntimeError, "token"):
                runtime.measured_state(state_for(wrong), observation=wrong)
        current = runtime.select_joint_observation(10.016)
        with self.assertRaisesRegex(RuntimeError, "token"):
            runtime.measured_state(state_for(original), observation=original)
        runtime.measured_state(state_for(current), observation=current)
        with self.assertRaisesRegex(RuntimeError, "differs"):
            runtime.measured_state(replace(state_for(current), q=[9., 9.]), observation=current)

    def test_true_future_stale_regressing_and_inconsistent_source_still_refuse(self):
        for kind in ("future", "stale", "mismatch", "sequence", "packet"):
            runtime = source()
            if kind == "future": runtime.states.rows = runtime.states.rows[-1:]
            if kind == "mismatch": runtime.states.rows[2]["q1_internal"] = 3.
            if kind == "sequence": runtime.states.rows[1]["upper_sequence1"] = -1.
            if kind == "packet": runtime.packets.rows = runtime.packets.rows[:1]
            with self.subTest(kind=kind), self.assertRaises(RuntimeError):
                runtime.select_joint_observation(10.1 if kind == "stale" else 10.012)
            self.assertIsNone(runtime._selected_observation)
        runtime = source()
        runtime.select_joint_observation(10.012)
        with self.assertRaisesRegex(RuntimeError, "clock regressed"):
            runtime.select_joint_observation(10.008)

    def test_provider_future_ros_is_only_diagnostic_body_stays_px4(self):
        runtime = source()
        node, adapter = feedback(runtime)
        original_node = deepcopy(node)
        original_node.data["joints"].header.stamp.nanosec = 12000000
        baseline = Px4EstimatedFeedback(adapter._document, original_node,
            require_acceleration=True, require_prediction_timing=True).read(original_node, 10.012, wall_now=100.)[0]
        node.data["joints"].header.stamp.nanosec = 16000000
        node.received_sim["joints"], node.received["joints"] = 999., -999.
        observation = runtime.select_joint_observation(10.012)
        state, stamp, diagnostic = adapter.read(node, 10.012, wall_now=100., joint_observation=observation)
        runtime.measured_state(state, observation=observation)
        for name in ("position", "rotation", "velocity", "omega_body", "acceleration_enu"):
            np.testing.assert_array_equal(getattr(state, name), getattr(baseline, name))
        np.testing.assert_array_equal(state.q, observation.q)
        self.assertEqual(stamp, observation.source_stamp_s)
        self.assertNotIn("joints", diagnostic["callbacks"])
        self.assertFalse(diagnostic["raw_ros_joint_diagnostic"]["used_for_control"])
        self.assertEqual(diagnostic["raw_ros_joint_diagnostic"]["header_stamp_s"], 10.016)
        self.assertEqual(node.data["joints"].header.stamp.nanosec, 16000000)
        self.assertFalse(diagnostic["joint_observation"]["projection_applied"])

    def test_legacy_future_guard_and_provider_declaration_are_required(self):
        runtime = source()
        node, legacy = feedback(runtime, declared=False)
        node.data["joints"].header.stamp.nanosec = 16000000
        with self.assertRaisesRegex(RuntimeError, "future joint"):
            legacy.read(node, 10.012, wall_now=100.)
        obs = runtime.select_joint_observation(10.012)
        with self.assertRaisesRegex(RuntimeError, "declared source"):
            legacy.read(node, 10.012, wall_now=100., joint_observation=replace(obs, source_kind=None))
        _, declared = feedback(runtime)
        with self.assertRaisesRegex(RuntimeError, "observation is missing"):
            declared.read(node, 10.012, wall_now=100.)

    def test_loop_selects_before_ros_interpretation_and_clock_ignores_raw_joint(self):
        runtime = source()
        node, adapter = feedback(runtime)
        node.data["joints"].header.stamp = NS(sec=99, nanosec=0)
        node.sim_time_s = lambda: 10.012
        node.joint_values = lambda: self.fail("ROS joints cannot supply selected control q/dq")
        xyz = lambda x=0., y=0., z=0.: NS(x=x, y=y, z=z)
        node.data["truth"] = NS(header=NS(stamp=NS(sec=10, nanosec=12000000)),
            pose=NS(pose=NS(position=xyz(99., 98., 97.), orientation=NS(w=1., x=0., y=0., z=0.))),
            twist=NS(twist=NS(linear=xyz(), angular=xyz())))
        extension = Ts20Extension({}, {})
        extension.runtime = runtime
        loop = shared.ControlLoop.__new__(shared.ControlLoop)
        loop.ctx, loop.extension, loop.feedback_adapter = NS(node=node), extension, adapter
        loop.recovery, loop.strict, loop.status_watchdog = False, False, None
        loop.recorder = NS(clock_s=10.012, now=lambda: 99.)
        loop.land_watchdog = NS(observe=lambda *a: None)
        loop.last_truth, loop.last_joint = None, None
        self.assertEqual(loop.control_now(), 10.012)
        self.assertEqual(loop.monitor_now(), 10.012)
        with patch("time.monotonic", return_value=100.):
            now, state, stamp = loop.read()
        self.assertEqual((now, stamp), (10.012, 10.012))
        self.assertEqual((runtime.states.calls, runtime.packets.calls), (1, 1))
        np.testing.assert_array_equal(state.position, [2., 3., 1.5])
        np.testing.assert_array_equal(loop.evaluation_state.position, [99., 98., 97.])
        runtime.context(state, .012)

    def test_recovery_invalidates_prior_token_without_touching_bad_logs(self):
        runtime = source()
        observation = runtime.select_joint_observation(10.012)
        runtime.measured_state(state_for(observation), observation=observation)
        runtime.states.read = lambda: self.fail("Recovery cannot read a failed TS20 source")
        extension = Ts20Extension({}, {})
        extension.runtime = runtime
        physical = object()
        self.assertEqual(extension.after_state_read(NS(recovery=True), physical, 12., 12.), (physical, 12.))
        self.assertIsNone(runtime._selected_observation)
        with self.assertRaisesRegex(RuntimeError, "successful measured snapshot"):
            runtime.context(state_for(observation), .012)

    def test_non_upper_commissioning_keeps_original_clock_without_new_subscription(self):
        loop = shared.ControlLoop.__new__(shared.ControlLoop)
        loop.extension = Ts20Extension({}, {})
        loop.ctx = NS(node=NS(estimator_clock_s=None))
        loop.feedback_adapter, loop.recovery = None, False
        loop.recorder = NS(now=lambda: 12.5)
        self.assertEqual(loop.control_now(), 12.5)
        self.assertEqual(loop.monitor_now(), 12.5)


if __name__ == "__main__":
    unittest.main()
