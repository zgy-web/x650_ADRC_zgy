"""Live-only diagnostic ownership, exact records and unchanged public copies.

The allocator deliberately reuses arrays/dicts to exercise the historical
isolation boundary. No ROS, numerical backend or physical process is started.
"""
from pathlib import Path
import json
import pickle
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(ROOT / "control_packages/x650_wholebody_v1"))
sys.path.insert(0, str(ROOT / "control_packages/x650_wholebody_v1/tests"))
from flyarm_mpc_reference import live_factory
from flyarm_mpc_reference.diagnostic_channel import CaptureAdapter, DiagnosticSender, HEADER, MAGIC
from x650_wholebody.adapter import ControllerAdapter
from x650_wholebody.api import ControlState, EndpointTarget
from test_model import example_model


def record_value(value):
    """Use the recorder's ndarray/scalar rules, including duplicate paths."""
    def default(item):
        if isinstance(item, np.ndarray):
            return item.tolist()
        if isinstance(item, np.generic):
            return item.item()
        raise TypeError(type(item).__name__)
    return json.dumps(value, default=default, sort_keys=True, allow_nan=False)


def assert_acyclic(test, value, ancestors=None):
    if not isinstance(value, (dict, list, tuple)):
        return
    ancestors = set() if ancestors is None else ancestors
    test.assertNotIn(id(value), ancestors, "diagnostic graph contains a cycle")
    path = ancestors | {id(value)}
    for child in value.values() if isinstance(value, dict) else value:
        assert_acyclic(test, child, path)


class ReusingAllocator:
    def __init__(self, model):
        self.model = model
        self.prediction = np.arange(150., dtype=float).reshape(15, 10)
        self.envelope = [{"time_s": i * .005, "q": np.array([i * .001, -.02]),
                          "lower": [-.2, -.3], "passed": True} for i in range(80)]
        self.allocation = dict(p_ref=np.array([.1, -.2, 1.9]),
            base_velocity_cmd=np.array([.01, -.03, .02]), qd_cmd=np.array([.02, -.01]),
            qdd_ref=np.array([.3, -.2]), a_ref=np.zeros(3),
            diagnostics=dict(accepted_optimum=True, previous_plan_fallback=False,
                ts20_nonlinear_check_passed=True, primal_violation=0.,
                kkt_stationarity_inf=1e-16, predicted_states=self.prediction,
                ts20_candidate_envelope=self.envelope,
                ts20_relinearization_attempts=[{"subproblem": 1, "passed": True}]))

    def reset(self, *args):
        pass

    def step(self, t, dt, state, target):
        self.prediction[0, 0] = t
        self.envelope[0]["time_s"] = t
        self.envelope[0]["q"][0] = t
        return self.allocation


def make_adapter(owned):
    model = example_model()
    allocator = ReusingAllocator(model)
    adapter = ControllerAdapter(allocator, model, {"MPC_XY_P": .95, "MPC_Z_P": 1.},
                                acceleration_feedforward_policy="zero")
    adapter.reset([0., 0., 2.], np.eye(3), [0., 0.])
    if owned:
        adapter.use_owned_diagnostic_snapshot()
    return adapter, allocator


def measured(i):
    return ControlState([0., 0., 2.], np.eye(3), [.01, .02, -.01],
                        [.01 * i, -.02 * i, 0.], [.02, -.01], [.1, -.1], 10. + i * .02)


def step(adapter, i):
    return adapter.step(i * .02, .02, measured(i), EndpointTarget([0., 0., 1.6]))


class LiveDiagnosticSnapshotTests(unittest.TestCase):
    def test_all_command_and_record_fields_exactly_match_public_adapter(self):
        original, _ = make_adapter(False)
        optimized, _ = make_adapter(True)
        for i in range(4):
            a, b = step(original, i), step(optimized, i)
            for name in ("position_reference", "velocity_feedforward", "acceleration_feedforward",
                         "base_velocity_target", "joint_velocity", "_feedback_position", "_feedback_velocity"):
                np.testing.assert_array_equal(getattr(a, name), getattr(b, name))
                self.assertFalse(getattr(b, name).flags.writeable)
            self.assertEqual(record_value(a.diagnostics), record_value(b.diagnostics))
            assert_acyclic(self, b.diagnostics)
            self.assertEqual(record_value(a.diagnostics),
                             record_value(pickle.loads(pickle.dumps(b.diagnostics, protocol=5))))

    def test_public_default_keeps_independent_nested_mutation_behavior(self):
        adapter, allocator = make_adapter(False)
        d = step(adapter, 0).diagnostics
        original_nested = record_value(d["allocation"]["diagnostics"])
        original_allocator = record_value(allocator.allocation)
        d["predicted_states"][0, 0] = 123.
        d["ts20_candidate_envelope"][0]["q"][0] = 321.
        self.assertEqual(record_value(d["allocation"]["diagnostics"]), original_nested)
        self.assertEqual(record_value(allocator.allocation), original_allocator)

    def test_live_result_owns_graph_without_self_cycle_or_controller_alias(self):
        adapter, allocator = make_adapter(True)
        d = step(adapter, 0).diagnostics
        nested = d["allocation"]["diagnostics"]
        self.assertIsNot(d, nested)
        self.assertIs(d["predicted_states"], nested["predicted_states"])
        self.assertFalse(np.shares_memory(d["predicted_states"], allocator.prediction))
        self.assertFalse(np.shares_memory(d["ts20_candidate_envelope"][0]["q"], allocator.envelope[0]["q"]))
        self.assertIsNot(d["ts20_candidate_envelope"], allocator.envelope)
        assert_acyclic(self, d)

    def test_later_allocator_steps_and_mutations_cannot_change_old_records(self):
        adapter, allocator = make_adapter(True)
        first = step(adapter, 0).diagnostics
        saved = record_value(first)
        second = step(adapter, 1).diagnostics
        allocator.prediction[:] = -999.
        allocator.envelope[0]["lower"][0] = -777.
        allocator.envelope[0]["q"][:] = 888.
        allocator.allocation["diagnostics"]["new_key"] = "late allocator mutation"
        second["predicted_states"][:] = -555.
        second["ts20_candidate_envelope"][0]["q"][:] = 666.
        self.assertEqual(record_value(first), saved)
        self.assertNotIn("new_key", first)

    def test_modifying_an_owned_record_cannot_change_allocator_or_later_result(self):
        adapter, allocator = make_adapter(True)
        first = step(adapter, 0).diagnostics
        allocator_before = record_value(allocator.allocation)
        first["predicted_states"][:] = -1234.
        first["ts20_candidate_envelope"][0]["lower"][0] = -4321.
        first["allocation"]["p_ref"][:] = 999.
        self.assertEqual(record_value(allocator.allocation), allocator_before)
        second = step(adapter, 1).diagnostics
        self.assertEqual(second["predicted_states"][0, 1], 1.)
        self.assertEqual(second["ts20_candidate_envelope"][0]["lower"][0], -.2)
        np.testing.assert_array_equal(second["allocation"]["p_ref"], [.1, -.2, 1.9])

    def test_real_diagnostic_sender_detaches_bytes_before_next_step(self):
        adapter, allocator = make_adapter(True)
        capture = CaptureAdapter(adapter)
        capture.step(0., .02, measured(0), EndpointTarget([0., 0., 1.6]))
        saved = record_value(capture.diagnostic)
        payloads = []
        fake_socket = Mock()
        fake_socket.send.side_effect = lambda packet: payloads.append(packet) or len(packet)
        with patch("flyarm_mpc_reference.diagnostic_channel.socket.socket", return_value=fake_socket), \
             patch("flyarm_mpc_reference.diagnostic_channel.socket.AF_UNIX", 1, create=True):
            sender = DiagnosticSender("unused-private-test-socket")
        self.assertTrue(sender.offer(1, {"diagnostic": capture.diagnostic}))
        capture.step(.02, .02, measured(1), EndpointTarget([0., 0., 1.6]))
        allocator.prediction[:] = 999.
        capture.diagnostic["predicted_states"][:] = -999.
        magic, sequence = HEADER.unpack(payloads[0][:HEADER.size])
        self.assertEqual((magic, sequence), (MAGIC, 1))
        restored = pickle.loads(payloads[0][HEADER.size:])
        self.assertEqual(record_value(restored["diagnostic"]), saved)
        self.assertEqual(sender.sent, 1)
        self.assertEqual(sender.dropped, 0)

    def test_selecting_ownership_after_step_is_rejected(self):
        adapter, _ = make_adapter(False)
        step(adapter, 0)
        with self.assertRaisesRegex(RuntimeError, "before the first step"):
            adapter.use_owned_diagnostic_snapshot()

    def test_private_path_does_not_accept_invalid_list_of_diagnostic_pairs(self):
        for owned in (False, True):
            adapter, allocator = make_adapter(owned)
            allocator.allocation["diagnostics"] = list(allocator.allocation["diagnostics"].items())
            with self.assertRaises((AttributeError, TypeError)):
                step(adapter, 0)
            self.assertIsNone(adapter._last_task_t)

    def test_hold_factory_selects_private_policy_before_live_handoff(self):
        adapter, _ = make_adapter(False)
        engine = NS(adapter=adapter, requests=0, live_started=False, _metadata={})
        initialization = dict(deferred_actual_handoff=True, simulation_live=True,
                              synthetic_software_fixture_only=True)
        with patch.object(live_factory, "_pinned_json", side_effect=[({}, {}), (initialization, {})]), \
             patch.object(live_factory, "create_engine", return_value=engine):
            result = live_factory.create_live_engine("root", "task", "hash", "init", "hash", 7, 1)
        self.assertIs(result, engine)
        self.assertTrue(adapter._owned_diagnostic_snapshot)
        self.assertEqual(engine._metadata["diagnostic_snapshot"], "owned_allocation_once_v1")

    def test_figure8_factory_early_return_also_selects_private_policy(self):
        adapter, _ = make_adapter(False)
        engine = NS(adapter=adapter, requests=0, live_started=False, _metadata={})
        with patch.object(live_factory, "_pinned_json", return_value=({"scenario": "x650_isolated_figure8_v1"}, {})), \
             patch("flyarm_mpc_reference.figure8_factory.create_figure8_engine", return_value=engine):
            result = live_factory.create_live_engine("root", "task", "hash", "init", "hash", 7, 1)
        self.assertIs(result, engine)
        self.assertTrue(adapter._owned_diagnostic_snapshot)
        self.assertEqual(engine._metadata["diagnostic_snapshot"], "owned_allocation_once_v1")

    def test_live_ownership_cannot_be_reconfigured_after_handoff_or_solve(self):
        for requests, started in ((1, False), (0, True)):
            adapter, _ = make_adapter(False)
            engine = NS(adapter=adapter, requests=requests, live_started=started, _metadata={})
            with self.assertRaisesRegex(ValueError, "before handoff"):
                live_factory._own_live_diagnostics(engine)
            self.assertFalse(adapter._owned_diagnostic_snapshot)


if __name__ == "__main__":
    unittest.main()
