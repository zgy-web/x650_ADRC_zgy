"""Software-only adapter contracts, using a fake allocator and real state API."""
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import numpy as np

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO/"control_packages/x650_wholebody_v1"))
from flyarm_mpc_reference.engine import ReferenceEngine, SnapshotContext
from flyarm_mpc_reference.factory import _pinned_json
from x650_wholebody.api import ControlState, EndpointTarget


def snapshot():
    pad = lambda values, length: list(values)+[0.]*(length-len(values))
    return NS(schema_version=1, session_id=7, epoch=1, sequence=1, replay_only=True,
        configuration_sha256=list(range(32)), issued_monotonic_ns=999_000_000,
        control_time_s=10.02, task_time_s=.02, dt_s=.02, state_time_s=10.02, joint_time_s=10.02,
        position_enu=[0., 0., 2.], rotation_row_major=np.eye(3).reshape(-1).tolist(),
        velocity_enu=[0.]*3, omega_body_flu=[0.]*3, acceleration_enu=[0.]*3, q=[.1, .2], dq=[.01, -.01],
        target_position_enu=[0., 0., 1.6], target_velocity_enu=[0.]*3, target_acceleration_enu=[0.]*3,
        position_gains=[.95, .95, 1.], reference_time_s=.02, bus_origin_s=0., reference_r=[.1, .2],
        command_count=1, packet_count=2, command_time_s=pad([-.02], 64), command_velocity=pad([0., 0.], 128),
        packet_send_time_s=pad([-.02, -.015], 64), packet_q_ref=pad([.1, .2, .1, .2], 128),
        packet_dq_ref=pad([0.]*4, 128), packet_tau_ff=pad([0.]*4, 128),
        packet_kp=pad([6.]*4, 128), packet_kd=pad([.25]*4, 128))


class FakeAdapter:
    def __init__(self, context):
        self.context, self.calls = context, []
        self.error, self.on_step = None, None
        self.diagnostic = dict(accepted_optimum=True, ts20_nonlinear_check_passed=True,
                              previous_plan_fallback=False, primal_violation=0., kkt_stationarity_inf=.00001,
                              ts20_relinearization_attempts=[{"passed": True}])

    def step(self, t, dt, state, target):
        self.calls.append((t, dt, state, target, self.context(state, t)))
        if self.on_step is not None:
            self.on_step()
        if self.error is not None:
            raise self.error
        return NS(position_reference=[1., 2., 3.], velocity_feedforward=[.01, .02, .03],
                  acceleration_feedforward=[0.]*3, base_velocity_target=[.04, .05, .06],
                  joint_velocity=[.01, -.01], diagnostics=self.diagnostic)


def engine(clock=None):
    context = SnapshotContext()
    adapter = FakeAdapter(context)
    worker = ReferenceEngine(adapter, context, task_origin=10., position_gains=[.95, .95, 1.],
        target_position=[0., 0., 1.6], session_id=7, epoch=1, configuration_sha256=list(range(32)),
        state_factory=ControlState, target_factory=EndpointTarget,
        clock_ns=clock or Mock(side_effect=[1_000_000_000, 1_001_000_000]))
    return worker, adapter


class ReferenceEngineTests(unittest.TestCase):
    def test_prearm_template_is_replaced_before_any_solve(self):
        class Constant:
            def __init__(self, target): self.target=np.asarray(target)
        worker,adapter=engine()
        worker.simulation_live=worker.live_reference_deferred=True
        adapter.reset=Mock();adapter.allocator=NS(prime_command_history=Mock(),trajectory=Constant([9.,9.,9.]))
        start=NS(session_id=7,epoch=1,configuration_sha256=list(range(32)),origin_s=10.,
            history_count=2,history_time_s=[9.98,10.]+[0.]*62,
            history_command=[.01,-.02,.03,.04,-.05]*2+[0.]*310,
            target_position_enu=[0.,0.,1.6],anchor_position_enu=[0.,0.,2.],
            anchor_rotation_row_major=np.eye(3).reshape(-1),anchor_q=[0.,0.])
        worker.begin_live(start)
        self.assertEqual(adapter.calls,[])
        np.testing.assert_array_equal(adapter.allocator.trajectory.target,start.target_position_enu)
        self.assertFalse(worker.live_reference_deferred)
        s=snapshot();s.replay_only=False
        self.assertEqual(worker.process(s,NS).status,0)

    def test_live_handoff_is_explicit_one_shot_and_primes_actual_history(self):
        worker,adapter=engine()
        worker.simulation_live=True
        adapter.reset=Mock();adapter.allocator=NS(prime_command_history=Mock())
        start=NS(session_id=7,epoch=1,configuration_sha256=list(range(32)),origin_s=10.,
            history_count=2,history_time_s=[9.98,10.]+[0.]*62,
            history_command=[.01,-.02,.03,.04,-.05]*2+[0.]*310,
            target_position_enu=[0.,0.,1.6],anchor_position_enu=[0.,0.,2.],
            anchor_rotation_row_major=np.eye(3).reshape(-1),anchor_q=[0.,0.])
        worker.begin_live(start)
        self.assertTrue(worker.live_started)
        adapter.reset.assert_called_once()
        history,origin=adapter.allocator.prime_command_history.call_args.args
        self.assertEqual(origin,10.)
        np.testing.assert_array_equal(history[-1][1],[.01,-.02,.03,.04,-.05])
        with self.assertRaises(ValueError):worker.begin_live(start)
        s=snapshot();s.replay_only=False
        result=worker.process(s,NS)
        self.assertEqual(result.status,0)
        self.assertFalse(result.replay_only)

    def test_live_mode_cannot_solve_before_handoff_or_accept_shadow_input(self):
        for replay in (True,False):
            worker,adapter=engine();worker.simulation_live=True
            s=snapshot();s.replay_only=replay
            self.assertNotEqual(worker.process(s,NS).status,0)
            self.assertEqual(adapter.calls,[])

    def test_valid_snapshot_calls_same_step_once_with_causal_owned_context(self):
        worker, adapter = engine()
        message = snapshot()
        original = deepcopy(message)
        out = worker.process(message, NS)
        self.assertEqual(out.status, 0)
        self.assertEqual((out.sequence, out.snapshot_sequence, out.session_id), (1, 1, 7))
        self.assertEqual(out.configuration_sha256, list(range(32)))
        self.assertTrue(out.accepted_optimum and out.nonlinear_check_passed)
        self.assertEqual(out.position_reference_enu, [1., 2., 3.])
        self.assertEqual(out.joint_velocity, [.01, -.01])
        self.assertEqual(out.subproblems, 1)
        self.assertEqual(len(adapter.calls), 1)
        t, dt, state, target, context = adapter.calls[0]
        self.assertEqual((t, dt, state.control_time_s, state.joint_stamp_s), (.02, .02, 10.02, 10.02))
        np.testing.assert_array_equal(state.q, original.q)
        np.testing.assert_array_equal(state.dq, original.dq)
        np.testing.assert_array_equal(context["reference_state"]["r"], original.reference_r)
        message.packet_q_ref[0] = 99.
        self.assertNotEqual(context["packet_history"][0]["q_ref"][0], 99.)
        self.assertIsNone(worker.context.value)
        self.assertFalse(worker.metadata()["native_mpc"])
        self.assertFalse(worker.metadata()["actuator_dispatch"])

    def test_monotonic_sequential_messages_keep_controller_memory_without_reset(self):
        worker, adapter = engine(clock=Mock(side_effect=[1_000_000_000, 1_001_000_000, 1_002_000_000, 1_003_000_000]))
        self.assertEqual(worker.process(snapshot(), NS).status, 0)
        second = snapshot()
        second.sequence, second.control_time_s, second.task_time_s = 2, 10.04, .04
        second.state_time_s = second.joint_time_s = 10.04
        second.reference_time_s = .04
        self.assertEqual(worker.process(second, NS).status, 0)
        self.assertEqual(len(adapter.calls), 2)
        self.assertEqual(worker.accepted, 2)
        self.assertFalse(hasattr(adapter, "reset"))

    def test_identity_wrong_order_and_live_output_requests_are_rejected_before_solver(self):
        for name, value in (("schema_version", 2), ("session_id", 8), ("epoch", 2),
                            ("sequence", 2), ("replay_only", False), ("configuration_sha256", [0]*32)):
            with self.subTest(name=name):
                worker, adapter = engine()
                message = snapshot()
                setattr(message, name, value)
                out = worker.process(message, NS)
                self.assertEqual(out.status, 1)
                self.assertEqual(adapter.calls, [])
                self.assertEqual(out.joint_velocity, [0., 0.])
                self.assertTrue(worker.faulted)

    def test_future_and_stale_state_joint_and_wrong_interval_are_rejected(self):
        for name, value in (("state_time_s", 10.021), ("state_time_s", 9.),
                            ("joint_time_s", 10.024), ("joint_time_s", 9.99),
                            ("dt_s", .056), ("dt_s", .019), ("task_time_s", .021)):
            with self.subTest(name=name, value=value):
                worker, adapter = engine()
                message = snapshot()
                setattr(message, name, value)
                self.assertEqual(worker.process(message, NS).status, 1)
                self.assertEqual(adapter.calls, [])

    def test_changed_gain_target_rotation_or_nonfinite_state_is_rejected(self):
        for name, value in (("position_gains", [1.]*3), ("target_position_enu", [0.]*3),
                            ("target_velocity_enu", [.01, 0., 0.]), ("rotation_row_major", [0.]*9),
                            ("position_enu", [float("nan"), 0., 0.]), ("q", [0.])):
            with self.subTest(name=name):
                worker, adapter = engine()
                message = snapshot()
                setattr(message, name, value)
                self.assertEqual(worker.process(message, NS).status, 1)
                self.assertEqual(adapter.calls, [])

    def test_history_count_padding_future_order_and_reference_anchor_are_rejected(self):
        cases = []
        for name, value in (("packet_count", 0), ("command_count", 65), ("reference_time_s", .03)):
            message = snapshot()
            setattr(message, name, value)
            cases.append(message)
        for field, index, value in (("packet_q_ref", 127, 1.), ("command_time_s", 63, .01),
                                    ("packet_send_time_s", 1, -.02), ("packet_send_time_s", 1, .021)):
            message = snapshot()
            getattr(message, field)[index] = value
            cases.append(message)
        for message in cases:
            worker, adapter = engine()
            self.assertEqual(worker.process(message, NS).status, 1)
            self.assertEqual(adapter.calls, [])

    def test_expired_snapshot_has_deadline_status_and_future_issue_is_input_error(self):
        for issued, status in ((900_000_000, 3), (1_000_000_001, 1)):
            worker, adapter = engine()
            message = snapshot()
            message.issued_monotonic_ns = issued
            self.assertEqual(worker.process(message, NS).status, status)
            self.assertEqual(adapter.calls, [])

    def test_solver_failure_or_missing_quality_never_returns_a_fallback(self):
        for changed in ("exception", "accepted_optimum", "ts20_nonlinear_check_passed", "previous_plan_fallback"):
            worker, adapter = engine()
            if changed == "exception":
                adapter.error = RuntimeError("QP rejected")
            else:
                adapter.diagnostic[changed] = changed == "previous_plan_fallback"
            out = worker.process(snapshot(), NS)
            self.assertEqual(out.status, 2)
            self.assertFalse(out.accepted_optimum or out.nonlinear_check_passed)
            self.assertEqual(out.joint_velocity, [0., 0.])
            self.assertIsNone(worker.context.value)

    def test_outer_deadline_covers_conversion_adapter_and_clears_output(self):
        worker, adapter = engine(clock=Mock(side_effect=[1_000_000_000, 1_050_000_001]))
        out = worker.process(snapshot(), NS)
        self.assertEqual(len(adapter.calls), 1)
        self.assertEqual(out.status, 3)
        self.assertEqual(out.position_reference_enu, [0.]*3)
        self.assertEqual(worker.accepted, 0)
        self.assertEqual(worker.last_control_time, 10.)
        self.assertTrue(worker.faulted)
        worker, adapter = engine(clock=Mock(side_effect=[1_000_000_000, 1_050_000_001]))
        adapter.error = RuntimeError("original solver failure retained")
        self.assertEqual(worker.process(snapshot(), NS).status, 3)
        self.assertIn("original solver failure", worker.last_error)

    def test_rejection_latches_and_does_not_run_next_controller(self):
        worker, adapter = engine(clock=Mock(side_effect=[1_000_000_000, 1_001_000_000, 1_002_000_000, 1_003_000_000]))
        first = snapshot()
        first.dt_s = .056
        self.assertEqual(worker.process(first, NS).status, 1)
        second = snapshot()
        second.sequence = 2
        self.assertNotEqual(worker.process(second, NS).status, 0)
        self.assertEqual(adapter.calls, [])

    def test_context_identity_and_task_time_are_bound(self):
        context = SnapshotContext()
        state = object()
        context.bind(state, .02, {"value": 1})
        with self.assertRaises(ValueError):
            context(object(), .02)
        with self.assertRaises(ValueError):
            context(state, .03)
        context.clear()
        with self.assertRaises(ValueError):
            context(state, .02)

    def test_input_file_hash_is_checked_before_json_or_configuration_loading(self):
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            _pinned_json(__file__, "0"*64)


if __name__ == "__main__":
    unittest.main()
