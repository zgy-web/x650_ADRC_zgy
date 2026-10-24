"""Private live publication boundary retains every diagnostic after publish."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_live_diagnostic_snapshot import make_adapter, measured, record_value
from test_isolated_worker_timing import make_case, row_dict
from flyarm_mpc_reference.deferred_diagnostics import DeferredLiveDiagnostics
from flyarm_mpc_reference.diagnostic_channel import CaptureAdapter
from flyarm_mpc_reference.isolated_node import process_snapshot
from x650_wholebody.api import EndpointTarget
from x650_wholebody import adapter as common


class DeferredDiagnosticTests(unittest.TestCase):
    def test_commands_and_complete_records_equal_owned_adapter_over_reused_buffers(self):
        original, _ = make_adapter(True)
        adapter, allocator = make_adapter(True)
        deferred = DeferredLiveDiagnostics(adapter)
        saved = []
        for i in range(30):
            s, target = measured(i), EndpointTarget([0., 0., 1.6])
            a = original.step(i*.02, .02, s, target)
            b = deferred.step(i*.02, .02, s, target)
            for name in ('position_reference', 'velocity_feedforward', 'acceleration_feedforward',
                         'base_velocity_target', 'joint_velocity', '_feedback_position', '_feedback_velocity'):
                np.testing.assert_array_equal(getattr(a, name), getattr(b, name))
            for key in ('accepted_optimum', 'previous_plan_fallback', 'ts20_nonlinear_check_passed',
                        'primal_violation', 'kkt_stationarity_inf', 'ts20_relinearization_attempts'):
                self.assertEqual(record_value(a.diagnostics[key]), record_value(b.diagnostics[key]))
            d = deferred.finish_diagnostics()
            self.assertEqual(record_value(a.diagnostics), record_value(d))
            saved.append((d, record_value(d)))
        allocator.prediction[:] = -999.
        allocator.envelope[0]['q'][:] = 888.
        self.assertTrue(all(record_value(d) == value for d, value in saved))

    def test_ff_and_copy_are_deferred_and_step_reset_cannot_overtake(self):
        adapter, _ = make_adapter(True)
        deferred = DeferredLiveDiagnostics(adapter)
        with patch.object(common, 'arm_reaction_feedforward', wraps=common.arm_reaction_feedforward) as ff, \
             patch.object(adapter, '_snapshot_diagnostics', wraps=adapter._snapshot_diagnostics) as clone:
            deferred.step(0., .02, measured(0), EndpointTarget([0., 0., 1.6]))
            self.assertEqual((ff.call_count, clone.call_count), (0, 0))
            with self.assertRaisesRegex(RuntimeError, 'Finish pending'):
                deferred.step(.02, .02, measured(1), EndpointTarget([0., 0., 1.6]))
            with self.assertRaisesRegex(RuntimeError, 'Finish pending'):
                deferred.reset([0., 0., 2.], np.eye(3), [0., 0.])
            deferred.finish_diagnostics()
            self.assertEqual((ff.call_count, clone.call_count), (1, 1))
            with self.assertRaisesRegex(RuntimeError, 'No pending'):
                deferred.finish_diagnostics()

    def test_only_pre_step_owned_zero_policy_can_opt_in(self):
        for owned, policy in ((False, 'zero'), (True, 'legacy')):
            adapter, _ = make_adapter(owned)
            adapter.acceleration_feedforward_policy = policy
            with self.assertRaises(ValueError): DeferredLiveDiagnostics(adapter)
        adapter, _ = make_adapter(True)
        adapter.step(0., .02, measured(0), EndpointTarget([0., 0., 1.6]))
        with self.assertRaises(ValueError): DeferredLiveDiagnostics(adapter)

    def test_invalid_diagnostics_fail_before_pending_and_no_clone(self):
        adapter, allocator = make_adapter(True)
        deferred = DeferredLiveDiagnostics(adapter)
        allocator.allocation['diagnostics'] = []
        with self.assertRaises(TypeError):
            deferred.step(0., .02, measured(0), EndpointTarget([0., 0., 1.6]))
        self.assertIsNone(deferred._pending)
        self.assertIsNone(adapter._last_task_t)

    def test_completion_failure_stays_pending_and_is_explicitly_after_publication(self):
        adapter, _ = make_adapter(True)
        deferred = DeferredLiveDiagnostics(adapter)
        capture = CaptureAdapter(deferred)
        capture.step(0., .02, measured(0), EndpointTarget([0., 0., 1.6]))
        with patch.object(common, 'arm_reaction_feedforward', side_effect=RuntimeError('diagnostic fixture')):
            with self.assertRaisesRegex(RuntimeError, 'diagnostic fixture'):
                capture.complete_diagnostic(17)
        self.assertEqual(capture.completion_error['sequence'], 17)
        self.assertTrue(capture.completion_error['after_publication'])
        self.assertFalse(capture.completion_error['complete'])
        self.assertIsNotNone(deferred._pending)

    def test_worker_finishes_after_publish_and_before_immutable_serialization(self):
        args, clock, order, packets, _ = make_case()
        def complete(sequence):
            self.assertEqual(sequence, 211)
            order.append('finish')
            clock.wait(1_450_000, cpu=True)
            args[1].diagnostic = {'complete': True}
        args[1].complete_diagnostic = complete
        process_snapshot(*args, clock_ns=clock.now, thread_clock_ns=clock.thread)
        self.assertEqual(order, ['engine', 'publish', 'finish', 'snapshot_cdr', 'proposal_cdr', 'offer'])
        row = row_dict(args[-1])
        self.assertEqual(row['diagnostic_complete_end_ns']-row['diagnostic_complete_begin_ns'], 1_450_000)
        self.assertLessEqual(row['publish_end_ns'], row['diagnostic_complete_begin_ns'])
        self.assertLessEqual(row['diagnostic_complete_end_ns'], row['snapshot_cdr_begin_ns'])
        self.assertEqual(row['diagnostic_offered'], 1)

    def test_worker_completion_error_faults_and_exits_without_fake_complete_record(self):
        args, clock, order, packets, _ = make_case()
        def complete(sequence): raise RuntimeError('finish failed')
        args[1].complete_diagnostic = complete
        with self.assertRaisesRegex(RuntimeError, 'finish failed'):
            process_snapshot(*args, clock_ns=clock.now, thread_clock_ns=clock.thread)
        self.assertEqual(order, ['engine', 'publish'])
        self.assertTrue(args[0].faulted)
        self.assertIn('Post-publication', args[0].last_error)
        row = row_dict(args[-1])
        self.assertGreater(row['diagnostic_complete_begin_ns'], 0)
        self.assertEqual(row['diagnostic_complete_end_ns'], 0)
        self.assertEqual(row['diagnostic_offered'], 0)
        self.assertEqual(packets, [])


if __name__ == '__main__':
    unittest.main()
