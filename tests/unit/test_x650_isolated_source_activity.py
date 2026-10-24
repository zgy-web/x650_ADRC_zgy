import gc
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_isolated_live_v1.source_activity import Phase, SourceActivityTrace


class Clock:
    def __init__(self):
        self.wall, self.cpu, self.tid = 1000, 200, 77

    def wall_ns(self):
        self.wall += 100
        return self.wall

    def cpu_ns(self):
        self.cpu += 10
        return self.cpu


class SourceActivityTest(unittest.TestCase):
    def make_trace(self, capacity=4, gc_capacity=8):
        clock = Clock()
        trace = SourceActivityTrace(capacity, gc_capacity, clock_ns=clock.wall_ns,
                                    thread_clock_ns=clock.cpu_ns, thread_id=lambda: clock.tid)
        self.addCleanup(trace.close)
        return trace, clock

    def row(self, report, index=0):
        return dict(zip(report['columns'], report['rows'][index]))

    def test_first_supervision_exception_retains_begin_failure_and_finally(self):
        trace, _ = self.make_trace()
        token = trace.begin()
        try:
            trace.mark(token, Phase.SUPERVISION)
            raise RuntimeError('first poll sees fault')
        except RuntimeError:
            trace.fail(token)
        finally:
            trace.end(token)
        trace.close()
        report = trace.report()
        row = self.row(report)
        self.assertTrue(report['complete'])
        self.assertEqual((row['failed'], row['failure_phase'], row['last_phase'], row['completed']),
                         (1, Phase.SUPERVISION, Phase.SUPERVISION, 1))
        self.assertLess(row['begin_ns'], row['supervision_begin_ns'])
        self.assertLess(row['supervision_begin_ns'], row['failure_ns'])
        self.assertLess(row['failure_ns'], row['end_ns'])
        self.assertLess(row['failure_thread_cpu_ns'], row['end_thread_cpu_ns'])
        self.assertEqual(row['mavlink_ros_begin_ns'], 0)
        self.assertEqual(report['freeze']['phase'], Phase.SUPERVISION)
        self.assertEqual(report['freeze']['reason'], 1)

    def test_end_failed_is_sufficient_and_preserves_failure_phase(self):
        trace, _ = self.make_trace()
        token = trace.begin()
        trace.mark(token, Phase.READ)
        trace.end(token, failed=True)
        trace.close()
        row = self.row(trace.report())
        self.assertEqual(row['failure_phase'], Phase.READ)
        self.assertEqual(row['last_phase'], Phase.READ)
        self.assertEqual(row['failed'], 1)
        self.assertGreater(row['end_ns'], row['failure_ns'])

    def test_repeated_mark_preserves_first_entry_with_separate_second_phase(self):
        trace, _ = self.make_trace()
        token = trace.begin()
        trace.mark(token, Phase.SUPERVISION)
        first = trace._rows[0, 12:14].copy()
        trace.mark(token, Phase.SUPERVISION)
        np.testing.assert_array_equal(trace._rows[0, 12:14], first)
        trace.mark(token, Phase.SUPERVISION_SECOND)
        trace.end(token)
        trace.close()
        report = trace.report()
        self.assertEqual(report['duplicate_marks'], 1)
        self.assertTrue(report['complete'])
        row = self.row(report)
        self.assertGreater(row['supervision_second_begin_ns'], row['supervision_begin_ns'])

    def test_external_freeze_keeps_open_finally_but_recovery_cannot_overwrite(self):
        trace, _ = self.make_trace()
        token = trace.begin()
        trace.mark(token, Phase.MAVLINK_ROS)
        trace.freeze()
        trace.mark(token, Phase.PUBLISH)
        trace.end(token)
        frozen = trace._rows.copy()
        self.assertEqual(trace.begin(), 0)
        trace.mark(0, Phase.READ)
        trace.end(0, failed=True)
        trace._gc_event('start', {'generation': 2})
        trace.end(token)  # stale token cannot rewrite the original finally
        np.testing.assert_array_equal(trace._rows, frozen)
        trace.close()
        report = trace.report()
        self.assertEqual((report['attempted'], report['gc']['attempted']), (1, 0))
        self.assertEqual(self.row(report)['publish_begin_ns'], 0)
        self.assertEqual(report['freeze']['reason'], 2)
        self.assertTrue(report['complete'])

    def test_capacity_overflow_is_explicit_and_retains_all_earlier_rows(self):
        trace, _ = self.make_trace(capacity=1, gc_capacity=1)
        token = trace.begin()
        trace.end(token)
        saved = trace._rows.copy()
        self.assertEqual(trace.begin(), 0)
        self.assertEqual(trace.begin(), 0)
        trace._gc_event('start', {'generation': 0})
        first_gc = trace._gc_rows.copy()
        trace._gc_event('stop', {'generation': 0, 'collected': 4})
        np.testing.assert_array_equal(trace._rows, saved)
        np.testing.assert_array_equal(trace._gc_rows, first_gc)
        trace.close()
        report = trace.report()
        self.assertEqual((report['attempted'], report['recorded'], report['overflow']), (3, 1, 2))
        self.assertEqual((report['gc']['attempted'], report['gc']['overflow']), (2, 1))
        self.assertFalse(report['complete'])

    def test_gc_records_raw_all_generations_threads_and_active_phase(self):
        trace, clock = self.make_trace()
        token = trace.begin()
        trace.mark(token, Phase.CONTEXT)
        info = {'generation': 0, 'collected': 3, 'uncollectable': 1}
        for generation in range(3):
            info['generation'] = generation
            clock.tid = 77 + generation
            trace._gc_event('start', info)
            trace._gc_event('stop', info)
        info.update(generation=99, collected=999)
        trace.end(token)
        trace.close()
        gc_report = trace.report()['gc']
        rows = [dict(zip(gc_report['columns'], row)) for row in gc_report['rows']]
        self.assertEqual([row['phase'] for row in rows], [1, 2] * 3)
        self.assertEqual([row['generation'] for row in rows], [0, 0, 1, 1, 2, 2])
        self.assertEqual([row['thread_id'] for row in rows], [77, 77, 78, 78, 79, 79])
        self.assertTrue(all(row['pump_id'] == token and row['pump_phase'] == Phase.CONTEXT
                            and row['collected'] == 3 and row['uncollectable'] == 1 for row in rows))
        self.assertTrue(all(a['monotonic_ns'] < b['monotonic_ns']
                            and a['thread_cpu_ns'] < b['thread_cpu_ns'] for a, b in zip(rows, rows[1:])))

    def test_gc_registration_does_not_change_policy_or_other_callbacks(self):
        trace, _ = self.make_trace()
        callbacks_before = tuple(gc.callbacks)
        policy_before = gc.isenabled(), gc.get_threshold()
        foreign = lambda phase, info: None
        try:
            with patch.object(gc, 'collect', side_effect=AssertionError('must not collect')), \
                    patch.object(gc, 'disable', side_effect=AssertionError('must not disable')), \
                    patch.object(gc, 'enable', side_effect=AssertionError('must not enable')), \
                    patch.object(gc, 'set_threshold', side_effect=AssertionError('must not retune')):
                trace.start_gc()
                trace.start_gc()
                self.assertEqual(sum(c is trace._gc_handle for c in gc.callbacks), 1)
                gc.callbacks.append(foreign)
                trace.close()
                trace.close()
            self.assertEqual(tuple(gc.callbacks), callbacks_before + (foreign,))
            self.assertEqual((gc.isenabled(), gc.get_threshold()), policy_before)
            self.assertTrue(trace.report()['gc']['policy_unchanged'])
        finally:
            if foreign in gc.callbacks:
                gc.callbacks.remove(foreign)

    def test_materialization_requires_shutdown_and_returns_owned_json_data(self):
        trace, _ = self.make_trace()
        with self.assertRaises(RuntimeError):
            trace.report()
        token = trace.begin()
        trace.end(token)
        trace.close()
        report = trace.report()
        json.dumps(report, allow_nan=False)
        report['rows'][0][0] = 999
        self.assertEqual(trace.report()['rows'][0][0], token)
        self.assertEqual(trace._rows.dtype, np.uint64)
        self.assertEqual(trace._gc_rows.dtype, np.uint64)
        self.assertEqual(report['allocated_bytes'],
                         (4 * len(trace.COLUMNS) + 8 * len(trace.GC_COLUMNS)) * 8)
        self.assertTrue(report['buffers_prefaulted'])

    def test_unfinished_pump_and_overlap_are_not_silently_complete(self):
        trace, _ = self.make_trace()
        trace.begin()
        self.assertEqual(trace.begin(), 0)
        trace.close()
        report = trace.report()
        self.assertEqual(report['unfinished_records'], 1)
        self.assertEqual(report['overlapping_pumps'], 1)
        self.assertFalse(report['complete'])
        self.assertGreater(self.row(report)['begin_ns'], 0)
        self.assertEqual(self.row(report)['end_ns'], 0)

    def test_stale_tokens_and_invalid_phases_cannot_corrupt_other_rows(self):
        trace, _ = self.make_trace()
        first = trace.begin()
        trace.end(first)
        second = trace.begin()
        trace.mark(first, Phase.READ)
        trace.end(first, failed=True)
        trace.mark(second, 999)
        trace.mark(second, Phase.PUBLISH)
        trace.end(second)
        trace.close()
        report = trace.report()
        self.assertEqual(self.row(report, 1)['read_begin_ns'], 0)
        self.assertGreater(self.row(report, 1)['publish_begin_ns'], 0)
        self.assertEqual(report['invalid_marks'], 1)
        self.assertFalse(report['complete'])

    def test_invalid_capacity_and_gc_restart_after_freeze_are_rejected(self):
        for value in (0, -1, 1.5, True):
            with self.assertRaises(ValueError):
                SourceActivityTrace(value)
            with self.assertRaises(ValueError):
                SourceActivityTrace(1, value)
        trace, _ = self.make_trace()
        trace.freeze()
        with self.assertRaises(RuntimeError):
            trace.start_gc()


if __name__ == '__main__':
    unittest.main()
