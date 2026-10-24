"""Thread placement contract without ROS, native solver, or CPU mutation."""
from copy import deepcopy
import ast
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flyarm_mpc_reference.worker_affinity import (
    WorkerAffinity, ExecutorCpuObserver, validate_thread_roles, verify_snapshot,
    verify_worker_evidence, thread_snapshot, WorkerStopRequest)

MASK = [87, 89, 91, 97]
ROLES = {'executor': [87], 'auxiliary': [89, 91, 97]}


class FakeThreads:
    def __init__(self):
        self.rows = {'10': {'start_ticks': 100, 'cpus': list(MASK), 'comm': 'python'},
                     '11': {'start_ticks': 101, 'cpus': list(MASK), 'comm': 'dds'}}
        self.sets = []
        self.now = 1000

    def snapshot(self, pid):
        assert pid == 10
        return deepcopy(self.rows)

    def set(self, tid, cpus):
        self.sets.append((tid, sorted(cpus)))
        self.rows[str(tid)]['cpus'] = sorted(cpus)

    def clock(self):
        self.now += 1
        return self.now

    def make(self, roles=ROLES):
        return WorkerAffinity(MASK, deepcopy(roles), pid=10,
                              snapshot=self.snapshot, setter=self.set, clock_ns=self.clock)


class WorkerAffinityTests(unittest.TestCase):
    def test_controlled_signal_requests_stop_without_ros_teardown(self):
        stop=WorkerStopRequest();old=object();calls=[]
        with patch('flyarm_mpc_reference.worker_affinity.signal.signal',return_value=old) as register:
            stop.install()
            self.assertEqual(register.call_count,2)
            def spin_once(**kwargs):
                calls.append(kwargs)
                if len(calls)==2:stop._request(2,None)
            stop.spin(NS(spin_once=spin_once),NS(ok=lambda:True))
            self.assertEqual(calls,[{'timeout_sec':.1}]*2)
            self.assertTrue(stop.requested)
            stop.restore()
            self.assertEqual(register.call_count,4)
            self.assertTrue(all(call.args[1] is old for call in register.call_args_list[-2:]))

    def test_proc_snapshot_parses_parenthesized_comm_and_detects_starttime_reuse(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            folder = Path(directory)/'10/task/10'; folder.mkdir(parents=True)
            stat = folder/'stat'
            def write(start): stat.write_text('10 (python (worker)) '+' '.join(['S']+['0']*18+[str(start)])+'\n')
            write(100)
            row = thread_snapshot(10, proc_root=directory, affinity=lambda tid: {87})
            self.assertEqual(row['10'], {'start_ticks': 100, 'cpus': [87], 'comm': 'python (worker)'})
            def racing_affinity(tid):
                write(101)
                return {87}
            with self.assertRaisesRegex(RuntimeError, 'identity changed'):
                thread_snapshot(10, proc_root=directory, affinity=racing_affinity)

    def test_valid_partition_and_explicit_legacy(self):
        self.assertEqual(validate_thread_roles(MASK, ROLES), ROLES)
        self.assertIsNone(validate_thread_roles(MASK, None))

    def test_invalid_overlap_outside_missing_or_malformed_masks(self):
        invalid = [
            {'executor': [87], 'auxiliary': MASK},
            {'executor': [86], 'auxiliary': [89, 91, 97]},
            {'executor': [87], 'auxiliary': [89, 91]},
            {'executor': [87, 89], 'auxiliary': [91, 97]},
            {'executor': [87], 'auxiliary': [97, 91, 89]},
            {'executor': [87], 'auxiliary': [89, 91, 91, 97]},
            {'executor': [True], 'auxiliary': [89, 91, 97]},
            {'executor': [], 'auxiliary': MASK},
            {'executor': [87], 'auxiliary': []},
            {'executor': [87], 'other': [89, 91, 97]},
        ]
        for roles in invalid:
            with self.subTest(roles=roles), self.assertRaises(ValueError):
                validate_thread_roles(MASK, roles)

    def test_bind_then_stages_preserve_tid_starttime_and_exact_masks(self):
        fixture = FakeThreads(); helper = fixture.make()
        helper.bind_before_ready()
        self.assertEqual(fixture.sets, [(10, [87]), (11, [89, 91, 97])])
        for stage in ('ready', 'live_start', 'final'):
            helper.check(stage)
        report = helper.document()
        self.assertTrue(report['complete'])
        self.assertEqual(verify_worker_evidence(report, fixture.rows, MASK, ROLES, require_final=True), 10)
        self.assertIn('short-lived', report['scope'])

    def test_legacy_does_not_repin_and_requires_full_masks(self):
        fixture = FakeThreads(); helper = fixture.make(None)
        helper.bind_before_ready(); helper.check('ready')
        self.assertEqual(fixture.sets, [])
        self.assertIsNone(helper.document()['thread_roles'])
        fixture.rows['11']['cpus'] = [89, 91, 97]
        with self.assertRaises(ValueError): helper.check('live_start')

    def test_unexpected_startup_mask_rejected_before_any_set(self):
        fixture = FakeThreads(); fixture.rows['11']['cpus'] = [87]
        with self.assertRaises(ValueError): fixture.make().bind_before_ready()
        self.assertEqual(fixture.sets, [])

    def test_thread_created_during_binding_rejected(self):
        fixture = FakeThreads(); original = fixture.set
        def setter(tid, cpus):
            original(tid, cpus)
            fixture.rows['12'] = {'start_ticks': 102, 'cpus': [87], 'comm': 'late'}
        helper = WorkerAffinity(MASK, ROLES, pid=10, snapshot=fixture.snapshot, setter=setter)
        with self.assertRaisesRegex(RuntimeError, 'changed during placement'):
            helper.bind_before_ready()

    def test_later_unknown_missing_reused_or_misplaced_thread_rejects(self):
        for case in ('unknown', 'missing', 'reuse', 'affinity'):
            fixture = FakeThreads(); helper = fixture.make(); helper.bind_before_ready()
            helper.check('ready')
            if case == 'unknown': fixture.rows['12'] = {'start_ticks': 102, 'cpus': ROLES['auxiliary'], 'comm': 'late'}
            elif case == 'missing': fixture.rows.pop('11')
            elif case == 'reuse': fixture.rows['11']['start_ticks'] = 999
            else: fixture.rows['11']['cpus'] = [87]
            with self.subTest(case=case), self.assertRaises(ValueError): helper.check('live_start')
            self.assertFalse(helper.document()['complete'])
            self.assertEqual(helper.document()['failure']['phase'], 'live_start')

    def test_snapshot_os_failure_latches_failed_evidence(self):
        fixture = FakeThreads(); helper = fixture.make(); helper.bind_before_ready()
        helper.snapshot = Mock(side_effect=ProcessLookupError('gone'))
        with self.assertRaises(ProcessLookupError): helper.check('ready')
        self.assertFalse(helper.document()['complete'])

    def test_cpu_boundary_checks_native_tid_exact_mask_and_actual_cpu(self):
        observer = ExecutorCpuObserver(10, [87], get_cpu=lambda: 87,
                                       get_tid=lambda: 10, get_affinity=lambda tid: {87})
        self.assertEqual(observer(), (10, 87))
        for changes in ({'get_cpu': lambda: 97}, {'get_tid': lambda: 11},
                        {'get_affinity': lambda tid: {87, 89}}, {'get_cpu': lambda: -1}):
            kwargs = {'get_cpu': lambda: 87, 'get_tid': lambda: 10, 'get_affinity': lambda tid: {87}}
            kwargs.update(changes)
            with self.subTest(changes=changes), self.assertRaises(RuntimeError):
                ExecutorCpuObserver(10, [87], **kwargs)()

    def test_source_or_stage_evidence_drift_rejected(self):
        fixture = FakeThreads(); helper = fixture.make(); helper.bind_before_ready()
        for phase in ('ready', 'live_start', 'final'): helper.check(phase)
        for case in ('source', 'missing_stage', 'clock', 'main_tid'):
            report = helper.document(); report = deepcopy(report)
            if case == 'source': report['source_sha256'] = '0'*64
            elif case == 'missing_stage': report['phases'].pop('live_start')
            elif case == 'clock': report['phases']['final']['monotonic_ns'] = 1
            else: report['executor_tid'] = 11
            with self.subTest(case=case), self.assertRaises(ValueError):
                verify_worker_evidence(report, fixture.rows, MASK, ROLES, require_final=True)

    def worker_method(self, name, namespace):
        path = Path(__file__).resolve().parents[1]/'flyarm_mpc_reference/isolated_node.py'
        tree = ast.parse(path.read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        return namespace[name]

    def test_binding_and_gc_finish_before_ready_marker(self):
        order = []
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            affinity = NS(pid=10, roles=ROLES,
                bind_before_ready=lambda: order.append('bind'),
                check=lambda phase: order.append('check:'+phase),
                write=lambda path: order.append('write:'+path.name))
            worker = NS(out=Path(directory), affinity=affinity)
            namespace = {'ExecutorCpuObserver': lambda *args: lambda: order.append('cpu'),
                         'collect_preflight_garbage': lambda path: order.append('gc')}
            self.worker_method('complete_preflight', namespace)(worker)
            self.assertEqual(order, ['bind', 'cpu', 'gc', 'check:ready', 'write:worker_affinity_ready.json'])
            self.assertTrue((worker.out/'worker_ready').is_file())

    def test_affinity_report_failure_cannot_block_existing_sender_trace_cleanup(self):
        order = []
        def failed_write(path):
            order.append('affinity_write')
            raise OSError('report write failed')
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            affinity = NS(check=lambda phase: order.append('check:'+phase), write=failed_write)
            worker = NS(out=Path(directory), affinity=affinity,
                engine=NS(metadata=lambda: {}), gc_event=object(), gc_was_enabled=True,
                gc_events=[], capture=NS(completion_error=None),
                sender=NS(close=lambda path: order.append('sender_close')),
                timing=NS(write=lambda path: order.append('timing_write')))
            fake_gc = NS(callbacks=NS(remove=lambda callback: order.append('gc_remove')), isenabled=lambda: True)
            method = self.worker_method('finish', {'gc': fake_gc, 'json': json})
            with self.assertRaisesRegex(OSError, 'report write failed'): method(worker)
            self.assertEqual(order, ['check:final', 'gc_remove', 'sender_close', 'timing_write', 'affinity_write'])
            self.assertTrue((worker.out/'worker_final.json').is_file())


if __name__ == '__main__': unittest.main()
