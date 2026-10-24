"""No-library tests for bounded observation and streamed full evidence."""
import gc
import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import sys
import unittest
from unittest.mock import patch
import weakref
import numpy as np

PATH = Path(__file__).resolve().parents[3]/'scripts/qualification_recording.py'
SPEC = importlib.util.spec_from_file_location('qualification_recording_under_test', PATH)
recording = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(recording)


class FakeCollector:
    def __init__(self, enabled=True, change_policy=False):
        self.enabled=enabled;self.change_policy=change_policy;self.calls=[];self.garbage=[]
    def isenabled(self): return self.enabled
    def get_count(self): return (3,2,1) if not self.calls else (0,0,0)
    def get_threshold(self): return (701,10,10) if self.calls and self.change_policy else (700,10,10)
    def get_stats(self): return [{'collections':len(self.calls),'collected':4,'uncollectable':0}]*3
    def get_freeze_count(self): return 0
    def collect(self,generation): self.calls.append(generation);return 4


def preparation():
    return dict(fixture_index=0,fixture={'amplitude':.005},actions=['constructor','promote'],
                controller_steps_before=0,controller_steps_after=0,discarded_before_collection=True)


class QualificationRecordingTests(unittest.TestCase):
    def test_startup_collects_once_preserves_policy_and_cannot_repeat(self):
        with tempfile.TemporaryDirectory() as output:
            fake=FakeCollector();clock=iter([10,25])
            result=recording.collect_startup_garbage(output,preparation(),collector=fake,clock_ns=lambda:next(clock))
            self.assertEqual(fake.calls,[2]);self.assertEqual(result['duration_ns'],15)
            self.assertEqual(result['collected'],4);self.assertTrue(result['gc_policy_unchanged'])
            self.assertEqual(recording.verified_json(result['artifact'],output),{k:v for k,v in result.items() if k!='artifact'})
            with self.assertRaises(FileExistsError):recording.collect_startup_garbage(output,preparation(),collector=fake)
            self.assertEqual(fake.calls,[2])

    def test_startup_disabled_or_changed_policy_fails_closed(self):
        for fake,expected_calls in ((FakeCollector(False),[]),(FakeCollector(change_policy=True),[2])):
            with self.subTest(enabled=fake.enabled),tempfile.TemporaryDirectory() as output:
                with self.assertRaises(RuntimeError):recording.collect_startup_garbage(output,preparation(),collector=fake)
                self.assertEqual(fake.calls,expected_calls)
        with tempfile.TemporaryDirectory() as output:
            invalid=preparation();invalid['controller_steps_after']=1;fake=FakeCollector()
            with self.assertRaises(ValueError):recording.collect_startup_garbage(output,invalid,collector=fake)
            self.assertEqual(fake.calls,[])

    def test_matrix_preparation_has_no_steps_and_releases_objects_before_collection(self):
        sys.path.insert(0,str(PATH.parent))
        import validate_x650_native_numerics as validator
        calls=[];refs=[]
        class Controller:
            step_count=0
            def step(self,*args): raise AssertionError('Startup must not step')
        def build(spec):
            calls.append('construct');c=Controller();refs.append(weakref.ref(c));return c,object(),object(),object()
        def promote(c,native): calls.append('promote')
        def collect(output,entry):
            calls.append('collect');self.assertTrue(all(ref() is None for ref in refs))
            self.assertEqual(entry['controller_steps_before'],0);self.assertEqual(entry['controller_steps_after'],0)
            return entry
        with patch.object(validator,'promote',promote),patch.object(validator,'collect_startup_garbage',collect):
            result=validator.prepare_matrix_startup(build,{'mode':'stuck'},object(),Path('unused'))
        self.assertEqual(calls,['construct','promote','collect']);self.assertEqual(result['fixture'],{'mode':'stuck'})
        with patch.object(validator,'collect_startup_garbage') as collector:
            with self.assertRaises(RuntimeError):
                validator.prepare_matrix_startup(lambda spec:(_ for _ in ()).throw(RuntimeError('construction failed')),{},None,None)
            collector.assert_not_called()

    def test_matrix_main_places_single_startup_before_natural_trace_and_outside_loop(self):
        tree=ast.parse((PATH.parent/'validate_x650_native_numerics.py').read_text())
        main=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=='main')
        calls=[node for node in ast.walk(main) if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='prepare_matrix_startup']
        self.assertEqual(len(calls),1)
        trace=next(node for node in ast.walk(main) if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='NaturalGcTrace')
        self.assertLess(calls[0].lineno,trace.lineno)
        for loop in (node for node in ast.walk(main) if isinstance(node,(ast.For,ast.While))):
            self.assertNotIn(calls[0],list(ast.walk(loop)))

    def test_startup_artifact_and_trace_window_are_bound(self):
        with tempfile.TemporaryDirectory() as output:
            startup=recording.collect_startup_garbage(output,preparation(),collector=FakeCollector())
            report=self.fixture(output)
            report.update(startup_gc=startup,startup_timing_condition=recording.STARTUP_TIMING_CONDITION)
            self.assertTrue(recording.verify_streamed_evidence(output,report)['startup_gc_verified'])
            self.assertLessEqual(startup['end_perf_counter_ns'],report['natural_gc']['window_start_perf_counter_ns'])

    def test_invalid_startup_evidence_cannot_be_hidden_by_rebinding_hash(self):
        for mode in ('policy','count','late','fixture','summary','missing'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as output:
                startup=recording.collect_startup_garbage(output,preparation(),collector=FakeCollector())
                report=self.fixture(output);report.update(startup_gc=startup,startup_timing_condition=recording.STARTUP_TIMING_CONDITION)
                path=Path(startup['artifact']['path']);record=json.loads(path.read_text())
                if mode=='missing':path.unlink()
                elif mode=='summary':startup['collected']+=1
                else:
                    if mode=='policy':record['after']['thresholds'][0]+=1
                    elif mode=='count':record['explicit_collection_count']=2
                    elif mode=='late':
                        record['end_perf_counter_ns']=report['natural_gc']['window_end_perf_counter_ns']+1
                        record['duration_ns']=record['end_perf_counter_ns']-record['start_perf_counter_ns']
                    else:record['preparation']['fixture']={'amplitude':.01}
                    path.write_text(json.dumps(record));report['startup_gc']=record|{'artifact':recording.identity(path)}
                with self.assertRaises((ValueError,FileNotFoundError)):recording.verify_streamed_evidence(output,report)

    def test_full_evidence_is_owned_by_file_not_summary_or_store(self):
        with tempfile.TemporaryDirectory() as output:
            store = recording.RowEvidenceStore(output)
            array = np.arange(12.).reshape(6, 2)
            reference = weakref.ref(array)
            row = dict(index=4, pass_=True, candidate_execution_evidence=[dict(
                actual_propagation_calls=1, inputs=dict(packets=array),
                result=dict(envelope=dict(events=[dict(q=.1, kind='position_zero')])) )])
            summary = store.write(4, row)
            self.assertNotIn('candidate_execution_evidence', summary)
            self.assertEqual(summary['candidate_execution_count'], 1)
            path = Path(summary['row_evidence']['path'])
            content = path.read_bytes()
            self.assertEqual(summary['row_evidence']['sha256'], hashlib.sha256(content).hexdigest())
            saved = json.loads(content)
            self.assertEqual(saved['candidate_execution_evidence'][0]['inputs']['packets'], array.tolist())
            self.assertEqual(saved['candidate_execution_evidence'][0]['result']['envelope']['events'][0]['q'], .1)
            del array, row
            self.assertIsNone(reference())

    def test_failed_row_is_retained_and_existing_evidence_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as output:
            store = recording.RowEvidenceStore(output)
            summary = store.write(3, {'index':3, 'pass':False, 'error':'original 50ms deadline'})
            self.assertFalse(summary['pass'])
            path = Path(summary['row_evidence']['path']); original = path.read_bytes()
            with self.assertRaises(FileExistsError): store.write(3, {'pass':True})
            self.assertEqual(path.read_bytes(), original)

    def test_gc_scope_row_and_failure_clocks_preserve_policy_and_callbacks(self):
        callbacks = list(gc.callbacks)
        policy = recording.NaturalGcTrace.policy()
        denied = lambda *a, **k: self.fail('Qualification changed GC policy or requested collection')
        with patch.object(gc, 'collect', denied), patch.object(gc, 'disable', denied), \
             patch.object(gc, 'enable', denied), patch.object(gc, 'freeze', denied), \
             patch.object(gc, 'unfreeze', denied), patch.object(gc, 'set_threshold', denied):
            with recording.NaturalGcTrace(32) as trace:
                trace.begin_row(147)
                with self.assertRaisesRegex(ValueError, 'original deadline'):
                    with trace.step('native'):
                        trace._callback('start', {'generation':2})
                        trace._callback('stop', {'generation':2})
                        raise ValueError('original deadline')
                row = trace.end_row()
            self.assertEqual(recording.NaturalGcTrace.policy(), policy)
        self.assertEqual(gc.callbacks, callbacks)
        self.assertEqual(row['step_spans'][0]['row_index'], 147)
        self.assertFalse(row['step_spans'][0]['completed'])
        target = [event for event in row['events'] if event[2:] == [2, 147, 2]]
        self.assertEqual([event[1] for event in target], [1, 2])
        span = row['step_spans'][0]
        self.assertLessEqual(span['start_perf_counter_ns'], target[0][0])
        self.assertLessEqual(target[-1][0], span['end_perf_counter_ns'])
        self.assertGreaterEqual(span['thread_elapsed_ns'], 0)
        self.assertEqual(span['wall_elapsed_ns'], span['end_perf_counter_ns']-span['start_perf_counter_ns'])

    def test_outside_scope_events_are_not_misattributed_to_controller(self):
        with recording.NaturalGcTrace(64) as trace:
            trace.begin_row(5)
            with trace.step('reference'):
                trace._callback('start', {'generation':1})
                trace._callback('stop', {'generation':1})
            row = trace.end_row()
            trace._callback('start', {'generation':0})
            trace._callback('stop', {'generation':0})
        self.assertTrue(any(event[2:] == [1, 5, 1] for event in row['events']))
        np.testing.assert_array_equal(trace.buffer[trace.used-2:trace.used, 3:], [[-1,0],[-1,0]])

    def test_overflow_is_explicit_and_capacity_never_grows(self):
        with recording.NaturalGcTrace(2) as trace:
            for _ in range(5): trace._callback('start', {'generation':2})
        self.assertEqual(trace.buffer.shape, (2, 5))
        self.assertEqual(trace.used, 2)
        self.assertGreaterEqual(trace.seen, 5)
        with tempfile.TemporaryDirectory() as output:
            summary = trace.write(output)
            self.assertFalse(summary['complete']); self.assertTrue(summary['overflow'])
            saved = json.loads(Path(summary['artifact']['path']).read_text())
            self.assertEqual(len(saved['events']), 2)
            self.assertEqual(saved['columns'], list(trace.COLUMNS))

    def test_scopes_cannot_nest_or_escape_a_row_and_write_requires_closed_trace(self):
        with recording.NaturalGcTrace(32) as trace:
            with self.assertRaises(RuntimeError):
                with trace.step('reference'): pass
            trace.begin_row(1)
            with self.assertRaises(RuntimeError): trace.begin_row(2)
            with trace.step('reference'):
                with self.assertRaises(RuntimeError): trace.end_row()
                with self.assertRaises(RuntimeError):
                    with trace.step('native'): pass
            trace.end_row()
            with tempfile.TemporaryDirectory() as output:
                with self.assertRaises(RuntimeError): trace.write(output)

    @staticmethod
    def fixture(output):
        store = recording.RowEvidenceStore(output)
        with recording.NaturalGcTrace(256) as trace:
            trace.begin_row(0)
            with trace.step('reference'):
                trace._callback('start', {'generation':0})
                trace._callback('stop', {'generation':0})
            row_gc = trace.end_row()
        inputs = dict(start=0., packets=[dict(q_ref=[.1,.2])], query_times_s=[.02])
        result = dict(time_s=[.02], q=[[.1,.2]], dq=[[0.,0.]], modes=[[1,1]],
                      envelope=dict(collected=True, events=[dict(q=.15, kind='velocity_zero')]))
        candidate = dict(captured_during_actual_call=True, extra_rollouts=0,
            public_inputs_sha256=recording.value_digest(inputs), native_inputs_sha256=recording.value_digest(inputs),
            public_result_sha256=recording.value_digest(result), native_result_sha256=recording.value_digest(result),
            native=dict(actual_propagation_calls=1, track_extrema=True, inputs=inputs, result=result,
                        candidate_return=deepcopy(result)))
        row = dict(index=0, t_s=.02, dt_s=.02, fixture={'amplitude':.005},
                   candidate_execution_evidence=[candidate], candidate_and_mit_packets_equal=True,
                   natural_gc=row_gc)
        row['pass'] = True
        return dict(rows=[store.write(0,row)], total=1, passed=1, natural_gc=trace.write(output))

    def test_bound_full_rows_and_gc_trace_are_reopened_and_verified(self):
        with tempfile.TemporaryDirectory() as output:
            report = self.fixture(output)
            verified = recording.verify_streamed_evidence(output, report)
            self.assertTrue(verified['passed'])
            self.assertEqual(verified['rows'], 1); self.assertEqual(verified['candidates'], 1)

    def test_missing_or_changed_external_artifacts_cannot_pass(self):
        for artifact in ('row','gc'):
            with self.subTest(artifact=artifact), tempfile.TemporaryDirectory() as output:
                report = self.fixture(output)
                receipt = report['rows'][0]['row_evidence'] if artifact == 'row' else report['natural_gc']['artifact']
                path = Path(receipt['path']); original = path.read_bytes()
                path.write_bytes(original+b' ')
                with self.assertRaisesRegex(ValueError, 'size/hash/path changed'):
                    recording.verify_streamed_evidence(output, report)
                path.unlink()
                with self.assertRaises(FileNotFoundError): recording.verify_streamed_evidence(output, report)

    def test_summary_identity_and_row_order_are_checked_beyond_file_hash(self):
        for key, value in (('t_s',.04), ('index',1), ('fixture',{'amplitude':.01})):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as output:
                report = self.fixture(output); report['rows'][0][key] = value
                with self.assertRaises(ValueError): recording.verify_streamed_evidence(output, report)

    def test_rebound_candidate_or_gc_payload_still_requires_complete_evidence(self):
        for field in ('packets','empty_candidate','gc_scope'):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as output:
                report = self.fixture(output); receipt=report['rows'][0]['row_evidence']; path=Path(receipt['path'])
                row=json.loads(path.read_text())
                if field == 'packets': row['candidate_execution_evidence'][0]['native']['inputs']['packets'][0]['q_ref'][0] += .1
                elif field == 'empty_candidate': row['candidate_execution_evidence']=[]
                else: row['natural_gc']['events'][0][4]=2
                path.write_text(json.dumps(row))
                summary={k:v for k,v in row.items() if k!='candidate_execution_evidence'}
                summary.update(candidate_execution_count=len(row['candidate_execution_evidence']),row_evidence=recording.identity(path))
                report['rows'][0]=summary
                with self.assertRaises(ValueError): recording.verify_streamed_evidence(output, report)


if __name__ == '__main__': unittest.main()
