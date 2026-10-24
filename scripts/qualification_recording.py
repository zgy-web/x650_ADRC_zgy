"""Bounded natural-GC observation and per-row evidence for offline qualification.

This module does not change collection policy or controller deadlines. Full
evidence is written only after the measured controller calls have returned.
"""
from contextlib import contextmanager
import gc
import hashlib
import json
from pathlib import Path
from time import perf_counter_ns, thread_time_ns
import numpy as np


def plain(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)): return [plain(item) for item in value]
    return value


def identity(path):
    data = path.read_bytes()
    return dict(path=str(path.resolve()), bytes=len(data), sha256=hashlib.sha256(data).hexdigest())


class RowEvidenceStore:
    """Keep no reference to a completed row or its candidate execution graphs."""
    def __init__(self, output):
        self.directory = Path(output)/'row_evidence'
        self.directory.mkdir(exist_ok=False)

    def write(self, index, row):
        path = self.directory/f'{index:06d}.json'
        with path.open('x', encoding='utf-8') as stream:
            json.dump(plain(row), stream, indent=2, allow_nan=False)
            stream.write('\n')
        # Only the bulky execution graph leaves the primary report. All
        # assertions already ran; every full row remains hash-bound on disk.
        summary = {key: plain(value) for key, value in row.items()
                   if key != 'candidate_execution_evidence'}
        evidence = row.get('candidate_execution_evidence', [])
        summary['candidate_execution_count'] = len(evidence)
        summary['row_evidence'] = identity(path)
        return summary


class NaturalGcTrace:
    """One bounded numeric buffer; callback performs no I/O or policy changes."""
    COLUMNS = ('perf_counter_ns', 'phase', 'generation', 'row_index', 'scope')
    SCOPES = {'outside_step': 0, 'reference': 1, 'native': 2, 'native_uninstrumented': 3}

    def __init__(self, capacity=32768):
        if capacity <= 0: raise ValueError('GC event capacity must be positive')
        self.buffer = np.empty((int(capacity), len(self.COLUMNS)), dtype=np.int64)
        self.used = 0
        self.seen = 0
        self.overflow = False
        self.active_row = -1
        self.active_scope = 0
        self._callback_handle = None
        self._spans = []
        self._row_begin = 0

    @staticmethod
    def policy():
        return dict(enabled=gc.isenabled(), thresholds=list(gc.get_threshold()),
                    frozen_count=gc.get_freeze_count())

    def _callback(self, phase, info):
        if phase not in ('start', 'stop'): return
        stamp = perf_counter_ns()
        self.seen += 1
        if self.used >= len(self.buffer):
            self.overflow = True
            return
        index = self.used
        self.buffer[index, 0] = stamp
        self.buffer[index, 1] = 1 if phase == 'start' else 2
        self.buffer[index, 2] = int(info['generation'])
        self.buffer[index, 3] = self.active_row
        self.buffer[index, 4] = self.active_scope
        self.used = index + 1

    def __enter__(self):
        if self._callback_handle is not None: raise RuntimeError('GC trace already active')
        self.before = self.policy()
        self._callback_handle = self._callback
        gc.callbacks.append(self._callback_handle)
        return self

    def __exit__(self, *exception):
        # Remove this exact callback, preserving unrelated callbacks/order.
        gc.callbacks.remove(self._callback_handle)
        self._callback_handle = None
        self.after = self.policy()

    def begin_row(self, index):
        if self.active_row != -1: raise RuntimeError('GC row already active')
        self.active_row = int(index)
        self._row_begin = self.used
        self._spans = []

    @contextmanager
    def step(self, backend):
        if self.active_row < 0 or self.active_scope != 0:
            raise RuntimeError('GC step must belong to one active row')
        code = self.SCOPES[backend]
        if code == 0: raise ValueError('Expected a controller step scope')
        span = dict(backend=backend, row_index=self.active_row,
                    start_perf_counter_ns=perf_counter_ns(),
                    start_thread_time_ns=thread_time_ns(), completed=False)
        self.active_scope = code
        try:
            yield
            span['completed'] = True
        finally:
            span['end_perf_counter_ns'] = perf_counter_ns()
            span['end_thread_time_ns'] = thread_time_ns()
            span['wall_elapsed_ns'] = span['end_perf_counter_ns']-span['start_perf_counter_ns']
            span['thread_elapsed_ns'] = span['end_thread_time_ns']-span['start_thread_time_ns']
            self.active_scope = 0
            self._spans.append(span)

    def end_row(self):
        if self.active_row < 0 or self.active_scope != 0:
            raise RuntimeError('GC row cannot close during a step')
        index = self.active_row
        self.active_row = -1
        events = self.buffer[self._row_begin:self.used].copy()
        events = events[events[:, 3] == index]
        spans, self._spans = self._spans, []
        return dict(row_index=index, columns=list(self.COLUMNS),
                    phases={'start': 1, 'stop': 2}, scopes=dict(self.SCOPES),
                    events=events.tolist(), step_spans=spans,
                    overflow=self.overflow, cumulative_events_seen=self.seen)

    def write(self, output):
        if self._callback_handle is not None:
            raise RuntimeError('Close GC trace before writing final evidence')
        report = dict(kind='natural_gc_observation_v1', complete=not self.overflow,
                      overflow=self.overflow, capacity=len(self.buffer),
                      events_seen=self.seen, events_recorded=self.used,
                      columns=list(self.COLUMNS), phases={'start': 1, 'stop': 2},
                      scopes=dict(self.SCOPES), events=self.buffer[:self.used].tolist(),
                      policy_before=self.before, policy_after=self.after,
                      policy_unchanged=self.before == self.after,
                      requested_collection=False, disabled_collection=False)
        path = Path(output)/'natural_gc_trace.json'
        with path.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write('\n')
        return {key: value for key, value in report.items() if key != 'events'} | {'artifact': identity(path)}


def verified_json(receipt, directory):
    path = Path(receipt['path']).resolve()
    # Artifacts are bound to this particular run, not arbitrary external files.
    if not path.is_relative_to(Path(directory).resolve()):
        raise ValueError('Evidence artifact escapes the qualification output')
    if identity(path) != receipt:
        raise ValueError('Evidence artifact size/hash/path changed: '+str(path))
    return json.loads(path.read_text(encoding='utf-8'))


def value_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def verify_streamed_evidence(output, report):
    """Re-open every bound file, releasing each full row before reading the next.

    Production qualification and deployment preflight must call this verifier
    too after integration; hashing only the compact report is insufficient.
    """
    gc_summary = report['natural_gc']
    trace = verified_json(gc_summary['artifact'], output)
    if {k:v for k,v in trace.items() if k != 'events'} != {k:v for k,v in gc_summary.items() if k != 'artifact'}:
        raise ValueError('GC trace summary differs from its full artifact')
    if (not trace['complete'] or trace['overflow'] or not trace['policy_unchanged']
            or trace['policy_before'] != trace['policy_after']
            or not trace['policy_before']['enabled']
            or trace['requested_collection'] or trace['disabled_collection']):
        raise ValueError('Natural GC evidence incomplete or collection policy changed')
    events = trace['events']
    if (trace['events_seen'] != trace['events_recorded'] or len(events) != trace['events_recorded']
            or len(events) > trace['capacity'] or trace['columns'] != list(NaturalGcTrace.COLUMNS)):
        raise ValueError('GC event count/schema differs')
    previous_ns = -1
    for event in events:
        if len(event) != 5 or event[0] < previous_ns or event[1] not in (1,2) or event[2] not in (0,1,2):
            raise ValueError('Invalid GC phase/generation/clock')
        previous_ns = event[0]
    rows = report['rows']
    if len(rows) != report['total'] or sum(bool(r['pass']) for r in rows) != report['passed']:
        raise ValueError('Qualification row count differs')
    candidates = 0
    for index, summary in enumerate(rows):
        row = verified_json(summary['row_evidence'], output)
        if summary['index'] != index or row['index'] != index:
            raise ValueError('Duplicate, missing, or reordered qualification row')
        expected = {key:value for key,value in row.items() if key != 'candidate_execution_evidence'}
        evidence = row.get('candidate_execution_evidence', [])
        expected.update(candidate_execution_count=len(evidence), row_evidence=summary['row_evidence'])
        if expected != summary:
            raise ValueError('Compact summary/fixture/time differs from full row')
        if row['pass'] and (not evidence or row.get('candidate_and_mit_packets_equal') is not True):
            raise ValueError('Successful row lacks actual candidate evidence')
        for candidate in evidence:
            native = candidate['native']
            if (candidate['captured_during_actual_call'] is not True or candidate['extra_rollouts'] != 0
                    or native['actual_propagation_calls'] != 1 or native['track_extrema'] is not True
                    or native['result']['envelope']['collected'] is not True
                    or native['candidate_return'] != native['result']):
                raise ValueError('Actual candidate/complete extrema evidence differs')
            for key in ('inputs', 'result'):
                actual = value_digest(native[key])
                if candidate['native_'+key+'_sha256'] != actual or candidate['public_'+key+'_sha256'] != actual:
                    raise ValueError('Captured candidate hash/equivalence differs')
            candidates += 1
        row_gc = row['natural_gc']
        if (row_gc['row_index'] != index or row_gc['overflow']
                or row_gc['events'] != [event for event in events if event[3] == index]):
            raise ValueError('Per-row GC events differ from complete trace')
        for span in row_gc['step_spans']:
            if (span['row_index'] != index or span['end_perf_counter_ns'] < span['start_perf_counter_ns']
                    or span['end_thread_time_ns'] < span['start_thread_time_ns']
                    or span['wall_elapsed_ns'] != span['end_perf_counter_ns']-span['start_perf_counter_ns']
                    or span['thread_elapsed_ns'] != span['end_thread_time_ns']-span['start_thread_time_ns']):
                raise ValueError('Invalid controller step clock span')
            code = NaturalGcTrace.SCOPES[span['backend']]
            for event in row_gc['events']:
                if event[4] == code and not span['start_perf_counter_ns'] <= event[0] <= span['end_perf_counter_ns']:
                    raise ValueError('GC event outside its controller step span')
        # Explicitly release the complete graph before the next file read.
        del row, evidence, expected
    return dict(passed=True, rows=len(rows), candidates=candidates,
                artifact_identity='path+bytes+sha256', complete_gc_trace_verified=True)
