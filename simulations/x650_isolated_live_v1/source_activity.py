"""Bounded source-thread observations; no I/O or GC policy changes in flight.

Allocate and touch both buffers before activation, then call ``start_gc``.
Each pump owns one integer token and one row, including a pump that fails at
its first supervision poll. A mark is the *entry* to a named phase; zero means
that phase was never reached. Duplicate marks retain the first timestamp.

Typical integration (all exceptions still propagate to the lifecycle owner)::

    token = trace.begin()
    try:
        trace.mark(token, trace.SUPERVISION)
        check_supervision()
        trace.mark(token, trace.MAVLINK_ROS)
        receive()
    except BaseException:
        trace.fail(token)  # freeze immediately, preserving the failing phase
        raise
    finally:
        trace.end(token)   # allowed once for the open token, even when frozen

On shutdown call ``close`` and only then ``report``. The caller owns any file
serialization. GC events are process-wide: their native thread IDs distinguish
source-thread collection from collection initiated by another Python thread.
"""
import gc
import threading
import time

import numpy as np


class Phase:
    """Integer checkpoint IDs; no enums or dynamic labels in the active loop."""
    SUPERVISION = 1
    MAVLINK_ROS = 2
    ROS_DRAIN = 3
    MONITOR = 4
    ALIVE = 5
    SUPERVISION_SECOND = 6
    READ = 7
    CONTEXT = 8
    SNAPSHOT = 9
    TARGET = 10
    HISTORY = 11
    PUBLISH = 12
    EVIDENCE = 13
    EVIDENCE_DONE = 14
    HANDLE_DONE = 15


class SourceActivityTrace(Phase):
    PHASE_NAMES = (
        'supervision', 'mavlink_ros', 'ros_drain', 'monitor', 'alive',
        'supervision_second', 'read', 'context', 'snapshot', 'target',
        'history', 'publish', 'evidence', 'evidence_done', 'handle_done')
    COLUMNS = (
        'pump_id', 'thread_id', 'begin_ns', 'begin_thread_cpu_ns',
        'end_ns', 'end_thread_cpu_ns', 'failed', 'failure_phase',
        'failure_ns', 'failure_thread_cpu_ns', 'last_phase', 'completed',
    ) + tuple(item for phase in PHASE_NAMES
              for item in (phase + '_begin_ns', phase + '_begin_thread_cpu_ns'))
    GC_COLUMNS = (
        'monotonic_ns', 'thread_cpu_ns', 'thread_id', 'generation',
        'phase', 'pump_id', 'pump_phase', 'collected', 'uncollectable')

    def __init__(self, capacity=131072, gc_capacity=32768, *,
                 clock_ns=time.monotonic_ns, thread_clock_ns=time.thread_time_ns,
                 thread_id=threading.get_native_id):
        if (not isinstance(capacity, int) or isinstance(capacity, bool) or capacity <= 0
                or not isinstance(gc_capacity, int) or isinstance(gc_capacity, bool)
                or gc_capacity <= 0):
            raise ValueError('Trace capacities must be positive integers')
        self.capacity, self.gc_capacity = capacity, gc_capacity
        self._clock, self._thread_clock, self._thread_id = clock_ns, thread_clock_ns, thread_id
        # fill touches every page before activation; np.zeros alone can defer
        # physical page allocation until the first writes in the active loop.
        self._rows = np.empty((capacity, len(self.COLUMNS)), dtype=np.uint64)
        self._gc_rows = np.empty((gc_capacity, len(self.GC_COLUMNS)), dtype=np.uint64)
        self._rows.fill(0)
        self._gc_rows.fill(0)
        self.count = self.attempted = self.overflow = 0
        self.gc_count = self.gc_attempted = self.gc_overflow = 0
        self.duplicate_marks = self.invalid_marks = self.overlapping_pumps = 0
        self.gc_unknown_phases = 0
        self._token = 0
        self._active_row = -1
        self._phase = 0
        self.frozen = self.closed = False
        self.freeze_ns = self.freeze_thread_cpu_ns = self.freeze_thread_id = 0
        self.freeze_pump_id = self.freeze_phase = self.freeze_reason = 0
        self._gc_handle = self._gc_event
        self._gc_registered = False
        self._gc_before = self._gc_after = None

    def start_gc(self):
        """Register exactly one observer, preserving collection policy."""
        if self.closed or self.frozen:
            raise RuntimeError('Cannot start GC observation after freeze/shutdown')
        if self._gc_registered:
            return
        self._gc_before = (gc.isenabled(), gc.get_threshold())
        gc.callbacks.append(self._gc_handle)
        self._gc_registered = True

    def begin(self):
        """Return an integer token; zero means frozen or explicitly overflowed."""
        if self.frozen or self.closed:
            return 0
        self.attempted += 1
        if self._token:
            self.overlapping_pumps += 1
            return 0
        if self.count == self.capacity:
            self.overflow += 1
            return 0
        index = self.count
        self.count += 1
        self._active_row = index
        self._token = self.attempted
        self._phase = 0
        self._rows[index, 0] = self._token
        self._rows[index, 1] = self._thread_id()
        self._rows[index, 2] = self._clock()
        self._rows[index, 3] = self._thread_clock()
        return self._token

    def mark(self, token, phase):
        """Record first entry to a phase without allocating a per-pump graph."""
        if self.frozen or not token or token != self._token:
            return
        if not isinstance(phase, int) or not 1 <= phase <= len(self.PHASE_NAMES):
            self.invalid_marks += 1
            return
        self._phase = phase
        index = self._active_row
        self._rows[index, 10] = phase
        column = 12 + 2 * (phase - 1)
        if self._rows[index, column]:
            self.duplicate_marks += 1
            return
        self._rows[index, column] = self._clock()
        self._rows[index, column + 1] = self._thread_clock()

    def fail(self, token):
        """Record the first failure point, then freeze before recovery begins."""
        if token and token == self._token:
            index = self._active_row
            if not self._rows[index, 6]:
                self._rows[index, 6] = 1
                self._rows[index, 7] = self._phase
                self._rows[index, 8] = self._clock()
                self._rows[index, 9] = self._thread_clock()
        self.freeze(reason=1)

    def end(self, token, *, failed=False):
        """Finalize the open row once, including after fail/freeze."""
        if not token or token != self._token:
            return
        if failed:
            self.fail(token)
        index = self._active_row
        self._rows[index, 4] = self._clock()
        self._rows[index, 5] = self._thread_clock()
        self._rows[index, 11] = 1
        self._token = 0
        self._active_row = -1
        self._phase = 0

    def freeze(self, *, reason=2):
        """Stop new observations; only the current pump's finally may finish.

        Reasons: 0 normal shutdown, 1 exception, 2 external fault/explicit freeze.
        Raw timestamps are preserved; no timestamp, deadline or GC policy is reset.
        """
        if self.frozen:
            return
        self.frozen = True
        self.freeze_ns = self._clock()
        self.freeze_thread_cpu_ns = self._thread_clock()
        self.freeze_thread_id = self._thread_id()
        self.freeze_pump_id, self.freeze_phase = self._token, self._phase
        self.freeze_reason = reason

    def _gc_event(self, phase, info):
        if self.frozen or self.closed:
            return
        if phase != 'start' and phase != 'stop':
            self.gc_unknown_phases += 1
            return
        wall = self._clock()
        cpu = self._thread_clock()
        native_id = self._thread_id()
        self.gc_attempted += 1
        if self.gc_count == self.gc_capacity:
            self.gc_overflow += 1
            return
        index = self.gc_count
        self._gc_rows[index, 0] = wall
        self._gc_rows[index, 1] = cpu
        self._gc_rows[index, 2] = native_id
        self._gc_rows[index, 3] = info['generation']
        self._gc_rows[index, 4] = 1 if phase == 'start' else 2
        self._gc_rows[index, 5] = self._token
        self._gc_rows[index, 6] = self._phase
        self._gc_rows[index, 7] = info.get('collected', 0)
        self._gc_rows[index, 8] = info.get('uncollectable', 0)
        self.gc_count = index + 1

    def close(self):
        """Unregister only this observer; do not collect or change thresholds."""
        if self.closed:
            return
        self.freeze(reason=0)
        if self._gc_registered:
            for index, callback in enumerate(gc.callbacks):
                if callback is self._gc_handle:
                    del gc.callbacks[index]
                    break
            self._gc_registered = False
        self._gc_after = (gc.isenabled(), gc.get_threshold())
        self.closed = True

    def report(self):
        """Materialize owned, JSON-safe output only after shutdown."""
        if not self.closed:
            raise RuntimeError('Source activity is materialized only after close()')
        unfinished = int(np.count_nonzero(self._rows[:self.count, 11] == 0))
        gc_policy_unchanged = self._gc_before is None or self._gc_before == self._gc_after
        return {
            'kind': 'isolated_source_activity_v1', 'schema_version': 1,
            'clock': 'same-host monotonic_ns; thread_cpu uses thread_time_ns',
            'scope': 'observations only; phase stamps are first entry; zero is not reached',
            'thread_id_kind': 'native OS thread ID; GC events may originate in other threads',
            'phases': {str(i + 1): name for i, name in enumerate(self.PHASE_NAMES)},
            'columns': self.COLUMNS, 'rows': self._rows[:self.count].tolist(),
            'capacity': self.capacity, 'recorded': self.count, 'attempted': self.attempted,
            'overflow': self.overflow, 'duplicate_marks': self.duplicate_marks,
            'invalid_marks': self.invalid_marks, 'overlapping_pumps': self.overlapping_pumps,
            'unfinished_records': unfinished,
            'freeze': {'monotonic_ns': self.freeze_ns,
                       'thread_cpu_ns': self.freeze_thread_cpu_ns,
                       'thread_id': self.freeze_thread_id, 'pump_id': self.freeze_pump_id,
                       'phase': self.freeze_phase, 'reason': self.freeze_reason},
            'gc': {'columns': self.GC_COLUMNS, 'rows': self._gc_rows[:self.gc_count].tolist(),
                   'capacity': self.gc_capacity, 'recorded': self.gc_count,
                   'attempted': self.gc_attempted, 'overflow': self.gc_overflow,
                   'unknown_phases': self.gc_unknown_phases,
                   'policy_before': self._gc_before, 'policy_after': self._gc_after,
                   'policy_unchanged': gc_policy_unchanged,
                   'boundary_note': 'Freeze can truncate a start/stop pair; raw events are never paired or synthesized'},
            'complete': not (self.overflow or self.gc_overflow or self.invalid_marks
                             or self.overlapping_pumps or unfinished or self.gc_unknown_phases)
                        and gc_policy_unchanged,
            'allocated_bytes': self._rows.nbytes + self._gc_rows.nbytes,
            'buffers_prefaulted': True,
        }
