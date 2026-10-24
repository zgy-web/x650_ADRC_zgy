"""Explicit worker thread placement; no optimizer or transport policy changes.

Stage snapshots reject unknown/reused TIDs. They cannot observe short-lived
threads between snapshots. Callback observations concern the executor only;
equal start/end CPU IDs alone do not prove absence of intermediate migration.
"""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import signal
import threading
import time


class WorkerStopRequest:
    """Defer ROS teardown until the main thread has written its final evidence."""
    def __init__(self):
        self.requested = False
        self.previous = {}

    def _request(self, signum, frame):
        self.requested = True

    def install(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            self.previous[signum] = signal.signal(signum, self._request)

    def restore(self):
        for signum, handler in self.previous.items():
            signal.signal(signum, handler)
        self.previous.clear()

    def spin(self, executor, context):
        # A fixed wait duration avoids resetting the ROS ready-callback
        # generator with alternating timeout arguments. No extra thread.
        while not self.requested and context.ok():
            executor.spin_once(timeout_sec=.1)


def _mask(value, label):
    if (not isinstance(value, list) or not value
            or any(type(cpu) is not int or cpu < 0 for cpu in value)
            or value != sorted(set(value))):
        raise ValueError(label + ' must be a nonempty sorted unique CPU list')
    return list(value)


def validate_thread_roles(resource_mask, roles):
    """None preserves the old all-threads-equal-to-process-mask contract."""
    resource_mask = _mask(resource_mask, 'worker resource mask')
    if roles is None:
        return None
    if not isinstance(roles, dict) or set(roles) != {'executor', 'auxiliary'}:
        raise ValueError('worker_thread_roles must declare executor and auxiliary')
    executor = _mask(roles['executor'], 'executor mask')
    auxiliary = _mask(roles['auxiliary'], 'auxiliary mask')
    if len(executor) != 1:
        raise ValueError('executor must have exactly one CPU')
    if set(executor) & set(auxiliary):
        raise ValueError('worker thread roles overlap')
    if sorted(executor + auxiliary) != resource_mask:
        raise ValueError('worker thread roles must exactly partition the worker resource mask')
    return {'executor': executor, 'auxiliary': auxiliary}


def thread_snapshot(pid, *, proc_root=Path('/proc'), affinity=os.sched_getaffinity if hasattr(os, 'sched_getaffinity') else None):
    """Read TID identity before/after affinity to reject exits or PID reuse."""
    if affinity is None:
        raise RuntimeError('Linux thread affinity is required')
    result = {}
    for folder in sorted((Path(proc_root)/str(pid)/'task').iterdir(), key=lambda p: int(p.name)):
        tid = int(folder.name)
        def identity():
            raw = (folder/'stat').read_text()
            end = raw.rfind(')')
            fields = raw[end+2:].split()
            if end < 0 or len(fields) <= 19 or int(raw.split(' ', 1)[0]) != tid:
                raise RuntimeError('Malformed thread identity')
            return int(fields[19]), raw[raw.find('(')+1:end]
        before, comm = identity()
        cpus = sorted(affinity(tid))
        after, _ = identity()
        if before != after:
            raise RuntimeError('Thread identity changed while observing affinity')
        result[str(tid)] = {'start_ticks': before, 'cpus': cpus, 'comm': comm}
    return result


def verify_snapshot(threads, pid, resource_mask, roles, ready_threads=None):
    roles = validate_thread_roles(resource_mask, roles)
    if not isinstance(threads, dict) or not threads or str(pid) not in threads:
        raise ValueError('Worker main TID missing')
    if ready_threads is not None and set(threads) != set(ready_threads):
        raise ValueError('Unknown or missing worker thread at stage boundary')
    for tid, item in threads.items():
        if not tid.isdecimal() or int(tid) <= 0 or type(item.get('start_ticks')) is not int or item['start_ticks'] <= 0:
            raise ValueError('Invalid worker TID/starttime')
        expected = resource_mask if roles is None else roles['executor' if int(tid) == pid else 'auxiliary']
        if item.get('cpus') != expected:
            raise ValueError('Worker thread CPU role differs: ' + tid)
        if ready_threads is not None and item['start_ticks'] != ready_threads[tid]['start_ticks']:
            raise ValueError('Worker thread TID reused: ' + tid)
    return True


class ExecutorCpuObserver:
    """Two syscalls and a TID read per boundary; no /proc walk in callbacks."""
    def __init__(self, pid, executor_mask, *, get_cpu=None,
                 get_tid=threading.get_native_id,
                 get_affinity=os.sched_getaffinity if hasattr(os, 'sched_getaffinity') else None):
        self.pid, self.expected = int(pid), set(executor_mask)
        self.get_tid, self.get_affinity = get_tid, get_affinity
        if get_cpu is None:
            # Keep the GIL for this tiny observation; no work is delegated.
            self.libc = ctypes.PyDLL(None)
            self.get_cpu = self.libc.sched_getcpu
            self.get_cpu.argtypes = []
            self.get_cpu.restype = ctypes.c_int
        else:
            self.get_cpu = get_cpu

    def __call__(self):
        tid = self.get_tid()
        affinity = set(self.get_affinity(0))
        cpu = int(self.get_cpu())
        if tid != self.pid or affinity != self.expected or cpu not in self.expected:
            raise RuntimeError('Executor TID/affinity/observed CPU differs from declaration')
        return tid, cpu


class WorkerAffinity:
    def __init__(self, resource_mask, roles, *, pid=None, snapshot=thread_snapshot,
                 setter=os.sched_setaffinity if hasattr(os, 'sched_setaffinity') else None,
                 clock_ns=time.monotonic_ns):
        self.pid = os.getpid() if pid is None else int(pid)
        self.mask = _mask(resource_mask, 'worker resource mask')
        self.roles = validate_thread_roles(self.mask, roles)
        self.snapshot, self.setter, self.clock_ns = snapshot, setter, clock_ns
        self.source_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.phases, self.ready_threads, self.failure = {}, None, None

    def bind_before_ready(self):
        before = self.snapshot(self.pid)
        # Startup process mask remains the declared union. Never expand an
        # already restricted or unexpectedly placed thread silently.
        verify_snapshot(before, self.pid, self.mask, None)
        if self.roles is not None:
            for tid in before:
                self.setter(int(tid), set(self.roles['executor' if int(tid) == self.pid else 'auxiliary']))
        after = self.snapshot(self.pid)
        if set(before) != set(after) or any(before[t]['start_ticks'] != after[t]['start_ticks'] for t in before):
            raise RuntimeError('Worker thread set changed during placement')
        verify_snapshot(after, self.pid, self.mask, self.roles)
        self.ready_threads = after
        self.phases['bound_before_ready'] = {'monotonic_ns': self.clock_ns(), 'threads': after}

    def check(self, phase):
        if phase in self.phases or self.ready_threads is None:
            raise RuntimeError('Worker affinity stage is duplicate or not initialized')
        try:
            observed = self.snapshot(self.pid)
            self.phases[phase] = {'monotonic_ns': self.clock_ns(), 'threads': observed}
            verify_snapshot(observed, self.pid, self.mask, self.roles, self.ready_threads)
        except BaseException as error:
            self.failure = {'phase': phase, 'error': str(error)}
            raise

    def document(self):
        return {'kind': 'worker_thread_affinity_v1', 'pid': self.pid, 'executor_tid': self.pid,
            'source_sha256': self.source_sha256,
            'resource_mask': self.mask, 'thread_roles': self.roles, 'phases': self.phases,
            'complete': self.failure is None and self.ready_threads is not None,
            'failure': self.failure,
            'scope': 'strict TID/starttime and exact affinity at declared stages; no claim to observe short-lived auxiliary threads between stages; executor boundaries additionally check native TID, singleton affinity and actual CPU'}

    def write(self, path):
        Path(path).write_text(json.dumps(self.document(), indent=2, allow_nan=False)+'\n')


def verify_worker_evidence(document, observed, resource_mask, roles, *, require_final=False):
    """Shared pure check used by handoff and both independent stage auditors."""
    roles = validate_thread_roles(resource_mask, roles)
    if (document.get('kind') != 'worker_thread_affinity_v1' or document.get('complete') is not True
            or document.get('failure') is not None or document.get('resource_mask') != resource_mask
            or document.get('thread_roles') != roles
            or document.get('source_sha256') != hashlib.sha256(Path(__file__).read_bytes()).hexdigest()):
        raise ValueError('Worker placement evidence differs from declaration')
    pid = document.get('pid')
    if type(pid) is not int or pid <= 0 or document.get('executor_tid') != pid:
        raise ValueError('Worker executor identity differs')
    phases = document['phases']
    required = ('bound_before_ready', 'ready', 'live_start', 'final') if require_final else ('bound_before_ready', 'ready')
    if any(name not in phases for name in required):
        raise ValueError('Worker placement stage evidence incomplete')
    ready = phases['bound_before_ready']['threads']
    previous = 0
    for name in required:
        stage = phases[name]
        if type(stage.get('monotonic_ns')) is not int or stage['monotonic_ns'] <= 0 or stage['monotonic_ns'] < previous:
            raise ValueError('Worker placement stage clocks differ')
        previous = stage['monotonic_ns']
        verify_snapshot(stage['threads'], pid, resource_mask, roles, ready)
    verify_snapshot(observed, pid, resource_mask, roles, ready)
    return pid


def verify_execution_resource_evidence(live, resources, protocol, observed):
    """Preserve the legacy exact-mask rule when no thread split is declared."""
    live = Path(live)
    roles = validate_thread_roles(resources['roles']['worker'], protocol.get('worker_thread_roles'))
    if resources['roles'] != protocol['cpu_roles']:
        raise ValueError('Resource roles differ from protocol')
    if set(observed) != {'source', 'worker', 'dispatcher', 'ingress', 'consumption', 'recorder', 'source_recorder'}:
        raise ValueError('Owned resource evidence incomplete')
    for role, threads in observed.items():
        if role == 'worker' and roles is not None:
            continue
        expected = resources['roles'].get(role, resources['background'])
        if not threads or any(cpus != expected for cpus in threads.values()):
            raise ValueError('CPU role differs: ' + role)
    if roles is None:
        return {'verified': True, 'thread_policy': 'legacy_all_threads_full_worker_mask'}
    read = lambda name: json.loads((live/name).read_text())
    ready, final = read('worker_affinity_ready.json'), read('worker_affinity_final.json')
    handoff = read('worker_affinity_at_handoff.json')
    if handoff.get('kind') != 'worker_thread_affinity_handoff_v1':
        raise ValueError('Worker handoff identity evidence missing')
    mask = resources['roles']['worker']
    pid = verify_worker_evidence(ready, handoff['threads'], mask, roles)
    if (verify_worker_evidence(final, handoff['threads'], mask, roles, require_final=True) != pid
            or handoff.get('worker_pid') != pid):
        raise ValueError('Worker process identity differs across stages')
    for phase in ('bound_before_ready', 'ready'):
        if final['phases'][phase] != ready['phases'][phase]:
            raise ValueError('Worker startup evidence changed')
    if (type(handoff.get('monotonic_ns')) is not int
            or not ready['phases']['ready']['monotonic_ns'] <= handoff['monotonic_ns'] <= final['phases']['live_start']['monotonic_ns']):
        raise ValueError('Worker handoff stage clock differs')
    if observed['worker'] != {tid: value['cpus'] for tid, value in handoff['threads'].items()}:
        raise ValueError('Worker raw affinity and identity snapshots disagree')
    timing = read('worker_timing.json')
    columns = timing.get('columns', [])
    required = ('callback_entry_ns', 'callback_exit_ns', 'callback_native_tid_begin', 'callback_actual_cpu_begin',
                'callback_native_tid_end', 'callback_actual_cpu_end', 'executor_affinity_checked')
    if (timing.get('complete') is not True or timing.get('dropped') != 0
            or len(columns) != len(set(columns)) or any(k not in columns for k in required)
            or not timing.get('rows')):
        raise ValueError('Worker executor CPU trace incomplete')
    for raw in timing['rows']:
        if len(raw) != len(columns):
            raise ValueError('Worker CPU trace width differs')
        row = dict(zip(columns, raw))
        if not (final['phases']['live_start']['monotonic_ns'] <= row['callback_entry_ns']
                <= row['callback_exit_ns'] <= final['phases']['final']['monotonic_ns']):
            raise ValueError('Worker callback lies outside placement stage window')
        if (row['executor_affinity_checked'] != 3 or row['callback_native_tid_begin'] != pid
                or row['callback_native_tid_end'] != pid
                or row['callback_actual_cpu_begin'] != roles['executor'][0]
                or row['callback_actual_cpu_end'] != roles['executor'][0]):
            raise ValueError('Worker callback executor placement differs')
    return {'verified': True, 'thread_policy': 'explicit_executor_and_auxiliary_v1',
            'worker_pid': pid, 'executor_cpu': roles['executor'][0],
            'callback_rows': len(timing['rows']), 'stage_thread_count': len(handoff['threads']),
            'scope': 'stage snapshots and executor callback boundary checks; not continuous proof of every auxiliary thread or intermediate CPU sampling'}
