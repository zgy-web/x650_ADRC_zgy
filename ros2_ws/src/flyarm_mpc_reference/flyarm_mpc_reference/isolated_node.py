"""Frozen algorithm worker; no ROS sensor drains or periodic actuator loop."""
import gc
from array import array
import json
from pathlib import Path
import time
from .factory import create_engine
from .live_factory import create_live_engine
from .startup import write_startup_metadata
from .node import _shutdown_ros
from .diagnostic_channel import CaptureAdapter, DiagnosticSender


class WorkerTimingTrace:
    """Bounded scalar observations, written only after the executor stops.

    All wall stamps use the same host monotonic clock as proposal/gate stamps.
    Zero denotes a phase never reached; it must not be treated as a duration.
    The callback never writes files, converts JSON, or copies diagnostic data
    for this trace. Overflow is explicit and does not alter control behavior.
    """
    columns = (
        'sequence', 'snapshot_issued_ns', 'callback_entry_ns',
        'engine_return_ns', 'proposal_compute_end_ns',
        'publish_begin_ns', 'publish_end_ns',
        'snapshot_cdr_begin_ns', 'snapshot_cdr_end_ns',
        'proposal_cdr_begin_ns', 'proposal_cdr_end_ns',
        'offer_begin_ns', 'offer_end_ns',
        'pickle_begin_ns', 'pickle_end_ns', 'socket_send_begin_ns',
        'socket_send_end_ns', 'sender_end_ns', 'callback_exit_ns',
        'callback_thread_cpu_begin_ns', 'callback_thread_cpu_end_ns',
        'publish_thread_cpu_begin_ns', 'publish_thread_cpu_end_ns',
        'diagnostic_offered', 'diagnostic_complete_begin_ns',
        'diagnostic_complete_end_ns')

    def __init__(self, capacity=20000):
        self.capacity = capacity
        # Allocate the 4.16 MB default outside the executor. Retaining a new
        # tracked list per callback would enlarge the runtime GC root set.
        self._rows = array('Q', [0]) * (capacity * len(self.columns))
        self.count = 0
        self.dropped = 0

    def append(self, row):
        if self.count < self.capacity:
            offset = self.count * len(self.columns)
            self._rows[offset:offset + len(self.columns)] = array('Q', row)
            self.count += 1
        else:
            self.dropped += 1

    @property
    def rows(self):
        # Materialize Python rows only for final output or offline tests.
        width = len(self.columns)
        return [self._rows[i:i + width].tolist() for i in range(0, self.count * width, width)]

    def write(self, path):
        value = {'kind': 'isolated_worker_phase_timing_v1',
            'clock': 'same-host monotonic_ns; thread_cpu columns use thread_time_ns',
            'scope': 'observation only; does not identify DDS delivery until joined by sequence with gate receipt',
            'columns': self.columns, 'rows': self.rows, 'capacity': self.capacity,
            'dropped': self.dropped, 'complete': self.dropped == 0}
        Path(path).write_text(json.dumps(value, separators=(',', ':'), allow_nan=False) + '\n')


def process_snapshot(engine, capture, publisher, sender, snapshot, proposal_type,
                     serialize, trace, *, clock_ns=time.monotonic_ns,
                     thread_clock_ns=time.thread_time_ns):
    """Preserve proposal-before-recording order and observe its actual phases.

    Message construction and every optimizer/candidate quality check remain
    inside engine.process. The live zero-policy adapter completes its FF
    diagnostics and owned copy only after publish. CDR and immutable pickle
    detach still finish synchronously before the next callback can run.
    """
    entry = clock_ns()
    row = [0] * len(WorkerTimingTrace.columns)
    row[0:3] = (int(snapshot.sequence), int(snapshot.issued_monotonic_ns), entry)
    row[19] = thread_clock_ns()
    try:
        capture.diagnostic = None
        proposal = engine.process(snapshot, proposal_type)
        row[3] = clock_ns()
        row[4] = int(proposal.compute_end_monotonic_ns)
        row[21] = thread_clock_ns()
        row[5] = clock_ns()
        publisher.publish(proposal)
        row[6] = clock_ns()
        row[22] = thread_clock_ns()
        complete = getattr(capture, 'complete_diagnostic', None)
        if complete is not None:
            row[24] = clock_ns()
            try:
                complete(int(snapshot.sequence))
            except BaseException:
                engine.faulted = True
                engine.last_error = 'Post-publication diagnostic completion failed'
                raise
            row[25] = clock_ns()
        row[7] = clock_ns()
        snapshot_cdr = serialize(snapshot)
        row[8] = clock_ns()
        row[9] = clock_ns()
        proposal_cdr = serialize(proposal)
        row[10] = clock_ns()
        row[11] = clock_ns()
        offered = sender.offer(int(snapshot.sequence), {'snapshot_cdr': snapshot_cdr,
            'proposal_cdr': proposal_cdr, 'diagnostic': capture.diagnostic,
            'error': engine.last_error})
        row[12] = clock_ns()
        row[23] = int(offered)
        if sender.last_timing_ns is not None:
            row[13:18] = sender.last_timing_ns
    finally:
        row[20] = thread_clock_ns()
        row[18] = clock_ns()
        trace.append(row)


def collect_preflight_garbage(output, *, collector=gc, clock_ns=time.monotonic_ns):
    """Collect setup cycles once before READY, retaining normal runtime GC.

    Call only after construction frames and startup-provenance temporaries
    have returned. The controller and its genuine state stay live; no object
    list or diagnostic heap snapshot is retained by this report.
    """
    def snapshot():
        return {'enabled': collector.isenabled(), 'counts': list(collector.get_count()),
                'thresholds': list(collector.get_threshold()),
                'stats': collector.get_stats(), 'garbage_count': len(collector.garbage),
                'frozen_count': collector.get_freeze_count()}
    before = snapshot()
    begin = clock_ns()
    collected = collector.collect(2)
    end = clock_ns()
    after = snapshot()
    unchanged = (before['enabled'] == after['enabled']
        and before['thresholds'] == after['thresholds']
        and before['frozen_count'] == after['frozen_count'])
    report = {'kind': 'worker_preflight_full_gc_v1', 'generation': 2,
        'explicit_collection': True, 'before_ready': True,
        'start_monotonic_ns': begin, 'end_monotonic_ns': end,
        'duration_ns': end-begin, 'collected': int(collected),
        'before': before, 'after': after, 'gc_policy_unchanged': unchanged,
        'scope': 'one preflight setup-cycle collection; natural runtime GC remains enabled as observed; no future pause bound'}
    with Path(output).open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    if not unchanged:
        raise RuntimeError('GC policy changed during preflight collection')
    return report


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    from rclpy.serialization import serialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal, LiveStart

    class Worker(Node):
        def __init__(self):
            super().__init__("isolated_frozen_mpc")
            for key in ("repo_root", "task_config", "task_config_sha256", "initialization_manifest",
                        "initialization_sha256", "output_dir", "diagnostic_socket"):
                self.declare_parameter(key, "")
            self.declare_parameter("expected_session_id", 0)
            self.declare_parameter("expected_epoch", 0)
            self.declare_parameter("simulation_live", False)
            get = lambda key: self.get_parameter(key).value
            self.out = Path(get("output_dir"))
            construct = create_live_engine if get("simulation_live") else create_engine
            self.engine = construct(get("repo_root"), get("task_config"), get("task_config_sha256"),
                get("initialization_manifest"), get("initialization_sha256"), get("expected_session_id"), get("expected_epoch"))
            write_startup_metadata(self.out / "worker_startup.json", self.engine)
            self.capture = CaptureAdapter(self.engine.adapter)
            self.engine.adapter = self.capture
            self.sender = DiagnosticSender(get("diagnostic_socket"))
            self.timing = WorkerTimingTrace()
            self.gc_begin, self.gc_events = {}, []
            self.gc_was_enabled = gc.isenabled()
            gc.callbacks.append(self.gc_event)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.proposals = self.create_publisher(ControlProposal, "/flyarm/isolated/v1/output/proposal", qos)
            self.snapshots = self.create_subscription(ControlSnapshot, "/flyarm/isolated/v1/admitted/snapshot", self.process, qos)
            self.start = self.create_subscription(LiveStart,"/flyarm/isolated/v1/live_start",self.begin,1)

        def complete_preflight(self):
            # __init__, factory and provenance frames have returned, and the
            # executor has been attached. No callbacks or solves have run.
            self.preflight_gc = collect_preflight_garbage(self.out / 'worker_preflight_gc.json')
            (self.out / "worker_ready").touch(exist_ok=False)

        def begin(self, start):
            if self.engine.simulation_live:
                self.engine.begin_live(start)

        def gc_event(self, phase, info):
            generation = info["generation"]
            if phase == "start":
                self.gc_begin[generation] = time.monotonic_ns()
            else:
                end = time.monotonic_ns()
                self.gc_events.append({"generation": generation, "start_ns": self.gc_begin.pop(generation, end),
                                       "end_ns": end, "collected": info["collected"]})

        def process(self, snapshot):
            process_snapshot(self.engine, self.capture, self.proposals, self.sender,
                snapshot, ControlProposal, serialize_message, self.timing)

        def finish(self):
            gc.callbacks.remove(self.gc_event)
            self.sender.close(self.out / "diagnostic_sender.json")
            self.timing.write(self.out / "worker_timing.json")
            (self.out / "worker_final.json").write_text(json.dumps({"engine": self.engine.metadata(),
                "gc_enabled_before": self.gc_was_enabled, "gc_enabled_after": gc.isenabled(),
                "preflight_gc": getattr(self,'preflight_gc',None),
                "gc_events": self.gc_events,
                "diagnostic_completion_error": self.capture.completion_error}, indent=2, allow_nan=False) + "\n")

    rclpy.init(args=args)
    node = executor = None
    try:
        node = Worker()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        node.complete_preflight()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.finish()
        finally:
            _shutdown_ros(node, executor, rclpy)
