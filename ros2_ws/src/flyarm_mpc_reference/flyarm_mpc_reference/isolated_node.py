"""Frozen algorithm worker; no ROS sensor drains or periodic actuator loop."""
import gc
import json
from pathlib import Path
import time
from .factory import create_engine
from .live_factory import create_live_engine
from .startup import write_startup_metadata
from .node import _shutdown_ros
from .diagnostic_channel import CaptureAdapter, DiagnosticSender


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
            self.capture.diagnostic = None
            proposal = self.engine.process(snapshot, ControlProposal)
            # Candidate output precedes diagnostic packaging. No reference is
            # committed by a recorder receipt, nor is a DDS send an actuator ACK.
            self.proposals.publish(proposal)
            self.sender.offer(int(snapshot.sequence), {"snapshot_cdr": serialize_message(snapshot),
                "proposal_cdr": serialize_message(proposal), "diagnostic": self.capture.diagnostic,
                "error": self.engine.last_error})

        def finish(self):
            gc.callbacks.remove(self.gc_event)
            self.sender.close(self.out / "diagnostic_sender.json")
            (self.out / "worker_final.json").write_text(json.dumps({"engine": self.engine.metadata(),
                "gc_enabled_before": self.gc_was_enabled, "gc_enabled_after": gc.isenabled(),
                "preflight_gc": getattr(self,'preflight_gc',None),
                "gc_events": self.gc_events}, indent=2, allow_nan=False) + "\n")

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
