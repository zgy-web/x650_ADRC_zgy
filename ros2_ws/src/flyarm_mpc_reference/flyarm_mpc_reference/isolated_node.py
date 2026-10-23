"""Frozen algorithm worker; no ROS sensor drains or periodic actuator loop."""
import gc
import json
from pathlib import Path
import time
from .factory import create_engine
from .startup import write_startup_metadata
from .node import _shutdown_ros
from .diagnostic_channel import CaptureAdapter, DiagnosticSender


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    from rclpy.serialization import serialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal

    class Worker(Node):
        def __init__(self):
            super().__init__("isolated_frozen_mpc")
            for key in ("repo_root", "task_config", "task_config_sha256", "initialization_manifest",
                        "initialization_sha256", "output_dir", "diagnostic_socket"):
                self.declare_parameter(key, "")
            self.declare_parameter("expected_session_id", 0)
            self.declare_parameter("expected_epoch", 0)
            get = lambda key: self.get_parameter(key).value
            self.out = Path(get("output_dir"))
            self.engine = create_engine(get("repo_root"), get("task_config"), get("task_config_sha256"),
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
                "gc_events": self.gc_events}, indent=2, allow_nan=False) + "\n")

    rclpy.init(args=args)
    node = executor = None
    try:
        node = Worker()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.finish()
        finally:
            _shutdown_ros(node, executor, rclpy)
