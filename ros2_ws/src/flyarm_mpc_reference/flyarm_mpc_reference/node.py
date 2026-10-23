"""Thin ROS2 process: admitted snapshots to offline proposals only."""
from .factory import create_engine
from .startup import write_startup_metadata


def _shutdown_ros(node, executor, rclpy):
    """Release owned resources even if SIGINT already shut down the context."""
    try:
        if executor is not None:
            executor.shutdown()
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal

    class ReferenceNode(Node):
        def __init__(self):
            super().__init__("flyarm_mpc_reference")
            for key in ("repo_root", "task_config", "task_config_sha256", "initialization_manifest", "initialization_sha256"):
                self.declare_parameter(key, "")
            self.declare_parameter("expected_session_id", 1)
            self.declare_parameter("expected_epoch", 1)
            self.declare_parameter("prefix", "/flyarm/offline/v1")
            self.declare_parameter("startup_metadata_path", "")
            get = lambda key: self.get_parameter(key).value
            self.engine = create_engine(get("repo_root"), get("task_config"), get("task_config_sha256"),
                get("initialization_manifest"), get("initialization_sha256"), get("expected_session_id"), get("expected_epoch"))
            prefix = get("prefix")
            if not isinstance(prefix, str) or not prefix.startswith("/") or prefix.endswith("/"):
                raise ValueError("Explicit absolute topic prefix without trailing slash required")
            metadata_path = get("startup_metadata_path")
            if metadata_path:
                write_startup_metadata(metadata_path, self.engine)
            qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
            self.proposals = self.create_publisher(ControlProposal, prefix+"/output/proposal", qos)
            self.snapshots = self.create_subscription(ControlSnapshot, prefix+"/admitted/snapshot", self.process, qos)

        def process(self, snapshot):
            # No complete JSON diagnostics, files, middleware replacement or
            # actuator API on this path. Failed proposals contain zero command.
            self.proposals.publish(self.engine.process(snapshot, ControlProposal))

    rclpy.init(args=args)
    node, executor = None, None
    try:
        node = ReferenceNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        _shutdown_ros(node, executor, rclpy)
