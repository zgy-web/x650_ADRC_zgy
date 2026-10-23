"""Independent finite offline recorder process; contains no control publisher."""
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import serialize_message
from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal, GateEvent, DrainRequest, RecordingReceipt

from .core import RecorderCore, RecorderFailure


def actual_sources():
    package = Path(__file__).resolve().parent
    files = {"flyarm_control_recorder/"+path.name: path for path in package.glob("*.py")}
    for name in ("flyarm_control_recorder", "flyarm_control_interfaces"):
        share = Path(get_package_share_directory(name))
        for path in [share/"package.xml", *sorted((share/"msg").glob("*.msg")),
                     *[share/filename for filename in ("setup.py", "setup.cfg") if (share/filename).exists()]]:
            files[name+"/"+path.relative_to(share).as_posix()] = path
    return files


class RecorderNode(Node):
    def __init__(self):
        super().__init__("flyarm_control_recorder")
        for name, default in (("expected_session_id", -1), ("expected_epoch", -1), ("output_dir", ""),
                              ("prefix", "/flyarm/offline/v1"), ("capacity", 8192), ("drain_grace_s", 2.0)):
            self.declare_parameter(name, default)
        get = lambda name: self.get_parameter(name).value
        prefix = get("prefix").rstrip("/")
        if not isinstance(prefix, str) or not prefix.startswith("/") or not get("output_dir"):
            raise ValueError("Explicit output directory and absolute topic prefix required")
        self.grace_s = float(get("drain_grace_s"))
        if not 0 < self.grace_s <= 30.:
            raise ValueError("drain_grace_s must be in (0,30]")
        self.core = RecorderCore(get("output_dir"), session_id=get("expected_session_id"),
            epoch=get("expected_epoch"), capacity=get("capacity"), source_files=actual_sources(),
            session_metadata={"prefix": prefix, "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                              "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"), "offline_only": True})
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=128,
                         reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.receipts = self.create_publisher(RecordingReceipt, prefix+"/recorder/output/receipt", qos)
        self.subscriptions_owned = [self.create_subscription(cls, prefix+topic,
            lambda message, stream=stream: self.on_record(stream, message), qos)
            for stream, cls, topic in ((0, ControlSnapshot, "/input/snapshot"),
                                      (1, ControlProposal, "/output/proposal"), (2, GateEvent, "/output/gate_event"))]
        self.subscriptions_owned.append(self.create_subscription(DrainRequest, prefix+"/recorder/input/drain", self.on_drain, qos))
        self.deadline = None
        self.timer = self.create_timer(.05, self.on_timer)

    def publish_receipts(self, receipts):
        for receipt in receipts:
            message = RecordingReceipt()
            message.schema_version, message.session_id, message.epoch = 1, self.core.session_id, self.core.epoch
            for field in ("stream", "sequence", "persisted", "complete", "failed"):
                setattr(message, field, getattr(receipt, field))
            message.sha256 = list(receipt.sha256)
            self.receipts.publish(message)

    def on_record(self, stream, message):
        try:
            receipts = self.core.record(stream, serialize_message(message), schema_version=message.schema_version,
                session_id=message.session_id, epoch=message.epoch, sequence=message.sequence,
                replay_only=message.replay_only, configuration_sha256=(bytes(message.configuration_sha256) if stream < 2 else None))
            self.publish_receipts(receipts)
        except RecorderFailure as error:
            self.get_logger().error(str(error))
            self.publish_receipts([error.receipt])

    def on_drain(self, message):
        try:
            receipts = self.core.drain(schema_version=message.schema_version, session_id=message.session_id,
                epoch=message.epoch, replay_only=message.replay_only, final_sequences=[int(value) for value in message.final_sequences])
            self.deadline = time.monotonic()+self.grace_s
            self.publish_receipts(receipts)
        except RecorderFailure as error:
            self.get_logger().error(str(error))
            self.publish_receipts([error.receipt])

    def on_timer(self):
        if self.deadline is not None and not self.core.closed and time.monotonic() >= self.deadline:
            try:
                self.core.expire_drain()
            except RecorderFailure as error:
                self.get_logger().error(str(error))
                self.publish_receipts([error.receipt])


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RecorderNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.core.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
