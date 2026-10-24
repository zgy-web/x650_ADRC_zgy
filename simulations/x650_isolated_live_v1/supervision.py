"""Small independent fault observer; detailed evidence keeps its own queues.

This observer never sends commands. The first matching C++ fault is sticky:
later WAITING/STOPPED messages cannot erase it. Callback receipt time is local
observation time, not a claim about DDS network arrival.
"""
import time


class FaultLatch:
    def __init__(self, session, epoch, clock=time.monotonic_ns):
        self.session, self.epoch, self.clock = int(session), int(epoch), clock
        self.first = None
        self.counts = {"dispatch": 0, "events": 0, "ignored_identity": 0}

    def observe(self, message, topic):
        observed = self.clock()
        if (int(message.session_id), int(message.epoch)) != (self.session, self.epoch):
            self.counts["ignored_identity"] += 1
            return
        self.counts[topic] += 1
        fault = message.status == 3 if topic == "dispatch" else not message.accepted
        if not fault or self.first is not None:
            return
        emitted = (message.actual_monotonic_ns if topic == "dispatch"
                   else message.received_monotonic_ns)
        self.first = {
            "topic": topic, "session_id": self.session, "epoch": self.epoch,
            "sequence": int(message.sequence), "reason": int(message.reason),
            "observed_monotonic_ns": int(observed),
            "producer_monotonic_ns": int(emitted),
            "producer_to_observation_ns": int(observed) - int(emitted),
        }

    def error(self):
        if self.first is None:
            return None
        kind = "output latched fault" if self.first["topic"] == "dispatch" else "candidate gate rejected"
        return "C++ " + kind + " reason " + str(self.first["reason"])

    def report(self):
        return {"kind": "isolated_fault_supervision_v1", "session_id": self.session,
                "epoch": self.epoch, "first_fault": self.first, "counts": dict(self.counts),
                "semantics": "independent executor, depth 1; first matching fault latched; observer only"}


class FaultSupervisor:
    def __init__(self, context, msg, session, epoch):
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.latch = FaultLatch(session, epoch)
        self.node = Node("flyarm_fault_supervisor", context=context)
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subs = [self.node.create_subscription(typ, "/flyarm/isolated/v1/" + topic,
            lambda m, name=name: self.latch.observe(m, name), qos)
            for typ, topic, name in ((msg.DispatchTick, "dispatch", "dispatch"),
                                     (msg.GateEvent, "event", "events"))]
        self.closed = False

    def poll(self):
        if not self.closed:
            # Always the same wait arguments, so a ready callback iterator can
            # progress fairly across both subscriptions. No serialization here.
            for _ in range(2):
                self.executor.spin_once(timeout_sec=0.)
        return self.latch.error()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=0.)
        self.node.destroy_node()
