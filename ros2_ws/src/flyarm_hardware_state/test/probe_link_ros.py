"""Synthetic ROS test only: isolated domain + localhost, no USB or agent."""
import json
import math
import os
import subprocess
import sys
import threading
import time

assert os.environ.get("ROS_LOCALHOST_ONLY") == "1"
assert os.environ.get("ROS_DOMAIN_ID") == "210"
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleAttitude, VehicleLocalPosition, VehicleOdometry,
                         VehicleStatus, VehicleCommand, VehicleCommandAck)


def topic(name, cls):
    v = cls.MESSAGE_VERSION
    return "/fmu/out/" + name + (f"_v{v}" if v else "")


class MockPX4(Node):
    def __init__(self, case):
        super().__init__("flyarm_mock_px4_only")
        self.case = case
        self.started = time.monotonic()
        self.commands = []
        self.pending = None
        self.bad = []
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
        self.pubs = {cls: self.create_publisher(cls, topic(name, cls), q)
                     for cls, name in ((VehicleAttitude, "vehicle_attitude"),
                                       (VehicleLocalPosition, "vehicle_local_position"),
                                       (VehicleOdometry, "vehicle_odometry"),
                                       (VehicleStatus, "vehicle_status"),
                                       (VehicleCommandAck, "vehicle_command_ack"))}
        self.sub = self.create_subscription(VehicleCommand, "/fmu/in/vehicle_command", self.command, 10)
        self.timer = self.create_timer(.02, self.tick)

    def stamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def command(self, msg):
        self.commands.append(msg)
        if (msg.command != 401 or msg.target_system != 1 or msg.target_component != 1 or
                msg.source_system != 245 or not msg.from_external or msg.confirmation != 0 or
                any(getattr(msg, f"param{i}") != 0 for i in range(1, 8))):
            self.bad.append("unexpected command payload")
        if self.case == "timeout":
            return
        bad = VehicleCommandAck()
        bad.timestamp = self.stamp()
        bad.command = 401
        bad.target_system = msg.source_system
        bad.target_component = msg.source_component + 1
        self.pubs[VehicleCommandAck].publish(bad)
        self.pending = (time.monotonic() + .15, msg)

    def tick(self):
        now = time.monotonic()
        stamp = self.stamp()
        elapsed = now - self.started
        a = VehicleAttitude()
        a.timestamp = a.timestamp_sample = stamp
        a.q = [1., 0., 0., 0.]
        p = VehicleLocalPosition()
        p.timestamp = p.timestamp_sample = stamp
        p.x, p.y, p.z = 2., 3., -1.
        # The first part must be blocked despite finite position values.
        p.xy_valid = p.z_valid = p.v_xy_valid = p.v_z_valid = elapsed >= 12
        p.heading_good_for_control = elapsed >= 12
        o = VehicleOdometry()
        o.timestamp = o.timestamp_sample = stamp
        o.angular_velocity = [1., 2., 3.]
        s = VehicleStatus()
        s.timestamp = stamp
        s.system_id = s.component_id = 1
        s.arming_state = 1
        s.nav_state = 4
        for m in (a, p, o, s):
            self.pubs[type(m)].publish(m)
        if self.pending and now >= self.pending[0]:
            msg = self.pending[1]
            ack = VehicleCommandAck()
            ack.timestamp = stamp
            ack.command = 401
            ack.target_system = msg.source_system
            ack.target_component = msg.source_component
            self.pubs[VehicleCommandAck].publish(ack)
            self.pending = None


def run(binary, case):
    mock = MockPX4(case)
    rows, output = [], []
    mode = "shadow" if case == "shadow" else "diagnostic_ack"
    child = subprocess.Popen([binary, "--ros-args", "-p", "mode:=" + mode,
                              "-p", "duration_s:=30.0", "-p", "settle_s:=5.0"],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    def drain():
        for line in child.stdout:
            output.append(line)
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    reader = threading.Thread(target=drain, daemon=True)
    reader.start()
    try:
        deadline = time.monotonic() + 40
        while child.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(mock, timeout_sec=.02)
        if child.poll() is None:
            raise RuntimeError("mock test wall deadline")
        reader.join(3)
        finals = [r for r in rows if r.get("final")]
        assert len(finals) == 1, output
        final = finals[0]
        assert not mock.bad, mock.bad
        assert final["flight_setpoints_sent"] == final["arm_requests_sent"] == final["offboard_requests_sent"] == 0
        assert not final["mpc_executed"] and not final["flight_ready"]
        if case == "timeout":
            assert child.returncode == 2 and not final["bench_checks_pass"] and not final["complete"]
            assert len(mock.commands) == 1 and not final["ack_received"]
        else:
            assert child.returncode == 0 and final["bench_checks_pass"], output
            assert final["reasons"].get("position_unusable", 0) > 0
            assert final["shadow_hold_candidates"] > 0
            assert final["position_enu_m"] == [3., 2., 1.]
            assert final["omega_flu_rad_s"] == [1., -2., -3.]
            assert len(mock.commands) == (0 if case == "shadow" else 1)
            if case != "shadow":
                assert final["ignored_acks"] >= 1 and final["ack_accepted"]
        print(json.dumps({"mock_case": case, "passed": True, "final": final}), flush=True)
    finally:
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=3)
        mock.destroy_node()


if __name__ == "__main__":
    rclpy.init()
    try:
        for case in ("shadow", "ack", "timeout"):
            run(sys.argv[1], case)
    finally:
        rclpy.shutdown()
