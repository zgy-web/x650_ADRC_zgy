"""Isolated ROS proof of bounded zero-thrust ownership probe. No USB/Agent."""
import json
import math
import os
import subprocess
import sys
import threading
import time

assert os.environ.get("ROS_LOCALHOST_ONLY") == "1" and os.environ.get("ROS_DOMAIN_ID") == "210"
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleStatus, VehicleAttitude, ManualControlSetpoint,
                         OffboardControlMode, VehicleAttitudeSetpoint, VehicleCommand)


def topic(direction, name, cls):
    return f"/fmu/{direction}/{name}" + (f"_v{cls.MESSAGE_VERSION}" if cls.MESSAGE_VERSION else "")


class Mock(Node):
    def __init__(self, case):
        super().__init__("offboard_mock_only")
        self.case = case
        self.first = None
        self.attitudes = []
        self.ocms = []
        self.errors = []
        self.last_stamp = {}
        self.frozen = None
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
        self.pubs = {cls: self.create_publisher(cls, topic("out", name, cls), q) for name, cls in
                     (("vehicle_status", VehicleStatus), ("vehicle_attitude", VehicleAttitude),
                      ("manual_control_setpoint", ManualControlSetpoint))}
        self.subs = [
            self.create_subscription(OffboardControlMode, "/fmu/in/offboard_control_mode", self.ocm, 10),
            self.create_subscription(VehicleAttitudeSetpoint, topic("in", "vehicle_attitude_setpoint", VehicleAttitudeSetpoint), self.att, 10),
            self.create_subscription(VehicleCommand, "/fmu/in/vehicle_command", lambda _: self.errors.append("forbidden command"), 10),
        ]
        self.timer = self.create_timer(.02, self.tick)

    def receive_stamp(self, name, m):
        if m.timestamp <= self.last_stamp.get(name, 0): self.errors.append("nonprogressing publication stamp")
        self.last_stamp[name] = m.timestamp

    def ocm(self, m):
        self.first = self.first or time.monotonic()
        self.receive_stamp("ocm", m)
        if not m.attitude or any(getattr(m, x) for x in
            ("position", "velocity", "acceleration", "body_rate", "thrust_and_torque", "direct_actuator")):
            self.errors.append("wrong control mode flags")
        self.ocms.append(m)

    def att(self, m):
        self.receive_stamp("att", m)
        if any(m.thrust_body) or m.yaw_sp_move_rate != 0 or any(abs(a-b)>1e-6 for a,b in zip(m.q_d, (1,0,0,0))):
            self.errors.append("wrong attitude/zero-thrust payload")
        self.attitudes.append(m)

    def tick(self):
        elapsed = time.monotonic()-self.first if self.first else -1
        stamp = self.get_clock().now().nanoseconds//1000
        s=VehicleStatus(); s.timestamp=stamp;s.system_id=s.component_id=1;s.arming_state=1;s.nav_state=1
        if elapsed>=.4: s.nav_state=14
        if self.case=="manual_takeover" and 1.1<=elapsed<1.6: s.nav_state=1
        a=VehicleAttitude();a.timestamp=a.timestamp_sample=stamp;a.q=[1.,0.,0.,0.]
        m=ManualControlSetpoint();m.timestamp=stamp;m.valid=True;m.data_source=1;m.throttle=-1.
        if elapsed>=0:
            if self.case=="duplicate":
                self.frozen=self.frozen or stamp;a.timestamp=self.frozen
            if self.case=="regress":a.timestamp=1
            if self.case=="high_throttle":m.throttle=0.
        self.pubs[VehicleStatus].publish(s);self.pubs[ManualControlSetpoint].publish(m)
        if not (elapsed>=0 and self.case=="stale"):self.pubs[VehicleAttitude].publish(a)


def run(binary, case):
    mock=Mock(case);events=[];lines=[]
    scenario="stream_loss" if case=="stream_loss" else "manual_takeover"
    child=subprocess.Popen([binary,"--ros-args","--disable-rosout-logs","--disable-external-lib-logs",
        "-p","approved_power_disconnected_bench:=true","-p","scenario:="+scenario],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    def drain():
        for line in child.stdout:
            lines.append(line)
            try:events.append(json.loads(line))
            except ValueError:pass
    t=threading.Thread(target=drain,daemon=True);t.start()
    deadline=time.monotonic()+18;triggered=False
    try:
        while time.monotonic()<deadline and child.poll() is None:
            rclpy.spin_once(mock,timeout_sec=.01)
            if not triggered and any(e.get("event")=="ready" for e in events):
                assert not mock.ocms and not mock.attitudes, "Published before explicit trigger"
                child.stdin.write("start\n");child.stdin.flush();triggered=True
        assert child.poll() is not None, "Timed out: "+"".join(lines)
        t.join(2)
        final=[e for e in events if e.get("event")=="bench_final"]
        stopped=[e for e in events if e.get("event")=="stream_stopped"]
        expected={"manual_takeover":"px4_left_offboard","stream_loss":"planned_stream_loss"}.get(case,"state_guard")
        assert child.returncode==0 and len(final)==len(stopped)==1, lines
        assert final[0]["reason"]==expected and final[0]["sent"]==stopped[0]["sent"], lines
        assert len(mock.ocms)>0 and len(mock.attitudes)>0 and not mock.errors, mock.errors
        assert final[0]["arm_requests"]==final[0]["mode_requests"]==0
        assert final[0]["offboard_seen"]==(case in ("manual_takeover","stream_loss"))
        print(json.dumps(dict(case=case,passed=True,sent=final[0]["sent"],reason=expected)),flush=True)
    finally:
        if child.poll() is None:child.terminate();child.wait(timeout=3)
        child.stdin.close();t.join(2);mock.destroy_node()


if __name__=="__main__":
    rclpy.init()
    try:
        for case in ("manual_takeover","stream_loss","stale","duplicate","regress","high_throttle"):
            run(sys.argv[1],case)
    finally:rclpy.shutdown()
