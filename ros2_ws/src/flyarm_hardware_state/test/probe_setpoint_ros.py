"""Isolated ROS tests for the bounded disarmed setpoint publisher; no hardware."""
import json
import math
import os
import subprocess
import sys
import threading
import time

assert os.environ.get("ROS_LOCALHOST_ONLY")=="1" and os.environ.get("ROS_DOMAIN_ID")=="210"
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from px4_msgs.msg import VehicleStatus,TrajectorySetpoint,VehicleCommand,OffboardControlMode


class Mock(Node):
    def __init__(self,case):
        super().__init__("setpoint_mock_px4")
        self.case=case;self.rows=[];self.bad=[];self.first=None;self.last_stamp=0
        self.topic="/fmu/out/vehicle_status"+(f"_v{VehicleStatus.MESSAGE_VERSION}" if VehicleStatus.MESSAGE_VERSION else "")
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        self.pub=self.create_publisher(VehicleStatus,self.topic,qos)
        self.sub=self.create_subscription(TrajectorySetpoint,"/fmu/in/trajectory_setpoint",self.receive,10)
        self.forbidden=[self.create_subscription(cls,topic,lambda _:self.bad.append("forbidden publisher"),10)
                        for cls,topic in ((VehicleCommand,"/fmu/in/vehicle_command"),(OffboardControlMode,"/fmu/in/offboard_control_mode"))]
        self.timer=self.create_timer(.05,self.tick)
    def tick(self):
        if self.first and self.case=="stale":return
        m=VehicleStatus();m.timestamp=self.get_clock().now().nanoseconds//1000
        m.system_id=m.component_id=1;m.arming_state=1;m.nav_state=4
        if self.first:
            if self.case=="armed":m.arming_state=2
            if self.case=="offboard":m.nav_state=14
            if self.case=="hil":m.hil_state=1
            if self.case=="regress":m.timestamp=1
        self.pub.publish(m)
    def receive(self,m):
        self.first=self.first or time.monotonic()
        valid=False
        for scale in (1.,-1.,.5):
            p=[-.25*scale,.125*scale,-.5*scale];v=[-.0625*scale,.03125*scale,-.015625*scale];a=[.125*scale,.0625*scale,.03125*scale]
            valid|=list(m.position)==p and list(m.velocity)==v and list(m.acceleration)==a
        if not valid or abs(m.yaw-math.pi/2)>1e-7 or not all(math.isnan(x) for x in m.jerk) or not math.isnan(m.yawspeed):
            self.bad.append("wire field mismatch")
        if m.timestamp<=self.last_stamp:self.bad.append("nonprogressing publication clock")
        self.last_stamp=m.timestamp;self.rows.append(m)


def run(binary,case):
    mock=Mock(case);events=[];lines=[];before_start=[]
    child=subprocess.Popen([binary,"--ros-args","--disable-rosout-logs","-p","approved_bench_probe:=true","-p","startup_budget_s:=4.0"],
                           stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    def drain():
        for line in child.stdout:
            lines.append(line)
            try:e=json.loads(line)
            except ValueError:continue
            events.append(e)
            if e.get("event")=="ready" and case!="no_start":
                before_start.append(len(mock.rows));child.stdin.write("start\n");child.stdin.flush()
    thread=threading.Thread(target=drain,daemon=True);thread.start()
    try:
        deadline=time.monotonic()+24
        while child.poll() is None and time.monotonic()<deadline:rclpy.spin_once(mock,timeout_sec=.02)
        assert child.poll() is not None,"Publisher exceeded process test deadline"
        for _ in range(5):rclpy.spin_once(mock,timeout_sec=.02)
        thread.join(2)
        finals=[x for x in events if x.get("event")=="probe_final"]
        assert len(finals)==1,lines
        f=finals[0];sent=[x for x in events if x.get("event")=="setpoint_sent"]
        assert not mock.bad,mock.bad
        assert not any(before_start) and f["sent"]==len(sent)<=200
        assert f["arm_requests_sent"]==f["offboard_requests_sent"]==f["actuator_commands_sent"]==0
        assert all(0<=x["elapsed_s"]<10 for x in sent)
        if case=="healthy":
            assert child.returncode==0 and f["complete"] and len(mock.rows)>=100,f
            assert all(x>=10 for x in f["pattern_counts"])
            assert {x["pattern"] for x in sent}=={0,1,2}
        else:
            assert child.returncode==2 and not f["complete"],f
            if case=="no_start":assert not mock.rows and not sent
            else:assert 0<len(sent)<50
        print(json.dumps(dict(setpoint_mock_case=case,passed=True,received=len(mock.rows),final=f)),flush=True)
    finally:
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)
        mock.destroy_node()


if __name__=="__main__":
    rclpy.init()
    try:
        for case in ("healthy","no_start","stale","armed","offboard","hil","regress"):run(sys.argv[1],case)
    finally:rclpy.shutdown()
