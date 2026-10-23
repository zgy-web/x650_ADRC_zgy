#!/usr/bin/env python3
"""Independent 200 Hz typed software stimulus; NOT a simulated physical plant."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, required=True)
    args = p.parse_args()
    cfg = json.loads(args.config.read_text())
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from flyarm_control_interfaces.msg import ControlSnapshot
    from run_x650_ros2_structure_smoke import make_snapshot
    stopped = [False]
    def stop(*unused): stopped[0] = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("isolated_software_state_source")
    pub = node.create_publisher(ControlSnapshot, "/flyarm/isolated/v1/source",
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    (Path(cfg["out"]) / "source_ready").touch()
    seq = 0
    start = None
    records = []
    try:
        while not stopped[0] and not Path(cfg["start_file"]).exists():
            rclpy.spin_once(node, timeout_sec=.005)
        start = time.monotonic_ns()
        due = start + 5_000_000
        while not stopped[0] and rclpy.ok():
            delay = (due - time.monotonic_ns()) / 1e9
            if delay > 0:
                time.sleep(delay)
            now = time.monotonic_ns()
            t = (now - start) / 1e9
            seq += 1
            s = make_snapshot(ControlSnapshot, sequence=seq, session=cfg["session"],
                config_sha=cfg["task_sha256"], target=cfg["target"], gains=cfg["gains"])
            s.control_time_s = 10. + t
            s.task_time_s = s.reference_time_s = t
            s.bus_origin_s = -.025  # fixture bus was already running during prestream
            s.state_time_s = s.joint_time_s = s.control_time_s
            s.command_time_s = [t - .02] + [0.] * 63
            ticks = list(range(math.floor((t - .02)/.005 + 1e-9), math.floor(t/.005 + 1e-9) + 1))
            s.packet_count = len(ticks)
            s.packet_send_time_s = [tick*.005 for tick in ticks] + [0.] * (64-len(ticks))
            s.packet_kp = [6.] * (2*len(ticks)) + [0.] * (128-2*len(ticks))
            s.packet_kd = [.25] * (2*len(ticks)) + [0.] * (128-2*len(ticks))
            # Small nonzero deterministic inputs exercise the frozen solver;
            # they are not the response to its output, so precision/flight
            # metrics must not be computed from this software stimulus.
            w = 2*math.pi/8
            s.position_enu = [.004*math.sin(w*t), .001*math.sin(w*t), 2.+.002*math.sin(w*t)]
            s.velocity_enu = [.004*w*math.cos(w*t), .001*w*math.cos(w*t), .002*w*math.cos(w*t)]
            s.acceleration_enu = [-.004*w*w*math.sin(w*t), -.001*w*w*math.sin(w*t), -.002*w*w*math.sin(w*t)]
            pitch = .002*math.sin(w*t)
            c, h = math.cos(pitch), math.sin(pitch)
            s.rotation_row_major = [c, 0., h, 0., 1., 0., -h, 0., c]
            s.omega_body_flu = [0., .002*w*math.cos(w*t), 0.]
            s.q = [.002*math.sin(w*t), .002*math.sin(w*t)]
            s.dq = [.002*w*math.cos(w*t), .002*w*math.cos(w*t)]
            s.issued_monotonic_ns = time.monotonic_ns()
            pub.publish(s)
            records.append((seq, due, now, s.issued_monotonic_ns))
            due += 5_000_000
            while due <= now:
                due += 5_000_000  # latest-state source; no catch-up burst
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        (Path(cfg["out"]) / "source_final.json").write_text(json.dumps({"pid": os.getpid(),
            "synthetic_software_stimulus": True, "start_ns": start, "rows": records,
            "columns": ["source_sequence", "scheduled_ns", "actual_ns", "issued_ns"]}) + "\n")


if __name__ == "__main__": main()
