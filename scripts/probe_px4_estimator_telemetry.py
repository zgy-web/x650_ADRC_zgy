#!/usr/bin/env python3
"""Read-only, wall-time-bounded estimator-topic probe on an existing ground runtime.

Run with the platform's ROS environment sourced. This script does not launch or
stop the runtime, arm, set flight modes, write parameters, or publish commands.
It deliberately does not call ApiNode.close(), whose legacy default publishes
zero joint commands. The JSON is a connectivity diagnostic, never a flight gate.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]

PX4_TYPES = {"position": "VehicleLocalPosition", "attitude": "VehicleAttitude",
             "angular_velocity": "VehicleAngularVelocity", "estimator_flags": "EstimatorStatusFlags",
             "status": "VehicleStatus", "land": "VehicleLandDetected"}
REQUIRED_FIELDS = {
    "position": ("timestamp", "timestamp_sample", "x", "y", "z", "vx", "vy", "vz",
        "xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control", "dead_reckoning",
        "ref_lat", "ref_lon", "ref_alt", "ref_timestamp", "xy_global", "z_global",
        "xy_reset_counter", "z_reset_counter", "heading_reset_counter", "vxy_reset_counter", "vz_reset_counter"),
    "attitude": ("timestamp", "timestamp_sample", "q", "quat_reset_counter"),
    "angular_velocity": ("timestamp", "timestamp_sample", "xyz"),
    "estimator_flags": ("timestamp", "timestamp_sample", "cs_ev_pos", "cs_ev_vel", "cs_ev_hgt",
        "cs_gnss_pos", "cs_gnss_vel", "cs_gps_hgt", "cs_baro_hgt", "cs_inertial_dead_reckoning", "cs_wind_dead_reckoning"),
    "status": ("arming_state", "nav_state", "failsafe"), "land": ("landed",),
    "joints": ("name", "position", "velocity", "header"), "clock": ("clock",)}


def plain(value):
    if isinstance(value, (bool, int, str)) or value is None:
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if hasattr(value, "tolist"):
        return plain(value.tolist())
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if hasattr(value, "sec") and hasattr(value, "nanosec"):
        return {"sec": int(value.sec), "nanosec": int(value.nanosec)}
    if hasattr(value, "stamp"):
        return {"stamp": plain(value.stamp), "frame_id": getattr(value, "frame_id", "")}
    try:
        return [plain(item) for item in value]
    except TypeError:
        return str(value)


class TelemetrySummary:
    """Bounded aggregate statistics; observes ApiNode without invoking methods."""
    def __init__(self, joint_names, started_wall):
        self.started_wall = started_wall
        self.keys = tuple(key for key in REQUIRED_FIELDS if key != "joints" or joint_names)
        self.topics = {}
        self.failure = None
        self.last_clock = None

    def observe(self, node, wall_now):
        if node.estimator_feedback_error:
            self.failure = "ApiNode clock error: "+str(node.estimator_feedback_error)
        clock_s = node.estimator_clock_s
        if clock_s is not None:
            if self.last_clock is not None and clock_s < self.last_clock-1e-9:
                self.failure = "Probe observed simulation-clock regression"
            self.last_clock = clock_s
        for key in self.keys:
            message = node.data.get(key)
            if message is None:
                continue
            sequence = node.callback_sequences.get(key, 0)
            received_wall = node.received.get(key)
            received_sim = node.received_sim.get(key)
            wall_age = None if received_wall is None else wall_now-received_wall
            sim_age = None if clock_s is None or received_sim is None else clock_s-received_sim
            topic = self.topics.setdefault(key, {"first_sequence": sequence, "last_sequence": sequence,
                "first_seen_wall_s": wall_now, "first_received_sim_s": received_sim,
                "new_callback_observations": 0, "max_wall_age_s": 0., "max_sim_age_s": 0.,
                "callbacks_before_clock_observed": 0, "missing_fields": []})
            if sequence < topic["last_sequence"]:
                self.failure = key+" callback sequence regressed"
            if topic["new_callback_observations"] == 0 or sequence > topic["last_sequence"]:
                topic["new_callback_observations"] += 1
                if received_sim is None:
                    topic["callbacks_before_clock_observed"] += 1
            topic.update(last_sequence=sequence, last_seen_wall_s=wall_now,
                received_wall_s=received_wall, received_sim_s=received_sim,
                last_wall_age_s=wall_age, last_sim_age_s=sim_age,
                message_type=type(message).__name__,
                last_fields={name: plain(getattr(message, name)) for name in REQUIRED_FIELDS[key] if hasattr(message, name)})
            topic["missing_fields"] = sorted(set(topic["missing_fields"]) | {
                name for name in REQUIRED_FIELDS[key] if not hasattr(message, name)})
            for name, age in (("wall", wall_age), ("sim", sim_age)):
                if age is not None:
                    topic["max_"+name+"_age_s"] = max(topic["max_"+name+"_age_s"], age)
                    if age < -1e-6:
                        self.failure = key+" has a future "+name+" receipt"
        status, land = node.data.get("status"), node.data.get("land")
        if status is not None and getattr(status, "arming_state", None) == 2:
            self.failure = "Ground-only probe observed an armed vehicle; no command was sent"
        if land is not None and getattr(land, "landed", None) is False:
            self.failure = "Ground-only probe observed a non-landed vehicle; no command was sent"
        return self.failure is None

    def report(self, ended_wall):
        missing = [key for key in self.keys if key not in self.topics]
        no_progress = [key for key, data in self.topics.items() if data["last_sequence"] <= data["first_sequence"]]
        missing_fields = {key: value["missing_fields"] for key, value in self.topics.items() if value["missing_fields"]}
        clockless = [key for key, value in self.topics.items() if value["received_sim_s"] is None]
        stale = [key for key, value in self.topics.items()
                 if value["last_wall_age_s"] is None or value["last_wall_age_s"] > 4.]
        ground = (self.topics.get("status", {}).get("last_fields", {}).get("arming_state") == 1
                  and self.topics.get("land", {}).get("last_fields", {}).get("landed") is True)
        passed = not (self.failure or missing or no_progress or missing_fields or clockless or stale) and ground
        return {"kind": "amsim_px4_estimator_telemetry_probe", "schema_version": 1,
            "diagnostic_only": True, "eligible_as_flight_gate": False, "passed": passed,
            "duration_wall_s": ended_wall-self.started_wall, "clock_source": "ROS /clock receipts",
            "snapshot_protocol": "latest asynchronous callbacks; no source-time synchronization",
            "sim_time_s": self.last_clock, "failure": self.failure, "ground_disarmed_verified": ground,
            "missing_streams": missing, "streams_without_sequence_progress": no_progress,
            "missing_fields": missing_fields, "streams_without_clock_receipt": clockless,
            "wall_stale_streams": stale, "topics": self.topics,
            "commands_published": False, "parameters_written": False, "runtime_started_or_stopped": False}


def probe(duration_wall_s):
    if not math.isfinite(duration_wall_s) or not 1. <= duration_wall_s <= 60.:
        raise ValueError("duration-wall-s must be between 1 and 60 seconds")
    if str(ROOT/"src") not in sys.path:
        sys.path.insert(0, str(ROOT/"src"))
    import rclpy
    from amsim.api.node import ApiNode
    from amsim.runtime.model_metadata import resolve_active_model
    from amsim.runtime.process import status
    runtime = status()
    if runtime.get("running") is not True:
        raise RuntimeError("Probe requires an already running ground runtime; it will not start one")
    model = resolve_active_model(runtime_state=runtime)
    owns_ros = not rclpy.ok()
    if owns_ros:
        rclpy.init()
    node = None
    try:
        node = ApiNode(model["joint_names"])
        node.enable_estimator_feedback()
        started = time.monotonic()
        summary = TelemetrySummary(model["joint_names"], started)
        deadline = started+duration_wall_s
        while time.monotonic() < deadline:
            node.spin_once(timeout_sec=min(.01, max(0., deadline-time.monotonic())))
            if not summary.observe(node, time.monotonic()):
                break
        result = summary.report(time.monotonic())
        result.update(model=model, runtime_pid=runtime.get("pid"), runtime_meta=runtime.get("runtime_meta"),
                      source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        return result
    finally:
        if node is not None:
            # ApiNode.close() includes zero_all(): avoid that command mutation.
            node._executor.shutdown()
            node.destroy_node()
        if owns_ros and rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-wall-s", type=float, default=10.)
    parser.add_argument("--output", type=Path, help="Optional new JSON file; existing files are refused")
    args = parser.parse_args()
    if args.output is not None and args.output.exists():
        parser.error("--output already exists; choose a new diagnostic path")
    result = probe(args.duration_wall_s)
    text = json.dumps(result, indent=2, allow_nan=False)+"\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(text)
    print(text, end="")
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
