#!/usr/bin/env python3
"""Synthetic ROS 2 probe of consumption_source startup and polling lifecycle.

Run in a sourced ROS 2 workspace on Linux. No Gazebo, PX4, actuator or real
plugin CSV is opened. Only this probe's own child PID is stopped/signalled.
The generated CSV and LiveStart are SOFTWARE FIXTURES, never physical proof.

Example:
  python3 scripts/probe_x650_consumption_startup.py --domain-id 218 \
    --binary /ABS/install/flyarm_control_gateway/lib/flyarm_control_gateway/consumption_source \
    --binary-sha256 REVIEWED_BINARY_SHA256 --output /ABS/NEW_PROBE_DIRECTORY
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import traceback
import uuid

TOPIC = "/flyarm/isolated/v1/"
HEADER = "sim_s,upper_sequence1,upper_sequence2,u1,u2,receipt1_s,receipt2_s\n"
POLL_GAP_ERROR = "Consumption reader polling gap exceeds 25 ms"
HASH_ERROR = "Consumption LiveStart identity/origin invalid"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def identity(path):
    path = Path(path).resolve(strict=True)
    return {"path": str(path), "sha256": sha(path), "bytes": path.stat().st_size}


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


class SyntheticCsv:
    """Actual parser columns, deliberately synthetic values and no plant."""
    def __init__(self, path):
        self.path = Path(path)
        self.stream = self.path.open("x", encoding="utf-8", newline="")
        self.stream.write(HEADER); self.stream.flush()
        self.rows, self.pending = [], None

    def begin_row(self, complete=True):
        require(self.pending is None, "Finish the synthetic partial row first")
        index = len(self.rows) + 1
        sim = 10. + index * .004
        row = {"index": index, "sim_s": sim, "counter": [1000 + index, 2000 + index],
               "velocity": [.01 + index % 5 * .001, -.02 - index % 7 * .001],
               "consumed_sim_s": [sim, sim], "complete": bool(complete),
               "write_monotonic_ns": time.monotonic_ns()}
        values = [sim, *row["counter"], *row["velocity"], sim, sim]
        line = ",".join(format(v, ".17g") for v in values)
        self.stream.write(line + ("\n" if complete else "")); self.stream.flush()
        self.rows.append(row)
        if not complete:
            self.pending = row
        return row

    def finish_row(self):
        require(self.pending is not None, "No synthetic partial row to complete")
        self.stream.write("\n"); self.stream.flush()
        self.pending["complete"] = True
        self.pending["completed_monotonic_ns"] = time.monotonic_ns()
        self.pending = None

    def close(self):
        self.stream.close()


def message_record(message):
    return {"received_monotonic_ns": time.monotonic_ns(), "session_id": int(message.session_id),
            "epoch": int(message.epoch), "observed_monotonic_ns": int(message.observed_monotonic_ns),
            "sim_s": float(message.sim_s), "consumed_counter": [int(x) for x in message.consumed_counter],
            "consumed_velocity": [float(x) for x in message.consumed_velocity],
            "consumed_sim_s": [float(x) for x in message.consumed_sim_s]}


def validate_states(states, rows, session, epoch, origin, old_row_limit, issued_ns):
    """Every delivered state must identify a complete, newly written row."""
    by_counter = {tuple(row["counter"]): row for row in rows}
    previous = None
    for state in states:
        require(state["session_id"] == session and state["epoch"] == epoch, "ActuatorState session/epoch mismatch")
        key = tuple(state["consumed_counter"])
        require(key in by_counter, "ActuatorState refers to a row never written by this fixture")
        row = by_counter[key]
        require(row["complete"] and row["index"] > old_row_limit,
                "An old or newline-incomplete row became live actuator state")
        require(state["sim_s"] > origin and math.isclose(state["sim_s"], row["sim_s"], rel_tol=0, abs_tol=1e-12),
                "Published simulation time is not the exact new row")
        for observed, expected in zip(state["consumed_velocity"] + state["consumed_sim_s"],
                                      row["velocity"] + row["consumed_sim_s"]):
            require(math.isfinite(observed) and math.isclose(observed, expected, rel_tol=0, abs_tol=1e-12),
                    "Published consumed values differ from the actual synthetic CSV row")
        require(issued_ns <= state["observed_monotonic_ns"] <= state["received_monotonic_ns"],
                "Observation time predates LiveStart or is in the future")
        if previous is not None:
            require(state["sim_s"] > previous["sim_s"] and state["observed_monotonic_ns"] >= previous["observed_monotonic_ns"],
                    "Delivered consumption states are duplicated/reordered")
        previous = state


class ProbeCase:
    def __init__(self, args, binary, rclpy, msg, deadline, label):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.args, self.binary, self.rclpy, self.msg = args, binary, rclpy, msg
        self.deadline, self.label = deadline, label
        self.folder = args.output / label
        self.folder.mkdir()
        self.ready = self.folder / "consumption_ready"
        self.csv = SyntheticCsv(self.folder / "SYNTHETIC_consumption.csv")
        self.session = int.from_bytes(os.urandom(4), "little") & 0x3fffffff or 1
        self.epoch = 1
        fixture = {"kind": "synthetic_consumption_startup_fixture_v1", "session_id": self.session,
                   "epoch": self.epoch, "case": label, "nonce": uuid.uuid4().hex,
                   "physical_validation": False, "actuator_commands": False}
        self.hash = hashlib.sha256(json.dumps(fixture, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        save(self.folder / "fixture_configuration.json", {**fixture, "configuration_sha256": self.hash})
        self.report = {"case": label, "passed": False, "synthetic_software_fixture_only": True,
                       "session_id": self.session, "epoch": self.epoch, "configuration_sha256": self.hash,
                       "events": [], "states": [], "csv_rows": self.csv.rows}
        self.node = rclpy.create_node("consumption_startup_probe_" + uuid.uuid4().hex[:12])
        self.subscription = self.node.create_subscription(msg.ActuatorState, TOPIC + "actuator_state",
            lambda message: self.report["states"].append(message_record(message)),
            QoSProfile(depth=1024, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.publisher = self.node.create_publisher(msg.LiveStart, TOPIC + "live_start",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.child, self.log = None, None
        self.stopped = False

    def event(self, name, **values):
        self.report["events"].append({"event": name, "monotonic_ns": time.monotonic_ns(), **values})

    def bounded(self):
        require(time.monotonic() < self.deadline, "Global probe timeout expired")

    def pump(self, seconds, *, feed=False, alive=True):
        until, next_row = time.monotonic() + seconds, time.monotonic()
        while time.monotonic() < until:
            self.bounded()
            if alive and self.child is not None:
                require(self.child.poll() is None, "Consumption process exited unexpectedly; inspect case log")
            if feed and time.monotonic() >= next_row:
                self.csv.begin_row(); next_row = time.monotonic() + .004
            self.rclpy.spin_once(self.node, timeout_sec=.001)

    def assert_silent(self, label):
        require(not self.report["states"], "ActuatorState published before new complete live rows: " + label)

    def start_child(self, check_empty_domain=False):
        if check_empty_domain:
            self.pump(.35)
            others = [name for name, namespace in self.node.get_node_names_and_namespaces()
                      if name != self.node.get_name() or namespace != self.node.get_namespace()]
            require(not others and self.publisher.get_subscription_count() == 0
                    and self.node.count_publishers(TOPIC + "actuator_state") == 0,
                    "Requested DDS domain is not empty; choose a separate unused domain")
            self.report["private_domain_initial_graph_empty"] = True
        params = {"simulation_live": True, "expected_session_id": self.session, "expected_epoch": self.epoch,
                  "configuration_sha256": self.hash, "state_csv": str(self.csv.path.resolve()),
                  "ready_file": str(self.ready.resolve())}
        command = [str(self.binary), "--ros-args"]
        for key, value in params.items():
            command += ["-p", key + ":=" + json.dumps(value)]
        if self.args.cpu is not None:
            taskset = shutil.which("taskset")
            require(taskset is not None, "Requested CPU pin requires the existing taskset executable")
            require(self.args.cpu in os.sched_getaffinity(0), "Requested CPU is outside the caller's allowed CPU set")
            command = [taskset, "--cpu-list", str(self.args.cpu), *command]
        self.log = (self.folder / "consumption.log").open("x", encoding="utf-8")
        self.child = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                      env=os.environ.copy(), start_new_session=True)
        self.report.update(command=command, child_pid=self.child.pid, requested_cpu=self.args.cpu)
        self.event("child_started")
        until = time.monotonic() + 8.
        while not self.ready.is_file() or self.publisher.get_subscription_count() < 1:
            self.pump(.004, feed=True)
            self.assert_silent("discovery")
            require(time.monotonic() < until, "Consumption readiness/DDS matching timed out")
        self.report["readiness_text"] = self.ready.read_text(encoding="utf-8")
        require("awaiting LiveStart and new-row barrier" in self.report["readiness_text"],
                "Binary readiness contract is not the reviewed deferred-start implementation")
        self.event("ready", csv_rows=len(self.csv.rows))

    def live_start(self, *, session=None, epoch=None, config_hash=None, origin=None):
        message = self.msg.LiveStart()
        message.session_id = self.session if session is None else session
        message.epoch = self.epoch if epoch is None else epoch
        message.configuration_sha256 = list(bytes.fromhex(self.hash if config_hash is None else config_hash))
        message.origin_s = self.csv.rows[-1]["sim_s"] if origin is None else origin
        # Set immediately before publish, after message construction.
        message.issued_monotonic_ns = time.monotonic_ns()
        self.publisher.publish(message)
        self.event("live_start_published", session_id=int(message.session_id), epoch=int(message.epoch),
                   configuration_sha256=bytes(message.configuration_sha256).hex(),
                   issued_monotonic_ns=int(message.issued_monotonic_ns), origin_s=float(message.origin_s))
        return int(message.issued_monotonic_ns), float(message.origin_s)

    def stop_for(self, seconds, label, *, feed=False):
        require(self.child is not None and self.child.poll() is None, "Own child is not alive for SIGSTOP")
        os.kill(self.child.pid, signal.SIGSTOP)
        self.stopped = True
        until = time.monotonic() + .5
        while True:
            self.bounded()
            require(self.child.poll() is None, "Own child exited before stop observation")
            text = Path(f"/proc/{self.child.pid}/status").read_text()
            if re.search(r"^State:\s+[Tt]", text, re.MULTILINE):
                break
            require(time.monotonic() < until, "SIGSTOP state was not observed")
            self.pump(.001)
        start = time.monotonic_ns()
        self.event(label + "_stopped", stopped_observed_ns=start)
        self.pump(seconds, feed=feed)
        duration_ns = time.monotonic_ns() - start
        require(duration_ns > 25_000_000, "Synthetic stopped interval did not exceed the actual 25 ms guard")
        os.kill(self.child.pid, signal.SIGCONT)
        self.stopped = False
        self.event(label + "_continued", stopped_duration_ns=duration_ns)
        return duration_ns

    def wait_failure(self, expected):
        until = time.monotonic() + 3.
        while self.child.poll() is None:
            self.pump(.002, alive=False)
            require(time.monotonic() < until, "Expected consumption rejection did not terminate process")
        code = self.child.wait(timeout=1.)
        self.log.flush()
        text = (self.folder / "consumption.log").read_text(encoding="utf-8", errors="replace")
        require(code != 0 and expected in text, "Exit does not contain the expected explicit rejection: " + expected)
        self.report.update(expected_rejection=expected, rejection_exit_code=code,
                           exit_observed_before_cleanup=True)
        self.event("expected_rejection_observed", exit_code=code, expected=expected)

    def cleanup(self):
        actions = []
        try:
            if self.child is not None and self.child.poll() is None:
                # A stopped process must be continued to handle SIGTERM. The
                # PID is exclusively the Popen handle created by this case.
                if self.stopped:
                    self.child.send_signal(signal.SIGCONT); actions.append("SIGCONT own PID")
                    self.stopped = False
                self.child.terminate(); actions.append("SIGTERM own PID")
                try:
                    self.child.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    self.child.kill(); actions.append("SIGKILL own PID")
                    self.child.wait(timeout=3.)
        finally:
            try:
                if self.log is not None:
                    self.log.close()
                self.csv.close()
            finally:
                self.node.destroy_node()
            self.report["cleanup_actions"] = actions
            self.report["child_returncode_after_cleanup"] = self.child.returncode if self.child is not None else None
            self.report["files"] = {p.name: identity(p) for p in self.folder.iterdir()
                                    if p.is_file() and p.name != "report.json"}
            save(self.folder / "report.json", self.report)


def primary_case(case):
    case.start_child(check_empty_domain=True)
    case.pump(.08, feed=True); case.assert_silent("ready before identity probes")
    case.live_start(session=case.session + 1)
    case.pump(.04, feed=True); case.assert_silent("wrong session")
    case.live_start(epoch=case.epoch + 1)
    case.pump(.04, feed=True); case.assert_silent("wrong epoch")
    ready_pause = case.stop_for(case.args.pause_s, "ready_pause", feed=True)
    case.pump(.08, feed=True); case.assert_silent("ready after long stopped interval")
    # First observe the READY EOF, then add a small pre-start backlog without
    # pumping the ROS observer. Whether parsed in READY or RESYNC, none of
    # these already-written rows may acquire a fresh live publication.
    case.pump(.02); case.assert_silent("ready EOF")
    for _ in range(5):
        case.csv.begin_row()
    old_limit = len(case.csv.rows)
    case.event("prestart_backlog_written", total_old_rows=old_limit, added_rows=5)
    issued, origin = case.live_start()
    case.report.update(actual_fixture_live_start_issued_ns=issued, origin_s=origin,
                       prestart_old_row_count=old_limit, ready_pause_ns=ready_pause,
                       ready_wrong_session_ignored=True, ready_wrong_epoch_ignored=True)
    case.pump(.08); case.assert_silent("LiveStart EOF resynchronization")
    partial = case.csv.begin_row(complete=False)
    case.pump(.04); case.assert_silent("post-start partial row lacks newline")
    case.csv.finish_row()
    case.event("first_new_row_completed", counter=partial["counter"], sim_s=partial["sim_s"])
    until = time.monotonic() + 3.
    while len(case.report["states"]) < 3:
        case.pump(.004, feed=True)
        require(time.monotonic() < until, "New complete CSV rows did not produce ROS actuator states")
    validate_states(case.report["states"], case.csv.rows, case.session, case.epoch, origin, old_limit, issued)
    case.event("active_new_rows_verified", received_states=len(case.report["states"]))
    active_pause = case.stop_for(case.args.pause_s, "active_pause")
    case.wait_failure(POLL_GAP_ERROR)
    # Drain any valid already-published samples; every received state is checked.
    case.pump(.02, alive=False)
    validate_states(case.report["states"], case.csv.rows, case.session, case.epoch, origin, old_limit, issued)
    case.report.update(passed=True, active_pause_ns=active_pause, zero_prestart_states=True,
        no_old_or_partial_rows_published=True, new_complete_rows_published=True, active_gap_rejected=True,
        semantics="READY may stall without output; LiveStart EOF resync; only later complete synthetic rows become live; ACTIVE retains 25 ms guard")


def wrong_hash_case(case):
    case.start_child()
    case.pump(.04, feed=True); case.assert_silent("wrong-hash setup")
    bad_hash = ("1" if case.hash[0] != "1" else "2") + case.hash[1:]
    case.live_start(config_hash=bad_hash)
    case.wait_failure(HASH_ERROR)
    case.pump(.02, alive=False); case.assert_silent("wrong configuration hash")
    case.report.update(passed=True, wrong_configuration_sha256_rejected=True, zero_states=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain-id", type=int, required=True, help="Explicit unused local DDS domain, 1..232")
    parser.add_argument("--binary", type=Path, help="Absolute consumption_source; defaults to sourced ament package")
    parser.add_argument("--binary-sha256", required=True, help="Reviewed compiled binary SHA256")
    parser.add_argument("--cpu", type=int, help="Optional allowed CPU for the child only, via existing taskset")
    parser.add_argument("--pause-s", type=float, default=.08, help="SIGSTOP interval, 0.05..0.5 s")
    parser.add_argument("--timeout-s", type=float, default=30., help="Global wall-clock deadline, 10..120 s")
    args = parser.parse_args()
    require(sys.platform.startswith("linux"), "This SIGSTOP/ROS probe requires Linux; it is not a local flight runner")
    require(1 <= args.domain_id <= 232, "Choose an explicit nondefault DDS domain between 1 and 232")
    require(re.fullmatch("[0-9a-f]{64}", args.binary_sha256) is not None, "Binary SHA256 must be 64 lowercase hex characters")
    require(math.isfinite(args.pause_s) and .05 <= args.pause_s <= .5, "Pause must deliberately exceed 25 ms and remain bounded")
    require(math.isfinite(args.timeout_s) and 10 <= args.timeout_s <= 120, "Global timeout must be 10..120 s")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    deadline = time.monotonic() + args.timeout_s
    report = {"kind": "x650_consumption_startup_synthetic_probe_v1", "passed": False, "complete": False,
              "synthetic_software_fixture_only": True, "physical_validation": False, "flight_accepted": False,
              "hardware_qualified": False, "eligible_as_stage_gate": False,
              "scope": "ROS 2 consumption-source software lifecycle only; no Gazebo/PX4 or actuator I/O",
              "domain_id": args.domain_id, "localhost_only": True, "timeout_s": args.timeout_s,
              "pause_s": args.pause_s, "started_monotonic_ns": time.monotonic_ns(), "cases": [],
              "probe_source": identity(__file__)}
    rclpy, initialized = None, False
    try:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        import rclpy
        from flyarm_control_interfaces import msg
        if args.binary is None:
            from ament_index_python.packages import get_package_prefix
            args.binary = Path(get_package_prefix("flyarm_control_gateway")) / "lib/flyarm_control_gateway/consumption_source"
        require(args.binary.is_absolute(), "Explicit binary path must be absolute")
        binary = args.binary.resolve(strict=True)
        require(binary.is_file() and os.access(binary, os.X_OK), "Consumption binary is not executable")
        report["binary"] = identity(binary)
        require(report["binary"]["sha256"] == args.binary_sha256, "Compiled consumption binary differs from reviewed SHA256")
        rclpy.init(); initialized = True
        for label, procedure in (("ready_live_active", primary_case), ("wrong_configuration_hash", wrong_hash_case)):
            case = ProbeCase(args, binary, rclpy, msg, deadline, label)
            report["cases"].append(case.report)
            try:
                procedure(case)
            except BaseException as error:
                case.report.update(error=repr(error), traceback=traceback.format_exc())
                raise
            finally:
                case.cleanup()
        require(sha(binary) == args.binary_sha256, "Consumption binary changed during the probe")
        require(sha(__file__) == report["probe_source"]["sha256"], "Probe source changed while running")
        report.update(passed=True, complete=True, binary_unchanged=True)
    except BaseException as error:
        report.update(error=repr(error), traceback=traceback.format_exc())
    finally:
        if initialized and rclpy.ok():
            rclpy.shutdown()
        report["finished_monotonic_ns"] = time.monotonic_ns()
        save(args.output / "report.json", report)
    print(json.dumps({"passed": report["passed"], "complete": report["complete"], "error": report.get("error"),
                      "report": str(args.output / "report.json"), "physical_validation": False}, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
