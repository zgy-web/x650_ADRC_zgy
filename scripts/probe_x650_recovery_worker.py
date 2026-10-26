#!/usr/bin/env python3
"""Replay the original 379 CDR inputs through the actual ABI-2 ROS worker.

Only issued_monotonic_ns is refreshed. Recorded states, task times, histories,
session, epoch and task hash are preserved. This private-domain software probe
creates no ingress, dispatcher, simulator, PX4 client or actuator publisher.
The new outcome at 379 is a recorded-state replay, not a counterfactual flight.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from audit_x650_recovery_replay import OUTPUT_DIMENSIONS, strict_json
from export_x650_recovery_recording import plain


def require(ok, why):
    if not ok:
        raise ValueError(why)


def identity(path):
    path = Path(path).resolve(strict=True)
    raw = path.read_bytes()
    return dict(path=str(path), bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())


def sha_argument(value):
    require(isinstance(value, str) and len(value) == 64
            and all(c in "0123456789abcdef" for c in value), "Lowercase SHA256 required")
    return value


def bound(path, digest):
    actual = identity(path)
    require(actual["sha256"] == sha_argument(digest), "Explicit artifact SHA differs: " + str(path))
    return actual


def receipt(value):
    require(isinstance(value, dict) and all(k in value for k in ("path", "bytes", "sha256")),
            "Explicit path/bytes/SHA receipt required")
    actual = bound(value["path"], value["sha256"])
    require(actual["bytes"] == value["bytes"], "Receipt byte size differs")
    return actual


def write_new(path, value):
    with Path(path).open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(plain(value), indent=2, allow_nan=False) + "\n")


def array_error(left, right):
    a, b = np.asarray(left, dtype=float), np.asarray(right, dtype=float)
    require(a.shape == b.shape and np.isfinite(a).all() and np.isfinite(b).all(),
            "Numeric shape or finite-value check failed")
    return float(np.max(np.abs(a-b))) if a.size else 0.


def same_fields(left, right, *, ignored=()):
    """Compare declared fields; CDR serializer padding is not a ROS field."""
    require(type(left) is type(right), "CDR message type differs")
    fields = left.get_fields_and_field_types()
    require(fields == right.get_fields_and_field_types(), "CDR message schema differs")
    for key in fields:
        if key not in ignored:
            require(np.array_equal(getattr(left, key), getattr(right, key)),
                    "CDR declared field differs: " + key)


def message_matches_json(message, value):
    fields = message.get_fields_and_field_types()
    require(set(fields) == set(value), "Decoded original JSON field set differs")
    for key in fields:
        require(np.array_equal(getattr(message, key), value[key]),
                "Original CDR/export field differs: " + key)


def proposal_errors(message, expected):
    if "command" in expected:
        wanted = dict(base_velocity_target=expected["command"][:3],
            joint_velocity=expected["joint_velocity"], position_reference_enu=expected["position_reference"],
            velocity_feedforward_enu=expected["velocity_feedforward"],
            acceleration_feedforward_enu=expected["acceleration_feedforward"])
    else:
        wanted = {k: expected[k] for k in ("base_velocity_target", "joint_velocity", "position_reference_enu",
                  "velocity_feedforward_enu", "acceleration_feedforward_enu")}
    return {key: array_error(getattr(message, key), value) for key, value in wanted.items()}


def cpu_list(text):
    try:
        values = [int(x) for x in text.split(",")]
    except ValueError as error:
        raise argparse.ArgumentTypeError("Comma-separated nonnegative CPU IDs required") from error
    if not values or any(x < 0 for x in values) or len(set(values)) != len(values):
        raise argparse.ArgumentTypeError("Distinct nonnegative auxiliary CPU IDs required")
    return values


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("recording", "startup", "worker", "recovered-report"):
        parser.add_argument("--" + name, type=Path, required=True)
        parser.add_argument("--" + name + "-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain-id", type=int, required=True)
    parser.add_argument("--executor-cpu", type=int, required=True)
    parser.add_argument("--auxiliary-cpus", type=cpu_list, required=True)
    parser.add_argument("--probe-cpu", type=int, required=True)
    args = parser.parse_args(argv)
    require(args.output.is_absolute() and not args.output.exists(), "New absolute evidence directory required")
    args.output.mkdir(parents=True)
    out = args.output
    report = dict(kind="x650_cpp_recovery_actual_worker_probe_v1", schema_version=1,
        complete=False, passed=False, physical_validation=False, physical_acceptance=False,
        counterfactual_closed_loop_claimed=False, injected_delay=False,
        only_refreshed_input_field="issued_monotonic_ns", rows=[], wire_crosscheck=[],
        no_actuator_publishers=False, started_unix=time.time(),
        scope="Original recorded-state CDR prefix through actual ABI-2 Compact ROS worker; no plant or actuator execution")
    child = node = log = rclpy = None
    inputs = []
    try:
        require(1 <= args.domain_id <= 232, "Private DDS domain must be 1..232")
        cpus = [args.executor_cpu, *args.auxiliary_cpus, args.probe_cpu]
        require(min(cpus) >= 0 and len(cpus) == len(set(cpus)), "Probe/executor/auxiliary CPUs must be distinct")
        require(set(cpus) <= os.sched_getaffinity(0), "Requested CPU outside current allowed affinity")
        os.sched_setaffinity(0, {args.probe_cpu})
        os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY="1")
        for name in ("recording", "startup", "worker", "recovered_report"):
            value = bound(getattr(args, name), getattr(args, name + "_sha256"))
            setattr(args, name, Path(value["path"]))
            report[name] = value
            inputs.append(value)
        report["probe"] = identity(__file__)
        inputs.append(report["probe"])
        recording, startup, recovered = (strict_json(getattr(args, n))
                                         for n in ("recording", "startup", "recovered_report"))
        require(recording.get("kind") == "x650_original_cpp_recording_export_v1"
                and recording.get("schema_version") == 1
                and recording.get("originals_verified_unchanged") is True,
                "Bound original recording export required")
        require(recording["config"] == startup["config"] and recording["trajectory"] == startup["trajectory"],
                "New worker must preserve original controller config and trajectory")
        start_values = recording["live_start"]
        require(startup["task_config"]["sha256"] == bytes(start_values["configuration_sha256"]).hex(),
                "Original task hash differs from new startup")
        require(recovered.get("kind") == "x650_cpp_recovery_raw_replay_v1"
                and recovered.get("complete") is True and recovered.get("returned") is True
                and recovered.get("core_abi") == 2 and recovered.get("memory_commit_valid") is True
                and recovered["memory_before"]["step_count"] == 378
                and recovered["memory_after"]["step_count"] == 379,
                "Qualified ABI-2 raw recovered call 379 required")
        rd = recovered["diagnostics"]
        require(rd["recovery_triggered"] is True and rd["recovery_trust_radius"] == .02
                and rd["subproblems"] == 2 and rd["accepted"] and rd["nonlinear_passed"]
                and rd["actual_bounds"] and rd["compute_ms"] <= 50., "Recovered raw evidence differs")
        libraries = {}
        for role, key in (("core", "core_library"), ("optimization", "optimization_library"),
                          ("predictor", "predictor_library")):
            libraries[role] = bound(startup[key]["path"], startup[key]["sha256"])
            inputs.append(libraries[role])
            require(libraries[role]["sha256"] == recovered["loaded_code"][role]["sha256"],
                    "Worker/raw loaded library differs: " + role)
        report["libraries"] = libraries
        for item in (startup["task_config"], *startup["source_artifacts"]):
            inputs.append(bound(item["path"], item["sha256"]))
        source_rows = recording["rows"]
        require(len(source_rows) == 379 and [r["sequence"] for r in source_rows] == list(range(1, 380)),
                "Complete original 1..379 prefix required")
        rows = []
        for seq, row_id in enumerate(source_rows, 1):
            item = receipt(row_id)
            inputs.append(item)
            row = strict_json(item["path"])
            require(row["sequence"] == seq and row["snapshot"]["sequence"] == seq
                    and row["proposal"]["sequence"] == seq, "Original row sequence differs")
            for key in ("snapshot_identity", "proposal_identity", "record_identity"):
                inputs.append(receipt(row[key]))
            require(row["proposal"]["status"] == (0 if seq < 379 else 2), "Original outcome prefix differs")
            for message in (row["snapshot"], row["proposal"]):
                for key in ("session_id", "epoch", "configuration_sha256"):
                    require(message[key] == start_values[key], "Original identity differs: " + key)
            rows.append(row)
        inputs.append(receipt(recording["live_start_cdr_identity"]))
        from rclpy.serialization import serialize_message, deserialize_message
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        import rclpy as ros
        from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal, LiveStart
        rclpy = ros
        rclpy.init()
        node = rclpy.create_node("x650_recovery_private_recorded_probe")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        snapshots = node.create_publisher(ControlSnapshot, "/flyarm/isolated/v1/admitted/snapshot", qos)
        starts = node.create_publisher(LiveStart, "/flyarm/isolated/v1/live_start", 1)
        received = []
        node.create_subscription(ControlProposal, "/flyarm/isolated/v1/output/proposal", received.append, qos)
        worker_dir, wire_dir = out/"worker", out/"probe_records"
        worker_dir.mkdir()
        wire_dir.mkdir()
        log = (out/"child.log").open("xb")
        command = ["taskset", "-c", ",".join(str(x) for x in cpus[:-1]), str(args.worker), "--ros-args",
            "-p", f"startup_manifest:={args.startup}", "-p", f"startup_sha256:={args.startup_sha256}",
            "-p", f"output_dir:={worker_dir}", "-p", f"expected_session_id:={start_values['session_id']}",
            "-p", f"expected_epoch:={start_values['epoch']}", "-p", "simulation_live:=true",
            "-p", f"executor_cpu:={args.executor_cpu}",
            "-p", "auxiliary_cpus:=[" + ",".join(str(x) for x in args.auxiliary_cpus) + "]"]
        report["command"] = command
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def spin_until(condition, timeout, label):
            end = time.monotonic() + timeout
            while not condition():
                if time.monotonic() >= end:
                    raise TimeoutError(label)
                rclpy.spin_once(node, timeout_sec=.001)
                if child.poll() is not None and not condition():
                    raise RuntimeError(label + ": worker exited")

        spin_until(lambda: (worker_dir/"worker_ready").exists()
                   and snapshots.get_subscription_count() == 1 and starts.get_subscription_count() == 1,
                   20., "Actual worker READY/discovery")
        worker_start = strict_json(worker_dir/"worker_startup.json")
        require(worker_start["core_abi"] == 2 and worker_start["diagnostic_mode"] == "compact"
                and worker_start["startup_controller_steps"] == 0, "Worker ABI/Compact/initial state differs")
        pubs = node.get_publisher_names_and_types_by_node("cpp_full_pipeline_mpc", "/")
        pub_names = {name for name, _ in pubs}
        require(pub_names <= {"/flyarm/isolated/v1/output/proposal", "/rosout", "/parameter_events"}
                and "/flyarm/isolated/v1/output/proposal" in pub_names,
                "Unexpected worker actuator/output publisher")
        graph = node.get_node_names_and_namespaces()
        require(set(graph) == {("cpp_full_pipeline_mpc", "/"), (node.get_name(), "/")},
                "Private domain contains another node")
        report.update(no_actuator_publishers=True, publisher_names=sorted(pub_names),
            node_graph=graph, ros_domain_id=args.domain_id, executor_cpu=args.executor_cpu,
            auxiliary_cpus=args.auxiliary_cpus, probe_cpu=args.probe_cpu)
        original_start = deserialize_message(Path(recording["live_start_cdr_identity"]["path"]).read_bytes(), LiveStart)
        message_matches_json(original_start, start_values)
        start = deserialize_message(serialize_message(original_start), LiveStart)
        start.issued_monotonic_ns = time.monotonic_ns()
        same_fields(original_start, start, ignored=("issued_monotonic_ns",))
        start_path = wire_dir/"live_start.sent.cdr"
        start_path.write_bytes(serialize_message(start))
        starts.publish(start)
        until = time.monotonic()+.04
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.001)
        due, first_task = time.monotonic(), rows[0]["snapshot"]["task_time_s"]
        sent_bytes = []
        for index, reference in enumerate(rows):
            sequence = index+1
            target_due = due + reference["snapshot"]["task_time_s"] - first_task
            while time.monotonic() < target_due:
                rclpy.spin_once(node, timeout_sec=min(.001, max(0., target_due-time.monotonic())))
            original = deserialize_message(Path(reference["snapshot_identity"]["path"]).read_bytes(), ControlSnapshot)
            message_matches_json(original, reference["snapshot"])
            original_proposal = deserialize_message(Path(reference["proposal_identity"]["path"]).read_bytes(), ControlProposal)
            message_matches_json(original_proposal, reference["proposal"])
            snapshot = deserialize_message(serialize_message(original), ControlSnapshot)
            snapshot.issued_monotonic_ns = time.monotonic_ns()
            same_fields(original, snapshot, ignored=("issued_monotonic_ns",))
            raw = bytes(serialize_message(snapshot))
            sent_bytes.append(raw)
            sent_path = wire_dir/f"{sequence:06d}.sent_snapshot.cdr"
            sent_path.write_bytes(raw)
            snapshots.publish(snapshot)
            spin_until(lambda: len(received) > index, 1., f"proposal {sequence}")
            got = received[index]
            got_path = wire_dir/f"{sequence:06d}.received_proposal.cdr"
            got_path.write_bytes(serialize_message(got))
            current = dict(sequence=sequence, passed=False, task_time_s=snapshot.task_time_s,
                status=int(got.status), sent_snapshot=identity(sent_path), received_proposal=identity(got_path),
                original_snapshot=reference["snapshot_identity"], original_proposal=reference["proposal_identity"])
            report["rows"].append(current)
            require(got.sequence == sequence and got.snapshot_sequence == sequence
                    and got.session_id == start.session_id and got.epoch == start.epoch
                    and list(got.configuration_sha256) == list(start.configuration_sha256)
                    and got.control_time_s == snapshot.control_time_s and got.task_time_s == snapshot.task_time_s
                    and not got.replay_only and got.schema_version == 1, "Proposal identity differs")
            callback_ms = (got.compute_end_monotonic_ns-got.compute_start_monotonic_ns)/1e6
            require(got.status == 0 and got.accepted_optimum and got.nonlinear_check_passed
                    and math.isfinite(callback_ms) and 0. <= callback_ms <= 50.
                    and math.isfinite(got.primal_violation) and got.primal_violation <= 2e-7
                    and math.isfinite(got.kkt_residual) and got.kkt_residual <= 2e-3,
                    f"Worker rejected/late/nonoptimal original input {sequence}; inspect native record")
            expected = reference["proposal"] if sequence < 379 else recovered["output"]
            errors = proposal_errors(got, expected)
            expected_subproblems = expected["subproblems"] if sequence < 379 else rd["subproblems"]
            require(max(errors.values()) <= 1e-8 and got.subproblems == expected_subproblems,
                    "Worker command/subproblem differs from bound expected output")
            current.update(passed=True, absolute_errors=errors, native_callback_ms=callback_ms,
                subproblems=int(got.subproblems))
        require(len(received) == 379, "Unexpected duplicate proposals")
        child.send_signal(signal.SIGINT)
        child.wait(timeout=8.)
        require(child.returncode == 0, "Normal worker SIGINT/finish failed")
        final = strict_json(worker_dir/"worker_final.json")
        sender = strict_json(worker_dir/"diagnostic_sender.json")
        timing = strict_json(worker_dir/"worker_timing.json")
        require(final["requests"] == final["accepted"] == 379 and not final["faulted"] and final["live_started"],
                "Worker final counters/health differ")
        require(sender["complete"] and sender["written"] == sender["offered"] == 379
                and sender["dropped"] == 0 and timing["complete"] and len(timing["rows"]) == 379,
                "Actual worker recorder/timing incomplete")
        recorded_start_path = worker_dir/"worker_live_start.cdr"
        same_fields(deserialize_message(recorded_start_path.read_bytes(), LiveStart), start)
        report["live_start_crosscheck"] = dict(sent=identity(start_path), received=identity(recorded_start_path),
            original=recording["live_start_cdr_identity"], all_fields_exact=True,
            original_fields_preserved_except_issued_ns=True)
        records = []
        for index, reference in enumerate(rows):
            sequence = index+1
            path = worker_dir/"native_records"/f"{sequence:06d}.json"
            record = strict_json(path)
            require(record["sequence"] == sequence and record["diagnostic_mode"] == "compact"
                    and not record["full_matrix_packet_capture"] and not record["error"], "Native record differs")
            snapshot_receipt, proposal_receipt = receipt(record["snapshot_cdr"]), receipt(record["proposal_cdr"])
            same_fields(deserialize_message(Path(snapshot_receipt["path"]).read_bytes(), ControlSnapshot),
                        deserialize_message(sent_bytes[index], ControlSnapshot))
            same_fields(deserialize_message(Path(proposal_receipt["path"]).read_bytes(), ControlProposal), received[index])
            result = record["result"]
            original_result = strict_json(reference["record_identity"]["path"])
            expected = original_result["result"] if sequence < 379 else recovered["output"]
            errors = {key: array_error(result[key], expected[key]) for key in OUTPUT_DIMENSIONS}
            require(max(errors.values()) <= 1e-8, "Full actual native output differs")
            d = result["diagnostics"]
            require(d["accepted"] and d["nonlinear_passed"] and d["actual_bounds"]
                    and not d["previous_plan_fallback"] and d["compute_ms"] <= 50., "Native candidate check failed")
            if sequence < 379:
                require(d["recovery_triggered"] is False and not d["recovery_attempts"], "Unexpected prefix recovery")
                old_d = expected["diagnostics"]
                diagnostic_errors = {key: array_error(d[key], old_d[key]) for key in
                    ("plan", "predicted_states", "predicted_tip", "predicted_rotations", "candidate_envelope", "horizon")}
                require(max(diagnostic_errors.values()) <= 1e-8, "Original complete native prefix differs")
            else:
                require(d["recovery_triggered"] is True and d["recovery_reason"] == rd["recovery_reason"]
                        and d["recovery_trust_radius"] == .02 and d["subproblems"] == rd["subproblems"]
                        and len(d["recovery_attempts"]) == len(rd["recovery_attempts"]), "Recovery branch evidence differs")
                diagnostic_errors = {key: array_error(d[key], rd[key]) for key in
                    ("plan", "predicted_states", "predicted_tip", "predicted_rotations", "candidate_envelope", "horizon")}
                for actual, saved in zip(d["recovery_attempts"], rd["recovery_attempts"]):
                    for key in ("centre_kind", "rejection", "nonlinear_passed", "original_constraints_passed",
                                "trusted", "projection_attempted", "projection_solved", "substeps"):
                        require(actual[key] == saved[key], "Recovery centre branch differs: "+key)
                    for key in ("centre_plan", "node_times", "node_states", "node_references", "envelope"):
                        diagnostic_errors["recovery_"+key] = array_error(actual[key], saved[key])
                require(max(diagnostic_errors.values()) <= 1e-8, "Recovered native trajectory/centre differs")
            report["rows"][index].update(full_native_output_absolute_errors=errors,
                diagnostic_absolute_errors=diagnostic_errors, recovery_triggered=d["recovery_triggered"])
            report["wire_crosscheck"].append(dict(sequence=sequence, snapshot=snapshot_receipt,
                proposal=proposal_receipt, snapshot_all_fields_exact=True, proposal_all_fields_exact=True))
            records.append(identity(path))
        for item in inputs:
            require(identity(item["path"]) == item, "Bound proof artifact changed during probe")
        report.update(passed=True, test_sequence_complete=True, rows_count=379, records=records,
            final=final, recorder=sender, child_exit=child.returncode,
            timing=identity(worker_dir/"worker_timing.json"), affinity=identity(worker_dir/"worker_affinity_final.json"),
            cdr_comparison="All declared fields exact; independent CDR bytes/SHA retained; serializer alignment padding excluded")
    except Exception as error:
        report["error"] = repr(error)
    finally:
        if child is not None and child.poll() is None:
            try:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:
                    child.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=2.)
            except Exception as error:
                report["cleanup_error"] = repr(error)
                report["passed"] = False
        if child is not None:
            report["child_exit"] = child.poll()
        if log is not None:
            log.close()
        if node is not None:
            try:
                node.destroy_node()
            except Exception as error:
                report["node_cleanup_error"] = repr(error)
                report["passed"] = False
        if rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as error:
                report["ros_cleanup_error"] = repr(error)
                report["passed"] = False
        report.update(complete=True, finished_unix=time.time(),
            test_sequence_complete=report.get("test_sequence_complete", False))
        write_new(out/"report.json", report)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
