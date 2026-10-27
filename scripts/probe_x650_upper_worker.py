#!/usr/bin/env python3
"""Exercise the actual upper worker on recorded states in three private DDS domains.

This is software evidence only. ActuatorSend messages in this probe are explicit
synthetic writer notifications, not actuator commands or actual consumption.
There is no gateway, simulator, PX4 client or joint-command publisher. Healthy
state/packet histories remain the original recorded values; a separate unsafe
case deliberately changes q/dq and must be rejected without committing Memory.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

from probe_x650_recovery_worker import (
    identity, receipt, require, write_new, same_fields, message_matches_json, cpu_list)

TOPIC = "/flyarm/isolated/v1/"
RETAGGED = ("session_id", "epoch", "configuration_sha256", "issued_monotonic_ns")


def read(path):
    return json.loads(Path(path).read_text())


def source_rows(recording_path):
    """Verify original bytes, never reconstruct measured states or packet history."""
    recording = read(recording_path)
    require(recording.get("originals_verified_unchanged") is True, "Verified recording required")
    require(recording.get("trajectory") == {"kind": "hold"}, "Hold reference recording required")
    bindings = recording["rows"]
    require(len(bindings) >= 24, "At least 24 complete original snapshots required")
    inputs = [identity(recording_path), receipt(recording["live_start_cdr_identity"])]
    rows = []
    for sequence, binding in enumerate(bindings[:24], 1):
        inputs.append(receipt(binding))
        row = read(binding["path"])
        require(row["sequence"] == row["snapshot"]["sequence"] == sequence,
                "Original sequential snapshot identity differs")
        inputs.append(receipt(row["snapshot_identity"]))
        require(row["snapshot"]["replay_only"] is False, "Original simulation input expected")
        rows.append(row)
    require(rows[-1]["snapshot"]["task_time_s"] + .6 < 2.,
            "Software probe must stay in the original preparation prefix")
    return recording, rows, inputs


def verify_nonlinear(record, tolerance):
    value = record["nmpc"]
    require(value["accepted_by_solver"] is True and value["selected_iteration"] >= 0,
            "Selected nonlinear solver result missing")
    require(value["offline_publication_flag"] is False and value["issue_age_finite"] is True
            and 0 <= value["issue_age_ms"] <= 50.
            and value["actual_stopping_violation"] <= tolerance,
            "Nonlinear admission/clock/stopping evidence failed")
    accepted = [row for row in value["iterations"] if row["accepted"]]
    require(accepted and accepted[-1]["index"] == value["selected_iteration"],
            "Selected nonlinear iteration differs")
    previous = None
    for row in accepted:
        require(row["native_passed"] and row["stopping_passed"], "Unsafe nonlinear candidate selected")
        objective = row["objective"]["total"]
        require(math.isfinite(objective) and objective >= 0, "Nonfinite nonlinear objective")
        if previous is not None:
            require(objective < previous - (1e-10 + 1e-7 * abs(previous)),
                    "Accepted nonlinear objective did not strictly decrease")
        previous = objective
    require(value["selected_objective"]["total"] == previous, "Selected objective/iteration mismatch")


def run_case(args, name, index, startup, options, recording, originals):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.serialization import serialize_message, deserialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal, LiveStart, ActuatorSend

    out = args.output / name
    out.mkdir()
    worker_dir, wire_dir = out / "worker", out / "wire"
    worker_dir.mkdir(); wire_dir.mkdir()
    report = dict(name=name, passed=False, complete=False, physical_acceptance=False,
        no_actuator_publishers=False, software_test_synthetic_writer_receipt=True,
        synthetic_receipt_scope="Software notification of selected output; no actual writer or consumption claim",
        continuous_closed_loop_replay=False, rows=[], synthetic_writer_receipts=[],
        original_state_and_packet_history_preserved=(name != "unsafe_state"),
        permitted_changes=list(RETAGGED) + (["q", "dq"] if name == "unsafe_state" else []))
    child = node = log = context = executor = None
    try:
        domain = args.domain_id + index
        os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
        context = Context()
        rclpy.init(context=context)
        node = rclpy.create_node("x650_upper_private_probe", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
        snapshots = node.create_publisher(ControlSnapshot, TOPIC + "admitted/snapshot", qos)
        starts = node.create_publisher(LiveStart, TOPIC + "live_start", 1)
        notifications = node.create_publisher(ActuatorSend, TOPIC + "actuator_send", qos)
        received = []
        node.create_subscription(ControlProposal, TOPIC + "output/proposal",
                                 lambda msg: received.append((msg, time.monotonic_ns())), qos)
        session = int(time.time_ns() // 1000) + index
        epoch = index + 1
        command = ["taskset", "-c", ",".join(map(str, [args.executor_cpu, *args.auxiliary_cpus])),
            str(args.worker), "--ros-args", "-p", f"startup_manifest:={args.startup}",
            "-p", f"startup_sha256:={identity(args.startup)['sha256']}",
            "-p", f"nmpc_options_manifest:={args.options}",
            "-p", f"nmpc_options_sha256:={identity(args.options)['sha256']}",
            "-p", f"output_dir:={worker_dir}", "-p", f"expected_session_id:={session}",
            "-p", f"expected_epoch:={epoch}", "-p", "simulation_live:=true",
            "-p", "timing_profile:=mpc25_dispatch50_v1", "-p", f"executor_cpu:={args.executor_cpu}",
            "-p", "auxiliary_cpus:=[" + ",".join(map(str, args.auxiliary_cpus)) + "]"]
        report.update(command=command, ros_domain_id=domain, session_id=session, epoch=epoch)
        log = (out / "child.log").open("xb")
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def wait(condition, timeout, label):
            end = time.monotonic() + timeout
            while not condition():
                if time.monotonic() >= end:
                    raise TimeoutError(label + "; child exit=" + str(child.poll()))
                executor.spin_once(timeout_sec=.001)

        def spin_for(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.001)

        wait(lambda: (worker_dir / "worker_ready").exists()
             and snapshots.get_subscription_count() == 1 and starts.get_subscription_count() == 1
             and notifications.get_subscription_count() == 1, 20., "Worker READY/discovery")
        actual = read(worker_dir / "worker_startup.json")
        implementation = "cpp_upper_nmpc_v1" if options["backend"] == "nmpc" else "cpp_callable_mpc_v1"
        require(actual["implementation"] == implementation and actual["startup_manifest"] == startup
                and actual["startup_controller_steps"] == 0 and actual["python_in_worker"] is False
                and actual["nmpc_options_manifest"] == options,
                "Selected worker/startup/options differ")
        for role in ("core", "optimization", "predictor"):
            require(Path(actual["loaded_" + role + "_path"]).resolve()
                    == Path(startup[role + "_library"]["path"]).resolve(), "Loaded library differs: " + role)
        wait(lambda: ("cpp_upper_nmpc", "/") in node.get_node_names_and_namespaces(),
             5., "Worker graph discovery")
        graph = node.get_node_names_and_namespaces()
        require(set(graph) == {("cpp_upper_nmpc", "/"), (node.get_name(), "/")},
                "Unexpected node in private test domain")
        allowed_worker = {TOPIC + "output/proposal", "/rosout", "/parameter_events"}
        worker_pubs = {p for p, _ in node.get_publisher_names_and_types_by_node("cpp_upper_nmpc", "/")}
        probe_pubs = {p for p, _ in node.get_publisher_names_and_types_by_node(node.get_name(), "/")}
        allowed_probe = {TOPIC + "admitted/snapshot", TOPIC + "live_start", TOPIC + "actuator_send",
                         "/rosout", "/parameter_events"}
        require(worker_pubs <= allowed_worker and TOPIC + "output/proposal" in worker_pubs
                and probe_pubs <= allowed_probe, "Unexpected actuator/PX4 publisher")
        report.update(no_actuator_publishers=True, node_graph=graph,
                      worker_publishers=sorted(worker_pubs), probe_publishers=sorted(probe_pubs))
        original_start = deserialize_message(Path(recording["live_start_cdr_identity"]["path"]).read_bytes(), LiveStart)
        message_matches_json(original_start, recording["live_start"])
        start = deserialize_message(serialize_message(original_start), LiveStart)
        start.session_id = session; start.epoch = epoch
        start.configuration_sha256 = list(bytes.fromhex(startup["task_config"]["sha256"]))
        start.issued_monotonic_ns = time.monotonic_ns()
        same_fields(original_start, start, ignored=RETAGGED)
        start_raw = bytes(serialize_message(start))
        starts.publish(start)
        (wire_dir / "live_start.sent.cdr").write_bytes(start_raw)
        spin_for(.03)
        count = 24 if name == "healthy" else 1
        scheduled_start = time.monotonic()
        first_time = originals[0]["snapshot"]["task_time_s"]
        sent_messages, received_messages = [], []
        for ordinal, original in enumerate(originals[:count], 1):
            due = scheduled_start + original["snapshot"]["task_time_s"] - first_time
            while time.monotonic() < due:
                executor.spin_once(timeout_sec=min(.001, max(0., due - time.monotonic())))
            source = deserialize_message(Path(original["snapshot_identity"]["path"]).read_bytes(), ControlSnapshot)
            message_matches_json(source, original["snapshot"])
            message = deserialize_message(serialize_message(source), ControlSnapshot)
            message.session_id = session; message.epoch = epoch
            message.configuration_sha256 = start.configuration_sha256
            if name == "unsafe_state":
                message.q = [startup["config"]["physical_upper"][0] + .1, source.q[1]]
                message.dq = [1., source.dq[1]]
                report["injected_state"] = dict(q=list(message.q), dq=list(message.dq))
            message.issued_monotonic_ns = time.monotonic_ns() - (60_000_000 if name == "stale_issue" else 0)
            same_fields(source, message, ignored=tuple(report["permitted_changes"]))
            raw = bytes(serialize_message(message))
            snapshots.publish(message)
            path = wire_dir / f"{ordinal:06d}.snapshot.sent.cdr"
            path.write_bytes(raw)
            sent_messages.append(raw)
            wait(lambda: len(received) >= ordinal, 2., "Proposal " + str(ordinal))
            got, received_ns = received[ordinal - 1]
            received_messages.append(got)
            got_path = wire_dir / f"{ordinal:06d}.proposal.received.cdr"
            got_path.write_bytes(serialize_message(got))
            row = dict(sequence=ordinal, passed=False, task_time_s=message.task_time_s,
                sent_snapshot=identity(path), received_proposal=identity(got_path),
                original_snapshot=original["snapshot_identity"], status=int(got.status),
                worker_ms=(got.compute_end_monotonic_ns - got.compute_start_monotonic_ns) / 1e6,
                issue_to_result_ms=(got.compute_end_monotonic_ns - message.issued_monotonic_ns) / 1e6,
                issue_to_probe_receive_ms=(received_ns - message.issued_monotonic_ns) / 1e6)
            report["rows"].append(row)
            require(got.sequence == got.snapshot_sequence == ordinal and got.session_id == session
                    and got.epoch == epoch and list(got.configuration_sha256) == list(start.configuration_sha256)
                    and not got.replay_only, "Proposal identity differs")
            if name == "healthy":
                require(got.status == 0 and got.accepted_optimum and got.nonlinear_check_passed
                        and got.control_time_s == message.control_time_s and got.task_time_s == message.task_time_s
                        and 0 <= row["worker_ms"] <= 50. and 0 <= row["issue_to_result_ms"] <= 50.
                        and 0 <= got.primal_violation <= startup["config"]["feasibility_tolerance"]
                        and 0 <= got.kkt_residual <= startup["config"]["kkt_tolerance"],
                        "Healthy worker rejected/late/invalid candidate " + str(ordinal))
                # No actuator client exists here. This synthetic metadata only
                # exercises publication-to-Memory transaction semantics.
                ack = ActuatorSend()
                ack.session_id = session; ack.epoch = epoch
                ack.command_sequence = ordinal; ack.dispatch_sequence = ordinal
                ack.sim_s = message.control_time_s
                ack.expected_counter = [ordinal, ordinal]
                ack.joint_velocity = got.joint_velocity
                ack.coordinate_frame = 1; ack.type_mask = 2048
                p, v = got.position_reference_enu, got.velocity_feedforward_enu
                ack.position_ned = [p[1], p[0], start.ref_alt - p[2]]
                ack.velocity_ned = [v[1], v[0], -v[2]]
                ack.acceleration_ned = [0., 0., 0.]; ack.yaw_ned = start.yaw_ned
                ack.sent_monotonic_ns = time.monotonic_ns()
                ack_raw = bytes(serialize_message(ack))
                notifications.publish(ack)
                ack_path = wire_dir / f"{ordinal:06d}.synthetic_writer_notification.cdr"
                ack_path.write_bytes(ack_raw)
                report["synthetic_writer_receipts"].append(identity(ack_path))
            else:
                require(got.status != 0 and not got.accepted_optimum and not got.nonlinear_check_passed,
                        "Injected bad input was accepted")
                require(all(value == 0. for field in ("position_reference_enu", "velocity_feedforward_enu",
                        "acceleration_feedforward_enu", "base_velocity_target", "joint_velocity")
                        for value in getattr(got, field)), "Rejected output retained executable command")
            row["passed"] = True
        spin_for(.08)
        require(len(received) == count, "Unexpected duplicate or extra proposal")
        if child.poll() is None:
            child.send_signal(signal.SIGTERM)
        child.wait(timeout=8.)
        final = read(worker_dir / "worker_final.json")
        sender = read(worker_dir / "diagnostic_sender.json")
        timing = read(worker_dir / "worker_timing.json")
        healthy = name == "healthy"
        require(final["requests"] == count and final["accepted"] == (count if healthy else 0)
                and final["controller_steps"] == (count if healthy else 0)
                and final["committed_send_sequence"] == (count if healthy else 0)
                and final["pending_send_sequence"] == final["discarded_unsent_sequence"] == 0
                and final["live_started"] is True,
                "Final request/Memory/synthetic-send transaction counters differ")
        require(final["faulted"] is (not healthy) and (child.returncode == 0 if healthy else child.returncode != 0),
                "Worker exit did not match healthy/rejection case")
        require(sender["complete"] and not sender["failed"] and sender["dropped"] == 0
                and sender["offered"] == sender["written"] == count
                and timing["complete"] and timing["dropped"] == 0 and len(timing["rows"]) == count,
                "Worker lost actual callback evidence")
        commits = read(worker_dir / "worker_commit_timing.json")
        require(commits["complete"] and commits["requires_consumer_ack"] is False
                and len(commits["rows"]) == (count if healthy else 0),
                "Synthetic writer notification/Memory commit evidence missing")
        commit_rows = [dict(zip(commits["columns"], row)) for row in commits["rows"]]
        for number, committed in enumerate(commit_rows, 1):
            ack = deserialize_message(Path(report["synthetic_writer_receipts"][number - 1]["path"]).read_bytes(), ActuatorSend)
            require(committed["sequence"] == committed["committed_memory_steps"] == number
                    and committed["dispatch_sequence"] == ack.dispatch_sequence
                    and committed["sent_monotonic_ns"] == ack.sent_monotonic_ns
                    and 0 <= committed["received_monotonic_ns"] - ack.sent_monotonic_ns <= 25_000_000,
                    "Selected Memory did not commit from its exact synthetic writer notification")
        same_fields(deserialize_message((worker_dir / "worker_live_start.cdr").read_bytes(), LiveStart), start)
        files = sorted((worker_dir / "native_records").glob("*.json"))
        require(len(files) == count, "Missing native result/rejection records")
        for index2, path in enumerate(files):
            record = read(path)
            require(record["sequence"] == index2 + 1, "Native record sequence differs")
            require(record["memory_before_step_count"] == (index2 if healthy else 0),
                    "Candidate attempt mutated earlier committed Memory")
            snap = receipt(record["snapshot_cdr"]); prop = receipt(record["proposal_cdr"])
            same_fields(deserialize_message(Path(snap["path"]).read_bytes(), ControlSnapshot),
                        deserialize_message(sent_messages[index2], ControlSnapshot))
            same_fields(deserialize_message(Path(prop["path"]).read_bytes(), ControlProposal), received_messages[index2])
            if healthy:
                require(not record["error"] and record["valid_proposal_published"] is True,
                        "Healthy actual record reports rejection")
                require(record["selected_next_step_count"] == index2 + 1,
                        "Internal proposals were counted as multiple control steps")
                d = record["result"]["diagnostics"]
                require(d["accepted"] and d["nonlinear_passed"] and d["actual_bounds"]
                        and not d["previous_plan_fallback"] and d["hold_guard"]["passed"],
                        "Actual native quality/hold evidence differs")
                if options["backend"] == "nmpc":
                    verify_nonlinear(record, startup["config"]["feasibility_tolerance"])
            else:
                require(record["error"] and record["valid_proposal_published"] is False,
                        "Rejected input lacks original rejection evidence")
            report["rows"][index2].update(native_record=identity(path), raw_wire_crosscheck=True)
        report.update(passed=True, final=final, recorder=sender, worker_timing=identity(worker_dir / "worker_timing.json"),
                      no_actual_consumption_claimed=True, original_histories_not_replaced_by_synthetic_receipts=True)
    except Exception as error:
        report["error"] = repr(error)
        report["traceback"] = traceback.format_exc()
    finally:
        if child is not None and child.poll() is None:
            try:
                child.send_signal(signal.SIGTERM); child.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait(timeout=3.)
                report["cleanup_forced_owned_worker"] = True
                report["passed"] = False
        if child is not None:
            report["child_exit"] = child.poll()
        # Preserve the worker's own cause even when the probe timed out before
        # reaching normal final-evidence validation. This never turns a failed
        # or partial run into a passing result.
        for filename, key in (("worker_final.json", "final"),
                              ("diagnostic_sender.json", "recorder")):
            artifact = worker_dir / filename
            if key not in report and artifact.exists():
                try:
                    report[key] = read(artifact)
                except (OSError, ValueError) as error:
                    report.setdefault("final_evidence_errors", {})[filename] = repr(error)
        if log is not None:
            log.close()
        if executor is not None:
            executor.shutdown(timeout_sec=1.)
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            rclpy.shutdown(context=context)
        report["complete"] = True
        report["artifact_identities"] = [identity(p) for p in sorted(out.rglob("*")) if p.is_file()]
        write_new(out / "report.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("startup", "options", "worker", "output", "recording"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--domain-id", type=int, default=217)
    parser.add_argument("--executor-cpu", type=int, default=4)
    parser.add_argument("--auxiliary-cpus", type=cpu_list, default=[6, 8, 10])
    parser.add_argument("--probe-cpu", type=int, default=18)
    args = parser.parse_args(argv)
    require(args.output.is_absolute() and not args.output.exists(), "New absolute probe output required")
    args.output.mkdir(parents=True)
    report = dict(kind="x650_upper_worker_probe_v1", complete=False, passed=False, physical_acceptance=False,
        physical_validation=False, no_actuator_publishers=False, software_test_synthetic_writer_receipt=True,
        scope="Private-domain recorded-state software test with synthetic writer metadata; no simulation or actual actuator use",
        counterfactual_closed_loop_claimed=False, cases=[])
    inputs = []
    try:
        require(1 <= args.domain_id <= 230, "Three private DDS domains must be in 1..232")
        cpus = [args.executor_cpu, *args.auxiliary_cpus, args.probe_cpu]
        require(len(set(cpus)) == len(cpus) and min(cpus) >= 0 and set(cpus) <= os.sched_getaffinity(0),
                "Distinct available worker/probe CPUs required")
        os.sched_setaffinity(0, {args.probe_cpu})
        for name in ("startup", "options", "worker", "recording"):
            path = getattr(args, name).resolve(strict=True); setattr(args, name, path)
            report[name] = identity(path); inputs.append(report[name])
        startup, options = read(args.startup), read(args.options)
        report["backend"] = options["backend"]
        require(options["backend"] in ("nmpc", "mpc_callable") and startup["trajectory"] == {"kind": "hold"},
                "Probe requires an explicit same-backend hold startup")
        require(options["task_config_sha256"] == startup["task_config"]["sha256"]
                and identity(options["worker_executable"]["path"]) == report["worker"]
                and report["worker"]["sha256"] == options["worker_executable"]["sha256"],
                "Worker/options/task bindings differ")
        for item in [startup["task_config"], *startup["source_artifacts"], *options["source_artifacts"],
                     *[startup[role + "_library"] for role in ("core", "optimization", "predictor")]]:
            actual = identity(item["path"])
            require(actual["sha256"] == item["sha256"], "Bound source/library changed")
            inputs.append(actual)
        inputs.append(identity(Path(__file__)))
        recording, originals, original_inputs = source_rows(args.recording)
        inputs.extend(original_inputs)
        require(recording["config"] == startup["config"], "Probe changed the original prediction model/configuration")
        for index, name in enumerate(("healthy", "stale_issue", "unsafe_state")):
            report["cases"].append(run_case(args, name, index, startup, options, recording, originals))
        for item in inputs:
            require(identity(item["path"]) == item, "Input/source/binary changed during probe")
        report["no_actuator_publishers"] = all(case["no_actuator_publishers"] for case in report["cases"])
        report["passed"] = report["no_actuator_publishers"] and all(case["passed"] for case in report["cases"])
    except Exception as error:
        report["error"] = repr(error)
    finally:
        evidence = {item["path"]: item for item in inputs}
        evidence.update({str(p.resolve()): identity(p) for p in sorted(args.output.rglob("*")) if p.is_file()})
        report.update(complete=True, artifact_identities=list(evidence.values()), finished_unix=time.time())
        write_new(args.output / "report.json", report)
    print(json.dumps({key: report.get(key) for key in ("kind", "passed", "complete", "backend", "error")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
