"""Recompute the small hold23 failure summary from this evidence directory.

This is an offline evidence reader. It does not start ROS, a simulator, or SSH.
The large raw CDR/plugin/ULog checks are separately reproducible with
review_remote.py against the unchanged source paths and SHA-256s.
"""
import argparse
import hashlib
import json
from pathlib import Path


def read(root, name):
    return json.loads((root / name).read_text(encoding="utf-8"))


def table(root, name):
    data = read(root, name)
    return [dict(zip(data["columns"], row)) for row in data["rows"]]


def summarize(root):
    retrieval = read(root, "retrieval_manifest.json")["files"]
    checked = []
    for name, item in retrieval.items():
        path = root / name
        if not path.exists():
            continue
        data = path.read_bytes()
        assert len(data) == item["bytes"]
        assert hashlib.sha256(data).hexdigest() == item["sha256"]
        checked.append(name)
    physical = read(root, "remote_writer_recovery_review.json")
    for name, item in physical["source_files"].items():
        if name in retrieval:
            assert item == retrieval[name], name
    fault = physical["first_output_fault"]
    start = read(root, "isolated_live/live_start.json")
    supervisor = read(root, "isolated_live/fault_supervision.json")["first_fault"]
    workers = table(root, "isolated_live/worker_timing.json")
    worker = {row["sequence"]: row for row in workers}
    dispatch = {row["sequence"]: row for row in physical["dispatch_tail"]}
    sends = read(root, "isolated_live/sends.json")
    send25 = next(row for row in sends if row["command_sequence"] == 25 and not row["stop_zero"])
    events = read(root, "isolated_live/events.json")
    assert len(events) == 25 and all(row["accepted"] and row["reason"] == 0 for row in events)
    assert all(row["proposal_sequence"] == i + 1 for i, row in enumerate(events))
    final = read(root, "isolated_live/worker_final.json")
    assert final["engine"]["requests"] == final["engine"]["accepted"] == len(workers) == 26
    assert not final["engine"]["faulted"]
    assert fault["status"] == 3 and fault["reason"] == 1 and fault["command_sequence"] == 25
    assert fault["command_age_ns"] == fault["actual_monotonic_ns"] - fault["command_issued_monotonic_ns"]
    assert fault["command_age_ns"] > 50_000_000
    assert supervisor["producer_monotonic_ns"] == fault["actual_monotonic_ns"]
    assert supervisor["producer_to_observation_ns"] == supervisor["observed_monotonic_ns"] - supervisor["producer_monotonic_ns"]

    before = read(root, "diagnostic_parameters_before.json")
    after = read(root, "diagnostic_parameters_restored.json")
    assert set(before) == set(after)
    for key in before:
        for field in ("value", "param_type", "type_name", "raw_value_hex"):
            assert before[key][field] == after[key][field], (key, field)
    lifecycle = read(root, "isolated_live/lifecycle.json")
    metadata = read(root, "controller_metadata.json")
    launch = read(root, "launch/launch.json")
    progress = read(root, "progress_v7.json")
    flight = read(root, "flight_tail.json")
    assert flight["source"] == retrieval["flight_samples.csv"]
    assert flight["rows"][-1]["armed"] == "0" and flight["rows"][-1]["landed"] == "1"
    assert metadata["landed_and_disarmed"] and metadata["diagnostic_parameters_restored"]
    assert not metadata["diagnostic_restore_pending"] and not metadata["complete"]
    assert all(item["returncode"] == 0 for item in lifecycle["processes"].values())
    assert launch["runtime_closed"] and launch["sources_unchanged"] and launch["runtime_binaries_unchanged"]
    assert not launch["complete"] and launch["exit_code"] == 2
    assert (root / "hold_v23.exit").read_text().strip() == "1"
    assert not progress["complete"] and not progress["fast_started"]

    resources = read(root, "resource_window.json")
    assert resources["source"] == retrieval["hold_v23_resources.jsonl"]
    rows = resources["rows"]
    assert rows[0]["monotonic_ns"] <= start["issued_monotonic_ns"] < rows[1]["monotonic_ns"]
    assert rows[-2]["monotonic_ns"] < fault["actual_monotonic_ns"] <= rows[-1]["monotonic_ns"]
    stats = [dict((k, int(v)) for k, v in [line.split() for line in row["cpu.stat"].splitlines()]) for row in rows]
    deltas = {key: stats[-1][key] - stats[0][key] for key in stats[0]}
    span_ns = rows[-1]["monotonic_ns"] - rows[0]["monotonic_ns"]
    phases = []
    for sequence in (25, 26):
        row = worker[sequence]
        phases.append({
            "sequence": sequence,
            "source_issue_to_callback_ms": (row["callback_entry_ns"] - row["snapshot_issued_ns"]) / 1e6,
            "engine_wall_ms": (row["engine_return_ns"] - row["callback_entry_ns"]) / 1e6,
            "whole_callback_wall_ms": (row["callback_exit_ns"] - row["callback_entry_ns"]) / 1e6,
            "whole_callback_thread_cpu_ms": (row["callback_thread_cpu_end_ns"] - row["callback_thread_cpu_begin_ns"]) / 1e6,
        })
    ulog = physical["ulog"]
    failsafe = [row for row in ulog["vehicle_status_changes"] if row["failsafe"]]
    assert failsafe and ulog["recovery"]["landed_and_not_armed_at_log_end"]
    return {
        "kind": "hold23_independent_failure_review_v1",
        "scope": "Failed actual PX4/Gazebo hold attempt; not full-window acceptance or hardware qualification.",
        "run": "x650_isolated_live_v1_20261024_121409_13665",
        "launch": "x650_isolated_launch_20261024_121323_13664",
        "full_hold_pass": False,
        "normal_figure8_started_by_progress": False,
        "fast_figure8_started": False,
        "checked_archived_original_files": checked,
        "worker_numerically_accepted": 26,
        "candidate_gate_accepted": 25,
        "candidate_gate_rejected": 0,
        "first_writer_fault": {key: value for key, value in fault.items() if key != "command"},
        "fault_after_live_start_host_s": (fault["actual_monotonic_ns"] - start["issued_monotonic_ns"]) / 1e9,
        "source_receive_age_at_fault_ms": (fault["actual_monotonic_ns"] - fault["source_received_monotonic_ns"]) / 1e6,
        "supervisor_callback_latency_ms": supervisor["producer_to_observation_ns"] / 1e6,
        "worker_phase_measurements": phases,
        "candidate25_engine_return_after_tick3223_scheduled_ms": (worker[25]["engine_return_ns"] - dispatch[3223]["scheduled_monotonic_ns"]) / 1e6,
        "candidate25_publish_end_after_tick3223_scheduled_ms": (worker[25]["publish_end_ns"] - dispatch[3223]["scheduled_monotonic_ns"]) / 1e6,
        "candidate25_gate_receipt_after_tick3223_actual_ms": (events[-1]["received_monotonic_ns"] - dispatch[3223]["actual_monotonic_ns"]) / 1e6,
        "candidate25_age_at_tick3224_decision_ms": dispatch[3224]["command_age_ns"] / 1e6,
        "candidate25_age_at_tick3224_actual_send_ms": (send25["sent_monotonic_ns"] - worker[25]["snapshot_issued_ns"]) / 1e6,
        "candidate26_engine_return_after_fault_ms": (worker[26]["engine_return_ns"] - fault["actual_monotonic_ns"]) / 1e6,
        "candidate26_publish_end_after_fault_ms": (worker[26]["publish_end_ns"] - fault["actual_monotonic_ns"]) / 1e6,
        "dispatch_tail_without_command_payload": [{key: value for key, value in row.items() if key != "command"} for row in physical["dispatch_tail"]],
        "consumption_and_ownership": {key: physical[key] for key in ("all_sends", "normal_sends", "normal_repeats", "all_receipts", "individual_joint_commands", "per_joint_counter_range", "exact_send_receipt_identity_and_raw_joint_counter_value_time_matches", "normal_send_waited_for_previous_actual_receipt", "normal_output_after_output_fault", "stop_zero_send_after_output_fault_ms", "stop_zero_receipt_after_output_fault_ms", "no_legacy_publication_overlap", "legacy_publication_records_checked", "all_materialized_ros_fields_match_original_cdr", "cdr_counts")},
        "resources": {
            "bracket_samples": len(rows), "bracket_duration_s": span_ns / 1e9,
            "cgroup_stat_deltas": deltas, "container_average_cpu_cores": deltas["usage_usec"] * 1000 / span_ns,
            "cpu_max_unchanged": len({row["cpu.max"] for row in rows}) == 1,
            "cpu_max": rows[0]["cpu.max"].strip(),
            "frequency_khz_ranges": {cpu: [min(int(row["frequency_khz"][cpu]) for row in rows), max(int(row["frequency_khz"][cpu]) for row in rows)] for cpu in rows[0]["frequency_khz"]},
            "limitations": "Container totals and coarse frequency samples bracket the active window. They cannot identify the worker's exact core/frequency at each operation or prove scheduler causality.",
        },
        "recovery": {
            "zero_consumed": lifecycle["release"]["zero_consumed"], "writer_exited": lifecycle["release"]["writer_exited"],
            "all_six_processes_returned_zero": True, "launch_runtime_closed": launch["runtime_closed"],
            "parameters_restored_exact_value_type_rawhex": len(before),
            "flight_final_armed": 0, "flight_final_landed": 1,
            "ulog_recovery": ulog["recovery"], "transient_px4_failsafe_present": True,
            "ulog_vehicle_status_changes": ulog["vehicle_status_changes"],
            "ulog_offboard_signal_lost_changes": ulog["offboard_signal_lost_changes"],
            "clock_limit": "ULog timestamps are original microseconds; no fitted alignment with host monotonic or simulation clock.",
        },
        "evidence_limitations": [
            "Large raw CDR, plugin CSV, execution trace and ULog remain at the hashed server paths; their independent checks are derived reports, not copied original bytes.",
            "resource_window.json and flight_tail.json are explicit derived selections, not byte-identical raw source files.",
            "All normal Python worker numerical calls passed, but candidate26 arrived after the writer fault and was not accepted/sent. Numerical acceptance does not establish full hold completion.",
            "Recovery completed with a transient PX4 failsafe. This report does not certify a failsafe-free exit.",
        ],
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verify", type=Path)
    args = parser.parse_args()
    result = summarize(args.source)
    if args.verify:
        assert result == json.loads(args.verify.read_text(encoding="utf-8"))
        print("hold23 summary reproduced; original hashes and stated checks matched")
    elif args.output:
        with args.output.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    else:
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
