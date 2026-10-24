"""Offline, deterministic hold24 failure/recovery summary. No ROS/SSH/actuation.

Large original CDR, plugin CSV and ULog were separately checked by review_remote.py
against the source fingerprints recorded in remote_writer_recovery_review.json.
This script rechecks archived original hashes and the summary arithmetic.
"""
import argparse
import hashlib
import json
from pathlib import Path
import statistics


def read(root, name):
    return json.loads((root / name).read_text(encoding="utf-8"))


def table(root, name):
    d = read(root, name)
    return [dict(zip(d["columns"], row)) for row in d["rows"]]


def summarize(root):
    retrieval = read(root, "retrieval_manifest.json")
    copied = {v["path"]: v for v in retrieval["files"]}
    for name, item in copied.items():
        raw = (root / name).read_bytes()
        assert len(raw) == item["bytes"] and hashlib.sha256(raw).hexdigest() == item["sha256"], name
    physical = read(root, "remote_writer_recovery_review.json")
    for name, item in physical["source_files"].items():
        if name in copied:
            assert all(item[k] == copied[name][k] for k in ("source_path", "bytes", "sha256")), name
    fault = physical["first_output_fault"]
    start = read(root, "isolated_live/live_start.json")
    sources = table(root, "isolated_live/source_timing.json")
    ingress = table(root, "isolated_live/ingress_timing.json")
    workers = table(root, "isolated_live/worker_timing.json")
    worker = {row["sequence"]: row for row in workers}
    last_source = sources[-1]
    source_rx = [r for r in ingress if r["kind"] == 1 and r["source_sequence"] == last_source["sequence"]]
    assert len(source_rx) == 1
    rx = source_rx[0]
    assert last_source["sequence"] == fault["source_sequence"] == 965
    assert last_source["issued_ns"] == rx["source_issued_ns"] == worker[247]["snapshot_issued_ns"]
    assert rx["entry_ns"] == rx["source_received_ns"] == fault["source_received_monotonic_ns"]
    assert fault["status"] == 3 and fault["reason"] == 2 and fault["command_sequence"] == 247
    source_age = fault["actual_monotonic_ns"] - last_source["issued_ns"]
    assert source_age > 25_000_000
    assert source_age == fault["command_age_ns"] < 50_000_000
    assert last_source["publish_return_ns"] <= rx["entry_ns"] < fault["actual_monotonic_ns"]

    final = read(root, "isolated_live/worker_final.json")
    events = read(root, "isolated_live/events.json")
    sends = read(root, "isolated_live/sends.json")
    normal = [r for r in sends if not r["stop_zero"]]
    assert final["engine"]["requests"] == final["engine"]["accepted"] == len(workers) == len(events) == 247
    assert not final["engine"]["faulted"] and not final.get("diagnostic_completion_error")
    assert all(e["accepted"] and e["reason"] == 0 for e in events)
    assert [e["proposal_sequence"] for e in events] == list(range(1, 248))
    assert len(normal) == 246 and normal[-1]["command_sequence"] == 246
    assert not any(r["command_sequence"] == 247 for r in normal)
    assert len(sends) == 247 and sends[-1]["stop_zero"]
    assert len(read(root, "isolated_live/receipts.json")) == len(sends)
    phases = []
    finish = []
    for i, row in enumerate(workers):
        stamps = [row[k] for k in ("publish_end_ns", "diagnostic_complete_begin_ns", "diagnostic_complete_end_ns", "snapshot_cdr_begin_ns")]
        assert all(v > 0 for v in stamps) and stamps == sorted(stamps), row["sequence"]
        finish.append((stamps[2] - stamps[1]) / 1e6)
        assert row["snapshot_cdr_begin_ns"] <= row["callback_exit_ns"]
        if i + 1 < len(workers):
            assert workers[i + 1]["callback_entry_ns"] >= row["callback_exit_ns"]
        if row["sequence"] >= 243:
            phases.append({"sequence": row["sequence"],
                "engine_ms": (row["engine_return_ns"] - row["callback_entry_ns"]) / 1e6,
                "callback_ms": (row["callback_exit_ns"] - row["callback_entry_ns"]) / 1e6,
                "post_publish_ms": (row["callback_exit_ns"] - row["publish_end_ns"]) / 1e6,
                "finish_diagnostics_ms": finish[-1]})
    engine = [(r["engine_return_ns"] - r["callback_entry_ns"]) / 1e6 for r in workers]
    callback = [(r["callback_exit_ns"] - r["callback_entry_ns"]) / 1e6 for r in workers]

    before = read(root, "diagnostic_parameters_before.json")
    after = read(root, "diagnostic_parameters_restored.json")
    assert set(before) == set(after)
    fields = ("value", "param_type", "type_name", "raw_value_hex", "source_system", "source_component")
    for name in before:
        assert all(before[name][k] == after[name][k] for k in fields), name
    lifecycle = read(root, "isolated_live/lifecycle.json")
    metadata = read(root, "controller_metadata.json")
    launch = read(root, "launch/launch.json")
    progress = read(root, "progress_v8.json")
    assert not metadata["complete"] and metadata["landed_and_disarmed"]
    assert metadata["diagnostic_parameters_restored"] and not metadata["diagnostic_restore_pending"]
    assert all(p["returncode"] == 0 for p in lifecycle["processes"].values())
    assert launch["runtime_closed"] and launch["sources_unchanged"] and launch["runtime_binaries_unchanged"]
    assert not launch["complete"] and launch["exit_code"] == 2
    assert (root / "hold_v24.exit").read_text().strip() == "1"
    assert not progress["complete"] and not progress["fast_started"] and progress["stage"] == "waiting_for_hold24"
    handoff = metadata["failed_recovery_handoff"]
    assert handoff["confirmed"] and handoff["decision"] == "fold_then_land"
    assert handoff["command"] == 176 and handoff["parameters"][:3] == [1., 4., 3.]
    assert handoff["release"]["writer_exited"] and handoff["release"]["zero_consumed"]
    assert handoff["last_status"]["callback_sequence"] > handoff["before"]["callback_sequence"]
    assert handoff["last_status"]["timestamp_us"] > handoff["before"]["timestamp_us"]
    assert handoff["last_status"]["nav_state"] == 4 and not handoff["last_status"]["failsafe"]
    command_boundary = handoff["command_boundary_wall_monotonic_s"]
    assert handoff["last_status"]["received_wall_monotonic_s"] > command_boundary
    assert any(a["result"] == 0 and a["received_wall_monotonic_s"] > command_boundary
        and a["target_system"] == 1 and a["target_component"] == 1 for a in handoff["acks"])
    confirm_ms = (handoff["confirmed_wall_monotonic_s"] - command_boundary) * 1000
    assert 0 <= confirm_ms <= 350
    ulog = physical["ulog"]
    assert ulog["recovery"]["landed_and_not_armed_at_log_end"]
    assert not any(r["failsafe"] for r in ulog["vehicle_status_changes"])
    hold_commands = [c for c in ulog["vehicle_commands"] if c["command"] == 176 and c["param2"] == 4 and c["param3"] == 3]
    assert len(hold_commands) == 1
    loiter = next(r for r in ulog["vehicle_status_changes"] if r["timestamp"] >= hold_commands[0]["timestamp"] and r["nav_state"] == 4)
    lost = next(r for r in ulog["offboard_signal_lost_changes"] if r["timestamp"] > hold_commands[0]["timestamp"] and r["offboard_control_signal_lost"])
    assert loiter["timestamp"] < lost["timestamp"]
    flight = read(root, "flight_tail.json")
    assert flight["rows"][-1]["armed"] == "0" and flight["rows"][-1]["landed"] == "1"
    supervisor = read(root, "isolated_live/fault_supervision.json")["first_fault"]
    assert supervisor["producer_monotonic_ns"] == fault["actual_monotonic_ns"]
    assert supervisor["producer_to_observation_ns"] == supervisor["observed_monotonic_ns"] - supervisor["producer_monotonic_ns"]
    resources = read(root, "resources.json")["rows"]
    assert resources[0]["monotonic_ns"] <= start["issued_monotonic_ns"] < resources[1]["monotonic_ns"]
    assert resources[-2]["monotonic_ns"] < fault["actual_monotonic_ns"] <= resources[-1]["monotonic_ns"]
    stats = [dict((k, int(v)) for k, v in [line.split() for line in r["cpu.stat"].splitlines()]) for r in resources]
    deltas = {k: stats[-1][k] - stats[0][k] for k in stats[0]}
    span_ns = resources[-1]["monotonic_ns"] - resources[0]["monotonic_ns"]
    assert deltas["nr_throttled"] == deltas["throttled_usec"] == 0
    return {
        "kind": "hold24_independent_failure_review_v1", "full_hold_pass": False,
        "scope": "Failed PX4/Gazebo complete-hold attempt; no hardware or full-hold acceptance.",
        "run": "x650_isolated_live_v1_20261024_124523_18157", "launch": "x650_isolated_launch_20261024_124434_18155",
        "task_sha256": launch["task_sha256"], "execution_revision": launch["protocol"]["execution_revision"],
        "normal_figure8_started_by_progress": False, "fast_figure8_started": False,
        "archived_original_files_verified": sorted(copied),
        "first_writer_fault": {k: v for k, v in fault.items() if k != "command"},
        "fault_name": "STALE_SOURCE", "source_issue_age_ms": source_age / 1e6,
        "source_received_age_ms": (fault["actual_monotonic_ns"] - rx["source_received_ns"]) / 1e6,
        "source_issue_to_ingress_ms": (rx["source_received_ns"] - last_source["issued_ns"]) / 1e6,
        "source_last": last_source, "ingress_last_source": rx,
        "last_source_task_time_s": last_source["control_s"] - start["origin_s"],
        "fault_after_live_start_host_s": (fault["actual_monotonic_ns"] - start["issued_monotonic_ns"]) / 1e9,
        "supervisor_callback_latency_ms": supervisor["producer_to_observation_ns"] / 1e6,
        "source_issue_clock_is_freshness_anchor": True,
        "worker": {"requests": 247, "numerically_accepted": 247, "gate_accepted": 247, "gate_rejected": 0,
            "engine_median_ms": statistics.median(engine), "engine_max_ms": max(engine),
            "callback_median_ms": statistics.median(callback), "callback_max_ms": max(callback),
            "last_normal_sent_candidate": 246, "candidate247_normal_sent": False,
            "all_deferred_diagnostic_phase_orders_verified": True, "diagnostic_completion_error": None,
            "max_finish_diagnostics_ms": max(finish), "last_five": phases},
        "consumption_and_ownership": {k: physical[k] for k in ("all_sends", "normal_sends", "normal_repeats", "all_receipts", "individual_joint_commands", "per_joint_counter_range", "exact_send_receipt_identity_and_raw_joint_counter_value_time_matches", "normal_send_waited_for_previous_actual_receipt", "normal_output_after_output_fault", "stop_zero_send_after_output_fault_ms", "stop_zero_receipt_after_output_fault_ms", "no_legacy_publication_overlap", "legacy_publication_records_checked", "all_materialized_ros_fields_match_original_cdr", "cdr_counts")},
        "recovery": {"zero_consumed": True, "writer_exited": True, "all_six_processes_returned_zero": True,
            "launch_runtime_closed": True, "parameters_restored_exact_value_type_rawhex": len(before),
            "fresh_autonomous_hold_confirmed": True, "hold_confirmation_ms": confirm_ms,
            "release_after_fault_ms": (handoff["release"]["begin_monotonic_ns"] - fault["actual_monotonic_ns"]) / 1e6,
            "release_duration_ms": (handoff["release"]["wall_monotonic_ns"] - handoff["release"]["begin_monotonic_ns"]) / 1e6,
            "ulog_status_rows": ulog["vehicle_status_rows"], "px4_failsafe_observed": False,
            "ulog_recovery": ulog["recovery"], "ulog_vehicle_status_changes": ulog["vehicle_status_changes"],
            "ulog_offboard_signal_lost_changes": ulog["offboard_signal_lost_changes"],
            "offboard_signal_lost_after_autonomous_hold_ms": (lost["timestamp"] - loiter["timestamp"]) / 1000,
            "flight_final_armed": 0, "flight_final_landed": 1},
        "resources": {"bracket_samples": len(resources), "bracket_duration_s": span_ns / 1e9,
            "cgroup_stat_deltas": deltas, "container_average_cpu_cores": deltas["usage_usec"] * 1000 / span_ns,
            "cpu_max": resources[0]["cpu.max"].strip(),
            "limitations": "Container totals rule out added cgroup throttling in this bracket; no per-operation scheduling, CPU-frequency or GIL cause is established here."},
        "evidence_limitations": [
            "Complete raw CDR/plugin/trace/ULog files remain at explicitly hashed server paths; the independent remote checks and excerpts are derived evidence, not original copies.",
            "Supervisor timestamp is callback observation, not DDS network arrival or diagnostic-file write time.",
            "ULog timestamps are original microseconds; no fitted mapping to host monotonic or simulation time is used.",
            "All 247 calculations passed, but only 246 ordinary candidates were sent before the fault; this is not a complete 30 s acceptance.",
            "This review establishes stale-source triggering and successful recorded recovery, not the source-mainloop pause mechanism.",
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
        print("hold24 summary reproduced; original hashes and stated checks matched")
    elif args.output:
        with args.output.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    else:
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
