"""Reproduce the hold21 failure review from this directory's original archive.

Requires Python and numpy only. CDR and ULog extracts are separately bound to
their original input hashes and have accompanying read-only extraction scripts.
Writes failure_report.json in this evidence directory; never starts control.
"""
import base64
from collections import Counter
import csv
import hashlib
import io
import json
from pathlib import Path
import struct
import zipfile

import numpy as np


ROOT = Path(__file__).resolve().parent


def sha(data):
    return hashlib.sha256(data).hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    assert len(a) and np.isfinite(a).all()
    return dict(zip(["minimum", "p50", "p95", "p99", "maximum"],
                    map(float, np.percentile(a, [0, 50, 95, 99, 100]))))


def same(left, right, tolerance=1e-10):
    a, b = np.asarray(left), np.asarray(right)
    return a.shape == b.shape and np.isfinite(a).all() and np.isfinite(b).all() and np.max(abs(a - b)) <= tolerance


def main():
    manifest = json.loads((ROOT / "manifest.json").read_text())
    with zipfile.ZipFile(ROOT / "original_sources.zip") as z:
        assert set(z.namelist()) == set(manifest["archive_entries"])
        raw = {name: z.read(name) for name in z.namelist()}
    for name, data in raw.items():
        entry = manifest["archive_entries"][name]
        assert len(data) == entry["bytes"] and sha(data) == entry["sha256"], name
    read = lambda name: json.loads(raw[name])
    csvrows = lambda name: list(csv.DictReader(io.StringIO(raw[name].decode())))
    source_manifest = read("launch/source_manifest.json")
    assert source_manifest == read("launch/source_manifest_after.json")
    with zipfile.ZipFile(io.BytesIO(raw["launch/sources.zip"])) as z:
        assert set(z.namelist()) == set(source_manifest)
        for name, expected in source_manifest.items():
            assert sha(z.read(name)) == expected
        old_adapter = z.read("simulations/x650_isolated_live_v1/run_case.py").decode()
        old_vehicle = z.read("src/amsim/api/vehicle.py").decode()
    assert "self.node.spin_once(timeout_sec=0.02)" in old_vehicle
    assert "spin_once(timeout_sec=0.)" in old_adapter and "250_000" in old_adapter
    assert "QoSProfile(depth=4000" in old_adapter
    assert "self._last_kwargs != kwargs" in raw["runtime_source/rclpy_executors.py"].decode()

    meta, launch = read("controller_metadata.json"), read("launch/launch.json")
    live = lambda name: read("isolated_live/" + name + ".json")
    start, lifecycle = live("live_start"), live("lifecycle")
    ticks, sends, receipts, gates = [live(name) for name in ["dispatch", "sends", "receipts", "events"]]
    fault = next(row for row in ticks if row["status"] == 3)
    fault_ns = fault["actual_monotonic_ns"]
    assert not fault["output_valid"] and fault["reason"] == 1 and fault["command_age_ns"] > 50_000_000
    assert all(not row["output_valid"] for row in ticks if row["actual_monotonic_ns"] >= fault_ns)
    assert [row["sequence"] for row in gates] == list(range(1, len(gates) + 1))
    assert all(row["accepted"] for row in gates)

    cdr = json.loads((ROOT / "cdr_review.json").read_text())
    assert cdr["source_sha256"] == sha(raw["isolated_live/source_recording/diagnostics.jsonl"])
    assert cdr["worker_sha256"] == sha(raw["isolated_live/recording/diagnostics.jsonl"])
    assert cdr["all_materialized_ros_fields_match_original_cdr"]
    proposals = {row["proposal"]["sequence"]: row for row in cdr["candidates"]}
    assert cdr["all_worker_candidates_numerically_accepted"]
    send_ids = {row["dispatch_sequence"]: row for row in sends}
    receipt_ids = {row["dispatch_sequence"]: row for row in receipts}
    assert len(send_ids) == len(sends) and len(receipt_ids) == len(receipts) and send_ids.keys() == receipt_ids.keys()
    physical = csvrows("ts20_state.csv")
    consumed = [{}, {}]
    for row in physical:
        for j in range(2):
            consumed[j].setdefault(int(row[f"upper_sequence{j+1}"]), row)
    lags = []
    first_send = {}
    for i, send in enumerate(sends):
        receipt = receipt_ids[send["dispatch_sequence"]]
        assert all(send[key] == receipt[key] == start[key] for key in ["session_id", "epoch"])
        assert send["command_sequence"] == receipt["command_sequence"]
        assert receipt["joint_consumed"] and not receipt["failed"]
        assert receipt["pva_send_returned"] is (not send["stop_zero"])
        assert same(receipt["consumed_counter"], send["expected_counter"], 0)
        if i:
            assert same(send["expected_counter"], np.asarray(sends[i-1]["expected_counter"]) + 1, 0)
            if not send["stop_zero"]:
                assert receipt_ids[sends[i-1]["dispatch_sequence"]]["observed_monotonic_ns"] <= send["sent_monotonic_ns"]
        for j in range(2):
            row = consumed[j][send["expected_counter"][j]]
            assert abs(float(row[f"u{j+1}"]) - send["joint_velocity"][j]) <= 1e-12
            stamp = float(row[f"receipt{j+1}_s"])
            assert abs(stamp - receipt["consumed_sim_s"][j]) <= 1e-9
            assert -.008 <= stamp - send["sim_s"] <= .05
            lags.append(stamp - send["sim_s"])
        if not send["stop_zero"]:
            first_send.setdefault(send["command_sequence"], send)
            p = proposals[send["command_sequence"]]["proposal"]
            assert send["coordinate_frame"] == 1 and send["type_mask"] == 2048
            assert same(send["position_ned"], [p["position_reference_enu"][1], p["position_reference_enu"][0], start["ref_alt"]-p["position_reference_enu"][2]])
            assert same(send["velocity_ned"], [p["velocity_feedforward_enu"][1], p["velocity_feedforward_enu"][0], -p["velocity_feedforward_enu"][2]])
            assert same(send["acceleration_ned"], [0., 0., 0.]) and same(send["joint_velocity"], p["joint_velocity"])
            assert abs(send["yaw_ned"] - start["yaw_ned"]) <= 1e-10
    zeros = [row for row in sends if row["stop_zero"]]
    assert len(zeros) == 1 and zeros[0] is sends[-1] and same(zeros[0]["joint_velocity"], [0., 0.])
    assert all(row["sent_monotonic_ns"] < fault_ns for row in sends if not row["stop_zero"])
    release = lifecycle["release"]
    assert release["zero_consumed"] and release["writer_exited"]
    publications = [json.loads(line) for line in raw["control_execution_trace.jsonl"].splitlines()]
    publications = [row for row in publications if "publish_before" in row]
    assert not any(start["issued_monotonic_ns"] <= row["publish_before"].get("wall_monotonic_s", -1)*1e9 < release["wall_monotonic_ns"] for row in publications)
    for sequence, pair in proposals.items():
        if sequence > 1:
            assert pair["proposal"]["compute_start_monotonic_ns"] >= first_send[sequence-1]["sent_monotonic_ns"]
        assert pair["proposal"]["compute_start_monotonic_ns"] >= pair["snapshot"]["issued_monotonic_ns"]

    worker_records = [json.loads(line) for line in raw["isolated_live/recording/diagnostics.jsonl"].splitlines()]
    wm = live("recording/manifest")
    assert wm["records_sha256"] == sha(raw["isolated_live/recording/diagnostics.jsonl"])
    assert wm["rows"] == len(worker_records) == live("diagnostic_sender")["sent"] == 55
    assert not wm["gaps"] and wm["error"] is None and wm["drained_and_fsynced"]
    wt = live("worker_timing")
    worker = {row[0]: dict(zip(wt["columns"], row)) for row in wt["rows"]}
    it = live("ingress_timing")
    ingress = [dict(zip(it["columns"], row)) for row in it["rows"]]
    phase = []
    for sequence in [54, 55]:
        row = worker[sequence]
        diag = worker_records[sequence-1]["diagnostic"]
        ingress_rows = [r for r in ingress if r["sequence"] == sequence and r["kind"] in [2, 3]]
        phase.append({"sequence": sequence,
            "snapshot_to_callback_ms": (row["callback_entry_ns"]-row["snapshot_issued_ns"])/1e6,
            "callback_to_engine_return_wall_ms": (row["engine_return_ns"]-row["callback_entry_ns"])/1e6,
            "whole_callback_wall_ms": (row["callback_exit_ns"]-row["callback_entry_ns"])/1e6,
            "whole_callback_thread_cpu_ms": (row["callback_thread_cpu_end_ns"]-row["callback_thread_cpu_begin_ns"])/1e6,
            "proposal_publish_wall_ms": (row["publish_end_ns"]-row["publish_begin_ns"])/1e6,
            "proposal_publish_thread_cpu_ms": (row["publish_thread_cpu_end_ns"]-row["publish_thread_cpu_begin_ns"])/1e6,
            "proposal_compute_end_after_fault_ms": (row["proposal_compute_end_ns"]-fault_ns)/1e6,
            "engine_return_after_fault_ms": (row["engine_return_ns"]-fault_ns)/1e6,
            "qp_ms": diag["solve_ms"], "allocator_timing": diag["ts20_timing_observation"],
            "ingress_records": ingress_rows,
            "proposal_publish_end_to_ingress_entry_ms": (ingress_rows[0]["entry_ns"]-row["publish_end_ns"])/1e6,
            "gate_event_exists": any(g["sequence"] == sequence for g in gates),
            "normal_send_exists": sequence in first_send})

    source_records = [json.loads(line) for line in raw["isolated_live/source_recording/diagnostics.jsonl"].splitlines()]
    sm, sender = live("source_recording/manifest"), live("source_sender")
    assert sm["records_sha256"] == sha(raw["isolated_live/source_recording/diagnostics.jsonl"])
    assert [r["sequence"] for r in source_records] == list(range(1, len(source_records)+1))
    assert sm["rows"] == sender["sent"] == sender["attempts"] == len(source_records) == 3800
    assert not sm["gaps"] and sm["error"] is None and sender["dropped"] == 0
    feedback = [r for r in source_records if r["kind"] == "feedback"]
    lo, hi = source_records.index(feedback[0]), source_records.index(feedback[-1])
    assert all(r["kind"] == "feedback" for r in source_records[lo:hi+1])
    fault_record = next(r for r in source_records if r["kind"] == "ros" and r["topic"] == "dispatch"
                        and struct.unpack_from("<QQQ", base64.b64decode(r["cdr"]["base64"]), 4)[2] == fault["sequence"])
    pump = live("source_pump_timing")
    assert all(row[3] == 1 for row in pump["rows"])
    wf = live("worker_final")
    active_gc = [row for row in wf["gc_events"] if row["start_ns"] >= start["issued_monotonic_ns"]]
    before, restored = read("diagnostic_parameters_before.json"), read("diagnostic_parameters_restored.json")
    assert before.keys() == restored.keys()
    assert all((before[k]["param_type"], before[k]["raw_value_hex"]) == (restored[k]["param_type"], restored[k]["raw_value_hex"]) for k in before)
    restore = read("diagnostic_restore_journal_completed.json")
    assert restore["restored_verified"] and restore["storage_save"]["returncode"] == 0
    flight = csvrows("flight_samples.csv")
    final = flight[-1]
    assert final["armed"] == "0" and final["landed"] == "1"
    assert meta["landed_and_disarmed"] and meta["diagnostic_parameters_restored"] and launch["runtime_closed"]
    assert launch["sources_unchanged"] and launch["runtime_binaries_unchanged"]
    assert all(p["returncode"] == 0 for p in lifecycle["processes"].values())
    ulog = json.loads((ROOT / "ulog_recovery_review.json").read_text())
    assert ulog["source_sha256"] == sha(raw["px4.ulg"])
    assert ulog["recovery"]["landed_and_not_armed_at_log_end"]

    report = {
        "kind": "hold21_failed_attempt_independent_review_v1", "derived_analysis": True,
        "complete": False, "passed": False, "eligible_for_normal_figure8": False,
        "scope": "Failed partial hold: independently verified CDR, physical joint consumption, observer delay, runtime recovery and PX4 ULog recovery; no full-hold performance acceptance.",
        "protocol_revision": 14, "task_sha256": sha(raw["controller_task.json"]),
        "native_library_sha256": wf["engine"]["numerical_backend"]["library_sha256"],
        "native_proof_sha256": wf["engine"]["numerical_backend"]["software_proof_sha256"],
        "task_start_sim_s": meta["task_start_sim_s"], "requested_hold_s": meta["requested_duration_s"],
        "last_worker_task_time_s": proposals[55]["proposal"]["task_time_s"],
        "reported_python_error": meta["error"], "scenario_returncode": read("run_manifest.json")["scenario_returncode"],
        "launch_exit_code": launch["exit_code"],
        "fault": {k: fault[k] for k in ["sequence", "actual_monotonic_ns", "source_sequence", "source_received_monotonic_ns", "command_sequence", "command_issued_monotonic_ns", "command_age_ns", "status", "reason", "output_valid"]},
        "worker": {"count": len(proposals), "all_numerically_accepted": True, "gate_accepted_count": len(gates),
            "candidate55_not_forwarded_or_sent": 55 not in first_send and not any(g["sequence"] == 55 for g in gates),
            "engine_wall_ms": stats([(r["engine_return_ns"]-r["callback_entry_ns"])/1e6 for r in worker.values()]),
            "actual_failure_phases": phase, "gc_generation_counts_after_live_start": dict(Counter(r["generation"] for r in active_gc)),
            "gc_max_us_after_live_start": max(r["end_ns"]-r["start_ns"] for r in active_gc)/1e3,
            "timing_scope": "26.923/23.137 ms are callback-entry to engine-return wall intervals; 27.801/24.017 ms CPU measurements cover the whole callback, not the identical interval. Offline replay/profile timing is not substituted for actual-run timing."},
        "observer": {"source_evidence_rows": len(source_records), "dropped": sender["dropped"],
            "first_feedback_evidence_sequence": feedback[0]["sequence"], "last_feedback_evidence_sequence": feedback[-1]["sequence"],
            "consecutive_feedback_without_any_ros_observer_record": len(feedback),
            "last_feedback_sim_s": feedback[-1]["value"]["sim_s"],
            "last_feedback_sink_after_fault_ms": (feedback[-1]["received_ns"]-fault_ns)/1e6,
            "first_post_feedback_observer_sink_after_fault_ms": (source_records[hi+1]["received_ns"]-fault_ns)/1e6,
            "first_fault_evidence_sequence": fault_record["sequence"], "first_fault_sink_monotonic_ns": fault_record["received_ns"],
            "fault_to_sink_ms": (fault_record["received_ns"]-fault_ns)/1e6,
            "sink_semantics": "Recorder received_ns is sampled after recvmsg and unpickle, not DDS arrival or callback entry. Exact callback-entry latency was not recorded.",
            "active_pump_count": len(pump["rows"]), "extra_callbacks_each_active_pump": 1,
            "source_sender_offer_max_ms": max(sender["offer_ns"])/1e6,
            "mechanism": "Archived VehicleAPI uses spin_once(0.02), active extra drain uses spin_once(0) with 250us budget. Installed rclpy resets its ready-callback iterator when kwargs change. All late-added ROS observation callbacks were starved throughout the recorded feedback block and were drained during recovery. This is more specific than queue depth alone.",
            "remediation_scope": "Use consistent executor timeout arguments and separately prioritize a sticky, latest-value fault supervisor; retain complete deep-queue CDR evidence. Prove notification latency under realistic high-rate callbacks before another stage claim."},
        "consumption": {"all_send_and_receipt_identities_verified": True, "all_raw_counter_velocity_and_timestamp_matches": True,
            "all_sends": len(sends), "normal_sends": len(sends)-1, "repeated_normal_sends": sum(r["repeated"] for r in sends if not r["stop_zero"]),
            "all_receipts": len(receipts), "individual_joint_commands": 2*len(sends), "per_joint_counter_range": [sends[0]["expected_counter"][0], sends[-1]["expected_counter"][0]],
            "actual_delivery_lag_sim_s": stats(lags), "normal_output_after_fault": False,
            "normal_pva_fields_match_original_proposal": True, "normal_output_waited_for_previous_receipt": True,
            "no_overlapping_legacy_output": True, "legacy_publication_records_checked": len(publications)},
        "recovery": {"zero_send": zeros[0], "zero_receipt": receipt_ids[zeros[0]["dispatch_sequence"]],
            "zero_send_after_fault_ms": (zeros[0]["sent_monotonic_ns"]-fault_ns)/1e6,
            "zero_consumption_receipt_after_fault_ms": (receipt_ids[zeros[0]["dispatch_sequence"]]["observed_monotonic_ns"]-fault_ns)/1e6,
            "release": release, "landed_and_disarmed": True, "final_flight_sample": final,
            "ulog_recovery_independently_verified": True, "ulog_recovery": ulog["recovery"],
            "restored_typed_parameter_count": len(before), "synchronous_parameter_save_returncode": restore["storage_save"]["returncode"],
            "runtime_closed": True, "owned_process_returncodes": {k:v["returncode"] for k,v in lifecycle["processes"].items()},
            "pending_zero_all_hardware_fault_scenarios_qualified": False},
        "provenance": {"all_archive_entry_hashes_match": True, "archived_launch_source_files_verified": len(source_manifest),
            "sources_unchanged": True, "runtime_binaries_unchanged": True,
            "raw_cdr_content_verified": True, "ulog_original_sha256": sha(raw["px4.ulg"])}
    }
    (ROOT / "failure_report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf8")
    print(json.dumps({"passed": False, "review_checks_completed": True, "worker": len(proposals), "sends": len(sends),
                      "fault_to_sink_ms": report["observer"]["fault_to_sink_ms"], "recovery_verified": True}))


if __name__ == "__main__":
    main()
