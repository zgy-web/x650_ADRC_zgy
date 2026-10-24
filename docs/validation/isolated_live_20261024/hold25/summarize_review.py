"""Recompute hold25 summary offline from the original-byte ZIP and review data.

No extraction, ROS, SSH, simulation or control output. review_remote.py separately
rechecks the large hashed server CDR/plugin/ULog originals.
"""
import argparse
import bisect
import hashlib
import json
from pathlib import Path
import statistics
import zipfile


def summarize(root):
    load = lambda name: json.loads((root / name).read_text(encoding="utf-8"))
    originals = {f["path"]: f for f in load("originals_manifest.json")["files"]}
    with zipfile.ZipFile(root / "originals.zip") as archive:
        assert len(archive.namelist()) == len(set(archive.namelist())) == len(originals)
        assert set(archive.namelist()) == set(originals)
        contents = {name: archive.read(name) for name in originals}
    for name, raw in contents.items():
        f = originals[name]
        assert len(raw) == f["bytes"] and hashlib.sha256(raw).hexdigest() == f["sha256"], name
    read = lambda name: json.loads(contents[name])
    def table(name):
        d = read(name)
        return d, [dict(zip(d["columns"], row)) for row in d["rows"]]
    physical = load("remote_writer_recovery_review.json")
    for name, f in physical["source_files"].items():
        local = "launch.json" if name == "launch/launch.json" else name
        if local in originals:
            assert all(f[k] == originals[local][k] for k in ("source_path", "bytes", "sha256"))
    fault = physical["first_output_fault"]
    assert fault["sequence"] == 4070 and fault["reason"] == 1 and fault["command_sequence"] == 1055
    assert fault["command_age_ns"] == fault["actual_monotonic_ns"] - fault["command_issued_monotonic_ns"] > 50_000_000
    fault_ns = fault["actual_monotonic_ns"]
    start = read("isolated_live/live_start.json")
    activity, pumps = table("isolated_live/source_activity.json")
    source_data, sources = table("isolated_live/source_timing.json")
    worker_data, workers = table("isolated_live/worker_timing.json")
    ingress_data, ingress = table("isolated_live/ingress_timing.json")
    assert activity["complete"] and activity["recorded"] == activity["attempted"] == len(pumps) == 7136
    assert not any(activity[k] for k in ("overflow", "duplicate_marks", "invalid_marks", "overlapping_pumps", "unfinished_records"))
    assert worker_data["complete"] and worker_data["dropped"] == 0
    assert ingress_data["complete"] and ingress_data["overflow"] == 0
    phases = list(activity["phases"].values())
    previous_end = 0
    for i, row in enumerate(pumps, 1):
        assert row["pump_id"] == i and row["completed"] == 1
        assert previous_end <= row["begin_ns"] <= row["end_ns"]
        stamps = [row[n + "_begin_ns"] for n in phases if row[n + "_begin_ns"]]
        assert stamps == sorted(stamps) and row["begin_ns"] <= stamps[0] <= stamps[-1] <= row["end_ns"]
        assert row["end_thread_cpu_ns"] >= row["begin_thread_cpu_ns"]
        previous_end = row["end_ns"]
    assert [r["pump_id"] for r in pumps if r["failed"]] == [len(pumps)]
    last_pump = pumps[-1]
    assert last_pump["failure_phase"] == activity["freeze"]["phase"] == 1
    assert activity["freeze"]["reason"] == 1 and activity["freeze"]["pump_id"] == last_pump["pump_id"]
    assert last_pump["failure_ns"] <= activity["freeze"]["monotonic_ns"] <= last_pump["end_ns"]
    begins = [r["begin_ns"] for r in pumps]
    for s in sources:
        i = bisect.bisect_right(begins, s["issued_ns"]) - 1
        assert i >= 0
        p = pumps[i]
        assert p["begin_ns"] <= s["issued_ns"] <= p["read_begin_ns"] <= p["publish_begin_ns"] <= s["publish_return_ns"] <= p["end_ns"]
    assert sources[-1]["sequence"] == fault["source_sequence"] == 4098
    source_age_ns = fault_ns - sources[-1]["issued_ns"]
    assert 0 <= source_age_ns <= 25_000_000
    assert any(r["kind"] == 1 and r["source_sequence"] == 4098 and r["source_issued_ns"] == sources[-1]["issued_ns"] and r["source_received_ns"] == fault["source_received_monotonic_ns"] for r in ingress)

    gc = activity["gc"]
    assert gc["policy_before"] is not None and gc["policy_unchanged"] and gc["policy_before"] == gc["policy_after"]
    assert gc["overflow"] == gc["unknown_phases"] == 0 and gc["recorded"] == gc["attempted"] == len(gc["rows"])
    opened, pairs = {}, []
    for raw in gc["rows"]:
        g = dict(zip(gc["columns"], raw));key = (g["thread_id"], g["generation"])
        if g["phase"] == 1:
            assert key not in opened;opened[key] = g
        else:
            assert g["phase"] == 2 and key in opened
            first = opened.pop(key)
            assert g["monotonic_ns"] >= first["monotonic_ns"] and g["thread_cpu_ns"] >= first["thread_cpu_ns"]
            pairs.append({"thread_id": key[0], "generation": key[1], "start_ns": first["monotonic_ns"], "end_ns": g["monotonic_ns"],
                "wall_ms": (g["monotonic_ns"] - first["monotonic_ns"]) / 1e6,
                "thread_cpu_ms": (g["thread_cpu_ns"] - first["thread_cpu_ns"]) / 1e6})
    assert not opened and len(pairs) == 19
    assert not any(p["start_ns"] <= fault_ns + 100_000_000 and p["end_ns"] >= fault_ns - 100_000_000 for p in pairs)
    worker = {r["sequence"]: r for r in workers}
    final = read("isolated_live/worker_final.json")
    assert len(workers) == final["engine"]["requests"] == final["engine"]["accepted"] == 1056
    assert not final["engine"]["faulted"] and final["diagnostic_completion_error"] is None
    for i, row in enumerate(workers):
        vals = [row[k] for k in ("publish_end_ns", "diagnostic_complete_begin_ns", "diagnostic_complete_end_ns", "snapshot_cdr_begin_ns")]
        assert all(v > 0 for v in vals) and vals == sorted(vals)
        if i + 1 < len(workers):assert row["callback_exit_ns"] <= workers[i + 1]["callback_entry_ns"]
    events = read("isolated_live/events.json");sends = read("isolated_live/sends.json");receipts = read("isolated_live/receipts.json")
    assert len(events) == 1055 and all(e["accepted"] and e["reason"] == 0 for e in events)
    assert [e["proposal_sequence"] for e in events] == list(range(1, 1056))
    assert len(sends) == len(receipts) == 1058 and len({r["dispatch_sequence"] for r in sends}) == len(sends)
    assert {r["dispatch_sequence"] for r in sends} == {r["dispatch_sequence"] for r in receipts}
    normal = [r for r in sends if not r["stop_zero"]]
    assert len(normal) == 1057 and sum(r["repeated"] for r in normal) == 2
    assert all(r["sent_monotonic_ns"] < fault_ns for r in normal)
    assert not any(r["command_sequence"] == 1056 for r in normal)
    send1055 = next(r for r in normal if r["command_sequence"] == 1055)
    ticks = {r["sequence"]: r for r in physical["dispatch_tail"]}
    assert ticks[4068]["command_sequence"] == 1054 and ticks[4068]["repeated"]
    assert ticks[4069]["command_sequence"] == 1055 and not ticks[4069]["repeated"]
    assert worker[1055]["engine_return_ns"] > ticks[4068]["scheduled_monotonic_ns"]
    assert worker[1056]["engine_return_ns"] > fault_ns

    before, after = [read(n) for n in ("diagnostic_parameters_before.json", "diagnostic_parameters_restored.json")]
    assert set(before) == set(after)
    fields = ("value", "param_type", "type_name", "raw_value_hex", "source_system", "source_component")
    assert all(before[k][f] == after[k][f] for k in before for f in fields)
    lifecycle = read("isolated_live/lifecycle.json");meta = read("controller_metadata.json");launch = read("launch.json")
    assert meta["landed_and_disarmed"] and meta["diagnostic_parameters_restored"] and not meta["diagnostic_restore_pending"]
    assert not meta["complete"] and not launch["complete"] and launch["exit_code"] == 2
    assert launch["runtime_closed"] and launch["sources_unchanged"] and launch["runtime_binaries_unchanged"]
    assert contents["hold_v25.exit"].decode().strip() == "1"
    assert all(p["returncode"] == 0 for p in lifecycle["processes"].values())
    handoff = meta["failed_recovery_handoff"]
    assert handoff["confirmed"] and handoff["decision"] == "fold_then_land" and handoff["command"] == 176
    assert handoff["release"]["zero_consumed"] and handoff["release"]["writer_exited"]
    assert handoff["parameters"][:3] == [1., 4., 3.]
    boundary = handoff["command_boundary_wall_monotonic_s"]
    assert handoff["last_status"]["callback_sequence"] > handoff["before"]["callback_sequence"]
    assert handoff["last_status"]["timestamp_us"] > handoff["before"]["timestamp_us"]
    assert handoff["last_status"]["nav_state"] == 4 and not handoff["last_status"]["failsafe"]
    assert handoff["last_status"]["received_wall_monotonic_s"] > boundary
    assert any(a["result"] == 0 and a["received_wall_monotonic_s"] > boundary and a["target_system"] == 1 and a["target_component"] == 1 for a in handoff["acks"])
    confirmation_ms = (handoff["confirmed_wall_monotonic_s"] - boundary) * 1000
    assert 0 <= confirmation_ms <= 350
    ulog = physical["ulog"]
    assert not any(s["failsafe"] for s in ulog["vehicle_status_changes"])
    assert ulog["recovery"]["landed_and_not_armed_at_log_end"]
    flight = load("flight_tail.json");assert flight["rows"][-1]["armed"] == "0" and flight["rows"][-1]["landed"] == "1"
    resource = load("resources.json");rr = resource["rows"]
    assert rr[0]["monotonic_ns"] <= start["issued_monotonic_ns"] < rr[1]["monotonic_ns"]
    assert rr[-2]["monotonic_ns"] < fault_ns <= rr[-1]["monotonic_ns"]
    stats = [dict((k, int(v)) for k, v in [line.split() for line in r["cpu.stat"].splitlines()]) for r in rr]
    deltas = {k: stats[-1][k] - stats[0][k] for k in stats[0]}
    assert deltas["nr_throttled"] == deltas["throttled_usec"] == 0
    assert len({r["cpu.max"].strip() for r in rr}) == 1
    quota, period = map(int, rr[0]["cpu.max"].split());span_ns = rr[-1]["monotonic_ns"] - rr[0]["monotonic_ns"]
    historical_host = read("host_v1.json");host = load("host_postrun.json")
    assert historical_host["hostname"] == host["hostname"] and historical_host["cpu_max"].strip() == host["cpu_max"] == rr[0]["cpu.max"].strip()
    topology = host["topology"]
    roles = launch["protocol"]["cpu_roles"];reserved = launch["execution_resources"]["reserved_with_smt_siblings"]
    background = launch["execution_resources"]["background"]
    for cpus in roles.values():
        for cpu in cpus:
            assert cpu in reserved and cpu not in background
            assert all(int(c) not in background for c in topology['cpu' + str(cpu)]["thread_siblings_list"].split(','))
    tail = [{"sequence": r["sequence"], "engine_ms": (r["engine_return_ns"] - r["callback_entry_ns"]) / 1e6,
             "callback_ms": (r["callback_exit_ns"] - r["callback_entry_ns"]) / 1e6,
             "callback_cpu_ms": (r["callback_thread_cpu_end_ns"] - r["callback_thread_cpu_begin_ns"]) / 1e6} for r in workers[-5:]]
    return {
        "kind": "hold25_independent_failure_archive_review_v1", "full_hold_pass": False,
        "run": "x650_isolated_live_v1_20261024_131702_20177", "launch": "x650_isolated_launch_20261024_131614_20176",
        "task_sha256": launch["task_sha256"], "execution_revision": launch["protocol"]["execution_revision"],
        "archived_originals_verified": len(originals), "physical_source_files_checked_remotely": physical["source_files"],
        "fault": {k: v for k, v in fault.items() if k != "command"},
        "task_progress_last_source_s": sources[-1]["control_s"] - start["origin_s"],
        "fresh_source_issue_age_ms": source_age_ns / 1e6,
        "fresh_source_received_age_ms": (fault_ns - fault["source_received_monotonic_ns"]) / 1e6,
        "supervisor_latency_ms": read("isolated_live/fault_supervision.json")["first_fault"]["producer_to_observation_ns"] / 1e6,
        "source_activity": {"complete": True, "pumps": len(pumps), "all_sources_bound_to_read_publish": len(sources),
            "unique_last_failure_preserved": True, "failed_phase": "supervision", "last_pump_wall_ms": (last_pump["end_ns"] - last_pump["begin_ns"]) / 1e6,
            "max_pump_ms": max((r["end_ns"] - r["begin_ns"]) / 1e6 for r in pumps),
            "max_between_pumps_ms": max((b["begin_ns"] - a["end_ns"]) / 1e6 for a,b in zip(pumps,pumps[1:])),
            "complete_gc_pairs": pairs, "gc_policy_unchanged": True, "no_gc_in_fault_plusminus100ms": True},
        "worker": {"numerical_accepted": len(workers), "gate_accepted": len(events), "gate_rejected": 0,
            "last_five": tail, "all_deferred_diagnostic_boundaries_valid": True,
            "command1055_return_after_tick4068_scheduled_ms": (worker[1055]["engine_return_ns"] - ticks[4068]["scheduled_monotonic_ns"]) / 1e6,
            "command1055_gate_after_tick4068_actual_ms": (events[-1]["received_monotonic_ns"] - ticks[4068]["actual_monotonic_ns"]) / 1e6,
            "command1055_first_send_age_ms": (send1055["sent_monotonic_ns"] - worker[1055]["snapshot_issued_ns"]) / 1e6,
            "command1056_return_after_fault_ms": (worker[1056]["engine_return_ns"] - fault_ns) / 1e6,
            "command1056_normal_sent": False},
        "execution": {k:physical[k] for k in ("all_sends","normal_sends","normal_repeats","all_receipts","individual_joint_commands","per_joint_counter_range","exact_send_receipt_identity_and_raw_joint_counter_value_time_matches","normal_output_after_output_fault","no_legacy_publication_overlap","all_materialized_ros_fields_match_original_cdr","cdr_counts","stop_zero_send_after_output_fault_ms","stop_zero_receipt_after_output_fault_ms")},
        "recovery": {"fresh_hold_confirmed": True,"hold_confirmation_ms":confirmation_ms,"six_processes_returned_zero":True,
            "runtime_closed": True,"restored_exact_parameters":len(before),"ulog_status_rows":ulog["vehicle_status_rows"],
            "px4_failsafe_observed":False,"ulog":ulog["recovery"],"ulog_status_changes":ulog["vehicle_status_changes"]},
        "resources": {"source":resource["source"],"bracket_samples":len(rr),"duration_s":span_ns/1e9,
            "cgroup_stat_deltas":deltas,"mean_cpu_cores":deltas["usage_usec"]*1000/span_ns,"quota_cpu_equivalent":quota/period,
            "cpu_max":rr[0]["cpu.max"].strip()},
        "hardware": {"hostname":host["hostname"],"cpu_model_postrun":host["cpu_model"],"logical_cpus_visible":host["logical_cpu_count"],
            "physical_cores_visible":len({(r['core_id'],r['physical_package_id']) for r in topology.values()}),
            "sockets_visible":len({r['physical_package_id'] for r in topology.values()}),"cpu_roles":roles,
            "own_background_excludes_smt_siblings":True,"cpu_resources_exclusive":False,
            "scope":"CPU model/topology post-run snapshot agrees with original host identity/quota; a role mask is allowed affinity, not observed execution per call."},
        "limitations":["This failed window does not qualify the 30 s hold or hardware deployment.",
            "Source freshness and GC evidence do not reproduce or explain the earlier hold24 source pause.",
            "No new quota throttling occurred in the bracket. Coarse worker-mask frequency samples do not establish each call's actual CPU frequency or a general hardware capacity bound.",
            "The 128 visible logical CPUs are not 128 exclusive CPUs; the container has a 12-CPU time quota on a shared host.",
            "Complete raw CDR, plugin CSV and ULog are retained at hashed server paths and independently decoded; they are not copied in this archive.",
            "Host monotonic, simulation time and original ULog microseconds are kept distinct without fitted clock conversion."]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--verify", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args();result = summarize(args.source)
    if args.verify:
        assert result == json.loads(args.verify.read_text(encoding="utf-8"))
        print("hold25 summary reproduced; all ZIP original hashes and stated checks matched")
    elif args.output:
        with args.output.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False);stream.write("\n")
    else:print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
