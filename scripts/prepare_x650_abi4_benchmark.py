#!/usr/bin/env python3
"""Workstation-only ABI 4 corpus: all calls, real contexts, no flight rerun.

The four complete records are retained regardless of physical acceptance.
Core Memory is reconstructed from every original successful commit. Heavy
sampling is selected by the original algorithm branch, never by new latency.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import io
import json
from pathlib import Path
import re
import zipfile

from prepare_x650_orin_benchmark import (ROOT, SOURCE_PATHS, atomic_write, identity,
    json_bytes, package_sources, sha256, strict_json)
from prepare_x650_recorded_core_benchmark import (OUTPUT_FIELDS, bundles,
    commit_recorded, decode_cdr, initial_memory, message_layout, raw_input, require)

NATIVE = "control_packages/x650_wholebody_v1/native/"
ROS = "ros2_ws/src/"
EXTRA_SOURCES = tuple(NATIVE+n for n in (
    "canonical_interval.hpp", "joint_safety.hpp", "test_multirate_core.cpp",
    "test_interval_core.cpp", "test_interval_observer.cpp")) + (
    "control_packages/x650_wholebody_v1/benchmark/test/test_benchmark_io.cpp",
    "control_packages/x650_wholebody_v1/benchmark/test/test_process.py",)
ALL_SOURCES = SOURCE_PATHS + EXTRA_SOURCES
PRODUCTION = tuple(n for n in ALL_SOURCES if "/benchmark/" not in n
    and "/test_" not in n and not n.endswith(("CMakeLists.txt", "LICENSE")))
WIRE_SOURCES = tuple(ROS+f"flyarm_control_interfaces/msg/{k}.msg" for k in
    ("LiveStart", "ControlSnapshot", "ControlProposal")) + tuple(ROS+n for n in (
    "flyarm_mpc_cpp/include/flyarm_mpc_cpp/worker_contract.hpp",
    "flyarm_mpc_cpp/include/flyarm_mpc_cpp/trajectory_plan.hpp"))

GROUPS = {
 "hold": ("multirate", "x650_cpp_multirate_hold_v1_20261026_213827_21743",
    "x650_cpp_multirate_launch_20261026_213744_21741", 711,
    "artifacts/cpp_multirate_20261026/hold_audit_v3/report.json"),
 "wrench": ("multirate", "x650_cpp_multirate_wrench_v1_20261026_220417_24257",
    "x650_cpp_multirate_launch_20261026_220335_24241", 570,
    "artifacts/cpp_multirate_20261026/wrench_audit_followup_v4/report.json"),
 "mass085": ("robustness", "x650_cpp_multirate_mass085_v1_20261027_102559_3271",
    "x650_cpp_multirate_robustness_launch_20261027_102520_3264", 569,
    "artifacts/cpp_multirate_robustness_20261027/mass085_audit_v1/report.json"),
 "mass115": ("robustness", "x650_cpp_multirate_mass115_v1_20261027_103022_4600",
    "x650_cpp_multirate_robustness_launch_20261027_102939_4594", 570,
    "artifacts/cpp_multirate_robustness_20261027/mass115_audit_v1/report.json"),
}


class Evidence:
    """Read selected members only, checking receipt, manifest and each member."""
    def __init__(self, receipt: Path):
        self.receipt = identity(receipt)
        rec = strict_json(receipt.read_bytes())
        require(rec["complete"] and rec["all_original_sizes_and_sha256_verified"], "Incomplete archive")
        self.zips, self.archives, manifest = {}, [], None
        for row in rec["archives"]:
            name = Path(row["path"]).name
            path = receipt.parent/name
            check = identity(path)
            require(all(check[k] == row[k] for k in ("bytes", "sha256")), "Archive receipt mismatch")
            z = zipfile.ZipFile(path)
            require(len(z.namelist()) == len(set(z.namelist())), "Duplicate ZIP member")
            raw = z.read("_archive_manifest.json")
            require(sha256(raw) == rec["manifest"]["sha256"], "Manifest identity mismatch")
            require(manifest is None or raw == manifest, "Embedded manifests differ")
            manifest = raw
            self.zips[name] = z
            self.archives.append(check)
        rows = strict_json(manifest)["original_files"]
        self.rows = {r["archive_entry"]: r for r in rows}
        require(len(rows) == len(self.rows), "Duplicate manifest member")

    def read(self, name):
        row = self.rows[name]
        raw = self.zips[row["archive"]].read(name)
        require(len(raw) == row["bytes"] and sha256(raw) == row["sha256"], "Member hash mismatch: "+name)
        return raw

    def close(self):
        for z in self.zips.values():
            z.close()


def recover(evidence: Evidence, stage: str):
    _, run, launch, count, audit_name = GROUPS[stage]
    prefix, launch_prefix = "runs/"+run+"/isolated_live/", "runs/"+launch+"/"
    source_raw = evidence.read(launch_prefix+"sources.zip")
    with zipfile.ZipFile(io.BytesIO(source_raw)) as source:
        require(len(source.namelist()) == len(set(source.namelist())), "Duplicate source entry")
        bindings = {}
        for name in PRODUCTION + WIRE_SOURCES:
            original = source.read(name)
            require(original == (ROOT/name).read_bytes(), "Current/archived source differs: "+name)
            bindings[name] = sha256(original)
        layouts = {k: message_layout(source.read(ROS+f"flyarm_control_interfaces/msg/{k}.msg"))
                   for k in ("LiveStart", "ControlSnapshot", "ControlProposal")}
    startup_raw = evidence.read(launch_prefix+"original_startup_manifest.json")
    startup = strict_json(startup_raw)
    cfg = startup["config"]
    require(cfg["nodes"] == 15 and cfg["node_dt"] == .04 and cfg["max_dt"] == .1
            and cfg["command_hold_s"] == .16 and cfg["bus_period"] == .005, "Wrong ABI 4 timing profile")
    require(startup["timing_profile"] == "mpc25_dispatch50_v1", "Wrong timing profile name")
    kind = "hold" if stage == "hold" else "fast"
    require(startup["trajectory"]["kind"] == ("hold" if kind == "hold" else "fast_figure8"), "Wrong trajectory")
    live_raw = evidence.read(prefix+"worker_live_start.cdr")
    live = decode_cdr(live_raw, layouts["LiveStart"])
    require(live == strict_json(evidence.read(prefix+"live_start.json")), "LiveStart JSON/CDR differs")
    audit_raw = evidence.read(audit_name)
    audit = strict_json(audit_raw)
    require(audit["complete"] is True, "Source run audit incomplete")
    physical = {k: audit[k] for k in ("passed", "complete", "certificate", "physical_acceptance")}
    memory, cases, records, attempts = initial_memory(live), [], [], Counter()
    names = sorted(n for n in evidence.rows if re.fullmatch(re.escape(prefix)+r"native_records/\d{6}\.json", n))
    require(len(names) == count, "Original run count differs")
    for sequence, name in enumerate(names, 1):
        record_raw = evidence.read(name)
        record = strict_json(record_raw)
        require(record["sequence"] == sequence and not record["error"], "Original call error/order differs")
        decoded, cdr_bindings = {}, {}
        for key, wire in (("snapshot_cdr", "ControlSnapshot"), ("proposal_cdr", "ControlProposal")):
            entry = name[:-5]+(".snapshot.cdr" if key == "snapshot_cdr" else ".proposal.cdr")
            raw = evidence.read(entry)
            require(len(raw) == record[key]["bytes"] and sha256(raw) == record[key]["sha256"], "CDR binding differs")
            decoded[key] = decode_cdr(raw, layouts[wire])
            cdr_bindings[key] = {"entry": entry, "bytes": len(raw), "sha256": sha256(raw)}
        s, p = decoded["snapshot_cdr"], decoded["proposal_cdr"]
        for value in (s, p):
            require(value["schema_version"] == 1 and value["sequence"] == sequence and value["replay_only"] is False
                    and all(value[k] == live[k] for k in ("session_id", "epoch", "configuration_sha256")), "Wire identity differs")
        require(p["snapshot_sequence"] == sequence and p["status"] == 0
            and p["accepted_optimum"] and p["nonlinear_check_passed"], "Original proposal rejected")
        result = record["result"]
        d = result["diagnostics"]
        require(d["accepted"] and d["nonlinear_passed"] and d["actual_bounds"] and not d["previous_plan_fallback"]
            and d["subproblems"] == len(d["attempts"]) == p["subproblems"] and 1 <= d["subproblems"] <= 3, "Original certificate differs")
        for field in ("position_reference", "velocity_feedforward", "acceleration_feedforward"):
            require(result[field] == p[field+"_enu"], "Result/proposal field differs")
        require(result["command"] == p["base_velocity_target"]+p["joint_velocity"]
                and result["joint_velocity"] == p["joint_velocity"], "Command/proposal differs")
        require(s["task_time_s"] == p["task_time_s"] and s["control_time_s"] == p["control_time_s"]
            and s["task_time_s"] > memory["last_t"] and abs(s["task_time_s"]-memory["last_t"]-s["dt_s"]) <= 1e-9,
            "Nonconsecutive historical control clocks")
        require(s["position_gains"] == cfg["base_position_gain"], "PX4 position gain differs")
        inp = raw_input(s, cfg, live["target_position_enu"], kind)
        require(d["horizon"] == [inp["dt"]]+[cfg["node_dt"]]*14 and len(d["plan"]) == 75
                and d["plan"][:5] == result["command"], "Original plan/horizon differs")
        guard = d["hold_guard"]
        require(guard["required"] and guard["checked"] and guard["passed"] and guard["duration_s"] == .16,
                "Original command hold guard did not pass")
        identifier = f"{stage}_{sequence:06d}"
        cases.append({"id": identifier, "memory": memory, "input": inp,
            "oracle": {k: deepcopy(result[k]) for k in OUTPUT_FIELDS}})
        records.append({"id": identifier, "original_sequence": sequence, "record_entry": name,
            "record_sha256": sha256(record_raw), "historical_subproblems": d["subproblems"],
            "historical_compute_ms": d["compute_ms"], "historical_active_constraints": d["active_constraints"],
            "historical_interval_rows": d["interval_rows"], "historical_recovery_triggered": d["recovery_triggered"],
            "historical_hold_guard": {k: guard[k] for k in ("required", "checked", "passed", "duration_s", "substeps")},
            **cdr_bindings})
        attempts[d["subproblems"]] += 1
        memory = commit_recorded(memory, inp, result)
    provenance = {"kind": "x650_abi4_recorded_corpus_v1", "stage": stage,
        "expected_core_abi": 4, "expected_optimization_abi": 2,
        "physical_audit": physical, "physical_audit_sha256": sha256(audit_raw),
        "new_physical_acceptance_claimed": False, "synthetic_inputs": False,
        "state_restored_before_every_call": True, "continuous_closed_loop_replay": False,
        "memory_was_directly_recorded": False,
        "memory_reconstruction": "Original LiveStart and every original successful commit, with current/archived sources byte identical",
        "preview_reconstruction": "Current/archived C++ trajectory port; current target checked to 1e-13; native output comparison still required",
        "original_total_calls": count, "historical_attempt_distribution": dict(attempts),
        "archives": evidence.archives, "receipt": evidence.receipt,
        "source_zip_sha256": sha256(source_raw), "startup_sha256": sha256(startup_raw),
        "live_start_sha256": sha256(live_raw), "bound_production_and_wire_sources": bindings,
        "cases": records}
    return cfg, cases, provenance


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--cpu", type=int, default=2)
    args = p.parse_args(argv)
    require(args.cpu >= 0, "Pin a common available logical CPU")
    args.output.mkdir(parents=True, exist_ok=False)
    protocol = {"kind": "x650_abi4_paired_protocol_v1", "expected_core_abi": 4,
        "expected_optimization_abi": 2, "nominal_mpc_period_ms": 40, "dispatch_period_ms": 20,
        "core_deadline_ms": 50, "command_hold_ms": 160, "field_policy": "recorded-derived-v2",
        "tolerances": {"position_reference": 1e-9, "other_outputs": 1e-7,
            "joint_acceleration_reference": "1e-7 / input.dt plus separate exact derivative identity check"},
        "cohort": "All 2420 consecutive calls from hold711/wrench570/mass085569/mass115570",
        "heavy_selection": "All original calls with subproblems > 1 or recovery_triggered or interval_rows > 0, no latency selection",
        "run": {"cpu": args.cpu, "warmup_rounds": 2, "measured_rounds": 10, "heavy_measured_rounds": 500},
        "all_failures_and_outliers_retained": True, "core_sources_changed": False,
        "nx_persistent_application_inputs_results_logs": False, "cuda_enabled": False}
    atomic_write(args.output/"protocol.json", json_bytes(protocol))
    protocol_hash = sha256(json_bytes(protocol))
    receipts = {"multirate": ROOT/"docs/validation/cpp_multirate_20261026/multirate_evidence_receipt.json",
        "robustness": ROOT/"docs/validation/cpp_multirate_robustness_20261027/delivery_v1/multirate_robustness_evidence_receipt.json"}
    readers = {k: Evidence(path) for k, path in receipts.items()}
    manifest = {"kind": "x650_abi4_preparation_v1", "protocol": identity(args.output/"protocol.json"), "groups": {}, "inputs": []}
    try:
        for stage, group in GROUPS.items():
            config, cases, provenance = recover(readers[group[0]], stage)
            manifest["groups"][stage] = {"calls": len(cases), "physical_audit": provenance["physical_audit"],
                "historical_attempt_distribution": provenance["historical_attempt_distribution"]}
            collections = [(stage, cases, provenance, 10)]
            indices = [i for i, r in enumerate(provenance["cases"]) if r["historical_subproblems"] > 1
                or r["historical_recovery_triggered"] or r["historical_interval_rows"] > 0]
            if indices:
                heavy = deepcopy(provenance)
                heavy["cases"] = [heavy["cases"][i] for i in indices]
                heavy["heavy_selection"] = protocol["heavy_selection"]
                collections.append((stage+"_heavy", [cases[i] for i in indices], heavy, 500))
            for label, selected, prov, rounds in collections:
                for i, value in enumerate(bundles(config, selected, prov, chunk_size=min(400, len(selected)), cpu=args.cpu,
                        measured_rounds=rounds, field_policy="recorded-derived-v2", protocol_sha256=protocol_hash)):
                    if "heavy_selection" in prov:
                        value["provenance"]["chunk"]["selection"] = prov["heavy_selection"]
                    value["provenance"]["benchmark_group"] = label
                    path = args.output/f"{label}_{i:03d}.json"
                    atomic_write(path, json_bytes(value))
                    manifest["inputs"].append({**identity(path), "group": label,
                        "cases": len(value["cases"]), "measured_rounds": rounds})
        manifest["source_package"] = package_sources(ROOT, args.output/"code_only_sources.tar.gz", source_paths=ALL_SOURCES)
        manifest["source_package"]["current_abi4_sources_match_original_runs"] = True
        manifest["complete"] = True
        atomic_write(args.output/"preparation.json", json_bytes(manifest))
        print(json.dumps({"complete": True, "groups": manifest["groups"],
            "input_chunks": len(manifest["inputs"]), "source_package_sha256": manifest["source_package"]["archive"]["sha256"]}))
    finally:
        for reader in readers.values():
            reader.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
