#!/usr/bin/env python3
"""Recover historical Core calls from accepted 2026-10-24 ROS CDR evidence.

This is workstation-side preparation, not a new closed-loop run. Memory was not
recorded directly: it is reconstructed from the exact archived reset/prime and
successful commit rules, using *every* preceding recorded result. Mathematical
equivalence remains a separate acceptance condition of the native benchmark.
Only selected ZIP entries are read; no flight assets or executables are unpacked.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import io
import json
import math
from pathlib import Path
import re
import struct
import sys
import zipfile

from prepare_x650_orin_benchmark import atomic_write, identity, json_bytes, sha256, strict_json

ROOT = Path(__file__).resolve().parents[1]
PREFIX = "accepted/run/isolated_live/"
SOURCE_PREFIX = "ros2_ws/src/"
CORE_PATH = "control_packages/x650_wholebody_v1/native/mpc_core.cpp"
SOURCE_HASHES = {
    CORE_PATH: "4f2994398b88515454b77c2c2d7ed0d8346498655cac3c25b876075f9c792162",
    SOURCE_PREFIX + "flyarm_mpc_cpp/include/flyarm_mpc_cpp/worker_contract.hpp": "d368dffa52209391a4c4c85ca8ef1fd3471dac1511e9b7f1d6469be2dce87b82",
    SOURCE_PREFIX + "flyarm_mpc_cpp/src/isolated_worker.cpp": "7ab06faa244df747b0b0fdf2846a4f8bbb2bab3c8951948bc17a02d05a3c931c",
    SOURCE_PREFIX + "flyarm_state_cpp/include/flyarm_state_cpp/source_core.hpp": "4ede458e7753c99cd770d4cc3f28e0f95464c6cddeac630928fe0d702e73071c",
    SOURCE_PREFIX + "flyarm_control_interfaces/msg/ControlSnapshot.msg": "2fd53a0988f9aed466ae697dd69828cbc521e058409249ed78e9fdffa13c7fa2",
    SOURCE_PREFIX + "flyarm_control_interfaces/msg/ControlProposal.msg": "b8875418e1b0d33a845fd0a96e0b5182dc9dcde1de3ac6baea3e13fce8024e82",
    SOURCE_PREFIX + "flyarm_control_interfaces/msg/LiveStart.msg": "0468cf668a55a9415622a3445ddf1b4eebc8b7cce43c52db31ef4a8194689417",
}
TRAJECTORY_HASHES = {
    "normal": "4c26fa8f0c3320aca389b423db77d8e5536095063b6cc6ee8e8e8d60864f0c1c",
    "fast": "90e4897f537aafb99f82261ffc8bc682d6473a26abca3d0d513241289cb3bd68",
}
PRIMITIVES = {"uint8": ("B", 1), "uint16": ("H", 2), "uint32": ("I", 4),
              "uint64": ("Q", 8), "float64": ("d", 8), "bool": ("B", 1)}
OUTPUT_FIELDS = ("command", "position_reference", "velocity_feedforward", "acceleration_feedforward",
                 "joint_velocity", "joint_reference", "joint_velocity_reference",
                 "joint_acceleration_reference", "nominal_acceleration", "tip_actual")


def require(value, message):
    if not value:
        raise ValueError(message)


def message_layout(source: bytes):
    layout = []
    for line in source.decode("utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        columns = line.split()
        require(len(columns) in (2, 3), "Unsupported frozen ROS message declaration")
        match = re.fullmatch(r"([a-z0-9]+)(?:\[(\d+)\])?", columns[0])
        require(match is not None and match[1] in PRIMITIVES, "Unsupported frozen ROS field type")
        require(re.fullmatch(r"[a-z][a-z0-9_]*", columns[1]), "Invalid ROS field name")
        layout.append((columns[1], match[1], int(match[2]) if match[2] else None))
    require(len({row[0] for row in layout}) == len(layout), "Duplicate ROS field")
    return layout


def decode_cdr(data: bytes, layout) -> dict:
    """CDR1 little-endian fixed primitives, alignment relative to payload byte 4."""
    require(data[:4] == b"\x00\x01\x00\x00", "Only original CDR1 little-endian encoding is supported")
    offset, result = 4, {}
    for name, typ, count in layout:
        fmt, alignment = PRIMITIVES[typ]
        offset += (-(offset - 4)) % alignment
        number = 1 if count is None else count
        end = offset + number * alignment
        require(end <= len(data), "Truncated original CDR")
        values = list(struct.unpack_from("<" + fmt * number, data, offset))
        if typ == "bool":
            require(all(v in (0, 1) for v in values), "Invalid CDR boolean")
            values = [bool(v) for v in values]
        if typ == "float64":
            require(all(math.isfinite(v) for v in values), "Nonfinite original CDR field")
        result[name] = values[0] if count is None else values
        offset = end
    require(offset == len(data), "Original CDR has unconsumed bytes or a changed layout")
    return result


def frozen_target(t: float, center: list, stage: str) -> dict:
    """Literal arithmetic port of archived source_core.hpp Trajectory::sample."""
    require(stage in ("normal", "fast", "hold"), "Unknown trajectory")
    if stage == "hold":
        return {"position": list(center), "velocity": [0.]*3, "acceleration": [0.]*3}
    period = 8. if stage == "fast" else 12.
    ax, az, start, ramp, cycles = .25, .10, 2., 2., 2
    w, total, duration = 2 * math.pi / period, 2 * math.pi * cycles, cycles * period + ramp
    s = t - start
    phi = speed = acc = 0.
    if s >= duration:
        phi = total
    elif s > 0 and (s < ramp or s > duration - ramp):
        ending = s > duration - ramp
        u = (duration - s if ending else s) / ramp
        a = 2.5 * math.pow(u, 4) - 3 * math.pow(u, 5) + math.pow(u, 6)
        b = 10 * math.pow(u, 3) - 15 * math.pow(u, 4) + 6 * math.pow(u, 5)
        c = 30 * u * u * (1 - u) * (1 - u)
        phi = total - w * ramp * a if ending else w * ramp * a
        speed, acc = w * b, (-1 if ending else 1) * w * c / ramp
    elif s > 0:
        phi, speed = w * (s - .5 * ramp), w
    d = [ax * math.cos(phi), 0., 2 * az * math.cos(2 * phi)]
    dd = [-ax * math.sin(phi), 0., -4 * az * math.sin(2 * phi)]
    p = list(center)
    p[0] += ax * math.sin(phi)
    p[2] += az * math.sin(2 * phi)
    return {"position": p, "velocity": [v * speed for v in d],
            "acceleration": [dd[j] * speed * speed + d[j] * acc for j in range(3)]}


def initial_memory(live: dict) -> dict:
    n, origin = live["history_count"], live["origin_s"]
    require(1 <= n <= 64 and origin >= 0., "Invalid LiveStart history/origin")
    times, commands = live["history_time_s"], live["history_command"]
    require(times[n - 1] == origin and all(times[i] < times[i+1] for i in range(n-1)),
            "LiveStart history must be ordered and end at origin")
    require(not any(times[n:]) and not any(commands[5*n:]), "Nonzero unused LiveStart history")
    return {"initialized": True, "history_primed": True, "has_last_t": True, "has_stamp": False,
        "has_origin": True, "anchor": {"position": live["anchor_position_enu"],
        "rotation": live["anchor_rotation_row_major"], "q": live["anchor_q"]},
        "previous_command": commands[5*(n-1):5*n], "last_t": times[n-1] - origin,
        "previous_stamp": 0., "origin": origin, "plan_t": 0., "step_count": 0,
        "plan": [], "plan_h": [], "joint_history": [
            {"time": times[i]-origin, "velocity": commands[5*i+3:5*i+5]}
            for i in range(n) if times[i]-origin >= -.75]}


def raw_input(s: dict, config: dict, center: list, stage: str) -> dict:
    n, p = s["command_count"], s["packet_count"]
    require(1 <= n <= 64 and 1 <= p <= 64, "Original TS20 context count outside bound")
    for key, count, stride in [("command_time_s", n, 1), ("command_velocity", n, 2),
        ("packet_send_time_s", p, 1), *[("packet_"+k, p, 2) for k in ("q_ref", "dq_ref", "tau_ff", "kp", "kd")]]:
        require(not any(s[key][count*stride:]), "Nonzero unused original context: " + key)
    context = {"reference_time": s["reference_time_s"], "bus_origin": s["bus_origin_s"],
        "reference": s["reference_r"], "commands": [
            {"time": s["command_time_s"][i], "velocity": s["command_velocity"][2*i:2*i+2]}
            for i in range(n)], "packets": [
            {"send_time": s["packet_send_time_s"][i], **{
                k: s["packet_"+k][2*i:2*i+2] for k in ("q_ref", "dq_ref", "tau_ff", "kp", "kd")}}
            for i in range(p)]}
    state = {k: s[v] for k, v in {"position": "position_enu", "velocity": "velocity_enu",
        "omega_body": "omega_body_flu", "acceleration": "acceleration_enu", "rotation": "rotation_row_major",
        "q": "q", "dq": "dq", "stamp": "state_time_s", "control_time": "control_time_s",
        "joint_stamp": "joint_time_s"}.items()}
    target = {k: s["target_"+k+"_enu"] for k in ("position", "velocity", "acceleration")}
    expected = frozen_target(s["task_time_s"], center, stage)
    require(max(abs(a-b) for k in target for a,b in zip(target[k], expected[k])) <= 1e-13,
            "Recorded target differs from frozen C++ trajectory")
    preview, offset = [], 0.
    for i in range(config["nodes"]):
        offset += s["dt_s"] if i == 0 else config["node_dt"]
        preview.append(frozen_target(s["task_time_s"] + offset, center, stage))
    return {"t": s["task_time_s"], "dt": s["dt_s"], "state": state,
            "target": target, "context": context, "preview": preview}


def commit_recorded(memory: dict, value: dict, result: dict) -> dict:
    """Exact successful Memory transition in frozen Core::step, without a solve."""
    m = deepcopy(memory)
    d, t, s = result["diagnostics"], value["t"], value["state"]
    m.update(plan=deepcopy(d["plan"]), plan_h=list(d["horizon"]), plan_t=t,
        previous_command=list(result["command"]), last_t=t, has_last_t=True,
        previous_stamp=s["stamp"], has_stamp=True, origin=s["control_time"]-t,
        has_origin=True, step_count=m["step_count"]+1)
    m["joint_history"].append({"time": t, "velocity": list(result["joint_velocity"])})
    m["joint_history"] = [row for row in m["joint_history"] if row["time"] >= t-.75]
    return m


def recover(archive: Path, receipt: Path, report: Path, stage: str):
    """Recover every original successful call in sequence; never skip history."""
    require(stage in ("normal", "fast"), "Unknown historical stage")
    archive_id, rec, certificate = identity(archive), strict_json(receipt.read_bytes()), report.read_bytes()
    require((archive_id["bytes"], archive_id["sha256"]) == (rec["bytes"], rec["sha256"]),
            "Archive differs from accepted receipt")
    cert = strict_json(certificate)
    require(all(cert.get(k) is True for k in ("passed", "complete", "certificate", "physical_acceptance")),
            "Original closed-loop experiment is not accepted")
    if "physical_certificate_sha256" in rec:
        require(sha256(certificate) == rec["physical_certificate_sha256"], "Certificate receipt SHA differs")
    with zipfile.ZipFile(archive) as z:
        require(len(z.namelist()) == len(set(z.namelist())), "Duplicate archive entries")
        require(z.read(f"software_and_audit/{stage}_audit_v1/report.json") == certificate,
                "Embedded accepted certificate differs")
        sources_raw = z.read("accepted/launch/sources.zip")
        with zipfile.ZipFile(io.BytesIO(sources_raw)) as source:
            require(len(source.namelist()) == len(set(source.namelist())), "Duplicate source archive entries")
            bound_sources = dict(SOURCE_HASHES)
            trajectory_path = SOURCE_PREFIX+"flyarm_mpc_cpp/include/flyarm_mpc_cpp/trajectory_plan.hpp"
            bound_sources[trajectory_path] = TRAJECTORY_HASHES[stage]
            for name, digest in bound_sources.items():
                require(sha256(source.read(name)) == digest, "Unsupported historical production source: " + name)
            layouts = {kind: message_layout(source.read(SOURCE_PREFIX+f"flyarm_control_interfaces/msg/{kind}.msg"))
                       for kind in ("LiveStart", "ControlSnapshot", "ControlProposal")}
        startup_raw = z.read("accepted/launch/startup_manifest.json")
        startup = strict_json(startup_raw)
        config = startup["config"]
        require(startup["trajectory"]["kind"] == stage+"_figure8", "Startup trajectory differs")
        require(config["nodes"] == 15 and config["node_dt"] == .04 and config["max_dt"] == .05,
                "Unsupported original horizon or deadline")
        live_raw = z.read(PREFIX+"worker_live_start.cdr")
        live = decode_cdr(live_raw, layouts["LiveStart"])
        require(live == strict_json(z.read(PREFIX+"live_start.json")), "CDR and JSON LiveStart differ")
        memory, cases, provenance_cases = initial_memory(live), [], []
        names = sorted(n for n in z.namelist() if re.fullmatch(re.escape(PREFIX)+r"native_records/\d{6}\.json", n))
        require(len(names) == (1598 if stage == "normal" else 1198), "Original historical call count differs")
        counts = Counter()
        for sequence, name in enumerate(names, start=1):
            record_raw = z.read(name)
            record = strict_json(record_raw)
            require(record["sequence"] == sequence and not record["error"], "Original call sequence/error differs")
            decoded, bindings = {}, {}
            for key, kind in (("snapshot_cdr", "ControlSnapshot"), ("proposal_cdr", "ControlProposal")):
                entry = name[:-5]+(".snapshot.cdr" if key == "snapshot_cdr" else ".proposal.cdr")
                raw = z.read(entry)
                binding = record[key]
                require((len(raw), sha256(raw)) == (binding["bytes"], binding["sha256"]), "Historical CDR SHA differs")
                decoded[key] = decode_cdr(raw, layouts[kind])
                bindings[key] = {"archive_entry": entry, "bytes": len(raw), "sha256": sha256(raw)}
            s, proposal = decoded["snapshot_cdr"], decoded["proposal_cdr"]
            for item in (s, proposal):
                require(item["schema_version"] == 1 and item["session_id"] == live["session_id"]
                        and item["epoch"] == live["epoch"] and item["configuration_sha256"] == live["configuration_sha256"]
                        and item["sequence"] == sequence and item["replay_only"] is False, "Historical wire identity differs")
            require(proposal["snapshot_sequence"] == sequence and proposal["status"] == 0
                    and proposal["accepted_optimum"] and proposal["nonlinear_check_passed"], "Historical proposal is rejected")
            result, d = record["result"], record["result"]["diagnostics"]
            require(d["accepted"] and d["nonlinear_passed"] and d["actual_bounds"] and not d["previous_plan_fallback"]
                    and d["subproblems"] == len(d["attempts"]) == proposal["subproblems"]
                    and 1 <= d["subproblems"] <= 3, "Historical optimum/candidate certificate differs")
            for field in ("position_reference", "velocity_feedforward", "acceleration_feedforward"):
                require(result[field] == proposal[field+"_enu"], "Recorded result and actual proposal differ")
            require(result["command"] == proposal["base_velocity_target"]+proposal["joint_velocity"]
                    and result["joint_velocity"] == proposal["joint_velocity"], "Recorded command and proposal differ")
            require(s["task_time_s"] == proposal["task_time_s"] and s["control_time_s"] == proposal["control_time_s"]
                    and s["task_time_s"] > memory["last_t"] and abs(s["task_time_s"]-memory["last_t"]-s["dt_s"]) <= 1e-9,
                    "Historical control clocks are not consecutive")
            require(s["position_gains"] == config["base_position_gain"], "Historical PX4 gains differ")
            value = raw_input(s, config, live["target_position_enu"], stage)
            require(d["horizon"] == [value["dt"]]+[config["node_dt"]]*(config["nodes"]-1)
                    and len(d["plan"]) == config["nodes"]*5 and d["plan"][:5] == result["command"], "Historical plan/horizon differs")
            identifier = f"{stage}_{sequence:06d}"
            cases.append({"id": identifier, "memory": memory, "input": value,
                          "oracle": {key: deepcopy(result[key]) for key in OUTPUT_FIELDS}})
            counts[d["subproblems"]] += 1
            provenance_cases.append({"id": identifier, "original_sequence": sequence,
                "task_time_s": value["t"], "record_archive_entry": name,
                "record_sha256": sha256(record_raw), "record_bytes": len(record_raw),
                "historical_subproblems": d["subproblems"], "historical_compute_ms": d["compute_ms"],
                "historical_active_constraints": d["active_constraints"], **bindings})
            memory = commit_recorded(memory, value, result)
        provenance = {"kind": "accepted_recorded_core_snapshot_corpus_v1", "stage": stage,
            "synthetic_inputs": False, "recorded_physical_state_corpus": True,
            "state_source": "Actual historical PX4 estimated feedback and joint encoder/MIT context",
            "continuous_closed_loop_replay": False, "state_restored_before_every_call": True,
            "memory_was_directly_recorded": False, "memory_reconstruction":
                "Full original LiveStart reset/prime, followed by every successful archived Core Memory commit; no intervening call skipped",
            "preview_reconstruction": "Archived C++ trajectory formula port; recorded current target checked to 1e-13; native oracle verification still required",
            "native_equivalence_already_verified": False,
            "scope": "Offline frozen mathematical replay of real recorded closed-loop inputs, not new flight, new tracking accuracy or continuous closed-loop acceptance",
            "source_archive": archive_id, "archive_receipt": identity(receipt),
            "accepted_physical_certificate": identity(report), "startup_manifest_sha256": sha256(startup_raw),
            "sources_zip_sha256": sha256(sources_raw), "bound_historical_sources": bound_sources,
            "live_start_cdr_sha256": sha256(live_raw), "original_production_deadline_ms": 50,
            "original_core_source_abi": 1, "original_total_calls": len(cases),
            "historical_attempt_distribution": {str(k): v for k,v in sorted(counts.items())},
            "oracle_sources": "Original native StepResult, cross-checked with actual published ControlProposal CDR",
            "cases": provenance_cases}
    return config, cases, provenance


def apply_field_policy(bundle, policy, protocol_sha256=None):
    """Keep the original native contract and expose all derived history.

    The frozen executable only supports a uniform optional-field tolerance.
    Under v2, all ten historical outputs remain available for independent,
    dimension-aware workstation checks; no original command bound changes.
    """
    require(policy in ("strict-v1", "recorded-derived-v2"), "Unknown field policy")
    if protocol_sha256 is not None:
        require(re.fullmatch(r"[0-9a-f]{64}", protocol_sha256), "Invalid protocol SHA")
    value = deepcopy(bundle)
    if policy == "strict-v1":
        require(protocol_sha256 is None, "Strict default does not accept a v2 protocol binding")
        return value
    rows = {r["id"]: r for r in value["provenance"]["cases"]}
    require(len(rows) == len(value["cases"]), "Historical case identities differ")
    for case in value["cases"]:
        require(set(case["oracle"]) == set(OUTPUT_FIELDS), "Full historical output required for derived policy")
        require(case["id"] in rows, "Historical case provenance missing")
        rows[case["id"]]["historical_output_oracle"] = case["oracle"]
        case["oracle"] = {key: case["oracle"][key] for key in
                          ("command", "position_reference", "velocity_feedforward")}
    value["provenance"]["comparison_protocol"] = {"field_policy": policy}
    if protocol_sha256 is not None:
        value["provenance"]["comparison_protocol"]["protocol_sha256"] = protocol_sha256
    return value


def bundles(config, cases, provenance, *, chunk_size=400, warmup_rounds=2, measured_rounds=10, cpu=-1,
            field_policy="strict-v1", protocol_sha256=None):
    require(1 <= chunk_size <= 512 and 0 <= warmup_rounds <= 1000 and 1 <= measured_rounds <= 10000 and cpu >= -1,
            "Invalid bounded benchmark run limits")
    require(chunk_size*max(warmup_rounds, measured_rounds) <= 100000, "Benchmark sample limit exceeded")
    for start in range(0, len(cases), chunk_size):
        subset = cases[start:start+chunk_size]
        p = dict(provenance)
        p["cases"] = provenance["cases"][start:start+chunk_size]
        p["chunk"] = {"index": start//chunk_size, "first_original_sequence": start+1,
            "last_original_sequence": start+len(subset), "selection": "All original calls in sequence; no latency-based selection"}
        value = {"schema_version": 1, "kind": "x650_mpc_memory_benchmark_input_v1",
            "config": config, "cases": subset, "run": {"warmup_rounds": warmup_rounds,
            "measured_rounds": measured_rounds, "cpu": cpu, "oracle_absolute_tolerance": 1e-7}, "provenance": p}
        value = apply_field_policy(value, field_policy, protocol_sha256)
        require(len(json_bytes(value)) <= 32*1024*1024, "Input exceeds native 32 MiB bound; reduce chunk-size")
        yield value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("normal", "fast"), required=True)
    parser.add_argument("--source-archive", type=Path)
    parser.add_argument("--archive-receipt", type=Path)
    parser.add_argument("--accepted-report", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--chunk-size", type=int, default=400)
    parser.add_argument("--warmup-rounds", type=int, default=2)
    parser.add_argument("--measured-rounds", type=int, default=10)
    parser.add_argument("--cpu", type=int, default=-1)
    parser.add_argument("--field-policy", choices=("strict-v1", "recorded-derived-v2"), default="strict-v1")
    parser.add_argument("--comparison-protocol", type=Path,
                        help="Optional v2 protocol JSON, frozen before running; run settings must match")
    args = parser.parse_args(argv)
    folder = ROOT/f"docs/validation/cpp_{args.stage}_20261024"
    archive = args.source_archive or folder/f"{args.stage}_full_raw_v1.zip"
    receipt = args.archive_receipt or folder/f"{args.stage}_full_raw_v1_receipt.json"
    report = args.accepted_report or folder/"report.json"
    protocol_sha256 = None
    if args.comparison_protocol is not None:
        protocol_raw = args.comparison_protocol.read_bytes()
        protocol = strict_json(protocol_raw)
        require(args.field_policy == protocol.get("field_policy") == "recorded-derived-v2",
                "Comparison protocol policy differs")
        require(protocol.get("run") == {"warmup_rounds": args.warmup_rounds,
                "measured_rounds": args.measured_rounds, "cpu": args.cpu,
                "oracle_absolute_tolerance": 1e-7}, "Comparison protocol run settings differ")
        protocol_sha256 = sha256(protocol_raw)
    config, cases, provenance = recover(archive, receipt, report, args.stage)
    output = list(bundles(config, cases, provenance, chunk_size=args.chunk_size,
        warmup_rounds=args.warmup_rounds, measured_rounds=args.measured_rounds, cpu=args.cpu,
        field_policy=args.field_policy, protocol_sha256=protocol_sha256))
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    for value in output:
        number = value["provenance"]["chunk"]["index"]
        path = args.output/f"{args.stage}_chunk_{number:03d}.json"
        atomic_write(path, json_bytes(value))
        rows.append({**identity(path), "cases": len(value["cases"]), "chunk": value["provenance"]["chunk"]})
    manifest = {"kind": "x650_recorded_core_workstation_preparation_v1", "complete": True,
        "stage": args.stage, "case_count": len(cases), "chunks": rows,
        "measured_calls_per_host": len(cases)*args.measured_rounds,
        "historical_attempt_distribution": provenance["historical_attempt_distribution"],
        "field_policy": args.field_policy, "comparison_protocol_sha256": protocol_sha256,
        "source_archive": provenance["source_archive"], "bound_historical_sources": provenance["bound_historical_sources"],
        "native_equivalence_already_verified": False, "nx_disk_inputs_or_outputs_required": False}
    atomic_write(args.output/"preparation.json", json_bytes(manifest))
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
