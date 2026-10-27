#!/usr/bin/env python3
"""Read-only supplement: validate archived MPC plans with the common event gate.

The following real snapshot must retain this call's adopted plan. No MPC/NMPC
solve, interpolation, plan extrapolation, or physical simulation is launched.
Original corpus files/reports remain unchanged; only a NEW output directory is
written. Missing or inconsistent successor bindings are explicitly excluded.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import time


def require(value, reason):
    if not value:
        raise ValueError(reason)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def identity(path):
    path = Path(path).resolve(strict=True)
    require(path.is_file(), "Expected evidence file: " + str(path))
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return dict(path=str(path), bytes=path.stat().st_size, sha256=h.hexdigest())


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      parse_constant=lambda x: (_ for _ in ()).throw(ValueError("Nonfinite JSON: " + x)))


def write(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def finite_vector(value, length):
    return isinstance(value, list) and len(value) == length and all(
        type(v) in (float, int) and math.isfinite(v) for v in value)


def recover_plan(current, successor, config):
    """Exact original commit identity; zero tolerance, no inferred tail plan."""
    c, n = current["row"], successor["row"]
    m, after, inp = c["memory"], n["memory"], c["input"]
    require(successor["sequence"] == current["sequence"] + 1, "Successor sequence is not consecutive")
    require(n["input"]["t"] > inp["t"], "Successor source time did not advance")
    require(after["plan_t"] == after["last_t"] == inp["t"], "Successor plan/last time differs from this input")
    require(type(m["step_count"]) is int and type(after["step_count"]) is int
            and after["step_count"] == m["step_count"] + 1, "Successor is not exactly one commit later")
    command = c["oracle"]["command"]
    plan = after["plan"]
    require(finite_vector(command, 5) and finite_vector(plan, 5 * config["nodes"]), "Missing finite complete plan/command")
    require(plan[:5] == command == after["previous_command"], "First candidate command differs from archived oracle")
    expected_horizon = [inp["dt"]] + [config["node_dt"]] * (config["nodes"] - 1)
    require(after["plan_h"] == expected_horizon, "Successor horizon differs from original input")
    expected = deepcopy(m)
    expected.update(plan=deepcopy(plan), plan_h=expected_horizon, plan_t=inp["t"],
        previous_command=command, last_t=inp["t"], has_last_t=True,
        previous_stamp=inp["state"]["stamp"], has_stamp=True,
        origin=inp["state"]["control_time"]-inp["t"], has_origin=True,
        step_count=m["step_count"]+1)
    expected["joint_history"].append(dict(time=inp["t"], velocity=command[3:]))
    expected["joint_history"] = [x for x in expected["joint_history"] if x["time"] >= inp["t"]-.75]
    require(after == expected, "Other successor Memory fields differ from exact original commit")
    return deepcopy(plan)


def load_sources(manifest_path, inputs):
    manifest_id = identity(manifest_path)
    manifest = read(manifest_path)
    files, groups, config = [], defaultdict(list), None
    for binding in manifest["inputs"]:
        require(re.fullmatch(r"[A-Za-z0-9_-]+", binding["name"]) is not None, "Unsafe corpus name")
        path = inputs / (binding["name"] + ".json")
        actual = identity(path)
        require(actual["sha256"] == binding["sha256"] and actual["bytes"] == binding["bytes"], "Corpus manifest differs")
        d = read(path); p = d["provenance"]
        require(d["kind"] == "x650_mpc_memory_benchmark_input_v1"
                and p["synthetic_inputs"] is False and p["new_physical_acceptance_claimed"] is False,
                "Only bound real historical corpus accepted")
        require(len(d["cases"]) == binding["count"], "Corpus count changed")
        if config is None:
            config = d["config"]
        require(d["config"] == config, "Config differs across original sources")
        provenance = {x["id"]: x for x in p["cases"]}
        require(len(provenance) == len(d["cases"]), "Case provenance count differs")
        group_identity = {k: p[k] for k in ("stage", "physical_audit_sha256", "startup_sha256", "live_start_sha256")}
        group = digest(group_identity)
        for row in d["cases"]:
            item = provenance[row["id"]]
            groups[group].append(dict(row=row, sequence=item["original_sequence"], source=actual,
                case_provenance=item, source_group=group_identity,
                historical_physical_audit=p["physical_audit"],
                memory_reconstruction=p["memory_reconstruction"],
                original_total_calls=p["original_total_calls"]))
        files.append(actual)
    require(config is not None and groups, "Empty corpus")
    for items in groups.values():
        items.sort(key=lambda x: x["sequence"])
        require(len({x["row"]["id"] for x in items}) == len(items), "Duplicate original case id")
        require([x["sequence"] for x in items] == list(range(1,items[0]["original_total_calls"]+1)),
                "Corpus omits or duplicates historical calls")
    return manifest_id, files, groups, config


def runtime_identity(executable):
    process = subprocess.run(["ldd", str(executable)], text=True, capture_output=True, check=True)
    paths = set(re.findall(r"=>\s+(/\S+)", process.stdout))
    libraries = [identity(p) for p in sorted(paths)]
    require(any(Path(x["path"]).name == "libx650_mpc_core.so" for x in libraries)
            and any(Path(x["path"]).name == "libx650_standard_event_predictor.so" for x in libraries),
            "Common native event runtime not linked")
    return dict(executable=identity(executable), libraries=libraries, ldd_stdout=process.stdout)


def summarize_group(records, rows, config):
    require(len(records) == len(rows), "Event validator record count differs")
    failures = []; counters = Counter(); max_violations = defaultdict(float)
    worst_margin = None; max_speeds = [0., 0.]
    for record, bound in zip(records, rows):
        require(record["id"] == bound["id"] and record["plan"] == bound["plan"]
                and record["task_time_s"] == bound["input"]["t"]
                and record["committed_step_count"] == bound["memory"]["step_count"], "Validator source identity changed")
        for key in ("input_valid", "command_bounds_passed", "reference_bounds_passed", "native_checked",
                    "native_bounds_passed", "stopping_passed", "pending_guard_passed", "hold_passed"):
            counters[key+"_failed"] += record[key] is not True
        for key in ("input_violation", "slew_violation", "reference_violation", "q_envelope_violation",
                    "dq_envelope_violation", "stopping_violation", "pending_violation"):
            value = record[key]
            require(type(value) in (float,int) and math.isfinite(value) and value >= 0., "Nonfinite validation metric")
            max_violations[key] = max(max_violations[key],value)
        if not record["passed"]:
            failures.append(dict(id=record["id"], t=record["task_time_s"], reason=record["reason"],
                                 stopping_violation=record["stopping_violation"]))
        if record["native_checked"]:
            require(len(record["node_states"]) == 4*config["nodes"], "Native node evidence missing")
            actual_violation = 0.
            for i in range(config["nodes"]):
                for j in range(2):
                    q, dq = record["node_states"][4*i+j], record["node_states"][4*i+2+j]
                    reserve = config["velocity_limit"][3+j]/(2*config["acceleration_limit"][3+j])
                    reserve += config["max_dt"]+config["command_hold_s"]+record["source_age_s"]
                    stop = q+reserve*dq
                    margin = min(stop-config["q_lower"][j],config["q_upper"][j]-stop)
                    actual_violation = max(actual_violation,-margin)
                    if worst_margin is None or margin < worst_margin["margin_rad"]:
                        worst_margin = dict(margin_rad=margin,id=record["id"],task_time_s=record["task_time_s"],
                            node=i,joint=j+1,q_rad=q,dq_rad_s=dq,reserve_s=reserve,protected_position_rad=stop)
            require(abs(actual_violation-record["stopping_violation"]) <= 1e-12,
                    "Independent node stopping check differs from C++")
            for j in range(2):
                max_speeds[j] = max(max_speeds[j],abs(record["envelope"][4+j]),abs(record["envelope"][6+j]))
    return dict(checked=len(rows),accepted=len(rows)-len(failures),all_common_checks_passed=not failures,
        failure_counts=dict(counters),maximum_violations=dict(max_violations),
        minimum_actual_stopping_margin=worst_margin,maximum_native_abs_joint_speed_rad_s=max_speeds,
        time_range_s=[rows[0]["input"]["t"],rows[-1]["input"]["t"]] if rows else None,failures=failures)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest",type=Path,required=True)
    parser.add_argument("--inputs-directory",type=Path,required=True)
    parser.add_argument("--validator",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--chunk-size",type=int,default=400)
    args = parser.parse_args(argv)
    require(args.output.is_absolute() and not args.output.exists(), "New absolute output directory required")
    require(1 <= args.chunk_size <= 512,"Bounded chunk size required")
    require(not args.output.resolve().is_relative_to(args.inputs_directory.resolve()),"Output must not enter original corpus")
    manifest, sources, groups, config = load_sources(args.manifest,args.inputs_directory)
    runtime = runtime_identity(args.validator.resolve(strict=True));script_id=identity(__file__)
    args.output.mkdir(parents=True)
    all_excluded=[];group_reports=[];generated=[]
    for group_key, items in groups.items():
        stage=items[0]["source_group"]["stage"]; selected=[];bindings=[];excluded=[]
        require(re.fullmatch(r"[A-Za-z0-9_-]+",stage) is not None,"Unsafe stage")
        for i,item in enumerate(items):
            try:
                require(i+1<len(items),"No subsequent snapshot retains this final candidate plan")
                following=items[i+1];plan=recover_plan(item,following,config)
            except (ValueError,KeyError,TypeError) as error:
                excluded.append(dict(id=item["row"]["id"],sequence=item["sequence"],
                    task_time_s=item["row"]["input"]["t"],reason=str(error)))
                continue
            row=item["row"]
            selected.append(dict(id=row["id"],memory=row["memory"],input=row["input"],plan=plan))
            bindings.append(dict(id=row["id"],source_file=item["source"],source_record=item["case_provenance"],
                successor_id=following["row"]["id"],successor_source=following["source"],
                successor_record=following["case_provenance"],plan_sha256=digest(plan),
                exact_oracle_first_command=True,exact_successor_commit=True))
        require(selected,"No exactly bound plans in source group")
        records=[]
        for start in range(0,len(selected),args.chunk_size):
            name=f"{stage}_{group_key[:8]}_{start//args.chunk_size:03d}"
            inp=args.output/(name+"_input.json");out=args.output/(name+"_event.json")
            write(inp,dict(config=config,cases=selected[start:start+args.chunk_size]))
            process=subprocess.run([str(args.validator),str(inp),str(out)],text=True,capture_output=True)
            (args.output/(name+".log")).write_text(process.stdout+process.stderr,encoding="utf-8")
            require(process.returncode==0,"Native replay failed; log retained: "+name)
            value=read(out)
            require(value["kind"]=="x650_standard_common_event_replay_v1"
                    and value["input_sha256"]==identity(inp)["sha256"],"Native replay input binding differs")
            records.extend(value["records"]);generated += [identity(inp),identity(out)]
        result=summarize_group(records,selected,config)
        group_reports.append(dict(stage=stage,source_group=items[0]["source_group"],
            historical_physical_audit=items[0]["historical_physical_audit"],original_calls=len(items),
            excluded=excluded,**result))
        write(args.output/(stage+"_"+group_key[:8]+"_bindings.json"),bindings)
        all_excluded.extend(excluded)
    for item in [manifest,*sources,runtime["executable"],*runtime["libraries"],script_id]:
        require(identity(item["path"])==item,"Bound source/runtime changed during audit")
    result=dict(kind="x650_legacy_common_gate_supplement_v1",complete=True,created_unix=time.time(),
        source_manifest=manifest,source_files=sources,script=script_id,runtime=runtime,
        config_sha256=digest(config),config=config,groups=group_reports,excluded=all_excluded,
        total_original_calls=sum(x["original_calls"] for x in group_reports),
        total_checked=sum(x["checked"] for x in group_reports),total_excluded=len(all_excluded),
        all_checked_common_checks_passed=all(x["all_common_checks_passed"] for x in group_reports),
        actual_stopping_failure_count=sum(x["failure_counts"]["stopping_passed_failed"] for x in group_reports),
        artifacts=generated,physical_acceptance=False,new_closed_loop_experiment=False,hardware_qualified=False,
        executed_optimizer=False,original_reports_modified=False,
        plan_source="Exact successor Memory plan, reconstructed from archived original commits; first command and complete successor commit checked without tolerance",
        scope="Existing historical hold/wrench corpus only; not the six fresh 20261027 paired flights or a new flight acceptance. Final candidates without successor are excluded.")
    write(args.output/"report.json",result)
    print(json.dumps({k:result[k] for k in ("complete","total_original_calls","total_checked","total_excluded",
        "all_checked_common_checks_passed","actual_stopping_failure_count")},indent=2))


if __name__ == "__main__":
    main()
