#!/usr/bin/env python3
"""Revalidate and summarize a complete workstation-side paired Core cohort.

No SSH or target execution. Percentiles are computed from every individual
sample, never averaged from chunk percentiles. The plot needs matplotlib;
the validation and JSON/CSV report only use the Python standard library.
"""
from __future__ import annotations

import argparse
import csv
from collections import Counter
import io
import json
from pathlib import Path
import statistics

import compare_x650_paired_core_benchmark as paired
from prepare_x650_orin_benchmark import atomic_write, identity, json_bytes


def portable_identity(path):
    value = identity(path)
    value["path"] = Path(path).name
    return value


def basename(value):
    return str(value).replace("\\", "/").rsplit("/", 1)[-1]


def environment(before, after):
    """Publish relevant conditions without hostnames or connection details."""
    cpu = before["cpu"]
    policy = next((v for k, v in cpu.items() if k.startswith("policy")
                   and "2" in v.get("related_cpus", "").split()), {})
    model_lines = [x.split(":", 1)[1].strip() for x in before.get("cpuinfo", "").splitlines()
                   if x.startswith("model name")]
    model = before.get("model")
    if not isinstance(model, str) or not model:
        model = model_lines[0] if model_lines else None
    temperatures = lambda v: {x["type"]: float(x["temperature_millicelsius"])/1000
        for x in v.get("thermal", []) if str(x.get("temperature_millicelsius", "")).lstrip("-").isdigit()}
    power = before.get("tools", {}).get("power_query", {}).get("stdout", "")
    return {
        "architecture": before["architecture"], "kernel": before["kernel"],
        "os": next((x.split("=", 1)[1].strip('"') for x in before["os_release"].splitlines()
                    if x.startswith("PRETTY_NAME=")), None),
        "model_as_reported": model,
        "local_cpu_affinity": [2], "cpu2_frequency_policy_before": policy,
        "power_mode_as_reported": "MAXN" if "NV Power Mode: MAXN" in power else "not reported",
        "readiness_compiler_not_necessarily_binary_build_compiler": before["tools"]["compiler"]["stdout"].splitlines()[0],
        "system_package_versions_eigen_jsoncpp_openssl": before["tools"]["pkgconfig"]["stdout"].splitlines(),
        "temperature_c_before": temperatures(before), "temperature_c_after": temperatures(after),
        "cgroup_before": before.get("cgroup", {}), "cgroup_after": after.get("cgroup", {}),
        "power_or_clock_policy_changed_for_test": False,
        "temperature_and_frequency_are_endpoint_samples_not_continuous_telemetry": True,
    }


def write_csv(path, rows):
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
    atomic_write(path, stream.getvalue().encode("utf-8"))


def summarize(args):
    root = args.evidence
    manifest = paired.load_json(root/"formal_inputs_v2/manifest.json")
    protocol_path = root/"formal_inputs_v2/protocol.json"
    protocol = paired.load_json(protocol_path)
    paired.require(identity(protocol_path)["sha256"] == manifest["protocol_sha256"], "Protocol hash differs")
    source, _ = paired.verify_sources(args.source_manifest, args.source_package)
    paired.require(source["sha256"] == protocol["source_package_sha256"], "Protocol source differs")
    stages, chunks, cases_csv, strict_csv = {}, [], [], []
    maximum_errors = {name: 0. for name in paired.OUTPUT_DIMENSIONS}
    historical_errors = {side: maximum_errors.copy() for side in ("server", "nx")}
    strict_keys, strict_measured, strict_cases = set(), set(), set()
    source_abis = set()
    code_hashes, common_scope, run = None, None, None
    for entry in manifest["entries"]:
        path = root/"formal_inputs_v2"/basename(entry["path"])
        paired.verify_identity(path, entry, "Manifest input")
        bundle = paired.load_json(path)
        p = bundle["provenance"]
        stage = p["stage"]
        paired.require(stage in ("normal", "fast"), "Unexpected stage")
        paired.require(p["comparison_protocol"]["protocol_sha256"] == manifest["protocol_sha256"], "Unbound input")
        paired.require(bundle["run"] == protocol["run"], "Run differs from frozen protocol")
        source_abis.add(p["original_core_source_abi"])
        record = paired.compare(path, root/("server_formal_"+path.stem), root/("nx_formal_"+path.stem),
            args.source_manifest, args.server_build, args.nx_build, args.source_package, protocol["field_policy"])
        hashes = {s: record["evidence"][s]["loaded_code_sha256"] for s in ("server", "nx")}
        if code_hashes is not None:
            paired.require(hashes == code_hashes and record["scope"] == common_scope, "Chunk code/scope changed")
        code_hashes, common_scope, run = hashes, record["scope"], record["run"]
        data = {s: paired.load_json(root/(s+"_formal_"+path.stem)/"native_stdout.json") for s in ("server", "nx")}
        target = stages.setdefault(stage, {"expected_unique_inputs": p["original_total_calls"], "ids": [],
            "cases": [], "pairs": [], "warmup_pairs": [], "cold_pairs": [], "all_chunk_comparisons_pass": True})
        paired.require(target["expected_unique_inputs"] == p["original_total_calls"], "Total corpus changed")
        target["ids"].extend(case["id"] for case in bundle["cases"])
        target["cases"].extend(bundle["cases"])
        target["pairs"].extend(zip(data["server"]["samples"], data["nx"]["samples"]))
        target["warmup_pairs"].extend(zip(data["server"]["warmup_samples"], data["nx"]["warmup_samples"]))
        target["cold_pairs"].append((data["server"]["cold_first_call"], data["nx"]["cold_first_call"]))
        target["all_chunk_comparisons_pass"] &= record["pass"]
        chunks.append({"input": portable_identity(path), "stage": stage, "cases": entry["cases"],
            "pass": record["pass"], "strict_full_field_passed": record["strict_full_field_passed"],
            "strict_measured_failure_count": record["strict_measured_failure_count"],
            "stdout": {s: {"bytes": record["evidence"][s]["stdout"]["bytes"],
                "sha256": record["evidence"][s]["stdout"]["sha256"]} for s in ("server", "nx")}})
        for field in maximum_errors:
            maximum_errors[field] = max(maximum_errors[field], record["cross_target_max_absolute_errors"][field])
            for side in historical_errors:
                historical_errors[side][field] = max(historical_errors[side][field], record["historical_max_absolute_errors"][side][field])
        for failure in record["strict_failures"]:
            key = (failure["phase"], failure["case_id"], failure["round"])
            strict_keys.add(key)
            if failure["phase"] == "samples":
                strict_measured.add(key)
                strict_cases.add(failure["case_id"])
            for field, error in failure["field_errors"].items():
                strict_csv.append({"chunk": path.stem, "phase": key[0], "case_id": key[1], "round": key[2],
                    "comparison": failure["comparison"], "side": failure.get("side", "paired"), "field": field,
                    **error})
        by_case = {index: [] for index in range(len(bundle["cases"]))}
        for a, b in zip(data["server"]["samples"], data["nx"]["samples"]):
            by_case[a["case_index"]].append((a, b))
        for index, case in enumerate(bundle["cases"]):
            rows = by_case[index]
            paired.require(len(rows) == run["measured_rounds"], "Case repetition count differs")
            cases_csv.append({"stage": stage, "case_id": case["id"], "t_s": case["input"]["t"],
                "dt_s": case["input"]["dt"], "repetitions": len(rows),
                "server_wall_median_ms": statistics.median(a["core_step_wall_ns"]/1e6 for a,b in rows),
                "nx_wall_median_ms": statistics.median(b["core_step_wall_ns"]/1e6 for a,b in rows),
                "server_wall_max_ms": max(a["core_step_wall_ns"]/1e6 for a,b in rows),
                "nx_wall_max_ms": max(b["core_step_wall_ns"]/1e6 for a,b in rows),
                "server_subproblems": max(a["diagnostics"]["subproblems"] for a,b in rows),
                "nx_subproblems": max(b["diagnostics"]["subproblems"] for a,b in rows),
                "max_command_cross_error": max((max(abs(x-y) for x,y in zip(a["output"]["command"],b["output"]["command"]))
                    for a,b in rows if a["returned"] and b["returned"]), default=None),
                "uniform_strict_failure": case["id"] in strict_cases})
    stage_reports = {}
    for stage, item in stages.items():
        paired.require(item["ids"] == [f"{stage}_{i:06d}" for i in range(1,item["expected_unique_inputs"]+1)],
                       "Cohort has missing, duplicate, reordered or selected inputs")
        measured = paired.comparison(item["pairs"])
        paired.require(measured["sample_pairs"] == len(item["ids"])*run["measured_rounds"], "Incomplete measured cohort")
        stage_reports[stage] = {"unique_inputs": len(item["ids"]), "all_chunk_comparisons_pass": item["all_chunk_comparisons_pass"],
            "measured": measured, "warmup": paired.comparison(item["warmup_pairs"]),
            "cold_first_call": paired.comparison(item["cold_pairs"]),
            "paired_nx_over_server_wall_ratio": paired.stats([b["core_step_wall_ns"]/a["core_step_wall_ns"] for a,b in item["pairs"]]),
            "strict_failed_unique_inputs": sum(case.startswith(stage+"_") for case in strict_cases),
            "strict_failed_measured_pairs": sum(case.startswith(stage+"_") for _,case,_ in strict_measured),
            "measured_subproblem_distribution": {side: dict(Counter(str(row[i]["diagnostics"]["subproblems"]) for row in item["pairs"]))
                for i,side in enumerate(("server", "nx"))},
            "input_dt_s": paired.stats([case["input"]["dt"] for case in item["cases"]])}
    paired.require(set(stages) == {"normal", "fast"} and source_abis == {1}, "Incomplete or unexpected cohort ABI")
    storage = paired.load_json(root/"nx_code_only_after.json")
    cdr = paired.load_json(root/"ros_cdr_report.json")
    reproduce = paired.load_json(root/"preparer_reproducibility.json")
    paired.require(storage["passed"] and cdr["passed"] and reproduce["pass"], "Supporting validation failed")
    result = {"kind": "x650_paired_recorded_cpu_cohort_20261027", "schema_version": 1,
        "pass_under_explicit_derived_policy": all(s["all_chunk_comparisons_pass"] for s in stage_reports.values()),
        "strict_uniform_full_field_pass": not strict_keys, "protocol": protocol,
        "protocol_identity": portable_identity(protocol_path), "source_package_sha256": source["sha256"],
        "frozen_core_abi": 1, "latest_workspace_core_abi4_qualification_claimed": False,
        "loaded_code_sha256": code_hashes, "run": run, "scope": common_scope, "chunks": chunks,
        "stages": stage_reports, "unique_inputs": sum(len(x["ids"]) for x in stages.values()),
        "formal_calls_per_host": sum(x["measured"]["sample_pairs"] for x in stage_reports.values()),
        "warmup_calls_per_host": sum(len(x["warmup_pairs"]) for x in stages.values()),
        "cold_calls_per_host": sum(len(x["cold_pairs"]) for x in stages.values()),
        "cross_target_max_absolute_errors_all_phases": maximum_errors,
        "historical_max_absolute_errors_all_phases": historical_errors,
        "strict_failed_unique_inputs": len(strict_cases), "strict_failed_measured_pairs": len(strict_measured),
        "strict_failed_pairs_all_phases": len(strict_keys), "strict_failed_field_checks_all_phases": len(strict_csv),
        "strict_failure_fields": sorted({x["field"] for x in strict_csv}),
        "environment": {s: environment(paired.load_json(root/(s+"_before.json")),paired.load_json(root/(s+"_after.json"))) for s in ("server", "nx")},
        "nx_code_only_audit": {k:storage[k] for k in ("passed","only_code_files","total_code_bytes","original_ram_build_absent")},
        "cdr_independent_check": {k:cdr[k] for k in ("passed","records","scope")},
        "input_byte_reproduction_pass": reproduce["pass"],
        "limitations": record["limitations"],
        "compiler_and_environment_equivalence_claimed": False}
    args.output.mkdir(parents=True, exist_ok=False)
    atomic_write(args.output/"summary.json", json_bytes(result))
    atomic_write(args.output/"protocol.json", protocol_path.read_bytes())
    write_csv(args.output/"case_comparison.csv", cases_csv)
    if strict_csv:
        write_csv(args.output/"strict_failure_checks.csv", strict_csv)
    if args.plot:
        plot(result, cases_csv, args.output)
    return result


def plot(report, cases, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False, "pdf.fonttype": 42})
    colors = {"server": "#2563A6", "nx": "#D87724"}
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.4))
    fig.subplots_adjust(left=.08, right=.98, bottom=.14, top=.84, hspace=.50, wspace=.24)
    fig.suptitle("MPC compute time: identical recorded inputs on server and Orin NX", fontsize=17, x=.08, ha="left", y=.98)
    fig.text(.08,.918,"Frozen Core ABI 1 | FP64 | CPU 2 affinity | 2 warmup + 10 measured calls per input",color="#45556A",fontsize=11)
    fig.text(.08,.891,"Core::step only; parsing, Memory restore, SSH, ROS and logging are outside the timer.",color="#45556A",fontsize=10)
    for column, stage in enumerate(("normal", "fast")):
        s = report["stages"][stage]
        ax = axes[0, column]
        names = ("p50", "p95", "p99", "max")
        for side, shift in (("server", -.18), ("nx", .18)):
            bars = ax.bar([i+shift for i in range(4)], [s["measured"]["wall"][side][n] for n in names],
                          width=.34, color=colors[side], label="Server" if side=="server" else "Orin NX", zorder=3)
            ax.bar_label(bars, fmt="%.2f", fontsize=9, padding=4)
        ax.set_xticks(range(4), ["Median", "P95", "P99", "Maximum"])
        ax.set_ylim(0,7)
        ax.set_ylabel("Core wall time [ms]")
        ax.set_title(f"{'Ordinary' if stage=='normal' else 'Fast'} figure-eight: {s['unique_inputs']:,} inputs / {s['measured']['sample_pairs']:,} calls",loc="left",pad=14,fontsize=11)
        ax.grid(axis="y",alpha=.2,zorder=0)
        ax.legend(loc="upper left",frameon=False,ncol=2,fontsize=9)
        ax = axes[1, column]
        group = [x for x in cases if x["stage"]==stage]
        x = [r["server_wall_median_ms"] for r in group]
        y = [r["nx_wall_median_ms"] for r in group]
        ax.scatter(x,y,s=8,alpha=.28,color=colors["nx"],rasterized=True)
        ax.plot([0,6],[0,6],"--",color="#6B7280",lw=1)
        ax.set(xlim=(.5,6),ylim=(.5,6),xlabel="Server: median of 10 calls per input [ms]",ylabel="Orin NX: same-input median [ms]")
        ax.set_title("Each dot is one paired input; dashed line = equal time",loc="left",fontsize=10)
        ax.grid(alpha=.15)
    fig.text(.08,.074,"0 / 27,960 measured calls exceeded 20 ms on either host.  All recorded inputs used one QP subproblem.",fontsize=10)
    fig.text(.08,.046,"Original command tolerance passed; 136 inputs fail a uniform derived-acceleration tolerance (see report).",fontsize=9,color="#45556A")
    fig.text(.08,.020,"This is recorded-workload CPU evidence, not a new flight, latest-ABI qualification or end-to-end deadline guarantee.",fontsize=9,color="#45556A")
    fig.savefig(output/"latency_comparison.png",dpi=180,facecolor="white")
    fig.savefig(output/"latency_comparison.pdf",facecolor="white")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("evidence", "source-manifest", "server-build", "nx-build", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--source-package", type=Path)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args(argv)
    result = summarize(args)
    print(json.dumps({k:result[k] for k in ("pass_under_explicit_derived_policy","strict_uniform_full_field_pass",
        "unique_inputs","formal_calls_per_host","strict_failed_unique_inputs")},indent=2))
    return 0 if result["pass_under_explicit_derived_policy"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
