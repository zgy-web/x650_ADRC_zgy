#!/usr/bin/env python3
"""Verify and compare identical-input x86/Orin CPU Core benchmarks offline.

All files are workstation-side evidence. This tool neither connects to a target
nor changes the production controller. Every cold, warmup and measured call is
checked; latency statistics retain failed and slow measured calls.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tarfile

from prepare_x650_orin_benchmark import atomic_write, identity, json_bytes
from run_x650_orin_benchmark import digest_argument, validate_result


OUTPUT_DIMENSIONS = {"command": 5, "position_reference": 3,
    "velocity_feedforward": 3, "acceleration_feedforward": 3,
    "joint_velocity": 2, "joint_reference": 2,
    "joint_velocity_reference": 2, "joint_acceleration_reference": 2,
    "nominal_acceleration": 3, "tip_actual": 3}
BUILD_KEYS = {"executable": "executable", "core": "core_library",
              "optimization": "optimization_library", "predictor": "predictor"}
FIELD_POLICIES = ("strict-v1", "recorded-derived-v2")
PRIMARY_ORACLE_FIELDS = {"command", "position_reference", "velocity_feedforward"}
EXPECTED_SCOPE = {
    "timed_function": "x650::Core::step(StepInput), Compact production diagnostics",
    "timed_wall_clock": "CLOCK_MONOTONIC_RAW",
    "timed_thread_clock": "CLOCK_THREAD_CPUTIME_ID",
    "clock_order": "thread begin, wall begin, Core::step, wall end, thread end",
    "memory_restore_inside_core_timer": False,
    "input_parse_hash_load_inside_core_timer": False,
    "oracle_validation_inside_core_timer": False,
    "production_deadline_ms": 50, "control_update_target_ms": 20,
    "runtime_python_calls": 0, "runtime_ros_calls": 0,
    "application_file_writes": 0, "measurement_loop_output_bytes": 0,
    "network_calls_in_measurement": 0, "cuda_implemented": False,
    "hardware_closed_loop_claimed": False,
    "local_control_stack_latency_measured": False,
    "all_measured_calls_retained": True,
    "wall_minus_thread_is_not_pure_scheduler_delay": True,
}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def load_json(path: Path):
    def constant(value):
        raise ValueError("Nonfinite JSON constant: " + value)
    def pairs(items):
        result = {}
        for name, value in items:
            require(name not in result, "Duplicate JSON object key: " + name)
            result[name] = value
        return result
    return json.loads(path.read_bytes(), parse_constant=constant, object_pairs_hook=pairs)


def integer(value, name, minimum=0):
    require(type(value) is int and value >= minimum, "Invalid integer " + name)
    return value


def finite(value, name):
    require(type(value) in (int, float) and math.isfinite(value), "Invalid number " + name)
    return value


def verify_identity(path: Path, record: dict, name: str):
    actual = identity(path)
    require(actual["sha256"] == digest_argument(record.get("sha256", ""))
            and actual["bytes"] == record.get("bytes"), name + " identity differs")
    return actual


def verify_sources(manifest_path: Path, package_path: Path | None = None):
    document = load_json(manifest_path)
    package = document.get("source_package", document)
    require(package.get("kind") == "x650_orin_code_only_source_package_v1",
            "Unexpected source package manifest kind")
    archive = package["archive"]
    path = package_path if package_path is not None else Path(archive["path"])
    verified = verify_identity(path, archive, "Source package")
    sources = package["sources"]
    require(isinstance(sources, list) and sources, "Missing source member manifest")
    expected = {row["path"]: row for row in sources}
    require(len(expected) == len(sources), "Duplicate source member manifest")
    with tarfile.open(fileobj=io.BytesIO(path.read_bytes()), mode="r:gz") as tar:
        members = tar.getmembers()
        require(len(members) == len(expected) and {m.name for m in members} == set(expected),
                "Source tar member list differs")
        for member in members:
            require(member.isfile() and not member.issym() and not member.islnk(),
                    "Source package must contain only regular files")
            data = tar.extractfile(member).read()
            row = expected[member.name]
            require(len(data) == row["bytes"] and hashlib.sha256(data).hexdigest() == row["sha256"],
                    "Source tar member differs: " + member.name)
    return verified, sources


def verify_build(build: dict, source_sha: str, architecture: str):
    require(build.get("complete") is True, "Build did not complete")
    require(build.get("architecture") == architecture, "Unexpected target architecture")
    require(build.get("package_sha256") == source_sha, "Build source package SHA differs")
    commands = build.get("commands", [])
    configure = [row for row in commands if isinstance(row, list) and row
                 and Path(row[0]).name in ("cmake", "cmake.exe") and "-S" in row]
    require(len(configure) == 1 and "-DCMAKE_BUILD_TYPE=Release" in configure[0],
            "Expected one explicit CMake Release configure command")
    settings = sorted(x for x in configure[0] if isinstance(x, str) and x.startswith("-D"))
    forbidden = ("fast-math", "Ofast", "CXX_FLAGS", "C_FLAGS", "CXX_COMPILER_LAUNCHER")
    require(not any(token in option for option in settings for token in forbidden),
            "Unqualified compiler flag override in build")
    files = build.get("files", {})
    code = {key: digest_argument(files.get(build_key, {}).get("sha256", ""))
            for key, build_key in BUILD_KEYS.items()}
    for build_key in BUILD_KEYS.values():
        integer(files[build_key].get("bytes"), "ELF size", 1)
    return code, settings


def verify_extra_sample(sample, case):
    # The native format exports status evidence, not a complete post-Memory
    # snapshot. These checks strengthen its contract without claiming to
    # reconstruct hidden history fields from a boolean.
    before = integer(case["memory"]["step_count"], "restored step count")
    after = integer(sample["step_count_after"], "step count after")
    if sample["returned"]:
        require(sample["memory_commit_valid"] is True and sample["memory_unchanged"] is False
                and after == before + 1, "Returned call Memory commit contract differs")
    else:
        require(sample["memory_unchanged"] is True and sample["memory_commit_valid"] is False
                and after == before, "Failed call mutated restored Memory")
    d = sample["diagnostics"]
    subproblems = integer(d.get("subproblems"), "subproblems")
    require(subproblems <= 3, "Unexpected original Core attempt count")
    integer(d.get("active_constraints"), "active constraints")
    for key in ("qp_rows", "qp_columns", "candidate_substeps"):
        integer(d.get(key), key)
    require(type(d.get("previous_plan_fallback")) is bool, "Missing fallback flag")
    attempts = d.get("attempts")
    require(isinstance(attempts, list) and len(attempts) <= 3,
            "Missing or unbounded attempt diagnostics")
    for a in attempts:
        require(type(a.get("passed")) is bool and type(a.get("actual_bounds")) is bool,
                "Invalid attempt acceptance flags")
        require(finite(a.get("elapsed_ms"), "attempt elapsed") >= 0,
                "Negative attempt elapsed")
        for key in ("q_error", "dq_error"):
            if a.get(key) is not None:
                finite(a[key], key)


def verify_side(folder: Path, bundle: dict, input_identity: dict,
                expected: dict, cpu: int):
    transport_path = folder / "transport_report.json"
    transport = load_json(transport_path)
    require(transport.get("complete") is True and transport.get("exit_code") == 0
            and transport.get("timed_out") is False, "Runner transport did not complete")
    require(transport.get("transport_wall_is_not_algorithm_time") is True
            and transport.get("remote_input_files_created") is False
            and transport.get("remote_result_files_created") is False,
            "Runner transport/storage boundary differs")
    require(transport.get("input", {}).get("sha256") == input_identity["sha256"]
            and transport.get("input", {}).get("bytes") == input_identity["bytes"],
            "Runner input identity differs")
    require(transport.get("expected_loaded_code_sha256") == expected,
            "Runner expected ELF hashes differ from build")
    stdout_path = folder / "native_stdout.json"
    verify_identity(stdout_path, transport["stdout"], "Native stdout")
    verify_identity(folder / "ssh_stderr.txt", transport["stderr"], "SSH stderr")
    result = load_json(stdout_path)
    checked = validate_result(result, bundle, input_identity["sha256"], expected)
    require(transport.get("pass") is checked["all_calls_valid"]
            and transport.get("all_calls_valid") is checked["all_calls_valid"],
            "Runner pass classification differs")
    require(result.get("allowed_cpus") == [cpu], "Native process was not pinned to input CPU")
    scope = result.get("scope", {})
    for key, value in EXPECTED_SCOPE.items():
        require(type(scope.get(key)) is type(value) and scope[key] == value,
                "Timing boundary differs: " + key)
    expected_tolerances = {key: 1e-9 if key == "position_reference" else 1e-7
                           for key in OUTPUT_DIMENSIONS}
    require(scope.get("oracle_field_absolute_tolerances") == expected_tolerances,
            "Output tolerance contract differs")
    require(result.get("clock_calibration", {}).get("subtracted_from_samples") is False,
            "Clock calibration may not be subtracted")
    for phase in ("cold_first_call", "warmup_samples", "samples"):
        rows = [result[phase]] if phase == "cold_first_call" else result[phase]
        for row in rows:
            verify_extra_sample(row, bundle["cases"][row["case_index"]])
    return result, {"transport": identity(transport_path), "stdout": identity(stdout_path),
                    "validation": checked, "loaded_code_sha256": expected}


def stats(values):
    values = sorted(values)
    result = {"sample_count": len(values), "percentile_method": "nearest_rank"}
    if values:
        result.update(min=values[0], mean=sum(values)/len(values), max=values[-1])
        for name, p in (("p50", .5), ("p95", .95), ("p99", .99), ("p999", .999)):
            result[name] = values[math.ceil(len(values)*p)-1]
    return result


def comparison(rows):
    # All timing rows stay in statistics, including failed or asymmetric calls.
    server, nx = [a for a, _ in rows], [b for _, b in rows]
    value = {"sample_pairs": len(rows), "all_failed_and_slow_calls_included": True,
        "server_invalid_calls": sum(not a["valid"] for a in server),
        "nx_invalid_calls": sum(not a["valid"] for a in nx),
        "both_returned": sum(a["returned"] and b["returned"] for a, b in rows),
        "status_mismatches": sum(any(a[k] != b[k] for k in
            ("returned", "core_error", "unexpected_exception", "valid")) for a, b in rows)}
    for name, field in (("wall", "core_step_wall_ns"),
                        ("thread_cpu", "core_step_thread_cpu_ns")):
        value[name] = {"unit": "ms", "server": stats([a[field]/1e6 for a in server]),
            "nx": stats([a[field]/1e6 for a in nx]),
            "nx_minus_server": stats([(b[field]-a[field])/1e6 for a, b in rows]),
            "server_over_nx_ratio": stats([a[field]/b[field] for a, b in rows])}
    value["deadlines"] = {side: {"over_20ms": sum(x["exceeded_20ms_target"] for x in data),
                                  "over_50ms": sum(x["exceeded_50ms_wall"] for x in data)}
                          for side, data in (("server", server), ("nx", nx))}
    return value


def field_tolerances(case, policy):
    require(policy in FIELD_POLICIES, "Unknown field policy")
    result = {key: 1e-9 if key == "position_reference" else 1e-7
              for key in OUTPUT_DIMENSIONS}
    if policy == "recorded-derived-v2":
        dt = finite(case["input"]["dt"], "case dt")
        require(dt > 0, "Derivative policy requires positive dt")
        result["joint_acceleration_reference"] = 1e-7 / dt
    return result


def historical_oracles(bundle, policy):
    if policy == "strict-v1":
        return {}
    provenance_cases = bundle["provenance"].get("cases")
    require(isinstance(provenance_cases, list), "Derived policy requires historical provenance cases")
    by_id = {}
    for row in provenance_cases:
        require(isinstance(row, dict) and row.get("id") not in by_id,
                "Duplicate or invalid historical provenance case")
        by_id[row.get("id")] = row.get("historical_output_oracle")
    require(set(by_id) == {case["id"] for case in bundle["cases"]},
            "Historical provenance case IDs differ")
    for case in bundle["cases"]:
        require(set(case["oracle"]) == PRIMARY_ORACLE_FIELDS,
                "Derived policy requires unchanged three-field native oracle")
        oracle = by_id[case["id"]]
        require(isinstance(oracle, dict) and set(oracle) == set(OUTPUT_DIMENSIONS),
                "Historical output oracle must contain all ten fields")
        for field, size in OUTPUT_DIMENSIONS.items():
            values = oracle[field]
            require(isinstance(values, list) and len(values) == size,
                    "Historical output oracle dimensions differ: " + field)
            for value in values:
                finite(value, "historical " + field)
        previous = case["memory"].get("previous_command")
        require(isinstance(previous, list) and len(previous) == 5,
                "Derived policy requires frozen previous command")
        for value in previous:
            finite(value, "previous command")
        field_tolerances(case, policy)
    return by_id


def output_errors(actual, wanted, tolerances):
    errors = {field: max(abs(a-b) for a, b in zip(actual[field], wanted[field]))
              for field in OUTPUT_DIMENSIONS}
    failures = {field: {"maximum_absolute_error": error,
                        "absolute_tolerance": tolerances[field]}
                for field, error in errors.items() if error > tolerances[field]}
    return errors, failures


def derived_acceleration_check(output, case):
    """Check the frozen Core identity separately from the propagated budget.

    The budget below is only floating-point subtraction/division roundoff. It
    cannot excuse an acceleration inconsistent with its own command, even when
    the looser, dimensionally propagated comparison budget would permit it.
    """
    dt, previous = case["input"]["dt"], case["memory"]["previous_command"]
    failures = []
    for joint in range(2):
        command = output["command"][joint + 3]
        expected = (command - previous[joint + 3]) / dt
        actual = output["joint_acceleration_reference"][joint]
        budget = 8 * sys.float_info.epsilon * (
            (abs(command) + abs(previous[joint + 3])) / dt + max(1., abs(expected)))
        error = abs(actual - expected)
        if error > budget:
            failures.append({"joint_index": joint, "actual": actual, "expected": expected,
                             "absolute_error": error, "roundoff_budget": budget})
    return failures


def compare(input_path: Path, server_folder: Path, nx_folder: Path,
            source_manifest: Path, server_build_path: Path, nx_build_path: Path,
            source_package: Path | None = None, field_policy: str = "strict-v1"):
    bundle = load_json(input_path)
    input_identity = identity(input_path)
    require(bundle.get("kind") == "x650_mpc_memory_benchmark_input_v1"
            and bundle.get("schema_version") == 1, "Unexpected input schema")
    run = bundle["run"]
    cpu = integer(run.get("cpu"), "fixed CPU")
    require(run.get("oracle_absolute_tolerance") == 1e-7, "Original tolerance changed")
    integer(run.get("warmup_rounds"), "warmup rounds")
    integer(run.get("measured_rounds"), "measured rounds", 1)
    require(bundle["config"].get("max_dt") == .05, "Original production dt gate changed")
    historical = historical_oracles(bundle, field_policy)
    source, members = verify_sources(source_manifest, source_package)
    server_code, server_settings = verify_build(load_json(server_build_path), source["sha256"], "x86_64")
    nx_code, nx_settings = verify_build(load_json(nx_build_path), source["sha256"], "aarch64")
    require(server_settings == nx_settings, "CMake configure settings differ between targets")
    server, server_evidence = verify_side(server_folder, bundle, input_identity, server_code, cpu)
    nx, nx_evidence = verify_side(nx_folder, bundle, input_identity, nx_code, cpu)
    require(server["scope"] == nx["scope"], "Full timing scopes differ between targets")
    for result in (server, nx):
        for case, actual in zip(bundle["cases"], result["cases"]):
            require(actual.get("input_t_s") == case["input"]["t"]
                    and actual.get("dt_s") == case["input"]["dt"]
                    and actual.get("step_count_before") == case["memory"]["step_count"],
                    "Exported case metadata differs from input")
    max_errors = {key: 0. for key in OUTPUT_DIMENSIONS}
    historical_max = {side: {key: 0. for key in OUTPUT_DIMENSIONS} for side in ("server", "nx")}
    failures, historical_failures, relation_failures, strict_failures, checked = [], [], [], [], 0
    historical_checked, relation_checked = {"server": 0, "nx": 0}, {"server": 0, "nx": 0}
    for phase in ("cold_first_call", "warmup_samples", "samples"):
        left = [server[phase]] if phase == "cold_first_call" else server[phase]
        right = [nx[phase]] if phase == "cold_first_call" else nx[phase]
        for a, b in zip(left, right):
            case = bundle["cases"][a["case_index"]]
            key = {"phase": phase, "case_index": a["case_index"], "case_id": case["id"],
                   "round": a["round"]}
            tolerances = field_tolerances(case, field_policy)
            strict_tolerances = field_tolerances(case, "strict-v1")
            if historical:
                oracle = historical[case["id"]]
                for side, sample in (("server", a), ("nx", b)):
                    if not sample["returned"]:
                        continue
                    historical_checked[side] += 1
                    errors, mismatched = output_errors(sample["output"], oracle, tolerances)
                    for field, error in errors.items():
                        historical_max[side][field] = max(historical_max[side][field], error)
                    if mismatched:
                        historical_failures.append({**key, "side": side, "fields": list(mismatched),
                                                    "field_errors": mismatched})
                    _, strict = output_errors(sample["output"], oracle, strict_tolerances)
                    if strict:
                        strict_failures.append({**key, "comparison": "historical", "side": side,
                                                "fields": list(strict), "field_errors": strict})
                    relation_checked[side] += 1
                    relationship = derived_acceleration_check(sample["output"], case)
                    if relationship:
                        relation_failures.append({**key, "side": side, "joint_errors": relationship})
            if not (a["returned"] and b["returned"]):
                continue
            checked += 1
            errors, mismatched = output_errors(a["output"], b["output"], tolerances)
            for field, error in errors.items():
                max_errors[field] = max(error, max_errors[field])
            if mismatched:
                failures.append({**key, "fields": list(mismatched), "field_errors": mismatched})
            _, strict = output_errors(a["output"], b["output"], strict_tolerances)
            if strict:
                strict_failures.append({**key, "comparison": "cross_target", "fields": list(strict),
                                        "field_errors": strict})
    rows = list(zip(server["samples"], nx["samples"]))
    groups = {}
    for dimension in ("case", "dt_s", "subproblems", "active_constraints"):
        grouped = {}
        for a, b in rows:
            case = bundle["cases"][a["case_index"]]
            if dimension == "case":
                label = case["id"]
            elif dimension == "dt_s":
                label = format(case["input"]["dt"], ".17g")
            else:
                label = f"server={a['diagnostics'][dimension]},nx={b['diagnostics'][dimension]}"
            grouped.setdefault(label, []).append((a, b))
        groups[dimension] = [{"group": name, **comparison(grouped[name])} for name in sorted(grouped)]
    both_valid = server["all_calls_valid"] and nx["all_calls_valid"]
    strict_keys = {(row["phase"], row["case_index"], row["round"]) for row in strict_failures}
    strict_measured = {key for key in strict_keys if key[0] == "samples"}
    return {"schema_version": 1, "kind": "x650_same_input_paired_cpu_comparison_v1",
        "complete": True, "pass": bool(both_valid and not failures and not historical_failures
                                         and not relation_failures),
        "field_policy": field_policy,
        "strict_full_field_passed": not strict_failures,
        "strict_failure_count": len(strict_keys),
        "strict_failure_count_unit": "unique logical phase/case/round pairs failing any strict historical or cross-target field check",
        "strict_measured_failure_count": len(strict_measured),
        "strict_failure_check_count": len(strict_failures),
        "strict_failures": strict_failures,
        "input": input_identity, "source_package": source,
        "source_members": members, "run": run,
        "builds": {"server": identity(server_build_path), "nx": identity(nx_build_path)},
        "evidence": {"server": server_evidence, "nx": nx_evidence},
        "scope": server["scope"], "independent_output_pairs_checked": checked,
        "cross_target_absolute_tolerances": {key: 1e-9 if key == "position_reference" else
            ("1e-7 / case.input.dt" if field_policy == "recorded-derived-v2"
             and key == "joint_acceleration_reference" else 1e-7) for key in OUTPUT_DIMENSIONS},
        "field_tolerances_by_case": [{"case_id": case["id"], "dt_s": case["input"]["dt"],
            "absolute_tolerances": field_tolerances(case, field_policy)} for case in bundle["cases"]],
        "historical_output_pairs_checked": historical_checked,
        "historical_max_absolute_errors": historical_max,
        "historical_output_mismatches": historical_failures,
        "derived_acceleration_relationship_checks": relation_checked,
        "derived_acceleration_relationship_failures": relation_failures,
        "derived_acceleration_relationship": "(output.command[j+3] - frozen Memory.previous_command[j+3]) / input.dt",
        "derived_acceleration_roundoff_budget": "8 * binary64_epsilon * ((abs(command) + abs(previous)) / dt + max(1, abs(expected_acceleration)))",
        "cross_target_max_absolute_errors": max_errors, "cross_target_mismatches": failures,
        "measured": comparison(rows), "groups": groups,
        "cold_first_call": comparison([(server["cold_first_call"], nx["cold_first_call"])]),
        "warmup": comparison(list(zip(server["warmup_samples"], nx["warmup_samples"]))),
        "limitations": ["Each case/round is a logical workload pair, not simultaneous sampling.",
            "Repeated frozen Memory snapshots do not constitute continuous closed-loop control.",
            "CPU indices pin a local core; they do not assert equal core hardware or frequency.",
            "Compiler versions, libraries, power, clocks and machine load remain target-specific.",
            "Ratios describe these runs and this corpus, not a universal hardware speedup.",
            "The native Memory commit/rollback flags are checked; a full post-Memory image is not exported.",
            "No end-to-end ROS/network/actuator timing or hard worst-case guarantee is claimed."],
        "compiler_and_environment_equivalence_claimed": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("input", "server-result", "nx-result", "source-manifest", "server-build", "nx-build", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--source-package", type=Path,
                        help="Optional relocated source tar; its original bytes/SHA must match manifest")
    parser.add_argument("--field-policy", choices=FIELD_POLICIES, default="strict-v1",
                        help="Explicit acceptance policy; strict-v1 remains the default")
    args = parser.parse_args(argv)
    report = compare(args.input, args.server_result, args.nx_result, args.source_manifest,
                     args.server_build, args.nx_build, args.source_package, args.field_policy)
    atomic_write(args.output, json_bytes(report))
    print(json.dumps({key: report[key] for key in ("complete", "pass", "independent_output_pairs_checked")}))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
