#!/usr/bin/env python3
"""Run one RAM-input NX benchmark and save all evidence on the workstation.

No scp, remote input file, redirect, tee or per-case process is used. SSH I/O and
local persistence surround, rather than enter, the native timed computation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
import time

from prepare_x650_orin_benchmark import atomic_write, identity, json_bytes, strict_json


HEX = re.compile(r"[0-9a-f]{64}\Z")
HOST = re.compile(r"[A-Za-z0-9][A-Za-z0-9.:-]*\Z")
USER = re.compile(r"[A-Za-z_][A-Za-z0-9_-]*\Z")


def digest_argument(value: str) -> str:
    if not HEX.fullmatch(value):
        raise ValueError("Expected a lowercase SHA-256 digest")
    return value


def remote_path(value: str) -> str:
    if (not value.startswith("/") or len(value) > 2048
            or any(ord(c) < 32 or ord(c) == 127 for c in value)
            or ".." in PurePosixPath(value).parts):
        raise ValueError("Remote code paths must be explicit absolute paths without control characters or '..'")
    return value


def make_command(args, input_sha: str) -> list[str]:
    if not HOST.fullmatch(args.host) or not USER.fullmatch(args.user):
        raise ValueError("Invalid SSH hostname or username")
    if not 1 <= args.port <= 65535:
        raise ValueError("Invalid SSH port")
    binary, predictor, library_dir = (remote_path(v) for v in
                                     (args.binary, args.predictor, args.library_dir))
    native = ["env", "OPENBLAS_NUM_THREADS=1", "OMP_NUM_THREADS=1", "MKL_NUM_THREADS=1",
              "LD_LIBRARY_PATH=" + library_dir, binary,
              "--predictor", predictor, "--predictor-sha256", digest_argument(args.predictor_sha256),
              "--input-sha256", digest_argument(input_sha)]
    # Disable application crash dumps. The command has no path-based output and
    # every user-supplied argument receives POSIX shell quoting exactly once.
    remote = "ulimit -c 0 && exec " + " ".join(shlex.quote(v) for v in native)
    command = [args.ssh, "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
               "-o", "StrictHostKeyChecking=yes", "-o", "IdentitiesOnly=yes",
               "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=4",
               "-p", str(args.port)]
    if args.identity_file is not None:
        command.extend(["-i", str(args.identity_file.resolve())])
    if getattr(args, "known_hosts_file", None) is not None:
        command.extend(["-o", "UserKnownHostsFile=" + str(args.known_hosts_file.resolve())])
    return command + [args.user + "@" + args.host, remote]


def finite_number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("Invalid finite " + name)
    return value


def check_sample(sample: dict, case: dict, case_index: int, round_index: int,
                 tolerance: float) -> bool:
    """Independently classify one raw call; never trust the reported pass flag."""
    if sample.get("case_index") != case_index or sample.get("round") != round_index:
        raise ValueError("Sample case/round ordering differs")
    for name in ("core_step_wall_ns", "core_step_thread_cpu_ns"):
        value = sample.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError("Invalid positive integer " + name)
    wall = sample["core_step_wall_ns"]
    for name, expected in (("exceeded_20ms_target", wall > 20_000_000),
                           ("exceeded_50ms_wall", wall > 50_000_000)):
        if sample.get(name) is not expected:
            raise ValueError("Sample deadline classification differs")
    flags = ("returned", "core_error", "unexpected_exception", "memory_unchanged",
             "memory_commit_valid", "oracle_checked", "valid", "production_deadline_error")
    if any(not isinstance(sample.get(name), bool) for name in flags):
        raise ValueError("Sample boolean status fields differ")
    if sum(sample[name] for name in ("returned", "core_error", "unexpected_exception")) != 1:
        raise ValueError("Sample return/error classification differs")
    error = sample.get("error")
    if error is not None and not isinstance(error, str):
        raise ValueError("Sample error must be a string or null")
    if sample["production_deadline_error"] is not ("deadline exceeded" in (error or "")):
        raise ValueError("Production deadline error classification differs")
    returned = sample["returned"]
    if sample["oracle_checked"] is not returned:
        raise ValueError("Oracle checked status differs from return status")
    oracle_match = False
    if returned:
        if error is not None:
            raise ValueError("Returned call also reports an exception")
        output = sample.get("output")
        if not isinstance(output, dict):
            raise ValueError("Returned call has no output")
        # Every exported result must be finite, including fields outside the
        # smaller historical oracle. Lengths are independently fixed by ABI.
        dimensions = {"command": 5, "position_reference": 3, "velocity_feedforward": 3,
                      "acceleration_feedforward": 3, "joint_velocity": 2,
                      "joint_reference": 2, "joint_velocity_reference": 2,
                      "joint_acceleration_reference": 2, "nominal_acceleration": 3,
                      "tip_actual": 3}
        finite_output = True
        for name, size in dimensions.items():
            values = output.get(name)
            if not isinstance(values, list) or len(values) != size:
                raise ValueError("Output ABI dimensions differ: " + name)
            for value in values:
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    finite_output = False
        maximum = 0.
        oracle_match = finite_output
        for name, wanted in case["oracle"].items():
            for actual, expected in zip(output[name], wanted):
                if not isinstance(actual, (int, float)) or isinstance(actual, bool) or not math.isfinite(actual):
                    oracle_match = False
                    continue
                delta = abs(actual - expected)
                maximum = max(maximum, delta)
                oracle_match = oracle_match and delta <= (1e-9 if name == "position_reference" else tolerance)
        if sample.get("oracle_match") is not oracle_match:
            raise ValueError("Reported oracle match differs from workstation recomputation")
        if not math.isclose(finite_number(sample.get("oracle_max_absolute_error"), "oracle error"),
                            maximum, rel_tol=1e-12, abs_tol=1e-15):
            raise ValueError("Reported oracle error differs from workstation recomputation")
        if sample.get("step_count_after") != case["memory"]["step_count"] + 1:
            raise ValueError("Returned call did not commit exactly one restored step")
    elif (sample.get("output") is not None or sample.get("oracle_match") is not None
          or sample.get("oracle_max_absolute_error") is not None):
        raise ValueError("Failed call reports a fabricated output/oracle result")
    diagnostics = sample.get("diagnostics", {})
    for name in ("accepted", "nonlinear_passed", "actual_bounds"):
        if not isinstance(diagnostics.get(name), bool):
            raise ValueError("Missing diagnostic acceptance flags")
    compute_ms = finite_number(diagnostics.get("compute_ms"), "diagnostic compute time")
    if compute_ms < 0:
        raise ValueError("Negative diagnostic compute time")
    valid = (returned and oracle_match and sample["memory_commit_valid"]
             and all(diagnostics[name] for name in ("accepted", "nonlinear_passed", "actual_bounds"))
             and wall <= 50_000_000 and compute_ms <= 50.)
    if sample["valid"] is not valid:
        raise ValueError("Reported sample validity differs from independent checks")
    return valid


def check_summary(summary: dict, samples: list[dict], validity: list[bool]) -> None:
    count = len(samples)
    counters = {"sample_count": count, "returned_count": sum(s["returned"] for s in samples),
                "valid_count": sum(validity), "oracle_checked_count": sum(s["oracle_checked"] for s in samples),
                "oracle_match_count": sum(s["oracle_match"] is True for s in samples),
                "core_error_count": sum(s["core_error"] for s in samples),
                "unexpected_exception_count": sum(s["unexpected_exception"] for s in samples),
                "exceeded_20ms_target_count": sum(s["exceeded_20ms_target"] for s in samples),
                "exceeded_50ms_wall_count": sum(s["exceeded_50ms_wall"] for s in samples),
                "production_deadline_error_count": sum(s["production_deadline_error"] for s in samples),
                "failed_calls_with_unchanged_memory": sum(not s["returned"] and s["memory_unchanged"] for s in samples)}
    if any(summary.get(key) != value for key, value in counters.items()):
        raise ValueError("Summary counters differ from all raw calls")
    if (summary.get("all_calls_valid") is not all(validity)
            or summary.get("statistics_include_all_failed_and_slow_calls") is not True
            or summary.get("percentile_method") != "nearest_rank"):
        raise ValueError("Summary completeness/validity classification differs")
    for key, field in (("wall", "core_step_wall_ns"), ("thread_cpu", "core_step_thread_cpu_ns")):
        stats = summary.get(key, {})
        values = sorted(s[field] for s in samples)
        expected = {"sample_count": count}
        if count:
            expected.update(min_ns=values[0], max_ns=values[-1],
                            median_ns=values[math.ceil(.5 * count) - 1],
                            p95_ns=values[math.ceil(.95 * count) - 1],
                            p99_ns=values[math.ceil(.99 * count) - 1])
        if any(stats.get(name) != value for name, value in expected.items()):
            raise ValueError("Summary percentile differs from all raw calls")
        if count and not math.isclose(finite_number(stats.get("mean_ns"), "mean time"),
                                      sum(values) / count, rel_tol=1e-12, abs_tol=1e-6):
            raise ValueError("Summary mean differs from all raw calls")


def validate_result(result: dict, bundle: dict, input_sha: str, expected: dict) -> dict:
    if not isinstance(result, dict):
        raise ValueError("Native output must be one JSON object")
    if result.get("kind") != "x650_mpc_memory_benchmark_result_v1" or result.get("schema_version") != 1:
        raise ValueError("Unexpected native result schema")
    if result.get("complete") is not True:
        raise ValueError("Native benchmark did not complete: " + str(result.get("error")))
    if result.get("backend") != "cpu" or result.get("mode") != "fixed_snapshot_paired":
        raise ValueError("Unexpected benchmark backend or replay mode")
    if result.get("input_sha256") != input_sha:
        raise ValueError("NX did not report the exact streamed input digest")
    if result.get("provenance") != bundle["provenance"] or result.get("run") != bundle["run"]:
        raise ValueError("NX result does not bind the input provenance and run settings")
    if result.get("case_count") != len(bundle["cases"]):
        raise ValueError("NX result case count differs")
    if [x["id"] for x in result.get("cases", [])] != [x["id"] for x in bundle["cases"]]:
        raise ValueError("NX result case IDs differ")
    count = len(bundle["cases"])
    if len(result.get("samples", [])) != count * bundle["run"]["measured_rounds"]:
        raise ValueError("Measured failures or calls disappeared from the result")
    if len(result.get("warmup_samples", [])) != count * bundle["run"]["warmup_rounds"]:
        raise ValueError("Warmup failures or calls disappeared from the result")
    loaded = result.get("loaded_code", {})
    for key, digest in expected.items():
        if loaded.get(key, {}).get("sha256") != digest:
            raise ValueError("Unexpected loaded " + key + " binary")
    scope = result.get("scope", {})
    for name in ("cuda_implemented", "hardware_closed_loop_claimed", "local_control_stack_latency_measured"):
        if scope.get(name) is not False:
            raise ValueError("CPU snapshot benchmark may not claim CUDA or hardware closed-loop validation")
    tolerance = bundle["run"]["oracle_absolute_tolerance"]
    cold_valid = check_sample(result.get("cold_first_call", {}), bundle["cases"][0], 0, 0, tolerance)
    valid = [cold_valid]
    for phase in ("warmup", "measured"):
        samples = result["warmup_samples"] if phase == "warmup" else result["samples"]
        phase_valid = [check_sample(s, bundle["cases"][i % count], i % count, i // count, tolerance)
                       for i, s in enumerate(samples)]
        check_summary(result.get("summary", {}).get(phase, {}), samples, phase_valid)
        valid.extend(phase_valid)
    if result.get("all_calls_valid") is not all(valid):
        raise ValueError("Top-level validity differs from retained cold/warmup/measured calls")
    return {"input_and_code_bound": True, "complete": True,
            "all_calls_valid": all(valid), "workstation_raw_result_recomputed": True,
            "measured_samples": len(result["samples"])}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="Workstation-side prepared JSON")
    parser.add_argument("--input-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True, help="New workstation-only result directory")
    parser.add_argument("--host", default="192.168.30.145")
    parser.add_argument("--user", default="jetson")
    parser.add_argument("--port", type=int, default=22, help="SSH port, not NoMachine NX port 4000")
    parser.add_argument("--ssh", default="ssh")
    parser.add_argument("--identity-file", type=Path)
    parser.add_argument("--known-hosts-file", type=Path,
                        help="Workstation-side host key file, StrictHostKeyChecking stays enabled")
    parser.add_argument("--binary", required=True)
    parser.add_argument("--library-dir", required=True)
    parser.add_argument("--predictor", required=True)
    parser.add_argument("--binary-sha256", required=True)
    parser.add_argument("--core-sha256", required=True)
    parser.add_argument("--optimization-sha256", required=True)
    parser.add_argument("--predictor-sha256", required=True)
    parser.add_argument("--timeout", type=float, default=1800.)
    args = parser.parse_args(argv)
    raw = args.input.read_bytes()
    input_sha = hashlib.sha256(raw).hexdigest()
    if input_sha != digest_argument(args.input_sha256):
        raise ValueError("Local input no longer matches its preparation identity")
    if len(raw) > 32 * 1024 * 1024:
        raise ValueError("Input exceeds the native 32 MiB RAM input limit")
    bundle = strict_json(raw)
    expected = {"executable": digest_argument(args.binary_sha256),
                "core": digest_argument(args.core_sha256),
                "optimization": digest_argument(args.optimization_sha256),
                "predictor": digest_argument(args.predictor_sha256)}
    if args.timeout <= 0:
        raise ValueError("Timeout must be positive")
    command = make_command(args, input_sha)
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    # communicate() uses one process and byte pipes, preloads all input, and
    # drains output without invoking a local shell. Native code emits no output
    # until every timed sample is finished; SSH backpressure cannot enter timers.
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    timed_out = False
    try:
        stdout, stderr = process.communicate(input=raw, timeout=args.timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        process.kill()
        stdout, stderr = process.communicate()
    elapsed = time.perf_counter() - started
    atomic_write(args.output / "native_stdout.json", stdout)
    atomic_write(args.output / "ssh_stderr.txt", stderr)
    record = {"kind": "x650_orin_workstation_transport_v1", "complete": False,
              "pass": False, "exit_code": process.returncode, "timed_out": timed_out,
              "input": identity(args.input), "expected_loaded_code_sha256": expected,
              "transport_wall_s": elapsed, "transport_wall_is_not_algorithm_time": True,
              "ssh_command": command, "remote_input_files_created": False,
              "remote_result_files_created": False,
              "stdout": identity(args.output / "native_stdout.json"),
              "stderr": identity(args.output / "ssh_stderr.txt")}
    try:
        if timed_out or process.returncode != 0:
            raise RuntimeError("SSH/native process did not finish successfully")
        result = strict_json(stdout)
        record.update(validate_result(result, bundle, input_sha, expected))
        # A fully captured failed call is still a valid experiment record, but
        # must not become a passing performance/correctness qualification.
        record["pass"] = record["all_calls_valid"]
    except Exception as error:
        record["error"] = repr(error)
    atomic_write(args.output / "transport_report.json", json_bytes(record))
    print(json.dumps({k: record.get(k) for k in
                      ("complete", "pass", "exit_code", "timed_out", "all_calls_valid", "error")}))
    return 0 if record["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
