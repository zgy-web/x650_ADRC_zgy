#!/usr/bin/env python3
"""Replay original reconstructed C++ fixtures against isolated old/new binaries.

The native executable runs once per version/call, restoring identical original
memory and actual history. Native timing excludes Python/JSON/file operations.
No simulator, ROS publisher, actuator, model fitting or source rewrite is used.
Every raw result, stderr/stdout, exit and independent audit is retained, including
failed/slow calls. A failed command/audit makes the overall process nonzero.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import audit_x650_recovery_replay as checker


KIND = "x650_cpp_recovery_recording_regression_v1"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def file_identity(path):
    return checker.identity(path)


def verify_receipt(receipt):
    require(isinstance(receipt, dict) and all(k in receipt for k in ("path", "bytes", "sha256")),
            "Explicit source path/size/SHA receipt required")
    path = Path(receipt["path"])
    require(path.is_absolute() and path.is_file(), "Receipt source must be an existing absolute file")
    actual = file_identity(path)
    require(actual["bytes"] == receipt["bytes"] and actual["sha256"] == receipt["sha256"],
            "Original receipt differs: " + str(path))
    return path


def sha_argument(value):
    require(isinstance(value, str) and len(value) == 64
            and all(x in "0123456789abcdef" for x in value), "Lowercase SHA256 required")
    return value


def config_identity(config):
    raw = json.dumps(config, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return {"canonical_json_sha256": hashlib.sha256(raw).hexdigest(), "value": config}


def load_original(report_path, report_sha):
    require(file_identity(report_path)["sha256"] == sha_argument(report_sha), "Original reconstruction report SHA differs")
    report = checker.strict_json(report_path)
    require(report.get("kind") == "x650_cpp_original_sequence_reconstruction_v1"
            and report.get("schema_version") == 1, "Original reconstruction schema differs")
    require(all(report.get(k) is True for k in ("complete", "all_original_outcomes_reproduced",
                "stopped_at_original_rejection", "memory_reconstructed_in_cpp"))
            and report.get("history_from_proposals") is False
            and report.get("counterfactual_closed_loop_claimed") is False
            and report.get("physical_validation_claimed") is False,
            "Original memory/proposal/source reconstruction did not qualify")
    rows = report.get("rows")
    require(isinstance(rows, list) and len(rows) == 379 and report.get("row_count") == 379
            and report.get("step_count_after") == 378, "Expected original 378 successes and failed call 379")
    recording_sha = sha_argument(report.get("recording_sha256"))
    for key in ("core", "optimization", "predictor"):
        receipt = report.get("loaded_code", {}).get(key, {})
        require(sha_argument(receipt.get("sha256")) == file_identity(receipt["path"])["sha256"],
                "Original loaded library identity changed: " + key)
    cases, canonical = [], None
    for index, row in enumerate(rows, 1):
        require(row.get("sequence") == index, "Original prefix has missing/duplicated calls")
        expected_status = 0 if index < 379 else 2
        require(row.get("original_status") == expected_status
                and row.get("returned") is (index < 379), "Original call status differs")
        if index < 379:
            require(row.get("original_proposal_match") is True
                    and row.get("original_full_result_match") is True
                    and checker.number(row["maximum_proposal_absolute_error"]) <= 1e-8
                    and checker.number(row["maximum_full_result_absolute_error"]) <= 1e-8,
                    "Original successful prefix was not checked against complete recorded results")
        else:
            require(row.get("original_failure_reproduced") is True and row.get("error") == "QP infeasible"
                    and row.get("original_error") == "QP infeasible" and row.get("memory_unchanged") is True,
                    "Original QP failure/rollback did not reproduce")
        fixture_path = verify_receipt(row["fixture"])
        fixture = checker.strict_json(fixture_path)
        require(fixture.get("schema_version") == 1 and fixture.get("label", {}).get("sequence") == index,
                "Fixture sequence/schema binding differs")
        provenance = fixture.get("provenance", {})
        require(provenance.get("recording_sha256") == recording_sha
                and provenance.get("memory_from_original_core_prefix") is True
                and provenance.get("recorded_physical_state") is True,
                "Fixture is not reconstructed from the exact original physical prefix")
        source_row_path = verify_receipt(provenance["row_identity"])
        source_row = checker.strict_json(source_row_path)
        require(source_row.get("sequence") == index, "Original source row sequence differs")
        for key in ("record_identity", "snapshot_identity", "proposal_identity"):
            verify_receipt(source_row[key])
        require(source_row["snapshot_identity"] == provenance["source_snapshot_identity"]
                and source_row["proposal_identity"] == provenance["source_proposal_identity"],
                "Fixture source CDR identity differs from exported raw row")
        current = config_identity(fixture["config"])
        if canonical is None:
            canonical = current
        require(current == canonical, "Original reconstructed fixtures mix controller configurations")
        require(fixture["memory"].get("step_count") == index-1,
                "Fixture memory is not the original successful prefix")
        require(fixture["input"]["t"] == row["input_task_time_s"], "Fixture task time differs")
        cases.append({"sequence": index, "fixture": row["fixture"], "value": fixture,
                      "expectation": "unchanged" if index < 379 else "recovered"})
    return report, cases, canonical


def write_new(path, value):
    with Path(path).open("x", encoding="utf-8", newline="\n") as output:
        output.write(json.dumps(value, indent=2, allow_nan=False)+"\n")


def execute(executable, predictor, predictor_sha, fixture_receipt, output_dir, timeout_s):
    raw_path = output_dir/"raw.json"
    command = [str(executable), str(predictor), predictor_sha, fixture_receipt["path"],
               fixture_receipt["sha256"], str(raw_path)]
    begin = time.time()
    record = {"command": command, "started_unix": begin, "returncode": None,
              "timed_out": False, "spawn_error": None}
    environment = os.environ.copy()
    environment.update(OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    try:
        process = subprocess.run(command, capture_output=True, timeout=timeout_s, env=environment)
        record["returncode"] = process.returncode
        stdout, stderr = process.stdout, process.stderr
    except subprocess.TimeoutExpired as error:
        record["timed_out"] = True
        stdout, stderr = error.stdout or b"", error.stderr or b""
    except OSError as error:
        record["spawn_error"] = str(error)
        stdout, stderr = b"", str(error).encode()
    record["process_wall_s"] = time.time()-begin
    for label, value in (("stdout", stdout), ("stderr", stderr)):
        path = output_dir/(label+".txt")
        with path.open("xb") as stream:
            stream.write(value)
        record[label] = file_identity(path)
    record["raw"] = file_identity(raw_path) if raw_path.is_file() else None
    write_new(output_dir/"process.json", record)
    return record


def check_exit(process, *, expected, label):
    require(process.get("timed_out") is False and process.get("spawn_error") is None
            and process.get("returncode") == expected and process.get("raw") is not None,
            f"{label} native process exit/output differs (expected {expected}, actual {process.get('returncode')})")


def run_case(case, args, old_libraries, new_libraries, folder):
    folder.mkdir()
    result = {"sequence": case["sequence"], "fixture": case["fixture"],
              "expectation": case["expectation"], "passed": False, "error": None}
    for version in ("old", "new"):
        (folder/version).mkdir()
        result[version] = execute(getattr(args, version+"_executable"), args.predictor,
            old_libraries["predictor"], case["fixture"], folder/version, args.process_timeout_s)
    try:
        verify_receipt(case["fixture"])
        require(checker.strict_json(case["fixture"]["path"]) == case["value"], "Fixture changed during replay")
        expected_old = 0 if case["expectation"] == "unchanged" else 1
        expected_new = 1 if case["expectation"] == "rejected" else 0
        check_exit(result["old"], expected=expected_old, label="Old")
        check_exit(result["new"], expected=expected_new, label="New")
        raw = {version: checker.strict_json(result[version]["raw"]["path"]) for version in ("old", "new")}
        for version in raw:
            result[version]["loaded_code"] = raw[version].get("loaded_code", {})
        audit = checker.audit(case["value"], raw["old"], raw["new"], case["fixture"]["sha256"],
            old_libraries, new_libraries, case["expectation"])
        audit["artifacts"] = {"input": case["fixture"], "old": result["old"]["raw"], "new": result["new"]["raw"]}
        audit["auditor"] = file_identity(checker.__file__)
        write_new(folder/"audit.json", audit)
        result["audit"] = file_identity(folder/"audit.json")
        result["passed"] = audit.get("passed") is True
    except (ValueError, KeyError, TypeError, OSError, json.JSONDecodeError) as error:
        result["error"] = str(error)
        write_new(folder/"audit.json", {"kind": "x650_cpp_recovery_independent_replay_audit_v1",
            "complete": True, "passed": False, "error": str(error), "expectation": case["expectation"]})
        result["audit"] = file_identity(folder/"audit.json")
    write_new(folder/"case_result.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("old-executable", "new-executable", "predictor", "original-report", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--original-report-sha256", required=True)
    parser.add_argument("--new-core-sha256", required=True)
    parser.add_argument("--new-optimization-sha256")
    parser.add_argument("--extra-rejected-fixture", type=Path)
    parser.add_argument("--extra-rejected-fixture-sha256")
    parser.add_argument("--process-timeout-s", type=float, default=10.)
    args = parser.parse_args(argv)
    require(not args.output.exists(), "Regression evidence directory must not be overwritten")
    require(args.output.is_absolute(), "Absolute new output directory required")
    args.output.mkdir(parents=True)
    summary = {"kind": KIND, "complete": False, "passed": False, "rows": [],
               "all_failures_and_process_exits_retained": True, "injected_delay": False,
               "closed_loop_or_physical_validation_claimed": False, "started_unix": time.time()}
    code_before = {}
    try:
        require(0 < args.process_timeout_s <= 60. and math_isfinite(args.process_timeout_s), "Process cleanup timeout must be finite and <=60 s")
        for name in ("old_executable", "new_executable", "predictor", "original_report"):
            path = getattr(args, name).resolve()
            require(path.is_file(), "Missing explicit replay artifact: " + name)
            setattr(args, name, path)
            code_before[name] = file_identity(path)
        report, cases, config = load_original(args.original_report, args.original_report_sha256)
        old_libraries = {key: report["loaded_code"][key]["sha256"] for key in ("core", "optimization", "predictor")}
        require(code_before["predictor"]["sha256"] == old_libraries["predictor"], "Predictor differs from original reconstruction")
        new_libraries = dict(old_libraries, core=sha_argument(args.new_core_sha256),
                            optimization=sha_argument(args.new_optimization_sha256) if args.new_optimization_sha256 else old_libraries["optimization"])
        require(args.extra_rejected_fixture is None or args.extra_rejected_fixture_sha256 is not None,
                "Extra rejected fixture requires explicit SHA256")
        if args.extra_rejected_fixture is not None:
            path = args.extra_rejected_fixture.resolve()
            receipt = file_identity(path)
            require(receipt["sha256"] == sha_argument(args.extra_rejected_fixture_sha256), "Extra rejection fixture SHA differs")
            value = checker.strict_json(path)
            require(config_identity(value["config"]) == config, "Extra rejection fixture changed controller parameters")
            cases.append({"sequence": "extra_rejected", "fixture": receipt, "value": value, "expectation": "rejected"})
        summary.update(original_reconstruction=file_identity(args.original_report), code_before=code_before,
                       configuration=config, expected_old_libraries=old_libraries, expected_new_libraries=new_libraries,
                       expected_case_count=len(cases), runner=file_identity(__file__), auditor=file_identity(checker.__file__))
        for case in cases:
            label = f"{case['sequence']:06d}" if isinstance(case["sequence"], int) else case["sequence"]
            result = run_case(case, args, old_libraries, new_libraries, args.output/label)
            summary["rows"].append(result)
            print(json.dumps({"case": case["sequence"], "expectation": case["expectation"],
                              "passed": result["passed"], "error": result["error"]}), flush=True)
        code_after = {name: file_identity(getattr(args, name)) for name in code_before}
        summary["code_after"] = code_after
        summary["replay_code_and_original_report_unchanged"] = code_after == code_before
        # Re-read and validate all original receipts after the entire run, too.
        _, checked_again, config_after = load_original(args.original_report, args.original_report_sha256)
        summary["original_source_and_fixtures_unchanged"] = (config_after == config and
            [c["fixture"] for c in checked_again] == [c["fixture"] for c in cases[:379]])
        summary["complete"] = len(summary["rows"]) == len(cases)
        summary["passed"] = (summary["complete"] and all(row["passed"] for row in summary["rows"])
            and summary["replay_code_and_original_report_unchanged"] and summary["original_source_and_fixtures_unchanged"])
    except (ValueError, KeyError, TypeError, OSError, json.JSONDecodeError) as error:
        summary["error"] = str(error)
    summary["finished_unix"] = time.time()
    summary["passed_cases"] = sum(row["passed"] for row in summary["rows"])
    summary["failed_cases"] = len(summary["rows"])-summary["passed_cases"]
    summary["native_exit_counts"] = {version: {
        str(code): sum(row[version]["returncode"] == code for row in summary["rows"])
        for code in sorted({row[version]["returncode"] for row in summary["rows"]}, key=str)}
        for version in ("old", "new")}
    summary["loaded_library_sha256_observed"] = {version: {key: sorted({
        row[version].get("loaded_code", {}).get(key, {}).get("sha256", "unavailable")
        for row in summary["rows"]}) for key in ("core", "optimization", "predictor")}
        for version in ("old", "new")}
    write_new(args.output/"report.json", summary)
    return 0 if summary["passed"] else 1


def math_isfinite(value):
    # Avoid accepting NaN where comparisons alone do not reject it.
    return value == value and abs(value) != float("inf")


if __name__ == "__main__":
    raise SystemExit(main())
