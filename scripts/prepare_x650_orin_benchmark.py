#!/usr/bin/env python3
"""Prepare an immutable workstation-side corpus and a code-only NX source tar.

The fast fixtures are synthetic independent numerical cases, NOT recorded flight
states. Their full raw files stay on the workstation. Only the compact, lossless
JSON-value projection is later streamed to the NX process's standard input.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import tarfile
import tempfile
import zipfile


ROOT = Path(__file__).resolve().parents[1]
FAST = ROOT / "docs/validation/cpp_fast_20261024"
RAW_PREFIX = "software_and_audit/dynamic_raw_v1/"
CHECK_PREFIX = "software_and_audit/dynamic_check_v1/"
SOURCE_PATHS = (
    "control_packages/x650_wholebody_v1/native/CMakeLists.txt",
    "control_packages/x650_wholebody_v1/native/mpc_core.cpp",
    "control_packages/x650_wholebody_v1/native/mpc_core.hpp",
    "control_packages/x650_wholebody_v1/native/optimization.cpp",
    "control_packages/x650_wholebody_v1/native/test_mpc_core.cpp",
    "control_packages/x650_wholebody_v1/native/test_quadprog_identity.cpp",
    "control_packages/x650_wholebody_v1/native/vendor/quadprogpp/Array.hh",
    "control_packages/x650_wholebody_v1/native/vendor/quadprogpp/QuadProg++.cc",
    "control_packages/x650_wholebody_v1/native/vendor/quadprogpp/QuadProg++.hh",
    "control_packages/x650_wholebody_v1/native/vendor/quadprogpp/LICENSE",
    "control_packages/x650_wholebody_v1/native/vendor/quadprogpp/UPSTREAM.json",
    "control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction_native.cpp",
    "control_packages/x650_wholebody_v1/benchmark/CMakeLists.txt",
    "control_packages/x650_wholebody_v1/benchmark/src/memory_benchmark.cpp",
    "control_packages/x650_wholebody_v1/benchmark/include/benchmark_io.hpp",
)


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def identity(path: Path) -> dict:
    data = path.read_bytes()
    return {"path": str(path.resolve()), "bytes": len(data), "sha256": sha256(data)}


def strict_json(data: bytes):
    def reject(value):
        raise ValueError("Nonfinite JSON value: " + value)
    return json.loads(data, parse_constant=reject)


def json_bytes(value) -> bytes:
    return (json.dumps(value, ensure_ascii=True, separators=(",", ":"),
                       allow_nan=False) + "\n").encode("ascii")


def atomic_write(path: Path, data: bytes):
    """One local file becomes visible only after its content is complete."""
    if path.exists():
        raise FileExistsError(path)
    fd, temporary = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def verify_archive(archive: Path, receipt: Path, report: Path):
    """Require the frozen archive receipt and the exact accepted certificate."""
    raw = archive.read_bytes()
    rec = strict_json(receipt.read_bytes())
    cert_raw = report.read_bytes()
    cert = strict_json(cert_raw)
    if (len(raw), sha256(raw)) != (rec["bytes"], rec["sha256"]):
        raise ValueError("Raw archive differs from its frozen receipt")
    if sha256(cert_raw) != rec["physical_certificate_sha256"]:
        raise ValueError("Accepted certificate differs from the archive receipt")
    if not all(cert.get(key) is True for key in
               ("passed", "complete", "certificate", "physical_acceptance")):
        raise ValueError("The source fast experiment is not accepted")
    with zipfile.ZipFile(io.BytesIO(raw)) as source:
        names = source.namelist()
        if len(names) != len(set(names)):
            raise ValueError("Duplicate raw archive entries")
        if source.read("software_and_audit/fast_audit_v1/report.json") != cert_raw:
            raise ValueError("Embedded and supplied accepted certificates differ")
    return raw, rec


def prepare_input(archive: Path, receipt: Path, report: Path, *,
                  warmup_rounds: int, measured_rounds: int, cpu: int) -> dict:
    if not 0 <= warmup_rounds <= 1000 or not 1 <= measured_rounds <= 10000 or cpu < -1:
        raise ValueError("Invalid benchmark run limits")
    archive_raw, rec = verify_archive(archive, receipt, report)
    with zipfile.ZipFile(io.BytesIO(archive_raw)) as source:
        manifest_raw = source.read(RAW_PREFIX + "manifest.json")
        manifest = strict_json(manifest_raw)
        check_raw = source.read(CHECK_PREFIX + "report.json")
        check = strict_json(check_raw)
        if not (manifest["complete"] and manifest["source_files_unchanged"]
                and manifest["synthetic_inputs"] and manifest["no_flight"]
                and manifest["physical_acceptance"] is False):
            raise ValueError("Raw fixture provenance does not match the synthetic corpus")
        if not (check["complete"] and check["pass"] and check["libraries_unchanged"]):
            raise ValueError("The independent numerical qualification did not pass")
        if check["cases"]["sha256"] != sha256(manifest_raw):
            raise ValueError("Numerical qualification targets a different raw manifest")
        if not (len(manifest["rows"]) == check["total"] == check["passed"] == 48):
            raise ValueError("Expected exactly 48 accepted frozen fast fixtures")
        if len(manifest["rows"]) * measured_rounds > 100000:
            raise ValueError("At most 100000 measured samples are supported")
        cases, provenance_cases, configuration = [], [], None
        for i, row in enumerate(manifest["rows"]):
            name = PurePosixPath(row["path"]).name
            if name != f"case_{i:03d}.json":
                raise ValueError("Raw case ordering or name changed")
            case_raw = source.read(RAW_PREFIX + name)
            if (len(case_raw), sha256(case_raw)) != (row["bytes"], row["sha256"]):
                raise ValueError("Raw case identity changed: " + name)
            case = strict_json(case_raw)
            old_result_raw = source.read(CHECK_PREFIX + name)
            old_result = strict_json(old_result_raw)
            if old_result.get("pass") is not True or old_result["input_sha256"] != row["sha256"]:
                raise ValueError("Historical native result is not bound to the raw input")
            old_command = old_result["command"]
            raw_command = case["oracle"]["command"]
            if (len(old_command) != 5 or len(raw_command) != 5
                    or max(abs(a-b) for a, b in zip(old_command, raw_command)) > 1e-7):
                raise ValueError("Historical native command differs from the independent Python oracle")
            if configuration is None:
                configuration = case["config"]
            elif configuration != case["config"]:
                raise ValueError("Mixed configurations are not a single benchmark cohort")
            # Preserve every config/memory/input JSON value. Do not alter timing,
            # command histories, plans or the original 50 ms production limit.
            oracle = {"command": raw_command,
                      "position_reference": old_result["position_reference"],
                      "velocity_feedforward": old_result["velocity_feedforward"]}
            cases.append({"id": name[:-5], "memory": case["memory"],
                          "input": case["input"], "oracle": oracle})
            provenance_cases.append({"id": name[:-5], "raw_source": row,
                "raw_archive_entry": RAW_PREFIX + name,
                "historical_native_archive_entry": CHECK_PREFIX + name,
                "historical_native_sha256": sha256(old_result_raw),
                "historical_native_bytes": len(old_result_raw), "label": case["label"]})
    return {"schema_version": 1, "kind": "x650_mpc_memory_benchmark_input_v1",
        "config": configuration, "cases": cases,
        "run": {"warmup_rounds": warmup_rounds, "measured_rounds": measured_rounds,
                "cpu": cpu, "oracle_absolute_tolerance": 1e-7},
        "provenance": {"kind": "accepted_fast_synthetic_snapshot_corpus_v1",
            "synthetic_inputs": True, "recorded_physical_state_corpus": False,
            "continuous_closed_loop_replay": False,
            "state_restored_before_every_call": True,
            "source_archive": identity(archive), "archive_receipt": identity(receipt),
            "accepted_physical_certificate": identity(report),
            "raw_manifest_sha256": sha256(manifest_raw),
            "numerical_qualification_sha256": sha256(check_raw),
            "expected_predictor_source_binary": check["predictor"],
            "expected_core_source_binary": check["core_library"],
            "expected_optimization_source_binary": check["optimization_library"],
            "original_production_deadline_ms": 50,
            "oracle_sources": {"command": "original independent Python raw oracle",
                "position_reference": "previously accepted native result, not this benchmark",
                "velocity_feedforward": "previously accepted native result, not this benchmark"},
            "scope": "48 independent synthetic fast-trajectory snapshots. No flight or NX timing claim. "
                     "Full original oracles remain in the source archive on the workstation.",
            "cases": provenance_cases}}


def frozen_source_bytes(root: Path, frozen_archive: Path) -> tuple[dict[str, bytes], list[dict]]:
    values, records = {}, []
    with zipfile.ZipFile(frozen_archive) as outer:
        with zipfile.ZipFile(io.BytesIO(outer.read("accepted/launch/sources.zip"))) as original:
            for relative in SOURCE_PATHS:
                if "/benchmark/" in relative or relative.endswith("/LICENSE"):
                    continue
                current = (root / relative).read_bytes()
                accepted = original.read(relative)
                values[relative] = accepted
                records.append({"path": relative, "bytes": len(accepted), "sha256": sha256(accepted),
                                "workspace_sha256": sha256(current),
                                "workspace_exact_match": accepted == current,
                                "packaged_source": "accepted/launch/sources.zip"})
    return values, records


def package_sources(root: Path, output: Path, frozen_archive: Path | None = None) -> dict:
    """Explicit allowlist excludes input files, logs, meshes and result archives."""
    frozen_bytes, frozen = (frozen_source_bytes(root, frozen_archive)
                            if frozen_archive is not None else ({}, None))
    records = []
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz", format=tarfile.PAX_FORMAT) as tar:
        for relative in SOURCE_PATHS:
            path = root / relative
            if path.is_symlink() or not path.is_file():
                raise ValueError("Required ordinary source file missing: " + relative)
            # Production code is taken byte-for-byte from the accepted archive.
            # New benchmark code and its license remain explicit workspace inputs.
            data = frozen_bytes.get(relative, path.read_bytes())
            info = tarfile.TarInfo(relative)
            info.size = len(data)
            info.mode = 0o644
            info.mtime = 0
            tar.addfile(info, io.BytesIO(data))
            records.append({"path": relative, "bytes": len(data), "sha256": sha256(data)})
    atomic_write(output, buffer.getvalue())
    return {"kind": "x650_orin_code_only_source_package_v1", "archive": identity(output),
            "sources": records, "includes_input_corpus": False,
            "includes_results_logs_meshes": False,
            "frozen_production_sources_verified": frozen is not None,
            "frozen_production_sources": frozen}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-archive", type=Path, default=FAST / "fast_full_raw_v1.zip")
    parser.add_argument("--archive-receipt", type=Path, default=FAST / "fast_full_raw_v1_receipt.json")
    parser.add_argument("--accepted-report", type=Path, default=FAST / "report.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--warmup-rounds", type=int, default=2)
    parser.add_argument("--measured-rounds", type=int, default=20)
    parser.add_argument("--cpu", type=int, default=-1)
    parser.add_argument("--input-only", action="store_true",
                        help="Do not package source files (e.g. during code development)")
    args = parser.parse_args(argv)
    bundle = prepare_input(args.source_archive, args.archive_receipt, args.accepted_report,
                           warmup_rounds=args.warmup_rounds,
                           measured_rounds=args.measured_rounds, cpu=args.cpu)
    args.output.mkdir(parents=True, exist_ok=False)
    atomic_write(args.output / "input.json", json_bytes(bundle))
    result = {"kind": "x650_orin_workstation_preparation_v1", "complete": True,
              "input": identity(args.output / "input.json"), "case_count": len(bundle["cases"]),
              "measured_samples": len(bundle["cases"]) * args.measured_rounds,
              "provenance": bundle["provenance"],
              "nx_disk_inputs_or_outputs_required": False}
    if not args.input_only:
        result["source_package"] = package_sources(ROOT, args.output / "code_only_sources.tar.gz",
                                                   args.source_archive)
    atomic_write(args.output / "preparation.json", json_bytes(result))
    print(json.dumps({"complete": True, "case_count": len(bundle["cases"]),
                      "input_sha256": result["input"]["sha256"], "output": str(args.output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
