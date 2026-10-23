"""Fresh TS20 software-plumbing proof; no flight or inherited test results.

The previous proof is a hash-pinned source of unchanged oracle/native/physical
identities only. Package, interface/auditor/shared regressions, full/view QP
equivalence and the original 30 fixtures are executed again in a fresh folder.
This revision includes causal joint selection, bounded recorder statistics and
the diagnostic-specific serializer; previous causal-v2 results are not reused.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
PKG = "control_packages/x650_wholebody_v1"
MATRIX_SCRIPT = "artifacts/ts20_prediction_chain_20261022/benchmark_gain_kd025_matrix_v1.py"
ORIGINAL_MATRIX_RESULT = "artifacts/ts20_prediction_chain_20261022/oracle_validation_gain_kd025_v1/full_mpc_matrix_v1.json"
ORIGINAL_RESULT_SHA256 = "64eea8833bd95665c1a00222c827f1dc4a203f96f891d888e74826cb958484e8"
EXTRA_SOURCES = (
    "simulations/x650_wholebody_mpc_v1/estimated_feedback.py",
    "simulations/x650_wholebody_mpc_v1/recording.py",
    "simulations/x650_wholebody_mpc_v1/run_case.py",
    "simulations/x650_ts20_hold_v1/run_case.py",
    "simulations/x650_ts20_hold_v1/audit.py",
    "simulations/x650_ts20_hold_v1/task.py",
    "simulations/x650_ts20_hold_v1/loop_timing.py",
    "simulations/x650_ts20_hold_v1/diagnostic_serialization.py",
    "simulations/x650_ts20_hold_v1/launch_chain_v4.py",
    "scripts/audit_x650_mixed_feedback.py",
    "scripts/validate_x650_ts20_causal_software.py",
    "scripts/profile_x650_ts20_diagnostic_serialization.py",
)
UNIT_PATTERNS = (
    "test_x650_ts20_*.py", "test_ts20_causal_feedback_audit.py",
    "test_ts20_loop_timing.py", "test_ts20_chain_software_provenance.py",
    "test_x650_estimated_feedback.py", "test_x650_prediction_timing_feedback.py",
    "test_x650_mixed_estimated_feedback.py", "test_x650_upper_stage_audit.py",
    "test_x650_activation_handoff.py", "test_x650_execution_trace.py",
    "test_ts20_diagnostic_serialization.py",
)
SHARED_PATTERNS = ("test_upper_loop.py", "test_lifecycle.py", "test_status_receipt_guard.py", "test_recording.py")
REVIEWED_CAUSAL_SOURCES = {
    PKG+"/x650_wholebody/ts20_runtime.py": "db2314ebad895527b57dea9b824cdc2190d1c1180de64fd488e52a0957906080",
    "simulations/x650_wholebody_mpc_v1/estimated_feedback.py": "312ccc060320ba1af4008f2582a6de4212934d7630c2b653d4e55ca8a0dc7090",
    "simulations/x650_wholebody_mpc_v1/run_case.py": "8ed20f88a9a626a038294df44b56becbd7486173fdfbb2433460e5fe2309e648",
    "simulations/x650_wholebody_mpc_v1/recording.py": "2378a55ae2db2f586843c99dcb3341a01875f0a9fae06b56e91d13a151803e38",
    "simulations/x650_wholebody_mpc_v1/tests/test_recording.py": "d784fd6ce2adc735c9ba86c4c5dba18d60129197cfd06e18c681e603f35fa1f2",
    "simulations/x650_ts20_hold_v1/run_case.py": "77f4c9dfa0c2c3f8f309cefb413c5d44815b6695c4ee02d6542d63c440495bd8",
    "simulations/x650_ts20_hold_v1/diagnostic_serialization.py": "c83575362bc097086c3bbe4ab50c292972dddb80fc16b7d5a130913e025ea1c3",
    "tests/unit/test_ts20_diagnostic_serialization.py": "56adbbf7867478745ce9cd0a6a0d7e2207987b5a7a37bfd0e8e77594070f6872",
    "tests/unit/test_x650_ts20_causal_joints.py": "21a91435d644471b650e2a0a0babce05ba39b126153eb68a0acbd7647afe9500",
    "tests/unit/test_x650_ts20_extension.py": "efe835b321f5f710496f12f0fa04cb99840510b1c7693c6b3a7c619483197e55",
    "scripts/audit_x650_mixed_feedback.py": "a0418b10be072dfb48087eba780e8b0de37a082d4352e3e2ba74198a1f7c65e6",
    "simulations/x650_ts20_hold_v1/audit.py": "79c8af6339e04e241b0a6b23c9396c55d3e0ebfc930b5ebb0da4cfdbba0d34b9",
    "tests/unit/test_ts20_causal_feedback_audit.py": "22b3876e75a6b28d1cde5fa9665b5a9e9a86fce1321e85f1fb1f118afac16734",
    "simulations/x650_ts20_hold_v1/loop_timing.py": "2d1f9242e3b3286a9b19448751ea440fb9da40294f3e487b24e83c8fbda0e535",
    "tests/unit/test_ts20_loop_timing.py": "bd89dc191a1dfea2bfef929be437a309da7797728517f52cd6bf4b3ffa024744",
    "scripts/profile_x650_ts20_diagnostic_serialization.py": "8b8320a09248cb7abbff00d614983b268b3f02cdca220125f69e618f2469ae48",
}


def identity(path):
    path = Path(path).resolve(strict=True)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "bytes": path.stat().st_size}


def write_new(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def inside(relative):
    path = (ROOT/relative).resolve(strict=True)
    if path.relative_to(ROOT).as_posix() != relative:
        raise ValueError("Noncanonical source outside the repository: "+relative)
    return path


def test_files():
    groups = {"package": sorted((ROOT/PKG/"tests").glob("test_*.py")), "interfaces_and_audits": [], "shared_loop": []}
    for group, directory, patterns in (("interfaces_and_audits", ROOT/"tests/unit", UNIT_PATTERNS),
                                        ("shared_loop", ROOT/"simulations/x650_wholebody_mpc_v1/tests", SHARED_PATTERNS)):
        paths = []
        for pattern in patterns:
            matches = sorted(directory.glob(pattern))
            if not matches:
                raise ValueError("Required regression source missing: "+pattern)
            paths.extend(matches)
        groups[group] = sorted(set(paths))
    return groups


def loaded_project_sources():
    result = {}
    for module in list(sys.modules.values()):
        name = getattr(module, "__file__", None)
        if name is None:
            continue
        path = Path(name).resolve()
        try:
            relative = path.relative_to(ROOT).as_posix()
        except ValueError:
            continue
        if path.suffix == ".py" and path.is_file():
            result[relative] = identity(path)["sha256"]
    return result


def run_tests(groups, out):
    # Supply the explicitly pinned native path before discovery evaluates its
    # optional decorator. No skip is silently counted as an executed check.
    loader, all_tests = unittest.TestLoader(), unittest.TestSuite()
    discovered = {}
    for name, paths in groups.items():
        suite = unittest.TestSuite()
        for path in paths:
            suite.addTests(loader.discover(str(path.parent), pattern=path.name,
                                          top_level_dir=str(path.parent)))
        discovered[name] = suite.countTestCases()
        all_tests.addTests(suite)
    loaded_before = loaded_project_sources()
    start = time.monotonic()
    with (out/"unittest.log").open("x", encoding="utf-8") as stream:
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(all_tests)
    loaded_after = loaded_project_sources()
    report = {"tests": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
        "skipped": len(result.skipped), "skipped_detail": [(str(test), reason) for test, reason in result.skipped],
        "successful": result.wasSuccessful(), "elapsed_s": time.monotonic()-start,
        "groups_discovered": discovered, "test_files": {k: [identity(p) for p in paths] for k, paths in groups.items()},
        "native_test_env": os.environ.get("TS20_TEST_NATIVE_LIB"),
        "python": sys.executable,
        "dependencies": {name: {"version": getattr(sys.modules.get(name), "__version__", None),
                                 "path": getattr(sys.modules.get(name), "__file__", None)}
                         for name in ("numpy", "scipy")},
        "process_environment": {name: os.environ.get(name) for name in (
            "PYTHONPATH", "OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")},
        "loaded_project_sources_after_discovery": loaded_before,
        "loaded_project_sources_after_execution": loaded_after,
        "loaded_source_observation_scope": "File bytes observed after actual imports; additional runtime imports first observed after execution, not claimed to have been snapshotted before discovery",
        "log": identity(out/"unittest.log")}
    write_new(out/"package_report.json", report)
    if not result.wasSuccessful() or discovered["package"] < 124:
        raise RuntimeError("Fresh regression failed or omitted the 124-test control package")
    if any(identity(inside(p))["sha256"] != value for p, value in loaded_before.items()):
        raise RuntimeError("Imported project source changed during test execution")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--previous-proof", type=Path, required=True)
    parser.add_argument("--previous-proof-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    out = args.output.resolve(strict=True)
    try:
        previous_id = identity(args.previous_proof)
        if previous_id["sha256"] != args.previous_proof_sha256:
            raise ValueError("Previous proof differs from caller-pinned bytes")
        previous = json.loads(args.previous_proof.read_text())
        if previous.get("no_flight") is not True:
            raise ValueError("Expected the explicit no-flight prior software evidence")
        shutil.copyfile(args.previous_proof, out/"previous_software_proof.json")
        native, oracle, condition = previous["native_library"], previous["oracle"], previous["operating_condition"]
        for entry in (native, oracle, condition):
            if identity(entry["path"])["sha256"] != entry["sha256"]:
                raise ValueError("Pinned unchanged dependency differs: "+entry["path"])
        candidate = inside(PKG+"/x650_wholebody/ts20_prediction.py")
        if (identity(candidate)["sha256"] != oracle["candidate_sha256"]
                or identity(native["cpp_path"])["sha256"] != native["cpp_sha256"]):
            raise ValueError("Prediction/kernel identity changed; the retained oracle cannot qualify it")
        if any(os.environ.get(name) != "1" for name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS")) or os.environ.get("PYTHONPATH"):
            raise ValueError("Run with process-only BLAS/OMP=1 and PYTHONPATH unset, as in the original matrix")
        os.environ["TS20_TEST_NATIVE_LIB"] = native["path"]
        os.chdir(ROOT)
        groups = test_files()
        paths = set(previous["deployment_sources"]) | set(EXTRA_SOURCES)
        paths.update(p.relative_to(ROOT).as_posix() for group in groups.values() for p in group)
        before = {p: identity(inside(p))["sha256"] for p in sorted(paths)}
        for relative, expected in REVIEWED_CAUSAL_SOURCES.items():
            if before.get(relative) != expected:
                raise ValueError("Reviewed causal source changed before validation: "+relative)
        # The only old-proof production mutation is the reviewed runtime. Other
        # approved shared/auditor files are explicit additions to that scope.
        for relative, old_sha in previous["deployment_sources"].items():
            if relative != PKG+"/x650_wholebody/ts20_runtime.py" and before[relative] != old_sha:
                raise ValueError("Unapproved prior-scope source differs: "+relative)
        archive = out/"executed_sources.zip"
        with zipfile.ZipFile(archive, "x", zipfile.ZIP_DEFLATED) as stream:
            for relative in before:
                content = inside(relative).read_bytes()
                if hashlib.sha256(content).hexdigest() != before[relative]:
                    raise ValueError("Source changed during archiving: "+relative)
                stream.writestr(relative, content)
        write_new(out/"sources_before.json", before)
        suite = run_tests(groups, out)
        additions = {}
        imported = {**suite["loaded_project_sources_after_discovery"],
                    **suite["loaded_project_sources_after_execution"]}
        for relative, loaded_sha in imported.items():
            if relative in before and before[relative] != loaded_sha:
                raise RuntimeError("Loaded source differs from initial deployment bytes: "+relative)
            if relative not in before:
                additions[relative] = loaded_sha
        if additions:
            with zipfile.ZipFile(archive, "a", zipfile.ZIP_DEFLATED) as stream:
                for relative, expected in sorted(additions.items()):
                    content = inside(relative).read_bytes()
                    if hashlib.sha256(content).hexdigest() != expected:
                        raise RuntimeError("Imported source changed before archiving: "+relative)
                    stream.writestr(relative, content)
            before.update(additions)
        write_new(out/"runtime_import_source_additions.json", additions)
        old_matrix = previous["software_matrix"]
        history = previous["historical_failures"]["matrix_29_of_30"]
        if identity(inside(MATRIX_SCRIPT))["sha256"] != old_matrix["original_script_sha256"]:
            raise ValueError("Original frozen matrix script differs")
        original_result = identity(inside(ORIGINAL_MATRIX_RESULT))
        if original_result["sha256"] != ORIGINAL_RESULT_SHA256:
            raise ValueError("Preserved original result differs from the read-only archive check")
        command = [sys.executable, str(ROOT/"scripts/validate_x650_ts20_qp_view.py"),
            "--matrix-script", str(inside(MATRIX_SCRIPT)), "--matrix-sha256", old_matrix["original_script_sha256"],
            "--historical-failure", history["path"], "--historical-sha256", history["sha256"],
            "--native-sha256", native["sha256"], "--original-result", str(inside(ORIGINAL_MATRIX_RESULT)),
            "--output", str(out/"matrix")]
        write_new(out/"matrix_command.json", command)
        with (out/"matrix_process.log").open("x", encoding="utf-8") as stream:
            subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, check=True)
        matrix_path = out/"matrix/validation_summary.json"
        matrix = json.loads(matrix_path.read_text())
        if (not matrix["equivalence_passed"] or matrix["matrix_total"] != 30 or matrix["matrix_passed"] != 30
                or matrix["matrix_failures"] or not 0 <= matrix["matrix_max_wall_ms"] <= 50.
                or matrix["original_failure_preserved"] is not True
                or any(not row["passed"] or not 0 <= row.get("wall_ms", float("inf")) <= 50.
                       for row in matrix["independent_timing"] if row["view"] == "view")):
            raise RuntimeError("Fresh original 30-case or full/view equivalence acceptance failed")
        if identity(inside(ORIGINAL_MATRIX_RESULT)) != original_result:
            raise RuntimeError("Preserved original matrix result changed")
        after = {p: identity(inside(p))["sha256"] for p in before}
        write_new(out/"sources_after.json", after)
        if after != before:
            raise RuntimeError("Deployment source changed during the software run")
        for entry in (native, oracle, condition, history):
            if identity(entry["path"])["sha256"] != entry["sha256"]:
                raise RuntimeError("Frozen evidence/library changed during validation")
        # These fields intentionally match launch_chain_v4.verify_software_proof.
        proof = {"schema": "ts20_causal_joint_software_acceptance_v1", "no_flight": True,
            "software_cpu_stage_ended": True, "deployment_sources": before,
            "native_library": native, "oracle": oracle, "operating_condition": condition,
            "retained_matching_oracle_not_rerun": True,
            "prior_proof_lineage_only": previous_id, "previous_test_results_inherited": False,
            "initial_source_observation": identity(out/"sources_before.json"),
            "additional_import_sources": identity(out/"runtime_import_source_additions.json"),
            "package_regression": {"tests": suite["tests"], "failures": suite["failures"], "errors": suite["errors"],
                "skipped": suite["skipped"], "skipped_detail": suite["skipped_detail"],
                "report_path": str(out/"package_report.json"), "report_sha256": identity(out/"package_report.json")["sha256"],
                "groups_discovered": suite["groups_discovered"]},
            "software_matrix": {"total": 30, "passed": matrix["matrix_passed"],
                "max_wall_ms": matrix["matrix_max_wall_ms"], "failures": matrix["matrix_failures"],
                "summary_path": str(matrix_path), "summary_sha256": identity(matrix_path)["sha256"],
                "original_script_sha256": matrix["original_script_sha256"], "original_failure_preserved": matrix["original_failure_preserved"]},
            "fresh_equivalence": {k: matrix[k] for k in ("equivalence_passed", "differences", "constraint_view", "independent_timing")},
            "historical_failures": previous["historical_failures"],
            "source_archive": identity(archive), "driver": identity(__file__),
            "scope": "Software plumbing revision only: causal joint selection, bounded recorder statistics and TS20-specific diagnostic encoding. Unchanged physics, predictor/kernel, gains, weights, horizon, GC policy and all safety/timing bounds. Fresh package/interface/shared regressions and original 30-case/full-view checks; no causal-v2 results or old physical certificates promoted."}
        write_new(out/"software_acceptance_compact.json", proof)
        print(json.dumps({"proof": identity(out/"software_acceptance_compact.json"),
                          "tests": suite["tests"], "skipped": suite["skipped"],
                          "matrix_passed": matrix["matrix_passed"], "matrix_max_wall_ms": matrix["matrix_max_wall_ms"]}, indent=2))
    except BaseException as error:
        try:
            write_new(out/"failure.json", {"no_flight": True, "error": repr(error), "traceback": traceback.format_exc(),
                "successful_proof_written": (out/"software_acceptance_compact.json").exists(), "driver": identity(__file__)})
        except Exception as recording_error:
            print("Failure-record write also failed: "+repr(recording_error), file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
