"""Packaging regression: reconfigure one build SQP -> RTI, report case failures.

Uses existing generated solvers and one bound original feedback fixture. This
does not generate a model, modify an OCP, publish commands, or claim real time.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path = Path(path)
    if path.exists():
        raise ValueError("Refusing to replace evidence: " + str(path))
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def run(command, log, expected=0):
    process = subprocess.run([str(x) for x in command], capture_output=True, text=True, timeout=600)
    record = dict(command=[str(x) for x in command], returncode=process.returncode,
                  stdout=process.stdout, stderr=process.stderr)
    write(log, record)
    if process.returncode != expected:
        raise RuntimeError(f"Expected exit {expected}, got {process.returncode}: {log}")
    return record


def flat(value):
    if isinstance(value, list):
        return [x for row in value for x in flat(row)]
    return [float(value)]


def difference(a, b):
    a, b = flat(a), flat(b)
    if len(a) != len(b) or not all(math.isfinite(x) for x in a + b):
        raise ValueError("Mismatched or nonfinite numerical results")
    return max(abs(x - y) for x, y in zip(a, b))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--acados-dir", type=Path, required=True)
    parser.add_argument("--numerical-dir", type=Path, required=True)
    parser.add_argument("--original", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    build = out / "same_build_directory"
    source = args.source_dir.resolve()
    original = read(args.original)
    if len(original["cases"]) != 1:
        raise ValueError("One previously bound original fixture is required")
    write(out / "good_input.json", original)
    mixed = copy.deepcopy(original)
    invalid = copy.deepcopy(mixed["cases"][0])
    invalid_case = invalid.get("case", invalid)
    invalid_case["id"] += "_unprimed_rejection"
    invalid_case["memory"]["history_primed"] = False
    mixed["cases"].append(invalid)
    write(out / "mixed_input.json", mixed)
    source_hashes = [{"path": str(p), "sha256": sha(p)} for p in
                     [source / "CMakeLists.txt", source / "native/solver_replay.cpp",
                      source / "native/acados_backend.cpp", source / "native/case_builder.cpp",
                      Path(__file__).resolve()]]
    records = []
    for mode, compiled_mode in [("sqp", "SQP"), ("rti", "SQP_RTI")]:
        stage = out / mode
        stage.mkdir()
        generated = (args.numerical_dir / f"generated_{mode}/c_generated_code").resolve()
        run(["cmake", "-S", source, "-B", build, "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF",
             "-DACADOS_SOURCE_DIR=" + str(args.acados_dir.resolve()),
             "-DX650_ACADOS_GENERATED_DIR=" + str(generated)], stage / "configure.process.json")
        run(["cmake", "--build", build, "--target", "x650_standard_solver_replay",
             "x650_standard_native_replay", "-j2"], stage / "build.process.json")
        cache = (build / "CMakeCache.txt").read_text()
        expected_library = generated / "libacados_ocp_solver_x650_standard.so"
        cache_bound = "X650_GENERATED_SOLVER:FILEPATH=" + str(expected_library) in cache
        if not cache_bound:
            raise AssertionError("Reconfigured CMake cache retained the wrong solver library")
        shutil.copy2(build / "CMakeCache.txt", stage / "CMakeCache.txt")
        binary = build / "x650_standard_solver_replay"
        linkage = run(["ldd", binary], stage / "linkage.process.json")
        if str(expected_library) not in linkage["stdout"]:
            raise AssertionError("The runtime loader selected a solver from another mode")
        for name, expected_exit in [("good", 0), ("mixed", 2)]:
            input_path = out / (name + "_input.json")
            run([binary, input_path, sha(input_path), stage / (name + ".json"), "1"],
                stage / (name + ".process.json"), expected_exit)
        good, bad = read(stage / "good.json"), read(stage / "mixed.json")
        for report in [good, bad]:
            if report["compiled_solver_mode"] != compiled_mode or report["accepted_for_publication"] is not False:
                raise AssertionError("Mode or no-publication contract differs")
        good_flags = good["complete"] is True and good["failed_cases"] == 0 and good["completed_samples"] == 1
        bad_flags = (bad["complete"] is False and bad["failed_cases"] == 1 and bad["completed_samples"] == 1
                     and len(bad["records"]) == 2 and bad["records"][0]["completed"] is True
                     and bad["records"][1]["completed"] is False and not bad["records"][1]["samples"]
                     and "primed" in bad["records"][1]["error"])
        reference = read(args.numerical_dir / f"case_000/acados_{mode}.json")
        sample = good["records"][0]["samples"][0]
        errors = {"plan": difference(sample["plan"], reference["plan"]),
                  "rollout": difference(sample["rollout_states"], reference["states"])}
        status_matches = sample["raw_status"] == reference["raw_status"]
        passed = good_flags and bad_flags and status_matches and max(errors.values()) <= 1e-6
        records.append(dict(mode=mode,passed=passed,cache_bound=cache_bound,loader_bound=True,
                            binary_sha256=sha(binary),generated_library_sha256=sha(expected_library),
                            good_flags=good_flags,mixed_failure_flags=bad_flags,status_matches=status_matches,
                            absolute_errors=errors,good_input_sha256=sha(out / "good_input.json"),
                            mixed_input_sha256=sha(out / "mixed_input.json")))
        print(json.dumps(records[-1]), flush=True)
    stable = all(sha(r["path"]) == r["sha256"] for r in source_hashes)
    report = dict(kind="x650_standard_native_packaging_contract_v2",passed=all(r["passed"] for r in records) and stable,
                  same_build_directory=str(build),order=["SQP","SQP_RTI"],records=records,
                  original_fixture=dict(path=str(args.original.resolve()),sha256=sha(args.original)),
                  source_identities=source_hashes,source_unchanged=stable,accepted_for_publication=False,
                  limitations=["One fixed case checks wrapper and build-mode selection, not full qualification.",
                               "Earlier full 16-case reproduction is preserved under its original binary identities.",
                               "This regression does not modify the graph or declare physical/real-time acceptance."])
    write(out / "report.json", report)
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
