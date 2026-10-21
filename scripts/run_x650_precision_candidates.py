#!/usr/bin/env python3
"""Sequential, fail-closed X650 candidate trials; no runtime repair or retries.

The existing launch.py owns each independent Gazebo/PX4 lifecycle. This driver
only selects reviewed PID candidates, verifies recovered evidence, and optionally
runs the existing full-ULog and native-step analyzers after runtime shutdown.
"""
from __future__ import annotations
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"simulations/x650_wholebody_mpc_v1"))
from task import validate_task_config, preflight
from diagnostic_parameters import PID_CANDIDATES, PRECISION_PROFILE, load_parameter_evidence


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    temporary.replace(path)


def now():
    return datetime.now(timezone.utc).isoformat()


def require_idle(root):
    paths = [Path(root)/".amsim/runtime.json", Path(root)/".amsim/x650_diagnostic_restore_pending.json"]
    present = [str(path) for path in paths if path.exists()]
    if present:
        raise RuntimeError("Runtime or pending restoration requires inspection; batch will not repair it: "+str(present))
    return {"runtime_absent": True, "restore_journal_absent": True, "verified_utc": now()}


def control_records(path):
    snapshot = read_json(path)
    records = snapshot.get("parameters", {})
    required = load_parameter_evidence().CORE_CONTROL_PARAMETERS
    if len(required) != 42 or not set(required) <= set(records):
        raise RuntimeError("All 42 canonical control originals are required: "+str(path))
    output = {}
    for name in required:
        record = records[name]
        kind, raw_hex = record.get("param_type"), record.get("raw_value_hex")
        if kind not in (6, 9) or not isinstance(raw_hex, str):
            raise RuntimeError("Missing typed raw parameter record: "+name)
        raw = bytes.fromhex(raw_hex)
        if len(raw) != 4:
            raise RuntimeError("Invalid raw parameter length: "+name)
        decoded = struct.unpack("<i" if kind == 6 else "<f", raw)[0]
        if not math.isfinite(decoded) or decoded != record.get("value"):
            raise RuntimeError("Parameter raw bytes disagree with numeric readback: "+name)
        output[name] = {"param_type": kind, "raw_value_hex": raw.hex(), "value": decoded}
    return output


def physics_identity(metadata):
    provenance = metadata["provenance"]
    result = {name: metadata[name] for name in ("model_sha256", "source_urdf_sha256", "physical_profile")}
    result.update({name: provenance[name] for name in
        ("flight_profile_sha256", "airframe_sha256", "total_mass_kg", "native_recorder_sha256", "preparation_report_sha256")})
    if any(not isinstance(result[name], str) or len(result[name]) != 64 for name in
           ("model_sha256", "source_urdf_sha256", "flight_profile_sha256", "airframe_sha256", "native_recorder_sha256", "preparation_report_sha256")):
        raise RuntimeError("Incomplete physics identity hashes")
    return result


def require_completed_run(folder, task):
    folder = Path(folder).resolve(strict=True)
    metadata = read_json(folder/"controller_metadata.json")
    expected = canonical_hash(task)
    if canonical_hash(metadata.get("task")) != expected or canonical_hash(read_json(folder/"controller_task.json")) != expected:
        raise RuntimeError("Run does not match the exact normalized task hash: "+str(folder))
    if any(metadata.get(key) is not True for key in ("complete", "landed_and_disarmed", "diagnostic_parameters_restored")):
        raise RuntimeError("Run is incomplete or landing/restoration was not certified: "+str(folder))
    if metadata.get("diagnostic_restore_pending") is not False:
        raise RuntimeError("Run restoration journal was not confirmed cleared")
    before, after = (control_records(folder/("px4_parameters_"+label+".json")) for label in ("before", "after"))
    if before != after:
        raise RuntimeError("Canonical 42 control parameters differ after landing")
    return metadata, before


def locate_baseline(root, task, explicit=None):
    if explicit is not None:
        folder = Path(explicit).resolve(strict=True)
        require_completed_run(folder, task)
        return folder
    expected, matches = canonical_hash(task), []
    for folder in sorted((Path(root)/"runs").glob(task["scenario"]+"_*")):
        path = folder/"controller_metadata.json"
        if not path.is_file():
            continue
        metadata = read_json(path)
        if canonical_hash(metadata.get("task")) == expected and metadata.get("complete") is True:
            matches.append(folder)
    if len(matches) != 1:
        raise RuntimeError("Need one exact completed baseline task match; pass --baseline-run to disambiguate, matches="+str(matches))
    require_completed_run(matches[0], task)
    return matches[0].resolve()


def validate_candidate_evidence(folder, task, baseline_metadata, baseline_controls):
    metadata, before = require_completed_run(folder, task)
    if physics_identity(metadata) != physics_identity(baseline_metadata):
        raise RuntimeError("Candidate physics differs from the verified baseline")
    if before != baseline_controls:
        raise RuntimeError("Candidate did not boot with the same 42 baseline control parameters")
    expected = task["pid_tuning_candidate"]
    actual = metadata.get("actual_pid_parameters", {})
    for name, record in baseline_controls.items():
        value = expected.get(name, record["value"])
        received = actual.get(name)
        if (isinstance(received, bool) or not isinstance(received, (int, float)) or not math.isfinite(received)
                or not math.isclose(received, value, rel_tol=2e-6, abs_tol=1e-7)):
            raise RuntimeError("Measurement-period controller metadata has wrong PID parameter: "+name)
    if metadata.get("pid_tuning_candidate") != expected:
        raise RuntimeError("Candidate metadata does not name the selected four-parameter profile")
    return {"run_directory": str(Path(folder).resolve()), "task_sha256": canonical_hash(task),
        "metadata_sha256": file_hash(Path(folder)/"controller_metadata.json"),
        "physics_identity_sha256": canonical_hash(physics_identity(metadata)),
        "baseline_control_42_sha256": canonical_hash(before), "candidate_readback": {name: actual[name] for name in expected},
        "complete": True, "landed_and_disarmed": True, "parameters_restored": True}


def run_batch(*, root, output, base_task, candidates, baseline_run=None, analyze=False,
              launch_python=None, analysis_python="/root/autodl-tmp/amsim-venv/bin/python", run_process=None,
              preflight_check=None):
    root, output = Path(root).resolve(strict=True), Path(output).resolve()
    base_task = Path(base_task).resolve(strict=True)
    if output.exists():
        raise ValueError("Batch output must be a new directory; existing evidence is never overwritten")
    candidates = list(candidates)
    if not candidates or len(set(candidates)) != len(candidates) or any(name not in PID_CANDIDATES for name in candidates):
        raise ValueError("Candidates must be unique names from the reviewed nine-entry catalog")
    base = validate_task_config(read_json(base_task))
    if (base["stage"] != "hover_folded" or base["controller"] != "pid_ff" or base.get("hover_reference_profile") is None
            or base.get("precision_test_profile") != "hover20_steps36_v1" or base.get("pid_tuning_candidate") is not None):
        raise ValueError("Base task must be the verified fixed-world 20 s hover + 36 s step baseline without a PID override")
    if analyze and base.get("diagnostic_profile") != PRECISION_PROFILE:
        raise ValueError("Existing full precision analyzer requires the explicit raw-EV precision profile")
    if analyze and not Path(analysis_python).is_file():
        raise ValueError("Existing analysis Python is missing; install/download is outside this driver")
    require_idle(root)
    baseline_folder = locate_baseline(root, base, baseline_run)
    baseline_metadata, baseline_controls = require_completed_run(baseline_folder, base)
    physics_identity(baseline_metadata)
    output.mkdir(parents=True, exist_ok=False)
    run_process = run_process or subprocess.run
    preflight_check = preflight_check or preflight
    launch_python = str(launch_python or sys.executable)
    status = {"kind": "x650_sequential_precision_candidate_batch", "schema_version": 1, "complete": False,
        "started_utc": now(), "base_task": str(base_task), "base_task_sha256": canonical_hash(base),
        "base_task_file_sha256": file_hash(base_task), "baseline_run": str(baseline_folder),
        "baseline_metadata_sha256": file_hash(baseline_folder/"controller_metadata.json"),
        "candidate_order": candidates, "analyze_requested": bool(analyze), "trials": [],
        "optimal_pid_claimed": False, "failure_policy": "Stop immediately; no retry, runtime repair, or further flight"}
    status_path = output/"batch_status.json"
    atomic_json(status_path, status)
    env = os.environ.copy()
    env.update(PYTHONPATH="/usr/lib/python3/dist-packages", PYTHONUNBUFFERED="1",
        OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    try:
        for index, candidate in enumerate(candidates):
            trial_dir = output/f"{index+1:02d}_{candidate}"
            trial_dir.mkdir()
            task = deepcopy(base)
            task["pid_tuning_candidate"] = deepcopy(PID_CANDIDATES[candidate])
            task = validate_task_config(task)
            task_path = trial_dir/"task.json"
            atomic_json(task_path, task)
            trial = {"candidate": candidate, "task_path": str(task_path), "task_sha256": canonical_hash(task),
                     "phase": "preflight", "complete": False, "started_utc": now()}
            status["trials"].append(trial)
            trial["idle_before"] = require_idle(root)
            atomic_json(status_path, status)
            _, current_provenance = preflight_check(task)
            expected_physics = physics_identity(baseline_metadata)
            for name in expected_physics:
                if name != "physical_profile" and current_provenance.get(name) != expected_physics[name]:
                    raise RuntimeError("Preflight physics differs from baseline before launch: "+name)
            trial["preflight_physics_verified"] = True
            before = set((root/"runs").glob(task["scenario"]+"_*"))
            launches_before = set((root/"runs").glob("x650_wb_launch_*"))
            command = [launch_python, str(root/"simulations/x650_wholebody_mpc_v1/launch.py"), "--task", str(task_path)]
            trial.update(phase="flight", command=command)
            atomic_json(status_path, status)
            print("X650_CANDIDATE_START", candidate, str(trial_dir), flush=True)
            with (trial_dir/"launch.log").open("w", encoding="utf-8") as log:
                result = run_process(command, cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
            trial["launch_exit_code"] = result.returncode
            trial["idle_after"] = require_idle(root)
            if result.returncode != 0:
                raise RuntimeError("Candidate launcher failed: "+candidate)
            created = set((root/"runs").glob(task["scenario"]+"_*"))-before
            launches = set((root/"runs").glob("x650_wb_launch_*"))-launches_before
            if len(created) != 1 or len(launches) != 1:
                raise RuntimeError("Ambiguous newly created launch/run identity; refusing latest-directory fallback")
            folder, launch_folder = next(iter(created)), next(iter(launches))
            launch = read_json(launch_folder/"launch.json")
            if (launch.get("complete") is not True or launch.get("runtime_closed") is not True
                    or canonical_hash(launch.get("task")) != canonical_hash(task)
                    or [str(Path(v).resolve()) for v in launch.get("run_directories", [])] != [str(folder.resolve())]):
                raise RuntimeError("Launch metadata does not uniquely bind the recovered run to this exact task")
            trial.update(phase="evidence", launch_directory=str(launch_folder),
                         evidence=validate_candidate_evidence(folder, task, baseline_metadata, baseline_controls))
            atomic_json(status_path, status)
            if analyze:
                analysis_env = env.copy()
                analysis_env.pop("PYTHONPATH", None)
                trial["analysis"] = []
                for script, name in (("analyze_x650_hover_precision.py", "hover_precision"),
                                     ("analyze_x650_position_steps.py", "position_steps")):
                    require_idle(root)
                    command = [str(analysis_python), str(root/"scripts"/script), "--run-dir", str(folder), "--output", str(trial_dir/name)]
                    item = {"name": name, "command": command, "complete": False}
                    trial["analysis"].append(item)
                    trial["phase"] = "analysis"
                    atomic_json(status_path, status)
                    with (trial_dir/(name+".log")).open("w", encoding="utf-8") as log:
                        result = run_process(command, cwd=root, env=analysis_env, stdout=log, stderr=subprocess.STDOUT)
                    item["exit_code"] = result.returncode
                    require_idle(root)
                    if result.returncode != 0:
                        raise RuntimeError("Analysis failed; no further candidate will fly: "+name)
                    report_path = trial_dir/name/("precision_audit.json" if name == "hover_precision" else "position_steps_audit.json")
                    report = read_json(report_path)
                    if report.get("source_run") != folder.name:
                        raise RuntimeError("Analysis report belongs to a different run")
                    if name == "hover_precision" and (report.get("native_only") is not False
                            or report.get("measurement_period_pid", {}).get("matches_controller_metadata") is not True
                            or report.get("ulog_control_chain", {}).get("fixed_world_target_verified_within_2mm") is not True):
                        raise RuntimeError("Full ULog PID/setpoint audit did not certify the declared control chain")
                    item.update(complete=True, report_path=str(report_path), report_sha256=file_hash(report_path))
            trial.update(complete=True, phase="complete", finished_utc=now())
            atomic_json(status_path, status)
            print("X650_CANDIDATE_COMPLETE", candidate, str(folder), flush=True)
        status["idle_final"] = require_idle(root)
        status.update(complete=True, finished_utc=now())
    except BaseException as exc:
        status.update(error=repr(exc), finished_utc=now())
        if status["trials"] and not status["trials"][-1]["complete"]:
            status["trials"][-1].update(error=repr(exc), phase="failed", finished_utc=now())
        raise
    finally:
        atomic_json(status_path, status)
    return status


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--base-task", type=Path, required=True)
    parser.add_argument("--baseline-run", type=Path, help="Exact verified baseline run; otherwise require a unique task-hash match")
    parser.add_argument("--candidates", nargs="+", choices=tuple(PID_CANDIDATES), required=True)
    parser.add_argument("--analyze", action="store_true", help="Run full-ULog hover and native-step audits after each recovered flight")
    parser.add_argument("--analysis-python", type=Path, default=Path("/root/autodl-tmp/amsim-venv/bin/python"))
    args = parser.parse_args(argv)
    run_batch(root=ROOT, output=args.output, base_task=args.base_task, candidates=args.candidates,
              baseline_run=args.baseline_run, analyze=args.analyze, analysis_python=args.analysis_python)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
