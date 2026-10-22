#!/usr/bin/env python3
"""Run one frozen mixed-response PID/MPC stage, independently audit, then stop.

Requires accepted response evidence and preceding physical stage certificates.
No automatic retry, retuning, fallback or advance to the next flight.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "simulations/x650_wholebody_mpc_v1"))
from task import validate_task_config, preflight
from run_x650_precision_candidates import (
    atomic_json, canonical_hash, file_hash, read_json, require_idle,
    require_completed_run,
)


def now():
    return datetime.now(timezone.utc).isoformat()


def run_one(*, task_path, output, launch_python,
            analysis_python, pyulog_site=None):
    task_path = Path(task_path).resolve(strict=True)
    output = Path(output).resolve()
    if output.exists():
        raise ValueError("Use a new job directory; previous attempts are preserved")
    cfg = validate_task_config(read_json(task_path))
    from research_condition import is_mixed_mpc_research
    from mixed_mpc_protocol import validate_mixed_mpc_protocol
    from accepted_response_model import load_accepted_response_model
    if not is_mixed_mpc_research(cfg):
        raise ValueError("This driver requires the frozen mixed v3 profile")
    protocol = validate_mixed_mpc_protocol(cfg.get("mixed_mpc_protocol"))
    binding = load_accepted_response_model(cfg)["binding"]
    for interpreter in (launch_python, analysis_python):
        if not Path(interpreter).is_file():
            raise ValueError("Existing Python interpreter required: " + str(interpreter))
    if pyulog_site and not (Path(pyulog_site) / "pyulog").is_dir():
        raise ValueError("Supplied existing PyULog package directory is missing")
    require_idle(ROOT)
    _, provenance = preflight(cfg)
    if provenance.get("accepted_response_binding") != binding:
        raise RuntimeError("Preflight accepted response binding changed before launch")
    output.mkdir(parents=True, exist_ok=False)
    status_path = output / "job_status.json"
    atomic_json(output / "task.json", cfg)
    status = {
        "kind": "x650_mixed_stage_job", "schema_version": 1,
        "stage": cfg["stage"], "controller": cfg["controller"], "complete": False, "started_utc": now(),
        "task_sha256": canonical_hash(cfg),
        "accepted_response_binding": binding,
        "preflight": provenance, "phase": "preflight",
        "failure_policy": "Stop; preserve evidence; no automatic retry or repair",
    }
    atomic_json(status_path, status)
    before_runs = set((ROOT / "runs").glob(cfg["scenario"] + "_*"))
    before_launches = set((ROOT / "runs").glob("x650_wb_launch_*"))
    env = os.environ.copy()
    env.update(PYTHONPATH=str(ROOT / "src") + ":/usr/lib/python3/dist-packages",
               PYTHONUNBUFFERED="1", OPENBLAS_NUM_THREADS="1",
               OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    try:
        command = [str(launch_python), str(ROOT / "simulations/x650_wholebody_mpc_v1/launch.py"),
                   "--task", str(output / "task.json")]
        status.update(phase="flight", command=command)
        atomic_json(status_path, status)
        with (output / "launch.log").open("w", encoding="utf-8") as log:
            result = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        status["launch_exit_code"] = result.returncode
        # Always record which launch was actually created, including failures.
        runs = set((ROOT / "runs").glob(cfg["scenario"] + "_*")) - before_runs
        launches = set((ROOT / "runs").glob("x650_wb_launch_*")) - before_launches
        status.update(run_directories=sorted(str(p) for p in runs),
                      launch_directories=sorted(str(p) for p in launches))
        atomic_json(status_path, status)
        status["idle_after"] = require_idle(ROOT)
        if result.returncode != 0 or len(runs) != 1 or len(launches) != 1:
            raise RuntimeError("Flight failed or new run/launch identity is ambiguous")
        run, launch = next(iter(runs)), next(iter(launches))
        launch_meta = read_json(launch / "launch.json")
        if (launch_meta.get("complete") is not True or launch_meta.get("runtime_closed") is not True
                or canonical_hash(launch_meta.get("task")) != canonical_hash(cfg)
                or [str(Path(p).resolve()) for p in launch_meta.get("run_directories", [])] != [str(run.resolve())]):
            raise RuntimeError("Launcher did not bind this exact completed flight")
        require_completed_run(run, cfg)
        if load_accepted_response_model(cfg)["binding"] != binding:
            raise RuntimeError("Accepted response binding changed during stage")
        status.update(phase="audit", run_directory=str(run), launch_directory=str(launch))
        atomic_json(status_path, status)
        analysis_env = env.copy()
        analysis_env.pop("PYTHONPATH", None)
        script = ROOT / "scripts/audit_x650_mixed_stage.py"
        args = [str(script), "--run-dir", str(run), "--launch-dir", str(launch),
                "--output", str(output / "audit")]
        if pyulog_site:
            bootstrap = ("import sys,runpy; sys.path.append(sys.argv.pop(1)); "
                         "script=sys.argv.pop(1); sys.argv[0]=script; "
                         "runpy.run_path(script,run_name='__main__')")
            command = [str(analysis_python), "-c", bootstrap, str(pyulog_site), *args]
        else:
            command = [str(analysis_python), *args]
        with (output / "audit.log").open("w", encoding="utf-8") as log:
            result = subprocess.run(command, cwd=ROOT, env=analysis_env, stdout=log, stderr=subprocess.STDOUT)
        status["audit_exit_code"] = result.returncode
        require_idle(ROOT)
        if result.returncode != 0:
            raise RuntimeError("Independent mixed-stage audit failed")
        report = read_json(output / "audit/mixed_mpc_stage_audit.json")
        verified = ("complete", "passed", "research_acceptance_verified", "full_window_ulog_verified",
                    "parameter_restore_verified", "upper_command_verified", "feedback_contract_verified")
        if (report.get("kind") != "x650_precision_mixed_stage_audit" or report.get("schema_version") != 3
                or any(report.get(key) is not True for key in verified)
                or report.get("source_run") != run.name
                or Path(report.get("source_run_directory", "")).resolve() != run.resolve()
                or report.get("stage") != cfg["stage"] or report.get("controller") != cfg["controller"]
                or report.get("accepted_response_binding") != binding):
            raise RuntimeError("Independent audit does not accept this run")
        if load_accepted_response_model(cfg)["binding"] != binding:
            raise RuntimeError("Accepted response binding changed during stage")
        status.update(complete=True, phase="complete", audit_directory=str(output / "audit"),
                      audit_sha256=file_hash(output / "audit/mixed_mpc_stage_audit.json"))
    except BaseException as exc:
        status.update(phase="failed", error=repr(exc))
        raise
    finally:
        status["finished_utc"] = now()
        atomic_json(status_path, status)
    return status


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--launch-python", type=Path, required=True)
    parser.add_argument("--analysis-python", type=Path, default=Path("/usr/bin/python3"))
    parser.add_argument("--pyulog-site", type=Path)
    args = parser.parse_args(argv)
    status = run_one(task_path=args.task, output=args.output,
                     launch_python=args.launch_python, analysis_python=args.analysis_python,
                     pyulog_site=args.pyulog_site)
    print(json.dumps({key: status[key] for key in
                     ("complete", "stage", "controller", "run_directory", "launch_directory", "audit_directory")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

