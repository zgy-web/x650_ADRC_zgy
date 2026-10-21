#!/usr/bin/env python3
"""Run one preregistered response flight and its independent audit, then stop.

Use a durable screen session on the simulation server. Training identification
and the subsequent holdout are deliberately separate commands. No automatic
retry, parameter repair, controller switch or acceptance override is provided.
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


def run_one(*, task_path, preregistration, output, launch_python,
            analysis_python, pyulog_site=None):
    task_path = Path(task_path).resolve(strict=True)
    preregistration = Path(preregistration).resolve(strict=True)
    output = Path(output).resolve()
    if output.exists():
        raise ValueError("Use a new job directory; previous attempts are preserved")
    cfg = validate_task_config(read_json(task_path))
    protocol = cfg.get("response_probe_protocol")
    if protocol is None:
        raise ValueError("This driver requires the explicit frozen response protocol")
    # The same verifier is used by fit and validate; importing the numerical
    # identifier is confined to this idle supervisor, never the flight loop.
    from identify_x650_response_protocol import verify_plan
    verify_plan(preregistration, task=cfg)
    for interpreter in (launch_python, analysis_python):
        if not Path(interpreter).is_file():
            raise ValueError("Existing Python interpreter required: " + str(interpreter))
    if pyulog_site and not (Path(pyulog_site) / "pyulog").is_dir():
        raise ValueError("Supplied existing PyULog package directory is missing")
    require_idle(ROOT)
    _, provenance = preflight(cfg)
    output.mkdir(parents=True, exist_ok=False)
    status_path = output / "job_status.json"
    atomic_json(output / "task.json", cfg)
    status = {
        "kind": "x650_frozen_response_job", "schema_version": 1,
        "role": protocol["role"], "complete": False, "started_utc": now(),
        "task_sha256": canonical_hash(cfg),
        "preregistration": str(preregistration),
        "preregistration_sha256": file_hash(preregistration),
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
        verify_plan(preregistration, task=cfg)
        status.update(phase="audit", run_directory=str(run), launch_directory=str(launch))
        atomic_json(status_path, status)
        analysis_env = env.copy()
        analysis_env.pop("PYTHONPATH", None)
        script = ROOT / "scripts/audit_x650_response_protocol.py"
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
            raise RuntimeError("Independent response audit failed")
        report = read_json(output / "audit/upper_stage_audit.json")
        if (report.get("passed") is not True or report.get("research_acceptance_verified") is not True
                or report.get("source_run") != run.name):
            raise RuntimeError("Independent audit does not accept this run")
        verify_plan(preregistration, task=cfg)
        status.update(complete=True, phase="complete", audit_directory=str(output / "audit"),
                      audit_sha256=file_hash(output / "audit/upper_stage_audit.json"))
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
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--launch-python", type=Path, required=True)
    parser.add_argument("--analysis-python", type=Path, default=Path("/usr/bin/python3"))
    parser.add_argument("--pyulog-site", type=Path)
    args = parser.parse_args(argv)
    status = run_one(task_path=args.task, preregistration=args.preregistration, output=args.output,
                     launch_python=args.launch_python, analysis_python=args.analysis_python,
                     pyulog_site=args.pyulog_site)
    print(json.dumps({key: status[key] for key in
                     ("complete", "role", "run_directory", "launch_directory", "audit_directory")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
