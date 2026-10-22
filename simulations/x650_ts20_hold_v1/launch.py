#!/usr/bin/env python3
"""Launch one NEW conditional TS20 commissioning stage, with owned logs."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_ts20_hold_v1 import task
from simulations.pid_ff_wholebody_v1.launch import atomic_json, flight_lock


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    cfg = task.validate_task_config(task.read(args.task))
    command = [str(ROOT / "scripts/amsim"), "run", cfg["scenario"], "--headless"]
    if args.dry_run:
        try:
            _, provenance = task.preflight(cfg)
            status = {"ready": True, "provenance": provenance}
        except Exception as exc:
            status = {"ready": False, "reason": str(exc)}
        print(json.dumps({"dry_run": True, "task": cfg, "preflight": status, "command": command}, indent=2))
        return 0
    _, provenance = task.preflight(cfg)
    with flight_lock(ROOT):
        runtime = ROOT / ".amsim/runtime.json"
        if runtime.exists():
            raise RuntimeError("Existing simulation runtime needs inspection")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        out = ROOT / "runs" / ("x650_ts20_launch_" + stamp + "_" + str(os.getpid()))
        out.mkdir(parents=True, exist_ok=False)
        logs = out / "actuator_live"
        logs.mkdir()
        atomic_json(out / "task.json", cfg)
        with zipfile.ZipFile(out / "controller_sources.zip", "x", zipfile.ZIP_DEFLATED) as archive:
            for relative, expected in provenance["source_files_sha256"].items():
                path = ROOT / relative
                if task.sha(path) != expected:
                    raise RuntimeError("Source changed after TS20 preflight: " + relative)
                archive.write(path, relative)
        env = os.environ.copy()
        env.update({task.CONFIG_ENV: str(out / "task.json"), "AMSIM_TS20_LOG_DIR": str(logs),
                    "PYTHONUNBUFFERED": "1", "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"})
        # Reuse the installed compatible system NumPy/SciPy and the existing
        # data-disk MAVLink/ULog packages. Do not replace either installation.
        site = Path("/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages")
        if not Path("/usr/bin/python3").is_file() or not site.is_dir():
            raise RuntimeError("Reviewed existing system-Python/data-disk dependency profile is unavailable")
        env.update(AMSIM_SCENARIO_PYTHON="/usr/bin/python3", AMSIM_CHECK_PYTHON="/usr/bin/python3")
        env["PYTHONPATH"] = "/usr/lib/python3/dist-packages:" + str(site) + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        env["PX4_GZ_MODEL_POSE"] = ",".join(str(value) for value in cfg["spawn_pose_enu_xyz_rpy"])
        if "AMSIM_VENV" not in env and Path("/root/autodl-tmp/amsim-venv").is_dir():
            env["AMSIM_VENV"] = "/root/autodl-tmp/amsim-venv"
        before = set((ROOT / "runs").glob(cfg["scenario"] + "_*"))
        status = {"complete": False, "task": cfg, "provenance": provenance, "command": command,
                  "started_utc": datetime.now(timezone.utc).isoformat(), "ts20_log_dir": str(logs),
                  "conditional_simulation_only": True, "supply_24v_qualified": False}
        status["python_dependency_profile"] = {"scenario_python": env["AMSIM_SCENARIO_PYTHON"],
            "numpy_scipy_source": "/usr/lib/python3/dist-packages", "existing_mavlink_ulog_source": str(site),
            "packages_installed_or_replaced": False}
        atomic_json(out / "launch.json", status)
        print("X650_TS20_LAUNCH", str(out), flush=True)
        try:
            with (out / "flight.log").open("w", encoding="utf-8") as log:
                result = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
            created = set((ROOT / "runs").glob(cfg["scenario"] + "_*")) - before
            status.update(exit_code=result.returncode, run_directories=[str(path) for path in sorted(created)],
                          runtime_closed=not runtime.exists())
            if len(created) == 1:
                path = next(iter(created)) / "result.json"
                if path.is_file():
                    status["amsim_result"] = task.read(path)
            status["complete"] = (result.returncode == 0 and len(created) == 1
                                  and status.get("amsim_result", {}).get("success") is True and status["runtime_closed"])
            if not status["complete"]:
                raise RuntimeError("TS20 experiment incomplete; inspect " + str(out))
        except BaseException as exc:
            status["error"] = repr(exc)
            raise
        finally:
            status["finished_utc"] = datetime.now(timezone.utc).isoformat()
            atomic_json(out / "launch.json", status)
        print("X650_TS20_COMPLETE", str(out), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
