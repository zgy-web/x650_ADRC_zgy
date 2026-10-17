#!/usr/bin/env python3
"""Run the packaged PID + feedforward controller through the existing AMSIM.

This launcher has no ROS or numerical dependencies. --dry-run is also usable
on the local workstation, without starting a simulator or writing files.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SCENARIO = "pid_ff_wholebody_v1"
CONFIG_ENV = "PID_FF_TASK_CONFIG"
MODES = {"off": "baseline", "on": "compensated",
         "baseline": "baseline", "compensated": "compensated"}


def build_task_config(trajectory="hover", mode="compensated"):
    """Named presets preserve the validated PID, allocation and speed limits."""
    if trajectory not in ("hover", "normal", "fast"):
        raise ValueError("trajectory must be hover, normal or fast")
    try:
        mode = MODES[mode.lower()]
    except (KeyError, AttributeError) as exc:
        raise ValueError("mode must be baseline/OFF or compensated/ON") from exc
    period = 8. if trajectory == "fast" else 12.
    return {
        "schema_version": 1,
        "controller": "flyarm_pid_ff",
        "scenario": SCENARIO,
        "model": "x500_underfold",
        "mode": mode,
        "trajectory": trajectory,
        "trajectory_parameters": None if trajectory == "hover" else {
            "amplitude_x": .25, "amplitude_z": .10, "period_s": period,
            "cycles": 2, "start_s": 2., "ramp_s": 2.,
        },
        "duration_s": 8. if trajectory == "hover" else 2. + 2. * period + 2. + 4.,
        "hover_height_m": 2.5,
        "control_period_s": .02,
        "nominal_arm_share": .2,
        "feedforward_enabled": True,
        "simulation_only": True,
        "external_disturbance": False,
    }


def validate_task_config(value):
    """Reject unsupported edits instead of silently retuning a frozen preset."""
    if not isinstance(value, dict):
        raise ValueError("Task configuration must be a JSON object")
    expected = build_task_config(value.get("trajectory"), value.get("mode"))
    if value != expected:
        changed = sorted(k for k in set(value) | set(expected)
                         if value.get(k) != expected.get(k))
        raise ValueError("Unsupported task configuration fields: " + ", ".join(changed))
    return expected


def selected_task_config():
    raw_path = os.environ.get(CONFIG_ENV)
    if not raw_path:
        return build_task_config()
    return validate_task_config(json.loads(Path(raw_path).expanduser().read_text(encoding="utf-8")))


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


@contextmanager
def flight_lock(root):
    """Share the already established flight lock with the formal experiments."""
    try:
        import fcntl
    except ImportError as exc:
        raise RuntimeError("A real AMSIM flight must run on the Linux simulation server") from exc
    path = root / "benchmark/whole_body_figure8_v1/flight.lock"
    if not path.parent.is_dir():
        raise RuntimeError("Existing server flight-lock directory is missing: " + str(path.parent))
    # Append mode neither truncates nor rewrites an existing lock file.
    with path.open("a") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("Another flight owns the simulation flight lock") from exc
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", choices=["hover", "normal", "fast"], default="hover")
    parser.add_argument("--mode", choices=list(MODES) + ["OFF", "ON"], default="compensated")
    parser.add_argument("--dry-run", action="store_true", help="Print the plan; do not start or change anything")
    args = parser.parse_args(argv)
    cfg = build_task_config(args.trajectory, args.mode)
    launcher = ROOT / "scripts/amsim"
    descriptor = ROOT / "scenarios" / (SCENARIO + ".yaml")
    package = ROOT / "control_packages/pid_ff_wholebody_v1/flyarm_pid_ff/__init__.py"
    runtime = ROOT / ".amsim/runtime.json"
    command = [str(launcher), "run", SCENARIO, "--headless"]
    platform = {"launcher_exists": launcher.is_file(),
                "scenario_exists": descriptor.is_file(),
                "controller_package_exists": package.is_file(),
                "runtime_active": runtime.exists()}
    if args.dry_run:
        print(json.dumps({"dry_run": True, "root": str(ROOT), "command": command,
                          "environment_variable": CONFIG_ENV, "task": cfg,
                          "platform": platform}, indent=2))
        return 0
    if not all(platform[k] for k in ("launcher_exists", "scenario_exists", "controller_package_exists")):
        raise RuntimeError("Deploy the package and new scenario into the existing AMSIM root first: " + str(platform))
    with flight_lock(ROOT):
        if runtime.exists():
            raise RuntimeError("An AMSIM runtime already exists; inspect it before starting another flight")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = ROOT / "runs" / ("pid_ff_launch_" + stamp + "_" + str(os.getpid()))
        output.mkdir(parents=True, exist_ok=False)
        config_path = output / "task.json"
        atomic_json(config_path, cfg)
        env = os.environ.copy()
        env.update({CONFIG_ENV: str(config_path), "PYTHONUNBUFFERED": "1"})
        if "AMSIM_VENV" not in env and Path("/root/autodl-tmp/amsim-venv").is_dir():
            env["AMSIM_VENV"] = "/root/autodl-tmp/amsim-venv"
        before = set((ROOT / "runs").glob(SCENARIO + "_*"))
        state = {"complete": False, "command": command, "task": cfg,
                 "started_utc": datetime.now(timezone.utc).isoformat(),
                 "log": str(output / "flight.log")}
        atomic_json(output / "launch.json", state)
        print("PID_FF_LAUNCH", json.dumps(state), flush=True)
        try:
            with (output / "flight.log").open("w", encoding="utf-8") as log:
                result = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
            state["exit_code"] = result.returncode
            created = set((ROOT / "runs").glob(SCENARIO + "_*")) - before
            state["run_directories"] = [str(p) for p in sorted(created)]
            if len(created) == 1:
                run = next(iter(created))
                for name, key in (("result.json", "amsim_result"), ("controller_metadata.json", "controller_metadata")):
                    if (run / name).is_file():
                        state[key] = json.loads((run / name).read_text(encoding="utf-8"))
            state["runtime_closed"] = not runtime.exists()
            state["complete"] = (result.returncode == 0 and len(created) == 1
                                 and state.get("amsim_result", {}).get("success", False)
                                 and state.get("controller_metadata", {}).get("complete", False)
                                 and state["runtime_closed"])
            if not state["complete"]:
                raise RuntimeError("Flight did not finish cleanly; preserve and inspect " + str(output))
        except BaseException as exc:
            state["error"] = repr(exc)
            raise
        finally:
            state["finished_utc"] = datetime.now(timezone.utc).isoformat()
            atomic_json(output / "launch.json", state)
        print("PID_FF_COMPLETE", json.dumps({"launch": str(output), "runs": state["run_directories"]}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
