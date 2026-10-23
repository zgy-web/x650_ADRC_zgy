#!/usr/bin/env python3
"""One fresh quiet hold with supplementary actual-platform source evidence.

The controller task, lifecycle and stage auditor remain unchanged. Historical
certificates are not rewritten. This launcher adds a separate provenance record
for the platform files actually present during this new experiment.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_ts20_hold_v1 import task
from simulations.pid_ff_wholebody_v1.launch import atomic_json, flight_lock

SOFTWARE = "artifacts/ts20_qp_view_acceptance_20261023/software_acceptance_compact.json"
SOFTWARE_SHA256 = "da4a4d50748f10527ad8648c1f30cd448bc76005db73e59162ec287fa44c8197"


def digest(content):
    return hashlib.sha256(content).hexdigest()


def platform_files(root, scenario):
    required = [root / "scripts/amsim", root / "scenarios" / (scenario + ".yaml"),
                root / Path(__file__).resolve().relative_to(ROOT)]
    sources = sorted(set(required + list((root / "src/amsim").rglob("*.py"))))
    if len(sources) < len(required) + 1 or not all(path.is_file() for path in sources):
        raise RuntimeError("Actual platform source scope is missing required files")
    return sources


def hashes(root, files):
    return {path.relative_to(root).as_posix(): digest(path.read_bytes()) for path in files}


def git_snapshot(root, output, prefix):
    commands = {"head": ["rev-parse", "HEAD"],
                "status": ["status", "--porcelain", "--untracked-files=no"],
                "diff": ["diff", "--no-ext-diff", "--binary", "HEAD", "--"]}
    record = {}
    for name, args in commands.items():
        result = subprocess.run(["git", *args], cwd=root, capture_output=True, check=True)
        path = output / (prefix + "_git_" + name + (".patch" if name == "diff" else ".txt"))
        with path.open("xb") as stream:
            stream.write(result.stdout)
        record[name] = {"file": path.name, "sha256": digest(result.stdout)}
        if name == "head":
            record["head_value"] = result.stdout.decode().strip()
    return record


def capture_before(root, output, cfg, provenance):
    before = hashes(root, platform_files(root, cfg["scenario"]))
    archive_path = output / "platform_sources.zip"
    with zipfile.ZipFile(archive_path, "x", zipfile.ZIP_DEFLATED) as archive:
        for relative, expected in before.items():
            content = (root / relative).read_bytes()
            if digest(content) != expected:
                raise RuntimeError("Platform source changed while archiving: " + relative)
            archive.writestr(relative, content)
    software = (root / SOFTWARE).read_bytes()
    if digest(software) != SOFTWARE_SHA256:
        raise RuntimeError("Reviewed software acceptance summary changed")
    with (output / "software_acceptance.json").open("xb") as stream:
        stream.write(software)
    record = {"kind": "x650_quiet_platform_source_manifest_v1", "captured_unix": time.time(),
        "hostname": socket.gethostname(), "launcher_pid": os.getpid(), "scenario": cfg["scenario"],
        "files": before, "archive": {"file": archive_path.name, "sha256": task.sha(archive_path)},
        "software_acceptance": {"file": "software_acceptance.json", "sha256": SOFTWARE_SHA256},
        "controller_files": provenance["source_files_sha256"],
        "git": git_snapshot(root, output, "before"),
        "scope": "scripts/amsim, selected scenario, this launcher and all src/amsim Python files; includes untracked sources",
        "git_status_scope": "tracked files only; selected source scopes are enumerated independently"}
    atomic_json(output / "platform_source_manifest.json", record)
    return record


def capture_after(root, output, cfg, before, run_directories):
    after = hashes(root, platform_files(root, cfg["scenario"]))
    current_controller = hashes(root, task.source_files_for_task(cfg))
    manifest = output / "platform_source_manifest.json"
    record = {"kind": "x650_quiet_platform_provenance_v1", "capture_complete": True,
        "launch_directory": str(output.resolve()), "run_directories": run_directories,
        "scenario": cfg["scenario"], "hostname": socket.gethostname(), "launcher_pid": os.getpid(),
        "captured_before_unix": before["captured_unix"], "captured_after_unix": time.time(),
        "platform_source_scope_equal": set(after) == set(before["files"]),
        "platform_source_bytes_equal": after == before["files"],
        "controller_source_scope_equal": set(current_controller) == set(before["controller_files"]),
        "controller_source_bytes_equal": current_controller == before["controller_files"],
        "files_after": after, "controller_files_after": current_controller,
        "before_manifest": {"file": manifest.name, "sha256": task.sha(manifest)},
        "archive": before["archive"], "software_acceptance": before["software_acceptance"],
        "git_before": before["git"], "git_after": git_snapshot(root, output, "after"),
        "semantics": "Supplementary evidence for this new run; no reconstruction of historical platform sources"}
    record["source_unchanged"] = all(record[name] for name in (
        "platform_source_scope_equal", "platform_source_bytes_equal",
        "controller_source_scope_equal", "controller_source_bytes_equal"))
    atomic_json(output / "platform_provenance.json", record)
    return record


def verify_platform(launch_directory, run_directory):
    """Independently verify the supplemental archive and same-run binding."""
    output, run = Path(launch_directory).resolve(), Path(run_directory).resolve()
    launch = task.read(output / "launch.json")
    record = task.read(output / "platform_provenance.json")
    before = task.read(output / "platform_source_manifest.json")
    binding = launch.get("platform_provenance", {})
    if (launch.get("complete") is not True or launch.get("runtime_closed") is not True
            or record.get("capture_complete") is not True or record.get("source_unchanged") is not True
            or record.get("run_directories") != [str(run)]
            or launch.get("run_directories") != [str(run)]
            or record.get("launch_directory") != str(output)
            or binding.get("file") != "platform_provenance.json"
            or binding.get("sha256") != task.sha(output / binding["file"])):
        raise RuntimeError("Supplementary platform provenance is not complete and bound to this run")
    for entry in [record["before_manifest"], record["archive"], record["software_acceptance"],
                  *record["git_before"].values(), *record["git_after"].values()]:
        if isinstance(entry, dict) and "file" in entry:
            path = (output / entry["file"]).resolve()
            if path.parent != output or task.sha(path) != entry["sha256"]:
                raise RuntimeError("Supplementary provenance artifact mismatch")
    if (before["files"] != record["files_after"]
            or before["controller_files"] != record["controller_files_after"]
            or before["controller_files"] != launch["provenance"]["source_files_sha256"]
            or record["software_acceptance"]["sha256"] != SOFTWARE_SHA256
            or record["captured_before_unix"] > record["captured_after_unix"]):
        raise RuntimeError("Before/after source identity differs")
    with zipfile.ZipFile(output / record["archive"]["file"]) as archive:
        if len(archive.namelist()) != len(set(archive.namelist())) or set(archive.namelist()) != set(before["files"]):
            raise RuntimeError("Platform source archive scope differs")
        for relative, expected in before["files"].items():
            if digest(archive.read(relative)) != expected:
                raise RuntimeError("Platform source archive byte mismatch: " + relative)
    return {"verified": True, "run_directory": str(run), "launch_directory": str(output),
            "source_unchanged": True, "platform_source_files": len(before["files"]),
            "controller_source_files": len(before["controller_files"]),
            "platform_provenance_sha256": task.sha(output / "platform_provenance.json"),
            "platform_source_manifest_sha256": task.sha(output / "platform_source_manifest.json"),
            "platform_source_archive_sha256": record["archive"]["sha256"],
            "git_head_before": record["git_before"]["head_value"],
            "git_head_after": record["git_after"]["head_value"],
            "software_acceptance_sha256": SOFTWARE_SHA256}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    cfg = task.validate_task_config(task.read(args.task))
    if cfg["stage"] != "tip_hold" or cfg["controller"] != "mpc" or cfg["disturbance_enabled"]:
        raise ValueError("This dedicated launcher only admits quiet MPC tip_hold")
    command = [str(ROOT / "scripts/amsim"), "run", cfg["scenario"], "--headless"]
    _, provenance = task.preflight(cfg)
    if args.dry_run:
        if task.sha(ROOT / SOFTWARE) != SOFTWARE_SHA256:
            raise RuntimeError("Software acceptance changed")
        print(json.dumps({"ready": True, "scenario": cfg["scenario"], "condition": provenance["ts20_operating_condition_sha256"],
                          "platform_source_files": len(platform_files(ROOT, cfg["scenario"]))}))
        return 0
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
                content = (ROOT / relative).read_bytes()
                if digest(content) != expected:
                    raise RuntimeError("Controller source changed after preflight: " + relative)
                archive.writestr(relative, content)
        before_platform = capture_before(ROOT, out, cfg, provenance)
        env = os.environ.copy()
        env.update({task.CONFIG_ENV: str(out / "task.json"), "AMSIM_TS20_LOG_DIR": str(logs),
                    "PYTHONUNBUFFERED": "1", "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"})
        site = Path("/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages")
        if not Path("/usr/bin/python3").is_file() or not site.is_dir():
            raise RuntimeError("Reviewed existing dependency profile is unavailable")
        env.update(AMSIM_SCENARIO_PYTHON="/usr/bin/python3", AMSIM_CHECK_PYTHON="/usr/bin/python3")
        env["PYTHONPATH"] = "/usr/lib/python3/dist-packages:" + str(site) + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        env["PX4_GZ_MODEL_POSE"] = ",".join(str(value) for value in cfg["spawn_pose_enu_xyz_rpy"])
        if "AMSIM_VENV" not in env:
            env["AMSIM_VENV"] = "/root/autodl-tmp/amsim-venv"
        before_runs = set((ROOT / "runs").glob(cfg["scenario"] + "_*"))
        status = {"complete": False, "task": cfg, "provenance": provenance, "command": command,
            "started_utc": datetime.now(timezone.utc).isoformat(), "ts20_log_dir": str(logs),
            "conditional_simulation_only": True, "supply_24v_qualified": False,
            "python_dependency_profile": {"scenario_python": env["AMSIM_SCENARIO_PYTHON"],
                "numpy_scipy_source": "/usr/lib/python3/dist-packages", "existing_mavlink_ulog_source": str(site),
                "packages_installed_or_replaced": False}}
        atomic_json(out / "launch.json", status)
        print("X650_TS20_LAUNCH", str(out), flush=True)
        problem = None
        try:
            with (out / "flight.log").open("w", encoding="utf-8") as log:
                result = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
            created = set((ROOT / "runs").glob(cfg["scenario"] + "_*")) - before_runs
            status.update(exit_code=result.returncode, run_directories=[str(path) for path in sorted(created)],
                          runtime_closed=not runtime.exists())
            if len(created) == 1 and (next(iter(created)) / "result.json").is_file():
                status["amsim_result"] = task.read(next(iter(created)) / "result.json")
            status["complete"] = (result.returncode == 0 and len(created) == 1
                                  and status.get("amsim_result", {}).get("success") is True and status["runtime_closed"])
        except BaseException as exc:
            problem = exc
            status["error"] = repr(exc)
        finally:
            try:
                captured = capture_after(ROOT, out, cfg, before_platform, status.get("run_directories", []))
                status["platform_provenance"] = {"file": "platform_provenance.json",
                    "sha256": task.sha(out / "platform_provenance.json"), "source_unchanged": captured["source_unchanged"]}
                status["complete"] = status["complete"] and captured["source_unchanged"]
            except BaseException as exc:
                status["complete"] = False
                status["platform_provenance_error"] = repr(exc)
                if problem is None:
                    problem = exc
            if not status["complete"] and problem is None:
                problem = RuntimeError("TS20 quiet experiment/source identity incomplete; inspect " + str(out))
                status["error"] = repr(problem)
            status["finished_utc"] = datetime.now(timezone.utc).isoformat()
            atomic_json(out / "launch.json", status)
        if problem is not None:
            raise problem
        print("X650_TS20_COMPLETE", str(out), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
