#!/usr/bin/env python3
"""Fresh TS20 hover/probe/quiet chain bound to its current software proof.

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

REQUIRED_SOURCES = tuple("control_packages/x650_wholebody_v1/x650_wholebody/" + name
                         for name in ("ts20_prediction.py", "mpc.py", "ts20_mpc.py",
                                      "qp_constraints.py", "ts20_runtime.py"))
NEW_PREDICTOR_SHA256 = "f25e45d755c236cd46c0e4fd2e11fb23fe926bc1b7e286dd87c9ac03414b7b66"


def digest(content):
    return hashlib.sha256(content).hexdigest()


def verify_software_proof(root, cfg, proof_path, expected_sha256):
    """Require the reviewed proof, its actual deployment, and this task's oracle.

    No optimization or simulation runs here. The expected digest is supplied
    explicitly after review; a proof's own declared hash is not an authority.
    """
    root, proof_path = Path(root).resolve(), Path(proof_path).resolve()
    if (not isinstance(expected_sha256, str) or len(expected_sha256) != 64
            or set(expected_sha256)-set("0123456789abcdef")
            or task.sha(proof_path) != expected_sha256):
        raise RuntimeError("Reviewed current software proof hash differs")
    proof = task.read(proof_path)
    sources = proof.get("deployment_sources")
    if (not isinstance(sources, dict) or not set(REQUIRED_SOURCES).issubset(sources)
            or proof.get("no_flight") is not True):
        raise RuntimeError("Current software proof lacks explicit deployment scope/no-flight evidence")
    for relative, expected in sources.items():
        path = (root / relative).resolve()
        try:
            inside = path.relative_to(root).as_posix()
        except ValueError:
            raise RuntimeError("Software source path escapes project") from None
        if inside != relative or task.sha(path) != expected:
            raise RuntimeError("Software deployment source differs: " + relative)
    predictor_sha = sources[REQUIRED_SOURCES[0]]
    if predictor_sha != NEW_PREDICTOR_SHA256:
        raise RuntimeError("This chain requires the newly validated FFI-only predictor")
    native, oracle = proof.get("native_library", {}), proof.get("oracle", {})
    inputs = cfg["ts20_inputs"]
    if (Path(native.get("path", "")).resolve() != Path(inputs["native_library"]).resolve()
            or native.get("sha256") != inputs["native_library_sha256"]
            or task.sha(inputs["native_library"]) != native.get("sha256")):
        raise RuntimeError("Software proof/native task library differs")
    cpp = Path(native.get("cpp_path", ""))
    cpp = cpp.resolve() if cpp.is_absolute() else (root / cpp).resolve()
    try:
        cpp_relative = cpp.relative_to(root).as_posix()
    except ValueError:
        raise RuntimeError("Native source must be inside the project") from None
    if (task.sha(cpp) != native.get("cpp_sha256")
            or sources.get(cpp_relative) != native.get("cpp_sha256")):
        raise RuntimeError("Native source/deployment proof differs")
    oracle_path = Path(oracle.get("path", "")).resolve()
    if (oracle_path != Path(inputs["oracle_validation"]).resolve()
            or task.sha(oracle_path) != oracle.get("sha256")
            or oracle.get("candidate_sha256") != predictor_sha
            or oracle.get("pass") is not True):
        raise RuntimeError("This task requires the new matching oracle validation")
    suite, matrix = proof.get("package_regression", {}), proof.get("software_matrix", {})
    if (type(suite.get("tests")) is not int or suite["tests"] <= 0
            or suite.get("failures") != 0 or suite.get("errors") != 0
            or matrix.get("total") != 30 or matrix.get("passed") != 30
            or matrix.get("failures") not in ([], 0)
            or not isinstance(matrix.get("max_wall_ms"), (int, float))
            or not 0 <= matrix["max_wall_ms"] <= 50.):
        raise RuntimeError("Current package/unchanged 30-case deadline acceptance is missing")
    dependencies = {"oracle": {"path": str(oracle_path), "sha256": oracle["sha256"]}}
    for name, value, path_key, hash_key in (
            ("package_report", suite, "report_path", "report_sha256"),
            ("matrix_summary", matrix, "summary_path", "summary_sha256")):
        path = Path(value[path_key])
        path = path.resolve() if path.is_absolute() else (root / path).resolve()
        if task.sha(path) != value[hash_key]:
            raise RuntimeError("Current software evidence differs: " + name)
        dependencies[name] = {"path": str(path), "sha256": value[hash_key]}
    return {"source_path": str(proof_path), "sha256": expected_sha256,
            "deployment_sources": sources, "native_library": native,
            "oracle": oracle, "dependencies": dependencies}


def archive_software(root, output, checked):
    archive_path = output / "software_sources.zip"
    with zipfile.ZipFile(archive_path, "x", zipfile.ZIP_DEFLATED) as archive:
        for relative, expected in checked["deployment_sources"].items():
            content = (root / relative).read_bytes()
            if digest(content) != expected:
                raise RuntimeError("Software source changed while archiving: " + relative)
            archive.writestr(relative, content)
    evidence = {}
    for name, entry in {"software_acceptance": {"path": checked["source_path"],
                                                 "sha256": checked["sha256"]},
                        **checked["dependencies"]}.items():
        content = Path(entry["path"]).read_bytes()
        if digest(content) != entry["sha256"]:
            raise RuntimeError("Software evidence changed while archiving")
        path = output / (name + (".log" if name == "package_report" else ".json"))
        with path.open("xb") as stream:
            stream.write(content)
        evidence[name] = {"file": path.name, "sha256": entry["sha256"]}
    return {"binding": checked, "archive": {"file": archive_path.name,
                                             "sha256": task.sha(archive_path)}, "evidence": evidence}


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


def capture_before(root, output, cfg, provenance, software):
    before = hashes(root, platform_files(root, cfg["scenario"]))
    archive_path = output / "platform_sources.zip"
    with zipfile.ZipFile(archive_path, "x", zipfile.ZIP_DEFLATED) as archive:
        for relative, expected in before.items():
            content = (root / relative).read_bytes()
            if digest(content) != expected:
                raise RuntimeError("Platform source changed while archiving: " + relative)
            archive.writestr(relative, content)
    software_archive = archive_software(root, output, software)
    record = {"kind": "x650_chain_platform_source_manifest_v2", "captured_unix": time.time(),
        "hostname": socket.gethostname(), "launcher_pid": os.getpid(), "scenario": cfg["scenario"],
        "files": before, "archive": {"file": archive_path.name, "sha256": task.sha(archive_path)},
        "software_acceptance": software_archive["evidence"]["software_acceptance"],
        "software_archive": software_archive,
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
    current_software = verify_software_proof(root, cfg, before["software_archive"]["binding"]["source_path"],
                                             before["software_acceptance"]["sha256"])
    record = {"kind": "x650_chain_platform_provenance_v2", "capture_complete": True,
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
        "software_archive": before["software_archive"],
        "software_binding_unchanged": current_software == before["software_archive"]["binding"],
        "git_before": before["git"], "git_after": git_snapshot(root, output, "after"),
        "semantics": "Supplementary evidence for this new run; no reconstruction of historical platform sources"}
    record["source_unchanged"] = all(record[name] for name in (
        "platform_source_scope_equal", "platform_source_bytes_equal",
        "controller_source_scope_equal", "controller_source_bytes_equal", "software_binding_unchanged"))
    atomic_json(output / "platform_provenance.json", record)
    return record


def verify_platform(launch_directory, run_directory, *, expected_software_sha256):
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
            or record["software_acceptance"]["sha256"] != expected_software_sha256
            or record["captured_before_unix"] > record["captured_after_unix"]):
        raise RuntimeError("Before/after source identity differs")
    with zipfile.ZipFile(output / record["archive"]["file"]) as archive:
        if len(archive.namelist()) != len(set(archive.namelist())) or set(archive.namelist()) != set(before["files"]):
            raise RuntimeError("Platform source archive scope differs")
        for relative, expected in before["files"].items():
            if digest(archive.read(relative)) != expected:
                raise RuntimeError("Platform source archive byte mismatch: " + relative)
    software = record["software_archive"]
    archived_proof = task.read(output / software["evidence"]["software_acceptance"]["file"])
    if (software != before["software_archive"] or record.get("software_binding_unchanged") is not True
            or software["binding"]["sha256"] != expected_software_sha256
            or archived_proof["deployment_sources"] != software["binding"]["deployment_sources"]
            or archived_proof["native_library"] != software["binding"]["native_library"]
            or archived_proof["oracle"] != software["binding"]["oracle"]):
        raise RuntimeError("Archived software proof/deployment graph differs")
    for entry in [software["archive"], *software["evidence"].values()]:
        path = (output / entry["file"]).resolve()
        if path.parent != output or task.sha(path) != entry["sha256"]:
            raise RuntimeError("Archived current software evidence differs")
    with zipfile.ZipFile(output / software["archive"]["file"]) as archive:
        expected = software["binding"]["deployment_sources"]
        if len(archive.namelist()) != len(set(archive.namelist())) or set(archive.namelist()) != set(expected):
            raise RuntimeError("Current software archive scope differs")
        if any(digest(archive.read(name)) != value for name, value in expected.items()):
            raise RuntimeError("Current software archive bytes differ")
    return {"verified": True, "run_directory": str(run), "launch_directory": str(output),
            "source_unchanged": True, "platform_source_files": len(before["files"]),
            "controller_source_files": len(before["controller_files"]),
            "platform_provenance_sha256": task.sha(output / "platform_provenance.json"),
            "platform_source_manifest_sha256": task.sha(output / "platform_source_manifest.json"),
            "platform_source_archive_sha256": record["archive"]["sha256"],
            "git_head_before": record["git_before"]["head_value"],
            "git_head_after": record["git_after"]["head_value"],
            "software_acceptance_sha256": expected_software_sha256}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", type=Path, required=True)
    parser.add_argument("--software-proof", type=Path, required=True)
    parser.add_argument("--software-sha256", required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    cfg = task.validate_task_config(task.read(args.task))
    if cfg["scenario"] != task.DAMPED_SCENARIO or cfg["disturbance_enabled"]:
        raise ValueError("This launcher admits only the new damped hover/probe/quiet chain")
    command = [str(ROOT / "scripts/amsim"), "run", cfg["scenario"], "--headless"]
    _, provenance = task.preflight(cfg)
    software = verify_software_proof(ROOT, cfg, args.software_proof, args.software_sha256)
    if args.dry_run:
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
        before_platform = capture_before(ROOT, out, cfg, provenance, software)
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
