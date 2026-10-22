#!/usr/bin/env python3
"""Launch one staged native X650 experiment in the existing AMSIM runtime."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from task import CONFIG_ENV, STAGES, build_task_config, preflight, validate_task_config
sys.path.insert(0, str(ROOT))
from simulations.pid_ff_wholebody_v1.launch import atomic_json, flight_lock


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--stage", choices=STAGES, default="hover_folded")
    p.add_argument("--controller", choices=("pid_ff", "mpc"), default="pid_ff")
    p.add_argument("--task", type=Path, help="Explicit reviewed JSON inputs; overrides preset selection")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args(argv)
    cfg = validate_task_config(json.loads(args.task.read_text(encoding="utf-8"))) if args.task else build_task_config(args.stage, args.controller)
    command = [str(ROOT/"scripts/amsim"), "run", cfg["scenario"], "--headless"]
    if args.dry_run:
        try:
            _, provenance = preflight(cfg)
            check = {"ready": True, "provenance": provenance}
        except Exception as exc:
            check = {"ready": False, "reason": str(exc)}
        print(json.dumps({"dry_run": True, "task": cfg, "preflight": check, "command": command}, indent=2))
        return 0
    # Validate all hashes, stage artifacts and native inputs before starting any process.
    _, provenance = preflight(cfg)
    runtime = ROOT/".amsim/runtime.json"
    with flight_lock(ROOT):
        if runtime.exists():
            raise RuntimeError("Existing AMSIM runtime needs inspection before another flight")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        out = ROOT/"runs"/("x650_wb_launch_"+stamp+"_"+str(os.getpid()))
        out.mkdir(parents=True, exist_ok=False)
        atomic_json(out/"task.json", cfg)
        candidate_archive = None
        binding = provenance.get("frozen_response_candidate_binding")
        if binding is not None:
            from task import sha
            candidate = Path(binding["path"])
            if sha(candidate) != binding["sha256"]:
                raise RuntimeError("Frozen response candidate changed after preflight")
            archived = out/"frozen_response_candidate.json"
            archived.write_bytes(candidate.read_bytes())
            if sha(archived) != binding["sha256"]:
                raise RuntimeError("Frozen response candidate archive digest mismatch")
            candidate_archive = {"path": str(archived), "sha256": binding["sha256"],
                                 "source_path": binding["path"]}
        accepted_archive = None
        accepted_binding = provenance.get("accepted_response_binding")
        if accepted_binding is not None:
            from task import sha
            accepted_archive = {}
            for key in ("model", "evidence", "preregistration", "source_archive", "candidate", "training_report"):
                binding = accepted_binding[key]
                source = Path(binding["path"])
                if sha(source) != binding["sha256"]:
                    raise RuntimeError("Accepted response changed after preflight: "+key)
                archived = out/("accepted_response_"+key+(".zip" if key == "source_archive" else ".json"))
                archived.write_bytes(source.read_bytes())
                if sha(archived) != binding["sha256"]:
                    raise RuntimeError("Accepted response archive checksum mismatch: "+key)
                accepted_archive[key] = {"path": str(archived), "sha256": binding["sha256"], "source_path": str(source)}
        with zipfile.ZipFile(out/"controller_sources.zip", "w", zipfile.ZIP_DEFLATED) as archive:
            for relative, expected in provenance["source_files_sha256"].items():
                from task import sha
                source = ROOT/relative
                if sha(source) != expected:
                    raise RuntimeError("Controller source changed after preflight: "+relative)
                archive.write(source, relative)
        env = os.environ.copy()
        env.update({CONFIG_ENV: str(out/"task.json"), "PYTHONUNBUFFERED": "1",
                    "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"})
        env["PX4_GZ_MODEL_POSE"] = ",".join(str(v) for v in cfg["spawn_pose_enu_xyz_rpy"])
        if "AMSIM_VENV" not in env and Path("/root/autodl-tmp/amsim-venv").is_dir():
            env["AMSIM_VENV"] = "/root/autodl-tmp/amsim-venv"
        before = set((ROOT/"runs").glob(cfg["scenario"]+"_*"))
        status = {"complete": False, "task": cfg, "provenance": provenance,
                  "command": command, "started_utc": datetime.now(timezone.utc).isoformat()}
        if candidate_archive is not None:
            status["frozen_response_candidate_archive"] = candidate_archive
        if accepted_archive is not None:
            status["accepted_response_archive"] = accepted_archive
        atomic_json(out/"launch.json", status)
        print("X650_LAUNCH", str(out), flush=True)
        try:
            with (out/"flight.log").open("w", encoding="utf-8") as log:
                result = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
            created = set((ROOT/"runs").glob(cfg["scenario"]+"_*"))-before
            status.update(exit_code=result.returncode, run_directories=[str(v) for v in sorted(created)], runtime_closed=not runtime.exists())
            if len(created) == 1:
                run = next(iter(created))
                if (run/"result.json").is_file():
                    status["amsim_result"] = json.loads((run/"result.json").read_text(encoding="utf-8"))
            status["complete"] = (result.returncode == 0 and len(created) == 1
                                  and status.get("amsim_result", {}).get("success") is True
                                  and status["runtime_closed"])
            if not status["complete"]:
                raise RuntimeError("Native experiment not complete; inspect "+str(out))
        except BaseException as exc:
            status["error"] = repr(exc)
            raise
        finally:
            status["finished_utc"] = datetime.now(timezone.utc).isoformat()
            atomic_json(out/"launch.json", status)
        print("X650_COMPLETE", json.dumps(status), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
