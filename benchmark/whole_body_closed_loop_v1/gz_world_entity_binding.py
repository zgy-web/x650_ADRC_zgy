"""Read-only live ECS identity capture for an explicitly owned AMSIM runtime.

The C++ probe copies a SerializedStepMap into its own ECM. This wrapper never
loads systems, changes model state, starts Gazebo, or applies a wrench.
"""
from __future__ import annotations
import hashlib
import json
import math
import re
from pathlib import Path
import subprocess
import time

NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_-]*")
SCHEMA = "gz_ecs_world_entity_binding_v1"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _positive_id(value):
    return type(value) is int and 0 < value < 2**64


def _process_identity(pid):
    if not _positive_id(pid):
        raise ValueError("Actual runtime requires a positive PID")
    # /proc/<pid>/stat field 2 can contain spaces; field 22 is starttime.
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    if fields[0] in {"Z", "X", "x"}:
        raise ValueError("Actual runtime process is a zombie or dead")
    return {"pid": pid, "start_ticks": int(fields[19]),
            "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip()}


def runtime_identity(root, instance):
    root = Path(root).resolve(strict=True)
    if not isinstance(instance, str) or not NAME.fullmatch(instance):
        raise ValueError("Explicit safe model instance required")
    state_path = root / ".amsim/runtime.json"
    state = json.loads(state_path.read_text())
    if state.get("ready") is not True:
        raise ValueError("AMSIM runtime is not ready")
    meta_path = Path(state["runtime_meta"])
    meta_path = (meta_path if meta_path.is_absolute() else root / meta_path).resolve(strict=True)
    meta = json.loads(meta_path.read_text())
    if meta.get("instance") != instance or state.get("model") != meta.get("model"):
        raise ValueError("Actual runtime metadata model/instance mismatch")
    worlds = set()
    if meta.get("world"):
        worlds.add(meta["world"])
    for value in meta.values():
        if isinstance(value, str):
            found = re.match(r"^/world/([A-Za-z_][A-Za-z0-9_-]*)/model/" + re.escape(instance) + r"/", value)
            if found:
                worlds.add(found.group(1))
    if len(worlds) != 1 or not NAME.fullmatch(next(iter(worlds), "")):
        raise ValueError("Runtime metadata must identify exactly one safe world")
    return {"project_root": str(root), "world": worlds.pop(), "model": instance,
            "runtime_state_path": str(state_path), "runtime_state_sha256": sha(state_path),
            "runtime_meta_path": str(meta_path), "runtime_meta_sha256": sha(meta_path),
            "process": _process_identity(state["pid"])}


def validate_probe(probe, world, model, link="base_link"):
    if (not isinstance(probe, dict) or probe.get("schema") != SCHEMA
            or probe.get("verified") is not True or probe.get("read_only") is not True
            or probe.get("api") != "gz_sim8_SerializedStepMap_local_ECM"):
        raise ValueError("A successful read-only ECS probe is required")
    if any(probe.get(k) != v for k, v in {"world": world, "model": model, "link": link,
                                          "service": f"/world/{world}/state"}.items()):
        raise ValueError("Probe world/model/link/service identity mismatch")
    ids = [probe.get(k) for k in ("world_id", "model_id", "link_id")]
    if any(not _positive_id(v) for v in ids) or len(set(ids)) != 3:
        raise ValueError("Distinct positive world/model/link IDs are required")
    if probe.get("model_parent_id") != ids[0] or probe.get("link_parent_id") != ids[1]:
        raise ValueError("ECS parent chain does not bind link to model to world")
    types = probe.get("component_types")
    if (not isinstance(types, dict) or set(types) != {"World", "Name", "Model", "Link", "ParentEntity"}
            or any(not _positive_id(v) for v in types.values()) or len(set(types.values())) != 5):
        raise ValueError("Probe must report five distinct registered component type IDs")
    if (type(probe.get("sim_sec")) is not int or probe["sim_sec"] < 0
            or type(probe.get("sim_nsec")) is not int or not 0 <= probe["sim_nsec"] < 10**9):
        raise ValueError("Invalid native simulation timestamp")
    return probe


def capture_binding(root, instance, output, executable, pump, *, timeout_s=8.0):
    """Create a fresh identity artifact. The caller owns the existing runtime."""
    if not math.isfinite(timeout_s) or not 1 <= timeout_s <= 15:
        raise ValueError("Finite read-only probe timeout from 1 to 15 seconds required")
    before = runtime_identity(root, instance)
    executable = Path(executable).resolve(strict=True)
    executable_sha = sha(executable)
    out = Path(output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    snapshot, stdout, stderr = [out / n for n in ("ecs_state.pb", "probe_stdout.json", "probe_stderr.log")]
    command = [str(executable), "--world", before["world"], "--model", instance,
               "--link", "base_link", "--timeout-ms", str(min(10000, max(100, int((timeout_s - .5) * 1000)))),
               "--state-output", str(snapshot)]
    # Inherits the caller's GZ transport environment/partition.
    with stdout.open("x") as sout, stderr.open("x") as serr:
        process = subprocess.Popen(command, stdout=sout, stderr=serr)
        try:
            deadline = time.monotonic() + timeout_s
            while process.poll() is None:
                pump()
                if time.monotonic() > deadline:
                    raise TimeoutError("Read-only ECS identity probe timed out")
                time.sleep(.002)
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
    if process.returncode != 0:
        raise RuntimeError("Read-only ECS probe failed; inspect " + str(stderr))
    probe = validate_probe(json.loads(stdout.read_text()), before["world"], instance)
    if not snapshot.is_file() or snapshot.stat().st_size == 0:
        raise ValueError("Missing raw ECS response evidence")
    after = runtime_identity(root, instance)
    if before != after or sha(executable) != executable_sha:
        raise ValueError("Runtime or read-only executable changed during identity capture")
    document = {"schema": SCHEMA, "complete": True, "runtime": before, "probe": probe,
                "captured_unix": time.time(), "probe_executable": str(executable),
                "probe_executable_sha256": executable_sha,
                "wrapper_source_sha256": sha(__file__),
                "evidence": {p.name: sha(p) for p in (snapshot, stdout, stderr)},
                "semantics": "Read-only ECS identity only; no disturbance load or force verification"}
    path = out / "entity_binding.json"
    with path.open("x") as stream:
        stream.write(json.dumps(document, indent=2, allow_nan=False) + "\n")
    return path


def validate_binding_for_load(path, instance, *, max_age_s=15.0):
    """Recheck immutable evidence and the current runtime before dynamic load."""
    path = Path(path).resolve(strict=True)
    data = json.loads(path.read_text())
    if data.get("schema") != SCHEMA or data.get("complete") is not True:
        raise ValueError("Incomplete live ECS identity binding")
    stamp = data.get("captured_unix")
    if (type(stamp) not in (int, float) or not math.isfinite(stamp)
            or not math.isfinite(max_age_s) or not 0 < max_age_s <= 15
            or not 0 <= time.time() - stamp <= max_age_s):
        raise ValueError("Identity binding is stale or from a future clock")
    identity = data["runtime"]
    if runtime_identity(identity["project_root"], instance) != identity:
        raise ValueError("Identity binding belongs to a changed/different runtime")
    probe = validate_probe(data["probe"], identity["world"], instance)
    expected_files = {"ecs_state.pb", "probe_stdout.json", "probe_stderr.log"}
    if set(data.get("evidence", {})) != expected_files:
        raise ValueError("Complete ECS probe evidence required")
    for name, digest in data["evidence"].items():
        if sha(path.parent / name) != digest:
            raise ValueError("ECS probe evidence hash mismatch: " + name)
    if json.loads((path.parent / "probe_stdout.json").read_text()) != probe:
        raise ValueError("Binding disagrees with captured probe output")
    if sha(data["probe_executable"]) != data["probe_executable_sha256"] or sha(__file__) != data["wrapper_source_sha256"]:
        raise ValueError("Identity verifier binary/source changed after capture")
    return {**probe, "binding_path": str(path), "binding_sha256": sha(path),
            "runtime_meta_sha256": identity["runtime_meta_sha256"]}


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--probe", type=Path, required=True)
    args = parser.parse_args()
    print(capture_binding(args.project_root, args.instance, args.output, args.probe, lambda: None))
