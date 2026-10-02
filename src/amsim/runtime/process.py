from __future__ import annotations
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from ..paths import PROJECT_ROOT, results_dir

STATE_DIR = PROJECT_ROOT / ".amsim"
STATE_FILE = STATE_DIR / "runtime.json"

def _load_state() -> dict | None:
    if not STATE_FILE.is_file():
        return None
    try:
        return json.loads(STATE_FILE.read_text())
    except Exception:
        return None

def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False

def status() -> dict:
    state = _load_state()
    if not state:
        return {"running": False, "reason": "no state file"}
    pid = int(state["pid"])
    return {**state, "running": _alive(pid)}

def start(wait_s: float = 75.0) -> dict:
    current = status()
    if current.get("running"):
        raise RuntimeError(f"runtime already running (pid={current['pid']})")

    STATE_DIR.mkdir(parents=True, exist_ok=True)
    control_logs = results_dir() / "_control"
    control_logs.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    launch_log = control_logs / f"start_{stamp}.log"
    before = set(results_dir().glob("runtime_*"))

    log = open(launch_log, "w")
    proc = subprocess.Popen(
        [sys.executable, "-m", "amsim.runtime.sitl_supervisor"],
        cwd=PROJECT_ROOT,
        env=os.environ.copy(),
        stdin=subprocess.DEVNULL,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    state = {
        "pid": proc.pid,
        "started_at": time.time(),
        "launch_log": str(launch_log),
    }
    STATE_FILE.write_text(json.dumps(state, indent=2))

    deadline = time.monotonic() + wait_s
    runtime_meta = None
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            log.close()
            tail = launch_log.read_text(errors="replace")[-6000:]
            raise RuntimeError(
                f"runtime exited with code {proc.returncode}\n{tail}"
            )
        created = [p for p in results_dir().glob("runtime_*") if p not in before]
        for folder in sorted(created, key=lambda p: p.stat().st_mtime, reverse=True):
            candidate = folder / "runtime.json"
            if candidate.is_file():
                runtime_meta = candidate
                break
        if runtime_meta:
            break
        time.sleep(0.5)

    log.flush()
    state["runtime_meta"] = str(runtime_meta) if runtime_meta else None
    state["ready"] = runtime_meta is not None
    STATE_FILE.write_text(json.dumps(state, indent=2))
    log.close()

    if not runtime_meta:
        raise RuntimeError(
            f"runtime did not become ready within {wait_s:.0f}s; see {launch_log}"
        )
    return state

def stop(timeout_s: float = 15.0) -> dict:
    state = _load_state()
    if not state:
        return {"stopped": True, "reason": "no state file"}

    pid = int(state["pid"])
    if not _alive(pid):
        STATE_FILE.unlink(missing_ok=True)
        return {"stopped": True, "reason": "process already gone", "pid": pid}

    os.killpg(pid, signal.SIGINT)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not _alive(pid):
            STATE_FILE.unlink(missing_ok=True)
            return {"stopped": True, "pid": pid, "signal": "SIGINT"}
        time.sleep(0.25)

    os.killpg(pid, signal.SIGTERM)
    time.sleep(1.0)
    STATE_FILE.unlink(missing_ok=True)
    return {"stopped": not _alive(pid), "pid": pid, "signal": "SIGTERM"}
