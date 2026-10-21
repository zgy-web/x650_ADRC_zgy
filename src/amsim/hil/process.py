from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from ..paths import PROJECT_ROOT, results_dir

STATE = PROJECT_ROOT / ".amsim" / "hil_runtime.json"


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def status() -> dict:
    if not STATE.is_file():
        return {"running": False, "reason": "no state file"}
    try:
        state = json.loads(STATE.read_text())
    except Exception:
        return {"running": False, "reason": "invalid state file"}
    return {**state, "running": _alive(int(state["pid"]))}


def start(
    mavlink: str,
    baud: int = 921600,
    model: str = "x500_underfold",
    xrce_serial: str | None = None,
    xrce_baud: int = 921600,
    wait_s: float = 60.0,
) -> dict:
    from ..modeling.manager import IMPORTED_SOURCES, resolve
    spec = resolve(model)
    if spec.source in IMPORTED_SOURCES:
        raise RuntimeError("Imported URDF models are supported by preview/SITL only; HIL binding has not been validated")
    current = status()
    if current.get("running"):
        raise RuntimeError(f"HIL runtime already running pid={current['pid']}")

    STATE.parent.mkdir(parents=True, exist_ok=True)
    before = set(results_dir().glob("hil_runtime_*"))
    log_dir = results_dir() / "_control"
    log_dir.mkdir(parents=True, exist_ok=True)
    launch_log = log_dir / f"hil_start_{time.strftime('%Y%m%d_%H%M%S')}.log"
    f = open(launch_log, "w")

    cmd = [
        sys.executable,
        "-m",
        "amsim.hil.supervisor",
        "--mavlink",
        mavlink,
        "--baud",
        str(baud),
        "--model",
        model,
    ]
    if xrce_serial:
        cmd.extend([
            "--xrce-serial",
            xrce_serial,
            "--xrce-baud",
            str(xrce_baud),
        ])

    proc = subprocess.Popen(
        cmd,
        cwd=PROJECT_ROOT,
        env=os.environ.copy(),
        stdin=subprocess.DEVNULL,
        stdout=f,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    state = {
        "pid": proc.pid,
        "mavlink": mavlink,
        "baud": baud,
        "model": model,
        "xrce_serial": xrce_serial,
        "launch_log": str(launch_log),
    }
    STATE.write_text(json.dumps(state, indent=2))

    deadline = time.monotonic() + wait_s
    runtime = None
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            f.close()
            tail = launch_log.read_text(errors="replace")[-8000:]
            STATE.unlink(missing_ok=True)
            raise RuntimeError(
                f"HIL runtime exited rc={proc.returncode}\n{tail}"
            )
        created = [
            p for p in results_dir().glob("hil_runtime_*")
            if p not in before and (p / "runtime.json").is_file()
        ]
        if created:
            runtime = max(created, key=lambda p: p.stat().st_mtime) / "runtime.json"
            break
        time.sleep(0.5)

    f.flush()
    f.close()
    if runtime is None:
        raise RuntimeError(
            f"HIL runtime did not become ready within {wait_s:.0f}s; "
            f"see {launch_log}"
        )

    state["runtime_meta"] = str(runtime)
    state["ready"] = True
    STATE.write_text(json.dumps(state, indent=2))
    return state


def stop(timeout_s: float = 10.0) -> dict:
    st = status()
    if not st.get("running"):
        STATE.unlink(missing_ok=True)
        return {"stopped": True, "reason": "not running"}

    pid = int(st["pid"])
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not _alive(pid):
            STATE.unlink(missing_ok=True)
            return {"stopped": True, "pid": pid, "signal": "SIGINT"}
        time.sleep(0.2)
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    time.sleep(0.5)
    STATE.unlink(missing_ok=True)
    return {"stopped": not _alive(pid), "pid": pid, "signal": "SIGTERM"}
