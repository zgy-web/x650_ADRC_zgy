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
        # A stopped supervisor can remain a zombie until its launching Python
        # process reaps it. kill(pid, 0) succeeds for zombies too, which used to
        # turn a successful shutdown into a false timeout/SIGTERM report.
        try:
            stat = Path(f"/proc/{pid}/stat").read_text()
            state = stat[stat.rfind(")") + 2:].split()[0]
            if state == "Z":
                return False
        except (OSError, IndexError):
            # Non-Linux hosts and restricted /proc retain the original probe.
            pass
        return True
    except OSError:
        return False

def status() -> dict:
    state = _load_state()
    if not state:
        return {"running": False, "reason": "no state file"}
    pid = int(state["pid"])
    return {**state, "running": _alive(pid)}


def _cleanup_started_process(proc: subprocess.Popen) -> None:
    """Stop only this launch's child; SIGINT lets its supervisor clean up.

    Do not call stop() here: runtime.json may have been replaced by a different
    launch. The Popen handle is the authority for signalling this process.
    """
    attempts = ((lambda: proc.send_signal(signal.SIGINT), 15.0),
                (proc.terminate, 5.0), (proc.kill, 5.0))
    for send, timeout in attempts:
        if proc.poll() is not None:
            return
        try:
            send()
        except ProcessLookupError:
            pass
        except (OSError, ValueError):
            # e.g. SIGINT is unsupported on a non-POSIX test host. Continue
            # with Popen's own terminate/kill methods, never a state-file PID.
            continue
        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass
    if proc.poll() is None:
        raise RuntimeError(f"could not stop newly launched supervisor pid={proc.pid}")


def _cleanup_note(original: BaseException, message: str) -> None:
    # Preserve the startup exception (including KeyboardInterrupt) even if
    # cleanup also fails. Python 3.10 has no BaseException.add_note().
    if hasattr(original, "add_note"):
        original.add_note(message)
    else:
        print(message, file=sys.stderr)


def start(wait_s: float = 75.0, env_overrides: dict[str, str] | None = None) -> dict:
    current = status()
    if current.get("running"):
        raise RuntimeError(f"runtime already running (pid={current['pid']})")

    STATE_DIR.mkdir(parents=True, exist_ok=True)
    control_logs = results_dir() / "_control"
    control_logs.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    launch_log = control_logs / f"start_{stamp}.log"
    before = set(results_dir().glob("runtime_*"))

    child_env = os.environ.copy()
    if env_overrides:
        child_env.update({str(k): str(v) for k, v in env_overrides.items()})

    proc = None
    state = None
    try:
        with open(launch_log, "w") as log:
            proc = subprocess.Popen(
                [sys.executable, "-m", "amsim.runtime.sitl_supervisor"],
                cwd=PROJECT_ROOT,
                env=child_env,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            state = {
                "pid": proc.pid,
                "started_at": time.time(),
                "launch_log": str(launch_log),
                "model": child_env.get("AMSIM_MODEL_NAME", "x500_underfold"),
            }
            STATE_FILE.write_text(json.dumps(state, indent=2))

            deadline = time.monotonic() + wait_s
            runtime_meta = None
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    log.flush()
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

            if not runtime_meta:
                raise RuntimeError(
                    f"runtime did not become ready within {wait_s:.0f}s; see {launch_log}"
                )
            return state
    except BaseException as exc:
        if proc is not None:
            try:
                _cleanup_started_process(proc)
            except Exception as cleanup_error:
                _cleanup_note(exc, f"Startup cleanup failed: {cleanup_error}")
            # Only remove our own stale state, and only once our child exited.
            # A concurrently replaced or unreadable state file is left alone.
            if state is not None and proc.poll() is not None:
                current = _load_state()
                if (isinstance(current, dict) and current.get("pid") == state["pid"]
                        and current.get("started_at") == state["started_at"]):
                    try:
                        STATE_FILE.unlink(missing_ok=True)
                    except OSError as cleanup_error:
                        _cleanup_note(exc, f"Could not remove failed launch state: {cleanup_error}")
        raise

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
